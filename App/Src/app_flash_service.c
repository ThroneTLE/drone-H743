/*
 * 持久化门面 + 地址路由。
 *
 * ================================ 路由为什么这样切 ================================
 * 上层（svc_param / app_flight_log）是按 4 MB 平坦 NOR 地址空间写的：
 * 4 KB 扇区擦除、32 KB 块擦除、字节寻址读写。MicoAir743v2 上这块 NOR 不存在，
 * 但那套几何本身没有问题，问题只在"背后是什么介质"。所以这里保持地址空间不变，
 * 只在**一个地方**决定每笔访问落到哪块介质上：
 *
 *   [PARAM_REGION_START, SIZE) → 片内 Flash。参数很小、必须掉电可靠、
 *                                而且不能依赖"卡插没插"，所以放片内。
 *   其余                        → SD 裸块。日志量大、可以缺失、没卡就降级不记。
 *
 * 参数区有**四个**逻辑槽，一一对应四个物理扇区：svc_param 的标定 blob 占 A/B，
 * app_control_config_store 的配置记录（含机体模型）占另一对 A/B。
 * 绝不能让两个槽共用一个物理扇区——片内 Flash 的擦除粒度是 128 KB，
 * 共用的话擦 A 会把 B 一起抹掉，双槽掉电保护就名存实亡了。
 *
 * NOR 专有的接口（JEDEC / 状态寄存器 / 写保护）继续打给真正的 SPI NOR 驱动。
 * 板上没有这颗芯片时它们如实失败，诊断页显示"没有外部 Flash"，这是对的。
 */

#include "app_flash_service.h"

#include "bsp_board.h"
#include "bsp_flash_bus.h"
#include "drv_intflash.h"
#include "drv_sdblock.h"

#include <stdio.h>
#include <string.h>

static DRV_GD25Q32_Device flash_device;
static uint8_t flash_bound;

/* ------------------------------------------------------------------ 路由 */

APP_FlashService_Backend APP_FlashService_BackendFor(uint32_t address)
{
    if (address >= APP_FLASH_SERVICE_SIZE_BYTES) {
        return APP_FLASH_BACKEND_NONE;
    }
    if (address >= APP_FLASH_SERVICE_PARAM_REGION_START) {
        return APP_FLASH_BACKEND_INTERNAL;
    }
    return APP_FLASH_BACKEND_SDBLOCK;
}

const char *APP_FlashService_BackendName(APP_FlashService_Backend backend)
{
    switch (backend) {
    case APP_FLASH_BACKEND_INTERNAL: return "INTFLASH";
    case APP_FLASH_BACKEND_SDBLOCK:  return "SDBLOCK";
    case APP_FLASH_BACKEND_NONE:
    default:                         return "NONE";
    }
}

uint8_t APP_FlashService_IsLogStorageReady(void)
{
    return DRV_SDBLOCK_IsReady();
}

void APP_FlashService_FormatSdDiag(char *out, uint32_t size)
{
    DRV_SDBLOCK_Diag d;

    if ((out == NULL) || (size == 0U)) {
        return;
    }
    DRV_SDBLOCK_GetDiag(&d);
    (void)snprintf(out, size,
                   "HW SD present=%u init_err=0x%08lX ready=%u init=%u bits=%u type=%lu "
                   "blocks=%lu probe=%u first_err=0x%08lX first_ms=%lu retry1=%u "
                   "retry_err=0x%08lX retry_ms=%lu fail_op=%u fail_err=0x%08lX",
                   (unsigned int)BSP_Board_SdCardPresent(),
                   (unsigned long)BSP_Board_SdInitError(),
                   (unsigned int)DRV_SDBLOCK_IsReady(), (unsigned int)d.init_status,
                   (unsigned int)d.bus_bits, (unsigned long)d.card_type,
                   (unsigned long)d.card_blocks, (unsigned int)d.probe_first,
                   (unsigned long)d.first_error, (unsigned long)d.first_ms,
                   (unsigned int)d.probe_1bit, (unsigned long)d.retry_error,
                   (unsigned long)d.retry_ms, (unsigned int)d.fail_op,
                   (unsigned long)d.fail_error);
}

/*
 * 逻辑地址 → 片内 Flash 物理地址。
 *
 * 四个逻辑槽按序落到四个物理扇区上：逻辑槽 i（4 KB 窗口）→ 物理扇区
 * FIRST_SECTOR+i（128 KB）的开头。逻辑窗口比物理扇区小得多，剩下的 124 KB
 * 空着——这不是浪费，是擦除粒度决定的：能独立擦除的最小单位就是 128 KB，
 * 想让一个槽被擦时不碰到另一个槽，就只能一个槽占一个扇区。
 */
_Static_assert(APP_FLASH_SERVICE_PARAM_SLOT_COUNT ==
                   DRV_INTFLASH_PARAM_SECTOR_COUNT,
               "logical param slots and physical intflash sectors must be 1:1");
_Static_assert(APP_FLASH_SERVICE_SECTOR_SIZE <= DRV_INTFLASH_SECTOR_SIZE,
               "a logical slot must fit inside one physical sector");

static uint8_t flash_param_physical(uint32_t address, uint32_t length,
                                    uint32_t *physical)
{
    uint32_t index;
    uint32_t offset;
    uint32_t in_slot;

    if ((length == 0U) || (address < APP_FLASH_SERVICE_PARAM_REGION_START)) {
        return 0U;
    }
    offset = address - APP_FLASH_SERVICE_PARAM_REGION_START;
    index = offset / APP_FLASH_SERVICE_SECTOR_SIZE;
    if (index >= APP_FLASH_SERVICE_PARAM_SLOT_COUNT) {
        return 0U;
    }

    /*
     * 整笔事务必须落在**同一个逻辑槽**内。跨槽的逻辑地址在物理上并不连续
     * （逻辑槽 4 KB、物理扇区 128 KB），放行的话后半段会悄悄写进本扇区那 124 KB
     * 空白里，而不是下一个槽——地址对不上，回读还"成功"。宁可拒绝。
     */
    in_slot = offset % APP_FLASH_SERVICE_SECTOR_SIZE;
    if (length > (APP_FLASH_SERVICE_SECTOR_SIZE - in_slot)) {
        return 0U;
    }

    *physical = DRV_INTFLASH_PARAM_SECTOR_ADDR(index) + in_slot;
    return 1U;
}

static APP_FlashService_Status flash_from_intflash(DRV_INTFLASH_Status status)
{
    switch (status) {
    case DRV_INTFLASH_OK:          return DRV_GD25Q32_OK;
    case DRV_INTFLASH_INVALID_ARG:
    case DRV_INTFLASH_ALIGN_ERROR: return DRV_GD25Q32_INVALID_ARG;
    case DRV_INTFLASH_VERIFY_ERROR:
    case DRV_INTFLASH_ERROR:
    default:                       return DRV_GD25Q32_ERROR;
    }
}

static APP_FlashService_Status flash_from_sdblock(DRV_SDBLOCK_Status status)
{
    switch (status) {
    case DRV_SDBLOCK_OK:          return DRV_GD25Q32_OK;
    case DRV_SDBLOCK_TIMEOUT:     return DRV_GD25Q32_TIMEOUT;
    case DRV_SDBLOCK_INVALID_ARG: return DRV_GD25Q32_INVALID_ARG;
    case DRV_SDBLOCK_NOT_READY:   return DRV_GD25Q32_BAD_ID;
    case DRV_SDBLOCK_ERROR:
    default:                      return DRV_GD25Q32_ERROR;
    }
}

#define APP_FLASH_SERVICE_LOCK_TIMEOUT_MS 10000U

static void flash_service_bind(void)
{
    if (flash_bound != 0U) {
        return;
    }

    memset(&flash_device, 0, sizeof(flash_device));
    flash_device.bus = *BSP_FlashBus_GetBus();
    BSP_FlashBus_RegisterDmaDevice(&flash_device);
    flash_bound = 1U;
}

static APP_FlashService_Status flash_service_lock(void)
{
    return BSP_FlashBus_Acquire(APP_FLASH_SERVICE_LOCK_TIMEOUT_MS);
}

static void flash_service_unlock(void)
{
    BSP_FlashBus_Release();
}

APP_FlashService_Status APP_FlashService_Init(void)
{
    APP_FlashService_Status status;
    DRV_GD25Q32_Bus bus;

    /*
     * 三块介质各自初始化，**互不阻断**：
     *   片内 Flash —— 参数存这里，必须成功；
     *   SD 卡      —— 没插卡是正常降级（不记日志照飞），不算初始化失败；
     *   外部 NOR   —— MicoAir 板上没有这颗，探测失败是预期结果，
     *                只影响诊断页的显示，不影响参数与日志。
     * 任何一路失败都不能把整机卡在初始化里（decoupling-spec D1-3）。
     */
    status = flash_service_lock();
    if (status != DRV_GD25Q32_OK) {
        return status;
    }

    /* 三个后端的绑定与探测都在锁内完成，免得别的任务撞见半绑定状态。 */
    DRV_INTFLASH_SetBus(BSP_Board_GetIntFlashBus());
    (void)DRV_SDBLOCK_Init(BSP_Board_GetSdBlockBus());

    flash_service_bind();
    bus = flash_device.bus;

    status = DRV_GD25Q32_Init(&flash_device, &bus);
    BSP_FlashBus_RegisterDmaDevice(&flash_device);
    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_ProbeJedecId(APP_FlashService_JedecId *jedec_id)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status != DRV_GD25Q32_OK) {
        return status;
    }

    status = DRV_GD25Q32_ReleaseFromPowerDown(&flash_device);
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadJedecId(&flash_device, jedec_id);
    }
    if ((status == DRV_GD25Q32_OK) &&
        ((jedec_id->manufacturer_id != DRV_GD25Q32_JEDEC_MANUFACTURER_ID) ||
         (jedec_id->memory_type != DRV_GD25Q32_JEDEC_MEMORY_TYPE) ||
         (jedec_id->capacity_id != DRV_GD25Q32_JEDEC_CAPACITY_ID))) {
        status = DRV_GD25Q32_BAD_ID;
    }

    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_ReadStatus1(uint8_t *status1)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadStatus1(&flash_device, status1);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_ReadStatus2(uint8_t *status2)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadStatus2(&flash_device, status2);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_ReadStatus3(uint8_t *status3)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ReadStatus3(&flash_device, status3);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_WriteEnableProbe(uint8_t *status_before,
                                                          uint8_t *status_after)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_WriteEnableProbe(&flash_device,
                                              status_before,
                                              status_after);
        flash_service_unlock();
    }
    return status;
}

APP_FlashService_Status APP_FlashService_ClearProtection(uint8_t *status1_before,
                                                         uint8_t *status2_before,
                                                         uint8_t *status1_after,
                                                         uint8_t *status2_after)
{
    APP_FlashService_Status status;

    flash_service_bind();
    status = flash_service_lock();
    if (status == DRV_GD25Q32_OK) {
        status = DRV_GD25Q32_ClearProtection(&flash_device,
                                             status1_before,
                                             status2_before,
                                             status1_after,
                                             status2_after);
        flash_service_unlock();
    }
    return status;
}

/* ------------------------------------------------ 数据通路：核心（不加锁） */

/*
 * 下面这三个 *_unlocked 函数**必须在持锁状态下调用**。
 *
 * 拆出来的唯一原因是 flashBusMutex 不是递归锁（Core/Src/freertos.c 里创建时只给了
 * .name，没有 osMutexRecursive）。PageProgram 要复用 WriteData 的逻辑、ReadDataFast
 * 要复用 ReadData 的逻辑，如果它们互相调用公开入口，同一个任务就会二次获取同一把锁，
 * 直接卡死 10 秒然后返回 TIMEOUT。所以公开入口只负责"加锁—调核心—解锁"，
 * 复用一律走核心。
 */
static APP_FlashService_Status flash_read_unlocked(uint32_t address, uint8_t *data,
                                                   uint32_t length)
{
    uint32_t physical = 0U;

    if (APP_FlashService_BackendFor(address) == APP_FLASH_BACKEND_INTERNAL) {
        if (flash_param_physical(address, length, &physical) == 0U) {
            return DRV_GD25Q32_INVALID_ARG;
        }
        return flash_from_intflash(DRV_INTFLASH_Read(physical, data, length));
    }

    return flash_from_sdblock(DRV_SDBLOCK_Read(address, data, length));
}

static APP_FlashService_Status flash_write_unlocked(uint32_t address,
                                                    const uint8_t *data,
                                                    uint32_t length)
{
    uint32_t physical = 0U;

    if (APP_FlashService_BackendFor(address) == APP_FLASH_BACKEND_INTERNAL) {
        if (flash_param_physical(address, length, &physical) == 0U) {
            return DRV_GD25Q32_INVALID_ARG;
        }
        return flash_from_intflash(DRV_INTFLASH_Write(physical, data, length));
    }

    return flash_from_sdblock(DRV_SDBLOCK_Write(address, data, length));
}

static APP_FlashService_Status flash_erase_unlocked(uint32_t address, uint32_t length)
{
    uint32_t physical = 0U;

    if (APP_FlashService_BackendFor(address) == APP_FLASH_BACKEND_INTERNAL) {
        /* 参数区只接受扇区擦除：块擦除会跨过槽边界，语义无法保证。 */
        if (length != APP_FLASH_SERVICE_SECTOR_SIZE) {
            return DRV_GD25Q32_INVALID_ARG;
        }
        if (flash_param_physical(address, length, &physical) == 0U) {
            return DRV_GD25Q32_INVALID_ARG;
        }
        /*
         * 片内擦除粒度是 128 KB，这里一次擦掉整个物理扇区。
         * 因为一个逻辑参数槽独占一个物理扇区，"擦一个逻辑扇区"与
         * "擦整个物理扇区"作用范围相同，另一个槽不受影响。
         */
        return flash_from_intflash(DRV_INTFLASH_EraseSector(physical));
    }

    return flash_from_sdblock(DRV_SDBLOCK_Erase(address, length));
}

/* ------------------------------------------------ 数据通路：公开入口（加锁） */

/*
 * ================================ 锁保护的是什么 ================================
 * 换后端之前，这几个入口打给 SPI NOR，每次都持 flashBusMutex。换成"片内 Flash +
 * SD 裸块"之后锁一度被漏掉了，而 SD 后端比 NOR 更需要它：drv_sdblock 内部有一个
 * **全局共享的 512 字节块缓冲**，不足整块的写是"读回整块 → 改中间几字节 → 写回"。
 *
 * 锁的粒度必须是**整笔事务**，不能只锁单次 HAL 调用。HAL 自己的 busy 状态挡不住
 * 这个窗口：后台日志任务读完块 A、HAL 已经返回、还没来得及 memcpy 出去，通信任务
 * 的 FLASH VERIFY 抢进来把同一个缓冲读成块 B——两边都拿到别人的数据，而且状态码
 * 全是 OK。日志写的读改写序列被这样插一脚，则是直接把邻居字节写坏。
 */
APP_FlashService_Status APP_FlashService_ReadData(uint32_t address, uint8_t *data, uint32_t length)
{
    APP_FlashService_Status status;

    if ((data == NULL) || (length == 0U)) { return DRV_GD25Q32_INVALID_ARG; }

    status = flash_service_lock();
    if (status != DRV_GD25Q32_OK) { return status; }

    status = flash_read_unlocked(address, data, length);
    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_ReadDataFast(uint32_t address, uint8_t *data, uint32_t length)
{
    /*
     * NOR 上 Fast Read 是另一条 DMA 指令；片内 Flash 与 SD 没有"快读"这个概念，
     * 直接复用普通读，语义完全一致，调用者不必区分。
     */
    return APP_FlashService_ReadData(address, data, length);
}

APP_FlashService_Status APP_FlashService_EraseSector(uint32_t address)
{
    APP_FlashService_Status status = flash_service_lock();

    if (status != DRV_GD25Q32_OK) { return status; }

    status = flash_erase_unlocked(address, APP_FLASH_SERVICE_SECTOR_SIZE);
    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_EraseBlock32K(uint32_t address)
{
    APP_FlashService_Status status = flash_service_lock();

    if (status != DRV_GD25Q32_OK) { return status; }

    status = flash_erase_unlocked(address, APP_FLASH_SERVICE_BLOCK32K_SIZE);
    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_EraseBlock64K(uint32_t address)
{
    APP_FlashService_Status status = flash_service_lock();

    if (status != DRV_GD25Q32_OK) { return status; }

    status = flash_erase_unlocked(address, APP_FLASH_SERVICE_BLOCK64K_SIZE);
    flash_service_unlock();
    return status;
}

APP_FlashService_Status APP_FlashService_PageProgram(uint32_t address,
                                                     const uint8_t *data,
                                                     uint16_t length)
{
    /* NOR 的页边界限制在这两个后端上都不存在，直接走通用写。 */
    return APP_FlashService_WriteData(address, data, (uint32_t)length);
}

APP_FlashService_Status APP_FlashService_WriteData(uint32_t address,
                                                   const uint8_t *data,
                                                   uint32_t length)
{
    APP_FlashService_Status status;

    if ((data == NULL) || (length == 0U)) { return DRV_GD25Q32_INVALID_ARG; }

    status = flash_service_lock();
    if (status != DRV_GD25Q32_OK) { return status; }

    status = flash_write_unlocked(address, data, length);
    flash_service_unlock();
    return status;
}

void APP_FlashService_Invalidate(void)
{
    memset(&flash_device, 0, sizeof(flash_device));
    BSP_FlashBus_InvalidateBinding();
    flash_bound = 0U;
}
