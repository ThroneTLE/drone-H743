/*
 * SDMMC 裸块驱动实现。设计取舍见 drv_sdblock.h。
 *
 * 实现上的两个要点：
 *   - 只用阻塞版 HAL_SD_ReadBlocks/WriteBlocks，它们是 CPU 轮询 FIFO 搬运，
 *     **不做也不能做 D-Cache 维护**。理由与改成 DMA 时该怎么做，见 drv_sdblock.h。
 *   - 所有搬运统一过一个静态块缓冲，不直接用调用者的指针。
 *     调用者的缓冲未必对齐，交给它自己保证太脆弱。
 *
 * 并发：本驱动**不自带锁**，那个静态块缓冲是全局共享的。串行化由唯一的调用者
 * App/Src/app_flash_service.c 负责（它对整笔读改写事务加锁，而不只是单次 HAL 调用）。
 */

#include "drv_sdblock.h"

#include <string.h>

#define SDBLOCK_DEFAULT_TIMEOUT_MS 1000U
#define SDBLOCK_CARD_READY_RETRY   1000U

static DRV_SDBLOCK_Bus sdblock_bus;
static uint8_t sdblock_ready;
static uint64_t sdblock_usable_bytes;
static DRV_SDBLOCK_Diag sdblock_diag;

/*
 * 放在 .dma_buffer（RAM_D2）并按 32 字节对齐，**不是**当前实现的要求——
 * CPU 轮询搬运对位置和对齐都没有要求。这么放是给日后可能改用 SDMMC IDMA 留余地：
 * 那时缓冲区必须对齐到 cache line 且落在 IDMA 够得到的内存里，现在先满足着，
 * 免得改传输方式时还要连带挪内存。
 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t sdblock_scratch[DRV_SDBLOCK_BLOCK_SIZE];

static uint32_t sdblock_timeout(void)
{
    return (sdblock_bus.timeout_ms != 0U) ? sdblock_bus.timeout_ms
                                          : SDBLOCK_DEFAULT_TIMEOUT_MS;
}

/*
 * 等卡回到 TRANSFER 态。重试次数和总时长（同一次传输超时）双上限，谁先到算谁，
 * 不做无界死等——一张接触不良的卡不该把后台任务永久钉住（D1-3）。
 */
static DRV_SDBLOCK_Status sdblock_wait_ready(void)
{
    const uint32_t start_ms = HAL_GetTick();
    uint32_t retry;

    for (retry = 0U; retry < SDBLOCK_CARD_READY_RETRY; retry++) {
        if (HAL_SD_GetCardState(sdblock_bus.hsd) == HAL_SD_CARD_TRANSFER) {
            return DRV_SDBLOCK_OK;
        }
        if ((uint32_t)(HAL_GetTick() - start_ms) >= sdblock_timeout()) {
            break;
        }
    }

    return DRV_SDBLOCK_TIMEOUT;
}

/*
 * 一次传输失败就把卡当作不可用，直到重新 Init。
 * 2026-09-25 实机：一张读不出的卡每次读都要等满超时，飞行日志开机扫描约 1000 个扇区，
 * 后台任务被钉住约 17 分钟，期间所有参数都写不进 Flash。宁可这次开机不记日志。
 */
static DRV_SDBLOCK_Status sdblock_mark_failed(DRV_SDBLOCK_Status status, uint8_t op)
{
    if (sdblock_ready != 0U) {
        sdblock_diag.fail_op = op;
        sdblock_diag.fail_error = sdblock_bus.hsd->ErrorCode;
        /* 超时返回时数据状态机还在等起始位，先停掉，别让它挡住下一条命令。 */
        (void)HAL_SD_Abort(sdblock_bus.hsd);
    }
    sdblock_ready = 0U;
    return status;
}

/* 读第一块可用块，只验证数据线通不通，不改卡上内容。 */
static uint8_t sdblock_probe(uint32_t *error, uint32_t *elapsed_ms)
{
    const uint32_t start_ms = HAL_GetTick();
    uint8_t ok = (sdblock_wait_ready() == DRV_SDBLOCK_OK) &&
                 (HAL_SD_ReadBlocks(sdblock_bus.hsd, sdblock_scratch,
                                    DRV_SDBLOCK_BASE_BLOCK, 1U,
                                    sdblock_timeout()) == HAL_OK);

    *elapsed_ms = HAL_GetTick() - start_ms;
    if (ok != 0U) {
        *error = 0U;
        return DRV_SDBLOCK_PROBE_OK;
    }
    *error = sdblock_bus.hsd->ErrorCode;
    (void)HAL_SD_Abort(sdblock_bus.hsd);
    return DRV_SDBLOCK_PROBE_FAIL;
}

/* 整卡重新初始化成 1 线：HAL_SD_Init 里的 CMD0 会把卡复位回 1 线。 */
static uint8_t sdblock_retry_1bit(void)
{
    SD_HandleTypeDef *hsd = sdblock_bus.hsd;
    const uint32_t start_ms = HAL_GetTick();
    uint8_t probe;

    (void)HAL_SD_DeInit(hsd);
    hsd->Init.BusWide = SDMMC_BUS_WIDE_1B;
    if (HAL_SD_Init(hsd) != HAL_OK) {
        sdblock_diag.retry_error = hsd->ErrorCode;
        sdblock_diag.retry_ms = HAL_GetTick() - start_ms;
        return DRV_SDBLOCK_PROBE_FAIL;
    }
    sdblock_diag.bus_bits = 1U;
    probe = sdblock_probe(&sdblock_diag.retry_error, &sdblock_diag.retry_ms);
    sdblock_diag.retry_ms = HAL_GetTick() - start_ms;
    return probe;
}

static DRV_SDBLOCK_Status sdblock_init(void)
{
    HAL_SD_CardInfoTypeDef info;
    uint8_t probe;

    if (sdblock_wait_ready() != DRV_SDBLOCK_OK) {
        return DRV_SDBLOCK_NOT_READY;
    }

    memset(&info, 0, sizeof(info));
    if (HAL_SD_GetCardInfo(sdblock_bus.hsd, &info) != HAL_OK) {
        return DRV_SDBLOCK_NOT_READY;
    }
    sdblock_diag.card_type = info.CardType;
    sdblock_diag.card_blocks = info.LogBlockNbr;

    if (info.LogBlockNbr <= DRV_SDBLOCK_BASE_BLOCK) {
        return DRV_SDBLOCK_NOT_READY;
    }

    sdblock_diag.bus_bits =
        (sdblock_bus.hsd->Init.BusWide == SDMMC_BUS_WIDE_1B) ? 1U : 4U;
    probe = sdblock_probe(&sdblock_diag.first_error, &sdblock_diag.first_ms);
    sdblock_diag.probe_first = probe;
    if ((probe != DRV_SDBLOCK_PROBE_OK) && (sdblock_diag.bus_bits != 1U)) {
        probe = sdblock_retry_1bit();
        sdblock_diag.probe_1bit = probe;
    }
    if (probe != DRV_SDBLOCK_PROBE_OK) {
        return DRV_SDBLOCK_ERROR;
    }

    sdblock_usable_bytes = (uint64_t)(info.LogBlockNbr - DRV_SDBLOCK_BASE_BLOCK) *
                           (uint64_t)DRV_SDBLOCK_BLOCK_SIZE;
    sdblock_ready = 1U;

    return DRV_SDBLOCK_OK;
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Init(const DRV_SDBLOCK_Bus *bus)
{
    DRV_SDBLOCK_Status status;

    sdblock_ready = 0U;
    sdblock_usable_bytes = 0U;
    memset(&sdblock_diag, 0, sizeof(sdblock_diag));

    if ((bus == NULL) || (bus->hsd == NULL)) {
        status = DRV_SDBLOCK_INVALID_ARG;
    } else {
        sdblock_bus = *bus;
        status = sdblock_init();
    }
    sdblock_diag.init_status = (uint8_t)status;
    return status;
}

void DRV_SDBLOCK_GetDiag(DRV_SDBLOCK_Diag *out)
{
    if (out != NULL) {
        *out = sdblock_diag;
    }
}

uint8_t DRV_SDBLOCK_IsReady(void)
{
    return sdblock_ready;
}

uint64_t DRV_SDBLOCK_GetUsableBytes(void)
{
    return sdblock_usable_bytes;
}

static DRV_SDBLOCK_Status sdblock_read_block(uint32_t block)
{
    DRV_SDBLOCK_Status status = sdblock_wait_ready();

    if (status != DRV_SDBLOCK_OK) { return sdblock_mark_failed(status, DRV_SDBLOCK_OP_WAIT); }

    if (HAL_SD_ReadBlocks(sdblock_bus.hsd, sdblock_scratch,
                          DRV_SDBLOCK_BASE_BLOCK + block, 1U,
                          sdblock_timeout()) != HAL_OK) {
        return sdblock_mark_failed(DRV_SDBLOCK_ERROR, DRV_SDBLOCK_OP_READ);
    }

    return DRV_SDBLOCK_OK;
}

static DRV_SDBLOCK_Status sdblock_write_block(uint32_t block)
{
    DRV_SDBLOCK_Status status = sdblock_wait_ready();

    if (status != DRV_SDBLOCK_OK) { return sdblock_mark_failed(status, DRV_SDBLOCK_OP_WAIT); }

    if (HAL_SD_WriteBlocks(sdblock_bus.hsd, sdblock_scratch,
                           DRV_SDBLOCK_BASE_BLOCK + block, 1U,
                           sdblock_timeout()) != HAL_OK) {
        return sdblock_mark_failed(DRV_SDBLOCK_ERROR, DRV_SDBLOCK_OP_WRITE);
    }

    status = sdblock_wait_ready();
    return (status == DRV_SDBLOCK_OK) ? status
                                      : sdblock_mark_failed(status, DRV_SDBLOCK_OP_WAIT);
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Read(uint32_t offset, uint8_t *data, uint32_t length)
{
    uint32_t done = 0U;

    if ((data == NULL) || (length == 0U)) { return DRV_SDBLOCK_INVALID_ARG; }
    if (sdblock_ready == 0U) { return DRV_SDBLOCK_NOT_READY; }

    while (done < length) {
        uint32_t abs = offset + done;
        uint32_t block = abs / DRV_SDBLOCK_BLOCK_SIZE;
        uint32_t in_block = abs % DRV_SDBLOCK_BLOCK_SIZE;
        uint32_t chunk = DRV_SDBLOCK_BLOCK_SIZE - in_block;
        DRV_SDBLOCK_Status status;

        if (chunk > (length - done)) { chunk = length - done; }

        status = sdblock_read_block(block);
        if (status != DRV_SDBLOCK_OK) { return status; }

        memcpy(&data[done], &sdblock_scratch[in_block], chunk);
        done += chunk;
    }

    return DRV_SDBLOCK_OK;
}

/*
 * 写入是读-改-写：不足整块的部分必须先把原块读回来，否则会把同块内的邻居抹成 0。
 * 整块覆盖时跳过回读，省一次往返。
 */
static DRV_SDBLOCK_Status sdblock_write_span(uint32_t offset, const uint8_t *data,
                                             uint8_t fill_value, uint32_t length)
{
    uint32_t done = 0U;

    if (sdblock_ready == 0U) { return DRV_SDBLOCK_NOT_READY; }

    while (done < length) {
        uint32_t abs = offset + done;
        uint32_t block = abs / DRV_SDBLOCK_BLOCK_SIZE;
        uint32_t in_block = abs % DRV_SDBLOCK_BLOCK_SIZE;
        uint32_t chunk = DRV_SDBLOCK_BLOCK_SIZE - in_block;
        DRV_SDBLOCK_Status status;

        if (chunk > (length - done)) { chunk = length - done; }

        if (chunk == DRV_SDBLOCK_BLOCK_SIZE) {
            memset(sdblock_scratch, fill_value, sizeof(sdblock_scratch));
        } else {
            status = sdblock_read_block(block);
            if (status != DRV_SDBLOCK_OK) { return status; }
        }

        if (data != NULL) {
            memcpy(&sdblock_scratch[in_block], &data[done], chunk);
        } else {
            memset(&sdblock_scratch[in_block], fill_value, chunk);
        }

        status = sdblock_write_block(block);
        if (status != DRV_SDBLOCK_OK) { return status; }

        done += chunk;
    }

    return DRV_SDBLOCK_OK;
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Write(uint32_t offset, const uint8_t *data,
                                     uint32_t length)
{
    if ((data == NULL) || (length == 0U)) { return DRV_SDBLOCK_INVALID_ARG; }
    return sdblock_write_span(offset, data, 0xFFU, length);
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Erase(uint32_t offset, uint32_t length)
{
    if (length == 0U) { return DRV_SDBLOCK_INVALID_ARG; }
    return sdblock_write_span(offset, NULL, 0xFFU, length);
}
