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
 * 等卡回到 TRANSFER 态。有明确的重试上限，超时就返回 TIMEOUT，
 * 不做无界死等——一张接触不良的卡不该把后台任务永久钉住（D1-3）。
 */
static DRV_SDBLOCK_Status sdblock_wait_ready(void)
{
    uint32_t retry;

    for (retry = 0U; retry < SDBLOCK_CARD_READY_RETRY; retry++) {
        if (HAL_SD_GetCardState(sdblock_bus.hsd) == HAL_SD_CARD_TRANSFER) {
            return DRV_SDBLOCK_OK;
        }
    }

    return DRV_SDBLOCK_TIMEOUT;
}

DRV_SDBLOCK_Status DRV_SDBLOCK_Init(const DRV_SDBLOCK_Bus *bus)
{
    HAL_SD_CardInfoTypeDef info;

    sdblock_ready = 0U;
    sdblock_usable_bytes = 0U;

    if ((bus == NULL) || (bus->hsd == NULL)) {
        return DRV_SDBLOCK_INVALID_ARG;
    }

    sdblock_bus = *bus;

    if (sdblock_wait_ready() != DRV_SDBLOCK_OK) {
        return DRV_SDBLOCK_NOT_READY;
    }

    memset(&info, 0, sizeof(info));
    if (HAL_SD_GetCardInfo(sdblock_bus.hsd, &info) != HAL_OK) {
        return DRV_SDBLOCK_NOT_READY;
    }

    if (info.LogBlockNbr <= DRV_SDBLOCK_BASE_BLOCK) {
        return DRV_SDBLOCK_NOT_READY;
    }

    sdblock_usable_bytes = (uint64_t)(info.LogBlockNbr - DRV_SDBLOCK_BASE_BLOCK) *
                           (uint64_t)DRV_SDBLOCK_BLOCK_SIZE;
    sdblock_ready = 1U;

    return DRV_SDBLOCK_OK;
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

    if (status != DRV_SDBLOCK_OK) { return status; }

    if (HAL_SD_ReadBlocks(sdblock_bus.hsd, sdblock_scratch,
                          DRV_SDBLOCK_BASE_BLOCK + block, 1U,
                          sdblock_timeout()) != HAL_OK) {
        return DRV_SDBLOCK_ERROR;
    }

    return DRV_SDBLOCK_OK;
}

static DRV_SDBLOCK_Status sdblock_write_block(uint32_t block)
{
    DRV_SDBLOCK_Status status = sdblock_wait_ready();

    if (status != DRV_SDBLOCK_OK) { return status; }

    if (HAL_SD_WriteBlocks(sdblock_bus.hsd, sdblock_scratch,
                           DRV_SDBLOCK_BASE_BLOCK + block, 1U,
                           sdblock_timeout()) != HAL_OK) {
        return DRV_SDBLOCK_ERROR;
    }

    return sdblock_wait_ready();
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
