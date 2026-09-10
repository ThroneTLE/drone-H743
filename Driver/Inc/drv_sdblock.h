#ifndef DRV_SDBLOCK_H
#define DRV_SDBLOCK_H

/*
 * SDMMC 裸块驱动：把 microSD 当成一块"没有文件系统的存储介质"，
 * 向上提供**字节粒度**的读 / 写 / 擦除语义，以便原样承接原来跑在 SPI NOR 上的
 * 飞行日志（App/Src/app_flight_log.c 那 1400 行几何逻辑因此一行都不用改）。
 *
 * 为什么不上 FatFs：
 *   日志是定长扇区环形写入，本来就自带头部与校验；套一层文件系统只会带来
 *   目录项写放大、掉电时 FAT 表损坏、以及为了它再引一套中间件。
 *   日志回传本来就走遥测链路，不依赖把卡插到电脑上读。
 *
 * ============================== 两个必须知道的事实 ==============================
 * 1. **这张卡会被当作裸介质使用**，上面原有的文件系统会被覆盖。
 *    为降低"卡在电脑上完全认不出来"的概率，本驱动从第 DRV_SDBLOCK_BASE_BLOCK 块
 *    起才开始用，永不触碰第 0 块（MBR / 引导扇区）。
 * 2. NOR 的"擦除"在这里退化成"写 0xFF"。SD 没有擦除态的概念，
 *    但上层的日志格式只依赖"擦完读出来是 0xFF"这一条，语义是等价的。
 */

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV_SDBLOCK_OK = 0,
    DRV_SDBLOCK_ERROR,
    DRV_SDBLOCK_TIMEOUT,
    DRV_SDBLOCK_INVALID_ARG,
    DRV_SDBLOCK_NOT_READY
} DRV_SDBLOCK_Status;

#define DRV_SDBLOCK_BLOCK_SIZE   512UL

/* 留出前 1 MB，保证第 0 块（MBR）不被覆盖。 */
#define DRV_SDBLOCK_BASE_BLOCK   2048UL

/*
 * ============================ 这里为什么没有 D-Cache 维护 ============================
 * 因为本驱动只用**阻塞版**的 HAL_SD_ReadBlocks / HAL_SD_WriteBlocks，而 H7 的这两个
 * 函数是 CPU 轮询 SDMMC FIFO 搬数据的，不是 DMA 写内存：
 *
 *     data = SDMMC_ReadFIFO(hsd->Instance);
 *     *tempbuff = (uint8_t)(data & 0xFFU); tempbuff++;
 *     ...                                    —— stm32h7xx_hal_sd.c, HAL_SD_ReadBlocks
 *
 * 既然搬运方是 CPU 自己，读写就都经过 D-Cache，天然一致，不需要任何缓存维护。
 * 反过来，**在这条路径上做维护是有害的**：读完再 invalidate 会把 CPU 刚写进 cache
 * 的脏行直接丢弃，于是读到的是旧内容，而状态码还是 OK——静默返回错数据。
 *
 * 只有改用 HAL_SD_ReadBlocks_DMA / _IT（走 SDMMC 内部 IDMA）时才需要缓存维护，
 * 那时要做的是：发起前 clean+invalidate、等 DMA 完成回调、之后再 invalidate，
 * 并确认缓冲区落在 IDMA 够得到的内存里。改之前先把这段注释一起改掉。
 */
typedef struct {
    SD_HandleTypeDef *hsd;
    uint32_t          timeout_ms;
} DRV_SDBLOCK_Bus;

/*
 * 绑定总线并确认卡在位。没有卡不是错误，是一种正常降级状态：
 * 返回 NOT_READY，由上层决定"不记日志继续飞"还是拒飞（decoupling-spec D1-3）。
 */
DRV_SDBLOCK_Status DRV_SDBLOCK_Init(const DRV_SDBLOCK_Bus *bus);

uint8_t  DRV_SDBLOCK_IsReady(void);
uint64_t DRV_SDBLOCK_GetUsableBytes(void);

DRV_SDBLOCK_Status DRV_SDBLOCK_Read(uint32_t offset, uint8_t *data, uint32_t length);
DRV_SDBLOCK_Status DRV_SDBLOCK_Write(uint32_t offset, const uint8_t *data,
                                     uint32_t length);

/* NOR 语义的擦除：把 [offset, offset+length) 全部写成 0xFF。 */
DRV_SDBLOCK_Status DRV_SDBLOCK_Erase(uint32_t offset, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
