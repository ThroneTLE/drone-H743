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

typedef struct {
    SD_HandleTypeDef *hsd;
    uint32_t          timeout_ms;
    void (*cache_clean)(const void *addr, uint32_t size);
    void (*cache_invalidate)(const void *addr, uint32_t size);
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
