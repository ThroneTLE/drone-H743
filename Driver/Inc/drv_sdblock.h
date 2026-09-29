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
 *
 * 卡在位后先试读一块。4 线读不出时整卡重新初始化成 1 线再试：HAL 的 4→1 线切换
 * 要先在 4 线下读 SCR，4 线不通时切不过去，只能靠 CMD0 复位回 1 线。
 */
DRV_SDBLOCK_Status DRV_SDBLOCK_Init(const DRV_SDBLOCK_Bus *bus);

uint8_t  DRV_SDBLOCK_IsReady(void);
uint64_t DRV_SDBLOCK_GetUsableBytes(void);

/* 试读结果：0 未试，1 成功，2 失败。 */
#define DRV_SDBLOCK_PROBE_NONE 0U
#define DRV_SDBLOCK_PROBE_OK   1U
#define DRV_SDBLOCK_PROBE_FAIL 2U

/* 运行期首次失败发生在哪一步。 */
#define DRV_SDBLOCK_OP_NONE  0U
#define DRV_SDBLOCK_OP_WAIT  1U
#define DRV_SDBLOCK_OP_READ  2U
#define DRV_SDBLOCK_OP_WRITE 3U

/* 只读诊断快照，给 STATUS? 分清"卡没认到 / 4 线读不通 / 运行中坏掉"。error 都是 HAL ErrorCode。 */
typedef struct {
    uint8_t  init_status;    /* 最近一次 Init 的 DRV_SDBLOCK_Status */
    uint8_t  bus_bits;       /* 当前数据线宽度 1 或 4；0 表示没走到 */
    uint8_t  probe_first;    /* 按配置线宽试读 */
    uint8_t  probe_1bit;     /* 首次失败后重新初始化成 1 线再试读 */
    uint32_t first_error;
    uint32_t first_ms;
    uint32_t retry_error;    /* 1 线重试：重新初始化或试读的错误码 */
    uint32_t retry_ms;
    uint32_t card_type;      /* HAL CardType：0 SDSC，1 SDHC/SDXC */
    uint32_t card_blocks;
    uint8_t  fail_op;        /* Init 成功后的首次失败，DRV_SDBLOCK_OP_* */
    uint32_t fail_error;
} DRV_SDBLOCK_Diag;

void DRV_SDBLOCK_GetDiag(DRV_SDBLOCK_Diag *out);

DRV_SDBLOCK_Status DRV_SDBLOCK_Read(uint32_t offset, uint8_t *data, uint32_t length);
DRV_SDBLOCK_Status DRV_SDBLOCK_Write(uint32_t offset, const uint8_t *data,
                                     uint32_t length);

/* NOR 语义的擦除：把 [offset, offset+length) 全部写成 0xFF。 */
DRV_SDBLOCK_Status DRV_SDBLOCK_Erase(uint32_t offset, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
