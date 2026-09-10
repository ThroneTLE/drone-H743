#ifndef DRV_INTFLASH_H
#define DRV_INTFLASH_H

/*
 * STM32H743 片内 Flash 驱动（用于参数持久化）。
 *
 * 背景：MicoAir743v2 板上**没有**外部 SPI NOR，原来挂在 SPI1 上的 GD25Q32 这条路断了。
 * 参数改存片内 Flash Bank2 的最后两个扇区，飞行日志改走 SD 裸块（drv_sdblock.h）。
 *
 * ======================= 片内 Flash 与 NOR 的三个硬性差异 =======================
 * 1. **擦除粒度 128 KB**（NOR 是 4 KB）。一个逻辑参数槽独占一个物理扇区，
 *    不能两个槽共用一个扇区，否则擦 A 会把 B 一起抹掉，双槽保护就失效了。
 * 2. **写粒度 32 字节**（256 bit flash word），且**一次擦除后每个 word 只能写一次**。
 *    H7 带 ECC，对同一个 word 二次编程即使只是把 1 变 0 也会报错。
 *    NOR 上常见的"先写记录、再回头把状态位从 WRITING 改成 VALID"在这里非法，
 *    调用者必须保证两次写落在**不同的 word** 上（见 SVC_PARAM_RECORD_STATE_OFFSET）。
 * 3. 读是直接内存访问，但 H7 有 D-Cache：擦/写之后必须失效对应区间的缓存，
 *    否则读回来的是旧内容。失效动作由调用者通过 cache_invalidate 回调注入，
 *    与 drv_gd25q32 的做法保持一致，驱动不反向依赖 BSP。
 */

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV_INTFLASH_OK = 0,
    DRV_INTFLASH_ERROR,
    DRV_INTFLASH_INVALID_ARG,
    DRV_INTFLASH_ALIGN_ERROR,
    DRV_INTFLASH_VERIFY_ERROR
} DRV_INTFLASH_Status;

/* H743 每个扇区 128 KB，Bank2 基址 0x08100000。 */
#define DRV_INTFLASH_SECTOR_SIZE      (128UL * 1024UL)
#define DRV_INTFLASH_WORD_SIZE        32UL
#define DRV_INTFLASH_BANK2_BASE       0x08100000UL

/*
 * 参数区：Bank2 的最后两个扇区（扇区 6 / 7），各 128 KB。
 * 与之配套，STM32H743XX_FLASH.ld 里的 FLASH LENGTH 已从 2048K 收到 1792K，
 * 保证代码段永远不会长进这两个扇区。改动其一必须同时改另一个。
 */
#define DRV_INTFLASH_PARAM_SLOT_A_ADDR   0x081C0000UL
#define DRV_INTFLASH_PARAM_SLOT_B_ADDR   0x081E0000UL
#define DRV_INTFLASH_PARAM_BANK          FLASH_BANK_2
#define DRV_INTFLASH_PARAM_SLOT_A_SECTOR 6U
#define DRV_INTFLASH_PARAM_SLOT_B_SECTOR 7U

typedef struct {
    void (*cache_invalidate)(const void *addr, uint32_t size);
} DRV_INTFLASH_Bus;

void DRV_INTFLASH_SetBus(const DRV_INTFLASH_Bus *bus);

/* 擦除一个 128 KB 扇区。address 必须是扇区起始地址。 */
DRV_INTFLASH_Status DRV_INTFLASH_EraseSector(uint32_t address);

/*
 * 写入。address 必须 32 字节对齐；length 不必是 32 的整数倍，
 * 尾部不足一个 word 的部分由驱动补 0xFF 后整字写入。
 * 调用者必须自己保证同一 word 不会被写第二次。
 */
DRV_INTFLASH_Status DRV_INTFLASH_Write(uint32_t address, const uint8_t *data,
                                       uint32_t length);

DRV_INTFLASH_Status DRV_INTFLASH_Read(uint32_t address, uint8_t *data,
                                      uint32_t length);

/* 地址是否落在本驱动管理的参数区内。 */
uint8_t DRV_INTFLASH_IsParamAddress(uint32_t address);

#ifdef __cplusplus
}
#endif

#endif
