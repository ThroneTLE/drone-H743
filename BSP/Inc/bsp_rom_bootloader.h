#ifndef BSP_ROM_BOOTLOADER_H
#define BSP_ROM_BOOTLOADER_H

#include <stdint.h>

/*
 * 跳进片内 ROM bootloader（USB DFU）所需的**机制**。
 *
 * 分工：本文件只管"怎么做"——备份寄存器怎么开、外设怎么静默、栈指针怎么换；
 * "什么时候可以跳"（有没有解锁、电调是不是怠速、快照新不新鲜）全部留在
 * App/Src/app_boot.c，那部分是纯逻辑，有宿主侧单测。
 *
 * 这样分的理由很直接：跳 ROM 这件事每一步都踩在具体芯片上（PWR 的 DBP 位、
 * RTC 备份寄存器、SCB 的 VTOR 与 cache、那段换 MSP 的裸汇编），换一族 MCU
 * 全都要重写；而"什么条件下允许跳"在任何飞控上都是同一套判据。
 *
 * === 契约 ===
 *
 * - `WriteRequestMagic` / `TakeRequestMagic` 用**掉电不丢、软复位仍在**的备份
 *   寄存器传递请求。调用方给魔数，本模块不解释它的含义。
 * - `Jump` **不返回**，且只能从 main 最早的钩子里调用：那时复位刚把栈指针选成
 *   MSP（不是 RTOS 的 PSP），中断也还没有被任何任务持有。
 * - `Jump` 之前调用方必须自己确认向量表是合理的——本模块不做那个判断，
 *   它是策略（见 APP_Boot_IsVectorReasonable）。
 */

/* 读向量表头两个字：初始 MSP 与复位向量。 */
void BSP_RomBootloader_ReadVector(uint32_t vector_address,
                                  uint32_t *initial_msp,
                                  uint32_t *reset_handler);

/* 写请求魔数。返回 0 = 没写进去（备份域打不开）。 */
uint8_t BSP_RomBootloader_WriteRequestMagic(uint32_t magic);

/*
 * 请求魔数在不在？**在的话顺手清掉**。
 *
 * 读和清必须是一次操作：分两步而中间掉电，下次上电会再跳一次，
 * 于是板子卡在"永远进 DFU"的循环里——那是要靠拆机短接才能救回来的状态。
 */
uint8_t BSP_RomBootloader_TakeRequestMagic(uint32_t magic);

/* 静默外设（NVIC / SysTick / cache / MPU）、切向量表、换栈、跳过去。不返回。 */
void BSP_RomBootloader_Jump(uint32_t vector_address,
                            uint32_t initial_msp,
                            uint32_t reset_handler);

/* 软复位整机。 */
void BSP_RomBootloader_SystemReset(void);

#endif /* BSP_ROM_BOOTLOADER_H */
