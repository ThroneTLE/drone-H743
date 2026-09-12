#ifndef BSP_CRITICAL_H
#define BSP_CRITICAL_H

#include <stdint.h>

/*
 * 极短临界区：关中断 / 恢复中断。
 *
 * 存在的理由是 D1-2——App 与 Services 里为了"把两三条赋值凑成原子的"而直接写
 * `__disable_irq()` / `__enable_irq()`，那是 CMSIS 内核头里的东西，得连着 HAL
 * 头一起包进来，于是整个文件就跟芯片绑上了。
 *
 * === 契约 ===
 *
 * - **必须成对**，且必须用 Enter 的返回值去 Exit：
 *
 *       uint32_t state = BSP_Critical_Enter();
 *       ...几条赋值...
 *       BSP_Critical_Exit(state);
 *
 *   不要写成"Exit 就是无条件开中断"。嵌套时内层无条件开中断会把外层的保护
 *   一起拆掉，而这种错误只在两段临界区恰好嵌套的时机才出问题。
 *
 * - **只保护几条指令**。这里挡的是中断，不是任务调度，所以代价直接落在
 *   1 kHz 控制环的抖动上。需要等待、需要循环、需要调用别人的，用 RTOS 的锁。
 *
 * - 可在任务与中断上下文调用。
 */

uint32_t BSP_Critical_Enter(void);
void BSP_Critical_Exit(uint32_t state);

/*
 * 内存屏障：保证屏障之前的写对之后的读者已经可见。
 *
 * 无锁发布（先写数据、再更新序号/标志）必须用它。单核的原子读写挡不住
 * **编译器和流水线重排**——少了这一句，读者可能先看到新序号、再读到旧数据，
 * 而这种错序只在特定的优化等级和时序下才出现，事后几乎复现不出来。
 */
void BSP_Critical_MemoryBarrier(void);

#endif /* BSP_CRITICAL_H */
