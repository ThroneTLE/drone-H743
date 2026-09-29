#ifndef BSP_ESC_PROTOCOL_H
#define BSP_ESC_PROTOCOL_H

#define BSP_ESC_PROTOCOL_PWM 0
#define BSP_ESC_PROTOCOL_DSHOT300 1
/*
 * 双向 DShot300：线电平取反、校验取反，电调在每帧之后回传电周期。
 * 与单向是**不同的线上协议**，电调侧也必须切到双向模式，两边对不上就完全收不到帧。
 * 因此它是独立的一档而不是"DShot300 加个开关"。
 */
#define BSP_ESC_PROTOCOL_DSHOT300_BIDIR 2
#ifndef BSP_ESC_PROTOCOL
#define BSP_ESC_PROTOCOL BSP_ESC_PROTOCOL_DSHOT300
#endif
#if BSP_ESC_PROTOCOL != BSP_ESC_PROTOCOL_PWM && \
    BSP_ESC_PROTOCOL != BSP_ESC_PROTOCOL_DSHOT300 && \
    BSP_ESC_PROTOCOL != BSP_ESC_PROTOCOL_DSHOT300_BIDIR
#error Unsupported ESC protocol
#endif

/* 发送路径在两档 DShot 下完全一致，只有极性与校验不同。 */
#define BSP_ESC_PROTOCOL_IS_DSHOT \
    ((BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300) || \
     (BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR))

/*
 * 双向档的**实现后端**。这是实现选择，不是线上协议——协议仍然是
 * DSHOT300_BIDIR，所以它不该占一个 BSP_ESC_PROTOCOL 取值。
 *
 *   TIMER   ：DMA burst 写 TIM1->DMAR 发送，发完把通道翻成输入捕获收回传。
 *             2026-09-21 实机：帧值手算核过、引脚 98~100% idle 高（量出来的）、
 *             `proto=2 avail=1` 确认过，而电调**完全不响应**——不转也不回传。
 *             参考实现明确记载 burst DMA 与双向 DShot 不兼容。
 *   BITBANG ：按固定节拍 DMA 写 GPIO 的 BSRR / 读 IDR，定时器只当时钟源。
 *             这是 Betaflight 在绝大多数板子上的默认做法。本板两路信号脚
 *             PE9/PE11 同在 GPIOE，所以一条流能同时驱动并同时采样两路。
 *
 * 默认 BITBANG。TIMER 仍然编得过、测得到，留着是为了能在同一块板子上 A/B——
 * 删掉它就没法再证明"换后端确实是那个变量"。
 */
#define BSP_ESC_BIDIR_BACKEND_TIMER   0
#define BSP_ESC_BIDIR_BACKEND_BITBANG 1
#ifndef BSP_ESC_BIDIR_BACKEND
#define BSP_ESC_BIDIR_BACKEND BSP_ESC_BIDIR_BACKEND_BITBANG
#endif
#if BSP_ESC_BIDIR_BACKEND != BSP_ESC_BIDIR_BACKEND_TIMER && \
    BSP_ESC_BIDIR_BACKEND != BSP_ESC_BIDIR_BACKEND_BITBANG
#error Unsupported bidirectional DShot backend
#endif

/* 只有双向档才谈得上后端选择；单向档与 PWM 恒为假。 */
#define BSP_ESC_PROTOCOL_IS_BITBANG \
    ((BSP_ESC_PROTOCOL == BSP_ESC_PROTOCOL_DSHOT300_BIDIR) && \
     (BSP_ESC_BIDIR_BACKEND == BSP_ESC_BIDIR_BACKEND_BITBANG))

#endif
