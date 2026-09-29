#ifndef BSP_DSHOT_H
#define BSP_DSHOT_H
#include <stdint.h>

typedef enum {
    BSP_DSHOT_OK = 0,
    BSP_DSHOT_BUSY,
    BSP_DSHOT_INVALID,
    BSP_DSHOT_ERROR
} BSP_DShotStatus;

typedef struct {
    uint32_t submitted;
    uint32_t completed; /* DMA completed, not ESC acknowledgement */
    uint32_t busy_rejected;
    uint32_t errors;
    uint32_t cancelled;
    uint32_t timer_clock_hz;
    uint16_t code[2];   /* last accepted code; unit=DShot code, not us */
    uint8_t enabled_mask;
    uint8_t busy;
    uint8_t fault;
} BSP_DShotSnapshot;

/* Init only at boot or explicit reinitialization. Never call from Set/Submit.
 * M4/PE9 = channel 1; M3/PE11 = channel 2. Init holds both outputs inactive.
 */
BSP_DShotStatus BSP_DShot_Init(void);
/* Atomic two-channel frame, no queue; mask bit0/1 correspond to channel1/2.
 * mask=0 means hard disable, code=0 with an enabled bit means a STOP frame.
 * Caller is the 500 Hz final actuator commit. Busy does not touch DMA memory.
 */
BSP_DShotStatus BSP_DShot_Submit(const uint16_t code[2], uint8_t enabled_mask);

/*
 * 发一帧 DShot 特殊命令（1..47）给 mask 指定的通道。
 *
 * 与 Submit 共用同一条发送路径，只有编码器不同（见 drv_dshot.h 的 EncodeCommand）。
 * **一次只发一帧**：电调要连着收到同一条命令若干帧才认，而"连发几帧、中间不许夹
 * 油门"是策略，归 App 层的 app_esc_command.c，不在这里循环——BSP 没有节拍。
 *
 * `enabled_mask == 0` 是非法，不像 Submit 那样被当成"全部禁用"：诊断命令不该有
 * 关闭输出这种副作用。
 */
BSP_DShotStatus BSP_DShot_SubmitCommand(uint16_t command, uint8_t enabled_mask);

/*
 * 两路电调信号脚（M4/PE9、M3/PE11）此刻的**实际电平**，bit0=通道1、bit1=通道2。
 *
 * 为什么需要它：双向 DShot 的空闲电平必须是**高**，而这件事整条链上没有任何地方
 * 能观测——CCER 的 CCxP、BDTR 的 OSSR、CR2 的 OISx 都是"我们以为设对了"，引脚上
 * 真正出来什么只有 IDR 知道。2026-09-21 电调完全不回话，而 AM32 恰恰是靠"未解锁
 * 时读到线上是高"来自检双向模式的（`AM32/Src/dshot.c`），于是"线到底高不高"从
 * 一个推断变成了必须量的东西。
 *
 * 引脚配成定时器复用输出时 IDR 仍然反映真实电平，所以这是真测量不是回读寄存器。
 * 只读，无副作用；单次采样，调用方自己决定采多久（帧占空比约 2.6%，单次大概率
 * 落在空闲段，但要下结论应当在一整个帧周期上统计）。
 */
uint8_t BSP_DShot_ReadEscPinLevels(void);
/* Immediate physical disable; usable with interrupts masked, never waits for IRQ.
 * Pending completion cannot re-enable output. Returns ERROR if DMA won't stop.
 */
BSP_DShotStatus BSP_DShot_Disable(uint8_t channel_mask);
void BSP_DShot_GetSnapshot(BSP_DShotSnapshot *out);

#endif
