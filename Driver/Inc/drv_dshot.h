#ifndef DRV_DSHOT_H
#define DRV_DSHOT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 单向 DShot300 协议。这里只生成数据，不驱动设备，也不拥有发送状态。
 * BSP 负责 DMA 内存、锁、硬件启动/停止以及发送期间的缓冲所有权。
 * 没有全局可变状态；输入和输出不得重叠。失败时所有输出保持原值。
 */
typedef enum {
    DRV_DSHOT_OK = 0,
    DRV_DSHOT_INVALID
} DRV_DShotStatus;

#define DRV_DSHOT_BIT_RATE       300000U
#define DRV_DSHOT_CHANNELS       2U
#define DRV_DSHOT_BITS          16U
#define DRV_DSHOT_TAIL_SLOTS     2U
#define DRV_DSHOT_SLOTS         (DRV_DSHOT_BITS + DRV_DSHOT_TAIL_SLOTS)
#define DRV_DSHOT_BURST_WORDS    (DRV_DSHOT_SLOTS * DRV_DSHOT_CHANNELS)
#define DRV_DSHOT_STOP          0U
#define DRV_DSHOT_CMD_MIN       1U
#define DRV_DSHOT_CMD_MAX       47U
#define DRV_DSHOT_THROTTLE_MIN  48U
#define DRV_DSHOT_THROTTLE_MAX  2047U
#define DRV_DSHOT_EQUIV_MIN_US  1100U
#define DRV_DSHOT_EQUIV_MAX_US  1940U

typedef struct {
    uint32_t timer_clock_hz; /* 定时器内核时钟，不能传 CPU/PCLK 频率代替 */
    uint32_t prescaler;      /* 写入 PSC 的值，本版始终为 0 */
    uint32_t auto_reload;    /* 写入 ARR 的值；一个 bit 有 ARR + 1 个 tick */
    uint32_t zero_ticks;     /* bit 0 高电平为 3/8 周期 */
    uint32_t one_ticks;      /* bit 1 高电平为 6/8 周期 */
} DRV_DShotTiming;

/* 要求时钟能精确产生上述时序，且适用于 16 位定时器。
 * 120 MHz -> PSC=0, ARR=399, zero_ticks=150, one_ticks=300。
 */
DRV_DShotStatus DRV_DShot_MakeTiming(uint32_t timer_clock_hz,
                                    DRV_DShotTiming *out);

/* 等效 PWM 命令 -> DShot 码，不代表同转速/同推力。
 * 1100 -> 0；1101..1940 -> 48 + floor((us-1101)*1999/839)。
 * us=0 是现有接口的“物理禁用”标记，必须由 BSP 单独处理，本函数拒绝它。
 */
DRV_DShotStatus DRV_DShot_FromPulseUs(uint16_t pulse_us, uint16_t *out);

/* 仅接受 0 或 48..2047；拒绝特殊命令 1..47。
 * telemetry=0，普通校验（不取反）。输出是 MSB 先发的 16-bit 数字帧。
 */
DRV_DShotStatus DRV_DShot_Encode(uint16_t throttle, uint16_t *out);

/* 单向 DShot 的特殊命令帧。仅接受 1..47；0 和 48..2047 是油门，一律拒绝
 * （拒绝它们不是多余的：调用方把油门误传进来时，静默编成一条命令是最坏结果）。
 * telemetry 位固定为 1——电调（如 AM32）靠这一位把该帧当命令而不是极低油门
 * 来解释，写 0 会被解成一个几乎堵转的油门值。校验不取反，规则与 DRV_DShot_Encode
 * 一致；走的是同一个入口下的独立函数，不放宽 DRV_DShot_Encode 本身的校验。
 */
DRV_DShotStatus DRV_DShot_EncodeCommand(uint16_t command, uint16_t *out);

/* 两路按 bit0/ch1, bit0/ch2, bit1/ch1, bit1/ch2... 交错。
 * 末尾两组 CCR=0 是低电平收尾，不是两个额外的 DShot 0 bit。
 * STOP 帧依然包含 16 个 bit-0 脉冲，不等同于物理禁用。
 * 只写 DRV_DSHOT_BURST_WORDS 个 uint32_t；余下空间不修改。
 * 本缓冲不定义定时器首位预装载的启动顺序，BSP 必须单独验证首/末位。
 */
DRV_DShotStatus DRV_DShot_BuildBurst(const uint16_t throttle[DRV_DSHOT_CHANNELS],
                                    const DRV_DShotTiming *timing,
                                    uint32_t *out, size_t capacity_words);

/*
 * 同样的交错，但输入是**已经编好的 16 bit 数字帧**而不是油门。
 * 双向 DShot 的校验是取反的（见 drv_dshot_telemetry.h），编码规则不同但线上
 * 交错完全一样；让它走这里，交错这段 PX4 来源的代码就只保留一份。
 * 不对 packet 内容做任何校验——校验属于生成 packet 的那一层。
 */
DRV_DShotStatus DRV_DShot_BuildBurstFromPackets(
    const uint16_t packet[DRV_DSHOT_CHANNELS],
    const DRV_DShotTiming *timing,
    uint32_t *out, size_t capacity_words);

#ifdef __cplusplus
}
#endif
#endif
