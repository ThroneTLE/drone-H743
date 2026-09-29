#ifndef DRV_DSHOT_TELEMETRY_H
#define DRV_DSHOT_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 双向 DShot300（bidirectional DShot / "bdshot"）的**纯解码**层。
 *
 * 这里只有整数运算：没有寄存器、没有 DMA、没有 RTOS、没有全局可变状态。
 * BSP 负责把线上电平翻转的时刻抓成一串定时器捕获值再交给本模块。
 * 这样做的唯一目的是让协议本身能在 PC 上用已知向量判对错——
 * GCR 表错一格、CRC 取反忘了、指数尾数搞反，这三种错在实机上都表现为
 * "偶尔读到一个离谱转速"，靠看波形是查不出来的。
 *
 * ──────────────── 与单向 DShot 的三点差别 ────────────────
 *
 *   1. **线电平取反**：空闲为高，数据位为低。BSP 负责把定时器输出极性设成
 *      低有效；本模块的 Encode 只管数字帧。
 *   2. **CRC 取反**：单向是 `crc = x ^ (x>>4) ^ (x>>8)`，双向是它的**反码**。
 *      电调用这一位区分两种模式，写错了电调会直接忽略整帧。
 *   3. **电调回话**：帧尾之后约 30 us，电调用 5/4 倍波特率（DShot300 → 375 kbit/s）
 *      发回 21 bit 的 GCR 编码转速帧。
 *
 * ──────────────── 回传帧的数值含义 ────────────────
 *
 * 解出来的 12 bit 是 `eeemmmmmmmmm`：3 bit 指数 + 9 bit 尾数，
 * 表示**电周期**而不是转速：`period_us = mantissa << exponent`。
 * 电转速 `eRPM = 60e6 / period_us`；机械转速还要除以极对数。
 * `0x0FFF` 是电调明确表示"没在转"，不是解码失败——两者必须分开报。
 */

typedef enum {
    DRV_DSHOT_TELEM_OK = 0,
    DRV_DSHOT_TELEM_INVALID,      /* 入参不合法 */
    DRV_DSHOT_TELEM_BAD_EDGES,    /* 边沿数/间隔无法还原成 21 bit */
    DRV_DSHOT_TELEM_BAD_GCR,      /* 五位组不在 GCR 表里 */
    DRV_DSHOT_TELEM_BAD_CRC       /* GCR 解出来了但校验不过 */
} DRV_DShotTelemStatus;

/* 回传帧固定 21 bit（1 个起始 0 + 20 bit GCR 载荷）。 */
#define DRV_DSHOT_TELEM_FRAME_BITS      21U
/* 21 bit 最多 21 次电平翻转，留一倍余量给毛刺，BSP 按此定捕获缓冲大小。 */
#define DRV_DSHOT_TELEM_MAX_EDGES       48U
/* 回传波特率是发送波特率的 5/4。 */
#define DRV_DSHOT_TELEM_RATE_NUM        5U
#define DRV_DSHOT_TELEM_RATE_DEN        4U
/* 电调明确回报"未旋转"的哨兵值，区别于解码失败。 */
#define DRV_DSHOT_TELEM_NOT_SPINNING    0x0FFFU

typedef struct {
    uint16_t period_us;     /* 电周期，us；not-spinning 时为 0 */
    uint32_t erpm;          /* 电转速；not-spinning 时为 0 */
    uint8_t  not_spinning;  /* 1 = 电调回报未旋转（有效回包，不是错误） */
    uint8_t  kind;          /* DRV_DSHOT_TELEM_VALUE_* */
    uint8_t  edt_type;      /* EDT type nibble (0x02..0x0e), otherwise 0 */
    uint8_t  edt_value;     /* EDT payload, type-specific units */
} DRV_DShotTelemValue;

typedef enum {
    DRV_DSHOT_TELEM_VALUE_ERPM = 0U,
    DRV_DSHOT_TELEM_VALUE_EDT = 1U
} DRV_DShotTelemValueKind;

#define DRV_DSHOT_EDT_TEMPERATURE  0x02U /* 1 deg C / LSB */
#define DRV_DSHOT_EDT_VOLTAGE      0x04U /* 0.25 V / LSB */
#define DRV_DSHOT_EDT_CURRENT      0x06U /* 1 A / LSB */
#define DRV_DSHOT_EDT_DEBUG1       0x08U
#define DRV_DSHOT_EDT_DEBUG2       0x0AU
#define DRV_DSHOT_EDT_DEBUG3       0x0CU
#define DRV_DSHOT_EDT_STATE_EVENT  0x0EU

/*
 * 双向 DShot 的数字帧。`telemetry_request` 为 1 时请求的是**扩展遥测**
 * （电压/温度等），常规转速回传该位给 0——电调只要处在双向模式就会回话。
 * 油门取值与单向一致：0 或 48..2047。
 */
DRV_DShotTelemStatus DRV_DShotTelem_EncodeRequest(uint16_t throttle,
                                                  uint8_t telemetry_request,
                                                  uint16_t *out);

/*
 * 双向档的特殊命令帧（例如命令 13 = Extended DShot Telemetry enable）。
 * 仅接受 1..47，规则与 DRV_DShot_EncodeCommand（drv_dshot.h）一致：
 * telemetry 位固定为 1，value 字段是 command。两个模块互不包含对方头文件，
 * 所以校验范围在这里独立重写一遍，但两处的数值边界必须保持一致——
 * 改一处务必同时改另一处。
 *
 * 双向档的同一件事：载荷相同，4 bit 校验取反（同 DRV_DShotTelem_EncodeRequest
 * 与 DRV_DShot_Encode 的关系）。
 */
DRV_DShotTelemStatus DRV_DShotTelem_EncodeCommand(uint16_t command, uint16_t *out);

/*
 * 把捕获到的边沿时刻还原成 21 bit 原始码流。
 *
 * `edge_ticks` 是定时器捕获值，按采集顺序排列，允许 16 位回绕（内部按无符号
 * 差值处理）。`edge_count` 必须 >= 2：第一个是空闲高电平后的**下降沿**，
 * 也就是起始位的开始。
 *
 * `ticks_per_bit_q8` 是"一个回传位占多少定时器 tick"的 Q8 定点值，
 * 由 BSP 用真实定时器时钟算出来传进来，不在这里写死。
 *
 * 末尾不足 21 bit 的部分按最后一次翻转后的电平补齐（线路回到空闲高电平）。
 */
DRV_DShotTelemStatus DRV_DShotTelem_BitsFromEdges(const uint16_t *edge_ticks,
                                                  size_t edge_count,
                                                  uint32_t ticks_per_bit_q8,
                                                  uint32_t *out_raw21);

/* 21 bit 原始码流 -> 12 bit 载荷。做 GCR 反查表与 4 bit 校验。 */
DRV_DShotTelemStatus DRV_DShotTelem_DecodeRaw(uint32_t raw21, uint16_t *out_payload12);

/* 12 bit 载荷 -> 周期与电转速。`0x0FFF` 走 not_spinning 分支。 */
DRV_DShotTelemStatus DRV_DShotTelem_ValueFromPayload(uint16_t payload12,
                                                     DRV_DShotTelemValue *out);

/* 边沿 -> 转速的一步到位版本，BSP 用这个。 */
DRV_DShotTelemStatus DRV_DShotTelem_Decode(const uint16_t *edge_ticks,
                                           size_t edge_count,
                                           uint32_t ticks_per_bit_q8,
                                           DRV_DShotTelemValue *out);

/*
 * 电转速 -> 机械转速。`pole_pairs` 是**极对数**，不是磁极数；
 * 传 0 视为非法并返回 0，不做"猜一个默认值"这种事。
 */
uint32_t DRV_DShotTelem_MechanicalRpm(uint32_t erpm, uint8_t pole_pairs);

/*
 * 给定发送波特率与定时器时钟，算出回传位的 Q8 tick 数。
 * 回传波特率 = bit_rate_hz * 5 / 4。
 */
DRV_DShotTelemStatus DRV_DShotTelem_TicksPerBitQ8(uint32_t timer_clock_hz,
                                                  uint32_t bit_rate_hz,
                                                  uint32_t *out_q8);

#ifdef __cplusplus
}
#endif
#endif
