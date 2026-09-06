#ifndef APP_TELEM_FRAME_H
#define APP_TELEM_FRAME_H

#include <stdint.h>

/*
 * 自描述掩码帧编码器（doc/telemetry-protocol.md）。
 *
 * 为什么帧要自带掩码和 schema 指纹：旧的 VOFA JustFloat 帧是定长 28 float，
 * 接收端靠"第 N 个 float 是谁"这条**约定**解帧。往通道表里插一条，线上还是
 * 一堆没有标签的 float，上位机不报错，只是从此每条曲线都错位一格。掩码帧把
 * 那条约定搬到了每一帧里：
 *
 *   - 包长可变：长度由 popcount(mask) 推出来，与 $X 的 len 交叉校验；
 *   - 不错位：schema 指纹变了接收端立刻丢帧并重拉通道表；
 *   - 不冗余：参数通道平时不置位，只有值变了才带上。
 *
 * payload（小端）：
 *   off  0  u8   ver
 *   off  1  u8   count      本帧样本数 >= 1
 *   off  2  u16  seq
 *   off  4  u32  schema     APP_Telemetry_SchemaHash()
 *   off  8  u32  t_us       首样本时间戳低 32 位
 *   off 12  u16  dt_us      批内间隔，count==1 时为 0
 *   off 14  u16  flags      bit0 = 全量刷新帧；bit1 = 宽掩码
 *   off 16  mask            bit1==0 时 8 字节（通道 0..63）
 *                           bit1==1 时 16 字节（通道 0..127）
 *   data                    紧接掩码之后，f32[count * popcount(mask)]，
 *                           按通道索引升序、再按样本序
 *
 * **掩码为什么是变长的**：通道表在 R-S5-1 之后就顶到了 u64 的 64 路上限，
 * 追加增益回显通道必须把掩码扩宽。但定长 16 字节掩码会让**每一帧**都多 8 B，
 * 40 Hz 下多 312 B/s，足以把默认配置顶过数传 57600 的 60% 带宽门限
 * （tests/test_telem_stream_contract.py 的 test_default_configuration_fits...）。
 * 实时通道全部在低 64 位，稳态帧因此仍发 8 字节掩码、与 v1 逐字节等长；
 * 只有触及高通道的全量刷新帧付那 8 B。宽窄由掩码内容唯一决定（hi 非零即宽），
 * 不是由发送方随手选的，所以黄金向量是确定的。
 *
 * 本模块**不依赖 HAL / FreeRTOS**：帧格式要能在宿主 gcc 上跑黄金向量，
 * 两端逐字节一致只能靠"同一份 C 代码产出期望字节"来钉，不能各写各的。
 */

/* v2：掩码变长（见上）。v1 是定长 u64 掩码，接收端必须整帧拒绝。 */
#define APP_TELEM_FRAME_VERSION           2U
#define APP_TELEM_FRAME_HEADER_BYTES      24U  /* 窄掩码 */
#define APP_TELEM_FRAME_HEADER_BYTES_WIDE 32U  /* 宽掩码 */
#define APP_TELEM_FRAME_MAX_CHANNELS      128U

/* $X 成帧开销：'$' 'X' dir flags fn(2) len(2) ... crc = 9 字节。 */
#define APP_TELEM_FRAME_OVERHEAD 9U

/* R-T1 的 payload 上限就是 $X 协议上限；R-T2 给 USB 出口单独放宽。 */
#define APP_TELEM_FRAME_MAX_PAYLOAD 256U

/* flags 位。其余位保留，必须写 0。 */
#define APP_TELEM_FRAME_FLAG_FULL_REFRESH 0x0001U
#define APP_TELEM_FRAME_FLAG_WIDE_MASK    0x0002U

/*
 * 128 位通道掩码。ARM 32 位 GCC 没有 `__int128`，所以用两个 u64 拼：`lo` 是
 * 通道 0..63，`hi` 是 64..127。按值传递而不是传指针——这一层全是纯函数，值
 * 语义让"掩码在哪儿被人偷偷改了"这类问题根本不存在。
 */
typedef struct {
    uint64_t lo;
    uint64_t hi;
} APP_TelemMask;

static inline APP_TelemMask APP_TelemMask_Zero(void)
{
    APP_TelemMask mask = {0ULL, 0ULL};
    return mask;
}

static inline APP_TelemMask APP_TelemMask_FromBit(uint32_t bit)
{
    APP_TelemMask mask = {0ULL, 0ULL};

    if (bit < 64U) {
        mask.lo = 1ULL << bit;
    } else if (bit < APP_TELEM_FRAME_MAX_CHANNELS) {
        mask.hi = 1ULL << (bit - 64U);
    }
    return mask;
}

static inline uint8_t APP_TelemMask_Test(APP_TelemMask mask, uint32_t bit)
{
    if (bit < 64U) {
        return ((mask.lo >> bit) & 1ULL) != 0ULL;
    }
    if (bit < APP_TELEM_FRAME_MAX_CHANNELS) {
        return ((mask.hi >> (bit - 64U)) & 1ULL) != 0ULL;
    }
    return 0U;
}

static inline APP_TelemMask APP_TelemMask_Or(APP_TelemMask a, APP_TelemMask b)
{
    APP_TelemMask mask = {a.lo | b.lo, a.hi | b.hi};
    return mask;
}

static inline APP_TelemMask APP_TelemMask_And(APP_TelemMask a, APP_TelemMask b)
{
    APP_TelemMask mask = {a.lo & b.lo, a.hi & b.hi};
    return mask;
}

/* a & ~b */
static inline APP_TelemMask APP_TelemMask_AndNot(APP_TelemMask a, APP_TelemMask b)
{
    APP_TelemMask mask = {a.lo & ~b.lo, a.hi & ~b.hi};
    return mask;
}

static inline uint8_t APP_TelemMask_IsEmpty(APP_TelemMask mask)
{
    return ((mask.lo == 0ULL) && (mask.hi == 0ULL)) ? 1U : 0U;
}

/* 需要 16 字节掩码字段吗。宽窄只由内容决定，发送方没有选择余地。 */
static inline uint8_t APP_TelemMask_IsWide(APP_TelemMask mask)
{
    return (mask.hi != 0ULL) ? 1U : 0U;
}

typedef enum {
    APP_TELEM_FRAME_OK = 0,
    APP_TELEM_FRAME_ERR_ARGS,       /* 参数不自洽（count/mask/值个数对不上） */
    APP_TELEM_FRAME_ERR_TOO_LARGE,  /* 超出本出口的 payload 上限 */
    APP_TELEM_FRAME_ERR_BUILD       /* $X 成帧失败（输出缓冲太小） */
} APP_TelemFrameStatus;

typedef struct {
    uint8_t       count;
    uint16_t      seq;
    uint32_t      schema;
    uint32_t      t_us;
    uint16_t      dt_us;
    uint16_t      flags;   /* WIDE_MASK 由编码器按 mask 内容自行填，调用方不必给 */
    APP_TelemMask mask;
} APP_TelemFrameDesc;

/* mask 里置位的通道数。 */
uint32_t APP_TelemFrame_PopCount(APP_TelemMask mask);

/* 该掩码对应的 payload 头长度（24 或 32）。 */
uint32_t APP_TelemFrame_HeaderBytes(APP_TelemMask mask);

/*
 * 该 (count, mask) 组合的 payload 字节数。返回 0 表示组合非法（count 为 0
 * 或 mask 为空）——调用方据此拒绝配置，**不允许**退化成"截断着发"。
 */
uint32_t APP_TelemFrame_PayloadLength(uint8_t count, APP_TelemMask mask);

/*
 * 编码一帧。values 必须正好有 count * popcount(mask) 个元素，按通道索引升序、
 * 再按样本序排列。max_payload 是本出口能接受的 payload 上限，超了返回
 * ERR_TOO_LARGE 而不是截断。
 */
APP_TelemFrameStatus APP_TelemFrame_Encode(const APP_TelemFrameDesc *desc,
                                           const float *values,
                                           uint32_t value_count,
                                           uint16_t max_payload,
                                           uint8_t *out_buffer,
                                           uint16_t out_capacity,
                                           uint16_t *out_length);

#endif /* APP_TELEM_FRAME_H */
