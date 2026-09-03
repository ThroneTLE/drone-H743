#ifndef APP_TELEM_FRAME_H
#define APP_TELEM_FRAME_H

#include <stdint.h>

/*
 * 遥测流 v2 的自描述掩码帧编码器（doc/telemetry-scope-plan.md §2.2）。
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
 *   off 14  u16  flags      bit0 = 全量刷新帧
 *   off 16  u64  mask       bit i = 通道 i 在本帧中出现
 *   off 24  f32  data[count * popcount(mask)]  按通道索引升序、再按样本序
 *
 * 本模块**不依赖 HAL / FreeRTOS**：帧格式要能在宿主 gcc 上跑黄金向量，
 * 两端逐字节一致只能靠"同一份 C 代码产出期望字节"来钉，不能各写各的。
 */

#define APP_TELEM_FRAME_VERSION      1U
#define APP_TELEM_FRAME_HEADER_BYTES 24U

/* $X 成帧开销：'$' 'X' dir flags fn(2) len(2) ... crc = 9 字节。 */
#define APP_TELEM_FRAME_OVERHEAD 9U

/* R-T1 的 payload 上限就是 $X 协议上限；R-T2 给 USB 出口单独放宽。 */
#define APP_TELEM_FRAME_MAX_PAYLOAD 256U

/* flags 位。其余位保留，必须写 0。 */
#define APP_TELEM_FRAME_FLAG_FULL_REFRESH 0x0001U

typedef enum {
    APP_TELEM_FRAME_OK = 0,
    APP_TELEM_FRAME_ERR_ARGS,       /* 参数不自洽（count/mask/值个数对不上） */
    APP_TELEM_FRAME_ERR_TOO_LARGE,  /* 超出本出口的 payload 上限 */
    APP_TELEM_FRAME_ERR_BUILD       /* $X 成帧失败（输出缓冲太小） */
} APP_TelemFrameStatus;

typedef struct {
    uint8_t  count;
    uint16_t seq;
    uint32_t schema;
    uint32_t t_us;
    uint16_t dt_us;
    uint16_t flags;
    uint64_t mask;
} APP_TelemFrameDesc;

/* mask 里置位的通道数。 */
uint32_t APP_TelemFrame_PopCount(uint64_t mask);

/*
 * 该 (count, mask) 组合的 payload 字节数。返回 0 表示组合非法（count 为 0
 * 或 mask 为空）——调用方据此拒绝配置，**不允许**退化成"截断着发"。
 */
uint32_t APP_TelemFrame_PayloadLength(uint8_t count, uint64_t mask);

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
