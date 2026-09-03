#include "app_telem_frame.h"

#include "app_proto.h"

#include <stddef.h>
#include <string.h>

/*
 * 暂存 payload 的缓冲放静态区而不是栈上：遥测任务栈只有 2 KB
 * （freertos.c 里 VOFA_Task stack_size = 512 * 4），256 B 的 payload 再加上
 * 调用方的成帧缓冲直接压栈会吃掉四分之一余量。
 *
 * 代价是本模块**不可重入**。当前只有遥测任务一个调用者，这个前提写在这里；
 * 出现第二个调用者时必须改成由调用方传缓冲区，不要只加一把锁了事。
 */
static uint8_t app_telem_frame_payload[APP_TELEM_FRAME_MAX_PAYLOAD];

/* 按字节写小端，不依赖宿主/目标的字节序——黄金向量要两边逐字节相同。 */
static void app_telem_frame_put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void app_telem_frame_put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFUL);
    dst[1] = (uint8_t)((value >> 8) & 0xFFUL);
    dst[2] = (uint8_t)((value >> 16) & 0xFFUL);
    dst[3] = (uint8_t)((value >> 24) & 0xFFUL);
}

static void app_telem_frame_put_u64(uint8_t *dst, uint64_t value)
{
    app_telem_frame_put_u32(dst, (uint32_t)(value & 0xFFFFFFFFULL));
    app_telem_frame_put_u32(dst + 4, (uint32_t)((value >> 32) & 0xFFFFFFFFULL));
}

static void app_telem_frame_put_f32(uint8_t *dst, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    app_telem_frame_put_u32(dst, bits);
}

uint32_t APP_TelemFrame_PopCount(uint64_t mask)
{
    uint32_t count = 0U;

    while (mask != 0ULL) {
        mask &= (mask - 1ULL);
        count++;
    }

    return count;
}

uint32_t APP_TelemFrame_PayloadLength(uint8_t count, uint64_t mask)
{
    uint32_t channels;

    if ((count == 0U) || (mask == 0ULL)) {
        return 0U;
    }

    channels = APP_TelemFrame_PopCount(mask);
    return APP_TELEM_FRAME_HEADER_BYTES + (4UL * (uint32_t)count * channels);
}

APP_TelemFrameStatus APP_TelemFrame_Encode(const APP_TelemFrameDesc *desc,
                                           const float *values,
                                           uint32_t value_count,
                                           uint16_t max_payload,
                                           uint8_t *out_buffer,
                                           uint16_t out_capacity,
                                           uint16_t *out_length)
{
    uint32_t payload_length;
    uint32_t expected_values;
    uint32_t index;
    uint32_t offset;

    if ((desc == NULL) || (values == NULL) || (out_buffer == NULL) ||
        (out_length == NULL)) {
        return APP_TELEM_FRAME_ERR_ARGS;
    }

    payload_length = APP_TelemFrame_PayloadLength(desc->count, desc->mask);
    if (payload_length == 0U) {
        return APP_TELEM_FRAME_ERR_ARGS;
    }

    expected_values = (uint32_t)desc->count * APP_TelemFrame_PopCount(desc->mask);
    if (expected_values != value_count) {
        /*
         * 值个数与掩码对不上就是编码方自己算错了。这里宁可整帧不发也不发一个
         * 长度自洽但内容错位的帧——后者在接收端完全合法，只会让每条曲线悄悄
         * 挪一格，正是掩码帧要消灭的失败模式。
         */
        return APP_TELEM_FRAME_ERR_ARGS;
    }

    if ((payload_length > (uint32_t)max_payload) ||
        (payload_length > APP_TELEM_FRAME_MAX_PAYLOAD)) {
        return APP_TELEM_FRAME_ERR_TOO_LARGE;
    }

    app_telem_frame_payload[0] = APP_TELEM_FRAME_VERSION;
    app_telem_frame_payload[1] = desc->count;
    app_telem_frame_put_u16(&app_telem_frame_payload[2], desc->seq);
    app_telem_frame_put_u32(&app_telem_frame_payload[4], desc->schema);
    app_telem_frame_put_u32(&app_telem_frame_payload[8], desc->t_us);
    app_telem_frame_put_u16(&app_telem_frame_payload[12], desc->dt_us);
    app_telem_frame_put_u16(&app_telem_frame_payload[14], desc->flags);
    app_telem_frame_put_u64(&app_telem_frame_payload[16], desc->mask);

    offset = APP_TELEM_FRAME_HEADER_BYTES;
    for (index = 0U; index < value_count; ++index) {
        app_telem_frame_put_f32(&app_telem_frame_payload[offset], values[index]);
        offset += 4U;
    }

    if (APP_Proto_BuildFrame(APP_PROTO_DIR_FROM_FC,
                             APP_PROTO_MSG_TELEM_FRAME,
                             app_telem_frame_payload,
                             (uint16_t)payload_length,
                             out_buffer,
                             out_capacity,
                             out_length) == 0U) {
        return APP_TELEM_FRAME_ERR_BUILD;
    }

    return APP_TELEM_FRAME_OK;
}
