#include "svc_flow_capture.h"

#include <stddef.h>

/* 64 KB 放 AXI SRAM（不清零段）：.bss 在 DTCM，DTCM 已用 97%。宿主测试编译时不加段属性。 */
#if defined(__arm__)
#define SVC_FLOW_CAPTURE_STORAGE __attribute__((section(".ram_d1_noinit"), aligned(32)))
#else
#define SVC_FLOW_CAPTURE_STORAGE
#endif

static SVC_FlowCaptureFrame flow_capture_frames[SVC_FLOW_CAPTURE_MAX_FRAMES] SVC_FLOW_CAPTURE_STORAGE;

/*
 * 单写者（光流服务所在任务）推、命令任务读：先写帧再加计数，读方只读 index < Count() 的帧。
 * 一次性模式下已记的帧不再改写，抓取中也可读；环形模式会覆盖最老的帧，须先 Stop 再读。
 */
static volatile uint8_t flow_capture_active;
static volatile uint8_t flow_capture_ring;
static volatile uint16_t flow_capture_head;
static volatile uint32_t flow_capture_total;
static volatile uint16_t flow_capture_target;
static uint32_t flow_capture_last_received_ms;

static void flow_capture_restart(uint8_t ring, uint16_t target)
{
    flow_capture_active = 0U;
    flow_capture_ring = ring;
    flow_capture_head = 0U;
    flow_capture_total = 0U;
    flow_capture_target = target;
    flow_capture_last_received_ms = 0U;
    flow_capture_active = 1U;
}

uint16_t SVC_FlowCapture_Arm(uint16_t frames)
{
    if ((frames == 0U) || (frames > SVC_FLOW_CAPTURE_MAX_FRAMES)) {
        frames = (uint16_t)SVC_FLOW_CAPTURE_MAX_FRAMES;
    }
    flow_capture_restart(0U, frames);
    return frames;
}

void SVC_FlowCapture_ArmRing(void)
{
    flow_capture_restart(1U, (uint16_t)SVC_FLOW_CAPTURE_MAX_FRAMES);
}

void SVC_FlowCapture_Stop(void)
{
    flow_capture_active = 0U;
}

void SVC_FlowCapture_Push(const SVC_FLOW_NAV_Sample *sample)
{
    SVC_FlowCaptureFrame *frame;
    uint16_t slot;

    if ((flow_capture_active == 0U) || (sample == NULL) ||
        (sample->flow_received_ms == 0U) ||
        (sample->flow_received_ms == flow_capture_last_received_ms)) {
        return;
    }
    if (flow_capture_ring != 0U) {
        slot = flow_capture_head;
    } else {
        if (flow_capture_total >= flow_capture_target) {
            flow_capture_active = 0U;
            return;
        }
        slot = (uint16_t)flow_capture_total;
    }
    flow_capture_last_received_ms = sample->flow_received_ms;
    frame = &flow_capture_frames[slot];
    frame->sensor_time_ms = sample->sensor_time_ms;
    frame->received_ms = sample->flow_received_ms;
    frame->vel_x = sample->flow_vel_x;
    frame->vel_y = sample->flow_vel_y;
    frame->distance_mm = (sample->distance_mm > 0xFFFFUL) ? 0xFFFFU : (uint16_t)sample->distance_mm;
    frame->quality = sample->flow_quality;
    frame->flags = (uint8_t)(((sample->flow_valid != 0U) ? SVC_FLOW_CAPTURE_FLAG_FLOW_VALID : 0U) |
                             ((sample->frame_valid != 0U) ? SVC_FLOW_CAPTURE_FLAG_FRAME_VALID : 0U) |
                             ((sample->distance_valid != 0U) ? SVC_FLOW_CAPTURE_FLAG_DISTANCE_VALID : 0U));
    flow_capture_head = (uint16_t)((slot + 1U) % SVC_FLOW_CAPTURE_MAX_FRAMES);
    flow_capture_total = flow_capture_total + 1U;
    if ((flow_capture_ring == 0U) && (flow_capture_total >= flow_capture_target)) {
        flow_capture_active = 0U;
    }
}

uint8_t SVC_FlowCapture_Active(void)
{
    return flow_capture_active;
}

uint8_t SVC_FlowCapture_Ring(void)
{
    return flow_capture_ring;
}

uint16_t SVC_FlowCapture_Count(void)
{
    const uint32_t total = flow_capture_total;
    return (uint16_t)((total > SVC_FLOW_CAPTURE_MAX_FRAMES) ? SVC_FLOW_CAPTURE_MAX_FRAMES : total);
}

uint32_t SVC_FlowCapture_Total(void)
{
    return flow_capture_total;
}

uint16_t SVC_FlowCapture_Target(void)
{
    return flow_capture_target;
}

uint8_t SVC_FlowCapture_Get(uint16_t index, SVC_FlowCaptureFrame *out)
{
    uint32_t slot = index;

    if ((out == NULL) || (index >= SVC_FlowCapture_Count())) {
        return 0U;
    }
    /* 环形且已绕圈：第 0 帧是最老的那帧，即下一个要写的位置。 */
    if ((flow_capture_ring != 0U) && (flow_capture_total > SVC_FLOW_CAPTURE_MAX_FRAMES)) {
        slot = ((uint32_t)flow_capture_head + index) % SVC_FLOW_CAPTURE_MAX_FRAMES;
    }
    *out = flow_capture_frames[slot];
    return 1U;
}
