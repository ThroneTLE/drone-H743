/*
 * `FLOWCAP` —— 光流逐帧抓取诊断（只记录，不改导航/控制；上锁、解锁都可用）。
 *
 * 2026-10-01：钢尺实测 5 点中值滤波比原始帧积分少 7–20% 且随运动而变，要看原始帧分布才能定怎么改。
 *
 *   FLOWCAP START [n]   → FLOWCAP state=armed target=<n>     抓接下来 n 帧新光流（默认/上限 4096）
 *   FLOWCAP START RING  → FLOWCAP state=ring max=4096         环形一直记最近 4096 帧（约 41 s）
 *   FLOWCAP?            → FLOWCAP active=<0|1> ring=<0|1> count=<c> total=<t> target=<n>
 *   FLOWCAP DUMP <i>    → 先冻结（环形边记边读会被覆盖），再从第 i 帧（最老=0）起最多 4 行、
 *                         每行 8 帧，末行 FLOWCAP END next=<i'> count=<c>
 *     （RING/冻结都挂在 START/DUMP 两个词下，AI 接口白名单不用再改、地面站不用重启。）
 *       FLOWCAP D i=<首帧序号> t0=<传感器ms> r0=<收帧ms> d=<首帧测距mm>
 *                 f=<dt传感器>,<dt收帧>,<vx>,<vy>,<质量>,<有效位>;...   （dt 相对本行首帧）
 *     vx/vy 是 FLU 原始计数（cm/s@1m，未中值、未转动补偿）；有效位 1=flow 2=frame 4=distance。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "svc_flow_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FLOWCAP_FRAMES_PER_LINE 8U
#define FLOWCAP_LINES_PER_DUMP  4U

static void flowcap_report(void)
{
    APP_Control_QueueText("FLOWCAP active=%u ring=%u count=%u total=%lu target=%u\r\n",
                          (unsigned int)SVC_FlowCapture_Active(),
                          (unsigned int)SVC_FlowCapture_Ring(),
                          (unsigned int)SVC_FlowCapture_Count(),
                          (unsigned long)SVC_FlowCapture_Total(),
                          (unsigned int)SVC_FlowCapture_Target());
}

static uint16_t flowcap_dump_line(uint16_t start)
{
    SVC_FlowCaptureFrame first;
    SVC_FlowCaptureFrame frame;
    char text[192];   /* 行头约 55 字符 + 本段 ≤ 255 */
    size_t used;
    uint16_t n = 0U;

    if (SVC_FlowCapture_Get(start, &first) == 0U) {
        return 0U;
    }
    used = 0U;
    text[0] = '\0';
    while ((n < FLOWCAP_FRAMES_PER_LINE) && (SVC_FlowCapture_Get((uint16_t)(start + n), &frame) != 0U)) {
        int written = snprintf(&text[used], sizeof(text) - used, "%ld,%ld,%d,%d,%u,%u;",
                               (long)(int32_t)(frame.sensor_time_ms - first.sensor_time_ms),
                               (long)(int32_t)(frame.received_ms - first.received_ms),
                               (int)frame.vel_x, (int)frame.vel_y,
                               (unsigned int)frame.quality, (unsigned int)frame.flags);
        if ((written < 0) || ((size_t)written >= (sizeof(text) - used))) {
            break;
        }
        used += (size_t)written;
        n++;
    }
    APP_Control_QueueText("FLOWCAP D i=%u t0=%lu r0=%lu d=%u f=%s\r\n",
                          (unsigned int)start, (unsigned long)first.sensor_time_ms,
                          (unsigned long)first.received_ms, (unsigned int)first.distance_mm, text);
    return n;
}

uint8_t app_control_handle_flowcap(char **tokens, uint32_t count)
{
    if ((tokens == NULL) || (count == 0U) || (tokens[0] == NULL)) {
        return 0U;
    }
    if (strcmp(tokens[0], "FLOWCAP?") == 0) {
        flowcap_report();
        return 1U;
    }
    if (strcmp(tokens[0], "FLOWCAP") != 0) {
        return 0U;
    }
    if ((count >= 3U) && (strcmp(tokens[1], "START") == 0) && (strcmp(tokens[2], "RING") == 0)) {
        SVC_FlowCapture_ArmRing();
        APP_Control_QueueText("FLOWCAP state=ring max=%u\r\n", (unsigned int)SVC_FLOW_CAPTURE_MAX_FRAMES);
        return 1U;
    }
    if ((count >= 2U) && (strcmp(tokens[1], "START") == 0)) {
        unsigned long frames = (count >= 3U) ? strtoul(tokens[2], NULL, 10) : 0UL;
        uint16_t target = SVC_FlowCapture_Arm((frames > 0xFFFFUL) ? 0U : (uint16_t)frames);
        APP_Control_QueueText("FLOWCAP state=armed target=%u\r\n", (unsigned int)target);
        return 1U;
    }
    if ((count >= 3U) && (strcmp(tokens[1], "DUMP") == 0)) {
        unsigned long start = strtoul(tokens[2], NULL, 10);
        uint16_t next = (start > 0xFFFFUL) ? 0xFFFFU : (uint16_t)start;
        if ((SVC_FlowCapture_Ring() != 0U) && (SVC_FlowCapture_Active() != 0U)) {
            SVC_FlowCapture_Stop();
        }
        for (uint32_t line = 0U; line < FLOWCAP_LINES_PER_DUMP; ++line) {
            uint16_t printed = flowcap_dump_line(next);
            if (printed == 0U) {
                break;
            }
            next = (uint16_t)(next + printed);
        }
        APP_Control_QueueText("FLOWCAP END next=%u count=%u\r\n",
                              (unsigned int)next, (unsigned int)SVC_FlowCapture_Count());
        return 1U;
    }
    APP_Control_QueueText("ERR usage FLOWCAP START [n|RING] | FLOWCAP? | FLOWCAP DUMP <i>\r\n");
    return 1U;
}
