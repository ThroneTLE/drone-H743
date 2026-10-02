#ifndef SVC_FLOW_CAPTURE_H
#define SVC_FLOW_CAPTURE_H

/*
 * 光流逐帧抓取（诊断）。2026-10-01：钢尺实测 5 点中值滤波比原始帧积分少 7–20%，且随运动
 * 而变——对正常帧中值不该有偏，说明原始帧分布本身是偏的。要看清原始帧长什么样，才能决定
 * 滤波怎么改。
 *
 * 用法：Arm(n) 一次性记接下来的 n 帧新光流，抓满自动停；ArmRing() 环形一直记最近
 * MAX 帧（100 Hz 约 41 s），Stop() 冻结后按 最老→最新 读。帧按 flow_received_ms 去重
 * （同一帧重复推入只记一次），原样记下：传感器时钟、本板收帧时刻、FLU 计数、质量、测距、
 * 有效位。只做记录，不改任何导航/控制量。纯数据模块，无 HAL。
 */

#include <stdint.h>

#include "svc_flow_nav.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SVC_FLOW_CAPTURE_MAX_FRAMES 4096U

#define SVC_FLOW_CAPTURE_FLAG_FLOW_VALID     0x01U
#define SVC_FLOW_CAPTURE_FLAG_FRAME_VALID    0x02U
#define SVC_FLOW_CAPTURE_FLAG_DISTANCE_VALID 0x04U

typedef struct {
    uint32_t sensor_time_ms;
    uint32_t received_ms;
    int16_t  vel_x;
    int16_t  vel_y;
    uint16_t distance_mm;
    uint8_t  quality;
    uint8_t  flags;
} SVC_FlowCaptureFrame;

/* 清空并开始抓接下来的 frames 帧（0 或超过上限按上限）。返回实际目标帧数。 */
uint16_t SVC_FlowCapture_Arm(uint16_t frames);
void SVC_FlowCapture_ArmRing(void);
void SVC_FlowCapture_Stop(void);
void SVC_FlowCapture_Push(const SVC_FLOW_NAV_Sample *sample);
uint8_t SVC_FlowCapture_Active(void);
uint8_t SVC_FlowCapture_Ring(void);
uint16_t SVC_FlowCapture_Count(void);     /* 可读帧数（环形绕圈后恒为 MAX） */
uint32_t SVC_FlowCapture_Total(void);     /* 本次累计记过的帧数 */
uint16_t SVC_FlowCapture_Target(void);
/* index < Count() 才返回 1。一次性模式抓取中也可读；环形模式须先 Stop。 */
uint8_t SVC_FlowCapture_Get(uint16_t index, SVC_FlowCaptureFrame *out);

#ifdef __cplusplus
}
#endif

#endif /* SVC_FLOW_CAPTURE_H */
