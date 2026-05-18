#ifndef APP_ELRS_H
#define APP_ELRS_H

#include <stdint.h>

#include "drv_elrs.h"

#ifdef __cplusplus
extern "C" {
#endif

void APP_ELRS_Init(void);

/*
 * 消费 DMA 缓冲区新字节，驱动 CRSF 协议解析。
 * 需要从高频任务（如 StabilizerTask，~1kHz）周期性调用。
 * 执行时间极其短，无阻塞。
 */
void APP_ELRS_Step(void);

/* RC 通道值，us 范围 [800, 2200] */
void APP_ELRS_GetChannels(uint16_t us_out[CRSF_CHANNEL_COUNT]);

/* ---- 遥测发送（非阻塞 DMA）---- */

/* 姿态: pitch/roll/yaw × 10000 弧度 */
void APP_ELRS_SendTelemetryAttitude(int16_t pitch_rad_x10000,
                                    int16_t roll_rad_x10000,
                                    int16_t yaw_rad_x10000);

/* 气压计高度: 分米 */
void APP_ELRS_SendTelemetryBaro(int32_t altitude_dm);

/* 电池: 电压 0.1V, 电流 0.1A, 容量 mAh, 剩余 % */
void APP_ELRS_SendTelemetryBattery(uint16_t voltage_dv,
                                   uint16_t current_da,
                                   uint32_t capacity_mah,
                                   uint8_t remaining_pct);

/* GPS: lat/lon ×1e7 °, 速度 0.1 km/h, 航向 0.01 °, 高度 m+1000, 卫星数 */
void APP_ELRS_SendTelemetryGps(int32_t lat_e7, int32_t lon_e7,
                               uint16_t speed_kmh_x10,
                               uint16_t heading_deg_x100,
                               uint16_t altitude_m, uint8_t satellites);

/* ---- HAL 回调（ISR 上下文，仅做事件标记）---- */
void APP_ELRS_OnRxEvent(uint16_t size);
void APP_ELRS_OnTxComplete(void);
void APP_ELRS_OnError(void);

/* ---- 诊断 ---- */
uint32_t APP_ELRS_GetRcFrames(void);
uint32_t APP_ELRS_GetCrcErrors(void);
const DRV_ELRS_LinkStats *APP_ELRS_GetLinkStats(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ELRS_H */
