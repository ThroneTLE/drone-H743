#ifndef APP_THRUST_LUT_H
#define APP_THRUST_LUT_H

#include <stdint.h>

/* 启动时调用一次（任务开始前）：把推力台查补表装进双桨控制器的推力换算。 */
void APP_ThrustLut_Init(void);
/* messageTask 每拍调用：更新电量电压估计（带载电压 + 压降补回，低通）。 */
void APP_ThrustLut_Step(void);
/* 当前电量电压估计，0 = 还没有可信估计（此时不做电压补偿）。 */
float APP_ThrustLut_ChargeVoltage(void);
/* Read-only readiness for experiment provenance, not a flight arming gate. */
uint8_t APP_ThrustLut_IsFresh(void);

/* 飞控实际使用的换算，charge_v 显式传入以便对拍；charge_v <= 0 表示不补偿。 */
uint16_t APP_ThrustLut_PulseForMotorThrust(float thrust_n, float charge_v);
float APP_ThrustLut_TotalThrustForPulse(uint16_t pulse_us, float charge_v);
/* 上下桨一起换算：保持两桨油门差，整体平移使二维表合推力 = upper_n + lower_n。返回平移量（等效油门 %）。 */
float APP_ThrustLut_PulsesForPair(float upper_n, float lower_n, float charge_v,
                                  uint16_t *upper_us, uint16_t *lower_us);

#endif /* APP_THRUST_LUT_H */
