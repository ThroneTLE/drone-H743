#ifndef BSP_IMU_RATE_H
#define BSP_IMU_RATE_H

#include <stdint.h>

/*
 * 当前选中的 IMU 实际编程进去的陀螺名义输出率 [Hz]：BMI088 1000，BMI270 800，
 * 未初始化或未知芯片返回 0。
 *
 * 单独成头、只依赖 <stdint.h>：bsp_imu.h 经 drv_imu.h 拖着 main.h（HAL），
 * 转速陷波的策略模块要在宿主上编译，不能包含它。实现在 bsp_imu.c。
 * 这是名义值；芯片振荡器的误差由 app_rpm_notch.c 按 IMU 时间戳估出来。
 */
uint16_t BSP_IMU_GetGyroOdrHz(void);

#endif /* BSP_IMU_RATE_H */
