#include "app.h"

#include "app_elrs.h"
#include "app_imu_capture.h"
#include "app_sensor.h"
#include "app_thrust_lut.h"
#include "bsp_imu.h"

void APP_Init(void)
{
    /*
     * 把 DRDY 中断接到 Sensor 模块上。
     *
     * 注册而不是让 App 去实现 HAL 的 `HAL_GPIO_EXTI_Callback()`：DRDY 落在哪个
     * 引脚是板级知识（本次移植它就从 PC0/EXTI0 散成了 PC15 与 PB7），比对与
     * 屏蔽都留在 BSP，App 只说"到了之后叫我"。
     */
    BSP_IMU_SetDrdyHandler(APP_IMU_OnDataReady);

    /* 任务开始前装好推力查补表，控制器第一拍就用它（见 app_thrust_lut.c）。 */
    APP_ThrustLut_Init();
    APP_ELRS_Init();
    APP_IMU_Capture_Init();
}
