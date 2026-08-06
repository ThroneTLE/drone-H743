#include "app.h"
#include "app_elrs.h"
#include "app_imu_capture.h"

void APP_Init(void)
{
    APP_ELRS_Init();
    APP_IMU_Capture_Init();
}
