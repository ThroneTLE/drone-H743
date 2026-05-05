#include "app_imu.h"

#include "app_messages.h"
#include "app_tasks.h"
#include "bsp_imu.h"
#include "cmsis_os2.h"

#include <limits.h>

typedef struct {
    uint32_t              sample_count;
    BSP_ICM42688_Status   last_status;
    BSP_ICM42688_ScaledData last_sample;
    uint8_t               who_am_i;
    uint8_t               initialized;
} APP_IMU_State;

static APP_IMU_State imu_state;

static int16_t APP_IMU_ScaleToInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > (float)INT16_MAX) {
        return INT16_MAX;
    }

    if (scaled < (float)INT16_MIN) {
        return INT16_MIN;
    }

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int16_t)scaled;
}

static int32_t APP_IMU_ScaleToInt32(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > (float)INT32_MAX) {
        return INT32_MAX;
    }

    if (scaled < (float)INT32_MIN) {
        return INT32_MIN;
    }

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
}

static void APP_IMU_PublishSample(void)
{
    APP_IMU_SampleMessage sample;

    if (imuSampleQueueHandle == 0) {
        return;
    }

    sample.sample_count = imu_state.sample_count;
    sample.who_am_i = imu_state.who_am_i;
    sample.reserved = 0U;
    sample.temperature_cdeg = APP_IMU_ScaleToInt16(imu_state.last_sample.temperature_c, 100.0f);
    sample.accel_x_mg = APP_IMU_ScaleToInt16(imu_state.last_sample.accel_x_g, 1000.0f);
    sample.accel_y_mg = APP_IMU_ScaleToInt16(imu_state.last_sample.accel_y_g, 1000.0f);
    sample.accel_z_mg = APP_IMU_ScaleToInt16(imu_state.last_sample.accel_z_g, 1000.0f);
    sample.gyro_x_mdps = APP_IMU_ScaleToInt32(imu_state.last_sample.gyro_x_dps, 1000.0f);
    sample.gyro_y_mdps = APP_IMU_ScaleToInt32(imu_state.last_sample.gyro_y_dps, 1000.0f);
    sample.gyro_z_mdps = APP_IMU_ScaleToInt32(imu_state.last_sample.gyro_z_dps, 1000.0f);

    if (osMessageQueuePut(imuSampleQueueHandle, &sample, 0U, 0U) != osOK) {
        APP_IMU_SampleMessage dropped;

        (void)osMessageQueueGet(imuSampleQueueHandle, &dropped, 0U, 0U);
        (void)osMessageQueuePut(imuSampleQueueHandle, &sample, 0U, 0U);
    }
}

void APP_IMU_Task_Init(void)
{
    BSP_IMU_Invalidate();
    imu_state.sample_count = 0U;
    imu_state.last_status = BSP_ICM42688_ERROR;
    imu_state.who_am_i = 0U;
    imu_state.initialized = 0U;
}

void APP_IMU_Task_Step(void)
{
    bool ready = false;

    if (imu_state.initialized == 0U) {
        imu_state.last_status = BSP_IMU_Init();
        if (imu_state.last_status != BSP_ICM42688_OK) {
            osDelay(50);
            return;
        }

        imu_state.who_am_i = BSP_IMU_GetWhoAmI();
        imu_state.initialized = 1U;
        osDelay(10);
        return;
    }

    imu_state.last_status = BSP_IMU_IsDataReady(&ready);
    if (imu_state.last_status != BSP_ICM42688_OK) {
        BSP_IMU_Invalidate();
        imu_state.initialized = 0U;
        osDelay(20);
        return;
    }

    if (ready) {
        imu_state.last_status = BSP_IMU_ReadScaled(&imu_state.last_sample);
        if (imu_state.last_status == BSP_ICM42688_OK) {
            imu_state.sample_count++;
            APP_IMU_PublishSample();
        } else {
            BSP_IMU_Invalidate();
            imu_state.initialized = 0U;
            osDelay(20);
            return;
        }
    }

    osDelay(1);
}
