#include "app_imu.h"

#include "app_messages.h"
#include "app_tasks.h"
#include "bsp_imu.h"
#include "cmsis_os2.h"

#include <limits.h>

typedef struct {
    uint32_t              sample_count;
    BSP_ICM42688_Status   last_status;
    BSP_ICM42688_Status   last_error;
    BSP_ICM42688_InitStage init_stage;
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
    imu_state.last_error = BSP_ICM42688_ERROR;
    imu_state.init_stage = BSP_ICM42688_INIT_STAGE_NONE;
    imu_state.who_am_i = 0U;
    imu_state.initialized = 0U;
}

void APP_IMU_Task_Step(void)
{
    bool ready = false;

    if (imu_state.initialized == 0U) {
        imu_state.last_status = BSP_IMU_Init();
        if (imu_state.last_status != BSP_ICM42688_OK) {
            const BSP_ICM42688_Device *dev = BSP_IMU_GetDevice();
            if (dev != 0) {
                imu_state.who_am_i = dev->who_am_i;
                imu_state.init_stage = dev->init_stage;
                imu_state.last_error = dev->last_error;
            } else {
                imu_state.last_error = imu_state.last_status;
            }
            osDelay(50);
            return;
        }

        imu_state.who_am_i = BSP_IMU_GetWhoAmI();
        imu_state.init_stage = BSP_ICM42688_INIT_STAGE_READY;
        imu_state.last_error = BSP_ICM42688_OK;
        imu_state.initialized = 1U;
        osDelay(10);
        return;
    }

    imu_state.last_status = BSP_IMU_IsDataReady(&ready);
    if (imu_state.last_status != BSP_ICM42688_OK) {
        BSP_IMU_Invalidate();
        imu_state.initialized = 0U;
        imu_state.init_stage = BSP_ICM42688_INIT_STAGE_NONE;
        imu_state.last_error = imu_state.last_status;
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
            imu_state.init_stage = BSP_ICM42688_INIT_STAGE_NONE;
            imu_state.last_error = imu_state.last_status;
            osDelay(20);
            return;
        }
    }

    osDelay(1);
}

void APP_IMU_GetStatus(APP_IMU_Status *status)
{
    BSP_IMU_Diag diag;

    if (status == 0) {
        return;
    }

    BSP_IMU_GetDiag(&diag);

    status->initialized = imu_state.initialized;
    status->who_am_i = imu_state.who_am_i;
    status->init_stage = (uint8_t)imu_state.init_stage;
    status->last_status = (int32_t)imu_state.last_status;
    status->last_error = (int32_t)imu_state.last_error;
    status->sample_count = imu_state.sample_count;
    status->temperature_cdeg = APP_IMU_ScaleToInt16(imu_state.last_sample.temperature_c, 100.0f);
    status->accel_x_mg = APP_IMU_ScaleToInt16(imu_state.last_sample.accel_x_g, 1000.0f);
    status->accel_y_mg = APP_IMU_ScaleToInt16(imu_state.last_sample.accel_y_g, 1000.0f);
    status->accel_z_mg = APP_IMU_ScaleToInt16(imu_state.last_sample.accel_z_g, 1000.0f);
    status->gyro_x_mdps = APP_IMU_ScaleToInt32(imu_state.last_sample.gyro_x_dps, 1000.0f);
    status->gyro_y_mdps = APP_IMU_ScaleToInt32(imu_state.last_sample.gyro_y_dps, 1000.0f);
    status->gyro_z_mdps = APP_IMU_ScaleToInt32(imu_state.last_sample.gyro_z_dps, 1000.0f);
    status->diag_mode0_tokmas = diag.mode0_tokmas;
    status->diag_mode0_msb = diag.mode0_msb;
    status->diag_mode0_bit0 = diag.mode0_bit0;
    status->diag_mode3_tokmas = diag.mode3_tokmas;
    status->diag_mode3_msb = diag.mode3_msb;
    status->diag_mode3_bit0 = diag.mode3_bit0;
    status->diag_burst_m0_b0_1 = diag.burst_m0_b0_1;
    status->diag_burst_m0_b0_2 = diag.burst_m0_b0_2;
    status->diag_burst_m0_b0_3 = diag.burst_m0_b0_3;
    status->diag_burst_m0_b0_4 = diag.burst_m0_b0_4;
    status->diag_burst_m3_tok_1 = diag.burst_m3_tok_1;
    status->diag_burst_m3_tok_2 = diag.burst_m3_tok_2;
    status->diag_burst_m3_tok_3 = diag.burst_m3_tok_3;
    status->diag_burst_m3_tok_4 = diag.burst_m3_tok_4;
    status->diag_best_mode = diag.best_mode;
    status->diag_best_header = diag.best_header;
    status->diag_valid = diag.valid;
}
