#include "bsp_imu.h"

#include "main.h"
#include "spi.h"

#include <string.h>

static BSP_ICM42688_Device imu_dev;
static uint8_t imu_initialized;

BSP_ICM42688_Status BSP_IMU_Init(void)
{
    BSP_ICM42688_Bus bus = {
        .hspi = &hspi2,
        .cs_port = IMU_CS_GPIO_Port,
        .cs_pin = IMU_CS_Pin,
        .timeout_ms = 100U,
        .delay_ms = HAL_Delay,
    };
    BSP_ICM42688_Config config;
    BSP_ICM42688_Status status;

    if (imu_initialized != 0U) {
        return BSP_ICM42688_OK;
    }

    BSP_ICM42688_DefaultConfig(&config);
    config.accel_range = BSP_ICM42688_ACCEL_RANGE_4G;
    config.gyro_range = BSP_ICM42688_GYRO_RANGE_2000DPS;
    config.accel_odr = BSP_ICM42688_ODR_1KHZ;
    config.gyro_odr = BSP_ICM42688_ODR_1KHZ;

    status = BSP_ICM42688_Init(&imu_dev, &bus, &config);
    if (status == BSP_ICM42688_OK) {
        imu_initialized = 1U;
    }

    return status;
}

BSP_ICM42688_Status BSP_IMU_ReadRaw(BSP_ICM42688_RawData *raw)
{
    if (imu_initialized == 0U) {
        return BSP_ICM42688_ERROR;
    }

    return BSP_ICM42688_ReadRaw(&imu_dev, raw);
}

BSP_ICM42688_Status BSP_IMU_ReadScaled(BSP_ICM42688_ScaledData *scaled)
{
    if (imu_initialized == 0U) {
        return BSP_ICM42688_ERROR;
    }

    return BSP_ICM42688_ReadScaled(&imu_dev, scaled);
}

BSP_ICM42688_Status BSP_IMU_IsDataReady(bool *ready)
{
    if (imu_initialized == 0U) {
        return BSP_ICM42688_ERROR;
    }

    return BSP_ICM42688_IsDataReady(&imu_dev, ready);
}

uint8_t BSP_IMU_GetWhoAmI(void)
{
    return imu_dev.who_am_i;
}

const BSP_ICM42688_Device *BSP_IMU_GetDevice(void)
{
    return &imu_dev;
}

void BSP_IMU_Invalidate(void)
{
    memset(&imu_dev, 0, sizeof(imu_dev));
    imu_initialized = 0U;
}
