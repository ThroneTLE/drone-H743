#include "bsp_imu.h"
#include "bsp_board.h"

#include "main.h"
#include "spi.h"

#include <string.h>

static DRV_IMU_Device imu_dev;
static BSP_IMU_Diag   imu_diag;
static uint8_t        imu_initialized;

static void BSP_IMU_ConfigureSpiMode(uint32_t polarity, uint32_t phase)
{
    if ((hspi2.Init.CLKPolarity == polarity) &&
        (hspi2.Init.CLKPhase == phase) &&
        (hspi2.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256)) {
        return;
    }

    (void)HAL_SPI_DeInit(&hspi2);
    hspi2.Init.CLKPolarity = polarity;
    hspi2.Init.CLKPhase = phase;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    (void)HAL_SPI_Init(&hspi2);
}

DRV_IMU_Status BSP_IMU_Init(void)
{
    DRV_IMU_Config config;
    DRV_IMU_Status status;

    if (imu_initialized != 0U) { return DRV_IMU_OK; }

    if (imu_diag.valid == 0U) {
        memset(&imu_diag, 0, sizeof(imu_diag));
        imu_diag.best_mode = 3U;
        imu_diag.best_header = 1U;
        imu_diag.valid = 1U;
    }

    BSP_IMU_ConfigureSpiMode(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);

    DRV_IMU_DefaultConfig(&config);
    config.accel_range = DRV_IMU_ACCEL_RANGE_4G;
    config.gyro_range  = DRV_IMU_GYRO_RANGE_1000DPS;
    config.accel_odr   = DRV_IMU_ODR_100HZ;
    config.gyro_odr    = DRV_IMU_ODR_100HZ;
    config.soft_reset_on_init = false;

    status = DRV_IMU_Init(&imu_dev, BSP_Board_GetImuBus(), &config);
    if (status == DRV_IMU_OK) { imu_initialized = 1U; }

    return status;
}

DRV_IMU_Status BSP_IMU_ReadRaw(DRV_IMU_RawData *raw)
{
    if (imu_initialized == 0U) { return DRV_IMU_ERROR; }
    return DRV_IMU_ReadRaw(&imu_dev, raw);
}

DRV_IMU_Status BSP_IMU_ReadScaled(DRV_IMU_ScaledData *scaled)
{
    if (imu_initialized == 0U) { return DRV_IMU_ERROR; }
    return DRV_IMU_ReadScaled(&imu_dev, scaled);
}

DRV_IMU_Status BSP_IMU_IsDataReady(bool *ready)
{
    if (imu_initialized == 0U) { return DRV_IMU_ERROR; }
    return DRV_IMU_IsDataReady(&imu_dev, ready);
}

uint8_t BSP_IMU_GetWhoAmI(void)             { return imu_dev.who_am_i; }
void BSP_IMU_GetDiag(BSP_IMU_Diag *diag)   { if (diag != NULL) { *diag = imu_diag; } }
const DRV_IMU_Device *BSP_IMU_GetDevice(void) { return &imu_dev; }

void BSP_IMU_Invalidate(void)
{
    memset(&imu_dev, 0, sizeof(imu_dev));
    imu_initialized = 0U;
}
