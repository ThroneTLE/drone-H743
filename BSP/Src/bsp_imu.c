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
        (hspi2.Init.CLKPhase == phase)) {
        return;
    }

    (void)HAL_SPI_DeInit(&hspi2);
    hspi2.Init.CLKPolarity = polarity;
    hspi2.Init.CLKPhase = phase;
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
    /*
     * +-16 g, not +-4 g. Replaying log/flightlog_20260726_021748.csv and
     * log/flightlog_20260727_045138.csv showed raw_accel_* hitting the int16
     * rail (32767 = 4 g) with p99.9 at 78-85 % of full scale, so coaxial-rotor
     * vibration was clipping the accelerometer in powered flight. Clipping is
     * unrecoverable and asymmetric, which biases the very mean that the gravity
     * direction is extracted from: raw accel norm averaged 1.59-2.04 g at hover
     * instead of 1.0. At +-16 g the 2048 LSB/g resolution still resolves about
     * 0.03 deg of tilt, far finer than the errors being chased.
     */
    config.accel_range = DRV_IMU_ACCEL_RANGE_16G;
    config.gyro_range  = DRV_IMU_GYRO_RANGE_1000DPS;
    config.accel_odr   = DRV_IMU_ODR_1KHZ;
    config.gyro_odr    = DRV_IMU_ODR_1KHZ;
    /*
     * Anti-alias filter, previously left at the power-on default (i.e. wide
     * open). At a 1 kHz ODR anything above the 500 Hz Nyquist folds down into
     * the attitude band, where no software filter can remove it.
     *
     * Airframe: 9050 two-blade coaxial rotors on KV1300. Blade passage is twice
     * shaft speed, so hover sits near 150-300 Hz and the second harmonic lands
     * around 300-600 Hz — straddling Nyquist. 213 Hz keeps the fundamental
     * observable for the rate loop while attenuating the harmonics that would
     * otherwise alias. Revisit once a spectrum capture pins the real hover tone;
     * this is a reasoned starting point, not a measured optimum.
     */
    config.accel_aaf_hz = 213U;
    config.gyro_aaf_hz  = 213U;
    config.soft_reset_on_init = true;

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
