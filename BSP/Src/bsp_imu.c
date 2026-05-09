#include "bsp_imu.h"

#include "main.h"
#include "spi.h"

#include <string.h>

#define BSP_IMU_WHO_AM_I_REG        0x75U
#define BSP_IMU_HEADER_TOKMAS_READ  0U
#define BSP_IMU_HEADER_MSB_READ     1U
#define BSP_IMU_HEADER_BIT0_FIRST   2U
#define BSP_IMU_SOFT_SPI_WAVEFORM_TEST 0U

static BSP_ICM42688_Device imu_dev;
static BSP_IMU_Diag imu_diag;
static uint8_t imu_initialized;

#if BSP_IMU_SOFT_SPI_WAVEFORM_TEST
static void BSP_IMU_TestDelay(void)
{
    for (volatile uint32_t i = 0U; i < 8000U; ++i) {
        __NOP();
    }
}

static void BSP_IMU_TestWritePins(uint8_t cs, uint8_t clk, uint8_t mosi)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, cs != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, clk != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, mosi != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void BSP_IMU_SendSoftSpiTestFrame(void)
{
    const uint8_t whoami_read[] = {0xF5U, 0x00U};

    BSP_IMU_TestWritePins(1U, 0U, 0U);
    HAL_Delay(1U);

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    BSP_IMU_TestDelay();
    for (uint32_t i = 0U; i < (sizeof(whoami_read) / sizeof(whoami_read[0])); ++i) {
        for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,
                              ((whoami_read[i] & mask) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            BSP_IMU_TestDelay();
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            BSP_IMU_TestDelay();
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            BSP_IMU_TestDelay();
        }
    }
    BSP_IMU_TestWritePins(1U, 0U, 0U);
    HAL_Delay(20U);
}
#endif

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

static uint8_t BSP_IMU_ReadWhoAmIProbe(uint8_t header_mode)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0U, 0U};

    if (header_mode == BSP_IMU_HEADER_BIT0_FIRST) {
        tx[0] = (uint8_t)(((BSP_IMU_WHO_AM_I_REG & 0x7FU) << 3U) | 0x04U);
    } else if (header_mode == BSP_IMU_HEADER_MSB_READ) {
        tx[0] = (uint8_t)(BSP_IMU_WHO_AM_I_REG | 0x80U);
    } else {
        tx[0] = (uint8_t)((BSP_IMU_WHO_AM_I_REG << 1U) | 0x01U);
    }
    tx[1] = 0U;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 100U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);

    return rx[1];
}

static void BSP_IMU_ReadBurstProbe(uint8_t header_mode, uint8_t *out_bytes, uint32_t count)
{
    uint8_t tx[5] = {0U};
    uint8_t rx[5] = {0U};
    uint32_t frames = (count < 4U) ? count : 4U;

    if (out_bytes == NULL) {
        return;
    }

    if (header_mode == BSP_IMU_HEADER_BIT0_FIRST) {
        tx[0] = (uint8_t)(((BSP_IMU_WHO_AM_I_REG & 0x7FU) << 3U) | 0x04U);
    } else if (header_mode == BSP_IMU_HEADER_MSB_READ) {
        tx[0] = (uint8_t)(BSP_IMU_WHO_AM_I_REG | 0x80U);
    } else {
        tx[0] = (uint8_t)((BSP_IMU_WHO_AM_I_REG << 1U) | 0x01U);
    }

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(frames + 1U), 100U);
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1U);

    for (uint32_t i = 0U; i < frames; ++i) {
        out_bytes[i] = rx[i + 1U];
    }
}

__attribute__((unused)) static void BSP_IMU_RunDiag(void)
{
    uint8_t burst[4] = {0U};

    memset(&imu_diag, 0, sizeof(imu_diag));

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(320U);

    BSP_IMU_ConfigureSpiMode(SPI_POLARITY_LOW, SPI_PHASE_1EDGE);
    imu_diag.mode0_tokmas = BSP_IMU_ReadWhoAmIProbe(BSP_IMU_HEADER_TOKMAS_READ);
    imu_diag.mode0_msb = BSP_IMU_ReadWhoAmIProbe(BSP_IMU_HEADER_MSB_READ);
    imu_diag.mode0_bit0 = BSP_IMU_ReadWhoAmIProbe(BSP_IMU_HEADER_BIT0_FIRST);
    BSP_IMU_ReadBurstProbe(BSP_IMU_HEADER_BIT0_FIRST, burst, 4U);
    imu_diag.burst_m0_b0_1 = burst[0];
    imu_diag.burst_m0_b0_2 = burst[1];
    imu_diag.burst_m0_b0_3 = burst[2];
    imu_diag.burst_m0_b0_4 = burst[3];

    BSP_IMU_ConfigureSpiMode(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);
    imu_diag.mode3_tokmas = BSP_IMU_ReadWhoAmIProbe(BSP_IMU_HEADER_TOKMAS_READ);
    imu_diag.mode3_msb = BSP_IMU_ReadWhoAmIProbe(BSP_IMU_HEADER_MSB_READ);
    imu_diag.mode3_bit0 = BSP_IMU_ReadWhoAmIProbe(BSP_IMU_HEADER_BIT0_FIRST);
    BSP_IMU_ReadBurstProbe(BSP_IMU_HEADER_TOKMAS_READ, burst, 4U);
    imu_diag.burst_m3_tok_1 = burst[0];
    imu_diag.burst_m3_tok_2 = burst[1];
    imu_diag.burst_m3_tok_3 = burst[2];
    imu_diag.burst_m3_tok_4 = burst[3];

    if (imu_diag.mode0_tokmas == BSP_ICM42688_WHO_AM_I_VALUE) {
        imu_diag.best_mode = 0U;
        imu_diag.best_header = BSP_IMU_HEADER_MSB_READ;
    } else if (imu_diag.mode0_msb == BSP_ICM42688_WHO_AM_I_VALUE) {
        imu_diag.best_mode = 0U;
        imu_diag.best_header = BSP_IMU_HEADER_MSB_READ;
    } else if (imu_diag.mode0_bit0 == BSP_ICM42688_WHO_AM_I_VALUE) {
        imu_diag.best_mode = 0U;
        imu_diag.best_header = BSP_IMU_HEADER_BIT0_FIRST;
    } else if (imu_diag.mode3_tokmas == BSP_ICM42688_WHO_AM_I_VALUE) {
        imu_diag.best_mode = 3U;
        imu_diag.best_header = BSP_IMU_HEADER_TOKMAS_READ;
    } else if (imu_diag.mode3_msb == BSP_ICM42688_WHO_AM_I_VALUE) {
        imu_diag.best_mode = 3U;
        imu_diag.best_header = BSP_IMU_HEADER_MSB_READ;
    } else if (imu_diag.mode3_bit0 == BSP_ICM42688_WHO_AM_I_VALUE) {
        imu_diag.best_mode = 3U;
        imu_diag.best_header = BSP_IMU_HEADER_BIT0_FIRST;
    } else {
        imu_diag.best_mode = 3U;
        imu_diag.best_header = BSP_IMU_HEADER_TOKMAS_READ;
    }

    imu_diag.valid = 1U;
}

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

#if BSP_IMU_SOFT_SPI_WAVEFORM_TEST
    memset(&imu_dev, 0, sizeof(imu_dev));
    imu_dev.bus.cs_port = IMU_CS_GPIO_Port;
    imu_dev.bus.cs_pin = IMU_CS_Pin;
    imu_dev.init_stage = BSP_ICM42688_INIT_STAGE_WHO_AM_I;
    imu_dev.last_error = BSP_ICM42688_BAD_ID;
    BSP_IMU_SendSoftSpiTestFrame();
    return BSP_ICM42688_BAD_ID;
#endif

    if (imu_diag.valid == 0U) {
        memset(&imu_diag, 0, sizeof(imu_diag));
        imu_diag.best_mode = 3U;
        imu_diag.best_header = BSP_IMU_HEADER_MSB_READ;
        imu_diag.valid = 1U;
    }

    BSP_IMU_ConfigureSpiMode(SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);

    BSP_ICM42688_DefaultConfig(&config);
    config.accel_range = BSP_ICM42688_ACCEL_RANGE_4G;
    config.gyro_range = BSP_ICM42688_GYRO_RANGE_1000DPS;
    config.accel_odr = BSP_ICM42688_ODR_100HZ;
    config.gyro_odr = BSP_ICM42688_ODR_100HZ;
    config.soft_reset_on_init = false;

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

void BSP_IMU_GetDiag(BSP_IMU_Diag *diag)
{
    if (diag == 0) {
        return;
    }

    *diag = imu_diag;
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
