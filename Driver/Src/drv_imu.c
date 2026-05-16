#include "drv_imu.h"

#include <string.h>

#define ICM42688_REG_DEVICE_CONFIG       0x11U
#define ICM42688_REG_INT_CONFIG          0x14U
#define ICM42688_REG_TEMP_DATA1          0x1DU
#define ICM42688_REG_INT_STATUS          0x2DU
#define ICM42688_REG_SIGNAL_PATH_RESET   0x4BU
#define ICM42688_REG_PWR_MGMT0           0x4EU
#define ICM42688_REG_GYRO_CONFIG0        0x4FU
#define ICM42688_REG_ACCEL_CONFIG0       0x50U
#define ICM42688_REG_INT_CONFIG1         0x64U
#define ICM42688_REG_INT_SOURCE0         0x65U
#define ICM42688_REG_WHO_AM_I            0x75U
#define ICM42688_REG_BANK_SEL            0x76U

#define ICM42688_SPI_READ_BIT            0x80U
#define ICM42688_DEFAULT_TIMEOUT_MS      100U
#define ICM42688_POWER_UP_DELAY_MS       100U
#define ICM42688_RESET_DELAY_MS          100U
#define ICM42688_SENSOR_STARTUP_MS       1U

#define ICM42688_DEVICE_SOFT_RESET       0x01U
#define ICM42688_BANK0                   0x00U
#define ICM42688_PWR_TEMP_ENABLE_MASK    0x20U
#define ICM42688_PWR_GYRO_ACCEL_LN       0x0FU
#define ICM42688_DATA_READY_MASK         0x08U
#define ICM42688_INT1_PUSH_PULL_ACTIVE_HIGH 0x03U
#define ICM42688_INT_CONFIG1_INT_PINS_OK    0x00U
#define ICM42688_UI_DRDY_INT1_EN            0x08U

static void icm42688_delay_ms(DRV_IMU_Device *dev, uint32_t delay_ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static uint32_t icm42688_timeout_ms(const DRV_IMU_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms : ICM42688_DEFAULT_TIMEOUT_MS;
}

static void icm42688_cs_low(DRV_IMU_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_RESET);
}

static void icm42688_cs_high(DRV_IMU_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_SET);
}

static DRV_IMU_Status icm42688_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:     return DRV_IMU_OK;
    case HAL_TIMEOUT: return DRV_IMU_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:          return DRV_IMU_ERROR;
    }
}

static HAL_StatusTypeDef icm42688_spi_exchange(DRV_IMU_Device *dev,
                                               uint8_t tx, uint8_t *rx)
{
    uint8_t rx_byte = 0U;
    HAL_StatusTypeDef status;

    status = HAL_SPI_TransmitReceive(dev->bus.hspi, &tx, &rx_byte, 1U,
                                     icm42688_timeout_ms(dev));
    if (rx != NULL) { *rx = rx_byte; }

    return status;
}

static uint8_t icm42688_build_accel_config0(const DRV_IMU_Config *config)
{
    return (uint8_t)(((uint8_t)config->accel_range << 5U) |
                     ((uint8_t)config->accel_odr & 0x0FU));
}

static uint8_t icm42688_build_gyro_config0(const DRV_IMU_Config *config)
{
    return (uint8_t)(((uint8_t)config->gyro_range << 5U) |
                     ((uint8_t)config->gyro_odr & 0x0FU));
}

static int16_t icm42688_make_int16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

static DRV_IMU_Status icm42688_fail(DRV_IMU_Device *dev, DRV_IMU_Status status)
{
    if (dev != NULL) { dev->last_error = status; }
    return status;
}

void DRV_IMU_DefaultConfig(DRV_IMU_Config *config)
{
    if (config == NULL) { return; }

    config->accel_range        = DRV_IMU_ACCEL_RANGE_4G;
    config->gyro_range         = DRV_IMU_GYRO_RANGE_1000DPS;
    config->accel_odr          = DRV_IMU_ODR_100HZ;
    config->gyro_odr           = DRV_IMU_ODR_100HZ;
    config->accel_filter_bw    = 1U;
    config->gyro_filter_bw     = 1U;
    config->enable_temp        = true;
    config->soft_reset_on_init = false;
}

DRV_IMU_Status DRV_IMU_Init(DRV_IMU_Device *dev, const DRV_IMU_Bus *bus,
                            const DRV_IMU_Config *config)
{
    uint8_t who_am_i = 0U;
    DRV_IMU_Config default_config;
    DRV_IMU_Status status;

    if ((dev == NULL) || (bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_port == NULL)) {
        return DRV_IMU_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;

    if (config != NULL) {
        dev->config = *config;
    } else {
        DRV_IMU_DefaultConfig(&default_config);
        dev->config = default_config;
    }

    icm42688_cs_high(dev);
    icm42688_delay_ms(dev, ICM42688_POWER_UP_DELAY_MS);

    dev->init_stage = DRV_IMU_INIT_STAGE_BANK_SELECT;
    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_BANK_SEL, ICM42688_BANK0);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_WHO_AM_I;
    status = DRV_IMU_ReadWhoAmI(dev, &who_am_i);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    dev->who_am_i = who_am_i;
    if (who_am_i != DRV_IMU_WHO_AM_I_VALUE) {
        return icm42688_fail(dev, DRV_IMU_BAD_ID);
    }

    if (dev->config.soft_reset_on_init) {
        dev->init_stage = DRV_IMU_INIT_STAGE_RESET;
        status = DRV_IMU_Reset(dev);
        if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

        dev->init_stage = DRV_IMU_INIT_STAGE_BANK_SELECT;
        status = DRV_IMU_WriteRegister(dev, ICM42688_REG_BANK_SEL, ICM42688_BANK0);
        if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }
    }

    dev->init_stage = DRV_IMU_INIT_STAGE_ACCEL_CONFIG;
    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_ACCEL_CONFIG0,
                                   icm42688_build_accel_config0(&dev->config));
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_GYRO_CONFIG;
    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_GYRO_CONFIG0,
                                   icm42688_build_gyro_config0(&dev->config));
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_PWR_MGMT;
    status = DRV_IMU_ReadRegister(dev, ICM42688_REG_PWR_MGMT0, &who_am_i);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }
    who_am_i &= (uint8_t)~ICM42688_PWR_TEMP_ENABLE_MASK;
    who_am_i |= ICM42688_PWR_GYRO_ACCEL_LN;
    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_PWR_MGMT0, who_am_i);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    icm42688_delay_ms(dev, ICM42688_SENSOR_STARTUP_MS);

    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_INT_CONFIG,
                                   ICM42688_INT1_PUSH_PULL_ACTIVE_HIGH);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_INT_CONFIG1,
                                   ICM42688_INT_CONFIG1_INT_PINS_OK);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_INT_SOURCE0,
                                   ICM42688_UI_DRDY_INT1_EN);
    if (status != DRV_IMU_OK) { return icm42688_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_READY;
    dev->last_error = DRV_IMU_OK;
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_IMU_Reset(DRV_IMU_Device *dev)
{
    DRV_IMU_Status status;

    if (dev == NULL) { return DRV_IMU_INVALID_ARG; }

    status = DRV_IMU_WriteRegister(dev, ICM42688_REG_DEVICE_CONFIG,
                                   ICM42688_DEVICE_SOFT_RESET);
    if (status != DRV_IMU_OK) { return status; }

    icm42688_delay_ms(dev, ICM42688_RESET_DELAY_MS);
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_IMU_ReadWhoAmI(DRV_IMU_Device *dev, uint8_t *who_am_i)
{
    return DRV_IMU_ReadRegister(dev, ICM42688_REG_WHO_AM_I, who_am_i);
}

DRV_IMU_Status DRV_IMU_ReadRegister(DRV_IMU_Device *dev, uint8_t reg, uint8_t *value)
{
    return DRV_IMU_ReadRegisters(dev, reg, value, 1U);
}

DRV_IMU_Status DRV_IMU_WriteRegister(DRV_IMU_Device *dev, uint8_t reg, uint8_t value)
{
    return DRV_IMU_WriteRegisters(dev, reg, &value, 1U);
}

DRV_IMU_Status DRV_IMU_ReadRegisters(DRV_IMU_Device *dev, uint8_t reg,
                                     uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (data == NULL) || (len == 0U)) {
        return DRV_IMU_INVALID_ARG;
    }

    icm42688_cs_low(dev);
    hal_status = icm42688_spi_exchange(dev, (uint8_t)(reg | ICM42688_SPI_READ_BIT), NULL);
    for (uint16_t i = 0U; (i < len) && (hal_status == HAL_OK); ++i) {
        hal_status = icm42688_spi_exchange(dev, 0x00U, &data[i]);
    }
    icm42688_cs_high(dev);

    return icm42688_from_hal_status(hal_status);
}

DRV_IMU_Status DRV_IMU_WriteRegisters(DRV_IMU_Device *dev, uint8_t reg,
                                      const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (data == NULL) || (len == 0U)) {
        return DRV_IMU_INVALID_ARG;
    }

    icm42688_cs_low(dev);
    hal_status = icm42688_spi_exchange(dev,
                                       (uint8_t)(reg & (uint8_t)~ICM42688_SPI_READ_BIT),
                                       NULL);
    for (uint16_t i = 0U; (i < len) && (hal_status == HAL_OK); ++i) {
        hal_status = icm42688_spi_exchange(dev, data[i], NULL);
    }
    icm42688_cs_high(dev);

    return icm42688_from_hal_status(hal_status);
}

DRV_IMU_Status DRV_IMU_ReadRaw(DRV_IMU_Device *dev, DRV_IMU_RawData *raw)
{
    uint8_t buffer[14];
    DRV_IMU_Status status;

    if ((dev == NULL) || (raw == NULL)) { return DRV_IMU_INVALID_ARG; }

    status = DRV_IMU_ReadRegisters(dev, ICM42688_REG_TEMP_DATA1, buffer, 14U);
    if (status != DRV_IMU_OK) { return status; }

    raw->temperature = icm42688_make_int16(buffer[0], buffer[1]);
    raw->accel_x     = icm42688_make_int16(buffer[2], buffer[3]);
    raw->accel_y     = icm42688_make_int16(buffer[4], buffer[5]);
    raw->accel_z     = icm42688_make_int16(buffer[6], buffer[7]);
    raw->gyro_x      = icm42688_make_int16(buffer[8], buffer[9]);
    raw->gyro_y      = icm42688_make_int16(buffer[10], buffer[11]);
    raw->gyro_z      = icm42688_make_int16(buffer[12], buffer[13]);

    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_IMU_ReadScaled(DRV_IMU_Device *dev, DRV_IMU_ScaledData *scaled)
{
    DRV_IMU_RawData raw;
    DRV_IMU_Status status;

    if ((dev == NULL) || (scaled == NULL)) { return DRV_IMU_INVALID_ARG; }

    status = DRV_IMU_ReadRaw(dev, &raw);
    if (status != DRV_IMU_OK) { return status; }

    DRV_IMU_ConvertRaw(dev, &raw, scaled);
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_IMU_IsDataReady(DRV_IMU_Device *dev, bool *ready)
{
    uint8_t int_status = 0U;
    DRV_IMU_Status status;

    if ((dev == NULL) || (ready == NULL)) { return DRV_IMU_INVALID_ARG; }

    status = DRV_IMU_ReadRegister(dev, ICM42688_REG_INT_STATUS, &int_status);
    if (status != DRV_IMU_OK) { return status; }

    *ready = ((int_status & ICM42688_DATA_READY_MASK) != 0U);
    return DRV_IMU_OK;
}

void DRV_IMU_ConvertRaw(const DRV_IMU_Device *dev,
                        const DRV_IMU_RawData *raw,
                        DRV_IMU_ScaledData *scaled)
{
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    if ((dev == NULL) || (raw == NULL) || (scaled == NULL)) { return; }

    accel_lsb_per_g = DRV_IMU_AccelLsbPerG(dev->config.accel_range);
    gyro_lsb_per_dps = DRV_IMU_GyroLsbPerDps(dev->config.gyro_range);

    scaled->temperature_c = ((float)raw->temperature / 132.48f) + 25.0f;
    scaled->accel_x_g  = (float)raw->accel_x / accel_lsb_per_g;
    scaled->accel_y_g  = (float)raw->accel_y / accel_lsb_per_g;
    scaled->accel_z_g  = (float)raw->accel_z / accel_lsb_per_g;
    scaled->gyro_x_dps = (float)raw->gyro_x / gyro_lsb_per_dps;
    scaled->gyro_y_dps = (float)raw->gyro_y / gyro_lsb_per_dps;
    scaled->gyro_z_dps = (float)raw->gyro_z / gyro_lsb_per_dps;
}

float DRV_IMU_AccelLsbPerG(DRV_IMU_AccelRange range)
{
    switch (range) {
    case DRV_IMU_ACCEL_RANGE_2G:  return 16384.0f;
    case DRV_IMU_ACCEL_RANGE_4G:  return 8192.0f;
    case DRV_IMU_ACCEL_RANGE_8G:  return 4096.0f;
    case DRV_IMU_ACCEL_RANGE_16G:
    default:                       return 2048.0f;
    }
}

float DRV_IMU_GyroLsbPerDps(DRV_IMU_GyroRange range)
{
    switch (range) {
    case DRV_IMU_GYRO_RANGE_15D625DPS: return 2097.2f;
    case DRV_IMU_GYRO_RANGE_31D25DPS:  return 1048.6f;
    case DRV_IMU_GYRO_RANGE_62D5DPS:   return 524.3f;
    case DRV_IMU_GYRO_RANGE_125DPS:    return 262.0f;
    case DRV_IMU_GYRO_RANGE_250DPS:    return 131.0f;
    case DRV_IMU_GYRO_RANGE_500DPS:    return 65.5f;
    case DRV_IMU_GYRO_RANGE_1000DPS:   return 32.8f;
    case DRV_IMU_GYRO_RANGE_2000DPS:
    default:                            return 16.4f;
    }
}
