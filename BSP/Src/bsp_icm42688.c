#include "bsp_icm42688.h"

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

static void icm42688_delay_ms(BSP_ICM42688_Device *dev, uint32_t delay_ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static uint32_t icm42688_timeout_ms(const BSP_ICM42688_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms : ICM42688_DEFAULT_TIMEOUT_MS;
}

static void icm42688_cs_low(BSP_ICM42688_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_RESET);
}

static void icm42688_cs_high(BSP_ICM42688_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_SET);
}

static BSP_ICM42688_Status icm42688_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_ICM42688_OK;
    case HAL_TIMEOUT:
        return BSP_ICM42688_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:
        return BSP_ICM42688_ERROR;
    }
}

static HAL_StatusTypeDef icm42688_spi_exchange(BSP_ICM42688_Device *dev,
                                               uint8_t tx,
                                               uint8_t *rx)
{
    uint8_t rx_byte = 0U;
    HAL_StatusTypeDef status;

    status = HAL_SPI_TransmitReceive(dev->bus.hspi,
                                     &tx,
                                     &rx_byte,
                                     1U,
                                     icm42688_timeout_ms(dev));
    if (rx != NULL) {
        *rx = rx_byte;
    }

    return status;
}

static uint8_t icm42688_build_accel_config0(const BSP_ICM42688_Config *config)
{
    return (uint8_t)(((uint8_t)config->accel_range << 5U) |
                     ((uint8_t)config->accel_odr & 0x0FU));
}

static uint8_t icm42688_build_gyro_config0(const BSP_ICM42688_Config *config)
{
    return (uint8_t)(((uint8_t)config->gyro_range << 5U) |
                     ((uint8_t)config->gyro_odr & 0x0FU));
}

static int16_t icm42688_make_int16(uint8_t msb, uint8_t lsb)
{
    return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

static BSP_ICM42688_Status icm42688_fail(BSP_ICM42688_Device *dev,
                                         BSP_ICM42688_Status status)
{
    if (dev != NULL) {
        dev->last_error = status;
    }

    return status;
}

void BSP_ICM42688_DefaultConfig(BSP_ICM42688_Config *config)
{
    if (config == NULL) {
        return;
    }

    config->accel_range        = BSP_ICM42688_ACCEL_RANGE_4G;
    config->gyro_range         = BSP_ICM42688_GYRO_RANGE_1000DPS;
    config->accel_odr          = BSP_ICM42688_ODR_100HZ;
    config->gyro_odr           = BSP_ICM42688_ODR_100HZ;
    config->accel_filter_bw    = 1U;
    config->gyro_filter_bw     = 1U;
    config->enable_temp        = true;
    config->soft_reset_on_init = false;
}

BSP_ICM42688_Status BSP_ICM42688_Init(BSP_ICM42688_Device *dev,
                                      const BSP_ICM42688_Bus *bus,
                                      const BSP_ICM42688_Config *config)
{
    uint8_t who_am_i = 0U;
    BSP_ICM42688_Config default_config;
    BSP_ICM42688_Status status;

    if ((dev == NULL) || (bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_port == NULL)) {
        return BSP_ICM42688_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;

    if (config != NULL) {
        dev->config = *config;
    } else {
        BSP_ICM42688_DefaultConfig(&default_config);
        dev->config = default_config;
    }

    icm42688_cs_high(dev);
    icm42688_delay_ms(dev, ICM42688_POWER_UP_DELAY_MS);

    dev->init_stage = BSP_ICM42688_INIT_STAGE_BANK_SELECT;
    status = BSP_ICM42688_WriteRegister(dev, ICM42688_REG_BANK_SEL, ICM42688_BANK0);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    dev->init_stage = BSP_ICM42688_INIT_STAGE_WHO_AM_I;
    status = BSP_ICM42688_ReadWhoAmI(dev, &who_am_i);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    dev->who_am_i = who_am_i;
    if (who_am_i != BSP_ICM42688_WHO_AM_I_VALUE) {
        return icm42688_fail(dev, BSP_ICM42688_BAD_ID);
    }

    if (dev->config.soft_reset_on_init) {
        dev->init_stage = BSP_ICM42688_INIT_STAGE_RESET;
        status = BSP_ICM42688_Reset(dev);
        if (status != BSP_ICM42688_OK) {
            return icm42688_fail(dev, status);
        }

        dev->init_stage = BSP_ICM42688_INIT_STAGE_BANK_SELECT;
        status = BSP_ICM42688_WriteRegister(dev, ICM42688_REG_BANK_SEL, ICM42688_BANK0);
        if (status != BSP_ICM42688_OK) {
            return icm42688_fail(dev, status);
        }
    }

    dev->init_stage = BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG;
    status = BSP_ICM42688_WriteRegister(dev,
                                        ICM42688_REG_ACCEL_CONFIG0,
                                        icm42688_build_accel_config0(&dev->config));
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    dev->init_stage = BSP_ICM42688_INIT_STAGE_GYRO_CONFIG;
    status = BSP_ICM42688_WriteRegister(dev,
                                        ICM42688_REG_GYRO_CONFIG0,
                                        icm42688_build_gyro_config0(&dev->config));
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    dev->init_stage = BSP_ICM42688_INIT_STAGE_PWR_MGMT;
    status = BSP_ICM42688_ReadRegister(dev, ICM42688_REG_PWR_MGMT0, &who_am_i);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }
    who_am_i &= (uint8_t)~ICM42688_PWR_TEMP_ENABLE_MASK;
    who_am_i |= ICM42688_PWR_GYRO_ACCEL_LN;
    status = BSP_ICM42688_WriteRegister(dev, ICM42688_REG_PWR_MGMT0, who_am_i);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    icm42688_delay_ms(dev, ICM42688_SENSOR_STARTUP_MS);

    status = BSP_ICM42688_WriteRegister(dev,
                                        ICM42688_REG_INT_CONFIG,
                                        ICM42688_INT1_PUSH_PULL_ACTIVE_HIGH);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    status = BSP_ICM42688_WriteRegister(dev,
                                        ICM42688_REG_INT_CONFIG1,
                                        ICM42688_INT_CONFIG1_INT_PINS_OK);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    status = BSP_ICM42688_WriteRegister(dev,
                                        ICM42688_REG_INT_SOURCE0,
                                        ICM42688_UI_DRDY_INT1_EN);
    if (status != BSP_ICM42688_OK) {
        return icm42688_fail(dev, status);
    }

    dev->init_stage = BSP_ICM42688_INIT_STAGE_READY;
    dev->last_error = BSP_ICM42688_OK;
    return BSP_ICM42688_OK;
}

BSP_ICM42688_Status BSP_ICM42688_Reset(BSP_ICM42688_Device *dev)
{
    BSP_ICM42688_Status status;

    if (dev == NULL) {
        return BSP_ICM42688_INVALID_ARG;
    }

    status = BSP_ICM42688_WriteRegister(dev,
                                        ICM42688_REG_DEVICE_CONFIG,
                                        ICM42688_DEVICE_SOFT_RESET);
    if (status != BSP_ICM42688_OK) {
        return status;
    }

    icm42688_delay_ms(dev, ICM42688_RESET_DELAY_MS);

    return BSP_ICM42688_OK;
}

BSP_ICM42688_Status BSP_ICM42688_ReadWhoAmI(BSP_ICM42688_Device *dev, uint8_t *who_am_i)
{
    return BSP_ICM42688_ReadRegister(dev, ICM42688_REG_WHO_AM_I, who_am_i);
}

BSP_ICM42688_Status BSP_ICM42688_ReadRegister(BSP_ICM42688_Device *dev,
                                              uint8_t reg,
                                              uint8_t *value)
{
    return BSP_ICM42688_ReadRegisters(dev, reg, value, 1U);
}

BSP_ICM42688_Status BSP_ICM42688_WriteRegister(BSP_ICM42688_Device *dev,
                                               uint8_t reg,
                                               uint8_t value)
{
    return BSP_ICM42688_WriteRegisters(dev, reg, &value, 1U);
}

BSP_ICM42688_Status BSP_ICM42688_ReadRegisters(BSP_ICM42688_Device *dev,
                                               uint8_t reg,
                                               uint8_t *data,
                                               uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (data == NULL) || (len == 0U)) {
        return BSP_ICM42688_INVALID_ARG;
    }

    icm42688_cs_low(dev);
    hal_status = icm42688_spi_exchange(dev, (uint8_t)(reg | ICM42688_SPI_READ_BIT), NULL);
    for (uint16_t i = 0U; (i < len) && (hal_status == HAL_OK); ++i) {
        hal_status = icm42688_spi_exchange(dev, 0x00U, &data[i]);
    }
    icm42688_cs_high(dev);

    return icm42688_from_hal_status(hal_status);
}

BSP_ICM42688_Status BSP_ICM42688_WriteRegisters(BSP_ICM42688_Device *dev,
                                                uint8_t reg,
                                                const uint8_t *data,
                                                uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (data == NULL) || (len == 0U)) {
        return BSP_ICM42688_INVALID_ARG;
    }

    icm42688_cs_low(dev);
    hal_status = icm42688_spi_exchange(dev, (uint8_t)(reg & (uint8_t)~ICM42688_SPI_READ_BIT), NULL);
    for (uint16_t i = 0U; (i < len) && (hal_status == HAL_OK); ++i) {
        hal_status = icm42688_spi_exchange(dev, data[i], NULL);
    }
    icm42688_cs_high(dev);

    return icm42688_from_hal_status(hal_status);
}

BSP_ICM42688_Status BSP_ICM42688_ReadRaw(BSP_ICM42688_Device *dev,
                                         BSP_ICM42688_RawData *raw)
{
    uint8_t buffer[14];
    BSP_ICM42688_Status status;

    if ((dev == NULL) || (raw == NULL)) {
        return BSP_ICM42688_INVALID_ARG;
    }

    status = BSP_ICM42688_ReadRegisters(dev, ICM42688_REG_TEMP_DATA1, buffer, 14U);
    if (status != BSP_ICM42688_OK) {
        return status;
    }

    raw->temperature = icm42688_make_int16(buffer[0], buffer[1]);
    raw->accel_x     = icm42688_make_int16(buffer[2], buffer[3]);
    raw->accel_y     = icm42688_make_int16(buffer[4], buffer[5]);
    raw->accel_z     = icm42688_make_int16(buffer[6], buffer[7]);
    raw->gyro_x      = icm42688_make_int16(buffer[8], buffer[9]);
    raw->gyro_y      = icm42688_make_int16(buffer[10], buffer[11]);
    raw->gyro_z      = icm42688_make_int16(buffer[12], buffer[13]);

    return BSP_ICM42688_OK;
}

BSP_ICM42688_Status BSP_ICM42688_ReadScaled(BSP_ICM42688_Device *dev,
                                            BSP_ICM42688_ScaledData *scaled)
{
    BSP_ICM42688_RawData raw;
    BSP_ICM42688_Status status;

    if ((dev == NULL) || (scaled == NULL)) {
        return BSP_ICM42688_INVALID_ARG;
    }

    status = BSP_ICM42688_ReadRaw(dev, &raw);
    if (status != BSP_ICM42688_OK) {
        return status;
    }

    BSP_ICM42688_ConvertRaw(dev, &raw, scaled);
    return BSP_ICM42688_OK;
}

BSP_ICM42688_Status BSP_ICM42688_IsDataReady(BSP_ICM42688_Device *dev, bool *ready)
{
    uint8_t int_status = 0U;
    BSP_ICM42688_Status status;

    if ((dev == NULL) || (ready == NULL)) {
        return BSP_ICM42688_INVALID_ARG;
    }

    status = BSP_ICM42688_ReadRegister(dev, ICM42688_REG_INT_STATUS, &int_status);
    if (status != BSP_ICM42688_OK) {
        return status;
    }

    *ready = ((int_status & ICM42688_DATA_READY_MASK) != 0U);
    return BSP_ICM42688_OK;
}

void BSP_ICM42688_ConvertRaw(const BSP_ICM42688_Device *dev,
                             const BSP_ICM42688_RawData *raw,
                             BSP_ICM42688_ScaledData *scaled)
{
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;

    if ((dev == NULL) || (raw == NULL) || (scaled == NULL)) {
        return;
    }

    accel_lsb_per_g = BSP_ICM42688_AccelLsbPerG(dev->config.accel_range);
    gyro_lsb_per_dps = BSP_ICM42688_GyroLsbPerDps(dev->config.gyro_range);

    scaled->temperature_c = ((float)raw->temperature / 132.48f) + 25.0f;
    scaled->accel_x_g = (float)raw->accel_x / accel_lsb_per_g;
    scaled->accel_y_g = (float)raw->accel_y / accel_lsb_per_g;
    scaled->accel_z_g = (float)raw->accel_z / accel_lsb_per_g;
    scaled->gyro_x_dps = (float)raw->gyro_x / gyro_lsb_per_dps;
    scaled->gyro_y_dps = (float)raw->gyro_y / gyro_lsb_per_dps;
    scaled->gyro_z_dps = (float)raw->gyro_z / gyro_lsb_per_dps;
}

float BSP_ICM42688_AccelLsbPerG(BSP_ICM42688_AccelRange range)
{
    switch (range) {
    case BSP_ICM42688_ACCEL_RANGE_2G:
        return 16384.0f;
    case BSP_ICM42688_ACCEL_RANGE_4G:
        return 8192.0f;
    case BSP_ICM42688_ACCEL_RANGE_8G:
        return 4096.0f;
    case BSP_ICM42688_ACCEL_RANGE_16G:
    default:
        return 2048.0f;
    }
}

float BSP_ICM42688_GyroLsbPerDps(BSP_ICM42688_GyroRange range)
{
    switch (range) {
    case BSP_ICM42688_GYRO_RANGE_15D625DPS:
        return 2097.2f;
    case BSP_ICM42688_GYRO_RANGE_31D25DPS:
        return 1048.6f;
    case BSP_ICM42688_GYRO_RANGE_62D5DPS:
        return 524.3f;
    case BSP_ICM42688_GYRO_RANGE_125DPS:
        return 262.0f;
    case BSP_ICM42688_GYRO_RANGE_250DPS:
        return 131.0f;
    case BSP_ICM42688_GYRO_RANGE_500DPS:
        return 65.5f;
    case BSP_ICM42688_GYRO_RANGE_1000DPS:
        return 32.8f;
    case BSP_ICM42688_GYRO_RANGE_2000DPS:
    default:
        return 16.4f;
    }
}
