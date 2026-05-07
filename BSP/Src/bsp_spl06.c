#include "bsp_spl06.h"

#include <string.h>

#define SPL06_REG_ID             0x0DU
#define SPL06_REG_PRS_CFG        0x06U
#define SPL06_REG_TMP_CFG        0x07U
#define SPL06_REG_MEAS_CFG       0x08U
#define SPL06_REG_CFG_REG        0x09U
#define SPL06_REG_COEF_SRCE      0x28U
#define SPL06_SPI_READ_BIT       0x80U
#define SPL06_DEFAULT_TIMEOUT_MS 100U
#define SPL06_SPI_WAKE_DELAY_MS  1U
#define SPL06_CONFIG_DELAY_MS    40U
#define SPL06_RATE_8HZ           0x30U
#define SPL06_OVERSAMPLE_1X      0x00U
#define SPL06_MEAS_BACKGROUND_PT 0x07U
#define SPL06_TMP_EXT_BIT        0x80U

static uint32_t spl06_timeout_ms(const BSP_SPL06_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms : SPL06_DEFAULT_TIMEOUT_MS;
}

static void spl06_delay_ms(BSP_SPL06_Device *dev, uint32_t delay_ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static void spl06_cs_low(BSP_SPL06_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_RESET);
}

static void spl06_cs_high(BSP_SPL06_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_SET);
}

static BSP_SPL06_Status spl06_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return BSP_SPL06_OK;
    case HAL_TIMEOUT:
        return BSP_SPL06_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:
        return BSP_SPL06_ERROR;
    }
}

BSP_SPL06_Status BSP_SPL06_Init(BSP_SPL06_Device *dev, const BSP_SPL06_Bus *bus)
{
    BSP_SPL06_Status status;
    uint8_t product_id = 0U;
    uint8_t tmp_cfg = SPL06_TMP_EXT_BIT | SPL06_RATE_8HZ | SPL06_OVERSAMPLE_1X;

    if ((dev == NULL) || (bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_port == NULL)) {
        return BSP_SPL06_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;

    spl06_cs_high(dev);
    spl06_delay_ms(dev, SPL06_SPI_WAKE_DELAY_MS);

    status = BSP_SPL06_ReadId(dev, &product_id);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    dev->product_id = product_id;
    if (product_id != BSP_SPL06_ID_VALUE) {
        return BSP_SPL06_BAD_ID;
    }

    status = BSP_SPL06_WriteRegister(dev,
                                     SPL06_REG_PRS_CFG,
                                     SPL06_RATE_8HZ | SPL06_OVERSAMPLE_1X);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    status = BSP_SPL06_WriteRegister(dev, SPL06_REG_TMP_CFG, tmp_cfg);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    status = BSP_SPL06_WriteRegister(dev, SPL06_REG_CFG_REG, 0x00U);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    status = BSP_SPL06_WriteRegister(dev,
                                     SPL06_REG_MEAS_CFG,
                                     SPL06_MEAS_BACKGROUND_PT);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    spl06_delay_ms(dev, SPL06_CONFIG_DELAY_MS);

    return BSP_SPL06_OK;
}

BSP_SPL06_Status BSP_SPL06_ReadId(BSP_SPL06_Device *dev, uint8_t *product_id)
{
    BSP_SPL06_Status status;

    status = BSP_SPL06_ReadRegister(dev, SPL06_REG_ID, product_id);
    if ((status == BSP_SPL06_OK) && (product_id != NULL)) {
        dev->product_id = *product_id;
    }

    return status;
}

BSP_SPL06_Status BSP_SPL06_ReadIdTxRx(BSP_SPL06_Device *dev, uint8_t *product_id)
{
    uint8_t tx_data[2] = {(uint8_t)(SPL06_REG_ID | SPL06_SPI_READ_BIT), 0U};
    uint8_t rx_data[2] = {0U, 0U};
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (product_id == NULL)) {
        return BSP_SPL06_INVALID_ARG;
    }

    spl06_cs_low(dev);
    hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi,
                                         tx_data,
                                         rx_data,
                                         (uint16_t)sizeof(tx_data),
                                         spl06_timeout_ms(dev));
    spl06_cs_high(dev);

    if (hal_status != HAL_OK) {
        return spl06_from_hal_status(hal_status);
    }

    *product_id = rx_data[1];
    dev->product_id = *product_id;

    return BSP_SPL06_OK;
}

BSP_SPL06_Status BSP_SPL06_ReadRegister(BSP_SPL06_Device *dev,
                                        uint8_t reg,
                                        uint8_t *value)
{
    return BSP_SPL06_ReadRegisters(dev, reg, value, 1U);
}

BSP_SPL06_Status BSP_SPL06_ReadRegisters(BSP_SPL06_Device *dev,
                                         uint8_t reg,
                                         uint8_t *data,
                                         uint16_t len)
{
    HAL_StatusTypeDef hal_status;
    uint8_t read_command;

    if ((dev == NULL) || (data == NULL) || (len == 0U)) {
        return BSP_SPL06_INVALID_ARG;
    }

    read_command = (uint8_t)(reg | SPL06_SPI_READ_BIT);

    spl06_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi,
                                  &read_command,
                                  1U,
                                  spl06_timeout_ms(dev));
    if (hal_status == HAL_OK) {
        hal_status = HAL_SPI_Receive(dev->bus.hspi,
                                     data,
                                     len,
                                     spl06_timeout_ms(dev));
    }
    spl06_cs_high(dev);

    return spl06_from_hal_status(hal_status);
}

BSP_SPL06_Status BSP_SPL06_WriteRegister(BSP_SPL06_Device *dev,
                                         uint8_t reg,
                                         uint8_t value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_data[2];

    if (dev == NULL) {
        return BSP_SPL06_INVALID_ARG;
    }

    tx_data[0] = (uint8_t)(reg & (uint8_t)~SPL06_SPI_READ_BIT);
    tx_data[1] = value;

    spl06_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi,
                                  tx_data,
                                  (uint16_t)sizeof(tx_data),
                                  spl06_timeout_ms(dev));
    spl06_cs_high(dev);

    return spl06_from_hal_status(hal_status);
}
