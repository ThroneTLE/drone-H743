#include "drv_mag.h"

#include <string.h>

#define DRV_MAG_I2C_TIMEOUT_MS 20U

#define IST8310_ADDR_7BIT  0x0EU
#define IST8310_WAI_REG    0x00U
#define IST8310_WAI_VALUE  0x10U
#define IST8310_CTRL1_REG  0x0AU
#define IST8310_CTRL2_REG  0x0BU
#define IST8310_CTRL3_REG  0x41U
#define IST8310_DATA_XL_REG 0x03U
#define IST8310_DATA_LEN    6U

#define HMC5883_ADDR_7BIT      0x1EU
#define HMC5883_REG_CFG_A      0x00U
#define HMC5883_REG_CFG_B      0x01U
#define HMC5883_REG_MODE       0x02U
#define HMC5883_REG_DATA_X_MSB 0x03U
#define HMC5883_DATA_LEN       6U
#define HMC5883_IDA_REG        0x0AU
#define HMC5883_IDB_REG        0x0BU
#define HMC5883_IDC_REG        0x0CU

#define QMC5883_ADDR_7BIT    0x0DU
#define QMC5883_REG_XOUT_L   0x00U
#define QMC5883_DATA_LEN     6U
#define QMC5883_REG_CTRL1    0x09U
#define QMC5883_REG_CTRL2    0x0AU
#define QMC5883_REG_CHIP_ID  0x0DU
#define QMC5883_CHIP_ID_VALUE 0xFFU

static uint16_t mag_i2c_addr8(uint8_t addr7)
{
    return (uint16_t)((uint16_t)addr7 << 1U);
}

static DRV_MAG_Status mag_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_MAG_OK;
    case HAL_TIMEOUT:  return DRV_MAG_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:           return DRV_MAG_ERROR;
    }
}

static DRV_MAG_Status mag_i2c_read(DRV_MAG_Device *dev, uint8_t addr7,
                                   uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;
    uint32_t timeout = (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                                    : DRV_MAG_I2C_TIMEOUT_MS;

    if ((data == NULL) || (len == 0U)) { return DRV_MAG_INVALID_ARG; }

    status = HAL_I2C_Mem_Read(dev->bus.hi2c, mag_i2c_addr8(addr7), reg,
                              I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    return mag_from_hal(status);
}

static DRV_MAG_Status mag_i2c_write(DRV_MAG_Device *dev, uint8_t addr7,
                                    uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;
    uint8_t data = value;
    uint32_t timeout = (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                                    : DRV_MAG_I2C_TIMEOUT_MS;

    status = HAL_I2C_Mem_Write(dev->bus.hi2c, mag_i2c_addr8(addr7), reg,
                               I2C_MEMADD_SIZE_8BIT, &data, 1U, timeout);
    return mag_from_hal(status);
}

static int16_t mag_make_i16_le(uint8_t l, uint8_t h)
{
    return (int16_t)((uint16_t)l | ((uint16_t)h << 8U));
}

static int16_t mag_make_i16_be(uint8_t h, uint8_t l)
{
    return (int16_t)(((uint16_t)h << 8U) | (uint16_t)l);
}

static DRV_MAG_Status mag_probe_ist8310(DRV_MAG_Device *dev)
{
    DRV_MAG_Status st;
    uint8_t who = 0U;

    st = mag_i2c_read(dev, IST8310_ADDR_7BIT, IST8310_WAI_REG, &who, 1U);
    if (st != DRV_MAG_OK) { return st; }
    dev->info.detected_ist8310 = 1U;
    if (who != IST8310_WAI_VALUE) { return DRV_MAG_BAD_ID; }

    dev->info.who_am_i = who;
    dev->info.address = IST8310_ADDR_7BIT;
    dev->info.type = DRV_MAG_TYPE_IST8310;
    return DRV_MAG_OK;
}

static DRV_MAG_Status mag_probe_hmc5883(DRV_MAG_Device *dev)
{
    DRV_MAG_Status st;
    uint8_t id[3] = {0U, 0U, 0U};

    st = mag_i2c_read(dev, HMC5883_ADDR_7BIT, HMC5883_IDA_REG, id, 3U);
    if (st != DRV_MAG_OK) { return st; }

    dev->info.detected_hmc5883 = 1U;
    dev->info.hmc_id[0] = id[0];
    dev->info.hmc_id[1] = id[1];
    dev->info.hmc_id[2] = id[2];

    if ((id[0] != (uint8_t)'H') || (id[1] != (uint8_t)'4') || (id[2] != (uint8_t)'3')) {
        return DRV_MAG_BAD_ID;
    }

    dev->info.who_am_i = id[0];
    dev->info.address = HMC5883_ADDR_7BIT;
    dev->info.type = DRV_MAG_TYPE_HMC5883;
    return DRV_MAG_OK;
}

static DRV_MAG_Status mag_probe_qmc5883(DRV_MAG_Device *dev)
{
    DRV_MAG_Status st;
    uint8_t id = 0U;

    st = mag_i2c_read(dev, QMC5883_ADDR_7BIT, QMC5883_REG_CHIP_ID, &id, 1U);
    if (st != DRV_MAG_OK) { return st; }
    dev->info.detected_qmc5883 = 1U;
    if (id != QMC5883_CHIP_ID_VALUE) { return DRV_MAG_BAD_ID; }

    dev->info.who_am_i = id;
    dev->info.address = QMC5883_ADDR_7BIT;
    dev->info.type = DRV_MAG_TYPE_QMC5883L;
    return DRV_MAG_OK;
}

static DRV_MAG_Status mag_configure_ist8310(DRV_MAG_Device *dev)
{
    DRV_MAG_Status st;

    st = mag_i2c_write(dev, IST8310_ADDR_7BIT, IST8310_CTRL2_REG, 0x01U);
    if (st != DRV_MAG_OK) { return st; }
    HAL_Delay(2U);
    st = mag_i2c_write(dev, IST8310_ADDR_7BIT, IST8310_CTRL3_REG, 0x00U);
    if (st != DRV_MAG_OK) { return st; }
    return mag_i2c_write(dev, IST8310_ADDR_7BIT, IST8310_CTRL1_REG, 0x01U);
}

static DRV_MAG_Status mag_configure_hmc5883(DRV_MAG_Device *dev)
{
    DRV_MAG_Status st;

    st = mag_i2c_write(dev, HMC5883_ADDR_7BIT, HMC5883_REG_CFG_A, 0x18U);
    if (st != DRV_MAG_OK) { return st; }
    st = mag_i2c_write(dev, HMC5883_ADDR_7BIT, HMC5883_REG_CFG_B, 0x20U);
    if (st != DRV_MAG_OK) { return st; }
    return mag_i2c_write(dev, HMC5883_ADDR_7BIT, HMC5883_REG_MODE, 0x00U);
}

static DRV_MAG_Status mag_configure_qmc5883(DRV_MAG_Device *dev)
{
    DRV_MAG_Status st;

    st = mag_i2c_write(dev, QMC5883_ADDR_7BIT, QMC5883_REG_CTRL2, 0x01U);
    if (st != DRV_MAG_OK) { return st; }
    return mag_i2c_write(dev, QMC5883_ADDR_7BIT, QMC5883_REG_CTRL1, 0x1DU);
}

static void mag_scale_data(DRV_MAG_Type type, const DRV_MAG_RawData *raw,
                           DRV_MAG_ScaledData *scaled)
{
    int32_t lsb_per_gauss = 1090;

    if ((raw == NULL) || (scaled == NULL)) { return; }

    if (type == DRV_MAG_TYPE_QMC5883L) {
        lsb_per_gauss = 12000;
    } else if (type == DRV_MAG_TYPE_IST8310) {
        lsb_per_gauss = 1600;
    }

    scaled->x_mgauss = ((int32_t)raw->x * 1000L) / lsb_per_gauss;
    scaled->y_mgauss = ((int32_t)raw->y * 1000L) / lsb_per_gauss;
    scaled->z_mgauss = ((int32_t)raw->z * 1000L) / lsb_per_gauss;
}

static DRV_MAG_Status mag_read_ist8310(DRV_MAG_Device *dev, DRV_MAG_RawData *raw)
{
    DRV_MAG_Status st;
    uint8_t data[IST8310_DATA_LEN];

    st = mag_i2c_write(dev, IST8310_ADDR_7BIT, IST8310_CTRL1_REG, 0x01U);
    if (st != DRV_MAG_OK) { return st; }
    HAL_Delay(10U);
    st = mag_i2c_read(dev, IST8310_ADDR_7BIT, IST8310_DATA_XL_REG, data, IST8310_DATA_LEN);
    if (st != DRV_MAG_OK) { return st; }

    raw->x = mag_make_i16_le(data[0], data[1]);
    raw->y = mag_make_i16_le(data[2], data[3]);
    raw->z = mag_make_i16_le(data[4], data[5]);
    return DRV_MAG_OK;
}

static DRV_MAG_Status mag_read_hmc5883(DRV_MAG_Device *dev, DRV_MAG_RawData *raw)
{
    DRV_MAG_Status st;
    uint8_t data[HMC5883_DATA_LEN];

    st = mag_i2c_read(dev, HMC5883_ADDR_7BIT, HMC5883_REG_DATA_X_MSB, data, HMC5883_DATA_LEN);
    if (st != DRV_MAG_OK) { return st; }

    raw->x = mag_make_i16_be(data[0], data[1]);
    raw->z = mag_make_i16_be(data[2], data[3]);
    raw->y = mag_make_i16_be(data[4], data[5]);
    return DRV_MAG_OK;
}

static DRV_MAG_Status mag_read_qmc5883(DRV_MAG_Device *dev, DRV_MAG_RawData *raw)
{
    DRV_MAG_Status st;
    uint8_t data[QMC5883_DATA_LEN];

    st = mag_i2c_read(dev, QMC5883_ADDR_7BIT, QMC5883_REG_XOUT_L, data, QMC5883_DATA_LEN);
    if (st != DRV_MAG_OK) { return st; }

    raw->x = mag_make_i16_le(data[0], data[1]);
    raw->y = mag_make_i16_le(data[2], data[3]);
    raw->z = mag_make_i16_le(data[4], data[5]);
    return DRV_MAG_OK;
}

DRV_MAG_Status DRV_MAG_Init(DRV_MAG_Device *dev, const DRV_MAG_Bus *bus)
{
    DRV_MAG_Status st = DRV_MAG_ERROR;

    if ((dev == NULL) || (bus == NULL) || (bus->hi2c == NULL)) {
        return DRV_MAG_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    dev->info.type = DRV_MAG_TYPE_NONE;

    st = mag_probe_ist8310(dev);
    if (st == DRV_MAG_OK) { st = mag_configure_ist8310(dev); }
    if (st != DRV_MAG_OK) {
        st = mag_probe_hmc5883(dev);
        if (st == DRV_MAG_OK) { st = mag_configure_hmc5883(dev); }
    }
    if (st != DRV_MAG_OK) {
        st = mag_probe_qmc5883(dev);
        if (st == DRV_MAG_OK) { st = mag_configure_qmc5883(dev); }
    }

    if (st == DRV_MAG_OK) {
        dev->initialized = 1U;
    } else {
        dev->info.type = DRV_MAG_TYPE_NONE;
        dev->initialized = 0U;
    }
    dev->last_status = st;
    return st;
}

DRV_MAG_Status DRV_MAG_Read(DRV_MAG_Device *dev, DRV_MAG_RawData *raw,
                            DRV_MAG_ScaledData *scaled)
{
    DRV_MAG_Status st;
    DRV_MAG_RawData local_raw;
    DRV_MAG_ScaledData local_scaled;

    if (dev->initialized == 0U) { return DRV_MAG_NOT_READY; }

    switch (dev->info.type) {
    case DRV_MAG_TYPE_IST8310:   st = mag_read_ist8310(dev, &local_raw);  break;
    case DRV_MAG_TYPE_HMC5883:   st = mag_read_hmc5883(dev, &local_raw);  break;
    case DRV_MAG_TYPE_QMC5883L:  st = mag_read_qmc5883(dev, &local_raw);  break;
    default:                     st = DRV_MAG_NOT_READY;                   break;
    }

    if (st != DRV_MAG_OK) { dev->last_status = st; return st; }

    mag_scale_data(dev->info.type, &local_raw, &local_scaled);
    dev->raw = local_raw;
    dev->scaled = local_scaled;
    dev->sample_count++;
    dev->last_status = DRV_MAG_OK;

    if (raw != NULL)    { *raw = local_raw; }
    if (scaled != NULL) { *scaled = local_scaled; }

    return DRV_MAG_OK;
}

void DRV_MAG_Invalidate(DRV_MAG_Device *dev)
{
    if (dev == NULL) { return; }
    memset(dev, 0, sizeof(*dev));
    dev->info.type = DRV_MAG_TYPE_NONE;
}

const char *DRV_MAG_TypeName(DRV_MAG_Type type)
{
    switch (type) {
    case DRV_MAG_TYPE_IST8310:  return "IST8310";
    case DRV_MAG_TYPE_HMC5883:  return "HMC5883";
    case DRV_MAG_TYPE_QMC5883L: return "QMC5883L";
    default:                    return "NONE";
    }
}
