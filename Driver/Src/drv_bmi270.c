/*
 * Bosch BMI270 驱动实现。
 *
 * 初始化时序按 Bosch datasheet，并与 ArduPilot `AP_InertialSensor_BMI270.cpp`
 * 和 Betaflight `accgyro_spi_bmi270.c` 交叉核对（配置上传、软复位后重新进 SPI 模式、
 * ois_range 位这三处易错点以那两份实测实现为准）。
 */

#include "drv_bmi270.h"

#include <string.h>

#define BMI270_REG_CHIP_ID          0x00U
#define BMI270_REG_STATUS           0x03U
#define BMI270_REG_ACC_DATA_X_LSB   0x0CU
#define BMI270_REG_INT_STATUS_1     0x1DU
#define BMI270_REG_INTERNAL_STATUS  0x21U
#define BMI270_REG_TEMPERATURE_LSB  0x22U
#define BMI270_REG_ACC_CONF         0x40U
#define BMI270_REG_ACC_RANGE        0x41U
#define BMI270_REG_GYR_CONF         0x42U
#define BMI270_REG_GYR_RANGE        0x43U
#define BMI270_REG_INT1_IO_CTRL     0x53U
#define BMI270_REG_INT2_IO_CTRL     0x54U
#define BMI270_REG_INT_MAP_DATA     0x58U
#define BMI270_REG_INIT_CTRL        0x59U
#define BMI270_REG_INIT_DATA        0x5EU
#define BMI270_REG_PWR_CONF         0x7CU
#define BMI270_REG_PWR_CTRL         0x7DU
#define BMI270_REG_CMD              0x7EU

#define BMI270_SPI_READ_BIT         0x80U
#define BMI270_CMD_SOFTRESET        0xB6U
#define BMI270_PWR_CTRL_ALL_ON      0x0EU  /* 加计 + 陀螺 + 温度，关 aux 接口 */
#define BMI270_PWR_CONF_RUN         0x02U  /* 关高级省电，开 FIFO self-wake */
#define BMI270_PWR_CONF_NO_SAVE     0x00U
#define BMI270_INTERNAL_STATUS_OK   0x01U
#define BMI270_GYR_DRDY_MASK        0x40U

#define BMI270_DEFAULT_TIMEOUT_MS   100U
#define BMI270_POWERUP_DELAY_MS       3U   /* datasheet：上电与软复位各需 2 ms */
#define BMI270_PWR_CONF_DELAY_MS      2U   /* 至少 450 us */
#define BMI270_CONFIG_LOAD_DELAY_MS  25U   /* datasheet 上限 20 ms，留一点余量 */
#define BMI270_INIT_MAX_TRIES         5U

/* SPI 读同样要跳一个 dummy 字节；一次最多读 12 字节（加计 6 + 陀螺 6）。 */
#define BMI270_XFER_MAX (12U + 2U)

/* ============================================================ 基础工具 */

static uint32_t bmi270_timeout_ms(const DRV_BMI270_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                       : BMI270_DEFAULT_TIMEOUT_MS;
}

static void bmi270_delay_ms(const DRV_BMI270_Device *dev, uint32_t ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(ms);
    } else {
        HAL_Delay(ms);
    }
}

static DRV_IMU_Status bmi270_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_IMU_OK;
    case HAL_TIMEOUT: return DRV_IMU_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:          return DRV_IMU_ERROR;
    }
}

static DRV_IMU_Status bmi270_fail(DRV_BMI270_Device *dev, DRV_IMU_Status status)
{
    if (dev != NULL) { dev->last_error = status; }
    return status;
}

static void bmi270_cs(DRV_BMI270_Device *dev, GPIO_PinState state)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, state);
}

static int16_t bmi270_make_int16(uint8_t lsb, uint8_t msb)
{
    /* BMI270 小端：先 LSB 后 MSB。 */
    return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

/* ============================================================ 传输 */

static DRV_IMU_Status bmi270_read(DRV_BMI270_Device *dev, uint8_t reg,
                                  uint8_t *data, uint16_t len)
{
    uint8_t tx[BMI270_XFER_MAX];
    uint8_t rx[BMI270_XFER_MAX];
    uint16_t total = (uint16_t)(len + 2U);
    HAL_StatusTypeDef hal_status;

    if (total > (uint16_t)BMI270_XFER_MAX) { return DRV_IMU_INVALID_ARG; }

    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | BMI270_SPI_READ_BIT);

    bmi270_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi, tx, rx, total,
                                         bmi270_timeout_ms(dev));
    bmi270_cs(dev, GPIO_PIN_SET);

    if (hal_status != HAL_OK) { return bmi270_from_hal(hal_status); }

    memcpy(data, &rx[2], len);
    return DRV_IMU_OK;
}

static DRV_IMU_Status bmi270_write(DRV_BMI270_Device *dev, uint8_t reg,
                                   uint8_t value)
{
    uint8_t frame[2] = { (uint8_t)(reg & (uint8_t)~BMI270_SPI_READ_BIT), value };
    HAL_StatusTypeDef hal_status;

    bmi270_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi, frame, 2U, bmi270_timeout_ms(dev));
    bmi270_cs(dev, GPIO_PIN_SET);

    return bmi270_from_hal(hal_status);
}

/*
 * 配置数据是一次性突发写：先发 INIT_DATA 地址字节，紧接着 328 字节数据，
 * 中途不能抬片选。分成两次 HAL_SPI_Transmit 是安全的（片选由我们自己控制），
 * 这样避免为了拼一个 329 字节缓冲再复制一遍。
 */
static DRV_IMU_Status bmi270_write_config_file(DRV_BMI270_Device *dev)
{
    uint8_t address = BMI270_REG_INIT_DATA;
    HAL_StatusTypeDef hal_status;
    uint32_t timeout = bmi270_timeout_ms(dev);

    bmi270_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi, &address, 1U, timeout);
    if (hal_status == HAL_OK) {
        hal_status = HAL_SPI_Transmit(dev->bus.hspi,
                                      (uint8_t *)(uintptr_t)drv_bmi270_config_file,
                                      (uint16_t)DRV_BMI270_CONFIG_SIZE,
                                      timeout);
    }
    bmi270_cs(dev, GPIO_PIN_SET);

    return bmi270_from_hal(hal_status);
}

/*
 * 上电或软复位之后，接口模式未定。做一次**丢弃结果**的 CHIP_ID 读，
 * 用那次 CSB 上升沿把芯片锁进 SPI（datasheet：CSB 上升沿后 200 us 生效）。
 */
static void bmi270_enter_spi_mode(DRV_BMI270_Device *dev)
{
    uint8_t discard = 0U;
    (void)bmi270_read(dev, BMI270_REG_CHIP_ID, &discard, 1U);
    bmi270_delay_ms(dev, 1U);
}

/* ============================================================ 公开接口 */

DRV_IMU_Status DRV_BMI270_Probe(DRV_BMI270_Device *dev, uint8_t *chip_id)
{
    DRV_IMU_Status status;
    uint8_t id = 0U;

    if ((dev == NULL) || (dev->bus.hspi == NULL) || (dev->bus.cs_port == NULL)) {
        return DRV_IMU_INVALID_ARG;
    }

    bmi270_cs(dev, GPIO_PIN_SET);
    bmi270_delay_ms(dev, BMI270_POWERUP_DELAY_MS);
    bmi270_enter_spi_mode(dev);

    status = bmi270_read(dev, BMI270_REG_CHIP_ID, &id, 1U);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    dev->chip_id = id;
    if (chip_id != NULL) { *chip_id = id; }

    return (id == DRV_BMI270_CHIP_ID) ? DRV_IMU_OK
                                      : bmi270_fail(dev, DRV_IMU_BAD_ID);
}

/*
 * 灌配置。整个流程有失败出口和明确的重试上限，不会把整机卡在初始化里（D1-3）：
 * 灌不进去就返回 ERROR，由上层决定降级到另一颗 IMU 还是拒飞。
 */
static DRV_IMU_Status bmi270_upload_config(DRV_BMI270_Device *dev)
{
    uint32_t attempt;

    for (attempt = 0U; attempt < BMI270_INIT_MAX_TRIES; attempt++) {
        DRV_IMU_Status status;
        uint8_t internal_status = 0U;

        dev->config_upload_tries = (uint8_t)(attempt + 1U);

        /* 软复位 → 重新进 SPI 模式 → 关省电 → 开配置窗口 → 灌数据 → 关窗口。 */
        (void)bmi270_write(dev, BMI270_REG_CMD, BMI270_CMD_SOFTRESET);
        bmi270_delay_ms(dev, BMI270_POWERUP_DELAY_MS);
        bmi270_enter_spi_mode(dev);

        status = bmi270_write(dev, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_NO_SAVE);
        if (status != DRV_IMU_OK) { continue; }
        bmi270_delay_ms(dev, BMI270_PWR_CONF_DELAY_MS);

        status = bmi270_write(dev, BMI270_REG_INIT_CTRL, 0x00U);
        if (status != DRV_IMU_OK) { continue; }

        status = bmi270_write_config_file(dev);
        if (status != DRV_IMU_OK) { continue; }

        status = bmi270_write(dev, BMI270_REG_INIT_CTRL, 0x01U);
        if (status != DRV_IMU_OK) { continue; }

        bmi270_delay_ms(dev, BMI270_CONFIG_LOAD_DELAY_MS);

        status = bmi270_read(dev, BMI270_REG_INTERNAL_STATUS, &internal_status, 1U);
        if (status != DRV_IMU_OK) { continue; }

        dev->internal_status = internal_status;
        if ((internal_status & 0x01U) == BMI270_INTERNAL_STATUS_OK) {
            return DRV_IMU_OK;
        }
    }

    return DRV_IMU_ERROR;
}

DRV_IMU_Status DRV_BMI270_Init(DRV_BMI270_Device *dev, const DRV_IMU_Config *config)
{
    DRV_IMU_Status status;
    uint8_t acc_conf;
    uint8_t gyr_conf;

    if (dev == NULL) { return DRV_IMU_INVALID_ARG; }

    if (config != NULL) {
        dev->config = *config;
    } else {
        DRV_IMU_DefaultConfig(&dev->config);
    }

    dev->init_stage = DRV_IMU_INIT_STAGE_WHO_AM_I;
    status = DRV_BMI270_Probe(dev, NULL);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_RESET;
    status = bmi270_upload_config(dev);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_ACCEL_CONFIG;
    acc_conf = DRV_BMI270_BuildAccConf(dev->config.accel_odr,
                                       dev->config.accel_aaf_hz,
                                       &dev->accel_bandwidth_actual_hz);
    status = bmi270_write(dev, BMI270_REG_ACC_CONF, acc_conf);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    status = bmi270_write(dev, BMI270_REG_ACC_RANGE,
                          DRV_BMI270_AccelRangeCode(dev->config.accel_range));
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_GYRO_CONFIG;
    gyr_conf = DRV_BMI270_BuildGyrConf(dev->config.gyro_odr,
                                       dev->config.gyro_aaf_hz,
                                       &dev->gyro_bandwidth_actual_hz);
    status = bmi270_write(dev, BMI270_REG_GYR_CONF, gyr_conf);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    /* ois_range 位必须一起置 1，理由见 drv_bmi270_tables.h 的注释。 */
    status = bmi270_write(dev, BMI270_REG_GYR_RANGE,
                          (uint8_t)(DRV_BMI270_GyroRangeCode(dev->config.gyro_range) |
                                    DRV_BMI270_GYR_RANGE_OIS_BIT));
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    /*
     * DRDY 同时映射到 INT1 与 INT2，两个引脚都配成推挽 / 高有效输出。
     * 上游 hwdef 只说 "DRDY3_BMI270" 接在 PB7，没说是 INT1 还是 INT2，
     * 两路都点亮就不必赌。
     */
    dev->init_stage = DRV_IMU_INIT_STAGE_FILTER_CONFIG;
    status = bmi270_write(dev, BMI270_REG_INT1_IO_CTRL, 0x0AU);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }
    status = bmi270_write(dev, BMI270_REG_INT2_IO_CTRL, 0x0AU);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }
    status = bmi270_write(dev, BMI270_REG_INT_MAP_DATA, 0x44U);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_PWR_MGMT;
    status = bmi270_write(dev, BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_ALL_ON);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    status = bmi270_write(dev, BMI270_REG_PWR_CONF, BMI270_PWR_CONF_RUN);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    dev->init_stage = DRV_IMU_INIT_STAGE_READY;
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI270_ReadRaw(DRV_BMI270_Device *dev, DRV_IMU_RawData *raw)
{
    uint8_t buf[12];
    uint8_t temp_buf[2];
    DRV_IMU_Status status;

    if ((dev == NULL) || (raw == NULL)) { return DRV_IMU_INVALID_ARG; }

    /* 加计与陀螺数据寄存器连续，一次突发读 12 字节，省一次总线往返。 */
    status = bmi270_read(dev, BMI270_REG_ACC_DATA_X_LSB, buf, 12U);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    raw->accel_x = bmi270_make_int16(buf[0], buf[1]);
    raw->accel_y = bmi270_make_int16(buf[2], buf[3]);
    raw->accel_z = bmi270_make_int16(buf[4], buf[5]);

    raw->gyro_x = bmi270_make_int16(buf[6], buf[7]);
    raw->gyro_y = bmi270_make_int16(buf[8], buf[9]);
    raw->gyro_z = bmi270_make_int16(buf[10], buf[11]);

    raw->temperature = 0;
    if (dev->config.enable_temp) {
        status = bmi270_read(dev, BMI270_REG_TEMPERATURE_LSB, temp_buf, 2U);
        if (status == DRV_IMU_OK) {
            uint16_t word = (uint16_t)(((uint16_t)temp_buf[1] << 8U) |
                                       (uint16_t)temp_buf[0]);
            /* 0x8000 是"无效"哨兵，不是有效温度，按 0 计数（= 23 °C）处理。 */
            raw->temperature = (word == 0x8000U) ? 0 : (int16_t)word;
        }
    }

    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI270_ReadScaled(DRV_BMI270_Device *dev,
                                     DRV_IMU_ScaledData *scaled)
{
    DRV_IMU_RawData raw;
    DRV_IMU_Status status;

    if ((dev == NULL) || (scaled == NULL)) { return DRV_IMU_INVALID_ARG; }

    status = DRV_BMI270_ReadRaw(dev, &raw);
    if (status != DRV_IMU_OK) { return status; }

    DRV_BMI270_ConvertRaw(dev->config.accel_range, dev->config.gyro_range,
                          &raw, scaled);
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI270_IsDataReady(DRV_BMI270_Device *dev, bool *ready)
{
    uint8_t status_reg = 0U;
    DRV_IMU_Status status;

    if ((dev == NULL) || (ready == NULL)) { return DRV_IMU_INVALID_ARG; }

    /* 节拍由陀螺定（角速率是最内环）。INT_STATUS_1 读后自清，正好是边沿语义。 */
    status = bmi270_read(dev, BMI270_REG_INT_STATUS_1, &status_reg, 1U);
    if (status != DRV_IMU_OK) { return bmi270_fail(dev, status); }

    *ready = ((status_reg & BMI270_GYR_DRDY_MASK) != 0U);
    return DRV_IMU_OK;
}

/* ============================================================ 函数表 */

static DRV_IMU_Status bmi270_ops_probe(void *ctx, uint8_t *chip_id)
{
    return DRV_BMI270_Probe((DRV_BMI270_Device *)ctx, chip_id);
}

static DRV_IMU_Status bmi270_ops_init(void *ctx, const DRV_IMU_Config *config)
{
    return DRV_BMI270_Init((DRV_BMI270_Device *)ctx, config);
}

static DRV_IMU_Status bmi270_ops_read_raw(void *ctx, DRV_IMU_RawData *raw)
{
    return DRV_BMI270_ReadRaw((DRV_BMI270_Device *)ctx, raw);
}

static DRV_IMU_Status bmi270_ops_read_scaled(void *ctx, DRV_IMU_ScaledData *scaled)
{
    return DRV_BMI270_ReadScaled((DRV_BMI270_Device *)ctx, scaled);
}

static DRV_IMU_Status bmi270_ops_is_data_ready(void *ctx, bool *ready)
{
    return DRV_BMI270_IsDataReady((DRV_BMI270_Device *)ctx, ready);
}

static const DRV_IMU_Ops bmi270_ops = {
    .kind          = DRV_IMU_CHIP_BMI270,
    .name          = "BMI270",
    .probe         = bmi270_ops_probe,
    .init          = bmi270_ops_init,
    .read_raw      = bmi270_ops_read_raw,
    .read_scaled   = bmi270_ops_read_scaled,
    .is_data_ready = bmi270_ops_is_data_ready,
};

const DRV_IMU_Ops *DRV_BMI270_GetOps(void)
{
    return &bmi270_ops;
}
