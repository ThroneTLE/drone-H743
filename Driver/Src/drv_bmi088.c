/*
 * Bosch BMI088 驱动实现。
 *
 * 寄存器与时序按 BST-BMI088-DS001 datasheet，并与 ArduPilot
 * `AP_InertialSensor_BMI088.cpp` 交叉核对（加计跳字节、写寄存器重试、
 * 陀螺软复位 30 ms 等易错点都以那份实测实现为准）。
 */

#include "drv_bmi088.h"

#include <string.h>

/* ---- 加速度计寄存器（独立片选，独立地址空间） ---- */
#define BMI088_ACC_REG_CHIP_ID      0x00U
#define BMI088_ACC_REG_STATUS       0x03U
#define BMI088_ACC_REG_X_LSB        0x12U
#define BMI088_ACC_REG_INT_STAT_1   0x1DU
#define BMI088_ACC_REG_TEMP_MSB     0x22U
#define BMI088_ACC_REG_CONF         0x40U
#define BMI088_ACC_REG_RANGE        0x41U
#define BMI088_ACC_REG_INT1_IO_CTRL 0x53U
#define BMI088_ACC_REG_INT2_IO_CTRL 0x54U
#define BMI088_ACC_REG_INT_MAP_DATA 0x58U
#define BMI088_ACC_REG_PWR_CONF     0x7CU
#define BMI088_ACC_REG_PWR_CTRL     0x7DU
#define BMI088_ACC_REG_SOFTRESET    0x7EU

/* ---- 陀螺仪寄存器 ---- */
#define BMI088_GYRO_REG_CHIP_ID       0x00U
#define BMI088_GYRO_REG_RATE_X_LSB    0x02U
#define BMI088_GYRO_REG_INT_STAT_1    0x0AU
#define BMI088_GYRO_REG_RANGE         0x0FU
#define BMI088_GYRO_REG_BANDWIDTH     0x10U
#define BMI088_GYRO_REG_LPM1          0x11U
#define BMI088_GYRO_REG_RATE_HBW      0x13U
#define BMI088_GYRO_REG_SOFTRESET     0x14U
#define BMI088_GYRO_REG_INT_CTRL      0x15U
#define BMI088_GYRO_REG_INT3_IO_CONF  0x16U
#define BMI088_GYRO_REG_INT3_IO_MAP   0x18U

#define BMI088_SPI_READ_BIT         0x80U
#define BMI088_SOFTRESET_CMD        0xB6U
#define BMI088_ACC_PWR_CTRL_ON      0x04U
#define BMI088_ACC_PWR_CONF_ACTIVE  0x00U
#define BMI088_GYRO_BW_RESERVED_BIT 0x80U
#define BMI088_GYRO_INT_CTRL_DRDY   0x80U
#define BMI088_GYRO_DRDY_MASK       0x80U
#define BMI088_ACC_DRDY_MASK        0x80U

#define BMI088_DEFAULT_TIMEOUT_MS   100U
#define BMI088_POWER_UP_DELAY_MS      5U
#define BMI088_ACC_RESET_DELAY_MS     5U
#define BMI088_ACC_PWR_DELAY_MS      50U
#define BMI088_GYRO_RESET_DELAY_MS   30U
#define BMI088_ACC_WRITE_RETRIES      8U
/* datasheet：写电源寄存器后至少 450 us 才能再访问。延时函数只有 ms 粒度，取 1。 */
#define BMI088_ACC_WRITE_SETTLE_MS    1U

/*
 * 加计 SPI 读要在地址字节后丢弃一个 dummy 字节，所以缓冲比数据长 2。
 * 最长一次读 6 字节（三轴），留到 16 有余量。
 */
#define BMI088_ACC_XFER_MAX  (16U + 2U)

/* ============================================================ 基础工具 */

static uint32_t bmi088_timeout_ms(const DRV_BMI088_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms
                                       : BMI088_DEFAULT_TIMEOUT_MS;
}

static void bmi088_delay_ms(const DRV_BMI088_Device *dev, uint32_t ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(ms);
    } else {
        HAL_Delay(ms);
    }
}

static DRV_IMU_Status bmi088_from_hal(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_IMU_OK;
    case HAL_TIMEOUT: return DRV_IMU_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:          return DRV_IMU_ERROR;
    }
}

static DRV_IMU_Status bmi088_fail(DRV_BMI088_Device *dev, DRV_IMU_Status status)
{
    if (dev != NULL) { dev->last_error = status; }
    return status;
}

static void bmi088_acc_cs(DRV_BMI088_Device *dev, GPIO_PinState state)
{
    HAL_GPIO_WritePin(dev->bus.acc_cs_port, dev->bus.acc_cs_pin, state);
}

static void bmi088_gyro_cs(DRV_BMI088_Device *dev, GPIO_PinState state)
{
    HAL_GPIO_WritePin(dev->bus.gyro_cs_port, dev->bus.gyro_cs_pin, state);
}

static int16_t bmi088_make_int16(uint8_t lsb, uint8_t msb)
{
    /* BMI088 是小端：先 LSB 后 MSB，和 ICM-42688 相反，写反了姿态会整个乱掉。 */
    return (int16_t)(((uint16_t)msb << 8U) | (uint16_t)lsb);
}

/* ============================================================ 陀螺传输 */

static DRV_IMU_Status bmi088_gyro_read(DRV_BMI088_Device *dev, uint8_t reg,
                                       uint8_t *data, uint16_t len)
{
    /*
     * 单次全双工事务，不是 Transmit 后再 Receive。
     *
     * 在 STM32H7 上，全双工主机模式下 HAL_SPI_Transmit 发地址字节的同时也会把
     * 对方那一拍的数据收进 RX FIFO，而 HAL 不会把它清掉；紧接着的 HAL_SPI_Receive
     * 于是先交出那个陈字节，整串数据错位一格。本文件其余读路径（加计）以及
     * drv_bmi270.c、drv_imu.c 都用的是 TransmitReceive，只有这里是两段式。
     *
     * 陀螺读没有 dummy 字节：发 1 字节地址，数据紧跟其后，所以有效字节从 rx[1] 起。
     */
    uint8_t tx[BMI088_ACC_XFER_MAX];
    uint8_t rx[BMI088_ACC_XFER_MAX];
    uint16_t total = (uint16_t)(len + 1U);
    HAL_StatusTypeDef hal_status;

    if (total > (uint16_t)BMI088_ACC_XFER_MAX) { return DRV_IMU_INVALID_ARG; }

    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | BMI088_SPI_READ_BIT);

    bmi088_gyro_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi, tx, rx, total,
                                         bmi088_timeout_ms(dev));
    bmi088_gyro_cs(dev, GPIO_PIN_SET);

    if (hal_status != HAL_OK) { return bmi088_from_hal(hal_status); }

    memcpy(data, &rx[1], len);
    return DRV_IMU_OK;
}

/* 同样用收发等长的事务，理由见下面 bmi088_acc_write_raw() 的注释。 */
static DRV_IMU_Status bmi088_gyro_write(DRV_BMI088_Device *dev, uint8_t reg,
                                        uint8_t value)
{
    uint8_t frame[2] = { (uint8_t)(reg & (uint8_t)~BMI088_SPI_READ_BIT), value };
    uint8_t discard[2] = { 0U, 0U };
    HAL_StatusTypeDef hal_status;

    bmi088_gyro_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi, frame, discard, 2U,
                                         bmi088_timeout_ms(dev));
    bmi088_gyro_cs(dev, GPIO_PIN_SET);

    return bmi088_from_hal(hal_status);
}

/* ============================================================ 加计传输 */

/*
 * 加计的 SPI 读：地址字节之后**第一个返回字节是无效的**，真数据从第二个开始。
 * 这是 BMI088 加计部分的硬件特性（datasheet 第 3 节），不是可选优化——
 * 不跳这个字节会读到整体错位一位的垃圾，而且看起来还挺"像数据"。
 */
static DRV_IMU_Status bmi088_acc_read(DRV_BMI088_Device *dev, uint8_t reg,
                                      uint8_t *data, uint16_t len)
{
    uint8_t tx[BMI088_ACC_XFER_MAX];
    uint8_t rx[BMI088_ACC_XFER_MAX];
    uint16_t total = (uint16_t)(len + 2U);
    HAL_StatusTypeDef hal_status;

    if (total > (uint16_t)BMI088_ACC_XFER_MAX) { return DRV_IMU_INVALID_ARG; }

    memset(tx, 0, sizeof(tx));
    tx[0] = (uint8_t)(reg | BMI088_SPI_READ_BIT);

    bmi088_acc_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi, tx, rx, total,
                                         bmi088_timeout_ms(dev));
    bmi088_acc_cs(dev, GPIO_PIN_SET);

    if (hal_status != HAL_OK) { return bmi088_from_hal(hal_status); }

    memcpy(data, &rx[2], len);
    return DRV_IMU_OK;
}

/*
 * 寄存器写也走 TransmitReceive，不是 Transmit。
 *
 * H7 的 SPI 在全双工主机模式下，发出去每一个字节的同时也会收进一个字节，而
 * HAL_SPI_Transmit 结束时（SPI_CloseTransfer）**不清 RX FIFO**——那两个字节就
 * 一直留在里面。紧接着的回读于是先把陈字节交出来，整串错位一格，校验永远不符：
 * bmi088_acc_write() 重试 8 次之后返回 ERROR，表现为 BMI088 探测通过却初始化失败。
 *
 * 2026-09-11 实机验证：用同样收发等长的事务写 ACC_RANGE 再立刻回读，0x01 → 0x03
 * 完全正确；而驱动原来的 Transmit + 回读必失败。rx 收下来就丢，只为让收发配平。
 */
static DRV_IMU_Status bmi088_acc_write_raw(DRV_BMI088_Device *dev, uint8_t reg,
                                           uint8_t value)
{
    uint8_t frame[2] = { (uint8_t)(reg & (uint8_t)~BMI088_SPI_READ_BIT), value };
    uint8_t discard[2] = { 0U, 0U };
    HAL_StatusTypeDef hal_status;

    bmi088_acc_cs(dev, GPIO_PIN_RESET);
    hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi, frame, discard, 2U,
                                         bmi088_timeout_ms(dev));
    bmi088_acc_cs(dev, GPIO_PIN_SET);

    return bmi088_from_hal(hal_status);
}

/*
 * 加计寄存器写入需要回读确认并重试。
 *
 * 这不是防御性冗余：BMI088 加计在 SPI 下对写入的接收窗口很窄，单次写有实测可见
 * 的失败率（ArduPilot 同样按 8 次重试处理）。配置寄存器写丢了不会报错，
 * 只会让量程或带宽停在上电默认值——那是一个安静的、要飞起来才发现的错误。
 * 重试有明确上限，不是无界等待（decoupling-spec D1-3）。
 */
static DRV_IMU_Status bmi088_acc_write(DRV_BMI088_Device *dev, uint8_t reg,
                                       uint8_t value)
{
    uint32_t attempt;

    for (attempt = 0U; attempt < BMI088_ACC_WRITE_RETRIES; attempt++) {
        uint8_t readback = 0U;
        DRV_IMU_Status status = bmi088_acc_write_raw(dev, reg, value);

        if (status != DRV_IMU_OK) { return status; }

        /*
         * 写完必须让芯片先把值吃进去，再回读。
         *
         * datasheet 要求写 ACC_PWR_CONF / ACC_PWR_CTRL 之后至少 450 µs 才能再访问，
         * 而这里原本是写完立刻回读——8 次重试挤在一起，很可能整段都落在那个窗口里，
         * 于是回读永远读到旧值，函数返回 ERROR。表现为**初始化时好时坏**：
         * 2026-09-11 连续复位实测约每三四次失败一次，失败时 BMI088 探测通过却
         * 上不了岗，只能退到 BMI270。这种不确定性比干脆失败更难查，
         * 也更不能带上天——两颗 IMU 的安装旋转与量程刻度都不一样。
         *
         * 延时放在这里而不是只包住那两个电源寄存器：init 一共七次写，多花 7 ms，
         * 换掉一个按寄存器名分叉的特例，值。
         */
        bmi088_delay_ms(dev, BMI088_ACC_WRITE_SETTLE_MS);

        status = bmi088_acc_read(dev, reg, &readback, 1U);
        if (status != DRV_IMU_OK) { return status; }
        if (readback == value) { return DRV_IMU_OK; }
    }

    return DRV_IMU_ERROR;
}

/*
 * 加计上电后总线模式未定，必须先做一次**丢弃结果**的 CHIP_ID 读，用那次 CSB
 * 上升沿把它锁进 SPI 模式。软复位之后同样要再来一次，否则后续全读回 0x00。
 */
static void bmi088_acc_enter_spi_mode(DRV_BMI088_Device *dev)
{
    uint8_t discard = 0U;
    (void)bmi088_acc_read(dev, BMI088_ACC_REG_CHIP_ID, &discard, 1U);
}

/* ============================================================ 公开接口 */

DRV_IMU_Status DRV_BMI088_Probe(DRV_BMI088_Device *dev, uint8_t *chip_id)
{
    DRV_IMU_Status status;
    uint8_t acc_id = 0U;
    uint8_t gyro_id = 0U;

    if ((dev == NULL) || (dev->bus.hspi == NULL) ||
        (dev->bus.acc_cs_port == NULL) || (dev->bus.gyro_cs_port == NULL)) {
        return DRV_IMU_INVALID_ARG;
    }

    bmi088_acc_cs(dev, GPIO_PIN_SET);
    bmi088_gyro_cs(dev, GPIO_PIN_SET);
    bmi088_delay_ms(dev, BMI088_POWER_UP_DELAY_MS);

    bmi088_acc_enter_spi_mode(dev);

    status = bmi088_acc_read(dev, BMI088_ACC_REG_CHIP_ID, &acc_id, 1U);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }
    dev->acc_chip_id = acc_id;
    if (chip_id != NULL) { *chip_id = acc_id; }

    /* 0x1E = BMI088，0x1F = 引脚兼容的 BMI085（量程 ±16 g）。两者都收。 */
    if ((acc_id != DRV_BMI088_ACC_CHIP_ID) && (acc_id != 0x1FU)) {
        return bmi088_fail(dev, DRV_IMU_BAD_ID);
    }

    status = bmi088_gyro_read(dev, BMI088_GYRO_REG_CHIP_ID, &gyro_id, 1U);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }
    dev->gyro_chip_id = gyro_id;

    if (gyro_id != DRV_BMI088_GYRO_CHIP_ID) {
        return bmi088_fail(dev, DRV_IMU_BAD_ID);
    }

    return DRV_IMU_OK;
}

static DRV_IMU_Status bmi088_init_accel(DRV_BMI088_Device *dev)
{
    DRV_IMU_Status status;
    uint8_t acc_conf;

    if (dev->config.soft_reset_on_init) {
        dev->init_stage = DRV_IMU_INIT_STAGE_RESET;
        (void)bmi088_acc_write_raw(dev, BMI088_ACC_REG_SOFTRESET,
                                   BMI088_SOFTRESET_CMD);
        bmi088_delay_ms(dev, BMI088_ACC_RESET_DELAY_MS);
        bmi088_acc_enter_spi_mode(dev);
    }

    /* 先离开 suspend，再开加计电源；顺序反了配置会写不进去。 */
    dev->init_stage = DRV_IMU_INIT_STAGE_PWR_MGMT;
    status = bmi088_acc_write(dev, BMI088_ACC_REG_PWR_CONF,
                              BMI088_ACC_PWR_CONF_ACTIVE);
    if (status != DRV_IMU_OK) { return status; }

    status = bmi088_acc_write(dev, BMI088_ACC_REG_PWR_CTRL,
                              BMI088_ACC_PWR_CTRL_ON);
    if (status != DRV_IMU_OK) { return status; }
    bmi088_delay_ms(dev, BMI088_ACC_PWR_DELAY_MS);

    dev->init_stage = DRV_IMU_INIT_STAGE_ACCEL_CONFIG;
    acc_conf = DRV_BMI088_BuildAccConf(dev->config.accel_odr,
                                       dev->config.accel_aaf_hz,
                                       &dev->accel_bandwidth_actual_hz);
    status = bmi088_acc_write(dev, BMI088_ACC_REG_CONF, acc_conf);
    if (status != DRV_IMU_OK) { return status; }

    status = bmi088_acc_write(dev, BMI088_ACC_REG_RANGE,
                              DRV_BMI088_AccelRangeCode(dev->config.accel_range));
    if (status != DRV_IMU_OK) { return status; }

    /*
     * DRDY 同时映射到 INT1 和 INT2，两个引脚都配成推挽 / 高有效。
     *
     * 板子把哪一路引到 MCU 的 EXTI 上，上游 hwdef 只写了 "DRDY2_BMI088_A"，
     * 没说是 INT1 还是 INT2。两路都点亮就不必赌，代价只是一个没接线的引脚在空跳。
     */
    dev->init_stage = DRV_IMU_INIT_STAGE_FILTER_CONFIG;
    status = bmi088_acc_write(dev, BMI088_ACC_REG_INT1_IO_CTRL, 0x0CU);
    if (status != DRV_IMU_OK) { return status; }
    status = bmi088_acc_write(dev, BMI088_ACC_REG_INT2_IO_CTRL, 0x0CU);
    if (status != DRV_IMU_OK) { return status; }
    status = bmi088_acc_write(dev, BMI088_ACC_REG_INT_MAP_DATA, 0x44U);
    if (status != DRV_IMU_OK) { return status; }

    return DRV_IMU_OK;
}

static DRV_IMU_Status bmi088_init_gyro(DRV_BMI088_Device *dev)
{
    DRV_IMU_Status status;
    uint8_t bw_code;

    /*
     * 陀螺软复位的写入本身经常不返回 ACK（datasheet 已知行为），
     * 所以不检查返回值，只靠之后的 30 ms 等待和寄存器回读来确认。
     */
    dev->init_stage = DRV_IMU_INIT_STAGE_RESET;
    (void)bmi088_gyro_write(dev, BMI088_GYRO_REG_SOFTRESET, BMI088_SOFTRESET_CMD);
    bmi088_delay_ms(dev, BMI088_GYRO_RESET_DELAY_MS);

    dev->init_stage = DRV_IMU_INIT_STAGE_GYRO_CONFIG;
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_RANGE,
                               DRV_BMI088_GyroRangeCode(dev->config.gyro_range));
    if (status != DRV_IMU_OK) { return status; }

    bw_code = DRV_BMI088_GyroBandwidthCode(dev->config.gyro_odr,
                                           dev->config.gyro_aaf_hz,
                                           NULL,
                                           &dev->gyro_bandwidth_actual_hz);
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_BANDWIDTH,
                               (uint8_t)(bw_code | BMI088_GYRO_BW_RESERVED_BIT));
    if (status != DRV_IMU_OK) { return status; }

    /* 正常功耗模式 + 取滤波后的数据（RATE_HBW=0），不要未滤波的高带宽通路。 */
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_LPM1, 0x00U);
    if (status != DRV_IMU_OK) { return status; }
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_RATE_HBW, 0x00U);
    if (status != DRV_IMU_OK) { return status; }

    /* 与加计同理：DRDY 同时映射到 INT3 和 INT4，两路都配成推挽 / 高有效。 */
    dev->init_stage = DRV_IMU_INIT_STAGE_FILTER_CONFIG;
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_INT_CTRL,
                               BMI088_GYRO_INT_CTRL_DRDY);
    if (status != DRV_IMU_OK) { return status; }
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_INT3_IO_CONF, 0x05U);
    if (status != DRV_IMU_OK) { return status; }
    status = bmi088_gyro_write(dev, BMI088_GYRO_REG_INT3_IO_MAP, 0x81U);
    if (status != DRV_IMU_OK) { return status; }

    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI088_Init(DRV_BMI088_Device *dev, const DRV_IMU_Config *config)
{
    DRV_IMU_Status status;

    if (dev == NULL) { return DRV_IMU_INVALID_ARG; }

    if (config != NULL) {
        dev->config = *config;
    } else {
        DRV_IMU_DefaultConfig(&dev->config);
    }

    dev->init_stage = DRV_IMU_INIT_STAGE_WHO_AM_I;
    status = DRV_BMI088_Probe(dev, NULL);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }

    status = bmi088_init_accel(dev);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }

    status = bmi088_init_gyro(dev);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }

    dev->accel_lsb_per_g = DRV_BMI088_AccelLsbPerG(dev->config.accel_range);
    dev->gyro_lsb_per_dps = DRV_BMI088_GyroLsbPerDps(dev->config.gyro_range);

    dev->init_stage = DRV_IMU_INIT_STAGE_READY;
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI088_ReadRaw(DRV_BMI088_Device *dev, DRV_IMU_RawData *raw)
{
    uint8_t accel_buf[6];
    uint8_t gyro_buf[6];
    uint8_t temp_buf[2];
    DRV_IMU_Status status;

    if ((dev == NULL) || (raw == NULL)) { return DRV_IMU_INVALID_ARG; }

    status = bmi088_acc_read(dev, BMI088_ACC_REG_X_LSB, accel_buf, 6U);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }

    status = bmi088_gyro_read(dev, BMI088_GYRO_REG_RATE_X_LSB, gyro_buf, 6U);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }

    raw->accel_x = bmi088_make_int16(accel_buf[0], accel_buf[1]);
    raw->accel_y = bmi088_make_int16(accel_buf[2], accel_buf[3]);
    raw->accel_z = bmi088_make_int16(accel_buf[4], accel_buf[5]);

    raw->gyro_x = bmi088_make_int16(gyro_buf[0], gyro_buf[1]);
    raw->gyro_y = bmi088_make_int16(gyro_buf[2], gyro_buf[3]);
    raw->gyro_z = bmi088_make_int16(gyro_buf[4], gyro_buf[5]);

    raw->temperature = 0;
    if (dev->config.enable_temp) {
        status = bmi088_acc_read(dev, BMI088_ACC_REG_TEMP_MSB, temp_buf, 2U);
        if (status == DRV_IMU_OK) {
            /* 11 位有符号，低 3 位在 LSB 寄存器的高位上。 */
            int32_t value = ((int32_t)temp_buf[0] * 8) +
                            (((int32_t)temp_buf[1] >> 5) & 0x07);
            if (value > 1023) { value -= 2048; }
            raw->temperature = (int16_t)value;
        }
    }

    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI088_ReadScaled(DRV_BMI088_Device *dev,
                                     DRV_IMU_ScaledData *scaled)
{
    DRV_IMU_RawData raw;
    DRV_IMU_Status status;

    if ((dev == NULL) || (scaled == NULL)) { return DRV_IMU_INVALID_ARG; }

    status = DRV_BMI088_ReadRaw(dev, &raw);
    if (status != DRV_IMU_OK) { return status; }

    DRV_BMI088_ConvertRaw(dev->config.accel_range, dev->config.gyro_range,
                          &raw, scaled);
    return DRV_IMU_OK;
}

DRV_IMU_Status DRV_BMI088_IsDataReady(DRV_BMI088_Device *dev, bool *ready)
{
    uint8_t status_reg = 0U;
    DRV_IMU_Status status;

    if ((dev == NULL) || (ready == NULL)) { return DRV_IMU_INVALID_ARG; }

    /* 节拍由陀螺定（角速率是最内环），所以只看陀螺的 DRDY。 */
    status = bmi088_gyro_read(dev, BMI088_GYRO_REG_INT_STAT_1, &status_reg, 1U);
    if (status != DRV_IMU_OK) { return bmi088_fail(dev, status); }

    *ready = ((status_reg & BMI088_GYRO_DRDY_MASK) != 0U);
    return DRV_IMU_OK;
}

/* ============================================================ 函数表 */

static DRV_IMU_Status bmi088_ops_probe(void *ctx, uint8_t *chip_id)
{
    return DRV_BMI088_Probe((DRV_BMI088_Device *)ctx, chip_id);
}

static DRV_IMU_Status bmi088_ops_init(void *ctx, const DRV_IMU_Config *config)
{
    return DRV_BMI088_Init((DRV_BMI088_Device *)ctx, config);
}

static DRV_IMU_Status bmi088_ops_read_raw(void *ctx, DRV_IMU_RawData *raw)
{
    return DRV_BMI088_ReadRaw((DRV_BMI088_Device *)ctx, raw);
}

static DRV_IMU_Status bmi088_ops_read_scaled(void *ctx, DRV_IMU_ScaledData *scaled)
{
    return DRV_BMI088_ReadScaled((DRV_BMI088_Device *)ctx, scaled);
}

static DRV_IMU_Status bmi088_ops_is_data_ready(void *ctx, bool *ready)
{
    return DRV_BMI088_IsDataReady((DRV_BMI088_Device *)ctx, ready);
}

static const DRV_IMU_Ops bmi088_ops = {
    .kind          = DRV_IMU_CHIP_BMI088,
    .name          = "BMI088",
    .probe         = bmi088_ops_probe,
    .init          = bmi088_ops_init,
    .read_raw      = bmi088_ops_read_raw,
    .read_scaled   = bmi088_ops_read_scaled,
    .is_data_ready = bmi088_ops_is_data_ready,
};

const DRV_IMU_Ops *DRV_BMI088_GetOps(void)
{
    return &bmi088_ops;
}
