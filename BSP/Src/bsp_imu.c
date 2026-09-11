/*
 * IMU 板级适配：拥有各芯片的设备实例与总线句柄，按顺序探测并选中一颗。
 *
 * 分工（decoupling-spec D1-4）：
 *   Driver/   —— 每颗芯片的寄存器方言（drv_bmi088 / drv_bmi270 / drv_imu）
 *   BSP/      —— 本文件：总线绑定、探测顺序、选中谁（碰 HAL）
 *   Services/ —— svc_imu：芯片轴 → 机体轴的装配变换与记账（不碰 HAL）
 *
 * 探测顺序即优先级：BMI088 → BMI270 → ICM-42688。
 * BMI088 排第一是因为它是 MicoAir743v2 上 ArduPilot 的主 IMU，
 * 加计与陀螺分片、抗振动表现更好，且初始化不需要上传配置固件，
 * 上电路径最短、失败面最小。
 */

#include "bsp_imu.h"
#include "bsp_board.h"

#include "drv_bmi088.h"
#include "drv_bmi270.h"
#include "drv_imu_iface.h"
#include "svc_imu.h"

#include "main.h"
#include "spi.h"

#include <string.h>

static DRV_IMU_Device    icm_dev;
static DRV_BMI088_Device bmi088_dev;
static DRV_BMI270_Device bmi270_dev;

static const DRV_IMU_Ops *imu_ops;
static void              *imu_ctx;
static SVC_IMU_Selection  imu_selection;
static BSP_IMU_Diag       imu_diag;
static uint8_t            imu_initialized;
static DRV_IMU_Config     imu_config;

/*
 * 三颗芯片都用 SPI 模式 3（CPOL=1, CPHA=2edge），与
 * doc/micoair743v2/vendor/ardupilot-hwdef.dat 里的 SPIDEV ... MODE3 一致。
 * 这个函数保留下来是因为老板子上曾出现过模式被别处改掉的情况，
 * 进入探测前强制回到已知状态，比假设"CubeMX 配好了"稳。
 *
 * 同时强制打开 MasterKeepIOState（CFG2 的 AFCNTR）—— 这一条是 2026-09-11 在
 * MicoAir743v2 上实测定位出来的，代价不小，记在这里：
 *
 * 它默认是 DISABLE：SPE=0 期间 SPI 把引脚控制权交还给 GPIO，SCK 不再被钉在
 * CPOL 电平上。下一笔事务时片选先拉低、随后使能 SPE，SCK 这一跳在从机看来
 * 就是一个多余的时钟沿，整串数据从此错位——读回来全是 0x00，而 HAL 一路报成功。
 *
 * 症状极具迷惑性：**一串事务里只有第一笔拿得到数据**，之后全 0；两笔间隔
 * 50 ms 仍失败，隔 100 ms 以上又全部正常（BMI088 自己恢复所需的时间）。
 * 开机探测正好踩中——驱动是"丢弃式首读 + 紧接着真读"，第二笔必败，于是
 * BMI088 被判 bad_id，板上两颗 IMU 只剩 BMI270 能用。
 *
 * SPI3 上的 BMI270 恰好能容忍这个毛刺，所以同一个缺陷在那条总线上从未暴露。
 * 两条总线都强制打开：这不是某颗芯片的偏方，是 H7 上用片选由软件控制的
 * SPI 主机时本就该有的设置。
 */
static void BSP_IMU_ConfigureSpiMode(SPI_HandleTypeDef *hspi,
                                     uint32_t polarity, uint32_t phase)
{
    if ((hspi->Init.CLKPolarity == polarity) &&
        (hspi->Init.CLKPhase == phase) &&
        (hspi->Init.MasterKeepIOState == SPI_MASTER_KEEP_IO_STATE_ENABLE)) {
        return;
    }

    (void)HAL_SPI_DeInit(hspi);
    hspi->Init.CLKPolarity = polarity;
    hspi->Init.CLKPhase = phase;
    hspi->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    (void)HAL_SPI_Init(hspi);
}

static void BSP_IMU_BuildConfig(DRV_IMU_Config *config)
{
    DRV_IMU_DefaultConfig(config);

    /*
     * +-16 g, not +-4 g. Replaying log/flightlog_20260726_021748.csv and
     * log/flightlog_20260727_045138.csv showed raw_accel_* hitting the int16
     * rail (32767 = 4 g) with p99.9 at 78-85 % of full scale, so coaxial-rotor
     * vibration was clipping the accelerometer in powered flight. Clipping is
     * unrecoverable and asymmetric, which biases the very mean that the gravity
     * direction is extracted from: raw accel norm averaged 1.59-2.04 g at hover
     * instead of 1.0. At +-16 g the 2048 LSB/g resolution still resolves about
     * 0.03 deg of tilt, far finer than the errors being chased.
     *
     * 换到 BMI088 后这一档映射成 ±24 g（该芯片没有 ±16 g），余量只多不少。
     */
    config->accel_range = DRV_IMU_ACCEL_RANGE_16G;
    config->gyro_range  = DRV_IMU_GYRO_RANGE_1000DPS;
    config->accel_odr   = DRV_IMU_ODR_1KHZ;
    config->gyro_odr    = DRV_IMU_ODR_1KHZ;
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
     *
     * BMI088 / BMI270 没有独立 AAF，这个数被它们的驱动解释为"期望数字带宽"，
     * 同样按支持档位向下取整；实际生效值见 BSP_IMU_GetInfo()。
     */
    config->accel_aaf_hz = 213U;
    config->gyro_aaf_hz  = 213U;
    config->soft_reset_on_init = true;
}

typedef struct {
    const DRV_IMU_Ops *ops;
    void              *ctx;
} BSP_IMU_Candidate;

static void BSP_IMU_BindBuses(void)
{
    const DRV_BMI088_Bus *bmi088_bus = BSP_Board_GetBmi088Bus();
    const DRV_BMI270_Bus *bmi270_bus = BSP_Board_GetBmi270Bus();
    const DRV_IMU_Bus    *icm_bus    = BSP_Board_GetImuBus();

    memset(&bmi088_dev, 0, sizeof(bmi088_dev));
    memset(&bmi270_dev, 0, sizeof(bmi270_dev));
    memset(&icm_dev, 0, sizeof(icm_dev));

    if (bmi088_bus != NULL) { bmi088_dev.bus = *bmi088_bus; }
    if (bmi270_bus != NULL) { bmi270_dev.bus = *bmi270_bus; }
    if (icm_bus != NULL)    { icm_dev.bus = *icm_bus; }
}

/* 各芯片的 DRDY 引脚。与 Core/Inc/main.h 的标签一致，改板时只改这一处。 */
static uint16_t bsp_imu_drdy_pin_of(DRV_IMU_ChipKind kind)
{
    switch (kind) {
    case DRV_IMU_CHIP_BMI088: return (uint16_t)BMI088_G_DRDY_Pin;
    case DRV_IMU_CHIP_BMI270: return (uint16_t)BMI270_DRDY_Pin;
    default:                  return 0U;   /* ICM-42688 在本板上没有 DRDY 走线 */
    }
}

/*
 * 只放行选中那颗的 DRDY，其余的 EXTI 直接在源头屏蔽。
 *
 * 单靠 App 层比对引脚号也能保证正确，但那样每秒仍要白进几百次中断。更重要的是：
 * 一个永远会被忽略的中断不该处于使能状态——留着它，下一个读代码的人会以为它有用。
 *
 * 用 EXTI 的屏蔽位而不是 HAL_NVIC_DisableIRQ：EXTI15_10 / EXTI9_5 都是多个引脚共用的
 * 中断线，关整条线会牵连无关引脚。也不用 HAL_GPIO_DeInit：那会把引脚打回模拟态，
 * BSP_IMU_Invalidate() 之后重新探测就得再配一遍。
 *
 * EXTI_D1 是 CM7 那一侧的屏蔽寄存器组，与 HAL_GPIO_Init 里写的是同一个
 * （HAL 内部叫 EXTI_CurrentCPU，但那是它 .c 文件里的局部变量，外面用不了）。
 * H743 单核，不存在 CM4 那一侧。
 */
static void BSP_IMU_RouteDrdy(DRV_IMU_ChipKind selected)
{
    static const DRV_IMU_ChipKind all[] = {
        DRV_IMU_CHIP_BMI088, DRV_IMU_CHIP_BMI270
    };
    uint32_t i;

    for (i = 0U; i < (sizeof(all) / sizeof(all[0])); i++) {
        uint16_t pin = bsp_imu_drdy_pin_of(all[i]);

        if (pin == 0U) { continue; }

        if (all[i] == selected) {
            EXTI_D1->IMR1 |= (uint32_t)pin;
        } else {
            EXTI_D1->IMR1 &= ~(uint32_t)pin;
        }
    }
}

uint16_t BSP_IMU_GetDrdyPin(void)
{
    return bsp_imu_drdy_pin_of(BSP_IMU_GetChipKind());
}

DRV_IMU_Status BSP_IMU_Init(void)
{
    const BSP_IMU_Candidate candidates[] = {
        { DRV_BMI088_GetOps(), &bmi088_dev },
        { DRV_BMI270_GetOps(), &bmi270_dev },
        { DRV_IMU_GetOps(),    &icm_dev    },
    };
    const uint32_t candidate_count =
        (uint32_t)(sizeof(candidates) / sizeof(candidates[0]));
    DRV_IMU_Status probe_status[sizeof(candidates) / sizeof(candidates[0])];
    uint8_t        probe_chip_id[sizeof(candidates) / sizeof(candidates[0])];
    /* 一颗都没探到时的默认错误码；探到但配置失败会被真实错误码覆盖。 */
    DRV_IMU_Status init_error = DRV_IMU_BAD_ID;
    uint32_t i;

    if (imu_initialized != 0U) { return DRV_IMU_OK; }

    if (imu_diag.valid == 0U) {
        memset(&imu_diag, 0, sizeof(imu_diag));
        imu_diag.best_mode = 3U;
        imu_diag.best_header = 1U;
        imu_diag.valid = 1U;
    }

    BSP_IMU_ConfigureSpiMode(&hspi2, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);
    BSP_IMU_ConfigureSpiMode(&hspi3, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);

    BSP_IMU_BuildConfig(&imu_config);
    BSP_IMU_BindBuses();
    SVC_IMU_SelectionReset(&imu_selection);

    imu_ops = NULL;
    imu_ctx = NULL;

    /*
     * 第一轮：逐个探测并记账。**任何一颗探不到都不是致命错误**，继续试下一颗；
     * 诊断命令因此能直接说出"哪颗在、读到的 ID 是多少"。
     */
    for (i = 0U; i < candidate_count; i++) {
        const DRV_IMU_Ops *ops = candidates[i].ops;
        uint8_t chip_id = 0U;

        probe_status[i] = DRV_IMU_BAD_ID;
        probe_chip_id[i] = 0U;

        if ((ops == NULL) || (ops->probe == NULL) || (ops->init == NULL)) {
            continue;
        }

        probe_status[i] = ops->probe(candidates[i].ctx, &chip_id);
        probe_chip_id[i] = chip_id;
        SVC_IMU_SelectionRecord(&imu_selection, ops->kind, chip_id,
                                probe_status[i]);
    }

    /*
     * 第二轮：按优先级逐个 init，**谁先配置成功谁上岗**。
     *
     * 探到 ≠ 能用：BMI270 要上传 328 字节配置固件，BMI088 有一串带回读重试的
     * 寄存器配置，两者都可能在 probe 之后失败。以前这里只 init 第一颗探到的芯片，
     * 失败就直接返回——采样任务外层重试时又选回同一颗，无限重试，
     * 旁边那颗完好的备用 IMU 一次都轮不到。板上有两颗 IMU 的意义就没了。
     */
    for (i = 0U; i < candidate_count; i++) {
        DRV_IMU_Status status;

        if (probe_status[i] != DRV_IMU_OK) { continue; }

        status = candidates[i].ops->init(candidates[i].ctx, &imu_config);
        SVC_IMU_SelectionRecordInit(&imu_selection, candidates[i].ops->kind,
                                    status);
        if (status != DRV_IMU_OK) {
            init_error = status;
            continue;   /* 换下一颗，不要卡在这颗上 */
        }

        imu_ops = candidates[i].ops;
        imu_ctx = candidates[i].ctx;
        SVC_IMU_SelectionCommit(&imu_selection, imu_ops->kind, probe_chip_id[i]);
        BSP_IMU_RouteDrdy(imu_ops->kind);
        imu_initialized = 1U;
        return DRV_IMU_OK;
    }

    /*
     * 走到这里说明没有一颗能用。区分两种失败，好让诊断说得准：
     * 压根没探到任何芯片 → BAD_ID（多半是接线或片选错了）；
     * 探到了但都配置失败 → 原样回报最后那颗的错误码。
     */
    imu_ops = NULL;
    imu_ctx = NULL;
    return init_error;
}

DRV_IMU_Status BSP_IMU_ReadRaw(DRV_IMU_RawData *raw)
{
    if ((imu_initialized == 0U) || (imu_ops == NULL)) { return DRV_IMU_ERROR; }
    return imu_ops->read_raw(imu_ctx, raw);
}

DRV_IMU_Status BSP_IMU_ReadScaled(DRV_IMU_ScaledData *scaled)
{
    if ((imu_initialized == 0U) || (imu_ops == NULL)) { return DRV_IMU_ERROR; }
    return imu_ops->read_scaled(imu_ctx, scaled);
}

DRV_IMU_Status BSP_IMU_IsDataReady(bool *ready)
{
    if ((imu_initialized == 0U) || (imu_ops == NULL)) { return DRV_IMU_ERROR; }
    return imu_ops->is_data_ready(imu_ctx, ready);
}

void BSP_IMU_RawToScaled(const DRV_IMU_RawData *raw, DRV_IMU_ScaledData *scaled)
{
    if ((raw == NULL) || (scaled == NULL)) { return; }

    switch (BSP_IMU_GetChipKind()) {
    case DRV_IMU_CHIP_BMI088:
        DRV_BMI088_ConvertRaw(bmi088_dev.config.accel_range,
                              bmi088_dev.config.gyro_range, raw, scaled);
        break;
    case DRV_IMU_CHIP_BMI270:
        DRV_BMI270_ConvertRaw(bmi270_dev.config.accel_range,
                              bmi270_dev.config.gyro_range, raw, scaled);
        break;
    case DRV_IMU_CHIP_ICM42688:
        DRV_IMU_ConvertRaw(&icm_dev, raw, scaled);
        break;
    case DRV_IMU_CHIP_NONE:
    default:
        /*
         * 还没选中任何芯片时，按当前请求的量程用 ICM 刻度换算。
         * 这条路径只在初始化失败后被走到，数值本身没有意义，
         * 但保证输出是有限值而不是未初始化内存。
         */
        memset(scaled, 0, sizeof(*scaled));
        break;
    }
}

DRV_IMU_ChipKind BSP_IMU_GetChipKind(void)
{
    return (imu_ops != NULL) ? imu_ops->kind : DRV_IMU_CHIP_NONE;
}

void BSP_IMU_GetInfo(BSP_IMU_Info *info)
{
    if (info == NULL) { return; }

    memset(info, 0, sizeof(*info));
    info->kind = BSP_IMU_GetChipKind();
    info->chip_id = imu_selection.selected_chip_id;
    info->initialized = imu_initialized;
    info->accel_range = imu_config.accel_range;
    info->gyro_range = imu_config.gyro_range;

    switch (info->kind) {
    case DRV_IMU_CHIP_BMI088:
        info->init_stage = bmi088_dev.init_stage;
        info->last_error = bmi088_dev.last_error;
        info->accel_bandwidth_hz = bmi088_dev.accel_bandwidth_actual_hz;
        info->gyro_bandwidth_hz = bmi088_dev.gyro_bandwidth_actual_hz;
        break;
    case DRV_IMU_CHIP_BMI270:
        info->init_stage = bmi270_dev.init_stage;
        info->last_error = bmi270_dev.last_error;
        info->accel_bandwidth_hz = bmi270_dev.accel_bandwidth_actual_hz;
        info->gyro_bandwidth_hz = bmi270_dev.gyro_bandwidth_actual_hz;
        break;
    case DRV_IMU_CHIP_ICM42688:
        info->init_stage = icm_dev.init_stage;
        info->last_error = icm_dev.last_error;
        info->accel_bandwidth_hz = icm_dev.accel_aaf_actual_hz;
        info->gyro_bandwidth_hz = icm_dev.gyro_aaf_actual_hz;
        break;
    case DRV_IMU_CHIP_NONE:
    default:
        info->init_stage = DRV_IMU_INIT_STAGE_NONE;
        info->last_error = DRV_IMU_BAD_ID;
        break;
    }
}

void BSP_IMU_GetSelection(SVC_IMU_Selection *selection)
{
    if (selection != NULL) { *selection = imu_selection; }
}

uint8_t BSP_IMU_GetWhoAmI(void)
{
    return imu_selection.selected_chip_id;
}

void BSP_IMU_GetDiag(BSP_IMU_Diag *diag)
{
    if (diag != NULL) { *diag = imu_diag; }
}

const DRV_IMU_Device *BSP_IMU_GetDevice(void)
{
    /* 只有真的选中 ICM-42688 时这个结构体才有意义。 */
    return (BSP_IMU_GetChipKind() == DRV_IMU_CHIP_ICM42688) ? &icm_dev : NULL;
}

void BSP_IMU_Invalidate(void)
{
    memset(&icm_dev, 0, sizeof(icm_dev));
    memset(&bmi088_dev, 0, sizeof(bmi088_dev));
    memset(&bmi270_dev, 0, sizeof(bmi270_dev));
    SVC_IMU_SelectionReset(&imu_selection);
    imu_ops = NULL;
    imu_ctx = NULL;
    imu_initialized = 0U;
}

/*
 * BMI088 背靠背读取自检（诊断专用，不参与采样路径）。
 *
 * 2026-09-10 首刷 MicoAir743v2 时定性出的缺陷，留在这里当回归探针：
 * BMI088 两颗（加计 PD4 → 0x1E、陀螺 PD5 → 0x0F）在 SPI2 上都能正确应答，
 * 但**一串事务里只有第一笔拿得到数据**，之后全读 0x00；两笔之间隔 50 ms 仍失败，
 * 隔 100 ms 及以上则全部成功。已排除：引脚复用与 PC2/PC3 模拟开关、SPI 模式与
 * 分频、RX FIFO 残留、SPI2 外设状态（连 RCC 硬复位都救不回来）、
 * 生成代码里残留的 PC1/PA9 复用、SPI2 的 NVIC 中断线。
 *
 * 四笔背靠背读加计 CHIP_ID。修好之后这里应当四笔全是 0x1E。
 */
void BSP_IMU_DebugRawBmi088(BSP_IMU_RawProbe *out)
{
    uint32_t i;

    if (out == NULL) { return; }
    memset(out, 0, sizeof(*out));

    out->sr_before = SPI2->SR;



    for (i = 0U; i < 4U; i++) {
        uint8_t tx[4] = { 0x80U, 0U, 0U, 0U };   /* 寄存器 0x00 | 读位 */
        uint8_t rx[4] = { 0U, 0U, 0U, 0U };

        HAL_GPIO_WritePin(BMI088_A_CS_GPIO_Port, BMI088_A_CS_Pin, GPIO_PIN_RESET);
        out->hal[i] = (uint8_t)HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3U, 10U);
        HAL_GPIO_WritePin(BMI088_A_CS_GPIO_Port, BMI088_A_CS_Pin, GPIO_PIN_SET);

        /* 加计读有一个 dummy 字节，有效数据在 rx[2]。 */
        out->chip_id[i] = rx[2];
        out->sr[i] = SPI2->SR;
    }

    out->sr_after = SPI2->SR;
}

/* ============================================================ 诊断：任意 SPI 事务 */

/* 诊断路径自己的超时：比采样路径的 5 ms 宽松，卡住也只影响这条命令。 */
#define BSP_IMU_DEBUG_XFER_TIMEOUT_MS 10U

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *port;
    uint16_t           pin;
    uint8_t            bus_index;
} BSP_IMU_CsEntry;

static const BSP_IMU_CsEntry *bsp_imu_cs_entry(BSP_IMU_SpiCs cs)
{
    static BSP_IMU_CsEntry table[BSP_IMU_SPI_CS_COUNT];

    table[BSP_IMU_SPI_CS_BMI088_ACC]  = (BSP_IMU_CsEntry){
        &hspi2, BMI088_A_CS_GPIO_Port, BMI088_A_CS_Pin, 2U };
    table[BSP_IMU_SPI_CS_BMI088_GYRO] = (BSP_IMU_CsEntry){
        &hspi2, BMI088_G_CS_GPIO_Port, BMI088_G_CS_Pin, 2U };
    table[BSP_IMU_SPI_CS_BMI270]      = (BSP_IMU_CsEntry){
        &hspi3, BMI270_CS_GPIO_Port, BMI270_CS_Pin, 3U };

    return ((uint32_t)cs < (uint32_t)BSP_IMU_SPI_CS_COUNT) ? &table[cs] : NULL;
}

uint8_t BSP_IMU_DebugSpiBusIndex(BSP_IMU_SpiCs cs)
{
    const BSP_IMU_CsEntry *e = bsp_imu_cs_entry(cs);
    return (e != NULL) ? e->bus_index : 0U;
}

/*
 * 让上位机能直接打一笔 SPI 事务，不必为每个实验重新编译固件。
 *
 * 2026-09-11 定位 BMI088 那个"只有第一笔读得到数据"的缺陷时，十几轮试验每一轮
 * 都要改代码 → 编译 → 跳 DFU → 烧录 → 重启，一轮约 40 秒。真正变的往往只是
 * 几个字节的收发内容。有了这个出口，那类实验在串口上就做完了。
 *
 * 刻意不做的事：不碰采样路径、不改任何寄存器配置、不拨 IMU 以外的 GPIO。
 * 是否允许调用（解锁状态等）由 App 层把关，这里只负责老实地收发。
 */
DRV_IMU_Status BSP_IMU_DebugSpiXfer(BSP_IMU_SpiCs cs, const uint8_t *tx,
                                    uint8_t *rx, uint16_t len)
{
    const BSP_IMU_CsEntry *e = bsp_imu_cs_entry(cs);
    HAL_StatusTypeDef hal;

    if ((e == NULL) || (tx == NULL) || (rx == NULL) ||
        (len == 0U) || (len > (uint16_t)BSP_IMU_SPI_XFER_MAX)) {
        return DRV_IMU_INVALID_ARG;
    }

    HAL_GPIO_WritePin(e->port, e->pin, GPIO_PIN_RESET);
    hal = HAL_SPI_TransmitReceive(e->hspi, (uint8_t *)(uintptr_t)tx, rx, len,
                                  BSP_IMU_DEBUG_XFER_TIMEOUT_MS);
    HAL_GPIO_WritePin(e->port, e->pin, GPIO_PIN_SET);

    switch (hal) {
    case HAL_OK:      return DRV_IMU_OK;
    case HAL_TIMEOUT: return DRV_IMU_TIMEOUT;
    default:          return DRV_IMU_ERROR;
    }
}
