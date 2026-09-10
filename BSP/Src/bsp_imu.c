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
 */
static void BSP_IMU_ConfigureSpiMode(SPI_HandleTypeDef *hspi,
                                     uint32_t polarity, uint32_t phase)
{
    if ((hspi->Init.CLKPolarity == polarity) &&
        (hspi->Init.CLKPhase == phase)) {
        return;
    }

    (void)HAL_SPI_DeInit(hspi);
    hspi->Init.CLKPolarity = polarity;
    hspi->Init.CLKPhase = phase;
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

DRV_IMU_Status BSP_IMU_Init(void)
{
    const BSP_IMU_Candidate candidates[] = {
        { DRV_BMI088_GetOps(), &bmi088_dev },
        { DRV_BMI270_GetOps(), &bmi270_dev },
        { DRV_IMU_GetOps(),    &icm_dev    },
    };
    const uint32_t candidate_count =
        (uint32_t)(sizeof(candidates) / sizeof(candidates[0]));
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
     * 逐个探测。**任何一颗探不到都不是致命错误**，继续试下一颗；
     * 全部探不到才返回失败，由上层决定拒飞还是降级（decoupling-spec D1-3）。
     * 每次探测都记账，诊断命令能直接说出"哪颗在、读到的 ID 是多少"。
     */
    for (i = 0U; i < candidate_count; i++) {
        const DRV_IMU_Ops *ops = candidates[i].ops;
        uint8_t chip_id = 0U;
        DRV_IMU_Status status;

        if ((ops == NULL) || (ops->probe == NULL)) { continue; }

        status = ops->probe(candidates[i].ctx, &chip_id);
        SVC_IMU_SelectionRecord(&imu_selection, ops->kind, chip_id, status);

        if ((status == DRV_IMU_OK) && (imu_ops == NULL)) {
            imu_ops = ops;
            imu_ctx = candidates[i].ctx;
        }
    }

    if (imu_ops == NULL) {
        return DRV_IMU_BAD_ID;
    }

    {
        DRV_IMU_Status status = imu_ops->init(imu_ctx, &imu_config);
        if (status != DRV_IMU_OK) {
            /* 初始化失败时不要留一个"选中但没配好"的半吊子状态。 */
            imu_ops = NULL;
            imu_ctx = NULL;
            return status;
        }
    }

    imu_initialized = 1U;
    return DRV_IMU_OK;
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
