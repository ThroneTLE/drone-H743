#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "drv_imu.h"
#include "svc_imu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef DRV_IMU_Status      BSP_ICM42688_Status;
typedef DRV_IMU_InitStage   BSP_ICM42688_InitStage;
typedef DRV_IMU_AccelRange  BSP_ICM42688_AccelRange;
typedef DRV_IMU_GyroRange   BSP_ICM42688_GyroRange;
typedef DRV_IMU_Odr         BSP_ICM42688_Odr;
typedef DRV_IMU_Bus         BSP_ICM42688_Bus;
typedef DRV_IMU_Config      BSP_ICM42688_Config;
typedef DRV_IMU_RawData     BSP_ICM42688_RawData;
typedef DRV_IMU_ScaledData  BSP_ICM42688_ScaledData;
typedef DRV_IMU_Device      BSP_ICM42688_Device;

#define BSP_ICM42688_CHIP_ID_VALUE   DRV_IMU_CHIP_ID_VALUE
#define BSP_ICM42688_WHO_AM_I_VALUE  DRV_IMU_WHO_AM_I_VALUE
#define BSP_ICM42688_OK              DRV_IMU_OK
#define BSP_ICM42688_ERROR           DRV_IMU_ERROR
#define BSP_ICM42688_TIMEOUT         DRV_IMU_TIMEOUT
#define BSP_ICM42688_BAD_ID          DRV_IMU_BAD_ID
#define BSP_ICM42688_INVALID_ARG     DRV_IMU_INVALID_ARG

#define BSP_ICM42688_ACCEL_RANGE_16G  DRV_IMU_ACCEL_RANGE_16G
#define BSP_ICM42688_ACCEL_RANGE_8G   DRV_IMU_ACCEL_RANGE_8G
#define BSP_ICM42688_ACCEL_RANGE_4G   DRV_IMU_ACCEL_RANGE_4G
#define BSP_ICM42688_ACCEL_RANGE_2G   DRV_IMU_ACCEL_RANGE_2G
#define BSP_ICM42688_GYRO_RANGE_2000DPS   DRV_IMU_GYRO_RANGE_2000DPS
#define BSP_ICM42688_GYRO_RANGE_1000DPS   DRV_IMU_GYRO_RANGE_1000DPS
#define BSP_ICM42688_GYRO_RANGE_500DPS    DRV_IMU_GYRO_RANGE_500DPS
#define BSP_ICM42688_GYRO_RANGE_250DPS    DRV_IMU_GYRO_RANGE_250DPS
#define BSP_ICM42688_GYRO_RANGE_125DPS    DRV_IMU_GYRO_RANGE_125DPS
#define BSP_ICM42688_GYRO_RANGE_62D5DPS   DRV_IMU_GYRO_RANGE_62D5DPS
#define BSP_ICM42688_GYRO_RANGE_31D25DPS  DRV_IMU_GYRO_RANGE_31D25DPS
#define BSP_ICM42688_GYRO_RANGE_15D625DPS DRV_IMU_GYRO_RANGE_15D625DPS
#define BSP_ICM42688_ODR_100HZ            DRV_IMU_ODR_100HZ
#define BSP_ICM42688_ODR_32KHZ          DRV_IMU_ODR_32KHZ
#define BSP_ICM42688_ODR_16KHZ          DRV_IMU_ODR_16KHZ
#define BSP_ICM42688_ODR_8KHZ           DRV_IMU_ODR_8KHZ
#define BSP_ICM42688_ODR_4KHZ           DRV_IMU_ODR_4KHZ
#define BSP_ICM42688_ODR_2KHZ           DRV_IMU_ODR_2KHZ
#define BSP_ICM42688_ODR_1KHZ           DRV_IMU_ODR_1KHZ
#define BSP_ICM42688_ODR_200HZ          DRV_IMU_ODR_200HZ
#define BSP_ICM42688_ODR_50HZ           DRV_IMU_ODR_50HZ
#define BSP_ICM42688_ODR_25HZ           DRV_IMU_ODR_25HZ
#define BSP_ICM42688_ODR_12D5HZ         DRV_IMU_ODR_12D5HZ
#define BSP_ICM42688_ODR_6D25HZ         DRV_IMU_ODR_6D25HZ
#define BSP_ICM42688_ODR_3D125HZ        DRV_IMU_ODR_3D125HZ
#define BSP_ICM42688_ODR_1D5625HZ       DRV_IMU_ODR_1D5625HZ
#define BSP_ICM42688_ODR_500HZ          DRV_IMU_ODR_500HZ

#define BSP_ICM42688_INIT_STAGE_NONE          DRV_IMU_INIT_STAGE_NONE
#define BSP_ICM42688_INIT_STAGE_BANK_SELECT   DRV_IMU_INIT_STAGE_BANK_SELECT
#define BSP_ICM42688_INIT_STAGE_RESET         DRV_IMU_INIT_STAGE_RESET
#define BSP_ICM42688_INIT_STAGE_WHO_AM_I      DRV_IMU_INIT_STAGE_WHO_AM_I
#define BSP_ICM42688_INIT_STAGE_GYRO_CONFIG   DRV_IMU_INIT_STAGE_GYRO_CONFIG
#define BSP_ICM42688_INIT_STAGE_ACCEL_CONFIG  DRV_IMU_INIT_STAGE_ACCEL_CONFIG
#define BSP_ICM42688_INIT_STAGE_FILTER_CONFIG DRV_IMU_INIT_STAGE_FILTER_CONFIG
#define BSP_ICM42688_INIT_STAGE_PWR_MGMT      DRV_IMU_INIT_STAGE_PWR_MGMT
#define BSP_ICM42688_INIT_STAGE_SIGNAL_RESET  DRV_IMU_INIT_STAGE_SIGNAL_RESET
#define BSP_ICM42688_INIT_STAGE_READY         DRV_IMU_INIT_STAGE_READY

typedef struct {
    uint8_t mode0_tokmas;
    uint8_t mode0_msb;
    uint8_t mode0_bit0;
    uint8_t mode3_tokmas;
    uint8_t mode3_msb;
    uint8_t mode3_bit0;
    uint8_t burst_m0_b0_1;
    uint8_t burst_m0_b0_2;
    uint8_t burst_m0_b0_3;
    uint8_t burst_m0_b0_4;
    uint8_t burst_m3_tok_1;
    uint8_t burst_m3_tok_2;
    uint8_t burst_m3_tok_3;
    uint8_t burst_m3_tok_4;
    uint8_t best_mode;
    uint8_t best_header;
    uint8_t valid;
} BSP_IMU_Diag;

/*
 * 芯片无关的 IMU 概况。
 *
 * 板上可能是 BMI088 / BMI270 / ICM-42688 中的任意一颗，三者的设备结构体各不相同，
 * 所以上层不能再直接读 DRV_IMU_Device。需要"当前量程 / 带宽 / 初始化到哪一步"
 * 的地方一律走这个结构体。
 */
typedef struct {
    DRV_IMU_ChipKind   kind;
    uint8_t            chip_id;
    uint8_t            initialized;
    DRV_IMU_InitStage  init_stage;
    DRV_IMU_Status     last_error;
    DRV_IMU_AccelRange accel_range;
    DRV_IMU_GyroRange  gyro_range;
    /* 实际生效的抗混叠 / 数字带宽，各芯片按支持值向下取整后回报。 */
    uint16_t           accel_bandwidth_hz;
    uint16_t           gyro_bandwidth_hz;
} BSP_IMU_Info;

DRV_IMU_Status BSP_IMU_Init(void);
DRV_IMU_Status BSP_IMU_ReadRaw(DRV_IMU_RawData *raw);
DRV_IMU_Status BSP_IMU_ReadScaled(DRV_IMU_ScaledData *scaled);
DRV_IMU_Status BSP_IMU_IsDataReady(bool *ready);
uint8_t BSP_IMU_GetWhoAmI(void);
void BSP_IMU_GetDiag(BSP_IMU_Diag *diag);
/* 背靠背读取自检的结果，见 BSP_IMU_DebugRawBmi088()。仅诊断命令使用。 */
typedef struct {
    uint32_t sr_before;
    uint32_t sr_after;
    uint32_t sr[4];
    uint8_t  hal[4];
    uint8_t  chip_id[4];
} BSP_IMU_RawProbe;

/*
 * 诊断用的任意 SPI 事务出口。
 *
 * 片选用枚举而不是让调用方给"总线号 + 引脚"：板上只有这三个 IMU 片选，
 * 总线由片选唯一确定（BMI088 在 SPI2、BMI270 在 SPI3）。这样拼不出不存在的
 * 组合，也不会有人拿它去拨别的 GPIO。
 */
typedef enum {
    BSP_IMU_SPI_CS_BMI088_ACC = 0,
    BSP_IMU_SPI_CS_BMI088_GYRO,
    BSP_IMU_SPI_CS_BMI270,
    BSP_IMU_SPI_CS_COUNT
} BSP_IMU_SpiCs;

/* 片选所在的 SPI 实例号（2 或 3），供诊断回报与调用方校验。 */
uint8_t BSP_IMU_DebugSpiBusIndex(BSP_IMU_SpiCs cs);

/* 一次全双工事务：拉低片选 → 收发 len 字节 → 抬片选。len 上限 BSP_IMU_SPI_XFER_MAX。 */
#define BSP_IMU_SPI_XFER_MAX 32U
DRV_IMU_Status BSP_IMU_DebugSpiXfer(BSP_IMU_SpiCs cs, const uint8_t *tx,
                                    uint8_t *rx, uint16_t len);

void BSP_IMU_DebugRawBmi088(BSP_IMU_RawProbe *out);

void BSP_IMU_Invalidate(void);

/*
 * 被选中那颗 IMU 的 DRDY 引脚掩码（未选中任何一颗时返回 0）。
 *
 * 采样节拍必须只认这一个引脚。板上两颗 IMU 各有各的 DRDY，而**软复位不给传感器
 * 掉电**——`BOOT DFU CONFIRM`、看门狗复位之后，上一轮配置过的那颗仍在按自己的
 * ODR 发边沿。2026-09-11 实测：BMI088 当选、陀螺 1000 Hz，实际节拍却是 1760 Hz，
 * 多出来的约 800 Hz 来自上一轮留下来的 BMI270（读回 PWR_CTRL=0x0E，加计陀螺都还开着）；
 * 把 BMI270 软复位之后速率立刻回到 1000 Hz。
 *
 * 后果不是"多几次空唤醒"那么轻：控制环被以约 1.7 倍于陀螺更新率的节奏唤醒，
 * 三成迭代读到的是重复样本。角速率是最内环，重复样本对 D 项就是噪声放大。
 */
uint16_t BSP_IMU_GetDrdyPin(void);

/*
 * 注册"选中那颗 IMU 的 DRDY 到了"的回调。
 *
 * 上层由此不必知道 DRDY 落在哪个引脚、走的是哪条 EXTI，也不必去实现 HAL 的弱
 * 回调 `HAL_GPIO_EXTI_Callback()`——那两件事都是板级知识，这次移植正好都变了
 * （老板子 PC0/EXTI0，本板 BMI088 在 PC15、BMI270 在 PB7）。
 *
 * **在中断上下文调用。** 回调里只许搬数据 / 置标志 / 唤醒任务（D2-1）。
 * 引脚比对由 BSP 做：未选中那颗的边沿不会送到这里——软复位后上一轮配置过的
 * 另一颗仍在按自己的 ODR 发边沿，实测能把节拍从 1000 Hz 顶到 1760 Hz。
 */
void BSP_IMU_SetDrdyHandler(void (*handler)(void));

/* ------------------------------------------------ IMU 总线现场快照（诊断用） */

/*
 * 引脚现场：MODER / AFR / ODR / IDR 四个字段就能把"配置对不对"和"线上有没有电平"
 * 分开。BMI088 探测读回 0x00 时，单看驱动分不清是 MISO 没配成 AF、还是芯片没应答；
 * 有了这张快照就不用猜。
 *
 * 快照由 BSP 生成而不是让上层自己读寄存器：引脚表是板级知识，这次移植里它整张
 * 都变了。表留在上层的话，换板后这条诊断会一本正经地描述**另一块板的硅片**。
 */
#define BSP_IMU_BUS_PIN_MAX 8U

typedef struct {
    const char *name;   /* 例如 "PD3_SCK"，含引脚号与用途，直接可读 */
    uint8_t     mode;   /* MODER：0=输入 1=输出 2=复用 3=模拟 */
    uint8_t     af;     /* AFR */
    uint8_t     od;     /* ODR */
    uint8_t     in;     /* IDR */
    uint8_t     expect_af;  /* 期望的复用号；0xFF = 这根脚本不该归本总线 */
} BSP_IMU_PinSnapshot;

typedef struct {
    const char *spi_name;      /* 选中 IMU 所在的 SPI，例如 "spi2" */
    uint32_t    pmcr;          /* SYSCFG->PMCR，PC2_C 那类模拟开关看这里 */
    uint32_t    spi_cr1;
    uint32_t    spi_cfg1;
    uint32_t    spi_cfg2;
    uint32_t    spi_sr;
    uint32_t    pin_count;
    BSP_IMU_PinSnapshot pins[BSP_IMU_BUS_PIN_MAX];
} BSP_IMU_BusSnapshot;

void BSP_IMU_GetBusSnapshot(BSP_IMU_BusSnapshot *out);


void             BSP_IMU_GetInfo(BSP_IMU_Info *info);
DRV_IMU_ChipKind BSP_IMU_GetChipKind(void);

/* 探测记账：每颗候选芯片读到的 ID 与结果，供诊断命令原样回报。 */
void BSP_IMU_GetSelection(SVC_IMU_Selection *selection);

/*
 * 原始计数 → 物理量，按当前选中的芯片换算。
 * 各芯片的量程刻度不同（BMI088 是 ±3/6/12/24 g，ICM 是 ±2/4/8/16 g），
 * 所以不能在上层用一份固定的 LSB 表。
 */
void BSP_IMU_RawToScaled(const DRV_IMU_RawData *raw, DRV_IMU_ScaledData *scaled);

/*
 * 仅当选中的是 ICM-42688 时返回其设备结构体，否则返回 NULL。
 * 保留它只为兼容既有调用点；新代码请用 BSP_IMU_GetInfo()。
 */
const DRV_IMU_Device *BSP_IMU_GetDevice(void);

#ifdef __cplusplus
}
#endif

#endif
