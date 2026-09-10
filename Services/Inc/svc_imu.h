#ifndef SVC_IMU_H
#define SVC_IMU_H

/*
 * IMU 服务层：芯片轴向 → 机体 FLU 的装配变换，以及选型记账。
 *
 * 为什么必须独立成 Service（decoupling-spec D1-4）：
 *   - 语义升维：驱动给的是"这颗芯片自己的 XYZ"，控制器要的是"机体的前左上"。
 *   - 多源仲裁：板上两颗 IMU，各自的贴装朝向不同，选中哪颗决定用哪个旋转。
 *   - 执行契约：芯片轴 → FLU 的映射是 D4-2 要求"出模块前必须完成"的那一步，
 *     不允许把符号翻转留给下游滤波器去猜。
 *
 * 本模块**不含 HAL、不含 RTOS、不占任务、不做 I/O**，是纯函数 + 纯数据，
 * 因此可以在 PC 上用 host gcc 直接单测（D5-1）。总线访问与探测顺序归 BSP。
 *
 * ============================ 一个必须讲清楚的符号陷阱 ============================
 * 上游（ArduPilot hwdef）给出的贴装朝向常数，例如 BMI088 的
 * ROTATION_ROLL_180_YAW_270，是把**芯片轴转到 ArduPilot 的机体系**，
 * 而 ArduPilot 的机体系是 **FRD**（前-右-下）。本仓库的机体系是 **FLU**（前-左-上）。
 *
 * 所以直接套用上游常数会得到一个 Y、Z 全反的姿态：飞机看起来"能起来"，
 * 但横滚方向是反的。正确做法是两步，且必须分开写好让人能审：
 *     芯片轴 --(上游 ROTATION_* 常数)--> FRD 机体 --(DRV_FRAME_FrdToFlu)--> FLU 机体
 * 第二步复用 Driver/Inc/drv_frame_contract.h 里既有的适配器，不另起一套。
 */

#include "drv_frame_contract.h"
#include "drv_imu_types.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 贴装朝向。枚举值刻意与 ArduPilot 的 ROTATION_* 编号一致，
 * 这样 hwdef 里写的常数可以一眼对上，不必在两套编号之间换算。
 *
 * 只实现"芯片平贴"的 8 种（正装 / 倒装 × 四个 90° 偏航），
 * 侧立安装的十几种本板用不上，需要时再补，不预先堆代码。
 */
typedef enum {
    SVC_IMU_ROTATION_NONE              = 0,
    SVC_IMU_ROTATION_YAW_90            = 2,
    SVC_IMU_ROTATION_YAW_180           = 4,
    SVC_IMU_ROTATION_YAW_270           = 6,
    SVC_IMU_ROTATION_ROLL_180          = 8,
    SVC_IMU_ROTATION_ROLL_180_YAW_90   = 10,
    SVC_IMU_ROTATION_PITCH_180         = 12,  /* = ROLL_180_YAW_180 */
    SVC_IMU_ROTATION_ROLL_180_YAW_270  = 14
} SVC_IMU_Rotation;

/*
 * 把一个三分量向量从芯片轴转到 FLU 机体轴。
 *
 * 极向量（比力）与轴向量（角速度）在这里用同一个映射：本枚举里的每一种都是
 * 真旋转（行列式 +1），不含镜像，所以两类量的变换相同。
 * 输入输出可以是同一个指针。
 */
DRV_FRAME_Vector3f SVC_IMU_RotateToFlu(SVC_IMU_Rotation rotation,
                                       DRV_FRAME_Vector3f chip_axis_value);

/*
 * 整帧变换：把驱动给的芯片轴 ScaledData 转成 FLU 机体轴。
 * 温度原样透传（标量，与朝向无关）。in 与 out 可以是同一个指针。
 */
void SVC_IMU_ApplyMounting(SVC_IMU_Rotation rotation,
                           const DRV_IMU_ScaledData *chip_axis,
                           DRV_IMU_ScaledData *body_flu);

/*
 * 各芯片在 MicoAir743v2 上的默认贴装朝向。
 *
 * 取自 doc/micoair743v2/vendor/ardupilot-hwdef.dat：
 *   IMU BMI088 ... ROTATION_ROLL_180_YAW_270
 *   IMU BMI270 ... ROTATION_ROLL_180
 * ICM-42688 不在这块板上，返回 NONE（老板子的朝向由老 .ioc 与接线决定）。
 *
 * **这只是初值**：上机后必须用 IMUCAP / 自检流程实测复核，
 * 静止水平时 FLU 比力应当约等于 [0, 0, +1] g（drv_frame_contract.h）。
 */
SVC_IMU_Rotation SVC_IMU_DefaultRotation(DRV_IMU_ChipKind kind);

/*
 * 芯片轴 → **legacy_intermediate_v1 中间轴**。
 *
 * 这是给 App/Src/app_sensor.c 的 APP_Sensor_AlignToAirframe() 用的。
 * 那条链路是既有的、且带持久化 ABI 的两段式：
 *     芯片轴 --(本函数，随芯片/板子固定)--> 中间轴 --(持久化 orientation code)--> FLU
 * 第二段的 24 项方向码表存在 Flash 里（IMUFRAME 参数），顺序不能动。
 * 所以换板换芯片时该改的是**第一段**，不是在别处再叠一层旋转——
 * 叠加会导致双重应用，姿态看着"差不多"但横滚符号是反的。
 *
 * 中间轴的物理含义（由老板子的 V0 拟合定义，见 app_sensor.c 顶部注释）：
 *     X 朝后、Y 朝右、Z 朝上
 * 与 FRD（前右下）的关系是 X、Z 取反、Y 不变。于是对新板上的芯片：
 *     中间轴 = ( -FRD.x, +FRD.y, -FRD.z )，其中 FRD 由上游 ROTATION_* 常数给出。
 *
 * ICM-42688 不在新板上，它的映射是老板子实测的固定值，无法由上游常数推导，
 * 因此原样保留（返回值与 app_sensor.c 里原来那三行完全一致）。
 */
void SVC_IMU_ChipToIntermediate(DRV_IMU_ChipKind kind,
                                const float in[3],
                                float out[3]);

/*
 * 上电时该用哪个持久化方向码（app_sensor 的 24 项方向表下标）。
 *
 * 为什么需要这个：方向码平时来自 Flash 里的 IMUFRAME 参数，但**全新的板子参数区
 * 是空的**，此时 app_sensor 会停在 legacy 哨兵上，退回老板子实测的符号补偿——
 * 那套补偿是给 ICM-42688 那个安装方向的，用在 MicoAir 上横滚方向是反的。
 *
 * 返回值的推导：MicoAir 上两颗芯片经 SVC_IMU_ChipToIntermediate 得到中间轴后，
 * 再套方向码 3（"-x,-y,+z"）的结果，恰好等于 SVC_IMU_RotateToFlu 的直接映射
 * （两颗都成立，tests/test_flu_seam0_sensor_frame.py 有断言）。所以 3 是正确默认值。
 * ICM-42688 仍返回哨兵，老板子的行为一个字节都不变。
 */
#define SVC_IMU_ORIENTATION_CODE_LEGACY 255U
#define SVC_IMU_ORIENTATION_CODE_MICOAIR  3U

uint8_t SVC_IMU_DefaultOrientationCode(DRV_IMU_ChipKind kind);

/* 选型记账：探测结果与最终选中的芯片，供诊断命令原样回报。 */
typedef struct {
    DRV_IMU_ChipKind selected;
    SVC_IMU_Rotation rotation;
    uint8_t          selected_chip_id;
    uint8_t          probe_count;
    uint8_t          probed_kind[4];
    uint8_t          probed_chip_id[4];
    DRV_IMU_Status   probed_status[4];
} SVC_IMU_Selection;

void SVC_IMU_SelectionReset(SVC_IMU_Selection *selection);

/*
 * 记一次探测结果。**只记账，不选中。**
 *
 * 这两件事必须分开：probe 成功只说明"总线上有这颗芯片、ID 对得上"，不代表它能用。
 * BMI270 要上传 328 字节配置固件，BMI088 有一串带回读重试的寄存器配置，这些都可能
 * 在 probe 之后失败。早先这里探到就选中，于是主 IMU 配置失败时，备用的那颗
 * 永远得不到机会——采样任务外层重试又选回同一颗，无限循环。
 *
 * 现在的契约是：BSP 先把所有候选探一遍并记账，再按优先级逐个 init，
 * 谁先 init 成功谁才被 SVC_IMU_SelectionCommit 选中。
 * 于是 selection->selected 的含义是"正在跑的那颗"，诊断不会与实际跑的芯片对不上。
 */
void SVC_IMU_SelectionRecord(SVC_IMU_Selection *selection,
                             DRV_IMU_ChipKind kind,
                             uint8_t chip_id,
                             DRV_IMU_Status status);

/* 敲定选型：init 成功之后才调，同时按芯片种类装好默认安装旋转。 */
void SVC_IMU_SelectionCommit(SVC_IMU_Selection *selection,
                             DRV_IMU_ChipKind kind,
                             uint8_t chip_id);

const char *SVC_IMU_ChipName(DRV_IMU_ChipKind kind);

#ifdef __cplusplus
}
#endif

#endif
