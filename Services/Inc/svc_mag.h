#ifndef SVC_MAG_H
#define SVC_MAG_H

/*
 * 磁力计服务层：芯片轴向 -> 机体 FLU 的装配变换，以及"能不能拿去融合"的轴向前提。
 *
 * 这是 Services/Inc/svc_imu.h 的磁力计对应物，风格、拆分理由与它一致，
 * 先读那份头文件顶部注释。与它不同的是本模块**不做**硬磁/软磁校准——
 * raw -> mgauss 的量程换算在 Driver/Src/drv_mag.c，零偏/软铁矩阵拟合是
 * 另一张工单（Services/svc_mag_cal，尚未创建）。本模块只回答一件事：
 * "芯片说的那个三轴向量，转到机体 FLU 之后应该长什么样"。
 *
 * 为什么必须独立成 Service（decoupling-spec D1-4）：
 *   - 语义升维：驱动给的是"这颗芯片自己的 XYZ"，估计器要的是"机体的前左上"。
 *   - 执行契约：芯片轴 -> FLU 的映射是 D4-2 要求"出模块前必须完成"的那一步，
 *     不允许把符号翻转留给下游融合算法去猜。
 *
 * 本模块**不含 HAL、不含 RTOS、不占任务、不做 I/O**，是纯函数 + 纯数据，
 * 因此可以在 PC 上用 host gcc 直接单测（D5-1）。故意不 #include "drv_mag.h"：
 * 那个头为了声明 I2C 句柄拖进了 "main.h"（HAL），会把本该能在 PC 上编译的
 * 纯逻辑代码也拖下水——drv_imu_types.h vs drv_imu.h 的拆分是同一个理由。
 *
 * ============================ 一个必须讲清楚的符号陷阱（同 svc_imu.h，这里为磁力计再钉一遍）
 * 上游（ArduPilot hwdef）给出的贴装朝向常数是把**芯片轴转到 ArduPilot 的机体系**，
 * 而 ArduPilot 的机体系是 **FRD**（前-右-下）。本仓库的机体系是 **FLU**（前-左-上）。
 * 哪怕常数恰好是 ROTATION_NONE（"不转"），那也只表示"芯片轴恒等于 FRD"，
 * 不表示"恒等于 FLU"——千万不要因为它是恒等旋转就省略第二步。
 *
 * 正确做法永远是两步，且必须分开写好让人能审：
 *     芯片轴 --(上游 ROTATION_* 常数)--> FRD 机体 --(DRV_FRAME_FrdToFlu)--> FLU 机体
 * 第二步复用 Driver/Inc/drv_frame_contract.h 里既有的适配器，不另起一套。
 *
 * ============================ 磁场向量的极向量性质
 * 磁场和比力（加速度计）一样是**极向量**，角速度是**轴向量（伪向量）**。
 * 两者在"真旋转"（行列式 +1，不含镜像）下用同一个变换矩阵；只有在允许镜像的
 * 变换下两者才会分道扬镳。本模块只实现贴装用的 8 种真旋转（详见下方枚举），
 * 所以磁场向量可以复用与 IMU 完全相同的旋转表，不需要为极性/轴性分开写代码。
 * Driver/Inc/drv_frame_contract.h 对此有同一节的规范性说明。
 */

#include "drv_frame_contract.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 贴装朝向。枚举值刻意与 ArduPilot 的 ROTATION_* 编号一致，
 * 这样 hwdef 里写的常数可以一眼对上，不必在两套编号之间换算（理由同 svc_imu.h）。
 *
 * 只实现"芯片平贴"的 8 种（正装 / 倒装 x 四个 90 度偏航），
 * 侧立安装的十几种本板用不上，需要时再补，不预先堆代码。
 */
typedef enum {
    SVC_MAG_ROTATION_NONE              = 0,
    SVC_MAG_ROTATION_YAW_90            = 2,
    SVC_MAG_ROTATION_YAW_180           = 4,
    SVC_MAG_ROTATION_YAW_270           = 6,
    SVC_MAG_ROTATION_ROLL_180          = 8,
    SVC_MAG_ROTATION_ROLL_180_YAW_90   = 10,
    SVC_MAG_ROTATION_PITCH_180         = 12,  /* = ROLL_180_YAW_180 */
    SVC_MAG_ROTATION_ROLL_180_YAW_270  = 14
} SVC_MAG_Rotation;

/*
 * 把一个三分量向量从芯片轴转到 FLU 机体轴。
 *
 * 输入单位与 Driver/Inc/drv_mag.h 的 DRV_MAG_ScaledData 一致（mgauss），
 * 但旋转本身与单位无关，调用方传角度/计数/任意标度的向量都成立。
 * 极向量（磁场）与轴向量（角速度）在这里用同一个映射：本枚举里的每一种都是
 * 真旋转（行列式 +1），不含镜像。输入输出可以是同一个指针。
 */
DRV_FRAME_Vector3f SVC_MAG_RotateToFlu(SVC_MAG_Rotation rotation,
                                       DRV_FRAME_Vector3f chip_axis_value);

/*
 * MicoAir743v2 板载磁力计（QMC5883L, I2C 0x0D）的默认贴装朝向。
 *
 * 取自 doc/micoair743v2/vendor/ardupilot-hwdef.dat:108-109：
 *     COMPASS QMC5883L I2C:ALL_EXTERNAL:0x0D true  ROTATION_NONE
 *     COMPASS QMC5883L I2C:ALL_INTERNAL:0x0D false ROTATION_NONE
 * 两条记录（外置/板载）都是 ROTATION_NONE，所以本板只有一个默认朝向可给，
 * 不需要按"哪颗芯片"再参数化——这也是本函数不带参数、不依赖
 * Driver/Inc/drv_mag.h 里 DRV_MAG_Type 的原因（那个类型和 HAL 绑在一起）。
 *
 * 推导（钉死，两步都要走，见上方符号陷阱说明）：
 *     芯片轴 --(ROTATION_NONE，恒等)--> FRD 机体 --(DRV_FRAME_FrdToFlu)--> FLU 机体
 *     => flu = ( chip.x, -chip.y, -chip.z )
 *
 * **这只是初值**，来自数据手册量程换算 + hwdef 铭牌位推断，从未经过实机验证。
 * 调用方必须先确认 SVC_MAG_AxisVerification 为 SVC_MAG_AXIS_VERIFIED
 * （见下方契约）才能把这个朝向的结果用于姿态融合；"公式推导正确"与
 * "已经实机验证"是两件事，本模块只负责前者，后者的判据放在头文件里钉死，
 * 但由调用方（Services 或 App）落地检查。
 */
SVC_MAG_Rotation SVC_MAG_DefaultRotation(void);

/*
 * 轴向验证状态 —— 契约（硬约束，不许放松，只许收紧）：
 *
 *   1. 默认值必须是 UNVERIFIED，且数值上等于 0：任何刚上电、从未跑过轴向
 *      验证流程、或用 memset(0) / 全零初始化得到的状态，天然落在这一态，
 *      不需要额外的"我是不是初始化过"标志。
 *   2. 只要状态是 UNVERIFIED，不论校准系数（硬磁零偏/软磁矩阵，属于尚未
 *      创建的 Services/svc_mag_cal）是否已写入，上层都**不得**把磁力计
 *      数据喂进姿态融合（FusionAhrsUpdate 的带磁力计版本）。
 *      SVC_MAG_DefaultRotation() 给出的初值在数学推导上是对的，但"推导对"
 *      不等于"贴装对"——芯片有可能贴反、hwdef 抄错、走线时被人为转向。
 *   3. 把 UNVERIFIED 状态改成 VERIFIED 属于运行时操作（例如 MAGFRAME VERIFY
 *      命令族），不属于本模块——本模块是纯函数+纯数据，不持有运行时状态，
 *      也不判断"已收到的一次验证证据是否充分"。这里只钉死取值语义与默认值，
 *      具体判据落地在哪个模块（Services 还是 App）由调用方决定。
 *
 * 与 SVC_IMU 的对照：IMU 没有这一层，因为 IMU 的装配错误由加速度计的
 * 水平自检（静止应约等于 [0,0,+1] g）间接兜底；磁力计没有等价的静态自检
 * （地磁场方向未知、且受机架/电机干扰），所以必须显式引入这个状态位。
 */
typedef enum {
    SVC_MAG_AXIS_UNVERIFIED = 0,
    SVC_MAG_AXIS_VERIFIED   = 1
} SVC_MAG_AxisVerification;

/* 便于调用方写判据；不持有状态，纯粹是契约第 2 条的可复用实现。 */
static inline uint8_t SVC_MAG_AxisUsableForFusion(SVC_MAG_AxisVerification state)
{
    return (state == SVC_MAG_AXIS_VERIFIED) ? 1U : 0U;
}

#ifdef __cplusplus
}
#endif

#endif
