#ifndef DRV_AIRFRAME_PARAMS_H
#define DRV_AIRFRAME_PARAMS_H

#include <stdint.h>

/*
 * 机体模型：**唯一来源是 Flash**，没有编译期兜底。
 *
 * 这些数是量出来的，会随装配变化（换电池、加挂载、改倾转机构）。以前它们是
 * `drv_airframe_model.h` 里的一堆 `#define`，每改一次都要重新编译烧录；更糟的是
 * 同一个量在代码里存了两份而且**对不上**——四个部件质量加起来 754.6 g，
 * 而 `DRV_AIRFRAME_MASS_KG` 写的是 1.3670 kg；重心 −0.0946 m 按前者算，
 * 重量 13.41 N 按后者算，两套并存，谁也没报错。
 *
 * 所以现在的规矩是：**代码里不再保留任何一份机体数据的副本。** 参数只从 Flash 读，
 * 由上位机写入。没有第二个来源，就不会有两个来源打架。
 *
 * ──────────────── 空参数区怎么办 ────────────────
 *
 * 一块刚烧完固件、还没写过机体数据的板子，`valid` 为 0，所有字段是零。
 * 这种状态下**禁止解锁**（见 DRV_Airframe_IsValid 的调用方）。
 *
 * 这是刻意的取舍：没有质量、没有惯量、没有力臂，控制律算出来的每一个力矩都没有
 * 物理含义。与其塞一组"出厂默认值"让它看起来能飞——那正是上一版埋下数据矛盾的
 * 方式——不如干脆拒绝起飞，并在诊断里说明缺什么。**失效要朝安全的方向倒。**
 *
 * ──────────────── 输入与派生的分界 ────────────────
 *
 *   **输入**  —— 拿尺和秤能直接量到的：各部件质量与其重心 z、几何距离、惯量。
 *   **派生**  —— 由输入算出来的：总质量、整机重心、重量、悬停油门百分比等。
 *
 * `derived_auto` 决定派生值怎么来：
 *   1（推荐）—— 由输入自动重算，写它们会被拒绝。量准之后应当用这一档。
 *   0        —— 派生值保持被写入的样子。留这个口子是因为有些量（例如整机惯量）
 *                可能来自双线摆实测而不是部件表推算，硬要它服从推算反而是错的。
 *                上位机必须把这一档显式标出来，别让人以为数字是算出来的。
 *
 * 上位机用 `DRV_Airframe_ComputeDerived()` 可以在不改变任何状态的前提下算出"若自动
 * 会是多少"，把现值与自动值并排显示——差多少一眼可见。
 */

typedef struct {
    /* ──────── 输入：部件质量 [g] 与各自重心的 z 坐标 [m]（原点为板中心，z+ 向上） */
    float board_mass_g;
    float battery_mass_g;
    float base_mass_g;
    float servo_motor_mass_g;
    float board_cg_z_m;
    float battery_cg_z_m;
    float base_cg_z_m;
    float servo_motor_cg_z_m;

    /* ──────── 输入：几何 [m] */
    float imu_z_m;
    float prop_plane_d_m;
    float roll_axis_to_prop_plane_m;
    float pitch_axis_to_prop_plane_m;
    float pitch_thrust_lever_arm_m;
    float roll_thrust_lever_arm_m;
    float servo1_axis_z_m;
    float servo2_axis_z_m;
    float thrust_point_z_m;
    float tether_attach_z_m;
    float tether_rope_m;

    /* ──────── 输入：惯量 [kg·m²]、旋向、环境与执行器标度 */
    float ixx_kgm2;
    float iyy_kgm2;
    float izz_kgm2;
    float lower_rotor_spin_sense;   /* +1 = 俯视逆时针，-1 = 顺时针 */
    float gravity_m_s2;
    float max_total_thrust_g;       /* 双桨合计最大推力 */
    float servo_deg_per_us;

    /* ──────── 派生：由上面算出，见 DRV_Airframe_ComputeDerived() */
    float mass_kg;
    float cg_z_m;
    float weight_n;
    float thrust_point_to_cg_z_m;   /* tau = r x F 里的 r，决定倾转力矩的**符号** */
    float tether_attach_to_cg_m;
    float tether_rod_to_cg_m;
    float max_total_force_n;
    float hover_thrust_percent;
    float servo_us_per_deg;

    /* 派生值是否自动重算。1 = 自动（推荐），0 = 保持写入值。 */
    float derived_auto;
} DRV_Airframe_Params;

/*
 * 机体模型是否可用。0 = Flash 里没有有效记录，或记录里有物理上不可能的值
 * （质量为零、惯量为零、重力为零……）。为 0 时必须禁止解锁。
 */
uint8_t DRV_Airframe_IsValid(void);

/* 逐项体检，返回第一个不合格字段的名字；全部合格返回 NULL。供诊断如实回报。 */
const char *DRV_Airframe_FirstInvalidName(void);

/* 清空为"未写入"状态。上电时若 Flash 无记录即为此状态。 */
void DRV_Airframe_Clear(void);

/* 只读视图，供控制律取值——热路径上不拷贝整个结构体。永不返回 NULL。 */
const DRV_Airframe_Params *DRV_Airframe_Get(void);

void DRV_Airframe_GetParams(DRV_Airframe_Params *out);

/*
 * 整体写入（持久化层加载 Flash 记录时调用）。写入后自动体检并更新 valid；
 * derived_auto 非零时顺便重算派生值，保证存进去的和算出来的一致。
 */
void DRV_Airframe_SetParams(const DRV_Airframe_Params *in);

/*
 * 具名读写，名字形如 "airframe.battery_mass_g"。
 * 返回 0 表示名字不认识；写派生值且 derived_auto 为 1 时同样返回 0（拒绝）。
 */
uint8_t DRV_Airframe_GetParam(const char *name, float *value);
uint8_t DRV_Airframe_SetParam(const char *name, float value);

/* 遍历用：上位机据此自动生成表单，不必在两边各维护一份名字清单。 */
uint32_t DRV_Airframe_GetParamCount(void);
const char *DRV_Airframe_GetParamNameAt(uint32_t index);

/* 该名字是不是派生值（上位机据此标注"算出来的"并默认置灰）。 */
uint8_t DRV_Airframe_IsDerivedName(const char *name);

/*
 * 按 `in` 的输入字段算出派生字段写进 `out`，**不触碰任何全局状态**。
 * `out` 的输入字段原样复制自 `in`。可以 in == out。
 * 上位机用它做"若切到自动会是多少"的预览。
 */
void DRV_Airframe_ComputeDerived(const DRV_Airframe_Params *in,
                                 DRV_Airframe_Params *out);

#endif /* DRV_AIRFRAME_PARAMS_H */
