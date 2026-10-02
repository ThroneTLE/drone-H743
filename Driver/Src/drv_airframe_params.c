#include "drv_airframe_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * 全零 = "还没写过"。这里刻意不放任何默认值：机体数据的唯一来源是 Flash，
 * 代码里再存一份就又有了两个会打架的来源（上一版正是这么坏的）。
 */
static DRV_Airframe_Params airframe_params;
static uint8_t             airframe_valid;

#define AIRFRAME_ENTRY(field) \
    { "airframe." #field, (uint16_t)offsetof(DRV_Airframe_Params, field), 0U }
#define AIRFRAME_DERIVED(field) \
    { "airframe." #field, (uint16_t)offsetof(DRV_Airframe_Params, field), 1U }

typedef struct {
    const char *name;
    uint16_t    offset;
    uint8_t     derived;
} DRV_Airframe_Entry;

static const DRV_Airframe_Entry airframe_table[] = {
    /* 输入：部件质量与重心 */
    AIRFRAME_ENTRY(board_mass_g),
    AIRFRAME_ENTRY(battery_mass_g),
    AIRFRAME_ENTRY(base_mass_g),
    AIRFRAME_ENTRY(servo_motor_mass_g),
    AIRFRAME_ENTRY(board_cg_z_m),
    AIRFRAME_ENTRY(battery_cg_z_m),
    AIRFRAME_ENTRY(base_cg_z_m),
    AIRFRAME_ENTRY(servo_motor_cg_z_m),
    /* 输入：几何 */
    AIRFRAME_ENTRY(imu_z_m),
    AIRFRAME_ENTRY(prop_plane_d_m),
    AIRFRAME_ENTRY(roll_axis_to_prop_plane_m),
    AIRFRAME_ENTRY(pitch_axis_to_prop_plane_m),
    /*
     * 俯仰/横滚"推力力臂"**不在这张表里**（2026-09-27 退役）：力臂现在是
     * servoN_axis_z_m − cg_z_m 两个实测高度之差，不是一个能单独 `PARAM SET`
     * 的数。留着它就又多了一个会和几何打架的来源——上一版正是这样把倾转权限
     * 高估了 2.3 倍而没人发现。
     */
    AIRFRAME_ENTRY(servo1_axis_z_m),
    AIRFRAME_ENTRY(servo2_axis_z_m),
    AIRFRAME_ENTRY(thrust_point_z_m),
    AIRFRAME_ENTRY(tether_attach_z_m),
    AIRFRAME_ENTRY(tether_rope_m),
    /*
     * 输入：惯量、环境、执行器标度。
     *
     * 旋向**不在这张表里**（2026-09-13 退役）：它是量出来的接线事实，归
     * `drv_prop_map`，由上位机通电标定。留在这里的话，一个可以随手 `PARAM SET`
     * 的浮点数就能翻转整机偏航方向，而现象（自稳仍然成立）与增益调错一模一样。
     */
    AIRFRAME_ENTRY(ixx_kgm2),
    AIRFRAME_ENTRY(iyy_kgm2),
    AIRFRAME_ENTRY(izz_kgm2),
    AIRFRAME_ENTRY(gravity_m_s2),
    AIRFRAME_ENTRY(max_total_thrust_g),
    AIRFRAME_ENTRY(servo_deg_per_us),
    /* 派生 */
    AIRFRAME_DERIVED(mass_kg),
    AIRFRAME_DERIVED(cg_z_m),
    AIRFRAME_DERIVED(weight_n),
    AIRFRAME_DERIVED(thrust_point_to_cg_z_m),
    AIRFRAME_DERIVED(tether_attach_to_cg_m),
    AIRFRAME_DERIVED(tether_rod_to_cg_m),
    AIRFRAME_DERIVED(max_total_force_n),
    AIRFRAME_DERIVED(hover_thrust_percent),
    AIRFRAME_DERIVED(servo_us_per_deg),
    /* 开关本身也可读写，上位机据此显示"自动/手动" */
    AIRFRAME_ENTRY(derived_auto),
    /*
     * 光流安装方向（R-FLOWMOUNT-1）。排在表尾：PARAM? 的既有顺序一行不动，
     * 上位机按行对齐的显示不会漂。取值集合见 airframe_flow_mount_value_ok()。
     */
    AIRFRAME_ENTRY(flow_mount_yaw_deg),
    AIRFRAME_ENTRY(flow_mount_mirror),
};

static const uint32_t airframe_table_count =
    (uint32_t)(sizeof(airframe_table) / sizeof(airframe_table[0]));

/* ─────────────────────────────────────────────────────── 体检 */

/*
 * 哪些字段必须非零才算"可飞"。
 *
 * 只列**零值会让控制律失去物理意义**的那些：质量出现在每一条力/加速度换算里，
 * 三个惯量是角加速度换算的分母，重力为零则悬停前馈整体塌掉。几何记账类
 * （系留、桨盘间距）不参与控制，缺了不该拦住起飞——把它们也列进来只会逼人
 * 填假数据。
 *
 * 两个舵机转轴高度（2026-09-27 起）是最要紧的两条：倾转力矩的力臂就是
 * r_z = 转轴 z − 重心 z，**力矩的大小和方向都由它定**。0 在这里不是"转轴就在
 * 板子平面上"，是"没填"——而没填的样子恰恰最危险：转轴留 0、重心算出来是负的，
 * r_z 就变成 +0.0946 m，符号反了，横滚/俯仰两轴同时从负反馈变成正反馈。那不是
 * "少填一项"，是起飞即翻。所以宁可拒绝解锁，也不能让它带着默认零值放行。
 * 填了但方向/大小不对的情形由下面 airframe_check_tilt_axis() 接着挡。
 *
 * `thrust_point_to_cg_z_m` 仍在此列：它不再决定力矩，但它是转轴方向的独立
 * 交叉核对（见 airframe_check_tilt_axis），为零时那道核对没有意义。
 */
static const char *airframe_check_nonzero[] = {
    "airframe.mass_kg",
    "airframe.ixx_kgm2",
    "airframe.iyy_kgm2",
    "airframe.izz_kgm2",
    "airframe.gravity_m_s2",
    "airframe.max_total_force_n",
    "airframe.servo1_axis_z_m",
    "airframe.servo2_axis_z_m",
    "airframe.thrust_point_to_cg_z_m",
};

/*
 * 旋向曾经也在上面这张必填表里，现在不在了——但那道闸门**没有被拆掉，是搬家了**：
 * 它变成 `DRV_PropMap_IsCalibrated()`，和这里一样排在解锁链上
 * （`App/Src/app_stabilizer.c`）。两处都拦住的仍然是同一件事：没有旋向，
 * 偏航力矩的方向没有定义。
 */

static const DRV_Airframe_Entry *airframe_find(const char *name)
{
    uint32_t i;

    if (name == NULL) { return NULL; }

    for (i = 0U; i < airframe_table_count; i++) {
        if (strcmp(airframe_table[i].name, name) == 0) {
            return &airframe_table[i];
        }
    }
    return NULL;
}

static float airframe_field(const DRV_Airframe_Params *p, uint16_t offset)
{
    return *(const float *)(const void *)((const uint8_t *)p + offset);
}

static void airframe_set_field(DRV_Airframe_Params *p, uint16_t offset, float v)
{
    *(float *)(void *)((uint8_t *)p + offset) = v;
}

/*
 * 倾转转轴的两条复合判据。"没填（为零）"已经在上面那张表里挡过，这里挡的是
 * "填了但不对"。
 *
 * 1. |r_z| >= DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M。转轴几乎就在重心上时倾转产生
 *    不了力矩，力矩反解会一路顶到倾角限位——看起来在控制，其实没有权限。
 *
 * 2. 转轴与推力点必须在重心同侧（r_z 与 thrust_point_to_cg_z_m 同号）。两个都是
 *    量出来的，这架飞机的桨挂在转轴下方的电机轴上，二者必然同侧；不同侧几乎
 *    只可能是某一个的符号填反——例如"板子下方 13 cm"填成了 +0.13。而力矩极性
 *    一翻就是起飞即翻，所以宁可拒绝解锁。
 *
 *    ⚠ 这条是按"转轴和桨都在重心下方"的构型定的。真有一种合理设计是转轴在
 *    重心上方、桨在重心下方（或反过来）时，这道闸门会误拦，那时要重新审视它
 *    ——换一个独立的方向核对，而不是直接删掉：删掉之后，符号填反就又没人拦了。
 */
static const char *airframe_check_tilt_axis(float axis_to_cg_z_m,
                                            const char *near_name,
                                            const char *sign_name)
{
    /*
     * 写成 !(|r| >= 下限)，NaN 同样判不合格（与下面非零判据同一个道理）。
     * ±Inf 却会满足 |r| >= 下限、再按符号过同侧核对——部件表溢出或手动档写进
     * 一个 Inf 重心就是这样被放行的，所以先单独挡掉非有限值，归到 :near
     * （"没有可用的力臂"），不另起原因名。
     */
    if (!isfinite(axis_to_cg_z_m) ||
        !((axis_to_cg_z_m >= DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M) ||
          (axis_to_cg_z_m <= -DRV_AIRFRAME_TILT_AXIS_MIN_LEVER_M))) {
        return near_name;
    }
    if ((axis_to_cg_z_m > 0.0f) !=
        (airframe_params.thrust_point_to_cg_z_m > 0.0f)) {
        return sign_name;
    }
    return NULL;
}

const char *DRV_Airframe_FirstInvalidName(void)
{
    const char *bad;
    uint32_t i;

    for (i = 0U; i < (sizeof(airframe_check_nonzero) /
                      sizeof(airframe_check_nonzero[0])); i++) {
        const DRV_Airframe_Entry *entry = airframe_find(airframe_check_nonzero[i]);
        float value;

        if (entry == NULL) { continue; }
        value = airframe_field(&airframe_params, entry->offset);

        /*
         * 同时挡住 NaN：`!(v > 0 || v < 0)` 对 NaN 为真，对 ±0 也为真，正好是
         * "既不是正数也不是负数"这层意思。写成 `v == 0.0f` 会把 NaN 放过去，
         * 而 NaN 一旦进了控制律就会沿着每一条乘法扩散，比零值更难追。
         * ±Inf 是"非零"，但同样不是量出来的数（部件表溢出、PARAM SET 写进
         * 1e39），要用 isfinite 另外挡。
         */
        if (!isfinite(value) || !((value > 0.0f) || (value < 0.0f))) {
            return airframe_table[entry - airframe_table].name;
        }
    }

    bad = airframe_check_tilt_axis(
        DRV_Airframe_RollTiltAxisToCgZ(&airframe_params),
        "airframe.servo1_axis_z_m:near", "airframe.servo1_axis_z_m:sign");
    if (bad != NULL) {
        return bad;
    }
    return airframe_check_tilt_axis(
        DRV_Airframe_PitchTiltAxisToCgZ(&airframe_params),
        "airframe.servo2_axis_z_m:near", "airframe.servo2_axis_z_m:sign");
}

uint8_t DRV_Airframe_IsValid(void)
{
    return airframe_valid;
}

static void airframe_revalidate(void)
{
    airframe_valid = (DRV_Airframe_FirstInvalidName() == NULL) ? 1U : 0U;
}

/* ─────────────────────────────────────────────────────── 光流安装方向 */

/*
 * 两项都是离散量，存成 float 只是为了和整张表同一种访问方式。允许集合写死在这里：
 * 转角只能是 90° 的整数倍（光流模块只可能正装、侧装或反装，没有"斜 30°"这回事，
 * 也没有需要连续标定的量），镜像只能开/关。别的值在写入口当场拒绝，而不是
 * 就近取整——"写了 45 却按 0 生效"正是 D5-3 说的撒谎。
 */
#define AIRFRAME_FLOW_MOUNT_YAW_OFFSET \
    ((uint16_t)offsetof(DRV_Airframe_Params, flow_mount_yaw_deg))
#define AIRFRAME_FLOW_MOUNT_MIRROR_OFFSET \
    ((uint16_t)offsetof(DRV_Airframe_Params, flow_mount_mirror))

static uint8_t airframe_flow_mount_yaw_ok(float value)
{
    return ((value == 0.0f) || (value == 90.0f) ||
            (value == 180.0f) || (value == 270.0f)) ? 1U : 0U;
}

static uint8_t airframe_flow_mount_mirror_ok(float value)
{
    return ((value == 0.0f) || (value == 1.0f)) ? 1U : 0U;
}

/*
 * 写入口的取值检查。其余字段一律放行（机体数据是量出来的事实，不设限，见
 * parameter_model.py 的说明）；只有这两项有离散集合。
 * 返回 1 时 *value 已规范化（-0 写成 +0，读回来不会是 "-0.000000"）。
 */
static uint8_t airframe_flow_mount_value_ok(uint16_t offset, float *value)
{
    if (offset == AIRFRAME_FLOW_MOUNT_YAW_OFFSET) {
        if (airframe_flow_mount_yaw_ok(*value) == 0U) { return 0U; }
    } else if (offset == AIRFRAME_FLOW_MOUNT_MIRROR_OFFSET) {
        if (airframe_flow_mount_mirror_ok(*value) == 0U) { return 0U; }
    } else {
        return 1U;
    }
    if (*value == 0.0f) {
        *value = 0.0f;   /* -0.0f 与 0.0f 相等，写成字面量 +0 */
    }
    return 1U;
}

/*
 * 整体写入（Flash 加载）时的兜底。经 SetParam 写进 Flash 的值不会越界，校验和也
 * 挡住了位翻转；能走到这里的只有绕过写入口的调用方。落回 0 = 恒等变换 = 加入这两项
 * 之前的行为，并且 PARAM? / FLOW? 报的就是实际生效的值。
 */
static void airframe_sanitize_flow_mount(DRV_Airframe_Params *p)
{
    if (airframe_flow_mount_yaw_ok(p->flow_mount_yaw_deg) == 0U) {
        p->flow_mount_yaw_deg = 0.0f;
    }
    if (airframe_flow_mount_mirror_ok(p->flow_mount_mirror) == 0U) {
        p->flow_mount_mirror = 0.0f;
    }
    if (p->flow_mount_yaw_deg == 0.0f) { p->flow_mount_yaw_deg = 0.0f; }
    if (p->flow_mount_mirror == 0.0f) { p->flow_mount_mirror = 0.0f; }
}

uint16_t DRV_Airframe_FlowMountYawDeg(const DRV_Airframe_Params *p)
{
    if ((p == NULL) || (airframe_flow_mount_yaw_ok(p->flow_mount_yaw_deg) == 0U)) {
        return 0U;
    }
    return (uint16_t)p->flow_mount_yaw_deg;
}

uint8_t DRV_Airframe_FlowMountMirror(const DRV_Airframe_Params *p)
{
    return ((p != NULL) && (p->flow_mount_mirror == 1.0f)) ? 1U : 0U;
}

/* derived_auto 是 float 字段（Flash ABI），按 |v| > 0.5 判"自动"。 */
static uint8_t airframe_derived_auto_on(const DRV_Airframe_Params *p)
{
    return ((p->derived_auto > 0.5f) || (p->derived_auto < -0.5f)) ? 1U : 0U;
}

/* ─────────────────────────────────────────────────────── 派生值 */

void DRV_Airframe_ComputeDerived(const DRV_Airframe_Params *in,
                                 DRV_Airframe_Params *out)
{
    float total_g;

    if ((in == NULL) || (out == NULL)) { return; }

    if (out != in) { *out = *in; }

    total_g = in->board_mass_g + in->battery_mass_g +
              in->base_mass_g + in->servo_motor_mass_g;

    out->mass_kg = total_g / 1000.0f;

    /*
     * 整机重心 = Σ(m_i·z_i)/Σm_i。部件表为空时不去算——0/0 会得到 NaN，
     * 而 NaN 比 0 更糟：它会沿着后面每一条乘法扩散，最后在某个毫不相干的地方
     * 表现成"姿态突然发散"，根本追不回这里。
     */
    if (total_g > 0.0f) {
        out->cg_z_m = ((in->board_mass_g       * in->board_cg_z_m) +
                       (in->battery_mass_g     * in->battery_cg_z_m) +
                       (in->base_mass_g        * in->base_cg_z_m) +
                       (in->servo_motor_mass_g * in->servo_motor_cg_z_m)) / total_g;
    } else {
        out->cg_z_m = 0.0f;
    }

    out->weight_n = out->mass_kg * in->gravity_m_s2;

    /*
     * 推力点（双桨中点）相对重心的 z。2026-09-11 ~ 2026-09-27 倾转力矩的符号
     * 取自它；现在力矩看的是推力作用线穿过的舵机转轴
     * （DRV_Airframe_Roll/PitchTiltAxisToCgZ），它退居为解锁闸门里的独立交叉
     * 核对：转轴与推力点必须在重心同侧。
     *
     * 仍然必须是两个实测量相减，而不是一个可以单独写下来的符号常量：符号常量
     * 能被"照着现象翻一下试试"改掉，那道交叉核对也就失去了意义。
     *
     * 推力挂在板子下方，所以这架飞机上它是负的。
     */
    out->thrust_point_to_cg_z_m = in->thrust_point_z_m - out->cg_z_m;

    out->tether_attach_to_cg_m = in->tether_attach_z_m - out->cg_z_m;
    out->tether_rod_to_cg_m    = in->tether_rope_m + out->tether_attach_to_cg_m;

    out->max_total_force_n = (in->max_total_thrust_g / 1000.0f) * in->gravity_m_s2;

    out->hover_thrust_percent = (out->max_total_force_n > 0.0f)
        ? ((out->weight_n / out->max_total_force_n) * 100.0f)
        : 0.0f;

    out->servo_us_per_deg = ((in->servo_deg_per_us > 0.0f) ||
                             (in->servo_deg_per_us < 0.0f))
        ? (1.0f / in->servo_deg_per_us)
        : 0.0f;
}

/*
 * τ = r × F 里的 r_z：倾转转轴相对整机重心。**倾转力矩的大小和方向只由它定。**
 *
 * 与 thrust_point_to_cg_z_m 一样必须是两个实测量相减，而不是一个能单独写下来的
 * 数：单独的"力臂"会像退役的 retired_*_thrust_lever_arm_m 那样，机体换了、
 * 重心动了，它还停在旧值上，谁也不报错。写成减法之后，想改它只能重新量飞机。
 *
 * 不存成派生字段是因为 DRV_Airframe_Params 是 Flash ABI，加字段就要升 CFG 版本并
 * 冻结旧布局（v26 的光流安装两项就是这么加的）；每次现算只是一次减法，不值得。
 * 手动派生档下 cg_z_m 是写入值，这里照用——与控制律用的是同一个数。
 */
float DRV_Airframe_RollTiltAxisToCgZ(const DRV_Airframe_Params *p)
{
    return (p != NULL) ? (p->servo1_axis_z_m - p->cg_z_m) : 0.0f;
}

float DRV_Airframe_PitchTiltAxisToCgZ(const DRV_Airframe_Params *p)
{
    return (p != NULL) ? (p->servo2_axis_z_m - p->cg_z_m) : 0.0f;
}

/* ─────────────────────────────────────────────────────── 读写 */

void DRV_Airframe_Clear(void)
{
    memset(&airframe_params, 0, sizeof(airframe_params));
    airframe_valid = 0U;
}

const DRV_Airframe_Params *DRV_Airframe_Get(void)
{
    return &airframe_params;
}

void DRV_Airframe_GetParams(DRV_Airframe_Params *out)
{
    if (out != NULL) { *out = airframe_params; }
}

void DRV_Airframe_SetParams(const DRV_Airframe_Params *in)
{
    if (in == NULL) { return; }

    airframe_params = *in;
    airframe_sanitize_flow_mount(&airframe_params);
    if (airframe_derived_auto_on(&airframe_params) != 0U) {
        DRV_Airframe_ComputeDerived(&airframe_params, &airframe_params);
    }
    airframe_revalidate();
}

uint8_t DRV_Airframe_GetParam(const char *name, float *value)
{
    const DRV_Airframe_Entry *entry = airframe_find(name);

    if ((entry == NULL) || (value == NULL)) { return 0U; }
    *value = airframe_field(&airframe_params, entry->offset);
    return 1U;
}

uint8_t DRV_Airframe_SetParam(const char *name, float value)
{
    const DRV_Airframe_Entry *entry = airframe_find(name);

    if (entry == NULL) { return 0U; }

    /*
     * 自动档下拒绝直接写派生值，而不是"写了再被下一次重算覆盖"。
     * 后者上位机会以为写成功了，下次读回来又变了，找不到原因——
     * 诊断宁可当场说不行（D5-3：宁可沉默，不可撒谎）。
     */
    if ((entry->derived != 0U) &&
        (airframe_derived_auto_on(&airframe_params) != 0U)) {
        return 0U;
    }

    /* 光流安装两项只收离散集合；不合格当场拒绝，原值不动。 */
    if (airframe_flow_mount_value_ok(entry->offset, &value) == 0U) {
        return 0U;
    }

    airframe_set_field(&airframe_params, entry->offset, value);

    /*
     * 改了输入就立刻把派生值跟上，读回来的永远是自洽的一组。
     *
     * 要不要重算按**写入之后**的 derived_auto 判，而不是写之前的：
     * `PARAM SET airframe.derived_auto 1` 本身就是一次输入写入，写之前还是
     * 手动档，按旧值判就不会重算——重心（以及由它现算的倾转力臂）停在手写值上，
     * 界面却已显示"自动"；直到下次重启 SetParams 才悄悄换成算出来的值。
     */
    if (airframe_derived_auto_on(&airframe_params) != 0U) {
        DRV_Airframe_ComputeDerived(&airframe_params, &airframe_params);
    }

    airframe_revalidate();
    return 1U;
}

uint32_t DRV_Airframe_GetParamCount(void)
{
    return airframe_table_count;
}

const char *DRV_Airframe_GetParamNameAt(uint32_t index)
{
    return (index < airframe_table_count) ? airframe_table[index].name : NULL;
}

uint8_t DRV_Airframe_IsDerivedName(const char *name)
{
    const DRV_Airframe_Entry *entry = airframe_find(name);

    return (entry != NULL) ? entry->derived : 0U;
}
