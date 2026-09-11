#include "drv_airframe_params.h"

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
    AIRFRAME_ENTRY(pitch_thrust_lever_arm_m),
    AIRFRAME_ENTRY(roll_thrust_lever_arm_m),
    AIRFRAME_ENTRY(servo1_axis_z_m),
    AIRFRAME_ENTRY(servo2_axis_z_m),
    AIRFRAME_ENTRY(thrust_point_z_m),
    AIRFRAME_ENTRY(tether_attach_z_m),
    AIRFRAME_ENTRY(tether_rope_m),
    /* 输入：惯量、旋向、环境、执行器标度 */
    AIRFRAME_ENTRY(ixx_kgm2),
    AIRFRAME_ENTRY(iyy_kgm2),
    AIRFRAME_ENTRY(izz_kgm2),
    AIRFRAME_ENTRY(lower_rotor_spin_sense),
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
};

static const uint32_t airframe_table_count =
    (uint32_t)(sizeof(airframe_table) / sizeof(airframe_table[0]));

/* ─────────────────────────────────────────────────────── 体检 */

/*
 * 哪些字段必须非零才算"可飞"。
 *
 * 只列**零值会让控制律失去物理意义**的那些：质量出现在每一条力/加速度换算里，
 * 三个惯量是角加速度换算的分母，力臂为零意味着倾转产生不出力矩，
 * 重力为零则悬停前馈整体塌掉。几何记账类（系留、舵机轴位置）不参与控制，
 * 缺了不该拦住起飞——把它们也列进来只会逼人填假数据。
 *
 * `thrust_point_to_cg_z_m` 也在此列，而且是最要紧的一条：它是 τ = r × F 里的
 * r_z，**倾转力矩的符号完全由它的正负决定**。为零时符号没有定义。更危险的是
 * 漏填的样子——thrust_point_z_m 留空、重心算出来是负的，r_z 就变成正值，
 * 俯仰和横滚极性同时翻转，整条姿态环从负反馈变成正反馈。那不是"少填一项"，
 * 是起飞即翻。所以宁可拒绝解锁，也不能让它带着默认零值放行。
 */
static const char *airframe_check_nonzero[] = {
    "airframe.mass_kg",
    "airframe.ixx_kgm2",
    "airframe.iyy_kgm2",
    "airframe.izz_kgm2",
    "airframe.gravity_m_s2",
    "airframe.max_total_force_n",
    "airframe.pitch_thrust_lever_arm_m",
    "airframe.roll_thrust_lever_arm_m",
    "airframe.lower_rotor_spin_sense",
    "airframe.thrust_point_to_cg_z_m",
};

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

const char *DRV_Airframe_FirstInvalidName(void)
{
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
         */
        if (!((value > 0.0f) || (value < 0.0f))) {
            return airframe_table[entry - airframe_table].name;
        }
    }
    return NULL;
}

uint8_t DRV_Airframe_IsValid(void)
{
    return airframe_valid;
}

static void airframe_revalidate(void)
{
    airframe_valid = (DRV_Airframe_FirstInvalidName() == NULL) ? 1U : 0U;
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
     * tau = r x F 里的 r_z，**倾转力矩的符号只由它定**。
     *
     * 它必须是两个实测量相减，而不是一个可以单独写下来的符号常量：符号常量
     * 能被"照着现象翻一下试试"改掉，而力矩极性翻错的表现（正反馈）和增益过大
     * 很像，很容易被误诊成调参问题。写成减法之后，想改它只能重新量飞机。
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
    if ((airframe_params.derived_auto > 0.5f) ||
        (airframe_params.derived_auto < -0.5f)) {
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
    uint8_t auto_on;

    if (entry == NULL) { return 0U; }

    auto_on = ((airframe_params.derived_auto > 0.5f) ||
               (airframe_params.derived_auto < -0.5f)) ? 1U : 0U;

    /*
     * 自动档下拒绝直接写派生值，而不是"写了再被下一次重算覆盖"。
     * 后者上位机会以为写成功了，下次读回来又变了，找不到原因——
     * 诊断宁可当场说不行（D5-3：宁可沉默，不可撒谎）。
     */
    if ((entry->derived != 0U) && (auto_on != 0U)) { return 0U; }

    airframe_set_field(&airframe_params, entry->offset, value);

    /* 改了输入就立刻把派生值跟上，读回来的永远是自洽的一组。 */
    if ((entry->derived == 0U) && (auto_on != 0U)) {
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
