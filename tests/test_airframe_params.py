"""机体模型运行时参数的契约（2026-09-11 立）。

为什么值得单独一套测试：这些数**直接进控制律**——质量、惯量、力臂、推力点到重心的
距离。填错一个不会编译失败、不会报错，只会让每一条力矩换算都偏掉，而症状要等飞起来
才出现，那时已经分不清是调参问题还是模型问题。

本模块不碰 HAL，所以直接用 host gcc 编真实源码跑，不需要任何替身。

钉住的性质：
    1. 派生值确实由输入算出，且公式与改造前的常量一致（拿仓库历史值验算）。
    2. 自动档下拒绝直接写派生值——不是"写了再被覆盖"，那样上位机会以为写成功了。
    3. 缺数据时 valid 为 0，且能说出**第一个**不合格的字段是谁。
    4. NaN 不能被当成有效值放过去。
    5. （2026-09-27）倾转转轴是控制关键量：没填、离重心不到 1 cm、与推力点不在
       重心同侧都禁止解锁，并点名该改哪一项；退役的两个"推力力臂"不在参数表里。
    6. ±Inf 与 NaN 一样不算有效值；derived_auto 由 0 写成 1 当场重算，不等重启。
"""

from __future__ import annotations

from pathlib import Path

from _micoair_hostfakes import CHECK_MACRO, ROOT, build_and_run


DRIVER_INC = ROOT / "Driver" / "Inc"
DRIVER_SRC = ROOT / "Driver" / "Src"


HARNESS = (
    CHECK_MACRO
    + r"""
#include "drv_airframe_params.h"
#include <math.h>
#include <string.h>

/*
 * 仓库改造前 drv_airframe_model.h 里的实测值。用它们验算派生公式：
 * 若公式写错，这里的期望值就对不上——而这些期望值有出处，不是我编的。
 */
static void fill_repo_measurements(DRV_Airframe_Params *p)
{
    memset(p, 0, sizeof(*p));
    p->board_mass_g        = 75.0f;
    p->battery_mass_g      = 232.0f;
    p->base_mass_g         = 99.0f;
    p->servo_motor_mass_g  = 348.6f;
    p->board_cg_z_m        = 0.0f;
    p->battery_cg_z_m      = 0.109f;
    p->base_cg_z_m         = -0.117f;
    p->servo_motor_cg_z_m  = -0.244f;
    p->thrust_point_z_m    = -0.2955f;
    p->tether_attach_z_m   = 0.1563f;
    p->tether_rope_m       = 0.6400f;
    p->gravity_m_s2        = 9.81f;
    p->max_total_thrust_g  = 1595.342f;
    p->servo_deg_per_us    = 0.090f;
    p->ixx_kgm2            = 0.051f;
    p->iyy_kgm2            = 0.051f;
    p->izz_kgm2            = 0.005f;
    /*
     * 2026-09-27 作者实测：两个倾转舵机转轴都在 z = −0.13 m（相交）。倾转力臂
     * 就是它减重心，退役的 pitch/roll_thrust_lever_arm_m 不再填。
     */
    p->servo1_axis_z_m     = -0.13f;
    p->servo2_axis_z_m     = -0.13f;
    p->derived_auto        = 1.0f;
}

static int close_to(float a, float b, float tol)
{
    float d = a - b;
    return ((d < tol) && (d > -tol)) ? 1 : 0;
}

static void check_derived_matches_repo_history(void)
{
    DRV_Airframe_Params p;
    fill_repo_measurements(&p);
    DRV_Airframe_ComputeDerived(&p, &p);

    /* 754.6 g = 四个部件之和。仓库里 MASS_KG 写的是 1.3670——那正是那处矛盾。 */
    CHECK(close_to(p.mass_kg, 0.7546f, 1e-4f), 1);

    /* CG_Z_M 的历史值 -0.0946 就是按这四个部件算出来的，能对上说明公式没写反。 */
    CHECK(close_to(p.cg_z_m, -0.09456f, 1e-4f), 2);

    CHECK(close_to(p.weight_n, 0.7546f * 9.81f, 1e-3f), 3);

    /* TETHER_ATTACH_TO_CG_M 历史值 0.2509 = 0.1563 - (-0.09456)。 */
    CHECK(close_to(p.tether_attach_to_cg_m, 0.2509f, 1e-3f), 4);
    /* TETHER_ROD_TO_CG_M 历史值 0.8909 = 0.64 + 0.2509。 */
    CHECK(close_to(p.tether_rod_to_cg_m, 0.8909f, 1e-3f), 5);

    /* THRUST_POINT_TO_CG_Z_M 必须为负：推力挂在板子下方，倾转力矩的符号靠它。 */
    CHECK(p.thrust_point_to_cg_z_m < 0.0f, 6);
    CHECK(close_to(p.thrust_point_to_cg_z_m, -0.2955f + 0.09456f, 1e-3f), 7);

    CHECK(close_to(p.servo_us_per_deg, 1.0f / 0.090f, 1e-3f), 8);
    CHECK(close_to(p.max_total_force_n, 1.595342f * 9.81f, 1e-3f), 9);
}

static void check_derived_write_is_refused_in_auto(void)
{
    DRV_Airframe_Params p;
    float value = 0.0f;

    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);

    /* 自动档：写派生值必须当场拒绝，而不是写进去再被下次重算悄悄覆盖。 */
    CHECK(DRV_Airframe_SetParam("airframe.mass_kg", 99.0f) == 0U, 20);
    CHECK(DRV_Airframe_GetParam("airframe.mass_kg", &value) == 1U, 21);
    CHECK(close_to(value, 0.7546f, 1e-4f), 22);

    /* 改输入则派生值立刻跟上，读回来永远是自洽的一组。 */
    CHECK(DRV_Airframe_SetParam("airframe.battery_mass_g", 332.0f) == 1U, 23);
    CHECK(DRV_Airframe_GetParam("airframe.mass_kg", &value) == 1U, 24);
    CHECK(close_to(value, 0.8546f, 1e-4f), 25);

    /* 手动档：派生值可写，且不再被重算覆盖——留给双线摆实测惯量这类场景。 */
    CHECK(DRV_Airframe_SetParam("airframe.derived_auto", 0.0f) == 1U, 26);
    CHECK(DRV_Airframe_SetParam("airframe.mass_kg", 1.367f) == 1U, 27);
    CHECK(DRV_Airframe_SetParam("airframe.base_mass_g", 120.0f) == 1U, 28);
    CHECK(DRV_Airframe_GetParam("airframe.mass_kg", &value) == 1U, 29);
    CHECK(close_to(value, 1.367f, 1e-4f), 30);
}

static void check_empty_model_is_invalid(void)
{
    DRV_Airframe_Params p;
    const char *bad;

    DRV_Airframe_Clear();
    CHECK(DRV_Airframe_IsValid() == 0U, 40);

    bad = DRV_Airframe_FirstInvalidName();
    CHECK(bad != NULL, 41);
    /* 必须说得出是谁不合格，不能只回一个 "invalid"。 */
    CHECK(strncmp(bad, "airframe.", 9) == 0, 42);

    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 1U, 43);
    CHECK(DRV_Airframe_FirstInvalidName() == NULL, 44);

    /* 惯量被清零 = 角加速度换算的分母没了，必须立刻判为不可用。 */
    CHECK(DRV_Airframe_SetParam("airframe.izz_kgm2", 0.0f) == 1U, 45);
    CHECK(DRV_Airframe_IsValid() == 0U, 46);
}

static void check_nan_is_not_accepted_as_valid(void)
{
    DRV_Airframe_Params p;

    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 1U, 50);

    /*
     * NaN 比零更危险：它会沿着每一条乘法扩散，最后在某个毫不相干的地方表现成
     * "姿态突然发散"。体检必须把它当成无效，而不是"非零所以合格"。
     */
    CHECK(DRV_Airframe_SetParam("airframe.ixx_kgm2", NAN) == 1U, 51);
    CHECK(DRV_Airframe_IsValid() == 0U, 52);
}

static int name_is(const char *actual, const char *expected)
{
    return (actual != NULL) && (strcmp(actual, expected) == 0);
}

/*
 * 2026-09-27：倾转力臂 = 舵机转轴 z − 重心 z，力矩的大小和方向都由它定。
 * 解锁闸门因此要挡住三类"量了等于没量"的情形，并且说出该改哪一项。
 */
static void check_tilt_axis_arming_gate(void)
{
    DRV_Airframe_Params p;

    /* 2026-09-27 当晚板上几何（部件表重心）：放行，r_z = −0.035442。 */
    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 1U, 70);
    CHECK(close_to(DRV_Airframe_RollTiltAxisToCgZ(DRV_Airframe_Get()),
                   -0.035442f, 2e-6f), 71);
    CHECK(close_to(DRV_Airframe_PitchTiltAxisToCgZ(DRV_Airframe_Get()),
                   -0.035442f, 2e-6f), 72);

    /*
     * 没填（0 = 板子平面）：r_z 会变成 +0.0946，符号反了，起飞即翻。
     * 必须拦住，并点名是哪一个转轴。
     */
    fill_repo_measurements(&p);
    p.servo1_axis_z_m = 0.0f;
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 73);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(), "airframe.servo1_axis_z_m"), 74);

    fill_repo_measurements(&p);
    p.servo2_axis_z_m = 0.0f;
    DRV_Airframe_SetParams(&p);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(), "airframe.servo2_axis_z_m"), 75);

    fill_repo_measurements(&p);
    p.servo2_axis_z_m = NAN;
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 76);

    /* 离重心不到 1 cm：倾转几乎没有力矩，反解会一路顶到限位。 */
    fill_repo_measurements(&p);
    p.servo1_axis_z_m = -0.1000f;          /* r_z ≈ −0.0054 m */
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 77);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo1_axis_z_m:near"), 78);

    fill_repo_measurements(&p);
    p.servo2_axis_z_m = -0.0880f;          /* r_z ≈ +0.0066 m，在重心上方也一样近 */
    DRV_Airframe_SetParams(&p);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo2_axis_z_m:near"), 79);

    /* 刚好 1 cm 以外：放行（门槛只挡"量错/填错"，不挡小力臂的正经构型）。 */
    fill_repo_measurements(&p);
    p.servo1_axis_z_m = -0.1050f;          /* r_z ≈ −0.0104 m */
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 1U, 80);

    /*
     * 符号填反："板下 13 cm"写成 +0.13。转轴跑到重心上方、推力点仍在下方，
     * 力矩极性会整个反掉。两个都是量出来的，不同侧就不放行。
     */
    fill_repo_measurements(&p);
    p.servo1_axis_z_m = 0.13f;
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 81);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo1_axis_z_m:sign"), 82);

    fill_repo_measurements(&p);
    p.servo2_axis_z_m = 0.13f;
    DRV_Airframe_SetParams(&p);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo2_axis_z_m:sign"), 83);

    /* 反过来：转轴对、推力点符号填反，同样拦住（报在转轴上，二者之一错了）。 */
    fill_repo_measurements(&p);
    p.thrust_point_z_m = 0.2955f;
    DRV_Airframe_SetParams(&p);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo1_axis_z_m:sign"), 84);

    /* 在线改：PARAM SET 写一个转轴，闸门当场重判。 */
    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_SetParam("airframe.servo2_axis_z_m", 0.13f) == 1U, 85);
    CHECK(DRV_Airframe_IsValid() == 0U, 86);
    CHECK(DRV_Airframe_SetParam("airframe.servo2_axis_z_m", -0.13f) == 1U, 87);
    CHECK(DRV_Airframe_IsValid() == 1U, 88);
}

/*
 * 退役的两个"推力力臂"不许再从参数表进出：留着就又多了一个会和几何打架的
 * 来源。字段本身还在结构体里（Flash ABI），但读写都必须被拒。
 */
static void check_retired_lever_names_are_gone(void)
{
    uint32_t n = DRV_Airframe_GetParamCount();
    uint32_t i;
    float value = 0.0f;
    DRV_Airframe_Params p;

    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);

    CHECK(DRV_Airframe_GetParam("airframe.pitch_thrust_lever_arm_m", &value) == 0U, 90);
    CHECK(DRV_Airframe_GetParam("airframe.roll_thrust_lever_arm_m", &value) == 0U, 91);
    CHECK(DRV_Airframe_SetParam("airframe.pitch_thrust_lever_arm_m", 0.145f) == 0U, 92);
    CHECK(DRV_Airframe_SetParam("airframe.roll_thrust_lever_arm_m", 0.145f) == 0U, 93);
    CHECK(DRV_Airframe_GetParam("airframe.retired_pitch_thrust_lever_arm_m", &value) == 0U, 94);
    for (i = 0U; i < n; i++) {
        CHECK(strstr(DRV_Airframe_GetParamNameAt(i), "thrust_lever_arm") == NULL, 95);
    }
}

/*
 * 手动档切回自动档（PARAM SET airframe.derived_auto 1）必须**当场**重算：
 * 重心、推力点到重心、由重心现算的倾转力臂和解锁判定都要立刻换成算出来的值。
 * 以前按写入前的开关值判断要不要重算，这一次写入恰好不算——界面显示"自动"，
 * 重心却还是手写值，直到重启才悄悄变。
 */
static void check_switch_to_auto_recomputes_at_once(void)
{
    DRV_Airframe_Params p;
    float value = 0.0f;

    fill_repo_measurements(&p);
    DRV_Airframe_ComputeDerived(&p, &p);
    /* 手写一个贴着转轴的重心：r_z = −0.13 − (−0.125) = −0.005，闸门拦下。 */
    p.derived_auto = 0.0f;
    p.cg_z_m = -0.125f;
    p.thrust_point_to_cg_z_m = p.thrust_point_z_m - p.cg_z_m;
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 100);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo1_axis_z_m:near"), 101);

    CHECK(DRV_Airframe_SetParam("airframe.derived_auto", 1.0f) == 1U, 102);
    CHECK(DRV_Airframe_GetParam("airframe.cg_z_m", &value) == 1U, 103);
    CHECK(close_to(value, -0.094558f, 2e-6f), 104);
    CHECK(close_to(DRV_Airframe_RollTiltAxisToCgZ(DRV_Airframe_Get()),
                   -0.035442f, 2e-6f), 105);
    CHECK(close_to(DRV_Airframe_Get()->thrust_point_to_cg_z_m,
                   -0.2955f + 0.094558f, 2e-6f), 106);
    CHECK(DRV_Airframe_IsValid() == 1U, 107);

    /* 开关本身是输入；切到自动之后，写派生值照旧当场拒绝。 */
    CHECK(DRV_Airframe_SetParam("airframe.cg_z_m", -0.2f) == 0U, 108);

    /* 切回手动：派生值保持切换那一刻的样子，不被清零或改写。 */
    CHECK(DRV_Airframe_SetParam("airframe.derived_auto", 0.0f) == 1U, 109);
    CHECK(close_to(DRV_Airframe_Get()->cg_z_m, -0.094558f, 2e-6f), 110);
    CHECK(DRV_Airframe_IsValid() == 1U, 111);
}

/*
 * ±Inf 是"非零"，但不是量出来的数。必填项写进 Inf（PARAM SET 1e39 解析出来
 * 就是它）、部件表乘积溢出、手动档写进 Inf 重心，都曾经整组通过体检：Inf 满足
 * "非零"，r_z = ±Inf 也满足 |r_z| ≥ 1 cm，再按符号过了同侧核对——然后控制律
 * 拿一个无穷大的力臂去算。
 */
static void check_infinity_is_not_accepted_as_valid(void)
{
    DRV_Airframe_Params p;

    fill_repo_measurements(&p);
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_SetParam("airframe.ixx_kgm2", INFINITY) == 1U, 120);
    CHECK(DRV_Airframe_IsValid() == 0U, 121);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(), "airframe.ixx_kgm2"), 122);

    /* 转轴写成 −Inf：r_z = −Inf 与推力点同侧，旧判据整组放行。 */
    fill_repo_measurements(&p);
    p.servo1_axis_z_m = -INFINITY;
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 123);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(), "airframe.servo1_axis_z_m"), 124);

    /* 自动档部件表溢出：m·z 溢出成 Inf、总质量仍有限 → 重心 = +Inf。 */
    fill_repo_measurements(&p);
    p.battery_mass_g = 1.0e30f;
    p.battery_cg_z_m = 1.0e10f;
    DRV_Airframe_SetParams(&p);
    CHECK(isinf(DRV_Airframe_Get()->cg_z_m), 125);
    CHECK(DRV_Airframe_IsValid() == 0U, 126);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.thrust_point_to_cg_z_m"), 127);

    /*
     * 手动档写进 Inf 重心、推力点到重心照手写的有限值：只剩倾转转轴那道复合
     * 判据拦得住，报 :near（"没有可用的力臂"），不另起原因名。
     */
    fill_repo_measurements(&p);
    DRV_Airframe_ComputeDerived(&p, &p);
    p.derived_auto = 0.0f;
    p.cg_z_m = INFINITY;
    DRV_Airframe_SetParams(&p);
    CHECK(DRV_Airframe_IsValid() == 0U, 128);
    CHECK(name_is(DRV_Airframe_FirstInvalidName(),
                  "airframe.servo1_axis_z_m:near"), 129);
}

static void check_table_is_walkable(void)
{
    uint32_t n = DRV_Airframe_GetParamCount();
    uint32_t i;
    uint32_t derived_seen = 0U;

    /* 上位机靠遍历自动生成表单，所以名字必须全都取得到、不重样。 */
    CHECK(n > 20U, 60);
    for (i = 0U; i < n; i++) {
        const char *name = DRV_Airframe_GetParamNameAt(i);
        float value = 0.0f;
        uint32_t j;

        CHECK(name != NULL, 61);
        CHECK(DRV_Airframe_GetParam(name, &value) == 1U, 62);
        derived_seen += DRV_Airframe_IsDerivedName(name);

        for (j = 0U; j < i; j++) {
            CHECK(strcmp(DRV_Airframe_GetParamNameAt(j), name) != 0, 63);
        }
    }
    CHECK(derived_seen >= 8U, 64);
    CHECK(DRV_Airframe_GetParamNameAt(n) == NULL, 65);
}

int main(void)
{
    check_derived_matches_repo_history();
    check_derived_write_is_refused_in_auto();
    check_empty_model_is_invalid();
    check_nan_is_not_accepted_as_valid();
    check_tilt_axis_arming_gate();
    check_retired_lever_names_are_gone();
    check_switch_to_auto_recomputes_at_once();
    check_infinity_is_not_accepted_as_valid();
    check_table_is_walkable();
    REPORT();
}
"""
)


def test_airframe_params_contract(tmp_path: Path) -> None:
    result = build_and_run(
        tmp_path,
        "airframe_params",
        HARNESS,
        sources=[DRIVER_SRC / "drv_airframe_params.c"],
        includes=[DRIVER_INC],
    )
    assert result.returncode == 0, result.stdout + result.stderr
