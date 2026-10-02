"""coax.hover_thrust_n：按悬停推力换算合力的有效质量（2026-09-30 槽式台架 Z 辨识）。

实测每 1 N 推力表读数给 0.683 m/s² 竖直加速度 = g/14.25，比 1/m 小约 20%：按 m·g 前馈时速度环积分
要背约 2.2 m/s²，而积分限幅只有 1.5。hover_thrust_n > 0 时合力 = hover_thrust_n/g·(a + g)，
三轴同乘（倾角不变）；= 0 时与原来的 mass·(a + g) 逐位相同。只在 RAM。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from _airframe_fixture import AIRFRAME_FIXTURE_C

ROOT = Path(__file__).resolve().parents[1]

HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"
#include "drv_airframe_params.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define CHECK(x,n) do { if (!(x)) { printf("fail %d\n", n); return (n); } } while (0)

static void run(float hover, DRV_COAX_CTRL_Debug *d)
{
    DRV_COAX_CTRL_AttitudeInput a = {0};
    DRV_COAX_CTRL_Reference r = {0};
    DRV_COAX_CTRL_Schedule s = {0};
    DRV_COAX_CTRL_Output o;
    DRV_COAX_CTRL_Params p;

    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_GetParams(&p);
    p.rate.kp[0] = p.rate.kp[1] = p.rate.kp[2] = 0.0f;
    p.attitude.att_kp[0] = p.attitude.att_kp[1] = p.attitude.att_kp[2] = 0.0f;
    p.hover_thrust_n = hover;
    DRV_COAX_CTRL_SetParams(&p);
    r.dt_sec = 0.01f;
    r.horizontal_velocity_valid = 1U;
    r.navigation_position_valid = 1U;
    r.navigation_velocity_valid = 1U;
    r.vx_m_s = 0.2f;            /* 水平速度误差 → a_x */
    r.z_m = 0.10f;              /* 高度误差 → a_z */
    a.acceleration_valid = 1U;
    s.position_update = s.velocity_update = s.attitude_update = s.rate_update = 1U;
    s.position_dt_s = 0.02f; s.velocity_dt_s = 0.01f;
    s.attitude_dt_s = 0.004f; s.rate_dt_s = 0.002f;
    DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    DRV_COAX_CTRL_GetLastDebug(d);
}

int main(void) {
    airframe_load_reference();
    const DRV_Airframe_Params *af = DRV_Airframe_Get();
    const float m = af->mass_kg, g = af->gravity_m_s2;
    DRV_COAX_CTRL_Debug off, on;

    run(0.0f, &off);
    CHECK(fabsf(off.accel_out_m_s2[0]) > 0.01f && fabsf(off.accel_out_m_s2[2]) > 0.01f, 1);
    /* 关：合力就是原来的 m·(a + g)（同一个表达式，逐位相同）。 */
    const float fx0 = m * off.accel_out_m_s2[0], fy0 = m * off.accel_out_m_s2[1];
    const float fz0 = m * (g + off.accel_out_m_s2[2]);
    CHECK(fabsf(off.total_force_n - sqrtf(fx0 * fx0 + fy0 * fy0 + fz0 * fz0)) < 1.0e-4f, 2);

    const float hover = 14.25f;
    run(hover, &on);
    for (int i = 0; i < 3; ++i) {
        CHECK(fabsf(on.accel_out_m_s2[i] - off.accel_out_m_s2[i]) < 1.0e-6f, 3);   /* 外环不受影响 */
    }
    const float me = hover / g;
    const float fx = me * on.accel_out_m_s2[0], fy = me * on.accel_out_m_s2[1];
    const float fz = me * (g + on.accel_out_m_s2[2]);
    CHECK(fabsf(on.total_force_n - sqrtf(fx * fx + fy * fy + fz * fz)) < 1.0e-4f, 4);
    CHECK(fabsf(on.total_force_n / off.total_force_n - me / m) < 1.0e-4f, 5);        /* 三轴同比放大 */
    CHECK(fabsf(on.target_attitude_rp_rad[0] - off.target_attitude_rp_rad[0]) < 1.0e-6f, 6);  /* 倾角不变 */
    CHECK(fabsf(on.target_attitude_rp_rad[1] - off.target_attitude_rp_rad[1]) < 1.0e-6f, 7);

    /* 有效质量函数：开 = hover/g，关 = 原样。 */
    CHECK(fabsf(DRV_COAX_CTRL_EffectiveMassKg(1.234f) - me) < 1.0e-6f, 8);
    DRV_COAX_CTRL_ResetParams();
    CHECK(DRV_COAX_CTRL_EffectiveMassKg(1.234f) == 1.234f, 9);

    /* 范围：0 = 关，否则 [1, 40] N；融合开关 0～1。 */
    CHECK(DRV_COAX_CTRL_SetParam("coax.hover_thrust_n", 0.0f) == 1U, 10);
    CHECK(DRV_COAX_CTRL_SetParam("coax.hover_thrust_n", 0.5f) == 0U, 11);
    CHECK(DRV_COAX_CTRL_SetParam("coax.hover_thrust_n", 1.0f) == 1U, 12);
    CHECK(DRV_COAX_CTRL_SetParam("coax.hover_thrust_n", 40.0f) == 1U, 13);
    CHECK(DRV_COAX_CTRL_SetParam("coax.hover_thrust_n", 40.5f) == 0U, 14);
    CHECK(DRV_COAX_CTRL_SetParam("coax.hover_thrust_n", -1.0f) == 0U, 15);
    CHECK(DRV_COAX_CTRL_SetParam("coax.z_vel_fusion", 1.0f) == 1U, 16);
    CHECK(DRV_COAX_CTRL_SetParam("coax.z_vel_fusion", 1.5f) == 0U, 17);
    CHECK(DRV_COAX_CTRL_SetParam("coax.z_vel_fusion", 0.0f) == 1U, 18);
    /* 默认：悬停推力关（0）、融合开（1，高度环 3/8/3 在融合关时不稳）。 */
    DRV_COAX_CTRL_ResetParams();
    float v = -1.0f;
    CHECK(DRV_COAX_CTRL_GetParam("coax.hover_thrust_n", &v) == 1U && v == 0.0f, 19);
    CHECK(DRV_COAX_CTRL_GetParam("coax.z_vel_fusion", &v) == 1U && v == 1.0f, 20);
    return 0;
}
"""

SOURCES = ["drv_airframe_params.c", "drv_prop_map.c", "drv_coax_ctrl.c", "drv_position_control.c",
           "drv_attitude_control.c", "drv_rate_control.c"]


def test_hover_thrust_scales_the_force_on_all_axes_and_keeps_the_tilt(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required")
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text(
        "#define BSP_PWM_ESC_MIN_US 1000U\n#define BSP_PWM_ESC_MAX_US 2000U\n", encoding="ascii")
    harness = tmp_path / "hover.c"
    exe = tmp_path / "hover.exe"
    harness.write_text(HARNESS, encoding="utf-8")
    result = subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{stub}", f"-I{ROOT / 'Driver' / 'Inc'}",
         *(str(ROOT / "Driver" / "Src" / name) for name in SOURCES), str(harness), "-lm", "-o", str(exe)],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stdout + run.stderr


def test_the_force_law_uses_the_effective_mass_everywhere() -> None:
    """生产合力三处与 ALT vel/pos 闭环都走 DRV_COAX_CTRL_EffectiveMassKg，不再直接乘 mass_kg。"""
    source = (ROOT / "Driver/Src/drv_coax_ctrl.c").read_text(encoding="utf-8")
    assert "coax_ctrl_params.mass_kg * debug->accel_out_m_s2" not in source
    assert source.count("mass_eff_kg * debug->accel_out_m_s2") == 2
    assert "mass_eff_kg *\n            (coax_ctrl_params.gravity_m_s2" in source.replace("\r\n", "\n")
    alt = (ROOT / "App/Src/app_sysid_alt.c").read_text(encoding="utf-8")
    assert "DRV_COAX_CTRL_EffectiveMassKg(alt.mass_kg)" in alt
    assert "total_n = alt.mass_kg *" not in alt
