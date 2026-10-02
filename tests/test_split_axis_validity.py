"""光流失效不拖累高度环 + 满推力给偏航留余量（2026-10-01 自由飞事故，receive_lok1zuby 72.8–74.9 s）。

链条：大倾角掉高 → 两桨顶满 1940 µs → 偏航余量归零、机身转到 −170 °/s → 光流速度判无效 1.7 s →
调度器只认光流样本令牌，位置/速度环（含高度）冻结在最大推力 → 冲到 1.7 m，作者收油门无效、只能上锁。
本文件在宿主上真编译控制器，钉住：
* split_axis_validity=1 时，光流无效（水平）而测距有效（竖直）：高度环照常对误差出力，水平误差按 0；
* split_axis_validity=0 时与旧语义逐位相同；
* 合推力上限 = min(机体总推力上限, 2·T单max − 0.8 N)。
源码契约：稳定器的调度令牌拼了测距样本时刻、参考里置分轴有效性。
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from _airframe_fixture import AIRFRAME_FIXTURE_C

ROOT = Path(__file__).resolve().parents[1]

CTRL = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"
#include <stdio.h>
#include <string.h>
/* 低电量时的推力映射：ESC 满行程两桨合推力 15.7 N（日志事故时刻的水平），其余线性。 */
static float map_total(uint16_t us) { return (us <= 1000U) ? 0.0f : 15.7f * (float)(us - 1000U) / 1000.0f; }
static uint16_t map_pulse(float n) { return (uint16_t)(1000.0f + 1000.0f * (2.0f * n) / 15.7f); }
static const DRV_COAX_CTRL_ThrustMap low_batt = { map_pulse, map_total, NULL };
static void base(DRV_COAX_CTRL_AttitudeInput *a, DRV_COAX_CTRL_Reference *r, DRV_COAX_CTRL_Schedule *s) {
    memset(a, 0, sizeof(*a)); memset(r, 0, sizeof(*r)); memset(s, 0, sizeof(*s));
    r->dt_sec = 0.01f; r->horizontal_velocity_valid = 1U;
    a->acceleration_valid = 1U;
    s->position_update = s->velocity_update = s->attitude_update = s->rate_update = 1U;
    s->integrator_enable = 1U;
    s->position_dt_s = 0.02f; s->velocity_dt_s = 0.01f; s->attitude_dt_s = 0.004f; s->rate_dt_s = 0.002f;
}
/* mode 0: 旧语义且导航全无效；1: 分轴、光流无效、竖直有效；2: 分轴、全有效；3: 旧语义全有效 */
static void run(int mode, float *az, float *ax, float *vzsp) {
    DRV_COAX_CTRL_AttitudeInput a; DRV_COAX_CTRL_Reference r; DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output o; DRV_COAX_CTRL_Debug d;
    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_SetHoverThrustAdapt(0.0f);
    DRV_COAX_CTRL_ResetState();
    base(&a, &r, &s);
    r.z_m = 0.40f; a.z_m = 1.30f; a.vz_m_s = 1.2f;          /* 冲高：高 0.9 m、上升 1.2 m/s */
    r.x_m = 0.0f; a.x_m = 0.5f; a.vx_m_s = 0.8f;            /* 水平也有大误差（光流陈旧值） */
    if (mode == 1 || mode == 2) { r.split_axis_validity = 1U; r.vertical_measurement_valid = 1U; }
    if (mode == 2 || mode == 3) { r.navigation_position_valid = 1U; r.navigation_velocity_valid = 1U; }
    for (int i = 0; i < 3; ++i) { DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o); }
    DRV_COAX_CTRL_GetLastDebug(&d);
    *az = d.accel_out_m_s2[2]; *ax = d.accel_out_m_s2[0]; *vzsp = d.velocity_sp_m_s[2];
}
int main(void) {
    float az, ax, vz;
    airframe_load_reference();
    for (int m = 0; m < 4; ++m) { run(m, &az, &ax, &vz); printf("%d %.6f %.6f %.6f\n", m, az, ax, vz); }
    {   /* 合推力上限：手动推力要 40 N */
        DRV_COAX_CTRL_AttitudeInput a; DRV_COAX_CTRL_Reference r; DRV_COAX_CTRL_Schedule s;
        DRV_COAX_CTRL_Output o; DRV_COAX_CTRL_Debug d;
        DRV_COAX_CTRL_ResetParams(); DRV_COAX_CTRL_ResetState();
        DRV_COAX_CTRL_SetThrustMap(&low_batt);
        base(&a, &r, &s);
        r.manual_total_force_valid = 1U; r.manual_total_force_n = 40.0f; r.direct_attitude_target_valid = 1U;
        DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
        DRV_COAX_CTRL_GetLastDebug(&d);
        printf("cap %.6f %.6f %u\n", d.total_force_n, DRV_COAX_CTRL_SingleMaxThrustN(), (unsigned)o.thrust_saturated);
        DRV_COAX_CTRL_SetThrustMap(NULL);
    }
    return 0;
}
"""


@pytest.fixture(scope="module")
def out(tmp_path_factory):
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("宿主没有 gcc")
    d = tmp_path_factory.mktemp("split")
    stub = d / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text("#define BSP_PWM_ESC_MIN_US 1000U\n#define BSP_PWM_ESC_MAX_US 2000U\n",
                                    encoding="ascii")
    (d / "c.c").write_text(CTRL, encoding="utf-8")
    exe = d / "c.exe"
    srcs = ["drv_airframe_params.c", "drv_prop_map.c", "drv_coax_ctrl.c", "drv_position_control.c",
            "drv_attitude_control.c", "drv_rate_control.c"]
    r = subprocess.run([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{stub}", f"-I{ROOT / 'Driver/Inc'}",
                        *(str(ROOT / "Driver/Src" / s) for s in srcs), str(d / "c.c"), "-lm", "-o", str(exe)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    rows = {}
    for ln in subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.split("\n"):
        if ln:
            k, *v = ln.split()
            rows[k] = [float(x) for x in v]
    return rows


def test_flow_invalid_still_closes_altitude_loop(out):
    az, ax, vzsp = out["1"]
    assert vzsp < -0.1                # 高 0.9 m：位置环要下降
    assert az < -1.0                  # 竖直速度误差大：要减推力（旧逻辑此时冻结在最大推力）
    assert abs(ax) < 1e-6             # 水平测量无效：不拿陈旧光流去打倾角


def test_split_with_all_valid_matches_legacy(out):
    assert out["2"] == pytest.approx(out["3"], abs=1e-6)


def test_legacy_semantics_unchanged_when_navigation_invalid(out):
    az, ax, vzsp = out["0"]
    assert vzsp == 0.0                # 旧语义：位置测量无效 → 速度目标 0（三轴一起）


def test_total_force_reserves_yaw_headroom(out):
    total, tmax, sat = out["cap"]
    assert tmax == pytest.approx(7.85, abs=1e-3)              # 低电量映射：单桨 7.85 N
    assert total == pytest.approx(2 * tmax - 0.8, abs=1e-3)   # 15.7 − 0.8，比机体上限 15.65 更紧
    assert sat == 1


def test_stabilizer_wiring():
    c = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    assert "(((uint64_t)navigation_state.height_sample_ms) << 32) |" in c
    assert "(navigation_state.height_valid != 0U)) ? 1U : 0U," in c
    assert "frame->reference.split_axis_validity = 1U;" in c
    assert "frame->reference.vertical_measurement_valid = frame->range_height_valid;" in c
    d = (ROOT / "Driver/Src/drv_coax_ctrl.c").read_text(encoding="utf-8")
    assert "#define DRV_COAX_CTRL_YAW_RESERVE_N        0.8f" in d
