"""悬停推力自适应接进控制器（2026-10-01）。

作者原话："让自适应学习出现效果，然后可以配合积分适当补偿"，并确认做法："收敛之后才用；
限制在初值上下 15% 以内，慢慢过渡；每次解锁重新学"。

起因：三次自由飞推力指令中位 13.4–13.6 N，竖直积分常驻 −0.51～−0.58 m/s²，配置 14.25 N 偏高约 5%；
学习器原来只学不用。本文件在宿主上真编译：
* Driver/Inc/drv_hover_adapt.h 的三个纯算术（目标、过渡、积分无扰补偿）；
* App/Src/app_hover_adapt.c 的策略（外部依赖用桩）：解锁上升沿重置学习器、重置前不用旧收敛值、±15%、
  0.5 N/s 过渡、未收敛保持、辨识等旁路时不动、关掉后回配置值；
* 真控制器（drv_coax_ctrl.c 等）：换悬停推力的当拍合推力不跳，竖直积分按 (g+a)(H0/H1−1) 挪。
"""
from __future__ import annotations

import ctypes
import shutil
import subprocess
from pathlib import Path

import pytest

from _airframe_fixture import AIRFRAME_FIXTURE_C

ROOT = Path(__file__).resolve().parents[1]
G = 9.80665


def _gcc():
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("宿主没有 gcc")
    return gcc


# --------------------------------------------------------------------------- 纯算术
MATH = r"""
#include <stdio.h>
#include "drv_hover_adapt.h"
int main(void) {
    printf("%.6f %.6f %.6f %.6f %.6f\n",
        DRV_HoverAdapt_Target(1, 1, 13.5f, 14.25f, 14.25f),
        DRV_HoverAdapt_Target(1, 1, 10.0f, 14.25f, 14.25f),
        DRV_HoverAdapt_Target(1, 1, 30.0f, 14.25f, 14.25f),
        DRV_HoverAdapt_Target(1, 0, 13.5f, 14.25f, 13.9f),
        DRV_HoverAdapt_Target(0, 1, 13.5f, 14.25f, 13.9f));
    printf("%.6f %.6f %.6f\n",
        DRV_HoverAdapt_Slew(14.25f, 13.5f, 0.1f),
        DRV_HoverAdapt_Slew(14.25f, 14.27f, 0.1f),
        DRV_HoverAdapt_Slew(0.0f, 13.5f, 0.1f));
    printf("%.6f %.6f\n",
        DRV_HoverAdapt_IntegratorShift(14.25f, 13.5f, 9.80665f, -0.55f),
        DRV_HoverAdapt_IntegratorShift(14.0f, 0.0f, 9.80665f, 0.0f));
    return 0;
}
"""


@pytest.fixture(scope="module")
def math_out(tmp_path_factory):
    d = tmp_path_factory.mktemp("hamath")
    (d / "m.c").write_text(MATH, encoding="utf-8")
    exe = d / "m.exe"
    subprocess.run([_gcc(), "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{ROOT / 'Driver/Inc'}",
                    str(d / "m.c"), "-lm", "-o", str(exe)], check=True)
    return [list(map(float, ln.split())) for ln in
            subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.split("\n") if ln]


def test_target_only_uses_converged_estimate_inside_15_percent(math_out):
    t = math_out[0]
    assert t[0] == pytest.approx(13.5)
    assert t[1] == pytest.approx(14.25 * 0.85)        # 学歪了也只到 −15%
    assert t[2] == pytest.approx(14.25 * 1.15)
    assert t[3] == pytest.approx(13.9)                 # 未收敛：保持当前已用值
    assert t[4] == pytest.approx(14.25)                # 关掉：回配置值


def test_slew_is_half_newton_per_second(math_out):
    s = math_out[1]
    assert s[0] == pytest.approx(14.20)
    assert s[1] == pytest.approx(14.27)
    assert s[2] == pytest.approx(13.5)                 # 未起步直接取目标


def test_integrator_shift_keeps_thrust(math_out):
    shift, bad = math_out[2]
    a0 = -0.55
    assert 14.25 * (1 + a0 / G) == pytest.approx(13.5 * (1 + (a0 + shift) / G), rel=1e-5)
    assert bad == 0.0


# --------------------------------------------------------------------------- 策略层
STUB_C = r"""
#include "app_hover_thrust.h"
uint8_t APP_HoverThrust_GetSnapshot(APP_HoverThrustSnapshot *out) { (void)out; return 0U; }
void APP_HoverThrust_RequestReset(void) {}
float DRV_COAX_CTRL_ConfiguredHoverThrustN(void) { return 14.25f; }
void DRV_COAX_CTRL_SetHoverThrustAdapt(float h) { (void)h; }
"""


class In(ctypes.Structure):
    _fields_ = [("dt_s", ctypes.c_float), ("armed", ctypes.c_uint8), ("bypass", ctypes.c_uint8),
                ("enabled", ctypes.c_uint8), ("converged", ctypes.c_uint8), ("samples", ctypes.c_uint32),
                ("est_n", ctypes.c_float), ("cfg_n", ctypes.c_float)]


class St(ctypes.Structure):
    _fields_ = [("applied_n", ctypes.c_float), ("armed_prev", ctypes.c_uint8), ("wait_reset", ctypes.c_uint8)]


@pytest.fixture(scope="module")
def pol(tmp_path_factory):
    d = tmp_path_factory.mktemp("hapol")
    (d / "stub.c").write_text(STUB_C, encoding="utf-8")
    out = d / "pol.dll"
    subprocess.run([_gcc(), "-shared", "-O0", "-Wall", "-Werror", f"-I{ROOT / 'App/Inc'}",
                    f"-I{ROOT / 'Driver/Inc'}", str(ROOT / "App/Src/app_hover_adapt.c"), str(d / "stub.c"),
                    "-lm", "-o", str(out)], check=True)
    dll = ctypes.CDLL(str(out))
    dll.APP_HoverAdapt_Step.argtypes = [ctypes.POINTER(St), ctypes.POINTER(In)]
    dll.APP_HoverAdapt_Step.restype = ctypes.c_uint8
    return dll


class Pol:
    def __init__(self, lib):
        self.lib, self.st = lib, St()
        self.i = In(dt_s=0.002, armed=0, bypass=0, enabled=1, converged=0, samples=0, est_n=14.25, cfg_n=14.25)

    def run(self, seconds, **kw):
        for k, v in kw.items():
            setattr(self.i, k, v)
        act = 0
        for _ in range(max(1, int(round(seconds / 0.002)))):
            act |= self.lib.APP_HoverAdapt_Step(ctypes.byref(self.st), ctypes.byref(self.i))
        return act


def test_rearm_resets_learner_and_ignores_stale_converged_estimate(pol):
    p = Pol(pol)
    p.run(0.1)
    assert p.st.applied_n == pytest.approx(14.25)
    # 上一飞次留下的收敛值（快照还没重置，samples≠0）：解锁这拍请求重置，且不拿旧值
    act = p.run(0.002, armed=1, converged=1, samples=900, est_n=13.0)
    assert act == 1
    p.run(0.5)
    assert p.st.applied_n == pytest.approx(14.25)
    # 学习器真正重置（samples 归零）之后、再次收敛前：仍是配置值
    p.run(0.1, converged=0, samples=0, est_n=14.25)
    p.run(0.5, converged=0, samples=300, est_n=13.6)
    assert p.st.applied_n == pytest.approx(14.25)


def test_converged_estimate_slews_in_and_holds_when_unconverged(pol):
    p = Pol(pol)
    p.run(0.01)
    p.run(0.002, armed=1)
    p.run(0.01, samples=0)
    p.run(1.0, converged=1, samples=600, est_n=13.5)
    assert p.st.applied_n == pytest.approx(13.75, abs=0.01)   # 0.5 N/s
    p.run(1.0)
    assert p.st.applied_n == pytest.approx(13.5, abs=1e-4)
    p.run(1.0, converged=0, est_n=20.0)                       # 学习器不稳：保持
    assert p.st.applied_n == pytest.approx(13.5, abs=1e-4)
    p.run(0.5, bypass=1, converged=1, est_n=12.5)             # 辨识等旁路：不动
    assert p.st.applied_n == pytest.approx(13.5, abs=1e-4)
    p.run(10.0, bypass=0, est_n=5.0)                          # 学歪：只到 −15%
    assert p.st.applied_n == pytest.approx(14.25 * 0.85, abs=1e-3)
    p.run(10.0, enabled=0)                                    # HOVER ADAPT OFF：回配置值
    assert p.st.applied_n == pytest.approx(14.25, abs=1e-3)
    p.run(0.01, armed=0, enabled=1)
    assert p.st.applied_n == pytest.approx(14.25)


# --------------------------------------------------------------------------- 真控制器：无扰切换
CTRL = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"
#include <stdio.h>
#include <string.h>
static void run(int change, float *force, float *zi) {
    DRV_COAX_CTRL_AttitudeInput a; DRV_COAX_CTRL_Reference r; DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output o; DRV_COAX_CTRL_Debug d; DRV_COAX_CTRL_Params p;
    memset(&a, 0, sizeof(a)); memset(&r, 0, sizeof(r)); memset(&s, 0, sizeof(s));
    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_GetParams(&p);
    p.hover_thrust_n = 14.0f;
    DRV_COAX_CTRL_SetParams(&p);
    DRV_COAX_CTRL_SetHoverThrustAdapt(0.0f);
    DRV_COAX_CTRL_ResetState();
    r.dt_sec = 0.01f; r.horizontal_velocity_valid = 1U;
    r.navigation_position_valid = 1U; r.navigation_velocity_valid = 1U;
    r.z_m = 0.5f; a.z_m = 0.42f; a.acceleration_valid = 1U;
    s.position_update = s.velocity_update = s.attitude_update = s.rate_update = 1U;
    s.integrator_enable = 1U;
    s.position_dt_s = 0.02f; s.velocity_dt_s = 0.01f; s.attitude_dt_s = 0.004f; s.rate_dt_s = 0.002f;
    for (int i = 0; i < 60; ++i) { DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o); }
    if (change) { DRV_COAX_CTRL_SetHoverThrustAdapt(13.0f); }
    DRV_COAX_CTRL_RunScheduled(&a, &r, &s, &o);
    DRV_COAX_CTRL_GetLastDebug(&d);
    *force = d.total_force_n; *zi = d.velocity_i_m_s2[2];
}
int main(void) {
    float f0, z0, f1, z1;
    airframe_load_reference();
    run(0, &f0, &z0);
    run(1, &f1, &z1);
    printf("%.6f %.6f %.6f %.6f %.6f\n", f0, z0, f1, z1, DRV_COAX_CTRL_EffectiveMassKg(1.0f) * 9.80665f);
    DRV_COAX_CTRL_SetHoverThrustAdapt(100.0f);   /* 越界 = 不覆盖 */
    printf("%.6f %.6f\n", DRV_COAX_CTRL_GetHoverThrustAdapt(), DRV_COAX_CTRL_ConfiguredHoverThrustN());
    return 0;
}
"""


@pytest.fixture(scope="module")
def ctrl_out(tmp_path_factory):
    d = tmp_path_factory.mktemp("hactrl")
    stub = d / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text("#define BSP_PWM_ESC_MIN_US 1000U\n#define BSP_PWM_ESC_MAX_US 2000U\n",
                                    encoding="ascii")
    (d / "c.c").write_text(CTRL, encoding="utf-8")
    exe = d / "c.exe"
    srcs = ["drv_airframe_params.c", "drv_prop_map.c", "drv_coax_ctrl.c", "drv_position_control.c",
            "drv_attitude_control.c", "drv_rate_control.c"]
    r = subprocess.run([_gcc(), "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{stub}", f"-I{ROOT / 'Driver/Inc'}",
                        *(str(ROOT / "Driver/Src" / s) for s in srcs), str(d / "c.c"), "-lm", "-o", str(exe)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    lines = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.split("\n")
    return [list(map(float, ln.split())) for ln in lines if ln]


def test_switching_hover_thrust_does_not_jump_total_force(ctrl_out):
    f0, z0, f1, z1, eff = ctrl_out[0]
    assert f0 > 13.0                                   # 有竖直误差，确实在出力
    assert f1 == pytest.approx(f0, abs=0.01)           # 14 → 13 N：合推力当拍不跳（不补偿会掉约 7%）
    assert z1 - z0 == pytest.approx((G + (f0 / 14.0 - 1) * G) * (14.0 / 13.0 - 1), rel=0.05)
    assert eff == pytest.approx(13.0, rel=1e-3)          # 机体参数 g=9.81，此处乘的是 9.80665


def test_out_of_range_override_falls_back_to_configured(ctrl_out):
    override, cfg = ctrl_out[1]
    assert override == 0.0 and cfg == pytest.approx(14.0)


def test_wiring_contract():
    stab = (ROOT / "App/Src/app_stabilizer.c").read_text(encoding="utf-8")
    learn = stab.index("APP_HoverThrust_Step(&in);")
    assert stab.index("APP_HoverAdapt_Update(frame->ctrl_dt_sec, frame->rc_armed,") > learn
    assert "App/Src/app_hover_adapt.c" in (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    cmd = (ROOT / "App/Src/app_cmd_hover.c").read_text(encoding="utf-8")
    assert '"adapt=%s applied_n=%s cfg_n=%s\\r\\n"' in cmd and '"ADAPT"' in cmd
    assert "HoverAdapt" not in (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8")
