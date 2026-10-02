"""悬停推力在线估计（Driver/Src/drv_hover_thrust_est.c + App/Src/app_hover_thrust.c）—— 宿主 gcc 编真 C。

四组判据：

* **算法单元**：收敛、跟踪慢漂、新息门限拒收与失锁恢复、learn=0 不更新、重置、非法输入；
* **何时学（策略层）**：压在支撑面/未解锁/封顶/倾角大/加速度无效时不学，离地后才学，初值来源；
* **实录回放**：2026-09-30 槽式台架 ALT 实录逐拍喂同一份 C 代码，收敛值须在 14.25 ± 0.3 N，
  初值给错（11.6 / 17 N）也要收敛，压在槽底的样本不进滤波器（回放脚本
  data/analysis/sysid-rig-params/2026-09-30/scripts/hover_est_replay.py 的库构建与读取共用）；
* **源码契约**：HOVER? 命令存在并挂在兜底链上、app_control.c 不许变长、学习条件含"离开支撑面"、
  接线在 app_stabilizer.c、估计值不回写 coax.hover_thrust_n。
"""
from __future__ import annotations

import ctypes
import importlib.util
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
DRV_SRC = ROOT / "Driver/Src/drv_hover_thrust_est.c"
APP_SRC = ROOT / "App/Src/app_hover_thrust.c"
APP_HDR = ROOT / "App/Inc/app_hover_thrust.h"
CMD_SRC = ROOT / "App/Src/app_cmd_hover.c"
FALLBACK = ROOT / "App/Src/app_cmd_fallback.c"
STABILIZER = ROOT / "App/Src/app_stabilizer.c"
APP_CONTROL = ROOT / "App/Src/app_control.c"
CMAKE = ROOT / "CMakeLists.txt"

# app_control.c 的天花板：写本测试时的行数与字节数（作者硬约束：只减不增）。
APP_CONTROL_MAX_LINES = 3308
APP_CONTROL_MAX_BYTES = 129928

G = 9.80665
RESULT_HELD, RESULT_ACCEPTED, RESULT_REJECTED, RESULT_IGNORED = range(4)

SHIM = r"""
#include <stdint.h>
#include "drv_hover_thrust_est.h"

static DRV_HoverEst he;

uint8_t hd_init(float init_n) { return DRV_HoverEst_Init(&he, 0, init_n); }
uint8_t hd_init_params(float init_n, const float *p, const uint32_t *u)
{
    DRV_HoverEstParams params;
    DRV_HoverEst_DefaultParams(&params);
    params.init_std_n = p[0]; params.process_std_n_sqrt_s = p[1]; params.meas_std_init = p[2];
    params.meas_std_min = p[3]; params.meas_std_max = p[4]; params.noise_alpha = p[5];
    params.gate_sigma = p[6]; params.inflate_std_n = p[7]; params.thrust_lag_s = p[8];
    params.accel_lpf_s = p[9]; params.converged_std_n = p[10];
    params.reject_inflate = (uint16_t)u[0]; params.converged_samples = u[1];
    return DRV_HoverEst_Init(&he, &params, init_n);
}
void hd_reset(void) { DRV_HoverEst_Reset(&he); }
int32_t hd_step(float dt, float thrust, float roll, float pitch, float a, float g, uint8_t learn)
{
    DRV_HoverEstInput in;
    in.dt_s = dt; in.thrust_n = thrust; in.roll_rad = roll; in.pitch_rad = pitch;
    in.a_up_m_s2 = a; in.gravity_m_s2 = g; in.learn = learn;
    return (int32_t)DRV_HoverEst_Step(&he, &in);
}
/* out = est, std, init, innov, meas_std；cnt = samples, rejected；flags = converged, learning */
void hd_out(float *out, uint32_t *cnt, uint8_t *flags)
{
    DRV_HoverEstOutput o;
    DRV_HoverEst_GetOutput(&he, &o);
    out[0] = o.est_n; out[1] = o.std_n; out[2] = o.init_n; out[3] = o.innov_m_s2;
    out[4] = o.meas_std_m_s2; cnt[0] = o.samples; cnt[1] = o.rejected;
    flags[0] = o.converged; flags[1] = o.learning;
}
uint8_t hd_params_valid_gate(float gate)
{
    DRV_HoverEstParams p;
    DRV_HoverEst_DefaultParams(&p);
    p.gate_sigma = gate;
    return DRV_HoverEst_ParamsValid(&p);
}
"""


class DriverLib:
    def __init__(self, path: Path) -> None:
        h = ctypes.CDLL(str(path))
        f32, u8, u32 = ctypes.c_float, ctypes.c_uint8, ctypes.c_uint32
        h.hd_init.argtypes, h.hd_init.restype = [f32], u8
        h.hd_init_params.argtypes = [f32, ctypes.POINTER(f32), ctypes.POINTER(u32)]
        h.hd_init_params.restype = u8
        h.hd_step.argtypes = [f32, f32, f32, f32, f32, f32, u8]
        h.hd_step.restype = ctypes.c_int32
        h.hd_out.argtypes = [ctypes.POINTER(f32), ctypes.POINTER(u32), ctypes.POINTER(u8)]
        h.hd_params_valid_gate.argtypes, h.hd_params_valid_gate.restype = [f32], u8
        self.h = h

    def init(self, init_n: float) -> int:
        return self.h.hd_init(init_n)

    def step(self, dt, thrust, a, learn=1, roll=0.0, pitch=0.0, g=G) -> int:
        return self.h.hd_step(dt, thrust, roll, pitch, a, g, learn)

    def out(self) -> dict:
        o, c, f = (ctypes.c_float * 5)(), (ctypes.c_uint32 * 2)(), (ctypes.c_uint8 * 2)()
        self.h.hd_out(o, c, f)
        return dict(est=o[0], std=o[1], init=o[2], innov=o[3], meas_std=o[4], samples=c[0],
                    rejected=c[1], converged=f[0], learning=f[1])


def _compile(sources: list[Path], out: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", "-Wshadow",
         "-Wdouble-promotion", "-Wconversion", "-Werror", "-I", str(ROOT / "Driver/Inc"),
         "-I", str(ROOT / "App/Inc"), *map(str, sources), "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.fixture(scope="module")
def drv(tmp_path_factory) -> DriverLib:
    work = tmp_path_factory.mktemp("hover_drv")
    (work / "shim.c").write_text(SHIM, encoding="utf-8")
    out = work / ("hover_drv.dll" if os.name == "nt" else "hover_drv.so")
    _compile([DRV_SRC, work / "shim.c"], out)
    return DriverLib(out)


def _load_replay_module():
    path = ROOT / "data/analysis/sysid-rig-params/2026-09-30/scripts/hover_est_replay.py"
    spec = importlib.util.spec_from_file_location("hover_est_replay", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["hover_est_replay"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def replay_mod():
    return _load_replay_module()


@pytest.fixture(scope="module")
def app(replay_mod, tmp_path_factory):
    """策略层 + 驱动的宿主库（shim 直接 #include app_hover_thrust.c）。"""
    return replay_mod.build_lib(tmp_path_factory.mktemp("hover_app"))


# ---------------------------------------------------------------- 一、算法单元


def _feed(drv, seconds, h_true, thrust_fn, dt=0.004, learn=1, noise=0.0, seed=1, t0=0.0):
    """按 a = g(T/h − 1) + 噪声喂一段合成数据；thrust_fn(t)、h_true 可为函数。"""
    rng = np.random.default_rng(seed)
    n = int(seconds / dt)
    for i in range(n):
        t = t0 + i * dt
        h = h_true(t) if callable(h_true) else h_true
        thr = thrust_fn(t)
        a = G * (thr / h - 1.0) + (rng.normal(0.0, noise) if noise > 0 else 0.0)
        drv.step(dt, thr, a, learn)
    return t0 + n * dt


def _dither(t):
    """围绕 14.3 N 的 ±1.2 N 缓慢抖动，模拟高度环闭环里推力的起伏（激励充分）。"""
    return 14.3 + 1.2 * math.sin(2 * math.pi * 0.6 * t) + 0.5 * math.sin(2 * math.pi * 1.7 * t)


def test_init_validates_and_reports_initial_value(drv):
    assert drv.init(14.25) == 1
    o = drv.out()
    assert o["est"] == pytest.approx(14.25) and o["init"] == pytest.approx(14.25)
    assert o["samples"] == 0 and o["converged"] == 0 and o["std"] == pytest.approx(3.0, rel=1e-3)
    assert drv.init(0.0) == 0 and drv.init(float("nan")) == 0 and drv.init(100.0) == 0
    assert drv.h.hd_params_valid_gate(0.5) == 0 and drv.h.hd_params_valid_gate(3.5) == 1


@pytest.mark.parametrize("init_n", [11.6, 14.25, 17.0])
def test_converges_to_true_hover_thrust_from_any_start(drv, init_n):
    drv.init(init_n)
    _feed(drv, 12.0, 14.25, _dither, noise=0.6)
    o = drv.out()
    assert abs(o["est"] - 14.25) < 0.15, o
    assert o["converged"] == 1 and o["samples"] > 500


def test_convergence_is_fast_enough(drv):
    """初值差 2.65 N，5 s 内进到 ±0.3 N。"""
    drv.init(11.6)
    t = _feed(drv, 5.0, 14.25, _dither, noise=0.6)
    assert abs(drv.out()["est"] - 14.25) < 0.3, t


def test_tracks_slow_battery_style_drift(drv):
    """悬停推力 90 s 里从 14.25 慢漂到 13.5：估计跟得上（滞后 < 0.3 N）。"""
    drv.init(14.25)
    h_true = lambda t: 14.25 - 0.75 * min(t / 90.0, 1.0)  # noqa: E731
    _feed(drv, 90.0, h_true, _dither, noise=0.5)
    assert abs(drv.out()["est"] - 13.5) < 0.3


def test_impact_spikes_are_rejected_and_do_not_move_the_estimate(drv):
    drv.init(14.25)
    _feed(drv, 6.0, 14.25, _dither, noise=0.4)
    before = drv.out()
    for _ in range(3):                              # 撞到槽壁：竖直加速度突然 +30 m/s²
        assert drv.step(0.004, 14.3, 30.0) == RESULT_REJECTED
    after = drv.out()
    assert after["rejected"] == before["rejected"] + 3
    assert after["samples"] == before["samples"]
    assert after["est"] == pytest.approx(before["est"], abs=1e-6)


def test_persistent_disagreement_does_not_lock_the_filter_out(drv):
    """P 已收得很小后基准突然变了（换了负载）：连续拒收触发 P 回抬，估计能追过去。"""
    drv.init(14.25)
    t = _feed(drv, 15.0, 14.25, _dither, noise=0.3)
    _feed(drv, 25.0, 15.75, _dither, noise=0.3, t0=t)
    assert abs(drv.out()["est"] - 15.75) < 0.3


def test_no_update_when_learn_is_zero(drv):
    drv.init(14.25)
    _feed(drv, 2.0, 14.25, _dither, learn=1, noise=0.3)          # 先让滤波器起步、学一点
    before = drv.out()
    _feed(drv, 5.0, 11.0, lambda t: 5.0, learn=0, noise=0.3, t0=2.0)  # 压在槽底：推力 5 N，a≈0
    after = drv.out()
    assert after["est"] == before["est"] and after["std"] == before["std"]
    assert after["samples"] == before["samples"] and after["learning"] == 0


def test_pressed_on_support_would_learn_low_which_is_why_the_gate_exists(drv):
    """反面对照：压在槽底（T=10 N < 重力，a=0）若照学，估计被拉到 ≈10 N。"""
    drv.init(14.25)
    for _ in range(2500):
        drv.step(0.004, 10.0, 0.0, 1)
    assert drv.out()["est"] < 11.5


def test_reset_returns_to_initial_value_and_clears_counters(drv):
    drv.init(14.25)
    _feed(drv, 6.0, 13.0, _dither, noise=0.4)
    assert drv.out()["est"] < 14.0 and drv.out()["samples"] > 0
    drv.h.hd_reset()
    o = drv.out()
    assert o["est"] == pytest.approx(14.25) and o["samples"] == 0 and o["rejected"] == 0
    assert o["converged"] == 0 and o["std"] == pytest.approx(3.0, rel=1e-3)


def test_invalid_inputs_are_ignored(drv):
    drv.init(14.25)
    assert drv.step(0.004, 14.0, 0.0) == RESULT_HELD            # 起步拍
    for args in ((0.0, 14.0, 0.0), (-0.004, 14.0, 0.0), (0.004, float("nan"), 0.0),
                 (0.004, 14.0, float("inf")), (0.004, -1.0, 0.0)):
        assert drv.step(*args) == RESULT_IGNORED, args
    assert drv.step(0.004, 14.0, 0.0, roll=1.6) == RESULT_IGNORED   # 倾角 > 90°
    assert drv.out()["samples"] == 0
    assert drv.step(0.004, 14.0, 0.0, g=0.5) == RESULT_IGNORED


def test_tilt_projection_uses_cos_roll_cos_pitch(drv):
    """T·cosφcosθ 才是竖直分量：倾 30° 且竖直加速度 0 时，h 应学成 T·cos30°。"""
    drv.init(12.0)
    thrust = 16.0
    for _ in range(4000):
        drv.step(0.004, thrust, 0.0, 1, roll=math.radians(30.0))
    assert drv.out()["est"] == pytest.approx(thrust * math.cos(math.radians(30.0)), abs=0.15)


# ---------------------------------------------------------------- 二、何时学（策略层）


def _synthetic_run(seconds=12.0, dt=0.004, thrust=14.3, az=0.0, height=0.6, ground_first=0.45):
    """一段合成的解锁后数据：第一拍在支撑面上，其后一直在 height。"""
    n = int(seconds / dt)
    t = np.arange(n) * dt
    rng = np.random.default_rng(3)
    rng_m = np.full(n, height)
    rng_m[0] = ground_first
    thr = thrust + 1.0 * np.sin(2 * np.pi * 0.6 * t)
    a = G * (thr / 14.25 - 1.0) + rng.normal(0, 0.4, n) if az is None else np.full(n, az)
    return dict(dt=np.full(n, dt), now_ms=(1_000_000 + t * 1000).astype(np.uint32), thrust=thr,
                az=a, range=rng_m, t=t)


def test_learns_only_after_leaving_the_support(app):
    run = _synthetic_run(az=None)
    r = app.replay(run, 14.25)
    assert r["gate"][-1] == 0x3F and r["learning"][-1] == 1
    first = int(np.flatnonzero(r["learning"])[0])
    assert 0.19 <= run["t"][first] <= 0.30                    # 0.2 s 保持后才开始
    assert r["learning"][:first].sum() == 0


def test_never_learns_while_resting_on_the_support(app):
    """推力 10 N、a≈0、测距贴着支撑面：一拍都不该学，估计不动。"""
    run = _synthetic_run(thrust=10.0, az=0.0, height=0.455, ground_first=0.45)
    r = app.replay(run, 14.25)
    assert r["learning"].sum() == 0 and r["samples"][-1] == 0
    assert r["est"][-1] == pytest.approx(14.25)
    assert (r["gate"] & 0x20).max() == 0                        # AIRBORNE 位一直没置


def test_slot_bottom_rise_under_thrust_is_not_mistaken_for_liftoff(app):
    """台架实录里槽底读数在推力作用下上抬 3–4 cm：低于 5 cm 的进入门限。"""
    run = _synthetic_run(thrust=13.5, az=0.0, height=0.45 + 0.038, ground_first=0.45)
    assert app.replay(run, 14.25)["learning"].sum() == 0


@pytest.mark.parametrize("what", ["disarmed", "saturated", "tilt", "no_thrust"])
def test_each_other_condition_blocks_learning(app, what):
    run = _synthetic_run(az=None)
    n = len(run["dt"])
    kwargs = {}
    if what == "disarmed":
        kwargs["armed"] = np.zeros(n)
    elif what == "saturated":
        kwargs["sat"] = np.ones(n)
    elif what == "tilt":
        kwargs["roll_rad"] = math.radians(25.0)               # 倾角 ≥ 20°
    elif what == "no_thrust":
        run["thrust"] = np.full(n, 1.0)                       # 怠速，低于 2 N
    r = app.replay(run, 14.25, **kwargs)
    assert r["learning"].sum() == 0, what
    assert r["est"][-1] == pytest.approx(14.25)


def test_landing_back_on_the_support_stops_learning(app):
    run = _synthetic_run(az=None)
    n = len(run["dt"])
    run["range"][n // 2:] = 0.455                              # 后半段落回支撑面
    r = app.replay(run, 14.25)
    assert r["learning"][: n // 2].sum() > 0
    assert r["learning"][n // 2 + 5:].sum() == 0


def test_ground_reference_only_moves_down_while_armed(app):
    """解锁后支撑面高度只往下追：一直飞在高处不会把'地面'追高，所以一直算离地。"""
    run = _synthetic_run(az=None, seconds=30.0)
    r = app.replay(run, 14.25)
    assert r["learning"][-1] == 1 and r["gate"][-1] == 0x3F


def test_initial_value_comes_from_config_then_from_mass(app):
    run = _synthetic_run(seconds=0.05, az=0.0, height=0.45)
    assert app.replay_lazy(run, hover_cfg_n=14.25)["est"][-1] == pytest.approx(14.25)
    assert app.replay_lazy(run, hover_cfg_n=0.0, mass_kg=1.184)["est"][-1] == pytest.approx(
        1.184 * G, abs=1e-3)
    # 两者都没有（airframe 未就绪）：不初始化，估计恒为 0，也不会崩。
    assert app.replay_lazy(run, hover_cfg_n=0.0, mass_kg=0.0)["est"][-1] == 0.0


def test_initial_value_follows_the_config_until_the_first_sample(app):
    """coax.hover_thrust_n 只在 RAM：上电为 0 → 起步按 m·g，之后才被设成台架值（2026-09-30 首次上板
    init_n=11.235 而配置已是 14.25）。一个样本都没学时初值跟着配置走；学过就不再动。"""
    f32 = lambda x: np.ascontiguousarray(x, dtype=np.float32)  # noqa: E731
    u8 = lambda x: np.ascontiguousarray(x, dtype=np.uint8)  # noqa: E731

    def seg(run, cfg, offset_ms):
        run = dict(run, now_ms=(run["now_ms"] + offset_ms).astype(np.uint32))
        return app._run(run, 0.0, cfg, None, None, f32, u8, len(run["dt"]), 1.184)

    idle = _synthetic_run(seconds=0.2, az=0.0, height=0.45)          # 贴着支撑面：不学
    assert app.replay_lazy(idle, hover_cfg_n=0.0, mass_kg=1.184)["est"][-1] == pytest.approx(
        1.184 * G, abs=1e-3)
    r = seg(idle, 14.25, 1_000)
    assert r["samples"][-1] == 0 and r["est"][-1] == pytest.approx(14.25)
    r = seg(_synthetic_run(az=None), 14.25, 2_000)
    assert r["samples"][-1] > 0
    learned = float(r["est"][-1])
    r = seg(idle, 9.0, 20_000)                                        # 学过之后改配置：不再重起步
    assert r["est"][-1] == pytest.approx(learned, abs=1e-4)


# ---------------------------------------------------------------- 三、实录回放

CLOSED = ("alt_054239_c5ffbf4f", "alt_054311_3b741f37", "alt_055939_c229ace1")
BREAKAWAY = ("alt_041404_103f13c3", "alt_041445_b70dcd2c", "alt_042331_e124357c")


def _need_data(replay_mod, names):
    for n in names:
        if not (replay_mod.DATA / n / "samples.csv").exists():
            pytest.skip(f"实录缺失：{n}")


@pytest.mark.parametrize("name", CLOSED)
def test_replay_closed_loop_runs_converge_to_the_rig_hover_thrust(app, replay_mod, name):
    _need_data(replay_mod, [name])
    run = replay_mod.load_run(name)
    r = app.replay(run, 14.25)
    assert abs(float(r["est"][-1]) - 14.25) < 0.3, (name, r["est"][-1])
    assert r["converged"][-1] == 1
    assert r["std"][-1] < 0.3
    learn = r["learning"].astype(bool)
    assert learn.sum() > 1000
    # 压在槽底（推力低于重力 11.6 N）时基本不学：学习拍里推力<11.6 N 的不到 1%。
    assert np.sum(learn & (run["thrust"] < 11.6)) < 0.01 * learn.sum()


@pytest.mark.parametrize("name", BREAKAWAY)
def test_replay_breakaway_runs_stay_within_tolerance(app, replay_mod, name):
    """离地找阈值的三轮只在离地后几秒里有样本：悬停对应表值 14.25 ± 0.07，摩擦 0.1–0.5 N。"""
    _need_data(replay_mod, [name])
    run = replay_mod.load_run(name)
    r = app.replay(run, 14.25)
    assert abs(float(r["est"][-1]) - 14.25) < 0.3, (name, r["est"][-1])


@pytest.mark.parametrize("init_n", [11.6, 17.0])
@pytest.mark.parametrize("name", CLOSED)
def test_replay_wrong_initial_value_converges_to_the_same_answer(app, replay_mod, name, init_n):
    _need_data(replay_mod, [name])
    run = replay_mod.load_run(name)
    good = app.replay(run, 14.25)
    bad = app.replay(run, init_n)
    assert abs(float(bad["est"][-1]) - 14.25) < 0.3
    assert abs(float(bad["est"][-1]) - float(good["est"][-1])) < 0.05
    t_conv = replay_mod.convergence_time(bad, run["t"], float(good["est"][-1]), tol=0.4)
    assert t_conv is not None and t_conv < 10.0, t_conv


def test_replay_dropping_the_support_gate_would_learn_a_biased_value(app, replay_mod):
    """反面对照：把测距整体抬高使'离地'恒成立，压在槽底的样本进滤波器，估计明显偏低（> 0.4 N）。"""
    _need_data(replay_mod, CLOSED[:1])
    run = replay_mod.load_run(CLOSED[0])
    ok = app.replay(run, 14.25)
    forged = dict(run)
    forged["range"] = run["range"].copy()
    forged["range"][1:] += 0.2
    bad = app.replay(forged, 14.25)
    assert ok["est"][-1] - bad["est"][-1] > 0.4


# ---------------------------------------------------------------- 四、源码契约


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_hover_commands_exist_and_are_dispatched_from_the_fallback_chain():
    cmd = _text(CMD_SRC)
    assert '"HOVER?"' in cmd and '"HOVER"' in cmd and '"RESET"' in cmd
    for field in ("est_n=", "std_n=", "converged=", "learning=", "samples=", "innov=", "init_n="):
        assert field in cmd, field
    assert "app_control_handle_hover(tokens, count)" in _text(FALLBACK)
    assert "app_control_handle_hover" in _text(ROOT / "App/Inc/app_control_internal.h")
    cmake = _text(CMAKE)
    for src in ("App/Src/app_cmd_hover.c", "App/Src/app_hover_thrust.c",
                "Driver/Src/drv_hover_thrust_est.c"):
        assert src in cmake, src


def test_hover_learning_armed_bit_uses_the_real_arm_state():
    """上锁时电机输出也走 flight_branch 那几支（怠速）：只看它会恒判已解锁（2026-09-30 首次上板上锁状态下
    gate 解锁位为 1，支撑面高度不再在上锁时重锁存）。解锁位必须叠加 frame->rc_armed。"""
    src = _text(ROOT / "App/Src/app_stabilizer.c")
    body = src[src.index("static void stabilizer_hover_thrust_step"):]
    body = body[:body.index("APP_HoverThrust_Step(&in);")]
    assert "in.armed = ((frame->rc_armed != 0U) && (flight_branch != 0U)) ? 1U : 0U;" in body
    assert "in.armed = flight_branch;" not in body


def test_firmware_text_output_never_uses_float_printf():
    """固件链的是 newlib-nano，没有浮点 printf：%f/%e/%g 在板上打出来是空的（2026-09-30 首次上板
    HOVER? 各数字段全空，宿主测试用的完整 libc 看不出来）。所有 APP_Control_QueueText 的格式串都不许用。"""
    call = re.compile(r'APP_Control_QueueText\(\s*((?:"(?:[^"\\]|\\.)*"\s*)+)')
    float_spec = re.compile(r"%[-+ #0-9.]*l?[fFeEgGaA]")
    offenders = []
    for folder in ("App/Src", "Driver/Src", "Services/Src"):
        for path in sorted((ROOT / folder).glob("*.c")):
            for m in call.finditer(path.read_text(encoding="utf-8", errors="replace")):
                if float_spec.search(m.group(1).replace("%%", "")):
                    offenders.append(f"{path.name}: {m.group(1)[:60]}")
    assert not offenders, offenders


def test_app_control_c_did_not_grow():
    data = APP_CONTROL.read_bytes()
    lines = data.count(b"\n") + (0 if data.endswith(b"\n") else 1)
    assert lines <= APP_CONTROL_MAX_LINES, lines
    assert len(data) <= APP_CONTROL_MAX_BYTES, len(data)
    assert "hover" not in data.decode("utf-8").lower().replace("hover_thrust_n", "")
    git = shutil.which("git")
    if git:
        head = subprocess.run([git, "show", "HEAD:App/Src/app_control.c"], cwd=ROOT,
                              capture_output=True)
        if head.returncode == 0:
            assert len(data) <= len(head.stdout), (len(data), len(head.stdout))


def test_learning_conditions_include_leaving_the_support_surface():
    app = _text(APP_SRC)
    hdr = _text(APP_HDR)
    assert "hover_update_airborne" in app and "ground_m" in app
    assert "APP_HOVER_GATE_AIRBORNE" in app and "APP_HOVER_GATE_ALL" in app
    for gate in ("ARMED", "THRUST", "UNSAT", "TILT", "ACCEL", "AIRBORNE"):
        assert f"APP_HOVER_GATE_{gate}" in app, gate
    assert "0x3F" in hdr and "APP_HOVER_LIFT_ON_M" in hdr
    assert re.search(r"learn\s*=\s*\(mask == APP_HOVER_GATE_ALL\)", app)


def test_stabilizer_wiring_covers_production_and_auto_throttle_paths():
    src = _text(STABILIZER).replace("\r\n", "\n")
    assert "APP_HoverThrust_Init();" in src and "APP_HoverThrust_Step(&in);" in src
    # 竖直加速度与 z 估计器同源（APP_SysIdAlt_VerticalAccel 那一份）。
    assert "frame->az_up_m_s2 = sysid_obs.az_m_s2;" in src
    helper = src[src.index("static void stabilizer_hover_thrust_step("):]
    helper = helper[: helper.index("static void stabilizer_control_commit(")]
    # 推力 = 提交后从 BSP 回读的上/下桨脉宽（三条路径同一口径），不在电机仲裁分支里埋钩子。
    assert "stabilizer_rotor_pulse_readback((uint8_t)DRV_PROP_ROLE_UPPER, 1U)" in helper
    assert "stabilizer_rotor_pulse_readback((uint8_t)DRV_PROP_ROLE_LOWER, 2U)" in helper
    for reason in ("STABILIZED_MIX", "ATTITUDE_DEBUG", "IDENT_DIRECT", "IMU_INVALID_DIRECT",
                   "DIRECT_THROTTLE"):
        assert f"APP_FLIGHT_LOG_MOTOR_REASON_{reason}" in helper, reason
    # 生产路径：ctrl_out 饱和标志；ALT/SYSID 自动油门：最高油门 % 封顶。
    assert "frame->ctrl_out.thrust_saturated" in helper
    assert "APP_SysId_GetThrottle(&sysid_target_n, &sysid_max_pct);" in helper
    # 每个控制拍在提交之后调用一次，且在 dshot 仲裁 seam 之外（不改 seam 切片的编译环境）。
    commit = src[src.index("static void stabilizer_control_commit("):]
    assert commit.count("stabilizer_hover_thrust_step(ctx, frame);") == 1
    assert commit.index("stabilizer_hover_thrust_step(ctx, frame);") > commit.index(
        "BSP_PWM_CommitEsc()")
    seam = commit[commit.index("  APP_PropSpin_Step(frame->now_ms,"):
                  commit.index("  if (APP_Acceptance_IsActive() != 0U) {\n    APP_AcceptanceObservation")]
    assert "hover" not in seam.lower()


def test_estimate_is_read_only_never_written_back_to_the_controller():
    for path in (DRV_SRC, APP_SRC, CMD_SRC):
        text = _text(path)
        assert "DRV_COAX_CTRL_SetParam" not in text and "hover_thrust_n =" not in text, path
    step = _text(STABILIZER)
    body = step[step.index("static void stabilizer_hover_thrust_step("):]
    body = body[: body.index("static void stabilizer_control_commit(")]
    assert "SetParam" not in body


def test_state_lives_in_axi_sram_not_dtcm():
    assert ".ram_d1_noinit" in _text(APP_SRC)
