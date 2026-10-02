"""竖直通道估计器（Driver/Src/drv_z_estimator.c）—— 宿主 gcc 编真 C。

三组判据：

* **模块行为**：静止收敛、零偏收敛、延迟补偿、新息门限与连续拒收重置、超时无效、Tick 的
  时钟簿记；离散增益确实把样本间隔 T 的误差系统三个极点放在 exp(−T/τ)。
* **实录回放**：2026-09-30 槽式台架 ALT 实录（250 Hz 的原始 TOF、生产 vz、竖直加速度 az），
  逐拍喂给同一份 C 代码，与 TOF 高度的零相位平滑导数比滞后、比 RMS，比槽底静止噪声；
  指标同 data/analysis/sysid-rig-params/2026-09-30/z_estimator_replay.txt（回放脚本就是
  import 本文件的这些函数）。
* **接线契约**：app_stabilizer.c 在 coax.z_vel_fusion < 0.5 时控制器看到的高度/速度/加速度
  仍是原来的测距量，≥ 0.5 才换成估计器输出；ALT 观测只换 height/vz。
"""
from __future__ import annotations

import csv
import ctypes
import math
import os
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Driver/Src/drv_z_estimator.c"
HEADER = ROOT / "Driver/Inc/drv_z_estimator.h"
STABILIZER = ROOT / "App/Src/app_stabilizer.c"
DATA = ROOT / "data/identification/attitude/2026-09-30"

# 机体离地上下运动过的四轮（其余几轮只在槽底蹭）。
MOTION_RUNS = ("alt_041404_103f13c3", "alt_041426_52fb01f2",
               "alt_041445_b70dcd2c", "alt_042331_e124357c")

MEAS_NONE, MEAS_ACCEPTED, MEAS_INIT, MEAS_REJECTED, MEAS_RESET, MEAS_IGNORED = range(6)

SHIM = r"""
#include <stdint.h>
#include "drv_z_estimator.h"

static DRV_ZEst zs;

uint32_t zs_history(void) { return DRV_ZEST_HISTORY; }

void zs_default_params(float *out, uint8_t *reject)
{
    DRV_ZEstParams p;
    DRV_ZEst_DefaultParams(&p);
    out[0] = p.tau_s; out[1] = p.delay_s; out[2] = p.gate_m; out[3] = p.timeout_s;
    *reject = p.reject_reset;
}

static DRV_ZEstParams zs_params(float tau, float delay, float gate, uint8_t reject, float timeout)
{
    DRV_ZEstParams p;
    p.tau_s = tau; p.delay_s = delay; p.gate_m = gate; p.reject_reset = reject;
    p.timeout_s = timeout;
    return p;
}

uint8_t zs_init(float tau, float delay, float gate, uint8_t reject, float timeout)
{
    DRV_ZEstParams p = zs_params(tau, delay, gate, reject, timeout);
    return DRV_ZEst_Init(&zs, &p);
}

uint8_t zs_init_default(void) { return DRV_ZEst_Init(&zs, 0); }

uint8_t zs_set_params(float tau, float delay, float gate, uint8_t reject, float timeout)
{
    DRV_ZEstParams p = zs_params(tau, delay, gate, reject, timeout);
    return DRV_ZEst_SetParams(&zs, &p);
}

void zs_get_params(float *out, uint8_t *reject)
{
    out[0] = zs.params.tau_s; out[1] = zs.params.delay_s; out[2] = zs.params.gate_m;
    out[3] = zs.params.timeout_s; *reject = zs.params.reject_reset;
}

void zs_reset(void) { DRV_ZEst_Reset(&zs); }

void zs_predict(float dt, float a, uint8_t valid) { DRV_ZEst_Predict(&zs, dt, a, valid); }

int32_t zs_correct(float h, float age) { return (int32_t)DRV_ZEst_Correct(&zs, h, age); }

int32_t zs_tick(uint64_t now_us, uint32_t now_ms, float a, uint8_t accel_valid,
                uint8_t range_valid, float range_m, uint32_t sample_ms)
{
    DRV_ZEstTickInput in;
    in.now_us = now_us; in.now_ms = now_ms; in.a_up_m_s2 = a; in.accel_valid = accel_valid;
    in.range_valid = range_valid; in.range_m = range_m; in.range_sample_ms = sample_ms;
    return (int32_t)DRV_ZEst_Tick(&zs, &in);
}

/* out = z, vz, accel, bias, innovation；flags = valid, accel_valid */
void zs_output(float *out, uint8_t *flags)
{
    DRV_ZEstOutput o;
    DRV_ZEst_GetOutput(&zs, &o);
    out[0] = o.z_m; out[1] = o.vz_m_s; out[2] = o.accel_m_s2; out[3] = o.bias_m_s2;
    out[4] = o.innovation_m; flags[0] = o.valid; flags[1] = o.accel_valid;
}

void zs_counts(uint32_t *out)
{
    out[0] = zs.accepted_count; out[1] = zs.rejected_count; out[2] = zs.reset_count;
}

/* 逐拍 Tick 一整段实录，每拍取输出。 */
void zs_replay(uint32_t n, const uint64_t *now_us, const uint32_t *now_ms, const float *a_up,
               const float *range_m, const uint32_t *sample_ms,
               float *z, float *vz, float *bias, float *innov, uint8_t *valid, uint8_t *result)
{
    for (uint32_t i = 0U; i < n; ++i) {
        DRV_ZEstOutput o;
        result[i] = (uint8_t)zs_tick(now_us[i], now_ms[i], a_up[i], 1U, 1U, range_m[i],
                                     sample_ms[i]);
        DRV_ZEst_GetOutput(&zs, &o);
        z[i] = o.z_m; vz[i] = o.vz_m_s; bias[i] = o.bias_m_s2; innov[i] = o.innovation_m;
        valid[i] = o.valid;
    }
}
"""


# ---------------------------------------------------------------- 宿主库


class ZEstLib:
    """ctypes 包装：一个库里只有一个估计器实例（shim 里的静态量）。"""

    def __init__(self, path: Path) -> None:
        h = ctypes.CDLL(str(path))
        f32, u8, u32, i32 = ctypes.c_float, ctypes.c_uint8, ctypes.c_uint32, ctypes.c_int32
        pf, pu8, pu32 = ctypes.POINTER(f32), ctypes.POINTER(u8), ctypes.POINTER(u32)
        pu64 = ctypes.POINTER(ctypes.c_uint64)
        sig = {
            "zs_history": ([], u32),
            "zs_default_params": ([pf, pu8], None),
            "zs_init": ([f32, f32, f32, u8, f32], u8),
            "zs_init_default": ([], u8),
            "zs_set_params": ([f32, f32, f32, u8, f32], u8),
            "zs_get_params": ([pf, pu8], None),
            "zs_reset": ([], None),
            "zs_predict": ([f32, f32, u8], None),
            "zs_correct": ([f32, f32], i32),
            "zs_tick": ([ctypes.c_uint64, u32, f32, u8, u8, f32, u32], i32),
            "zs_output": ([pf, pu8], None),
            "zs_counts": ([pu32], None),
            "zs_replay": ([u32, pu64, pu32, pf, pf, pu32, pf, pf, pf, pf, pu8, pu8], None),
        }
        for name, (args, res) in sig.items():
            fn = getattr(h, name)
            fn.argtypes = args
            fn.restype = res
        self.h = h
        self.history = h.zs_history()

    def default_params(self) -> dict:
        out, rej = (ctypes.c_float * 4)(), ctypes.c_uint8()
        self.h.zs_default_params(out, ctypes.byref(rej))
        return dict(tau=out[0], delay=out[1], gate=out[2], timeout=out[3], reject=rej.value)

    def params(self) -> dict:
        out, rej = (ctypes.c_float * 4)(), ctypes.c_uint8()
        self.h.zs_get_params(out, ctypes.byref(rej))
        return dict(tau=out[0], delay=out[1], gate=out[2], timeout=out[3], reject=rej.value)

    def init(self, **kw) -> int:
        p = {**self.default_params(), **kw}
        return self.h.zs_init(p["tau"], p["delay"], p["gate"], p["reject"], p["timeout"])

    def set_params(self, **kw) -> int:
        p = {**self.params(), **kw}
        return self.h.zs_set_params(p["tau"], p["delay"], p["gate"], p["reject"], p["timeout"])

    def predict(self, dt, a, valid=1):
        self.h.zs_predict(dt, a, valid)

    def correct(self, h, age=0.0) -> int:
        return self.h.zs_correct(h, age)

    def tick(self, now_us, now_ms, a=0.0, accel_valid=1, range_valid=1, range_m=0.0,
             sample_ms=0) -> int:
        return self.h.zs_tick(now_us, now_ms, a, accel_valid, range_valid, range_m, sample_ms)

    def out(self) -> dict:
        o, f = (ctypes.c_float * 5)(), (ctypes.c_uint8 * 2)()
        self.h.zs_output(o, f)
        return dict(z=o[0], vz=o[1], accel=o[2], bias=o[3], innov=o[4], valid=f[0],
                    accel_valid=f[1])

    def counts(self) -> tuple:
        c = (ctypes.c_uint32 * 3)()
        self.h.zs_counts(c)
        return tuple(c)

    def replay(self, run: dict, **params) -> dict:
        """整段实录逐拍 Tick（加速度与测距都当有效）；params 缺省为模块默认值。"""
        assert self.init(**params) == 1
        n = len(run["now_us"])
        out = {k: np.zeros(n, dtype=np.float32) for k in ("z", "vz", "bias", "innov")}
        valid, result = np.zeros(n, dtype=np.uint8), np.zeros(n, dtype=np.uint8)
        ptr = lambda a, t: a.ctypes.data_as(ctypes.POINTER(t))  # noqa: E731
        self.h.zs_replay(n, ptr(run["now_us"], ctypes.c_uint64), ptr(run["now_ms"], ctypes.c_uint32),
                         ptr(run["az32"], ctypes.c_float), ptr(run["hr32"], ctypes.c_float),
                         ptr(run["sample_ms"], ctypes.c_uint32),
                         ptr(out["z"], ctypes.c_float), ptr(out["vz"], ctypes.c_float),
                         ptr(out["bias"], ctypes.c_float), ptr(out["innov"], ctypes.c_float),
                         ptr(valid, ctypes.c_uint8), ptr(result, ctypes.c_uint8))
        out.update(valid=valid, result=result)
        return out


def build_zest(work: Path) -> ZEstLib:
    """Driver/Src/drv_z_estimator.c + 本文件的 shim 编成宿主动态库；告警即失败。"""
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required")
    work.mkdir(parents=True, exist_ok=True)
    shim = work / "zest_shim.c"
    shim.write_text(SHIM, encoding="utf-8")
    out = work / ("zest.dll" if os.name == "nt" else "zest.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O1", "-Wall", "-Wextra", "-Wshadow",
         "-Wdouble-promotion", "-Wconversion", "-Werror", "-I", str(ROOT / "Driver/Inc"),
         str(SOURCE), str(shim), "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return ZEstLib(out)


@pytest.fixture(scope="module")
def zest(tmp_path_factory) -> ZEstLib:
    return build_zest(tmp_path_factory.mktemp("zest"))


# ---------------------------------------------------------------- 实录与指标（回放脚本共用）

GRID_DT = 0.002          # 指标网格 2 ms
REF_CUTOFF_HZ = 5.0      # 参考速度：TOF 高度零相位 2 阶 Butterworth（前后各一遍）的截止
DESPIKE_M = 0.015        # 参考去尖刺门限（槽底 TOF σ 3–8 mm）
LAG_BAND_HZ = (0.2, 2.5) # 滞后主指标的比较频段


def load_run(run: str) -> dict:
    """读一轮 ALT 实录，并按固件 Tick 的口径准备输入：µs/ms 时刻、原始 TOF 按变化去重成样本令牌。"""
    rows = list(csv.DictReader(open(DATA / run / "samples.csv", encoding="utf-8")))
    raw = np.array([int(r["t_us"]) for r in rows], dtype=np.int64)
    step = np.diff(raw) & 0xFFFFFFFF                          # 32 位 µs 计数回绕
    now_us = (1_000_000 + np.concatenate(([0], np.cumsum(step)))).astype(np.uint64)
    col = lambda k: np.array([float(r[k]) for r in rows])  # noqa: E731
    hr = col("height_raw")
    new = np.concatenate(([True], np.diff(hr) != 0.0))       # 值变了 = 新样本（同值连发会并掉一个）
    now_ms = (now_us // 1000).astype(np.uint32)
    token = np.where(new, now_ms, 0).astype(np.uint32)
    token = np.maximum.accumulate(token).astype(np.uint32)   # 没有新样本的拍沿用上一个令牌
    t = (now_us - now_us[0]).astype(np.float64) * 1e-6
    return dict(run=run, t=t, now_us=now_us, now_ms=now_ms, sample_ms=token, new=new,
                hr=hr, hr32=hr.astype(np.float32), height=col("height"), vz=col("vz"),
                az=col("az"), az32=col("az").astype(np.float32), thrust=col("thrust"))


def zero_phase_lowpass(x: np.ndarray, cutoff_hz: float, dt: float) -> np.ndarray:
    from scipy import signal
    b, a = signal.butter(2, cutoff_hz / (0.5 / dt))
    return signal.filtfilt(b, a, x)


def despike(h: np.ndarray, limit_m: float = DESPIKE_M) -> np.ndarray:
    """TOF 样本序列去尖刺：偏离前后各 2 个样本中位数超过 limit 的换成中位数（Hampel 式）。
    实录里有单点跳 +44 mm 又回来的读数（041445 t≈12.8 s，IMU 同期没有对应加速度），
    不去掉的话零相位平滑会把它摊成 ±0.4 m/s 的"参考速度"。"""
    out = h.copy()
    for i in range(2, len(h) - 2):
        med = float(np.median(h[i - 2:i + 3]))
        if abs(h[i] - med) > limit_m:
            out[i] = med
    return out


def reference(run: dict, cutoff_hz: float = REF_CUTOFF_HZ) -> dict:
    """参考：TOF 样本（去重、去尖刺）线性插到 2 ms 网格 → 零相位低通 → 中心差分。非因果、无滞后。"""
    grid = np.arange(run["t"][0], run["t"][-1], GRID_DT)
    h = np.interp(grid, run["t"][run["new"]], despike(run["hr"][run["new"]]))
    hf = zero_phase_lowpass(h, cutoff_hz, GRID_DT)
    return dict(grid=grid, h=hf, v=np.gradient(hf, GRID_DT),
                thrust=np.interp(grid, run["t"], run["thrust"]),
                az=np.interp(grid, run["t"], run["az"]))


def windows(ref: dict) -> dict:
    """运动窗：参考高度离槽底 > 20 mm 的首末时刻各外扩 0.5 s（槽底取推力 ≥ 10 N 段的 20 分位）。
    静止窗：0.5 s（避开起步瞬态）到运动窗开始前 0.3 s——前段推力 < 10 N 抬不动，后段推力
    已过 10 N 但被槽的静摩擦压着，都在槽底、电机在转。"""
    grid, h, thrust = ref["grid"], ref["h"], ref["thrust"]
    engaged = np.where(thrust >= 10.0)[0]
    bottom = np.percentile(h[engaged], 20)
    up = np.where((h > bottom + 0.020) & (thrust >= 10.0))[0]
    pad = int(round(0.5 / GRID_DT))
    motion = slice(max(0, up[0] - pad), min(len(grid), up[-1] + pad))
    rest_from = int(round(0.5 / GRID_DT))
    rest_to = motion.start - int(round(0.3 / GRID_DT))
    low_to = engaged[0]
    return dict(motion=motion, rest=slice(rest_from, rest_to),
                rest_low=slice(rest_from, low_to), rest_high=slice(low_to, rest_to))


def on_grid(run: dict, ref: dict, x: np.ndarray) -> np.ndarray:
    return np.interp(ref["grid"], run["t"], x)


def xcorr_lag(ref: np.ndarray, x: np.ndarray, max_lag_s: float = 0.2) -> tuple:
    """x 相对 ref 的滞后 [s]（正 = x 落后），互相关峰 + 抛物线插值；返回 (滞后, 峰值相关)。"""
    ref = ref - ref.mean()
    x = x - x.mean()
    m = int(round(max_lag_s / GRID_DT))
    cc = []
    for lag in range(-m, m + 1):
        a, b = (ref[:len(ref) - lag], x[lag:]) if lag >= 0 else (ref[-lag:], x[:len(x) + lag])
        cc.append(float(np.dot(a, b) / math.sqrt(np.dot(a, a) * np.dot(b, b))))
    k = int(np.argmax(cc))
    frac = 0.0
    if 0 < k < len(cc) - 1:
        den = cc[k - 1] - 2.0 * cc[k] + cc[k + 1]
        frac = 0.5 * (cc[k - 1] - cc[k + 1]) / den if den != 0.0 else 0.0
    return (k - m + frac) * GRID_DT, cc[k]


def imu_velocity(ref: dict, sl: slice) -> np.ndarray:
    """运动窗里 az 直接积分、去二次趋势：纯 IMU 的速度，只拿来对时（同 z_plant_ident.py 的做法）。"""
    grid, az = ref["grid"][sl], ref["az"][sl]
    v = np.cumsum(az - az.mean()) * GRID_DT
    return v - np.polyval(np.polyfit(grid, v, 2), grid)


def detrend2(ref: dict, sl: slice, x: np.ndarray) -> np.ndarray:
    grid = ref["grid"][sl]
    return x - np.polyval(np.polyfit(grid, x, 2), grid)


def zero_phase_band(x: np.ndarray, lo_hz: float = LAG_BAND_HZ[0],
                    hi_hz: float = LAG_BAND_HZ[1]) -> np.ndarray:
    from scipy import signal
    b, a = signal.butter(2, [lo_hz / (0.5 / GRID_DT), hi_hz / (0.5 / GRID_DT)], "bandpass")
    return signal.filtfilt(b, a, x)


def metrics(run: dict, ref: dict, win: dict, vz: np.ndarray) -> dict:
    """一条速度估计的全部指标。vz 在实录时刻上，内部插到网格。

    lag_band_ms：参考与估计**同样**过 0.2–2.5 Hz 零相位带通后在运动窗里互相关——高度环
      关心的频段（速度环穿越约 3–4 rad/s），同一个零相位滤波不引入相对时移，却能滤掉 TOF 的
      阶梯跳变与落底冲击这些把全频互相关峰拉成双峰的东西。主指标。
    lag_ms：不带通的全频互相关（参考本身 5 Hz 低通），附带峰值相关，供对照。
    lag_imu_ms：对 az 直接积分去趋势的纯 IMU 速度（z_plant_ident.py 得出"约 92 ms"的同一口径）。
      融合估计本身用了 az，这一列对它不是独立证据，只用来和生产 vz 的旧数字对上。
    rms：运动窗内对参考的均方根误差；rest_*：槽底静止窗的标准差与 |v| 的 99 分位。"""
    v = on_grid(run, ref, vz)
    mo, rest = win["motion"], win["rest"]
    lag, corr = xcorr_lag(ref["v"][mo], v[mo])
    lag_band, corr_band = xcorr_lag(zero_phase_band(ref["v"])[mo], zero_phase_band(v)[mo])
    lag_imu, corr_imu = xcorr_lag(imu_velocity(ref, mo), detrend2(ref, mo, v[mo]))
    err = v[mo] - ref["v"][mo]
    return dict(lag_band_ms=lag_band * 1e3, corr_band=corr_band,
                lag_ms=lag * 1e3, corr=corr, lag_imu_ms=lag_imu * 1e3, corr_imu=corr_imu,
                rms=float(np.sqrt(np.mean(err ** 2))),
                rest_std=float(np.std(v[rest])),
                rest_p99=float(np.percentile(np.abs(v[rest]), 99)),
                rest_low_std=float(np.std(v[win["rest_low"]])),
                rest_high_std=float(np.std(v[win["rest_high"]])))


def evaluate(zest: ZEstLib, run_name: str, **params) -> dict:
    run = load_run(run_name)
    ref = reference(run)
    win = windows(ref)
    rep = zest.replay(run, **params)
    new = run["new"]
    innov = rep["innov"][new]
    return dict(run=run, ref=ref, win=win, rep=rep,
                prod=metrics(run, ref, win, run["vz"]),
                fused=metrics(run, ref, win, rep["vz"].astype(np.float64)),
                innov_rms=float(np.sqrt(np.mean(innov[1:] ** 2))),
                innov_max=float(np.max(np.abs(innov[1:]))),
                rejects=int(np.sum(rep["result"] == MEAS_REJECTED)),
                resets=int(np.sum(rep["result"] == MEAS_RESET)),
                always_valid=bool(np.all(rep["valid"] == 1)))


# ---------------------------------------------------------------- 模块行为

WIDE = dict(gate=2.0, timeout=1.0)     # 单元用例里不想让门限/超时插手时用


def gains(tau: float, t: float) -> np.ndarray:
    """头文件里的离散增益（z, vz, 零偏），e = 测量 − 预测。"""
    p = math.exp(-t / tau)
    q = 1.0 - p
    return np.array([1.0 - p ** 3, 1.5 * q * q * (1.0 + p) / t, -(q ** 3) / (t * t)])


def transition(t: float) -> np.ndarray:
    """a_up = 0 时一个样本间隔的预测：a = −b。"""
    return np.array([[1.0, t, -0.5 * t * t], [0.0, 1.0, -t], [0.0, 0.0, 1.0]])


@pytest.mark.parametrize("tau,t", [(0.25, 0.011), (0.25, 0.05), (0.05, 0.05), (1.0, 0.002)])
def test_discrete_gains_put_all_three_error_poles_at_exp_minus_t_over_tau(tau, t):
    """样本间隔 T 的误差系统 A = (I − K·H)·F：特征多项式恰为 (z − p)³，p = exp(−T/τ)。
    T/τ 取到 1（0.05/0.05）也成立——不是小步长近似，所以测距断续、T 忽大忽小都稳。"""
    k = gains(tau, t).reshape(3, 1)
    a = (np.eye(3) - k @ np.array([[1.0, 0.0, 0.0]])) @ transition(t)
    p = math.exp(-t / tau)
    assert np.allclose(np.poly(a), np.poly([p, p, p]), atol=1e-12)


def test_c_update_matches_the_alpha_beta_gamma_model_sample_by_sample(zest):
    """C 实现逐样本对拍上面的线性模型（D = 0、a_up = 0、测量阶跃 0 → 0.1 m）。"""
    tau, t = 0.25, 0.01
    assert zest.init(tau=tau, delay=0.0, **WIDE) == 1
    assert zest.correct(0.0) == MEAS_INIT
    x = np.zeros(3)
    k, f = gains(tau, t), transition(t)
    for _ in range(60):
        zest.predict(t, 0.0)
        assert zest.correct(0.1) == MEAS_ACCEPTED
        x = f @ x
        x = x + k * (0.1 - x[0])
        o = zest.out()
        assert o["z"] == pytest.approx(x[0], abs=2e-6)
        assert o["vz"] == pytest.approx(x[1], abs=2e-5)
        assert o["bias"] == pytest.approx(x[2], abs=2e-4)


def run_ticks(zest, seconds, truth, *, a_bias=0.0, a_noise=0.0, tof_noise=0.0,
              tof_delay=0.0, tof_period=0.011, tick=0.002, seed=1):
    """500 Hz 逐拍 Tick：truth(t) → (h, v, a)。测距每 tof_period 出一个样本，读数是
    tof_delay 以前的真高度。返回每拍 (t, 真 v, 估计输出)。"""
    rng = np.random.default_rng(seed)
    out = []
    last_k = -1
    token = 0
    reading = 0.0
    for i in range(int(round(seconds / tick))):
        t = i * tick
        _h, v, a = truth(t)
        k = int(math.floor((t + 1e-9) / tof_period))
        if k != last_k:
            last_k = k
            token = 1000 + k * int(round(tof_period * 1000))
            reading = truth(k * tof_period - tof_delay)[0] + tof_noise * rng.standard_normal()
        now_us = 1_000_000 + int(round(t * 1e6))
        zest.tick(now_us, now_us // 1000, a + a_bias + a_noise * rng.standard_normal(), 1, 1,
                  reading, token)
        out.append((t, v, zest.out()))
    return out


def test_static_estimate_converges_and_is_quiet_with_tof_and_vibration_noise(zest):
    """静止 0.5 m：TOF σ 5 mm @ 90 Hz、加速度 σ 1 m/s²（槽底电机在转时的实录量级）。"""
    assert zest.init() == 1
    rows = run_ticks(zest, 6.0, lambda t: (0.5, 0.0, 0.0), tof_noise=0.005, a_noise=1.0)
    tail = [o for t, _v, o in rows if t >= 3.0]
    vz = np.array([o["vz"] for o in tail])
    z = np.array([o["z"] for o in tail])
    assert all(o["valid"] for o in tail)
    assert abs(float(np.mean(z)) - 0.5) < 0.002
    # 白噪声加速度 σ 1 m/s² @ 500 Hz 积出来约 0.02 m/s；对 TOF 直接差分是 σ·√2/T ≈ 0.6 m/s
    assert float(np.std(vz)) < 0.03
    assert abs(float(np.mean(vz))) < 0.005


def test_accel_bias_is_learned_and_does_not_leak_into_velocity(zest):
    """a_up 带 +0.3 m/s² 常值零偏、机体不动：零偏被校出来，速度回到 0、高度不漂。"""
    assert zest.init() == 1
    rows = run_ticks(zest, 5.0, lambda t: (0.5, 0.0, 0.0), a_bias=0.3)
    o = rows[-1][2]
    assert o["bias"] == pytest.approx(0.3, abs=0.003)
    assert o["accel"] == pytest.approx(0.0, abs=0.003)
    assert abs(o["vz"]) < 0.002
    assert o["z"] == pytest.approx(0.5, abs=0.001)


def sine(freq=1.5, amp=0.03, base=0.5):
    w = 2.0 * math.pi * freq
    return lambda t: (base + amp * math.sin(w * t), amp * w * math.cos(w * t),
                      -amp * w * w * math.sin(w * t))


def velocity_error(rows, after=2.0):
    err = np.array([o["vz"] - v for t, v, o in rows if t >= after])
    return float(np.sqrt(np.mean(err ** 2)))


def test_delay_compensation_removes_the_tof_lag(zest):
    """TOF 读数晚 30 ms、加速度带零偏：D = 30 ms 时速度误差近乎为 0，D = 0 时明显变大。"""
    assert zest.init(delay=0.030) == 1
    matched = velocity_error(run_ticks(zest, 5.0, sine(), a_bias=0.2, tof_delay=0.030))
    assert zest.init(delay=0.0) == 1
    ignored = velocity_error(run_ticks(zest, 5.0, sine(), a_bias=0.2, tof_delay=0.030))
    assert matched < 0.004
    assert ignored > 4.0 * matched


def test_delay_compensation_equals_filtering_at_the_delayed_horizon(zest):
    """线性系统下"查历史算新息 + 修正沿模型平移历史与当前"与"在延迟时刻跑 D = 0 的滤波，
    再用加速度推到现在"完全等价：a_up = 0 时 vz_now(t) = vz_d(t − D) − b_d(t − D)·D。
    历史若不随修正平移，D 内相继样本会重复修同一份误差，这个等式立刻不成立。"""
    delay, lag = 0.080, 20                 # 4 ms 一拍，20 拍 = 80 ms
    stream = [0.5 + 0.02 * math.sin(0.036 * k) + 0.004 * ((-1) ** k) for k in range(400)]

    def run(d, shift):
        assert zest.init(delay=d, **WIDE) == 1
        vz, bias = [], []
        for i in range(1200):
            t_ms = 1000 + 4 * i
            k = (i - shift) // 3             # 每 3 拍（12 ms）一个样本，整体晚 shift 拍到
            sample = (1000 + 12 * k + 4 * shift) if k >= 0 else 0
            zest.tick(t_ms * 1000, t_ms, 0.0, 1, 1, stream[max(k, 0)], sample)
            o = zest.out()
            vz.append(o["vz"])
            bias.append(o["bias"])
        return np.array(vz), np.array(bias)

    v_now, _ = run(delay, lag)             # 读数晚 D 到（它是被测对象 D 以前的高度）
    v_d, b_d = run(0.0, 0)                 # 读数一到就是当时的高度
    expected = v_d[:-lag] - b_d[:-lag] * delay
    assert np.max(np.abs(v_now[2 * lag:] - expected[lag:])) < 2e-4


def test_sample_age_is_part_of_the_delay(zest):
    """D = 30 ms、样本年龄 0 与 D = 25 ms、年龄 5 ms 查的是同一时刻：输出一致。"""
    def run(delay, age):
        assert zest.init(delay=delay, **WIDE) == 1
        truth = sine()
        rows = []
        for i in range(600):
            zest.predict(0.002, truth(i * 0.002)[2])
            if i % 5 == 0:
                zest.correct(truth(i * 0.002 - 0.03)[0], age)
            rows.append(zest.out()["vz"])
        return np.array(rows)
    assert np.max(np.abs(run(0.030, 0.0) - run(0.025, 0.005))) < 1e-5


def test_innovation_gate_rejects_spikes_and_resets_after_a_persistent_jump(zest):
    assert zest.init(gate=0.04, reject=5) == 1
    assert zest.correct(0.5) == MEAS_INIT
    for _ in range(50):
        zest.predict(0.011, 0.0)
        assert zest.correct(0.5) == MEAS_ACCEPTED
    zest.predict(0.011, 0.0)
    assert zest.correct(0.56) == MEAS_REJECTED           # 单点尖刺：不进估计
    assert zest.out()["z"] == pytest.approx(0.5, abs=1e-5)
    assert zest.out()["innov"] == pytest.approx(0.06, abs=1e-5)
    zest.predict(0.011, 0.0)
    assert zest.correct(0.5) == MEAS_ACCEPTED            # 回来了：连续计数清零
    results = []
    for _ in range(5):                                   # 地形台阶：读数真跳 0.3 m
        zest.predict(0.011, 0.0)
        results.append(zest.correct(0.8))
    assert results == [MEAS_REJECTED] * 4 + [MEAS_RESET]
    o = zest.out()
    assert o["z"] == pytest.approx(0.8, abs=1e-4)        # 高度平移到测量值
    assert abs(o["vz"]) < 1e-4 and abs(o["bias"]) < 1e-4  # 速度、零偏不受牵连
    assert o["valid"] == 1
    zest.predict(0.011, 0.0)
    assert zest.correct(0.8) == MEAS_ACCEPTED            # 历史也跟着平移了，下一个样本照常收
    assert zest.counts() == (53, 6, 1)             # 触发重置的那个也算过门限


def test_missing_range_times_out_and_the_next_sample_restarts(zest):
    assert zest.init(timeout=0.1) == 1
    assert zest.out()["valid"] == 0                      # 起步前无效
    assert zest.correct(0.5) == MEAS_INIT
    for _ in range(30):
        zest.predict(0.002, 0.0)
        zest.correct(0.5)
    for _ in range(45):                                  # 90 ms 没样本，加速度照积
        zest.predict(0.002, 1.0)
    assert zest.out()["valid"] == 1
    for _ in range(10):
        zest.predict(0.002, 1.0)
    assert zest.out()["valid"] == 0                      # 110 ms：无效
    assert zest.correct(0.7, 0.2) == MEAS_IGNORED        # 到手已过期的样本不要
    assert zest.correct(0.7) == MEAS_INIT                # 新样本：不过门限，以它起步
    o = zest.out()
    assert o["valid"] == 1 and o["z"] == pytest.approx(0.7) and o["vz"] == 0.0


def test_tick_clock_bookkeeping(zest):
    assert zest.init() == 1
    assert zest.tick(5_000_000, 5000, 0.0, 1, 1, 0.5, 4999) == MEAS_INIT   # 第一拍只起时钟
    assert zest.tick(5_002_000, 5002, 0.0, 1, 1, 0.5, 4999) == MEAS_NONE   # 同一个样本不重复用
    assert zest.tick(5_004_000, 5004, 0.0, 1, 0, 0.5, 5003) == MEAS_NONE   # 服务报无效
    assert zest.tick(5_006_000, 5006, 0.0, 1, 1, 0.5, 0) == MEAS_NONE      # 没有样本
    # 样本时间戳比 now_ms 还新（两次取时刻之间刚到）：按年龄 0 收，不当过期丢
    assert zest.tick(5_008_000, 5008, 0.0, 1, 1, 0.5, 5009) == MEAS_ACCEPTED
    zest.tick(5_010_000, 5010, 2.0, 0, 1, 0.5, 5009)                       # 加速度无效：匀速外推
    o = zest.out()
    assert (o["accel_valid"], o["accel"]) == (0, 0.0)
    before = o["z"]
    zest.tick(5_009_000, 5011, 3.0, 1, 1, 0.5, 5009)                       # µs 倒退：本拍不推进
    assert zest.out()["z"] == before and zest.out()["accel"] == pytest.approx(3.0)


def test_parameter_validation(zest):
    defaults = zest.default_params()
    for bad in (dict(tau=0.01), dict(tau=float("nan")), dict(delay=-0.001), dict(delay=0.2),
                dict(gate=0.0), dict(reject=0), dict(timeout=5.0)):
        assert zest.init(**bad) == 0, bad
        assert zest.params() == defaults                 # 非法：整组换回默认
    assert zest.init(tau=0.5) == 1
    assert zest.set_params(delay=0.5) == 0
    assert zest.params()["delay"] == pytest.approx(defaults["delay"])
    assert zest.set_params(delay=0.05) == 1
    header = HEADER.read_text(encoding="utf-8")
    delay_max = float(re.search(r"DRV_ZEST_DELAY_MAX_S\s+([0-9.]+)f", header).group(1))
    # 历史覆盖：500 Hz 控制拍下 HISTORY 条要装得下最大延迟 + 20 ms 样本年龄
    assert zest.history * 0.002 >= delay_max + 0.020


def test_defaults_are_the_ones_archived_by_the_replay(zest):
    archive = (ROOT / "data/analysis/sysid-rig-params/2026-09-30/z_estimator_replay.txt"
               ).read_text(encoding="utf-8")
    line = re.search(r"默认参数：(.*)", archive).group(1)
    got = dict(re.findall(r"(\w+)=([0-9.]+)", line))
    d = zest.default_params()
    assert float(got["tau_s"]) == pytest.approx(d["tau"])
    assert float(got["delay_s"]) == pytest.approx(d["delay"])
    assert float(got["gate_m"]) == pytest.approx(d["gate"])
    assert int(got["reject_reset"]) == d["reject"]
    assert float(got["timeout_s"]) == pytest.approx(d["timeout"])


# ---------------------------------------------------------------- 实录回放（默认参数）

CLEAN_RUNS = ("alt_041426_52fb01f2", "alt_042331_e124357c")


@pytest.mark.parametrize("run_name", CLEAN_RUNS)
def test_replay_cuts_lag_and_rest_noise_against_production(zest, run_name):
    """两轮离地最大、IMU 与 TOF 对得上的实录：滞后 ≤ 30 ms 且比生产 vz 少 30 ms 以上；
    槽底静止噪声（标准差与 99 分位）不到生产的一半；运动窗 RMS 不比生产差。"""
    pytest.importorskip("scipy")
    r = evaluate(zest, run_name)
    prod, fused = r["prod"], r["fused"]
    assert prod["lag_band_ms"] > 55.0                    # 生产 vz 的已知滞后（对 TOF 参考）
    assert fused["lag_band_ms"] <= 30.0
    assert fused["lag_band_ms"] <= prod["lag_band_ms"] - 30.0
    assert fused["corr_band"] > 0.9
    assert fused["rest_std"] < 0.5 * prod["rest_std"]
    assert fused["rest_p99"] < 0.5 * prod["rest_p99"]
    assert fused["rms"] < 1.05 * prod["rms"]
    assert r["always_valid"] and r["rejects"] == 0 and r["resets"] == 0


def test_replay_rejects_the_tof_spike_in_041445(zest):
    """041445 t≈12.78 s：TOF 连着三个读数跳高 42–75 mm 又回来，IMU 同期没有对应加速度。
    默认门限拒掉恰好这三个；生产 vz 被它甩到 +0.56 m/s，融合估计不受牵连。"""
    pytest.importorskip("scipy")
    r = evaluate(zest, "alt_041445_b70dcd2c")
    rejected = r["run"]["t"][r["rep"]["result"] == MEAS_REJECTED]
    assert len(rejected) == 3 and np.all((rejected > 12.77) & (rejected < 12.80))
    assert r["resets"] == 0 and r["always_valid"]
    assert r["fused"]["rms"] < 0.8 * r["prod"]["rms"]
    assert r["fused"]["rest_std"] < 0.5 * r["prod"]["rest_std"]


# ---------------------------------------------------------------- 接线契约（app_stabilizer.c）


def strip_c_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", " ", source, flags=re.S)
    return re.sub(r"//[^\n]*", " ", source)


def squash(source: str) -> str:
    return re.sub(r"\s+", " ", source)


@pytest.fixture(scope="module")
def stabilizer() -> str:
    return strip_c_comments(STABILIZER.read_text(encoding="utf-8"))


def function_body(source: str, name: str) -> str:
    start = source.index(f"static void {name}(")
    brace = source.index("{", source.index(")", start))
    depth = 0
    for i in range(brace, len(source)):
        depth += {"{": 1, "}": -1}.get(source[i], 0)
        if depth == 0:
            return source[brace:i + 1]
    raise AssertionError(name)


def test_estimator_runs_every_tick_from_the_same_accel_and_the_raw_range(stabilizer):
    step = squash(function_body(stabilizer, "stabilizer_z_estimator_step"))
    # 测距：服务里未平滑的原始值及其到达时刻（与 now_ms 同一个 ms 钟），不是 EMA 后的 height_m
    for text in ("in.now_us = frame->now_us;", "in.now_ms = frame->now_ms;",
                 "in.a_up_m_s2 = a_up_m_s2;", "in.accel_valid = frame->imu_control_valid;",
                 "in.range_valid = nav->height_valid;", "in.range_m = nav->height_raw_m;",
                 "in.range_sample_ms = nav->height_sample_ms;",
                 "(void)DRV_ZEst_Tick(&stabilizer_z_est, &in);",
                 "DRV_ZEst_GetOutput(&stabilizer_z_est, &frame->z_est);",
                 '(void)DRV_COAX_CTRL_GetParam("coax.z_vel_fusion", &z_vel_fusion);',
                 "frame->z_fusion = ((z_vel_fusion >= 0.5f) && (frame->z_est.valid != 0U)) ? 1U : 0U;"):
        assert text in step, text
    prepare = squash(function_body(stabilizer, "stabilizer_control_prepare"))
    # 每拍无条件调用（紧跟 ALT 观测的初始化之后），在 IMU 有效判定之后、辨识 Update 之前
    call = prepare.index("}; stabilizer_z_estimator_step(frame, &sysid_nav, sysid_obs.az_m_s2);")
    assert prepare.index("frame->imu_control_valid = 1U;") < call
    assert prepare.index("SVC_FlowNav_GetState(&sysid_nav);") < call
    assert prepare.index("APP_SysIdObserve sysid_obs = {") < call < prepare.index(
        "APP_SysId_Update(&sysid_obs);")
    # 加速度与 ALT 观测是同一个值：全文件只算一次 a_up
    assert squash(stabilizer).count("APP_SysIdAlt_VerticalAccel(") == 1
    assert ".az_m_s2 = APP_SysIdAlt_VerticalAccel(" in prepare
    assert squash(stabilizer).count("stabilizer_z_estimator_step(") == 2    # 定义 + 唯一调用
    assert "(void)DRV_ZEst_Init(&stabilizer_z_est, NULL);" in stabilizer
    assert "memset(&frame, 0, sizeof(frame));" in stabilizer                 # z_fusion 每拍默认 0


def enclosing_conditions(source: str, pos: int) -> list:
    """pos 往外逐层找包住它的 `{`，返回每层 `{` 前面那段条件文本（squash 过）。"""
    out, depth = [], 0
    for i in range(pos, -1, -1):
        c = source[i]
        if c == "}":
            depth += 1
        elif c == "{":
            if depth == 0:
                out.append(squash(source[max(0, i - 120):i]).rstrip())
            else:
                depth -= 1
    return out


def test_switch_off_keeps_every_controller_input_on_the_original_range_path(stabilizer):
    """改动前的每一句都原样留着；估计器输出只在 `if (frame->z_fusion != 0U)` 块里覆盖上去——
    开关为 0 时这些块不执行，控制器与 ALT 观测看到的量逐位不变。"""
    flat = squash(stabilizer)
    for text in (
        "frame->relative_height_m = frame->range_height_m - ctx->height_origin_m;",
        "frame->attitude.z_m = frame->relative_height_m;",
        "frame->attitude.vz_m_s = frame->range_velocity_m_s;",
        "frame->attitude.accel_m_s2[2] = ctx->vertical_accel_m_s2;",
        "frame->attitude.acceleration_valid = ((nav_state.velocity_valid != 0U) && "
        "(ctx->vertical_accel_ready != 0U)) ? 1U : 0U;",
        ".height_m = sysid_height_m,", ".vz_m_s = sysid_vz_m_s,",
        ".height_raw_m = sysid_nav.height_raw_m,", ".height_sample_ms = sysid_height_ms,",
        "ctx->height_origin_m = frame->range_height_m;",       # 原点逻辑不变
    ):
        assert text in flat, text
    # 开关为 1 时的覆盖，逐句钉住，并且排在原句之后
    overrides = (
        ("frame->relative_height_m = frame->range_height_m - ctx->height_origin_m;",
         "if (frame->z_fusion != 0U) { frame->relative_height_m = frame->z_est.z_m - "
         "ctx->height_origin_m; } if (frame->relative_height_m < 0.0f) {"),
        ("(ctx->vertical_accel_ready != 0U)) ? 1U : 0U;",
         "if (frame->z_fusion != 0U) { if (frame->range_height_valid != 0U) { "
         "frame->attitude.vz_m_s = frame->z_est.vz_m_s; } "
         "frame->attitude.accel_m_s2[2] = frame->z_est.accel_m_s2; "
         "frame->attitude.acceleration_valid = ((nav_state.velocity_valid != 0U) && "
         "(frame->z_est.accel_valid != 0U)) ? 1U : 0U; }"),
        ("stabilizer_z_estimator_step(frame, &sysid_nav, sysid_obs.az_m_s2);",
         "if (frame->z_fusion != 0U) { sysid_obs.height_m = frame->z_est.z_m; "
         "sysid_obs.vz_m_s = frame->z_est.vz_m_s; } APP_SysId_Update(&sysid_obs);"),
    )
    for before, text in overrides:
        assert text in flat, text
        assert flat.index(before) < flat.index(text)
    assert flat.index("frame->attitude.vz_m_s = frame->range_velocity_m_s;") < flat.index(
        "frame->attitude.vz_m_s = frame->z_est.vz_m_s;")
    # ALT 的原始测距与样本时刻不被覆盖
    assert not re.search(r"sysid_obs\.(height_raw_m|height_sample_ms)\s*=", stabilizer)
    # 估计器输出（除算开关本身的那一句）只在开关块里被用到
    for m in re.finditer(r"frame->z_est\.", stabilizer):
        line = stabilizer[stabilizer.rfind(chr(10), 0, m.start()) + 1:stabilizer.find(chr(10), m.start())]
        if "frame->z_fusion = " in line:
            continue
        conds = enclosing_conditions(stabilizer, m.start())
        assert any(c.endswith("if (frame->z_fusion != 0U)") for c in conds[:3]), line.strip()
