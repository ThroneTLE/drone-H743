"""横滚/俯仰指令整形（参考模型 + 前馈）与速率环力矩出口陷波 —— 宿主上跑真实 C。

设计来源：data/analysis/sysid-rig-params/2026-09-28/scripts/attitude_design_compare.py
（notch_biquad、sim）与 attitude_filter_search.py。本文件里的 Python 参考实现逐字抄自
那里，只把依赖 scipy / 拟合文件的部分换成 numpy，逐项注明出处；对拍的是**同一公式**，
不是另写一份"差不多"的控制律。

判据分五组，每组对应一件上机才会出事的事：

* **算法本身**（drv_moment_notch.h / drv_att_reference.h）：陷波系数与设计脚本一致、
  f0 处 −20 dB、1 Hz 处增益/相位与连续原型一致；参考模型阶跃逐点等于设计脚本；
  延后按时间插值；关着时逐位直通。
* **关着时逐位不变**：两个开关为 0 时，控制器输出（整个 Output 与 Debug 结构的字节）
  与把整形接线从源码里剥掉的那一版完全相同。
* **开着时接对了**：α_ff 横滚/俯仰非零且只进前馈、ω_sp 用延后参考；陷波只滤反馈、
  直接姿态模式每拍清回路状态时仍然生效；复位/来源切换对齐当前姿态不跳变，
  而每拍的 integrator_reset **不许**把参考对齐掉（否则角度环永远没有误差）。
* **闭环**：真实 C 控制器 + 辨识对象（悬停名义与最坏角点）跑 3° 阶跃，
  90% 时间 < 0.7 s、超调 < 8%，并与设计脚本同构的 Python 仿真对拍。
* **同一条路**：光杆 ANGLE/RATE 验证轮用的是同一段实现；参数范围、持久化（v24 块、
  v23 旧记录落回"关"）与上位机能力表的镜像。

第二级出口陷波（coax.rate_out_notch2_*，2026-09-28 晚，压舵机高阶动态/结构那段约 15 Hz 的回路）
串在第一级之后、滤同一个反馈信号，另有一组判据：关着时与"只有第一级"的实现逐位相同（算法层对拍
改动前单级函数的逐字副本，控制器层对拍剥掉第二级接线的同一份源码）；开着时 f0 处反馈约 −20 dB、
前馈不过它；参数范围同第一级；v25 块往返、v24 旧记录第二级落回"关"；光杆辨识同样两级串联，
开跑溯源行带 onotch2 两项且最宽写法放得进文本缓冲。
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import re
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE, PROP_MAP_SOURCE

ROOT = Path(__file__).resolve().parents[1]
DRIVER_INC = ROOT / "Driver" / "Inc"
DRIVER_SRC = ROOT / "Driver" / "Src"
CTRL_SOURCE = DRIVER_SRC / "drv_coax_ctrl.c"
FIT_JSON = (ROOT / "data" / "identification" / "attitude" / "2026-09-28" /
            "rod_004508_ce57f2e5" / "fit_joint.json")

CTRL_DEPS = [
    AIRFRAME_SOURCE,
    PROP_MAP_SOURCE,
    DRIVER_SRC / "drv_position_control.c",
    DRIVER_SRC / "drv_attitude_control.c",
    DRIVER_SRC / "drv_rate_control.c",
]

BSP_PWM_STUB = (
    "#ifndef BSP_PWM_H\n#define BSP_PWM_H\n"
    "#define BSP_PWM_ESC_MIN_US 1000U\n#define BSP_PWM_ESC_MAX_US 2000U\n#endif\n"
)


def _gcc() -> str:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is required")
    return gcc


def _build(work: Path, name: str, harness: str, sources: list[Path],
           includes: list[Path], *, werror: bool = True, defines: tuple[str, ...] = ()) -> Path:
    work.mkdir(parents=True, exist_ok=True)
    stub = work / "stub"
    stub.mkdir(exist_ok=True)
    (stub / "bsp_pwm.h").write_text(BSP_PWM_STUB, encoding="ascii")
    c_file = work / f"{name}.c"
    c_file.write_text(harness, encoding="utf-8")
    exe = work / f"{name}.exe"
    flags = ["-std=c11", "-O1", "-Wall", "-Wextra"] + (["-Werror"] if werror else [])
    flags += [f"-D{d}" for d in defines]
    cmd = [_gcc(), *flags, f"-I{stub}", *[f"-I{p}" for p in includes],
           *[str(s) for s in sources], str(c_file), "-lm", "-o", str(exe)]
    built = subprocess.run(cmd, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    return exe


def _run(exe: Path, *args: object) -> list[str]:
    result = subprocess.run([str(exe), *[str(a) for a in args]],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout.splitlines()


def _rows(lines: list[str]) -> np.ndarray:
    return np.array([[float(v) for v in line.split()] for line in lines])


# --------------------------------------------------------------------------- Python 参考


def notch_biquad(f0, q, depth=0.1, fs_hz=1000.0):
    """逐字抄自 attitude_design_compare.notch_biquad（同一连续原型，双线性 + 预畸变）。"""
    w0 = 2 * math.pi * f0
    k = w0 / math.tan(w0 / (2 * fs_hz))
    b = [1.0, depth * w0 / q, w0 * w0]
    a = [1.0, w0 / q, w0 * w0]
    bz = [b[0] * k * k + b[1] * k + b[2], 2 * (b[2] - b[0] * k * k), b[0] * k * k - b[1] * k + b[2]]
    az = [a[0] * k * k + a[1] * k + a[2], 2 * (a[2] - a[0] * k * k), a[0] * k * k - a[1] * k + a[2]]
    return np.array(bz) / az[0], np.array(az) / az[0]


def notch_continuous(f_hz, f0, q, depth=0.1):
    """连续原型 N(jω)。"""
    s = 2j * math.pi * f_hz
    w0 = 2 * math.pi * f0
    return (s * s + depth * w0 / q * s + w0 * w0) / (s * s + w0 / q * s + w0 * w0)


def df1_primed(b, a, x):
    """DF1，首个样本按直流稳态起步（与 drv_moment_notch.h 的"未就绪"语义一致）。"""
    y = np.empty(len(x))
    x1 = x2 = y1 = y2 = None
    for k, xk in enumerate(x):
        if x1 is None:
            yk = xk
            x1 = x2 = y1 = y2 = xk
        else:
            yk = b[0] * xk + b[1] * x1 + b[2] * x2 - a[1] * y1 - a[2] * y2
            x2, x1, y2, y1 = x1, xk, y1, yk
        y[k] = yk
    return y


def reference_steps(wr, target, dt, n, start=0.0):
    """attitude_design_compare.sim 的参考部分：rdd 用推进前状态，rd、r 依次半隐式推进。"""
    r, rd = start, 0.0
    out = np.empty((n, 3))
    for k in range(n):
        rdd = wr ** 2 * (target - r) - 2 * wr * rd
        rd += rdd * dt
        r += rd * dt
        out[k] = (r, rd, rdd)
    return out


def delayed(samples, times, base, t_query):
    """按时间线性插值取 t_query 时刻的值；早于第一条时，时间线在"对齐点"（第一条之前一拍、
    值为 base、速度 0）之前恒为 base——drv_att_reference.h 的对齐语义。"""
    t = np.concatenate(([times[0] - (times[1] - times[0])], times))
    v = np.concatenate(([base], samples))
    if t_query <= t[0]:
        return base
    return float(np.interp(t_query, t, v))


# ------------------------------------------------------------------ 飞行对象（设计脚本同构）


def plant_models():
    """NOM 来自 fit_joint.json（与 attitude_optimize.NOM 同源）；HOVER 与最坏角点与
    attitude_design_compare 一致：ρ×7.4/11.235；最坏角点 = worst_hover() 在当前增益下
    搜出的 ρ×1.2、T+5 ms、κ×0.9、ωs×1.15（2026-09-28 实跑脚本得到）。"""
    fit = json.loads(FIT_JSON.read_text(encoding="utf-8"))
    nom = dict(I=fit["inertia_kg_m2"], kappa=fit["torque_model_scale"], T=fit["dead_time_s"],
               wn=fit["servo_wn_rad_s"], zeta=fit["servo_zeta"], rho=fit["reaction_couple_s2"])
    hover = dict(nom, rho=nom["rho"] * 7.4 / 11.235)
    worst = dict(hover, rho=hover["rho"] * 1.2, T=nom["T"] + 0.005,
                 kappa=nom["kappa"] * 0.9, wn=nom["wn"] * 1.15)
    return nom, hover, worst


def _expm(m):
    """numpy 版矩阵指数（缩放平方 + Taylor），替代 scipy.linalg.expm。"""
    norm = np.linalg.norm(m, 1)
    squarings = max(0, int(math.ceil(math.log2(norm))) + 1) if norm > 0.5 else 0
    a = m / (2 ** squarings)
    term = np.eye(m.shape[0])
    total = term.copy()
    for k in range(1, 30):
        term = term @ a / k
        total = total + term
    for _ in range(squarings):
        total = total @ total
    return total


def discrete_plant(m, dt=0.001):
    """attitude_optimize.discrete_plant 的 numpy 版：状态 [舵机, 舵机速率, θ, ω]。"""
    w2 = m["wn"] ** 2
    a = np.array([[0, 1, 0, 0], [-w2, -2 * m["zeta"] * m["wn"], 0, 0], [0, 0, 0, 1],
                  [(m["kappa"] - m["rho"] * w2) / m["I"], -2 * m["rho"] * m["zeta"] * m["wn"] / m["I"], 0, 0]])
    b = np.array([0, w2, 0, m["rho"] * w2 / m["I"]])
    aug = np.zeros((5, 5))
    aug[:4, :4] = a * dt
    aug[:4, 4] = b * dt
    e = _expm(aug)
    return e[:4, :4], e[:4, 4]


def design_sim(m, g, *, notch=None, ref_wr=None, ref_delay=0.0, target_deg=3.0, seconds=3.0,
               ff_inertia=None):
    """attitude_design_compare.sim（dist = 0）的 numpy 版，逐行同构。"""
    dt = 0.001
    phi, gam = discrete_plant(m, dt)
    x = np.zeros(4)
    queue = [0.0] * int(round(m["T"] / dt))
    integ = 0.0
    lag = [(0.0, 0.0)] * int(round(ref_delay / dt))
    filt = notch_biquad(*notch) if notch else None
    zx = [0.0, 0.0]
    zy = [0.0, 0.0]
    tgt = math.radians(target_deg)
    r = rd = 0.0
    n = int(seconds / dt)
    th = np.empty(n)
    if ff_inertia is None:
        nom = plant_models()[0]
        ff_i = nom["I"] / nom["kappa"]   # 脚本：u += NOM["I"]·rdd/NOM["kappa"]（名义对象，不随角点变）
    else:
        ff_i = ff_inertia
    for k in range(n):
        if ref_wr is None:
            r, rd, rdd = tgt, 0.0, 0.0
        else:
            rdd = ref_wr ** 2 * (tgt - r) - 2 * ref_wr * rd
            rd += rdd * dt
            r += rd * dt
        lag.append((r, rd))
        rf, rdf = lag.pop(0) if ref_delay else (r, rd)
        err = g["att"] * (rf - x[2]) + rdf - x[3]
        if abs(err) <= 1.5:
            integ = min(max(integ + g["ki"] * err * dt, -0.05), 0.05)
        u = g["kp"] * err + integ
        if filt is not None:
            b, a = filt
            y = b[0] * u + b[1] * zx[0] + b[2] * zx[1] - a[1] * zy[0] - a[2] * zy[1]
            zx = [u, zx[0]]
            zy = [y, zy[0]]
            u = y
        u += ff_i * rdd
        queue.append(u)
        v = queue.pop(0)
        x = phi @ x + gam * v
        th[k] = x[2]
    return th


def step_metrics(th, target_deg=3.0, dt=0.001):
    """attitude_optimize.step_metrics 逐字。"""
    target = math.radians(target_deg)
    t = np.arange(th.size) * dt
    hit = np.nonzero(th >= 0.9 * target)[0]
    outside = np.nonzero(np.abs(th - target) > 0.02 * target)[0]
    return dict(t90=float(t[hit[0]]) if hit.size else float("nan"),
                t2=float(t[outside[-1] + 1]) if outside.size and outside[-1] + 1 < th.size else float("nan"),
                over=max(0.0, float((th.max() - target) / target * 100)))


# =============================================================================== 算法模块


DRIVER_HARNESS = r"""
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "drv_att_reference.h"
#include "drv_moment_notch.h"

#define TWO_PI 6.28318530717958647692

static void coef(float f0, float q, float dt)
{
    DRV_MomentNotchCoef c;
    int ok;
    memset(&c, 0, sizeof(c));
    ok = DRV_MomentNotch_Design(&c, f0, q, DRV_MOMENT_NOTCH_DEPTH, dt);
    printf("%d %.9g %.9g %.9g %.9g %.9g\n", ok, c.b0, c.b1, c.b2, c.a1, c.a2);
}

/* 一路正弦过 Configure/Apply/Commit，dt 可在两个值之间交替（模拟调度抖动）。 */
static void sine(float f0, float q, float dt_a, float dt_b, float freq, int n)
{
    DRV_MomentNotch nt;
    double t = 0.0;
    DRV_MomentNotch_Reset(&nt);
    for (int k = 0; k < n; ++k) {
        const float dt = (k % 2 == 0) ? dt_a : dt_b;
        const float x = (float)sin(TWO_PI * freq * t);
        float y;
        (void)DRV_MomentNotch_Configure(&nt, f0, q, dt);
        y = DRV_MomentNotch_Apply(&nt, 1U, x);
        /* 同拍重算一次（飞控保护缩放的路径）：不许推进历史。 */
        y = DRV_MomentNotch_Apply(&nt, 1U, x);
        DRV_MomentNotch_Commit(&nt);
        printf("%.9g %.9g %.9g\n", t, x, y);
        t += dt;
    }
}

static int same_bits(float a, float b) { return memcmp(&a, &b, sizeof(a)) == 0; }

/* 关、参数非法、f0 过高：逐位直通；未生效时 ApplyToRateOutput 一个字节都不动。 */
static void passthrough(void)
{
    static const float xs[] = { 0.0f, -0.0f, 1.5f, -3.25e-7f, 12345.678f, 1.0e-30f };
    DRV_MomentNotch nt;
    DRV_RateControl_Input in;
    DRV_RateControl_Output out, copy;
    int bad = 0;

    DRV_MomentNotch_Reset(&nt);
    if (DRV_MomentNotch_Configure(&nt, 0.0f, 1.2f, 0.002f) != 0U) bad |= 1;
    for (unsigned i = 0; i < sizeof(xs) / sizeof(xs[0]); ++i) {
        const float y = DRV_MomentNotch_Apply(&nt, 0U, xs[i]);
        DRV_MomentNotch_Commit(&nt);
        if (!same_bits(y, xs[i])) bad |= 2;
    }
    if (DRV_MomentNotch_Configure(&nt, 100.0f, 1.2f, 0.0045f) != 0U) bad |= 4;  /* f0·dt = 0.45 */
    if (DRV_MomentNotch_Configure(&nt, 6.0f, 0.1f, 0.002f) != 0U) bad |= 8;     /* Q 过小 */
    if (DRV_MomentNotch_Configure(&nt, 6.0f, 1.2f, 0.2f) != 0U) bad |= 16;      /* dt 超界 */
    if (DRV_MomentNotch_Configure(&nt, -6.0f, 1.2f, 0.002f) != 0U) bad |= 32;
    if (DRV_MomentNotch_Configure(&nt, NAN, 1.2f, 0.002f) != 0U) bad |= 64;
    memset(&in, 0x5A, sizeof(in));
    memset(&out, 0xA5, sizeof(out));
    copy = out;
    DRV_MomentNotch_ApplyToRateOutput(&nt, &in, &out);
    if (memcmp(&out, &copy, sizeof(out)) != 0) bad |= 128;
    printf("%d\n", bad);
}

/* 只滤反馈：p/i/d 走陷波，ff 与其余项原样叠回；然后按限幅重钳、重算饱和标志。 */
static void feedback_only(float dt, int n)
{
    DRV_MomentNotch nt;
    DRV_RateControl_Input in;
    DRV_RateControl_Output out;
    double t = 0.0;

    DRV_MomentNotch_Reset(&nt);
    memset(&in, 0, sizeof(in));
    for (unsigned a = 0; a < 3U; ++a) {
        in.saturation_positive[a] = 0.12f;
        in.saturation_negative[a] = -0.10f;
    }
    for (int k = 0; k < n; ++k) {
        float fb;
        memset(&out, 0, sizeof(out));
        out.p_term[1] = (float)(0.10 * sin(TWO_PI * 6.0 * t));
        out.i_term[1] = 0.02f;
        out.d_term[1] = (float)(0.01 * sin(TWO_PI * 3.0 * t));
        out.ff_term[1] = (float)(0.05 * sin(TWO_PI * 6.0 * t));
        fb = out.p_term[1] + out.i_term[1] - out.d_term[1];
        out.moment_unsat[1] = fb + out.ff_term[1] + 0.003f;   /* 0.003 = 陀螺耦合项 */
        out.moment_cmd[1] = out.moment_unsat[1];
        (void)DRV_MomentNotch_Configure(&nt, 6.0f, 1.2f, dt);
        DRV_MomentNotch_ApplyToRateOutput(&nt, &in, &out);
        DRV_MomentNotch_Commit(&nt);
        printf("%.9g %.9g %.9g %.9g %d %d\n", fb, out.ff_term[1] + 0.003f,
               out.moment_unsat[1], out.moment_cmd[1],
               out.saturated_pos[1], out.saturated_neg[1]);
        t += dt;
    }
}

/*
 * 第二级陷波加入之前的单级实现：逐字抄自改动前的 drv_moment_notch.h
 * DRV_MomentNotch_ApplyToRateOutput（只改了函数名）。用它钉住"第二级关着 = 以前的行为"。
 */
static void pre_notch2_apply_to_rate_output(DRV_MomentNotch *n,
                                            const DRV_RateControl_Input *input,
                                            DRV_RateControl_Output *output)
{
    if ((n == NULL) || (input == NULL) || (output == NULL) || (n->active == 0U)) {
        return;
    }
    for (uint32_t axis = 0U; axis < DRV_MOMENT_NOTCH_AXES; ++axis) {
        const float feedback = output->p_term[axis] + output->i_term[axis] -
                               output->d_term[axis];
        const float bypass = output->moment_unsat[axis] - feedback;
        float positive = input->saturation_positive[axis];
        float negative = input->saturation_negative[axis];

        output->moment_unsat[axis] = DRV_MomentNotch_Apply(n, axis, feedback) + bypass;
        output->moment_cmd[axis] = output->moment_unsat[axis];
        output->saturated_pos[axis] = 0U;
        output->saturated_neg[axis] = 0U;
        if (!isfinite(positive) || !isfinite(negative) || (positive <= negative)) {
            continue;
        }
        if (output->moment_cmd[axis] >= positive) {
            output->moment_cmd[axis] = positive;
            output->saturated_pos[axis] = 1U;
        } else if (output->moment_cmd[axis] <= negative) {
            output->moment_cmd[axis] = negative;
            output->saturated_neg[axis] = 1U;
        }
    }
}

static unsigned lcg_state = 12345U;

/* 确定的伪随机数，[-1, 1)。 */
static float lcg_unit(void)
{
    lcg_state = lcg_state * 1664525U + 1013904223U;
    return (float)((double)(lcg_state >> 8) / 8388608.0 - 1.0);
}

/*
 * 第二级关着时逐位不变。同一串输入（2/3 ms 抖动、同拍重算、每 5 拍一拍只算不推进、
 * 力矩限随机变化而时常顶到、偶发非有限输入、第一级中途关一段再开）分四路：
 * 改动前的单级实现 / 串联版第二级为 NULL / 串联版第二级 f0 = 0（每拍照常 Configure）/
 * 单级包装函数。四路的输出与四个第一级实例的全部字节必须逐拍相同。
 */
static void cascade_off_bitwise(int n)
{
    DRV_MomentNotch legacy, first_null, first_off, second_off, first_wrap;
    DRV_RateControl_Input in;
    int bad = 0;

    DRV_MomentNotch_Reset(&legacy);
    DRV_MomentNotch_Reset(&first_null);
    DRV_MomentNotch_Reset(&first_off);
    DRV_MomentNotch_Reset(&second_off);
    DRV_MomentNotch_Reset(&first_wrap);
    memset(&in, 0, sizeof(in));
    for (int k = 0; k < n; ++k) {
        const float dt = (k % 3 == 0) ? 0.003f : 0.002f;
        const float f0 = (((k / 500) % 3) == 1) ? 0.0f : 6.0f;
        const int advance = (k % 5) != 4;
        DRV_RateControl_Output base, o_legacy, o_null, o_off, o_wrap;

        memset(&base, 0, sizeof(base));
        for (unsigned a = 0; a < 3U; ++a) {
            in.saturation_positive[a] = 0.06f + 0.03f * lcg_unit();
            in.saturation_negative[a] = -0.06f + 0.03f * lcg_unit();
            base.p_term[a] = 0.1f * lcg_unit();
            base.i_term[a] = 0.02f * lcg_unit();
            base.d_term[a] = 0.01f * lcg_unit();
            base.ff_term[a] = 0.05f * lcg_unit();
            base.moment_unsat[a] = base.p_term[a] + base.i_term[a] - base.d_term[a] +
                                   base.ff_term[a] + 0.003f;
            base.moment_cmd[a] = base.moment_unsat[a];
        }
        if ((k % 97) == 0) {
            base.p_term[k % 2] = NAN;
        }
        if (advance) {
            (void)DRV_MomentNotch_Configure(&legacy, f0, 1.2f, dt);
            (void)DRV_MomentNotch_Configure(&first_null, f0, 1.2f, dt);
            (void)DRV_MomentNotch_Configure(&first_off, f0, 1.2f, dt);
            (void)DRV_MomentNotch_Configure(&second_off, 0.0f, 1.0f, dt);
            (void)DRV_MomentNotch_Configure(&first_wrap, f0, 1.2f, dt);
        }
        for (int again = 0; again < 2; ++again) {   /* 同拍重算一次（保护缩放的路径） */
            memcpy(&o_legacy, &base, sizeof(base));
            memcpy(&o_null, &base, sizeof(base));
            memcpy(&o_off, &base, sizeof(base));
            memcpy(&o_wrap, &base, sizeof(base));
            pre_notch2_apply_to_rate_output(&legacy, &in, &o_legacy);
            DRV_MomentNotch_ApplyCascadeToRateOutput(&first_null, NULL, &in, &o_null);
            DRV_MomentNotch_ApplyCascadeToRateOutput(&first_off, &second_off, &in, &o_off);
            DRV_MomentNotch_ApplyToRateOutput(&first_wrap, &in, &o_wrap);
        }
        if (memcmp(&o_legacy, &o_null, sizeof(base)) != 0) bad |= 1;
        if (memcmp(&o_legacy, &o_off, sizeof(base)) != 0) bad |= 2;
        if (memcmp(&o_legacy, &o_wrap, sizeof(base)) != 0) bad |= 4;
        if (advance) {
            DRV_MomentNotch_Commit(&legacy);
            DRV_MomentNotch_Commit(&first_null);
            DRV_MomentNotch_Commit(&first_off);
            DRV_MomentNotch_Commit(&second_off);
            DRV_MomentNotch_Commit(&first_wrap);
        } else {
            DRV_MomentNotch_Discard(&legacy);
            DRV_MomentNotch_Discard(&first_null);
            DRV_MomentNotch_Discard(&first_off);
            DRV_MomentNotch_Discard(&second_off);
            DRV_MomentNotch_Discard(&first_wrap);
        }
        if ((memcmp(&legacy, &first_null, sizeof(legacy)) != 0) ||
            (memcmp(&legacy, &first_off, sizeof(legacy)) != 0) ||
            (memcmp(&legacy, &first_wrap, sizeof(legacy)) != 0)) bad |= 8;
        if (second_off.active != 0U) bad |= 16;
        if ((legacy.active != 0U) != (f0 > 0.0f)) bad |= 32;   /* 覆盖面：第一级确实开过也关过 */
    }
    printf("%d\n", bad);
}

/*
 * 两级串联：p/i/d 先过第一级（f1 = 0 = 关）再过第二级，ff 与陀螺耦合项原样叠回，然后按
 * 同一组限幅重钳、重算饱和标志。反馈是 freq 的正弦 + 直流积分 + 3 Hz 的 D，前馈是同频正弦。
 */
static void cascade_feedback(float f1, float q1, float f2, float q2, float freq, float dt, int n)
{
    DRV_MomentNotch first, second;
    DRV_RateControl_Input in;
    DRV_RateControl_Output out;
    double t = 0.0;

    DRV_MomentNotch_Reset(&first);
    DRV_MomentNotch_Reset(&second);
    memset(&in, 0, sizeof(in));
    for (unsigned a = 0; a < 3U; ++a) {
        in.saturation_positive[a] = 0.12f;
        in.saturation_negative[a] = -0.10f;
    }
    for (int k = 0; k < n; ++k) {
        float fb;
        memset(&out, 0, sizeof(out));
        out.p_term[1] = (float)(0.10 * sin(TWO_PI * freq * t));
        out.i_term[1] = 0.02f;
        out.d_term[1] = (float)(0.01 * sin(TWO_PI * 3.0 * t));
        out.ff_term[1] = (float)(0.05 * sin(TWO_PI * freq * t));
        fb = out.p_term[1] + out.i_term[1] - out.d_term[1];
        out.moment_unsat[1] = fb + out.ff_term[1] + 0.003f;   /* 0.003 = 陀螺耦合项 */
        out.moment_cmd[1] = out.moment_unsat[1];
        (void)DRV_MomentNotch_Configure(&first, f1, q1, dt);
        (void)DRV_MomentNotch_Configure(&second, f2, q2, dt);
        DRV_MomentNotch_ApplyCascadeToRateOutput(&first, &second, &in, &out);
        DRV_MomentNotch_Commit(&first);
        DRV_MomentNotch_Commit(&second);
        printf("%.9g %.9g %.9g %.9g %d %d\n", fb, out.ff_term[1] + 0.003f,
               out.moment_unsat[1], out.moment_cmd[1],
               out.saturated_pos[1], out.saturated_neg[1]);
        t += dt;
    }
}

/* 参考模型：每拍 Evaluate（两次，第二次必须逐位相同）再 Commit。dt_b 用于抖动。 */
static void ref(float wr, float td_ms, float dt_a, float dt_b, float target,
                float align0, int n)
{
    DRV_AttRef r;
    DRV_AttRefStep st, st2;
    DRV_AttRefOutput o, o2;
    const float cmd[2] = { target, -target };
    const float al[2] = { align0, 0.0f };
    int repeat_mismatch = 0;

    memset(&r, 0xCC, sizeof(r));   /* NOLOAD 内存：Reset 之前是垃圾 */
    DRV_AttRef_Reset(&r);
    for (int k = 0; k < n; ++k) {
        const float dt = (k % 2 == 0) ? dt_a : dt_b;
        DRV_AttRef_Evaluate(&r, wr, td_ms * 1.0e-3f, cmd, al, dt, &st, &o);
        DRV_AttRef_Evaluate(&r, wr, td_ms * 1.0e-3f, cmd, al, dt, &st2, &o2);
        if (memcmp(&o, &o2, sizeof(o)) != 0) repeat_mismatch = 1;
        DRV_AttRef_Commit(&r, &st);
        printf("%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d %d\n", dt, st.angle[0], st.rate[0],
               st.accel[0], o.angle[0], o.rate[0], o.accel[0], o.angle[1], o.delay_truncated,
               repeat_mismatch);
    }
}

/* dt = 0 的拍不推进：输出保持上一拍（含 θ̈_ref），也不占历史。 */
static void hold(void)
{
    DRV_AttRef r;
    DRV_AttRefStep st;
    DRV_AttRefOutput last, o;
    const float cmd[2] = { 0.05f, 0.0f };
    const float al[2] = { 0.0f, 0.0f };
    int bad = 0;

    DRV_AttRef_Reset(&r);
    for (int k = 0; k < 30; ++k) {
        DRV_AttRef_Evaluate(&r, 8.0f, 0.055f, cmd, al, 0.004f, &st, &last);
        DRV_AttRef_Commit(&r, &st);
    }
    for (int k = 0; k < 5; ++k) {
        const float moved[2] = { -0.3f, 0.2f };   /* 目标变了也不推进 */
        DRV_AttRef_Evaluate(&r, 8.0f, 0.055f, moved, al, 0.0f, &st, &o);
        DRV_AttRef_Commit(&r, &st);
        if (memcmp(&o, &last, sizeof(o)) != 0) bad |= 1;
    }
    if (r.count != 31U) bad |= 2;   /* 对齐点 + 30 拍；dt = 0 不占历史 */
    printf("%d\n", bad);
}

int main(int argc, char **argv)
{
    if (argc < 2) return 2;
    if (strcmp(argv[1], "coef") == 0) {
        coef((float)atof(argv[2]), (float)atof(argv[3]), (float)atof(argv[4]));
    } else if (strcmp(argv[1], "sine") == 0) {
        sine((float)atof(argv[2]), (float)atof(argv[3]), (float)atof(argv[4]),
             (float)atof(argv[5]), (float)atof(argv[6]), atoi(argv[7]));
    } else if (strcmp(argv[1], "passthrough") == 0) {
        passthrough();
    } else if (strcmp(argv[1], "feedback") == 0) {
        feedback_only((float)atof(argv[2]), atoi(argv[3]));
    } else if (strcmp(argv[1], "ref") == 0) {
        ref((float)atof(argv[2]), (float)atof(argv[3]), (float)atof(argv[4]),
            (float)atof(argv[5]), (float)atof(argv[6]), (float)atof(argv[7]), atoi(argv[8]));
    } else if (strcmp(argv[1], "hold") == 0) {
        hold();
    } else if (strcmp(argv[1], "cascade_off") == 0) {
        cascade_off_bitwise(atoi(argv[2]));
    } else if (strcmp(argv[1], "cascade") == 0) {
        cascade_feedback((float)atof(argv[2]), (float)atof(argv[3]), (float)atof(argv[4]),
                         (float)atof(argv[5]), (float)atof(argv[6]), (float)atof(argv[7]),
                         atoi(argv[8]));
    } else {
        return 2;
    }
    return 0;
}
"""


@pytest.fixture(scope="module")
def driver(tmp_path_factory):
    work = tmp_path_factory.mktemp("shaping_driver")
    return _build(work, "driver", DRIVER_HARNESS, [], [DRIVER_INC])


@pytest.mark.parametrize("f0, q, dt", [(6.0, 1.2, 0.001), (6.0, 1.2, 0.002), (6.0, 1.2, 0.003),
                                       (4.0, 0.8, 0.002), (8.0, 0.5, 0.004)])
def test_notch_coefficients_match_the_design_script(driver, f0, q, dt):
    ok, b0, b1, b2, a1, a2 = (float(v) for v in _run(driver, "coef", f0, q, dt)[0].split())
    b, a = notch_biquad(f0, q, 0.1, 1.0 / dt)
    assert ok == 1
    assert [b0, b1, b2] == pytest.approx(list(b), rel=2e-6, abs=2e-6)
    assert [1.0, a1, a2] == pytest.approx(list(a), rel=2e-6, abs=2e-6)


@pytest.mark.parametrize("dt", [0.002, 0.003])
def test_notch_frequency_response_matches_the_continuous_prototype(driver, dt):
    ok, b0, b1, b2, a1, a2 = (float(v) for v in _run(driver, "coef", 6.0, 1.2, dt)[0].split())
    assert ok == 1

    def h(f):
        z = np.exp(-2j * math.pi * f * dt)
        return (b0 + b1 * z + b2 * z * z) / (1 + a1 * z + a2 * z * z)

    # 预畸变到 f0：中心处深度精确为 d = 0.1（−20 dB）。
    assert 20 * math.log10(abs(h(6.0))) == pytest.approx(-20.0, abs=0.05)
    # 1 Hz：离散与连续原型的增益、相位一致（设计里 3–8 Hz 之外基本不动）。
    ref_1hz = notch_continuous(1.0, 6.0, 1.2)
    assert 20 * math.log10(abs(h(1.0))) == pytest.approx(20 * math.log10(abs(ref_1hz)), abs=0.01)
    assert math.degrees(np.angle(h(1.0))) == pytest.approx(math.degrees(np.angle(ref_1hz)), abs=0.3)
    # 直流增益为 1：配平与积分稳态不受影响。系数是 float，1 + a1 + a2 只有约 0.005，
    # 舍入放大后差 1e-5 量级——这是 float 双二阶的固有精度，与设计无关。
    assert abs(h(0.0)) == pytest.approx(1.0, abs=5e-5)


def _fit_amplitude_phase(t, x, y, freq):
    """最小二乘取 y 相对 x 的增益与相位（x 是单位正弦）。"""
    basis = np.column_stack([np.sin(2 * math.pi * freq * t), np.cos(2 * math.pi * freq * t)])
    cx, *_ = np.linalg.lstsq(basis, x, rcond=None)
    cy, *_ = np.linalg.lstsq(basis, y, rcond=None)
    zx = complex(cx[0], cx[1])
    zy = complex(cy[0], cy[1])
    return abs(zy) / abs(zx), math.degrees(np.angle(zy / zx))


@pytest.mark.parametrize("freq, expect_db", [(6.0, -20.0), (1.0, None)])
def test_notch_time_domain_gain_with_real_timestamp_jitter(driver, freq, expect_db):
    """控制拍在 2–3 ms 之间抖：系数按每拍真实 dt 现算，陷波中心仍在 6 Hz。"""
    rows = _rows(_run(driver, "sine", 6.0, 1.2, 0.002, 0.003, freq, 3000))
    t, x, y = rows[:, 0], rows[:, 1], rows[:, 2]
    tail = t > 3.0
    gain, phase = _fit_amplitude_phase(t[tail], x[tail], y[tail], freq)
    if expect_db is not None:
        # 每拍按真实 dt 现算系数：抖动下中心仍约 −19 dB。对照：系数按名义 500 Hz 固定时，
        # 同一串 2/3 ms 交替的样本在 6 Hz 只剩约 −6 dB（有效中心漂到约 4.8 Hz）。
        assert 20 * math.log10(gain) == pytest.approx(expect_db, abs=1.2)
        b, a = notch_biquad(6.0, 1.2, 0.1, 500.0)
        fixed_gain, _ = _fit_amplitude_phase(t[tail], x[tail], df1_primed(b, a, x)[tail], freq)
        assert 20 * math.log10(fixed_gain) > -10.0
    else:
        ref = notch_continuous(freq, 6.0, 1.2)
        assert gain == pytest.approx(abs(ref), rel=3e-3)
        assert phase == pytest.approx(math.degrees(np.angle(ref)), abs=0.5)


def test_notch_off_or_invalid_is_bitwise_passthrough(driver):
    assert _run(driver, "passthrough") == ["0"]


def test_notch_filters_only_the_feedback_and_reclamps(driver):
    dt = 0.002
    rows = _rows(_run(driver, "feedback", dt, 1500))
    fb, bypass, unsat, cmd, sat_pos, sat_neg = rows.T
    b, a = notch_biquad(6.0, 1.2, 0.1, 1.0 / dt)
    expect = df1_primed(b, a, fb) + bypass
    assert np.max(np.abs(unsat - expect)) < 1e-5   # float 递推 vs double 参考
    # 前馈（6 Hz，正好在陷波中心）原样叠回：出口里的 6 Hz 前馈分量不衰减。
    t = np.arange(len(fb)) * dt
    tail = t > 1.0
    basis = np.column_stack([np.sin(2 * math.pi * 6 * t[tail]), np.cos(2 * math.pi * 6 * t[tail])])
    coeffs, *_ = np.linalg.lstsq(basis, (unsat - df1_primed(b, a, fb))[tail], rcond=None)
    assert math.hypot(*coeffs) == pytest.approx(0.05, rel=1e-3)
    # 重钳：+0.12 / −0.10，饱和标志按钳位后的值重算。
    assert np.all(cmd <= 0.12 + 1e-9) and np.all(cmd >= -0.10 - 1e-9)
    assert np.array_equal(sat_pos == 1, unsat >= 0.12)
    assert np.array_equal(sat_neg == 1, unsat <= -0.10)
    assert np.all(cmd[(sat_pos == 0) & (sat_neg == 0)] == unsat[(sat_pos == 0) & (sat_neg == 0)])


def _component(t, y, freq, others=(3.0,)):
    """y 里 freq 分量的复幅值（同时拟合直流与 others 里的频率，免得它们漏进来）。"""
    cols = [np.ones_like(t)]
    for f in (freq, *others):
        cols += [np.sin(2 * math.pi * f * t), np.cos(2 * math.pi * f * t)]
    coeffs, *_ = np.linalg.lstsq(np.column_stack(cols), y, rcond=None)
    return complex(coeffs[1], coeffs[2])


def _digital_gain(b, a, f_hz, dt):
    z = np.exp(-2j * math.pi * f_hz * dt)
    return abs((b[0] + b[1] * z + b[2] * z * z) / (1 + a[1] * z + a[2] * z * z))


def test_second_notch_off_is_bitwise_the_pre_change_single_notch(driver):
    """第二级为 NULL / f0 = 0 / 走单级包装：与改动前单级实现的逐字副本逐拍逐字节相同。"""
    assert _run(driver, "cascade_off", 3000) == ["0"]


@pytest.mark.parametrize("f1", [0.0, 6.0])
def test_second_notch_filters_the_feedback_after_the_first_and_skips_feedforward(driver, f1):
    """第二级 16 Hz、Q 1：接在第一级之后滤同一个反馈；f0 处约 −20 dB；前馈与陀螺项不过它。"""
    dt = 0.002
    rows = _rows(_run(driver, "cascade", f1, 1.2, 16.0, 1.0, 16.0, dt, 2000))
    fb, bypass, unsat, cmd, sat_pos, sat_neg = rows.T
    b2, a2 = notch_biquad(16.0, 1.0, 0.1, 1.0 / dt)
    if f1 > 0.0:
        b1, a1 = notch_biquad(f1, 1.2, 0.1, 1.0 / dt)
        filtered = df1_primed(b2, a2, df1_primed(b1, a1, fb))
        first_db = 20 * math.log10(_digital_gain(b1, a1, 16.0, dt))
    else:
        filtered = df1_primed(b2, a2, fb)
        first_db = 0.0
    assert np.max(np.abs(unsat - (filtered + bypass))) < 1e-5   # float 递推 vs double 参考
    t = np.arange(len(fb)) * dt
    tail = t > 1.0
    gain = abs(_component(t[tail], unsat[tail] - bypass[tail], 16.0)) / \
        abs(_component(t[tail], fb[tail], 16.0))
    assert 20 * math.log10(gain) == pytest.approx(-20.0 + first_db, abs=0.3)
    # 直流（积分项 0.02）增益为 1：两级都不动配平。
    tt = t[tail]
    cols = np.column_stack([np.ones_like(tt)] + [fn(2 * math.pi * f * tt) for f in (16.0, 3.0)
                                                 for fn in (np.sin, np.cos)])
    dc = np.linalg.lstsq(cols, (unsat - bypass)[tail], rcond=None)[0][0]
    assert dc == pytest.approx(0.02, abs=1e-4)
    # 前馈（16 Hz，正好在第二级中心）原样叠回：出口里的 16 Hz 前馈分量不衰减。
    assert abs(_component(t[tail], (unsat - filtered)[tail], 16.0)) == pytest.approx(0.05, rel=1e-3)
    # 重钳：+0.12 / −0.10，饱和标志按钳位后的值重算。
    assert np.all(cmd <= 0.12 + 1e-9) and np.all(cmd >= -0.10 - 1e-9)
    assert np.array_equal(sat_pos == 1, unsat >= 0.12)
    assert np.array_equal(sat_neg == 1, unsat <= -0.10)


@pytest.mark.parametrize("dt", [0.001, 0.004])
def test_reference_step_matches_the_design_script_point_by_point(driver, dt):
    """3° 阶跃、ωr 8、Td 55 ms：当前参考与延后参考都与设计脚本 sim() 的参考部分逐点一致。"""
    target = math.radians(3.0)
    n = int(1.5 / dt)
    rows = _rows(_run(driver, "ref", 8.0, 55.0, dt, dt, target, 0.0, n))
    assert np.all(rows[:, 9] == 0), "同一拍重复 Evaluate 必须逐位相同"
    assert np.all(rows[:, 8] == 0), "历史容量应覆盖 55 ms"
    py = reference_steps(8.0, target, dt, n)
    assert np.max(np.abs(rows[:, 1] - py[:, 0])) < 2e-7
    assert np.max(np.abs(rows[:, 2] - py[:, 1])) < 2e-6
    assert np.max(np.abs(rows[:, 3] - py[:, 2])) < 2e-5
    assert np.array_equal(rows[:, 6], rows[:, 3]), "α_ff 用的是本拍 θ̈_ref，不延后"
    if dt == 0.001:
        # 设计脚本：lag 队列长 55，rf = r[k−55]，开头 55 拍是 (0, 0)。
        lag = 55
        exp_r = np.concatenate([np.zeros(lag), py[:-lag, 0]])
        exp_rd = np.concatenate([np.zeros(lag), py[:-lag, 1]])
    else:
        times = np.arange(1, n + 1) * dt
        exp_r = np.array([delayed(py[:, 0], times, 0.0, tq - 0.055) for tq in times])
        exp_rd = np.array([delayed(py[:, 1], times, 0.0, tq - 0.055) for tq in times])
    assert np.max(np.abs(rows[:, 4] - exp_r)) < 2e-6
    assert np.max(np.abs(rows[:, 5] - exp_rd)) < 2e-5
    # 第二轴：目标取反、对齐 0 → 结果取反（两轴独立、同一公式）。
    assert np.max(np.abs(rows[:, 7] + rows[:, 4])) < 1e-9
    # 临界阻尼：不越过目标（直接姿态目标已按 tilt_limit 夹过，参考越不过它）。
    assert rows[:, 1].max() <= target * (1 + 1e-6)


def test_reference_aligns_to_the_given_angle_and_delays_in_time(driver):
    """对齐点之前视为一直停在对齐角；调度抖动（2/3 ms 交替）下延后仍是时间意义的 55 ms。"""
    target = math.radians(3.0)
    align = math.radians(-2.0)
    rows = _rows(_run(driver, "ref", 8.0, 55.0, 0.002, 0.003, target, align, 400))
    dts = rows[:, 0]
    times = np.cumsum(dts)
    # 第一拍从对齐角起步，不跳到目标。
    assert abs(rows[0, 1] - align) < 1e-4
    # 开头 Td 内延后参考就是对齐角，速度 0。
    early = times < 0.055
    assert np.all(np.abs(rows[early, 4] - align) < 1e-6)
    assert np.all(np.abs(rows[early, 5]) < 1e-6)
    later = times > 0.08
    exp = np.array([delayed(rows[:, 1], times, align, tq - 0.055) for tq in times[later]])
    assert np.max(np.abs(rows[later, 4] - exp)) < 2e-6


def test_reference_history_truncation_is_flagged(driver):
    """1 kHz 下 64 条历史只够 64 ms：要 80 ms 时按最老一条给并置位（飞控 250 Hz、光杆 500 Hz 不会到这里）。"""
    rows = _rows(_run(driver, "ref", 8.0, 80.0, 0.001, 0.001, 0.05, 0.0, 200))
    assert rows[:40, 8].max() == 0
    assert rows[-1, 8] == 1


def test_reference_hold_does_not_advance(driver):
    assert _run(driver, "hold") == ["0"]


# =============================================================================== 控制器接线


CTRL_HARNESS = AIRFRAME_FIXTURE_C + r"""
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "drv_coax_ctrl.h"

static void need(int ok, const char *what)
{
    if (!ok) {
        printf("FAIL %s\n", what);
        exit(3);
    }
}

static void set(const char *name, float value)
{
    need(DRV_COAX_CTRL_SetParam(name, value) != 0U, name);
}

static uint64_t fnv(uint64_t h, const void *data, size_t n)
{
    const unsigned char *p = (const unsigned char *)data;
    for (size_t i = 0; i < n; ++i) {
        h ^= p[i];
        h *= 1099511628211ULL;
    }
    return h;
}

/* 1 kHz 节拍：速率每 2 拍（dt 在 2/3 ms 间抖）、姿态每 4 拍、速度每 10 拍、位置每 20 拍。 */
static void flight_schedule(int k, DRV_COAX_CTRL_Schedule *s)
{
    memset(s, 0, sizeof(*s));
    s->rate_update = (uint8_t)((k % 2) == 0);
    s->rate_dt_s = s->rate_update ? (((k / 2) % 3 == 0) ? 0.003f : 0.002f) : 0.0f;
    s->attitude_update = (uint8_t)((k % 4) == 0);
    s->attitude_dt_s = s->attitude_update ? 0.004f : 0.0f;
    s->velocity_update = (uint8_t)((k % 10) == 0);
    s->velocity_dt_s = s->velocity_update ? 0.010f : 0.0f;
    s->position_update = (uint8_t)((k % 20) == 0);
    s->position_dt_s = s->position_update ? 0.020f : 0.0f;
}

/*
 * 覆盖面尽量宽的一段：直接姿态（每拍 integrator_reset，与 app_stabilizer 一致）与力矢量
 * （位置/速度环）交替、目标越过 tilt_limit、姿态保护触发同拍重算、速度无效保护、中途改参、
 * 偏航前馈。每拍对 Output 与 Debug 的全部字节取哈希。
 * notch_hz / notch2_hz 非 0 时先打开对应的出口陷波（0 = 不碰，改动前的源码也能跑）。
 */
static void scenario(float notch_hz, float notch2_hz)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Debug dbg;

    if (notch_hz != 0.0f) {
        set("coax.rate_out_notch_hz", notch_hz);
    }
    if (notch2_hz != 0.0f) {
        set("coax.rate_out_notch2_hz", notch2_hz);
    }
    set("coax.rate_pitch_kd", 0.002f);
    set("coax.rate_roll_ki", 0.2f);
    for (int k = 0; k < 1600; ++k) {
        const double t = 0.001 * k;
        const int direct = ((k / 400) % 2) == 0;
        uint64_t h = 1469598103934665603ULL;

        memset(&att, 0, sizeof(att));
        memset(&ref, 0, sizeof(ref));
        att.roll_rad = (float)(0.12 * sin(4.4 * t) + ((k >= 300 && k < 330) ? 0.55 : 0.0));
        att.pitch_rad = (float)(0.09 * cos(8.2 * t));
        att.yaw_rad = (float)(0.3 * sin(0.9 * t));
        att.gyro_x_rad_s = (float)(0.5 * cos(4.4 * t));
        att.gyro_y_rad_s = (float)(-0.7 * sin(8.2 * t));
        att.gyro_z_rad_s = (float)(0.27 * cos(0.9 * t));
        att.x_m = (float)(0.1 * t);
        att.y_m = -0.05f;
        att.z_m = 0.4f;
        att.vx_m_s = (float)(0.2 * sin(1.1 * t));
        att.vy_m_s = (float)(-0.1 * cos(0.7 * t));
        att.vz_m_s = 0.03f;
        att.accel_m_s2[0] = 0.1f;
        att.accel_m_s2[1] = -0.2f;
        att.accel_m_s2[2] = 0.05f;
        att.acceleration_valid = 1U;
        ref.yaw_rad = (float)(0.1 * t);
        ref.yaw_rate_rad_s = 0.1f;
        ref.yaw_accel_rad_s2 = (float)(0.05 * sin(t));
        if (direct) {
            ref.direct_attitude_target_valid = 1U;
            ref.manual_total_force_valid = 1U;
            ref.manual_total_force_n = (float)(11.0 + sin(2.0 * t));
            ref.target_roll_rad = ((k % 200) < 100) ? 0.08f : -0.12f;
            ref.target_pitch_rad = (float)(0.2 * sin(3.0 * t)) + (((k % 250) < 60) ? 0.6f : 0.0f);
        } else {
            ref.vx_m_s = 0.3f;
            ref.vy_m_s = -0.2f;
            ref.z_m = 0.5f;
            ref.ax_m_s2 = 0.1f;
            ref.horizontal_velocity_valid = (uint8_t)((k % 97) != 0);
            ref.navigation_velocity_valid = 1U;
            ref.navigation_position_valid = 1U;
        }
        flight_schedule(k, &s);
        s.integrator_enable = (uint8_t)(direct ? 0 : 1);
        s.integrator_freeze = (uint8_t)(direct ? 1 : 0);
        s.integrator_reset = (uint8_t)((direct || k == 1234) ? 1 : 0);
        if (k == 777) {
            set("coax.rate_roll_kp", 0.15f);
        }
        DRV_COAX_CTRL_RunScheduled(&att, &ref, &s, &out);
        memset(&dbg, 0, sizeof(dbg));
        DRV_COAX_CTRL_GetLastDebug(&dbg);
        h = fnv(h, &out, sizeof(out));
        h = fnv(h, &dbg, sizeof(dbg));
        printf("%d %08lx%08lx %.9g %.9g %u %u\n", k, (unsigned long)(h >> 32),
               (unsigned long)(h & 0xFFFFFFFFULL),
               dbg.moment_cmd_n_m[0], dbg.moment_cmd_n_m[1],
               out.servo_alpha_us, out.servo_beta_us);
    }
}

static void direct_reference(DRV_COAX_CTRL_Reference *ref, float roll, float pitch)
{
    memset(ref, 0, sizeof(*ref));
    ref->direct_attitude_target_valid = 1U;
    ref->manual_total_force_valid = 1U;
    ref->manual_total_force_n = 11.0f;
    ref->target_roll_rad = roll;
    ref->target_pitch_rad = pitch;
}

/*
 * 参考模型开着的直接姿态阶跃（姿态不动，只看整形接线）。每拍 integrator_reset = 1，
 * 与 app_stabilizer 的直接姿态模式一致：参考不许被它对齐掉。
 */
static void shape_step(float wr, float td_ms)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Debug dbg;

    set("coax.att_ref_wr_rad_s", wr);
    set("coax.att_ref_delay_ms", td_ms);
    set("coax.rate_roll_ff", 1.0f);
    set("coax.rate_pitch_ff", 1.0f);
    for (int k = 0; k < 1200; ++k) {
        memset(&att, 0, sizeof(att));
        direct_reference(&ref, -0.03f, 0.05f);
        memset(&s, 0, sizeof(s));
        s.rate_update = (uint8_t)((k % 2) == 0);
        s.rate_dt_s = s.rate_update ? 0.002f : 0.0f;
        s.attitude_update = (uint8_t)((k % 4) == 0);
        s.attitude_dt_s = s.attitude_update ? 0.004f : 0.0f;
        s.integrator_freeze = 1U;
        s.integrator_reset = 1U;
        DRV_COAX_CTRL_RunScheduled(&att, &ref, &s, &out);
        DRV_COAX_CTRL_GetLastDebug(&dbg);
        printf("%d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
               k, s.attitude_update,
               dbg.target_attitude_rp_rad[0], dbg.target_attitude_rp_rad[1],
               dbg.desired_attitude_rpy_rad[0], dbg.desired_attitude_rpy_rad[1],
               dbg.omega_ff_rad_s[0], dbg.omega_ff_rad_s[1], dbg.omega_ff_rad_s[2],
               dbg.omega_sp_rad_s[1], dbg.rate_ff_n_m[0], dbg.rate_ff_n_m[1],
               dbg.rate_ff_n_m[2], dbg.moment_cmd_n_m[1]);
    }
}

/*
 * 对齐：解锁后第一拍、DRV_COAX_CTRL_ResetState()、目标来源切换，都从当前实测姿态起步。
 * 段落：0–299 直接姿态（实测 0.1）；300 起调一次 ResetState 且实测变成 −0.05；
 * 600 起切到力矢量模式、实测 −0.02；900 起切回直接姿态、实测 0.03。
 */
static void align(void)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Debug dbg;

    set("coax.att_ref_wr_rad_s", 8.0f);
    set("coax.att_ref_delay_ms", 55.0f);
    for (int k = 0; k < 1200; ++k) {
        const float measured = (k < 300) ? 0.1f : (k < 600) ? -0.05f : (k < 900) ? -0.02f : 0.03f;
        const int direct = (k < 600) || (k >= 900);

        if (k == 300) {
            DRV_COAX_CTRL_ResetState();
        }
        memset(&att, 0, sizeof(att));
        att.pitch_rad = measured;
        if (direct) {
            direct_reference(&ref, 0.0f, 0.0f);
        } else {
            memset(&ref, 0, sizeof(ref));
            ref.horizontal_velocity_valid = 1U;
            ref.navigation_velocity_valid = 1U;
        }
        memset(&s, 0, sizeof(s));
        s.rate_update = (uint8_t)((k % 2) == 0);
        s.rate_dt_s = s.rate_update ? 0.002f : 0.0f;
        s.attitude_update = (uint8_t)((k % 4) == 0);
        s.attitude_dt_s = s.attitude_update ? 0.004f : 0.0f;
        s.integrator_enable = (uint8_t)(direct ? 0 : 1);
        s.integrator_freeze = (uint8_t)(direct ? 1 : 0);
        s.integrator_reset = (uint8_t)(direct ? 1 : 0);
        DRV_COAX_CTRL_RunScheduled(&att, &ref, &s, &out);
        DRV_COAX_CTRL_GetLastDebug(&dbg);
        printf("%d %.9g %.9g %.9g\n", k, measured, dbg.desired_attitude_rpy_rad[1],
               dbg.target_attitude_rp_rad[1]);
    }
}

/*
 * 出口陷波接线：俯仰陀螺给正弦，角度与目标都为 0，于是反馈 = −kp·ω（积分每拍清）。
 * notch2_hz 非 0 时再开第二级（Q 取默认 1.0）；wr 非 0 时打开参考模型、俯仰目标给 0.05 rad，
 * 于是力矩前馈非零——用来看前馈是不是绕过了两级陷波。
 */
static void notch_wire(float notch_hz, float freq, int reset_each_tick, float notch2_hz, float wr)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Debug dbg;

    set("coax.rate_out_notch_hz", notch_hz);
    set("coax.rate_out_notch_q", 1.2f);
    if (notch2_hz != 0.0f) {
        set("coax.rate_out_notch2_hz", notch2_hz);
    }
    if (wr != 0.0f) {
        set("coax.att_ref_wr_rad_s", wr);
        set("coax.rate_pitch_ff", 1.0f);
    }
    for (int k = 0; k < 4000; ++k) {
        memset(&att, 0, sizeof(att));
        att.gyro_y_rad_s = (float)(0.2 * sin(6.28318530717958647692 * freq * 0.001 * k));
        direct_reference(&ref, 0.0f, (wr != 0.0f) ? 0.05f : 0.0f);
        memset(&s, 0, sizeof(s));
        s.rate_update = (uint8_t)((k % 2) == 0);
        s.rate_dt_s = s.rate_update ? 0.002f : 0.0f;
        s.attitude_update = (uint8_t)((k % 4) == 0);
        s.attitude_dt_s = s.attitude_update ? 0.004f : 0.0f;
        s.integrator_freeze = 1U;
        s.integrator_reset = (uint8_t)reset_each_tick;
        DRV_COAX_CTRL_RunScheduled(&att, &ref, &s, &out);
        DRV_COAX_CTRL_GetLastDebug(&dbg);
        if (s.rate_update) {
            printf("%.9g %.9g %.9g\n",
                   dbg.rate_p_n_m[1] + dbg.rate_i_n_m[1] - dbg.rate_d_n_m[1],
                   dbg.rate_ff_n_m[1], dbg.moment_cmd_n_m[1]);
        }
    }
}

/*
 * 闭环：真实控制器 + 辨识对象（离散矩阵由 Python 按设计脚本同一方法算好传进来）。
 * mode 0：所有环每 1 ms 一拍（与设计脚本同拍，对拍用）；mode 1：飞控真实节拍
 * （速率 500 Hz、姿态 250 Hz，每 1 ms 调一次 RunScheduled），对象用舵机量化后的实际力矩。
 */
static void closed(int argc, char **argv)
{
    int mode, n, lag;
    float kp, ki, attkp, wr, td, notch, q, inertia, target, force;
    double phi[4][4], gam[4], x[4] = { 0, 0, 0, 0 };
    double queue[256];
    DRV_Airframe_Params frame;
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Schedule s;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Debug dbg;

    need(argc >= 35, "args");
    mode = atoi(argv[2]);
    kp = (float)atof(argv[3]);
    ki = (float)atof(argv[4]);
    attkp = (float)atof(argv[5]);
    wr = (float)atof(argv[6]);
    td = (float)atof(argv[7]);
    notch = (float)atof(argv[8]);
    q = (float)atof(argv[9]);
    inertia = (float)atof(argv[10]);
    target = (float)atof(argv[11]);
    n = atoi(argv[12]);
    lag = atoi(argv[13]);
    force = (float)atof(argv[14]);
    need(lag < 256, "lag");
    for (int i = 0; i < 16; ++i) phi[i / 4][i % 4] = atof(argv[15 + i]);
    for (int i = 0; i < 4; ++i) gam[i] = atof(argv[31 + i]);
    for (int i = 0; i < lag; ++i) queue[i] = 0.0;

    DRV_Airframe_GetParams(&frame);
    frame.ixx_kgm2 = inertia;
    frame.iyy_kgm2 = inertia;
    DRV_Airframe_SetParams(&frame);
    need(DRV_Airframe_IsValid() != 0U, "airframe");
    set("coax.rate_pitch_kp", kp);
    set("coax.rate_pitch_ki", ki);
    set("coax.att_pitch_kp", attkp);
    set("coax.rate_pitch_ff", 1.0f);
    set("coax.rate_pitch_i_limit_n_m", 0.05f);
    set("coax.att_ref_wr_rad_s", wr);
    set("coax.att_ref_delay_ms", td);
    set("coax.rate_out_notch_hz", notch);
    set("coax.rate_out_notch_q", q);
    for (int k = 0; k < n; ++k) {
        double u, v;
        double nx[4];

        memset(&att, 0, sizeof(att));
        att.pitch_rad = (float)x[2];
        att.gyro_y_rad_s = (float)x[3];
        direct_reference(&ref, 0.0f, target);
        ref.manual_total_force_n = force;
        memset(&s, 0, sizeof(s));
        if (mode == 0) {
            s.rate_update = 1U;
            s.rate_dt_s = 0.001f;
            s.attitude_update = 1U;
            s.attitude_dt_s = 0.001f;
        } else {
            s.rate_update = (uint8_t)((k % 2) == 0);
            s.rate_dt_s = s.rate_update ? 0.002f : 0.0f;
            s.attitude_update = (uint8_t)((k % 4) == 0);
            s.attitude_dt_s = s.attitude_update ? 0.004f : 0.0f;
        }
        s.integrator_enable = 1U;
        DRV_COAX_CTRL_RunScheduled(&att, &ref, &s, &out);
        DRV_COAX_CTRL_GetLastDebug(&dbg);
        u = (mode == 0) ? (double)dbg.moment_cmd_n_m[1] : (double)out.moment_achieved_n_m[1];
        if (lag > 0) {
            v = queue[k % lag];
            queue[k % lag] = u;
        } else {
            v = u;
        }
        for (int i = 0; i < 4; ++i) {
            nx[i] = gam[i] * v;
            for (int j = 0; j < 4; ++j) nx[i] += phi[i][j] * x[j];
        }
        memcpy(x, nx, sizeof(x));
        printf("%.12g %.9g\n", x[2], u);
    }
}

#ifndef PRE_CHANGE_BASELINE   /* 改动前的头文件里没有这四个字段 */
/* 参数校验：逐个 PARAM SET，打印是否接受与之后的回读值。 */
static void params(void)
{
    static const struct { const char *name; float value; } cases[] = {
        { "coax.rate_out_notch_hz", 0.0f }, { "coax.rate_out_notch_hz", 0.5f },
        { "coax.rate_out_notch_hz", 1.0f }, { "coax.rate_out_notch_hz", 6.0f },
        { "coax.rate_out_notch_hz", 100.0f }, { "coax.rate_out_notch_hz", 100.5f },
        { "coax.rate_out_notch_hz", -1.0f }, { "coax.rate_out_notch_hz", NAN },
        { "coax.rate_out_notch_hz", INFINITY },
        { "coax.rate_out_notch_q", 0.3f }, { "coax.rate_out_notch_q", 0.29f },
        { "coax.rate_out_notch_q", 10.0f }, { "coax.rate_out_notch_q", 10.5f },
        { "coax.rate_out_notch_q", 0.0f }, { "coax.rate_out_notch_q", -1.2f },
        { "coax.rate_out_notch_q", NAN },
        { "coax.rate_out_notch2_hz", 0.0f }, { "coax.rate_out_notch2_hz", 0.5f },
        { "coax.rate_out_notch2_hz", 1.0f }, { "coax.rate_out_notch2_hz", 16.0f },
        { "coax.rate_out_notch2_hz", 100.0f }, { "coax.rate_out_notch2_hz", 100.5f },
        { "coax.rate_out_notch2_hz", -1.0f }, { "coax.rate_out_notch2_hz", NAN },
        { "coax.rate_out_notch2_hz", INFINITY },
        { "coax.rate_out_notch2_q", 0.3f }, { "coax.rate_out_notch2_q", 0.29f },
        { "coax.rate_out_notch2_q", 10.0f }, { "coax.rate_out_notch2_q", 10.5f },
        { "coax.rate_out_notch2_q", 0.0f }, { "coax.rate_out_notch2_q", -1.0f },
        { "coax.rate_out_notch2_q", NAN },
        { "coax.att_ref_wr_rad_s", 0.0f }, { "coax.att_ref_wr_rad_s", 0.4f },
        { "coax.att_ref_wr_rad_s", 0.5f }, { "coax.att_ref_wr_rad_s", 30.0f },
        { "coax.att_ref_wr_rad_s", 30.5f }, { "coax.att_ref_wr_rad_s", -8.0f },
        { "coax.att_ref_wr_rad_s", NAN }, { "coax.att_ref_wr_rad_s", INFINITY },
        { "coax.att_ref_delay_ms", 0.0f }, { "coax.att_ref_delay_ms", 80.0f },
        { "coax.att_ref_delay_ms", 80.1f }, { "coax.att_ref_delay_ms", -1.0f },
        { "coax.att_ref_delay_ms", INFINITY },
    };
    DRV_COAX_CTRL_Params p;

    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        float before = -99.0f, after = -99.0f;
        uint8_t ok;
        need(DRV_COAX_CTRL_GetParam(cases[i].name, &before) != 0U, "get");
        ok = DRV_COAX_CTRL_SetParam(cases[i].name, cases[i].value);
        need(DRV_COAX_CTRL_GetParam(cases[i].name, &after) != 0U, "get");
        printf("%s %.9g %u %.9g %.9g\n", cases[i].name, cases[i].value, ok, before, after);
    }
    /* 整份写入同样走校验：坏值整份拒收（第二级的坏值也一样）。 */
    DRV_COAX_CTRL_GetParams(&p);
    p.rate_out_notch2_hz = 0.5f;
    p.att_ref_wr_rad_s = 8.0f;
    DRV_COAX_CTRL_SetParams(&p);
    DRV_COAX_CTRL_GetParams(&p);
    printf("bulk2 %.9g %.9g\n", p.rate_out_notch2_hz, p.att_ref_wr_rad_s);
    DRV_COAX_CTRL_GetParams(&p);
    p.att_ref_wr_rad_s = -1.0f;
    DRV_COAX_CTRL_SetParams(&p);
    DRV_COAX_CTRL_GetParams(&p);
    printf("bulk %.9g\n", p.att_ref_wr_rad_s);
}
#endif

int main(int argc, char **argv)
{
    airframe_load_reference();
    DRV_COAX_CTRL_ResetParams();
    if (argc < 2) return 2;
    if (strcmp(argv[1], "scenario") == 0) {
        scenario((argc > 2) ? (float)atof(argv[2]) : 0.0f,
                 (argc > 3) ? (float)atof(argv[3]) : 0.0f);
    } else if (strcmp(argv[1], "shape_step") == 0) {
        shape_step((float)atof(argv[2]), (float)atof(argv[3]));
    } else if (strcmp(argv[1], "align") == 0) {
        align();
    } else if (strcmp(argv[1], "notchwire") == 0) {
        notch_wire((float)atof(argv[2]), (float)atof(argv[3]), atoi(argv[4]),
                   (argc > 5) ? (float)atof(argv[5]) : 0.0f,
                   (argc > 6) ? (float)atof(argv[6]) : 0.0f);
    } else if (strcmp(argv[1], "closed") == 0) {
        closed(argc, argv);
#ifndef PRE_CHANGE_BASELINE
    } else if (strcmp(argv[1], "params") == 0) {
        params();
#endif
    } else {
        return 2;
    }
    return 0;
}
"""

# 把整形接线从控制器源码里剥掉：得到"加入整形之前"的控制律（同一份源码的其余部分）。
SHAPING_WIRING = (
    "    coax_ctrl_shape_attitude_target(attitude, reference, schedule,\n"
    "                                    &target_roll_rad, &target_pitch_rad,\n"
    "                                    ref_rate_ff, ref_accel_ff);\n",
    "    attitude_input.desired_rate_in_desired_frame[0] = ref_rate_ff[0];\n"
    "    attitude_input.desired_rate_in_desired_frame[1] = ref_rate_ff[1];\n",
    "    rate_input.alpha_ff[0] = ref_accel_ff[0];\n"
    "    rate_input.alpha_ff[1] = ref_accel_ff[1];\n",
    "    coax_ctrl_apply_moment_notch(&rate_input, schedule);\n",
    "    coax_ctrl_commit_shaping(schedule);\n",
)


def stripped_controller_source() -> str:
    text = CTRL_SOURCE.read_text(encoding="utf-8").replace("\r\n", "\n")
    for anchor in SHAPING_WIRING:
        assert text.count(anchor) == 1, f"整形接线点已改写，需同步本测试：{anchor!r}"
        text = text.replace(anchor, "")
    return text


# 把第二级出口陷波的接线从控制器源码里剥掉：得到"只有第一级"的控制律，接线与第二级加入之前
# 的写法逐字相同（单级包装函数与改动前单级实现逐位相同，见 test_second_notch_off_is_bitwise_the_pre_change_single_notch）。
NOTCH2_WIRING = (
    ("        (coax_ctrl_shaping.notch.active == 0U) &&\n"
     "        !(coax_ctrl_params.rate_out_notch2_hz > 0.0f) &&\n"
     "        (coax_ctrl_shaping.notch2.active == 0U)) {\n",
     "        (coax_ctrl_shaping.notch.active == 0U)) {\n"),
    ("        (void)DRV_MomentNotch_Configure(&coax_ctrl_shaping.notch2,\n"
     "                                        coax_ctrl_params.rate_out_notch2_hz,\n"
     "                                        coax_ctrl_params.rate_out_notch2_q,\n"
     "                                        schedule->rate_dt_s);\n", ""),
    ("    DRV_MomentNotch_ApplyCascadeToRateOutput(&coax_ctrl_shaping.notch,\n"
     "                                             &coax_ctrl_shaping.notch2, rate_input,\n"
     "                                             &coax_ctrl_state.rate_output);\n",
     "    DRV_MomentNotch_ApplyToRateOutput(&coax_ctrl_shaping.notch, rate_input,\n"
     "                                      &coax_ctrl_state.rate_output);\n"),
    ("        DRV_MomentNotch_Commit(&coax_ctrl_shaping.notch2);\n", ""),
    ("        DRV_MomentNotch_Discard(&coax_ctrl_shaping.notch2);\n", ""),
)


def single_notch_controller_source() -> str:
    text = CTRL_SOURCE.read_text(encoding="utf-8").replace("\r\n", "\n")
    for anchor, replacement in NOTCH2_WIRING:
        assert text.count(anchor) == 1, f"第二级陷波接线点已改写，需同步本测试：{anchor!r}"
        text = text.replace(anchor, replacement)
    assert "notch2" not in text.split("static void coax_ctrl_apply_moment_notch(", 1)[1].split(
        "static void coax_ctrl_compute_balance_solution(", 1)[0], "剥得不干净"
    return text


def _ctrl_build(work: Path, name: str, controller: Path, extra_includes=(), defines=()) -> Path:
    return _build(work, name, CTRL_HARNESS, [controller, *CTRL_DEPS],
                  [*extra_includes, DRIVER_INC], werror=controller == CTRL_SOURCE,
                  defines=tuple(defines))


@pytest.fixture(scope="module")
def ctrl(tmp_path_factory):
    work = tmp_path_factory.mktemp("shaping_ctrl")
    return _ctrl_build(work, "ctrl", CTRL_SOURCE)


def test_shaping_off_is_bitwise_the_controller_without_the_wiring(ctrl, tmp_path):
    """两个开关为 0：每拍 Output + Debug 的全部字节与剥掉整形接线的那一版完全相同。"""
    stripped = tmp_path / "drv_coax_ctrl_stripped.c"
    stripped.write_text(stripped_controller_source(), encoding="utf-8")
    baseline = _ctrl_build(tmp_path, "stripped", stripped)
    assert _run(ctrl, "scenario") == _run(baseline, "scenario")


@pytest.fixture(scope="module")
def single_notch_ctrl(tmp_path_factory):
    work = tmp_path_factory.mktemp("shaping_single_notch")
    source = work / "drv_coax_ctrl_single_notch.c"
    source.write_text(single_notch_controller_source(), encoding="utf-8")
    return _ctrl_build(work, "single_notch", source)


@pytest.mark.parametrize("notch_hz", [0.0, 6.0, 16.0])
def test_second_notch_off_is_bitwise_the_single_notch_controller(ctrl, single_notch_ctrl, notch_hz):
    """第二级关着（notch2_hz = 0 且未生效）：第一级关/开时，每拍 Output + Debug 的全部字节与
    剥掉第二级接线的那一版完全相同。"""
    ours = _run(ctrl, "scenario", notch_hz, 0.0)
    assert ours == _run(single_notch_ctrl, "scenario", notch_hz, 0.0)
    if notch_hz == 6.0:
        # 这段场景确实经过第二级：打开它输出就变（比较有分辨力）。
        assert _run(ctrl, "scenario", notch_hz, 16.0) != ours


@pytest.mark.skipif(not os.environ.get("SHAPING_BASELINE_DIR"),
                    reason="设 SHAPING_BASELINE_DIR=<含 Src/drv_coax_ctrl.c 与 Inc/drv_coax_ctrl.h 的改动前快照> 才跑")
def test_shaping_off_is_bitwise_the_pre_change_controller(ctrl, tmp_path):
    """与改动前的真实源码（快照目录）逐拍逐字节对拍——合并前的一次性证据，平时跳过。"""
    snap = Path(os.environ["SHAPING_BASELINE_DIR"])
    baseline = _ctrl_build(tmp_path, "pre_change", snap / "Src" / "drv_coax_ctrl.c",
                           extra_includes=(snap / "Inc",), defines=("PRE_CHANGE_BASELINE",))
    assert _run(ctrl, "scenario") == _run(baseline, "scenario")


def _shape_rows(ctrl, wr=8.0, td=55.0):
    return _rows(_run(ctrl, "shape_step", wr, td))


def test_enabled_reference_drives_delayed_feedback_and_torque_feedforward(ctrl):
    rows = _shape_rows(ctrl)
    att_upd = rows[:, 1]
    tgt_r, tgt_p, des_r, des_p = rows[:, 2], rows[:, 3], rows[:, 4], rows[:, 5]
    wff_p, wff_y, wsp_p = rows[:, 7], rows[:, 8], rows[:, 9]
    ff_r, ff_p, ff_y = rows[:, 10], rows[:, 11], rows[:, 12]
    iyy = 0.051   # 夹具机体（tests/_airframe_fixture.py）
    # 调试里的 target 是指令本身；desired 是角度环真正跟的延后参考。
    assert np.all(tgt_p.astype(np.float32) == np.float32(0.05))
    assert np.all(tgt_r.astype(np.float32) == np.float32(-0.03))
    steps = np.nonzero(att_upd == 1)[0]
    n = steps.size
    py_p = reference_steps(8.0, float(np.float32(0.05)), 0.004, n)
    py_r = reference_steps(8.0, float(np.float32(-0.03)), 0.004, n)
    times = (np.arange(n) + 1) * 0.004
    for j, tick in enumerate(steps):
        tq = times[j] - 0.055
        # 每拍 integrator_reset = 1 也没把参考对齐掉：延后参考沿着设计轨迹走。
        assert des_p[tick] == pytest.approx(delayed(py_p[:, 0], times, 0.0, tq), abs=2e-6)
        assert des_r[tick] == pytest.approx(delayed(py_r[:, 0], times, 0.0, tq), abs=2e-6)
        # 角度环前馈 = 延后的 θ̇_ref（姿态为 0，期望系到机体系的旋转不改 y 分量）。
        assert wff_p[tick] == pytest.approx(delayed(py_p[:, 1], times, 0.0, tq), abs=2e-5)
        # 力矩前馈 = ff × I × θ̈_ref(t)，不延后。
        assert ff_p[tick] == pytest.approx(iyy * py_p[j, 2], rel=1e-5, abs=1e-7)
        assert ff_r[tick] == pytest.approx(iyy * py_r[j, 2], rel=1e-5, abs=1e-7)
    # 姿态拍之间保持（α_ff 与参考都不推进）。
    for tick in range(len(rows)):
        if att_upd[tick] == 0:
            last = steps[steps < tick].max()
            assert des_p[tick] == des_p[last] and ff_p[tick] == ff_p[last]
    # ω_sp = 前馈 − att_kp·e_R：实际姿态为单位阵时 SO(3) 误差的 y 分量是
    # −sinθ_d·(1 + cosφ_d)/2（θ_d、φ_d 为延后参考），反馈看的是延后参考而不是指令。
    att_kp = 1.7131   # 俯仰默认角度环（2026-09-28 晚 A1 基线）
    later = steps[steps > 20]
    expect = wff_p[later] + att_kp * np.sin(des_p[later]) * (1 + np.cos(des_r[later])) / 2
    assert np.allclose(wsp_p[later], expect, atol=2e-6)
    # 偏航的力矩前馈不动（参考模型只管横滚/俯仰）；偏航角速度前馈只有期望系倾斜带来的
    # 二阶耦合（R_d·ω_d 的 z 分量），量级远小于俯仰。
    assert np.all(ff_y == 0.0)
    assert np.max(np.abs(wff_y)) < 0.05 * np.max(np.abs(wff_p))
    assert np.abs(ff_p).max() > 1e-3 and np.abs(ff_r).max() > 1e-3


def test_disabled_reference_leaves_targets_and_feedforward_untouched(ctrl):
    rows = _shape_rows(ctrl, wr=0.0)
    # desired 由期望旋转矩阵反解欧拉角得到，只差舍入。
    assert np.max(np.abs(rows[:, 5] - 0.05)) < 1e-6 and np.max(np.abs(rows[:, 4] + 0.03)) < 1e-6
    assert np.all(rows[:, 6] == 0.0) and np.all(rows[:, 7] == 0.0)
    assert np.all(rows[:, 10] == 0.0) and np.all(rows[:, 11] == 0.0)


def test_reference_aligns_on_reset_and_source_switch_without_jumping(ctrl):
    rows = _rows(_run(ctrl, "align"))
    desired = rows[:, 2]
    # 起步：延后参考就是当前实测角，角度环误差 0，不会猛打一下。
    assert np.all(np.abs(desired[:50] - 0.1) < 1e-6)
    # 每拍 integrator_reset（直接姿态模式）不对齐：参考沿设计轨迹从 0.1 走向目标 0。
    py = reference_steps(8.0, 0.0, 0.004, 75, start=float(np.float32(0.1)))
    times = (np.arange(75) + 1) * 0.004
    for tick in range(0, 300, 4):
        expect = delayed(py[:, 0], times, float(np.float32(0.1)), times[tick // 4] - 0.055)
        assert desired[tick] == pytest.approx(expect, abs=2e-6), tick
    assert desired[296] < 0.05
    assert np.max(np.abs(np.diff(desired[:300]))) < 2e-3
    # ResetState 之后、从力矢量切回直接姿态之后，都从当时的实测角起步。
    for start, value in ((300, -0.05), (900, 0.03)):
        assert np.all(np.abs(desired[start:start + 50] - value) < 1e-6), start
    # 力矢量（位置/速度环）来源不过参考模型：期望姿态就是原始目标，不从实测角慢慢走
    # （2026-10-01 自由飞定点 2.4 s 振荡，参考模型在速度环里多约 0.22 s 滞后）。
    target = rows[:, 3]
    assert np.array_equal(desired[600:900], target[600:900])
    assert np.all(np.abs(desired[600:650] - (-0.02)) > 1e-3)


@pytest.mark.parametrize("reset_each_tick", [1, 0])
def test_output_notch_is_wired_on_the_feedback_and_survives_per_tick_resets(ctrl, reset_each_tick):
    """直接姿态模式每拍 integrator_reset：陷波状态必须留着，否则它悄悄变成直通。"""
    rows = _rows(_run(ctrl, "notchwire", 6.0, 6.0, reset_each_tick))
    fb, ff, moment = rows.T
    assert np.all(ff == 0.0)
    b, a = notch_biquad(6.0, 1.2, 0.1, 500.0)
    # float 系数下的深陷波对舍入敏感：按输入幅值的 1e-4 比。
    assert np.max(np.abs(moment - df1_primed(b, a, fb))) < 1e-4 * np.max(np.abs(fb))
    t = np.arange(len(fb)) * 0.002
    tail = t > 3.0
    gain, _ = _fit_amplitude_phase(t[tail], fb[tail], moment[tail], 6.0)
    assert 20 * math.log10(gain) == pytest.approx(-20.0, abs=0.2)


def test_output_notch_off_passes_the_feedback_straight_through(ctrl):
    rows = _rows(_run(ctrl, "notchwire", 0.0, 6.0, 1))
    fb, _, moment = rows.T
    assert np.array_equal(fb, moment)


@pytest.mark.parametrize("notch1", [0.0, 6.0])
def test_second_output_notch_is_wired_in_series_after_the_first(ctrl, notch1):
    """飞控真实节拍（速率 500 Hz）：第二级 16 Hz 接在第一级之后；16 Hz 反馈约 −20 dB（再乘第一级
    在 16 Hz 的增益）；直接姿态模式每拍 integrator_reset 也不清它。"""
    rows = _rows(_run(ctrl, "notchwire", notch1, 16.0, 1, 16.0, 0.0))
    fb, ff, moment = rows.T
    assert np.all(ff == 0.0)
    b2, a2 = notch_biquad(16.0, 1.0, 0.1, 500.0)
    if notch1 > 0.0:
        b1, a1 = notch_biquad(notch1, 1.2, 0.1, 500.0)
        expect = df1_primed(b2, a2, df1_primed(b1, a1, fb))
        first_db = 20 * math.log10(_digital_gain(b1, a1, 16.0, 0.002))
    else:
        expect = df1_primed(b2, a2, fb)
        first_db = 0.0
    assert np.max(np.abs(moment - expect)) < 1e-4 * np.max(np.abs(fb))
    t = np.arange(len(fb)) * 0.002
    tail = t > 3.0
    gain, _ = _fit_amplitude_phase(t[tail], fb[tail], moment[tail], 16.0)
    assert 20 * math.log10(gain) == pytest.approx(-20.0 + first_db, abs=0.3)


def test_second_output_notch_leaves_the_reference_feedforward_untouched(ctrl):
    """参考模型开着（力矩前馈非零）：出口 = 两级串联滤过的反馈 + 原样的前馈。"""
    rows = _rows(_run(ctrl, "notchwire", 6.0, 16.0, 1, 16.0, 8.0))
    fb, ff, moment = rows.T
    assert np.abs(ff).max() > 1e-3
    b1, a1 = notch_biquad(6.0, 1.2, 0.1, 500.0)
    b2, a2 = notch_biquad(16.0, 1.0, 0.1, 500.0)
    expect = df1_primed(b2, a2, df1_primed(b1, a1, fb)) + ff
    assert np.max(np.abs(moment - expect)) < 1e-4 * max(np.abs(fb).max(), np.abs(ff).max())


# ------------------------------------------------------------------------------- 闭环


ROBUST = dict(kp=0.240, ki=0.288, att=0.25 * 2 * math.pi * 0.97)


def _closed_loop(ctrl, model, *, mode, wr=8.0, td=55.0, notch=6.0, q=1.2, gains=ROBUST,
                 seconds=3.0, force=11.235):
    phi, gam = discrete_plant(model)
    lag = int(round(model["T"] / 0.001))
    args = ["closed", mode, gains["kp"], gains["ki"], gains["att"], wr, td, notch, q,
            model["I"], math.radians(3.0), int(seconds / 0.001), lag, force,
            *[repr(float(v)) for v in phi.flatten()], *[repr(float(v)) for v in gam]]
    rows = _rows(_run(ctrl, *args))
    return rows[:, 0], rows[:, 1]


def test_python_replica_reproduces_the_design_script_numbers():
    """设计脚本（2026-09-28 实跑）：ωr 8 + 前馈 + 延后 55 ms，悬停名义 90% 0.521 s、超调 ~0%，
    最坏角点 90% 0.520 s、超调 4.93%。numpy 复刻必须给出同样的数，否则下面的对拍没有意义。"""
    _, hover, worst = plant_models()
    s = step_metrics(design_sim(hover, ROBUST, notch=(6.0, 1.2), ref_wr=8.0, ref_delay=0.055))
    assert s["t90"] == pytest.approx(0.521, abs=1e-3) and s["over"] < 0.01
    s = step_metrics(design_sim(worst, ROBUST, notch=(6.0, 1.2), ref_wr=8.0, ref_delay=0.055))
    assert s["t90"] == pytest.approx(0.520, abs=1e-3) and s["over"] == pytest.approx(4.93, abs=0.01)


@pytest.mark.parametrize("which", ["hover", "worst"])
def test_real_controller_tracks_the_design_simulation_at_1khz(ctrl, which):
    """同拍（1 kHz）对拍：真实 C 控制器 + 同一对象，θ 轨迹与设计脚本同构仿真重合。"""
    _, hover, worst = plant_models()
    model = hover if which == "hover" else worst
    theta, _ = _closed_loop(ctrl, model, mode=0)
    # 前馈惯量：固件是 ff × I_yy（机体模型），脚本是 I/κ；这里按固件口径对拍。
    py = design_sim(model, ROBUST, notch=(6.0, 1.2), ref_wr=8.0, ref_delay=0.055,
                    ff_inertia=model["I"])
    print(which, "max |dtheta|", float(np.max(np.abs(theta - py))))
    assert np.max(np.abs(theta - py)) < 5e-5


@pytest.mark.parametrize("which", ["hover", "worst"])
def test_real_flight_schedule_step_meets_the_design_targets(ctrl, which):
    """飞控真实节拍 + 舵机量化后的实际力矩：3° 阶跃 90% < 0.7 s、超调 < 8%。"""
    _, hover, worst = plant_models()
    model = hover if which == "hover" else worst
    theta, u = _closed_loop(ctrl, model, mode=1)
    metrics = step_metrics(theta)
    print(which, metrics, "peak_u", float(np.abs(u).max()))
    assert metrics["t90"] < 0.7
    assert metrics["over"] < 8.0
    py = step_metrics(design_sim(model, ROBUST, notch=(6.0, 1.2), ref_wr=8.0, ref_delay=0.055))
    assert metrics["t90"] == pytest.approx(py["t90"], abs=0.08)
    assert metrics["over"] == pytest.approx(py["over"], abs=3.0)


# ------------------------------------------------------------------------- 参数与持久化


def test_parameters_reject_negative_nonfinite_and_out_of_range(ctrl):
    lines = _run(ctrl, "params")
    accepted = {}
    for line in lines[:-2]:
        name, value, ok, before, after = line.split()
        accepted[(name, value)] = ok == "1"
        if ok == "1":
            assert float(after) == pytest.approx(float(value)), line
        else:
            assert after == before, f"被拒的值不许写进去：{line}"
    expect_ok = {
        ("coax.rate_out_notch_hz", "0"), ("coax.rate_out_notch_hz", "1"),
        ("coax.rate_out_notch_hz", "6"), ("coax.rate_out_notch_hz", "100"),
        ("coax.rate_out_notch_q", "0.300000012"), ("coax.rate_out_notch_q", "10"),
        # 第二级与第一级同一范围：0 = 关，否则 [1, 100] Hz；Q [0.3, 10]。
        ("coax.rate_out_notch2_hz", "0"), ("coax.rate_out_notch2_hz", "1"),
        ("coax.rate_out_notch2_hz", "16"), ("coax.rate_out_notch2_hz", "100"),
        ("coax.rate_out_notch2_q", "0.300000012"), ("coax.rate_out_notch2_q", "10"),
        ("coax.att_ref_wr_rad_s", "0"), ("coax.att_ref_wr_rad_s", "0.5"),
        ("coax.att_ref_wr_rad_s", "30"),
        ("coax.att_ref_delay_ms", "0"), ("coax.att_ref_delay_ms", "80"),
    }
    assert {key for key, ok in accepted.items() if ok} == expect_ok
    assert lines[-2] == "bulk2 100 30", "第二级是坏值时整份拒收（wr 也不许顺带写进去）"
    assert lines[-1] == "bulk 30", "整份写入里有坏值必须整份拒收（保持上一个合法值 30）"


def test_defaults_keep_every_switch_off(ctrl):
    """默认全关（这次改动烧进去之前不改变任何飞行行为）；Q 与 Td 按 A1 预置（Q1.2、Td 45 ms），
    第二级 Q 预置 1.0。"""
    source = CTRL_SOURCE.read_text(encoding="utf-8")
    assert "params->rate_out_notch_hz = 0.0f;" in source
    assert "params->rate_out_notch2_hz = 0.0f;" in source
    assert "params->att_ref_wr_rad_s = 0.0f;" in source
    assert "params->rate_out_notch_q = 1.2f;" in source
    assert "params->rate_out_notch2_q = 1.0f;" in source
    assert "params->att_ref_delay_ms = 45.0f;" in source
    # ff 倍率：力矩单位已是真实 N·m，俯仰 1；横滚 Ixx 未辨识取 0（2026-09-28 作者指定的现行最优）。
    assert "params->rate.ff_gain[0] = 1.0f;" in source
    assert "params->rate.ff_gain[1] = 1.0f;" in source


STORE_HARNESS = r"""
#include "app_control_config_store.h"
#include "app_magxy.h"
#include "drv_coax_ctrl.h"

extern void fake_flash_reset(void);

#include <stddef.h>

static float get(const char *name)
{
    float v = -1.0f;
    CHECK(DRV_COAX_CTRL_GetParam(name, &v) != 0U, 900);
    return v;
}

static void put(const char *name, float value)
{
    CHECK(DRV_COAX_CTRL_SetParam(name, value) != 0U, 901);
}

/* 与 app_control_config_store.c 的 config_checksum 同一算法（旧记录要自己拼）。 */
static uint32_t checksum(const uint8_t *data, uint32_t length)
{
    uint32_t sum = 0xA5A55A5AUL;
    for (uint32_t i = 0U; i < length; ++i) {
        sum = (sum << 5U) | (sum >> 27U);
        sum ^= data[i];
        sum += 0x9E3779B9UL;
    }
    return sum;
}

static uint8_t current[16384];
static uint8_t saved[16384];
static uint8_t forged[16384];

/* 把 forged 的前 length 字节当成一条"没有提交字的旧格式记录"放进槽 A，槽 B 清空。 */
static void plant(uint32_t length)
{
    CHECK(APP_FlashService_EraseSector(APP_CONTROL_CFG_SLOT_A) == APP_FLASH_SERVICE_OK, 910);
    CHECK(APP_FlashService_EraseSector(APP_CONTROL_CFG_SLOT_B) == APP_FLASH_SERVICE_OK, 911);
    CHECK(APP_FlashService_WriteData(APP_CONTROL_CFG_SLOT_A, forged, length) ==
          APP_FLASH_SERVICE_OK, 912);
}

int main(void)
{
    APP_ControlConfig cfg, loaded;
    uint16_t size27, size26, size25, size24, size23;
    uint32_t total27, total25;
    const uint32_t shaping_size = (uint32_t)sizeof(APP_ControlCoaxShapingParams);
    const uint32_t shaping_v24_size = (uint32_t)sizeof(APP_ControlCoaxShapingParamsV24);

    DRV_COAX_CTRL_Init();
    fake_flash_reset();
    memset(&cfg, 0, sizeof(cfg));

    /* 默认全关；Q 与 Td 按 A1 预置，第二级 Q 1.0。 */
    CHECK(get("coax.rate_out_notch_hz") == 0.0f, 1);
    CHECK(get("coax.att_ref_wr_rad_s") == 0.0f, 2);
    CHECK(get("coax.rate_out_notch_q") == 1.2f, 3);
    CHECK(get("coax.att_ref_delay_ms") == 45.0f, 4);
    CHECK(shaping_size == 24U && shaping_v24_size == 16U, 5);
    CHECK(get("coax.rate_out_notch2_hz") == 0.0f, 6);
    CHECK(get("coax.rate_out_notch2_q") == 1.0f, 7);
    /* v24 的四个字段在 v25 块里原位不动：v24 块就是 v25 块的前缀。 */
    CHECK(offsetof(APP_ControlCoaxShapingParams, att_ref_delay_ms) ==
          offsetof(APP_ControlCoaxShapingParamsV24, att_ref_delay_ms), 8);
    CHECK(offsetof(APP_ControlCoaxShapingParams, rate_out_notch2_hz) == shaping_v24_size, 9);

    /* v25 往返：存进去的六个值原样读回，增益块不受影响。 */
    put("coax.rate_out_notch_hz", 6.0f);
    put("coax.rate_out_notch_q", 1.5f);
    put("coax.att_ref_wr_rad_s", 8.0f);
    put("coax.att_ref_delay_ms", 50.0f);
    put("coax.rate_out_notch2_hz", 16.0f);
    put("coax.rate_out_notch2_q", 0.9f);
    put("coax.rate_pitch_kp", 0.24f);
    CHECK(APP_ControlConfigStore_Save(&cfg) == APP_FLASH_SERVICE_OK, 10);
    put("coax.rate_out_notch_hz", 0.0f);
    put("coax.rate_out_notch_q", 1.2f);
    put("coax.att_ref_wr_rad_s", 0.0f);
    put("coax.att_ref_delay_ms", 45.0f);
    put("coax.rate_out_notch2_hz", 0.0f);
    put("coax.rate_out_notch2_q", 1.0f);
    put("coax.rate_pitch_kp", 0.3f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 11);
    CHECK(get("coax.rate_out_notch_hz") == 6.0f, 12);
    CHECK(get("coax.rate_out_notch_q") == 1.5f, 13);
    CHECK(get("coax.att_ref_wr_rad_s") == 8.0f, 14);
    CHECK(get("coax.att_ref_delay_ms") == 50.0f, 15);
    CHECK(get("coax.rate_pitch_kp") == 0.24f, 16);
    CHECK(get("coax.rate_out_notch2_hz") == 16.0f, 17);
    CHECK(get("coax.rate_out_notch2_q") == 0.9f, 18);

    /* 刚存的那条在槽 B（两个槽都空时活动槽是 A，写对面）。 */
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, current, 8U) == APP_FLASH_SERVICE_OK, 20);
    memcpy(&size27, &current[6], sizeof(size27));
    total27 = 8U + size27 + 4U;
    CHECK(total27 <= sizeof(current), 21);
    CHECK(APP_FlashService_ReadData(APP_CONTROL_CFG_SLOT_B, current, total27) ==
          APP_FLASH_SERVICE_OK, 22);
    CHECK(current[4] == APP_CONTROL_CFG_VERSION && current[5] == 0U, 23);   /* 当前版本（v28） */

    /*
     * 2026-09-29（R-FLOWMOUNT-1）：v26 在机体块尾部追加了光流安装两项（8 字节）。下面这些
     * 旧记录仍按各自当年的字节布局拼，所以先把那 8 字节去掉，得到一条 v25 记录（整形块
     * 仍在记录尾部，下面"去掉尾部字节"的拼法照旧成立）。
     */
    {
        const uint32_t af_tail = 8U + (uint32_t)sizeof(APP_ControlConfig) +
                                 (uint32_t)sizeof(APP_ControlCoaxTunableParams) +
                                 (uint32_t)sizeof(APP_RcConfig) +
                                 (uint32_t)sizeof(APP_ControlAirframeParamsV25);
        size26 = (uint16_t)(size27 - sizeof(APP_MagXY_Persisted) -
                            sizeof(APP_ControlZChannelParams) -
                            sizeof(APP_ControlFlightLimitParams));   /* 去掉 v27 XY、v28 竖直通道块、v29 飞行限幅块 */
        size25 = (uint16_t)(size26 - 8U);
        memcpy(saved, current, af_tail);
        memcpy(&saved[af_tail], &current[af_tail + 8U], size26 - af_tail);
        saved[4] = 25U;
        saved[5] = 0U;
        memcpy(&saved[6], &size25, sizeof(size25));
        {
            const uint32_t sum = checksum(&saved[8], size25);
            memcpy(&saved[8U + size25], &sum, sizeof(sum));
        }
        total25 = 8U + size25 + 4U;
    }

    /* 拼一条 v24 记录：同样的字节去掉整形块尾部的第二级，版本 24，重算校验和。 */
    size24 = (uint16_t)(size25 - (shaping_size - shaping_v24_size));
    memcpy(forged, saved, 8U + size24);
    forged[4] = 24U;
    forged[5] = 0U;
    memcpy(&forged[6], &size24, sizeof(size24));
    {
        const uint32_t sum = checksum(&forged[8], size24);
        memcpy(&forged[8U + size24], &sum, sizeof(sum));
    }
    plant(8U + size24 + 4U);
    put("coax.rate_out_notch2_hz", 17.0f);   /* RAM 里留着别的值：读旧记录必须显式落回默认 */
    put("coax.rate_out_notch2_q", 2.0f);
    put("coax.rate_out_notch_hz", 7.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 25);
    CHECK(get("coax.rate_out_notch_hz") == 6.0f, 26);   /* v24 的四项照读 */
    CHECK(get("coax.rate_out_notch_q") == 1.5f, 27);
    CHECK(get("coax.att_ref_wr_rad_s") == 8.0f, 28);
    CHECK(get("coax.att_ref_delay_ms") == 50.0f, 29);
    CHECK(get("coax.rate_out_notch2_hz") == 0.0f, 30);  /* 第二级落回默认：关、Q 1.0 */
    CHECK(get("coax.rate_out_notch2_q") == 1.0f, 31);
    CHECK(get("coax.rate_pitch_kp") == 0.24f, 32);

    /* 拼一条 v23 记录：同样的字节去掉整个整形块，版本 23，重算校验和。 */
    size23 = (uint16_t)(size25 - shaping_size);
    memcpy(forged, saved, 8U + size23);
    forged[4] = 23U;
    forged[5] = 0U;
    memcpy(&forged[6], &size23, sizeof(size23));
    {
        const uint32_t sum = checksum(&forged[8], size23);
        memcpy(&forged[8U + size23], &sum, sizeof(sum));
    }
    plant(8U + size23 + 4U);
    put("coax.rate_out_notch_hz", 7.0f);   /* RAM 里留着别的值：读旧记录必须显式落回默认 */
    put("coax.att_ref_wr_rad_s", 5.0f);
    put("coax.rate_out_notch2_hz", 17.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 33);
    CHECK(get("coax.rate_out_notch_hz") == 0.0f, 34);
    CHECK(get("coax.att_ref_wr_rad_s") == 0.0f, 35);
    CHECK(get("coax.rate_out_notch_q") == 1.2f, 36);
    CHECK(get("coax.att_ref_delay_ms") == 45.0f, 37);
    CHECK(get("coax.rate_out_notch2_hz") == 0.0f, 38);
    CHECK(get("coax.rate_out_notch2_q") == 1.0f, 39);
    CHECK(get("coax.rate_pitch_kp") == 0.24f, 40);   /* 旧记录里的增益照常读回 */

    /* v25 记录里整形块是坏值（校验和对）：整形整块落回"关"，其余照常。 */
    memcpy(forged, saved, total25);
    {
        const float bad = -3.0f;
        uint32_t sum;
        memcpy(&forged[8U + size23], &bad, sizeof(bad));
        sum = checksum(&forged[8], size25);
        memcpy(&forged[8U + size25], &sum, sizeof(sum));
    }
    plant(total25);
    put("coax.rate_out_notch_hz", 6.0f);
    put("coax.att_ref_wr_rad_s", 8.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 41);
    CHECK(get("coax.rate_out_notch_hz") == 0.0f, 42);
    CHECK(get("coax.att_ref_wr_rad_s") == 0.0f, 43);
    CHECK(get("coax.rate_out_notch2_hz") == 0.0f, 44);
    CHECK(get("coax.rate_pitch_kp") == 0.24f, 45);

    /* 只有第二级是坏值（0.5 Hz 不在 [1, 100]）：同样整块拒收，不会只装前四项。 */
    memcpy(forged, saved, total25);
    {
        const float bad = 0.5f;
        uint32_t sum;
        memcpy(&forged[8U + size24], &bad, sizeof(bad));
        sum = checksum(&forged[8], size25);
        memcpy(&forged[8U + size25], &sum, sizeof(sum));
    }
    plant(total25);
    put("coax.rate_out_notch_hz", 7.0f);
    put("coax.rate_out_notch2_hz", 17.0f);
    CHECK(APP_ControlConfigStore_Load(&loaded) == APP_FLASH_SERVICE_OK, 46);
    CHECK(get("coax.rate_out_notch_hz") == 0.0f, 47);
    CHECK(get("coax.att_ref_wr_rad_s") == 0.0f, 48);
    CHECK(get("coax.rate_out_notch2_hz") == 0.0f, 49);
    CHECK(get("coax.rate_pitch_kp") == 0.24f, 50);
    REPORT();
}
"""


def test_shaping_block_roundtrips_and_old_records_load_as_off(tmp_path):
    import test_config_store_ab_slots as ab
    from _micoair_hostfakes import CHECK_MACRO, build_and_run, write_fakes

    fakes = write_fakes(tmp_path, {"cmsis_os2.h": ab.FAKE_CMSIS_OS2_H, "main.h": ab.FAKE_MAIN_H})
    result = build_and_run(tmp_path, "shaping_store",
                           CHECK_MACRO + ab.FAKE_FLASH_C + ab.HOST_STUBS_C + STORE_HARNESS,
                           sources=ab.SOURCES, includes=ab.includes(fakes))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout


def test_host_capabilities_mirror_the_firmware_ranges():
    from tools.panel_lib import parameter_model as pm

    notch = (DRIVER_INC / "drv_moment_notch.h").read_text(encoding="utf-8")
    ref = (DRIVER_INC / "drv_att_reference.h").read_text(encoding="utf-8")

    def macro(text, name):
        return float(re.search(rf"#define\s+{name}\s+([0-9.]+)f", text).group(1))

    assert pm.COAX_SHAPING_RANGES == {
        "rate_out_notch_hz": (0.0, macro(notch, "DRV_MOMENT_NOTCH_HZ_MAX")),
        "rate_out_notch_q": (macro(notch, "DRV_MOMENT_NOTCH_Q_MIN"), macro(notch, "DRV_MOMENT_NOTCH_Q_MAX")),
        "rate_out_notch2_hz": (0.0, macro(notch, "DRV_MOMENT_NOTCH_HZ_MAX")),
        "rate_out_notch2_q": (macro(notch, "DRV_MOMENT_NOTCH_Q_MIN"), macro(notch, "DRV_MOMENT_NOTCH_Q_MAX")),
        "att_ref_wr_rad_s": (0.0, macro(ref, "DRV_ATT_REF_WR_MAX_RAD_S")),
        "att_ref_delay_ms": (0.0, macro(ref, "DRV_ATT_REF_DELAY_MAX_MS")),
    }
    for name, value, ok in (("coax.rate_out_notch_hz", "6", True), ("coax.rate_out_notch_hz", "0", True),
                            ("coax.rate_out_notch_hz", "101", False), ("coax.rate_out_notch_hz", "-1", False),
                            ("coax.rate_out_notch_q", "0.2", False), ("coax.rate_out_notch_q", "1.2", True),
                            ("coax.rate_out_notch2_hz", "16", True), ("coax.rate_out_notch2_hz", "0", True),
                            ("coax.rate_out_notch2_hz", "101", False), ("coax.rate_out_notch2_q", "1", True),
                            ("coax.rate_out_notch2_q", "0.2", False), ("coax.rate_out_notch2_q", "11", False),
                            ("coax.att_ref_wr_rad_s", "8", True), ("coax.att_ref_wr_rad_s", "31", False),
                            ("coax.att_ref_delay_ms", "55", True), ("coax.att_ref_delay_ms", "81", False),
                            ("coax.att_ref_delay_ms", "nan", False)):
        assert pm.validate_parameter_text(name, value)[0] is ok, (name, value)


# ------------------------------------------------------------------ 光杆辨识走同一条路


@pytest.fixture(scope="module")
def sysid_lib(tmp_path_factory):
    import test_sysid_runtime_contract as rc

    lib = rc.build_lib(tmp_path_factory, name="sysid-shaping")
    rc.configure_api(lib)
    return lib


def _sysid_run(lib, mode, *, wr=0.0, notch=0.0, notch2=0.0, amplitude=0.02):
    """跑一整轮光杆闭环验证（手动油门、θ = 0、陀螺 0，所以控制器输出只由指令决定），
    返回解码后的样本与开跑溯源行。双脉冲每段 150 ms，整轮 600 ms。"""
    import test_sysid_runtime_contract as rc
    from sysid.decode import decode_batch, parse_schema_lines

    spec = rc.Excitation(profile=rc.PROFILE_DOUBLET, amplitude_rad_s=0.1, duration_ms=600,
                         hold_ms=150, repeat=2, ramp_ms=10, chirp_f0_hz=0.3, chirp_f1_hz=6.0,
                         prbs_bit_ms=40, prbs_seed=1)
    rc.reset(lib, rate_hz=500, spec=spec)
    lib.APP_SysId_ReportSchema()
    schema = parse_schema_lines(rc.texts(lib))
    assert lib.DRV_COAX_CTRL_SetParam(b"coax.att_ref_wr_rad_s", wr)
    assert lib.DRV_COAX_CTRL_SetParam(b"coax.rate_out_notch_hz", notch)
    assert lib.DRV_COAX_CTRL_SetParam(b"coax.rate_out_notch2_hz", notch2)
    samples = []
    try:
        assert lib.APP_SysId_SetMode(mode, amplitude)
        lib.harness_reset()
        assert lib.APP_SysId_Start()
        start = [t for t in rc.texts(lib) if t.startswith("SYSID start ")]
        ran = rc.run_ticks(lib, 1000, mutate=lambda i, _obs: (
            lib.APP_SysId_StreamTick() if i % 8 == 0 else None))
        assert lib.APP_SysId_GetLastReason() == b"complete", lib.APP_SysId_GetLastReason()
        for _ in range(64):
            lib.APP_SysId_StreamTick()
        for _function, payload in rc.frames(lib):
            samples.extend(decode_batch(payload, schema).samples)
        assert lib.APP_SysId_GetDroppedSamples() == 0
    finally:
        lib.APP_SysId_Stop(b"test")
        lib.APP_SysId_StreamTick()
        assert lib.APP_SysId_SetMode(0, 0.0523598776)
        assert lib.DRV_COAX_CTRL_SetParam(b"coax.att_ref_wr_rad_s", 0.0)
        assert lib.DRV_COAX_CTRL_SetParam(b"coax.rate_out_notch_hz", 0.0)
        assert lib.DRV_COAX_CTRL_SetParam(b"coax.rate_out_notch2_hz", 0.0)
    assert len(samples) >= ran - 2 and ran > 250
    return samples, start


def reference_track(wr, commands, dt, start=0.0):
    """与 reference_steps 同一递推，指令逐拍变化。"""
    r, rd = start, 0.0
    out = np.empty((len(commands), 3))
    for k, target in enumerate(commands):
        rdd = wr ** 2 * (target - r) - 2 * wr * rd
        rd += rdd * dt
        r += rd * dt
        out[k] = (r, rd, rdd)
    return out


def test_sysid_angle_mode_uses_the_flight_reference_model(sysid_lib):
    # 杆轴 45°：投影后的角度环增益 = (横滚 + 俯仰)/2，取的是控制器当前的 att_kp。本例显式给两轴
    # 不同的值（不依赖默认值是否两轴相同），跑完还原。
    gains = {b"coax.att_roll_kp": 1.375, b"coax.att_pitch_kp": 1.524}
    previous = {}
    for name, value in gains.items():
        old = ctypes.c_float()
        assert sysid_lib.DRV_COAX_CTRL_GetParam(name, ctypes.byref(old))
        previous[name] = old.value
        assert sysid_lib.DRV_COAX_CTRL_SetParam(name, value)
    try:
        off, start_off = _sysid_run(sysid_lib, 2, wr=0.0)
        on, start_on = _sysid_run(sysid_lib, 2, wr=8.0)
    finally:
        for name, value in previous.items():
            assert sysid_lib.DRV_COAX_CTRL_SetParam(name, value)
    n = min(len(on), len(off))
    angle_sp = np.array([s["angle_sp"] for s in on[:n]])
    # 记录的 angle_sp 始终是指令本身，与参考模型开不开无关。
    assert np.array_equal(angle_sp, np.array([s["angle_sp"] for s in off[:n]]))
    assert np.abs(angle_sp).max() > 0.015
    k_att = (gains[b"coax.att_roll_kp"] + gains[b"coax.att_pitch_kp"]) / 2
    omega_off = np.array([s["omega_sp"] for s in off[:n]])
    omega_on = np.array([s["omega_sp"] for s in on[:n]])
    assert np.max(np.abs(omega_off - k_att * np.sin(angle_sp))) < 1.5e-3
    # 开着：ω_sp = θ̇_ref(t−Td) + k·sin θ_ref(t−Td)，参考是在飞同一个模型（500 Hz、默认 Td 45 ms）。
    dt = 0.002
    track = reference_track(8.0, angle_sp, dt)
    times = (np.arange(n) + 1) * dt
    r_d = np.array([delayed(track[:, 0], times, 0.0, t - 0.045) for t in times])
    rd_d = np.array([delayed(track[:, 1], times, 0.0, t - 0.045) for t in times])
    assert np.max(np.abs(omega_on - (rd_d + k_att * np.sin(r_d)))) < 2e-3
    assert np.max(np.abs(omega_on - omega_off)) > 0.01
    # 开跑溯源行带上本轮整形配置（上位机整行存进 conditions.json）。
    assert ("ref_wr_mrad_s=8000 ref_td_us=45000 onotch_mhz=0 onotch_q_milli=1200 "
            "onotch2_mhz=0 onotch2_q_milli=1000\r\n") in start_on[0] + "\r\n"
    assert "ref_wr_mrad_s=0 " in start_off[0]


def test_sysid_rate_mode_applies_the_same_output_notch(sysid_lib):
    off, _ = _sysid_run(sysid_lib, 1, notch=0.0)
    on, start = _sysid_run(sysid_lib, 1, notch=6.0)
    n = min(len(on), len(off))
    assert [s["omega_sp"] for s in on[:n]] == [s["omega_sp"] for s in off[:n]]
    assert [s["thrust"] for s in on[:n]] == [s["thrust"] for s in off[:n]]
    # 横滚倾转 β 满足 τ_x = L·T·sin β、τ_x = τ_杆·cosψ：推力相同，sin β 与杆轴力矩成正比，
    # 所以开陷波那轮的 sin β 应当正好是关陷波那轮过同一个陷波（500 Hz、首拍稳态起步）。
    s_off = np.sin(np.array([s["tilt_y"] for s in off[:n]]))
    s_on = np.sin(np.array([s["tilt_y"] for s in on[:n]]))
    b, a = notch_biquad(6.0, 1.2, 0.1, 500.0)
    # 最后一条是回中后的输出（激励结束那拍力矩置 0），不参与。
    assert np.max(np.abs(s_on[:-1] - df1_primed(b, a, s_off[:-1]))) < 3e-4
    assert np.max(np.abs(s_on[:-1] - s_off[:-1])) > 3e-3
    assert "onotch_mhz=6000" in start[0]


def test_sysid_rate_mode_chains_the_second_output_notch(sysid_lib):
    """光杆 RATE 轮同样两级串联：sin β 正好是关陷波那轮先过第一级、再过第二级（500 Hz、首拍稳态起步）。"""
    off, _ = _sysid_run(sysid_lib, 1)
    single, _ = _sysid_run(sysid_lib, 1, notch=6.0)
    on, start = _sysid_run(sysid_lib, 1, notch=6.0, notch2=16.0)
    n = min(len(on), len(off), len(single))
    assert [s["omega_sp"] for s in on[:n]] == [s["omega_sp"] for s in off[:n]]
    s_off = np.sin(np.array([s["tilt_y"] for s in off[:n]]))
    s_single = np.sin(np.array([s["tilt_y"] for s in single[:n]]))
    s_on = np.sin(np.array([s["tilt_y"] for s in on[:n]]))
    b1, a1 = notch_biquad(6.0, 1.2, 0.1, 500.0)
    b2, a2 = notch_biquad(16.0, 1.0, 0.1, 500.0)
    # 最后一条是回中后的输出（激励结束那拍力矩置 0），不参与。
    expect = df1_primed(b2, a2, df1_primed(b1, a1, s_off[:-1]))
    assert np.max(np.abs(s_on[:-1] - expect)) < 3e-4
    assert np.max(np.abs(s_on[:-1] - s_single[:-1])) > 1e-3, "第二级确实串进去了"
    assert "onotch_mhz=6000 onotch_q_milli=1200 onotch2_mhz=16000 onotch2_q_milli=1000" in start[0]


def test_sysid_start_line_fits_the_text_buffer_at_its_widest():
    """开跑溯源行加了 onotch2 两项：每个字段取固件允许的最宽写法，整行（含 \\r\\n）仍 ≤ APP_UART_TX_TEXT_SIZE − 1，
    否则 vsnprintf 截掉的正是行尾这几项。"""
    def define(path, name):
        text = (ROOT / path).read_text(encoding="utf-8")
        return float(re.search(rf"#define\s+{name}\s+([0-9.]+)[uUf]?", text).group(1))

    source = (ROOT / "App" / "Src" / "app_sysid.c").read_text(encoding="utf-8")
    call = source[source.index('"SYSID start run='):]
    call = call[:call.index(");")]
    fmt = "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', call)).replace("\\r\\n", "\r\n")
    keys = re.findall(r"(\w+)=%l?[ud]", fmt)
    notch_h = "Driver/Inc/drv_moment_notch.h"
    ref_h = "Driver/Inc/drv_att_reference.h"
    worst = {
        "run": 65535,                                       # uint16_t
        "profile": 3,                                       # DRV_SYSID_PROFILE_COUNT − 1
        "amp_mrad_s": int(define("Driver/Inc/drv_sysid_excitation.h", "DRV_SYSID_EXC_MAX_RATE_RAD_S") * 1000),
        "dur_ms": int(define("Driver/Inc/drv_sysid_excitation.h", "DRV_SYSID_EXC_MAX_DURATION_MS")),
        "rate_hz": int(define("App/Inc/app_sysid.h", "APP_SYSID_RATE_MAX_HZ")),
        "I": 10_000_000,                                    # SYSID INERTIA ≤ 10 kg·m²
        "psi_mrad": -3142,                                  # 杆轴方位 ±π
        "auto": 1,
        "target_cn": 1_000_000,                             # 推力目标 ≤ 机体上限，按 10 kN 放宽
        "ref_wr_mrad_s": int(define(ref_h, "DRV_ATT_REF_WR_MAX_RAD_S") * 1000),
        "ref_td_us": int(define(ref_h, "DRV_ATT_REF_DELAY_MAX_MS") * 1000),
        "onotch_mhz": int(define(notch_h, "DRV_MOMENT_NOTCH_HZ_MAX") * 1000),
        "onotch_q_milli": int(define(notch_h, "DRV_MOMENT_NOTCH_Q_MAX") * 1000),
        "onotch2_mhz": int(define(notch_h, "DRV_MOMENT_NOTCH_HZ_MAX") * 1000),
        "onotch2_q_milli": int(define(notch_h, "DRV_MOMENT_NOTCH_Q_MAX") * 1000),
    }
    assert keys == list(worst), keys
    line = fmt % tuple(worst[k] for k in keys)
    assert line.endswith(" onotch2_mhz=100000 onotch2_q_milli=10000\r\n")
    budget = int(define("App/Inc/app_messages.h", "APP_UART_TX_TEXT_SIZE")) - 1
    assert len(line) <= budget, (len(line), budget)
