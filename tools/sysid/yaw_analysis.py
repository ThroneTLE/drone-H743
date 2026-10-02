"""吊绳偏航辨识 YAW（SYSID MODE YAW）的分析：纯函数，不碰 Tk、不写文件，也不依赖 numpy。

契约见 `doc/sysid-yaw-contract.md`。记录字段表沿用 ALT 追加的 7 个尾字段，YAW 轮里含义变了，
先在这一层清楚地改名（`rename_samples`），页面上不再出现 "height"：

    height -> psi_height 记录里的偏航角（1e-4 rad、int16，±3.2767 rad 会饱和，只作参考）[rad]
    vz -> omega          偏航角速度（原始陀螺 z，未陷波）[rad/s]
    vz_sp -> r_ref       偏航角速度参考 r（diff 为 0）[rad/s]
    az -> delta_t        差速推力 ΔT = T_lower − T_upper（分配器钳位后实际下发的；与 M 同号）[N]
    vbat -> vbat         电池电压 [V]

偏航角 psi **由 omega 按时间积分得到**（梯形法、开跑清零），不用记录里会饱和的 height。
偏航力矩 m_act = k·delta_t（k 取 YAWSTART 的 yaw_k_um_per_n×1e-6；`torque` 字段分辨率 1e-4 N·m 太粗，
只在拿不到 k 时顶替），m_cmd = `torque`（饱和前的指令，只用来判「指令是否超过上限」）。
另有 `thrust`（总推力 F [N]）、`erpm` / `erpm_lower`（上/下桨转速）。

* diff（开环，辨对象）：拟合 ω̇ = b·M(t−τ) − d·ω − s·ψ + c。
  - ω̇ 由 ω 做零相位（中心）滑动平均后中心差分；M、ω、ψ 过同一个滑动平均，保证回归两边口径一致；
    ψ 由 ω 积分得到（记录里的 height 会在 ±187° 饱和），M = k·ΔT（az 是钳位后实际下发的差速）；
  - τ 在 0–150 ms 网格上搜索（2.5 ms 一档，小数延迟线性插值），每个 τ 做 4 参数最小二乘，取残差最小者；
  - b = 1/Izz_有效（rad/s² per N·m，M 是**指令**力矩，所以 b 里已含 k 实测/k 模型的倍数）；
    b·Izz_模型 = k 实测 / k 模型（Izz_模型 与 k 取自 `SYSID YAWSTART`）；
  - 偏航角速度环建议：ω_c = min(1/(4τ), 12 rad/s)，kp = ω_c/b，ki = kp·ω_c/5
    （单位 N·m/(rad/s) 与 N·m/rad，与固件 `coax.rate_yaw_kp/ki` 一致）。
  - 悬停时最大可用偏航力矩 M_max = k·min(F, 2·T单max − F)（F = 悬停推力）；最大角加速度 = b·M_max。
* rate（闭环，验整定）：参考 r = vz_sp，测量 = vz，沿用 XY 的平台切分与阶跃指标
  （上升时间、超调、末值误差、抖动），另报饱和时间占比：|M| 触及 k·min(F, 2·T单max − F) 的样本比例；
  rate 没有参考模型前馈，rate_yaw_ff 在本模式不起作用。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

from .breakaway import read_run_dir
from .xy_analysis import _column, _control_end, _derivative, _resample, analyse_steps

#: 记录字段（schema 名）-> YAW 模式下的名字。
FIELD_RENAMES = {
    "height": "psi_height", "vz": "omega", "vz_sp": "r_ref", "az": "delta_t",
}
#: YAW 分析用得到的（改名后的）字段。
REQUIRED_RENAMED = ("omega", "r_ref", "delta_t", "thrust")

#: 分析网格 [Hz] 与 τ 网格。
GRID_HZ = 200.0
TAU_MAX_S = 0.150
TAU_STEP_S = 0.0025
#: 回归两边共用的零相位滑动平均窗宽 [s]。
SMOOTH_S = 0.100
#: 激励窗：首个有激励的拍往前留、末个有激励的拍往后留的时间 [s]。
WINDOW_LEAD_S = 0.3
WINDOW_TAIL_S = 0.8
#: 力矩指令超过峰值的这个比例才算「有激励」。
ACTIVE_FRACTION = 0.02
#: 角速度环建议：穿越频率上限 [rad/s]，ki = kp·ω_c/KI_DIVISOR。
WC_MAX_RAD_S = 12.0
KI_DIVISOR = 5.0
#: |M| 达到饱和线的这个比例就算饱和。
SATURATION_FRACTION = 0.98
MIN_SAMPLES = 50
MIN_R2 = 0.6
MIN_OMEGA_PEAK_RAD_S = 0.05


# ---------------------------------------------------------------- 改名与小工具


def integrate_psi(times_s, omega) -> list[float]:
    """偏航角 = 角速度对时间的梯形积分（开跑清零）[rad]；缺测点按上一个有效值保持。"""
    psi, previous, last = [0.0], None, 0.0
    for i, value in enumerate(omega):
        number = _finite(value)
        if number is None:
            number = last
        last = number
        if i:
            psi.append(psi[-1] + 0.5 * (previous + number) * (times_s[i] - times_s[i - 1]))
        previous = number
    return psi[:len(omega)]


def rename_samples(samples, times_s=None) -> list[dict]:
    """把 ALT 尾字段名换成 YAW 含义的名字（其余字段照旧）；输入不改动。

    给了 `times_s` 就再补一列 `psi`：陀螺 z 对时间积分的偏航角（不用会饱和的 height）；
    不给时间轴就拿 height 顶替（只供画图的粗看）。"""
    rows = [{FIELD_RENAMES.get(key, key): value for key, value in dict(sample).items()}
            for sample in samples]
    if times_s is not None and len(times_s) == len(rows):
        psi = integrate_psi(times_s, [row.get("omega") for row in rows])
        for row, value in zip(rows, psi):
            row["psi"] = value
    else:
        for row in rows:
            row["psi"] = row.get("psi_height", float("nan"))
    return rows


def with_moment(rows, k: float | None) -> list[dict]:
    """补 `m_act`（实际偏航力矩 k·ΔT）与 `m_cmd`（饱和前的力矩指令 = torque）两列。

    拿不到 k 时 m_act 顶替成 torque 字段（分辨率只有 1e-4 N·m，会在结果里提示）。"""
    out = []
    for row in rows:
        new = dict(row)
        torque = _finite(row.get("torque"))
        delta = _finite(row.get("delta_t"))
        new["m_cmd"] = torque if torque is not None else float("nan")
        if k is not None and delta is not None:
            new["m_act"] = k * delta
        else:
            new["m_act"] = torque if torque is not None else float("nan")
        out.append(new)
    return out


def _finite(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _solve(matrix, vector):
    """高斯消元（列主元）解 A x = b；奇异返回 None。"""
    n = len(vector)
    a = [list(row) + [vector[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[pivot][col]) < 1e-12:
            return None
        a[col], a[pivot] = a[pivot], a[col]
        for row in range(col + 1, n):
            factor = a[row][col] / a[col][col]
            for k in range(col, n + 1):
                a[row][k] -= factor * a[col][k]
    x = [0.0] * n
    for row in range(n - 1, -1, -1):
        x[row] = (a[row][n] - sum(a[row][k] * x[k] for k in range(row + 1, n))) / a[row][row]
    return x


def _lstsq(columns, y):
    """最小二乘 y ≈ Σ coef_j · columns[j]；列按 RMS 归一化后解正规方程。返回 (系数, 残差平方和) 或 None。"""
    n = len(y)
    scales = []
    for col in columns:
        rms = math.sqrt(sum(v * v for v in col) / n)
        scales.append(rms if rms > 1e-12 else 1.0)
    scaled = [[v / s for v in col] for col, s in zip(columns, scales)]
    size = len(columns)
    gram = [[sum(a * b for a, b in zip(scaled[i], scaled[j])) for j in range(size)]
            for i in range(size)]
    rhs = [sum(a * b for a, b in zip(scaled[i], y)) for i in range(size)]
    for i in range(size):
        gram[i][i] += 1e-9 * n          # 极小的岭项，防止共线时奇异
    solution = _solve(gram, rhs)
    if solution is None:
        return None
    coefs = [c / s for c, s in zip(solution, scales)]
    sse = 0.0
    for i in range(n):
        pred = sum(coefs[j] * columns[j][i] for j in range(size))
        sse += (y[i] - pred) ** 2
    return coefs, sse


def _provenance(conditions: dict) -> dict:
    yaw = (conditions or {}).get("yaw")
    return dict(yaw) if isinstance(yaw, dict) else {}


def _number(provenance: dict, key: str, scale: float) -> float | None:
    value = _finite(provenance.get(key))
    return None if value is None else value * scale


def run_inject(conditions: dict) -> str | None:
    """本轮注入类型：页面下发并核对过的请求优先，其次固件溯源。"""
    conditions = conditions or {}
    request = conditions.get("yaw_request")
    if isinstance(request, dict) and request.get("inject"):
        return str(request["inject"])
    yaw = conditions.get("yaw")
    if isinstance(yaw, dict) and yaw.get("yaw_inject"):
        return str(yaw["yaw_inject"])
    return None


def plant_constants(conditions: dict) -> dict:
    """YAWSTART 给的模型量：k [N·m/N]、Izz [kg·m²]、T单max [N]、总推力 [N]、悬停推力 [N]（缺的是 None）。"""
    prov = _provenance(conditions)
    hover = None
    try:
        hover = float(((conditions or {}).get("parameter_echo") or {}).get("coax.hover_thrust_n"))
        if not (math.isfinite(hover) and hover > 0.0):
            hover = None
    except (TypeError, ValueError):
        hover = None
    thrust = _number(prov, "yaw_thrust_mn", 1e-3)
    if hover is None and thrust is not None:
        hover = 2.0 * thrust            # 默认总推力 = 0.5×悬停推力
    return dict(k_m_per_n=_number(prov, "yaw_k_um_per_n", 1e-6),
                izz_model_kg_m2=_number(prov, "yaw_izz_ugm2", 1e-6),
                t_single_max_n=_number(prov, "yaw_single_max_mn", 1e-3),
                thrust_n=thrust, hover_n=hover,
                twist_deg=_finite(prov.get("yaw_twist_deg")))


def yaw_moment_limit(k: float | None, t_single_max: float | None, force_n: float | None):
    """总推力 F 时偏航力矩上限 k·min(F, 2·T单max − F) [N·m]；缺量返回 None。"""
    if k is None or t_single_max is None or force_n is None:
        return None
    return k * max(0.0, min(force_n, 2.0 * t_single_max - force_n))


def saturation_fraction(rows, k: float | None, t_single_max: float | None) -> float | None:
    """激励窗内 |M| 触及饱和线的样本比例；缺 k / T单max 或没有激励返回 None。

    M 取指令（torque，饱和前）与实际（k·ΔT，分配器钳位后）里大的那个：指令超线、或实际差速已顶到线都算饱和。"""
    if k is None or t_single_max is None:
        return None
    command, actual = _column(rows, "m_cmd"), _column(rows, "m_act")
    moment = [max((abs(v) for v in pair if math.isfinite(v)), default=float("nan"))
              for pair in zip(command, actual)]
    thrust = _column(rows, "thrust")
    active = [(m, f) for m, f in zip(moment, thrust) if math.isfinite(m) and math.isfinite(f)]
    peak = max((abs(m) for m, _f in active), default=0.0)
    if peak <= 0.0:
        return None
    counted = saturated = 0
    for m, f in active:
        if abs(m) < ACTIVE_FRACTION * peak:
            continue
        counted += 1
        limit = yaw_moment_limit(k, t_single_max, f)
        if limit is not None and abs(m) >= SATURATION_FRACTION * limit:
            saturated += 1
    return saturated / counted if counted else None


# ---------------------------------------------------------------- diff（开环辨对象）


def _excitation_window(times_s, rows) -> tuple[int, int]:
    """`[起, 止)` 样本下标：力矩指令有激励的区间前后各留一点，并切掉推力回落（RAMP_DOWN）。"""
    moment = _column(rows, "m_act")
    peak = max((abs(v) for v in moment if math.isfinite(v)), default=0.0)
    if peak <= 0.0:
        raise ValueError("力矩全程为 0：这一轮没有激励，没法拟合")
    active = [i for i, v in enumerate(moment) if math.isfinite(v) and abs(v) >= ACTIVE_FRACTION * peak]
    first, last = active[0], active[-1]
    start_t = max(times_s[0], times_s[first] - WINDOW_LEAD_S)
    end_t = times_s[last] + WINDOW_TAIL_S
    start = next(i for i, t in enumerate(times_s) if t >= start_t)
    stop = next((i for i, t in enumerate(times_s) if t > end_t), len(times_s))
    stop = min(stop, _control_end(rows))
    return start, max(stop, start + 1)


def _box(values, half: int):
    """零相位滑动平均：窗 ±half 个网格点（两端按实际可用点数平均）。按点数而不是按秒数，
    免得窗边界落在网格点上时被浮点误差随机多/少算一个点（会让差分出来的角加速度逐点跳动）。"""
    n = len(values)
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    out = []
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        out.append((prefix[hi] - prefix[lo]) / (hi - lo))
    return out


def _delayed(values, lo, hi, lag_samples: float):
    """values[i − lag]（lag 可为小数，线性插值；越过开头取第 0 个）。"""
    whole = int(math.floor(lag_samples))
    frac = lag_samples - whole
    out = []
    for i in range(lo, hi):
        a = values[max(0, i - whole)]
        b = values[max(0, i - whole - 1)]
        out.append(a + frac * (b - a))
    return out


def analyse_diff(times_s, rows, conditions: dict | None = None) -> dict:
    """diff 开环轮：拟合 ω̇ = b·M(t−τ) − d·ω − s·ψ + c，给 b、k 倍数、d、s、τ、R² 与建议增益。"""
    if len(rows) < MIN_SAMPLES:
        raise ValueError(f"样本太少（不足 {MIN_SAMPLES} 条），拟合不了")
    columns = {name: _column(rows, name) for name in ("omega", "psi", "m_act")}
    if not all(all(math.isfinite(v) for v in col) for col in columns.values()):
        raise ValueError("记录里有缺失的角速度或差速（力矩）数据")
    start, stop = _excitation_window(times_s, rows)
    step = 1.0 / GRID_HZ
    grid = [times_s[0] + i * step for i in range(int((times_s[-1] - times_s[0]) / step) + 1)]
    half = max(1, int(round(SMOOTH_S * GRID_HZ / 2.0)))
    omega = _box(_resample(times_s, columns["omega"], grid), half)
    psi = _box(_resample(times_s, columns["psi"], grid), half)
    moment = _box(_resample(times_s, columns["m_act"], grid), half)
    alpha = _derivative(grid, omega)
    lo = next(i for i, t in enumerate(grid) if t >= times_s[start])
    hi = next((i for i, t in enumerate(grid) if t > times_s[min(stop, len(times_s)) - 1]), len(grid))
    if hi - lo < MIN_SAMPLES:
        raise ValueError("激励窗太短，拟合不了")
    y = alpha[lo:hi]
    mean_y = statistics.fmean(y)
    sst = sum((v - mean_y) ** 2 for v in y)
    if sst <= 0.0:
        raise ValueError("角加速度全程不变：电机没转或数据冻结")
    best = None
    steps = int(round(TAU_MAX_S / TAU_STEP_S))
    for lag in range(steps + 1):
        delayed = _delayed(moment, lo, hi, lag * TAU_STEP_S * GRID_HZ)
        cols = [delayed, [-v for v in omega[lo:hi]], [-v for v in psi[lo:hi]], [1.0] * (hi - lo)]
        fit = _lstsq(cols, y)
        if fit is None:
            continue
        coefs, sse = fit
        if best is None or sse < best[0]:
            best = (sse, lag, coefs)
    if best is None:
        raise ValueError("最小二乘矩阵奇异：激励太单一，分不出 b、d、s")
    sse, lag, (b, d, s, c) = best
    tau = lag * TAU_STEP_S
    r2 = 1.0 - sse / sst
    constants = plant_constants(conditions or {})
    k, izz_model = constants["k_m_per_n"], constants["izz_model_kg_m2"]
    t_max, hover = constants["t_single_max_n"], constants["hover_n"]
    omega_raw = columns["omega"][start:stop]
    result: dict = {
        "inject": "diff", "warnings": [], "b": b, "d": d, "s": s, "c": c, "tau_s": tau, "r2": r2,
        "izz_effective_kg_m2": (1.0 / b) if b > 0.0 else None,
        "omega_peak": max((abs(v) for v in omega_raw), default=0.0),
        "psi_peak_rad": max((abs(v) for v in columns["psi"]), default=0.0),
        "moment_peak": max(abs(v) for v in columns["m_act"]),
        "m_source": "k·ΔT" if k is not None else "torque 字段",
        "k_model": k, "izz_model": izz_model, "t_single_max_n": t_max,
        "samples_used": hi - lo,
    }
    if izz_model is not None:
        result["k_ratio"] = b * izz_model               # k 实测 / k 模型
        if k is not None:
            result["k_effective"] = b * izz_model * k
    if b <= 0.0:
        result["warnings"].append("拟合出的 b ≤ 0：偏航极性或陀螺 z 符号与指令相反，或这一轮几乎没有响应；"
                                  "不给建议增益")
    else:
        cap = WC_MAX_RAD_S
        wc = min(1.0 / (4.0 * tau), cap) if tau > 0.0 else cap
        kp = wc / b
        result.update(wc_rad_s=wc, rate_yaw_kp=kp, rate_yaw_ki=kp * wc / KI_DIVISOR)
        limit = yaw_moment_limit(k, t_max, hover)
        if limit is not None:
            result.update(hover_n=hover, moment_max_hover=limit, alpha_max_hover=b * limit)
    result["saturation_fraction"] = saturation_fraction(rows[start:stop], k, t_max)
    if k is None:
        result["warnings"].append("没有 YAWSTART 的 k：力矩只能取 torque 字段（分辨率 1e-4 N·m，偏粗），b 的精度变差")
    if r2 < MIN_R2:
        result["warnings"].append(f"拟合优度只有 {r2:.2f}（< {MIN_R2}）：绳子扭转非线性、摩擦或数据噪声大，"
                                  "b 与建议增益仅供参考；可加大差速幅值或换更长的吊绳再跑一轮")
    if result["omega_peak"] < MIN_OMEGA_PEAK_RAD_S:
        result["warnings"].append(f"偏航角速度峰值只有 {result['omega_peak']:.3f} rad/s：响应太小，"
                                  "信噪比低，b 不可靠，加大差速幅值")
    if lag >= steps:
        result["warnings"].append("纯延迟 τ 贴在搜索上限 150 ms：模型不完整（可能有更慢的动态），τ 与建议增益偏保守或不准")
    if (result["saturation_fraction"] or 0.0) > 0.05:
        result["warnings"].append(f"{result['saturation_fraction'] * 100:.0f}% 的激励拍触及差速饱和线：b 会被低估，"
                                  "减小差速幅值再跑")
    twist = constants["twist_deg"]
    if twist and math.degrees(result["psi_peak_rad"]) > 0.8 * twist:
        result["warnings"].append(f"偏航角用掉了绞绳上限 {twist:g}° 的 80% 以上：下次减小幅值或缩短平台")
    return result


# ---------------------------------------------------------------- rate（闭环验整定）


def analyse_rate(times_s, rows, conditions: dict | None = None) -> dict:
    """rate 闭环轮：偏航角速度阶跃指标 + 饱和时间占比。"""
    if len(rows) < MIN_SAMPLES:
        raise ValueError(f"样本太少（不足 {MIN_SAMPLES} 条），算不了阶跃指标")
    mapped = [{"vel_sp_u": row.get("r_ref"), "vel_u": row.get("omega"), "pos_sp_u": 0.0,
               "thrust": row.get("thrust", 0.0)} for row in rows]
    steps = analyse_steps(times_s, mapped, "vel")
    steps["inject"] = "rate"
    constants = plant_constants(conditions or {})
    steps["saturation_fraction"] = saturation_fraction(rows, constants["k_m_per_n"],
                                                       constants["t_single_max_n"])
    if (steps["saturation_fraction"] or 0.0) > 0.05:
        steps["warnings"].append(f"{steps['saturation_fraction'] * 100:.0f}% 的激励拍触及差速饱和线："
                                 "偏航环被力矩上限卡住，上升时间与超调不代表线性响应")
    steps["notes"] = ["rate 模式没有参考模型前馈，coax.rate_yaw_ff 在本模式不起作用"]
    return steps


# ---------------------------------------------------------------- 入口与文字


def analyse_conditions(times_s, samples, conditions: dict) -> dict:
    """按本轮注入类型分析；`samples` 是按 schema 解出的原始样本（尾字段还叫 height 等）。"""
    inject = run_inject(conditions)
    if inject not in ("diff", "rate"):
        raise ValueError("本轮 YAW 注入类型不明（conditions 里没有 yaw_request / yaw 溯源）")
    rows = rename_samples(samples, times_s)
    if not rows or len(rows) != len(times_s):
        raise ValueError("样本与时间戳对不上")
    missing = [name for name in REQUIRED_RENAMED if name not in rows[0]]
    if missing:
        raise ValueError(f"记录里缺少 YAW 字段：{'、'.join(missing)}")
    k = plant_constants(conditions)["k_m_per_n"]
    rows = with_moment(rows, k)
    if inject == "diff":
        return analyse_diff(times_s, rows, conditions)
    return analyse_rate(times_s, rows, conditions)


def analyse_run_dir(path) -> dict:
    times, samples, conditions = read_run_dir(path)
    result = analyse_conditions(times, samples, conditions)
    end = conditions.get("end") or {}
    result.update(folder=str(path), end_state=end.get("state"), end_reason=end.get("reason"),
                  data_error=conditions.get("data_error") or "")
    return result


def summary_text(result: dict) -> str:
    """结果区的中文摘要。"""
    lines = []
    if result.get("inject") == "diff":
        b = result["b"]
        lines.append(f"偏航对象（ω̇ = b·M(t−τ) − d·ω − s·ψ + c）：拟合优度 R² = {result['r2']:.3f}，"
                     f"纯延迟 τ = {result['tau_s'] * 1000:.0f} ms")
        lines.append(f"b = {b:.1f} rad/s² per N·m"
                     + (f"（有效惯量 Izz = {result['izz_effective_kg_m2']:.5f} kg·m²）"
                        if result.get("izz_effective_kg_m2") else "（≤ 0，不可用）"))
        if result.get("k_ratio") is not None:
            text = f"b × Izz_模型 = {result['k_ratio']:.3f}（= k 实测 / k 模型 的倍数，1 = 模型对）"
            if result.get("k_effective") is not None:
                text += f"，等效 k ≈ {result['k_effective'] * 1e3:.2f} mN·m/N（模型 {result['k_model'] * 1e3:.2f}）"
            lines.append(text)
        else:
            lines.append("没有 YAWSTART 的 Izz_模型：给不出 k 实测 / k 模型 倍数")
        lines.append(f"阻尼 d = {result['d']:.3f} 1/s，扭转刚度项 s = {result['s']:.3f} 1/s²（绳子的扭转回复，"
                     "实机偏航没有它），偏置 c = {:.3f} rad/s²".format(result["c"]))
        if result.get("rate_yaw_kp") is not None:
            lines.append(f"建议偏航角速度环：穿越频率 ω_c = {result['wc_rad_s']:.1f} rad/s，"
                         f"rate_yaw_kp = {result['rate_yaw_kp']:.5f} N·m/(rad/s)，"
                         f"rate_yaw_ki = {result['rate_yaw_ki']:.5f} N·m/rad"
                         "（kp = ω_c/b，ki = kp·ω_c/5；仅 RAM 试用后再用 rate 注入验证）")
        if result.get("moment_max_hover") is not None:
            lines.append(f"悬停（F = {result['hover_n']:.1f} N）最大可用偏航力矩 ≈ "
                         f"{result['moment_max_hover'] * 1e3:.2f} mN·m（k·min(F, 2·T单max − F)），"
                         f"对应最大角加速度 ≈ {result['alpha_max_hover']:.2f} rad/s² "
                         f"（{math.degrees(result['alpha_max_hover']):.0f}°/s²）")
        if result.get("saturation_fraction") is not None:
            lines.append(f"差速饱和占比 {result['saturation_fraction'] * 100:.1f}%")
    else:
        steps = result.get("steps", [])
        lines.append(f"偏航角速度闭环阶跃：识别到 {len(steps)} 次，跟上 {result.get('followed', 0)} 次")
        if result.get("rise90_s") is not None:
            lines.append(f"90% 上升时间 = {result['rise90_s'] * 1000:.0f} ms（自参考开始变化起，均值）")
        if result.get("overshoot") is not None:
            lines.append(f"超调 = {result['overshoot'] * 100:.1f}%（各次最大）")
        if steps:
            lines.append(f"稳态（末值）误差 = {result['final_error']:.3f} rad/s（各次绝对值均值），"
                         f"稳态抖动 = {result['jitter']:.3f} rad/s")
        if result.get("saturation_fraction") is not None:
            lines.append(f"饱和时间占比 {result['saturation_fraction'] * 100:.1f}%"
                         "（|M| 触及 k·(2·T单max − F) 的激励拍）")
    lines.extend(f"说明：{n}" for n in result.get("notes", []))
    lines.extend(f"注意：{w}" for w in result.get("warnings", []))
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="吊绳偏航辨识（YAW）分析，只读存档")
    parser.add_argument("folders", nargs="+", type=Path,
                        help="yaw_* 存档目录（含 samples.csv 与 conditions.json）")
    args = parser.parse_args(argv)
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    for folder in args.folders:
        try:
            result = analyse_run_dir(folder)
        except (OSError, ValueError, KeyError, json.JSONDecodeError, csv.Error) as error:
            print(f"{folder.name}：读不出来或算不出来：{error}")
            continue
        print(f"{folder.name}：")
        print("  " + summary_text(result).replace("\n", "\n  "))
    return 0


__all__ = ["FIELD_RENAMES", "REQUIRED_RENAMED", "analyse_conditions", "analyse_diff", "analyse_rate",
           "analyse_run_dir", "integrate_psi", "main", "plant_constants", "read_run_dir",
           "rename_samples", "run_inject", "saturation_fraction", "summary_text", "with_moment",
           "yaw_moment_limit"]


if __name__ == "__main__":
    raise SystemExit(main())
