"""水平槽 XY 辨识（SYSID MODE XY，R-XYID-1）的分析：纯函数，不碰 Tk、不写文件。

契约见 `doc/sysid-xy-contract.md`。记录字段表沿用 ALT 追加的 7 个尾字段，XY 轮里含义变了，
所以先在这一层**清楚地改名**（`rename_samples`），下游和页面上不再出现 "height" 这个名字：

    height -> pos_u        沿 u 的光流位置（相对起点）[m]
    height_raw -> pos_perp 垂直于 u 的光流位置（只记录不控制）[m]
    height_sp -> pos_sp_u  沿 u 的位置参考（tilt 注入为 0）[m]
    vz -> vel_u            沿 u 的光流速度 [m/s]
    vz_sp -> vel_sp_u      速度环参考（含前馈；tilt 注入为 0）[m/s]
    az -> acc_u            沿 u 的期望水平加速度（tilt 注入为 g·tanθ）[m/s²]
    vbat -> vbat           电池电压 [V]

`angle` 是绕杆（n 轴）的绝对角度，含开跑姿态；**正角把推力偏向 −u**（固件 `app_sysid_xy.c`）。
`angle_sp` 只是目标倾角偏置在杆轴上的分量（不含开跑姿态），同一口径：tilt 注入 θ>0 时 `angle_sp < 0`。
`acc_u`（原 az）才是沿 +u 为正的指令加速度。

分析：

* tilt（开环，辨对象）：
  - 沿 u 的加速度由光流速度做零相位平滑后中心差分得到；
  - 增益 k 与动摩擦等效加速度 c：只取「正在滑动」的点，最小二乘拟合
    `a_meas = k·a_cmd − c·sign(v)`（a_cmd = g·tanθ 指令）；k 是「倾角 → 加速度」相对理想 g·tanθ 的增益；
  - 静摩擦门槛角：激励开始后光流速度第一次越过噪声门限的那一刻，往回扣掉光流滞后，取当时的
    实测倾角（相对静止基线）的绝对值；
  - 光流滞后：光流加速度对 IMU 侧实测倾角换算的加速度 −g·tan(angle − 基线) 做互相关，取峰值滞后。
* vel / pos（闭环，验证）：把参考信号切成平台，相邻平台之间的跳变就是一次阶跃，逐次给
  90% 上升时间（自阶跃开始起）、超调、末值误差、稳态抖动，再汇总。
"""
from __future__ import annotations

import math
import statistics

from .breakaway import read_run_dir, smooth

GRAVITY_M_S2 = 9.80665

#: 记录字段（schema 名）-> XY 模式下的名字。
FIELD_RENAMES = {
    "height": "pos_u", "height_raw": "pos_perp", "height_sp": "pos_sp_u",
    "vz": "vel_u", "vz_sp": "vel_sp_u", "az": "acc_u",
}
#: XY 分析用得到的（改名后的）字段。
REQUIRED_RENAMED = ("pos_u", "pos_sp_u", "vel_u", "vel_sp_u", "acc_u", "angle", "angle_sp")

#: 分析网格 [Hz]、加速度用的速度平滑窗 [s]、滞后搜索上限 [s]。
GRID_HZ = 100.0
VEL_SMOOTH_S = 0.12
MAX_LAG_S = 0.5
#: 噪声基线取记录开头这么长（且倾角指令仍为 0）[s]。
BASELINE_S = 0.4
#: 判「已滑动」的速度门限：max(N σ, 下限) [m/s]。
MOVE_SIGMAS = 4.0
MOVE_FLOOR_M_S = 0.004
#: 倾角指令大于这个值算激励开始 [rad]。
EXCITE_TILT_RAD = 1e-3
MIN_MOVING_POINTS = 20
#: 增益/滞后拟合只用速度明显高于门限的点（速度过零附近符号不可靠）。
FIT_SPEED_FACTOR = 3.0
MAX_RELATIVE_RESIDUAL = 0.6
#: 平台切分：与平台首点差在 band·A 以内算同一平台；平台至少持续这么久 [s]；跳变至少占幅值的比例。
PLATEAU_BAND = 0.08
MIN_PLATEAU_S = 0.25
MIN_STEP_FRACTION = 0.3
#: 阶跃后的响应量至少是参考跳变的这么多才算「跟上」。
MIN_FOLLOW_FRACTION = 0.3


# ---------------------------------------------------------------- 改名与小工具


def rename_samples(samples) -> list[dict]:
    """把 ALT 尾字段名换成 XY 含义的名字（其余字段照旧）；输入不改动。"""
    out = []
    for sample in samples:
        row = {FIELD_RENAMES.get(key, key): value for key, value in dict(sample).items()}
        out.append(row)
    return out


def _finite(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _column(rows, name) -> list[float]:
    return [(_finite(row.get(name)) if _finite(row.get(name)) is not None else float("nan"))
            for row in rows]


def _resample(times_s, values, grid_s):
    """线性插值到均匀网格（缺测点用邻点线性桥接）。"""
    pairs = [(t, v) for t, v in zip(times_s, values) if v is not None and math.isfinite(v)]
    if len(pairs) < 2:
        return [float("nan")] * len(grid_s)
    out, j = [], 0
    for g in grid_s:
        while j + 2 < len(pairs) and pairs[j + 1][0] < g:
            j += 1
        (t0, v0), (t1, v1) = pairs[j], pairs[j + 1]
        span = t1 - t0
        w = 0.0 if span <= 0.0 else min(1.0, max(0.0, (g - t0) / span))
        out.append(v0 + w * (v1 - v0))
    return out


def _median(values) -> float | None:
    data = [v for v in values if v is not None and math.isfinite(v)]
    return statistics.median(data) if data else None


def _std(values) -> float:
    data = [v for v in values if v is not None and math.isfinite(v)]
    return statistics.pstdev(data) if len(data) > 1 else 0.0


def _derivative(grid_s, values):
    """中心差分（两端单侧）。"""
    n = len(values)
    if n < 3:
        return [0.0] * n
    step = grid_s[1] - grid_s[0]
    out = [(values[1] - values[0]) / step]
    out += [(values[i + 1] - values[i - 1]) / (2.0 * step) for i in range(1, n - 1)]
    out.append((values[-1] - values[-2]) / step)
    return out


# ---------------------------------------------------------------- tilt


def _lstsq2(rows):
    """最小二乘 y = k·x1 + c·x2；返回 (k, c) 或 None（矩阵奇异）。"""
    s11 = s12 = s22 = b1 = b2 = 0.0
    for x1, x2, y in rows:
        s11 += x1 * x1
        s12 += x1 * x2
        s22 += x2 * x2
        b1 += x1 * y
        b2 += x2 * y
    det = s11 * s22 - s12 * s12
    if abs(det) < 1e-12 * max(s11 * s22, 1e-30):
        return None
    return (b1 * s22 - b2 * s12) / det, (s11 * b2 - s12 * b1) / det


def _fit_with_lag(regressor, vel_s, acc, moving, step_s: float):
    """在 0..MAX_LAG_S 里逐个滞后试：`acc(t) ≈ k·regressor(t − lag) − c·sign(v)`，取残差最小的滞后。

    只用「正在滑动」的点，这样静摩擦的停滞段不会把滞后拉偏。返回 (滞后 s, k, c, 点数, 相对残差) 或 None。
    """
    best = None
    for lag in range(int(MAX_LAG_S / step_s) + 1):
        rows = [(regressor[i - lag], -math.copysign(1.0, vel_s[i]), acc[i])
                for i in moving if i - lag >= 0 and abs(regressor[i - lag]) > 1e-3]
        if len(rows) < MIN_MOVING_POINTS:
            continue
        fit = _lstsq2(rows)
        if fit is None:
            continue
        k, c = fit
        sse = sum((y - k * x1 - c * x2) ** 2 for x1, x2, y in rows)
        var = sum(y * y for _x1, _x2, y in rows)
        relative = sse / var if var > 0 else float("inf")
        if best is None or relative < best[4]:
            best = (lag * step_s, k, c, len(rows), relative)
    return best


def analyse_tilt(times_s, rows) -> dict:
    """tilt 轮：增益、动摩擦、静摩擦门槛角、光流滞后。`rows` 是改名后的样本。"""
    if len(rows) < 50:
        raise ValueError("样本太少（不足 50 条），算不了倾角对加速度的关系")
    t0, t1 = times_s[0], times_s[-1]
    if t1 - t0 < 1.0:
        raise ValueError("记录不足 1 秒")
    n_grid = int((t1 - t0) * GRID_HZ) + 1
    step = 1.0 / GRID_HZ
    grid = [t0 + i * step for i in range(n_grid)]
    vel = _resample(times_s, _column(rows, "vel_u"), grid)
    a_cmd = _resample(times_s, _column(rows, "acc_u"), grid)
    angle = _resample(times_s, _column(rows, "angle"), grid)
    angle_sp = _resample(times_s, _column(rows, "angle_sp"), grid)
    if not all(math.isfinite(v) for v in vel + a_cmd + angle + angle_sp):
        raise ValueError("记录里有缺失的光流速度或倾角数据")
    vel_s = smooth(grid, vel, VEL_SMOOTH_S)
    acc = _derivative(grid, vel_s)

    excite = next((i for i, v in enumerate(angle_sp) if abs(v) > EXCITE_TILT_RAD), None)
    if excite is None:
        raise ValueError("倾角指令全程为 0：这一轮没有激励，没法辨")
    quiet = slice(0, max(excite, 1))
    base_angle = _median(angle[quiet]) or 0.0
    noise = _std(vel_s[quiet]) if excite >= 5 else 0.0
    threshold = max(MOVE_SIGMAS * noise, MOVE_FLOOR_M_S)
    result: dict = {"inject": "tilt", "base_angle_rad": base_angle, "move_threshold_m_s": threshold,
                    "warnings": []}

    moving = [i for i in range(n_grid) if abs(vel_s[i]) > FIT_SPEED_FACTOR * threshold]
    # 正绕杆角把推力偏向 −u，换成沿 +u 的加速度要取负号。
    a_angle = [-GRAVITY_M_S2 * math.tan(a - base_angle) for a in angle]
    # 加速度是「平滑后的速度」求导出来的，等于把真加速度过了一遍窗宽 VEL_SMOOTH_S 的滑动平均；
    # 回归量也得过同一个窗，否则方波式的倾角指令（doublet）会把增益与摩擦系统性拉低（端到端彩排实测：
    # 增益低 10~20%）。
    a_angle = smooth(grid, a_angle, VEL_SMOOTH_S)
    a_cmd = smooth(grid, a_cmd, VEL_SMOOTH_S)

    # 光流滞后与「实测倾角 → 加速度」增益：以 IMU 侧实测倾角为时间基准。
    by_angle = _fit_with_lag(a_angle, vel_s, acc, moving, step)
    if by_angle is None or by_angle[4] > MAX_RELATIVE_RESIDUAL:
        result.update(flow_lag_s=None, gain_vs_measured_angle=None)
        result["warnings"].append("光流滞后算不出来（滑动的点太少或拟合太差，激励太小或速度太噪）")
    else:
        result.update(flow_lag_s=by_angle[0], gain_vs_measured_angle=by_angle[1],
                      measured_friction_m_s2=by_angle[2], flow_lag_residual=by_angle[4])

    # 相对指令 g·tanθ 的增益与动摩擦（指令到光流的总滞后另算，含姿态环响应）。
    by_cmd = _fit_with_lag(a_cmd, vel_s, acc, moving, step)
    if by_cmd is None:
        result.update(gain=None, kinetic_friction_m_s2=None)
        result["warnings"].append("滑动的点太少，增益与动摩擦算不出来：加大倾角幅值")
    else:
        result.update(gain=by_cmd[1], kinetic_friction_m_s2=by_cmd[2], cmd_lag_s=by_cmd[0],
                      gain_points=by_cmd[3])
    # 动摩擦以 IMU 侧实测倾角那一拟合为准：指令 → 倾角是一阶滞后，用「纯延迟 + 增益」去配指令会把
    # 摩擦项系统性配小（端到端彩排：真值 0.2 m/s² 时指令口径只配出 0.03~0.06）。算不出才退回指令口径。
    if result.get("measured_friction_m_s2") is not None:
        result["kinetic_friction_m_s2"] = result["measured_friction_m_s2"]

    # 静摩擦门槛角。
    lag_used = result["flow_lag_s"] or 0.0
    moved = next((i for i in range(excite, n_grid) if abs(vel_s[i]) > threshold), None)
    if moved is None:
        result["break_angle_rad"] = None
        result["warnings"].append("整轮没滑动：倾角幅值不够越过静摩擦，加大幅值再跑")
    else:
        back = max(excite, moved - int(round(lag_used / step)))
        result["break_angle_rad"] = abs(angle[back] - base_angle)
        result["break_angle_cmd_rad"] = abs(angle_sp[back])
        result["break_time_s"] = grid[moved] - t0
        result["break_direction"] = 1 if vel_s[moved] > 0 else -1
    return result


# ---------------------------------------------------------------- vel / pos


def _plateaus(times_s, ref):
    peak = max(abs(v) for v in ref)
    band = PLATEAU_BAND * peak
    found, i, n = [], 0, len(ref)
    while i < n:
        j = i
        while j < n and abs(ref[j] - ref[i]) <= band:
            j += 1
        if times_s[j - 1] - times_s[i] >= MIN_PLATEAU_S:
            found.append((i, j - 1, _median(ref[i:j])))
        i = max(j, i + 1)
    return found, peak


def _step_metrics(times_s, ref, meas, before, after):
    a0, a1, level0 = before
    b0, b1, level1 = after
    delta_ref = level1 - level0
    sign = 1.0 if delta_ref > 0 else -1.0
    pre_from = max(a0, a1 - max(5, (a1 - a0) // 3))
    y_pre = _median(meas[pre_from:a1 + 1])
    tail = max(b0, b1 - max(5, (b1 - b0 + 1) * 3 // 10))
    tail_y = meas[tail:b1 + 1]
    y_fin = _median(tail_y)
    if y_pre is None or y_fin is None:
        return None
    delta_y = y_fin - y_pre
    t_start = times_s[min(a1 + 1, b1)]
    entry = {"t_start_s": t_start - times_s[0], "delta_ref": delta_ref, "delta_meas": delta_y,
             "final_error": y_fin - level1, "jitter": _std(tail_y)}
    if sign * delta_y < MIN_FOLLOW_FRACTION * abs(delta_ref):
        entry.update(followed=False, rise90_s=None, overshoot=None)
        return entry
    rise = None
    peak = 0.0
    for k in range(a1, b1 + 1):
        progress = sign * (meas[k] - y_pre)
        peak = max(peak, progress)
        if rise is None and progress >= 0.9 * sign * delta_y:
            rise = times_s[k] - t_start
    entry.update(followed=True, rise90_s=max(rise, 0.0) if rise is not None else None,
                 overshoot=max(0.0, (peak - sign * delta_y) / abs(delta_y)))
    return entry


#: 由位置参考反推注入速度用的差分跨度 [s]，与「位置参考至少走了这么多才算有注入」[m]。
INJECT_DIFF_SPAN_S = 0.1
INJECT_MIN_TRAVEL_M = 1e-3


def _injected_velocity(times_s, pos_sp):
    """vel 注入真正的阶跃参考 v_inj = d(pos_sp_u)/dt；位置参考没有有效变化时返回 None（退回 vel_sp_u）。

    固件记的 vz_sp 是速度环的输入：位置环对 p_sp 的跟踪误差（摩擦卡住、起步慢）也在里面，
    误差一大它就不再是平台式的阶跃，平台切分会切出假阶跃、把真阶跃漏掉（端到端彩排：摩擦 + 弱增益
    时 vz_sp 峰值是注入幅值的 2 倍多）。vel 注入的 p_sp = ∫v_inj dt（契约），反推即得干净的注入剖面。
    """
    if not all(math.isfinite(v) for v in pos_sp) or len(pos_sp) < 5:
        return None
    if max(pos_sp) - min(pos_sp) < INJECT_MIN_TRAVEL_M:
        return None
    n, out, hi = len(pos_sp), [], 0
    lo = 0
    for i in range(n):
        while times_s[i] - times_s[lo] > INJECT_DIFF_SPAN_S / 2.0:
            lo += 1
        while hi + 1 < n and times_s[hi + 1] - times_s[i] <= INJECT_DIFF_SPAN_S / 2.0:
            hi += 1
        span = times_s[hi] - times_s[lo]
        out.append((pos_sp[hi] - pos_sp[lo]) / span if span > 0.0 else 0.0)
    # 激励结束时固件把 p_sp 直接清零（残余位移不为 0 时差分出一根孤立尖峰，会把 ref_peak 抬高一倍），
    # 注入剖面本身是平台 + 斜坡，用滑动中位数把短于窗宽一半的尖峰去掉、平台与斜坡不动。
    half = max(1, int(round(INJECT_SPIKE_WINDOW_S / 2.0 / max((times_s[-1] - times_s[0]) / (n - 1), 1e-6))))
    return [statistics.median(out[max(0, i - half):i + half + 1]) for i in range(n)]


#: 推力掉到保持推力的这个比例以下 = 已进入回落（RAMP_DOWN / 软停），位置/速度环不再工作。
RAMP_DOWN_THRUST_FRACTION = 0.97
INJECT_SPIKE_WINDOW_S = 0.3


def _control_end(rows) -> int:
    """闭环还在工作的最后一拍（不含）：推力先升到保持值，之后第一次掉到 97% 以下就是回落开始。

    RAMP_DOWN 里固件把位置/速度参考清零、环也不跑，这一段记录上「参考回到 0」不是一次受控阶跃，
    分析得把它切掉，否则会多出一个「没跟上」的假阶跃（或一个假的上升时间/超调混进均值）。
    推力列缺失或全 0（合成数据）时不切。
    """
    thrust = _column(rows, "thrust")
    finite = [v for v in thrust if math.isfinite(v)]
    top = max(finite, default=0.0)
    if top <= 0.0:
        return len(rows)
    floor = RAMP_DOWN_THRUST_FRACTION * top
    reached = next((i for i, v in enumerate(thrust) if v >= floor), None)
    if reached is None:
        return len(rows)
    return next((i for i in range(reached, len(rows)) if thrust[i] < floor), len(rows))


def analyse_steps(times_s, rows, inject: str) -> dict:
    """vel / pos 闭环轮：阶跃指标。参考 = vel_sp_u（vel）或 pos_sp_u（pos），测量 = vel_u / pos_u。"""
    if len(rows) < 50:
        raise ValueError("样本太少（不足 50 条），算不了阶跃指标")
    ref_name, meas_name = ("vel_sp_u", "vel_u") if inject == "vel" else ("pos_sp_u", "pos_u")
    ref, meas = _column(rows, ref_name), _column(rows, meas_name)
    if not all(math.isfinite(v) for v in ref + meas):
        raise ValueError("记录里有缺失的参考或测量数据")
    if inject == "vel":
        injected = _injected_velocity(times_s, _column(rows, "pos_sp_u"))
        if injected is not None:
            ref = injected
    end = _control_end(rows)
    if end < 50:
        end = len(rows)
    times_s, ref, meas = times_s[:end], ref[:end], meas[:end]
    plateaus, peak = _plateaus(times_s, ref)
    if peak < 1e-9:
        raise ValueError("参考全程为 0：这一轮没有激励，没法算阶跃指标")
    steps = []
    for before, after in zip(plateaus, plateaus[1:]):
        if abs(after[2] - before[2]) < MIN_STEP_FRACTION * peak:
            continue
        entry = _step_metrics(times_s, ref, meas, before, after)
        if entry is not None:
            steps.append(entry)
    result: dict = {"inject": inject, "steps": steps, "warnings": [], "ref_peak": peak}
    if not steps:
        result["warnings"].append("没识别到阶跃：参考的平台太短（要 ≥ 0.25 s）或幅值变化太小")
        return result
    followed = [s for s in steps if s["followed"]]
    result["followed"] = len(followed)
    if len(followed) < len(steps):
        result["warnings"].append(f"{len(steps) - len(followed)} 次阶跃测量没跟上参考（响应不足 30%）")
    rises = [s["rise90_s"] for s in followed if s["rise90_s"] is not None]
    result["rise90_s"] = statistics.mean(rises) if rises else None
    result["overshoot"] = max((s["overshoot"] for s in followed), default=None)
    result["final_error"] = statistics.mean(abs(s["final_error"]) for s in steps)
    result["jitter"] = statistics.mean(s["jitter"] for s in steps)
    return result


# ---------------------------------------------------------------- 入口与文字


def run_inject(conditions: dict) -> str | None:
    """本轮注入类型：页面下发并核对过的请求优先，其次固件溯源。"""
    conditions = conditions or {}
    request = conditions.get("xy_request")
    if isinstance(request, dict) and request.get("inject"):
        return str(request["inject"])
    xy = conditions.get("xy")
    if isinstance(xy, dict) and xy.get("xy_inject"):
        return str(xy["xy_inject"])
    return None


def analyse_conditions(times_s, samples, conditions: dict) -> dict:
    """按本轮注入类型分析；`samples` 是按 schema 解出的原始样本（尾字段还叫 height 等）。"""
    inject = run_inject(conditions)
    if inject not in ("tilt", "vel", "pos"):
        raise ValueError("本轮 XY 注入类型不明（conditions 里没有 xy_request / xy 溯源）")
    rows = rename_samples(samples)
    if not rows or len(rows) != len(times_s):
        raise ValueError("样本与时间戳对不上")
    missing = [name for name in REQUIRED_RENAMED if name not in rows[0]]
    if missing:
        raise ValueError(f"记录里缺少 XY 字段：{'、'.join(missing)}")
    if inject == "tilt":
        return analyse_tilt(times_s, rows)
    return analyse_steps(times_s, rows, inject)


def summary_text(result: dict) -> str:
    """结果区的中文摘要。"""
    lines = []
    if result.get("inject") == "tilt":
        gain = result.get("gain")
        lines.append("倾角 → 加速度增益："
                     + (f"k = {gain:.3f}（沿 u 实测加速度 / 指令 g·tanθ，1 = 理想）" if gain is not None
                        else "算不出来"))
        if result.get("gain_vs_measured_angle") is not None:
            lines.append(f"按 IMU 实测倾角算的增益 = {result['gain_vs_measured_angle']:.3f}"
                         "（与上一行之差 = 指令到实际倾角这段的损失）")
        if result.get("kinetic_friction_m_s2") is not None:
            lines.append(f"动摩擦等效加速度 = {result['kinetic_friction_m_s2']:.3f} m/s²"
                         f"（约等效倾角 {math.degrees(math.atan(result['kinetic_friction_m_s2'] / GRAVITY_M_S2)):.2f}°）")
        if result.get("break_angle_rad") is not None:
            side = "沿 +u" if result.get("break_direction", 1) > 0 else "沿 −u"
            lines.append(f"静摩擦门槛角 ≈ {math.degrees(result['break_angle_rad']):.2f}°"
                         f"（激励后 {result['break_time_s']:.2f} s {side}开始滑动，"
                         f"指令倾角 {math.degrees(result['break_angle_cmd_rad']):.2f}°）")
        if result.get("flow_lag_s") is not None:
            lines.append(f"光流相对 IMU 的滞后 ≈ {result['flow_lag_s'] * 1000:.0f} ms"
                         f"（拟合残差 {result['flow_lag_residual'] * 100:.0f}%）")
    else:
        unit, scale = ("mm/s", 1000.0) if result.get("inject") == "vel" else ("mm", 1000.0)
        what = "速度" if result.get("inject") == "vel" else "位置"
        steps = result.get("steps", [])
        lines.append(f"{what}闭环阶跃：识别到 {len(steps)} 次，跟上 {result.get('followed', 0)} 次")
        if result.get("rise90_s") is not None:
            lines.append(f"90% 上升时间 = {result['rise90_s'] * 1000:.0f} ms（自参考开始变化起，均值）")
        if result.get("overshoot") is not None:
            lines.append(f"超调 = {result['overshoot'] * 100:.1f}%（各次最大）")
        if steps:
            lines.append(f"末值误差 = {result['final_error'] * scale:.1f} {unit}（各次绝对值均值），"
                         f"稳态抖动 = {result['jitter'] * scale:.1f} {unit}（末段标准差均值）")
    lines.extend(f"注意：{w}" for w in result.get("warnings", []))
    return "\n".join(lines)


__all__ = ["FIELD_RENAMES", "REQUIRED_RENAMED", "analyse_conditions", "analyse_steps",
           "analyse_tilt", "read_run_dir", "rename_samples", "run_inject", "summary_text"]
