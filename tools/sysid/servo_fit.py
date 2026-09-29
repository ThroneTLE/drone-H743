"""舵机单独（电机不转）台架轮：舵机甩动倾转组件对机体的反作用惯量 J、舵机二阶与纯延迟。

为什么单独测：带桨轮里 TWD 结构（`fit._fit_twd`）的反作用力偶 ρ 与推力增益 G 在同一条
数据里此消彼长，而 ρ 恰是限制速率环增益的主因。电机不转时推力为 0，杆上只剩舵机甩动
组件的反作用，ρ 的来源可以单独量出来，作为带桨拟合的**交叉核对**（本版不钉死进联合拟合）。

模型（绕光杆轴一自由度，小角度线性化）::

    I_杆·θ̈ + c·θ̇ + K·θ = −J·s̈(t) + H·s(t)
    s̈ = ωs²·(u(t − T) − s) − 2ζs·ωs·ṡ          u = 固件记录的 servo_tilt（指令倾角）

`servo_tilt` 的正向 = 杆轴正向力矩的倾转方向（与推力力矩同向）。输出取陀螺沿杆轴的 θ̇。
拟合 J/I_杆 与 H/I_杆（符号都放开）、c、T、ωs、ζs，以及摆频 K/I_杆；K 取挂砝码实测的
台架刚度，没有时取 m·g·d。于是 I_杆 = K/(K/I_杆)，J = (J/I_杆)·I_杆，H 同理。

H·s 是倾转组件的**静态重力偏心**：组件质心不在舵机轴上（本机电机座挂在轴下约 0.11 m），
倾转 s 时质心横移 r·s，绕杆多出 m_组件·g·r·s 的力矩——与 s̈ 无关、与 K 同量级。漏掉它，
快的部分仍只定得住 J/I_杆，而 I_杆 = K/k 靠的摆频被这项静力矩拽偏，J 跟着错同一个倍数
（合成数据 H = ±0.35 N·m/rad 时 J 偏 −34%/+15%，报的不确定度只有 2–3%）。它在杆上才有：
飞行时绕质心转，重力过质心不产生力矩。H > 0 = 组件倾向推力力矩正向时静力矩也朝正向。

J 的符号按上式约定：**J < 0** 表示舵机加速把机体推向推力力矩的同一方向——带桨时这就是
TWD 的一对虚零点（频响凹口 + 相位 +180°），与带桨拟合的 ρ > 0 对应。换到带桨轮的
固件力矩单位（u_fw = |L_fw|·T·s）::

    ρ_pred = −J / (|L_fw| · T)

与 `fit._twd_matrices` 里"杆上力矩 = G·u + ρ·u''"同一约定，可以直接和带桨拟合的 ρ 比。

J/I_杆、H/I_杆 在给定其余参数时对输出是线性的，所以和初值/零偏一起每次最小二乘解掉
（变量投影），符号自然放开；非线性只剩摆频、阻尼、延迟与舵机二阶。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import fit as _fit

try:
    from scipy.optimize import least_squares
    _HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    _HAVE_SCIPY = False

_NAN = float("nan")

#: 代价频段与起步段沿用带桨 TWD 拟合（12 Hz/4 阶零相位、0.5 s 之后）：两者同一把尺子。
BAND_HZ = _fit.TWD_BAND_HZ
BAND_ORDER = _fit.TWD_BAND_ORDER
BURN_IN_S = _fit.BURN_IN_S

#: 带内拟合优度下限 [%]，与带桨 TWD 相同。
FIT_PERCENT_MIN = _fit.TWD_FIT_PERCENT_MIN

#: 参数边界：纯延迟 [s]、舵机自然频率 [rad/s]、阻尼比、c/I_杆 [1/s]。
DEAD_TIME_BOUNDS_S = (0.0, 0.15)
SERVO_WN_BOUNDS = _fit.SERVO_WN_BOUNDS
SERVO_ZETA_BOUNDS = _fit.SERVO_ZETA_BOUNDS
DAMPING_RATIO_BOUNDS = (0.0, 20.0)

#: 多起点：(纯延迟 [s], 舵机自然频率 [rad/s])。
STARTS = ((0.03, 33.0), (0.02, 45.0), (0.05, 25.0))

#: J 的相对不确定度超过它只作提示 [%]。
REACTION_UNCERTAINTY_NOTE_PCT = 25.0

#: 两轮幅值相差不到这个比例就算同一个幅值（回差估计至少要两个不同幅值）。
AMPLITUDE_DISTINCT_RATIO = 1.1

BACKLASH_METHOD = (
    "方法：回差按间隙 ±b 的迟滞环处理。双脉冲是方波式指令，每次换向舵机先走完空程 b 才带动"
    "组件，一次谐波（描述函数）增益 N(A) ≈ 1 − b/A，所以表观 J(A) = J·(1 − b/A)：对 1/A 做"
    "线性回归，截距是大幅值极限 J，斜率是 −J·b。纯延迟同理随 1/A 变大（先走空程），另报"
    " T(A) 对 1/A 的回归。前提：小角度、方波近似；正弦激励的描述函数不同，只作量级。")


@dataclass(frozen=True)
class ServoFit:
    #: 反作用惯量 J [kg·m²]，按 I_杆·θ̈ + c·θ̇ + K·θ = −J·s̈ 的约定（符号见文件头）。
    reaction_inertia_kg_m2: float
    #: J/I_杆（拟合直接得到的量）。
    reaction_ratio: float
    #: 静态重力偏心 H [N·m/rad]：杆上力矩多出的 H·s（倾转组件质心不在舵机轴上）。
    pod_gravity_n_m_rad: float
    inertia_rod_kg_m2: float
    #: 用的回中刚度 K [N·m/rad]（实测台架刚度或 m·g·d）。
    stiffness_n_m_rad: float
    damping_n_m_s: float
    natural_hz: float
    dead_time_s: float
    servo_wn_rad_s: float
    servo_zeta: float
    #: 1 Hz 处"纯延迟 + 舵机"的等效延迟 [s]。
    equivalent_delay_1hz_s: float
    fit_percent: float
    #: 指令倾角幅值 max|servo_tilt| [rad]。
    amplitude_rad: float
    #: σ(J)/|J| [%]。
    reaction_uncertainty_pct: float
    samples: int
    notes: tuple[str, ...] = ()
    blockers: tuple[str, ...] = ()
    structure: str = "servo"

    @property
    def warnings(self) -> list[str]:
        return list(self.notes)


def predicted_reaction_couple(reaction_inertia_kg_m2: float, lever_m: float | None,
                              thrust_n: float | None) -> float:
    """ρ_pred = −J/(|L_fw|·T)：带桨轮"杆上力矩 = G·u + ρ·u''"里的 ρ（固件力矩单位）。

    `lever_m` 是那套固件沿杆轴的倾转力臂，`thrust_n` 是带桨时的推力（悬停取机重）。缺一个 NaN。
    """
    if lever_m is None or thrust_n is None:
        return _NAN
    lever, thrust = abs(float(lever_m)), float(thrust_n)
    if not (math.isfinite(lever) and lever > 1e-6 and math.isfinite(thrust) and thrust > 0.0):
        return _NAN
    return -float(reaction_inertia_kg_m2) / (lever * thrust)


def rod_axis_lever(levers_m: tuple[float, float] | None, azimuth_rad: float) -> float | None:
    """沿杆轴的固件倾转力臂 |L|：按 cos²ψ / sin²ψ 加权（权重 < 0.02 的轴忽略），缺数 None。"""
    if levers_m is None:
        return None
    axes = [(math.cos(azimuth_rad) ** 2, levers_m[0]), (math.sin(azimuth_rad) ** 2, levers_m[1])]
    used = [(weight, lever) for weight, lever in axes if weight >= _fit.TILT_AXIS_WEIGHT_MIN]
    total = sum(weight for weight, _lever in used)
    if not used or any(lever is None or not math.isfinite(float(lever)) for _w, lever in used):
        return None
    return sum(weight / total * abs(float(lever)) for weight, lever in used)


def _columns(params: dict, t: np.ndarray, command: np.ndarray, left: float, fs: float
             ) -> tuple[np.ndarray, ...]:
    """(J/I_杆 = 1 时的强迫响应, H/I_杆 = 1 时的强迫响应, θ₀=1 自由响应, ω₀=1 自由响应)，都是 θ̇。

    借用带桨 TWD 的状态空间（I_杆 = 1）：反作用列取 G = 0、ρ = −1，即 θ̈ + b·θ̇ + k·θ = −s̈；
    偏心列取 G = 1、ρ = 0，即 θ̈ + b·θ̇ + k·θ = s。两列走同一个舵机二阶与纯延迟。
    """
    delayed = np.interp(t - params["T"], t, command, left=left, right=command[-1])
    start = np.array([left, 0.0, 0.0, 0.0])
    a_matrix, b_vector, c_vector = _fit._twd_matrices(
        1.0, 0.0, params["k"], params["wn"], params["zeta"], -1.0, params["b"])
    forced, (free_angle, free_rate) = _fit._state_space_columns(
        a_matrix, b_vector, c_vector, 1.0 / fs, delayed, start, (2, 3))
    a_matrix, b_vector, c_vector = _fit._twd_matrices(
        1.0, 1.0, params["k"], params["wn"], params["zeta"], 0.0, params["b"])
    static, _unused = _fit._state_space_columns(
        a_matrix, b_vector, c_vector, 1.0 / fs, delayed, start, ())
    return forced, static, free_angle, free_rate


def fit_servo_only(t: np.ndarray, servo_tilt_rad: np.ndarray, rate_rad_s: np.ndarray, *,
                   stiffness_n_m_rad: float, inertia_rod_guess_kg_m2: float,
                   inertia_rod_kg_m2: float | None = None) -> ServoFit:
    """拟合一轮舵机单独数据。`t` 必须等距（先 `fit.uniform_signal`）。

    `stiffness_n_m_rad` 是回中刚度 K（台架实测或 m·g·d），`inertia_rod_guess_kg_m2` 只定界；
    给了 `inertia_rod_kg_m2` 就钉死 I_杆（摆频不再拟合）。
    """
    if not _HAVE_SCIPY:
        raise RuntimeError("舵机单独拟合需要 scipy")
    t = np.asarray(t, dtype=float)
    command = np.asarray(servo_tilt_rad, dtype=float)
    rate = np.asarray(rate_rad_s, dtype=float)
    if t.size < 64 or command.size != t.size or rate.size != t.size:
        raise ValueError("样本太少或长度不一致，拟合没有意义")
    stiffness = float(stiffness_n_m_rad)
    if not (math.isfinite(stiffness) and stiffness > 0.0):
        raise ValueError("回中刚度必须为正：量杆高（杆在质心上方）或做挂砝码试验")
    guess = float(inertia_rod_guess_kg_m2)
    if not (math.isfinite(guess) and guess > 0.0):
        raise ValueError("绕杆惯量的估计必须为正")
    if float(np.ptp(command)) < 1e-4:
        raise ValueError("舵机倾角几乎没动（servo_tilt 全程不变），这段数据辨不出东西")
    fs = 1.0 / _fit._sample_period(t)
    keep = int(round(BURN_IN_S * fs))
    if t.size - keep < 64:
        raise ValueError("扣掉起步段后样本太少，拟合没有意义")
    left = float(command[0])
    band = _fit.lowpass_zero_phase(rate, fs, BAND_HZ, BAND_ORDER)
    norm = max(float(np.linalg.norm(band[keep:] - float(np.mean(band[keep:])))), 1e-12)

    pinned_k = None
    if inertia_rod_kg_m2 is not None:
        pinned = float(inertia_rod_kg_m2)
        if not (math.isfinite(pinned) and pinned > 0.0):
            raise ValueError("钉死的绕杆惯量必须为正")
        pinned_k = stiffness / pinned
    k0 = stiffness / guess
    names = ["b"] + ([] if pinned_k is not None else ["k"]) + ["T", "wn", "zeta"]
    bounds = {"b": DAMPING_RATIO_BOUNDS, "k": (0.05 * k0, 20.0 * k0), "T": DEAD_TIME_BOUNDS_S,
              "wn": SERVO_WN_BOUNDS, "zeta": SERVO_ZETA_BOUNDS}
    lower = np.array([bounds[name][0] for name in names])
    upper = np.array([bounds[name][1] for name in names])

    def unpack(x) -> dict:
        values = dict(zip(names, (float(v) for v in x)))
        values.setdefault("k", pinned_k)
        return values

    def solve(params: dict) -> tuple[np.ndarray, np.ndarray]:
        columns = _fit.lowpass_zero_phase(np.column_stack(_columns(params, t, command, left, fs)),
                                          fs, BAND_HZ, BAND_ORDER)
        basis = np.column_stack([columns[keep:], np.ones(t.size - keep)])
        coefficients, *_ = np.linalg.lstsq(basis, band[keep:], rcond=None)
        return basis @ coefficients - band[keep:], coefficients

    def residual(x) -> np.ndarray:
        error = solve(unpack(x))[0] / norm
        return error if np.all(np.isfinite(error)) else np.full(error.size, 1e3)

    result = None
    for delay0, wn0 in STARTS:
        start = {"b": 0.3, "k": k0, "T": delay0, "wn": wn0, "zeta": 0.5}
        x0 = np.clip(np.array([start[name] for name in names]), lower, upper)
        candidate = least_squares(residual, x0, bounds=(lower, upper), x_scale="jac",
                                  diff_step=1e-4, max_nfev=300)
        if result is None or candidate.cost < result.cost:
            result = candidate
    params = unpack(result.x)
    error, coefficients = solve(params)
    ratio, gravity_ratio = float(coefficients[0]), float(coefficients[1])
    fit_pct = 100.0 * (1.0 - float(np.linalg.norm(error)) / norm)

    # 协方差：把 J/I_杆 也当参数，在最优点上求一次雅可比（其余线性量——含 H/I_杆——仍变量
    # 投影，所以 J 的不确定度已经含了它与 H 的此消彼长）。
    full_names = ["a"] + names

    def full_residual(x) -> np.ndarray:
        values = unpack(x[1:])
        columns = _fit.lowpass_zero_phase(np.column_stack(_columns(values, t, command, left, fs)),
                                          fs, BAND_HZ, BAND_ORDER)
        basis = np.column_stack([columns[keep:, 1:], np.ones(t.size - keep)])
        target = band[keep:] - float(x[0]) * columns[keep:, 0]
        solution, *_ = np.linalg.lstsq(basis, target, rcond=None)
        return (basis @ solution - target) / norm

    span = max(abs(ratio), 1e-6) * 10.0
    full_lower = np.concatenate([[ratio - span], lower])
    full_upper = np.concatenate([[ratio + span], upper])
    x_full = np.clip(np.concatenate([[ratio], result.x]), full_lower, full_upper)
    refined = least_squares(full_residual, x_full, bounds=(full_lower, full_upper),
                            x_scale="jac", diff_step=1e-4, max_nfev=5)
    effective = (t.size - keep) * _fit._lowpass_noise_gain(fs, BAND_HZ, BAND_ORDER)
    covariance = _fit._covariance(refined.jac, full_residual(refined.x), effective,
                                  refined.active_mask)

    k = params["k"]
    inertia_rod = stiffness / k
    reaction = ratio * inertia_rod
    pod_gravity = gravity_ratio * inertia_rod
    gradient = np.zeros(len(full_names))
    gradient[0] = inertia_rod
    if pinned_k is None:
        gradient[full_names.index("k")] = -ratio * stiffness / (k * k)
    uncertainty = _fit._uncertainty_pct(covariance, gradient, abs(reaction))
    damping = params["b"] * inertia_rod
    natural = math.sqrt(k) / (2.0 * math.pi)
    delay_eq = _fit.equivalent_delay_s(params["T"], params["wn"], params["zeta"])

    notes: list[str] = []
    blockers: list[str] = []
    word = ("舵机加速把机体推向推力力矩的同一方向（带桨时就是 TWD 凹口）" if reaction < 0.0
            else "舵机加速把机体推向推力力矩的反方向（带桨时是实零点，非最小相位）")
    notes.append(f"反作用惯量 J = {reaction:+.5f} kg·m²（模型 I_杆·θ̈ + c·θ̇ + K·θ = −J·s̈）：{word}。")
    lean = "推力力矩正向" if pod_gravity >= 0.0 else "推力力矩反向"
    notes.append(f"静态重力偏心 H = {pod_gravity:+.4f} N·m/rad（倾转组件质心不在舵机轴上，倾转时绕杆"
                 f"多出 H·s，朝{lean}；约为回中刚度 K 的 {abs(pod_gravity) / stiffness:.0%}）：已在"
                 "模型里单独拟合，不再混进 I_杆 与 J。它只在杆上有，飞行时绕质心转不产生。")
    if pinned_k is not None:
        notes.append(f"I_杆 按给定值钉死为 {inertia_rod:.5f} kg·m²，摆频不拟合。")
    at_bound = [label for name, label in (("wn", "ωs"), ("zeta", "ζs"), ("T", "T"))
                if min(params[name] - bounds[name][0], bounds[name][1] - params[name])
                <= _fit.BOUND_TOLERANCE * (bounds[name][1] - bounds[name][0])]
    if at_bound:
        notes.append(f"{'、'.join(at_bound)} 贴到拟合边界：激励没把舵机转折频率附近激出来，"
                     "舵机二阶与纯延迟分不开，只有 1 Hz 等效延迟可信。")
    if fit_pct < FIT_PERCENT_MIN:
        notes.append(f"带内（{BAND_HZ:g} Hz 以下、{BURN_IN_S:g} s 之后）拟合优度只有 {fit_pct:.1f}%，"
                     "模型没有描述住这段数据（回差、线缆或台架模态）。")
        blockers.append(f"带内拟合优度 {fit_pct:.0f}% 低于 {FIT_PERCENT_MIN:.0f}%")
    if not uncertainty <= REACTION_UNCERTAINTY_NOTE_PCT:
        notes.append(f"J 的相对不确定度约 {uncertainty:.0f}%：加大舵机摆幅或加长激励重测。")
    return ServoFit(
        reaction_inertia_kg_m2=float(reaction), reaction_ratio=ratio,
        pod_gravity_n_m_rad=float(pod_gravity), inertia_rod_kg_m2=float(inertia_rod),
        stiffness_n_m_rad=stiffness,
        damping_n_m_s=float(damping), natural_hz=float(natural),
        dead_time_s=float(params["T"]), servo_wn_rad_s=float(params["wn"]),
        servo_zeta=float(params["zeta"]), equivalent_delay_1hz_s=float(delay_eq),
        fit_percent=float(fit_pct), amplitude_rad=float(np.max(np.abs(command))),
        reaction_uncertainty_pct=float(uncertainty), samples=int(t.size),
        notes=tuple(notes), blockers=tuple(blockers))


def simulate_servo_only(t: np.ndarray, servo_tilt_rad: np.ndarray, *, inertia_rod: float,
                        stiffness: float, damping: float, reaction_inertia: float,
                        dead_time_s: float, servo_wn: float, servo_zeta: float,
                        pod_gravity: float = 0.0) -> np.ndarray:
    """模型本身的 θ̇（零初值、舵机停在首个指令）：给测试与对照用。"""
    t = np.asarray(t, dtype=float)
    command = np.asarray(servo_tilt_rad, dtype=float)
    params = {"b": damping / inertia_rod, "k": stiffness / inertia_rod, "T": dead_time_s,
              "wn": servo_wn, "zeta": servo_zeta}
    forced, static, _angle, _rate = _columns(params, t, command, float(command[0]),
                                             1.0 / _fit._sample_period(t))
    return (reaction_inertia * forced + pod_gravity * static) / inertia_rod


@dataclass(frozen=True)
class BacklashEstimate:
    #: (指令幅值 [rad], J [kg·m²], 纯延迟 [s], 1 Hz 等效延迟 [s])，按幅值升序。
    rows: tuple[tuple[float, float, float, float], ...]
    #: 大幅值极限的 J 与回差半宽 b [rad]；数据不支持回差模型时 b 为 NaN。
    reaction_limit_kg_m2: float
    backlash_rad: float
    #: T(A) = T∞ + t₁/A 的 T∞ 与 t₁。
    delay_limit_s: float
    delay_slope_s_rad: float
    notes: tuple[str, ...]


def backlash_estimate(runs) -> BacklashEstimate | None:
    """几轮不同指令幅值的舵机单独拟合 → J 与延迟随幅值的变化、回差估计（方法见 `BACKLASH_METHOD`）。

    `runs` 是若干 `(幅值 rad, J, 纯延迟 s, 1 Hz 等效延迟 s)`；不同幅值不足两个时返回 None。
    """
    rows = sorted((float(a), float(j), float(d), float(e)) for a, j, d, e in runs
                  if math.isfinite(float(a)) and float(a) > 0.0 and math.isfinite(float(j)))
    amplitudes = sorted({row[0] for row in rows})
    if len(rows) < 2 or amplitudes[-1] < AMPLITUDE_DISTINCT_RATIO * amplitudes[0]:
        return None
    inverse = np.array([1.0 / row[0] for row in rows])
    design = np.column_stack([np.ones(len(rows)), inverse])
    sign = 1.0 if sum(row[1] for row in rows) >= 0.0 else -1.0
    magnitude = np.array([sign * row[1] for row in rows])
    (p0, p1), *_ = np.linalg.lstsq(design, magnitude, rcond=None)
    delays = np.array([row[2] for row in rows])
    (t0, t1), *_ = np.linalg.lstsq(design, delays, rcond=None)
    notes = [BACKLASH_METHOD]
    backlash = _NAN
    if p0 > 0.0:
        backlash = -p1 / p0
        if backlash <= 0.0:
            notes.append("小幅值的 |J| 不比大幅值小：这几轮看不出回差（或被其它非线性盖过），b 不报。")
            backlash = _NAN
        elif backlash >= amplitudes[0]:
            notes.append(f"估出的回差半宽 {math.degrees(backlash):.2f}° 不小于最小幅值：最小幅值那轮"
                         "大半落在空程里，结论只作量级。")
    else:
        notes.append("大幅值极限的 J 符号与各轮不一致，回差模型描述不了这组数据。")
    return BacklashEstimate(rows=tuple(rows), reaction_limit_kg_m2=float(sign * p0),
                            backlash_rad=float(backlash), delay_limit_s=float(t0),
                            delay_slope_s_rad=float(t1), notes=tuple(notes))


__all__ = ["BACKLASH_METHOD", "BacklashEstimate", "ServoFit", "backlash_estimate",
           "fit_servo_only", "predicted_reaction_couple", "rod_axis_lever",
           "simulate_servo_only"]
