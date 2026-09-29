"""由辨识模型合成候选增益，并**用真实 C 控制器**闭环验证。

两条纪律，都是针对老那套的具体毛病：

1. **增益不是拍出来的。** 老面板写的是 `kp = 0.35/|K|`，没有任何带宽或相位裕度
   的依据。这里从辨出来的 (I, c, T_d) 出发解析地给，并把"为什么是这个数"写进
   返回值里。
2. **合成完必须在真控制器上跑一遍。** `tools/sim_xz/controller_bridge` 能把在飞
   的那套 C 四环编译成 host DLL 并按名读写全部 `coax.*` 参数。不跑这一步的话，
   合成的增益只在纸上的二阶模型里成立——而纸上的模型没有限幅、没有积分饱和、
   没有调度分频。

实测杆高的台架用 `synthesise_rate_shaped`：在辨出的飞行 TWD 对象上直接搜 kp，
按增益/相位裕度取值；`synthesise_rate`（延迟 → 带宽公式）只留给旧工况与对照。

**永不自动写 Flash。** 候选值经 `SYSID PARAM` 只写 RAM 实时验证，落盘是作者事后的
单独动作。程序替人做这个决定，等于把一组还没飞过的增益变成下次上电的默认值。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

#: 带延迟的系统里，穿越频率乘以总延迟就是延迟贡献的相位滞后（rad）。
#: 留 45° 相位裕度给延迟，即 ω_c·T_d ≤ π/4。
DELAY_PHASE_BUDGET_RAD = math.pi / 4.0

#: 角速度环带宽相对"延迟允许的上限"再打的折扣。留给未建模的高频（桨叶柔性、
#: 舵机非线性），这些在 ±15° 小角度台架上量不出来，但在飞的时候都在。
RATE_BANDWIDTH_MARGIN = 0.6

#: 角度环带宽取角速度环的几分之一。时标分离是串级能当串级用的前提。
ATTITUDE_BANDWIDTH_RATIO = 0.25

#: 角速度环带宽的绝对上限 [Hz]。延迟辨成接近 0 时 π/(4·T_d) 会发散（kp 上到 1e10 量级）；
#: 2–3 ms 的控制拍、舵机与桨叶柔性都撑不起这之上的穿越频率。
RATE_BANDWIDTH_MAX_HZ = 12.0


@dataclass(frozen=True)
class RateGains:
    kp: float
    ki: float
    kd: float
    bandwidth_hz: float
    rationale: str


@dataclass(frozen=True)
class AttitudeGains:
    kp: float
    bandwidth_hz: float
    rationale: str


def rate_bandwidth_hz(delay_s: float, *, margin: float = RATE_BANDWIDTH_MARGIN) -> float:
    """延迟允许的角速度环带宽。

    总延迟 T_d 在穿越频率 ω_c 处贡献 ω_c·T_d 的相位滞后。给它 45° 的预算，
    于是 ω_c ≤ π/(4·T_d)，再乘一个余量。**带宽由延迟决定，不由惯量决定**——
    这也是为什么这套辨识必须把延迟测准，而不是沿用手册上的舵机参数。
    结果不超过 `RATE_BANDWIDTH_MAX_HZ`。
    """
    if not (delay_s > 0.0) or not math.isfinite(delay_s):
        raise ValueError("延迟必须是正的有限值；先把延迟辨出来再谈整定")
    omega_c = DELAY_PHASE_BUDGET_RAD / delay_s * margin
    return min(omega_c / (2.0 * math.pi), RATE_BANDWIDTH_MAX_HZ)


def synthesise_rate(inertia_kg_m2: float, damping_n_m_s: float, delay_s: float,
                    *, margin: float = RATE_BANDWIDTH_MARGIN) -> RateGains:
    """角速度环 PI(D)。

    被控对象在角速度输出上是一阶：I·ω̇ + c·ω = τ。要把闭环一阶极点放在 ω_c，
    比例项需要 `kp = I·ω_c − c`（抵消已有阻尼），积分项按 ω_c/5 的转折频率取
    `ki = kp·ω_c/5`——积分再快就会和延迟一起把相位吃光。

    微分项留 0：本台架上角加速度只能由陀螺差分得到，差分把噪声放大 ω 倍，
    而这条链上真正缺的是延迟补偿而不是阻尼。要加也应该先加前馈。
    """
    if inertia_kg_m2 <= 0.0:
        raise ValueError("惯量必须为正")
    bandwidth = rate_bandwidth_hz(delay_s, margin=margin)
    omega_c = 2.0 * math.pi * bandwidth
    kp = inertia_kg_m2 * omega_c - damping_n_m_s
    if kp <= 0.0:
        # 阻尼已经比目标带宽要求的还大：这时对象本身就够慢，比例项取一个正的小值。
        kp = inertia_kg_m2 * omega_c * 0.25
    ki = kp * omega_c / 5.0
    capped = "，已封顶" if bandwidth >= RATE_BANDWIDTH_MAX_HZ else ""
    return RateGains(
        kp=kp, ki=ki, kd=0.0, bandwidth_hz=bandwidth,
        rationale=(
            f"总延迟 {delay_s * 1000:.1f} ms 允许的穿越频率 "
            f"{bandwidth:.2f} Hz（45° 相位预算 × {margin:g} 余量，"
            f"上限 {RATE_BANDWIDTH_MAX_HZ:g} Hz{capped}）；"
            f"kp = I·ωc − c = {inertia_kg_m2:.5f}×{omega_c:.2f} − "
            f"{damping_n_m_s:.5f}；ki 按 ωc/5 转折取。kd 留 0："
            "角加速度只能靠陀螺差分，噪声放大得不偿失。"))


def synthesise_attitude(rate: RateGains,
                        ratio: float = ATTITUDE_BANDWIDTH_RATIO) -> AttitudeGains:
    """角度环 P。带宽取角速度环的 1/4，保证时标分离。

    比例增益就是带宽本身（单位 rad/s per rad）：角度环输出是角速度目标，
    P 控制器的闭环极点正好落在 kp 处。
    """
    bandwidth = rate.bandwidth_hz * ratio
    kp = 2.0 * math.pi * bandwidth
    return AttitudeGains(
        kp=kp, bandwidth_hz=bandwidth,
        rationale=(f"角度环带宽取角速度环的 {ratio:g} 倍 = {bandwidth:.2f} Hz，"
                   "时标分离是串级能当串级用的前提。"))


# ---------------------------------------------------------------- 按对象回路整形（TWD）

#: 回路整形的裕度要求。
GAIN_MARGIN_MIN_DB = 6.0
PHASE_MARGIN_MIN_DEG = 45.0

#: PI 的积分转折取穿越频率的这个比例：ki = kp·ωc·ratio。
INTEGRAL_CORNER_RATIO = 0.2

#: 裕度评估的频率网格 [Hz]：对数等距，0.01–100 Hz。
_MARGIN_GRID = (0.01, 100.0, 4000)


@dataclass(frozen=True)
class FeedbackNotch:
    """控制用陀螺上的转速陷波（固件 Driver/Src/drv_rpm_notch.c），放在反馈通路里::

        H(z) = ∏ (b0 + b1·z⁻¹ + b0·z⁻²) / (1 + b1·z⁻¹ + a2·z⁻²)，z = e^{j2πf/fs}

    每个中心一个 RBJ 陷波（与固件同一组闭式系数），直流增益恰为 1。拟合出的纯延迟 T
    含 80 Hz 低通（记录的陀螺在低通之后）却**不含**陷波（记录在陷波之前），所以整定
    模型要单独乘上它。
    """

    centers_hz: tuple        # one entry per cascaded notch
    q: float
    fs_hz: float

    def __post_init__(self):
        if not (self.fs_hz > 0.0 and math.isfinite(self.fs_hz)):
            raise ValueError("陷波采样率必须为正")
        if not (self.q > 0.0 and math.isfinite(self.q)):
            raise ValueError("陷波 Q 必须为正")
        for center in self.centers_hz:
            if not (0.0 < center < 0.5 * self.fs_hz):
                raise ValueError(f"陷波中心 {center} Hz 必须在 (0, fs/2) 内")

    def response(self, freq_hz):
        import numpy as np

        z_inv = np.exp(-2j * np.pi * np.asarray(freq_hz, dtype=float) / self.fs_hz)
        total = np.ones_like(z_inv)
        for center in self.centers_hz:
            w0 = 2.0 * math.pi * center / self.fs_hz
            alpha = math.sin(w0) / (2.0 * self.q)
            k = 1.0 / (1.0 + alpha)
            b0, b1, a2 = k, -2.0 * math.cos(w0) * k, (1.0 - alpha) * k
            total = total * ((b0 + b1 * z_inv + b0 * z_inv * z_inv)
                             / (1.0 + b1 * z_inv + a2 * z_inv * z_inv))
        return total


#: 部署中的转速陷波按**最坏位置**建模：两个电机的 1x 都停在 min_hz + fade_hz = 70 Hz
#: （全权重的最低频率；weight(f)/f 在这里最大，10 Hz 处相位代价最大），Q3，fs 1000 Hz。
#: 不需要悬停转速数据，陷波开或关都安全。与 App/Inc/app_rpm_notch.h 的默认值由测试钉住。
DEPLOYED_RPM_NOTCH = FeedbackNotch(centers_hz=(70.0, 70.0), q=3.0, fs_hz=1000.0)


@dataclass(frozen=True)
class FlightPlant:
    """飞行中绕质心的速率对象，力矩按固件单位（控制器输出的就是它）::

        P(s) = e^{−sT} · ωs²/(s² + 2ζs·ωs·s + ωs²) · (κ + ρ·s²) / (I_cg·s)

    舵机二阶 + 纯延迟之后，倾转推力给真实力矩 κ·u，舵机甩动 349 g 组件的反作用
    （"尾巴摇狗"）再加 ρ·u''——飞行中也存在。ρ 按纯力偶的最坏情况原样搬到质心。
    `feedback_notch` 给出时再乘上反馈通路里的转速陷波（None 与原来逐位相同）。
    """

    inertia_cg_kg_m2: float
    torque_model_scale: float
    dead_time_s: float
    servo_wn_rad_s: float
    servo_zeta: float
    reaction_couple_s2: float
    feedback_notch: FeedbackNotch | None = None

    def response(self, freq_hz):
        import numpy as np

        s = 2j * np.pi * np.asarray(freq_hz, dtype=float)
        wn, zeta = self.servo_wn_rad_s, self.servo_zeta
        servo = wn * wn / (s * s + 2.0 * zeta * wn * s + wn * wn)
        plant = (np.exp(-s * self.dead_time_s) * servo
                 * (self.torque_model_scale + self.reaction_couple_s2 * s * s)
                 / (self.inertia_cg_kg_m2 * s))
        if self.feedback_notch is None:
            return plant
        return plant * self.feedback_notch.response(freq_hz)


@dataclass(frozen=True)
class LoopMargins:
    gain_margin_db: float
    phase_margin_deg: float
    #: 第一个增益穿越频率（主穿越）；没有穿越时为 NaN。
    crossover_hz: float


@dataclass(frozen=True)
class ShapedRateGains(RateGains):
    """按对象回路整形得到的角速度环 PI，附带在该对象上的裕度。"""

    gain_margin_db: float = float("nan")
    phase_margin_deg: float = float("nan")
    crossover_hz: float = float("nan")
    #: 是否同时满足增益/相位裕度与带宽上限。
    feasible: bool = False


def _margin_grid():
    import numpy as np

    low, high, count = _MARGIN_GRID
    return np.logspace(math.log10(low), math.log10(high), count)


def _first_crossover(freq, magnitude) -> float:
    import numpy as np

    above = magnitude >= 1.0
    index = np.where(above[:-1] & ~above[1:])[0]
    return float(freq[index[0]]) if index.size else float("nan")


def loop_margins(kp: float, ki: float, plant: FlightPlant, freq_hz=None) -> LoopMargins:
    """L = (kp + ki/s)·P 的增益裕度、相位裕度与主穿越频率。

    相位裕度取所有增益穿越里最小的一个（TWD 零点之后回路增益可能再抬头）。
    增益裕度取主穿越之上所有 −180° 穿越里最小的一个：主穿越之下的 −180° 穿越属于
    积分器低频的条件稳定，与"加大增益会不会在高频失稳"无关。
    """
    import numpy as np

    freq = _margin_grid() if freq_hz is None else np.asarray(freq_hz, dtype=float)
    s = 2j * np.pi * freq
    loop = (kp + ki / s) * plant.response(freq)
    magnitude = np.abs(loop)
    crossover = _first_crossover(freq, magnitude)
    crossings = np.where(np.diff(np.sign(magnitude - 1.0)) != 0)[0]
    if crossings.size:
        angles = np.degrees(np.angle(loop[crossings]))
        phase_margin = float(np.min(np.where(angles + 180.0 > 180.0, angles - 180.0,
                                             angles + 180.0)))
    else:
        phase_margin = float("nan")
    imag = np.imag(loop)
    phase_cross = np.where((np.diff(np.sign(imag)) != 0) & (np.real(loop[:-1]) < 0.0))[0]
    if math.isfinite(crossover):
        phase_cross = phase_cross[freq[phase_cross] >= crossover]
    gains = magnitude[phase_cross]
    gain_margin = (float(np.min(-20.0 * np.log10(np.maximum(gains, 1e-300))))
                   if gains.size else float("inf"))
    return LoopMargins(gain_margin_db=gain_margin, phase_margin_deg=phase_margin,
                       crossover_hz=crossover)


def _pi_for(kp: float, plant: FlightPlant, freq, response) -> tuple[float, LoopMargins]:
    """给定 kp：按 P 回路的穿越频率 ωc 取 ki = kp·ωc/5，再算 PI 回路的裕度。"""
    import numpy as np

    crossover = _first_crossover(freq, kp * np.abs(response))
    if not math.isfinite(crossover):
        return 0.0, LoopMargins(float("inf"), float("nan"), float("nan"))
    ki = kp * 2.0 * math.pi * crossover * INTEGRAL_CORNER_RATIO
    return ki, loop_margins(kp, ki, plant, freq)


def synthesise_rate_shaped(plant: FlightPlant, *,
                           gain_margin_min_db: float = GAIN_MARGIN_MIN_DB,
                           phase_margin_min_deg: float = PHASE_MARGIN_MIN_DEG,
                           max_bandwidth_hz: float = RATE_BANDWIDTH_MAX_HZ) -> ShapedRateGains:
    """在 kp 上搜索：取同时满足 GM ≥ 6 dB、PM ≥ 45°、穿越 ≤ 12 Hz 的最大 kp。

    不再按"延迟 → 带宽"公式给增益：TWD 对象在舵机频段（约 10 Hz）的回路增益并不
    随频率下降，按刚体积分器 + 延迟合成的 kp 在那里的增益裕度约为 0 dB。
    满足不了时返回裕度最宽的那个 kp，并标 `feasible=False`。
    """
    import numpy as np

    if not (plant.inertia_cg_kg_m2 > 0.0 and math.isfinite(plant.inertia_cg_kg_m2)):
        raise ValueError("绕质心惯量必须为正")
    if not (plant.torque_model_scale > 0.0 and plant.servo_wn_rad_s > 0.0):
        raise ValueError("力矩模型比例与舵机带宽必须为正")
    freq = _margin_grid()
    response = plant.response(freq)

    def ok(margins: LoopMargins) -> bool:
        return (margins.gain_margin_db >= gain_margin_min_db
                and margins.phase_margin_deg >= phase_margin_min_deg
                and margins.crossover_hz <= max_bandwidth_hz)

    candidates = np.logspace(-4, 1, 161)
    evaluated = [(kp, *_pi_for(kp, plant, freq, response)) for kp in candidates]
    feasible = [index for index, (_kp, _ki, margins) in enumerate(evaluated) if ok(margins)]
    if feasible:
        index = max(feasible)
        low_kp = evaluated[index][0]
        high_kp = candidates[index + 1] if index + 1 < candidates.size else low_kp
        for _ in range(30):               # 在最后一个可行点与下一个网格点之间二分
            middle = math.sqrt(low_kp * high_kp)
            if ok(_pi_for(middle, plant, freq, response)[1]):
                low_kp = middle
            else:
                high_kp = middle
        kp = low_kp
        ki, margins = _pi_for(kp, plant, freq, response)
        verdict = (f"取满足 GM ≥ {gain_margin_min_db:g} dB、PM ≥ {phase_margin_min_deg:g}°、"
                   f"穿越 ≤ {max_bandwidth_hz:g} Hz 的最大 kp")
        is_feasible = True
    else:
        def score(item):
            _kp, _ki, margins = item
            if not math.isfinite(margins.crossover_hz):
                return -math.inf
            return min(margins.gain_margin_db / gain_margin_min_db,
                       margins.phase_margin_deg / phase_margin_min_deg)
        kp, ki, margins = max(evaluated, key=score)
        verdict = (f"此对象下无法同时满足 {gain_margin_min_db:g} dB/{phase_margin_min_deg:g}° "
                   "裕度，给出的是裕度最宽的一组，不要上机")
        is_feasible = False
    crossover = margins.crossover_hz
    notch = plant.feedback_notch
    notch_note = "" if notch is None else (
        "反馈通路另乘控制用陀螺的转速陷波（中心 "
        + "/".join(f"{center:g}" for center in notch.centers_hz)
        + f" Hz，Q{notch.q:g}，fs {notch.fs_hz:g} Hz；10 Hz 处相位 "
        f"{math.degrees(float(np.angle(notch.response(10.0)))):.1f}°，记录在陷波之前，T 里没有它）；")
    return ShapedRateGains(
        kp=float(kp), ki=float(ki), kd=0.0,
        bandwidth_hz=float(crossover) if math.isfinite(crossover) else 0.0,
        gain_margin_db=float(margins.gain_margin_db),
        phase_margin_deg=float(margins.phase_margin_deg),
        crossover_hz=float(crossover), feasible=is_feasible,
        rationale=(
            "按对象回路整形：飞行对象 P(s) = e^{−sT}·ωs²/(s²+2ζs·ωs·s+ωs²)·(κ+ρs²)/(I_cg·s)，"
            f"T = {plant.dead_time_s * 1000:.1f} ms，ωs = {plant.servo_wn_rad_s:.1f} rad/s，"
            f"ζs = {plant.servo_zeta:.2f}，κ = {plant.torque_model_scale:.3f}，"
            f"ρ = {plant.reaction_couple_s2:.5f} s²（纯力偶最坏情况），"
            f"I_cg = {plant.inertia_cg_kg_m2:.5f} kg·m²；{notch_note}PI 的 ki = kp·ωc/5。"
            f"{verdict}：kp = {kp:.4f}，ki = {ki:.4f}，穿越 {crossover:.2f} Hz，"
            f"GM {margins.gain_margin_db:.1f} dB，PM {margins.phase_margin_deg:.1f}°。"
            "kd 留 0：角加速度只能靠陀螺差分，噪声放大得不偿失。"))


# ---------------------------------------------------------------- 闭环验证


@dataclass(frozen=True)
class ClosedLoopScore:
    overshoot_percent: float
    settle_s: float
    steady_error: float
    stable: bool
    note: str

    @property
    def acceptable(self) -> bool:
        return self.stable and self.overshoot_percent < 25.0


def score_step_response(t: list[float], response: list[float],
                        target: float) -> ClosedLoopScore:
    """一条阶跃响应的打分。判据写成人话，不是一个复合分数。

    复合分数看起来专业，但调参的人需要知道"是超调大还是稳不下来"，
    一个 0.73 分回答不了这个问题。
    """
    if not t or len(t) != len(response):
        raise ValueError("时间与响应长度必须一致")
    if target == 0.0:
        raise ValueError("阶跃目标不能为 0")

    peak = max(response) if target > 0 else min(response)
    overshoot = (peak - target) / target * 100.0
    overshoot = max(0.0, overshoot)

    band = abs(target) * 0.05
    settle = float("nan")
    for index in range(len(t) - 1, -1, -1):
        if abs(response[index] - target) > band:
            if index + 1 < len(t):
                settle = t[index + 1]
            break
    else:
        settle = t[0]

    steady = abs(response[-1] - target)
    diverging = abs(response[-1]) > abs(target) * 5.0
    growing = (len(response) > 20
               and abs(response[-1]) > abs(response[len(response) // 2]) * 2.0)
    stable = not (diverging or growing)

    note = "稳定" if stable else "发散：这组增益不能上机"
    if stable and not math.isfinite(settle):
        note = "在这段时长里没有进入 ±5% 带"
    return ClosedLoopScore(overshoot_percent=overshoot, settle_s=settle,
                           steady_error=steady, stable=stable, note=note)


def verify_with_real_controller(rate: RateGains, attitude: AttitudeGains,
                                *, bridge=None, duration_s: float = 2.0,
                                dt_s: float = 0.002,
                                target_pitch_rad: float = 0.15) -> ClosedLoopScore:
    """把候选增益写进真实 C 控制器，跑一条角度阶跃。

    `bridge` 留空时自行构造 `tools/sim_xz/controller_bridge.ControllerBridge`——
    它把在飞的那套四环编译成 host DLL，参数按名读写。没有它就只能在纸上的二阶
    模型里验证，而纸上的模型没有限幅、没有积分饱和、没有调度分频。
    """
    if bridge is None:
        import sys
        from pathlib import Path

        tools = Path(__file__).resolve().parents[1]
        if str(tools) not in sys.path:
            sys.path.insert(0, str(tools))
        from sim_xz.controller_bridge import ControllerBridge

        bridge = ControllerBridge()

    # roll / pitch 写同一个值：45° 杆上辨出来的就是 XY 共用的那个惯量。
    applied = []
    for name, value in (("coax.rate_roll_kp", rate.kp),
                        ("coax.rate_pitch_kp", rate.kp),
                        ("coax.rate_roll_ki", rate.ki),
                        ("coax.rate_pitch_ki", rate.ki),
                        ("coax.rate_roll_kd", rate.kd),
                        ("coax.rate_pitch_kd", rate.kd),
                        ("coax.att_roll_kp", attitude.kp),
                        ("coax.att_pitch_kp", attitude.kp)):
        if bridge.set_param(name, value):
            applied.append(name)
    bridge.reset()

    times, pitch = [], []
    state_pitch = 0.0
    state_rate = 0.0
    steps = int(duration_s / dt_s)
    for index in range(steps):
        out = bridge.step(
            pitch_rad=state_pitch, pitch_rate_rad_s=state_rate,
            target_pitch_rad=target_pitch_rad, dt_s=dt_s,
            position_control_bypass=1, direct_attitude_target_valid=1,
            manual_total_force_n=13.4, manual_total_force_valid=1)
        # 极简刚体积分。这里只要求"控制器自己不发散"，被控对象的细节由
        # tools/sim_xz/physics.py 那套负责，不在本文件重造。
        moment = out.moment_achieved_n_m[1]
        state_rate += moment / 0.019 * dt_s
        state_pitch += state_rate * dt_s
        times.append(index * dt_s)
        pitch.append(state_pitch)

    score = score_step_response(times, pitch, target_pitch_rad)
    if not applied:
        return ClosedLoopScore(
            overshoot_percent=score.overshoot_percent, settle_s=score.settle_s,
            steady_error=score.steady_error, stable=score.stable,
            note="控制器没有接受任何候选参数名，这次验证说明不了问题")
    return score
