"""激励幅值 → 预计舵机摆幅，以及让舵机摆约 10° 的建议幅值。纯函数，不碰 Tk。

为什么幅值不能写死：固件把期望角速度的变化率乘假定惯量当力矩（τ = I·α），再按**当前板子**
的倾转力臂反解成舵机倾角 δ = asin(τ/(L·T))。几何力矩模型（2026-09-27 起）的
L = cg_z_m − servoN_axis_z_m 随重心能差好几倍：L = 0.0354 m 时 0.065 rad/s 摆约 10°，
重心改到 −0.01（L = 0.12 m）后同一幅值只摆约 3°。而舵机有回差：2026-09-27 的幅值阶梯
（旧模型 0.03/0.06/0.10/0.15/0.20 rad/s）带内拟合度 15/35/52/85/82%，摆幅不到 5～6° 的
轮次基本是回差里的非线性。

近似（用 2026-09-27 旧模型双脉冲实录核过：amp 0.15、L 0.0825 m、T 7.4 N、斜坡 150 ms、
I 0.051 → 记录力矩峰值 0.102 N·m、舵机约 9.6°）：

* 双脉冲、PRBS：一次过渡在斜坡时间内把设定值从 +amp 翻到 −amp，α 峰值 = 2·amp/斜坡；
  阶跃只从 0 升到 amp，α 峰值 = amp/斜坡。
* 扫频：α(f) = amp·2πf，摆幅在起始频率最小、终止频率最大。终止频率处舵机转速 2πf1·δ
  还得远低于舵机约 400°/s 的上限（被转速限住同样是非线性），建议值按 300°/s 封顶。
* L 取杆轴方向的等效力臂 |sin²ψ·L_pitch + cos²ψ·L_roll|，T 取程序油门的目标合推力。

摆幅是沿杆轴的倾角，与 v3 记录里 servo_tilt 同口径。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

#: 建议幅值瞄准的舵机摆幅 [deg]（2026-09-27 阶梯里约 9.6° 那档拟合度最高）。
TARGET_SWING_DEG = 10.0
#: 小于它会落在舵机回差里；大于 HIGH_SWING_DEG 离行程和饱和太近。两者之间不另给建议。
BACKLASH_SWING_DEG = 6.0
HIGH_SWING_DEG = 15.0
#: 开始前提醒（不阻止）的门槛 [deg]。
START_WARN_SWING_DEG = 5.0
#: 舵机转速上限约 400°/s；扫频建议值只用到它的 3/4。
SERVO_SLEW_LIMIT_DPS = 400.0
SLEW_SAFE_DPS = 300.0
#: 固件按 1 mrad/s 回显幅值（amp_mrad_s），建议值取到 0.001；上限同 `Excitation` 的 5 rad/s。
AMP_STEP_RAD_S = 0.001
MAX_AMP_RAD_S = 5.0

#: 斜坡类剖面：一次过渡里 α 峰值 = 系数 × amp / 斜坡。
_RAMP_FACTOR = {"step": 1.0, "doublet": 2.0, "prbs": 2.0}


@dataclass(frozen=True)
class SwingEstimate:
    profile: str
    amplitude_rad_s: float
    lever_m: float
    thrust_n: float
    inertia_kg_m2: float
    #: 最大摆幅 [deg]：斜坡类是过渡时的峰值，扫频是终止频率处。
    swing_deg: float
    #: 让最大摆幅约 10°（扫频再受转速封顶）的幅值 [rad/s]，已取到 0.001。
    suggested_amp_rad_s: float
    #: 建议幅值对应的最大摆幅 [deg]。
    suggested_swing_deg: float
    #: 建议值瞄准的最大摆幅 [deg]：一般 10°，扫频被舵机转速封顶时更小。
    target_swing_deg: float = TARGET_SWING_DEG
    #: 扫频专用：起止频率 [Hz]、起始频率处摆幅 [deg]、终止频率处舵机转速 [deg/s]。
    f0_hz: float | None = None
    f1_hz: float | None = None
    swing_f0_deg: float | None = None
    slew_dps: float | None = None

    @property
    def slew_too_fast(self) -> bool:
        return self.slew_dps is not None and self.slew_dps > SLEW_SAFE_DPS

    @property
    def slew_limited(self) -> bool:
        """建议值是被舵机转速压下来的（瞄准不到 10°）。"""
        return self.target_swing_deg < TARGET_SWING_DEG

    @property
    def needs_new_amplitude(self) -> bool:
        return (not BACKLASH_SWING_DEG <= self.swing_deg <= HIGH_SWING_DEG) or self.slew_too_fast

    @property
    def warn_before_start(self) -> bool:
        return self.swing_deg < START_WARN_SWING_DEG


def rod_lever_m(levers: Sequence[float] | None, psi_deg: float) -> float | None:
    """杆轴方向的等效力臂 |sin²ψ·L_pitch + cos²ψ·L_roll| [m]。

    `levers` 是 `firmware_tilt_levers` 的 (roll, pitch)；ψ 为杆轴方位角 [deg]。算不出返回 None。
    """
    if levers is None:
        return None
    try:
        roll, pitch = (float(value) for value in levers)
        psi = math.radians(float(psi_deg))
    except (TypeError, ValueError):
        return None
    lever = abs(math.sin(psi) ** 2 * pitch + math.cos(psi) ** 2 * roll)
    return lever if math.isfinite(lever) and lever > 1e-6 else None


def swing_from_torque_deg(torque_n_m: float, lever_m: float, thrust_n: float) -> float:
    """δ = asin(τ/(L·T)) [deg]；力矩超过 L·T 时封在 90°（固件会按饱和中止）。"""
    return math.degrees(math.asin(min(abs(torque_n_m) / (lever_m * thrust_n), 1.0)))


def format_amp(value: float) -> str:
    """幅值写回输入框的样子：最多 3 位小数，去掉尾零（0.227、0.2）。"""
    return f"{value:.3f}".rstrip("0").rstrip(".")


def _positive(*values: float) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0.0 for v in values)


def estimate(profile: str, amplitude_rad_s: float, *, ramp_ms: float, chirp_f0_hz: float,
             chirp_f1_hz: float, lever_m: float, thrust_n: float,
             inertia_kg_m2: float) -> SwingEstimate | None:
    """当前激励设置的预计舵机摆幅与建议幅值；剖面未知或数不合法返回 None。"""
    if not _positive(amplitude_rad_s, lever_m, thrust_n, inertia_kg_m2):
        return None
    f0 = f1 = low = slew = None
    target = TARGET_SWING_DEG
    if profile == "chirp":
        if not (_positive(chirp_f0_hz, chirp_f1_hz) and chirp_f1_hz > chirp_f0_hz):
            return None
        f0, f1 = float(chirp_f0_hz), float(chirp_f1_hz)
        torque_per_amp = inertia_kg_m2 * 2.0 * math.pi * f1
        low = swing_from_torque_deg(inertia_kg_m2 * 2.0 * math.pi * f0 * amplitude_rad_s,
                                    lever_m, thrust_n)
        target = min(TARGET_SWING_DEG, SLEW_SAFE_DPS / (2.0 * math.pi * f1))
    elif profile in _RAMP_FACTOR:
        if not _positive(ramp_ms):
            return None
        torque_per_amp = inertia_kg_m2 * _RAMP_FACTOR[profile] / (ramp_ms * 1e-3)
    else:
        return None
    swing = swing_from_torque_deg(torque_per_amp * amplitude_rad_s, lever_m, thrust_n)
    if f1 is not None:
        slew = 2.0 * math.pi * f1 * swing
    wanted = math.sin(math.radians(target)) * lever_m * thrust_n / torque_per_amp
    # 被转速封顶时向下取整：进位会让建议值本身又超过转速线。
    steps = wanted / AMP_STEP_RAD_S
    steps = max(1, math.floor(steps + 1e-9) if target < TARGET_SWING_DEG else round(steps))
    suggested = round(min(MAX_AMP_RAD_S, steps * AMP_STEP_RAD_S), 3)
    return SwingEstimate(
        profile=profile, amplitude_rad_s=float(amplitude_rad_s), lever_m=float(lever_m),
        thrust_n=float(thrust_n), inertia_kg_m2=float(inertia_kg_m2), swing_deg=swing,
        suggested_amp_rad_s=suggested,
        suggested_swing_deg=swing_from_torque_deg(torque_per_amp * suggested, lever_m, thrust_n),
        target_swing_deg=target, f0_hz=f0, f1_hz=f1, swing_f0_deg=low, slew_dps=slew)


def swing_text(est: SwingEstimate) -> str:
    """幅值旁那行灰字的第一句。"""
    where = ""
    if est.f1_hz is not None:
        where = f"扫到 {est.f1_hz:g} Hz 时；{est.f0_hz:g} Hz 起步时约 {est.swing_f0_deg:.1f}°；"
    text = (f"预计舵机摆幅约 {est.swing_deg:.1f}°"
            f"（{where}小于 {BACKLASH_SWING_DEG:.0f}° 会落在舵机回差里）")
    if est.slew_too_fast:
        text += (f"；{est.f1_hz:g} Hz 处舵机要转约 {est.slew_dps:.0f}°/s，"
                 f"离舵机上限约 {SERVO_SLEW_LIMIT_DPS:.0f}°/s 太近")
    return text


def suggestion_text(est: SwingEstimate) -> str:
    """「建议幅值 Y」及它对应的舵机摆幅。"""
    text = f"建议幅值 {format_amp(est.suggested_amp_rad_s)}（舵机约 {est.suggested_swing_deg:.0f}°"
    if est.slew_limited:
        text += f"，{est.f1_hz:g} Hz 处舵机转速不超过 {SLEW_SAFE_DPS:.0f}°/s"
    return text + "）"


__all__ = [
    "BACKLASH_SWING_DEG", "HIGH_SWING_DEG", "SERVO_SLEW_LIMIT_DPS", "SLEW_SAFE_DPS",
    "START_WARN_SWING_DEG", "SwingEstimate", "TARGET_SWING_DEG", "estimate", "format_amp",
    "rod_lever_m", "suggestion_text", "swing_from_torque_deg", "swing_text",
]
