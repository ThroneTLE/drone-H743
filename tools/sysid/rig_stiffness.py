"""台架刚度的挂砝码试验：把"机体吊在杆下的回中刚度"直接量出来，代替只算重力的 m·g·d。

为什么要量：杆上辨识里重力是**唯一已知的力矩**，I_杆 由 K/I_杆 = (2π·f摆)² 标定，κ、k 都
跟着 I_杆 走。K 只按 m·g·d 算时，线缆、杆的夹持这些额外的回中刚度会被当成更大的 I_杆
之外的东西——2026-09-27 在实测重心（d = 0.05 m）下 k 只有 0.45，额外刚度是嫌疑之一。

试验：机体静止在杆上，读绕杆转角 θ₀（俯仰杆上就是俯仰角，其它方位见下）；在杆的高度上、
离杆轴水平 x 处挂质量 m_w 的砝码，一次挂一侧（θ_前）、一次挂另一侧（θ_后）。砝码的重力矩
m_w·g·x 被台架刚度平衡::

    K = m_w·g·x / ((θ_前 − θ_后)/2)          （两边一起用，零点误差对消）
    K_前 = m_w·g·x / (θ_前 − θ₀)，K_后 = m_w·g·x / (θ₀ − θ_后)   （各自核对）

前提（界面上照写）：小角度（sin θ ≈ θ，±10° 内误差 < 0.6%）；砝码挂在**杆的高度**上——
这样它的重力矩只取决于水平距离 x，不随摆角变化，也不改变台架本身的刚度。

**杆轴方位。** 上面的 θ 是**绕杆**的转角。杆轴沿 (cosψ, sinψ) 时，绕杆转 θ 在姿态上显示为
横滚 θ·cosψ、俯仰 θ·sinψ（固件的杆轴角 = 横滚·cosψ + 俯仰·sinψ 正是它的反过来）。所以
ψ = 45° 的斜杆上直接拿俯仰读数当 θ，K 会大 1/sin45° = 1.41 倍；ψ = 0° 的横滚杆上前后挂
砝码根本不产生绕杆力矩。这里按 ψ 选读数（|sinψ| ≥ |cosψ| 读俯仰，否则读横滚），读数除以
投影换回绕杆转角；砝码要挂在与杆轴垂直的水平方向上（`hanging_sides`）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

GRAVITY_M_S2 = 9.80665

#: 前后单边估计相差超过它就提示（两边不对称：零点读数、砝码位置或非线性）。
SIDE_DISAGREEMENT_MAX = 0.15

ASSUMPTIONS = ("小角度（sin θ ≈ θ）；砝码挂在杆的高度上、与杆轴垂直的水平方向，水平距离从杆轴"
               "量起（它的重力矩不随摆角变化，也不改变台架刚度）；机体只绕杆转（姿态读数按杆轴方位"
               "换成绕杆转角）；每次等机体静止再读角度。")


def reading_axis(azimuth_deg: float) -> tuple[str, float]:
    """(读哪个姿态角, 读数 / 绕杆转角)：|sinψ| ≥ |cosψ| 读俯仰（投影 sinψ），否则读横滚（cosψ）。
    投影的绝对值至少 0.707，不会除出放大很多倍的数。"""
    psi = math.radians(float(azimuth_deg))
    if not math.isfinite(psi):
        raise ValueError("杆轴方位角要填有限数字（「台架」里的 ψ）")
    sin_psi, cos_psi = math.sin(psi), math.cos(psi)
    return ("俯仰", sin_psi) if abs(sin_psi) >= abs(cos_psi) else ("横滚", cos_psi)


def hanging_sides(azimuth_deg: float) -> tuple[str, str]:
    """砝码该挂的两侧（与杆轴垂直的水平方向，FLU：+x 前、+y 左）：(第一侧, 第二侧)。

    第一侧取让读数轴那一分量为正的方向（读俯仰时偏"前"，读横滚时偏"左"）。"""
    psi = math.radians(float(azimuth_deg))
    x, y = math.sin(psi), -math.cos(psi)          # 与杆轴 (cosψ, sinψ) 垂直
    if (x if reading_axis(azimuth_deg)[0] == "俯仰" else y) < 0.0:
        x, y = -x, -y

    def name(vx: float, vy: float) -> str:
        side = ("左" if vy > 0.38 else "右" if vy < -0.38 else "")
        return side + ("前" if vx > 0.38 else "后" if vx < -0.38 else "")

    return name(x, y), name(-x, -y)


@dataclass(frozen=True)
class StiffnessEstimate:
    #: 前后一起算的台架总刚度 K [N·m/rad]。
    stiffness_n_m_rad: float
    #: 只用前/后与不挂砝码角度各算一次；没填不挂砝码的角度时 NaN。
    front_n_m_rad: float
    back_n_m_rad: float
    #: |K_前 − K_后| / 两者均值；算不出来时 NaN。
    side_disagreement: float
    notes: tuple[str, ...] = ()
    #: 读的是哪个姿态角、读数 / 绕杆转角（按杆轴方位 ψ）。
    reading: str = "俯仰"
    projection: float = 1.0

    @property
    def consistent(self) -> bool:
        return not (self.side_disagreement > SIDE_DISAGREEMENT_MAX)


def _finite(value, what: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"「{what}」要填数字") from None
    if not math.isfinite(number):
        raise ValueError(f"「{what}」要填有限数字")
    return number


def weight_test_stiffness(weight_g, distance_m, front_deg, back_deg,
                          level_deg=None, *, azimuth_deg: float = 90.0) -> StiffnessEstimate:
    """挂砝码试验 → 台架刚度。角度单位 deg，是面板上的姿态读数（按杆轴方位 ψ 读俯仰或横滚，
    见 `reading_axis`），这里除以投影换成绕杆转角；`distance_m` 是砝码到杆轴的垂直水平距离。
    不合理的输入抛中文 ValueError。"""
    reading, projection = reading_axis(azimuth_deg)
    weight = _finite(weight_g, "砝码质量")
    distance = abs(_finite(distance_m, "水平距离"))
    front = _finite(front_deg, f"砝码挂第一侧的{reading}角") / projection
    back = _finite(back_deg, f"砝码挂第二侧的{reading}角") / projection
    if weight <= 0.0:
        raise ValueError("砝码质量要填正数（单位 g）")
    if distance < 1e-3 or distance > 1.0:
        raise ValueError("水平距离要在 0.001～1 m 之间（单位 m，从杆轴量到砝码）")
    moment = weight * 1e-3 * GRAVITY_M_S2 * distance
    half = math.radians(front - back) / 2.0
    if abs(half) < math.radians(0.05):
        raise ValueError("挂前、挂后的角度几乎一样：砝码太轻或读数没变，算不出刚度")
    stiffness = moment / abs(half)
    notes = []
    front_k = back_k = disagreement = float("nan")
    if abs(abs(projection) - 1.0) > 1e-3:
        notes.append(f"杆轴方位 ψ = {float(azimuth_deg):g}°：绕杆转 θ 时{reading}只显示 "
                     f"{abs(projection):.2f}·θ，已按 θ = {reading}/{abs(projection):.2f} 换算"
                     "（直接拿读数当 θ 会把 K 算大这个倍数的倒数）。")
    if level_deg not in (None, ""):
        level = _finite(level_deg, f"不挂砝码的{reading}角") / projection
        front_delta = math.radians(front - level)
        back_delta = math.radians(level - back)
        if front_delta * back_delta <= 0.0 or min(abs(front_delta), abs(back_delta)) < 1e-4:
            notes.append("不挂砝码的角度不在挂前、挂后两者之间：零点读数或前后记反了，单边刚度不报。")
            disagreement = float("inf")
        else:
            front_k = moment / abs(front_delta)
            back_k = moment / abs(back_delta)
            disagreement = abs(front_k - back_k) / (0.5 * (front_k + back_k))
            if disagreement > SIDE_DISAGREEMENT_MAX:
                notes.append(f"前后单边刚度相差 {100.0 * disagreement:.0f}%（前 {front_k:.3f}、后 "
                             f"{back_k:.3f} N·m/rad），超过 {100.0 * SIDE_DISAGREEMENT_MAX:.0f}%："
                             "零点读数、砝码位置或台架非线性（线缆一边松一边紧）有问题，重做一遍。")
    swing = max(abs(front), abs(back))
    if swing > 10.0:
        notes.append(f"挂砝码后绕杆转角到了 {swing:.1f}°，小角度近似开始有误差：换轻一点的砝码。")
    return StiffnessEstimate(stiffness_n_m_rad=float(stiffness), front_n_m_rad=float(front_k),
                             back_n_m_rad=float(back_k), side_disagreement=float(disagreement),
                             notes=tuple(notes), reading=reading, projection=float(projection))


def implied_pivot_m(stiffness_n_m_rad: float, mass_kg: float,
                    gravity_m_s2: float = GRAVITY_M_S2) -> float:
    """等效杆高 d_eff = K/(m·g)：把全部回中刚度都当重力时杆该有多高。"""
    return float(stiffness_n_m_rad) / (float(mass_kg) * gravity_m_s2)


def extra_stiffness(stiffness_n_m_rad: float, mass_kg: float, pivot_m: float,
                    gravity_m_s2: float = GRAVITY_M_S2) -> float:
    """重力之外多出来的刚度 K − m·g·d（线缆、夹持等）。"""
    return float(stiffness_n_m_rad) - float(mass_kg) * gravity_m_s2 * float(pivot_m)


__all__ = ["ASSUMPTIONS", "GRAVITY_M_S2", "SIDE_DISAGREEMENT_MAX", "StiffnessEstimate",
           "extra_stiffness", "hanging_sides", "implied_pivot_m", "reading_axis",
           "weight_test_stiffness"]
