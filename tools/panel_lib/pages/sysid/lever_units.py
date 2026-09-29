"""候选增益的力矩单位换算：录制那套固件的倾转力臂 → 当前连着的飞控的倾转力臂。

为什么必须换：速率环 kp/ki/kd 的输出是**固件力矩**（N·m），固件再按自己的力臂
L = cg_z_m − servoN_axis_z_m 把它换成舵机倾角（τ_fw = L·T·sin δ）。真实力矩 = κ·τ_fw，
κ 与录制时的 L_src 成反比，所以辨识合成的增益只在 L_src 那套单位里成立。同一组数写进
力臂是 L_now 的固件，实际的软硬变成 L_src/L_now 倍——旧力矩模型（L ≈ 0.08 m）录的数据
写进今晚的几何模型（L = 0.0354 m）会硬 2.3 倍，重心改到 −0.01 后（L = 0.12 m）又软一半多。

换算：rate_{axis}_{kp,ki,kd,i_limit_n_m} × L_now/L_src（逐轴）；att_* 输出的是角速度，不换。
任一力臂读不出来、或两边力臂符号相反（力矩方向都不一样）就拒绝，不猜。
"""
from __future__ import annotations

import math
import re
from typing import Mapping

#: 输出是固件力矩（N·m）的速率环量：按力臂比换算。
_TORQUE_UNIT = re.compile(r"^coax\.rate_(roll|pitch)_(kp|ki|kd|i_limit_n_m)$")
_AXIS_INDEX = {"roll": 0, "pitch": 1}
_AXIS_TEXT = {"roll": "横滚", "pitch": "俯仰"}
_MODEL_TEXT = {"legacy": "旧力矩模型", "geometric": "几何力矩模型", "unknown": "力矩模型未知"}

#: 力臂比离 1 不到它就当作同一套单位（Flash/回显的 6 位小数量化）。
SAME_UNITS_TOLERANCE = 1e-4


def torque_record(parameter_echo: Mapping[str, str] | None) -> dict:
    """`{"signature": ..., "levers_m": [roll, pitch] 或 None}`：一份参数回显的力矩单位签名。"""
    from . import _core

    echo = parameter_echo or {}
    lever_helper = getattr(_core, "firmware_tilt_levers", None)
    signature_helper = getattr(_core, "torque_model_signature", None)
    try:
        levers = lever_helper(echo) if lever_helper is not None else None
        signature = str(signature_helper(echo)) if signature_helper is not None else "unknown"
    except Exception:      # 拿不准就当"未知"：应用时会因此拒绝，不会猜
        levers, signature = None, "unknown"
    return {"signature": signature,
            "levers_m": None if levers is None else [float(levers[0]), float(levers[1])]}


def _levers(record: Mapping | None) -> tuple[float, float] | None:
    try:
        levers = (record or {}).get("levers_m")
        roll, pitch = (float(v) for v in levers)
    except (TypeError, ValueError, AttributeError):
        return None
    if not all(math.isfinite(v) and abs(v) > 1e-6 for v in (roll, pitch)):
        return None
    return roll, pitch


def describe(record: Mapping | None) -> str:
    levers = _levers(record)
    model = _MODEL_TEXT.get(str((record or {}).get("signature")), "力矩模型未知")
    if levers is None:
        return f"{model}，力臂未知"
    return f"{model}，横滚力臂 {levers[0]:+.4f} m、俯仰力臂 {levers[1]:+.4f} m"


def axis_factors(source: Mapping | None, target: Mapping | None, axes,
                 *, source_what: str = "候选增益录制时") -> dict[str, float]:
    """逐轴 L_now/L_src；读不出或符号相反抛中文 ValueError（调用方原样显示并拒绝写入）。"""
    source_levers, target_levers = _levers(source), _levers(target)
    if source_levers is None:
        raise ValueError(f"{source_what}的固件倾转力臂读不出来（{describe(source)}），不知道增益是哪套"
                         "力矩单位，拒绝写入。")
    if target_levers is None:
        raise ValueError("读不到当前飞控的倾转力臂（机体参数 airframe.cg_z_m 与 servo1/2_axis_z_m "
                         "缺失或为 0），无法确认增益单位，拒绝写入：先到「机体模型」页确认参数已下发。")
    factors = {}
    for axis in axes:
        index = _AXIS_INDEX[axis]
        old, new = source_levers[index], target_levers[index]
        if old * new <= 0.0:
            raise ValueError(f"{_AXIS_TEXT[axis]}力臂在{source_what}（{old:+.4f} m）与当前飞控"
                             f"（{new:+.4f} m）符号相反：力矩方向都不一样，拒绝写入。先核对机体模型页的"
                             "舵机转轴与重心。")
        factors[axis] = new / old
    return factors


def needs_units(names) -> bool:
    """这些参数里有没有输出是固件力矩（要按力臂换算）的。"""
    return any(_TORQUE_UNIT.match(name) for name in names)


def convert(pairs, source: Mapping | None, target: Mapping | None, *,
            source_what: str = "候选增益录制时") -> tuple[list[tuple[str, str]], str]:
    """`[(参数名, 值)]` → 换算后的同样列表 + 一句给界面看的说明。拒绝时抛中文 ValueError。"""
    axes = sorted({m.group(1) for name, _v in pairs if (m := _TORQUE_UNIT.match(name))},
                  key=_AXIS_INDEX.get)
    if not axes:
        return list(pairs), ""
    factors = axis_factors(source, target, axes, source_what=source_what)
    converted = []
    for name, value in pairs:
        match = _TORQUE_UNIT.match(name)
        if match and abs(factors[match.group(1)] - 1.0) > SAME_UNITS_TOLERANCE:
            value = f"{float(value) * factors[match.group(1)]:.6g}"
        converted.append((name, value))
    source_levers, target_levers = _levers(source), _levers(target)
    parts = []
    for axis in axes:
        index = _AXIS_INDEX[axis]
        parts.append(f"{_AXIS_TEXT[axis]}力臂 {abs(source_levers[index]):.4f} → "
                     f"{abs(target_levers[index]):.4f} m，速率环 kp/ki/kd ×{factors[axis]:.3f}")
    if all(abs(factors[axis] - 1.0) <= SAME_UNITS_TOLERANCE for axis in axes):
        note = f"力矩单位核对：{source_what}与当前飞控力臂相同（{'；'.join(parts)}），不用换算。"
    else:
        note = (f"按力臂换算后写入：{'；'.join(parts)}（{source_what}：{describe(source)}；"
                f"当前飞控：{describe(target)}）。姿态环 P 输出的是角速度，不换。")
    return converted, note


__all__ = ["SAME_UNITS_TOLERANCE", "axis_factors", "convert", "describe", "needs_units",
           "torque_record"]
