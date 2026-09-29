"""台架几何：杆到质心距离 d 由"你量的"和"机体参数给的"两段相加。

    d = 杆到飞控板的垂直距离（尺量，杆在飞控上方为正）
      + 飞控到质心的距离（机体参数 airframe.imu_z_m − airframe.cg_z_m）

d 是拟合的**输入**，不是拟合结果：杆上数据里 d 与固件力矩模型的比例系数分不开，
所以必须量出来。飞控到质心那一段不让人填，免得和机体模型页的数对不上。
"""

from __future__ import annotations

import math

#: 杆/倾转轴到飞控板的距离上限。超过 1 m 几乎一定是把厘米当成了米。
LENGTH_LIMIT_M = 1.0


def fc_above_cg_m(params: dict, ready: dict) -> tuple[float, str] | None:
    """`(飞控在质心上方的距离 m, 来源说明)`；读不到返回 None。"""
    try:
        value = float(params["airframe.imu_z_m"]) - float(params["airframe.cg_z_m"])
        if math.isfinite(value):
            return value, "机体参数 imu_z − cg_z"
    except (KeyError, TypeError, ValueError):
        pass
    try:
        value = float(ready["imu_off_um"]) * 1e-6
        if math.isfinite(value):
            return value, "飞控状态报告"
    except (KeyError, TypeError, ValueError):
        pass
    return None


def parse_length(text: str, what: str, limit: float = LENGTH_LIMIT_M) -> float | None:
    """空白返回 None；否则必须是 |x| ≤ limit 的有限数字（单位 m）。"""
    text = (text or "").strip()
    if not text:
        return None
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f"「{what}」要填数字（单位 m）") from None
    if not math.isfinite(value):
        raise ValueError(f"「{what}」要填有限数字（单位 m）")
    if abs(value) > limit:
        raise ValueError(f"「{what}」填了 {value:g} m，超过 {limit:g} m：单位是米，15 cm 填 0.15。")
    return value


def needed_pivots(psi_deg: float) -> tuple[bool, bool]:
    """`(要横滚倾转轴, 要俯仰倾转轴)`：绕 Y（90°）只要俯仰，绕 X（0°）只要横滚，斜向两个都要。"""
    rest = psi_deg % 180.0
    if math.isclose(rest, 90.0, abs_tol=1e-6):
        return False, True
    if math.isclose(rest, 0.0, abs_tol=1e-6) or math.isclose(rest, 180.0, abs_tol=1e-6):
        return True, False
    return True, True


#: 横滚倾转轴 = 舵机 1 转轴，俯仰倾转轴 = 舵机 2 转轴（与固件几何力矩模型同一组参数）。
SERVO_AXIS_PARAM = {"roll": "airframe.servo1_axis_z_m", "pitch": "airframe.servo2_axis_z_m"}
#: 手填值与机体参数里的舵机转轴高度相差超过它就提醒。
PIVOT_MISMATCH_M = 0.005

SOURCE_SERVO = "servo"
SOURCE_PROP_PLANE = "prop_plane"


def airframe_servo_pivot(params: dict, axis: str) -> float | None:
    """机体参数里的舵机转轴高度，换算成"倾转轴到飞控板"（= servoN_axis_z_m − imu_z_m）。

    0 或缺失都当作"没填"。2026-09-27 起固件的倾转力矩就用这组参数算力臂，所以它是首选。
    """
    try:
        servo = float(params[SERVO_AXIS_PARAM[axis]])
        if not math.isfinite(servo) or servo == 0.0:
            return None
        value = servo - float(params["airframe.imu_z_m"])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _prop_plane_pivot(params: dict, axis: str) -> float | None:
    """旧路子：thrust_point_z_m + {axis}_axis_to_prop_plane_m − imu_z_m；该项为 0 表示没填。"""
    try:
        plane = float(params[f"airframe.{axis}_axis_to_prop_plane_m"])
        if not math.isfinite(plane) or plane == 0.0:
            return None
        value = (float(params["airframe.thrust_point_z_m"]) + plane
                 - float(params["airframe.imu_z_m"]))
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def pivot_prefill(params: dict, axis: str) -> tuple[float, str] | None:
    """`(倾转轴到飞控板的预填值, 来源)`：先用舵机转轴高度，没有再退回桨盘换算。"""
    value = airframe_servo_pivot(params, axis)
    if value is not None:
        return value, SOURCE_SERVO
    value = _prop_plane_pivot(params, axis)
    if value is not None:
        return value, SOURCE_PROP_PLANE
    return None


def metres(value: float) -> str:
    """按微米取整后的命令文本：与回显的 µm 整数一一对应，不丢精度。"""
    rounded = round(value, 6)
    if rounded == 0:
        return "0"
    return f"{rounded:.6f}".rstrip("0").rstrip(".")


__all__ = ["LENGTH_LIMIT_M", "fc_above_cg_m", "metres", "needed_pivots", "parse_length",
           "pivot_prefill", "airframe_servo_pivot", "PIVOT_MISMATCH_M", "SERVO_AXIS_PARAM"]
