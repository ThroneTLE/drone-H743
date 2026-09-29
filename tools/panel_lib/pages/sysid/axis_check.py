"""杆轴方向核对：跟着激励同步动的角速度应当几乎全在杆轴上。

光杆只放行绕杆转动；页面上选的杆轴方向（ψ）若与实际装法不符（2026-09-28：实际 −45° 却选了 +45°），
固件把力矩打在选的方向上，真正的杆把这部分顶住，机体在"被约束"的方向上跟着激励晃，沿杆反而动得少——
拟合只剩百分之几，但页面不会说为什么。这里用输入（舵机单独轮的 servo_tilt，其余轮的 torque）与
沿杆/垂直杆角速度的互谱，在 0.8–6 Hz 比较两者"与激励同步"的幅度：同一批实测里选对的轮次 ≤ 0.25
（多数 ≤ 0.05），选错的 0.45–2.05。只看同步部分，台架自己的晃动（摆、碳杆模态）不参与。
"""
from __future__ import annotations

import math

#: 垂直杆 / 沿杆 的同步幅度比超过它就判"杆轴方向可能选错"。
AXIS_WRONG_RATIO = 0.35
BAND_HZ = (0.8, 6.0)
FS_HZ = 250.0


def perpendicular_ratio(times_s, samples, azimuth_rad: float, mode) -> float | None:
    """垂直杆与沿杆两路角速度中与激励同步部分的幅度比；数据不够或没有激励时 None。"""
    try:
        import numpy as np
        from scipy.signal import csd
    except ImportError:
        return None
    times = np.asarray(times_s, dtype=float)
    if times.size < 256 or len(samples) != times.size or not (times[-1] > times[0]):
        return None
    key = "servo_tilt" if str(mode) == "3" else "torque"
    try:
        u = np.array([float(s[key]) for s in samples])
        gx = np.array([float(s["gx"]) for s in samples])
        gy = np.array([float(s["gy"]) for s in samples])
    except (KeyError, TypeError, ValueError):
        return None
    grid = np.arange(times[0], times[-1], 1.0 / FS_HZ)
    if grid.size < 256 or not np.all(np.isfinite(u)) or float(np.ptp(u)) <= 0.0:
        return None
    u, gx, gy = (np.interp(grid, times, v) for v in (u, gx, gy))
    along = math.cos(azimuth_rad) * gx + math.sin(azimuth_rad) * gy
    perp = -math.sin(azimuth_rad) * gx + math.cos(azimuth_rad) * gy
    segment = min(512, grid.size // 2)
    freq, s_along = csd(u, along, fs=FS_HZ, nperseg=segment)
    _, s_perp = csd(u, perp, fs=FS_HZ, nperseg=segment)
    band = (freq >= BAND_HZ[0]) & (freq <= BAND_HZ[1])
    denominator = float(np.abs(s_along[band]).sum())
    if not band.any() or not denominator > 0.0:
        return None
    return float(np.abs(s_perp[band]).sum()) / denominator


def axis_warning(ratio: float | None, azimuth_rad: float) -> str:
    """超阈值时的一句中文（放在分析结果最前面）；没超或算不出返回空串。"""
    if ratio is None or not ratio > AXIS_WRONG_RATIO:
        return ""
    chosen = math.degrees(azimuth_rad)
    return (f"⚠ 杆轴方向很可能选错：垂直杆方向跟着激励动的量是沿杆方向的 {ratio:.2f} 倍"
            f"（选对时通常 < 0.1）。页面选的是 {chosen:+.0f}°，请核对实际装法"
            "（例如实际是 −45° 却选了 +45°）；这一轮的拟合结果不能用。")


__all__ = ["AXIS_WRONG_RATIO", "axis_warning", "perpendicular_ratio"]
