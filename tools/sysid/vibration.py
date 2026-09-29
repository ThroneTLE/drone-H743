"""陀螺振动主频与电调转速的对照：eRPM → 振动比。

双向 DShot 回传的是**电转速** eRPM；机械转频 = eRPM/60/极对数。桨与电机的一阶不平衡
振动在机械转频上，所以"陀螺振动主频 ÷ (eRPM/60)"应当接近 1/极对数——对上了说明这条
振动线就是那个转子的一阶，也顺带验证了转速回传的标度。

主频用 Lomb–Scargle 在**原始时间戳**上算：记录跟着控制拍走，采样间隔在 2～6 ms 之间抖动，
先插值成等距会把接近奈奎斯特的振动削掉（`results.tracking_summary` 同理）。低频的姿态运动
先扣掉（等距重采样 → 15 Hz 零相位低通 → 插回原始时刻相减），否则它的泄漏会压过振动线。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import fit as _fit

#: 搜振动主频的频段 [Hz] 与频率步长。
BAND_HZ = (20.0, 300.0)
STEP_HZ = 0.1

#: 扣低频姿态运动用的截止 [Hz]（与跟踪误差的分界同一个数）。
MOTION_CUTOFF_HZ = 15.0

#: 比值离最近的 1/n 不超过这个相对误差，就报"对得上 n 对极"。
POLE_MATCH_TOLERANCE = 0.05

ROTORS = (("erpm", "上桨"), ("erpm_lower", "下桨"))


@dataclass(frozen=True)
class RotorRatio:
    label: str
    #: 本轮 eRPM 中位数（只算 > 0 的样本）与电频率 eRPM/60 [Hz]。
    erpm: float
    electrical_hz: float
    #: 振动主频 ÷ (eRPM/60)；期望 1/极对数。
    ratio: float
    #: 最接近的整数极对数（比值离 1/n 超出容差时为 None）。
    pole_pairs: int | None


@dataclass(frozen=True)
class VibrationReport:
    dominant_hz: float
    #: 主频处功率占频段总功率的比例（越大越"是一根线"）。
    peak_share: float
    #: 采样间隔中位数对应的名义奈奎斯特 [Hz]：主频高过它时靠时间戳抖动分辨，可能是混叠。
    nominal_nyquist_hz: float
    rotors: tuple[RotorRatio, ...]

    def lines(self) -> list[str]:
        text = [f"陀螺振动主频 {self.dominant_hz:.1f} Hz（Lomb–Scargle，原始时间戳，"
                f"{BAND_HZ[0]:g}–{BAND_HZ[1]:g} Hz）"]
        if self.dominant_hz > self.nominal_nyquist_hz:
            text.append(f"主频高于名义奈奎斯特 {self.nominal_nyquist_hz:.0f} Hz：靠采样时刻的抖动"
                        "分辨，可能是混叠，提高线上采样率再看")
        for rotor in self.rotors:
            match = (f"，对得上 {rotor.pole_pairs} 对极的一阶" if rotor.pole_pairs
                     else "，不像任何整数极对数的一阶")
            text.append(f"{rotor.label} {rotor.erpm:.0f} eRPM（电频率 {rotor.electrical_hz:.1f} Hz）："
                        f"eRPM→振动比 {rotor.ratio:.4f}（期望 1/极对数{match}）")
        return text


def _motion_removed(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    steps = np.diff(t)
    fs = 1.0 / float(np.median(steps[steps > 0.0]))
    grid, uniform = _fit.uniform_signal(t, x, fs)
    low = _fit.lowpass_zero_phase(uniform, fs, min(MOTION_CUTOFF_HZ, 0.4 * fs), 2)
    start = math.ceil(float(t[0]) * fs) / fs          # uniform_signal 的网格起点（绝对时间）
    return x - np.interp(t, start + grid, low)


def dominant_vibration_hz(times_s, channels, band_hz=BAND_HZ, step_hz: float = STEP_HZ
                          ) -> tuple[float, float, float]:
    """(主频 [Hz], 主频功率占比, 名义奈奎斯特 [Hz])：各通道 Lomb–Scargle 功率相加取最大。"""
    from scipy.signal import lombscargle

    t = np.asarray(times_s, dtype=float)
    if t.size < 64 or np.any(np.diff(t) <= 0.0):
        raise ValueError("时间戳太少或不递增，算不出振动主频")
    freqs = np.arange(band_hz[0], band_hz[1] + 0.5 * step_hz, step_hz)
    power = np.zeros(freqs.size)
    for channel in channels:
        x = _motion_removed(t, np.asarray(channel, dtype=float))
        x = x - float(np.mean(x))
        if float(np.std(x)) < 1e-12:
            continue
        power += lombscargle(t, x, 2.0 * math.pi * freqs)
    if not np.any(power > 0.0):
        raise ValueError("陀螺在振动频段里没有能量")
    index = int(np.argmax(power))
    nyquist = 0.5 / float(np.median(np.diff(t)))
    return float(freqs[index]), float(power[index] / np.sum(power)), nyquist


def _pole_pairs(ratio: float) -> int | None:
    if not (math.isfinite(ratio) and ratio > 0.0):
        return None
    n = max(1, int(round(1.0 / ratio)))
    return n if abs(ratio * n - 1.0) <= POLE_MATCH_TOLERANCE else None


def erpm_vibration_report(times_s, samples) -> VibrationReport | None:
    """有真实 eRPM（> 0 的样本）时给出振动主频与各转子的 eRPM→振动比；没有就 None。"""
    rotors_present = []
    for key, label in ROTORS:
        values = np.array([float(s.get(key, 0.0) or 0.0) for s in samples])
        spinning = values[values > 0.0]
        if spinning.size >= max(16, values.size // 10):
            rotors_present.append((label, float(np.median(spinning))))
    if not rotors_present:
        return None
    gyro = [[float(s.get(axis, 0.0)) for s in samples] for axis in ("gx", "gy", "gz")]
    dominant, share, nyquist = dominant_vibration_hz(times_s, gyro)
    rotors = []
    for label, erpm in rotors_present:
        electrical = erpm / 60.0
        ratio = dominant / electrical
        rotors.append(RotorRatio(label=label, erpm=erpm, electrical_hz=electrical,
                                 ratio=ratio, pole_pairs=_pole_pairs(ratio)))
    return VibrationReport(dominant_hz=dominant, peak_share=share,
                           nominal_nyquist_hz=nyquist, rotors=tuple(rotors))


__all__ = ["BAND_HZ", "RotorRatio", "VibrationReport", "dominant_vibration_hz",
           "erpm_vibration_report"]
