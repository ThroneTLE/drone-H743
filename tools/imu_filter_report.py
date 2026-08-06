#!/usr/bin/env python3
"""Visualise the vibration spectrum and the 1st- vs 2nd-order filter tradeoff.

Reads the throttle-sweep captures produced by imu_vibration_capture.py and
produces a multi-panel figure plus a printed table:

  1. Raw gyro spectra for every throttle step, with the blade tone marked.
  2. Blade tone vs rotor command, which is how a real tone (rises with rpm) is
     distinguished from an alias (falls with rpm).
  3. Filter magnitude response, old vs new, with the tone band shaded.
  4. Before/after spectrum at hover for the chosen design.
  5. Group delay, so the cost of the extra attenuation is visible.

Mirrors the firmware filter in App/Src/app_sensor.c (RBJ biquad, Q=1/sqrt(2)).

Usage:
    python tools/imu_filter_report.py                    # all captures found
    python tools/imu_filter_report.py --no-show          # write PNG only
"""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "tools" / "data" / "imu_vibration"

ACCEL_LSB_PER_G = 2048.0     # +-16 g
GYRO_LSB_PER_DPS = 32.8      # +-1000 dps

# Firmware configuration, old and new. Keep in sync with Sensor_Task.
OLD = {"order": 1, "gyro_hz": 80.0, "accel_hz": 30.0}
NEW = {"order": 2, "gyro_hz": 80.0, "accel_hz": 40.0}


def biquad_coeffs(fc: float, rate: float):
    """RBJ cookbook lowpass, Q = 1/sqrt(2). Same math as APP_Sensor_LpfInit."""
    fc = min(fc, 0.45 * rate)
    w0 = 2.0 * np.pi * fc / rate
    cos_w0, sin_w0 = np.cos(w0), np.sin(w0)
    alpha = sin_w0 * 0.70710678
    a0 = 1.0 + alpha
    b0 = ((1.0 - cos_w0) * 0.5) / a0
    return (b0, (1.0 - cos_w0) / a0, b0, (-2.0 * cos_w0) / a0,
            (1.0 - alpha) / a0)


def apply_biquad(x: np.ndarray, fc: float, rate: float) -> np.ndarray:
    b0, b1, b2, a1, a2 = biquad_coeffs(fc, rate)
    y = np.empty_like(x)
    x1 = x2 = y1 = y2 = float(x[0])
    for i, v in enumerate(x):
        out = b0 * v + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, v
        y2, y1 = y1, out
        y[i] = out
    return y


def apply_iir1(x: np.ndarray, fc: float, rate: float) -> np.ndarray:
    dt = 1.0 / rate
    alpha = dt / (1.0 / (2.0 * np.pi * fc) + dt)
    y = np.empty_like(x)
    state = float(x[0])
    for i, v in enumerate(x):
        state += alpha * (v - state)
        y[i] = state
    return y


def response_db(fc: float, order: int, freqs: np.ndarray, rate: float) -> np.ndarray:
    z = np.exp(-1j * 2.0 * np.pi * freqs / rate)
    if order == 1:
        dt = 1.0 / rate
        a = dt / (1.0 / (2.0 * np.pi * fc) + dt)
        h = a / (1.0 - (1.0 - a) * z)
    else:
        b0, b1, b2, a1, a2 = biquad_coeffs(fc, rate)
        h = (b0 + b1 * z + b2 * z**2) / (1.0 + a1 * z + a2 * z**2)
    return 20.0 * np.log10(np.maximum(np.abs(h), 1e-12))


def group_delay_ms(fc: float, order: int, freq: float, rate: float) -> float:
    eps = 0.3
    def phase(f):
        z = np.exp(-1j * 2.0 * np.pi * f / rate)
        if order == 1:
            dt = 1.0 / rate
            a = dt / (1.0 / (2.0 * np.pi * fc) + dt)
            return np.angle(a / (1.0 - (1.0 - a) * z))
        b0, b1, b2, a1, a2 = biquad_coeffs(fc, rate)
        return np.angle((b0 + b1 * z + b2 * z**2) / (1.0 + a1 * z + a2 * z**2))
    return float(-(phase(freq + eps) - phase(freq - eps)) / (2 * np.pi * 2 * eps) * 1000.0)


def spectrum(sig: np.ndarray, rate: float):
    x = np.asarray(sig, dtype=float)
    x = x - x.mean()
    w = np.hanning(len(x))
    amp = np.abs(np.fft.rfft(x * w)) * 2.0 / w.sum()
    return np.fft.rfftfreq(len(x), 1.0 / rate), amp


def load_captures(paths: list[Path]) -> list[dict]:
    out = []
    for path in paths:
        df = pd.read_csv(path)
        tag = path.name.split("imu_vib_")[1].rsplit("_2026", 1)[0]
        ts = df.timestamp_us.values.astype(np.int64)
        rate = 1e6 / float(np.median(np.diff(ts)))
        gyro = {a: df[f"raw_gyro_{a}"].values / GYRO_LSB_PER_DPS for a in "xyz"}
        accel = {a: df[f"raw_accel_{a}"].values / ACCEL_LSB_PER_G for a in "xyz"}

        # Dominant tone above 20 Hz, taken from the strongest gyro axis.
        best = (0.0, 0.0, "x")
        for axis, sig in gyro.items():
            freq, amp = spectrum(sig, rate)
            mask = freq > 20.0
            if not mask.any():
                continue
            k = np.argmax(np.where(mask, amp, 0.0))
            if amp[k] > best[1]:
                best = (float(freq[k]), float(amp[k]), axis)

        out.append({
            "tag": tag, "path": path, "rate": rate, "gyro": gyro, "accel": accel,
            "tone_hz": best[0], "tone_axis": best[2],
            "motor_us": float(df.motor_upper_us.median()),
            "motor_max_us": float(df.motor_upper_us.max()),
            "gyro_rms": float(np.sqrt(sum(v.std() ** 2 for v in gyro.values()))),
            "accel_rms": float(np.sqrt(sum(v.std() ** 2 for v in accel.values()))),
        })
    return out


def evaluate(captures: list[dict]) -> list[dict]:
    """Apply both filter designs to the real data and measure the outcome."""
    rows = []
    for cap in captures:
        rate = cap["rate"]

        def rms(sigs, fn):
            return float(np.sqrt(sum(fn(v).std() ** 2 for v in sigs.values())))

        old_g = rms(cap["gyro"], lambda v: apply_iir1(v, OLD["gyro_hz"], rate))
        new_g = rms(cap["gyro"], lambda v: apply_biquad(v, NEW["gyro_hz"], rate))
        old_a = rms(cap["accel"], lambda v: apply_iir1(v, OLD["accel_hz"], rate))
        new_a = rms(cap["accel"], lambda v: apply_biquad(v, NEW["accel_hz"], rate))

        # Vibration-band energy only (above 80 Hz): the part the filter should
        # remove. Total RMS is dominated by real motion and hides the effect.
        def band_rms(sigs, fn, lo=80.0):
            total = 0.0
            for v in sigs.values():
                freq, amp = spectrum(fn(v), rate)
                total += np.sum(amp[freq >= lo] ** 2) / 2.0
            return float(np.sqrt(total))

        rows.append({
            "tag": cap["tag"],
            "motor_us": cap["motor_us"],
            "tone_hz": cap["tone_hz"],
            "raw_gyro": cap["gyro_rms"],
            "old_gyro": old_g,
            "new_gyro": new_g,
            "raw_accel": cap["accel_rms"],
            "old_accel": old_a,
            "new_accel": new_a,
            "vib_raw": band_rms(cap["gyro"], lambda v: v),
            "vib_old": band_rms(cap["gyro"], lambda v: apply_iir1(v, OLD["gyro_hz"], rate)),
            "vib_new": band_rms(cap["gyro"], lambda v: apply_biquad(v, NEW["gyro_hz"], rate)),
        })
    return rows


def print_table(rows: list[dict], rate: float) -> None:
    print("\n=== 桨叶频率与滤波效果（基于实测数据）===")
    print("%-10s %8s %9s | %-22s | %-22s" % (
        "档位", "电机us", "主频Hz", "陀螺振动带 RMS (dps)", "加速度总 RMS (g)"))
    print("%-10s %8s %9s | %7s %7s %7s | %7s %7s" % (
        "", "", "", "原始", "一阶", "二阶", "一阶", "二阶"))
    for r in rows:
        print("%-10s %8.0f %9.1f | %7.3f %7.3f %7.3f | %7.4f %7.4f" % (
            r["tag"], r["motor_us"], r["tone_hz"],
            r["vib_raw"], r["vib_old"], r["vib_new"],
            r["old_accel"], r["new_accel"]))

    spin = [r for r in rows if r["motor_us"] > 1200]
    if spin:
        old_db = 20 * np.log10(np.mean([r["vib_old"] for r in spin])
                               / np.mean([r["vib_raw"] for r in spin]))
        new_db = 20 * np.log10(np.mean([r["vib_new"] for r in spin])
                               / np.mean([r["vib_raw"] for r in spin]))
        print(f"\n  振动带平均衰减: 一阶 {old_db:+.1f} dB -> 二阶 {new_db:+.1f} dB"
              f"  (改善 {old_db - new_db:.1f} dB)")

    print("\n=== 截止频率与延迟代价 ===")
    print("%-24s %9s %9s %9s %10s" % ("配置", "@107Hz", "@141Hz", "@180Hz", "延迟@10Hz"))
    for label, cfg in (("原 一阶 80Hz", OLD), ("新 二阶 80Hz", NEW)):
        att = [response_db(cfg["gyro_hz"], cfg["order"], np.array([f]), rate)[0]
               for f in (107.2, 140.6, 179.9)]
        d = group_delay_ms(cfg["gyro_hz"], cfg["order"], 10.0, rate)
        print("%-24s %9.1f %9.1f %9.1f %9.2fms" % (label, *att, d))


def make_figure(captures: list[dict], rows: list[dict], out_png: Path,
                show: bool = True) -> Path:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rate = captures[0]["rate"]
    nyq = rate / 2.0
    fig = plt.figure(figsize=(15, 11))
    gs = fig.add_gridspec(3, 2, hspace=0.34, wspace=0.22)

    spin = sorted([c for c in captures if c["motor_us"] > 1200],
                  key=lambda c: c["motor_us"])
    hover = next((c for c in captures if c["tag"].startswith("hover")), None)

    # 1. Raw spectra per throttle step -------------------------------------
    ax = fig.add_subplot(gs[0, :])
    colors = plt.cm.viridis(np.linspace(0.15, 0.9, max(len(spin), 1)))
    for cap, color in zip(spin, colors):
        axis = cap["tone_axis"]
        freq, amp = spectrum(cap["gyro"][axis], cap["rate"])
        ax.semilogy(freq, np.maximum(amp, 1e-5), lw=0.9, color=color,
                    label=f"{cap['tag']}  ({cap['motor_us']:.0f}us)  {cap['tone_hz']:.0f}Hz")
        ax.axvline(cap["tone_hz"], color=color, ls=":", lw=1.2, alpha=0.8)
    if hover is not None:
        freq, amp = spectrum(hover["gyro"][hover["tone_axis"]], hover["rate"])
        ax.semilogy(freq, np.maximum(amp, 1e-5), lw=1.1, color="crimson",
                    label=f"hover  {hover['tone_hz']:.0f}Hz")
        ax.axvline(hover["tone_hz"], color="crimson", ls=":", lw=1.4)
    ax.axvline(nyq, color="k", ls="--", lw=1.2)
    ax.text(nyq - 6, ax.get_ylim()[1] * 0.3, f"Nyquist {nyq:.0f}Hz",
            rotation=90, va="top", ha="right", fontsize=8)
    ax.set_xlim(0, rate / 2)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Gyro amplitude (dps)")
    ax.set_title("1. Raw gyro spectrum per throttle step — blade tone rises with rotor speed")
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.3, which="both")

    # 2. Tone vs rotor command --------------------------------------------
    ax = fig.add_subplot(gs[1, 0])
    if len(spin) >= 2:
        us = np.array([c["motor_us"] for c in spin])
        tone = np.array([c["tone_hz"] for c in spin])
        ax.plot(us, tone, "o-", color="#1f77b4", ms=7, label="measured tone")
        slope, icpt = np.polyfit(us, tone, 1)
        xs = np.linspace(us.min() - 40, us.max() + 40, 50)
        r = np.corrcoef(us, tone)[0, 1]
        ax.plot(xs, slope * xs + icpt, "--", color="gray",
                label=f"linear fit  r={r:.4f}")
        for c in spin:
            ax.annotate(c["tag"], (c["motor_us"], c["tone_hz"]),
                        textcoords="offset points", xytext=(6, -10), fontsize=8)
    ax.set_xlabel("Motor command (us)")
    ax.set_ylabel("Blade tone (Hz)")
    ax.set_title("2. Tone vs rotor speed\n(rising = real vibration, falling = aliasing)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # 3. Filter response, old vs new --------------------------------------
    ax = fig.add_subplot(gs[1, 1])
    freqs = np.logspace(np.log10(1.0), np.log10(nyq), 400)
    ax.semilogx(freqs, response_db(OLD["gyro_hz"], 1, freqs, rate),
                lw=2, color="#d62728", label=f"old: 1st order {OLD['gyro_hz']:.0f}Hz")
    ax.semilogx(freqs, response_db(NEW["gyro_hz"], 2, freqs, rate),
                lw=2, color="#2ca02c", label=f"new: 2nd order {NEW['gyro_hz']:.0f}Hz")
    if spin or hover:
        tones = [c["tone_hz"] for c in spin + ([hover] if hover else [])]
        ax.axvspan(min(tones), max(tones), color="orange", alpha=0.18,
                   label=f"blade band {min(tones):.0f}-{max(tones):.0f}Hz")
    ax.axhline(-3, color="gray", ls=":", lw=1)
    for f in (107.2, 179.9):
        for cfg, color in ((OLD, "#d62728"), (NEW, "#2ca02c")):
            db = response_db(cfg["gyro_hz"], cfg["order"], np.array([f]), rate)[0]
            ax.plot([f], [db], "o", color=color, ms=6)
            ax.annotate(f"{db:.1f}dB", (f, db), textcoords="offset points",
                        xytext=(6, -3), fontsize=8, color=color)
    ax.set_xlim(1, nyq)
    ax.set_ylim(-45, 8)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title("3. Gyro filter response — chosen cutoff")
    ax.legend(fontsize=8, loc="lower left")
    ax.grid(alpha=0.3, which="both")

    # 4. Before/after spectrum on real data -------------------------------
    ax = fig.add_subplot(gs[2, 0])
    ref = hover if hover is not None else (spin[-1] if spin else captures[0])
    sig = ref["gyro"][ref["tone_axis"]]
    for label, data, color in (
            ("raw", sig, "#999999"),
            (f"1st {OLD['gyro_hz']:.0f}Hz", apply_iir1(sig, OLD["gyro_hz"], ref["rate"]), "#d62728"),
            (f"2nd {NEW['gyro_hz']:.0f}Hz", apply_biquad(sig, NEW["gyro_hz"], ref["rate"]), "#2ca02c")):
        freq, amp = spectrum(data, ref["rate"])
        ax.semilogy(freq, np.maximum(amp, 1e-5), lw=1.0, color=color, label=label)
    ax.axvline(ref["tone_hz"], color="orange", ls=":", lw=1.5)
    ax.set_xlim(0, nyq)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Gyro amplitude (dps)")
    ax.set_title(f"4. Before/after on real data ({ref['tag']}, gyro_{ref['tone_axis']})")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")

    # 5. Group delay -------------------------------------------------------
    ax = fig.add_subplot(gs[2, 1])
    band = np.linspace(1, 60, 120)
    for label, cfg, color in (("old 1st 80Hz", OLD, "#d62728"),
                              ("new 2nd 80Hz", NEW, "#2ca02c")):
        delays = [group_delay_ms(cfg["gyro_hz"], cfg["order"], f, rate) for f in band]
        ax.plot(band, delays, lw=2, color=color, label=label)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Group delay (ms)")
    ax.set_title("5. Delay cost in the attitude band\n(the price paid for extra attenuation)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle("drone-H743 IMU vibration & filter design — measured on hardware",
                 fontsize=13, y=0.985)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=130, bbox_inches="tight")
    print(f"\n  图已保存: {out_png}")
    if show:
        plt.show()
    return out_png


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("captures", nargs="*", type=Path,
                        help="capture CSVs (default: all in tools/data/imu_vibration)")
    parser.add_argument("--out", type=Path,
                        default=DATA_DIR / "filter_design_report.png")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args(argv)

    paths = args.captures or [Path(p) for p in
                              sorted(glob.glob(str(DATA_DIR / "imu_vib_*.csv")))]
    paths = [p for p in paths if "_analysis" not in p.name]
    if not paths:
        print(f"no captures found in {DATA_DIR}", file=sys.stderr)
        return 1

    captures = load_captures(paths)
    rows = evaluate(captures)
    print_table(rows, captures[0]["rate"])
    make_figure(captures, rows, args.out, show=not args.no_show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
