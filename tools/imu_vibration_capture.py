#!/usr/bin/env python3
"""Capture full-rate raw IMU samples over USB CDC and analyse the vibration spectrum.

Why this exists: the flight log (~250 Hz) and VOFA stream (40 Hz) are both
decimated and carry post-LPF values, so neither can show the 150-300 Hz coaxial
rotor band. Sizing the ICM-42688 anti-alias filter and the software IIR requires
undecimated pre-filter samples, which is what firmware `IMUCAP` records.

Firmware side: App/Src/app_imu_capture.c, command handler in App/Src/app_control.c.

Typical use:

    # record ~6 s at the current rotor state, then pull it over USB
    python tools/imu_vibration_capture.py --port COM7 --capture --analyse

    # analyse an existing capture without touching hardware
    python tools/imu_vibration_capture.py --analyse-file tools/data/imu_vibration/xxx.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import struct
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    import serial
except Exception:  # pragma: no cover - only needed for live capture
    serial = None

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = ROOT / "tools" / "data" / "imu_vibration"

# Must not be four printable bytes: the old "IMUC" value collided with the
# "IMUCAP DUMP ok ..." status line and the reader parsed that text as a header.
CAPTURE_MAGIC = 0xA5C3494D
CAPTURE_VERSION = 3
FLAG_LAST = 0x00000001

# Must match APP_IMU_CaptureBlockHeader in App/Inc/app_imu_capture.h.
HEADER_FMT = "<IHHIIIHHHHHHII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Must match APP_IMU_CaptureSample.
SAMPLE_FMT = "<I3h3h3h3hhhhHHHHBBh"
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

FUSION_FLAG_NAMES = {
    0x01: "accel_ignored",
    0x02: "norm_rejected",
    0x04: "accel_recovery",
    0x08: "rate_recovery",
    0x10: "startup",
}

CSV_FIELDS = (
    "index",
    "timestamp_us",
    "dt_us",
    "raw_accel_x", "raw_accel_y", "raw_accel_z",
    "raw_gyro_x", "raw_gyro_y", "raw_gyro_z",
    "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "accel_filt_x_g", "accel_filt_y_g", "accel_filt_z_g",
    "gyro_filt_x_dps", "gyro_filt_y_dps", "gyro_filt_z_dps",
    "roll_deg", "pitch_deg", "yaw_deg",
    "motor_upper_us", "motor_lower_us",
    "servo_alpha_us", "servo_beta_us",
    "accel_error_deg",
    "fusion_flags", "flight_flags", "armed", "gyro_bias_ready",
)


def _accel_lsb_per_g(range_g: int) -> float:
    return {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}.get(range_g, 2048.0)


def _gyro_lsb_per_dps(range_dps: int) -> float:
    return {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}.get(range_dps, 32.8)


def crc32(data: bytes) -> int:
    """Bitwise CRC-32 matching imu_capture_crc32() in the firmware."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


def parse_header(raw: bytes) -> dict:
    fields = struct.unpack(HEADER_FMT, raw[:HEADER_SIZE])
    return {
        "magic": fields[0],
        "version": fields[1],
        "header_size": fields[2],
        "session_id": fields[3],
        "total_samples": fields[4],
        "offset_samples": fields[5],
        "block_samples": fields[6],
        "sample_size": fields[7],
        "accel_aaf_hz": fields[8],
        "gyro_aaf_hz": fields[9],
        "accel_range_g": fields[10],
        "gyro_range_dps": fields[11],
        "flags": fields[12],
        "payload_crc32": fields[13],
    }


def decode_samples(payload: bytes, count: int) -> list[tuple]:
    return [
        struct.unpack_from(SAMPLE_FMT, payload, i * SAMPLE_SIZE)
        for i in range(count)
    ]


class CaptureLink:
    """Text command channel plus binary block reader on one CDC port."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        if serial is None:
            raise RuntimeError("pyserial is required for live capture")
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.buf = bytearray()

    def close(self) -> None:
        self.ser.close()

    def send(self, line: str) -> None:
        self.ser.write((line + "\r\n").encode("ascii"))
        self.ser.flush()

    def read_text_line(self, deadline: float) -> str | None:
        """Read one CRLF-terminated status line, skipping binary noise."""
        while time.monotonic() < deadline:
            idx = self.buf.find(b"\n")
            if idx >= 0:
                line = bytes(self.buf[:idx])
                del self.buf[: idx + 1]
                try:
                    return line.decode("ascii", "replace").strip()
                except Exception:
                    return ""
            chunk = self.ser.read(4096)
            if chunk:
                self.buf.extend(chunk)
        return None

    def _fill(self, need: int, deadline: float) -> bool:
        while len(self.buf) < need:
            if time.monotonic() >= deadline:
                return False
            chunk = self.ser.read(max(need - len(self.buf), 1024))
            if chunk:
                self.buf.extend(chunk)
        return True

    def read_blocks(self, timeout_s: float = 30.0, verbose: bool = True,
                    progress=None) -> tuple[list[tuple], dict]:
        """Collect binary blocks until the LAST flag arrives.

        Resynchronises on the magic word so a stray status line interleaved with
        the stream cannot desynchronise the reader.
        """
        deadline = time.monotonic() + timeout_s
        magic = struct.pack("<I", CAPTURE_MAGIC)
        samples: dict[int, tuple] = {}
        meta: dict = {}
        bad_crc = 0
        mismatches: list[str] = []

        while time.monotonic() < deadline:
            idx = self.buf.find(magic)
            if idx < 0:
                # Keep a 3-byte tail in case the magic straddles a read.
                if len(self.buf) > 3:
                    del self.buf[: len(self.buf) - 3]
                if not self._fill(4, deadline):
                    break
                continue
            del self.buf[:idx]

            if not self._fill(HEADER_SIZE, deadline):
                break
            header = parse_header(bytes(self.buf[:HEADER_SIZE]))

            # A magic match can still be a false positive (stray bytes, or a
            # value that happens to appear inside a payload). Treat an
            # implausible header as noise and resync past it rather than
            # aborting the whole dump.
            plausible = (
                header["version"] == CAPTURE_VERSION
                and header["sample_size"] == SAMPLE_SIZE
                and header["header_size"] == HEADER_SIZE
                and 0 < header["block_samples"] <= 1024
                and header["total_samples"] <= 1 << 20
                and header["offset_samples"] <= header["total_samples"]
            )
            if not plausible:
                mismatches.append(
                    f"version={header['version']} sample_size={header['sample_size']}")
                del self.buf[:4]  # step past this magic and keep looking
                continue

            payload_bytes = header["block_samples"] * SAMPLE_SIZE
            if not self._fill(HEADER_SIZE + payload_bytes, deadline):
                break
            payload = bytes(self.buf[HEADER_SIZE : HEADER_SIZE + payload_bytes])
            del self.buf[: HEADER_SIZE + payload_bytes]

            if crc32(payload) != header["payload_crc32"]:
                bad_crc += 1
                continue

            meta = header
            for i, sample in enumerate(decode_samples(payload, header["block_samples"])):
                samples[header["offset_samples"] + i] = sample

            got = len(samples)
            total = header["total_samples"] or 1
            if verbose:
                print(f"\r  received {got}/{total} samples", end="", flush=True)
            if progress is not None:
                progress(f"receiving {got}/{total} samples", got / total)

            if header["flags"] & FLAG_LAST:
                break

        if verbose:
            print()
        if bad_crc:
            print(f"  warning: {bad_crc} block(s) failed CRC and were skipped")

        if not samples and mismatches:
            # Every candidate header was rejected: almost always a firmware
            # build that predates the host's layout.
            raise RuntimeError(
                "no valid blocks received; host expects version="
                f"{CAPTURE_VERSION} sample_size={SAMPLE_SIZE} but saw "
                f"{mismatches[0]}. Reflash the firmware so both sides match.")

        total = meta.get("total_samples", 0)
        missing = [i for i in range(total) if i not in samples]
        if missing:
            print(f"  warning: {len(missing)} sample(s) missing from the stream")
        ordered = [samples[i] for i in sorted(samples)]
        return ordered, meta


def samples_to_rows(samples: list[tuple], meta: dict) -> list[dict]:
    a_lsb = _accel_lsb_per_g(meta.get("accel_range_g", 16))
    g_lsb = _gyro_lsb_per_dps(meta.get("gyro_range_dps", 1000))
    rows = []
    prev_ts = None

    for index, s in enumerate(samples):
        (ts, ax, ay, az, gx, gy, gz,
         afx, afy, afz, gfx, gfy, gfz,
         roll, pitch, yaw,
         mu, ml, sa, sb, fflags, flflags, aerr) = s

        # uint32 microsecond stamps wrap about every 71 minutes.
        dt = None if prev_ts is None else (ts - prev_ts) & 0xFFFFFFFF
        prev_ts = ts

        rows.append({
            "index": index,
            "timestamp_us": ts,
            "dt_us": dt if dt is not None else "",
            "raw_accel_x": ax, "raw_accel_y": ay, "raw_accel_z": az,
            "raw_gyro_x": gx, "raw_gyro_y": gy, "raw_gyro_z": gz,
            "accel_x_g": ax / a_lsb, "accel_y_g": ay / a_lsb, "accel_z_g": az / a_lsb,
            "gyro_x_dps": gx / g_lsb, "gyro_y_dps": gy / g_lsb, "gyro_z_dps": gz / g_lsb,
            "accel_filt_x_g": afx / 1000.0,
            "accel_filt_y_g": afy / 1000.0,
            "accel_filt_z_g": afz / 1000.0,
            "gyro_filt_x_dps": gfx / 100.0,
            "gyro_filt_y_dps": gfy / 100.0,
            "gyro_filt_z_dps": gfz / 100.0,
            "roll_deg": roll / 100.0,
            "pitch_deg": pitch / 100.0,
            "yaw_deg": yaw / 100.0,
            "motor_upper_us": mu, "motor_lower_us": ml,
            "servo_alpha_us": sa, "servo_beta_us": sb,
            "accel_error_deg": aerr / 100.0,
            "fusion_flags": fflags,
            "flight_flags": flflags,
            "armed": 1 if (flflags & 0x01) else 0,
            "gyro_bias_ready": 1 if (flflags & 0x02) else 0,
        })
    return rows


def write_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(CSV_FIELDS))
        writer.writeheader()
        writer.writerows(rows)


def _spectrum(signal: np.ndarray, rate: float) -> tuple[np.ndarray, np.ndarray]:
    """Single-sided amplitude spectrum with a Hann window and mean removed."""
    x = np.asarray(signal, dtype=float)
    x = x - x.mean()
    window = np.hanning(len(x))
    # Coherent gain correction so tone amplitudes read true.
    spec = np.abs(np.fft.rfft(x * window)) * 2.0 / np.sum(window)
    freq = np.fft.rfftfreq(len(x), 1.0 / rate)
    return freq, spec


def _top_peaks(freq: np.ndarray, spec: np.ndarray, count: int = 5,
               min_hz: float = 5.0) -> list[tuple[float, float]]:
    mask = freq >= min_hz
    f, s = freq[mask], spec[mask]
    if len(f) < 3:
        return []
    # Local maxima only, so one broad peak is not reported as several.
    peak_idx = [
        i for i in range(1, len(s) - 1)
        if s[i] > s[i - 1] and s[i] >= s[i + 1]
    ]
    peak_idx.sort(key=lambda i: s[i], reverse=True)
    return [(float(f[i]), float(s[i])) for i in peak_idx[:count]]


def analyse(rows: list[dict], meta: dict, label: str = "") -> dict:
    if len(rows) < 64:
        raise RuntimeError(f"need at least 64 samples to analyse, got {len(rows)}")

    dt = np.array([r["dt_us"] for r in rows[1:] if r["dt_us"] != ""], dtype=float)
    rate = 1e6 / float(np.median(dt)) if len(dt) else 1000.0

    report: dict = {
        "label": label,
        "samples": len(rows),
        "effective_rate_hz": rate,
        "nyquist_hz": rate / 2.0,
        "duration_s": len(rows) / rate,
        "freq_resolution_hz": rate / len(rows),
        "timing": {
            "dt_us_median": float(np.median(dt)) if len(dt) else None,
            "dt_us_p99": float(np.percentile(dt, 99)) if len(dt) else None,
            "dt_us_max": float(dt.max()) if len(dt) else None,
        },
        "firmware": {
            "accel_range_g": meta.get("accel_range_g"),
            "gyro_range_dps": meta.get("gyro_range_dps"),
            "accel_aaf_hz": meta.get("accel_aaf_hz"),
            "gyro_aaf_hz": meta.get("gyro_aaf_hz"),
        },
    }

    # Clipping check: raw values at the int16 rail mean the range is too small.
    raw_cols = {
        "accel": ("raw_accel_x", "raw_accel_y", "raw_accel_z"),
        "gyro": ("raw_gyro_x", "raw_gyro_y", "raw_gyro_z"),
    }
    clip = {}
    for name, cols in raw_cols.items():
        vals = np.array([[r[c] for c in cols] for r in rows], dtype=float)
        near_rail = np.abs(vals) >= 32000
        clip[name] = {
            "abs_max_lsb": float(np.abs(vals).max()),
            "p99_9_lsb": float(np.percentile(np.abs(vals), 99.9)),
            "clipped_pct": float(100.0 * near_rail.mean()),
        }
    report["clipping"] = clip

    rotor = np.array([r["motor_upper_us"] for r in rows], dtype=float)
    report["rotor"] = {
        "motor_upper_us_median": float(np.median(rotor)),
        "motor_upper_us_min": float(rotor.min()),
        "motor_upper_us_max": float(rotor.max()),
        "spinning": bool(np.median(rotor) > 1200),
    }

    # Spectra of the raw (pre-LPF) signals: this is what sizes the filters.
    spectra = {}
    for axis in ("x", "y", "z"):
        for kind, unit in (("accel", "g"), ("gyro", "dps")):
            raw_key = f"raw_{kind}_{axis}"
            scale = (_accel_lsb_per_g(meta.get("accel_range_g", 16))
                     if kind == "accel"
                     else _gyro_lsb_per_dps(meta.get("gyro_range_dps", 1000)))
            sig = np.array([r[raw_key] for r in rows], dtype=float) / scale
            freq, spec = _spectrum(sig, rate)
            spectra[f"{kind}_{axis}"] = {
                "unit": unit,
                "rms": float(np.std(sig)),
                "peaks_hz": [
                    {"freq_hz": round(f, 2), "amplitude": round(a, 5)}
                    for f, a in _top_peaks(freq, spec)
                ],
            }
    report["raw_spectra"] = spectra

    # Measured LPF attenuation: compare recorded pre- and post-filter RMS.
    lpf = {}
    for kind, pre_scale in (("accel", _accel_lsb_per_g(meta.get("accel_range_g", 16))),
                            ("gyro", _gyro_lsb_per_dps(meta.get("gyro_range_dps", 1000)))):
        for axis in ("x", "y", "z"):
            pre = np.array([r[f"raw_{kind}_{axis}"] for r in rows], dtype=float) / pre_scale
            post_key = f"{kind}_filt_{axis}_" + ("g" if kind == "accel" else "dps")
            if post_key not in rows[0]:
                continue
            post = np.array([r[post_key] for r in rows], dtype=float)
            if np.allclose(post, 0.0):
                continue
            pre_rms, post_rms = float(np.std(pre)), float(np.std(post))
            lpf[f"{kind}_{axis}"] = {
                "pre_rms": round(pre_rms, 5),
                "post_rms": round(post_rms, 5),
                "attenuation_db": (round(20 * np.log10(post_rms / pre_rms), 2)
                                   if pre_rms > 0 and post_rms > 0 else None),
            }
    report["lpf_measured"] = lpf

    # Fusion health during the window: gate lockout must not be recurring.
    flags = np.array([r["fusion_flags"] for r in rows], dtype=int)
    report["fusion_health"] = {
        name: round(float(100.0 * ((flags & bit) != 0).mean()), 2)
        for bit, name in FUSION_FLAG_NAMES.items()
    }
    report["fusion_health"]["accel_error_deg_median"] = round(
        float(np.median([r["accel_error_deg"] for r in rows])), 3)

    report["recommendation"] = _recommend(report)
    return report


def _recommend(report: dict) -> dict:
    """Turn the spectrum into concrete filter guidance."""
    nyq = report["nyquist_hz"]
    notes: list[str] = []

    # Dominant tone across gyro axes, which is what the rate loop sees.
    tones = [
        p["freq_hz"]
        for key, block in report["raw_spectra"].items()
        if key.startswith("gyro")
        for p in block["peaks_hz"][:1]
    ]
    dominant = float(np.median(tones)) if tones else None

    for kind in ("accel", "gyro"):
        if report["clipping"][kind]["clipped_pct"] > 0.1:
            notes.append(
                f"{kind} is clipping ({report['clipping'][kind]['clipped_pct']:.1f}% "
                "of samples at the int16 rail) - increase the range before "
                "trusting any spectrum from this capture")

    if dominant is not None and dominant > 0.8 * nyq:
        notes.append(
            f"dominant tone {dominant:.0f} Hz is above 80% of Nyquist "
            f"({nyq:.0f} Hz); it may be an alias, so re-capture at a higher ODR "
            "before setting filters")

    aaf = report["firmware"].get("gyro_aaf_hz")
    if dominant is not None and aaf:
        if aaf < dominant:
            notes.append(
                f"AAF ({aaf} Hz) sits below the dominant tone ({dominant:.0f} Hz), "
                "so real rotor content is being attenuated as if it were noise")

    return {
        "dominant_gyro_tone_hz": round(dominant, 2) if dominant is not None else None,
        # Keep the fundamental observable for the rate loop, attenuate above it.
        "suggested_gyro_lpf_hz": (round(max(dominant * 0.5, 40.0))
                                  if dominant is not None else None),
        "suggested_accel_lpf_hz": (round(max(dominant * 0.25, 20.0))
                                   if dominant is not None else None),
        "notes": notes,
    }


def print_report(report: dict) -> None:
    print()
    print(f"=== {report.get('label') or 'capture'}")
    print(f"  samples {report['samples']}  rate {report['effective_rate_hz']:.1f} Hz  "
          f"duration {report['duration_s']:.2f} s  resolution {report['freq_resolution_hz']:.2f} Hz")
    t = report["timing"]
    if t["dt_us_median"]:
        print(f"  sample spacing: median {t['dt_us_median']:.0f} us  "
              f"p99 {t['dt_us_p99']:.0f}  max {t['dt_us_max']:.0f}")
    fw = report["firmware"]
    print(f"  firmware: accel +-{fw['accel_range_g']}g  gyro +-{fw['gyro_range_dps']}dps  "
          f"AAF accel {fw['accel_aaf_hz']}Hz gyro {fw['gyro_aaf_hz']}Hz")
    r = report["rotor"]
    print(f"  rotor: motor_upper median {r['motor_upper_us_median']:.0f} us  "
          f"spinning={r['spinning']}")

    for kind in ("accel", "gyro"):
        c = report["clipping"][kind]
        print(f"  {kind} raw: |max| {c['abs_max_lsb']:.0f} LSB  p99.9 {c['p99_9_lsb']:.0f}  "
              f"clipped {c['clipped_pct']:.2f}%")

    print("  dominant tones (raw, pre-LPF):")
    for key in sorted(report["raw_spectra"]):
        block = report["raw_spectra"][key]
        peaks = ", ".join(f"{p['freq_hz']:.1f}Hz({p['amplitude']:.4f})"
                          for p in block["peaks_hz"][:3])
        print(f"    {key:<10} rms {block['rms']:.4f} {block['unit']:<4} {peaks}")

    if report["lpf_measured"]:
        print("  measured LPF attenuation:")
        for key in sorted(report["lpf_measured"]):
            m = report["lpf_measured"][key]
            att = f"{m['attenuation_db']:.1f} dB" if m["attenuation_db"] is not None else "n/a"
            print(f"    {key:<10} {m['pre_rms']:.4f} -> {m['post_rms']:.4f}  {att}")

    fh = report["fusion_health"]
    print(f"  fusion: accel_ignored {fh['accel_ignored']}%  "
          f"norm_rejected {fh['norm_rejected']}%  "
          f"recovery {fh['accel_recovery']}%  "
          f"innovation median {fh['accel_error_deg_median']:.2f} deg")

    rec = report["recommendation"]
    print("  recommendation:")
    print(f"    dominant gyro tone: {rec['dominant_gyro_tone_hz']} Hz")
    print(f"    suggested gyro LPF: {rec['suggested_gyro_lpf_hz']} Hz   "
          f"accel LPF: {rec['suggested_accel_lpf_hz']} Hz")
    for note in rec["notes"]:
        print(f"    ! {note}")
    if not rec["notes"]:
        print("    (no anomalies flagged)")


def read_csv(path: Path) -> tuple[list[dict], dict]:
    rows = []
    with path.open(newline="", encoding="utf-8") as fh:
        for raw in csv.DictReader(fh):
            row = {}
            for k, v in raw.items():
                if v == "" or v is None:
                    row[k] = ""
                elif k in ("index", "timestamp_us", "motor_upper_us", "motor_lower_us",
                           "servo_alpha_us", "servo_beta_us", "fusion_flags",
                           "flight_flags", "armed", "gyro_bias_ready") or k.startswith("raw_"):
                    row[k] = int(float(v))
                else:
                    row[k] = float(v)
            rows.append(row)

    meta = {}
    sidecar = path.with_name(path.stem + "_meta.json")
    if sidecar.exists():
        meta = json.loads(sidecar.read_text(encoding="utf-8")).get("block_header", {})
    meta.setdefault("accel_range_g", 16)
    meta.setdefault("gyro_range_dps", 1000)
    return rows, meta


def list_serial_ports() -> list[str]:
    """Available serial ports, newest-looking first. Empty if pyserial is absent."""
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    return [p.device for p in list_ports.comports()]


def run_capture(port: str, *, baud: int = 115200, samples: int = 0,
                tag: str = "", out_dir: Path | None = None,
                record_timeout: float = 30.0, dump_timeout: float = 60.0,
                progress=None) -> Path:
    """Record one capture and write it to CSV. Returns the CSV path.

    `progress` is an optional callable taking (message, fraction|None); it lets
    the GUI report status without this function knowing about Tk.
    """
    def report(message: str, fraction: float | None = None) -> None:
        if progress is not None:
            progress(message, fraction)
        else:
            print(f"  {message}")

    out_dir = Path(out_dir) if out_dir is not None else DEFAULT_OUT_DIR
    link = CaptureLink(port, baud)
    try:
        link.send("IMUCAP?")
        status = link.read_text_line(time.monotonic() + 3.0)
        if status:
            report(status)

        report(f"starting capture of {samples or 'full buffer'} samples ...", 0.0)
        link.send(f"IMUCAP START {samples}" if samples else "IMUCAP START")
        line = link.read_text_line(time.monotonic() + 3.0)
        if line:
            report(line)

        # Recording is bounded by the requested sample count; poll until ready.
        wait_deadline = time.monotonic() + record_timeout
        while time.monotonic() < wait_deadline:
            time.sleep(0.3)
            link.send("IMUCAP?")
            line = link.read_text_line(time.monotonic() + 2.0)
            if line and "IMUCAP" in line:
                stored = requested = None
                for token in line.split():
                    if token.startswith("stored="):
                        stored = int(token.split("=", 1)[1])
                    elif token.startswith("requested="):
                        requested = int(token.split("=", 1)[1])
                frac = (stored / requested) if (stored and requested) else None
                report(line.strip(), frac)
                if "state=ready" in line:
                    break

        link.send("IMUCAP STOP")
        link.read_text_line(time.monotonic() + 2.0)

        report("dumping over USB ...", None)
        link.send("IMUCAP DUMP")
        samples_out, meta = link.read_blocks(timeout_s=dump_timeout,
                                             verbose=progress is None,
                                             progress=progress)
        if not samples_out:
            raise RuntimeError(
                "no samples received; check that the USB CDC link is connected")
    finally:
        link.close()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"imu_vib_{tag}_{stamp}" if tag else f"imu_vib_{stamp}"
    csv_path = out_dir / f"{name}.csv"

    rows = samples_to_rows(samples_out, meta)
    write_csv(rows, csv_path)
    (out_dir / f"{name}_meta.json").write_text(json.dumps({
        "created_at": datetime.now(timezone.utc).isoformat(),
        "port": port,
        "tag": tag,
        "sample_count": len(rows),
        "block_header": meta,
    }, indent=2), encoding="utf-8")

    report(f"wrote {csv_path.name} ({len(rows)} samples)", 1.0)
    return csv_path


def do_capture(args: argparse.Namespace) -> Path:
    return run_capture(
        args.port, baud=args.baud, samples=args.samples, tag=args.tag,
        out_dir=Path(args.out_dir), record_timeout=args.record_timeout,
        dump_timeout=args.dump_timeout)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="USB CDC serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--capture", action="store_true", help="record a new capture")
    parser.add_argument("--samples", type=int, default=0,
                        help="samples to record (0 = full firmware buffer)")
    parser.add_argument("--tag", default="", help="label for the output filename, e.g. thr50")
    parser.add_argument("--analyse", action="store_true", help="analyse after capturing")
    parser.add_argument("--analyse-file", type=Path, help="analyse an existing CSV")
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--json", type=Path, help="write the analysis report as JSON")
    parser.add_argument("--record-timeout", type=float, default=30.0)
    parser.add_argument("--dump-timeout", type=float, default=60.0)
    args = parser.parse_args(argv)

    target: Path | None = args.analyse_file

    if args.capture:
        if not args.port:
            parser.error("--capture requires --port")
        target = do_capture(args)

    if (args.analyse or args.analyse_file) and target is not None:
        rows, meta = read_csv(target)
        report = analyse(rows, meta, label=target.name)
        print_report(report)
        out_json = args.json or target.with_name(target.stem + "_analysis.json")
        out_json.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\n  report written to {out_json}")
    elif not args.capture:
        parser.error("nothing to do: pass --capture and/or --analyse-file")

    return 0


if __name__ == "__main__":
    sys.exit(main())

