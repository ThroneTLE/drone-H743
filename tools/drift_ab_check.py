"""PIPELINE R-M3-2 · 静止漂移 A/B 采集与对比（只读）。

以 ~8Hz 轮询 `IMU?` 采集一段静止遥测，喂给 stationary_drift 分析器生成
DriftReport 并落盘，然后与最近一份历史基线（默认取非当日最新，即
2026-08-29 旧固件基线）做 A/B。验收判据（PIPELINE R-M3-2）：新报告的
整体分级不劣于基线（PASS<WARN<FAIL<INCOMPLETE），逐项差异原文记录。

同标定代次对比时 stationary_drift.compare 会自带提示“这是重复性对比”，
这正是本 REQ 的语义：验证当前固件没有让传感链变差。
"""

from __future__ import annotations

import argparse
import re
import time
from datetime import date, datetime

import serial

from m1_baseline_check import CAL_RE, PROVENANCE_RE
from stationary_drift import (
    DriftSample,
    analyze_drift,
    compare,
    recent_reports,
    load_report,
    summarise,
    write_report,
)

# m1 的 SAMPLE_RE 不含姿态；漂移分析需要板上融合姿态，这里扩展捕获。
DRIFT_SAMPLE_RE = re.compile(
    r"IMU sample seq=(\d+) ax=(-?\d+) ay=(-?\d+) az=(-?\d+) "
    r"gx=(-?\d+) gy=(-?\d+) gz=(-?\d+) roll=(-?\d+) pitch=(-?\d+) yaw=(-?\d+)"
)

STATUS_RANK = {"PASS": 0, "WARN": 1, "FAIL": 2, "INCOMPLETE": 3}


def collect(port_name: str, seconds: float, hz: float) -> tuple[list[DriftSample], dict]:
    period = 1.0 / hz
    port = serial.Serial(port_name, 115200, timeout=0.05)
    time.sleep(0.3)
    samples: list[DriftSample] = []
    context: dict = {"sample_source": "IMU? poll via drift_ab_check"}
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        cycle = time.monotonic()
        port.reset_input_buffer()
        port.write(b"IMU?\r\n")
        port.flush()
        deadline = time.monotonic() + 0.30
        text = ""
        while time.monotonic() < deadline:
            chunk = port.read(4096)
            if chunk:
                text += chunk.decode(errors="replace")
                if CAL_RE.search(text) and DRIFT_SAMPLE_RE.search(text):
                    break
        provenance = PROVENANCE_RE.search(text)
        sample = DRIFT_SAMPLE_RE.search(text)
        calibration = CAL_RE.search(text)
        if provenance and sample:
            (_seq, ax, ay, az, gx, gy, gz, roll, pitch, yaw) = (int(v) for v in sample.groups())
            samples.append(DriftSample(
                timestamp_s=int(provenance.group(8)) / 1000.0,
                gyro_dps=(gx / 1000.0, gy / 1000.0, gz / 1000.0),
                accel_g=(ax / 1000.0, ay / 1000.0, az / 1000.0),
                attitude_deg=(roll / 100.0, pitch / 100.0, yaw / 100.0),
                temperature_c=int(provenance.group(13)) / 100.0,
                sequence=int(provenance.group(9)),
            ))
        if calibration:
            context["cal_generation"] = int(calibration.group(1))
            context["firmware_crc32"] = calibration.group(3)
        left = period - (time.monotonic() - cycle)
        if left > 0:
            time.sleep(left)
    port.close()
    return samples, context


def main() -> None:
    parser = argparse.ArgumentParser(description="静止漂移 A/B（只读采集）")
    parser.add_argument("--port", default="COM31")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--hz", type=float, default=8.0)
    parser.add_argument("--baseline", default=None, help="基线报告路径；默认取非当日最新")
    args = parser.parse_args()

    samples, context = collect(args.port, args.seconds, args.hz)
    report = analyze_drift(samples, context=context)
    out_path = write_report(report)
    print(f"new report: {out_path}")
    print(summarise(report))

    if args.baseline:
        base_path, base = args.baseline, load_report(__import__("pathlib").Path(args.baseline))
    else:
        today = date.today().isoformat()
        candidates = [(p, r) for p, r in recent_reports(limit=12) if today not in str(p)]
        if not candidates:
            print("verdict=INCOMPLETE (no historical baseline found)")
            return
        base_path, base = candidates[0]
    print(f"baseline: {base_path} (status={base.status}, fw={base.context.get('firmware_crc32')})")

    for line in compare(base, report):
        print("A/B:", line)

    degraded = STATUS_RANK.get(report.status, 3) > STATUS_RANK.get(base.status, 3)
    print(f"verdict={'FAIL' if degraded else 'PASS'} "
          f"(baseline={base.status} -> new={report.status}, "
          f"new_fw={context.get('firmware_crc32')}, cal_gen={context.get('cal_generation')})")


if __name__ == "__main__":
    main()
