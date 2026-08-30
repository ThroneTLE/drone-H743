"""PIPELINE M1 底层与原始数据健康 · 实机基线采集器。

通过 USB CDC 裸文本命令通道（APP_Control_ProcessLine）以固定频率轮询
`IMU?`，在静止条件下累计一段时间的证据，输出结构化 JSON 报告：

- 链路健康：轮询完整率（provenance/sample/health/calibration 四行齐）、
  期间捕获的 USBCDC/UART 周期统计行；
- 器件与采样：WHO_AM_I、firmware 上报 rate_hz、seq/ts 推算实际速率、
  时间戳单调性、seq 跳变（丢样窗口）；
- 静止合理性：|a| 相对 1g 偏差、陀螺静止均值/峰值、温度范围。

只读工具：不发送任何写入、校准或执行器命令。判据故意保守，任何一项
FAIL 都应回写 PIPELINE.md 并阻止 M1 置绿。
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import time
from datetime import datetime
from pathlib import Path

import serial

try:
    from project_paths import M1_BASELINE_ANALYSIS_DIR, dated_directory, ensure_directory
except ImportError:  # 直接以脚本运行时
    from tools.project_paths import (  # type: ignore
        M1_BASELINE_ANALYSIS_DIR,
        dated_directory,
        ensure_directory,
    )

PROVENANCE_RE = re.compile(
    r"IMU sample valid=(\d) source=(\S+) frame=(\S+) units=(\S+) contract=(\d+) "
    r"migration=(\S+) orientation=(\d+) ts_ms=(\d+) seq=(\d+) bias=\d+ armed=(\d) "
    r"m1=(\d+) m2=(\d+) temp_cdeg=(-?\d+)"
)
SAMPLE_RE = re.compile(
    r"IMU sample seq=(\d+) ax=(-?\d+) ay=(-?\d+) az=(-?\d+) "
    r"gx=(-?\d+) gy=(-?\d+) gz=(-?\d+)"
)
HEALTH_RE = re.compile(r"IMU health level=(\d+) rate_hz=(\d+) fault=(\d+) fault_ever=(\d+)")
ID_RE = re.compile(r"IMU ok=(\d) stage=(\S+).* who=(\S+) exp=(\S+)")
CAL_RE = re.compile(r"IMU calibration cal_generation=(\d+) valid_mask=(\S+) firmware_crc32=(\S+)")
STATS_RE = re.compile(r"(USBCDC tx_sent=\d+ tx_dropped=\d+|UART rx_bytes=\d+.*)")

EXPECTED_RATE_HZ = 990.0
RATE_TOLERANCE = 0.05          # ±5%
GRAVITY_TOLERANCE_MG = 40      # |a|-1000mg 允差（含未标定零偏）
GYRO_STATIC_LIMIT_MDPS = 800   # 静止陀螺均值绝对上限（0.8 dps，未标定）
COMPLETE_RATIO_PASS = 0.99
TEMP_RANGE_C = (10.0, 45.0)


def poll_once(port: serial.Serial) -> dict | None:
    port.reset_input_buffer()
    port.write(b"IMU?\r\n")
    port.flush()
    deadline = time.monotonic() + 0.35
    text = ""
    while time.monotonic() < deadline:
        chunk = port.read(4096)
        if chunk:
            text += chunk.decode(errors="replace")
            # health 行在 calibration 行之后到达；两者齐了才收工，否则会与
            # 最后一行发生竞态（首版实测 588 轮询丢 6 次 health 行）。
            if HEALTH_RE.search(text) and CAL_RE.search(text):
                break
    record: dict = {"raw_lines": text.count("\n")}
    for key, pattern in (
        ("provenance", PROVENANCE_RE),
        ("sample", SAMPLE_RE),
        ("health", HEALTH_RE),
        ("id", ID_RE),
        ("calibration", CAL_RE),
    ):
        match = pattern.search(text)
        record[key] = match.groups() if match else None
    stats = STATS_RE.findall(text)
    if stats:
        record["periodic_stats"] = stats
    return record


def run(port_name: str, seconds: float, hz: float) -> dict:
    period = 1.0 / hz
    port = serial.Serial(port_name, 115200, timeout=0.05)
    time.sleep(0.3)

    polls: list[dict] = []
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        cycle_start = time.monotonic()
        polls.append(poll_once(port))
        sleep_left = period - (time.monotonic() - cycle_start)
        if sleep_left > 0:
            time.sleep(sleep_left)
    port.close()

    complete = [p for p in polls if p["provenance"] and p["sample"] and p["health"]]
    findings: list[str] = []
    verdict = "PASS"

    def fail(msg: str) -> None:
        nonlocal verdict
        verdict = "FAIL"
        findings.append("FAIL: " + msg)

    def warn(msg: str) -> None:
        nonlocal verdict
        if verdict != "FAIL":
            verdict = "WARN"
        findings.append("WARN: " + msg)

    complete_ratio = len(complete) / max(len(polls), 1)
    if complete_ratio < COMPLETE_RATIO_PASS:
        fail(f"轮询完整率 {complete_ratio:.1%} < {COMPLETE_RATIO_PASS:.0%}（链路丢行）")

    report: dict = {
        "format": "drone-h743-m1-baseline",
        "schema": 1,
        "created_at": datetime.now().astimezone().isoformat(),
        "port": port_name,
        "duration_s": seconds,
        "poll_hz": hz,
        "polls_total": len(polls),
        "polls_complete": len(complete),
        "complete_ratio": round(complete_ratio, 4),
    }

    if complete:
        first_id = next((p["id"] for p in complete if p["id"]), None)
        if first_id:
            ok, stage, who, exp = first_id
            report["imu_id"] = {"ok": int(ok), "stage": stage, "who_am_i": who, "expected": exp}
            if ok != "1" or who != exp:
                fail(f"IMU 器件异常 ok={ok} who={who} exp={exp}")

        ts = [int(p["provenance"][7]) for p in complete]
        seq = [int(p["provenance"][8]) for p in complete]
        temps = [int(p["provenance"][12]) / 100.0 for p in complete]
        armed = {p["provenance"][9] for p in complete}
        frames = {p["provenance"][2] for p in complete}

        if any(b - a <= 0 for a, b in zip(ts, ts[1:])):
            fail("ts_ms 非严格单调递增")
        if any(b - a < 0 for a, b in zip(seq, seq[1:])):
            fail("seq 出现倒退")
        rates = [
            (sb - sa) * 1000.0 / (tb - ta)
            for (sa, sb, ta, tb) in zip(seq, seq[1:], ts, ts[1:])
            if tb > ta
        ]
        if rates:
            mean_rate = statistics.fmean(rates)
            report["sample_rate"] = {
                "mean_hz": round(mean_rate, 1),
                "min_hz": round(min(rates), 1),
                "max_hz": round(max(rates), 1),
                "expected_hz": EXPECTED_RATE_HZ,
            }
            if abs(mean_rate - EXPECTED_RATE_HZ) > EXPECTED_RATE_HZ * RATE_TOLERANCE:
                fail(f"平均采样率 {mean_rate:.0f}Hz 偏离 {EXPECTED_RATE_HZ}Hz ±{RATE_TOLERANCE:.0%}")
            worst_gap = max(
                (tb - ta) - (sb - sa) * 1000.0 / mean_rate
                for (sa, sb, ta, tb) in zip(seq, seq[1:], ts, ts[1:])
            )
            report["sample_rate"]["worst_gap_ms_vs_mean"] = round(worst_gap, 1)

        health_rates = [int(p["health"][1]) for p in complete]
        faults = [(int(p["health"][2]), int(p["health"][3])) for p in complete]
        report["firmware_health"] = {
            "rate_hz_min": min(health_rates),
            "rate_hz_max": max(health_rates),
            "fault_any": int(any(f or fe for f, fe in faults)),
        }
        if any(f or fe for f, fe in faults):
            fail("IMU health fault/fault_ever 非零")

        ax = [int(p["sample"][1]) for p in complete]
        ay = [int(p["sample"][2]) for p in complete]
        az = [int(p["sample"][3]) for p in complete]
        gx = [int(p["sample"][4]) for p in complete]
        gy = [int(p["sample"][5]) for p in complete]
        gz = [int(p["sample"][6]) for p in complete]
        norm = statistics.fmean(
            (x * x + y * y + z * z) ** 0.5 for x, y, z in zip(ax, ay, az)
        )
        report["static_accel"] = {
            "mean_mg": [round(statistics.fmean(v), 1) for v in (ax, ay, az)],
            "norm_mean_mg": round(norm, 1),
        }
        if abs(norm - 1000.0) > GRAVITY_TOLERANCE_MG:
            warn(f"|a| 均值 {norm:.0f}mg 偏离 1g 超过 {GRAVITY_TOLERANCE_MG}mg")
        gyro_means = [statistics.fmean(v) for v in (gx, gy, gz)]
        report["static_gyro"] = {
            "mean_mdps": [round(v, 1) for v in gyro_means],
            "peak_mdps": max(max(map(abs, v)) for v in (gx, gy, gz)),
        }
        if max(map(abs, gyro_means)) > GYRO_STATIC_LIMIT_MDPS:
            warn(f"静止陀螺均值超 {GYRO_STATIC_LIMIT_MDPS} mdps（未标定零偏偏大或未静止）")

        report["temperature_c"] = {
            "min": min(temps),
            "max": max(temps),
            "span": round(max(temps) - min(temps), 2),
        }
        if not (TEMP_RANGE_C[0] <= min(temps) and max(temps) <= TEMP_RANGE_C[1]):
            warn(f"温度超出常温窗口 {TEMP_RANGE_C}")

        report["context"] = {
            "frames_seen": sorted(frames),
            "armed_values_seen": sorted(armed),
            "calibration": next((p["calibration"] for p in complete if p["calibration"]), None),
        }
        if armed != {"0"}:
            fail("采集期间 armed 非 0")
        periodic = [line for p in polls for line in p.get("periodic_stats", ())]
        if periodic:
            report["periodic_stats_last"] = periodic[-1]

    report["verdict"] = verdict
    report["findings"] = findings
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="M1 底层健康基线采集（只读）")
    parser.add_argument("--port", default="COM31")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--hz", type=float, default=10.0)
    args = parser.parse_args()

    report = run(args.port, args.seconds, args.hz)
    out_dir = ensure_directory(dated_directory(M1_BASELINE_ANALYSIS_DIR))
    out_path = out_dir / f"m1_baseline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    out_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"verdict={report['verdict']}")
    for finding in report["findings"]:
        print(finding)
    print(f"report={out_path}")


if __name__ == "__main__":
    main()
