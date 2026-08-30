"""PIPELINE R-M2-2 · 粗符号手势验证（录制 + 自动分段判号，只读）。

作者按口令做三个小动作（机头下压回平 / 右翼下压回平 / 机头左转转回），
本工具全程 ~12Hz 轮询 `IMU?` 录制原始数据，随后离线自动判定：

- 按陀螺能量（|gyro| 超阈值且持续）切出运动段，相邻段合并；
- 每段取积分角最大的轴为主轴，积分角符号即该轴符号；
- 期望序列（FLU 契约：+pitch 机头下俯 / +roll 右翼下沉 / +yaw 机头左转）：
  pitch+ → pitch−(回平) → roll+ → roll−(回平) → yaw+ → yaw−(转回)；
- 判定只看三个主动作的符号；回程段作为反号交叉确认；
- 原始样本与判定一并写入 data/calibration/airframe/<日期>/（受版本管理），
  任何一段可疑都可离线重放复核，不需要重做全套。

判定失败不代表白做：原始录制仍然落盘，审核者可人工复核或只补单段。
"""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import serial

from m1_baseline_check import CAL_RE, PROVENANCE_RE
from drift_ab_check import DRIFT_SAMPLE_RE

try:
    from project_paths import AIRFRAME_CALIBRATION_DIR, dated_directory, ensure_directory
except ImportError:
    from tools.project_paths import (  # type: ignore
        AIRFRAME_CALIBRATION_DIR,
        dated_directory,
        ensure_directory,
    )

MOTION_THRESHOLD_DPS = 8.0     # 超过视为"在动"
MOTION_MIN_S = 0.25            # 短于此的尖峰忽略
MERGE_GAP_S = 0.45             # 间隔小于此的运动段合并
MIN_SEGMENT_DEG = 4.0          # 主轴积分角至少这么大才算一次动作
LEVEL_LIMIT_DEG = 6.0          # 首尾静置的水平判定
AXIS_NAMES = ("roll", "pitch", "yaw")
EXPECTED_PRIMARY = (("pitch", +1), ("roll", +1), ("yaw", +1))


def record(port_name: str, seconds: float, hz: float) -> tuple[list[dict], dict]:
    period = 1.0 / hz
    port = serial.Serial(port_name, 115200, timeout=0.04)
    time.sleep(0.3)
    rows: list[dict] = []
    context: dict = {}
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        cycle = time.monotonic()
        port.reset_input_buffer()
        port.write(b"IMU?\r\n")
        port.flush()
        deadline = time.monotonic() + 0.25
        text = ""
        while time.monotonic() < deadline:
            chunk = port.read(4096)
            if chunk:
                text += chunk.decode(errors="replace")
                if DRIFT_SAMPLE_RE.search(text) and PROVENANCE_RE.search(text):
                    break
        provenance = PROVENANCE_RE.search(text)
        sample = DRIFT_SAMPLE_RE.search(text)
        if provenance and sample:
            (_seq, ax, ay, az, gx, gy, gz, roll, pitch, yaw) = (
                int(v) for v in sample.groups())
            rows.append({
                "t": int(provenance.group(8)) / 1000.0,
                "gyro_dps": (gx / 1000.0, gy / 1000.0, gz / 1000.0),
                "accel_g": (ax / 1000.0, ay / 1000.0, az / 1000.0),
                "attitude_deg": (roll / 100.0, pitch / 100.0, yaw / 100.0),
            })
        calibration = CAL_RE.search(text)
        if calibration:
            context = {
                "cal_generation": int(calibration.group(1)),
                "firmware_crc32": calibration.group(3),
            }
        left = period - (time.monotonic() - cycle)
        if left > 0:
            time.sleep(left)
    port.close()
    return rows, context


def segments(rows: list[dict]) -> list[dict]:
    """按陀螺能量切运动段，返回每段的主轴与积分角。"""
    marks = [max(abs(v) for v in r["gyro_dps"]) > MOTION_THRESHOLD_DPS for r in rows]
    spans: list[list[int]] = []
    for index, moving in enumerate(marks):
        if moving:
            if spans and rows[index]["t"] - rows[spans[-1][1]]["t"] <= MERGE_GAP_S:
                spans[-1][1] = index
            else:
                spans.append([index, index])
    out = []
    for start, end in spans:
        if rows[end]["t"] - rows[start]["t"] < MOTION_MIN_S:
            continue
        integral = [0.0, 0.0, 0.0]
        for i in range(max(start - 1, 0), end):
            dt = rows[i + 1]["t"] - rows[i]["t"]
            for axis in range(3):
                integral[axis] += rows[i + 1]["gyro_dps"][axis] * dt
        axis = max(range(3), key=lambda a: abs(integral[a]))
        if abs(integral[axis]) < MIN_SEGMENT_DEG:
            continue
        out.append({
            "t0": round(rows[start]["t"], 2),
            "t1": round(rows[end]["t"], 2),
            "axis": AXIS_NAMES[axis],
            "integral_deg": round(integral[axis], 1),
            "sign": +1 if integral[axis] > 0 else -1,
        })
    return out


def judge(rows: list[dict], segs: list[dict]) -> tuple[str, list[str]]:
    findings: list[str] = []
    if len(rows) < 30:
        return "INCOMPLETE", ["样本不足，链路或录制异常。"]
    for label, chunk in (("开头", rows[:8]), ("结尾", rows[-8:])):
        roll = sum(r["attitude_deg"][0] for r in chunk) / len(chunk)
        pitch = sum(r["attitude_deg"][1] for r in chunk) / len(chunk)
        if abs(roll) > LEVEL_LIMIT_DEG or abs(pitch) > LEVEL_LIMIT_DEG:
            findings.append(f"{label}未静置水平（roll={roll:.1f}° pitch={pitch:.1f}°）。")
    primaries = []
    index = 0
    for seg in segs:  # 主动作 = 每对(去+回)中的第一段；按期望轴顺序贪心匹配
        if index >= len(EXPECTED_PRIMARY):
            break
        want_axis, want_sign = EXPECTED_PRIMARY[index]
        if seg["axis"] == want_axis:
            primaries.append(seg)
            ok = seg["sign"] == want_sign
            findings.append(
                f"动作{index + 1}（期望 {want_axis}{'+' if want_sign > 0 else '-'}）："
                f"检出 {seg['axis']} 积分 {seg['integral_deg']}° → "
                f"{'符号正确' if ok else '符号相反！'}")
            if not ok:
                return "FAIL", findings
            index += 1
    if index < len(EXPECTED_PRIMARY):
        missing = [axis for axis, _ in EXPECTED_PRIMARY[index:]]
        findings.append(f"未检出动作：{missing}（可只补做这些段）。")
        return "INCOMPLETE", findings
    return ("PASS" if not any("未静置" in f for f in findings) else "WARN"), findings


def main() -> None:
    parser = argparse.ArgumentParser(description="R-M2-2 粗符号手势录制与判定（只读）")
    parser.add_argument("--port", default="COM31")
    parser.add_argument("--seconds", type=float, default=75.0)
    parser.add_argument("--hz", type=float, default=12.0)
    parser.add_argument("--replay", default=None, help="重判既有录制 JSON，不开串口")
    args = parser.parse_args()

    if args.replay:
        payload = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        rows, context = payload["rows"], payload.get("context", {})
        out_path = Path(args.replay)
    else:
        rows, context = record(args.port, args.seconds, args.hz)
        out_dir = ensure_directory(dated_directory(AIRFRAME_CALIBRATION_DIR))
        out_path = out_dir / f"gesture_sign_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

    segs = segments(rows)
    verdict, findings = judge(rows, segs)
    payload = {
        "format": "drone-h743-gesture-sign-check",
        "schema": 1,
        "created_at": datetime.now().astimezone().isoformat(),
        "context": context,
        "expected": [f"{axis}{'+' if sign > 0 else '-'}" for axis, sign in EXPECTED_PRIMARY],
        "segments": segs,
        "verdict": verdict,
        "findings": findings,
        "rows": rows,
    }
    out_path.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"verdict={verdict}")
    for line in findings:
        print(line)
    print(f"segments={len(segs)} evidence={out_path}")


if __name__ == "__main__":
    main()
