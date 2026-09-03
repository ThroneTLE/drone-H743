#!/usr/bin/env python3
"""光流质量 vs 噪声扫描：为质量门限取值提供实测依据。

用法（传感器保持静止，真值速度恒为 0）：
    python tools/flow_quality_probe.py --seconds 100 --port COM31

采集期间人为改变光照/表面/高度，把 quality 铺开。关键在于读的是 `FLOW raw`
里 Driver 层统计的 `vx_avg` / `vx_pp`——它们在任何质量门之前算出，所以哪怕
该帧会被门拒掉，也照样能看到"如果收下会是什么值"。

输出按 quality 分桶给出等效速度噪声：
    sigma_v ≈ (vx_pp / 4) * 0.01 * height_m
这条换算与固件 `v = filt_vel * 0.01 * height` 同源。
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
import time
from pathlib import Path

import serial

FLOW_KV = re.compile(r"([A-Za-z0-9_]+)=(-?\d+)")


def parse_kv(line: str) -> dict[str, int]:
    return {k: int(v) for k, v in FLOW_KV.findall(line)}


def collect(port: str, seconds: float, period: float) -> list[dict[str, int | float]]:
    rows: list[dict[str, int | float]] = []
    try:
        link = serial.Serial(port, 115200, timeout=0.25)
    except serial.SerialException as exc:
        raise SystemExit(
            f"打不开 {port}：{exc}\n"
            "串口同一时刻只能被一个进程占用。地面站面板如果连着同一个口，"
            "请先在面板上断开连接（或关掉面板）再跑本脚本。"
        ) from exc
    try:
        time.sleep(0.4)
        link.reset_input_buffer()
        start = time.monotonic()
        while (time.monotonic() - start) < seconds:
            link.write(b"FLOW?\r\n")
            link.flush()
            raw: dict[str, int] = {}
            mico: dict[str, int] = {}
            deadline = time.monotonic() + period
            while time.monotonic() < deadline:
                text = link.readline().decode("utf-8", "replace").strip()
                if text.startswith("FLOW raw "):
                    raw = parse_kv(text)
                elif text.startswith("FLOW mico "):
                    mico = parse_kv(text)
            if not raw or not mico:
                continue
            rows.append({
                "t_s": round(time.monotonic() - start, 2),
                "quality": mico.get("quality", 0),
                "q_avg": raw.get("q_avg", 0),
                "vx_avg": raw.get("vx_avg", 0),
                "vy_avg": raw.get("vy_avg", 0),
                "vx_pp": raw.get("vx_pp", 0),
                "vy_pp": raw.get("vy_pp", 0),
                "dist_mm": raw.get("dist_avg", 0),
                "strength": raw.get("strength_avg", 0),
                "dt_avg_us": raw.get("dt_avg", 0),
            })
    finally:
        link.close()
    return rows


def sigma_v_m_s(row: dict[str, int | float], axis: str) -> float:
    """峰峰值折等效 RMS（除以 4），再按固件同款换算成 m/s。"""
    height_m = float(row["dist_mm"]) * 0.001
    return (float(row[f"v{axis}_pp"]) / 4.0) * 0.01 * height_m


def report(rows: list[dict[str, int | float]]) -> None:
    if not rows:
        print("没有采到样本")
        return

    qs = [int(r["q_avg"]) for r in rows]
    print(f"样本 {len(rows)} 个，跨度 {rows[-1]['t_s']:.1f}s")
    print(f"q_avg 范围 {min(qs)}..{max(qs)}，均值 {statistics.mean(qs):.0f}")
    print(f"高度 {min(int(r['dist_mm']) for r in rows)}.."
          f"{max(int(r['dist_mm']) for r in rows)} mm")

    print("\n--- quality 分桶 → 静止时的等效速度噪声（真值应为 0）---")
    print(f"{'q 桶':>10} {'样本':>5} {'|vx_avg|':>9} {'σvx m/s':>9} "
          f"{'σvy m/s':>9} {'高度mm':>7}")
    edges = [0, 30, 40, 50, 60, 70, 80, 100, 120, 150, 180, 256]
    for lo, hi in zip(edges, edges[1:]):
        bucket = [r for r in rows if lo <= int(r["q_avg"]) < hi]
        if not bucket:
            continue
        print(f"{lo:>4}~{hi - 1:<5} {len(bucket):>5} "
              f"{statistics.mean(abs(int(r['vx_avg'])) for r in bucket):>9.1f} "
              f"{statistics.mean(sigma_v_m_s(r, 'x') for r in bucket):>9.3f} "
              f"{statistics.mean(sigma_v_m_s(r, 'y') for r in bucket):>9.3f} "
              f"{statistics.mean(int(r['dist_mm']) for r in bucket):>7.0f}")

    print("\n--- 跌破门限 80 的事件（瞬态深度与时长）---")
    episode: list[dict[str, int | float]] = []
    episodes: list[list[dict[str, int | float]]] = []
    for row in rows:
        if int(row["q_avg"]) < 80:
            episode.append(row)
        elif episode:
            episodes.append(episode)
            episode = []
    if episode:
        episodes.append(episode)
    if not episodes:
        print("  本次采集期间 q_avg 未跌破 80")
    for index, ep in enumerate(episodes, start=1):
        span = float(ep[-1]["t_s"]) - float(ep[0]["t_s"])
        print(f"  #{index}  t={ep[0]['t_s']:.1f}~{ep[-1]['t_s']:.1f}s "
              f"({span:.1f}s)  最低 q={min(int(r['q_avg']) for r in ep)}  "
              f"期间 σvx 最大 {max(sigma_v_m_s(r, 'x') for r in ep):.3f} m/s")
        if span > 0.25:
            print("       ↑ 超过 FLOW_STALE_RESET_MS(250ms)：现固件会把水平速度归零")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM31")
    parser.add_argument("--seconds", type=float, default=100.0)
    parser.add_argument("--period", type=float, default=0.10)
    parser.add_argument("--csv", default="")
    args = parser.parse_args()

    rows = collect(args.port, args.seconds, args.period)
    report(rows)

    if args.csv and rows:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\n原始样本已写入 {path}")


if __name__ == "__main__":
    main()
