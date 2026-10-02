"""槽式台架「离地/滑落阈值」（ALT inject=break）的分析：纯函数，不碰 Tk、不写文件。

实验（跨侧契约 2026-09-30）：机体压在竖直槽底，固件把 LUT 合推力指令慢慢升（速率 r），离地后制停，
再慢慢降到滑落。上行离地推力 F_up ≈ m·g + Fs，下行滑落推力 F_down ≈ m·g − Fs（都是 LUT 合推力
**指令**，不是测得的力），所以

* mg_LUT = (F_up + F_down)/2：推力表认为「刚好托住本轮移动质量」的合推力；
* Fs = (F_up − F_down)/2：槽的静摩擦（按表值计）；
* 推力表比例 scale = m_run·g / mg_LUT（真实推力 ≈ scale × 表值）。

固件的离地/滑落粗判是 8 样本平均高度升/降 12 mm，那时推力已经多走了一截，这里往回找真正的起点：

* 高度做 60 ms 中心滑动平均；噪声 σ 取 climb 前半段原始高度的标准差，基线取同段中位数。
* F_up：settle（离地粗判）之前，平滑高度最后一次 ≤ 基线 + σ 的时刻 t_up，取那一刻的推力；
  不确定度 = 粗判推力（settle 回报）− F_up。
* F_down：excite（慢降）段，基线 = 进 excite 前 300 ms 的高度中位数，平滑高度最后一次 ≥ 基线 − σ 的时刻；
  不确定度 = F_down − 粗判推力（descend 回报）。
* 没有下行阈值（制停后已滑回槽底、慢降到下限仍没滑落、或中止）时只报 F_up：它是 mg_LUT 的上界。

`SYSID PHASE` 行没有时间戳（`run= phase= pulse_us= thrust_cn=`，thrust_cn 是进入该阶段那一拍的
推力指令），各阶段起点按推力序列对出来（`locate_phases`）。

命令行：`python -m tools.sysid.breakaway <alt_* 存档目录> [...]`，多轮时给均值与离散。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path

GRAVITY_M_S2 = 9.80665
#: 高度中心滑动平均的窗宽 [s]。
SMOOTH_S = 0.060
#: 下行基线：进 excite 前这么长的高度中位数 [s]。
PRE_EXCITE_S = 0.300
#: 记录里推力按 0.01 N 定点，指令又经脉宽量化：对推力值时留约 3 个量化单位。
THRUST_TOL_N = 0.03
#: settle 第一拍之后推力降 0.4 N：对不上 settle 回报时，按这么大的一拍下跌认出制停起点。
SETTLE_DROP_N = 0.2
#: ramp_down 从当前指令约 1 s 降到怠速：比进它时的指令低这么多才算已经开始降。
RAMP_DOWN_DROP_N = 0.1
#: 基线段至少要有这么多样本才算可靠。
MIN_BASELINE_SAMPLES = 5
MIN_SAMPLES = 20

#: 结果字典里参与多轮统计的量：(键, 中文名, 小数位)。
STAT_KEYS = (("f_up_n", "离地推力 F_up [N]", 3), ("f_down_n", "滑落推力 F_down [N]", 3),
             ("mg_lut_n", "悬停对应表值 mg_LUT [N]", 3), ("fs_n", "静摩擦 Fs [N]", 3),
             ("scale", "推力表比例", 4))


# ---------------------------------------------------------------- 小工具


def _finite(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def smooth(times_s, values, width_s: float = SMOOTH_S) -> list[float]:
    """按时间的中心滑动平均（窗宽 width_s，两端按实际可用点数平均）；非有限值当作缺测跳过。"""
    n = len(values)
    half = width_s / 2.0
    prefix, count = [0.0], [0]
    for value in values:
        ok = value is not None and math.isfinite(value)
        prefix.append(prefix[-1] + (value if ok else 0.0))
        count.append(count[-1] + (1 if ok else 0))
    out, lo, hi = [], 0, 0
    for i in range(n):
        while times_s[lo] < times_s[i] - half:
            lo += 1
        while hi < n and times_s[hi] <= times_s[i] + half:
            hi += 1
        used = count[hi] - count[lo]
        out.append((prefix[hi] - prefix[lo]) / used if used else math.nan)
    return out


def _index_at(times_s, t_s: float) -> int:
    """第一个时刻 ≥ t_s 的样本下标（都早于它就返回最后一个）。"""
    for i, t in enumerate(times_s):
        if t >= t_s:
            return i
    return len(times_s) - 1


# ---------------------------------------------------------------- 阶段起点


def _normalise(entries) -> list[tuple[float | None, str, float | None]]:
    """PHASE 回报（页面记的 dict：phase、thrust_cn）或 (t_s, 名, 推力 N) → 统一成三元组。"""
    out = []
    for entry in entries or ():
        if isinstance(entry, dict):
            name = str(entry.get("phase", ""))
            thrust_cn = _finite(entry.get("thrust_cn"))
            thrust = None if thrust_cn is None else thrust_cn / 100.0
            out.append((_finite(entry.get("t_s")), name, thrust))
        else:
            t_s, name, thrust = entry
            out.append((_finite(t_s), str(name), _finite(thrust)))
    return out


def _first(indices, predicate) -> int | None:
    for i in indices:
        if predicate(i):
            return i
    return None


def _locate_one(name, thrust, previous, thrusts, cursor) -> int | None:
    """按推力序列找一个阶段的起点下标；找不到返回 None。`previous` 是上一阶段的回报推力。"""
    n = len(thrusts)
    after = range(cursor, n)
    if name == "ramp_up":
        return cursor
    if thrust is None:
        return None
    if name in ("climb", "settle"):
        # 推力还在往上走：第一次到达回报值（settle 回报的是离地粗判那一拍的指令）。
        found = _first(after, lambda i: thrusts[i] >= thrust - THRUST_TOL_N)
        if found is None and name == "settle":
            # 对不上就找制停那一拍的下跌（F_det → F_det − 0.4 N），取下跌前一个样本。
            found = _first(range(cursor + 1, n),
                           lambda i: thrusts[i] < thrusts[i - 1] - SETTLE_DROP_N)
            found = None if found is None else found - 1
        return found
    if name == "excite":
        # settle 可能分几档降到 F_hold，停在那儿，然后开始慢降：取停在 F_hold 的最后一个样本。
        level = _first(after, lambda i: thrusts[i] <= thrust + THRUST_TOL_N)
        if level is None:
            return None
        falling = _first(range(level, n), lambda i: thrusts[i] < thrust - THRUST_TOL_N)
        if falling is None:
            return None
        index = falling - 1
        while index > level and thrusts[index] < thrust - 0.005:
            index -= 1
        return max(index, level)
    if name == "descend":
        # 慢降时推力一路往下：第一次降到回报值（滑落粗判那一拍）。
        return _first(after, lambda i: thrusts[i] <= thrust + THRUST_TOL_N)
    if name == "ramp_down":
        floor = min(thrust, previous) if previous is not None else thrust
        falling = _first(after, lambda i: thrusts[i] < floor - RAMP_DOWN_DROP_N)
        if falling is None:
            return None
        while falling > cursor and thrusts[falling - 1] < floor - THRUST_TOL_N:
            falling -= 1
        return falling
    # 闭环（vel/pos）等别的阶段：推力回报对得上就用，对不上就不定位。
    return _first(after, lambda i: abs(thrusts[i] - thrust) <= THRUST_TOL_N)


def locate_phases(times_s, thrusts_n, entries) -> list[tuple[float, str, float | None]]:
    """把 PHASE 回报按顺序对到时间轴上：`[(t_s, 阶段名, 回报推力 N), ...]`。

    带时间的条目直接用；没时间的按推力序列找（见 `_locate_one`）。某一段对不上时，后面的都不再
    对（顺序已乱，宁缺勿错）。
    """
    located, cursor, previous = [], 0, None
    for t_s, name, thrust in _normalise(entries):
        if t_s is not None:
            index = _index_at(times_s, t_s)
        else:
            index = _locate_one(name, thrust, previous, thrusts_n, cursor) if thrusts_n else None
            if index is None:
                break
            t_s = times_s[index]
        located.append((t_s, name, thrust))
        cursor, previous = index, thrust
    return located


# ---------------------------------------------------------------- 找阈值


def _window(times_s, start_s: float, end_s: float) -> list[int]:
    return [i for i, t in enumerate(times_s) if start_s <= t < end_s]


def _usable(values, indices) -> list[float]:
    return [values[i] for i in indices if values[i] is not None and math.isfinite(values[i])]


def analyse(times_s, heights_m, thrusts_n, phases, mass_kg, vbat=None) -> dict:
    """一轮 break 记录 → 离地/滑落阈值（结果字典，键见下方 `result`）。

    `phases`：`(t_s, 阶段名, 推力 N)` 列表，或页面记下的 PHASE 回报（dict，没有时间就按推力对）。
    输入本身不成立（长度不一、质量无效、样本太少）抛中文 ValueError；找不到阈值不抛，写进 notes。
    """
    times = [float(t) for t in times_s]
    heights = [_finite(h) for h in heights_m]
    thrusts = [_finite(f) for f in thrusts_n]
    if not (len(times) == len(heights) == len(thrusts)):
        raise ValueError("时间、高度、推力的样本数不一致")
    if len(times) < MIN_SAMPLES:
        raise ValueError(f"样本太少（{len(times)} 个）")
    if any(f is None for f in thrusts):
        raise ValueError("推力记录里有无效值")
    mass = _finite(mass_kg)
    if mass is None or mass <= 0.0:
        raise ValueError("本轮移动质量无效")
    weight = mass * GRAVITY_M_S2
    located = locate_phases(times, thrusts, phases)
    marks = {}
    for t_s, name, thrust in located:
        marks.setdefault(name, (t_s, thrust))
    notes: list[str] = []
    result = dict(ok=False, mass_kg=mass, weight_n=weight, phases=located, sigma_m=None,
                  baseline_up_m=None, t_up_s=None, f_up_n=None, f_up_coarse_n=None,
                  f_up_uncertainty_n=None, baseline_down_m=None, t_down_s=None, f_down_n=None,
                  f_down_coarse_n=None, f_down_uncertainty_n=None, mg_lut_n=None, fs_n=None,
                  scale=None, mg_lut_upper_n=None, scale_lower=None, vbat_v=None, notes=notes)
    volts = [v for v in (_finite(x) for x in (vbat or ())) if v is not None and v > 1.0]
    result["vbat_v"] = statistics.fmean(volts) if volts else None
    if "settle" not in marks:
        reported = any(name == "settle" for _t, name, _f in _normalise(phases))
        notes.append("settle 阶段回报的推力在记录里对不上，算不出离地推力" if reported
                     else "未离地（没有 settle 阶段回报），算不出离地推力")
        return result
    start = marks.get("climb") or marks.get("ramp_up") or (times[0], None)
    t_start, (t_det, f_det) = start[0], marks["settle"]
    if f_det is None:
        f_det = thrusts[_index_at(times, t_det)]
    smoothed = smooth(times, heights)

    # 噪声与上行基线：climb 前半段（机体还压在槽底）。
    base_idx = _window(times, t_start, t_start + (t_det - t_start) / 2.0)
    base = _usable(heights, base_idx)
    if not base:
        notes.append("离地前没有可用的高度样本，算不出离地推力")
        return result
    if len(base) < MIN_BASELINE_SAMPLES:
        notes.append(f"离地前的静止段只有 {len(base)} 个样本，基线不可靠")
    sigma = statistics.pstdev(base) if len(base) > 1 else 0.0
    baseline_up = statistics.median(base)
    result.update(sigma_m=sigma, baseline_up_m=baseline_up)
    window = _window(times, t_start, t_det)
    still = [i for i in window if math.isfinite(smoothed[i]) and smoothed[i] <= baseline_up + sigma]
    if still:
        up = still[-1]
    else:
        up = window[0] if window else _index_at(times, t_start)
        notes.append("离地粗判之前平滑高度一直高于基线 + σ（基线段可能已经在动）")
    f_up = thrusts[up]
    result.update(ok=True, t_up_s=times[up], f_up_n=f_up, f_up_coarse_n=f_det,
                  f_up_uncertainty_n=f_det - f_up)

    # 下行：excite 段慢降找滑落。
    if "excite" not in marks:
        notes.append("制停后已回到槽底（没有 excite 阶段），本轮没有下行阈值" if "ramp_down" in marks
                     else "本轮没到慢降（excite），没有下行阈值")
    elif "descend" not in marks:
        notes.append("慢降到下限仍未滑落或本轮中止（没有 descend 阶段），没有下行阈值")
    else:
        t_exc, _f_hold = marks["excite"]
        t_slide, f_slide = marks["descend"]
        if f_slide is None:
            f_slide = thrusts[_index_at(times, t_slide)]
        pre = _usable(heights, _window(times, t_exc - PRE_EXCITE_S, t_exc))
        if not pre:
            notes.append("进慢降前没有可用的高度样本，算不出滑落推力")
        else:
            baseline_down = statistics.median(pre)
            window = _window(times, t_exc, t_slide)
            held = [i for i in window
                    if math.isfinite(smoothed[i]) and smoothed[i] >= baseline_down - sigma]
            if held:
                down = held[-1]
            else:
                down = window[0] if window else _index_at(times, t_exc)
                notes.append("滑落粗判之前平滑高度一直低于基线 − σ（进慢降时可能已经在滑）")
            f_down = thrusts[down]
            result.update(baseline_down_m=baseline_down, t_down_s=times[down], f_down_n=f_down,
                          f_down_coarse_n=f_slide, f_down_uncertainty_n=f_down - f_slide)

    if result["f_down_n"] is not None:
        f_down = result["f_down_n"]
        mg_lut = (f_up + f_down) / 2.0
        result.update(mg_lut_n=mg_lut, fs_n=(f_up - f_down) / 2.0,
                      scale=weight / mg_lut if mg_lut > 0 else None)
        if f_down > f_up:
            notes.append("滑落推力高于离地推力，不合物理：核对测距与阶段回报")
    else:
        result.update(mg_lut_upper_n=f_up, scale_lower=weight / f_up if f_up > 0 else None)
    return result


# ---------------------------------------------------------------- 由存档/快照取输入


def run_mass_kg(conditions: dict) -> float:
    """本轮移动质量 [kg]：飞控开跑溯源 alt_mass_g 优先，其次页面下发的 alt_request.mass_g。"""
    for source in ((conditions.get("alt") or {}).get("alt_mass_g"),
                   (conditions.get("start") or {}).get("alt_mass_g"),
                   (conditions.get("alt_request") or {}).get("mass_g")):
        grams = _finite(source)
        if grams is not None and grams > 0:
            return grams / 1000.0
    raise ValueError("本轮条件里没有移动质量（alt_mass_g / alt_request.mass_g）")


def analyse_conditions(times_s, samples, conditions: dict) -> dict:
    """页面内存里的一轮（样本 dict 列表 + 快照）或存档读回的一轮 → `analyse` 的结果。"""
    nan = math.nan
    return analyse(times_s, [s.get("height", nan) for s in samples],
                   [s.get("thrust", nan) for s in samples],
                   (conditions or {}).get("alt_phases") or [], run_mass_kg(conditions or {}),
                   [s.get("vbat", nan) for s in samples])


def read_run_dir(path) -> tuple[list[float], list[dict], dict]:
    """读一轮存档：`(时间 s（从 0 起）, 样本, conditions)`；固件时间戳按 32 位回绕展开。"""
    folder = Path(path)
    conditions = json.loads((folder / "conditions.json").read_text(encoding="utf-8"))
    if not isinstance(conditions, dict):
        raise ValueError("conditions.json 不是对象")
    times, samples, gaps = [], [], 0
    with (folder / "samples.csv").open(newline="", encoding="utf-8") as handle:
        previous = None
        for row in csv.DictReader(handle):
            stamp = int(float(row["t_us"]))
            if previous is None:
                times.append(0.0)
            else:
                times.append(times[-1] + ((stamp - previous) & 0xFFFFFFFF) * 1e-6)
            previous = stamp
            gaps += row.get("gap") == "1"
            samples.append({key: _finite(value) for key, value in row.items()
                            if key not in ("t_us", "run_id", "gap")})
    conditions = dict(conditions, _gap_rows=gaps)
    return times, samples, conditions


def analyse_run_dir(path) -> dict:
    """一轮存档目录（samples.csv + conditions.json）→ 结果字典，另带 folder / end / data_error。"""
    times, samples, conditions = read_run_dir(path)
    result = analyse_conditions(times, samples, conditions)
    end = conditions.get("end") or {}
    result.update(folder=str(path), end_state=end.get("state"), end_reason=end.get("reason"),
                  data_error=conditions.get("data_error") or "")
    if conditions.get("_gap_rows"):
        result["notes"].append(f"记录里有 {conditions['_gap_rows']} 处断点")
    if end.get("state") not in (None, "done"):
        result["notes"].append(f"本轮中止（{end.get('reason')}），结果仅供诊断")
    return result


# ---------------------------------------------------------------- 中文摘要


def summary_text(result: dict) -> str:
    """一两行中文：离地/滑落推力、悬停对应表值、静摩擦、推力表比例、电池。"""
    notes = list(result.get("notes") or [])
    tail = [f"电池 {result['vbat_v']:.1f} V"] if result.get("vbat_v") else []
    if not result.get("ok"):
        return "离地/滑落阈值：" + ("；".join(notes) or "算不出来")
    f_up = result["f_up_n"]
    parts = [f"离地推力 F_up ≈ {f_up:.2f} N（±{abs(result['f_up_uncertainty_n']):.2f}）"]
    if result.get("f_down_n") is not None:
        parts += [f"滑落推力 F_down ≈ {result['f_down_n']:.2f} N"
                  f"（±{abs(result['f_down_uncertainty_n']):.2f}）",
                  f"悬停对应表值 {result['mg_lut_n']:.2f} N",
                  f"静摩擦 {result['fs_n']:.2f} N"]
        if result.get("scale") is not None:
            parts.append(f"推力表比例 {result['scale']:.3f}（移动质量重力 {result['weight_n']:.2f} N）")
    else:
        parts.append(f"没有滑落阈值：悬停对应表值 ≤ {f_up:.2f} N")
        if result.get("scale_lower") is not None:
            parts.append(f"推力表比例 ≥ {result['scale_lower']:.3f}"
                         f"（移动质量重力 {result['weight_n']:.2f} N）")
    text = " · ".join(parts + tail)
    return text + ("\n说明：" + "；".join(notes) if notes else "")


def group_stats(results) -> list[str]:
    """多轮的均值与离散（样本标准差）；某一项少于 2 轮就只报有几轮。"""
    lines = []
    for key, label, digits in STAT_KEYS:
        values = [r[key] for r in results if r.get(key) is not None]
        if len(values) >= 2:
            lines.append(f"{label}：均值 {statistics.fmean(values):.{digits}f}，"
                         f"标准差 {statistics.stdev(values):.{digits}f}（{len(values)} 轮）")
        else:
            lines.append(f"{label}：只有 {len(values)} 轮有值，不算离散")
    return lines


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="槽式台架离地/滑落阈值（ALT break）分析，只读存档")
    parser.add_argument("folders", nargs="+", type=Path, help="alt_* 存档目录（含 samples.csv 与 conditions.json）")
    args = parser.parse_args(argv)
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass
    results = []
    for folder in args.folders:
        try:
            result = analyse_run_dir(folder)
        except (OSError, ValueError, KeyError, json.JSONDecodeError, csv.Error) as error:
            print(f"{folder.name}：读不出来或算不出来：{error}")
            continue
        print(f"{folder.name}：{summary_text(result)}")
        if result.get("ok"):
            results.append(result)
    if len(args.folders) > 1:
        print(f"多轮汇总（{len(results)} 轮算出了离地推力）：")
        for line in group_stats(results):
            print("  " + line)
    return 0 if results else 2


__all__ = ["GRAVITY_M_S2", "analyse", "analyse_conditions", "analyse_run_dir", "group_stats",
           "locate_phases", "main", "read_run_dir", "run_mass_kg", "smooth", "summary_text"]


if __name__ == "__main__":
    raise SystemExit(main())
