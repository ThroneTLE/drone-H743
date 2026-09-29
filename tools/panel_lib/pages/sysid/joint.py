"""多轮联合分析：把同一台架、同一设置连采的几轮 FF 数据放在一起拟合。

一轮 4 s 的数据里 I_杆 与 κ 会此消彼长；几轮共享一组 (I_杆, G, T) 一起拟合，
整定量更稳。这里只管三件事：

* 扫存档（`ATTITUDE_IDENT_DIR/<日期>/rod_*/`），只列**已正常结束的 FF 轮**；
* 默认勾选与最近一轮**同条件**的（同一天、杆轴方向、杆高、目标推力相同；激励形状可以不同）；
* 从 `samples.csv` 还原成页面同样的 `(相对秒, 样本字典)`。拟合本身在 `analysis.py` 的后台线程里跑。
"""
from __future__ import annotations

import csv
import json
import math
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import ttk

from . import settings_store

_SKIP_COLUMNS = {"t_us", "run_id", "gap"}
_MODEL_NAMES = {"legacy": "旧力矩模型", "geometric": "几何力矩模型", "unknown": "力矩模型未知"}


def torque_model_of(parameter_echo) -> str:
    """"legacy" / "geometric" / "unknown"：这一轮录制时固件用的是哪套力矩模型。

    判别规则在 `_core.torque_model_signature`（拟合核心维护）；拿不到就当 "unknown"，
    页面不自己猜。2026-09-27 起固件改成几何力臂，前后两套力矩单位不同，不能混在一起联合。
    """
    try:
        from . import _core
        helper = getattr(_core, "torque_model_signature", None)
        return str(helper(parameter_echo or {})) if helper is not None else "unknown"
    except Exception:
        return "unknown"


def torque_levers_of(parameter_echo) -> tuple | None:
    """录制时固件的倾转力臂（0.1 mm 取整，作分组键）；读不出 None。

    同是几何力矩模型，重心或舵机转轴一改力臂就变（今晚 0.0354 m，重心改到 −0.01 后 0.12 m），
    力矩单位跟着变——只比签名不够，力臂不同的轮次同样不能联合。
    """
    try:
        from . import _core
        helper = getattr(_core, "firmware_tilt_levers", None)
        levers = helper(parameter_echo or {}) if helper is not None else None
        return None if levers is None else tuple(round(float(v), 4) for v in levers)
    except Exception:
        return None


_PROFILE_NAMES = {"0": "阶跃", "1": "双脉冲", "2": "扫频", "3": "伪随机"}


@dataclass
class ArchivedRun:
    folder: Path
    conditions: dict

    @property
    def torque_model(self) -> str:
        return torque_model_of(self.conditions.get("parameter_echo"))

    @property
    def torque_levers(self) -> tuple | None:
        return torque_levers_of(self.conditions.get("parameter_echo"))

    @property
    def backlash(self) -> bool:
        """录制时开没开舵机回差补偿（conditions 的 backlash 溯源；旧轮次没有就按没开）。"""
        backlash = self.conditions.get("backlash")
        return isinstance(backlash, dict) and str(backlash.get("en")) == "1"

    @property
    def key(self) -> tuple:
        """"同条件"：同一天、杆轴方向、杆高、目标推力、同一套固件力矩模型与力臂、同一回差补偿
        开关（补偿前后舵机小幅值的延迟差约 35 ms，是两个对象）。激励形状可以不同——双脉冲和
        扫频正是要放在一起联合分析的。"""
        c = self.conditions
        target = (c.get("start") or {}).get("target_cn", c.get("target_cn"))
        return (self.folder.parent.name, str(c.get("psi_mrad")), str(c.get("axis_off_um")),
                str(target), self.torque_model, self.torque_levers, self.backlash)

    @property
    def label(self) -> str:
        c = self.conditions
        stamp = self.folder.name.split("_")[1] if self.folder.name.count("_") >= 2 else self.folder.name
        clock = f"{stamp[:2]}:{stamp[2:4]}:{stamp[4:6]}" if len(stamp) >= 6 else stamp
        try:
            psi = math.degrees(float(c["psi_mrad"]) * 1e-3)
            rod = float(c["axis_off_um"]) * 1e-6
            target = float((c.get("start") or {}).get("target_cn", c.get("target_cn", 0))) / 100.0
            amp = float(c.get("amp_mrad_s", 0)) / 1000.0
        except (KeyError, TypeError, ValueError):
            return f"{self.folder.parent.name} {clock}（{self.folder.name}）"
        shape = _PROFILE_NAMES.get(str(c.get("profile")), "激励")
        return (f"{self.folder.parent.name} {clock} · 杆轴 {psi:.0f}° · d {rod:.3f} m · "
                f"目标 {target:.1f} N · {shape} {amp:.3f} rad/s · {_MODEL_NAMES[self.torque_model]}"
                + (" · 回差补偿开" if self.backlash else ""))


def scan_runs(root: Path | None = None) -> list[ArchivedRun]:
    """已正常结束、FF 模式、带 samples.csv 的轮次，最新的在前。"""
    root = root or settings_store.settings_path().parent
    runs = []
    for path in root.glob("*/rod_*/conditions.json"):
        try:
            conditions = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        end = conditions.get("end") or {}
        if (not isinstance(end, dict) or end.get("state") != "done"
                or str(conditions.get("mode")) != "0"
                or not (path.parent / "samples.csv").is_file()):
            continue
        runs.append(ArchivedRun(path.parent, conditions))
    runs.sort(key=lambda run: (run.folder.parent.name, run.folder.name), reverse=True)
    return runs


def latest_crossover_hz(key: tuple, root: Path | None = None) -> float | None:
    """同条件（`ArchivedRun.key`）里最近一次拟合给出的速率环穿越频率；没有就 None。"""
    for run in scan_runs(root):
        if run.key != key:
            continue
        for name in ("fit_joint.json", "fit.json"):
            try:
                value = float(json.loads((run.folder / name).read_text(encoding="utf-8"))["crossover_hz"])
            except (OSError, ValueError, KeyError, TypeError):
                continue
            if math.isfinite(value) and value > 0:
                return value
    return None


def default_selection(runs: list[ArchivedRun]) -> list[bool]:
    if not runs:
        return []
    latest = runs[0].key
    return [run.key == latest for run in runs]


def read_samples(folder: Path) -> tuple[list[float], list[dict]]:
    """`samples.csv` → (相对首样本的秒数, 与页面同名字段的样本字典)。有断点就拒绝。"""
    stamps, samples = [], []
    with (folder / "samples.csv").open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("gap") not in (None, "", "0"):
                raise ValueError(f"{folder.name} 中途丢过样（有断点），不能参加联合分析")
            stamps.append(int(float(row["t_us"])))
            samples.append({name: float(value) for name, value in row.items()
                            if name not in _SKIP_COLUMNS})
    if len(samples) < 64:
        raise ValueError(f"{folder.name} 样本太少（{len(samples)} 个）")
    times = [0.0]
    for a, b in zip(stamps, stamps[1:]):
        times.append(times[-1] + ((b - a) & 0xffffffff) * 1e-6)
    return times, samples


class JointPanel:
    """「3 · 结果」里的联合分析区：扫描 → 勾选 → 后台联合拟合。"""

    def _build_joint(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="联合分析多轮（同一台架、同一设置连采的几轮放一起算，更稳）",
                             padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        row = ttk.Frame(box)
        row.pack(fill=tk.X)
        ttk.Button(row, text="联合分析多轮", command=self.scan_joint_runs).pack(side=tk.LEFT)
        self.joint_run_button = ttk.Button(row, text="用勾选的轮次联合分析",
                                           command=self.run_joint_fit, state=tk.DISABLED)
        self.joint_run_button.pack(side=tk.LEFT, padx=(6, 0))
        self.joint_hint_var = tk.StringVar(value="先点「联合分析多轮」列出存档里已正常结束的 FF 轮。")
        ttk.Label(box, textvariable=self.joint_hint_var, style="Muted.TLabel", wraplength=640,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        self.joint_list = ttk.Frame(box)
        self.joint_list.pack(fill=tk.X, pady=(4, 0))
        self.joint_choices: list[tuple[ArchivedRun, tk.BooleanVar]] = []

    def scan_joint_runs(self) -> None:
        for child in self.joint_list.winfo_children():
            child.destroy()
        try:
            runs = scan_runs()
        except OSError as error:
            self.joint_hint_var.set(f"读不了存档目录：{error}")
            runs = []
        picks = default_selection(runs)
        self.joint_choices = []
        for run, picked in zip(runs, picks):
            variable = tk.BooleanVar(value=picked)
            ttk.Checkbutton(self.joint_list, text=run.label + ("（同条件）" if picked else ""),
                            variable=variable).pack(anchor=tk.W)
            self.joint_choices.append((run, variable))
        if not runs:
            self.joint_hint_var.set("存档里没有已正常结束的 FF 轮。")
        else:
            self.joint_hint_var.set(f"找到 {len(runs)} 轮，已勾选与最近一轮同条件的 {sum(picks)} 轮；"
                                    "几何用「1 · 准备」页当前填写的值。")
        self.joint_run_button.configure(state=tk.NORMAL if runs else tk.DISABLED)

    def selected_joint_runs(self) -> list[ArchivedRun]:
        return [run for run, variable in self.joint_choices if variable.get()]

    def run_joint_fit(self) -> None:
        runs = self.selected_joint_runs()
        if not runs:
            # 只勾一轮 = 对存档里的这一轮单独重新分析（重开地面站后没有"本轮"可分析时就靠它）。
            self.joint_hint_var.set("至少勾一轮：勾一轮是单独重新分析那一轮，勾多轮是联合分析。")
            return
        if len({str(run.conditions.get("psi_mrad")) for run in runs}) > 1:
            self.joint_hint_var.set("勾选的轮次杆轴方向不同，不能放在一起算。")
            return
        if len({run.torque_model for run in runs}) > 1:
            self.joint_hint_var.set("勾选的轮次录制时固件力矩模型不同（旧模型/几何模型），"
                                    "力矩单位不一样，不能放在一起算。")
            return
        if len({run.torque_levers for run in runs}) > 1:
            self.joint_hint_var.set("勾选的轮次录制时固件的倾转力臂不同（重心或舵机转轴改过），"
                                    "力矩单位不一样，不能放在一起算。")
            return
        if len({run.backlash for run in runs}) > 1:
            self.joint_hint_var.set("勾选的轮次有的开了舵机回差补偿、有的没开：补偿前后舵机小幅值的"
                                    "延迟差约 35 ms，是两个对象，不能放在一起算。")
            return
        self.workflow.joint_fit(runs)


__all__ = ["ArchivedRun", "JointPanel", "default_selection", "latest_crossover_hz",
           "read_samples", "scan_runs", "torque_levers_of", "torque_model_of"]
