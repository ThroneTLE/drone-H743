"""分析：本轮自动分析 / 「重新分析」/ 多轮联合分析。

三条路都在后台线程里算，结果经 `workflow.results` 回到 Tk 主线程（`pump`）；
后台线程不碰任何控件。几何一律取**「1 · 准备」页当前填写的值**（杆到飞控、倾转轴到飞控、
挂砝码试验），飞控到质心取那一轮的快照——这样作者补量倾转轴或台架刚度之后，已采的轮次
直接重算，不用重跑。舵机单独轮（mode=3）走 `fit_servo_only`，只出对象特性，不出参数。
高度轮（mode=4）不跑拟合：break 轮重算离地/滑落阈值（`alt_breakaway.py`），vel/pos 轮只说明离线分析。

候选参数的力矩单位随拟合一起带走（`context["source"]`，录制那轮参数回显的倾转力臂），
「临时应用」时按当前飞控的力臂换算（见 `lever_units.py`）。
"""
from __future__ import annotations

import inspect
import json
import math
import threading
from dataclasses import asdict, is_dataclass

from ._core import fit_inner_loop, fit_inner_loop_multi, fit_servo_only
from .alt_breakaway import alt_result_text
from .alt_config import ALT_MODE_CODE
from .axis_check import axis_warning, perpendicular_ratio
from .joint import read_samples
from .lever_units import torque_record
from .backlash_panel import backlash_provenance_text
from .notch_panel import notch_provenance_text
from .reasons import cannot_analyse
from .results import fit_card
from .servo_results import (SERVO_FIT_FILE, group_pendulum_hz, servo_card, servo_cross_check,
                            summarise)

REPORT_TAIL = ("\n\n推力来自当前标定查表，力矩来自最终脉宽反算；均不是直接测力矩。"
               "这一个转轴不能独立分离两个舵机的延迟，也不能证明最佳 PID。\n")


def _plain(result) -> dict:
    data = asdict(result) if is_dataclass(result) else dict(vars(result))
    data["warnings"] = list(getattr(result, "warnings", []) or [])
    return data


def _run_inputs(conditions: dict) -> dict:
    """一轮快照/存档里拟合要用的固定量。"""
    r_z = (conditions.get("parameter_echo") or {}).get("airframe.thrust_point_to_cg_z_m")
    return dict(psi=float(conditions["psi_mrad"]) * 1e-3,
                mass=float(conditions["mass_mg"]) * 1e-6,
                inertia=float(conditions["I_ugm2"]) * 1e-6,
                r_z=float(r_z) if r_z not in (None, "") else None,
                fc=float(conditions.get("imu_off_um", 0)) * 1e-6,
                d_snapshot=float(conditions["axis_off_um"]) * 1e-6)


def firmware_lever_kwargs(fit_function, parameter_echo) -> dict:
    """`firmware_tilt_levers_m=…`：拟合核心有这个帮助函数、拟合函数也收这个参数时才传。

    不靠 TypeError 兜底——先查帮助函数在不在、再查函数签名，缺哪样都不传（旧核心照旧能算）。
    """
    from . import _core
    helper = getattr(_core, "firmware_tilt_levers", None)
    if helper is None:
        return {}
    try:
        parameters = inspect.signature(fit_function).parameters
    except (TypeError, ValueError):
        return {}
    if "firmware_tilt_levers_m" not in parameters and not any(
            p.kind is inspect.Parameter.VAR_KEYWORD for p in parameters.values()):
        return {}
    try:
        levers = helper(parameter_echo or {})
    except Exception:
        levers = None
    return {"firmware_tilt_levers_m": levers}


class Analysis:
    def fit(self, manual: bool = False):
        p = self.page
        try:
            if self.awaiting or self.job:
                raise ValueError("正在等待飞控回显或上一次分析，稍后再试")
            if not self.end:
                raise ValueError("还没有正常结束的一轮数据：先点「开始辨识」跑完一轮再分析。")
            if self.end.get("state") != "done":
                raise ValueError(cannot_analyse(self.end.get("reason"),
                                                (self.snapshot or {}).get("mode")))
            if str((self.snapshot or {}).get("mode")) == str(ALT_MODE_CODE):
                # 高度轮不跑姿态拟合：break 轮重算离地/滑落阈值，写在「Z 高度」页就返回。
                p.alt_result(alt_result_text(self.snapshot, p._timestamps(), p.samples,
                                             self.alt_quality_note()))
                p.analysis_note = self.alt_quality_note()
                p.refresh_banner()
                return
            if self.data_error or p.gap_count or int(self.end.get("dropped", 0)):
                raise ValueError("这一轮数据有缺口（丢包或断点），不能跨断点拼接分析。请重新跑一轮；"
                                 "反复出现就在「高级设置」调低采样率。")
            if not p.batches or not p.batches[0].first or not p.batches[-1].last:
                raise ValueError("数据首尾不完整（缺第一包或最后一包），请重新跑一轮。")
            mode = (self.snapshot or {}).get("mode")
            if mode not in ("0", "3"):
                raise ValueError("只有「FF 测模型」和「舵机单独」的数据能拟合；RATE/ANGLE 是验证轮，"
                                 "看上面的跟踪误差。")
            run = _run_inputs(self.snapshot)
            geometry = p.current_geometry(run["fc"], run["d_snapshot"])
            # 挂砝码的角度按这一轮记录的杆轴方位换成绕杆转角。
            stiffness = p.rig_stiffness_n_m_rad(math.degrees(run["psi"]))
            times, samples = p._timestamps(), list(p.samples)
            if len(samples) < 64:
                raise ValueError("样本太少（不到 64 个），请重新跑一轮。")
        except (ValueError, KeyError, TypeError) as error:
            p.fit_var.set(str(error))
            p.analysis_note = f"没能自动分析：{error}"
            p.refresh_banner()
            return
        if stiffness is not None:
            p.save_stiffness_settings()
        extra = {} if stiffness is None else {"rig_stiffness_n_m_rad": stiffness}
        # 抬头：重算说明 + 本轮陷波与回差补偿溯源（conditions 的 rpm_notch / backlash，
        # 固件开跑时回的 SYSID NOTCH / SYSID BACKLASH）。舵机单独轮总写明补没补。
        # 杆轴方向核对放最前面：选错时整轮结果都不能用（axis_check.py）。
        axis_note = axis_warning(perpendicular_ratio(times, samples, run["psi"], mode), run["psi"])
        header = "\n".join(part for part in (axis_note,
                                             "用当前填写的几何重新计算" if manual else "",
                                             notch_provenance_text(self.snapshot),
                                             backlash_provenance_text(self.snapshot)) if part)
        if mode == "3":
            self._fit_servo(run, geometry, times, samples, extra, header)
            return

        echo = self.snapshot.get("parameter_echo")
        levers = firmware_lever_kwargs(fit_inner_loop, echo)

        def compute():
            return fit_inner_loop(times, samples, azimuth_rad=run["psi"], mass_kg=run["mass"],
                                  assumed_inertia_kg_m2=run["inertia"],
                                  thrust_point_to_cg_z_m=run["r_z"],
                                  pivot_above_cg_m=geometry["d"],
                                  roll_pivot_to_cg_z_m=geometry["roll"],
                                  pitch_pivot_to_cg_z_m=geometry["pitch"], **levers, **extra)

        context = dict(psi=run["psi"], pivot_m=geometry["d"], header=header,
                       geometry=geometry["text"], out_dir=self.saved, joint=False,
                       source=torque_record(echo))
        conditions, folder = dict(self.snapshot), self.saved
        self._launch(compute, context, "fit.json", "report.md", "# 杆上辨识：本轮拟合",
                     cross_check=lambda result: servo_cross_check(
                         result, conditions, folder, azimuth_rad=run["psi"]))

    def _fit_servo(self, run, geometry, times, samples, extra, header):
        """舵机单独轮：拟合 + 折算/对照/幅值依赖（读存档也在后台线程）。"""
        conditions = dict(self.snapshot)
        folder = self.saved

        def compute():
            # 杆摆频取同组带桨拟合（舵机单独激不起杆摆，自由拟合会跑到别的极小值）。
            pin = group_pendulum_hz(folder, conditions)
            result = fit_servo_only(times, samples, azimuth_rad=run["psi"], mass_kg=run["mass"],
                                    assumed_inertia_kg_m2=run["inertia"],
                                    pivot_above_cg_m=geometry["d"],
                                    pendulum_hz=pin[0] if pin else None, **extra)
            return summarise(result, conditions, folder, azimuth_rad=run["psi"],
                             pendulum_pin=pin)

        context = dict(psi=run["psi"], pivot_m=geometry["d"], header=header,
                       geometry=geometry["text"], out_dir=folder, joint=False, kind="servo")
        self._launch(compute, context, SERVO_FIT_FILE, "report_servo.md",
                     "# 杆上辨识：舵机单独（电机不转）", render=servo_card)

    def joint_fit(self, runs):
        """`runs`：`joint.ArchivedRun` 列表（最新的在前）。读文件和拟合都在后台线程。"""
        p = self.page
        try:
            if self.awaiting or self.job:
                raise ValueError("正在等待飞控回显或上一次分析，稍后再试")
            if self.run_id is not None and self.end is None:
                raise ValueError("辨识正在进行，结束后再联合分析")
            latest = runs[0]
            run = _run_inputs(latest.conditions)
            geometry = p.current_geometry(run["fc"], run["d_snapshot"])
            stiffness = p.rig_stiffness_n_m_rad(math.degrees(run["psi"]))
        except (ValueError, KeyError, TypeError, IndexError) as error:
            p.fit_var.set(f"没能联合分析：{error}")
            return
        if stiffness is not None:
            p.save_stiffness_settings()
        extra = {} if stiffness is None else {"rig_stiffness_n_m_rad": stiffness}
        folders = [r.folder for r in runs]
        # 联合的各轮同一套力矩单位（勾选时已检查力矩模型与力臂），固件力臂取第一轮的参数回显。
        echo = runs[0].conditions.get("parameter_echo")
        levers = firmware_lever_kwargs(fit_inner_loop_multi, echo)

        def compute():
            records = [read_samples(folder) for folder in folders]
            return fit_inner_loop_multi(records, azimuth_rad=run["psi"], mass_kg=run["mass"],
                                        assumed_inertia_kg_m2=run["inertia"],
                                        thrust_point_to_cg_z_m=run["r_z"],
                                        pivot_above_cg_m=geometry["d"],
                                        roll_pivot_to_cg_z_m=geometry["roll"],
                                        pitch_pivot_to_cg_z_m=geometry["pitch"], **levers,
                                        **extra)

        names = "、".join(folder.name for folder in folders)
        header = (f"存档单轮重新分析：{names}（用当前填写的几何）" if len(folders) == 1
                  else f"联合 {len(folders)} 轮：{names}（用当前填写的几何）")
        context = dict(psi=run["psi"], pivot_m=geometry["d"], header=header,
                       geometry=geometry["text"], out_dir=latest.folder, joint=True,
                       runs=[str(folder) for folder in folders], source=torque_record(echo))
        self._launch(compute, context, "fit_joint.json", "report_joint.md",
                     f"# 杆上辨识：联合 {len(folders)} 轮",
                     cross_check=lambda result: servo_cross_check(
                         result, latest.conditions, latest.folder, azimuth_rad=run["psi"]))

    def _launch(self, compute, context, json_name, report_name, title, render=None,
                cross_check=None):
        """后台算 → 写 JSON 与报告 → 回主线程。`cross_check(result)` 给一句同组交叉核对
        （带桨拟合对照舵机单独轮），放进 `context["cross_check"]`，失败就不报。"""
        p = self.page
        token = object()
        self.job = token
        p.fit_var.set("后台分析中…")
        p.fit_ref_var.set("")
        p.plant_var.set("")
        p.analysis_note = "正在分析…"
        p.refresh_banner()
        out_dir = context.get("out_dir")

        def work():
            try:
                result = compute()
            except Exception as error:     # 后台线程：任何失败都只变成一句中文原因
                self.results.put(("fit_error", (token, str(error))))
                return
            note = ""
            if cross_check is not None:
                try:
                    context["cross_check"] = cross_check(result)
                except Exception:          # 核对是附加信息：读存档失败不影响本次结果
                    context["cross_check"] = ""
            if out_dir:
                try:
                    data = _plain(result)
                    data["geometry"] = context.get("geometry")
                    if context.get("runs"):
                        data["runs"] = context["runs"]
                    if context.get("source") is not None:
                        data["torque_model"] = context["source"]    # 候选参数是哪套力矩单位
                    (out_dir / json_name).write_text(
                        json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
                    card = (render(result) if render is not None else
                            fit_card(result, pivot_m=context.get("pivot_m"),
                                     header=context.get("header", ""),
                                     geometry=context.get("geometry", "")))
                    if context.get("cross_check"):
                        card += "\n" + context["cross_check"]
                    (out_dir / report_name).write_text(
                        f"{title}\n\n" + card.replace("\n", "\n\n") + REPORT_TAIL,
                        encoding="utf-8")
                except Exception as error:
                    note = f"（拟合结果写盘失败：{error}）"
            self.results.put(("fit", (token, result, note, context)))

        threading.Thread(target=work, daemon=True).start()
        self._schedule_pump()


__all__ = ["Analysis"]
