"""候选增益的临时应用与撤销：只写 RAM，绝不落 Flash。

普通 `PARAM SET` 成功后固件 1.5 s 会自动存 Flash——一组还没在杆上验证过的候选增益
就这样变成了下次上电的默认值。所以这里一律发 `SYSID PARAM <name> <value>`：固件只接受
`coax.rate_*` / `coax.att_*`，只写 RAM、不排自动保存，成功回
`OK sysid param name=<n> value=<v> ram=1`。回显不是这个（包括旧固件的 ERR）就失败、不重试。

**力矩单位。** 候选增益是录制那轮的固件力矩单位（见 `lever_units.py`）。点应用时先重读
一次飞控参数（`PARAM?` + 一份状态报告作顺序屏障），按当前飞控的倾转力臂换算速率环
kp/ki/kd 再写；任一力臂读不出或符号相反就拒绝。恢复原参数同理：原增益连同读它时飞控的
力臂一起存，恢复时按当前力臂换算；没记力臂的旧记录拒绝自动恢复。

应用前的原增益写进本轮存档目录的 `original_gains.json`：断线重连、甚至重开面板之后，
「恢复原参数」照样能从最近一次应用的存档读回原值。
"""
from __future__ import annotations

import json
import math
from datetime import datetime

from . import lever_units, settings_store

ORIGINAL_GAINS_FILE = "original_gains.json"
ORIGINAL_GAINS_VERSION = 2
#: 页面「临时应用」只放行姿态增益。固件的 SYSID PARAM 另接受高度环 coax.pos_z_kp、coax.vel_z_kp/ki/kd、
#: coax.vel_z_i_limit_m_s2（R-ALTID-1，只写 RAM），但那些由离线分析脚本直接经 SYSID PARAM 试用，
#: 本页的高度轮（ALT）不拟合、不出候选参数，所以这里有意不加。
_ALLOWED = ("coax.rate_", "coax.att_")
#: 应用/恢复前重读飞控参数：PARAM? 之后的第一份状态报告到时，参数一定已经收齐。
_PARAM_BARRIER = ("PARAM?", "SYSID?")


def ram_command(command: str) -> tuple[str, str]:
    """把一条候选参数命令（`SYSID PARAM n v` 或旧式 `PARAM SET n v`）拆成 `(n, v)`。"""
    parts = command.split()
    if len(parts) != 4 or parts[:2] not in (["SYSID", "PARAM"], ["PARAM", "SET"]):
        raise ValueError(f"不认识的参数命令，未应用：{command}")
    name, value = parts[2], parts[3]
    if not name.startswith(_ALLOWED):
        raise ValueError(f"只允许试用控制增益（coax.rate_* / coax.att_*），未应用：{name}")
    try:
        number = float(value)
    except ValueError:
        raise ValueError(f"参数值不是数字，未应用：{command}") from None
    if not math.isfinite(number):
        raise ValueError(f"参数值无效，未应用：{command}")
    return name, value


def read_original_record(path) -> tuple[dict, dict | None] | None:
    """`(原增益, 读它时飞控的力矩单位签名或 None)`；文件缺失/损坏返回 None。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        gains = data["gains"]
        if not isinstance(gains, dict) or not gains:
            return None
        for name, value in gains.items():
            ram_command(f"SYSID PARAM {name} {value}")
        torque = data.get("torque_model")
        return ({str(name): str(value) for name, value in gains.items()},
                torque if isinstance(torque, dict) else None)
    except (OSError, ValueError, KeyError, TypeError):
        return None


def read_original_gains(path) -> dict | None:
    record = read_original_record(path)
    return None if record is None else record[0]


def latest_original_gains_file():
    """存档里最近一次「临时应用」留下的原增益记录。"""
    root = settings_store.settings_path().parent
    found = [p for pattern in (f"*/{ORIGINAL_GAINS_FILE}", f"*/*/{ORIGINAL_GAINS_FILE}")
             for p in root.glob(pattern)]
    return max(found, key=lambda p: p.stat().st_mtime) if found else None


class RamParams:
    def apply(self, scale: float = 1.0):
        """`scale`：试用比例，只缩放 kp/ki（角速度环与姿态 P），kd 不动（本来是 0）。"""
        if self.awaiting or self.job:
            self.page.status_var.set("正在等待飞控回显或分析结束，请稍等"); return
        joint = bool(getattr(self.page, "_commands_joint", False))
        if not joint and (not self.end or self.end.get("state") != "done" or self.data_error):
            self.fail("本轮未有效结束，不能应用候选参数"); return
        if self.run_id is not None and self.end is None:
            self.fail("辨识正在进行，结束后再应用"); return
        commands = getattr(self.page, "_commands", None)
        if not commands:
            self.fail("先从有效数据合成候选参数"); return
        try:
            pairs = [ram_command(command) for command in commands]
        except ValueError as error:
            self.fail(str(error)); return
        if self.saving:
            self.page.status_var.set("本轮数据还在保存，稍等一秒再点"); return
        source = getattr(self.page, "_commands_source", None)
        self.applied_scale = scale
        self.ram_done = 0
        self.conversion_note = ""
        self._forget_airframe_params()
        self.submit([*_PARAM_BARRIER, lambda: self._first_apply_command(pairs, source, scale)],
                    "params")

    def _first_apply_command(self, pairs, source, scale):
        """参数重读完之后才生成：换算单位、记原参数、返回第一条命令，其余排进 pending。"""
        target = lever_units.torque_record(self.params)
        converted, note = lever_units.convert(pairs, source, target)
        if self.original_gains is None:
            folder = getattr(self, "candidate_dir", None) or self.saved
            here = folder / ORIGINAL_GAINS_FILE if folder else None
            stored = read_original_record(here) if here is not None and here.exists() else None
            if stored is None:
                names = [name for name, _ in pairs]
                if any(name not in self.params for name in names):
                    raise ValueError("缺少原参数回读，不能提供可靠撤销；重新读取 PARAM? 后再试")
                gains = {name: self.params[name] for name in names}
                self._write_original_gains(gains, target, here)
                stored = (gains, target)
            else:
                self.original_gains_path = here
            self.original_gains, self.original_torque = stored
        if scale != 1.0:
            converted = [(name, f"{float(value) * scale:.6g}" if name.endswith(("_kp", "_ki"))
                          else value) for name, value in converted]
        commands = [f"SYSID PARAM {name} {value}" for name, value in converted]
        self.conversion_note = note
        self.pending[:] = commands[1:]
        return commands[0]

    def _write_original_gains(self, gains, torque, path):
        if path is None:
            self.original_gains_path = None
            return
        try:
            path.write_text(json.dumps({
                "version": ORIGINAL_GAINS_VERSION,
                "saved_at": datetime.now().isoformat(timespec="seconds"),
                "note": "「临时应用」之前飞控 RAM 里的原增益；「恢复原参数」从这里读回，"
                        "并按 torque_model 里的力臂换算到恢复时飞控的力矩单位。",
                "gains": gains, "torque_model": torque}, ensure_ascii=False, indent=2),
                encoding="utf-8")
            self.original_gains_path = path
        except OSError as error:
            self.original_gains_path = None
            append = getattr(self.page.panel, "_append", None)
            if callable(append):
                append(f"[系统辨识] 原参数记录没能写进存档（{error}）；断线后将无法从存档恢复。")

    def restore(self):
        if self.awaiting or self.job:
            self.page.status_var.set("正在等待飞控回显或分析结束，请稍等"); return
        if self.run_id is not None and self.end is None:
            self.page.status_var.set("辨识正在进行，先停止再恢复参数"); return
        gains, torque = self.original_gains, getattr(self, "original_torque", None)
        source = "本次记下的原参数"
        if not gains:
            path = self.original_gains_path or latest_original_gains_file()
            record = read_original_record(path) if path is not None else None
            gains, torque = record if record is not None else (None, None)
            source = f"存档 {path.parent.name} 里的原参数" if path is not None else ""
        if not gains:
            self.page.status_var.set("找不到应用前的原参数记录（本次和存档里都没有）；"
                                     "可到参数页手动改回。"); return
        self.restore_source = source
        self.ram_done = 0
        self.conversion_note = ""
        self._forget_airframe_params()
        self.submit([*_PARAM_BARRIER, lambda: self._first_restore_command(gains, torque)],
                    "restore")

    def _first_restore_command(self, gains, torque):
        if torque is None and lever_units.needs_units(gains):
            raise ValueError("这份原参数记录没记下当时飞控的倾转力臂（旧版上位机写的），不知道它是哪套"
                             "力矩单位，拒绝自动恢复：请到参数页手动核对后改回。")
        target = lever_units.torque_record(self.params)
        converted, note = lever_units.convert(list(gains.items()), torque, target,
                                              source_what="原参数记录时")
        commands = [f"SYSID PARAM {name} {value}" for name, value in converted]
        self.conversion_note = note
        self.pending[:] = commands[1:]
        return commands[0]

    def on_ram_echo(self, values):
        """`OK sysid param name=<n> value=<v> ram=1`。"""
        name, value = values.get("name"), values.get("value")
        if name and value:
            self.params[name] = value
        if not (self.awaiting and self.awaiting.startswith("SYSID PARAM ")):
            return
        _, _, expected_name, expected_value = self.awaiting.split()
        if name != expected_name:
            return
        try:
            same = math.isclose(float(value), float(expected_value), rel_tol=1e-4, abs_tol=1.5e-6)
        except (TypeError, ValueError):
            same = False
        if values.get("ram") != "1" or not same:
            self.fail(f"参数回显不对（{name}），没确认只写 RAM，已停止应用。" + self.ram_partial_note())
            return
        self.ram_done += 1
        self.advance()

    def ram_partial_note(self):
        if self.kind in ("params", "restore") and getattr(self, "ram_done", 0):
            return f"（前面 {self.ram_done} 项已写入 RAM，可点「恢复原参数」撤回）"
        return ""

    def finish_ram(self):
        page = self.page
        note = getattr(self, "conversion_note", "")
        tail = f"\n{note}" if note else ""
        if self.kind == "restore":
            page.status_var.set(f"已恢复原参数（{self.restore_source}，只写 RAM，没存 Flash）。{tail}")
            page.analysis_note = "已恢复原参数。"
        else:
            percent = f"{getattr(self, 'applied_scale', 1.0) * 100:.0f}%"
            page.status_var.set(f"已按 {percent} 临时应用候选参数（只写 RAM，没存 Flash，断电即恢复）。{tail}")
            page.analysis_note = f"已按 {percent} 临时应用，把「本轮做」换成 RATE 再点开始。"
        page.refresh_banner()


__all__ = ["ORIGINAL_GAINS_FILE", "RamParams", "latest_original_gains_file",
           "ram_command", "read_original_gains", "read_original_record"]
