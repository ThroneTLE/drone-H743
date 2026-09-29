"""「高级设置」里的「舵机回差补偿」：看状态、开/关飞控的舵机回差补偿（固件 `BACKLASH`）。

舵机与倾转连杆之间有约 1.1° 的空程（2026-09-27 舵机单独三轮按幅值依赖拟出，俯仰舵机）：
换向时舵机先空走过这段、机体才跟上，小幅修正整段落在空程里。固件的补偿在舵机换向时让它
多走一个半宽，把空程一次走完（App/Src/app_servo_backlash.c）；只补在飞控制器（解锁时）与
辨识的指令，回中、标定、点动、验收一律原样。本文件只摆控件、发命令、把回复翻成中文：

* 读状态：点「读取状态」；新连接后页面第一次空闲时、每轮结束后空闲时各自动读一次。
* 开/关：发 `BACKLASH ON|OFF`，飞控回一整份状态或一行拒绝，原样显示。解锁中、辨识进行中
  按钮灰掉并写明原因——飞控自己也会拒（reason=armed|sysid），这里先挡是为了别让人以为
  "点了没反应"。
* 回复配对：同陷波（`notch_panel.py`）——只有 en 与请求一致的那一块才算确认，3 秒等不到就作废。
* 本轮溯源：开跑时固件回一行 `SYSID BACKLASH`，工作流把它存进 conditions.json 的 backlash，
  结果抬头用 `backlash_provenance_text` 显示；舵机单独轮总要写明补没补（量的是哪一个舵机）。

阶段一只在飞控内存里：重新上电回到固件默认（关）。半宽、迟滞只能用命令改（`BACKLASH SET ...`）。
"""
from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

from ...proto import parse_kv

BACKLASH_QUERY = "BACKLASH ?"
#: 状态块两行，按这个顺序到；最后一行到齐才整块重画。
BLOCK_KINDS = ("cfg", "count")
SOURCE_TEXT = {"none": "没有（回中/上锁/其它占用）", "controller": "飞行控制器", "sysid": "辨识"}
REJECT_TEXT = {
    "armed": "飞控拒绝：已解锁。先用遥控器上锁再切换。",
    "sysid": "飞控拒绝：辨识正占用台架。结束并用遥控器上锁后再切换。",
    "range": "飞控拒绝：参数超出范围。",
    "usage": "飞控拒绝：命令格式不对（面板和固件版本可能不一致）。",
}
UNSUPPORTED = "这版固件不认 BACKLASH，没有舵机回差补偿：需要更新固件。"
#: 开/关发出后等 en 对得上的回复最多这么久（数传、蓝牙上回复要几百毫秒）。
BACKLASH_REPLY_TIMEOUT_S = 3.0


def _now() -> float:
    return time.monotonic()


def _number(values: dict, key: str, default: int = 0) -> int:
    try:
        return int(values.get(key, default))
    except (TypeError, ValueError):
        return default


def _mrad(values: dict, key: str) -> str:
    """"20 mrad（1.15°）"；读不到写问号。"""
    try:
        mrad = int(values[key])
    except (KeyError, TypeError, ValueError):
        return "? mrad"
    return f"{mrad} mrad（{math.degrees(mrad * 1e-3):.2f}°）"


def backlash_provenance_text(conditions: dict | None) -> str:
    """结果抬头的一句：这一轮舵机回差补偿开没开（conditions 的 backlash）。

    没有溯源时：舵机单独轮也要说一句（量的是不是补偿过的舵机，看结果的人必须知道），其它轮空串。
    """
    conditions = conditions or {}
    servo_only = str(conditions.get("mode")) == "3"
    backlash = conditions.get("backlash")
    if not isinstance(backlash, dict) or not backlash:
        return ("本轮舵机回差补偿：固件没报（旧固件没有这项功能），按未补偿处理。" if servo_only else "")
    if str(backlash.get("en")) != "1":
        text = "本轮舵机回差补偿：关"
        if servo_only:
            text += "（量的是舵机加连杆本来的样子，含空程）"
        return text
    text = (f"本轮舵机回差补偿：开（横滚 {backlash.get('alpha_mrad', '?')} / 俯仰 "
            f"{backlash.get('beta_mrad', '?')} mrad，换向迟滞 {backlash.get('thr_mrad', '?')} mrad）")
    if servo_only:
        text += "；这一轮量的是补偿之后的舵机，拟出的回差应接近 0，不能和未补偿的轮次合在一起看"
    return text


def describe_block(lines: dict) -> tuple[str, str]:
    """两行状态 → (大字一行, 细节几行)。"""
    cfg, count = lines.get("cfg", {}), lines.get("count", {})
    if cfg.get("en") != "1":
        head = "舵机回差补偿：关"
    elif cfg.get("active") == "1":
        head = f"舵机回差补偿：开 · 正在补偿（指令来自{SOURCE_TEXT.get(cfg.get('src', ''), '未知')}）"
    else:
        head = "舵机回差补偿：开 · 待命（上锁、回中或别的功能占着舵机时不补）"
    details = [
        f"横滚舵机（alpha）半宽 {_mrad(cfg, 'alpha_mrad')} · 俯仰舵机（beta）半宽 {_mrad(cfg, 'beta_mrad')} · "
        f"换向迟滞 {_mrad(cfg, 'thr_mrad')}",
        f"当前方向 横滚 {_number(cfg, 'dir_alpha'):+d} / 俯仰 {_number(cfg, 'dir_beta'):+d}，"
        f"脉宽多走 横滚 {_number(cfg, 'off_alpha_us'):+d} µs / 俯仰 {_number(cfg, 'off_beta_us'):+d} µs",
        f"计数：换向 横滚 {count.get('rev_alpha', '0')} · 俯仰 {count.get('rev_beta', '0')} · "
        f"重新定向 {count.get('reset', '0')} · 非有限 {count.get('nonfinite', '0')} · "
        f"补偿过的控制拍 {count.get('ticks', '0')} · 顶到标定端点 {count.get('clamped', '0')}",
    ]
    return head, "\n".join(details)


class BacklashPanel:
    def _init_backlash_vars(self) -> None:
        self.backlash_status_var = tk.StringVar(value="舵机回差补偿：状态未读取")
        self.backlash_detail_var = tk.StringVar(value="")
        self.backlash_block_var = tk.StringVar(value="")
        self.backlash_reply_var = tk.StringVar(value="")
        self.backlash_lines: dict[str, dict] = {}
        self._backlash_generation = None      # 已自动读过状态的连接代次
        self._backlash_unsupported = None     # 固件不认 BACKLASH 的连接代次
        self._backlash_refresh_pending = False
        self._backlash_action = None          # 等 ON/OFF 的回复：{"want": "1"/"0", "deadline", "seen"}

    def _build_backlash(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="舵机回差补偿", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, style="Muted.TLabel", wraplength=620, justify=tk.LEFT, text=(
            "舵机换向时先空走约 1.1° 的连杆空程、机体才跟上；打开后换向那一刻让舵机多走这一段。"
            "只补飞行控制器（解锁时）和辨识的指令，回中、标定、点动一律不补。"
            "辨识记录的倾转仍是补偿前的指令。只存在飞控内存里，重新上电回到关。"
            "做对比时关、开交替各跑一轮（舵机单独轮看回差是否接近 0）。")).pack(anchor=tk.W)
        ttk.Label(box, textvariable=self.backlash_status_var, wraplength=640,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(box, textvariable=self.backlash_detail_var, style="Mono.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        row = ttk.Frame(box)
        row.pack(fill=tk.X, pady=(6, 0))
        self.backlash_read_button = ttk.Button(row, text="读取状态", command=self.backlash_refresh)
        self.backlash_read_button.pack(side=tk.LEFT)
        self.backlash_on_button = ttk.Button(row, text="打开补偿（ON）",
                                             command=lambda: self.backlash_switch(True))
        self.backlash_on_button.pack(side=tk.LEFT, padx=(6, 0))
        self.backlash_off_button = ttk.Button(row, text="关闭补偿（OFF）",
                                              command=lambda: self.backlash_switch(False))
        self.backlash_off_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(box, textvariable=self.backlash_block_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.backlash_reply_var, wraplength=640,
                  justify=tk.LEFT).pack(anchor=tk.W)
        self.refresh_backlash_controls()

    # ------------------------------------------------------------ 能不能点

    def backlash_block_reason(self) -> str:
        """开/关此刻不能点的原因（中文）；空串 = 可以点。"""
        w = self.workflow
        transport = getattr(self.panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            return "未连接飞控。"
        if self._backlash_unsupported == w.generation():
            return UNSUPPORTED
        if (w.run_id is not None and w.end is None) or w.awaiting == "SYSID START" or (
                w.kind == "start" and (w.awaiting or w.pending)):
            return ("辨识正在进行（或正在开始）：这一轮用的补偿配置不能中途改。"
                    "结束并用遥控器上锁后再切换。")
        if w.awaiting:
            return "正在等待飞控回显，稍后再切换。"
        thr = self._current_thr()
        if thr is not None and str(thr.get("armed")) == "1":
            return "飞控已解锁：先用遥控器上锁再切换（解锁时飞控也会拒绝）。"
        return ""

    def refresh_backlash_controls(self) -> None:
        if not hasattr(self, "backlash_on_button"):
            return
        reason = self.backlash_block_reason()
        for button in (self.backlash_on_button, self.backlash_off_button):
            button.configure(state="disabled" if reason else "normal")
        transport = getattr(self.panel, "transport", None)
        connected = bool(transport is not None and getattr(transport, "is_connected", False))
        self.backlash_read_button.configure(state="normal" if connected else "disabled")
        if not reason and connected and self._current_thr() is None:
            reason = "解锁状态暂时未知；飞控解锁时会自己拒绝切换。"
        self.backlash_block_var.set(reason)

    # ------------------------------------------------------------ 发命令

    def backlash_refresh(self, quiet: bool = False) -> bool:
        sent = self.send(BACKLASH_QUERY, quiet=quiet)
        if sent and not quiet:
            self.backlash_reply_var.set("已请求回差补偿状态，等待飞控回复…")
        return sent

    def backlash_switch(self, on: bool) -> bool:
        reason = self.backlash_block_reason()
        if reason:
            self.backlash_reply_var.set(reason)
            return False
        command = "BACKLASH ON" if on else "BACKLASH OFF"
        if not self.send(command):
            self.backlash_reply_var.set(f"{command} 没发出去：{self.status_var.get()}")
            return False
        self._backlash_action = {"want": "1" if on else "0",
                                 "deadline": _now() + BACKLASH_REPLY_TIMEOUT_S, "seen": None}
        self.backlash_reply_var.set(f"已发送 {command}，等待飞控回复…")
        return True

    def backlash_expire(self) -> None:
        """开/关发出后 3 秒还没等到 en 对得上的回复：说清楚并作废。轮询每秒调一次。"""
        action = self._backlash_action
        if action is None or _now() <= action["deadline"]:
            return
        self._backlash_action = None
        command = "BACKLASH ON" if action["want"] == "1" else "BACKLASH OFF"
        if action["seen"] is not None:
            self.backlash_reply_var.set(f"飞控回复与请求不符：{command} 之后读到的是 en={action['seen']}。"
                                        "点「读取状态」再确认一次。")
        else:
            self.backlash_reply_var.set(f"{BACKLASH_REPLY_TIMEOUT_S:.0f} 秒内没等到飞控对 {command} 的回复"
                                        "（链路可能丢了一行）。点「读取状态」看实际状态。")

    def backlash_poll_idle(self) -> bool:
        """空闲时自动读一次状态：新连接后第一次、每轮结束后一次。辨识进行中不打扰。"""
        self.backlash_expire()
        w = self.workflow
        generation = w.generation()
        if self._backlash_generation == generation and not self._backlash_refresh_pending:
            return False
        if w.closed or w.awaiting or (w.run_id is not None and w.end is None):
            return False
        if self._backlash_unsupported == generation or not self._visible_and_connected():
            return False
        if not self.backlash_refresh(quiet=True):
            return False
        self._backlash_generation = generation
        self._backlash_refresh_pending = False
        return True

    # ------------------------------------------------------------ 收行

    def backlash_handle_line(self, text: str) -> bool:
        """返回 True = 这一行是补偿自己的回复，不再交给辨识工作流（它不属于任何辨识事务）。"""
        if text.startswith("SYSID end "):
            self._backlash_refresh_pending = True      # 这一轮的换向计数值得看一眼
            return False
        if text.startswith("ERR unknown cmd BACKLASH"):
            # 旧固件：不能让这行 ERR 打断正在等回显的辨识事务。
            self._backlash_unsupported = self.workflow.generation()
            self._backlash_action = None
            self.backlash_status_var.set(UNSUPPORTED)
            self.backlash_detail_var.set("")
            self.backlash_reply_var.set("")
            self.refresh_backlash_controls()
            return True
        if not text.startswith("BACKLASH "):
            return False
        values = parse_kv(text)
        if text.startswith("BACKLASH event=rejected"):
            self._backlash_action = None
            self.backlash_reply_var.set(f"{REJECT_TEXT.get(values.get('reason', ''), '飞控拒绝。')}"
                                        f"（飞控原文：{text}）")
            return True
        kind = text.split()[1]
        if kind not in BLOCK_KINDS:
            return True
        if kind == "cfg":
            self.backlash_lines = {}
            self._note_backlash_reply(values)
        self.backlash_lines[kind] = values
        if kind == BLOCK_KINDS[-1]:
            head, detail = describe_block(self.backlash_lines)
            self.backlash_status_var.set(head)
            self.backlash_detail_var.set(detail)
        return True

    def _note_backlash_reply(self, cfg: dict) -> None:
        self.backlash_expire()                 # 过期的开关先了结，不拿它配这一块
        action = self._backlash_action
        if action is None:
            if self.backlash_reply_var.get().startswith("已请求回差补偿状态"):
                self.backlash_reply_var.set("")
            return
        if cfg.get("en") != action["want"]:
            # 多半是开关之前那次查询的回复先到了：记下，接着等真正的回复（到期还没有才算不符）。
            action["seen"] = cfg.get("en")
            return
        self._backlash_action = None
        self.backlash_reply_var.set("飞控已确认：回差补偿已打开。" if action["want"] == "1"
                                    else "飞控已确认：回差补偿已关闭。")


__all__ = ["BacklashPanel", "backlash_provenance_text", "describe_block"]
