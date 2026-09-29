"""「高级设置」里的「陷波（桨叶振动）」：看状态、开/关飞控的转速陷波（固件 `RPMNOTCH`）。

固件在**控制用**陀螺上按电调转速挖掉桨叶振动线（App/Src/app_rpm_notch.c，约 107 Hz）；
记录的陀螺、姿态融合仍是原始值。本文件只摆控件、发命令、把回复翻成中文：

* 读状态：点「读取状态」；新连接后页面第一次空闲时、每轮结束后空闲时各自动读一次。
* 开/关：发 `RPMNOTCH ON|OFF`，飞控回一整份状态或一行拒绝，原样显示。解锁中、辨识进行中
  按钮灰掉并写明原因——飞控自己也会拒（reason=armed|sysid），这里先挡是为了别让人以为
  "点了没反应"。
* 回复配对：回复按发送顺序回来，开关之前发出的查询，它的回复会先到。所以只有 en 与请求一致的
  那一块才算"确认"；3 秒内等不到就说清楚并作废，免得以后哪次查询的回复被当成这次开关的结果。
* 本轮溯源：开跑时固件回一行 `SYSID NOTCH`，工作流把它存进 conditions.json 的 rpm_notch，
  结果抬头用 `notch_provenance_text` 显示。

阶段一只在飞控内存里：重新上电回到固件默认（关）。
"""
from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk

from ...proto import parse_kv

NOTCH_QUERY = "RPMNOTCH ?"
#: 状态块四行，按这个顺序到；最后一行到齐才整块重画。
BLOCK_KINDS = ("cfg", "esc", "count", "time")

STATE_TEXT = {
    "off": "关",
    "unavailable": "已打开，但不可用：这版固件没有双向 DShot 转速回传，陷波直通",
    "fs_unknown": "已打开，但陀螺采样率未知，陷波直通",
    "fs_bad": "已打开，但陀螺采样率偏离名义值超过 5%，陷波已淡出直通",
    "idle": "开 · 待命（电机没转，或转速低于淡入下限）",
    "tracking": "开 · 正在跟踪桨叶振动",
}
STATE_SHORT = {"off": "关", "unavailable": "不可用", "fs_unknown": "采样率未知",
               "fs_bad": "采样率异常", "idle": "待命", "tracking": "跟踪中"}
REJECT_TEXT = {
    "armed": "飞控拒绝：已解锁。先用遥控器上锁再切换。",
    "sysid": "飞控拒绝：辨识正占用台架。结束并用遥控器上锁后再切换。",
    "range": "飞控拒绝：参数超出范围。",
    "usage": "飞控拒绝：命令格式不对（面板和固件版本可能不一致）。",
}
UNSUPPORTED = "这版固件不认 RPMNOTCH，没有转速陷波：需要更新固件。"
#: 开/关发出后等 en 对得上的回复最多这么久（数传、蓝牙上回复要几百毫秒）。
NOTCH_REPLY_TIMEOUT_S = 3.0


def _now() -> float:
    return time.monotonic()


def _number(values: dict, key: str, default: int = 0) -> int:
    try:
        return int(values.get(key, default))
    except (TypeError, ValueError):
        return default


def harmonics_text(mask: int) -> str:
    parts = [f"{h}x" for h in (1, 2, 3) if mask & (1 << (h - 1))]
    return "+".join(parts) or "无"


def base_harmonic(mask: int) -> int:
    """固件报的权重是掩码里最低那个谐波的（默认只滤 1x）。"""
    return next((h for h in (1, 2, 3) if mask & (1 << (h - 1))), 1)


def notch_provenance_text(conditions: dict | None) -> str:
    """结果抬头的一句：这一轮控制用陀螺的陷波配置（conditions 的 rpm_notch）；没有就空串。"""
    notch = (conditions or {}).get("rpm_notch")
    if not isinstance(notch, dict) or not notch:
        return ""
    enabled = str(notch.get("en")) == "1"
    state = str(notch.get("state", ""))
    text = (f"本轮陷波（桨叶振动）：{'开' if enabled else '关'}（开跑时 "
            f"{STATE_SHORT.get(state, state or '未知')}，极对数 {notch.get('pp', '?')}）")
    if str((conditions or {}).get("mode")) in ("0", "3"):
        text += "；FF / 舵机单独轮是开环激励，不经过陷波"
    return text


def describe_block(lines: dict) -> tuple[str, str]:
    """四行状态 → (大字一行, 细节几行)。"""
    cfg, esc = lines.get("cfg", {}), lines.get("esc", {})
    count, timing = lines.get("count", {}), lines.get("time", {})
    state = cfg.get("state", "")
    head = "陷波（桨叶振动）：" + STATE_TEXT.get(state, state or "状态未知")
    source = "双向 DShot" if cfg.get("src") == "bidir" else "没有（非双向固件）"
    low, fade = _number(cfg, "min_hz"), _number(cfg, "fade_hz")
    details = [
        f"极对数 {cfg.get('pp', '?')} · 谐波 {harmonics_text(_number(cfg, 'harm'))} · "
        f"Q {_number(cfg, 'q_x100') / 100:.2f} · {low}–{low + fade} Hz 淡入 · "
        f"陀螺 {_number(cfg, 'fs_x10') / 10:.1f} Hz（名义 {cfg.get('fs_nom', '?')}）· "
        f"转速来源 {source} · 工作中的陷波 {cfg.get('active', '0')} 个"]
    base = base_harmonic(_number(cfg, "harm", 1))
    weight = "权重" if base == 1 else f"{base}x 权重"
    motors = []
    for channel in (1, 2):
        age = _number(esc, f"ch{channel}_age_ms", 9999)
        heard = "从未收到回包" if age >= 9999 else f"回包 {age} ms 前"
        motors.append(f"电调{channel}：{_number(esc, f'ch{channel}_hz_x10') / 10:.1f} Hz · "
                      f"{weight} {_number(esc, f'ch{channel}_w_x100')}% · {heard}")
    details.append("；".join(motors))
    details.append(
        f"计数：过期 {count.get('stale', '0')} · 丢一样 {count.get('gap1', '0')} · "
        f"复位 {count.get('reset', '0')} · 非有限 {count.get('nonfinite', '0')} · "
        f"限速 {count.get('slewclamp', '0')} · 拒收 {count.get('reject', '0')} · "
        f"看门狗 {count.get('wdog', '0')}")
    spin = _number(timing, "spin")
    tracked = (f"{100.0 * _number(timing, 'tracked') / spin:.1f}%" if spin else "—")
    details.append(
        f"转动时满权重跟踪 {tracked}（{spin} 个样本）· 每样本耗时 平均 "
        f"{_number(timing, 'apply_us_avg_x100') / 100:.2f} µs / 最大 {timing.get('apply_us_max', '0')} µs · "
        f"每拍 平均 {_number(timing, 'tick_us_avg_x100') / 100:.2f} µs / 最大 "
        f"{timing.get('tick_us_max', '0')} µs")
    return head, "\n".join(details)


class NotchPanel:
    def _init_notch_vars(self) -> None:
        self.notch_status_var = tk.StringVar(value="陷波（桨叶振动）：状态未读取")
        self.notch_detail_var = tk.StringVar(value="")
        self.notch_block_var = tk.StringVar(value="")
        self.notch_reply_var = tk.StringVar(value="")
        self.notch_lines: dict[str, dict] = {}
        self._notch_generation = None      # 已自动读过状态的连接代次
        self._notch_unsupported = None     # 固件不认 RPMNOTCH 的连接代次
        self._notch_refresh_pending = False
        self._notch_action = None          # 等 ON/OFF 的回复：{"want": "1"/"0", "deadline", "seen"}

    def _build_notch(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="陷波（桨叶振动）", padding=8)
        box.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(box, style="Muted.TLabel", wraplength=620, justify=tk.LEFT, text=(
            "按电调转速把约 107 Hz 的桨叶振动从控制用陀螺里挖掉：只影响 RATE/ANGLE 闭环和飞行控制，"
            "记录的陀螺仍是原始值。需要双向 DShot 固件；只存在飞控内存里，重新上电回到关。"
            "做对比时关、开、关、开交替各跑一轮。")).pack(anchor=tk.W)
        ttk.Label(box, textvariable=self.notch_status_var, wraplength=640,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))
        ttk.Label(box, textvariable=self.notch_detail_var, style="Mono.TLabel", wraplength=660,
                  justify=tk.LEFT).pack(anchor=tk.W)
        row = ttk.Frame(box)
        row.pack(fill=tk.X, pady=(6, 0))
        self.notch_read_button = ttk.Button(row, text="读取状态", command=self.notch_refresh)
        self.notch_read_button.pack(side=tk.LEFT)
        self.notch_on_button = ttk.Button(row, text="打开陷波（ON）",
                                          command=lambda: self.notch_switch(True))
        self.notch_on_button.pack(side=tk.LEFT, padx=(6, 0))
        self.notch_off_button = ttk.Button(row, text="关闭陷波（OFF）",
                                           command=lambda: self.notch_switch(False))
        self.notch_off_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(box, textvariable=self.notch_block_var, style="Muted.TLabel", wraplength=620,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(box, textvariable=self.notch_reply_var, wraplength=640,
                  justify=tk.LEFT).pack(anchor=tk.W)
        self.refresh_notch_controls()

    # ------------------------------------------------------------ 能不能点

    def notch_block_reason(self) -> str:
        """开/关此刻不能点的原因（中文）；空串 = 可以点。"""
        w = self.workflow
        transport = getattr(self.panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            return "未连接飞控。"
        if self._notch_unsupported == w.generation():
            return UNSUPPORTED
        if (w.run_id is not None and w.end is None) or w.awaiting == "SYSID START" or (
                w.kind == "start" and (w.awaiting or w.pending)):
            return ("辨识正在进行（或正在开始）：这一轮用的陷波配置不能中途改。"
                    "结束并用遥控器上锁后再切换。")
        if w.awaiting:
            return "正在等待飞控回显，稍后再切换。"
        thr = self._current_thr()
        if thr is not None and str(thr.get("armed")) == "1":
            return "飞控已解锁：先用遥控器上锁再切换（解锁时飞控也会拒绝）。"
        return ""

    def refresh_notch_controls(self) -> None:
        if not hasattr(self, "notch_on_button"):
            return
        reason = self.notch_block_reason()
        for button in (self.notch_on_button, self.notch_off_button):
            button.configure(state="disabled" if reason else "normal")
        transport = getattr(self.panel, "transport", None)
        connected = bool(transport is not None and getattr(transport, "is_connected", False))
        self.notch_read_button.configure(state="normal" if connected else "disabled")
        if not reason and connected and self._current_thr() is None:
            reason = "解锁状态暂时未知；飞控解锁时会自己拒绝切换。"
        self.notch_block_var.set(reason)

    # ------------------------------------------------------------ 发命令

    def notch_refresh(self, quiet: bool = False) -> bool:
        sent = self.send(NOTCH_QUERY, quiet=quiet)
        if sent and not quiet:
            self.notch_reply_var.set("已请求陷波状态，等待飞控回复…")
        return sent

    def notch_switch(self, on: bool) -> bool:
        reason = self.notch_block_reason()
        if reason:
            self.notch_reply_var.set(reason)
            return False
        command = "RPMNOTCH ON" if on else "RPMNOTCH OFF"
        if not self.send(command):
            self.notch_reply_var.set(f"{command} 没发出去：{self.status_var.get()}")
            return False
        self._notch_action = {"want": "1" if on else "0", "deadline": _now() + NOTCH_REPLY_TIMEOUT_S,
                              "seen": None}
        self.notch_reply_var.set(f"已发送 {command}，等待飞控回复…")
        return True

    def notch_expire(self) -> None:
        """开/关发出后 3 秒还没等到 en 对得上的回复：说清楚并作废。轮询每秒调一次。"""
        action = self._notch_action
        if action is None or _now() <= action["deadline"]:
            return
        self._notch_action = None
        command = "RPMNOTCH ON" if action["want"] == "1" else "RPMNOTCH OFF"
        if action["seen"] is not None:
            en, state = action["seen"]
            self.notch_reply_var.set(f"飞控回复与请求不符：{command} 之后读到的是 en={en}（状态 {state}）。"
                                     "点「读取状态」再确认一次。")
        else:
            self.notch_reply_var.set(f"{NOTCH_REPLY_TIMEOUT_S:.0f} 秒内没等到飞控对 {command} 的回复"
                                     "（链路可能丢了一行）。点「读取状态」看实际状态。")

    def notch_poll_idle(self) -> bool:
        """空闲时自动读一次状态：新连接后第一次、每轮结束后一次。辨识进行中不打扰。"""
        self.notch_expire()
        w = self.workflow
        generation = w.generation()
        if self._notch_generation == generation and not self._notch_refresh_pending:
            return False
        if w.closed or w.awaiting or (w.run_id is not None and w.end is None):
            return False
        if self._notch_unsupported == generation or not self._visible_and_connected():
            return False
        if not self.notch_refresh(quiet=True):
            return False
        self._notch_generation = generation
        self._notch_refresh_pending = False
        return True

    # ------------------------------------------------------------ 收行

    def notch_handle_line(self, text: str) -> bool:
        """返回 True = 这一行是陷波自己的回复，不再交给辨识工作流（它不属于任何辨识事务）。"""
        if text.startswith("SYSID end "):
            self._notch_refresh_pending = True      # 这一轮的计数值得看一眼
            return False
        if text.startswith("ERR unknown cmd RPMNOTCH"):
            # 旧固件：不能让这行 ERR 打断正在等回显的辨识事务。
            self._notch_unsupported = self.workflow.generation()
            self._notch_action = None
            self.notch_status_var.set(UNSUPPORTED)
            self.notch_detail_var.set("")
            self.notch_reply_var.set("")
            self.refresh_notch_controls()
            return True
        if not text.startswith("RPMNOTCH "):
            return False
        values = parse_kv(text)
        if text.startswith("RPMNOTCH event=rejected"):
            self._notch_action = None
            self.notch_reply_var.set(f"{REJECT_TEXT.get(values.get('reason', ''), '飞控拒绝。')}"
                                     f"（飞控原文：{text}）")
            return True
        if text.startswith("RPMNOTCH event=cleared"):
            self.notch_reply_var.set("飞控已清零陷波计数。")
            return True
        kind = text.split()[1]
        if kind not in BLOCK_KINDS:
            return True
        if kind == "cfg":
            self.notch_lines = {}
            self._note_notch_reply(values)
        self.notch_lines[kind] = values
        if kind == BLOCK_KINDS[-1]:
            head, detail = describe_block(self.notch_lines)
            self.notch_status_var.set(head)
            self.notch_detail_var.set(detail)
        return True

    def _note_notch_reply(self, cfg: dict) -> None:
        self.notch_expire()                  # 过期的开关先了结，不拿它配这一块
        action = self._notch_action
        if action is None:
            if self.notch_reply_var.get().startswith("已请求陷波状态"):
                self.notch_reply_var.set("")
            return
        state = STATE_SHORT.get(cfg.get("state", ""), cfg.get("state", "?"))
        if cfg.get("en") != action["want"]:
            # 多半是开关之前那次查询的回复先到了：记下，接着等真正的回复（到期还没有才算不符）。
            action["seen"] = (cfg.get("en"), state)
            return
        self._notch_action = None
        if action["want"] == "1":
            self.notch_reply_var.set(f"飞控已确认：陷波已打开（状态 {state}）。")
        else:
            self.notch_reply_var.set("飞控已确认：陷波已关闭。")


__all__ = ["NotchPanel", "describe_block", "notch_provenance_text"]
