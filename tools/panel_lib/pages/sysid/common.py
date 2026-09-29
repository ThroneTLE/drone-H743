"""系统辨识三个子页共用的东西：出口、状态条、物理条件编辑器。

**出口必须过面板的安全门。** 直接 `transport.send_line` 会在验收会话和固件升级
窗口里照样把命令发出去——这条口子在 LEDMAP 页上真出过（见 `pages/led_map.py`
的 `send()` 注释），这里不再犯。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk


class SysIdPageBase:
    """出口 + 状态条。三个子页都继承它。"""

    def __init__(self, panel, parent: ttk.Frame) -> None:
        self.panel = panel
        self.parent = parent
        self.status_var = tk.StringVar(value="未连接")

    # ------------------------------------------------------------ 出口

    def send(self, text: str, *, quiet: bool = False) -> bool:
        """`quiet=True` 给后台轮询用：不写日志、不改状态行，失败就安静地不发。"""
        panel = self.panel
        workflow = getattr(self, "workflow", None)
        if workflow and workflow.awaiting and text != workflow.awaiting and text != "SYSID STOP":
            if not quiet:
                self.status_var.set("正在核对上一条回显，请等待或停止")
            return False
        transport = getattr(panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            if not quiet:
                self.status_var.set("未连接飞控，命令未发送")
            return False
        allowed = getattr(panel, "_validation_command_allowed", None)
        if callable(allowed) and not allowed(text):
            if not quiet:
                self.status_var.set("面板挡下了这条命令：验收会话或固件升级正在进行。")
            return False
        append = getattr(panel, "_append", None)
        if callable(append) and not quiet:
            append(f"> {text}")
        sent = bool(transport.send_line(text))
        if sent and workflow is not None and hasattr(workflow, "note_sent"):
            workflow.note_sent(text)
        return sent

    def mount_status(self, parent: ttk.Frame) -> ttk.Label:
        label = ttk.Label(parent, textvariable=self.status_var, wraplength=560,
                          style="Mono.TLabel")
        label.pack(fill=tk.X, pady=(6, 0))
        return label


def labelled_entry(parent: ttk.Frame, row: int, text: str, variable: tk.Variable,
                   *, width: int = 10, hint: str = "",
                   hint_var: tk.Variable | None = None,
                   label_var: tk.Variable | None = None) -> ttk.Entry:
    """一行"标签 + 输入框 + 灰字提示"。`label_var` 给了就用它当标签（随设置变的标签）。"""
    label = {"textvariable": label_var} if label_var is not None else {"text": text}
    ttk.Label(parent, **label).grid(row=row, column=0, sticky=tk.W, pady=2)
    entry = ttk.Entry(parent, textvariable=variable, width=width)
    entry.grid(row=row, column=1, sticky=tk.W, padx=(6, 8), pady=2)
    if hint or hint_var is not None:
        options = {"textvariable": hint_var} if hint_var is not None else {"text": hint}
        ttk.Label(parent, style="Muted.TLabel", wraplength=420, justify=tk.LEFT,
                  **options).grid(row=row, column=2, sticky=tk.W, pady=2)
    return entry


def not_implemented_frame(parent: ttk.Frame, title: str, why: str,
                          needs: list[str]) -> ttk.Frame:
    """未实现的子页长什么样。

    **不放假按钮。** 一个点了没反应的「开始辨识」比一句「还没做」危险得多：
    前者会让人以为是飞控的问题，然后去查一条根本不存在的链路。
    这里明说缺什么、为什么要单独做，以及做之前需要先有什么。
    """
    box = ttk.LabelFrame(parent, text=title, padding=10)
    box.pack(fill=tk.X, pady=(0, 10))
    ttk.Label(box, text="本页尚未实现。", style="Mono.TLabel").pack(anchor=tk.W)
    ttk.Label(box, text=why, wraplength=620, justify=tk.LEFT).pack(
        anchor=tk.W, pady=(6, 6))
    if needs:
        ttk.Label(box, text="做之前需要先有：").pack(anchor=tk.W)
        for item in needs:
            ttk.Label(box, text=f"· {item}", wraplength=600,
                      justify=tk.LEFT).pack(anchor=tk.W, padx=(12, 0))
    return box
