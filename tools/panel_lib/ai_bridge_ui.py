"""AI 接口的界面与接线：连接区下面一行小标签 + 开关（上锁时 AI 可自动进 DFU 烧录，不再要作者许可）。

接线点是 `shell.build_ui` 里的 `mount_ai_bridge(self, root)`，
所以 `drone_tcp_panel.py` 一行都不用加。
"""

import os
import tkinter as tk
from tkinter import ttk

from .ai_bridge import AiBridge
from .board_line_hooks import register_board_line_hook

PUMP_PERIOD_MS = 40


def _bridge_disabled_by_environment() -> bool:
    """测试进程不许起服务：否则会覆盖作者正在用的地面站写在 %TEMP% 里的端口/token。"""
    flag = os.environ.get("DRONE_PANEL_AI_BRIDGE", "")
    if flag == "force":
        return False
    return flag == "0" or bool(os.environ.get("PYTEST_CURRENT_TEST"))


def mount_ai_bridge(panel, parent):
    panel.ai_bridge = None
    if _bridge_disabled_by_environment():
        return None
    bridge = AiBridge(panel)
    panel.ai_bridge = bridge
    enabled_var = tk.BooleanVar(value=True)
    text_var = tk.StringVar(value="AI 接口：启动中…")
    panel.ai_bridge_enabled_var = enabled_var

    row = ttk.Frame(parent)
    row.pack(fill=tk.X, pady=(2, 0))
    ttk.Label(row, textvariable=text_var, style="Muted.TLabel").pack(side=tk.LEFT)
    ttk.Checkbutton(row, text="开启", variable=enabled_var,
                    command=lambda: bridge.set_enabled(enabled_var.get())).pack(side=tk.LEFT, padx=(10, 2))

    def refresh():
        if not bridge.port:
            return
        state = "已开启" if bridge.enabled else "已关闭"
        text_var.set(f"AI 接口：127.0.0.1:{bridge.port} {state}（上锁时可自动进 DFU 烧录）")

    bridge.on_state_change = refresh
    register_board_line_hook(panel, bridge.on_board_line)

    try:
        bridge.start()
    except OSError as exc:
        text_var.set(f"AI 接口：未启动（{exc}）")
        return bridge
    refresh()

    state = {"alive": True}

    def pump():
        if not state["alive"]:
            return
        bridge.pump()
        panel.after(PUMP_PERIOD_MS, pump)

    def on_destroy(event):
        if event.widget is panel and state["alive"]:
            state["alive"] = False
            bridge.stop()

    panel.bind("<Destroy>", on_destroy, add="+")
    panel.after(PUMP_PERIOD_MS, pump)
    return bridge
