"""Panel-local state persistence and best-effort log helpers."""

from __future__ import annotations

import json
import traceback
import tkinter as tk
from datetime import datetime
from pathlib import Path

from .transport import serial_port_fingerprint

try:
    from ..project_paths import LOG_DIR, PANEL_STATE_PATH, ensure_directory
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.project_paths import LOG_DIR, PANEL_STATE_PATH, ensure_directory
    except ImportError:
        from project_paths import LOG_DIR, PANEL_STATE_PATH, ensure_directory


PANEL_CRASH_LOG = LOG_DIR / "panel_crash.log"
RC_WIZARD_TRACE_LOG = LOG_DIR / "rc_wizard.log"


class PanelStateMixin:
    def destroy(self) -> None:
        from .tk_lifecycle import release_variables
        interpreter = self.tk
        super().destroy()
        release_variables(interpreter)

    def _load_panel_state(self) -> dict[str, object]:
        try:
            raw = PANEL_STATE_PATH.read_text(encoding="utf-8")
        except (OSError, ValueError):
            return {}
        try:
            state = json.loads(raw)
        except json.JSONDecodeError:
            return {}
        return state if isinstance(state, dict) else {}

    def _save_panel_state(self) -> None:
        """记下这次连的是什么，供下次启动自动重连。写失败不能影响正在跑的连接。"""
        state = dict(self._panel_state)
        state["transport"] = self.transport_var.get()
        state["auto_connect"] = bool(self.auto_connect_var.get())
        if self.transport is self.serial_transport:
            device = self.serial_transport.active_port or ""
            if device:
                state["serial_port"] = device
                state["serial_fingerprint"] = serial_port_fingerprint(
                    self._serial_port_identity.get(device)
                )
                try:
                    state["serial_baud"] = int(self.serial_baud_var.get())
                except (tk.TclError, ValueError):
                    pass
        elif self.transport is self.tcp_transport:
            state["tcp_host"] = self.host_var.get()
            state["tcp_port"] = self.port_var.get()
        self._panel_state = state
        try:
            ensure_directory(PANEL_STATE_PATH.parent)
            PANEL_STATE_PATH.write_text(
                json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
        except OSError:
            pass


def append_log(path: Path, text: str) -> None:
    """尽力写日志。日志本身失败绝不能再把程序带下去。"""
    try:
        ensure_directory(path.parent)
        stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        with path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(f"[{stamp}] {text.rstrip()}\n")
    except OSError:
        pass


def record_panel_crash(exc_type, exc_value, exc_tb) -> str:
    """把异常写进日志并返回摘要。

    在这之前，回调里的异常只会打到 stderr——从资源管理器或 IDE 启动时根本没有
    stderr，用户看到的就是"窗口突然没了"，而且校准做到一半的进度全丢，没有任何
    可查的线索。
    """
    text = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
    append_log(PANEL_CRASH_LOG, "PANEL EXCEPTION\n" + text)
    lines = [line for line in text.strip().splitlines() if line.strip()]
    return "\n".join(lines[-4:]) if lines else repr(exc_value)


__all__ = [
    "LOG_DIR",
    "PANEL_CRASH_LOG",
    "PANEL_STATE_PATH",
    "PanelStateMixin",
    "RC_WIZARD_TRACE_LOG",
    "append_log",
    "record_panel_crash",
]
