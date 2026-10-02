"""「IMU 重新标定」按钮：发 `IMUZERO`，约 3.5 s 后回读 `IMUZERO?`（固件见 App/Src/app_cmd_imuzero.c）。

作者 2026-10-01："你给我上位机来一个IMUZERO的按钮"。台架上每次烧录舵机一动机体就绕杆摆，
上电零偏把摆动当成零偏；扶稳后点一下，按上电同一规则重新采陀螺零偏与姿态零点。
固件只在上锁时接受（解锁回 armed_blocked）。三个辨识子页各放一个，共用同一行状态。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from ...proto import parse_kv

IMUZERO_BUTTON_TEXT = "IMU 重新标定"
#: 固件要求静止 3 s；多等半秒再回读，没就绪再读两次（机体还在晃时零点会晚一点）。
IMUZERO_READBACK_MS = (3500, 6000, 9000)


class ImuZeroPanel:
    def _init_imuzero_vars(self) -> None:
        self.imuzero_var = tk.StringVar(value="")
        self._imuzero_pending = 0          # 还剩几次回读

    def build_imuzero_button(self, row: ttk.Frame) -> ttk.Button:
        """放在开始/停止那一排的右边；状态文字跟在按钮后面。"""
        button = ttk.Button(row, text=IMUZERO_BUTTON_TEXT, command=self.imuzero_request)
        button.pack(side=tk.LEFT, padx=(16, 0))
        ttk.Label(row, textvariable=self.imuzero_var, style="Muted.TLabel").pack(side=tk.LEFT, padx=(8, 0))
        return button

    def imuzero_request(self) -> None:
        if not self.send("IMUZERO"):
            return
        self.imuzero_var.set("已请求：扶稳机体、保持静止 3 秒…")
        self._imuzero_pending = len(IMUZERO_READBACK_MS)
        after = getattr(self.panel, "after", None)
        if callable(after):
            for delay in IMUZERO_READBACK_MS:
                after(delay, self._imuzero_readback)

    def _imuzero_readback(self) -> None:
        if self._imuzero_pending > 0:
            self.send("IMUZERO?", quiet=True)

    def imuzero_handle_line(self, text: str) -> bool:
        if text.startswith("IMUZERO state=armed_blocked"):
            self._imuzero_pending = 0
            self.imuzero_var.set("飞机已解锁：先上锁再标定")
            return True
        if text.startswith("IMUZERO state=restarted"):
            return True
        if not text.startswith("IMUZERO gyro_bias="):
            return False
        values = parse_kv(text)
        ready = values.get("gyro_bias") == "1" and values.get("attitude_zero") == "1"
        try:
            ferr = f"{int(values.get('ferr_cdeg', '-1')) / 100:.2f}°"
        except ValueError:
            ferr = "?"
        if ready:
            self._imuzero_pending = 0
            self.imuzero_var.set(f"标定完成：陀螺零偏、姿态零点已就绪（融合误差 {ferr}）")
        else:
            self._imuzero_pending = max(0, self._imuzero_pending - 1)
            self.imuzero_var.set("还没就绪：机体可能还在晃，扶稳后再点一次" if self._imuzero_pending == 0
                                 else f"标定中…（融合误差 {ferr}）")
        return True


__all__ = ["IMUZERO_BUTTON_TEXT", "IMUZERO_READBACK_MS", "ImuZeroPanel"]
