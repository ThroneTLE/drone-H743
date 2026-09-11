"""仿真独立栏目：启动器 + 「仿真参数只在仿真运行时开放」的说明与闸门。

原来这条仿真启动栏常驻主窗口最上方。那个位置有两个问题：

  1. 它占着**每次上机都要先看一眼**的位置，而真正该放那儿的是解锁状态；
  2. 仿真的调参滑块绑的是 `sim_*` 通道，它们映射到真实的 `coax.*` 参数
     （tools/sim_xz/control_catalog.py 第 6 列）。仿真工作区一旦留在仪表盘布局里，
     接上真机之后拖一下滑块就是往飞机上写参数。

所以：启动器搬进本栏目，仿真专用的工作区改成**永不落盘**（Workspace.ephemeral），
只在仿真运行期间存在，停止即移除。本页把这条规则写出来并显示当前闸门状态，
免得它变成一条只有读代码才知道的约定。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from ..simulation_launcher import SimulationBar

try:
    from ...sim_xz.control_catalog import GAIN_CHANNELS
except ImportError:  # 直接跑 tools/drone_tcp_panel.py 时没有包上下文
    try:
        from tools.sim_xz.control_catalog import GAIN_CHANNELS
    except ImportError:
        from sim_xz.control_catalog import GAIN_CHANNELS


REFRESH_MS = 500


class SimulationPage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent, padding=0)
        self.panel = panel
        self.gate_var = tk.StringVar(value="仿真未运行 · 仿真调参工作区已关闭")
        self._timer = None
        self._disposed = False

        ttk.Label(self, text="SIMULATION  /  本机仿真",
                  style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(self, text="X–Z 平面被控对象与真实 C 级联控制律",
                  style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            self,
            text=(
                "仿真在本机跑，不需要飞控硬件：上位机连到 127.0.0.1，仿真进程扮演飞控。"
                "被控对象用的是真实的 C 控制律（tools/sim_xz/sim_controller_bridge.c 直接"
                "编译 Driver/Src/drv_coax_ctrl.c），所以调出来的增益对真机是有参考价值的；"
                "但机体模型用的是仿真自己那一组参考数值，不是你飞机 Flash 里的那份。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))

        self.bar = SimulationBar(self, panel)
        self.bar.pack(fill=tk.X)
        panel.simulation_bar = self.bar

        gate = ttk.LabelFrame(self, text="仿真参数闸门", padding=10)
        gate.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(gate, textvariable=self.gate_var,
                  style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            gate,
            text=(
                "仿真调参滑块绑的是 sim_* 通道，它们一一映射到真实的 coax.* 参数。"
                "为了不污染真机调参，这些工作区只在仿真运行期间存在，停止即移除，"
                "而且**永不写入面板布局文件**——所以它们不可能在下次接真机时冒出来。"
                "想调真机参数请用「参数 / PID」页或仪表盘里你自己建的工作区。"
            ),
            style="Muted.TLabel", wraplength=1080,
        ).pack(anchor=tk.W, pady=(4, 8))

        listing = ttk.Frame(gate)
        listing.pack(fill=tk.X)
        ttk.Label(listing, text="受闸门管辖的通道（仿真名 → 飞控参数）：",
                  style="Muted.TLabel").grid(row=0, column=0, columnspan=4, sticky=tk.W)
        for index, channel in enumerate(GAIN_CHANNELS):
            ttk.Label(listing, text=f"{channel[0]} → {channel[5]}",
                      style="Muted.TLabel").grid(
                row=1 + index // 3, column=index % 3, sticky=tk.W, padx=(0, 18))

        panel.bind("<Destroy>", self._on_destroy, add="+")
        self._tick()

    def _tick(self) -> None:
        if self._disposed:
            return
        try:
            running = self.bar.process is not None
            installed = bool(self.bar.workspaces)
            if running and installed:
                self.gate_var.set("仿真运行中 · 仿真调参工作区已开放（仪表盘页）")
            elif running:
                self.gate_var.set("仿真启动中 · 工作区尚未装入")
            else:
                self.gate_var.set("仿真未运行 · 仿真调参工作区已关闭")
        finally:
            self._timer = self.panel.after(REFRESH_MS, self._tick)

    def _on_destroy(self, event) -> None:
        if event.widget is self.panel:
            self._disposed = True
            if self._timer is not None:
                try:
                    self.panel.after_cancel(self._timer)
                except tk.TclError:
                    pass
                self._timer = None


def mount_simulation(panel, parent) -> SimulationPage:
    page = SimulationPage(parent, panel)
    page.pack(fill=tk.BOTH, expand=True)
    panel.simulation_page = page
    return page
