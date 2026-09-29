"""「系统辨识 · Z 高度」页 —— 占位；高度辨识的采集已在「内环」页的 ALT 模式（槽式台架）实现。

高度**必须单独辨**，不能套用水平那一套。理由都是物理上的：

* 水平通道靠**倾转**产生力，量级受限于 `F·sin(tilt)`，而且是小角度线性化的；
  高度通道靠**总推力直接**产生力，是强非线性的（推力大致正比于转速平方）。
* 水平方向没有恒定偏置；垂直方向永远挂着一个 `m·g`，工作点就在悬停推力附近，
  而那一点的增益 dF/d油门 与低油门处差好几倍。
* 执行器完全不同：水平是舵机（延迟 16~41 ms 量级），高度是电调 + 桨的转速动态
  （一阶惯性，时间常数在几十到上百毫秒，而且上升/下降不对称）。
* 电池电压掉下来时推力跟着掉，高度环会直接感受到；水平环基本不受影响。

所以把两者当成同一个模型去辨，辨出来的东西两边都不对。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .common import not_implemented_frame


class SysIdAltitudePage:
    def __init__(self, panel, parent: ttk.Frame) -> None:
        self.panel = panel
        not_implemented_frame(
            parent,
            "Z 高度辨识",
            why=(
                "高度通道的物理特性和水平不一样，必须单独辨：力来自**总推力**"
                "而不是倾转，工作点固定在悬停推力附近，执行器是电调+桨的转速动态"
                "（上升/下降不对称），而且电池掉压会直接改变增益。"
                "把它和水平当成同一个模型，两边都会辨错。\n\n"
                "高度辨识已可在「角速度 / 角度内环」页的「本轮做」选 ALT 模式，在槽式台架"
                "（杆的轴装在竖直槽里，机体连杆一起上下移动）上采集，分析离线进行；"
                "本页本身仍不做采集或拟合。"),
            needs=[
                ("推力台的 `推力 = f(转速, 电压)` 标定（「推力台」程序，P4）："
                 "2026-09-22 起已有实机整批模型，但带载电压只覆盖约 11.7～12.4 V，"
                 "升降速动态尚未拟合"),
                ("双向 DShot 的 eRPM 回传：`DSHOT300_BIDIR` 档（bitbang 后端）"
                 "2026-09-21 已实机打通；默认构建仍是单向档，要用须烧双向档"),
                "测高链路（气压/测距）的噪声与延迟已知",
            ])
        ttk.Label(parent, wraplength=660, justify=tk.LEFT, text=(
            "内环辨识给出的惯量与延迟对这一环没有直接用处——垂直方向上"
            "对应的是质量与推力动态，不是转动惯量。")).pack(anchor=tk.W, pady=(4, 0))


def mount_sysid_altitude(panel, parent: ttk.Frame) -> SysIdAltitudePage:
    page = SysIdAltitudePage(panel, parent)
    panel.sysid_altitude_page = page
    return page
