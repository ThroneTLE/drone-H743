"""「系统辨识 · XY 速度/位置环」页 —— 占位，本次不实现。

留空而不是放几个点了没反应的按钮：后者会让人以为是飞控出了问题，
然后去查一条根本不存在的链路。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .common import not_implemented_frame


class SysIdHorizontalPage:
    def __init__(self, panel, parent: ttk.Frame) -> None:
        self.panel = panel
        not_implemented_frame(
            parent,
            "XY 速度 / 位置环辨识",
            why=(
                "外环辨的是「机体倾角 -> 水平加速度 -> 速度 -> 位置」这条链，"
                "而本台架是一根穿过质心的光杆：机体只能绕杆转，**动不了位置**。"
                "所以这一环在当前台架上物理上就没法辨——它需要自由飞行或一套"
                "能让机体平移的台架。\n\n"
                "内环那一页辨出来的惯量、延迟与增益是这一环的输入，不是替代品："
                "外环的带宽必须显著低于内环，而内环带宽由内环辨出来的延迟决定。"),
            needs=[
                "能平移的台架，或可控的自由飞行场地",
                "光流/测距的速度与高度估计已经标定（「传感器 · 光流」页）",
                "内环辨识已收敛（惯量比值接近 1）并写入了验证过的增益",
            ])
        ttk.Label(parent, wraplength=660, justify=tk.LEFT, text=(
            "XY 两轴在本机上是耦合的，且结构近似对称，所以真做起来仍然只需要辨"
            "一组参数，和内环一样。")).pack(anchor=tk.W, pady=(4, 0))


def mount_sysid_horizontal(panel, parent: ttk.Frame) -> SysIdHorizontalPage:
    page = SysIdHorizontalPage(panel, parent)
    panel.sysid_horizontal_page = page
    return page
