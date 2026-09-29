"""「系统辨识」页组：角速度/角度内环、XY 速度/位置环、Z 高度。

分成三个子页而不是一页，是因为这三件事的**物理特性不一样**，共用一套参数和一条
拟合流程只会互相污染：内环靠倾转产生力矩、外环靠倾角产生水平加速度、高度靠总推
力直接产生升力且强非线性。本次只有内环是完整实现的，另外两页明说未实现。
"""

from __future__ import annotations

from tkinter import ttk

from ...viewport import VerticalScrolledFrame
from .altitude import mount_sysid_altitude
from .horizontal import mount_sysid_horizontal
from .inner_loop import mount_sysid_inner_loop

SYSID_TAB_TEXT = "系统辨识"
SYSID_INNER_TAB_TEXT = "角速度 / 角度内环"
SYSID_HORIZONTAL_TAB_TEXT = "XY 速度 / 位置环"
SYSID_ALTITUDE_TAB_TEXT = "Z 高度"

__all__ = [
    "SYSID_TAB_TEXT", "SYSID_INNER_TAB_TEXT", "SYSID_HORIZONTAL_TAB_TEXT",
    "SYSID_ALTITUDE_TAB_TEXT", "mount_sysid",
]


def mount_sysid(panel, parent: ttk.Frame) -> ttk.Notebook:
    notebook = ttk.Notebook(parent)
    notebook.pack(fill="both", expand=True)

    inner_scroll = VerticalScrolledFrame(notebook)
    horizontal_scroll = VerticalScrolledFrame(notebook)
    altitude_scroll = VerticalScrolledFrame(notebook)

    notebook.add(inner_scroll, text=SYSID_INNER_TAB_TEXT)
    notebook.add(horizontal_scroll, text=SYSID_HORIZONTAL_TAB_TEXT)
    notebook.add(altitude_scroll, text=SYSID_ALTITUDE_TAB_TEXT)

    mount_sysid_inner_loop(panel, inner_scroll.content)
    mount_sysid_horizontal(panel, horizontal_scroll.content)
    mount_sysid_altitude(panel, altitude_scroll.content)

    panel.sysid_notebook = notebook
    return notebook
