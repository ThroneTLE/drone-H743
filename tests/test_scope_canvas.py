"""`panel_lib/scope.py` 的 `ScopeCanvas` 与 min/max 抽稀。

从 `tests/test_scope_page.py` 迁过来（R-T1-5 让 `pages/scope.py` 退役，但
**画布本身继续服役**——工作台的波形组件用的就是它）。页面级行为迁到
`tests/test_dashboard_page.py`。
"""

from __future__ import annotations

import time
import tkinter as tk

import numpy as np
import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib import scope as scope_widget


@pytest.fixture(scope="module")
def _panel():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:  # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        yield instance
    finally:
        instance.destroy()


def test_min_max_decimation_keeps_the_extremes() -> None:
    """等间隔取样会把毛刺整个漏掉，而毛刺往往正是要看的东西。"""
    times = np.arange(10000, dtype=np.float64) * 0.001
    values = np.zeros(10000, dtype=np.float32)
    values[4321] = 7.5
    values[8765] = -3.25

    out_t, out_v = scope_widget.decimate_min_max(times, values, 400)
    assert out_v.max() == pytest.approx(7.5)
    assert out_v.min() == pytest.approx(-3.25)
    assert out_t.size == out_v.size
    # 抽稀后的点数只跟列数有关，与 10 k 的原始点数无关。
    assert out_v.size <= 400 * 2 + 3


def test_short_series_are_not_decimated() -> None:
    times = np.arange(50, dtype=np.float64)
    values = np.arange(50, dtype=np.float32)
    out_t, out_v = scope_widget.decimate_min_max(times, values, 400)
    assert out_v.tolist() == values.tolist()
    assert out_t.tolist() == times.tolist()


def test_decimation_is_safe_on_empty_input() -> None:
    empty_t = np.empty(0, dtype=np.float64)
    empty_v = np.empty(0, dtype=np.float32)
    out_t, out_v = scope_widget.decimate_min_max(empty_t, empty_v, 400)
    assert out_t.size == 0 and out_v.size == 0


def test_redraw_benchmark_eight_curves_ten_thousand_points(_panel) -> None:
    """8 曲线 × 10 k 点，单次重绘 ≤5 ms（R-T1-3 判据，画布仍要守住）。"""
    canvas = scope_widget.ScopeCanvas(_panel, width=900, height=320)
    canvas.canvas.pack()
    canvas.set_curves([(index, f"ch{index}") for index in range(8)])
    canvas.set_window(60.0)
    times = np.linspace(0.0, 60.0, 10000, dtype=np.float64)
    series = {
        index: (times, np.sin(times * (index + 1)).astype(np.float32))
        for index in range(8)
    }

    canvas.render(series)          # 预热：第一次会建 grid item
    _panel.update_idletasks()

    samples = []
    for _ in range(15):
        start = time.perf_counter()
        canvas.render(series)
        samples.append(time.perf_counter() - start)
    samples.sort()
    median = samples[len(samples) // 2]
    # 判据卡在中位数而不是最好一次（最好一次可以靠运气），也不卡最坏一次
    # （那条尾巴是 GC 和操作系统调度，不是这段代码）。
    assert median <= 0.005, (
        f"median redraw {median * 1000:.2f} ms "
        f"(best {samples[0] * 1000:.2f}, worst {samples[-1] * 1000:.2f})"
    )
    # 抽稀之后送进 Tcl 的点数必须只跟画布宽度有关，与缓冲里的点数无关。
    left, _top, right, _bottom = canvas.plot_area()
    assert canvas.curves[0].points <= min(
        right - left, scope_widget.SCOPE_MAX_COLUMNS
    ) * 2 + 3
    canvas.canvas.destroy()
