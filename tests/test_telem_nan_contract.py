"""NaN 语义：固件对"无效 / 过期"发 NaN，不发 0。主机三层都不许把它变成 0。

R-PWR-1 给 `batt_v` / `batt_i` 定的就是这条：**没读到不是 0 V / 0 A**。0 在电源
这件事上格外危险——0 A 看起来像"电机没转"，0 V 看起来像"电池拔了"，两个都是
会让人做出错误判断的假事实。

三层各自要证明的事：

1. 解码器（`telem_stream.TelemDecoder`）—— 帧不能因为值是 NaN 就被判废或污染
   拒帧统计；长度/版本/掩码/指纹四条校验本来就不看值，这里把它钉住。
2. 工作台组件（`dashboard/tiles.py`、`scope.py`）—— 渲染不崩，显示"无数据"，
   波形上不能出现一条掉到 0 的线。
3. 电源页 —— 显示 `—` 并给出原因（在 `tests/test_power_page.py` 里）。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tools.panel_lib.dashboard import layout as dash_layout
from tools.panel_lib.dashboard import tiles as dash_tiles
from tools.panel_lib.scope import ScopeCanvas
from tools.panel_lib.telem_stream import TelemDecoder, TelemRing, TelemSchema

# 复用电源页那份装置：同一张带 `batt_v` / `batt_i` 的通道表、同一个真实面板夹具。
# `app` / `live` 是 pytest fixture，必须进本模块命名空间才能被用例参数解析到。
from test_power_page import (                                   # noqa: F401
    NAME_TO_INDEX,
    SCHEMA_LINES,
    app,
    live,
    load_schema,
    telem_frame,
)


# ---------------------------------------------------------------- 1. 解码器


def built_schema() -> TelemSchema:
    schema = TelemSchema()
    for line in SCHEMA_LINES:
        schema.feed_line(line)
    return schema


def test_a_nan_sample_does_not_reject_the_frame_or_dirty_the_counters():
    decoder = TelemDecoder()
    decoder.bind_schema(built_schema())
    payload = telem_frame({"batt_v": float("nan"), "batt_i": float("nan")})

    samples = decoder.feed(payload)

    assert len(samples) == 1
    assert decoder.stats.frames_ok == 1 and decoder.stats.samples == 1
    assert decoder.stats.rejected_total == 0
    assert decoder.stats.rejected_length == 0 and decoder.stats.rejected_schema == 0
    values = samples[0].values
    assert math.isnan(values[NAME_TO_INDEX["batt_v"]])
    assert math.isnan(values[NAME_TO_INDEX["batt_i"]])


def test_a_nan_frame_does_not_break_the_sequence_bookkeeping():
    """NaN 不是坏帧。把它算成丢帧会让"链路质量"这条统计从此说谎。"""
    decoder = TelemDecoder()
    decoder.bind_schema(built_schema())
    for seq in range(3):
        value = float("nan") if seq == 1 else 12.0
        decoder.feed(telem_frame({"batt_v": value}, seq=seq))
    assert decoder.stats.frames_ok == 3
    assert decoder.stats.seq_gaps == 0 and decoder.stats.lost_frames == 0


def test_the_ring_stores_nan_without_turning_it_into_zero():
    ring = TelemRing(capacity=8, channel_count=16)
    decoder = TelemDecoder()
    decoder.bind_schema(built_schema())
    ring.push_many(decoder.feed(telem_frame({"batt_v": float("nan")})))
    ring.push_many(decoder.feed(telem_frame({"batt_v": 12.0}, seq=1, t_us=25000)))

    _times, values = ring.snapshot(NAME_TO_INDEX["batt_v"])
    assert math.isnan(values[0]) and values[1] == pytest.approx(12.0)
    latest = ring.latest(NAME_TO_INDEX["batt_v"])
    assert latest is not None and latest[1] == pytest.approx(12.0)


# ---------------------------------------------------------------- 2. 组件


def test_the_tile_facing_accessor_reports_nan_as_no_data(live):
    """组件已经全都处理 `None`，所以在取数这一处收口最难漏。"""
    app = live
    load_schema(app)
    app._dashboard_on_binary_frame(0x2230, telem_frame({"batt_v": float("nan")}))

    assert app._dashboard_latest("batt_v") is None
    # 但原始值仍然拿得到：电源页要凭它区分"没有样本"和"收到 NaN"。
    raw = app._telem_latest_raw("batt_v")
    assert raw is not None and math.isnan(raw[1])

    app._dashboard_on_binary_frame(0x2230, telem_frame({"batt_v": 12.0}, seq=1, t_us=25000))
    assert app._dashboard_latest("batt_v") == pytest.approx(12.0)


def test_a_value_tile_shows_a_dash_for_nan_not_a_number(live):
    app = live
    load_schema(app)
    card = next(t for t in app.dashboard_tiles
                if t.spec.type == dash_layout.TILE_VALUE and not t.missing)
    name = card.spec.bindings[0]

    app._dashboard_on_binary_frame(0x2230, telem_frame({name: 1.25}))
    card.refresh()
    assert "1.25" in card.value_var.get()

    app._dashboard_on_binary_frame(
        0x2230, telem_frame({name: float("nan")}, seq=1, t_us=25000))
    card.refresh()
    assert card.value_var.get() == "—"
    assert "nan" not in card.value_var.get().lower()
    assert card.value_var.get() != dash_tiles.MISSING_CHANNEL_TEXT


def test_a_wave_tile_legend_shows_a_dash_for_nan(live):
    app = live
    load_schema(app)
    wave = next(t for t in app.dashboard_tiles
                if t.spec.type == dash_layout.TILE_WAVE and not t.missing)
    name = wave.spec.bindings[0]

    app._dashboard_on_binary_frame(0x2230, telem_frame({name: float("nan")}))
    wave.refresh()                                  # 不崩是判据之一
    assert "nan" not in wave.legend_vars[name].get().lower()
    assert wave.legend_vars[name].get().startswith(f"{name} —")


def test_the_scope_drops_nan_instead_of_drawing_a_line_to_zero(live):
    """NaN 进画布会做两件坏事：把自动量程算成 NaN、把 NaN 坐标递给 Tcl。"""
    app = live
    canvas = ScopeCanvas(app, width=200, height=120)
    canvas.set_curves([(0, "batt_v")])
    times = np.array([0.0, 0.1, 0.2, 0.3], dtype=np.float64)
    values = np.array([12.0, float("nan"), 12.4, float("nan")], dtype=np.float32)

    canvas.render({0: (times, values)})

    low, high = canvas.last_range
    assert math.isfinite(low) and math.isfinite(high)
    assert low <= 12.0 and high >= 12.4
    assert low > 0.0, "量程不该被 NaN 顶到兜底的 ±1，更不该把 0 当成一个样本"
    coords = canvas.canvas.coords(canvas.curves[0].item)
    assert coords and all(math.isfinite(value) for value in coords)
    assert canvas.curves[0].points == 2
    assert canvas.curves[0].latest == pytest.approx(12.4)
    canvas.canvas.destroy()


def test_an_all_nan_curve_is_hidden_rather_than_flattened(live):
    app = live
    canvas = ScopeCanvas(app, width=200, height=120)
    canvas.set_curves([(0, "batt_v")])
    times = np.array([0.0, 0.1], dtype=np.float64)
    values = np.array([float("nan"), float("nan")], dtype=np.float32)

    canvas.render({0: (times, values)})

    assert canvas.curves[0].points == 0
    assert str(canvas.canvas.itemcget(canvas.curves[0].item, "state")) == "hidden"
    canvas.canvas.destroy()


# ---------------------------------------------------------------- 3. 录制


def test_a_recorded_nan_is_not_written_as_zero():
    """CSV 里 NaN 必须还是 NaN：留空是"本帧没带这条通道"，0 是一个假读数。"""
    from tools.panel_lib.record_service import RecordSchema, format_row
    from tools.panel_lib.telem_stream import TelemSample

    schema = RecordSchema.from_telem_schema(built_schema())
    row = format_row(
        TelemSample(t_us=0, values={NAME_TO_INDEX["batt_v"]: float("nan")}, seq=0),
        schema.channel_count,
    )
    cells = row.strip("\n").split(",")
    # 第 0 列是时间戳；通道列从 1 开始。
    assert cells[1 + NAME_TO_INDEX["batt_v"]].lower() == "nan"
    # 本帧没带的通道仍然留空，而不是 0——两种"没有"不能混在一起。
    assert cells[1 + NAME_TO_INDEX["batt_i"]] == ""
