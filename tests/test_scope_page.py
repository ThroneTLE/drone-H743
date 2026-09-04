"""R-T1-3：“示波器 / 调参”页（`panel_lib/scope.py` + `panel_lib/pages/scope.py`）。

全部用真实的 `DronePanel()` + FakeTransport 驱动，照 `test_flow_monitor_page.py`
的范式：不做源码文本断言（那部分在末尾单独一节），行为断言一律走真实控件。

覆盖规划文档 §4 的上位机页面条目：
  * 选中页发 `TELEM STREAM on`、切走发 `off`；
  * 勾选通道发 `TELEM MASK`，且掩码里始终带着全部参数通道；
  * 滑块拖动节流 ≤5 Hz、松手必发；
  * 回显一致 → confirmed，不一致 → diverged 并跳到固件值；
  * 清空缓冲 / 录制不发协议帧以外的东西；
  * `schema` 指纹不符触发整表重拉；
  * 重绘基准（8 曲线 × 10 k 点 ≤5 ms）。
"""

from __future__ import annotations

import struct
import time
import tkinter as tk
from pathlib import Path

import numpy as np
import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib import scope as scope_widget
from tools.panel_lib.pages import scope as scope_page
from tools.panel_lib.telem_stream import TELEM_FRAME_HEADER_BYTES, TelemSchema


ROOT = Path(__file__).resolve().parents[1]


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.lines: list[str] = []
        self.frames: list[tuple[int, bytes]] = []
        self.binary_sink = None
        self.binary_unclaimed = 0

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        self.frames.append((int(function), bytes(payload)))
        return True

    def set_binary_sink(self, sink) -> None:
        self.binary_sink = sink


# 一份最小但真实形状的通道表：3 条曲线通道 + 2 条参数通道。
SCHEMA_LINES = [
    "TELEM ver=2 n=5 rate=40 page=6 hash=00000000",
    "TELEM CH idx=0 name=roll unit=deg min=-180.000 max=180.000 grp=attitude param=-",
    "TELEM CH idx=1 name=pitch unit=deg min=-90.000 max=90.000 grp=attitude param=-",
    "TELEM CH idx=2 name=vel_est_x unit=m/s min=-5.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=3 name=roll_rate_kd unit=- min=0.000 max=10.000 grp=gain param=coax.roll_rate_kd",
    "TELEM CH idx=4 name=pitch_rate_kd unit=- min=0.000 max=10.000 grp=gain param=coax.pitch_rate_kd",
    "TELEM PAGE from=0 count=5 next=-1",
]


def schema_hash() -> int:
    schema = TelemSchema()
    for line in SCHEMA_LINES:
        schema.feed_line(line)
    return schema.computed_hash()


def telem_frame(values: dict[int, float], *, seq: int = 0, t_us: int = 0,
                schema: int | None = None) -> bytes:
    """按规划 §2.2 拼一帧 payload（transport 已经把 $X 壳剥掉了）。"""
    mask = 0
    for index in values:
        mask |= 1 << index
    ordered = [values[index] for index in sorted(values)]
    head = struct.pack(
        "<BBHIIHHQ", 1, 1, seq & 0xFFFF,
        schema_hash() if schema is None else schema, t_us, 0, 0, mask,
    )
    assert len(head) == TELEM_FRAME_HEADER_BYTES
    return head + b"".join(struct.pack("<f", value) for value in ordered)


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


@pytest.fixture
def app(_panel):
    """复用同一个面板，只清这条链路的数据状态——控件引用要留着。"""
    _panel._scope_reset_session()
    _panel.transport = FakeTransport()
    _panel.update_idletasks()
    return _panel


def load_schema(app) -> None:
    for line in SCHEMA_LINES:
        app._handle_board_line(line)
    app.update_idletasks()


def select_scope_tab(app) -> None:
    app.notebook.select(app.scope_tab)
    app.update_idletasks()


def leave_scope_tab(app) -> None:
    app.notebook.select(app.notebook.tabs()[0])
    app.update_idletasks()


# ---------------------------------------------------------------- 挂载


def test_the_page_is_mounted_as_a_real_notebook_tab(app) -> None:
    assert app.scope_tab is not None
    labels = [app.notebook.tab(tab, "text") for tab in app.notebook.tabs()]
    assert "示波器 / 调参" in labels
    assert app.scope_canvas is not None


def test_panel_entry_point_only_carries_the_mount(app) -> None:
    """`drone_tcp_panel.py` 只减不增：本页在那里只允许留挂载。"""
    source = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
    assert source.count("_scope_") == 3          # mount / handle_line / poll_tick
    assert "self._scope_mount(self.notebook)" in source
    assert "self._scope_handle_line(line)" in source
    assert "self._scope_poll_tick(now)" in source
    # 页面实现一行都不许留在这里。
    assert "ScopeCanvas" not in source
    assert "TELEM MASK" not in source
    assert "PARAM SET" not in source or "PARAM SET {name}" in source


# ---------------------------------------------------------------- 可见性门控


def test_stream_follows_tab_visibility(app) -> None:
    select_scope_tab(app)
    app._scope_poll_tick(time.monotonic())
    assert "TELEM STREAM on" in app.transport.lines
    assert app.transport.binary_sink is not None

    leave_scope_tab(app)
    app._scope_poll_tick(time.monotonic())
    assert app.transport.lines[-1] == "TELEM STREAM off"
    # 切走之后收线程不再往本页写：sink 摘掉，链路上的帧计进 binary_unclaimed。
    assert app.transport.binary_sink is None


def test_stream_is_not_toggled_every_tick(app) -> None:
    select_scope_tab(app)
    app._scope_poll_tick(time.monotonic())
    app._scope_poll_tick(time.monotonic())
    app._scope_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1


def test_schema_is_pulled_once_when_the_page_opens(app) -> None:
    select_scope_tab(app)
    app._scope_poll_tick(time.monotonic())
    assert "TELEM?" in app.transport.lines
    assert "TELEM CH from=0" in app.transport.lines

    before = list(app.transport.lines)
    app._scope_poll_tick(time.monotonic())
    assert app.transport.lines == before


# ---------------------------------------------------------------- 通道表


def test_schema_builds_the_channel_tree_and_the_sliders(app) -> None:
    load_schema(app)

    assert app.scope_schema.complete
    # 曲线树只放非参数通道；增益走滑块。
    leaves = [
        item
        for group in app.scope_channel_tree.get_children("")
        for item in app.scope_channel_tree.get_children(group)
    ]
    assert sorted(leaves) == ["0", "1", "2"]
    assert sorted(app.scope_slider_vars) == [3, 4]


def test_selecting_channels_sends_a_mask_that_keeps_the_gain_channels(app) -> None:
    load_schema(app)
    app.transport.lines.clear()

    app._scope_toggle_channel(1)                      # 取消勾选 pitch
    assert app.transport.lines, "勾选变化必须立刻改帧掩码"
    mask = int(app.transport.lines[-1].split()[-1], 16)
    assert mask & (1 << 1) == 0
    assert mask & (1 << 0)
    # 参数通道永远在掩码里：它们平时不置位，但 1 Hz 全量刷新帧要靠它们把
    # 滑块喂回来；剔出去滑块就永远停在初值。
    assert mask & (1 << 3) and mask & (1 << 4)


WIDE_SCHEMA_LINES = (
    [f"TELEM ver=2 n=12 rate=40 page=12 hash=00000000"]
    + [
        f"TELEM CH idx={index} name=ch{index} unit=- "
        f"min=-1.000 max=1.000 grp=nav param=-"
        for index in range(12)
    ]
    + ["TELEM PAGE from=0 count=12 next=-1"]
)


def test_curve_count_is_capped_and_says_so(app) -> None:
    """点了没反应是最难查的界面问题之一，到上限必须说出来。"""
    for line in WIDE_SCHEMA_LINES:
        app._handle_board_line(line)
    app.update_idletasks()

    assert len(app.scope_selected) == scope_page.SCOPE_MAX_CURVES
    unselected = sorted(set(range(12)) - app.scope_selected)
    assert unselected

    before = set(app.scope_selected)
    app._scope_toggle_channel(unselected[0])
    assert app.scope_selected == before
    assert f"最多同时显示 {scope_page.SCOPE_MAX_CURVES} 条曲线" in app.scope_hint_var.get()

    # 取消一条之后就又能勾上了。
    app._scope_toggle_channel(sorted(before)[0])
    app._scope_toggle_channel(unselected[0])
    assert unselected[0] in app.scope_selected
    assert len(app.scope_selected) == scope_page.SCOPE_MAX_CURVES


# ---------------------------------------------------------------- 数据路径


def test_frames_from_the_rx_thread_reach_the_ring(app) -> None:
    load_schema(app)
    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.0, 1: 2.0, 2: 3.0}, t_us=0))
    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.5, 1: 2.5, 2: 3.5},
                                                   seq=1, t_us=25000))

    times, values = app.scope_ring.snapshot(0)
    assert values.tolist() == [1.0, 1.5]
    assert times.tolist() == pytest.approx([0.0, 0.025])
    assert app.scope_decoder.stats.rejected_total == 0


def test_a_stale_schema_hash_triggers_a_full_reload(app) -> None:
    load_schema(app)
    select_scope_tab(app)
    app.transport.lines.clear()

    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.0}, schema=0xDEADBEEF))
    assert app.scope_decoder.needs_schema_reload
    # 指纹不符的帧一个样本都不许入环。
    assert app.scope_ring.used(0) == 0

    app._scope_poll_tick(time.monotonic())
    assert "TELEM?" in app.transport.lines
    assert "TELEM CH from=0" in app.transport.lines


def test_clearing_the_buffer_sends_nothing(app) -> None:
    load_schema(app)
    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.0}))
    assert app.scope_ring.used(0) == 1

    app.transport.lines.clear()
    app.transport.frames.clear()
    app._scope_clear_buffer()

    assert app.scope_ring.used(0) == 0
    # 固件那边没有"缓冲"可清，所以这个按钮一帧都不该发。
    assert app.transport.lines == []
    assert app.transport.frames == []


# ---------------------------------------------------------------- 滑块三态


def test_drag_is_throttled_and_release_always_sends(app) -> None:
    load_schema(app)
    app.transport.frames.clear()

    now = time.monotonic()
    app.scope_slider_vars[3].set(1.0)
    app._scope_send_param(3, now)
    assert len(app.transport.frames) == 1

    # 节流窗口内的拖动被吃掉。
    app.scope_slider_vars[3].set(1.1)
    app._scope_slider_dragged(3)
    assert len(app.transport.frames) == 1

    # 松手必发——否则滑块停在 1.1、固件停在 1.0，而且没有任何提示。
    app._scope_slider_released(3)
    assert len(app.transport.frames) == 2
    function, payload = app.transport.frames[-1]
    assert function == panel.PROTO_REQ_PARAM_SET
    assert payload.decode("utf-8").startswith("PARAM SET coax.roll_rate_kd 1.1")


def test_throttle_rate_is_at_most_five_per_second(app) -> None:
    assert scope_page.SCOPE_SLIDER_THROTTLE_S >= 0.2

    load_schema(app)
    app.transport.frames.clear()
    start = time.monotonic()
    for step in range(50):
        app.scope_slider_vars[3].set(step * 0.1)
        app._scope_slider_dragged(3)
    elapsed = max(time.monotonic() - start, 1e-6)
    assert len(app.transport.frames) <= max(1, int(elapsed * 5) + 1)


def test_matching_echo_confirms_and_a_clamped_echo_diverges(app) -> None:
    load_schema(app)

    app.scope_slider_vars[3].set(2.0)
    app._scope_send_param(3, time.monotonic())
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_PENDING

    app._scope_note_param_echo(3, 2.0)
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_CONFIRMED

    # 固件钳位：滑块必须跳到固件的实际值，留在用户拖到的位置等于界面撒谎。
    app.scope_slider_vars[4].set(9.5)
    app._scope_send_param(4, time.monotonic())
    # 宽限期（在线旧帧窗口）之后仍是旧值才判 diverged，见 test_scope_page_link_lifecycle。
    app._scope_note_param_echo(4, 5.0, now=time.monotonic() + scope_page.SCOPE_ECHO_GRACE_S)
    assert app.scope_slider_state[4] == scope_page.SCOPE_STATE_DIVERGED
    assert app.scope_slider_vars[4].get() == pytest.approx(5.0)


def test_an_unsolicited_echo_just_follows_the_firmware(app) -> None:
    """LOAD / DEFAULTS 之后固件自己变了值：滑块跟随，不报 diverged。"""
    load_schema(app)
    app._scope_note_param_echo(3, 4.25)
    assert app.scope_slider_vars[3].get() == pytest.approx(4.25)
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_IDLE


def test_echo_tolerance_separates_rounding_from_rejection(app) -> None:
    load_schema(app)
    app.scope_slider_vars[3].set(3.0)
    app._scope_send_param(3, time.monotonic())
    # 固件按 1e-3 精度格式化，这种量级的差异是舍入不是拒绝。
    app._scope_note_param_echo(3, 3.0 + 1e-7)
    assert app.scope_slider_state[3] == scope_page.SCOPE_STATE_CONFIRMED


# ---------------------------------------------------------------- 统计条


def test_stats_bar_reports_link_state(app) -> None:
    load_schema(app)
    app._handle_board_line(
        "TELEM STREAM stream=1 rate=40 mask=000000000000001F refresh=1 fmt=bin "
        "sink=auto active=uart seq=12 frames=12 drop=3 usb_lost=0"
    )
    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.0}, seq=0))
    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.0}, seq=5))
    app._scope_refresh_stats()

    assert app.scope_stat_vars["sink"].get() == "uart"
    assert app.scope_stat_vars["drop"].get() == "3"
    assert "1 次" in app.scope_stat_vars["gap"].get()
    assert app.scope_stat_vars["schema"].get() == f"{schema_hash():08X}"


# ---------------------------------------------------------------- CSV 录制


def test_recording_writes_a_header_with_names_and_hash(app, tmp_path, monkeypatch) -> None:
    load_schema(app)
    monkeypatch.setattr(scope_page, "TELEMETRY_DIR", tmp_path)
    monkeypatch.setattr(scope_page, "dated_directory", lambda root: root / "2026-09-03")

    app.transport.lines.clear()
    app.transport.frames.clear()
    app._scope_toggle_record()
    assert app.scope_record_handle is not None
    # 录制是纯本地动作，不许顺手改固件的流配置。
    assert app.transport.lines == []
    assert app.transport.frames == []

    app._scope_on_binary_frame(0x2230, telem_frame({0: 1.0, 2: 3.0}, t_us=1000))
    app._scope_flush_record()
    app._scope_stop_record()

    text = app.scope_record_path.read_text(encoding="utf-8").splitlines()
    assert text[0] == f"# schema_hash={schema_hash():08X}"
    assert text[1] == "t_s,roll,pitch,vel_est_x,roll_rate_kd,pitch_rate_kd"
    # 本帧没带的通道留空而不是补 0：补 0 会让"没发"和"值就是 0"分不开。
    assert text[2] == "0.001000,1,,3,,"


def test_recording_lands_under_the_project_data_root() -> None:
    """录制路径必须走 project_paths，不许各页自己拼目录。"""
    source = (ROOT / "tools" / "panel_lib" / "pages" / "scope.py").read_text(encoding="utf-8")
    assert "project_paths import TELEMETRY_DIR" in source
    assert "dated_directory(TELEMETRY_DIR)" in source


# ---------------------------------------------------------------- 抽稀与基准


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


def test_redraw_benchmark_eight_curves_ten_thousand_points(app) -> None:
    """规划文档 §4 的性能判据：8 曲线 × 10 k 点，单次重绘 ≤5 ms。"""
    canvas = app.scope_canvas
    if canvas is None:  # pragma: no cover - 无显示环境
        pytest.skip("scope canvas unavailable")

    canvas.set_curves([(index, f"ch{index}") for index in range(8)])
    canvas.set_window(60.0)
    times = np.linspace(0.0, 60.0, 10000, dtype=np.float64)
    series = {
        index: (times, np.sin(times * (index + 1)).astype(np.float32))
        for index in range(8)
    }

    canvas.render(series)          # 预热：第一次会建 grid item
    app.update_idletasks()

    samples = []
    for _ in range(15):
        start = time.perf_counter()
        canvas.render(series)
        samples.append(time.perf_counter() - start)
    samples.sort()
    median = samples[len(samples) // 2]
    # 判据卡在中位数而不是最好一次：最好一次可以靠运气，而中位数是"平时"。
    # 也不卡最坏一次——那条尾巴是 GC 和操作系统调度，不是这段代码。
    assert median <= 0.005, (
        f"median redraw {median * 1000:.2f} ms "
        f"(best {samples[0] * 1000:.2f}, worst {samples[-1] * 1000:.2f})"
    )
    # 抽稀之后送进 Tcl 的点数必须只跟画布宽度有关，与缓冲里的点数无关。
    left, _top, right, _bottom = canvas.plot_area()
    assert canvas.curves[0].points <= min(right - left, scope_widget.SCOPE_MAX_COLUMNS) * 2 + 3
