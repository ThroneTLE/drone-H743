"""R-T1-5：“状态监视”工作台页（`panel_lib/pages/dashboard.py` + `dashboard/`）。

全部用真实的 `DronePanel()` + FakeTransport 驱动，照 `test_flow_monitor_page.py`
的范式。**本文件同时是 `tests/test_scope_page.py` 与
`tests/test_scope_page_link_lifecycle.py` 的迁移目的地**（R-T1-3 的页面退役），
所以那两份里每一条页面级行为在这里都必须还有对应的一条：

  * 可见性门控 STREAM on/off、不逐拍重发、schema 只拉一次；
  * 先开页再连线 / 拔插重连后补发（审核者实机复现的缺陷 2）；
  * 指纹不符触发整表重拉且坏帧零入环；
  * 收线程帧进环形缓冲；
  * 滑块节流 ≤5 Hz、松手必发、confirmed / diverged / 跟随；
  * 发送后宽限期内的旧值不判 diverged（审核者实机复现的缺陷 3）；
  * 统计条读数；CSV 表头与"未携带通道留空"语义；
  * 挂载点只在 `drone_tcp_panel.py` 留三处调用。

再加上 R-T1-5 自己的判据：首位页签且默认选中、三种组件增删改绑拖动缩放、
掩码并集、输入框发送、预设开箱、换表按名重绑、总重绘 ≤10 ms。
"""

from __future__ import annotations

import struct
import time
import tkinter as tk
from pathlib import Path

import numpy as np
import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib.dashboard import layout as dash_layout
from tools.panel_lib.dashboard import tiles as dash_tiles
from tools.panel_lib.pages import dashboard as dashboard_page
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


class SwitchableTransport(FakeTransport):
    def __init__(self, connected: bool = True) -> None:
        super().__init__()
        self.is_connected = connected


# 一份最小但真实形状的通道表：预设用到的曲线通道 + 两条参数通道。
SCHEMA_LINES = [
    "TELEM ver=2 n=8 rate=40 page=8 hash=00000000",
    "TELEM CH idx=0 name=roll unit=deg min=-180.000 max=180.000 grp=attitude param=-",
    "TELEM CH idx=1 name=pitch unit=deg min=-90.000 max=90.000 grp=attitude param=-",
    "TELEM CH idx=2 name=yaw unit=deg min=-180.000 max=180.000 grp=attitude param=-",
    "TELEM CH idx=3 name=vel_est_x unit=m/s min=-5.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=4 name=vel_est_y unit=m/s min=-5.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=5 name=flow_height unit=m min=0.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=6 name=roll_rate_kd unit=- min=0.000 max=10.000 grp=gain param=coax.roll_rate_kd",
    "TELEM CH idx=7 name=pitch_rate_kd unit=- min=0.000 max=10.000 grp=gain param=coax.pitch_rate_kd",
    "TELEM PAGE from=0 count=8 next=-1",
]

NAME_TO_INDEX = {
    "roll": 0, "pitch": 1, "yaw": 2, "vel_est_x": 3, "vel_est_y": 4,
    "flow_height": 5, "roll_rate_kd": 6, "pitch_rate_kd": 7,
}


def schema_hash() -> int:
    schema = TelemSchema()
    for line in SCHEMA_LINES:
        schema.feed_line(line)
    return schema.computed_hash()


def telem_frame(values: dict[str, float], *, seq: int = 0, t_us: int = 0,
                schema: int | None = None) -> bytes:
    """按规划 §2.2 拼一帧 payload（transport 已经把 $X 壳剥掉了）。"""
    indexed = {NAME_TO_INDEX[name]: value for name, value in values.items()}
    mask = 0
    for index in indexed:
        mask |= 1 << index
    head = struct.pack(
        "<BBHIIHHQ", 1, 1, seq & 0xFFFF,
        schema_hash() if schema is None else schema, t_us, 0, 0, mask,
    )
    assert len(head) == TELEM_FRAME_HEADER_BYTES
    return head + b"".join(
        struct.pack("<f", indexed[index]) for index in sorted(indexed)
    )


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
def app(_panel, monkeypatch):
    """复用同一个面板，只清链路状态与布局——控件引用要留着。

    布局也恢复出厂：用例之间互相看见对方拖过的 tile 会让失败极难定位。
    落盘直接掐掉，测试不该改用户的 panel_state.json。
    """
    monkeypatch.setattr(type(_panel), "_save_panel_state", lambda self: None)
    _panel._dashboard_reset_session()
    _panel.dashboard_layout = dash_layout.default_layout()
    _panel.dashboard_workspace_var.set(0)
    _panel._dashboard_rebuild_workspace_bar()
    _panel._dashboard_rebuild_tiles()
    _panel.transport = SwitchableTransport(connected=True)
    _panel.update_idletasks()
    return _panel


def load_schema(app) -> None:
    for line in SCHEMA_LINES:
        app._handle_board_line(line)
    app.update_idletasks()


def select_dashboard(app) -> None:
    app.notebook.select(app.dashboard_tab)
    app.update_idletasks()


def leave_dashboard(app) -> None:
    for tab in app.notebook.tabs():
        if tab != str(app.dashboard_tab):
            app.notebook.select(tab)
            break
    app.update_idletasks()


def tiles_of(app, tile_type: str) -> list:
    return [t for t in app.dashboard_tiles if t.spec.type == tile_type]


# ---------------------------------------------------------------- 挂载


def test_the_workbench_is_the_first_tab_and_selected_by_default(app) -> None:
    """作者裁决：这是飞控的默认主页面。"""
    tabs = app.notebook.tabs()
    assert tabs[0] == str(app.dashboard_tab)
    assert app.notebook.tab(tabs[0], "text") == dashboard_page.DASHBOARD_TAB_TEXT


def test_panel_entry_point_only_carries_the_mount() -> None:
    """`drone_tcp_panel.py` 只减不增：本页在那里只允许留三处挂载调用。"""
    source = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
    assert source.count("_dashboard_") == 3
    assert "self._dashboard_mount(self.notebook)" in source
    assert "self._dashboard_handle_line(line)" in source
    assert "self._dashboard_poll_tick(now)" in source
    # 退役的那一页一点痕迹都不许留。
    assert "_scope_" not in source
    assert not (ROOT / "tools" / "panel_lib" / "pages" / "scope.py").exists()
    # 页面实现一行都不许回流。
    for leaked in ("ScopeCanvas", "TELEM MASK", "TileSpec"):
        assert leaked not in source


def test_presets_are_available_out_of_the_box(app) -> None:
    names = [w.name for w in app.dashboard_layout.workspaces]
    assert names == ["飞行监控", "控制器调参"]
    assert len(app.dashboard_tiles) == len(dash_layout.flight_monitor_workspace().tiles)

    app.dashboard_workspace_var.set(1)
    app._dashboard_switch_workspace()
    assert len(tiles_of(app, dash_layout.TILE_PARAM)) == 14
    assert len(tiles_of(app, dash_layout.TILE_WAVE)) == 2


# ---------------------------------------------------------------- 可见性门控


def test_stream_follows_tab_visibility(app) -> None:
    leave_dashboard(app)
    app._dashboard_poll_tick(time.monotonic())
    app.transport.lines.clear()

    select_dashboard(app)
    app._dashboard_poll_tick(time.monotonic())
    assert "TELEM STREAM on" in app.transport.lines
    assert app.transport.binary_sink is not None

    leave_dashboard(app)
    app._dashboard_poll_tick(time.monotonic())
    assert app.transport.lines[-1] == "TELEM STREAM off"
    assert app.transport.binary_sink is None


def test_stream_is_not_toggled_every_tick(app) -> None:
    select_dashboard(app)
    for _ in range(3):
        app._dashboard_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1


def test_schema_is_pulled_once_when_the_page_opens(app) -> None:
    select_dashboard(app)
    app._dashboard_poll_tick(time.monotonic())
    assert "TELEM?" in app.transport.lines
    assert "TELEM CH from=0" in app.transport.lines

    before = list(app.transport.lines)
    app._dashboard_poll_tick(time.monotonic())
    assert app.transport.lines == before


def test_stream_starts_when_the_link_comes_up_after_the_page_is_open(app) -> None:
    """审核者实机复现的缺陷 2：只盯可见性翻转，看不见链路变化。"""
    app.transport = SwitchableTransport(connected=False)
    select_dashboard(app)
    app._dashboard_poll_tick(time.monotonic())
    assert "TELEM STREAM on" not in app.transport.lines

    app.transport.is_connected = True
    app._dashboard_poll_tick(time.monotonic())
    app._dashboard_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1
    assert app.transport.binary_sink is not None


def test_stream_is_re_requested_after_a_reconnect(app) -> None:
    select_dashboard(app)
    app._dashboard_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1

    app.transport.is_connected = False
    app._dashboard_poll_tick(time.monotonic())
    app.transport = SwitchableTransport(connected=True)
    app._dashboard_poll_tick(time.monotonic())
    app._dashboard_poll_tick(time.monotonic())
    assert app.transport.lines.count("TELEM STREAM on") == 1
    assert app.transport.binary_sink is not None


# ---------------------------------------------------------------- 通道表


def test_schema_binds_tiles_by_name(app) -> None:
    load_schema(app)
    assert app.dashboard_schema.complete
    waves = tiles_of(app, dash_layout.TILE_WAVE)
    assert waves[0].spec.bindings == ["roll", "pitch", "yaw"]
    assert waves[0].missing == []
    # 预设里的 pos_est_x/y 不在这份精简表里：必须明说，而不是静默消失。
    assert waves[2].missing == ["pos_est_x", "pos_est_y"]


def test_a_missing_channel_is_labelled_not_dropped(app) -> None:
    load_schema(app)
    values = tiles_of(app, dash_layout.TILE_VALUE)
    absent = [t for t in values if t.missing]
    assert absent, "这份精简表里 fusion_acc_err 不存在，应该有卡片报缺"
    assert absent[0].value_var.get() == dash_tiles.MISSING_CHANNEL_TEXT
    # 卡片还在布局里，没有被悄悄删掉。
    assert absent[0].frame.winfo_exists()


def test_rebinding_after_a_new_schema_recovers_the_channel(app) -> None:
    """换固件 / 换表之后按名重绑，之前报缺的卡片要自己回来。"""
    load_schema(app)
    values = tiles_of(app, dash_layout.TILE_VALUE)
    target = next(t for t in values if t.missing)
    missing_name = target.missing[0]

    wider = [
        f"TELEM ver=2 n=9 rate=40 page=9 hash=00000000",
        *[line for line in SCHEMA_LINES if line.startswith("TELEM CH ")],
        f"TELEM CH idx=8 name={missing_name} unit=- min=-1.000 max=1.000 grp=nav param=-",
        "TELEM PAGE from=0 count=9 next=-1",
    ]
    for line in wider:
        app._handle_board_line(line)
    app.update_idletasks()

    assert target.missing == []
    assert target.value_var.get() != dash_tiles.MISSING_CHANNEL_TEXT


def test_mask_is_the_union_of_bound_channels_plus_parameters(app) -> None:
    select_dashboard(app)
    load_schema(app)
    mask = app._dashboard_mask()

    for name in ("roll", "pitch", "yaw", "vel_est_x", "vel_est_y", "flow_height"):
        assert mask & (1 << NAME_TO_INDEX[name]), name
    # 参数通道恒在掩码里：平时不置位，但 1 Hz 全量刷新帧要靠它们喂回滑块。
    assert mask & (1 << NAME_TO_INDEX["roll_rate_kd"])
    assert mask & (1 << NAME_TO_INDEX["pitch_rate_kd"])
    assert f"TELEM MASK {mask:X}" in app.transport.lines


def test_switching_workspace_resends_the_mask(app) -> None:
    select_dashboard(app)
    load_schema(app)
    app.transport.lines.clear()

    app.dashboard_workspace_var.set(1)
    app._dashboard_switch_workspace()
    masks = [line for line in app.transport.lines if line.startswith("TELEM MASK ")]
    assert masks, "看不见的工作区没有理由占带宽，切换必须重发掩码"


def test_a_stale_schema_hash_triggers_a_full_reload(app) -> None:
    load_schema(app)
    select_dashboard(app)
    app.transport.lines.clear()

    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll": 1.0}, schema=0xDEADBEEF))
    assert app.dashboard_decoder.needs_schema_reload
    assert app.dashboard_ring.used(0) == 0          # 坏帧零入环

    app._dashboard_poll_tick(time.monotonic())
    assert "TELEM?" in app.transport.lines


# ---------------------------------------------------------------- 数据路径


def test_frames_from_the_rx_thread_reach_the_ring(app) -> None:
    load_schema(app)
    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll": 1.0, "pitch": 2.0}))
    app._dashboard_on_binary_frame(
        0x2230, telem_frame({"roll": 1.5, "pitch": 2.5}, seq=1, t_us=25000)
    )

    times, values = app.dashboard_ring.snapshot(0)
    assert values.tolist() == [1.0, 1.5]
    assert times.tolist() == pytest.approx([0.0, 0.025])
    assert app._dashboard_latest("roll") == pytest.approx(1.5)
    assert app.dashboard_decoder.stats.rejected_total == 0


def test_value_tile_shows_the_latest_reading(app) -> None:
    load_schema(app)
    card = next(t for t in tiles_of(app, dash_layout.TILE_VALUE)
                if t.spec.bindings == ["flow_height"])
    app._dashboard_on_binary_frame(0x2230, telem_frame({"flow_height": 0.612}))
    card.refresh()

    assert "0.612" in card.value_var.get()
    assert card.unit_var.get() == "m"


def test_wave_tile_legend_carries_the_current_value(app) -> None:
    load_schema(app)
    wave = tiles_of(app, dash_layout.TILE_WAVE)[0]
    app._dashboard_on_binary_frame(
        0x2230, telem_frame({"roll": -12.5, "pitch": 3.0, "yaw": 90.0})
    )
    wave.refresh()

    assert "-12.5" in wave.legend_vars["roll"].get()
    assert "deg" in wave.legend_vars["roll"].get()


def test_clearing_the_buffer_sends_nothing(app) -> None:
    load_schema(app)
    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll": 1.0}))
    assert app.dashboard_ring.used(0) == 1

    app.transport.lines.clear()
    app.transport.frames.clear()
    app._dashboard_clear_buffer()

    assert app.dashboard_ring.used(0) == 0
    # 固件那边没有"缓冲"可清，所以这个按钮一帧都不该发。
    assert app.transport.lines == []
    assert app.transport.frames == []


def test_a_wave_tile_binds_at_most_four_channels(app) -> None:
    """一张画布上超过 4 条曲线颜色就分不开了，而且是 R-T1-3 被打回的老路。"""
    load_schema(app)
    assert dash_tiles.WaveTile.MAX_BINDINGS == dash_tiles.WAVE_MAX_BINDINGS == 4
    wave = tiles_of(app, dash_layout.TILE_WAVE)[0]
    wave.spec.bindings = ["roll", "pitch", "yaw", "vel_est_x", "vel_est_y"]
    wave.rebind()
    assert len(wave._curve_names) == 4


def test_tile_header_pickers_choose_live_channels_update_mask_and_persist(app) -> None:
    """不用进编辑模式：每张波形/数值卡在顶部直接按当前 schema 改绑定。"""
    select_dashboard(app)
    load_schema(app)
    wave = tiles_of(app, dash_layout.TILE_WAVE)[0]
    card = next(tile for tile in tiles_of(app, dash_layout.TILE_VALUE)
                if tile.spec.bindings == ["flow_height"])

    assert wave.channel_picker.selected == ("roll", "pitch", "yaw")
    assert card.channel_picker.selected == ("flow_height",)
    app.transport.lines.clear()

    assert wave.channel_picker.choose(["vel_est_x", "vel_est_y", "flow_height"])
    assert wave.spec.bindings == ["vel_est_x", "vel_est_y", "flow_height"]
    assert set(wave._curve_names.values()) == {"vel_est_x", "vel_est_y", "flow_height"}
    assert not wave.channel_picker.choose([
        "roll", "pitch", "yaw", "vel_est_x", "vel_est_y",
    ]), "波形一张最多四条，不能悄悄截断用户选择"
    assert wave.spec.bindings == ["vel_est_x", "vel_est_y", "flow_height"]
    assert [line for line in app.transport.lines if line.startswith("TELEM MASK ")]

    app.transport.lines.clear()
    assert card.channel_picker.choose(["yaw"])
    assert card.spec.bindings == ["yaw"]
    assert card.name_var.get() == "yaw"
    assert card.unit_var.get() == "deg"
    app._dashboard_on_binary_frame(0x2230, telem_frame({"yaw": 90.0}))
    card.refresh()
    assert "90" in card.value_var.get()
    assert [line for line in app.transport.lines if line.startswith("TELEM MASK ")]

    persisted = dash_layout.DashboardLayout.from_json(app._panel_state["dashboard"])
    assert persisted is not None
    assert persisted.workspaces[0].tiles[0].bindings == [
        "vel_est_x", "vel_est_y", "flow_height",
    ]
    assert persisted.workspaces[0].tiles[3].bindings == ["yaw"]


# ---------------------------------------------------------------- 参数滑块卡


def param_card(app, name: str):
    app.dashboard_workspace_var.set(1)
    app._dashboard_switch_workspace()
    load_schema(app)
    return next(t for t in tiles_of(app, dash_layout.TILE_PARAM)
                if t.spec.bindings == [name])


def test_drag_is_throttled_and_release_always_sends(app) -> None:
    card = param_card(app, "roll_rate_kd")
    app.transport.frames.clear()

    card.scale_var.set(1.0)
    card._on_drag()
    assert len(app.transport.frames) == 1

    card.scale_var.set(1.1)
    card._on_drag()                       # 节流窗口内被吃掉
    assert len(app.transport.frames) == 1

    card._on_release()                    # 松手必发
    assert len(app.transport.frames) == 2
    function, payload = app.transport.frames[-1]
    assert function == panel.PROTO_REQ_PARAM_SET
    assert payload.decode("utf-8").startswith("PARAM SET coax.roll_rate_kd 1.1")


def test_throttle_rate_is_at_most_five_per_second(app) -> None:
    assert dash_tiles.PARAM_THROTTLE_S >= 0.2
    card = param_card(app, "roll_rate_kd")
    app.transport.frames.clear()

    start = time.monotonic()
    for step in range(50):
        card.scale_var.set(step * 0.1)
        card._on_drag()
    elapsed = max(time.monotonic() - start, 1e-6)
    assert len(app.transport.frames) <= max(1, int(elapsed * 5) + 1)


def test_the_entry_box_sends_an_exact_value(app) -> None:
    """滑块拖不到 0.0671 这种数，输入框才是调 PID 真正用的入口。"""
    card = param_card(app, "roll_rate_kd")
    app.transport.frames.clear()

    card.entry_var.set("0.0671")
    card._on_entry_commit()

    assert len(app.transport.frames) == 1
    assert app.transport.frames[0][1].decode("utf-8") == "PARAM SET coax.roll_rate_kd 0.0671"
    assert card.scale_var.get() == pytest.approx(0.0671)


def test_a_malformed_entry_sends_nothing(app) -> None:
    """输错了就退回当前回显值。静默发一个 0 出去比什么都不做糟糕得多。"""
    card = param_card(app, "roll_rate_kd")
    app.transport.frames.clear()
    card.entry_var.set("十二")
    card._on_entry_commit()
    assert app.transport.frames == []


def test_matching_echo_confirms_and_a_clamped_echo_diverges(app) -> None:
    card = param_card(app, "roll_rate_kd")
    tracker = card.tracker
    now = time.monotonic()

    tracker.note_sent(2.0, now)
    assert tracker.state == dash_tiles.PARAM_STATE_PENDING
    assert tracker.note_echo(2.0, now + 0.04) == "follow"
    assert tracker.state == dash_tiles.PARAM_STATE_CONFIRMED

    tracker.note_sent(9.5, now)
    assert tracker.note_echo(5.0, now + 1.0) == "follow"
    assert tracker.state == dash_tiles.PARAM_STATE_DIVERGED
    assert tracker.display == pytest.approx(5.0)


def test_a_stale_echo_right_after_sending_does_not_diverge(app) -> None:
    """审核者实机复现的缺陷 3：在线旧值帧把滑块拽回去。

    发送值不是飞控值。尚未有真实回显时，界面必须保持“未知”而不是拿上位机
    刚发出去的值冒充已应用的参数。
    """
    card = param_card(app, "roll_rate_kd")
    tracker = card.tracker
    now = time.monotonic()

    tracker.note_sent(2.0, now)
    assert tracker.note_echo(0.5, now + 0.010) == "hold"
    assert tracker.state == dash_tiles.PARAM_STATE_PENDING
    assert tracker.display is None

    assert tracker.note_echo(2.0, now + 0.040) == "follow"
    assert tracker.state == dash_tiles.PARAM_STATE_CONFIRMED


def test_an_echo_inside_the_throttle_window_does_not_snap_the_slider_back(app) -> None:
    """拖动中被拽回固件上一拍的值，手感就是滑块往回蹦。"""
    card = param_card(app, "roll_rate_kd")
    tracker = card.tracker
    now = time.monotonic()

    tracker.note_sent(3.0, now)
    tracker.note_echo(3.0, now + 0.01)          # confirmed，不再 pending
    assert tracker.note_echo(0.1, now + 0.05) == "hold"


def test_an_unsolicited_echo_just_follows_the_firmware(app) -> None:
    """LOAD / DEFAULTS 之后固件自己变了值：跟随，不报 diverged。"""
    card = param_card(app, "roll_rate_kd")
    tracker = card.tracker
    assert tracker.note_echo(4.25, time.monotonic()) == "follow"
    assert tracker.state == dash_tiles.PARAM_STATE_IDLE
    assert tracker.display == pytest.approx(4.25)


def test_param_card_refresh_drives_the_state_from_telemetry(app) -> None:
    card = param_card(app, "roll_rate_kd")
    app.transport.frames.clear()
    card.scale_var.set(2.5)
    card._on_release()

    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll_rate_kd": 2.5}))
    card.refresh()
    assert card.state_var.get() == dash_tiles.PARAM_STATE_CONFIRMED
    assert "2.5" in card.value_var.get()


def test_param_card_refresh_survives_a_tcl_only_combobox_popdown_focus(
    app, monkeypatch,
) -> None:
    """打开 ttk 下拉框时，内部 popdown 没有对应的 Tkinter Widget。"""
    card = param_card(app, "roll_rate_kd")
    tracker = card.tracker
    assert tracker is not None
    tracker.note_echo(2.5, time.monotonic())

    combo = app._serial_port_combo
    popdown = str(app.tk.call("ttk::combobox::PopdownWindow", str(combo)))
    listbox = f"{popdown}.f.l"
    assert "popdown" not in combo.children
    with pytest.raises(KeyError, match="popdown"):
        app.nametowidget(listbox)

    def tcl_only_focus_get():
        raise KeyError("popdown")

    monkeypatch.setattr(card.frame, "focus_get", tcl_only_focus_get)
    card.refresh()

    assert card.entry_var.get() == "2.5"


def test_param_card_marks_only_a_flight_controller_echo_green(app) -> None:
    """绿色只能由遥测回显触发，不能由本机 send_param 成功触发。"""
    card = param_card(app, "roll_rate_kd")
    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll_rate_kd": 1.0}))
    card.refresh()

    card.scale_var.set(2.5)
    card._on_release()
    assert card.state_var.get() == dash_tiles.PARAM_STATE_PENDING
    # 大字继续展示已回显的 1.0，不能展示刚发送、尚未证实的 2.5。
    assert float(card.value_var.get()) == pytest.approx(1.0)
    assert card.feedback_var.get() == "等待飞控回显"
    assert card.feedback_dot.itemcget(card._feedback_dot, "fill") == (
        dash_tiles.PARAM_FEEDBACK_PENDING_COLOUR
    )

    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll_rate_kd": 2.5}))
    card.refresh()
    assert card.state_var.get() == dash_tiles.PARAM_STATE_CONFIRMED
    assert card.feedback_var.get() == "飞控已回显"
    assert card.feedback_dot.itemcget(card._feedback_dot, "fill") == (
        dash_tiles.PARAM_FEEDBACK_CONFIRMED_COLOUR
    )


def test_param_card_marks_a_missing_echo_red_without_claiming_the_sent_value(app) -> None:
    card = param_card(app, "roll_rate_kd")
    tracker = card.tracker
    assert tracker is not None
    tracker.note_sent(2.5, time.monotonic() - dash_tiles.PARAM_ECHO_TIMEOUT_S - 0.01)

    card.refresh()
    assert card.state_var.get() == dash_tiles.PARAM_STATE_DIVERGED
    assert card.feedback_var.get() == "未收到飞控回显"
    assert card.feedback_dot.itemcget(card._feedback_dot, "fill") == (
        dash_tiles.PARAM_FEEDBACK_DIVERGED_COLOUR
    )
    assert card.value_var.get() == "—"


# ---------------------------------------------------------------- 编辑模式


def test_edit_mode_overlays_appear_and_disappear(app) -> None:
    select_dashboard(app)
    editor = app.dashboard_editor
    assert editor.overlays == {}

    app.dashboard_edit_var.set(True)
    app._dashboard_toggle_edit()
    assert len(editor.overlays) == len(app.dashboard_tiles)

    app.dashboard_edit_var.set(False)
    app._dashboard_toggle_edit()
    assert editor.overlays == {}


def test_a_tile_can_be_moved_and_the_move_is_persisted(app) -> None:
    saved: list[dict] = []
    app._panel_state["dashboard"] = None
    original_persist = app._dashboard_persist

    def capture() -> None:
        original_persist()
        saved.append(app._panel_state["dashboard"])

    app._dashboard_persist = capture
    try:
        spec = app._dashboard_specs()[3]         # 一张数值卡
        editor = app.dashboard_editor
        moved = dash_layout.TileSpec(spec.type, spec.col, spec.row + 4,
                                     spec.colspan, spec.rowspan)
        assert editor.try_apply(spec, moved)
        editor.commit()
    finally:
        del app._dashboard_persist

    assert saved, "拖完必须落盘"
    restored = dash_layout.DashboardLayout.from_json(saved[-1])
    assert restored is not None
    assert restored.workspaces[0].tiles[3].row == moved.row


def test_a_move_onto_another_tile_is_refused(app) -> None:
    editor = app.dashboard_editor
    specs = app._dashboard_specs()
    first, second = specs[0], specs[1]
    before = (first.col, first.row)

    collide = dash_layout.TileSpec(first.type, second.col, second.row,
                                   first.colspan, first.rowspan)
    assert not editor.try_apply(first, collide)
    assert (first.col, first.row) == before


def test_a_tile_can_be_resized_within_the_grid(app) -> None:
    editor = app.dashboard_editor
    spec = app._dashboard_specs()[3]
    resized = dash_layout.TileSpec(spec.type, spec.col, spec.row, spec.colspan, 1)
    assert editor.try_apply(spec, resized)
    assert spec.rowspan == 1


def test_adding_and_deleting_a_tile_round_trips(app) -> None:
    select_dashboard(app)
    load_schema(app)
    before = len(app._dashboard_specs())

    spec = app._dashboard_add_tile(dash_layout.TILE_VALUE)
    assert spec is not None
    assert len(app._dashboard_specs()) == before + 1
    assert len(app.dashboard_tiles) == before + 1

    app._dashboard_delete_tile(spec)
    assert len(app._dashboard_specs()) == before
    assert len(app.dashboard_tiles) == before


def test_rebinding_through_the_properties_path_updates_mask_and_tile(app) -> None:
    select_dashboard(app)
    load_schema(app)
    card = next(t for t in tiles_of(app, dash_layout.TILE_VALUE)
                if t.spec.bindings == ["flow_height"])
    app.transport.lines.clear()

    card.spec.bindings = ["yaw"]
    app._dashboard_apply_properties(card.spec)

    rebuilt = next(t for t in tiles_of(app, dash_layout.TILE_VALUE)
                   if t.spec.bindings == ["yaw"])
    assert rebuilt.missing == []
    assert [line for line in app.transport.lines if line.startswith("TELEM MASK ")]


def test_layout_survives_a_reload_from_panel_state(app) -> None:
    select_dashboard(app)
    spec = app._dashboard_specs()[0]
    spec.options["window_s"] = 30
    app._dashboard_persist()

    app._dashboard_load_layout()
    assert app._dashboard_specs()[0].options["window_s"] == 30


def test_restore_presets_puts_the_factory_layout_back(app) -> None:
    app._dashboard_specs().clear()
    app._dashboard_rebuild_tiles()
    assert app.dashboard_tiles == []

    app._dashboard_restore_presets()
    assert [w.name for w in app.dashboard_layout.workspaces] == ["飞行监控", "控制器调参"]
    assert len(app.dashboard_tiles) == len(dash_layout.flight_monitor_workspace().tiles)


# ---------------------------------------------------------------- R-T1-5b 第二批组件


SECOND_BATCH_TYPES = (
    dash_layout.TILE_GAUGE,
    dash_layout.TILE_BUTTON,
    dash_layout.TILE_ATTITUDE,
    dash_layout.TILE_CHANNELS,
)


def add_dashboard_tile(app, tile_type: str, bindings: list[str] | None = None,
                       options: dict | None = None):
    """走真实页面的新增/改绑/重建路径，不手搓 widget。"""
    spec = app._dashboard_add_tile(tile_type)
    assert spec is not None
    spec.bindings = list(bindings or [])
    spec.options.update(options or {})
    app._dashboard_apply_properties(spec)
    return next(tile for tile in app.dashboard_tiles if tile.spec is spec)


def test_second_batch_components_are_registered_and_appear_in_the_add_menu(app) -> None:
    """未 import 就不会执行 register_tile，菜单会静默少掉整批组件。"""
    assert set(SECOND_BATCH_TYPES) <= set(dash_tiles.TILE_CLASSES)
    menu = app.dashboard_add_menu
    labels = [menu.entrycget(index, "label") for index in range(menu.index("end") + 1)]
    for tile_type in SECOND_BATCH_TYPES:
        assert dash_tiles.TILE_CLASSES[tile_type].LABEL in labels


def test_extra_tiles_add_bind_and_persist_through_the_real_dashboard(app) -> None:
    """四种第二批 tile 都能新增、改绑（需要绑定的）并随面板状态往返。"""
    load_schema(app)
    gauge = add_dashboard_tile(app, dash_layout.TILE_GAUGE, ["flow_height"])
    button = add_dashboard_tile(
        app, dash_layout.TILE_BUTTON,
        options={"label": "清零", "command": "FLOW ZERO"},
    )
    attitude = add_dashboard_tile(app, dash_layout.TILE_ATTITUDE, ["roll", "pitch", "yaw"])
    channels = add_dashboard_tile(app, dash_layout.TILE_CHANNELS)

    assert gauge.spec.colspan == 3 and gauge.spec.rowspan == 3
    assert button.spec.colspan == 3 and button.spec.rowspan == 1
    assert attitude.spec.colspan == 4 and attitude.spec.rowspan == 4
    assert channels.spec.colspan == 4 and channels.spec.rowspan == 6

    app._dashboard_persist()
    restored = dash_layout.DashboardLayout.from_json(app._panel_state["dashboard"])
    assert restored is not None
    persisted = {tile.type: tile for tile in restored.workspaces[0].tiles}
    assert persisted[dash_layout.TILE_GAUGE].bindings == ["flow_height"]
    assert persisted[dash_layout.TILE_BUTTON].options["command"] == "FLOW ZERO"
    assert persisted[dash_layout.TILE_ATTITUDE].bindings == ["roll", "pitch", "yaw"]
    assert persisted[dash_layout.TILE_CHANNELS].bindings == []


def test_gauge_uses_channel_table_range_and_accepts_an_empty_new_card(app) -> None:
    load_schema(app)
    # "添加组件"必然先创建空绑定卡；这里不能因为 `[0]` 崩掉，属性对话框才有机会
    # 让用户选择通道。
    gauge = add_dashboard_tile(app, dash_layout.TILE_GAUGE)
    assert "请选择" in gauge.value_var.get()

    gauge.spec.bindings = ["flow_height"]
    app._dashboard_apply_properties(gauge.spec)
    gauge = next(tile for tile in tiles_of(app, dash_layout.TILE_GAUGE))
    assert gauge.canvas.itemcget(gauge._low, "text") == "0"
    assert gauge.canvas.itemcget(gauge._high, "text") == "5"

    app._dashboard_on_binary_frame(0x2230, telem_frame({"flow_height": 0.612}))
    gauge.refresh()
    assert "0.612" in gauge.value_var.get()
    assert abs(float(gauge.canvas.itemcget(gauge._value_arc, "extent"))) > 0.0


def test_command_button_uses_the_existing_validation_gate_for_push_and_toggle(app,
                                                                                monkeypatch) -> None:
    """组件绝不能直接拿 transport；拒绝时一条命令都不许出面板。"""
    load_schema(app)
    button = add_dashboard_tile(
        app, dash_layout.TILE_BUTTON,
        options={"label": "清零", "command": "FLOW ZERO"},
    )
    checks: list[str] = []
    monkeypatch.setattr(
        app, "_validation_command_allowed",
        lambda command: checks.append(command) or command != "FLOW ZERO",
    )
    app.transport.lines.clear()
    button._on_click()
    assert checks == ["FLOW ZERO"]
    assert app.transport.lines == []
    assert button.state_var.get().startswith("已拦截")

    button.spec.options.update({
        "mode": "toggle", "command_on": "TELEM STREAM on", "command_off": "TELEM STREAM off",
    })
    app._dashboard_apply_properties(button.spec)
    button = next(tile for tile in tiles_of(app, dash_layout.TILE_BUTTON))
    monkeypatch.setattr(app, "_validation_command_allowed", lambda _command: True)
    # 属性应用会按新工作区重发掩码；后面只核对按钮自身实际发送的两条命令。
    app.transport.lines.clear()
    button._on_click()
    button._on_click()
    assert app.transport.lines == ["TELEM STREAM on", "TELEM STREAM off"]


def test_button_properties_expose_all_command_options(app) -> None:
    """不能只在 TileSpec 里藏字段；用户必须能在真实属性框填写按钮/开关命令。"""
    load_schema(app)
    spec = dash_layout.TileSpec(dash_layout.TILE_BUTTON)
    applied: list[dash_layout.TileSpec] = []
    dialog = dashboard_page.TilePropertiesDialog(
        app.dashboard_tab, spec, channels=app.dashboard_schema.ordered(),
        tile_classes=dash_tiles.TILE_CLASSES, on_apply=applied.append,
    )
    dialog.update_idletasks()
    assert {"label", "command", "command_on", "command_off", "mode"} <= set(dialog.option_vars)
    dialog.option_vars["label"].set("光流清零")
    dialog.option_vars["command"].set("FLOW ZERO")
    dialog.option_vars["command_on"].set("TELEM STREAM on")
    dialog.option_vars["command_off"].set("TELEM STREAM off")
    dialog.option_vars["mode"].set("toggle")
    dialog._apply()

    assert applied == [spec]
    assert spec.options == {
        "label": "光流清零",
        "command": "FLOW ZERO",
        "command_on": "TELEM STREAM on",
        "command_off": "TELEM STREAM off",
        "mode": "toggle",
    }


def test_attitude_and_channel_list_render_live_schema_values(app) -> None:
    load_schema(app)
    add_dashboard_tile(app, dash_layout.TILE_ATTITUDE, ["roll", "pitch", "yaw"])
    channels = add_dashboard_tile(app, dash_layout.TILE_CHANNELS)
    # 第二次新增会整页重建，前一张卡的 Tk 控件已销毁；取回当前实例再刷新。
    attitude = next(tile for tile in tiles_of(app, dash_layout.TILE_ATTITUDE))
    assert channels.tree.item("roll", "text") == "roll"
    assert channels.tree.item("roll", "image"), "每行需要带一个彩色点图片"
    assert "roll" in channels._colour_dots, "必须持有图片引用，避免 Tk GC 后颜色点消失"

    app._dashboard_on_binary_frame(
        0x2230, telem_frame({"roll": 12.5, "pitch": -3.0, "yaw": 90.0})
    )
    attitude.refresh()
    channels._last_refresh = 0.0
    channels.refresh()
    assert "roll +12.5" in attitude.readout_var.get()
    assert channels.tree.set("roll", "value") == "+12.5"
    assert len(attitude.canvas.coords(attitude._horizon)) == 4
    assert len(attitude.canvas.coords(attitude._compass)) == 4


def test_layout_json_import_export_round_trip_and_rejects_bad_input(app, tmp_path, monkeypatch) -> None:
    """导入/导出走 UI 的文件选择路径，坏 JSON 不能毁掉当前工作区。"""
    load_schema(app)
    add_dashboard_tile(app, dash_layout.TILE_GAUGE, ["flow_height"])
    layout_path = tmp_path / "状态监视布局.json"
    monkeypatch.setattr(
        dashboard_page.filedialog, "asksaveasfilename", lambda **_kwargs: str(layout_path)
    )
    app._dashboard_export_layout()
    exported = layout_path.read_text(encoding="utf-8")
    assert dash_layout.DashboardLayout.loads(exported) is not None

    app.dashboard_layout.active_workspace().tiles.clear()
    app._dashboard_rebuild_tiles()
    monkeypatch.setattr(
        dashboard_page.filedialog, "askopenfilename", lambda **_kwargs: str(layout_path)
    )
    app._dashboard_import_layout()
    assert any(tile.spec.type == dash_layout.TILE_GAUGE for tile in app.dashboard_tiles)
    before = app.dashboard_layout.dumps()
    assert not app._dashboard_import_layout_text("{not json")
    assert app.dashboard_layout.dumps() == before


# ---------------------------------------------------------------- 统计与录制


def test_stats_bar_reports_link_state(app) -> None:
    load_schema(app)
    app._handle_board_line(
        "TELEM STREAM stream=1 rate=40 mask=00000000000000FF refresh=1 fmt=bin "
        "sink=auto active=usb seq=12 frames=12 drop=3 usb_lost=0"
    )
    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll": 1.0}, seq=0))
    app._dashboard_on_binary_frame(0x2230, telem_frame({"roll": 1.0}, seq=5))
    app._dashboard_refresh_stats()

    assert app.dashboard_stat_vars["sink"].get() == "usb"
    assert app.dashboard_stat_vars["drop"].get() == "3"
    assert "1 次" in app.dashboard_stat_vars["gap"].get()
    assert app.dashboard_stat_vars["schema"].get() == f"{schema_hash():08X}"


def test_recording_writes_a_header_with_names_and_hash(app, tmp_path, monkeypatch) -> None:
    load_schema(app)
    monkeypatch.setattr(dashboard_page, "TELEMETRY_DIR", tmp_path)
    monkeypatch.setattr(dashboard_page, "dated_directory", lambda root: root / "2026-09-03")

    app.transport.lines.clear()
    app.transport.frames.clear()
    app._dashboard_toggle_record()
    assert app.dashboard_record_handle is not None
    # 录制是纯本地动作，不许顺手改固件的流配置。
    assert app.transport.lines == []
    assert app.transport.frames == []

    app._dashboard_on_binary_frame(
        0x2230, telem_frame({"roll": 1.0, "yaw": 3.0}, t_us=1000)
    )
    app._dashboard_flush_record()
    app._dashboard_stop_record()

    text = app.dashboard_record_path.read_text(encoding="utf-8").splitlines()
    assert text[0] == f"# schema_hash={schema_hash():08X}"
    assert text[1].startswith("t_s,roll,pitch,yaw,")
    # 本帧没带的通道留空而不是补 0：8 路通道 = 8 个单元格，只有 roll 和 yaw 有值。
    assert text[2] == "0.001000,1,,3,,,,,"
    assert len(text[2].split(",")) == 1 + app.dashboard_schema.channel_count


def test_recording_lands_under_the_project_data_root() -> None:
    source = (ROOT / "tools" / "panel_lib" / "pages" / "dashboard.py").read_text(
        encoding="utf-8"
    )
    assert "project_paths import TELEMETRY_DIR" in source
    assert "dated_directory(TELEMETRY_DIR)" in source


# ---------------------------------------------------------------- 性能


def test_total_redraw_benchmark(app) -> None:
    """规划 §2.8：4 波形 × 4 曲线 × 10 k 点 + 20 张卡，一次总重绘 ≤10 ms。"""
    load_schema(app)
    workspace = app.dashboard_layout.active_workspace()
    workspace.tiles.clear()
    for index in range(4):
        workspace.tiles.append(dash_layout.TileSpec(
            dash_layout.TILE_WAVE, (index % 2) * 6, (index // 2) * 5, 6, 5,
            ["roll", "pitch", "yaw", "vel_est_x"],
        ))
    for index in range(20):
        workspace.tiles.append(dash_layout.TileSpec(
            dash_layout.TILE_VALUE, (index % 4) * 3, 10 + (index // 4) * 2, 3, 2,
            ["flow_height"],
        ))
    app._dashboard_rebuild_tiles()

    # 每通道灌满 10 k 个样本，直接写环形缓冲（走帧要 10 k 次解码，测的就不是重绘了）。
    times = np.linspace(0.0, 60.0, 10000, dtype=np.float64)
    for name in ("roll", "pitch", "yaw", "vel_est_x", "flow_height"):
        index = NAME_TO_INDEX[name]
        ring = app.dashboard_ring
        ring._time[index, :] = np.resize(times, ring.capacity)
        ring._value[index, :] = np.resize(
            np.sin(times * (index + 1)).astype(np.float32), ring.capacity
        )
        ring._used[index] = ring.capacity
        ring._head[index] = 0

    for tile in app.dashboard_tiles:
        tile.refresh()
    app.update_idletasks()

    samples = []
    for _ in range(11):
        start = time.perf_counter()
        for tile in app.dashboard_tiles:
            tile.refresh()
        samples.append(time.perf_counter() - start)
    samples.sort()
    median = samples[len(samples) // 2]
    assert median <= 0.010, (
        f"median total redraw {median * 1000:.2f} ms "
        f"(best {samples[0] * 1000:.2f}, worst {samples[-1] * 1000:.2f})"
    )


def test_dashboard_defers_tile_painting_while_a_resize_is_active(app, monkeypatch) -> None:
    """拖边框时仍收数据，但不应和卡片绘图争抢 Tk 主线程。"""
    app.dashboard_tab_visible = True
    calls: list[str] = []
    for index, tile in enumerate(app.dashboard_tiles):
        monkeypatch.setattr(tile, "refresh", lambda index=index: calls.append(str(index)))
    monkeypatch.setattr(app, "after", lambda *_args, **_kwargs: None)

    app.dashboard_resize.render_suspended = True
    app._dashboard_render_tick()
    assert calls == []

    app.dashboard_resize.render_suspended = False
    app._dashboard_render_tick()
    assert len(calls) == len(app.dashboard_tiles)
