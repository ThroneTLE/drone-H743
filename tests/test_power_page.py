"""电源页（R-PWR-1）：实时区走遥测推送、诊断区按需问答、两者共用一条链路。

合并了原 `test_battery_page.py`（真实固件字节 → transport → Rx 分发 → 页面）与
`test_current_monitor_page.py`（`CURRENT` 行解析、会话/代次防护、无效不当零）的
覆盖面，并加上合并本身带来的新契约：掩码并集、流开关引用计数、独占抑制、sink
分发的来源身份，以及 NaN 的三条路径。

协议夹具的出处：
* 电池二进制回包 —— `tests/test_battery_runtime.py::firmware_battery`，由真实
  固件 C 代码编译出来再发出来的那一帧，不是手打的。
* `CURRENT ...` 行 —— 真实固件 `APP_Current_Report()` 编译执行后的输出。
* `batt_v` / `batt_i` 的通道元数据 —— R-PWR-1 契约表（固件侧由并行工单落地）。
"""

import math
import shutil
import struct
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib import rx_dispatch
from tools.panel_lib.connection_state import receive_context
from tools.panel_lib.pages import power as power_page
from tools.panel_lib.proto import (
    PROTO_DIR_FROM_FC,
    PROTO_MSG_BATTERY,
    PROTO_REQ_STATUS,
)
from tools.panel_lib.telem_stream import TelemSchema
from tools.panel_lib.telem_subscription import (
    TELEM_EXCLUSIVE_FLIGHT_LOG,
    TELEM_OWNER_DASHBOARD,
    TELEM_OWNER_POWER,
)
from tools.panel_lib.transport import TransportBase, build_proto_frame

from test_battery_runtime import firmware_battery                     # noqa: F401


ROOT = Path(__file__).resolve().parents[1]
TELEM_FRAME_FUNCTION = 0x2230


@pytest.fixture(scope="module")
def firmware_current_line(tmp_path_factory):
    """真实 `APP_Current_Report()` 的输出，不是手打的一行。

    从原 `test_current_monitor_page.py` 原样搬过来：固件从没发过的字段名不会
    报错，只会让解析永远匹配不上，所以夹具必须钉在编译执行的真实输出上。
    """
    d = tmp_path_factory.mktemp("current-report")
    (d / "app_control.h").write_text("void APP_Control_QueueText(const char*,...);\n")
    (d / "report.c").write_text(r'''
#include "app_current.h"
#include "bsp_current.h"
#include <stdio.h>
#include <stdarg.h>
uint32_t BSP_Critical_Enter(void){return 0;}
void BSP_Critical_Exit(uint32_t x){(void)x;}
uint32_t SVC_Timestamp_Ms(void){return 100;}
BSP_CurrentStatus BSP_Current_Init(void){return BSP_CURRENT_OK;}
BSP_CurrentStatus BSP_Current_Read(uint32_t *raw){*raw=12000;return BSP_CURRENT_OK;}
void APP_Control_QueueText(const char *fmt,...){va_list a;va_start(a,fmt);vprintf(fmt,a);va_end(a);}
int main(void){APP_Current_Init();APP_Current_Step();APP_Current_Report();return 0;}
''')
    cmd = [shutil.which("gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2"]
    for inc in (d, ROOT / "App/Inc", ROOT / "BSP/Inc", ROOT / "Driver/Inc", ROOT / "Services/Inc"):
        cmd += ["-I", str(inc)]
    cmd += [str(d / "report.c"), str(ROOT / "App/Src/app_current.c"),
            str(ROOT / "Driver/Src/drv_current.c"),
            # 2026-09-21：app_current.c 起用块平均后多了这个依赖。
            str(ROOT / "Driver/Src/drv_current_filter.c"),
            "-lm", "-o", str(d / "report.exe")]
    subprocess.run(cmd, check=True, capture_output=True)
    return subprocess.check_output([str(d / "report.exe")], text=True).strip()

# R-PWR-1 契约表：复用退役的 reserved_7 / reserved_8 槽位，名字才是契约。
SCHEMA_LINES = [
    "TELEM ver=4 n=9 rate=40 page=9 hash=00000000 frame=body_flu contract=1",
    "TELEM CH idx=0 name=roll unit=deg min=-180.000 max=180.000 grp=attitude param=-",
    "TELEM CH idx=1 name=pitch unit=deg min=-90.000 max=90.000 grp=attitude param=-",
    "TELEM CH idx=2 name=yaw unit=deg min=-180.000 max=180.000 grp=attitude param=-",
    "TELEM CH idx=3 name=vel_est_x unit=m/s min=-5.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=4 name=vel_est_y unit=m/s min=-5.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=5 name=flow_height unit=m min=0.000 max=5.000 grp=nav param=-",
    "TELEM CH idx=6 name=rate_roll_kd unit=kg.m^2 min=0.000 max=10.000 grp=gain param=coax.rate_roll_kd",
    "TELEM CH idx=7 name=batt_v unit=V min=0.000 max=30.000 grp=power param=-",
    "TELEM CH idx=8 name=batt_i unit=A min=-10.000 max=60.000 grp=power param=-",
    "TELEM PAGE from=0 count=9 next=-1",
]
NAME_TO_INDEX = {"roll": 0, "pitch": 1, "yaw": 2, "vel_est_x": 3, "vel_est_y": 4,
                 "flow_height": 5, "rate_roll_kd": 6, "batt_v": 7, "batt_i": 8}


def schema_hash() -> int:
    schema = TelemSchema()
    for line in SCHEMA_LINES:
        schema.feed_line(line)
    return schema.computed_hash()


def telem_frame(values: dict, *, seq: int = 0, t_us: int = 0) -> bytes:
    indexed = {NAME_TO_INDEX[name]: value for name, value in values.items()}
    mask = 0
    for index in indexed:
        mask |= 1 << index
    head = struct.pack("<BBHIIHHQ", 2, 1, seq & 0xFFFF, schema_hash(), t_us, 0, 0, mask)
    return head + b"".join(struct.pack("<f", indexed[i]) for i in sorted(indexed))


class Wire(TransportBase):
    """真实 `TransportBase` 的收发两端，一个字节都不出进程。"""

    is_connected = True
    connection_generation = 1
    _stamp_received = True

    def __init__(self, rx):
        super().__init__(rx)
        self.commands = []
        self.frames = []
        self.binary_sink = None
        self.binary_sink_with_context = False

    def set_binary_sink(self, sink, *, with_context=False):
        """Capture only the public registration contract for test injection."""
        super().set_binary_sink(sink, with_context=with_context)
        self.binary_sink = sink
        self.binary_sink_with_context = bool(with_context)

    def start(self, *_a):
        self.is_connected = True

    def stop(self):
        self.is_connected = False

    def send_line(self, line):
        self.commands.append(line)
        return True

    def send_frame(self, function, payload=b""):
        self.frames.append((int(function), bytes(payload)))
        return True


@pytest.fixture(scope="module")
def app():
    instance = panel.DronePanel()
    yield instance
    instance.destroy()


@pytest.fixture
def live(app, monkeypatch):
    """干净链路 + 干净工作台会话；页面回到"刚打开"的状态。"""
    monkeypatch.setattr(type(app), "_save_panel_state", lambda self: None)
    app._dashboard_reset_session()
    app.telem_registry = type(app.telem_registry)()
    app.telem_fanout = type(app.telem_fanout)()
    app.telem_fanout.attach(TELEM_OWNER_DASHBOARD, app._dashboard_on_binary_frame)
    app.transport = Wire(app.rx_queue)
    page = app.power_page
    page.subscribed = False
    page.telem_stamp = None
    page.dirty = False
    page.session = None
    page._connection()
    page.nonce = 41
    select_power(app)
    return app


def select_power(app):
    app.notebook.select(app.sensor_group_tab)
    app.sensor_notebook.select(app.power_tab)
    app.update_idletasks()


def leave_power(app):
    """离开电源页，并且**不要**落到工作台上——否则工作台自己会把流打开。"""
    app.notebook.select(app.calibration_group_tab)
    app.update_idletasks()


def load_schema(app):
    for line in SCHEMA_LINES:
        app._handle_board_line(line)
    app.update_idletasks()


def push(app, values, *, transport=None, seq=0, t_us=0):
    """把一帧遥测送进 transport 唯一那个 sink 槽（工作台开流时挂上去的那个）。"""
    source = app.transport if transport is None else transport
    sink = source.binary_sink
    assert sink is not None, "流没开，sink 没挂上"
    payload = telem_frame(values, seq=seq, t_us=t_us)
    if source.binary_sink_with_context:
        sink(TELEM_FRAME_FUNCTION, payload, receive_context(source))
    else:
        sink(TELEM_FRAME_FUNCTION, payload)


def feed_binary(app, data, *, age=0, generation=None):
    context = receive_context(app.transport, received_at=time.monotonic() - age,
                              generation=generation)
    buffer = bytearray()
    for i in range(0, len(data), 7):
        buffer.extend(data[i:i + 7])
        app.transport._consume_buffer(buffer, context=context)
    rx_dispatch.drain_rx(app, 100, 5, 50)


def feed_line(app, line, *, age=0, generation=None, framed=False):
    from tools.panel_lib.connection_state import ReceivedMessage
    context = receive_context(app.transport, received_at=time.monotonic() - age,
                              generation=generation)
    payload = ("proto", panel.PROTO_MSG_TEXT_LINE, line) if framed else line
    app.rx_queue.put(ReceivedMessage(payload, context))
    rx_dispatch.drain_rx(app, 1000, 5, 50)


def mutate(wire, nonce, *, flags=None, cells=None, low=None, recover=None, arm=None):
    payload = bytearray(wire[8:-1])
    struct.pack_into("<I", payload, 4, nonce)
    if flags is not None:
        payload[1] = flags
    if cells is not None:
        payload[2] = cells
    if low is not None:
        struct.pack_into("<I", payload, 28, low)
    if recover is not None:
        struct.pack_into("<I", payload, 32, recover)
    if arm is not None:
        struct.pack_into("<I", payload, 56, arm)
    return build_proto_frame(PROTO_DIR_FROM_FC, PROTO_MSG_BATTERY, payload)


def prime(app, firmware_battery):
    """让诊断区拿到一份真实回包，串数/阈值因此可用。"""
    page = app.power_page
    page.fetched_for = None
    page._tick()
    assert page.pending == "query"
    page.nonce = page.nonce
    feed_binary(app, mutate(firmware_battery, page.nonce))
    return page


# ---------------------------------------------------------------- 合并本身


def test_the_two_sensor_pages_became_one(app):
    labels = [app.sensor_notebook.tab(t, "text") for t in app.sensor_notebook.tabs()]
    assert labels.count(power_page.POWER_TAB_TEXT) == 1
    assert "电流计" not in labels and "电池电压" not in labels
    assert not hasattr(app, "current_tab") and not hasattr(app, "battery_tab")
    assert not hasattr(app, "current_page") and not hasattr(app, "battery_page")


def test_current_is_displayed_exactly_once(live, firmware_battery, firmware_current_line):
    """合并的验收点：同一个物理量不能在一页上出现两个可能打架的读数。"""
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    feed_line(app, firmware_current_line)
    app._telem_apply()
    push(app, {"batt_v": 12.0, "batt_i": 47.393})
    page._tick()

    amps = [name for name, var in page.fields.items() if var.get().endswith(" A")]
    assert amps == [], f"诊断区不该再有第二个电流读数：{amps}"
    assert page.current_var.get() == "47.393 A"


# ---------------------------------------------------------------- 实时区


def test_the_live_region_follows_pushed_frames_not_polling(live, firmware_battery):
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    push(app, {"batt_v": 12.6, "batt_i": 4.0})
    page._tick()

    assert page.voltage_var.get() == "12.600 V"
    assert page.current_var.get() == "4.000 A"
    assert page.cell_var.get() == "4.200 V"          # 12.6 / 3S
    assert page.power_var.get() == "50.4 W"
    assert "实时推送中" in page.live_state_var.get()

    push(app, {"batt_v": 11.1, "batt_i": 10.0})
    page._tick()
    assert page.voltage_var.get() == "11.100 V"
    assert page.power_var.get() == "111.0 W"


def test_canonical_x_frame_reaches_the_real_power_page(live, firmware_battery):
    """Exercise framing/CRC/transport fanout and the actual page together."""
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    frame = build_proto_frame(
        PROTO_DIR_FROM_FC, TELEM_FRAME_FUNCTION,
        telem_frame({"batt_v": 12.4, "batt_i": 2.5}, seq=7, t_us=175000))
    context = receive_context(app.transport)
    buffer = bytearray()
    for offset in range(0, len(frame), 5):
        buffer.extend(frame[offset:offset + 5])
        app.transport._consume_buffer(buffer, context=context)
    page._tick()
    assert not buffer
    assert page.voltage_var.get() == "12.400 V"
    assert page.current_var.get() == "2.500 A"
    assert page.power_var.get() == "31.0 W"


def test_no_command_is_ever_sent_on_a_timer(live, firmware_battery, monkeypatch):
    """两个页面原来的 2 秒轮询定时器必须已经拆掉。"""
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    app.transport.commands.clear()
    app.transport.frames.clear()

    clock = [time.monotonic()]
    monkeypatch.setattr(power_page, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    for _ in range(120):                     # 120 × 0.25 s = 30 s
        clock[0] += 0.25
        page._tick()

    assert [c for c in app.transport.commands if c.startswith("BATTERY")] == []
    assert [f for f in app.transport.frames if f[0] == PROTO_REQ_STATUS] == []
    assert not hasattr(page, "auto") and not hasattr(page, "auto_var")


def test_the_diagnostics_are_fetched_once_on_open_and_on_demand(live, firmware_battery):
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    assert any(c.startswith("BATTERY? ") for c in app.transport.commands)
    assert (PROTO_REQ_STATUS, b"STATUS?") in app.transport.frames

    app.transport.commands.clear()
    app.transport.frames.clear()
    for _ in range(5):
        page._tick()
    assert app.transport.commands == [] and app.transport.frames == []

    page.refresh_diagnostics()
    assert any(c.startswith("BATTERY? ") for c in app.transport.commands)
    assert (PROTO_REQ_STATUS, b"STATUS?") in app.transport.frames


def test_a_nan_reading_shows_a_dash_with_a_reason_not_a_zero(live, firmware_battery):
    """固件对无效/过期发 NaN。显示 0 会让"没读到"看起来像"真的是 0"。"""
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    invalid = mutate(firmware_battery, page.nonce, flags=0x00)
    page.refresh_diagnostics()
    feed_binary(app, mutate(invalid, page.nonce, flags=0x00))
    push(app, {"batt_v": float("nan"), "batt_i": float("nan")})
    page._tick()

    assert page.voltage_var.get() == "—"
    assert page.current_var.get() == "—"
    assert page.power_var.get() == "—"
    assert "无效" in page.live_state_var.get()
    assert "ADC状态" in page.live_state_var.get()
    # 半边 NaN 也一样：功率不能拿一个有效值乘一个未知数。
    push(app, {"batt_v": 12.0, "batt_i": float("nan")})
    page._tick()
    assert page.voltage_var.get() == "12.000 V"
    assert page.current_var.get() == "—" and page.power_var.get() == "—"


def test_a_stale_stream_stops_showing_the_last_value(live, firmware_battery, monkeypatch):
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    push(app, {"batt_v": 12.6, "batt_i": 4.0})
    page._tick()
    assert page.voltage_var.get() == "12.600 V"

    clock = [time.monotonic() + 60.0]
    monkeypatch.setattr(power_page, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    page._tick()
    assert page.voltage_var.get() == "—"
    assert "过期" in page.live_state_var.get()


def test_frames_from_the_previous_link_never_reach_the_live_region(live, firmware_battery):
    """换连接代次之后，上一条链路的最后一帧不能算成新会话的数据。"""
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    origin = app.transport
    sink = origin.binary_sink
    push(app, {"batt_v": 12.6, "batt_i": 4.0})
    page._tick()
    assert page.voltage_var.get() == "12.600 V"

    origin.connection_generation += 1
    page._tick()
    assert page.voltage_var.get() == "—"
    # 旧闭包还在路上送来一帧：身份对不上，不许复活。
    sink(TELEM_FRAME_FUNCTION, telem_frame({"batt_v": 3.3, "batt_i": 99.0}))
    page._tick()
    assert page.voltage_var.get() == "—"
    assert page.current_var.get() == "—"


def test_disconnecting_clears_the_page(live, firmware_battery):
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    push(app, {"batt_v": 12.6, "batt_i": 4.0})
    page._tick()
    app.transport.is_connected = False
    page._tick()
    assert page.voltage_var.get() == "—" and page.live_state_var.get() == "未连接"
    assert page.fields["v_raw"].get() == "—"
    assert page.battery is None


# ---------------------------------------------------------------- 订阅仲裁


def test_the_page_subscribes_only_while_visible(live):
    app = live
    load_schema(app)
    page = app.power_page
    page._tick()
    assert page.subscribed
    assert app.telem_registry.is_registered(TELEM_OWNER_POWER)
    mask = app.telem_registry.mask(app.dashboard_index_by_name)
    assert mask & (1 << NAME_TO_INDEX["batt_v"])
    assert mask & (1 << NAME_TO_INDEX["batt_i"])

    leave_power(app)
    page._tick()
    assert not page.subscribed
    assert not app.telem_registry.is_registered(TELEM_OWNER_POWER)
    after = app.telem_registry.mask(app.dashboard_index_by_name)
    assert not after & (1 << NAME_TO_INDEX["batt_v"])
    assert not after & (1 << NAME_TO_INDEX["batt_i"])


def test_the_mask_is_the_union_and_neither_page_erases_the_other(live):
    """`TELEM MASK` 是整条覆盖的。两个页面各发各的就会互相擦位。

    切页的那一瞬间两边都还持有订阅，这是最容易互相擦位的时刻，所以按那个状态
    断言：工作台可见（实时通道在）+ 电源页订阅还没撤（batt_v/batt_i 也在）。
    """
    app = live
    load_schema(app)
    app.notebook.select(app.dashboard_tab)
    app.update_idletasks()
    app._dashboard_poll_tick(time.monotonic())
    app.power_page._sync_subscription(True)

    sent = [c for c in app.transport.commands if c.startswith("TELEM MASK ")]
    assert sent
    mask = int(sent[-1].split()[-1], 16)
    for name in ("roll", "pitch", "yaw", "rate_roll_kd", "batt_v", "batt_i"):
        assert mask & (1 << NAME_TO_INDEX[name]), name

    # 电源页退订只拿走自己那两位，工作台的一位都不许掉。
    app.power_page._sync_subscription(False)
    after = app.telem_registry.mask(app.dashboard_index_by_name)
    for name in ("roll", "pitch", "yaw", "rate_roll_kd"):
        assert after & (1 << NAME_TO_INDEX[name]), name
    assert not after & (1 << NAME_TO_INDEX["batt_v"])
    assert not after & (1 << NAME_TO_INDEX["batt_i"])


def test_the_power_page_alone_keeps_the_stream_and_the_sink_up(live):
    """工作台不可见时也必须有人挂 sink，否则订阅了也收不到帧。"""
    app = live
    load_schema(app)
    page = app.power_page
    select_power(app)
    app._dashboard_poll_tick(time.monotonic())      # 工作台不可见 → 自己不要流
    page._tick()

    assert "TELEM STREAM on" in app.transport.commands
    assert app.transport.binary_sink is not None
    push(app, {"batt_v": 12.0, "batt_i": 1.0})
    page._tick()
    assert page.voltage_var.get() == "12.000 V"

    app.transport.commands.clear()
    leave_power(app)
    page._tick()
    app._dashboard_poll_tick(time.monotonic())
    assert "TELEM STREAM off" in app.transport.commands
    assert "TELEM STREAM on" not in app.transport.commands
    assert app.transport.binary_sink is None


def test_the_stream_stays_up_while_either_page_wants_it(live):
    app = live
    load_schema(app)
    page = app.power_page
    select_power(app)
    page._tick()
    assert "TELEM STREAM on" in app.transport.commands
    app.transport.commands.clear()

    # 工作台可见 → 两个订阅者；电源页离开不能把流关掉。
    app.notebook.select(app.dashboard_tab)
    app.update_idletasks()
    app._dashboard_poll_tick(time.monotonic())
    page._tick()
    assert "TELEM STREAM off" not in app.transport.commands
    assert app.transport.binary_sink is not None


def test_a_flight_log_export_wins_over_the_reference_count(live):
    """日志导出要独占链路。"还有页面在订阅"不是把流顶回去的理由。"""
    app = live
    load_schema(app)
    page = app.power_page
    select_power(app)
    page._tick()
    assert app.transport.binary_sink is not None

    app._telem_suppress(TELEM_EXCLUSIVE_FLIGHT_LOG, "日志导出独占当前串口")
    assert app.transport.binary_sink is None
    assert app.transport.commands[-1] == "TELEM STREAM off"

    app.transport.commands.clear()
    for _ in range(5):
        page._tick()
        app._dashboard_poll_tick(time.monotonic())
    assert "TELEM STREAM on" not in app.transport.commands
    assert app.transport.binary_sink is None

    app._telem_release_exclusive(TELEM_EXCLUSIVE_FLIGHT_LOG)
    assert app.transport.binary_sink is not None
    assert "TELEM STREAM on" in app.transport.commands


def test_only_the_visible_pages_realtime_channels_are_on_the_wire(live):
    """只打开电源页时，线上不该还在跑工作台摆的那一堆波形通道。"""
    app = live
    select_power(app)
    load_schema(app)
    app._dashboard_poll_tick(time.monotonic())     # 工作台不可见
    app.power_page._tick()

    mask = app.telem_registry.mask(app.dashboard_index_by_name)
    assert mask & (1 << NAME_TO_INDEX["batt_v"])
    assert mask & (1 << NAME_TO_INDEX["batt_i"])
    # 参数通道恒在：滑块靠每秒一次的全量刷新帧喂回来。
    assert mask & (1 << NAME_TO_INDEX["rate_roll_kd"])
    for name in ("roll", "pitch", "yaw", "vel_est_x", "vel_est_y", "flow_height"):
        assert not mask & (1 << NAME_TO_INDEX[name]), name


def test_a_refused_mask_is_retried_instead_of_being_swallowed(live, monkeypatch):
    """掩码曾经是一次性边沿：发不出去也当发过了，之后并集不再变就永不重发。

    这条路真实可达——日志导出持有串口租约时 `serial_session._enqueue()` 对非租约
    的非停止命令一律返回 False。表现极具误导性：别的通道照常喂帧，只有自己订阅
    的那两路永远不在任何一帧里，于是被当成 ADC 故障去查。
    """
    app = live
    select_power(app)
    load_schema(app)
    refused = {"on": True}
    real_send = app.transport.send_line

    def send(line):
        real_send(line)
        return not (line.startswith("TELEM MASK ") and refused["on"])

    monkeypatch.setattr(app.transport, "send_line", send)
    app.power_page._tick()
    app._dashboard_poll_tick(time.monotonic())
    assert [c for c in app.transport.commands if c.startswith("TELEM MASK ")]
    wanted = app._dashboard_mask()
    assert wanted & (1 << NAME_TO_INDEX["batt_v"])
    assert app.telem_mask_sent is None or app.telem_mask_sent[0] != wanted

    app.transport.commands.clear()
    app._dashboard_poll_tick(time.monotonic())
    assert [c for c in app.transport.commands if c.startswith("TELEM MASK ")], "被拒之后必须重试"

    refused["on"] = False
    app.transport.commands.clear()
    app._dashboard_poll_tick(time.monotonic())
    assert [c for c in app.transport.commands if c.startswith("TELEM MASK ")]
    assert app.telem_mask_sent is not None and app.telem_mask_sent[0] == wanted

    # 成功之后就不再每拍重发——上行本来就紧张。
    app.transport.commands.clear()
    for _ in range(5):
        app._dashboard_poll_tick(time.monotonic())
    assert [c for c in app.transport.commands if c.startswith("TELEM MASK ")] == []


def test_a_silent_stream_is_reopened_because_the_board_may_have_reset(live, monkeypatch):
    """数传口上飞控自己复位时，地面端串口**从不掉线**，代次也不变。

    固件那头 `APP_TelemStream_Init()` 已经把流关了、掩码还原成默认；主机这边
    却还以为"流开着、sink 挂着、代次相等"，于是一个字节都不发。合并之前这条是
    被两页的 2 秒轮询顺手兜住的，轮询删掉就得显式写出来。
    """
    app = live
    select_power(app)
    load_schema(app)
    page = app.power_page
    page._tick()
    assert "TELEM STREAM on" in app.transport.commands
    push(app, {"batt_v": 12.0, "batt_i": 1.0})

    base = time.monotonic()
    app._dashboard_poll_tick(base)
    app.transport.commands.clear()
    # 帧停了，但 transport 一切正常：is_connected 为真、代次不变。
    app._dashboard_poll_tick(base + 10.0)
    app._dashboard_poll_tick(base + 10.1)
    assert "TELEM STREAM on" in app.transport.commands
    assert [c for c in app.transport.commands if c.startswith("TELEM MASK ")]
    assert app.transport.binary_sink is not None

    # 限流：不许按链路速度空转。
    app.transport.commands.clear()
    for offset in (10.2, 10.3, 10.4):
        app._dashboard_poll_tick(base + offset)
    assert app.transport.commands.count("TELEM STREAM on") == 0


def test_a_channel_that_fell_out_of_the_mask_is_not_shown_as_live(live, firmware_battery):
    """帧还在来，但 batt_v 已经不在里面了：不能把复位前的旧电压当成实时值。"""
    app = live
    load_schema(app)
    page = prime(app, firmware_battery)
    push(app, {"batt_v": 12.6, "batt_i": 4.0})
    page._tick()
    assert page.voltage_var.get() == "12.600 V"

    # 固件复位后掩码回到默认：别的通道继续喂帧（时间戳照常前进），电源两路缺席。
    for step in range(1, 6):
        push(app, {"roll": 1.0 * step}, seq=step, t_us=step * 1_000_000)
    page._tick()
    assert page.voltage_var.get() == "—" and page.current_var.get() == "—"
    assert "不在遥测帧里" in page.live_state_var.get()


def test_the_open_page_fetch_retries_after_a_refused_link(live, monkeypatch):
    """链路被占用时那一次"打开页面取一次"不能就这么丢了，也不能变成轮询。"""
    app = live
    load_schema(app)
    page = app.power_page
    monkeypatch.setattr(app.transport, "send_line", lambda line: False)
    clock = [time.monotonic()]
    monkeypatch.setattr(power_page, "time", SimpleNamespace(monotonic=lambda: clock[0]))

    page._tick()
    assert page.fetched_for is None and "未发出" in page.notice.get()
    attempts = len(app.transport.frames)
    for _ in range(8):                        # 2 s 内不再试
        clock[0] += 0.25
        page._tick()
    assert len(app.transport.frames) == attempts

    clock[0] += power_page.POWER_FETCH_RETRY_S
    page._tick()
    assert len(app.transport.frames) > attempts


def test_the_sink_fanout_keeps_the_source_transport(live):
    """一个 sink 槽分发给两个消费者，来源身份一次都不许丢。"""
    app = live
    load_schema(app)
    app.power_page._tick()
    origin = app.transport
    sink = origin.binary_sink
    assert set(app.telem_fanout.owners()) == {TELEM_OWNER_DASHBOARD, TELEM_OWNER_POWER}

    seen = []
    app.dashboard_recorder.submit = lambda s, **kw: seen.append(kw) or True
    app.transport = Wire(app.rx_queue)            # 当前 transport 已经换了
    sink(TELEM_FRAME_FUNCTION, telem_frame({"batt_v": 12.0}))

    assert seen and seen[0]["transport"] is origin
    assert app.power_page.telem_stamp.transport is origin


# ---------------------------------------------------------------- 配置问答


def test_real_firmware_bytes_fill_the_diagnostics(live, firmware_battery):
    app = live
    page = prime(app, firmware_battery)
    assert page.fields["cells"].get() == "3 S"
    assert page.fields["v_raw"].get() == "11283"
    assert page.fields["v_valid"].get() == "是"
    assert page.fields["v_adc_status"].get() == "正常"
    assert page.fields["limits"].get() == "3.50 / 3.60 V每节"
    assert page._can_configure()


def test_config_needs_explicit_firmware_ack_and_preserves_new_draft(live, firmware_battery):
    app = live
    page = prime(app, firmware_battery)
    page.low.set(3400)
    assert page.apply_config()
    nonce = page.nonce
    assert app.transport.commands[-1] == f"BATTERY SET {nonce} 3 3400 3600"
    assert page.battery.low_mv == 3500

    page.low.set(3300)
    feed_binary(app, mutate(firmware_battery, nonce, flags=0x15 | 32, low=3400))
    assert page.battery.low_mv == 3400 and page.low.get() == 3300 and page.dirty
    assert "已应用" in page.notice.get()

    assert page.apply_config()
    nonce = page.nonce
    feed_binary(app, mutate(firmware_battery, nonce, flags=0x15 | 64, low=3400))
    assert "未确认" in page.notice.get() and page.battery.low_mv == 3400


def test_a_mismatched_nonce_never_confirms_a_write(live, firmware_battery):
    app = live
    page = prime(app, firmware_battery)
    page.low.set(3400)
    assert page.apply_config()
    feed_binary(app, mutate(firmware_battery, page.nonce + 7, flags=0x15 | 32, low=3400))
    assert page.pending is not None and "已应用" not in page.notice.get()


def test_armed_stale_foreign_and_disconnect_refuse_config(live, firmware_battery):
    app = live
    page = prime(app, firmware_battery)
    page.refresh_diagnostics()
    feed_binary(app, mutate(firmware_battery, page.nonce, arm=2))
    assert not page.apply_config()

    page.refresh_diagnostics()
    feed_binary(app, mutate(firmware_battery, page.nonce), age=4)
    assert not page.apply_config()

    # 别的会话送来的回包连队列边界都过不去，快照不许被它改写。
    page.refresh_diagnostics()
    before = page.battery
    feed_binary(app, mutate(firmware_battery, page.nonce, low=3900, recover=4000),
                generation=0)
    assert page.battery is before and page.pending is not None

    # 换一条链路：上一条的快照必须当场清掉。
    app.transport = Wire(app.rx_queue)
    page._tick()
    assert page.battery is None and page.fields["v_raw"].get() == "—"


def test_missing_firmware_and_bad_packet_do_not_leave_good_fields(live, firmware_battery):
    app = live
    page = prime(app, firmware_battery)
    page.refresh_diagnostics()
    feed_binary(app, b"ERR unknown cmd BATTERY?\r\n")
    assert page.unsupported and "未提供" in page.diag_state_var.get()

    page.refresh_diagnostics()
    feed_binary(app, build_proto_frame(PROTO_DIR_FROM_FC, PROTO_MSG_BATTERY, b"bad"))
    assert page.battery is None and page.fields["v_valid"].get() == "—"


# ---------------------------------------------------------------- CURRENT 行


def test_the_current_line_fills_only_the_diagnostics(live, firmware_current_line):
    app = live
    page = app.power_page
    for framed in (False, True):
        feed_line(app, firmware_current_line, framed=framed)
        assert page.fields["i_raw"].get() == "12000"
        assert page.fields["i_calibrated"].get() == "未校准 · 标称换算"
        assert page.fields["i_source"].get() == "AM32_55A_CURR"
        assert page.fields["i_nominal"].get() == "12.75 mV/A"


@pytest.mark.parametrize("old,new", [(" raw=12000", ""), ("valid=1", "valid=2"),
                                     ("adc_status=0", "adc_status=2"),
                                     ("saturated=0", "saturated=1")])
def test_a_malformed_current_line_clears_the_previous_reading(live, firmware_current_line,
                                                              old, new):
    app = live
    page = app.power_page
    feed_line(app, firmware_current_line)
    feed_line(app, firmware_current_line.replace(old, new))
    assert page.current is None
    assert page.fields["i_raw"].get() == "—"
    assert "无效" in page.diag_state_var.get()


def test_an_invalid_current_measurement_is_not_zero(live, firmware_current_line):
    from tools.panel_lib.current_monitor import parse_current
    app = live
    parsed = parse_current(firmware_current_line)
    line = (firmware_current_line.replace("valid=1", "valid=0")
            .replace(f"current_a={parsed['current_a']:.3f}", "current_a=nan"))
    feed_line(app, line)
    page = app.power_page
    assert page.fields["i_raw"].get() == "12000"
    assert page.fields["i_valid"].get() == "否"
    assert not math.isfinite(page.current["current_a"])


def test_a_current_line_from_a_foreign_session_is_ignored(live, firmware_current_line):
    app = live
    page = app.power_page
    feed_line(app, firmware_current_line, generation=0)
    assert page.current is None


# ---------------------------------------------------------------- 纯函数


@pytest.mark.parametrize("volts,cells,expected", [
    (12.6, 3, 4.2), (0.0, 3, 0.0), (12.6, 0, None), (12.6, None, None),
    (float("nan"), 3, None),
])
def test_average_cell_volts(volts, cells, expected):
    result = power_page.average_cell_volts(volts, cells)
    if expected is None:
        assert result is None
    else:
        assert result == pytest.approx(expected)


@pytest.mark.parametrize("volts,amps,expected", [
    (12.0, 2.0, 24.0), (12.0, -1.0, -12.0), (None, 2.0, None), (12.0, None, None),
    (float("nan"), 2.0, None), (12.0, float("nan"), None),
])
def test_electrical_power_watts(volts, amps, expected):
    result = power_page.electrical_power_watts(volts, amps)
    if expected is None:
        assert result is None
    else:
        assert result == pytest.approx(expected)
