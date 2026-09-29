"""链路仲裁器（`telem_subscription.py`）的纯逻辑契约。

这里一行 Tk、一个 transport 都不碰：仲裁规则本身能被判对错，才谈得上把掩码、
流开关、sink 三件共享资源交给多个页面。页面侧的接线在
`tests/test_power_page.py` 与 `tests/test_dashboard_page.py` 里。
"""

import pytest

from tools.panel_lib.telem_subscription import (
    TELEM_EXCLUSIVE_FLIGHT_LOG,
    TELEM_OWNER_DASHBOARD,
    TELEM_OWNER_POWER,
    TelemBinaryFanout,
    TelemSubscriptionRegistry,
)


INDEX = {"roll": 0, "pitch": 1, "batt_v": 7, "batt_i": 8, "rate_roll_kp": 20}


@pytest.fixture
def registry():
    return TelemSubscriptionRegistry()


def test_the_mask_is_the_union_of_every_subscriber(registry):
    registry.register(TELEM_OWNER_DASHBOARD, ["roll", "pitch", "rate_roll_kp"])
    registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_i"])
    demand = registry.demand(INDEX)
    assert demand.mask == (1 << 0) | (1 << 1) | (1 << 20) | (1 << 7) | (1 << 8)
    assert demand.owners == (TELEM_OWNER_DASHBOARD, TELEM_OWNER_POWER)
    assert demand.missing == ()


def test_unregistering_clears_only_that_subscribers_bits(registry):
    registry.register(TELEM_OWNER_DASHBOARD, ["roll", "pitch"])
    registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_i"])
    assert registry.unregister(TELEM_OWNER_POWER)
    assert registry.mask(INDEX) == (1 << 0) | (1 << 1)
    # 退订之后再退一次不算变化，调用方据此避免重发掩码。
    assert not registry.unregister(TELEM_OWNER_POWER)
    assert not registry.is_registered(TELEM_OWNER_POWER)


def test_a_shared_channel_survives_one_owner_leaving(registry):
    """两个人都要 `batt_v`：走掉一个，位不能跟着掉。"""
    registry.register(TELEM_OWNER_DASHBOARD, ["batt_v"])
    registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_i"])
    registry.unregister(TELEM_OWNER_POWER)
    assert registry.mask(INDEX) == 1 << 7


def test_registering_the_same_request_twice_reports_no_change(registry):
    assert registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_i"])
    assert not registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_i"])
    # 顺序不同但集合相同仍然算"变了"是可以接受的；这里钉的是同一份请求不重发。
    assert registry.register(TELEM_OWNER_POWER, ["batt_v"])


def test_names_outside_the_channel_table_are_reported_not_dropped(registry):
    """固件通道表里没有的名字必须说出来。静默丢弃 = 页面永远空着且没人知道为什么。"""
    registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_nope"])
    demand = registry.demand(INDEX)
    assert demand.missing == ("batt_nope",)
    assert demand.mask == 1 << 7


def test_an_empty_schema_yields_no_mask_but_still_lists_the_demand(registry):
    registry.register(TELEM_OWNER_POWER, ["batt_v"])
    demand = registry.demand(None)
    assert demand.mask == 0
    assert demand.channels == ("batt_v",) and demand.missing == ("batt_v",)


# ---------------------------------------------------------------- 流开关


def test_the_stream_is_reference_counted(registry):
    registry.register(TELEM_OWNER_DASHBOARD, ["roll"], stream=True)
    registry.register(TELEM_OWNER_POWER, ["batt_v"], stream=True)
    assert registry.demand(INDEX).stream

    # 工作台不可见了，但电源页还开着：流不能关。
    registry.register(TELEM_OWNER_DASHBOARD, ["roll"], stream=False)
    assert registry.demand(INDEX).stream

    registry.unregister(TELEM_OWNER_POWER)
    assert not registry.demand(INDEX).stream


def test_an_invisible_subscriber_keeps_its_params_but_drops_its_realtime_channels(registry):
    """看不见的页面没有理由让波形通道占带宽；滑块靠的参数通道必须留着。

    两类通道的带宽行为完全不同：实时通道每帧都带值，参数通道按脏位回显、稳态帧
    里不置位，只在每秒一次的全量刷新帧里出现。把参数通道也撤掉，滑块会永远停在
    初值——而且不会有任何提示。
    """
    registry.register(TELEM_OWNER_DASHBOARD, ["roll", "pitch"],
                      params=["rate_roll_kp"], stream=False)
    demand = registry.demand(INDEX)
    assert demand.mask == 1 << 20
    assert demand.channels == ("rate_roll_kp",)
    assert not demand.stream

    registry.register(TELEM_OWNER_DASHBOARD, ["roll", "pitch"],
                      params=["rate_roll_kp"], stream=True)
    assert registry.demand(INDEX).mask == (1 << 0) | (1 << 1) | (1 << 20)


def test_only_the_visible_subscribers_realtime_channels_are_on_the_wire(registry):
    """作者的要求：点开哪一页就传哪一页要的实时通道，不是把全部数据一直传着。"""
    registry.register(TELEM_OWNER_DASHBOARD, ["roll", "pitch"],
                      params=["rate_roll_kp"], stream=False)
    registry.register(TELEM_OWNER_POWER, ["batt_v", "batt_i"], stream=True)

    demand = registry.demand(INDEX)
    assert demand.mask == (1 << 7) | (1 << 8) | (1 << 20)
    assert not demand.mask & (1 << 0) and not demand.mask & (1 << 1)
    assert demand.stream


def test_exclusive_suppression_beats_the_reference_count(registry):
    """日志导出说关就必须关；"还有人在订阅"不是把它顶回去的理由。"""
    registry.register(TELEM_OWNER_DASHBOARD, ["roll"], stream=True)
    registry.register(TELEM_OWNER_POWER, ["batt_v"], stream=True)
    assert registry.suppress(TELEM_EXCLUSIVE_FLIGHT_LOG, "日志导出独占当前串口")

    demand = registry.demand(INDEX)
    assert not demand.stream
    assert demand.suppressed and demand.suppressed_by == (TELEM_EXCLUSIVE_FLIGHT_LOG,)
    assert registry.suppression_reason() == "日志导出独占当前串口"
    # 抑制期间继续订阅也打不开流。
    registry.register("another", ["pitch"], stream=True)
    assert not registry.demand(INDEX).stream
    # 但掩码照算——交还链路之后不用重新发现一遍谁要什么。
    assert registry.demand(INDEX).mask & (1 << 1)

    assert registry.release(TELEM_EXCLUSIVE_FLIGHT_LOG)
    assert registry.demand(INDEX).stream


def test_releasing_an_unheld_exclusive_is_a_no_op(registry):
    assert not registry.release(TELEM_EXCLUSIVE_FLIGHT_LOG)
    registry.register(TELEM_OWNER_POWER, ["batt_v"])
    assert registry.demand(INDEX).stream


def test_suppression_with_no_subscriber_still_keeps_the_stream_closed(registry):
    registry.suppress(TELEM_EXCLUSIVE_FLIGHT_LOG)
    assert not registry.demand(INDEX).stream
    registry.release(TELEM_EXCLUSIVE_FLIGHT_LOG)
    assert not registry.demand(INDEX).stream


# ---------------------------------------------------------------- sink 分发


def test_the_fanout_keeps_the_registering_transport_for_every_consumer():
    """transport 只有一个 sink 槽。多接一个消费者不能把来源身份弄丢。"""
    fanout = TelemBinaryFanout()
    seen: list[tuple[str, object]] = []
    fanout.attach("a", lambda fn, payload, *, transport, generation=None: seen.append(("a", transport)))
    fanout.attach("b", lambda fn, payload, *, transport, generation=None: seen.append(("b", transport)))

    origin = object()
    sink = fanout.bind(origin)
    other = object()
    fanout.bind(other)                       # 另绑一个不影响已经发出去的闭包
    sink(0x2230, b"payload")

    assert sorted(seen) == sorted([("a", origin), ("b", origin)])
    assert fanout.delivered == 1


def test_the_fanout_pins_the_generation_at_bind_time():
    """串口重连是在同一个对象上把代次加一。现读代次会让旧闭包冒充新会话。"""
    fanout = TelemBinaryFanout()
    seen: list = []
    fanout.attach("a", lambda fn, payload, *, transport, generation=None:
                  seen.append(generation))

    class Link:
        connection_generation = 3

    link = Link()
    sink = fanout.bind(link)
    link.connection_generation = 4          # 重连了
    sink(0x2230, b"late frame")
    assert seen == [3]


def test_detaching_a_consumer_stops_only_that_one():
    fanout = TelemBinaryFanout()
    kept: list = []
    fanout.attach("keep", lambda fn, payload, *, transport, generation=None: kept.append(payload))
    fanout.attach("drop", lambda fn, payload, *, transport, generation=None: kept.append(b"wrong"))
    assert fanout.detach("drop")
    assert not fanout.detach("drop")
    fanout.bind(object())(0x2230, b"ok")
    assert kept == [b"ok"]
    assert fanout.owners() == ("keep",)


def test_a_consumer_can_detach_itself_from_inside_the_callback():
    """收线程里退订不能打坏正在进行的这一次分发。"""
    fanout = TelemBinaryFanout()
    calls: list[str] = []

    def leaver(fn, payload, *, transport, generation=None):
        calls.append("leaver")
        fanout.detach("leaver")

    fanout.attach("leaver", leaver)
    fanout.attach("stay", lambda fn, payload, *, transport, generation=None: calls.append("stay"))
    sink = fanout.bind(object())
    sink(0x2230, b"")
    sink(0x2230, b"")
    assert calls == ["leaver", "stay", "stay"]
