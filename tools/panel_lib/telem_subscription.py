"""遥测链路的订阅仲裁：掩码并集、流开关引用计数、二进制 sink 的单槽分发。

这条链路上的三样东西都**只有一份**，而想动它们的页面不止一个：

* `TELEM MASK <hex>` —— 后发的整条覆盖先发的。原来只有工作台在发，掩码就等于
  工作台自己的需求；再多一个页面要订阅通道，两边就会互相把对方的位擦掉。
* `TELEM STREAM on|off` —— 工作台按可见性开关流，日志导出要独占链路所以单方面
  关流。两个 owner，没有任何仲裁：谁最后说话谁算数。
* `transport.set_binary_sink()` —— transport 只有一个 sink 槽。工作台不可见时
  它把 sink 摘掉，于是"另一个页面订阅了通道却收不到帧"。

所以这里持有**谁要什么**的注册表，对外只给出仲裁后的结论：

    掩码   = 所有订阅者通道名的并集，按调用方传进来的 schema 映射成通道号
    流开关 = 任一订阅者要开就开，全部退订才关
    sink   = 一个分发器接多个消费者，挂在唯一那个槽上

**独占抑制不是引用计数的一张票**。日志导出需要独占链路，它说"关"就必须关，
不能因为还有别的页面在订阅就被引用计数重新打开——那会让导出中途被遥测帧插队。
它因此是一个单独的状态，压过 `stream` 的计算结果。

本模块是纯逻辑：不 import tkinter、不碰 transport、不发命令、不读时钟。
它只回答"应该是什么样"，让调用方去把链路变成那样——这样仲裁规则能脱离 Tk
和串口被单测判对错。通道名到通道号的映射由调用方把 schema 传进来。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Mapping


#: 工作台（`pages/dashboard.py`）。绑定通道 ∪ 全部参数通道。
TELEM_OWNER_DASHBOARD = "dashboard"
#: 电源页（`pages/power.py`）。可见才订阅 `batt_v` / `batt_i`。
TELEM_OWNER_POWER = "power"
#: 桨叶与电机方向页（`pages/prop_map.py`）。可见才订阅 `batt_v` / `batt_i`。
TELEM_OWNER_PROP_MAP = "prop_map"
#: 日志导出（`log_receive_view.py`）。独占抑制者，不是订阅者。
TELEM_EXCLUSIVE_FLIGHT_LOG = "flight_log"


def _names(values: Iterable[str]) -> tuple[str, ...]:
    return tuple(dict.fromkeys(str(name) for name in values if str(name)))


@dataclass(frozen=True)
class TelemSubscription:
    """一个订阅者要什么。通道按**带宽行为**分两类，不按页面分：

    * `realtime` —— 每一帧都带值的通道（波形、数值卡、`batt_v` / `batt_i`）。
      订阅者不可见时必须从掩码里撤出去：看不见的页面没有理由占数传带宽，
      重新可见时波形本来就是从头画。
    * `params` —— 参数回显通道。固件按脏位回显，稳态帧里不置位，只在每秒一次的
      全量刷新帧里出现（27 路 × 4 B ≈ 108 B/s）。它们**不随可见性撤出**：撤掉
      滑块就再也收不到飞控的实际值，会永远停在初值。

    `wants_stream` 再独立一层：不可见的订阅者两类通道都还能登记 params，但不要求
    开流。
    """

    owner: str
    realtime: tuple[str, ...]
    params: tuple[str, ...]
    wants_stream: bool
    visible: bool

    @property
    def channels(self) -> tuple[str, ...]:
        """实际进掩码的通道：可见才带实时通道，参数通道恒在。"""
        if self.visible:
            return self.realtime + tuple(
                name for name in self.params if name not in self.realtime)
        return self.params


@dataclass(frozen=True)
class TelemDemand:
    """仲裁结论。调用方照着它把链路收敛过去。"""

    owners: tuple[str, ...]
    channels: tuple[str, ...]
    mask: int
    #: 订阅了但当前通道表里没有的名字。不静默丢弃——名字对不上是"安静失败"的
    #: 典型入口，必须让页面能把它显示出来。
    missing: tuple[str, ...]
    stream: bool
    suppressed_by: tuple[str, ...]

    @property
    def suppressed(self) -> bool:
        return bool(self.suppressed_by)


class TelemSubscriptionRegistry:
    """谁订阅了哪些通道、谁要求开流、谁在独占抑制。"""

    def __init__(self) -> None:
        self._subscriptions: dict[str, TelemSubscription] = {}
        self._exclusive: dict[str, str] = {}

    # ------------------------------------------------------------- 订阅

    def register(self, owner: str, realtime: Iterable[str] = (), *,
                 params: Iterable[str] = (), stream: bool = True,
                 visible: bool | None = None) -> bool:
        """登记一个订阅者。返回本次是否**改变**了注册表。

        `visible` 缺省跟随 `stream`——想开流的订阅者就是可见的那个。两者分开写只是
        为了让"可见但不要流"这种情况也能表达，不必再加一个概念。

        返回值是给调用方省一次重发用的：掩码和流开关都是幂等命令，但每拍重发
        会把本来就紧张的上行占满。
        """
        owner = str(owner)
        entry = TelemSubscription(
            owner,
            _names(realtime),
            _names(params),
            bool(stream),
            bool(stream) if visible is None else bool(visible),
        )
        if self._subscriptions.get(owner) == entry:
            return False
        self._subscriptions[owner] = entry
        return True

    def unregister(self, owner: str) -> bool:
        """退订。返回本次是否真的移除了一个订阅者。"""
        return self._subscriptions.pop(str(owner), None) is not None

    def is_registered(self, owner: str) -> bool:
        return str(owner) in self._subscriptions

    def subscription(self, owner: str) -> TelemSubscription | None:
        return self._subscriptions.get(str(owner))

    def owners(self) -> tuple[str, ...]:
        return tuple(sorted(self._subscriptions))

    def channels(self) -> tuple[str, ...]:
        names: set[str] = set()
        for entry in self._subscriptions.values():
            names.update(entry.channels)
        return tuple(sorted(names))

    # ------------------------------------------------------------- 独占

    def suppress(self, owner: str, reason: str = "") -> bool:
        """进入独占抑制：无论还有多少订阅者，流一律关。

        这不是引用计数的一票否决票，是一个单独的闸。日志导出期间链路归导出，
        任何"还有人在订阅"都不能把流重新打开。
        """
        owner = str(owner)
        text = str(reason)
        if self._exclusive.get(owner) == text:
            return False
        self._exclusive[owner] = text
        return True

    def release(self, owner: str) -> bool:
        return self._exclusive.pop(str(owner), None) is not None

    @property
    def suppressed(self) -> bool:
        return bool(self._exclusive)

    def suppressors(self) -> tuple[str, ...]:
        return tuple(sorted(self._exclusive))

    def suppression_reason(self) -> str:
        for owner in self.suppressors():
            reason = self._exclusive[owner]
            if reason:
                return reason
        return ""

    # ------------------------------------------------------------- 结论

    def mask(self, index_by_name: Mapping[str, int] | None = None) -> int:
        return self.demand(index_by_name).mask

    def demand(self, index_by_name: Mapping[str, int] | None = None) -> TelemDemand:
        lookup: Mapping[str, int] = {} if index_by_name is None else index_by_name
        mask = 0
        missing: list[str] = []
        names = self.channels()
        for name in names:
            index = lookup.get(name)
            if index is None:
                missing.append(name)
                continue
            mask |= 1 << int(index)
        wants_stream = any(
            entry.wants_stream for entry in self._subscriptions.values()
        )
        return TelemDemand(
            owners=self.owners(),
            channels=names,
            mask=mask,
            missing=tuple(missing),
            stream=wants_stream and not self.suppressed,
            suppressed_by=self.suppressors(),
        )


#: 二进制消费者的签名：`(function, payload, *, transport)`。
BinaryConsumer = Callable[..., None]


class TelemBinaryFanout:
    """一个 sink 槽 → 多个消费者，并且**保住来源身份**。

    `bind(transport)` 返回的闭包把 transport 钉死在里面。回调签名本身不带
    transport，消费者只读 `panel.transport` 的话会把重连前后的帧算到同一条链路
    上——那正是新连接的第一帧混进旧录制文件的路径（审核 R2）。分发给多个消费者
    之后这条身份必须原样传下去，否则多一个消费者就等于把它丢一次。
    """

    def __init__(self) -> None:
        self._consumers: dict[str, BinaryConsumer] = {}
        self.delivered = 0

    def attach(self, owner: str, consumer: BinaryConsumer) -> bool:
        owner = str(owner)
        # `==` 而不是 `is`：`page._on_telem_frame` 每次取属性都是一个**新的**绑定
        # 方法对象，用 `is` 比会把"同一个消费者"判成"换人了"，于是每拍都报一次
        # 变化，调用方跟着每拍重发一条 `TELEM MASK`。绑定方法的 `==` 比的是
        # (函数, 实例)，正是这里要的语义。
        if self._consumers.get(owner) == consumer:
            return False
        self._consumers[owner] = consumer
        return True

    def detach(self, owner: str) -> bool:
        return self._consumers.pop(str(owner), None) is not None

    def owners(self) -> tuple[str, ...]:
        return tuple(sorted(self._consumers))

    def __len__(self) -> int:
        return len(self._consumers)

    def bind(self, transport, generation=None):
        """给 `transport.set_binary_sink()` 用的闭包；来源身份在这里定死。

        连接代次也在这里**取一次并钉住**，不在分发时现读：串口重连是在同一个
        transport 对象上把 `connection_generation` 加一，现读的话，旧闭包在重连
        之后送来的最后一帧会顶着**新**代次进来，看上去就像新会话的数据。
        """
        if generation is None:
            generation = getattr(transport, "connection_generation", None)

        def sink(function: int, payload: bytes) -> None:
            self.deliver(function, payload, transport=transport, generation=generation)

        return sink

    def deliver(self, function: int, payload: bytes, *, transport,
                generation=None) -> None:
        self.delivered += 1
        # 收线程在跑：先拷一份再遍历，免得某个消费者在回调里退订时改坏迭代器。
        for _owner, consumer in tuple(self._consumers.items()):
            consumer(function, payload, transport=transport, generation=generation)


__all__ = [
    "TELEM_EXCLUSIVE_FLIGHT_LOG",
    "TELEM_OWNER_DASHBOARD",
    "TELEM_OWNER_POWER",
    "TELEM_OWNER_PROP_MAP",
    "TelemBinaryFanout",
    "TelemDemand",
    "TelemSubscription",
    "TelemSubscriptionRegistry",
]
