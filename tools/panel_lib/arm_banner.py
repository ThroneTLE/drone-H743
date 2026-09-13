"""主页面顶部的解锁状态横幅。

为什么放在最上面而不是塞进某一页：解锁状态是**每一次上机都要先看一眼**的东西。
在这之前，飞控表达"为什么解不了锁"的唯一途径是 LED_3 闪几下——要数、要查表、
室外还看不清。而且原因链是有序的，它只报第一条不满足的条件；同时缺两样时
（比如既没插遥控器又没写机体模型），修完一个仍然解不了锁，很容易以为没修对。

所以横幅做两件事：
  1. 大字报当前状态与**飞控自己给出的**被拒原因（不是上位机猜的）；
  2. 把每一项条件的通过与否一次列全，让人一眼看到还差几样。

数据来自 `REQ id=... mod=ARM op=STATUS`（App/Src/app_cmd_arm.c）。飞控还没跑过一圈
控制环时报 block=unknown，横幅照实说"未知"——"还不知道"和"没问题"是两回事，
后者会让人以为已经可以解锁了。
"""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk

from .proto import PROTO_REQ_STATUS


POLL_SECONDS = 0.5
STALE_SECONDS = 3.0

BLOCK_TEXT = {
    "none": "无阻塞 · 可以解锁",
    "no_rc": "没有收到过遥控器数据",
    "rc_loss": "遥控链路丢失",
    "arm_switch": "解锁拨杆未打（需低→高的动作沿）",
    "throttle_high": "油门未收到底",
    "imu": "IMU 采样链不健康",
    "frame_migration": "坐标系迁移未完成 / 标定候选未提交",
    "airframe": "机体模型未写入或不完整",
    "battery": "电池电压低或采样无效；恢复后需重新拨动解锁开关",
    "unknown": "未知（飞控控制环尚未跑过一圈）",
}

# (报文键, 显示名)。顺序照着飞控原因链的判定顺序排，方便和 block 对照。
CONDITIONS = (
    ("rc_seen", "收到过遥控"),
    ("rc_ok", "遥控链路正常"),
    ("frame", "坐标迁移已完成"),
    ("imu_health", "IMU 采样健康"),
    ("airframe", "机体模型有效"),
    ("battery_ok", "电池电压满足解锁要求"),
    ("switch", "解锁拨杆已打"),
    ("throttle_low", "油门已收到底"),
    ("imu", "IMU 数据可用"),
    ("servo_cal_idle", "无舵机标定占用"),
    ("accept_idle", "无验收流程占用"),
)


class ArmBanner(ttk.LabelFrame):
    def __init__(self, parent, panel):
        super().__init__(parent, text="解锁状态", padding=(10, 6))
        self.panel = panel
        self.state_var = tk.StringVar(value="未连接")
        self.reason_var = tk.StringVar(value="连接飞控后自动刷新")
        self.condition_vars: dict[str, tk.StringVar] = {}
        self._last_rx = 0.0
        self._last_poll = 0.0
        self._timer = None
        self._disposed = False

        head = ttk.Frame(self)
        head.pack(fill=tk.X)
        self.state_label = ttk.Label(head, textvariable=self.state_var,
                                     style="PageTitle.TLabel")
        self.state_label.pack(side=tk.LEFT)
        ttk.Label(head, textvariable=self.reason_var,
                  style="Muted.TLabel", wraplength=820).pack(side=tk.LEFT, padx=(14, 0))

        grid = ttk.Frame(self)
        grid.pack(fill=tk.X, pady=(4, 0))
        for index, (key, label) in enumerate(CONDITIONS):
            var = tk.StringVar(value=f"· {label}")
            self.condition_vars[key] = var
            ttk.Label(grid, textvariable=var, style="Muted.TLabel").grid(
                row=index // 5, column=index % 5, sticky=tk.W, padx=(0, 16))

        panel.bind("<Destroy>", self._on_destroy, add="+")
        self._tick()

    # ── 接收 ──────────────────────────────────────────────────────────
    def handle_rsp(self, payload: dict[str, str]) -> None:
        """消费 `RSP id=.. mod=ARM op=STATUS ...` 的键值对。"""
        self.panel.arm_status = dict(payload)
        self._last_rx = time.monotonic()

        armed = payload.get("armed") == "1"
        block = payload.get("block", "unknown")
        known = payload.get("known") == "1"

        if armed:
            self.state_var.set("已解锁")
            style = "Pass.TLabel"
        elif block == "none" and known:
            self.state_var.set("可以解锁")
            style = "Pass.TLabel"
        else:
            self.state_var.set("未解锁")
            style = "Fail.TLabel" if known else "Muted.TLabel"
        try:
            self.state_label.configure(style=style)
        except tk.TclError:      # 主题里没有这些样式时不值得让面板崩掉
            pass

        reason = BLOCK_TEXT.get(block, block)
        if block == "airframe":
            missing = payload.get("airframe_missing", "-")
            if missing not in ("-", ""):
                reason = f"{reason}（缺 {missing}）"
        if armed:
            reason = "飞机处于解锁状态，桨会转"
            if payload.get("battery_ok") == "0":
                reason = "电池低压或电压数据失效告警 · 当前仍处于解锁状态"
        self.reason_var.set(reason)

        for key, label in CONDITIONS:
            value = payload.get(key)
            if value is None:
                mark = "·"
            elif value == "1":
                mark = "✓"
            else:
                mark = "✗"
            self.condition_vars[key].set(f"{mark} {label}")

    # ── 轮询 ──────────────────────────────────────────────────────────
    def _tick(self) -> None:
        if self._disposed:
            return
        now = time.monotonic()
        try:
            connected = self.panel._transport_connected()
            if connected and (now - self._last_poll) >= POLL_SECONDS:
                self._last_poll = now
                self.panel._send_proto_silent(PROTO_REQ_STATUS,
                                              "REQ id=9101 mod=ARM op=STATUS")
            if not connected:
                self.state_var.set("未连接")
                self.reason_var.set("连接飞控后自动刷新")
                self._clear_conditions()
            elif self._last_rx and (now - self._last_rx) > STALE_SECONDS:
                # D5-3：宁可说"数据过期"，也不要让上一帧的"可以解锁"一直挂着。
                self.state_var.set("状态过期")
                self.reason_var.set("超过 3 秒没有收到 ARM 状态，显示的是旧值")
        finally:
            self._timer = self.panel.after(int(POLL_SECONDS * 1000), self._tick)

    def _clear_conditions(self) -> None:
        for key, label in CONDITIONS:
            self.condition_vars[key].set(f"· {label}")

    def _on_destroy(self, event) -> None:
        if event.widget is self.panel:
            self._disposed = True
            if self._timer is not None:
                try:
                    self.panel.after_cancel(self._timer)
                except tk.TclError:
                    pass
                self._timer = None


def mount_arm_banner(panel, parent) -> ArmBanner:
    panel.arm_status = {}
    banner = ArmBanner(parent, panel)
    banner.pack(fill=tk.X, pady=(8, 0))
    panel.arm_banner = banner
    return banner


def handle_arm_rsp(panel, payload: dict[str, str]) -> None:
    """drone_tcp_panel._handle_rsp_line 的转发点；横幅没挂载时也不能丢数据。"""
    panel.arm_status = dict(payload)
    banner = getattr(panel, "arm_banner", None)
    if banner is not None:
        banner.handle_rsp(payload)
