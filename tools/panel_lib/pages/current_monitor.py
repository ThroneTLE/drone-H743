"""AM32 current readback. Reuses the panel transport and its receive provenance."""
from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

from ..connection_state import receive_context
from ..proto import PROTO_REQ_STATUS, parse_kv

POLL_SECONDS = 2.0
STALE_SECONDS = 3.0
UINT32_MAX = 0xFFFFFFFF
ADC_STATUS = {0: "正常", 1: "尚未初始化", 2: "转换超时", 3: "采样错误"}


def parse_current(line: str) -> dict:
    """A complete firmware CURRENT line is one snapshot; never merge fragments."""
    if not line.startswith("CURRENT "):
        raise ValueError("不是电流回包")
    values = parse_kv(line)
    result = {}
    for key in ("raw", "valid", "calibrated", "saturated", "age_ms", "samples", "errors", "adc_status"):
        text = values[key]
        if not text.isdecimal():
            raise ValueError(key)
        result[key] = int(text)
        maximum = 65535 if key == "raw" else (1 if key in ("valid", "calibrated", "saturated") else UINT32_MAX)
        if result[key] > maximum:
            raise ValueError(key)
    if result["adc_status"] not in ADC_STATUS or values["source"] != "AM32_55A_CURR":
        raise ValueError("不支持的电流来源或状态")
    for key in ("adc_v", "current_a", "nominal_mv_per_a"):
        result[key] = float(values[key])
    if not math.isfinite(result["nominal_mv_per_a"]) or result["nominal_mv_per_a"] <= 0:
        raise ValueError("比例无效")
    if not (math.isnan(result["adc_v"]) or 0 <= result["adc_v"] <= 3.6):
        raise ValueError("ADC 电压越界")
    if result["valid"] and (not math.isfinite(result["current_a"]) or
                            not math.isfinite(result["adc_v"]) or result["saturated"] or
                            result["adc_status"] != 0 or result["samples"] == 0 or result["age_ms"] > 250):
        raise ValueError("回包有效性矛盾")
    result["source"] = values["source"]
    return result


class CurrentMonitorPage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent)
        self.panel = panel
        self.snapshot = None
        self.receipt = None
        self.last_attempt = None
        self.first_request = None
        self.parse_error = ""
        self._session = None
        self._timer = None
        self._disposed = False
        self.auto_var = tk.BooleanVar(self, value=True)
        self.state_var = tk.StringVar(self, value="未连接")
        self.current_var = tk.StringVar(self, value="— A")
        self.fields = {key: tk.StringVar(self, value="—") for key in (
            "raw", "adc_v", "valid", "calibrated", "saturated", "age_ms", "received_age",
            "samples", "errors", "adc_status", "source", "nominal_mv_per_a")}
        ttk.Label(self, text="电流计", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(self, text="AM32 55A · 最近回读", style="Muted.TLabel").pack(anchor=tk.W)
        actions = ttk.Frame(self)
        actions.pack(fill=tk.X, pady=10)
        self.read_button = ttk.Button(actions, text="立即读取", command=self.request_read)
        self.read_button.pack(side=tk.LEFT)
        ttk.Checkbutton(actions, text="自动刷新（2 秒）", variable=self.auto_var).pack(side=tk.LEFT, padx=12)
        ttk.Label(self, textvariable=self.current_var, style="PageTitle.TLabel").pack(anchor=tk.W, pady=(4, 0))
        ttk.Label(self, textvariable=self.state_var, style="Muted.TLabel", wraplength=600).pack(anchor=tk.W, pady=(0, 10))
        box = ttk.LabelFrame(self, text="飞控回读信息", padding=10)
        box.pack(fill=tk.X)
        box.columnconfigure(1, weight=1)
        labels = ("ADC 原始值", "ADC 电压", "回包采样有效", "电流计校准", "ADC 高端饱和", "回包时样本年龄",
                  "距收到回包", "采样次数", "采样错误次数", "ADC 状态", "信号来源", "标称灵敏度")
        self.value_labels = []
        for row, (key, label) in enumerate(zip(self.fields, labels)):
            ttk.Label(box, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 20), pady=2)
            value = ttk.Label(box, textvariable=self.fields[key], style="Mono.TLabel")
            value.grid(row=row, column=1, sticky=tk.W, pady=2)
            self.value_labels.append(value)
        ttk.Label(self, text="电流传感器位于电调，飞控读取 Curr 信号。标称换算尚未实测校准；"
                  "零读数不能证明 Curr 接线正常，也不能区分两台电机的电流。",
                  style="Muted.TLabel", wraplength=600).pack(fill=tk.X, pady=12)
        self.bind("<Destroy>", self._destroyed, add="+")
        self._tick()

    def _check_session(self):
        transport = self.panel.transport
        session = (transport, getattr(transport, "connection_generation", 0))
        if session != self._session or not self.panel._transport_connected():
            self.snapshot = self.receipt = self.last_attempt = self.first_request = None
            self.parse_error = ""
            self._session = session

    def handle_line(self, line):
        self._check_session()
        receipt = getattr(self.panel, "_rx_context", None) or receive_context(self.panel.transport)
        if not receipt.is_current(self.panel.transport):
            return
        try:
            snapshot = parse_current(line)
        except (KeyError, TypeError, ValueError, OverflowError):
            self.snapshot = None
            self.receipt = receipt
            self.parse_error = "电流回包缺字段或数据无效"
        else:
            self.snapshot, self.receipt = snapshot, receipt
            self.parse_error = ""
        self.refresh()

    def request_read(self):
        self._check_session()
        now = time.monotonic()
        if not self.panel._transport_connected() or (self.last_attempt is not None and now - self.last_attempt < POLL_SECONDS):
            return False
        self.last_attempt = now
        sent = self.panel._send_proto_silent(PROTO_REQ_STATUS, "STATUS?")
        if sent and self.first_request is None:
            self.first_request = now
        if not sent:
            self.state_var.set("读取未发送：连接或当前会话拒绝")
        return sent

    def refresh(self):
        self._check_session()
        now = time.monotonic()
        connected = self.panel._transport_connected()
        self.read_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        for variable in self.fields.values():
            variable.set("—")
        self.current_var.set("— A")
        if not connected:
            self.state_var.set("未连接")
            return
        if self.parse_error:
            self.state_var.set(self.parse_error)
            return
        if self.snapshot is None or self.receipt is None:
            missing = self.first_request is not None and now - self.first_request > STALE_SECONDS
            self.state_var.set("未收到电流回包，请确认固件支持 CURRENT" if missing else "等待电流回包")
            return
        age = max(0.0, now - self.receipt.received_at)
        s = self.snapshot
        for key in self.fields:
            if key in s:
                self.fields[key].set(str(s[key]))
        self.fields["adc_v"].set(f"{s['adc_v']:.5f} V" if math.isfinite(s["adc_v"]) else "不可用")
        self.fields["valid"].set("是" if s["valid"] else "否")
        self.fields["calibrated"].set("已标记校准" if s["calibrated"] else "未校准 · 标称换算")
        self.fields["saturated"].set("是" if s["saturated"] else "否")
        self.fields["age_ms"].set("无样本" if s["age_ms"] == UINT32_MAX else f"{s['age_ms']} ms")
        self.fields["received_age"].set(f"{age:.1f} s")
        self.fields["adc_status"].set(ADC_STATUS[s["adc_status"]])
        self.fields["nominal_mv_per_a"].set(f"{s['nominal_mv_per_a']:g} mV/A")
        if age > STALE_SECONDS:
            self.state_var.set("电流回包已过期 · 下方为上次回读信息")
        elif not s["valid"]:
            self.state_var.set("电流不可用 · 检查 ADC 状态、样本年龄和饱和标识")
        else:
            self.current_var.set(f"{s['current_a']:.3f} A")
            self.state_var.set("最近回读有效 · " + ("已标记校准" if s["calibrated"] else "尚未实测校准"))

    def _tick(self):
        if self._disposed:
            return
        self.refresh()
        visible = (self.panel.notebook.select() == str(self.panel.sensor_group_tab) and
                   self.panel.sensor_notebook.select() == str(self.panel.current_tab))
        if visible and self.auto_var.get():
            self.request_read()
        self._timer = self.after(250, self._tick)

    def _destroyed(self, event):
        if event.widget is self:
            self._disposed = True
            if self._timer is not None:
                self.after_cancel(self._timer)
                self._timer = None


def consume_current_message(panel, item):
    page = getattr(panel, "current_page", None)
    line = item[2] if isinstance(item, tuple) and len(item) == 3 and item[0] == "proto" else item
    if page is not None and isinstance(line, str) and line.startswith("CURRENT "):
        page.handle_line(line)


def mount_current(panel, parent):
    panel.current_page = CurrentMonitorPage(parent, panel)
    panel.current_page.pack(fill=tk.BOTH, expand=True)
