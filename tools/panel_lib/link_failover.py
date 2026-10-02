"""USB 串口意外断开后，自动搜索并切换到蓝牙（Tk 线程上的小状态机）。

触发条件（全部满足才切）：
* 开关打开（默认开，持久化键 `usb_failover_bluetooth`）。
* 断开的是 USB 串口会话（不是蓝牙口），且断开原因不是用户/程序主动停止
  （`DisconnectInfo.phase` 为 stop / open cancelled / cancel 一律不触发）。
* 当时没有固件升级流程，也不在 DFU 保持窗口内（`hold()`，AI 接口发 BOOT DFU 前调用）。
* 宽限期（GRACE_S）后该 USB 口仍未重新出现、用户也没有手动连回别的链路。
  飞控重启导致的短暂掉线会在宽限期内回来，不切换。

选设备：先用持久化里上次成功使用的蓝牙地址，找不到再按现有发现逻辑（MicoAir 名称）搜；
最多 MAX_ATTEMPTS 次、总时长 TOTAL_TIMEOUT_S。失败只写界面状态与日志，不弹窗。
切换后 USB 重新插回不会抢回（沿用现有约定：只有启动时才按记录自动连）。
"""
from __future__ import annotations

import time

STATE_KEY = "usb_failover_bluetooth"
POLL_MS = 500
GRACE_S = 3.0
RETRY_GAP_S = 4.0
MAX_ATTEMPTS = 3
TOTAL_TIMEOUT_S = 60.0
DFU_HOLD_S = 90.0
# 这些 phase 是我们自己（用户停止/取消）造成的断开，不算意外。
INTENTIONAL_PHASES = frozenset({"stop", "open cancelled", "cancel"})


def firmware_busy(panel) -> bool:
    return bool(getattr(panel, "firmware_update_pending", False)
                or getattr(panel, "firmware_update_running", False)
                or getattr(panel, "firmware_programming", False))


def is_unexpected(info) -> bool:
    return info is not None and str(info.phase) not in INTENTIONAL_PHASES


def hold(panel, seconds: float = DFU_HOLD_S) -> None:
    """主动进 DFU 等会让 USB 消失的操作，在发命令前调用；窗口内的断开不切蓝牙。"""
    failover = getattr(panel, "link_failover", None)
    if failover is not None:
        failover.hold(seconds)


class LinkFailover:
    def __init__(self, panel, clock=time.monotonic):
        self.panel = panel
        self.clock = clock
        self._hold_until = 0.0
        self._seen = self._token(panel.serial_transport.last_disconnect)
        self._phase = "idle"       # idle / grace / attempt / wait_scan / wait_open / gap
        self._info = None
        self._deadline = 0.0
        self._total_deadline = 0.0
        self._attempt = 0
        self._remembered = ""
        self._gap_until = 0.0
        self._after = None
        self._closed = False

    # ---- 生命周期 -------------------------------------------------------
    def start(self) -> None:
        self._after = self.panel.after(POLL_MS, self.tick)

    def close(self) -> None:
        self._closed = True
        if self._after is not None:
            try:
                self.panel.after_cancel(self._after)
            except Exception:
                pass
            self._after = None

    def hold(self, seconds: float) -> None:
        self._hold_until = max(self._hold_until, self.clock() + float(seconds))
        if self._phase != "idle":
            self._give_up("DFU/固件操作开始，取消自动切蓝牙", failed=False)

    @property
    def enabled(self) -> bool:
        var = getattr(self.panel, "usb_failover_var", None)
        try:
            return bool(var.get()) if var is not None else False
        except Exception:
            return False

    @staticmethod
    def _token(info):
        return None if info is None else (info.port, info.generation, info.occurred_at)

    # ---- 主循环 ---------------------------------------------------------
    def tick(self) -> None:
        if self._closed:
            return
        try:
            self._step()
        finally:
            if not self._closed:
                self._after = self.panel.after(POLL_MS, self.tick)

    def _status(self, text: str) -> None:
        self.panel.autoconnect_var.set(text)

    def _log(self, text: str) -> None:
        try:
            self.panel._append(f"[链路] {text}")
        except Exception:
            pass

    def _step(self) -> None:
        panel = self.panel
        now = self.clock()
        info = panel.serial_transport.last_disconnect
        token = self._token(info)
        if token != self._seen:
            self._seen = token
            self._on_disconnect(info, now)
        if self._phase == "idle":
            return
        if self._phase != "grace" and now > self._total_deadline:
            self._give_up(f"超过 {TOTAL_TIMEOUT_S:.0f} 秒仍未连上蓝牙")
            return
        if self._phase == "grace":
            self._step_grace(now)
        elif self._phase == "attempt" or (self._phase == "gap" and now >= self._gap_until):
            self._begin_attempt(now)
        elif self._phase == "wait_scan":
            self._step_scan(now)
        elif self._phase == "wait_open":
            self._step_open(now)

    def _on_disconnect(self, info, now: float) -> None:
        panel = self.panel
        if not self.enabled or not is_unexpected(info):
            return
        if now < self._hold_until or firmware_busy(panel):
            return
        identity = panel._serial_port_identity.get(str(info.port)) or {}
        if identity.get("bluetooth_address"):
            return                      # 断的是蓝牙口，不是 USB
        if getattr(panel, "transport", None) is not panel.serial_transport:
            return
        if self._phase != "idle":
            return
        self._info = info
        self._phase = "grace"
        self._deadline = now + GRACE_S
        self._status(f"USB 串口 {info.port} 意外断开，{GRACE_S:.0f} 秒后仍未恢复将自动切换蓝牙…")
        self._log(f"USB {info.port} 意外断开（{info.phase}），进入自动切蓝牙宽限期")

    def _step_grace(self, now: float) -> None:
        panel = self.panel
        if not self.enabled:
            self._reset()
            return
        if now < self._hold_until or firmware_busy(panel):
            self._give_up("DFU/固件操作开始，取消自动切蓝牙", failed=False)
            return
        if panel._transport_connected():
            self._reset()               # 用户已手动连上别的链路
            return
        if now < self._deadline:
            return
        port = str(self._info.port)
        try:
            panel._refresh_serial_ports()
        except Exception:
            pass
        if any(str(dev).casefold() == port.casefold() for dev in panel._serial_port_map.values()):
            self._status(f"USB 串口 {port} 已恢复，未切换蓝牙（请手动重新连接）")
            self._log(f"{port} 在宽限期内重新出现，不切蓝牙")
            self._reset()
            return
        state = getattr(panel, "_panel_state", {}) or {}
        self._remembered = str(state.get("bluetooth_address") or "")
        self._attempt = 0
        self._total_deadline = now + TOTAL_TIMEOUT_S
        self._phase = "attempt"
        self._begin_attempt(now)

    def _begin_attempt(self, now: float) -> None:
        panel = self.panel
        if panel._transport_connected():
            self._reset()
            return
        self._attempt += 1
        if self._attempt > MAX_ATTEMPTS:
            self._give_up(f"已尝试 {MAX_ATTEMPTS} 次")
            return
        # 奇数次先试上次用过的设备，偶数次放开按名称搜；没有记录就一直放开搜。
        preferred = self._remembered if (self._remembered and self._attempt % 2 == 1) else ""
        self._status(f"USB 已断开，正在自动切换蓝牙（第 {self._attempt}/{MAX_ATTEMPTS} 次，"
                     f"{'上次的设备' if preferred else '搜索 MicoAir 设备'}）…")
        panel.transport_var.set("蓝牙")        # 触发界面切到蓝牙并扫描
        panel._request_bluetooth_scan(connect=True, preferred=preferred)
        self._phase = "wait_scan"

    def _step_scan(self, now: float) -> None:
        panel = self.panel
        if getattr(panel, "_bt_scanning", False):
            return
        if getattr(panel, "_bt_opening", False) or panel.serial_transport.is_connected:
            self._phase = "wait_open"
            return
        # 扫描结束却没有开始连接：没找到设备 / 设备不可用 / 识别失败。
        self._retry_later(now, str(panel.autoconnect_var.get()))

    def _step_open(self, now: float) -> None:
        panel = self.panel
        if getattr(panel, "_bt_opening", False):
            return
        if panel.serial_transport.is_connected:
            port = panel.serial_transport.active_port or ""
            self._status(f"已自动切到蓝牙（USB 断开后）：{port}")
            self._log(f"USB 断开后已自动切换到蓝牙 {port}")
            self._reset()
            return
        self._retry_later(now, "蓝牙打开失败")

    def _retry_later(self, now: float, why: str) -> None:
        if self._attempt >= MAX_ATTEMPTS:
            self._give_up(why)
            return
        self._status(f"自动切蓝牙第 {self._attempt} 次未成功（{why}），稍后重试…")
        self._gap_until = now + RETRY_GAP_S
        self._phase = "gap"

    def _give_up(self, why: str, failed: bool = True) -> None:
        panel = self.panel
        try:
            panel._cancel_bluetooth()
        except Exception:
            pass
        if failed:
            self._status(f"USB 断开后自动切蓝牙失败：{why}。请检查飞控蓝牙已配对/上电，或手动连接")
            self._log(f"自动切蓝牙失败：{why}")
            if not panel._transport_connected():
                panel.transport_var.set("serial")   # 回到串口模式，方便插回 USB 后手动连
        else:
            self._status(why)
        self._reset()

    def _reset(self) -> None:
        self._phase = "idle"
        self._info = None


def install(panel):
    """建开关变量并启动监视。由连接栏构建时调用一次。"""
    import tkinter as tk
    state = getattr(panel, "_panel_state", {}) or {}
    panel.usb_failover_var = tk.BooleanVar(master=panel, value=bool(state.get(STATE_KEY, True)))
    panel.link_failover = LinkFailover(panel)
    panel.link_failover.start()
    panel.bind("<Destroy>", lambda e: panel.link_failover.close() if e.widget is panel else None, add="+")
    return panel.link_failover
