"""USB CDC 空闲探活回归测试。

背景（2026-08-28 ST-Link 实测确认）：飞控在 USB CDC 上是纯命令/响应通道——
周期性 VOFA 遥测在 App/Src/app_vofa.c 里被 APP_AiWB2_IsSocketReady() 挡住，
只走 Ai-WB2 socket。因此上位机不主动问就永远收不到字节，链路状态会把
"没人问" 误判成 "掉线"，表现为必须手点 PING 才显示已连接、5 秒后又变红。

这些测试锁死修复后的行为：空闲时自动探活、探活不污染验收日志、
在 V1/固件升级独占 USB 时让位，且让位期间不得误报链路异常。
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from tools import drone_tcp_panel as panel


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
VOFA_SOURCE = (ROOT / "App" / "Src" / "app_vofa.c").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    end = source.find("\n    def ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


class RecordingTransport:
    def __init__(self, connected: bool = True, accept: bool = True) -> None:
        self.connected = connected
        self.accept = accept
        self.frames: list[tuple[int, bytes]] = []
        self.lines: list[str] = []

    @property
    def is_connected(self) -> bool:
        return self.connected

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        if not self.accept:
            return False
        self.frames.append((function, payload))
        return True

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True


def keepalive_subject(
    *,
    serial: RecordingTransport | None = None,
    last_board_rx: float = 0.0,
    last_keepalive_tx: float = 0.0,
    v1_alive: bool = False,
    firmware_update_running: bool = False,
    firmware_update_pending: bool = False,
    firmware_programming: bool = False,
    allowed: bool = True,
) -> SimpleNamespace:
    serial = serial if serial is not None else RecordingTransport()
    subject = SimpleNamespace(
        transport=serial,
        serial_transport=serial,
        udp_transport=RecordingTransport(connected=False),
        tcp_transport=RecordingTransport(connected=False),
        structured_protocol_supported=False,
        last_board_rx=last_board_rx,
        last_keepalive_tx=last_keepalive_tx,
        v1_worker=SimpleNamespace(is_alive=lambda: v1_alive) if v1_alive else None,
        firmware_update_running=firmware_update_running,
        firmware_update_pending=firmware_update_pending,
        firmware_programming=firmware_programming,
    )
    subject._transport_connected = lambda: serial.is_connected
    subject._validation_command_allowed = lambda payload: allowed
    subject._send_keepalive_probe = (
        lambda: panel.DronePanel._send_keepalive_probe(subject)
    )
    subject._link_keepalive_suppressed_reason = (
        lambda: panel.DronePanel._link_keepalive_suppressed_reason(subject)
    )
    return subject


# --------------------------------------------------------------------------
# 根因锚点
# --------------------------------------------------------------------------

def test_firmware_has_no_unsolicited_usb_telemetry() -> None:
    """VOFA 帧只在 Ai-WB2 socket 就绪时发出，USB CDC 上没有心跳。

    这条不变量正是上位机必须自己探活的理由；若将来固件真的加了 USB 心跳，
    这个测试会失败，提醒同步放宽 LINK_STALE_S 或移除探活。
    """
    assert "APP_AiWB2_IsSocketReady() == 0U" in VOFA_SOURCE
    assert "APP_USB_CDC_Write" not in VOFA_SOURCE


def test_link_health_check_drives_the_keepalive() -> None:
    body = function_body(SOURCE, "def _check_link_health(self)")
    assert "self._maybe_send_link_keepalive()" in body
    # 旧的裸 5 秒判定必须已被探活感知的判定取代。
    assert "超过 5 秒未收到 STM32 数据" not in body
    assert "LINK_STALE_S" in body


# --------------------------------------------------------------------------
# 探活触发条件
# --------------------------------------------------------------------------

def test_idle_link_is_probed_with_readonly_ping() -> None:
    serial = RecordingTransport()
    subject = keepalive_subject(
        serial=serial, last_board_rx=_monotonic() - 3.0)

    panel.DronePanel._maybe_send_link_keepalive(subject)

    assert serial.frames == [(panel.PROTO_REQ_PING, b"PING")]
    assert subject.last_keepalive_tx > 0.0


def test_never_received_anything_still_gets_probed() -> None:
    """last_board_rx == 0 就是"连上但没人 PING 过"的初始态，必须自动探。"""
    serial = RecordingTransport()
    subject = keepalive_subject(serial=serial, last_board_rx=0.0)

    panel.DronePanel._maybe_send_link_keepalive(subject)

    assert serial.frames == [(panel.PROTO_REQ_PING, b"PING")]


def test_busy_link_is_not_probed() -> None:
    serial = RecordingTransport()
    subject = keepalive_subject(
        serial=serial, last_board_rx=_monotonic() - 0.4)

    panel.DronePanel._maybe_send_link_keepalive(subject)

    assert serial.frames == []


def test_keepalive_is_rate_limited() -> None:
    serial = RecordingTransport()
    now = _monotonic()
    subject = keepalive_subject(
        serial=serial, last_board_rx=now - 30.0, last_keepalive_tx=now - 0.5)

    panel.DronePanel._maybe_send_link_keepalive(subject)

    assert serial.frames == []


def test_disconnected_transport_is_not_probed() -> None:
    serial = RecordingTransport(connected=False)
    subject = keepalive_subject(serial=serial, last_board_rx=0.0)

    panel.DronePanel._maybe_send_link_keepalive(subject)

    assert serial.frames == []


def test_failed_send_does_not_advance_the_rate_limiter() -> None:
    serial = RecordingTransport(accept=False)
    subject = keepalive_subject(
        serial=serial, last_board_rx=_monotonic() - 3.0)

    panel.DronePanel._maybe_send_link_keepalive(subject)

    assert subject.last_keepalive_tx == 0.0


# --------------------------------------------------------------------------
# 让位与只读约束
# --------------------------------------------------------------------------

def test_v1_capture_suppresses_the_keepalive() -> None:
    serial = RecordingTransport()
    subject = keepalive_subject(
        serial=serial, last_board_rx=_monotonic() - 30.0, v1_alive=True)

    assert subject._link_keepalive_suppressed_reason() == "IMU 校准正在独占 USB CDC"
    panel.DronePanel._maybe_send_link_keepalive(subject)
    assert serial.frames == []


def test_firmware_update_suppresses_the_keepalive() -> None:
    for flag in ("firmware_update_running", "firmware_update_pending", "firmware_programming"):
        serial = RecordingTransport()
        subject = keepalive_subject(
            serial=serial, last_board_rx=_monotonic() - 30.0, **{flag: True})

        assert subject._link_keepalive_suppressed_reason() == "固件升级进行中"
        panel.DronePanel._maybe_send_link_keepalive(subject)
        assert serial.frames == [], flag


def test_probe_respects_the_validation_readonly_gate() -> None:
    serial = RecordingTransport()
    subject = keepalive_subject(
        serial=serial, last_board_rx=_monotonic() - 3.0, allowed=False)

    assert panel.DronePanel._send_keepalive_probe(subject) is False
    assert serial.frames == []
    assert serial.lines == []


def test_ping_stays_inside_the_v0_command_allowlist() -> None:
    """探活用的命令必须是 V0 会话本来就允许的只读命令。"""
    assert "PING" in panel.VALIDATION_ALLOWED_COMMANDS


def test_probe_does_not_pollute_the_log_or_last_command() -> None:
    body = function_body(SOURCE, "def _send_keepalive_probe(self)")
    assert "_append(" not in body
    assert "last_cmd_var" not in body


def test_ascii_compat_serial_does_not_send_a_duplicate_ping() -> None:
    """串口 ASCII 兼容模式下 send_frame 已降级为 send_line，不得再补一条。"""
    assert panel.SERIAL_ASCII_COMPAT_MODE is True
    serial = RecordingTransport()
    subject = keepalive_subject(serial=serial, last_board_rx=0.0)
    subject.structured_protocol_supported = False

    assert panel.DronePanel._send_keepalive_probe(subject) is True
    assert serial.lines == []


def test_legacy_tcp_target_gets_the_text_fallback() -> None:
    tcp = RecordingTransport()
    subject = keepalive_subject(last_board_rx=0.0)
    subject.transport = tcp
    subject.tcp_transport = tcp
    subject.structured_protocol_supported = False

    assert panel.DronePanel._send_keepalive_probe(subject) is True
    assert tcp.frames == [(panel.PROTO_REQ_PING, b"PING")]
    assert tcp.lines == ["PING"]


# --------------------------------------------------------------------------
# 状态判定
# --------------------------------------------------------------------------

def health_subject(
    *, idle: float, suppressed: str | None = None, connected: bool = True
) -> SimpleNamespace:
    label = SimpleNamespace(style=None)
    label.configure = lambda style: setattr(label, "style", style)
    text = SimpleNamespace(value="")
    text.set = lambda value: setattr(text, "value", value)
    transport = RecordingTransport(connected=connected)
    subject = SimpleNamespace(
        transport=transport,
        tcp_transport=RecordingTransport(connected=False),
        udp_transport=RecordingTransport(connected=False),
        serial_transport=transport,
        last_board_rx=_monotonic() - idle,
        link_var=text,
        link_status_label=label,
        after=lambda *_args, **_kwargs: None,
    )
    subject._refresh_serial_selection_lock = lambda: None
    subject._maybe_send_link_keepalive = lambda: None
    subject._transport_connected = lambda: transport.is_connected
    subject._transport_label = lambda: "串口"
    subject._link_keepalive_suppressed_reason = lambda: suppressed
    subject._check_link_health = lambda: None  # 供末尾的 after() 重排引用
    return subject


def test_short_idle_stays_green() -> None:
    subject = health_subject(idle=3.0)

    panel.DronePanel._check_link_health(subject)

    assert subject.link_status_label.style == "Pass.TLabel"
    assert "空闲探活中" in subject.link_var.value


def test_probe_silence_beyond_the_stale_window_is_reported_as_failure() -> None:
    subject = health_subject(idle=LINK_STALE_PLUS)

    panel.DronePanel._check_link_health(subject)

    assert subject.link_status_label.style == "Fail.TLabel"
    assert "探活" in subject.link_var.value


def test_suppressed_keepalive_never_reports_a_dead_link() -> None:
    """V1 独占 USB 期间没有回包是预期的，不能显示成固件挂了。"""
    subject = health_subject(idle=LINK_STALE_PLUS, suppressed="V1 正在独占 USB CDC")

    panel.DronePanel._check_link_health(subject)

    assert subject.link_status_label.style == "Warn.TLabel"
    assert "探活暂停" in subject.link_var.value


def test_keepalive_period_leaves_margin_before_the_stale_verdict() -> None:
    """探活周期必须明显短于判定窗口，否则健康链路也会周期性翻红。"""
    worst_case_idle = (
        panel.LINK_KEEPALIVE_IDLE_S + 1.0 + panel.CMD_REPLY_TIMEOUT_MS / 1000.0
    )
    assert worst_case_idle < panel.LINK_STALE_S
    assert panel.LINK_KEEPALIVE_MIN_INTERVAL_S < panel.LINK_KEEPALIVE_IDLE_S


def _monotonic() -> float:
    import time

    return time.monotonic()


LINK_STALE_PLUS = panel.LINK_STALE_S + 1.0
