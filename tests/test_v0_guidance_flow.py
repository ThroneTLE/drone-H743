"""V0 引导可达性回归测试。

V0 页的四项准备清单（连接/快照/零偏/安全输出）与 A/B/C 按钮全部由
_validation_refresh_readiness 依据实时 stabilizer snapshot 解锁，页面文案也明确
承诺"V0会自动轮询IMU"。但 _imu_poll_tick 的 poll_requested 条件里只列了
imu/firmware/v1 三个页签，漏掉了 validation 页签——用户打开 V0 后按引导
"等待准备状态变 ✓"会永远等不到，A/B/C 全程灰着，引导无法走完。

这些测试锁死：V0 页可见即轮询；且在 V1 采集或固件升级独占 USB 时必须让位。
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from tools import drone_tcp_panel as panel


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
VALIDATION_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "validation_v0.py"
).read_text(encoding="utf-8")


def poll_subject(
    *,
    visible_tab: str,
    connected: bool = True,
    v1_alive: bool = False,
    firmware_update_pending: bool = False,
    firmware_update_running: bool = False,
    validation_poll_requested: bool = False,
    imu_poll_enabled: bool = False,
    drift_recording: bool = False,
) -> SimpleNamespace:
    sent: list[tuple[int, str]] = []
    subject = SimpleNamespace(
        imu_last_sample_time=0.0,
        imu_vars={},
        notebook=SimpleNamespace(select=lambda: visible_tab),
        imu_tab="imu",
        firmware_tab="firmware",
        v1_tab="v1",
        validation_tab="validation",
        rc_tab="rc",
        rc_last_poll=0.0,
        _imu_dirty=False,
        _last_imu_draw_ns=0,
        validation_active_stage=None,
        validation_latest_host_time=0.0,
        _last_validation_readiness_ns=0,
        validation_poll_requested=validation_poll_requested,
        # 静止漂移录制期间必须继续取样，哪怕用户切走了页签。
        drift_recording=drift_recording,
        imu_poll_enabled=SimpleNamespace(get=lambda: imu_poll_enabled),
        firmware_update_pending=firmware_update_pending,
        firmware_update_running=firmware_update_running,
        v1_worker=SimpleNamespace(is_alive=lambda: v1_alive) if v1_alive else None,
        imu_last_poll=0.0,
        sent=sent,
    )
    subject._validation_refresh_readiness = lambda: None
    subject._firmware_refresh_safety = lambda: None
    subject._draw_imu_attitude = lambda: None
    subject._validation_abort_active_stage = lambda *_a: None
    subject._transport_connected = lambda: connected
    subject._send_proto_silent = lambda function, payload: (
        sent.append((function, payload)) or True
    )
    subject.after = lambda *_a, **_k: None
    subject._imu_poll_tick = lambda: None  # 供末尾的 after() 重排引用
    subject._rc_render = lambda: None
    # R-T1-3 把两页各自的门控轮询从 _imu_poll_tick 里搬进了页面模块，这里只
    # 需要它们存在；它们各自的行为由 test_flow_monitor_page / test_scope_page
    # 用真实 DronePanel 覆盖，不在这个纯逻辑桩里重复一遍。
    subject._flow_monitor_poll_tick = lambda _now: None
    subject._dashboard_poll_tick = lambda _now: None
    return subject


def polled(subject: SimpleNamespace) -> bool:
    panel.DronePanel._imu_poll_tick(subject)
    return (panel.PROTO_REQ_IMU, "IMU?") in subject.sent


def test_v0_page_polls_as_soon_as_it_is_visible() -> None:
    """这是引导可达性的前提：不轮询就没有快照，准备清单永远不会变 ✓。"""
    assert polled(poll_subject(visible_tab="validation"))


def test_v0_page_does_not_poll_while_disconnected() -> None:
    assert not polled(poll_subject(visible_tab="validation", connected=False))


def test_v0_page_yields_usb_to_v1_capture() -> None:
    assert not polled(poll_subject(visible_tab="validation", v1_alive=True))


def test_v0_page_yields_usb_to_firmware_update() -> None:
    assert not polled(
        poll_subject(visible_tab="validation", firmware_update_running=True))
    assert not polled(
        poll_subject(visible_tab="validation", firmware_update_pending=True))


def test_active_session_keeps_polling_from_any_tab() -> None:
    """会话进行中即使切走页签也要继续取样，否则采集步骤会因快照过期作废。"""
    assert polled(
        poll_subject(visible_tab="overview", validation_poll_requested=True))


def test_unrelated_tab_does_not_poll() -> None:
    assert not polled(poll_subject(visible_tab="overview"))


def test_readiness_copy_promises_the_automatic_poll() -> None:
    """UI 文案承诺自动轮询；轮询条件必须真的覆盖 V0 页，否则文案在说谎。"""
    assert "坐标系校准页会自动轮询 IMU" in VALIDATION_SOURCE
    start = SOURCE.index("def _imu_poll_tick(self)")
    end = SOURCE.index("\n    def ", start + 1)
    body = SOURCE[start:end]
    assert "validation_tab_visible" in body


def test_a_stationary_drift_recording_keeps_polling_from_any_tab() -> None:
    """录 30~60 秒的过程中用户很可能去看别的页；切走就停采会毁掉这次录制。"""
    subject = poll_subject(visible_tab="overview", drift_recording=True)

    panel.DronePanel._imu_poll_tick(subject)

    assert any(command == "IMU?" for _function, command in subject.sent)
