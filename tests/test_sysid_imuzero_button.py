"""系统辨识页的「IMU 重新标定」按钮（作者 2026-10-01："你给我上位机来一个IMUZERO的按钮"）。"""

from __future__ import annotations

from pathlib import Path

from tools.panel_lib.pages.sysid.imuzero_panel import IMUZERO_READBACK_MS, ImuZeroPanel

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


class _Var:
    def __init__(self) -> None:
        self.value = ""

    def set(self, value: str) -> None:
        self.value = value

    def get(self) -> str:
        return self.value


class _Panel:
    def __init__(self) -> None:
        self.scheduled = []

    def after(self, delay, callback) -> None:
        self.scheduled.append((delay, callback))


class _Engine(ImuZeroPanel):
    def __init__(self, sent_ok: bool = True) -> None:
        self.panel = _Panel()
        self.sent = []
        self.sent_ok = sent_ok
        self.imuzero_var = _Var()
        self._imuzero_pending = 0

    def send(self, text: str, *, quiet: bool = False) -> bool:
        self.sent.append((text, quiet))
        return self.sent_ok


def test_click_sends_imuzero_then_quiet_readbacks() -> None:
    e = _Engine()
    e.imuzero_request()
    assert e.sent == [("IMUZERO", False)]
    assert [d for d, _ in e.panel.scheduled] == list(IMUZERO_READBACK_MS)
    assert "扶稳" in e.imuzero_var.get()
    e.panel.scheduled[0][1]()
    assert e.sent[-1] == ("IMUZERO?", True)


def test_ready_reply_stops_further_readbacks() -> None:
    e = _Engine()
    e.imuzero_request()
    assert e.imuzero_handle_line("IMUZERO gyro_bias=1 attitude_zero=1 ferr_cdeg=34")
    assert "标定完成" in e.imuzero_var.get() and "0.34°" in e.imuzero_var.get()
    for _, cb in e.panel.scheduled:
        cb()
    assert e.sent == [("IMUZERO", False)], "已就绪就不再回读"


def test_not_ready_after_last_readback_asks_to_retry() -> None:
    e = _Engine()
    e.imuzero_request()
    for _ in IMUZERO_READBACK_MS:
        e.imuzero_handle_line("IMUZERO gyro_bias=1 attitude_zero=0 ferr_cdeg=208")
    assert "再点一次" in e.imuzero_var.get()


def test_armed_and_unsent_cases() -> None:
    e = _Engine()
    e.imuzero_request()
    assert e.imuzero_handle_line("IMUZERO state=armed_blocked")
    assert "先上锁" in e.imuzero_var.get()
    assert e.imuzero_handle_line("IMUZERO state=restarted keep_still_ms=3000")
    assert not e.imuzero_handle_line("SYSID THR auto=1")
    blocked = _Engine(sent_ok=False)
    blocked.imuzero_request()
    assert blocked.panel.scheduled == [] and blocked.imuzero_var.get() == ""


def test_button_is_on_all_three_sysid_pages_and_replies_are_routed() -> None:
    inner = read("tools/panel_lib/pages/sysid/inner_loop.py")
    assert "self.build_imuzero_button(box)" in inner
    assert "e.build_imuzero_button(box)" in read("tools/panel_lib/pages/sysid/altitude.py")
    assert "e.build_imuzero_button(box)" in read("tools/panel_lib/pages/sysid/horizontal.py")
    assert "self._init_imuzero_vars()" in inner
    assert inner.index("self.imuzero_handle_line(text)") < inner.index("self.workflow.on_line(text)")
    assert "IMUZERO" not in read("tools/drone_tcp_panel.py"), "新功能不进面板巨文件"
