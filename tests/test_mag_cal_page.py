"""「校准 · 磁力计校准」页：真实 `DronePanel`，覆盖 R-MAG-1 交付的五条判据。

1. 覆盖度不足时（样本太少，或方向分布退化成单轴旋转）无法进入拟合/提交。
2. 预览 -> 应用到 RAM -> 写入 Flash 两段式，未在确认对话框里确认就不发 COMMIT。
3. `axis_effective` 未验证时，融合状态横幅明确显示"磁力计不参与姿态融合"。
4. 断连 / 换连接代次后不显示上一条链路的数据（状态与采样缓冲都清空）。
5. 固件回包非法 / 缺字段 / 过期（陈旧连接代次）不伪报有效。

协议夹具的字段名与固件真实格式串已经在 `tests/test_mag_protocol.py` 锚定校验过，
本文件复用相同的字面量拼法，不重新验证格式本身，只验证页面对已解码结果的反应。
"""

from __future__ import annotations

import math
import tkinter as tk

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib.connection_state import receive_context
from tools.panel_lib.pages import mag_cal as mag_cal_page


def _sphere_samples(count: int = 200, radius: float = 400.0) -> list[tuple[float, float, float]]:
    """Fibonacci 螺旋球面点：覆盖度判据要求的"多方向"合成数据，仅用于本页单测。"""
    golden_angle = math.pi * (3.0 - math.sqrt(5.0))
    samples = []
    for i in range(count):
        y = 1.0 - (i / (count - 1)) * 2.0
        radius_at_y = math.sqrt(max(0.0, 1.0 - y * y))
        theta = golden_angle * i
        x = math.cos(theta) * radius_at_y
        z = math.sin(theta) * radius_at_y
        samples.append((x * radius, y * radius, z * radius))
    return samples


def _single_axis_samples(count: int = 200, radius: float = 400.0) -> list[tuple[float, float, float]]:
    """只绕一根轴转的退化采样：覆盖度必须拒绝，见 `mag_cal_fit.evaluate_coverage`。"""
    return [
        (radius * math.cos(2.0 * math.pi * i / count), radius * math.sin(2.0 * math.pi * i / count), 0.0)
        for i in range(count)
    ]


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.connection_generation = 1
        self.lines: list[str] = []

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True

    def send_ascii_line(self, line: str) -> bool:
        return self.send_line(line)

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        return True

    def set_binary_sink(self, sink) -> None:
        pass


@pytest.fixture(scope="module")
def app():
    instance = panel.DronePanel()
    yield instance
    instance.destroy()


def select_mag_cal(app) -> None:
    app.notebook.select(app.calibration_group_tab)
    app.calibration_notebook.select(app.mag_cal_tab)
    app.update_idletasks()


@pytest.fixture
def live(app, monkeypatch):
    """干净链路 + 干净页面会话；每个用例都从"刚打开、什么都没读到"开始。"""
    monkeypatch.setattr(type(app), "_save_panel_state", lambda self: None)
    app.transport = FakeTransport()
    app._rx_context = None
    page = app.mag_cal_page
    page.session = None
    page.status = None
    page.bias = None
    page.matrix = None
    page.draft = None
    page.fusion = None
    page.frame = None
    page.status_received_at = None
    page.live_chip_mgauss = None
    page.live_flu_mgauss = None
    page.applied = False
    page.apply_pending = False
    page.draft_acks = set()
    page.unsupported = False
    page.protocol_errors = 0
    page._reset_sampling()
    page.notice.set("")
    page._connection()
    select_mag_cal(app)
    return app


def state_of(widget) -> str:
    return str(widget["state"])


# ------------------------------------------------------------------ 挂载与接线


def test_mag_cal_tab_is_mounted_once(app):
    assert isinstance(app.mag_cal_page, mag_cal_page.MagCalPage)
    labels = [app.calibration_notebook.tab(t, "text") for t in app.calibration_notebook.tabs()]
    assert labels.count(mag_cal_page.MAG_CAL_TAB_TEXT) == 1


def test_board_line_hook_is_reachable_through_the_real_dispatch_chain(live):
    """走 `_handle_board_line`（`dispatch_board_line` 的真实调用点），不是直接调页面方法。"""
    app = live
    page = app.mag_cal_page
    assert page.status is None
    app._handle_board_line(
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1 dirty=0")
    assert page.status is not None
    assert page.status.calibrated is True


# ------------------------------------------------------------------ 1. 覆盖度门控拟合/提交


def test_fit_button_disabled_with_no_samples(live):
    app = live
    page = app.mag_cal_page
    page._render()
    assert state_of(page.fit_button) == tk.DISABLED
    page.run_fit()
    assert page.fit_result is None


def test_fit_blocked_below_minimum_sample_count(live):
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _sphere_samples(count=50)
    page._render()
    assert state_of(page.fit_button) == tk.DISABLED
    page.run_fit()
    assert page.fit_result is None


def test_fit_blocked_for_single_axis_rotation_even_with_enough_samples(live):
    """一个平面内的圆是完全共面的退化输入：`evaluate_coverage` 直接拒绝（不是返回
    `passed=False` 的报告），页面必须原样呈现为"没有覆盖度可看"，拟合按钮照样禁用——
    这仍然是"覆盖度不足时无法进入拟合"的同一条判据，只是走的是更早的退化检测。
    """
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _single_axis_samples(count=200)
    page._render()
    assert page.coverage is None
    assert "coplanar" in page.coverage_var.get() or "共面" in page.coverage_var.get()
    assert state_of(page.fit_button) == tk.DISABLED
    page.run_fit()
    assert page.fit_result is None


def test_fit_succeeds_once_sphere_coverage_passes(live):
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _sphere_samples()
    page._render()
    assert page.coverage is not None
    assert page.coverage.passed is True
    assert state_of(page.fit_button) == tk.NORMAL
    page.run_fit()
    assert page.fit_result is not None
    # A near-perfect sphere already centred at the origin should fit close to
    # zero bias and a near-identity matrix.
    assert page.fit_result.hard_iron_offset_mgauss == pytest.approx((0.0, 0.0, 0.0), abs=2.0)


def test_apply_is_unavailable_without_a_fit_result(live):
    app = live
    page = app.mag_cal_page
    page._render()
    assert state_of(page.apply_button) == tk.DISABLED
    app.transport.lines.clear()
    page.apply_to_ram()
    assert app.transport.lines == []


# ------------------------------------------------------------------ 2. 预览 -> RAM -> Flash 两段式


def test_apply_to_ram_sends_draft_and_apply_in_order(live):
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _sphere_samples()
    page._render()
    page.run_fit()
    assert page.fit_result is not None

    app.transport.lines.clear()
    page.apply_to_ram()
    assert page.apply_pending is True
    lines = app.transport.lines
    assert len(lines) == 5
    assert lines[0].startswith("MAGCAL SET BIAS ")
    assert lines[1].startswith("MAGCAL SET MATRIX 0 ")
    assert lines[2].startswith("MAGCAL SET MATRIX 1 ")
    assert lines[3].startswith("MAGCAL SET MATRIX 2 ")
    assert lines[4] == "MAGCAL APPLY"
    # Nothing has reached Flash yet -- APPLY only ever touches RAM.
    assert not any("COMMIT" in line for line in lines)


def test_apply_progress_reflects_which_draft_parts_were_acknowledged(live):
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _sphere_samples()
    page._render()
    page.run_fit()
    page.apply_to_ram()
    page._render()
    assert "零偏…" in page.apply_progress_var.get()

    page.handle_board_line("MAGCAL state=draft_bias_set")
    page.handle_board_line("MAGCAL state=draft_matrix_row row=0")
    page._render()
    text = page.apply_progress_var.get()
    assert "零偏✓" in text
    assert "矩阵行0✓" in text
    assert "矩阵行1…" in text

    page.handle_board_line("MAGCAL state=applied_ram")
    page._render()
    assert page.apply_progress_var.get() == ""  # cleared once no longer pending


def test_commit_is_refused_before_any_successful_apply(live, monkeypatch):
    app = live
    page = app.mag_cal_page
    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: True)
    app.transport.lines.clear()
    page.commit_to_flash()
    assert app.transport.lines == []


def test_commit_requires_explicit_confirmation_after_apply_succeeds(live, monkeypatch):
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _sphere_samples()
    page._render()
    page.run_fit()
    page.apply_to_ram()
    app.transport.lines.clear()
    page.handle_board_line("MAGCAL state=applied_ram")
    assert page.applied is True
    assert page.apply_pending is False

    # Declining the confirmation dialog must not write Flash.
    app.transport.lines.clear()
    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: False)
    page.commit_to_flash()
    assert app.transport.lines == []

    # Confirming is what actually sends COMMIT -- never automatic.
    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: True)
    page.commit_to_flash()
    assert app.transport.lines == ["MAGCAL COMMIT"]


def test_apply_rejection_keeps_commit_disabled(live):
    pytest.importorskip("numpy")
    app = live
    page = app.mag_cal_page
    page.samples = _sphere_samples()
    page._render()
    page.run_fit()
    page.apply_to_ram()
    page.handle_board_line("MAGCAL state=apply_rejected reason=bad_determinant")
    assert page.applied is False
    assert page.apply_pending is False
    page._render()
    assert state_of(page.commit_button) == tk.DISABLED


def test_clear_calibration_requires_confirmation_and_explains_axis_reset(live, monkeypatch):
    app = live
    page = app.mag_cal_page
    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: False)
    app.transport.lines.clear()
    page.clear_calibration()
    assert app.transport.lines == []

    captured = {}

    def confirm(title, message):
        captured["message"] = message
        return True

    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", confirm)
    page.clear_calibration()
    assert app.transport.lines == ["MAGCAL CLEAR"]
    assert "轴向验证状态" in captured["message"]


def test_verify_axis_requires_the_checkbox_and_confirmation(live, monkeypatch):
    app = live
    page = app.mag_cal_page
    app.transport.lines.clear()
    page.verify_confirmed_var.set(False)
    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: True)
    page.verify_axis()
    assert app.transport.lines == []  # checkbox not ticked: never even prompts

    page.verify_confirmed_var.set(True)
    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: False)
    page.verify_axis()
    assert app.transport.lines == []  # declined the confirmation

    monkeypatch.setattr(mag_cal_page.messagebox, "askyesno", lambda *a, **k: True)
    page.verify_axis()
    assert app.transport.lines == ["MAGFRAME VERIFY CONFIRM"]


# ------------------------------------------------------------------ 3. 融合横幅明确显示参与与否


def test_banner_explains_not_participating_when_axis_unverified(live):
    app = live
    page = app.mag_cal_page
    page.handle_board_line(
        "MAGCAL calibrated=1 axis_verified=0 axis_effective=0 contract_stored=1 contract_live=1 dirty=0")
    page.handle_board_line(
        "MAGCAL fusion subsystem_enabled=0 used=0 field_rejected=0 ignored=0 recovery=0 error_deg=0.00")
    page._render()
    text = page.fusion_banner_var.get()
    assert "不参与姿态融合" in text
    assert "未验证" in text
    assert str(page.fusion_banner_label["style"]) == "Fail.TLabel"


def test_banner_shows_participating_once_used_is_true(live):
    app = live
    page = app.mag_cal_page
    page.handle_board_line(
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1 dirty=0")
    page.handle_board_line(
        "MAGCAL fusion subsystem_enabled=1 used=1 field_rejected=0 ignored=0 recovery=0 error_deg=1.20")
    page._render()
    text = page.fusion_banner_var.get()
    assert "正在参与姿态融合" in text
    assert str(page.fusion_banner_label["style"]) == "Pass.TLabel"


def test_banner_flags_contract_version_drift_as_not_participating(live):
    """`axis_verified` 存储值曾经=1，但坐标契约版本已变化：有效值必须视为未验证。"""
    app = live
    page = app.mag_cal_page
    page.handle_board_line(
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=0 contract_stored=1 contract_live=2 dirty=0")
    page.handle_board_line(
        "MAGCAL fusion subsystem_enabled=0 used=0 field_rejected=0 ignored=0 recovery=0 error_deg=0.00")
    page._render()
    text = page.fusion_banner_var.get()
    assert "不参与姿态融合" in text
    assert "坐标契约版本变化" in text


# ------------------------------------------------------------------ 4. 断连/换代次清空显示与采样


def test_reconnecting_clears_status_and_sample_buffer(live):
    app = live
    page = app.mag_cal_page
    page.handle_board_line(
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1 dirty=0")
    page.samples = _sphere_samples(count=160)
    page.sampling = True
    assert page.status is not None
    assert page.samples

    app.transport.connection_generation += 1
    page._connection()
    assert page.status is None
    assert page.bias is None
    assert page.fusion is None
    assert page.samples == []
    assert page.sampling is False


def test_disconnecting_clears_status(live):
    app = live
    page = app.mag_cal_page
    page.handle_board_line(
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1 dirty=0")
    assert page.status is not None
    app.transport.is_connected = False
    page._connection()
    assert page.status is None
    app.transport.is_connected = True  # restore for fixture teardown hygiene


# ------------------------------------------------------------------ 5. 非法/缺字段/过期回包不伪报有效


def test_malformed_status_line_is_rejected_not_partially_applied(live):
    app = live
    page = app.mag_cal_page
    assert page.status is None
    page.handle_board_line(  # calibrated=9 is out of {0,1} -> MagProtocolError
        "MAGCAL calibrated=9 axis_verified=0 axis_effective=0 contract_stored=1 contract_live=1 dirty=0")
    assert page.status is None
    assert page.protocol_errors == 1


def test_missing_field_status_line_is_rejected(live):
    app = live
    page = app.mag_cal_page
    page.handle_board_line(  # dirty= missing entirely
        "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1")
    assert page.status is None
    assert page.protocol_errors == 1


def test_stale_connection_generation_reply_is_dropped(live):
    app = live
    page = app.mag_cal_page
    assert page.status is None
    stale_context = receive_context(app.transport, generation=app.transport.connection_generation)
    app.transport.connection_generation += 1  # a reconnect happened after this context was captured
    app._rx_context = stale_context
    try:
        page.handle_board_line(
            "MAGCAL calibrated=1 axis_verified=1 axis_effective=1 contract_stored=1 contract_live=1 dirty=0")
        assert page.status is None
    finally:
        app._rx_context = None


def test_unsupported_firmware_is_reported_not_silently_ignored(live):
    app = live
    page = app.mag_cal_page
    page.handle_board_line("ERR unknown cmd MAGCAL")
    assert page.unsupported is True
    page._render()
    assert "未提供磁力计校准命令族" in page.status_summary_var.get()


# ------------------------------------------------------------------ MAG? 原始采样：FLU 旋转与开关


def test_raw_sample_rotates_to_flu_and_only_collects_while_sampling(live):
    app = live
    page = app.mag_cal_page
    page.sampling = False
    page.handle_board_line(
        "MAG ok=1 init=0 st=0 type=QMC5883L addr=0x0D who=0xFF n=1 raw=1,2,3 mgauss=100,-50,25")
    assert page.live_flu_mgauss == (100.0, 50.0, -25.0)
    assert page.samples == []

    page.sampling = True
    page.handle_board_line(
        "MAG ok=1 init=0 st=0 type=QMC5883L addr=0x0D who=0xFF n=2 raw=1,2,3 mgauss=200,10,5")
    assert page.samples == [(200.0, -10.0, -5.0)]

    page.handle_board_line(
        "MAG ok=0 init=1 st=1 type=NONE addr=0x00 who=0x00 n=0 raw=0,0,0 mgauss=0,0,0")
    assert page.live_flu_mgauss is None
    assert page.samples == [(200.0, -10.0, -5.0)]  # not-ok sample never appended


def test_fit_verdict_shows_radius_and_rejects_quarter_scale_field(live):
    """Fit line shows the radius; a field below the firmware gate says unusable."""
    pytest.importorskip("numpy")
    page = live.mag_cal_page
    page.samples = _sphere_samples(radius=400.0)
    page._render()
    page.run_fit()
    page._render()
    assert "拟合半径 400" in page.fit_summary_var.get()
    assert page.fit_verdict_var.get().startswith("可以用")

    page.samples = _sphere_samples(radius=125.0)
    page._render()
    page.run_fit()
    page._render()
    assert page.fit_verdict_var.get().startswith("不能用")
    assert state_of(page.apply_button) == tk.NORMAL
