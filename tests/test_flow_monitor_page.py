"""R-S1-2：“传感器 · 光流”实时监控页。

这里全部用真实的 `DronePanel()` 驱动，不做源码文本断言（那部分在
`tests/test_ground_calibration.py`）。钉四件事：

  1. 轮询有自己的可见性门控——只有本页被选中时才发 `FLOW?`；
  2. 质量 / 高度 / 速度读数与速度曲线缓冲跟着回包走，且 `FLOW comp` 那行的
     `valid=` 不许冲掉状态行的有效性标志；
  3. 累计位移按真实经过时间积分，速度无效时暂停，长空档不外推；
  4. “重置”只清本地累加，一帧都不发。

另外回归一条边界：校准页 `flow_ranging.py` 的解析路径必须照常工作。
"""

from __future__ import annotations

import tkinter as tk
from pathlib import Path

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib.pages import flow_monitor as flow_monitor_page


class FakeClock:
    """替掉 flow_monitor 模块里的 time，让积分的“真实经过时间”可控可断言。"""

    def __init__(self) -> None:
        self.now = 1000.0

    def monotonic(self) -> float:
        return self.now

    def monotonic_ns(self) -> int:
        return int(self.now * 1_000_000_000)

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.lines: list[str] = []

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True

    def send_frame(self, *_args, **_kwargs) -> bool:
        return True


def flow_reply(
    *, valid: int = 1, vel_valid: int = 1, height_valid: int = 1,
    quality: int = 180, vx_mm_s: int = 0, vy_mm_s: int = 0, height_mm: int = 400,
    fw_disp_x_mm: int = 0, fw_disp_y_mm: int = 0, fw_steps: int = 0,
) -> list[str]:
    """一次 `FLOW?` 的真实回包形状：4 行来自 APP_OpticalFlow_Report + 2 行补偿快照。"""
    return [
        (
            "FLOW ok=1 init=0 health=1 attempts=1 recover=0 vel_rej=0 baud=19200 "
            f"bytes=1024 frames=64 valid={valid} age_ms=12 source=flow "
            f"vel_valid={vel_valid} height_valid={height_valid}"
        ),
        (
            "FLOW mico dev=0x0F sys=0x01 msg=0x51 seq=7 t_ms=1234 dist_mm=405 "
            "dist_valid=1 range_q=90 dist_age=5 flow_vx=3 flow_vy=-2 filt_vx=3 "
            f"filt_vy=-2 filt_ready=1 quality={quality} min_q=80 flow_st=0 "
            "flow_age=10 sample_us=10000"
        ),
        (
            f"FLOW data height_raw_mm=405 height_mm={height_mm} vz_mm_s=0 "
            f"vx_mm_s={vx_mm_s} vy_mm_s={vy_mm_s} cksum=0 frame_err=0 ignored=0 "
            "short=0 rst=0 dma_evt=1 dma_size=9 uerr=0 last_err=0x0"
        ),
        (
            "FLOW raw n=5 vx_avg=3 vy_avg=-2 dt_avg=10000 dist_avg=405 "
            "strength_avg=90 q_avg=180 vx_pp=1 vy_pp=1 dt_pp=10 dist_pp=2"
        ),
        # R-M5-5 追加的固件侧导航行：融合速度 + 传感器时间轴积分出来的位移。
        (
            f"FLOW nav pos_x_mm={fw_disp_x_mm} pos_y_mm={fw_disp_y_mm} "
            f"disp_x_mm={fw_disp_x_mm} disp_y_mm={fw_disp_y_mm} "
            f"vel_x_mm_s=0 vel_y_mm_s=0 steps={fw_steps} dt_us=10000"
        ),
        # 这一行的 valid 说的是旋转补偿快照，不是光流本身。
        "FLOW comp valid=0 export=canonical_flu reason=no_snapshot",
    ]


@pytest.fixture(scope="module")
def _panel():
    """整个模块共用一个 Tk root：反复建 DronePanel 会把 Tcl 初始化拖垮。"""
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:  # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        yield instance
    finally:
        instance.destroy()


@pytest.fixture
def app(_panel, monkeypatch):
    """复用同一个面板，但把监控页的流状态清干净——控件引用要留着。"""
    clock = FakeClock()
    monkeypatch.setattr(flow_monitor_page, "time", clock)
    _panel.flow_monitor_values.clear()
    _panel.flow_monitor_samples.clear()
    _panel.flow_monitor_track.clear()
    _panel.flow_monitor_anchor = None
    _panel.flow_monitor_dx_m = 0.0
    _panel.flow_monitor_dy_m = 0.0
    _panel.flow_monitor_frames = 0
    _panel.flow_monitor_invalid_frames = 0
    _panel.flow_monitor_last_poll = 0.0
    _panel.flow_monitor_last_render_ns = 0
    _panel.flow_monitor_dirty = False
    _panel.flow_diag_values.clear()
    _panel._flow_monitor_refresh_readouts()
    _panel.transport = FakeTransport()
    _panel.clock = clock
    _panel.update_idletasks()
    return _panel


def select_flow_tab(app) -> None:
    app.notebook.select(app.sensor_group_tab)
    app.sensor_notebook.select(app.flow_sensor_tab)
    app.update_idletasks()


def feed(app, **kwargs) -> None:
    for line in flow_reply(**kwargs):
        app._handle_board_line(line)


# ---------------------------------------------------------------- 轮询门控


def test_flow_poll_only_runs_while_the_flow_tab_is_selected(app) -> None:
    app.notebook.select(app.sensor_group_tab)
    app.sensor_notebook.select(app.baro_tab)
    app.update_idletasks()
    app.flow_monitor_last_poll = 0.0
    app._imu_poll_tick()
    assert "FLOW?" not in app.transport.lines
    assert app.flow_monitor_tab_visible is False

    select_flow_tab(app)
    app.flow_monitor_last_poll = 0.0
    app._imu_poll_tick()
    assert app.flow_monitor_tab_visible is True
    assert app.transport.lines.count("FLOW?") == 1

    # 节流：同一个周期内不再重复发。
    app._imu_poll_tick()
    assert app.transport.lines.count("FLOW?") == 1


def test_flow_poll_does_not_depend_on_the_calibration_collect_switch(app) -> None:
    """标定页那个 flow_cal_collecting 关着，本页照样要能拿到数据。"""
    assert app.flow_cal_collecting is False
    select_flow_tab(app)
    app.flow_monitor_last_poll = 0.0
    app._imu_poll_tick()
    assert "FLOW?" in app.transport.lines


# ---------------------------------------------------------------- 读数


def test_readouts_follow_the_reply_and_survive_the_comp_line(app) -> None:
    feed(app, quality=190, height_mm=612, vx_mm_s=250, vy_mm_s=-125)

    assert app.flow_monitor_vars["valid"].get() == "有效"
    assert app.flow_monitor_vars["quality"].get() == "190 / 最低 80"
    assert app.flow_monitor_quality_value.get() == 190.0
    assert "0.612 m" in app.flow_monitor_vars["height"].get()
    assert "vx +0.250" in app.flow_monitor_vars["velocity"].get()
    assert "vy -0.125" in app.flow_monitor_vars["velocity"].get()
    assert app.flow_monitor_vars["age"].get() == "12 ms"
    assert len(app.flow_monitor_samples) == 1


def test_invalid_data_is_marked_and_greys_out_nothing_silently(app) -> None:
    feed(app, valid=0, vel_valid=0, height_valid=0, vx_mm_s=500, vy_mm_s=500)

    assert app.flow_monitor_vars["valid"].get() == "数据无效"
    assert "数据无效" in app.flow_monitor_vars["height"].get()
    assert "未计入累计位移" in app.flow_monitor_vars["velocity"].get()
    assert str(app.flow_monitor_value_labels["valid"]["style"]) == "Fail.TLabel"
    assert str(app.flow_monitor_value_labels["velocity"]["style"]) == "Fail.TLabel"

    feed(app)
    assert str(app.flow_monitor_value_labels["valid"]["style"]) == "Pass.TLabel"


def test_sample_buffer_is_bounded(app) -> None:
    assert app.flow_monitor_samples.maxlen == flow_monitor_page.FLOW_MONITOR_MAX_SAMPLES
    assert app.flow_monitor_track.maxlen == flow_monitor_page.FLOW_MONITOR_MAX_TRACK_POINTS
    for _ in range(flow_monitor_page.FLOW_MONITOR_MAX_SAMPLES + 25):
        app.clock.advance(0.2)
        feed(app, vx_mm_s=100, vy_mm_s=0)
    assert len(app.flow_monitor_samples) == flow_monitor_page.FLOW_MONITOR_MAX_SAMPLES


# ---------------------------------------------------------------- 累计位移


def test_displacement_integrates_the_real_elapsed_time(app) -> None:
    feed(app, vx_mm_s=1000, vy_mm_s=0)          # 1.000 m/s，只落锚点
    assert app.flow_monitor_dx_m == pytest.approx(0.0)

    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000, vy_mm_s=-500)       # 梯形：x 1.0m/s×0.2s
    assert app.flow_monitor_dx_m == pytest.approx(0.2)
    assert app.flow_monitor_dy_m == pytest.approx(0.5 * (0.0 - 0.5) * 0.2)

    # 同样两帧、不同的真实间隔，位移必须不同——不是按固定周期数帧。
    app.clock.advance(0.4)
    feed(app, vx_mm_s=1000, vy_mm_s=-500)
    assert app.flow_monitor_dx_m == pytest.approx(0.6)
    assert app.flow_monitor_vars["dx"].get() == "+0.600 m"
    assert len(app.flow_monitor_track) == 2


def test_invalid_velocity_pauses_integration_instead_of_counting_zero(app) -> None:
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    baseline = app.flow_monitor_dx_m
    assert baseline == pytest.approx(0.2)

    # 一段速度无效的采样：既不加位移，也不把锚点留给它后面那一帧。
    for _ in range(3):
        app.clock.advance(0.2)
        feed(app, vel_valid=0, vx_mm_s=9000)
    assert app.flow_monitor_dx_m == pytest.approx(baseline)
    assert app.flow_monitor_anchor is None
    assert app.flow_monitor_invalid_frames == 3

    # 恢复后的第一帧只重新落锚，第二帧才继续积。
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    assert app.flow_monitor_dx_m == pytest.approx(baseline)
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    assert app.flow_monitor_dx_m == pytest.approx(baseline + 0.2)


def test_long_gap_is_not_extrapolated(app) -> None:
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    app.clock.advance(30.0)     # 切页/断连造成的空档
    feed(app, vx_mm_s=1000)
    assert app.flow_monitor_dx_m == pytest.approx(0.0)
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    assert app.flow_monitor_dx_m == pytest.approx(0.2)


def test_reset_clears_local_state_only_and_sends_nothing(app) -> None:
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000, vy_mm_s=1000)
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000, vy_mm_s=1000)
    assert app.flow_monitor_dx_m > 0.0
    assert app.flow_monitor_track

    sent_before = list(app.transport.lines)
    app._flow_monitor_reset_displacement()

    assert app.flow_monitor_dx_m == 0.0
    assert app.flow_monitor_dy_m == 0.0
    assert not app.flow_monitor_track
    assert app.flow_monitor_anchor is None
    assert app.flow_monitor_vars["dx"].get() == "+0.000 m"
    assert app.flow_monitor_vars["dy"].get() == "+0.000 m"
    # 作者授权后新增：固件那份里程计只能由飞控自己清，所以补发一条 FLOW ZERO。
    # 它只动累计位移，不碰控制位置 / 速度估计 / EKF 协方差。
    assert app.transport.lines == sent_before + ["FLOW ZERO"]


# ---------------------------------------------------------------- 绘图节流


def test_plot_redraw_is_throttled_and_skipped_while_hidden(app, monkeypatch) -> None:
    if not flow_monitor_page.HAS_MATPLOTLIB:
        pytest.skip("matplotlib 未安装，曲线区停用")
    draws: list[str] = []
    monkeypatch.setattr(
        app.flow_monitor_velocity_canvas, "draw_idle", lambda: draws.append("v")
    )
    monkeypatch.setattr(
        app.flow_monitor_track_canvas, "draw_idle", lambda: draws.append("t")
    )

    app.notebook.select(app.sensor_group_tab)
    app.sensor_notebook.select(app.baro_tab)
    app.update_idletasks()
    app._imu_poll_tick()
    feed(app, vx_mm_s=100)
    app._flow_monitor_tick()
    assert draws == [], "页面不可见时不重绘"

    select_flow_tab(app)
    # Revealing a matplotlib canvas may schedule its own backend resize repaint;
    # the contract below measures the page's throttled data redraws.
    draws.clear()
    app._imu_poll_tick()
    app.clock.advance(1.0)
    app._flow_monitor_tick()
    assert draws == ["v", "t"]

    # 同一个节流窗口内再来一帧不重绘。
    app.clock.advance(0.05)
    feed(app, vx_mm_s=100)
    app._flow_monitor_tick()
    assert draws == ["v", "t"]

    app.clock.advance(1.0)
    app._flow_monitor_tick()
    assert draws == ["v", "t", "v", "t"]


# ---------------------------------------------------------------- 边界回归


def test_calibration_page_parsing_still_works(app) -> None:
    """监控页只是加了一份并行消费者，校准页那条解析路径必须照常。"""
    app.flow_diag_values.clear()
    feed(app, quality=133, height_mm=800)
    assert app.flow_diag_values.get("quality") == "133"
    assert app.flow_diag_values.get("height_mm") == "800"
    assert "q=133/80" in app.flow_cal_live_var.get()
    # 反过来：校准页把所有 FLOW 行 merge 进一个字典（含 comp 行的 valid），
    # 监控页按段收，所以两边的 valid 现在本来就不是同一个值。
    assert app.flow_diag_values.get("valid") == "0"
    assert app.flow_monitor_values["valid"] == "1"


# ------------------------------------------------- 固件侧累计位移（R-M5-5 之后）


def test_firmware_displacement_is_shown_alongside_the_host_one(app) -> None:
    """R-M5-5 之后固件自己也积位移，页面不能再声称“固件并不保存这个量”。"""
    feed(app, fw_disp_x_mm=1250, fw_disp_y_mm=-380, fw_steps=417)

    assert app.flow_monitor_vars["fw_dx"].get() == "+1.250 m"
    assert app.flow_monitor_vars["fw_dy"].get() == "-0.380 m"
    assert "417 步" in app.flow_monitor_vars["fw_steps"].get()
    assert "10000 µs" in app.flow_monitor_vars["fw_steps"].get()
    # 两份口径互不干扰：固件那份不进上位机的积分状态。
    assert app.flow_monitor_dx_m == pytest.approx(0.0)
    assert len(app.flow_monitor_samples) == 1


def test_firmware_displacement_reports_absence_instead_of_a_fake_zero(app) -> None:
    """老固件不发 FLOW nav 时必须如实标注，不能显示成 0.000 m 让人以为没动。"""
    for line in flow_reply():
        if not line.startswith("FLOW nav "):
            app._handle_board_line(line)

    for key in ("fw_dx", "fw_dy", "fw_steps"):
        assert "固件未上报" in app.flow_monitor_vars[key].get()


def test_local_reset_does_not_touch_the_firmware_accumulator(app) -> None:
    feed(app, fw_disp_x_mm=1250, fw_steps=417)
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000, fw_disp_x_mm=1250, fw_steps=417)

    sent_before = list(app.transport.lines)
    app.flow_monitor_reset_button.invoke()

    assert app.flow_monitor_dx_m == 0.0
    # 本地立刻清零；固件那份要等它自己处理完 FLOW ZERO 再回包，所以显示值不变。
    assert app.flow_monitor_vars["fw_dx"].get() == "+1.250 m"
    assert app.transport.lines == sent_before + ["FLOW ZERO"]


def test_page_text_no_longer_claims_the_firmware_has_no_displacement() -> None:
    source = (
        Path(flow_monitor_page.__file__).read_text(encoding="utf-8")
    )
    assert "固件并不保存这个量" not in source
    assert "累计位移（上位机按真实时间积分）" not in source
    assert "FLOW nav" in source


def test_reset_falls_back_to_local_only_when_disconnected(app) -> None:
    """没连上时本地照清，不能因为发不出去就整个不动。"""
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    app.clock.advance(0.2)
    feed(app, vx_mm_s=1000)
    assert app.flow_monitor_dx_m > 0.0

    app.transport.is_connected = False
    try:
        app.flow_monitor_reset_button.invoke()
    finally:
        app.transport.is_connected = True

    assert app.flow_monitor_dx_m == 0.0
    assert app.transport.lines == []
