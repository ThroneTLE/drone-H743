"""R-FLOWMOUNT-1：「光流与测距」标定页的安装方向推荐与「写入飞控（需上锁）」。

全部用真实的 `DronePanel()` 驱动：两步采样走真实的 `_flow_cal_start` / 回包解析 /
`_flow_cal_stop`，回包按固件真实格式（app_optical_flow.c 的 `FLOW ok` 行、
app_cmd_flow.c 的 `FLOW comp` 行、app_cmd_arm.c 的 `RSP ... mod=ARM`、app_control.c 的
`OK param` / `PARAM name=`）逐行喂进 `_handle_board_line`。钉这些事：

  1. 两步采完给出推荐与依据；推荐不可用时按钮灰掉并写明原因；
  2. 写入顺序：先现读 ARM 确认上锁 → 两条 PARAM SET 逐条核对回显 → PARAM? 读回；
     任何一步不对就停，已解锁时一条 PARAM SET 都不发；
  3. 证据 JSON：写入过才 target_parameters_written=true，并带写入前后值与依据；
  4. 写入后提示重采，旧两步不再给推荐；重采两步正向符号都对才算验证通过。
"""

from __future__ import annotations

import json
import tkinter as tk
from pathlib import Path

import pytest

from tools import drone_tcp_panel as panel
from tools.panel_lib import arm_banner as arm_banner_module
from tools.panel_lib import parameter_model
from tools.panel_lib.pages import flow_mount as flow_mount_page
from tools.panel_lib.pages import flow_ranging as flow_ranging_page
from tools.panel_lib.proto import PROTO_REQ_PARAM_SET, PROTO_REQ_PARAMS, PROTO_REQ_STATUS


YAW = "airframe.flow_mount_yaw_deg"
MIRROR = "airframe.flow_mount_mirror"


class FakeClock:
    def __init__(self) -> None:
        self.now = 5000.0

    def monotonic(self) -> float:
        return self.now


class FakeTransport:
    is_connected = True

    def __init__(self) -> None:
        self.lines: list[str] = []
        self.frames: list[tuple[int, str]] = []

    def send_line(self, line: str) -> bool:
        self.lines.append(line)
        return True

    def send_frame(self, function: int, payload: bytes, *_args, **_kwargs) -> bool:
        self.frames.append((function, payload.decode("utf-8")))
        return True


@pytest.fixture(scope="module")
def _panel():
    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:  # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        yield instance
    finally:
        instance.destroy()


@pytest.fixture
def app(_panel, monkeypatch, tmp_path):
    # 既有缺陷（与本任务无关，已报主控）：drone_tcp_panel.py 按包导入（tools.drone_tcp_panel）
    # 那一支 try 里漏了 `arm_banner as _panel_arm_banner`，于是任何 `RSP ... mod=ARM` 行走到
    # _handle_rsp_line 都会 NameError。直接 `python tools/drone_tcp_panel.py` 走的是第三支，
    # 不受影响。那个文件只减不增，这里只在测试里补上同一个模块对象。
    monkeypatch.setattr(panel, "_panel_arm_banner", arm_banner_module, raising=False)
    clock = FakeClock()
    monkeypatch.setattr(flow_ranging_page, "time", clock)
    monkeypatch.setattr(flow_ranging_page, "FLOW_RANGE_CALIBRATION_DIR", tmp_path)
    monkeypatch.setattr(flow_mount_page.messagebox, "askokcancel", lambda *a, **k: True)
    _panel.transport = FakeTransport()
    _panel.structured_protocol_supported = True
    _panel.validation_session_active = False
    _panel.flow_cal_collecting = False
    _panel.flow_cal_active_stage = None
    for stage in _panel.flow_cal_samples:
        _panel.flow_cal_samples[stage] = []
    _panel.flow_cal_results.clear()
    _panel.flow_diag_values.clear()
    if getattr(_panel, "flow_mount_timer", None) is not None:
        _panel._flow_mount_cancel_timer()
    _panel._flow_mount_state_ready = False
    _panel._flow_mount_init_state()
    _panel.flow_mount_status_var.set("")
    _panel._flow_mount_refresh()
    _panel.clock = clock
    _panel.sample_ms = 100
    return _panel


def flow_ok(mount: tuple[int, int] | None) -> str:
    line = ("FLOW ok=1 init=0 health=1 attempts=1 recover=0 vel_rej=0 baud=115200 "
            "bytes=4096 frames=200 valid=1 age_ms=10 source=flow vel_valid=1 height_valid=1")
    if mount is not None:
        line += f" mount_yaw={mount[0]} mount_mirror={mount[1]}"
    return line


def flow_comp(app, vx_mm_s: int, vy_mm_s: int) -> str:
    app.sample_ms += 100
    return (
        f"FLOW comp valid=1 sample_ms={app.sample_ms} contract=1 orientation=3 "
        "source=calibrated_body_flu export=canonical_flu "
        f"sensor_vx_mm_s={vx_mm_s} sensor_vy_mm_s={vy_mm_s} "
        f"corr_vx_mm_s={vx_mm_s} corr_vy_mm_s={vy_mm_s}"
    )


def sample_stage(app, stage: str, velocity_mm_s: tuple[int, int],
                 mount: tuple[int, int] | None = (0, 0), count: int = 11) -> dict:
    """真实的一步：选步骤 → 开始 → 每 0.1 s 一次 FLOW?（状态行 + 补偿行）→ 停止并分析。"""
    app.flow_cal_stage_var.set(flow_ranging_page.FLOW_CALIBRATION_STAGES[stage])
    app._flow_cal_start()
    for _ in range(count):
        app.clock.now += 0.1
        app._handle_board_line(flow_ok(mount))
        app._handle_board_line(flow_comp(app, *velocity_mm_s))
    app._flow_cal_stop()
    return app.flow_cal_results[stage]


def button_state(app) -> str:
    return str(app.flow_mount_write_button.cget("state"))


def feed(app, *lines: str) -> None:
    for line in lines:
        app._handle_board_line(line)


ARM_DISARMED = ("RSP id=0 mod=ARM op=STATUS armed=0 block=none blinks=0 known=1 rc_seen=1 "
                "rc_ok=1 switch=0 throttle_low=1 imu=1 imu_health=1 frame=1 airframe=1 "
                "servo_cal_idle=1 accept_idle=1 airframe_missing=- battery_ok=1")


def echo(name: str, value: int) -> tuple[str, str]:
    return (f"OK param name={name} value={value}.000000",
            f"PARAM name={name} value={value}.000000")


def sets(app) -> list[str]:
    return [payload for function, payload in app.transport.frames if function == PROTO_REQ_PARAM_SET]


def rotated_setup(app) -> None:
    """模块转了 90°：沿机头推，旧换算读成 +Y；向左推读成 −X。推荐应是转 270°、不镜像。"""
    sample_stage(app, "forward_x", (0, 500))
    sample_stage(app, "left_y", (-500, 0))


# ── 推荐 ─────────────────────────────────────────────────────────────────


def test_both_steps_give_a_recommendation_with_its_basis(app) -> None:
    rotated_setup(app)
    forward = app.flow_cal_results["forward_x"]
    assert forward["positive_sign_ok"] is False
    # 采样期间的安装参数记进了这一步的结果（随证据落盘）。
    assert (forward["flow_mount_yaw_deg"], forward["flow_mount_mirror"]) == (0, 0)
    text = app.flow_mount_text_var.get()
    assert "建议写入：旋转 270°、不镜像（当前：旋转 0°、不镜像）" in text
    assert "依据：+X 步位移 0.500 m" in text and "+Y 步位移 0.500 m" in text
    assert button_state(app) == "normal"
    assert app.flow_mount_recommendation["yaw_deg"] == 270


def test_unusable_step_disables_the_button_and_says_why(app) -> None:
    sample_stage(app, "forward_x", (0, 20))       # 1 s 只挪了 2 cm
    sample_stage(app, "left_y", (-500, 0))
    assert "暂无推荐" in app.flow_mount_text_var.get()
    assert "没推动" in app.flow_mount_text_var.get()
    assert button_state(app) == "disabled"
    app._flow_mount_write()
    assert app.transport.frames == []
    assert "没有可写入的推荐" in app.flow_mount_status_var.get()


def test_old_firmware_without_mount_keys_gets_no_recommendation(app) -> None:
    sample_stage(app, "forward_x", (0, 500), mount=None)
    sample_stage(app, "left_y", (-500, 0), mount=None)
    assert "固件没有上报安装参数" in app.flow_mount_text_var.get()
    assert button_state(app) == "disabled"


def test_steps_taken_under_different_mounts_are_not_combined(app) -> None:
    sample_stage(app, "forward_x", (0, 500), mount=(0, 0))
    sample_stage(app, "left_y", (-500, 0), mount=(90, 0))
    assert "不同的安装参数" in app.flow_mount_text_var.get()
    assert button_state(app) == "disabled"


def test_live_flow_line_shows_the_reported_mount(app) -> None:
    feed(app, flow_ok((180, 1)))
    live = app.flow_cal_live_var.get()
    assert "mount_yaw=180 mount_mirror=1" in live


# ── 写入事务 ─────────────────────────────────────────────────────────────


def test_write_reads_arm_then_sets_each_param_then_reads_back_and_saves_evidence(app) -> None:
    rotated_setup(app)
    app._flow_mount_write()
    assert app.transport.frames == [(PROTO_REQ_STATUS, flow_mount_page.FLOW_MOUNT_ARM_REQUEST)]
    assert button_state(app) == "disabled"       # 进行中不许连点

    feed(app, ARM_DISARMED)
    assert sets(app) == [f"PARAM SET {YAW} 270"]
    feed(app, *echo(YAW, 270))
    assert sets(app) == [f"PARAM SET {YAW} 270", f"PARAM SET {MIRROR} 0"]
    feed(app, *echo(MIRROR, 0))
    assert app.transport.frames[-1] == (PROTO_REQ_PARAMS, "PARAM?")
    assert app.flow_mount_write_record is None   # 读回之前不算成功

    feed(app,
         "PARAM name=coax.rate_roll_kp value=0.249000",
         f"PARAM name={YAW} value=270.000000",
         f"PARAM name={MIRROR} value=0.000000")
    status = app.flow_mount_status_var.get()
    assert "已写入并读回确认：旋转 270°、不镜像" in status
    assert "请重采 +X/+Y 两步验证，两步都显示正向符号正确才算通过" in status
    # 旧两步是在 0° 下采的：不再给推荐，按钮灰掉。
    assert "请重采 +X/+Y 两步验证" in app.flow_mount_text_var.get()
    assert button_state(app) == "disabled"

    app._flow_cal_save_report()
    report_path = Path(app.flow_cal_status_var.get().split("：", 1)[1])
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["target_parameters_written"] is True
    write = report["flow_mount"]["write"]
    assert write["before"] == {"yaw_deg": 0, "mirror": 0}
    assert write["after"] == {"yaw_deg": 270, "mirror": 0}
    assert write["readback"] == {YAW: "270.000000", MIRROR: "0.000000"}
    assert write["parameters"] == [YAW, MIRROR]
    assert set(write["basis_stages"]) == {"forward_x", "left_y"}
    assert write["recommendation"]["yaw_deg"] == 270
    assert report["stages"]["forward_x"]["flow_mount_yaw_deg"] == 0

    # 飞控的状态行报出新值，页面照实显示；重采两步（新参数下读数已纠正）→ 验证通过。
    feed(app, flow_ok((270, 0)))
    assert "mount_yaw=270 mount_mirror=0" in app.flow_cal_live_var.get()
    sample_stage(app, "forward_x", (500, 0), mount=(270, 0))
    sample_stage(app, "left_y", (0, 500), mount=(270, 0))
    text = app.flow_mount_text_var.get()
    assert "写入后重采验证：通过" in text
    assert "当前安装参数已正确：旋转 270°、不镜像，无需写入" in text
    assert button_state(app) == "disabled"
    evidence = app._flow_mount_evidence()
    assert evidence["flow_mount"]["verification"]["passed"] is True


def test_armed_aircraft_gets_no_param_set_at_all(app) -> None:
    rotated_setup(app)
    app._flow_mount_write()
    feed(app, ARM_DISARMED.replace("armed=0", "armed=1"))
    assert sets(app) == []
    assert "解锁" in app.flow_mount_status_var.get()
    assert app.flow_mount_txn is None
    assert button_state(app) == "normal"          # 上锁后可以再试
    assert app._flow_mount_evidence()["target_parameters_written"] is False


def test_rejected_param_stops_before_the_second_write(app) -> None:
    rotated_setup(app)
    app._flow_mount_write()
    feed(app, ARM_DISARMED, f"ERR param target {YAW}")
    assert sets(app) == [f"PARAM SET {YAW} 270"]
    assert "飞控拒绝" in app.flow_mount_status_var.get()
    assert app.flow_mount_write_record is None


def test_wrong_echo_value_stops_the_write(app) -> None:
    rotated_setup(app)
    app._flow_mount_write()
    feed(app, ARM_DISARMED, f"OK param name={YAW} value=90.000000")
    assert sets(app) == [f"PARAM SET {YAW} 270"]
    assert "不符" in app.flow_mount_status_var.get()


def test_readback_mismatch_is_not_success(app) -> None:
    rotated_setup(app)
    app._flow_mount_write()
    feed(app, ARM_DISARMED, *echo(YAW, 270), *echo(MIRROR, 0))
    # 读回的转角不是 270（例如被别的地方又改了）：不能记成已写入。
    feed(app, f"PARAM name={YAW} value=0.000000", f"PARAM name={MIRROR} value=0.000000")
    status = app.flow_mount_status_var.get()
    assert "读回" in status and "不符" in status
    assert "已被飞控接受的" in status               # 如实报告哪些已经写进去了
    assert app._flow_mount_evidence()["target_parameters_written"] is False


def test_timeout_fails_the_step_it_was_waiting_for(app) -> None:
    rotated_setup(app)
    app._flow_mount_write()
    token = app.flow_mount_txn["token"]
    app._flow_mount_timeout(token, "set_yaw")         # 不是当前这一步：忽略
    assert app.flow_mount_txn is not None
    app._flow_mount_timeout(token, "arm")
    assert app.flow_mount_txn is None
    assert "超时" in app.flow_mount_status_var.get()
    feed(app, ARM_DISARMED)                           # 迟到的回包不能再触发写入
    assert sets(app) == []


def test_cancelled_confirmation_sends_nothing(app, monkeypatch) -> None:
    rotated_setup(app)
    monkeypatch.setattr(flow_mount_page.messagebox, "askokcancel", lambda *a, **k: False)
    app._flow_mount_write()
    assert app.transport.frames == []
    assert "已取消" in app.flow_mount_status_var.get()


def test_host_capabilities_mirror_the_firmware_value_sets() -> None:
    ok, _ = parameter_model.validate_parameter_text(YAW, "90")
    assert ok
    ok, reason = parameter_model.validate_parameter_text(YAW, "45")
    assert not ok and "0/90/180/270" in reason
    ok, _ = parameter_model.validate_parameter_text(MIRROR, "1")
    assert ok
    ok, reason = parameter_model.validate_parameter_text(MIRROR, "0.5")
    assert not ok and "0/1" in reason
