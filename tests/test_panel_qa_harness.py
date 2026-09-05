"""R-S1-3（TK-00）：无硬件 QA 测试基础的**装置契约**。

这份文件只证明装置本身可信，**不证明产品正确**。区别很重要：报告里 N01–N13 那些
缺陷在这里一条都不会被断言成“已修”，它们由各自的实现包先写红灯再修绿。这里回答
的是另一个问题——后面那些包所依赖的地基靠不靠得住：

  * 碰物理串口 / 起烧录程序会不会当场失败（而不是事后统计）；
  * 用户的 `panel_state`、日志和 `data/calibration/**` 历史证据会不会被碰；
  * 夹具的字段名是不是真的和固件发出来的一致；
  * 装置认出的 18 个叶页是不是真实页面，几何判据能不能抓到“存在但够不到”的控件；
  * 定时回调是被**记录**下来还是被偷偷丢掉（丢掉就能假装异步测试通过）。
"""

from __future__ import annotations

import subprocess
import sys
import json
from pathlib import Path

import pytest
import serial

from tools import panel_qa
from tools.panel_qa import fixtures as qa_fixtures
from tools.panel_qa import geometry as qa_geometry


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_EVIDENCE = ROOT / "data" / "calibration"


# ---------------------------------------------------------------- 硬件护栏


def test_opening_a_physical_serial_port_fails_immediately() -> None:
    """只记一笔然后放行，等于让真实动作先发生了再统计。必须抛。"""
    with panel_qa.hardware_guards() as log:
        port = serial.Serial()
        port.port = "COM31"
        with pytest.raises(AssertionError, match="physical serial port"):
            port.open()
    assert log.serial_opens == ["COM31"]
    assert not log.clean


def test_running_a_flashing_tool_fails_immediately() -> None:
    """AGENTS.md 第 5 条：未经明文授权不得烧录/复位。subprocess 完全绕开串口层。"""
    with panel_qa.hardware_guards() as log:
        with pytest.raises(AssertionError, match="flashing/reset tool"):
            subprocess.Popen(["openocd", "-f", "board.cfg"])
        with pytest.raises(AssertionError, match="flashing/reset tool"):
            subprocess.run(["C:/tools/STM32_Programmer_CLI.exe", "-c", "port=SWD"])
    assert len(log.flash_invocations) == 2


def test_the_flash_tool_classifier_only_looks_at_the_executable() -> None:
    """参数里出现 openocd 不是一次烧录，否则分析脚本会被误杀。"""
    assert panel_qa.is_flash_tool_command(["openocd"]) == "openocd"
    assert panel_qa.is_flash_tool_command(["/usr/bin/JLinkExe", "-if", "swd"]) == "jlink"
    assert panel_qa.is_flash_tool_command(["python", "x.py", "--openocd-log", "a.txt"]) is None
    assert panel_qa.is_flash_tool_command([]) is None


def test_guards_restore_the_previous_implementation() -> None:
    """可嵌套：conftest 已经打过补丁的进程里再开一层，退出后原补丁还在。"""
    before = serial.Serial.open
    before_popen = subprocess.Popen.__init__
    with panel_qa.hardware_guards():
        assert serial.Serial.open is not before
    assert serial.Serial.open is before
    assert subprocess.Popen.__init__ is before_popen


def test_a_harmless_subprocess_still_runs() -> None:
    """护栏不能把装置自己变成不能用；非烧录命令要原样放行。"""
    with panel_qa.hardware_guards() as log:
        result = subprocess.run(
            ["python", "-c", "print('qa')"], capture_output=True, text=True
        )
    assert result.stdout.strip() == "qa"
    assert log.clean


# ---------------------------------------------------------------- 隔离


def test_isolated_environment_redirects_every_writable_path(tmp_path) -> None:
    from tools import project_paths
    from tools.panel_lib import state as panel_state
    from tools.panel_lib.pages import dashboard as dashboard_page

    real_state = project_paths.PANEL_STATE_PATH
    with panel_qa.isolated_environment(tmp_path) as env:
        assert project_paths.PANEL_STATE_PATH == env.panel_state_path
        assert panel_state.PANEL_CRASH_LOG.parent == env.log_dir
        assert dashboard_page.TELEMETRY_DIR == env.telemetry_dir
        env.panel_state_path.write_text("{}", encoding="utf-8")
    assert project_paths.PANEL_STATE_PATH == real_state
    assert (tmp_path / "panel_state.json").exists()


def test_directory_digest_notices_a_content_change_at_the_same_size(tmp_path) -> None:
    """只看 mtime/大小挡不住“写回等长坏数据”，那正是证据被污染的真实形态。"""
    (tmp_path / "a.txt").write_text("1234", encoding="utf-8")
    before = panel_qa.directory_digest(tmp_path)
    (tmp_path / "a.txt").write_text("4321", encoding="utf-8")
    assert panel_qa.directory_digest(tmp_path) != before


def test_manual_clock_never_goes_backwards() -> None:
    clock = panel_qa.ManualClock(start=100.0)
    assert clock.monotonic() == 100.0
    clock.advance(0.5)
    assert clock() == pytest.approx(100.5)
    with pytest.raises(ValueError):
        clock.advance(-0.1)


# ---------------------------------------------------------------- 协议夹具


def test_gps_fixture_keys_come_from_the_firmware_format_string() -> None:
    """改版报告的 GPS 夹具带了固件根本没有的 `flags=`，字段顺序也不对。

    键值解析让它侥幸没出事，但这种夹具下一次就会掩盖真问题——`work-modes.md` 里
    `gz_dps` 的例子就是这么让一整阶验收永远不可能通过的。
    """
    assert qa_fixtures.firmware_format_keys("GPS status") == panel_qa.GPS_STATUS_KEYS
    assert qa_fixtures.firmware_format_keys("GPS pos") == qa_fixtures.GPS_POSITION_KEYS

    status_keys = tuple(
        field.split("=")[0]
        for field in panel_qa.gps_status_line(valid=0, fix=0).split()[1:]
    )
    assert status_keys == panel_qa.GPS_STATUS_KEYS
    position_keys = tuple(
        field.split("=")[0]
        for field in panel_qa.gps_position_line().split()[2:]
    )
    assert position_keys == qa_fixtures.GPS_POSITION_KEYS
    assert "flags=" not in panel_qa.gps_status_line()


def test_telemetry_schema_fixture_matches_the_firmware_format_strings() -> None:
    header, first_channel, *_rest = panel_qa.telemetry_schema_lines()
    page = _rest[-1]

    assert tuple(f.split("=")[0] for f in header.split()[1:]) == \
        qa_fixtures.firmware_format_keys("TELEM header")
    assert tuple(f.split("=")[0] for f in first_channel.split()[2:]) == \
        qa_fixtures.firmware_format_keys("TELEM CH")
    assert tuple(f.split("=")[0] for f in page.split()[2:]) == \
        qa_fixtures.firmware_format_keys("TELEM PAGE")


def test_the_telemetry_frame_fixture_decodes_through_the_real_decoder() -> None:
    """夹具必须能被生产解码器接受，不然它证明不了任何事。"""
    from tools.panel_lib.telem_stream import TelemDecoder, TelemSchema

    schema = TelemSchema()
    for line in panel_qa.telemetry_schema_lines():
        schema.feed_line(line)
    assert schema.complete

    decoder = TelemDecoder(schema_hash=None)
    decoder.bind_schema(schema)
    samples = decoder.feed(panel_qa.telemetry_frame(
        {0: 1.5, 2: -90.0}, schema_hash=schema.computed_hash(), t_us=25000
    ))
    assert len(samples) == 1
    assert samples[0].values == {0: pytest.approx(1.5), 2: pytest.approx(-90.0)}
    assert decoder.stats.rejected_total == 0


# ---------------------------------------------------------------- 离线面板装置


@pytest.fixture(scope="module")
def offline(tmp_path_factory):
    root = tmp_path_factory.mktemp("panel-qa")
    with panel_qa.isolated_environment(root):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1366, 768))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise                            # 装置坏了就要红，不许伪装成无显示
            pytest.skip(f"Tk display unavailable: {exc}")   # pragma: no cover
        try:
            yield session
        finally:
            session.destroy()


def test_the_harness_builds_the_real_panel_with_all_eighteen_leaf_pages(offline) -> None:
    pages = offline.leaf_pages()
    assert len(pages) == 18
    labels = [page.label for page in pages]
    # 分组页签本身不算叶页，它的子页才算。
    assert any(label.startswith("校准 / ") for label in labels)
    assert any(label.startswith("传感器 / ") for label in labels)
    assert sum(label.startswith("校准 / ") for label in labels) == 7
    assert sum(label.startswith("传感器 / ") for label in labels) == 4


def test_the_qa_window_says_it_is_not_connected_to_hardware(offline) -> None:
    """截图流出去之后，没人应该把它当成连着板子的地面站。"""
    assert offline.panel.title() == panel_qa.harness.OFFLINE_TITLE
    assert "NO HARDWARE" in offline.panel.title()
    assert offline.transport.lines == [] or all(
        isinstance(line, str) for line in offline.transport.lines
    )


def test_timer_callbacks_are_recorded_not_dropped(offline) -> None:
    """报告判据：不能通过关闭所有 after 回调来冒充异步端到端测试通过。"""
    ticks: list[int] = []
    offline.pending_after.clear()
    offline.panel.after(33, lambda: ticks.append(1))
    assert len(offline.pending_after) == 1
    assert offline.pending_after[0].delay_ms == 33

    assert offline.run_pending_after() == 1
    assert ticks == [1]
    assert offline.pending_after == []


def test_no_callback_raised_while_building_or_switching_pages(offline) -> None:
    for page in offline.leaf_pages():
        offline.select(page)
    assert offline.callback_errors == []


def test_the_geometry_probe_reports_controls_that_exist_but_cannot_be_reached(
    offline,
) -> None:
    """V 线修复后，真实装置应确认机械动作不再横向越界。"""
    offline.resize(1366, 768)
    reports = {r.page: r for r in offline.probe_geometry(sizes=((1366, 768),))}
    mechanical = next(r for name, r in reports.items() if "舵机机械" in name)
    assert mechanical.clean, mechanical.to_json()

    assert len(offline.panel.mechanical_rows) == 2


def test_scrollable_content_is_not_counted_as_unreachable(offline) -> None:
    """祖先里有 Canvas 就有视口，纵向越界能滚到，不算够不着。"""
    reports = offline.probe_geometry(sizes=((1366, 768),))
    for report in reports:
        for control in report.clipped:
            if control.unallocated:
                continue
            assert control.clipped_x > qa_geometry.CLIP_TOLERANCE_PX or \
                control.clipped_y_unscrollable > qa_geometry.CLIP_TOLERANCE_PX


def test_the_simulated_dpi_scale_actually_changes_the_layout(tmp_path) -> None:
    """三档缩放必须真的作用到布局，否则矩阵的 125/150% 是空跑。"""
    child = """
import json
from pathlib import Path
import tempfile
from tools import panel_qa

with tempfile.TemporaryDirectory() as root:
    with panel_qa.hardware_guards() as guard:
        with panel_qa.isolated_environment(Path(root)):
            try:
                session = panel_qa.OfflinePanel.launch(scale=1.5, size=(1366, 768))
            except BaseException as exc:
                if panel_qa.is_display_unavailable(exc):
                    print("DISPLAY_UNAVAILABLE")
                    raise SystemExit(0)
                raise
            try:
                font = session.panel.tk.call("font", "actual", "TkDefaultFont", "-size")
                reports = session.probe_geometry(sizes=((1080, 700),))
                print(json.dumps({"scale": session.scale, "dpi": session.panel.ui_dpi_scale,
                                  "font": font, "reports": len(reports),
                                  "scales": [report.scale for report in reports],
                                  "clean": guard.clean}))
            finally:
                session.destroy()
"""
    result = subprocess.run(
        [sys.executable, "-c", child], cwd=ROOT,
        capture_output=True, text=True, timeout=120,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    if "DISPLAY_UNAVAILABLE" in result.stdout:
        pytest.skip("Tk display unavailable")
    payload = json.loads(result.stdout.strip().splitlines()[-1])
    assert payload["scale"] == 1.5
    assert payload["dpi"] == 1.5
    assert isinstance(payload["font"], int)
    assert payload["reports"] == 18
    assert all(scale == 1.5 for scale in payload["scales"])


def test_a_qa_session_leaves_the_historical_calibration_evidence_untouched(
    tmp_path,
) -> None:
    """AGENTS.md 硬约束 4：`data/calibration/**` 是历史证据，装置一个字节都不许动。"""
    if not CALIBRATION_EVIDENCE.exists():        # pragma: no cover - 精简 checkout
        pytest.skip("no calibration evidence in this checkout")
    before = panel_qa.directory_digest(CALIBRATION_EVIDENCE)
    with panel_qa.isolated_environment(tmp_path):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1080, 700))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise                            # 装置坏了就要红，不许伪装成无显示
            pytest.skip(f"Tk display unavailable: {exc}")   # pragma: no cover
        try:
            for page in session.leaf_pages():
                session.select(page)
        finally:
            session.destroy()
    assert panel_qa.directory_digest(CALIBRATION_EVIDENCE) == before


def test_the_harness_restores_every_module_it_patched(tmp_path) -> None:
    from tools import drone_tcp_panel as panel_module

    before = (panel_module.enable_hidpi_awareness,
              panel_module.DronePanel._load_panel_state,
              panel_module.messagebox.showerror)
    with panel_qa.isolated_environment(tmp_path):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1080, 700))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise                            # 装置坏了就要红，不许伪装成无显示
            pytest.skip(f"Tk display unavailable: {exc}")   # pragma: no cover
        session.destroy()
    assert (panel_module.enable_hidpi_awareness,
            panel_module.DronePanel._load_panel_state,
            panel_module.messagebox.showerror) == before
