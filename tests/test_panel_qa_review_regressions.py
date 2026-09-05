"""R-S1-3 返修：关闭 2026-09-04 软件审核的 Q1~Q6。

审核对象是 `3269a7fe`。六条的共同点是：**装置声称的保证比它实际做到的强**，而且原来
那批测试恰好都绕过了差额——`test_a_qa_session_leaves_the_historical_calibration_
evidence_untouched` 只是"构造面板后比目录摘要"，而构造面板本来就不写校准文件，所以
它对 Q1 一句话都说不出来。

  Q1  只重指了四个模块里的六个常量。机械、光流、V0 的校准目录和 RC 向导日志仍指向
      真实工作树——手工名单必然漏，因为 `from ..project_paths import X` 在每个消费
      模块里各绑一份。
  Q2  只换了 serial 和当前 transport；tcp/udp 仍是真实适配器，选到 TCP 再点连接会
      走到真实 `_start()`。
  Q3  `blocked_popen` 只看 argv[0]，`Popen(..., executable='openocd.exe')` 直接穿过；
      而且 subprocess 护栏只在局部 `hardware_guards()` 里生效，默认 pytest 全程只有
      串口那一条。
  Q4  启动定时器先被一把取消再换 `after`，记录里只剩两个 resize 回调；`after_cancel`
      是空函数，取消不掉任何东西。
  Q5  `exclusive_path()` 只是挑名字不创建，两个调用者会拿到同一个路径。
  Q6  `tests/golden/*.bin` 被 `.gitignore` 的 `*.bin` 吞掉，干净检出跑不了黄金向量。
"""

from __future__ import annotations

import subprocess
import threading
import tkinter as tk
from pathlib import Path

import pytest
import serial

from tools import panel_qa
from tools.panel_qa import isolation as qa_isolation


ROOT = Path(__file__).resolve().parents[1]


# ---------------------------------------------------------------- Q1 隔离完整性


def test_every_data_path_constant_is_redirected(tmp_path) -> None:
    """审核 Q1：判据不是"构造面板后目录摘要没变"，而是**每一个写入目标**都在 QA 根里。"""
    import tools.project_paths as project_paths

    real_data_root = project_paths.DATA_ROOT
    with panel_qa.isolated_environment(tmp_path) as env:
        leaked: list[str] = []
        for module in qa_isolation._module_candidates():
            for attr, value in list(vars(module).items()):
                if not attr.isupper() or not isinstance(value, Path):
                    continue
                try:
                    value.relative_to(real_data_root)
                except ValueError:
                    continue                     # 本来就不在 data/ 下
                leaked.append(f"{module.__name__}.{attr} -> {value}")
        assert leaked == [], "这些路径常量还指着真实工作树：\n" + "\n".join(leaked)
        assert env.contains(project_paths.PANEL_STATE_PATH)
    assert project_paths.DATA_ROOT == real_data_root, "退出后必须原样还原"


@pytest.mark.parametrize("attribute", [
    "SERVO_MECHANICAL_CALIBRATION_DIR",
    "FLOW_RANGE_CALIBRATION_DIR",
    "FLIGHT_ACCEPTANCE_CALIBRATION_DIR",
    "IMU_METROLOGY_CALIBRATION_DIR",
    "TELEMETRY_DIR",
    "LOG_DIR",
])
def test_the_output_directories_the_review_named_are_isolated(tmp_path, attribute) -> None:
    """审核 Q1 逐个点名过：机械、光流、V0 校准目录。"""
    import tools.project_paths as project_paths

    with panel_qa.isolated_environment(tmp_path) as env:
        assert env.contains(getattr(project_paths, attribute)), attribute


def test_the_rc_wizard_log_follows_the_isolated_log_dir(tmp_path) -> None:
    """RC 向导自己 import 了一份 `RC_WIZARD_TRACE_LOG`，改 LOG_DIR 改不到它。"""
    from tools.panel_lib import state as panel_state

    with panel_qa.isolated_environment(tmp_path) as env:
        assert env.contains(panel_state.RC_WIZARD_TRACE_LOG)
        assert env.contains(panel_state.PANEL_CRASH_LOG)


# ---------------------------------------------------------------- Q2 全部 transport


@pytest.fixture(scope="module")
def offline(tmp_path_factory):
    root = tmp_path_factory.mktemp("qa-review")
    with panel_qa.isolated_environment(root):
        try:
            session = panel_qa.OfflinePanel.launch(scale=1.0, size=(1366, 768))
        except Exception as exc:
            if not panel_qa.is_display_unavailable(exc):
                raise
            pytest.skip(f"Tk display unavailable: {exc}")     # pragma: no cover
        try:
            yield session
        finally:
            session.destroy()


def test_no_real_transport_adapter_survives_in_the_harness(offline) -> None:
    """审核 Q2：只换 serial 和当前 transport 不够，另外两条仍是真实适配器。"""
    panel = offline.panel
    for name in ("serial_transport", "tcp_transport", "udp_transport"):
        adapter = getattr(panel, name)
        assert isinstance(adapter, panel_qa.MemoryTransport), (
            f"{name} 仍是 {type(adapter).__name__}"
        )


def test_every_transport_mode_resolves_to_a_fake(offline) -> None:
    """遍历连接方式，不能只测当前那一条。"""
    panel = offline.panel
    original = panel.transport_var.get()
    try:
        for mode in ("serial", "tcp", "udp"):
            panel.transport_var.set(mode)
            assert isinstance(panel._current_transport(), panel_qa.MemoryTransport), mode
    finally:
        panel.transport_var.set(original)


def test_the_fake_transport_reports_connection_state_and_generation() -> None:
    """替身的 start/stop 必须如实翻状态并进代次，否则重连契约在装置里演不出来。"""
    transport = panel_qa.MemoryTransport(connected=False)
    before = transport.connection_generation
    transport.start()
    assert transport.is_connected
    assert transport.connection_generation == before + 1
    transport.stop()
    assert not transport.is_connected
    assert transport.binary_sink is None
    assert transport.connection_generation == before + 2


# ---------------------------------------------------------------- Q3 烧录护栏


def test_the_guard_catches_the_executable_argument() -> None:
    """审核 Q3：`Popen(['safe-name'], executable='openocd.exe')` 曾经直接穿过去。"""
    assert panel_qa.is_flash_tool_command(["safe-name"], "C:/tools/openocd.exe") == "openocd"
    with panel_qa.hardware_guards() as log:
        with pytest.raises(AssertionError, match="flashing/reset tool"):
            subprocess.Popen(["safe-name"], executable="C:/tools/openocd.exe")
    assert len(log.flash_invocations) == 1


def test_the_guard_is_installed_for_the_whole_default_pytest_run(pytestconfig) -> None:
    """审核 Q3 后半：局部上下文管理器不等于"整个默认 pytest 都有双向保护"。"""
    log = getattr(pytestconfig, "_hardware_guard_log", None)
    assert log is not None, "conftest 必须在收集之前装上全局护栏"
    # 护栏真的在位：不进任何局部上下文，直接试就该被拦。
    with pytest.raises(AssertionError, match="physical serial port"):
        port = serial.Serial()
        port.port = "COM99"
        port.open()
    with pytest.raises(AssertionError, match="flashing/reset tool"):
        subprocess.Popen(["openocd", "-f", "board.cfg"])
    # 本用例是**刻意**触发，从全局计数里扣掉，别让它把整个 session 判失败。
    log.attempts[:] = [
        entry for entry in log.attempts
        if entry.detail not in ("COM99",) and "board.cfg" not in entry.detail
    ]


# ---------------------------------------------------------------- Q4 定时器替身


def test_the_startup_timer_chain_is_preserved(offline) -> None:
    """审核 Q4：第一版记录里只有两个 resize 回调，接收/链路/录制的循环全没了。"""
    names = set(offline.scheduled_callbacks())
    for expected in ("_drain_rx", "_check_link_health", "_dashboard_render_tick"):
        assert expected in names, f"启动链里少了 {expected}；当前只有 {sorted(names)}"


def test_after_cancel_actually_cancels(offline) -> None:
    """审核 Q4：`after_cancel` 是空函数，取消过的回调照样会被执行。"""
    ran: list[str] = []
    ident = offline.panel.after(9999, lambda: ran.append("nope"))
    assert isinstance(ident, str) and ident
    offline.panel.after_cancel(ident)
    offline.run_pending_after()
    assert ran == [], "取消过的回调仍然被执行了"


def test_callbacks_run_in_due_order_when_the_virtual_clock_advances(offline) -> None:
    order: list[str] = []
    offline.pending_after.clear()
    offline.panel.after(200, lambda: order.append("late"))
    offline.panel.after(50, lambda: order.append("early"))

    assert offline.advance_ms(100) == 1
    assert order == ["early"], "还没到期的回调不该被执行"
    assert offline.advance_ms(200) == 1
    assert order == ["early", "late"]


# ---------------------------------------------------------------- Q5 排他申领


def test_claiming_an_output_path_reserves_the_file(tmp_path) -> None:
    """审核 Q5：只挑名字不创建，两个调用者会拿到同一个路径再互相覆盖。"""
    first_handle, first = panel_qa.claim_output_path(tmp_path, "observations", ".json")
    with first_handle:
        first_handle.write("first")
    second_handle, second = panel_qa.claim_output_path(tmp_path, "observations", ".json")
    with second_handle:
        second_handle.write("second")

    assert first != second, "两次申领拿到了同一个路径"
    assert first.read_text(encoding="utf-8") == "first"
    assert second.read_text(encoding="utf-8") == "second"


def test_concurrent_claims_never_collide(tmp_path) -> None:
    claimed: list[Path] = []
    lock = threading.Lock()

    def claim() -> None:
        handle, path = panel_qa.claim_output_path(tmp_path, "observations", ".json")
        with handle:
            handle.write(path.name)
        with lock:
            claimed.append(path)

    threads = [threading.Thread(target=claim) for _ in range(12)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert len(set(claimed)) == 12
    assert all(path.read_text(encoding="utf-8") == path.name for path in claimed)


# ---------------------------------------------------------------- Q6 干净检出


def test_the_golden_fixture_is_tracked_by_git() -> None:
    """审核 Q6：`.gitignore` 的 `*.bin` 把它吞了，独立干净检出跑不了黄金向量。"""
    golden = ROOT / "tests" / "golden" / "telem_frames_v1.bin"
    assert golden.exists(), "黄金向量夹具不在工作树里"
    listed = subprocess.run(
        ["git", "ls-files", "--error-unmatch", str(golden.relative_to(ROOT).as_posix())],
        cwd=ROOT, capture_output=True, text=True,
    )
    assert listed.returncode == 0, (
        "黄金向量没有纳入 Git；干净检出会因 FileNotFoundError 失败三项测试\n"
        f"{listed.stderr}"
    )


def test_the_ignore_exception_is_narrow() -> None:
    """例外必须精确到这个目录，不能顺手把整棵树的 .bin 都放进版本库。"""
    ignore = (ROOT / ".gitignore").read_text(encoding="utf-8")
    assert "!tests/golden/*.bin" in ignore
    assert "!*.bin" not in ignore
