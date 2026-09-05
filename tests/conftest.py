"""离线测试套件的全局硬件护栏与状态隔离。

两条路都要堵，而且必须在**收集之前**就装上：

  * 物理串口 —— `serial.Serial.open`
  * 烧录 / 调试探针 —— `subprocess.Popen`（openocd / STM32_Programmer / JLink / pyocd …）

原来这里只有串口那条。`tools/panel_qa` 里的 subprocess 护栏只在主动进入
`hardware_guards()` 的局部作用域生效，于是"整个默认 pytest 具备双向保护"这个说法
并不成立（2026-09-04 软件审核 Q3）。AGENTS.md 第 5 条说的是未经 REQ 明文授权不得
烧录或复位，而 `subprocess` 完全绕开串口层。

刻意验证护栏的测试自己开一层 `hardware_guards()`，命中记在那一层的 log 里，不会
进全局计数——所以下面的"尝试次数"为非零时，一定是意外触碰，判整个 session 失败。
"""

import pytest


def pytest_configure(config):
    from tools.panel_qa.guards import install_hardware_guards

    log, uninstall = install_hardware_guards()
    config._hardware_guard_log = log
    config._hardware_guard_uninstall = uninstall


@pytest.fixture(scope="session", autouse=True)
def isolate_panel_defaults(tmp_path_factory):
    """把面板**自身**的状态与日志挪到临时目录，并掐掉启动副作用。

    这里刻意只动 panel_state / 日志这几项，不整棵重指 `DATA_ROOT`：
    `tests/test_flight_log_paths.py` 断言的正是 `data/` 树的规范定义，全局重指会把
    那条正当契约打掉。需要完整写入面隔离的用例（QA 装置、页面保存/导出）自己进
    `panel_qa.isolated_environment()`——审核 Q1 修的是那个入口。
    """
    from tools import drone_tcp_panel as panel
    from tools import project_paths
    from tools.panel_lib import state

    patch = pytest.MonkeyPatch()
    root = tmp_path_factory.mktemp("panel-defaults")
    for name, value in {"PANEL_STATE_PATH": root / "panel_state.json", "LOG_DIR": root,
                        "PANEL_CRASH_LOG": root / "panel_crash.log",
                        "RC_WIZARD_TRACE_LOG": root / "rc_wizard.log"}.items():
        for module in (panel, state, project_paths):
            if hasattr(module, name):
                patch.setattr(module, name, value)
    original_init = panel.DronePanel.__init__

    def isolated_init(self, *args, **kwargs):
        # Only suppress startup side effects; explicit test calls retain the API.
        with pytest.MonkeyPatch.context() as startup:
            for method in ("_restore_last_connection", "_save_panel_state",
                           "_validation_load_latest_artifact", "_v1_load_latest_session"):
                startup.setattr(panel.DronePanel, method, lambda self: None)
            startup.setattr(panel.DronePanel, "_load_panel_state", lambda self: {})
            original_init(self, *args, **kwargs)

    patch.setattr(panel.DronePanel, "__init__", isolated_init)
    yield
    patch.undo()


def pytest_sessionfinish(session, exitstatus):
    log = getattr(session.config, "_hardware_guard_log", None)
    if log is not None and not log.clean:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter, config):
    log = getattr(config, "_hardware_guard_log", None)
    if log is None:                              # pragma: no cover - configure 失败
        return
    terminalreporter.write_line(
        f"Physical serial open attempts: {len(log.serial_opens)}"
    )
    terminalreporter.write_line(
        f"Flashing/probe tool invocation attempts: {len(log.flash_invocations)}"
    )
    for detail in log.attempts:
        terminalreporter.write_line(f"  blocked {detail.kind}: {detail.detail}")


def pytest_unconfigure(config):
    uninstall = getattr(config, "_hardware_guard_uninstall", None)
    if uninstall is not None:
        uninstall()
