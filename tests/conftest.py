"""Offline suite: block physical serial opens before collection and Tk callbacks."""

import pytest


def pytest_configure(config):
    import serial
    config._physical_serial_opens = []
    config._serial_open_original = serial.Serial.open

    def forbidden_open(port, *args, **kwargs):
        config._physical_serial_opens.append(str(getattr(port, "port", None)))
        raise AssertionError("Tests must use a fake transport; physical serial open blocked")

    serial.Serial.open = forbidden_open


@pytest.fixture(scope="session", autouse=True)
def isolate_panel_defaults(tmp_path_factory):
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
    if session.config._physical_serial_opens:
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter, config):
    terminalreporter.write_line(
        f"Physical serial open attempts: {len(config._physical_serial_opens)}"
    )


def pytest_unconfigure(config):
    import serial
    serial.Serial.open = config._serial_open_original
