"""S6."""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONTROL_SRC = ROOT / "App" / "Src" / "app_control.c"
SERVOCAL_SRC = ROOT / "App" / "Src" / "app_cmd_servocal.c"
CMAKELISTS = ROOT / "CMakeLists.txt"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for offset in range(brace, len(source)):
        if source[offset] == "{":
            depth += 1
        elif source[offset] == "}":
            depth -= 1
            if depth == 0:
                return source[start:offset + 1]
    raise AssertionError(f"unbalanced braces after {signature!r}")


def test_split() -> None:
    servocal_src = _read(SERVOCAL_SRC)
    control_src = _read(CONTROL_SRC)

    # 1. Functions exist in app_cmd_servocal.c and zero private definitions in app_control.c
    for sig in (
        "static void app_control_servocal_clear_preview(void)",
        "static void app_control_servocal_set_event(const char *event,",
        "static void app_control_report_servocal_record(",
        "static void app_control_report_servocal(void)",
        "static uint8_t app_control_parse_servocal(",
        "void app_control_handle_servocal(char **tokens, uint32_t count)",
        "void app_control_service_servocal(void)",
        "void app_cmd_servocal_init(void)",
        "void app_cmd_servocal_notify_persisted(",
        "uint8_t app_cmd_servocal_is_busy(void)",
    ):
        assert sig in servocal_src, f"Missing {sig} in app_cmd_servocal.c"

    for func_def in (
        "app_control_servocal_clear_preview",
        "app_control_servocal_set_event",
        "app_control_report_servocal_record",
        "app_control_report_servocal(",
        "app_control_parse_servocal",
    ):
        assert func_def not in control_src, f"Residue of {func_def} in app_control.c"

    # 2. Static variables isolated in app_cmd_servocal.c
    for var in (
        "control_servocal_preview;",
        "control_servocal_pending_record;",
        "control_servocal_preview_generation;",
        "control_servocal_last_request;",
        "control_servocal_applied;",
        "control_servocal_commit_pending;",
        "control_servocal_last_event;",
        "control_servocal_last_reason;",
    ):
        assert var in servocal_src, f"Missing {var} in app_cmd_servocal.c"
        assert var not in control_src, f"Residue of {var} in app_control.c"

    # 3. Wiring in app_control.c
    assert '#include "app_cmd_servocal.h"' in control_src
    dispatch_body = _function_body(
        control_src,
        "static void app_control_dispatch_tokens(char **tokens, uint32_t count, uint8_t emit_ack)\n{",
    )
    assert 'strcmp(tokens[0], "SERVOCAL?") == 0' in dispatch_body
    assert 'strcmp(tokens[0], "SERVOCAL") == 0' in dispatch_body
    assert "app_control_handle_servocal(tokens, count);" in dispatch_body

    tick_body = _function_body(
        control_src,
        "static void app_control_tick_common(uint8_t emit_heartbeat)\n{",
    )
    assert "app_control_service_servocal();" in tick_body

    init_body = _function_body(control_src, "void APP_Control_Init(void)\n{")
    assert "app_cmd_servocal_init();" in init_body

    assert "app_cmd_servocal_notify_persisted(&calibration);" in control_src
    assert "app_cmd_servocal_is_busy()" in control_src

    # 4. File size and build registration
    servocal_lines = len(SERVOCAL_SRC.read_text(encoding="utf-8").splitlines())
    assert servocal_lines <= 800, f"app_cmd_servocal.c has {servocal_lines} lines (> 800)"
    assert "App/Src/app_cmd_servocal.c" in _read(CMAKELISTS)

