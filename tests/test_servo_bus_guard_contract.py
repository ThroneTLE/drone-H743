"""R-S7-3 contract: bus-only servo paths are inert in PWM mode."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError(f"unterminated function: {signature}")


def test_guard_owns_pwm_selection_and_all_bus_only_commands() -> None:
    header = read("App/Inc/app_servo_bus_guard.h")
    source = read("App/Src/app_servo_bus_guard.c")
    cmake = read("CMakeLists.txt")

    assert "APP_ServoBusGuard_IsPwmMode" in header
    assert "APP_ServoBusGuard_IsBusOnlyCommand" in header
    assert "APP_ServoType_GetActive()" in source
    assert "App/Src/app_servo_bus_guard.c" in cmake
    for command in (
        "MOVE",
        "MOVEALL",
        "ID",
        "SETID",
        "MODE",
        "ENABLE",
        "CMD",
        "RAW",
        "BAUDRATE",
        "FB",
    ):
        assert f'"{command}"' in source


def test_control_has_one_minimal_pwm_rejection_before_bus_dispatch() -> None:
    control = read("App/Src/app_control.c")
    handler = _function_body(
        control, "static void app_control_handle_servo(char **tokens, uint32_t count)"
    )
    guard = handler.index("APP_ServoBusGuard_IsPwmMode")
    jog = handler.index('strcmp(tokens[1], "JOG")')

    assert guard < jog
    assert "APP_ServoBusGuard_IsBusOnlyCommand(tokens[1])" in handler
    assert "unsupported mode=pwm" in handler
    # BUS-mode dispatch remains the existing implementation below the guard.
    for call in (
        "BSP_BusServo_Move(",
        "BSP_BusServo_SetId(",
        "BSP_BusServo_SetMode(",
        "BSP_BusServo_SendRaw(",
        "BSP_BusServo_SetBaudRate(",
    ):
        assert call in handler


def test_legacy_vofa_servo_command_is_rejected_only_in_pwm_mode() -> None:
    control = read("App/Src/app_control.c")
    pwm_branch = control.index(
        'strncmp(tokens[0], "Servor", 6) == 0) &&\n'
        '               (APP_ServoBusGuard_IsPwmMode() != 0U)'
    )
    legacy_branch = control.index(
        '} else if (strncmp(tokens[0], "Servor", 6) == 0) {', pwm_branch
    )
    legacy_body = _function_body(
        control,
        '} else if (strncmp(tokens[0], "Servor", 6) == 0) {',
    )

    assert pwm_branch < legacy_branch
    assert 'ERR servo legacy vofa unsupported mode=pwm\\r\\n' in control
    # The original BUS-mode legacy branch still contains its direct move path.
    assert "BSP_BusServo_Move(" in legacy_body


def test_servo_cal_pwm_gestures_only_clear_state_and_post_async_rejection() -> None:
    source = read("App/Src/app_servo_cal.c")
    step = _function_body(
        source,
        "APP_ServoCalResult APP_ServoCal_Step(const uint16_t ch[16],",
    )
    pwm_block = step[: step.index("if (servo_cal_state == APP_SERVO_CAL_STATE_SAVE_LOCK_ACK)")]

    assert "APP_ServoBusGuard_IsPwmMode()" in pwm_block
    assert "servo_cal_release_hold_start_ms = 0U" in pwm_block
    assert "servo_cal_save_hold_start_ms = 0U" in pwm_block
    assert 'ERR servo_cal unsupported mode=pwm\\r\\n' in pwm_block
    assert "servo_cal_pwm_notice_sent" in pwm_block
    assert "BSP_BusServo_" not in pwm_block


def test_servotype_transaction_blocks_type_switch_while_servo_gesture_is_active() -> None:
    source = read("App/Src/app_cmd_servotype.c")
    transaction = _function_body(
        source, "static uint8_t app_servotype_transaction_available(void)"
    )

    assert '#include "app_servo_cal.h"' in source
    assert "APP_ServoCal_IsActive()" in transaction
    # The guard remains part of the existing transaction rejection set; no
    # separate PWM-only path can bypass it.
    assert "app_cmd_servocal_is_busy()" in transaction


def test_feedback_bench_rejects_pwm_before_every_bus_capable_entry() -> None:
    source = read("App/Src/app_servo_feedback_bench.c")
    for signature in (
        "uint8_t APP_ServoFeedbackBench_Start(",
        "uint8_t APP_ServoFeedbackBench_StartSweep(",
        "uint8_t APP_ServoFeedbackBench_StartStep(",
        "void APP_ServoFeedbackBench_Stop(",
        "void APP_ServoFeedbackBench_ReportStatus(",
        "void APP_ServoFeedbackBench_ApplyTargets(",
        "void APP_ServoFeedbackBench_Step(",
        "void APP_ServoFeedbackBench_RecordMoveResult(",
        "uint8_t APP_ServoFeedbackBench_IsActive(",
        "uint8_t APP_ServoFeedbackBench_MoveRefreshDue(",
    ):
        assert "servo_fb_pwm_blocked()" in _function_body(source, signature)
    assert 'ERR servo fb unsupported mode=pwm\\r\\n' in source


def test_guard_runtime_selection_and_command_table_with_host_c(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        return

    harness = tmp_path / "servo_bus_guard.c"
    executable = tmp_path / "servo_bus_guard.exe"
    harness.write_text(
        r'''
#include "app_servo_bus_guard.h"

#include <assert.h>

int main(void) {
    APP_ServoType_ResetActive();
    assert(APP_ServoBusGuard_IsPwmMode() == 0U);
    assert(APP_ServoBusGuard_IsBusOnlyCommand("MOVE") == 1U);
    assert(APP_ServoBusGuard_IsBusOnlyCommand("MOVEALL") == 1U);
    assert(APP_ServoBusGuard_IsBusOnlyCommand("FB") == 1U);
    assert(APP_ServoBusGuard_IsBusOnlyCommand("JOG") == 0U);
    assert(APP_ServoBusGuard_IsBusOnlyCommand(NULL) == 0U);

    assert(APP_ServoType_PublishActive(APP_SERVO_TYPE_PWM) == 1U);
    assert(APP_ServoBusGuard_IsPwmMode() == 1U);
    assert(APP_ServoBusGuard_IsBusOnlyCommand("RAW") == 1U);
    return 0;
}
''',
        encoding="ascii",
    )
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'App' / 'Inc'}",
            str(ROOT / "App/Src/app_servo_type.c"),
            str(ROOT / "App/Src/app_servo_bus_guard.c"),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True)
