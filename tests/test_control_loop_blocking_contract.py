"""控制环禁止同步阻塞 I/O 的契约。

APP_Control_QueueText 在入队 UART 前会同步阻塞等 USB CDC（最坏 3x
APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS）。app_servo_cal.c 的手势状态机整体
跑在 500Hz 控制环（stabilizer_control_prepare -> APP_ServoCal_Step），
曾经四处直接调用 QueueText，把最坏 ~30ms 的阻塞埋进姿态环。

修复后的结构：控制环内只 vsnprintf 进单条通告缓冲并置标志；通信任务的
app_control_tick_common 经 APP_ServoCal_TakeNotice() 取走后再排队发送。
本文件防止任何一半被回退。
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
SERVO_CAL = ROOT / "App" / "Src" / "app_servo_cal.c"
SERVO_CAL_HEADER = ROOT / "App" / "Inc" / "app_servo_cal.h"
CONTROL = ROOT / "App" / "Src" / "app_control.c"
CONTROL_CORE = ROOT / "App" / "Src" / "app_control_core.c"
SERVOCAL_CMD = ROOT / "App" / "Src" / "app_cmd_servocal.c"
CONTROL_INTERNAL = ROOT / "App" / "Inc" / "app_control_internal.h"
CMAKE = ROOT / "CMakeLists.txt"

STEP_A_BODY_SHA256 = {
    "app_control_queue_proto_text": "852630f596f0c6851e32a04b97106655e8f29080254b8b5ccc0b2de3493ad5f8",
    "app_control_tokenize": "5186a0c9ef1a832f2619926b25b3e14cf2c4fe859baa2075cc28e0bff9f12923",
    "app_control_parse_u32": "9b35592f3df9183e87294ba31661553305887e7be944392ff724be2a570820d7",
    "app_control_parse_i32": "6c8d3ff6ef993cc6bd8726e99676aa1cbea12a2e4267c74a0b205fb1d98f1923",
    "app_control_crc32_update": "fca3b9139ae7aab310b73813e60e704bb163d463413a953cd4cb83f9c4ecf6c3",
    "app_control_crc32": "3169f7344b5ead9306fc097785bd32626479e1b7c01040d8b8d6c44b12d2998a",
    "app_control_token_value": "9305c3293c9348edbcd2f09908a04a8d54270fc4441eee05a146d43167ff968e",
}
STEP_B_LEGACY_BODY_SHA256 = {
    # C extracts only the contiguous SERVOCAL persisted fragment; the rest of
    # sync is pinned by this direct-parent hash and the fragment below.
    "app_control_imuframe_sync_param": "1d5b5697c1d6b853bb76d6e38dc51268f4960410733dd87a8e1b9cd7ceb35db6",
    "app_control_imucal_safety": "87327cf50ef57f4566ad276d868912f8bbac6f3e99524e80b20b2b0b544d4c78",
}
STEP_B_ACCESSOR_BODIES = {
    "const void *app_control_internal_imucal_confirmed_record(void)": (
        "{\n    return &control_imucal_confirmed;\n}"
    ),
    "uint32_t app_control_internal_imucal_confirmed_generation(void)": (
        "{\n    return control_imucal_confirmed_generation;\n}"
    ),
    "uint8_t app_control_internal_imucal_confirmed_valid(void)": (
        "{\n    return control_imucal_confirmed_valid;\n}"
    ),
    "void app_control_internal_imuframe_sync_param(void)": (
        "{\n    app_control_imuframe_sync_param();\n}"
    ),
    "const char *app_control_internal_imucal_safety(": (
        "{\n"
        "    return app_control_imucal_safety(\n"
        "        (StabilizerValidationImuSnapshot *)snapshot,\n"
        "        require_sequence_progress);\n"
        "}"
    ),
    "uint8_t app_control_internal_imucal_upload_state(void)": (
        "{\n    return (uint8_t)control_imucal_upload.state;\n}"
    ),
    "uint8_t app_control_internal_imucal_applied(void)": (
        "{\n    return control_imucal_applied;\n}"
    ),
    "uint8_t app_control_internal_imucal_commit_pending(void)": (
        "{\n    return control_imucal_commit_pending;\n}"
    ),
    "uint8_t app_control_internal_imuframe_confirmed_code(void)": (
        "{\n    return control_imuframe_confirmed_code;\n}"
    ),
}
STEP_C_BODY_SHA256 = {
    "app_control_servocal_clear_preview": "b1854d00dff1cdcfcf730b1c1e10b03aa2ef2d85f6bd37a55f3cde2a8505f84e",
    "app_control_servocal_set_event": "a4aae2e90420e2db7233774aaa71d786c3040f2360e1a22bfb9c2e65128b9dc7",
    "app_control_report_servocal_record": "41de7a2923ed57076d428b92cfa7dc33dbfcf0f1e42329a8262188aba60e1283",
    "app_control_report_servocal": "47dff691d685afff45143df773e0eb692c1ae47b5c90af6743965299cf2b15d4",
    "app_control_parse_servocal": "8ac31c2ea6d80c4f2c0c6c0ac730b903d738ae22ff746d2fa500dd226e15cb5b",
    "app_control_handle_servocal": "75f9f7389664a52523366bd3d3b6390d5b5905d90e38282214d752ccc9aab07d",
    "app_control_service_servocal": "ec23df6b66699a1aa7aeec88247c39007acbe3d5dbcb46d81c1a994b4f76d135",
}
STEP_C_STATE_NAMES = {
    "control_servocal_preview",
    "control_servocal_pending_record",
    "control_servocal_preview_generation",
    "control_servocal_last_request",
    "control_servocal_applied",
    "control_servocal_commit_pending",
    "control_servocal_last_event",
    "control_servocal_last_reason",
}
STEP_C_PERSISTED_FRAGMENT = """    if (control_servocal_commit_pending != 0U) {
        if (memcmp(&calibration,
                   &control_servocal_pending_record,
                   sizeof(calibration)) == 0) {
            app_control_servocal_set_event("committed", "none");
        } else {
            app_control_servocal_set_event("commit_failed", "record_mismatch");
        }
        app_control_servocal_clear_preview();
    } else if (control_servocal_applied != 0U) {
        app_control_servocal_set_event("reverted", "persisted_changed");
        app_control_servocal_clear_preview();
    }"""
STEP_C_INIT_FRAGMENT = """    memset(&control_servocal_preview, 0,
           sizeof(control_servocal_preview));
    memset(&control_servocal_pending_record, 0,
           sizeof(control_servocal_pending_record));
    control_servocal_preview_generation = 0U;
    control_servocal_last_request = 0U;
    control_servocal_applied = 0U;
    control_servocal_commit_pending = 0U;
    app_control_servocal_set_event("init", "none");
    APP_Stabilizer_SetServoCalibrationCandidateArmLock(0U);"""

CORE_HARNESS = r"""
#include "app_control_internal.h"
#include "app_messages.h"
#include "app_tasks.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

osMessageQueueId_t uartTxQueueHandle = (osMessageQueueId_t)1;
static APP_UART_TxMessage captured;
static uint32_t put_count;
static uint32_t usb_timeout_ms;
static uint32_t notify_count;
static uint32_t maint_count;
static uint8_t maint_active;

osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *message,
                             uint8_t priority, uint32_t timeout)
{
    (void)queue; (void)priority; (void)timeout;
    captured = *(const APP_UART_TxMessage *)message;
    put_count++;
    return osOK;
}

osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void *message,
                             uint8_t *priority, uint32_t timeout)
{
    (void)queue; (void)message; (void)priority; (void)timeout;
    return osOK;
}

uint8_t APP_IMU_Capture_IsExportActive(void) { return 0U; }
uint8_t APP_USB_CDC_Write(const uint8_t *data, uint16_t length,
                          uint32_t timeout_ms)
{
    (void)data; (void)length;
    usb_timeout_ms = timeout_ms;
    return 1U;
}
void APP_UART_NotifyTxPending(void) { notify_count++; }
void APP_MaintUART_Write(const char *text, uint16_t length)
{
    (void)text; (void)length;
    maint_count++;
}
uint8_t app_control_internal_maint_output_active(void) { return maint_active; }

int main(void)
{
    char line[] = "CMD value=42 signed=-17";
    char *tokens[4] = {0};
    uint32_t u32 = 0U;
    int32_t i32 = 0;
    const uint8_t crc_input[] = "123456789";

    CHECK(app_control_tokenize(line, tokens, 4U) == 3U, 1);
    CHECK(strcmp(tokens[0], "CMD") == 0, 2);
    CHECK(strcmp(app_control_token_value(tokens, 3U, "value"), "42") == 0, 3);
    CHECK(app_control_token_value(tokens, 3U, "missing") == NULL, 4);
    CHECK(app_control_parse_u32("429", &u32) == 1U && u32 == 429U, 5);
    CHECK(app_control_parse_u32("42x", &u32) == 0U, 6);
    CHECK(app_control_parse_i32("-17", &i32) == 1U && i32 == -17, 7);
    CHECK(app_control_parse_i32("", &i32) == 0U, 8);
    CHECK(app_control_crc32(crc_input, 9U) == 0xCBF43926UL, 9);

    app_control_queue_proto_text(0x2222U, "VALUE %lu\r\n", (unsigned long)u32);
    CHECK(captured.function == 0x2222U, 10);
    CHECK(strcmp(captured.text, "VALUE 429\r\n") == 0, 11);
    CHECK(usb_timeout_ms == 10U, 12);
    CHECK(put_count == 1U && notify_count == 1U && maint_count == 0U, 13);

    maint_active = 1U;
    app_control_queue_proto_text(0x2223U, "MAINT\r\n");
    CHECK(put_count == 1U && maint_count == 1U, 14);
    return 0;
}
"""


def _c_function_body(source: str, name: str) -> str:
    pattern = re.compile(
        rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
        rf"{re.escape(name)}\s*\(",
        re.MULTILINE,
    )
    matches = list(pattern.finditer(source))
    assert matches, name
    match = matches[-1]
    brace = source.index("{", match.start())
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"unterminated C function: {name}")


def _write_core_stubs(stub_dir: Path) -> None:
    stub_dir.mkdir()
    (stub_dir / "app_imu_capture.h").write_text(
        "#include <stdint.h>\nuint8_t APP_IMU_Capture_IsExportActive(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_maint_uart.h").write_text(
        "#include <stdint.h>\nvoid APP_MaintUART_Write(const char *, uint16_t);\n",
        encoding="ascii",
    )
    (stub_dir / "app_messages.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint16_t function; uint16_t length; char text[256]; } "
        "APP_UART_TxMessage;\n",
        encoding="ascii",
    )
    (stub_dir / "app_tasks.h").write_text(
        "#include <stdint.h>\n"
        "typedef void *osMessageQueueId_t; typedef int32_t osStatus_t;\n"
        "#define osOK 0\n"
        "extern osMessageQueueId_t uartTxQueueHandle;\n"
        "osStatus_t osMessageQueuePut(osMessageQueueId_t,const void*,uint8_t,uint32_t);\n"
        "osStatus_t osMessageQueueGet(osMessageQueueId_t,void*,uint8_t*,uint32_t);\n",
        encoding="ascii",
    )
    (stub_dir / "app_uart.h").write_text(
        "void APP_UART_NotifyTxPending(void);\n", encoding="ascii"
    )
    (stub_dir / "app_usb_cdc.h").write_text(
        "#include <stdint.h>\n"
        "uint8_t APP_USB_CDC_Write(const uint8_t*,uint16_t,uint32_t);\n",
        encoding="ascii",
    )


def _body_after_signature(source: str, signature: str) -> str:
    start = source.rindex(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(f"unterminated C function after {signature}")


def _check_app_control_step_b() -> None:
    control = CONTROL.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    for name, expected_hash in STEP_B_LEGACY_BODY_SHA256.items():
        body = _c_function_body(control, name)
        assert hashlib.sha256(body.encode()).hexdigest() == expected_hash
    for signature, expected_body in STEP_B_ACCESSOR_BODIES.items():
        assert _body_after_signature(control, signature) == expected_body
        assert signature in internal
    assert not re.search(r"\bextern\b.*\bcontrol_(?:imucal|imuframe)", internal)


def _write_servocal_stubs(stub_dir: Path) -> None:
    (stub_dir / "app_control.h").write_text(
        "void APP_Control_QueueText(const char *, ...);\n", encoding="ascii"
    )
    (stub_dir / "app_acceptance.h").write_text(
        "#include <stdint.h>\nuint8_t APP_Acceptance_IsActive(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_proto.h").write_text(
        "#define APP_PROTO_MSG_SERVO_CAL 0x2225U\n", encoding="ascii"
    )
    (stub_dir / "app_sensor.h").write_text(
        "#include <stdint.h>\nuint8_t APP_Sensor_GetFluOrientation(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_stabilizer.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t value[16]; } StabilizerValidationImuSnapshot;\n"
        "uint8_t APP_Stabilizer_IsServoCalibrationCandidateArmLocked(void);\n"
        "void APP_Stabilizer_SetServoCalibrationCandidateArmLock(uint8_t);\n",
        encoding="ascii",
    )
    (stub_dir / "drv_coax_ctrl.h").write_text(
        "#include <stdint.h>\n"
        "#define DRV_COAX_CTRL_SERVO_COUNT 2U\n"
        "#define DRV_COAX_CTRL_SERVO_ALPHA_INDEX 0U\n"
        "#define DRV_COAX_CTRL_SERVO_BETA_INDEX 1U\n"
        "typedef struct { uint16_t center_us[2]; uint16_t min_us[2]; "
        "uint16_t max_us[2]; int8_t pulse_sign[2]; } DRV_COAX_CTRL_ServoCalibration;\n"
        "void DRV_COAX_CTRL_GetDefaultServoCalibration(DRV_COAX_CTRL_ServoCalibration*);\n"
        "uint8_t DRV_COAX_CTRL_ValidateServoCalibration(const DRV_COAX_CTRL_ServoCalibration*);\n",
        encoding="ascii",
    )
    (stub_dir / "svc_param.h").write_text(
        "#include <stdint.h>\n"
        "typedef enum { SVC_PARAM_STATUS_OK = 0 } SVC_ParamStatus;\n"
        "uint8_t SVC_Param_IsDirty(void);\n"
        "SVC_ParamStatus SVC_Param_SetBlob(const uint8_t*,uint32_t);\n"
        "uint32_t SVC_Param_RequestSaveBlob(void);\n",
        encoding="ascii",
    )
    (stub_dir / "app_flight_calibration.h").write_text(
        "#include <stdint.h>\n"
        "typedef struct { uint32_t calibration_generation; uint8_t bytes[156]; } APP_FlightCalibration;\n"
        "typedef struct { APP_FlightCalibration calibration; uint32_t generation; } APP_FlightCalibrationSnapshot;\n"
        "typedef enum { APP_FLIGHT_CAL_UPLOAD_EMPTY = 0, APP_FLIGHT_CAL_UPLOAD_READY = 3 } APP_FlightCalibrationUploadState;\n"
        "typedef struct { APP_FlightCalibrationUploadState state; } APP_FlightCalibrationUpload;\n"
        "uint8_t APP_FlightCalibration_ReadActive(APP_FlightCalibrationSnapshot*);\n"
        "uint8_t APP_FlightCalibration_BuildServoMechanical(const APP_FlightCalibration*,void*);\n"
        "uint8_t APP_FlightCalibration_UpdateServoMechanical(APP_FlightCalibration*,const void*);\n"
        "uint8_t APP_FlightCalibration_PublishPreview(const APP_FlightCalibration*);\n"
        "uint32_t APP_FlightCalibration_Encode(const APP_FlightCalibration*,uint8_t*,uint32_t);\n",
        encoding="ascii",
    )


def _check_app_control_step_c(tmp_path: Path) -> None:
    assert SERVOCAL_CMD.is_file()
    legacy = CONTROL.read_text(encoding="utf-8")
    servocal = SERVOCAL_CMD.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    for name, expected_hash in STEP_C_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(servocal, name).encode()).hexdigest() == expected_hash
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
            rf"{re.escape(name)}\s*\(",
            legacy,
            re.MULTILINE,
        ), name
    for name in STEP_C_STATE_NAMES:
        assert re.search(rf"^static .*\b{re.escape(name)}(?:\b|;)", servocal, re.MULTILINE)
        assert name not in legacy
    assert len(servocal.splitlines()) <= 800
    assert "extern" not in servocal
    assert STEP_C_PERSISTED_FRAGMENT in servocal
    assert STEP_C_INIT_FRAGMENT in servocal
    sync = _c_function_body(legacy, "app_control_imuframe_sync_param")
    assert sync.index("app_control_imucal_clear_candidate();") < sync.index(
        "app_cmd_servocal_on_persisted(&calibration);"
    ) < sync.index("control_imuframe_confirmed_code = orientation_code;")
    report = _c_function_body(servocal, "app_control_report_servocal")
    handler = _c_function_body(servocal, "app_control_handle_servocal")
    assert report.index("app_control_imuframe_sync_param();") < report.index("memset(&active")
    assert handler.index("app_control_imuframe_sync_param();") < handler.index('strcmp(tokens[0], "SERVOCAL?")')
    for call in (
        "app_cmd_servocal_init();",
        "app_cmd_servocal_on_persisted(&calibration);",
        "app_cmd_servocal_is_busy()",
        "app_control_service_servocal();",
        "app_control_handle_servocal(tokens, count);",
    ):
        assert call in legacy
    for declaration in (
        "void app_control_handle_servocal(char **tokens, uint32_t count);",
        "void app_control_service_servocal(void);",
        "void app_cmd_servocal_init(void);",
        "void app_cmd_servocal_on_persisted(const void *record);",
        "uint8_t app_cmd_servocal_is_busy(void);",
    ):
        assert declaration in internal

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "servocal_stubs"
    stub_dir.mkdir()
    _write_servocal_stubs(stub_dir)
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            "-c",
            str(SERVOCAL_CMD),
            "-o",
            str(tmp_path / "app_cmd_servocal.o"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _check_app_control_step_a(tmp_path: Path) -> None:
    assert CONTROL_CORE.is_file()
    assert CONTROL_INTERNAL.is_file()
    control = CONTROL.read_text(encoding="utf-8")
    core = CONTROL_CORE.read_text(encoding="utf-8")
    internal = CONTROL_INTERNAL.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")

    for name, expected_hash in STEP_A_BODY_SHA256.items():
        assert hashlib.sha256(_c_function_body(core, name).encode()).hexdigest() == expected_hash
        assert not re.search(
            rf"^(?:static\s+)?(?:void|uint8_t|uint32_t|const char \*\s*)\s*"
            rf"{re.escape(name)}\s*\(",
            control,
            re.MULTILINE,
        ), name
    assert '#include "app_control_internal.h"' in control
    assert "App/Src/app_control_core.c" in cmake
    assert "extern" not in core
    assert "#define APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US 100000ULL" in control
    assert "#define APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS 10U" in internal
    assert re.findall(
        r"#define\s+APP_CONTROL_IMUCAL_SNAPSHOT_MAX_AGE_US\s+(\S+)",
        control + core + internal,
    ) == ["100000ULL"]
    assert re.findall(
        r"#define\s+APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS\s+(\S+)",
        control + core + internal,
    ) == ["10U"]
    assert "return control_maint_output_active;" in control

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub_dir = tmp_path / "control_core_stubs"
    _write_core_stubs(stub_dir)
    harness = tmp_path / "control_core_harness.c"
    harness.write_text(CORE_HARNESS, encoding="ascii")
    executable = tmp_path / "control_core_harness.exe"
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{stub_dir}",
            f"-I{ROOT / 'App' / 'Inc'}",
            str(CONTROL_CORE),
            str(harness),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_servo_cal_state_machine_never_calls_blocking_text_send() -> None:
    source = SERVO_CAL.read_text(encoding="utf-8")
    assert "APP_Control_QueueText(" not in source, (
        "app_servo_cal.c runs inside the 500Hz control loop; it must post "
        "notices instead of calling the blocking USB/UART text path"
    )
    assert '#include "app_control.h"' not in source, (
        "the control-loop module must not depend on app_control.h at all"
    )


def test_servo_cal_posts_notices_through_the_pending_buffer() -> None:
    source = SERVO_CAL.read_text(encoding="utf-8")
    assert "servo_cal_post_notice(" in source
    assert "servo_cal_notice_pending" in source
    assert "uint16_t APP_ServoCal_TakeNotice(" in source
    header = SERVO_CAL_HEADER.read_text(encoding="utf-8")
    assert "APP_ServoCal_TakeNotice" in header


def test_comms_task_flushes_the_notice_each_tick(tmp_path: Path) -> None:
    source = CONTROL.read_text(encoding="utf-8")
    flush = source.find("APP_ServoCal_TakeNotice(")
    tick = source.find("static void app_control_tick_common(uint8_t emit_heartbeat)\n{")
    assert flush != -1, "app_control.c must flush the servo-cal notice"
    assert tick != -1, "app_control_tick_common definition not found"
    body = source[tick : source.find("\n}", tick)]
    assert "app_control_service_servo_cal_notice()" in body, (
        "app_control_tick_common must call the notice flush service"
    )
    _check_app_control_step_a(tmp_path)
    _check_app_control_step_b()
    _check_app_control_step_c(tmp_path)
