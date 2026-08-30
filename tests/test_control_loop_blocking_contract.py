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
    match = pattern.search(source)
    assert match is not None, name
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
