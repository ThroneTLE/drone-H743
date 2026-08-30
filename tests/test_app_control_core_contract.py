"""Core contract."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
CORE_SRC = ROOT / "App" / "Src" / "app_control_core.c"
CORE_HDR = ROOT / "App" / "Inc" / "app_control_core.h"
CMAKELISTS = ROOT / "CMakeLists.txt"

HARNESS = r"""
#include "app_control_core.h"

#include <stdio.h>
#include <string.h>

#define CHECK(cond, code) do { if (!(cond)) return (code); } while (0)

int main(void)
{
    char buf[] = "SERVOCAL APPLY ac=1500 an=1000 ax=2000 as=-1 bc=1500 bn=1000 bx=2000 bs=-1";
    char *tokens[16];
    uint32_t count = app_control_core_tokenize(buf, tokens, 16);
    CHECK(count == 10, 1);
    CHECK(strcmp(tokens[0], "SERVOCAL") == 0, 2);
    CHECK(strcmp(tokens[1], "APPLY") == 0, 3);

    const char *ac_val = app_control_core_token_value(tokens, count, "ac");
    CHECK(ac_val != NULL, 4);
    CHECK(strcmp(ac_val, "1500") == 0, 5);

    uint32_t u32_val = 0;
    CHECK(app_control_core_parse_u32("1500", &u32_val) == 1, 6);
    CHECK(u32_val == 1500, 7);

    int32_t i32_val = 0;
    CHECK(app_control_core_parse_i32("-1", &i32_val) == 1, 8);
    CHECK(i32_val == -1, 9);

    uint8_t data[] = {1, 2, 3, 4};
    uint32_t crc = app_control_core_crc32(data, 4);
    CHECK(crc != 0, 10);

    return 0;
}
"""


def test_core(tmp_path: Path) -> None:
    lines = len(CORE_SRC.read_text(encoding="utf-8").splitlines())
    assert lines <= 800, f"app_control_core.c is {lines} lines (> 800)"
    assert "App/Src/app_control_core.c" in CMAKELISTS.read_text(encoding="utf-8")

    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc not available")

    harness_c = tmp_path / "harness.c"
    harness_c.write_text(HARNESS, encoding="utf-8")
    out_exe = tmp_path / "test_core.exe"

    cmd = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{ROOT / 'App' / 'Inc'}",
        f"-I{ROOT / 'Driver' / 'Inc'}",
        f"-I{ROOT / 'Services' / 'Inc'}",
        str(CORE_SRC),
        str(harness_c),
        f"-I{tmp_path}",
        "-o",
        str(out_exe),
    ]

    # Create stub cmsis_os2.h and other stubs in tmp_path for host build
    cmsis_stub = tmp_path / "cmsis_os2.h"
    cmsis_stub.write_text(
        "#pragma once\n"
        "#include <stdint.h>\n"
        "typedef void *osMessageQueueId_t;\n"
        "typedef void *osSemaphoreId_t;\n"
        "typedef int osStatus_t;\n"
        "#define osOK 0\n"
        "static inline osStatus_t osMessageQueuePut(osMessageQueueId_t q, const void *msg, uint8_t prio, uint32_t timeout) { (void)q; (void)msg; (void)prio; (void)timeout; return 0; }\n"
        "static inline osStatus_t osMessageQueueGet(osMessageQueueId_t q, void *msg, uint8_t *prio, uint32_t timeout) { (void)q; (void)msg; (void)prio; (void)timeout; return 0; }\n",
        encoding="utf-8",
    )

    # Stub stm32 / hal / driver symbols
    stub_c = tmp_path / "stubs.c"
    stub_c.write_text(
        "#include \"app_flight_calibration.h\"\n"
        "#include \"app_stabilizer.h\"\n"
        "osMessageQueueId_t uartTxQueueHandle = (osMessageQueueId_t)1;\n"
        "uint8_t control_maint_output_active = 0;\n"
        "APP_FlightCalibration control_imucal_confirmed;\n"
        "uint32_t control_imucal_confirmed_generation = 0;\n"
        "uint8_t control_imucal_confirmed_valid = 0;\n"
        "uint8_t APP_IMU_Capture_IsExportActive(void) { return 0; }\n"
        "uint32_t APP_USB_CDC_Write(const uint8_t *buf, uint16_t len, uint32_t to) { (void)buf; (void)len; (void)to; return len; }\n"
        "void APP_UART_NotifyTxPending(void) {}\n"
        "void APP_MaintUART_Write(const char *buf, uint16_t len) { (void)buf; (void)len; }\n"
        "uint8_t SVC_Param_IsDirty(void) { return 0; }\n"
        "uint8_t APP_Stabilizer_ReadValidationImuSnapshot(StabilizerValidationImuSnapshot *s) { (void)s; return 1; }\n"
        "uint64_t SVC_Timestamp_Us(void) { return 1000; }\n"
        "uint8_t APP_Boot_HasSequenceAdvanced(uint32_t a, uint32_t b) { (void)a; (void)b; return 1; }\n",
        encoding="utf-8",
    )

    cmd.append(str(stub_c))

    build = subprocess.run(cmd, capture_output=True, text=True)
    assert build.returncode == 0, f"gcc build failed:\nstdout: {build.stdout}\nstderr: {build.stderr}"

    run = subprocess.run([str(out_exe)], capture_output=True, text=True)
    assert run.returncode == 0, f"test run failed with code {run.returncode}"
