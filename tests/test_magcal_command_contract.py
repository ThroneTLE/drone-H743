"""Host-side tests for the MAGCAL / MAGFRAME command family
(App/Src/app_cmd_magcal.c) and the C6 frame-contract-version invalidation
rule exposed through App/Inc/app_magcal.h.

Compiles the real app_cmd_magcal.c and the real drv_mag_calibration.c with
host gcc, stubbing only the handful of App-layer symbols it depends on
(APP_Control_QueueText, the parse helpers, APP_Stabilizer_IsArmed, the
commit-persist trigger, and the critical-section primitive) -- the same
"stub the boundary, run the real module" style already used for LEDMAP and
PROPCAL in tests/test_config_store_ab_slots.py.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

FAKE_MAIN_H = r"""
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H
/* app_flash_service.h -> drv_gd25q32.h only needs the *names* of these two
 * types (never dereferenced); mirrors tests/test_config_store_ab_slots.py. */
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#endif
"""

FAKE_CMSIS_OS2_H = r"""
#ifndef FAKE_CMSIS_OS2_H
#define FAKE_CMSIS_OS2_H
/* app_stabilizer.h only uses these as handle-type names, never calls into
 * the RTOS; mirrors tests/test_config_store_ab_slots.py. */
typedef void *osSemaphoreId_t;
typedef void *osMessageQueueId_t;
typedef void *osMutexId_t;
typedef void *osThreadId_t;
#endif
"""

HARNESS = r"""
#include "app_magcal.h"
#include "app_magxy.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "drv_frame_contract.h"
#include "drv_mag_calibration.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) { \
    fprintf(stderr, "line %d, code %d: %s\n", __LINE__, (code), #condition); \
    return (code); } } while (0)

/* ---- App/Src/app_cmd_magcal.c dependencies, stubbed at the boundary ---- */

static char log_buf[16384];
static size_t log_len;

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;
    int written;

    va_start(args, format);
    written = vsnprintf(log_buf + log_len, sizeof(log_buf) - log_len, format,
                        args);
    va_end(args);
    if (written > 0) {
        log_len += (size_t)written;
        if (log_len >= sizeof(log_buf)) {
            log_len = sizeof(log_buf) - 1U;
        }
    }
}

static void log_reset(void) { log_len = 0U; log_buf[0] = '\0'; }
static int log_has(const char *needle) { return strstr(log_buf, needle) != NULL; }

/* Strict-enough host stand-ins for the real app_control_core.c parsers. */
uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == NULL) || (text[0] == '\0')) { return 0U; }
    parsed = strtoul(text, &end, 10);
    if (*end != '\0') { return 0U; }
    *value = (uint32_t)parsed;
    return 1U;
}

uint8_t app_control_parse_f32(const char *text, float *value)
{
    char *end;
    float parsed;

    if ((text == NULL) || (text[0] == '\0')) { return 0U; }
    parsed = strtof(text, &end);
    if ((*end != '\0') || !isfinite(parsed)) { return 0U; }
    *value = parsed;
    return 1U;
}

static uint8_t stub_armed;
uint8_t APP_Stabilizer_IsArmed(void) { return stub_armed; }

static uint8_t stub_commit_status; /* APP_FLASH_SERVICE_OK == 0 */
uint8_t app_control_internal_commit_config_persist(void)
{
    return stub_commit_status;
}

static int critical_depth;
uint32_t BSP_Critical_Enter(void) { return (uint32_t)critical_depth++; }
void BSP_Critical_Exit(uint32_t state) { critical_depth = (int)state; }

/* app_stabilizer.c's real fusion-status mirror does not build in this
 * RTOS-free harness; MAGCAL? only needs a linkable stand-in here, its
 * content is covered by tests/test_attitude_fusion_magnetometer.py. */
void APP_Stabilizer_GetMagFusionStatus(APP_Stabilizer_MagFusionStatus *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
}

/* MAGXY has a separate real-module harness; legacy MAGCAL behaviour stays
 * isolated here so its command ABI can be compared unchanged. */
uint8_t APP_MagXY_HandleCommand(char **tokens, uint32_t count)
{
    (void)tokens; (void)count;
    return 0U;
}

/* ---- helpers ---- */

/* Minimal space tokenizer (mirrors app_control_tokenize()'s contract just
 * enough for this harness); avoids relying on the non-C11 strtok_r. */
static uint8_t send(const char *line)
{
    static char buffer[128];
    char *tokens[10];
    uint32_t count = 0U;
    char *cursor;

    log_reset();
    strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';

    cursor = buffer;
    while ((*cursor != '\0') && (count < 10U)) {
        while (*cursor == ' ') { *cursor++ = '\0'; }
        if (*cursor == '\0') { break; }
        tokens[count++] = cursor;
        while ((*cursor != '\0') && (*cursor != ' ')) { cursor++; }
    }
    return app_control_handle_magcal(tokens, count);
}

static const DRV_MAG_Calibration *current(void)
{
    return (const DRV_MAG_Calibration *)app_cmd_magcal_config();
}

int main(void)
{
    DRV_MAG_Calibration external;

    /* Fresh state: never loaded, must read as the C3 default. */
    CHECK(send("MAGCAL?") == 1U, 1);
    CHECK(log_has("calibrated=0"), 2);
    CHECK(log_has("axis_verified=0"), 3);
    CHECK(log_has("axis_effective=0"), 4);
    CHECK(log_has("dirty=0"), 5);
    CHECK(current()->calibrated == 0U, 6);
    CHECK(current()->soft_iron_matrix[0][0] == 1.0f, 7);
    CHECK(current()->soft_iron_matrix[1][1] == 1.0f, 8);
    CHECK(current()->soft_iron_matrix[2][2] == 1.0f, 9);

    /* Unclaimed commands fall through. */
    {
        char *tokens[1] = { (char *)"UNRELATED" };
        CHECK(app_control_handle_magcal(tokens, 1U) == 0U, 10);
    }
    {
        CHECK(app_control_handle_magcal(NULL, 0U) == 0U, 11);
    }

    /* Bad SET usage: wrong arg count / non-numeric. */
    CHECK(send("MAGCAL SET BIAS 1 2") == 1U, 12);
    CHECK(log_has("ERR usage MAGCAL SET BIAS"), 13);
    CHECK(send("MAGCAL SET BIAS x y z") == 1U, 14);
    CHECK(log_has("ERR usage MAGCAL SET BIAS"), 15);
    CHECK(send("MAGCAL SET MATRIX 3 1 0 0") == 1U, 16); /* row out of [0,2] */
    CHECK(log_has("ERR usage MAGCAL SET MATRIX"), 17);
    CHECK(send("MAGCAL BOGUS") == 1U, 18);
    CHECK(log_has("ERR usage MAGCAL SET|APPLY|COMMIT|CLEAR"), 19);

    /* APPLY rejects an incomplete draft. */
    CHECK(send("MAGCAL SET BIAS 1 2 3") == 1U, 20);
    CHECK(log_has("state=draft_bias_set"), 21);
    CHECK(send("MAGCAL APPLY") == 1U, 22);
    CHECK(log_has("apply_rejected reason=incomplete_draft"), 23);
    CHECK(current()->calibrated == 0U, 24); /* untouched by a rejected apply */

    /* Complete the draft (identity matrix) and apply. */
    CHECK(send("MAGCAL SET MATRIX 0 1 0 0") == 1U, 25);
    CHECK(send("MAGCAL SET MATRIX 1 0 1 0") == 1U, 26);
    CHECK(send("MAGCAL SET MATRIX 2 0 0 1") == 1U, 27);
    CHECK(send("MAGCAL APPLY") == 1U, 28);
    CHECK(log_has("state=applied_ram"), 29);
    CHECK(current()->calibrated == 1U, 30);
    CHECK(current()->hard_iron_bias_mgauss[0] == 1.0f, 31);
    CHECK(current()->hard_iron_bias_mgauss[1] == 2.0f, 32);
    CHECK(current()->hard_iron_bias_mgauss[2] == 3.0f, 33);

    /* A degenerate (singular) matrix is rejected, and the prior good
     * calibration survives untouched. */
    CHECK(send("MAGCAL SET BIAS 9 9 9") == 1U, 34);
    CHECK(send("MAGCAL SET MATRIX 0 0 0 0") == 1U, 35);
    CHECK(send("MAGCAL SET MATRIX 1 0 0 0") == 1U, 36);
    CHECK(send("MAGCAL SET MATRIX 2 0 0 0") == 1U, 37);
    CHECK(send("MAGCAL APPLY") == 1U, 38);
    CHECK(log_has("apply_rejected reason=bad_determinant"), 39);
    CHECK(current()->hard_iron_bias_mgauss[0] == 1.0f, 40); /* unchanged */

    /* Armed blocks writes but not reads. */
    stub_armed = 1U;
    CHECK(send("MAGCAL SET BIAS 5 5 5") == 1U, 41);
    CHECK(log_has("state=armed_blocked"), 42);
    CHECK(send("MAGFRAME VERIFY CONFIRM") == 1U, 43);
    CHECK(log_has("state=armed_blocked"), 44);
    CHECK(send("MAGCAL?") == 1U, 45);
    CHECK(!log_has("armed_blocked"), 46);
    stub_armed = 0U;

    /* MAGFRAME VERIFY requires the literal CONFIRM token. */
    CHECK(send("MAGFRAME VERIFY") == 1U, 47);
    CHECK(log_has("ERR usage MAGFRAME VERIFY CONFIRM"), 48);
    CHECK(current()->axis_verified == DRV_MAG_CAL_AXIS_UNVERIFIED, 49);
    CHECK(send("MAGFRAME VERIFY CONFIRM") == 1U, 50);
    CHECK(log_has("state=verified"), 51);
    CHECK(current()->axis_verified == DRV_MAG_CAL_AXIS_VERIFIED, 52);
    CHECK(current()->frame_contract_version ==
         (uint32_t)DRV_FRAME_CONTRACT_VERSION, 53);
    CHECK(send("MAGFRAME?") == 1U, 54);
    CHECK(log_has("axis_verified=1"), 55);
    CHECK(log_has("axis_effective=1"), 56);

    /* COMMIT: validates first, then delegates to the shared persist trigger. */
    stub_commit_status = 0U; /* APP_FLASH_SERVICE_OK */
    CHECK(send("MAGCAL COMMIT") == 1U, 57);
    CHECK(log_has("state=committed"), 58);
    CHECK(send("MAGCAL?") == 1U, 59);
    CHECK(log_has("dirty=0"), 60);

    CHECK(send("MAGFRAME VERIFY CONFIRM") == 1U, 61); /* make it dirty again */
    stub_commit_status = 1U; /* nonzero == failure */
    CHECK(send("MAGCAL COMMIT") == 1U, 62);
    CHECK(log_has("state=commit_failed"), 63);

    /* CLEAR resets to the C3 default and clears any in-flight draft. */
    CHECK(send("MAGCAL SET BIAS 7 7 7") == 1U, 64); /* dangling draft field */
    CHECK(send("MAGCAL CLEAR") == 1U, 65);
    CHECK(log_has("state=cleared_ram"), 66);
    CHECK(current()->calibrated == 0U, 67);
    CHECK(current()->axis_verified == DRV_MAG_CAL_AXIS_UNVERIFIED, 68);
    CHECK(current()->hard_iron_bias_mgauss[0] == 0.0f, 69);
    CHECK(send("MAGCAL APPLY") == 1U, 70); /* draft was cleared too */
    CHECK(log_has("apply_rejected reason=incomplete_draft"), 71);

    /* app_cmd_magcal_apply_config(): NULL and a corrupt record both fall
     * back to the C3 default, mirroring LEDMAP/PROPCAL migration safety. */
    memset(&external, 0, sizeof(external));
    external.calibrated = 1U;
    external.soft_iron_matrix[0][0] = 100.0f;
    external.soft_iron_matrix[1][1] = 100.0f;
    external.soft_iron_matrix[2][2] = 100.0f; /* det way over DET_MAX */
    app_cmd_magcal_apply_config(&external);
    CHECK(current()->calibrated == 0U, 72);
    CHECK(current()->soft_iron_matrix[0][0] == 1.0f, 73);

    memset(&external, 0, sizeof(external));
    external.calibrated = 1U;
    external.axis_verified = DRV_MAG_CAL_AXIS_VERIFIED;
    external.frame_contract_version = (uint32_t)DRV_FRAME_CONTRACT_VERSION;
    external.hard_iron_bias_mgauss[0] = 42.0f;
    external.soft_iron_matrix[0][0] = 1.0f;
    external.soft_iron_matrix[1][1] = 1.0f;
    external.soft_iron_matrix[2][2] = 1.0f;
    app_cmd_magcal_apply_config(&external);
    CHECK(current()->calibrated == 1U, 74);
    CHECK(current()->hard_iron_bias_mgauss[0] == 42.0f, 75);

    /* C6: a stored record whose frame_contract_version no longer matches
     * the live contract must read as axis-unverified through the
     * *effective* view -- without mutating the stored/persisted copy. */
    {
        DRV_MAG_Calibration effective;

        APP_MagCal_GetEffective(&effective);
        CHECK(effective.axis_verified == DRV_MAG_CAL_AXIS_VERIFIED, 76);
        CHECK(effective.calibrated == 1U, 77);

        memset(&external, 0, sizeof(external));
        external.calibrated = 1U;
        external.axis_verified = DRV_MAG_CAL_AXIS_VERIFIED;
        external.frame_contract_version =
            (uint32_t)DRV_FRAME_CONTRACT_VERSION + 999U; /* stale contract */
        external.soft_iron_matrix[0][0] = 1.0f;
        external.soft_iron_matrix[1][1] = 1.0f;
        external.soft_iron_matrix[2][2] = 1.0f;
        app_cmd_magcal_apply_config(&external);

        CHECK(current()->axis_verified == DRV_MAG_CAL_AXIS_VERIFIED, 78);
        APP_MagCal_GetEffective(&effective);
        CHECK(effective.axis_verified == DRV_MAG_CAL_AXIS_UNVERIFIED, 79);
        /* calibrated itself is untouched -- gate condition 2 alone already
         * excludes it from fusion; C4's other conditions are independent. */
        CHECK(effective.calibrated == 1U, 80);
    }

    /* APP_MagCal_GetEffective(NULL) must be a safe no-op. */
    APP_MagCal_GetEffective(NULL);

    return 0;
}
"""


def test_magcal_command_family(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    fakes = tmp_path / "fakes"
    fakes.mkdir()
    (fakes / "main.h").write_text(FAKE_MAIN_H, encoding="ascii")
    (fakes / "cmsis_os2.h").write_text(FAKE_CMSIS_OS2_H, encoding="ascii")

    harness = tmp_path / "magcal_harness.c"
    executable = tmp_path / "magcal_harness.exe"
    harness.write_text(HARNESS, encoding="ascii")
    result = subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{fakes}",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            f"-I{ROOT / 'BSP' / 'Inc'}",
            f"-I{ROOT / 'Services' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_cmd_magcal.c"),
            str(ROOT / "Driver" / "Src" / "drv_mag_calibration.c"),
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    run_result = subprocess.run(
        [str(executable)], capture_output=True, text=True
    )
    assert run_result.returncode == 0, run_result.stdout + run_result.stderr
