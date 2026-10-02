"""Compile the real MAGXY command/default-on persistence module on host."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
FAKE_MAIN_H = """
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#endif
"""
FAKE_CMSIS_OS2_H = """
#ifndef FAKE_CMSIS_OS2_H
#define FAKE_CMSIS_OS2_H
typedef void *osSemaphoreId_t;
typedef void *osMessageQueueId_t;
typedef void *osMutexId_t;
typedef void *osThreadId_t;
#endif
"""

HARNESS = r"""
#include "app_magxy.h"
#include "app_stabilizer.h"
#include "app_control_internal.h"
#include "drv_frame_contract.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x, n) do { if (!(x)) { fprintf(stderr, "check %d\n", n); return n; } } while (0)

static char log_buf[2048];
static uint8_t armed;
static int critical_depth;
static unsigned save_calls;
static uint8_t save_status;

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;
    size_t used = strlen(log_buf);
    va_start(args, format);
    (void)vsnprintf(log_buf + used, sizeof(log_buf) - used, format, args);
    va_end(args);
}
uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed = strtoul(text, &end, 10);
    if ((text[0] == '\0') || (*end != '\0')) { return 0U; }
    *value = (uint32_t)parsed;
    return 1U;
}
uint8_t app_control_parse_f32(const char *text, float *value)
{
    char *end;
    float parsed = strtof(text, &end);
    if ((*end != '\0') || !isfinite(parsed)) { return 0U; }
    *value = parsed;
    return 1U;
}
uint8_t APP_Stabilizer_IsArmed(void) { return armed; }
void APP_Stabilizer_GetMagXYStatus(APP_Stabilizer_MagXYStatus *out)
{
    memset(out, 0, sizeof(*out));
    out->ready_count = 7U;
    out->z_mean_mgauss = 699.25f;
}
uint32_t BSP_Critical_Enter(void) { return (uint32_t)critical_depth++; }
void BSP_Critical_Exit(uint32_t state) { critical_depth = (int)state; }
uint8_t app_control_internal_commit_config_persist(void)
{
    ++save_calls;
    return save_status;
}

int main(void)
{
    SVC_MAG_HeadingConfig config;
    APP_MagXY_Persisted persisted;
    char *query[] = {"MAGXY?"};
    char *enable[] = {"MAGXY", "ENABLE", "1"};
    char *set[] = {"MAGXY", "SET", "-3.725", "135.432", "268.292"};
    char *verify[] = {"MAGXY", "VERIFY", "CONFIRM"};
    char *clear[] = {"MAGXY", "CLEAR"};
    char *commit[] = {"MAGXY", "COMMIT"};
    char *unknown[] = {"NOT_MAGXY"};

    CHECK(APP_MagXY_HandleCommand(unknown, 1U) == 0U, 1);
    CHECK(APP_MagXY_HandleCommand(query, 1U) == 1U, 2);
    CHECK(strstr(log_buf, "configured=0 enabled=0 axis_effective=0") != NULL, 3);
    CHECK(APP_MagXY_HandleCommand(enable, 3U) == 1U, 4);
    CHECK(strstr(log_buf, "reason=unconfigured") != NULL, 5);
    CHECK(APP_MagXY_HandleCommand(set, 5U) == 1U, 6);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 0U && config.axis_verified == 0U, 7);
    CHECK(fabsf(config.bias_x_mgauss + 3.725f) < 0.001f, 8);
    CHECK(APP_MagXY_HandleCommand(enable, 3U) == 1U, 9);
    CHECK(strstr(log_buf, "reason=axis_unverified") != NULL, 10);
    CHECK(APP_MagXY_HandleCommand(verify, 3U) == 1U, 11);
    APP_MagXY_GetConfig(&config);
    CHECK(config.axis_verified == 1U, 12);
    CHECK(config.frame_contract_version == DRV_FRAME_CONTRACT_VERSION, 13);
    CHECK(config.enabled == 1U, 131);
    CHECK(APP_MagXY_HandleCommand(enable, 3U) == 1U, 14);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 1U, 15);
    log_buf[0] = '\0';
    CHECK(APP_MagXY_HandleCommand(query, 1U) == 1U, 16);
    CHECK(strstr(log_buf, "configured=1 enabled=1 axis_effective=1") != NULL, 17);
    CHECK(strstr(log_buf, "ready=7") != NULL, 18);
    CHECK(APP_MagXY_HandleCommand(commit, 2U) == 1U, 181);
    CHECK(save_calls == 1U, 182);
    APP_MagXY_GetPersisted(&persisted);
    CHECK(persisted.enabled_default == 1U, 183);
    APP_MagXY_ApplyPersisted(NULL);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 0U, 184);
    APP_MagXY_ApplyPersisted(&persisted);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 1U, 185);
    persisted.frame_contract_version++;
    APP_MagXY_ApplyPersisted(&persisted);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 0U, 186);
    persisted.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;
    APP_MagXY_ApplyPersisted(&persisted);
    save_status = 1U;
    CHECK(APP_MagXY_HandleCommand(commit, 2U) == 1U, 187);
    CHECK(strstr(log_buf, "commit_failed") != NULL, 188);
    save_status = 0U;
    armed = 1U;
    CHECK(APP_MagXY_HandleCommand(clear, 2U) == 1U, 19);
    CHECK(APP_MagXY_HandleCommand(commit, 2U) == 1U, 191);
    CHECK(save_calls == 2U, 192);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 1U, 20);
    armed = 0U;
    CHECK(APP_MagXY_HandleCommand(clear, 2U) == 1U, 21);
    APP_MagXY_GetConfig(&config);
    CHECK(config.enabled == 0U && config.axis_verified == 0U, 22);
    CHECK(APP_MagXY_HandleCommand(commit, 2U) == 1U, 221);
    CHECK(save_calls == 3U, 222);
    CHECK(critical_depth == 0, 23);
    return 0;
}
"""


def test_real_magxy_command_module(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc unavailable")
    fakes = tmp_path / "fakes"
    fakes.mkdir()
    (fakes / "main.h").write_text(FAKE_MAIN_H, encoding="ascii")
    (fakes / "cmsis_os2.h").write_text(FAKE_CMSIS_OS2_H, encoding="ascii")
    harness = tmp_path / "magxy_harness.c"
    harness.write_text(HARNESS, encoding="ascii")
    executable = tmp_path / "magxy_harness.exe"
    command = [
        gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
        f"-I{fakes}", f"-I{ROOT / 'App' / 'Inc'}",
        f"-I{ROOT / 'Driver' / 'Inc'}", f"-I{ROOT / 'BSP' / 'Inc'}",
        f"-I{ROOT / 'Services' / 'Inc'}",
        str(ROOT / "App/Src/app_magxy.c"),
        str(ROOT / "Services/Src/svc_mag_heading.c"),
        str(harness), "-lm", "-o", str(executable),
    ]
    built = subprocess.run(command, capture_output=True, text=True)
    assert built.returncode == 0, built.stdout + built.stderr
    run = subprocess.run([str(executable)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
