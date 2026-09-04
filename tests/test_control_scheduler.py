from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r"""
#include "app_control_scheduler.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static unsigned rate_count, attitude_count, velocity_count, position_count;

static int near(float a, float b) { return fabsf(a - b) < 1.0e-6f; }

int main(void) {
    APP_ControlSchedulerState state;
    APP_ControlSchedule out;
    APP_ControlScheduler_Reset(&state);

    for (uint64_t t = 2000ULL; t <= 1000000ULL; t += 2000ULL) {
        uint64_t nav = ((t % 10000ULL) == 0ULL) ? t : state.last_nav_sample_us;
        APP_ControlScheduler_Step(&state, t, nav, nav != 0ULL, &out);
        rate_count += out.rate_due;
        attitude_count += out.attitude_due;
        velocity_count += out.velocity_due;
        position_count += out.position_due;
    }
    if (rate_count != 500U || attitude_count != 250U ||
        velocity_count != 100U || position_count != 50U) return 1;

    /* Reusing a navigation timestamp must not rerun or reintegrate slow loops. */
    APP_ControlScheduler_Step(&state, 1002000ULL, 1000000ULL, 1U, &out);
    if (out.velocity_due || out.position_due || out.navigation_sample_new) return 2;

    /* Repeated/backward controller timestamps are rejected, not integrated. */
    APP_ControlScheduler_Step(&state, 1002000ULL, 1002000ULL, 1U, &out);
    if (!out.timestamp_fault || out.rate_due || out.attitude_due) return 3;

    /* A long gap is bounded and explicitly marked. */
    APP_ControlScheduler_Step(&state, 1302000ULL, 1302000ULL, 1U, &out);
    if (!out.timestamp_fault || !out.rate_due || !out.attitude_due ||
        !out.velocity_due || !out.position_due) return 4;
    if (!near(out.rate_dt_s, 0.1f) || !near(out.velocity_dt_s, 0.1f)) return 5;

    APP_ControlScheduler_Step(&state, 1304000ULL, 1400000ULL, 1U, &out);
    if (!out.timestamp_fault || out.navigation_sample_new ||
        out.velocity_due || out.position_due) return 6;

    printf("rate=%u attitude=%u velocity=%u position=%u\n",
           rate_count, attitude_count, velocity_count, position_count);
    return 0;
}
"""


def test_control_scheduler_host_harness(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("gcc is required for the pure-C scheduler harness")
    harness = tmp_path / "scheduler_harness.c"
    exe = tmp_path / "scheduler_harness.exe"
    harness.write_text(HARNESS, encoding="utf-8")
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(ROOT / "App" / "Inc"),
            str(harness),
            str(ROOT / "App" / "Src" / "app_control_scheduler.c"),
            "-o",
            str(exe),
        ],
        check=True,
        cwd=ROOT,
    )
    result = subprocess.run([str(exe)], check=False, capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "rate=500 attitude=250 velocity=100 position=50" in result.stdout


def test_scheduler_is_pure_and_declares_real_timestamp_contract() -> None:
    source = (ROOT / "App" / "Src" / "app_control_scheduler.c").read_text(
        encoding="utf-8"
    )
    header = (ROOT / "App" / "Inc" / "app_control_scheduler.h").read_text(
        encoding="utf-8"
    )
    forbidden = ("cmsis_os", "FreeRTOS", "stm32", "HAL_", "APP_USB", "Flash")
    assert not any(token in source for token in forbidden)
    assert "navigation sample timestamp" in header
    assert "APP_CONTROL_SCHED_RATE_PERIOD_US      2000ULL" in header
    assert "APP_CONTROL_SCHED_ATTITUDE_PERIOD_US  4000ULL" in header
    assert "APP_CONTROL_SCHED_VELOCITY_PERIOD_US 10000ULL" in header
    assert "APP_CONTROL_SCHED_POSITION_PERIOD_US 20000ULL" in header
