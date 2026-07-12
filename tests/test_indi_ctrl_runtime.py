from __future__ import annotations

import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_indi_runtime_dynamics(tmp_path: Path) -> None:
    harness = tmp_path / "indi_harness.c"
    executable = tmp_path / "indi_harness.exe"
    harness.write_text(
        r'''
#include "drv_indi_ctrl.h"

#include <math.h>
#include <stdio.h>

static int nearf(float a, float b, float tolerance)
{
    return fabsf(a - b) <= tolerance;
}

int main(void)
{
    DRV_INDI_Config config;
    DRV_INDI_State state;
    DRV_INDI_Input input = {0};
    DRV_INDI_Output output;
    DRV_INDI_State light_state;
    DRV_INDI_State heavy_state;
    DRV_INDI_Output light_output;
    DRV_INDI_Output heavy_output;
    DRV_INDI_Config heavy_config;

    DRV_INDI_DefaultConfig(&config);
    DRV_INDI_Reset(&state);
    config.enable = 0.0f;
    input.base_alpha_rad = 0.10f;
    input.base_beta_rad = -0.20f;
    input.total_force_n = 7.4f;
    input.tilt_lever_arm_m = 0.18f;
    input.dt_sec = 0.002f;

    DRV_INDI_Step(&state, &config, &input, &output);
    if (output.active != 0U ||
        !nearf(output.alpha_rad, 0.10f, 1.0e-6f) ||
        !nearf(output.beta_rad, -0.20f, 1.0e-6f)) {
        return 1;
    }

    config.enable = 1.0f;
    config.angular_accel_lpf_alpha = 0.0f;
    config.increment_limit_rad = 0.5f;
    config.correction_limit_rad = 0.5f;
    config.correction_leak_hz = 0.0f;
    config.roll_attitude_kp_rad_s2_per_rad = 0.0f;
    config.pitch_attitude_kp_rad_s2_per_rad = 0.0f;
    config.roll_rate_kd_rad_s2_per_rad_s = 0.0f;
    config.pitch_rate_kd_rad_s2_per_rad_s = 0.0f;
    DRV_INDI_Reset(&light_state);
    DRV_INDI_Reset(&heavy_state);

    DRV_INDI_Step(&light_state, &config, &input, &light_output);
    DRV_INDI_Step(&heavy_state, &config, &input, &heavy_output);
    input.gyro_x_rad_s = 0.02f;
    DRV_INDI_Step(&light_state, &config, &input, &light_output);
    heavy_config = config;
    heavy_config.roll_inertia_kg_m2 = 2.0f * config.roll_inertia_kg_m2;
    DRV_INDI_Reset(&heavy_state);
    input.gyro_x_rad_s = 0.0f;
    DRV_INDI_Step(&heavy_state, &heavy_config, &input, &heavy_output);
    input.gyro_x_rad_s = 0.02f;
    DRV_INDI_Step(&heavy_state, &heavy_config, &input, &heavy_output);

    if (!(light_output.correction_rad[0] < 0.0f)) {
        return 2;
    }
    if (!nearf(heavy_output.correction_rad[0],
               2.0f * light_output.correction_rad[0],
               1.0e-4f)) {
        return 3;
    }

    config.increment_limit_rad = 0.01f;
    config.correction_limit_rad = 0.015f;
    DRV_INDI_Reset(&state);
    input.gyro_x_rad_s = 0.0f;
    input.roll_rad = -1.0f;
    input.roll_ref_rad = 0.0f;
    config.roll_attitude_kp_rad_s2_per_rad = 100.0f;
    DRV_INDI_Step(&state, &config, &input, &output);
    DRV_INDI_Step(&state, &config, &input, &output);
    DRV_INDI_Step(&state, &config, &input, &output);
    if (!nearf(output.correction_rad[0], 0.015f, 1.0e-6f)) {
        return 4;
    }

    puts("indi runtime ok");
    return 0;
}
''',
        encoding="utf-8",
    )

    compile_result = subprocess.run(
        [
            "gcc",
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(harness),
            str(ROOT / "Driver" / "Src" / "drv_indi_ctrl.c"),
            "-lm",
            "-o",
            str(executable),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert compile_result.returncode == 0, compile_result.stderr

    run_result = subprocess.run(
        [str(executable)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert run_result.returncode == 0, run_result.stdout + run_result.stderr
    assert "indi runtime ok" in run_result.stdout
