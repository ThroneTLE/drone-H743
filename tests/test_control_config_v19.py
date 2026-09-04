"""CFG V19 cascade migration contract; synthetic records are unit tests only."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r"""
#include "app_control_config_compat.h"
#include <math.h>
#include <string.h>

#define CHECK(x, n) do { if (!(x)) return (n); } while (0)
#define NEAR(a,b) (fabsf((a)-(b)) < 1.0e-5f)

int main(void) {
    APP_ControlCoaxTunableParamsV18 old = {0};
    APP_ControlCoaxTunableParams now;
    old.pos_x_kp = 0.30f; old.pos_y_kp = 0.30f; old.pos_z_kp = 3.8f;
    old.pos_z_ki = 0.25f;
    old.vel_x_kd = 0.80f; old.vel_y_kd = 0.80f; old.vel_z_kd = 0.0f;
    old.vel_loop_enable = 1.0f;
    old.roll_angle_kp = 0.0671f; old.pitch_angle_kp = 0.0660f;
    old.roll_rate_kd = 0.1104f; old.pitch_rate_kd = 0.1138f;
    old.tilt_limit_rad = 0.4886922f;
    old.yaw_angle_kp = 1.0f; old.yaw_rate_kd = 0.15f;
    CHECK(APP_ControlConfigCompat_V18ToCurrent(&old, &now), 1);
    CHECK(NEAR(now.pos_x_kp, 0.375f) && NEAR(now.vel_x_kp, 0.8f), 2);
    CHECK(NEAR(now.pos_x_kp * now.vel_x_kp, old.pos_x_kp), 3);
    CHECK(NEAR(now.pos_y_kp * now.vel_y_kp, old.pos_y_kp), 4);
    CHECK(NEAR(now.vel_z_kp, 1.0f) && NEAR(now.pos_z_kp, 3.8f), 5);
    CHECK(now.vel_x_ki == 0.0f && now.vel_y_ki == 0.0f &&
          now.vel_z_ki == 0.0f && now.vel_x_kd == 0.0f &&
          now.vel_y_kd == 0.0f && now.vel_z_kd == 0.0f, 6);
    CHECK(NEAR(now.rate_roll_kp * now.att_roll_kp, old.roll_angle_kp), 7);
    CHECK(NEAR(now.rate_pitch_kp * now.att_pitch_kp, old.pitch_angle_kp), 8);
    CHECK(NEAR((now.rate_yaw_kp * now.att_yaw_kp) / 0.00035f,
               old.yaw_angle_kp), 9);

    APP_ControlCoaxTunableParamsV17 v17 = {0};
    memcpy(&v17, &old, 8U * sizeof(float));
    v17.roll_angle_kp = old.roll_angle_kp; v17.pitch_angle_kp = old.pitch_angle_kp;
    v17.roll_rate_kd = old.roll_rate_kd; v17.pitch_rate_kd = old.pitch_rate_kd;
    v17.tilt_limit_rad = old.tilt_limit_rad;
    v17.yaw_angle_kp = old.yaw_angle_kp; v17.yaw_rate_kd = old.yaw_rate_kd;
    CHECK(APP_ControlConfigCompat_V17ToCurrent(&v17, &now), 10);

    APP_ControlCoaxTunableParamsV15 v15 = {0};
    v15.pos_x_kp = old.pos_x_kp; v15.pos_y_kp = old.pos_y_kp;
    v15.pos_z_kp = old.pos_z_kp; v15.vel_x_kd = old.vel_x_kd;
    v15.vel_y_kd = old.vel_y_kd; v15.roll_angle_kp = old.roll_angle_kp;
    v15.pitch_angle_kp = old.pitch_angle_kp; v15.roll_rate_kd = old.roll_rate_kd;
    v15.pitch_rate_kd = old.pitch_rate_kd; v15.tilt_limit_rad = old.tilt_limit_rad;
    v15.yaw_angle_kp = old.yaw_angle_kp; v15.yaw_rate_kd = old.yaw_rate_kd;
    CHECK(APP_ControlConfigCompat_V15ToCurrent(&v15, &now), 11);
    return 0;
}
"""


def test_v18_v17_v15_migration_math_on_host(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required")
    harness = tmp_path / "cfg_v19.c"
    exe = tmp_path / "cfg_v19.exe"
    harness.write_text(HARNESS, encoding="ascii")
    result = subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_control_config_compat.c"),
            str(harness),
            "-lm",
            "-o",
            str(exe),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stdout + run.stderr


def test_cfg_v19_store_is_extracted_and_backward_compatible() -> None:
    store = (ROOT / "App" / "Src" / "app_control_config_store.c").read_text(
        encoding="utf-8"
    )
    header = (ROOT / "App" / "Inc" / "app_control_config_store.h").read_text(
        encoding="utf-8"
    )
    control = (ROOT / "App" / "Src" / "app_control.c").read_text(encoding="utf-8")
    assert "APP_CONTROL_CFG_VERSION     19U" in header
    for version in ("V18", "V17", "V16", "V15"):
        assert f"APP_CONTROL_CFG_VERSION_{version}" in header
        assert f"config_read_{version.lower()}" in store
    assert "config_checksum" in store
    assert "APP_ControlConfigStore_Load(&control_config)" in control
    assert "APP_ControlConfigStore_Save(&control_config)" in control
    assert "APP_ControlFlashRecordV17" not in control
    assert "pos_z_ki" not in store
