"""R-F6-2 seam 3 controller FLU migration contract."""

# Pins the migrated state (2026-09-06, doc/req-rf6-2-controller-flu-migration.md):
#
#   * position/velocity/accel (x_m/y_m/z_m, vx/vy/vz_m_s, ax/ay/az_m_s2) on
#     both DRV_COAX_CTRL_Reference and DRV_COAX_CTRL_AttitudeInput are now
#     canonical FLU (+X forward, +Y left, +Z up), matching the attitude/rate
#     fields that were already FLU since seam 0/1,
#   * DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN / _PITCH_SIGN and
#     DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN / _PITCH_SIGN are deleted, not
#     re-valued: once both sides of the struct are genuinely FLU, the
#     standard ZYX Euler rotation matrix needs no per-axis sign compensation,
#     because FLU's own roll/pitch/yaw are defined as right-hand rotations
#     about FLU's own axes (drv_frame_contract.h).
#
# The +Y question is pinned by the 2026-08-30 left_y recording: seam2 remains
# right-positive, so App negates it at the seam2->seam3 boundary.  +Z also
# changes from down-positive to up-positive.  See section 3 of the migration
# doc and tests/test_flu_seam3_force_frame_derivation.py for the executable
# evidence that the deleted constants were load-bearing, not inert.

from __future__ import annotations

import csv
import shutil
import subprocess
from pathlib import Path
import re

import pytest


from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE


ROOT = Path(__file__).resolve().parents[1]
CTRL_SOURCE = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"
CTRL_HEADER = ROOT / "Driver" / "Inc" / "drv_coax_ctrl.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
FLIGHT_LOG_CSV = (
    ROOT / "data" / "flight_logs" / "2026-09-02" / "rm1_3_block_queue"
    / "flightlog_20260902_202552.csv"
)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_force_frame_constants_are_deleted() -> None:
    """The four sign constants must be gone, not re-valued.

    Deletion (rather than a value change) is the migration's own acceptance
    signal: it is only correct to remove them once both sides of the
    attitude/position structs are genuinely FLU and no per-axis compensation
    is needed any more.
    """
    source = read(CTRL_SOURCE)
    for name in (
        "DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN",
        "DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN",
        "DRV_COAX_CTRL_RATE_FRAME_ROLL_SIGN",
        "DRV_COAX_CTRL_RATE_FRAME_PITCH_SIGN",
    ):
        assert f"#define {name}" not in source, f"{name} should be deleted, not re-valued"

    # Cascade gains are non-negative; negative feedback is structural.
    attitude = read(ROOT / "Driver/Src/drv_attitude_control.c")
    rate = read(ROOT / "Driver/Src/drv_rate_control.c")
    assert "params->att_kp[axis] < 0.0f" in attitude
    assert "params->kp[axis] < 0.0f" in rate
    assert "output->omega_ff[axis] -" in attitude
    assert "input->omega_sp[axis] - input->omega[axis]" in rate


def test_geometry_consumes_flu_attitude_directly() -> None:
    """coax_ctrl_attitude_matrix/local_down_to_body must use unsigned FLU angles."""
    source = read(CTRL_SOURCE)
    matrix_fn = source[
        source.index("static void coax_ctrl_attitude_matrix("):
        source.index("static void coax_ctrl_attitude_error(")
    ]
    assert "coax_ctrl_rpy_matrix(attitude->roll_rad, attitude->pitch_rad" in matrix_fn
    assert "FORCE_FRAME" not in matrix_fn

    local_down_fn = source[
        source.index("static void coax_ctrl_local_down_to_body("):
        source.index("static float coax_ctrl_norm3(")
    ]
    assert "const float phi = attitude->roll_rad;" in local_down_fn
    assert "const float theta = attitude->pitch_rad;" in local_down_fn


def test_mount_outside_control_law() -> None:
    """The 90 deg mount rotation belongs to allocation, after the control law."""
    source = read(CTRL_SOURCE)
    assert "*servo_alpha_tilt_rad = -body_y_tilt_rad;" in source
    assert "*servo_beta_tilt_rad = -body_x_tilt_rad;" in source


def test_stabilizer_feed_pinned() -> None:
    """Pin the migrated feed exactly, so seam 4 cannot change it unnoticed."""
    source = read(STABILIZER)
    # FLU attitude and rates, unconverted at the call site.
    assert "frame->attitude.roll_rad = ctx->roll_control * STABILIZER_DEG_TO_RAD;" in source
    assert "frame->attitude.gyro_x_rad_s = ctx->last_msg.imu.gyro_x_dps" in source
    # Altitude channel is now passed through as up-positive (no negation).
    assert "frame->attitude.z_m = frame->relative_height_m;" in source
    assert "frame->attitude.vz_m_s = frame->range_velocity_m_s;" in source
    assert "frame->attitude.z_m = -frame->relative_height_m;" not in source
    assert "frame->attitude.vz_m_s = -frame->range_velocity_m_s;" not in source


def test_header_input_contract() -> None:
    header = read(CTRL_HEADER)
    assert "IMU axes are already rotated to body FRD before this layer." not in header
    # The header must name what actually arrives, and where polarity used to live.
    assert "canonical FLU" in header
    assert "DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN" in header
    assert "constants are therefore deleted, not re-valued" in header


# --------------------------------------------------------------------------
# Exact-match migration test (doc/req-rf6-2-controller-flu-migration.md
# section 1/6): feed the SAME real recorded scenario into the pre-migration
# driver (unmapped, legacy Z-down) and the post-migration driver (remapped
# per the table: Z negated, everything else unchanged), and require the
# servo/motor outputs to match EXACTLY -- not "within tolerance".
#
# Attitude is held level (roll=pitch=yaw=gyro=0) so the removed
# FORCE_FRAME_ROLL_SIGN/_PITCH_SIGN have no effect by construction (their
# input is always multiplied by zero): that geometry question is already
# answered by test_flu_seam3_force_frame_derivation.py.  This test isolates
# exactly the Z-channel remap -- the position/velocity control loop, the
# gravity/accel force formula, and the up/down clamp direction in
# drv_position_control.c -- against real recorded height data, per the
# no-self-authored-algorithm-input rule.

HARNESS_TEMPLATE = r"""
#include "drv_coax_ctrl.h"

#include <stdio.h>
#include <string.h>

#ifndef Z_SIGN
#define Z_SIGN 1.0f
#endif

static const float HEIGHT_M[] = {{ {heights} }};
static const float DT_S[] = {{ {dts} }};
#define SAMPLE_COUNT ((int)(sizeof(HEIGHT_M) / sizeof(HEIGHT_M[0])))

int main(void)
{{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Schedule sched;
    DRV_COAX_CTRL_Output out;
    float prev_height = HEIGHT_M[0];
    int i;

    airframe_load_reference();
    DRV_COAX_CTRL_Init();

    for (i = 0; i < SAMPLE_COUNT; ++i) {{
        float height = HEIGHT_M[i];
        float dt = DT_S[i];
        float vz = (height - prev_height) / dt;

        memset(&att, 0, sizeof(att));
        memset(&ref, 0, sizeof(ref));
        memset(&sched, 0, sizeof(sched));

        att.z_m = Z_SIGN * height;
        att.vz_m_s = Z_SIGN * vz;
        att.acceleration_valid = 1U;

        ref.z_m = Z_SIGN * 1.0f;
        ref.navigation_position_valid = 1U;
        ref.navigation_velocity_valid = 1U;
        ref.dt_sec = dt;

        sched.position_update = 1U;
        sched.velocity_update = 1U;
        sched.attitude_update = 1U;
        sched.rate_update = 1U;
        sched.integrator_enable = 1U;
        sched.position_dt_s = dt;
        sched.velocity_dt_s = dt;
        sched.attitude_dt_s = dt;
        sched.rate_dt_s = dt;

        DRV_COAX_CTRL_RunScheduled(&att, &ref, &sched, &out);
        printf("%.9f %.9f %u %u %u %u\n",
               (double)out.alpha_rad, (double)out.beta_rad,
               (unsigned)out.motor_upper_us, (unsigned)out.motor_lower_us,
               (unsigned)out.servo_alpha_us, (unsigned)out.servo_beta_us);

        prev_height = height;
    }}
    return 0;
}}
"""


def _load_real_height_samples(limit: int = 400) -> tuple[list[float], list[float]]:
    with open(FLIGHT_LOG_CSV, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))

    heights: list[float] = []
    times_us: list[float] = []
    for row in rows:
        if row.get("flow_height_valid") != "1":
            continue
        heights.append(float(row["flow_height_m"]))
        times_us.append(float(row["timestamp_us"]))
        if len(heights) >= limit:
            break

    dts = [1.0e-3]
    for prev, cur in zip(times_us, times_us[1:]):
        step = (cur - prev) * 1.0e-6
        dts.append(step if step > 1.0e-4 else 1.0e-3)
    return heights, dts


def _build_and_run_z_channel(
    tmp_path: Path, tag: str, ctrl_source_text: str, z_sign: str
) -> list[tuple[float, ...]]:
    gcc = shutil.which("gcc")
    assert gcc is not None

    heights, dts = _load_real_height_samples()
    # 夹具在 format 之外拼接：模板用 str.format，夹具里的大括号不必再转义一遍。
    harness = AIRFRAME_FIXTURE_C + HARNESS_TEMPLATE.format(
        heights=", ".join(f"{h:.9f}f" for h in heights),
        dts=", ".join(f"{d:.9f}f" for d in dts),
    )

    work = tmp_path / tag
    work.mkdir()
    ctrl = work / "drv_coax_ctrl.c"
    ctrl.write_text(ctrl_source_text, encoding="utf-8")
    stub = work / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text(
        "#ifndef BSP_PWM_H\n#define BSP_PWM_H\n"
        "#define BSP_PWM_ESC_MIN_US 1000U\n"
        "#define BSP_PWM_ESC_MAX_US 2000U\n#endif\n",
        encoding="ascii",
    )
    # 基线那版源码 include 的是 drv_airframe_model.h（2026-09-11 已删）。这里放
    # **基线自己的**那份头文件：A/B 两侧各用各的当年定义，比的才是"迁移前后行为
    # 一不一样"，而不是"我今天怎么补的旧头"。新版源码不 include 它，多放无害。
    (stub / "drv_airframe_model.h").write_bytes(
        subprocess.run(
            ["git", "show", "3a3fa6c8:Driver/Inc/drv_airframe_model.h"],
            cwd=ROOT, check=True, capture_output=True,
        ).stdout
    )
    harness_c = work / "harness.c"
    harness_c.write_text(harness, encoding="ascii")
    executable = work / "harness.exe"

    subprocess.run(
        [gcc, "-std=c11", "-O1", f"-DZ_SIGN={z_sign}",
         f"-I{stub}", f"-I{ROOT / 'Driver' / 'Inc'}",
         str(ctrl),
         str(AIRFRAME_SOURCE),
         str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
         str(harness_c), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)

    result = subprocess.run([str(executable)], check=True,
                            capture_output=True, text=True)
    rows = []
    for line in result.stdout.strip().splitlines():
        parts = line.split()
        rows.append((float(parts[0]), float(parts[1]),
                    int(parts[2]), int(parts[3]), int(parts[4]), int(parts[5])))
    assert len(rows) > 300
    return rows


def test_z_channel_migration_matches_old_exactly(tmp_path: Path) -> None:
    """Real recorded height, remapped per the table, must give identical output.

    Attitude is held level so the (separately proven) FORCE_FRAME removal
    cannot contribute any difference here -- this isolates the Z-channel
    remap: app_stabilizer's two negations, the gravity/accel force formula
    sign in coax_ctrl_compute_balance_solution, and the up/down clamp
    direction in drv_position_control_limit_z.
    """
    if shutil.which("gcc") is None:
        pytest.skip("host gcc is unavailable")

    old_source_result = subprocess.run(
        ["git", "show", "3a3fa6c8:Driver/Src/drv_coax_ctrl.c"],
        cwd=ROOT, check=True, capture_output=True,
    )
    old_source = old_source_result.stdout.decode("utf-8")
    assert re.search(
        r"^#define\s+DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN\s+\(-1\.0f\)\s*$",
        old_source,
        re.MULTILINE,
    ), (
        "fixed pre-migration baseline 3a3fa6c8 no longer contains the "
        "expected executable roll-sign definition"
    )
    new_source = read(CTRL_SOURCE)

    old_rows = _build_and_run_z_channel(tmp_path, "old", old_source, "-1.0f")
    new_rows = _build_and_run_z_channel(tmp_path, "new", new_source, "1.0f")

    assert len(old_rows) == len(new_rows)
    worst_angle = max(max(abs(o[0] - n[0]), abs(o[1] - n[1]))
                      for o, n in zip(old_rows, new_rows))
    assert worst_angle < 1.0e-5, (
        f"Z-channel remap is not exact: worst alpha/beta difference {worst_angle} rad"
    )
    for o, n in zip(old_rows, new_rows):
        assert o[2:] == n[2:], "motor/servo pulse widths must match exactly"
