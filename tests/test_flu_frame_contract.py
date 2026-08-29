from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "Driver" / "Inc" / "drv_frame_contract.h"
SKILL = ROOT / ".agents" / "skills" / "drone-h743-project" / "SKILL.md"
REFERENCE = (
    ROOT
    / ".agents"
    / "skills"
    / "drone-h743-project"
    / "references"
    / "flu-coordinate-contract.md"
)


def macro_uint(name: str) -> int:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+(0x[0-9A-Fa-f]+|\d+)U\s*$",
        source,
        re.MULTILINE,
    )
    assert match is not None, f"missing integer contract macro {name}"
    return int(match.group(1), 0)


def test_skill_routes_coordinate_work_to_the_normative_contract() -> None:
    skill = SKILL.read_text(encoding="utf-8")
    reference = REFERENCE.read_text(encoding="utf-8")

    assert "Canonical FLU Body Frame" in skill
    assert "references/flu-coordinate-contract.md" in skill
    assert "Driver/Inc/drv_frame_contract.h" in skill
    assert "sole normative, machine-readable body-frame definition" in reference
    for statement in (
        "+X`: forward",
        "+Y`: left",
        "+Z`: up",
        "Positive roll: right wing down",
        "Positive pitch: nose down",
        "Positive yaw: nose left",
    ):
        assert statement in reference


def test_runtime_migration_cannot_be_declared_complete_yet() -> None:
    reference = REFERENCE.read_text(encoding="utf-8")

    assert macro_uint("DRV_FRAME_CONTRACT_VERSION") == 1
    assert macro_uint("DRV_FRAME_CANONICAL_BODY_IS_FLU") == 1
    assert macro_uint("DRV_FRAME_BODY_IS_RIGHT_HANDED") == 1
    assert macro_uint("DRV_FRAME_CANONICAL_ANGLE_UNIT_IS_RADIAN") == 1
    assert macro_uint("DRV_FRAME_CANONICAL_RATE_UNIT_IS_RAD_PER_SECOND") == 1
    assert macro_uint("DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK") == 0x3F
    assert macro_uint("DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK") == 0
    assert "#define DRV_FRAME_RUNTIME_MIGRATION_COMPLETE" in HEADER.read_text(encoding="utf-8")
    assert "Runtime migration is incomplete" in reference
    for legacy_path in (
        "App/Src/app_sensor.c",
        "App/Src/app_stabilizer.c",
        "Core/Src/freertos.c",
        "Driver/Src/drv_attitude_fusion.c",
        "Driver/Src/drv_imu_nav.c",
        "Driver/Src/drv_coax_ctrl.c",
    ):
        assert legacy_path in reference


FLU_HARNESS = r"""
#include "drv_frame_contract.h"

#include <math.h>
#include <stdio.h>

#define CHECK(condition, code) do { if (!(condition)) { \
    printf("FAIL %d\n", (code)); return (code); } } while (0)

static int near(float lhs, float rhs)
{
    return fabsf(lhs - rhs) < 1.0e-6f;
}

int main(void)
{
    const DRV_FRAME_Vector3f x_forward = {1.0f, 0.0f, 0.0f};
    const DRV_FRAME_Vector3f y_left = {0.0f, 1.0f, 0.0f};
    const DRV_FRAME_Vector3f z_up = {0.0f, 0.0f, 1.0f};
    const DRV_FRAME_Vector3f right_wing = {0.0f, -1.0f, 0.0f};
    const DRV_FRAME_Vector3f arbitrary_flu = {1.0f, 2.0f, 3.0f};
    DRV_FRAME_Vector3f cross_xy;
    DRV_FRAME_Vector3f positive_pitch_nose_motion;
    DRV_FRAME_Vector3f positive_yaw_nose_motion;
    DRV_FRAME_Vector3f positive_roll_right_wing_motion;
    DRV_FRAME_Vector3f arbitrary_frd;
    DRV_FRAME_Vector3f round_trip;
    DRV_FRAME_Vector3f frd_x;
    DRV_FRAME_Vector3f frd_y;
    DRV_FRAME_Vector3f frd_z;
    DRV_FRAME_Vector3f cross_frd_xy;
    DRV_FRAME_Vector3f level_specific_force_flu;
    DRV_FRAME_Vector3f level_specific_force_frd;

    CHECK(DRV_FRAME_AXIS_X_FORWARD == 0, 1);
    CHECK(DRV_FRAME_AXIS_Y_LEFT == 1, 2);
    CHECK(DRV_FRAME_AXIS_Z_UP == 2, 3);
    CHECK(DRV_FRAME_POSITIVE_ROLL_IS_RIGHT_WING_DOWN == 1U, 4);
    CHECK(DRV_FRAME_POSITIVE_PITCH_IS_NOSE_DOWN == 1U, 5);
    CHECK(DRV_FRAME_POSITIVE_YAW_IS_NOSE_LEFT == 1U, 6);
    CHECK(DRV_FRAME_MIGRATION_SENSOR_TO_FLU_BIT == (1U << 0), 37);
    CHECK(DRV_FRAME_MIGRATION_ESTIMATOR_ADAPTER_BIT == (1U << 1), 38);
    CHECK(DRV_FRAME_MIGRATION_NAVIGATION_BIT == (1U << 2), 39);
    CHECK(DRV_FRAME_MIGRATION_CONTROLLER_BIT == (1U << 3), 40);
    CHECK(DRV_FRAME_MIGRATION_RC_ACTUATOR_BIT == (1U << 4), 41);
    CHECK(DRV_FRAME_MIGRATION_TELEMETRY_LOG_BIT == (1U << 5), 42);
    CHECK((DRV_FRAME_MIGRATION_SENSOR_TO_FLU_BIT |
           DRV_FRAME_MIGRATION_ESTIMATOR_ADAPTER_BIT |
           DRV_FRAME_MIGRATION_NAVIGATION_BIT |
           DRV_FRAME_MIGRATION_CONTROLLER_BIT |
           DRV_FRAME_MIGRATION_RC_ACTUATOR_BIT |
           DRV_FRAME_MIGRATION_TELEMETRY_LOG_BIT) ==
          DRV_FRAME_RUNTIME_MIGRATION_REQUIRED_MASK, 43);
    CHECK(DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK == 0U, 44);
    CHECK(DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0U, 45);

    cross_xy = DRV_FRAME_Cross(x_forward, y_left);
    CHECK(near(cross_xy.x, z_up.x), 7);
    CHECK(near(cross_xy.y, z_up.y), 8);
    CHECK(near(cross_xy.z, z_up.z), 9);

    /* d(v)/d(theta) = axis x v for a positive right-hand rotation. */
    positive_roll_right_wing_motion = DRV_FRAME_Cross(x_forward, right_wing);
    CHECK(near(positive_roll_right_wing_motion.x, 0.0f), 28);
    CHECK(near(positive_roll_right_wing_motion.y, 0.0f), 29);
    CHECK(positive_roll_right_wing_motion.z < 0.0f, 30); /* right wing down */
    positive_pitch_nose_motion = DRV_FRAME_Cross(y_left, x_forward);
    CHECK(near(positive_pitch_nose_motion.x, 0.0f), 22);
    CHECK(near(positive_pitch_nose_motion.y, 0.0f), 23);
    CHECK(positive_pitch_nose_motion.z < 0.0f, 24); /* nose moves down */
    positive_yaw_nose_motion = DRV_FRAME_Cross(z_up, x_forward);
    CHECK(near(positive_yaw_nose_motion.x, y_left.x), 25);
    CHECK(near(positive_yaw_nose_motion.y, y_left.y), 26);
    CHECK(near(positive_yaw_nose_motion.z, y_left.z), 27); /* nose moves left */

    CHECK(near(DRV_FRAME_LEVEL_SPECIFIC_FORCE_X_G, 0.0f), 10);
    CHECK(near(DRV_FRAME_LEVEL_SPECIFIC_FORCE_Y_G, 0.0f), 11);
    CHECK(near(DRV_FRAME_LEVEL_SPECIFIC_FORCE_Z_G, 1.0f), 12);
    level_specific_force_flu.x = DRV_FRAME_LEVEL_SPECIFIC_FORCE_X_G;
    level_specific_force_flu.y = DRV_FRAME_LEVEL_SPECIFIC_FORCE_Y_G;
    level_specific_force_flu.z = DRV_FRAME_LEVEL_SPECIFIC_FORCE_Z_G;
    level_specific_force_frd = DRV_FRAME_FluToFrd(level_specific_force_flu);
    CHECK(near(level_specific_force_frd.x, 0.0f), 31);
    CHECK(near(level_specific_force_frd.y, 0.0f), 32);
    CHECK(near(level_specific_force_frd.z, -1.0f), 33);

    arbitrary_frd = DRV_FRAME_FluToFrd(arbitrary_flu);
    CHECK(near(arbitrary_frd.x, 1.0f), 13);
    CHECK(near(arbitrary_frd.y, -2.0f), 14);
    CHECK(near(arbitrary_frd.z, -3.0f), 15);
    round_trip = DRV_FRAME_FrdToFlu(arbitrary_frd);
    CHECK(near(round_trip.x, arbitrary_flu.x), 16);
    CHECK(near(round_trip.y, arbitrary_flu.y), 17);
    CHECK(near(round_trip.z, arbitrary_flu.z), 18);

    /* R(ex) x R(ey) == R(ez): the FLU<->FRD adapter is a proper rotation. */
    frd_x = DRV_FRAME_FluToFrd(x_forward);
    frd_y = DRV_FRAME_FluToFrd(y_left);
    frd_z = DRV_FRAME_FluToFrd(z_up);
    CHECK(frd_x.x > 0.0f && near(frd_x.y, 0.0f) && near(frd_x.z, 0.0f), 34);
    CHECK(frd_y.y < 0.0f && near(frd_y.x, 0.0f) && near(frd_y.z, 0.0f), 35);
    CHECK(frd_z.z < 0.0f && near(frd_z.x, 0.0f) && near(frd_z.y, 0.0f), 36);
    cross_frd_xy = DRV_FRAME_Cross(frd_x, frd_y);
    CHECK(near(cross_frd_xy.x, frd_z.x), 19);
    CHECK(near(cross_frd_xy.y, frd_z.y), 20);
    CHECK(near(cross_frd_xy.z, frd_z.z), 21);

    puts("ok");
    return 0;
}
"""


def test_flu_contract_compiles_and_executes(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("FLU contract requires a host gcc or clang compiler")

    harness = tmp_path / "flu_contract_harness.c"
    executable = tmp_path / "flu_contract_harness.exe"
    harness.write_text(FLU_HARNESS, encoding="ascii")
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(harness),
            "-lm",
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
    assert result.stdout.strip() == "ok"
