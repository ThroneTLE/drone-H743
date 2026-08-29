from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    end = source.find("\nstatic ", start + len(signature))
    if end < 0:
        end = source.find("\nvoid ", start + len(signature))
    return source[start:] if end < 0 else source[start:end]


def test_frame_candidate_is_applied_once_before_any_estimator_consumer() -> None:
    source = read("App/Src/app_stabilizer.c")
    body = function_body(source, "static void stabilizer_imu_step(")

    apply_index = body.index("APP_Sensor_ApplyFrameCorrection(&msg->imu)")
    first_rate_read = body.index("msg->imu.gyro_x_dps")
    first_accel_read = body.index("msg->imu.accel_x_g")

    assert apply_index < first_rate_read
    assert apply_index < first_accel_read
    assert "msg->imu_frame_orientation_code = orientation_code;" in body


def test_frame_change_resets_state_and_selects_nwu_for_canonical_flu() -> None:
    source = read("App/Src/app_stabilizer.c")
    reset = function_body(source, "static void stabilizer_reset_for_imu_frame(")

    assert "DRV_AttitudeFusion_InitForConvention(" in reset
    assert "DRV_ATTITUDE_FUSION_CONVENTION_NWU" in reset
    assert "DRV_ATTITUDE_FUSION_CONVENTION_NED" in reset
    assert "DRV_IMU_NAV_Reset(&ctx->nav_state);" in reset
    assert "stabilizer_velocity_estimator_reset(&ctx->vel_estimator);" in reset
    assert "ctx->attitude_zero_ready = 0U;" in reset
    assert "ctx->last_gyro_ready = 0U;" in reset


def test_canonical_flu_fusion_branch_uses_one_unmodified_vector_contract() -> None:
    source = read("App/Src/app_stabilizer.c")
    body = function_body(source, "static void stabilizer_imu_step(")
    canonical = body[body.index("if (flu_active != 0U) {") : body.index("} else {", body.index("if (flu_active != 0U) {"))]

    for axis in range(3):
        assert (
            f"fusion_input.gyroscope_dps[{axis}] = msg->imu.gyro_"
            in canonical
        )
        assert (
            f"fusion_input.accelerometer_g[{axis}] = msg->imu.accel_"
            in canonical
        )
    assert "= -msg->imu" not in canonical

    assert "atan2f(msg->imu.accel_y_g, msg->imu.accel_z_g)" in body
    assert "atan2f(-msg->imu.accel_x_g" in body


def test_partial_runtime_migration_is_hard_locked_against_arming() -> None:
    header = read("App/Inc/app_stabilizer.h")
    source = read("App/Src/app_stabilizer.c")
    arm = function_body(source, "static uint8_t stabilizer_rc_update_armed(")

    assert "uint8_t APP_Stabilizer_IsImuFrameArmLocked(void);" in header
    assert "APP_Sensor_IsFluOrientationActive()" in source
    assert "DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0U" in source
    assert "APP_Stabilizer_IsImuFrameArmLocked()" in arm
    assert arm.index("APP_Stabilizer_IsImuFrameArmLocked()") < arm.index(
        "if (rc_link_ok == 0U)"
    )


def test_snapshot_carries_the_frame_code_from_the_same_processed_sample() -> None:
    messages = read("App/Inc/app_messages.h")
    header = read("App/Inc/app_stabilizer.h")
    source = read("App/Src/app_stabilizer.c")

    assert "uint8_t imu_frame_orientation_code;" in messages
    assert "uint8_t imu_frame_orientation_code;" in header
    assert (
        "next.imu_frame_orientation_code = msg->imu_frame_orientation_code;"
        in source
    )


def test_fusion_driver_exposes_explicit_ned_and_nwu_initialisation() -> None:
    header = read("Driver/Inc/drv_attitude_fusion.h")
    source = read("Driver/Src/drv_attitude_fusion.c")

    assert "DRV_ATTITUDE_FUSION_CONVENTION_NED" in header
    assert "DRV_ATTITUDE_FUSION_CONVENTION_NWU" in header
    assert "DRV_AttitudeFusion_InitForConvention(" in header
    assert "FusionConventionNwu" in source
    assert "FusionConventionNed" in source
