from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_quality_adaptive_flow_ekf_is_enabled_in_stabilizer() -> None:
    cmake = read("CMakeLists.txt")
    freertos = read("Core/Src/freertos.c")
    control = read("App/Src/app_control.c")

    assert "Driver/Src/drv_nav_ekf.c" in cmake
    assert "App/Src/app_nav_estimator.c" in cmake
    assert '#include "drv_nav_ekf.h"' in freertos
    assert '#include "app_nav_estimator.h"' in freertos
    assert '#include "app_nav_estimator.h"' in control
    assert "#define STABILIZER_NAV_USE_FLOW_EKF 1U" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_NOISE_MIN_M_S 0.04f" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_NOISE_MAX_M_S 0.45f" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_QUALITY_HIGH 180U" in freertos
    assert "stabilizer_flow_noise_from_quality(uint8_t quality)" in freertos
    assert "weak * weak" in freertos
    assert "DRV_NAV_EKF_State ekf;" in freertos
    assert "DRV_NAV_EKF_Diagnostics diagnostics;" in freertos
    assert "DRV_NAV_EKF_DefaultConfig(&config);" in freertos
    assert "DRV_NAV_EKF_Reset(&state->ekf, &config);" in freertos
    assert "DRV_NAV_EKF_Predict(&state->ekf, acc_x_m_s2, acc_y_m_s2, dt_sec);" in freertos
    assert "flow_noise_m_s = stabilizer_flow_noise_from_quality(flow_quality);" in freertos
    assert "DRV_NAV_EKF_FuseFlow(&state->ekf," in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);" in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);" not in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_NONE);" in freertos
    assert "APP_NavEstimator_PublishVelocityEKF(&state->diagnostics);" in freertos
    assert "STATUS ekf init=%u pred=%lu upd=%lu rej=%lu skip=%lu nis_milli=%ld" in control
    assert "innov_mm_s=%ld,%ld noise_mm_s=%ld vx_mm_s=%ld vy_mm_s=%ld" in control


def test_nav_ekf_exposes_industry_consistency_metrics() -> None:
    header = read("Driver/Inc/drv_nav_ekf.h")
    source = read("Driver/Src/drv_nav_ekf.c")

    assert "flow_gate_nis" in header
    assert "last_innovation_m_s" in header
    assert "last_nis" in header
    assert "flow_update_count" in header
    assert "flow_reject_count" in header
    assert "flow_skip_count" in header
    assert "covariance_diag" in header
    assert "config->flow_gate_nis = 9.21f;" in source
    assert "config->max_velocity_m_s = 2.50f;" in source
    assert "config->predict_leak_hz = 1.50f;" in source
    assert "nis > state->config.flow_gate_nis" in source
    assert "state->flow_reject_count++;" in source
    assert "state->flow_skip_count++;" in source
    assert "last_flow_update_ms" in header
    assert "DRV_NAV_EKF_GetDiagnostics" in source


def test_velocity_control_uses_flow_ekf_with_limited_compensated_imu_bridge() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_NAV_EKF_CONTROL_TIMEOUT_MS 150U" in freertos
    assert "#define STABILIZER_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS 80U" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_SOFT_HOLD_MS 150U" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_STALE_RESET_MS 250U" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_LOST_DECAY_HZ 1.0f" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_STALE_DECAY_HZ 12.0f" in freertos
    assert "#define STABILIZER_NAV_EKF_ZERO_FLOW_SPEED_M_S 0.035f" in freertos
    assert "#define STABILIZER_NAV_EKF_ZERO_ACCEL_M_S2 0.30f" in freertos
    assert "#define STABILIZER_NAV_EKF_ZERO_FLOW_COUNT 8U" in freertos
    assert "config.flow_gate_nis = 0.0f;" in freertos
    assert "#define STABILIZER_NAV_EKF_CONTROL_MAX_SPEED_M_S 1.50f" in freertos
    assert "#define STABILIZER_IMU_LEVER_ARM_Z_M (-0.10f)" in freertos
    assert "#define STABILIZER_IMU_RATE_WEIGHT_SOFT_RAD_S 2.0f" in freertos
    assert "#define STABILIZER_IMU_ALPHA_WEIGHT_SOFT_RAD_S2 20.0f" in freertos
    assert "stabilizer_compensated_imu_accel_nav_xy(" in freertos
    assert "stabilizer_cross3(alpha, r_imu_m, alpha_cross_r);" in freertos
    assert "stabilizer_cross3(omega, omega_cross_r, omega_cross_omega_cross_r);" in freertos
    assert "f_cg_body_m_s2[axis] = f_body_m_s2[axis] -" in freertos
    assert "rate_norm / STABILIZER_IMU_RATE_WEIGHT_SOFT_RAD_S" in freertos
    assert "alpha_norm / STABILIZER_IMU_ALPHA_WEIGHT_SOFT_RAD_S2" in freertos
    assert "last_gyro_ready = 0U;" in freertos
    assert "#define STABILIZER_FLOW_ROT_COMP_ENABLE 1U" in freertos
    assert "#define STABILIZER_FLOW_SENSOR_OFFSET_X_M 0.20f" in freertos
    assert "#define STABILIZER_FLOW_SENSOR_OFFSET_Z_M 0.22f" in freertos
    assert "#define STABILIZER_FLOW_ONLY_MAX_ACCEL_M_S2 30.0f" in freertos
    assert "#define STABILIZER_FLOW_ONLY_MIN_STEP_M_S 0.30f" in freertos
    assert "uint8_t zero_flow_count;" in freertos
    assert "stabilizer_velocity_estimator_zero_horizontal(state);" in freertos
    assert "flow_age_ms > STABILIZER_NAV_EKF_FLOW_STALE_RESET_MS" in freertos
    assert "flow_age_ms > STABILIZER_NAV_EKF_FLOW_SOFT_HOLD_MS" in freertos
    assert "decay_hz = STABILIZER_NAV_EKF_FLOW_STALE_DECAY_HZ;" in freertos
    assert "STABILIZER_NAV_EKF_ZERO_ACCEL_M_S2 *" in freertos
    assert "state->zero_flow_count >= STABILIZER_NAV_EKF_ZERO_FLOW_COUNT" in freertos
    assert "stabilizer_velocity_estimator_control_ok(&vel_estimator, now)" not in freertos
    assert "velocity_loop_enabled = (vel_loop_enable >= 0.5f) ? 1U : 0U;" in freertos
    assert "reference.horizontal_velocity_valid = velocity_loop_enabled;" in freertos
    assert "attitude.vx_m_s = velocity_control_x_m_s;" in freertos
    assert "attitude.vy_m_s = velocity_control_y_m_s;" in freertos
    assert "STABILIZER_VELOCITY_MEAS_Y_SIGN * velocity_state_y_m_s" in freertos
    assert "stabilizer_compensate_flow_rotation(" in freertos
    assert "-STABILIZER_FLOW_ROT_COMP_GAIN * height_m * gyro_y_rad_s;" in freertos
    assert "body_vx_m_s += debug->optical_rot_comp_m_s[0] +" in freertos
    assert "STABILIZER_FLOW_ROT_COMP_GAIN * height_m * gyro_x_rad_s;" in freertos
    assert "body_vy_m_s += debug->optical_rot_comp_m_s[1] +" in freertos
    assert "gyro_y_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Z_M" in freertos
    assert "gyro_x_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Z_M" in freertos
    assert "gyro_z_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_X_M" in freertos
    assert "debug->offset_rot_comp_m_s[1]" in freertos
    assert "APP_OpticalFlow_GetHeightSample(&flow_height_m," in freertos
    assert "APP_OpticalFlow_GetStatus(&flow_status);" in freertos
    assert "flow_status.flow_quality," in freertos
    assert "stabilizer_flow_velocity_plausible(state," in freertos
    assert "state->ekf.last_flow_sample_ms = flow_sample_ms;" in freertos
    assert "(void)imu_vx_m_s;" in freertos
    assert "(void)imu_vy_m_s;" in freertos
    assert "reference.ax_m_s2 = 0.0f;" in freertos
    assert "reference.ay_m_s2 = 0.0f;" in freertos
    assert "predict_leak_hz" in read("Driver/Inc/drv_nav_ekf.h")
    assert "state->vel_m_s[0] *= leak;" in read("Driver/Src/drv_nav_ekf.c")
    assert "state->vel_m_s[1] *= leak;" in read("Driver/Src/drv_nav_ekf.c")


def test_nav_ekf_state_model_contains_velocity_and_accel_bias() -> None:
    header = read("Driver/Inc/drv_nav_ekf.h")
    source = read("Driver/Src/drv_nav_ekf.c")

    assert "float vel_m_s[2];" in header
    assert "float accel_bias_m_s2[2];" in header
    assert "state->vel_m_s[0] += (acc_x_m_s2 - state->accel_bias_m_s2[0]) * dt;" in source
    assert "state->vel_m_s[1] += (acc_y_m_s2 - state->accel_bias_m_s2[1]) * dt;" in source
    assert "state->accel_bias_m_s2[0] +=" in source
    assert "state->accel_bias_m_s2[1] +=" in source
