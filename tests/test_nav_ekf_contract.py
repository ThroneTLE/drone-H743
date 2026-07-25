from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_nav_ekf_is_built_and_wired_into_stabilizer() -> None:
    cmake = read("CMakeLists.txt")
    freertos = read("Core/Src/freertos.c")
    control = read("App/Src/app_control.c")

    assert "Driver/Src/drv_nav_ekf.c" in cmake
    assert "App/Src/app_nav_estimator.c" in cmake
    assert '#include "drv_nav_ekf.h"' in freertos
    assert '#include "app_nav_estimator.h"' in freertos
    assert '#include "app_nav_estimator.h"' in control
    assert "#define STABILIZER_NAV_USE_FLOW_EKF 1U" in freertos
    assert "DRV_NAV_EKF_State ekf;" in freertos
    assert "DRV_NAV_EKF_Diagnostics diagnostics;" in freertos
    assert "DRV_NAV_EKF_DefaultConfig(&config);" in freertos
    assert "DRV_NAV_EKF_Reset(&state->ekf, &config);" in freertos
    assert "DRV_NAV_EKF_Predict(&state->ekf, acc_x_m_s2, acc_y_m_s2, dt_sec);" in freertos
    assert "DRV_NAV_EKF_FuseFlow(&state->ekf," in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);" in freertos
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);" in freertos
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


def test_velocity_control_enable_bypasses_estimator_health_gate_for_commissioning() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "#define STABILIZER_NAV_EKF_CONTROL_TIMEOUT_MS 150U" in freertos
    assert "#define STABILIZER_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS 80U" in freertos
    assert "#define STABILIZER_NAV_EKF_FLOW_LOST_DECAY_HZ 8.0f" in freertos
    assert "#define STABILIZER_NAV_EKF_CONTROL_MAX_SPEED_M_S 1.50f" in freertos
    assert "imu_bridge_ok" in freertos
    assert "DRV_NAV_EKF_Predict(&state->ekf, acc_x_m_s2, acc_y_m_s2, dt_sec);" in freertos
    assert "state->ekf.vel_m_s[0] *= decay;" in freertos
    assert "state->ekf.vel_m_s[1] *= decay;" in freertos
    assert "stabilizer_velocity_estimator_control_ok(&vel_estimator, now)" not in freertos
    assert "velocity_loop_enabled = (vel_loop_enable >= 0.5f) ? 1U : 0U;" in freertos
    assert "reference.horizontal_velocity_valid = velocity_loop_enabled;" in freertos
    assert "attitude.vx_m_s = velocity_state_x_m_s;" in freertos
    assert "attitude.vy_m_s = velocity_state_y_m_s;" in freertos
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
