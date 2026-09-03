from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_quality_adaptive_flow_ekf_is_owned_by_the_flow_nav_service() -> None:
    """R-M5-5：EKF 从稳定环搬到 Service，整定量与接线一并跟着走。

    断言全部等价保留，只是换了归属：数值、映射公式、诊断出口一个没少。
    """
    cmake = read("CMakeLists.txt")
    service_h = read("Services/Inc/svc_flow_nav.h")
    service_c = read("Services/Src/svc_flow_nav.c")
    stabilizer = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    control = read("App/Src/app_cmd_system.c")

    assert "Driver/Src/drv_nav_ekf.c" in cmake
    assert "App/Src/app_nav_estimator.c" in cmake
    assert "Services/Src/svc_flow_nav.c" in cmake
    assert '#include "drv_nav_ekf.h"' in service_h
    assert '#include "svc_flow_nav.h"' in stabilizer
    assert '#include "app_nav_estimator.h"' in stabilizer
    assert '#include "app_nav_estimator.h"' in control

    # 质量自适应量测噪声：数值与二次映射不变，只是常量前缀换成 SVC_。
    assert "#define SVC_FLOW_NAV_EKF_FLOW_NOISE_MIN_M_S   0.04f" in service_h
    assert "#define SVC_FLOW_NAV_EKF_FLOW_NOISE_MAX_M_S   0.45f" in service_h
    assert "#define SVC_FLOW_NAV_EKF_FLOW_QUALITY_HIGH    180U" in service_h
    assert "flow_nav_noise_from_quality(uint8_t quality)" in service_c
    assert "weak * weak" in service_c

    # EKF 本体调用全部落在 Service 内。
    assert "DRV_NAV_EKF_State ekf;" in service_c
    assert "DRV_NAV_EKF_Diagnostics diagnostics;" in service_c
    assert "DRV_NAV_EKF_DefaultConfig(&config);" in service_c
    assert "DRV_NAV_EKF_Reset(&flow_nav_ctx.ekf, &config);" in service_c
    assert "DRV_NAV_EKF_Predict(&flow_nav_ctx.ekf," in service_c
    assert "flow_noise_m_s = flow_nav_noise_from_quality(input->flow_quality);" in service_c
    assert "DRV_NAV_EKF_FuseFlow(&flow_nav_ctx.ekf," in service_c
    # 稳定环不许再直接碰 EKF——这是本次架构重构的硬边界。
    assert "DRV_NAV_EKF" not in stabilizer

    # 速度来源标注与诊断出口不变。
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_FLOW);" in stabilizer
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_IMU);" not in stabilizer
    assert "APP_OpticalFlow_SetVelocitySource(APP_OPTICAL_FLOW_VEL_SOURCE_NONE);" in stabilizer
    assert "APP_NavEstimator_PublishVelocityEKF();" in stabilizer
    assert "SVC_FlowNav_GetEkfDiagnostics(&nav_estimator_velocity_ekf);" in read(
        "App/Src/app_nav_estimator.c")
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
    """桥接 / 衰减 / 零速判定跟着 EKF 搬进 Service；稳定环侧的补偿与接线不变。"""
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")
    service_h = read("Services/Inc/svc_flow_nav.h")
    service_c = read("Services/Src/svc_flow_nav.c")

    # --- 归 Service 的整定量：数值逐一对应，一个没改 ---
    assert "#define SVC_FLOW_NAV_EKF_IMU_BRIDGE_TIMEOUT_MS 80U" in service_h
    assert "#define SVC_FLOW_NAV_EKF_FLOW_SOFT_HOLD_MS    150U" in service_h
    assert "#define SVC_FLOW_NAV_EKF_FLOW_STALE_RESET_MS  250U" in service_h
    assert "#define SVC_FLOW_NAV_EKF_FLOW_LOST_DECAY_HZ   1.0f" in service_h
    assert "#define SVC_FLOW_NAV_EKF_FLOW_STALE_DECAY_HZ  12.0f" in service_h
    assert "#define SVC_FLOW_NAV_EKF_ZERO_FLOW_SPEED_M_S  0.035f" in service_h
    assert "#define SVC_FLOW_NAV_EKF_ZERO_ACCEL_M_S2      0.30f" in service_h
    assert "#define SVC_FLOW_NAV_EKF_ZERO_FLOW_COUNT      8U" in service_h
    assert "#define SVC_FLOW_NAV_EKF_CONTROL_MAX_SPEED_M_S 1.50f" in service_h
    assert "#define SVC_FLOW_NAV_FLOW_ONLY_MAX_ACCEL_M_S2 30.0f" in service_h
    assert "#define SVC_FLOW_NAV_FLOW_ONLY_MIN_STEP_M_S   0.30f" in service_h

    # --- 归 Service 的判决逻辑 ---
    assert "config.flow_gate_nis = 0.0f;" in service_c
    assert "uint8_t  zero_flow_count;" in service_c
    assert "flow_nav_estimator_zero_horizontal();" in service_c
    assert "flow_age_ms > SVC_FLOW_NAV_EKF_FLOW_STALE_RESET_MS" in service_c
    assert "flow_age_ms > SVC_FLOW_NAV_EKF_FLOW_SOFT_HOLD_MS" in service_c
    assert "decay_hz = SVC_FLOW_NAV_EKF_FLOW_STALE_DECAY_HZ;" in service_c
    assert "SVC_FLOW_NAV_EKF_ZERO_ACCEL_M_S2 *" in service_c
    assert "flow_nav_ctx.zero_flow_count >=" in service_c
    assert "SVC_FLOW_NAV_EKF_ZERO_FLOW_COUNT) {" in service_c
    assert "flow_nav_control_velocity_plausible(input->flow_vx_m_s," in service_c
    assert "flow_nav_ctx.ekf.last_flow_sample_ms = input->flow_sample_ms;" in service_c

    # --- 仍归稳定环的部分：IMU 加速度补偿与光流旋转补偿，一字不动 ---
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
    assert "fuse_input.flow_quality = flow_status.flow_quality;" in freertos

    # --- 速度环接线不变 ---
    assert "velocity_loop_enabled = (vel_loop_enable >= 0.5f) ? 1U : 0U;" in freertos
    assert "reference.horizontal_velocity_valid = velocity_loop_enabled;" in freertos
    assert "attitude.vx_m_s = velocity_control_x_m_s;" in freertos
    assert "attitude.vy_m_s = velocity_control_y_m_s;" in freertos
    assert "STABILIZER_VELOCITY_MEAS_Y_SIGN * nav_vy_m_s" in freertos
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
