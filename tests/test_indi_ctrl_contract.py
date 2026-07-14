from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_indi_is_a_portable_driver_and_is_built() -> None:
    cmake = read("CMakeLists.txt")
    header = read("Driver/Inc/drv_indi_ctrl.h")
    source = read("Driver/Src/drv_indi_ctrl.c")

    assert "Driver/Src/drv_indi_ctrl.c" in cmake
    assert "DRV_INDI_Config" in header
    assert "DRV_INDI_State" in header
    assert "DRV_INDI_Step" in header
    assert '#include "drv_airframe_model.h"' in source
    assert "HAL_" not in source
    assert "osDelay" not in source
    assert "config->enable = 1.0f;" in source
    assert "config->roll_inertia_kg_m2 = DRV_AIRFRAME_IXX_KGM2;" in source
    assert "config->pitch_inertia_kg_m2 = DRV_AIRFRAME_IYY_KGM2;" in source


def test_indi_uses_measured_angular_acceleration_and_control_effectiveness() -> None:
    source = read("Driver/Src/drv_indi_ctrl.c")

    assert "input->gyro_x_rad_s - state->previous_rate_rad_s[0]" in source
    assert "input->gyro_y_rad_s - state->previous_rate_rad_s[1]" in source
    assert "force_n * lever_arm_m / roll_inertia" in source
    assert "force_n * lever_arm_m / pitch_inertia" in source
    assert "roll_virtual_accel - state->angular_accel_rad_s2[0]" in source
    assert "pitch_virtual_accel - state->angular_accel_rad_s2[1]" in source
    assert "increment_limit" in source
    assert "correction_limit" in source
    assert "indi_config_finite" in source
    assert "indi_input_finite" in source
    assert "DRV_INDI_Reset(state);" in source


def test_coax_wrapper_applies_indi_over_acceleration_feedforward() -> None:
    header = read("Driver/Inc/drv_coax_ctrl.h")
    wrapper = read("Driver/Src/drv_coax_ctrl.c")
    freertos = read("Core/Src/freertos.c")

    assert "float roll_rad;" in header
    assert "float pitch_rad;" in header
    assert "float dt_sec;" in header
    assert "float indi_enable;" in header
    assert "float indi_roll_inertia_kg_m2;" in header
    assert "float indi_pitch_inertia_kg_m2;" in header
    assert "DRV_COAX_CTRL_INDIDebug" in header
    assert "DRV_COAX_CTRL_GetLastINDIDebug" in header
    assert "DRV_INDI_Step(&coax_ctrl_indi_state" in wrapper
    assert "params->indi_correction_limit_rad > params->tilt_limit_rad" in wrapper
    assert "params->indi_increment_limit_rad > params->indi_correction_limit_rad" in wrapper
    assert "if (coax_ctrl_params.indi_enable >= 0.5f)" in wrapper
    assert "pitch_rate_d_rad = 0.0f;" in wrapper
    assert "roll_rate_d_rad = 0.0f;" in wrapper
    assert "reference.roll_rad = 0.0f;" in freertos
    assert "reference.pitch_rad = 0.0f;" in freertos
    assert "attitude.dt_sec = ctrl_dt_sec;" in freertos
    assert "DRV_COAX_CTRL_ResetRuntime();" in freertos
    assert "params->indi_enable = indi_defaults.enable;" in wrapper
    assert "params->indi_roll_effectiveness_sign = indi_defaults.roll_effectiveness_sign;" in wrapper
    assert "params->indi_pitch_effectiveness_sign = indi_defaults.pitch_effectiveness_sign;" in wrapper


def test_flash_v8_migrates_v7_and_defaults_new_indi_fields() -> None:
    control = read("App/Src/app_control.c")

    assert "#define APP_CONTROL_CFG_VERSION     8U" in control
    assert "APP_ControlCoaxParamsV7" in control
    assert "APP_ControlFlashRecordV8" in control
    assert "offsetof(DRV_COAX_CTRL_Params, indi_enable)" in control
    assert "DRV_COAX_CTRL_GetDefaultParams(params);" in control
    assert "memcpy(params, legacy, sizeof(*legacy));" in control
    assert "(record.version == 7U)" in control


def test_indi_runtime_diagnostics_are_available_over_control_protocol() -> None:
    control = read("App/Src/app_control.c")

    assert "static void app_control_report_indi(void)" in control
    assert "DRV_COAX_CTRL_GetLastINDIDebug(&debug);" in control
    assert 'strcmp(tokens[0], "INDI?") == 0' in control
    assert '"INDI active=%u roll_accel=%s pitch_accel=%s "' in control
