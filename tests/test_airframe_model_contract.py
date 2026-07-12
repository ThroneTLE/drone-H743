from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def load_panel_module():
    path = ROOT / "tools" / "drone_tcp_panel.py"
    spec = importlib.util.spec_from_file_location("drone_tcp_panel_airframe", path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_airframe_constants_capture_measured_tether_geometry() -> None:
    header = read("Driver/Inc/drv_airframe_model.h")

    assert "#define DRV_AIRFRAME_BOARD_MASS_G                 75.0f" in header
    assert "#define DRV_AIRFRAME_BATTERY_MASS_G              232.0f" in header
    assert "#define DRV_AIRFRAME_BASE_MASS_G                  99.0f" in header
    assert "#define DRV_AIRFRAME_SERVO_MOTOR_MASS_G          348.6f" in header
    assert "#define DRV_AIRFRAME_MASS_KG                       0.7546f" in header
    assert "#define DRV_AIRFRAME_CG_Z_M                       -0.0946f" in header
    assert "#define DRV_AIRFRAME_TETHER_ATTACH_Z_M             0.1563f" in header
    assert "#define DRV_AIRFRAME_TETHER_ATTACH_TO_CG_M         0.2509f" in header
    assert "#define DRV_AIRFRAME_TETHER_ROPE_M                 0.6400f" in header
    assert "#define DRV_AIRFRAME_TETHER_ROD_TO_CG_M            0.8909f" in header
    assert "#define DRV_AIRFRAME_THRUST_TABLE_SCOPE            \"dual_motor_total\"" in header
    assert "#define DRV_AIRFRAME_SERVO_DEG_PER_US              0.090f" in header
    assert "#define DRV_AIRFRAME_SERVO_US_PER_DEG             11.111111f" in header
    assert "#define DRV_AIRFRAME_MAX_TOTAL_FORCE_N            13.375052f" in header
    assert "#define DRV_AIRFRAME_HOVER_THRUST_PERCENT         56.079367f" in header


def test_coax_defaults_use_airframe_model_not_old_placeholder_mass() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert '#include "drv_airframe_model.h"' in source
    assert "params->mass_kg = DRV_AIRFRAME_MASS_KG;" in source
    assert "params->min_total_force_n = DRV_AIRFRAME_WEIGHT_N;" in source
    assert "params->max_total_force_n = DRV_AIRFRAME_MAX_TOTAL_FORCE_N;" in source
    assert "params->tilt_lever_arm_m = 0.18f;" in source
    assert "params->mass_kg = 2.2f;" not in source
    assert "params->max_total_force_n = 38.8476f;" not in source


def test_flash_v3_migration_forces_new_physical_model() -> None:
    source = read("App/Src/app_control.c")

    assert "#define APP_CONTROL_CFG_VERSION     8U" in source
    assert "typedef APP_ControlFlashRecordV3 APP_ControlFlashRecordV4;" in source
    assert "} APP_ControlFlashRecordV5;" in source
    assert "typedef APP_ControlFlashRecordV5 APP_ControlFlashRecordV6;" in source
    assert "typedef APP_ControlFlashRecordV5 APP_ControlFlashRecordV7;" in source
    assert "} APP_ControlFlashRecordV8;" in source
    assert "offsetof(DRV_COAX_CTRL_Params, indi_enable)" in source
    assert "app_control_migrate_coax_params_v7" in source
    assert "(record.version == 7U)" in source
    assert "app_control_apply_v7_safety_defaults(&migrated_params);" in source
    assert "params->tilt_limit_rad = defaults.tilt_limit_rad;" in source
    assert "(record.version == 3U) || (record.version == 4U)" in source
    assert "app_control_migrate_coax_params_v4(&legacy->coax_params, &migrated_params);" in source
    assert "app_control_apply_new_coax_param_defaults(params);" in source
    assert "params->mass_kg = DRV_AIRFRAME_MASS_KG;" in source
    assert "params->min_total_force_n = DRV_AIRFRAME_WEIGHT_N;" in source
    assert "params->max_total_force_n = DRV_AIRFRAME_MAX_TOTAL_FORCE_N;" in source


def test_airframe_query_is_text_control_payload() -> None:
    app_control = read("App/Src/app_control.c")
    app_aiwb2 = read("App/Src/app_aiwb2.c")
    proto = read("App/Inc/app_proto.h")

    assert 'strcmp(tokens[0], "AIRFRAME?") == 0' in app_control
    assert "APP_PROTO_MSG_AIRFRAME_RECORD" in app_control
    assert 'strcmp(line, "AIRFRAME?") == 0' in app_aiwb2
    assert "#define APP_PROTO_REQ_AIRFRAME       0x101CU" in proto
    assert "#define APP_PROTO_REQ_IDENT          0x101DU" in proto


def test_gui_airframe_parser_and_ident_meta_payload(tmp_path: Path) -> None:
    panel = load_panel_module()
    line = (
        "AIRFRAME mass_kg=0.754600 cg_z_m=-0.094600 imu_z_m=0.000000 "
        "tether_attach_z_m=0.156300 tether_attach_to_cg_m=0.250900 "
        "rope_m=0.640000 rod_to_cg_m=0.890900 servo_deg_per_us=0.090000 "
        "servo_us_per_deg=11.111111 thrust_scope=dual_motor_total "
        "max_total_force_n=13.375052 hover_thrust_pct=56.079367"
    )
    record = panel.airframe_record_from_line(line)

    assert record is not None
    assert record["mass_kg"] == 0.7546
    assert record["thrust_scope"] == "dual_motor_total"
    assert record["hover_thrust_pct"] == 56.079367

    class Dummy:
        ident_current_command = "IDENT STEP roll pulse_us=20 duration_ms=3000"
        airframe_info = record
        ident_current_path = tmp_path / "ident_20260526_120000.csv"

        class Var:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        ident_axis_var = Var("roll")
        ident_mode_var = Var("STEP")
        ident_pulse_var = Var(20)
        ident_duration_var = Var(3000)
        ident_hold_var = Var(800)
        ident_repeat_var = Var(2)
        ident_bit_var = Var(250)
        ident_seed_var = Var(1)
        ident_alpha_center_var = Var(1441)
        ident_beta_center_var = Var(1877)

    dummy = Dummy()
    dummy._ident_meta_payload = panel.DronePanel._ident_meta_payload.__get__(dummy, Dummy)
    panel.DronePanel._ident_write_meta(dummy)
    meta = json.loads((tmp_path / "ident_20260526_120000_meta.json").read_text(encoding="utf-8"))

    assert meta["command"] == "IDENT STEP roll pulse_us=20 duration_ms=3000"
    assert meta["center"] == {"alpha_us": 1441, "beta_us": 1877}
    assert meta["airframe"]["max_total_force_n"] == 13.375052
