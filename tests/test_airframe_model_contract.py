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


def test_measured_airframe_geometry_has_no_compile_time_copy_left() -> None:
    """机体数据的唯一来源是 Flash，代码里不许再留一份副本。

    2026-09-11 之前这些量是 drv_airframe_model.h 里的一堆 `#define`。本条测试
    当时钉的是"这些实测值要在代码里写死并保持不变"。它现在钉的是反过来那件事，
    原因在那次改动里已经暴露：同一个量在代码里存了两份而且**对不上**——四个
    部件质量加起来 754.6 g，`MASS_KG` 却写 1.3670 kg；重心按前者算、重量按后者
    算，两套并存，谁也没报错，也没有任何测试能发现，因为两边都是"写死的常量"。

    所以规矩换成：量出来的机体数据一律走 airframe.* 运行时参数，代码里不留副本。
    没有第二个来源，就不会有两个来源打架。
    """
    assert not (ROOT / "Driver/Inc/drv_airframe_model.h").exists()

    fields = read("Driver/Inc/drv_airframe_params.h")
    table = read("Driver/Src/drv_airframe_params.c")

    # 每一个原来的实测常量，现在必须是运行时参数表里的一条。
    for field in (
        "board_mass_g", "battery_mass_g", "base_mass_g", "servo_motor_mass_g",
        "board_cg_z_m", "battery_cg_z_m", "base_cg_z_m", "servo_motor_cg_z_m",
        "imu_z_m", "prop_plane_d_m", "roll_axis_to_prop_plane_m",
        "pitch_axis_to_prop_plane_m", "pitch_thrust_lever_arm_m",
        "roll_thrust_lever_arm_m", "servo1_axis_z_m", "servo2_axis_z_m",
        "thrust_point_z_m", "tether_attach_z_m", "tether_rope_m",
        "ixx_kgm2", "iyy_kgm2", "izz_kgm2", "lower_rotor_spin_sense",
        "gravity_m_s2", "max_total_thrust_g", "servo_deg_per_us",
    ):
        assert f"float {field};" in fields, field
        assert f"AIRFRAME_ENTRY({field})" in table, field

    for derived in (
        "mass_kg", "cg_z_m", "weight_n", "thrust_point_to_cg_z_m",
        "tether_attach_to_cg_m", "tether_rod_to_cg_m", "max_total_force_n",
        "hover_thrust_percent", "servo_us_per_deg",
    ):
        assert f"float {derived};" in fields, derived
        assert f"AIRFRAME_DERIVED({derived})" in table, derived

    # 出厂没有机体数据 = 不许解锁。这是整套设计的落脚点，失效朝安全方向倒。
    assert "static DRV_Airframe_Params airframe_params;" in table
    assert "= 1.3670f" not in table and "= 0.051f" not in table, (
        "运行时模块里不许出现出厂默认值"
    )
    stabilizer = read("App/Src/app_stabilizer.c")
    assert "if (DRV_Airframe_IsValid() == 0U) {" in stabilizer
    assert "APP_LED_ARM_BLOCK_AIRFRAME" in stabilizer


def test_servo_actuator_identification_stays_a_frozen_evidence_record() -> None:
    """舵机 FOPDT 辨识结果留在代码里，而且必须连拟合质量一起留。

    它和上面那些实测尺寸不同：这是一次台架辨识的产物，系数离开 R²/RMSE/样本数
    就没法判断可不可信。做成可在线改写的参数等于给一份历史实验结果开写入口，
    所以它不进 airframe.*，而是冻在头文件里。飞控固件目前一处都不用它，
    只有 tools/sim_xz 的被控对象在用。
    """
    header = read("Driver/Inc/drv_servo_actuator_model.h")

    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_GAIN                 0.781794f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_DELAY_S              0.016231f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_TAU_INCREASE_S       0.071236f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_TAU_DECREASE_S       0.306416f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_NEUTRAL_FEEDBACK_US        1457.177648f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_R2               0.963346f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_RMSE_US          17.696884f" in header
    assert "#define DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_FIT_SAMPLE_COUNT    835U" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_GAIN                  0.769332f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_DELAY_S               0.041320f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_INCREASE_S        0.076881f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_TAU_DECREASE_S        0.057051f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_NEUTRAL_FEEDBACK_US         1509.527201f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_R2                0.994138f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_RMSE_US            8.459764f" in header
    assert "#define DRV_AIRFRAME_SERVO_BETA_ACTUATOR_FIT_SAMPLE_COUNT    1253U" in header

    # 拟合质量必须和系数一起保存：只有系数的话，没人知道它值不值得信。
    for evidence in ("FIT_R2", "FIT_RMSE_US", "FIT_SAMPLE_COUNT"):
        assert f"DRV_AIRFRAME_SERVO_ALPHA_ACTUATOR_{evidence}" in header
        assert f"DRV_AIRFRAME_SERVO_BETA_ACTUATOR_{evidence}" in header

    # 几何/质量/惯量一条都不许回到这个头文件里。
    for moved in ("MASS_KG", "CG_Z_M", "IXX_KGM2", "IZZ_KGM2",
                  "THRUST_LEVER_ARM_M", "TETHER", "LOWER_ROTOR_SPIN_SENSE"):
        assert moved not in header, moved


def test_coax_defaults_read_the_runtime_airframe_model() -> None:
    source = read("Driver/Src/drv_coax_ctrl.c")

    assert '#include "drv_airframe_params.h"' in source
    assert "drv_airframe_model.h" not in source
    assert not (ROOT / "Driver/Inc/drv_motor_model.h").exists()
    assert "drv_motor_model.h" not in source
    assert "motor_hammerstein" not in source
    assert "params->mass_kg = airframe->mass_kg;" in source
    assert (
        "params->pitch_tilt_lever_arm_m = airframe->pitch_thrust_lever_arm_m;"
        in source
    )
    assert (
        "params->roll_tilt_lever_arm_m = airframe->roll_thrust_lever_arm_m;"
        in source
    )
    assert "params->yaw_inertia = airframe->izz_kgm2;" in source
    assert "params->mass_kg = 2.2f;" not in source

    # 模型可能在两次 Run 之间被上位机改写，控制律必须每拍重新取，
    # 否则会出现"模型已改、控制律还在用旧质量"这种只有重启才暴露的中间态。
    run_body = source.split("void DRV_COAX_CTRL_RunScheduled", 1)[1]
    assert "coax_ctrl_apply_fixed_model_params(&coax_ctrl_params);" in run_body


def test_flash_config_preserves_current_record_and_migrates_v15_coax_tunables() -> None:
    source = (
        read("App/Inc/app_control_config_compat.h")
        + read("App/Src/app_control_config_compat.c")
        + read("App/Inc/app_control_config_store.h")
        + read("App/Src/app_control_config_store.c")
    )

    # V17 = 加入遥控映射；V16/V15 都必须还能读回来，否则升级会连舵机/PID 一起丢。
    # v20 在记录尾部追加了机体模型块（机体数据的唯一来源改为 Flash）。
    assert "#define APP_CONTROL_CFG_VERSION     20U" in source
    assert "#define APP_CONTROL_CFG_VERSION_V19 19U" in source
    assert "#define APP_CONTROL_CFG_VERSION_V18 18U" in source
    assert "#define APP_CONTROL_CFG_VERSION_V16 16U" in source
    assert "#define APP_CONTROL_CFG_VERSION_V15 15U" in source
    assert "APP_ControlFlashRecordV16" in source
    assert "APP_ControlFlashRecordV15" in source
    assert "app_control_migrate_coax_params" not in source
    assert "case APP_CONTROL_CFG_VERSION:" in source
    assert "case APP_CONTROL_CFG_VERSION_V16:" in source
    assert "case APP_CONTROL_CFG_VERSION_V15:" in source
    assert "APP_CONTROL_RECORD_TYPE(APP_ControlFlashRecord," in source
    assert "float vel_z_ki;" in source
    assert "current->vel_z_ki" not in source
    assert "APP_ControlConfigStore_CaptureTunables(&record.coax_tunables);" in source
    assert "config_apply_tunables(&record.coax_tunables);" in source
    assert "APP_ControlConfigCompat_V15ToCurrent" in source
    assert "DRV_COAX_CTRL_GetDefaultParams(&params);" in source
    assert "DRV_COAX_CTRL_SetParams(&params);" in source
    assert "DRV_COAX_CTRL_Params coax_params;" not in source
    assert "record.coax_params" not in source
    tunable_struct = source[source.index("typedef struct {\n    float pos_x_kp;"):source.index("} APP_ControlCoaxTunableParams;")]
    apply_tunables = source[source.index("static void config_apply_tunables"):source.index("static uint8_t config_read_current")]
    for fixed_model_field in (
        "mass_kg",
        "gravity_m_s2",
        "pitch_tilt_lever_arm_m",
        "roll_tilt_lever_arm_m",
        "yaw_inertia",
        "motor_single_max_thrust_n",
        "yaw_torque_upper_m_per_n",
        "yaw_torque_lower_m_per_n",
    ):
        assert fixed_model_field not in tunable_struct
        assert fixed_model_field not in apply_tunables


def test_v17_config_compatibility_is_extracted_from_the_oversize_entrypoint() -> None:
    control = read("App/Src/app_control.c")
    header = read("App/Inc/app_control_config_compat.h")
    compat = read("App/Src/app_control_config_compat.c")
    cmake = read("CMakeLists.txt")

    assert '#include "app_control_config_store.h"' in control
    assert '#include "app_control_config_compat.h"' in read("App/Src/app_control_config_store.c")
    assert "APP_ControlCoaxTunableParamsV17" in header
    assert "APP_ControlConfigCompat_V17ToCurrent" in compat
    for dead_gain in (
        "vel_loop_x_kp", "vel_loop_x_ki", "vel_loop_x_kd",
        "vel_loop_y_kp", "vel_loop_y_ki", "vel_loop_y_kd",
    ):
        assert dead_gain in header
        assert f"current->{dead_gain}" not in compat
    assert "App/Src/app_control_config_compat.c" in cmake


def test_airframe_query_is_text_control_payload() -> None:
    app_control = read("App/Src/app_control.c")
    # 报文体 2026-09-11 搬进 app_cmd_airframe.c（app_control.c 只减不增）；
    # 入口与报文键名不变，上位机的证据存档因此仍然对得上。
    app_cmd_airframe = read("App/Src/app_cmd_airframe.c")
    app_aiwb2 = read("App/Src/app_aiwb2.c")
    proto = read("App/Inc/app_proto.h")

    assert 'strcmp(tokens[0], "AIRFRAME?") == 0' in app_control
    assert "app_control_report_airframe_record()" in app_control
    assert "APP_PROTO_MSG_AIRFRAME_RECORD" in app_cmd_airframe
    assert "AIRFRAME mass_kg=%s cg_z_m=%s imu_z_m=%s" in app_cmd_airframe
    assert 'strcmp(line, "AIRFRAME?") == 0' in app_aiwb2
    assert "#define APP_PROTO_REQ_AIRFRAME       0x101CU" in proto
    assert "#define APP_PROTO_REQ_IDENT          0x101DU" in proto


def test_gui_airframe_parser_and_ident_meta_payload(tmp_path: Path) -> None:
    panel = load_panel_module()
    line = (
        "AIRFRAME mass_kg=1.367000 cg_z_m=-0.094600 imu_z_m=0.000000 "
        "tether_attach_z_m=0.156300 tether_attach_to_cg_m=0.250900 "
        "rope_m=0.640000 rod_to_cg_m=0.890900 servo_deg_per_us=0.090000 "
        "servo_us_per_deg=11.111111 thrust_scope=dual_motor_total "
        "max_total_force_n=15.644959 hover_thrust_pct=85.716236"
    )
    record = panel.airframe_record_from_line(line)

    assert record is not None
    assert record["mass_kg"] == 1.367
    assert record["thrust_scope"] == "dual_motor_total"
    assert record["hover_thrust_pct"] == 85.716236

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
        ident_alpha_center_var = Var(1500)
        ident_beta_center_var = Var(1500)

    dummy = Dummy()
    dummy._ident_meta_payload = panel.DronePanel._ident_meta_payload.__get__(dummy, Dummy)
    panel.DronePanel._ident_write_meta(dummy)
    meta = json.loads((tmp_path / "ident_20260526_120000_meta.json").read_text(encoding="utf-8"))

    assert meta["command"] == "IDENT STEP roll pulse_us=20 duration_ms=3000"
    assert meta["center"] == {"alpha_us": 1500, "beta_us": 1500}
    assert meta["airframe"]["max_total_force_n"] == 15.644959
