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
        "pitch_axis_to_prop_plane_m", "servo1_axis_z_m", "servo2_axis_z_m",
        "thrust_point_z_m", "tether_attach_z_m", "tether_rope_m",
        "ixx_kgm2", "iyy_kgm2", "izz_kgm2",
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

    # 2026-09-27：两个"推力力臂"退役——力臂改由 servoN_axis_z_m − cg_z_m 算出。
    # 字段留在结构体里是 Flash ABI（改名 retired_ 让漏改的旧代码编译失败），
    # 但不许再出现在参数表里，否则又多一个会和几何打架的来源。
    for retired in ("pitch_thrust_lever_arm_m", "roll_thrust_lever_arm_m"):
        assert f"float retired_{retired};" in fields, retired
        assert f"float {retired};" not in fields.replace(f"retired_{retired}", ""), retired
        assert f"AIRFRAME_ENTRY({retired})" not in table, retired
        assert f"AIRFRAME_ENTRY(retired_{retired})" not in table, retired

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
    # 2026-09-27：倾转力臂是带符号的几何量 重心 z − 舵机转轴 z，不再是输入字段。
    assert (
        "params->pitch_tilt_lever_arm_m = -DRV_Airframe_PitchTiltAxisToCgZ(airframe);"
        in source
    )
    assert (
        "params->roll_tilt_lever_arm_m = -DRV_Airframe_RollTiltAxisToCgZ(airframe);"
        in source
    )
    assert "thrust_lever_arm_m" not in source
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
    # v21 起记录里多了状态灯颜色绑定块（APP_LedConfig）；v20 的读取器
    # 仍在（config_read_v20），所以旧记录照样读得回来。
    # 2026-09-20（R-MAG-1）：v23 在记录尾部追加磁力计校准块，当前版本号随之
    # 推进到 23；v22 → v23 的覆盖见下一条测试。
    # 2026-09-28：v24 追加指令整形/出口陷波块；同日晚 v25 在该块尾部追加第二级出口陷波。
    assert "#define APP_CONTROL_CFG_VERSION     25U" in source
    assert "#define APP_CONTROL_CFG_VERSION_V24 24U" in source
    assert "#define APP_CONTROL_CFG_VERSION_V23 23U" in source
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


def test_the_v22_reader_keeps_the_airframe_block_and_defaults_the_new_mag_block() -> None:
    """v22 → v23 迁移覆盖：机体模型要保留，新的磁力计块要落回未校准。

    2026-09-20（R-MAG-1）：v23 在记录尾部追加磁力计校准块（当前版本号见上一条
    测试）。config_read_v22 读一条 v22 记录时，机体模型必须按记录里的真实
    字段应用（`&record.airframe`）——没有机体模型飞控禁止解锁，这条迁移路径
    要是漏了它，后果是一架好端端配过的飞机升级后突然飞不起来。磁力计块在
    v22 记录里不存在，必须显式落回未校准（NULL）。
    """
    source = (
        read("App/Inc/app_control_config_store.h")
        + read("App/Src/app_control_config_store.c")
    )
    reader = source[source.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v22"):]
    reader = reader[:reader.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v21")]

    assert "DRV_Airframe_SetParams(&record.airframe);" in reader
    assert "app_cmd_magcal_apply_config(NULL);" in reader


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


def test_gui_airframe_parser_reads_the_firmware_line(tmp_path: Path) -> None:
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


def test_the_identification_profile_carries_the_airframe_snapshot(tmp_path: Path) -> None:
    """辨识产出必须带着当时的机体条件。

    旧的做法是每个 CSV 旁边写一个 `*_meta.json`（`_ident_meta_payload`），
    2026-09-13 随整页迁出（R-SYSID-1）。承接它的是**辨识档案**：同一件事，
    但它同时是下一轮辨识的输入，所以不会像 sidecar 那样写完就没人再看。

    为什么非要存这份快照：辨识结论只在那组几何下成立。半年后回答"这组 PID 是在
    哪个机体上辨的"，要的就是它。
    """
    import sys

    sys.path.insert(0, str(ROOT / "tools"))
    try:
        from sysid.profile import FitResult, Gains, IdentProfile
        from sysid.rig import Rig
    finally:
        sys.path.pop(0)

    profile = IdentProfile(
        name="光杆架-2026-09-13",
        rig=Rig(imu_above_cg_m=0.0946),
        airframe={"mass_kg": 1.367, "cg_z_m": -0.0946,
                  "max_total_force_n": 15.644959},
        assumed_inertia_kg_m2=0.019,
        fit=FitResult(inertia_kg_m2=0.0201, delay_s=0.031),
        gains=Gains(rate_kp=0.5),
        firmware_id="deadbee")
    path = profile.save(tmp_path)
    again = IdentProfile.load(path)

    assert again.airframe["max_total_force_n"] == 15.644959
    assert again.rig.imu_above_cg_m == 0.0946
    assert again.firmware_id == "deadbee"
    assert again.created_utc, "没有时间戳的档案没法回答「什么时候辨的」"
    # 候选增益存在档案里，但档案本身不会让任何东西落 Flash。
    assert "SAVE" not in path.read_text(encoding="utf-8")
