"""转速陷波的接线（静态）：谁拿滤过的陀螺、谁仍拿原始陀螺，分层与构建清单。

行为在 test_rpm_notch_filter.py / test_rpm_notch_policy.py / test_sysid_runtime_contract.py 里测；
这里钉的是"接在哪"——接错位置在宿主测试里是全绿的，到实机才表现成控制环吃了错的信号。
"""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    return re.sub(r"//[^\n]*", "", source)


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace:index + 1]
    raise AssertionError(signature)


# ---------------------------------------------------------------- 分层


def test_the_driver_is_pure():
    source = read("Driver/Src/drv_rpm_notch.c")
    header = read("Driver/Inc/drv_rpm_notch.h")
    for text in (source, header):
        includes = re.findall(r'#include\s+[<"]([^>"]+)[>"]', text)
        assert set(includes) <= {"drv_rpm_notch.h", "math.h", "stddef.h", "string.h", "stdint.h"}, includes
    code = strip_comments(source)
    # 文件级可变量：顶格的 static 非函数声明、或顶格的全局变量。
    for line in code.splitlines():
        if line.startswith("static ") and "(" not in line:
            raise AssertionError(f"file-scope static: {line}")


def test_the_policy_does_not_print_from_the_control_task():
    source = read("App/Src/app_rpm_notch.c")
    assert "APP_Control_QueueText(" not in source
    assert '#include "app_control.h"' not in source
    assert '#include "bsp_imu.h"' not in source, "bsp_imu.h 拖着 HAL，名义 ODR 走 bsp_imu_rate.h"
    assert '#include "bsp_imu_rate.h"' in source


def test_the_odr_getter_header_is_hal_free():
    header = read("BSP/Inc/bsp_imu_rate.h")
    assert re.findall(r'#include\s+[<"]([^>"]+)[>"]', header) == ["stdint.h"]
    assert "uint16_t BSP_IMU_GetGyroOdrHz(void);" in header
    body = function_body(read("BSP/Src/bsp_imu.c"), "uint16_t BSP_IMU_GetGyroOdrHz(void)")
    assert "DRV_BMI088_GyroBandwidthCode(imu_config.gyro_odr, imu_config.gyro_aaf_hz, &hz, NULL)" in body
    assert "DRV_BMI270_OdrCode(imu_config.gyro_odr, 1U, &hz)" in body
    assert "imu_initialized == 0U" in body


def test_boot_default_is_on_after_the_hardware_ab():
    """2026-09-28 作者定为开机即开：台架 A/B 上 107 Hz 桨振动线被压下约 −37 dB。
    关着时逐位直通等行为仍在 test_rpm_notch_policy.py 里显式配置成关来测。"""
    header = read("App/Inc/app_rpm_notch.h")
    assert re.search(r"#define APP_RPM_NOTCH_DEFAULT_ENABLE\s+1U", header)


def test_the_policy_compiles_for_a_non_bidir_build(tmp_path):
    """默认 CMake 档是 DSHOT300（没有转速）：策略层要能按那一档编过（-Werror）。"""
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required")
    for protocol in ("0", "1", "2"):
        result = subprocess.run(
            [compiler, "-fsyntax-only", "-std=c11", "-Wall", "-Wextra", "-Werror",
             f"-DBSP_ESC_PROTOCOL={protocol}", "-I", str(ROOT / "App/Inc"), "-I", str(ROOT / "Driver/Inc"),
             "-I", str(ROOT / "BSP/Inc"), "-I", str(ROOT / "Services/Inc"),
             str(ROOT / "App/Src/app_rpm_notch.c")],
            capture_output=True, text=True)
        assert result.returncode == 0, (protocol, result.stderr)


# ---------------------------------------------------------------- 稳定环接线


def test_the_notch_runs_per_imu_sample_right_after_the_rad_s_conversion():
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    body = function_body(source, "static void stabilizer_imu_step(")
    convert = body.index("gyro_rad_s[2] = msg->imu.gyro_z_dps * STABILIZER_DEG_TO_RAD;")
    apply = body.index("APP_RpmNotch_ApplySample(gyro_rad_s, ctx->gyro_ctrl_rad_s, msg->base.timestamp_us);")
    fusion = body.index("DRV_AttitudeFusion_Update(")
    alpha = body.index("alpha_rad_s2[0] = (gyro_rad_s[0] - ctx->last_gyro_rad_s[0]) / dt_sec;")
    assert convert < apply < alpha < fusion
    # 估计器仍是原始陀螺。
    assert "fusion_input.gyroscope_dps[0] = msg->imu.gyro_x_dps;" in body
    assert "ctx->last_gyro_rad_s[0] = gyro_rad_s[0];" in body


def test_only_the_control_consumers_read_the_notched_gyro():
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    readers = re.findall(r"[^\n]*ctx->gyro_ctrl_rad_s\[[^\n]*", source)
    assert len(readers) == 4, readers                # 3 × attitude feed + sysid_obs 的一行
    assert "frame->attitude.gyro_x_rad_s = ctx->gyro_ctrl_rad_s[0];" in source
    assert "frame->attitude.gyro_y_rad_s = ctx->gyro_ctrl_rad_s[1];" in source
    assert "frame->attitude.gyro_z_rad_s = ctx->gyro_ctrl_rad_s[2];" in source
    sysid = source[source.index("APP_SysIdObserve sysid_obs = {"):source.index("APP_SysId_Update(&sysid_obs);")]
    assert ".gyro_rad_s = {\n        ctx->last_msg.imu.gyro_x_dps * STABILIZER_DEG_TO_RAD," in sysid
    assert ".gyro_ctrl_rad_s = {" in sysid
    # 辨识观测、验收、飞行日志 imu 仍是原始陀螺。
    assert "observation.rate_dps[0] = ctx->last_msg.imu.gyro_x_dps;" in source
    assert "flog_snapshot.imu = ctx->last_msg.imu;" in source


def test_tick_follows_the_commit_and_reset_follows_the_frame_reset():
    source = strip_comments(read("App/Src/app_stabilizer.c"))
    assert re.search(r"stabilizer_control_commit\(ctx, &frame\);\s*APP_RpmNotch_Tick\(\);", source)
    reset = function_body(source, "static void stabilizer_reset_for_imu_frame(")
    assert "APP_RpmNotch_ResetState();" in reset
    init = function_body(source, "static void stabilizer_init(")
    assert init.index("memset(ctx, 0, sizeof(*ctx));") < init.index("APP_RpmNotch_Init();")


def test_the_sysid_closed_loop_reads_the_control_gyro_and_logs_the_raw_one():
    source = strip_comments(read("App/Src/app_sysid.c"))
    closed = function_body(source, "static uint8_t sysid_closed_loop(")
    assert "in.omega[i] = obs->gyro_ctrl_rad_s[i];" in closed and "gyro_rad_s" not in closed.replace(
        "gyro_ctrl_rad_s", "")
    measure = function_body(source, "static uint8_t sysid_sample_measurements(")
    assert "sample->gyro_rad_s[0] = obs->gyro_rad_s[0];" in measure
    assert "DRV_SysIdRig_RateResidual(&sysid.rig, obs->gyro_rad_s, &residual_rad_s);" in source
    gate = function_body(source, "static const char *sysid_gate(")
    assert "!isfinite(obs->gyro_ctrl_rad_s[2])" in gate
    assert "app_rpm_notch" not in source, "app_sysid.c 不依赖陷波模块"
    header = read("App/Inc/app_sysid.h")
    observe = header[header.index("typedef struct {\n    uint32_t now_ms;"):header.index("} APP_SysIdObserve;")]
    # 控制用陀螺之后只许再有 ALT 的高度观测（R-ALTID-1，末尾追加，Python 镜像同序）。
    tail = observe[observe.index("float    gyro_ctrl_rad_s[3];"):]
    assert [line.split()[-1] for line in tail.splitlines()
            if line.strip().startswith(("float", "uint8_t"))] == [
        "gyro_ctrl_rad_s[3];", "height_valid;", "height_m;", "height_raw_m;", "vz_m_s;",
        "az_m_s2;", "vbat_v;"]


# ---------------------------------------------------------------- 命令与构建


def test_the_command_family_hangs_off_the_fallback_chain():
    fallback = read("App/Src/app_cmd_fallback.c")
    assert '#include "app_rpm_notch.h"' in fallback
    chain = fallback[fallback.index("void app_control_handle_unclaimed("):]
    assert chain.index("APP_RpmNotch_Command(tokens, count)") < chain.index('"ERR unknown cmd %s')
    assert "RPMNOTCH" not in read("App/Src/app_control.c")


def test_provenance_follows_only_a_successful_start():
    source = strip_comments(read("App/Src/app_cmd_sysid.c"))
    # 陷波与舵机回差补偿两行溯源都只跟在开跑成功之后（回差那行见 test_servo_backlash_policy.py）。
    assert re.search(r"if \(APP_SysId_Start\(\) != 0U\) \{\s*"
                     r"APP_RpmNotch_ReportProvenance\(APP_SysId_GetRunId\(\)\);\s*"
                     r"APP_ServoBacklash_ReportProvenance\(APP_SysId_GetRunId\(\)\);\s*\}", source)


def test_cmake_lists_the_new_sources():
    cmake = read("CMakeLists.txt")
    for path in ("Driver/Src/drv_rpm_notch.c", "App/Src/app_rpm_notch.c", "App/Src/app_cmd_rpmnotch.c"):
        assert f"    {path}\n" in cmake.replace("\r\n", "\n"), path
