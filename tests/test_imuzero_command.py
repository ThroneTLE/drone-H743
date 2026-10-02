"""`IMUZERO` 软件重新标定陀螺零偏与姿态零点（2026-10-01）。

作者原话："每次重新烧录舵机会动一下然后让机身开始晃动，然后这个时候又校准不了，我扶着也不是很准，
你不然就来一个软件重新标定的命令"。上电零偏把台架上的绕杆摆动当成了零偏（实测约 1.5°/s），融合
加速度误差 3.3° 超过姿态零点门限 3°，姿态角一直报 0。

本文件钉住接线：命令在独立模块、只在上锁时请求、两个任务各自取走请求后按上电同一规则重新采样。
"""

from __future__ import annotations

from pathlib import Path

from tools.panel_lib import ai_bridge_policy as policy

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


CMD = read("App/Src/app_cmd_imuzero.c")


def test_command_module_is_registered_and_built() -> None:
    assert "if (app_control_handle_imuzero(tokens, count) != 0U) { return; }" in read("App/Src/app_cmd_fallback.c")
    assert "uint8_t app_control_handle_imuzero(char **tokens, uint32_t count);" in read(
        "App/Inc/app_control_internal.h")
    assert "App/Src/app_cmd_imuzero.c" in read("CMakeLists.txt")
    assert "IMUZERO" not in read("App/Src/app_control.c"), "新命令不进 app_control.c"


def test_requests_only_when_disarmed() -> None:
    handler = CMD.split("uint8_t app_control_handle_imuzero")[1]
    guard = handler.index("APP_Stabilizer_IsArmed() != 0U")
    assert guard < handler.index("APP_Sensor_RequestGyroRecal();")
    assert guard < handler.index("APP_Stabilizer_RequestAttitudeRezero();")
    assert "IMUZERO state=armed_blocked" in handler
    assert "IMUZERO state=restarted" in handler
    assert "%f" not in CMD and "%.1f" not in CMD, "newlib-nano 没有浮点 printf"


def test_sensor_task_restarts_the_gyro_bias_before_calibrating() -> None:
    task = read("Core/Src/freertos.c")
    take = task.index("APP_Sensor_TakeGyroRecalRequest() != 0U")
    assert take < task.index("APP_Sensor_CalibrateGyroBias(g[0], g[1], g[2], &gyro_bias)")
    assert "gyro_bias = (APP_Sensor_GyroBias){0};" in task
    sensor = read("App/Src/app_sensor.c")
    assert "app_sensor_gyro_recal_request = 0U;" in sensor


def test_stabilizer_restarts_the_attitude_zero_window() -> None:
    stab = read("App/Src/app_stabilizer.c")
    block = stab.split("if (stabilizer_rezero_request != 0U) {")[1].split("}")[0]
    for line in ("ctx->attitude_zero_ready = 0U;", "ctx->attitude_zero_start_ms = 0U;",
                 "ctx->attitude_zero_count = 0U;", "ctx->roll_zero_sum = 0.0f;",
                 "ctx->pitch_zero_sum = 0.0f;", "ctx->yaw_zero_sum = 0.0f;"):
        assert line in block, line
    # 请求在零点采样判断之前取走，同一拍就按新规则开始。
    assert stab.index("if (stabilizer_rezero_request != 0U) {") < stab.index(
        "if ((ctx->attitude_zero_ready == 0U) &&")
    assert "stabilizer_attitude_zero_ready_mirror = ctx->attitude_zero_ready;" in stab


def test_ai_bridge_may_send_it() -> None:
    assert policy.classify("IMUZERO").kind == policy.ALLOW
    assert policy.classify("IMUZERO?").kind == policy.ALLOW
