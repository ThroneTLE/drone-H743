"""控制环禁止同步阻塞 I/O 的契约。

APP_Control_QueueText 在入队 UART 前会同步阻塞等 USB CDC（最坏 3x
APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS）。app_servo_cal.c 的手势状态机整体
跑在 500Hz 控制环（stabilizer_control_prepare -> APP_ServoCal_Step），
曾经四处直接调用 QueueText，把最坏 ~30ms 的阻塞埋进姿态环。

修复后的结构：控制环内只 vsnprintf 进单条通告缓冲并置标志；通信任务的
app_control_tick_common 经 APP_ServoCal_TakeNotice() 取走后再排队发送。
本文件防止任何一半被回退。
"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVO_CAL = ROOT / "App" / "Src" / "app_servo_cal.c"
SERVO_CAL_HEADER = ROOT / "App" / "Inc" / "app_servo_cal.h"
CONTROL = ROOT / "App" / "Src" / "app_control.c"


def test_servo_cal_state_machine_never_calls_blocking_text_send() -> None:
    source = SERVO_CAL.read_text(encoding="utf-8")
    assert "APP_Control_QueueText(" not in source, (
        "app_servo_cal.c runs inside the 500Hz control loop; it must post "
        "notices instead of calling the blocking USB/UART text path"
    )
    assert '#include "app_control.h"' not in source, (
        "the control-loop module must not depend on app_control.h at all"
    )


def test_servo_cal_posts_notices_through_the_pending_buffer() -> None:
    source = SERVO_CAL.read_text(encoding="utf-8")
    assert "servo_cal_post_notice(" in source
    assert "servo_cal_notice_pending" in source
    assert "uint16_t APP_ServoCal_TakeNotice(" in source
    header = SERVO_CAL_HEADER.read_text(encoding="utf-8")
    assert "APP_ServoCal_TakeNotice" in header


def test_comms_task_flushes_the_notice_each_tick() -> None:
    source = CONTROL.read_text(encoding="utf-8")
    flush = source.find("APP_ServoCal_TakeNotice(")
    tick = source.find("static void app_control_tick_common(uint8_t emit_heartbeat)\n{")
    assert flush != -1, "app_control.c must flush the servo-cal notice"
    assert tick != -1, "app_control_tick_common definition not found"
    body = source[tick : source.find("\n}", tick)]
    assert "app_control_service_servo_cal_notice()" in body, (
        "app_control_tick_common must call the notice flush service"
    )
