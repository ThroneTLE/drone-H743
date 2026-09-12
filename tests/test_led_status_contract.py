"""LED 状态灯契约。

原来这一份只做字符串存在性检查（`assert "APP_LED_ARM_BLOCK_NO_RC" in header`），
从来没有验证过原因**选择逻辑**。于是 2026-09-05 实机撞上这个：坐标迁移未完成的
硬锁没有对应分支，拨杆已打、油门已收的正常状态一路掉进最后的 else，灯闪 3 下
报"拨杆没打"，把真正原因藏了起来。

少一个分支不新增任何字符串，所以 grep 式测试对这类缺陷完全隐形。下面两条机检
针对的就是这一类：**解锁门与 LED 原因必须一一对应，且顺序正确。**
"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def arm_updater_body() -> str:
    """`stabilizer_rc_update_armed()` 的函数体 —— 真正决定能否解锁的地方。"""
    source = read("App/Src/app_stabilizer.c")
    start = source.index("stabilizer_rc_update_armed(uint8_t switch_high")
    end = source.index("stabilizer_servo_should_send", start)
    return source[start:end]


def led_reason_chain() -> str:
    """LED 原因链 —— 从第一条判定到 APP_LED_SetArmStatus 调用为止。"""
    source = read("App/Src/app_stabilizer.c")
    start = source.index("if (frame->rc_link_seen == 0U) {")
    end = source.index("APP_LED_SetArmStatus(", start)
    return source[start:end]


def test_led_status_reports_arm_block_reasons_and_flow_health() -> None:
    header = read("App/Inc/app_led.h")
    source = read("App/Src/app_led.c")
    uart = read("App/Src/app_uart.c")
    tasks = read("App/Src/app_tasks.c")
    freertos = read("Core/Src/freertos.c") + read("App/Src/app_stabilizer.c")

    assert "APP_LED_ARM_BLOCK_NO_RC" in header
    assert "APP_LED_ARM_BLOCK_RC_LOSS" in header
    assert "APP_LED_ARM_BLOCK_ARM_SWITCH" in header
    assert "APP_LED_ARM_BLOCK_THROTTLE_HIGH" in header
    assert "APP_LED_ARM_BLOCK_IMU" in header
    assert "APP_LED_ARM_BLOCK_FRAME" in header
    assert "void APP_LED_SetArmStatus" in header
    assert "APP_OPTICAL_FLOW_HEALTH_OK" in source
    assert "APP_OPTICAL_FLOW_HEALTH_RETRYING" in source
    assert "APP_OPTICAL_FLOW_HEALTH_FAILED" in source
    # 原因码**原样**当闪烁次数用。中间插一次换算（+1、查表、按颜色分档）就会让
    # "数灯"和 `blinks=` 报的数字对不上，而两边单看都说得通。
    assert "app_led_pulses(app_led_amber, (uint8_t)reason)" in source
    # 灯的节拍归 LED 自己的任务，不再搭 UART 任务的车（那个节拍随串口忙闲变）。
    assert "APP_LED_Task_Step();" not in uart
    assert "APP_LED_Task_Step();" in tasks
    assert "APP_Task_LED_Init();" in freertos
    assert "APP_Task_LED_Start();" in freertos
    assert "APP_LED_SetArmStatus(frame->rc_armed, frame->led_arm_block_reason);" in freertos


def test_every_hard_arm_gate_has_its_own_led_reason() -> None:
    """每个在解锁函数里直接否决解锁的硬门，都必须在 LED 原因链里有一条分支。

    这是 2026-09-05 那个缺陷的直接机检：`APP_Stabilizer_IsImuFrameArmLocked()`
    在解锁函数最开头就返回 0，LED 链里却没有它，于是报了假原因。
    """
    gates = set(re.findall(r"(APP_\w+_Is\w*(?:ArmLocked|ArmBlocked))\(\)", arm_updater_body()))
    # 基线取自源码本身，不是手抄的名单：解锁函数里现在就有这两道硬门。
    assert gates == {
        "APP_Stabilizer_IsImuFrameArmLocked",
        "APP_ImuHealth_IsArmBlocked",
    }, f"解锁硬门集合变了，LED 原因链要同步：{sorted(gates)}"

    chain = led_reason_chain()
    for gate in sorted(gates):
        assert f"{gate}()" in chain, (
            f"{gate}() 能否决解锁，但 LED 原因链里没有它 —— "
            f"该状态会掉进后面的分支报出假原因"
        )


def test_hard_gates_are_tested_before_the_rc_armed_branch() -> None:
    """硬门必须排在 `rc_armed == 0` 之前。

    硬门会把 rc_armed 强制清零，排在它后面就永远走不到，等于没写。
    源码里 IMU 健康那一档旁边的注释讲的就是这条，但当初只防住了它自己。
    """
    chain = led_reason_chain()
    rc_armed_at = chain.index("frame->rc_armed == 0U")

    for gate in ("APP_Stabilizer_IsImuFrameArmLocked", "APP_ImuHealth_IsArmBlocked"):
        assert chain.index(f"{gate}()") < rc_armed_at, (
            f"{gate}() 必须排在 rc_armed 判定之前，否则该分支永远走不到"
        )


def test_every_declared_reason_is_actually_reachable() -> None:
    """头文件里声明的每个原因码，都必须真的被赋值过一次。

    声明了却没人赋值 = 现场永远看不到那个闪烁次数，等于骗人。
    """
    header = read("App/Inc/app_led.h")
    declared = set(re.findall(r"(APP_LED_ARM_BLOCK_\w+)\s*=\s*\d+", header))
    assigned = set(re.findall(r"=\s*(APP_LED_ARM_BLOCK_\w+)\s*;", read("App/Src/app_stabilizer.c")))

    assert declared, "头文件里应当声明原因码"
    assert declared == assigned, (
        f"声明与赋值不一致：只声明未赋值={sorted(declared - assigned)}，"
        f"只赋值未声明={sorted(assigned - declared)}"
    )


def test_reason_codes_are_append_only() -> None:
    """数值即闪烁次数，现场靠数闪几下判断，**只能追加不能重排**。"""
    header = read("App/Inc/app_led.h")
    codes = dict(
        (name, int(value))
        for name, value in re.findall(r"(APP_LED_ARM_BLOCK_\w+)\s*=\s*(\d+)", header)
    )
    assert codes["APP_LED_ARM_BLOCK_NONE"] == 0
    assert codes["APP_LED_ARM_BLOCK_NO_RC"] == 1
    assert codes["APP_LED_ARM_BLOCK_RC_LOSS"] == 2
    assert codes["APP_LED_ARM_BLOCK_ARM_SWITCH"] == 3
    assert codes["APP_LED_ARM_BLOCK_THROTTLE_HIGH"] == 4
    assert codes["APP_LED_ARM_BLOCK_IMU"] == 5
    assert codes["APP_LED_ARM_BLOCK_FRAME"] == 6
    assert len(set(codes.values())) == len(codes), "原因码数值不得重复"
