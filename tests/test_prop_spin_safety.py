"""点电机窗口（`App/*/app_prop_spin.*`）的安全契约。

这是全仓库唯一一条能从上位机让电机转起来的路径，所以本文件钉的全部是
**"停下来"这件事**，而不是"能不能转"：

1. **心跳超时必须关窗，而不是把油门归零。** 归零但仍然开着，意味着下一条
   迟到的命令会让它立刻重新转起来——而那时人可能已经离开了。

2. **超时判定跑在 500 Hz 控制环里**，不在收命令的文本任务里。文本任务会因为
   Flash 擦写、大段打印阻塞几十毫秒甚至更久；判断"该停了"的代码要是和收命令的
   代码在同一个任务里，它们会一起卡住。（这一条由 tests/test_dshot_bsp.py 的
   仲裁 seam 在真 BSP 上验，那里才看得见寄存器。）

3. **非法命令不续命，而且当场关窗。** 一个把百分比算错的上位机不该继续被当成
   活着的心跳，让电机保持上一次的转速而界面上的数字在变。

4. **谁抢走执行器谁说了算。** 解锁 / 验收 / 舵机标定 / 辨识一旦开始，窗口立刻
   关闭；重新开窗需要人再确认一次，不是"让位后自动回来"。

5. **油门上限由固件裁决，而且默认那条不因为别的用途被放宽。** 界面是可以被绕过的，
   而这条命令能让桨转。上限现在分两层：不说话是 20%，想更高必须在开窗时显式说出来
   且越不过 100% 的硬顶，并且跟着窗口一起失效——开着的窗口不能靠重发命令涨权限。

harness 用宿主 gcc 真编译固件那份 .c。状态机不含 HAL/RTOS，所以能整个跑一遍，
包括时间推进和 32 位毫秒回绕。
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
APP_INC = ROOT / "App" / "Inc"
APP_SRC = ROOT / "App" / "Src"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


HARNESS = r"""
#include "app_prop_spin.h"

#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

static APP_PropSpinOutput out_now(void)
{
    APP_PropSpinOutput out;

    APP_PropSpin_GetOutput(&out);
    return out;
}

/* ------------------------------------------------ 关着的时候是真的关着 */

static int test_closed_window_commands_nothing(void)
{
    APP_PropSpinOutput out;

    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_IsActive() == 0U, 10);

    out = out_now();
    CHECK(out.active == 0U, 11);
    CHECK(out.channel == 0U, 12);
    CHECK(out.percent[0] == 0U && out.percent[1] == 0U, 13);

    /* 没开窗时的油门命令必须被拒，且不会把窗口"顺手打开"。 */
    CHECK(APP_PropSpin_Command(1000U, 1U, 5U) == 0U, 14);
    CHECK(APP_PropSpin_IsActive() == 0U, 15);
    CHECK(out_now().percent[0] == 0U, 16);
    return 0;
}

/* ------------------------------------------------ 一次只转一路 */

static int test_only_one_channel_spins(void)
{
    APP_PropSpinOutput out;

    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(1000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 20);
    /* 刚开窗时油门必须是 0：开窗本身不该让任何东西转起来。 */
    out = out_now();
    CHECK(out.active == 1U && out.channel == 0U, 21);
    CHECK(out.percent[0] == 0U && out.percent[1] == 0U, 22);

    CHECK(APP_PropSpin_Command(1000U, 2U, 7U) == 1U, 23);
    out = out_now();
    CHECK(out.channel == 2U, 24);
    CHECK(out.percent[0] == 0U, 25);
    CHECK(out.percent[1] == 7U, 26);

    /* 换一路：上一路必须回零，不能两路一起转。 */
    CHECK(APP_PropSpin_Command(1010U, 1U, 3U) == 1U, 27);
    out = out_now();
    CHECK(out.percent[0] == 3U, 28);
    CHECK(out.percent[1] == 0U, 29);
    return 0;
}

/* ------------------------------------------------ 心跳 */

static int test_heartbeat_timeout_closes_the_window(void)
{
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(5000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 30);
    CHECK(APP_PropSpin_Command(5000U, 1U, 10U) == 1U, 31);

    /* 差一毫秒到阈值：照转。阈值贴着重发周期设会让窗口随机关闭，
     * 而"随机停"会训练操作者去忽略它。 */
    APP_PropSpin_Step(5000U + APP_PROP_SPIN_TIMEOUT_MS - 1U, 0U);
    CHECK(APP_PropSpin_IsActive() == 1U, 32);
    CHECK(out_now().percent[0] == 10U, 33);

    /* 到阈值：关窗。不是"油门归零但还开着"——归零但开着意味着一条迟到的
     * 命令能让它立刻重新转起来。 */
    APP_PropSpin_Step(5000U + APP_PROP_SPIN_TIMEOUT_MS, 0U);
    CHECK(APP_PropSpin_IsActive() == 0U, 34);
    CHECK(APP_PropSpin_LastStopReason() == (uint8_t)APP_PROP_SPIN_STOP_HEARTBEAT, 35);
    CHECK(out_now().percent[0] == 0U, 36);
    CHECK(out_now().channel == 0U, 37);

    /* 关窗之后迟到的心跳不该复活它。 */
    CHECK(APP_PropSpin_Command(5000U + APP_PROP_SPIN_TIMEOUT_MS, 1U, 10U) == 0U, 38);
    CHECK(APP_PropSpin_IsActive() == 0U, 39);
    return 0;
}

static int test_each_command_refreshes_the_heartbeat(void)
{
    uint32_t now = 9000U;
    uint32_t i;

    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(now, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 40);

    /* 每 100 ms 重发一次，跑够 3 秒：一直不该关。 */
    for (i = 0U; i < 30U; i++) {
        uint32_t tick;

        CHECK(APP_PropSpin_Command(now, 1U, 5U) == 1U, 41);
        for (tick = 0U; tick < 100U; tick += 2U) {
            APP_PropSpin_Step(now + tick, 0U);
        }
        now += 100U;
    }
    CHECK(APP_PropSpin_IsActive() == 1U, 42);
    CHECK(out_now().percent[0] == 5U, 43);
    return 0;
}

static int test_the_timeout_survives_the_millisecond_wrap(void)
{
    /*
     * now_ms 是 32 位毫秒计数，约 49 天回绕一次。用减法比较是无符号的，
     * 所以回绕点上仍然得到真实的经过时间。写成 `now > last + TIMEOUT` 的话，
     * 回绕那一瞬间窗口会永远关不掉——而那台飞控已经连续上电了七周。
     */
    const uint32_t before_wrap = 0xFFFFFF00UL;

    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(before_wrap, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 50);
    CHECK(APP_PropSpin_Command(before_wrap, 1U, 5U) == 1U, 51);

    /* 跨过 0 之后还没到超时。 */
    APP_PropSpin_Step((uint32_t)(before_wrap + APP_PROP_SPIN_TIMEOUT_MS - 1U), 0U);
    CHECK(APP_PropSpin_IsActive() == 1U, 52);

    APP_PropSpin_Step((uint32_t)(before_wrap + APP_PROP_SPIN_TIMEOUT_MS), 0U);
    CHECK(APP_PropSpin_IsActive() == 0U, 53);
    CHECK(APP_PropSpin_LastStopReason() == (uint8_t)APP_PROP_SPIN_STOP_HEARTBEAT, 54);
    return 0;
}

/* ------------------------------------------------ 非法命令 */

static int test_a_bad_command_stops_instead_of_being_ignored(void)
{
    /*
     * 忽略非法命令的话，一个把百分比算错成 80 的上位机会一直被当成活着的心跳，
     * 电机保持上一次的转速，而界面上的数字在变——两边各自自洽，谁也不会发现。
     */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(2000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 60);
    CHECK(APP_PropSpin_Command(2000U, 1U, 5U) == 1U, 61);

    CHECK(APP_PropSpin_Command(2010U, 1U,
                               (uint8_t)(APP_PROP_SPIN_DEFAULT_MAX_PERCENT + 1U)) == 0U, 62);
    CHECK(APP_PropSpin_IsActive() == 0U, 63);
    CHECK(APP_PropSpin_LastStopReason() == (uint8_t)APP_PROP_SPIN_STOP_REJECTED, 64);
    CHECK(out_now().percent[0] == 0U, 65);

    /* 通道号同理。 */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(2000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 66);
    CHECK(APP_PropSpin_Command(2000U, 3U, 1U) == 0U, 67);
    CHECK(APP_PropSpin_IsActive() == 0U, 68);

    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(2000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 69);
    CHECK(APP_PropSpin_Command(2000U, 0U, 1U) == 0U, 70);
    CHECK(APP_PropSpin_IsActive() == 0U, 71);

    /* 上限本身收得住。 */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(2000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 72);
    CHECK(APP_PropSpin_Command(2000U, 1U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 73);
    CHECK(out_now().percent[0] == APP_PROP_SPIN_DEFAULT_MAX_PERCENT, 74);
    return 0;
}

/* ------------------------------------------------ 抢占 */

static int test_anything_that_wants_the_actuators_closes_the_window(void)
{
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(3000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 80);
    CHECK(APP_PropSpin_Command(3000U, 2U, 8U) == 1U, 81);

    /* 心跳还新鲜，但有人抢执行器：立刻关，不是等超时。 */
    APP_PropSpin_Step(3000U, 1U);
    CHECK(APP_PropSpin_IsActive() == 0U, 82);
    CHECK(APP_PropSpin_LastStopReason() == (uint8_t)APP_PROP_SPIN_STOP_INHIBIT, 83);
    CHECK(out_now().percent[1] == 0U, 84);

    /* 抢占结束后**不会**自动回来：重新开窗需要人再确认一次。 */
    APP_PropSpin_Step(3000U, 0U);
    CHECK(APP_PropSpin_IsActive() == 0U, 85);
    return 0;
}

static int test_reopening_does_not_inherit_the_old_throttle(void)
{
    /*
     * 断线重连之后的第一拍应该是停着的，让操作者重新决定给多少，
     * 而不是接着上次的转速往下跑——上一次给多少，重连的人未必记得。
     */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(4000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 90);
    CHECK(APP_PropSpin_Command(4000U, 1U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 91);
    CHECK(out_now().percent[0] == APP_PROP_SPIN_DEFAULT_MAX_PERCENT, 92);

    CHECK(APP_PropSpin_Open(4100U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 93);      /* 幂等重开 */
    CHECK(APP_PropSpin_IsActive() == 1U, 94);
    CHECK(out_now().percent[0] == 0U, 95);
    CHECK(out_now().channel == 0U, 96);
    return 0;
}

static int test_explicit_stop_reports_why(void)
{
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(6000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 100);
    CHECK(APP_PropSpin_Command(6000U, 1U, 4U) == 1U, 101);
    APP_PropSpin_Close(6000U, (uint8_t)APP_PROP_SPIN_STOP_REQUEST);
    CHECK(APP_PropSpin_IsActive() == 0U, 102);
    CHECK(APP_PropSpin_LastStopReason() == (uint8_t)APP_PROP_SPIN_STOP_REQUEST, 103);
    CHECK(APP_PropSpin_HeartbeatAgeMs(9999U) == 0U, 104);

    /* 每个停机原因都要有能报出来的名字，不能是 "unknown"。 */
    CHECK(strcmp(APP_PropSpin_StopReasonName(
              (uint8_t)APP_PROP_SPIN_STOP_HEARTBEAT), "heartbeat_lost") == 0, 105);
    CHECK(strcmp(APP_PropSpin_StopReasonName(
              (uint8_t)APP_PROP_SPIN_STOP_INHIBIT), "inhibited") == 0, 106);
    CHECK(strcmp(APP_PropSpin_StopReasonName(
              (uint8_t)APP_PROP_SPIN_STOP_REJECTED), "rejected") == 0, 107);
    CHECK(strcmp(APP_PropSpin_StopReasonName(200U), "unknown") == 0, 108);
    return 0;
}

static int test_heartbeat_age_is_reported_for_diagnosis(void)
{
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(7000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 110);
    CHECK(APP_PropSpin_Command(7000U, 1U, 2U) == 1U, 111);
    CHECK(APP_PropSpin_HeartbeatAgeMs(7120U) == 120U, 112);
    return 0;
}

/* ------------------------------------------------ 本次窗口的油门上限 */

static int test_the_ceiling_belongs_to_the_window_not_to_the_build(void)
{
    /*
     * 上限从编译期常量变成"开窗时说出来的数"之后，多出三条必须钉住的性质：
     * 越不过硬顶、开着的窗口不能靠重发命令涨权限、关掉之后回到默认。
     * 少任何一条，"先用 20% 确认安全再偷偷抬到 100%"就成了一条无声的路径。
     */
    APP_PropSpin_Reset();

    /* 1. 硬顶之外一律不开窗。0 也不行：一个永远拒绝所有油门的窗口没有意义，
     *    而它会让操作者以为"开了但是不转"。 */
    CHECK(APP_PropSpin_Open(1000U, 0U) == 0U, 120);
    CHECK(APP_PropSpin_IsActive() == 0U, 121);
    CHECK(APP_PropSpin_Open(1000U,
              (uint8_t)(APP_PROP_SPIN_HARD_MAX_PERCENT + 1U)) == 0U, 122);
    CHECK(APP_PropSpin_IsActive() == 0U, 123);

    /* 2. 说出来的上限就是本次判据，硬顶本身可达。 */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(1000U, APP_PROP_SPIN_HARD_MAX_PERCENT) == 1U, 124);
    CHECK(out_now().max_percent == APP_PROP_SPIN_HARD_MAX_PERCENT, 125);
    CHECK(APP_PropSpin_Command(1000U, 1U, APP_PROP_SPIN_HARD_MAX_PERCENT) == 1U, 126);
    CHECK(out_now().percent[0] == APP_PROP_SPIN_HARD_MAX_PERCENT, 127);

    /* 3. 关掉之后回到默认，而不是留着上一次那个高上限等人误拖。 */
    APP_PropSpin_Close(1000U, (uint8_t)APP_PROP_SPIN_STOP_REQUEST);
    CHECK(out_now().max_percent == APP_PROP_SPIN_DEFAULT_MAX_PERCENT, 128);

    /* 4. 开着的窗口只降不升。 */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(2000U, 20U) == 1U, 130);
    CHECK(APP_PropSpin_Open(2000U, 100U) == 1U, 131);   /* 幂等成功，但不涨权限 */
    CHECK(out_now().max_percent == 20U, 132);
    CHECK(APP_PropSpin_Command(2000U, 1U, 50U) == 0U, 133);
    CHECK(APP_PropSpin_IsActive() == 0U, 134);

    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(3000U, 100U) == 1U, 135);
    CHECK(APP_PropSpin_Open(3000U, 30U) == 1U, 136);    /* 降是允许的 */
    CHECK(out_now().max_percent == 30U, 137);
    CHECK(APP_PropSpin_Command(3000U, 1U, 31U) == 0U, 138);

    /* 5. 参数写错的 ARM 不该顺手把正在转的电机停掉——它只是不算数。 */
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(4000U, 40U) == 1U, 140);
    CHECK(APP_PropSpin_Command(4000U, 2U, 35U) == 1U, 141);
    CHECK(APP_PropSpin_Open(4000U, 0U) == 0U, 142);
    CHECK(APP_PropSpin_IsActive() == 1U, 143);
    CHECK(out_now().max_percent == 40U, 144);
    CHECK(out_now().percent[1] == 35U, 145);

    /* 6. 默认上限没被这次改动放宽：不说话仍然是 20%。 */
    CHECK(APP_PROP_SPIN_DEFAULT_MAX_PERCENT == 20U, 146);
    APP_PropSpin_Reset();
    CHECK(APP_PropSpin_Open(5000U, APP_PROP_SPIN_DEFAULT_MAX_PERCENT) == 1U, 147);
    CHECK(APP_PropSpin_Command(5000U, 1U, 21U) == 0U, 148);
    CHECK(APP_PropSpin_IsActive() == 0U, 149);
    return 0;
}

int main(void)
{
    int rc;

    rc = test_closed_window_commands_nothing(); if (rc) { return rc; }
    rc = test_only_one_channel_spins(); if (rc) { return rc; }
    rc = test_heartbeat_timeout_closes_the_window(); if (rc) { return rc; }
    rc = test_each_command_refreshes_the_heartbeat(); if (rc) { return rc; }
    rc = test_the_timeout_survives_the_millisecond_wrap(); if (rc) { return rc; }
    rc = test_a_bad_command_stops_instead_of_being_ignored(); if (rc) { return rc; }
    rc = test_anything_that_wants_the_actuators_closes_the_window(); if (rc) { return rc; }
    rc = test_reopening_does_not_inherit_the_old_throttle(); if (rc) { return rc; }
    rc = test_explicit_stop_reports_why(); if (rc) { return rc; }
    rc = test_heartbeat_age_is_reported_for_diagnosis(); if (rc) { return rc; }
    rc = test_the_ceiling_belongs_to_the_window_not_to_the_build(); if (rc) { return rc; }

    printf("prop spin harness ok\n");
    return 0;
}
"""


def test_prop_spin_on_host_gcc(tmp_path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{APP_INC}",
            str(APP_SRC / "app_prop_spin.c"),
            str(harness), "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    assert "prop spin harness ok" in result.stdout.decode("ascii", "replace")


def test_the_module_stays_free_of_hardware_and_rtos() -> None:
    """纯状态机：不含 HAL、不含 RTOS、不做 I/O。

    做不到就说明超时判定没法在宿主上整个跑一遍，而那正是本文件唯一要证的东西。
    """
    for path in ("App/Inc/app_prop_spin.h", "App/Src/app_prop_spin.c"):
        text = strip_c_comments(read(path))
        for banned in ("stm32h7xx_hal", "main.h", "cmsis_os", "HAL_",
                       "osDelay", "BSP_", "printf"):
            assert banned not in text, f"{path} 引入了 {banned}"


def test_the_timeout_is_evaluated_by_the_control_loop_not_the_text_task() -> None:
    """`APP_PropSpin_Step()` 必须跑在 500 Hz 执行器提交点上。

    这不是风格问题：收命令的文本任务会因为 Flash 擦写、大段打印阻塞几十毫秒
    甚至更久。判断"该停了"的代码要是和收命令的代码在同一个任务里，它们会一起
    卡住——而卡住期间电机一直转着。放在控制环里，唯一能让它停不下来的情形是
    控制环本身死了，那时看门狗会复位。
    """
    stabilizer = strip_c_comments(read("App/Src/app_stabilizer.c"))
    assert "APP_PropSpin_Step(frame->now_ms," in stabilizer
    # 文本任务那边只准发命令，不准自己判超时。
    propcal = strip_c_comments(read("App/Src/app_cmd_propcal.c"))
    assert "APP_PropSpin_Step(" not in propcal

    # Step 必须排在输出仲裁**之前**：排在后面的话，超时那一拍仍然会按旧油门
    # 发一次，而"停"是这条链上唯一不能迟一拍的动作。
    step = stabilizer.index("APP_PropSpin_Step(frame->now_ms,")
    arbitration = stabilizer.index("if (APP_Acceptance_IsActive() != 0U) {", step - 400)
    assert step < arbitration


def test_arming_and_every_other_actuator_owner_inhibits_the_window() -> None:
    """抢占集合必须与实际会写 ESC 的那几条分支一致。

    漏掉任何一条的后果是两个写者同时往同一路 ESC 发命令，而最后一个写的赢——
    表现为"油门偶尔跳一下"，查起来像电调问题。
    """
    stabilizer = strip_c_comments(read("App/Src/app_stabilizer.c"))
    start = stabilizer.index("APP_PropSpin_Step(frame->now_ms,")
    inhibit = stabilizer[start:stabilizer.index(");", start)]

    for owner in ("frame->rc_armed", "APP_Acceptance_IsActive()",
                  "frame->servo_cal_active", "frame->ident_running"):
        assert owner in inhibit, owner


def test_raw_motor_commands_are_still_disabled() -> None:
    """点电机窗口**没有**放宽原来那道总闸。

    `APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS` 仍然是 0：新路径是另开的一扇带锁的门，
    不是把旧门的锁拆了。顺手把它打开会让 `MOTOR SET` 这类没有心跳保护的命令
    一起复活，而那条路径上没有任何东西会让电机停下来。
    """
    control = read("App/Src/app_control.c")
    assert "#define APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS 0U" in control
    assert "#define APP_CONTROL_ALLOW_IDENT_MOTOR_TEST 0U" in control


def test_opening_the_window_needs_an_explicit_confirmation_word() -> None:
    """`PROPCAL SPIN` 手滑敲一半不该让电机转起来。"""
    propcal = read("App/Src/app_cmd_propcal.c")
    assert 'PROPCAL_SPIN_CONFIRM_TOKEN "safe"' in propcal
    body = propcal[propcal.index('if (strcmp(tokens[2], "ARM") == 0)'):]
    body = body[:body.index("APP_PropSpin_Open(")]
    assert "PROPCAL_SPIN_CONFIRM_TOKEN" in body
    assert "need_confirm" in body
    # 开窗前必须逐条检查前置条件，且回包要说明是哪一条。
    assert "propcal_spin_block_reason()" in body


def test_raising_the_throttle_ceiling_has_to_be_asked_for() -> None:
    """默认那条上限不能因为"台架要推到 100%"被整体放宽。

    把常量从 20 改成 100 也能让作者在台架上推满油门，但那样一来，**看旋向**这条
    老路径的保护就一起没了：一次误拖滑条就是满推力。所以默认值必须原地不动，
    抬高上限得是一个显式的、当次有效的动作。

    这条用例针对的是"以后有人图省事把默认值改大"——运行期用例里那几个断言都是
    拿 `APP_PROP_SPIN_DEFAULT_MAX_PERCENT` 写的，跟着一起变就仍然全绿。
    """
    header = read("App/Inc/app_prop_spin.h")
    assert re.search(r"#define\s+APP_PROP_SPIN_DEFAULT_MAX_PERCENT\s+20U", header)
    assert re.search(r"#define\s+APP_PROP_SPIN_HARD_MAX_PERCENT\s+100U", header)

    propcal = read("App/Src/app_cmd_propcal.c")
    body = propcal[propcal.index('if (strcmp(tokens[2], "ARM") == 0)'):]
    body = body[:body.index("propcal_report_spin();")]
    # 省略 max_pct 走默认值；写了就必须校验，越界要说清楚是哪一条不合法。
    assert "APP_PROP_SPIN_DEFAULT_MAX_PERCENT" in body
    assert '"max_pct"' in body
    assert "bad_max_pct" in body
    assert "APP_PROP_SPIN_HARD_MAX_PERCENT" in body


def test_a_missing_esc_reading_reports_a_dash_not_a_zero() -> None:
    """心跳回包捎带的电调数据，缺失时必须是 `-`。

    `0` 在这几路上**本来就是合法值**：电调明确回报"未旋转"时 eRPM 就是 0，EDT 电流
    的分辨率是 1 A/LSB，小电流下也是 0。拿一个合法值当"没有"，上位机就再也分不清
    "电调说它停着"和"根本没收到回传"——而这一页正是用来判断电调有没有在说话的。

    新鲜度同样是判据的一部分：过期报最后一个值，界面上就是一个僵住不动的数字，
    比一个"没有"更容易被当成真的。
    """
    propcal = read("App/Src/app_cmd_propcal.c")
    assert "esc_telem=%u" in propcal
    for field in ("esc_i1=%s", "esc_i2=%s", "esc_erpm1=%s", "esc_erpm2=%s"):
        assert field in propcal
    assert 'snprintf(out, size, "-")' in propcal
    assert "PROPCAL_ESC_ERPM_FRESH_MS" in propcal
    assert "PROPCAL_ESC_CURRENT_FRESH_MS" in propcal


def test_the_decode_counters_do_not_ride_on_the_heartbeat() -> None:
    """解码计数只在开窗/停止/裸查询时发，不跟着 10 Hz 心跳走。

    两条理由，缺一条这个设计就不成立：
    1. 心跳那一行已经接近 `APP_UART_TX_TEXT_SIZE` 的一半，再塞六个 32 位计数会
       把它顶到溢出边上——截断的回包比没有回包更难查。
    2. 计数是慢变量。挂在 10 Hz 上等于为一个"跑一轮前后对比一次"的诊断，
       常年多占约 800 B/s 的链路。
    """
    propcal = read("App/Src/app_cmd_propcal.c")
    spin = propcal[propcal.index("static void propcal_handle_spin"):]
    spin = spin[:spin.index('reason=PROPCAL SPIN ARM|SET|STOP')]
    set_branch = spin[spin.index('if (strcmp(tokens[2], "SET") == 0)'):]
    assert "propcal_report_spin();" in set_branch
    assert "propcal_report_esc_diag();" not in set_branch
    # 而开窗 / 停止 / 裸查询三条路上都要有。
    assert spin[:spin.index('if (strcmp(tokens[2], "SET") == 0)')].count(
        "propcal_report_esc_diag();") == 3
    # 三类计数一个都不能少：少一个就分不出"没收到"和"解不开"。
    for field in ("fr1=%lu", "crc1=%lu", "to1=%lu",
                  "fr2=%lu", "crc2=%lu", "to2=%lu", "avail=%u", "proto=%u"):
        assert field in propcal


def test_the_window_blocks_editing_the_map_while_a_motor_spins() -> None:
    """正在转的那一路的含义，不能在人眼盯着它的时候被改掉。"""
    propcal = strip_c_comments(read("App/Src/app_cmd_propcal.c"))
    allowed = propcal[propcal.index("static uint8_t propcal_write_allowed(void)"):]
    allowed = allowed[:allowed.index("\n}")]
    assert "APP_Stabilizer_IsArmed() == 0U" in allowed
    assert "APP_PropSpin_IsActive() == 0U" in allowed
