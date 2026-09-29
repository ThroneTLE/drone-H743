"""ESC command window safety: only EDT on/off or the fixed AM32 KV write reach the ESC.

电调特殊命令窗口（`App/*/app_esc_command.*`）的安全契约。

这是全仓库**第二条**能让飞控主动对电调做点什么的路径（第一条是 `PROPCAL SPIN`
让电机转）。它比点电机更需要小心的地方在于：DShot 特殊命令号 1..47 里躺着
7/8/20/21（改电机转向）和 12（写电调 Flash）——一条发错的命令不是"这次没反应"，
而是**持久化到电调里、重启也恢复不了**。

所以本文件钉的是：

1. **只有 EDT 那两条命令号能被发出去。** 命令号是编译期常量，不从命令行取。
2. **抢占比点电机更严。** 解锁 / 验收 / 舵机标定 / 辨识 / 点电机 / 台架，任何一个
   在跑都立刻中止——命令帧会顶掉那一拍的油门，而后两者正靠连续油门维持电调解锁。
3. **中途失败整条作废，不补发。** 电调认的是连续若干帧；补上去的是一条长度不够的
   新序列，而上层会以为发成功了。
4. **油门入口永远编不出特殊命令。** 这条在 Driver 层（tests/test_dshot_command_encode.py），
   这里钉的是 App 层不绕过它。

harness 用宿主 gcc 真编译固件那份 .c。状态机不含 HAL/RTOS，能整个跑一遍。
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


HARNESS = r"""
#include "app_esc_command.h"

#include <stdio.h>
#include <string.h>

#define CHECK(cond, code) do { if (!(cond)) { \
    fprintf(stderr, "check %d failed at line %d: %s\n", (code), __LINE__, #cond); \
    return (code); } } while (0)

/* 把窗口推进 n 拍，每拍都"发成功"。返回真正发出去的帧数。 */
static uint32_t drain(uint32_t ticks)
{
    uint32_t sent = 0U;
    for (uint32_t i = 0U; i < ticks; ++i) {
        uint16_t cmd = 0U;
        APP_EscCommand_Step(0U);
        if (APP_EscCommand_Pending(&cmd) != 0U) {
            APP_EscCommand_Consume();
            sent++;
        }
    }
    return sent;
}

static int test_only_the_edt_pair_is_accepted(void)
{
    APP_EscCommand_Reset();
    /* 改转向的 7/8/20/21 和写 Flash 的 12 必须进不来。 */
    CHECK(APP_EscCommand_Request(7U) == 0U, 10);
    CHECK(APP_EscCommand_Request(8U) == 0U, 11);
    CHECK(APP_EscCommand_Request(12U) == 0U, 12);
    CHECK(APP_EscCommand_Request(20U) == 0U, 13);
    CHECK(APP_EscCommand_Request(21U) == 0U, 14);
    /* 油门范围同样不行：它根本不是命令。 */
    CHECK(APP_EscCommand_Request(0U) == 0U, 15);
    CHECK(APP_EscCommand_Request(48U) == 0U, 16);
    CHECK(APP_EscCommand_Request(2047U) == 0U, 17);
    /* 一条都没被接受，所以窗口仍然是关着的。 */
    CHECK(APP_EscCommand_IsActive() == 0U, 18);
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_IDLE, 19);

    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 20);
    CHECK(APP_EscCommand_IsActive() == 1U, 21);
    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_DISABLE) == 1U, 22);
    return 0;
}

static int test_it_sends_exactly_the_repeat_count(void)
{
    uint16_t cmd = 0U;

    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 30);
    /* 请求那一刻还没发过任何帧：请求不等于发送。 */
    CHECK(APP_EscCommand_SentFrames() == 0U, 31);
    CHECK(APP_EscCommand_Pending(&cmd) == 1U, 32);
    CHECK(cmd == APP_ESC_COMMAND_EDT_ENABLE, 33);

    /* 给足余量的拍数，实际只该发 REPEATS 帧然后自己停。 */
    CHECK(drain(APP_ESC_COMMAND_REPEATS * 3U) == APP_ESC_COMMAND_REPEATS, 34);
    CHECK(APP_EscCommand_SentFrames() == APP_ESC_COMMAND_REPEATS, 35);
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_DONE, 36);
    CHECK(APP_EscCommand_IsActive() == 0U, 37);
    /* 发完之后不再占住提交点——否则油门永远回不来。 */
    CHECK(APP_EscCommand_Pending(&cmd) == 0U, 38);
    return 0;
}

static int test_anything_that_wants_the_actuators_aborts_it(void)
{
    uint16_t cmd = 0U;

    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 40);
    CHECK(drain(3U) == 3U, 41);

    /* 抢占：当场中止，而不是"发完这一条再让位"。 */
    APP_EscCommand_Step(1U);
    CHECK(APP_EscCommand_IsActive() == 0U, 42);
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_ABORTED, 43);
    CHECK(APP_EscCommand_LastAbortReason() ==
          (uint8_t)APP_ESC_COMMAND_ABORT_INHIBIT, 44);
    /* 中止之后一帧都不许再发出去。 */
    CHECK(APP_EscCommand_Pending(&cmd) == 0U, 45);
    CHECK(drain(APP_ESC_COMMAND_REPEATS * 2U) == 0U, 46);
    /* 只发了 3 帧就断了，计数必须如实反映，不能凑成 REPEATS。 */
    CHECK(APP_EscCommand_SentFrames() == 3U, 47);

    /* 抢占结束**不会**自动接着发：重新发需要人再请求一次。 */
    APP_EscCommand_Step(0U);
    CHECK(APP_EscCommand_IsActive() == 0U, 48);
    return 0;
}

static int test_a_failed_frame_kills_the_whole_sequence(void)
{
    uint16_t cmd = 0U;

    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 50);
    CHECK(drain(2U) == 2U, 51);

    /*
     * 提交失败 = 序列断了。补发出去的是一条长度不够的新序列，电调不会认，
     * 而上层会以为发成功了——那是最坏的一种失败。
     */
    CHECK(APP_EscCommand_Pending(&cmd) == 1U, 52);
    APP_EscCommand_Fail();
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_ABORTED, 53);
    CHECK(APP_EscCommand_LastAbortReason() ==
          (uint8_t)APP_ESC_COMMAND_ABORT_OUTPUT, 54);
    CHECK(APP_EscCommand_Pending(&cmd) == 0U, 55);
    CHECK(drain(APP_ESC_COMMAND_REPEATS * 2U) == 0U, 56);
    CHECK(APP_EscCommand_SentFrames() == 2U, 57);
    return 0;
}

static int test_a_second_request_does_not_cut_the_first_short(void)
{
    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 60);
    CHECK(drain(2U) == 2U, 61);
    /*
     * 发到一半被另一条顶掉，电调那边两条都收不满连续帧——两条都不生效，
     * 而界面显示"已发送"。拒绝比排队更诚实：排队掩盖不了第一条已经断了。
     */
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_DISABLE) == 0U, 62);
    CHECK(APP_EscCommand_LastCommand() == APP_ESC_COMMAND_EDT_ENABLE, 63);
    CHECK(drain(APP_ESC_COMMAND_REPEATS) == APP_ESC_COMMAND_REPEATS - 2U, 64);
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_DONE, 65);
    /* 发完之后可以发下一条。 */
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_DISABLE) == 1U, 66);
    CHECK(APP_EscCommand_SentFrames() == 0U, 67);
    return 0;
}

static int test_every_state_has_a_name(void)
{
    CHECK(strcmp(APP_EscCommand_StateName(APP_ESC_COMMAND_IDLE), "idle") == 0, 70);
    CHECK(strcmp(APP_EscCommand_StateName(APP_ESC_COMMAND_SENDING), "sending") == 0, 71);
    CHECK(strcmp(APP_EscCommand_StateName(APP_ESC_COMMAND_DONE), "done") == 0, 72);
    CHECK(strcmp(APP_EscCommand_StateName(APP_ESC_COMMAND_ABORTED), "aborted") == 0, 73);
    CHECK(strcmp(APP_EscCommand_AbortReasonName(
              (uint8_t)APP_ESC_COMMAND_ABORT_INHIBIT), "inhibited") == 0, 74);
    CHECK(strcmp(APP_EscCommand_AbortReasonName(
              (uint8_t)APP_ESC_COMMAND_ABORT_OUTPUT), "output_failed") == 0, 75);
    CHECK(strcmp(APP_EscCommand_AbortReasonName(200U), "unknown") == 0, 76);
    return 0;
}

static int test_pending_rejects_a_null_out(void)
{
    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 80);
    CHECK(APP_EscCommand_Pending(NULL) == 0U, 81);
    /* 问都没问成，计数不该动。 */
    CHECK(APP_EscCommand_SentFrames() == 0U, 82);
    return 0;
}

/* AM32 >= 2.18 programming: enter x6 (a 7th would become the position), position,
 * value, commit, save x6. Any extra frame inside would be swallowed as data. */
static int test_motor_kv_sequence_is_the_exact_am32_programming_frames(void)
{
    static const uint16_t expected[15] = {
        36U, 36U, 36U, 36U, 36U, 36U, 26U, 32U, 37U, 12U, 12U, 12U, 12U, 12U, 12U};
    uint8_t kv_byte = 0U;

    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_MotorKvByte(1300U, &kv_byte) == 1U && kv_byte == 32U, 90);
    CHECK(APP_EscCommand_MotorKvByte(2220U, &kv_byte) == 0U, 91);
    CHECK(APP_EscCommand_MotorKvByte(1310U, &kv_byte) == 0U, 92);
    CHECK(APP_EscCommand_MotorKvByte(260U, &kv_byte) == 0U, 93);
    CHECK(APP_EscCommand_MotorKvByte(1900U, &kv_byte) == 1U && kv_byte == 47U, 94);
    CHECK(APP_EscCommand_MotorKvByte(1300U, NULL) == 0U, 95);
    CHECK(APP_EscCommand_RequestMotorKv(1310U) == 0U, 96);
    CHECK(APP_EscCommand_RequestMotorKv(1300U) == 1U, 97);
    CHECK(APP_EscCommand_PlannedFrames() == 15U, 98);
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 0U, 99);
    CHECK(APP_EscCommand_RequestMotorKv(1300U) == 0U, 100);
    for (uint32_t i = 0U; i < 15U; ++i) {
        uint16_t cmd = 0U;
        APP_EscCommand_Step(0U);
        CHECK(APP_EscCommand_Pending(&cmd) == 1U, 101);
        CHECK(cmd == expected[i], 102);
        APP_EscCommand_Consume();
    }
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_DONE, 103);
    CHECK(APP_EscCommand_SentFrames() == 15U, 104);
    {
        uint16_t cmd = 0U;
        CHECK(APP_EscCommand_Pending(&cmd) == 0U, 105);
    }
    return 0;
}

static int test_motor_kv_sequence_aborts_like_the_edt_window(void)
{
    uint16_t cmd = 0U;

    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_RequestMotorKv(1300U) == 1U, 110);
    CHECK(drain(3U) == 3U, 111);
    APP_EscCommand_Step(1U);
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_ABORTED, 112);
    CHECK(APP_EscCommand_Pending(&cmd) == 0U, 113);

    APP_EscCommand_Reset();
    CHECK(APP_EscCommand_RequestMotorKv(1300U) == 1U, 114);
    CHECK(drain(7U) == 7U, 115);
    APP_EscCommand_Fail();
    CHECK(APP_EscCommand_GetState() == APP_ESC_COMMAND_ABORTED, 116);
    CHECK(APP_EscCommand_Pending(&cmd) == 0U, 117);

    /* A later EDT request still sends only its own command, ten times. */
    CHECK(APP_EscCommand_Request(APP_ESC_COMMAND_EDT_ENABLE) == 1U, 118);
    CHECK(APP_EscCommand_Pending(&cmd) == 1U && cmd == APP_ESC_COMMAND_EDT_ENABLE, 119);
    CHECK(APP_EscCommand_PlannedFrames() == APP_ESC_COMMAND_REPEATS, 120);
    CHECK(drain(20U) == APP_ESC_COMMAND_REPEATS, 121);
    return 0;
}

int main(void)
{
    int rc;

    rc = test_only_the_edt_pair_is_accepted(); if (rc) { return rc; }
    rc = test_it_sends_exactly_the_repeat_count(); if (rc) { return rc; }
    rc = test_anything_that_wants_the_actuators_aborts_it(); if (rc) { return rc; }
    rc = test_a_failed_frame_kills_the_whole_sequence(); if (rc) { return rc; }
    rc = test_a_second_request_does_not_cut_the_first_short(); if (rc) { return rc; }
    rc = test_every_state_has_a_name(); if (rc) { return rc; }
    rc = test_pending_rejects_a_null_out(); if (rc) { return rc; }
    rc = test_motor_kv_sequence_is_the_exact_am32_programming_frames(); if (rc) { return rc; }
    rc = test_motor_kv_sequence_aborts_like_the_edt_window(); if (rc) { return rc; }

    printf("esc command harness ok\n");
    return 0;
}
"""


def test_esc_command_on_host_gcc(tmp_path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-o", str(executable),
         str(harness), str(ROOT / "App" / "Src" / "app_esc_command.c"),
         f"-I{ROOT / 'App' / 'Inc'}"],
        check=True, capture_output=True, text=True)
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_the_command_number_never_comes_from_the_command_line() -> None:
    """`ESC EDT ON|OFF` 只认两个词，命令号是编译期常量。

    做成 `ESC CMD <n>` 的话，敲错一个数字就可能发出 7/8/20/21（改电机转向）或
    12（写电调 Flash）——那不是重启能恢复的。所以这条不是风格问题：解析命令行
    整数的代码一旦出现在这条路径上，这道门就没了。
    """
    source = read("App/Src/app_cmd_esc_edt.c")
    assert '"ON"' in source and '"OFF"' in source
    # 命令号只能来自这两个常量。
    assert "APP_ESC_COMMAND_EDT_ENABLE" in source
    assert "APP_ESC_COMMAND_EDT_DISABLE" in source
    # 任何把命令行 token 转成整数的做法都不许出现在这个文件里。
    assert "app_control_parse_u32" not in source
    assert "atoi" not in source and "strtoul" not in source

    module = read("App/Src/app_esc_command.c")
    assert "APP_ESC_COMMAND_EDT_ENABLE" in module
    assert "APP_ESC_COMMAND_EDT_DISABLE" in module


def test_the_request_is_refused_while_armed_in_source() -> None:
    """解锁门是安全判据，源码里要看得见。

    运行期 harness 用的是纯状态机，够不到 IsArmed；而 500 Hz 那一侧的抢占判定
    是另一道门。两道门有意重复：这一道给出可读的拒绝理由，那一道保证即使从别处
    请求也停得下来。
    """
    source = read("App/Src/app_cmd_esc_edt.c")
    assert "APP_Stabilizer_IsArmed()" in source
    assert "reason=armed" in source


def test_the_500hz_seam_aborts_on_every_actuator_owner() -> None:
    """抢占判定必须把六个占用执行器的来源都算上。

    比点电机那条更严：命令帧会顶掉那一拍的油门，而点电机和台架正靠连续的油门帧
    维持电调解锁——中间插几帧命令，电机会掉出解锁状态，表现为"转着转着停了"。
    """
    source = read("App/Src/app_stabilizer.c")
    call = source.split("APP_EscCommand_Step(")[1].split(";")[0]
    for owner in ("rc_armed", "APP_Acceptance_IsActive", "servo_cal_active",
                  "ident_running", "APP_PropSpin_IsActive",
                  "APP_ThrustBench_IsActive"):
        assert owner in call, owner


def test_the_command_frame_never_clears_the_staged_throttle() -> None:
    """命令帧顶掉这一拍的提交，但**不动**暂存的油门。

    清零会让电调掉出解锁状态，表现成"发个 EDT 命令把电机停了"。正确行为是
    电调看到"命令 xN，然后油门回来"。
    """
    source = read("BSP/Src/bsp_pwm.c")
    body = source.split("BSP_PWM_Status BSP_PWM_CommitEscCommand(")[1]
    body = body.split("\n}")[0]
    assert "esc_pulses_us" not in body
    assert "BSP_DShot_SubmitCommand" in body


def test_motor_kv_write_needs_confirm_and_keeps_the_frame_list_fixed() -> None:
    """`ESC KV <kv> CONFIRM`：命令行只给 KV 值，帧序列在编译期写死。

    与 EDT 同两道门（解锁拒绝、PWM 档拒绝）；少了 CONFIRM 一律当用法错误。
    KV 字节位置 26 与 37 提交码只在 app_esc_command.h 定义，处理文件不能自造命令号。
    """
    source = read("App/Src/app_cmd_esc_kv.c")
    assert '"CONFIRM"' in source
    assert "APP_Stabilizer_IsArmed()" in source and "reason=armed" in source
    assert "reason=not_dshot" in source
    assert "APP_EscCommand_RequestMotorKv(" in source
    assert "APP_EscCommand_Request(" not in source
    assert "36U" not in source and "37U" not in source and "12U" not in source
    header = read("App/Inc/app_esc_command.h")
    for needle in ("APP_ESC_PROGRAM_ENTER          36U",
                   "APP_ESC_PROGRAM_ENTER_REPEATS   6U",
                   "APP_ESC_EEPROM_MOTOR_KV        26U",
                   "APP_ESC_PROGRAM_COMMIT         37U"):
        assert needle in header
    fallback = read("App/Src/app_cmd_fallback.c")
    assert fallback.index("app_control_handle_esc_kv(") < fallback.index(
        "app_control_handle_esc_edt_set(")
    assert "App/Src/app_cmd_esc_kv.c" in read("CMakeLists.txt")
