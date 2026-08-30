"""SERVO JOG（保持型地面点动）契约测试。

背景（M4 台架实测缺陷）：稳定环 commit 以 3µs 死区 + 500ms 强制刷新持续流式
下发舵机目标，一次性 SERVO MOVE 慢移会在下一次强制刷新被拉回中点。修复后的
点动必须：

1. 在稳定环输出仲裁点接管（优先级 手势标定 > 验收 > 反馈台架 > 解锁 > 点动）；
2. 从稳定环当前目标匀速斜坡逼近并保持，不跳变、不超调；
3. 超时 / 更高优先级出现时自动交还并通告；
4. 控制环上下文绝不排队 USB 文本（通告缓冲模式，同 app_servo_cal.c）。

行为部分用宿主 gcc 编译真实的 App/Src/app_servo_jog.c 验证；仲裁接线顺序无法
在宿主链接稳定环，退回源码级契约锁定。
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
STABILIZER = (ROOT / "App" / "Src" / "app_stabilizer.c").read_text(encoding="utf-8")
CONTROL = (ROOT / "App" / "Src" / "app_control.c").read_text(encoding="utf-8")
FREERTOS = (ROOT / "Core" / "Src" / "freertos.c").read_text(encoding="utf-8")
JOG_SOURCE = (ROOT / "App" / "Src" / "app_servo_jog.c").read_text(encoding="utf-8")


HARNESS = r"""
#include "app_servo_jog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

/* 宿主替身：捕获通信上下文的直接回复文本。 */
static char queue_text_last[256];

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    (void)vsnprintf(queue_text_last, sizeof(queue_text_last), format, args);
    va_end(args);
}

/* 以 2ms 控制节拍推进 Apply，返回两路输出脉宽（模拟稳定环流式目标）。 */
static void tick(uint32_t *now_ms, const char *yield_reason,
                 uint16_t stream_alpha, uint16_t stream_beta,
                 uint16_t *out_alpha, uint16_t *out_beta)
{
    *now_ms += 2U;
    *out_alpha = stream_alpha;
    *out_beta = stream_beta;
    APP_ServoJog_Apply(*now_ms, yield_reason, out_alpha, out_beta);
}

int main(void)
{
    uint32_t now_ms = 1000U;
    uint16_t alpha;
    uint16_t beta;
    char notice[64];

    APP_ServoJog_Init();

    /* 1. 校验拒绝：通道越界、脉宽出物理范围。 */
    CHECK(APP_ServoJog_Request(2U, 1500U, now_ms) == 0U, 10);
    CHECK(APP_ServoJog_Request(0U, 499U, now_ms) == 0U, 11);
    CHECK(APP_ServoJog_Request(0U, 2501U, now_ms) == 0U, 12);
    CHECK(APP_ServoJog_IsActive() == 0U, 13);

    /* 2. 未激活时 Apply 完全透传稳定环目标。 */
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1500U && beta == 1520U, 20);

    /* 3. 接管：从稳定环当前目标（1500）起以 500µs/s 斜坡逼近，1µs/2ms 节拍。 */
    CHECK(APP_ServoJog_Request(0U, 1850U, now_ms) == 1U, 30);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1501U, 31);          /* 第一拍：无跳变，只前进 1µs */
    CHECK(beta == 1520U, 32);           /* 未点动通道保持透传 */
    for (int i = 0; i < 348; ++i) {
        tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    }
    CHECK(alpha == 1849U, 33);          /* 349 拍 = 349µs，尚未到达 */
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1850U, 34);          /* 到达 */
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1850U, 35);          /* 保持：不再被流式目标拉回 */

    /* 4. 中途重定目标：从当前斜坡位置继续，不跳变（中点微调场景）。 */
    CHECK(APP_ServoJog_Request(0U, 1840U, now_ms) == 1U, 40);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1849U, 41);
    for (int i = 0; i < 20; ++i) {
        tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    }
    CHECK(alpha == 1840U, 42);

    /* 5. 手动释放后透传恢复；再次接管重新从流式目标起坡。 */
    APP_ServoJog_ReleaseAll();
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1500U, 50);
    CHECK(APP_ServoJog_Request(1U, 1600U, now_ms) == 1U, 51);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(beta == 1521U && alpha == 1500U, 52);

    /* 6. 更高优先级出现（解锁）：立即让位、透传、发通告。 */
    tick(&now_ms, "armed", 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1500U && beta == 1520U, 60);
    CHECK(APP_ServoJog_IsActive() == 0U, 61);
    CHECK(APP_ServoJog_TakeNotice(notice, sizeof(notice)) != 0U, 62);
    CHECK(strstr(notice, "released") != NULL, 63);
    CHECK(strstr(notice, "armed") != NULL, 64);
    CHECK(APP_ServoJog_TakeNotice(notice, sizeof(notice)) == 0U, 65);

    /* 7. 超时自动交还 + 通告（防遗忘点动长期霸占输出）。 */
    CHECK(APP_ServoJog_Request(0U, 1700U, now_ms) == 1U, 70);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1501U, 71);
    now_ms += APP_SERVO_JOG_HOLD_TIMEOUT_MS;
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1500U, 72);
    CHECK(APP_ServoJog_IsActive() == 0U, 73);
    CHECK(APP_ServoJog_TakeNotice(notice, sizeof(notice)) != 0U, 74);
    CHECK(strstr(notice, "timeout") != NULL, 75);

    /* 8. 命令面（通信上下文）：合法/STOP/非法用法的直接回复。 */
    {
        char t_servo[] = "SERVO";
        char t_jog[] = "JOG";
        char t_ch[] = "0";
        char t_us[] = "1850";
        char t_stop[] = "STOP";
        char t_bad[] = "9999";
        char *ok_tokens[4] = { t_servo, t_jog, t_ch, t_us };
        char *stop_tokens[3] = { t_servo, t_jog, t_stop };
        char *bad_tokens[4] = { t_servo, t_jog, t_ch, t_bad };

        queue_text_last[0] = '\0';
        APP_ServoJog_HandleCommand(ok_tokens, 4U, now_ms);
        CHECK(strstr(queue_text_last, "OK servo_jog ch=0 target=1850") != NULL, 80);
        CHECK(APP_ServoJog_IsActive() == 1U, 81);

        APP_ServoJog_HandleCommand(stop_tokens, 3U, now_ms);
        CHECK(strstr(queue_text_last, "released manual") != NULL, 82);
        CHECK(APP_ServoJog_IsActive() == 0U, 83);

        APP_ServoJog_HandleCommand(bad_tokens, 4U, now_ms);
        CHECK(strstr(queue_text_last, "ERR usage SERVO JOG") != NULL, 84);
        CHECK(APP_ServoJog_IsActive() == 0U, 85);
    }

    return 0;
}
"""


def test_servo_jog_runtime_behaviour(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    harness_path = tmp_path / "servo_jog_harness.c"
    harness_path.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "servo_jog_harness.exe"

    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'App' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_servo_jog.c"),
            str(harness_path),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for offset in range(brace, len(source)):
        if source[offset] == "{":
            depth += 1
        elif source[offset] == "}":
            depth -= 1
            if depth == 0:
                return source[start:offset + 1]
    raise AssertionError(f"unbalanced braces after {signature!r}")


def test_stabilizer_commit_arbitrates_jog_after_higher_priority_owners() -> None:
    commit = _function_body(
        STABILIZER, "static void stabilizer_control_commit(StabilizerContext *ctx,")

    # 手势标定活动时点动必须立即让位。
    assert 'APP_ServoJog_ForceRelease("servo_cal")' in commit

    # 仲裁顺序：验收覆盖 → 反馈台架目标 → 点动改写 → 记录目标。
    acceptance = commit.index("APP_Acceptance_GetServoOverride")
    fb_targets = commit.index("APP_ServoFeedbackBench_ApplyTargets")
    jog_apply = commit.index("APP_ServoJog_Apply")
    record = commit.index("stabilizer_servo_record_target")
    assert acceptance < fb_targets < jog_apply < record

    # 让位条件必须覆盖三个更高优先级来源。
    for reason in ('"acceptance"', '"fb_bench"', '"armed"'):
        assert reason in commit


def test_control_dispatches_jog_and_flushes_notice() -> None:
    handle_servo = _function_body(
        CONTROL, "static void app_control_handle_servo(char **tokens, uint32_t count)")
    assert 'strcmp(tokens[1], "JOG")' in handle_servo
    assert "APP_ServoJog_HandleCommand(tokens, count, HAL_GetTick());" in handle_servo

    # 带定义左括号匹配，避免撞上文件头部的前向声明。
    tick_common = _function_body(
        CONTROL, "static void app_control_tick_common(uint8_t emit_heartbeat)\n{")
    assert "app_control_service_servo_jog_notice();" in tick_common
    assert "APP_ServoJog_TakeNotice" in CONTROL


def test_jog_module_keeps_control_loop_context_contract() -> None:
    # 控制环上下文函数（Apply/ForceRelease/channel_apply）不得同步排队 USB 文本。
    for signature in (
        "void APP_ServoJog_Apply(",
        "void APP_ServoJog_ForceRelease(",
        "static void servo_jog_channel_apply(",
    ):
        body = _function_body(JOG_SOURCE, signature)
        assert "APP_Control_QueueText(" not in body, signature

    assert "APP_ServoJog_Init();" in FREERTOS


def test_jog_slew_and_timeout_constants_are_bench_sane() -> None:
    header = (ROOT / "App" / "Inc" / "app_servo_jog.h").read_text(encoding="utf-8")
    slew = int(re.search(r"APP_SERVO_JOG_SLEW_US_PER_S\s+(\d+)U", header).group(1))
    timeout_ms = int(re.search(
        r"APP_SERVO_JOG_HOLD_TIMEOUT_MS\s+(\d+)UL", header).group(1))
    # 慢到肉眼可跟、快到全行程（±1000µs）数秒可达；保持窗足够手工测量。
    assert 200 <= slew <= 1000
    assert 60000 <= timeout_ms <= 300000
