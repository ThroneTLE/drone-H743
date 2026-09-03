"""R-S7-7：PWM 调试页即时移动通路 与 标定页慢速点动 的解耦契约。

缺陷背景（R-S7-4 遗留）：PWM 模式下 servo_debug.py 的「移动此舵机」下发裸
`SERVO JOG <ch> <us>`，而该命令的斜坡速率是 APP_SERVO_JOG_SLEW_US_PER_S=500µs/s
——那是 M4 阶段专为 mechanical.py 拆桨标定点动设计的安全慢速（1000µs 行程要 2s）。
调试页因此从总线模式 `SERVO MOVE`（用户可设小 time）的近瞬间响应退化成肉眼可见
的爬行。

修复方式：斜坡速率改为随请求走的每通道字段，不是全局参数。
- `SERVO JOG ch us`      → APP_ServoJog_Request，500µs/s，mechanical.py 语义冻结；
- `SERVO JOG ch us NOW`  → APP_ServoJog_RequestImmediate，一拍到位，调试页专用。
两条路径只共用「接管 / 保持 / 让位 / 超时」这套仲裁，不共用速率。

为什么原测试没挡住：R-S7-4 的断言全是存在性/隔离性的（PWM 下发的是 JOG、总线
专属子命令一条不发），没有任何一条判据描述「调试页移动应与总线模式同量级」。
响应速度这类交互体验判据不会自己长出来，必须显式写成契约——本文件即是。
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tools.panel_lib.pages.servo_debug import ServoDebugPageMixin

ROOT = Path(__file__).resolve().parents[1]
JOG_HEADER = (ROOT / "App" / "Inc" / "app_servo_jog.h").read_text(encoding="utf-8")
JOG_SOURCE = (ROOT / "App" / "Src" / "app_servo_jog.c").read_text(encoding="utf-8")
MECHANICAL_SOURCE = (
    ROOT / "tools" / "panel_lib" / "pages" / "mechanical.py").read_text(encoding="utf-8")

PROTO_REQ_SERVO_MOVE = 0x1010
PROTO_REQ_SERVO_MOVE_ALL = 0x1011


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class DummyPanel(ServoDebugPageMixin):
    """两个槽位的最小替身，覆盖「同时移动两路」。"""

    def __init__(self, output_type: str, pulses=(1600, 1450)):
        self.servo_type_active_var = Value(output_type)
        self.servo_type_var = Value(output_type)
        self.servo_widgets = [
            {
                "id": Value(index + 1),
                "pulse": Value(pulse),
                "time": Value(500),
                "mode": Value(1),
                "enabled": Value(1),
                "new_id": Value(index + 1),
                "baud": Value(4),
            }
            for index, pulse in enumerate(pulses)
        ]
        self.sent = []

    def _send_proto(self, function, label, payload="", expect_reply=True):
        self.sent.append((function, label, payload, expect_reply))


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


def _python_function_body(source: str, name: str) -> str:
    start = source.index(f"    def {name}(")
    end = source.index("\n    def ", start + 1)
    return source[start:end]


# --- 上位机：调试页两个动作都必须走即时通路 -----------------------------------


def test_pwm_debug_move_uses_immediate_path_not_the_calibration_slew() -> None:
    pwm = DummyPanel("pwm")
    pwm._servo_move(0)
    assert pwm.sent == [
        (PROTO_REQ_SERVO_MOVE, "SERVO JOG 0 1600 NOW", "SERVO JOG 0 1600 NOW", True),
    ]
    # 裸 JOG（= 标定页 500µs/s 慢速）绝不能再出现在调试页下发的报文里。
    assert all(payload != "SERVO JOG 0 1600" for _, _, payload, _ in pwm.sent)


def test_pwm_debug_move_all_sends_immediate_for_every_slot() -> None:
    pwm = DummyPanel("pwm", pulses=(1600, 1450))
    pwm._servo_move_all()
    assert pwm.sent == [
        (PROTO_REQ_SERVO_MOVE, "SERVO JOG 0 1600 NOW", "SERVO JOG 0 1600 NOW", True),
        (PROTO_REQ_SERVO_MOVE, "SERVO JOG 1 1450 NOW", "SERVO JOG 1 1450 NOW", True),
    ]
    # PWM 下不得出现总线专属的 MOVEALL。
    assert all(function != PROTO_REQ_SERVO_MOVE_ALL for function, _, _, _ in pwm.sent)


def test_bus_mode_move_and_move_all_are_unchanged() -> None:
    bus = DummyPanel("bus", pulses=(1600, 1450))
    bus._servo_move(0)
    bus._servo_move(1)
    bus._servo_move_all()
    assert bus.sent == [
        (PROTO_REQ_SERVO_MOVE, "SERVO MOVE 0 1600 500", "SERVO MOVE 0 1600 500", True),
        (PROTO_REQ_SERVO_MOVE, "SERVO MOVE 1 1450 500", "SERVO MOVE 1 1450 500", True),
        (PROTO_REQ_SERVO_MOVE_ALL, "SERVO MOVEALL", "", True),
    ]


# --- 标定页：JOG 调用点逐字节不变 ---------------------------------------------


def test_mechanical_page_jog_call_sites_are_byte_identical() -> None:
    move = _python_function_body(MECHANICAL_SOURCE, "_mechanical_move")
    nudge = _python_function_body(MECHANICAL_SOURCE, "_mechanical_nudge_center")
    stop = _python_function_body(MECHANICAL_SOURCE, "_mechanical_jog_stop")

    assert 'payload = f"SERVO JOG {index} {target}"' in move
    assert 'payload = f"SERVO JOG {index} {center}"' in nudge
    assert 'self._send_proto(PROTO_REQ_SERVO_MOVE, "SERVO JOG STOP", "SERVO JOG STOP")' in stop

    # 标定页任何一处都不得沾上即时通路：慢速是拆桨标定的安全设计，不是缺陷。
    for line in MECHANICAL_SOURCE.splitlines():
        if "SERVO JOG" in line:
            assert "NOW" not in line, line
    assert "500 µs/s" in move  # 状态栏仍然如实告知用户是慢速斜坡


# --- 固件：速率随请求走，两条路径不共用参数 -----------------------------------


def test_calibration_slew_constant_is_untouched_and_immediate_is_separate() -> None:
    slew = int(re.search(r"APP_SERVO_JOG_SLEW_US_PER_S\s+(\d+)U", JOG_HEADER).group(1))
    immediate = int(
        re.search(r"APP_SERVO_JOG_SLEW_IMMEDIATE\s+(\d+)U", JOG_HEADER).group(1))
    assert slew == 500
    assert immediate == 0


def test_ramp_rate_comes_from_the_request_not_from_a_shared_constant() -> None:
    apply_body = _function_body(JOG_SOURCE, "static void servo_jog_channel_apply(")
    # 控制环只读通道字段；常量只在请求构造处出现，两条通路因此无法互相带跑。
    assert "APP_SERVO_JOG_SLEW_US_PER_S" not in apply_body
    assert "channel->slew_us_per_s" in apply_body

    default_request = _function_body(JOG_SOURCE, "uint8_t APP_ServoJog_Request(")
    immediate_request = _function_body(
        JOG_SOURCE, "uint8_t APP_ServoJog_RequestImmediate(")
    assert "APP_SERVO_JOG_SLEW_US_PER_S" in default_request
    assert "APP_SERVO_JOG_SLEW_IMMEDIATE" in immediate_request

    # 即时通路复用同一套仲裁，不得绕开让位/超时。
    for signature in ("void APP_ServoJog_Apply(", "void APP_ServoJog_ForceRelease("):
        assert "APP_Control_QueueText(" not in _function_body(JOG_SOURCE, signature)


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

static char queue_text_last[256];

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;

    va_start(args, format);
    (void)vsnprintf(queue_text_last, sizeof(queue_text_last), format, args);
    va_end(args);
}

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

    char t_servo[] = "SERVO";
    char t_jog[] = "JOG";
    char t_ch[] = "0";
    char t_us[] = "1850";
    char t_now[] = "NOW";
    char t_junk[] = "SOON";
    char *slow_tokens[4] = { t_servo, t_jog, t_ch, t_us };
    char *now_tokens[5] = { t_servo, t_jog, t_ch, t_us, t_now };
    char *junk_tokens[5] = { t_servo, t_jog, t_ch, t_us, t_junk };

    APP_ServoJog_Init();

    /* 1. 即时请求：一拍到位（对照慢速的第一拍只走 1µs）。 */
    CHECK(APP_ServoJog_RequestImmediate(0U, 1850U, now_ms) == 1U, 10);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1850U, 11);
    CHECK(beta == 1520U, 12);           /* 未请求通道仍然透传 */
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1850U, 13);          /* 保持：不被流式目标拉回 */

    /* 2. 即时通路重定目标仍是一拍到位，且方向双向都成立。 */
    CHECK(APP_ServoJog_RequestImmediate(0U, 1200U, now_ms) == 1U, 20);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1200U, 21);

    /* 3. 速率随请求走：同一通道换回慢速请求，立刻恢复 500µs/s 斜坡。 */
    APP_ServoJog_ReleaseAll();
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1500U, 30);
    CHECK(APP_ServoJog_Request(0U, 1850U, now_ms) == 1U, 31);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1501U, 32);          /* 标定页语义未被即时通路污染 */

    /* 4. 即时通路同样受更高优先级让位约束（解锁）。 */
    APP_ServoJog_ReleaseAll();
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(APP_ServoJog_RequestImmediate(1U, 1700U, now_ms) == 1U, 40);
    tick(&now_ms, "armed", 1500U, 1520U, &alpha, &beta);
    CHECK(beta == 1520U, 41);
    CHECK(APP_ServoJog_IsActive() == 0U, 42);
    CHECK(APP_ServoJog_TakeNotice(notice, sizeof(notice)) != 0U, 43);
    CHECK(strstr(notice, "armed") != NULL, 44);

    /* 5. 即时通路同样受保持超时约束，不会永久霸占输出。 */
    CHECK(APP_ServoJog_RequestImmediate(0U, 1700U, now_ms) == 1U, 50);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1700U, 51);
    now_ms += APP_SERVO_JOG_HOLD_TIMEOUT_MS;
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1500U, 52);
    CHECK(APP_ServoJog_IsActive() == 0U, 53);
    CHECK(APP_ServoJog_TakeNotice(notice, sizeof(notice)) != 0U, 54);
    CHECK(strstr(notice, "timeout") != NULL, 55);

    /* 6. 边界校验与慢速路径一致（通道/脉宽越界一律拒绝）。 */
    CHECK(APP_ServoJog_RequestImmediate(2U, 1500U, now_ms) == 0U, 60);
    CHECK(APP_ServoJog_RequestImmediate(0U, 499U, now_ms) == 0U, 61);
    CHECK(APP_ServoJog_RequestImmediate(0U, 2501U, now_ms) == 0U, 62);
    CHECK(APP_ServoJog_IsActive() == 0U, 63);

    /* 7. 命令面：4 词 = 慢速（标定页原样），5 词 NOW = 即时（调试页）。 */
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    queue_text_last[0] = '\0';
    APP_ServoJog_HandleCommand(slow_tokens, 4U, now_ms);
    CHECK(strstr(queue_text_last, "OK servo_jog ch=0 target=1850 slew=500") != NULL, 70);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1501U, 71);          /* mechanical.py 报文行为逐字节不变 */

    APP_ServoJog_ReleaseAll();
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    queue_text_last[0] = '\0';
    APP_ServoJog_HandleCommand(now_tokens, 5U, now_ms);
    CHECK(strstr(queue_text_last, "slew=immediate") != NULL, 72);
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    CHECK(alpha == 1850U, 73);          /* 调试页一拍到位 */

    /* 8. 未知第 5 词不得被当成慢速静默执行。 */
    APP_ServoJog_ReleaseAll();
    tick(&now_ms, NULL, 1500U, 1520U, &alpha, &beta);
    queue_text_last[0] = '\0';
    APP_ServoJog_HandleCommand(junk_tokens, 5U, now_ms);
    CHECK(strstr(queue_text_last, "ERR usage SERVO JOG") != NULL, 80);
    CHECK(APP_ServoJog_IsActive() == 0U, 81);

    return 0;
}
"""


def test_immediate_jog_runtime_behaviour(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    harness_path = tmp_path / "servo_jog_immediate_harness.c"
    harness_path.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "servo_jog_immediate_harness.exe"

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
