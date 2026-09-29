"""`CURRENT PROBE` 引脚接线自检命令的契约（App/Src/app_cmd_currprobe.c）。

用 host gcc 真编译该模块，只替换它依赖的边界符号（QueueText / IsArmed /
BSP 两个探测入口 / osDelay），沿用 tests/test_magcal_command_contract.py 的
"stub the boundary, run the real module" 写法。

钉住的性质里最要紧的一条是 **RESTORE 必须总是发生**：中途任何一步失败而不恢复，
PC1 会永久停在数字输入上，电流采样从此彻底不工作——比这次要诊断的故障更糟。
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

FAKE_MAIN_H = r"""
#ifndef FAKE_MAIN_H
#define FAKE_MAIN_H
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
#endif
"""

FAKE_CMSIS_OS2_H = r"""
#ifndef FAKE_CMSIS_OS2_H
#define FAKE_CMSIS_OS2_H
#include <stdint.h>
typedef void *osSemaphoreId_t;
typedef void *osMessageQueueId_t;
typedef void *osMutexId_t;
typedef void *osThreadId_t;
typedef int osStatus_t;
/* 记录每次 osDelay 的毫秒数：等待时长本身是判据的一部分（等不够会把浮空
   误判成已驱动），所以它必须可观测，不能只当成"随便睡一下"。 */
extern unsigned int fake_delay_calls;
extern unsigned int fake_delay_total_ms;
static inline osStatus_t osDelay(uint32_t ms)
{
    fake_delay_calls++;
    fake_delay_total_ms += (unsigned int)ms;
    return 0;
}
#endif
"""

HARNESS = r"""
#include "app_control.h"
#include "app_control_internal.h"
#include "bsp_current.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

unsigned int fake_delay_calls;
unsigned int fake_delay_total_ms;

/* ---- 被替换的边界 ---- */
static char last_line[256];
void APP_Control_QueueText(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(last_line, sizeof(last_line), fmt, args);
    va_end(args);
}

static uint8_t fake_armed;
uint8_t APP_Stabilizer_IsArmed(void) { return fake_armed; }

/* 探测模式调用序列，用来断言顺序与"总是恢复" */
static BSP_CurrentPinProbeMode mode_log[8];
static uint32_t mode_count;
static uint8_t fake_level_pullup, fake_level_pulldown;
static BSP_CurrentPinProbeMode current_mode;
static int fail_on_mode; /* -1 = 不失败 */

BSP_CurrentStatus BSP_Current_SetPinProbeMode(BSP_CurrentPinProbeMode mode)
{
    if (mode_count < 8U) { mode_log[mode_count++] = mode; }
    current_mode = mode;
    if ((int)mode == fail_on_mode) { return BSP_CURRENT_ERROR; }
    return BSP_CURRENT_OK;
}

uint8_t BSP_Current_ReadPinLevel(void)
{
    if (current_mode == BSP_CURRENT_PIN_PROBE_PULLUP) { return fake_level_pullup; }
    if (current_mode == BSP_CURRENT_PIN_PROBE_PULLDOWN) { return fake_level_pulldown; }
    return 0U;
}

/* `CURRENT SEQ` 的取证入口；这里只验命令层的编排，真序列行为归 BSP 自己的测试。 */
static uint32_t fake_seq[BSP_CURRENT_SEQ_BURST_MAX];
static uint32_t fake_seq_done;
static BSP_CurrentStatus fake_seq_status = BSP_CURRENT_OK;
static uint32_t seq_calls;
BSP_CurrentStatus BSP_Current_SeqBurst(uint32_t *out, uint32_t count,
                                       uint32_t *out_done)
{
    uint32_t i;
    seq_calls++;
    if ((out == NULL) || (out_done == NULL) || (count == 0U)) {
        return BSP_CURRENT_ERROR;
    }
    for (i = 0U; (i < fake_seq_done) && (i < count); ++i) { out[i] = fake_seq[i]; }
    *out_done = (fake_seq_done < count) ? fake_seq_done : count;
    return fake_seq_status;
}

static void reset(void)
{
    mode_count = 0U; fail_on_mode = -1; fake_armed = 0U;
    fake_delay_calls = 0U; fake_delay_total_ms = 0U;
    last_line[0] = '\0';
}

#define CHECK(cond, code) do { if (!(cond)) { \
    fprintf(stderr, "check %d failed at line %d: %s\n", (code), __LINE__, #cond); \
    return (code); } } while (0)

static uint8_t call(const char *a, const char *b)
{
    char t0[32], t1[32];
    char *tokens[2] = { t0, t1 };
    strncpy(t0, a, sizeof(t0) - 1); t0[sizeof(t0) - 1] = '\0';
    strncpy(t1, b, sizeof(t1) - 1); t1[sizeof(t1) - 1] = '\0';
    return app_control_handle_currprobe(tokens, 2U);
}

int main(void)
{
    /* 1. 不是自己的命令就不认领，必须让链上后面的处理函数有机会。 */
    reset();
    CHECK(call("MAGCAL", "PROBE") == 0U, 1);
    CHECK(call("CURRENT", "STATUS") == 0U, 2);
    CHECK(mode_count == 0U, 3);

    /* 2. 解锁状态下拒绝，且**一次都不碰引脚模式**。 */
    reset();
    fake_armed = 1U;
    CHECK(call("CURRENT", "PROBE") == 1U, 10);
    CHECK(mode_count == 0U, 11);
    CHECK(strstr(last_line, "reason=armed") != NULL, 12);

    /* 3. 正常一轮：上拉 -> 下拉 -> 恢复，顺序固定；两次电平不同 = 浮空。 */
    reset();
    fake_level_pullup = 1U; fake_level_pulldown = 0U;
    CHECK(call("CURRENT", "PROBE") == 1U, 20);
    CHECK(mode_count == 3U, 21);
    CHECK(mode_log[0] == BSP_CURRENT_PIN_PROBE_PULLUP, 22);
    CHECK(mode_log[1] == BSP_CURRENT_PIN_PROBE_PULLDOWN, 23);
    CHECK(mode_log[2] == BSP_CURRENT_PIN_PROBE_RESTORE, 24);
    CHECK(strstr(last_line, "verdict=floating") != NULL, 25);
    /* 每种拉电阻状态都要等，且等待时长必须出现在回包里供人工判断。 */
    CHECK(fake_delay_calls == 2U, 26);
    CHECK(strstr(last_line, "settle_ms=") != NULL, 27);

    /* 4. 两次电平相同 = 被低阻源驱动，不能报成浮空。 */
    reset();
    fake_level_pullup = 0U; fake_level_pulldown = 0U;
    CHECK(call("CURRENT", "PROBE") == 1U, 30);
    CHECK(strstr(last_line, "verdict=driven_low") != NULL, 31);

    reset();
    fake_level_pullup = 1U; fake_level_pulldown = 1U;
    CHECK(call("CURRENT", "PROBE") == 1U, 35);
    CHECK(strstr(last_line, "verdict=driven_high") != NULL, 36);

    /* 5. 上拉那步就失败：仍然必须恢复，否则 PC1 永久停在数字输入上。 */
    reset();
    fail_on_mode = (int)BSP_CURRENT_PIN_PROBE_PULLUP;
    CHECK(call("CURRENT", "PROBE") == 1U, 40);
    CHECK(mode_count >= 2U, 41);
    CHECK(mode_log[mode_count - 1U] == BSP_CURRENT_PIN_PROBE_RESTORE, 42);
    CHECK(strstr(last_line, "state=failed") != NULL, 43);

    /* 5b. `CURRENT SEQ` 认领自己的命令字，解锁时拒绝，且把每个原始值都报出来。 */
    reset();
    fake_seq_done = 4U;
    fake_seq[0] = 5U; fake_seq[1] = 11002U; fake_seq[2] = 1226U; fake_seq[3] = 11003U;
    seq_calls = 0U;
    CHECK(call("CURRENT", "SEQ") == 1U, 90);
    CHECK(seq_calls == 1U, 91);
    /* 四个值一个都不能少——少报一个就看不出奇偶位的差别，整条取证就废了。 */
    CHECK(strstr(last_line, "5,11002,1226,11003") != NULL, 92);
    CHECK(strstr(last_line, "n=4") != NULL, 93);
    /* 对位说明必须跟着一起发，否则读日志的人无法判断哪个是电流。 */
    CHECK(strstr(last_line, "order=rank1_first_current") != NULL, 94);

    reset();
    fake_armed = 1U; seq_calls = 0U;
    CHECK(call("CURRENT", "SEQ") == 1U, 95);
    CHECK(seq_calls == 0U, 96);
    CHECK(strstr(last_line, "reason=armed") != NULL, 97);

    /* 6. 下拉那步失败：同样必须恢复。 */
    reset();
    fail_on_mode = (int)BSP_CURRENT_PIN_PROBE_PULLDOWN;
    CHECK(call("CURRENT", "PROBE") == 1U, 50);
    CHECK(mode_log[mode_count - 1U] == BSP_CURRENT_PIN_PROBE_RESTORE, 51);
    CHECK(strstr(last_line, "state=failed") != NULL, 52);

    return 0;
}
"""


def test_currprobe_command_contract(tmp_path: Path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    fakes = tmp_path / "fakes"
    fakes.mkdir()
    (fakes / "main.h").write_text(FAKE_MAIN_H, encoding="utf-8")
    (fakes / "cmsis_os2.h").write_text(FAKE_CMSIS_OS2_H, encoding="utf-8")

    harness = tmp_path / "currprobe_harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "currprobe_harness.exe"

    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Werror",
            "-o",
            str(executable),
            str(harness),
            str(ROOT / "App" / "Src" / "app_cmd_currprobe.c"),
            f"-I{fakes}",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            f"-I{ROOT / 'BSP' / 'Inc'}",
            f"-I{ROOT / 'Services' / 'Inc'}",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


def test_the_probe_is_refused_while_armed_in_source() -> None:
    """解锁门是安全判据，不能只靠上面的运行期用例——源码里也要看得见。

    运行期用例用的是替身 IsArmed；如果有人把这个判断整段删掉，替身也就不再被调用，
    用例 2 仍然会因为 mode_count==0 不成立而红。这条是双保险，针对的是"把判断改成
    恒真/恒假"这类改法。
    """
    source = (ROOT / "App" / "Src" / "app_cmd_currprobe.c").read_text(encoding="utf-8")
    assert "APP_Stabilizer_IsArmed()" in source
    assert "reason=armed" in source
