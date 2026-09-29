"""舵机 PWM 帧率运行时切换（SERVOHZ，2026-09-28）：50 Hz 与 333 Hz 台架 A/B。

模型（data/analysis/sysid-rig-params/2026-09-28）：舵机指令等下一帧的延迟 50 Hz 平均约 10 ms，
333 Hz 约 1.5 ms；少 10 ms 纯延迟，鲁棒反馈穿越约 0.95 → 1.05 Hz。这里钉住：
* BSP：范围 [50, 333]、先开 ARR 预装载再写 ARR（不出 65 ms 长帧）、最快帧仍装得下最长脉冲；
* 命令：查询/切换/拒绝（解锁、辨识在跑、越界、BSP 失败），只在 RAM；
* 溯源：光杆辨识开跑的 SYSID BACKLASH 行尾带 servo_hz。
"""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
BSP_H = ROOT / "BSP/Inc/bsp_pwm.h"
BSP_C = ROOT / "BSP/Src/bsp_pwm.c"
CMD_C = ROOT / "App/Src/app_cmd_servohz.c"


def test_the_bsp_switch_preloads_the_reload_before_writing_it():
    header = BSP_H.read_text(encoding="utf-8")
    source = BSP_C.read_text(encoding="utf-8")
    assert "#define BSP_PWM_SERVO_FRAME_HZ_MIN  50U" in header
    assert "#define BSP_PWM_SERVO_FRAME_HZ_MAX 333U" in header
    # 2026-09-28 作者定为上电 333 Hz：杆上 A/B −45° 带载纯延迟 30.2 → 22.4 ms、闭环滞后 46 → 38 ms，
    # 2–10 Hz 抖动不变。换模拟舵机仍可经 SERVOHZ 切回 50（下面的命令用例）。
    assert "#define BSP_PWM_SERVO_FRAME_HZ     333U" in header, "上电 333 Hz"
    body = source.split("BSP_PWM_Status BSP_PWM_SetServoFrameHz(uint32_t frame_hz)", 1)[1].split("\n}\n", 1)[0]
    assert body.index("TIM_CR1_ARPE") < body.index("__HAL_TIM_SET_AUTORELOAD(&htim4"), "先开预装载再写 ARR"
    assert "BSP_PWM_INVALID_PARAM" in body and "pwm_started == 0U" in body
    assert "(BSP_PWM_TIMER_TICK_HZ / BSP_PWM_SERVO_FRAME_HZ_MAX) > BSP_PWM_SERVO_MAX_US" in source
    # 333 Hz 帧 3003 us，最长脉冲 2500 us（校准上限）。
    assert 1_000_000 // 333 > int(re.search(r"#define BSP_PWM_SERVO_MAX_US\s+(\d+)U", header).group(1))


def test_the_command_is_wired_and_the_provenance_carries_the_rate():
    fallback = (ROOT / "App/Src/app_cmd_fallback.c").read_text(encoding="utf-8")
    assert "if (APP_ServoHz_Command(tokens, count) != 0U) { return; }" in fallback
    assert "App/Src/app_cmd_servohz.c" in (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    backlash = (ROOT / "App/Src/app_cmd_backlash.c").read_text(encoding="utf-8")
    assert "servo_hz=%lu\\r\\n" in backlash and "BSP_PWM_GetServoFrameHz()" in backlash
    assert "#include \"stm32" not in CMD_C.read_text(encoding="utf-8"), "App 不直接碰 HAL"


HARNESS = r"""
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "app_servo_hz.h"
#include "bsp_pwm.h"
/* 装置显式从 50 起步（上电是 333）：查询 50 → 切到 333 → 切回 50，每步都是真实切换。 */
static unsigned long hz = 50; static int armed, running, fail;
BSP_PWM_Status BSP_PWM_SetServoFrameHz(uint32_t f) {
    if (fail) return BSP_PWM_ERROR;
    if (f < BSP_PWM_SERVO_FRAME_HZ_MIN || f > BSP_PWM_SERVO_FRAME_HZ_MAX) return BSP_PWM_INVALID_PARAM;
    hz = f; return BSP_PWM_OK; }
uint32_t BSP_PWM_GetServoFrameHz(void) { return (uint32_t)hz; }
uint8_t APP_Stabilizer_IsArmed(void) { return (uint8_t)armed; }
uint8_t APP_SysId_IsRunning(void) { return (uint8_t)running; }
void APP_Control_QueueText(const char *f, ...) { va_list a; va_start(a, f); vprintf(f, a); va_end(a); }
static void run(const char *a, const char *b) {
    char x[16], y[16]; char *t[2]; unsigned n = 1;
    strcpy(x, a); t[0] = x; if (b) { strcpy(y, b); t[1] = y; n = 2; }
    printf("%u|", (unsigned)APP_ServoHz_Command(t, n));
    fflush(stdout);
}
int main(void) {
    run("SERVOHZ", 0); run("SERVOHZ", "?");
    run("SERVOHZ", "333"); run("SERVOHZ", "?");
    run("SERVOHZ", "40"); run("SERVOHZ", "334"); run("SERVOHZ", "3x3");
    armed = 1; run("SERVOHZ", "50"); armed = 0;
    running = 1; run("SERVOHZ", "50"); running = 0;
    fail = 1; run("SERVOHZ", "200"); fail = 0;
    run("SERVOHZ", "50"); run("RPMNOTCH", "?");
    return 0;
}
"""


def test_the_command_switches_only_when_disarmed_and_idle(tmp_path):
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub = tmp_path / "stub"
    stub.mkdir()
    # 只要各头文件里的声明；装置自己给实现。
    (stub / "cmsis_os2.h").write_text("typedef void *osSemaphoreId_t; typedef void *osMessageQueueId_t;\n")
    (tmp_path / "harness.c").write_text(HARNESS, encoding="utf-8")
    exe = tmp_path / "servohz.exe"
    subprocess.run([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{stub}",
                    *[f"-I{ROOT / d}" for d in ("App/Inc", "Driver/Inc", "BSP/Inc", "Services/Inc")],
                    str(CMD_C), str(tmp_path / "harness.c"), "-o", str(exe)],
                   check=True, capture_output=True, text=True)
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout
    out = " ".join(out.split())          # Windows 文本模式会把 \r\n 变成两个换行：只比内容
    assert out.count("SERVOHZ hz=50 frame_us=20000") == 3, out
    assert "SERVOHZ hz=333 frame_us=3003 1|SERVOHZ hz=333 frame_us=3003" in out
    assert out.count("reason=range") == 3
    assert "reason=armed" in out and "reason=sysid" in out and "reason=pwm" in out
    assert out.rstrip().endswith("0|"), "别家的命令不认领"
