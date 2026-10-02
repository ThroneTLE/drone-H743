"""单桨可用最大推力改为"推力映射按当前电压换 ESC 满行程"（2026-10-01，作者："你先修模型"）。

手扶带桨台架：作者"我控制YAW基本没感觉到反馈"。日志里实现偏航力矩一直卡在 0.0237 N·m——
上限按写死的单桨 10.2 N 算（悬停 14.25 N 时 0.031 N·m），而推力表满油门两桨合计约 16 N@12 V
（单桨约 8 N），真实上限约 0.01 N·m；推力早已顶满，分配器还以为有 6 N 差速余量。
本文件钉住：五处用法都改走 coax_ctrl_single_max_thrust_n()；映射给出合理值时用它（封顶 10.2），
映射缺失/结果不合理时退回 10.2（旧行为）；偏航上限随之按真实余量算。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE, PROP_MAP_SOURCE

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


def test_all_uses_go_through_runtime_single_max() -> None:
    src = read("Driver/Src/drv_coax_ctrl.c")
    # 只剩助手里取封顶值这一处直接读参数。
    assert src.count("coax_ctrl_params.motor_single_max_thrust_n") == 1
    helper = src.split("static float coax_ctrl_single_max_thrust_n(void)")[1].split("\n}\n")[0]
    assert "map->total_thrust_for_pulse(BSP_PWM_ESC_MAX_US)" in helper
    assert "DRV_COAX_CTRL_SINGLE_MAX_THRUST_MIN_N" in helper and "fminf(half, cap)" in helper
    assert "const float span = (ku + kl) * coax_ctrl_single_max_thrust_n();" in src
    assert "const float single_max_n = coax_ctrl_single_max_thrust_n();" in src
    assert "float DRV_COAX_CTRL_SingleMaxThrustN(void);" in read("Driver/Inc/drv_coax_ctrl.h")
    assert "tmax_mn=%ld hover_mn=%ld" in read("App/Src/app_cmd_thrmode.c")


HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"
#include <math.h>
#include <stdio.h>

static float fake_total = 16.0f;
static uint16_t fake_pulse(float t) { (void)t; return 1500U; }
static float fake_total_for_pulse(uint16_t p) { (void)p; return fake_total; }

int main(void)
{
    static const DRV_COAX_CTRL_ThrustMap map = { fake_pulse, fake_total_for_pulse, NULL };
    airframe_load_reference();
    DRV_COAX_CTRL_ResetParams();
    DRV_COAX_CTRL_SetThrustMap(NULL);
    printf("%.3f ", DRV_COAX_CTRL_SingleMaxThrustN());          /* 无映射 -> 10.2 */
    DRV_COAX_CTRL_SetThrustMap(&map);
    printf("%.3f ", DRV_COAX_CTRL_SingleMaxThrustN());          /* 16 N 合推力 -> 8 */
    printf("%.3f ", DRV_COAX_CTRL_MotorPulseToTotalThrust(2000U)); /* 合推力上限跟着 16 */
    fake_total = 25.0f;
    printf("%.3f ", DRV_COAX_CTRL_SingleMaxThrustN());          /* 封顶 10.2 */
    fake_total = 0.0f;
    printf("%.3f ", DRV_COAX_CTRL_SingleMaxThrustN());          /* 电压未知 -> 10.2 */
    fake_total = NAN;
    printf("%.3f\n", DRV_COAX_CTRL_SingleMaxThrustN());         /* 非法 -> 10.2 */
    return 0;
}
"""


def test_runtime_single_max_from_thrust_map(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")
    stub = tmp_path / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text("#ifndef BSP_PWM_H\n#define BSP_PWM_H\n#define BSP_PWM_ESC_MIN_US 1000U\n"
                                    "#define BSP_PWM_ESC_MAX_US 2000U\n#endif\n", encoding="ascii")
    h = tmp_path / "h.c"
    h.write_text(HARNESS, encoding="utf-8")
    exe = tmp_path / "h.exe"
    subprocess.run([gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", f"-I{stub}", f"-I{ROOT / 'Driver' / 'Inc'}",
                    str(AIRFRAME_SOURCE), str(PROP_MAP_SOURCE), str(ROOT / "Driver/Src/drv_coax_ctrl.c"),
                    str(ROOT / "Driver/Src/drv_position_control.c"), str(ROOT / "Driver/Src/drv_attitude_control.c"),
                    str(ROOT / "Driver/Src/drv_rate_control.c"), str(h), "-lm", "-o", str(exe)],
                   check=True, capture_output=True, text=True)
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout.split()
    assert out == ["10.200", "8.000", "16.000", "10.200", "10.200", "10.200"], out
