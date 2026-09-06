"""R-F6-2 核心矩阵重导：力坐标系符号常量到底在做什么。

这个模块存在的唯一理由，是仓库里曾经有一句被复述了四遍的论断：

    "此符号同时作用于实测姿态和目标姿态，因此在姿态误差中相消"

它是**错的**，而且不是无害的措辞问题——R-F6-2 工单的验收判据
（"删除四个符号常量……力矩输出浮点容差内一致"）就是照这句话写的，
照着做必然做不出来。

为什么错：姿态误差走的是 SO(3) 的 e_R = 0.5*vee(R_d^T R_a - R_a^T R_d)，
是矩阵乘出来的**非线性**量。把 roll 反号只是改了 Rz(psi)Ry(theta)Rx(phi)
里 Rx 一个因子的参数，这不是相似变换（只有三个角同时反号才是），所以它在
e_R 里不消失，还会串到 pitch/yaw 通道上去。

下面的 test_force_frame_roll_sign_is_load_bearing 用真控制器把这件事变成
可执行证据：编译两份真 drv_coax_ctrl.c，只差那一个常量，同一组姿态输入
比舵机输出。谁再想"反正它会相消"就删掉常量，这条会红。

本模块只钉"这个符号在做实事"，**不**主张当前取值对不对——绝对物理方向的
最后一环是运行时 ServoCalibration 的 pulse_sign[]，主机上没有，只能拆桨实测
（R-F6-5）。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
CTRL_SOURCE = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
SENSOR = ROOT / "App" / "Src" / "app_sensor.c"

ROLL_SIGN_DEFINE = "#define DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN  (-1.0f)"

# drv_coax_ctrl.c 自己的舵机角容差。差异超过它就不是浮点噪声，是真的换了指令。
SERVO_ANGLE_TOL_RAD = 8.0e-4


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


AB_HARNESS = r"""
#include "drv_coax_ctrl.h"
#include "drv_airframe_model.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Output out;
    static const float grid[5] = { -0.30f, -0.12f, 0.0f, 0.12f, 0.30f };

    DRV_COAX_CTRL_Init();

    for (int a = 0; a < 5; ++a) {
      for (int b = 0; b < 5; ++b) {
        for (int c = 0; c < 5; ++c) {
          for (int d = 0; d < 5; ++d) {
            memset(&att, 0, sizeof(att));
            memset(&ref, 0, sizeof(ref));
            ref.direct_attitude_target_valid = 1U;
            ref.manual_total_force_valid = 1U;
            ref.manual_total_force_n =
                DRV_AIRFRAME_MASS_KG * DRV_AIRFRAME_GRAVITY_M_S2;

            att.roll_rad  = grid[a];
            att.pitch_rad = grid[b];
            att.yaw_rad   = grid[c];
            ref.target_roll_rad  = grid[d];
            ref.target_pitch_rad = grid[(d + 2) % 5];
            /* Yaw reference is deliberately one step off the measured yaw:
             * in flight the yaw error is never exactly zero, and that is the
             * condition under which the sign constant cross-couples. */
            ref.yaw_rad = grid[(c + 1) % 5];

            DRV_COAX_CTRL_Run(&att, &ref, &out);
            printf("%.9f %.9f\n", (double)out.alpha_rad, (double)out.beta_rad);
          }
        }
      }
    }
    return 0;
}
"""


def _build_and_run(tmp_path: Path, tag: str, source_text: str) -> list[tuple[float, float]]:
    gcc = shutil.which("gcc")
    assert gcc is not None

    work = tmp_path / tag
    work.mkdir()
    ctrl = work / "drv_coax_ctrl.c"
    ctrl.write_text(source_text, encoding="utf-8")

    stub = work / "stub"
    stub.mkdir()
    (stub / "bsp_pwm.h").write_text(
        "#ifndef BSP_PWM_H\n#define BSP_PWM_H\n"
        "#define BSP_PWM_ESC_MIN_US 1000U\n"
        "#define BSP_PWM_ESC_MAX_US 2000U\n#endif\n",
        encoding="ascii",
    )

    harness = work / "ab_harness.c"
    harness.write_text(AB_HARNESS, encoding="ascii")
    executable = work / "ab_harness.exe"

    subprocess.run(
        [gcc, "-std=c11", "-O1",
         f"-I{stub}", f"-I{ROOT / 'Driver' / 'Inc'}",
         str(ctrl),
         str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
         str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)

    result = subprocess.run([str(executable)], check=True,
                            capture_output=True, text=True)
    rows = [tuple(float(v) for v in line.split())
            for line in result.stdout.strip().splitlines()]
    assert len(rows) == 625
    return rows


def test_force_frame_roll_sign_is_load_bearing(tmp_path: Path) -> None:
    """删掉 FORCE_FRAME_ROLL_SIGN 会改舵机指令，不是"在误差里相消"。

    实测（2026-09-06，625 组直接姿态模式 + 悬停推力）：beta 最大差
    67.5 mrad（约等于整个翻号），alpha 最大差 20.4 mrad，都远超舵机角容差
    0.8 mrad；519/625 组的输出符号发生翻转。

    所以 R-F6-2 不可能"删掉常量而力矩不变"——迁移必须重导矩阵，并且改动后
    存档的 ServoCalibration 极性随之失效，必须重新拆桨实测。
    """
    if shutil.which("gcc") is None:
        pytest.skip("host gcc is unavailable")

    source = read(CTRL_SOURCE)
    assert ROLL_SIGN_DEFINE in source, "力坐标系 roll 符号常量已改名或改值"

    base = _build_and_run(tmp_path, "base", source)
    patched = _build_and_run(
        tmp_path, "no_roll_sign",
        source.replace(
            ROLL_SIGN_DEFINE,
            "#define DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN  (1.0f)"))

    worst = max(max(abs(a0 - a1), abs(b0 - b1))
                for (a0, b0), (a1, b1) in zip(base, patched))
    flips = sum(1 for (a0, b0), (a1, b1) in zip(base, patched)
                if (b0 * b1 < 0.0) or (a0 * a1 < 0.0))

    assert worst > SERVO_ANGLE_TOL_RAD, (
        "力坐标系 roll 符号看起来在姿态误差里相消了。若这是有意的重导，"
        "请连同本文件的论证一并更新；若不是，说明符号链被改坏了。"
    )
    assert flips > len(base) // 2, (
        f"只有 {flips}/{len(base)} 组输出翻转，与 2026-09-06 实测的 519 组不符"
    )


def test_the_cancellation_claim_is_not_reasserted() -> None:
    """那句错话曾在四处被复述，别让它回来。"""
    for path in (CTRL_SOURCE,
                 ROOT / "tests" / "test_coax_sign_convention.py",
                 ROOT / "tests" / "test_flu_seam3_controller_frame.py"):
        text = read(path)
        assert "在姿态误差中相消" not in text, path
        assert "cancel in the attitude error" not in text, path
        assert "cancels in the attitude error" not in text, path


def test_nothing_compensates_the_seam01_pitch_flip() -> None:
    """seam 0/1 迁到 FLU 把 pitch 口径翻了，seam 3 没有任何补偿。

    姿态融合按 `flu_active` 在 NED / NWU 之间切换（见
    tests/test_flu_seam1_estimator_frame.py 的可执行断言）：
      legacy(NED/FRD)：pitch > 0 = 机头上仰
      FLU(NWU)       ：pitch > 0 = 机头下俯
    roll 在两种口径下都是"右翼下沉为正"，所以只有 pitch 翻了。

    而 `FORCE_FRAME_PITCH_SIGN` 仍是 +1，且全仓库唯一读
    `APP_Sensor_IsFluOrientationActive()` 的地方是解锁互锁本身。也就是说，
    半迁移状态下控制器拿到的 pitch 就是反的——**这正是那道互锁存在的原因，
    它必须留着**，直到 R-F6-2 把矩阵重导完。
    """
    assert "#define DRV_COAX_CTRL_FORCE_FRAME_PITCH_SIGN (1.0f)" in read(CTRL_SOURCE)

    # 姿态从融合到控制器全程不翻号：口径完全由融合的 convention 决定。
    stabilizer = read(STABILIZER)
    assert "ctx->pitch = ctx->attitude_fusion.pitch_deg;" in stabilizer
    assert "ctx->pitch_control = ctx->pitch - ctx->pitch_zero;" in stabilizer
    assert ("frame->attitude.pitch_rad = ctx->pitch_control * STABILIZER_DEG_TO_RAD;"
            in stabilizer)

    # 唯一的 FLU 感知点是互锁，不是控制律里的补偿分支。
    users = [path for path in (ROOT / "App" / "Src").glob("*.c")
             if "APP_Sensor_IsFluOrientationActive()" in read(path)]
    assert users == [STABILIZER], (
        f"多了 FLU 条件分支：{[p.name for p in users]}；"
        "运行时按朝向切控制律口径会让两种朝向都缺乏证据，见 frame-migration 模式"
    )
    assert stabilizer.count("APP_Sensor_IsFluOrientationActive()") == 1
    assert "DRV_FRAME_RUNTIME_MIGRATION_COMPLETE == 0U" in stabilizer
    assert "APP_Sensor_IsFluOrientationActive(void)" in read(SENSOR)
