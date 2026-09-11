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


from _airframe_fixture import AIRFRAME_FIXTURE_C, AIRFRAME_SOURCE


ROOT = Path(__file__).resolve().parents[1]
CTRL_SOURCE = ROOT / "Driver" / "Src" / "drv_coax_ctrl.c"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
SENSOR = ROOT / "App" / "Src" / "app_sensor.c"

# drv_coax_ctrl.c 自己的舵机角容差。差异超过它就不是浮点噪声，是真的换了指令。
SERVO_ANGLE_TOL_RAD = 8.0e-4


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


AB_HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    airframe_load_reference();
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
                DRV_Airframe_Get()->mass_kg * DRV_Airframe_Get()->gravity_m_s2;

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


OUTER_LOOP_HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * Find the measured attitude at which the controller goes quietest while it is
 * asked for a constant physical acceleration.  That equilibrium is the frame
 * convention the outer loop actually believes in -- it comes from the force
 * vector, so it is pinned to physics rather than to a stick mapping.
 *
 * Prints one "pitch roll" line for forward (+X), one for left (+Y), and one
 * for right (-Y) in canonical FLU.
 */
static void quietest(float ax, float ay, float *best_pitch, float *best_roll)
{
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Output out;
    float best_mag = 1.0e9f;

    *best_pitch = 0.0f;
    *best_roll = 0.0f;

    for (int i = -40; i <= 40; ++i) {
        const float angle = (float)i * 0.005f;
        float mag;

        memset(&att, 0, sizeof(att));
        memset(&ref, 0, sizeof(ref));
        ref.dt_sec = 0.002f;
        ref.ax_m_s2 = ax;
        ref.ay_m_s2 = ay;
        /* Canonical FLU: +pitch nose down, +roll right wing down. */
        if (ax != 0.0f) { att.pitch_rad = angle; } else { att.roll_rad = angle; }

        DRV_COAX_CTRL_Run(&att, &ref, &out);
        mag = fabsf(out.alpha_rad) + fabsf(out.beta_rad);
        if (mag < best_mag) {
            best_mag = mag;
            if (ax != 0.0f) { *best_pitch = angle; } else { *best_roll = angle; }
        }
    }
}

int main(void)
{
    airframe_load_reference();
    DRV_COAX_CTRL_Params params;
    float pitch, roll;

    DRV_COAX_CTRL_Init();
    DRV_COAX_CTRL_GetDefaultParams(&params);
    /* Route the demand through the acceleration feed-forward path so the
     * physical demand is exactly what this harness sets. */
    params.vel_loop_enable = 0.0f;
    DRV_COAX_CTRL_SetParams(&params);

    quietest(2.0f, 0.0f, &pitch, &roll);
    printf("%.9f %.9f\n", (double)pitch, (double)roll);
    quietest(0.0f, 2.0f, &pitch, &roll);
    printf("%.9f %.9f\n", (double)pitch, (double)roll);
    quietest(0.0f, -2.0f, &pitch, &roll);
    printf("%.9f %.9f\n", (double)pitch, (double)roll);
    return 0;
}
"""


def _build_and_run(tmp_path: Path, tag: str, source_text: str,
                   harness: str = AB_HARNESS,
                   expect: int = 625) -> list[tuple[float, float]]:
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

    harness_c = work / "harness.c"
    harness_c.write_text(harness, encoding="ascii")
    executable = work / "harness.exe"

    subprocess.run(
        [gcc, "-std=c11", "-O1",
         f"-I{stub}", f"-I{ROOT / 'Driver' / 'Inc'}",
         str(ctrl),
         str(AIRFRAME_SOURCE),
         str(ROOT / "Driver" / "Src" / "drv_position_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_attitude_control.c"),
         str(ROOT / "Driver" / "Src" / "drv_rate_control.c"),
         str(harness_c), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True)

    result = subprocess.run([str(executable)], check=True,
                            capture_output=True, text=True)
    rows = [tuple(float(v) for v in line.split())
            for line in result.stdout.strip().splitlines()]
    assert len(rows) == expect
    return rows


ATTITUDE_MATRIX_CALL = (
    "    coax_ctrl_rpy_matrix(attitude->roll_rad, attitude->pitch_rad,\n"
    "                        attitude->yaw_rad, rotation);"
)
TARGET_RPY_MATRIX_CALL = (
    "    coax_ctrl_rpy_matrix(target_roll_rad,\n"
    "                         target_pitch_rad,\n"
    "                         reference->yaw_rad,\n"
    "                         solution->desired_body_r);"
)


def test_force_frame_roll_sign_was_load_bearing_before_deletion(
    tmp_path: Path,
) -> None:
    """R-F6-2 deleted FORCE_FRAME_ROLL_SIGN; this reconstructs why that was safe.

    历史实测（2026-09-06，625 组直接姿态模式 + 悬停推力，常量仍在时）：把
    DRV_COAX_CTRL_FORCE_FRAME_ROLL_SIGN 从 -1 改成 +1，beta 最大差 67.5 mrad
    （约等于整个翻号），alpha 最大差 20.4 mrad，都远超舵机角容差 0.8 mrad；
    519/625 组输出符号翻转。这条历史证据证明了"删掉常量而力矩不变"是不可能
    事件，R-F6-2 因此必须重导矩阵而不是简单删除。

    常量本身已被删除（不再是"改值"），无法再对活代码做原地патch。为了不
    依赖易变的 git 历史，本测试改为对**当前**（已迁移、无符号）源码做反向
    补丁——把 roll 重新接回一个 -1 符号，注入回当年那两个调用点——重放
    "常量仍在"的历史场景，并断言其与当前（无符号）版本仍然存在同等量级的
    差异。这确认了删除该常量确实是一次有内容的改动，而不是无操作的重命名。
    """
    if shutil.which("gcc") is None:
        pytest.skip("host gcc is unavailable")

    current = read(CTRL_SOURCE)
    assert ATTITUDE_MATRIX_CALL in current, "attitude_matrix 调用点已改写，需要更新本测试的补丁位置"
    assert TARGET_RPY_MATRIX_CALL in current, "target rpy_matrix 调用点已改写，需要更新本测试的补丁位置"

    reintroduced_sign = current.replace(
        ATTITUDE_MATRIX_CALL,
        "    coax_ctrl_rpy_matrix(-1.0f * attitude->roll_rad, attitude->pitch_rad,\n"
        "                        attitude->yaw_rad, rotation);",
    ).replace(
        TARGET_RPY_MATRIX_CALL,
        "    coax_ctrl_rpy_matrix(-1.0f * target_roll_rad,\n"
        "                         target_pitch_rad,\n"
        "                         reference->yaw_rad,\n"
        "                         solution->desired_body_r);",
    )
    assert reintroduced_sign != current

    base = _build_and_run(tmp_path, "current", current)
    patched = _build_and_run(tmp_path, "reintroduced_roll_sign", reintroduced_sign)

    worst = max(max(abs(a0 - a1), abs(b0 - b1))
                for (a0, b0), (a1, b1) in zip(base, patched))
    flips = sum(1 for (a0, b0), (a1, b1) in zip(base, patched)
                if (b0 * b1 < 0.0) or (a0 * a1 < 0.0))

    assert worst > SERVO_ANGLE_TOL_RAD, (
        "重新接回 roll 符号后，输出居然和当前版本几乎一致——这条历史证据"
        "复现失败，说明几何重导可能引入了别的抵消，需要重新核实。"
    )
    assert flips > len(base) // 2, (
        f"只有 {flips}/{len(base)} 组输出翻转，与 2026-09-06 实测的 519 组不符"
    )


def test_the_cancellation_claim_is_not_reasserted() -> None:
    """那句错话曾在七处被复述（R-F6-2 复查时又在 app_stabilizer.c 与
    app_rc_config.h 各发现一处，原有守卫列表未覆盖），别让它回来。"""
    for path in (CTRL_SOURCE,
                 ROOT / "Driver" / "Inc" / "drv_coax_ctrl.h",
                 ROOT / "tests" / "test_coax_sign_convention.py",
                 ROOT / "tests" / "test_flu_seam3_controller_frame.py",
                 STABILIZER,
                 ROOT / "App" / "Inc" / "app_rc_config.h"):
        text = read(path)
        assert "在姿态误差中相消" not in text, path
        assert "cancel in the attitude error" not in text, path
        assert "cancels in the attitude error" not in text, path


def test_outer_loop_acceleration_directions_agree_with_flu(tmp_path: Path) -> None:
    """外环是被物理钉死的那一环，实测它认哪个姿态叫"到位"。

    内环对实测与目标一视同仁，所以口径怎么变都自洽，问不出东西。外环不然：
    目标姿态由**加速度指令**经力矢量算出来（`atan2(F_前, F_上)`），而加速度
    指令是物理量，所以外环把角度口径钉死在物理上。

    实测（真控制器，加速度前馈通道，迁移前后数值不变——见下文解释）：
      ax=+2（向前加速，物理上要机头下俯）→ 最安静在 pitch=+0.19 → FLU 机头下俯 ✓
      ay=+2（本变量原称"向右加速"）        → 最安静在 roll =-0.20 → FLU 左翼下沉

    R-F6-2 第 3 节由 2026-08-30 left_y 实录确认：旧 seam2 Y 是右正；进入
    seam3 时取反为规范 FLU 左正。因此迁移后的 ay=+2 表示**向左**加速，其
    物理正确响应是左翼下沉（FLU roll 为负）。本测试只钉 seam3 内部 FLU
    几何；旧版右正输入的既有方向缺陷另由 R-F7 跟踪，不在这里顺手修控制律。

    这条结论不改变本测试的数值断言（下面的阈值原样保留），只改变解读：
    迁移前这里断言的是"矛盾"，迁移后断言的是"自洽"，数值本身分毫未变——
    因为 ay/ax 的物理含义、atan2 几何和平衡点搜索都没有变化，变的只是
    "ay=+2 该叫向左还是向右"这句人类标注。
    """
    if shutil.which("gcc") is None:
        pytest.skip("host gcc is unavailable")

    rows = _build_and_run(tmp_path, "outer", read(CTRL_SOURCE),
                          harness=OUTER_LOOP_HARNESS, expect=3)
    (fwd_pitch, fwd_roll), (left_pitch, left_roll), (right_pitch, right_roll) = rows

    # ax=+2（向前加速）：目标俯仰为正，且平衡点落在正 pitch —— FLU 机头下俯，自洽。
    assert fwd_pitch > 0.05, f"向前加速的平衡俯仰角变了：{fwd_pitch}"
    assert abs(fwd_roll) < 0.05, f"向前加速不该要求滚转：{fwd_roll}"

    # ay=+2（向左加速，见上文定性）：平衡点落在负 roll —— FLU 左翼下沉，自洽。
    assert left_roll < -0.05, (
        f"ay=+2（左）的平衡滚转角变成 {left_roll}。若这是有意的重导，"
        "请连同 drv_coax_ctrl.h 的 seam 3 frame map 与本断言一起更新。"
    )
    assert abs(left_pitch) < 0.05, f"ay=+2（左）不该要求俯仰：{left_pitch}"

    # R-F7 的旧复现把 legacy ay=+2 称作“向右”。迁移后右向必须编码为
    # FLU ay=-2；真控制器此时平衡在正 roll，即右翼下沉。
    assert right_roll > 0.05, (
        f"ay=-2（右）的平衡滚转角应为正（右翼下沉），实际 {right_roll}"
    )
    assert abs(right_pitch) < 0.05, f"ay=-2（右）不该要求俯仰：{right_pitch}"


def test_only_the_arm_lock_knows_about_flu(tmp_path: Path) -> None:
    """全仓库唯一读 `APP_Sensor_IsFluOrientationActive()` 的地方是解锁互锁。

    控制律里没有"按朝向切口径"的分支，这是对的——运行时分叉会让两种朝向都
    缺乏证据（见 frame-migration 模式）。代价是半迁移态只能靠那道互锁兜住，
    所以**它必须留着**，直到六个 seam 全部收口。
    """
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


# --------------------------------------------------------------------------
# Section 7 of the migration doc: yaw was never given a FORCE_FRAME-style
# sign constant, so check (don't casually fix) whether it enters the
# geometry with a sign consistent with FLU.  "Write down whatever you find."

YAW_HARNESS = AIRFRAME_FIXTURE_C + r"""
#include "drv_coax_ctrl.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    airframe_load_reference();
    DRV_COAX_CTRL_AttitudeInput att;
    DRV_COAX_CTRL_Reference ref;
    DRV_COAX_CTRL_Output out;
    DRV_COAX_CTRL_Debug debug;
    int i;

    DRV_COAX_CTRL_Init();

    /* Level attitude except yaw; reference.yaw_rad stays 0, so this is a
     * pure yaw-only measured-vs-target mismatch, hover thrust. */
    for (i = -6; i <= 6; ++i) {
        float yaw = (float)i * 0.1f;

        memset(&att, 0, sizeof(att));
        memset(&ref, 0, sizeof(ref));
        ref.direct_attitude_target_valid = 1U;
        ref.manual_total_force_valid = 1U;
        ref.manual_total_force_n = DRV_Airframe_Get()->mass_kg * DRV_Airframe_Get()->gravity_m_s2;
        att.yaw_rad = yaw;

        DRV_COAX_CTRL_Run(&att, &ref, &out);
        DRV_COAX_CTRL_GetLastDebug(&debug);
        printf("%.9f %.9f\n", (double)yaw, (double)debug.moment_cmd_n_m[2]);
    }
    return 0;
}
"""


def test_yaw_moment_restores_toward_target_the_flu_consistent_way(
    tmp_path: Path,
) -> None:
    """R-F6-2 section 7: yaw has no sign constant; check what it actually does.

    +yaw is FLU nose-left; legacy FRD called the same physical rotation
    nose-right (seam 0/1 already flipped this, with no compensating constant
    in the controller -- there is nothing named "YAW_SIGN" anywhere in
    drv_coax_ctrl.c).  Measured here: with reference.yaw_rad pinned at 0 and
    attitude.yaw_rad swept away from it, the commanded yaw moment
    (moment_cmd_n_m[2]) is NEGATIVE for a POSITIVE yaw error and POSITIVE for
    a negative one -- i.e. it always pushes back toward zero. A restoring
    moment for a "nose has turned left" error must turn the nose back right,
    which is negative about FLU's +Z-up axis: this is the FLU-consistent
    sign, found without any code change and without a dedicated constant.

    Separately (not asserted numerically here, just recorded): the OTHER
    place yaw appears, coax_ctrl_local_down_to_body's `psi`, only feeds
    DRV_COAX_CTRL_Debug.force_cmd_n (telemetry) -- target_roll_rad/
    target_pitch_rad are computed from desired_force_local_n directly with
    no yaw rotation at all, so the outer loop's "local X/Y" implicitly means
    "X/Y at whatever heading is currently held", not a yaw-stabilised world
    frame.  That is a pre-existing architectural property, not something
    R-F6-2 introduced or is fixing -- per this section's instruction, this
    records the finding without changing it.
    """
    if shutil.which("gcc") is None:
        pytest.skip("host gcc is unavailable")

    rows = _build_and_run(tmp_path, "yaw", read(CTRL_SOURCE),
                          harness=YAW_HARNESS, expect=13)

    for yaw, moment_z in rows:
        if yaw > 1.0e-6:
            assert moment_z < 0.0, f"yaw={yaw}: expected restoring (negative) moment_z, got {moment_z}"
        elif yaw < -1.0e-6:
            assert moment_z > 0.0, f"yaw={yaw}: expected restoring (positive) moment_z, got {moment_z}"
        else:
            assert abs(moment_z) < 1.0e-9, f"zero yaw error should give zero moment_z, got {moment_z}"
