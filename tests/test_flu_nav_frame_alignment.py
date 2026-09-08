"""EKF 的两路输入必须处在同一个坐标系里。

背景：`SVC_FlowNav_Fuse` 用 IMU 水平加速度做预测、用光流水平速度做更新，两者
按同一组 X/Y 直接对应。光流给的是**机体水平地速**；而 IMU 那一路曾经在转平之后
又乘了一次 Rz(yaw)，把加速度送进"上电时刻朝向"的固定世界系。于是两路输入之间
差着一个累计偏航角，滤波器的预测和观测互相对抗。

这个缺陷有个恶劣的性质：**偏航为 0 时它完全不存在**。上电不转向的台架测试、
以及任何只看单次悬停的对拍，都不会暴露它。所以这里不用自造输入，直接拿实录的
姿态与比力序列量出它到底有多大。

修法不是"把光流转到固定世界系"：下游（EKF 输出、控制器目标姿态、遥测声明的
`frame=body_flu`、遥控杆意图）本来就全是机头相对口径，把 IMU 对齐到机头改动
最小、且让摇杆保持"永远相对机头"。
"""

from __future__ import annotations

import csv
import math
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
FLOW_NAV_HEADER = ROOT / "Services" / "Inc" / "svc_flow_nav.h"
FLOW_NAV_SOURCE = ROOT / "Services" / "Src" / "svc_flow_nav.c"
RECORDING = (ROOT / "data" / "flight_logs" / "2026-09-02" / "rm1_3_block_queue" /
             "flightlog_20260902_202552.csv")

GRAVITY_M_S2 = 9.81


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_imu_level_adapter_takes_no_yaw() -> None:
    """机头对齐是靠"少乘一次 yaw"实现的，所以 yaw 根本不该传进来。

    把 yaw 留在参数表里、只是恰好传 0，是会被后人"修好"的写法。签名里没有它，
    才叫钉死。
    """
    source = read(STABILIZER)

    assert "stabilizer_compensated_imu_accel_nav_xy" not in source, (
        "旧函数名意味着固定世界系口径，必须随实现一起消失"
    )
    signature = source.split("stabilizer_compensated_imu_accel_level_xy(", 1)[1]
    signature = signature.split(")", 1)[0]
    assert "yaw_rad" not in signature, signature

    call = source.split("stabilizer_compensated_imu_accel_level_xy(\n", 1)[1]
    call = call.split(");", 1)[0]
    assert "yaw_control" not in call, call

    # 实现里也不能再出现偏航三角量。
    body = source.split("stabilizer_compensated_imu_accel_level_xy(", 1)[1]
    body = body.split("static uint8_t stabilizer_rc_update_armed", 1)[0]
    for banned in ("cosf(yaw_rad)", "sinf(yaw_rad)", "cy *", "sy *"):
        assert banned not in body, banned


def test_flow_nav_header_names_the_heading_aligned_frame() -> None:
    header = read(FLOW_NAV_HEADER)
    assert "stabilizer_compensated_imu_accel_level_xy" in header
    assert "机头对齐" in header
    # 不许再把它说成固定世界系或 NED/ENU。
    assert "NED" in header and "不" in header


@pytest.fixture(scope="module")
def samples() -> list[tuple[float, float, float, float, float, float]]:
    if not RECORDING.is_file():
        pytest.skip(f"recording missing: {RECORDING}")
    rows: list[tuple[float, float, float, float, float, float]] = []
    with RECORDING.open(newline="", encoding="utf-8", errors="replace") as handle:
        for row in csv.DictReader(handle):
            try:
                rows.append((
                    float(row["accel_x_g"]),
                    float(row["accel_y_g"]),
                    float(row["accel_z_g"]),
                    math.radians(float(row["roll_deg"])),
                    math.radians(float(row["pitch_deg"])),
                    math.radians(float(row["yaw_deg"])),
                ))
            except (TypeError, ValueError, KeyError):
                continue
    assert len(rows) > 5000, len(rows)
    return rows


def _heading_aligned(ax_g: float, ay_g: float, az_g: float,
                     roll: float, pitch: float) -> tuple[float, float]:
    """当前实现：只用 Ry(pitch)*Rx(roll) 转平。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    f = (ax_g * GRAVITY_M_S2, ay_g * GRAVITY_M_S2, az_g * GRAVITY_M_S2)
    return (cp * f[0] + sp * sr * f[1] + sp * cr * f[2],
            cr * f[1] - sr * f[2])


def _fixed_world(ax_g: float, ay_g: float, az_g: float,
                 roll: float, pitch: float, yaw: float) -> tuple[float, float]:
    """迁移前基线：Rz(yaw)*Ry(pitch)*Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    f = (ax_g * GRAVITY_M_S2, ay_g * GRAVITY_M_S2, az_g * GRAVITY_M_S2)
    return ((cy * cp) * f[0] + (cy * sp * sr - sy * cr) * f[1] +
            (cy * sp * cr + sy * sr) * f[2],
            (sy * cp) * f[0] + (sy * sp * sr + cy * cr) * f[1] +
            (sy * sp * cr - cy * sr) * f[2])


def test_recording_actually_exercises_yaw(samples) -> None:
    """没有偏航的实录证明不了任何事——先确认这份数据有资格当证据。"""
    yaw_deg = [math.degrees(s[5]) for s in samples]
    span = max(yaw_deg) - min(yaw_deg)
    beyond_10 = sum(1 for v in yaw_deg if abs(v) > 10.0) / len(yaw_deg)
    assert span > 180.0, span
    assert beyond_10 > 0.5, beyond_10


def test_old_frame_differs_from_new_by_exactly_the_yaw_rotation(samples) -> None:
    """两版之差就是 Rz(yaw)，一分不多。

    这条把"改了什么"说死：不是换了增益、不是改了补偿，就是少乘一个偏航旋转。
    """
    worst = 0.0
    for ax, ay, az, roll, pitch, yaw in samples:
        nx, ny = _heading_aligned(ax, ay, az, roll, pitch)
        ox, oy = _fixed_world(ax, ay, az, roll, pitch, yaw)
        cy, sy = math.cos(yaw), math.sin(yaw)
        worst = max(worst,
                    abs(ox - (cy * nx - sy * ny)),
                    abs(oy - (sy * nx + cy * ny)))
    assert worst < 1.0e-9, worst


def test_the_defect_was_material_on_real_data_and_invisible_at_zero_yaw(
    samples,
) -> None:
    """量出这个夹角在实录上到底有多大，以及它为什么一直没被发现。

    实测（2026-09-02，6909 样本，偏航跨度 224°）：
      * 全程最大偏差 24.04 m/s^2 —— 约 2.4 g 的纯坐标系误差直接进了 EKF 预测；
      * 但 |yaw| < 0.1° 的样本里最大只有 0.009 m/s^2。
    后一条就是它躲过所有台架测试的原因：上电不转向时它根本不存在。
    """
    worst_all = 0.0
    worst_level = 0.0
    level_count = 0
    for ax, ay, az, roll, pitch, yaw in samples:
        nx, ny = _heading_aligned(ax, ay, az, roll, pitch)
        ox, oy = _fixed_world(ax, ay, az, roll, pitch, yaw)
        delta = max(abs(ox - nx), abs(oy - ny))
        worst_all = max(worst_all, delta)
        if abs(math.degrees(yaw)) < 0.1:
            level_count += 1
            worst_level = max(worst_level, delta)

    assert level_count > 100, level_count
    assert worst_all > 10.0, worst_all
    assert worst_level < 0.05, worst_level


# ─────────────────────── 旋转系的 -ω × v 运动学项 ───────────────────────
#
# 上面把两路输入对齐到了同一个「机头对齐的本地水平系」。但那个系是**随偏航转动
# 的**，所以速度分量的导数不等于加速度：
#     dv/dt|分量 = a - ω × v,   ω = (0, 0, ω_z)
#     ω × v = (-ω_z*v_y, +ω_z*v_x, 0)
# 不补这一项，定速直飞中原地转向会被 EKF 读成"速度没变"，而机头相对的速度分量
# 其实正在互换。


def test_rotating_frame_term_lives_in_the_service_not_the_ekf() -> None:
    """这一项耦合 X/Y，因此不能塞进被定义为两轴独立的纯数值 EKF 里。"""
    service = read(FLOW_NAV_SOURCE)
    stabilizer = read(STABILIZER)
    ekf = read(ROOT / "Driver" / "Src" / "drv_nav_ekf.c")

    assert "accel_x_eff = input->accel_x_m_s2 +" in service
    assert "(omega_z * flow_nav_ctx.ekf.vel_m_s[1])" in service
    assert "accel_y_eff = input->accel_y_m_s2 -" in service
    assert "(omega_z * flow_nav_ctx.ekf.vel_m_s[0])" in service
    # 驱动层必须保持无坐标系、两轴不互串。
    assert "yaw_rate" not in ekf
    assert "omega" not in ekf
    # ω_z 来自规范 FLU 的陀螺 Z 轴，由 App 传入。
    assert "fuse_input.yaw_rate_rad_s = gyro_rad_s[2];" in stabilizer


def test_steady_yaw_rotates_the_heading_relative_velocity(tmp_path: Path) -> None:
    """真编译 Service+EKF：定速前飞 + 原地左转，横向分量必须往右长出来。

    物理约定：机头左转是 FLU 的正 ω_z。地速方向不变而机头左转，则速度相对机头
    偏到**右**边，即 FLU 的 -Y。所以 v_y 必须变负。

    两次运行只差 `yaw_rate_rad_s` 一个量，其余（衰减、桥接、量测）完全相同，
    所以差值只能来自这一项。
    """
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "rot.c"
    harness.write_text(
        r'''#include "svc_flow_nav.h"
#include <stdio.h>
#include <string.h>

static void run(float omega_z, float *vx, float *vy)
{
    SVC_FLOW_NAV_FuseInput in;
    uint32_t t = 1000U;
    int i;

    SVC_FlowNav_Init();
    SVC_FlowNav_Reset();

    /* Seed 1 m/s forward and arm the IMU bridge with real flow updates. */
    for (i = 0; i < 60; ++i) {
        memset(&in, 0, sizeof(in));
        in.flow_valid = 1U;
        in.flow_quality = 200U;
        in.flow_vx_m_s = 1.0f;
        in.flow_sample_ms = t;
        in.dt_sec = 0.002f;
        in.now_ms = t;
        (void)SVC_FlowNav_Fuse(&in);
        t += 2U;
    }
    /* Bridge only: zero true acceleration, steady yaw rate, 70 ms < 80 ms. */
    for (i = 0; i < 35; ++i) {
        memset(&in, 0, sizeof(in));
        in.yaw_rate_rad_s = omega_z;
        in.dt_sec = 0.002f;
        in.now_ms = t;
        (void)SVC_FlowNav_Fuse(&in);
        t += 2U;
    }
    SVC_FlowNav_GetVelocity(vx, vy);
}

int main(void)
{
    float vx0, vy0, vx1, vy1;
    run(0.0f, &vx0, &vy0);
    run(2.0f, &vx1, &vy1);
    printf("%.6f %.6f %.6f %.6f\n",
           (double)vx0, (double)vy0, (double)vx1, (double)vy1);
    return 0;
}
''',
        encoding="ascii",
    )
    executable = tmp_path / "rot.exe"
    subprocess.run(
        [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
         f"-I{ROOT / 'Services' / 'Inc'}", f"-I{ROOT / 'Driver' / 'Inc'}",
         str(FLOW_NAV_SOURCE), str(ROOT / "Driver" / "Src" / "drv_nav_ekf.c"),
         str(harness), "-lm", "-o", str(executable)],
        check=True, capture_output=True, text=True,
    )
    vx0, vy0, vx1, vy1 = (
        float(v) for v in subprocess.run(
            [str(executable)], check=True, capture_output=True, text=True,
        ).stdout.split()
    )

    assert vx0 > 0.5, f"seeding failed, vx0={vx0}"
    assert abs(vy0) < 1.0e-3, f"ω_z=0 时不该长出横向速度: vy0={vy0}"
    assert vy1 < -0.02, f"左转时速度应偏到机头右侧(-Y): vy1={vy1}"
    # 一阶量级核对：Δv_y ≈ -ω_z * v_x * T = -2.0 * ~0.9 * 0.07 ≈ -0.13
    assert -0.30 < (vy1 - vy0) < -0.05, (vy1 - vy0)


def test_rotating_frame_term_is_material_in_flight_and_zero_at_hover(
    samples,
) -> None:
    """在实录上量这一项到底多大，别拿"理论上应该有"当理由。

    用同一份 2026-09-02 实录的 `gyro_z_dps` 与 `vel_est_m_s_*`：
      * 峰值 2.22 m/s^2（约 0.23 g）—— 偏航机动中不是可忽略量；
      * 中位数 0.009 m/s^2 —— 悬停 v≈0 时确实趋零，所以以前看不出来。
    这也解释了为什么它排在坐标系夹角（24.04 m/s^2）之后：小一个数量级。
    """
    values: list[float] = []
    if not RECORDING.is_file():
        pytest.skip("recording missing")
    with RECORDING.open(newline="", encoding="utf-8", errors="replace") as handle:
        for row in csv.DictReader(handle):
            try:
                omega_z = math.radians(float(row["gyro_z_dps"]))
                vx = float(row["vel_est_m_s_0"])
                vy = float(row["vel_est_m_s_1"])
            except (TypeError, ValueError, KeyError):
                continue
            values.append(max(abs(omega_z * vy), abs(omega_z * vx)))

    assert len(values) > 5000, len(values)
    values.sort()
    assert values[-1] > 1.0, values[-1]
    assert values[len(values) // 2] < 0.05, values[len(values) // 2]
