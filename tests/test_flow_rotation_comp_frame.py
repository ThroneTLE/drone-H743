"""光流方言边界契约（2026-09-07）。

实测现象：手飞画圆时，传感器页的原始光流两轴都是干净的正弦，状态页却只剩 X 的
正弦、Y 被压到接近 0。

根因不在 EKF（`drv_nav_ekf.c` 的 4 维状态 X/Y 完全镜像），也不在两轴对称的门限，
而在**方言转换的位置**：旋转补偿项由规范 FLU 的陀螺算出（`APP_Sensor_ApplyFrame
Correction` 已把 accel/gyro 一起转成 FLU），却被加在尚未转换的 FRD 光流速度上——
`DRV_FRAME_FrdToFlu` 原本在 `stabilizer_compensate_flow_rotation` 的**末尾**才调用。

  X 前向：FLU 与 FRD 同号  → 补偿正确
  Y 横向：FLU 朝左、FRD 朝右 → 补偿反号，净效果是减了两倍

为什么偏偏圆周飞行暴露它：靠滚转产生向心加速度时 φ ≈ -a_y/g，故 ω_x = dφ/dt 与
v_y **同相**；错号的 Y 补偿正比于 ω_x，正好反相抵消真实横向速度。

修复不是把那次转换往前挪（那只是换个地方再犯一次），而是把边界钉死在"芯片原话
变成载具量测"的唯一一处——app_optical_flow.c 的 app_flow_fill_sample()，在任何机体
量（陀螺、姿态）混进来之前。此后 Service / EKF / 控制器 / 遥测全链都是 FLU，
Service 依然保持 seam2 要求的"不旋转不换轴"纯数值性质。

本文件同时挡住"好心"地在下游把转换加回来。
"""

from __future__ import annotations

import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8", errors="replace")


STABILIZER = read("App/Src/app_stabilizer.c")
FLOW_NAV = read("Services/Src/svc_flow_nav.c")
FLOW_APP = read("App/Src/app_optical_flow.c")


def _macro(name: str) -> float:
    m = re.search(rf"#define\s+{name}\s+([-\d.]+)f", STABILIZER)
    assert m is not None, name
    return float(m.group(1))


GAIN = _macro("STABILIZER_FLOW_ROT_COMP_GAIN")
OFFSET_Z = _macro("STABILIZER_FLOW_SENSOR_OFFSET_Z_M")
GRAVITY = 9.81


# --------------------------------------------------------------------------
# 边界只有一个，且在采集侧
# --------------------------------------------------------------------------

def test_dialect_boundary_lives_where_the_driver_frame_becomes_a_sample() -> None:
    """边界必须在任何机体量混进来之前，也就是驱动帧搬进 Service 样本的那一行。"""
    fill = FLOW_APP.split("static void app_flow_fill_sample")[1].split("\n}")[0]
    assert "sample->flow_vel_x = frame->flow_vel_x;" in fill
    assert "-frame->flow_vel_y" in fill, "Y 必须取负：传感器 FRD → 规范 FLU"
    assert "INT16_MIN" in fill, "取负要挡住 INT16_MIN 溢出"
    # 原始计数照旧透传给传感器页，不要顺手也转了。
    assert "status->flow_vel_y = frame->flow_vel_y;" in FLOW_APP


def test_service_stays_pure_numerics() -> None:
    """seam2 的既有契约：Service 不旋转、不换轴。边界不能塞回这里。"""
    assert "FrdToFlu" not in FLOW_NAV
    assert "FluToFrd" not in FLOW_NAV
    assert "flow_nav_ctx.filtered_flow_vel_y * 0.01f" in FLOW_NAV


def test_app_layer_has_no_frame_conversion_left() -> None:
    """稳定器里不得再出现任何方言转换——这是这次缺陷的形状，别让它回潮。"""
    assert "FrdToFlu" not in STABILIZER, (
        "app_stabilizer.c 不该做方言转换：光流在 app_flow_fill_sample() 就已经是"
        "FLU 了。在这里补一次转换正是 2026-09-07 缺陷的原始形态。"
    )
    assert "FluToFrd" not in STABILIZER

    fn = STABILIZER.split("static void stabilizer_compensate_flow_rotation")[1]
    fn = fn.split("\n  }\n")[0]
    # 入参直接当 FLU 用，出口直接交出，不做二次翻转。
    assert "body_vx_m_s = *flow_vx_m_s;" in fn
    assert "body_vy_m_s = *flow_vy_m_s;" in fn
    assert "*flow_vx_m_s = body_vx_m_s;" in fn
    assert "*flow_vy_m_s = body_vy_m_s;" in fn


def test_stale_second_order_note_is_gone() -> None:
    """-ω×v 项已经在 Service 里补上了；旧注释说"EKF 目前没有"会误导后续修改。"""
    assert "EKF 目前没有" not in STABILIZER
    assert "omega_z * flow_nav_ctx.ekf.vel_m_s[1]" in FLOW_NAV
    assert "omega_z * flow_nav_ctx.ekf.vel_m_s[0]" in FLOW_NAV


# --------------------------------------------------------------------------
# 补偿公式的物理（2026-09-30 台架实测后改正）
# --------------------------------------------------------------------------
#
# 光流把"地面点在像里的运动"报成载具速度：读数 = v_传感器 + ω × r_地面，
# r_地面 = (0, 0, -h)（FLU，地面在下方）；v_传感器 = v_重心 + ω × r_安装。
# 所以 v_重心 = 读数 - ω × (r_安装 + r_地面)。
#
# 旧写法（2026-09-07 只修了方言边界）把光学伪像当成"读数里少了的那份"又加了一遍，
# 安装偏置 Z 也按 Z 朝下写成 +0.22。水平槽台架手摆（纯绕杆转动）实测：原始读数与旧
# 补偿项 12/12 同号、幅值比约 1.18 =(h+杆下约 8 cm)/h；若旧写法正确，二者应反号。
# 旧写法每弧度漏进约 1 m 假位移，tilt 激励（0.07 rad/100 ms 斜坡）会造成约 0.6 m/s
# 假速度。证据：data/analysis/sysid-rig-params/2026-09-30/xy_rock_check_*.txt。

OFFSET_X = _macro("STABILIZER_FLOW_SENSOR_OFFSET_X_M")
OFFSET_Y = _macro("STABILIZER_FLOW_SENSOR_OFFSET_Y_M")


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _sensor_reading(v_cg, omega, height_m):
    """物理：传感器读数（FLU，已过方言边界）。"""
    r_mount = (OFFSET_X, OFFSET_Y, OFFSET_Z)
    v_sensor = [v_cg[k] + _cross(omega, r_mount)[k] for k in range(2)]
    artifact = _cross(omega, (0.0, 0.0, -height_m))
    return (v_sensor[0] + artifact[0], v_sensor[1] + artifact[1])


def _firmware_compensate(raw, omega, height_m):
    """逐项照抄 stabilizer_compensate_flow_rotation（下面的源码断言把两边钉在一起）。"""
    gx, gy, gz = omega
    opt = (GAIN * height_m * gy, -GAIN * height_m * gx)
    off = (-((gy * OFFSET_Z) - (gz * OFFSET_Y)), -((gz * OFFSET_X) - (gx * OFFSET_Z)))
    return (raw[0] + opt[0] + off[0], raw[1] + opt[1] + off[1])


def test_python_mirror_matches_the_firmware_formula() -> None:
    fn = STABILIZER.split("static void stabilizer_compensate_flow_rotation")[1].split("\n  }\n")[0]
    compact = " ".join(fn.split())
    assert "debug->optical_rot_comp_m_s[0] = STABILIZER_FLOW_ROT_COMP_GAIN * height_m * gyro_y_rad_s;" in compact
    assert "debug->optical_rot_comp_m_s[1] = -STABILIZER_FLOW_ROT_COMP_GAIN * height_m * gyro_x_rad_s;" in compact
    assert ("debug->offset_rot_comp_m_s[0] = -((gyro_y_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Z_M) - "
            "(gyro_z_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Y_M));") in compact
    assert ("debug->offset_rot_comp_m_s[1] = -((gyro_z_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_X_M) - "
            "(gyro_x_rad_s * STABILIZER_FLOW_SENSOR_OFFSET_Z_M));") in compact
    assert OFFSET_Z < 0.0, "FLU Z 朝上：光流模块在重心下方，安装偏置 Z 必须为负"
    assert GAIN == 1.0


def test_pure_rotation_leaves_no_phantom_velocity() -> None:
    """静止的重心、任意角速度：补偿后速度为 0（台架手摆的情形）。"""
    for omega in ((0.8, 0.0, 0.0), (0.0, -1.2, 0.0), (0.5, 0.7, 0.3), (-0.9, 0.4, -0.6)):
        for height_m in (0.3, 0.47, 1.5):
            raw = _sensor_reading((0.0, 0.0), omega, height_m)
            corrected = _firmware_compensate(raw, omega, height_m)
            assert abs(corrected[0]) < 1e-9 and abs(corrected[1]) < 1e-9, (omega, height_m, corrected)


def test_circle_flight_recovers_both_axes() -> None:
    """2026-09-07 的场景（靠滚转做圆周，ω_x 与 v_y 同相）：补偿后两轴都等于真值。"""
    radius_m, period_s, height_m = 0.5, 2.9, 1.0
    omega_c = 2.0 * math.pi / period_s
    for step in range(200):
        phase = omega_c * period_s * step / 200.0
        v = (-radius_m * omega_c * math.sin(phase), radius_m * omega_c * math.cos(phase))
        rates = (radius_m * omega_c ** 3 * math.cos(phase) / GRAVITY,
                 radius_m * omega_c ** 3 * math.sin(phase) / GRAVITY, 0.0)
        corrected = _firmware_compensate(_sensor_reading(v, rates, height_m), rates, height_m)
        assert math.isclose(corrected[0], v[0], abs_tol=1e-9)
        assert math.isclose(corrected[1], v[1], abs_tol=1e-9)


def test_bench_rocking_signature_rules_out_the_old_sign() -> None:
    """台架判据：纯转动时原始读数与光学补偿项必须反号（旧写法同号，实测 12/12 同号）。"""
    height_m = 0.47
    for omega in ((0.6, 0.0, 0.0), (0.0, 0.6, 0.0), (-0.4, 0.4, 0.0)):
        raw = _sensor_reading((0.0, 0.0), omega, height_m)
        opt = (GAIN * height_m * omega[1], -GAIN * height_m * omega[0])
        for k in range(2):
            if abs(raw[k]) > 1e-6:
                assert raw[k] * opt[k] < 0.0, (omega, raw, opt)
