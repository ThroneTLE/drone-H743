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
# 数值复现：为什么只有 Y 死、且只在圆周飞行时死
# --------------------------------------------------------------------------

def _simulate(circle_period_s: float, radius_m: float, height_m: float,
              buggy: bool) -> tuple[float, float]:
    """半径 R、周期 T 的匀速圆周飞行，返回补偿后 X/Y 速度幅值。

    FLU 下 v = RΩ(-sin Ωt, cos Ωt)，a = -RΩ²(cos Ωt, sin Ωt)。
    小角近似 a_y = -g·φ、a_x = +g·θ，于是
        ω_x = dφ/dt = +RΩ³cos(Ωt)/g   与 v_y 同相  ← 这就是 Y 被抵消的原因
        ω_y = dθ/dt = +RΩ³sin(Ωt)/g   与 v_x 反相
    """
    omega = 2.0 * math.pi / circle_period_s
    lever = (GAIN * height_m) + OFFSET_Z
    xs: list[float] = []
    ys: list[float] = []

    for step in range(400):
        phase = omega * (circle_period_s * step / 400.0)
        v_x_flu = -radius_m * omega * math.sin(phase)
        v_y_flu = radius_m * omega * math.cos(phase)
        gyro_x = radius_m * (omega ** 3) * math.cos(phase) / GRAVITY
        gyro_y = radius_m * (omega ** 3) * math.sin(phase) / GRAVITY

        # 传感器原始读数含旋转伪像，且以 FRD 报出（Y 朝右）。
        artifact_x = -lever * gyro_y
        artifact_y = lever * gyro_x
        raw_frd_x = v_x_flu - artifact_x
        raw_frd_y = -(v_y_flu - artifact_y)

        if buggy:
            # 修复前：FLU 补偿加在 FRD 速度上，末尾再整体翻 Y。
            xs.append(raw_frd_x + artifact_x)
            ys.append(-(raw_frd_y + artifact_y))
        else:
            # 修复后：采集边界先转 FLU，补偿项本来就是 FLU。
            xs.append(raw_frd_x + artifact_x)
            ys.append(-raw_frd_y + artifact_y)

    return (max(map(abs, xs)), max(map(abs, ys)))


def test_bug_cancels_lateral_velocity_while_forward_survives() -> None:
    """复现实测：约 2.9 s 一圈时 Y 被抵消殆尽，X 完好。"""
    radius_m = 0.5
    truth = 2.0 * math.pi / 2.9 * radius_m

    bad_x, bad_y = _simulate(2.9, radius_m, 1.0, buggy=True)
    good_x, good_y = _simulate(2.9, radius_m, 1.0, buggy=False)

    # 修复后两轴都还原成真实速度幅值。
    assert math.isclose(good_x, truth, rel_tol=1e-6)
    assert math.isclose(good_y, truth, rel_tol=1e-6)

    # 修复前 X 也是对的——所以这个缺陷单看 X 完全看不出来。
    assert math.isclose(bad_x, truth, rel_tol=1e-6)

    # 而 Y 只剩两成不到，正是"仿佛被抑制了"。
    assert bad_y < 0.2 * truth, f"expected Y suppressed, got {bad_y:.3f}/{truth:.3f}"


def test_suppression_ratio_follows_the_closed_form() -> None:
    """误差/信号 = 2(GAIN·h + offset_z)Ω²/g。

    抵消点随圈速与高度移动：飞得慢或飞得低时这个缺陷会自己"变轻"，很容易被误判成
    已经修好了——所以判据钉的是闭式解，不是某一次实测的幅值。
    """
    radius_m = 0.5
    for period_s, height_m in ((2.0, 1.0), (4.0, 1.0), (3.0, 0.6)):
        omega = 2.0 * math.pi / period_s
        lever = (GAIN * height_m) + OFFSET_Z
        predicted = abs(1.0 - (2.0 * lever * omega * omega / GRAVITY))
        _, bad_y = _simulate(period_s, radius_m, height_m, buggy=True)
        assert math.isclose(bad_y / (omega * radius_m), predicted, rel_tol=1e-6)
