"""光流诊断里程计（2026-10-01 水平槽台架尺度排查）。

作者目测 vel 轮"移动了45，差不多是中间位置到边缘了"，光流只记约 10 cm；贴胶带后上锁手推 90 cm
读 56–59 cm。为分出尺度丢在哪一层，Service 里另累计两路不经 EKF/零速钳位/转动补偿的位移：
原始计数与中值滤波后，都按 计数×0.01×高度×传感器步长 换算，`FLOW?` 多报一行 `FLOW odo`。
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAV = (ROOT / "Services/Src/svc_flow_nav.c").read_text(encoding="utf-8")
APP = (ROOT / "App/Src/app_optical_flow.c").read_text(encoding="utf-8")


def test_the_diag_odometer_integrates_raw_and_filtered_counts_before_the_ekf() -> None:
    push = NAV.split("SVC_FLOW_NAV_SampleResult SVC_FlowNav_PushSample(")[1]
    block = push.split("if (dt_us != 0UL) {")[1].split("}")[0]
    assert "(float)sample->flow_vel_x * 0.01f * flow_nav_ctx.height_m * dt_s" in block
    assert "(float)sample->flow_vel_y * 0.01f * flow_nav_ctx.height_m * dt_s" in block
    assert "flow_nav_ctx.diag_filt_m[0] += sensor_vx_m_s * dt_s;" in block
    # 在 EKF 更新之前、与 EKF 速度累计（displacement）分开。
    assert push.index("flow_nav_ctx.diag_raw_m[0] +=") < push.index("flow_nav_ctx.vx_m_s = sensor_vx_m_s;")
    assert "diag_raw_m" not in NAV.split("static void flow_nav_integrate_position(void)")[1].split("\n}\n")[0]


def test_flow_report_prints_the_diag_line() -> None:
    assert "FLOW odo raw_x_mm=%ld raw_y_mm=%ld filt_x_mm=%ld filt_y_mm=%ld steps=%lu" in APP
    assert "SVC_FlowNav_GetDiagOdometer(raw_m, filt_m, &steps);" in APP
