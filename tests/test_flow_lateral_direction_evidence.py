"""光流横向方向：用 2026-08-30 的地面实录把控制器系 Y 的正方向钉死。

R-F6-2 卡在一个问题上：控制器系的 `y_m` / `vy_m_s` 到底是机体**右**正还是
**左**正。仓库里五处注释说"右正"，`15d83454` 的交付改口说"一直是左正"，两边
都只有措辞、没有签名证据，而这两种答案会导出方向相反的控制器。

`data/calibration/flow_range/2026-08-30/` 里有现成的答案，是 R-F5 期间录的地面
标定，`body_frame=FLU`、`orientation_code=3`：

  * 面板的 `left_y` 阶段提示语是"向机体左侧移动（+Y）"——**物理方向已知**；
  * 该阶段记录的 `vy_m_s` 取自 `FLOW comp` 的 `sensor_vy_mm_s`，也就是
    `snapshot.sensor_velocity_flu_m_s[1]`（导出前已声明 `export=canonical_flu`）；
  * 旧实现曾在快照出口手写取反；随后集中到光流处理函数**出口**的具名 Adapter，
    但那已经晚了——同一函数里的旋转补偿用的是早已转成 FLU 的陀螺，补偿被加在
    尚未转换的 FRD 速度上，Y 反号（2026-09-07 修复）。现在这个变换钉在
    `app_optical_flow.c::app_flow_fill_sample()`，即驱动帧变成 Service 样本、
    任何机体量混入**之前**的那一行，之后全链只透传 FLU。

于是链条闭合：向**左**移 → FLU 导出为**正**（符合 +Y 左）。这份实录既钉住
传感器安装 Adapter，也防止任何下游 Module 再次翻转 Y。

时间线核对过：那处取反由 `e5d65322` 于 2026-08-30 00:23 引入，实录在同日
19:57 / 20:07，晚 19 小时；此后 `sensor_velocity_m_s` 链路再未改动。作者亦确认
光流机械结构自始未变。

`forward_x` 阶段**不可用**（`axis_dominance_ratio` 只有 1.05、横轴位移 164 mm
对在轴 172 mm，基本是斜着推的），本模块只用 `left_y`（占优 114:1、横轴 4 mm）。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
RECORDING = (ROOT / "data" / "calibration" / "flow_range" / "2026-08-30" /
             "flow_range_20260830_200716.json")
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
FLOW_CMD = ROOT / "App" / "Src" / "app_cmd_flow.c"
FLOW_PAGE = ROOT / "tools" / "panel_lib" / "pages" / "flow_ranging.py"

def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def recording() -> dict:
    if not RECORDING.exists():
        pytest.skip(f"缺少地面实录 {RECORDING}")
    return json.loads(read(RECORDING))


def test_recording_is_the_flu_ground_evidence(recording: dict) -> None:
    """先确认这份实录本身可用，再拿它下结论。"""
    assert recording["format"] == "drone-h743-flow-range-ground-evidence"
    assert recording["body_frame"] == "FLU"

    stage = recording["stages"]["left_y"]
    assert stage["expected_axis"] == "y"
    assert stage["positive_sign_ok"] is True
    # 横移够干净才能当方向证据用：在轴 472 mm，横轴 4 mm。
    assert stage["axis_dominance_ratio"] > 20.0, stage
    assert stage["cross_axis_distance_m"] < 0.05, stage

    for sample in recording["samples"]["left_y"]:
        assert sample["orientation_code"] == 3, "实录不是 FLU 朝向下采的"


def test_moving_left_reads_positive_in_the_flu_export(recording: dict) -> None:
    """面板提示"向机体左侧移动（+Y）"，导出值必须整段为正。"""
    assert '"left_y": "向机体左侧移动（+Y）"' in read(FLOW_PAGE)
    # 实录里的 vy_m_s 就是 FLOW comp 的 sensor_vy_mm_s。
    assert "vy_m_s=safe_int(values.get(\"sensor_vy_mm_s\"), 0) * 0.001," in read(FLOW_PAGE)

    moving = [s["vy_m_s"] for s in recording["samples"]["left_y"]
              if abs(s["vy_m_s"]) > 0.02]
    assert len(moving) >= 5, "运动段样本太少，不足以定符号"
    assert all(v > 0.0 for v in moving), moving


def test_flow_is_converted_once_before_navigation_math() -> None:
    """安装方言只准出现在采集边界，之后全链必须是 FLU。

    2026-09-07：判据从"补偿函数里恰好 4 次 FrdToFlu"改为"稳定器里一次都没有"。
    原判据钉的是缺陷本身——转换发生在补偿之后，Y 因此反号。
    """
    stabilizer = read(STABILIZER)
    flow_app = read(ROOT / "App" / "Src" / "app_optical_flow.c")
    compensation = stabilizer.split(
        "static void stabilizer_compensate_flow_rotation", 1
    )[1].split("static void stabilizer_vofa_debug_publish", 1)[0]

    # 补偿函数进出都是 FLU，自己不做任何转换。
    assert "FrdToFlu" not in compensation
    assert "*flow_vy_m_s = body_vy_m_s;" in compensation
    assert "STABILIZER_VELOCITY_MEAS_Y_SIGN" not in stabilizer

    # 变换在采集边界，且只此一处。
    fill = flow_app.split("static void app_flow_fill_sample")[1].split("\n}")[0]
    assert "-frame->flow_vel_y" in fill
    assert "source=calibrated_body_flu export=canonical_flu" in read(FLOW_CMD)


def test_navigation_controller_and_snapshot_do_not_readapt_y() -> None:
    """转换一次以后，EKF、控制器和快照都只能直接透传 FLU Y。"""
    stabilizer = read(STABILIZER)
    header = read(ROOT / "Driver" / "Inc" / "drv_coax_ctrl.h")
    assert ("+Y left" in header) or ("+Y 左" in header)
    assert "fuse_input.flow_vy_m_s = flow_vy_m_s;" in stabilizer
    assert "ctx->vofa_debug.vel_est_m_s[1] = nav_vy_m_s;" in stabilizer
    assert "velocity_control_y_m_s = nav_vy_m_s;" in stabilizer
    assert "position_state_y_m *= " not in stabilizer
    assert "sensor_velocity_flu_m_s[1] = debug->sensor_velocity_m_s[1]" in stabilizer
