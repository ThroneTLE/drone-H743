"""光流横向方向：用 2026-08-30 的地面实录把控制器系 Y 的正方向钉死。

R-F6-2 卡在一个问题上：控制器系的 `y_m` / `vy_m_s` 到底是机体**右**正还是
**左**正。仓库里五处注释说"右正"，`15d83454` 的交付改口说"一直是左正"，两边
都只有措辞、没有签名证据，而这两种答案会导出方向相反的控制器。

`data/calibration/flow_range/2026-08-30/` 里有现成的答案，是 R-F5 期间录的地面
标定，`body_frame=FLU`、`orientation_code=3`：

  * 面板的 `left_y` 阶段提示语是"向机体左侧移动（+Y）"——**物理方向已知**；
  * 该阶段记录的 `vy_m_s` 取自 `FLOW comp` 的 `sensor_vy_mm_s`，也就是
    `snapshot.sensor_velocity_flu_m_s[1]`（导出前已声明 `export=canonical_flu`）；
  * `app_stabilizer.c` 生成该字段时做了取反：
    `sensor_velocity_flu_m_s[1] = -debug->sensor_velocity_m_s[1]`。

于是链条闭合：向**左**移 → FLU 导出为**正**（符合 +Y 左）→ 取反前的控制器系
Y 为**负** → **控制器系 Y 是右正**。

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

FLU_EXPORT_NEGATION = (
    "sensor_velocity_flu_m_s[1] = -debug->sensor_velocity_m_s[1]"
)


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


def test_controller_frame_y_is_right_positive() -> None:
    """闭合链条：向左 → FLU 正 → 取反前的控制器系 Y 为负 → 控制器系 Y 右正。

    这条钉住的是**取反本身存在**。它在，`sensor_vy_mm_s` 才等于
    −(控制器系 Y)，上面那条实录断言才能推出"控制器系 Y 右正"。
    谁要改这个结论，得先解释这一行为什么可以删。
    """
    stabilizer = read(STABILIZER)
    assert FLU_EXPORT_NEGATION in stabilizer
    assert "corrected_velocity_flu_m_s[1] = -debug->corrected_velocity_m_s[1]" in stabilizer
    # 固件自己也这么标注，且这条串是评审与实录的溯源依据。
    assert "source=controller_legacy_x_forward_y_right export=canonical_flu" in read(FLOW_CMD)


def test_lateral_migration_sign_matches_the_declared_frame() -> None:
    """seam 3 若声称位置/速度是规范 FLU（+Y 左），横向适配符号必须跟着翻。

    控制器系 Y 是右正（上面三条已证），所以把 `Reference`/`AttitudeInput` 的
    `y_m`/`vy_m_s` 迁到规范 FLU（+Y 左）**必须**同时把 seam2→seam3 的适配器
    `STABILIZER_VELOCITY_MEAS_Y_SIGN` 由 `+1` 改成 `-1`；只改注释不改符号，
    等于把一个右正的量贴上"左正"的标签。

    这一条是 R-F6-2 审核退回的第 ④ 项：`15d83454` 只改了标签。
    """
    stabilizer = read(STABILIZER)
    header = read(ROOT / "Driver" / "Inc" / "drv_coax_ctrl.h")

    sign_is_plus_one = (
        "#define STABILIZER_VELOCITY_MEAS_Y_SIGN (1.0f)" in stabilizer
    )
    header_claims_flu_left = ("+Y left" in header) or ("+Y 左" in header)

    assert not (sign_is_plus_one and header_claims_flu_left), (
        "drv_coax_ctrl.h 声称位置/速度是规范 FLU（+Y 左），但 "
        "STABILIZER_VELOCITY_MEAS_Y_SIGN 仍是 +1，喂进来的是右正量。"
        "两者必须同时成立或同时不成立——要么把符号改成 -1 完成横向迁移，"
        "要么把头文件改回「legacy 右正」。见 "
        "doc/req-rf6-2-controller-flu-migration.md 第 3 节。"
    )
