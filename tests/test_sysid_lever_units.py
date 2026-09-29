"""候选增益的力矩单位换算（2026-09-27 评审确认的安全问题）。

候选增益是录制那轮固件的力矩单位：速率环输出 N·m，固件按自己的力臂 L 换成舵机倾角。
旧力矩模型（力臂 0.145 × 有效系数：横滚 0.0842、俯仰 0.0825 m）录的实录合成的增益，
直接写进今晚的几何模型（L = cg − 舵机转轴 = 0.035442 m）会硬 2.3 倍；重心按实测改到
−0.01 后（L = 0.12 m）又会软到 0.69 倍。这里用**实录的参数回显**当"录制时"，用合成的
几何飞控当"当前连着的板子"，钉住换算倍数与拒绝条件。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DAY = ROOT / "data" / "identification" / "attitude" / "2026-09-27"
LEGACY_RUN = DAY / "rod_012332_422504f4"

sys.path.insert(0, str(ROOT / "tools"))
try:
    from panel_lib.pages.sysid import lever_units
finally:
    sys.path.pop(0)


def geometric_board(cg_z_m: float, axis_z_m: float = -0.13) -> dict:
    """几何力矩模型的飞控参数回显：PARAM? 的样子（没有旧力臂两项）。"""
    return {"airframe.cg_z_m": f"{cg_z_m:.6f}", "airframe.servo1_axis_z_m": f"{axis_z_m:.6f}",
            "airframe.servo2_axis_z_m": f"{axis_z_m:.6f}", "airframe.imu_z_m": "0.000000"}


#: 今晚板上的几何（部件表重心 −0.094558）与明天按实测重心修正后的几何。
TONIGHT = geometric_board(-0.094558)
CG_FIXED = geometric_board(-0.01)
CANDIDATES = [("coax.rate_pitch_kp", "0.3"), ("coax.rate_pitch_ki", "0.1"),
              ("coax.rate_pitch_kd", "0.002"), ("coax.att_pitch_kp", "0.42")]


@pytest.fixture(scope="module")
def legacy_echo():
    if not (LEGACY_RUN / "conditions.json").is_file():
        pytest.skip("实录不在本机")
    return json.loads((LEGACY_RUN / "conditions.json").read_text(encoding="utf-8"))["parameter_echo"]


def test_the_real_run_records_legacy_torque_units(legacy_echo):
    record = lever_units.torque_record(legacy_echo)
    assert record["signature"] == "legacy"
    assert record["levers_m"] == pytest.approx([0.145 * 0.581, 0.145 * 0.569])


def test_the_geometric_board_levers_are_cg_minus_axis():
    assert lever_units.torque_record(TONIGHT) == {
        "signature": "geometric", "levers_m": pytest.approx([0.035442, 0.035442])}
    assert lever_units.torque_record(CG_FIXED)["levers_m"] == pytest.approx([0.12, 0.12])


@pytest.mark.parametrize("board, factor", [(TONIGHT, 0.035442 / (0.145 * 0.569)),
                                           (CG_FIXED, 0.12 / (0.145 * 0.569))])
def test_legacy_candidates_are_scaled_by_the_lever_ratio(legacy_echo, board, factor):
    """kp/ki/kd × L_now/L_src（今晚 ×0.43：不换算就硬 2.3 倍）；姿态 P 输出角速度，不换。"""
    converted, note = lever_units.convert(CANDIDATES, lever_units.torque_record(legacy_echo),
                                          lever_units.torque_record(board))
    values = dict(converted)
    assert float(values["coax.rate_pitch_kp"]) == pytest.approx(0.3 * factor, rel=1e-5)
    assert float(values["coax.rate_pitch_ki"]) == pytest.approx(0.1 * factor, rel=1e-5)
    assert float(values["coax.rate_pitch_kd"]) == pytest.approx(0.002 * factor, rel=1e-5)
    assert values["coax.att_pitch_kp"] == "0.42"
    assert f"×{factor:.3f}" in note and "0.0825 →" in note and "姿态环 P" in note
    assert "旧力矩模型" in note and "几何力矩模型" in note


def test_each_axis_uses_its_own_lever(legacy_echo):
    pairs = [("coax.rate_roll_kp", "1"), ("coax.rate_pitch_kp", "1")]
    converted = dict(lever_units.convert(pairs, lever_units.torque_record(legacy_echo),
                                         lever_units.torque_record(TONIGHT))[0])
    assert float(converted["coax.rate_roll_kp"]) == pytest.approx(0.035442 / (0.145 * 0.581), rel=1e-5)
    assert float(converted["coax.rate_pitch_kp"]) == pytest.approx(0.035442 / (0.145 * 0.569), rel=1e-5)


def test_the_same_units_are_left_alone():
    record = lever_units.torque_record(TONIGHT)
    converted, note = lever_units.convert(CANDIDATES, record, record)
    assert converted == CANDIDATES
    assert "不用换算" in note


@pytest.mark.parametrize("source, target, message", [
    (None, TONIGHT, "录制时的固件倾转力臂读不出来"),
    ({}, TONIGHT, "录制时的固件倾转力臂读不出来"),
    ("legacy", {}, "读不到当前飞控的倾转力臂"),
    ("legacy", geometric_board(-0.094558, axis_z_m=0.0), "读不到当前飞控的倾转力臂"),
    ("legacy", geometric_board(-0.2, axis_z_m=-0.13), "符号相反"),
])
def test_unknown_or_reversed_levers_are_refused(legacy_echo, source, target, message):
    source_record = (lever_units.torque_record(legacy_echo) if source == "legacy"
                     else None if source is None else lever_units.torque_record(source))
    with pytest.raises(ValueError, match=message):
        lever_units.convert(CANDIDATES, source_record, lever_units.torque_record(target))


def test_attitude_only_candidates_need_no_lever():
    converted, note = lever_units.convert([("coax.att_pitch_kp", "0.4")], None, None)
    assert converted == [("coax.att_pitch_kp", "0.4")] and note == ""
