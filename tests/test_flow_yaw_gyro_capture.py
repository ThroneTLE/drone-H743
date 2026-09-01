"""M5 台架实测暴露的缺陷：旋转补偿阶永远拿不到偏航角速度。

2026-08-30 现场证据：`flow_range_20260830_200716.json` 里 299 个实采样本，
`gyro_z_dps` **无一非 null**，第六阶段因此恒定报"样本为空或包含非有限数值"。

根因是上位机解析器认的键名和固件发的对不上——固件按 `units=mg_mdps_cdeg`
发裸 `gz`（mdps），解析器只认 `gz_dps` / `gz_mdps`，两个名字固件从未发过。
这类"名字对不上"的缺陷不会报错，只会安静地让一整阶验收永远做不完，所以这里
直接拿**固件源码里的真实格式串**当基准钉死，而不是自己编一行假报文。
"""

from __future__ import annotations

from pathlib import Path
import re

import pytest

from tools import drone_tcp_panel as panel


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_SAMPLE_LINE = (
    "IMU sample seq=1855573 ax=18 ay=-12 az=1016 gx=34 gy=38 gz=-17500 "
    "roll=-1 pitch=-5 yaw=3621 fusion_flags=0x00 ferr_cdeg=3 ftrig_milli=0 fcorr=1854574"
)


def test_firmware_still_emits_bare_gz_in_mdps() -> None:
    """基准取自固件源码：格式串一旦改名，这里先红，而不是等台架上再发现。"""
    source = (ROOT / "App" / "Src" / "app_control.c").read_text(encoding="utf-8")
    assert '"IMU sample seq=%lu ax=%ld ay=%ld az=%ld gx=%ld gy=%ld gz=%ld' in source
    assert "units=mg_mdps_cdeg" in source
    assert "gz_dps" not in source and "gz_mdps" not in source
    # 真实报文的键名集合必须覆盖解析器要读的那个键。
    keys = set(re.findall(r"(\w+)=", FIRMWARE_SAMPLE_LINE))
    assert "gz" in keys and "gz_dps" not in keys and "gz_mdps" not in keys


class _Holder:
    """只提供被测分支要用的状态；其余下游调用一律吞掉，免得拉起整个 Tk。"""

    def __init__(self) -> None:
        self.flow_latest_gyro_z_dps: float | None = None

    def __getattr__(self, name: str):
        return lambda *args, **kwargs: None


def capture(line: str) -> float | None:
    holder = _Holder()
    panel.DronePanel._update_imu_line(holder, line)
    return holder.flow_latest_gyro_z_dps


@pytest.mark.parametrize(
    ("line", "expected_dps"),
    [
        (FIRMWARE_SAMPLE_LINE, -17.5),
        ("IMU sample gz_mdps=15000", 15.0),
        ("IMU sample gz_dps=22.5", 22.5),
    ],
)
def test_every_known_gyro_spelling_lands_in_dps(line: str, expected_dps: float) -> None:
    assert capture(line) == pytest.approx(expected_dps)


def test_yaw_stage_can_reach_the_15_dps_gate_from_a_real_line() -> None:
    """光靠"解析到了"不够——要确认量纲对，否则 15 dps 的门永远够不着。"""
    # gz=-17500 mdps = -17.5 dps，绝对值已越过 R-M5-4 的 15 dps 动作门槛。
    assert abs(capture(FIRMWARE_SAMPLE_LINE)) >= 15.0
