"""推力台标定：称重、扫描编排、转速来源、拟合、产出。

**为什么重写**（旧的是 `tools/pressure_rs485_gui.py`，1837 行单文件）：

* 转速是 `kv * voltage * pct/100` **算**出来的（`:250`），报告里却和实测的
  推力并排显示，看报告的人分不出哪个是量的哪个是猜的。本包里每个转速来源
  都带一个 `quality`，报告必须印。
* **没有按电池电压分层**。同一档油门在 12.6 V 和 11.1 V 下推力能差 20%，
  不分层的标定表只在"当时那块电池那个电量"下成立。
* 采集、拟合、界面全混在一个 Tk 类里，任何一条判据都没法离线复核。
  本包无 Tk 部分可单测（`tests/test_thrust_bench.py`），`ui.py` 只做编排与显示。

实机采集会话写 `data/identification/thrust/YYYY-MM-DD/<时分秒>-<id>/`（2026-09-22 起已有
实测会话），跨次实验库是 `data/identification/thrust/experiments.sqlite3`，整批模型导出写
`data/identification/thrust/models/YYYY-MM-DD/training-*/`。
"""

from .rpm import DshotErpm, KvEstimate, NoRpm, RpmSource
from .scale import LoadCell, ScaleCalibration
from .sweep import SweepPlan, SweepPoint, plan_sweep

__all__ = [
    "DshotErpm", "KvEstimate", "NoRpm", "RpmSource",
    "LoadCell", "ScaleCalibration",
    "SweepPlan", "SweepPoint", "plan_sweep",
]
