"""「XY 速度 / 位置环」页结果区文字：跑完一轮（或点「重新分析」）当场分析并拼成几行中文。

tilt 轮给加速度增益、动摩擦、静摩擦门槛角、光流滞后；vel/pos 轮给阶跃指标
（`tools/sysid/xy_analysis.py`，经 `_core`）。算不出来只写一句原因，不影响存档。
"""
from __future__ import annotations

from ._core import xy_analysis, xy_summary
from .xy_config import OFFLINE_NOTE, provenance_text


def xy_analysis_text(snapshot: dict | None, times_s, samples) -> str:
    """分析摘要；算不出来返回一句原因（不抛）。"""
    try:
        return xy_summary(xy_analysis(times_s, samples, snapshot or {}))
    except (ValueError, TypeError, KeyError, IndexError, ZeroDivisionError) as error:
        return f"水平槽分析算不出来：{error}"


def xy_result_text(snapshot: dict | None, times_s, samples, quality_note: str,
                   vibration: str = "") -> str:
    """结果区：溯源 → 分析摘要 → 说明 → 数据质量 → 振动。"""
    return "\n".join(part for part in (provenance_text(snapshot),
                                       xy_analysis_text(snapshot, times_s, samples),
                                       OFFLINE_NOTE, quality_note, vibration) if part)


__all__ = ["xy_analysis_text", "xy_result_text"]
