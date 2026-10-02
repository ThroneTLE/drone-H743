"""「偏航（吊绳）」页结果区文字：跑完一轮（或点「重新分析」）当场分析并拼成几行中文。

diff 轮给 b=1/Izz、k 实测/k 模型、阻尼、绳扭转刚度、延迟、偏航角速度环建议增益与悬停最大偏航力矩；
rate 轮给阶跃指标与饱和占比（`tools/sysid/yaw_analysis.py`，经 `_core`）。算不出来只写一句原因，不影响存档。
"""
from __future__ import annotations

from ._core import yaw_analysis, yaw_summary
from .yaw_config import OFFLINE_NOTE, provenance_text


def yaw_analysis_text(snapshot: dict | None, times_s, samples) -> str:
    """分析摘要；算不出来返回一句原因（不抛）。"""
    try:
        return yaw_summary(yaw_analysis(times_s, samples, snapshot or {}))
    except (ValueError, TypeError, KeyError, IndexError, ZeroDivisionError) as error:
        return f"吊绳偏航分析算不出来：{error}"


def yaw_result_text(snapshot: dict | None, times_s, samples, quality_note: str,
                    vibration: str = "") -> str:
    """结果区：溯源 → 分析摘要 → 说明 → 数据质量 → 振动。"""
    return "\n".join(part for part in (provenance_text(snapshot),
                                       yaw_analysis_text(snapshot, times_s, samples),
                                       OFFLINE_NOTE, quality_note, vibration) if part)


__all__ = ["yaw_analysis_text", "yaw_result_text"]
