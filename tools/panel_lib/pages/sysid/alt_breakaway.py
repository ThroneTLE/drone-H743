"""「Z 高度」页高度轮的结果区文字；break 轮（离地/滑落阈值）当场算阈值。

跑完一轮（或点「重新分析」）时，用内存里的样本与本轮记下的 `SYSID PHASE` 回报算离地/滑落推力
（`tools/sysid/breakaway.py`，经 `_core`），和溯源、数据质量说明拼成结果区那几行。
vel/pos 闭环轮不算阈值，仍写离线分析的说明。算不出来只写一句原因，不影响存档。
"""
from __future__ import annotations

from ._core import breakaway_analysis, breakaway_summary
from .alt_config import BREAK_NOTE, OFFLINE_NOTE, provenance_text


def is_breakaway(snapshot: dict | None) -> bool:
    return ((snapshot or {}).get("alt_request") or {}).get("control") == "breakaway"


def breakaway_text(snapshot: dict | None, times_s, samples) -> str:
    """break 轮的阈值摘要；不是 break 轮返回空串，算不出来返回一句原因（不抛）。"""
    if not is_breakaway(snapshot):
        return ""
    try:
        return breakaway_summary(breakaway_analysis(times_s, samples, snapshot))
    except (ValueError, TypeError, KeyError, IndexError, ZeroDivisionError) as error:
        return f"离地/滑落阈值算不出来：{error}"


def alt_result_text(snapshot: dict | None, times_s, samples, quality_note: str,
                    vibration: str = "") -> str:
    """结果区：溯源 → 阈值摘要（break）→ 说明 → 数据质量 → 振动。"""
    summary = breakaway_text(snapshot, times_s, samples)
    note = BREAK_NOTE if summary else OFFLINE_NOTE
    return "\n".join(part for part in (provenance_text(snapshot), summary, note, quality_note,
                                       vibration) if part)


__all__ = ["alt_result_text", "breakaway_text", "is_breakaway"]
