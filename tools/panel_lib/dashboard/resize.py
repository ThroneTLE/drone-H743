"""状态监视工作台的窗口 resize 合并。

拖动系统窗口边框会产生连续的 ``<Configure>`` 事件。卡片使用 12 列整数网格，
所以大量像素级事件并不改变任一张卡的几何；若每一拍都对所有 tile ``place()``，
再与 33 ms 波形重绘争抢 Tk 主线程，就会把窗口拖动变成卡顿。

本协调器不认识 Tk widget，只要求页面提供 ``after``、当前格宽、重排与编辑覆盖层
重定位四个窄接口。因此计时与合并规则可在无显示环境下直接测试。
"""

from __future__ import annotations

from typing import Any


class DashboardResizeCoordinator:
    """合并 resize：格宽未变零重排、突发事件最多每 16 ms 布局一次。

    数据接收和环形缓冲从不暂停；``render_suspended`` 只让 Tk 绘图短暂停下，
    等边框停止移动后立刻以最新快照画下一帧，因此不会丢遥测数据。
    """

    RELAYOUT_DELAY_MS = 16
    RESIZE_QUIET_MS = 150

    def __init__(self, host) -> None:
        self.host = host
        self.render_suspended = False
        self._last_cell_width: int | None = None
        self._relayout_token: Any | None = None
        self._quiet_token: Any | None = None

    def mark_laid_out(self) -> None:
        """供立即布局路径标记当前格宽，避免紧随其后的 Configure 重复工作。"""
        self._last_cell_width = int(self.host.dashboard_cell_width())

    def note_configure(self) -> None:
        """记录一个窗口尺寸事件；耗时布局留给合并后的延迟回调。"""
        self.render_suspended = True
        self._restart_quiet_timer()

        cell_width = int(self.host.dashboard_cell_width())
        if cell_width == self._last_cell_width:
            return
        self._last_cell_width = cell_width
        if self._relayout_token is None:
            self._relayout_token = self.host.after(
                self.RELAYOUT_DELAY_MS, self._apply_relayout
            )

    def _apply_relayout(self) -> None:
        self._relayout_token = None
        self.host.dashboard_relayout()
        self.host.dashboard_reposition_overlays()

    def _restart_quiet_timer(self) -> None:
        if self._quiet_token is not None:
            self.host.after_cancel(self._quiet_token)
        self._quiet_token = self.host.after(self.RESIZE_QUIET_MS, self._end_resize)

    def _end_resize(self) -> None:
        self._quiet_token = None
        self.render_suspended = False


__all__ = ["DashboardResizeCoordinator"]
