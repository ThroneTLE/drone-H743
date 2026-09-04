"""状态监视工作台的窗口 resize 合并器。

这个层不能靠肉眼判“似乎顺了一点”：Tk 拖动窗口时会连续发出大量 Configure
事件。这里用无 Tk 的假调度器钉住三件事：同一个 12 列格宽的像素抖动不重排、
多个事件合成一次布局、安静窗口结束前不恢复波形重绘。
"""

from __future__ import annotations

from tools.panel_lib.dashboard.resize import DashboardResizeCoordinator


class FakeScheduler:
    def __init__(self) -> None:
        self._next = 0
        self.pending: dict[int, tuple[int, object]] = {}

    def after(self, delay_ms: int, callback):
        self._next += 1
        self.pending[self._next] = (delay_ms, callback)
        return self._next

    def after_cancel(self, token: int) -> None:
        self.pending.pop(token, None)

    def run_delay(self, delay_ms: int) -> None:
        due = [token for token, (delay, _callback) in self.pending.items()
               if delay == delay_ms]
        for token in due:
            _delay, callback = self.pending.pop(token)
            callback()


class FakeDashboard(FakeScheduler):
    def __init__(self) -> None:
        super().__init__()
        self.cell_width = 80
        self.relayouts = 0
        self.overlay_repositions = 0

    def dashboard_cell_width(self) -> int:
        return self.cell_width

    def dashboard_relayout(self) -> None:
        self.relayouts += 1

    def dashboard_reposition_overlays(self) -> None:
        self.overlay_repositions += 1


def test_resize_skips_pixel_events_that_do_not_change_a_grid_cell() -> None:
    host = FakeDashboard()
    coordinator = DashboardResizeCoordinator(host)
    coordinator.mark_laid_out()

    coordinator.note_configure()

    assert host.relayouts == 0
    assert not any(delay == coordinator.RELAYOUT_DELAY_MS
                   for delay, _callback in host.pending.values())
    assert coordinator.render_suspended


def test_resize_coalesces_multiple_grid_changes_into_one_layout_pass() -> None:
    host = FakeDashboard()
    coordinator = DashboardResizeCoordinator(host)
    coordinator.mark_laid_out()

    host.cell_width = 81
    coordinator.note_configure()
    host.cell_width = 82
    coordinator.note_configure()
    host.cell_width = 83
    coordinator.note_configure()

    assert sum(delay == coordinator.RELAYOUT_DELAY_MS
               for delay, _callback in host.pending.values()) == 1
    host.run_delay(coordinator.RELAYOUT_DELAY_MS)
    assert host.relayouts == 1
    assert host.overlay_repositions == 1
    assert coordinator.render_suspended


def test_rendering_resumes_only_after_the_resize_quiet_window() -> None:
    host = FakeDashboard()
    coordinator = DashboardResizeCoordinator(host)
    coordinator.mark_laid_out()

    host.cell_width = 81
    coordinator.note_configure()
    host.run_delay(coordinator.RELAYOUT_DELAY_MS)
    assert coordinator.render_suspended

    host.run_delay(coordinator.RESIZE_QUIET_MS)
    assert not coordinator.render_suspended
