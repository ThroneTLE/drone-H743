"""Optional matplotlib backend for the panel pages.

matplotlib 是可选依赖：没装也必须能开面板，只是曲线区停用。守卫放在这里，
让大面板和 panel_lib/pages/* 共用同一份判定，不出现两处各自 try/except。
"""

from __future__ import annotations

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure

    HAS_MATPLOTLIB = True
    MATPLOTLIB_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on local optional package
    FigureCanvasTkAgg = None  # type: ignore[assignment]
    Figure = None  # type: ignore[assignment]
    HAS_MATPLOTLIB = False
    MATPLOTLIB_ERROR = str(exc)


__all__ = [
    "FigureCanvasTkAgg",
    "Figure",
    "HAS_MATPLOTLIB",
    "MATPLOTLIB_ERROR",
]
