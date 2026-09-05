"""叶页枚举与几何探针。

判据照抄改版报告 §2 的三条，一个字不改，这样“修好了”能和报告里的告警逐条对上：

1. **横向越界** —— 控件右边界越过窗口右边界。横向没有滚动条，越界就是**够不到**。
2. **不可滚动区的纵向越界** —— 祖先里没有 Canvas（也就没有视口）时，下边界越出
   窗口同样是够不到。祖先里有 Canvas 的按可滚动处理，不计入。
3. **已管理但分不到空间** —— 控件被 geometry manager 接管了，实际尺寸却是 1×1，
   而它的 requested 尺寸明显更大。这类控件 `winfo_exists()` 为真、测试里一断言就过，
   用户却根本看不到——报告点名的“不能只查控件存在”说的就是它。

探针只**报告**几何事实，不判定谁该修：同一个告警在“该页本来就没内容”和“停止按钮
被挤出去了”之间是不同的东西，需要人看截图甄别。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator


# 报告用的三种客户区与三档模拟缩放。缩放是注入 DPI 因子的**字体/布局模拟**，
# 不是多显示器真实 DPI 切换验收——后者只能人工做。
WINDOW_SIZES = ((1080, 700), (1366, 768), (1500, 900))
SCALES = (1.0, 1.25, 1.5)

# 只看用户要操作的那几类控件。Label/Frame 越界通常只是文字排版，不是“点不到”。
INTERACTIVE_CLASSES = (
    "TButton", "TEntry", "TSpinbox", "TCombobox", "Treeview", "TCheckbutton",
    "TRadiobutton", "TMenubutton", "TScale", "Button", "Entry", "Spinbox",
    "Checkbutton", "Radiobutton", "Scale", "Listbox",
)

CLIP_TOLERANCE_PX = 3


@dataclass(frozen=True)
class LeafPage:
    """一个真正显示内容的页面。分组页签本身不算，它的子页才算。"""

    label: str
    top: object                     # 顶层 notebook 里的那个 frame
    sub_notebook: object | None     # 分组内层 notebook；None 表示顶层直挂
    sub_tab: str | None
    # 切页由 `OfflinePanel.select()` 驱动：页面描述不持有面板，才能被序列化进报告。


@dataclass
class ClippedControl:
    text: str
    widget_class: str
    path: str
    clipped_x: int = 0
    clipped_y_unscrollable: int = 0
    unallocated: bool = False
    requested: tuple[int, int] | None = None

    def to_json(self) -> dict:
        payload = {
            "text": self.text,
            "class": self.widget_class,
            "path": self.path,
            "clipped_x": self.clipped_x,
            "clipped_y_unscrollable": self.clipped_y_unscrollable,
        }
        if self.unallocated:
            payload["unallocated"] = True
            payload["requested"] = list(self.requested or ())
        return payload


@dataclass
class PageGeometryReport:
    page: str
    size: tuple[int, int]
    scale: float
    switch_ms: float = 0.0
    clipped: list[ClippedControl] = field(default_factory=list)

    @property
    def clean(self) -> bool:
        return not self.clipped

    def to_json(self) -> dict:
        return {
            "page": self.page,
            "size": list(self.size),
            "scale_simulated": self.scale,
            "switch_ms": self.switch_ms,
            "clipped_controls": [c.to_json() for c in self.clipped],
        }


def _managed_descendants(widget) -> Iterator:
    """遍历当前**可见**子树：未选中的 notebook 页和没被 manager 接管的控件不算。"""
    for child in widget.winfo_children():
        if widget.winfo_class() == "TNotebook" and str(child) != widget.select():
            continue
        if not child.winfo_manager():
            continue
        yield child
        yield from _managed_descendants(child)


def _widget_text(widget) -> str:
    for option in ("text", "textvariable"):
        try:
            value = widget.cget(option)
        except Exception:                       # noqa: BLE001 - 该控件没这个选项
            continue
        if not value:
            continue
        if option == "text":
            return str(value)
        try:
            return str(widget.tk.globalgetvar(value))
        except Exception:                       # noqa: BLE001
            return ""
    return ""


def _inside_canvas(widget) -> bool:
    node = getattr(widget, "master", None)
    while node is not None:
        if node.winfo_class() == "Canvas":
            return True
        node = getattr(node, "master", None)
    return False


def collect_clipped_controls(root, page_root) -> list[ClippedControl]:
    """在**当前**窗口尺寸下，找出这一页够不到的交互控件。"""
    found: list[ClippedControl] = []
    for widget in _managed_descendants(page_root):
        if widget.winfo_class() not in INTERACTIVE_CLASSES:
            continue
        width, height = widget.winfo_width(), widget.winfo_height()
        if width <= 1 or height <= 1:
            requested = (widget.winfo_reqwidth(), widget.winfo_reqheight())
            if requested[0] > 8 and requested[1] > 8:
                found.append(ClippedControl(
                    text=_widget_text(widget), widget_class=widget.winfo_class(),
                    path=str(widget), unallocated=True, requested=requested,
                ))
            continue
        x = widget.winfo_rootx() - root.winfo_rootx()
        y = widget.winfo_rooty() - root.winfo_rooty()
        clipped_x = max(0, x + width - root.winfo_width())
        clipped_y = max(0, y + height - root.winfo_height())
        if _inside_canvas(widget):
            clipped_y = 0                        # 有视口，能滚到
        if clipped_x > CLIP_TOLERANCE_PX or clipped_y > CLIP_TOLERANCE_PX:
            found.append(ClippedControl(
                text=_widget_text(widget), widget_class=widget.winfo_class(),
                path=str(widget), clipped_x=clipped_x, clipped_y_unscrollable=clipped_y,
            ))
    return found


def probe_geometry(offline_panel, *, sizes=WINDOW_SIZES) -> list[PageGeometryReport]:
    """在给定尺寸下逐页测量。缩放由 `OfflinePanel.launch(scale=...)` 决定。"""
    import time

    reports: list[PageGeometryReport] = []
    pages = offline_panel.leaf_pages()
    for width, height in sizes:
        offline_panel.resize(width, height)
        for page in pages:
            started = time.perf_counter()
            offline_panel.select(page)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            reports.append(PageGeometryReport(
                page=page.label,
                size=(offline_panel.panel.winfo_width(), offline_panel.panel.winfo_height()),
                scale=offline_panel.scale,
                switch_ms=elapsed_ms,
                clipped=collect_clipped_controls(offline_panel.panel, page.top),
            ))
    return reports


__all__ = [
    "CLIP_TOLERANCE_PX",
    "INTERACTIVE_CLASSES",
    "SCALES",
    "WINDOW_SIZES",
    "ClippedControl",
    "LeafPage",
    "PageGeometryReport",
    "collect_clipped_controls",
    "probe_geometry",
]
