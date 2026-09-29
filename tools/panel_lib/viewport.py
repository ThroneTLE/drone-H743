"""Local-input vertical viewports used by long panel pages."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .theme import UI_PALETTE

# Widget classes that own the keyboard while the user is typing or picking.
# Hovering a viewport must never pull focus out of one of these, or the next
# keystrokes land on the canvas and the user's input silently disappears.
KEYBOARD_OWNER_CLASSES = frozenset(
    {
        "TEntry",
        "Entry",
        "TSpinbox",
        "Spinbox",
        "TCombobox",
        "Text",
        "Listbox",
    }
)


def leaf_tab_visible(panel, leaf, outer_selection: str) -> bool:
    """叶页此刻是不是真的露在前台——不管它挂在顶层还是某个二级分组里。

    轮询/重绘的闸门都拿"这一页看得见吗"当条件。页面被收进二级 Notebook 之后，
    顶层 selection 变成了**分组**，再拿它和叶页比就永远为假：页面看着是开的，
    数据却不再刷新，而且没有任何报错——`sensor_notebook` 那次就是这么停摆的。

    这里按 widget 的实际父子关系判断，所以以后再收一层分组也不用改调用方。
    """
    if leaf is None:
        return False
    holder = getattr(leaf, "master", None)
    top = getattr(panel, "notebook", None)
    if (holder is None) or (top is None) or (str(holder) == str(top)):
        return outer_selection == str(leaf)
    return (outer_selection == str(getattr(holder, "master", ""))) and (
        str(holder.select()) == str(leaf)
    )


class VerticalScrolledFrame(ttk.Frame):
    """A width-following viewport with local wheel and keyboard routing.

    Bindings are attached to this widget's descendants through a private bindtag;
    no ``bind_all`` state is used, so another page cannot steal the wheel when a
    viewport is hidden or destroyed.
    """

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, style="Page.TFrame")
        self.canvas = tk.Canvas(
            self, background=UI_PALETTE["surface"], highlightthickness=0, borderwidth=0
        )
        self.scrollbar = ttk.Scrollbar(
            self, orient=tk.VERTICAL, command=self.canvas.yview
        )
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.content = ttk.Frame(
            self.canvas, padding=(14, 14, 10, 18), style="Page.TFrame"
        )
        self._window = self.canvas.create_window(
            (0, 0), window=self.content, anchor=tk.NW
        )
        self._bindtag = f"PanelViewport_{id(self):x}"
        self._destroyed = False
        self._bound_direct_children: tuple[str, ...] = ()
        self._scroll_region: tuple[int, int, int, int] | None = None
        self._window_width: int | None = None
        self.bind_class(self._bindtag, "<MouseWheel>", self._on_mousewheel)
        for sequence in ("<KeyPress-Prior>", "<KeyPress-Next>",
                         "<KeyPress-Home>", "<KeyPress-End>"):
            self.bind_class(self._bindtag, sequence, self._on_keypress)
        self.content.bind("<Configure>", self._sync_scroll_region)
        self.canvas.bind("<Configure>", self._sync_width)
        # The canvas itself is a useful inspection seam and a direct fallback for
        # synthetic events; descendants use the private bindtag below.
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        # Hover grants the keyboard only when nobody is typing; a click always
        # claims it. Wheel routing goes through the bindtag above and needs no
        # focus at all -- only PageUp/PageDown/Home/End do.
        self.canvas.bind("<Enter>", self._focus_viewport)
        self.content.bind("<Enter>", self._focus_viewport)
        self.canvas.bind("<Button-1>", self._claim_focus)
        self.content.bind("<Button-1>", self._claim_focus)
        self._bindtag_to(self.canvas)
        self._bindtag_to(self.content)

    def _bindtag_to(self, widget: tk.Misc) -> None:
        if self._destroyed:
            return
        tags = widget.bindtags()
        if self._bindtag not in tags:
            widget.bindtags((self._bindtag, *tags))
        for child in widget.winfo_children():
            self._bindtag_to(child)

    def _sync_scroll_region(self, _event: tk.Event | None = None) -> None:
        # Writing an unchanged scrollregion still dirties the canvas and costs a
        # redraw, and a page switch delivers a whole burst of <Configure>.  Only
        # write when the value actually moved.
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        direct_children = tuple(str(child) for child in self.content.winfo_children())
        if direct_children != self._bound_direct_children:
            self._bound_direct_children = direct_children
            self._bindtag_to(self.content)

    def _sync_width(self, event: tk.Event) -> None:
        # Same reason, and this one matters more: itemconfigure reflows the
        # embedded frame, whose <Configure> comes straight back here.  Writing
        # the width it already has turns one resize into a relayout ping-pong.
        self.canvas.itemconfigure(self._window, width=event.width)

    def keyboard_owner_class(self) -> str:
        """Class of whatever currently owns the keyboard, ``""`` if nothing does.

        Reads the raw Tcl focus path instead of ``focus_get()``: ttk popdowns
        are Tcl-only windows absent from Tkinter's children table, and
        ``focus_get()`` raises ``KeyError`` trying to reify them (same reason
        as ``dashboard/tiles.py``).
        """
        try:
            path = str(self.tk.call("focus"))
            if not path or path == "none":
                return ""
            return str(self.tk.call("winfo", "class", path))
        except tk.TclError:
            return ""

    def _focus_viewport(self, _event: tk.Event | None = None) -> None:
        if self.keyboard_owner_class() in KEYBOARD_OWNER_CLASSES:
            return
        self.canvas.focus_set()

    def _claim_focus(self, _event: tk.Event | None = None) -> None:
        self.canvas.focus_set()

    def _on_mousewheel(self, event: tk.Event) -> str:
        delta = int(getattr(event, "delta", 0))
        if delta:
            self.canvas.yview_scroll(-1 if delta > 0 else 1, "units")
        return "break"

    def _on_keypress(self, event: tk.Event) -> str:
        key = str(getattr(event, "keysym", ""))
        if key == "Prior":
            self.canvas.yview_scroll(-1, "pages")
        elif key == "Next":
            self.canvas.yview_scroll(1, "pages")
        elif key == "Home":
            self.canvas.yview_moveto(0.0)
        elif key == "End":
            self.canvas.yview_moveto(1.0)
        return "break"

    def set_content_height(self, height: int) -> None:
        """Set a minimum scrollable height for pages that place children with ``place``."""
        self.content.configure(height=max(height, 1))
        self._scroll_region = (0, 0, 0, max(height, 1))
        self.canvas.configure(scrollregion=self._scroll_region)

    def viewport_width(self) -> int:
        width = int(self.canvas.winfo_width())
        return width if width > 1 else 1200

    def destroy(self) -> None:
        if not self._destroyed:
            self._destroyed = True
            try:
                for sequence in ("<MouseWheel>", "<KeyPress-Prior>", "<KeyPress-Next>",
                                 "<KeyPress-Home>", "<KeyPress-End>"):
                    self.unbind_class(self._bindtag, sequence)
            except tk.TclError:
                pass
        super().destroy()


class FixedActionViewport(ttk.Frame):
    """A page with a visible action strip above a scrollable content region."""

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, style="Page.TFrame")
        self.fixed = ttk.Frame(self, style="Page.TFrame")
        self.fixed.pack(fill=tk.X, padx=14, pady=(14, 6))
        self.viewport = VerticalScrolledFrame(self)
        self.viewport.pack(fill=tk.BOTH, expand=True)
        self.content = self.viewport.content

    @property
    def canvas(self):
        return self.viewport.canvas


__all__ = ["FixedActionViewport", "KEYBOARD_OWNER_CLASSES", "VerticalScrolledFrame"]
