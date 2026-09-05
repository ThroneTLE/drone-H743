"""Local-input vertical viewports used by long panel pages."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .theme import UI_PALETTE


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
        self.bind_class(self._bindtag, "<MouseWheel>", self._on_mousewheel)
        for sequence in ("<KeyPress-Prior>", "<KeyPress-Next>",
                         "<KeyPress-Home>", "<KeyPress-End>"):
            self.bind_class(self._bindtag, sequence, self._on_keypress)
        self.content.bind("<Configure>", self._sync_scroll_region)
        self.canvas.bind("<Configure>", self._sync_width)
        # The canvas itself is a useful inspection seam and a direct fallback for
        # synthetic events; descendants use the private bindtag below.
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Enter>", self._focus_viewport)
        self.content.bind("<Enter>", self._focus_viewport)
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
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        direct_children = tuple(str(child) for child in self.content.winfo_children())
        if direct_children != self._bound_direct_children:
            self._bound_direct_children = direct_children
            self._bindtag_to(self.content)

    def _sync_width(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self._window, width=event.width)

    def _focus_viewport(self, _event: tk.Event) -> None:
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
        self.canvas.configure(scrollregion=(0, 0, 0, max(height, 1)))

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


__all__ = ["FixedActionViewport", "VerticalScrolledFrame"]
