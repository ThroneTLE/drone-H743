"""编辑模式：拖动、缩放、增删、属性。

编辑模式下每个 tile 上面盖一层**透明覆盖层**，拖动和手柄都绑在覆盖层上。
不这么做的话，拖动事件会被 tile 内部的滑块、下拉框、画布抢走——用户想挪一张
参数卡，结果把参数改了。退出编辑模式覆盖层整体销毁，组件恢复完全可交互。

所有几何计算都落到 `layout.py` 的纯函数上（吸附、越界钳制、重叠拒绝），
这里只负责"把鼠标位移换算成格子数"。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

from .layout import TileSpec, can_place, clamp_tile


EDIT_HANDLE_SIZE = 14


class TileOverlay:
    """一个 tile 的编辑覆盖层：整块可拖动，右下角手柄可缩放，右上角两个按钮。"""

    def __init__(self, editor: "DashboardEditor", spec: TileSpec) -> None:
        self.editor = editor
        self.spec = spec
        self.frame = tk.Frame(
            editor.container, highlightthickness=2,
            highlightbackground=editor.accent, highlightcolor=editor.accent,
            cursor="fleur",
        )
        self.frame.bind("<ButtonPress-1>", self._on_press)
        self.frame.bind("<B1-Motion>", self._on_move)
        self.frame.bind("<ButtonRelease-1>", self._on_release)
        self.frame.bind("<Button-3>", lambda _event: self.editor.open_properties(self.spec))

        bar = tk.Frame(self.frame, background=editor.accent)
        bar.pack(side=tk.TOP, fill=tk.X)
        tk.Label(
            bar, text="⚙", background=editor.accent, cursor="hand2",
        ).pack(side=tk.RIGHT, padx=(0, 2))
        tk.Label(
            bar, text="✕", background=editor.accent, cursor="hand2",
        ).pack(side=tk.RIGHT, padx=(0, 4))
        children = bar.winfo_children()
        children[0].bind("<Button-1>", lambda _e: self.editor.delete(self.spec))
        children[1].bind("<Button-1>", lambda _e: self.editor.open_properties(self.spec))
        tk.Label(
            bar, text=self.spec.type, background=editor.accent,
        ).pack(side=tk.LEFT, padx=4)

        self.handle = tk.Frame(
            self.frame, width=EDIT_HANDLE_SIZE, height=EDIT_HANDLE_SIZE,
            background=editor.accent, cursor="bottom_right_corner",
        )
        self.handle.place(relx=1.0, rely=1.0, anchor=tk.SE)
        self.handle.bind("<ButtonPress-1>", self._on_handle_press)
        self.handle.bind("<B1-Motion>", self._on_handle_move)
        self.handle.bind("<ButtonRelease-1>", self._on_release)

        self._origin = (0, 0)
        self._start = (spec.col, spec.row, spec.colspan, spec.rowspan)

    # ------------------------------------------------------------- 拖动

    def _begin(self, event: tk.Event) -> None:
        self._origin = (event.x_root, event.y_root)
        self._start = (self.spec.col, self.spec.row, self.spec.colspan, self.spec.rowspan)

    def _grid_delta(self, event: tk.Event) -> tuple[int, int]:
        cell_w = max(1, self.editor.cell_width())
        row_h = max(1, self.editor.row_height())
        return (
            round((event.x_root - self._origin[0]) / cell_w),
            round((event.y_root - self._origin[1]) / row_h),
        )

    def _on_press(self, event: tk.Event) -> None:
        self._begin(event)

    def _on_move(self, event: tk.Event) -> None:
        d_col, d_row = self._grid_delta(event)
        col, row, colspan, rowspan = self._start
        candidate = TileSpec(self.spec.type, col + d_col, row + d_row, colspan, rowspan)
        self.editor.try_apply(self.spec, candidate)

    def _on_handle_press(self, event: tk.Event) -> None:
        self._begin(event)

    def _on_handle_move(self, event: tk.Event) -> None:
        d_col, d_row = self._grid_delta(event)
        col, row, colspan, rowspan = self._start
        candidate = TileSpec(
            self.spec.type, col, row, max(1, colspan + d_col), max(1, rowspan + d_row)
        )
        self.editor.try_apply(self.spec, candidate)

    def _on_release(self, _event: tk.Event) -> None:
        self.editor.commit()

    # ------------------------------------------------------------- 几何

    def place(self, x: int, y: int, width: int, height: int) -> None:
        self.frame.place(x=x, y=y, width=width, height=height)
        self.frame.lift()

    def destroy(self) -> None:
        self.frame.destroy()


class DashboardEditor:
    """编辑模式控制器。`host` 由页面实现。"""

    def __init__(self, container: tk.Misc, host) -> None:
        self.container = container
        self.host = host
        self.active = False
        self.overlays: dict[int, TileOverlay] = {}
        self.accent = "#3b6ea5"

    # ------------------------------------------------------------- 开关

    def set_active(self, active: bool) -> None:
        self.active = bool(active)
        self.rebuild()

    def rebuild(self) -> None:
        for overlay in self.overlays.values():
            overlay.destroy()
        self.overlays = {}
        if not self.active:
            return
        for spec in self.host.dashboard_specs():
            self.overlays[id(spec)] = TileOverlay(self, spec)
        self.reposition()

    def reposition(self) -> None:
        cell_w = self.host.dashboard_cell_width()
        row_h = self.host.dashboard_row_height()
        for spec in self.host.dashboard_specs():
            overlay = self.overlays.get(id(spec))
            if overlay is None:
                continue
            overlay.place(
                spec.col * cell_w, spec.row * row_h,
                spec.colspan * cell_w, spec.rowspan * row_h,
            )

    def cell_width(self) -> int:
        return self.host.dashboard_cell_width()

    def row_height(self) -> int:
        return self.host.dashboard_row_height()

    # ------------------------------------------------------------- 编辑动作

    def try_apply(self, spec: TileSpec, candidate: TileSpec) -> bool:
        """把候选几何套到 spec 上。放不下就整个拒绝，**不做部分应用**。

        部分应用（比如只接受 X 方向）会让 tile 在拖动中沿着障碍物滑走，看起来
        像是自己长了腿。拒绝的表现是"拖不动"，用户一眼就懂。
        """
        candidate = clamp_tile(candidate)
        others = [other for other in self.host.dashboard_specs() if other is not spec]
        if not can_place(others, candidate):
            return False
        if (spec.col, spec.row, spec.colspan, spec.rowspan) == (
            candidate.col, candidate.row, candidate.colspan, candidate.rowspan
        ):
            return True
        spec.col, spec.row = candidate.col, candidate.row
        spec.colspan, spec.rowspan = candidate.colspan, candidate.rowspan
        self.host.dashboard_relayout()
        self.reposition()
        return True

    def commit(self) -> None:
        self.host.dashboard_persist()

    def delete(self, spec: TileSpec) -> None:
        self.host.dashboard_delete(spec)

    def open_properties(self, spec: TileSpec) -> None:
        self.host.dashboard_open_properties(spec)


class TilePropertiesDialog(tk.Toplevel):
    """组件属性：类型、绑定通道、标题、选项。

    通道列表由**当前通道表**生成，不是硬编码：参数卡只列 `param!=-` 的通道，
    波形和数值卡只列其余通道。这样用户不可能把一张参数卡绑到 `uptime` 上。
    """

    def __init__(self, parent: tk.Misc, spec: TileSpec, *, channels, tile_classes,
                 on_apply) -> None:
        super().__init__(parent)
        self.title("组件属性")
        self.transient(parent)
        self.resizable(False, False)
        self.spec = spec
        self.channels = list(channels)
        self.tile_classes = tile_classes
        self.on_apply = on_apply

        body = ttk.Frame(self, padding=12)
        body.pack(fill=tk.BOTH, expand=True)

        ttk.Label(body, text="类型").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.type_var = tk.StringVar(value=spec.type)
        self.type_box = ttk.Combobox(
            body, textvariable=self.type_var, state="readonly", width=18,
            values=sorted(tile_classes),
        )
        self.type_box.grid(row=0, column=1, sticky=tk.EW, pady=3)
        self.type_box.bind("<<ComboboxSelected>>", lambda _e: self._refresh_channels())

        ttk.Label(body, text="标题").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.title_var = tk.StringVar(value=str(spec.options.get("title", "")))
        ttk.Entry(body, textvariable=self.title_var, width=20).grid(
            row=1, column=1, sticky=tk.EW, pady=3
        )

        ttk.Label(body, text="绑定通道").grid(row=2, column=0, sticky=tk.NW, pady=3)
        self.channel_list = tk.Listbox(body, selectmode=tk.EXTENDED, height=10,
                                       exportselection=False)
        self.channel_list.grid(row=2, column=1, sticky=tk.EW, pady=3)

        self.hint_var = tk.StringVar(value="")
        ttk.Label(body, textvariable=self.hint_var, style="Muted.TLabel",
                  wraplength=260).grid(row=3, column=0, columnspan=2, sticky=tk.W)

        buttons = ttk.Frame(body)
        buttons.grid(row=4, column=0, columnspan=2, sticky=tk.E, pady=(10, 0))
        ttk.Button(buttons, text="取消", command=self.destroy).pack(side=tk.RIGHT)
        ttk.Button(buttons, text="应用", command=self._apply).pack(side=tk.RIGHT, padx=(0, 6))

        self._refresh_channels()

    def _selectable(self) -> list:
        factory = self.tile_classes.get(self.type_var.get())
        if factory is None:
            return self.channels
        if factory.PARAM_ONLY is None:
            return self.channels
        return [c for c in self.channels if c.is_parameter == factory.PARAM_ONLY]

    def _refresh_channels(self) -> None:
        self.channel_list.delete(0, tk.END)
        options = self._selectable()
        for channel in options:
            self.channel_list.insert(tk.END, channel.name)
        for index, channel in enumerate(options):
            if channel.name in self.spec.bindings:
                self.channel_list.selection_set(index)
        factory = self.tile_classes.get(self.type_var.get())
        if factory is not None:
            self.hint_var.set(
                f"{factory.LABEL}：可绑 {factory.MIN_BINDINGS}~{factory.MAX_BINDINGS} 个通道"
            )

    def _apply(self) -> None:
        factory = self.tile_classes.get(self.type_var.get())
        if factory is None:
            return
        options = self._selectable()
        chosen = [options[index].name for index in self.channel_list.curselection()]
        if not (factory.MIN_BINDINGS <= len(chosen) <= factory.MAX_BINDINGS):
            self.hint_var.set(
                f"{factory.LABEL} 需要 {factory.MIN_BINDINGS}~{factory.MAX_BINDINGS} 个通道，"
                f"当前选了 {len(chosen)} 个。"
            )
            return
        self.spec.type = self.type_var.get()
        self.spec.bindings = chosen
        title = self.title_var.get().strip()
        if title:
            self.spec.options["title"] = title
        else:
            self.spec.options.pop("title", None)
        self.on_apply(self.spec)
        self.destroy()


__all__ = ["DashboardEditor", "EDIT_HANDLE_SIZE", "TileOverlay", "TilePropertiesDialog"]
