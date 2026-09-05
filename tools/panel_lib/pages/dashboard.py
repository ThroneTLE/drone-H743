"""“状态监视”首页：可自由编排的 tile 工作台。

取代 R-T1-3 的单窗口示波器页（作者裁决，规划文档 §2.8）。那一页被打回的直接
原因是布局：8 条曲线（含 uptime≈600 s）共用一根 Y 轴，其余全成直线；滑块被挤
在角落且没有数值输入；调参时滑块和波形不在一个视野里。

**保留复用的资产**：解码（`telem_stream.py`）、环形缓冲、`ScopeCanvas`、滑块三态
判定（含审核者在实机上修掉的两处竞态，现在收在 `dashboard/tiles.ParamEchoTracker`）。
重做的只是页面本身。

数据路径与 R-T1-3 相同，也是这一页的骨架：

    固件掩码帧 -> transport 二进制分支 -> (收线程) TelemDecoder -> TelemRing
                                                                    |
                                        (Tk 线程, 33 ms 一次) 只读快照 -> 各 tile

掩码 = **当前工作区**所有组件绑定通道的并集 + 全部参数通道。用当前工作区而不是
所有工作区：看不见的工作区没有理由占数传带宽；切换工作区会重发一次掩码。
"""

from __future__ import annotations

import time
import tkinter as tk
from collections import deque
from pathlib import Path
from tkinter import filedialog, ttk

import numpy as np

from ..dashboard.editor import DashboardEditor, TilePropertiesDialog
# R-T1-5b 的组件通过注册表接入。把副作用 import 放在页面边界，而不是
# dashboard 包的 __init__：layout.py 仍可在没有 Tk 的环境里单独使用和测试。
from ..dashboard import tiles_extra as _tiles_extra  # noqa: F401
from ..dashboard.resize import DashboardResizeCoordinator
from ..dashboard.layout import (
    CARD_COLSPAN,
    CARD_ROWSPAN,
    DASHBOARD_COLUMNS,
    DASHBOARD_ROW_HEIGHT,
    DashboardLayout,
    TileSpec,
    default_layout,
    find_free_slot,
)
from ..dashboard.tiles import (
    TILE_CLASSES,
    ParamEchoTracker,
    TileContext,
    build_tile,
)
from ..proto import PROTO_REQ_PARAM_SET, parse_kv
from ..record_service import RecordSchema, TelemetryRecorder
from ..scope import SCOPE_RENDER_PERIOD_MS
from ..telem_stream import TelemDecoder, TelemRing, TelemSchema


try:  # 三段回退与面板其它页一致：包内 / tools 包 / 直接跑脚本
    from ...project_paths import TELEMETRY_DIR, dated_directory, ensure_directory
except ImportError:  # pragma: no cover - 取决于调用方的 sys.path
    try:
        from tools.project_paths import TELEMETRY_DIR, dated_directory, ensure_directory
    except ImportError:
        from project_paths import TELEMETRY_DIR, dated_directory, ensure_directory


DASHBOARD_TAB_TEXT = "状态监视"
DASHBOARD_RING_CAPACITY = 2400
DASHBOARD_TILE_GAP = 6
# 没有真实窗口宽度时（测试、刚构造还没 map）用它算格子宽，保证几何可预期。
DASHBOARD_NOMINAL_WIDTH = 1200
DASHBOARD_STATE_KEY = "dashboard"


class _ScrollHost(ttk.Frame):
    """一个只做纵向滚动的最小视口。

    不复用 `drone_tcp_panel.VerticalScrolledFrame`：那个类在面板主文件里，
    从 panel_lib 反向 import 会绕成循环依赖。这里只要 30 行。
    """

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, style="Page.TFrame")
        self.canvas = tk.Canvas(self, highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.content = ttk.Frame(self.canvas, style="Page.TFrame")
        self._window = self.canvas.create_window((0, 0), window=self.content, anchor=tk.NW)
        self.canvas.bind("<Configure>", self._sync_width)

    def _sync_width(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self._window, width=event.width)

    def set_content_height(self, height: int) -> None:
        self.content.configure(height=max(height, 1))
        self.canvas.configure(scrollregion=(0, 0, 0, max(height, 1)))

    def viewport_width(self) -> int:
        width = int(self.canvas.winfo_width())
        return width if width > 1 else DASHBOARD_NOMINAL_WIDTH


class PanelTileContext(TileContext):
    """把组件的取数请求转给页面。组件因此不认识 `DronePanel`。"""

    def __init__(self, page) -> None:
        self.page = page

    def channel(self, name: str):
        return self.page._dashboard_channel(name)

    def latest(self, name: str) -> float | None:
        return self.page._dashboard_latest(name)

    def series(self, name: str):
        return self.page._dashboard_series(name)

    def send_param(self, name: str, value: float) -> bool:
        return self.page._dashboard_send_param(name, value)

    def param_tracker(self, name: str) -> ParamEchoTracker:
        return self.page._dashboard_param_tracker(name)

    def send_command(self, text: str) -> bool:
        return self.page._dashboard_send_command(text)

    def all_channels(self) -> list:
        return self.page.dashboard_schema.ordered()

    def bindings_changed(self, _spec: TileSpec) -> None:
        self.page._dashboard_bindings_changed()


class DashboardPageMixin:
    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------

    def _init_dashboard_state(self) -> None:
        """控件引用 + 数据状态。只在挂载时调一次。"""
        self.dashboard_tab = None
        self.dashboard_host = None
        self.dashboard_stat_vars: dict[str, tk.StringVar] = {}
        self.dashboard_workspace_var = None
        self.dashboard_edit_var = None
        self.dashboard_record_var = None
        self.dashboard_hint_var = None
        self.dashboard_workspace_bar = None
        self.dashboard_context = PanelTileContext(self)
        self.dashboard_editor = None
        self.dashboard_resize = None
        self.dashboard_tiles: list = []
        # 录制的文件、列宽、指纹和代次由服务持有（R-T1-6）。页面只留一个"当前文件在
        # 哪"的引用给用户看，不再自己拿句柄，也不再自己攒队列。
        self.dashboard_recorder = TelemetryRecorder()
        self.dashboard_record_path = None
        self.dashboard_record_generation: int | None = None
        self.dashboard_rate_window: deque = deque(maxlen=64)
        self.dashboard_layout = default_layout()
        self.dashboard_param_trackers: dict[str, ParamEchoTracker] = {}
        self._dashboard_reset_session()

    def _dashboard_reset_session(self) -> None:
        """清这条链路的数据状态，**保留控件与布局**。

        布局是用户的，不该因为拔了根线就没了；schema 与解码器是链路的，换一台
        通道表不同的飞控还留着旧表继续解，就会得到一堆长度合法但含义错位的
        曲线——正是掩码帧要消灭的那类失败。
        """
        self.dashboard_tab_visible = False
        self.dashboard_stream_requested = False
        self.dashboard_schema = TelemSchema()
        self.dashboard_decoder = TelemDecoder(schema_hash=None)
        self.dashboard_ring = TelemRing(capacity=DASHBOARD_RING_CAPACITY)
        self.dashboard_schema_pending_from: int | None = None
        self.dashboard_schema_reload_requested = False
        self.dashboard_stream_status: dict[str, str] = {}
        self.dashboard_frames_seen = 0
        self.dashboard_measured_hz = 0.0
        self.dashboard_index_by_name: dict[str, int] = {}
        self.dashboard_param_trackers = {}
        self.dashboard_rate_window.clear()

    # ------------------------------------------------------------------
    # 挂载
    # ------------------------------------------------------------------

    def _dashboard_mount(self, notebook: ttk.Notebook) -> None:
        # 状态初始化收在这里而不是让面板再加一行调用：`drone_tcp_panel.py`
        # 只减不增，本期只允许改三处挂载调用的名字。
        self._init_dashboard_state()
        frame = ttk.Frame(notebook, padding=10, style="Page.TFrame")
        self.dashboard_tab = frame
        # 首位并默认选中：这是飞控的默认主页面（作者裁决）。
        notebook.insert(0, frame, text=DASHBOARD_TAB_TEXT)
        self._build_dashboard_page(frame)
        self._dashboard_load_layout()
        notebook.select(frame)
        # 关窗时把在录的会话收尾。绑在本页自己的 frame 上而不是往面板加一个退出钩子：
        # `drone_tcp_panel.py` 只减不增，本页在那里只有三处挂载调用。
        frame.bind("<Destroy>", self._dashboard_on_destroy)
        self.after(SCOPE_RENDER_PERIOD_MS, self._dashboard_render_tick)

    def _dashboard_on_destroy(self, _event=None) -> None:
        recorder = getattr(self, "dashboard_recorder", None)
        if recorder is not None:
            recorder.close()

    def _build_dashboard_page(self, parent: ttk.Frame) -> None:
        self._build_dashboard_toolbar(parent)
        self._build_dashboard_stats(parent)
        self.dashboard_host = _ScrollHost(parent)
        self.dashboard_host.pack(fill=tk.BOTH, expand=True)
        self.dashboard_editor = DashboardEditor(self.dashboard_host.content, self)
        self.dashboard_resize = DashboardResizeCoordinator(self)
        self.dashboard_host.canvas.bind("<Configure>", self._dashboard_on_resize, add="+")

    def _build_dashboard_toolbar(self, parent: ttk.Frame) -> None:
        bar = ttk.Frame(parent)
        bar.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(bar, text="TELEM  /  状态监视", style="Eyebrow.TLabel").pack(side=tk.LEFT)

        self.dashboard_workspace_bar = ttk.Frame(bar)
        self.dashboard_workspace_bar.pack(side=tk.LEFT, padx=(12, 0))
        self.dashboard_workspace_var = tk.IntVar(value=0)

        self.dashboard_record_var = tk.StringVar(value="未录制")
        ttk.Label(bar, textvariable=self.dashboard_record_var, style="Mono.TLabel").pack(
            side=tk.RIGHT
        )
        ttk.Button(bar, text="录制 CSV", command=self._dashboard_toggle_record,
                   style="Secondary.TButton").pack(side=tk.RIGHT, padx=(6, 6))
        ttk.Button(bar, text="清空缓冲", command=self._dashboard_clear_buffer,
                   style="Secondary.TButton").pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Button(bar, text="恢复预设", command=self._dashboard_restore_presets,
                   style="Secondary.TButton").pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Button(bar, text="导出布局", command=self._dashboard_export_layout,
                   style="Secondary.TButton").pack(side=tk.RIGHT, padx=(6, 0))
        ttk.Button(bar, text="导入布局", command=self._dashboard_import_layout,
                   style="Secondary.TButton").pack(side=tk.RIGHT, padx=(6, 0))
        self.dashboard_add_button = ttk.Menubutton(bar, text="添加组件",
                                                   style="Secondary.TButton")
        menu = tk.Menu(self.dashboard_add_button, tearoff=False)
        for tile_type, factory in sorted(TILE_CLASSES.items()):
            menu.add_command(
                label=factory.LABEL,
                command=lambda t=tile_type: self._dashboard_add_tile(t),
            )
        self.dashboard_add_button.configure(menu=menu)
        self.dashboard_add_button.pack(side=tk.RIGHT, padx=(6, 0))
        self.dashboard_add_menu = menu

        self.dashboard_edit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="编辑布局", variable=self.dashboard_edit_var,
                        command=self._dashboard_toggle_edit).pack(side=tk.RIGHT, padx=(6, 0))

    def _build_dashboard_stats(self, parent: ttk.Frame) -> None:
        box = ttk.Frame(parent)
        box.pack(fill=tk.X, pady=(0, 6))
        rows = (
            ("sink", "出口"),
            ("rate", "帧率"),
            ("gap", "seq 缺口"),
            ("drop", "固件丢帧"),
            ("schema", "通道表"),
            ("reject", "本机拒帧"),
        )
        for column, (key, label) in enumerate(rows):
            self.dashboard_stat_vars[key] = tk.StringVar(value="-")
            ttk.Label(box, text=label, style="Muted.TLabel").grid(
                row=0, column=column * 2, sticky=tk.W, padx=(0 if column == 0 else 12, 4)
            )
            ttk.Label(box, textvariable=self.dashboard_stat_vars[key],
                      style="Mono.TLabel").grid(row=0, column=column * 2 + 1, sticky=tk.W)
        self.dashboard_hint_var = tk.StringVar(value="等待通道表…")
        ttk.Label(box, textvariable=self.dashboard_hint_var, style="Muted.TLabel").grid(
            row=0, column=len(rows) * 2, sticky=tk.W, padx=(16, 0)
        )

    # ------------------------------------------------------------------
    # 布局
    # ------------------------------------------------------------------

    def _dashboard_load_layout(self) -> None:
        raw = (self._panel_state or {}).get(DASHBOARD_STATE_KEY)
        restored = DashboardLayout.from_json(raw) if raw is not None else None
        self.dashboard_layout = restored or default_layout()
        self.dashboard_workspace_var.set(self.dashboard_layout.active)
        self._dashboard_rebuild_workspace_bar()
        self._dashboard_rebuild_tiles()

    def _dashboard_persist(self) -> None:
        """布局随面板状态落盘。写失败不能影响正在跑的链路。"""
        self.dashboard_layout.active = int(self.dashboard_workspace_var.get())
        try:
            self._panel_state[DASHBOARD_STATE_KEY] = self.dashboard_layout.to_json()
            self._save_panel_state()
        except Exception:       # pragma: no cover - 落盘失败不该带走界面
            pass

    def _dashboard_export_layout_text(self) -> str:
        """导出可读 JSON 的纯入口，便于测试且不把文件对话框混进模型。"""
        return self.dashboard_layout.dumps()

    def _dashboard_import_layout_text(self, text: str) -> bool:
        """接收用户导出的 JSON；坏文件保持当前布局原样不动。"""
        restored = DashboardLayout.loads(text)
        if restored is None:
            self.dashboard_hint_var.set("布局 JSON 无效，未导入")
            return False
        self.dashboard_layout = restored
        self.dashboard_workspace_var.set(restored.active)
        self._dashboard_rebuild_workspace_bar()
        self._dashboard_rebuild_tiles()
        # 导入可能换了工作区和绑定，必须立即收敛到新掩码，不能让旧布局继续
        # 占数传带宽。
        self._dashboard_send_mask()
        self._dashboard_persist()
        self.dashboard_hint_var.set("布局已导入")
        return True

    def _dashboard_export_layout(self) -> None:
        """把布局写到用户明确选择的位置；不是证据数据，不落到 data/。"""
        selected = filedialog.asksaveasfilename(
            parent=self.dashboard_tab,
            title="导出状态监视布局",
            defaultextension=".json",
            filetypes=(("JSON 布局", "*.json"), ("所有文件", "*.*")),
        )
        if not selected:
            return
        try:
            Path(selected).write_text(self._dashboard_export_layout_text(), encoding="utf-8")
        except OSError as exc:
            self.dashboard_hint_var.set(f"布局导出失败：{exc}")
            return
        self.dashboard_hint_var.set(f"已导出布局：{Path(selected).name}")

    def _dashboard_import_layout(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self.dashboard_tab,
            title="导入状态监视布局",
            filetypes=(("JSON 布局", "*.json"), ("所有文件", "*.*")),
        )
        if not selected:
            return
        try:
            text = Path(selected).read_text(encoding="utf-8")
        except OSError as exc:
            self.dashboard_hint_var.set(f"布局导入失败：{exc}")
            return
        self._dashboard_import_layout_text(text)

    def _dashboard_rebuild_workspace_bar(self) -> None:
        bar = self.dashboard_workspace_bar
        if bar is None:
            return
        for child in bar.winfo_children():
            child.destroy()
        for index, workspace in enumerate(self.dashboard_layout.workspaces):
            ttk.Radiobutton(
                bar, text=workspace.name, value=index,
                variable=self.dashboard_workspace_var,
                command=self._dashboard_switch_workspace,
            ).pack(side=tk.LEFT, padx=(0, 6))

    def _dashboard_switch_workspace(self) -> None:
        self.dashboard_layout.active = int(self.dashboard_workspace_var.get())
        self._dashboard_rebuild_tiles()
        # 掩码跟着工作区走：看不见的工作区没有理由占数传带宽。
        self._dashboard_send_mask()
        self._dashboard_persist()

    def _dashboard_specs(self) -> list[TileSpec]:
        return self.dashboard_layout.active_workspace().tiles

    def _dashboard_rebuild_tiles(self) -> None:
        for tile in self.dashboard_tiles:
            tile.destroy()
        self.dashboard_tiles = []
        if self.dashboard_host is None:
            return
        for spec in self._dashboard_specs():
            tile = build_tile(self.dashboard_host.content, spec, self.dashboard_context)
            if tile is not None:
                self.dashboard_tiles.append(tile)
        self._dashboard_relayout()
        if self.dashboard_resize is not None:
            self.dashboard_resize.mark_laid_out()
        if self.dashboard_editor is not None:
            self.dashboard_editor.rebuild()

    def _dashboard_cell_width(self) -> int:
        if self.dashboard_host is None:
            return DASHBOARD_NOMINAL_WIDTH // DASHBOARD_COLUMNS
        return max(1, self.dashboard_host.viewport_width() // DASHBOARD_COLUMNS)

    def _dashboard_row_height(self) -> int:
        return DASHBOARD_ROW_HEIGHT

    def _dashboard_relayout(self) -> None:
        cell_w = self._dashboard_cell_width()
        row_h = self._dashboard_row_height()
        for tile in self.dashboard_tiles:
            spec = tile.spec
            tile.frame.place(
                x=spec.col * cell_w + DASHBOARD_TILE_GAP // 2,
                y=spec.row * row_h + DASHBOARD_TILE_GAP // 2,
                width=max(spec.colspan * cell_w - DASHBOARD_TILE_GAP, 1),
                height=max(spec.rowspan * row_h - DASHBOARD_TILE_GAP, 1),
            )
        rows = self.dashboard_layout.active_workspace().rows_used()
        if self.dashboard_host is not None:
            self.dashboard_host.set_content_height(rows * row_h + DASHBOARD_TILE_GAP)

    def _dashboard_on_resize(self, _event=None) -> None:
        if self.dashboard_resize is not None:
            self.dashboard_resize.note_configure()

    # --- 编辑器回调（DashboardEditor 的 host 协议） ---

    def dashboard_specs(self) -> list[TileSpec]:
        return self._dashboard_specs()

    def dashboard_cell_width(self) -> int:
        return self._dashboard_cell_width()

    def dashboard_row_height(self) -> int:
        return self._dashboard_row_height()

    def dashboard_relayout(self) -> None:
        self._dashboard_relayout()

    def dashboard_reposition_overlays(self) -> None:
        if self.dashboard_editor is not None and self.dashboard_editor.active:
            self.dashboard_editor.reposition()

    def dashboard_persist(self) -> None:
        self._dashboard_persist()

    def dashboard_delete(self, spec: TileSpec) -> None:
        self._dashboard_delete_tile(spec)

    def dashboard_open_properties(self, spec: TileSpec) -> None:
        self._dashboard_open_properties(spec)

    # --- 编辑动作 ---

    def _dashboard_toggle_edit(self) -> None:
        if self.dashboard_editor is not None:
            self.dashboard_editor.set_active(bool(self.dashboard_edit_var.get()))

    def _dashboard_add_tile(self, tile_type: str) -> TileSpec | None:
        factory = TILE_CLASSES.get(tile_type)
        if factory is None:
            return None
        specs = self._dashboard_specs()
        colspan, rowspan = getattr(factory, "DEFAULT_SPAN", (CARD_COLSPAN, CARD_ROWSPAN))
        col, row = find_free_slot(specs, colspan, rowspan)
        spec = TileSpec(tile_type, col, row, colspan, rowspan, [], {})
        specs.append(spec)
        self._dashboard_rebuild_tiles()
        self._dashboard_send_mask()
        self._dashboard_persist()
        return spec

    def _dashboard_delete_tile(self, spec: TileSpec) -> None:
        specs = self._dashboard_specs()
        if spec in specs:
            specs.remove(spec)
        self._dashboard_rebuild_tiles()
        self._dashboard_send_mask()
        self._dashboard_persist()

    def _dashboard_open_properties(self, spec: TileSpec) -> None:
        TilePropertiesDialog(
            self.dashboard_tab, spec,
            channels=self.dashboard_schema.ordered(),
            tile_classes=TILE_CLASSES,
            on_apply=self._dashboard_apply_properties,
        )

    def _dashboard_apply_properties(self, _spec: TileSpec) -> None:
        self._dashboard_rebuild_tiles()
        self._dashboard_send_mask()
        self._dashboard_persist()

    def _dashboard_bindings_changed(self) -> None:
        """卡片顶部下拉直接改绑后的收敛点。

        不重建整页：选择菜单正在执行时销毁它会让 Tk 丢事件。当前卡已自行 rebind，
        页面这里只做两件必须全局一致的事——更新固件掩码和持久化用户布局。
        """
        self._dashboard_send_mask()
        self._dashboard_persist()

    def _dashboard_restore_presets(self) -> None:
        self.dashboard_layout = default_layout()
        self.dashboard_workspace_var.set(0)
        self._dashboard_rebuild_workspace_bar()
        self._dashboard_rebuild_tiles()
        self._dashboard_send_mask()
        self._dashboard_persist()

    def _dashboard_clear_buffer(self) -> None:
        """只清本地环形缓冲，一帧都不发——固件那边没有"缓冲"可清。"""
        self.dashboard_ring.clear()
        self.dashboard_rate_window.clear()
        self.dashboard_frames_seen = 0
        self.dashboard_measured_hz = 0.0

    # ------------------------------------------------------------------
    # 链路
    # ------------------------------------------------------------------

    def _dashboard_poll_tick(self, now: float) -> None:
        tab = getattr(self, "dashboard_tab", None)
        visible = tab is not None and self.notebook.select() == str(tab)
        connected = self._transport_connected()
        # 换了一次连接就是另一次飞行/另一台飞控，不能续到同一个文件里。
        self.dashboard_recorder.note_generation(
            getattr(self.transport, "connection_generation", None)
        )
        self._dashboard_refresh_record_status()
        if visible != self.dashboard_tab_visible:
            self.dashboard_tab_visible = visible
            self._dashboard_sync_stream(visible)
        elif visible and connected and not self._dashboard_stream_attached():
            # 可见性没变但链路变了：先开页再连线，或者拔插后重连。固件在 USB
            # 出口下拔线会自己 stream=0，重连后不再发一次 STREAM on 就永远没
            # 波形；只盯可见性翻转看不见这两种情况（审核者实机复现的缺陷）。
            self._dashboard_sync_stream(True)
        if not (visible and connected):
            if not connected:
                self.dashboard_stream_requested = False
            return
        if not self.dashboard_schema.complete and self.dashboard_schema_pending_from is None:
            self._dashboard_request_schema()
        elif (self.dashboard_decoder.needs_schema_reload
              and not self.dashboard_schema_reload_requested):
            self.dashboard_schema_reload_requested = True
            self._dashboard_request_schema()

    def _dashboard_stream_attached(self) -> bool:
        """本页是否已在**当前** transport 上开了流并挂上二进制 sink。"""
        if not self.dashboard_stream_requested:
            return False
        transport = getattr(self, "transport", None)
        return getattr(transport, "_binary_sink", None) is not None or (
            getattr(transport, "binary_sink", None) is not None
        )

    def _dashboard_sync_stream(self, active: bool) -> None:
        if not self._transport_connected():
            self.dashboard_stream_requested = False
            return
        command = "TELEM STREAM on" if active else "TELEM STREAM off"
        if not self._validation_command_allowed(command):
            return
        self.dashboard_stream_requested = active
        if active:
            self.transport.set_binary_sink(self._dashboard_on_binary_frame)
        self.transport.send_line(command)
        if not active:
            self.transport.set_binary_sink(None)

    def _dashboard_request_schema(self) -> None:
        self.dashboard_schema = TelemSchema()
        self.dashboard_schema_pending_from = 0
        for command in ("TELEM?", "TELEM CH from=0"):
            if self._validation_command_allowed(command):
                self.transport.send_line(command)

    def _dashboard_handle_line(self, line: str) -> None:
        if not hasattr(self, "dashboard_schema"):
            return
        if line.startswith("TELEM STREAM "):
            self.dashboard_stream_status = parse_kv(line)
            return
        if not self.dashboard_schema.feed_line(line):
            return
        if not line.startswith("TELEM PAGE "):
            return
        next_page = self.dashboard_schema.next_page
        self.dashboard_schema_pending_from = next_page
        if next_page is not None and self._transport_connected():
            command = f"TELEM CH from={next_page}"
            if self._validation_command_allowed(command):
                self.transport.send_line(command)
            return
        if self.dashboard_schema.complete:
            self._dashboard_adopt_schema()

    def _dashboard_adopt_schema(self) -> None:
        """通道表齐了：绑定解码器、按名重绑所有组件、发一次掩码。"""
        self.dashboard_schema_reload_requested = False
        # 换表要先给录制收尾：旧文件的表头解释不了新宽度的行（N12）。必须在解码器
        # 换绑之前做，否则新表的样本会先一步进到旧会话里。
        self.dashboard_recorder.note_schema(
            RecordSchema.from_telem_schema(self.dashboard_schema)
        )
        # 被动结束要当场告诉用户，不能等到下一次渲染拍——这一页很可能根本不可见。
        self._dashboard_refresh_record_status()
        self.dashboard_decoder.bind_schema(self.dashboard_schema)
        self.dashboard_ring = TelemRing(
            capacity=DASHBOARD_RING_CAPACITY,
            channel_count=max(self.dashboard_schema.channel_count, 1),
        )
        self.dashboard_index_by_name = {
            channel.name: channel.index for channel in self.dashboard_schema.ordered()
        }
        for tile in self.dashboard_tiles:
            tile.rebind()
        missing = sorted({name for tile in self.dashboard_tiles for name in tile.missing})
        if self.dashboard_schema.body_frame == "body_flu":
            frame_text = (
                f"；坐标 FLU v{self.dashboard_schema.frame_contract}"
                "（X前 / Y左 / Z上）"
            )
        else:
            frame_text = "；坐标未声明（legacy schema）"
        self.dashboard_hint_var.set(
            f"通道表 {self.dashboard_schema.channel_count} 路"
            + frame_text
            + (f"；本工作区有 {len(missing)} 个绑定找不到通道：{', '.join(missing)}"
               if missing else "")
        )
        self._dashboard_send_mask()

    def _dashboard_mask(self) -> int:
        """当前工作区绑定通道并集 + 全部参数通道。

        参数通道恒在掩码里：它们平时不置位（固件按变化回显），但 1 Hz 的全量
        刷新帧要靠它们把滑块喂回来；剔出去滑块会永远停在初值。
        """
        mask = 0
        for name in self.dashboard_layout.active_workspace().bound_channels():
            index = self.dashboard_index_by_name.get(name)
            if index is not None:
                mask |= 1 << index
        for channel in self.dashboard_schema.ordered():
            if channel.is_parameter:
                mask |= 1 << channel.index
        return mask

    def _dashboard_send_mask(self) -> None:
        if not self.dashboard_schema.complete or not self._transport_connected():
            return
        mask = self._dashboard_mask()
        if mask == 0:
            return
        command = f"TELEM MASK {mask:X}"
        if self._validation_command_allowed(command):
            self.transport.send_line(command)

    # ------------------------------------------------------------------
    # 数据（收线程）
    # ------------------------------------------------------------------

    def _dashboard_on_binary_frame(self, function: int, payload: bytes) -> None:
        """transport 收线程直接调用：解码、入环、记 CSV 行。

        不经过 `rx_queue`：那条队列由 Tk 主循环按批抽干，40 Hz~1 kHz 的帧走
        那里会让波形跟着界面卡顿走样。这里只碰自己的环形缓冲和两个 deque。
        """
        del function
        samples = self.dashboard_decoder.feed(payload)
        if not samples:
            return
        self.dashboard_ring.push_many(samples)
        self.dashboard_frames_seen += 1
        self.dashboard_rate_window.append(time.monotonic())
        # 提交是非阻塞的：收线程绝不能因为磁盘慢而停下来，那会直接让波形跟着卡。
        for sample in samples:
            self.dashboard_recorder.submit(sample)

    # ------------------------------------------------------------------
    # 取数（TileContext 的实现）
    # ------------------------------------------------------------------

    def _dashboard_channel(self, name: str):
        index = self.dashboard_index_by_name.get(name)
        if index is None:
            return None
        return self.dashboard_schema.channels.get(index)

    def _dashboard_latest(self, name: str) -> float | None:
        index = self.dashboard_index_by_name.get(name)
        if index is None:
            return None
        entry = self.dashboard_ring.latest(index)
        return None if entry is None else entry[1]

    def _dashboard_series(self, name: str):
        index = self.dashboard_index_by_name.get(name)
        if index is None:
            return np.empty(0, dtype="float64"), np.empty(0, dtype="float32")
        return self.dashboard_ring.snapshot(index)

    def _dashboard_param_tracker(self, name: str) -> ParamEchoTracker:
        tracker = self.dashboard_param_trackers.get(name)
        if tracker is None:
            tracker = ParamEchoTracker(name)
            self.dashboard_param_trackers[name] = tracker
        return tracker

    def _dashboard_send_param(self, name: str, value: float) -> bool:
        channel = self._dashboard_channel(name)
        if channel is None or not channel.is_parameter:
            return False
        if not self._transport_connected():
            return False
        payload = f"PARAM SET {channel.param} {value:.6g}"
        if not self._validation_command_allowed(payload):
            return False
        return bool(self.transport.send_frame(PROTO_REQ_PARAM_SET, payload.encode("utf-8")))

    def _dashboard_send_command(self, text: str) -> bool:
        """按钮组件的出口（R-T1-5b）。**必须**过安全门，不得绕过。"""
        command = text.strip()
        if not command or not self._transport_connected():
            return False
        if not self._validation_command_allowed(command):
            return False
        return bool(self.transport.send_line(command))

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------

    def _dashboard_render_tick(self) -> None:
        try:
            if self.dashboard_tab_visible:
                if self.dashboard_resize is None or not self.dashboard_resize.render_suspended:
                    for tile in self.dashboard_tiles:
                        tile.refresh()
                self._dashboard_refresh_stats()
            # 只读一个状态快照，不碰磁盘、不等队列：写盘在服务自己的线程里。
            self._dashboard_refresh_record_status()
        finally:
            self.after(SCOPE_RENDER_PERIOD_MS, self._dashboard_render_tick)

    def _dashboard_refresh_stats(self) -> None:
        if not self.dashboard_stat_vars:
            return
        stamps = list(self.dashboard_rate_window)
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            self.dashboard_measured_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        status = self.dashboard_stream_status
        stats = self.dashboard_decoder.stats
        self.dashboard_stat_vars["sink"].set(status.get("active", "-"))
        self.dashboard_stat_vars["rate"].set(f"{self.dashboard_measured_hz:.1f} Hz")
        self.dashboard_stat_vars["gap"].set(
            f"{stats.seq_gaps} 次 / 丢 {stats.lost_frames} 帧"
        )
        self.dashboard_stat_vars["drop"].set(status.get("drop", "-"))
        self.dashboard_stat_vars["schema"].set(
            f"{self.dashboard_schema.computed_hash():08X}"
            if self.dashboard_schema.complete else "-"
        )
        unclaimed = getattr(self.transport, "binary_unclaimed", 0)
        self.dashboard_stat_vars["reject"].set(
            f"{stats.rejected_total} 帧 / 未接手 {unclaimed}"
        )

    # ------------------------------------------------------------------
    # CSV 录制
    # ------------------------------------------------------------------

    def _dashboard_record_directory(self) -> Path:
        """录制落在项目 data 根下的按日目录。**路径归页面，文件完整性归服务。**"""
        return ensure_directory(dated_directory(TELEMETRY_DIR))

    def _dashboard_toggle_record(self) -> None:
        if self.dashboard_recorder.status().active:
            self._dashboard_stop_record()
            return
        if not self.dashboard_schema.complete:
            self.dashboard_record_var.set("还没取到通道表")
            return
        generation = getattr(self.transport, "connection_generation", None)
        try:
            directory = self._dashboard_record_directory()
        except OSError as exc:
            # 连按日目录都建不出来：说清楚，不要留一个假的"录制中"。
            self.dashboard_record_var.set(f"无法开始录制：{exc}")
            return
        status = self.dashboard_recorder.start(
            directory,
            RecordSchema.from_telem_schema(self.dashboard_schema),
            generation=generation,
        )
        self.dashboard_record_path = status.path
        self.dashboard_record_generation = generation
        self._dashboard_refresh_record_status()

    def _dashboard_stop_record(self) -> None:
        status = self.dashboard_recorder.stop()
        self.dashboard_record_path = status.path
        self._dashboard_refresh_record_status()

    def _dashboard_flush_record(self) -> None:
        """等后台写线程把当前队列写完。

        渲染回调**不**调它——那正是 N10 的形态。它留给"停止录制"和测试做一个有限期
        的确定性屏障；超时就返回，界面不会为了写盘停住。
        """
        self.dashboard_recorder.drain()

    def _dashboard_refresh_record_status(self) -> None:
        """把服务状态映射成一行字。失败、被动结束、正常保存必须能分辨。"""
        if self.dashboard_record_var is None:
            return
        text = self.dashboard_recorder.status().describe()
        if self.dashboard_record_var.get() != text:
            self.dashboard_record_var.set(text)


__all__ = [
    "DASHBOARD_NOMINAL_WIDTH",
    "DASHBOARD_RING_CAPACITY",
    "DASHBOARD_STATE_KEY",
    "DASHBOARD_TAB_TEXT",
    "DASHBOARD_TILE_GAP",
    "DashboardPageMixin",
    "PanelTileContext",
]
