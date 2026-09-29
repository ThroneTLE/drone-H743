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
from ..dashboard.workspace_bar import (
    render_view_row,
    render_workspace_bar,
    workspace_tree,
)
from ..proto import PROTO_REQ_PARAM_SET, parse_kv
from ..record_service import LinkIdentity, RecordSchema, TelemetryRecorder
from ..scope import SCOPE_RENDER_PERIOD_MS
from ..telem_stream import TelemDecoder, TelemRing, TelemSchema
from ..telem_subscription import (
    TELEM_OWNER_DASHBOARD,
    TelemBinaryFanout,
    TelemSubscriptionRegistry,
)
from ..viewport import VerticalScrolledFrame


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
# 通道表握手停滞多久算卡住。取 2 s：数传 57600 上一页（6 条 CH + 1 条 PAGE，
# 约 600 B）在流占满带宽时最坏也就几百毫秒，2 s 没有任何一行进来就不是慢，
# 是掉了。太短会在正常的慢链路上重复请求，白占本来就紧张的上行。
DASHBOARD_SCHEMA_STALL_S = 2.0

# 流"应该开着却一帧都不来"多久算不对劲。40 Hz 下 3 s = 120 帧；最慢的默认档
# （数传 40 Hz）也远比这密，所以 3 s 静默不是抖动，是固件那头真的不在发了。
TELEM_STREAM_SILENCE_S = 3.0
# 两次强制重开之间的最小间隔。链路被日志导出独占、或处在 V0 只读会话时，重开
# 尝试一定失败；限流是为了不让它按链路速度空转。
TELEM_STREAM_RECOVER_S = 5.0


_ScrollHost = VerticalScrolledFrame


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
        self.dashboard_workspace_root_var = None
        self.dashboard_edit_var = None
        self.dashboard_record_var = None
        self.dashboard_hint_var = None
        self.dashboard_workspace_bar = None
        self.dashboard_workspace_view_bar = None
        self.dashboard_actions_bar = None
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
        # 链路仲裁器是**面板级**的，不属于工作台：电源页也要订阅通道、也要求
        # 开流。它活得比任何一次链路会话都久，所以不进 `_dashboard_reset_session`。
        self.telem_registry = TelemSubscriptionRegistry()
        self.telem_fanout = TelemBinaryFanout()
        self.telem_fanout.attach(TELEM_OWNER_DASHBOARD, self._dashboard_on_binary_frame)
        self._dashboard_reset_session()

    def _dashboard_reset_session(self) -> None:
        """清这条链路的数据状态，**保留控件与布局**。

        布局是用户的，不该因为拔了根线就没了；schema 与解码器是链路的，换一台
        通道表不同的飞控还留着旧表继续解，就会得到一堆长度合法但含义错位的
        曲线——正是掩码帧要消灭的那类失败。
        """
        self.dashboard_tab_visible = False
        self.dashboard_stream_requested = False
        #: 流是在哪一次连接（transport, 代次）上开起来的。None = 没开。
        self.telem_stream_link = None
        #: 已**成功**下发的 (掩码, 链路身份)。None = 这条链路上还没发成功过。
        self.telem_mask_sent = None
        #: 最近一帧遥测到达的时刻（收线程写）。用来发现"固件自己复位了"。
        self.telem_last_frame_at = None
        #: 最近一帧里最新样本的固件时间戳（µs）。判"某一路是不是掉出掩码了"。
        self.telem_last_frame_t_us = None
        #: 上一次因为收不到帧而强制重开流的时刻，限流用。
        self.telem_stream_recovered_at = 0.0
        self.dashboard_schema = TelemSchema()
        self.dashboard_decoder = TelemDecoder(schema_hash=None)
        self.dashboard_ring = TelemRing(capacity=DASHBOARD_RING_CAPACITY)
        self.dashboard_schema_pending_from: int | None = None
        self.dashboard_schema_reload_requested = False
        # 握手进度计数（收线程只做 +1，不碰时钟）与看门狗的观察点
        # (上次看到的计数, 观察时刻)。None = 这条链路上还没请求过。
        self.dashboard_schema_progress = 0
        self.dashboard_schema_watch: tuple[int, float] | None = None
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
        bar.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(bar, text="TELEM  /  状态监视", style="Eyebrow.TLabel").pack(side=tk.LEFT)

        self.dashboard_workspace_bar = ttk.Frame(bar)
        self.dashboard_workspace_bar.pack(side=tk.LEFT, padx=(12, 0))
        # 一级工作区用自己的变量：子视图被选中时，第一行仍要停在它所属的那个
        # 一级工作区上，而 dashboard_workspace_var 这时已经指向子视图了。
        self.dashboard_workspace_var = tk.IntVar(value=0)
        self.dashboard_workspace_root_var = tk.IntVar(value=0)

        self.dashboard_record_var = tk.StringVar(value="未录制")
        ttk.Label(bar, textvariable=self.dashboard_record_var, style="Mono.TLabel").pack(
            side=tk.RIGHT
        )

        # 子视图行只在有内容时才占版面。
        #
        # 先写成"常驻但通常是空"，因为空 frame 只有 1 px、省掉了重新显示时的插入
        # 位置计算。实测这是错的：`test_total_redraw_benchmark` 由 5/5 掉到 4/5。
        # 下面 `columns = 4` 那段注释早就写着"排成三行会多占 39 px，实测把 dashboard
        # 总重绘从 5/5 过压到 3/5 挂"——这一页的重绘预算本来就贴着线，多一行常驻
        # 就是多一行要参与布局计算，哪怕它是空的。
        self.dashboard_workspace_view_bar = ttk.Frame(parent)

        actions = ttk.Frame(parent)
        actions.pack(fill=tk.X, pady=(0, 6))
        # 记着它，第二行要插在工具条和按钮行之间（pack 默认追加到末尾）。
        self.dashboard_actions_bar = actions
        action_specs = (
            ("录制 CSV", self._dashboard_toggle_record),
            ("清空缓冲", self._dashboard_clear_buffer),
            ("恢复预设", self._dashboard_restore_presets),
            ("导出布局", self._dashboard_export_layout),
            ("导入布局", self._dashboard_import_layout),
        )
        # 每个控件按同一个计数器落格，谁都不再手写行列号——"编辑布局"曾被硬编码
        # 成 row=1/column=0，正好压在"导出布局"上，两个控件叠在一格里。
        # 四列而不是三列：七个控件正好两行排满，工具条高度和出缺陷那版一样。
        # 排成三行会多占 39 px，实测把 dashboard 总重绘从 5/5 过压到 3/5 挂。
        columns = 4
        cells = iter(
            {"row": index // columns, "column": index % columns, "padx": (0, 6),
             "pady": 2, "sticky": tk.W}
            for index in range(len(action_specs) + 2)
        )
        for label, command in action_specs:
            ttk.Button(actions, text=label, command=command,
                       style="Secondary.TButton").grid(**next(cells))
        self.dashboard_add_button = ttk.Menubutton(
            actions, text="添加组件", style="Secondary.TButton"
        )
        menu = tk.Menu(self.dashboard_add_button, tearoff=False)
        for tile_type, factory in sorted(TILE_CLASSES.items()):
            menu.add_command(
                label=factory.LABEL,
                command=lambda t=tile_type: self._dashboard_add_tile(t),
            )
        self.dashboard_add_button.configure(menu=menu)
        self.dashboard_add_button.grid(**next(cells))
        self.dashboard_add_menu = menu

        self.dashboard_edit_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(actions, text="编辑布局", variable=self.dashboard_edit_var,
                        command=self._dashboard_toggle_edit).grid(**next(cells))
        for column in range(columns):
            actions.columnconfigure(column, weight=1)

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
        view_bar = self.dashboard_workspace_view_bar
        if bar is None or view_bar is None:
            return
        self._dashboard_show_view_row(render_workspace_bar(
            bar, view_bar,
            workspace_tree(self.dashboard_layout.workspaces,
                           int(self.dashboard_workspace_var.get())),
            root_var=self.dashboard_workspace_root_var,
            view_var=self.dashboard_workspace_var,
            on_root=self._dashboard_select_root,
            on_view=self._dashboard_switch_workspace,
        ))

    def _dashboard_show_view_row(self, visible: bool) -> None:
        """第二行没内容时整行让出版面，不是留个 1 px 的空 frame。

        这一页的重绘预算贴着线（见 `_build_dashboard_toolbar` 里的行高注释），
        常驻一个空行就足以把 `test_total_redraw_benchmark` 从 5/5 压到 4/5。
        """
        row = self.dashboard_workspace_view_bar
        if row is None:
            return
        if visible:
            if not row.winfo_manager():
                row.pack(fill=tk.X, before=self.dashboard_actions_bar)
        elif row.winfo_manager():
            row.pack_forget()

    def _dashboard_select_root(self, index: int) -> None:
        """点第一行：回到这个一级工作区自己的视图，并**只**重画第二行。

        换了父，第二行的内容就全变了（或者整行该消失）；第一行不重画，因为这次
        调用正来自第一行的某个单选钮。
        """
        self._dashboard_switch_workspace(index)
        self._dashboard_show_view_row(render_view_row(
            self.dashboard_workspace_view_bar,
            workspace_tree(self.dashboard_layout.workspaces, index),
            view_var=self.dashboard_workspace_var,
            on_view=self._dashboard_switch_workspace,
        ))

    def _dashboard_switch_workspace(self, index: int | None = None) -> None:
        if index is not None:
            self.dashboard_workspace_var.set(index)
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
            # Geometry probes record ``after`` callbacks; do the first changed
            # grid-cell placement synchronously while still coalescing pixel noise.
            cell_width = self._dashboard_cell_width()
            if self.dashboard_resize._last_cell_width != cell_width:
                self._dashboard_relayout()
                self.dashboard_resize.mark_laid_out()
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

    def _dashboard_visible(self) -> bool:
        tab = getattr(self, "dashboard_tab", None)
        if tab is None:
            return False
        try:
            return self.notebook.select() == str(tab)
        except tk.TclError:                     # pragma: no cover - 窗口正在销毁
            return False

    def _dashboard_poll_tick(self, now: float) -> None:
        visible = self._dashboard_visible()
        connected = self._transport_connected()
        # 换了一次连接就是另一次飞行/另一台飞控，不能续到同一个文件里。
        self.dashboard_recorder.note_generation(
            getattr(self.transport, "connection_generation", None)
        )
        self._dashboard_refresh_record_status()
        if visible != self.dashboard_tab_visible:
            self.dashboard_tab_visible = visible
            self._telem_register_dashboard()
        # 每拍都收敛一次。`_telem_apply()` 与 `_dashboard_send_mask()` 都是幂等的，
        # 状态没变就一个字节都不发；可见性没变但链路变了也能自己接上：先开页再
        # 连线、或者拔插后重连时，固件在 USB 出口下会自己 stream=0，不再发一次
        # STREAM on 就永远没波形（审核者实机复现的缺陷）。
        self._telem_stream_watchdog(now)
        self._telem_apply()
        if not connected:
            return
        # 通道表不只服务工作台：电源页可见而工作台不可见时也得有表，否则按名
        # 算掩码永远算不出 batt_v 的位，订阅了也收不到。
        if not (visible or self.telem_registry.demand().stream):
            return
        self._dashboard_drive_schema(now)
        # 掩码的收敛回路。一次性边沿触发挡不住被拒的那一次：命令被吞之后并集
        # 不会再变，于是永远不会重发，而页面上看起来只是"那两路通道没数据"。
        self._dashboard_send_mask()

    def _dashboard_drive_schema(self, now: float) -> None:
        """通道表握手的推进器。

        握手是分页的：每收到一行 `TELEM PAGE` 才请求下一页。原来没有任何超时，
        于是**丢一行 PAGE 就永久卡住**——`pending_from` 停在那一页，界面停在
        "等待通道表"，再也不会自己恢复。

        掉行在数传出口上是常态而不是意外：文本回包和 40 Hz 二进制流挤同一条
        深度 32、满时丢最旧的 `uartTxQueue`。USB 出口看不见这个问题，因为文本
        是直写 CDC 的，根本不过那条队列。所以这里必须能自愈。
        """
        if (self.dashboard_decoder.needs_schema_reload
                and not self.dashboard_schema_reload_requested):
            self.dashboard_schema_reload_requested = True
            self._dashboard_request_schema(now)
            return
        if self.dashboard_schema.complete:
            return
        watch = self.dashboard_schema_watch
        if watch is None:
            self._dashboard_request_schema(now)
            return
        seen, since = watch
        if self.dashboard_schema_progress != seen:
            # 还在往里进货，不算停滞，重新计时。
            self.dashboard_schema_watch = (self.dashboard_schema_progress, now)
            return
        if (now - since) < DASHBOARD_SCHEMA_STALL_S:
            return
        resume = self.dashboard_schema.first_missing
        if resume is None:
            # 连表头都没到：整轮重来。
            self._dashboard_request_schema(now)
        else:
            self._dashboard_resume_schema(now, resume)

    def _telem_link_identity(self):
        transport = getattr(self, "transport", None)
        return (transport, getattr(transport, "connection_generation", None))

    def _dashboard_stream_attached(self) -> bool:
        """流是否已在**当前这一次连接**上开着并挂上了二进制 sink。

        身份要连代次一起比，不能只比 transport 对象：串口重连是在同一个对象上
        把代次加一，只比对象的话，重连之后这边会以为流还开着——而固件那头已经
        `stream=0` 了，于是永远没有波形，也永远不会重挂 sink。
        """
        if not self.dashboard_stream_requested:
            return False
        transport = getattr(self, "transport", None)
        attached = getattr(transport, "_binary_sink", None) is not None or (
            getattr(transport, "binary_sink", None) is not None
        )
        return attached and self.telem_stream_link == self._telem_link_identity()

    # ------------------------------------------------------------------
    # 链路仲裁（掩码 / 流开关 / sink 三者都只有一份，见 telem_subscription.py）
    # ------------------------------------------------------------------

    def _telem_register_dashboard(self) -> bool:
        """把工作台自己那份需求登记进仲裁器，按**带宽行为**分成两类。

        * 实时通道（当前工作区的绑定通道）随可见性进出掩码：工作台不可见时，
          它摆的那些波形通道没有理由继续占数传带宽——用户此刻在看的是别的页。
        * 参数通道恒在：固件按脏位回显，稳态帧里不置位、几乎不占带宽，但每秒
          一次的全量刷新帧要靠它们把滑块喂回来；撤掉滑块会永远停在初值。
        """
        # 现查而不是读 `dashboard_tab_visible`：那个标志由轮询节拍更新，比页签切换
        # 慢一拍。掩码在慢的那一拍里会把工作台的波形通道整批甩掉再加回来，白白
        # 多发两条 `TELEM MASK`。
        visible = self._dashboard_visible()
        return self.telem_registry.register(
            TELEM_OWNER_DASHBOARD,
            self.dashboard_layout.active_workspace().bound_channels(),
            params=[channel.name for channel in self.dashboard_schema.ordered()
                    if channel.is_parameter],
            stream=visible,
            visible=visible,
        )

    def _telem_subscribe(self, owner: str, channels=(), *, params=(),
                         stream: bool = True, consumer=None) -> bool:
        """别的页面登记订阅。返回本次是否改变了仲裁结果。"""
        changed = self.telem_registry.register(
            owner, channels, params=params, stream=stream)
        if consumer is not None:
            changed = self.telem_fanout.attach(owner, consumer) or changed
        if changed:
            self._telem_apply()
            # 掩码走工作台原来那条发送路径，不另起一套：并集变了就得重发，
            # 而 `TELEM MASK` 是整条覆盖的，漏发一次订阅者就永远收不到自己的通道。
            self._dashboard_send_mask()
        return changed

    def _telem_unsubscribe(self, owner: str) -> bool:
        changed = self.telem_registry.unregister(owner)
        changed = self.telem_fanout.detach(owner) or changed
        if changed:
            self._telem_apply()
            self._dashboard_send_mask()
        return changed

    def _telem_suppress(self, owner: str, reason: str = "") -> bool:
        """独占抑制：链路归 `owner`，谁在订阅都不算数。"""
        changed = self.telem_registry.suppress(owner, reason)
        if changed:
            self._telem_apply()
        return changed

    def _telem_release_exclusive(self, owner: str) -> bool:
        changed = self.telem_registry.release(owner)
        if changed:
            self._telem_apply()
        return changed

    def _telem_stream_watchdog(self, now: float) -> None:
        """流"应该开着却一帧都不来"——多半是飞控自己复位了。

        数传 / 蓝牙口上，飞控按复位键、看门狗咬、掉电重插电池时，**地面端的串口
        从头到尾没有掉线**：`serial_session` 的 `is_connected` 只看端口对象在不在，
        连接代次只在开关口时才加。于是主机这边 `dashboard_stream_requested` 还是
        True、sink 还挂着、代次也相等，`_telem_apply()` 一看"状态没变"就一个字节
        都不发。而固件那边 `APP_TelemStream_Init()` 已经把 `vofaStreamActive` 清零、
        掩码还原成默认（不含 `batt_v` / `batt_i`）。

        合并之前这条是被两个页面的 2 秒轮询顺手兜住的：`STATUS?` 与
        `BATTERY? <nonce>` 跟固件的流状态无关，复位后照样有回包。轮询删掉了，
        这条自愈路径就得显式写出来。

        处置是把链路身份作废，让下一拍的 `_telem_apply()` 重新走一遍"开流 + 重发
        掩码"。限流到 `TELEM_STREAM_RECOVER_S`，免得链路被独占或处于只读会话时
        空转。
        """
        if not self.dashboard_stream_requested or not self._dashboard_stream_attached():
            return
        if not self.telem_registry.demand().stream:
            return
        last = self.telem_last_frame_at
        if last is None or (now - last) < TELEM_STREAM_SILENCE_S:
            return
        if (now - self.telem_stream_recovered_at) < TELEM_STREAM_RECOVER_S:
            return
        self.telem_stream_recovered_at = now
        # 掩码也要重记账：固件复位后它已经回到默认值，不重发就永远少那两路。
        self.telem_stream_link = None
        self.telem_mask_sent = None

    def _telem_apply(self) -> None:
        """把仲裁结论落到链路上。幂等：状态没变就一个字节都不发。"""
        if not self._transport_connected():
            # 线断了就不可能"还开着"。留着 True 会让重连后那一拍以为不用重开。
            self.dashboard_stream_requested = False
            return
        want = bool(self.telem_registry.demand().stream)
        if want == self._dashboard_stream_attached():
            return
        command = "TELEM STREAM on" if want else "TELEM STREAM off"
        if not self._validation_command_allowed(command):
            return
        transport = self.transport
        if want:
            transport.set_binary_sink(self._dashboard_binary_sink(transport))
            self.dashboard_stream_requested = True
            self.telem_stream_link = self._telem_link_identity()
            if not transport.send_line(command):
                # 发不出去就别声称流开着——日志导出占着串口时 `send_line` 会
                # 直接被拒。留着 True 的话下一拍看见"已挂载"就不会再试了。
                transport.set_binary_sink(None)
                self.dashboard_stream_requested = False
                self.telem_stream_link = None
                return
            # 刚开起来的流必须带着正确的掩码。固件复位后掩码回到默认（不含
            # `batt_v` / `batt_i`），只发 `STREAM on` 会得到一条"有帧但没有我要的
            # 通道"的链路——最难判的一种故障。
            #
            # 看门狗的计时窗口从**开流这一刻**起算，不是"还没收到过帧就等于超时"：
            # 后者会让开流之后的第一拍立刻判定静默，于是每拍重开一次。
            self.telem_last_frame_at = time.monotonic()
            self._dashboard_send_mask()
            return
        # 关流：先摘 sink 再发命令。命令发不出去（链路被独占 / 只读会话）也必须
        # 认账——这边已经不再消费帧了，状态得说实话。
        self.dashboard_stream_requested = False
        self.telem_stream_link = None
        transport.set_binary_sink(None)
        transport.send_line(command)

    def _dashboard_binary_sink(self, transport):
        """挂进 transport 唯一那个 sink 槽的闭包。

        transport 钉死在闭包里：回调签名不带 transport，消费者只读 `self.transport`
        会把重连前后的帧记到同一条链路上——那正是新连接第一帧混进旧录制文件的
        路径（审核 R2）。分发器把这条身份原样传给每一个消费者。
        """
        return self.telem_fanout.bind(transport)

    def _dashboard_request_schema(self, now: float | None = None) -> None:
        """整轮重拉：丢掉半张表，从表头开始。"""
        self.dashboard_schema = TelemSchema()
        self.dashboard_schema_pending_from = 0
        self.dashboard_schema_watch = (
            self.dashboard_schema_progress,
            now if now is not None else time.monotonic(),
        )
        for command in ("TELEM?", "TELEM CH from=0"):
            if self._validation_command_allowed(command):
                self.transport.send_line(command)

    def _dashboard_resume_schema(self, now: float, from_index: int) -> None:
        """从第一个缺口续拉，**保留已收到的通道**。

        重发 `TELEM?` 会把已经拼好的部分推倒重来（表头到达即清空残表）。近百行
        的握手在掉行的链路上因此永远收敛不了——每一轮都被下一次掉行打回原点。
        """
        self.dashboard_schema_pending_from = from_index
        self.dashboard_schema_watch = (self.dashboard_schema_progress, now)
        command = f"TELEM CH from={from_index}"
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
        # 有回包就算有进展。收线程只加计数、不读时钟：看门狗的计时归 Tk 线程，
        # 两个线程各自只碰自己那一半，不需要锁。
        self.dashboard_schema_progress += 1
        if not line.startswith("TELEM PAGE "):
            return
        if self.dashboard_schema.complete:
            self._dashboard_adopt_schema()
            return
        requested = self.dashboard_schema_pending_from
        target = self._dashboard_next_page()
        self.dashboard_schema_pending_from = target
        if target is None or not self._transport_connected():
            # 翻到表尾了还不齐：剩下的洞交给停滞看门狗续拉。
            return
        if target == requested:
            # 刚请求的这一页连它自己的首条都没送到。立刻重发就成了按链路速度
            # 空转的热循环，交给看门狗按 DASHBOARD_SCHEMA_STALL_S 的节奏重试。
            return
        command = f"TELEM CH from={target}"
        if self._validation_command_allowed(command):
            self.transport.send_line(command)

    def _dashboard_next_page(self) -> int | None:
        """翻页目标：固件给的 `next=`，但已经齐了的前缀直接跳过去。

        单取 `next=` 会把已补好的页再拉一遍；单取"第一个缺口"则会在某条通道
        始终收不到时卡在原地、再也走不到表尾。取两者中靠后的那个，前进和补洞
        就都不会互相挡路。
        """
        nxt = self.dashboard_schema.next_page
        hole = self.dashboard_schema.first_missing
        if nxt is None or hole is None:
            return None
        return max(nxt, hole)

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
        """全部订阅者通道名的并集，按当前通道表映射成掩码。

        工作台自己那份仍是"当前工作区绑定通道 + 全部参数通道"。参数通道恒在掩码
        里：它们平时不置位（固件按变化回显），但 1 Hz 的全量刷新帧要靠它们把滑块
        喂回来；剔出去滑块会永远停在初值。

        并集由 `telem_registry` 算：`TELEM MASK` 是整条覆盖的，两个页面各发各的
        就会互相把对方的位擦掉。
        """
        self._telem_register_dashboard()
        return self.telem_registry.mask(self.dashboard_index_by_name)

    def _dashboard_send_mask(self) -> None:
        """把并集掩码下发，并且**只在真的发出去之后**才记账。

        掩码原来是一次性边沿触发的：并集变了就发一次，发不出去也照样当成发过了。
        而 `send_line()` 真的会返回 False —— 日志导出持有串口租约时，
        `serial_session._enqueue()` 对非租约、非停止命令一律拒绝；V0 只读会话则
        在上面那道门就被拦。此时的表现极具误导性：别的通道还在喂帧，页面看起来
        "有数据"，唯独自己订阅的那两路永远不在任何一帧里，于是被诊断成 ADC 故障。

        所以这里改成幂等收敛：已成功下发的 `(掩码, 链路身份)` 记账，相同就不发，
        不同就发；由 `_dashboard_poll_tick` 每拍调一次，被拒的下一拍自动重试，
        重连（代次变）也会自动重发。
        """
        if not self.dashboard_schema.complete or not self._transport_connected():
            return
        mask = self._dashboard_mask()
        if mask == 0:
            return
        identity = self._telem_link_identity()
        if self.telem_mask_sent == (mask, identity):
            return
        command = f"TELEM MASK {mask:X}"
        if not self._validation_command_allowed(command):
            return
        if self.transport.send_line(command):
            self.telem_mask_sent = (mask, identity)

    # ------------------------------------------------------------------
    # 数据（收线程）
    # ------------------------------------------------------------------

    def _dashboard_on_binary_frame(self, function: int, payload: bytes,
                                   *, transport=None, generation=None) -> None:
        """transport 收线程直接调用：解码、入环、记 CSV 行。

        不经过 `rx_queue`：那条队列由 Tk 主循环按批抽干，40 Hz~1 kHz 的帧走
        那里会让波形跟着界面卡顿走样。这里只碰自己的环形缓冲和录制服务的队列。

        `transport` / `generation` 由 `_dashboard_binary_sink()` 钉在 sink 上，是
        这一帧的**来源身份**；录制服务据此判断这一帧属不属于当前录制的那条链路。
        直接调用（仿真、离线回放）可以不给，此时按来源当前的代次算。
        """
        del function
        samples = self.dashboard_decoder.feed(payload)
        if not samples:
            return
        source = self.transport if transport is None else transport
        if generation is None:
            generation = getattr(source, "connection_generation", None)
        self.dashboard_ring.push_many(samples)
        self.dashboard_frames_seen += 1
        arrived = time.monotonic()
        # 收线程只写这两个标量，看门狗的计时归 Tk 线程；两边各碰自己那一半。
        self.telem_last_frame_at = arrived
        # 本帧最新样本的**固件**时间戳。消费者据此判某一路通道是不是已经不在
        # 掩码里了——用主机时钟判不出来，因为别的通道还在喂帧。
        self.telem_last_frame_t_us = samples[-1].t_us
        self.dashboard_rate_window.append(arrived)
        # 提交是非阻塞的：收线程绝不能因为磁盘慢而停下来，那会直接让波形跟着卡。
        for sample in samples:
            self.dashboard_recorder.submit(
                sample, transport=source, generation=generation
            )

    # ------------------------------------------------------------------
    # 取数（TileContext 的实现）
    # ------------------------------------------------------------------

    def _dashboard_channel(self, name: str):
        index = self.dashboard_index_by_name.get(name)
        if index is None:
            return None
        return self.dashboard_schema.channels.get(index)

    def _dashboard_latest(self, name: str) -> float | None:
        """组件侧的"现在是多少"。**非有限值一律当没有数据**。

        固件在某个量无效或过期时发 `NaN`，不发 0（`batt_v` / `batt_i` 就是这么
        约定的）。把 NaN 当成一个数往下传，数值卡会显示 `+nan`、波形会在自动量程
        里把整张画布拽废、阈值着色会静默判成"正常"。组件已经全都处理 `None`，
        所以在这一处收口最省事，也最难漏。

        需要区分"没有样本"和"收到 NaN"的页面（电源页要按 adc_status 给出原因）
        走 `_telem_latest_raw()`，不走这里。
        """
        entry = self._telem_latest_raw(name)
        if entry is None:
            return None
        value = entry[1]
        return None if value != value or value in (float("inf"), float("-inf")) else value

    def _telem_latest_raw(self, name: str) -> tuple[float, float] | None:
        """环形缓冲里该通道的最新 `(t_seconds, value)`，**不过滤 NaN**。"""
        index = self.dashboard_index_by_name.get(name)
        if index is None:
            return None
        return self.dashboard_ring.latest(index)

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

    def _dashboard_note_param_error(self, param: str, reason: str) -> bool:
        """把固件的 `ERR param target <param>` 接到对应的滑块上。

        Dashboard 以前**根本不看** OK/ERR 回复，被拒绝的写入只能等 0.75 s 回显
        超时才变红，而且措辞是"未收到飞控回显"——飞控明明回了。这里按固件参数名
        反查通道，立刻判红并写出真正的原因。

        入参是**固件参数名**（`coax.att_yaw_kp`），不是通道名，所以要反查：
        通道表里 `channel.param` 才是发给飞控的那个名字。
        """
        target = (param or "").strip()
        if not target:
            return False
        schema = getattr(self, "dashboard_schema", None)
        if schema is None:
            return False
        for channel in schema.channels.values():
            if getattr(channel, "param", "") != target:
                continue
            tracker = self.dashboard_param_trackers.get(channel.name)
            if tracker is None:
                # 没人动过这个滑块就没有 tracker；此时无须凭空建一个。
                return False
            tracker.note_rejected(reason)
            for tile in self.dashboard_tiles:
                tile.refresh()
            return True
        return False

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
        """按钮回调。**全程不碰磁盘、不等待**——文件由服务的写线程建。"""
        if self.dashboard_recorder.status().busy:
            self._dashboard_stop_record()
            return
        if not self.dashboard_schema.complete:
            self.dashboard_record_var.set("还没取到通道表")
            return
        transport = self.transport
        generation = getattr(transport, "connection_generation", None)
        try:
            directory = self._dashboard_record_directory()
        except OSError as exc:
            # 建按日目录是唯一还留在这里的文件系统调用（mkdir，不写内容）。失败要说
            # 清楚，不要留一个假的"录制中"。
            self.dashboard_record_var.set(f"无法开始录制：{exc}")
            return
        self.dashboard_recorder.start(
            directory,
            RecordSchema.from_telem_schema(self.dashboard_schema),
            generation=generation,
            link=LinkIdentity(transport, generation),
        )
        self.dashboard_record_generation = generation
        self._dashboard_refresh_record_status()

    def _dashboard_stop_record(self) -> None:
        """只投递结束请求。文件的收尾（尾行 + close）在写线程里做。"""
        self.dashboard_recorder.stop()
        self._dashboard_refresh_record_status()

    def _dashboard_refresh_record_status(self) -> None:
        """把服务状态映射成一行字。失败、被动结束、正常保存必须能分辨。

        路径也从状态里取：`starting` 阶段文件还没建出来，`path` 是 None，界面显示
        "正在创建录制文件…"，而不是先编一个文件名出来。
        """
        status = self.dashboard_recorder.status()
        if status.path is not None:
            self.dashboard_record_path = status.path
        if self.dashboard_record_var is None:
            return
        text = status.describe()
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
