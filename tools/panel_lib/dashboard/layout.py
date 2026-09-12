"""工作台的布局模型：网格、tile 规格、工作区、JSON、出厂预设。

**本模块不 import tkinter**。布局规则（吸附网格、越界钳制、重叠拒绝、序列化
往返）是纯数据变换，放在这里就能在没有显示环境的机器上跑测试，也能让编辑器
的交互代码只剩"把鼠标位置换算成格子坐标"这一件事。

网格取 12 列（Synex 用 96 列，那是给像素级微调准备的；这里的组件最小 2 列，
12 列已经能表达 1/2、1/3、1/4、2/3 这些常用宽度，而且格子够大，拖动时不用
瞄准）。行高固定，行数由内容决定。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from .parameter_layout_migration import migrate_parameter_bindings


DASHBOARD_COLUMNS = 12
DASHBOARD_ROW_HEIGHT = 62
DASHBOARD_MAX_ROWS = 40

# 组件类型标识。字符串而不是枚举：它们要原样进 JSON，而且用户导出的布局要能
# 用肉眼看懂。第一批（R-T1-5）三种，第二批（R-T1-5b）四种，外加一个不绑通道的
# 分组标题条。
TILE_WAVE = "wave"
TILE_VALUE = "value"
TILE_PARAM = "param"
TILE_GAUGE = "gauge"
TILE_BUTTON = "button"
TILE_ATTITUDE = "attitude"
TILE_CHANNELS = "channels"
TILE_SECTION = "section"

LAYOUT_VERSION = 1

# 仿真通道的前缀。只在本机模拟器跑着的时候存在，接真机时一条都没有。
SIMULATION_CHANNEL_PREFIX = "sim_"


@dataclass(frozen=True)
class TileGeometry:
    """一个 tile 在网格里占的矩形，单位是格子。"""

    col: int
    row: int
    colspan: int
    rowspan: int

    @property
    def col_end(self) -> int:
        return self.col + self.colspan

    @property
    def row_end(self) -> int:
        return self.row + self.rowspan

    def overlaps(self, other: "TileGeometry") -> bool:
        return (
            self.col < other.col_end
            and other.col < self.col_end
            and self.row < other.row_end
            and other.row < self.row_end
        )


@dataclass
class TileSpec:
    """一个组件的完整规格。`bindings` 是**通道名**，不是通道号。"""

    type: str
    col: int = 0
    row: int = 0
    colspan: int = 3
    rowspan: int = 2
    bindings: list[str] = field(default_factory=list)
    options: dict = field(default_factory=dict)

    @property
    def geometry(self) -> TileGeometry:
        return TileGeometry(self.col, self.row, self.colspan, self.rowspan)

    def moved(self, col: int, row: int) -> "TileSpec":
        return replace(self, col=col, row=row)

    def resized(self, colspan: int, rowspan: int) -> "TileSpec":
        return replace(self, colspan=colspan, rowspan=rowspan)

    def to_json(self) -> dict:
        return {
            "type": self.type,
            "col": self.col,
            "row": self.row,
            "colspan": self.colspan,
            "rowspan": self.rowspan,
            "bindings": list(self.bindings),
            "options": dict(self.options),
        }

    @classmethod
    def from_json(cls, raw: object) -> "TileSpec | None":
        """坏数据一律返回 None 而不是抛异常。

        面板状态文件是可以被手改的；一个字段写错就让整个上位机起不来，代价
        远大于丢掉那一个 tile。丢掉的 tile 由调用方计数并显示出来。
        """
        if not isinstance(raw, dict):
            return None
        tile_type = raw.get("type")
        if not isinstance(tile_type, str) or not tile_type:
            return None
        try:
            spec = cls(
                type=tile_type,
                col=int(raw.get("col", 0)),
                row=int(raw.get("row", 0)),
                colspan=int(raw.get("colspan", 3)),
                rowspan=int(raw.get("rowspan", 2)),
            )
        except (TypeError, ValueError):
            return None
        bindings = raw.get("bindings", [])
        if isinstance(bindings, list):
            spec.bindings = [str(name) for name in bindings if isinstance(name, str)]
        options = raw.get("options", {})
        if isinstance(options, dict):
            spec.options = dict(options)
        spec.bindings, spec.options = migrate_parameter_bindings(spec.bindings, spec.options)
        return clamp_tile(spec)


def clamp_tile(spec: TileSpec, columns: int = DASHBOARD_COLUMNS) -> TileSpec:
    """把 tile 钳进网格。越界不是错误，是"拖出去了"，钳回来即可。"""
    colspan = max(1, min(int(spec.colspan), columns))
    rowspan = max(1, min(int(spec.rowspan), DASHBOARD_MAX_ROWS))
    col = max(0, min(int(spec.col), columns - colspan))
    row = max(0, min(int(spec.row), DASHBOARD_MAX_ROWS - rowspan))
    spec.col, spec.row, spec.colspan, spec.rowspan = col, row, colspan, rowspan
    return spec


def can_place(tiles: list[TileSpec], candidate: TileSpec,
              ignore: TileSpec | None = None,
              columns: int = DASHBOARD_COLUMNS) -> bool:
    """候选位置是否合法：不越界、不与别的 tile 重叠。

    重叠直接拒绝而不是自动挤开：自动挤开会让一次拖动连锁改动半个工作区，
    用户既看不懂也撤不回。
    """
    if candidate.colspan < 1 or candidate.rowspan < 1:
        return False
    if candidate.col < 0 or candidate.row < 0:
        return False
    if candidate.col + candidate.colspan > columns:
        return False
    if candidate.row + candidate.rowspan > DASHBOARD_MAX_ROWS:
        return False
    geometry = candidate.geometry
    for tile in tiles:
        if tile is ignore or tile is candidate:
            continue
        if geometry.overlaps(tile.geometry):
            return False
    return True


def find_free_slot(tiles: list[TileSpec], colspan: int, rowspan: int,
                   columns: int = DASHBOARD_COLUMNS) -> tuple[int, int]:
    """给新组件找第一个放得下的位置，从左上往右下扫。"""
    for row in range(DASHBOARD_MAX_ROWS - rowspan + 1):
        for col in range(columns - colspan + 1):
            probe = TileSpec(type="", col=col, row=row, colspan=colspan, rowspan=rowspan)
            if can_place(tiles, probe, columns=columns):
                return col, row
    return 0, 0


@dataclass
class Workspace:
    name: str
    tiles: list[TileSpec] = field(default_factory=list)
    #
    # 仿真专用工作区：只在仿真运行期间存在，**永不落盘**（见 to_json 的过滤）。
    #
    # 为什么要这条：仿真的调参滑块绑 sim_* 通道，而那些通道一一映射到真实的
    # coax.* 参数。工作区一旦被写进面板布局文件，下次接上真机它还在，拖一下
    # 滑块就是往飞机上写参数。靠"停止时记得移除"是不够的——进程被杀、断电、
    # 异常退出都不会走到那条路径，而落盘可能已经发生过了。
    ephemeral: bool = False
    #
    # 归属：本工作区是哪个一级工作区下面的一个视图（填父工作区的 `name`）。
    # 选择条据此分两行，见 `dashboard/workspace_bar.py`。
    #
    # **不进 JSON**（`to_json` 里没有这一项）。目前只有 ephemeral 的仿真工作区用它，
    # 而那些工作区本来就永不落盘；给一个落不了盘的字段加序列化，只会多出一条
    # "写得进去、读不回来"的路径要维护。
    parent: str | None = None

    def rows_used(self) -> int:
        return max((tile.row + tile.rowspan for tile in self.tiles), default=1)

    def bound_channels(self) -> list[str]:
        """本工作区所有组件绑定的通道名，去重、保持首次出现的顺序。"""
        seen: list[str] = []
        for tile in self.tiles:
            for name in tile.bindings:
                if name not in seen:
                    seen.append(name)
        return seen

    def to_json(self) -> dict:
        return {"name": self.name, "tiles": [tile.to_json() for tile in self.tiles]}

    @classmethod
    def from_json(cls, raw: object) -> "Workspace | None":
        if not isinstance(raw, dict):
            return None
        name = raw.get("name")
        if not isinstance(name, str) or not name:
            return None
        tiles_raw = raw.get("tiles", [])
        if not isinstance(tiles_raw, list):
            return None
        tiles: list[TileSpec] = []
        for entry in tiles_raw:
            spec = TileSpec.from_json(entry)
            if spec is None:
                continue
            # 载入时也做一次重叠检查：手改过的文件可能让两个 tile 叠在一起，
            # 那样界面上会有一个永远点不到。放不下的按新组件重新找位置。
            if not can_place(tiles, spec):
                spec.col, spec.row = find_free_slot(tiles, spec.colspan, spec.rowspan)
            tiles.append(spec)
        return cls(name=name, tiles=tiles)


def is_stale_simulation_view(workspace: "Workspace") -> bool:
    """这个工作区是不是一份**遗留在状态文件里**的仿真视图。

    `Workspace.ephemeral` 只挡得住"今天写进去"，挡不住"昨天已经写进去了"：
    `from_json` 造出来的工作区一律 `ephemeral=False`，所以在 ephemeral 这条规矩
    之前漏进状态文件的那几个，会一次次被读回来、又一次次被原样写回去，永远出不去。
    作者截图里那三个"…· P—PID"就是这么来的——当时连飞控都没接，三页全是"通道不存在"。

    判据是**绑定**而不是名字：名字能被改，而"这一页只认 sim_* 通道"这件事改不了。
    要求 `all()` 而不是 `any()`：用户自己搭的工作区里混进一张仿真卡片，整页不该
    被删掉——那张卡片在真机上只是显示"通道不存在"，删掉整页的代价大得多。
    """
    channels = workspace.bound_channels()
    return bool(channels) and all(
        name.startswith(SIMULATION_CHANNEL_PREFIX) for name in channels
    )


@dataclass
class DashboardLayout:
    workspaces: list[Workspace] = field(default_factory=list)
    active: int = 0

    def active_workspace(self) -> Workspace:
        if not self.workspaces:
            self.workspaces = [Workspace(name="工作区 1")]
        self.active = max(0, min(self.active, len(self.workspaces) - 1))
        return self.workspaces[self.active]

    def to_json(self) -> dict:
        # ephemeral 工作区（仿真专用）在这里被滤掉，所以它们进不了状态文件，
        # 也就不可能在下一次接真机时冒出来。见 Workspace.ephemeral 的注释。
        persisted = [w for w in self.workspaces if not w.ephemeral]
        active = self.active
        if active >= len(persisted):
            # 仿真工作区正被选中时落盘：回到最后一个持久工作区，而不是写一个
            # 越界的索引让下次启动落到别处。
            active = max(0, len(persisted) - 1)
        return {
            "version": LAYOUT_VERSION,
            "active": active,
            "workspaces": [workspace.to_json() for workspace in persisted],
        }

    @classmethod
    def from_json(cls, raw: object) -> "DashboardLayout | None":
        if not isinstance(raw, dict):
            return None
        workspaces_raw = raw.get("workspaces", [])
        if not isinstance(workspaces_raw, list):
            return None
        parsed = [w for w in (Workspace.from_json(e) for e in workspaces_raw) if w]
        try:
            active = int(raw.get("active", 0))
        except (TypeError, ValueError):
            active = 0
        # 载入是清掉历史遗留仿真视图的唯一时机：写出去那一侧已经由 ephemeral 挡住，
        # 但挡不住存量。这里不清，它们就会被原样写回去，一辈子留在选择条上。
        kept = [index for index, w in enumerate(parsed) if not is_stale_simulation_view(w)]
        workspaces = [parsed[index] for index in kept]
        if not workspaces:
            return None
        # active 按存活项重新编号，而不是简单钳一下：被删的正好是选中项时，
        # 钳出来的下标会落到一个完全无关的工作区上，用户看到的是"我的布局怎么变了"。
        if parsed:
            active = max(0, min(active, len(parsed) - 1))
        active = min(sum(1 for index in kept if index < active), len(workspaces) - 1)
        layout = cls(workspaces=workspaces, active=active)
        layout.active_workspace()       # 顺手把 active 钳进范围
        return layout

    def dumps(self) -> str:
        return json.dumps(self.to_json(), ensure_ascii=False, indent=2)

    @classmethod
    def loads(cls, text: str) -> "DashboardLayout | None":
        try:
            return cls.from_json(json.loads(text))
        except (json.JSONDecodeError, TypeError, ValueError):
            return None


# ------------------------------------------------------------------ 出厂预设

# 四个串级环的增益分组，用固件参数表里的**真名**（通道 64 起）。顺序就是串级
# 从外到内：位置 → 速度 → 角度 → 角速度。这也是调参时唯一安全的推进顺序
# （内环没稳之前动外环，看到的响应不是外环的），所以分块顺序照抄它。
#
# 两条不那么显然的归属：
#
# * `vel_loop_enable` 是速度环的总开关而不是增益，但速度环那几个 kp/ki/kd 有没有
#   效果全看它，单独拎出去会让人对着不起作用的滑块调半天。
# * 两个低通截止（`accel_lpf` / `angular_accel_lpf`）跟各自的 D 项放在一起。
#   D 项吃的是差分噪声还是真信号由截止频率决定，先定它再调 Kd；不同屏的话，
#   调 Kd 的人根本不知道自己在跟什么较劲。
PARAM_LOOP_GROUPS = (
    ("位置环 P", ("pos_x_kp", "pos_y_kp", "pos_z_kp")),
    ("速度环 PID", ("vel_x_kp", "vel_y_kp", "vel_z_kp",
                    "vel_x_ki", "vel_y_ki", "vel_z_ki",
                    "vel_x_kd", "vel_y_kd", "vel_z_kd",
                    "vel_loop_enable", "accel_lpf")),
    ("角度环 P", ("att_roll_kp", "att_pitch_kp", "att_yaw_kp")),
    ("角速度环 PID", ("rate_roll_kp", "rate_pitch_kp", "rate_yaw_kp",
                      "rate_roll_ki", "rate_pitch_ki", "rate_yaw_ki",
                      "rate_roll_kd", "rate_pitch_kd", "rate_yaw_kd",
                      "angular_accel_lpf")),
)

# 出厂预设摆出来的滑块 = 上面四块拍平。顺序即摆放顺序。
PARAM_CHANNEL_NAMES = tuple(
    name for _, names in PARAM_LOOP_GROUPS for name in names
)

# 两个出厂工作区的名字。取成常量是因为"控制器调参"还被当作父工作区的**标识**用
# （仿真的三个 P—PID 视图挂在它下面，见 `Workspace.parent`）；名字散成字面量，
# 改一处漏一处的结果是那三个视图默默退回一级，正好回到本次要修的样子。
FLIGHT_MONITOR_WORKSPACE = "飞行监控"
CONTROLLER_TUNING_WORKSPACE = "控制器调参"

WAVE_COLSPAN, WAVE_ROWSPAN = 6, 5
CARD_COLSPAN, CARD_ROWSPAN = 3, 2
SECTION_ROWSPAN = 1
PARAM_CARDS_PER_ROW = DASHBOARD_COLUMNS // CARD_COLSPAN


def flight_monitor_workspace() -> Workspace:
    """“飞行监控”：三张波形 + 四张数值卡。

    姿态角、速度、位置各一张波形，因为这三组量要看趋势；光流高度、速度分量、
    Fusion 加速度误差用数值卡——它们是"现在是多少"的问题，画成曲线反而要眯着
    眼睛读刻度（作者看过 R-T1-3 截图后的原话）。
    """
    return Workspace(
        name=FLIGHT_MONITOR_WORKSPACE,
        tiles=[
            TileSpec(TILE_WAVE, 0, 0, WAVE_COLSPAN, WAVE_ROWSPAN,
                     ["roll", "pitch", "yaw"], {"title": "姿态角"}),
            TileSpec(TILE_WAVE, 6, 0, WAVE_COLSPAN, WAVE_ROWSPAN,
                     ["vel_est_x", "vel_est_y"], {"title": "速度估计"}),
            TileSpec(TILE_WAVE, 0, 5, WAVE_COLSPAN, WAVE_ROWSPAN,
                     ["pos_est_x", "pos_est_y"], {"title": "位置估计"}),
            TileSpec(TILE_VALUE, 6, 5, CARD_COLSPAN, CARD_ROWSPAN, ["flow_height"]),
            TileSpec(TILE_VALUE, 9, 5, CARD_COLSPAN, CARD_ROWSPAN, ["vel_est_x"]),
            TileSpec(TILE_VALUE, 6, 7, CARD_COLSPAN, CARD_ROWSPAN, ["vel_est_y"]),
            TileSpec(TILE_VALUE, 9, 7, CARD_COLSPAN, CARD_ROWSPAN, ["fusion_acc_err"]),
        ],
    )


def controller_tuning_workspace() -> Workspace:
    """“控制器调参”：两张波形 + 14 张参数滑块卡，滑块按串级环分块。

    R-T1-3 被打回的直接原因就是滑块和波形不在一个视野里——调参时要边拖边看
    响应，两者不同屏等于把这件事拆成了两步。

    滑块此前按固件通道号顺序平铺，于是 `roll_rate_kd` 和 `roll_angle_kp` 隔着
    七张卡，同一个环的三个轴也不挨着；调参时要在屏幕上来回找。现在改成按
    `PARAM_LOOP_GROUPS` 分块，每块一条标题条，块内一行放得下就放一行。
    """
    tiles = [
        TileSpec(TILE_WAVE, 0, 0, WAVE_COLSPAN, WAVE_ROWSPAN,
                 ["roll", "pitch", "yaw"], {"title": "姿态角"}),
        TileSpec(TILE_WAVE, 6, 0, WAVE_COLSPAN, WAVE_ROWSPAN,
                 ["vel_est_x", "vel_est_y"], {"title": "速度估计"}),
    ]
    row = WAVE_ROWSPAN
    for title, names in PARAM_LOOP_GROUPS:
        tiles.append(TileSpec(TILE_SECTION, 0, row, DASHBOARD_COLUMNS, SECTION_ROWSPAN,
                              [], {"title": title}))
        row += SECTION_ROWSPAN
        for index, name in enumerate(names):
            tiles.append(TileSpec(
                TILE_PARAM,
                (index % PARAM_CARDS_PER_ROW) * CARD_COLSPAN,
                row + (index // PARAM_CARDS_PER_ROW) * CARD_ROWSPAN,
                CARD_COLSPAN, CARD_ROWSPAN, [name],
            ))
        card_rows = -(-len(names) // PARAM_CARDS_PER_ROW)
        row += card_rows * CARD_ROWSPAN
    return Workspace(name=CONTROLLER_TUNING_WORKSPACE, tiles=tiles)


def default_layout() -> DashboardLayout:
    return DashboardLayout(
        workspaces=[flight_monitor_workspace(), controller_tuning_workspace()],
        active=0,
    )


PRESET_BUILDERS = {
    FLIGHT_MONITOR_WORKSPACE: flight_monitor_workspace,
    CONTROLLER_TUNING_WORKSPACE: controller_tuning_workspace,
}


__all__ = [
    "CARD_COLSPAN",
    "CARD_ROWSPAN",
    "CONTROLLER_TUNING_WORKSPACE",
    "DASHBOARD_COLUMNS",
    "DASHBOARD_MAX_ROWS",
    "DASHBOARD_ROW_HEIGHT",
    "DashboardLayout",
    "FLIGHT_MONITOR_WORKSPACE",
    "LAYOUT_VERSION",
    "PARAM_CARDS_PER_ROW",
    "PARAM_CHANNEL_NAMES",
    "PARAM_LOOP_GROUPS",
    "PRESET_BUILDERS",
    "SECTION_ROWSPAN",
    "SIMULATION_CHANNEL_PREFIX",
    "TILE_ATTITUDE",
    "TILE_BUTTON",
    "TILE_CHANNELS",
    "TILE_GAUGE",
    "TILE_PARAM",
    "TILE_SECTION",
    "TILE_VALUE",
    "TILE_WAVE",
    "TileGeometry",
    "TileSpec",
    "WAVE_COLSPAN",
    "WAVE_ROWSPAN",
    "Workspace",
    "can_place",
    "clamp_tile",
    "controller_tuning_workspace",
    "default_layout",
    "find_free_slot",
    "flight_monitor_workspace",
    "is_stale_simulation_view",
]
