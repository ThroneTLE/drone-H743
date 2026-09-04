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


DASHBOARD_COLUMNS = 12
DASHBOARD_ROW_HEIGHT = 62
DASHBOARD_MAX_ROWS = 40

# 组件类型标识。字符串而不是枚举：它们要原样进 JSON，而且用户导出的布局要能
# 用肉眼看懂。第一批（R-T1-5）三种，第二批（R-T1-5b）四种。
TILE_WAVE = "wave"
TILE_VALUE = "value"
TILE_PARAM = "param"
TILE_GAUGE = "gauge"
TILE_BUTTON = "button"
TILE_ATTITUDE = "attitude"
TILE_CHANNELS = "channels"

LAYOUT_VERSION = 1


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
        return {
            "version": LAYOUT_VERSION,
            "active": self.active,
            "workspaces": [workspace.to_json() for workspace in self.workspaces],
        }

    @classmethod
    def from_json(cls, raw: object) -> "DashboardLayout | None":
        if not isinstance(raw, dict):
            return None
        workspaces_raw = raw.get("workspaces", [])
        if not isinstance(workspaces_raw, list):
            return None
        workspaces = [w for w in (Workspace.from_json(e) for e in workspaces_raw) if w]
        if not workspaces:
            return None
        try:
            active = int(raw.get("active", 0))
        except (TypeError, ValueError):
            active = 0
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

# 14 个增益参数通道，顺序与固件通道表一致（app_telemetry.c）。
PARAM_CHANNEL_NAMES = (
    "roll_rate_kd", "pitch_rate_kd", "yaw_angle_kp", "yaw_rate_kd",
    "pos_x_kp", "pos_y_kp", "vel_x_kd", "vel_y_kd",
    "vel_loop_enable", "roll_angle_kp", "pitch_angle_kp",
    "pos_z_kp", "pos_z_ki", "vel_z_kd",
)

WAVE_COLSPAN, WAVE_ROWSPAN = 6, 5
CARD_COLSPAN, CARD_ROWSPAN = 3, 2


def flight_monitor_workspace() -> Workspace:
    """“飞行监控”：三张波形 + 四张数值卡。

    姿态角、速度、位置各一张波形，因为这三组量要看趋势；光流高度、速度分量、
    Fusion 加速度误差用数值卡——它们是"现在是多少"的问题，画成曲线反而要眯着
    眼睛读刻度（作者看过 R-T1-3 截图后的原话）。
    """
    return Workspace(
        name="飞行监控",
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
    """“控制器调参”：14 张参数滑块卡 + 两张波形，滑块与波形同屏。

    R-T1-3 被打回的直接原因就是滑块和波形不在一个视野里——调参时要边拖边看
    响应，两者不同屏等于把这件事拆成了两步。
    """
    tiles = [
        TileSpec(TILE_WAVE, 0, 0, WAVE_COLSPAN, WAVE_ROWSPAN,
                 ["roll", "pitch", "yaw"], {"title": "姿态角"}),
        TileSpec(TILE_WAVE, 6, 0, WAVE_COLSPAN, WAVE_ROWSPAN,
                 ["vel_est_x", "vel_est_y"], {"title": "速度估计"}),
    ]
    for index, name in enumerate(PARAM_CHANNEL_NAMES):
        column = (index % 4) * CARD_COLSPAN
        row = WAVE_ROWSPAN + (index // 4) * CARD_ROWSPAN
        tiles.append(TileSpec(TILE_PARAM, column, row, CARD_COLSPAN, CARD_ROWSPAN, [name]))
    return Workspace(name="控制器调参", tiles=tiles)


def default_layout() -> DashboardLayout:
    return DashboardLayout(
        workspaces=[flight_monitor_workspace(), controller_tuning_workspace()],
        active=0,
    )


PRESET_BUILDERS = {
    "飞行监控": flight_monitor_workspace,
    "控制器调参": controller_tuning_workspace,
}


__all__ = [
    "CARD_COLSPAN",
    "CARD_ROWSPAN",
    "DASHBOARD_COLUMNS",
    "DASHBOARD_MAX_ROWS",
    "DASHBOARD_ROW_HEIGHT",
    "DashboardLayout",
    "LAYOUT_VERSION",
    "PARAM_CHANNEL_NAMES",
    "PRESET_BUILDERS",
    "TILE_ATTITUDE",
    "TILE_BUTTON",
    "TILE_CHANNELS",
    "TILE_GAUGE",
    "TILE_PARAM",
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
]
