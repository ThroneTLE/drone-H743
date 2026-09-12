"""仿真的三个 P—PID 参数页是"控制器调参"的子视图，不是和它平级的工作区。

作者看到的界面是这样的（截图）：

    状态监视  ○飞行监控 ○控制器调参 ○水平平动·P—PID ○高度·P—PID ◉俯仰姿态·P—PID

五个平级单选钮，而且截图里还没接飞控——后三个全是"通道不存在"。它们不该在
那儿：那是"控制器调参"在仿真跑起来之后才有的几种参数视图，仿真一停就整组消失
（绑 `sim_*` 通道，`Workspace.ephemeral` 保证永不落盘）。

所以判据有三条，一条都不能少：

  1. 它们不出现在第一行；
  2. 它们挂在"控制器调参"下面，而不是随便哪个工作区下面；
  3. 仿真没开时第二行**整行不存在**，不是显示一个只有一项的空选择器。
"""

from __future__ import annotations

import tkinter as tk

import pytest

from tools.panel_lib.dashboard.layout import (
    CONTROLLER_TUNING_WORKSPACE,
    FLIGHT_MONITOR_WORKSPACE,
    TileSpec,
    TILE_PARAM,
    Workspace,
    default_layout,
)
from tools.panel_lib.dashboard.workspace_bar import (
    PARENT_VIEW_LABEL,
    workspace_tree,
)
from tools.panel_lib.simulation_workspaces import simulation_workspaces


SIM_NAMES = ["水平平动 · P—PID", "高度 · P—PID", "俯仰姿态 · P—PID"]


def with_simulation() -> list[Workspace]:
    return [*default_layout().workspaces, *simulation_workspaces()]


# ------------------------------------------------------------------ 归属

def test_simulation_workspaces_declare_the_tuning_workspace_as_parent() -> None:
    spaces = simulation_workspaces()
    assert [w.name for w in spaces] == SIM_NAMES
    assert all(w.parent == CONTROLLER_TUNING_WORKSPACE for w in spaces)


def test_parent_is_not_persisted_because_these_workspaces_never_persist() -> None:
    """`parent` 只在运行期有意义：用它的工作区全是 ephemeral 的。

    钉住这条是为了拦"顺手给 to_json 加个字段"——加了就会出现一个写得进状态文件、
    读回来却是 None 的字段，而唯一的症状是子视图某天悄悄退回一级。
    """
    assert "parent" not in Workspace(name="x", parent="y").to_json()
    restored = Workspace.from_json({"name": "x", "tiles": [], "parent": "y"})
    assert restored is not None and restored.parent is None


# ------------------------------------------------------------------ 分组

def test_simulation_views_are_not_top_level() -> None:
    tree = workspace_tree(with_simulation(), active=0)
    assert [chip.label for chip in tree.roots] == [
        FLIGHT_MONITOR_WORKSPACE, CONTROLLER_TUNING_WORKSPACE]


def test_the_view_row_is_empty_until_simulation_installs_its_workspaces() -> None:
    """作者的原话是"开启仿真之后才会显示"。空元组 = 那一行整个不画。"""
    for active in range(2):
        assert workspace_tree(default_layout().workspaces, active).views == ()


def test_selecting_the_tuning_workspace_reveals_its_views() -> None:
    tree = workspace_tree(with_simulation(), active=1)
    assert [chip.label for chip in tree.views] == [PARENT_VIEW_LABEL, *SIM_NAMES]
    assert tree.active_root == 1
    # 第一项指回父工作区自己：选中它就是回到那 24 个增益滑块。
    assert tree.views[0].index == 1


def test_a_selected_view_keeps_its_parent_highlighted_on_the_first_row() -> None:
    """选中"俯仰姿态"时，第一行必须仍停在"控制器调参"。

    否则第二行会凭空冒出来而第一行谁都没选中，看上去像是界面丢了状态。
    """
    tree = workspace_tree(with_simulation(), active=4)
    assert tree.active == 4
    assert tree.active_root == 1
    assert [chip.label for chip in tree.views] == [PARENT_VIEW_LABEL, *SIM_NAMES]


def test_views_belong_to_their_own_parent_only() -> None:
    """站在"飞行监控"上时不显示别人的子视图——哪怕仿真正在跑。"""
    assert workspace_tree(with_simulation(), active=0).views == ()


def test_an_orphan_view_falls_back_to_the_first_row() -> None:
    """父工作区不在列表里（被删/被改名）时，子视图退回一级而不是消失。

    消失的代价是用户既看不到它、也没法把它删掉；多一个平级单选钮只是难看。
    """
    orphan = Workspace(name="没爹的视图", parent="并不存在的工作区")
    tree = workspace_tree([*default_layout().workspaces, orphan], active=2)
    assert [chip.label for chip in tree.roots][-1] == "没爹的视图"
    assert tree.active_root == 2
    assert tree.views == ()


def test_nesting_stops_at_one_level() -> None:
    """孙子辈不认爷爷：父必须自己是一级工作区，否则退回一级。

    两行是刻意的上限。第三行放不进这条工具条，而"看不见的层"比平级更难查。
    """
    child = Workspace(name="子", parent=CONTROLLER_TUNING_WORKSPACE)
    grandchild = Workspace(name="孙", parent="子")
    tree = workspace_tree([*default_layout().workspaces, child, grandchild], active=3)
    assert [chip.label for chip in tree.roots] == [
        FLIGHT_MONITOR_WORKSPACE, CONTROLLER_TUNING_WORKSPACE, "孙"]


def test_empty_and_out_of_range_inputs_do_not_raise() -> None:
    """布局文件是可以被手改的；越界的 active 不该带走整个面板。"""
    assert workspace_tree([], active=7).roots == ()
    tree = workspace_tree(with_simulation(), active=99)
    assert tree.active == len(with_simulation()) - 1


# ------------------------------------------------------------------ 真界面

@pytest.fixture
def app():
    from tools.drone_tcp_panel import DronePanel

    try:
        instance = DronePanel()
    except tk.TclError as exc:                      # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    instance._save_panel_state = lambda: None       # 测试不碰用户的 panel_state.json
    try:
        yield instance
    finally:
        instance.destroy()


def labels(frame) -> list[str]:
    return [child.cget("text") for child in frame.winfo_children()
            if child.winfo_class() == "TRadiobutton"]


def install(app, workspaces, active) -> None:
    app.dashboard_layout.workspaces = list(workspaces)
    app.dashboard_workspace_var.set(active)
    app._dashboard_rebuild_workspace_bar()


def test_the_bar_shows_two_rows_only_while_the_views_exist(app) -> None:
    install(app, default_layout().workspaces, 0)
    assert labels(app.dashboard_workspace_bar) == [
        FLIGHT_MONITOR_WORKSPACE, CONTROLLER_TUNING_WORKSPACE]
    assert labels(app.dashboard_workspace_view_bar) == []
    # 空行也不许留下别的控件（那个"└ 视图"的记号也是仿真开着才有）。
    assert app.dashboard_workspace_view_bar.winfo_children() == []
    # 而且整行要让出版面，不是留个 1 px 的空 frame：这一页的重绘预算贴着线，
    # `_build_dashboard_toolbar` 里的行高注释写过多一行的代价。
    assert app.dashboard_workspace_view_bar.winfo_manager() == ""

    install(app, with_simulation(), 4)
    assert labels(app.dashboard_workspace_bar) == [
        FLIGHT_MONITOR_WORKSPACE, CONTROLLER_TUNING_WORKSPACE]
    assert labels(app.dashboard_workspace_view_bar) == [PARENT_VIEW_LABEL, *SIM_NAMES]
    assert app.dashboard_workspace_root_var.get() == 1
    assert app.dashboard_workspace_var.get() == 4
    # 有内容时才占版面，而且要插在按钮行之前，不能被 pack 追加到页面最底下。
    assert app.dashboard_workspace_view_bar.winfo_manager() == "pack"


def test_clicking_a_view_switches_the_tiles_without_rebuilding_the_bar(app) -> None:
    """点第二行不能重建选择条——那等于在单选钮自己的回调里把它销毁掉。"""
    install(app, with_simulation(), 1)
    chips = [c for c in app.dashboard_workspace_view_bar.winfo_children()
             if c.winfo_class() == "TRadiobutton"]
    before = [str(c) for c in chips]

    chips[2].invoke()                               # "高度 · P—PID"
    assert app.dashboard_layout.active_workspace().name == "高度 · P—PID"
    assert app.dashboard_workspace_root_var.get() == 1
    assert [str(c) for c in app.dashboard_workspace_view_bar.winfo_children()
            if c.winfo_class() == "TRadiobutton"] == before
    assert {"sim_pos_z_kp", "sim_vel_z_ki"} <= set(
        app.dashboard_layout.active_workspace().bound_channels())


def test_clicking_the_parent_returns_to_its_own_tiles(app) -> None:
    """点第一行只能重画第二行。

    连点两下用的是**同一批**控件引用：如果实现改成整条重画，第一下就把这些单选钮
    销毁了，第二下会直接 `TclError: invalid command name`——真界面上表现为第一行
    点一次之后就点不动了。
    """
    install(app, with_simulation(), 4)
    roots = [c for c in app.dashboard_workspace_bar.winfo_children()
             if c.winfo_class() == "TRadiobutton"]

    roots[1].invoke()                               # 已经高亮着，但仍然要能点回来
    assert app.dashboard_layout.active_workspace().name == CONTROLLER_TUNING_WORKSPACE
    assert labels(app.dashboard_workspace_view_bar) == [PARENT_VIEW_LABEL, *SIM_NAMES]
    assert app.dashboard_workspace_var.get() == 1

    roots[0].invoke()                               # 换到没有子视图的一级工作区
    assert app.dashboard_layout.active_workspace().name == FLIGHT_MONITOR_WORKSPACE
    assert app.dashboard_workspace_view_bar.winfo_children() == []

    roots[1].invoke()                               # 再回去，第二行要重新长出来
    assert labels(app.dashboard_workspace_view_bar) == [PARENT_VIEW_LABEL, *SIM_NAMES]


def test_param_tiles_survive_the_two_row_bar(app) -> None:
    """分两行只是选择条的事，工作区内容一格都不该动。"""
    install(app, with_simulation(), 2)
    app._dashboard_switch_workspace(2)
    cards = [t for t in app.dashboard_tiles if t.spec.type == TILE_PARAM]
    assert len(cards) == 4
    assert isinstance(cards[0].spec, TileSpec)
