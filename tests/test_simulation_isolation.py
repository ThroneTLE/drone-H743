"""仿真调参不许污染真机调参。

风险是具体的：仿真的调参滑块绑 `sim_*` 通道，而 tools/sim_xz/control_catalog.py
第 6 列把它们一一映射到真实的 `coax.*` 参数。仿真工作区一旦留在仪表盘布局里，
接上真机之后拖一下滑块就是往飞机上写参数。

"停止时记得移除"挡不住这件事——进程被杀、断电、异常退出都不会走到那条路径，
而落盘可能已经发生过了。所以规则改成结构性的：仿真工作区是 ephemeral 的，
`DashboardLayout.to_json()` 直接把它们滤掉，**永远进不了状态文件**。
"""

from __future__ import annotations

import json
from pathlib import Path

from tools.panel_lib.dashboard.layout import (
    DashboardLayout,
    TileSpec,
    TILE_PARAM,
    Workspace,
)
from tools.panel_lib.simulation_workspaces import simulation_workspaces


ROOT = Path(__file__).resolve().parents[1]


def test_simulation_workspaces_are_marked_ephemeral() -> None:
    spaces = simulation_workspaces()
    assert len(spaces) == 3
    assert all(space.ephemeral for space in spaces)


def test_ephemeral_workspaces_never_reach_the_state_file() -> None:
    real = Workspace(name="我的调参区",
                     tiles=[TileSpec(TILE_PARAM, 0, 0, 3, 2, ["coax.rate_roll_kp"], {})])
    layout = DashboardLayout(workspaces=[real, *simulation_workspaces()], active=0)

    encoded = json.loads(layout.dumps())
    names = [w["name"] for w in encoded["workspaces"]]
    assert names == ["我的调参区"]

    # 落盘的内容里不能出现任何 sim_* 通道，否则下次接真机就会翻到它们。
    assert "sim_" not in layout.dumps()


def test_active_index_is_clamped_when_a_simulation_workspace_is_selected() -> None:
    """仿真工作区被选中时落盘：active 必须回到最后一个持久工作区。

    直接写下越界的索引会让下次启动落到一个完全无关的工作区上——用户看到的是
    "我的布局怎么变了"，而原因藏在一次和布局毫无关系的仿真会话里。
    """
    real = Workspace(name="我的调参区")
    layout = DashboardLayout(workspaces=[real, *simulation_workspaces()], active=2)

    encoded = layout.to_json()
    assert encoded["active"] == 0
    assert len(encoded["workspaces"]) == 1

    restored = DashboardLayout.from_json(encoded)
    assert restored is not None
    assert restored.active_workspace().name == "我的调参区"


def test_a_state_file_written_before_the_ephemeral_rule_gets_cleaned_on_load() -> None:
    """存量也要清掉，不只是不再写新的。

    `ephemeral` 是后加的。在它之前漏进 panel_state.json 的仿真工作区，被
    `from_json` 造回来时一律 `ephemeral=False`，于是每次启动读回来、每次退出又
    原样写回去——作者截图里那三个"…· P—PID"就是这么钉在选择条上的，当时连飞控
    都没接，三页全是"通道不存在"。载入是唯一能把它们清出去的地方。
    """
    leaked = {
        "version": 1,
        "active": 3,
        "workspaces": [
            {"name": "飞行监控", "tiles": [
                {"type": TILE_PARAM, "bindings": ["roll"]}]},
            *[w.to_json() for w in simulation_workspaces()],
        ],
    }
    restored = DashboardLayout.from_json(leaked)
    assert restored is not None
    assert [w.name for w in restored.workspaces] == ["飞行监控"]
    # active 原本指着第 3 个（一个仿真页）：必须落到存活的那个，而不是越界后
    # 被钳到一个无关的工作区上。
    assert restored.active_workspace().name == "飞行监控"


def test_cleanup_keys_on_bindings_so_a_renamed_view_still_goes() -> None:
    """判据是"这一页只认 sim_* 通道"，不是名字——名字能被改。

    反过来，用户自己搭的工作区里混进一张仿真卡片，整页不能被删：那张卡片在真机上
    只是显示"通道不存在"，而删掉的是用户排了半天的布局。
    """
    renamed = Workspace(name="我随手改的名字", tiles=[
        TileSpec(TILE_PARAM, 0, 0, 3, 2, ["sim_pos_z_kp"], {})])
    mixed = Workspace(name="我的工作区", tiles=[
        TileSpec(TILE_PARAM, 0, 0, 3, 2, ["sim_pos_z_kp"], {}),
        TileSpec(TILE_PARAM, 3, 0, 3, 2, ["coax.rate_roll_kp"], {})])

    restored = DashboardLayout.from_json(
        DashboardLayout(workspaces=[renamed, mixed], active=0).to_json())
    assert restored is not None
    assert [w.name for w in restored.workspaces] == ["我的工作区"]


def test_a_layout_that_is_nothing_but_simulation_falls_back_to_the_presets() -> None:
    """全被清光时返回 None，调用方（`_dashboard_load_layout`）落到出厂预设。"""
    layout = DashboardLayout(workspaces=list(simulation_workspaces()), active=0)
    encoded = {"version": 1, "active": 0,
               "workspaces": [w.to_json() for w in layout.workspaces]}
    assert DashboardLayout.from_json(encoded) is None


def test_simulation_launcher_lives_in_its_own_tab_not_the_main_header() -> None:
    """仿真启动栏不再常驻主窗口顶部——那个位置留给解锁状态。

    位置之争不是排版偏好：主窗口最上方是每次上机都要先看一眼的地方，
    而"能不能解锁、差什么"才是那个必须一眼看到的东西。
    """
    shell = (ROOT / "tools" / "panel_lib" / "shell.py").read_text(encoding="utf-8")

    assert "mount_arm_banner(self, root)" in shell
    assert "mount_simulation_bar" not in shell
    assert "mount_simulation(self, simulation)" in shell
    assert 'self.notebook.add(simulation_scroll, text="仿真")' in shell
