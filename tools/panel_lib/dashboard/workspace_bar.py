"""工作区选择条：一级工作区一行，某个工作区的子视图另起一行。

**为什么要分两行。** 仿真那三个 P—PID 参数页原本和"飞行监控""控制器调参"排在
同一行单选钮里，看上去是四个平级的工作区。它们不是：

* 它们是"控制器调参"在**仿真跑起来之后**才存在的几种参数视图，仿真一停就整组
  消失（绑的是 `sim_*` 通道，`Workspace.ephemeral` 保证它们永不落盘）；
* 平级排等于承诺"随时可点"，而接真机时那一行里的三项全是"通道不存在"；
* 五个长名字挤一行，真正天天要点的那两个反而不显眼。

**本模块不认识仿真。** 它只看 `Workspace.parent`：任何工作区把 `parent` 写成另一个
顶层工作区的名字，就会被收进那一行下面。仿真只是第一个用户。

分组是纯数据变换（`workspace_tree`），所以能在没有显示环境的机器上测；
只有 `render_workspace_bar` 碰 tkinter。
"""

from __future__ import annotations

import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk


# 子视图行里代表"父工作区自己"的那一项。不直接复用父工作区的名字：那样第一行和
# 第二行会同时高亮同一个词，看不出第二行在选什么。
PARENT_VIEW_LABEL = "全部参数"

# 子视图行左端的层级记号。纯装饰，但没有它两行单选钮看起来还是平级的。
VIEW_ROW_PREFIX = "└ 视图"


@dataclass(frozen=True)
class WorkspaceChip:
    """选择条上的一个单选钮。`index` 是 `DashboardLayout.workspaces` 的下标。

    下标而不是名字：面板其余部分（active、掩码、落盘）全按下标走，这里换一套
    标识只会多出一处需要对齐的地方。
    """

    index: int
    label: str


@dataclass(frozen=True)
class WorkspaceTree:
    roots: tuple[WorkspaceChip, ...]
    # 当前一级工作区的子视图。**空元组 = 这一行整个不显示**，而不是显示一个只有
    # 一项的选择器——仿真没开时就是这种情况。
    views: tuple[WorkspaceChip, ...]
    active: int
    active_root: int


def _parent_name(workspace) -> str:
    name = getattr(workspace, "parent", None)
    return name if isinstance(name, str) and name else ""


def workspace_tree(workspaces, active: int) -> WorkspaceTree:
    """把扁平的工作区列表分成"一级 / 当前一级下的子视图"两层。

    只认一层嵌套，而且父工作区必须真的在列表里。认不出父的（父被改名、被删、
    或者父自己也是子工作区）一律退回一级：宁可多一个平级单选钮，也不能让一个
    工作区在界面上完全够不着——那时候用户既看不到它，也没法把它删掉。
    """
    items = list(workspaces)
    if not items:
        return WorkspaceTree((), (), 0, 0)

    top_names = {w.name for w in items if not _parent_name(w)}
    owners = [name if (name := _parent_name(w)) in top_names else "" for w in items]

    root_index: dict[str, int] = {}
    for index, workspace in enumerate(items):
        if not owners[index]:
            root_index.setdefault(workspace.name, index)

    active = max(0, min(int(active), len(items) - 1))
    active_root = root_index.get(owners[active], active)

    roots = tuple(
        WorkspaceChip(index, workspace.name)
        for index, workspace in enumerate(items)
        if not owners[index]
    )
    children = tuple(
        WorkspaceChip(index, workspace.name)
        for index, workspace in enumerate(items)
        if owners[index] and root_index[owners[index]] == active_root
    )
    views = (WorkspaceChip(active_root, PARENT_VIEW_LABEL), *children) if children else ()
    return WorkspaceTree(roots, views, active, active_root)


def _clear(row: ttk.Frame) -> None:
    for child in row.winfo_children():
        child.destroy()


def render_workspace_bar(root_row: ttk.Frame, view_row: ttk.Frame, tree: WorkspaceTree,
                         *, root_var: tk.IntVar, view_var: tk.IntVar,
                         on_root, on_view) -> bool:
    """整条选择条重画。工作区列表变了（换布局、仿真起停）才需要走这条。

    返回第二行是否有内容。
    """
    _clear(root_row)
    for chip in tree.roots:
        ttk.Radiobutton(root_row, text=chip.label, value=chip.index, variable=root_var,
                        command=lambda index=chip.index: on_root(index)).pack(
                            side=tk.LEFT, padx=(0, 6))
    root_var.set(tree.active_root)
    return render_view_row(view_row, tree, view_var=view_var, on_view=on_view)


def render_view_row(view_row: ttk.Frame, tree: WorkspaceTree, *,
                    view_var: tk.IntVar, on_view) -> bool:
    """只重画第二行。返回它是否有内容，由调用方决定这一行占不占版面。

    单独开这个口子不是为了省几个控件：点第一行会换掉第二行的内容（换了个父），
    而**两行都重画就会销毁刚刚被点的那个单选钮**——Tk 事后再碰它就是
    `invalid command name`。第一行的内容只跟工作区列表有关、跟选中谁无关，
    所以这里不碰它，销毁的永远是"别人那一行"。

    返回值而不是自己 pack：这一行该插在工具条的哪两行之间是页面的事，本模块
    只认自己那个 frame。
    """
    _clear(view_row)
    if not tree.views:
        return False
    ttk.Label(view_row, text=VIEW_ROW_PREFIX, style="Muted.TLabel").pack(
        side=tk.LEFT, padx=(16, 8))
    for chip in tree.views:
        ttk.Radiobutton(view_row, text=chip.label, value=chip.index, variable=view_var,
                        command=lambda index=chip.index: on_view(index)).pack(
                            side=tk.LEFT, padx=(0, 6))
    view_var.set(tree.active)
    return True


__all__ = [
    "PARENT_VIEW_LABEL",
    "VIEW_ROW_PREFIX",
    "WorkspaceChip",
    "WorkspaceTree",
    "render_view_row",
    "render_workspace_bar",
    "workspace_tree",
]
