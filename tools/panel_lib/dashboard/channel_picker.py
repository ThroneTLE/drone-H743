"""遥测通道下拉选择器。

工作台的卡片都按通道名保存绑定；这个控件把 :class:`TelemSchema` 当前收到的
通道表变成一个可单选或多选的 ``ttk.Menubutton``。它不认识任何 tile，因而波形、
数值卡和以后新增的显示组件可以共用同一套选择、限额和回调语义。
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
import tkinter as tk
from tkinter import ttk
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..telem_stream import TelemChannel, TelemSchema


BindingCallback = Callable[[tuple[str, ...]], None]


class ChannelPicker(ttk.Menubutton):
    """显示并编辑一组遥测通道名。

    ``set_schema()`` 接收当前 :class:`TelemSchema`，``set_channels(channels,
    selected)`` 或 ``set_bindings()`` 用已有布局的绑定初始化或回写选择。选择顺序
    保持调用者给出的顺序，新勾选的通道追加在末尾；这样波形颜色不会因打开菜单而
    悄悄重排。``on_change`` 仅在选择真的改变时调用，参数是不可变的通道名元组。

    ``min_selected`` / ``max_selected`` 约束的是绑定数量。菜单在达到上限时禁用
    未选项，在达到下限时禁用已选项，因此用户不会得到一个短暂的非法选择状态。
    尚未出现在新 schema 里的旧绑定会保留，避免换表或分页装配期间丢掉布局。
    """

    def __init__(
        self,
        parent: tk.Misc,
        *,
        schema: TelemSchema | None = None,
        channels: Iterable[TelemChannel] | Mapping[object, TelemChannel] | None = None,
        bindings: Iterable[str] = (),
        multiple: bool = True,
        min_selected: int = 0,
        max_selected: int | None = None,
        empty_text: str = "选择通道",
        on_change: BindingCallback | None = None,
        **kwargs,
    ) -> None:
        if min_selected < 0:
            raise ValueError("min_selected 不能小于 0")
        if not multiple:
            if max_selected is not None and max_selected > 1:
                raise ValueError("单选模式的 max_selected 不能大于 1")
            max_selected = 1
        if max_selected is not None and max_selected < min_selected:
            raise ValueError("max_selected 不能小于 min_selected")

        self._summary_var = tk.StringVar(master=parent)
        super().__init__(parent, textvariable=self._summary_var, **kwargs)
        self.menu = tk.Menu(self, tearoff=False)
        self.configure(menu=self.menu)

        self.multiple = multiple
        self.min_selected = min_selected
        self.max_selected = max_selected
        self.empty_text = empty_text
        self.on_change = on_change

        self._channels_by_name: dict[str, TelemChannel] = {}
        self._selection: list[str] = []
        self._check_vars: dict[str, tk.BooleanVar] = {}
        self._menu_indices: dict[str, int] = {}

        # 先同步持久化布局，再装表；这样 schema 尚未完整时也不会丢掉已有绑定。
        self.set_bindings(bindings, notify=False)
        if schema is not None:
            self.set_schema(schema)
        elif channels is not None:
            self.set_channels(channels)
        else:
            self._refresh_summary()

    # ------------------------------------------------------------- 公共 API

    @property
    def bindings(self) -> tuple[str, ...]:
        """当前选择的通道名，供布局模型直接持久化。"""
        return tuple(self._selection)

    @property
    def selected(self) -> tuple[str, ...]:
        """当前选择的只读快照；``bindings`` 是为布局语义保留的同义 API。"""
        return self.bindings

    def set_schema(self, schema: TelemSchema | None) -> None:
        """按当前 ``TelemSchema`` 重建菜单，不改已有绑定。"""
        if schema is None:
            self.set_channels(())
        else:
            self.set_channels(schema.ordered())

    def set_channels(
        self,
        channels: Iterable[TelemChannel] | Mapping[object, TelemChannel],
        selected: Iterable[str] | None = None,
    ) -> None:
        """用通道迭代器或 ``schema.channels`` 字典更新菜单。

        通道按 ``index`` 升序显示；同名项只保留最早的一项，避免一个布局名对应
        两个菜单项的歧义。若给出 ``selected``，它先按已有布局同步选择；装配和
        菜单重建本身不触发 ``on_change``。
        """
        if selected is not None:
            self.set_bindings(selected, notify=False)
        values = channels.values() if isinstance(channels, Mapping) else channels
        indexed = list(enumerate(values))
        indexed.sort(key=lambda pair: self._channel_sort_key(pair[0], pair[1]))

        self._channels_by_name = {}
        for _position, channel in indexed:
            name = str(getattr(channel, "name", "")).strip()
            if name and name not in self._channels_by_name:
                self._channels_by_name[name] = channel
        self._rebuild_menu()

    def set_bindings(self, bindings: Iterable[str], *, notify: bool = False) -> None:
        """按已有布局同步选择；默认不把加载布局当作一次用户修改。"""
        normalized: list[str] = []
        for value in bindings:
            name = str(value).strip()
            if name and name not in normalized:
                normalized.append(name)
            if self.max_selected is not None and len(normalized) >= self.max_selected:
                break
        self._replace_selection(normalized, notify=notify)

    def choose(self, names: list[str]) -> bool:
        """无鼠标地应用一次选择；不满足通道或数量约束时不改当前状态。

        这给真实 ``DronePanel()`` 端到端测试和外层快捷操作一个与菜单点击完全
        相同的入口。``True`` 表示选择已接受，``False`` 表示输入未触及控件。
        """
        normalized: list[str] = []
        for value in names:
            name = str(value).strip()
            if not name or name not in self._channels_by_name:
                return False
            if name not in normalized:
                normalized.append(name)
        count = len(normalized)
        if count < self.min_selected:
            return False
        if self.max_selected is not None and count > self.max_selected:
            return False
        self._replace_selection(normalized, notify=True)
        return True

    # ------------------------------------------------------------- 菜单实现

    @staticmethod
    def _channel_sort_key(position: int, channel: TelemChannel) -> tuple[int, int]:
        try:
            return int(getattr(channel, "index")), position
        except (TypeError, ValueError):
            return position, position

    @staticmethod
    def _channel_label(channel: TelemChannel) -> str:
        name = str(getattr(channel, "name", ""))
        unit = str(getattr(channel, "unit", "")).strip()
        return f"{name}  [{unit}]" if unit and unit != "-" else name

    def _rebuild_menu(self) -> None:
        self.menu.delete(0, tk.END)
        self._check_vars = {}
        self._menu_indices = {}
        for name, channel in self._channels_by_name.items():
            variable = tk.BooleanVar(master=self, value=name in self._selection)
            self.menu.add_checkbutton(
                label=self._channel_label(channel),
                variable=variable,
                onvalue=True,
                offvalue=False,
                command=lambda chosen=name: self._on_toggled(chosen),
            )
            self._check_vars[name] = variable
            index = self.menu.index(tk.END)
            if index is not None:
                self._menu_indices[name] = int(index)
        self._refresh_summary()
        self._refresh_menu_states()

    def _on_toggled(self, name: str) -> None:
        variable = self._check_vars[name]
        selected = bool(variable.get())
        if selected:
            if self.max_selected is not None and len(self._selection) >= self.max_selected:
                variable.set(False)
                return
            self._replace_selection([*self._selection, name], notify=True)
            return

        if len(self._selection) <= self.min_selected:
            variable.set(True)
            return
        self._replace_selection(
            [chosen for chosen in self._selection if chosen != name], notify=True
        )

    def _replace_selection(self, selection: list[str], *, notify: bool) -> None:
        changed = selection != self._selection
        self._selection = selection
        for name, variable in self._check_vars.items():
            variable.set(name in self._selection)
        self._refresh_summary()
        self._refresh_menu_states()
        if changed and notify and self.on_change is not None:
            self.on_change(self.bindings)

    def _refresh_menu_states(self) -> None:
        may_add = self.max_selected is None or len(self._selection) < self.max_selected
        may_remove = len(self._selection) > self.min_selected
        for name, index in self._menu_indices.items():
            allowed = may_remove if name in self._selection else may_add
            self.menu.entryconfigure(index, state=tk.NORMAL if allowed else tk.DISABLED)

    def _refresh_summary(self) -> None:
        if not self._selection:
            self._summary_var.set(self.empty_text)
            return

        labels = [self._summary_label(name) for name in self._selection]
        if len(labels) <= 2:
            self._summary_var.set(", ".join(labels))
        else:
            self._summary_var.set(f"{labels[0]}, {labels[1]} +{len(labels) - 2}")

    def _summary_label(self, name: str) -> str:
        channel = self._channels_by_name.get(name)
        if channel is None:
            return f"{name}（缺失）"
        unit = str(getattr(channel, "unit", "")).strip()
        return f"{name} [{unit}]" if unit and unit != "-" else name
