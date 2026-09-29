"""板载文本行的观察者注册表。

**为什么需要它。** 飞控回来的每一行文本都流经 `drone_tcp_panel.py` 的
`_handle_board_line`，而那是一条写死的 if/elif 链，没有注册钩子——偏偏那个文件
是**只减不增**的。以前每来一个新命令族就往里加一行转发（ARM 就是这么加的），
等于给一个已经超限的文件开了个慢性口子。

有了这张表，`panel_lib` 里的任何页面都能自己来取整行文本，那个文件永远不用再改。

**为什么挂在 panel 实例上而不是模块级全局。** 测试里会同时存在多个
`DronePanel()`；模块级的表会让上一个面板的回调收到下一个面板的行，症状是
"测试单跑通过、全量跑就串台"。
"""

from __future__ import annotations

_ATTRIBUTE = "_board_line_hooks"


def register_board_line_hook(panel, hook) -> None:
    """`hook(line)` 会收到飞控回来的**每一行**原始文本，包括 OK/ERR 之外的。

    重复注册同一个可调用对象只会记一次：页面可能因为重新挂载而再走一次注册，
    注册两次的表现是每行被处理两遍——而对"收到回显才算确认"这类逻辑，
    处理两遍不会报错，只会让计数悄悄翻倍。
    """
    hooks = getattr(panel, _ATTRIBUTE, None)
    if hooks is None:
        hooks = []
        setattr(panel, _ATTRIBUTE, hooks)
    if hook not in hooks:
        hooks.append(hook)


def unregister_board_line_hook(panel, hook) -> None:
    hooks = getattr(panel, _ATTRIBUTE, None)
    if hooks and (hook in hooks):
        hooks.remove(hook)


def dispatch_board_line(panel, line: str) -> None:
    """把一行文本发给所有钩子。

    单个钩子抛异常不能带走整条接收链路——那会让飞控的回包从某一刻起全部丢失，
    而用户看到的只是"面板不刷新了"。谁抛的异常记在面板的日志里，不往上传。
    """
    hooks = getattr(panel, _ATTRIBUTE, None)
    if not hooks:
        return
    for hook in list(hooks):
        try:
            hook(line)
        except Exception as exc:          # pragma: no cover - 防御性
            logger = getattr(panel, "_log", None)
            if callable(logger):
                logger(f"board line hook failed: {exc}")


__all__ = [
    "dispatch_board_line",
    "register_board_line_hook",
    "unregister_board_line_hook",
]
