"""日志接收范围（只导最近一次 / 全部）的持久化，沿用地面站的 data/panel_state.json。

默认只导最近一次：全量导出要把整个环形区（约 4 MB）读完，而一次台架试验通常只占很小一部分。
选项存在 panel_state.json 的 `flight_log_export_all` 里；面板自己也持有一份内存里的状态并会整份写回，
所以保存时要同步更新 `panel._panel_state`，否则下一次面板保存连接状态就把这个键冲掉了。
"""
from __future__ import annotations

import json

try:
    from .. import project_paths
except ImportError:  # 直接运行 / 旧式导入
    import project_paths  # type: ignore

STATE_KEY = "flight_log_export_all"


def _read_state() -> dict:
    try:
        raw = json.loads(project_paths.PANEL_STATE_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return raw if isinstance(raw, dict) else {}


def load_export_all(panel=None) -> bool:
    """True = 导出全部；缺省或读不出来 = False（只导最近一次）。"""
    memory = getattr(panel, "_panel_state", None)
    if isinstance(memory, dict) and STATE_KEY in memory:
        return memory[STATE_KEY] is True
    return _read_state().get(STATE_KEY) is True


def save_export_all(value: bool, panel=None) -> bool:
    """写进面板状态文件；写失败不影响导出，返回是否写成。"""
    value = bool(value)
    memory = getattr(panel, "_panel_state", None)
    if isinstance(memory, dict):
        memory[STATE_KEY] = value
    state = _read_state()
    state[STATE_KEY] = value
    try:
        project_paths.ensure_directory(project_paths.PANEL_STATE_PATH.parent)
        project_paths.PANEL_STATE_PATH.write_text(
            json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    except OSError:
        return False
    return True
