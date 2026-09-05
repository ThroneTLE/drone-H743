"""隔离：用户状态、日志、data 输出、时钟，以及“没动过用户数据”的可核验指纹。

四件事分开：

* `isolated_environment()` 把面板会写的每一个路径常量改指到临时根目录。它患的是
  和 `tests/conftest.py` 同一种病的解药，但作用域是**一次调用**而不是整个 session，
  所以离线 QA 脚本（不跑在 pytest 里）也能用同一份实现。
* `claim_output_path()` 排他创建一份输出文件并把句柄交出来。
* `directory_digest()` 给出目录的内容指纹。报告判据里“用户原状态/历史目录内容哈希
  不变”需要一个能真的比出来的东西，而不是一句承诺；`data/calibration/**` 是
  AGENTS.md 明令不许动的历史证据，正是它的第一个使用者。
* `ManualClock` 让节流、超时、新鲜度这些判断可以被逐拍驱动。

2026-09-04 软件审核关掉了这里的两个缺口：

**Q1**：第一版只重指了四个模块里的六个常量，于是机械标定、光流、V0 校准的输出目录
和 RC 向导日志仍然指向真实工作树。手工维护一张常量名单必然漏——`from
..project_paths import X` 会在每个消费模块里各绑一份，改一个不改另一个只会让一半的
写入漏出去。现在改成**按对象 identity 全量扫描**。

**Q5**：`exclusive_path()` 只是"挑一个不存在的名字"，选名与写入之间有窗口，两个
调用者会拿到同一个路径再互相覆盖——和本批在生产 CSV 上刚修掉的 N13 是同一个型。
现在申领即创建，把句柄一起交出去。
"""

from __future__ import annotations

import hashlib
import importlib
import sys
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


# 扫描前先确保这几个模块在场：没被 import 的模块里没有可打的补丁，而面板会在第一次
# 构造时把页面模块全部拉进来。
_ALWAYS_IMPORT = (
    "tools.project_paths",
    "tools.panel_lib.state",
    "tools.drone_tcp_panel",       # 它会把所有页面模块一起带进来
)


# 除了"落在 DATA_ROOT 之下"这条规则，这几个名字**无论当前指向哪里**都要接管。
# 原因：`tests/conftest.py` 会先把它们挪到一个临时目录（那是面板自身的状态，不是
# data 树的一部分），于是相对路径规则就认不出它们了。按名字兜底，两种情形都覆盖。
_ALWAYS_ISOLATED = {
    "PANEL_STATE_PATH": "panel_state.json",
    "LOG_DIR": "logs",
    "PANEL_CRASH_LOG": "logs/panel_crash.log",
    "RC_WIZARD_TRACE_LOG": "logs/rc_wizard.log",
    "TELEMETRY_DIR": "telemetry",
}


def _module_candidates():
    for name, module in list(sys.modules.items()):
        if module is not None and (name == "tools" or name.startswith("tools.")):
            yield module


def _writable_path_constants(data_root: Path) -> dict[int, tuple[Path, Path]]:
    """`{id(原对象): (原对象, 相对 DATA_ROOT 的路径)}`。

    只认大写名字——那是本仓库路径常量的写法。按 identity 索引是关键：同一个 Path
    对象被 N 个模块 import 进去，identity 相同，一次扫描就能把它们全找出来。
    """
    found: dict[int, tuple[Path, Path]] = {}
    for module in _module_candidates():
        for attr, value in list(vars(module).items()):
            if not attr.isupper() or not isinstance(value, Path):
                continue
            try:
                relative = value.relative_to(data_root)
            except ValueError:
                continue
            found.setdefault(id(value), (value, relative))
    return found


@dataclass
class QaEnvironment:
    """一次 QA 运行的全部可写位置。除了这些目录，装置不该产生任何副作用。"""

    root: Path
    panel_state_path: Path
    log_dir: Path
    telemetry_dir: Path
    output_dir: Path

    @classmethod
    def create(cls, root: Path) -> "QaEnvironment":
        root = Path(root)
        env = cls(
            root=root,
            panel_state_path=root / "panel_state.json",
            log_dir=root / "logs",
            telemetry_dir=root / "telemetry",
            output_dir=root / "out",
        )
        for directory in (env.log_dir, env.telemetry_dir, env.output_dir):
            directory.mkdir(parents=True, exist_ok=True)
        return env

    def contains(self, path: Path) -> bool:
        """这个路径是不是落在 QA 根里。验收判据直接用它逐个 owner 核对。"""
        try:
            Path(path).resolve().relative_to(self.root.resolve())
        except ValueError:
            return False
        return True


@contextmanager
def isolated_environment(root: Path) -> Iterator[QaEnvironment]:
    """把 `tools.*` 里**所有**指向 `data/` 的路径常量重指到 `root`，退出时逐个还原。"""
    env = QaEnvironment.create(root)
    for name in _ALWAYS_IMPORT:
        try:
            importlib.import_module(name)
        except ImportError:                      # pragma: no cover - 取决于 sys.path
            continue
    project_paths = sys.modules.get("tools.project_paths")
    if project_paths is None:                    # pragma: no cover - 隔离不了就别假装
        raise RuntimeError("tools.project_paths is not importable; cannot isolate")

    data_root = project_paths.DATA_ROOT
    replacements = {
        original_id: (env.root if str(relative) == "." else env.root / relative)
        for original_id, (_original, relative)
        in _writable_path_constants(data_root).items()
    }
    restore: list[tuple[object, str, object]] = []
    for module in _module_candidates():
        for attr, value in list(vars(module).items()):
            if not attr.isupper() or not isinstance(value, Path):
                continue
            replacement = replacements.get(id(value))
            if replacement is None and attr in _ALWAYS_ISOLATED:
                replacement = env.root.joinpath(*_ALWAYS_ISOLATED[attr].split("/"))
            if replacement is None:
                continue
            replacement.parent.mkdir(parents=True, exist_ok=True)
            restore.append((module, attr, value))
            setattr(module, attr, replacement)
    env.panel_state_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        yield env
    finally:
        for module, attr, original in reversed(restore):
            setattr(module, attr, original)


@contextmanager
def redirected_dated_directory(target: Path) -> Iterator[Path]:
    """把 `dated_directory()` 钉到一个固定目录。

    单独一个入口是因为它不是常量而是函数：录制、导出这些按日期分目录的产物，测试
    里必须落在自己的临时目录，否则第一次跑就会往真实 `data/telemetry/<今天>` 里
    塞文件。
    """
    target = Path(target)
    target.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        for module in _module_candidates():
            if not hasattr(module, "dated_directory"):
                continue
            original = module.dated_directory
            setattr(module, "dated_directory", lambda _root, _t=target: _t)
            stack.callback(setattr, module, "dated_directory", original)
        yield target


def directory_digest(path: Path, *, max_bytes: int | None = None) -> str:
    """目录内容指纹：相对路径 + 大小 + 内容 SHA-256，按路径排序。

    读内容而不是只看 mtime：写回同样长度的坏数据、或者写完再把时间戳改回去，都是
    “历史证据被动过”的真实形态。
    """
    path = Path(path)
    if not path.exists():
        return "missing"
    digest = hashlib.sha256()
    for entry in sorted(p for p in path.rglob("*") if p.is_file()):
        relative = entry.relative_to(path).as_posix()
        data = entry.read_bytes() if max_bytes is None else entry.read_bytes()[:max_bytes]
        digest.update(relative.encode("utf-8"))
        digest.update(str(entry.stat().st_size).encode("ascii"))
        digest.update(hashlib.sha256(data).digest())
    return digest.hexdigest()


def claim_output_path(directory: Path, stem: str, suffix: str):
    """**排他创建**一份输出文件，返回 `(handle, path)`。

    "先看存不存在、再选名字，之后由调用者 write_text" 不是排他：两个调用者会在这个
    窗口里拿到同一个路径然后互相覆盖（审核 Q5）。这个包刚在生产 CSV 上修掉的 N13
    就是同一个型，QA 工具自己犯同样的错说不过去。
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    candidate = directory / f"{stem}{suffix}"
    index = 1
    while True:
        try:
            handle = candidate.open("x", encoding="utf-8")
        except FileExistsError:
            candidate = directory / f"{stem}_{index}{suffix}"
            index += 1
            if index > 10000:                    # pragma: no cover - 目录出问题了
                raise
        else:
            return handle, candidate


class ManualClock:
    """单调时钟替身。只前进，不后退——真实 `time.monotonic()` 的语义就是这样。"""

    def __init__(self, start: float = 1000.0) -> None:
        self._now = float(start)

    def monotonic(self) -> float:
        return self._now

    __call__ = monotonic

    def advance(self, seconds: float) -> float:
        if seconds < 0:
            raise ValueError("monotonic clock cannot go backwards")
        self._now += float(seconds)
        return self._now


__all__ = [
    "ManualClock",
    "QaEnvironment",
    "claim_output_path",
    "directory_digest",
    "isolated_environment",
    "redirected_dated_directory",
]
