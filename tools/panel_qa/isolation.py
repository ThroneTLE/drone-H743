"""隔离：用户状态、日志、data 输出、时钟，以及“没动过用户数据”的可核验指纹。

三件事分开：

* `isolated_environment()` 把面板会写的每一个路径常量改指到临时根目录。它患的是
  和 `tests/conftest.py` 同一种病的解药，但作用域是**一次调用**而不是整个 session，
  所以离线 QA 脚本（不跑在 pytest 里）也能用同一份实现。
* `directory_digest()` 给出目录的内容指纹。报告判据里“用户原状态/历史目录内容哈希
  不变”需要一个能真的比出来的东西，而不是一句承诺；`data/calibration/**` 是
  AGENTS.md 明令不许动的历史证据，正是它的第一个使用者。
* `ManualClock` 让节流、超时、新鲜度这些判断可以被逐拍驱动。用 `time.sleep` 等真实
  时间的测试要么慢要么飘，而 `record_service` 这类东西必须能精确复现“队列还没写完
  就切了 schema”。
"""

from __future__ import annotations

import hashlib
import importlib
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


# 面板落盘用到的路径常量。同一个名字在多个模块里各 import 一份，改一个不改另一个
# 就会有一半的写入漏到用户真实目录，所以按名字全量覆盖。
_PATH_CONSTANTS = (
    "PANEL_STATE_PATH",
    "LOG_DIR",
    "PANEL_CRASH_LOG",
    "RC_WIZARD_TRACE_LOG",
    "TELEMETRY_DIR",
    "DATA_ROOT",
)

_PATCH_TARGETS = (
    "tools.project_paths",
    "tools.panel_lib.state",
    "tools.drone_tcp_panel",
    "tools.panel_lib.pages.dashboard",
)


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


@contextmanager
def isolated_environment(root: Path) -> Iterator[QaEnvironment]:
    """把面板的落盘路径常量重指到 `root`，退出时逐个还原。"""
    env = QaEnvironment.create(root)
    values = {
        "PANEL_STATE_PATH": env.panel_state_path,
        "LOG_DIR": env.log_dir,
        "PANEL_CRASH_LOG": env.log_dir / "panel_crash.log",
        "RC_WIZARD_TRACE_LOG": env.log_dir / "rc_wizard.log",
        "TELEMETRY_DIR": env.telemetry_dir,
        "DATA_ROOT": env.root,
    }
    restore: list[tuple[object, str, object]] = []
    for module_name in _PATCH_TARGETS:
        try:
            module = importlib.import_module(module_name)
        except ImportError:                      # pragma: no cover - 取决于调用方 sys.path
            continue
        for name in _PATH_CONSTANTS:
            if not hasattr(module, name):
                continue
            restore.append((module, name, getattr(module, name)))
            setattr(module, name, values[name])
    try:
        yield env
    finally:
        for module, name, original in reversed(restore):
            setattr(module, name, original)


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
        for module_name in _PATCH_TARGETS:
            try:
                module = importlib.import_module(module_name)
            except ImportError:                  # pragma: no cover
                continue
            if not hasattr(module, "dated_directory"):
                continue
            original = module.dated_directory
            setattr(module, "dated_directory", lambda _root, _t=target: _t)
            stack.callback(setattr, module, "dated_directory", original)
        yield target


def directory_digest(path: Path, *, max_bytes: int | None = None) -> str:
    """目录内容指纹：相对路径 + 大小 + 内容 SHA-256，按路径排序。

    读内容而不是只看 mtime：写回同样长度的坏数据、或者写完再把时间戳改回去，都是
    “历史证据被动过”的真实形态。`max_bytes` 只在需要给巨大目录做快速筛查时使用，
    默认全读。
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


def exclusive_path(directory: Path, stem: str, suffix: str) -> Path:
    """排他创建一个不碰撞的输出文件路径。

    观测脚本存在的全部理由就是保住“修复前”的那一份证据，用 `"w"` 把它盖掉就太
    讽刺了。这里刻意不 import 生产侧的同类实现：QA 工具不能依赖某个还没写出来的
    生产模块，反过来生产代码也不该依赖 QA 工具。
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    candidate = directory / f"{stem}{suffix}"
    index = 1
    while candidate.exists():
        candidate = directory / f"{stem}_{index}{suffix}"
        index += 1
    return candidate


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
    "directory_digest",
    "exclusive_path",
    "isolated_environment",
    "redirected_dated_directory",
]
