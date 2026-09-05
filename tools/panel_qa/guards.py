"""全局硬件护栏：物理串口 + 烧录/复位程序。

`tests/conftest.py` 已经挡住了 `serial.Serial.open`。那只挡住一条路：AGENTS.md
第 5 条说的是**未经 REQ 明文授权不得烧录、复位、发送任何目标板命令**，而
`subprocess` 起一个 openocd 或 STM32_Programmer_CLI 完全绕开串口层。改版报告的判据
写得很直白——“物理打开/烧录尝试应直接使装置测试失败”，所以两条路都要挡，而且
必须**当场抛异常**：只记一笔然后放行，等于让一次真实烧录发生了再来事后统计。

护栏是可嵌套的：`install()` 保存上一层的实现并在 `uninstall()` 时原样放回，所以
在 conftest 已经打过补丁的进程里再开一层不会把原补丁弄丢。
"""

from __future__ import annotations

import subprocess
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Iterator


# 命令名里出现这些片段就当成烧录/调试探针工具。宁可误伤：QA 装置本来就不该起
# 外部进程，误伤的代价是加一行白名单，漏网的代价是真的动了板子。
FLASH_TOOL_TOKENS = (
    "openocd",
    "st-flash",
    "st-util",
    "st-info",
    "stlink",
    "stm32_programmer",
    "stm32flash",
    "stm32cubeprogrammer",
    "jlink",
    "pyocd",
    "dfu-util",
    "dfu-suffix",
)


@dataclass(frozen=True)
class HardwareAccessAttempt:
    """一次被拦下的硬件访问。`detail` 保留原始参数，便于事后定位是谁发起的。"""

    kind: str           # "serial" | "flash_tool"
    detail: str


@dataclass
class HardwareGuardLog:
    attempts: list[HardwareAccessAttempt] = field(default_factory=list)

    @property
    def serial_opens(self) -> list[str]:
        return [a.detail for a in self.attempts if a.kind == "serial"]

    @property
    def flash_invocations(self) -> list[str]:
        return [a.detail for a in self.attempts if a.kind == "flash_tool"]

    @property
    def clean(self) -> bool:
        return not self.attempts

    def summary(self) -> str:
        return (
            f"physical serial opens={len(self.serial_opens)} "
            f"flash tool invocations={len(self.flash_invocations)}"
        )


def _command_tokens(args) -> list[str]:
    if isinstance(args, (str, bytes)):
        raw = [args]
    else:
        try:
            raw = list(args)
        except TypeError:                       # pragma: no cover - 非法调用交给 subprocess 报错
            return []
    tokens = []
    for item in raw:
        if isinstance(item, bytes):
            tokens.append(item.decode("utf-8", "replace"))
        else:
            tokens.append(str(item))
    return tokens


def is_flash_tool_command(args) -> str | None:
    """命中就返回触发的那个 token，用于错误信息；没命中返回 None。

    只看**可执行文件名**（argv[0] 的 basename），不看后面的参数：
    `python analyse.py --openocd-log x.txt` 不是一次烧录。
    """
    tokens = _command_tokens(args)
    if not tokens:
        return None
    executable = tokens[0].replace("\\", "/").rsplit("/", 1)[-1].lower()
    for token in FLASH_TOOL_TOKENS:
        if token in executable:
            return token
    return None


class _GuardInstallation:
    def __init__(self, log: HardwareGuardLog) -> None:
        self.log = log
        self._serial_original = None
        self._popen_original = None

    def install(self) -> None:
        import serial

        self._serial_original = serial.Serial.open
        self._popen_original = subprocess.Popen.__init__
        log = self.log

        def blocked_open(instance, *args, **kwargs):
            port = str(getattr(instance, "port", None) or (args[0] if args else None))
            log.attempts.append(HardwareAccessAttempt("serial", port))
            raise AssertionError(
                f"QA harness must not open a physical serial port (attempted {port!r})"
            )

        def blocked_popen(instance, *args, **kwargs):
            # `args` 可以是位置参数也可以是关键字参数（`subprocess.run` 走位置，
            # 但库里到处都有 `Popen(args=...)` 的写法）。两种都要看到。
            command = args[0] if args else kwargs.get("args")
            token = is_flash_tool_command(command)
            if token is not None:
                detail = " ".join(_command_tokens(command))
                log.attempts.append(HardwareAccessAttempt("flash_tool", detail))
                raise AssertionError(
                    "QA harness must not run a flashing/reset tool "
                    f"(matched {token!r} in {detail!r})"
                )
            return self._popen_original(instance, *args, **kwargs)

        serial.Serial.open = blocked_open
        subprocess.Popen.__init__ = blocked_popen

    def uninstall(self) -> None:
        import serial

        if self._serial_original is not None:
            serial.Serial.open = self._serial_original
        if self._popen_original is not None:
            subprocess.Popen.__init__ = self._popen_original


@contextmanager
def hardware_guards(log: HardwareGuardLog | None = None) -> Iterator[HardwareGuardLog]:
    """在作用域内拦截物理串口与烧录工具，退出时把上一层实现原样放回。"""
    guard_log = HardwareGuardLog() if log is None else log
    installation = _GuardInstallation(guard_log)
    installation.install()
    try:
        yield guard_log
    finally:
        installation.uninstall()


__all__ = [
    "FLASH_TOOL_TOKENS",
    "HardwareAccessAttempt",
    "HardwareGuardLog",
    "hardware_guards",
    "is_flash_tool_command",
]
