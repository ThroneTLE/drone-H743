"""电机驱动通道抽象：经飞控串口，或直连 ESP 板。

两条路都保留（旧那套支持的就是这两条），但它们的**安全语义**统一在这里：

* 任何一条路都必须能**无条件停机**，而且停机不依赖上一条命令是否成功；
* 下发之后要能确认对方收到，收不到确认就当成没发出去——推力台上的
  "以为停了其实没停"是会伤人的。

`percent_to_pulse` 与固件 `DRV_Motor_PercentToPulse` 同一条换算，不另写一份。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

# Flight-controller equivalent PWM units come from BSP_PWM_PercentToPulse.
# Physical-disable 0 is a separate state and must never be inverted as a percent.
FC_ESC_MIN_US = 1100
FC_ESC_MAX_US = 1940


def percent_to_pulse(percent: float) -> int:
    """0..100% -> the FC's 1100..1940 equivalent-us integer mapping."""
    clamped = max(0.0, min(100.0, percent))
    # C integer division truncates; preserve it for integral command percentages.
    return FC_ESC_MIN_US + int((FC_ESC_MAX_US - FC_ESC_MIN_US) * clamped) // 100


class MotorDriver(Protocol):
    name: str

    def set_percent(self, motor: int, percent: float) -> bool:
        ...

    def stop_all(self) -> bool:
        ...


#: 与固件 `PROPCAL_SPIN_CONFIRM_TOKEN`（App/Src/app_cmd_propcal.c:37）一致。
SPIN_CONFIRM_TOKEN = "safe"
#: 心跳周期 [s]。固件那边 300 ms 收不到心跳就关窗，所以本地要发得更勤。
SPIN_HEARTBEAT_S = 0.2


@dataclass
class FlightControllerDriver:
    """经飞控维护串口下发，走 `PROPCAL SPIN` 那条**受心跳保护**的窗口。

    这是全仓库唯一一条能从上位机让桨转起来的路径，而且它的保护正是推力台需要
    的：窗口靠心跳维持，上位机崩了、线掉了、程序被 Ctrl-C 了，飞控自己会在
    300 ms 内关窗断电。裸的电机命令没有这个性质——发出去之后飞控会一直保持，
    哪怕对面的程序已经不在了。

    `set_percent` 每次调用本身就兼作一次心跳（重发当前油门），
    所以扫描回路只要按不低于心跳周期的节奏调它就行。
    """

    link: object
    name: str = "飞控串口"
    heartbeat_period_s: float = SPIN_HEARTBEAT_S

    def arm(self) -> bool:
        """开窗。开窗那一刻油门必须是 0——开窗本身不该让任何东西转起来。"""
        return bool(self.link.send_line(
            f"PROPCAL SPIN ARM confirm={SPIN_CONFIRM_TOKEN}"))

    def set_percent(self, motor: int, percent: float) -> bool:
        return bool(self.link.send_line(
            f"PROPCAL SPIN SET ch={int(motor)} pct={int(round(percent))}"))

    def heartbeat(self, motor: int, percent: float) -> bool:
        """重发当前油门。发不出去就该本地停——两边说的是同一件事。"""
        return self.set_percent(motor, percent)

    def stop_all(self) -> bool:
        """关窗。发两遍。

        第一遍可能撞上链路的一次丢包，而这条命令是整套台架里唯一一条
        "发不出去就会出事"的。重发的代价是零。
        （飞控那边还有心跳超时兜底，但兜底不是不发的理由。）
        """
        first = bool(self.link.send_line("PROPCAL SPIN STOP"))
        second = bool(self.link.send_line("PROPCAL SPIN STOP"))
        return first or second


@dataclass
class EspDriver:
    """直连 ESP 驱动板（旧台架那条路）。"""

    link: object
    name: str = "ESP 直连"

    def set_percent(self, motor: int, percent: float) -> bool:
        return bool(self.link.send_line(f"M{int(motor)} {int(round(percent))}"))

    def stop_all(self) -> bool:
        first = bool(self.link.send_line("M1 0"))
        second = bool(self.link.send_line("M2 0"))
        return first and second


class SafeRun:
    """`with SafeRun(driver):` —— 退出时无条件停机。

    包括异常退出、包括 Ctrl-C。旧那套的停机写在正常流程的末尾，
    中途抛异常就停不了——而中途抛异常正是最需要停机的时候。
    """

    def __init__(self, driver: MotorDriver) -> None:
        self.driver = driver
        self.stopped = False

    def __enter__(self) -> MotorDriver:
        return self.driver

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.stopped = self.driver.stop_all()
        return False  # 不吞异常


def settle(seconds: float, sleep=time.sleep) -> None:
    """等桨转速稳下来。`sleep` 可注入，便于离线测试。"""
    if seconds > 0.0:
        sleep(seconds)
