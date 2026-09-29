"""转速来源抽象。

旧那套把 `kv * voltage * pct/100` 的**估算值**和实测推力并排写进报告
（`tools/pressure_rs485_gui.py:250`），看报告的人分不出哪个是量的、哪个是猜的。
于是每个来源在这里都必须自报 `quality` 和 `measured`，产出侧照抄。

这不是形式主义：转速进的是 `C_T = F / (ρ n² D⁴)`，**平方**关系。
kv 估算里那个"带载系数"随手填 0.80，实际在 0.6~0.9 之间，
于是 C_T 会差出两倍——而两倍的 C_T 足以让一架飞机按图纸算出来能飞、实际飞不起来。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class RpmSource(Protocol):
    """给一档油门（以及可选的电压），回机械转速 [RPM]。"""

    #: 人读的来源说明，写进报告。
    quality: str
    #: True = 实测，False = 推算。产出侧据此决定要不要在数字旁边打 (est)。
    measured: bool

    def rpm(self, percent: float, voltage_v: float | None = None) -> float | None:
        ...


@dataclass
class NoRpm:
    """没有转速。**回 None，不回 0**。

    回 0 的话下游的 `F/(ρn²D⁴)` 会除零或给出 inf，而那个 inf 传下去几步就变成
    一个看起来普通的大数。缺数据就明说缺。
    """

    quality: str = "未接转速回传"
    measured: bool = False

    def rpm(self, percent: float, voltage_v: float | None = None) -> float | None:
        del percent, voltage_v
        return None


@dataclass
class KvEstimate:
    """kv × 电压 × 油门 × 带载系数。**推算值，不是实测**。

    保留它是因为没有双向 DShot 的时候它是唯一能给出数量级的东西，
    但凡是它给出的数，报告里都会带 `(est)`。

    `load_factor` 是"带载之后转速掉到空载的百分之几"。它**不是**一个常数：
    随桨、随电压、随油门都变，这里的默认 0.80 只是一个占位。
    真要用这条链做定量结论，先去装双向 DShot。
    """

    kv_rpm_per_volt: float
    load_factor: float = 0.80
    quality: str = "kv 推算（非实测）"
    measured: bool = False

    def rpm(self, percent: float, voltage_v: float | None = None) -> float | None:
        if voltage_v is None or self.kv_rpm_per_volt <= 0.0:
            return None
        return (self.kv_rpm_per_volt * voltage_v * (percent / 100.0)
                * self.load_factor)


@dataclass
class DshotErpm:
    """双向 DShot 回传的电转速，除以极对数得到机械转速。

    电调回的是**电周期**，换算成 eRPM 之后要除以极对数才是桨的转速
    （见 `Driver/Inc/drv_dshot_telemetry.h`）。极对数填错会让转速整体差一个
    整数倍，而曲线形状完全正常——所以这里要求显式给。
    """

    pole_pairs: int
    #: 由采集回路按油门档填入：{percent: erpm}。
    samples: dict[float, float] = None  # type: ignore[assignment]
    quality: str = "双向 DShot 实测"
    measured: bool = True

    def __post_init__(self) -> None:
        if self.pole_pairs < 1:
            raise ValueError("极对数至少是 1；填错会让转速整体差一个整数倍")
        if self.samples is None:
            self.samples = {}

    def rpm(self, percent: float, voltage_v: float | None = None) -> float | None:
        del voltage_v
        erpm = self.samples.get(percent)
        if erpm is None:
            return None
        return erpm / self.pole_pairs

    def note(self, percent: float, erpm: float) -> None:
        self.samples[percent] = erpm


def annotate(value: float | None, source: RpmSource) -> str:
    """报告里的一格。缺数据写 `--`，推算值带 `(est)`。"""
    if value is None:
        return "--"
    return f"{value:.0f}" if source.measured else f"{value:.0f} (est)"
