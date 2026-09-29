"""激励剖面的主机镜像 —— 与 `Driver/Src/drv_sysid_excitation.c` 逐样本一致。

存在的理由只有两个，都很具体：

* 上位机要能在**不连飞控**的情况下画出"这条命令将要发出去的波形"；
* `tests/test_sysid_excitation_parity.py` 拿它和固件逐样本对拍——两边算出不同的
  东西时，实机上表现为"图上画的和飞机做的不是一回事"，而那种偏差只会被当成模型
  误差记进辨识结果里。

所以本文件**不许**做任何"更漂亮"的改进（换个更平滑的斜坡、用更精确的相位）。
要改先改固件，再同步这里，再让对拍测试说话。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

PROFILE_STEP = 0
PROFILE_DOUBLET = 1
PROFILE_CHIRP = 2
PROFILE_PRBS = 3
PROFILE_COUNT = 4

PROFILE_NAMES = {
    PROFILE_STEP: "step",
    PROFILE_DOUBLET: "doublet",
    PROFILE_CHIRP: "chirp",
    PROFILE_PRBS: "prbs",
}
PROFILE_CODES = {name: code for code, name in PROFILE_NAMES.items()}

MAX_RATE_RAD_S = 5.0
MAX_DURATION_MS = 30000
MIN_RAMP_MS = 5
MAX_REPEAT = 20

TWO_PI = 6.2831853071795864769


class ExcitationInvalid(ValueError):
    """参数没通过体检。固件会原样拒绝，这里也拒绝——不裁剪。

    裁剪会让人以为发出去的就是填的值。
    """


@dataclass(frozen=True)
class Excitation:
    profile: int = PROFILE_DOUBLET
    amplitude_rad_s: float = 1.0
    duration_ms: int = 4000
    hold_ms: int = 250
    repeat: int = 4
    ramp_ms: int = 20
    chirp_f0_hz: float = 0.3
    chirp_f1_hz: float = 6.0
    prbs_bit_ms: int = 40
    prbs_seed: int = 1

    @property
    def profile_name(self) -> str:
        return PROFILE_NAMES.get(self.profile, "?")

    def validate(self) -> None:
        if not 0 <= self.profile < PROFILE_COUNT:
            raise ExcitationInvalid(f"未知剖面 {self.profile}")
        if not math.isfinite(self.amplitude_rad_s) or self.amplitude_rad_s <= 0.0:
            raise ExcitationInvalid("幅值必须为正的有限值")
        if self.amplitude_rad_s > MAX_RATE_RAD_S:
            raise ExcitationInvalid(f"幅值 {self.amplitude_rad_s} 超过上限 {MAX_RATE_RAD_S} rad/s")
        if self.ramp_ms < MIN_RAMP_MS:
            # ramp=0 的方波导数是冲激，而冲激力矩做不出来：前馈会要求无穷大倾角。
            raise ExcitationInvalid(f"斜坡不得短于 {MIN_RAMP_MS} ms")
        if self.duration_ms == 0 or self.duration_ms > MAX_DURATION_MS:
            raise ExcitationInvalid("总时长越界")

        if self.profile == PROFILE_STEP:
            if self.hold_ms < self.ramp_ms:
                raise ExcitationInvalid("平台短于斜坡，根本到不了幅值")
        elif self.profile == PROFILE_DOUBLET:
            if self.hold_ms < self.ramp_ms:
                raise ExcitationInvalid("半周期短于斜坡，根本到不了幅值")
            if not 1 <= self.repeat <= MAX_REPEAT:
                raise ExcitationInvalid(f"repeat 必须在 1..{MAX_REPEAT}")
        elif self.profile == PROFILE_CHIRP:
            for value in (self.chirp_f0_hz, self.chirp_f1_hz):
                if not math.isfinite(value) or value <= 0.0:
                    raise ExcitationInvalid("扫频频率必须为正")
            if self.chirp_f1_hz <= self.chirp_f0_hz:
                raise ExcitationInvalid("终止频率必须高于起始频率")
            if self.chirp_f1_hz > 100.0:
                raise ExcitationInvalid("终止频率上限 100 Hz")
        elif self.profile == PROFILE_PRBS:
            if self.prbs_bit_ms == 0 or self.prbs_bit_ms < self.ramp_ms:
                raise ExcitationInvalid("PRBS 每位时长不得短于斜坡")

    def total_ms(self) -> int:
        self.validate()
        if self.profile == PROFILE_STEP:
            total = self.ramp_ms + self.hold_ms + self.ramp_ms
        elif self.profile == PROFILE_DOUBLET:
            total = self.hold_ms * 2 * self.repeat
        else:
            total = self.duration_ms
        return min(total, self.duration_ms)

    def eval(self, t_ms: int) -> tuple[float, float, bool]:
        """时间的纯函数。返回 (ω_sp [rad/s], α_ff [rad/s²], finished)。"""
        total = self.total_ms()
        if t_ms >= total:
            return 0.0, 0.0, True

        if self.profile == PROFILE_STEP:
            ramp = self.ramp_ms
            if t_ms < ramp:
                return (*_ramped(0.0, self.amplitude_rad_s, t_ms, ramp), False)
            if t_ms < ramp + self.hold_ms:
                return self.amplitude_rad_s, 0.0, False
            return (*_ramped(self.amplitude_rad_s, 0.0,
                             t_ms - (ramp + self.hold_ms), ramp), False)

        if self.profile == PROFILE_DOUBLET:
            half = self.hold_ms
            segment = t_ms // half
            since_edge = t_ms - segment * half
            current = self.amplitude_rad_s if segment % 2 == 0 else -self.amplitude_rad_s
            if segment == 0:
                previous = 0.0
            else:
                previous = (self.amplitude_rad_s if (segment - 1) % 2 == 0
                            else -self.amplitude_rad_s)
            return (*_ramped(previous, current, since_edge, self.ramp_ms), False)

        if self.profile == PROFILE_CHIRP:
            seconds = t_ms * 0.001
            span = total * 0.001
            sweep = (self.chirp_f1_hz - self.chirp_f0_hz) / span
            phase = TWO_PI * (self.chirp_f0_hz * seconds
                              + 0.5 * sweep * seconds * seconds)
            frequency = self.chirp_f0_hz + sweep * seconds
            # 解析导数，不是差分：差分在高频段引入相位误差，而相位正是要辨的东西。
            return (self.amplitude_rad_s * math.sin(phase),
                    self.amplitude_rad_s * math.cos(phase) * TWO_PI * frequency,
                    False)

        bit_ms = self.prbs_bit_ms
        index = t_ms // bit_ms
        since_edge = t_ms - index * bit_ms
        current = self.amplitude_rad_s if prbs_bit(self.prbs_seed, index) else -self.amplitude_rad_s
        if index == 0:
            previous = 0.0
        else:
            previous = (self.amplitude_rad_s if prbs_bit(self.prbs_seed, index - 1)
                        else -self.amplitude_rad_s)
        return (*_ramped(previous, current, since_edge, self.ramp_ms), False)

    def preview(self, step_ms: int = 2) -> tuple[list[float], list[float], list[float]]:
        """整条剖面的采样，用于界面预览。返回 (t[s], ω_sp, α_ff)。"""
        total = self.total_ms()
        t_s, omega, alpha = [], [], []
        for t_ms in range(0, total, step_ms):
            w, a, _ = self.eval(t_ms)
            t_s.append(t_ms * 0.001)
            omega.append(w)
            alpha.append(a)
        return t_s, omega, alpha

    def to_dict(self) -> dict:
        return {
            "profile": self.profile_name,
            "amplitude_rad_s": self.amplitude_rad_s,
            "duration_ms": self.duration_ms,
            "hold_ms": self.hold_ms,
            "repeat": self.repeat,
            "ramp_ms": self.ramp_ms,
            "chirp_f0_hz": self.chirp_f0_hz,
            "chirp_f1_hz": self.chirp_f1_hz,
            "prbs_bit_ms": self.prbs_bit_ms,
            "prbs_seed": self.prbs_seed,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Excitation":
        profile = data.get("profile", PROFILE_DOUBLET)
        if isinstance(profile, str):
            profile = PROFILE_CODES[profile]
        defaults = cls()
        return cls(
            profile=int(profile),
            amplitude_rad_s=float(data.get("amplitude_rad_s", defaults.amplitude_rad_s)),
            duration_ms=int(data.get("duration_ms", defaults.duration_ms)),
            hold_ms=int(data.get("hold_ms", defaults.hold_ms)),
            repeat=int(data.get("repeat", defaults.repeat)),
            ramp_ms=int(data.get("ramp_ms", defaults.ramp_ms)),
            chirp_f0_hz=float(data.get("chirp_f0_hz", defaults.chirp_f0_hz)),
            chirp_f1_hz=float(data.get("chirp_f1_hz", defaults.chirp_f1_hz)),
            prbs_bit_ms=int(data.get("prbs_bit_ms", defaults.prbs_bit_ms)),
            prbs_seed=int(data.get("prbs_seed", defaults.prbs_seed)),
        )

    def command(self) -> str:
        """生成对应的固件命令行。界面发出去的就是这一条。"""
        return (f"SYSID EXC profile={self.profile_name} "
                f"amp={self.amplitude_rad_s:g} dur_ms={self.duration_ms} "
                f"hold_ms={self.hold_ms} repeat={self.repeat} "
                f"ramp_ms={self.ramp_ms} f0={self.chirp_f0_hz:g} "
                f"f1={self.chirp_f1_hz:g} bit_ms={self.prbs_bit_ms} "
                f"seed={self.prbs_seed}")


def _ramped(previous: float, current: float, since_edge_ms: int,
            ramp_ms: int) -> tuple[float, float]:
    if ramp_ms == 0 or since_edge_ms >= ramp_ms:
        return current, 0.0
    fraction = since_edge_ms / ramp_ms
    return (previous + (current - previous) * fraction,
            (current - previous) / (ramp_ms * 0.001))


def prbs_bit(seed: int, index: int) -> int:
    """31 位 LFSR（x^31 + x^28 + 1）的第 index 位。

    两处必须和固件一模一样：

    1. **种子先乘 Knuth 常数散开**。1、7 这种小种子高 30 位全是 0，直接跑会有
       30 拍"只左移、反馈恒为 0"的暖机期，而一趟激励也就几十位——于是 seed=1 和
       seed=7 给出完全相同的序列。两次实验以为换了激励其实没换，数据里看不出来。
    2. **取最高位而不是最低位**。左移式 LFSR 的最低位是刚灌进去的反馈位，
       暖机期内恒为 0；最高位才携带完整状态。
    """
    state = ((seed if seed else 1) * 2654435761) & 0xFFFFFFFF
    state &= 0x7FFFFFFF
    if state == 0:
        state = 1
    for _ in range(index):
        feedback = ((state >> 30) ^ (state >> 27)) & 1
        state = ((state << 1) | feedback) & 0x7FFFFFFF
    return (state >> 30) & 1
