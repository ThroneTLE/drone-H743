"""RC mapping and guided-calibration page builder and handlers."""

from __future__ import annotations

import time
import tkinter as tk
from tkinter import messagebox, ttk

from ..proto import PROTO_REQ_RCMAP, parse_kv, safe_int
from ..state import RC_WIZARD_TRACE_LOG, append_log

# ---------------------------------------------------------------------------
# 遥控通道映射（对应固件 App/Src/app_rc_config.c）
# ---------------------------------------------------------------------------

RC_CHANNEL_COUNT = 16
RC_FUNCTIONS: tuple[tuple[str, str], ...] = (
    ("roll", "左右 · 横滚"),
    ("pitch", "前后 · 俯仰"),
    ("throttle", "油门 · 升降"),
    ("yaw", "转向 · 偏航"),
    ("arm", "解锁开关"),
    ("mode", "姿态调试开关"),
)
RC_US_MIN = 800
RC_US_MAX = 2200
RC_MIN_SPAN_US = 200
RC_LIVE_FRESH_S = 1.5
# 自动识别的判据：一个通道要被认成"用户正在拨的那个"，必须动得比其余所有通道
# 都明显得多。否则手一抖、或者两根杆同时动，就会绑错功能。
RC_DETECT_MIN_TRAVEL_US = 250
RC_DETECT_DOMINANCE = 2.5


def rc_channel_travel(samples: list[list[int]]) -> list[int]:
    """每个通道在采样窗口内的行程（max-min）。"""
    if not samples:
        return [0] * RC_CHANNEL_COUNT
    travel = []
    for index in range(RC_CHANNEL_COUNT):
        values = [row[index] for row in samples if index < len(row)]
        travel.append(max(values) - min(values) if values else 0)
    return travel


def rc_detect_channel(samples: list[list[int]]) -> tuple[int | None, str]:
    """从采样窗口里挑出用户正在拨动的那一路。

    返回 (channel_index 或 None, 说明)。判据故意保守：宁可让用户多拨一次，
    也不要在两路都动时静默绑错——绑错油门/解锁是会伤人的。
    """
    travel = rc_channel_travel(samples)
    ranked = sorted(range(RC_CHANNEL_COUNT), key=lambda i: travel[i], reverse=True)
    best = ranked[0]
    if travel[best] < RC_DETECT_MIN_TRAVEL_US:
        return None, f"没有检测到明显动作（最大行程 {travel[best]} µs）；请把杆打到底再回中"
    runner_up = travel[ranked[1]]
    if runner_up > 0 and travel[best] < runner_up * RC_DETECT_DOMINANCE:
        return None, (
            f"CH{best + 1} 与 CH{ranked[1] + 1} 同时在动"
            f"（{travel[best]} / {runner_up} µs）；请只拨动一路"
        )
    return best, f"CH{best + 1}  行程 {travel[best]} µs"


def rc_map_is_valid(
    functions: dict[str, dict[str, int]],
) -> tuple[bool, str]:
    """和固件 APP_RcConfig_Validate 同一套判据，让界面提前拦住写不进去的组合。"""
    used: dict[int, str] = {}
    for name, entry in functions.items():
        channel = entry.get("channel", -1)
        if channel < 0:
            continue
        if channel >= RC_CHANNEL_COUNT:
            return False, f"{name}: 通道号 {channel} 越界"
        if channel in used:
            return False, f"CH{channel + 1} 同时绑给了 {used[channel]} 和 {name}"
        used[channel] = name
        low, mid, high = entry["min"], entry["mid"], entry["max"]
        if low < RC_US_MIN or high > RC_US_MAX:
            return False, f"{name}: 端点超出 {RC_US_MIN}~{RC_US_MAX} µs"
        if high - low < RC_MIN_SPAN_US:
            return False, f"{name}: 行程只有 {high - low} µs，至少要 {RC_MIN_SPAN_US}"
        if not (low < mid < high):
            return False, f"{name}: 中位 {mid} 不在 {low}~{high} 之间"
    if not used:
        return False, "没有任何功能绑定了通道"
    return True, "映射合法"


# --- 引导式校准（PX4 风格）-------------------------------------------------
#
# 一次走完三件事：哪一路对应哪个功能、方向正不正、行程两端在哪。分开做的话用户要
# 先猜通道号、再单独跑一次行程，而且没有任何东西能证明"我推的是油门"这件事。
#
# 每步只认一路：要求主导通道的偏移量超过阈值，且明显大于第二名。摇杆同轴串扰
# （推油门带动偏航几十 µs）会被这条挡掉；两根杆真的一起动时宁可让用户重做。

RC_WIZARD_MIN_DEVIATION_US = 200
RC_WIZARD_DOMINANCE = 2.5
RC_WIZARD_HOLD_TOLERANCE_US = 40
RC_WIZARD_HOLD_FRAMES = 8

# 不自回中的通道：油门杆松手停在哪就是哪，开关只有两个位置。这一条同时决定两件事：
#
#   1. 中位怎么取——"开始那一刻的读数"对它们不是中位，必须取实测行程的中点。油门尤其
#      不能弄错：本机油门是双用的，throttle_01 走 min..max 管直通油门，而 norm 是绕中位
#      算的、喂给定高速率（50% 行程 = 保持当前高度）。中位若被定在行程底部，稍微推一点
#      油门就等于满爬升率。
#   2. 每一步拿什么当参考——自回中的杆比"固定中位"，因为松手一定回得去；不自回中的
#      比"上一步结束时的位置"，因为它们根本回不去中位，用固定中位判会永远等不到动作。
RC_WIZARD_NON_CENTERING = ("throttle", "arm", "mode")
# 自回中的杆：中位应当落在行程中段。落在两端 20% 以内说明"回中"那一刻手还压着杆，
# 这种读数不能当 subtrim 用。
RC_WIZARD_CENTER_MARGIN = 0.2

#
# 提示语按"动作"写，不按"术语"写：用户此刻还不知道哪根杆是横滚，只知道自己想让飞机
# 往哪走。反过来正是这些动作定义了映射。
RC_WIZARD_STEPS: tuple[tuple[str, int, str], ...] = (
    ("throttle", +1, "把油门推到最高，保持不动"),
    ("throttle", -1, "把油门拉到最低，保持不动"),
    ("pitch", +1, "把控制「前后」的杆向前推到底，保持不动"),
    ("pitch", -1, "把控制「前后」的杆向后拉到底，保持不动"),
    ("roll", +1, "把控制「左右」的杆向右推到底，保持不动"),
    ("roll", -1, "把控制「左右」的杆向左推到底，保持不动"),
    ("yaw", +1, "把控制「转向」的杆向右转到底，保持不动"),
    ("yaw", -1, "把控制「转向」的杆向左转到底，保持不动"),
    ("arm", +1, "把解锁开关拨到「解锁」一侧，保持不动"),
    ("arm", -1, "把解锁开关拨回「上锁」一侧，保持不动"),
    ("mode", +1, "把姿态调试开关拨到高位，保持不动"),
    ("mode", -1, "把姿态调试开关拨回低位，保持不动"),
)


def rc_wizard_dominant(center: list[int], frame: list[int]) -> tuple[int | None, int]:
    """这一帧里偏离中位最远的那一路，以及它的带符号偏移量。"""
    # 两边都要够长：参考基准来自"开闸那一刻"的快照，若那一帧是残缺的（串口把一行
    # 截断过），后面按固定 16 路索引就会 IndexError，而这条路径每帧都跑。
    if len(center) < RC_CHANNEL_COUNT or len(frame) < RC_CHANNEL_COUNT:
        return None, 0
    deviations = [frame[i] - center[i] for i in range(RC_CHANNEL_COUNT)]
    ranked = sorted(range(RC_CHANNEL_COUNT), key=lambda i: abs(deviations[i]), reverse=True)
    best = ranked[0]
    if abs(deviations[best]) < RC_WIZARD_MIN_DEVIATION_US:
        return None, deviations[best]
    runner_up = abs(deviations[ranked[1]])
    if runner_up > 0 and abs(deviations[best]) < runner_up * RC_WIZARD_DOMINANCE:
        return None, deviations[best]
    return best, deviations[best]


def rc_wizard_step_ready(
    reference: list[int],
    window: list[list[int]],
    opposite_of: int = 0,
) -> tuple[int | None, int, str]:
    """窗口里是否已经稳定停在某一路的极限位置。

    返回 (channel 或 None, 带符号偏移量, 说明)。要求整个窗口都指向同一路且抖动
    很小——没有这个"保持"条件，用户从一端扫到另一端的途中就会被误判成到位。

    `opposite_of` 非零时，还要求这次的偏移方向与它相反：一对步骤（推到底 / 拉到底）
    必须真的往两个方向走过，否则判不出正反。
    """
    if len(window) < RC_WIZARD_HOLD_FRAMES:
        return None, 0, "等待动作…"
    recent = window[-RC_WIZARD_HOLD_FRAMES:]
    picks = [rc_wizard_dominant(reference, frame) for frame in recent]
    channels = {channel for channel, _dev in picks}
    if len(channels) != 1 or None in channels:
        return None, 0, "等待动作…"
    channel = picks[-1][0]
    assert channel is not None
    values = [frame[channel] for frame in recent]
    if max(values) - min(values) > RC_WIZARD_HOLD_TOLERANCE_US:
        return None, picks[-1][1], f"CH{channel + 1} 还在动，请保持不动"
    deviation = picks[-1][1]
    if opposite_of != 0 and (deviation * opposite_of) > 0:
        return None, deviation, f"CH{channel + 1} 还在同一侧，请往相反方向推到底"
    return channel, deviation, f"CH{channel + 1}  {deviation:+d} µs"


def rc_wizard_window_stable(window: list[list[int]]) -> bool:
    """最近这段时间所有通道都没在动。"""
    if len(window) < RC_WIZARD_HOLD_FRAMES:
        return False
    recent = window[-RC_WIZARD_HOLD_FRAMES:]
    for index in range(RC_CHANNEL_COUNT):
        values = [frame[index] for frame in recent if index < len(frame)]
        if not values or (max(values) - min(values)) > RC_WIZARD_HOLD_TOLERANCE_US:
            return False
    return True


def rc_wizard_gate_open(
    window: list[list[int]],
    center: list[int],
    release_channel: int | None,
) -> tuple[bool, str]:
    """上一步采完之后，什么时候才允许开始判定下一步。

    没有这道闸，用户推到底不动手，下一步会立刻在同一个位置上再采一次——12 步会在
    几秒内自己跑完，而且每一步记的都是同一个读数。

    `release_channel` 是上一步用掉的自回中通道，必须先松回中位；不自回中的通道
    （油门、开关）没有中位可回，只要求读数稳定下来。
    """
    if not rc_wizard_window_stable(window):
        return False, "等待动作稳定…"
    if release_channel is None:
        return True, ""
    last = window[-1]
    if release_channel >= min(len(last), len(center)):
        return True, ""
    if abs(last[release_channel] - center[release_channel]) >= RC_WIZARD_MIN_DEVIATION_US:
        return False, f"请先松开 CH{release_channel + 1}，让它回到中位"
    return True, ""


def rc_wizard_build_map(
    center: list[int],
    travel_min: list[int],
    travel_max: list[int],
    results: dict[tuple[str, int], tuple[int, int]],
    previous: dict[str, dict[str, int]],
) -> tuple[dict[str, dict[str, int]], list[str]]:
    """把引导采到的结果拼成一份映射。

    `results` 的键是 (功能, 方向)，值是 (通道, 带符号偏移量)。端点取整个引导过程中
    该通道的实测最小/最大值，而不只是两个步骤的瞬时值——用户在中途扫过的更极端位置
    同样是真实行程。
    """
    functions = {name: dict(previous.get(name, {})) for name, _label in RC_FUNCTIONS}
    warnings: list[str] = []

    for name, _label in RC_FUNCTIONS:
        high = results.get((name, +1))
        low = results.get((name, -1))
        if high is None or low is None:
            warnings.append(f"{name}: 两个方向没有都采到，保留原设置")
            continue
        if high[0] != low[0]:
            warnings.append(
                f"{name}: 两次动作落在不同通道（CH{high[0] + 1} / CH{low[0] + 1}），保留原设置"
            )
            continue
        channel = high[0]
        if high[1] * low[1] >= 0:
            warnings.append(f"{name}: 两次动作方向相同，无法判断正反，保留原设置")
            continue
        low_us = travel_min[channel]
        high_us = travel_max[channel]
        if high_us - low_us < RC_MIN_SPAN_US:
            warnings.append(f"{name}: CH{channel + 1} 行程只有 {high_us - low_us} µs，保留原设置")
            continue
        # 用户被要求"推到最高/最右"时通道值反而变小 → 这一路是反的。
        reversed_flag = 1 if high[1] < 0 else 0
        span = high_us - low_us
        mid = low_us + span // 2
        if name not in RC_WIZARD_NON_CENTERING and center:
            measured = center[channel]
            margin = int(span * RC_WIZARD_CENTER_MARGIN)
            if (low_us + margin) <= measured <= (high_us - margin):
                # 自回中的杆保留实测中位，这就是 subtrim：发射机上的微调偏移会被吃掉。
                mid = measured
            else:
                warnings.append(
                    f"{name}: 回中时 CH{channel + 1} 停在 {measured} µs，不在行程中段，"
                    f"按行程中点 {mid} 取中位"
                )
        functions[name] = {
            "channel": channel,
            "reversed": reversed_flag,
            "min": low_us,
            "mid": mid,
            "max": high_us,
        }

    assigned: dict[int, str] = {}
    for name, entry in functions.items():
        channel = entry.get("channel", -1)
        if channel < 0:
            continue
        if channel in assigned:
            warnings.append(
                f"CH{channel + 1} 同时被 {assigned[channel]} 和 {name} 认领，请重做这两步"
            )
        else:
            assigned[channel] = name
    return functions, warnings


def rc_normalize(entry: dict[str, int], channel_us: int, deadband_us: int) -> float:
    """复刻固件 APP_RcConfig_Normalize，用于本地预览摇杆位置。"""
    if entry.get("channel", -1) < 0:
        return 0.0
    centered = channel_us - entry["mid"]
    if -deadband_us < centered < deadband_us:
        return 0.0
    if centered > 0:
        span = entry["max"] - entry["mid"]
        centered = min(centered, span)
    else:
        span = entry["mid"] - entry["min"]
        centered = max(centered, -span)
    if span <= 0:
        return 0.0
    value = centered / span
    if entry.get("reversed"):
        value = -value
    return max(-1.0, min(1.0, value))

UI_PALETTE = {
    "accent": "#4DA3F5",
    "border_strong": "#4C5666",
    "console": "#171B21",
}


class RcWizardPageMixin:
    def _build_rc_page(self, parent: ttk.Frame) -> None:
        header = ttk.Frame(parent)
        header.pack(fill=tk.X)
        ttk.Label(header, text="遥控通道标定", style="PageTitle.TLabel").pack(side=tk.LEFT)
        ttk.Label(header, textvariable=self.rc_link_var, style="Muted.TLabel").pack(side=tk.RIGHT)
        ttk.Label(
            parent,
            text=(
                "把通道号、正反向和端点行程从固件里挪出来：先看实时通道确认接收机在收，"
                "再用「自动识别」把每个功能绑到对应的那一路，最后走一次端点标定并写入 Flash。"
                "所有写操作都要求飞控处于上锁状态。"
            ),
            wraplength=1120,
            style="Muted.TLabel",
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        live = ttk.LabelFrame(parent, text="1 · 实时通道", padding=10)
        live.pack(fill=tk.X)
        bars = ttk.Frame(live)
        bars.pack(side=tk.LEFT, fill=tk.X, expand=True)
        bars.columnconfigure(1, weight=1)
        self.rc_channel_bars = []
        self.rc_channel_labels = []
        for index in range(RC_CHANNEL_COUNT):
            row = index % 8
            column = (index // 8) * 3
            ttk.Label(bars, text=f"CH{index + 1}", width=5, style="Muted.TLabel").grid(
                row=row, column=column, sticky=tk.W, padx=(0 if column == 0 else 16, 4), pady=1
            )
            bar = ttk.Progressbar(bars, maximum=1000.0, length=170)
            bar.grid(row=row, column=column + 1, sticky=tk.EW, pady=1)
            value = tk.StringVar(value="----")
            ttk.Label(bars, textvariable=value, width=6, style="Mono.TLabel").grid(
                row=row, column=column + 2, sticky=tk.W, padx=(6, 0), pady=1
            )
            self.rc_channel_bars.append(bar)
            self.rc_channel_labels.append(value)
        bars.columnconfigure(1, weight=1)
        bars.columnconfigure(4, weight=1)

        sticks = ttk.Frame(live)
        sticks.pack(side=tk.RIGHT, padx=(18, 0))
        self.rc_stick_canvas = tk.Canvas(
            sticks, width=250, height=126, highlightthickness=0,
            background=UI_PALETTE["console"],
        )
        self.rc_stick_canvas.pack()
        ttk.Label(
            sticks, text="左：油门 / 转向     右：前后 / 左右",
            style="Muted.TLabel",
        ).pack(pady=(4, 0))

        wizard = ttk.LabelFrame(parent, text="2 · 引导校准（推荐）", padding=10)
        wizard.pack(fill=tk.X, pady=(9, 0))
        ttk.Label(
            wizard,
            text=(
                "按提示依次把每根杆和开关推到两端。一次走完就能定出「哪一路是哪个功能」、"
                "「方向正不正」和「行程两端在哪」三件事——不用先知道通道号。"
                "自动回中的杆保留实测中位（吃掉发射机微调）；油门和开关不自回中，"
                "中位按实测行程中点推算，不看开始时停在哪。"
            ),
            style="Muted.TLabel", wraplength=1080,
        ).pack(fill=tk.X)
        prompt = ttk.Frame(wizard)
        prompt.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(prompt, textvariable=self.rc_wizard_step_var, width=10,
                  style="Muted.TLabel").pack(side=tk.LEFT)
        ttk.Label(prompt, textvariable=self.rc_wizard_prompt_var,
                  style="SectionTitle.TLabel", wraplength=680).pack(side=tk.LEFT)
        ttk.Label(prompt, textvariable=self.rc_wizard_detect_var,
                  style="Mono.TLabel").pack(side=tk.RIGHT)
        self.rc_wizard_progress = ttk.Progressbar(
            wizard, mode="determinate", maximum=float(len(RC_WIZARD_STEPS)),
            variable=self.rc_wizard_progress_var,
        )
        self.rc_wizard_progress.pack(fill=tk.X, pady=(8, 0))
        wizard_buttons = ttk.Frame(wizard)
        wizard_buttons.pack(fill=tk.X, pady=(8, 0))
        self.rc_wizard_start_button = ttk.Button(
            wizard_buttons, text="开始引导校准", style="Primary.TButton",
            command=self._rc_wizard_start,
        )
        self.rc_wizard_start_button.pack(side=tk.LEFT)
        self.rc_wizard_back_button = ttk.Button(
            wizard_buttons, text="上一步", style="Secondary.TButton",
            command=self._rc_wizard_back, state=tk.DISABLED,
        )
        self.rc_wizard_back_button.pack(side=tk.LEFT, padx=(8, 0))
        self.rc_wizard_skip_button = ttk.Button(
            wizard_buttons, text="跳过这步", style="Secondary.TButton",
            command=self._rc_wizard_skip, state=tk.DISABLED,
        )
        self.rc_wizard_skip_button.pack(side=tk.LEFT, padx=(8, 0))
        self.rc_wizard_cancel_button = ttk.Button(
            wizard_buttons, text="取消", style="Danger.TButton",
            command=self._rc_wizard_cancel, state=tk.DISABLED,
        )
        self.rc_wizard_cancel_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(wizard, textvariable=self.rc_wizard_result_var,
                  style="Guide.TLabel", wraplength=1080).pack(fill=tk.X, pady=(8, 0))

        mapping = ttk.LabelFrame(parent, text="3 · 功能绑定（引导结果，可手工微调）", padding=10)
        mapping.pack(fill=tk.X, pady=(9, 0))
        grid = ttk.Frame(mapping)
        grid.pack(fill=tk.X)
        headers = ("功能", "通道", "反向", "最小", "中位", "最大", "实时", "")
        for column, title in enumerate(headers):
            ttk.Label(grid, text=title, style="Muted.TLabel").grid(
                row=0, column=column, sticky=tk.W, padx=(0, 10), pady=(0, 4)
            )
        self.rc_rows = {}
        for row, (name, label) in enumerate(RC_FUNCTIONS, start=1):
            entry: dict[str, tk.Variable] = {}
            ttk.Label(grid, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=2)
            entry["channel"] = tk.StringVar(value="-")
            ttk.Combobox(
                grid, textvariable=entry["channel"], width=6, state="readonly",
                values=["-"] + [f"CH{i + 1}" for i in range(RC_CHANNEL_COUNT)],
            ).grid(row=row, column=1, sticky=tk.W, padx=(0, 10), pady=2)
            entry["reversed"] = tk.BooleanVar(value=False)
            ttk.Checkbutton(grid, variable=entry["reversed"]).grid(
                row=row, column=2, sticky=tk.W, padx=(0, 10), pady=2
            )
            for column, key in ((3, "min"), (4, "mid"), (5, "max")):
                entry[key] = tk.StringVar(value="----")
                ttk.Entry(grid, textvariable=entry[key], width=7).grid(
                    row=row, column=column, sticky=tk.W, padx=(0, 10), pady=2
                )
            entry["live"] = tk.StringVar(value="----")
            ttk.Label(grid, textvariable=entry["live"], width=12, style="Mono.TLabel").grid(
                row=row, column=6, sticky=tk.W, padx=(0, 10), pady=2
            )
            ttk.Button(
                grid, text="自动识别", style="Secondary.TButton",
                command=lambda func=name: self._rc_start_detect(func),
            ).grid(row=row, column=7, sticky=tk.W, pady=2)
            self.rc_rows[name] = entry

        deadband = ttk.Frame(mapping)
        deadband.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(deadband, text="摇杆死区 µs").pack(side=tk.LEFT, padx=(0, 6))
        ttk.Entry(deadband, textvariable=self.rc_deadband_var, width=7).pack(side=tk.LEFT)
        ttk.Label(
            deadband, textvariable=self.rc_detect_var, style="Guide.TLabel", wraplength=760,
        ).pack(side=tk.LEFT, padx=(18, 0), fill=tk.X, expand=True)

        cal = ttk.LabelFrame(parent, text="4 · 手工端点标定与写入", padding=10)
        cal.pack(fill=tk.X, pady=(9, 0))
        buttons = ttk.Frame(cal)
        buttons.pack(fill=tk.X)
        self.rc_center_button = ttk.Button(
            buttons, text="① 记录中位（松杆）", style="Secondary.TButton",
            command=self._rc_capture_center,
        )
        self.rc_center_button.pack(side=tk.LEFT)
        self.rc_sweep_button = ttk.Button(
            buttons, text="② 开始记录行程", style="Secondary.TButton",
            command=self._rc_toggle_sweep,
        )
        self.rc_sweep_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            buttons, text="从飞控读取", style="Secondary.TButton",
            command=self._rc_request_map,
        ).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Button(
            buttons, text="应用到飞控 RAM", style="Secondary.TButton",
            command=self._rc_apply_ram,
        ).pack(side=tk.LEFT, padx=(8, 0))
        self.rc_commit_button = ttk.Button(
            buttons, text="③ 写入 Flash", style="Warning.TButton",
            command=self._rc_commit,
        )
        self.rc_commit_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            buttons, text="恢复出厂映射", style="Danger.TButton",
            command=self._rc_reset_defaults,
        ).pack(side=tk.RIGHT)
        ttk.Label(
            cal, textvariable=self.rc_status_var, style="Guide.TLabel", wraplength=1080,
        ).pack(fill=tk.X, pady=(9, 0))
        ttk.Label(
            cal,
            text=(
                "走完上面的引导校准后这里通常不用碰。要单独补一路时：①松杆记录中位，"
                "②开始记录行程、把要补的那一路推到两端、再点一次停止。"
                "中位必须在最小和最大之间，行程至少 200 µs，否则飞控会拒绝写入。"
            ),
            style="Muted.TLabel", wraplength=1080,
        ).pack(fill=tk.X, pady=(6, 0))

        self._rc_refresh_controls()
        self._rc_wizard_refresh()

    def _rc_row_entry(self, name: str) -> dict[str, int]:
        """把界面上一行读成固件那套整数字段；无法解析时回落到默认值。"""
        row = self.rc_rows[name]
        label = row["channel"].get()
        channel = int(label[2:]) - 1 if label.startswith("CH") else -1

        def number(key: str, fallback: int) -> int:
            try:
                return int(float(row[key].get()))
            except (TypeError, ValueError):
                return fallback

        return {
            "channel": channel,
            "reversed": 1 if row["reversed"].get() else 0,
            "min": number("min", 1000),
            "mid": number("mid", 1500),
            "max": number("max", 2000),
        }

    def _rc_collect_map(self) -> dict[str, dict[str, int]]:
        return {name: self._rc_row_entry(name) for name, _label in RC_FUNCTIONS}

    def _rc_set_row(self, name: str, entry: dict[str, int]) -> None:
        row = self.rc_rows[name]
        channel = entry.get("channel", -1)
        row["channel"].set(f"CH{channel + 1}" if channel >= 0 else "-")
        row["reversed"].set(bool(entry.get("reversed")))
        for key in ("min", "mid", "max"):
            row[key].set(str(entry.get(key, 0)))

    def _rc_deadband(self) -> int:
        try:
            return max(0, min(200, int(float(self.rc_deadband_var.get()))))
        except (TypeError, ValueError):
            return 20

    def _rc_handle_live_line(self, line: str) -> None:
        values = parse_kv(line)
        if "us" in values:
            try:
                channels = [int(part) for part in values["us"].split(",")]
            except ValueError:
                return
            if len(channels) == RC_CHANNEL_COUNT:
                self.rc_channels = channels
                self.rc_live_time = time.monotonic()
                self._rc_accumulate(channels)
            return
        if "fresh" in values:
            self.rc_link_values = values

    def _rc_handle_map_line(self, line: str) -> None:
        values = parse_kv(line)
        if "func" in values:
            channel = safe_int(values.get("ch"), -1)
            self.rc_map_values[values["func"]] = {
                "channel": channel,
                "reversed": safe_int(values.get("rev"), 0),
                "min": safe_int(values.get("min"), 1000),
                "mid": safe_int(values.get("mid"), 1500),
                "max": safe_int(values.get("max"), 2000),
            }
            if values["func"] in self.rc_rows:
                self._rc_set_row(values["func"], self.rc_map_values[values["func"]])
            return
        if "state" in values:
            self.rc_map_state = values["state"]
            self.rc_deadband_var.set(values.get("deadband_us", "20"))
            self.rc_status_var.set(self._rc_state_text(values))
            self._rc_refresh_controls()

    _RC_STATE_TEXT = {
        "status": "已读取飞控当前映射",
        "applied_ram": "已应用到飞控 RAM（尚未写 Flash，断电即失效）",
        "reset_ram": "已恢复出厂映射到 RAM（尚未写 Flash）",
        "committed": "已写入 Flash，重启后仍然生效",
        "rejected": "飞控拒绝：映射非法（通道冲突 / 行程不足 / 中位越界）",
        "armed_blocked": "飞控处于解锁状态，拒绝改遥控映射；请先上锁",
        "commit_failed": "写 Flash 失败，映射仍只在 RAM 中",
        "invalid_usage": "命令格式错误",
        "bad_function": "飞控不认识这个功能名",
    }

    def _rc_state_text(self, values: dict[str, str]) -> str:
        state = values.get("state", "")
        text = self._RC_STATE_TEXT.get(state, f"飞控返回 state={state}")
        dirty = safe_int(values.get("dirty"), 0)
        valid = safe_int(values.get("valid"), 1)
        suffix = []
        if dirty:
            suffix.append("RAM 与 Flash 不一致")
        if not valid:
            suffix.append("当前映射非法")
        suffix.append(f"generation={values.get('generation', '-')}")
        return text + "  ·  " + " · ".join(suffix)

    # --- RC：自动识别与端点标定 ---------------------------------------

    def _rc_accumulate(self, channels: list[int]) -> None:
        self._rc_wizard_feed(channels)
        if self.rc_detect_function is not None:
            self.rc_detect_samples.append(list(channels))
            if time.monotonic() >= self.rc_detect_deadline:
                self._rc_finish_detect()
        if self.rc_sweep_active:
            if not self.rc_sweep_min:
                self.rc_sweep_min = list(channels)
                self.rc_sweep_max = list(channels)
            else:
                for index, value in enumerate(channels):
                    self.rc_sweep_min[index] = min(self.rc_sweep_min[index], value)
                    self.rc_sweep_max[index] = max(self.rc_sweep_max[index], value)

    # --- RC：引导校准 --------------------------------------------------

    def _rc_wizard_start(self) -> None:
        if not self._rc_live_ok():
            messagebox.showwarning(
                "没有遥控数据",
                "飞控还没有收到遥控帧。请先给发射机上电并确认接收机已对码，"
                "「实时通道」里能看到数字跳动之后再开始。",
            )
            return
        if not messagebox.askyesno(
            "开始引导校准",
            "开始前请确认：\n\n"
            "  · 螺旋桨已经拆下\n"
            "  · 飞控处于上锁状态\n"
            "  · 会自动回中的杆（横滚 / 俯仰 / 偏航）松手\n"
            "  · 油门和开关放哪都行——它们的中位由实测行程推算，不看现在的位置\n\n"
            "接下来会逐步提示你把每根杆和开关推到两端。确定开始吗？",
        ):
            return
        self.rc_wizard_active = True
        self.rc_wizard_index = 0
        self.rc_wizard_results = {}
        self.rc_wizard_window = []
        self.rc_wizard_armed = False
        self.rc_wizard_release_channel = None
        self.rc_wizard_baseline = list(self.rc_channels)
        # 中位取"开始那一刻"的位置，所以上面的对话框特意要求先回中。
        self.rc_wizard_center = list(self.rc_channels)
        self.rc_wizard_min = list(self.rc_channels)
        self.rc_wizard_max = list(self.rc_channels)
        self.rc_wizard_result_var.set("")
        self._rc_wizard_last_detail = ""
        self._rc_wizard_trace("start", f"center={self.rc_wizard_center}")
        self._rc_wizard_refresh()

    def _rc_wizard_cancel(self) -> None:
        self._rc_wizard_trace("cancel")
        self.rc_wizard_active = False
        self.rc_wizard_window = []
        self.rc_wizard_detect_var.set("")
        self.rc_wizard_prompt_var.set("已取消；映射未改动")
        self.rc_wizard_step_var.set("")
        self.rc_wizard_progress_var.set(0.0)
        self._rc_wizard_refresh()

    def _rc_wizard_skip(self) -> None:
        if not self.rc_wizard_active:
            return
        self._rc_wizard_advance()

    def _rc_wizard_back(self) -> None:
        if not self.rc_wizard_active or self.rc_wizard_index == 0:
            return
        self.rc_wizard_index -= 1
        function, direction, _prompt = RC_WIZARD_STEPS[self.rc_wizard_index]
        self.rc_wizard_results.pop((function, direction), None)
        self.rc_wizard_window = []
        self.rc_wizard_armed = False
        self.rc_wizard_release_channel = None
        self._rc_wizard_refresh()

    def _rc_wizard_advance(self, *, release_channel: int | None = None) -> None:
        previous = RC_WIZARD_STEPS[self.rc_wizard_index][0]
        self.rc_wizard_index += 1
        self.rc_wizard_window = []
        # 每进一步都必须重新过闸：先等读数稳定（必要时先松杆回中），再开始判定。
        # 少了这一步，用户推到底不松手就会被连续判成 12 步全部完成。
        self.rc_wizard_armed = False
        if (
            release_channel is not None
            and self.rc_wizard_index < len(RC_WIZARD_STEPS)
            and RC_WIZARD_STEPS[self.rc_wizard_index][0] == previous
        ):
            # 同一个功能的两步是同一根杆（前推 → 后拉），中间不必回中：用户本来就
            # 是一路扫过去的，硬要求回中只是多余的摩擦。换功能时才需要松手。
            release_channel = None
        self.rc_wizard_release_channel = release_channel
        if self.rc_wizard_index >= len(RC_WIZARD_STEPS):
            self._rc_wizard_finish()
        else:
            self._rc_wizard_refresh()

    def _rc_wizard_reference(self) -> list[int]:
        """判定的参考是"本步开闸那一刻的静止状态"，不是固定中位。

        用固定中位会被停在一边的通道永久压住：油门推到顶不回中，对中位就是个恒定
        450µs 的偏移，之后每一步都会和它打平、判不出主导通道。开闸时全场已经静止，
        以那一刻为零点，任何偏移就一定是用户刚做的新动作。
        """
        return self.rc_wizard_baseline or self.rc_wizard_center

    def _rc_wizard_trace(self, event: str, detail: str = "") -> None:
        """引导校准的状态变化落盘，供事后排查"卡住/闪退/绑错"。

        只记状态跳变，不记每一帧：一次完整校准也就几十行。
        """
        step = "done"
        if self.rc_wizard_active and self.rc_wizard_index < len(RC_WIZARD_STEPS):
            function, direction, _prompt = RC_WIZARD_STEPS[self.rc_wizard_index]
            step = f"{self.rc_wizard_index + 1}/{len(RC_WIZARD_STEPS)} {function}{direction:+d}"
        append_log(
            RC_WIZARD_TRACE_LOG,
            f"{event} step={step} armed={int(self.rc_wizard_armed)} "
            f"release={self.rc_wizard_release_channel} "
            f"us={','.join(str(v) for v in self.rc_channels)}"
            + (f" | {detail}" if detail else ""),
        )

    def _rc_wizard_feed(self, channels: list[int]) -> None:
        """每一帧遥控数据都喂进来：更新实测行程，并判断当前步骤是否已到位。"""
        if not self.rc_wizard_active:
            return
        for index, value in enumerate(channels):
            self.rc_wizard_min[index] = min(self.rc_wizard_min[index], value)
            self.rc_wizard_max[index] = max(self.rc_wizard_max[index], value)
        self.rc_wizard_window.append(list(channels))
        if len(self.rc_wizard_window) > 4 * RC_WIZARD_HOLD_FRAMES:
            del self.rc_wizard_window[0]

        if not self.rc_wizard_armed:
            open_gate, detail = rc_wizard_gate_open(
                self.rc_wizard_window,
                self.rc_wizard_center,
                self.rc_wizard_release_channel,
            )
            self.rc_wizard_detect_var.set(detail)
            if not open_gate:
                if detail != self._rc_wizard_last_detail:
                    self._rc_wizard_last_detail = detail
                    self._rc_wizard_trace("gate_wait", detail)
                return
            # 闸开的这一刻就是本步的起点：不自回中的通道以它为参考。
            self.rc_wizard_baseline = list(channels)
            self.rc_wizard_armed = True
            self.rc_wizard_window = []
            self._rc_wizard_last_detail = ""
            self._rc_wizard_trace("gate_open")
            return

        function, direction, _prompt = RC_WIZARD_STEPS[self.rc_wizard_index]
        opposite_of = 0
        if direction < 0:
            # 一对步骤必须真的往两个方向走过，否则判不出正反。这条同时挡住"推同一
            # 边两次"——每次都从静止零点起算，同向就是没换方向。
            paired = self.rc_wizard_results.get((function, +1))
            if paired is not None:
                opposite_of = paired[1]
        channel, deviation, detail = rc_wizard_step_ready(
            self._rc_wizard_reference(), self.rc_wizard_window, opposite_of
        )
        self.rc_wizard_detect_var.set(detail)
        if channel is None:
            if detail != self._rc_wizard_last_detail:
                self._rc_wizard_last_detail = detail
                self._rc_wizard_trace("waiting", detail)
            return
        self.rc_wizard_results[(function, direction)] = (channel, deviation)
        self._rc_wizard_trace("captured", f"ch={channel} dev={deviation:+d}")
        self._rc_wizard_last_detail = ""
        release = None if function in RC_WIZARD_NON_CENTERING else channel
        self._rc_wizard_advance(release_channel=release)

    def _rc_wizard_finish(self) -> None:
        self._rc_wizard_trace("finish", f"results={self.rc_wizard_results}")
        self.rc_wizard_active = False
        functions, warnings = rc_wizard_build_map(
            self.rc_wizard_center,
            self.rc_wizard_min,
            self.rc_wizard_max,
            self.rc_wizard_results,
            self._rc_collect_map(),
        )
        for name, entry in functions.items():
            self._rc_set_row(name, entry)
        self.rc_wizard_prompt_var.set("引导校准完成")
        self.rc_wizard_detect_var.set("")
        self.rc_wizard_step_var.set("")
        self.rc_wizard_progress_var.set(float(len(RC_WIZARD_STEPS)))
        ok, reason = rc_map_is_valid(functions)
        lines = [
            f"{label}: CH{entry['channel'] + 1 if entry['channel'] >= 0 else '-'}"
            f"{'（反向）' if entry.get('reversed') else ''} "
            f"{entry['min']}~{entry['max']}"
            for (name, label), entry in zip(RC_FUNCTIONS, functions.values())
        ]
        summary = "结果：" + "；".join(lines)
        if warnings:
            summary += "\n注意：" + "；".join(warnings)
        if not ok:
            summary += f"\n映射仍不合法：{reason}"
        else:
            # 立刻下发到飞控 RAM。不下发的话，上面的摇杆十字按新映射画、飞控却还按旧的
            # 飞，用户"看着方向是对的"其实什么都没验证到。Flash 仍然要单独确认。
            self._rc_send_map(commit=False)
            summary += (
                "\n已应用到飞控 RAM。下一步：推杆确认第 1 节的摇杆十字方向正确，"
                "再点「③ 写入 Flash」持久化。"
            )
        self.rc_wizard_result_var.set(summary)
        self._rc_wizard_refresh()

    def _rc_wizard_refresh(self) -> None:
        if not hasattr(self, "rc_wizard_start_button"):
            return
        active = self.rc_wizard_active
        self.rc_wizard_start_button.configure(state=tk.DISABLED if active else tk.NORMAL)
        self.rc_wizard_back_button.configure(
            state=tk.NORMAL if active and self.rc_wizard_index > 0 else tk.DISABLED
        )
        self.rc_wizard_skip_button.configure(state=tk.NORMAL if active else tk.DISABLED)
        self.rc_wizard_cancel_button.configure(state=tk.NORMAL if active else tk.DISABLED)
        if active:
            function, direction, prompt = RC_WIZARD_STEPS[self.rc_wizard_index]
            self.rc_wizard_step_var.set(
                f"{self.rc_wizard_index + 1}/{len(RC_WIZARD_STEPS)}"
            )
            self.rc_wizard_prompt_var.set(prompt)
            self.rc_wizard_progress_var.set(float(self.rc_wizard_index))

    def _rc_start_detect(self, function: str) -> None:
        if not self._rc_live_ok():
            messagebox.showwarning("没有遥控数据", "飞控还没有收到遥控帧，请先开机并确认接收机已对码。")
            return
        self.rc_detect_function = function
        self.rc_detect_samples = []
        self.rc_detect_deadline = time.monotonic() + 4.0
        self.rc_detect_var.set(f"正在识别「{function}」：把要绑定的那一路拨到底再回中（4 秒）…")

    def _rc_finish_detect(self) -> None:
        function = self.rc_detect_function
        samples = self.rc_detect_samples
        self.rc_detect_function = None
        self.rc_detect_samples = []
        if function is None:
            return
        channel, detail = rc_detect_channel(samples)
        if channel is None:
            self.rc_detect_var.set(f"「{function}」识别失败：{detail}")
            return
        self.rc_rows[function]["channel"].set(f"CH{channel + 1}")
        self.rc_detect_var.set(f"「{function}」→ {detail}")

    def _rc_capture_center(self) -> None:
        if not self._rc_live_ok():
            messagebox.showwarning("没有遥控数据", "飞控还没有收到遥控帧，无法记录中位。")
            return
        self.rc_center = list(self.rc_channels)
        for name, _label in RC_FUNCTIONS:
            entry = self._rc_row_entry(name)
            if 0 <= entry["channel"] < RC_CHANNEL_COUNT:
                self.rc_rows[name]["mid"].set(str(self.rc_center[entry["channel"]]))
        self.rc_status_var.set("已记录中位；接下来点②并把每根杆和开关都打到两端各走一圈")

    def _rc_toggle_sweep(self) -> None:
        if self.rc_sweep_active:
            self.rc_sweep_active = False
            self._rc_apply_sweep()
            self._rc_refresh_controls()
            return
        if not self._rc_live_ok():
            messagebox.showwarning("没有遥控数据", "飞控还没有收到遥控帧，无法记录行程。")
            return
        self.rc_sweep_active = True
        self.rc_sweep_min = []
        self.rc_sweep_max = []
        self.rc_status_var.set("正在记录行程：把每根杆和开关都推到两端，完成后再点一次停止")
        self._rc_refresh_controls()

    def _rc_apply_sweep(self) -> None:
        if not self.rc_sweep_min:
            self.rc_status_var.set("没有采到任何行程样本")
            return
        applied = 0
        skipped: list[str] = []
        for name, _label in RC_FUNCTIONS:
            entry = self._rc_row_entry(name)
            channel = entry["channel"]
            if not (0 <= channel < RC_CHANNEL_COUNT):
                continue
            low = self.rc_sweep_min[channel]
            high = self.rc_sweep_max[channel]
            # 行程不足的通道保留旧端点：把 1490~1510 这种没拨到的开关写进去，
            # 会让归一化增益放大 50 倍，比"没标定"危险得多。
            if high - low < RC_MIN_SPAN_US:
                skipped.append(f"{name}(CH{channel + 1} 只动了 {high - low}µs)")
                continue
            self.rc_rows[name]["min"].set(str(low))
            self.rc_rows[name]["max"].set(str(high))
            mid = int(self.rc_rows[name]["mid"].get() or 0)
            if not (low < mid < high):
                self.rc_rows[name]["mid"].set(str((low + high) // 2))
            applied += 1
        text = f"行程标定完成：{applied} 个功能已更新端点"
        if skipped:
            text += "；未更新（行程不足）：" + "、".join(skipped)
        self.rc_status_var.set(text)

    # --- RC：与飞控通信 -----------------------------------------------

    def _rc_live_ok(self) -> bool:
        return (
            self.rc_live_time > 0.0
            and (time.monotonic() - self.rc_live_time) <= RC_LIVE_FRESH_S
            and safe_int(self.rc_link_values.get("fresh"), 0) != 0
        )

    def _rc_request_map(self) -> None:
        if not self._transport_connected():
            messagebox.showwarning("未连接", "请先连接飞控。")
            return
        self.rc_map_values.clear()
        self._send_proto_once(PROTO_REQ_RCMAP, "RCMAP?")

    def _rc_send_map(self, *, commit: bool) -> None:
        if not self._transport_connected():
            messagebox.showwarning("未连接", "请先连接飞控。")
            return
        if safe_int(self.rc_link_values.get("armed"), 0) != 0:
            messagebox.showwarning("飞控已解锁", "改遥控映射前必须先上锁。")
            return
        functions = self._rc_collect_map()
        ok, reason = rc_map_is_valid(functions)
        if not ok:
            messagebox.showerror("映射非法", reason)
            self.rc_status_var.set(f"未发送：{reason}")
            return
        self.transport.send_line(f"RCMAP DEADBAND {self._rc_deadband()}")
        for name, entry in functions.items():
            self.transport.send_line(
                f"RCMAP SET {name} {entry['channel']} {entry['reversed']} "
                f"{entry['min']} {entry['mid']} {entry['max']}"
            )
        self.transport.send_line("RCMAP CALIBRATED 1")
        if commit:
            self.transport.send_line("RCMAP COMMIT")
        self.rc_status_var.set(
            "已发送映射，等待飞控确认…" if commit else "已应用到 RAM，等待飞控确认…"
        )

    def _rc_apply_ram(self) -> None:
        self._rc_send_map(commit=False)

    def _rc_commit(self) -> None:
        functions = self._rc_collect_map()
        ok, reason = rc_map_is_valid(functions)
        if not ok:
            messagebox.showerror("映射非法", reason)
            return
        summary = "\n".join(
            f"  {label}: CH{entry['channel'] + 1 if entry['channel'] >= 0 else '-'}"
            f"{' 反向' if entry['reversed'] else ''}  "
            f"{entry['min']}/{entry['mid']}/{entry['max']}"
            for (name, label), entry in zip(RC_FUNCTIONS, functions.values())
        )
        if not messagebox.askyesno(
            "写入遥控映射到 Flash",
            f"以下映射将写入飞控参数 Flash，重启后生效：\n\n{summary}\n\n确定继续吗？",
        ):
            return
        self._rc_send_map(commit=True)

    def _rc_reset_defaults(self) -> None:
        if not self._transport_connected():
            messagebox.showwarning("未连接", "请先连接飞控。")
            return
        if not messagebox.askyesno(
            "恢复出厂映射",
            "将把遥控映射恢复为 CH1~CH6 / 1000-1500-2000 并写入 Flash。确定吗？",
        ):
            return
        self.transport.send_line("RCMAP RESET")
        self.transport.send_line("RCMAP COMMIT")
        self.rc_status_var.set("已请求恢复出厂映射…")

    # --- RC：渲染 ------------------------------------------------------

    def _rc_refresh_controls(self) -> None:
        if not hasattr(self, "rc_sweep_button"):
            return
        self.rc_sweep_button.configure(
            text="② 停止记录行程" if self.rc_sweep_active else "② 开始记录行程"
        )

    def _rc_render(self) -> None:
        if not hasattr(self, "rc_channel_bars"):
            return
        live = self._rc_live_ok()
        for index, bar in enumerate(self.rc_channel_bars):
            value = self.rc_channels[index] if index < len(self.rc_channels) else 0
            bar["value"] = max(0, min(1000, value - 1000))
            self.rc_channel_labels[index].set(str(value) if live else "----")

        deadband = self._rc_deadband()
        for name, _label in RC_FUNCTIONS:
            entry = self._rc_row_entry(name)
            channel = entry["channel"]
            if not live or not (0 <= channel < RC_CHANNEL_COUNT):
                self.rc_rows[name]["live"].set("----")
                continue
            raw = self.rc_channels[channel]
            self.rc_rows[name]["live"].set(
                f"{raw}  {rc_normalize(entry, raw, deadband):+.2f}"
            )

        if live:
            fresh = self.rc_link_values
            self.rc_link_var.set(
                f"遥控：✓ 在线  LQ={fresh.get('lq', '-')}  RSSI={fresh.get('rssi', '-')}  "
                f"帧率={safe_int(fresh.get('fps_x10'), 0) / 10:.1f} Hz  "
                f"armed={fresh.get('armed', '-')}"
            )
        else:
            self.rc_link_var.set("遥控：✗ 未收到遥控帧（发射机是否开机 / 接收机是否对码）")
        self._rc_draw_sticks(live, deadband)

    def _rc_draw_sticks(self, live: bool, deadband: int) -> None:
        canvas = self.rc_stick_canvas
        canvas.delete("all")
        pairs = (
            (60, "yaw", "throttle", True),
            (190, "roll", "pitch", False),
        )
        for center_x, x_func, y_func, throttle_axis in pairs:
            center_y = 63
            half = 52
            canvas.create_rectangle(
                center_x - half, center_y - half, center_x + half, center_y + half,
                outline=UI_PALETTE["border_strong"], width=1,
            )
            canvas.create_line(center_x - half, center_y, center_x + half, center_y,
                               fill=UI_PALETTE["border_strong"])
            canvas.create_line(center_x, center_y - half, center_x, center_y + half,
                               fill=UI_PALETTE["border_strong"])
            if not live:
                continue
            x_entry = self._rc_row_entry(x_func)
            y_entry = self._rc_row_entry(y_func)
            x_norm = self._rc_axis_value(x_entry, deadband)
            if throttle_axis:
                # 油门画成 0..1 的绝对行程，回中不代表零输出。
                y_norm = -(self._rc_throttle_value(y_entry) * 2.0 - 1.0)
            else:
                y_norm = -self._rc_axis_value(y_entry, deadband)
            px = center_x + x_norm * half
            py = center_y + y_norm * half
            canvas.create_oval(px - 6, py - 6, px + 6, py + 6,
                               fill=UI_PALETTE["accent"], outline="")

    def _rc_axis_value(self, entry: dict[str, int], deadband: int) -> float:
        channel = entry["channel"]
        if not (0 <= channel < RC_CHANNEL_COUNT):
            return 0.0
        return rc_normalize(entry, self.rc_channels[channel], deadband)

    def _rc_throttle_value(self, entry: dict[str, int]) -> float:
        channel = entry["channel"]
        if not (0 <= channel < RC_CHANNEL_COUNT):
            return 0.0
        span = entry["max"] - entry["min"]
        if span <= 0:
            return 0.0
        value = (self.rc_channels[channel] - entry["min"]) / span
        if entry["reversed"]:
            value = 1.0 - value
        return max(0.0, min(1.0, value))


__all__ = [
    "RC_CHANNEL_COUNT",
    "RC_DETECT_DOMINANCE",
    "RC_DETECT_MIN_TRAVEL_US",
    "RC_FUNCTIONS",
    "RC_LIVE_FRESH_S",
    "RC_MIN_SPAN_US",
    "RC_US_MAX",
    "RC_US_MIN",
    "RC_WIZARD_CENTER_MARGIN",
    "RC_WIZARD_DOMINANCE",
    "RC_WIZARD_HOLD_FRAMES",
    "RC_WIZARD_HOLD_TOLERANCE_US",
    "RC_WIZARD_MIN_DEVIATION_US",
    "RC_WIZARD_NON_CENTERING",
    "RC_WIZARD_STEPS",
    "RcWizardPageMixin",
    "rc_channel_travel",
    "rc_detect_channel",
    "rc_map_is_valid",
    "rc_normalize",
    "rc_wizard_build_map",
    "rc_wizard_dominant",
    "rc_wizard_gate_open",
    "rc_wizard_step_ready",
    "rc_wizard_window_stable",
]
