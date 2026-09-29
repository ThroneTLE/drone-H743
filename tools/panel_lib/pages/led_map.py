"""「维护 · 状态灯」页：给飞控的每一种状态/报错挑颜色和节奏。

飞控是**唯一权威**。本页只是编辑器：所有值都从 `LEDMAP?` 读回来，改动先发到飞控的
RAM（立刻能在板子上看到），确认满意了再 `LEDMAP COMMIT` 落 Flash。本页不在
panel_state.json 里缓存飞控的值——缓存一份就会出现"界面显示的和板子上的不一样"，
而那正是这整套颜色语言最不能出的问题。

**节奏按档位给，不让用户填毫秒。** 固件收的是 on/off/gap/period/dim 五个毫秒数，
但"闪多快"本来就只有那么几种有意义的取值：分不清 160 ms 和 180 ms 的闪，填第二个
数字只是在制造一种"我在精细调节"的错觉。所以界面给一串命名档位（常亮 / 慢闪 /
快呼吸 / 数闪…），每个档位就是一组固定毫秒数。档位表里写着秒数，所以命名没有歧义。
`自定义…` 仍然留着——固件支持任意值，界面不该把它锁死，但它退到第二层。

**档位是显示语言，不是存储格式。** 存进 Flash 的仍是毫秒数。板子上读回来的值若不
等于任何一档（老配置、或别的工具写的），界面显示 `自定义…` 并把真实毫秒数摊开，
**不四舍五入到最近的档位**——那会让"我什么都没改"的一次打开悄悄改掉板子上的值。

**主从布局而不是 16 行平铺。** 16 条绑定 × 十来个控件 = 一百多个控件，
`tests/test_log_pages_geometry.py` 会在 3 档 DPI × 3 种窗口尺寸下遍历每个叶页，
控件被裁且不可滚动就判红。左边一列只读的清单 + 右边一个编辑器，控件数少一个量级，
而且"当前在编哪一条"一眼看得见。

取色有三条路：**常用色块**（一下点到，覆盖九成场合）、**取色盘**
（`colorchooser.askcolor`，系统对话框自带 HSV / 自定义色 / 最近用色）、
**十六进制输入框**（无显示环境要能手打，几何闸门和离线面板跑不了模态对话框）。
自绘 HSV 面板要么逐像素画 PhotoImage、要么引 Pillow，为一个"一次调好就不再动"的
功能不划算。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import colorchooser, ttk

from ..board_line_hooks import register_board_line_hook
from ..proto import parse_kv
from ..theme import UI_PALETTE


LED_MAP_TAB_TEXT = "状态灯"

# 分组只影响左侧清单的排版，与固件的绑定 ID 无关。
LED_GROUPS = (
    ("飞行状态", ("armed", "ready", "heartbeat")),
    ("解锁被拒 · 闪几下 = 原因码", (
        "block_no_rc", "block_rc_loss", "block_arm_switch", "block_throttle_high",
        "block_imu", "block_frame", "block_airframe",
    )),
    ("舵机标定", ("cal_released", "cal_save_ack", "cal_error")),
    ("光流", ("flow_starting", "flow_retrying", "flow_failed")),
)

# 界面上的人话。固件报的是 bind= 的英文名，两边靠这张表对起来；
# 表里没有的名字直接显示原名，不猜。
LED_BIND_LABEL = {
    "armed": "已解锁（桨会转）",
    "ready": "就绪，可以解锁",
    "heartbeat": "心跳（控制环还没发话）",
    "block_no_rc": "1 · 没收到过遥控",
    "block_rc_loss": "2 · 遥控链路丢了",
    "block_arm_switch": "3 · 解锁拨杆没打",
    "block_throttle_high": "4 · 油门没收到底",
    "block_imu": "5 · IMU 不健康",
    "block_frame": "6 · 坐标迁移未完成",
    "block_airframe": "7 · 没有机体模型",
    "cal_released": "扭矩已释放",
    "cal_save_ack": "保存成功",
    "cal_error": "标定出错",
    "flow_starting": "启动中",
    "flow_retrying": "重试中",
    "flow_failed": "失败",
}

LED_EFFECTS = ("off", "solid", "blink", "pulses", "breathe")

# 每种效果真正用得上的时间字段。别的字段对这个效果没有意义，
# 比较档位时不看它们，自定义行里也置灰。
LED_EFFECT_FIELDS = {
    "off": (),
    "solid": (),
    "blink": ("on", "off"),
    "pulses": ("on", "off", "gap"),
    "breathe": ("period", "dim"),
}

LED_FIELD_LABEL = {
    "on": "亮 (ms)", "off": "灭 (ms)", "gap": "组间停顿 (ms)",
    "period": "周期 (ms)", "dim": "谷底亮度 0-200",
}

LED_CUSTOM_RHYTHM = "自定义…"

# ── 节奏档位 ───────────────────────────────────────────────────────────
#
# 这十档**恰好覆盖固件出厂表用到的全部节奏**（App/Src/app_led_config.c 的
# led_default_binding），所以出厂状态下十六条绑定每一条都显示得出名字，
# 一条 `自定义…` 都不会出现。往表里加档位没问题；删档位或改毫秒数之前先想清楚：
# 出厂值会因此变成"自定义"，用户打开页面会以为是自己改过。
# tests/test_led_map_page.py 把两边对起来。
#
# 每一档都必须过得了固件的校验（App/Src/app_led_config.c 的 led_binding_valid）：
# blink/pulses 的亮灭不能为 0；pulses 的组间停顿 ≥ 400 ms 且 ≥ 2 倍的灭；
# breathe 的周期在 100..60000 ms、谷底亮度 ≤ 200。
LED_RHYTHMS = (
    ("熄灭",                   "off",     0,   0,   0,    0,  0),
    ("常亮",                   "solid",   0,   0,   0,    0,  0),
    ("慢呼吸 · 2.6 秒一个来回", "breathe", 0,   0,   0, 2600, 10),
    ("快呼吸 · 1.2 秒一个来回", "breathe", 0,   0,   0, 1200, 10),
    ("轻点一下 · 亮 0.12 秒",   "blink", 120, 380,   0,    0,  0),
    ("慢闪 · 0.5 秒",          "blink", 500, 500,   0,    0,  0),
    ("中闪 · 0.32 秒",         "blink", 320, 320,   0,    0,  0),
    ("快闪 · 0.16 秒",         "blink", 160, 160,   0,    0,  0),
    ("更快闪 · 0.12 秒",       "blink", 120, 120,   0,    0,  0),
    ("急闪 · 0.08 秒",         "blink",  80,  80,   0,    0,  0),
    ("数闪 · 几下 = 原因码",    "pulses",160, 160, 760,    0,  0),
)

LED_RHYTHM_LABELS = tuple(item[0] for item in LED_RHYTHMS) + (LED_CUSTOM_RHYTHM,)

LED_PULSES_RHYTHM = "数闪 · 几下 = 原因码"

# 解锁被拒那 7 条，顺序 = 原因码。
LED_BLOCK_BINDINGS = LED_GROUPS[1][1]


def pulse_count_of(name) -> int:
    """这条绑定用数闪时闪几下。**必须和固件的 `APP_LedConfig_PulseCount` 一致。**

    返回 0 表示"这条数不出次数"：固件把 count=0 的 PULSES 直接画成黑，所以它
    既不该出现在档位表里，波形也不该画出任何一下。这里报 1 而固件报 0 的后果，
    是屏幕上闪着而板子上是灭的。
    """
    if name in LED_BLOCK_BINDINGS:
        return LED_BLOCK_BINDINGS.index(name) + 1
    return 3 if name == "flow_failed" else 0

# 常用色。点一下就发出去——挑颜色这件事九成场合到此为止，不必开对话框。
LED_QUICK_COLOURS = (
    ("红", "#FF0000"), ("橙", "#FF6E00"), ("黄", "#FFD000"), ("绿", "#00FF00"),
    ("青", "#00C8FF"), ("蓝", "#0000FF"), ("紫", "#8A2BE2"), ("白", "#FFFFFF"),
)

SWATCH_WIDTH = 22
SWATCH_HEIGHT = 14
QUICK_SWATCH = 18
WAVE_WIDTH = 440
WAVE_HEIGHT = 40
WAVE_SPAN_MS = 4000


def _hex_of(red: int, green: int, blue: int) -> str:
    return f"#{red:02X}{green:02X}{blue:02X}"


def _parse_hex(text: str):
    """`#RRGGBB` / `RRGGBB` → (r, g, b)；认不出返回 None，不猜也不钳。"""
    value = text.strip().lstrip("#")
    if len(value) != 6:
        return None
    try:
        return (int(value[0:2], 16), int(value[2:4], 16), int(value[4:6], 16))
    except ValueError:
        return None


def rhythm_of(record) -> str | None:
    """一条绑定属于哪一档，认不出返回 None（界面显示 `自定义…`）。

    只比这个效果**用得上**的字段。举例：effect=solid 的那条，on/off 是多少都不影响
    灯长什么样，固件也不会去读；把它们算进比较，只会让一条本来就是"常亮"的绑定
    因为残留的历史毫秒数而显示成"自定义"。
    """
    if record is None:
        return None
    effect = record.get("effect")
    for label, candidate, on_ms, off_ms, gap_ms, period_ms, dim in LED_RHYTHMS:
        if candidate != effect:
            continue
        wanted = {"on": on_ms, "off": off_ms, "gap": gap_ms,
                  "period": period_ms, "dim": dim}
        used = LED_EFFECT_FIELDS.get(effect, ())
        if all(_int_of(record, key) == wanted[key] for key in used):
            return label
    return None


def _int_of(record: dict, key: str) -> int:
    try:
        return max(0, min(65535, int(record.get(key, "0"))))
    except (TypeError, ValueError):
        return 0


class LedMapPage:
    """页面的全部状态都挂在这个对象上，不往 panel 上撒属性。"""

    def __init__(self, panel, parent: ttk.Frame) -> None:
        self.panel = panel
        self.bindings: dict[str, dict] = {}
        self.order: list[str] = []
        self.selected: str | None = None
        self.dirty = False
        self.generation = None
        # 已经发出去、还没等到飞控回话的那一改。飞控回 applied_ram 才并进
        # self.bindings——在那之前界面上的清单仍然显示板子上真实的值。
        self.pending: tuple[str, dict] | None = None
        self.row_swatch: dict[str, tk.Canvas] = {}
        self.row_button: dict[str, ttk.Button] = {}
        self.row_summary: dict[str, tk.StringVar] = {}
        self.field_vars: dict[str, tk.StringVar] = {}
        self.field_widgets: dict[str, ttk.Entry] = {}
        self._build(parent)
        register_board_line_hook(panel, self.handle_board_line)

    # ------------------------------------------------------------ 构建

    def _build(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="状态灯颜色绑定",
                  style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=("同一时刻只有一盏灯、只说一件事。优先级从高到低："
                  "点名 → 已解锁 → 标定 → 解锁被拒 → 告警 → 就绪 → 心跳。"
                  "所以给两个不会同时出现的状态挑相近的颜色没关系，"
                  "真正要分开的是排在一起的那几条。"),
            style="Muted.TLabel", wraplength=820, justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(2, 8))

        body = ttk.Frame(parent)
        body.pack(fill=tk.BOTH, expand=True)
        self._build_list(body)
        self._build_editor(body)
        self._build_actions(parent)

        self.status_var = tk.StringVar(value="未读取。点「从飞控读取」。")
        ttk.Label(parent, textvariable=self.status_var,
                  style="Muted.TLabel", wraplength=820,
                  justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))

    def _build_list(self, parent: ttk.Frame) -> None:
        column = ttk.Frame(parent)
        column.pack(side=tk.LEFT, anchor=tk.N, padx=(0, 16))
        for title, names in LED_GROUPS:
            ttk.Label(column, text=title, style="Muted.TLabel").pack(
                anchor=tk.W, pady=(8, 2))
            for name in names:
                self.order.append(name)
                self._build_row(column, name)

    def _build_row(self, parent: ttk.Frame, name: str) -> None:
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=1)
        swatch = tk.Canvas(row, width=SWATCH_WIDTH, height=SWATCH_HEIGHT,
                           background=UI_PALETTE["console"], highlightthickness=1,
                           highlightbackground=UI_PALETTE["border"])
        swatch.pack(side=tk.LEFT, padx=(0, 6))
        button = ttk.Button(row, text=LED_BIND_LABEL.get(name, name), width=20,
                            style="Secondary.TButton",
                            command=lambda n=name: self.select(n))
        button.pack(side=tk.LEFT)
        summary = tk.StringVar(value="—")
        ttk.Label(row, textvariable=summary, style="Mono.TLabel").pack(
            side=tk.LEFT, padx=(6, 0))
        self.row_swatch[name] = swatch
        self.row_button[name] = button
        self.row_summary[name] = summary

    def _build_editor(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="编辑", padding=(10, 8))
        box.pack(side=tk.LEFT, anchor=tk.N, fill=tk.BOTH, expand=True)

        self.editor_title = tk.StringVar(value="左边选一条")
        ttk.Label(box, textvariable=self.editor_title).pack(anchor=tk.W, pady=(0, 8))

        colour_row = ttk.Frame(box)
        colour_row.pack(anchor=tk.W, fill=tk.X)
        ttk.Label(colour_row, text="颜色", width=5, style="Muted.TLabel").pack(
            side=tk.LEFT)
        self.big_swatch = tk.Canvas(colour_row, width=52, height=24,
                                    background=UI_PALETTE["console"],
                                    highlightthickness=1,
                                    highlightbackground=UI_PALETTE["border_strong"])
        self.big_swatch.pack(side=tk.LEFT, padx=(0, 10))
        self.quick_swatch: dict[str, tk.Canvas] = {}
        for label, value in LED_QUICK_COLOURS:
            chip = tk.Canvas(colour_row, width=QUICK_SWATCH, height=QUICK_SWATCH,
                             background=value, highlightthickness=1,
                             highlightbackground=UI_PALETTE["border"])
            chip.pack(side=tk.LEFT, padx=1)
            chip.bind("<Button-1>", lambda _event, v=value: self.use_colour(v))
            self.quick_swatch[label] = chip

        hex_row = ttk.Frame(box)
        hex_row.pack(anchor=tk.W, fill=tk.X, pady=(4, 0))
        ttk.Label(hex_row, text="", width=5, style="Muted.TLabel").pack(side=tk.LEFT)
        ttk.Button(hex_row, text="取色盘…", style="Secondary.TButton",
                   command=self.pick_colour).pack(side=tk.LEFT)
        self.hex_var = tk.StringVar(value="")
        entry = ttk.Entry(hex_row, textvariable=self.hex_var, width=10)
        entry.pack(side=tk.LEFT, padx=(8, 4))
        entry.bind("<Return>", lambda _event: self.apply_colour())
        ttk.Button(hex_row, text="应用", style="Secondary.TButton",
                   command=self.apply_colour).pack(side=tk.LEFT)

        rhythm_row = ttk.Frame(box)
        rhythm_row.pack(anchor=tk.W, fill=tk.X, pady=(10, 0))
        ttk.Label(rhythm_row, text="节奏", width=5, style="Muted.TLabel").pack(
            side=tk.LEFT)
        self.rhythm_var = tk.StringVar(value="常亮")
        self.rhythm_combo = ttk.Combobox(
            rhythm_row, textvariable=self.rhythm_var, state="readonly",
            values=list(LED_RHYTHM_LABELS), width=22)
        self.rhythm_combo.pack(side=tk.LEFT)
        self.rhythm_combo.bind("<<ComboboxSelected>>",
                               lambda _event: self.choose_rhythm())
        ttk.Button(rhythm_row, text="在板子上试一下", style="Secondary.TButton",
                   command=self.preview).pack(side=tk.LEFT, padx=(10, 0))

        self.custom_row = ttk.Frame(box)
        self._build_custom(self.custom_row)

        self.lock_var = tk.StringVar(value="")
        ttk.Label(box, textvariable=self.lock_var, style="Muted.TLabel",
                  wraplength=420, justify=tk.LEFT).pack(anchor=tk.W, pady=(8, 0))

        ttk.Label(box, text="4 秒预览（上：这一条；下：优先级相邻的那一条）",
                  style="Muted.TLabel").pack(anchor=tk.W, pady=(10, 2))
        self.wave = tk.Canvas(box, width=WAVE_WIDTH, height=WAVE_HEIGHT * 2 + 6,
                              background=UI_PALETTE["console"], highlightthickness=1,
                              highlightbackground=UI_PALETTE["border"])
        self.wave.pack(anchor=tk.W)

    def _build_custom(self, parent: ttk.Frame) -> None:
        """自定义毫秒。默认收起来——它是逃生口，不是主路径。"""
        effect_row = ttk.Frame(parent)
        effect_row.pack(anchor=tk.W, fill=tk.X, pady=(6, 0))
        ttk.Label(effect_row, text="效果", width=5, style="Muted.TLabel").pack(
            side=tk.LEFT)
        self.effect_var = tk.StringVar(value="solid")
        self.effect_combo = ttk.Combobox(effect_row, textvariable=self.effect_var,
                                         values=list(LED_EFFECTS), state="readonly",
                                         width=10)
        self.effect_combo.pack(side=tk.LEFT)
        self.effect_combo.bind("<<ComboboxSelected>>",
                               lambda _event: self.refresh_field_state())

        fields = ttk.Frame(parent)
        fields.pack(anchor=tk.W, pady=(4, 0))
        for index, key in enumerate(("on", "off", "gap", "period", "dim")):
            ttk.Label(fields, text=LED_FIELD_LABEL[key], style="Muted.TLabel").grid(
                row=index // 3, column=(index % 3) * 2, sticky=tk.W,
                padx=(0 if index % 3 == 0 else 10, 2), pady=1)
            var = tk.StringVar(value="0")
            widget = ttk.Entry(fields, textvariable=var, width=7)
            widget.grid(row=index // 3, column=(index % 3) * 2 + 1, sticky=tk.W, pady=1)
            self.field_vars[key] = var
            self.field_widgets[key] = widget

        ttk.Button(parent, text="应用自定义节奏", style="Secondary.TButton",
                   command=self.apply_rhythm).pack(anchor=tk.W, pady=(6, 0))

    def _build_actions(self, parent: ttk.Frame) -> None:
        actions = ttk.Frame(parent)
        actions.pack(fill=tk.X, pady=(10, 0))
        for label, command in (("从飞控读取", self.request_read),
                               ("恢复出厂颜色", self.reset_all),
                               ("保存到飞控", self.commit)):
            ttk.Button(actions, text=label, command=command,
                       style="Secondary.TButton").pack(side=tk.LEFT, padx=(0, 8))
        self.dirty_var = tk.StringVar(value="")
        ttk.Label(actions, textvariable=self.dirty_var,
                  style="Mono.TLabel").pack(side=tk.LEFT, padx=(8, 0))

    # ------------------------------------------------------------ 收发

    def send(self, text: str) -> bool:
        """出口。**必须**过面板的安全门，并在控制台留痕。

        以前这里先试 `panel._send_command`——那个方法在 DronePanel 上根本不存在
        （全仓只有 tools/flash_timing_capture.py 有个同名函数），于是恒定走裸发：
        V0 只读验收会话里、固件升级窗口里，点「保存到飞控」照样会触发一次同步的
        128 KB 扇区擦写，而控制台和证据记录里查不到这次写 Flash 曾经发生。
        """
        panel = self.panel
        transport = getattr(panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            self.status_var.set("未连接飞控，命令未发送（改动保留在界面上）")
            return False
        allowed = getattr(panel, "_validation_command_allowed", None)
        if callable(allowed) and not allowed(text):
            self.status_var.set(
                "面板挡下了这条命令：验收会话或固件升级正在进行，此时不该写 Flash。")
            return False
        append = getattr(panel, "_append", None)
        if callable(append):
            append(f"> {text}")
        return bool(transport.send_line(text))

    def request_read(self) -> None:
        if self.send("LEDMAP?"):
            self.status_var.set("已请求；等待飞控回表…")

    def handle_board_line(self, line: str) -> None:
        if not line.startswith("LEDMAP"):
            return
        values = parse_kv(line)
        if "bind" in values and "effect" in values:
            self._absorb_binding(values)
            return
        state = values.get("state")
        if state is None:
            return
        self.generation = values.get("gen", self.generation)
        self.dirty = values.get("dirty") == "1"
        self._refresh_dirty()
        if state == "status":
            self.status_var.set(
                f"已读取 {values.get('n', '?')} 条绑定"
                f"（飞控 gen={self.generation}，"
                f"{'已自定义' if values.get('customized') == '1' else '仍是出厂值'}）")
        elif state == "applied_ram":
            self._confirm_pending(values.get("bind"))
            self.status_var.set(
                f"{self._label(values.get('bind'))} 已在飞控上生效"
                f"（尚未保存，断电会丢）")
        elif state == "rejected":
            self._revert_pending()
            self.status_var.set(
                f"飞控拒绝了 {self._label(values.get('bind'))}："
                f"{self._reason_text(values.get('reason'))}")
        elif state == "armed_blocked":
            self._revert_pending()
            self.status_var.set("飞控已解锁，拒绝改灯色。先上锁再改。")
        elif state == "committed":
            self.status_var.set("已保存到飞控 Flash，断电不丢。")
        elif state == "commit_failed":
            self.status_var.set(f"保存失败 st={values.get('st', '?')}，配置仍在 RAM 里。")
        elif state == "reset_ram":
            self.pending = None
            self.status_var.set("已恢复出厂颜色（尚未保存）。")
            self.request_read()
        elif state == "preview":
            self.status_var.set(
                f"板子上正在试 {self._label(values.get('bind'))}，"
                f"{values.get('hold_ms', '?')} ms 后自动还原。")
        elif state == "invalid_usage":
            self._revert_pending()
            self.status_var.set(f"命令用法不对：{values.get('reason', '')}")

    def _confirm_pending(self, bind) -> None:
        """飞控说收下了，这才把改动并进清单。

        不在点击时就更新清单：那等于把"我按了"说成"它接受了"，而
        `conflicts_with_armed` 这类拒绝恰恰只在飞控那边才判得出来。
        """
        if self.pending is None:
            return
        name, patch = self.pending
        self.pending = None
        if bind is not None and bind != name:
            return
        record = dict(self.bindings.get(name, {}))
        record.update(patch)
        self.bindings[name] = record
        self._refresh_row(name)
        if self.selected == name:
            self._load_editor(name)
        self._draw_wave()

    def _revert_pending(self) -> None:
        """飞控拒了，把控件退回板子上真实的值。

        否则界面停在用户刚选的那一档、而板子还是旧的，两边就此分家——
        而用户下一眼看的是界面。
        """
        self.pending = None
        if self.selected is not None:
            self._load_editor(self.selected)

    @staticmethod
    def _label(name) -> str:
        return LED_BIND_LABEL.get(name, name or "?")

    @staticmethod
    def _reason_text(reason) -> str:
        return {
            "bad_bind": "不认识这个状态名",
            "bad_effect": "不认识这个效果",
            "bad_channel": "颜色要在 0..255",
            "bad_number": "数字不合法",
            "bad_period": "呼吸周期要在 100..60000 ms",
            "zero_timing": "亮/灭时长不能是 0（驱动会替你换成 1 ms，界面和实际就对不上了）",
            "gap_too_short": "组间停顿太短，数不清闪了几下",
            "dim_too_high": "谷底亮度上限 200",
            "conflicts_with_armed": "和「已解锁」长得一样，会让人以为没解锁而去装桨",
            "pulses_not_countable": "这一条数不出次数，配成数闪会一直不亮",
            "invalid": "整份配置没通过校验",
        }.get(reason, reason or "未说明")

    def _absorb_binding(self, values: dict) -> None:
        name = values.get("bind")
        if name not in self.row_swatch:
            return
        record = {key: values.get(key, "0") for key in
                  ("r", "g", "b", "on", "off", "gap", "period", "dim")}
        record["effect"] = values.get("effect", "solid")
        record["lock"] = values.get("lock", "-")
        self.bindings[name] = record
        self._refresh_row(name)
        if self.selected is None:
            self.select(name)
        elif self.selected == name:
            self._load_editor(name)
        self._draw_wave()

    # ------------------------------------------------------------ 显示

    @staticmethod
    def _int(record: dict, key: str) -> int:
        return _int_of(record, key)

    def _colour_of(self, name: str) -> str:
        record = self.bindings.get(name)
        if record is None:
            return UI_PALETTE["console"]
        return _hex_of(min(255, self._int(record, "r")),
                       min(255, self._int(record, "g")),
                       min(255, self._int(record, "b")))

    def _refresh_row(self, name: str) -> None:
        record = self.bindings.get(name)
        self.row_swatch[name].configure(background=self._colour_of(name))
        if record is None:
            self.row_summary[name].set("—")
            return
        self.row_summary[name].set(rhythm_of(record) or LED_CUSTOM_RHYTHM)

    def _refresh_dirty(self) -> None:
        self.dirty_var.set("● 已改未保存" if self.dirty else "")

    def select(self, name: str) -> None:
        previous = self.selected
        self.selected = name
        # 选中的那条换个按钮样式：16 行长得一模一样时，"我在编哪一条"只能靠
        # 编辑框标题去确认，而那需要把视线挪到另一列。
        if previous is not None and previous in self.row_button:
            self.row_button[previous].configure(style="Secondary.TButton")
        if name in self.row_button:
            self.row_button[name].configure(style="Primary.TButton")
        self._load_editor(name)
        self._draw_wave()

    def _load_editor(self, name: str) -> None:
        record = self.bindings.get(name)
        self.editor_title.set(f"{LED_BIND_LABEL.get(name, name)}   [{name}]")
        if record is None:
            self.lock_var.set("还没从飞控读到这一条。点「从飞控读取」。")
            return
        red, green, blue = (self._int(record, "r"), self._int(record, "g"),
                            self._int(record, "b"))
        self.hex_var.set(_hex_of(min(255, red), min(255, green), min(255, blue)))
        self.big_swatch.configure(background=self._colour_of(name))
        self.effect_var.set(record.get("effect", "solid"))
        for key in ("on", "off", "gap", "period", "dim"):
            self.field_vars[key].set(str(self._int(record, key)))
        self.refresh_field_state()
        # 数不出次数的绑定不提供「数闪」——固件会拒，界面就不该给出这个选项。
        self.rhythm_combo.configure(values=[
            label for label in LED_RHYTHM_LABELS
            if (label != LED_PULSES_RHYTHM) or (pulse_count_of(name) > 0)
        ])
        matched = rhythm_of(record)
        self.rhythm_var.set(matched or LED_CUSTOM_RHYTHM)
        self._show_custom(matched is None)
        if "count" not in (record.get("lock") or ""):
            self.lock_var.set("")
        elif name in LED_BLOCK_BINDINGS:
            self.lock_var.set(
                "闪几下 = 解锁被拒的原因码，固件写死，改不了——"
                "这样现场数出来的次数和 ARM? 报的 block= 永远对得上。颜色随便改。")
        else:
            self.lock_var.set(
                f"闪几下（{pulse_count_of(name)} 下）固件写死，改不了。颜色和快慢随便改。")

    def _show_custom(self, visible: bool) -> None:
        # winfo_manager() 而不是 winfo_ismapped()：页签没被选中时整页都不是 mapped，
        # 拿 ismapped 判断会误以为"还没摆上去"，于是每次都重新 pack 一遍。
        packed = self.custom_row.winfo_manager() != ""
        if visible and not packed:
            self.custom_row.pack(anchor=tk.W, fill=tk.X,
                                 after=self.rhythm_combo.master)
        elif packed and not visible:
            self.custom_row.pack_forget()

    def refresh_field_state(self) -> None:
        used = LED_EFFECT_FIELDS.get(self.effect_var.get(), ())
        for key, widget in self.field_widgets.items():
            widget.configure(state="normal" if key in used else "disabled")

    # ------------------------------------------------------------ 编辑

    def use_colour(self, value: str) -> None:
        """常用色块：填进输入框并立刻发出去。"""
        if self.selected is None:
            return
        self.hex_var.set(value)
        self.apply_colour()

    def pick_colour(self) -> None:
        if self.selected is None:
            return
        initial = self.hex_var.get() or "#808080"
        # parent= 必须传：测试进程里同时存在多个 DronePanel，不传会挂到错误的 root。
        chosen = colorchooser.askcolor(color=initial, parent=self.wave,
                                       title="挑一个颜色")
        if not chosen or chosen[1] is None:
            return
        parsed = _parse_hex(chosen[1])
        if parsed is None:
            return
        self.hex_var.set(_hex_of(*parsed))
        self.apply_colour()

    def apply_colour(self) -> None:
        if self.selected is None:
            return
        parsed = _parse_hex(self.hex_var.get())
        if parsed is None:
            self.status_var.set("颜色要写成 #RRGGBB，例如 #FF6E00。没有发送。")
            return
        self.big_swatch.configure(background=_hex_of(*parsed))
        patch = {"r": str(parsed[0]), "g": str(parsed[1]), "b": str(parsed[2])}
        if self.send(f"LEDMAP SET {self.selected} {parsed[0]} {parsed[1]} {parsed[2]}"):
            self.pending = (self.selected, patch)

    def choose_rhythm(self) -> None:
        """选了一档就直接发。挑档位本身就是"我要这个"，不该再按一次确认。"""
        if self.selected is None:
            return
        label = self.rhythm_var.get()
        if label == LED_CUSTOM_RHYTHM:
            # 自定义不自动发：毫秒数还没填完就发出去，只会连着收几条拒绝。
            self._show_custom(True)
            self.status_var.set("填好毫秒数后点「应用自定义节奏」。")
            return
        self._show_custom(False)
        for item in LED_RHYTHMS:
            if item[0] == label:
                self.effect_var.set(item[1])
                for key, value in zip(("on", "off", "gap", "period", "dim"), item[2:]):
                    self.field_vars[key].set(str(value))
                self.refresh_field_state()
                self._send_rhythm(item[1], item[2:])
                return

    def apply_rhythm(self) -> None:
        if self.selected is None:
            return
        try:
            numbers = [int(self.field_vars[key].get())
                       for key in ("on", "off", "gap", "period", "dim")]
        except (TypeError, ValueError):
            self.status_var.set("时间要填整数。没有发送。")
            return
        self._send_rhythm(self.effect_var.get(), numbers)

    def _send_rhythm(self, effect: str, numbers) -> None:
        numbers = list(numbers)
        patch = {"effect": effect}
        patch.update({key: str(value) for key, value
                      in zip(("on", "off", "gap", "period", "dim"), numbers)})
        if self.send("LEDMAP RHYTHM {} {} {} {} {} {} {}".format(
                self.selected, effect, *numbers)):
            self.pending = (self.selected, patch)

    def preview(self) -> None:
        if self.selected is not None:
            self.send(f"LEDMAP PREVIEW {self.selected}")

    def reset_all(self) -> None:
        self.send("LEDMAP RESET")

    def commit(self) -> None:
        self.send("LEDMAP COMMIT")

    # ------------------------------------------------------------ 波形

    def _neighbour(self) -> str | None:
        """优先级上挨着当前这条的另一条——最可能被看混的就是它。"""
        if self.selected is None or self.selected not in self.order:
            return None
        index = self.order.index(self.selected)
        if index + 1 < len(self.order):
            return self.order[index + 1]
        return self.order[index - 1] if index > 0 else None

    def _draw_wave(self) -> None:
        self.wave.delete("all")
        for slot, name in enumerate((self.selected, self._neighbour())):
            if name is None or name not in self.bindings:
                continue
            top = slot * (WAVE_HEIGHT + 6)
            self._draw_one(name, top)

    def _draw_one(self, name: str, top: int) -> None:
        record = self.bindings[name]
        colour = self._colour_of(name)
        count = pulse_count_of(name)
        self.wave.create_text(4, top + 6, anchor=tk.W, fill=UI_PALETTE["muted"],
                              text=f"{LED_BIND_LABEL.get(name, name)}"
                                   f"  ·  {rhythm_of(record) or LED_CUSTOM_RHYTHM}",
                              font=("TkDefaultFont", 7))
        # 每毫秒一格太细，按像素反算时间即可：横轴固定 4 秒。
        for x in range(WAVE_WIDTH):
            now_ms = (x * WAVE_SPAN_MS) // WAVE_WIDTH
            if self._lit(record, now_ms, count):
                self.wave.create_line(x, top + 12, x, top + WAVE_HEIGHT - 2,
                                      fill=colour)

    def _lit(self, record: dict, now_ms: int, count: int = 1) -> bool:
        """复刻 `drv_rgb_led.c` 的相位判断。

        只算"亮不亮"，不算亮度——呼吸在这条 1 像素宽的时间轴上画不出层次，
        画成"全亮"反而会让人以为它和常亮一样。呼吸单独用虚线密度表示。
        """
        effect = record.get("effect")
        if effect in (None, "off"):
            return False
        if effect == "solid":
            return True
        if effect == "blink":
            on_ms = max(1, self._int(record, "on"))
            off_ms = max(1, self._int(record, "off"))
            return (now_ms % (on_ms + off_ms)) < on_ms
        if effect == "pulses":
            on_ms = max(1, self._int(record, "on"))
            off_ms = max(1, self._int(record, "off"))
            gap_ms = self._int(record, "gap")
            burst = count * (on_ms + off_ms)
            cycle = burst + gap_ms
            if cycle <= 0 or count == 0:
                return False
            phase = now_ms % cycle
            if phase >= burst:
                return False
            return (phase % (on_ms + off_ms)) < on_ms
        if effect == "breathe":
            period = max(1, self._int(record, "period"))
            phase = now_ms % period
            half = period // 2 or 1
            level = phase / half if phase < half else (period - phase) / half
            # 用"点亮的疏密"表示亮度：每 4 px 判一次，越亮越密。
            return ((now_ms // 8) % 4) < max(0, min(4, round(level * 4)))
        return False

    def _preview_count(self, name: str | None = None) -> int:
        """这一行画几下。

        取的是**这一行自己**的次数，不是当前选中项的。并排两条波形的唯一用途
        就是"6 下和 7 下一眼分得清"，拿选中项的次数去画邻行，等于把对比图
        画成两条一模一样的——预览直接骗人，而它正是用来判断分不分得开的。
        """
        return pulse_count_of(name if name is not None else self.selected)


def mount_led_map(panel, parent: ttk.Frame) -> LedMapPage:
    page = LedMapPage(panel, parent)
    panel.led_map_page = page
    return page
