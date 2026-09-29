"""「校准 · 桨叶与电机方向」页：把偏航极性从推测变成量出来的事实。

**这一页解决的是什么问题。** 偏航力矩极性以前来自机体模型里的
`lower_rotor_spin_sense`，而那个值是**从调参现象反推的**——作者观察到偏航角速度
环 Kp 加大会抖振而不是发散，抖振说明闭环是负反馈，于是反推出下桨俯视顺时针。
推理自洽，但它证明的是"整条链的符号彼此不矛盾"，不是"桨真的往那边转"：
换一套增益、或者把某处符号和它一起翻过来，现象一模一样，而飞机的偏航方向已经反了。

所以这一页让人**通电看一眼**：哪个 ESC 通道接的是上桨、哪个是下桨，每一路往哪边转。
填完之后偏航极性由它推出（极性 = −下桨旋向），与任何 PID 的正负无关。

**飞控是唯一权威。** 本页只是编辑器：所有值都从 `PROPCAL?` 读回来，改动先进飞控
RAM，确认满意了再 `PROPCAL COMMIT` 落 Flash。本页不在 panel_state.json 里缓存飞控
的值——缓存一份就会出现"界面显示的和板子上的不一样"，而偏航方向正是最不能出这种
问题的地方。

**点电机是靠心跳续命的。** "发一条命令转 3 秒然后自己停"这种写法，在上位机崩溃、
USB 拔掉、笔记本睡眠或者人转身走开时，电机照转不误——因为停下来不需要任何人还活着。
心跳把它反过来：**转下去才需要有人活着。**本页每 100 ms 重发一次油门命令，飞控
超过 300 ms 没听到就关窗；而那个超时判定跑在飞控的 500 Hz 控制环里，不在收命令的
任务里（见 `App/Src/app_prop_spin.c`）。所以关掉这个窗口、拔掉线、杀掉这个进程，
结果都一样是停。
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk

from ..board_line_hooks import register_board_line_hook
from ..proto import parse_kv, safe_int
from ..telem_subscription import TELEM_OWNER_PROP_MAP


PROP_MAP_TAB_TEXT = "桨叶与电机方向"

# 心跳周期。飞控那边的超时是 300 ms，留三拍余量：Tk 的 after 回调在重绘忙时会
# 晚到，周期贴着阈值设会让窗口在正常使用中随机关闭，而"随机停"会训练人去忽略它。
HEARTBEAT_MS = 100

# 与固件 `PROPCAL SPIN ARM confirm=` 的确认词一致。改这里要连固件一起改，
# 否则界面点了没反应而飞控只回一句 need_confirm。
SPIN_CONFIRM_TOKEN = "safe"

# 本次开窗油门上限的合法范围，对应固件 `PROPCAL SPIN ARM max_pct=`。越界或非
# 整数一律本地拦截，不发一条让飞控用 reason=bad_number 拒绝的命令——省一次
# 回合，也不会让人以为点了按钮却什么都没发生。
SPIN_MAX_PCT_MIN = 1
SPIN_MAX_PCT_MAX = 100
SPIN_MAX_PCT_DEFAULT = 20

# 固件出厂默认上限是 20%：超过这个值，油门已经能产生可观推力，界面必须提醒
# 先确认桨叶已拆或机体已固定，而不是让人凭感觉去拉这个输入框。
SPIN_MAX_PCT_WARN_ABOVE = 20

# 实时读数 · 总线两路。走遥测流仲裁器登记，通道名是契约（与 power.py 同名同源）。
PROP_MAP_CHANNEL_VOLTAGE = "batt_v"
PROP_MAP_CHANNEL_CURRENT = "batt_i"
PROP_MAP_BUS_CHANNELS = (PROP_MAP_CHANNEL_VOLTAGE, PROP_MAP_CHANNEL_CURRENT)

# 多久没有新遥测帧就认定总线读数不能再显示。与 power.py 取值一致：40 Hz 的流
# 断 1.5 s 等于丢 60 帧，不是抖动是真的停了。
PROP_MAP_TELEM_STALE_S = 1.5

# 界面刷新节拍：只把缓冲里已有的值画出来并同步订阅可见性，不发送任何周期性
# 命令——数据来自遥测流推送，这里不轮询，和点电机的心跳定时器彼此独立。
PROP_MAP_RENDER_MS = 250

ROLE_LABEL = {"upper": "上桨", "lower": "下桨", "-": "（未填）"}
ROLE_FROM_LABEL = {v: k for k, v in ROLE_LABEL.items() if k != "-"}

# "俯视"这三个字是契约的一部分：同一个电机从上面看是顺时针、从下面看就是逆时针，
# 而填这个值的人手里拿着的是一架正放在桌上的飞机。
SPIN_LABEL = {
    "cw": "俯视顺时针 (CW)",
    "ccw": "俯视逆时针 (CCW)",
    "-": "（未填）",
}
SPIN_FROM_LABEL = {v: k for k, v in SPIN_LABEL.items() if k != "-"}

STATE_TEXT = {
    "status": "",
    "applied_ram": "两路都填好了，飞控已按新映射工作（尚未保存，断电会丢）。",
    "partial": "还没填全：飞控此刻按「未标定」处理，偏航通道没有权限。",
    "conflict": "两路互相矛盾，飞控拒绝采用（详见下方结论）。",
    "reset_ram": "已清空标定（尚未保存）。",
    "committed": "已保存到飞控 Flash，断电不丢。",
    "write_blocked": "飞控拒绝改映射。",
    "rejected": "飞控拒绝了这次修改。",
    "invalid_usage": "命令用法不对。",
    "commit_rejected": "还不能保存。",
}

REASON_TEXT = {
    "armed": "飞机已解锁，先上锁再改。",
    "spin_open": "点电机窗口开着，先停下来再改。",
    "bad_channel": "通道号不对。",
    "bad_role": "上下桨没选。",
    "bad_spin": "旋向没选。",
    "incomplete": "两路都填完才能保存。",
    "conflict": "两路互相矛盾：上下桨不能都选同一个，旋向必须一正一反。",
    "need_confirm": "没有确认现场安全。",
    "not_open": "点电机窗口没开。",
    "bad_number": "数值不合法。",
    "acceptance_active": "验收流程正在占用执行器。",
    "servo_cal_active": "舵机标定正在占用执行器。",
    "ident_running": "系统辨识正在占用执行器。",
    "heartbeat_lost": "心跳中断（连接断了或界面卡住），飞控已断开电机。",
    "inhibited": "解锁/验收/标定抢走了执行器，窗口已关闭。",
    "rejected": "命令不合法，窗口已关闭。",
    "request": "已停止。",
    "none": "",
    "unknown": "",
}


class _TelemStamp:
    """最近一帧遥测的到达证据。**在收线程里写**，Tk 线程只读。

    做法与 `pages/power.py` 一致：只记到达时刻和链路身份，不在这里解码——
    解码已经由工作台的消费者做过，值从同一个环形缓冲里取。这里只回答
    "这一帧是不是**当前这条链路**刚送来的"：重连之后旧链路补送的最后一帧，
    不能被当成新会话的数据显示出来。
    """

    __slots__ = ("at", "transport", "generation")

    def __init__(self, at: float, transport, generation) -> None:
        self.at = at
        self.transport = transport
        self.generation = generation

    def is_current(self, transport) -> bool:
        return (self.transport is transport
                and self.generation == getattr(transport, "connection_generation", 0))


class PropMapPage:
    """页面的全部状态都挂在这个对象上，不往 panel 上撒属性。"""

    def __init__(self, panel, parent: ttk.Frame) -> None:
        self.panel = panel
        # 飞控回来的真值。界面上的下拉框是编辑草稿，两者分开——后台回读不该
        # 覆盖正在编辑的草稿，而草稿也不该冒充板子上的事实。
        self.channels: dict[int, dict[str, str]] = {}
        self.calibrated = False
        self.dirty = False
        self.generation = None
        self.upper_channel = 0
        self.lower_channel = 0
        self.lower_spin = "-"
        self.yaw_polarity = "0"

        self.spin_active = False
        self.spin_stop_reason = "none"
        self._timer = None
        self._disposed = False

        # 实时读数 · 总线（遥测流）的订阅状态与到达证据；电调那一半跟着
        # PROPCAL spin= 回包走，不需要单独的状态，见 _absorb_esc_telem。
        self.subscribed = False
        self.telem_stamp: _TelemStamp | None = None
        self._telem_timer = None

        self.role_vars: dict[int, tk.StringVar] = {}
        self.spin_vars: dict[int, tk.StringVar] = {}
        self.pad_vars: dict[int, tk.StringVar] = {}
        self._suppress_trace = False

        self._build(parent)
        register_board_line_hook(panel, self.handle_board_line)
        panel.bind("<Destroy>", self._on_destroy, add="+")
        # 立即画一次（多半是"没订阅"），随后自己按节拍续下去——做法和
        # power.py 的 `self._tick()` 一致，不新开一套不一样的机制。
        self._telem_tick()

    # ------------------------------------------------------------ 构建

    def _build(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="桨叶与电机接线标定",
                  style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=("偏航极性由这里推出（极性 = −下桨旋向），"
                  "不再靠 PID 的正负反推。两路都填完之前，飞控按「未标定」处理："
                  "偏航通道没有权限，而不是朝一个猜出来的方向使劲。"),
            style="Muted.TLabel", wraplength=820, justify=tk.LEFT,
        ).pack(anchor=tk.W, pady=(2, 8))

        self._build_channels(parent)
        self._build_conclusion(parent)
        self._build_spin(parent)
        self._build_live(parent)
        self._build_actions(parent)

        self.status_var = tk.StringVar(value="未读取。点「从飞控读取」。")
        ttk.Label(parent, textvariable=self.status_var, style="Muted.TLabel",
                  wraplength=820, justify=tk.LEFT).pack(anchor=tk.W, pady=(6, 0))

    def _build_channels(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="每一路 ESC 通道接的是哪个电机", padding=8)
        box.pack(fill=tk.X)
        ttk.Label(box, text="焊盘名由飞控报出（BSP 里的板级事实），不是界面猜的。",
                  style="Muted.TLabel").grid(row=0, column=0, columnspan=5,
                                             sticky=tk.W, pady=(0, 6))
        for channel in (1, 2):
            row = channel
            self.pad_vars[channel] = tk.StringVar(value=f"通道 {channel}")
            ttk.Label(box, textvariable=self.pad_vars[channel],
                      style="Mono.TLabel").grid(row=row, column=0, sticky=tk.W,
                                                padx=(0, 12), pady=2)

            self.role_vars[channel] = tk.StringVar(value=ROLE_LABEL["-"])
            role = ttk.Combobox(box, textvariable=self.role_vars[channel], width=10,
                                state="readonly",
                                values=[ROLE_LABEL["upper"], ROLE_LABEL["lower"]])
            role.grid(row=row, column=1, sticky=tk.W, padx=(0, 12), pady=2)
            role.bind("<<ComboboxSelected>>",
                      lambda _e, ch=channel: self._send_channel(ch))

            self.spin_vars[channel] = tk.StringVar(value=SPIN_LABEL["-"])
            spin = ttk.Combobox(box, textvariable=self.spin_vars[channel], width=20,
                                state="readonly",
                                values=[SPIN_LABEL["cw"], SPIN_LABEL["ccw"]])
            spin.grid(row=row, column=2, sticky=tk.W, padx=(0, 12), pady=2)
            spin.bind("<<ComboboxSelected>>",
                      lambda _e, ch=channel: self._send_channel(ch))

            ttk.Button(box, text=f"点这一路 →",
                       command=lambda ch=channel: self._select_spin_channel(ch)).grid(
                row=row, column=3, sticky=tk.W, pady=2)

    def _build_conclusion(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="飞控据此得到的结论", padding=8)
        box.pack(fill=tk.X, pady=(8, 0))
        self.conclusion_var = tk.StringVar(value="尚未读取")
        ttk.Label(box, textvariable=self.conclusion_var, style="Mono.TLabel",
                  wraplength=800, justify=tk.LEFT).pack(anchor=tk.W)

    def _build_spin(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="通电点一下电机（看它往哪边转）", padding=8)
        box.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            box,
            text=("靠心跳续命：本页每 100 ms 重发一次油门，飞控超过 300 ms 没听到"
                  "就立刻断电。关掉窗口、拔线、杀进程，结果都一样是停。"),
            style="Muted.TLabel", wraplength=780, justify=tk.LEFT,
        ).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 6))

        self.confirm_var = tk.BooleanVar(value=False)
        self.confirm_check = ttk.Checkbutton(
            box, variable=self.confirm_var,
            text="我已确认现场安全：桨已拆或周围已清场，人和物都不在旋转面内",
            command=self._refresh_spin_controls)
        self.confirm_check.grid(row=1, column=0, columnspan=4, sticky=tk.W)

        # 本次窗口的油门上限：默认 20，可调到 100。固件的 ARM 只降不升——窗口
        # 开着时改这个数没有意义，所以开窗之后就把输入框锁住（见
        # _refresh_spin_controls）；想抬高上限必须先停止再重新开窗，界面不能
        # 让人以为改了数字就立刻对已经开着的窗口生效。
        limit_row = ttk.Frame(box)
        limit_row.grid(row=2, column=0, columnspan=4, sticky=tk.W, pady=(6, 0))
        ttk.Label(limit_row,
                  text=f"本次上限（{SPIN_MAX_PCT_MIN}-{SPIN_MAX_PCT_MAX}%）："
                  ).pack(side=tk.LEFT)
        self.spin_max_pct_var = tk.StringVar(value=str(SPIN_MAX_PCT_DEFAULT))
        self.spin_max_pct_spinbox = ttk.Spinbox(
            limit_row, from_=SPIN_MAX_PCT_MIN, to=SPIN_MAX_PCT_MAX, width=5,
            textvariable=self.spin_max_pct_var, justify=tk.RIGHT,
            style="Numeric.TSpinbox")
        self.spin_max_pct_spinbox.pack(side=tk.LEFT, padx=(4, 12))
        self.max_pct_warning_var = tk.StringVar(value="")
        ttk.Label(limit_row, textvariable=self.max_pct_warning_var,
                  style="Warn.TLabel", wraplength=560,
                  justify=tk.LEFT).pack(side=tk.LEFT)
        # 用 StringVar 而不是 IntVar：手动敲键盘会经过"空字符串""半个数字"这些
        # 中间态，IntVar.get() 在这些中间态上直接抛 TclError。校验统一放
        # _parsed_max_pct，这里只负责把警示语跟着输入实时刷新。
        self.spin_max_pct_var.trace_add("write", self._on_max_pct_var_changed)

        self.spin_channel_var = tk.IntVar(value=1)
        picker = ttk.Frame(box)
        picker.grid(row=3, column=0, sticky=tk.W, pady=(6, 0))
        for channel in (1, 2):
            ttk.Radiobutton(picker, text=f"通道 {channel}", value=channel,
                            variable=self.spin_channel_var).pack(side=tk.LEFT,
                                                                 padx=(0, 10))

        self.throttle_var = tk.IntVar(value=0)
        self.max_percent = 20
        self.throttle_scale = ttk.Scale(box, from_=0, to=self.max_percent,
                                        orient=tk.HORIZONTAL, length=240,
                                        command=self._on_throttle)
        self.throttle_scale.grid(row=3, column=1, sticky=tk.W, padx=(10, 8),
                                 pady=(6, 0))
        self.throttle_label_var = tk.StringVar(value="油门 0%")
        ttk.Label(box, textvariable=self.throttle_label_var,
                  style="Mono.TLabel").grid(row=3, column=2, sticky=tk.W, pady=(6, 0))

        buttons = ttk.Frame(box)
        buttons.grid(row=4, column=0, columnspan=4, sticky=tk.W, pady=(8, 0))
        self.start_button = ttk.Button(buttons, text="开始通电",
                                       command=self.start_spin, state=tk.DISABLED)
        self.start_button.pack(side=tk.LEFT)
        self.stop_button = ttk.Button(buttons, text="停止", command=self.stop_spin,
                                      state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, padx=(8, 0))

        self.spin_status_var = tk.StringVar(value="未通电")
        ttk.Label(box, textvariable=self.spin_status_var, style="Mono.TLabel",
                  wraplength=780, justify=tk.LEFT).grid(
            row=5, column=0, columnspan=4, sticky=tk.W, pady=(6, 0))

    def _build_live(self, parent: ttk.Frame) -> None:
        """实时读数：总线（遥测流）+ 电调（DShot EDT 回传）。

        摆在点电机的正下面，是因为这一页存在的直接理由就是"边拉油门边看电流"
        ——原来必须切到电源页才看得到总电流，来回切页在这种需要盯着现场的
        操作里本身就是一种风险。
        """
        box = ttk.LabelFrame(parent, text="实时读数", padding=8)
        box.pack(fill=tk.X, pady=(8, 0))

        bus = ttk.Frame(box)
        bus.pack(fill=tk.X)
        ttk.Label(bus, text="总线（遥测流推送）", style="Eyebrow.TLabel").grid(
            row=0, column=0, columnspan=4, sticky=tk.W)
        self.bus_voltage_var = tk.StringVar(value="—")
        self.bus_current_var = tk.StringVar(value="—")
        ttk.Label(bus, text="总电压").grid(row=1, column=0, sticky=tk.W,
                                         padx=(0, 6), pady=(4, 0))
        ttk.Label(bus, textvariable=self.bus_voltage_var,
                  style="Mono.TLabel").grid(row=1, column=1, sticky=tk.W,
                                            padx=(0, 24), pady=(4, 0))
        ttk.Label(bus, text="总电流").grid(row=1, column=2, sticky=tk.W,
                                         padx=(0, 6), pady=(4, 0))
        ttk.Label(bus, textvariable=self.bus_current_var,
                  style="Mono.TLabel").grid(row=1, column=3, sticky=tk.W,
                                            pady=(4, 0))
        ttk.Label(bus, text="电流是固件里 25 点块平均，不是瞬时值。",
                  style="Muted.TLabel").grid(row=2, column=0, columnspan=4,
                                             sticky=tk.W, pady=(2, 0))
        self.bus_state_var = tk.StringVar(value="等待遥测流")
        ttk.Label(bus, textvariable=self.bus_state_var, style="Muted.TLabel",
                  wraplength=760, justify=tk.LEFT).grid(
            row=3, column=0, columnspan=4, sticky=tk.W, pady=(4, 0))

        esc = ttk.Frame(box)
        esc.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(esc, text="电调（DShot EDT，跟随心跳回包，10 Hz）",
                  style="Eyebrow.TLabel").grid(row=0, column=0, columnspan=5,
                                               sticky=tk.W)
        self.esc_current_vars: dict[int, tk.StringVar] = {}
        self.esc_erpm_vars: dict[int, tk.StringVar] = {}
        for offset, channel in enumerate((1, 2)):
            row = 1 + offset
            self.esc_current_vars[channel] = tk.StringVar(value="—")
            self.esc_erpm_vars[channel] = tk.StringVar(value="—")
            ttk.Label(esc, text=f"通道 {channel}").grid(
                row=row, column=0, sticky=tk.W, padx=(0, 10), pady=2)
            ttk.Label(esc, text="电流").grid(row=row, column=1, sticky=tk.E,
                                            padx=(0, 4), pady=2)
            ttk.Label(esc, textvariable=self.esc_current_vars[channel],
                      style="Mono.TLabel").grid(row=row, column=2, sticky=tk.W,
                                                padx=(0, 16), pady=2)
            ttk.Label(esc, text="电转速").grid(row=row, column=3, sticky=tk.E,
                                             padx=(0, 4), pady=2)
            ttk.Label(esc, textvariable=self.esc_erpm_vars[channel],
                      style="Mono.TLabel").grid(row=row, column=4, sticky=tk.W,
                                                pady=2)
        self.esc_state_var = tk.StringVar(value="尚未收到心跳回包")
        ttk.Label(esc, textvariable=self.esc_state_var, style="Muted.TLabel",
                  wraplength=760, justify=tk.LEFT).grid(
            row=3, column=0, columnspan=5, sticky=tk.W, pady=(4, 0))
        ttk.Label(
            esc,
            text=("默认构建是单向 DShot300：esc_telem 恒为 0，电调不会回传"
                  "电流/转速。EDT 电流分辨率为 1 A/LSB，小电流下恒为 0，只能当"
                  "高油门下的粗略旁证；这里给的是电转速 (eRPM)，换算成机械转速"
                  "要除以电机极对数，固件没有这个配置，所以不在这里折算。"),
            style="Muted.TLabel", wraplength=760, justify=tk.LEFT,
        ).grid(row=4, column=0, columnspan=5, sticky=tk.W, pady=(2, 0))

        # 解码计数。没有它，"回不来数"只是一个 "—"，而"根本没收到"和"收到了
        # 解不开"要查的是完全不同的东西（见固件 propcal_report_esc_diag 的注释）。
        # 计数是慢变量，固件只在开窗/停止/裸查询时发，所以这里给一个按钮而不是
        # 跟着心跳刷——跑一轮前后各点一次，看哪个计数在涨。
        self.esc_diag_var = tk.StringVar(value="尚未读取")
        ttk.Label(esc, textvariable=self.esc_diag_var, style="Mono.TLabel",
                  wraplength=760, justify=tk.LEFT).grid(
            row=5, column=0, columnspan=5, sticky=tk.W, pady=(6, 0))
        self.esc_diag_hint_var = tk.StringVar(value="")
        ttk.Label(esc, textvariable=self.esc_diag_hint_var, style="Warn.TLabel",
                  wraplength=760, justify=tk.LEFT).grid(
            row=6, column=0, columnspan=5, sticky=tk.W, pady=(2, 0))
        esc_buttons = ttk.Frame(esc)
        esc_buttons.grid(row=7, column=0, columnspan=5, sticky=tk.W, pady=(6, 0))
        ttk.Button(esc_buttons, text="读取电调解码计数",
                   command=self.request_esc_diag).pack(side=tk.LEFT)
        # 逐路电流属于 EDT，而 EDT 只能由飞控发一条 DShot 特殊命令打开——它不是
        # 电调配置器里的开关。命令号固定在固件里（只有开/关两条），界面这边给不了
        # 任意命令号：1..47 里躺着改转向和写电调 Flash 的命令。
        ttk.Button(esc_buttons, text="打开电调扩展遥测 (EDT)",
                   command=lambda: self.request_edt("ON")).pack(side=tk.LEFT,
                                                                padx=(8, 0))
        ttk.Button(esc_buttons, text="关闭",
                   command=lambda: self.request_edt("OFF")).pack(side=tk.LEFT,
                                                                 padx=(8, 0))
        self.edt_var = tk.StringVar(value="")
        ttk.Label(esc, textvariable=self.edt_var, style="Mono.TLabel",
                  wraplength=760, justify=tk.LEFT).grid(
            row=8, column=0, columnspan=5, sticky=tk.W, pady=(4, 0))

    def _build_actions(self, parent: ttk.Frame) -> None:
        actions = ttk.Frame(parent)
        actions.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(actions, text="从飞控读取",
                   command=self.request_read).pack(side=tk.LEFT)
        ttk.Button(actions, text="清空标定",
                   command=self.reset).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(actions, text="保存到飞控 Flash",
                   command=self.commit).pack(side=tk.LEFT, padx=(8, 0))
        self.dirty_var = tk.StringVar(value="")
        ttk.Label(actions, textvariable=self.dirty_var,
                  style="Mono.TLabel").pack(side=tk.LEFT, padx=(12, 0))

    # ------------------------------------------------------------ 收发

    def send(self, text: str) -> bool:
        """出口。**必须**过面板的安全门，并在控制台留痕。"""
        panel = self.panel
        transport = getattr(panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            self.status_var.set("未连接飞控，命令未发送。")
            return False
        allowed = getattr(panel, "_validation_command_allowed", None)
        if callable(allowed) and not allowed(text):
            self.status_var.set(
                "面板挡下了这条命令：验收会话或固件升级正在进行。")
            return False
        append = getattr(panel, "_append", None)
        if callable(append):
            append(f"> {text}")
        return bool(transport.send_line(text))

    def request_read(self) -> None:
        if self.send("PROPCAL?"):
            self.status_var.set("已请求；等待飞控回表…")

    def reset(self) -> None:
        self.send("PROPCAL RESET")

    def commit(self) -> None:
        self.send("PROPCAL COMMIT")

    def _send_channel(self, channel: int) -> None:
        """把这一路的两项一起发出去。

        两项合成一条命令而不是改一项发一条：中间态（角色改了、旋向还没改）会让
        飞控先判一次 conflict 再判一次 applied，界面上闪过一句"两路互相矛盾"，
        而用户其实什么都没做错。
        """
        if self._suppress_trace:
            return
        role = ROLE_FROM_LABEL.get(self.role_vars[channel].get())
        spin = SPIN_FROM_LABEL.get(self.spin_vars[channel].get())
        if role is None and spin is None:
            return
        parts = [f"PROPCAL SET ch={channel}"]
        if role is not None:
            parts.append(f"role={role}")
        if spin is not None:
            parts.append(f"spin={spin}")
        self.send(" ".join(parts))

    # ------------------------------------------------------------ 点电机

    def _refresh_spin_controls(self) -> None:
        ready = bool(self.confirm_var.get()) and not self.spin_active
        self.start_button.configure(state=tk.NORMAL if ready else tk.DISABLED)
        self.stop_button.configure(
            state=tk.NORMAL if self.spin_active else tk.DISABLED)
        # 固件的 ARM 只降不升：窗口开着时把这个框松开会让界面和飞控各说各话——
        # 界面写着 60、飞控其实还是开窗那一刻定的 20。想调高必须先停止再重新
        # 开窗，所以窗口开着的这段时间直接把输入框锁住，不留"改了没生效"的空子。
        self.spin_max_pct_spinbox.configure(
            state=tk.DISABLED if self.spin_active else "normal")

    def _parsed_max_pct(self) -> int | None:
        """上限输入框的文本转成合法整数；越界或非数字一律 None，不猜也不夹断。"""
        text = self.spin_max_pct_var.get().strip()
        try:
            value = int(text)
        except ValueError:
            return None
        if not (SPIN_MAX_PCT_MIN <= value <= SPIN_MAX_PCT_MAX):
            return None
        return value

    def _on_max_pct_var_changed(self, *_args) -> None:
        value = self._parsed_max_pct()
        if value is not None and value > SPIN_MAX_PCT_WARN_ABOVE:
            self.max_pct_warning_var.set(
                f"超过 {SPIN_MAX_PCT_WARN_ABOVE}% 已经能产生可观推力："
                "开始通电前必须先确认桨叶已拆除，或者机体已经牢固固定。")
        else:
            self.max_pct_warning_var.set("")

    def _on_throttle(self, value) -> None:
        percent = max(0, min(self.max_percent, int(float(value))))
        self.throttle_var.set(percent)
        self.throttle_label_var.set(f"油门 {percent}%")

    def _select_spin_channel(self, channel: int) -> None:
        """「点这一路 →」只是把选择挪过来，**不会**顺手通电。

        一个按钮既选通道又让电机转，意味着误点一下桨就转了。开始通电永远是
        单独的一步，而且要先勾确认。
        """
        self.spin_channel_var.set(channel)
        self.status_var.set(
            f"已选中通道 {channel}。勾上确认后点「开始通电」，再拉油门。")

    def start_spin(self) -> None:
        if not self.confirm_var.get():
            self.spin_status_var.set("先勾上安全确认。")
            return
        max_pct = self._parsed_max_pct()
        if max_pct is None:
            self.spin_status_var.set(
                f"本次上限必须是 {SPIN_MAX_PCT_MIN}-{SPIN_MAX_PCT_MAX} 之间的"
                "整数，命令未发送。")
            return
        # 开窗那一刻油门必须是 0：开窗本身不该让任何东西转起来。
        self.throttle_var.set(0)
        self.throttle_scale.set(0)
        self.throttle_label_var.set("油门 0%")
        if not self.send(
                f"PROPCAL SPIN ARM confirm={SPIN_CONFIRM_TOKEN} "
                f"max_pct={max_pct}"):
            return
        self.spin_active = True
        self._refresh_spin_controls()
        self._schedule_heartbeat()

    def stop_spin(self) -> None:
        self._cancel_heartbeat()
        self.spin_active = False
        self._refresh_spin_controls()
        self.send("PROPCAL SPIN STOP")

    def _schedule_heartbeat(self) -> None:
        if self._disposed:
            return
        self._timer = self.panel.after(HEARTBEAT_MS, self._heartbeat)

    def _cancel_heartbeat(self) -> None:
        if self._timer is not None:
            try:
                self.panel.after_cancel(self._timer)
            except tk.TclError:      # 面板已经在拆了，取消本身不重要
                pass
            self._timer = None

    def _heartbeat(self) -> None:
        """每拍重发一次油门。**发不出去就本地停**。

        发不出去意味着连接断了或者被安全门挡了。此时飞控那边 300 ms 内也会因为
        收不到心跳而关窗——本地跟着停只是让界面别继续显示"正在转"，两边说的是
        同一件事。
        """
        self._timer = None
        if self._disposed or not self.spin_active:
            return
        channel = int(self.spin_channel_var.get())
        percent = int(self.throttle_var.get())
        if not self.send(f"PROPCAL SPIN SET ch={channel} pct={percent}"):
            self.spin_active = False
            self._refresh_spin_controls()
            self.spin_status_var.set("命令发不出去，已停止（飞控也会因心跳超时断电）。")
            return
        self._schedule_heartbeat()

    # ------------------------------------------------------------ 回包

    def handle_board_line(self, line: str) -> None:
        if line.startswith("ESC EDT"):
            self._absorb_edt(parse_kv(line))
            return
        if not line.startswith("PROPCAL"):
            return
        # 按整行前缀认领，不靠 key 组合猜：escdiag 的字段名和别的回包毫无交集，
        # 但"没有交集"是今天的事实，前缀是固件明写出来的契约。
        if line.startswith("PROPCAL escdiag"):
            self._absorb_esc_diag(parse_kv(line))
            return
        values = parse_kv(line)
        if "ch" in values and "role" in values:
            self._absorb_channel(values)
            return
        if "spin" in values and "age_ms" in values:
            self._absorb_spin(values)
            return
        state = values.get("state")
        if state is None:
            return
        self._absorb_status(state, values)

    def _absorb_channel(self, values: dict[str, str]) -> None:
        channel = safe_int(values.get("ch"), 0)
        if channel not in (1, 2):
            return
        self.channels[channel] = dict(values)
        pad = values.get("pad", "-")
        self.pad_vars[channel].set(
            f"通道 {channel} · {pad}" if pad not in ("-", "") else f"通道 {channel}")
        # 回读覆盖下拉框，但**不能**顺手把它当成一次用户修改再发回去。
        self._suppress_trace = True
        try:
            self.role_vars[channel].set(ROLE_LABEL.get(values.get("role", "-"),
                                                       ROLE_LABEL["-"]))
            declared = values.get("declared") == "1"
            spin = values.get("spin", "-")
            self.spin_vars[channel].set(
                SPIN_LABEL.get(spin, SPIN_LABEL["-"]) if declared
                else SPIN_LABEL["-"])
        finally:
            self._suppress_trace = False

    def _absorb_status(self, state: str, values: dict[str, str]) -> None:
        if "calibrated" in values:
            self.calibrated = values.get("calibrated") == "1"
            self.dirty = values.get("dirty") == "1"
            self.generation = values.get("gen", self.generation)
            self.upper_channel = safe_int(values.get("upper_ch"), 0)
            self.lower_channel = safe_int(values.get("lower_ch"), 0)
            self.lower_spin = values.get("lower_spin", "-")
            self.yaw_polarity = values.get("yaw_polarity", "0")
            self._refresh_conclusion()
            self._refresh_dirty()

        text = STATE_TEXT.get(state, state)
        reason = REASON_TEXT.get(values.get("reason", "-"), "")
        if state == "commit_failed":
            text = f"保存失败 st={values.get('st', '?')}，标定仍在 RAM 里。"
        self.status_var.set(f"{text} {reason}".strip() or text)

    def _absorb_spin(self, values: dict[str, str]) -> None:
        self.max_percent = safe_int(values.get("max_pct"), self.max_percent) or 20
        self.throttle_scale.configure(to=self.max_percent)
        active = values.get("spin") == "active"
        self.spin_stop_reason = values.get("stop", "none")
        if active:
            channel = safe_int(values.get("ch"), 0)
            percent = safe_int(values.get("pct"), 0)
            self.spin_status_var.set(
                f"通电中：通道 {channel} 油门 {percent}%"
                f"（心跳年龄 {values.get('age_ms', '?')} ms，"
                f"超时 {values.get('timeout_ms', '?')} ms 后自动断电）")
        else:
            why = REASON_TEXT.get(self.spin_stop_reason, self.spin_stop_reason)
            self.spin_status_var.set(f"未通电。{why}".strip())
        self._warn_if_ceiling_was_not_honoured()
        if self.spin_active and not active:
            # 飞控说停了（超时/抢占/拒绝），本地必须跟着停：继续发心跳会让窗口
            # 在人已经不看着的情况下重新转起来。
            self._cancel_heartbeat()
        self.spin_active = active
        self._refresh_spin_controls()
        self._absorb_esc_telem(values)

    def _warn_if_ceiling_was_not_honoured(self) -> None:
        """飞控报回来的上限比要的小，就说出来是哪一方在压着。

        滑条的 `to` 一律跟飞控走（飞控是唯一权威），但**默默跟着**会让人以为是
        界面坏了：2026-09-21 作者把上限填成 82，滑条仍然只到 20，页面上一个字
        都没解释——因为板子上那版固件还不认 `max_pct=`，回包里恒是 20。

        飞控只降不升，所以"报回来的比要的小"只有两种可能，都值得当场说出来：
        固件太旧不认这个参数，或者窗口还开着（开着时 ARM 不抬高上限）。
        """
        want = self._parsed_max_pct()
        if want is None or self.max_percent >= want:
            return
        self.spin_status_var.set(
            f"{self.spin_status_var.get()} "
            f"注意：本次上限要的是 {want}%，飞控只给到 {self.max_percent}%——"
            "多半是飞控固件还不认 max_pct=（需要升级固件）；"
            "若固件已支持，则是窗口还开着（开着时上限只降不升，先停止再开窗）。")

    def request_edt(self, mode: str) -> None:
        """`ESC EDT ON|OFF`。界面只送两个词，命令号在固件里。"""
        if self.send(f"ESC EDT {mode}"):
            self.edt_var.set(f"已请求 {mode}；等待飞控回状态…")

    def _absorb_edt(self, values: dict[str, str]) -> None:
        """回包说的是"帧发出去了"，**不是"EDT 打开了"**。

        DShot 是单向发送，电调不对特殊命令回 ACK。能证明的只有"N 帧连续命令帧
        已经提交"。EDT 到底开没开，唯一判据是之后收不收得到 EDT 帧——看解码计数
        和电流字段。把"发出去了"说成"打开了"，会让人在下一步去查接收侧。
        """
        event = values.get("event", "-")
        reason = values.get("reason", "-")
        if event == "rejected":
            hint = {"armed": "已解锁，先上锁。",
                    "busy": "上一条还在发，等它发完。",
                    "not_dshot": "本次构建不是 DShot 档，没有这条通道。",
                    "usage": "用法不对。"}.get(reason, reason)
            self.edt_var.set(f"飞控拒绝：{hint}")
            return
        state = values.get("state", "-")
        sent = values.get("sent", "?")
        repeats = values.get("repeats", "?")
        if state == "done":
            self.edt_var.set(
                f"已发出 {sent}/{repeats} 帧命令。"
                "这只说明帧发出去了，不代表 EDT 已生效——"
                "点「读取电调解码计数」，看电流字段有没有开始出数。")
        elif state == "aborted":
            self.edt_var.set(
                f"发到 {sent}/{repeats} 帧被中止（{values.get('reason', '-')}）。"
                "序列断了就整条作废，重新点一次。")
        else:
            self.edt_var.set(f"状态 {state}，已发 {sent}/{repeats} 帧。")

    def request_esc_diag(self) -> None:
        """裸 `PROPCAL SPIN` 是状态查询，不开窗、不让任何东西转。"""
        if self.send("PROPCAL SPIN"):
            self.esc_diag_var.set("已请求；等待飞控回计数…")

    def _absorb_esc_diag(self, values: dict[str, str]) -> None:
        """把解码计数翻译成"接下来该查什么"，而不是原样堆六个数字。

        三种走向指向三个完全不同的修法：
          - 超时在涨、成功与校验错都不动 → 线上压根没有回传，电调多半还在单向模式；
          - 校验错在涨                   → 有回传但解不开：极性 / 校验取反 / GCR 表 / 位宽；
          - 成功在涨                     → 链路通，剩下的是数值对不对。
        `doc/esc-output.md` 里 GCR 的 nibble→五位组对应关系 2026-09-21 已在 bitbang
        后端实机钉死（十几万帧累计 crc 错 1 次），"校验错在涨"时 GCR 表不再是首要嫌疑。
        """
        proto = safe_int(values.get("proto"), -1)
        proto_text = {0: "PWM", 1: "单向 DShot300",
                      2: "双向 DShot300"}.get(proto, f"未知({proto})")
        avail = values.get("avail") == "1"
        counts = {}
        for channel, keys in ((1, ("fr1", "crc1", "to1")),
                              (2, ("fr2", "crc2", "to2"))):
            counts[channel] = tuple(safe_int(values.get(key), 0) for key in keys)
        self.esc_diag_var.set(
            f"构建档位 {proto_text} · 接收相 {'已启用' if avail else '未启用'}\n"
            + "\n".join(
                f"通道 {ch}：成功 {c[0]} · 校验错 {c[1]} · 超时 {c[2]}"
                for ch, c in counts.items()))

        frames = sum(c[0] for c in counts.values())
        crc = sum(c[1] for c in counts.values())
        timeouts = sum(c[2] for c in counts.values())
        grace = safe_int(values.get("grace"), 0)
        if not avail:
            hint = ("本次构建不是双向档，接收相根本没启用——"
                    "要用 DShot 回传必须烧 ESC_PROTOCOL=DSHOT300_BIDIR 的固件。")
        elif grace > 0:
            # 宽限期内"没收到"是**设计如此**，不是故障：此刻固件压根没在听，
            # 正把线驱动在空闲高让电调自检出双向模式。不说明白的话，这段时间的
            # 全零计数会被读成"电调不回话"，把人引去查完全无关的地方。
            hint = (f"开机自检宽限期还剩 {grace} 帧（约 {grace / 500.0:.1f} s）："
                    "此刻固件没在听，正把线驱动在空闲高让电调自检出双向模式。"
                    "等它归零再读一次。")
        elif frames == 0 and crc == 0 and timeouts == 0:
            hint = "三个计数都是 0：还没发过帧，先开窗给一点油门再读一次。"
        elif frames == 0 and crc == 0:
            hint = ("只有超时在涨：线上没有回传。先确认电调固件已切到双向 DShot"
                    "（注意不是电机正反转的 bi_direction 设置）。")
        elif frames == 0:
            hint = ("有回传但一帧都没解开：极性 / 校验取反 / GCR 表 / 位宽有一个不对。"
                    "GCR 对应关系 2026-09-21 已在 bitbang 后端实机钉死，先查其余几项。")
        else:
            hint = ""
        self.esc_diag_hint_var.set(hint)

    def _absorb_esc_telem(self, values: dict[str, str]) -> None:
        """电调 EDT 回传跟着心跳回包走（10 Hz），直接在这里刷新，不必另开定时器。

        没数据时整块降级成一句解释、不显示任何数字：显示 0 A / 0 eRPM 会被读成
        "量过了，确实是 0"，而实际是根本没有这条数据。

        **"没有"分两种，必须分开说。** `esc_telem=0` 是飞控自己报的"这一档构建
        不回传"；而整个 `esc_telem` 字段**缺席**只说明对面那版固件还不发这几个
        字段——此时界面无从知道它是哪一档构建。把两者合并成一句"本次构建是单向
        DShot300"，就是在替一个没说话的固件下结论：2026-09-21 作者拿新面板连旧
        固件时正是被这句话带偏，去查电调而不是先升级固件。
        """
        raw = values.get("esc_telem")
        if raw is None:
            self.esc_state_var.set(
                "飞控固件这一版不报电调回传字段（esc_telem 缺席），先升级固件。")
            for channel in (1, 2):
                self.esc_current_vars[channel].set("—")
                self.esc_erpm_vars[channel].set("—")
            return
        if raw != "1":
            self.esc_state_var.set("飞控报本次构建是单向 DShot300，电调不回传遥测。")
            for channel in (1, 2):
                self.esc_current_vars[channel].set("—")
                self.esc_erpm_vars[channel].set("—")
            return
        self.esc_state_var.set("已收到电调回传。")
        for channel, i_key, erpm_key in ((1, "esc_i1", "esc_erpm1"),
                                         (2, "esc_i2", "esc_erpm2")):
            self.esc_current_vars[channel].set(
                self._format_dash_int(values.get(i_key), "A"))
            self.esc_erpm_vars[channel].set(
                self._format_dash_int(values.get(erpm_key), "eRPM"))

    @staticmethod
    def _format_dash_int(raw: str | None, unit: str) -> str:
        """固件用字面量 `-` 表示"这一路没有有效数据"，不是数值 0。"""
        if raw is None or raw == "-":
            return "—"
        try:
            value = int(raw, 0)
        except ValueError:
            return "—"
        return f"{value} {unit}"

    def _refresh_dirty(self) -> None:
        if self.dirty:
            self.dirty_var.set("● 有未保存的改动")
        else:
            self.dirty_var.set("")

    def _refresh_conclusion(self) -> None:
        if not self.calibrated:
            self.conclusion_var.set(
                "未标定：偏航极性 = 0（没有方向，不是默认正方向）。\n"
                "分配式因此给不出偏航力矩，执行器也查不到上/下桨该去哪个通道。")
            return
        spin_text = SPIN_LABEL.get(self.lower_spin, self.lower_spin)
        if self.yaw_polarity == "+1":
            rule = "正偏航（机头左转）靠**加大下桨**推力获得"
        elif self.yaw_polarity == "-1":
            rule = "正偏航（机头左转）靠**加大上桨**推力获得"
        else:
            rule = "极性未定"
        self.conclusion_var.set(
            f"上桨 = 通道 {self.upper_channel}，下桨 = 通道 {self.lower_channel}；"
            f"下桨旋向 {spin_text}。\n"
            f"偏航力矩极性 = −下桨旋向 = {self.yaw_polarity} —— {rule}。")

    # ------------------------------------------------------------ 实时读数（总线）
    #
    # 做法照抄 power.py：可见才订阅、不可见就退订；订阅了但没帧/帧过期/通道
    # 不在掩码里一律显示 "—" 并说明原因，不把陈旧值当实时值。

    def visible(self) -> bool:
        """本页是不是当前正显示着的那一个：顶层"校准"分组 + 二级"桨叶与电机方向"。"""
        panel = self.panel
        group = getattr(panel, "calibration_group_tab", None)
        tab = getattr(panel, "prop_map_tab", None)
        notebook = getattr(panel, "notebook", None)
        calibration = getattr(panel, "calibration_notebook", None)
        if group is None or tab is None or notebook is None or calibration is None:
            return False
        try:
            return (notebook.select() == str(group)
                    and calibration.select() == str(tab))
        except tk.TclError:                       # pragma: no cover - 窗口正在销毁
            return False

    def _sync_subscription(self, visible: bool) -> None:
        """可见才订阅 `batt_v` / `batt_i`，不可见就退订。

        退订走仲裁器（`panel._telem_subscribe/_unsubscribe`）而不是直接发
        `TELEM MASK`：并集要和别的页面一起算，各发各的会互相把对方的位擦掉。
        """
        subscribe = getattr(self.panel, "_telem_subscribe", None)
        unsubscribe = getattr(self.panel, "_telem_unsubscribe", None)
        if subscribe is None or unsubscribe is None:  # pragma: no cover - 无工作台的替身面板
            return
        if visible:
            subscribe(TELEM_OWNER_PROP_MAP, PROP_MAP_BUS_CHANNELS, stream=True,
                      consumer=self._on_telem_frame)
            self.subscribed = True
        elif self.subscribed:
            unsubscribe(TELEM_OWNER_PROP_MAP)
            self.subscribed = False

    def _on_telem_frame(self, function: int, payload: bytes, *, transport,
                        generation=None) -> None:
        """**收线程**回调：只留到达证据，不碰任何 Tk 控件。

        来源身份在分发那一刻钉死，不是现读 `panel.transport`：重连之后旧链路
        补送的最后一帧不能被当成新会话的数据显示出来（做法与 power.py 一致）。
        """
        del function, payload
        if generation is None:
            generation = getattr(transport, "connection_generation", 0)
        self.telem_stamp = _TelemStamp(time.monotonic(), transport, generation)

    def _channel_value(self, name: str, *, newest_t: float | None = None):
        """通道最新值，同时判年龄。NaN 与陈旧一律返回 None，不当 0 显示。

        年龄按**固件时间轴**比较（这一路自己的时间戳 vs. 本帧最新时间戳），
        不受主机时钟和调度抖动影响——做法与 `power.py::_channel_value` 一致。
        """
        entry = self.panel._telem_latest_raw(name)
        if entry is None:
            return None
        stamp, value = float(entry[0]), float(entry[1])
        if not math.isfinite(value):
            return None
        if newest_t is not None and (newest_t - stamp) > PROP_MAP_TELEM_STALE_S:
            return None
        return value

    def _newest_channel_time(self) -> float | None:
        """最近一帧遥测的固件时间戳（秒），当"现在几点了"的参照（同 power.py）。"""
        t_us = getattr(self.panel, "telem_last_frame_t_us", None)
        return None if t_us is None else float(t_us) * 1e-6

    def _render_bus(self) -> None:
        """把总线两路的最新值画出来；显示不了就说明属于哪一类原因。"""
        if not self.subscribed:
            self._blank_bus("没订阅：页面不可见时不订阅遥测流。")
            return
        if getattr(self.panel, "_telem_latest_raw", None) is None:
            self._blank_bus("当前面板没有遥测流。")  # pragma: no cover - 无工作台的替身面板
            return
        stamp = self.telem_stamp
        transport = getattr(self.panel, "transport", None)
        if stamp is None or transport is None or not stamp.is_current(transport):
            self._blank_bus("没帧：尚未收到当前连接的遥测帧。")
            return
        age = time.monotonic() - stamp.at
        if age > PROP_MAP_TELEM_STALE_S:
            self._blank_bus(f"没帧：遥测流已经 {age:.1f} s 没有新帧。")
            return
        missing = [name for name in PROP_MAP_BUS_CHANNELS
                   if self.panel._telem_latest_raw(name) is None]
        if missing:
            self._blank_bus(f"{' / '.join(missing)} 不在遥测帧里（通道不在掩码里）。")
            return
        newest_t = self._newest_channel_time()
        volts = self._channel_value(PROP_MAP_CHANNEL_VOLTAGE, newest_t=newest_t)
        amps = self._channel_value(PROP_MAP_CHANNEL_CURRENT, newest_t=newest_t)
        self.bus_voltage_var.set("—" if volts is None else f"{volts:.3f} V")
        self.bus_current_var.set("—" if amps is None else f"{amps:.3f} A")
        if volts is None or amps is None:
            self.bus_state_var.set("数据无效或已过期（NaN，或超过 1.5 s 未更新）。")
        else:
            self.bus_state_var.set(f"实时推送中 · {age * 1000:.0f} ms 前到达。")

    def _blank_bus(self, reason: str) -> None:
        self.bus_voltage_var.set("—")
        self.bus_current_var.set("—")
        self.bus_state_var.set(reason)

    def _schedule_telem_tick(self) -> None:
        if self._disposed:
            return
        self._telem_timer = self.panel.after(PROP_MAP_RENDER_MS, self._telem_tick)

    def _cancel_telem_tick(self) -> None:
        if self._telem_timer is not None:
            try:
                self.panel.after_cancel(self._telem_timer)
            except tk.TclError:      # 面板已经在拆了，取消本身不重要
                pass
            self._telem_timer = None

    def _telem_tick(self) -> None:
        """界面节拍：同步订阅可见性 + 画总线缓冲里的最新值。

        不发送任何周期性命令——数据来自遥测流推送，这里只读缓冲；和点电机的
        心跳定时器（`_schedule_heartbeat`）完全独立，互不影响。
        """
        self._telem_timer = None
        if self._disposed:
            return
        self._sync_subscription(self.visible())
        self._render_bus()
        self._schedule_telem_tick()

    # ------------------------------------------------------------ 拆除

    def _on_destroy(self, event) -> None:
        if event.widget is not self.panel:
            return
        self._disposed = True
        self._cancel_heartbeat()
        self._cancel_telem_tick()
        self._sync_subscription(False)
        # 这里**不发** STOP：面板正在拆，发不发得出去都没保证。真正让电机停下来
        # 的是飞控那边的心跳超时——心跳没了就是停，不依赖我们还能说上话。
        self.spin_active = False


def mount_prop_map(panel, parent: ttk.Frame) -> PropMapPage:
    page = PropMapPage(panel, parent)
    panel.prop_map_page = page
    return page
