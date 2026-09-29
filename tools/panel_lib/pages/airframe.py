"""机体模型页：把量出来的机体数据写进飞控 Flash。

为什么要有这一页：2026-09-11 起飞控固件里**不再保留任何机体数据**，
唯一来源是这一页写进去的那份（Driver/Inc/drv_airframe_params.h）。
一块刚烧完固件的板子没有质量、没有惯量、没有力臂，控制律算出来的每个力矩都
没有物理含义，所以它**禁止解锁**——这一页不填，飞机就飞不了。

界面分三层，分层本身是有意的：

  基础（可手测量）—— 秤和尺能直接得到的：部件质量、部件重心 z、几何距离、
      双桨最大推力。直接可改。
  派生（算出来的）—— 整机质量、重心、重量、悬停油门……自动档下置灰，
      手动档下可改并且并排显示"若自动会是多少"。
  高级（估算/辨识/反推）—— 惯量、重力、舵机标度。
      默认折叠，改之前要二次确认。

倾转力臂不是输入项（2026-09-27 起）：它是"舵机转轴 z − 整机重心 z"，派生区下方
单独列出上位机按飞控派生的重心算出的预览值——飞控在控制律里现算同一个减法，
但不把它存成参数。改部件质量/重心或舵机转轴会让力臂变：速率环增益是 N·m 单位，
固件按力臂把它换成倾角，所以同一组增益的实际软硬会变成 L_旧/L_新 倍。写入前
预览里就标出"旧力臂 → 新力臂"，点「写入飞控」时再弹窗确认，并提醒改完要重新辨识。
派生档从手动切回自动也算：飞控在那一刻按部件表重算重心（手动填的实测重心被盖掉），
所以「切换自动/手动」与高级区里改 derived_auto 走同一套提醒，写完再重读一遍参数。

高级层单独关起来不是为了少占地方：惯量是**估计值**、下桨旋向是**反推值**，
它们和"电池多重"摆在一起会让人以为同样可信，随手就改了。而下桨旋向单独决定
偏航力矩极性，翻错了表现是正反馈——和增益调大很像，很容易被误诊。

本页不重复实现校验：值经 parameter_model.validate_parameter_text 后走
`PARAM SET airframe.<字段> <值>`，与参数页同一条写入路径；飞控是权威，
本页只负责别让明显打错的数发出去。
"""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk

from ..airframe_model import (
    AIRFRAME_FIELDS,
    DERIVED_AUTO_FIELD,
    TILT_AXES,
    AirframeField,
    compute_derived,
    describe_invalid,
    first_missing,
    tilt_axis_to_cg_z,
)
from ..parameter_model import validate_parameter_text
from ..proto import PROTO_REQ_PARAMS, PROTO_REQ_PARAM_SET, PROTO_REQ_SAVE


REFRESH_MS = 400

#: 改了会让倾转力臂 L = cg_z_m − servoN_axis_z_m 变的输入：部件质量与重心（自动派生档下
#: 重心由它们算）、手动档的整机重心、两个舵机转轴。
_LEVER_INPUTS = ("board_mass_g", "battery_mass_g", "base_mass_g", "servo_motor_mass_g",
                 "board_cg_z_m", "battery_cg_z_m", "base_cg_z_m", "servo_motor_cg_z_m",
                 "servo1_axis_z_m", "servo2_axis_z_m", "cg_z_m")
#: 部件表推算重心要用的八项：自动档下缺任何一项就不预测新重心（不拿 0 冒充）。
_CG_INPUTS = _LEVER_INPUTS[:8]
#: 力臂变化小于它不提醒 [m]（参数 6 位小数、部件表的量化）。
LEVER_CHANGE_MIN_M = 0.0005
_AXIS_TEXT = {"roll": "横滚（舵机 1）", "pitch": "俯仰（舵机 2）"}

_FIELD_BY_NAME = {field.name: field for field in AIRFRAME_FIELDS}
_FIELD_BY_NAME[DERIVED_AUTO_FIELD.name] = DERIVED_AUTO_FIELD


def _fmt(value: float | None, digits: int = 6) -> str:
    if value is None:
        return "—"
    return f"{value:.{digits}f}".rstrip("0").rstrip(".") or "0"


class AirframePage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent, padding=0)
        self.panel = panel
        self.entries: dict[str, ttk.Entry] = {}
        self.vars: dict[str, tk.StringVar] = {}
        self.derived_labels: dict[str, ttk.Label] = {}
        self.preview_labels: dict[str, ttk.Label] = {}
        self.tilt_lever_labels: dict[str, ttk.Label] = {}
        self.tilt_lever_source_var = tk.StringVar(value="")
        self.tilt_lever_change_var = tk.StringVar(value="")
        self.advanced_unlocked = False
        self._save_pending = False
        self.status_var = tk.StringVar(value="尚未读取飞控机体模型")
        self.detail_var = tk.StringVar(value="")
        self.derived_auto_var = tk.StringVar(value="—")
        self._timer = None
        self._disposed = False

        self._build()
        panel.bind("<Destroy>", self._on_destroy, add="+")
        self._tick()

    # ── 构建 ──────────────────────────────────────────────────────────
    def _build(self) -> None:
        ttk.Label(self, text="AIRFRAME MODEL  /  机体模型",
                  style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(self, text="量出来的机体数据，飞控的唯一来源",
                  style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            self,
            text=(
                "固件里不保留任何机体数据：质量、重心、惯量、力臂全部只从 Flash 读，"
                "由本页写入。没有有效模型时飞控**禁止解锁**，并在解锁横幅里指出缺哪一项。"
                "改完先「写入飞控」（立刻生效，掉电丢失），确认无误再「保存到 Flash」。"
                "坐标基准 FLU：原点 = 飞控板中心，+Z 向上，所以挂在板子下方的部件 z 为负。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))

        banner = ttk.LabelFrame(self, text="模型状态", padding=10)
        banner.pack(fill=tk.X)
        ttk.Label(banner, textvariable=self.status_var,
                  style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(banner, textvariable=self.detail_var,
                  style="Muted.TLabel", wraplength=1080).pack(anchor=tk.W, pady=(2, 0))

        bar = ttk.Frame(self)
        bar.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(bar, text="从飞控读取", command=self._read_all).pack(side=tk.LEFT)
        ttk.Button(bar, text="写入飞控（RAM）", command=self._write_all,
                   style="Primary.TButton").pack(side=tk.LEFT, padx=6)
        ttk.Button(bar, text="保存到 Flash", command=self._save).pack(side=tk.LEFT)
        ttk.Label(bar, text="派生值：").pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(bar, textvariable=self.derived_auto_var).pack(side=tk.LEFT)
        ttk.Button(bar, text="切换自动/手动",
                   command=self._toggle_derived_auto).pack(side=tk.LEFT, padx=6)

        body = ttk.Frame(self)
        body.pack(fill=tk.BOTH, expand=True, pady=(12, 0))

        basic = [f for f in AIRFRAME_FIELDS if f.tier == "basic" and not f.derived]
        derived = [f for f in AIRFRAME_FIELDS if f.derived]
        advanced = [f for f in AIRFRAME_FIELDS if f.tier == "advanced" and not f.derived]

        self._build_input_groups(body, basic, "基础 · 拿秤和尺量得到")
        self._build_derived_group(body, derived)
        self._build_tilt_lever_preview(body)

        self.advanced_frame = ttk.LabelFrame(
            body, text="高级 · 估算 / 辨识 / 反推得来（改前请确认）", padding=10)
        self.advanced_frame.pack(fill=tk.X, pady=(12, 0))
        self.advanced_button = ttk.Button(
            self.advanced_frame, text="解锁高级参数…", command=self._unlock_advanced)
        self.advanced_button.pack(anchor=tk.W)
        self.advanced_body = ttk.Frame(self.advanced_frame)
        self._advanced_fields = advanced + [DERIVED_AUTO_FIELD]

    def _build_input_groups(self, parent, fields: list[AirframeField], title: str) -> None:
        outer = ttk.LabelFrame(parent, text=title, padding=10)
        outer.pack(fill=tk.X)
        for group in dict.fromkeys(f.group for f in fields):
            frame = ttk.LabelFrame(outer, text=group, padding=8)
            frame.pack(fill=tk.X, pady=(0, 8))
            for row, field in enumerate(f for f in fields if f.group == group):
                self._add_input_row(frame, row, field)

    def _add_input_row(self, frame, row: int, field: AirframeField) -> None:
        ttk.Label(frame, text=field.label).grid(row=row, column=0, sticky=tk.W, pady=3)
        var = tk.StringVar()
        entry = ttk.Entry(frame, textvariable=var, width=14)
        entry.grid(row=row, column=1, sticky=tk.W, padx=(10, 4), pady=3)
        ttk.Label(frame, text=field.unit, style="Muted.TLabel").grid(
            row=row, column=2, sticky=tk.W)
        if field.note:
            ttk.Label(frame, text=field.note, style="Muted.TLabel",
                      wraplength=720).grid(row=row, column=3, sticky=tk.W, padx=(14, 0))
        frame.columnconfigure(3, weight=1)
        self.vars[field.name] = var
        self.entries[field.name] = entry

    def _build_derived_group(self, parent, fields: list[AirframeField]) -> None:
        outer = ttk.LabelFrame(parent, text="派生 · 由上面算出", padding=10)
        outer.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(
            outer,
            style="Muted.TLabel", wraplength=1080,
            text=(
                "自动档下这些值由飞控重算，写入会被当场拒绝——那是刻意的：写进去再被"
                "下次重算盖掉，会让上位机以为写成功了，下次读回来又变了，找不到原因。"
                "手动档留给「惯量来自双线摆实测而不是部件表推算」这类场景，代价是它可能"
                "和部件表悄悄对不上，所以右侧并排显示推算值。"
            ),
        ).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 8))
        for column, text in enumerate(("飞控当前值", "手动写入", "按部件表推算"), start=1):
            ttk.Label(outer, text=text, style="Muted.TLabel").grid(
                row=1, column=column, sticky=tk.W, padx=(10, 4))
        for row, field in enumerate(fields, start=2):
            ttk.Label(outer, text=field.label).grid(row=row, column=0, sticky=tk.W, pady=2)
            current = ttk.Label(outer, text="—")
            current.grid(row=row, column=1, sticky=tk.W, padx=(10, 4))

            # 手动档下派生值可写（留给"惯量来自双线摆实测"这类场景）；自动档下
            # 置灰，因为飞控会**当场拒绝**写入，给个能打字的框只会让人白填一遍。
            var = tk.StringVar()
            entry = ttk.Entry(outer, textvariable=var, width=12, state="disabled")
            entry.grid(row=row, column=2, sticky=tk.W, padx=(10, 4))
            self.vars[field.name] = var
            self.entries[field.name] = entry

            preview = ttk.Label(outer, text="—", style="Muted.TLabel")
            preview.grid(row=row, column=3, sticky=tk.W, padx=(10, 4))
            ttk.Label(outer, text=field.unit, style="Muted.TLabel").grid(
                row=row, column=4, sticky=tk.W)
            self.derived_labels[field.name] = current
            self.preview_labels[field.name] = preview
        outer.columnconfigure(4, weight=1)

    def _build_tilt_lever_preview(self, parent) -> None:
        """每轴倾转力臂 r_z 的**上位机预览**。

        不放进派生表：那张表的每一行都是飞控参数（可读、手动档可写），而 r_z 不是
        ——飞控在控制律里现算，不存成参数。混在一起会让人以为它能 PARAM GET/SET。
        """
        outer = ttk.LabelFrame(
            parent, text="倾转力臂 r_z · 上位机计算（非飞控参数）", padding=10)
        outer.pack(fill=tk.X, pady=(12, 0))
        ttk.Label(
            outer, style="Muted.TLabel", wraplength=1080,
            text=(
                "r_z = 舵机转轴 z − 整机重心 z。推力作用线穿过舵机转轴，所以倾转力矩 = "
                "−r_z × 推力 × sin(倾角)：r_z 的大小就是力臂，正负就是力矩方向（负 = 转轴在"
                "重心下方）。飞控在控制律里现算同一个减法，但不存成参数；这里用飞控派生出来"
                "的重心算给你看。为 0、|r_z| < 1 cm 或与推力点不在重心同侧时飞控禁止解锁。"
            ),
        ).grid(row=0, column=0, columnspan=3, sticky=tk.W, pady=(0, 8))
        labels = {"roll": "横滚 r_z（舵机 1 转轴 − 重心）",
                  "pitch": "俯仰 r_z（舵机 2 转轴 − 重心）"}
        for row, (axis, _key) in enumerate(TILT_AXES, start=1):
            ttk.Label(outer, text=labels[axis]).grid(row=row, column=0, sticky=tk.W, pady=2)
            value = ttk.Label(outer, text="—")
            value.grid(row=row, column=1, sticky=tk.W, padx=(10, 4))
            ttk.Label(outer, text="m", style="Muted.TLabel").grid(
                row=row, column=2, sticky=tk.W)
            self.tilt_lever_labels[axis] = value
        ttk.Label(outer, textvariable=self.tilt_lever_source_var,
                  style="Muted.TLabel").grid(
            row=len(TILT_AXES) + 1, column=0, columnspan=3, sticky=tk.W, pady=(4, 0))
        # 输入框里改了、还没写入时：旧力臂 → 新力臂，以及增益软硬的变化。
        ttk.Label(outer, textvariable=self.tilt_lever_change_var, style="Warn.TLabel",
                  wraplength=1080, justify=tk.LEFT).grid(
            row=len(TILT_AXES) + 2, column=0, columnspan=3, sticky=tk.W, pady=(4, 0))

    # ── 高级层二次确认 ────────────────────────────────────────────────
    def _unlock_advanced(self) -> None:
        ok = messagebox.askokcancel(
            "解锁高级机体参数",
            "下面这些不是量出来的：\n\n"
            "· 转动惯量 I_xx/I_yy/I_zz —— 估计值，没有实测\n"
            "· 下桨旋向 —— 反推值，由「偏航高增益抖振而非发散」推出；"
            "它单独决定偏航力矩极性，翻错了是正反馈\n\n"
            "改它们会直接改变控制律的行为，而且症状要等飞起来才出现。\n"
            "确定要打开吗？",
            icon=messagebox.WARNING, parent=self,
        )
        if not ok:
            return
        self.advanced_unlocked = True
        self.advanced_button.pack_forget()
        self.advanced_body.pack(fill=tk.X)
        for row, field in enumerate(self._advanced_fields):
            self._add_input_row(self.advanced_body, row, field)
        self._refresh_from_panel(force_entries=True)

    def _toggle_derived_auto(self) -> None:
        """切到自动档时飞控当场按部件表重算重心：力臂会变就先弹窗，写完重读派生值。"""
        current = self._target(DERIVED_AUTO_FIELD.name)
        new = 0.0 if (current is not None and abs(current) > 0.5) else 1.0
        edit = [(DERIVED_AUTO_FIELD.name, f"{new:.0f}")]
        warning = self.lever_change_text(edit)
        if warning and not self._confirm_lever_change(warning):
            self.detail_var.set("已取消切换：" + warning.splitlines()[0])
            return
        if self._send_param(*edit[0]):
            self._reread_after_mode_change()
            if warning:
                self.detail_var.set("已切换派生档。" + warning.splitlines()[-1])

    def _reread_after_mode_change(self) -> None:
        """写 derived_auto 后飞控只回显这一项；重算出来的重心等派生值要 PARAM? 才读得回来，
        不重读的话页面上的力臂预览还停在旧重心。"""
        self._read_all()

    # ── 与飞控通信 ────────────────────────────────────────────────────
    def _read_all(self) -> None:
        if not self.panel._transport_connected():
            self.detail_var.set("连接不可用，未发送")
            return
        self.panel._send_proto(PROTO_REQ_PARAMS, "PARAM?")

    def _send_param(self, name: str, text: str) -> bool:
        valid, reason = validate_parameter_text(name, text)
        if not valid:
            self.detail_var.set(f"{name}：{reason}")
            return False
        if not self.panel._transport_connected():
            self.detail_var.set("连接不可用，草稿保留且未发送")
            return False
        self.panel._mark_param_pending(name, text)
        self.panel._send_proto(PROTO_REQ_PARAM_SET, f"PARAM SET {name} {text}")
        return True

    def _pending_edits(self) -> list[tuple[str, str]]:
        """输入框里与飞控当前值不同、这次要写的 (参数名, 文本)。"""
        pending = []
        auto_on = self._derived_auto_on()
        for name, var in self.vars.items():
            text = var.get().strip()
            if not text:
                continue
            # 自动档下派生值由飞控重算，发过去只会被逐条拒绝，真正改错的那条反而淹了。
            if auto_on and self._is_derived(name):
                continue
            target = self._target(name)
            if target is not None and _fmt(target) == _fmt(_safe_float(text)):
                continue
            pending.append((name, text))
        # 派生档放最后发：飞控写 derived_auto = 1 的那一刻按部件表重算，部件先改完，重算才用上
        # 新值（lever_change_text 按这个顺序预测新重心）。
        pending.sort(key=lambda item: item[0] == DERIVED_AUTO_FIELD.name)
        return pending

    def lever_change_text(self, pending: list[tuple[str, str]]) -> str:
        """这批改动会让飞控的倾转力臂怎么变；不涉及力臂或算不出来时返回空串。

        旧力臂按飞控回报的重心与转轴；新力臂按"飞控当前值 + 这批改动"。新重心照飞控的
        写入顺序推（派生档最后发）：写入时处在自动档的部件改动、以及切回自动档那一下，都会
        让飞控按部件表重算重心（与飞控同一公式）；一直是手动档就是整机重心那一项。
        """
        edits = {}
        for name, text in pending:
            value = _safe_float(text)
            if name.startswith("airframe.") and value is not None:
                edits[name.split(".", 1)[1]] = value
        auto_before = self._derived_auto_on()
        auto_after = (abs(edits[DERIVED_AUTO_FIELD.key]) > 0.5
                      if DERIVED_AUTO_FIELD.key in edits else auto_before)
        turning_auto_on = auto_after and not auto_before
        if not (set(edits) & set(_LEVER_INPUTS) or turning_auto_on):
            return ""
        known = {field.key: self._target(field.name) for field in AIRFRAME_FIELDS}
        known = {key: value for key, value in known.items() if value is not None}
        merged = {**known, **edits}
        old_cg = known.get("cg_z_m")
        if auto_after or (auto_before and set(edits) & set(_CG_INPUTS)):
            new_cg = (compute_derived(merged)["cg_z_m"]
                      if all(key in merged for key in _CG_INPUTS) else None)
        else:
            new_cg = merged.get("cg_z_m")
        if turning_auto_on and new_cg is None:
            return ("切回自动派生后，飞控会按部件表重算整机重心（盖掉手动填的重心），但页面还没读全"
                    "部件表，算不出新的倾转力臂：速率环增益（N·m 单位）的等效软硬可能变化。\n"
                    "改完需重新辨识（杆上「FF 测模型」），再按新结果调速率环增益。")
        lines = []
        for axis, key in TILT_AXES:
            old_axis, new_axis = known.get(key), merged.get(key)
            # 转轴 0 在固件里是"没填"（解锁闸门拒绝），没有可比的力臂。
            if None in (old_cg, new_cg, old_axis, new_axis) or 0.0 in (old_axis, new_axis):
                continue
            old, new = old_cg - old_axis, new_cg - new_axis
            if abs(new - old) < LEVER_CHANGE_MIN_M:
                continue
            line = f"{_AXIS_TEXT[axis]}倾转力臂 L = 重心 − 转轴：{old:.4f} m → {new:.4f} m"
            if old * new <= 0.0:
                line += "（方向反了：飞控会拒绝解锁，多半是某个符号填反了）"
            else:
                ratio = old / new
                feel = "变软" if ratio < 1.0 else "变硬"
                line += (f"：速率环增益（N·m 单位）的等效软硬会变为 L_旧/L_新 = {ratio:.2f} 倍"
                         f"（{feel}）")
            lines.append(line)
        if not lines:
            return ""
        if turning_auto_on:
            lines.insert(0, f"切回自动派生：飞控按部件表把整机重心从 {old_cg:.4f} m 重算成 "
                            f"{new_cg:.4f} m（手动填的重心被盖掉）")
        return "\n".join(lines + ["改完需重新辨识（杆上「FF 测模型」），再按新结果调速率环增益。"])

    def _confirm_lever_change(self, text: str) -> bool:
        return messagebox.askokcancel(
            "倾转力臂会变", text + "\n\n确定写入飞控吗？", icon=messagebox.WARNING, parent=self)

    def _write_all(self) -> None:
        """只发改过的字段。

        全量重发看着更简单，但它会把派生值也一起送出去——自动档下飞控逐条拒绝，
        界面上就是一串没头没脑的 ERR，真正改错的那条反而淹了。
        改动会让倾转力臂变时，发之前先弹窗说清楚旧 → 新和增益软硬的变化。
        """
        sent = 0
        skipped: list[str] = []
        pending = self._pending_edits()
        warning = self.lever_change_text(pending)
        if warning and not self._confirm_lever_change(warning):
            self.detail_var.set("已取消写入：" + warning.splitlines()[0])
            return
        for name, text in pending:
            if self._send_param(name, text):
                sent += 1
            else:
                skipped.append(name)
        if DERIVED_AUTO_FIELD.name in dict(pending) and DERIVED_AUTO_FIELD.name not in skipped:
            self._reread_after_mode_change()
        if sent == 0 and not skipped:
            self.detail_var.set("没有改动需要写入")
        elif skipped:
            self.detail_var.set(f"已写入 {sent} 项；{len(skipped)} 项被拒：{skipped[0]}")
        else:
            self.detail_var.set(f"已写入 {sent} 项到飞控 RAM，确认无误后请保存到 Flash"
                                + (f"。{warning.splitlines()[-1]}" if warning else ""))

    def _save(self) -> None:
        if not self.panel._transport_connected():
            self.detail_var.set("连接不可用，未发送")
            return
        self._save_pending = True
        self.panel._send_proto_once(PROTO_REQ_SAVE, "SAVE")
        self.detail_var.set("已发送 SAVE，等待飞控回报写入结果…")

    def _refresh_save_result(self) -> None:
        """把 SAVE 的**真实结果**说出来，不要让按钮看起来总是成功。

        这条不是锦上添花：机体模型不写进 Flash 就等于没写，下次上电飞控依旧
        拒绝解锁。而参数 Flash 在 MicoAir743V2 上目前是坏的（板上没有 GD25Q32，
        存储后端还没迁到 H743 片内 Flash），`SAVE` 会回 `st=4`。按钮要是照样
        看着成功，用户会以为存好了，等到下次上电解不了锁才发现——那时已经在
        外面了。
        """
        if not self._save_pending:
            return
        state = self.panel.param_states.get("config.last_flash_status")
        if state is None or state.target is None:
            return
        self._save_pending = False
        status = str(state.target).strip()
        if status == "0":
            self.detail_var.set("已保存到 Flash，下次上电自动加载")
        else:
            self.detail_var.set(
                f"⚠ 保存失败（st={status}）：参数没有写进 Flash，下次上电依旧无模型。"
                "请先在「诊断 / 命令」页确认参数 Flash 是否可用。"
            )

    # ── 刷新 ──────────────────────────────────────────────────────────
    def _target(self, name: str) -> float | None:
        state = self.panel.param_states.get(name)
        if state is None or state.target is None:
            return None
        return _safe_float(state.target)

    def _derived_auto_on(self) -> bool:
        value = self._target(DERIVED_AUTO_FIELD.name)
        return value is not None and abs(value) > 0.5

    @staticmethod
    def _is_derived(name: str) -> bool:
        field = _FIELD_BY_NAME.get(name)
        return bool(field and field.derived)

    def _tick(self) -> None:
        if self._disposed:
            return
        try:
            self._refresh_from_panel()
            self._refresh_save_result()
        finally:
            self._timer = self.panel.after(REFRESH_MS, self._tick)

    def _refresh_from_panel(self, force_entries: bool = False) -> None:
        values = {
            field.key: self._target(field.name)
            for field in (*AIRFRAME_FIELDS, DERIVED_AUTO_FIELD)
        }
        known = {k: v for k, v in values.items() if v is not None}

        auto = values.get("derived_auto")
        auto_on = auto is not None and abs(auto) > 0.5
        self.derived_auto_var.set(
            "自动重算" if auto_on else ("手动保持" if auto is not None else "—"))

        for field in AIRFRAME_FIELDS:
            var = self.vars.get(field.name)
            if var is None:
                continue
            target = values.get(field.key)
            # 只在输入框为空（或刚解锁高级层）时回填，否则会把用户正在打的字冲掉。
            if target is not None and (force_entries or not var.get().strip()):
                var.set(_fmt(target))

        preview = compute_derived(known)
        for field in AIRFRAME_FIELDS:
            if not field.derived:
                continue
            label = self.derived_labels.get(field.name)
            if label is not None:
                label.configure(text=_fmt(values.get(field.key)))
            hint = self.preview_labels.get(field.name)
            if hint is not None:
                hint.configure(text=_fmt(preview.get(field.key)))
            entry = self.entries.get(field.name)
            if entry is not None:
                entry.configure(state="disabled" if auto_on else "normal")

        self._refresh_tilt_levers(known, preview)
        self._refresh_status(values, preview, auto_on)

    def _refresh_tilt_levers(self, known: dict[str, float], preview: dict[str, float]) -> None:
        """重心优先用飞控回报的派生值；飞控还没回报时退回部件表推算，并如实标注。"""
        firmware_cg = known.get("cg_z_m")
        cg = firmware_cg if firmware_cg is not None else preview.get("cg_z_m")
        levers = tilt_axis_to_cg_z({**known, "cg_z_m": cg})
        for axis, label in self.tilt_lever_labels.items():
            label.configure(text=_fmt(levers.get(axis)))
        change = self.lever_change_text(self._pending_edits())
        self.tilt_lever_change_var.set(("还没写入的改动：\n" + change) if change else "")
        if firmware_cg is not None:
            self.tilt_lever_source_var.set(f"重心取飞控派生值 {_fmt(firmware_cg)} m")
        elif any(v is not None for v in levers.values()):
            self.tilt_lever_source_var.set("飞控尚未回报重心，暂按部件表推算")
        else:
            self.tilt_lever_source_var.set("")

    def _refresh_status(self, values, preview, auto_on: bool) -> None:
        """状态以**飞控的 ARM 报文**为准，本地判断只在还没连上时兜底。

        两边都能算"模型有没有效"，但只有飞控那份决定能不能解锁。本地那份写在
        airframe_model.first_missing() 里，是为了让没连线时页面也能提示；
        一旦收到 ARM 报文就改用它，免得出现"页面说没问题、飞机不肯解锁"。
        """
        arm = getattr(self.panel, "arm_status", None)
        if arm and arm.get("airframe_missing") not in (None, ""):
            missing = arm.get("airframe_missing")
            if missing == "-":
                self.status_var.set("模型有效 · 飞控已接受")
                self.detail_var.set("")
                return
            self.status_var.set(f"模型无效 · 解锁被挡住：{describe_invalid(missing)}")
            self.detail_var.set("飞控口径。填好后点「写入飞控」，再「保存到 Flash」。")
            return

        merged = {k: v for k, v in values.items() if v is not None}
        merged.update({k: v for k, v in preview.items() if merged.get(k) is None})
        missing = first_missing(merged)
        if missing is None:
            self.status_var.set("本地看起来完整 · 等待飞控确认")
            self.detail_var.set("尚未收到飞控的 ARM 状态，以飞控回报为准。")
        else:
            self.status_var.set(f"模型不完整：{describe_invalid('airframe.' + missing)}")
            self.detail_var.set(
                "本地判断（尚未连上飞控）。这些是零值或方向矛盾会让控制律失去物理含义的项。")

    def _on_destroy(self, event) -> None:
        if event.widget is self.panel:
            self._disposed = True
            if self._timer is not None:
                try:
                    self.panel.after_cancel(self._timer)
                except tk.TclError:
                    pass
                self._timer = None


def _safe_float(text) -> float | None:
    try:
        return float(str(text).strip())
    except (TypeError, ValueError):
        return None


def mount_airframe(panel, parent) -> AirframePage:
    page = AirframePage(parent, panel)
    page.pack(fill=tk.BOTH, expand=True)
    panel.airframe_page = page
    return page
