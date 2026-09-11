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
  高级（估算/辨识/反推）—— 惯量、下桨旋向、有效力臂、重力、舵机标度。
      默认折叠，改之前要二次确认。

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
    AirframeField,
    compute_derived,
    first_missing,
)
from ..parameter_model import validate_parameter_text
from ..proto import PROTO_REQ_PARAMS, PROTO_REQ_PARAM_SET, PROTO_REQ_SAVE


REFRESH_MS = 400

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

    # ── 高级层二次确认 ────────────────────────────────────────────────
    def _unlock_advanced(self) -> None:
        ok = messagebox.askokcancel(
            "解锁高级机体参数",
            "下面这些不是量出来的：\n\n"
            "· 转动惯量 I_xx/I_yy/I_zz —— 估计值，没有实测\n"
            "· 下桨旋向 —— 反推值，由「偏航高增益抖振而非发散」推出；"
            "它单独决定偏航力矩极性，翻错了是正反馈\n"
            "· 有效力臂 —— 2026-07-25 系统辨识结果，不是卷尺量的距离\n\n"
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
        current = self._target("airframe.derived_auto")
        new = 0.0 if (current is not None and abs(current) > 0.5) else 1.0
        self._send_param("airframe.derived_auto", f"{new:.0f}")

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

    def _write_all(self) -> None:
        """只发改过的字段。

        全量重发看着更简单，但它会把派生值也一起送出去——自动档下飞控逐条拒绝，
        界面上就是一串没头没脑的 ERR，真正改错的那条反而淹了。
        """
        sent = 0
        skipped: list[str] = []
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
            if self._send_param(name, text):
                sent += 1
            else:
                skipped.append(name)
        if sent == 0 and not skipped:
            self.detail_var.set("没有改动需要写入")
        elif skipped:
            self.detail_var.set(f"已写入 {sent} 项；{len(skipped)} 项被拒：{skipped[0]}")
        else:
            self.detail_var.set(f"已写入 {sent} 项到飞控 RAM，确认无误后请保存到 Flash")

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

        self._refresh_status(values, preview, auto_on)

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
            self.status_var.set(f"模型无效 · 解锁被挡住：缺 {missing}")
            self.detail_var.set("飞控口径。填好后点「写入飞控」，再「保存到 Flash」。")
            return

        merged = {k: v for k, v in values.items() if v is not None}
        merged.update({k: v for k, v in preview.items() if merged.get(k) is None})
        missing = first_missing(merged)
        if missing is None:
            self.status_var.set("本地看起来完整 · 等待飞控确认")
            self.detail_var.set("尚未收到飞控的 ARM 状态，以飞控回报为准。")
        else:
            self.status_var.set(f"模型不完整：缺 airframe.{missing}")
            self.detail_var.set(
                "本地判断（尚未连上飞控）。这些是零值会让控制律失去物理含义的项。")

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
