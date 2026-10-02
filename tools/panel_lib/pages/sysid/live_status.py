"""顶部大字状态、程序油门与台架几何的输入、每秒一次的 `SYSID THR?` 轮询。

轮询只在本页可见、已连接、没有事务在等回显、上一份轮询回复已回来时才发，
所以它不会插进开始前的配置核对里（见 `workflow.py` 文件头的记账说明）。
"""

from __future__ import annotations

import math
import time
import tkinter as tk

from .banner import TONE_STYLES, BannerInput, describe, rc_summary
from .geometry import (PIVOT_MISMATCH_M, SOURCE_SERVO, airframe_servo_pivot, fc_above_cg_m,
                       needed_pivots, parse_length, pivot_prefill)

#: STM32 应用固件的 USB CDC 身份。
USB_CDC_VID_PID = (0x0483, 0x5740)

#: 后台状态轮询周期。
POLL_MS = 1000
#: 遥控器状态多久没刷新就当作"未知"：过期的"已解锁"不能冒充当前状态。
THR_FRESH_S = 3.5

STOP_TEXT_AUTO = "停止（电机回到遥控器油门）"
STOP_TEXT_MANUAL = "停止辨识"
STOP_HINT_AUTO = "停止或中止后电机立即回到遥控器油门（油门杆在最低 = 怠速）；舵机保持回中，上锁后释放。"
STOP_HINT_MANUAL = "停止辨识不会停电机！结束或中止后，请用遥控器收油门并锁定。"


class LiveStatus:
    def _current_thr(self) -> dict | None:
        if self.thr is None or self.thr_generation != self.workflow.generation():
            return None
        if time.monotonic() - self.thr_time > THR_FRESH_S:
            return None
        return self.thr

    def current_fw_ver(self) -> int | None:
        """READY 行报的 SYSID 版本；换了连接就作废（新连接可能是另一块板/另一版固件）。"""
        if self.fw_ver_generation != self.workflow.generation():
            return None
        return self.fw_ver

    def firmware_outdated(self) -> bool:
        version = self.current_fw_ver()
        return (version is not None and version < 3) or not self.workflow.thr_poll_allowed()

    def refresh_banner(self) -> None:
        if hasattr(self, "refresh_alt_height"):
            self.refresh_alt_height()         # 「Z 高度」页的实时测距跟着 THR 回报与每秒轮询刷新
        if hasattr(self, "refresh_xy_live"):
            self.refresh_xy_live()            # XY 页的实时位置/速度同理
        if hasattr(self, "refresh_yaw_live"):
            self.refresh_yaw_live()           # 偏航页的实时偏航角速度/偏航角（最近一个记录样本）
        if not hasattr(self, "banner_detail_label"):
            return
        if hasattr(self, "refresh_notch_controls"):
            self.refresh_notch_controls()     # 陷波开关跟着解锁/运行状态灰显
        if hasattr(self, "refresh_backlash_controls"):
            self.refresh_backlash_controls()  # 回差补偿开关同上
        w = self.workflow
        active_control = ((w.snapshot or {}).get("alt_request") or {}).get("control")
        alt_control = active_control if (w.run_id is not None or w.awaiting == "SYSID START") else (
            "breakaway" if self.mode_var.get() == "ALT" and self.alt_inject_var.get() == "break"
            else "closed_loop" if self.mode_var.get() == "ALT" else None)
        transport = getattr(self.panel, "transport", None)
        alt = self.alt_run_active() if hasattr(self, "alt_run_active") else False
        xy = self.xy_run_active() if hasattr(self, "xy_run_active") else False
        yaw = self.yaw_run_active() if hasattr(self, "yaw_run_active") else False
        state = BannerInput(
            connected=bool(transport is not None and getattr(transport, "is_connected", False)),
            # 高度 / 水平槽辨识总是程序油门：内环页的手动油门勾选对它们不起作用。
            manual=bool(self.manual_throttle_var.get()) and not (alt or xy or yaw),
            config_busy=bool(w.awaiting) and w.awaiting != "SYSID START",
            start_pending=w.awaiting == "SYSID START",
            stopping=self.stopping,
            run_active=w.run_id is not None and w.end is None,
            phase=self.phase, samples=len(self.samples), end=w.end,
            analysis=self.analysis_note, data_error=w.data_error, notice=w.notice,
            thr=self._current_thr(), fw_outdated=self.firmware_outdated(),
            mode=(w.snapshot or {}).get("mode"), servo=self.servo_mode_active(),
            amp_warning=self.amp_swing_warning() if hasattr(self, "amp_swing_warning") else "",
            alt=alt, alt_control=alt_control, xy=xy, yaw=yaw)
        title, detail, tone = describe(state)
        shown = (title, detail, tone, rc_summary(state.thr, alt_control))
        if shown == self._banner_shown:
            return
        self._banner_shown = shown
        self.banner_var.set(title)
        self.banner_detail_var.set(detail)
        # 「Z 高度」页、XY 页挂着同一组状态文字（altitude.py、horizontal.py），颜色一起跟。
        for label in (self.banner_detail_label, *getattr(self, "extra_banner_detail_labels", ())):
            label.configure(style=TONE_STYLES.get(tone, "TLabel"))
        self.rc_var.set(shown[3])

    def servo_mode_active(self) -> bool:
        """正在跑（或刚结束）的是舵机单独轮，或还没开始时「本轮做」选了 SERVO。"""
        w = self.workflow
        if w.run_id is not None or w.awaiting == "SYSID START":
            return str((w.snapshot or {}).get("mode")) == "3"
        return self.mode_var.get() == "SERVO"

    def refresh_thrust_hint(self) -> None:
        """内环页程序油门的灰字；高度页的合推力提示在 `alt_section.refresh_alt_hint`。"""
        text = self.target_thrust_var.get().strip()
        if text:
            self.thrust_hint_var.set(f"将使用 {text} N（清空则用机重）")
            return
        weight = self.workflow.weight_n()
        self.thrust_hint_var.set(
            f"留空 = 机重，当前 {round(weight, 1):.1f} N（来自飞控参数）" if weight is not None
            else "留空 = 机重（连上飞控后自动读取）")

    def geometry_inputs(self) -> tuple[float | None, float | None]:
        """`(杆到飞控, 直接填的杆到质心)`，单位 m；两个都空时抛中文 ValueError。"""
        override = parse_length(self.axis_override_var.get(), "杆到质心距离（覆盖）")
        rod = parse_length(self.rod_to_fc_var.get(), "杆到飞控板的垂直距离")
        if rod is None and override is None:
            raise ValueError("先用尺子量「杆到飞控板的垂直距离」填在「准备」页"
                             "（杆中心到飞控板，杆在飞控上方为正），再点开始。")
        return rod, override

    def pivot_inputs(self) -> tuple[float | None, float | None]:
        """`(横滚倾转轴到飞控板, 俯仰倾转轴到飞控板)`，m；没填是 None，填错抛中文 ValueError。"""
        return (parse_length(self.roll_pivot_var.get(), "横滚倾转轴到飞控板的垂直距离"),
                parse_length(self.pitch_pivot_var.get(), "俯仰倾转轴到飞控板的垂直距离"))

    def current_geometry(self, fc: float, fallback_d: float) -> dict:
        """拟合用的几何：「1 · 准备」页**当前**填写的值 + 那一轮快照里的飞控到质心 `fc`。

        杆到质心 d = 覆盖值，或 杆到飞控 + fc；两项都空时退回那一轮下发的 d。
        倾转轴到质心 = 倾转轴到飞控 + fc，没填是 None（拟合会给 blocker）。填错抛中文 ValueError。
        """
        override = parse_length(self.axis_override_var.get(), "杆到质心距离（覆盖）")
        rod = parse_length(self.rod_to_fc_var.get(), "杆到飞控板的垂直距离")
        d = override if override is not None else (rod + fc if rod is not None else fallback_d)
        roll, pitch = self.pivot_inputs()
        roll = None if roll is None else roll + fc
        pitch = None if pitch is None else pitch + fc

        def show(value):
            return "没填" if value is None else f"{value:+.3f} m"

        text = (f"几何：杆到质心 d = {d:.3f} m；俯仰倾转轴到质心 {show(pitch)}；"
                f"横滚倾转轴到质心 {show(roll)}（飞控到质心 {fc:+.3f} m 取自那一轮）")
        return dict(d=d, roll=roll, pitch=pitch, text=text)

    def _prefill_pivots(self) -> None:
        """机体参数里有倾转轴位置、而输入框还空着时，替你预填（可改）。"""
        for axis, variable in (("roll", self.roll_pivot_var), ("pitch", self.pitch_pivot_var)):
            if variable.get().strip():
                continue
            found = pivot_prefill(self.workflow.params, axis)
            if found is not None:
                variable.set(f"{found[0]:.3f}")

    def _pivot_notes(self, pivots) -> tuple[list[str], list[str]]:
        """`(来源说明, 不一致提醒)`：对照机体参数里的舵机转轴高度（固件算力矩用的就是它）。"""
        notes, warnings = [], []
        for axis, label, value in (("roll", "横滚", pivots[0]), ("pitch", "俯仰", pivots[1])):
            airframe = airframe_servo_pivot(self.workflow.params, axis)
            if value is None or airframe is None:
                continue
            if abs(value - airframe) <= PIVOT_MISMATCH_M:
                if (pivot_prefill(self.workflow.params, axis) or (None, None))[1] == SOURCE_SERVO:
                    notes.append(f"{label}倾转轴：来自机体参数的舵机转轴高度（与固件力矩模型同源）")
            else:
                warnings.append(f"{label}倾转轴 {value:.3f} m 与机体参数里的舵机转轴高度 "
                                f"{airframe:.3f} m 不一致：固件用机体参数算力矩，建议两边改成同一个值")
        return notes, warnings

    def update_pivot_rows(self) -> None:
        """按杆轴方向只露出需要的倾转轴输入：Pitch 只要俯仰，Roll 只要横滚，斜向两个都要。"""
        rows = getattr(self, "pivot_rows", None)
        if not rows:
            return
        try:
            need_roll, need_pitch = needed_pivots(float(self.psi_var.get()))
        except ValueError:
            need_roll = need_pitch = True
        for widgets, shown in ((rows["roll"], need_roll), (rows["pitch"], need_pitch)):
            for widget in widgets:
                widget.grid() if shown else widget.grid_remove()

    def refresh_geometry_hint(self) -> None:
        if not hasattr(self, "geometry_hint_var"):
            return
        self._prefill_pivots()
        found = fc_above_cg_m(self.workflow.params, self.workflow.ready)
        fc_text = ("飞控到质心：连上飞控后自动从机体参数读取" if found is None else
                   f"飞控到质心：{found[0]:+.3f} m（来自{found[1]}，飞控在质心上方为正）")
        try:
            rod, override = self.geometry_inputs()
        except ValueError:
            rod = override = None
        if override is not None:
            d_text = f"杆到质心 d = {override:.3f} m（用的是「高级设置」里直接填的值）"
        elif rod is None:
            d_text = "杆到质心 d：填上杆到飞控的距离后自动算出"
        elif found is None:
            d_text = f"杆到质心 d = {rod:.3f} m + 飞控到质心（待读取）"
        else:
            d_text = f"杆到质心 d = {rod:.3f} + {found[0]:.3f} = {rod + found[0]:.3f} m"
        lines = [fc_text, d_text]
        try:
            pivots = self.pivot_inputs()
        except ValueError:
            pivots = (None, None)
        for label, value in zip(("横滚", "俯仰"), pivots):
            if value is not None and found is not None:
                lines.append(f"{label}倾转轴到质心 = {value:.3f} + {found[0]:.3f} = {value + found[0]:.3f} m")
        notes, warnings = self._pivot_notes(pivots)
        self.geometry_hint_var.set("\n".join(lines + notes))
        self.pivot_warning_var.set("\n".join(warnings))

    def _on_throttle_mode(self) -> None:
        manual = bool(self.manual_throttle_var.get())
        self.stop_button.configure(text=STOP_TEXT_MANUAL if manual else STOP_TEXT_AUTO)
        self.stop_hint_var.set(STOP_HINT_MANUAL if manual else STOP_HINT_AUTO)
        self.refresh_banner()

    def throttle_settings(self) -> tuple[bool, float | None, float]:
        """`(手动?, 目标合推力或 None=机重, 最高油门%)`；填错抛 ValueError（中文）。

        ALT 读「Z 高度」页自己的合推力与最高油门，而且总是程序油门（手动 = False）。
        """
        if self.mode_var.get() == "YAW" and hasattr(self, "yaw_throttle_settings"):
            return self.yaw_throttle_settings()     # 偏航页：总推力 + 固定最高油门，总是程序油门
        alt = self.mode_var.get() == "ALT"
        xy = self.mode_var.get() == "XY"
        breakaway = alt and self.alt_inject_var.get() == "break"
        target_name = ("离地搜索上限" if breakaway else "托住推力" if xy else "目标合推力")
        fallback = ("" if breakaway else "，或留空用飞控悬停推力参数" if xy else "，或留空用机重")
        max_var = self.alt_max_pct_var if alt else self.xy_max_pct_var if xy else self.max_pct_var
        try:
            max_pct = float(max_var.get())
        except ValueError:
            raise ValueError("最高油门要填数字（10～95）") from None
        if not (math.isfinite(max_pct) and 10.0 <= max_pct <= 95.0):
            raise ValueError("最高油门要在 10%～95% 之间")
        text = (self.alt_target_var if alt else self.xy_target_var if xy
                else self.target_thrust_var).get().strip()
        target = None
        if text:
            try:
                target = float(text)
            except ValueError:
                raise ValueError(f"{target_name}要填数字（单位 N）{fallback}") from None
            if not (math.isfinite(target) and target > 0):
                raise ValueError(f"{target_name}要填正数（单位 N）{fallback}")
        return bool(self.manual_throttle_var.get()) and not (alt or xy), target, max_pct

    def _schedule_poll(self) -> None:
        if self.workflow.closed:
            return
        try:
            self._poll_job = self.parent.after(POLL_MS, self._poll_tick)
        except tk.TclError:
            self._poll_job = None

    def _poll_tick(self) -> None:
        self._poll_job = None
        if self.workflow.closed:
            return
        try:
            if not self.prime_status():
                self.poll_status()
            if hasattr(self, "notch_poll_idle"):
                self.notch_poll_idle()
            if hasattr(self, "backlash_poll_idle"):
                self.backlash_poll_idle()
            if hasattr(self, "yaw_poll_idle"):
                self.yaw_poll_idle()          # 偏航页在前台：隔几秒读一次单桨最大推力（THRMODE?）
            self.refresh_banner()
        finally:
            self._schedule_poll()

    def _visible_and_connected(self) -> bool:
        transport = getattr(self.panel, "transport", None)
        if transport is None or not getattr(transport, "is_connected", False):
            return False
        # 内环页、「Z 高度」页或 XY 页任一在前台都要轮询：几页共用这一个事务与顶部状态。
        views = [getattr(self, "alt_view", None), getattr(self, "xy_view", None),
                 getattr(self, "yaw_view", None)]
        try:
            return bool(self.parent.winfo_viewable() or
                        any(view is not None and view.parent.winfo_viewable() for view in views))
        except tk.TclError:
            return False

    def prime_status(self) -> bool:
        """新连接上本页第一次可见且空闲时，静默要一次 `SYSID?` + `PARAM?`。

        这样机重、飞控到质心、倾转轴预填在点开始之前就能显示出来。两条都走现有簿记：
        `SYSID?` 的整份报告记在"非事务报告"账上，不会被当成配置回显。
        """
        w = self.workflow
        generation = w.generation()
        if self._primed_generation == generation or w.closed or w.awaiting:
            return False
        if w.run_id is not None and w.end is None:
            return False
        if not self._visible_and_connected():
            return False
        if self.send("SYSID?", quiet=True) and self.send("PARAM?", quiet=True):
            self._primed_generation = generation
            return True
        return False

    def link_is_usb(self) -> bool:
        """当前链路是不是 USB CDC。蓝牙/网络一定不是；串口按 USB 身份判断，判断不了算是。"""
        panel = self.panel
        transport = getattr(panel, "transport", None)
        serial = getattr(panel, "serial_transport", None)
        if serial is not None and transport is not serial:
            return False
        variable = getattr(panel, "transport_var", None)
        try:
            mode = str(variable.get()) if variable is not None else ""
        except tk.TclError:
            mode = ""
        if mode in ("蓝牙", "bluetooth", "tcp", "udp"):
            return False
        port = getattr(transport, "active_port", None)
        identities = getattr(panel, "_serial_port_identity", None) or {}
        identity = identities.get(str(port)) if port else None
        if identity and identity.get("vid") is not None:
            return (identity.get("vid"), identity.get("pid")) == USB_CDC_VID_PID
        return True

    def poll_status(self) -> bool:
        """每秒一次 `SYSID THR?`（只回一行）：让「是否解锁 / 油门杆是否在最低」保持新鲜。

        有事务在等回显、或上一份轮询回复还没回来时不发——轮询绝不能插进配置核对里。
        """
        w = self.workflow
        if w.closed or w.awaiting or w.thr_polls_in_flight() or self.firmware_outdated():
            return False
        if not self._visible_and_connected():
            return False
        return self.send("SYSID THR?", quiet=True)

    def _on_destroy(self, event) -> None:
        if event.widget is not self.parent:
            return
        if self._poll_job is not None:
            try:
                self.parent.after_cancel(self._poll_job)
            except tk.TclError:
                pass
            self._poll_job = None
