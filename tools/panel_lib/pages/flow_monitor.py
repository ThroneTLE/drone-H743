"""“传感器 · 光流”实时监控页及其遥测行处理。

和“校准 · 光流与测距”那一页的分工：那页是标定专用通路，只在采集期间轮询、
只服务于零偏/比例/旋转补偿的证据采集。本页是常驻监视——自己的可见性门控轮询、
自己的解析状态、自己的缓冲，与标定页零耦合。

数据全部来自既有文本命令 `FLOW?`，不新增任何协议。
"""

from __future__ import annotations

import time
import tkinter as tk
from collections import deque
from tkinter import ttk

from ..plotting import Figure, FigureCanvasTkAgg, HAS_MATPLOTLIB, MATPLOTLIB_ERROR
from ..proto import parse_kv, safe_int


# 固件内部约 10 Hz 更新；一次 FLOW? 回六行，5 Hz 已经够画曲线，
# 再快只是把链路带宽让给重复帧。
FLOW_MONITOR_POLL_PERIOD_S = 0.20
FLOW_MONITOR_RENDER_PERIOD_NS = 300_000_000
FLOW_MONITOR_MAX_SAMPLES = 900          # 5 Hz × 900 ≈ 3 分钟
FLOW_MONITOR_MAX_TRACK_POINTS = 900
# 单次积分允许跨越的最大真实时长。断连、切页、界面卡顿都会留下长空档，
# 越过这个阈值就只重新起锚点，不拿一个旧速度去外推几秒的位移。
FLOW_MONITOR_MAX_INTEGRATION_DT_S = 0.5
FLOW_MONITOR_QUALITY_FULL_SCALE = 255   # 固件里 flow_quality 是 uint8


class FlowMonitorPageMixin:
    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------

    def _init_flow_monitor_state(self) -> None:
        self.flow_monitor_values: dict[str, str] = {}
        self.flow_monitor_samples: deque[tuple[float, float, float, bool]] = deque(
            maxlen=FLOW_MONITOR_MAX_SAMPLES
        )
        self.flow_monitor_track: deque[tuple[float, float]] = deque(
            maxlen=FLOW_MONITOR_MAX_TRACK_POINTS
        )
        self.flow_monitor_last_poll = 0.0
        self.flow_monitor_tab_visible = False
        self.flow_monitor_dirty = False
        self.flow_monitor_last_render_ns = 0
        self.flow_monitor_anchor: tuple[float, float, float] | None = None
        self.flow_monitor_dx_m = 0.0
        self.flow_monitor_dy_m = 0.0
        self.flow_monitor_frames = 0
        self.flow_monitor_invalid_frames = 0
        self.flow_monitor_vars: dict[str, tk.StringVar] = {}
        self.flow_monitor_value_labels: dict[str, ttk.Label] = {}
        self.flow_monitor_quality_value = tk.DoubleVar(value=0.0)
        self.flow_monitor_velocity_figure = None
        self.flow_monitor_velocity_axis = None
        self.flow_monitor_velocity_canvas = None
        self.flow_monitor_track_figure = None
        self.flow_monitor_track_axis = None
        self.flow_monitor_track_canvas = None

    # ------------------------------------------------------------------
    # 页面
    # ------------------------------------------------------------------

    def _build_sensor_flow_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="FLOW  /  只读实时监控", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="光流质量、激光高度、速度与累计位移", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=(
                "本页选中时才按 5 Hz 发 FLOW?，切走即停。质量、高度、速度直接来自固件回包；"
                "累计位移是上位机按真实经过时间对速度做的梯形积分，固件并不保存这个量，"
                "所以“重置”只清本地累加，不会向飞控发任何命令。"
            ),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=2)
        panes.add(right, weight=3)

        self._build_flow_monitor_readouts(left)
        self._build_flow_monitor_displacement(left)
        self._build_flow_monitor_plots(right)
        self._flow_monitor_refresh_readouts()

    def _build_flow_monitor_readouts(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="实时读数", padding=10)
        box.pack(fill=tk.X)
        box.columnconfigure(1, weight=1)

        rows = (
            ("valid", "数据有效性"),
            ("quality", "光流质量"),
            ("height", "激光高度"),
            ("velocity", "融合速度"),
            ("age", "数据时延"),
            ("frames", "采样计数"),
        )
        for row, (key, label) in enumerate(rows):
            self.flow_monitor_vars[key] = tk.StringVar(value="-")
            ttk.Label(box, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=3)
            value = ttk.Label(box, textvariable=self.flow_monitor_vars[key], style="Mono.TLabel")
            value.grid(row=row, column=1, sticky=tk.W, pady=3)
            self.flow_monitor_value_labels[key] = value

        bar = ttk.Progressbar(
            box, orient=tk.HORIZONTAL, mode="determinate",
            maximum=float(FLOW_MONITOR_QUALITY_FULL_SCALE),
            variable=self.flow_monitor_quality_value,
        )
        bar.grid(row=len(rows), column=0, columnspan=2, sticky=tk.EW, pady=(6, 0))
        self.flow_monitor_quality_bar = bar

    def _build_flow_monitor_displacement(self, parent: ttk.Frame) -> None:
        box = ttk.LabelFrame(parent, text="累计位移（上位机按真实时间积分）", padding=10)
        box.pack(fill=tk.X, pady=(10, 0))
        box.columnconfigure(1, weight=1)

        for row, (key, label) in enumerate((("dx", "X 累计"), ("dy", "Y 累计"))):
            self.flow_monitor_vars[key] = tk.StringVar(value="+0.000 m")
            ttk.Label(box, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=3)
            value = ttk.Label(box, textvariable=self.flow_monitor_vars[key], style="Mono.TLabel")
            value.grid(row=row, column=1, sticky=tk.W, pady=3)
            self.flow_monitor_value_labels[key] = value

        self.flow_monitor_reset_button = ttk.Button(
            box, text="重置累计位移", command=self._flow_monitor_reset_displacement,
            style="Secondary.TButton",
        )
        self.flow_monitor_reset_button.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(8, 0))
        ttk.Label(
            box,
            text="速度无效的采样一律不参与积分，那段时间按“暂停”处理，既不补 0 也不外推。",
            style="Muted.TLabel", wraplength=380, justify=tk.LEFT,
        ).grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))

    def _build_flow_monitor_plots(self, parent: ttk.Frame) -> None:
        velocity_box = ttk.LabelFrame(parent, text="速度曲线 vx / vy (m/s)", padding=8)
        velocity_box.pack(fill=tk.BOTH, expand=True)
        track_box = ttk.LabelFrame(parent, text="累计位移轨迹 (m)", padding=8)
        track_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        if not (HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None):
            for box in (velocity_box, track_box):
                ttk.Label(
                    box,
                    text=(
                        "未安装 matplotlib，曲线区停用。安装后重启面板即可绘图。\n"
                        f"python -m pip install matplotlib\n{MATPLOTLIB_ERROR}"
                    ),
                    wraplength=520,
                ).pack(fill=tk.X, pady=(6, 0))
            return

        self.flow_monitor_velocity_figure = Figure(figsize=(5, 2.6), dpi=100)
        self.flow_monitor_velocity_axis = self.flow_monitor_velocity_figure.add_subplot(111)
        self.flow_monitor_velocity_canvas = FigureCanvasTkAgg(
            self.flow_monitor_velocity_figure, master=velocity_box
        )
        self.flow_monitor_velocity_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.flow_monitor_track_figure = Figure(figsize=(5, 2.6), dpi=100)
        self.flow_monitor_track_axis = self.flow_monitor_track_figure.add_subplot(111)
        self.flow_monitor_track_canvas = FigureCanvasTkAgg(
            self.flow_monitor_track_figure, master=track_box
        )
        self.flow_monitor_track_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._update_flow_monitor_plots()

    # ------------------------------------------------------------------
    # 遥测
    # ------------------------------------------------------------------

    @staticmethod
    def _flow_monitor_section(line: str) -> str:
        """把一行 FLOW 回包归到它自己的段：status / mico / data / raw / comp / ..."""
        tail = line[len("FLOW "):].strip() if line.startswith("FLOW ") else ""
        head = tail.split(" ", 1)[0] if tail else ""
        return "status" if "=" in head else head

    def _flow_monitor_handle_line(self, line: str) -> None:
        """FLOW? 回包逐行喂进来，本页只收 status / mico / data 三段。

        不能像诊断页那样把所有 FLOW 行无差别 merge 进一个字典：`FLOW comp` 那行
        也带 `valid=`，但它说的是旋转补偿快照有没有取到，与光流自身的 valid 不是
        一回事；merge 会把状态行的 valid 冲掉，让本页的有效性标志跟着补偿快照走。
        """
        if not hasattr(self, "flow_monitor_values"):
            return
        section = self._flow_monitor_section(line)
        if section not in ("status", "mico", "data"):
            return
        self.flow_monitor_values.update(parse_kv(line))
        if section != "data":
            return
        # data 段是一次回包里最后一段带速度和高度的，走到这里三段已经齐了。
        self._flow_monitor_ingest_frame(time.monotonic())
        self._flow_monitor_refresh_readouts()

    def _flow_monitor_ingest_frame(self, now: float) -> None:
        values = self.flow_monitor_values
        velocity_valid = values.get("valid") == "1" and values.get("vel_valid") == "1"
        vx = safe_int(values.get("vx_mm_s"), 0) * 0.001
        vy = safe_int(values.get("vy_mm_s"), 0) * 0.001
        self.flow_monitor_frames += 1
        self.flow_monitor_samples.append((now, vx, vy, velocity_valid))
        self._flow_monitor_integrate(now, vx, vy, velocity_valid)
        self.flow_monitor_dirty = True

    def _flow_monitor_integrate(
        self, now: float, vx: float, vy: float, velocity_valid: bool
    ) -> None:
        """对真实经过的墙钟时间做梯形积分，速度无效时暂停。

        选“暂停”而不是“把无效样本当 0 速度”：后者会在传感器丢帧时把真实发生过的
        位移抹平成静止；也不沿用上一次有效速度外推，那会在丢帧期间凭空造出位移。
        暂停的代价只是那段时间不计数，读数偏保守，不会撒谎。
        """
        anchor = self.flow_monitor_anchor
        if not velocity_valid:
            self.flow_monitor_anchor = None
            self.flow_monitor_invalid_frames += 1
            return
        self.flow_monitor_anchor = (now, vx, vy)
        if anchor is None:
            return
        dt = now - anchor[0]
        if dt <= 0.0 or dt > FLOW_MONITOR_MAX_INTEGRATION_DT_S:
            return
        self.flow_monitor_dx_m += 0.5 * (anchor[1] + vx) * dt
        self.flow_monitor_dy_m += 0.5 * (anchor[2] + vy) * dt
        self.flow_monitor_track.append((self.flow_monitor_dx_m, self.flow_monitor_dy_m))

    def _flow_monitor_reset_displacement(self) -> None:
        """只清上位机自己的积分状态。

        固件里没有“累计位移”这个量——它是本页拿 vx/vy 对时间积出来的，
        所以归零不需要、也不该向飞控发任何帧。
        """
        self.flow_monitor_dx_m = 0.0
        self.flow_monitor_dy_m = 0.0
        self.flow_monitor_track.clear()
        self.flow_monitor_anchor = None
        self.flow_monitor_frames = 0
        self.flow_monitor_invalid_frames = 0
        self._flow_monitor_refresh_readouts()
        self.flow_monitor_last_render_ns = time.monotonic_ns()
        self._update_flow_monitor_track_plot()

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------

    def _flow_monitor_mark(self, key: str, ok: bool | None) -> None:
        label = self.flow_monitor_value_labels.get(key)
        if label is None:
            return
        if ok is None:
            label.configure(style="Mono.TLabel")
        else:
            label.configure(style="Pass.TLabel" if ok else "Fail.TLabel")

    def _flow_monitor_refresh_readouts(self) -> None:
        if not self.flow_monitor_vars:
            return
        values = self.flow_monitor_values
        if not values:
            for key in ("valid", "quality", "height", "velocity", "age"):
                self.flow_monitor_vars[key].set("尚无 FLOW 回包")
                self._flow_monitor_mark(key, None)
            self.flow_monitor_vars["frames"].set("0 帧")
            self._flow_monitor_refresh_displacement()
            return

        ok = values.get("ok") == "1"
        valid = values.get("valid") == "1"
        velocity_valid = valid and values.get("vel_valid") == "1"
        height_valid = valid and values.get("height_valid") == "1"
        quality = safe_int(values.get("quality"), 0)
        min_quality = safe_int(values.get("min_q"), 0)
        height_m = safe_int(values.get("height_mm"), 0) * 0.001
        height_raw_m = safe_int(values.get("height_raw_mm"), 0) * 0.001
        vx = safe_int(values.get("vx_mm_s"), 0) * 0.001
        vy = safe_int(values.get("vy_mm_s"), 0) * 0.001

        if not ok:
            self.flow_monitor_vars["valid"].set("模块异常：固件报 ok=0")
        else:
            self.flow_monitor_vars["valid"].set("有效" if valid else "数据无效")
        self._flow_monitor_mark("valid", ok and valid)

        self.flow_monitor_vars["quality"].set(f"{quality} / 最低 {min_quality}")
        self.flow_monitor_quality_value.set(float(quality))
        self._flow_monitor_mark("quality", quality >= min_quality)

        self.flow_monitor_vars["height"].set(
            f"{height_m:.3f} m（原始 {height_raw_m:.3f} m）" if height_valid
            else f"{height_m:.3f} m —— 数据无效"
        )
        self._flow_monitor_mark("height", height_valid)

        self.flow_monitor_vars["velocity"].set(
            f"vx {vx:+.3f}   vy {vy:+.3f} m/s" if velocity_valid
            else f"vx {vx:+.3f}   vy {vy:+.3f} m/s —— 数据无效，未计入累计位移"
        )
        self._flow_monitor_mark("velocity", velocity_valid)

        self.flow_monitor_vars["age"].set(f"{safe_int(values.get('age_ms'), 0)} ms")
        self._flow_monitor_mark("age", None)
        self._flow_monitor_refresh_displacement()

    def _flow_monitor_refresh_displacement(self) -> None:
        self.flow_monitor_vars["frames"].set(
            f"{self.flow_monitor_frames} 帧，其中 {self.flow_monitor_invalid_frames} 帧速度无效"
        )
        self.flow_monitor_vars["dx"].set(f"{self.flow_monitor_dx_m:+.3f} m")
        self.flow_monitor_vars["dy"].set(f"{self.flow_monitor_dy_m:+.3f} m")

    def _flow_monitor_tick(self) -> None:
        now_ns = time.monotonic_ns()
        if (
            self.flow_monitor_dirty
            and self.flow_monitor_tab_visible
            and (now_ns - self.flow_monitor_last_render_ns) >= FLOW_MONITOR_RENDER_PERIOD_NS
        ):
            self.flow_monitor_dirty = False
            self.flow_monitor_last_render_ns = now_ns
            self._update_flow_monitor_plots()
        self.after(250, self._flow_monitor_tick)

    def _update_flow_monitor_plots(self) -> None:
        self._update_flow_monitor_velocity_plot()
        self._update_flow_monitor_track_plot()

    def _update_flow_monitor_velocity_plot(self) -> None:
        axis = self.flow_monitor_velocity_axis
        canvas = self.flow_monitor_velocity_canvas
        if not HAS_MATPLOTLIB or axis is None or canvas is None:
            return
        samples = list(self.flow_monitor_samples)
        axis.clear()
        axis.set_xlabel("time (s)")
        axis.set_ylabel("m/s")
        axis.grid(True, alpha=0.3)
        if samples:
            base = samples[0][0]
            times = [sample[0] - base for sample in samples]
            # 无效样本画成 NaN，曲线在那里断开——一眼看得出哪一段数据不能用。
            axis.plot(times, [s[1] if s[3] else float("nan") for s in samples],
                      linewidth=1.4, label="vx")
            axis.plot(times, [s[2] if s[3] else float("nan") for s in samples],
                      linewidth=1.4, label="vy")
            axis.legend(loc="upper right", fontsize=8)
        else:
            axis.text(0.5, 0.5, "waiting for FLOW samples", ha="center", va="center",
                      transform=axis.transAxes)
        self.flow_monitor_velocity_figure.tight_layout()
        canvas.draw_idle()

    def _update_flow_monitor_track_plot(self) -> None:
        axis = self.flow_monitor_track_axis
        canvas = self.flow_monitor_track_canvas
        if not HAS_MATPLOTLIB or axis is None or canvas is None:
            return
        track = list(self.flow_monitor_track)
        axis.clear()
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.grid(True, alpha=0.3)
        axis.set_aspect("equal", adjustable="datalim")
        if track:
            xs = [point[0] for point in track]
            ys = [point[1] for point in track]
            axis.plot(xs, ys, linewidth=1.2)
            axis.scatter([xs[-1]], [ys[-1]], s=40, color="#d62728", zorder=3)
            pad = max(max(xs) - min(xs), max(ys) - min(ys), 0.2) * 0.1
            axis.set_xlim(min(xs) - pad, max(xs) + pad)
            axis.set_ylim(min(ys) - pad, max(ys) + pad)
        else:
            axis.text(0.5, 0.5, "no integrated displacement yet", ha="center", va="center",
                      transform=axis.transAxes)
        self.flow_monitor_track_figure.tight_layout()
        canvas.draw_idle()


__all__ = [
    "FLOW_MONITOR_MAX_INTEGRATION_DT_S",
    "FLOW_MONITOR_MAX_SAMPLES",
    "FLOW_MONITOR_MAX_TRACK_POINTS",
    "FLOW_MONITOR_POLL_PERIOD_S",
    "FLOW_MONITOR_QUALITY_FULL_SCALE",
    "FLOW_MONITOR_RENDER_PERIOD_NS",
    "FlowMonitorPageMixin",
]
