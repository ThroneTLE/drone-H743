"""GPS validity, provenance and visibility-gated track rendering."""

from __future__ import annotations

import csv
import math
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox

from ..plotting import HAS_MATPLOTLIB
from ..proto import first_float, first_value, parse_kv, safe_int

try:
    from ...project_paths import TELEMETRY_DIR, dated_directory
except ImportError:  # Allows direct import from tools/panel_lib.
    try:
        from tools.project_paths import TELEMETRY_DIR, dated_directory
    except ImportError:
        from project_paths import TELEMETRY_DIR, dated_directory


MAX_GPS_TRACK_POINTS = 5000
MAX_GPS_RAW_SAMPLES = 5000
GPS_PLOT_PERIOD_NS = 200_000_000


class GpsValidityPageMixin:
    """Keep raw GPS lines, valid navigation points and last-known data separate."""

    def _init_gps_state(self) -> None:
        self.gps_track: list[dict[str, float | str | int | bool]] = []
        self.gps_raw_samples: deque[dict[str, object]] = deque(maxlen=MAX_GPS_RAW_SAMPLES)
        self.gps_last_known: dict[str, object] | None = None
        self.gps_status_values: dict[str, str] = {}
        self.gps_nav_valid = False
        self.gps_track_segment = 0
        self.gps_track_break_pending = False
        self.gps_origin_lat: float | None = None
        self.gps_origin_lon: float | None = None
        self.gps_last_plot_ns = 0
        self.gps_plot_dirty = False
        self.gps_track_line = None
        self.gps_track_marker = None
        self.gps_track_empty_text = None

    def _gps_is_visible(self) -> bool:
        sensor_group_tab = getattr(self, "sensor_group_tab", None)
        sensor_notebook = getattr(self, "sensor_notebook", None)
        gps_tab = getattr(self, "gps_tab", None)
        if sensor_group_tab is None or sensor_notebook is None or gps_tab is None:
            return False
        return (
            self.notebook.select() == str(sensor_group_tab)
            and sensor_notebook.select() == str(gps_tab)
        )

    def _bind_gps_visibility_events(self) -> None:
        sensor_notebook = getattr(self, "sensor_notebook", None)
        notebook = getattr(self, "notebook", None)
        if sensor_notebook is not None:
            sensor_notebook.bind("<<NotebookTabChanged>>", self._gps_on_tab_changed, add="+")
        if notebook is not None:
            notebook.bind("<<NotebookTabChanged>>", self._gps_on_tab_changed, add="+")

    def _gps_on_tab_changed(self, _event: tk.Event | None = None) -> None:
        if not self._gps_is_visible() or not self.gps_plot_dirty:
            return
        self.gps_plot_dirty = False
        self.gps_last_plot_ns = time.monotonic_ns()
        self._update_gps_plot()

    def _gps_record_raw(self, line: str, values: dict[str, str]) -> None:
        kind = line.split(maxsplit=2)[1] if len(line.split(maxsplit=2)) > 1 else "unknown"
        self.gps_raw_samples.append(
            {
                "received_at": time.time(),
                "kind": kind,
                "values": dict(values),
                "line": line,
            }
        )

    def _gps_lat_lon_from_values(self, values: dict[str, str]) -> tuple[float | None, float | None]:
        lat = first_float(values, "lat_deg", "latitude_deg", "latitude")
        lon = first_float(values, "lon_deg", "longitude_deg", "longitude")
        if lat is None:
            lat = first_float(values, "lat", "lat_deg_e7")
        if lon is None:
            lon = first_float(values, "lon", "lon_deg_e7")
        if lat is not None and abs(lat) > 90.0:
            lat /= 10000000.0
        if lon is not None and abs(lon) > 180.0:
            lon /= 10000000.0
        return lat, lon

    def _gps_xy_from_lat_lon(self, lat: float, lon: float) -> tuple[float, float]:
        if self.gps_origin_lat is None or self.gps_origin_lon is None:
            self.gps_origin_lat = lat
            self.gps_origin_lon = lon
        radius_m = 6378137.0
        lat0 = self.gps_origin_lat
        lon0 = self.gps_origin_lon
        x = math.radians(lon - lon0) * radius_m * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * radius_m
        return x, y

    @staticmethod
    def _gps_age_text(values: dict[str, str]) -> str:
        age = first_value(values, "age_ms", "age")
        if age in {"4294967295", "-1"}:
            return "过期"
        if age != "-" and age.isdigit():
            return f"{age} ms"
        return age

    def _gps_refresh_status(self, values: dict[str, str]) -> None:
        self.gps_status_values.update(values)
        if "valid" in values:
            self.gps_nav_valid = safe_int(values.get("valid"), 0) != 0
            if not self.gps_nav_valid:
                self.gps_track_break_pending = True

        ok = safe_int(values.get("ok"), 0) != 0
        init_value = values.get("init")
        initialized = ok and (init_value is None or safe_int(init_value, 1) == 0)
        fix_text = first_value(values, "fix", "fix_type")
        previous_fix = self.gps_vars.get("fix").get() if "fix" in self.gps_vars else "0"
        fix = safe_int(fix_text, safe_int(previous_fix, 0))
        sv_text = first_value(values, "sv", "num_sv")
        previous_sv = self.gps_vars.get("sv").get() if "sv" in self.gps_vars else "0"
        sv = safe_int(sv_text, safe_int(previous_sv, 0))

        if not ok and "ok" in values:
            state = "模块异常"
        elif "valid" in values and not self.gps_nav_valid:
            state = "无有效定位"
        elif self.gps_nav_valid:
            state = "已定位"
        elif initialized:
            state = "已初始化（无有效定位）"
        else:
            state = "等待 GPS"

        updates = {
            "state": state,
            "fix": str(fix) if fix_text != "-" else "-",
            "sv": str(sv) if sv_text != "-" else "-",
            "age": self._gps_age_text(values),
        }
        for key, value in updates.items():
            if key in self.gps_vars and value != "-":
                self.gps_vars[key].set(value)

        self._update_module(
            "GPS",
            state=state,
            stage=f"fix={fix}",
            value=f"sv={sv}",
            code=(
                f"valid={int(self.gps_nav_valid)} age_ms="
                f"{first_value(values, 'age_ms', 'age')}"
            ),
            hint=self._hardware_hint("GPS", self.gps_nav_valid, values),
            line="GPS " + " ".join(f"{key}={value}" for key, value in values.items()),
        )
        self.gps_status_var.set(f"{state}  fix={fix}  sv={sv}")

    def _gps_accept_position(self, line: str, values: dict[str, str]) -> None:
        lat, lon = self._gps_lat_lon_from_values(values)
        valid_position = (
            self.gps_nav_valid
            and lat is not None
            and lon is not None
            and abs(lat) <= 90.0
            and abs(lon) <= 180.0
        )
        if not valid_position:
            if "state" in self.gps_vars:
                self.gps_vars["state"].set(
                    "无有效定位" if not self.gps_nav_valid else "定位坐标无效"
                )
            self.gps_plot_dirty = True
            return

        if self.gps_track_break_pending:
            self.gps_track_segment += 1
            self.gps_track_break_pending = False
        x, y = self._gps_xy_from_lat_lon(lat, lon)
        fix = safe_int(first_value(self.gps_status_values, "fix", "fix_type"), 0)
        sv = safe_int(first_value(self.gps_status_values, "sv", "num_sv"), 0)
        point = {
            "time": time.time(),
            "lat": lat,
            "lon": lon,
            "x_m": x,
            "y_m": y,
            "fix": fix,
            "sv": sv,
            "hmsl_mm": safe_int(first_value(values, "hmsl_mm", "hmsl"), 0),
            "hacc_mm": safe_int(first_value(values, "hacc_mm", "hacc"), 0),
            "valid": True,
            "segment": self.gps_track_segment,
            "line": line,
        }
        self.gps_track.append(point)
        if len(self.gps_track) > MAX_GPS_TRACK_POINTS:
            del self.gps_track[: len(self.gps_track) - MAX_GPS_TRACK_POINTS]
        self.gps_last_known = dict(point)

        updates = {
            "state": "已定位",
            "lon": f"{lon:.7f}",
            "lat": f"{lat:.7f}",
            "hmsl": first_value(values, "hmsl_mm", "hmsl"),
            "x": f"{x:.2f}",
            "y": f"{y:.2f}",
        }
        heading = first_float(values, "head_e5", "heading_motion_deg_e5", "heading_deg_e5")
        if heading is not None:
            updates["hdg"] = f"{heading / 100000.0:.2f}"
        speed = first_value(values, "gspd", "ground_speed_mm_s", "speed_mm_s")
        if speed == "-":
            vn = first_float(values, "vn", "vel_n_mm_s")
            ve = first_float(values, "ve", "vel_e_mm_s")
            if vn is not None and ve is not None:
                speed = f"{math.hypot(vn, ve):.0f}"
        updates["spd"] = speed
        for key, value in updates.items():
            if key in self.gps_vars and value != "-":
                self.gps_vars[key].set(value)

        self._update_module(
            "GPS",
            state="已定位",
            stage=f"fix={fix}",
            value=f"fix={fix} sv={sv} lat={lat:.7f} lon={lon:.7f}",
            code=f"valid=1 age_ms={self.gps_status_values.get('age_ms', '-')}",
            hint=self._hardware_hint("GPS", True, values),
            line=line,
        )
        self.gps_count_var.set(f"轨迹点: {len(self.gps_track)}")
        self.gps_status_var.set(f"已定位  fix={fix}  sv={sv}")
        self.gps_plot_dirty = True
        now_ns = time.monotonic_ns()
        if self._gps_is_visible() and now_ns - self.gps_last_plot_ns >= GPS_PLOT_PERIOD_NS:
            self.gps_plot_dirty = False
            self.gps_last_plot_ns = now_ns
            self._update_gps_plot()

    def _update_gps_line(self, line: str) -> None:
        values = parse_kv(line)
        if not values:
            return
        self._gps_record_raw(line, values)
        has_status = bool({"ok", "init", "valid", "fix", "fix_type", "sv", "age_ms"} & values.keys())
        has_position = bool({"lat", "lon", "lat_deg", "lon_deg", "latitude", "longitude"} & values.keys())
        if has_status:
            self._gps_refresh_status(values)
        if has_position:
            self._gps_accept_position(line, values)

    def _update_gps_plot(self) -> None:
        if not HAS_MATPLOTLIB or self.gps_axis is None or self.gps_canvas is None:
            return
        axis = self.gps_axis
        if self.gps_track_line is None:
            self.gps_track_line, = axis.plot([], [], linewidth=1.2, marker=".", markersize=3)
            self.gps_track_marker = axis.scatter([], [], s=45, color="#d62728", zorder=3)
            self.gps_track_empty_text = axis.text(
                0.5, 0.5, "waiting for valid GPS", ha="center", va="center", transform=axis.transAxes
            )
            axis.set_title("M9N XY track")
            axis.set_xlabel("X east (m)")
            axis.set_ylabel("Y north (m)")
            axis.grid(True, alpha=0.3)
            axis.set_aspect("equal", adjustable="datalim")

        xs: list[float] = []
        ys: list[float] = []
        last_segment = None
        for point in self.gps_track:
            segment = point.get("segment")
            if last_segment is not None and segment != last_segment:
                xs.append(float("nan"))
                ys.append(float("nan"))
            xs.append(float(point["x_m"]))
            ys.append(float(point["y_m"]))
            last_segment = segment
        self.gps_track_line.set_data(xs, ys)
        if self.gps_track_marker is not None:
            if self.gps_track:
                last = self.gps_track[-1]
                self.gps_track_marker.set_offsets([[float(last["x_m"]), float(last["y_m"])]])
                self.gps_track_marker.set_visible(True)
            else:
                # Some Matplotlib releases reject an empty 1-D offsets array
                # with IndexError. Hiding the marker is the same visual result
                # and keeps the collection object reusable.
                self.gps_track_marker.set_visible(False)
        if self.gps_track_empty_text is not None:
            self.gps_track_empty_text.set_visible(not self.gps_track)
        if self.gps_track:
            finite_x = [value for value in xs if math.isfinite(value)]
            finite_y = [value for value in ys if math.isfinite(value)]
            pad = max(max(finite_x) - min(finite_x), max(finite_y) - min(finite_y), 2.0) * 0.08
            axis.set_xlim(min(finite_x) - pad, max(finite_x) + pad)
            axis.set_ylim(min(finite_y) - pad, max(finite_y) + pad)
        else:
            axis.relim()
            axis.autoscale_view()
        self.gps_figure.tight_layout()
        self.gps_canvas.draw_idle()

    def _clear_gps_track(self) -> None:
        self.gps_track.clear()
        self.gps_last_known = None
        self.gps_track_break_pending = False
        self.gps_track_segment = 0
        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self.gps_count_var.set("轨迹点: 0")
        self.gps_status_var.set("等待 GPS")
        self.gps_plot_dirty = True
        if self._gps_is_visible():
            self.gps_plot_dirty = False
            self.gps_last_plot_ns = time.monotonic_ns()
            self._update_gps_plot()

    def _export_gps_csv(self) -> None:
        if not self.gps_track:
            messagebox.showinfo("没有数据", "GPS 轨迹为空")
            return
        initial = dated_directory(TELEMETRY_DIR) / f"gps_track_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        initial.parent.mkdir(parents=True, exist_ok=True)
        filename = filedialog.asksaveasfilename(
            title="导出 GPS 有效轨迹",
            defaultextension=".csv",
            initialdir=str(initial.parent),
            initialfile=initial.name,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        fields = ["time", "lat", "lon", "x_m", "y_m", "fix", "sv", "hmsl_mm", "hacc_mm", "segment", "valid", "line"]
        with open(filename, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.gps_track)
        self._append(f"[上位机] 已导出 GPS 有效轨迹（无效原始行未冒充轨迹）: {filename}")


__all__ = ["GPS_PLOT_PERIOD_NS", "GpsValidityPageMixin", "MAX_GPS_RAW_SAMPLES", "MAX_GPS_TRACK_POINTS"]
