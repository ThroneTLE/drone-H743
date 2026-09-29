"""Share the original scale settings with the single H743 control session."""
from __future__ import annotations

from .legacy_calibration import LegacyCalibration, LegacyLoadCell


class PressureGuiBenchBridge:
    def __init__(self, gui) -> None:
        self.gui = gui
        self.bench_active = False

    def shared_variables(self) -> dict:
        return {
            "scale_port": self.gui.port_var,
            "scale_baud": self.gui.baud_var,
            "fc_port": self.gui.fc_port_var,
            "fc_baud": self.gui.fc_baud_var,
            "dip": self.gui.dip_var,
        }

    def scale_blocked(self, action: str) -> bool:
        if not self.bench_active:
            return False
        from tkinter import messagebox
        messagebox.showinfo("称重串口正在使用", f"H743台架会话正在读取称重数据，请先断开台架会话再执行：{action}")
        return True

    def _assert_idle(self) -> None:
        worker = self.gui.worker
        if worker is not None and worker.is_alive():
            raise RuntimeError("请先停止独立称重读取；连接H743后会继续显示实时重量")

    def open_bench(self, frame, snapshot_hz: float):
        if self.bench_active:
            raise RuntimeError("实测辨识已经占用串口")
        self._assert_idle()
        port, baud, addr, channel, _interval, timeout = self.gui._settings()
        fc_port = self.gui.fc_port_var.get().strip()
        if not fc_port:
            raise ValueError("请在原窗口填写飞控串口")
        if port.casefold() == fc_port.casefold():
            raise ValueError("称重与飞控必须使用两个不同串口")
        calibration = LegacyCalibration.from_points(tuple(self.gui.calibration_points))
        self.bench_active = True
        try:
            frame.scale_transport.connect(port, int(baud), timeout_s=float(timeout))
            frame.connection.connect(
                fc_port, int(self.gui.fc_baud_var.get(), 0),
                snapshot_hz=snapshot_hz)
            load_cell = LegacyLoadCell(
                frame.scale_transport, addr, calibration,
                register=(channel - 1) * 2)
        except Exception:
            self.release_bench()
            frame.scale_transport.close()
            frame.connection.disconnect()
            raise
        metadata = {
            "scale_settings_source": "pressure_rs485_gui",
            "scale_port": port,
            "scale_baud": int(baud),
            "scale_address": addr,
            "scale_channel": channel,
            "scale_register": (channel - 1) * 2,
            "scale_timeout_s": float(timeout),
            "scale_calibration": calibration.to_dict(),
        }
        return load_cell, metadata

    def release_bench(self) -> None:
        self.bench_active = False
