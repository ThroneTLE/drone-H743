from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_simulation_ui_has_animation_slow_motion_and_ab_controls() -> None:
    source = (ROOT / "tools" / "sim_xz" / "app.py").read_text(encoding="utf-8")
    assert "class SimulationApp" in source
    assert "self.after(33, self._render)" in source
    assert "slow motion" in source
    assert "Save A parameter snapshot" in source
    assert "Run B vs saved A" in source
    assert "tuned_params = self.device.engine.bridge.parameter_snapshot()" in source
    assert "A/B: pitch, pitch_rate, vx, x, z" in source
    assert "thrust_scale" in source
    assert "target_px" in source
    assert "SimulationApp().mainloop()" in source


def test_worker_device_does_not_call_tk() -> None:
    source = (ROOT / "tools" / "sim_xz" / "device.py").read_text(encoding="utf-8")
    assert "tkinter" not in source
    assert "self.after" not in source
