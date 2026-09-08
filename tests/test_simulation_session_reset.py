"""Stopping a simulator must not leave its samples in the next session."""
import time
from tools.drone_tcp_panel import DronePanel


def test_stopping_simulator_clears_its_telemetry_session():
    app=DronePanel()
    try:
        bar=app.simulation_bar
        bar.headless=True
        bar.start()
        deadline=time.monotonic()+15
        while time.monotonic()<deadline and app.dashboard_frames_seen<3:
            app.update()
            time.sleep(.015)
        assert app.dashboard_frames_seen>=3
        assert app._dashboard_latest('sim_pos_x_kp') is not None
        bar.stop()
        assert app.dashboard_frames_seen==0
        assert app._dashboard_latest('sim_pos_x_kp') is None
    finally:
        app.simulation_bar.stop()
        app.destroy()
