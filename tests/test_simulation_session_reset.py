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
        # 必须等通道表收全，不能只等帧数。
        #
        # 掩码帧的通道名来自 TELEM CH 分页，而分页要几个来回；在那之前帧照收、
        # dashboard_frames_seen 照涨，但 dashboard_index_by_name 还是空的，
        # _dashboard_latest() 对**任何**通道都返回 None（包括 sim_x）。
        # 只等帧数是一个竞态：机器快一点就在通道表收全前先撞上断言。
        while time.monotonic()<deadline and not (
            app.dashboard_schema.complete and app.dashboard_frames_seen>=3
        ):
            app.update()
            time.sleep(.015)
        assert app.dashboard_schema.complete
        assert app.dashboard_frames_seen>=3
        assert app._dashboard_latest('sim_pos_x_kp') is not None
        bar.stop()
        assert app.dashboard_frames_seen==0
        assert app._dashboard_latest('sim_pos_x_kp') is None
    finally:
        app.simulation_bar.stop()
        app.destroy()
