"""Actual panel startup, loopback child lifecycle, and no hardware switching."""
import time
from types import SimpleNamespace

import pytest

from tools.drone_tcp_panel import DronePanel
from tools.sim_xz.control_catalog import GAIN_CHANNELS


@pytest.fixture
def panel():
    app = DronePanel()
    yield app
    app.simulation_bar.stop()
    app.destroy()


def pump(app, condition, timeout=15):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        app.update()
        if condition(): return
        time.sleep(.015)
    raise AssertionError(app.simulation_bar.status.get())


def test_one_click_starts_child_connects_streams_and_stop_reaps(panel):
    bar = panel.simulation_bar
    bar.headless = True  # Same executable/device path, without a visible test window.
    old = panel.transport_var.get(), panel.host_var.get(), panel.port_var.get()
    old_workspaces = list(panel.dashboard_layout.workspaces)
    bar.start_button.invoke()
    pump(panel, lambda: bar.connected_once and panel.dashboard_schema.complete
         and panel.dashboard_frames_seen > 2)
    child = bar.process
    assert child is not None and child.poll() is None
    assert panel.host_var.get() == '127.0.0.1'
    assert panel.port_var.get() > 0
    assert str(panel.link_status_label.cget("textvariable")) == str(bar.link_status)
    assert "仿真数据" in bar.link_status.get()
    assert panel.dashboard_layout.active_workspace().name == '二维仿真'
    bar.start()  # Repeated click must not launch a second process.
    assert bar.process is child
    # Check motion with the real default gains before deliberately writing test values.
    pump(panel, lambda: abs(panel._dashboard_latest('sim_vx') or 0)>1e-6)
    for offset, channel in enumerate(GAIN_CHANNELS):
        value=.05 + offset*.001
        assert panel._dashboard_send_param(channel[0], value)
        pump(panel, lambda channel=channel, value=value:
             abs((panel._dashboard_latest(channel[0]) or 0)-value)<1e-4)
    assert len(bar.workspaces)==2
    height_workspace=bar.workspaces[1]
    assert {'sim_pos_z_kp','sim_vel_z_ki','sim_vel_z_kd'} <= set(height_workspace.bound_channels())

    bar.stop_button.invoke()
    pump(panel, lambda: child.poll() is not None)
    assert not panel.tcp_transport.is_connected
    assert (panel.transport_var.get(),panel.host_var.get(),panel.port_var.get()) == old
    assert panel.dashboard_layout.workspaces == old_workspaces
    assert not bar.pending
    assert str(panel.link_status_label.cget("textvariable")) == str(panel.link_var)


def test_active_device_is_not_replaced(panel, monkeypatch):
    bar = panel.simulation_bar
    called=[]
    monkeypatch.setattr(panel, '_start', lambda: called.append('start'))
    original=panel.serial_transport
    panel.serial_transport=SimpleNamespace(is_connected=True)
    try:
        bar.start()
        assert not called
        assert '先断开' in bar.status.get()
    finally:
        panel.serial_transport=original


def test_process_start_failure_releases_listener_and_allows_retry(panel):
    def fail(*args, **kwargs): raise OSError('test launch error')
    bar=panel.simulation_bar
    bar.process_factory=fail
    bar.start()
    pump(panel, lambda: not bar.pending)
    assert '无法启动' in bar.status.get()
    assert panel.tcp_transport.sock is None
    assert not bar.start_button.instate(['disabled'])


def test_destroy_reaps_owned_child(panel):
    bar=panel.simulation_bar
    bar.headless=True
    bar.start()
    pump(panel, lambda: bar.connected_once)
    child=bar.process
    # Exercise root-destruction binding without destroying the test fixture twice.
    bar._destroyed(SimpleNamespace(widget=panel))
    child.wait(timeout=5)
    assert child.poll() is not None
    assert not bar.pending


def test_titlebar_height_choice_starts_vertical_experiment(panel):
    bar=panel.simulation_bar
    bar.headless=True
    bar.experiment_var.set('高度阶跃 · Z')
    bar.start_button.invoke()
    pump(panel, lambda: bar.connected_once and panel.dashboard_schema.complete
         and (panel._dashboard_latest('sim_z') or 0)>1.01)
    assert panel.dashboard_layout.active_workspace().name=='高度 P—速度 PID'
    assert panel._dashboard_send_param('sim_vel_z_ki',.12)
    pump(panel, lambda: abs((panel._dashboard_latest('sim_vel_z_ki') or 0)-.12)<1e-4)
