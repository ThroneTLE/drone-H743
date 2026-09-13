"""SPP identity, real Tk selection and asynchronous serial cancellation contracts."""
import queue
import threading
import time
from types import SimpleNamespace
import pytest
from tools.panel_lib import bluetooth_channel as ui, bluetooth_devices as devices, transport
from tools.panel_qa import OfflinePanel
from tools.panel_qa.geometry import SCALES, WINDOW_SIZES

ADDRESS='AABBCCDDEEFF'


def instance(address):
    return f'BTHENUM\\{{{devices.SPP_UUID}}}_LOCALMFG&0002\\7&650EFA4&0&{address}_C00000000'


def candidates(port='COM35',address=ADDRESS,available=True):
    return (devices.BluetoothDevice(address,'MicoAir743v2-test',port,instance(address),available),)


def test_join_outgoing_port_and_name_excludes_incoming_and_usb():
    records=[{'name':'MicoAir743v2-test','hwid':f'BTHENUM\\DEV_{ADDRESS}\\7&xx'},
             {'name':'蓝牙链接上的标准串行 (COM35)','hwid':instance(ADDRESS)},
             {'name':'蓝牙链接上的标准串行 (COM4)','hwid':instance('000000000000')}]
    ports=[SimpleNamespace(device='COM35',description='标准串行',hwid=instance(ADDRESS)),
           SimpleNamespace(device='COM22',description='STM32',hwid='USB VID:PID=0483:5740')]
    result=devices.collect_devices(ports,records)
    assert len(result)==1 and result[0].address==ADDRESS and result[0].name=='MicoAir743v2-test'
    assert result[0].available and result[0].port=='COM35'
    assert devices.choose_device(result,ADDRESS)[0] is not None
    assert devices.choose_device(candidates('COM8'),ADDRESS)[0].port=='COM8'
    assert devices.choose_device(candidates('COM35','112233445566'),ADDRESS)[0] is None
    assert devices.choose_device(candidates()+candidates('COM9','112233445566'))[0] is None
    assert not devices.collect_devices([],records)[0].available
    assert devices.bluetooth_address(instance('000000000000'))==''


def test_bluetooth_identity_cannot_pass_usb_safety_override():
    identity=candidates()[0].identity()
    assert transport.serial_port_fingerprint(identity)=='BTH/'+ADDRESS
    assert transport.serial_device_identity_policy(identity)[0]=='rejected'


def pump_scan(p):
    deadline=time.monotonic()+1
    while p._bt_scanning and time.monotonic()<deadline:
        p._bt_tick();time.sleep(.005)
    assert not p._bt_scanning


@pytest.fixture
def app(monkeypatch):
    monkeypatch.setattr(ui,'discover_devices',lambda:candidates())
    qa=OfflinePanel.launch(connected=False)
    yield qa
    qa.destroy()


def test_real_panel_auto_selects_and_keeps_usb_record(app,monkeypatch):
    p=app.panel
    p._panel_state={'serial_port':'COM22','serial_fingerprint':'USB/TEST','serial_baud':57600}
    p.serial_baud_var.set(57600)
    assert '蓝牙' in p._transport_combo['values']
    p.transport_var.set('蓝牙');pump_scan(p)
    assert ADDRESS[-6:] in p.bluetooth_device_var.get()
    assert p._current_transport() is p.serial_transport
    assert not p.serial_transport.is_connected
    calls=[]
    def start(port,baud):
        calls.append((port,baud));p.serial_transport.start(port,baud);p.serial_transport.active_port=port
    monkeypatch.setattr(p.serial_transport,'start_async',start,raising=False)
    p._start();pump_scan(p);p._bt_tick()
    assert calls==[('COM35',115200)]
    from tools.panel_lib.state import PanelStateMixin
    PanelStateMixin._save_panel_state(p)
    assert p._panel_state['transport']=='bluetooth'
    assert p._panel_state['bluetooth_address']==ADDRESS
    assert p._panel_state['serial_port']=='COM22' and p._panel_state['serial_baud']==57600
    p._stop();p.transport_var.set('serial')
    assert p.serial_baud_var.get()==57600


def test_stop_and_switch_drop_delayed_scan(app,monkeypatch):
    p=app.panel;entered=threading.Event();release=threading.Event()
    def scan():entered.set();release.wait(1);return candidates()
    monkeypatch.setattr(ui,'discover_devices',scan)
    p.transport_var.set('蓝牙');assert entered.wait(1)
    p._start();p._stop();p.transport_var.set('tcp');release.set();time.sleep(.02);p._bt_tick()
    assert not p.serial_transport.is_connected and not p._bt_connect_pending
    assert p._current_transport() is p.tcp_transport


def test_unavailable_or_ambiguous_device_never_opens(app,monkeypatch):
    p=app.panel
    for found in (candidates(available=False),candidates()+candidates('COM8','112233445566'),()):
        p._stop();p._bt_devices={};p.bluetooth_device_var.set('');p._panel_state={}
        monkeypatch.setattr(ui,'discover_devices',lambda found=found:found)
        p.transport_var.set('tcp');p.transport_var.set('蓝牙');p._start();pump_scan(p)
        assert not p.serial_transport.is_connected


def test_async_driver_open_can_be_cancelled(monkeypatch):
    entered=threading.Event();release=threading.Event();closed=threading.Event()
    class Port:
        is_open=True
        def close(self):closed.set()
        def cancel_read(self):pass
        def cancel_write(self):pass
    def slow_open(*a,**kw):entered.set();release.wait(1);return Port()
    monkeypatch.setattr(transport.serial,'Serial',slow_open)
    session=transport.SerialTransport(queue.Queue())
    started=time.monotonic();session.start_async('COM_FAKE',115200)
    assert time.monotonic()-started<.1 and entered.wait(1)
    session.stop();release.set();assert closed.wait(1)
    assert not session.is_connected and session.active_port is None


@pytest.mark.parametrize('scale',SCALES)
def test_bluetooth_controls_layout(scale,monkeypatch):
    monkeypatch.setattr(ui,'discover_devices',lambda:candidates())
    qa=OfflinePanel.launch(scale=scale,connected=False)
    try:
        p=qa.panel;p.transport_var.set('蓝牙');pump_scan(p)
        for width,height in WINDOW_SIZES:
            qa.resize(width,height)
            assert p._bluetooth_combo.winfo_ismapped()
            assert p._bluetooth_refresh.winfo_ismapped()
            assert p._action_frame.winfo_rootx()+p._action_frame.winfo_width() <= p.winfo_rootx()+p.winfo_width()
        assert not qa.callback_errors
    finally:qa.destroy()
