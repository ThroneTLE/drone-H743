"""Real firmware bytes through transport/Rx dispatch to the actual battery page."""
import queue
import struct
import time
import pytest
from tools import drone_tcp_panel as panel
from tools.panel_lib import rx_dispatch
from tools.panel_lib.connection_state import receive_context
from tools.panel_lib.transport import TransportBase,build_proto_frame
from tools.panel_lib.proto import PROTO_MSG_BATTERY,PROTO_DIR_FROM_FC
from tools.panel_qa import OfflinePanel
from tools.panel_qa.geometry import SCALES,WINDOW_SIZES
from test_battery_runtime import firmware_battery


class Wire(TransportBase):
    is_connected=True;connection_generation=1;_stamp_received=True
    def __init__(self,rx):super().__init__(rx);self.commands=[]
    def start(self,*a):self.is_connected=True
    def stop(self):self.is_connected=False
    def send_line(self,line):self.commands.append(line);return True
    def send_frame(self,*a):raise AssertionError('battery commands use existing ASCII uplink')


@pytest.fixture(scope='module')
def app():
    p=panel.DronePanel();p.battery_page.auto.set(False)
    yield p;p.destroy()


def prepare(p):
    p.transport=Wire(p.rx_queue);page=p.battery_page;page._connection();page.nonce=41
    page.dirty=False;assert page.request();return page


def feed(p,data,*,age=0,generation=None):
    context=receive_context(p.transport,received_at=time.monotonic()-age,generation=generation)
    buffer=bytearray()
    for i in range(0,len(data),7):
        buffer.extend(data[i:i+7]);p.transport._consume_buffer(buffer,context=context)
    rx_dispatch.drain_rx(p,100,5,50)


def mutate(wire,nonce,*,flags=None,cells=None,low=None,recover=None,arm=None):
    payload=bytearray(wire[8:-1]);struct.pack_into('<I',payload,4,nonce)
    if flags is not None:payload[1]=flags
    if cells is not None:payload[2]=cells
    if low is not None:struct.pack_into('<I',payload,28,low)
    if recover is not None:struct.pack_into('<I',payload,32,recover)
    if arm is not None:struct.pack_into('<I',payload,56,arm)
    return build_proto_frame(PROTO_DIR_FROM_FC,PROTO_MSG_BATTERY,payload)


def test_actual_c_voltage_and_current_are_separate_from_signal_voltage(app,firmware_battery):
    page=prepare(app);feed(app,firmware_battery)
    assert page.value.get()=='11.999 V'  # Exact nominal conversion of the real C fixture's ADC code.
    assert page.fields['cell'].get()=='4.000 V'
    assert page.fields['raw'].get()=='11283'
    assert page.fields['current'].get()=='47.393 A'
    assert page._can_configure()


def test_config_needs_explicit_firmware_ack_and_preserves_new_draft(app,firmware_battery):
    page=prepare(app);feed(app,firmware_battery)
    page.low.set(3400);assert page.apply_config();nonce=page.nonce
    assert app.transport.commands[-1]==f'BATTERY SET {nonce} 3 3400 3600'
    assert page.snapshot.low_mv==3500
    page.low.set(3300)
    feed(app,mutate(firmware_battery,nonce,flags=0x15|32,low=3400))
    assert page.snapshot.low_mv==3400 and page.low.get()==3300 and page.dirty
    assert '已应用' in page.notice.get()
    assert page.apply_config();nonce=page.nonce
    feed(app,mutate(firmware_battery,nonce,flags=0x15|64,low=3400))
    assert '未确认' in page.notice.get() and page.snapshot.low_mv==3400


def test_armed_stale_foreign_and_disconnect_refuse_config(app,firmware_battery):
    page=prepare(app);feed(app,mutate(firmware_battery,42,arm=2))
    assert not page.apply_config()
    page=prepare(app);feed(app,firmware_battery,age=4)
    assert page.value.get()=='— V' and not page.apply_config()
    page=prepare(app);feed(app,firmware_battery,generation=0)
    assert page.snapshot is None
    feed(app,firmware_battery);assert page.snapshot is not None
    app.transport.connection_generation+=1
    assert not page.apply_config() and page.snapshot is None
    app.transport.is_connected=False;page._tick();assert page.value.get()=='— V'


def test_missing_firmware_and_bad_packet_do_not_leave_good_fields(app,firmware_battery):
    page=prepare(app);feed(app,b'ERR unknown cmd BATTERY?\r\n')
    assert page.unsupported and '未提供' in page.status.get()
    page=prepare(app);feed(app,firmware_battery)
    assert page.request()
    bad=build_proto_frame(PROTO_DIR_FROM_FC,PROTO_MSG_BATTERY,b'bad')
    feed(app,bad);assert page.value.get()=='— V' and page.fields['valid'].get()=='—'


@pytest.mark.parametrize('scale',SCALES)
def test_battery_page_matrix(scale):
    qa=OfflinePanel.launch(scale=scale,connected=False)
    try:
        leaf=next(p for p in qa.leaf_pages() if p.label=='传感器 / 电池电压')
        qa.select(leaf)
        for width,height in WINDOW_SIZES:
            qa.resize(width,height)
            p=qa.panel.battery_page
            assert p.value_label.winfo_ismapped() and p.apply_button.winfo_exists()
            assert p.apply_button.instate(['disabled'])
        assert not qa.callback_errors
    finally:qa.destroy()
