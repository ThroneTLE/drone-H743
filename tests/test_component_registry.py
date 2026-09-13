"""Real C registry frames through transport and a hardware-agnostic overview."""
from pathlib import Path
import shutil
import subprocess
import struct
import time
import queue
import random
import zlib

import pytest
from tools import drone_tcp_panel as panel
from tools.panel_lib import component_registry as registry, rx_dispatch
from tools.panel_lib.connection_state import receive_context
from tools.panel_lib.proto import PROTO_MSG_COMPONENTS, PROTO_DIR_FROM_FC
from tools.panel_lib.transport import TransportBase, build_proto_frame

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def firmware(tmp_path_factory):
    d=tmp_path_factory.mktemp("components-c")
    source=r'''
#include "app_components.h"
#include "app_proto.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
static FILE *file;static int mode,depth,samples;static uint32_t now;
uint32_t BSP_Critical_Enter(void){return (uint32_t)depth++;}
void BSP_Critical_Exit(uint32_t old){depth=(int)old;}
uint8_t APP_Components_ExportBusy(void){return mode==2;}
uint32_t APP_Components_NowMs(void){return now;}
uint8_t APP_Components_Send(const uint8_t *p,uint16_t n){
    assert(!depth);uint8_t frame[256];uint16_t len;
    assert(APP_Proto_BuildFrame('>',APP_PROTO_MSG_COMPONENTS,p,n,frame,sizeof(frame),&len));
    fwrite(frame,1,len,file);return 1;
}
static void sample(DRV_ComponentRecord *r){
    assert(depth);samples++;
    r->name="Arbitrary sensor";r->model="MODEL-X9";r->bus="custom bus";
    r->stage="sampled";r->note="registered by firmware";
    r->state=DRV_COMPONENT_READY;r->age_ms=12;r->samples=17;r->field_count=2;
    r->fields[0]=(DRV_ComponentField){.label="pressure",.unit="Pa",.type=DRV_COMPONENT_F32,.valid=1,.value.f=100000.125f};
    r->fields[1]=(DRV_ComponentField){.label="unavailable",.unit="V",.type=DRV_COMPONENT_F32,.valid=0,.value.f=0};
}
static void other(DRV_ComponentRecord *r){sample(r);r->name="New device";r->model="UNLISTED-77";r->state=DRV_COMPONENT_DISABLED;}
void APP_Components_RegisterBoard(void){
    if(mode==1)return;
    assert(APP_Components_Register(41,sample));assert(APP_Components_Register(902,other));
    if(mode==3)assert(!APP_Components_Register(41,other));
    assert(APP_Components_Register(41,sample));
}
int main(int argc,char **argv){
    assert(argc==3);mode=atoi(argv[1]);file=fopen("frames.bin","wb");assert(file);
    char *tokens[]={"REGISTRY?",argv[2]};assert(APP_Components_Command(tokens,2));
    if(mode==2||mode==3)assert(samples==0);
    fclose(file);return 0;
}
'''
    (d/'test.c').write_text(source)
    cmd=[shutil.which('gcc'),'-std=c11','-O2','-Wall','-Wextra','-Werror']
    for inc in ('App/Inc','Driver/Inc','BSP/Inc'):cmd += ['-I',str(ROOT/inc)]
    cmd += [str(d/'test.c'),str(ROOT/'App/Src/app_components.c'),str(ROOT/'Driver/Src/drv_component_proto.c'),str(ROOT/'App/Src/app_proto.c'),'-o',str(d/'test.exe')]
    r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
    def run(mode=0,nonce=42):
        r=subprocess.run([str(d/'test.exe'),str(mode),str(nonce)],cwd=d,capture_output=True)
        assert r.returncode==0,r.stderr
        return (d/'frames.bin').read_bytes()
    return run


def payloads(wire):
    result=[]
    while wire:
        size=int.from_bytes(wire[6:8],'little')
        result.append(wire[8:8+size]);wire=wire[9+size:]
    return result


def decode(data,nonce=42):
    txn=registry.RegistryTransaction(nonce);result=None
    for payload in payloads(data):result=txn.feed(payload)
    return result


def test_actual_c_snapshot_and_empty_busy_duplicate(firmware):
    rows=decode(firmware())
    assert [r.id for r in rows]==[41,902]
    assert rows[0].model=='MODEL-X9' and rows[1].model=='UNLISTED-77'
    assert rows[0].fields[0].value==100000.125
    assert rows[0].fields[1].value is None
    assert decode(firmware(1))==()
    for mode in (2,3):
        with pytest.raises(ValueError,match='飞控注册响应错误'):decode(firmware(mode))


def test_independent_golden_bytes(firmware):
    # Independent layout from doc/component-registry.md, including wire CRC8.
    def string(value):
        data=value.encode();return bytes([len(data)])+data
    records=[]
    for index,identifier,name,model,state in ((0,41,'Arbitrary sensor','MODEL-X9',1),
                                               (1,902,'New device','UNLISTED-77',3)):
        body=struct.pack('<HBBiII',identifier,state,2,0,12,17)
        body+=b''.join(string(s) for s in (name,model,'custom bus','sampled','registered by firmware'))
        body+=string('pressure')+string('Pa')+struct.pack('<BBf',3,1,100000.125)
        body+=string('unavailable')+string('V')+struct.pack('<BBf',3,0,0)
        records.append(struct.pack('<BBHIH',1,2,2,42,index)+body)
    crc=zlib.crc32(b''.join(records))
    parts=[struct.pack('<BBHIH',1,1,2,42,0),*records,struct.pack('<BBHIHI',1,3,2,42,2,crc)]
    expected=bytearray()
    for payload in parts:
        data=struct.pack('<BHH',0,0x2231,len(payload))+payload
        crc8=0
        for byte in data:
            crc8^=byte
            for _ in range(8):crc8=((crc8<<1)^0xd5)&255 if crc8&128 else (crc8<<1)&255
        expected.extend(b'$X>'+data+bytes([crc8]))
    assert firmware()==bytes(expected)


def test_nonce_range_and_bounded_fuzz(firmware):
    assert decode(firmware(nonce=4294967295),4294967295)
    for nonce in ('4294967296','-1','x',''):
        with pytest.raises(ValueError,match='code=4'):decode(firmware(nonce=nonce),0)
    original=payloads(firmware())
    rng=random.Random(2231)
    # Mutate a record only: unchanged end CRC must reject every altered snapshot.
    for _ in range(2000):
        damaged=bytearray(original[1]);pos=rng.randrange(len(damaged))
        mode=rng.randrange(3)
        if mode==0:damaged[pos]^=1<<rng.randrange(8)
        elif mode==1:damaged[pos:pos]=bytes([rng.randrange(256)])
        else:del damaged[pos:pos+1]
        txn=registry.RegistryTransaction(42)
        with pytest.raises((ValueError,UnicodeError)):
            for part in (original[0],bytes(damaged),*original[2:]):
                assert txn.feed(part) is None


def test_incomplete_reordered_duplicate_and_crc(firmware):
    parts=payloads(firmware())
    bad_batches=[parts[1:], [parts[0],parts[2],parts[1],parts[3]],
                 [parts[0],parts[1],parts[1],parts[3]],parts[:-1]+[parts[-1][:-1]],
                 parts[:-1]+[parts[-1][:-1]+bytes([parts[-1][-1]^1])]]
    for batch in bad_batches:
        txn=registry.RegistryTransaction(42)
        with pytest.raises(ValueError):
            for data in batch:txn.feed(data)
    txn=registry.RegistryTransaction(42)
    for data in parts[:-1]:assert txn.feed(data) is None
    for data in parts:assert registry.RegistryTransaction(99).feed(data) is None


def test_record_truncation_and_bad_float(firmware):
    record=payloads(firmware())[1][registry.HEADER.size:]
    for size in range(len(record)):
        with pytest.raises((ValueError,UnicodeError)):registry.decode_record(record[:size])
    with pytest.raises(ValueError):registry.decode_record(record+b'\0')
    value=record.index(struct.pack('<f',100000.125))
    for bad in (float('nan'),float('inf')):
        with pytest.raises(ValueError,match='非有限'):registry.decode_record(record[:value]+struct.pack('<f',bad)+record[value+4:])


class Wire(TransportBase):
    is_connected=True
    connection_generation=1
    _stamp_received=True
    def __init__(self,rx):self.rx_queue=rx;self.commands=[]
    def start(self,*args):pass
    def stop(self):self.is_connected=False
    def send_frame(self,*args):raise AssertionError('registry uplink must stay ASCII')
    def send_line(self,line):self.commands.append(line);return True


@pytest.fixture(scope="module")
def app():
    p=panel.DronePanel();yield p;p.destroy()


def prepare(app):
    while not app.rx_queue.empty():app.rx_queue.get_nowait()
    app.transport=Wire(app.rx_queue);page=app.overview_page;page._connection();page.nonce=41
    assert page.request();assert app.transport.commands==['REGISTRY? 42']
    return page


def feed(app,wire,*,age=0,generation=None):
    context=receive_context(app.transport,received_at=time.monotonic()-age,generation=generation)
    buffer=bytearray()
    for i in range(0,len(wire),11):
        buffer.extend(wire[i:i+11]);app.transport._consume_buffer(buffer,context=context)
    rx_dispatch.drain_rx(app,100,5,50)


def test_real_transport_populates_only_registered_unknown_models(app,firmware):
    page=prepare(app)
    app._handle_board_line('HW FLASH ok=0 id=000000')
    assert not page.tree.get_children()
    feed(app,firmware())
    assert page.tree.get_children()==('41','902')
    assert 'MODEL-X9' in page.tree.item('41','values')
    assert 'UNLISTED-77' in page.tree.item('902','values')
    assert len(app.module_state)==0
    page.tree.selection_set('41');page.refresh_detail()
    assert 'pressure=100000' in page.detail.get() and 'unavailable=—' in page.detail.get()


def test_session_and_stale_receipt_do_not_show_green(app,firmware):
    page=prepare(app);feed(app,firmware(),age=4)
    assert page.tree.item('41','values')[1]=='回包已过期'
    app.transport.connection_generation+=1;page._connection()
    assert not page.tree.get_children()
    page.nonce=41;assert page.request()
    feed(app,firmware(),generation=1)
    assert not page.tree.get_children()
    feed(app,firmware());assert page.tree.get_children()
    app.transport.is_connected=False;page._connection();assert not page.tree.get_children()


def test_old_firmware_explicitly_unsupported(app):
    page=prepare(app);feed(app,b'ERR unknown cmd REGISTRY?\r\n')
    assert page.unsupported and not page.tree.get_children()
    assert '未提供' in page.status.get()


def test_request_refusal_timeout_and_recovery(app,firmware,monkeypatch):
    page=prepare(app)
    assert not page.request() and len(app.transport.commands)==1
    page.started=time.monotonic()-3;page._tick()
    assert page.transaction is None and '未收到完整' in page.status.get()
    page.last_attempt=None
    with monkeypatch.context() as patch:
        patch.setattr(app.transport,'send_line',lambda _:False)
        assert not page.request() and page.transaction is None
        assert '未发送' in page.status.get()
    page.last_attempt=None;page.nonce=41;assert page.request()
    feed(app,firmware());assert page.tree.get_children()==('41','902')
    app.transport=Wire(app.rx_queue);page._connection()
    with monkeypatch.context() as patch:
        patch.setattr(app,'_validation_command_allowed',lambda _:False)
        assert not page.request() and not app.transport.commands
        assert '拒绝' in page.status.get()


def test_tcp_ascii_and_binary_sink_separation(firmware):
    from tools.panel_lib.transport import TcpTransport
    tcp=TcpTransport(queue.Queue())
    assert not tcp.send_ascii_line('REGISTRY? 42')
    tcp.client=object()  # No socket or network thread is opened.
    assert tcp.send_ascii_line('REGISTRY? 42')
    assert tcp._send_queue.get_nowait()[1]==b'REGISTRY? 42\r\n'
    wire=Wire(queue.Queue());seen=[];wire.binary_sink=lambda *args:seen.append(args)
    wire._consume_buffer(bytearray(firmware()))
    assert not seen and wire.rx_queue.qsize()==4


def test_schema_ids_and_no_legacy_overview_inventory():
    assert '#define APP_PROTO_MSG_COMPONENTS        0x2231U' in (ROOT/'App/Inc/app_proto.h').read_text(encoding='utf-8')
    assert PROTO_MSG_COMPONENTS==0x2231
    assert panel.MODULES==()
