from __future__ import annotations
import json, queue, threading, time
from types import SimpleNamespace
import math
import inspect
from dataclasses import replace
from tools.thrust_bench import fc_protocol
from tools.thrust_bench.acquisition import AckTransaction, AcquisitionEngine
from tools.thrust_bench.connection import (FlightControllerConnection, LinkEvent,
                                           estimated_wire_bytes_per_second)
from tools.panel_lib.transport import PROTO_DIR_FROM_FC, build_proto_frame
from tools.thrust_bench.plans import common_sweep, dynamic_steps, grid, single_prop, with_voltage_layer
from tools.thrust_bench.records import FcSnapshot
from tools.thrust_bench.session import SessionStore, read_samples

class Store:
    def __init__(self): self.events=[]; self.fc=[]; self.scale=[]; self.samples=[]
    def event(self,kind,**data): self.events.append((kind,data))
    def raw_fc(self,**data): self.fc.append(data)
    def raw_scale(self,**data): self.scale.append(data)
    def sample(self,sample): self.samples.append(sample)
    def update_metadata(self,**data): pass
class Connection:
    def __init__(self): self.events=queue.Queue(); self.generation=2; self.is_connected=True; self.sent=[]
    def send_command(self,text): self.sent.append(text); return True
    def disconnect(self): self.is_connected=False; self.generation+=1
    def validate_snapshot_rate(self,hz):
        return {"snapshot_hz":float(hz),"rate_policy":"offline_fake_explicit"}


def test_rejection_survives_idle_and_propcal_uses_existing_event_channel():
    connection=Connection(); engine=AcquisitionEngine(connection,None,Store())
    # Text shapes from app_cmd_thrust_bench.c / app_cmd_propcal.c.
    connection.events.put(LinkEvent("fc_text",2,10.0,"TBENCH state=rejected reason=mapping_unconfirmed"))
    connection.events.put(LinkEvent("fc_text",2,10.1,"TBENCH state=idle reason=host_stop"))
    propcal=LinkEvent("fc_text",2,10.2,"PROPCAL state=unconfirmed calibrated=0 upper_ch=0 lower_ch=0")
    connection.events.put(propcal); engine._drain_events()
    assert engine.last_rejection_reason=="mapping_unconfirmed"
    events=[]
    while not engine.ui_events.empty(): events.append(engine.ui_events.get_nowait())
    assert any(e.kind=="rejected" and e.data["reason"]=="mapping_unconfirmed" for e in events)
    assert any(e.kind=="stopped_confirmed" for e in events)
    assert engine.propcal_events.get_nowait() is propcal


def test_persistent_health_gap_records_source_age_and_link_backlog():
    store=Store();conn=Connection()
    now=[10.0]
    engine=AcquisitionEngine(conn,None,store,clock=lambda:now[0])
    engine._latest_fc=(snapshot(),9.0,conn.generation,1000)
    engine._latest_scale=(100.0,10.0)
    engine._armed=True;engine._stream_established=True
    assert engine._monitor_active_health()
    assert engine.armed and not engine.cancel.is_set()
    now[0]+=engine.TELEMETRY_GAP_GRACE_S+.01
    assert not engine._monitor_active_health()
    kind,data=next((kind,data) for kind,data in store.events if kind=="active_stream_lost")
    assert kind=="active_stream_lost"
    assert data["fc_receive_age_ms"]==3010.0
    assert data["fc_erpm_source_age_ms"]==list(snapshot().erpm_age_ms)
    assert data["pending_snapshots"]==0 and data["link_queue_depth"]==0
    assert engine.cancel.is_set() and not engine.armed


def test_health_check_accepts_a_fresh_reply_processed_during_short_queue_drain():
    from dataclasses import replace
    conn=Connection();store=Store()
    engine=AcquisitionEngine(conn,None,store,snapshot_hz=30)
    valid=replace(snapshot(),erpm=(14000,14000),erpm_age_ms=(0,0),
                  voltage_v=12.0,voltage_age_ms=0)
    engine._latest_fc=(valid,time.monotonic()-.30,conn.generation,1000)
    engine._latest_scale=(100.0,time.monotonic())
    engine._armed=True;engine._stream_established=True
    def deliver():
        time.sleep(.003)
        with engine._observation_lock:
            engine._latest_fc=(valid,time.monotonic(),conn.generation,1100)
        with engine._snapshot_condition:
            engine._snapshot_condition.notify_all()
    worker=threading.Thread(target=deliver);worker.start()
    try:
        assert engine._monitor_active_health()
    finally:
        worker.join(.2)
    assert engine.armed and not engine.cancel.is_set()
    assert not any(kind=="active_stream_lost" for kind,_ in store.events)


def test_266ms_snapshot_gap_recovers_without_stopping_motor_window():
    clock=[10.0];conn=Connection();store=Store()
    engine=AcquisitionEngine(conn,None,store,clock=lambda:clock[0])
    live=replace(snapshot(),erpm=(14000,14000),erpm_age_ms=(0,0),
                 voltage_v=12.0,voltage_age_ms=0)
    engine._latest_fc=(live,clock[0]-.266,conn.generation,1000)
    engine._latest_scale=(100.0,clock[0])
    engine._armed=True;engine._stream_established=True
    assert engine._monitor_active_health()
    clock[0]+=.05
    engine._latest_fc=(live,clock[0],conn.generation,1300)
    assert engine._monitor_active_health()
    assert engine.armed and not engine.cancel.is_set()
    assert conn.sent==[]
    assert [kind for kind,_ in store.events]==[
        "telemetry_gap_started","telemetry_gap_recovered"]


def test_engine_contract_has_no_pole_pair_or_current_calibration_inputs():
    parameters=inspect.signature(AcquisitionEngine).parameters
    assert "upper_pole_pairs" not in parameters
    assert "lower_pole_pairs" not in parameters
    assert "current_calibration" not in parameters
def snapshot(nonce=1):
    return FcSnapshot(nonce,1000,1,1,2,(1520,0),(14000,None),(10,None),12.0,3.0,5,6,False,True,20,False,(None,None),(None,None),(False,False),0)

def test_all_plan_modes_use_frozen_vocabulary():
    plans=[single_prop("upper",minimum=0,maximum=10,step=10),single_prop("lower",minimum=0,maximum=10,step=10),common_sweep(minimum=0,maximum=10,step=10),grid(upper_values=(0,5),lower_values=(0,5)),dynamic_steps(baseline=(5,5),axis="upper",delta=2)]
    assert {p.mode for p in plans} == {"upper","lower","dual"}
    assert {x.phase for p in plans for x in p.points} <= {"settle","steady","dynamic"}
    assert {x.direction for p in plans for x in p.points} <= {"up","down","steady"}

def test_join_uses_mapping_and_direct_measured_erpm():
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store(),clock=lambda:10.0)
    mapped=FcSnapshot(1,1000,1,2,1,(0,1520),(28000,14000),(20,10),12.0,3.0,5,6,False,True,20,False,(None,None),(None,None),(False,False),0)
    engine._latest_fc=(mapped,9.99,2,1000); engine._latest_scale=(100.0,9.99)
    point=single_prop("upper",minimum=50,maximum=50,step=5).points[0]; sample=engine._joined_sample("r","upper",point)
    assert sample.upper_erpm==14000 and sample.lower_erpm==28000
    assert sample.upper_command_pct==50 and sample.lower_command_pct is None
    assert sample.board_current_a==3.0 and sample.thrust_n==0.980665

def test_nonce_and_generation_mismatch_are_not_joined():
    conn=Connection(); store=Store(); engine=AcquisitionEngine(conn,None,store,clock=lambda:10.0)
    payload=fc_protocol._FRAME.pack(1,fc_protocol.PAYLOAD_BYTES,0,99,1,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
    conn.events.put(LinkEvent("fc_payload",1,9.0,payload)); conn.events.put(LinkEvent("fc_payload",2,9.1,payload)); engine._drain_events()
    assert [x[0] for x in store.events]==["stale_generation","stale_nonce"] and engine._latest_fc is None
    assert store.fc[0]["classification"]=="stale_or_duplicate_nonce"

def test_unique_session_round_trip_and_raw_evidence(tmp_path):
    meta={"two_propellers_installed":True,"current_calibration":{"confirmed":False}}
    first=SessionStore(meta,tmp_path); second=SessionStore(meta,tmp_path); assert first.path!=second.path
    engine=AcquisitionEngine(Connection(),None,first,clock=lambda:10.0); engine._latest_fc=(snapshot(),9.99,2,1000); engine._latest_scale=(100.0,9.99)
    first.sample(engine._joined_sample("run","dual",common_sweep(minimum=0,maximum=0,step=1).points[0])); first.raw_fc(host_time_s=10,raw_current_a=3.0); first.event("crc_error")
    assert read_samples(first.samples_path)[0].upper_erpm==14000
    metadata=json.loads((first.path/"metadata.json").read_text(encoding="utf-8"))
    assert metadata["two_propellers_installed"] is True
    assert metadata["fc_link_rate"]["rate_policy"]=="offline_fake_explicit"
    assert first.fc_raw_path.exists() and first.events_path.exists()

def test_commands_are_atomic_and_arm_limit_is_explicit():
    assert fc_protocol.arm_command(20,request_id=5)=="TBENCH ARM confirm=bench max_pct=20 request_id=5"
    assert fc_protocol.set_command(7,9,request_id=6,window_token=5)=="TBENCH SET upper_pct=7 lower_pct=9 request_id=6 token=5"
    assert fc_protocol.stop_command()=="TBENCH STOP"

def test_fc_connection_reuses_canonical_x_framing_and_crc_parser():
    events=queue.Queue(); connection=FlightControllerConnection(events)
    payload=fc_protocol._FRAME.pack(1,fc_protocol.PAYLOAD_BYTES,0,7,1,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
    wire=bytearray(build_proto_frame(PROTO_DIR_FROM_FC,fc_protocol.FUNCTION_ID,payload))
    class Context:
        generation=37; received_at=123.5
        def is_current(self,transport): return True
    connection.transport._consume_buffer(wire,context=Context())
    event=events.get_nowait()
    assert event.kind=="fc_payload" and event.data==payload and event.generation==37 and event.host_time_s==123.5 and wire==bytearray()

def test_board_adc_current_is_diagnostic_and_never_calibrates_dshot_current():
    engine=AcquisitionEngine(Connection(),None,Store(),clock=lambda:10.0)
    mapped=replace(snapshot(),erpm=(14000,14000),erpm_age_ms=(0,0),
                   total_current_a=3.0,current_calibrated=True,
                   esc_current_a=(4.0,5.0),esc_current_age_ms=(10,20))
    engine._latest_fc=(mapped,10.0,2,1000); engine._latest_scale=(100.0,10.0)
    sample=engine._joined_sample("r","dual",common_sweep(minimum=0,maximum=0,step=1).points[0])
    assert sample.board_current_a==3.0 and sample.board_current_calibrated
    assert sample.upper_esc_current_a==4.0 and sample.lower_esc_current_a==5.0
    assert sample.current_source=="dshot" and not sample.esc_current_calibrated


def test_dshot_currents_follow_role_mapping_and_expire_at_one_second():
    engine=AcquisitionEngine(Connection(),None,Store(),clock=lambda:10.02)
    mapped=replace(snapshot(),upper_channel=2,lower_channel=1,
                   erpm=(14000,28000),erpm_age_ms=(0,0),
                   esc_current_a=(4.0,5.0),esc_current_age_ms=(990,5))
    engine._latest_fc=(mapped,10.0,2,1000); engine._latest_scale=(100.0,10.0)
    sample=engine._joined_sample("r","dual",common_sweep(minimum=0,maximum=0,step=1).points[0])
    assert sample.upper_erpm==28000 and sample.lower_erpm==14000
    assert sample.upper_esc_current_a==5.0 and sample.lower_esc_current_a is None
    assert sample.upper_esc_current_age_ms==5 and sample.lower_esc_current_age_ms==990
    assert "lower_esc_current_stale" in sample.quality


def test_one_missing_dshot_current_is_explicit_but_not_a_motor_stop_gate():
    engine=AcquisitionEngine(Connection(),None,Store(),clock=lambda:10.0)
    source=replace(snapshot(),erpm=(14000,14000),erpm_age_ms=(0,0),
                   esc_current_a=(None,5.0),esc_current_age_ms=(None,5))
    engine._latest_fc=(source,10.0,2,1000); engine._latest_scale=(100.0,10.0)
    sample=engine._joined_sample("r","dual",common_sweep(minimum=0,maximum=0,step=1).points[0])
    assert sample.upper_esc_current_a is None and sample.lower_esc_current_a==5.0
    assert "upper_esc_current_missing" in sample.quality
    assert AcquisitionEngine._required_missing(sample)==[]

def test_dynamic_rise_and_fall_each_share_one_segment():
    plan=dynamic_steps(baseline=(5,5),axis="upper",delta=2,repeats=1)
    assert [p.segment_id for p in plan.points[1:]] == ["dyn-upper-00-rise"]*2+["dyn-upper-00-fall"]*2
    assert [(p.upper_percent,p.direction) for p in plan.points[1:]] == [(5,"up"),(7,"up"),(7,"down"),(5,"down")]

def test_ack_requires_tbench_prefix_generation_time_ids_token_and_targets():
    conn=Connection(); store=Store(); clock=[10.0]
    engine=AcquisitionEngine(conn,None,store,clock=lambda:clock[0])
    expected={"request_id":"8","token":"7","upper_pct":"5","lower_pct":"6"}
    engine._pending_ack=AckTransaction("set",expected,10.0,2)
    wrong=("OTHER state=set request_id=8 token=7 upper_pct=5.00 lower_pct=6.00",
           "TBENCH state=set request_id=8 token=9 upper_pct=5.00 lower_pct=6.00")
    for text in wrong: conn.events.put(LinkEvent("fc_text",2,10.1,text))
    engine._drain_events(); assert engine._pending_ack is not None
    conn.events.put(LinkEvent("fc_text",2,10.2,"TBENCH state=set request_id=8 token=7 upper_pct=5.00 lower_pct=6.00"))
    engine._drain_events(); assert engine._pending_ack is None

def test_stale_fc_and_scale_are_none_not_old_values():
    engine=AcquisitionEngine(Connection(),None,Store(),clock=lambda:20.0)
    engine._latest_fc=(snapshot(),19.0,2,1000); engine._latest_scale=(100.0,19.0)
    sample=engine._joined_sample("r","dual",common_sweep(minimum=0,maximum=0,step=1).points[0])
    assert sample.upper_erpm is None and sample.voltage_v is None and sample.thrust_n is None
    assert set(sample.quality)=={"fc_stale","scale_stale"}

def test_each_fc_source_expires_by_its_own_age_without_changing_source_time():
    engine=AcquisitionEngine(Connection(),None,Store(),clock=lambda:10.1)
    aged=replace(snapshot(),erpm=(14000,14000),erpm_age_ms=(200,10),voltage_age_ms=200,current_age_ms=10)
    engine._latest_fc=(aged,10.0,2,0x1_0000_0100); engine._latest_scale=(100.0,10.0)
    sample=engine._joined_sample("r","dual",common_sweep(minimum=0,maximum=0,step=1).points[0])
    assert sample.fc_time_ms==0x1_0000_0100
    assert sample.upper_erpm is None and sample.lower_erpm==14000
    assert sample.voltage_v is None and sample.board_current_a==3.0
    assert sample.lower_erpm_age_ms==10 and "upper_erpm_stale" in sample.quality

def test_stop_cancels_active_run_and_second_run_is_rejected():
    engine=AcquisitionEngine(Connection(),None,Store())
    engine._running=True; engine._armed=True; engine.stop()
    assert engine.cancel.is_set() and not engine._armed
    import pytest
    with pytest.raises(RuntimeError): engine.run_plan(common_sweep(minimum=0,maximum=0,step=1),max_percent=20)

def test_transport_binary_sink_opt_in_context_preserves_old_two_arg_api():
    from tools.panel_lib.transport import SerialTransport
    received=[]; transport=SerialTransport(queue.Queue())
    transport.set_binary_sink(lambda fn,payload: received.append((fn,payload)))
    transport._deliver_binary(1,b"old",object())
    transport.set_binary_sink(lambda fn,payload,context: received.append((fn,payload,context.generation)),with_context=True)
    class C: generation=4
    transport._deliver_binary(2,b"new",C())
    assert received==[(1,b"old"),(2,b"new",4)]

def test_corrupt_frame_reports_original_context_then_parser_recovers_good_frame():
    events=queue.Queue(); connection=FlightControllerConnection(events)
    payload=fc_protocol._FRAME.pack(1,fc_protocol.PAYLOAD_BYTES,0,7,1,1,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)
    good=build_proto_frame(PROTO_DIR_FROM_FC,fc_protocol.FUNCTION_ID,payload); bad=bytearray(good); bad[-1]^=0xff
    class Context:
        generation=9; received_at=42.25
        def is_current(self,transport): return True
    buffer=bad+good; connection.transport._consume_buffer(buffer,context=Context())
    first,second=events.get_nowait(),events.get_nowait()
    assert first.kind=="fc_frame_error" and first.data["error"]=="crc" and first.generation==9
    assert second.kind=="fc_payload" and second.data==payload and not buffer

def test_frame_error_observer_is_bounded_but_counter_is_exact():
    events=queue.Queue(); connection=FlightControllerConnection(events)
    class Context:
        generation=3; received_at=1.0
        def is_current(self,transport): return True
    for _ in range(10): connection.transport._consume_buffer(bytearray(b"$X\xff\x00\x00\x00\x00\x00\x00"),context=Context())
    assert connection.transport.frame_error_counts["invalid_direction"]==10
    assert events.qsize()==5  # 1,2,3,4,8 only

def test_voltage_layer_is_a_target_label_while_samples_keep_measured_voltage(tmp_path):
    plan=with_voltage_layer(common_sweep(minimum=5,maximum=5,step=1),label="中",target_v=11.5)
    assert plan.points[0].voltage_layer_v==11.5 and plan.points[0].voltage_layer_label=="中"
    store=SessionStore({"runs":[]},tmp_path); clock=[10.0]
    engine=AcquisitionEngine(Connection(),None,store,clock=lambda:clock[0])
    measured=replace(snapshot(),voltage_v=11.18)
    engine._latest_fc=(measured,10.0,2,1000); engine._latest_scale=(100.0,10.0)
    sample=engine._joined_sample("r","dual",plan.points[0])
    assert sample.voltage_layer_v==11.5 and sample.voltage_v==11.18
    store.record_run({"run_id":"r","voltage_layer_label":"中","voltage_layer_target_v":11.5,
                      "voltage_target_is_measured":False,"loaded_voltage_min_v":11.12,
                      "loaded_voltage_max_v":11.22})
    run=json.loads((store.path/"metadata.json").read_text(encoding="utf-8"))["runs"][0]
    assert run["voltage_target_is_measured"] is False and run["loaded_voltage_min_v"]==11.12

def test_request_id_starts_after_fresh_snapshot_global_sequence():
    assert AcquisitionEngine._next_request_id(41)==42
    assert AcquisitionEngine._next_request_id(0xffffffff)==1

def test_arm_refuses_non_bidirectional_dshot_before_sending_command():
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store(),clock=lambda:10.0)
    engine._latest_fc=(snapshot(),10.0,2,1000)
    assert engine.arm(20) is False and conn.sent==[]
    assert "DSHOT300_BIDIR" in engine.ui_events.get_nowait().data

def test_health_monitor_stops_after_bounded_gap_during_settle():
    for failed_source in ("scale","fc","rpm"):
        conn=Connection(); store=Store(); engine=AcquisitionEngine(
            conn,None,store)
        now=time.monotonic()
        live=replace(snapshot(),esc_protocol=2,erpm=(14000,14000),erpm_age_ms=(0,0))
        engine._latest_fc=(live,now,2,1000); engine._latest_scale=(100.0,now)
        engine._armed=True; engine._stream_established=True
        engine._max_pct=20; engine._window_token=5; engine._request_id=5
        if failed_source=="scale": engine._latest_scale=None
        elif failed_source=="fc": engine._latest_fc=None
        else:
            bad=replace(live,erpm=(None,14000),erpm_age_ms=(None,0))
            engine._latest_fc=(bad,now,2,1000)
        assert engine._monitor_active_health()
        engine._telemetry_gap_started-=engine.TELEMETRY_GAP_GRACE_S+.01
        assert not engine._monitor_active_health()
        assert conn.sent.count("TBENCH STOP")>=2

def test_startup_stream_grace_is_short_and_available_only_once():
    clock=[10.0]; conn=Connection(); engine=AcquisitionEngine(
        conn,None,Store(),clock=lambda:clock[0])
    engine._armed=True; engine._startup_grace_used=True; engine._startup_grace_until=10.6
    assert engine._monitor_active_health() is True
    clock[0]=10.61
    assert engine._monitor_active_health() is True
    clock[0]+=engine.TELEMETRY_GAP_GRACE_S+.01
    assert engine._monitor_active_health() is False
    assert conn.sent.count("TBENCH STOP")==2

def _snapshot_payload(*,nonce,fc_ms):
    return fc_protocol._FRAME.pack(
        1,fc_protocol.PAYLOAD_BYTES,0,nonce,fc_ms,2,1,2,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,5)

def test_fc_time_wrap_out_of_order_and_reset_are_distinguished():
    cases=((0xfffffff0,1.0,10,2.0,"wrap",True),
           (100,2.0,200,1.0,"out_of_order_snapshot",False),
           (1000,1.0,900,2.0,"fc_time_reset",False))
    for previous,last_request,received,request_time,expected,accepted in cases:
        conn=Connection(); store=Store(); engine=AcquisitionEngine(
            conn,None,store,clock=lambda:3.0)
        engine._last_fc_raw=previous; engine._last_snapshot_request_time=last_request
        engine._pending_snapshots[7]=(request_time,2)
        conn.events.put(LinkEvent("fc_payload",2,request_time+.01,
                                  _snapshot_payload(nonce=7,fc_ms=received)))
        engine._drain_events()
        if expected=="wrap":
            assert engine._fc_epoch==1<<32 and engine._last_fc_raw==received
        else:
            assert any(kind==expected for kind,_ in store.events)
            assert engine._last_fc_raw==previous
        if expected=="fc_time_reset": assert conn.sent.count("TBENCH STOP")==2

def test_duplicate_and_regressing_source_times_are_not_new_samples():
    engine=AcquisitionEngine(Connection(),None,Store())
    assert engine._classify_sample_sources((100,100))=="new"
    engine._last_emitted_source_times=(100,100)
    assert engine._classify_sample_sources((100,100))=="duplicate"
    assert engine._classify_sample_sources((101,99))=="regression"
    assert engine._classify_sample_sources((101,101))=="new"

def test_default_wire_budget_uses_real_ascii_and_x_envelopes():
    assert estimated_wire_bytes_per_second(15,10)==3405
    assert estimated_wire_bytes_per_second(15,10)<3456
    assert estimated_wire_bytes_per_second(20,10)>3456

def test_real_connection_rejects_non_usb_high_rate_before_opening_port():
    class Transport:
        connection_generation=0; is_connected=False
        def __init__(self): self.started=0
        def set_binary_sink(self,*args,**kwargs): pass
        def set_frame_error_sink(self,*args,**kwargs): pass
        def start(self,*args): self.started+=1
        def stop(self): pass
        def send_frame(self,*args): return True
    transport=Transport()
    connection=FlightControllerConnection(
        queue.Queue(),transport=transport,
        identity_resolver=lambda port:{"device":port,"vid":0x1A86,"pid":0x7523,
                                      "description":"CH340","hwid":"USB CH340"})
    import pytest
    with pytest.raises(ValueError,match="STM32 USB CDC"):
        connection.connect("COM9",snapshot_hz=20)
    assert transport.started==0

def test_real_connection_allows_confirmed_stm32_usb_high_rate_and_unknown_default():
    class Transport:
        def __init__(self): self.connection_generation=0; self.is_connected=False; self.started=0
        def set_binary_sink(self,*args,**kwargs): pass
        def set_frame_error_sink(self,*args,**kwargs): pass
        def start(self,*args): self.started+=1; self.is_connected=True; self.connection_generation+=1
        def stop(self): self.is_connected=False; self.connection_generation+=1
        def send_frame(self,*args): return True
    usb_transport=Transport()
    usb=FlightControllerConnection(
        queue.Queue(),transport=usb_transport,
        identity_resolver=lambda port:{"device":port,"vid":0x0483,"pid":0x5740,
                                      "description":"STM32 Virtual COM Port","hwid":"USB"})
    usb.connect("COM10",snapshot_hz=40)
    assert usb_transport.started==1 and usb.last_rate_decision["confirmed_stm32_usb_cdc"]
    usb.disconnect()
    unknown_transport=Transport()
    unknown=FlightControllerConnection(
        queue.Queue(),transport=unknown_transport,identity_resolver=lambda port:None)
    unknown.connect("COM11",snapshot_hz=15)
    assert unknown_transport.started==1
    assert unknown.last_rate_decision["rate_policy"]=="serial_budget"
    unknown.disconnect()

def test_plan_set_resets_heartbeat_deadline_and_prevents_immediate_duplicate():
    clock=[0.0]; conn=Connection(); engine=AcquisitionEngine(
        conn,None,Store(),clock=lambda:clock[0])
    live=replace(snapshot(),esc_protocol=2,erpm=(14000,14000),erpm_age_ms=(0,0))
    engine._latest_fc=(live,0.0,2,1000); engine._latest_scale=(100.0,0.0)
    engine._armed=True; engine._stream_established=True
    engine._max_pct=20; engine._window_token=5; engine._request_id=5
    transaction=engine._begin_set(6,7,wait_for_slot=True)
    assert transaction is not None and len(conn.sent)==1
    with engine._ack:
        transaction.outcome="matched"; engine._pending_ack=None; engine._ack.notify_all()
    thread=threading.Thread(target=engine._heartbeat_loop); thread.start()
    time.sleep(.13)
    assert len(conn.sent)==1, "计划SET后的100ms窗口内不应补发无意义心跳"
    clock[0]=.101
    deadline=time.monotonic()+.25
    while len(conn.sent)<2 and time.monotonic()<deadline: time.sleep(.005)
    engine._shutdown.set(); thread.join(.3)
    assert len(conn.sent)==2

def test_consecutive_plan_targets_share_the_ten_hz_set_budget():
    conn=Connection(); engine=AcquisitionEngine(
        conn,None,Store())
    engine._armed=True; engine._max_pct=20; engine._window_token=5; engine._request_id=5
    first=engine._begin_set(5,5,wait_for_slot=True)
    with engine._ack:
        first.outcome="matched"; engine._pending_ack=None; engine._ack.notify_all()
    started=time.monotonic(); second=engine._begin_set(6,6,wait_for_slot=True)
    assert second is not None and time.monotonic()-started>=.08
    assert len(conn.sent)==2

def test_cancelled_arm_transaction_cannot_resurrect_armed_state():
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    fresh=replace(snapshot(),esc_protocol=2,last_request_id=40)
    engine._latest_fc=(fresh,time.monotonic(),2,1000)
    result=[]; thread=threading.Thread(target=lambda:result.append(engine.arm(20))); thread.start()
    deadline=time.monotonic()+.2
    while not conn.sent and time.monotonic()<deadline: time.sleep(.002)
    engine.stop(); thread.join(.5)
    assert result==[False] and not engine._armed

def test_new_set_waits_for_heartbeat_transaction_then_sends_own_target():
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    engine._armed=True; engine._max_pct=20; engine._window_token=5; engine._request_id=6
    heartbeat=AckTransaction("set",{"request_id":"6"},time.monotonic(),2)
    engine._pending_ack=heartbeat; result=[]
    thread=threading.Thread(target=lambda:result.append(engine._begin_set(8,9,wait_for_slot=True))); thread.start()
    time.sleep(.02); assert not conn.sent
    with engine._ack:
        heartbeat.outcome="matched"; engine._pending_ack=None; engine._ack.notify_all()
    thread.join(.5)
    assert result[0] is not None and result[0].expected["upper_pct"]=="8"
    assert any("upper_pct=8 lower_pct=9" in command for command in conn.sent)


def test_plan_waiter_has_priority_over_heartbeat_after_rate_wait():
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    engine._armed=True; engine._max_pct=20; engine._window_token=5
    engine._request_id=10; engine._command=(2.0,3.0)
    engine._last_set_send_time=time.monotonic()
    result=[]
    thread=threading.Thread(
        target=lambda:result.append(engine._try_begin_set(8,9,wait_for_slot=True)))
    thread.start()
    deadline=time.monotonic()+.2
    while engine._plan_set_waiters!=1 and time.monotonic()<deadline: time.sleep(.002)
    heartbeat,reason=engine._try_begin_set(None,None,wait_for_slot=False)
    assert heartbeat is None and reason=="yield_to_plan"
    thread.join(.4)
    transaction,reason=result[0]
    assert reason=="sent" and transaction.expected["upper_pct"]=="8"
    assert len(conn.sent)==1 and "upper_pct=8 lower_pct=9" in conn.sent[0]


def test_old_begin_set_interface_reproduces_and_prevents_heartbeat_slot_theft():
    """Negative control on 996d6bf0: heartbeat steals the slot, plan returns None."""
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    engine._armed=True; engine._max_pct=20; engine._window_token=5
    engine._request_id=10; engine._command=(2.0,3.0)
    engine._last_set_send_time=time.monotonic()
    planned=[]
    thread=threading.Thread(
        target=lambda:planned.append(
            engine._begin_set(8,9,wait_for_slot=True)))
    thread.start()
    time.sleep(.02)  # scanner is inside the 100 ms rate wait
    heartbeat=engine._begin_set(
        *engine._command,wait_for_slot=False)
    thread.join(.4)
    assert heartbeat is None
    assert planned[0] is not None
    assert planned[0].expected["upper_pct"]=="8"
    assert len(conn.sent)==1 and "upper_pct=8 lower_pct=9" in conn.sent[0]


def test_heartbeat_loop_treats_plan_priority_as_normal_yield_not_failure():
    class OneShot:
        def __init__(self): self.calls=0
        def wait(self,_timeout): self.calls+=1; return self.calls>1
        def is_set(self): return False
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    engine._shutdown=OneShot(); engine._armed=True; engine._plan_set_waiters=1
    engine._monitor_active_health=lambda:True
    engine._heartbeat_loop()
    assert not any(command=="TBENCH STOP" for command in conn.sent)


def test_heartbeat_does_not_timeout_transaction_already_matched_under_lock():
    class OneShot:
        def __init__(self): self.calls=0
        def wait(self,_timeout): self.calls+=1; return self.calls>1
        def is_set(self): return False
    conn=Connection(); store=Store(); engine=AcquisitionEngine(conn,None,store)
    engine._shutdown=OneShot(); engine._armed=True; engine._monitor_active_health=lambda:True
    transaction=AckTransaction("set",{"request_id":"9"},time.monotonic()-1,2,outcome="matched")
    engine._pending_ack=transaction
    engine._heartbeat_loop()
    assert not any(command=="TBENCH STOP" for command in conn.sent)
    assert not any(kind=="command_ack" and data.get("result")=="timeout" for kind,data in store.events)


def test_heartbeat_timeout_cancels_window_before_plan_waiter_can_send():
    class OneShot:
        def __init__(self): self.calls=0
        def wait(self,_timeout): self.calls+=1; return self.calls>1
        def is_set(self): return False
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    engine._shutdown=OneShot(); engine._armed=True; engine._max_pct=20
    engine._monitor_active_health=lambda:True
    expired=AckTransaction("set",{"request_id":"7"},time.monotonic()-1,2)
    engine._pending_ack=expired; planned=[]
    thread=threading.Thread(
        target=lambda:planned.append(
            engine._begin_set(8,9,wait_for_slot=True)))
    thread.start(); time.sleep(.02)
    engine._heartbeat_loop(); thread.join(.4)
    assert planned==[None]
    assert not any("upper_pct=8 lower_pct=9" in command for command in conn.sent)
    assert conn.sent.count("TBENCH STOP")==2

def test_legacy_or_wrong_schema_csv_is_rejected(tmp_path):
    legacy=tmp_path/"legacy.csv"; legacy.write_text("pct,kv,thrust_g\n20,1300,500\n",encoding="utf-8")
    import pytest
    with pytest.raises(ValueError,match="thrust_bench v2 samples.csv"): read_samples(legacy)
    store=SessionStore({},tmp_path)
    engine=AcquisitionEngine(Connection(),None,store,clock=lambda:10.0)
    engine._latest_fc=(snapshot(),9.99,2,1000); engine._latest_scale=(100.0,9.99)
    store.sample(engine._joined_sample("r","dual",common_sweep(minimum=0,maximum=0,step=1).points[0]))
    text=store.samples_path.read_text(encoding="utf-8").replace("2,", "1,",1)
    store.samples_path.write_text(text,encoding="utf-8")
    with pytest.raises(ValueError,match="schema_version"): read_samples(store.samples_path)

def test_logging_failure_stops_all_engine_workers_and_sends_stop():
    class FailingStore(Store):
        def event(self,kind,**data): raise OSError("disk full")
    conn=Connection(); engine=AcquisitionEngine(conn,None,FailingStore())
    engine._pending_snapshots[1]=(time.monotonic()-1,2); engine.start_io()
    deadline=time.monotonic()+.5
    while not engine._shutdown.is_set() and time.monotonic()<deadline: time.sleep(.005)
    assert engine._shutdown.is_set()
    assert conn.sent.count("TBENCH STOP")>=2
    engine.close()

def test_reconnect_joins_old_connection_reader_before_starting_new_one():
    class Transport:
        def __init__(self): self.connection_generation=0; self.is_connected=False
        def set_binary_sink(self,*args,**kwargs): pass
        def set_frame_error_sink(self,*args,**kwargs): pass
        def start(self,*args): self.connection_generation+=1; self.is_connected=True
        def stop(self): self.connection_generation+=1; self.is_connected=False
        def send_frame(self,*args): return self.is_connected
    connection=FlightControllerConnection(queue.Queue(),transport=Transport())
    connection.connect("FAKE1"); old=connection._reader
    connection.connect("FAKE2"); new=connection._reader
    assert old is not None and not old.is_alive() and new is not old and new.is_alive()
    connection.disconnect(); assert not new.is_alive()

def test_dynamic_plan_through_acquisition_writes_v2_erpm_samples(tmp_path):
    store=SessionStore({"source":"fake_fc_and_scale_contract"},tmp_path)
    clock=[0.0]; engine=AcquisitionEngine(Connection(),None,store,
        clock=lambda:clock[0])
    plan=with_voltage_layer(dynamic_steps(baseline=(5,5),axis="upper",delta=3,repeats=1,hold_s=1.2),label="高",target_v=12.3)
    samples=[]; rpm=2000.0; fc_ms=1000
    for point in plan.points:
        target=3000.0 if point.upper_percent>5 else 2000.0
        for _ in range(24):
            rpm=target+(rpm-target)*math.exp(-.05/.2); fc_ms+=50; clock[0]+=.05
            pulse=1100+int(840*point.upper_percent)//100
            snap=FcSnapshot(nonce=1,fc_time_ms=fc_ms,esc_protocol=1,upper_channel=1,lower_channel=2,
                command_us=(pulse,1142),erpm=(round(rpm*7),14000),erpm_age_ms=(0,0),
                voltage_v=11.9,total_current_a=3.0,voltage_age_ms=0,current_age_ms=0,
                bench_active=True,bench_max_percent=20,last_request_id=5)
            engine._latest_fc=(snap,clock[0],2,fc_ms); engine._latest_scale=(100.0,clock[0])
            sample=engine._joined_sample("run-dynamic","dual",point)
            samples.append(sample); store.sample(sample)
    loaded=read_samples(store.samples_path)
    assert loaded and all(sample.speed_source=="dshot_erpm" for sample in loaded)
    assert all(sample.upper_erpm is not None for sample in loaded)
    assert all(not hasattr(sample,"upper_rpm") for sample in loaded)


def test_set_rounds_to_board_resolution_so_the_echo_matches():
    # 2026-09-23 024554-fea681fc: SET upper_pct=8.125 was echoed as 8.13 and timed out.
    conn=Connection(); store=Store(); clock=[10.0]
    engine=AcquisitionEngine(conn,None,store,clock=lambda:clock[0])
    engine._armed=True; engine._max_pct=100; engine._window_token=7; engine._request_id=7
    transaction,reason=engine._try_begin_set(8.125,93.875,wait_for_slot=False)
    assert reason=="sent" and "upper_pct=8.13 lower_pct=93.88" in conn.sent[-1]
    assert engine._command==(8.13,93.88)
    conn.events.put(LinkEvent("fc_text",2,10.01,
        "TBENCH state=set active=1 upper_pct=8.13 lower_pct=93.88 age_ms=0 request_id=8 token=7"))
    engine._drain_events()
    assert transaction.outcome=="matched" and engine._pending_ack is None


def test_esc_replies_are_routed_for_the_kv_writer():
    conn=Connection(); engine=AcquisitionEngine(conn,None,Store())
    line="ESC KV event=queued state=sending kv=1300 byte=32 sent=0 frames=15 reason=none proto=2"
    conn.events.put(LinkEvent("fc_text",2,10.0,line)); engine._drain_events()
    assert engine.esc_events.get_nowait().data==line


class KvBoard:
    """Replies in the exact app_cmd_esc_kv.c format."""
    def __init__(self,script):
        self.events=queue.Queue(); self.sent=[]; self.script=list(script); self.generation=2
    def send_command(self,text):
        self.sent.append(text)
        if self.script:
            reply=self.script.pop(0)
            if reply: self.events.put(LinkEvent("fc_text",2,time.monotonic(),reply))
        return True


def run_kv_writer(script):
    from tools.thrust_bench.esc_kv import EscKvWriter
    board=KvBoard(script); owner=[]
    engine=SimpleNamespace(connection=board,esc_events=board.events,
        claim_operation=lambda name:(owner.append(name) or True),release_operation=lambda name:owner.remove(name))
    writer=EscKvWriter(engine)
    assert writer.start(); writer._worker.join(5)
    messages=[]
    while not writer.events.empty(): messages.append(writer.events.get_nowait().data)
    return writer,board,owner,messages


def test_kv_writer_waits_for_the_whole_sequence_then_asks_for_a_power_cycle():
    status="ESC KV event={} state={} kv=1300 byte=32 sent={} frames=15 reason=none proto=2"
    writer,board,owner,messages=run_kv_writer([status.format("queued","sending",0),status.format("status","done",15)])
    assert board.sent==["ESC KV 1300 CONFIRM","ESC KV ?"]
    assert writer.state=="finished" and owner==[]
    assert "15/15" in messages[-1]["message"] and "拔掉电池再插上" in messages[-1]["message"]


def test_kv_writer_reports_rejection_old_firmware_and_broken_sequence():
    writer,_,owner,messages=run_kv_writer(["ESC KV event=rejected reason=armed"])
    assert writer.state=="error" and "解锁状态" in messages[-1]["message"] and owner==[]
    writer,_,_,messages=run_kv_writer([None])
    assert writer.state=="error" and "旧固件" in messages[-1]["message"]
    writer,_,_,messages=run_kv_writer(["ESC KV event=queued state=aborted kv=1300 byte=32 sent=7 frames=15 reason=inhibited proto=2"])
    assert writer.state=="error" and "7/15" in messages[-1]["message"] and "拔插电池" in messages[-1]["message"]
