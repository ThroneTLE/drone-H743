from __future__ import annotations

import queue
import threading
import time
from dataclasses import replace

import pytest

from tools.thrust_bench.acquisition import AcquisitionEngine
from tools.thrust_bench.manual_control import ManualControl
from tools.thrust_bench.records import FcSnapshot


class FakeEngine:
    def __init__(self):
        self.owner=None; self._armed=False; self.arm_result=True; self.set_result=True
        self.stops=0; self.targets=[]; self.set_gate=None; self.arm_gate=None
    def claim_operation(self,owner):
        if self.owner is not None: return False
        self.owner=owner; return True
    def release_operation(self,owner):
        if self.owner==owner: self.owner=None
    @property
    def armed(self): return self._armed
    def arm(self,max_percent):
        if self.arm_gate is not None: self.arm_gate.wait(.7)
        self._armed=self.arm_result; return self.arm_result
    def set_targets(self,upper,lower):
        self.targets.append((upper,lower))
        if self.set_gate is not None: self.set_gate.wait(.5)
        return self.set_result
    def stop(self): self.stops+=1; self._armed=False; return True
    def latest_observation(self): return "observation"


def wait_state(control,state,timeout=.7):
    deadline=time.monotonic()+timeout
    while control.state!=state and time.monotonic()<deadline: time.sleep(.005)
    assert control.state==state


def test_arm_owns_engine_and_emits_ui_event():
    engine=FakeEngine(); control=ManualControl(engine)
    assert control.arm(20); wait_state(control,"armed")
    assert control.busy and engine.owner=="manual"
    events=[]
    while not control.events.empty(): events.append(control.events.get_nowait())
    assert events[-1].kind=="manual_state" and events[-1].data["state"]=="armed"
    control.close(); assert engine.owner is None and engine.stops==1


def test_apply_is_explicit_and_hold_timeout_stops():
    engine=FakeEngine(); control=ManualControl(engine)
    control.arm(20); wait_state(control,"armed")
    assert engine.targets==[],"draft slider changes cannot send without apply()"
    assert control.apply(6,8,.06); wait_state(control,"holding")
    assert engine.targets==[(6.0,8.0)]
    wait_state(control,"idle"); assert engine.stops==1 and engine.owner is None


def test_reapply_replaces_hold_deadline_without_early_stop():
    engine=FakeEngine(); control=ManualControl(engine)
    control.arm(20); wait_state(control,"armed")
    control.apply(5,5,.15); wait_state(control,"holding"); time.sleep(.05)
    control.apply(7,9,.20); wait_state(control,"holding"); time.sleep(.12)
    assert engine.stops==0 and engine.targets==[(5.0,5.0),(7.0,9.0)]
    wait_state(control,"idle",.3); assert engine.stops==1


def test_stop_cancels_inflight_apply_and_never_restores_target():
    engine=FakeEngine(); gate=threading.Event(); engine.set_gate=gate
    control=ManualControl(engine); control.arm(20); wait_state(control,"armed")
    control.apply(4,4,.5); wait_state(control,"applying")
    control.stop(); gate.set(); wait_state(control,"idle"); time.sleep(.03)
    assert engine.stops>=2 and engine.owner is None and control.state=="idle"


def test_engine_safety_stop_during_hold_becomes_error_without_resend():
    engine=FakeEngine(); control=ManualControl(engine)
    control.arm(20); wait_state(control,"armed")
    control.apply(5,6,.5); wait_state(control,"holding")
    engine._armed=False
    wait_state(control,"error")
    assert engine.owner is None and engine.targets==[(5.0,6.0)]
    control.close()


def test_health_stop_drains_blocked_set_before_releasing_lease():
    engine=FakeEngine(); gate=threading.Event(); engine.set_gate=gate
    control=ManualControl(engine); control.arm(20); wait_state(control,"armed")
    control.apply(5,6,.5); wait_state(control,"applying")
    engine._armed=False
    wait_state(control,"stopping")
    assert not engine.claim_operation("plan"),"manual lease must remain until old SET exits"
    gate.set(); wait_state(control,"error")
    events=[]
    while not control.events.empty(): events.append(control.events.get_nowait().data["state"])
    stopping_index=events.index("stopping")
    assert "holding" not in events[stopping_index:]
    assert engine.owner is None and engine.stops>=2


def test_engine_safety_stop_before_first_apply_releases_armed_lease():
    engine=FakeEngine(); control=ManualControl(engine)
    control.arm(20); wait_state(control,"armed"); engine._armed=False
    wait_state(control,"error")
    assert engine.owner is None and engine.targets==[]


def test_arm_failure_always_stops_possible_unacknowledged_window():
    engine=FakeEngine(); engine.arm_result=False; control=ManualControl(engine)
    control.arm(20); wait_state(control,"error")
    assert engine.stops==1 and engine.owner is None


def test_mapping_rejection_is_retained_and_explicit_retry_can_recover():
    engine=FakeEngine(); engine.arm_result=False
    engine.last_rejection_reason="mapping_unconfirmed"
    control=ManualControl(engine)
    assert control.arm(20); wait_state(control,"error")
    assert "映射" in control.last_error and "上下桨" in control.last_error
    assert engine.targets==[] and engine.owner is None
    engine.arm_result=True; engine.last_rejection_reason=""
    assert control.arm(20); wait_state(control,"armed")
    assert control.last_error=="" and engine.targets==[]
    control.close()


def test_error_retry_cannot_bypass_an_undrained_owner():
    control=ManualControl(FakeEngine())
    with control._lock:
        control._state="error"; control._owns=True
    assert not control.arm(20)


def test_stop_is_generation_barrier_against_late_arm_success():
    engine=FakeEngine(); gate=threading.Event(); engine.arm_gate=gate
    control=ManualControl(engine); assert control.arm(20); wait_state(control,"arming")
    stopped=[]; stopper=threading.Thread(target=lambda:stopped.append(control.stop())); stopper.start()
    time.sleep(.03); assert stopper.is_alive(),"stop must retain lease until old ARM exits"
    gate.set(); stopper.join(.8); wait_state(control,"idle")
    events=[]
    while not control.events.empty(): events.append(control.events.get_nowait().data["state"])
    stopping_index=events.index("stopping")
    assert "armed" not in events[stopping_index:]
    assert engine.owner is None and engine.stops>=2


def test_old_monitor_exits_before_fast_rearm_can_claim_new_session():
    engine=FakeEngine(); control=ManualControl(engine)
    control.arm(20); wait_state(control,"armed"); old_monitor=control._monitor_thread
    control.stop(); assert old_monitor is not None and not old_monitor.is_alive()
    assert control.arm(20); wait_state(control,"armed"); time.sleep(.06)
    assert control.state=="armed" and engine.owner=="manual"
    control.close()


def test_manual_and_plan_operation_owners_are_mutually_exclusive():
    engine=FakeEngine(); engine.owner="plan"; control=ManualControl(engine)
    assert not control.arm(20) and control.state=="error"


def test_idle_manual_stop_cannot_stop_a_plan_it_does_not_own():
    engine=FakeEngine(); engine.owner="plan"; control=ManualControl(engine)
    assert control.stop() is False
    assert engine.stops==0 and engine.owner=="plan"


def test_apply_validates_targets_and_hold():
    control=ManualControl(FakeEngine())
    with pytest.raises(ValueError): control.apply(float("nan"),0,1)
    with pytest.raises(ValueError): control.apply(101,0,1)
    with pytest.raises(ValueError): control.apply(0,0,0)


def test_latest_observation_drops_old_sources_even_when_frame_arrived_fresh():
    from test_thrust_bench_acquisition import Connection, Store, snapshot
    clock=[10.0]
    engine=AcquisitionEngine(Connection(),None,Store(),clock=lambda:clock[0])
    source=replace(snapshot(),erpm=(14000,14000),erpm_age_ms=(240,5),
                   voltage_age_ms=240,current_age_ms=5,
                   esc_current_a=(4.0,5.0),esc_current_age_ms=(990,5))
    engine._latest_fc=(source,9.98,2,1000); engine._latest_scale=(123.0,9.98)
    observed=engine.latest_observation()
    assert observed.snapshot.erpm==(None,14000)
    assert observed.snapshot.voltage_v is None
    assert observed.snapshot.total_current_a==3.0
    assert observed.snapshot.erpm_age_ms==(None,25)
    assert observed.snapshot.current_age_ms==25
    assert observed.snapshot.esc_current_a==(None,5.0)
    assert observed.snapshot.esc_current_age_ms==(None,25)
    assert observed.scale_grams==123.0
    assert set(observed.quality)=={"erpm_1_stale","voltage_stale","esc_current_1_stale"}


def test_latest_observation_immediately_rejects_disconnect_generation_and_future_time():
    from test_thrust_bench_acquisition import Connection, Store, snapshot
    clock=[10.0]; connection=Connection()
    engine=AcquisitionEngine(connection,None,Store(),clock=lambda:clock[0])
    engine._latest_fc=(snapshot(),10.1,2,1000); engine._latest_scale=(1.0,10.1)
    connection.generation=3
    observed=engine.latest_observation()
    assert observed.snapshot is None and "fc_generation_stale" in observed.quality
    assert observed.scale_grams is None and "scale_time_invalid" in observed.quality
    connection.is_connected=False
    assert "fc_disconnected" in engine.latest_observation().quality


def test_engine_refuses_operation_claim_after_shutdown_or_disconnect():
    from test_thrust_bench_acquisition import Connection, Store
    connection=Connection(); engine=AcquisitionEngine(
        connection,None,Store())
    connection.is_connected=False
    assert not engine.claim_operation("manual")
    connection.is_connected=True; engine._shutdown.set()
    assert not engine.claim_operation("manual")
