from __future__ import annotations
import queue, threading, time
import pytest
from tools.thrust_bench.connection import LinkEvent
from tools.thrust_bench.mapping_control import MappingControl, estimated_mapping_wire_bytes_per_second


class Connection:
    def __init__(self): self.is_connected=True; self.generation=4; self.commands=[]; self.events=None; self.mapping=None; self.reject_arm=False
    def send_command(self,command):
        self.commands.append(command); now=time.monotonic()
        def emit(text): self.events.put(LinkEvent("fc_text",self.generation,now,text))
        if command=="PROPCAL?": self._mapping_reply(emit,"status")
        elif "SPIN ARM" in command:
            emit("PROPCAL state=spin_rejected reason=inhibited" if self.reject_arm else "PROPCAL spin=active ch=0 pct=0 age_ms=0 max_pct=20 timeout_ms=300 stop=-")
        elif "SPIN SET" in command:
            values=dict(token.split("=",1) for token in command.split() if "=" in token)
            emit(f"PROPCAL spin=active ch={values['ch']} pct={values['pct']} age_ms=0 max_pct=20 timeout_ms=300 stop=-")
        elif command=="PROPCAL SPIN STOP": emit("PROPCAL spin=idle ch=0 pct=0 age_ms=0 max_pct=0 timeout_ms=300 stop=request")
        elif command.startswith("PROPCAL SET"):
            values=dict(token.split("=",1) for token in command.split() if "=" in token); ch=int(values["ch"])
            if self.mapping is None: self.mapping={}
            self.mapping[ch]=values
            if len(self.mapping)==1: state="partial"
            elif len({v["role"] for v in self.mapping.values()})<2 or len({v["spin"] for v in self.mapping.values()})<2: state="conflict"
            else: state="applied_ram"
            self._mapping_reply(emit,state)
        elif command=="PROPCAL COMMIT": self._mapping_reply(emit,"committed")
        return True
    def validate_snapshot_rate(self,hz):
        return {"snapshot_hz":hz,"confirmed_stm32_usb_cdc":False}
    def _mapping_reply(self,emit,state):
        mapping=self.mapping or {1:{"role":"upper","spin":"cw"},2:{"role":"lower","spin":"ccw"}}
        calibrated="1" if state in {"applied_ram","committed","status"} and len(mapping)==2 else "0"
        upper=next((ch for ch,v in mapping.items() if v["role"]=="upper"),0)
        lower=next((ch for ch,v in mapping.items() if v["role"]=="lower"),0)
        emit(f"PROPCAL state={state} calibrated={calibrated} gen=2 dirty=1 upper_ch={upper} lower_ch={lower} lower_spin=ccw yaw_polarity=+1 reason=-")
        for ch in (1,2):
            item=mapping.get(ch,{"role":"none","spin":"none"})
            emit(f"PROPCAL ch={ch} pad=M{ch} role={item['role']} spin={item['spin']} declared={1 if ch in mapping else 0}")


class Engine:
    def __init__(self):
        self.connection=Connection(); self.propcal_events=queue.Queue(maxsize=128); self.connection.events=self.propcal_events; self.owner=None; self.snapshot_period=1/15
    def claim_operation(self,owner):
        if self.owner is not None: return False
        self.owner=owner; return True
    def release_operation(self,owner):
        if self.owner==owner: self.owner=None


def wait_state(control,state,timeout=1):
    deadline=time.monotonic()+timeout
    while control.state!=state and time.monotonic()<deadline: time.sleep(.005)
    assert control.state==state


def test_query_is_read_only_and_reports_channels():
    engine=Engine(); control=MappingControl(engine)
    assert control.query(); wait_state(control,"idle")
    assert engine.connection.commands==["PROPCAL?"]
    event=control.events.get_nowait(); event=control.events.get_nowait()
    assert event.kind=="mapping_state" and set(event.data["channels"])=={1,2}


def test_unconfirmed_channel_test_sends_nothing():
    engine=Engine(); control=MappingControl(engine)
    assert not control.test_channel(1,confirmed=False)
    assert engine.connection.commands==[] and engine.owner is None


def test_channel_test_arms_heartbeats_and_stops_at_deadline():
    engine=Engine(); control=MappingControl(engine)
    assert control.test_channel(2,percent=5,duration_s=.28,confirmed=True)
    wait_state(control,"idle")
    commands=engine.connection.commands
    assert commands[0]=="PROPCAL SPIN ARM confirm=safe max_pct=20"
    assert commands.count("PROPCAL SPIN SET ch=2 pct=5")>=2
    assert commands[-1]=="PROPCAL SPIN STOP" and engine.owner is None


def test_test_limits_and_wire_budget():
    control=MappingControl(Engine())
    for args in ((3,5,1),(1,0,1),(1,21,1),(1,5,3.1)):
        with pytest.raises(ValueError): control.test_channel(*args,confirmed=True)
    assert estimated_mapping_wire_bytes_per_second(15)<=3456


def test_apply_requires_opposite_roles_and_spins_then_stays_ram_only():
    engine=Engine(); control=MappingControl(engine)
    with pytest.raises(ValueError): control.apply_mapping({1:{"role":"upper","spin":"cw"},2:{"role":"upper","spin":"ccw"}})
    mapping={1:{"role":"lower","spin":"cw"},2:{"role":"upper","spin":"ccw"}}
    assert control.apply_mapping(mapping); wait_state(control,"idle")
    assert "PROPCAL COMMIT" not in engine.connection.commands
    assert all({key:engine.connection.mapping[ch][key] for key in ("role","spin")}==mapping[ch] for ch in (1,2))
    assert engine.connection.commands[0]=="PROPCAL SPIN STOP"


def test_reapplying_existing_complete_mapping_accepts_applied_ram_on_first_step():
    engine=Engine(); engine.connection.mapping={1:{"role":"upper","spin":"cw"},2:{"role":"lower","spin":"ccw"}}
    control=MappingControl(engine); mapping={1:{"role":"upper","spin":"cw"},2:{"role":"lower","spin":"ccw"}}
    assert control.apply_mapping(mapping); wait_state(control,"idle")
    assert engine.owner is None


def test_swapping_roles_accepts_intermediate_conflict_but_requires_final_full_match():
    engine=Engine(); engine.connection.mapping={1:{"role":"upper","spin":"ccw"},2:{"role":"lower","spin":"cw"}}
    control=MappingControl(engine); mapping={1:{"role":"lower","spin":"cw"},2:{"role":"upper","spin":"ccw"}}
    assert control.apply_mapping(mapping); wait_state(control,"idle")
    assert all(engine.connection.mapping[ch]["role"]==mapping[ch]["role"] for ch in (1,2))


def test_save_is_separate_and_requires_committed_reply():
    engine=Engine(); control=MappingControl(engine)
    assert control.save(); wait_state(control,"idle")
    assert engine.connection.commands==["PROPCAL SPIN STOP","PROPCAL COMMIT"]


def test_stop_is_generation_barrier_against_late_active_reply():
    engine=Engine(); control=MappingControl(engine)
    original=engine.connection.send_command
    gate=threading.Event()
    def delayed(command):
        if "SPIN SET" in command:
            engine.connection.commands.append(command); gate.wait(.5)
            engine.propcal_events.put(LinkEvent("fc_text",4,time.monotonic(),"PROPCAL spin=active ch=1 pct=5")); return True
        return original(command)
    engine.connection.send_command=delayed
    control.test_channel(1,5,.8,confirmed=True); time.sleep(.08)
    stopped=[]; thread=threading.Thread(target=lambda:stopped.append(control.stop())); thread.start(); time.sleep(.05)
    assert engine.owner=="mapping"
    gate.set(); thread.join(1); wait_state(control,"idle")
    count=len(engine.connection.commands); time.sleep(.05)
    assert len(engine.connection.commands)==count and engine.connection.commands[-1]=="PROPCAL SPIN STOP"


def test_stop_timeout_retains_lease_until_old_worker_reaper_finishes():
    engine=Engine(); control=MappingControl(engine); gate=threading.Event(); original=control._send_effect
    def blocked(generation,link_generation,command):
        if "SPIN SET" in command: gate.wait(2)
        return original(generation,link_generation,command)
    control._send_effect=blocked
    control.test_channel(1,5,1,confirmed=True); time.sleep(.08)
    assert control.stop() is False and engine.owner=="mapping"
    gate.set(); wait_state(control,"error",1)
    deadline=time.monotonic()+1
    while engine.owner is not None and time.monotonic()<deadline: time.sleep(.01)
    assert engine.owner is None


def test_stop_sends_immediate_safety_command_before_blocked_worker_drains():
    engine=Engine(); control=MappingControl(engine); gate=threading.Event(); original=control._send_effect
    def blocked(generation,link_generation,command):
        if "SPIN SET" in command: gate.wait(2)
        return original(generation,link_generation,command)
    control._send_effect=blocked
    control.test_channel(1,5,1,confirmed=True); time.sleep(.08)
    stopper=threading.Thread(target=control.stop); stopper.start(); time.sleep(.05)
    assert "PROPCAL SPIN STOP" in engine.connection.commands
    assert stopper.is_alive() and engine.owner=="mapping"
    gate.set(); stopper.join(1)


def test_reaper_after_close_releases_lease_without_overwriting_closed_state():
    engine=Engine(); control=MappingControl(engine); gate=threading.Event(); original=control._send_effect
    def blocked(generation,link_generation,command):
        if "SPIN SET" in command: gate.wait(2)
        return original(generation,link_generation,command)
    control._send_effect=blocked
    control.test_channel(1,5,1,confirmed=True); time.sleep(.08)
    closer=threading.Thread(target=control.close); closer.start(); time.sleep(.85)
    assert control.state=="closed" and engine.owner=="mapping"
    gate.set(); closer.join(1)
    deadline=time.monotonic()+1
    while engine.owner is not None and time.monotonic()<deadline: time.sleep(.01)
    assert engine.owner is None and control.state=="closed"


def test_disconnect_fails_and_releases_lease():
    engine=Engine(); engine.connection.is_connected=False; control=MappingControl(engine)
    assert control.query(); wait_state(control,"error")
    assert engine.owner is None


def test_reconnect_generation_terminates_query_and_releases_without_accepting_old_reply():
    engine=Engine(); control=MappingControl(engine)
    def no_reply(command): engine.connection.commands.append(command); return True
    engine.connection.send_command=no_reply
    assert control.query(); time.sleep(.03); engine.connection.generation+=1
    engine.propcal_events.put(LinkEvent("fc_text",4,time.monotonic(),"PROPCAL state=status calibrated=1"))
    wait_state(control,"error")
    assert engine.owner is None


def test_high_load_mapping_test_requires_confirmed_usb():
    engine=Engine(); engine.snapshot_period=1/20; control=MappingControl(engine)
    with pytest.raises(ValueError,match="超过数传预算"): control.test_channel(1,5,.2,confirmed=True)
    engine.connection.validate_snapshot_rate=lambda hz:{"confirmed_stm32_usb_cdc":True}
    assert control.test_channel(1,5,.05,confirmed=True); wait_state(control,"idle")


def test_rejected_arm_stops_and_waits_for_idle_before_release():
    engine=Engine(); engine.connection.reject_arm=True; control=MappingControl(engine)
    assert control.test_channel(1,5,.2,confirmed=True); wait_state(control,"error")
    assert engine.connection.commands[-1]=="PROPCAL SPIN STOP"
    assert engine.owner is None


@pytest.mark.parametrize("kind,target",(
    ("query","PROPCAL?"),
    ("arm","PROPCAL SPIN ARM"),
    ("set","PROPCAL SET ch=1"),
    ("commit","PROPCAL COMMIT"),
))
def test_stop_generation_barrier_prevents_old_worker_side_effect(kind,target):
    engine=Engine(); control=MappingControl(engine); entered=threading.Event(); release=threading.Event()
    original=control._send_effect
    def blocked(generation,link_generation,command):
        if command.startswith(target): entered.set(); release.wait(1)
        return original(generation,link_generation,command)
    control._send_effect=blocked
    if kind=="query": assert control.query()
    elif kind=="arm": assert control.test_channel(1,5,.2,confirmed=True)
    elif kind=="set": assert control.apply_mapping({1:{"role":"upper","spin":"cw"},2:{"role":"lower","spin":"ccw"}})
    else: assert control.save()
    assert entered.wait(.5)
    stopper=threading.Thread(target=control.stop); stopper.start(); time.sleep(.04); release.set(); stopper.join(1)
    assert not any(command.startswith(target) for command in engine.connection.commands)
    assert engine.owner is None and control.state=="idle"


def test_reconnect_before_effect_send_prevents_command_on_new_link():
    engine=Engine(); control=MappingControl(engine); entered=threading.Event(); release=threading.Event()
    original=control._send_effect
    def blocked(generation,link_generation,command):
        entered.set(); release.wait(.5)
        return original(generation,link_generation,command)
    control._send_effect=blocked
    assert control.query(); assert entered.wait(.3)
    engine.connection.generation+=1; release.set(); wait_state(control,"error")
    assert "PROPCAL?" not in engine.connection.commands and engine.owner is None
