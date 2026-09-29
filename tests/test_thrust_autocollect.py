from __future__ import annotations
import time
import threading
from collections import Counter
from dataclasses import replace
from types import SimpleNamespace
import pytest
from tools.thrust_bench.autocollect import AutoCollector
from tools.thrust_bench.acquisition import StopVoltageReached
from tools.thrust_bench.records import BenchSample, FcSnapshot
from tools.thrust_bench.sweep_schedule import (
    CHARGE_BAND_V, SAG_K_DEFAULT, ChargeEstimator, CoverageSchedule, GridCell,
    build_command_grid, load_index)


class Engine:
    def __init__(self,voltage=12.0,follow=True):
        self.owner=None;self.voltage=voltage;self.follow=follow;self.commands=[];self.stops=0;self.samples=[]
        self.arms=0
        self.upper=0.;self.lower=0.;self.snapshot_period=.05;self._plan_thread=None
        self._sample_index=0
        self.recorded_runs=[]
        self.store=SimpleNamespace(record_run=lambda record:self.recorded_runs.append(record))
    def claim_operation(self,owner):
        if self.owner:return False
        self.owner=owner;return True
    def release_operation(self,owner):
        if self.owner==owner:self.owner=None
    def arm(self,_max):self.arms+=1;return True
    def stop(self):self.stops+=1;return True
    def set_targets(self,upper,lower):
        self.upper,self.lower=upper,lower;self.commands.append((upper,lower));return True
    def latest_observation(self):
        eu=self.upper*1000 if self.follow else 1000
        el=self.lower*1000 if self.follow else 1000
        snap=FcSnapshot(1,1000,2,1,2,(1100,1100),(round(eu),round(el)),(0,0),self.voltage,None,0,None,
                        bench_active=True,bench_max_percent=20)
        return SimpleNamespace(snapshot=snap,scale_grams=100.0)
    def wait_for_adaptive_stability(self,*_args,**_kwargs):return True,[]
    def capture_latest_sample(self,run_id,segment_id,**_kwargs):
        self._sample_index+=1
        obs=self.latest_observation();sample=BenchSample(run_id,segment_id,time.monotonic(),fc_time_ms=1000+self._sample_index*50,
            scale_time_s=time.monotonic(),upper_command_pct=self.upper,lower_command_pct=self.lower,
            upper_erpm=obs.snapshot.erpm[0],lower_erpm=obs.snapshot.erpm[1],thrust_n=1,voltage_v=self.voltage,
            upper_erpm_age_ms=0,lower_erpm_age_ms=0)
        return sample
    def commit_steady_samples(self,samples):self.samples.extend(samples)


def wait_done(control,timeout=2):
    deadline=time.monotonic()+timeout
    while control.state not in {"finished","waiting_battery","waiting_recovery","error"} and time.monotonic()<deadline:time.sleep(.01)
    assert control.state in {"finished","waiting_battery","waiting_recovery","error"}


def test_requires_explicit_voltage_duration_points_and_speed_boundary():
    with pytest.raises(ValueError):AutoCollector(Engine(),[],max_percent=20,stop_voltage_v=0,max_duration_s=10,max_points=2)
    with pytest.raises(ValueError):AutoCollector(Engine(),[],max_percent=20,stop_voltage_v=10.8,max_duration_s=0,max_points=2)


def test_short_snapshot_gap_holds_target_then_resumes_same_run():
    class GappedEngine(Engine):
        TELEMETRY_GAP_GRACE_S=.8
        def __init__(self):
            super().__init__();self.gap_until=None
        def set_targets(self,upper,lower):
            result=super().set_targets(upper,lower)
            if self.gap_until is None:self.gap_until=time.monotonic()+.30
            return result
        def latest_observation(self):
            if self.gap_until is not None and time.monotonic()<self.gap_until:
                return SimpleNamespace(snapshot=None,scale_grams=None)
            return super().latest_observation()
    engine=GappedEngine()
    control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    assert control.start()
    deadline=time.monotonic()+1
    while engine.gap_until is None and time.monotonic()<deadline:time.sleep(.005)
    assert engine.gap_until is not None
    time.sleep(.10)
    assert engine.stops==0 and engine.commands==[(2.0,2.0)]
    wait_done(control,3)
    assert control.state=="finished"
    assert len(engine.samples)>=5
    assert engine.recorded_runs[0]["recorded_points"]==1


def test_persistent_snapshot_gap_stops_and_waits_for_recovery():
    class LostEngine(Engine):
        TELEMETRY_GAP_GRACE_S=.15
        def __init__(self):super().__init__();self.lost=False
        def set_targets(self,upper,lower):
            result=super().set_targets(upper,lower)
            self.lost=True
            return result
        def latest_observation(self):
            if self.lost:return SimpleNamespace(snapshot=None,scale_grams=None)
            return super().latest_observation()
    engine=LostEngine()
    control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    assert control.start();wait_done(control,3)
    assert control.state=="waiting_recovery"
    assert engine.stops>=1 and engine.owner is None
    assert engine.samples==[]


def test_low_voltage_stops_even_if_rpm_and_scale_are_missing():
    class LowVoltageEngine(Engine):
        def latest_observation(self):
            observation=super().latest_observation()
            observation.snapshot=replace(observation.snapshot,erpm=(None,None))
            observation.scale_grams=None
            return observation
    engine=LowVoltageEngine(voltage=10.7)
    control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    with pytest.raises(StopVoltageReached):control._voltage()


def test_partial_steady_samples_before_a_gap_are_not_committed():
    class MidSegmentGapEngine(Engine):
        def capture_latest_sample(self,*args,**kwargs):
            sample=super().capture_latest_sample(*args,**kwargs)
            if self._sample_index==3:raise RuntimeError("核心采样失效：voltage")
            return sample
    engine=MidSegmentGapEngine();engine.upper=5;engine.lower=6
    control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    control._run_id="same-run"
    result=control._record_stable_segment("segment",time.monotonic())
    assert result is not None and len(engine.samples)==5
    assert min(sample.fc_time_ms for sample in engine.samples)>=1200


def test_stop_voltage_waits_for_explicit_battery_continue():
    engine=Engine(voltage=10.7);control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=2,max_points=2)
    assert not control.start();wait_done(control)
    assert control.state=="waiting_battery" and engine.owner is None
    assert engine.arms==0
    assert not control.continue_after_battery_change(confirmed=False)
    engine.voltage=11.5
    assert control.continue_after_battery_change(confirmed=True)
    deadline=time.monotonic()+5
    while control._worker and control._worker.is_alive() and time.monotonic()<deadline:
        time.sleep(.01)
    assert control._worker is not None and not control._worker.is_alive()
    assert control.state=="finished"


def test_user_stop_never_auto_resumes_output():
    engine=Engine(follow=False);control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=5,max_points=3)
    control.start();time.sleep(.05);control.stop();count=len(engine.commands);time.sleep(.1)
    assert len(engine.commands)==count and engine.owner is None


def test_stop_keeps_lease_until_blocked_worker_exits():
    class DelayedEngine(Engine):
        def __init__(self):
            super().__init__();self.entered=threading.Event();self.release=threading.Event()
        def set_targets(self,upper,lower):
            self.entered.set();self.release.wait(3)
            if self.stops: return False
            return super().set_targets(upper,lower)
    engine=DelayedEngine()
    control=AutoCollector(engine,[],max_percent=20,
                          stop_voltage_v=10.8,max_duration_s=5,max_points=2)
    assert control.start() and engine.entered.wait(1)
    assert control.stop() is False
    assert engine.owner=="autocollect" and not engine.claim_operation("manual")
    engine.release.set();wait_done(control,2)
    assert engine.owner is None and engine.commands==[]


def test_low_voltage_during_partial_segment_does_not_commit_training_rows():
    class DepletingEngine(Engine):
        def capture_latest_sample(self,run_id,segment_id,**kw):
            sample=super().capture_latest_sample(run_id,segment_id,**kw)
            if self._sample_index==3:self.voltage=10.7
            return sample
    engine=DepletingEngine()
    control=AutoCollector(engine,[],max_percent=20,
                          stop_voltage_v=10.8,max_duration_s=3,max_points=1)
    assert control.start();wait_done(control,3)
    assert control.state=="waiting_battery"
    assert engine.samples==[]
    assert engine.owner is None and engine.stops>=1


def test_board_health_stop_is_not_masked_by_float_none():
    class LostStreamEngine(Engine):
        def __init__(self):
            super().__init__();self.cancel=threading.Event();self.fail_once=True
        def claim_operation(self,owner):
            self.cancel.clear();return super().claim_operation(owner)
        def set_targets(self,upper,lower):
            result=super().set_targets(upper,lower)
            if self.fail_once:
                self.fail_once=False
                self.cancel.set()  # Board/engine health monitor has already stopped.
            return result
        def latest_observation(self):
            observation=super().latest_observation()
            if self.cancel.is_set():
                observation.snapshot=replace(observation.snapshot,
                    erpm=(None,None),voltage_v=None)
            return observation
    engine=LostStreamEngine()
    control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    assert control.start();wait_done(control,3)
    messages=[]
    while not control.events.empty(): messages.append(control.events.get_nowait().data["message"])
    assert control.state=="waiting_recovery" and engine.owner is None
    assert any("数据流失效" in message for message in messages)
    assert all("float()" not in message and "NoneType" not in message for message in messages)
    assert not control.continue_after_recovery(confirmed=False)
    with pytest.raises(ValueError,match="恢复新鲜回读"):
        control.continue_after_recovery(confirmed=True)
    engine.cancel.clear()  # Simulate genuinely fresh FC replies after the STOP.
    assert control.continue_after_recovery(confirmed=True)
    deadline=time.monotonic()+4
    while control._worker and control._worker.is_alive() and time.monotonic()<deadline:
        time.sleep(.01)
    assert engine.arms==2 and engine.owner is None


def test_expired_voltage_alone_pauses_and_can_resume_after_fresh_reply():
    class VoltageGapEngine(Engine):
        def __init__(self):super().__init__();self.drop_once=True;self.drop=False
        def set_targets(self,upper,lower):
            result=super().set_targets(upper,lower)
            if self.drop_once:
                self.drop_once=False;self.drop=True
            return result
        def latest_observation(self):
            observation=super().latest_observation()
            if self.drop:
                observation.snapshot=replace(observation.snapshot,voltage_v=None)
            return observation
    engine=VoltageGapEngine()
    control=AutoCollector(engine,[],max_percent=20,
        stop_voltage_v=10.8,max_duration_s=1,max_points=1)
    assert control.start();wait_done(control,3)
    assert control.state=="waiting_recovery" and engine.owner is None
    assert not control.continue_after_recovery(confirmed=False)
    with pytest.raises(ValueError,match="恢复新鲜回读"):
        control.continue_after_recovery(confirmed=True)
    engine.drop=False
    assert control.continue_after_recovery(confirmed=True)
    deadline=time.monotonic()+3
    while control._worker and control._worker.is_alive() and time.monotonic()<deadline:
        time.sleep(.01)
    assert control._worker is not None and not control._worker.is_alive()
    assert engine.arms==2 and engine.owner is None


def test_reached_erpm_is_recorded_without_chasing_a_target():
    # 2026-09-23 bench run: 11 of 16 settled points were dropped for missing an eRPM target.
    engine=Engine(follow=False)
    control=AutoCollector(engine,[],max_percent=20,stop_voltage_v=10.8,max_duration_s=3,max_points=3)
    assert control.start();wait_done(control,3)
    run=engine.recorded_runs[0]
    assert run["recorded_points"]==3 and run["filled_gap_points"]==3
    assert {(s.upper_erpm,s.lower_erpm) for s in engine.samples}=={(1000,1000)}
    assert len({s.segment_id for s in engine.samples})==3
    assert all(command in control.targets for command in engine.commands)


def test_complete_charge_band_keeps_measuring_instead_of_holding():
    grid=AutoCollector(Engine(),[],max_percent=20,stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    covered=[{"upper_command_pct":u,"lower_command_pct":l,"upper_erpm":u*1000,"lower_erpm":l*1000,
              "voltage_v":12.0-grid.estimator.k*((u/10)**3+(l/10)**3)} for u,l in grid.targets]
    engine=Engine()
    control=AutoCollector(engine,covered,max_percent=20,stop_voltage_v=10.8,max_duration_s=3,max_points=4)
    assert control.coverage_gap(12.0)["missing"]==0
    started=time.monotonic();assert control.start();wait_done(control,3)
    assert time.monotonic()-started<2
    run=engine.recorded_runs[0]
    assert run["recorded_points"]==4 and run["filled_gap_points"]==0 and run["stop_reason"]=="max_points"
    messages=[]
    while not control.events.empty():messages.append(control.events.get_nowait().data)
    assert not any(event["state"]=="discharging" for event in messages)
    assert "其中0个补在此前缺失的格子" in messages[-1]["message"]


def test_command_steps_are_bounded_and_stay_under_limit():
    engine=Engine()
    control=AutoCollector(engine,[],max_percent=100,stop_voltage_v=10.8,max_duration_s=3,max_points=8)
    assert control.start();wait_done(control,3)
    previous=(0.0,0.0)
    for command in engine.commands:
        assert max(abs(a-b) for a,b in zip(command,previous))<=10.0+1e-9
        assert max(command)<=100
        previous=command
    assert engine.recorded_runs[0]["command_grid_pct"]=={"min":2.0,"max":100.0,"step":6.125,"cells":289}


def test_drifting_record_window_drops_its_unsteady_head():
    class DriftingEngine(Engine):
        def capture_latest_sample(self,*args,**kwargs):
            sample=super().capture_latest_sample(*args,**kwargs)
            return replace(sample,thrust_n=3.0 if self._sample_index<=2 else 1.0)
    engine=DriftingEngine();engine.upper=5;engine.lower=6
    control=AutoCollector(engine,[],max_percent=20,stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    control._run_id="drift"
    result=control._record_stable_segment("segment",time.monotonic())
    assert result is not None and len(engine.samples)==5
    assert {sample.thrust_n for sample in engine.samples}=={1.0}


def point(upper_pct, lower_pct, voltage, upper_erpm=None, lower_erpm=None):
    return {"upper_command_pct": upper_pct, "lower_command_pct": lower_pct, "voltage_v": voltage,
            "upper_erpm": upper_erpm or upper_pct * 900, "lower_erpm": lower_erpm or lower_pct * 900}


def at_charge(upper_pct, lower_pct, charge):
    """A point whose loaded voltage already includes the default sag at its load."""
    sag = SAG_K_DEFAULT * load_index(upper_pct * 900, lower_pct * 900)
    return point(upper_pct, lower_pct, charge - sag)


@pytest.mark.parametrize("maximum,count,step,tiers", [
    (20, 25, 4.5, {0: 25}),
    (50, 81, 6.0, {0: 25, 1: 56}),
    (100, 289, 6.125, {0: 25, 1: 56, 2: 208}),
])
def test_command_grid_is_nested_coarse_to_fine(maximum, count, step, tiers):
    cells, grid_step = build_command_grid(maximum)
    assert len(cells) == count and grid_step == pytest.approx(step)
    assert Counter(cell.tier for cell in cells) == tiers
    corners = {(2.0, 2.0), (2.0, float(maximum)), (float(maximum), 2.0), (float(maximum), float(maximum))}
    assert corners <= {(cell.upper_pct, cell.lower_pct) for cell in cells if cell.tier == 0}
    assert max(max(cell.upper_pct, cell.lower_pct) for cell in cells) == maximum


def test_missing_coarse_cells_come_first_then_nearest_move():
    schedule = CoverageSchedule(50, [at_charge(2, 2, 12.2)])
    first = schedule.next_cell(12.2, (0.0, 0.0))
    assert first.tier == 0 and (first.upper_pct, first.lower_pct) in {(2.0, 14.0), (14.0, 2.0)}
    for cell in schedule.cells:
        if cell.tier == 0:
            schedule.add_point(at_charge(cell.upper_pct, cell.lower_pct, 12.2))
    finer = schedule.next_cell(12.2, (50.0, 50.0))
    assert finer.tier == 1 and max(50.0 - finer.upper_pct, 50.0 - finer.lower_pct) == schedule.step


def test_load_compensation_keeps_same_charge_cells_covered():
    charge = 12.2
    high_load = load_index(40000, 40000)
    schedule = CoverageSchedule(50, [
        point(2, 2, charge - SAG_K_DEFAULT * load_index(2000, 2000), 2000, 2000),
        point(50, 50, charge - SAG_K_DEFAULT * high_load, 40000, 40000)])
    assert SAG_K_DEFAULT * high_load > CHARGE_BAND_V  # Raw loaded V alone would re-open one of them.
    assert schedule.covered(GridCell(2.0, 2.0, 0), charge)
    assert schedule.covered(GridCell(50.0, 50.0, 0), charge)
    assert not schedule.covered(GridCell(50.0, 50.0, 0), charge - CHARGE_BAND_V)


def test_complete_band_still_returns_a_cell_instead_of_idling():
    schedule = CoverageSchedule(20)
    for cell in schedule.cells:
        charge = 12.10 if (cell.upper_pct, cell.lower_pct) == (20.0, 20.0) else 12.20
        schedule.add_point(at_charge(cell.upper_pct, cell.lower_pct, charge))
    assert schedule.missing_count(12.2) == 0
    assert schedule.next_cell(12.2, (2.0, 2.0)) == GridCell(20.0, 20.0, 0)


def test_failed_cell_waits_for_the_next_charge_band():
    schedule = CoverageSchedule(20)
    corner = schedule.next_cell(12.3, (0.0, 0.0))
    schedule.mark_failed(corner, 12.3)
    assert schedule.next_cell(12.3, (0.0, 0.0)) != corner
    assert schedule.next_cell(12.3 - CHARGE_BAND_V, (0.0, 0.0)) == corner


def test_points_outside_grid_or_not_spinning_are_ignored():
    schedule = CoverageSchedule(20)
    assert not schedule.add_point(point(60, 60, 12.2))
    assert not schedule.add_point({**point(2, 2, 12.2), "upper_erpm": 0.0})
    assert not schedule.add_point({"voltage_v": 12.2})
    assert schedule.missing_count(12.2) == len(schedule.cells)


def test_charge_estimator_refits_sag_from_this_battery():
    estimator = ChargeEstimator()
    for index in range(12):
        upper = 5000 + 3500 * index
        load = load_index(upper, upper)
        estimator.observe(index * 1.5, 12.3 - 0.001 * index * 1.5 - 0.005 * load, upper, upper)
    assert estimator.k == pytest.approx(0.005, rel=1e-3)
    flat = ChargeEstimator()
    for index in range(12):
        flat.observe(index, 12.2, 10000, 10000)
    assert flat.k == SAG_K_DEFAULT  # No load spread: keep the recorded default.


def max_thrust_messages(control):
    messages=[]
    while not control.events.empty():messages.append(control.events.get_nowait().data)
    return messages


def test_max_thrust_ramps_to_armed_limit_records_steady_point_and_stops():
    from tools.thrust_bench.max_thrust import MaxThrustTest
    engine=Engine()
    control=MaxThrustTest(engine,max_percent=100,stop_voltage_v=10.8)
    assert control.start();wait_done(control,3)
    previous=(0.0,0.0)
    for command in engine.commands:
        assert max(abs(a-b) for a,b in zip(command,previous))<=10.0+1e-9 and max(command)<=100
        previous=command
    assert previous==(100.0,100.0)
    assert len(engine.samples)>=5 and all(s.upper_command_pct==100 and s.lower_command_pct==100 for s in engine.samples)
    result=control.result
    assert result["steady"] and result["thrust_gf"]==pytest.approx(1000/9.80665)
    assert result["upper_erpm"]==100000 and result["loaded_voltage_v"]==12.0
    run=engine.recorded_runs[0]
    assert run["plan_name"]=="max_thrust_test" and run["recorded_points"]==1 and run["max_thrust"]==result
    assert engine.stops>=1 and engine.owner is None and control.state=="finished"
    assert "最大推力（稳态平均） 102 克" in max_thrust_messages(control)[-1]["message"]


def test_max_thrust_never_exceeds_the_typed_limit():
    from tools.thrust_bench.max_thrust import MaxThrustTest
    engine=Engine()
    control=MaxThrustTest(engine,max_percent=60,stop_voltage_v=10.8)
    assert control.start();wait_done(control,3)
    assert max(max(command) for command in engine.commands)==60


def test_unsettled_max_thrust_is_reported_but_not_committed():
    from tools.thrust_bench.max_thrust import MaxThrustTest
    class NeverSteadyEngine(Engine):
        def wait_for_adaptive_stability(self,*_args,**_kwargs):return False,[]
    engine=NeverSteadyEngine()
    control=MaxThrustTest(engine,max_percent=100,stop_voltage_v=10.8)
    assert control.start();wait_done(control,4)
    assert engine.samples==[] and engine.recorded_runs[0]["recorded_points"]==0
    assert control.result["steady"] is False and control.result["thrust_gf"]==pytest.approx(100.0)
    assert "未达稳态" in max_thrust_messages(control)[-1]["message"] and engine.owner is None


class SagBench(Engine):
    """Loaded voltage = charge minus the default sag; 100 % reaches 75k eRPM like the real rotors."""
    def __init__(self,charge):
        super().__init__();self.charge=charge
    def latest_observation(self):
        observation=super().latest_observation()
        erpm=tuple(round(value*.75) for value in observation.snapshot.erpm)
        self.voltage=self.charge-SAG_K_DEFAULT*load_index(*erpm)
        observation.snapshot=replace(observation.snapshot,erpm=erpm,voltage_v=self.voltage)
        return observation


def test_normal_load_sag_on_a_full_pack_does_not_trigger_battery_change():
    """2026-09-23: 80 % sagged a 12.3 V pack to 10.7 V and a 10.8 V threshold ended the run."""
    from tools.thrust_bench.max_thrust import MaxThrustTest
    engine=SagBench(12.3)
    control=MaxThrustTest(engine,max_percent=100,stop_voltage_v=10.8)
    assert control.start();wait_done(control,3)
    assert control.state=="finished" and control.result["steady"]
    assert control.result["loaded_voltage_v"]<10.0 and max(max(c) for c in engine.commands)==100


def test_max_thrust_stops_when_loaded_voltage_drops_below_cell_floor():
    from tools.thrust_bench.max_thrust import MaxThrustTest
    engine=SagBench(11.9)  # 100 % would sag it to 8.95 V
    control=MaxThrustTest(engine,max_percent=100,stop_voltage_v=10.8)
    assert control.start();wait_done(control,3)
    assert control.result is None and control.state=="error"
    assert "跌破每节 3.0 V" in max_thrust_messages(control)[-1]["message"]
    assert engine.stops>=1 and engine.owner is None


def test_low_charge_stops_even_at_light_load():
    class DrainedEngine(Engine):
        def set_targets(self,upper,lower):
            if upper>=10:self.voltage=10.7
            return super().set_targets(upper,lower)
    from tools.thrust_bench.max_thrust import MaxThrustTest
    engine=DrainedEngine(follow=False)  # 1000 eRPM: no sag to add back
    control=MaxThrustTest(engine,max_percent=100,stop_voltage_v=10.8)
    assert control.start();wait_done(control,3)
    assert control.result is None and "电量电压约 10.70 V" in max_thrust_messages(control)[-1]["message"]
    assert max(max(command) for command in engine.commands)<=20


def test_auto_zero_only_tares_with_both_rotors_stopped(monkeypatch):
    import tools.thrust_bench.autocollect as ac
    monkeypatch.setattr(ac,"AUTO_ZERO_WAIT_S",.3);monkeypatch.setattr(ac,"AUTO_ZERO_STILL_S",.05)
    class ZeroEngine(Engine):
        def __init__(self,spinning):
            super().__init__();self.load_cell=SimpleNamespace(tare=lambda samples:self.tares.append(samples) or 4321.0)
            self.tares=[];self.upper=self.lower=5.0 if spinning else 0.0;self.meta={}
            self.store=SimpleNamespace(record_run=lambda r:None,update_metadata=lambda **kw:self.meta.update(kw),event=lambda *a,**k:None)
    still=ZeroEngine(False);control=AutoCollector(still,[],max_percent=20,stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    assert control._zero_scale()==4321.0 and still.tares==[8] and still.meta=={"scale_tare_raw":4321.0}
    spinning=ZeroEngine(True);control=AutoCollector(spinning,[],max_percent=20,stop_voltage_v=10.8,max_duration_s=2,max_points=1)
    assert control._zero_scale() is None and spinning.tares==[]


def toy_model(tmp_path):
    import json as _json
    from tools.thrust_bench import throttle_model as tm
    points=[{"upper_command_pct":c,"lower_command_pct":c2,"charge_v":v,"thrust_gf":400*(c/100*v/12+c2/100*v/12)**2,
             "session":"s","time_s":float(i)}
            for i,(c,c2,v) in enumerate((a,b,v) for a in range(2,101,7) for b in range(2,101,7) for v in (11.5,12.1))]
    model=tm.fit(points);path=tmp_path/"throttle_model.json";path.write_text(_json.dumps(model),encoding="utf-8")
    return model,path


def test_model_validation_reaches_high_targets_through_normal_sag(tmp_path):
    from tools.thrust_bench.model_validation import ModelValidationTest,TARGET_FRACTIONS
    model,path=toy_model(tmp_path)
    engine=SagBench(12.0)  # 80 % loads it to ~10.5 V, 95 % to ~9.5 V
    control=ModelValidationTest(engine,model,path,stop_voltage_v=10.8)
    assert control.start();wait_done(control,6)
    run=engine.recorded_runs[0]
    assert run["stop_reason"]=="completed" and run["summary"]["measured"]==len(TARGET_FRACTIONS)
    assert max(max(command) for command in engine.commands)>90


def test_closing_a_finished_run_does_not_stop_the_next_one(tmp_path):
    """The UI closed the old controller after starting the new one; its STOP killed every retry."""
    from tools.thrust_bench.model_validation import ModelValidationTest,TARGET_FRACTIONS
    class CancelEngine(Engine):
        def __init__(self):
            super().__init__(voltage=11.8,follow=False);self.cancel=threading.Event()
        def claim_operation(self,owner):
            claimed=super().claim_operation(owner)
            if claimed:self.cancel.clear()
            return claimed
        def stop(self):
            self.cancel.set();return super().stop()
    model,path=toy_model(tmp_path);engine=CancelEngine()
    first=ModelValidationTest(engine,model,path,stop_voltage_v=10.8)
    assert first.start();wait_done(first,6)
    second=ModelValidationTest(engine,model,path,stop_voltage_v=10.8)
    assert second.start();first.close();wait_done(second,6)
    assert engine.recorded_runs[-1]["summary"]["measured"]==len(TARGET_FRACTIONS)


def test_lookup_table_validation_also_checks_the_speed_table(tmp_path):
    from tools.thrust_bench import thrust_lut
    from tools.thrust_bench.model_validation import ModelValidationTest,TARGET_FRACTIONS
    points=[{"upper_command_pct":a,"lower_command_pct":b,"charge_v":v,"upper_erpm":60*a*v,"lower_erpm":60*b*v,
             "thrust_gf":16*((6e-3*a*v)**2+(6e-3*b*v)**2),"session":"s","time_s":float(i)}
            for i,(a,b,v) in enumerate((a,b,v) for a in range(2,101,7) for b in range(2,101,7) for v in (11.5,12.1))]
    table=thrust_lut.build(points);path=tmp_path/"thrust_lut.json"
    engine=SagBench(12.0)  # 750 eRPM per %: inside the table's measured speed range
    control=ModelValidationTest(engine,table,path,stop_voltage_v=10.8,backend=thrust_lut)
    assert control.start();wait_done(control,8)
    run=engine.recorded_runs[0]
    assert run["model_schema"]=="thrust_lut_v1" and run["summary"]["measured"]==len(TARGET_FRACTIONS)
    assert run["summary"]["speed_checked"]>0 and all("speed_table_gf" in entry for entry in run["targets"])
    messages=[]
    while not control.events.empty():messages.append(control.events.get_nowait().data)
    assert "按实测转速查转速表" in messages[-1]["message"]
    assert messages[-1]["report"].endswith(f"validation-{run['run_id']}.html")
    assert (tmp_path/f"validation-{run['run_id']}.png").is_file()


def test_split_validation_runs_old_and_new_allocation_back_to_back(tmp_path):
    from tools.thrust_bench import thrust_lut
    from tools.thrust_bench.model_validation import SplitValidationTest,SPLIT_ORDER
    x=lambda pct,v:6e-3*pct*v  # interacting rotors: the per-rotor sum misses the pair total
    points=[{"upper_command_pct":a,"lower_command_pct":b,"charge_v":v,"upper_erpm":60*a*v,"lower_erpm":60*b*v,
             "thrust_gf":16*x(a,v)**2+20*x(b,v)**2-6*x(a,v)*x(b,v),"session":"s","time_s":float(i)}
            for i,(a,b,v) in enumerate((a,b,v) for a in range(2,101,7) for b in range(2,101,7) for v in (11.5,12.1))]
    table=thrust_lut.build(points);path=tmp_path/"thrust_lut.json"
    engine=SagBench(12.0)
    control=SplitValidationTest(engine,table,path,stop_voltage_v=10.8)
    assert control.start();wait_done(control,15)
    run=engine.recorded_runs[0];targets=run["targets"]
    assert run["plan_name"]=="split_validation" and len(targets)==2*len(SPLIT_ORDER)
    assert [t["allocation"] for t in targets[:4]]==["new","old","old","new"]  # neither always first
    for new,old in zip(*[iter(sorted(targets,key=lambda t:(t["target_gf"],t["split"],t["allocation"])))]*2):
        assert new["target_gf"]==old["target_gf"] and new["split"]==old["split"]
        assert new["upper_pct"]-new["lower_pct"]==pytest.approx(old["upper_pct"]-old["lower_pct"],abs=0.02)
    assert any(abs(t.get("shift_pct",0))>0.1 for t in targets if t["allocation"]=="new")
    assert run["summary"]["new"]["measured"]==run["summary"]["old"]["measured"]==len(SPLIT_ORDER)
    assert (tmp_path/f"validation-{run['run_id']}.html").is_file() and "tools.thrust_bench" not in str(path)
    messages=[]
    while not control.events.empty():messages.append(control.events.get_nowait().data)
    assert "差速实测验证" in messages[-1]["message"] and messages[-1]["report"].endswith(".html")


def test_model_validation_inverts_commands_measures_and_writes_an_independent_record(tmp_path):
    import json as _json
    from tools.thrust_bench import throttle_model as tm
    from tools.thrust_bench.model_validation import ModelValidationTest,TARGET_FRACTIONS
    model,path=toy_model(tmp_path)
    engine=Engine(voltage=11.8,follow=False)  # fixed eRPM: this fake bench never sags its voltage
    control=ModelValidationTest(engine,model,path,stop_voltage_v=10.8)
    assert control.start();wait_done(control,6)
    run=engine.recorded_runs[0]
    assert run["plan_name"]=="model_validation" and run["model_id"]==model["model_id"]
    assert len(run["targets"])==len(TARGET_FRACTIONS) and run["summary"]["measured"]==len(TARGET_FRACTIONS)
    for entry in run["targets"]:
        assert tm.predict_gf(model,entry["command_pct"],entry["command_pct"],11.8)==pytest.approx(entry["target_gf"],abs=2)
        assert entry["measured_gf"]==pytest.approx(1000/9.80665)  # fake bench always measures 1 N
    assert max(max(command) for command in engine.commands)<=100 and engine.owner is None
    saved=_json.loads((tmp_path/f"validation-{run['run_id']}.json").read_text(encoding="utf-8"))
    assert saved["summary"]==run["summary"]
    messages=[]
    while not control.events.empty():messages.append(control.events.get_nowait().data)
    assert "实测验证" in messages[-1]["message"] and "超出 50 克" in messages[-1]["message"]
