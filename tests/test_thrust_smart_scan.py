from __future__ import annotations

import pytest
import time
from dataclasses import replace

from tools.thrust_bench.acquisition import AcquisitionEngine, StopVoltageReached
from tools.thrust_bench.plans import PlanPoint, common_sweep
from tools.thrust_bench.records import BenchSample
from tools.thrust_bench.smart_scan import build_smart_plan
from test_thrust_bench_acquisition import Connection, Store, snapshot


def commands(plan):
    return [(point.upper_percent,point.lower_percent) for point in plan.points]


def test_quick_plan_has_bounded_same_and_differential_points():
    plan=build_smart_plan("quick",20)
    assert len(plan.points)==5 and plan.max_percent<=20
    assert any(u==l for u,l in commands(plan))
    assert any(u!=l for u,l in commands(plan))
    assert all(point.adaptive and point.max_wait_s<=4 for point in plan.points)


def test_standard_is_finite_interleaved_two_dimensional_plan():
    plan=build_smart_plan("standard",30)
    assert len(plan.points)==16 and len(set(commands(plan)))==16
    assert all(0<=u<=30 and 0<=l<=30 for u,l in commands(plan))
    assert commands(plan)[:4]==[(7.5,7.5),(15,15),(22.5,22.5),(30,30)]
    assert set(commands(plan))==set((u,l) for u in (7.5,15,22.5,30)
                                   for l in (7.5,15,22.5,30))


def test_supplement_requires_command_and_loaded_voltage_coverage():
    target=build_smart_plan("standard",20).points[0]
    old_voltage=[{"upper_command_pct":target.upper_percent,
                  "lower_command_pct":target.lower_percent,
                  "voltage_v":10.0}]
    different_voltage=build_smart_plan(
        "supplement",20,old_voltage,current_voltage_v=12.0)
    assert different_voltage.reference_voltage_v==12.0
    assert different_voltage.voltage_tolerance_v==pytest.approx(.15)
    assert (target.upper_percent,target.lower_percent) in commands(different_voltage)
    same_voltage=build_smart_plan(
        "supplement",20,old_voltage,current_voltage_v=10.1)
    assert (target.upper_percent,target.lower_percent) not in commands(same_voltage)


def test_supplement_missing_coverage_or_voltage_falls_back_to_standard_grid():
    standard=commands(build_smart_plan("standard",20))
    assert commands(build_smart_plan("supplement",20,(),current_voltage_v=12))==standard
    assert commands(build_smart_plan("supplement",20,[{"upper_command_pct":5,"lower_command_pct":5,"voltage_v":12}],current_voltage_v=None))==standard


def test_fully_covered_supplement_still_rechecks_one_real_point():
    standard=build_smart_plan("standard",20)
    covered=[{"upper_command_pct":u,"lower_command_pct":l,"voltage_v":12}
             for u,l in commands(standard)]
    supplement=build_smart_plan("supplement",20,covered,current_voltage_v=12)
    assert commands(supplement)==[(10,10)]


def test_old_plan_points_keep_nonadaptive_behavior():
    point=common_sweep(minimum=0,maximum=10,step=10).points[0]
    assert not point.adaptive and point.max_wait_s==0 and point.stable_window_s==0


def test_old_model_excludes_adaptive_settling_phase():
    from tools.thrust_bench.model import _steady_points
    settling=replace(sample(0.0,1000),phase="settle",
                     quality=("adaptive_settling",))
    points,reasons=_steady_points([settling],{})
    assert points==[] and reasons=={}


def sample(host,fc_ms,upper=14000,lower=14000,thrust=1.0):
    return BenchSample(
        run_id="run",segment_id="smart",host_time_s=host,fc_time_ms=fc_ms,
        scale_time_s=host,mode="dual",phase="steady",
        upper_command_pct=10,lower_command_pct=10,
        upper_erpm=upper,lower_erpm=lower,thrust_n=thrust,
        voltage_v=12,upper_erpm_age_ms=0,lower_erpm_age_ms=0)


def test_stability_window_requires_independent_time_span_and_low_variation():
    times=(0.0,.067,.145,.212,.290,.357,.435)
    stable=[((1000+int(t*1000),1000+int(t*1000)),sample(t,1000+int(t*1000),14000+i*5,14100-i*5,1+i*.002)) for i,t in enumerate(times)]
    assert AcquisitionEngine._adaptive_window_stable(stable,.4)
    unstable=list(stable); unstable[-1]=(unstable[-1][0],sample(times[-1],1435,17000,14100,1))
    assert not AcquisitionEngine._adaptive_window_stable(unstable,.4)
    repeated_scale=[(source,replace(item,scale_time_s=0.0)) for source,item in stable]
    assert not AcquisitionEngine._adaptive_window_stable(repeated_scale,.4)


def test_adaptive_wait_saves_settling_samples_and_reports_timeout_skip():
    clock=[0.0]; store=Store(); engine=AcquisitionEngine(
        Connection(),None,store,clock=lambda:clock[0]); engine.snapshot_period=.001
    count=[0]
    def changing(*_args):
        count[0]+=1; clock[0]+=.1
        return sample(clock[0],1000+count[0]*100,
                      upper=10000+count[0]*1000,lower=12000,
                      thrust=1+count[0]*.1)
    engine._joined_sample=changing
    point=PlanPoint("smart-timeout",10,10,adaptive=True,
                    max_wait_s=.35,stable_window_s=.2)
    stable,voltages=engine._await_adaptive_stability("run","dual",point)
    assert not stable and voltages
    assert store.samples and all("adaptive_settling" in item.quality for item in store.samples)
    assert any(kind=="adaptive_point_skipped" and data["reason"]=="等待稳定超时" for kind,data in store.events)
    skipped=[event for event in list(engine.ui_events.queue) if event.kind=="point_skipped"]
    assert skipped and "message" in skipped[0].data
    assert all(item.phase=="settle" for item in store.samples)


def test_adaptive_wait_reports_stable_and_reserves_latest_source_for_steady_data():
    clock=[0.0]; store=Store(); engine=AcquisitionEngine(
        Connection(),None,store,clock=lambda:clock[0]); engine.snapshot_period=.001
    count=[0]
    def steady(*_args):
        count[0]+=1; clock[0]+=.1
        return sample(clock[0],1000+count[0]*100,
                      upper=14000+count[0],lower=14100-count[0],
                      thrust=1+count[0]*.001)
    engine._joined_sample=steady
    point=PlanPoint("smart-stable",10,10,adaptive=True,
                    max_wait_s=1,stable_window_s=.3)
    stable,_voltages=engine._await_adaptive_stability("run","dual",point)
    assert stable and engine._last_emitted_source_times is not None
    stable_events=[event for event in list(engine.ui_events.queue) if event.kind=="point_stable"]
    assert stable_events and stable_events[0].data["message"]=="转速与推力已稳定"


def test_adaptive_wait_core_data_loss_skips_point_without_stopping_window():
    clock=[0.0]; engine=AcquisitionEngine(
        Connection(),None,Store(),clock=lambda:clock[0]); engine.snapshot_period=.001
    def invalid(*_args):
        clock[0]+=.1
        return sample(clock[0],1000,lower=None)
    engine._joined_sample=invalid
    point=PlanPoint("smart-invalid",10,10,adaptive=True,
                    max_wait_s=.3,stable_window_s=.2)
    stable,_=engine._await_adaptive_stability("run","dual",point)
    assert not stable and not engine.cancel.is_set()


def test_adaptive_wait_checks_low_voltage_each_observation_cycle():
    clock=[0.0]; engine=AcquisitionEngine(
        Connection(),None,Store(),clock=lambda:clock[0]); engine.snapshot_period=.001
    def low_voltage(*_args):
        clock[0]+=.1
        return replace(sample(clock[0],1000),voltage_v=10.7)
    engine._joined_sample=low_voltage
    point=PlanPoint("smart-low",10,10,adaptive=True,
                    max_wait_s=4,stable_window_s=.35,min_scale_updates=5)
    with pytest.raises(StopVoltageReached):
        engine._await_adaptive_stability(
            "run","dual",point,stop_voltage_v=10.8)
    assert clock[0]==pytest.approx(.1)


def test_smart_plan_must_be_repreviewed_after_loaded_voltage_moves():
    connection=Connection(); engine=AcquisitionEngine(connection,None,Store())
    fc=replace(snapshot(),voltage_v=11.0,voltage_age_ms=0)
    engine._latest_fc=(fc,time.monotonic(),2,1000)
    plan=build_smart_plan("supplement",20,(),current_voltage_v=12.0)
    with pytest.raises(RuntimeError,match="重新预览"):
        engine.run_plan(plan,max_percent=20)
    assert engine.active_operation is None


def test_one_unchanged_erpm_source_waits_but_strict_regression_fails():
    assert AcquisitionEngine._source_progress((101,100),(100,100))=="waiting"
    assert AcquisitionEngine._source_progress((101,99),(100,100))=="regression"
    assert AcquisitionEngine._source_progress((101,101),(100,100))=="new"
