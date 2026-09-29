"""Automatic dataset collection: open-loop throttle-grid sweep on one Engine lease.

Every steady cell is recorded at whatever eRPM it reaches; the battery drains
through the measurements themselves instead of an idle discharge hold.
"""
from __future__ import annotations
import math, queue, statistics, threading, time
from collections import deque
from dataclasses import replace
from uuid import uuid4
from .acquisition import StopVoltageReached, UiEvent
from .plans import PlanPoint
from .sweep_schedule import CHARGE_BAND_V, ChargeEstimator, CoverageSchedule

AUTO_COMMAND_SLEW_PCT=10.0
AUTO_POINT_MAX_WAIT_S=3.0
AUTO_RECORD_WINDOW_S=1.5
AUTO_MAX_POINTS=5000
AUTO_MAX_DURATION_S=3600.0
AUTO_ZERO_WAIT_S=5.0
AUTO_ZERO_STILL_S=0.5
AUTO_ZERO_STOPPED_ERPM=100.0
# The battery-change threshold is a state of charge: an 80 % step sags a full 3S pack to ~10.7 V,
# so it is judged with the load sag added back. The loaded floor guards the cells directly.
LOADED_FLOOR_V=9.0  # 3S x 3.0 V/cell; lowest loaded reading in the 423-point dataset was 9.36 V


class SourceUnavailable(RuntimeError):
    """Fresh measurement did not return within the bounded telemetry grace."""



class AutoCollector:
    OWNER="autocollect"
    def __init__(self,engine,covered_points,*,max_percent,stop_voltage_v,max_duration_s,max_points,
                 events=None,clock=time.monotonic):
        self.engine=engine; self.max_percent=float(max_percent)
        self.stop_voltage_v=float(stop_voltage_v); self.max_duration_s=float(max_duration_s); self.max_points=int(max_points)
        self.clock=clock; self.events=events or queue.Queue(); self._state="idle"; self._lock=threading.RLock()
        self._cancel=threading.Event(); self._worker=None; self._closed=False; self._run_id=None
        self._recorded_total=0; self._elapsed_total=0.0
        self._charge_window=deque(maxlen=7)
        self._battery_window=deque(maxlen=3)
        self._last_command=(0.0,0.0)
        self.last_segment_samples=[]
        if not 1<=self.max_percent<=100: raise ValueError("max_percent必须在1..100")
        if not math.isfinite(self.stop_voltage_v) or self.stop_voltage_v<=0: raise ValueError("必须明确填写停止电压")
        if not 1<=self.max_points<=AUTO_MAX_POINTS or not 1<=self.max_duration_s<=AUTO_MAX_DURATION_S:
            raise ValueError("采集点数或总时长超出边界")
        self.estimator=ChargeEstimator()
        self.schedule=CoverageSchedule(self.max_percent,covered_points or (),self.estimator)
    @property
    def targets(self):
        return [(cell.upper_pct,cell.lower_pct) for cell in self.schedule.cells]
    def coverage_gap(self,voltage_v,upper_erpm=0.0,lower_erpm=0.0):
        """Grid cells still missing near this state of charge (library display)."""
        charge=self.estimator.compensated(voltage_v,upper_erpm or 0.0,lower_erpm or 0.0)
        return {"charge_v":charge,"total":len(self.schedule.cells),"missing":self.schedule.missing_count(charge)}
    @property
    def state(self):
        with self._lock:return self._state
    @property
    def busy(self):return self.state not in {"idle","finished","waiting_battery","waiting_recovery","error","closed"}
    def _publish(self,state,message,**data):
        with self._lock:
            if self._closed:return
            self._state=state
        self.events.put(UiEvent("autocollect_state",{"state":state,"message":message,**data}))
    def start(self):
        with self._lock:
            if self._closed or self.busy or (self._worker and self._worker.is_alive()):return False
            observation=self.engine.latest_observation()
            voltage=observation.snapshot.voltage_v if observation.snapshot is not None else None
            if voltage is None:raise ValueError("飞控电压尚未恢复新鲜回读")
            if voltage<=self.stop_voltage_v+0.3:
                self._publish("waiting_battery","当前电压接近换电阈值，请先换电池")
                return False
            if not self.engine.claim_operation(self.OWNER):self._publish("error","其他台架操作正在运行");return False
            self._cancel.clear();self._run_id=uuid4().hex[:12]
            self._charge_window.clear();self._battery_window.clear()
            self._worker=threading.Thread(target=self._run,daemon=True,name="tbench-autocollect");self._worker.start();return True
    def continue_after_battery_change(self,*,confirmed=False):
        if not confirmed or self.state!="waiting_battery":return False
        worker=self._worker
        if worker and worker is not threading.current_thread() and worker.is_alive():
            worker.join(timeout=.3)
            if worker.is_alive():return False
        observation=self.engine.latest_observation(); voltage=observation.snapshot.voltage_v if observation.snapshot else None
        if voltage is None or voltage<=self.stop_voltage_v+0.3:raise ValueError("未确认换入电压更高的电池")
        self._elapsed_total=0.0;self._recorded_total=0
        return self.start()
    def continue_after_recovery(self,*,confirmed=False):
        """Explicitly resume a fresh run after the board has safely stopped."""
        if not confirmed or self.state!="waiting_recovery":return False
        worker=self._worker
        if worker and worker is not threading.current_thread() and worker.is_alive():
            worker.join(timeout=.3)
            if worker.is_alive():return False
        observation=self.engine.latest_observation()
        snapshot=observation.snapshot
        if snapshot is None or snapshot.voltage_v is None or observation.scale_grams is None:
            raise ValueError("电压或称重还没有恢复新鲜回读")
        if {snapshot.upper_channel,snapshot.lower_channel}!={1,2} or any(
                snapshot.erpm[channel-1] is None
                for channel in (snapshot.upper_channel,snapshot.lower_channel)):
            raise ValueError("两路DShot电转速还没有恢复新鲜回读")
        if snapshot.voltage_v<=self.stop_voltage_v+0.3:
            raise ValueError("当前电压已接近换电阈值，请先换电池")
        if getattr(self.engine,"active_operation",None) is not None:
            raise ValueError("上一次控制操作尚未退出")
        return self.start()
    def stop(self):
        self._cancel.set();sent=self.engine.stop()
        worker=self._worker
        if worker and worker is not threading.current_thread():
            worker.join(timeout=1)
            if worker.is_alive():
                self._publish("stopping","停止指令已发出，等待旧采集操作退出",stop_sent=bool(sent))
                return False  # Worker owns the lease until its finally block.
        self._publish("idle","已停止",stop_sent=bool(sent));return bool(sent)
    def close(self):
        # A finished controller must not STOP the bench: a newer operation may own it now.
        if self._worker is not None and self._worker.is_alive():self.stop()
        with self._lock:self._closed=True;self._state="closed"
    def _zero_scale(self):
        """Tare with both rotors confirmed stopped; a shifted zero biased a whole session by +28 g."""
        load_cell=getattr(self.engine,"load_cell",None)
        if load_cell is None:return None
        deadline=self.clock()+AUTO_ZERO_WAIT_S;still_since=None
        while self.clock()<deadline and not self._cancel.is_set():
            snapshot=self.engine.latest_observation().snapshot
            stopped=(snapshot is not None and all(value is not None and abs(value)<AUTO_ZERO_STOPPED_ERPM
                                                   for value in snapshot.erpm))
            still_since=(still_since if still_since is not None else self.clock()) if stopped else None
            if still_since is not None and self.clock()-still_since>=AUTO_ZERO_STILL_S:
                raw=load_cell.tare(samples=8)
                store=getattr(self.engine,"store",None)
                if store is not None and hasattr(store,"update_metadata"):store.update_metadata(scale_tare_raw=raw)
                if store is not None and hasattr(store,"event"):store.event("auto_tare",raw=raw,owner=self.OWNER)
                self._publish("zeroing","电机静止，已自动去皮")
                return raw
            self._cancel.wait(.05)
        self._publish("zeroing","桨未确认静止（或无转速回传），本次未自动去皮")
        return None
    def _run(self):
        started=self.clock();recorded=0;filled=0;reason="completed";reason_text=""
        collected_voltages=[]
        self.engine._plan_thread=threading.current_thread()
        try:
            self._zero_scale()
            self._publish("arming","正在解锁自动采集")
            if not self.engine.arm(int(math.ceil(self.max_percent))):
                from .manual_control import rejection_message
                board_reason=getattr(self.engine,"last_rejection_reason","")
                raise RuntimeError(rejection_message(board_reason) if board_reason else "台架解锁未确认")
            self._last_command=(0.0,0.0)
            total=len(self.schedule.cells)
            while True:
                if self._cancel.is_set():reason="user_stop";break
                if self._recorded_total+recorded>=self.max_points:reason="max_points";break
                if self._elapsed_total+self.clock()-started>=self.max_duration_s:reason="max_duration";break
                self._voltage()
                charge=self._charge_voltage()
                cell=self.schedule.next_cell(charge,self._last_command)
                if cell is None:reason="no_reachable_targets";break
                gap=not self.schedule.covered(cell,charge)
                missing=self.schedule.missing_count(charge)
                self._publish("tracking",
                    f"电量≈{charge:.2f}V：本段已覆盖{total-missing}/{total}格；"
                    f"本轮已记{recorded}点（补缺{filled}）；测上{cell.upper_pct:g}%/下{cell.lower_pct:g}%",
                    target=(cell.upper_pct,cell.lower_pct),covered=total-missing,total=total,
                    recorded=recorded,filled=filled)
                sample=self._measure_cell(cell,started)
                if sample is None:
                    self.schedule.mark_failed(cell,charge);continue
                self.schedule.add_point({"upper_erpm":sample.upper_erpm,"lower_erpm":sample.lower_erpm,"voltage_v":sample.voltage_v,
                                         "upper_command_pct":sample.upper_command_pct,"lower_command_pct":sample.lower_command_pct})
                self.estimator.observe(sample.host_time_s,sample.voltage_v,sample.upper_erpm,sample.lower_erpm)
                collected_voltages.append(float(sample.voltage_v))
                recorded+=1;filled+=gap
        except StopVoltageReached as exc:
            reason="stop_voltage";reason_text=str(exc)
        except Exception as exc:
            reason=("user_stop" if self._cancel.is_set() else
                    "source_lost" if isinstance(exc,SourceUnavailable) or self._stream_stopped() else "error")
            reason_text=str(exc)
            if reason!="user_stop":
                self._publish("waiting_recovery" if reason=="source_lost" else "error",reason_text)
        finally:
            self._recorded_total+=recorded; self._elapsed_total+=max(0.,self.clock()-started)
            self.engine.stop();self.engine.release_operation(self.OWNER)
            store=getattr(self.engine,"store",None)
            if store is not None and hasattr(store,"record_run"):
                try:
                    store.record_run({
                        "run_id":self._run_id,"plan_name":"automatic_dataset_collection",
                        "mode":"dual","recorded_points":recorded,"stop_reason":reason,
                        "max_percent":self.max_percent,
                        "stop_voltage_v":self.stop_voltage_v,
                        "max_duration_s":self.max_duration_s,
                        "max_points":self.max_points,
                        "schedule":"open_loop_command_grid",
                        "command_grid_pct":{"min":self.schedule.cells[0].upper_pct,"max":self.schedule.cells[-1].upper_pct,
                                            "step":self.schedule.step,"cells":len(self.schedule.cells)},
                        "charge_band_v":CHARGE_BAND_V,
                        "sag_k_v_per_erpm3_1e12":self.estimator.k,
                        "filled_gap_points":filled,
                        "loaded_voltage_min_v":min(collected_voltages) if collected_voltages else None,
                        "loaded_voltage_max_v":max(collected_voltages) if collected_voltages else None,
                        "completed":reason in {"max_points","max_duration","no_reachable_targets"},
                        "cancelled":reason in {"user_stop","error"},
                    })
                except Exception as exc:
                    self._publish("error",f"自动采集记录轮次失败：{exc}")
            if self.engine._plan_thread is threading.current_thread():self.engine._plan_thread=None
            state=("waiting_battery" if reason=="stop_voltage" else
                   "waiting_recovery" if reason=="source_lost" else
                   "error" if reason=="error" else "finished")
            tally=f"本轮记录{recorded}个稳态点，其中{filled}个补在此前缺失的格子"
            message=(f"{reason_text or '电压已到换电阈值'}；电机已停止。{tally}。换电后点击‘开始 / 换电继续’" if reason=="stop_voltage"
                     else f"飞控数据流曾失效，电机已停；{tally}。待读数恢复后点击‘开始 / 换电继续’" if reason=="source_lost"
                     else f"自动采集已停止：{reason_text}；{tally}" if reason=="error"
                     else f"自动采集已结束：{reason}；{tally}")
            self._publish(state,message,reason=reason,recorded_points=recorded,filled_gap_points=filled,run_id=self._run_id)
    def _measure_cell(self,cell,started):
        """Open loop: command the cell, wait for steady state, keep whatever eRPM it reached."""
        if not self._send_target(cell.upper_pct,cell.lower_pct,started):return None
        point=PlanPoint(f"auto-{uuid4().hex[:6]}",cell.upper_pct,cell.lower_pct,adaptive=True,
                        max_wait_s=AUTO_POINT_MAX_WAIT_S,stable_window_s=.35,min_scale_updates=5)
        stable,_=self.engine.wait_for_adaptive_stability(self._run_id,point,stop_voltage_v=LOADED_FLOOR_V)
        if not stable:
            self._publish("tracking","该格未在时限内稳定，本电量段跳过",segment_id=point.segment_id)
            return None
        return self._record_stable_segment(point.segment_id,started)

    def _record_stable_segment(self,segment_id,started):
        """Keep independent post-settle sources so this point can train a model."""
        deadline=min(started+self.max_duration_s,self.clock()+AUTO_RECORD_WINDOW_S)
        accepted=[]
        while self.clock()<deadline and not self._cancel.is_set():
            self._voltage()
            try:
                sample=self.engine.capture_latest_sample(self._run_id,segment_id,persist=False)
            except RuntimeError as exc:
                if "核心采样失效" in str(exc):
                    # Partial steady windows cannot straddle a measurement gap.
                    self._fresh_observation(scale=True,erpm=True)
                    accepted.clear()
                    deadline=min(started+self.max_duration_s,self.clock()+AUTO_RECORD_WINDOW_S)
                    continue
                raise
            if sample is not None:
                accepted.append(sample)
                while len(accepted)>1 and not self._spread_ok(accepted):accepted.pop(0)
                if self._independent_updates(accepted)>=5:
                    if self._cancel.is_set():return None
                    self.engine.commit_steady_samples(accepted)
                    self.last_segment_samples=list(accepted)
                    return replace(
                        accepted[-1],thrust_n=statistics.fmean(float(item.thrust_n) for item in accepted),
                        voltage_v=float(statistics.median(float(item.voltage_v) for item in accepted)),
                        upper_erpm=float(statistics.median(float(item.upper_erpm) for item in accepted)),
                        lower_erpm=float(statistics.median(float(item.lower_erpm) for item in accepted)))
            if self._cancel.wait(min(.08,max(.01,float(self.engine.snapshot_period)))):break
        self._publish("tracking","称重或转速未能稳定独立更新，该点暂不计入覆盖",segment_id=segment_id)
        return None
    @staticmethod
    def _independent_updates(samples):
        scale={item.scale_time_s for item in samples if item.scale_time_s is not None}
        sources={(item.fc_time_ms-item.upper_erpm_age_ms,item.fc_time_ms-item.lower_erpm_age_ms)
                 for item in samples if None not in (item.fc_time_ms,item.upper_erpm_age_ms,item.lower_erpm_age_ms)}
        return min(len(scale),len(sources))
    @staticmethod
    def _spread_ok(samples):
        """Default model._steady_points limits, so a committed point is one the model accepts."""
        thrust=[float(item.thrust_n) for item in samples]
        mean=statistics.fmean(thrust)
        if abs(mean)>.05 and statistics.stdev(thrust)/abs(mean)>.05:return False
        for name in ("upper_erpm","lower_erpm"):
            values=[float(getattr(item,name)) for item in samples]
            if statistics.stdev(values)/max(abs(statistics.fmean(values)),1.0)>.02:return False
        return True
    def _send_target(self,upper,lower,started):
        desired=(float(upper),float(lower))
        if any(value<0 or value>self.max_percent for value in desired):
            raise ValueError("目标油门超过本次最高油门")
        while any(abs(goal-current)>1e-6 for current,goal in zip(self._last_command,desired)):
            if self._cancel.is_set() or self.clock()-started>=self.max_duration_s:return False
            self._voltage()
            step=tuple(current+max(-AUTO_COMMAND_SLEW_PCT,min(AUTO_COMMAND_SLEW_PCT,goal-current))
                       for current,goal in zip(self._last_command,desired))
            if not self.engine.set_targets(*step):raise RuntimeError("双路油门未确认")
            self._last_command=step
        return True
    def _voltage(self):
        """Loaded voltage of a fresh reading; the battery check has already run on it."""
        obs=self._fresh_observation(scale=True,erpm=True);value=float(obs.snapshot.voltage_v)
        self._charge_window.append(self.estimator.compensated(value,*self._role_erpm(obs.snapshot)))
        return value
    def _check_battery(self,snapshot):
        loaded=float(snapshot.voltage_v)
        if loaded<=LOADED_FLOOR_V:
            raise StopVoltageReached(f"带载电压 {loaded:.2f} V，已跌破每节 3.0 V")
        try:self._battery_window.append(self.estimator.compensated(loaded,*self._role_erpm(snapshot)))
        except SourceUnavailable:
            if self._battery_window:return  # Sag unknown during a gap: keep the last charge estimate.
            self._battery_window.append(loaded)
        charge=statistics.median(self._battery_window)
        if charge<=self.stop_voltage_v:
            raise StopVoltageReached(f"电量电压约 {charge:.2f} V（带载 {loaded:.2f} V），已到换电阈值 {self.stop_voltage_v:g} V")
    def _charge_voltage(self):
        """Median load-compensated voltage: the state-of-charge coordinate for coverage."""
        if not self._charge_window:self._voltage()
        return float(statistics.median(self._charge_window))
    def _fresh_observation(self,*,scale=False,erpm=False):
        """Hold the last acknowledged target through a short measurement gap."""
        grace=float(getattr(self.engine,"TELEMETRY_GAP_GRACE_S",2.0))
        deadline=time.monotonic()+grace
        announced=False
        while True:
            if self._cancel.is_set():raise SourceUnavailable("自动采集已取消")
            if self._stream_stopped():raise SourceUnavailable("飞控控制窗口已因数据流失效停止")
            obs=self.engine.latest_observation()
            snap=obs.snapshot
            if snap is not None and snap.voltage_v is not None:self._check_battery(snap)
            good=snap is not None and snap.voltage_v is not None
            if good and scale:good=obs.scale_grams is not None
            if good and erpm:
                good=({snap.upper_channel,snap.lower_channel}=={1,2} and
                      all(snap.erpm[channel-1] is not None
                          for channel in (snap.upper_channel,snap.lower_channel)))
            if good:
                if announced:self._publish("tracking","读数恢复，继续采集；缺失时段未计入数据")
                return obs
            if not announced:
                self._publish("waiting_telemetry","读数短暂中断，保持当前油门并跳过缺失数据")
                announced=True
            if time.monotonic()>=deadline:
                raise SourceUnavailable(f"飞控读数持续中断超过{grace:g}秒，已停止")
            self._cancel.wait(min(.05,max(.01,float(self.engine.snapshot_period))))
    def _stream_stopped(self):
        cancel=getattr(self.engine,"cancel",None)
        return bool(cancel is not None and cancel.is_set() and not self._cancel.is_set())
    @staticmethod
    def _role_erpm(snapshot):
        upper=snapshot.erpm[snapshot.upper_channel-1]
        lower=snapshot.erpm[snapshot.lower_channel-1]
        if upper is None or lower is None:
            raise SourceUnavailable("DShot电转速回读已过期，自动采集停止")
        return float(upper),float(lower)
