"""Full-throttle thrust test.

Ramp both rotors to 100 %, record one steady segment, stop.
"""
from __future__ import annotations
import math, statistics, threading, time
from uuid import uuid4
from .acquisition import StopVoltageReached
from .autocollect import LOADED_FLOOR_V, AutoCollector, SourceUnavailable
from .plans import PlanPoint
from .sweep_schedule import SAG_K_DEFAULT, load_index

MAX_THRUST_SETTLE_S=4.0
MAX_THRUST_FALLBACK_S=1.0
GRAMS_PER_NEWTON=1000.0/9.80665


class MaxThrustTest(AutoCollector):
    """Reuses the auto-collect lease, gap tolerance, stop-voltage check and ramp."""
    OWNER="max_thrust"
    def __init__(self,engine,*,max_percent,stop_voltage_v,events=None,clock=time.monotonic):
        super().__init__(engine,(),max_percent=max_percent,stop_voltage_v=stop_voltage_v,
                         max_duration_s=60.0,max_points=1,events=events,clock=clock)
        self.result=None
    def _run(self):
        started=self.clock();reason="completed";reason_text="";result=None
        target=self.max_percent
        self.engine._plan_thread=threading.current_thread()
        try:
            self._zero_scale()
            self._publish("arming",f"正在解锁最大推力测试（目标双桨 {target:g}%）")
            if not self.engine.arm(int(math.ceil(target))):
                from .manual_control import rejection_message
                board_reason=getattr(self.engine,"last_rejection_reason","")
                raise RuntimeError(rejection_message(board_reason) if board_reason else "台架解锁未确认")
            self._last_command=(0.0,0.0)
            self._publish("ramping",f"油门每步≤10%升到 {target:g}%")
            if not self._send_target(target,target,started):
                reason="user_stop"
            else:
                point=PlanPoint(f"max-{uuid4().hex[:6]}",target,target,adaptive=True,
                                max_wait_s=MAX_THRUST_SETTLE_S,stable_window_s=.35,min_scale_updates=5)
                self._publish("settling","已到目标油门，等待推力稳定")
                stable,_=self.engine.wait_for_adaptive_stability(self._run_id,point,stop_voltage_v=LOADED_FLOOR_V)
                sample=self._record_stable_segment(point.segment_id,started) if stable else None
                result=(self._steady_result(sample,target) if sample is not None
                        else self._fallback_result(target))
                if result is None and self._cancel.is_set():reason="user_stop"
        except StopVoltageReached as exc:
            reason="stop_voltage";reason_text=str(exc)
        except Exception as exc:
            reason=("user_stop" if self._cancel.is_set() else
                    "source_lost" if isinstance(exc,SourceUnavailable) or self._stream_stopped() else "error")
            reason_text=str(exc)
        finally:
            self.engine.stop();self.engine.release_operation(self.OWNER)
            self.result=result
            store=getattr(self.engine,"store",None)
            if store is not None and hasattr(store,"record_run"):
                try:
                    store.record_run({"run_id":self._run_id,"plan_name":"max_thrust_test","mode":"dual",
                        "target_percent":target,"stop_voltage_v":self.stop_voltage_v,"stop_reason":reason,
                        "recorded_points":int(bool(result and result["steady"])),"max_thrust":result,
                        "completed":result is not None,"cancelled":reason in {"user_stop","error"}})
                except Exception as exc:
                    self._publish("error",f"最大推力测试记录失败：{exc}")
            if self.engine._plan_thread is threading.current_thread():self.engine._plan_thread=None
            self._publish("finished" if result is not None or reason=="user_stop" else "error",
                          self._message(result,reason,reason_text),
                          reason=reason,result=result,run_id=self._run_id,recorded_points=int(bool(result and result["steady"])))
    def _steady_result(self,sample,target):
        thrust=[float(item.thrust_n)*GRAMS_PER_NEWTON for item in self.last_segment_samples]
        return {"steady":True,"target_percent":target,"thrust_gf":statistics.fmean(thrust),
                "peak_gf":max(thrust),"spread_gf":statistics.stdev(thrust),"samples":len(thrust),
                "upper_erpm":float(sample.upper_erpm),"lower_erpm":float(sample.lower_erpm),
                "loaded_voltage_v":float(sample.voltage_v),
                "charge_voltage_v":float(sample.voltage_v)+SAG_K_DEFAULT*load_index(sample.upper_erpm,sample.lower_erpm)}
    def _fallback_result(self,target):
        """Not steady within the settle limit: report live readings, never enter the dataset."""
        readings=[];deadline=self.clock()+MAX_THRUST_FALLBACK_S
        while self.clock()<deadline and not self._cancel.is_set():
            observation=self._fresh_observation(scale=True,erpm=True)
            readings.append((float(observation.scale_grams),*self._role_erpm(observation.snapshot),
                             float(observation.snapshot.voltage_v)))
            if self._cancel.wait(min(.08,max(.01,float(self.engine.snapshot_period)))):break
        if not readings:return None
        grams,upper,lower,voltage=(list(column) for column in zip(*readings))
        return {"steady":False,"target_percent":target,"thrust_gf":statistics.fmean(grams),
                "peak_gf":max(grams),"spread_gf":statistics.pstdev(grams),"samples":len(grams),
                "upper_erpm":statistics.fmean(upper),"lower_erpm":statistics.fmean(lower),
                "loaded_voltage_v":statistics.fmean(voltage),
                "charge_voltage_v":statistics.fmean(voltage)+SAG_K_DEFAULT*load_index(
                    statistics.fmean(upper),statistics.fmean(lower))}
    @staticmethod
    def _message(result,reason,reason_text):
        if result is not None:
            head="最大推力（稳态平均）" if result["steady"] else "最大推力（未达稳态，仅供参考、不入库）"
            return (f"{head} {result['thrust_gf']:.0f} 克，窗口内峰值 {result['peak_gf']:.0f} 克，波动±{result['spread_gf']:.0f} 克；"
                    f"油门 {result['target_percent']:g}%，上桨 {result['upper_erpm']:.0f}、下桨 {result['lower_erpm']:.0f} eRPM；"
                    f"带载 {result['loaded_voltage_v']:.2f} V（电量约 {result['charge_voltage_v']:.2f} V）。电机已停止。")
        return {"stop_voltage":f"{reason_text or '电压到换电阈值'}，已停止；换电后再测",
                "user_stop":"已停止，未得到最大推力",
                "source_lost":f"读数中断，已停止：{reason_text}"}.get(reason,f"最大推力测试失败：{reason_text or reason}")
