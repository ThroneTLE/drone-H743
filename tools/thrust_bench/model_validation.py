"""Model check on the bench.

For each target thrust: invert the throttle model at the present battery charge,
command both rotors to that throttle, measure the steady thrust, compare.
With the lookup-table backend the speed table is also checked at the measured
eRPM. The run is recorded as `model_validation` and never used for training.

SplitValidationTest: yaw-like splits of a total thrust, each commanded with the old
per-rotor allocation and the new one that keeps the total on the 2D table (A/B).
"""
from __future__ import annotations
import json, statistics, threading, time
from pathlib import Path
from uuid import uuid4
from .acquisition import StopVoltageReached
from .autocollect import LOADED_FLOOR_V, AutoCollector, SourceUnavailable
from .plans import PlanPoint
from . import result_pages, throttle_model, thrust_lut

TARGET_FRACTIONS=(0.35,0.8,0.15,0.6,0.95,0.25,0.7,0.45,0.9,0.05)  # interleaved: no one-way drift
TARGET_LOW_PCT=8.0
TARGET_HIGH_PCT=95.0
ERROR_LIMIT_GF=50.0
SETTLE_S=4.0
SPLIT_TOTAL_FRACTIONS=(0.3,0.55,0.8)  # of the 8..95 % same-throttle thrust range at this charge
SPLIT_RATIOS=(0.15,-0.15,0.3,-0.3)  # (upper - lower) / total, controller convention
SPLIT_ORDER=((1,2),(0,3),(2,0),(1,1),(0,0),(2,3),(1,0),(0,1),(2,2),(1,3),(0,2),(2,1))  # interleaved


class ModelValidationTest(AutoCollector):
    OWNER="model_validation"
    PLAN_NAME="model_validation"
    def __init__(self,engine,model,model_path,*,stop_voltage_v,backend=throttle_model,events=None,clock=time.monotonic,
                 max_points=len(TARGET_FRACTIONS)):
        super().__init__(engine,(),max_percent=100,stop_voltage_v=stop_voltage_v,
                         max_duration_s=600.0,max_points=max_points,events=events,clock=clock)
        self.model=model;self.model_path=Path(model_path) if model_path else None;self.results=[]
        self.backend=backend  # predict_gf/balanced_throttle: throttle_model or thrust_lut
    def target_thrusts(self,charge_v):
        low=self.backend.predict_gf(self.model,TARGET_LOW_PCT,TARGET_LOW_PCT,charge_v)
        high=self.backend.predict_gf(self.model,TARGET_HIGH_PCT,TARGET_HIGH_PCT,charge_v)
        if low is None or high is None:return []
        return [round(low+fraction*(high-low)) for fraction in TARGET_FRACTIONS]
    def _cases(self,charge_v):
        return [{"target_gf":target} for target in self.target_thrusts(charge_v)]
    def _command(self,case,charge_v):
        command=self.backend.balanced_throttle(self.model,case["target_gf"],charge_v)
        return None if command is None else (command,command)
    @staticmethod
    def _describe(case):
        return f"目标 {case['target_gf']} 克"
    def _page(self,record,folder):
        return result_pages.validation_page(record,folder)
    def _run(self):
        started=self.clock();reason="completed";reason_text="";self.results=[]
        self.engine._plan_thread=threading.current_thread()
        try:
            if self._zero_scale() is None and getattr(self.engine,"load_cell",None) is not None:
                raise RuntimeError("桨未确认静止，没能自动去皮；等桨停稳后再点")
            charge=self._charge_voltage()
            cases=self._cases(charge)
            if not cases:
                low,high=self.model["domain"].get("charge_v_usable",self.model["domain"]["charge_v"])
                raise RuntimeError(f"当前电量约 {charge:.2f} V，不在模型范围 {low:.2f}～{high:.2f} V 内")
            self._publish("arming",f"实测验证：{len(cases)} 个目标，按表反推油门后实测")
            if not self.engine.arm(100):
                raise RuntimeError("台架解锁未确认")
            self._last_command=(0.0,0.0)
            for index,case in enumerate(cases,1):
                if self._cancel.is_set():reason="user_stop";break
                self._voltage()
                charge=self._charge_voltage()
                command=self._command(case,charge)
                target=case["target_gf"]
                entry={**case,"charge_v_before":charge}
                if hasattr(self.backend,"is_extrapolated"):entry["charge_extrapolated"]=self.backend.is_extrapolated(self.model,charge)
                if command is None:
                    entry.update(command_pct=None,skipped="当前电量下查不到或已饱和")
                    self.results.append(entry);continue
                upper,lower=(round(value,2) for value in command)
                entry.update(upper_pct=upper,lower_pct=lower)
                if upper==lower:entry["command_pct"]=upper
                self._publish("tracking",f"第 {index}/{len(cases)} 个：{self._describe(case)} → 油门 上{upper:.1f}%/下{lower:.1f}%（电量 {charge:.2f} V）")
                if not self._send_target(upper,lower,started):reason="user_stop";break
                point=PlanPoint(f"val-{uuid4().hex[:6]}",upper,lower,adaptive=True,
                                max_wait_s=SETTLE_S,stable_window_s=.35,min_scale_updates=5)
                stable,_=self.engine.wait_for_adaptive_stability(self._run_id,point,stop_voltage_v=LOADED_FLOOR_V)
                sample=self._record_stable_segment(point.segment_id,started) if stable else None
                if sample is None:
                    entry["skipped"]="推力未稳定";self.results.append(entry);continue
                measured=float(sample.thrust_n)*throttle_model.GRAMS_PER_NEWTON
                charge_measured=throttle_model.charge_voltage(sample.voltage_v,sample.upper_erpm,sample.lower_erpm)
                predicted=self.backend.predict_gf(self.model,upper,lower,charge_measured)
                entry.update(measured_gf=measured,error_gf=measured-target,charge_v_measured=charge_measured,
                             predicted_at_measured_charge_gf=predicted,
                             upper_erpm=float(sample.upper_erpm),lower_erpm=float(sample.lower_erpm))
                speed_text=""
                if hasattr(self.backend,"thrust_from_speed"):
                    speed=self.backend.thrust_from_speed(self.model,sample.upper_erpm,sample.lower_erpm)
                    if speed is not None:
                        entry.update(speed_table_gf=speed,speed_error_gf=measured-speed)
                        speed_text=f"；转速表查得 {speed:.0f} 克，误差 {measured-speed:+.0f} 克"
                self.results.append(entry)
                self._publish("tracking",f"{self._describe(case)} → 实测 {measured:.0f} 克，误差 {measured-target:+.0f} 克{speed_text}")
        except StopVoltageReached as exc:
            reason="stop_voltage";reason_text=str(exc)
        except Exception as exc:
            reason=("user_stop" if self._cancel.is_set() else
                    "source_lost" if isinstance(exc,SourceUnavailable) or self._stream_stopped() else "error")
            reason_text=str(exc)
        finally:
            self.engine.stop();self.engine.release_operation(self.OWNER)
            summary=self.summary();report=None
            record={"run_id":self._run_id,"plan_name":self.PLAN_NAME,"mode":"dual","stop_reason":reason,
                    "model_id":self.model.get("model_id"),"model_schema":self.model.get("schema"),"model_path":str(self.model_path) if self.model_path else None,
                    "error_limit_gf":ERROR_LIMIT_GF,"summary":summary,"targets":self.results,
                    "recorded_points":summary["measured"],"completed":reason=="completed","cancelled":reason in {"user_stop","error"}}
            store=getattr(self.engine,"store",None)
            if store is not None and hasattr(store,"record_run"):
                try:store.record_run(record)
                except Exception as exc:self._publish("error",f"验证记录失败：{exc}")
            if self.model_path is not None and self._run_id:
                try:(self.model_path.parent/f"validation-{self._run_id}.json").write_text(
                        json.dumps(record,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
                except OSError as exc:self._publish("error",f"验证结果文件写入失败：{exc}")
                try:report=str(self._page(record,self.model_path.parent))
                except Exception as exc:self._publish("error",f"验证结果图生成失败：{exc}")
            if self.engine._plan_thread is threading.current_thread():self.engine._plan_thread=None
            self._publish("finished" if summary["measured"] else "error",self._message(summary,reason,reason_text),
                          reason=reason,summary=summary,run_id=self._run_id,report=report)
    def summary(self):
        errors=[entry["error_gf"] for entry in self.results if "error_gf" in entry]
        speed=[entry["speed_error_gf"] for entry in self.results if "speed_error_gf" in entry]
        return {"targets":len(self.results),"measured":len(errors),
                "skipped":sum("skipped" in entry for entry in self.results),
                "mae_gf":statistics.fmean(abs(e) for e in errors) if errors else None,
                "max_abs_gf":max((abs(e) for e in errors),default=None),
                "within_limit":sum(abs(e)<=ERROR_LIMIT_GF for e in errors),
                "speed_checked":len(speed),
                "speed_mae_gf":statistics.fmean(abs(e) for e in speed) if speed else None,
                "speed_max_abs_gf":max((abs(e) for e in speed),default=None)}
    @staticmethod
    def _message(summary,reason,reason_text):
        head={"stop_voltage":f"{reason_text or '电压到换电阈值'}，已停止；","user_stop":"已手动停止；",
              "source_lost":f"读数中断已停止（{reason_text}）；",
              "error":f"验证中止：{reason_text}；"}.get(reason,"")
        if not summary["measured"]:
            return head+"没有得到实测对比点"
        verdict=("全部在 50 克以内" if summary["within_limit"]==summary["measured"]
                 else f"{summary['measured']-summary['within_limit']} 个超出 50 克")
        speed=(f"按实测转速查转速表：平均误差 {summary['speed_mae_gf']:.0f} 克，最大 {summary['speed_max_abs_gf']:.0f} 克。"
               if summary.get("speed_checked") else "")
        return (f"{head}实测验证：{summary['measured']} 个目标实测完成（跳过 {summary['skipped']} 个）；"
                f"按油门反推：平均误差 {summary['mae_gf']:.0f} 克，最大 {summary['max_abs_gf']:.0f} 克，{verdict}。{speed}"
                "电机已停止。点“查看拟合效果”看每个目标的误差图。")


class SplitValidationTest(ModelValidationTest):
    """Yaw-like splits, old per-rotor allocation vs the pair allocation the firmware now uses."""
    OWNER="split_validation"
    PLAN_NAME="split_validation"
    def __init__(self,engine,model,model_path,*,stop_voltage_v,events=None,clock=time.monotonic):
        super().__init__(engine,model,model_path,stop_voltage_v=stop_voltage_v,backend=thrust_lut,events=events,
                         clock=clock,max_points=2*len(SPLIT_ORDER))
        self._diag=thrust_lut.balanced_diagonal(model)
    def _cases(self,charge_v):
        low=thrust_lut.predict_gf(self.model,TARGET_LOW_PCT,TARGET_LOW_PCT,charge_v)
        high=thrust_lut.predict_gf(self.model,TARGET_HIGH_PCT,TARGET_HIGH_PCT,charge_v)
        if low is None or high is None:return []
        cases=[]
        for index,(total_index,ratio_index) in enumerate(SPLIT_ORDER):
            total=round(low+SPLIT_TOTAL_FRACTIONS[total_index]*(high-low));ratio=SPLIT_RATIOS[ratio_index]
            order=("new","old") if index%2==0 else ("old","new")  # neither allocation always runs first
            cases+=[{"target_gf":total,"split":ratio,"upper_gf":total*(1+ratio)/2,"lower_gf":total*(1-ratio)/2,
                     "allocation":allocation} for allocation in order]
        return cases
    def _command(self,case,charge_v):
        pair=thrust_lut.pair_throttle(self.model,case["upper_gf"],case["lower_gf"],charge_v,
                                      correct=case["allocation"]=="new",diag=self._diag)
        if pair is None or max(pair["upper_pct"],pair["lower_pct"])>=100.0:return None
        case["shift_pct"]=pair["shift_pct"]
        return pair["upper_pct"],pair["lower_pct"]
    @staticmethod
    def _describe(case):
        side="上" if case["split"]>0 else "下"
        return (f"总推力 {case['target_gf']} 克、{side}桨多分 {abs(case['split'])*100:.0f}%"
                f"（{'新' if case['allocation']=='new' else '旧'}分配）")
    def _page(self,record,folder):
        return result_pages.split_validation_page(record,folder)
    def summary(self):
        result=super().summary()
        for allocation in ("new","old"):
            errors=[e["error_gf"] for e in self.results if e.get("allocation")==allocation and "error_gf" in e]
            result[allocation]={"measured":len(errors),
                                "mae_gf":statistics.fmean(abs(v) for v in errors) if errors else None,
                                "max_abs_gf":max((abs(v) for v in errors),default=None),
                                "bias_gf":statistics.fmean(errors) if errors else None}
        return result
    @staticmethod
    def _message(summary,reason,reason_text):
        head={"stop_voltage":f"{reason_text or '电压到换电阈值'}，已停止；","user_stop":"已手动停止；",
              "source_lost":f"读数中断已停止（{reason_text}）；",
              "error":f"验证中止：{reason_text}；"}.get(reason,"")
        if not summary["measured"]:
            return head+"没有得到实测对比点"
        part=lambda name,s:(f"{name} {s['measured']} 点：平均误差 {s['mae_gf']:.0f} 克、最大 {s['max_abs_gf']:.0f} 克、"
                            f"平均偏 {s['bias_gf']:+.0f} 克" if s["measured"] else f"{name}：无实测点")
        return (f"{head}差速实测验证（总推力是否随上下分配变化）：{part('新分配',summary['new'])}；"
                f"{part('旧分配',summary['old'])}。电机已停止。点“查看拟合效果”看对比图。")
