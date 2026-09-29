"""Asynchronous H743 manual control using the existing safe TBENCH engine."""
from __future__ import annotations
import math, queue, threading, time
from .acquisition import UiEvent


def rejection_message(reason):
    hints = {
        "mapping_unconfirmed": "上下桨映射尚未确认：请先逐通道试转，选择上下桨及俯视旋向，再提交映射",
        "armed": "飞控处于飞行解锁状态，请先锁定飞控，再使用台架解锁",
        "battery_not_ready": "电压读数尚未有效，请检查电源和电压回读",
        "battery_low": "电压低于允许解锁范围",
        "prop_spin_active": "通道试转尚未停止，请先停止试转",
        "sysid_running": "系统辨识正在运行，请先停止",
        "already_open": "飞控台架窗口已经打开，请先停止并确认空闲",
    }
    return hints.get(reason, f"飞控拒绝台架操作：{reason or '原因未知'}")


class ManualControl:
    OWNER = "manual"
    def __init__(self, engine, event_queue=None, *, clock=time.monotonic):
        self.engine=engine; self.events=event_queue or queue.Queue(); self.clock=clock
        self._lock=threading.RLock(); self._condition=threading.Condition(self._lock)
        self._state="idle"; self._closed=False; self._owns=False
        self._last_error=""
        self._session=0; self._apply_id=0; self._deadline=None
        self._arm_thread=None; self._apply_thread=None; self._monitor_thread=None

    @property
    def state(self):
        with self._lock: return self._state
    @property
    def busy(self): return self.state not in {"idle","closed","error"}
    @property
    def last_error(self):
        with self._lock: return self._last_error
    def _publish_locked(self,state,message,**data):
        self._state=state
        if state=="error": self._last_error=message
        elif state=="arming": self._last_error=""
        self.events.put(UiEvent("manual_state",{"state":state,"message":message,**data}))

    def arm(self,max_percent):
        with self._condition:
            if self._state not in {"idle","error"} or self._closed or self._owns or self._workers_alive_locked(): return False
            if not self.engine.claim_operation(self.OWNER):
                self._publish_locked("error","扫描或其他台架操作正在占用控制窗口"); return False
            self._owns=True; self._session+=1; session=self._session
            self._publish_locked("arming","正在等待台架解锁确认")
            self._arm_thread=threading.Thread(target=self._arm_worker,args=(int(max_percent),session),daemon=True,name="tbench-manual-arm")
            self._arm_thread.start(); return True

    def _arm_worker(self,max_percent,session):
        try:
            success=self.engine.arm(max_percent)
            reason=getattr(self.engine,"last_rejection_reason","")
            message=("台架已解锁；当前双路目标仍为0%" if success else
                     rejection_message(reason) if reason else "台架解锁未确认：请检查连接或等待回包后重试")
        except Exception as exc: success=False; message=f"台架解锁异常：{exc}"
        if not success: self.engine.stop()  # ACK may be lost after board accepted ARM.
        with self._condition:
            stale=session!=self._session or self._closed
            if stale: self._condition.notify_all()
            else:
                if success:
                    self._publish_locked("armed",message,max_percent=max_percent)
                    self._monitor_thread=threading.Thread(target=self._monitor_worker,args=(session,),daemon=True,name="tbench-manual-monitor")
                    self._monitor_thread.start()
                else:
                    self._release_locked(); self._publish_locked("error",message+"；已停止")
                self._condition.notify_all()
        if stale:
            if success: self.engine.stop()
            return

    def apply(self,upper,lower,hold_s):
        upper,lower,hold_s=float(upper),float(lower),float(hold_s)
        if not all(math.isfinite(v) for v in (upper,lower,hold_s)): raise ValueError("手动目标和保持时间必须是有限数字")
        if not 0<=upper<=100 or not 0<=lower<=100: raise ValueError("上下桨目标必须在0..100%")
        if not 0<hold_s<=60: raise ValueError("保持时间必须在0..60秒")
        with self._condition:
            if self._state not in {"armed","holding"} or self._closed: return False
            if self._apply_thread is not None and self._apply_thread.is_alive(): return False
            session=self._session; self._apply_id+=1; apply_id=self._apply_id
            self._publish_locked("applying","正在等待双路油门确认",upper=upper,lower=lower,hold_s=hold_s)
            self._apply_thread=threading.Thread(target=self._apply_worker,args=(upper,lower,hold_s,session,apply_id),daemon=True,name="tbench-manual-apply")
            self._apply_thread.start(); return True

    def _apply_worker(self,upper,lower,hold_s,session,apply_id):
        try: success=self.engine.set_targets(upper,lower); message="双路油门已确认" if success else "双路油门被拒绝或超时"
        except Exception as exc: success=False; message=f"双路油门异常：{exc}"
        with self._condition:
            current=(session==self._session and apply_id==self._apply_id and not self._closed and self._state not in {"stopping","closed"})
            if not current: self._condition.notify_all()
            if success:
                if current:
                    self._deadline=self.clock()+hold_s
                    self._publish_locked("holding",message,upper=upper,lower=lower,hold_s=hold_s,deadline=self._deadline)
                    self._condition.notify_all(); return
        if not current:
            if success: self.engine.stop()
            return
        self._stop_session(message+"；已停止",final_state="error",expected=session)

    def _monitor_worker(self,session):
        while True:
            with self._condition:
                if session!=self._session or self._closed: self._condition.notify_all(); return
                if self._state in {"stopping","idle","error","closed"}: self._condition.notify_all(); return
                if not self.engine.armed:
                    self._deadline=None
                    break
                if self._deadline is not None and self.clock()>=self._deadline: self._deadline=None; break
                timeout=.05 if self._deadline is None else min(.05,max(0.,self._deadline-self.clock()))
                self._condition.wait(timeout=timeout)
        message=("连接或安全监视已关闭台架控制窗口，已停止"
                 if not self.engine.armed else "保持时间结束，已自动停止")
        self._stop_session(message,final_state="error" if not self.engine.armed else "idle",expected=session)

    def latest_observation(self): return self.engine.latest_observation()
    def stop(self): return self._stop_session("已请求手动停止",final_state="idle")
    def _stop_session(self,message,*,final_state,expected=None):
        with self._condition:
            if self._state=="closed": return False
            if expected is not None and expected!=self._session: return False
            owns=self._owns; self._session+=1; barrier=self._session; self._apply_id+=1; self._deadline=None
            self._publish_locked("stopping",message); self._condition.notify_all()
        sent=bool(self.engine.stop()) if owns else False
        drained=self._join_old_workers()
        with self._condition:
            if barrier!=self._session: return sent
            if not drained:
                self._publish_locked("error","旧手动操作未能及时退出，控制租约仍被保留",stop_sent=sent); return sent
            self._release_locked(); self._publish_locked(final_state,message,stop_sent=sent); self._condition.notify_all()
        return sent

    def _join_old_workers(self):
        current=threading.current_thread(); deadline=time.monotonic()+.8
        threads=(self._arm_thread,self._apply_thread,self._monitor_thread)
        for thread in threads:
            if thread is None or thread is current: continue
            remaining=deadline-time.monotonic()
            if remaining>0: thread.join(timeout=remaining)
        return all(t is None or t is current or not t.is_alive() for t in threads)
    def _workers_alive_locked(self):
        return any(t is not None and t.is_alive() for t in (self._arm_thread,self._apply_thread,self._monitor_thread))
    def _release_locked(self):
        if self._owns:
            self._owns=False; self.engine.release_operation(self.OWNER)
    def close(self):
        with self._condition:
            if self._state=="closed": return
        self._stop_session("手动控制已关闭",final_state="closed")
        with self._condition:
            self._closed=True; self._state="closed"; self._condition.notify_all()
