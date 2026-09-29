"""Safe asynchronous propeller mapping over the existing PROPCAL command path."""
from __future__ import annotations

import math
import queue
import threading
import time

from tools.panel_lib.proto import parse_kv
from .acquisition import UiEvent
from .connection import estimated_wire_bytes_per_second

HEARTBEAT_S = 0.120
REPLY_TIMEOUT_S = 0.45


def estimated_mapping_wire_bytes_per_second(snapshot_hz: float = 15.0) -> int:
    command = len("PROPCAL SPIN SET ch=2 pct=20\r\n".encode())
    reply = len((
        "PROPCAL spin=active ch=2 pct=20 age_ms=4294967295 max_pct=20 "
        "timeout_ms=300 stop=heartbeat_lost esc_telem=1 "
        "esc_i1=4294967295 esc_i2=4294967295 "
        "esc_erpm1=4294967295 esc_erpm2=4294967295").encode()) + 9
    snapshot_only = estimated_wire_bytes_per_second(snapshot_hz, 0.0)
    return math.ceil(snapshot_only + (command + reply) / HEARTBEAT_S)


class MappingControl:
    OWNER = "mapping"
    def __init__(self, engine, events=None, *, clock=time.monotonic):
        self.engine=engine; self.events=events or queue.Queue(); self.clock=clock
        self._lock=threading.RLock(); self._state="idle"; self._generation=0
        self._worker=None; self._owns=False; self._closed=False
    @property
    def state(self):
        with self._lock: return self._state
    @property
    def busy(self): return self.state not in {"idle","error","closed"}
    def _publish_locked(self,state,message,**data):
        self._state=state
        self.events.put(UiEvent("mapping_state",{"state":state,"message":message,**data}))
    def _publish(self,state,message,**data):
        with self._lock: self._publish_locked(state,message,**data)
    def _start(self,state,target,*args):
        with self._lock:
            if self._closed or self._state not in {"idle","error"} or (self._worker and self._worker.is_alive()): return False
            if not self.engine.claim_operation(self.OWNER):
                self._publish_locked("error","手动控制或扫描正在占用飞控"); return False
            self._owns=True; self._generation+=1; generation=self._generation
            link_generation=self.engine.connection.generation
            self._publish_locked(state,"操作已开始")
            self._worker=threading.Thread(target=target,args=(*args,generation,link_generation),daemon=True,name=f"prop-map-{state}")
            self._worker.start(); return True
    def query(self): return self._start("querying",self._query_worker)
    def test_channel(self,channel,percent=5,duration_s=1.5,confirmed=False):
        channel=int(channel); percent=float(percent); duration_s=float(duration_s)
        if not confirmed: return False
        if channel not in (1,2): raise ValueError("测试通道必须是1或2")
        if not math.isfinite(percent) or not 1<=percent<=20: raise ValueError("测试油门必须在1..20%")
        if not math.isfinite(duration_s) or not 0<duration_s<=3: raise ValueError("测试时间必须在0..3秒")
        snapshot_hz=1.0/float(self.engine.snapshot_period)
        decision=self.engine.connection.validate_snapshot_rate(snapshot_hz)
        total=estimated_mapping_wire_bytes_per_second(snapshot_hz)
        if total>3456 and not decision.get("confirmed_stm32_usb_cdc",False):
            raise ValueError(f"点转心跳与{snapshot_hz:g}Hz快照预计{total}B/s，超过数传预算；请使用已确认USB或降低快照频率")
        return self._start("testing",self._test_worker,channel,int(round(percent)),duration_s)
    def apply_mapping(self,mapping):
        normalized=self._normalize_mapping(mapping)
        return self._start("applying",self._apply_worker,normalized)
    def save(self): return self._start("saving",self._save_worker)

    @staticmethod
    def _normalize_mapping(mapping):
        if set(mapping)!={1,2}: raise ValueError("映射必须同时包含通道1和2")
        result={}
        for channel in (1,2):
            role=str(mapping[channel].get("role","")).lower(); spin=str(mapping[channel].get("spin","")).lower()
            if role not in {"upper","lower"} or spin not in {"cw","ccw"}: raise ValueError("角色须为upper/lower，旋向须为cw/ccw")
            result[channel]={"role":role,"spin":spin}
        if result[1]["role"]==result[2]["role"]: raise ValueError("两路角色必须不同")
        if result[1]["spin"]==result[2]["spin"]: raise ValueError("两路旋向必须相反（统一俯视）")
        return result

    def _query_worker(self,generation,link_generation):
        try:
            sent=self.clock(); self._drain();
            if not self._send_effect(generation,link_generation,"PROPCAL?"): raise RuntimeError("查询未发出")
            status=self._collect_mapping(sent,generation,link_generation,require_state=None)
            self._finish(generation,link_generation,"idle","已读取飞控映射",status=status,channels=status.get("channels"))
        except Exception as exc: self._fail(generation,link_generation,exc,stop=False)

    def _test_worker(self,channel,percent,duration_s,generation,link_generation):
        try:
            self._drain(); sent=self.clock()
            if not self._send_effect(generation,link_generation,"PROPCAL SPIN ARM confirm=safe max_pct=20"): raise RuntimeError("台架解锁命令未发出")
            self._wait_spin(sent,generation,link_generation,"active",0,0)
            deadline=self.clock()+duration_s; next_send=-math.inf
            while self.clock()<deadline:
                self._check_generation(generation,link_generation)
                if self.clock()>=next_send:
                    sent=self.clock()
                    if not self._send_effect(generation,link_generation,f"PROPCAL SPIN SET ch={channel} pct={percent}"): raise RuntimeError("测试油门未发出")
                    self._wait_spin(sent,generation,link_generation,"active",channel,percent)
                    next_send=self.clock()+HEARTBEAT_S
                time.sleep(min(.02,max(0.,deadline-self.clock())))
            self._stop_window(generation,link_generation)
            self._finish(generation,link_generation,"idle",f"通道{channel}测试结束，已停止")
        except Exception as exc: self._fail(generation,link_generation,exc,stop=True)

    def _apply_worker(self,mapping,generation,link_generation):
        try:
            self._stop_window(generation,link_generation)
            for channel in (1,2):
                item=mapping[channel]; sent=self.clock(); self._drain()
                command=f"PROPCAL SET ch={channel} role={item['role']} spin={item['spin']}"
                if not self._send_effect(generation,link_generation,command): raise RuntimeError("RAM映射命令未发出")
                allowed={"partial","conflict","applied_ram"} if channel==1 else {"applied_ram"}
                status=self._collect_mapping(sent,generation,link_generation,require_state=allowed)
                if not self._channel_matches(status.get("channels",{}),channel,item): raise RuntimeError(f"通道{channel}回表与所选角色/旋向不一致")
            if (status.get("calibrated")!="1" or not self._mapping_matches(status,mapping)): raise RuntimeError("飞控整表与所选映射不一致")
            self._finish(generation,link_generation,"idle","RAM映射已应用（尚未保存）",status=status,channels=status["channels"])
        except Exception as exc: self._fail(generation,link_generation,exc,stop=True)

    def _save_worker(self,generation,link_generation):
        try:
            self._stop_window(generation,link_generation); self._drain(); sent=self.clock()
            if not self._send_effect(generation,link_generation,"PROPCAL COMMIT"): raise RuntimeError("保存命令未发出")
            status=self._collect_mapping(sent,generation,link_generation,require_state={"committed"})
            if status.get("calibrated")!="1": raise RuntimeError("飞控未确认标定状态")
            self._finish(generation,link_generation,"idle","映射已保存到Flash",status=status,channels=status.get("channels"))
        except Exception as exc: self._fail(generation,link_generation,exc,stop=True)

    @staticmethod
    def _channel_matches(channels,channel,expected):
        actual=channels.get(channel,{})
        return actual.get("role")==expected["role"] and actual.get("spin")==expected["spin"]

    @classmethod
    def _mapping_matches(cls,status,mapping):
        channels=status.get("channels",{})
        if not all(cls._channel_matches(channels,ch,mapping[ch]) for ch in (1,2)): return False
        if not all(channels.get(ch,{}).get("declared")=="1" for ch in (1,2)): return False
        upper=next(ch for ch in (1,2) if mapping[ch]["role"]=="upper")
        lower=next(ch for ch in (1,2) if mapping[ch]["role"]=="lower")
        return status.get("upper_ch")==str(upper) and status.get("lower_ch")==str(lower)

    def _collect_mapping(self,sent,generation,link_generation,require_state):
        deadline=self.clock()+REPLY_TIMEOUT_S; status={"channels":{}}
        while self.clock()<deadline:
            event=self._next_event(deadline,generation,link_generation,sent); values=parse_kv(str(event.data))
            if "ch" in values and "role" in values:
                try: status["channels"][int(values["ch"])]=values
                except ValueError: pass
            if "state" in values: status.update(values)
            state_ok=(require_state is None and "state" in status) or status.get("state") in (require_state or set())
            if state_ok and set(status["channels"])=={1,2}: return status
        raise TimeoutError("等待映射回表超时")

    def _wait_spin(self,sent,generation,link_generation,spin,channel,percent):
        deadline=self.clock()+REPLY_TIMEOUT_S
        while self.clock()<deadline:
            values=parse_kv(str(self._next_event(deadline,generation,link_generation,sent).data))
            if values.get("spin")==spin and values.get("ch")==str(channel) and values.get("pct")==str(percent): return values
            if values.get("state")=="spin_rejected": raise RuntimeError("飞控拒绝点转："+values.get("reason","unknown"))
        raise TimeoutError("等待点转状态超时")

    def _next_event(self,deadline,generation,link_generation,sent):
        while True:
            self._check_generation(generation,link_generation)
            remaining=deadline-self.clock()
            if remaining<=0: raise TimeoutError("等待PROPCAL回包超时")
            try: event=self.engine.propcal_events.get(timeout=min(.03,remaining))
            except queue.Empty: continue
            if event.generation==link_generation and event.host_time_s>=sent: return event

    def _stop_window(self,generation,link_generation):
        self._check_generation(generation,link_generation); self._drain(); sent=self.clock()
        self.engine.connection.send_command("PROPCAL SPIN STOP")
        return self._wait_spin(sent,generation,link_generation,"idle",0,0)
    def _send_effect(self,generation,link_generation,command):
        """Serialize old-worker effects against STOP generation changes."""
        with self._lock:
            if (generation!=self._generation or self._closed
                    or not self.engine.connection.is_connected
                    or self.engine.connection.generation!=link_generation):
                raise RuntimeError("操作已取消或飞控已重连")
            return bool(self.engine.connection.send_command(command))
    def stop(self):
        with self._lock:
            if not self._owns: return False
            self._generation+=1; generation=self._generation; link_generation=self.engine.connection.generation; worker=self._worker; self._publish_locked("stopping","正在停止点转")
        # Safety command goes out before waiting for a possibly blocked old worker.
        if (self.engine.connection.is_connected and
                self.engine.connection.generation==link_generation):
            self.engine.connection.send_command("PROPCAL SPIN STOP")
        if worker is not None and worker is not threading.current_thread(): worker.join(timeout=.8)
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            with self._lock:
                self._publish_locked("error","旧映射操作未及时退出，控制租约仍被保留")
            threading.Thread(target=self._reap_worker,args=(worker,generation,link_generation),daemon=True,name="prop-map-reaper").start()
            return False
        try:
            self._confirm_stop(generation,link_generation)
        except Exception as exc:
            with self._lock:
                if generation==self._generation:
                    self._release_locked(); self._publish_locked("error",f"停止确认失败：{exc}")
            return False
        with self._lock:
            if generation!=self._generation or link_generation!=self.engine.connection.generation: return False
            self._release_locked(); self._publish_locked("idle","已停止")
        return True
    def _confirm_stop(self,generation,link_generation):
        self._check_generation(generation,link_generation)
        self._drain(); sent=self.clock()
        if not self.engine.connection.send_command("PROPCAL SPIN STOP"): raise RuntimeError("停止命令未发出")
        return self._wait_spin(sent,generation,link_generation,"idle",0,0)
    def _reap_worker(self,worker,generation,link_generation):
        worker.join()
        stop_error=None
        try: self._confirm_stop(generation,link_generation)
        except Exception as exc: stop_error=exc
        with self._lock:
            if generation!=self._generation: return
            self._release_locked()
            if self._closed: return
            message="旧映射操作已退出并确认停止；请重新查询状态" if stop_error is None else f"旧映射操作已退出，但停止确认失败：{stop_error}"
            self._publish_locked("error",message)
    def close(self):
        self.stop()
        with self._lock: self._closed=True; self._state="closed"
    def _fail(self,generation,link_generation,exc,stop):
        with self._lock:
            if generation!=self._generation: return
            link_changed=link_generation!=self.engine.connection.generation
        if link_changed:
            with self._lock:
                if generation!=self._generation: return
                self._release_locked(); self._publish_locked("error","飞控连接已断开或已重连")
            return
        stop_error=None
        if stop:
            try: self._stop_window(generation,link_generation)
            except Exception as stop_exc:
                stop_error=stop_exc
                if (self.engine.connection.is_connected and
                        self.engine.connection.generation==link_generation):
                    self.engine.connection.send_command("PROPCAL SPIN STOP")
        message=str(exc)
        if stop_error is not None: message+=f"；停止确认失败：{stop_error}"
        with self._lock:
            if generation!=self._generation or link_generation!=self.engine.connection.generation: return
            self._release_locked(); self._publish_locked("error",message)
    def _finish(self,generation,link_generation,state,message,**data):
        with self._lock:
            if generation!=self._generation: return
            if link_generation!=self.engine.connection.generation:
                self._release_locked(); self._publish_locked("error","飞控连接已断开或已重连"); return
            self._release_locked(); self._publish_locked(state,message,**data)
    def _release(self):
        with self._lock:
            self._release_locked()
    def _release_locked(self):
        if not self._owns: return
        self._owns=False; self.engine.release_operation(self.OWNER)
    def _check_generation(self,generation,link_generation):
        with self._lock:
            if generation!=self._generation or self._closed: raise RuntimeError("操作已取消")
        if not self.engine.connection.is_connected or self.engine.connection.generation!=link_generation: raise ConnectionError("飞控连接已断开或已重连")
    def _drain(self):
        while True:
            try: self.engine.propcal_events.get_nowait()
            except queue.Empty: return
