"""Host side of ESC KV.

Writes the AM32 motor-KV setting through the flight controller (`ESC KV <kv> CONFIRM`).
"""
from __future__ import annotations
import queue, threading, time
from tools.panel_lib.proto import parse_kv
from .acquisition import UiEvent

MOTOR_KV=1300  # AEO CRM2413-KV1300; AM32 factory default is 2220.
REPLY_TIMEOUT_S=1.0
SEND_TIMEOUT_S=2.0


class EscKvWriter:
    """One-shot: claim the bench, queue the fixed 15-frame sequence, wait until sent."""
    OWNER="esc_kv"
    def __init__(self,engine,*,kv=MOTOR_KV,events=None,clock=time.monotonic):
        self.engine=engine; self.kv=int(kv); self.events=events or queue.Queue(); self.clock=clock
        self.state="idle"; self.result=None; self._worker=None
    def start(self):
        if self._worker and self._worker.is_alive():return False
        if not self.engine.claim_operation(self.OWNER):return False
        self.state="writing"
        self._worker=threading.Thread(target=self._run,daemon=True,name="tbench-esc-kv");self._worker.start();return True
    def stop(self):pass  # 15 frames take 30 ms; the firmware aborts on any actuator owner.
    def close(self):
        if self._worker and self._worker is not threading.current_thread():self._worker.join(timeout=SEND_TIMEOUT_S+REPLY_TIMEOUT_S)
    def _publish(self,state,message,**data):
        self.state=state
        self.events.put(UiEvent("esc_kv_state",{"state":state,"message":message,**data}))
    def _reply(self,since,deadline):
        while self.clock()<deadline:
            try:event=self.engine.esc_events.get(timeout=.05)
            except queue.Empty:continue
            text=str(event.data)
            if event.host_time_s>=since and text.startswith("ESC KV "):return parse_kv(text)
        return None
    def _run(self):
        try:
            self._publish("writing",f"正在向两路电调写入 KV={self.kv}（电机须静止）")
            while True:
                try:self.engine.esc_events.get_nowait()
                except queue.Empty:break
            since=self.clock()
            if not self.engine.connection.send_command(f"ESC KV {self.kv} CONFIRM"):
                raise RuntimeError("命令未能发送到飞控")
            reply=self._reply(since,self.clock()+REPLY_TIMEOUT_S)
            if reply is None:raise RuntimeError("飞控没有回复；可能是旧固件，请先烧录新固件")
            if reply.get("event")=="rejected":
                reason={"armed":"飞控处于解锁状态","busy":"上一条电调命令还没发完","not_dshot":"当前固件不是 DShot 档",
                        "usage":"命令格式错误","kv_not_representable":"KV 值电调无法表示"}.get(reply.get("reason"),reply.get("reason"))
                raise RuntimeError(f"飞控拒绝：{reason}")
            deadline=self.clock()+SEND_TIMEOUT_S
            while reply.get("state")=="sending" and self.clock()<deadline:
                time.sleep(.1);since=self.clock()
                self.engine.connection.send_command("ESC KV ?")
                reply=self._reply(since,self.clock()+REPLY_TIMEOUT_S) or reply
            self.result=reply
            if reply.get("state")!="done":
                raise RuntimeError(f"写入序列未完成（state={reply.get('state')}，已发 {reply.get('sent')}/{reply.get('frames')} 帧，"
                                   f"原因 {reply.get('reason')}）。请拔插电池让电调复位后重试")
            self._publish("finished",
                f"已向两路电调连续发出 {reply.get('sent')}/{reply.get('frames')} 帧，写入 KV={self.kv}。"
                "电调应“嘀”一声。请拔掉电池再插上（电调只在上电时读取 KV），然后做最大推力测试确认。",result=reply)
        except Exception as exc:
            self._publish("error",f"电调 KV 写入失败：{exc}")
        finally:
            self.engine.release_operation(self.OWNER)
