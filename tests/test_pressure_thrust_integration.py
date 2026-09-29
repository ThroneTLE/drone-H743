from __future__ import annotations
import gc

import json
import os
import subprocess
import sys
import tkinter as tk

import pytest

from tools import pressure_rs485_gui as legacy


@pytest.fixture
def gui(tmp_path, monkeypatch):
    calibration=tmp_path/"pressure_calibration.json"
    calibration.write_text(json.dumps({"points":[{"raw":2190.0,"grams":2181.0}]}),encoding="utf-8")
    monkeypatch.setattr(legacy,"CALIBRATION_FILE",calibration)
    monkeypatch.setattr(legacy.list_ports,"comports",lambda:[])
    try: app=legacy.PressureGui()
    except tk.TclError: pytest.skip("Tk display unavailable")
    app.withdraw(); app.update_idletasks()
    yield app
    try:
        if app.winfo_exists(): app.on_close()
    except tk.TclError:
        pass


def test_single_page_h743_replaces_esp_and_reuses_scale(gui):
    texts=[];classes=[]
    def walk(w):
        for child in w.winfo_children():
            classes.append(child.winfo_class())
            try: texts.append(str(child.cget("text")))
            except tk.TclError: pass
            walk(child)
    walk(gui)
    assert "TNotebook" not in classes
    assert not any("ESP" in t or "旧油门" in t for t in texts)
    assert "连接H743台架" in texts and "应用上下桨油门" in texts
    assert "开始 / 换电继续" in texts and "实验库：选择训练与验证" in texts
    assert gui.thrust_bench_frame.vars["fc_port"] is gui.fc_port_var
    assert gui.thrust_bench_frame.vars["scale_port"] is gui.port_var
    assert gui.calibration_points==[(2190.0,2181.0)]
    assert gui.calibrated_grams(4380)==4362
    assert not hasattr(gui,"open_esp_serial") and not hasattr(gui,"_ident_loop")


def test_original_capture_and_save_work_without_motor_configuration(gui):
    # Offline raw fixture: exercise the real existing button callback and file.
    gui.last_raw_value = 0
    gui.ref_weight_var.set("0")
    gui.capture_calibration_point()
    saved = json.loads(legacy.CALIBRATION_FILE.read_text(encoding="utf-8"))
    assert saved["points"] == [{"raw": 0.0, "grams": 0.0},
                               {"raw": 2190.0, "grams": 2181.0}]
    assert gui.calibrated_grams(1095) == 1090.5
    assert "upper_poles" not in gui.thrust_bench_frame.vars


def test_scale_stop_still_updates_chinese_status(gui):
    gui.status_var.set("正在停止读取…")
    gui.events.put(("stopped",None)); gui._poll_events()
    assert gui.status_var.get()=="读取已停止"


def test_manual_drafts_are_explicit_and_global_stop_also_cancels_plan(gui):
    from types import SimpleNamespace
    import threading
    calls=[]
    frame=gui.thrust_bench_frame
    frame.manual=SimpleNamespace(arm=lambda limit:calls.append(("arm",limit)) or True,
        apply=lambda u,l,h:calls.append(("apply",u,l,h)) or True,
        stop=lambda:calls.append(("manual_stop",)), close=lambda:None)
    frame.engine=SimpleNamespace(cancel=threading.Event(),stop=lambda:calls.append(("engine_stop",)),close=lambda:None)
    frame.mapping=SimpleNamespace(stop=lambda:calls.append(("mapping_stop",)),close=lambda:None)
    frame.manual_upper.set(7);frame.manual_lower.set(3);frame.manual_hold.set("2")
    assert calls==[]
    frame.manual_arm();frame.manual_apply()
    assert calls[:2]==[("arm",20),("apply",7.0,3.0,2.0)]
    gui.global_stop.invoke()
    assert frame.engine.cancel.is_set() and calls[-1]==("engine_stop",)
    assert ("mapping_stop",) in calls and ("manual_stop",) in calls


def test_mapping_choices_only_send_on_explicit_actions(gui):
    from types import SimpleNamespace
    calls=[]; frame=gui.thrust_bench_frame
    frame.mapping=SimpleNamespace(test_channel=lambda *a,**kw:calls.append(("test",a,kw)) or True,
        apply_mapping=lambda data:calls.append(("apply",data)) or True,
        save=lambda:calls.append(("save",)) or True, close=lambda:None)
    frame.mapping_roles[1].set("下桨"); frame.mapping_roles[2].set("上桨")
    frame.mapping_spins[1].set("顺时针"); frame.mapping_spins[2].set("逆时针")
    assert calls==[]
    frame.mapping_test(1)
    assert calls==[] and "固定台架" in frame.mapping_status.get()
    frame.mapping_confirmed.set(True); frame.mapping_test(1)
    assert calls==[("test",(1,5.0,1.5),{"confirmed":True})]
    frame.mapping_apply()
    assert calls[-1]==("apply",{1:{"role":"lower","spin":"cw"},2:{"role":"upper","spin":"ccw"}})
    assert not any(c[0]=="save" for c in calls)
    frame.sample_count=1; frame.mapping_apply()
    assert "重连" in frame.mapping_status.get() and len(calls)==2
    frame.mapping_save(); assert calls[-1]==("save",)


def test_rejection_is_not_hidden_by_idle_and_repeated_arm_explains_apply(gui,monkeypatch):
    from types import SimpleNamespace
    import queue
    frame=gui.thrust_bench_frame
    monkeypatch.setattr(frame,"_refresh_observation",lambda:None)
    monkeypatch.setattr(frame,"after",lambda *a:None)
    frame.engine=SimpleNamespace(ui_events=queue.Queue(),close=lambda:None)
    frame.engine.ui_events.put(SimpleNamespace(kind="rejected",data={"reason":"mapping_unconfirmed"}))
    frame.engine.ui_events.put(SimpleNamespace(kind="stopped_confirmed",data={}))
    frame._poll()
    assert "映射" in frame.status.get() and "上下桨" in frame.status.get()
    frame.manual=SimpleNamespace(state="armed",last_error="",arm=lambda n:False,close=lambda:None)
    frame.manual_arm()
    assert "已经解锁" in frame.status.get() and "应用上下桨油门" in frame.status.get()


def test_board_stream_loss_reason_survives_late_auto_worker_error(gui,monkeypatch):
    from types import SimpleNamespace
    import queue
    from tools.thrust_bench.acquisition import UiEvent
    frame=gui.thrust_bench_frame
    monkeypatch.setattr(frame,"_refresh_observation",lambda:None)
    monkeypatch.setattr(frame,"after",lambda *args:None)
    monkeypatch.setattr(frame,"_get_library_view",lambda:SimpleNamespace(
        last_run_id=None,last_recorded_count=0,refresh=lambda:None))
    frame.engine=SimpleNamespace(ui_events=queue.Queue(),close=lambda:None)
    frame.autocollect=SimpleNamespace(events=queue.Queue(),close=lambda:None)
    message="活动期数据流失效，已停止：upper_erpm,lower_erpm,voltage"
    frame.engine.ui_events.put(UiEvent("error",message))
    frame._poll()
    frame.autocollect.events.put(UiEvent("autocollect_state",{
        "state":"error","message":"飞控控制窗口已因数据流失效停止"}))
    frame._poll()
    assert message in frame.status.get()


def test_start_button_explicitly_resumes_only_a_recovered_auto_run(gui):
    from types import SimpleNamespace
    frame=gui.thrust_bench_frame;calls=[]
    frame.engine=SimpleNamespace(active_operation=None,close=lambda:None)
    frame.store=SimpleNamespace(metadata={})
    frame.autocollect=SimpleNamespace(
        state="waiting_recovery",max_percent=20.0,stop_voltage_v=10.8,
        continue_after_recovery=lambda *,confirmed:calls.append(confirmed) or True,
        close=lambda:None)
    frame.start()
    assert calls==[True]
    assert "继续补缺口" in frame.scan_summary.get()


def test_report_button_shows_summary_and_opens_actual_dashboard(gui,tmp_path,monkeypatch):
    import webbrowser
    frame=gui.thrust_bench_frame
    monkeypatch.setattr(frame,"after",lambda *a:None)
    dashboard=tmp_path/"report.html"; dashboard.write_text("<html>report</html>",encoding="utf-8")
    model=tmp_path/"model.json"; model.write_text(json.dumps({"static":{"sufficiency":{"summary":"还缺独立验证"}}}),encoding="utf-8")
    frame._analysis_running=True
    frame.analysis_events.put(("analysis",{"dashboard":dashboard,"model":model}))
    frame._poll()
    assert not frame._analysis_running and frame.report_summary.get()=="还缺独立验证"
    calls=[]; monkeypatch.setattr(webbrowser,"open",lambda uri:calls.append(uri))
    frame.report_open_button.invoke()
    assert calls==[dashboard.resolve().as_uri()]


def test_repeat_preparation_keeps_samples_and_does_not_start_motors(gui):
    from types import SimpleNamespace
    frame=gui.thrust_bench_frame
    frame.sample_count=123; frame._voltage_layer_index=3
    frame.engine=SimpleNamespace(active_operation="manual",close=lambda:None)
    frame.repeat_voltage_layers(); assert frame._voltage_layer_index==3
    frame.engine.active_operation=None
    frame.repeat_voltage_layers()
    assert frame._voltage_layer_index==0 and frame.sample_count==123
    assert "不会自动启动" in frame.status.get()


def test_active_motor_window_refuses_tare(gui):
    from types import SimpleNamespace
    frame=gui.thrust_bench_frame
    frame.engine=SimpleNamespace(active_operation="manual",close=lambda:None)
    frame.tare()
    assert "先停止上下桨" in frame.status.get()


def test_live_readout_uses_current_mapping_and_clears_stale_values(gui):
    import queue
    from types import SimpleNamespace
    from tools.thrust_bench.acquisition import AcquisitionEngine
    from tools.thrust_bench.records import FcSnapshot
    now=[10.0]
    connection=SimpleNamespace(generation=3,is_connected=True,events=queue.Queue(),
        validate_snapshot_rate=lambda hz:{"offline_test":True},send_command=lambda cmd:True,
        disconnect=lambda:None)
    store=SimpleNamespace(update_metadata=lambda **kw:None,event=lambda *a,**kw:None)
    engine=AcquisitionEngine(connection,None,store,clock=lambda:now[0])
    snapshot=FcSnapshot(1,100,2,2,1,(1100,1100),(14000,28000),(0,0),12.0,999.0,0,0,esc_current_a=(2.0,3.0),esc_current_age_ms=(0,0))
    engine._latest_fc=(snapshot,10.0,3,100);engine._latest_scale=(250.0,10.0)
    frame=gui.thrust_bench_frame;frame.engine=engine
    frame._refresh_observation()
    assert "上桨 28000.0" in frame.telemetry_text.get()
    assert "下桨 14000.0" in frame.telemetry_text.get()
    assert "上桨 3.0 A" in frame.telemetry_text.get()
    assert "下桨 2.0 A" in frame.telemetry_text.get()
    assert "输入功率 60.0" in frame.telemetry_text.get()
    assert gui.cal_value_var.get()=="重量 250.0 克"
    from dataclasses import replace
    engine._latest_fc=(replace(snapshot,esc_current_a=(2.0,None),esc_current_age_ms=(0,None)),10.0,3,100)
    frame._refresh_observation()
    assert "上桨 -- A" in frame.telemetry_text.get() and "下桨 2.0 A" in frame.telemetry_text.get()
    assert "合计 -- A" in frame.telemetry_text.get() and "输入功率 -- W" in frame.telemetry_text.get()
    assert "999" not in frame.telemetry_text.get()
    now[0]=11.0;frame._refresh_observation()
    assert "上桨 --" in frame.telemetry_text.get() and "电压 --" in frame.telemetry_text.get()
    assert gui.cal_value_var.get()=="重量 -- 克"


@pytest.mark.parametrize("confirmed_usb",(False,True))
def test_connect_uses_erpm_and_dshot_without_pole_configuration(gui, tmp_path, monkeypatch, confirmed_usb):
    from tools.thrust_bench import ui
    import queue
    from tools.thrust_bench import manual_control
    class Manual:
        def __init__(self, engine): self.events=queue.Queue()
        def close(self): pass
    monkeypatch.setattr(manual_control,"ManualControl",Manual)
    engines = []
    class Scale:
        def connect(self, *args, **kwargs): pass
        def close(self): pass
    class Fc:
        last_rate_decision = {"test": "offline_fake", "confirmed_stm32_usb_cdc": confirmed_usb}
        def connect(self, *args, **kwargs): pass
        def validate_snapshot_rate(self,hz):
            self.validated_hz=hz
            return {"confirmed_stm32_usb_cdc":confirmed_usb}
        def disconnect(self): pass
    class Engine:
        def __init__(self, connection, cell, store, **kwargs):
            self.load_cell = cell; self.store = store; self.kwargs = kwargs
            self.ui_events = queue.Queue(); self.closed = False
            engines.append(self)
        def start_io(self): pass
        def close(self): self.closed = True
    monkeypatch.setattr(ui, "AcquisitionEngine", Engine)
    frame = gui.thrust_bench_frame
    frame.connection = Fc(); frame.scale_transport = Scale(); frame.session_root = tmp_path
    gui.port_var.set("COM_SCALE"); gui.fc_port_var.set("COM_FC")
    frame.connect()
    assert len(engines) == 1, frame.status.get()
    engine = engines[0]
    assert engine.kwargs["snapshot_hz"]==(30.0 if confirmed_usb else 15.0)
    if confirmed_usb: assert frame.connection.validated_hz==30.0
    assert "upper_pole_pairs" not in engine.kwargs and "lower_pole_pairs" not in engine.kwargs
    assert "current_calibration" not in engine.kwargs
    assert engine.load_cell.grams_from_raw(2190) == 2181
    gui.calibration_points[:] = [(100, 300)]
    assert engine.load_cell.grams_from_raw(2190) == 2181
    metadata = json.loads((frame.store.path / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["scale_calibration"]["kind"] == "legacy_single_point_ratio"
    assert metadata["speed_domain"] == "electrical_erpm"
    assert metadata["current_source"] == "dshot"
    assert "pole_pairs" not in metadata
    frame.connect()
    assert frame.engine is engine and not engine.closed and len(engines) == 1
    frame.disconnect()
    assert engine.closed and not gui.bench_bridge.bench_active



def test_bridge_hands_off_addr_channel_timeout_and_exact_legacy_snapshot(gui):
    class Scale:
        def __init__(self): self.args=None
        def connect(self,*args,**kwargs): self.args=(args,kwargs)
        def close(self): pass
    class Fc:
        def __init__(self): self.args=None
        def connect(self,*args,**kwargs): self.args=(args,kwargs)
        def disconnect(self): pass
    class Frame:
        scale_transport=Scale(); connection=Fc()
    gui.port_var.set("COM_SCALE"); gui.baud_var.set("19200")
    gui.addr_var.set("7"); gui.channel_var.set("3"); gui.timeout_var.set("0.4")
    gui.fc_port_var.set("COM_FC"); gui.fc_baud_var.set("115200")
    cell,metadata=gui.bench_bridge.open_bench(Frame(),15.0)
    assert Frame.scale_transport.args==(("COM_SCALE",19200),{"timeout_s":.4})
    assert Frame.connection.args==(("COM_FC",115200),{"snapshot_hz":15.0})
    assert cell.addr==7 and cell.register==4
    assert cell.calibration.points==((2190.0,2181.0),)
    assert metadata["scale_channel"]==3
    assert metadata["scale_calibration"]["single_point_assumes_zero_origin"] is True
    gui.bench_bridge.release_bench()


def test_h743_session_owns_scale_and_tare_reuses_same_session(gui,monkeypatch):
    shown=[];monkeypatch.setattr(legacy.messagebox,"showinfo",lambda *a:shown.append(a))
    gui.bench_bridge.bench_active=True
    tare=[];monkeypatch.setattr(gui.thrust_bench_frame,"tare",lambda:tare.append(True))
    gui.start_reading();gui.read_once();gui.scan_addr();gui.write_command(1)
    assert len(shown)==4 and gui.worker is None
    gui.write_command(2)
    assert tare==[True]


def test_window_close_shuts_embedded_frame_before_destroy(gui,monkeypatch):
    called=[]
    monkeypatch.setattr(gui.thrust_bench_frame,"shutdown",lambda:called.append("shutdown"))
    gui.on_close()
    assert called==["shutdown"]
    with pytest.raises(tk.TclError): gui.winfo_exists()


def test_window_close_still_destroys_old_root_when_embedded_shutdown_fails(gui,monkeypatch):
    monkeypatch.setattr(gui.thrust_bench_frame,"shutdown",lambda:(_ for _ in ()).throw(RuntimeError("boom")))
    with pytest.raises(RuntimeError,match="boom"): gui.on_close()
    assert gui.stop_event.is_set()
    with pytest.raises(tk.TclError): gui.winfo_exists()


def test_stop_remains_visible_when_content_scrolls(gui):
    gui.geometry("1024x768+3000+2000");gui.deiconify();gui.update()
    gui.main_canvas.yview_moveto(1);gui.update()
    y=gui.global_stop.winfo_rooty()-gui.winfo_rooty()
    assert 0<=y and y+gui.global_stop.winfo_height()<=768


def test_missing_scale_calibration_is_rejected_before_either_port_opens(gui):
    class Scale:
        opened=0; closed=0
        def connect(self,*args,**kwargs): self.opened+=1
        def close(self): self.closed+=1
    class Fc:
        opened=0; disconnected=0; last_rate_decision=None
        def connect(self,*args,**kwargs): self.opened+=1
        def disconnect(self): self.disconnected+=1
    frame=gui.thrust_bench_frame; frame.scale_transport=Scale(); frame.connection=Fc()
    gui.port_var.set("COM_SCALE"); gui.fc_port_var.set("COM_FC")
    gui.calibration_points=[]
    frame.connect()
    assert frame.scale_transport.opened==0 and frame.connection.opened==0
    assert "连接失败" in frame.status.get()


def test_direct_script_import_chain_constructs_without_devices(tmp_path):
    code=("import tkinter as tk; tk.Tk.mainloop=lambda self: None; "
          "import pressure_rs485_gui as p; "
          "p.list_ports.comports=lambda: []; p.CALIBRATION_FILE=__import__('pathlib').Path(r'%s'); "
          "p.main(); tk._default_root.destroy() if tk._default_root else None") % str(tmp_path/"missing.json")
    env=dict(os.environ); env["PYTHONPATH"]=str(legacy.Path(__file__).resolve().parents[1]/"tools")
    gc.collect()  # Dispose old Tk objects on the owner thread before pipe readers start.
    result=subprocess.run([sys.executable,"-c",code],cwd=tmp_path,env=env,capture_output=True,text=True,timeout=15)
    assert result.returncode==0,result.stderr


@pytest.mark.parametrize("width,height",((1024,768),(1280,800),(1440,900)))
@pytest.mark.parametrize("scaling",(1.0,1.25,1.5))
def test_single_page_layout_keeps_h743_controls_and_fixed_stop(
        width,height,scaling,tmp_path):
    code=r'''
import json,sys,tkinter as tk
from pathlib import Path
original=tk.Tk.__init__
scale=float(sys.argv[3])
def init(self,*a,**k): original(self,*a,**k); self.tk.call("tk","scaling",scale)
tk.Tk.__init__=init
from tools import pressure_rs485_gui as p
p.list_ports.comports=lambda:[]; p.CALIBRATION_FILE=Path(sys.argv[4])
app=p.PressureGui(); width,height=int(sys.argv[1]),int(sys.argv[2]); app.geometry(f"{width}x{height}+3000+2000"); app.update_idletasks(); app.update()
def box(w): return [w.winfo_rootx()-app.winfo_rootx(),w.winfo_rooty()-app.winfo_rooty(),w.winfo_width(),w.winfo_height()]
stop_box=box(app.global_stop); status_box=box(app.thrust_bench_frame.status_label)
clipped=[]
def walk(w):
 for child in w.winfo_children():
  if child.winfo_class() in ("TButton","TEntry","TCombobox") and child.winfo_ismapped():
   x,y,wid,hei=box(child)
   if x<0 or x+wid>width: clipped.append([str(child),x,wid])
  walk(child)
walk(app)
app.main_canvas.yview_moveto(1);app.update()
print(json.dumps({"stop":stop_box,"status":status_box,"clipped":clipped,"stop_after_scroll":box(app.global_stop)}));app.on_close()
'''
    calibration=tmp_path/"none.json"
    gc.collect()  # Dispose old Tk objects on the owner thread before pipe readers start.
    result=subprocess.run([sys.executable,"-c",code,str(width),str(height),str(scaling),str(calibration)],cwd=legacy.Path(__file__).resolve().parents[1],capture_output=True,text=True,timeout=20,check=True)
    evidence=json.loads(result.stdout.strip().splitlines()[-1])
    for name in ("stop","status"):
        x,y,w,h=evidence[name]
        assert x>=0 and y>=0 and x+w<=width and y+h<=height,evidence
    assert evidence["clipped"] == [], evidence
    assert evidence["stop_after_scroll"] == evidence["stop"], evidence
