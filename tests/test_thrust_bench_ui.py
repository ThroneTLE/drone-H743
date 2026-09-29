from __future__ import annotations
import gc
import json, queue, subprocess, sys, tkinter as tk
from tkinter import ttk
import pytest
from tools.thrust_bench.ui import ThrustBenchApp

class FakeConnection:
    def __init__(self): self.events=queue.Queue(); self.generation=0; self.is_connected=False; self.disconnected=0; self.last_rate_decision=None
    def disconnect(self): self.disconnected+=1
class FakeScale:
    def __init__(self): self.closed=0
    def close(self): self.closed+=1

def test_real_tk_constructs_plans_and_close_stops_links(tmp_path):
    try: app=ThrustBenchApp(connection=FakeConnection(),scale_transport=FakeScale(),session_root=tmp_path)
    except tk.TclError: pytest.skip("Tk display unavailable")
    app.withdraw(); app.update_idletasks()
    try:
        for name in ("单上桨","单下桨","双桨共增","二维网格","动态上桨阶跃","动态下桨阶跃"):
            app.vars["mode"].set(name); assert app._plan().points
        frames=[]
        def collect(widget):
            for child in widget.winfo_children():
                if isinstance(child,ttk.LabelFrame): frames.append(child)
                collect(child)
        collect(app)
        assert any("模型横轴使用电转速eRPM" in w.cget("text") for w in frames)
        assert app._voltage_layers()==(("高",12.3),("中",11.5),("低",10.8))
        for index in range(510): app._append(f"line-{index}")
        assert int(app.live.index("end-1c").split(".")[0])<=501
        connection, scale=app.connection, app.scale_transport; app.close()
        assert connection.disconnected and scale.closed
    finally:
        try:
            if app.winfo_exists(): app.destroy()
        except tk.TclError: pass


def test_repeated_connect_keeps_the_existing_engine_alive(tmp_path):
    app=ThrustBenchApp(connection=FakeConnection(),scale_transport=FakeScale(),session_root=tmp_path)
    app.withdraw()
    class Engine:
        closed=0
        def close(self): self.closed+=1
    engine=Engine(); app.frame.engine=engine
    try:
        app.connect()
        assert engine.closed==0
        assert "已经连接" in app.status.get()
    finally:
        app.frame.engine=None; app.close()


def test_disconnect_releases_host_even_when_close_paths_raise(tmp_path):
    class BadScale(FakeScale):
        fail=True
        def close(self):
            self.closed+=1
            if self.fail: raise RuntimeError("scale close failed")
    class Bridge:
        released=0
        def release_bench(self): self.released+=1
    app=ThrustBenchApp(connection=FakeConnection(),scale_transport=BadScale(),session_root=tmp_path)
    app.withdraw(); bridge=Bridge(); app.frame.host_bridge=bridge
    from types import SimpleNamespace
    bridge.gui=SimpleNamespace(cal_value_var=tk.StringVar(app),value_var=tk.StringVar(app))
    app.frame.telemetry_text=tk.StringVar(app)
    class Engine:
        def close(self): raise RuntimeError("engine close failed")
    app.frame.engine=Engine()
    try:
        app.frame.disconnect()
        assert bridge.released==1 and "请检查串口状态" in app.status.get()
    finally:
        app.scale_transport.fail=False; app.close()


@pytest.mark.parametrize("width,height", ((1024,768),(1280,800),(1440,900)))
@pytest.mark.parametrize("scaling", (1.0,1.25,1.5))
def test_real_tk_layout_matrix_keeps_critical_controls_reachable(width,height,scaling):
    """`tk scaling` is the absolute pixel-per-point value, set before widgets exist."""
    code=r'''
import json, sys, tkinter as tk
import tools.thrust_bench.ui as ui
width,height,scaling=int(sys.argv[1]),int(sys.argv[2]),float(sys.argv[3])
ui.available_ports=lambda: ()
original=tk.Tk.__init__
def scaled_init(self,*args,**kwargs):
    original(self,*args,**kwargs)
    self.tk.call("tk","scaling",scaling)
tk.Tk.__init__=scaled_init
app=ui.ThrustBenchApp()
app.geometry(f"{width}x{height}+3000+2000")
app.update_idletasks(); app.update()
targets={
  "fc_port":str(app.vars["fc_port"]), "fc_baud":str(app.vars["fc_baud"]),
  "scale_port":str(app.vars["scale_port"]), "scale_baud":str(app.vars["scale_baud"]),
  "voltage_layers":str(app.vars["voltage_layers"]), "mode":str(app.vars["mode"]),
  "minimum":str(app.vars["minimum"]), "maximum":str(app.vars["maximum"]),
  "step":str(app.vars["step"]), "upper_values":str(app.vars["upper_values"]),
  "lower_values":str(app.vars["lower_values"]), "max_pct":str(app.vars["max_pct"]),
      "status":str(app.status),
}
buttons={"start":"确认已调整电量并开始本层","stop":"立即停止","analysis":"离线分析当前会话"}
found={}
def walk(widget):
  for child in widget.winfo_children():
    for option in ("textvariable","variable"):
      try:
        variable=str(child.cget(option))
        for name,target in targets.items():
          if variable==target: found[name]=child
      except tk.TclError: pass
    try:
      text=str(child.cget("text"))
      for name,target in buttons.items():
        if text==target: found[name]=child
    except tk.TclError: pass
    walk(child)
walk(app)
def box(widget):
  return [widget.winfo_rootx()-app.winfo_rootx(),widget.winfo_rooty()-app.winfo_rooty(),widget.winfo_width(),widget.winfo_height()]
boxes={name:box(widget) for name,widget in found.items()}
missing=sorted((set(targets)|set(buttons))-set(found))
clipped=sorted(name for name,(x,y,w,h) in boxes.items() if x<0 or y<0 or x+w>width or y+h>height)
print(json.dumps({"missing":missing,"clipped":clipped,"boxes":boxes,"status_text":app.status.get(),"request":[app.winfo_reqwidth(),app.winfo_reqheight()],"viewport":[width,height],"scaling":scaling}))
app.destroy()
'''
    gc.collect()  # Dispose old Tk objects on the owner thread before pipe readers start.
    result=subprocess.run(
        [sys.executable,"-c",code,str(width),str(height),str(scaling)],
        check=True,capture_output=True,text=True,timeout=30)
    evidence=json.loads(result.stdout.strip().splitlines()[-1])
    assert evidence["missing"]==[],evidence
    assert evidence["clipped"]==[],evidence
    assert evidence["boxes"]["stop"][1]+evidence["boxes"]["stop"][3]<=height
    assert evidence["boxes"]["status"][1]+evidence["boxes"]["status"][3]<=height
    assert evidence["status_text"]=="请选择两个串口并连接"


@pytest.mark.parametrize("value,message", [("", "请选择称重标定 JSON 文件"), ("   ", "请选择称重标定 JSON 文件"), (".", "称重标定路径不是文件"), ("missing-calibration.json", "称重标定路径不是文件")])
def test_connect_rejects_invalid_calibration_before_opening_ports(tmp_path, value, message):
    app=ThrustBenchApp(connection=FakeConnection(),scale_transport=FakeScale(),session_root=tmp_path)
    app.withdraw()
    try:
        app.vars["fc_port"].set("COM_TEST_FC")
        app.vars["scale_port"].set("COM_TEST_SCALE")
        app.vars["cal_file"].set(value)
        app.connect()
        assert message in app.status.get(), app.status.get()
        assert app.engine is None
        assert not list(tmp_path.iterdir())
    finally:
        app.destroy()
