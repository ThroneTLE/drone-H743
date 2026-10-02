"""Exercise actual Tk views rather than checking widget-name strings."""
import math
from collections import deque
from types import SimpleNamespace
import tkinter as tk
import pytest

from tools.sim_xz.app import SimulationApp
from tools.sim_xz.experiments import SimulationEngine
from tools.sim_xz.physics import SimulationState

class OfflineDevice:
    def __init__(self):
        self.engine = SimulationEngine()
        self.connected = False
        self.time_scale = 1.0
    def start(self): pass
    def stop(self): pass
    def reset(self): self.engine.reset()
    def set_kind(self, value): self.engine.set_kind(value)

@pytest.fixture
def view():
    app = SimulationApp(OfflineDevice())
    app.geometry("1320x820+20000+20000")
    app.deiconify()
    app.update()
    yield app
    app._close()

@pytest.mark.slow_ui  # 真面板尺寸/缩放矩阵，慢；默认只在界面文件有改动时跑（tests/conftest.py）
@pytest.mark.parametrize("size", ["1100x740", "1320x820", "1500x960"])
@pytest.mark.parametrize("scaling", [1.0, 1.25, 1.5])
def test_controls_and_five_chart_rows_fit(view, size, scaling):
    view.tk.call("tk", "scaling", scaling)
    view.geometry(size + "+20000+20000")
    samples = tuple(SimulationState(time_s=i/10, pitch_rad=i/100, x_m=i/100) for i in range(31))
    view._ab_result = SimpleNamespace(baseline=samples, tuned=samples)
    view.update()
    view._render()
    view.update()
    for widget in (view.save_a_button, view.run_b_button, view.start_button, view.ab_canvas):
        x=y=0
        node=widget
        while node is not view:
            x+=node.winfo_x()
            y+=node.winfo_y()
            node=node.master
        assert x >= 0 and y >= 0
        assert x+widget.winfo_width() <= view.winfo_width()
        assert y+widget.winfo_height() <= view.winfo_height()
    c=view.ab_canvas
    for item in c.find_all():
        box=c.bbox(item)
        assert box[3] <= c.winfo_height()+2
    text=[c.itemcget(i,"text") for i in c.find_all() if c.type(i)=="text"]
    assert any("俯仰角速度" in t for t in text)
    assert any("Z 高度" in t for t in text)
    assert "A" in text and "B" in text

def test_reset_clears_history_and_paused_refresh_does_not_append(view):
    view._render()
    count=len(view._trajectory)
    view._render()
    assert len(view._trajectory)==count
    view._reset()
    assert not view._trajectory and not view._history
    view.target_x.set("nan")
    view._apply_targets()
    assert "有限正数" in view.status.get()
    view._render()
    assert "有限正数" in view.status.get()

def test_disconnected_start_is_explicit_and_cannot_run(view):
    view._toggle()
    assert not view.device.engine.running
    assert "尚未连接" in view.status.get()

def test_upward_force_draws_upward_and_live_plot_has_units(view):
    view._render()
    arrow=[i for i in view.canvas.find_all() if view.canvas.type(i)=="line"
           and view.canvas.itemcget(i, "arrow")=="last"]
    assert len(arrow)==1
    x0,y0,x1,y1=view.canvas.coords(arrow[0])
    assert y1 < y0
    assert math.isclose(x0,x1,abs_tol=1e-5)
    text=[view.ab_canvas.itemcget(i,"text") for i in view.ab_canvas.find_all()
          if view.ab_canvas.type(i)=="text"]
    assert "实时响应" in text

@pytest.mark.parametrize("pitch,tilt", [(0.0,0.0),(0.2,0.05),(-0.2,0.1)])
def test_vector_annotation_follows_world_force_and_body_is_vertical(view,pitch,tilt):
    from dataclasses import replace
    state=replace(view.device.engine.snapshot(), pitch_rad=pitch, pitch_tilt_rad=tilt)
    view._draw_scene(state)
    arrow=view.canvas.find_withtag("thrust")[0]
    x0,y0,x1,y1=view.canvas.coords(arrow)
    length=math.hypot(x1-x0,y1-y0)
    assert (x1-x0)/length == pytest.approx(math.sin(pitch-tilt))
    assert -(y1-y0)/length == pytest.approx(math.cos(pitch-tilt))
    labels=[view.canvas.itemcget(i,"text") for i in view.canvas.find_all()
            if view.canvas.type(i)=="text"]
    assert any("合推力 T" in t for t in labels)
    assert any("Fx " in t and "Fz " in t for t in labels)
    if pitch==0:
        coords=view.canvas.coords(view.canvas.find_withtag("fuselage")[0])
        assert max(coords[1::2])-min(coords[1::2]) > 2*(max(coords[::2])-min(coords[::2]))
