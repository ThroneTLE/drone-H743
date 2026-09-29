"""Real original Tk window: explicit dataset choices and simplified scan entry."""
from __future__ import annotations
import time
from types import SimpleNamespace

from test_pressure_thrust_integration import gui
from tools.thrust_bench.library_view import LibraryView


def drain(view):
    deadline=time.monotonic()+2
    while view.busy and time.monotonic()<deadline:
        view.poll(); time.sleep(.005)
    assert not view.busy


def test_library_whole_run_assignment_is_explicit_and_persists_refresh(gui,monkeypatch,tmp_path):
    rows=[dict(id="one",session_id="session",run_id="run-one",created_at="2026-09-22",
               mode="dual",sample_count=100,steady_point_count=10,voltage_min=11.8,
               voltage_max=12.1,split="excluded",compatibility_key="config",warnings=[])]
    assignments=[]
    class Library:
        def import_sessions(self,root): return {"runs":1}
        def list_runs(self,*,include_deleted=False): return rows
        def set_split(self,ids,split):
            assignments.append((ids,split));rows[0]["split"]=split
        def close(self): pass
    monkeypatch.setattr(LibraryView,"_library",lambda self:Library())
    frame=gui.thrust_bench_frame;frame.session_root=tmp_path
    frame.open_library();view=frame.library_view;drain(view)
    assert view.tree.get_children()==("one",) and assignments==[]
    view.tree.selection_set("one");view.assign("validation");drain(view)
    assert assignments==[(("one",),"validation")]
    assert "验证" in view.tree.item("one","values")
    view.refresh();drain(view)
    assert "验证 1" in view.summary.get()
    view.window.destroy()


def test_ctrl_a_selects_all_visible_runs_and_delete_can_be_restored(gui,monkeypatch,tmp_path):
    rows=[dict(id=name,session_id="session",run_id=name,created_at="2026-09-22",
               mode="dual",sample_count=9,steady_point_count=1,voltage_min=11.8,
               voltage_max=12.1,split="excluded",compatibility_key="config",
               warnings=[],deleted=False) for name in ("first","second")]
    class Library:
        def import_sessions(self,root):return {"runs":2}
        def list_runs(self,*,include_deleted=False):
            return [row for row in rows if include_deleted or not row["deleted"]]
        def delete_runs(self,ids):
            for row in rows:
                if row["id"] in ids:row["deleted"]=True
        def restore_runs(self,ids):
            for row in rows:
                if row["id"] in ids:row["deleted"]=False
        def close(self):pass
    monkeypatch.setattr(LibraryView,"_library",lambda self:Library())
    frame=gui.thrust_bench_frame;frame.session_root=tmp_path
    frame.open_library();view=frame.library_view;drain(view)
    assert view.tree.bind("<Control-a>") and view.window.bind("<Control-a>")
    view._select_all()
    assert set(view.tree.selection())=={"first","second"}
    view.delete_selected();drain(view)
    assert view.tree.get_children()==()
    view.show_deleted.set(True);view.refresh();drain(view)
    assert set(view.tree.get_children())=={"first","second"}
    view._select_all()
    view.restore_selected();drain(view)
    assert all(not row["deleted"] for row in rows)
    view.window.destroy()


def test_auto_finish_reports_recorded_vs_confirmed_points_and_remaining_gap(gui):
    frame=gui.thrust_bench_frame
    view=LibraryView(frame)
    view.last_run_id="auto-run"
    view.last_recorded_count=25
    view.events.put(("done",{"rows":[{"id":"one","run_id":"auto-run",
        "steady_point_count":18,"sample_count":997}],
        "gap":{"voltage_v":11.3,"total":9,"missing":4,"included_steady_points":101}}))
    view.poll()
    assert "记录 25" in frame.scan_summary.get()
    assert "入库确认 18" in frame.scan_summary.get()
    assert "尚缺 4 / 9" in frame.scan_summary.get()


def test_default_scan_is_bounded_smart_plan_and_preview_never_starts(gui):
    frame=gui.thrust_bench_frame;sent=[]
    assert frame.smart_mode.get()=="自动补数（换电继续）"
    frame.smart_mode.set("标准采集")
    frame.engine=SimpleNamespace(active_operation=None,run_plan=lambda p,**kw:sent.append((p,kw)),close=lambda:None)
    frame.vars["max_pct"].set("20")
    frame.preview_scan()
    assert sent==[] and "16" in frame.scan_summary.get()
    frame.start()
    plan,args=sent[0]
    assert len(plan.points)==16 and args=={"max_percent":20}
    assert all(p.adaptive and p.voltage_layer_v is None and max(p.upper_percent,p.lower_percent)<=20 for p in plan.points)
    assert not frame.custom_scan.get()


def test_dataset_training_cannot_start_while_motor_operation_active(gui,monkeypatch):
    frame=gui.thrust_bench_frame
    view=LibraryView(frame)
    frame.engine=SimpleNamespace(active_operation="manual",close=lambda:None)
    calls=[];monkeypatch.setattr(view,"_work",lambda fn:calls.append(fn))
    view.train()
    assert calls==[] and "先停止" in frame.status.get()


def test_report_action_opens_dataset_selection_without_current_connection(gui,monkeypatch):
    frame=gui.thrust_bench_frame;calls=[]
    monkeypatch.setattr(frame,"open_library",lambda:calls.append("library"))
    assert frame.engine is None and frame.store is None
    frame.analyze()
    assert calls==["library"]


def test_mixed_historical_scale_groups_explain_which_runs_remain_usable(gui,monkeypatch):
    frame=gui.thrust_bench_frame;view=LibraryView(frame)
    view.rows={
        "new":{"split":"train","compatibility_key":"new","steady_point_count":101,
               "compatibility_identity":{"scale_calibration":{"points":[{"raw":239,"grams":240}]}}},
        "old":{"split":"train","compatibility_key":"old","steady_point_count":2,
               "compatibility_identity":{"scale_calibration":{"points":[{"raw":207,"grams":214}]}}},
    }
    view.group_names={"new":"组1","old":"组2"}
    calls=[];monkeypatch.setattr(view,"_work",lambda fn:calls.append(fn))
    view.train()
    assert calls==[]
    assert "239 → 240" in frame.status.get() and "207 → 214" in frame.status.get()
    assert "101个稳态点" in frame.status.get() and "2个稳态点" in frame.status.get()
    assert "旧记录都保留" in frame.status.get()


def test_stop_cancels_background_auto_preparation_before_any_arm(gui,tmp_path,monkeypatch):
    import threading,queue
    frame=gui.thrust_bench_frame;frame.session_root=tmp_path
    calls=[]
    frame.engine=SimpleNamespace(
        active_operation=None,cancel=threading.Event(),ui_events=queue.Queue(),
        latest_observation=lambda:SimpleNamespace(snapshot=SimpleNamespace(voltage_v=12.0)),
        stop=lambda:calls.append("stop") or True,close=lambda:None)
    frame.store=SimpleNamespace(metadata={"motor_model":"AEO CRM2413-KV1300"})
    monkeypatch.setattr(frame,"after",lambda *args:None)
    monkeypatch.setattr(frame,"_refresh_observation",lambda:None)
    frame.start_autocollect()
    assert frame._auto_preparing
    frame.stop()
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        frame._poll()
        if frame.analysis_events.empty() and not frame._auto_preparing: break
        time.sleep(.01)
    assert frame.autocollect is None and calls and frame.engine.cancel.is_set()


def test_max_thrust_button_uses_typed_limit_and_global_stop_ends_it(gui,monkeypatch):
    import threading
    frame=gui.thrust_bench_frame
    started=[]
    class FakeTest:
        def __init__(self,engine,*,max_percent,stop_voltage_v):
            self.args=(max_percent,stop_voltage_v);self.events=__import__("queue").Queue();self.stopped=False
        def start(self):started.append(self.args);return True
        def stop(self):self.stopped=True
        def close(self):self.stop()
    monkeypatch.setattr("tools.thrust_bench.max_thrust.MaxThrustTest",FakeTest)
    frame.engine=SimpleNamespace(active_operation=None,cancel=threading.Event(),stop=lambda:True,close=lambda:None)
    frame.store=SimpleNamespace(metadata={})
    frame.vars["max_pct"].set("20");frame.stop_voltage.set("10.8")  # Button ignores the scan limit.
    buttons=[]
    def walk(widget):
        for child in widget.winfo_children():
            if child.winfo_class()=="TButton":buttons.append(child.cget("text"))
            walk(child)
    walk(frame)
    assert "最大推力测试（100%）" in buttons and "电调KV设为1300" in buttons
    assert "建立查补表" in buttons and "查补表实测验证" in buttons and "差速实测验证" in buttons
    frame.start_max_thrust()
    assert started==[(100,10.8)] and "100%" in frame.scan_summary.get()
    test=frame.max_thrust
    frame.stop()
    assert test.stopped and frame.engine.cancel.is_set()
    frame.engine.active_operation="autocollect"
    frame.start_max_thrust()
    assert started==[(100,10.8)] and "其他操作" in frame.status.get()


def test_lut_build_and_bench_validation_enable_the_fit_view_button(gui,tmp_path,monkeypatch):
    """Both used to leave "查看拟合效果" greyed out: their pages never reached the button."""
    import queue
    frame=gui.thrust_bench_frame
    monkeypatch.setattr(frame,"after",lambda *args:None)
    frame.report_open_button.configure(state="disabled");frame._report_path=None
    page=tmp_path/"thrust_lut.html";page.write_text("x",encoding="utf-8")
    frame.analysis_events.put(("throttle_model",(str(tmp_path/"thrust_lut.json"),"查补表已建立",str(page))))
    frame._poll()
    assert frame._report_path==page and str(frame.report_open_button.cget("state"))=="normal"
    validation=tmp_path/"validation-r1.html";validation.write_text("x",encoding="utf-8")
    events=queue.Queue();events.put(SimpleNamespace(data={"state":"finished","message":"完成","report":str(validation)}))
    frame.model_check=SimpleNamespace(events=events)
    monkeypatch.setattr(frame,"_get_library_view",lambda:SimpleNamespace(refresh=lambda:None))
    frame._poll()
    assert frame._report_path==validation
    frame.model_check=None
