"""Experiment catalogue and explicit whole-run dataset selection in the scale app."""
from __future__ import annotations

import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from uuid import uuid4


class LibraryView:
    def __init__(self, frame):
        self.frame = frame
        self.window = None
        self.tree = None
        self.rows = {}
        self.events = queue.Queue()
        self.busy = False
        self.group_names = {}
        self.show_deleted = tk.BooleanVar(frame,value=False)
        self.coverage_text = tk.StringVar(frame,value="连接台架后可估算当前电压附近仍缺的转速组合。")
        self.last_run_id = None
        self.last_recorded_count = 0

    @staticmethod
    def _calibration_label(row):
        calibration=(row.get("compatibility_identity") or {}).get("scale_calibration") or {}
        points=calibration.get("points") or []
        if len(points)==1:
            point=points[0]
            try:
                return f"称重原始值 {float(point['raw']):g} → {float(point['grams']):g} 克"
            except (KeyError,TypeError,ValueError): pass
        return f"称重标定点 {len(points)} 个" if points else "称重标定未记录"

    @property
    def root(self):
        from tools.project_paths import THRUST_IDENT_DIR
        return Path(self.frame.session_root or THRUST_IDENT_DIR)

    def _library(self):
        from .experiment_library import ExperimentLibrary
        return ExperimentLibrary(self.root / "experiments.sqlite3")

    def _work(self, operation):
        if self.busy:
            self.frame.status.set("实验库正在处理，请稍候")
            return False
        self.busy = True
        def worker():
            library = None
            try:
                library = self._library()
                result = operation(library)
                self.events.put(("done", result))
            except Exception as exc:
                self.events.put(("error", str(exc)))
            finally:
                if library is not None: library.close()
        threading.Thread(target=worker, daemon=True, name="thrust-experiment-library").start()
        return True

    def refresh(self):
        root = self.root
        show_deleted=self.show_deleted.get()
        voltage=None;metadata=None;maximum=None;speeds=(0.0,0.0)
        if self.frame.engine is not None and self.frame.store is not None:
            observation=self.frame.engine.latest_observation()
            snapshot=observation.snapshot
            voltage=snapshot.voltage_v if snapshot else None
            if snapshot is not None and {snapshot.upper_channel,snapshot.lower_channel}=={1,2}:
                speeds=tuple(snapshot.erpm[channel-1] or 0.0 for channel in (snapshot.upper_channel,snapshot.lower_channel))
            metadata=dict(self.frame.store.metadata)
            try:maximum=int(self.frame.vars["max_pct"].get())
            except ValueError:pass
        def load(library):
            summary = library.import_sessions(root)
            gap=None
            if metadata is not None and voltage is not None and maximum is not None:
                from .autocollect import AutoCollector
                covered=library.covered_points(metadata,include_unassigned=True)
                controller=AutoCollector(self.frame.engine,covered,max_percent=maximum,
                    stop_voltage_v=1.0,max_duration_s=1.0,max_points=1)
                gap={"voltage_v":voltage,**controller.coverage_gap(voltage,*speeds),
                     "included_steady_points":len(covered)}
            return {"rows": library.list_runs(include_deleted=show_deleted),
                    "import_summary": summary,"gap":gap}
        return self._work(load)

    def open(self):
        if self.window is not None and self.window.winfo_exists():
            self.window.lift()
            self.refresh()
            return
        window = self.window = tk.Toplevel(self.frame)
        window.title("推力实验库 · 选择训练与验证")
        window.geometry("1040x610")
        window.minsize(760, 460)
        outer = ttk.Frame(window, padding=12)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="每行是一整轮实验。Ctrl+A 全选；删除从实验库移除，可恢复，原始记录保留。",
                  wraplength=960).pack(anchor="w", pady=(0, 8))
        columns = ("time", "session", "mode", "points", "voltage", "split", "config")
        grid = ttk.Frame(outer); grid.pack(fill="both", expand=True)
        self.tree = ttk.Treeview(grid, columns=columns, show="headings", selectmode="extended", height=13)
        for name, title, width in zip(columns,
                ("采集时间", "实验 / 轮次", "测量模式", "有效点 / 记录行", "实测电压 V", "用途", "测量组"),
                (135, 170, 80, 115, 120, 80, 90)):
            self.tree.heading(name, text=title)
            self.tree.column(name, width=width, minwidth=65)
        scroll = ttk.Scrollbar(grid, orient="vertical", command=self.tree.yview)
        horizontal = ttk.Scrollbar(grid, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        self.tree.grid(row=0, column=0, sticky="nsew"); scroll.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        grid.rowconfigure(0, weight=1); grid.columnconfigure(0, weight=1)
        self.detail = tk.StringVar(window, value="支持 Ctrl / Shift 多选。同一轮只属于一种用途；不同配置需要分开训练。")
        self.tree.bind("<<TreeviewSelect>>", self._selection_changed)
        self.tree.bind("<Control-a>",self._select_all)
        self.tree.bind("<Control-A>",self._select_all)
        window.bind("<Control-a>",self._select_all)
        window.bind("<Control-A>",self._select_all)
        ttk.Label(outer, textvariable=self.detail, wraplength=980).pack(fill="x", pady=8)
        actions = ttk.Frame(outer); actions.pack(fill="x")
        for title, split in (("选中 → 训练集", "train"), ("选中 → 验证集", "validation"), ("选中 → 暂不使用", "excluded")):
            ttk.Button(actions, text=title, command=lambda s=split: self.assign(s)).pack(side="left", padx=(0, 6))
        ttk.Button(actions, text="刷新历史实验", command=self.refresh).pack(side="left")
        ttk.Button(actions,text="删除选中",command=self.delete_selected).pack(side="left",padx=(8,4))
        ttk.Checkbutton(actions,text="显示已删除",variable=self.show_deleted,
                        command=self.refresh).pack(side="left",padx=4)
        ttk.Button(actions,text="恢复选中",command=self.restore_selected).pack(side="left",padx=4)
        ttk.Button(actions, text="训练并生成报告", command=self.train).pack(side="right")
        self.summary = tk.StringVar(window, value="正在读取历史实验…")
        ttk.Label(outer, textvariable=self.summary, wraplength=980).pack(fill="x", pady=10)
        ttk.Label(outer,textvariable=self.coverage_text,wraplength=980).pack(fill="x",pady=(0,6))
        ttk.Label(outer, text="验证集用于比较候选模型；最终可信度还需新的独立复测。没有验证集也能查看训练效果，但会明确标为未验证。",
                  wraplength=980).pack(fill="x")
        self.refresh()

    def _select_all(self,_event=None):
        if self.tree is not None:
            self.tree.selection_set(self.tree.get_children())
        return "break"

    def delete_selected(self):
        ids=tuple(self.tree.selection()) if self.tree is not None else ()
        if not ids:
            self.frame.status.set("请先选中要从实验库移除的轮次");return
        def remove(library):
            library.delete_runs(ids)
            return {"refresh_needed":True}
        self._work(remove)

    def restore_selected(self):
        ids=tuple(self.tree.selection()) if self.tree is not None else ()
        if not ids:
            self.frame.status.set("请先选中要恢复的轮次");return
        def restore(library):
            library.restore_runs(ids)
            return {"refresh_needed":True}
        self._work(restore)

    def _selection_changed(self, _event=None):
        ids = self.tree.selection()
        if len(ids) != 1: return
        row = self.rows.get(ids[0], {})
        warnings = row.get("warnings") or []
        group=self.group_names.get(str(row.get("compatibility_key","")),"未分组")
        self.detail.set(f"轮次 {row.get('run_id', '')}；{group}：{self._calibration_label(row)}。"
                        + ("提示：" + str(warnings) if warnings else "可按整轮选择用途。"))

    def assign(self, split):
        ids = tuple(self.tree.selection()) if self.tree is not None else ()
        if not ids:
            self.frame.status.set("请先在实验库中选中实验行")
            return
        if any(self.rows.get(ident,{}).get("deleted") for ident in ids):
            self.frame.status.set("已删除的实验先点‘恢复选中’，再分配用途")
            return
        def save(library):
            library.set_split(ids, split)
            return {"refresh_needed":True}
        self._work(save)

    def train(self):
        frame = self.frame
        if self.busy or frame._analysis_running or frame._scan_preparing:
            frame.status.set("实验库或模型正在处理，请稍候")
            return
        if frame.engine and frame.engine.active_operation is not None:
            frame.status.set("请先停止试转或扫描，再训练模型")
            return
        selected=[row for row in self.rows.values() if row.get("split") in {"train","validation"}]
        groups={str(row.get("compatibility_key","")) for row in selected}
        if len(groups)>1:
            descriptions=[]
            for key in sorted(groups,key=lambda item:self.group_names.get(item,item)):
                group_rows=[row for row in selected if str(row.get("compatibility_key",""))==key]
                descriptions.append(f"{self.group_names.get(key,'测量组')}（{self._calibration_label(group_rows[0])}，"
                                    f"{len(group_rows)}轮／{sum(int(row.get('steady_point_count',0)) for row in group_rows)}个稳态点）")
            message="已选实验跨两种测量记录："+"；".join(descriptions)+"。请按测量组分别训练；旧记录都保留。"
            frame.status.set(message)
            frame.report_summary.set(message)
            if self.window is not None and self.window.winfo_exists(): self.summary.set(message)
            return
        root = self.root
        from datetime import date
        output = root / "models" / date.today().isoformat() / ("training-" + uuid4().hex[:12])
        frame._analysis_running = True
        frame.report_summary.set("正在用选定实验训练；验证实验不参与参数拟合…")
        frame.report_open_button.configure(state="disabled")
        def train(library):
            from .dataset_report import write_dataset_analysis
            library.import_sessions(root)
            training, validation, metadata = library.load_selection()
            return {"analysis": write_dataset_analysis(training, validation, metadata, output)}
        self._work(train)

    def poll(self):
        for _ in range(20):
            try: kind, result = self.events.get_nowait()
            except queue.Empty: break
            self.busy = False
            if kind == "error":
                self.frame._analysis_running = False
                self.frame.status.set("实验库：" + result)
                self.frame.report_summary.set(result)
                if self.frame._report_path is not None:
                    self.frame.report_open_button.configure(state="normal")
                if self.window is not None and self.window.winfo_exists(): self.summary.set(result)
                continue
            if "analysis" in result:
                self.frame.analysis_events.put(("analysis", result["analysis"]))
            if result.get("refresh_needed"):
                self.refresh()
                continue
            if "rows" in result:
                self.rows = {str(row["id"]): row for row in result["rows"]}
                if self.window is not None and self.window.winfo_exists():
                    selected = self.tree.selection()
                    self.tree.delete(*self.tree.get_children())
                    counts = {key: 0 for key in ("train", "validation", "excluded", "pending", "deleted")}
                    group_names={key:f"组{index+1}" for index,key in enumerate(
                        dict.fromkeys(str(row.get("compatibility_key","")) for row in self.rows.values()))}
                    self.group_names=group_names
                    for ident, row in self.rows.items():
                        split = row.get("split", "excluded")
                        if row.get("deleted"): split="deleted"
                        elif split=="excluded" and not row.get("assignment_explicit",False): split="pending"
                        counts[split] = counts.get(split, 0) + 1
                        lo, hi = row.get("voltage_min"), row.get("voltage_max")
                        voltage = f"{lo:.2f}～{hi:.2f}" if lo is not None and hi is not None else "无有效电压"
                        mode = {"dual": "双桨", "upper": "单上桨", "lower": "单下桨"}.get(row.get("mode"), row.get("mode", "--"))
                        self.tree.insert("", "end", iid=ident, values=(str(row.get("created_at", ""))[:19],
                            f"{str(row.get('session_id', '')).rsplit('/',1)[-1]} / {str(row.get('run_id', ''))[:6]}", mode,
                            f"{row.get('steady_point_count', 0)} / {row.get('sample_count', 0)}", voltage,
                            {"train": "训练", "validation": "验证", "excluded": "暂不使用", "pending":"待分配", "deleted":"已删除"}.get(split, split),
                            group_names[str(row.get("compatibility_key", ""))]))
                    self.tree.selection_set(*(key for key in selected if key in self.rows))
                    group_note="；".join(f"{group_names[key]}：{self._calibration_label(next(row for row in self.rows.values() if str(row.get('compatibility_key',''))==key))}"
                                         for key in group_names)
                    self.summary.set(f"当前显示 {len(self.rows)} 轮；训练 {counts['train']}，验证 {counts['validation']}，待分配 {counts['pending']}，暂不使用 {counts['excluded']}，已删除 {counts['deleted']}。{group_note}")
            if result.get("gap") is not None:
                gap=result["gap"]
                self.coverage_text.set(
                    f"当前约 {gap['voltage_v']:.2f} V：历史已有 {gap['included_steady_points']} 个合格稳态点；"
                    + f"按本次最高油门共 {gap['total']} 个油门格，当前电量段尚缺 {gap['missing']} 个。")
            elif "gap" in result:
                self.coverage_text.set("连接台架后可估算当前电压附近仍缺的转速组合。")
            if self.last_run_id:
                match=next((row for row in self.rows.values() if row.get("run_id")==self.last_run_id),None)
                if match is not None:
                    message=(f"本轮自动采集记录 {self.last_recorded_count} 个点，"
                             f"入库确认 {match.get('steady_point_count',0)} 个可用稳态点；"
                             f"原始记录 {match.get('sample_count',0)} 行。")
                    if result.get("gap") is not None:
                        gap=result["gap"]
                        message+=f"当前约 {gap['voltage_v']:.2f} V 电量段尚缺 {gap['missing']} / {gap['total']} 个油门格。"
                    self.frame.scan_summary.set(message)
                    self.frame.status.set(message)
                    self.last_run_id=None
