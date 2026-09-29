"""Tk front end. Worker threads publish queue events and never touch widgets."""
from __future__ import annotations
import json, queue, threading, tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk
from .acquisition import AcquisitionEngine
from .current_sources import dshot_total_current, dshot_input_power
from .connection import FlightControllerConnection, ScaleSerialTransport
from .plans import common_sweep, dynamic_steps, grid, single_prop, with_voltage_layer
from .scale import LoadCell, ScaleCalibration, dip_to_addr
from .session import SessionStore, read_samples

def available_ports():
    try:
        from serial.tools import list_ports
        return tuple(sorted(p.device for p in list_ports.comports()))
    except Exception: return ()

class ThrustBenchFrame(ttk.Frame):
    def __init__(self, master, *, connection=None, scale_transport=None,
                 session_root=None, host_bridge=None):
        super().__init__(master)
        self.host_bridge=host_bridge; self.analysis_events=queue.Queue(); self.fc_events=queue.Queue()
        self.connection=connection or FlightControllerConnection(self.fc_events)
        self.scale_transport=scale_transport or ScaleSerialTransport(); self.session_root=session_root
        self.engine=self.store=None; self.manual=None; self.mapping=None; self._control_error=""; self._scale_calibration=None; self._host_metadata={}; self.sample_count=0; self.status=tk.StringVar(master=self,value="请选择两个串口并连接")
        defaults={"fc_port":"","scale_port":"","fc_baud":"115200","scale_baud":"9600","dip":"1000",
          "cal_file":"","snapshot_hz":"15","prop":"两片桨均安装","spacing":"",
"mode":"双桨共增","minimum":"0","maximum":"20","step":"5",
          "upper_values":"0,5,10,15,20","lower_values":"0,5,10,15,20","delta":"3","repeats":"3","max_pct":"20",
          "voltage_layers":"高:12.3,中:11.5,低:10.8"}
        self._voltage_layer_index=0
        self.vars={k:tk.StringVar(master=self,value=v) for k,v in defaults.items()}
        if host_bridge is not None:
            self.vars.update(host_bridge.shared_variables())
            self.vars["cal_file"].set("原窗口内存标定快照")
        self._closed=False; self._poll_after_id=None; self._analysis_running=False; self._report_path=None
        self.library_view=None; self._scan_preparing=False; self._prepared_scan=None
        self.autocollect=None; self._auto_preparing=False; self._auto_ticket=0
        self.max_thrust=None; self.esc_kv=None; self.model_check=None
        self._active_smart=False
        self._build(); self._poll_after_id=self.after(40,self._poll)
    def _build(self):
        if self.host_bridge is not None:
            from .integrated_view import build_integrated_view
            build_integrated_view(self)
            return
        self._build_standalone()

    def _build_standalone(self):
        outer=ttk.Frame(self,padding=10); outer.pack(fill="both",expand=True)
        conn=ttk.LabelFrame(outer,text="连接（继承原窗口设置）" if self.host_bridge else "连接（必须明确选择）",padding=8); conn.pack(fill="x")
        for col,(label,key) in enumerate((("飞控串口","fc_port"),("飞控波特率","fc_baud"),("称重串口","scale_port"),("称重波特率","scale_baud"))):
            ttk.Label(conn,text=label).grid(row=0,column=col*2,sticky="w")
            widget=ttk.Combobox(conn,textvariable=self.vars[key],width=12,state="readonly" if self.host_bridge else "normal") if "port" in key else ttk.Entry(conn,textvariable=self.vars[key],width=9,state="readonly" if self.host_bridge else "normal")
            widget.grid(row=0,column=col*2+1,padx=(3,10));
            if "port" in key: widget["values"]=available_ports()
        ttk.Button(conn,text="连接",command=self.connect).grid(row=1,column=0,pady=6,sticky="w")
        ttk.Button(conn,text="断开",command=self.disconnect).grid(row=1,column=1,sticky="w")
        cfg=ttk.LabelFrame(outer,text="装配、标定与电流来源",padding=8); cfg.pack(fill="x",pady=8)
        entries=(("采样频率（Hz）","snapshot_hz"),("桨/安装","prop"),("桨间距","spacing"),("称重拨码","dip"),("称重标定来源","cal_file"))
        for i,(label,key) in enumerate(entries):
            r,c=divmod(i,4); ttk.Label(cfg,text=label).grid(row=r*2,column=c,sticky="w"); ttk.Entry(cfg,textvariable=self.vars[key],width=20,state="readonly" if self.host_bridge is not None and key=="cal_file" else "normal").grid(row=r*2+1,column=c,padx=(0,10),sticky="ew")
        if self.host_bridge is None:
            ttk.Button(cfg,text="选标定文件",command=self._choose_cal).grid(row=3,column=3,sticky="w")
        else:
            ttk.Label(cfg,text="使用原窗口软件砝码点快照；单点标定建议补空载点。",wraplength=360).grid(row=3,column=3,sticky="w")
        ttk.Button(cfg,text="空载去皮",command=self.tare).grid(row=6,column=3,sticky="w")
        ttk.Label(cfg,text="转速直接使用DShot电转速eRPM；电流仅使用DShot分路回传，缺失不回退板总电流。",wraplength=800).grid(row=4,column=0,columnspan=4,sticky="w")
        plan=ttk.LabelFrame(outer,text="扫描计划（二维网格是指令网格，模型横轴使用电转速eRPM）",padding=8); plan.pack(fill="x")
        ttk.Combobox(plan,textvariable=self.vars["mode"],state="readonly",values=("单上桨","单下桨","双桨共增","二维网格","动态上桨阶跃","动态下桨阶跃"),width=18).grid(row=0,column=0)
        for i,(label,key) in enumerate((("最小%","minimum"),("最大%","maximum"),("步长%","step"),("上轴列表","upper_values"),("下轴列表","lower_values"),("小阶跃%","delta"),("重复","repeats"),("本次油门上限%","max_pct"))):
            ttk.Label(plan,text=label).grid(row=1+(i//4)*2,column=i%4,sticky="w"); ttk.Entry(plan,textvariable=self.vars[key],width=18).grid(row=2+(i//4)*2,column=i%4,padx=(0,8),sticky="ew")
        ttk.Label(plan,text="电量层标签:目标中心V").grid(row=5,column=0,sticky="w")
        ttk.Entry(plan,textvariable=self.vars["voltage_layers"],width=45).grid(row=6,column=0,columnspan=3,sticky="ew")
        ttk.Label(plan,text="每层必须人工调整/更换电池后再次点击；目标中心不是实测电压。",wraplength=420).grid(row=6,column=3,sticky="w")
        actions=ttk.Frame(outer); actions.pack(fill="x",pady=8)
        ttk.Button(actions,text="确认已调整电量并开始本层",command=self.start).pack(side="left")
        ttk.Button(actions,text="立即停止",command=self.stop).pack(side="left",padx=6)
        ttk.Button(actions,text="离线分析当前会话",command=self.analyze).pack(side="left")
        self.status_label=ttk.Label(outer,textvariable=self.status,wraplength=930)
        self.status_label.pack(side="bottom",fill="x",pady=(6,0))
        self.live=tk.Text(outer,height=13,state="disabled"); self.live.pack(fill="both",expand=True)
    def _choose_cal(self):
        chosen=filedialog.askopenfilename(filetypes=(("JSON","*.json"),("全部","*")))
        if chosen: self.vars["cal_file"].set(chosen)
    def _metadata(self):
        return {"tool":"thrust_bench","plan_kind":self.smart_mode.get() if self.host_bridge and not self.custom_scan.get() else self.vars["mode"].get(),
                "two_propellers_installed":True,"motor_model":"AEO CRM2413-KV1300",
                "prop_installation":self.vars["prop"].get(),"prop_spacing":self.vars["spacing"].get(),
                "speed_domain":"electrical_erpm","speed_source":"dshot_erpm",
                "current_source":"dshot","esc_current_calibrated":False,
                "current_resolution_a":1.0,"current_resolution_source":"TBENCH v1 whole-amp ESC current fields",
                "snapshot_hz_requested":float(self.vars["snapshot_hz"].get()),
                "fc_link_rate":self.connection.last_rate_decision,
                "channel_mapping_source":"awaiting_firmware_snapshot",
                "non_driven_propeller":"unknown_or_windmilling",
                "scale_calibration_file":self.vars["cal_file"].get(),
                "scale_calibration":self._scale_calibration.to_dict() if self._scale_calibration else None,
                "notes":"电转速不除极对数；板总电流仅诊断，不用于电流/功耗拟合；双桨保持安装。",
                **self._host_metadata}
    def connect(self):
        if self.engine is not None:
            self.status.set("实测辨识已经连接，请先断开当前会话")
            return
        try:
            fc=self.vars["fc_port"].get().strip(); scale=self.vars["scale_port"].get().strip()
            if not fc or not scale or fc.casefold()==scale.casefold(): raise ValueError("必须选择两个不同的明确串口")
            auto_rate=self.host_bridge is not None and self.auto_rate.get()
            requested_hz=15.0 if auto_rate else float(self.vars["snapshot_hz"].get())
            calibration=None
            if self.host_bridge is None:
                calibration_text=self.vars["cal_file"].get().strip()
                if not calibration_text: raise ValueError("请选择称重标定 JSON 文件（点击“选标定文件”）")
                calibration_path=Path(calibration_text)
                if not calibration_path.is_file(): raise ValueError(f"称重标定路径不是文件或文件不存在：{calibration_path}")
                calibration=ScaleCalibration.load(calibration_path)
            if self.host_bridge is None:
                self._scale_calibration=calibration; self.scale_transport.connect(scale,int(self.vars["scale_baud"].get()))
                cell=LoadCell(self.scale_transport,dip_to_addr(self.vars["dip"].get()),calibration); self.connection.connect(fc,int(self.vars["fc_baud"].get()),snapshot_hz=requested_hz)
            else:
                cell,self._host_metadata=self.host_bridge.open_bench(self,requested_hz)
                self._scale_calibration=cell.calibration
            if auto_rate:
                if (self.connection.last_rate_decision or {}).get("confirmed_stm32_usb_cdc"):
                    requested_hz=30.0
                    self.connection.validate_snapshot_rate(requested_hz)
                self.vars["snapshot_hz"].set(f"{requested_hz:g}")
            self.store=SessionStore(self._metadata(),root=self.session_root); self.engine=AcquisitionEngine(self.connection,cell,self.store,snapshot_hz=requested_hz); self.engine.start_io()
            if self.host_bridge is not None:
                from .manual_control import ManualControl
                self.manual=ManualControl(self.engine)
                from .mapping_control import MappingControl
                self.mapping=MappingControl(self.engine)
            self.sample_count=0; self._control_error=""
            self._prepared_scan=None
            self._voltage_layer_index=0
            self.status.set(f"已连接；新会话 {self.store.path}")
        except Exception as exc:
            message=f"连接失败：{exc}"; self.disconnect(); self.status.set(message)
    def disconnect(self):
        self._prepared_scan=None
        self._auto_ticket+=1; self._auto_preparing=False
        error=None
        if self.autocollect is not None:
            try: self.autocollect.close()
            except Exception as exc: error=exc
            finally: self.autocollect=None
        if self.max_thrust is not None:
            try: self.max_thrust.close()
            except Exception as exc: error=exc
            finally: self.max_thrust=None
        if self.esc_kv is not None:
            try: self.esc_kv.close()
            except Exception as exc: error=exc
            finally: self.esc_kv=None
        if self.model_check is not None:
            try: self.model_check.close()
            except Exception as exc: error=exc
            finally: self.model_check=None
        if self.mapping is not None:
            try: self.mapping.close()
            except Exception as exc: error=exc
            finally: self.mapping=None
        if self.manual is not None:
            try: self.manual.close()
            except Exception as exc: error=error or exc
            finally: self.manual=None
        engine,self.engine=self.engine,None
        try:
            if engine: engine.close()
            else: self.connection.disconnect()
        except Exception as exc:
            error=error or exc
        finally:
            try:
                self.scale_transport.close()
            except Exception as exc:
                error=error or exc
            finally:
                if self.host_bridge is not None:
                    self.host_bridge.release_bench()
                    self.telemetry_text.set("飞控未连接；转速、电压和电流 --")
                    if hasattr(self,"raw_channel_text"): self.raw_channel_text.set("通道1：-- eRPM    通道2：-- eRPM")
                    if hasattr(self,"mapping_confirmed"): self.mapping_confirmed.set(False)
                    self.host_bridge.gui.cal_value_var.set("重量 -- 克")
                    self.host_bridge.gui.value_var.set("原始值 --")
                self.status.set("已断开；不会自动恢复解锁或补发指令" if error is None else f"断开时发生错误，请检查串口状态：{error}")
    def _floats(self,key): return tuple(float(x.strip()) for x in self.vars[key].get().split(",") if x.strip())
    def _plan(self):
        mode=self.vars["mode"].get(); lo=float(self.vars["minimum"].get()); hi=float(self.vars["maximum"].get()); step=float(self.vars["step"].get())
        if mode=="单上桨": return single_prop("upper",minimum=lo,maximum=hi,step=step)
        if mode=="单下桨": return single_prop("lower",minimum=lo,maximum=hi,step=step)
        if mode=="双桨共增": return common_sweep(minimum=lo,maximum=hi,step=step)
        if mode=="二维网格": return grid(upper_values=self._floats("upper_values"),lower_values=self._floats("lower_values"))
        axis="upper" if "上桨" in mode else "lower"; return dynamic_steps(baseline=(lo,lo),axis=axis,delta=float(self.vars["delta"].get()),repeats=int(self.vars["repeats"].get()))
    def _voltage_layers(self):
        layers=[]
        for item in self.vars["voltage_layers"].get().split(","):
            item=item.strip()
            if not item: continue
            if ":" not in item: raise ValueError("电量层格式应为 高:12.3,中:11.5,低:10.8")
            label,target=item.split(":",1); layers.append((label.strip(),float(target)))
        return tuple(layers)
    def start(self):
        try:
            if self.engine is None: raise RuntimeError("尚未连接")
            if self._analysis_running: raise RuntimeError("正在生成报告，请稍后开始扫描")
            if self._scan_preparing or self._auto_preparing: raise RuntimeError("正在准备采集计划，请稍候")
            if self.host_bridge is not None and not self.custom_scan.get() and self.smart_mode.get()=="自动补数（换电继续）":
                self._control_error=""
                self.start_autocollect(); return
            self._control_error=""
            maximum=int(self.vars["max_pct"].get())
            if self.host_bridge is not None and not self.custom_scan.get():
                kind={"快速检查":"quick","标准采集":"standard","补充测量":"supplement"}[self.smart_mode.get()]
                from .smart_scan import build_smart_plan
                if kind=="supplement":
                    if self._prepared_scan is None or self._prepared_scan[:2]!=(kind,maximum):
                        self.preview_scan(); return
                    from .smart_scan import VOLTAGE_TOLERANCE_V
                    current=self.engine.latest_observation().snapshot
                    if (current is None or current.voltage_v is None or
                            abs(current.voltage_v-self._prepared_scan[3])>VOLTAGE_TOLERANCE_V):
                        self.preview_scan(); return
                    plan=self._prepared_scan[2]
                else: plan=build_smart_plan(kind,maximum)
                if not plan.points: raise RuntimeError("当前补测计划没有待测点；可以选择标准采集做独立复测")
                self._active_smart=True
                self.scan_summary.set(f"正在记录 {len(plan.points)} 个目标；以实际电压入库，稳定后收点。")
                self.engine.run_plan(plan,max_percent=maximum)
                self._prepared_scan=None
                return
            self._active_smart=False; plan=self._plan()
            if plan.max_percent>maximum: raise ValueError("计划油门超过本次解锁上限")
            layers=self._voltage_layers()
            if layers:
                if self._voltage_layer_index>=len(layers): raise RuntimeError("全部电量层已完成；修改层配置或重新连接可开始新一轮")
                label,target=layers[self._voltage_layer_index]
                plan=with_voltage_layer(plan,label=label,target_v=target)
                self.status.set(f"开始 {label} 电量层（目标中心 {target:g} V，仅标签；实际范围取带载回读）")
            else: self.status.set("开始未分层计划；结果只覆盖本次实际带载电压范围")
            self.engine.run_plan(plan,max_percent=maximum)
        except Exception as exc: self.status.set(f"无法开始：{exc}")

    def _auto_config(self):
        maximum=int(self.vars["max_pct"].get())
        stop_voltage=float(self.stop_voltage.get())
        duration=float(self.auto_minutes.get())*60.0
        points=int(self.auto_points.get())
        if not 1<=maximum<=100: raise ValueError("本次最高油门须在1～100%")
        if not 10.5<=stop_voltage<=12.0: raise ValueError("3S换电电压（静置等效）须在10.5～12.0 V")
        if not 0<duration<=3600: raise ValueError("每轮最长时间须在0～60分钟")
        if not 1<=points<=5000: raise ValueError("每轮最多点数须在1～5000")
        return maximum,stop_voltage,duration,points

    def start_max_thrust(self):
        """Both rotors ramp to 100 % (the button says so), record steady thrust, then STOP."""
        try:
            if self.engine is None or self.store is None: raise RuntimeError("请先连接H743台架")
            if self._analysis_running or self._scan_preparing or self._auto_preparing:
                raise RuntimeError("正在准备或生成报告，请稍候")
            if self.engine.active_operation is not None: raise RuntimeError("台架正在执行其他操作，请先停止")
            _maximum,stop_voltage,_duration,_points=self._auto_config()
            maximum=100
            from .max_thrust import MaxThrustTest
            controller=MaxThrustTest(self.engine,max_percent=maximum,stop_voltage_v=stop_voltage)
            if self.max_thrust is not None: self.max_thrust.close()
            if not controller.start(): raise RuntimeError("当前电压接近换电阈值或台架被占用")
            self.max_thrust=controller;self._control_error=""
            self.scan_summary.set(f"最大推力测试：双桨升到 {maximum}% 并记录稳态推力，约5～8秒后自动停止；随时可点立即停止。")
        except Exception as exc: self.status.set(f"无法开始最大推力测试：{exc}")

    def write_esc_kv(self):
        """AM32 caps duty by its stored KV (default 2220); write the motor's real KV once."""
        try:
            if self.engine is None: raise RuntimeError("请先连接H743台架")
            if self.engine.active_operation is not None: raise RuntimeError("台架正在执行其他操作，请先停止")
            from .esc_kv import EscKvWriter
            writer=EscKvWriter(self.engine)
            if not writer.start(): raise RuntimeError("台架被占用")
            self.esc_kv=writer
        except Exception as exc: self.status.set(f"无法写入电调KV：{exc}")

    def _thrust_root(self):
        from tools import project_paths
        return Path(self.session_root) if self.session_root else project_paths.THRUST_IDENT_DIR

    def build_throttle_model(self):
        """Lookup tables from post-KV sessions with a verified scale zero."""
        if self._analysis_running: self.status.set("正在处理模型，请稍候"); return
        root=self._thrust_root();self._analysis_running=True
        self.scan_summary.set("正在用 KV 修改后的数据建立查补表…")
        def work():
            try:
                from . import throttle_model, thrust_lut
                points,sources,rejected=throttle_model.load_points(root)
                lut=thrust_lut.build(points,sources,rejected)
                path=thrust_lut.save(lut,root)
                thrust_lut.write_figure(lut,points,path.parent)
                from .result_pages import lut_page
                page=lut_page(lut,points,path.parent)
                active=thrust_lut.current(root)
                note=(f"这是候选表 {lut['model_id']}；飞控当前用 {active[0]['model_id']}。验证通过后运行 "
                      "python -m tools.thrust_bench.thrust_lut_export 并烧录才会生效。" if active else "")
                self.analysis_events.put(("throttle_model",(str(path),thrust_lut.summary(lut)+note,str(page))))
            except Exception as exc: self.analysis_events.put(("throttle_model_error",str(exc)))
        threading.Thread(target=work,daemon=True,name="tbench-throttle-model").start()

    def start_model_validation(self):
        """Invert the latest lookup table, command it, measure, compare (independent test run)."""
        try:
            if self.engine is None or self.store is None: raise RuntimeError("请先连接H743台架")
            if self._analysis_running: raise RuntimeError("正在处理模型，请稍候")
            if self.engine.active_operation is not None: raise RuntimeError("台架正在执行其他操作，请先停止")
            _maximum,stop_voltage,_duration,_points=self._auto_config()
            from . import thrust_lut
            from .model_validation import ModelValidationTest
            found=thrust_lut.latest(self._thrust_root())
            if found is None: raise RuntimeError("还没有查补表，请先点“建立查补表”")
            model,path=found
            controller=ModelValidationTest(self.engine,model,path,stop_voltage_v=stop_voltage,backend=thrust_lut)
            if self.model_check is not None: self.model_check.close()
            if not controller.start(): raise RuntimeError("当前电压接近换电阈值或台架被占用")
            self.model_check=controller;self._control_error=""
            self.scan_summary.set(f"查补表实测验证：用查补表 {model.get('model_id')} 反推油门，逐个实测 10 个目标推力…")
        except Exception as exc: self.status.set(f"无法开始实测验证：{exc}")

    def start_split_validation(self):
        """Yaw-like splits: old per-rotor vs new pair allocation, measured back to back (A/B)."""
        try:
            if self.engine is None or self.store is None: raise RuntimeError("请先连接H743台架")
            if self._analysis_running: raise RuntimeError("正在处理模型，请稍候")
            if self.engine.active_operation is not None: raise RuntimeError("台架正在执行其他操作，请先停止")
            _maximum,stop_voltage,_duration,_points=self._auto_config()
            from . import thrust_lut
            from .model_validation import SplitValidationTest
            found=thrust_lut.latest(self._thrust_root())
            if found is None: raise RuntimeError("还没有查补表，请先点“建立查补表”")
            model,path=found
            controller=SplitValidationTest(self.engine,model,path,stop_voltage_v=stop_voltage)
            if self.model_check is not None: self.model_check.close()
            if not controller.start(): raise RuntimeError("当前电压接近换电阈值或台架被占用")
            self.model_check=controller;self._control_error=""
            self.scan_summary.set("差速实测验证：12 组上下分配，每组新旧分配各测一次（约 3 分钟）…")
        except Exception as exc: self.status.set(f"无法开始差速验证：{exc}")

    def start_autocollect(self, *, preview_only=False):
        try:
            if self.engine is None or self.store is None: raise RuntimeError("请先连接H743台架")
            maximum,stop_voltage,duration,points=self._auto_config()
            if (not preview_only and self.autocollect is not None and
                    self.autocollect.state=="waiting_battery"):
                if (maximum!=self.autocollect.max_percent or
                        stop_voltage!=self.autocollect.stop_voltage_v):
                    raise RuntimeError("本轮上限或换电阈值已修改；请先停止旧轮，再重新开始")
                if not self.autocollect.continue_after_battery_change(confirmed=True):
                    raise RuntimeError("换电后尚未达到安全继续条件；请检查实测电压")
                self.scan_summary.set("新电池已确认，正在继续补齐缺少的转速和电压点…")
                return
            if (not preview_only and self.autocollect is not None and
                    self.autocollect.state=="waiting_recovery"):
                if (maximum!=self.autocollect.max_percent or
                        stop_voltage!=self.autocollect.stop_voltage_v):
                    raise RuntimeError("本轮上限或换电阈值已修改；请停止旧轮后重新开始")
                if not self.autocollect.continue_after_recovery(confirmed=True):
                    raise RuntimeError("旧操作尚未退出，请等停止确认后重试")
                self.scan_summary.set("数据已恢复；按当前电池电压继续补缺口…")
                return
            if self.engine.active_operation is not None: raise RuntimeError("台架正在执行其他操作，请先停止")
            observation=self.engine.latest_observation()
            voltage=observation.snapshot.voltage_v if observation.snapshot else None
            if voltage is None: raise RuntimeError("没有新鲜带载电压，无法启动自动采集")
            if voltage<=stop_voltage+0.3: raise RuntimeError("当前电压太接近换电阈值，请先更换电池")
            self._auto_ticket+=1;ticket=self._auto_ticket;engine=self.engine
            link_generation=self.connection.generation
            metadata=dict(self.store.metadata);root=self._get_library_view().root
            self._auto_preparing=True
            self.scan_summary.set("正在检查同装配历史数据，选择缺少的转速与电压工况…")
            def prepare():
                from .experiment_library import ExperimentLibrary
                library=None
                try:
                    library=ExperimentLibrary(root/"experiments.sqlite3")
                    library.import_sessions(root)
                    covered=library.covered_points(metadata,include_unassigned=True)
                    self.analysis_events.put(("autocollect_ready",(ticket,engine,link_generation,
                        covered,maximum,stop_voltage,duration,points,preview_only)))
                except Exception as exc: self.analysis_events.put(("autocollect_error",(ticket,str(exc))))
                finally:
                    if library is not None: library.close()
            threading.Thread(target=prepare,daemon=True,name="tbench-auto-prepare").start()
        except Exception as exc: self.status.set(f"无法自动采集：{exc}")

    def _get_library_view(self):
        if self.library_view is None:
            from .library_view import LibraryView
            self.library_view=LibraryView(self)
        return self.library_view

    def open_library(self):
        self._get_library_view().open()

    def preview_scan(self):
        try:
            if self._scan_preparing: return
            if self._analysis_running: raise RuntimeError("正在训练模型，请稍后准备扫描")
            if self.host_bridge is not None and not self.custom_scan.get() and self.smart_mode.get()=="自动补数（换电继续）":
                self.start_autocollect(preview_only=True); return
            if self.engine and self.engine.active_operation is not None: raise RuntimeError("请先停止当前操作，再准备计划")
            if self.custom_scan.get():
                plan=self._plan()
                self.scan_summary.set(f"自定义计划：{len(plan.points)} 个目标，最大 {plan.max_percent:g}%；点击开始才输出。")
                return
            kind={"快速检查":"quick","标准采集":"standard","补充测量":"supplement"}[self.smart_mode.get()]
            maximum=int(self.vars["max_pct"].get())
            from .smart_scan import build_smart_plan
            if kind!="supplement":
                plan=build_smart_plan(kind,maximum)
                self._prepared_scan=(kind,maximum,plan)
                self.scan_summary.set(f"{len(plan.points)} 个同速/差速目标，最高 {plan.max_percent:g}%；确认台架后点击开始。")
                return
            if self.engine is None: raise RuntimeError("补测需要先连接，读取当前实际电压")
            observation=self.engine.latest_observation()
            voltage=observation.snapshot.voltage_v if observation.snapshot else None
            if voltage is None: raise RuntimeError("当前没有新鲜电压，暂时不能准备补测")
            metadata=dict(self.store.metadata)
            root=self._get_library_view().root
            self._scan_preparing=True
            def prepare():
                from .experiment_library import ExperimentLibrary
                library=None
                try:
                    library=ExperimentLibrary(root/"experiments.sqlite3")
                    library.import_sessions(root)
                    points=library.covered_points(metadata)
                    plan=build_smart_plan(kind,maximum,points,current_voltage_v=voltage)
                    self.analysis_events.put(("smart_plan",(kind,maximum,plan,voltage)))
                except Exception as exc: self.analysis_events.put(("smart_plan_error",str(exc)))
                finally:
                    if library is not None: library.close()
            threading.Thread(target=prepare,daemon=True).start()
            self.scan_summary.set("正在对照当前电压和历史有效数据，准备补测计划…")
        except Exception as exc: self.status.set(f"无法准备扫描：{exc}")
    def _mapping_choices(self):
        return tuple((self.mapping_roles[ch].get(),self.mapping_spins[ch].get()) for ch in (1,2))

    def mapping_read(self):
        try:
            if self.mapping is None: raise RuntimeError("请先连接H743台架")
            self._mapping_read_draft=self._mapping_choices()
            if not self.mapping.query(): raise RuntimeError("当前有控制操作，请先停止再读取映射")
        except Exception as exc:
            self._mapping_read_draft=None; self.mapping_status.set(str(exc))

    def mapping_test(self,channel):
        try:
            if self._analysis_running: raise RuntimeError("正在生成报告，请稍后试转")
            if self.mapping is None: raise RuntimeError("请先连接H743台架")
            if not self.mapping_confirmed.get(): raise ValueError("请先确认已固定台架并清场")
            if not self.mapping.test_channel(channel,float(self.mapping_percent.get()),float(self.mapping_duration.get()),confirmed=True):
                raise RuntimeError("当前有控制操作，请先停止再测试通道")
        except Exception as exc: self.mapping_status.set(str(exc))

    def mapping_apply(self):
        try:
            if self.mapping is None: raise RuntimeError("请先连接H743台架")
            if self.sample_count: raise RuntimeError("本会话已有扫描数据；请断开重连后再修改映射，避免混合两种配置")
            roles={"上桨":"upper","下桨":"lower"};spins={"顺时针":"cw","逆时针":"ccw"}
            selected={}
            for ch in (1,2):
                role=roles.get(self.mapping_roles[ch].get());spin=spins.get(self.mapping_spins[ch].get())
                if role is None or spin is None: raise ValueError("请根据实际观察，为两路都选择上下桨和俯视旋向")
                selected[ch]={"role":role,"spin":spin}
            if not self.mapping.apply_mapping(selected): raise RuntimeError("当前有控制操作，请先停止再提交映射")
        except Exception as exc: self.mapping_status.set(str(exc))

    def mapping_save(self):
        try:
            if self.mapping is None: raise RuntimeError("请先连接H743台架")
            if not self.mapping.save(): raise RuntimeError("当前有控制操作，请先停止再保存映射")
        except Exception as exc: self.mapping_status.set(str(exc))

    def _poll_mapping(self):
        if self.mapping is None: return
        for _ in range(40):
            try: event=self.mapping.events.get_nowait()
            except queue.Empty: break
            data=event.data;message=str(data.get("message",""));state=data.get("state","")
            self.mapping_status.set(message);self._append(message)
            if state=="error":
                self._mapping_read_draft=None;self._control_error=message;self.status.set(message)
            elif state in {"testing","querying","applying","saving"}:
                self._control_error="";self.status.set(message)
            elif not self._control_error: self.status.set(message)
            channels=data.get("channels")
            if channels and self._mapping_read_draft is not None:
                if self._mapping_choices()==self._mapping_read_draft:
                    for ch in (1,2):
                        row=channels.get(ch,{})
                        self.mapping_roles[ch].set({"upper":"上桨","lower":"下桨"}.get(row.get("role"),"请选择"))
                        self.mapping_spins[ch].set({"cw":"顺时针","ccw":"逆时针"}.get(row.get("spin"),"请选择") if row.get("declared")=="1" else "请选择")
                self._mapping_read_draft=None
            if data.get("status",{}).get("calibrated")=="1" and state=="idle":
                self._control_error=""
                self.status.set(message+"；现在可以使用台架解锁")

    def manual_arm(self):
        try:
            if self._analysis_running: raise RuntimeError("正在生成报告，请稍后解锁")
            if self.manual is None: raise RuntimeError("请先连接H743台架")
            if not self.manual.arm(int(self.vars["max_pct"].get())):
                state=getattr(self.manual,"state","")
                if state in {"armed","holding"}:
                    self.status.set("台架已经解锁；调整油门后点击“应用上下桨油门”，无需再次解锁")
                else:
                    self.status.set(getattr(self.manual,"last_error","") or "正在处理上一条操作，请等确认后再解锁")
        except Exception as exc: self.status.set(f"无法解锁：{exc}")

    def manual_apply(self):
        try:
            if self.manual is None: raise RuntimeError("请先连接H743台架")
            if not self.manual.apply(float(self.manual_upper.get()),float(self.manual_lower.get()),float(self.manual_hold.get())):
                reason=getattr(self.manual,"last_error","")
                self.status.set((reason+"；排除后请重新点击台架解锁") if reason else
                                "请先完成台架解锁；上一条油门指令确认前不能重复提交")
        except Exception as exc: self.status.set(f"无法应用油门：{exc}")

    def stop(self):
        self._control_error=""
        self._auto_ticket+=1;self._auto_preparing=False
        try:
            if self.autocollect is not None: self.autocollect.stop()
            if self.max_thrust is not None: self.max_thrust.stop()
            if self.model_check is not None: self.model_check.stop()
            if self.mapping is not None: self.mapping.stop()
        finally:
            try:
                if self.manual is not None: self.manual.stop()
            finally:
                if self.engine: self.engine.cancel.set(); self.engine.stop()
        self.status.set("已请求停止；等待飞控空闲确认，板上另有 300 ms 超时")
    def tare(self):
        if self.engine and getattr(self.engine,"active_operation",None) is not None:
            self.status.set("请先停止上下桨，再执行空载去皮")
            return
        if not self.engine or not self.engine.load_cell: self.status.set("尚未连接称重传感器"); return
        engine=self.engine; store=self.store
        def work():
            try:
                value=engine.load_cell.tare(samples=8); store.update_metadata(scale_tare_raw=value); self.analysis_events.put(("tare",value))
            except Exception as exc: self.analysis_events.put(("error",str(exc)))
        threading.Thread(target=work,daemon=True).start(); self.status.set("正在空载去皮…")
    def analyze(self):
        if self.host_bridge is not None:
            self.open_library(); return
        if not self.store: self.status.set("还没有会话"); return
        if self._analysis_running: self.status.set("正在生成报告，请稍候"); return
        if self.engine and getattr(self.engine,"active_operation",None) is not None:
            self.status.set("请先停止试转或扫描，再生成拟合报告"); return
        path=self.store.path;self._analysis_running=True
        if hasattr(self,"report_open_button"): self.report_open_button.configure(state="disabled")
        def work():
            try:
                from .model_report import write_analysis
                from uuid import uuid4
                samples=read_samples(path/"samples.csv")
                metadata=json.loads((path/"metadata.json").read_text(encoding="utf-8"))
                output=path/("analysis-"+uuid4().hex[:8])
                self.analysis_events.put(("analysis",write_analysis(samples,metadata,output)))
            except Exception as exc: self.analysis_events.put(("analysis_error",str(exc)))
        threading.Thread(target=work,daemon=True).start();self.status.set("后台分析中…")
        if hasattr(self,"report_summary"): self.report_summary.set("正在分析训练误差、独立验证和实际电压覆盖…")

    def repeat_voltage_layers(self):
        if self._analysis_running or (self.engine and getattr(self.engine,"active_operation",None) is not None):
            self.status.set("请先停止当前操作，再准备独立复测"); return
        self._voltage_layer_index=0
        self.status.set("已准备从首个电量层复测；原数据保留。实际调整电池后点击开始实测扫描，不会自动启动。")

    def _show_result(self,page):
        """Newest result page (model report, lookup-table build or bench validation) behind one button."""
        self._report_path=Path(page)
        if hasattr(self,"report_open_button"): self.report_open_button.configure(state="normal")

    def open_report(self):
        if self._report_path is None or not self._report_path.is_file():
            self.status.set("还没有可看的结果：先生成模型报告、建立查补表或完成一次实测验证");return
        import webbrowser
        webbrowser.open(self._report_path.resolve().as_uri())

    def _append(self,text):
        self.live.configure(state="normal")
        self.live.insert("end",text+"\n")
        line_count=int(self.live.index("end-1c").split(".")[0])
        if line_count>500:
            self.live.delete("1.0",f"{line_count-500}.0")
        self.live.see("end")
        self.live.configure(state="disabled")
    def _poll(self):
        if self._closed: return
        if self.library_view is not None: self.library_view.poll()
        for controller in (self.autocollect,self.max_thrust,self.esc_kv,self.model_check):
            if controller is None: continue
            for _ in range(30):
                try: event=controller.events.get_nowait()
                except queue.Empty: break
                data=event.data if isinstance(event.data,dict) else {}
                message=str(data.get("message",data.get("state","自动采集状态已更新")))
                if not self._control_error:
                    self.status.set(message)
                self.scan_summary.set(message);self._append(message)
                if data.get("report"): self._show_result(data["report"])
                if data.get("state") in {"waiting_battery","waiting_recovery","finished","error","stopped"}:
                    view=self._get_library_view()
                    if data.get("run_id") and controller is self.autocollect:
                        view.last_run_id=str(data["run_id"])
                        view.last_recorded_count=int(data.get("recorded_points",0))
                    view.refresh()
        self._poll_mapping()
        if self.manual is not None:
            for _ in range(30):
                try: event=self.manual.events.get_nowait()
                except queue.Empty: break
                data=event.data
                message=str(data.get("message",data.get("state",""))) if isinstance(data,dict) else str(data)
                state=data.get("state") if isinstance(data,dict) else None
                if state=="error": self._control_error=message
                elif state in {"arming","armed","applying","holding"}: self._control_error=""
                if not self._control_error or state=="error": self.status.set(message)
        if self.engine:
            if self.host_bridge is not None: self._refresh_observation()
            for _ in range(100):
                try: event=self.engine.ui_events.get_nowait()
                except queue.Empty: break
                if event.kind=="sample":
                    self.sample_count+=1; s=event.data; show=lambda value: "-" if value is None else f"{value:.3f}"
                    power=dshot_input_power(s.voltage_v,s.voltage_age_ms,s.upper_esc_current_a,s.lower_esc_current_a,s.upper_esc_current_age_ms,s.lower_esc_current_age_ms)
                    self._append(f"{s.segment_id} 上桨={show(s.upper_erpm)}eRPM 下桨={show(s.lower_erpm)}eRPM 推力={show(s.thrust_n)}N 电压={show(s.voltage_v)}V 上桨电流={show(s.upper_esc_current_a)}A 下桨电流={show(s.lower_esc_current_a)}A 输入功率={show(power)}W 质量标记={','.join(s.quality) or '正常'}")
                elif event.kind=="error":
                    self._control_error=f"错误：{event.data}"
                    self.status.set(self._control_error); self._append(self._control_error)
                elif event.kind=="rejected":
                    from .manual_control import rejection_message
                    self._control_error=rejection_message(event.data.get("reason","unknown"))
                    self.status.set(self._control_error); self._append(self._control_error)
                elif event.kind=="run_finished":
                    if self.host_bridge is not None:
                        self._get_library_view().refresh()
                    if self._active_smart:
                        self._active_smart=False
                        if not self._control_error: self.status.set("本轮已结束并请求停止；实验正在入库，可选择训练与验证")
                        continue
                    if self._control_error:
                        self._append("扫描已结束；请先处理显示的失败原因")
                        continue
                    layers=self._voltage_layers()
                    completed=bool(event.data.get("completed")) if isinstance(event.data,dict) else False
                    if layers and completed:
                        self._voltage_layer_index+=1
                        if self._voltage_layer_index<len(layers):
                            label,target=layers[self._voltage_layer_index]; self.status.set(f"本层结束且 停止指令已发送。请实际调整/更换电池到 {label} 层约 {target:g} V，再点击开始本层。")
                        else: self.status.set(f"全部电量层扫描线程结束；停止指令已发送；已记录 {self.sample_count} 个拼接样本")
                    elif layers: self.status.set("本层未完成，停止指令已发送；请排除错误后在同一真实电量层重试。")
                    else: self.status.set(f"扫描线程结束，停止指令已发送；已记录 {self.sample_count} 个拼接样本")
                elif event.kind=="stopped_confirmed":
                    if not self._control_error: self.status.set("飞控已确认台架窗口已空闲")
                elif event.kind in {"point_started","point_progress","point_skipped","point_stable"}:
                    data=event.data
                    message=str(data.get("message",data)) if isinstance(data,dict) else str(data)
                    if hasattr(self,"scan_summary"): self.scan_summary.set(message)
                    self._append(message)
        while True:
            try: item=self.analysis_events.get_nowait()
            except queue.Empty: break
            if isinstance(item,tuple) and item[0]=="analysis":
                self._analysis_running=False; paths=item[1]
                self._report_path=Path(paths["dashboard"]) if paths.get("dashboard") else Path(paths["report"])
                try:
                    result=json.loads(Path(paths["model"]).read_text(encoding="utf-8"))
                    summary=result.get("summary") or result.get("static",{}).get("sufficiency",{}).get("summary","报告已生成，请查看实测/预测、误差与数据覆盖")
                    if not isinstance(summary,str): summary="报告已生成，查看验证误差、适用范围和补测建议"
                except (OSError,ValueError,KeyError): summary="报告已生成，请查看拟合效果"
                if hasattr(self,"report_summary"): self.report_summary.set(summary)
                if hasattr(self,"report_open_button"): self.report_open_button.configure(state="normal")
                self.status.set("报告已生成，点击“查看拟合效果”")
            elif isinstance(item,tuple) and item[0]=="analysis_error":
                self._analysis_running=False;self.status.set("分析失败："+item[1])
                if hasattr(self,"report_summary"): self.report_summary.set("分析失败："+item[1])
                if self._report_path is not None and hasattr(self,"report_open_button"): self.report_open_button.configure(state="normal")
            elif isinstance(item,tuple) and item[0]=="smart_plan":
                self._scan_preparing=False; self._prepared_scan=item[1]
                kind,maximum,plan,voltage=item[1]
                self.scan_summary.set(f"补测计划已准备：{len(plan.points)} 个目标，上限 {maximum}%；点击开始才输出。")
            elif isinstance(item,tuple) and item[0]=="smart_plan_error":
                self._scan_preparing=False; self.status.set("补测计划失败："+item[1])
            elif isinstance(item,tuple) and item[0]=="autocollect_ready":
                ticket,engine,link_generation,covered,maximum,stop_voltage,duration,points,preview_only=item[1]
                if ticket!=self._auto_ticket or engine is not self.engine or self._closed: continue
                self._auto_preparing=False
                if (link_generation!=self.connection.generation or
                        engine.active_operation is not None):
                    self.status.set("准备期间连接或控制状态变化；自动采集未启动")
                    continue
                try:
                    from .autocollect import AutoCollector
                    controller=AutoCollector(engine,covered,max_percent=maximum,
                        stop_voltage_v=stop_voltage,max_duration_s=duration,max_points=points)
                    if preview_only:
                        count=len(controller.targets)
                        self.scan_summary.set(
                            f"历史中有 {len(covered)} 个同装配有效稳态点；油门 {controller.schedule.cells[0].upper_pct:g}～{maximum}% "
                            f"共 {count} 格，每段电量先扫5×5粗网格再加密，测完不空转。"
                            + f"每轮最多 {points} 点或 {duration/60:g} 分钟；电量（扣除负载压降）降到 {stop_voltage:g} V 或带载跌破 9.0 V 时停止；预览未启动电机。")
                        continue
                    if self.autocollect is not None:
                        self.autocollect.close()
                    if not controller.start(): raise RuntimeError("台架已有控制操作或不能安全启动")
                    self.autocollect=controller
                    self.scan_summary.set("自动采集中；同一电量下连续扫油门网格，到换电阈值停机提醒。")
                except Exception as exc: self.status.set("自动采集未启动："+str(exc))
            elif isinstance(item,tuple) and item[0]=="autocollect_error":
                ticket,message=item[1]
                if ticket==self._auto_ticket:
                    self._auto_preparing=False;self.status.set("无法准备自动采集："+message)
            elif isinstance(item,tuple) and item[0]=="throttle_model":
                self._analysis_running=False;path,summary,page=item[1]
                self._show_result(page)
                self.scan_summary.set(summary);self.status.set("查补表已保存，点“查看拟合效果”看图："+path);self._append(summary)
            elif isinstance(item,tuple) and item[0]=="throttle_model_error":
                self._analysis_running=False;self.status.set("查补表建立失败："+item[1]);self.scan_summary.set("查补表建立失败："+item[1])
            elif isinstance(item,tuple) and item[0]=="tare": self.status.set(f"去皮完成，零点原始值={item[1]:.1f}，已写入会话元数据")
            elif isinstance(item,tuple): self.status.set(f"错误：{item[1]}")
        self._poll_after_id=self.after(40,self._poll)
    def _refresh_observation(self):
        observation=self.engine.latest_observation()
        fc=observation.snapshot
        show=lambda value: "--" if value is None else f"{value:.1f}"
        speeds=[None,None];currents=[None,None];ages=[None,None]
        if fc is not None and {fc.upper_channel,fc.lower_channel}=={1,2}:
            for role,channel in enumerate((fc.upper_channel,fc.lower_channel)):
                speeds[role]=fc.erpm[channel-1]
                currents[role]=fc.esc_current_a[channel-1]
                ages[role]=fc.esc_current_age_ms[channel-1]
        voltage=fc.voltage_v if fc else None
        total=dshot_total_current(*currents,*ages)
        power=dshot_input_power(voltage,fc.voltage_age_ms if fc else None,*currents,*ages)
        self.telemetry_text.set(f"上桨 {show(speeds[0])} eRPM    下桨 {show(speeds[1])} eRPM    电压 {show(voltage)} V\nDShot电流：上桨 {show(currents[0])} A    下桨 {show(currents[1])} A    合计 {show(total)} A    输入功率 {show(power)} W")
        self.raw_channel_text.set("通道1："+show(fc.erpm[0] if fc else None)+" eRPM    通道2："+show(fc.erpm[1] if fc else None)+" eRPM")
        gui=self.host_bridge.gui
        gui.cal_value_var.set(f"重量 {show(observation.scale_grams)} 克")
        gui.value_var.set("原始值见会话记录")
        gui.status_var.set("H743台架实时称重" if observation.scale_grams is not None else "称重数据未到达或已过期")

    def shutdown(self):
        if self._closed: return
        self._closed=True
        if self._poll_after_id is not None:
            with __import__("contextlib").suppress(tk.TclError): self.after_cancel(self._poll_after_id)
            self._poll_after_id=None
        self.disconnect()


class ThrustBenchApp(tk.Tk):
    """Compatibility wrapper; users enter through the original pressure window."""
    def __init__(self, **kwargs):
        super().__init__(); self.title("共轴推力台采集"); self.geometry("980x720")
        self.frame=ThrustBenchFrame(self,**kwargs); self.frame.pack(fill="both",expand=True)
        self.protocol("WM_DELETE_WINDOW",self.close)
    def __getattr__(self,name):
        frame=self.__dict__.get("frame")
        if frame is not None: return getattr(frame,name)
        raise AttributeError(name)
    def close(self):
        self.frame.shutdown(); self.destroy()
def main(): ThrustBenchApp().mainloop()
if __name__=="__main__": main()
