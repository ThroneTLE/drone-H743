"""Tk presentation only: no controller, network or experiment mutation."""
import math
import tkinter as tk
from tkinter import ttk

from .experiments import ExperimentKind

BG = '#10171f'
PANEL = '#17212c'
INK = '#e0e8ef'
MUTED = '#8e9ead'
GRID = '#253341'
TEAL = '#55d6be'
GOLD = '#f2bd68'
BLUE = '#70b7ed'
KINDS = {'位置阶跃 · X': ExperimentKind.POSITION_STEP,
         '速度阶跃 · X': ExperimentKind.VELOCITY_STEP,
         '俯仰阶跃': ExperimentKind.PITCH_STEP}
FONT = 'Microsoft YaHei UI'


class PresentationMixin:
    def _build_widgets(self):
        self.configure(background=BG)
        self.minsize(1100, 740)
        style = ttk.Style(self)
        style.theme_use('clam')
        style.configure('.', background=PANEL, foreground=INK, font=(FONT, 10))
        style.configure('TFrame', background=PANEL)
        style.configure('TLabel', background=PANEL, foreground=INK)
        style.configure('Muted.TLabel', foreground=MUTED, font=(FONT, 9))
        style.configure('Title.TLabel', font=(FONT, 13, 'bold'))
        style.configure('TButton', padding=(10, 8), background='#253847', borderwidth=0)
        style.map('TButton', background=[('active', '#354c60'), ('disabled', '#202c38')],
                  foreground=[('disabled', '#6c7c8b')])
        style.configure('Accent.TButton', background=TEAL, foreground='#0e2421')
        style.map('Accent.TButton', background=[('active', '#7fe5d2'), ('disabled', '#294840')])
        style.configure('TEntry', fieldbackground='#0f1923', foreground=INK, padding=6,
                        insertcolor=INK, bordercolor=GRID)
        style.configure('TCombobox', fieldbackground='#0f1923', foreground=INK, padding=6)
        style.map('TCombobox', fieldbackground=[('readonly', '#0f1923')],
                  foreground=[('readonly', INK)], selectbackground=[('readonly', '#0f1923')])
        self.option_add('*TCombobox*Listbox.background', PANEL)
        self.option_add('*TCombobox*Listbox.foreground', INK)
        self.option_add('*TCombobox*Listbox.font', (FONT, 10))

        header = ttk.Frame(self, padding=(20, 14))
        header.grid(row=0, column=0, columnspan=3, sticky='ew', padx=12, pady=(12, 10))
        ttk.Label(header, text='XZ  /  飞行控制实验室', font=(FONT, 18, 'bold')).pack(side='left')
        ttk.Label(header, text='二维教学仿真  ·  同源 C 控制器', style='Muted.TLabel').pack(side='right')
        controls = ttk.Frame(self, padding=16)
        controls.grid(row=1, column=0, sticky='nsew', padx=(12, 10))
        scene = ttk.Frame(self, padding=12)
        scene.grid(row=1, column=1, sticky='nsew', padx=(0, 10))
        charts = ttk.Frame(self, padding=12)
        charts.grid(row=1, column=2, sticky='nsew', padx=(0, 12))
        self.columnconfigure(0, minsize=242)
        self.columnconfigure(1, weight=3, minsize=450)
        self.columnconfigure(2, weight=2, minsize=340)
        self.rowconfigure(1, weight=1)

        ttk.Label(controls, text='01  实验设置', style='Title.TLabel').pack(anchor='w')
        self.kind = tk.StringVar(value=ExperimentKind.POSITION_STEP.value)
        self.experiment_name = tk.StringVar(value=next(iter(KINDS)))
        combo = ttk.Combobox(controls, textvariable=self.experiment_name, values=list(KINDS),
                             state='readonly', width=21)
        combo.pack(fill='x', pady=(12, 10))
        def select(_):
            kind = KINDS[self.experiment_name.get()]
            self.kind.set(kind.value)
            self.device.set_kind(kind)
        combo.bind('<<ComboboxSelected>>', select)
        targets = ttk.Frame(controls)
        targets.pack(fill='x')
        self.target_x, self.target_vx, self.target_pitch = [tk.StringVar(value=v) for v in ('0.5', '0.2', '3.0')]
        for row, (label, variable) in enumerate((('位移  /  m', self.target_x),
                ('速度  /  m/s', self.target_vx), ('俯仰  /  °', self.target_pitch))):
            ttk.Label(targets, text=label, style='Muted.TLabel').grid(row=row, column=0, sticky='w', pady=4)
            ttk.Entry(targets, textvariable=variable, width=9).grid(row=row, column=1, sticky='e', pady=4)
        targets.columnconfigure(1, weight=1)
        ttk.Button(controls, text='应用实验目标', command=self._apply_targets).pack(fill='x', pady=(6, 12))
        ttk.Label(controls, text='先保持 1 秒，再施加阶跃。', style='Muted.TLabel').pack(anchor='w')
        ttk.Separator(controls).pack(fill='x', pady=14)
        self.start_button = ttk.Button(controls, text='开始 / 暂停', style='Accent.TButton', command=self._toggle)
        self.start_button.pack(fill='x')
        ttk.Button(controls, text='复位场景', command=self._reset).pack(fill='x', pady=6)
        speeds = ttk.Frame(controls)
        speeds.pack(fill='x', pady=8)
        ttk.Label(speeds, text='慢放', style='Muted.TLabel').pack(side='left')
        self.scale = tk.DoubleVar(value=1.0)
        for v in (0.25, 0.5, 1.0):
            ttk.Radiobutton(speeds, text=f'{v:g}×', value=v, variable=self.scale,
                            command=lambda: setattr(self.device, 'time_scale', self.scale.get())).pack(side='left', padx=3)
        ttk.Separator(controls).pack(fill='x', pady=12)
        ttk.Label(controls, text='02  模型假设', style='Title.TLabel').pack(anchor='w')
        ttk.Label(controls, text='电机响应时间常数  /  s', style='Muted.TLabel').pack(anchor='w', pady=(10, 4))
        self.motor_tau = tk.StringVar(value=f'{self.device.engine.plant.thrust_tau_s:.3f}')
        ttk.Entry(controls, textvariable=self.motor_tau, width=9).pack(fill='x')
        ttk.Button(controls, text='应用模型假设', command=self._apply_model).pack(fill='x', pady=6)
        ttk.Label(controls, text='越大，推力响应越慢。\n该值是教学假设，可自行比较。',
                  style='Muted.TLabel').pack(anchor='w')

        scene_head = ttk.Frame(scene)
        scene_head.pack(fill='x', pady=(2, 8))
        ttk.Label(scene_head, text='侧视场景', style='Title.TLabel').pack(side='left')
        self.run_status = tk.StringVar(value='暂停')
        ttk.Label(scene_head, textvariable=self.run_status, foreground=TEAL).pack(side='right')
        self.metrics = tk.StringVar()
        ttk.Label(scene, textvariable=self.metrics, style='Muted.TLabel').pack(anchor='w', pady=(0, 10))
        self.canvas = tk.Canvas(scene, width=440, height=420, background=BG, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        ttk.Label(scene, text='● 目标   ━ 实际轨迹   ↑ 合推力    /    +X 向前 · +Z 向上',
                  style='Muted.TLabel').pack(anchor='w', pady=(10, 0))

        ttk.Label(charts, text='03  响应对比', style='Title.TLabel').pack(anchor='w')
        ttk.Label(charts, text='保存 A → 在上位机调参 → 运行 B', style='Muted.TLabel').pack(anchor='w', pady=(6, 12))
        self._baseline_params = None
        self.save_a_button = ttk.Button(charts, text='保存 A 参数快照', command=self._save_a)
        self.save_a_button.pack(fill='x')
        self.run_b_button = ttk.Button(charts, text='运行 B 并保存', command=self._run_ab)
        self.run_b_button.pack(fill='x', pady=6)
        self.ab_status = tk.StringVar(value='尚未保存 A。运行场景时，下方显示实时响应。')
        ttk.Label(charts, textvariable=self.ab_status, style='Muted.TLabel', wraplength=300).pack(fill='x', pady=(4, 12))
        self.ab_canvas = tk.Canvas(charts, width=320, height=380, background=BG, highlightthickness=0)
        self.ab_canvas.pack(fill='both', expand=True)
        self.status = tk.StringVar(value='等待地面站')
        footer = ttk.Frame(self, padding=(16, 9))
        footer.grid(row=2, column=0, columnspan=3, sticky='ew', padx=12, pady=(10, 12))
        ttk.Label(footer, textvariable=self.status, style='Muted.TLabel').pack(side='left')
        ttk.Label(footer, text='仅用于理解响应 · 不代表实机性能', style='Muted.TLabel').pack(side='right')

    def _draw_scene(self, state):
        c = self.canvas
        c.delete('all')
        w, h = max(1, c.winfo_width()), max(1, c.winfo_height())
        unit = min((w - 64) / 3.0, (h - 72) / 2.6)
        center = round(state.x_m) if abs(state.x_m) > 1 else 0
        ox, oz = w / 2 - center * unit, h - 48
        def point(x, z): return ox + x * unit, oz - z * unit
        for i in range(-4, 5):
            gx = center + i * 0.5
            px, _ = point(gx, 0)
            if 25 < px < w - 15:
                c.create_line(px, 32, px, h - 32, fill=GRID)
                c.create_text(px, h - 18, text=f'{gx:g}', fill=MUTED, font=(FONT, 8))
        for i in range(6):
            gz = i * 0.5
            _, py = point(0, gz)
            if py > 30:
                c.create_line(30, py, w - 16, py, fill=GRID)
                c.create_text(14, py, text=f'{gz:g}', fill=MUTED, font=(FONT, 8))
        c.create_text(16, 16, text='Z / m', anchor='w', fill=MUTED, font=(FONT, 9))
        c.create_text(w - 18, 16, text='X / m', anchor='e', fill=MUTED, font=(FONT, 9))
        if len(self._trajectory) > 1:
            c.create_line(*[v for x, z in self._trajectory for v in point(x, z)], fill=GOLD, width=2)
        target_x = self.device.engine.targets.position_step_m if self.device.engine.kind is ExperimentKind.POSITION_STEP and state.time_s >= 1 else (0 if self.device.engine.kind is ExperimentKind.POSITION_STEP else state.x_m)
        tx, tz = point(target_x, 1)
        c.create_oval(tx-9, tz-9, tx+9, tz+9, outline=TEAL, width=2, dash=(3, 3))
        c.create_text(tx, tz-20, text='目标', fill=TEAL, font=(FONT, 9))
        x, z = point(state.x_m, state.z_m)
        co, si = math.cos(state.pitch_rad), math.sin(state.pitch_rad)
        def local(a, b): return x+a*co-b*si, z+a*si+b*co
        # Vertical fuselage (+body Z upward); horizontal +X is a coordinate,
        # not a drawing of the fuselage long axis.
        c.create_polygon(*[v for p in ((0,-64),(13,-45),(12,14),(-12,14),(-13,-45)) for v in local(*p)],
                         fill='#334555', outline=GOLD, width=2, tags='fuselage')
        c.create_line(*local(0,-48), *local(0,12), fill='#566c7d', width=2)
        c.create_text(*local(-20,-61), text='机体上端', anchor='e', fill=GOLD, font=(FONT,9))
        angle = state.pitch_rad - state.pitch_tilt_rad
        for offset in (26, 38):
            bx, bz = local(0, offset)
            c.create_line(*local(0,14), bx, bz, fill=MUTED, width=3)
            dx, dz = 34*math.cos(angle), 34*math.sin(angle)
            c.create_line(bx-dx, bz-dz, bx+dx, bz+dz, fill='#b5a2e6', width=3)
        c.create_text(*local(-42,38), text='同轴双桨', anchor='e', fill='#b5a2e6', font=(FONT,9))
        # Draw the resultant at the thrust application point below the CG.
        bx,bz=local(0,32)
        length = min(155, max(0,state.thrust_n*9))
        tip_x,tip_z=bx+length*math.sin(angle),bz-length*math.cos(angle)
        if abs(tip_x-bx)>1:
            c.create_line(bx,bz,tip_x,bz,tip_x,tip_z,fill='#365f75',dash=(3,3))
        c.create_line(bx,bz,tip_x,tip_z,fill=BLUE,width=3,
                      arrow=tk.LAST,arrowshape=(10,12,5),tags='thrust')
        c.create_text(tip_x+12,tip_z-6,text=f'合推力 T  {state.thrust_n:.2f} N',
                      anchor='w',fill=BLUE,font=(FONT,10,'bold'))
        fx=state.thrust_n*math.sin(angle)
        fz=state.thrust_n*math.cos(angle)
        c.create_text(44,42,text=f'Fx {fx:+.2f} N   ·   Fz {fz:+.2f} N',
                      anchor='w',fill=BLUE,font=(FONT,9),tags='force_components')
        c.create_text(44,62,text=f'舵机倾转 {math.degrees(state.pitch_tilt_rad):+.1f}°',
                      anchor='w',fill=MUTED,font=(FONT,9))
        c.create_oval(x-4,z-4,x+4,z+4, fill=INK, outline='')

    def _draw_ab_plot(self):
        c = self.ab_canvas
        c.delete('all')
        w, h = max(1,c.winfo_width()), max(1,c.winfo_height())
        result = self._ab_result
        streams = [(result.baseline, GOLD, ()), (result.tuned, TEAL, (4,3))] if result else [(tuple(self._history), BLUE, ())]
        c.create_text(12,16,text='A/B 五量对比' if result else '实时响应',anchor='w',fill=INK,font=(FONT,10,'bold'))
        if result:
            c.create_line(w-134,16,w-114,16,fill=GOLD,width=2)
            c.create_text(w-106,16,text='A',anchor='w',fill=GOLD)
            c.create_line(w-68,16,w-48,16,fill=TEAL,width=2,dash=(4,3))
            c.create_text(w-40,16,text='B',anchor='w',fill=TEAL)
        fields = [('pitch_rad','俯仰角','°',180/math.pi), ('pitch_rate_rad_s','俯仰角速度','°/s',180/math.pi),
                  ('vx_m_s','X 速度','m/s',1), ('x_m','X 位置','m',1), ('z_m','Z 高度','m',1)]
        row_h = (h-62)/5
        for row,(key,label,unit,factor) in enumerate(fields):
            top=36+row*row_h
            y0,y1=top+22,top+row_h-8
            c.create_text(10,top+5,text=f'{label} / {unit}',anchor='w',fill=MUTED,font=(FONT,9))
            values=[getattr(s,key)*factor for samples,_,_ in streams for s in samples]
            lo,hi=(min(values),max(values)) if values else (0,1)
            margin=max((hi-lo)*.08, .005)
            lo,hi=lo-margin,hi+margin
            c.create_text(54,y0,text=f'{hi:.4g}',anchor='e',fill=MUTED,font=(FONT,8))
            c.create_text(54,y1,text=f'{lo:.4g}',anchor='e',fill=MUTED,font=(FONT,8))
            c.create_line(62,y0,w-12,y0,fill=GRID)
            c.create_line(62,y1,w-12,y1,fill=GRID)
            for samples,color,dash in streams:
                if len(samples)<2: continue
                start,end=samples[0].time_s,samples[-1].time_s
                stride=max(1,len(samples)//max(1,int(w-60)))
                selected=list(samples[::stride])
                if selected[-1] is not samples[-1]: selected.append(samples[-1])
                pts=[]
                for sample in selected:
                    px=63+(sample.time_s-start)/max(end-start,1e-6)*(w-77)
                    py=y1-(getattr(sample,key)*factor-lo)/(hi-lo)*(y1-y0)
                    pts.extend((px,py))
                c.create_line(*pts,fill=color,width=2,dash=dash)
        if streams[0][0]:
            samples=streams[0][0]
            c.create_text(63,h-12,text=f'{samples[0].time_s:.1f} s',anchor='w',fill=MUTED,font=(FONT,8))
            c.create_text(w-12,h-12,text=f'{samples[-1].time_s:.1f} s',anchor='e',fill=MUTED,font=(FONT,8))
