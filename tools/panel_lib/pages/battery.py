"""Battery readback, low-voltage warning and confirmed RAM configuration."""
import secrets
import time
import tkinter as tk
from tkinter import ttk
from ..battery_monitor import decode_battery


class BatteryPage(ttk.Frame):
    def __init__(self,parent,panel):
        super().__init__(parent,padding=12);self.panel=panel
        self.session=None;self.snapshot=None;self.received=None;self.pending=None;self.deadline=0.0
        self.nonce=secrets.randbelow(0xfffffffe)+1;self.last_poll=0.0
        self.closed=False;self.timer=None;self.dirty=False;self.updating=False;self.protocol_errors=0
        self.unsupported=False
        self.value=tk.StringVar(self,value='— V');self.status=tk.StringVar(self,value='等待飞控电池数据')
        self.notice=tk.StringVar(self,value='')
        self.auto=tk.BooleanVar(self,value=True)
        ttk.Label(self,text='电池电压与低压保护',style='PageTitle.TLabel').pack(anchor=tk.W)
        bar=ttk.Frame(self);bar.pack(fill=tk.X,pady=10)
        ttk.Button(bar,text='立即读取',command=self.request).pack(side=tk.LEFT)
        ttk.Checkbutton(bar,text='自动刷新（2秒）',variable=self.auto).pack(side=tk.LEFT,padx=8)
        self.value_label=ttk.Label(self,textvariable=self.value,style='PageTitle.TLabel');self.value_label.pack(anchor=tk.W)
        self.status_label=ttk.Label(self,textvariable=self.status,style='Muted.TLabel');self.status_label.pack(anchor=tk.W,pady=6)
        box=ttk.LabelFrame(self,text='飞控回读',padding=10);box.pack(fill=tk.X,pady=6)
        self.fields={}
        for row,(key,label) in enumerate((('cells','配置串数'),('cell','平均单节电压'),('current','电流'),
                ('raw','电压ADC原始值'),('valid','电压采样有效'),('age','回包时样本年龄'),('samples','采样次数'),
                ('errors','采样错误次数'),('limits','告警 / 恢复线'),('can_arm','电池项允许新解锁'),
                ('tx','ELRS交给DMA / 拒绝次数'))):
            ttk.Label(box,text=label).grid(row=row,column=0,sticky=tk.W,padx=(0,18),pady=2)
            var=tk.StringVar(self,value='—');self.fields[key]=var
            ttk.Label(box,textvariable=var).grid(row=row,column=1,sticky=tk.W)
        config=ttk.LabelFrame(self,text='本次运行配置（重启恢复默认）',padding=10);config.pack(fill=tk.X,pady=6)
        self.cells=tk.IntVar(self,value=3);self.low=tk.IntVar(self,value=3500);self.recover=tk.IntVar(self,value=3600)
        for row,(label,var,lo,hi) in enumerate((('串数',self.cells,1,12),('每节告警 mV',self.low,2500,4100),
                                              ('每节恢复 mV',self.recover,2501,4400))):
            ttk.Label(config,text=label).grid(row=row,column=0,sticky=tk.W,pady=3)
            ttk.Spinbox(config,textvariable=var,from_=lo,to=hi,width=9,style='Numeric.TSpinbox').grid(row=row,column=1,sticky=tk.W,padx=8)
            var.trace_add('write',self._draft_changed)
        self.apply_button=ttk.Button(config,text='应用并回读',command=self.apply_config,state=tk.DISABLED)
        self.apply_button.grid(row=3,column=0,columnspan=2,sticky=tk.W,pady=5)
        ttk.Label(config,textvariable=self.notice,style='Muted.TLabel').grid(row=4,column=0,columnspan=3,sticky=tk.W)
        note=ttk.Label(self,text='总电压由飞控ADC分压换算；平均单节=总压/串数，未逐节测量。比例为标称值。\n'
                       '低压或电压数据无效时拒绝新解锁；已解锁后仅告警，不自动切断电机。\n'
                       'ELRS回传电压、电流；容量和剩余电量没有可靠估算时保持未知。',
                       style='Muted.TLabel',wraplength=820,justify=tk.LEFT)
        note.pack(anchor=tk.W,pady=8);self.bind('<Configure>',lambda e:note.configure(wraplength=max(220,e.width-28)))
        self.bind('<Destroy>',self._destroy,add='+');self._tick()

    def _draft_changed(self,*args):
        if not self.updating:self.dirty=True;self.notice.set('配置有未提交修改')

    def _connection(self):
        t=self.panel.transport;session=(t,getattr(t,'connection_generation',0))
        connected=self.panel._transport_connected()
        if session!=self.session or not connected:
            self.session=session;self.snapshot=None;self.pending=None;self.received=None;self.last_poll=0
            self.unsupported=False
            self.value.set('— V');self.status.set('等待当前连接的电池回包' if connected else '未连接')
            for var in self.fields.values():var.set('—')
        return connected

    def _send(self,command,kind):
        if not self._connection():return False
        if not self.panel._validation_guard_command(command):return False
        sender=getattr(self.panel.transport,'send_ascii_line',self.panel.transport.send_line)
        if not sender(command):self.notice.set('发送被拒绝或链路占用');return False
        self.pending=kind;self.deadline=time.monotonic()+2.0;return True

    def request(self):
        if not self._connection() or self.pending is not None:return False
        self.unsupported=False
        self.nonce=(self.nonce+1)&0xffffffff or 1
        self.last_poll=time.monotonic()
        return self._send(f'BATTERY? {self.nonce}','query')

    def apply_config(self):
        if not self._can_configure():self.notice.set('需要当前连接的新鲜、未解锁回包');return False
        try:values=(int(self.cells.get()),int(self.low.get()),int(self.recover.get()))
        except (ValueError,tk.TclError):self.notice.set('配置必须为整数');return False
        cells,low,recover=values
        if not (1<=cells<=12 and 2500<=low<=4100 and low<recover<=4400):
            self.notice.set('串数或阈值超界；恢复线必须高于告警线');return False
        self.nonce=(self.nonce+1)&0xffffffff or 1
        return self._send(f'BATTERY SET {self.nonce} {cells} {low} {recover}',values)

    def _can_configure(self):
        if not self._connection():return False
        return (self.snapshot is not None and self.received is not None and
                time.monotonic()-self.received<=3.0 and self.snapshot.arm_state==1 and
                self.pending is None and self.panel._transport_connected())

    def accept(self,payload,context):
        if not self._connection() or self.pending is None:return
        if context is not None and not context.is_current(self.panel.transport):return
        try:snapshot=decode_battery(payload)
        except ValueError as error:
            self.protocol_errors+=1;self.pending=None;self.snapshot=None;self.value.set('— V')
            for var in self.fields.values():var.set('—')
            self.status_label.configure(style='Fail.TLabel')
            self.status.set(f'{error} · 协议错误 {self.protocol_errors}');return
        if snapshot.nonce!=self.nonce:return
        operation=self.pending;self.pending=None;self.snapshot=snapshot
        self.received=time.monotonic() if context is None else context.received_at
        self._render()
        if operation!='query':
            actual=(snapshot.cells,snapshot.low_mv,snapshot.recover_mv)
            acknowledged=bool(snapshot.flags&32) and actual==operation
            self.notice.set('飞控已应用 · RAM配置' if acknowledged else '飞控未确认应用；当前值以回读为准')
            try:unchanged=(self.cells.get(),self.low.get(),self.recover.get())==operation
            except tk.TclError:unchanged=False
            if acknowledged and unchanged:self.dirty=False
        if not self.dirty:
            self.updating=True
            for var,value in ((self.cells,snapshot.cells),(self.low,snapshot.low_mv),(self.recover,snapshot.recover_mv)):var.set(value)
            self.updating=False

    def _render(self):
        s=self.snapshot
        if s is None:return
        stale=self.received is None or time.monotonic()-self.received>3
        readable=s.valid and not stale
        self.value.set(f'{s.voltage_mv/1000:.3f} V' if readable else '— V')
        message='电压正常' if s.can_arm else ('低电压告警 · 禁止新解锁' if s.valid else '电压采样无效 · 禁止新解锁')
        self.status.set('回包已过期' if stale else message)
        self.status_label.configure(style='Pass.TLabel' if readable and s.can_arm else 'Fail.TLabel')
        values={'cells':f'{s.cells} S','cell':f'{s.voltage_mv/s.cells/1000:.3f} V' if readable else '—',
                'current':f'{s.current_ma/1000:.3f} A' if s.current_ma is not None and not stale else '—',
                'raw':str(s.raw),'valid':'是' if readable else '否','age':f'{s.age_ms} ms',
                'samples':str(s.samples),'errors':str(s.errors),
                'limits':f'{s.low_mv/1000:.2f} / {s.recover_mv/1000:.2f} V每节',
                'can_arm':'是' if s.can_arm and not stale else '否',
                'tx':f'{s.tx_accepted} / {s.tx_rejected}'}
        for key,value in values.items():self.fields[key].set(value)

    def _tick(self):
        if self.closed:return
        now=time.monotonic();connected=self._connection()
        if self.pending is not None and now>self.deadline:
            self.pending=None;self.last_poll=now;self.notice.set('电池回包超时，未确认操作结果')
        self.apply_button.configure(state=tk.NORMAL if self._can_configure() else tk.DISABLED)
        if self.snapshot is not None and self.received is not None and now-self.received>3:self._render()
        if connected and self.winfo_ismapped() and self.auto.get() and not self.unsupported and now-self.last_poll>=2:self.request()
        self.timer=self.after(250,self._tick)

    def _destroy(self,event):
        if event.widget is self:
            self.closed=True
            if self.timer is not None:self.after_cancel(self.timer);self.timer=None

    def handle_line(self,line):
        if self.pending is not None and line.startswith('ERR unknown cmd BATTERY'):
            self.pending=None;self.unsupported=True;self.snapshot=None;self.value.set('— V')
            for var in self.fields.values():var.set('—')
            self.status.set('当前固件未提供电池电压回读')


def mount_battery(panel,parent):
    panel.battery_page=BatteryPage(parent,panel);panel.battery_page.pack(fill=tk.BOTH,expand=True)
