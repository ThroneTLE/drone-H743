"""Overview populated only by complete firmware component registrations."""
from __future__ import annotations
import secrets
import time
import tkinter as tk
from tkinter import ttk
from ..component_registry import HEADER, RegistryTransaction, field_text

STATE = {0:("未知", "warn"), 1:("已就绪", "pass"), 2:("等待数据", "warn"),
         3:("未启用", "warn"), 4:("故障", "fail")}


class OverviewPage(ttk.Frame):
    def __init__(self, parent, panel):
        super().__init__(parent)
        self.panel = panel
        self.session = None
        self.transaction = None
        self.started = self.last_rx = self.last_attempt = None
        self.first_received = None
        self.records = ()
        self.errors = 0
        self.unsupported = False
        self.was_visible = False
        self.needs_initial = True
        self._stale_rendered = False
        self.nonce = secrets.randbelow(0xfffffffe) + 1
        self.status = tk.StringVar(self, value="等待飞控注册信息")
        self.detail = tk.StringVar(self, value="选择元件查看固件报告")
        self._timer = None
        self._disposed = False
        ttk.Label(self, text="系统总览", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(self, text="元件清单、型号、状态和读数均由飞控提供；进入页面读取一次，可手动刷新。", style="Muted.TLabel").pack(anchor=tk.W, pady=(3, 10))
        bar = ttk.Frame(self); bar.pack(fill=tk.X)
        self.refresh_button = ttk.Button(bar, text="刷新注册信息", command=self.request, style="Primary.TButton")
        self.refresh_button.pack(side=tk.LEFT)
        ttk.Label(bar, textvariable=self.status, style="Muted.TLabel").pack(side=tk.LEFT, padx=12)
        table = ttk.Frame(self); table.pack(fill=tk.BOTH, expand=True, pady=10)
        self.tree = ttk.Treeview(table, columns=("model","state","stage","value","code","bus"), show="tree headings", height=12)
        self.tree.heading("#0", text="已注册元件"); self.tree.column("#0", width=125, minwidth=90)
        for key, label, width in (("model","型号 / 类型",125),("state","状态",85),
                                  ("stage","阶段",120),("value","固件读数",340),
                                  ("code","诊断码",70),("bus","接口",130)):
            self.tree.heading(key,text=label); self.tree.column(key,width=width,minwidth=65,stretch=key=="value")
        scroll = ttk.Scrollbar(table, orient=tk.HORIZONTAL, command=self.tree.xview)
        vertical = ttk.Scrollbar(table, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(xscrollcommand=scroll.set, yscrollcommand=vertical.set)
        table.rowconfigure(0, weight=1); table.columnconfigure(0, weight=1)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns"); scroll.grid(row=1, column=0, sticky="ew")
        self.tree.tag_configure("pass", foreground="#00d084")
        self.tree.tag_configure("fail", foreground="#ff8d80")
        self.tree.tag_configure("warn", foreground="#929ba9")
        self.tree.bind("<<TreeviewSelect>>", lambda e:self.refresh_detail())
        box=ttk.LabelFrame(self,text="元件详情",padding=10);box.pack(fill=tk.X)
        detail_label=ttk.Label(box,textvariable=self.detail,wraplength=1000,justify=tk.LEFT,style="Mono.TLabel")
        detail_label.pack(anchor=tk.W, fill=tk.X)
        box.bind("<Configure>", lambda event:detail_label.configure(wraplength=max(180,event.width-24)))
        self.bind("<Destroy>",self._destroy,add="+")
        self._tick()

    def _connection(self):
        t=self.panel.transport
        session=(t,getattr(t,"connection_generation",0))
        connected=self.panel._transport_connected()
        if session!=self.session or not connected:
            self.session=session;self.records=();self.transaction=None
            self.started=self.last_rx=self.last_attempt=None;self.unsupported=False
            self.needs_initial=True;self.first_received=None;self._stale_rendered=False
            self.tree.delete(*self.tree.get_children());self.detail.set("等待飞控注册信息")
        return connected

    def request(self):
        if not self._connection():self.status.set("未连接飞控");return False
        now=time.monotonic()
        if self.transaction is not None:self.status.set("正在接收注册清单");return False
        if self.last_attempt is not None and now-self.last_attempt<1.0:return False
        self.last_attempt=now;self.unsupported=False
        self.needs_initial=False
        self.nonce=(self.nonce+1)&0xffffffff or 1
        command=f"REGISTRY? {self.nonce}"
        if not self.panel._validation_command_allowed(command):self.status.set("当前会话拒绝读取");return False
        sender=getattr(self.panel.transport,"send_ascii_line",self.panel.transport.send_line)
        if not sender(command):self.status.set("读取未发送：连接忙或被占用");return False
        self.transaction=RegistryTransaction(self.nonce);self.started=now
        self.first_received=None
        self.status.set("等待飞控注册清单…")
        return True

    def accept(self, payload, context):
        if not self._connection() or self.transaction is None:return
        if context is not None and not context.is_current(self.panel.transport):return
        received=time.monotonic() if context is None else context.received_at
        try:
            result=self.transaction.feed(payload)
        except (ValueError,UnicodeError) as error:
            self.errors+=1;self.transaction=None;self.records=()
            self.tree.delete(*self.tree.get_children());self.detail.set("注册信息未通过校验")
            self.status.set(f"{error} · 累计 {self.errors}");return
        if len(payload)>=HEADER.size and HEADER.unpack_from(payload)[3]==self.nonce:
            self.first_received=received if self.first_received is None else min(self.first_received,received)
        if result is not None:
            self.transaction=None;self.records=result;self.last_rx=self.first_received
            self._render()

    def handle_line(self,line):
        if self.transaction is not None and line.startswith("ERR unknown cmd REGISTRY?"):
            self.transaction=None;self.unsupported=True;self.records=()
            self.tree.delete(*self.tree.get_children());self.status.set("固件未提供元件注册接口")

    def _render(self):
        selected=self.tree.selection()
        self.tree.delete(*self.tree.get_children())
        stale=self.last_rx is None or time.monotonic()-self.last_rx>3.0
        self._stale_rendered=stale
        for r in self.records:
            state,tag=STATE[r.state] if not stale else ("回包已过期","warn")
            self.tree.insert("","end",iid=str(r.id),text=r.name,values=(r.model,state,r.stage,
                              "；".join(field_text(f) for f in r.fields) if not stale else "—",r.code,r.bus),tags=(tag,))
        if selected and self.tree.exists(selected[0]):self.tree.selection_set(selected)
        self.status.set("注册回包已过期" if stale else f"飞控注册 {len(self.records)} 个元件 · 校验完整")
        self.refresh_detail()

    def refresh_detail(self):
        selected=self.tree.selection()
        r=next((x for x in self.records if selected and str(x.id)==selected[0]),None)
        if r is None:self.detail.set("选择元件查看固件报告");return
        stale=self.last_rx is None or time.monotonic()-self.last_rx>3.0
        self.detail.set(f"{r.name} / {r.model}\n接口：{r.bus}　阶段：{r.stage}　诊断码：{r.code}\n"
                        f"采样计数：{r.samples}　回包时采样年龄：{r.age_ms if r.age_ms is not None else '未知'} ms\n"
                        +("回包过期，读数不可用" if stale else "\n".join(field_text(f) for f in r.fields))
                        +f"\n{r.note}")

    def _tick(self):
        if self._disposed:return
        now=time.monotonic();connected=self._connection()
        self.refresh_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        if not connected:self.status.set("未连接飞控")
        elif self.transaction is not None and now-self.started>2.0:
            self.transaction=None;self.errors+=1
            self.status.set("未收到完整注册清单，请确认固件版本或链路")
        if self.records and not self._stale_rendered and self.last_rx is not None and now-self.last_rx>3.0:self._render()
        visible=bool(self.winfo_ismapped())
        if visible and not self.was_visible:self.needs_initial=True
        self.was_visible=visible
        # A full catalog is a one-shot diagnostic, not another continuous stream.
        # Repeating ~1.8 kB each second would consume the UART telemetry reserve.
        if (connected and visible and self.needs_initial and not self.unsupported
                and self.transaction is None):self.request()
        self._timer=self.after(250,self._tick)

    def _destroy(self,event):
        if event.widget is self:
            self._disposed=True
            if self._timer is not None:self.after_cancel(self._timer);self._timer=None


def mount_overview(panel,parent):
    page=OverviewPage(parent,panel);page.pack(fill=tk.BOTH,expand=True)
    panel.overview_page=page;panel.module_tree=page.tree
    # Legacy text diagnostics can continue to feed dedicated pages, but may not
    # invent or overwrite rows in the firmware-owned overview.
    panel.module_state={};panel.selected_module=None
