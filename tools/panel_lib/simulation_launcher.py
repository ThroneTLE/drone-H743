"""Own only the loopback listener and child process started by the simulation bar."""
from __future__ import annotations

from datetime import datetime
from pathlib import Path
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk


try:
    from ..sim_xz.control_catalog import EXPERIMENT_LABELS
except ImportError:
    from sim_xz.control_catalog import EXPERIMENT_LABELS

ROOT = Path(__file__).resolve().parents[2]


class SimulationBar(ttk.LabelFrame):
    def __init__(self, parent, panel, *, process_factory=None, headless=False):
        super().__init__(parent, text='仿真 · X–Z / 水平 P—PID—P—PID · 高度 P—PID', padding=(10, 6))
        self.panel = panel
        self.process_factory = process_factory or subprocess.Popen
        self.headless = headless
        self.process = None
        self.listener = None
        self.workspace = None
        self.workspaces = []
        self.pending = False
        self.connected_once = False
        self.log_file = None
        self.log_path = None
        self.timer = None
        self.deadline = 0.0
        self.disposed = False
        self.status = tk.StringVar(value='本机运行，无需飞控硬件')
        self.link_status = tk.StringVar(value='正在准备本机模拟器')
        self.previous_link_variable = None
        self.start_button = ttk.Button(self, text='一键启动并连接', command=self.start,
                                       style='Primary.TButton')
        self.start_button.pack(side='left')
        self.stop_button = ttk.Button(self, text='停止仿真', command=self.stop, state='disabled')
        self.stop_button.pack(side='left', padx=8)
        self.experiment_var = tk.StringVar(value=next(iter(EXPERIMENT_LABELS)))
        self.experiment_combo = ttk.Combobox(self, textvariable=self.experiment_var,
                                             values=list(EXPERIMENT_LABELS), state='readonly', width=17)
        self.experiment_combo.pack(side='left', padx=(0,8))
        ttk.Label(self, textvariable=self.status).pack(side='left', padx=8)
        panel.bind('<Destroy>', self._destroyed, add='+')

    def _buttons(self, active):
        self.start_button.state(['disabled'] if active else ['!disabled'])
        self.stop_button.state(['!disabled'] if active else ['disabled'])
        self.experiment_combo.configure(state='disabled' if active else 'readonly')

    def start(self):
        if self.pending or self.process is not None:
            return
        p = self.panel
        if any(t.is_connected for t in (p.serial_transport, p.udp_transport, p.tcp_transport)):
            self.status.set('已有设备连接：请先断开，再启动本机仿真')
            return
        if getattr(p, 'validation_session_active', False):
            self.status.set('请先结束校准/验收会话，再启动仿真')
            return
        if getattr(p, 'v1_worker', None) is not None and p.v1_worker.is_alive():
            self.status.set('采集任务仍在运行，请先结束采集')
            return
        logs = getattr(p, 'logs_page', None)
        if logs is not None and not logs.allow_main_connect():
            self.status.set('日志任务占用连接，请先结束日志任务')
            return
        self.previous = (p.transport_var.get(), p.host_var.get(), p.port_var.get())
        p.transport_var.set('tcp')
        p.host_var.set('127.0.0.1')
        p.port_var.set(0)  # Let the OS reserve a free port; do not probe then race a bind.
        p._start()
        self.generation = p.tcp_transport._send_generation
        self.pending = True
        self.previous_link_variable = p.link_status_label.cget('textvariable')
        p.link_status_label.configure(textvariable=self.link_status)
        self.connected_once = False
        self.deadline = time.monotonic() + 30
        self.status.set('正在准备本机连接与仿真窗口…')
        self._buttons(True)
        self._schedule()

    def _schedule(self):
        if not self.disposed:
            self.timer = self.after(100, self._poll)

    def _owns_listener(self):
        tcp = self.panel.tcp_transport
        return (self.panel.transport is tcp and tcp._send_generation == self.generation
                and (self.listener is None or tcp.sock is self.listener))

    def _spawn(self, port):
        # Import at action time so the panel's existing test/data isolation applies.
        try:
            from ..project_paths import SIMULATION_DIR
        except ImportError:
            from project_paths import SIMULATION_DIR
        folder = SIMULATION_DIR / datetime.now().strftime('%Y-%m-%d')
        folder.mkdir(parents=True, exist_ok=True)
        self.log_path = folder / f'launcher_{datetime.now():%H%M%S_%f}.log'
        self.log_file = self.log_path.open('w', encoding='utf-8')
        command = [sys.executable, '-m', 'tools.sim_xz', '--host', '127.0.0.1',
                   '--port', str(port), '--autostart', '--experiment', EXPERIMENT_LABELS[self.experiment_var.get()]]
        if self.headless:
            command.append('--headless')
        self.process = self.process_factory(command, cwd=str(ROOT), stdin=subprocess.DEVNULL,
            stdout=self.log_file, stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))

    def _poll(self):
        self.timer = None
        if self.disposed or not self.pending:
            return
        if not self._owns_listener():
            self.stop('连接已切换，仿真已停止')
            return
        tcp = self.panel.tcp_transport
        try:
            if self.process is None:
                with tcp.lock:
                    listener = tcp.sock
                    port = listener.getsockname()[1] if listener else None
                if port is not None:
                    self.listener = listener
                    self.panel.port_var.set(port)
                    self._spawn(port)
            elif self.process.poll() is not None:
                code = self.process.returncode
                self.stop('仿真窗口已关闭' if code == 0 else f'仿真退出（{code}），详见 {self.log_path.name}')
                return
            if tcp.is_connected:
                if not self.connected_once:
                    self.connected_once = True
                    # Use the existing receive queue and dashboard decoder, not a second link.
                    tcp.send_line('CAPS?')
                    tcp.send_line('PARAM?')
                    tcp.send_line('TELEM?')
                    self._install_workspace()
                    self.panel.notebook.select(self.panel.dashboard_tab)
                self.link_status.set('本机模拟器已连接 · 当前为仿真数据，无实机')
                self.status.set(f'仿真已启动 · 127.0.0.1:{self.panel.port_var.get()} · 参数在上位机调节')
            elif self.connected_once:
                self.stop('仿真连接已断开，请重新启动')
                return
            elif time.monotonic() > self.deadline:
                self.stop('启动超时；检查仿真日志与本机编译器')
                return
        except (OSError, RuntimeError) as exc:
            self.stop(f'无法启动仿真：{exc}')
            return
        self._schedule()

    def _install_workspace(self):
        from .simulation_workspaces import simulation_workspaces
        p = self.panel
        self.previous_workspace = p.dashboard_layout.active
        self.workspaces = simulation_workspaces()
        self.workspace = self.workspaces[0]
        p.dashboard_layout.active = len(p.dashboard_layout.workspaces)
        if EXPERIMENT_LABELS[self.experiment_var.get()] == 'height_step':
            p.dashboard_layout.active += 1
        p.dashboard_layout.workspaces.extend(self.workspaces)
        p.dashboard_workspace_var.set(p.dashboard_layout.active)
        p._dashboard_rebuild_workspace_bar()
        p._dashboard_rebuild_tiles()

    def _remove_workspace(self):
        p = self.panel
        if self.workspaces:
            for workspace in self.workspaces:
                if workspace in p.dashboard_layout.workspaces:
                    p.dashboard_layout.workspaces.remove(workspace)
            p.dashboard_layout.active = min(self.previous_workspace, len(p.dashboard_layout.workspaces)-1)
            p.dashboard_workspace_var.set(p.dashboard_layout.active)
            p._dashboard_rebuild_workspace_bar()
            p._dashboard_rebuild_tiles()
            p._dashboard_persist()
        self.workspace = None
        self.workspaces = []

    @staticmethod
    def _reap(process):
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def stop(self, message='仿真已停止'):
        if self.timer is not None:
            try:
                self.after_cancel(self.timer)
            except tk.TclError:
                pass
            self.timer = None
        owns = self.pending and self._owns_listener()
        self.pending = False
        child, self.process = self.process, None
        if child is not None:
            if child.poll() is None:
                child.terminate()
            threading.Thread(target=self._reap, args=(child,), daemon=True).start()
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        if owns:
            self.panel.tcp_transport.stop()
            if not self.disposed:
                self.panel._parameter_on_disconnect()
                mode, host, port = self.previous
                self.panel.transport_var.set(mode)
                self.panel.host_var.set(host)
                self.panel.port_var.set(port)
                self.panel.transport = self.panel._current_transport()
        self.listener = None
        if not self.disposed:
            if self.previous_link_variable is not None:
                self.panel.link_status_label.configure(textvariable=self.previous_link_variable)
                self.previous_link_variable = None
            self._remove_workspace()
            self.status.set(message)
            self._buttons(False)

    def _destroyed(self, event):
        if event.widget is self.panel:
            self.disposed = True
            self.stop()


def mount_simulation_bar(panel, parent):
    panel.simulation_bar = SimulationBar(parent, panel)
    panel.simulation_bar.pack(fill='x', pady=(8, 0))
