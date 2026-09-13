"""Tk Bluetooth selection and cancellable discovery; uses the existing serial session."""
import queue
import threading
import tkinter as tk
from tkinter import ttk
from .bluetooth_devices import choose_device,discover_devices


def channel_mode(panel):
    value=panel.transport_var.get()
    return 'bluetooth' if value in ('蓝牙','bluetooth') else value


class BluetoothChannelMixin:
    def _build_bluetooth_controls(self,parent):
        self._bt_generation=0;self._bt_scanning=False;self._bt_connect_pending=False
        self._bt_events=queue.Queue();self._bt_devices={};self._bt_opening=False
        self._bt_preferred='';self._bt_after=None;self._bt_open_after=None;self._bt_closed=False
        self._bt_preferred_port=''
        self.bluetooth_device_var=tk.StringVar(self,value='')
        self._bluetooth_frame=ttk.Frame(parent)
        ttk.Label(self._bluetooth_frame,text='蓝牙设备').pack(side=tk.LEFT)
        self._bluetooth_combo=ttk.Combobox(self._bluetooth_frame,textvariable=self.bluetooth_device_var,
                                         state='readonly',width=34)
        self._bluetooth_combo.pack(side=tk.LEFT,padx=(4,4))
        self._bluetooth_refresh=ttk.Button(self._bluetooth_frame,text='自动查找',command=self._refresh_bluetooth)
        self._bluetooth_refresh.pack(side=tk.LEFT)
        ttk.Label(self._bluetooth_frame,text='115200').pack(side=tk.LEFT,padx=6)
        self.bind('<Destroy>',self._bluetooth_destroy,add='+')
        self._bt_tick()

    def _bluetooth_destroy(self,event):
        if event.widget is self:
            self._cancel_bluetooth();self._bt_closed=True
            if self._bt_after is not None:
                self.after_cancel(self._bt_after);self._bt_after=None

    def _cancel_bluetooth(self):
        if not hasattr(self,'_bt_generation'):return
        if self._bt_opening and not self.serial_transport.is_connected:
            self.serial_transport.stop()
        if self._bt_open_after is not None:
            self.after_cancel(self._bt_open_after);self._bt_open_after=None
        self._bt_generation+=1;self._bt_scanning=False;self._bt_connect_pending=False
        self._bt_opening=False

    def _refresh_bluetooth(self):
        self._request_bluetooth_scan()

    def _request_bluetooth_scan(self,connect=False,preferred=None):
        if channel_mode(self)!='bluetooth':return
        if self._transport_connected():
            self.autoconnect_var.set('请先停止当前连接，再选择蓝牙设备');return
        self._bt_connect_pending |= bool(connect)
        selected=self._bt_devices.get(self.bluetooth_device_var.get())
        self._bt_preferred=(preferred if preferred is not None else
                            (selected.address if selected else str(self._panel_state.get('bluetooth_address') or '')))
        self._bt_preferred_port=selected.port if selected else str(self._panel_state.get('bluetooth_port') or '')
        if self._bt_scanning:return
        self._bt_generation+=1;generation=self._bt_generation;self._bt_scanning=True
        self.autoconnect_var.set('正在识别已配对的蓝牙设备…')
        events=self._bt_events
        def discover():
            try:events.put((generation,discover_devices(),None))
            except Exception as error:events.put((generation,(),str(error)))
        threading.Thread(target=discover,daemon=True).start()

    def _bt_tick(self):
        if self._bt_closed:return
        try:
            while True:
                generation,devices,error=self._bt_events.get_nowait()
                if generation!=self._bt_generation or channel_mode(self)!='bluetooth':continue
                self._bt_scanning=False
                if error:
                    self._bt_devices={};self.bluetooth_device_var.set('');self._bluetooth_combo.configure(values=())
                    self._bt_connect_pending=False;self.autoconnect_var.set(f'蓝牙识别失败：{error}');continue
                self._bt_devices={d.label:d for d in devices}
                self._bluetooth_combo.configure(values=tuple(self._bt_devices))
                device,note=choose_device(devices,self._bt_preferred,self._bt_preferred_port)
                self.bluetooth_device_var.set(device.label if device else '')
                self.autoconnect_var.set(f'{note}'+(f'：{device.label}' if device else ''))
                connect=self._bt_connect_pending;self._bt_connect_pending=False
                if connect and device is not None and device.available:self._open_bluetooth(device)
        except queue.Empty:pass
        connected=self._transport_connected()
        self._bluetooth_combo.configure(state=tk.DISABLED if connected or self._bt_opening else 'readonly')
        self._bluetooth_refresh.configure(state=tk.DISABLED if connected or self._bt_scanning or self._bt_opening else tk.NORMAL)
        if hasattr(self,'_transport_combo'):
            self._transport_combo.configure(state=tk.DISABLED if connected or self._bt_opening else 'readonly')
        if self._bt_opening and self.serial_transport.is_connected:
            self._bt_opening=False;self._save_panel_state()
            self.autoconnect_var.set(f'蓝牙已连接：{self.serial_transport.active_port}')
        self._bt_after=self.after(50,self._bt_tick)

    def _open_bluetooth(self,device):
        # The scan verified the current port/address association before opening.
        self._stop();self.transport=self.serial_transport
        self._serial_port_identity[device.port]=device.identity()
        self.serial_baud_var.set(115200)
        self._bt_opening=True;generation=self._bt_generation
        self.autoconnect_var.set(f'正在连接蓝牙：{device.label}')
        starter=getattr(self.serial_transport,'start_async',self.serial_transport.start)
        starter(device.port,115200)
        def finish_open():
            if generation!=self._bt_generation:return
            self._bt_open_after=None
            if not self.serial_transport.is_connected:
                self.serial_transport.stop()  # Cancel a driver open that returns after this deadline.
                self._bt_opening=False
                self.autoconnect_var.set('蓝牙未连接，请检查设备是否已上电，或点击停止后重试')
        self._bt_open_after=self.after(5000,finish_open)
