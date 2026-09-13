"""Classic Bluetooth SPP discovery and identity matching. Enumeration never opens a port."""
from dataclasses import dataclass
import base64
import json
import re
import subprocess
import sys

SPP_UUID = '00001101-0000-1000-8000-00805F9B34FB'


def bluetooth_address(hwid: str) -> str:
    value=str(hwid or '').upper()
    if 'BTHENUM' not in value:
        return ''
    match=re.search(r'\\DEV_([0-9A-F]{12})(?:\\|$)',value)
    if match is None:
        match=re.search(r'&([0-9A-F]{12})_(?:C[0-9A-F]{8}|[0-9A-F]{8})$',value)
    address=match.group(1) if match else ''
    return '' if address=='000000000000' else address


@dataclass(frozen=True)
class BluetoothDevice:
    address: str
    name: str
    port: str
    hwid: str
    available: bool

    @property
    def label(self):
        state=self.port if self.available else f'{self.port or "无串口"} · 当前不可用'
        return f'{self.name} · {self.address[-6:]} ({state})'

    @property
    def is_micoair(self):
        return bool(re.match(r'^micoair743v2(?:[- _]|$)',self.name,re.I))

    def identity(self):
        return dict(device=self.port,description=self.name,hwid=self.hwid,vid=None,pid=None,
                    serial_number='',location='',bluetooth_address=self.address)


def windows_bluetooth_records():
    """Read paired-device/SPP PnP records. No inquiry, pairing, or radio connection."""
    if sys.platform != 'win32':
        return []
    script=r'''
$ErrorActionPreference='Stop'
[Console]::OutputEncoding=[System.Text.Encoding]::UTF8
$rows=@(Get-PnpDevice -Class Bluetooth,Ports |
    Where-Object { $_.InstanceId -match '^BTHENUM\\DEV_' -or $_.InstanceId -like '*00001101-0000-1000-8000-00805F9B34FB*' } |
    Select-Object @{n='name';e={$_.FriendlyName}},@{n='hwid';e={$_.InstanceId}})
ConvertTo-Json -InputObject $rows -Compress
'''
    encoded=base64.b64encode(script.encode('utf-16-le')).decode('ascii')
    result=subprocess.run(['powershell.exe','-NoProfile','-NonInteractive','-EncodedCommand',encoded],
                          capture_output=True,timeout=8,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    if result.returncode:
        raise OSError('Windows蓝牙设备枚举失败')
    records=json.loads(result.stdout.decode('utf-8-sig') or '[]')
    return records if isinstance(records,list) else [records]


def collect_devices(ports,records):
    """Join friendly device names to outgoing SPP ports by address, never COM history."""
    names={}
    for record in records:
        hwid=str(record.get('hwid') or '')
        address=bluetooth_address(hwid)
        if address and re.match(r'^BTHENUM\\DEV_',hwid,re.I):
            names[address]=re.sub(r'[\r\n\t]',' ',str(record.get('name') or '蓝牙设备'))[:96]
    live={str(port.device).casefold():port for port in ports}
    candidates={}
    for record in records:
        hwid=str(record.get('hwid') or '')
        if SPP_UUID not in hwid.upper():continue
        address=bluetooth_address(hwid)
        if not address:continue  # Incoming/listening COM ports have no remote address.
        match=re.search(r'\bCOM[0-9]+\b',str(record.get('name') or ''),re.I)
        if match:
            port=match.group(0).upper()
            candidates[(address,port)]=BluetoothDevice(address,names.get(address,'蓝牙设备'),port,hwid,port.casefold() in live)
    for port in ports:
        hwid=str(getattr(port,'hwid','') or '')
        address=bluetooth_address(hwid)
        if not address:continue
        key=(address,str(port.device).upper())
        name=names.get(address) or str(getattr(port,'description','') or '蓝牙设备')
        candidates[key]=BluetoothDevice(address,name,str(port.device),hwid,True)
    found=set(address for address,_ in candidates)
    for address,name in names.items():
        if address not in found and re.match(r'^micoair743v2(?:[- _]|$)',name,re.I):
            candidates[(address,'')]=BluetoothDevice(address,name,'',f'BTHENUM\\DEV_{address}',False)
    return tuple(sorted(candidates.values(),key=lambda d:(not d.is_micoair,not d.available,d.name,d.port)))


def choose_device(devices,remembered_address='',preferred_port=''):
    preferred=remembered_address.upper()
    if preferred:
        matches=[d for d in devices if d.address==preferred]
        if not matches:return None,'未找到上次的蓝牙设备'
    else:
        matches=[d for d in devices if d.is_micoair]
    active=[d for d in matches if d.available]
    if len(active)==1:return active[0],'已按蓝牙设备地址匹配'
    if preferred_port:
        selected=[d for d in active if d.port.casefold()==preferred_port.casefold()]
        if len(selected)==1:return selected[0],'已核对所选蓝牙端口的设备地址'
    if len(matches)==1:return matches[0],'已识别设备，蓝牙串口当前不可用'
    if matches:return None,'发现多个匹配设备，请选择要连接的飞控'
    return None,'未找到MicoAir蓝牙设备；请在Windows配对后刷新'


def discover_devices():
    from . import transport
    ports=[] if not transport.HAS_PYSERIAL else list(transport.serial.tools.list_ports.comports())
    return collect_devices(ports,windows_bluetooth_records())
