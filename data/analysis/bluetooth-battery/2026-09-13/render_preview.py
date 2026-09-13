"""Real Tk layout evidence; never opens a physical transport or supplies measured values."""
from pathlib import Path
import sys
import tempfile
import time
sys.path.insert(0,str(Path.cwd()))
from tools.panel_qa import OfflinePanel,isolated_environment
from tools.panel_qa.guards import hardware_guards
from PIL import ImageGrab


def capture(qa,path):
    try:
        return qa.screenshot(path)
    except OSError:
        # Render only our own test window if the desktop cannot be captured.
        image=ImageGrab.grab(window=qa.panel.winfo_id())
        assert image.getbbox() is not None,'Empty window render'
        image.save(path)

output=Path(sys.argv[1]).resolve();output.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory() as tmp,hardware_guards(),isolated_environment(Path(tmp)):
    qa=OfflinePanel.launch(size=(1366,900),connected=False)
    sys.excepthook=sys.__excepthook__
    try:
        p=qa.panel
        if hasattr(p,'_bluetooth_combo'):
            p.transport_var.set('蓝牙')
            deadline=time.monotonic()+10
            while p._bt_scanning and time.monotonic()<deadline:
                p._bt_tick();time.sleep(.05)
            p.autoconnect_var.set('离线界面预览：只读取Windows配对记录，未连接目标板')
            capture(qa,output/'bluetooth-offline.png')
            qa.select(next(leaf for leaf in qa.leaf_pages() if leaf.label=='传感器 / 电池电压'))
            p.battery_page.notice.set('离线布局 · 没有实机电压数据')
            capture(qa,output/'battery-offline.png')
        else:
            p.autoconnect_var.set('改动前的离线界面 · 未连接目标板')
            capture(qa,output/'connection-before.png')
        assert not qa.callback_errors
        print('Offline layout images saved; no board commands')
    finally:qa.destroy()
