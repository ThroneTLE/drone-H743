"""Reproduce Python's script-path layout, without repository-root PYTHONPATH."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def test_direct_script_builds_panel_from_an_unrelated_working_directory(tmp_path):
    code = r'''
import runpy, sys, tkinter as tk
from pathlib import Path
script=Path(sys.argv[1])
# A direct python /path/tools/drone_tcp_panel.py supplies tools/, not repo/.
sys.path.insert(0,str(script.parent))
ns=runpy.run_path(str(script),run_name="cli_startup_regression")
Panel=ns["DronePanel"]
for name in ("_restore_last_connection","_save_panel_state",
             "_validation_load_latest_artifact","_v1_load_latest_session"):
    setattr(Panel,name,lambda self:None)
Panel._load_panel_state=lambda self:{}
original=tk.Tk.__init__
def hidden(self,*args,**kwargs):
    original(self,*args,**kwargs)
    self.withdraw()
tk.Tk.__init__=hidden
app=Panel()
assert app.simulation_bar.start_button.cget("text")
app.destroy()
print("DIRECT_SCRIPT_WINDOW_OK")
'''
    result = subprocess.run([sys.executable, '-I', '-c', code,
                             str(ROOT/'tools/drone_tcp_panel.py')],
                            cwd=tmp_path, capture_output=True, text=True,
                            encoding='utf-8', errors='replace', timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'DIRECT_SCRIPT_WINDOW_OK' in result.stdout


def test_catalog_import_does_not_load_simulator_runtime(tmp_path):
    code = r'''
import sys
sys.path.insert(0,sys.argv[1])
from sim_xz.control_catalog import EXPERIMENT_LABELS
assert "height_step" in EXPERIMENT_LABELS.values()
assert "sim_xz.protocol" not in sys.modules
assert "sim_xz.controller_bridge" not in sys.modules
'''
    result=subprocess.run([sys.executable,'-I','-c',code,str(ROOT/'tools')],
                          cwd=tmp_path,capture_output=True,text=True,timeout=15)
    assert result.returncode == 0, result.stdout+result.stderr


def test_package_public_exports_still_resolve():
    from tools.sim_xz import ControllerBridge, XZPlant, SimulatorProtocol
    assert ControllerBridge.__module__.endswith('controller_bridge')
    assert XZPlant.__module__.endswith('physics')
    assert SimulatorProtocol.__module__.endswith('protocol')
