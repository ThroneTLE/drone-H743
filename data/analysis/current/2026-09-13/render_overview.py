"""Preview the real Tk page with ARM-emulated frames; never connect hardware."""
from pathlib import Path
import sys
import tempfile
import time
sys.path.insert(0,str(Path.cwd()))
from tools.panel_qa import OfflinePanel, isolated_environment
from tools.panel_qa.guards import hardware_guards
from tools.panel_lib.component_registry import RegistryTransaction

directory=Path(__file__).resolve().parent
wire=(directory/'registry-arm.bin').read_bytes()
transaction=RegistryTransaction(42)
rows=None
while wire:
    length=int.from_bytes(wire[6:8],'little')
    rows=transaction.feed(wire[8:8+length]);wire=wire[9+length:]
assert rows is not None
with tempfile.TemporaryDirectory() as temporary, hardware_guards(), isolated_environment(Path(temporary)):
    qa=OfflinePanel.launch(size=(1366,900))
    try:
        qa.select(next(leaf for leaf in qa.leaf_pages() if leaf.label=='总览'))
        page=qa.panel.overview_page
        page._connection();page.records=rows;page.last_rx=time.monotonic();page._render()
        page.tree.selection_set('5');page.refresh_detail()
        qa.screenshot(directory/'overview-offline.png')
        assert not qa.callback_errors
        print('Actual DronePanel preview: 13 firmware registry rows, injected ADC, no hardware')
    finally:qa.destroy()
