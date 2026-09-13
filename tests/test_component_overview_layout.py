"""Real Tk overview selection, one-shot queries and scroll access at nine sizes/scales."""
import time
import pytest
from tools.panel_qa import OfflinePanel
from tools.panel_qa.geometry import SCALES, WINDOW_SIZES
from tools.panel_lib.component_registry import Component, Field


@pytest.mark.parametrize('scale',SCALES)
def test_overview_matrix_and_one_shot(scale):
    qa=OfflinePanel.launch(scale=scale)
    try:
        page=qa.panel.overview_page
        qa.select(next(leaf for leaf in qa.leaf_pages() if leaf.label=='总览'))
        page._tick()
        assert len([line for line in qa.transport.lines if line.startswith('REGISTRY?')])==1
        # Resolve a transaction as a presentation fixture (wire path tested separately).
        page.transaction=None;page.records=tuple(Component(i,1,0,0,1,f'元件 {i}',
             'FIRMWARE MODEL','SPI2','sampling','Firmware note',(Field('value','V',3,3.3),)) for i in range(1,17))
        page.last_rx=time.monotonic();page._render()
        page.last_attempt=time.monotonic()-5;page._tick()
        assert len([line for line in qa.transport.lines if line.startswith('REGISTRY?')])==1
        for width,height in WINDOW_SIZES:
            qa.resize(width,height);qa.panel.update_idletasks()
            assert page.refresh_button.winfo_ismapped()
            page.tree.see('16');page.tree.selection_set('16');page.refresh_detail()
            qa.panel.update_idletasks()
            assert page.tree.bbox('16'),(scale,width,height)
            assert '元件 16' in page.detail.get()
            assert page.tree.winfo_width()>100 and page.tree.winfo_height()>30
            page.tree.xview_moveto(1);assert page.tree.xview()[1]>0.99
            page.tree.xview_moveto(0)
        assert not qa.callback_errors
    finally:qa.destroy()
