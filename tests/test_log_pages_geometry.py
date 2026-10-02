"""Three sizes x three simulated DPI scales, using the real panel and pages."""
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.mark.slow_ui  # 真面板尺寸/缩放矩阵，慢；默认只在界面文件有改动时跑（tests/conftest.py）
@pytest.mark.parametrize("scale", [1.0, 1.25, 1.5])
def test_log_controls_remain_reachable(scale):
    script = '''
from pathlib import Path
import tempfile,json,sys
from tools import panel_qa
from tools.panel_qa.geometry import collect_clipped_controls,WINDOW_SIZES
with tempfile.TemporaryDirectory() as temp:
 with panel_qa.hardware_guards() as guard, panel_qa.isolated_environment(Path(temp)):
  session=panel_qa.OfflinePanel.launch(scale=float(sys.argv[1]))
  try:
   session.panel.logs_page.show_receiver()
   reports=[]
   for size in WINDOW_SIZES:
    session.resize(*size)
    for page in session.leaf_pages():
     if page.label.startswith("日志 / "):
      session.select(page)
      reports.append({"page":page.label,"size":size,"clipped":[c.to_json() for c in collect_clipped_controls(session.panel,page.top)]})
   assert len(reports)==9
   assert guard.clean
   assert all(not row["clipped"] for row in reports),json.dumps(reports,ensure_ascii=False)
   assert not session.callback_errors,session.callback_errors
  finally:session.destroy()
'''
    result = subprocess.run([sys.executable, "-c", script, str(scale)],
                            cwd=Path(__file__).resolve().parents[1], capture_output=True,
                            text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
