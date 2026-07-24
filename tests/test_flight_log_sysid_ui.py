from __future__ import annotations

from pathlib import Path

from tools import flight_log_sysid_ui as ui


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_ui_helpers_are_importable_without_starting_tk() -> None:
    assert ui.format_cell(None) == ""
    assert ui.format_cell(1.23456, 2) == "1.23"
    assert ui.default_report_dir(Path("flightlog_demo.csv")) == Path(".tmp") / "sysid_flightlog_demo"


def test_ui_exposes_expected_flight_log_views() -> None:
    source = read("tools/flight_log_sysid_ui.py")

    assert "class FlightLogSysidUI(tk.Tk)" in source
    assert "ttk.Notebook" in source
    assert '"参数组"' in source
    assert '"时间片段"' in source
    assert '"执行器拟合"' in source
    assert '"通道统计"' in source
    assert '"图表"' in source
    assert '"风险提示"' in source
    assert '"打开CSV"' in source
    assert "中文注释" in source
    assert "translate_flag" in source
    assert "sysid.analyze_flight_log" in source
    assert "sysid.write_reports" in source
    assert "filedialog.askopenfilename" in source
