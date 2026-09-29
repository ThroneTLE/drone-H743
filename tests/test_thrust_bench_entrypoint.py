"""Both advertised commands open the original extended pressure application."""
from tools.thrust_bench import __main__ as entry


def test_package_gui_entry_uses_original_pressure_window(monkeypatch):
    from tools import pressure_rs485_gui
    opened = []
    monkeypatch.setattr(pressure_rs485_gui, "main", lambda: opened.append("pressure"))
    assert entry.main([]) == 0
    assert opened == ["pressure"]


def test_offline_parser_keeps_report_paths(tmp_path):
    args = entry.parser().parse_args(["analyze", str(tmp_path / "samples.csv"),
                                     "--metadata", str(tmp_path / "metadata.json"),
                                     "--output", str(tmp_path / "analysis")])
    assert args.command == "analyze"
    assert args.samples == tmp_path / "samples.csv"
    assert args.metadata == tmp_path / "metadata.json"
    assert args.output == tmp_path / "analysis"
