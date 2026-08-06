from __future__ import annotations

from pathlib import Path

from tools import flight_log_waveform_ui as ui


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_waveform_ui_helpers_do_not_start_tk(tmp_path: Path) -> None:
    first = tmp_path / "flightlog_old.csv"
    second = tmp_path / "flightlog_new.csv"
    first.write_text("timestamp_us,roll_deg\n0,0\n", encoding="utf-8")
    second.write_text("timestamp_us,pitch_deg\n0,0\n", encoding="utf-8")

    files = ui.discover_csv_files(tmp_path)

    assert {path.name for path in files} == {"flightlog_old.csv", "flightlog_new.csv"}
    assert ui.human_size(1024) == "1.0 KB"
    assert ui.choose_time_column(["sequence", "timestamp_us", "roll_deg"]) == "timestamp_us"
    assert ui.choose_time_column(["sequence", "roll_deg"]) == "记录序号"
    assert ui.channel_matches_group("servo_alpha_feedback_us", "舵机")
    assert ui.channel_matches_group("flow_corrected_velocity_m_s_0", "光流")
    assert ui.downsample_indices(1000, 100).step == 10
    assert ui.zoom_limits(0.0, 10.0, 5.0, 0.5) == (2.5, 7.5)
    assert ui.zoom_limits(10.0, 0.0, 5.0, 0.5) == (7.5, 2.5)
    assert ui.WHEEL_ZOOM_MODES == ("总缩放", "横轴", "纵轴")


def test_waveform_ui_exposes_folder_file_channel_plot_workflow() -> None:
    source = read("tools/flight_log_waveform_ui.py")

    assert "class FlightLogWaveformUI(tk.Tk)" in source
    assert '"选择文件夹"' in source
    assert '"日志文件"' in source
    assert '"通道选择"' in source
    assert '"画选中"' in source
    assert '"分图"' in source
    assert '"归一化"' in source
    assert '"滚轮"' in source
    assert '"总缩放"' in source
    assert '"横轴"' in source
    assert '"纵轴"' in source
    assert '"选中通道统计"' in source
    assert "filedialog.askdirectory" in source
    assert "filedialog.askopenfilename" in source
    assert "pd.read_csv" in source
    assert 'mpl_connect("scroll_event", self._on_scroll_zoom)' in source
    assert "def _on_scroll_zoom" in source
    assert "zoom_limits" in source
    assert "中文注释" in source
