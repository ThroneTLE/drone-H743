"""R-M1-3 block-erase timing measurement and throughput contracts."""

from pathlib import Path
import importlib.util


ROOT = Path(__file__).resolve().parents[1]


def _load_analysis():
    path = ROOT / "tools/flash_timing_analysis.py"
    spec = importlib.util.spec_from_file_location("flash_timing_analysis", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_firmware_probe_is_thin_and_keeps_normal_polling_default() -> None:
    header = (ROOT / "Driver/Inc/drv_gd25q32_timing_probe.h").read_text(encoding="utf-8")
    source = (ROOT / "Driver/Src/drv_gd25q32_timing_probe.c").read_text(encoding="utf-8")
    driver = (ROOT / "Driver/Src/drv_gd25q32.c").read_text(encoding="utf-8")
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "DRV_GD25Q32_TIMING_SAMPLE_CAPACITY 32U" in header
    assert "g_drv_gd25q32_timing_probe" in header
    assert "tight_poll_enabled" in header
    assert ".tight_poll_enabled = 0U" in source
    assert "DRV_GD25Q32_TimingProbe_RecordBlock" in source
    assert "DRV_GD25Q32_TimingProbe_RecordPage" in source
    assert "DRV_GD25Q32_TimingProbe_TightPollEnabled" in driver
    assert "#define GD25Q32_BUSY_POLL_DELAY_MS     1U" in driver
    assert "App/Src/app_control.c" in cmake
    assert "Driver/Src/drv_gd25q32_timing_probe.c" in cmake
    assert "FLASH TIMING" not in (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8")

    flight_log = (ROOT / "App/Src/app_flight_log.c").read_text(encoding="utf-8")
    assert "#define APP_FLIGHT_LOG_QUEUE_CAPACITY     32U" in flight_log
    assert "#define APP_FLIGHT_LOG_WRITE_BATCH_RECORDS 4U" in flight_log
    assert "sizeof(APP_FlightLogRecord) == 528U" in flight_log


def test_page_program_count_matches_real_sector_layout_and_batching() -> None:
    analysis = _load_analysis()
    assert analysis.page_programs_per_sector(
        sector_size=4096,
        header_size=256,
        record_size=528,
        batch_records=4,
        page_size=256,
    ) == 17


def test_throughput_uses_worst_samples_and_includes_background_waits() -> None:
    analysis = _load_analysis()
    result = analysis.block_throughput(
        block_size=65536,
        sector_size=4096,
        header_size=256,
        record_size=528,
        batch_records=4,
        page_size=256,
        block_erase_samples_us=[210_000, 250_000, 235_000],
        page_program_samples_us=[420, 700, 510],
        background_wait_ms=5,
        required_rate_hz=250.0,
        required_margin_fraction=0.20,
    )

    # 16 sectors * 7 records, using max erase=250ms and max page=700us.
    assert result["logical_sectors"] == 16
    assert result["records_per_sector"] == 7
    assert result["page_programs_per_sector"] == 17
    assert result["erase_worst_us"] == 250_000
    assert result["page_program_worst_us"] == 700
    assert result["background_waits_per_sector"] == 2
    assert result["cycle_us"] == 600_400
    assert abs(result["throughput_hz"] - (112_000_000 / 600_400)) < 1e-9
    assert result["mean_page_throughput_hz"] > result["throughput_hz"]
    assert result["no_wait_mean_page_throughput_hz"] > result["throughput_hz"]
    assert result["go"] is False


def test_go_requires_twenty_percent_margin_not_merely_250hz() -> None:
    analysis = _load_analysis()
    common = dict(
        block_size=65536,
        sector_size=4096,
        header_size=256,
        record_size=528,
        batch_records=4,
        page_size=256,
        page_program_samples_us=[100],
        background_wait_ms=5,
        required_rate_hz=250.0,
        required_margin_fraction=0.20,
    )
    barely = analysis.block_throughput(block_erase_samples_us=[220_000], **common)
    clear = analysis.block_throughput(block_erase_samples_us=[150_000], **common)
    assert barely["throughput_hz"] > 250.0
    assert barely["go"] is False
    assert clear["throughput_hz"] >= 300.0
    assert clear["go"] is True
