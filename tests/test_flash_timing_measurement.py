"""R-M1-3 block-erase timing measurement and throughput contracts."""

from pathlib import Path
import importlib.util
import sys


ROOT = Path(__file__).resolve().parents[1]


def _load_analysis():
    path = ROOT / "tools/flash_timing_analysis.py"
    spec = importlib.util.spec_from_file_location("flash_timing_analysis", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_capture():
    tools = str(ROOT / "tools")
    if tools not in sys.path:
        sys.path.insert(0, tools)
    path = ROOT / "tools/flash_timing_capture.py"
    spec = importlib.util.spec_from_file_location("flash_timing_capture_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_firmware_probe_is_thin_and_keeps_normal_polling_default() -> None:
    header = (ROOT / "Driver/Inc/drv_gd25q32_timing_probe.h").read_text(encoding="utf-8")
    source = (ROOT / "Driver/Src/drv_gd25q32_timing_probe.c").read_text(encoding="utf-8")
    driver = (ROOT / "Driver/Src/drv_gd25q32.c").read_text(encoding="utf-8")
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "DRV_GD25Q32_TIMING_SAMPLE_CAPACITY 64U" in header
    assert "g_drv_gd25q32_timing_probe" in header
    assert "tight_poll_enabled" in header
    assert ".tight_poll_enabled = 0U" in source
    assert "DRV_GD25Q32_TimingProbe_RecordBlock" in source
    assert "erase_4k" in header
    assert "block_size == DRV_GD25Q32_SECTOR_SIZE" in source
    assert "DRV_GD25Q32_TimingProbe_RecordPage" in source
    assert "DRV_GD25Q32_TimingProbe_RecordSuspend" in source
    assert "DRV_GD25Q32_TimingProbe_RecordResume" in source
    assert ".suspend_resume_enabled = 0U" in source
    assert "DRV_GD25Q32_TimingProbe_TightPollEnabled" in driver
    assert "GD25Q32_CMD_PROGRAM_ERASE_SUSPEND 0x75U" in driver
    assert "GD25Q32_CMD_PROGRAM_ERASE_RESUME  0x7AU" in driver
    assert "GD25Q32_STATUS2_SUS1" in driver
    assert "gd25q32_timing_probe_suspend_resume" in driver
    assert "#define GD25Q32_BUSY_POLL_DELAY_MS     1U" in driver
    assert "App/Src/app_control.c" in cmake
    assert "Driver/Src/drv_gd25q32_timing_probe.c" in cmake
    assert "FLASH TIMING" not in (ROOT / "App/Src/app_control.c").read_text(encoding="utf-8")

    flight_log = (ROOT / "App/Src/app_flight_log.c").read_text(encoding="utf-8")
    assert "#define APP_FLIGHT_LOG_QUEUE_CAPACITY     64U" in flight_log
    assert "#define APP_FLIGHT_LOG_WRITE_BATCH_RECORDS 4U" in flight_log
    assert "sizeof(APP_FlightLogRecord) == 808U" in flight_log


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


def test_cooperative_throughput_accounts_for_every_page_and_interruption() -> None:
    analysis = _load_analysis()
    result = analysis.cooperative_block_throughput(
        block_size=32768,
        sector_size=4096,
        header_size=256,
        record_size=528,
        batch_records=4,
        page_size=256,
        block_erase_samples_us=[100_000, 104_205],
        page_program_samples_us=[800, 1_006],
        suspend_samples_us=[30, 40],
        resume_samples_us=[20, 25],
        required_rate_hz=250.0,
        required_margin_fraction=0.20,
    )

    # 8 logical sectors, each opened/written in two background batches.
    assert result["logical_sectors"] == 8
    assert result["records_per_block"] == 56
    assert result["page_programs_per_block"] == 8 * 17
    assert result["interruptions_per_block"] == 8 * 2
    expected_us = 104_205 + 8 * 17 * 1_006 + 16 * (40 + 25)
    assert result["cycle_us"] == expected_us
    assert abs(result["throughput_hz"] - 56_000_000 / expected_us) < 1e-9
    assert result["required_with_margin_hz"] == 300.0
    assert result["go"] is False


def test_even_zero_cost_suspend_resume_must_clear_the_300hz_gate() -> None:
    analysis = _load_analysis()
    result = analysis.cooperative_block_throughput(
        block_size=65536,
        sector_size=4096,
        header_size=256,
        record_size=528,
        batch_records=4,
        page_size=256,
        block_erase_samples_us=[152_714],
        page_program_samples_us=[1_299],
        suspend_samples_us=[0],
        resume_samples_us=[0],
        required_rate_hz=250.0,
        required_margin_fraction=0.20,
    )

    assert result["zero_handshake_upper_bound_hz"] < 250.0
    assert result["throughput_hz"] == result["zero_handshake_upper_bound_hz"]
    assert result["go"] is False


def test_queue_peak_model_includes_unaligned_region_fringe_and_startup() -> None:
    analysis = _load_analysis()
    result = analysis.worst_case_queue_peak(
        rate_hz=125,
        queue_capacity=32,
        batch_records=4,
        sector_size=4096,
        header_size=256,
        record_size=528,
        page_size=256,
        page_program_us=1_006,
        sector_erase_us=50_000,
        block_erase_us=104_205,
        background_wait_us=5_000,
        region_start=0x2000,
        region_end_excl=0x3FC000,
        block_size=32768,
    )

    assert result["head_fringe_sectors"] == 6
    assert result["tail_fringe_sectors"] == 4
    assert result["steady_wrap_sector_erases"] == 10
    assert result["startup_worst_sector_erases"] == 17
    assert result["queue_capacity"] == 32
    assert result["comfort_limit"] == 24
    assert result["peak_records"] > 0
    assert result["overflow_records"] >= 0


def test_capture_nudges_delayed_reply_without_repeating_destructive_command(monkeypatch) -> None:
    capture = _load_capture()

    class FakeTime:
        now = 0.0

        @classmethod
        def monotonic(cls):
            cls.now += 0.1
            return cls.now

    class FakeTransport:
        def __init__(self):
            self.writes = []

        def write_line(self, line):
            self.writes.append(line)

        def reset_input(self):
            raise AssertionError("a delayed valid reply must not be discarded")

    class FakeReader:
        def __init__(self):
            self.transport = FakeTransport()
            self.delivered = False

        def poll(self):
            if "PING" in self.transport.writes and not self.delivered:
                self.delivered = True
                return ["FLASH scratch_test erase_st=0 match=1"]
            return []

    monkeypatch.setattr(capture, "time", FakeTime)
    reader = FakeReader()
    transcript = []
    lines = capture._send_command(
        reader,
        "FLASH SCRATCH TEST 0x010000 4",
        ("FLASH scratch_test",),
        2.0,
        transcript,
    )

    assert reader.transport.writes == [
        "FLASH SCRATCH TEST 0x010000 4",
        "PING",
    ]
    assert lines == ["FLASH scratch_test erase_st=0 match=1"]
    assert transcript[0]["flush_sent"] is True
