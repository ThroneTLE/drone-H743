"""R-M1-3 block-erase + queue contracts for flight-log pipeline."""

from pathlib import Path
import importlib.util


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def _load_analysis():
    path = ROOT / "tools/flash_timing_analysis.py"
    spec = importlib.util.spec_from_file_location("flash_timing_analysis", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_rate_and_trigger_divider_is_exact_125hz_quadrature() -> None:
    header = read("App/Inc/app_flight_log.h")
    stabilizer = read("App/Src/app_stabilizer.c")

    assert "#define APP_FLIGHT_LOG_RATE_HZ            125U" in header
    assert "(ctx->flight_log_divider & 0x03U) == 0U" in stabilizer
    assert "ctx->flight_log_divider = (uint8_t)((uint32_t)(ctx->flight_log_divider + 1U) & 0x03U);" in stabilizer
    lines = stabilizer.splitlines()
    observe_calls = [i for i, line in enumerate(lines) if "APP_FlightLog_Observe(" in line]
    divider_lines = [i for i, line in enumerate(lines) if "(ctx->flight_log_divider & 0x03U) == 0U" in line]
    assert len(divider_lines) == 1
    assert len(observe_calls) == 1
    assert divider_lines[0] < observe_calls[0]


def test_queue_capacity_is_128_and_record_queue_storage_matches() -> None:
    source = read("App/Src/app_flight_log.c")

    assert "#define APP_FLIGHT_LOG_QUEUE_CAPACITY     128U" in source
    assert "static APP_FlightLogRecord flight_log_queue[APP_FLIGHT_LOG_QUEUE_CAPACITY];" in source


def test_block_erase_only_for_head_tail_safe_32k_candidates_with_fallback() -> None:
    source = read("App/Src/app_flight_log.c")

    assert "static uint8_t flight_log_is_block32k_candidate(uint32_t sector_index)" in source
    assert "if (sector_index < APP_FLIGHT_LOG_BLOCK32K_HEAD_FRINGE_SECTORS) {" in source
    assert "if (block_end > (APP_FLIGHT_LOG_SECTOR_COUNT - APP_FLIGHT_LOG_BLOCK32K_TAIL_FRINGE_SECTORS)) {" in source
    assert "return ((address % APP_FLASH_SERVICE_BLOCK32K_SIZE) == 0U) ? 1U : 0U;" in source
    assert "if (flight_log_is_block32k_candidate(sector) != 0U) {" in source
    assert "st = APP_FlashService_EraseBlock32K(address);" in source
    assert "flight_log_prepared_block_ready = 0U;" in source
    assert "st = APP_FlashService_EraseSector(address);" in source


def test_block_erase_success_removes_exact_block_from_order_and_prevents_double_erase() -> None:
    source = read("App/Src/app_flight_log.c")

    assert "static void flight_log_order_remove_block(uint32_t sector_index)" in source
    assert "for (offset = 0U; offset < APP_FLIGHT_LOG_BLOCK32K_SECTOR_COUNT; ++offset) {" in source
    assert "flight_log_order_remove_sector(sector_index + offset);" in source

    # prepared_block tracks a current 32K prepared window.
    assert "if ((flight_log_prepared_block_ready != 0U) &&" in source
    assert "(flight_log_sector_in_prepared_block(sector) == 0U)" in source
    assert "flight_log_prepared_block_ready = 0U;" in source

    # On block-erase success, remove exactly this block from order once.
    assert "flight_log_prepared_block_ready = 1U;" in source
    assert "flight_log_prepared_block_sector = sector;" in source
    assert "flight_log_order_remove_block(sector);" in source


def test_block_erase_failure_falls_back_to_current_4k_and_clears_ready_flag() -> None:
    source = read("App/Src/app_flight_log.c")

    assert "flight_log_prepared_block_ready = 0U;" in source
    assert "st = APP_FlashService_EraseSector(address);" in source
    assert "if (st != APP_FLASH_SERVICE_OK) {\n                return 0U;\n            }" in source


def test_sector_geometry_and_region_safety_contracts_stay_4k_friendly() -> None:
    header = read("App/Inc/app_flight_log.h")
    source = read("App/Src/app_flight_log.c")
    flash_driver = read("Driver/Inc/drv_gd25q32.h")
    app_service = read("App/Inc/app_flash_service.h")
    receiver = read("tools/flight_log_receive.py")

    assert "#define APP_FLIGHT_LOG_SECTOR_HEADER_SIZE 256U" in header
    assert "#define APP_FLIGHT_LOG_RATE_HZ            125U" in header
    assert "#define APP_FLIGHT_LOG_VERSION            12U" in source
    assert "_Static_assert(sizeof(APP_FlightLogRecord) == 844U" in source
    assert "header->sector_size = APP_FLASH_SERVICE_SECTOR_SIZE;" in source
    assert "#define APP_FLASH_SERVICE_SECTOR_SIZE           DRV_GD25Q32_SECTOR_SIZE" in app_service
    assert "#define DRV_GD25Q32_SECTOR_SIZE           4096U" in flash_driver
    assert "SECTOR_SIZE = 4096" in receiver
    assert "_Static_assert(APP_FLIGHT_LOG_REGION_END_EXCL <=" in source
    assert "(APP_FLASH_SERVICE_SIZE_BYTES - (4UL * APP_FLASH_SERVICE_SECTOR_SIZE))" in source


def test_queue_peak_model_with_actual_125hz_parameters_stays_within_64_capacity_75_percent() -> None:
    analysis = _load_analysis()
    result = analysis.worst_case_queue_peak(
        rate_hz=125,
        queue_capacity=64,
        batch_records=4,
        sector_size=4096,
        header_size=256,
        record_size=512,
        page_size=256,
        page_program_us=1006,
        # 20-round on-device sector erase worst: 38_245 us (from
        # data/analysis/flight_log_flash_timing/2026-09-02/flash_sector_timing_capture_20260902_201214.json)
        sector_erase_us=38_245,
        block_erase_us=104_205,
        background_wait_us=5_000,
        region_start=0x2000,
        region_end_excl=0x3FC000,
        block_size=32768,
    )

    assert result["comfort_limit"] == 48
    assert result["head_fringe_sectors"] == 6
    assert result["tail_fringe_sectors"] == 4
    assert result["peak_records"] == 33
    assert result["worst_start_sector"] == 1007
    assert result["startup_worst_sector_erases"] == 17
    assert result["overflow_records"] == 0
    assert result["peak_records"] <= result["comfort_limit"]
