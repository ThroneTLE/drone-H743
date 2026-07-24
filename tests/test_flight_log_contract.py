from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_flight_log_region_leaves_reserved_flash_sectors() -> None:
    header = read("App/Inc/app_flight_log.h")
    source = read("App/Src/app_flight_log.c")

    assert "#define APP_FLIGHT_LOG_REGION_START       0x00002000UL" in header
    assert "#define APP_FLIGHT_LOG_REGION_END_EXCL    0x003FC000UL" in header
    assert "APP_CONTROL_FLASH_SCRATCH_ADDR (APP_FLASH_SERVICE_SIZE_BYTES - 4U * 4096UL)" in read("App/Src/app_control.c")
    assert "APP_CONTROL_CFG_ADDRESS     (APP_FLASH_SERVICE_SIZE_BYTES - 4096UL)" in read("App/Src/app_control.c")
    assert "APP_FLIGHT_LOG_REGION_END_EXCL <=" in source
    assert "last four reserved sectors" in source


def test_flight_log_uses_app_flash_service_only() -> None:
    header = read("App/Inc/app_flight_log.h")
    source = read("App/Src/app_flight_log.c")
    cmake = read("CMakeLists.txt")

    assert "App/Src/app_flight_log.c" in cmake
    assert "APP_FlashService_ReadData" in source
    assert "APP_FlashService_ReadDataFast" not in source
    assert "USART1 export is link-speed limited" in source
    assert "APP_FlashService_EraseSector" in source
    assert "APP_FlashService_WriteData" in source
    assert "BSP_Flash" not in source
    assert "DRV_GD25Q32" not in source
    assert "APP_FlashService" not in header


def test_stabilizer_records_snapshots_without_direct_flash_access() -> None:
    freertos = read("Core/Src/freertos.c")
    drv_header = read("Driver/Inc/drv_coax_ctrl.h")
    drv_source = read("Driver/Src/drv_coax_ctrl.c")

    assert '#include "app_flight_log.h"' in freertos
    assert "APP_FlightLog_Observe(flight_log_should_record ? &flog_snapshot : NULL" in freertos
    assert "DRV_COAX_CTRL_GetLastDebug(&flog_snapshot.ctrl_debug);" in freertos
    assert "flight_log_divider ^= 1U;" in freertos
    assert "STABILIZER_FLIGHT_LOG_TAIL_RECORDS" in freertos
    assert "flog_snapshot.motor_output_reason" in freertos
    assert "APP_FLIGHT_LOG_MOTOR_REASON_RC_LOSS_DISABLE" in freertos
    assert "msg.raw_imu              = raw;" in freertos
    assert "APP_FlashService_" not in freertos
    assert "typedef struct {\n    float pos_p_m_s2[3];" in drv_header
    assert "void DRV_COAX_CTRL_GetLastDebug(DRV_COAX_CTRL_Debug *debug);" in drv_header
    assert "debug->yaw_angle_p_rad_s" in drv_source
    assert "debug.yaw_torque_cmd =" in drv_source
    assert "coax_ctrl_last_debug = debug;" in drv_source


def test_background_task_drives_flight_log_slow_work() -> None:
    background = read("App/Src/app_background.c")

    assert '#include "app_flight_log.h"' in background
    assert "APP_FlightLog_BackgroundStep();" in background
    assert "APP_FLIGHT_LOG_BACKGROUND_IDLE_MS" in background
    assert "osWaitForever" not in background


def test_flog_commands_and_vofa_export_gate_are_reachable() -> None:
    control = read("App/Src/app_control.c")
    aiwb2 = read("App/Src/app_aiwb2.c")
    freertos = read("Core/Src/freertos.c")
    script = read("tools/flight_log_receive.py")

    assert "FLOG?" in control
    assert "FLOG DUMP" in control
    assert "FLOG CANCEL" in control
    assert "FLOG TESTFILL" in control
    assert "APP_FlightLog_StartDump()" in control
    assert "APP_FlightLog_CancelDump()" in control
    assert "APP_FlightLog_TestFill(sectors)" in control
    assert '(strcmp(line, "FLOG?") == 0)' in aiwb2
    assert '(aiwb2_starts_with(line, "FLOG ") != 0U)' in aiwb2
    assert "APP_FlightLog_IsExportActive() != 0U" in freertos
    assert "vofaStreamActive = 0U;" in read("App/Src/app_flight_log.c")
    assert "Sensor_Data:0\\r\\n" in script
    assert "FLOG DUMP\\r\\n" in script
    assert "DEFAULT_BAUD = 57600" in script
    assert "MOTOR_REASON_NAMES" in script


def test_export_begin_and_end_lines_are_not_dropped_on_full_uart_queue() -> None:
    source = read("App/Src/app_flight_log.c")

    assert "static uint8_t flight_log_export_finish" in source
    assert 'flight_log_queue_printf("FLOG END reason=%s sent=%lu total=%lu\\r\\n"' in source
    assert "return 0U;" in source[source.index("static uint8_t flight_log_export_finish"):source.index("static APP_FlightLogCommandStatus flight_log_start_export_from_background")]
    assert "flight_log_status.export_active = 1U;" in source
    assert source.index("FLOG BEGIN version=%u") < source.index("flight_log_status.export_active = 1U;")
    assert "flight_log_export_pending = 0U;" in source[source.index("flight_log_status.export_active = 1U;"):]
