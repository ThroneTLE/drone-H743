"""V11/V12 C/Python log contract（v12 = v11 + 导航/电池尾块）。"""
from pathlib import Path
import re
import shutil
import struct
import subprocess

import pytest
from tools import flight_log_receive as flog
from test_flight_log_receive import make_sector_header, make_v3_record
from test_dshot_log_metadata import with_tag

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module", params=[0, 1], ids=["PWM", "DSHOT300"])
def encoded(tmp_path_factory, request):
    d = tmp_path_factory.mktemp("dshot-log")
    (d / "main.h").write_text("#include <stdint.h>\ntypedef struct SPI_HandleTypeDef SPI_HandleTypeDef;\ntypedef struct GPIO_TypeDef GPIO_TypeDef;\n")
    source = (ROOT / "App/Src/app_flight_log.c").read_text(encoding="utf-8")
    record = source.split("} APP_FlightLogSectorHeader;", 1)[1].split("} APP_FlightLogRecord;", 1)[0] + "} APP_FlightLogRecord;"
    def function(signature):
        return signature + source.split(signature, 1)[1].split("\n}\n", 1)[0] + "\n}\n"
    defines = "\n".join(line for line in source.splitlines() if re.match(
        r"#define APP_FLIGHT_LOG_(VERSION\s|RECORD_MAGIC\s|QUEUE_CAPACITY\s)", line))
    harness = r'''
#include "app_flight_log.h"
#include "app_esc_log.h"
#include "bsp_dshot.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static APP_FlightLogStatus flight_log_status;
static uint32_t flight_log_record_sequence, flight_log_queue_count, flight_log_queue_head;
static uint8_t flight_log_frame_orientation_code,flight_log_frame_contract_version,flight_log_frame_provenance_ready;
static uint32_t flight_log_frame_calibration_generation,flight_log_export_pending,flight_log_flush_requested,flight_log_new_run_pending;
static BSP_DShotSnapshot diag;
static unsigned reads;
void BSP_DShot_GetSnapshot(BSP_DShotSnapshot *s){*s=diag;reads++;}
'''
    main = r'''
int main(void){
    APP_FlightLogSnapshot s={0};
    flight_log_status.initialized=1;
    s.timestamp_us=1000;s.motor_upper_us=1500;s.motor_lower_us=1600;s.rc_armed=1;
    s.nav.ekf_accel_bias_m_s2[0]=0.25f;s.nav.ekf_accel_bias_m_s2[1]=-0.5f;
    s.nav.ekf_innovation_m_s[0]=0.125f;s.nav.ekf_innovation_m_s[1]=-0.0625f;s.nav.ekf_nis=1.5f;
    s.nav.ekf_flow_update_count=1000;s.nav.ekf_flow_reject_count=7;
    s.nav.flow_filtered_x=-123;s.nav.flow_filtered_y=45;s.nav.battery_mv=11800;
    s.nav.nav_flags=APP_FLIGHT_LOG_NAV_FLAG_VELOCITY_VALID|APP_FLIGHT_LOG_NAV_FLAG_BATTERY_VALID|APP_FLIGHT_LOG_NAV_FLAG_POSITION_VALID;
    diag.code[0]=313;diag.code[1]=2047;diag.enabled_mask=3;diag.busy=1;
    diag.submitted=20;diag.completed=18;diag.busy_rejected=7;diag.cancelled=1;diag.timer_clock_hz=120000000;
    APP_FlightLog_Observe(&s,0);
    if(reads || flight_log_queue_count) return 1;
    APP_FlightLog_Observe(&s,1);
    if(flight_log_queue_count!=1 || !flight_log_status.recording) return 2;
    diag.fault=1;diag.errors=1;diag.enabled_mask=0;diag.busy=0;
    s.timestamp_us=2000; APP_FlightLog_Observe(&s,1);
    APP_FlightLog_Observe(NULL,0);
    if(flight_log_queue_count!=2 || flight_log_status.recording || !flight_log_flush_requested) return 3;
#if BSP_ESC_PROTOCOL == 1
    if(reads!=2 || flight_log_queue[0].dshot.fault || !flight_log_queue[1].dshot.fault) return 4;
#else
    if(reads || flight_log_queue[0].dshot.present) return 5;
#endif
    FILE *f=fopen("records.bin","wb"); if(!f)return 6;
    fwrite(flight_log_queue,sizeof(APP_FlightLogRecord),2,f); fclose(f);return 0;
}
'''
    unit = harness + defines + record + "\nstatic APP_FlightLogRecord flight_log_queue[APP_FLIGHT_LOG_QUEUE_CAPACITY];\n"
    unit += '_Static_assert(sizeof(APP_FlightLogRecord)==844,"record ABI");\n_Static_assert(offsetof(APP_FlightLogRecord,dshot)==772,"old offsets");\n_Static_assert(offsetof(APP_FlightLogRecord,nav)==804,"nav tail offset");\n'
    unit += function("static uint32_t flight_log_crc32(")
    unit += function("static void flight_log_record_from_snapshot(")
    unit += function("void APP_FlightLog_Observe(") + main
    (d / "test.c").write_text(unit, encoding="utf-8")
    cmd = [shutil.which("gcc"), "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", f"-DBSP_ESC_PROTOCOL={request.param}"]
    for inc in (d, ROOT / "App/Inc", ROOT / "Driver/Inc", ROOT / "BSP/Inc"):
        cmd += ["-I", str(inc)]
    cmd += [str(d / "test.c"), str(ROOT / "App/Src/app_esc_log.c"), "-o", str(d / "test.exe")]
    r = subprocess.run(cmd, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    subprocess.run([str(d / "test.exe")], cwd=d, check=True)
    return request.param, (d / "records.bin").read_bytes()


def test_c_to_csv(encoded, tmp_path):
    protocol, raw = encoded
    assert flog.RECORD_SIZE == 844 and flog.V11_RECORD_SIZE == 808 and flog.V10_RECORD_SIZE == 776
    tag = b"\xd5\x01" + bytes([2 if protocol else 1, 0])
    header = with_tag(make_sector_header(), tag)
    assert flog.parse_sector_header(header, 0)["version"] == 12
    sectors, rows, errors = flog.parse_flash_image((header + raw).ljust(4096, b"\xff"))
    assert not errors and len(rows) == 2 and len(sectors) == 1
    first, fault = rows
    assert first["motor_upper_us"] == 1500 and first["motor_lower_us"] == 1600
    assert first["timestamp_us"] == 1000 and fault["timestamp_us"] == 2000
    if protocol:
        assert first["dshot_code_ch1"] == 313 and first["dshot_code_ch2"] == 2047
        assert first["dshot_enabled_mask"] == 3 and first["dshot_busy"] == 1
        assert first["dshot_submitted"] == 20 and first["dshot_completed"] == 18
        assert first["dshot_busy_rejected"] == 7 and first["dshot_timer_clock_hz"] == 120000000
        assert first["dshot_fault"] == 0 and fault["dshot_fault"] == 1 and fault["dshot_errors"] == 1
    else:
        assert first["dshot_present"] == 0
        assert all(first[k] is None for k in flog.DSHOT_NAMES[1:])
    assert first["nav_ekf_accel_bias_x_m_s2"] == 0.25 and first["nav_ekf_accel_bias_y_m_s2"] == -0.5
    assert first["nav_ekf_innovation_x_m_s"] == 0.125 and first["nav_ekf_nis"] == 1.5
    assert first["nav_ekf_flow_update_count"] == 1000 and first["nav_ekf_flow_reject_count"] == 7
    assert first["nav_flow_filtered_x"] == -123 and first["nav_flow_filtered_y"] == 45
    assert first["nav_battery_mv"] == 11800
    assert (first["nav_velocity_valid"], first["nav_battery_valid"], first["nav_position_valid"]) == (1, 1, 1)
    assert (first["nav_battery_low"], first["nav_accel_valid"], first["nav_attitude_debug"]) == (0, 0, 0)
    output = tmp_path / "log.csv"
    flog.write_csv(output, rows)
    assert "dshot_completed" in output.read_text(encoding="utf-8").splitlines()[0]
    assert "nav_battery_mv" in output.read_text(encoding="utf-8").splitlines()[0]


def test_v11_compatibility_and_mixed_csv(encoded, tmp_path):
    """v11（808 字节，无尾块）照旧可读；新旧记录混在一个 CSV 里列集合一致。"""
    _, raw = encoded
    v11 = bytearray(raw[:804] + bytes(4))
    struct.pack_into("<HH", v11, 4, 11, 808)
    struct.pack_into("<I", v11, 804, flog.crc32(v11))
    row = flog.parse_record(bytes(v11))
    assert row is not None and row["motor_upper_us"] == 1500
    assert all(row[k] is None for k in flog.NAV_COLUMNS)
    new = flog.parse_record(raw[:844])
    assert list(new) == list(row)
    flog.write_csv(tmp_path / "mixed.csv", [row, new])


def test_v10_compatibility(encoded):
    _, raw = encoded
    old = bytearray(raw[:772] + bytes(4))
    struct.pack_into("<HH", old, 4, 10, 776)
    struct.pack_into("<I", old, 772, flog.crc32(old))
    row = flog.parse_record(bytes(old))
    assert row is not None and row["motor_upper_us"] == 1500
    assert all(row[k] is None for k in flog.DSHOT_NAMES)
    assert all(row[k] is None for k in flog.NAV_COLUMNS)
    assert all(flog.parse_record(make_v3_record())[k] is None for k in flog.DSHOT_NAMES)


def test_corrupt_records(encoded):
    _, raw = encoded
    record = raw[:844]
    for cut in range(1, len(record)):
        assert flog.parse_record(record[:cut]) is None
    damaged = bytearray(record); damaged[779] ^= 1
    assert flog.parse_record(bytes(damaged)) is None
    struct.pack_into("<H", damaged, 4, 10)
    damaged[-4:] = bytes(4); struct.pack_into("<I", damaged, 840, flog.crc32(damaged))
    assert flog.parse_record(bytes(damaged)) is None
