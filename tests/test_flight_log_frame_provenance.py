"""R-F5b flight-log frame provenance and version migration."""

# Seam 5 could only pin the mismatch, not fix it, because the log carried no
# way to tell which convention a file was recorded under.  R-F5b adds that
# provenance and lets the replay pick per file.
#
# Placement follows spec section 6: the sector header is a fixed 256 bytes and
# evolution may only consume its reserved area, so the new fields take 12 bytes
# out of reserved[84] -> reserved[72] and every pre-existing field keeps its
# offset.  Provenance is a per-session property, so putting it in the sector
# header costs 12 bytes per sector instead of per record at 250 Hz.
#
# Migration rules being locked here (spec section 6):
#   * the reader dispatches on the header version,
#   * a v7 sector still validates against its own CRC -- old versions stay
#     readable forever,
#   * missing fields read as "no provenance" defaults rather than garbage.
#
# firmware_crc32 is deliberately NOT sourced in the stabilizer snapshot: the
# first APP_FirmwareIdentity_Get() CRCs the whole image, and the 250 Hz control
# loop may not block on that (spec section 5).  It is sampled where the sector
# header is filled, which is reachable only from APP_FlightLog_BackgroundStep().

from __future__ import annotations

from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
LOG_SOURCE = ROOT / "App" / "Src" / "app_flight_log.c"
LOG_HEADER = ROOT / "App" / "Inc" / "app_flight_log.h"
STABILIZER = ROOT / "App" / "Src" / "app_stabilizer.c"
RECEIVE = ROOT / "tools" / "flight_log_receive.py"
REPLAY = ROOT / "tools" / "flight_log_rerun_replay.py"

SECTOR_MAGIC = 0x31534C46
SECTOR_HEADER_SIZE = 256
PARAMS_SIZE = 92
PREFIX_SIZE = 56


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def slice_between(source: str, start: str, end: str) -> str:
    begin = source.index(start)
    stop = source.index(end, begin) + len(end)
    return source[begin:stop]


def test_sector_header_declares_provenance_in_reserved_area() -> None:
    source = read(LOG_SOURCE)
    assert "#define APP_FLIGHT_LOG_VERSION            9U" in source
    # The previous version must remain a named, readable constant.
    assert "APP_FLIGHT_LOG_VERSION_V7" in source
    header = slice_between(
        source, "typedef struct __attribute__((packed)) {", "} APP_FlightLogSectorHeader;"
    )
    for field in (
        "uint8_t  frame_provenance_valid;",
        "uint8_t  frame_orientation_code;",
        "uint8_t  frame_contract_version;",
        "uint32_t firmware_crc32;",
        "uint32_t calibration_generation;",
    ):
        assert field in header, field
    # 12 bytes taken out of the reserved area; the header stays 256 bytes.
    assert "uint8_t reserved[96];" in header
    assert "uint8_t reserved[108];" not in header


def test_stabilizer_fills_snapshot_provenance_from_existing_sources() -> None:
    source = read(STABILIZER)
    assert "flog_snapshot.frame_orientation_code = ctx->imu_frame_orientation_code;" in source
    assert "flog_snapshot.frame_contract_version = DRV_FRAME_CONTRACT_VERSION;" in source
    assert "flog_snapshot.calibration_generation = ctx->imu_calibration_generation;" in source
    # The image CRC must not be taken here; it would block the control loop.
    assert "APP_FirmwareIdentity" not in source


def test_firmware_crc_is_sampled_off_the_control_loop() -> None:
    source = read(LOG_SOURCE)
    fill = slice_between(
        source, "static void flight_log_fill_sector_header(", "\n}\n"
    )
    assert "APP_FirmwareIdentity_GetCrc32()" in fill


HARNESS = r"""
#include "drv_coax_ctrl.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { if (!(condition)) return (code); } while (0)

/* The validator only takes sizeof() of the record, so an opaque block of the
 * asserted size is enough and keeps drv_imu.h (which needs main.h) out. */
typedef struct __attribute__((packed)) { uint8_t raw[528]; } APP_FlightLogRecord;

#define APP_FLASH_SERVICE_SECTOR_SIZE 4096UL

int main(void)
{
    APP_FlightLogSectorHeader header;
    uint32_t crc;

    CHECK(sizeof(APP_FlightLogSectorHeader) == APP_FLIGHT_LOG_SECTOR_HEADER_SIZE, 1);

    /* A current-version sector carries provenance and validates. */
    memset(&header, 0, sizeof(header));
    header.magic = APP_FLIGHT_LOG_SECTOR_MAGIC;
    header.version = APP_FLIGHT_LOG_VERSION;
    header.header_size = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    header.sector_size = APP_FLASH_SERVICE_SECTOR_SIZE;
    header.record_size = sizeof(APP_FlightLogRecord);
    header.region_start = APP_FLIGHT_LOG_REGION_START;
    header.region_end_excl = APP_FLIGHT_LOG_REGION_END_EXCL;
    header.frame_provenance_valid = 1U;
    header.frame_orientation_code = 3U;
    header.frame_contract_version = 1U;
    header.firmware_crc32 = 0xF12AD9F5UL;
    header.calibration_generation = 3UL;
    header.header_crc32 = 0U;
    crc = flight_log_crc32((const uint8_t *)&header, sizeof(header));
    header.header_crc32 = crc;
    CHECK(flight_log_sector_header_valid(&header) == 1U, 10);
    CHECK(header.frame_provenance_valid == 1U, 11);
    CHECK(header.frame_orientation_code == 3U, 12);
    CHECK(header.firmware_crc32 == 0xF12AD9F5UL, 13);
    CHECK(header.calibration_generation == 3UL, 14);

    /*
     * A v7 sector predates the provenance block: its bytes are zero there, it
     * must still validate against its own CRC, and it must read back as "no
     * provenance" rather than as a bogus orientation code.
     */
    memset(&header, 0, sizeof(header));
    header.magic = APP_FLIGHT_LOG_SECTOR_MAGIC;
    header.version = APP_FLIGHT_LOG_VERSION_V7;
    header.header_size = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    header.sector_size = APP_FLASH_SERVICE_SECTOR_SIZE;
    header.record_size = sizeof(APP_FlightLogRecord);
    header.region_start = APP_FLIGHT_LOG_REGION_START;
    header.region_end_excl = APP_FLIGHT_LOG_REGION_END_EXCL;
    header.header_crc32 = 0U;
    crc = flight_log_crc32((const uint8_t *)&header, sizeof(header));
    header.header_crc32 = crc;
    CHECK(flight_log_sector_header_valid(&header) == 1U, 20);
    CHECK(header.frame_provenance_valid == 0U, 21);

    /* A corrupted CRC must still be rejected in both versions. */
    header.header_crc32 = crc ^ 0x1U;
    CHECK(flight_log_sector_header_valid(&header) == 0U, 30);

    /* An unknown version must be rejected, not guessed at. */
    memset(&header, 0, sizeof(header));
    header.magic = APP_FLIGHT_LOG_SECTOR_MAGIC;
    header.version = 6U;
    header.header_size = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    header.sector_size = APP_FLASH_SERVICE_SECTOR_SIZE;
    header.record_size = sizeof(APP_FlightLogRecord);
    header.region_start = APP_FLIGHT_LOG_REGION_START;
    header.region_end_excl = APP_FLIGHT_LOG_REGION_END_EXCL;
    header.header_crc32 = 0U;
    crc = flight_log_crc32((const uint8_t *)&header, sizeof(header));
    header.header_crc32 = crc;
    CHECK(flight_log_sector_header_valid(&header) == 0U, 40);

    puts("ok");
    return 0;
}
"""


def test_sector_header_migration_runs_on_host(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.fail("R-F5b contract requires host gcc or clang")

    source = read(LOG_SOURCE)
    log_header = read(LOG_HEADER)
    defines = "\n".join(
        line
        for line in log_header.splitlines()
        if line.startswith("#define APP_FLIGHT_LOG_REGION")
        or line.startswith("#define APP_FLIGHT_LOG_SECTOR_HEADER_SIZE")
    )
    version_defines = "\n".join(
        line
        for line in source.splitlines()
        if line.startswith("#define APP_FLIGHT_LOG_SECTOR_MAGIC")
        or line.startswith("#define APP_FLIGHT_LOG_VERSION")
    )
    struct_source = slice_between(
        source,
        "typedef struct __attribute__((packed)) {",
        "} APP_FlightLogSectorHeader;",
    )
    crc_source = slice_between(source, "static uint32_t flight_log_crc32(", "\n}\n")
    valid_source = slice_between(
        source, "static uint8_t flight_log_sector_header_valid(", "\n}\n"
    )

    unit = tmp_path / "flight_log_header.c"
    unit.write_text(
        "#include <stdint.h>\n#include <stddef.h>\n"
        + HARNESS.split("int main(void)")[0]
        + defines
        + "\n"
        + version_defines
        + "\n"
        + struct_source
        + "\n"
        + crc_source
        + "\n"
        + valid_source
        + "\n"
        + "int main(void)"
        + HARNESS.split("int main(void)")[1],
        encoding="utf-8",
    )
    executable = tmp_path / "flight_log_header.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(unit),
            "-o",
            str(executable),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    result = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True
    )
    assert result.stdout.strip() == "ok"


def build_sector_header(version: int, *, provenance: tuple | None = None) -> bytes:
    """Synthesise a sector header exactly as the firmware lays it out."""
    prefix = struct.pack(
        "<IHHIIIIIIIIQII",
        SECTOR_MAGIC,
        version,
        SECTOR_HEADER_SIZE,
        4096,
        528,
        7,  # session_id
        1,  # sector_seq
        0,  # sector_index
        250,
        0x00002000,
        0x003FC000,
        1234567890,
        PARAMS_SIZE,
        0,  # header_crc32 placeholder
    )
    body = bytearray(prefix + b"\x00" * PARAMS_SIZE)
    if provenance is not None:
        valid, orientation, contract, fw_crc, cal_gen = provenance
        body += struct.pack("<BBBBII", valid, orientation, contract, 0, fw_crc, cal_gen)
        body += b"\x00" * 96
    else:
        body += b"\x00" * 108
    assert len(body) == SECTOR_HEADER_SIZE
    import zlib

    crc = zlib.crc32(bytes(body[:52]) + b"\x00\x00\x00\x00" + bytes(body[56:]))
    body[52:56] = struct.pack("<I", crc & 0xFFFFFFFF)
    return bytes(body)


def test_receive_tool_reads_provenance_and_still_reads_v7() -> None:
    sys.path.insert(0, str(ROOT / "tools"))
    try:
        import flight_log_receive as receive
    finally:
        sys.path.pop(0)

    v8 = receive.parse_sector_header(
        build_sector_header(8, provenance=(1, 3, 1, 0xF12AD9F5, 3)), 0
    )
    assert v8 is not None
    assert v8["frame_provenance_valid"] == 1
    assert v8["frame_orientation_code"] == 3
    assert v8["frame_contract_version"] == 1
    assert v8["firmware_crc32"] == 0xF12AD9F5
    assert v8["calibration_generation"] == 3

    # Old sectors stay readable and report no provenance.
    v7 = receive.parse_sector_header(build_sector_header(7), 0)
    assert v7 is not None
    assert v7["version"] == 7
    assert v7["frame_provenance_valid"] == 0


def test_replay_selects_convention_per_file() -> None:
    source = read(REPLAY)
    # A named resolver, not a scattered conditional.
    assert re.search(r"def resolve_attitude_frame\(", source)
    # Both outcomes must exist and be explicit.
    assert "canonical_flu" in source
    assert "legacy_frd" in source
    # No provenance must keep the legacy geometry and say so on screen.
    assert "未标注坐标系" in source or "no frame provenance" in source
