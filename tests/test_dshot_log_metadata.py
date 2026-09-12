"""Tagged sector metadata keeps equivalent PWM commands distinct from real PWM."""
import struct
from pathlib import Path
import pytest
from tools import flight_log_receive as flog
from test_flight_log_receive import make_sector_header

ROOT = Path(__file__).resolve().parents[1]


def with_tag(header, tag):
    data = bytearray(header)
    data[252:256] = tag
    data[52:56] = bytes(4)
    struct.pack_into("<I", data, 52, flog.crc32(bytes(data)))
    return bytes(data)


@pytest.mark.parametrize("tag,protocol,unit", [
    (b"\xd5\x01\x02\0", "DSHOT300", "pwm_equivalent_us"),
    (b"\xd5\x01\x01\0", "PWM", "us"),
    (bytes(4), "legacy_unspecified", "us"),
    (b"\xff" * 4, "legacy_unspecified", "us"),
])
def test_tagged_v10_and_legacy_v10(tag, protocol, unit):
    original = make_sector_header()
    parsed = flog.parse_sector_header(with_tag(original, tag), 0)
    assert parsed["esc_protocol"] == protocol
    assert parsed["motor_command_unit"] == unit
    assert parsed["params"] == flog.parse_sector_header(original, 0)["params"]


def test_older_layout_never_infers_dshot_from_reserved_bytes():
    original = make_sector_header(version=9, record_size=flog.V9_RECORD_STRUCT.size,
                                  params_struct=flog.V9_PARAMS_STRUCT, param_names=flog.V9_PARAM_NAMES)
    assert flog.parse_sector_header(with_tag(original, b"\xd5\x01\x02\0"), 0)["esc_protocol"] == "legacy_unspecified"


def test_firmware_marker_location_and_crc_order_are_pinned():
    source = (ROOT / "App/Src/app_flight_log.c").read_text(encoding="utf-8")
    assert "offsetof(APP_FlightLogSectorHeader, reserved) == 252U" in source
    fill = source.split("static void flight_log_fill_sector_header(")[1].split("\n}")[0]
    assert "header->reserved[0] = 0xD5U" in fill
    assert "header->reserved[1] = 1U" in fill
    assert "? 2U : 1U" in fill
    assert fill.index("header->reserved[0]") < fill.index("header->header_crc32")


def test_protocol_metadata_follows_records_into_csv(tmp_path):
    header = with_tag(make_sector_header(), b"\xd5\x01\x02\0")
    record = bytearray(flog.RECORD_SIZE)
    struct.pack_into("<IHH", record, 0, flog.RECORD_MAGIC, 10, flog.RECORD_SIZE)
    struct.pack_into("<I", record, len(record) - 4, flog.crc32(bytes(record)))
    image = header + bytes(record)
    image += b"\xff" * (flog.SECTOR_SIZE - len(image))
    sectors, records, errors = flog.parse_flash_image(image)
    assert not errors and len(records) == 1
    assert records[0]["esc_protocol"] == sectors[0]["esc_protocol"] == "DSHOT300"
    assert records[0]["motor_command_unit"] == "pwm_equivalent_us"
    output = tmp_path / "dshot.csv"
    flog.write_csv(output, records)
    assert "esc_protocol,motor_command_unit" in output.read_text(encoding="utf-8").splitlines()[0]
