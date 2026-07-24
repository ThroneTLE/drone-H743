import io
import json
import struct

import pytest

from tools import flight_log_receive as flog


class FakePort:
    def __init__(self, data: bytes) -> None:
        self._rx = io.BytesIO(data)
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def read(self, length: int) -> bytes:
        return self._rx.read(length)

    def readline(self) -> bytes:
        return self._rx.readline()


def test_export_block_parser_validates_crc() -> None:
    payload = b"abc123"
    header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        7,
        42,
        len(payload),
        flog.EXPORT_BLOCK_LAST,
        flog.crc32(payload),
    )
    block = flog.read_export_block(io.BytesIO(header + payload))

    assert block.seq == 7
    assert block.offset == 42
    assert block.payload == payload
    assert block.flags == flog.EXPORT_BLOCK_LAST


def test_export_block_parser_rejects_crc_mismatch() -> None:
    payload = b"abc123"
    header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        0,
        0,
        len(payload),
        0,
        0x12345678,
    )

    with pytest.raises(flog.FlightLogError, match="crc mismatch"):
        flog.read_export_block(io.BytesIO(header + payload))


def test_begin_line_requires_flight_log_magic() -> None:
    begin = flog.parse_begin_line(
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1"
    )

    assert begin.total == 4096


def test_end_line_requires_done_and_matching_counts() -> None:
    end = flog.read_until_end(
        FakePort(b"FLOG END reason=done sent=4096 total=4096\r\n"),
        received=4096,
        expected_total=4096,
    )

    assert end.reason == "done"
    assert end.sent == 4096
    assert end.total == 4096

    with pytest.raises(flog.FlightLogError, match="reason=error"):
        flog.read_until_end(
            FakePort(b"FLOG END reason=error sent=128 total=4096\r\n"),
            received=128,
            expected_total=4096,
        )

    with pytest.raises(flog.FlightLogError, match="export end mismatch"):
        flog.read_until_end(
            FakePort(b"FLOG END reason=done sent=128 total=4096\r\n"),
            received=4096,
            expected_total=4096,
        )


def test_parse_record_and_csv_fields(tmp_path) -> None:
    packed = make_record()

    row = flog.parse_record(packed)
    assert row is not None
    assert row["sequence"] == 3
    assert row["raw_gyro_z"] == 7
    assert row["rc_ch8_us"] == 1007
    assert row["motor_output_reason"] == 5
    assert row["motor_output_reason_name"] == "rc_loss_disable"
    assert row["arm_switch_high"] == 1
    assert row["vel_pid_d_m_s2_1"] == 17.0
    assert row["ctrl_pos_p_m_s2_0"] == 19.0
    assert row["ctrl_tilt_angle_p_rad_0"] == 33.0
    assert row["ctrl_motor_cmd_us_1"] == 46.0

    csv_path = tmp_path / "out.csv"
    flog.write_csv(csv_path, [row])
    text = csv_path.read_text(encoding="utf-8")
    assert "timestamp_us" in text
    assert "vel_pid_out_m_s2_0" in text


def make_record() -> bytes:
    values = []
    values.extend([flog.RECORD_MAGIC, 1, flog.RECORD_SIZE, 3, 0, 1000, 12, 99])
    values.extend([1, 2, 3, 4, 5, 6, 7])
    values.extend([25.0, 0.1, 0.2, 0.3, 10.0, 11.0, 12.0])
    values.extend([1.0, 2.0, 3.0])
    values.extend([1000 + i for i in range(8)])
    values.extend([1200, 1500, 1500, 1300, 1310])
    values.extend([1, 1, 1, 1, 5, 1, 1, 1, 1, 0, 0, 0])
    values.extend([float(i) for i in range(48)])
    values.append(0)
    packed_without_crc = flog.RECORD_STRUCT.pack(*values)
    crc = flog.crc32(packed_without_crc[:-4] + b"\x00\x00\x00\x00")
    values[-1] = crc
    return flog.RECORD_STRUCT.pack(*values)


def make_sector_header() -> bytes:
    params = [float(i) for i in range(len(flog.PARAM_NAMES))]
    prefix = flog.SECTOR_HEADER_PREFIX.pack(
        flog.SECTOR_MAGIC,
        1,
        flog.SECTOR_HEADER_SIZE,
        flog.SECTOR_SIZE,
        flog.RECORD_SIZE,
        123,
        5,
        0,
        250,
        0x2000,
        0x3FC000,
        100,
        flog.PARAMS_STRUCT.size,
        0,
    )
    reserved_len = flog.SECTOR_HEADER_SIZE - len(prefix) - flog.PARAMS_STRUCT.size
    reserved = b"\x00" * reserved_len
    header = bytearray(prefix + flog.PARAMS_STRUCT.pack(*params) + reserved)
    assert len(header) == flog.SECTOR_HEADER_SIZE
    crc = flog.crc32(bytes(header))
    struct.pack_into("<I", header, 52, crc)
    return bytes(header)


def make_legacy_sector_header() -> bytes:
    params = [float(i) for i in range(len(flog.LEGACY_PARAM_NAMES))]
    prefix = flog.SECTOR_HEADER_PREFIX.pack(
        flog.SECTOR_MAGIC,
        1,
        flog.SECTOR_HEADER_SIZE,
        flog.SECTOR_SIZE,
        flog.RECORD_SIZE,
        122,
        4,
        0,
        250,
        0x2000,
        0x3FC000,
        100,
        flog.LEGACY_PARAMS_STRUCT.size,
        0,
    )
    reserved_len = (
        flog.SECTOR_HEADER_SIZE - len(prefix) - flog.LEGACY_PARAMS_STRUCT.size
    )
    header = bytearray(
        prefix
        + flog.LEGACY_PARAMS_STRUCT.pack(*params)
        + (b"\x00" * reserved_len)
    )
    crc = flog.crc32(bytes(header))
    struct.pack_into("<I", header, 52, crc)
    return bytes(header)


def make_export_block(seq: int, offset: int, payload: bytes, flags: int = 0) -> bytes:
    header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        seq,
        offset,
        len(payload),
        flags,
        flog.crc32(payload),
    )
    return header + payload


def make_export_blocks(payload: bytes) -> bytes:
    blocks = []
    seq = 0
    for offset in range(0, len(payload), flog.EXPORT_PAYLOAD_MAX):
        chunk = payload[offset : offset + flog.EXPORT_PAYLOAD_MAX]
        flags = flog.EXPORT_BLOCK_LAST if (offset + len(chunk)) >= len(payload) else 0
        blocks.append(make_export_block(seq, offset, chunk, flags))
        seq += 1
    return b"".join(blocks)


def make_export_hex_blocks(payload: bytes) -> bytes:
    blocks = []
    seq = 0
    for offset in range(0, len(payload), flog.EXPORT_PAYLOAD_MAX):
        chunk = payload[offset : offset + flog.EXPORT_PAYLOAD_MAX]
        flags = flog.EXPORT_BLOCK_LAST if (offset + len(chunk)) >= len(payload) else 0
        blocks.append(
            (
                f"FLOG BLK seq={seq} offset={offset} len={len(chunk)} "
                f"flags={flags} crc={flog.crc32(chunk):08X} "
                f"data={flog.xor_whiten(chunk, offset).hex().upper()}\r\n"
            ).encode("ascii")
        )
        seq += 1
    return b"".join(blocks)


def test_sector_header_and_flash_image_parse() -> None:
    image = make_sector_header() + (b"\xFF" * (flog.SECTOR_SIZE - flog.SECTOR_HEADER_SIZE))

    sectors, records, errors = flog.parse_flash_image(image)

    assert errors == []
    assert records == []
    assert sectors[0]["session_id"] == 123
    assert sectors[0]["params"]["vel_loop_x_kp"] == float(flog.PARAM_NAMES.index("vel_loop_x_kp"))
    assert sectors[0]["params"]["yaw_torque_lower_m_per_n"] == float(
        flog.PARAM_NAMES.index("yaw_torque_lower_m_per_n")
    )


def test_legacy_single_tilt_lever_header_still_parses_without_field_shift() -> None:
    image = make_legacy_sector_header() + (
        b"\xFF" * (flog.SECTOR_SIZE - flog.SECTOR_HEADER_SIZE)
    )

    sectors, records, errors = flog.parse_flash_image(image)

    assert errors == []
    assert records == []
    assert sectors[0]["session_id"] == 122
    assert sectors[0]["params"]["tilt_lever_arm_m"] == float(
        flog.LEGACY_PARAM_NAMES.index("tilt_lever_arm_m")
    )
    assert "pitch_tilt_lever_arm_m" not in sectors[0]["params"]


def test_receive_dump_writes_bin_csv_and_meta(tmp_path) -> None:
    image = bytearray(b"\xFF" * flog.SECTOR_SIZE)
    image[:flog.SECTOR_HEADER_SIZE] = make_sector_header()
    record = make_record()
    image[flog.SECTOR_HEADER_SIZE : flog.SECTOR_HEADER_SIZE + len(record)] = record
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1 "
        f"sector_size=4096 header_size=256 record_size={flog.RECORD_SIZE} log_rate=250 baud=57600 "
        "session=123\r\n"
    ).encode("ascii")
    payload = bytes(image)
    end = b"FLOG END reason=done sent=4096 total=4096\r\n"
    stream = begin + make_export_blocks(payload) + end
    port = FakePort(stream)

    result = flog.receive_dump(port, tmp_path)

    assert port.writes == [
        b"Sensor_Data:0\r\n",
        b"FLOG CANCEL\r\n",
        b"Sensor_Data:0\r\n",
        b"FLOG DUMP\r\n",
    ]
    assert result.total_bytes == flog.SECTOR_SIZE
    assert result.good_bytes == flog.SECTOR_SIZE
    assert result.missing_bytes == 0
    assert result.complete is True
    assert result.blocks == (flog.SECTOR_SIZE + flog.EXPORT_PAYLOAD_MAX - 1) // flog.EXPORT_PAYLOAD_MAX
    assert result.records == 1
    assert result.sectors == 1
    assert result.bin_path.read_bytes() == payload
    assert "ctrl_total_force_n" in result.csv_path.read_text(encoding="utf-8")
    meta = json.loads(result.meta_path.read_text(encoding="utf-8"))
    assert meta["record_size"] == flog.RECORD_SIZE
    assert meta["baud"] == 57600
    assert meta["record_count"] == 1
    assert meta["complete"] is True
    assert meta["missing_ranges"] == []
    assert meta["end"]["reason"] == "done"


def test_receive_dump_skips_duplicate_begin_text_before_binary_block(tmp_path) -> None:
    image = bytearray(b"\xFF" * flog.SECTOR_SIZE)
    image[:flog.SECTOR_HEADER_SIZE] = make_sector_header()
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1\r\n"
    ).encode("ascii")
    duplicate_begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1\r\n"
    ).encode("ascii")
    end = b"FLOG END reason=done sent=4096 total=4096\r\n"
    port = FakePort(begin + duplicate_begin + make_export_blocks(bytes(image)) + end)

    result = flog.receive_dump(port, tmp_path)

    assert result.complete is True
    assert result.good_bytes == flog.SECTOR_SIZE
    assert any("duplicate FLOG BEGIN" in error for error in result.errors)


def test_receive_dump_accepts_hex_text_blocks(tmp_path) -> None:
    image = bytearray(b"\xFF" * flog.SECTOR_SIZE)
    image[:flog.SECTOR_HEADER_SIZE] = make_sector_header()
    record = make_record()
    image[flog.SECTOR_HEADER_SIZE : flog.SECTOR_HEADER_SIZE + len(record)] = record
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 encoding=hex total=4096 sectors=1\r\n"
    ).encode("ascii")
    end = b"FLOG END reason=done sent=4096 total=4096\r\n"
    port = FakePort(begin + make_export_hex_blocks(bytes(image)) + end)

    result = flog.receive_dump(port, tmp_path)

    assert result.complete is True
    assert result.good_bytes == flog.SECTOR_SIZE
    assert result.records == 1
    assert result.bin_path.read_bytes() == bytes(image)


def test_receive_dump_saves_later_blocks_after_crc_error(tmp_path) -> None:
    first = b"A" * flog.EXPORT_PAYLOAD_MAX
    second = b"B" * flog.EXPORT_PAYLOAD_MAX
    total = len(first) + len(second)
    begin = (
        f"FLOG BEGIN version=1 block_magic=0x31424C46 total={total} sectors=1\r\n"
    ).encode("ascii")
    bad_header = flog.EXPORT_HEADER.pack(
        flog.EXPORT_BLOCK_MAGIC,
        1,
        flog.EXPORT_HEADER.size,
        0,
        0,
        len(first),
        0,
        0x12345678,
    )
    end = f"FLOG END reason=done sent={total} total={total}\r\n".encode("ascii")
    stream = begin + bad_header + first + make_export_block(1, len(first), second, flog.EXPORT_BLOCK_LAST) + end
    port = FakePort(stream)

    result = flog.receive_dump(port, tmp_path)

    image = result.bin_path.read_bytes()
    assert result.complete is False
    assert result.good_bytes == len(second)
    assert result.missing_bytes == len(first)
    assert image[: len(first)] == b"\xFF" * len(first)
    assert image[len(first) :] == second
    assert any("crc mismatch at block 0" in error for error in result.errors)
    meta = json.loads(result.meta_path.read_text(encoding="utf-8"))
    assert meta["missing_ranges"] == [{"offset": 0, "length": len(first)}]


def test_receive_dump_saves_partial_when_end_line_is_missing(tmp_path) -> None:
    payload = b"\xFF" * flog.SECTOR_SIZE
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1\r\n"
    ).encode("ascii")
    stream = begin + make_export_blocks(payload)
    port = FakePort(stream)

    result = flog.receive_dump(port, tmp_path)

    assert result.complete is False
    assert result.good_bytes == flog.SECTOR_SIZE
    assert result.missing_bytes == 0
    assert any("timeout waiting for FLOG END" in error for error in result.errors)


def test_receive_dump_cancel_sends_flog_cancel(tmp_path) -> None:
    begin = (
        "FLOG BEGIN version=1 block_magic=0x31424C46 total=4096 sectors=1\r\n"
    ).encode("ascii")
    port = FakePort(begin)

    with pytest.raises(flog.FlightLogError, match="cancelled"):
        flog.receive_dump(port, tmp_path, should_cancel=lambda: True)

    assert port.writes == [
        b"Sensor_Data:0\r\n",
        b"FLOG CANCEL\r\n",
        b"Sensor_Data:0\r\n",
        b"FLOG DUMP\r\n",
        b"FLOG CANCEL\r\n",
    ]
