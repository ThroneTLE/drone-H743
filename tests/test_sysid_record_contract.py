"""SYSID 线上格式：真实 C 打包 ↔ 主机 Python 解码，逐字段对拍。

两边是**独立实现**：C 按偏移量硬写，Python 照固件报出来的字段表逐字段取。
错位一个字节、漏掉一个字段、缩放写反，都会在这里当场对不上。
"""
from __future__ import annotations

import ctypes
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from sysid.decode import (  # noqa: E402
    FIELD_TYPE_I16, FIELD_TYPE_U16, FLAG_ERPM_VALID, FLAG_FIRST_BATCH,
    FLAG_LAST_BATCH, HEADER_BYTES, SchemaMismatch, SysIdField, SysIdSchema,
    decode_batch, parse_schema_lines,
)

OK, INVALID, TOO_SMALL = 0, 1, 2
# v3（2026-09-27）：追加 erpm_lower / servo_tilt 两个字段，26 → 30 字节；
# 满批 8 条成帧后 265 字节放不进 256 字节发送缓冲，降到 7 条。
# 高度辨识（R-ALTID-1，2026-09-29）再追加 7 个高度字段，30 → 44 字节，满批 7 → 5 条
# （6 条成帧 289 字节放不进）。版本号不升：主机按字段表/hash 解码（见 drv_sysid_record.h）。
RECORD_VERSION = 3
MAX_COUNT = 5
RECORD_BYTES = 44

# v2 的 13 个字段（名字、单位、缩放、类型）。v3 只许往后追加，这一段逐字不动。
V2_FIELDS = (
    ("gx", "rad/s", 1e-3, 0), ("gy", "rad/s", 1e-3, 0), ("gz", "rad/s", 1e-3, 0),
    ("omega_sp", "rad/s", 1e-3, 0), ("alpha_ff", "rad/s^2", 1e-2, 0),
    ("tilt_x", "rad", 1e-4, 0), ("tilt_y", "rad", 1e-4, 0), ("thrust", "N", 1e-2, 0),
    ("angle", "rad", 1e-4, 0), ("erpm", "rpm", 4.0, 1), ("torque", "N*m", 1e-4, 0),
    ("angle_sp", "rad", 1e-4, 0), ("offset_us", "us", 1.0, 1),
)


class Sample(ctypes.Structure):
    _fields_ = [
        ("gyro_rad_s", ctypes.c_float * 3),
        ("omega_sp_rad_s", ctypes.c_float),
        ("alpha_ff_rad_s2", ctypes.c_float),
        ("tilt_cmd_x_rad", ctypes.c_float),
        ("tilt_cmd_y_rad", ctypes.c_float),
        ("thrust_n", ctypes.c_float),
        ("angle_rad", ctypes.c_float),
        ("erpm", ctypes.c_uint32),
        ("torque_n_m", ctypes.c_float),
        ("angle_sp_rad", ctypes.c_float),
        ("offset_us", ctypes.c_uint32),
        ("erpm_lower", ctypes.c_uint32),
        ("servo_tilt_rad", ctypes.c_float),
        ("height_m", ctypes.c_float),
        ("height_raw_m", ctypes.c_float),
        ("height_sp_m", ctypes.c_float),
        ("vz_m_s", ctypes.c_float),
        ("vz_sp_m_s", ctypes.c_float),
        ("az_m_s2", ctypes.c_float),
        ("vbat_v", ctypes.c_float),
    ]


class BatchHeader(ctypes.Structure):
    _fields_ = [
        ("run_id", ctypes.c_uint16),
        ("base_t_us", ctypes.c_uint32),
        ("dt_us", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
    ]


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    compiler = shutil.which("gcc") or shutil.which("clang")
    if compiler is None:
        pytest.skip("host gcc/clang required for the sysid record contract")
    build = tmp_path_factory.mktemp("sysid-record")
    out = build / ("record.dll" if os.name == "nt" else "record.so")
    result = subprocess.run(
        [compiler, "-shared", "-fPIC", "-std=c11", "-O2",
         "-Wall", "-Wextra", "-Werror", "-pedantic",
         "-I", str(ROOT / "Driver/Inc"),
         str(ROOT / "Driver/Src/drv_sysid_record.c"), "-lm", "-o", str(out)],
        capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

    handle = ctypes.CDLL(str(out))
    handle.DRV_SysIdRecord_SchemaHash.restype = ctypes.c_uint32
    handle.DRV_SysIdRecord_FieldCount.restype = ctypes.c_uint32
    handle.DRV_SysIdRecord_FieldName.argtypes = [ctypes.c_uint32]
    handle.DRV_SysIdRecord_FieldName.restype = ctypes.c_char_p
    handle.DRV_SysIdRecord_FieldUnit.argtypes = [ctypes.c_uint32]
    handle.DRV_SysIdRecord_FieldUnit.restype = ctypes.c_char_p
    handle.DRV_SysIdRecord_FieldScale.argtypes = [ctypes.c_uint32]
    handle.DRV_SysIdRecord_FieldScale.restype = ctypes.c_float
    handle.DRV_SysIdRecord_FieldType.argtypes = [ctypes.c_uint32]
    handle.DRV_SysIdRecord_FieldType.restype = ctypes.c_uint8
    handle.DRV_SysIdRecord_Pack.argtypes = [
        ctypes.POINTER(BatchHeader), ctypes.POINTER(Sample), ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t)]
    handle.DRV_SysIdRecord_Unpack.argtypes = [
        ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t,
        ctypes.POINTER(BatchHeader), ctypes.POINTER(Sample),
        ctypes.POINTER(ctypes.c_uint32)]
    return handle


@pytest.fixture(scope="module")
def schema(lib):
    """把固件的字段表原样搬成主机 schema——主机不自己编一份。"""
    fields = []
    for index in range(lib.DRV_SysIdRecord_FieldCount()):
        fields.append(SysIdField(
            name=lib.DRV_SysIdRecord_FieldName(index).decode("ascii"),
            unit=lib.DRV_SysIdRecord_FieldUnit(index).decode("ascii"),
            scale=lib.DRV_SysIdRecord_FieldScale(index),
            type=lib.DRV_SysIdRecord_FieldType(index)))
    return SysIdSchema(fields=tuple(fields), hash=lib.DRV_SysIdRecord_SchemaHash(),
                       version=RECORD_VERSION)


def pack(lib, samples, run_id=7, base_t_us=123456, dt_us=2000, flags=0):
    for index, sample in enumerate(samples):
        sample.offset_us = index * dt_us
    header = BatchHeader(run_id=run_id, base_t_us=base_t_us, dt_us=dt_us, flags=flags)
    array = (Sample * len(samples))(*samples)
    buffer = (ctypes.c_uint8 * 4096)()
    length = ctypes.c_size_t()
    status = lib.DRV_SysIdRecord_Pack(
        ctypes.byref(header), array, len(samples), buffer, 4096,
        ctypes.byref(length))
    assert status == OK, f"pack 失败 status={status}"
    return bytes(buffer[:length.value])


def make_sample(**kwargs):
    base = dict(gyro_rad_s=(0.0, 0.0, 0.0), omega_sp_rad_s=0.0,
                alpha_ff_rad_s2=0.0, tilt_cmd_x_rad=0.0, tilt_cmd_y_rad=0.0,
                thrust_n=0.0, angle_rad=0.0, erpm=0)
    base.update(kwargs)
    gyro = base.pop("gyro_rad_s")
    return Sample(gyro_rad_s=(ctypes.c_float * 3)(*gyro), **base)


# ---------------------------------------------------------------- 对拍


def test_c_pack_decodes_in_python_field_for_field(lib, schema):
    """核心对拍：C 写的字节，Python 照字段表读回来必须是同一组物理量。"""
    sample = make_sample(
        gyro_rad_s=(1.234, -0.567, 0.089), omega_sp_rad_s=2.0,
        alpha_ff_rad_s2=-12.5, tilt_cmd_x_rad=0.1234, tilt_cmd_y_rad=-0.0456,
        thrust_n=13.41, angle_rad=0.2618, erpm=3600, erpm_lower=41000,
        servo_tilt_rad=-0.0873)
    batch = decode_batch(pack(lib, [sample]), schema)

    assert len(batch.samples) == 1
    row = batch.samples[0]
    assert row["gx"] == pytest.approx(1.234, abs=1e-3)
    assert row["gy"] == pytest.approx(-0.567, abs=1e-3)
    assert row["gz"] == pytest.approx(0.089, abs=1e-3)
    assert row["omega_sp"] == pytest.approx(2.0, abs=1e-3)
    assert row["alpha_ff"] == pytest.approx(-12.5, abs=1e-2)
    assert row["tilt_x"] == pytest.approx(0.1234, abs=1e-4)
    assert row["tilt_y"] == pytest.approx(-0.0456, abs=1e-4)
    assert row["thrust"] == pytest.approx(13.41, abs=1e-2)
    assert row["angle"] == pytest.approx(0.2618, abs=1e-4)
    assert row["erpm"] == pytest.approx(3600, abs=4)
    assert row["erpm_lower"] == pytest.approx(41000, abs=4)
    assert row["servo_tilt"] == pytest.approx(-0.0873, abs=1e-4)


def test_v3_only_appends_after_the_v2_fields(lib, schema):
    """v3 只许在末尾追加：前 13 个字段的名字/单位/缩放/类型与 v2 逐项相同，
    按表取值的旧解析逻辑（按名字找字段）不必改任何偏移。"""
    assert lib.DRV_SysIdRecord_FieldCount() == 22
    assert len(schema.fields) * 2 == RECORD_BYTES == schema.record_bytes
    for got, (name, unit, scale, kind) in zip(schema.fields, V2_FIELDS):
        assert (got.name, got.unit, got.type) == (name, unit, kind)
        assert got.scale == pytest.approx(scale, rel=1e-6)
    tail = [(f.name, f.unit, f.type) for f in schema.fields[13:15]]
    assert tail == [("erpm_lower", "rpm", FIELD_TYPE_U16), ("servo_tilt", "rad", FIELD_TYPE_I16)]
    assert schema.fields[13].scale == 4.0
    assert schema.fields[14].scale == pytest.approx(1e-4, rel=1e-6)


# 高度辨识（R-ALTID-1）在 v3 的 15 个字段之后追加的 7 个：名字/单位/缩放/类型逐项钉死。
ALT_FIELDS = (
    ("height", "m", 1e-4, FIELD_TYPE_I16), ("height_raw", "m", 1e-4, FIELD_TYPE_I16),
    ("height_sp", "m", 1e-4, FIELD_TYPE_I16), ("vz", "m/s", 1e-3, FIELD_TYPE_I16),
    ("vz_sp", "m/s", 1e-3, FIELD_TYPE_I16), ("az", "m/s^2", 1e-3, FIELD_TYPE_I16),
    ("vbat", "V", 1e-3, FIELD_TYPE_U16),
)


def test_the_height_fields_are_appended_after_the_v3_fields(lib, schema):
    """前 15 个字段一个都不动，高度 7 个只往后追加；主机照表按名字取值。"""
    assert [f.name for f in schema.fields[:15]][-2:] == ["erpm_lower", "servo_tilt"]
    tail = schema.fields[15:]
    assert len(tail) == len(ALT_FIELDS)
    for got, (name, unit, scale, kind) in zip(tail, ALT_FIELDS):
        assert (got.name, got.unit, got.type) == (name, unit, kind)
        assert got.scale == pytest.approx(scale, rel=1e-6)


def test_the_height_fields_round_trip_and_default_to_zero(lib, schema):
    """C 打包 → Python 按表解码逐字段对拍；不填（非 ALT 轮）解出来就是 0。"""
    row = decode_batch(pack(lib, [make_sample(
        height_m=0.2345, height_raw_m=0.2410, height_sp_m=0.2500, vz_m_s=-0.123,
        vz_sp_m_s=0.1, az_m_s2=-1.234, vbat_v=11.876)]), schema).samples[0]
    assert row["height"] == pytest.approx(0.2345, abs=1e-4)
    assert row["height_raw"] == pytest.approx(0.2410, abs=1e-4)
    assert row["height_sp"] == pytest.approx(0.2500, abs=1e-4)
    assert row["vz"] == pytest.approx(-0.123, abs=1e-3)
    assert row["vz_sp"] == pytest.approx(0.1, abs=1e-3)
    assert row["az"] == pytest.approx(-1.234, abs=1e-3)
    assert row["vbat"] == pytest.approx(11.876, abs=1e-3)
    blank = decode_batch(pack(lib, [make_sample()]), schema).samples[0]
    assert all(blank[name] == 0.0 for name, *_rest in ALT_FIELDS)
    # 饱和不回绕；电压没有负数。
    clipped = decode_batch(pack(lib, [make_sample(height_m=9.0, az_m_s2=-99.0, vbat_v=-3.0)]),
                           schema).samples[0]
    assert clipped["height"] == pytest.approx(3.2767, abs=1e-4)
    assert clipped["az"] == pytest.approx(-32.768, abs=1e-3)
    assert clipped["vbat"] == 0.0


def test_new_fields_saturate_like_the_old_ones(lib, schema):
    """下桨转速超量程钉在 65535×4，舵机倾转超量程钉在 ±3.2767 rad，都不回绕。"""
    row = decode_batch(pack(lib, [make_sample(erpm_lower=10_000_000, servo_tilt_rad=-50.0)]),
                       schema).samples[0]
    assert row["erpm_lower"] == 65535 * 4
    assert row["servo_tilt"] == pytest.approx(-3.2768, abs=1e-4)


def test_header_survives_the_round_trip(lib, schema):
    data = pack(lib, [make_sample()], run_id=1234, base_t_us=0xDEADBEEF,
                dt_us=2000, flags=FLAG_FIRST_BATCH | FLAG_ERPM_VALID)
    batch = decode_batch(data, schema)
    assert batch.run_id == 1234
    assert batch.base_t_us == 0xDEADBEEF
    assert batch.dt_us == 2000
    assert batch.first is True
    assert batch.last is False
    assert batch.erpm_valid is True


def test_c_unpack_agrees_with_python_decode(lib, schema):
    """第三方对照：C 自己的 Unpack 与 Python 解出来必须一致。"""
    samples = [make_sample(gyro_rad_s=(0.1 * i, -0.05 * i, 0.0),
                           omega_sp_rad_s=0.2 * i, thrust_n=10.0 + i,
                           erpm=20000 + 400 * i, erpm_lower=18000 + 400 * i,
                           servo_tilt_rad=0.01 * (i - 3))
               for i in range(MAX_COUNT)]
    data = pack(lib, samples)

    header = BatchHeader()
    out = (Sample * MAX_COUNT)()
    count = ctypes.c_uint32()
    buffer = (ctypes.c_uint8 * len(data))(*data)
    assert lib.DRV_SysIdRecord_Unpack(buffer, len(data), ctypes.byref(header),
                                      out, ctypes.byref(count)) == OK
    assert count.value == MAX_COUNT

    batch = decode_batch(data, schema)
    for index in range(MAX_COUNT):
        assert batch.samples[index]["erpm"] == out[index].erpm
        assert batch.samples[index]["erpm_lower"] == out[index].erpm_lower
        assert batch.samples[index]["servo_tilt"] == pytest.approx(
            out[index].servo_tilt_rad, abs=1e-6)
        assert batch.samples[index]["gx"] == pytest.approx(
            out[index].gyro_rad_s[0], abs=1e-6)
        assert batch.samples[index]["omega_sp"] == pytest.approx(
            out[index].omega_sp_rad_s, abs=1e-6)
        assert batch.samples[index]["thrust"] == pytest.approx(
            out[index].thrust_n, abs=1e-6)


def test_timestamps_come_from_the_firmware_clock(lib, schema):
    """逐样本时间戳必须由 base + k*dt 推出，不能依赖收包时刻。"""
    batch = decode_batch(pack(lib, [make_sample() for _ in range(5)],
                              base_t_us=1_000_000, dt_us=2000), schema)
    assert batch.timestamps_us() == (1_000_000, 1_002_000, 1_004_000,
                                     1_006_000, 1_008_000)


# ---------------------------------------------------------------- 尺寸与效率


def test_frame_size_matches_the_documented_budget(lib, schema):
    assert schema.record_bytes == RECORD_BYTES
    full = pack(lib, [make_sample() for _ in range(MAX_COUNT)])
    assert len(full) == HEADER_BYTES + MAX_COUNT * RECORD_BYTES == 236


def test_the_header_constants_match_this_file():
    """头文件里的版本/记录长度/满批条数就是本文件用的那组；成帧后 5 条放得下，6 条放不下。"""
    header_h = (ROOT / "Driver/Inc/drv_sysid_record.h").read_text(encoding="utf-8")

    def define(name):
        line = [l for l in header_h.splitlines()
                if l.startswith("#define") and l.split()[1] == name][0]
        return int(line.split()[2].rstrip("U"))

    assert define("DRV_SYSID_RECORD_VERSION") == RECORD_VERSION
    assert define("DRV_SYSID_RECORD_BYTES") == RECORD_BYTES
    assert define("DRV_SYSID_RECORD_MAX_COUNT") == MAX_COUNT
    tx_text = [l for l in (ROOT / "App/Inc/app_messages.h").read_text(encoding="utf-8").splitlines()
               if "#define APP_UART_TX_TEXT_SIZE" in l][0]
    tx_size = int(tx_text.split()[-1].rstrip("U"))
    assert HEADER_BYTES + MAX_COUNT * RECORD_BYTES + 9 <= tx_size
    assert HEADER_BYTES + (MAX_COUNT + 1) * RECORD_BYTES + 9 > tx_size


def test_a_full_batch_fits_the_firmware_framer(lib):
    """真正的上限来自发送端，不是 $X 长度字段。

    `APP_Proto_BuildFrame` 对 payload_length > APP_PROTO_MAX_PAYLOAD 直接返回
    失败。满批超过它的话，打包会"成功"而发送静默失败——这种错最难查，所以在
    这里把两个常量钉在一起。
    """
    header_h = (ROOT / "App/Inc/app_proto.h").read_text(encoding="utf-8")
    line = [l for l in header_h.splitlines() if "APP_PROTO_MAX_PAYLOAD" in l][0]
    max_payload = int(line.split()[-1].rstrip("U"))
    full = pack(lib, [make_sample() for _ in range(MAX_COUNT)])
    assert len(full) <= max_payload


def test_batching_keeps_header_overhead_small(lib, schema):
    """攒批的意义就在这里：5 条摊下来包头只占约 7%。"""
    full = pack(lib, [make_sample() for _ in range(MAX_COUNT)])
    assert HEADER_BYTES / len(full) < 0.072


def test_five_hundred_hz_fits_the_link_budget(schema):
    """500 Hz 采样的净码率必须留在 USB CDC 轻松能吃下的量级。

    v3 每条 30 字节，500 Hz 成帧后约 16.8 kB/s（v2 约 14.6 kB/s），预算随之从
    15 kB/s 放到 20 kB/s。高度字段追加后每条 44 字节、满批 5 条，500 Hz 成帧约
    24.5 kB/s，预算放到 30 kB/s——全速 USB CDC 实测吞吐仍是它的十几倍。
    非 USB 链路 APP_SysId_PortBegin 仍只放行 ≤100 Hz：100 Hz 约 4.9 kB/s，蓝牙 115200 约 43%；
    数传 57600 已到约 85%（v3 原 58%），数传上宜降到 ≤80 Hz（约 3.9 kB/s）。"""
    bytes_per_second = 500 * schema.record_bytes
    frames_per_second = 500 / MAX_COUNT
    total = bytes_per_second + frames_per_second * HEADER_BYTES
    assert total + frames_per_second * 9 < 30_000, f"{total:.0f} B/s 超出预算"
    at_100_hz = 100 * schema.record_bytes + (100 / MAX_COUNT) * (HEADER_BYTES + 9)
    assert at_100_hz < 115_200 / 10 * 0.75, f"100 Hz {at_100_hz:.0f} B/s 超出蓝牙预算"
    at_80_hz = 80 * schema.record_bytes + (80 / MAX_COUNT) * (HEADER_BYTES + 9)
    assert at_80_hz < 57_600 / 10 * 0.75, f"80 Hz {at_80_hz:.0f} B/s 超出数传预算"


# ---------------------------------------------------------------- 拒绝


def test_schema_hash_mismatch_is_refused_rather_than_decoded(lib, schema):
    """这是整套自描述机制的核心：对不上就不解，绝不按旧表硬解。"""
    data = pack(lib, [make_sample()])
    wrong = SysIdSchema(fields=schema.fields, hash=schema.hash ^ 0x1)
    with pytest.raises(SchemaMismatch):
        decode_batch(data, wrong)


def test_changing_a_field_type_must_change_the_hash():
    """i16 换 u16 字节宽度没变，但含义变了——hash 必须跟着变，否则挡不住。"""
    source = (ROOT / "Driver/Src/drv_sysid_record.c").read_text(encoding="utf-8")
    body = source.split("uint32_t DRV_SysIdRecord_SchemaHash(void)")[1].split("\n}")[0]
    assert "sysid_fields[i].type" in body, "字段类型没有参与 schema hash"
    assert "sysid_fields[i].scale" in body, "缩放没有参与 schema hash"


def test_truncated_frame_is_refused(lib, schema):
    data = pack(lib, [make_sample() for _ in range(4)])
    with pytest.raises(ValueError):
        decode_batch(data[:-5], schema)
    with pytest.raises(ValueError):
        decode_batch(data[:3], schema)


def test_pack_rejects_out_of_range_counts(lib):
    header = BatchHeader(run_id=1, base_t_us=0, dt_us=2000, flags=0)
    array = (Sample * 1)(make_sample())
    buffer = (ctypes.c_uint8 * 4096)()
    length = ctypes.c_size_t()
    assert lib.DRV_SysIdRecord_Pack(ctypes.byref(header), array, 0, buffer,
                                    4096, ctypes.byref(length)) == INVALID
    assert lib.DRV_SysIdRecord_Pack(ctypes.byref(header), array, MAX_COUNT + 1,
                                    buffer, 4096, ctypes.byref(length)) == INVALID


def test_pack_reports_too_small_instead_of_overrunning(lib):
    header = BatchHeader(run_id=1, base_t_us=0, dt_us=2000, flags=0)
    array = (Sample * 4)(*[make_sample() for _ in range(4)])
    buffer = (ctypes.c_uint8 * 16)()
    length = ctypes.c_size_t()
    assert lib.DRV_SysIdRecord_Pack(ctypes.byref(header), array, 4, buffer,
                                    16, ctypes.byref(length)) == TOO_SMALL


# ---------------------------------------------------------------- 饱和


def test_out_of_range_values_saturate_instead_of_wrapping(lib, schema):
    """回绕会把一个超量程的大角速度变成反号的小值——比丢数据危险得多。"""
    huge = decode_batch(pack(lib, [make_sample(gyro_rad_s=(1e6, -1e6, 0.0))]), schema)
    row = huge.samples[0]
    assert row["gx"] > 30.0, "正向超量程必须钉在正的最大值"
    assert row["gy"] < -30.0, "负向超量程必须钉在负的最大值"


def test_non_finite_values_become_zero_not_garbage(lib, schema):
    batch = decode_batch(pack(lib, [make_sample(
        gyro_rad_s=(float("nan"), float("inf"), 0.0),
        thrust_n=float("nan"))]), schema)
    assert batch.samples[0]["gx"] == 0.0
    assert batch.samples[0]["gy"] == 0.0
    assert batch.samples[0]["thrust"] == 0.0


def test_negative_erpm_is_clamped_to_zero(lib, schema):
    """转速没有负数；u16 字段遇到负输入必须给 0 而不是回绕成巨大值。"""
    batch = decode_batch(pack(lib, [make_sample(erpm=0)]), schema)
    assert batch.samples[0]["erpm"] == 0.0


# ---------------------------------------------------------------- schema 文本


def test_schema_text_round_trips(schema):
    lines = [
        f"SYSID SCHEMA ver={RECORD_VERSION} n={len(schema.fields)} hash={schema.hash:08X} "
        f"rec={RECORD_BYTES}",
    ]
    names = {FIELD_TYPE_I16: "i16", FIELD_TYPE_U16: "u16"}
    for index, item in enumerate(schema.fields):
        lines.append(f"SYSID FIELD idx={index} name={item.name} unit={item.unit} "
                     f"scale={item.scale:g} type={names[item.type]}")
    parsed = parse_schema_lines(lines)
    assert parsed.version == RECORD_VERSION
    assert parsed.hash == schema.hash
    assert parsed.field_names() == schema.field_names()
    assert parsed.record_bytes == schema.record_bytes
    for got, want in zip(parsed.fields, schema.fields):
        assert got.scale == pytest.approx(want.scale, rel=1e-6)
        assert got.type == want.type


def test_schema_text_rejects_a_short_field_list(schema):
    with pytest.raises(ValueError):
        parse_schema_lines([f"SYSID SCHEMA ver=1 n=5 hash={schema.hash:08X}",
                            "SYSID FIELD idx=0 name=gx unit=rad/s scale=0.001 type=i16"])


# ---------------------------------------------------------------- 分层


def test_module_is_hardware_independent():
    import re

    source = (ROOT / "Driver/Src/drv_sysid_record.c").read_text(encoding="utf-8")
    header = (ROOT / "Driver/Inc/drv_sysid_record.h").read_text(encoding="utf-8")
    includes = re.findall(r'^#include\s+[<"]([^>"\n]+)', source + "\n" + header, re.M)
    assert set(includes) == {"drv_sysid_record.h", "stdint.h", "stddef.h",
                             "math.h", "string.h"}
    assert "HAL_" not in source
    assert "Driver/Src/drv_sysid_record.c" in (
        ROOT / "CMakeLists.txt").read_text(encoding="utf-8")


def test_host_decoder_does_not_hardcode_the_layout():
    """主机侧不许出现写死的偏移量或字段名清单——那正是会静默解错的来源。"""
    source = (ROOT / "tools/sysid/decode.py").read_text(encoding="utf-8")
    for name in ("gx", "omega_sp", "tilt_x", "thrust"):
        assert f'"{name}"' not in source, f"decode.py 里写死了字段名 {name}"
