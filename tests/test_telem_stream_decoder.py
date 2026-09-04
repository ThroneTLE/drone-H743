"""R-T1-2：上位机遥测流解码（`tools/panel_lib/telem_stream.py` + transport 二进制分支）。

四件事：

  1. **黄金向量对称**：`tests/golden/telem_frames_v1.bin` 是固件自己的编码器在
     宿主 gcc 上产出的字节（见 `tests/test_telem_stream_contract.py`）。这里把
     那份文件原样喂进 transport + 解码器，断言解出来的值与固件编码时的输入
     完全相同。两端不允许各写各的"看起来一样"。
  2. **模糊测试**：随机截断 / 插入 / 翻转字节的流，断言解码器永不产出错长度
     样本、能在一帧之内重新同步、hash 不符的帧一条都不进环形缓冲。
  3. **schema**：hash 与固件复算一致；表里任何一个字段变了 hash 必须跟着变；
     固件换表之后解码器要能明确地要求重拉，而不是继续解一堆错位的曲线。
  4. **环形缓冲**：有界、每通道独立时间轴、按时间升序出快照。
"""

from __future__ import annotations

import queue
import random
import struct
from pathlib import Path

import pytest

from tools.panel_lib import transport as transport_module
from tools.panel_lib.telem_stream import (
    TELEM_FRAME_FLAG_FULL_REFRESH,
    TELEM_FRAME_HEADER_BYTES,
    TelemDecoder,
    TelemRing,
    TelemSchema,
    fnv1a,
)
from tools.panel_lib.transport import PROTO_BINARY_FUNCTIONS, TransportBase
from tools.panel_lib.proto import PROTO_MSG_TELEM_FRAME

from tests.test_telem_stream_contract import GOLDEN_CASES, GOLDEN_FRAMES, expected_frame


ROOT = Path(__file__).resolve().parents[1]


class CollectingTransport(TransportBase):
    """只用来驱动 `_consume_buffer` 的最小 Transport。

    刻意用真实的 `TransportBase`：帧同步、CRC、二进制分支都在那里，测一个
    仿写的解析器等于什么都没测。
    """

    def __init__(self) -> None:
        super().__init__(queue.Queue())
        self.binary: list[tuple[int, bytes]] = []
        self.set_binary_sink(lambda fn, payload: self.binary.append((fn, payload)))

    @property
    def is_connected(self) -> bool:
        return True

    def start(self, *args, **kwargs) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        return True


def feed_bytes(transport: CollectingTransport, blob: bytes,
               chunk: int = 7) -> None:
    """按小块喂进去：真实串口就是这样零碎到达的，一次喂整块测不到跨块重组。"""
    buffer = bytearray()
    for offset in range(0, len(blob), chunk):
        buffer += blob[offset:offset + chunk]
        transport._consume_buffer(buffer)


# ---------------------------------------------------------------- 黄金向量


def test_transport_delivers_telemetry_payloads_as_bytes() -> None:
    assert PROTO_MSG_TELEM_FRAME in PROTO_BINARY_FUNCTIONS

    transport = CollectingTransport()
    feed_bytes(transport, GOLDEN_FRAMES.read_bytes())

    assert len(transport.binary) == len(GOLDEN_CASES)
    for function, payload in transport.binary:
        assert function == PROTO_MSG_TELEM_FRAME
        assert isinstance(payload, bytes)
    # 文本队列一条都不该有：二进制帧不许掉进 UTF-8 分支。
    assert transport.rx_queue.empty()


def test_v3_schema_preserves_body_frame_provenance() -> None:
    schema = TelemSchema()
    assert schema.feed_line(
        "TELEM ver=3 n=1 rate=40 page=1 hash=00000000 "
        "frame=body_flu contract=1"
    )
    assert schema.feed_line(
        "TELEM CH idx=0 name=vel_est_y unit=m/s min=-5.000 max=5.000 "
        "grp=nav param=-"
    )
    assert schema.feed_line("TELEM PAGE from=0 count=1 next=-1")

    assert schema.body_frame == "body_flu"
    assert schema.frame_contract == 1
    assert schema.canonical_text().startswith("v3|1|40|body_flu|1\n")


def test_v3_schema_rejects_missing_frame_but_v2_stays_legacy_compatible() -> None:
    schema = TelemSchema()
    assert not schema.feed_line("TELEM ver=3 n=1 rate=40 page=1 hash=00000000")
    assert not schema.feed_line(
        "TELEM ver=3 n=1 rate=40 page=1 hash=00000000 frame=body_frd contract=1"
    )
    assert not schema.feed_line(
        "TELEM ver=3 n=1 rate=40 page=1 hash=00000000 frame=body_flu contract=0"
    )

    assert schema.feed_line("TELEM ver=2 n=1 rate=40 page=1 hash=00000000")
    assert schema.body_frame == "legacy_unspecified"
    assert schema.frame_contract == 0
    assert schema.canonical_text().startswith("v2|1|40\n")


def test_decoder_reproduces_exactly_what_the_firmware_encoded() -> None:
    transport = CollectingTransport()
    feed_bytes(transport, GOLDEN_FRAMES.read_bytes())

    for (function, payload), case in zip(transport.binary, GOLDEN_CASES):
        count, seq, schema, t_us, dt_us, flags, mask, values = case
        decoder = TelemDecoder(schema_hash=schema)
        samples = decoder.feed(payload)

        channels = [i for i in range(64) if mask >> i & 1]
        assert len(samples) == count
        for sample_index, sample in enumerate(samples):
            assert sample.seq == seq
            assert sample.full_refresh == bool(flags & TELEM_FRAME_FLAG_FULL_REFRESH)
            assert sample.t_us == t_us + sample_index * dt_us
            chunk = values[sample_index * len(channels):(sample_index + 1) * len(channels)]
            assert sample.values == dict(zip(channels, chunk))
        assert decoder.stats.rejected_total == 0


def test_a_frame_whose_length_disagrees_with_its_mask_is_dropped_whole() -> None:
    """长度和掩码对不上就是错位的开始，半解一帧比丢一帧危险得多。"""
    payload = struct.pack("<BBHIIHHQ", 1, 1, 0, 0x1234, 0, 0, 0, 0b111)
    decoder = TelemDecoder(schema_hash=0x1234)

    assert decoder.feed(payload + struct.pack("<ff", 1.0, 2.0)) == []
    assert decoder.stats.rejected_length == 1
    assert decoder.feed(payload + struct.pack("<ffff", 1.0, 2.0, 3.0, 4.0)) == []
    assert decoder.stats.rejected_length == 2
    # 正好三个才收。
    assert len(decoder.feed(payload + struct.pack("<fff", 1.0, 2.0, 3.0))) == 1


def test_unknown_version_and_empty_mask_are_rejected() -> None:
    decoder = TelemDecoder(schema_hash=0x1234)
    assert decoder.feed(struct.pack("<BBHIIHHQ", 2, 1, 0, 0x1234, 0, 0, 0, 1) +
                        struct.pack("<f", 1.0)) == []
    assert decoder.stats.rejected_version == 1
    assert decoder.feed(struct.pack("<BBHIIHHQ", 1, 1, 0, 0x1234, 0, 0, 0, 0)) == []
    assert decoder.stats.rejected_mask == 1
    assert decoder.feed(b"\x01\x01") == []
    assert decoder.stats.rejected_length == 1


def test_channels_outside_the_table_are_rejected_not_ignored() -> None:
    decoder = TelemDecoder(schema_hash=0x1234, channel_count=8)
    payload = struct.pack("<BBHIIHHQ", 1, 1, 0, 0x1234, 0, 0, 0, 1 << 30)
    assert decoder.feed(payload + struct.pack("<f", 1.0)) == []
    assert decoder.stats.rejected_mask == 1


def test_sequence_gaps_are_counted() -> None:
    decoder = TelemDecoder(schema_hash=0x99)

    def frame(seq: int) -> bytes:
        return struct.pack("<BBHIIHHQ", 1, 1, seq, 0x99, seq * 1000, 0, 0, 1) + \
            struct.pack("<f", float(seq))

    for seq in (0, 1, 2):
        assert decoder.feed(frame(seq))
    assert decoder.stats.seq_gaps == 0

    assert decoder.feed(frame(7))
    assert decoder.stats.seq_gaps == 1
    assert decoder.stats.lost_frames == 4

    # 回绕不算丢帧。
    decoder2 = TelemDecoder(schema_hash=0x99)
    assert decoder2.feed(frame(0xFFFF))
    assert decoder2.feed(frame(0))
    assert decoder2.stats.lost_frames == 0


def test_timestamps_are_unwrapped_across_the_32_bit_rollover() -> None:
    decoder = TelemDecoder(schema_hash=0x99)

    def frame(seq: int, t_us: int) -> bytes:
        return struct.pack("<BBHIIHHQ", 1, 1, seq, 0x99, t_us, 0, 0, 1) + \
            struct.pack("<f", 0.0)

    first = decoder.feed(frame(0, 0xFFFFFF00))[0]
    second = decoder.feed(frame(1, 0x00000100))[0]
    # 71 分钟一次的回绕不能让时间轴倒退——倒退会让曲线折回去自己叠自己。
    assert second.t_us > first.t_us
    assert second.t_us - first.t_us == 0x200


# ---------------------------------------------------------------- 模糊测试


FUZZ_CASES = 400


def _reference_frame(seq: int) -> bytes:
    return expected_frame(
        count=1, seq=seq & 0xFFFF, schema=0xABCD1234, t_us=seq * 25000,
        dt_us=0, flags=0, mask=0b1011, values=[1.0, 2.0, 3.0],
    )


def _corrupt(blob: bytes, rng: random.Random) -> bytes:
    data = bytearray(blob)
    mode = rng.randrange(3)
    if not data:
        return bytes(data)
    if mode == 0:                                   # 截断
        cut = rng.randrange(len(data))
        del data[cut:cut + rng.randrange(1, 12)]
    elif mode == 1:                                 # 插入
        at = rng.randrange(len(data))
        data[at:at] = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 12)))
    else:                                           # 翻位
        at = rng.randrange(len(data))
        data[at] ^= 1 << rng.randrange(8)
    return bytes(data)


def test_fuzzed_stream_never_yields_a_wrong_length_sample() -> None:
    """任意截断 / 插入 / 翻转，解码器都不许产出一个错长度的样本。

    这是掩码帧存在的理由：定长 JustFloat 帧遇到同样的破坏会照常解出 28 个
    float，只是全部挪了位——不报错、不崩溃，只是数值"有点怪"。
    """
    rng = random.Random(20260903)
    produced = 0

    for _ in range(FUZZ_CASES):
        stream = b"".join(_reference_frame(seq) for seq in range(6))
        stream = _corrupt(stream, rng)

        transport = CollectingTransport()
        feed_bytes(transport, stream, chunk=rng.randrange(1, 40))

        decoder = TelemDecoder(schema_hash=0xABCD1234, channel_count=28)
        for _fn, payload in transport.binary:
            for sample in decoder.feed(payload):
                produced += 1
                mask = struct.unpack_from("<Q", payload, 16)[0]
                assert len(sample.values) == bin(mask).count("1")
                assert set(sample.values) <= set(range(28))

    # 破坏是随机的，绝大多数流里仍有完整帧；一个样本都没解出来说明测试本身失效了。
    assert produced > FUZZ_CASES


def test_parser_resynchronises_within_one_frame_after_garbage() -> None:
    """一次破坏只允许吃掉一帧，后面的帧必须照常解出来。"""
    rng = random.Random(4242)
    frames = [_reference_frame(seq) for seq in range(8)]

    for _ in range(120):
        stream = bytearray(b"".join(frames))
        # 只破坏第 2 帧内部的一个字节。
        offset = len(frames[0]) + len(frames[1]) + rng.randrange(len(frames[2]))
        stream[offset] ^= 0xFF

        transport = CollectingTransport()
        feed_bytes(transport, bytes(stream), chunk=rng.randrange(1, 33))

        decoder = TelemDecoder(schema_hash=0xABCD1234, channel_count=28)
        seqs = [s.seq for _fn, payload in transport.binary for s in decoder.feed(payload)]

        # 最后两帧一定要回来——重同步不能拖到流末尾。
        assert 6 in seqs and 7 in seqs
        assert len(seqs) >= len(frames) - 2


def test_a_hash_mismatch_never_reaches_the_ring() -> None:
    ring = TelemRing(capacity=64, channel_count=28)
    decoder = TelemDecoder(schema_hash=0xABCD1234, channel_count=28)

    transport = CollectingTransport()
    good = _reference_frame(0)
    stale = expected_frame(
        count=1, seq=1, schema=0xDEADBEEF, t_us=1000, dt_us=0, flags=0,
        mask=0b1011, values=[9.0, 9.0, 9.0],
    )
    feed_bytes(transport, good + stale)

    for _fn, payload in transport.binary:
        ring.push_many(decoder.feed(payload))

    assert decoder.stats.rejected_schema == 1
    assert decoder.needs_schema_reload
    assert decoder.schema_mismatch_hash == 0xDEADBEEF
    # 只有那一帧好的进了环。
    assert ring.used(0) == 1
    assert ring.snapshot(0)[1].tolist() == [1.0]


def test_fuzzed_stream_with_a_foreign_schema_stays_out_of_the_ring() -> None:
    rng = random.Random(777)
    for _ in range(120):
        stream = b"".join(
            expected_frame(count=1, seq=seq, schema=0x11111111, t_us=seq,
                           dt_us=0, flags=0, mask=0b111, values=[1.0, 2.0, 3.0])
            for seq in range(4)
        )
        transport = CollectingTransport()
        feed_bytes(transport, _corrupt(stream, rng), chunk=rng.randrange(1, 24))

        ring = TelemRing(capacity=32, channel_count=28)
        decoder = TelemDecoder(schema_hash=0x22222222, channel_count=28)
        for _fn, payload in transport.binary:
            ring.push_many(decoder.feed(payload))

        assert all(ring.used(index) == 0 for index in range(28))


# ---------------------------------------------------------------- schema


def schema_lines_from_firmware(tmp_path) -> list[str]:
    from tests.test_telemetry_schema_contract import build_and_run

    return build_and_run(tmp_path)


def build_schema(lines: list[str]) -> TelemSchema:
    schema = TelemSchema()
    for line in lines:
        schema.feed_line(line)
    return schema


def test_schema_assembles_from_the_real_firmware_reply(tmp_path) -> None:
    schema = build_schema(schema_lines_from_firmware(tmp_path))

    assert schema.complete
    assert schema.next_page is None
    assert schema.version == 3
    assert schema.body_frame == "body_flu"
    assert schema.frame_contract == 1
    assert schema.rate_hz == 40
    # 独立复算，不照抄回包里的 hash=。
    assert schema.hash_matches(), (
        f"reported=0x{schema.reported_hash:08X} computed=0x{schema.computed_hash():08X}"
    )


def test_schema_exposes_the_parameter_binding_for_sliders(tmp_path) -> None:
    schema = build_schema(schema_lines_from_firmware(tmp_path))
    by_name = {channel.name: channel for channel in schema.ordered()}

    assert by_name["roll_rate_kd"].param == "coax.roll_rate_kd"
    assert by_name["roll_rate_kd"].is_parameter
    assert not by_name["roll"].is_parameter
    # 滑块由通道表数据驱动生成，上位机不再自备一张增益表。
    sliders = [c.name for c in schema.ordered() if c.is_parameter]
    assert len(sliders) == 13


def test_changing_any_metadata_field_changes_the_computed_hash(tmp_path) -> None:
    lines = schema_lines_from_firmware(tmp_path)
    baseline = build_schema(lines).computed_hash()

    for old, new in (
        ("name=roll ", "name=rollX "),
        ("unit=deg ", "unit=rad "),
        ("grp=attitude ", "grp=att "),
        ("param=coax.roll_rate_kd", "param=coax.pitch_rate_kd"),
    ):
        mutated = [line.replace(old, new, 1) for line in lines]
        assert mutated != lines, old
        assert build_schema(mutated).computed_hash() != baseline, old


def test_a_new_header_restarts_assembly_instead_of_merging(tmp_path) -> None:
    """重连时收到新表头，残留的旧通道行必须整体作废，不能和新表混在一起。"""
    lines = schema_lines_from_firmware(tmp_path)
    schema = build_schema(lines)
    assert schema.complete

    schema.feed_line("TELEM ver=2 n=3 rate=40 page=6 hash=00000000")
    assert not schema.complete
    assert schema.channels == {}


def test_fnv1a_matches_the_firmware_seed_and_prime() -> None:
    # 与 App/Src/app_telemetry.c 的 0x811C9DC5 / 0x01000193 同源的已知向量。
    assert fnv1a("") == 0x811C9DC5
    assert fnv1a("a") == 0xE40C292C
    assert fnv1a("foobar") == 0xBF9CF968


# ---------------------------------------------------------------- 环形缓冲


def test_ring_is_bounded_and_ordered() -> None:
    ring = TelemRing(capacity=8, channel_count=4)
    decoder = TelemDecoder(schema_hash=None, channel_count=4)

    for seq in range(20):
        payload = struct.pack("<BBHIIHHQ", 1, 1, seq & 0xFFFF, 0, seq * 1000, 0, 0, 1)
        payload += struct.pack("<f", float(seq))
        ring.push_many(decoder.feed(payload))

    times, values = ring.snapshot(0)
    assert len(values) == 8
    assert values.tolist() == [float(v) for v in range(12, 20)]
    assert list(times) == sorted(times)
    # 没出现过的通道不许凭空长出样本。
    assert ring.used(3) == 0


def test_each_channel_keeps_its_own_timeline() -> None:
    """掩码逐帧变化，通道不能共用一根时间轴——否则缺席的通道得靠补值。"""
    ring = TelemRing(capacity=16, channel_count=4)
    decoder = TelemDecoder(schema_hash=None, channel_count=4)

    # 三帧只有通道 0，第四帧才带上通道 2。
    for seq in range(3):
        payload = struct.pack("<BBHIIHHQ", 1, 1, seq, 0, seq * 1000, 0, 0, 0b001)
        ring.push_many(decoder.feed(payload + struct.pack("<f", float(seq))))
    payload = struct.pack("<BBHIIHHQ", 1, 1, 3, 0, 3000, 0, 0, 0b101)
    ring.push_many(decoder.feed(payload + struct.pack("<ff", 3.0, 42.0)))

    assert ring.used(0) == 4
    assert ring.used(2) == 1
    assert ring.snapshot(2)[1].tolist() == [42.0]
    assert ring.snapshot(2)[0].tolist() == [pytest.approx(0.003)]


def test_ring_clear_resets_every_channel() -> None:
    ring = TelemRing(capacity=4, channel_count=2)
    decoder = TelemDecoder(schema_hash=None, channel_count=2)
    payload = struct.pack("<BBHIIHHQ", 1, 1, 0, 0, 0, 0, 0, 0b11)
    ring.push_many(decoder.feed(payload + struct.pack("<ff", 1.0, 2.0)))
    assert ring.used(0) == 1
    ring.clear()
    assert ring.used(0) == 0 and ring.used(1) == 0


# ---------------------------------------------------------------- 边界


def test_transport_change_is_confined_to_the_binary_branch() -> None:
    source = (ROOT / "tools" / "panel_lib" / "transport.py").read_text(encoding="utf-8")

    # 文本路径一字未改：$X 帧头、方向字节、CRC 都不动。
    assert 'self.rx_queue.put(("proto", function, text))' in source
    assert "proto_crc8_dvb_s2(body) == frame[-1]" in source
    assert "if function in PROTO_BINARY_FUNCTIONS:" in source


def test_panel_entry_point_is_untouched_by_this_req() -> None:
    """R-T1-2 的验收判据之一：不改 tools/drone_tcp_panel.py。"""
    panel = (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")
    assert "telem_stream" not in panel
