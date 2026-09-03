"""修 bug：`$X` 解析器有两处会永久停摆的重同步空洞。

发现于 2026-09-03，R-T1-2 的模糊测试。缺陷早于遥测流工作，命中的是所有
`$X` 帧共用的 `TransportBase._consume_buffer`，**文本回复同样中招**。

固定下来的可观测事实（修复前，8 帧一串，把第 2 帧内某个字节整字节翻 0xFF，
逐个偏移穷举）：

  * 45 B 的遥测掩码帧：45 个位置里有 **2 个**会让其后的所有帧一条都解不出来，
    - `intra=2`（方向字节）→ 只解出 seq `[0, 1]`；
    - `intra=7`（len 高字节）→ 只解出 seq `[0, 1]`。
    其余 43 个位置只损失被打坏的那一帧，解出 `[0, 1, 3, 4, 5, 6, 7]`——
    说明重同步本身是有的，就这两个口子漏了。
  * 17 B 的文本回复帧（`$X` + `PROTO_MSG_TEXT_LINE`）：同样是方向字节与 len
    两处中招，证明这不是遥测帧特有的问题，命令回复一直暴露在同一个洞下。

根因，两处都是"既不消费字节、又永远不可能成功"的分支：

  1. 方向字节非法时不进帧分支，落到文本分支后 `buffer.find(b"$X")` 返回 0，
     `if frame_index > 0` 不成立就 `break`。缓冲区永远以这个假帧头开头，
     一个字节都不再消费——死锁。
  2. `len` 被打坏成一个巨大的值时，`if len(buffer) < frame_length: break`
     会一直等下去。$X 的 len 是 u16，最坏要等 65 KB；57600 baud 的数传上就是
     十几秒黑屏，而且这十几秒里到达的每一帧好数据都被吞进那个假帧里。

**为什么现有测试没挡住它**：`tests/test_panel_transport.py` 一类的用例喂的都是
结构完整的帧，从来没喂过被打坏的字节流。解析器的健壮性路径（丢字节重找帧头）
在此之前**一条测试都没有**——它不抛异常、不报错，只是安静地停下来。

修复只动这两个分支：非法方向字节丢一个字节重找；`len` 超过协议上限
（`PROTO_MAX_FRAME_PAYLOAD`）同样丢一个字节重找。`$X` 帧头、方向字节取值、
CRC8-DVB-S2 算法、以及正常帧的解析路径一字未动。

**修复后仍然存在、且无法在主机侧消除的残留行为**（如实记下来，不粉饰）：
`len` 被打坏成一个**合法范围内**的较大值时（比如 8 变成 247），解析器无从判断
它是假的，只能等够那么多字节、CRC 失败之后再丢一个字节重找。所以恢复不是
"一帧之内"，而是**有界**：最坏 `PROTO_MAX_FRAME_PAYLOAD + 9` ≈ 1 KB 的流，
57600 baud 上约 0.18 s。这是"长度前缀 + 帧尾单 CRC"这种帧格式的固有代价，
要根除得改线上格式（本期明令禁止）。修复把"永久停摆"降成了"有界延迟"。
"""

from __future__ import annotations

import queue
import random

from tools.panel_lib.proto import (
    PROTO_DIR_FROM_FC,
    PROTO_MAX_FRAME_PAYLOAD,
    PROTO_MSG_TEXT_LINE,
)
from tools.panel_lib.transport import TransportBase, build_proto_frame


class LoopbackTransport(TransportBase):
    def __init__(self) -> None:
        super().__init__(queue.Queue())

    @property
    def is_connected(self) -> bool:
        return True

    def start(self, *args, **kwargs) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        return True


def text_frame(index: int) -> bytes:
    return build_proto_frame(
        PROTO_DIR_FROM_FC, PROTO_MSG_TEXT_LINE, f"LINE {index:03d}".encode("ascii")
    )


def decode(blob: bytes, chunk: int = 7) -> list[str]:
    transport = LoopbackTransport()
    buffer = bytearray()
    for offset in range(0, len(blob), chunk):
        buffer += blob[offset:offset + chunk]
        transport._consume_buffer(buffer)

    lines: list[str] = []
    while not transport.rx_queue.empty():
        item = transport.rx_queue.get_nowait()
        if isinstance(item, tuple) and item[0] == "proto":
            lines.append(str(item[2]))
    return lines


def test_intact_stream_still_decodes_every_frame() -> None:
    frames = [text_frame(i) for i in range(8)]
    assert decode(b"".join(frames)) == [f"LINE {i:03d}" for i in range(8)]


def test_a_corrupt_direction_byte_only_costs_its_own_frame() -> None:
    """修复前：解到第 2 帧就永久停摆。"""
    frames = [text_frame(i) for i in range(8)]
    stream = bytearray(b"".join(frames))
    stream[len(frames[0]) * 2 + 2] ^= 0xFF          # 第 2 帧的方向字节

    lines = decode(bytes(stream))
    assert "LINE 007" in lines
    assert "LINE 002" not in lines
    assert len(lines) == 7


def test_a_corrupt_length_high_byte_only_costs_its_own_frame() -> None:
    """修复前：len 被放大到 u16 量级后解析器一直等下去，后面的帧全被吞掉。"""
    frames = [text_frame(i) for i in range(8)]
    stream = bytearray(b"".join(frames))
    stream[len(frames[0]) * 2 + 7] ^= 0xFF          # 第 2 帧的 len 高字节

    lines = decode(bytes(stream))
    assert "LINE 007" in lines
    assert len(lines) == 7


# 一帧被打坏之后，最坏要再走这么多字节才能重新同步上。
RECOVERY_WINDOW_BYTES = PROTO_MAX_FRAME_PAYLOAD + 9


def tail_frames(start: int) -> tuple[bytes, str]:
    """够长的一段好帧，长到覆盖最坏恢复窗口；返回字节和最后一帧的文本。"""
    frames = []
    index = start
    total = 0
    while total <= RECOVERY_WINDOW_BYTES * 2:
        frame = text_frame(index)
        frames.append(frame)
        total += len(frame)
        index += 1
    return b"".join(frames), f"LINE {index - 1:03d}"


def test_every_single_byte_flip_recovers_within_the_bounded_window() -> None:
    """全覆盖：帧内任何一个字节被打坏，解析器都必须在有界窗口内重新同步上。

    这条是上面两个缺陷的通用形式。修复前有 2 个位置会让流**永久**停摆——
    后面再来多少好数据都解不出来。现在最坏也只是被那个假长度吞掉一段。
    """
    frames = [text_frame(i) for i in range(8)]
    frame_length = len(frames[0])
    assert all(len(frame) == frame_length for frame in frames)
    tail, last_line = tail_frames(8)

    for intra in range(frame_length):
        stream = bytearray(b"".join(frames))
        stream[frame_length * 2 + intra] ^= 0xFF
        lines = decode(bytes(stream) + tail)
        assert last_line in lines, f"stalled after a flip at intra offset {intra}"


def test_recovery_after_a_corrupt_length_is_bounded_not_unbounded() -> None:
    """把残留行为本身钉住：假长度最多吞掉一个恢复窗口，不许再多。

    这不是"修好了"，是"从永久变成有界"。要根除得改线上帧格式。
    """
    frames = [text_frame(i) for i in range(3)]
    frame_length = len(frames[0])
    stream = bytearray(b"".join(frames))
    stream[frame_length * 2 + 6] ^= 0xFF            # len 低字节 -> 合法但假的大长度

    tail, last_line = tail_frames(3)
    lines = decode(bytes(stream) + tail)
    assert last_line in lines
    # 恢复窗口之后的帧一条都不该少。
    swallowed = sum(1 for i in range(3, 3 + len(tail) // frame_length)
                    if f"LINE {i:03d}" not in lines)
    assert swallowed * frame_length <= RECOVERY_WINDOW_BYTES


def test_an_impossible_length_is_rejected_instead_of_awaited() -> None:
    """len 超过协议上限时不许当成"还没收全"。"""
    transport = LoopbackTransport()
    bogus = bytearray(b"$X>")
    bogus += bytes([0, 0x01, 0x20])
    bogus += (PROTO_MAX_FRAME_PAYLOAD + 1).to_bytes(2, "little")
    bogus += b"\x00" * 2
    buffer = bytearray(bogus + text_frame(1))
    transport._consume_buffer(buffer)

    lines = []
    while not transport.rx_queue.empty():
        item = transport.rx_queue.get_nowait()
        if isinstance(item, tuple) and item[0] == "proto":
            lines.append(str(item[2]))
    assert lines == ["LINE 001"]


def test_random_corruption_never_stalls_the_parser() -> None:
    rng = random.Random(20260903)
    frames = [text_frame(i) for i in range(6)]
    tail, last_line = tail_frames(6)

    for _ in range(400):
        stream = bytearray(b"".join(frames))
        for _ in range(rng.randrange(1, 4)):
            at = rng.randrange(len(stream))
            stream[at] ^= 1 << rng.randrange(8)
        lines = decode(bytes(stream) + tail, chunk=rng.randrange(1, 40))
        # 尾部那一大段好帧从未被碰过：解析器必须重新同步上并解出最后一帧。
        assert last_line in lines


def test_the_wire_format_itself_is_untouched() -> None:
    """只修主机侧的重同步；帧头、方向字节、CRC 一字不动。"""
    frame = text_frame(0)
    assert frame[0:2] == b"$X"
    assert frame[2] == PROTO_DIR_FROM_FC
    assert frame[3] == 0
    assert frame[4] | (frame[5] << 8) == PROTO_MSG_TEXT_LINE
    assert frame[6] | (frame[7] << 8) == len(b"LINE 000")
    assert decode(frame) == ["LINE 000"]


def test_pathological_lengths_do_not_swallow_the_stream() -> None:
    """0xFFFF 的假 len 不许把 65 KB 的后续数据吞进一个永远等不到的帧里。"""
    transport = LoopbackTransport()
    # 9 字节的假帧头（len=0xFFFF，远超协议上限）后面紧跟一帧好的。
    buffer = bytearray(b"$X>\x00\x01\x20\xff\xff\x00" + text_frame(1))
    transport._consume_buffer(buffer)

    lines = []
    while not transport.rx_queue.empty():
        item = transport.rx_queue.get_nowait()
        if isinstance(item, tuple) and item[0] == "proto":
            lines.append(str(item[2]))
    assert lines == ["LINE 001"]
    # 假帧头被逐字节丢弃，没有残留在缓冲区里等 65 KB。
    assert len(buffer) == 0
