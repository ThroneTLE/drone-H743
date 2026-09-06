"""遥测流 v2 的上位机侧：通道表装配、掩码帧解码、每通道环形缓冲。

对应固件的 `App/Src/app_telem_frame.c`（帧格式）与 `App/Src/app_telemetry.c`
（通道表与 FNV-1a 指纹）。帧格式的唯一事实源是
`doc/telemetry-protocol.md`；本模块不重新解释它，只实现它。

三件事分开放：

* `TelemSchema` —— 由 `TELEM?` / `TELEM CH from=` 的文本回包装配出来的通道表，
  并**独立复算**一遍固件的 hash。复算而不是照抄回包里的 `hash=`：照抄的话，
  固件改了通道表但 hash 计算漏了某个字段这种错误，上位机永远发现不了。
* `TelemDecoder` —— 吃 `$X` 帧的 payload，吐 `TelemSample`。所有拒绝路径都
  记在计数器上，一条都不许悄悄丢。
* `TelemRing` —— 每通道一条环形缓冲。**在收线程里写**，Tk 线程只读快照，
  这样波形不会被 Tk 的事件循环节奏牵着走。

不同帧的掩码不同，所以每条通道有自己的时间轴，不能共用一根。
"""

from __future__ import annotations

import struct
import threading
from dataclasses import dataclass, field

import numpy as np

from .proto import parse_kv


# 帧头（payload 内部，不含 $X 那 9 字节）
#
# v2 起掩码是变长的：flags 的 WIDE_MASK 位为 0 时掩码 8 字节（通道 0..63）、
# 头长 24；为 1 时掩码 16 字节（通道 0..127）、头长 32。实时通道全在低 64 位，
# 所以稳态帧仍是 24 字节头，与 v1 逐字节等长；只有触及高通道的全量刷新帧变宽。
# **不接受 v1**：v1 的 flags bit1 恒为 0，用 v1 规则去读一个宽掩码帧会把掩码高
# 半当成数据，长度校验虽然会拦下，但版本号拦得更早也更说得清。
TELEM_FRAME_VERSION = 2
TELEM_FRAME_HEADER_BYTES = 24
TELEM_FRAME_HEADER_BYTES_WIDE = 32
TELEM_FRAME_FLAG_FULL_REFRESH = 0x0001
TELEM_FRAME_FLAG_WIDE_MASK = 0x0002
TELEM_FRAME_MAX_CHANNELS = 128

_HEADER_STRUCT = struct.Struct("<BBHIIHHQ")
_MASK_HI_STRUCT = struct.Struct("<Q")

# t_us 是固件 64 位微秒时间戳的低 32 位，约 71 分钟回绕一次。
TELEM_TIME_WRAP_US = 1 << 32

FNV_OFFSET_BASIS = 0x811C9DC5
FNV_PRIME = 0x01000193


def fnv1a(text: str, seed: int = FNV_OFFSET_BASIS) -> int:
    """FNV-1a 32 位，与 App/Src/app_telemetry.c 的 app_telem_hash_bytes 一致。"""
    digest = seed
    for byte in text.encode("ascii"):
        digest ^= byte
        digest = (digest * FNV_PRIME) & 0xFFFFFFFF
    return digest


@dataclass(frozen=True)
class TelemChannel:
    index: int
    name: str
    unit: str
    group: str
    param: str
    min_text: str
    max_text: str

    @property
    def minimum(self) -> float:
        return float(self.min_text)

    @property
    def maximum(self) -> float:
        return float(self.max_text)

    @property
    def is_parameter(self) -> bool:
        """带参数键的通道 = 滑块；其余 = 曲线候选（规划文档 §2.3）。"""
        return bool(self.param) and self.param != "-"


class TelemSchema:
    """`TELEM?` + `TELEM CH from=` 分页装配出来的通道表。

    装配过程按分页回包驱动，中途可能不完整；`complete` 为真之前不要用它建图。
    """

    def __init__(self) -> None:
        self.version = 0
        self.channel_count = 0
        self.rate_hz = 0
        self.page_size = 0
        self.body_frame = "legacy_unspecified"
        self.frame_contract = 0
        self.reported_hash: int | None = None
        self.channels: dict[int, TelemChannel] = {}
        self.next_page: int | None = 0

    # ------------------------------------------------------------- 装配

    def feed_line(self, line: str) -> bool:
        """喂一行 TELEM 回包。返回 True 表示这行被本表吃掉了。"""
        if line.startswith("TELEM CH "):
            return self._feed_channel(line)
        if line.startswith("TELEM PAGE "):
            return self._feed_page(line)
        if line.startswith("TELEM STREAM "):
            # 流状态行归示波器页的统计条，不属于通道表。
            return False
        if line.startswith("TELEM ver="):
            return self._feed_header(line)
        return False

    def _feed_header(self, line: str) -> bool:
        values = parse_kv(line)
        try:
            version = int(values["ver"])
            channel_count = int(values["n"])
            rate_hz = int(values["rate"])
            page_size = int(values["page"])
            reported_hash = int(values["hash"], 16)
            body_frame = values.get("frame", "legacy_unspecified")
            frame_contract = int(values.get("contract", "0"))
        except (KeyError, ValueError):
            return False
        if version >= 3 and (
            body_frame != "body_flu" or frame_contract <= 0
        ):
            return False
        arriving = (version, channel_count, rate_hz, reported_hash,
                    body_frame, frame_contract)
        # 表头换了就重新装配：上一轮的残表不能和新表混在一起。表头**没换**时
        # 保留已收到的通道——数传出口上握手要 14 个来回、近百行文本，中途掉行
        # 是常态，每次重试都清零的话残表永远补不齐（2026-09-06 实机：面板卡在
        # "等待通道表"不动）。hash 覆盖整张表的全部字段，相等即可安全续拼。
        if arriving != self.identity:
            self.channels = {}
        self.version = version
        self.channel_count = channel_count
        self.rate_hz = rate_hz
        self.page_size = page_size
        self.reported_hash = reported_hash
        self.body_frame = body_frame
        self.frame_contract = frame_contract
        self.next_page = 0
        return True

    @property
    def identity(self) -> tuple:
        """这张表是"哪一张"。字段与固件表头一一对应，用于判断能否续拼。"""
        return (self.version, self.channel_count, self.rate_hz,
                self.reported_hash, self.body_frame, self.frame_contract)

    def _feed_channel(self, line: str) -> bool:
        values = parse_kv(line)
        try:
            index = int(values["idx"])
            channel = TelemChannel(
                index=index,
                name=values["name"],
                unit=values["unit"],
                group=values["grp"],
                # v1 固件没有 param 字段；缺就是"没有参数"，不是解析失败。
                param=values.get("param", "-"),
                min_text=values["min"],
                max_text=values["max"],
            )
        except (KeyError, ValueError):
            return False
        self.channels[index] = channel
        return True

    def _feed_page(self, line: str) -> bool:
        values = parse_kv(line)
        try:
            next_from = int(values["next"])
        except (KeyError, ValueError):
            return False
        self.next_page = None if next_from < 0 else next_from
        return True

    @property
    def complete(self) -> bool:
        return (
            self.channel_count > 0
            and len(self.channels) == self.channel_count
            and set(self.channels) == set(range(self.channel_count))
        )

    @property
    def first_missing(self) -> int | None:
        """还没收到的最小通道号；表齐了或还没表头则为 None。

        续拉的起点。掉一行就从头重来是这条链路上永远拼不齐的原因。
        """
        if self.channel_count <= 0:
            return None
        for index in range(self.channel_count):
            if index not in self.channels:
                return index
        return None

    def ordered(self) -> list[TelemChannel]:
        return [self.channels[i] for i in sorted(self.channels)]

    # ------------------------------------------------------------- 指纹

    def canonical_text(self) -> str:
        """与 App/Src/app_telemetry.c::APP_Telemetry_SchemaHash 逐字节相同的规范化文本。

        v2 起 param 也进指纹：滑块是按 param 数据驱动生成的，param 变了而
        hash 不变，滑块就会绑到错的参数上，而且没有任何人会报错。
        """
        if self.version >= 3:
            text = (
                f"v{self.version}|{self.channel_count}|{self.rate_hz}|"
                f"{self.body_frame}|{self.frame_contract}\n"
            )
        else:
            text = f"v{self.version}|{self.channel_count}|{self.rate_hz}\n"
        for channel in self.ordered():
            text += (
                f"{channel.index}|{channel.name}|{channel.unit}|"
                f"{channel.min_text}|{channel.max_text}|{channel.group}|"
                f"{channel.param}\n"
            )
        return text

    def computed_hash(self) -> int:
        return fnv1a(self.canonical_text())

    def hash_matches(self) -> bool:
        """独立复算的指纹与固件自报的是否一致。"""
        return self.complete and self.reported_hash == self.computed_hash()


@dataclass
class TelemSample:
    """一个样本：某一瞬间、某一组通道的值。"""

    t_us: int
    values: dict[int, float]
    seq: int
    full_refresh: bool = False


@dataclass
class TelemDecoderStats:
    frames_ok: int = 0
    samples: int = 0
    rejected_length: int = 0
    rejected_version: int = 0
    rejected_schema: int = 0
    rejected_mask: int = 0
    seq_gaps: int = 0
    lost_frames: int = 0

    @property
    def rejected_total(self) -> int:
        return (
            self.rejected_length
            + self.rejected_version
            + self.rejected_schema
            + self.rejected_mask
        )


class TelemDecoder:
    """掩码帧解码器。

    只有**完全自洽**的帧才产出样本：版本对、长度正好等于
    `header + 4*count*popcount(mask)`（header 由 flags 的 WIDE_MASK 位决定，
    24 或 32）、掩码不含表外通道、schema 指纹与当前通道表一致。任何一条不满足
    都整帧丢弃并计数——半解的帧比丢掉的帧危险得多，它长度合法、看起来正常，
    只是每条曲线都挪了一格。
    """

    def __init__(self, schema_hash: int | None = None,
                 channel_count: int = TELEM_FRAME_MAX_CHANNELS) -> None:
        self.schema_hash = schema_hash
        self.channel_count = channel_count
        self.stats = TelemDecoderStats()
        self._last_seq: int | None = None
        self._time_base = 0
        self._last_raw_t_us: int | None = None
        self.schema_mismatch_hash: int | None = None

    def bind_schema(self, schema: TelemSchema) -> None:
        self.schema_hash = schema.computed_hash()
        self.channel_count = schema.channel_count
        self.schema_mismatch_hash = None
        self.reset_timeline()

    def reset_timeline(self) -> None:
        self._last_seq = None
        self._time_base = 0
        self._last_raw_t_us = None

    @property
    def needs_schema_reload(self) -> bool:
        """收到过一个指纹不认识的帧 —— 固件通道表变了，得重拉 `TELEM?`。"""
        return self.schema_mismatch_hash is not None

    def feed(self, payload: bytes) -> list[TelemSample]:
        if len(payload) < TELEM_FRAME_HEADER_BYTES:
            self.stats.rejected_length += 1
            return []

        version, count, seq, schema, t_us, dt_us, flags, mask = _HEADER_STRUCT.unpack_from(
            payload, 0
        )

        if version != TELEM_FRAME_VERSION:
            self.stats.rejected_version += 1
            return []

        # 宽掩码：高 64 位紧跟在低 64 位之后，数据区随之后移 8 字节。宽窄读
        # flags 而不是猜长度——长度校验要拿它当输入，不能反过来靠长度推宽度。
        if flags & TELEM_FRAME_FLAG_WIDE_MASK:
            header_bytes = TELEM_FRAME_HEADER_BYTES_WIDE
            if len(payload) < header_bytes:
                self.stats.rejected_length += 1
                return []
            mask |= _MASK_HI_STRUCT.unpack_from(payload, TELEM_FRAME_HEADER_BYTES)[0] << 64
        else:
            header_bytes = TELEM_FRAME_HEADER_BYTES

        if count == 0 or mask == 0:
            self.stats.rejected_mask += 1
            return []

        if self.channel_count < TELEM_FRAME_MAX_CHANNELS:
            if mask >> self.channel_count:
                # 表外通道：宁可整帧丢，也不"忽略高位"——高位有东西说明两端
                # 对通道表的理解已经不一致了。
                self.stats.rejected_mask += 1
                return []

        channels = [i for i in range(TELEM_FRAME_MAX_CHANNELS) if mask >> i & 1]
        expected = header_bytes + 4 * count * len(channels)
        if len(payload) != expected:
            self.stats.rejected_length += 1
            return []

        if self.schema_hash is not None and schema != self.schema_hash:
            self.stats.rejected_schema += 1
            self.schema_mismatch_hash = schema
            return []

        base_t_us = self._unwrap(t_us)
        self._note_seq(seq)

        floats = struct.unpack_from(f"<{count * len(channels)}f", payload,
                                    header_bytes)
        full = bool(flags & TELEM_FRAME_FLAG_FULL_REFRESH)
        samples: list[TelemSample] = []
        for sample_index in range(count):
            chunk = floats[sample_index * len(channels):(sample_index + 1) * len(channels)]
            samples.append(
                TelemSample(
                    t_us=base_t_us + sample_index * dt_us,
                    values=dict(zip(channels, chunk)),
                    seq=seq,
                    full_refresh=full,
                )
            )

        self.stats.frames_ok += 1
        self.stats.samples += len(samples)
        return samples

    def _unwrap(self, t_us: int) -> int:
        """把低 32 位时间戳解回绕成单调的绝对微秒。"""
        if self._last_raw_t_us is not None and t_us < self._last_raw_t_us:
            # 只在真正跨过 2^32 时才加一圈。倒退小于半圈的一律当回绕处理，
            # 因为固件的 t_us 本来就是单调的。
            self._time_base += TELEM_TIME_WRAP_US
        self._last_raw_t_us = t_us
        return self._time_base + t_us

    def _note_seq(self, seq: int) -> None:
        if self._last_seq is not None:
            gap = (seq - self._last_seq - 1) & 0xFFFF
            if gap:
                self.stats.seq_gaps += 1
                self.stats.lost_frames += gap
        self._last_seq = seq


class TelemRing:
    """每通道一条环形缓冲：收线程写，Tk 线程读快照。

    每条通道有自己的时间轴，不共用一根：掩码逐帧变化，参数通道只在值变了那
    一帧出现，硬拉成同一根时间轴就得给缺席的通道补值，那等于凭空造数据。
    """

    def __init__(self, capacity: int = 2400,
                 channel_count: int = TELEM_FRAME_MAX_CHANNELS) -> None:
        self.capacity = int(capacity)
        self.channel_count = int(channel_count)
        self._lock = threading.Lock()
        self._time = np.zeros((self.channel_count, self.capacity), dtype=np.float64)
        self._value = np.zeros((self.channel_count, self.capacity), dtype=np.float32)
        self._head = np.zeros(self.channel_count, dtype=np.int64)
        self._used = np.zeros(self.channel_count, dtype=np.int64)

    def clear(self) -> None:
        with self._lock:
            self._head[:] = 0
            self._used[:] = 0

    def push(self, sample: TelemSample) -> None:
        with self._lock:
            seconds = sample.t_us * 1e-6
            for index, value in sample.values.items():
                if not 0 <= index < self.channel_count:
                    continue
                head = int(self._head[index])
                self._time[index, head] = seconds
                self._value[index, head] = value
                self._head[index] = (head + 1) % self.capacity
                if self._used[index] < self.capacity:
                    self._used[index] += 1

    def push_many(self, samples: list[TelemSample]) -> None:
        for sample in samples:
            self.push(sample)

    def used(self, index: int) -> int:
        return int(self._used[index])

    def latest(self, index: int) -> tuple[float, float] | None:
        """最新一个样本 `(t_seconds, value)`，没有则 None。

        数值卡这类只关心"现在是多少"的组件走这里，不走 `snapshot()`：后者要拷
        整条环形缓冲（2400 个点），一屏二十张卡每 33 ms 拷一遍纯属浪费。
        """
        if not 0 <= index < self.channel_count:
            return None
        with self._lock:
            used = int(self._used[index])
            if used == 0:
                return None
            last = (int(self._head[index]) - 1) % self.capacity
            return float(self._time[index, last]), float(self._value[index, last])

    def snapshot(self, index: int) -> tuple[np.ndarray, np.ndarray]:
        """按时间升序返回该通道的 (t_seconds, values) 拷贝。"""
        if not 0 <= index < self.channel_count:
            return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float32)
        with self._lock:
            used = int(self._used[index])
            head = int(self._head[index])
            if used == 0:
                return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float32)
            if used < self.capacity:
                return (self._time[index, :used].copy(),
                        self._value[index, :used].copy())
            order = np.r_[head:self.capacity, 0:head]
            return self._time[index, order].copy(), self._value[index, order].copy()


__all__ = [
    "TELEM_FRAME_FLAG_FULL_REFRESH",
    "TELEM_FRAME_FLAG_WIDE_MASK",
    "TELEM_FRAME_HEADER_BYTES",
    "TELEM_FRAME_HEADER_BYTES_WIDE",
    "TELEM_FRAME_MAX_CHANNELS",
    "TELEM_FRAME_VERSION",
    "TELEM_TIME_WRAP_US",
    "TelemChannel",
    "TelemDecoder",
    "TelemDecoderStats",
    "TelemRing",
    "TelemSample",
    "TelemSchema",
    "fnv1a",
]
