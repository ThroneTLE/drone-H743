"""R-T1-6（TK-05）：遥测录制服务。

页面原来自己拿着一个文件句柄和一个 deque，在 Tk 的渲染回调里 while 循环写盘。四个
问题是同一个根因的四张脸——**没有“录制会话”这个东西**，只有一堆散落的全局变量：

* 文件名只有 `telem_HHMMSS.csv` 且用 `"w"` 打开，同一秒再点一次录制，前一份数据被
  截断成 0 行（N13，P1：这是**已完成数据被静默销毁**）；
* 表头在开始时按当时的通道表写死，写行时却读**当前**全局 `channel_count`，录制中途
  换 schema 就得到宽度 3/3/4 的 CSV，文件头的指纹还是旧的（N12）；
* 写失败时异常从渲染回调里冒出去，句柄还留着、样本已经出队，之后每一帧重复报错，
  用户看不出这份文件到底完不完整（N11）；
* 整条队列在渲染回调里一次写完，慢盘时界面就跟着停（N10）。

所以这里的核心不是“加个线程”，是把**会话**变成一等公民：一个会话锁定它自己的
文件、通道表指纹、列宽和连接代次，从开始到结束都不会被外面的全局状态改写。写盘在
后台线程；Tk 线程只提交请求、读状态。

队列是**有界**的：`tk-ui` 模式明确禁止用无限队列或静默丢数据来解决背压，所以溢出
既要有计数，也要写进文件尾的说明行里，让事后拿到 CSV 的人能看见。
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Callable, Sequence


# 40 Hz 下 20000 行约 8 分钟余量；写盘线程正常情况下根本追得上，这个上限只在
# 磁盘真的卡住时起作用，让“丢了多少”变成一个数字而不是一次内存膨胀。
DEFAULT_QUEUE_LIMIT = 20000
# 停止/等待时最多阻塞多久。UI 线程不能为了写盘无限期停住。
DEFAULT_DRAIN_TIMEOUT_S = 5.0

STATE_IDLE = "idle"
STATE_RECORDING = "recording"
STATE_FINISHED = "finished"
STATE_FAILED = "failed"

REASON_USER = "user"
REASON_SCHEMA_CHANGED = "schema_changed"
REASON_LINK_CHANGED = "link_changed"
REASON_SHUTDOWN = "shutdown"
REASON_WRITE_ERROR = "write_error"

_REASON_TEXT = {
    REASON_USER: "用户停止",
    REASON_SCHEMA_CHANGED: "通道表变化",
    REASON_LINK_CHANGED: "连接代次变化",
    REASON_SHUTDOWN: "面板退出",
    REASON_WRITE_ERROR: "写入失败",
}


@dataclass(frozen=True)
class RecordSchema:
    """一份**冻结**的通道表快照。会话开始后它就不再跟着面板的全局 schema 变。"""

    schema_hash: int
    channel_names: tuple[str, ...]

    @property
    def channel_count(self) -> int:
        return len(self.channel_names)

    @classmethod
    def from_telem_schema(cls, schema) -> "RecordSchema":
        return cls(
            schema_hash=int(schema.computed_hash()),
            channel_names=tuple(channel.name for channel in schema.ordered()),
        )


@dataclass(frozen=True)
class RecordStatus:
    """给 Tk 线程读的状态快照。读它不需要碰锁以外的任何东西，更不会碰磁盘。"""

    state: str = STATE_IDLE
    path: Path | None = None
    rows_written: int = 0
    rows_queued: int = 0
    rows_dropped: int = 0
    schema_hash: int | None = None
    generation: int | None = None
    end_reason: str | None = None
    error: str | None = None

    @property
    def active(self) -> bool:
        return self.state == STATE_RECORDING

    def describe(self) -> str:
        """一行给用户看的话。失败和“结束了但不是你按的”必须能分辨。"""
        if self.state == STATE_IDLE:
            return "未录制"
        name = self.path.name if self.path is not None else "?"
        if self.state == STATE_RECORDING:
            if self.rows_dropped:
                return f"{name}（已丢 {self.rows_dropped} 行）"
            return name
        if self.state == STATE_FAILED:
            return f"录制失败：{self.error}（部分文件 {name}）"
        reason = _REASON_TEXT.get(self.end_reason or "", self.end_reason or "")
        if self.end_reason in (None, REASON_USER):
            return f"已保存 {name}（{self.rows_written} 行）"
        return f"已结束 {name}：{reason}（{self.rows_written} 行）"


@dataclass
class _Session:
    """一次录制。文件、列宽、指纹、代次在这里绑定，之后谁也改不了。"""

    ident: int
    path: Path
    handle: object
    schema: RecordSchema
    generation: int | None
    rows_written: int = 0
    rows_dropped: int = 0
    finished: bool = False
    end_reason: str | None = None
    error: str | None = None


@dataclass
class _Command:
    kind: str                       # "row" | "finish"
    session: _Session
    sample: object = None
    reason: str | None = None


def unique_record_path(directory: Path, stem: str, suffix: str = ".csv"):
    """排他创建一个不会碰撞的文件，返回 `(handle, path)`。

    用 `"x"` 而不是 `"w"`：这是 N13 的根因。`"w"` 会把同名文件截断成 0 字节，而同名
    在现实里太容易出现——同一秒内双击、两个面板实例对着同一个目录、失败后立刻重试。
    `"x"` 由操作系统保证“不存在才创建”，多进程之间也成立；已经存在就换一个后缀继续
    试，**绝不覆盖任何已完成的文件**。
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    candidate = directory / f"{stem}{suffix}"
    index = 1
    while True:
        try:
            handle = candidate.open("x", encoding="utf-8", newline="")
        except FileExistsError:
            candidate = directory / f"{stem}_{index}{suffix}"
            index += 1
            if index > 10000:                    # pragma: no cover - 目录出问题了
                raise
        else:
            return handle, candidate


def format_row(sample, channel_count: int) -> str:
    """一行 CSV。列宽来自**会话**的通道数，不是当前全局 schema——这是 N12 的根因。

    本帧没带的通道留空、不补 0：补 0 会让“这一帧没发这条通道”和“这条通道的值就是
    0”变得不可区分。这条语义从旧实现原样保留。
    """
    cells = [f"{sample.t_us * 1e-6:.6f}"]
    values = sample.values
    cells += [
        f"{values[index]:.6g}" if index in values else ""
        for index in range(channel_count)
    ]
    return ",".join(cells) + "\n"


class TelemetryRecorder:
    """录制服务。UI 只调 `start/stop/status`，收线程只调 `submit`。

    线程分工：
        收线程  -> submit()      入队，非阻塞，满了就计数丢弃
        写线程  -> _run()        唯一碰文件句柄的地方
        Tk 线程 -> status()      只读快照；drain() 是可选的有限期等待
    """

    def __init__(self, *, queue_limit: int = DEFAULT_QUEUE_LIMIT,
                 now: Callable[[], datetime] = datetime.now) -> None:
        self._queue: "queue.Queue[_Command | None]" = queue.Queue(maxsize=queue_limit)
        self._now = now
        self._lock = threading.Lock()
        self._idle = threading.Event()
        self._idle.set()
        self._session: _Session | None = None
        self._status = RecordStatus()
        self._next_ident = 1
        self._thread: threading.Thread | None = None
        self._closed = False

    # ---------------------------------------------------------------- 生命周期

    def _ensure_thread(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._run, name="telemetry-recorder", daemon=True
        )
        self._thread.start()

    def start(self, directory: Path, schema: RecordSchema, *,
              generation: int | None = None, stem: str | None = None) -> RecordStatus:
        """开一个新会话。已有会话时先按用户意图结束它。"""
        if self._closed:
            raise RuntimeError("recorder is closed")
        if self.status().active:
            self.stop(REASON_USER)
        stamp = stem or f"telem_{self._now().strftime('%Y%m%d_%H%M%S')}"
        handle = path = None
        try:
            handle, path = unique_record_path(Path(directory), stamp)
            # 表头随文件一起冻结：没有指纹的话，事后没人能确定这份 CSV 是哪张通道表
            # 下录的，列名就成了无法验证的说法。
            handle.write(f"# schema_hash={schema.schema_hash:08X}\n")
            handle.write("t_s," + ",".join(schema.channel_names) + "\n")
            handle.flush()
        except OSError as exc:
            # 目录没了、只读卷、开头就写不下去：都必须变成一个明确的失败态，而不是
            # 一个"录制中"的假象，也不是从按钮回调里冒出去的裸异常。
            if handle is not None:
                try:
                    handle.close()
                except Exception:                # noqa: BLE001 - 已经坏了
                    pass
            with self._lock:
                self._session = None
                self._status = RecordStatus(
                    state=STATE_FAILED, path=path,
                    schema_hash=schema.schema_hash, generation=generation,
                    end_reason=REASON_WRITE_ERROR, error=f"{type(exc).__name__}: {exc}",
                )
                return self._status
        with self._lock:
            session = _Session(
                ident=self._next_ident, path=path, handle=handle,
                schema=schema, generation=generation,
            )
            self._next_ident += 1
            self._session = session
            self._status = RecordStatus(
                state=STATE_RECORDING, path=path, schema_hash=schema.schema_hash,
                generation=generation,
            )
        self._ensure_thread()
        return self.status()

    def stop(self, reason: str = REASON_USER) -> RecordStatus:
        """请求结束当前会话，并在有限时间内等写线程把队列写完。"""
        with self._lock:
            session = self._session
            if session is None:
                return self._status
            self._session = None
        self._enqueue(_Command("finish", session, reason=reason), force=True)
        self.drain()
        return self.status()

    def close(self) -> RecordStatus:
        """面板退出。必须把在录的会话收尾，否则最后一段数据留在队列里没人写。"""
        status = self.stop(REASON_SHUTDOWN) if self.status().active else self.status()
        self._closed = True
        thread = self._thread
        if thread is not None and thread.is_alive():
            self._queue.put(None)
            thread.join(timeout=DEFAULT_DRAIN_TIMEOUT_S)
        self._thread = None
        return status

    # ---------------------------------------------------------------- 提交

    def submit(self, sample) -> bool:
        """收线程调用。**永不阻塞**：队列满就丢最新的一行并计数。"""
        with self._lock:
            session = self._session
        if session is None:
            return False
        return self._enqueue(_Command("row", session, sample=sample))

    def _enqueue(self, command: _Command, *, force: bool = False) -> bool:
        self._idle.clear()
        try:
            if force:
                self._queue.put(command)
            else:
                self._queue.put_nowait(command)
        except queue.Full:
            with self._lock:
                command.session.rows_dropped += 1
                if self._status.path == command.session.path:
                    self._status = replace(
                        self._status, rows_dropped=command.session.rows_dropped
                    )
            return False
        return True

    # ---------------------------------------------------------------- 状态

    def status(self) -> RecordStatus:
        with self._lock:
            return self._status

    def queued(self) -> int:
        return self._queue.qsize()

    def drain(self, timeout: float = DEFAULT_DRAIN_TIMEOUT_S) -> bool:
        """等写线程把当前队列处理完。超时返回 False，**不会**无限期挂住 UI。"""
        if self._queue.empty() and self._idle.is_set():
            return True                          # 没在录也没积压，不为此起一条线程
        self._ensure_thread()
        return self._idle.wait(timeout)

    # ---------------------------------------------------------------- 外部事件

    def note_schema(self, schema: RecordSchema) -> RecordStatus:
        """通道表换了。旧文件的表头解释不了新宽度的行，所以结束会话并写明原因。"""
        current = self.status()
        if current.active and current.schema_hash != schema.schema_hash:
            return self.stop(REASON_SCHEMA_CHANGED)
        return current

    def note_generation(self, generation: int | None) -> RecordStatus:
        """连接代次变了：这已经是另一台/另一次飞控，不能续到同一个文件里。"""
        current = self.status()
        if current.active and current.generation is not None \
                and generation is not None and generation != current.generation:
            return self.stop(REASON_LINK_CHANGED)
        return current

    # ---------------------------------------------------------------- 写线程

    def _run(self) -> None:
        while True:
            try:
                command = self._queue.get(timeout=0.2)
            except queue.Empty:
                self._idle.set()
                if self._closed and self._session is None:
                    return
                continue
            if command is None:
                self._idle.set()
                return
            try:
                self._apply(command)
            finally:
                self._queue.task_done()
                if self._queue.empty():
                    self._idle.set()

    def _apply(self, command: _Command) -> None:
        session = command.session
        if session.finished:
            return                               # 会话已收尾，迟到的行不再写
        if command.kind == "row":
            try:
                session.handle.write(
                    format_row(command.sample, session.schema.channel_count)
                )
            except OSError as exc:
                self._fail(session, exc)
            else:
                session.rows_written += 1
                self._publish(session)
            return
        self._finish(session, command.reason or REASON_USER)

    def _trailer(self, session: _Session, reason: str) -> str:
        return (
            f"# end reason={reason} rows={session.rows_written} "
            f"dropped={session.rows_dropped}\n"
        )

    def _finish(self, session: _Session, reason: str) -> None:
        session.finished = True
        session.end_reason = reason
        try:
            session.handle.write(self._trailer(session, reason))
            session.handle.close()
        except OSError as exc:
            self._fail(session, exc, closing=True)
            return
        with self._lock:
            self._status = RecordStatus(
                state=STATE_FINISHED, path=session.path,
                rows_written=session.rows_written, rows_dropped=session.rows_dropped,
                schema_hash=session.schema.schema_hash, generation=session.generation,
                end_reason=reason,
            )

    def _fail(self, session: _Session, exc: BaseException, *, closing: bool = False) -> None:
        """一次错误只产生一个失败态：句柄关掉，会话作废，部分文件留给用户。"""
        if session.finished and closing is False:
            return
        session.finished = True
        session.end_reason = REASON_WRITE_ERROR
        session.error = f"{type(exc).__name__}: {exc}"
        try:
            session.handle.close()
        except Exception:                        # noqa: BLE001 - 已经坏了，别再抛
            pass
        with self._lock:
            if self._session is session:
                self._session = None
            self._status = RecordStatus(
                state=STATE_FAILED, path=session.path,
                rows_written=session.rows_written, rows_dropped=session.rows_dropped,
                schema_hash=session.schema.schema_hash, generation=session.generation,
                end_reason=REASON_WRITE_ERROR, error=session.error,
            )

    def _publish(self, session: _Session) -> None:
        with self._lock:
            if self._status.path == session.path and self._status.state == STATE_RECORDING:
                self._status = replace(
                    self._status,
                    rows_written=session.rows_written,
                    rows_dropped=session.rows_dropped,
                )


def read_record_rows(path: Path) -> list[Sequence[str]]:
    """读回一份录制文件的数据行（跳过 `#` 说明行与表头）。测试与事后核对共用。"""
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    rows: list[Sequence[str]] = []
    header_seen = False
    for line in lines:
        if line.startswith("#"):
            continue
        if not header_seen:
            header_seen = True
            continue
        if line:
            rows.append(line.split(","))
    return rows


def record_header(path: Path) -> list[str]:
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    header = [line for line in lines if line.startswith("#")]
    return header


__all__ = [
    "DEFAULT_DRAIN_TIMEOUT_S",
    "DEFAULT_QUEUE_LIMIT",
    "REASON_LINK_CHANGED",
    "REASON_SCHEMA_CHANGED",
    "REASON_SHUTDOWN",
    "REASON_USER",
    "REASON_WRITE_ERROR",
    "STATE_FAILED",
    "STATE_FINISHED",
    "STATE_IDLE",
    "STATE_RECORDING",
    "RecordSchema",
    "RecordStatus",
    "TelemetryRecorder",
    "format_row",
    "read_record_rows",
    "record_header",
    "unique_record_path",
]
