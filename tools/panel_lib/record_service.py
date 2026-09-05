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

2026-09-04 软件审核（R1~R4）指出第一版只把**渲染回调**挪开了，别的路径还在阻塞，
而且会话缺少身份。第二版按四条重做：

**① Tk 线程一次 I/O 都不做。** `start()` 不再自己建文件、写表头；它只登记一个
会话并投递一条 `open` 命令，状态先进 `starting`，由写线程完成后翻成 `recording`。
`stop()` 只投递 `finish` 就返回（`stopping`），不再 drain。整条链路上唯一还会等的
地方是 `close()`——那是进程要退出了，不等就等于把最后一段数据扔掉，而且此时已经
没有界面可卡；这个等待有上限且会如实报告有没有等完。

**② 控制命令不受队列上限约束。** 用 deque + Condition 而不是 `queue.Queue`：
行受上限（溢出计数可见），`open`/`finish`/`shutdown` 无条件入队。第一版那个
`put(force=True)` 在队列满时会无限期挂住调用者，也就是挂住 Tk。

**③ 接受样本与结束栅栏在同一把锁里。** 第一版在锁内取 session、出锁后才入队，
`stop()` 可以插到中间，于是 `submit()` 返回 True 的样本排在 `finish` 后面被无声
丢弃，既没写进去也没计进 dropped。现在两件事原子完成：返回 True 就一定排在本会话
的 `finish` 之前。

**④ 会话带链路身份，且只更新自己的状态。** 样本在**接收线程的入口**就要带上
transport 对象与连接代次；对不上就地结束会话，不再等下一次 UI 轮询补救——等轮询
意味着新连接的第一帧已经写进旧文件了。终态发布按会话 ident 判断，旧会话收尾不再
覆盖新会话的状态（第一版超时重启后会出现状态指向旧文件、`_session` 指向新文件）。

队列是**有界**的：`tk-ui` 模式明确禁止用无限队列或静默丢数据来解决背压，所以溢出
既要有计数，也要写进文件尾的说明行里，让事后拿到 CSV 的人能看见。
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Callable, Sequence


# 40 Hz 下 20000 行约 8 分钟余量；写盘线程正常情况下根本追得上，这个上限只在
# 磁盘真的卡住时起作用，让“丢了多少”变成一个数字而不是一次内存膨胀。
DEFAULT_QUEUE_LIMIT = 20000
# 退出时最多等写线程多久。只有 `close()` 会用到它——那时进程要走了，不等就是丢数据。
DEFAULT_SHUTDOWN_TIMEOUT_S = 2.0

STATE_IDLE = "idle"
STATE_STARTING = "starting"
STATE_RECORDING = "recording"
STATE_STOPPING = "stopping"
STATE_FINISHED = "finished"
STATE_FAILED = "failed"

_BUSY_STATES = (STATE_STARTING, STATE_RECORDING, STATE_STOPPING)

REASON_USER = "user"
REASON_SCHEMA_CHANGED = "schema_changed"
REASON_LINK_CHANGED = "link_changed"
REASON_SHUTDOWN = "shutdown"
REASON_WRITE_ERROR = "write_error"

_REASON_TEXT = {
    REASON_USER: "用户停止",
    REASON_SCHEMA_CHANGED: "通道表变化",
    REASON_LINK_CHANGED: "连接已更换",
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


class LinkIdentity:
    """一次录制绑定的链路身份：**transport 对象本身** + 它当时的连接代次。

    光比整数代次不够：每个 transport 各有一个从 0 开始的计数器，串口换成 TCP 时
    两边的 `connection_generation` 完全可能都是 2，于是"换了一条链路"这件事在数字
    上看不出来。所以这里同时按对象 identity 比较，并持有强引用——只有持有引用，
    `is` 比较才不会因为对象被回收、地址被复用而给出错误的"相同"。
    """

    __slots__ = ("transport", "generation")

    def __init__(self, transport: object, generation: int | None) -> None:
        self.transport = transport
        self.generation = generation

    def matches(self, transport: object, generation: int | None) -> bool:
        return self.transport is transport and self.generation == generation

    def __repr__(self) -> str:                   # pragma: no cover - 诊断用
        return f"LinkIdentity({type(self.transport).__name__}, gen={self.generation})"


@dataclass(frozen=True)
class RecordStatus:
    """给 Tk 线程读的状态快照。读它只碰一把短锁，绝不碰磁盘。"""

    state: str = STATE_IDLE
    path: Path | None = None
    rows_written: int = 0
    rows_queued: int = 0
    rows_dropped: int = 0
    rows_rejected: int = 0
    schema_hash: int | None = None
    generation: int | None = None
    end_reason: str | None = None
    error: str | None = None

    @property
    def active(self) -> bool:
        """还在收样本。`starting` 也算——文件还没建好但会话已经登记。"""
        return self.state in (STATE_STARTING, STATE_RECORDING)

    @property
    def busy(self) -> bool:
        """会话尚未落地（含正在收尾）。按钮的"再点一次"要看它，不是看 active。"""
        return self.state in _BUSY_STATES

    def describe(self) -> str:
        """一行给用户看的话。失败和“结束了但不是你按的”必须能分辨。"""
        if self.state == STATE_IDLE:
            return "未录制"
        name = self.path.name if self.path is not None else "…"
        if self.state == STATE_STARTING:
            return "正在创建录制文件…"
        if self.state == STATE_RECORDING:
            suffix = f"（已丢 {self.rows_dropped} 行）" if self.rows_dropped else ""
            return f"{name}{suffix}"
        if self.state == STATE_STOPPING:
            return f"{name}：正在收尾…"
        if self.state == STATE_FAILED:
            return f"录制失败：{self.error}（部分文件 {name}）"
        reason = _REASON_TEXT.get(self.end_reason or "", self.end_reason or "")
        if self.end_reason in (None, REASON_USER):
            return f"已保存 {name}（{self.rows_written} 行）"
        return f"已结束 {name}：{reason}（{self.rows_written} 行）"


@dataclass
class _Session:
    """一次录制。文件、列宽、指纹、链路身份在这里绑定，之后谁也改不了。"""

    ident: int
    directory: Path
    stem: str
    schema: RecordSchema
    link: LinkIdentity | None
    generation: int | None
    path: Path | None = None
    handle: object = None
    rows_written: int = 0
    rows_dropped: int = 0
    rows_rejected: int = 0
    finished: bool = False
    end_reason: str | None = None
    error: str | None = None


@dataclass
class _Command:
    kind: str                       # "open" | "row" | "finish" | "shutdown"
    session: _Session | None = None
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
    """录制服务。

    线程分工（这是整个模块的要点，改之前先读）：

        Tk 线程  -> start / stop / note_* / status     只投递请求与读快照，零 I/O
        收线程   -> submit(sample, transport, gen)     入队，非阻塞，带链路身份
        写线程   -> _run()                             唯一碰文件句柄的地方
    """

    def __init__(self, *, queue_limit: int = DEFAULT_QUEUE_LIMIT,
                 now: Callable[[], datetime] = datetime.now) -> None:
        self._cv = threading.Condition()
        self._pending: deque[_Command] = deque()
        self._queued_rows = 0
        self._queue_limit = max(1, int(queue_limit))
        self._now = now
        self._idle = threading.Event()
        self._idle.set()
        self._session: _Session | None = None
        self._status = RecordStatus()
        self._status_ident: int | None = None
        self._history: list[RecordStatus] = []
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
              generation: int | None = None, stem: str | None = None,
              link: LinkIdentity | None = None) -> RecordStatus:
        """登记一个新会话并**立刻返回**。

        文件创建与表头写入由写线程完成，因此这个调用不碰磁盘——它可能在 Tk 线程上
        被按钮直接调用。返回的状态是 `starting`。
        """
        if self._closed:
            raise RuntimeError("recorder is closed")
        stamp = stem or f"telem_{self._now().strftime('%Y%m%d_%H%M%S')}"
        with self._cv:
            if self._session is not None:
                self._enqueue_locked(_Command("finish", self._session, reason=REASON_USER))
                self._session = None
            session = _Session(
                ident=self._next_ident, directory=Path(directory), stem=stamp,
                schema=schema, link=link, generation=generation,
            )
            self._next_ident += 1
            self._session = session
            self._status = RecordStatus(
                state=STATE_STARTING, schema_hash=schema.schema_hash,
                generation=generation,
            )
            self._status_ident = session.ident
            self._enqueue_locked(_Command("open", session))
            status = self._status
        self._ensure_thread()
        return status

    def stop(self, reason: str = REASON_USER) -> RecordStatus:
        """请求结束当前会话并**立刻返回**（`stopping`）。不等写线程。"""
        with self._cv:
            session = self._session
            if session is None:
                return self._status
            self._session = None
            self._enqueue_locked(_Command("finish", session, reason=reason))
            if self._status_ident == session.ident:
                self._status = replace(self._status, state=STATE_STOPPING,
                                       end_reason=reason)
            status = self._status
        self._ensure_thread()
        return status

    def close(self, timeout: float = DEFAULT_SHUTDOWN_TIMEOUT_S) -> RecordStatus:
        """面板退出。

        **这是整个服务里唯一会等的地方**，理由很直接：进程马上就没了，不等就等于把
        最后一段已接受的数据扔掉，而此时也已经没有界面可以被卡住。等待有上限，等没
        等完可以从返回状态看出来（还停在 `stopping` 就是没等完）。
        """
        if self.status().busy:
            self.stop(REASON_SHUTDOWN)
        with self._cv:
            self._closed = True
            self._enqueue_locked(_Command("shutdown"))
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        self._thread = None
        return self.status()

    # ---------------------------------------------------------------- 提交

    def submit(self, sample, *, transport: object = None,
               generation: int | None = None) -> bool:
        """收线程调用。**永不阻塞**，且接受与结束在同一把锁里完成。

        返回 True 表示这一行一定排在本会话的 `finish` 之前；返回 False 一定伴随一个
        可解释的计数（拒收 / 丢弃）或"根本没在录"。
        """
        with self._cv:
            session = self._session
            if session is None:
                return False
            if session.link is not None and not session.link.matches(transport, generation):
                # 换了链路。就地结束，不等下一次 UI 轮询——等轮询意味着新连接的第一帧
                # 已经写进旧文件了（审核 R2）。
                session.rows_rejected += 1
                self._session = None
                self._enqueue_locked(
                    _Command("finish", session, reason=REASON_LINK_CHANGED)
                )
                if self._status_ident == session.ident:
                    self._status = replace(
                        self._status, state=STATE_STOPPING,
                        end_reason=REASON_LINK_CHANGED,
                        rows_rejected=session.rows_rejected,
                    )
                return False
            if self._queued_rows >= self._queue_limit:
                session.rows_dropped += 1
                if self._status_ident == session.ident:
                    self._status = replace(self._status,
                                           rows_dropped=session.rows_dropped)
                return False
            self._enqueue_locked(_Command("row", session, sample=sample))
            return True

    def _enqueue_locked(self, command: _Command) -> None:
        """调用者必须持有 `_cv`。

        控制命令（open/finish/shutdown）不受行上限约束：第一版把它们和数据行放进同
        一个有界 `queue.Queue`，队列满时 `put()` 会把 Tk 线程无限期挂住。
        """
        self._pending.append(command)
        if command.kind == "row":
            self._queued_rows += 1
        self._idle.clear()
        self._cv.notify()

    # ---------------------------------------------------------------- 状态

    def status(self) -> RecordStatus:
        with self._cv:
            return replace(self._status, rows_queued=self._queued_rows)

    def history(self) -> list[RecordStatus]:
        """已落地会话的终态，按结束顺序。超时后旧会话的结果从这里查，不靠全局状态。"""
        with self._cv:
            return list(self._history)

    def queued(self) -> int:
        with self._cv:
            return self._queued_rows

    def wait_idle(self, timeout: float = 5.0) -> bool:
        """等写线程把当前队列处理完。

        **UI 不许调它**——这是给测试和收尾用的确定性栅栏。留一个显式的名字，比让
        界面代码顺手调一个叫 `flush` 的东西安全。
        """
        with self._cv:
            if not self._pending and self._idle.is_set():
                return True
        self._ensure_thread()
        return self._idle.wait(timeout)

    # ---------------------------------------------------------------- 外部事件

    def note_schema(self, schema: RecordSchema) -> RecordStatus:
        """通道表换了。旧文件的表头解释不了新宽度的行，所以结束会话并写明原因。"""
        with self._cv:
            session = self._session
            changed = session is not None and session.schema.schema_hash != schema.schema_hash
        if changed:
            return self.stop(REASON_SCHEMA_CHANGED)
        return self.status()

    def note_generation(self, generation: int | None) -> RecordStatus:
        """连接代次变了。

        这是**兜底**，不是主路径：真正的把关在 `submit()` 的链路身份核对里，因为帧
        走接收线程，比任何 UI 轮询都早到。
        """
        with self._cv:
            session = self._session
            changed = (
                session is not None
                and session.generation is not None
                and generation is not None
                and generation != session.generation
            )
        if changed:
            return self.stop(REASON_LINK_CHANGED)
        return self.status()

    # ---------------------------------------------------------------- 写线程

    def _run(self) -> None:
        while True:
            with self._cv:
                while not self._pending:
                    self._idle.set()
                    if self._closed:
                        return
                    self._cv.wait(0.2)
                command = self._pending.popleft()
                if command.kind == "row":
                    self._queued_rows -= 1
            if command.kind == "shutdown":
                with self._cv:
                    self._idle.set()
                return
            # I/O 在锁外做：写盘可能很慢，而 Tk 线程随时会来读状态。
            self._apply(command)
            with self._cv:
                if not self._pending:
                    self._idle.set()

    def _apply(self, command: _Command) -> None:
        session = command.session
        if session is None:                      # pragma: no cover - 只有 shutdown
            return
        if command.kind == "open":
            self._open(session)
            return
        if session.finished:
            return                               # 会话已收尾，迟到的行不再写
        if command.kind == "row":
            if session.handle is None:           # open 失败过，后面的行直接不写
                return
            try:
                session.handle.write(
                    format_row(command.sample, session.schema.channel_count)
                )
            except OSError as exc:
                self._fail(session, exc)
            else:
                session.rows_written += 1
                self._publish_progress(session)
            return
        self._finish(session, command.reason or REASON_USER)

    def _open(self, session: _Session) -> None:
        try:
            handle, path = unique_record_path(session.directory, session.stem)
            # 表头随文件一起冻结：没有指纹的话，事后没人能确定这份 CSV 是哪张通道表
            # 下录的，列名就成了无法验证的说法。
            handle.write(f"# schema_hash={session.schema.schema_hash:08X}\n")
            handle.write("t_s," + ",".join(session.schema.channel_names) + "\n")
            handle.flush()
        except OSError as exc:
            session.path = getattr(exc, "filename", None) and Path(exc.filename)
            self._fail(session, exc)
            return
        session.handle = handle
        session.path = path
        with self._cv:
            if self._status_ident == session.ident:
                self._status = replace(self._status, state=STATE_RECORDING, path=path)
            elif self._session is session:       # pragma: no cover - 防御
                self._status_ident = session.ident

    def _trailer(self, session: _Session, reason: str) -> str:
        return (
            f"# end reason={reason} rows={session.rows_written} "
            f"dropped={session.rows_dropped} rejected={session.rows_rejected}\n"
        )

    def _finish(self, session: _Session, reason: str) -> None:
        session.finished = True
        session.end_reason = reason
        if session.handle is not None:
            try:
                session.handle.write(self._trailer(session, reason))
                session.handle.close()
            except OSError as exc:
                self._fail(session, exc, closing=True)
                return
        self._publish_terminal(session, RecordStatus(
            state=STATE_FINISHED, path=session.path,
            rows_written=session.rows_written, rows_dropped=session.rows_dropped,
            rows_rejected=session.rows_rejected,
            schema_hash=session.schema.schema_hash, generation=session.generation,
            end_reason=reason,
        ))

    def _fail(self, session: _Session, exc: BaseException, *, closing: bool = False) -> None:
        """一次错误只产生一个失败态：句柄关掉，会话作废，部分文件留给用户。"""
        if session.finished and not closing:
            return
        session.finished = True
        session.end_reason = REASON_WRITE_ERROR
        session.error = f"{type(exc).__name__}: {exc}"
        if session.handle is not None:
            try:
                session.handle.close()
            except Exception:                    # noqa: BLE001 - 已经坏了，别再抛
                pass
        with self._cv:
            if self._session is session:
                self._session = None
        self._publish_terminal(session, RecordStatus(
            state=STATE_FAILED, path=session.path,
            rows_written=session.rows_written, rows_dropped=session.rows_dropped,
            rows_rejected=session.rows_rejected,
            schema_hash=session.schema.schema_hash, generation=session.generation,
            end_reason=REASON_WRITE_ERROR, error=session.error,
        ))

    def _publish_terminal(self, session: _Session, status: RecordStatus) -> None:
        """终态只更新**它自己那个会话**的状态。

        第一版无条件写 `_status`：stop 超时后用户又开了新会话，旧 finish 完成时会把
        界面状态改回旧文件的 finished，而 `_session` 指向新文件（审核 R4）。终态一律
        进 `history()`，随时可查。
        """
        with self._cv:
            self._history.append(status)
            if self._status_ident == session.ident:
                self._status = status

    def _publish_progress(self, session: _Session) -> None:
        with self._cv:
            if self._status_ident == session.ident and self._status.state == STATE_RECORDING:
                self._status = replace(
                    self._status,
                    rows_written=session.rows_written,
                    rows_dropped=session.rows_dropped,
                    rows_rejected=session.rows_rejected,
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
    return [line for line in lines if line.startswith("#")]


__all__ = [
    "DEFAULT_QUEUE_LIMIT",
    "DEFAULT_SHUTDOWN_TIMEOUT_S",
    "REASON_LINK_CHANGED",
    "REASON_SCHEMA_CHANGED",
    "REASON_SHUTDOWN",
    "REASON_USER",
    "REASON_WRITE_ERROR",
    "STATE_FAILED",
    "STATE_FINISHED",
    "STATE_IDLE",
    "STATE_RECORDING",
    "STATE_STARTING",
    "STATE_STOPPING",
    "LinkIdentity",
    "RecordSchema",
    "RecordStatus",
    "TelemetryRecorder",
    "format_row",
    "read_record_rows",
    "record_header",
    "unique_record_path",
]
