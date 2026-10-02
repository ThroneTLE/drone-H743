"""本机 AI 共享接口：让 AI 和作者共用同一个地面站连接，不必关掉地面站释放串口。

结构
----
* `AiBridge`（本文件）：环形行缓冲、armed 记录、DFU 一次性许可、命令处理、HTTP 服务。
* `ai_bridge_policy`：命令白名单/永久拒绝表（纯函数）。
* `ai_bridge_ui`：连接区下面的一行小界面（状态标签、开关、DFU 一次性许可按钮）。

线程模型
--------
HTTP 在后台线程里跑；Tk 和传输层的发送（`_send` 会动 Tk 变量）只能在 Tk 线程做。
所以 HTTP 线程把“发送/写日志”投进 `_jobs` 队列，由 Tk 线程上的 `pump()`
（`panel.after` 轮询）取出执行，并用 Event 把结果交还给等待中的 HTTP 线程。
收行走 `board_line_hooks` 旁路：只读一份拷贝进缓冲区，不改变原有处理。
"""

from __future__ import annotations

import hmac
import json
import os
import queue
import re
import secrets
import tempfile
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import link_failover
from .ai_bridge_policy import DENY, DFU, classify

HOST = "127.0.0.1"
DEFAULT_PORT = 8765
PORT_TRIES = 20
BUFFER_LINES = 2000
ARMED_FRESH_S = 2.0            # DFU：armed=0 回报必须在这么久以内
DFU_PROBE_LINE = "SYSID THR?"  # 没有新鲜回报时，桥自己问一次（只读查询）
DFU_PROBE_WAIT_S = 1.5
RECONNECT_TIMEOUT_S = 180.0    # DFU 后等飞控串口回来的上限（擦写+校验约 10~20 s，留足余量）
RECONNECT_POLL_MS = 1000
JOB_TIMEOUT_S = 3.0
MAX_WAIT_MS = 10000
MAX_BODY = 4096
ARMED_RE = re.compile(r"\barmed=([01])\b")


def info_path() -> str:
    return os.path.join(tempfile.gettempdir(), "drone_panel_ai_bridge.json")


def note_sysid_sent(panel, line: str) -> None:
    """AI 接口直接走传输层发送，绕过系统辨识页的 send；要报给它记账。

    页面按「发出一条会回整份状态报告的命令 = 将来会多一份报告」来区分哪份报告是自己事务的回显
    （`workflow.note_sent`）。AI 发的 `SYSID?`、`SYSID MODE`、`SYSID THR?`、`SYSID XY` 等的回复如果没记账，
    事务进行中会被页面当成自己那条命令的回显：要么提前放行下一条，要么拿旧值核对出「配置回读不符」。
    """
    note = getattr(getattr(getattr(panel, "sysid_page", None), "workflow", None), "note_sent", None)
    if note is not None:
        try:
            note(line.strip())
        except Exception:                # 记账出错不能挡住 AI 的命令
            pass


class AiBridge:
    def __init__(self, panel, *, info_file: str | None = None, clock=time.monotonic,
                 default_port: int = DEFAULT_PORT):
        self.panel = panel
        self.clock = clock
        self.info_file = info_file or info_path()
        self.default_port = default_port
        self.token = secrets.token_hex(16)
        self.port = 0
        self.enabled = True
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._lines: deque = deque(maxlen=BUFFER_LINES)
        self._seq = 0
        self._armed: int | None = None
        self._armed_at = 0.0
        self._dfu_ok = False
        self._jobs: queue.Queue = queue.Queue()
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self.on_state_change = None      # Tk 线程回调，用来刷新界面标签

    # ---- 收行旁路（Tk 线程，由 board_line_hooks 调用）-------------------
    def on_board_line(self, line: str) -> None:
        text = str(line)
        with self._cond:
            self._seq += 1
            self._lines.append({"seq": self._seq, "t": time.time(), "line": text})
            found = None if text.startswith(("[上位机]", "> ", "[AI]")) else ARMED_RE.search(text)
            if found:
                self._armed = int(found.group(1))
                self._armed_at = self.clock()
            self._cond.notify_all()

    # ---- DFU 一次性许可（Tk 线程，按钮调用）-----------------------------
    def grant_dfu_once(self) -> None:
        with self._lock:
            self._dfu_ok = True
        self._changed()

    def revoke_dfu(self) -> None:
        with self._lock:
            self._dfu_ok = False
        self._changed()

    @property
    def dfu_granted(self) -> bool:
        with self._lock:
            return self._dfu_ok

    def set_enabled(self, value: bool) -> None:
        self.enabled = bool(value)
        if not self.enabled:
            self.revoke_dfu()
        self._changed()

    def _changed(self) -> None:
        cb = self.on_state_change
        if cb is not None:
            try:
                cb()
            except Exception:
                pass

    # ---- 查询 -----------------------------------------------------------
    def status(self) -> dict:
        panel = self.panel
        try:
            connected = bool(panel._transport_connected())
        except Exception:
            connected = False
        label = ""
        port = ""
        try:
            label = panel._transport_label()
            if label == "串口":
                port = str(getattr(panel.serial_transport, "active_port", "") or "")
        except Exception:
            pass
        with self._lock:
            armed = self._armed
            age = None if armed is None else round(self.clock() - self._armed_at, 2)
            seq, dfu = self._seq, self._dfu_ok
        return {"connected": connected, "transport": label, "port": port,
                "armed": armed, "armed_age_s": age, "seq": seq,
                "dfu_granted": dfu, "enabled": self.enabled, "pid": os.getpid()}

    def telem_latest(self, names: list) -> dict:
        """面板遥测流里这些通道的最新值（只读，走仪表盘同一个带锁环形缓冲）。

        2026-09-30：量化 XY 磁航向的偏航漂移要读 yaw，而姿态只在二进制遥测流里、文字行里没有。
        通道要在仪表盘当前订阅里（掩码），不在则值为 None。
        """
        out: dict = {}
        latest = getattr(self.panel, "_dashboard_latest", None)
        for name in names[:32]:
            try:
                value = latest(name) if latest is not None else None
            except Exception:
                value = None
            out[name] = None if value is None else float(value)
        return {"t": time.time(), "values": out}

    def lines_since(self, since: int = 0, limit: int = 200) -> dict:
        limit = max(1, min(int(limit), BUFFER_LINES))
        with self._lock:
            items = [dict(x) for x in self._lines if x["seq"] > since]
            seq = self._seq
        if len(items) > limit:
            items = items[-limit:]
        return {"seq": seq, "lines": items}

    def _armed_fresh(self):
        with self._lock:
            if self._armed is None or self.clock() - self._armed_at > ARMED_FRESH_S:
                return None
            return self._armed

    # ---- Tk 线程任务泵 ----------------------------------------------------
    def pump(self) -> None:
        """在 Tk 线程里执行 HTTP 线程投来的任务。"""
        while True:
            try:
                fn, box = self._jobs.get_nowait()
            except queue.Empty:
                return
            try:
                box["result"] = fn()
            except Exception as exc:          # 不让一条坏任务带走泵
                box["result"] = (False, f"面板内部错误: {exc}")
            box["done"].set()

    def _run_on_tk(self, fn):
        box = {"done": threading.Event(), "result": (False, "面板未响应")}
        self._jobs.put((fn, box))
        if not box["done"].wait(JOB_TIMEOUT_S):
            return False, "面板主线程忙，命令未发送"
        return box["result"]

    def _tk_log(self, text: str) -> None:
        def job():
            self.panel._append(text)
            return True, ""
        self._run_on_tk(job)

    def _tk_send(self, line: str, *, bypass_guard: bool = False):
        panel = self.panel

        def job():
            if not panel._transport_connected():
                return False, "地面站当前没有连接飞控"
            allowed = getattr(panel, "_validation_command_allowed", None)
            if allowed is not None and not bypass_guard and not allowed(line):
                return False, "地面站验收会话的只读门阻止了该命令"
            panel.last_cmd_var.set(f"最近命令: {line}")
            panel._append(f"[AI] > {line}")
            if not panel.transport.send_line(line):
                panel._append("[AI] 发送失败")
                return False, "传输层拒绝发送（未连接或正在导出日志）"
            note_sysid_sent(panel, line)
            return True, ""
        return self._run_on_tk(job)

    # ---- 命令处理 -----------------------------------------------------------
    def handle_cmd(self, line, wait_ms=800, expect=None) -> dict:
        if not self.enabled:
            return self._refuse("作者已在地面站关闭 AI 接口", line)
        verdict = classify(line)
        if verdict.kind == DENY:
            return self._refuse(verdict.reason, line)
        line = line.strip()
        pattern = None
        if expect:
            try:
                pattern = re.compile(expect)
            except re.error as exc:
                return self._refuse(f"expect 正则无效: {exc}", line, log=False)
        reconnect = ("", 0)
        if verdict.kind == DFU:
            reason = self._dfu_precheck()
            if reason:
                return self._refuse(reason, line)
            got, target = self._run_on_tk(self._reconnect_target)
            reconnect = target if got else ("", 0)
            self._run_on_tk(lambda: (True, link_failover.hold(self.panel)))  # 主动进 DFU：断开别切蓝牙
        try:
            wait_ms = max(0, min(int(wait_ms), MAX_WAIT_MS))
        except (TypeError, ValueError):
            wait_ms = 800
        with self._lock:
            start_seq = self._seq
        # BOOT 类命令平时被面板的固件升级门挡住；DFU 已经过本接口的双条件检查，
        # 所以只有它绕过验收会话那道门。
        ok, why = self._tk_send(line, bypass_guard=(verdict.kind == DFU))
        if verdict.kind == DFU and ok:
            self.revoke_dfu()            # 一次性：发出去就作废
            self._run_on_tk(lambda: (True, self._watch_reconnect(*reconnect)))
        if not ok:
            return self._refuse(why, line, log=False)
        lines, matched = self._collect(start_seq, wait_ms, pattern)
        return {"sent": True, "reason": "", "lines": lines, "matched": matched}

    # ---- DFU 之后自动重连（Tk 线程）--------------------------------------
    def _reconnect_target(self):
        """发 DFU 前记下要连回去的串口与波特率；不是串口连接就不重连。"""
        panel = self.panel
        port = getattr(panel.serial_transport, "active_port", "") or ""
        if panel.transport is not panel.serial_transport or not port:
            return True, ("", 0)
        try:
            baud = int(panel.serial_baud_var.get())
        except Exception:
            baud = 115200
        return True, (port, baud)

    def _watch_reconnect(self, port: str, baud: int) -> None:
        """面板自己的「固件升级」流程烧完会自动重连；经本接口进的 DFU 不走那条流程，
        以前烧完地面站就一直断着（2026-09-30 多次）。这里补上：等串口断开、原端口重新
        枚举后，复用面板的 `_firmware_auto_reconnect` 连回去；打不开就下一秒再试。"""
        if not port:
            return
        deadline = self.clock() + RECONNECT_TIMEOUT_S
        state = {"gone": False}

        def tick():
            panel = self.panel
            if panel._transport_connected():
                if not state["gone"] and self.clock() <= deadline:
                    panel.after(RECONNECT_POLL_MS, tick)   # 还没断开，继续等
                return                                       # 已有人连回去（或 DFU 根本没发生）
            state["gone"] = True
            if self.clock() > deadline:
                panel._append(f"[AI] DFU 后 {RECONNECT_TIMEOUT_S:.0f} s 内没等到 {port} 回来，请手动连接")
                return
            panel._refresh_serial_ports()
            present = any(str(device).casefold() == port.casefold()
                          for device in panel._serial_port_map.values())
            if present:
                panel.firmware_reconnect_baud = baud
                if panel._firmware_auto_reconnect(port):
                    panel._append(f"[AI] DFU 后已自动重连 {port}")
                    return
            panel.after(RECONNECT_POLL_MS, tick)

        self.panel.after(RECONNECT_POLL_MS, tick)

    def _refuse(self, reason: str, line, log: bool = True) -> dict:
        if log:
            self._tk_log(f"[AI] 已拒绝: {str(line)[:120]}（{reason}）")
        return {"sent": False, "reason": reason, "lines": [], "matched": None}

    def _dfu_precheck(self) -> str:
        # 2026-09-30 作者："你就别让我允许了改成全自动的"——不再要界面一次性许可，只留硬条件：
        # 最近 2 s 内飞控回报 armed=0（没有就先问一次），已解锁或问不到一律拒绝。
        if self._armed_fresh() is None:
            with self._lock:
                start = self._seq
            ok, why = self._tk_send(DFU_PROBE_LINE)
            if not ok:
                return f"无法确认 armed 状态（探测失败: {why}）"
            self._collect(start, int(DFU_PROBE_WAIT_S * 1000), ARMED_RE)
        armed = self._armed_fresh()
        if armed is None:
            return f"最近 {ARMED_FRESH_S:g} 秒内没有收到飞控的 armed 回报，拒绝进 DFU"
        if armed != 0:
            return "飞控回报 armed=1（已解锁），拒绝进 DFU"
        return ""

    def _collect(self, start_seq: int, wait_ms: int, pattern):
        deadline = self.clock() + wait_ms / 1000.0
        cursor = start_seq
        out: list = []
        matched = None if pattern is None else False
        with self._cond:
            while True:
                for item in [x for x in self._lines if x["seq"] > cursor]:
                    out.append(dict(item))
                    cursor = item["seq"]
                    # "[host] serial tx ..." 是面板对发出去那一行的回显，不是飞控的回复，不参与匹配。
                    if (pattern is not None and not item["line"].startswith("[host]")
                            and pattern.search(item["line"])):
                        return out, True
                remaining = deadline - self.clock()
                if remaining <= 0:
                    return out, matched
                self._cond.wait(min(remaining, 0.05))

    # ---- HTTP 服务 ----------------------------------------------------------
    def start(self) -> int:
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.0"

            def log_message(self, *args):
                pass

            def _send_json(self, code, payload):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _authorized(self) -> bool:
                host = (self.headers.get("Host") or "").split(":")[0].lower()
                token = self.headers.get("X-Token") or ""
                if host not in ("127.0.0.1", "localhost") or not hmac.compare_digest(
                        token.encode("utf-8"), bridge.token.encode("utf-8")):
                    self._send_json(403, {"error": "token 或 Host 不正确"})
                    return False
                return True

            def do_GET(self):
                if not self._authorized():
                    return
                url = urlparse(self.path)
                query = parse_qs(url.query)
                if url.path == "/status":
                    return self._send_json(200, bridge.status())
                if url.path == "/lines":
                    try:
                        since = int(query.get("since", ["0"])[0])
                        limit = int(query.get("limit", ["200"])[0])
                    except ValueError:
                        return self._send_json(400, {"error": "since/limit 必须是整数"})
                    return self._send_json(200, bridge.lines_since(since, limit))
                if url.path == "/telem":
                    names = [n for n in (query.get("names", [""])[0]).split(",") if n]
                    return self._send_json(200, bridge.telem_latest(names))
                self._send_json(404, {"error": "未知路径"})

            def do_POST(self):
                # 先读完请求体再回话：拒绝时留着没读的请求体就关连接，Windows 上客户端收到的是
                # "连接被中断"而不是 403，会被误报成连不上地面站（test_token_required 偶发）。
                try:
                    size = int(self.headers.get("Content-Length") or 0)
                except ValueError:
                    size = -1
                body = self.rfile.read(size) if 0 <= size <= MAX_BODY else None
                if not self._authorized():
                    return
                if urlparse(self.path).path != "/cmd":
                    return self._send_json(404, {"error": "未知路径"})
                try:
                    if body is None:
                        raise ValueError("请求体长度无效或过大")
                    data = json.loads(body.decode("utf-8"))
                    line = data["line"]
                    if not isinstance(line, str):
                        raise ValueError("line 必须是字符串")
                except (ValueError, KeyError, TypeError, UnicodeDecodeError) as exc:
                    return self._send_json(400, {"sent": False, "reason": f"请求无效: {exc}",
                                                 "lines": [], "matched": None})
                result = bridge.handle_cmd(line, data.get("wait_ms", 800), data.get("expect") or None)
                self._send_json(200, result)

        class _Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = False      # Windows 上 True 会允许多个进程同端口

        for offset in range(PORT_TRIES):
            try:
                self._server = _Server((HOST, self.default_port + offset), Handler)
                break
            except OSError:
                continue
        if self._server is None:
            raise OSError("AI 接口找不到空闲端口")
        self.port = self._server.server_address[1]
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        name="ai-bridge", daemon=True)
        self._thread.start()
        with open(self.info_file, "w", encoding="utf-8") as fh:
            json.dump({"port": self.port, "token": self.token, "pid": os.getpid()}, fh)
        return self.port

    def stop(self) -> None:
        server, self._server = self._server, None
        if server is not None:
            server.shutdown()
            server.server_close()
        try:
            with open(self.info_file, encoding="utf-8") as fh:
                mine = json.load(fh).get("pid") == os.getpid()
            if mine:
                os.remove(self.info_file)
        except (OSError, ValueError):
            pass


__all__ = ["AiBridge", "info_path"]
