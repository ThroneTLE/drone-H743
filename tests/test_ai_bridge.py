"""AI 共享接口：假面板 + 真 HTTP（只绑回环），不碰串口、不启动真地面站。"""

import json
import os
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from tools.panel_lib import ai_bridge as ab
from tools.panel_lib import ai_bridge_policy as policy
from tools.panel_lib.board_line_hooks import dispatch_board_line, register_board_line_hook

ROOT = Path(__file__).resolve().parents[1]

# 改动前 tools/drone_tcp_panel.py 的体量（本任务不许让它变长）。
PANEL_MAX_LINES = 3858
PANEL_MAX_BYTES = 182378


class Var:
    def __init__(self):
        self.value = ""

    def set(self, v):
        self.value = v


class FakeTransport:
    def __init__(self):
        self.sent = []
        self.ok = True
        self.is_connected = True

    def send_line(self, line):
        self.sent.append(line)
        return self.ok


class FakePanel:
    """只有桥会碰的那几个成员；`after` 用后台线程模拟 Tk 线程泵。"""

    def __init__(self):
        self.transport = FakeTransport()
        self.serial_transport = type("S", (), {"active_port": "COM22"})()
        self.last_cmd_var = Var()
        self.log = []
        self.connected = True

    def _transport_connected(self):
        return self.connected

    def _transport_label(self):
        return "串口"

    def _append(self, line):
        self.log.append(line)


@pytest.fixture
def env(tmp_path):
    panel = FakePanel()
    bridge = ab.AiBridge(panel, info_file=str(tmp_path / "info.json"), default_port=18765)
    register_board_line_hook(panel, bridge.on_board_line)
    port = bridge.start()
    stop = threading.Event()

    def tk_thread():
        while not stop.is_set():
            bridge.pump()
            time.sleep(0.005)

    thread = threading.Thread(target=tk_thread, daemon=True)
    thread.start()
    yield panel, bridge, port
    stop.set()
    thread.join(2)
    bridge.stop()


def call(bridge, port, method, path, body=None, token="auto", host=None):
    headers = {"X-Token": bridge.token if token == "auto" else token}
    if host:
        headers["Host"] = host
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=data,
                                 method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode())


def feed(panel, line):
    dispatch_board_line(panel, line)


# ------------------------------------------------------------------ 服务端
def test_binds_loopback_only_and_writes_info_file(env):
    panel, bridge, port = env
    assert bridge._server.server_address[0] == "127.0.0.1"
    info = json.loads(Path(bridge.info_file).read_text(encoding="utf-8"))
    assert info == {"port": port, "token": bridge.token, "pid": os.getpid()}
    assert len(bridge.token) >= 32


def test_port_falls_forward_when_busy(tmp_path):
    a = ab.AiBridge(FakePanel(), info_file=str(tmp_path / "a.json"), default_port=18800)
    b = ab.AiBridge(FakePanel(), info_file=str(tmp_path / "b.json"), default_port=18800)
    try:
        assert a.start() == 18800
        assert b.start() == 18801
    finally:
        a.stop()
        b.stop()


def test_token_required(env):
    panel, bridge, port = env
    assert call(bridge, port, "GET", "/status", token="bad")[0] == 403
    assert call(bridge, port, "GET", "/status", token="")[0] == 403
    assert call(bridge, port, "POST", "/cmd", {"line": "PARAM?"}, token="bad")[0] == 403
    assert panel.transport.sent == []
    assert call(bridge, port, "GET", "/status", host="evil.example")[0] == 403
    assert call(bridge, port, "GET", "/status")[0] == 200


def test_status_reports_connection_and_armed(env):
    panel, bridge, port = env
    feed(panel, "SYSID THR auto=0 phase=idle armed=0 thr_low=1")
    code, data = call(bridge, port, "GET", "/status")
    assert code == 200
    assert data["connected"] is True and data["transport"] == "串口" and data["port"] == "COM22"
    assert data["armed"] == 0 and data["seq"] == 1
    feed(panel, "SYSID THR auto=0 armed=1")
    assert call(bridge, port, "GET", "/status")[1]["armed"] == 1


# ------------------------------------------------------------------ 缓冲
def test_ring_buffer_and_since_semantics(env):
    panel, bridge, port = env
    for i in range(ab.BUFFER_LINES + 50):
        feed(panel, f"L{i}")
    data = bridge.lines_since(0, 5000)
    assert len(data["lines"]) == ab.BUFFER_LINES
    assert data["lines"][0]["line"] == "L50" and data["seq"] == ab.BUFFER_LINES + 50
    tail = call(bridge, port, "GET", f"/lines?since={data['seq'] - 3}&limit=10")[1]
    assert [x["line"] for x in tail["lines"]] == [f"L{ab.BUFFER_LINES + i}" for i in (47, 48, 49)]
    assert all("t" in x and "seq" in x for x in tail["lines"])
    limited = call(bridge, port, "GET", "/lines?since=0&limit=2")[1]
    assert [x["seq"] for x in limited["lines"]] == [data["seq"] - 1, data["seq"]]


def test_hook_tap_does_not_change_original_handling(env):
    panel, bridge, port = env
    seen = []
    register_board_line_hook(panel, seen.append)

    def boom(line):
        raise RuntimeError("别的钩子坏了")

    register_board_line_hook(panel, boom)
    feed(panel, "OK param")
    assert seen == ["OK param"]                      # 别的观察者照常收到
    assert bridge.lines_since(0)["lines"][0]["line"] == "OK param"
    # 桥不回写、不改变行内容
    assert panel.log == [] and panel.transport.sent == []


# ------------------------------------------------------------------ /cmd
def test_cmd_collects_until_expect_match(env):
    panel, bridge, port = env

    def reply():
        time.sleep(0.1)
        feed(panel, "noise")
        feed(panel, "SYSID THR auto=0 armed=0")
        feed(panel, "after-match")

    threading.Thread(target=reply, daemon=True).start()
    t0 = time.monotonic()
    code, data = call(bridge, port, "POST", "/cmd",
                      {"line": "SYSID THR?", "wait_ms": 3000, "expect": r"SYSID THR .*armed=0"})
    assert code == 200 and data["sent"] is True and data["matched"] is True
    assert [x["line"] for x in data["lines"]] == ["noise", "SYSID THR auto=0 armed=0"]
    assert time.monotonic() - t0 < 2.0
    assert panel.transport.sent == ["SYSID THR?"]
    assert "[AI] > SYSID THR?" in panel.log


def test_cmd_expect_skips_the_host_tx_echo(env):
    """面板会把发出去的那一行回显成 "[host] serial tx ... data=SYSID THR?"：它不是飞控回复，不许提前匹配
    （2026-09-30 实机首次用时 expect=SYSID THR 撞上了这条回显）。"""
    panel, bridge, port = env

    def reply():
        time.sleep(0.1)
        feed(panel, "[host] serial tx bytes=12 data=SYSID THR?\\r\\n")
        feed(panel, "SYSID THR auto=1 armed=0")

    threading.Thread(target=reply, daemon=True).start()
    code, data = call(bridge, port, "POST", "/cmd",
                      {"line": "SYSID THR?", "wait_ms": 3000, "expect": r"SYSID THR"})
    assert code == 200 and data["matched"] is True
    assert data["lines"][-1]["line"] == "SYSID THR auto=1 armed=0"


def test_cmd_times_out_without_match_and_without_expect(env):
    panel, bridge, port = env
    t0 = time.monotonic()
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM?", "wait_ms": 300, "expect": "NEVER"})[1]
    assert data["sent"] is True and data["matched"] is False and data["lines"] == []
    assert 0.25 < time.monotonic() - t0 < 2.0
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM?", "wait_ms": 50})[1]
    assert data["matched"] is None


def test_cmd_bad_requests(env):
    panel, bridge, port = env
    assert call(bridge, port, "POST", "/cmd", {"nope": 1})[0] == 400
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM?", "expect": "("})[1]
    assert data["sent"] is False and "正则" in data["reason"]
    assert panel.transport.sent == []


def test_cmd_reports_disconnected_and_transport_failure(env):
    panel, bridge, port = env
    panel.connected = False
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM?"})[1]
    assert data["sent"] is False and "没有连接" in data["reason"]
    panel.connected = True
    panel.transport.ok = False
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM?"})[1]
    assert data["sent"] is False and "传输层" in data["reason"]


def test_disabled_switch_refuses_everything(env):
    panel, bridge, port = env
    bridge.set_enabled(False)
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM?"})[1]
    assert data["sent"] is False and "关闭" in data["reason"]
    assert panel.transport.sent == []
    bridge.set_enabled(True)
    assert call(bridge, port, "POST", "/cmd", {"line": "PARAM?", "wait_ms": 0})[1]["sent"] is True


def test_ai_log_prefix_for_sent_and_refused(env):
    panel, bridge, port = env
    call(bridge, port, "POST", "/cmd", {"line": "PARAM GET x", "wait_ms": 0})
    call(bridge, port, "POST", "/cmd", {"line": "ESC KV 1300", "wait_ms": 0})
    assert panel.log[0] == "[AI] > PARAM GET x"
    assert panel.log[1].startswith("[AI] 已拒绝: ESC KV 1300（")
    assert panel.transport.sent == ["PARAM GET x"]


def test_validation_read_only_gate_respected(env):
    panel, bridge, port = env
    panel._validation_command_allowed = lambda line: False
    data = call(bridge, port, "POST", "/cmd", {"line": "PARAM SET a 1"})[1]
    assert data["sent"] is False and "只读门" in data["reason"]
    assert panel.transport.sent == []


# ------------------------------------------------------------------ 策略
ALLOWED = [
    "PARAM?", "STATUS?", "SYSID?", "SYSID ?", "SYSID THR?", "PARAM GET coax.att_kp", "param set coax.att_kp 1.2",
    "PARAM SET coax.rate_kp 0.3", "SYSID PARAM coax.rate_kp 0.3", "SYSID PARAM ?",
    "SYSID ALT inject=break mass_g=1145", "SYSID MODE ALT", "SYSID MODE YAW",
    "SYSID YAW inject=diff thrust_mn=3700 twist_deg=720", "SYSID YAW", "SYSID EXC profile=prbs amp=0.1",
    "SYSID LIMIT angle_deg=5", "SYSID THROTTLE target_n=0 max_pct=30", "SYSID STOP", "STOP",
    "MOTOR STOP", "IDENT STOP", "TBENCH STOP", "PROPCAL SPIN STOP", "ACCEPT V2 STOP",
    "  SYSID   STOP  ",
]

# 每一条都核对过固件里确实存在（出处见 ai_bridge_policy 顶部注释）。
NEVER = [
    "ARM", "arm", "IDENT ARM", "TBENCH ARM confirm=bench max_pct=10 request_id=1",
    "PROPCAL SPIN ARM confirm=prop", "REQ id=1 mod=ARM op=ARM",
    "MOTOR SET 0 10", "MOTOR SET 1 100", "MOTOR SET 0", "MOTOR SET 0 0 5", "MOTOR JUNK",
    "PWM1:50", "PWM2:1",
    "SYSID START", "sysid start", "IDENT START", "IDENT STEP", "IDENT DOUBLET", "IDENT PRBS",
    "ACCEPT V2 START props=2", "ACCEPT V2 STAGE", "ACCEPT V2 KEEPALIVE",
    "ESC KV 1300", "ESC EDT 1", "ESC KV",
    "TBENCH SET upper_pct=5 lower_pct=5", "PROPCAL SPIN", "PROPCAL SPIN START",
    "SYSID ARM",
]

DEFAULT_REFUSED = ["SAVE", "DEFAULTS", "SERVO MOVE 1 10", "LED RGB 1 2 3", "BOOT", "BOOT DFU", "BOOT DFU NOW",
                   "MOTOR SET 0 0", "WIFI EN 1", "FLASH ERASE", "PROBE", "SYSID HOLD", "DISARM"]


@pytest.mark.parametrize("line", ALLOWED)
def test_policy_allows_whitelist(line):
    assert policy.classify(line).kind == policy.ALLOW, line


@pytest.mark.parametrize("line", NEVER)
def test_policy_never_rules(line):
    verdict = policy.classify(line)
    assert verdict.kind == policy.DENY and verdict.reason, line


@pytest.mark.parametrize("line", DEFAULT_REFUSED)
def test_policy_default_refuses(line):
    assert policy.classify(line).kind == policy.DENY, line


@pytest.mark.parametrize("line", ["", "   ", "PARAM?\r\nARM", "PARAM?\nSYSID START", "PARAM?\x00", "X" * 300])
def test_policy_rejects_multiline_and_garbage(line):
    assert policy.classify(line).kind == policy.DENY


def test_never_rules_beat_whitelist(monkeypatch):
    monkeypatch.setattr(policy, "WHITELIST_PREFIXES", policy.WHITELIST_PREFIXES + (("SYSID", "START"), ("ESC",), ("MOTOR",)))
    for line in ("SYSID START", "ESC KV 1", "MOTOR SET 0 50"):
        assert policy.classify(line).kind == policy.DENY


def test_denied_never_reaches_board(env):
    panel, bridge, port = env
    for line in NEVER[:12]:
        data = call(bridge, port, "POST", "/cmd", {"line": line})[1]
        assert data["sent"] is False and data["reason"]
    assert panel.transport.sent == []
    assert len([x for x in panel.log if x.startswith("[AI] 已拒绝")]) == 12


def test_firmware_really_has_the_dangerous_commands():
    """永久拒绝表不是编的：这些命令名在固件源码里都能找到。"""
    def text(name):
        return (ROOT / "App" / "Src" / name).read_text(encoding="utf-8", errors="replace")

    control, sysid = text("app_control.c"), text("app_cmd_sysid.c")
    assert 'strcmp(tokens[0], "MOTOR") == 0' in control and 'strcmp(tokens[1], "SET")' in control
    assert 'strncmp(tokens[0], "PWM", 3)' in control
    assert 'strcmp(tokens[1], "START") == 0' in control       # IDENT START
    assert 'strcmp(sub, "START") == 0' in sysid and 'strcmp(sub, "ARM") == 0' in sysid
    assert 'strcmp(tokens[1], "ARM") == 0' in text("app_cmd_thrust_bench.c")
    assert 'strcmp(tokens[1], "SPIN") == 0' in text("app_cmd_propcal.c")
    assert 'strcmp(tokens[1], "KV")' in text("app_cmd_esc_kv.c")
    assert 'strcmp(tokens[1], "EDT")' in text("app_cmd_esc_edt.c")
    assert '"START"' in control and '"KEEPALIVE"' in control
    assert 'strcmp(tokens[1], "DFU")' in control and 'strcmp(tokens[2], "CONFIRM")' in control
    for word in ("THR?", "THROTTLE", "EXC", "LIMIT", "MODE", "ALT", "PARAM", "STOP"):
        assert f'"{word}"' in sysid or word in sysid


# ------------------------------------------------------------------ DFU
def grant(bridge):
    bridge.grant_dfu_once()


def test_dfu_is_automatic_but_only_when_disarmed(env):
    """2026-09-30 作者："你就别让我允许了改成全自动的"——不再要界面许可，只要新鲜的 armed=0。"""
    panel, bridge, port = env
    dfu = {"line": "BOOT DFU CONFIRM", "wait_ms": 0}

    # 1) 已解锁：拒绝
    feed(panel, "SYSID THR auto=0 armed=1")
    data = call(bridge, port, "POST", "/cmd", dfu)[1]
    assert data["sent"] is False and "armed=1" in data["reason"]
    assert "BOOT DFU CONFIRM" not in panel.transport.sent

    # 2) 上锁：不用任何许可就放行，可以连着用
    feed(panel, "SYSID THR auto=0 armed=0")
    data = call(bridge, port, "POST", "/cmd", dfu)[1]
    assert data["sent"] is True
    assert panel.transport.sent[-1] == "BOOT DFU CONFIRM"
    feed(panel, "SYSID THR auto=0 armed=0")
    assert call(bridge, port, "POST", "/cmd", dfu)[1]["sent"] is True

    # 3) 作者关掉 AI 接口：一律拒绝
    bridge.set_enabled(False)
    feed(panel, "SYSID THR auto=0 armed=0")
    assert call(bridge, port, "POST", "/cmd", dfu)[1]["sent"] is False


def test_dfu_armed_report_must_be_fresh(tmp_path):
    clock = {"t": 100.0}
    panel = FakePanel()
    bridge = ab.AiBridge(panel, info_file=str(tmp_path / "i.json"), clock=lambda: clock["t"])
    register_board_line_hook(panel, bridge.on_board_line)
    feed(panel, "SYSID THR armed=0")
    clock["t"] += 2.5
    assert bridge._armed_fresh() is None
    clock["t"] = 101.0
    bridge.on_board_line("SYSID THR armed=0")
    clock["t"] += 1.9
    assert bridge._armed_fresh() == 0


def test_dfu_stale_armed_triggers_probe_then_accepts(env):
    panel, bridge, port = env
    grant(bridge)
    # 过期的 armed 记录
    bridge._armed, bridge._armed_at = 0, bridge.clock() - 30

    original = panel.transport.send_line

    def answering(line):
        ok = original(line)
        if line == "SYSID THR?":
            threading.Timer(0.05, lambda: feed(panel, "SYSID THR auto=0 armed=0")).start()
        return ok

    panel.transport.send_line = answering
    data = call(bridge, port, "POST", "/cmd", {"line": "BOOT DFU CONFIRM", "wait_ms": 0})[1]
    assert data["sent"] is True
    assert panel.transport.sent[:2] == ["SYSID THR?", "BOOT DFU CONFIRM"]


def test_dfu_no_report_at_all_refused(env):
    panel, bridge, port = env
    grant(bridge)
    bridge_probe_wait = ab.DFU_PROBE_WAIT_S
    ab.DFU_PROBE_WAIT_S = 0.2
    try:
        data = call(bridge, port, "POST", "/cmd", {"line": "BOOT DFU CONFIRM"})[1]
    finally:
        ab.DFU_PROBE_WAIT_S = bridge_probe_wait
    assert data["sent"] is False and "armed" in data["reason"]
    assert "BOOT DFU CONFIRM" not in panel.transport.sent


def test_dfu_bypasses_validation_gate_only_after_checks(env):
    panel, bridge, port = env
    panel._validation_command_allowed = lambda line: False
    grant(bridge)
    feed(panel, "SYSID THR armed=0")
    assert call(bridge, port, "POST", "/cmd", {"line": "BOOT DFU CONFIRM", "wait_ms": 0})[1]["sent"] is True


def test_disabling_bridge_revokes_dfu(env):
    panel, bridge, port = env
    grant(bridge)
    bridge.set_enabled(False)
    assert bridge.dfu_granted is False


class ReconnectPanel(FakePanel):
    """串口直连的面板：`after` 只登记回调，由测试手动逐拍推进。"""

    def __init__(self):
        super().__init__()
        self.serial_transport = self.transport
        self.serial_transport.active_port = "COM22"
        self.serial_baud_var = type("B", (), {"get": lambda self: "921600"})()
        self._serial_port_map = {}
        self.pending = []
        self.open_results = []
        self.reconnect_calls = []

    def after(self, ms, fn):
        self.pending.append(fn)

    def step(self):
        jobs, self.pending = self.pending, []
        for fn in jobs:
            fn()

    def _refresh_serial_ports(self):
        return list(self._serial_port_map)

    def _firmware_auto_reconnect(self, port):
        self.reconnect_calls.append((port, self.firmware_reconnect_baud))
        ok = self.open_results.pop(0)
        self.connected = ok
        return ok


def test_dfu_through_the_bridge_reconnects_when_the_port_returns(tmp_path):
    """2026-09-30：经 AI 接口进 DFU 烧完，地面站一直断着（面板只有自己的升级流程会重连）。"""
    clock = [0.0]
    panel = ReconnectPanel()
    bridge = ab.AiBridge(panel, info_file=str(tmp_path / "i.json"), clock=lambda: clock[0])
    assert bridge._reconnect_target() == (True, ("COM22", 921600))
    bridge._watch_reconnect("COM22", 921600)

    panel.step()                       # 还连着：继续等断开
    assert panel.pending and not panel.reconnect_calls
    panel.connected = False            # CDC 断开，DFU 中端口不在
    panel.step()
    assert not panel.reconnect_calls
    panel._serial_port_map = {"COM22 STM": "COM22"}
    panel.open_results = [False, True]  # 刚枚举出来第一次打不开，下一拍成功
    panel.step()
    panel.step()
    assert panel.reconnect_calls == [("COM22", 921600), ("COM22", 921600)]
    assert panel.connected and not panel.pending
    assert any("已自动重连 COM22" in line for line in panel.log)


def test_dfu_reconnect_gives_up_after_the_timeout_and_skips_tcp(tmp_path):
    clock = [0.0]
    panel = ReconnectPanel()
    bridge = ab.AiBridge(panel, info_file=str(tmp_path / "i.json"), clock=lambda: clock[0])
    bridge._watch_reconnect("COM22", 921600)
    panel.connected = False
    clock[0] = ab.RECONNECT_TIMEOUT_S + 1.0
    panel.step()
    assert not panel.pending and not panel.reconnect_calls
    assert any("请手动连接" in line for line in panel.log)

    tcp = ReconnectPanel()
    tcp.transport = FakeTransport()    # 当前走 TCP，不是串口
    assert ab.AiBridge(tcp, info_file=str(tmp_path / "j.json"))._reconnect_target() == (True, ("", 0))


# ------------------------------------------------------------------ 客户端 / 体量
def test_client_reports_missing_panel_in_chinese(monkeypatch, tmp_path, capsys):
    from tools import ai_bridge_client as client
    monkeypatch.setenv("TEMP", str(tmp_path))
    monkeypatch.setenv("TMP", str(tmp_path))
    monkeypatch.setenv("TMPDIR", str(tmp_path))
    import tempfile
    monkeypatch.setattr(tempfile, "tempdir", str(tmp_path))
    with pytest.raises(SystemExit) as exc:
        client.main(["status"])
    assert "地面站" in str(exc.value)


def test_client_roundtrip_against_bridge(env, monkeypatch, capsys):
    from tools import ai_bridge_client as client
    panel, bridge, port = env
    monkeypatch.setattr(client, "info_path", lambda: bridge.info_file)
    feed(panel, "hello 中文")
    assert client.main(["lines", "--since", "0"]) == 0
    assert "hello 中文" in capsys.readouterr().out
    assert client.main(["send", "ESC KV 1"]) == 2
    assert "未发送" in capsys.readouterr().out
    assert client.main(["send", "PARAM?", "--wait-ms", "50"]) == 0


def test_panel_source_did_not_grow():
    path = ROOT / "tools" / "drone_tcp_panel.py"
    raw = path.read_bytes()
    assert raw.count(b"\n") <= PANEL_MAX_LINES
    assert len(raw) <= PANEL_MAX_BYTES


def test_bridge_is_wired_outside_panel_file():
    shell = (ROOT / "tools" / "panel_lib" / "shell.py").read_text(encoding="utf-8")
    assert "mount_ai_bridge(self, root)" in shell
    assert "ai_bridge" not in (ROOT / "tools" / "drone_tcp_panel.py").read_text(encoding="utf-8")


def test_environment_guard_blocks_server_under_pytest():
    from tools.panel_lib import ai_bridge_ui
    assert ai_bridge_ui._bridge_disabled_by_environment() is True


def test_telem_endpoint_reads_the_dashboard_latest_values(env):
    """2026-09-30：量化磁航向漂移要读 yaw（只在二进制遥测流里）：/telem 走面板仪表盘的最新值。"""
    panel, bridge, port = env
    panel._dashboard_latest = lambda name: {"yaw": -137.25, "roll": 0.1}.get(name)
    code, data = call(bridge, port, "GET", "/telem?names=yaw,roll,pitch")
    assert code == 200
    assert data["values"] == {"yaw": -137.25, "roll": 0.1, "pitch": None}
