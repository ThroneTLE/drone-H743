"""地面站 AI 接口的命令行客户端，给 AI 会话在 bash 里用。

用法示例：
  python tools/ai_bridge_client.py status
  python tools/ai_bridge_client.py lines --since 120
  python tools/ai_bridge_client.py send "SYSID THR?" --wait-ms 800 --expect "SYSID THR"
  printf 'PARAM?\nSYSID ALT\n' | python tools/ai_bridge_client.py sendmany

端口和 token 从 %TEMP%\\drone_panel_ai_bridge.json 读取（地面站启动时写入）。
"""

import argparse
import json
import os
import sys
import tempfile
import urllib.error
import urllib.request


def info_path() -> str:
    return os.path.join(tempfile.gettempdir(), "drone_panel_ai_bridge.json")


def load_info() -> dict:
    try:
        with open(info_path(), encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError):
        raise SystemExit("错误：找不到地面站 AI 接口信息文件。请确认地面站已启动（并且是带 AI 接口的新版本）。")


def request(method: str, path: str, body=None, timeout: float = 15.0):
    info = load_info()
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        f"http://127.0.0.1:{info['port']}{path}", data=data, method=method,
        headers={"X-Token": info["token"], "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        if exc.code == 403:
            raise SystemExit("错误：token 被拒绝，地面站可能重启过；请重试或确认信息文件是最新的。")
        try:
            return json.loads(exc.read().decode("utf-8"))
        except ValueError:
            raise SystemExit(f"错误：地面站返回 HTTP {exc.code}")
    except (urllib.error.URLError, ConnectionError, TimeoutError, OSError):
        raise SystemExit("错误：连不上地面站 AI 接口。地面站没开、已关闭，或信息文件是旧的（进程已退出）。")


def print_lines(lines) -> None:
    for item in lines:
        print(f"{item['seq']:>6} {item['line']}")


def cmd_send(line: str, wait_ms: int, expect) -> int:
    body = {"line": line, "wait_ms": wait_ms}
    if expect:
        body["expect"] = expect
    result = request("POST", "/cmd", body, timeout=wait_ms / 1000.0 + 10.0)
    if not result.get("sent"):
        print(f"[未发送] {line}  原因：{result.get('reason', '')}")
        return 2
    print(f"[已发送] {line}")
    print_lines(result.get("lines", []))
    if expect:
        print(f"[expect] {'匹配' if result.get('matched') else '超时未匹配'}")
        return 0 if result.get("matched") else 1
    return 0


def main(argv=None) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description="地面站 AI 接口客户端")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status", help="查看连接与 armed 状态")
    p_lines = sub.add_parser("lines", help="读取最近收到的行")
    p_lines.add_argument("--since", type=int, default=0)
    p_lines.add_argument("--limit", type=int, default=200)
    p_send = sub.add_parser("send", help="发送一行命令并收集回复")
    p_send.add_argument("line")
    p_send.add_argument("--wait-ms", type=int, default=800)
    p_send.add_argument("--expect", default=None)
    p_many = sub.add_parser("sendmany", help="从标准输入逐行发送")
    p_many.add_argument("--wait-ms", type=int, default=800)
    p_telem = sub.add_parser("telem", help="读遥测通道最新值（通道须在仪表盘订阅里）")
    p_telem.add_argument("--names", default="roll,pitch,yaw")
    args = parser.parse_args(argv)

    if args.cmd == "telem":
        print(json.dumps(request("GET", f"/telem?names={args.names}"), ensure_ascii=False))
        return 0

    if args.cmd == "status":
        print(json.dumps(request("GET", "/status"), ensure_ascii=False, indent=2))
        return 0
    if args.cmd == "lines":
        result = request("GET", f"/lines?since={args.since}&limit={args.limit}")
        print_lines(result.get("lines", []))
        print(f"[seq={result.get('seq')}]")
        return 0
    if args.cmd == "send":
        return cmd_send(args.line, args.wait_ms, args.expect)
    worst = 0
    for raw in sys.stdin:
        line = raw.strip()
        if line:
            worst = max(worst, cmd_send(line, args.wait_ms, None))
    return worst


if __name__ == "__main__":
    sys.exit(main())
