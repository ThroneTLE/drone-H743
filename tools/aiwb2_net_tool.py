#!/usr/bin/env python3
"""PC-side helper for Ai-WB2-12F network bring-up.

This tool keeps the computer side simple while the module is tested through
AT commands. It can listen for UDP/TCP data from the module, send UDP probes
to the module, and optionally configure the module over a CH340 serial port.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass


DEFAULT_WIFI_SSID = os.getenv("AIWB2_WIFI_SSID")
DEFAULT_WIFI_PASSWORD = os.getenv("AIWB2_WIFI_PASSWORD")
DEFAULT_MODULE_IP = os.getenv("AIWB2_MODULE_IP", "192.168.223.181")


def env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


DEFAULT_TCP_PORT = env_int("AIWB2_TCP_PORT", 6666)


def local_ip_for(target_ip: str) -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((target_ip, 9))
        return probe.getsockname()[0]
    finally:
        probe.close()


def print_line(prefix: str, data: bytes, addr: tuple[str, int] | None = None) -> None:
    when = time.strftime("%H:%M:%S")
    text = data.decode("utf-8", errors="replace")
    if addr is None:
        print(f"[{when}] {prefix}: {text!r}")
    else:
        print(f"[{when}] {prefix} {addr[0]}:{addr[1]}: {text!r}")
    sys.stdout.flush()


def run_udp_server(bind_ip: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, port))
    print(f"UDP listening on {bind_ip}:{port}")
    print("Press Ctrl-C to stop.")
    while True:
        data, addr = sock.recvfrom(4096)
        print_line("UDP RX from", data, addr)


def run_udp_reply_console(bind_ip: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, port))
    last_addr: list[tuple[str, int] | None] = [None]

    print(f"UDP reply console listening on {bind_ip}:{port}")
    print("Wait for the module to send one packet first.")
    print("Then type lines here to reply to the module's source port.")
    print("Press Ctrl-C to stop.")

    def rx_loop() -> None:
        while True:
            data, addr = sock.recvfrom(4096)
            last_addr[0] = addr
            print_line("UDP RX from", data, addr)

    thread = threading.Thread(target=rx_loop, daemon=True)
    thread.start()

    for line in sys.stdin:
        line = line.rstrip("\n")
        if not line:
            continue
        if last_addr[0] is None:
            print("No module packet seen yet; cannot reply.")
            continue
        sock.sendto(line.encode("utf-8"), last_addr[0])
        print(f"UDP TX to {last_addr[0][0]}:{last_addr[0][1]}: {line!r}")


def run_udp_probe(local_port: int, module_ip: str, module_port: int, message: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if local_port > 0:
        sock.bind(("", local_port))
    payload = message.encode("utf-8")
    sent = sock.sendto(payload, (module_ip, module_port))
    print(f"UDP TX {sent} bytes to {module_ip}:{module_port}: {message!r}")


def run_udp_bridge(bind_ip: str,
                   local_port: int,
                   module_ip: str,
                   module_port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_ip, local_port))
    print(f"UDP bridge listening on {bind_ip}:{local_port}")
    print(f"Type lines to send to module {module_ip}:{module_port}.")
    print("Incoming datagrams are printed below. Press Ctrl-C to stop.")

    def rx_loop() -> None:
        while True:
            data, addr = sock.recvfrom(4096)
            print_line("UDP RX from", data, addr)

    thread = threading.Thread(target=rx_loop, daemon=True)
    thread.start()

    for line in sys.stdin:
        line = line.rstrip("\n")
        if line:
            sock.sendto(line.encode("utf-8"), (module_ip, module_port))
            print(f"UDP TX to {module_ip}:{module_port}: {line!r}")


def run_tcp_server(bind_ip: str, port: int) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_ip, port))
    server.listen(1)
    print(f"TCP listening on {bind_ip}:{port}")
    print("Wait for Ai-WB2 to run: AT+SOCKET=4,<pc-ip>,<port>")

    conn, addr = server.accept()
    print(f"TCP client connected from {addr[0]}:{addr[1]}")
    conn.settimeout(0.2)

    stop = False

    def rx_loop() -> None:
        nonlocal stop
        while not stop:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            if not data:
                print("TCP client closed.")
                stop = True
                break
            print_line("TCP RX", data)

    thread = threading.Thread(target=rx_loop, daemon=True)
    thread.start()

    try:
        for line in sys.stdin:
            if stop:
                break
            conn.sendall(line.encode("utf-8"))
    finally:
        stop = True
        conn.close()
        server.close()


@dataclass
class AtConfig:
    serial_port: str
    baud: int
    ssid: str
    password: str
    local_port: int


def serial_configure(config: AtConfig) -> None:
    try:
        import serial  # type: ignore
    except ModuleNotFoundError:
        print("pyserial is not installed.")
        print("Install it with: python3 -m pip install pyserial")
        raise SystemExit(2)

    def read_until_idle(port: "serial.Serial", idle_s: float = 0.4, total_s: float = 8.0) -> str:
        end = time.monotonic() + total_s
        idle_end = time.monotonic() + idle_s
        chunks: list[bytes] = []
        while time.monotonic() < end:
            data = port.read(4096)
            if data:
                chunks.append(data)
                idle_end = time.monotonic() + idle_s
            elif time.monotonic() >= idle_end:
                break
        return b"".join(chunks).decode("utf-8", errors="replace")

    commands = [
        "ATE0",
        "AT+WMODE=1,1",
        f'AT+WJAP="{config.ssid}","{config.password}"',
        "AT+WAUTOCONN=1",
        f"AT+SOCKETAUTOTT=1,{config.local_port}",
        "AT+RST",
    ]

    with serial.Serial(config.serial_port, config.baud, timeout=0.05) as port:
        for command in commands:
            print(f">>> {command}")
            port.write((command + "\r\n").encode("utf-8"))
            port.flush()
            response = read_until_idle(port, total_s=20.0 if "WJAP" in command else 8.0)
            if response:
                print(response, end="" if response.endswith("\n") else "\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Ai-WB2-12F PC-side network helper")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_ip = sub.add_parser("ip", help="print the PC IP selected for reaching a target")
    p_ip.add_argument("--target", default=DEFAULT_MODULE_IP)

    p_udp = sub.add_parser("udp-server", help="listen for UDP data from the module")
    p_udp.add_argument("--bind", default="0.0.0.0")
    p_udp.add_argument("--port", type=int, default=6666)

    p_reply = sub.add_parser("udp-reply-console",
                             help="listen and reply to the module's last UDP source port")
    p_reply.add_argument("--bind", default="0.0.0.0")
    p_reply.add_argument("--port", type=int, default=6666)

    p_probe = sub.add_parser("udp-send", help="send one UDP datagram to the module")
    p_probe.add_argument("--local-port", type=int, default=0)
    p_probe.add_argument("--module-ip", required=True)
    p_probe.add_argument("--module-port", type=int, required=True)
    p_probe.add_argument("--message", default="PING1234")

    p_bridge = sub.add_parser("udp-bridge", help="receive UDP and type lines to send")
    p_bridge.add_argument("--bind", default="0.0.0.0")
    p_bridge.add_argument("--local-port", type=int, default=6666)
    p_bridge.add_argument("--module-ip", required=True)
    p_bridge.add_argument("--module-port", type=int, required=True)

    p_tcp = sub.add_parser("tcp-server", help="listen for a TCP client from the module")
    p_tcp.add_argument("--bind", default="0.0.0.0")
    p_tcp.add_argument("--port", type=int, default=DEFAULT_TCP_PORT)

    p_at = sub.add_parser("configure-at", help="configure module over CH340 serial")
    p_at.add_argument("--serial-port", required=True)
    p_at.add_argument("--baud", type=int, default=115200)
    p_at.add_argument("--ssid",
                      default=DEFAULT_WIFI_SSID,
                      required=DEFAULT_WIFI_SSID is None)
    p_at.add_argument("--password",
                      default=DEFAULT_WIFI_PASSWORD,
                      required=DEFAULT_WIFI_PASSWORD is None)
    p_at.add_argument("--local-port", type=int, default=7777)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.cmd == "ip":
        print(local_ip_for(args.target))
    elif args.cmd == "udp-server":
        run_udp_server(args.bind, args.port)
    elif args.cmd == "udp-reply-console":
        run_udp_reply_console(args.bind, args.port)
    elif args.cmd == "udp-send":
        run_udp_probe(args.local_port, args.module_ip, args.module_port, args.message)
    elif args.cmd == "udp-bridge":
        run_udp_bridge(args.bind, args.local_port, args.module_ip, args.module_port)
    elif args.cmd == "tcp-server":
        run_tcp_server(args.bind, args.port)
    elif args.cmd == "configure-at":
        serial_configure(AtConfig(args.serial_port,
                                  args.baud,
                                  args.ssid,
                                  args.password,
                                  args.local_port))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
