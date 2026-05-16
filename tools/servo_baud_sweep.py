#!/usr/bin/env python3
"""Sweep Zhongling bus-servo baud rates from a PC serial adapter.

This is meant for recovering a servo after its ID or baud rate is unknown.
Connect the USB-UART TX and GND to the servo signal and GND. Use a separate
servo power supply, and keep grounds common.
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError as exc:  # pragma: no cover - user environment check
    raise SystemExit("pyserial is required: python -m pip install pyserial") from exc


BAUD_BY_CODE: dict[int, int] = {
    1: 9600,
    2: 19200,
    3: 38400,
    4: 57600,
    5: 115200,
    6: 128000,
    7: 256000,
    8: 1000000,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sweep Zhongling bus-servo baud rates and send broadcast commands."
    )
    parser.add_argument("port", help="Serial port, for example COM8")
    parser.add_argument(
        "--commands",
        nargs="+",
        default=["#255P1500T0500!", "#255PID!", "#255PVER!"],
        help="Commands to send at every baud. Default: center, read ID, read version.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=2,
        help="How many times to send each command at each baud.",
    )
    parser.add_argument(
        "--gap",
        type=float,
        default=0.20,
        help="Delay between commands in seconds.",
    )
    parser.add_argument(
        "--read-time",
        type=float,
        default=0.35,
        help="How long to listen after each command.",
    )
    parser.add_argument(
        "--recover-baud",
        type=int,
        choices=sorted(BAUD_BY_CODE),
        help="After finding a responsive baud manually, set servo baud code by broadcast.",
    )
    parser.add_argument(
        "--single-baud",
        type=int,
        help="Only test one serial baud instead of sweeping all known rates.",
    )
    return parser.parse_args()


def read_reply(port: "serial.Serial", seconds: float) -> bytes:
    end = time.monotonic() + seconds
    data = bytearray()
    while time.monotonic() < end:
        chunk = port.read(256)
        if chunk:
            data.extend(chunk)
        else:
            time.sleep(0.01)
    return bytes(data)


def write_command(port: "serial.Serial", command: str) -> bytes:
    payload = command.encode("ascii")
    port.reset_input_buffer()
    port.write(payload)
    port.flush()
    return payload


def send_at_baud(args: argparse.Namespace, baud: int, commands: list[str]) -> bool:
    got_reply = False
    print(f"\n=== baud {baud} ===")
    try:
        with serial.Serial(baudrate=baud, port=args.port, timeout=0.02, write_timeout=0.5) as port:
            time.sleep(0.15)
            for command in commands:
                for index in range(args.repeat):
                    payload = write_command(port, command)
                    print(f"TX[{index + 1}/{args.repeat}] {payload!r}")
                    reply = read_reply(port, args.read_time)
                    if reply:
                        got_reply = True
                        print(f"RX {reply!r}")
                    time.sleep(args.gap)
    except serial.SerialException as exc:
        print(f"ERR opening/sending at {baud}: {exc}", file=sys.stderr)
    return got_reply


def main() -> int:
    args = parse_args()
    bauds = [args.single_baud] if args.single_baud else list(BAUD_BY_CODE.values())
    commands = list(args.commands)

    print("Zhongling bus-servo baud sweep")
    print("Use one servo at a time when changing ID or baud.")
    print("Broadcast ID is 255; factory default servo ID is usually 000.")

    responsive: list[int] = []
    for baud in bauds:
        if send_at_baud(args, baud, commands):
            responsive.append(baud)

    if args.recover_baud is not None:
        recover_cmd = f"#255PBD{args.recover_baud}!"
        target = BAUD_BY_CODE[args.recover_baud]
        print(f"\nRecovery command requested: {recover_cmd} -> baud {target}")
        print("Send this only if one servo is connected and you accept changing its baud.")
        for baud in responsive or bauds:
            send_at_baud(args, baud, [recover_cmd])

    print("\nDone.")
    if responsive:
        print("Responsive baud(s): " + ", ".join(str(v) for v in responsive))
    else:
        print("No reply captured. If the servo moves but no RX is wired, this can still be useful.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
