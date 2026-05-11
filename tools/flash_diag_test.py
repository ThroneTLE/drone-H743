#!/usr/bin/env python3
"""Run drone-H743 RTOS/flash diagnostics over a noisy UART stream.

The firmware may continuously print IMU samples. This tool sends one command at
a time, filters unrelated lines, waits for the expected RTOS/FLASH response, and
prints a compact pass/fail result.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, TextIO


DEFAULT_SERIAL_PORT = "COM18" if os.name == "nt" else ""
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 3.0
DEFAULT_SETTLE_S = 0.25
DEFAULT_MIN_STACK_WORDS = 64

VERIFY_COMMANDS = (
    "FLASH VERIFY 0x000000 16",
    "FLASH VERIFY 0x000000 31",
    "FLASH VERIFY 0x000000 32",
    "FLASH VERIFY 0x000000 33",
    "FLASH VERIFY 0x0000F0 512",
    "FLASH VERIFY 0x000FF0 512",
    "FLASH VERIFY 0x001000 4096",
)

BENCH_COMMANDS = (
    "FLASH BENCH READ 0x000000 256 200 0",
    "FLASH BENCH READ 0x000000 256 200 1",
    "FLASH BENCH READ 0x000000 4096 100 0",
    "FLASH BENCH READ 0x000000 4096 100 1",
)

KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=([^,\s]+)")


@dataclass
class CommandSpec:
    text: str
    kind: str
    timeout_s: float
    settle_s: float


@dataclass
class CheckResult:
    status: str
    detail: str


class SerialTransport:
    def __init__(self, port: str, baud: int) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc

        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05,
            write_timeout=1.0,
            rtscts=False,
            dsrdtr=False,
        )

    def read_some(self) -> bytes:
        waiting = getattr(self._serial, "in_waiting", 0)
        size = min(max(waiting, 1), 4096)
        return self._serial.read(size)

    def write_line(self, line: str) -> None:
        self._serial.write((line + "\r\n").encode("ascii"))
        self._serial.flush()

    def reset_input(self) -> None:
        self._serial.reset_input_buffer()

    def close(self) -> None:
        self._serial.close()


class LineReader:
    def __init__(self, transport: SerialTransport) -> None:
        self.transport = transport
        self.buffer = bytearray()

    def poll(self) -> List[str]:
        data = self.transport.read_some()
        if not data:
            return []

        self.buffer.extend(data)
        if len(self.buffer) > 16384:
            del self.buffer[:-4096]

        lines: List[str] = []
        while True:
            try:
                index = self.buffer.index(0x0A)
            except ValueError:
                break

            raw = bytes(self.buffer[:index])
            del self.buffer[: index + 1]
            raw = raw.rstrip(b"\r")
            text = raw.decode("utf-8", errors="replace").strip()
            if text:
                lines.append(text)
        return lines


def parse_kv(line: str) -> Dict[str, str]:
    return {key: value for key, value in KV_RE.findall(line)}


def int_value(fields: Dict[str, str], key: str) -> Optional[int]:
    value = fields.get(key)
    if value is None:
        return None
    try:
        return int(value, 0)
    except ValueError:
        return None


def infer_kind(command: str) -> str:
    cmd = command.strip().upper()
    if cmd == "RTOS?":
        return "rtos"
    if cmd == "FLASH?":
        return "flash_status"
    if cmd.startswith("FLASH VERIFY"):
        return "flash_verify"
    if cmd.startswith("FLASH BENCH READ"):
        return "flash_bench"
    if cmd.startswith("FLASH SCRATCH"):
        return "flash_scratch"
    return "generic"


def is_expected_line(kind: str, line: str) -> bool:
    if line.startswith("ERR "):
        return True
    if kind == "rtos":
        return line.startswith("RTOS ")
    if kind == "flash_status":
        return line.startswith("FLASH ok=") or line.startswith("FLASH cfg_addr=")
    if kind == "flash_verify":
        return line.startswith("FLASH verify")
    if kind == "flash_bench":
        return line.startswith("FLASH bench_read")
    if kind == "flash_scratch":
        return line.startswith("FLASH scratch")
    return line.startswith("RTOS ") or line.startswith("FLASH ") or line.startswith("OK ")


def is_noise_line(line: str) -> bool:
    prefixes = (
        "n=",
        "roll=",
        "ll=",
        "ax=",
        "ay=",
        "az=",
        "gx=",
        "gy=",
        "gz=",
        ",gx=",
        "READY",
        "RX ",
        "ACK ",
        "BARO ",
        "IMU ",
        "GPS ",
        "MAG ",
        "PONG ",
    )
    return line.startswith(prefixes)


def evaluate(kind: str, lines: List[str], min_stack_words: int) -> CheckResult:
    for line in lines:
        if line.startswith("ERR "):
            return CheckResult("FAIL", line)

    if kind == "rtos":
        return evaluate_rtos(lines, min_stack_words)
    if kind == "flash_status":
        return evaluate_flash_status(lines)
    if kind == "flash_verify":
        return evaluate_flash_verify(lines)
    if kind == "flash_bench":
        return evaluate_flash_bench(lines)
    if kind == "flash_scratch":
        return CheckResult("PASS", lines[0] if lines else "scratch sector reported")

    if lines:
        return CheckResult("PASS", "response received")
    return CheckResult("FAIL", "no expected response")


def evaluate_rtos(lines: List[str], min_stack_words: int) -> CheckResult:
    heap_line = next((line for line in lines if line.startswith("RTOS heap_free=")), None)
    if heap_line is None:
        return CheckResult("FAIL", "missing RTOS heap line")

    fields = parse_kv(heap_line)
    stack_fault = int_value(fields, "fault_stack")
    malloc_fault = int_value(fields, "fault_malloc")
    if stack_fault != 0 or malloc_fault != 0:
        return CheckResult("FAIL", heap_line)

    free_values: List[int] = []
    for line in lines:
        if not line.startswith("RTOS task="):
            continue
        task_fields = parse_kv(line)
        free = int_value(task_fields, "free_stack_words")
        if free is not None:
            free_values.append(free)

    heap_free = fields.get("heap_free", "?")
    heap_min = fields.get("heap_min", "?")
    if free_values:
        min_free = min(free_values)
        detail = f"heap_free={heap_free} heap_min={heap_min} min_stack_words={min_free}"
        if min_free < min_stack_words:
            return CheckResult("WARN", detail)
        return CheckResult("PASS", detail)

    return CheckResult("PASS", f"heap_free={heap_free} heap_min={heap_min}")


def evaluate_flash_status(lines: List[str]) -> CheckResult:
    status_line = next((line for line in lines if line.startswith("FLASH ok=")), None)
    if status_line is None:
        return CheckResult("FAIL", "missing FLASH ok line")

    fields = parse_kv(status_line)
    if int_value(fields, "ok") != 1:
        return CheckResult("FAIL", status_line)

    flash_id = fields.get("id", "?")
    expected_id = fields.get("exp")
    if expected_id is not None and flash_id.upper() != expected_id.upper():
        return CheckResult("FAIL", status_line)

    cfg_line = next((line for line in lines if line.startswith("FLASH cfg_addr=")), "")
    sr1 = fields.get("sr1", "?")
    cfg_fields = parse_kv(cfg_line)
    cfg_valid = cfg_fields.get("cfg_valid", "?")
    return CheckResult("PASS", f"id={flash_id} sr1={sr1} cfg_valid={cfg_valid}")


def evaluate_flash_verify(lines: List[str]) -> CheckResult:
    line = next((item for item in lines if item.startswith("FLASH verify")), None)
    if line is None:
        return CheckResult("FAIL", "missing FLASH verify line")

    fields = parse_kv(line)
    block_ok = int_value(fields, "st_block") == 0
    dma_ok = int_value(fields, "st_dma") == 0
    match_ok = int_value(fields, "match") == 1
    if block_ok and dma_ok and match_ok:
        return CheckResult("PASS", line)
    return CheckResult("FAIL", line)


def evaluate_flash_bench(lines: List[str]) -> CheckResult:
    line = next((item for item in lines if item.startswith("FLASH bench_read")), None)
    if line is None:
        return CheckResult("FAIL", "missing FLASH bench_read line")

    fields = parse_kv(line)
    status_ok = int_value(fields, "st") == 0
    loops = int_value(fields, "loops")
    ok = int_value(fields, "ok")
    loops_ok = (loops is not None) and (ok == loops)
    if status_ok and loops_ok:
        mode = fields.get("mode", "?")
        length = fields.get("len", "?")
        bps = fields.get("bps", "?")
        return CheckResult("PASS", f"mode={mode} len={length} bps={bps}")
    return CheckResult("FAIL", line)


def run_command(
    reader: LineReader,
    command: CommandSpec,
    min_stack_words: int,
    show_noise: bool,
    raw_log: Optional[TextIO],
) -> CheckResult:
    print(f"\n> {command.text}")
    reader.transport.write_line(command.text)

    matched: List[str] = []
    ignored = 0
    deadline = time.monotonic() + command.timeout_s
    last_match_time: Optional[float] = None

    while time.monotonic() < deadline:
        lines = reader.poll()
        if not lines:
            if last_match_time is not None and (time.monotonic() - last_match_time) >= command.settle_s:
                break
            continue

        for line in lines:
            if raw_log is not None:
                raw_log.write(f"{time.time():.3f} {line}\n")
                raw_log.flush()

            if is_expected_line(command.kind, line):
                matched.append(line)
                last_match_time = time.monotonic()
                print(f"  {line}")
            else:
                ignored += 1
                if show_noise and not is_noise_line(line):
                    print(f"  [ignored] {line}")

        if last_match_time is not None and (time.monotonic() - last_match_time) >= command.settle_s:
            break

    result = evaluate(command.kind, matched, min_stack_words)
    suffix = f" filtered={ignored}" if ignored else ""
    print(f"  {result.status}: {result.detail}{suffix}")
    return result


def drain_input(reader: LineReader, seconds: float, raw_log: Optional[TextIO]) -> int:
    deadline = time.monotonic() + seconds
    count = 0
    while time.monotonic() < deadline:
        for line in reader.poll():
            count += 1
            if raw_log is not None:
                raw_log.write(f"{time.time():.3f} {line}\n")
        time.sleep(0.01)
    if raw_log is not None:
        raw_log.flush()
    return count


def build_commands(args: argparse.Namespace) -> List[CommandSpec]:
    if args.cmd:
        texts = args.cmd
    else:
        texts = ["RTOS?", "FLASH?"]
        if args.scratch:
            texts.append("FLASH SCRATCH?")
        texts.extend(VERIFY_COMMANDS[:3] if args.quick else VERIFY_COMMANDS)
        if args.bench:
            texts.extend(BENCH_COMMANDS)
        if args.final_rtos:
            texts.append("RTOS?")

    return [
        CommandSpec(
            text=text.strip(),
            kind=infer_kind(text),
            timeout_s=args.timeout,
            settle_s=args.settle,
        )
        for text in texts
        if text.strip()
    ]


def list_ports() -> int:
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0

    for port in ports:
        print(f"{port.device:12} {port.description}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Filter noisy UART output and run drone-H743 RTOS/flash diagnostics."
    )
    parser.add_argument("--serial", default=os.environ.get("DRONE_SERIAL_PORT", DEFAULT_SERIAL_PORT))
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument("--drain", type=float, default=0.5, help="seconds to discard old UART data before testing")
    parser.add_argument("--min-stack-words", type=int, default=DEFAULT_MIN_STACK_WORDS)
    parser.add_argument("--bench", action="store_true", help="also run FLASH BENCH READ commands")
    parser.add_argument("--quick", action="store_true", help="only run the first three FLASH VERIFY ranges")
    parser.add_argument("--scratch", action="store_true", help="also query the reserved scratch sector")
    parser.add_argument("--final-rtos", action="store_true", help="query RTOS? again after flash tests")
    parser.add_argument("--cmd", action="append", help="custom command; repeat for multiple commands")
    parser.add_argument("--show-noise", action="store_true", help="show non-matching board lines")
    parser.add_argument("--raw-log", help="write all decoded board lines to this file")
    parser.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_ports:
        return list_ports()

    if not args.serial:
        print("Missing --serial. Use --list-ports to find the COM port.", file=sys.stderr)
        return 2

    raw_log: Optional[TextIO] = None
    if args.raw_log:
        raw_log = open(args.raw_log, "a", encoding="utf-8", newline="\n")

    transport: Optional[SerialTransport] = None
    try:
        transport = SerialTransport(args.serial, args.baud)
        reader = LineReader(transport)
        print(f"Opened {args.serial} @ {args.baud}. Close other serial terminals first.")

        if args.drain > 0:
            drained = drain_input(reader, args.drain, raw_log)
            print(f"Drained {drained} stale/noise lines.")

        counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
        for command in build_commands(args):
            result = run_command(reader, command, args.min_stack_words, args.show_noise, raw_log)
            counts[result.status] = counts.get(result.status, 0) + 1
            time.sleep(0.05)

        print(
            "\nSummary: "
            f"pass={counts.get('PASS', 0)} "
            f"warn={counts.get('WARN', 0)} "
            f"fail={counts.get('FAIL', 0)}"
        )
        return 1 if counts.get("FAIL", 0) else 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    finally:
        if transport is not None:
            transport.close()
        if raw_log is not None:
            raw_log.close()


if __name__ == "__main__":
    sys.exit(main())
