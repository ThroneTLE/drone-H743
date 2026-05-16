#!/usr/bin/env python3
"""RS485 Modbus-RTU test tool for the 4-channel weighing/pressure transmitter.

Manual notes used here:
- Protocol: Modbus RTU, 8 data bits, no parity, 1 stop bit.
- Baud rate can be set by DIP switch; user requested 9600.
- DIP station address uses weights 1, 2, 4, 8. DIP "1000" therefore means
  station address 1 when the first switch is the 1x bit.
- Channel 1 realtime value is holding register 0x0000/0x0001, signed 32-bit,
  low word first.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

try:
    import serial
except ImportError as exc:  # pragma: no cover - user environment hint
    raise SystemExit("pyserial is required: pip install pyserial") from exc


FUNC_READ_HOLDING = 0x03
FUNC_WRITE_SINGLE = 0x06
DEFAULT_BAUD = 9600
DEFAULT_DIP = "1000"
DEFAULT_TIMEOUT = 0.25


@dataclass
class ReadResult:
    addr: int
    register: int
    words: list[int]
    raw: bytes


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def add_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + struct.pack("<H", crc)


def check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    received = frame[-2] | (frame[-1] << 8)
    return crc16_modbus(frame[:-2]) == received


def dip_to_addr(dip: str) -> int:
    cleaned = "".join(ch for ch in dip if ch in "01")
    if len(cleaned) != 4:
        raise ValueError("DIP must contain exactly four 0/1 bits, for example 1000")
    return sum((1 << i) for i, bit in enumerate(cleaned) if bit == "1")


def parse_u16(value: str) -> int:
    return int(value, 0)


def txrx(
    ser: serial.Serial,
    request: bytes,
    expected_min: int,
    *,
    log=None,
) -> bytes:
    ser.reset_input_buffer()
    ser.write(request)
    ser.flush()

    deadline = time.monotonic() + float(ser.timeout or DEFAULT_TIMEOUT)
    data = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(max(1, expected_min - len(data)))
        if chunk:
            data.extend(chunk)
            if len(data) >= expected_min:
                break

    response = bytes(data)
    if log is not None:
        log(request, response)
    return response


def read_holding_registers(
    ser: serial.Serial,
    addr: int,
    start_register: int,
    count: int,
    *,
    verbose: bool = False,
    log=None,
) -> ReadResult:
    request = add_crc(bytes([
        addr & 0xFF,
        FUNC_READ_HOLDING,
        (start_register >> 8) & 0xFF,
        start_register & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ]))
    expected = 5 + count * 2
    response = txrx(ser, request, expected, log=log)

    if verbose:
        print(f"TX {request.hex(' ').upper()}")
        print(f"RX {response.hex(' ').upper() if response else '(timeout)'}")

    if len(response) < expected:
        raise TimeoutError(f"timeout/short response: expected {expected} bytes, got {len(response)}")
    if not check_crc(response):
        raise ValueError(f"bad CRC: {response.hex(' ').upper()}")
    if response[0] != addr:
        raise ValueError(f"unexpected slave address: got {response[0]}, expected {addr}")
    if response[1] & 0x80:
        raise RuntimeError(f"modbus exception: function=0x{response[1]:02X}, code=0x{response[2]:02X}")
    if response[1] != FUNC_READ_HOLDING:
        raise ValueError(f"unexpected function: 0x{response[1]:02X}")
    if response[2] != count * 2:
        raise ValueError(f"unexpected byte count: {response[2]}")

    words = []
    payload = response[3:3 + response[2]]
    for index in range(0, len(payload), 2):
        words.append((payload[index] << 8) | payload[index + 1])

    return ReadResult(addr=addr, register=start_register, words=words, raw=response)


def write_single_register(
    ser: serial.Serial,
    addr: int,
    register: int,
    value: int,
    *,
    verbose: bool = False,
    log=None,
) -> bytes:
    request = add_crc(bytes([
        addr & 0xFF,
        FUNC_WRITE_SINGLE,
        (register >> 8) & 0xFF,
        register & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ]))
    response = txrx(ser, request, 8, log=log)

    if verbose:
        print(f"TX {request.hex(' ').upper()}")
        print(f"RX {response.hex(' ').upper() if response else '(timeout)'}")

    if len(response) < 8:
        raise TimeoutError(f"timeout/short response: expected 8 bytes, got {len(response)}")
    if not check_crc(response):
        raise ValueError(f"bad CRC: {response.hex(' ').upper()}")
    if response[:6] != request[:6]:
        raise ValueError(f"unexpected write echo: {response.hex(' ').upper()}")
    return response


def words_to_signed32_low_word_first(words: list[int]) -> int:
    if len(words) < 2:
        raise ValueError("need two registers for signed 32-bit value")
    value = (words[1] << 16) | words[0]
    if value & 0x80000000:
        value -= 0x100000000
    return value


def read_channel_weight(
    ser: serial.Serial,
    addr: int,
    channel: int,
    *,
    verbose: bool = False,
    log=None,
) -> int:
    if channel < 1:
        raise ValueError("channel must start from 1")
    start_register = (channel - 1) * 2
    result = read_holding_registers(ser, addr, start_register, 2, verbose=verbose, log=log)
    return words_to_signed32_low_word_first(result.words)


def scan_addresses(ser: serial.Serial, start: int, end: int, *, verbose: bool = False) -> list[int]:
    found = []
    for addr in range(start, end + 1):
        try:
            read_holding_registers(ser, addr, 0x0000, 2, verbose=verbose)
            found.append(addr)
            print(f"FOUND addr={addr}")
        except Exception:
            pass
    return found


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read channel-1 pressure/weight from the RS485 Modbus transmitter."
    )
    parser.add_argument("port", help="Serial port, e.g. COM31")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Default: 9600")
    parser.add_argument("--dip", default=DEFAULT_DIP, help="4-bit address DIP, default: 1000 -> addr 1")
    parser.add_argument("--addr", type=parse_u16, default=None, help="Override Modbus station address")
    parser.add_argument("--channel", type=int, default=1, help="Sensor channel, default: 1")
    parser.add_argument("--interval", type=float, default=0.2, help="Loop read interval seconds")
    parser.add_argument("--count", type=int, default=0, help="Read count, 0 means forever")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Serial timeout seconds")
    parser.add_argument("--scan", action="store_true", help="Scan addresses 1..15 before reading")
    parser.add_argument("--scan-start", type=int, default=1)
    parser.add_argument("--scan-end", type=int, default=15)
    parser.add_argument("--raw", action="store_true", help="Print TX/RX frames")
    parser.add_argument("--tare", action="store_true", help="Write 2 to channel command register for tare")
    parser.add_argument("--zero", action="store_true", help="Write 1 to channel command register for zero calibration")
    return parser


def main(argv: list[str]) -> int:
    args = build_parser().parse_args(argv)
    addr = args.addr if args.addr is not None else dip_to_addr(args.dip)
    command_register = 0x0028 + (args.channel - 1) * 10

    print("Pressure/weight RS485 Modbus test")
    print(f"port={args.port} baud={args.baud} addr={addr} dip={args.dip} channel={args.channel}")
    print("format: channel realtime value is signed int32, low word first")

    with serial.Serial(
        args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=args.timeout,
        write_timeout=args.timeout,
    ) as ser:
        if args.scan:
            found = scan_addresses(ser, args.scan_start, args.scan_end, verbose=args.raw)
            print(f"scan result: {found}")
            if not found:
                return 2
            if args.addr is None:
                addr = found[0]
                print(f"using first found addr={addr}")

        if args.zero:
            print(f"zero channel {args.channel}: write reg 0x{command_register:04X}=1")
            write_single_register(ser, addr, command_register, 1, verbose=args.raw)

        if args.tare:
            print(f"tare channel {args.channel}: write reg 0x{command_register:04X}=2")
            write_single_register(ser, addr, command_register, 2, verbose=args.raw)

        index = 0
        while args.count == 0 or index < args.count:
            try:
                value = read_channel_weight(ser, addr, args.channel, verbose=args.raw)
                now = time.strftime("%H:%M:%S")
                print(f"{now} ch{args.channel}={value}")
            except KeyboardInterrupt:
                print()
                return 0
            except Exception as exc:
                print(f"ERR {exc}")

            index += 1
            time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
