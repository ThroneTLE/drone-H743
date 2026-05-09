#!/usr/bin/env python3
"""Decode SPI bytes from a Saleae raw digital.csv export.

Default mapping follows the current Saleae hookup:
  Channel 0 = CS, Channel 1 = SCK, Channel 2 = MOSI, Channel 3 = MISO
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("csv", type=Path)
    p.add_argument("--cs", type=int, default=0)
    p.add_argument("--sck", type=int, default=1)
    p.add_argument("--mosi", type=int, default=2)
    p.add_argument("--miso", type=int, default=3)
    p.add_argument("--mode", type=int, choices=[0, 3], default=0)
    return p.parse_args()


def load_rows(path: Path) -> list[list[int | float]]:
    with path.open("r", newline="") as f:
        r = csv.reader(f)
        next(r)
        rows = []
        for row in r:
            if row:
                rows.append([float(row[0])] + [int(v) for v in row[1:]])
    return rows


def decode(rows: list[list[int | float]], args: argparse.Namespace) -> None:
    cs_i = args.cs + 1
    sck_i = args.sck + 1
    mosi_i = args.mosi + 1
    miso_i = args.miso + 1
    # SPI mode 0 and mode 3 both sample on the rising edge.
    # Mode 0: idle low, first edge rising. Mode 3: idle high, second edge rising.
    sample_on_rising = args.mode in (0, 3)

    transactions = []
    active = False
    bits_mosi = []
    bits_miso = []
    start_t = 0.0
    prev = rows[0]

    for row in rows[1:]:
        prev_cs = int(prev[cs_i])
        cur_cs = int(row[cs_i])
        prev_sck = int(prev[sck_i])
        cur_sck = int(row[sck_i])

        if (not active) and prev_cs == 1 and cur_cs == 0:
            active = True
            start_t = float(row[0])
            bits_mosi = []
            bits_miso = []
        elif active and prev_cs == 0 and cur_cs == 1:
            transactions.append((start_t, float(row[0]), bits_mosi[:], bits_miso[:]))
            active = False

        if active:
            edge = (prev_sck == 0 and cur_sck == 1) if sample_on_rising else (prev_sck == 1 and cur_sck == 0)
            if edge:
                bits_mosi.append(int(row[mosi_i]))
                bits_miso.append(int(row[miso_i]))

        prev = row

    if active:
        transactions.append((start_t, float(rows[-1][0]), bits_mosi[:], bits_miso[:]))

    print(f"transactions={len(transactions)} mode={args.mode}")
    for i, (start, end, mosi_bits, miso_bits) in enumerate(transactions[:40]):
        mosi_bytes = bits_to_bytes(mosi_bits)
        miso_bytes = bits_to_bytes(miso_bits)
        print(
            f"tx#{i} {start:.9f}-{end:.9f}s bits={len(mosi_bits)} "
            f"MOSI={fmt_bytes(mosi_bytes)} MISO={fmt_bytes(miso_bytes)}"
        )


def bits_to_bytes(bits: list[int]) -> list[int]:
    out = []
    for i in range(0, len(bits), 8):
        chunk = bits[i:i + 8]
        if len(chunk) < 8:
            break
        value = 0
        for bit in chunk:
            value = (value << 1) | bit
        out.append(value)
    return out


def fmt_bytes(values: list[int]) -> str:
    return " ".join(f"{v:02X}" for v in values) if values else "-"


def main() -> None:
    args = parse_args()
    rows = load_rows(args.csv)
    if len(rows) < 2:
        print("not enough rows")
        return
    decode(rows, args)


if __name__ == "__main__":
    main()
