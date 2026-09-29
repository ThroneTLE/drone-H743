"""Write a thrust lookup table into firmware: Driver/Src/drv_thrust_lut_table.inc.

    python -m tools.thrust_bench.thrust_lut_export [path/to/thrust_lut.json]

Exporting is what makes a table current: models/lut/current.json then names it, and
that is the only table the flight controller runs. Without an argument the current table
is re-exported; a newer build is a candidate until it is validated and exported by path.
The same-throttle diagonal is sampled here with the host reader, so the firmware
inverse is a 1D interpolation over identical numbers (tests compare both sides).
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from . import thrust_lut
from .sweep_schedule import SAG_K_DEFAULT

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "Driver" / "Src" / "drv_thrust_lut_table.inc"
diagonal = thrust_lut.balanced_diagonal


def usable_charge(lut: dict[str, Any]) -> list[float]:
    low, high = lut["domain"]["charge_v"]
    return list(lut["domain"].get("charge_v_usable", thrust_lut.usable_range(low, high)))


def _grid(name: str, table: dict[str, Any]) -> tuple[str, str]:
    ax, ay = (np.asarray(axis, float) for axis in table["axes"])
    for axis in (ax, ay):
        if not np.allclose(np.diff(axis), axis[1] - axis[0]):
            raise ValueError("固件只支持等间距网格")
    values = np.asarray(table["values"], float)
    rows = ",\n".join("    " + ", ".join(f"{v:.2f}f" for v in row) for row in values)
    data = f"static const float lut_{name}_values[{values.size}] = {{\n{rows},\n}};\n"
    body = (f"{{{ax[0]:.1f}f, {ax[1] - ax[0]:.1f}f, {len(ax)}U, {ay[0]:.1f}f, {ay[1] - ay[0]:.1f}f, {len(ay)}U, "
            f"lut_{name}_values}}")
    return data, body


def _floats(values: list[float], per_line: int = 10) -> str:
    return ",\n".join("    " + ", ".join(f"{v:.3f}f" for v in values[i:i + per_line])
                      for i in range(0, len(values), per_line))


def render(lut: dict[str, Any], source: str) -> str:
    effective, thrust = diagonal(lut)
    speed_data, speed = _grid("speed", lut["speed"])
    effective_data, effective_grid = _grid("effective", lut["throttle"])
    low, high = lut["domain"]["charge_v"]
    use_low, use_high = usable_charge(lut)
    k = float(lut.get("charge_voltage", {}).get("k", SAG_K_DEFAULT))
    sessions = ", ".join(item["session"] for item in lut["training"]["sources"])
    return (f"/*\n * 生成文件，勿手改：python -m tools.thrust_bench.thrust_lut_export\n"
            f" * 来源 {source}\n"
            f" * 查补表 {lut['model_id']}：{lut['training']['points']} 个稳态点（{sessions}），"
            f"实测电量 {low:.2f}~{high:.2f} V，补偿可用 {use_low:.2f}~{use_high:.2f} V。\n */\n"
            '#include "drv_thrust_lut.h"\n\n'
            f"{speed_data}\n{effective_data}\n"
            f"static const float lut_diag_effective[{len(effective)}] = {{\n{_floats(effective)},\n}};\n\n"
            f"static const float lut_diag_thrust_g[{len(thrust)}] = {{\n{_floats(thrust)},\n}};\n\n"
            "const DRV_ThrustLutTable DRV_ThrustLut_Table = {\n"
            f'    "{lut["model_id"]}",\n'
            f"    {float(lut.get('v_ref', thrust_lut.V_REF)):.1f}f,\n"
            f"    {k:.5f}f,\n"
            f"    {{{low:.3f}f, {high:.3f}f}},\n"
            f"    {{{use_low:.3f}f, {use_high:.3f}f}},\n"
            f"    {speed},\n"
            f"    {effective_grid},\n"
            f"    {len(effective)}U,\n"
            "    lut_diag_effective,\n"
            "    lut_diag_thrust_g,\n"
            "};\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("source", nargs="?", type=Path)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    from tools import project_paths
    root = project_paths.THRUST_IDENT_DIR
    found = thrust_lut.current(root) if args.source is None else None
    if args.source is None and found is None:
        found = thrust_lut.latest(root)
    if args.source is None:
        if found is None:
            print("还没有查补表：先在推力台点“建立查补表”", file=sys.stderr)
            return 1
        lut, path = found
    else:
        path = args.source
        lut = json.loads(path.read_text(encoding="utf-8"))
    try:
        source = path.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        source = path.name
    args.output.write_text(render(lut, source), encoding="utf-8", newline="\n")
    if args.output.resolve() == OUTPUT.resolve():
        thrust_lut.set_current(root, lut, source, OUTPUT.relative_to(ROOT).as_posix(), datetime.now().astimezone().isoformat())
    print(f"已写入 {args.output}（查补表 {lut['model_id']}，已登记为当前表）；重新编译并烧录固件后生效")
    newest = thrust_lut.latest(root)
    if newest is not None and newest[0].get("model_id") != lut["model_id"]:
        print(f"注意：还有更新的候选表 {newest[0]['model_id']}（{newest[1]}），未写入飞控")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
