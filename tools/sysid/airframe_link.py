"""机体参数的唯一入口 —— 复用 `panel_lib/airframe_model` 的字段表与派生公式。

**不在这里再抄一份字段名。** `drone_tcp_panel.py` 里曾经有过第二副 `airframe_info`
词汇表，和真表对不上，于是界面上的数和飞控里的数各说各的。辨识如果再抄第三份，
结论会落在一组根本没在飞的几何上。

本模块只做三件事：
  1. 把 `PARAM?` 的回显行解析成 `airframe.*` 快照；
  2. 用同一份 `compute_derived` 补齐派生量（质量、质心、重量、最大推力…）；
  3. 给辨识需要的几个物理量提供有名字的出口（惯量对角、IMU→CG 偏移）。
"""
from __future__ import annotations

import sys
from pathlib import Path

_TOOLS = Path(__file__).resolve().parents[1]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from panel_lib.airframe_model import (  # noqa: E402
    AIRFRAME_FIELD_BY_NAME,
    INPUT_FIELDS,
    compute_derived,
    first_missing,
)

PREFIX = "airframe."


class AirframeIncomplete(ValueError):
    """机体模型没填全。力臂为 0 时力矩恒为 0，辨识只会记下一串零。"""


def parse_param_lines(lines: list[str]) -> dict[str, float]:
    """从 `PARAM?` 的回显里抓出 `airframe.*`。

    接受两种常见形态：`airframe.ixx_kgm2=0.019` 和 `airframe.ixx_kgm2 0.019`。
    不认识的名字直接忽略——固件加了字段而主机还没更新时，宁可少读一个，
    也不要因为一行没见过的文本就整份放弃。
    """
    values: dict[str, float] = {}
    for line in lines:
        text = line.strip()
        if not text.startswith(PREFIX):
            continue
        parts = text.split()
        head = parts[0]
        if "=" in head:
            name, _, raw = head.partition("=")
        elif len(parts) >= 2:
            name, raw = head, parts[1]
        else:
            continue
        # 表的键是**带前缀**的全名（`airframe.ixx_kgm2`），而 `compute_derived`
        # 与 `first_missing` 用的是不带前缀的短名。两种写法都来自同一份表，
        # 这里只做转换，不另立第三种叫法。
        if name not in AIRFRAME_FIELD_BY_NAME:
            continue
        short = name[len(PREFIX):]
        try:
            values[short] = float(raw)
        except ValueError:
            continue
    return values


def complete(values: dict[str, float]) -> dict[str, float]:
    """补齐派生量。用的是界面和固件同一份公式。"""
    merged = dict(values)
    merged.update(compute_derived(merged))
    return merged


def require_complete(values: dict[str, float]) -> dict[str, float]:
    missing = first_missing(values)
    if missing is not None:
        raise AirframeIncomplete(
            f"机体模型缺 {missing}；先在「机体参数」页填完再辨识——"
            "力臂为 0 时力矩恒为 0，这一趟只会记下一串零")
    return complete(values)


def inertia_diag(values: dict[str, float]) -> tuple[float, float, float]:
    """惯量对角 [Ixx, Iyy, Izz]。绕 45° 杆轴的有效惯量恒等于 Ixx。"""
    return (float(values.get("ixx_kgm2", 0.0)),
            float(values.get("iyy_kgm2", 0.0)),
            float(values.get("izz_kgm2", 0.0)))


def imu_above_cg_m(values: dict[str, float]) -> float:
    """IMU 相对质心的垂直偏移，`imu_z_m - cg_z_m`。

    作者点名要的那个物理条件。角速度不受它影响（刚体不变量），
    但加速度计会多出 ω̇×r 与 ω×(ω×r)——杆过质心时那正是姿态角误差的主要来源。
    """
    complete_values = complete(values)
    return float(complete_values.get("imu_z_m", 0.0)) - float(
        complete_values.get("cg_z_m", 0.0))


def snapshot(values: dict[str, float]) -> dict[str, float]:
    """存进辨识档案的快照：输入项 + 派生项，一个不落（短名为键）。

    存全份而不是"辨识用得到的那几个"：半年后回答"这组 PID 是在哪个机体上辨的"
    要的是完整几何，而当时觉得用不到的那几个数往往正是解释差异的那个。
    """
    complete_values = complete(values)
    keep = {short_name(f) for f in INPUT_FIELDS} | set(complete_values)
    return {name: float(complete_values[name]) for name in sorted(keep)
            if name in complete_values}


def short_name(field) -> str:
    """字段表里的全名 -> `compute_derived` 用的短名。"""
    return field.name[len(PREFIX):] if field.name.startswith(PREFIX) else field.name


def param_commands(values: dict[str, float]) -> list[str]:
    """把一份快照写回飞控的命令行（**只写 RAM，不含 SAVE**）。"""
    out = []
    for field in INPUT_FIELDS:
        short = short_name(field)
        if short in values:
            out.append(f"PARAM SET {field.name} {values[short]:.6g}")
    return out
