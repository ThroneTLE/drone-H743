"""台架设置的持久化：杆轴方向、杆到飞控板距离、程序油门、模式、舵机摆幅、挂砝码试验、
高度（ALT）分区——下次打开不用重填。

* 存在 `data/identification/attitude/rig_settings.json`（`project_paths.ATTITUDE_IDENT_DIR`），
  UTF-8 JSON，带 `"version": 1`。
* 只在「开始辨识」通过输入检查、开始发命令的那一刻保存一次，不随按键写盘；挂砝码试验
  另在分析用到它时单独记一次（`save_stiffness`：只改那几项，其余照旧）。
* 读的时候逐项体检：文件不存在/损坏/某项非法都回落默认值，不弹窗，只在状态行提示一次。
* 写盘用同目录临时文件 + `os.replace`，不会留下写了一半的文件；失败只提示，不影响开跑。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from uuid import uuid4

from .alt_config import EXTRA_MASS_RANGE_G, INJECTS, LIFT_RANGE_MM, WIN_RANGE_MM
from .geometry import LENGTH_LIMIT_M

SETTINGS_VERSION = 1
FILENAME = "rig_settings.json"
#: 顺序就是固件的模式编号（FF 0 … SERVO 3、ALT 4），只许在末尾追加；旧文件里的模式照样认。
MODES = ("FF", "RATE", "ANGLE", "SERVO", "ALT")
#: 高度（ALT）分区的数值项 -> 页面变量名（都可空，空就不记）。
_ALT_VARS = dict(alt_extra_mass_g="alt_extra_mass_var", alt_win_mm="alt_win_var",
                 alt_lift_mm="alt_lift_var")
#: 挂砝码试验的五项（都可空）：砝码 g、水平距离 m、挂两侧/不挂时的姿态读数 deg（按杆轴方位
#: 读俯仰或横滚，键名沿用 front/back）。
STIFFNESS_KEYS = ("stiffness_weight_g", "stiffness_distance_m", "stiffness_front_deg",
                  "stiffness_back_deg", "stiffness_level_deg")
_STIFFNESS_VARS = dict(stiffness_weight_g="stiffness_weight_var",
                       stiffness_distance_m="stiffness_distance_var",
                       stiffness_front_deg="stiffness_front_var",
                       stiffness_back_deg="stiffness_back_var",
                       stiffness_level_deg="stiffness_level_var")


def settings_path() -> Path:
    try:
        from ....project_paths import ATTITUDE_IDENT_DIR
    except ImportError:
        try:
            from tools.project_paths import ATTITUDE_IDENT_DIR
        except ImportError:
            from project_paths import ATTITUDE_IDENT_DIR
    return Path(ATTITUDE_IDENT_DIR) / FILENAME


def _number(value, low: float, high: float, *, allow_none: bool = False,
            open_low: bool = False):
    """合法返回数值（或 None），非法抛 ValueError。bool 不算数字。"""
    if value is None and allow_none:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError
    number = float(value)
    if not math.isfinite(number) or number > high or number < low or (open_low and number == low):
        raise ValueError
    return number


def _reject():
    raise ValueError


def _flag(value):
    if not isinstance(value, bool):
        raise ValueError
    return value


def _mode(value):
    if value not in MODES:
        raise ValueError
    return value


#: 字段 -> 体检函数。范围与页面输入检查一致或更宽，只挡明显的坏值。
_CHECKS = {
    "psi_deg": lambda v: _number(v, -180.0, 180.0),
    "rod_to_fc_m": lambda v: _number(v, -LENGTH_LIMIT_M, LENGTH_LIMIT_M, allow_none=True),
    "axis_override_m": lambda v: _number(v, -LENGTH_LIMIT_M, LENGTH_LIMIT_M, allow_none=True),
    "roll_pivot_to_fc_m": lambda v: _number(v, -LENGTH_LIMIT_M, LENGTH_LIMIT_M, allow_none=True),
    "pitch_pivot_to_fc_m": lambda v: _number(v, -LENGTH_LIMIT_M, LENGTH_LIMIT_M, allow_none=True),
    "target_thrust_n": lambda v: _number(v, 0.0, 1000.0, allow_none=True, open_low=True),
    "max_throttle_pct": lambda v: _number(v, 10.0, 95.0),
    "manual_throttle": _flag,
    "mode": _mode,
    "angle_amp_deg": lambda v: _number(v, 0.0, 15.0, open_low=True),
    "experiment": lambda v: v if v in ("doublet", "chirp") else _reject(),
    "servo_tilt_deg": lambda v: _number(v, 0.5, 15.1),
    "stiffness_weight_g": lambda v: _number(v, 0.0, 10000.0, allow_none=True, open_low=True),
    "stiffness_distance_m": lambda v: _number(v, -1.0, 1.0, allow_none=True),
    "stiffness_front_deg": lambda v: _number(v, -90.0, 90.0, allow_none=True),
    "stiffness_back_deg": lambda v: _number(v, -90.0, 90.0, allow_none=True),
    "stiffness_level_deg": lambda v: _number(v, -90.0, 90.0, allow_none=True),
    "alt_inject": lambda v: v if v in INJECTS else _reject(),
    "alt_extra_mass_g": lambda v: _number(v, *EXTRA_MASS_RANGE_G),
    "alt_win_mm": lambda v: _number(v, *WIN_RANGE_MM),
    "alt_lift_mm": lambda v: _number(v, *LIFT_RANGE_MM),
}


def load(path: Path | None = None) -> tuple[dict, str]:
    """`(合法字段, 给状态行的一句提示)`；提示为空表示一切正常。"""
    path = path or settings_path()
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return {}, "首次使用：台架设置用默认值，点「开始辨识」时会自动记住。"
    except OSError as error:
        return {}, f"台架设置读不出来（{error}），已用默认值。"
    try:
        data = json.loads(text)
        if not isinstance(data, dict) or data.get("version") != SETTINGS_VERSION:
            raise ValueError
    except ValueError:
        return {}, "台架设置文件损坏或版本不对，已用默认值；下次开始辨识时会重新保存。"
    values, rejected = {}, []
    for key, check in _CHECKS.items():
        if key not in data:
            continue
        try:
            values[key] = check(data[key])
        except ValueError:
            rejected.append(key)
    note = (f"台架设置里有 {len(rejected)} 项无效（{', '.join(rejected)}），这几项已用默认值。"
            if rejected else "")
    return values, note


def save(values: dict, path: Path | None = None) -> Path:
    path = path or settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"version": SETTINGS_VERSION}
    payload.update({key: values[key] for key in _CHECKS if key in values})
    temporary = path.with_name(f"{path.name}.tmp-{uuid4().hex[:8]}")
    try:
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return path


def _text(value) -> str:
    return "" if value is None else f"{value:g}"


def apply_to_page(page, values: dict) -> None:
    if "psi_deg" in values:
        page.psi_var.set(_text(values["psi_deg"]))
    if "rod_to_fc_m" in values:
        page.rod_to_fc_var.set(_text(values["rod_to_fc_m"]))
    if "axis_override_m" in values:
        page.axis_override_var.set(_text(values["axis_override_m"]))
    if "roll_pivot_to_fc_m" in values:
        page.roll_pivot_var.set(_text(values["roll_pivot_to_fc_m"]))
    if "pitch_pivot_to_fc_m" in values:
        page.pitch_pivot_var.set(_text(values["pitch_pivot_to_fc_m"]))
    if "target_thrust_n" in values:
        page.target_thrust_var.set(_text(values["target_thrust_n"]))
    if "max_throttle_pct" in values:
        page.max_pct_var.set(_text(values["max_throttle_pct"]))
    if "manual_throttle" in values:
        page.manual_throttle_var.set(values["manual_throttle"])
    if "mode" in values:
        page.mode_var.set(values["mode"])
    if "angle_amp_deg" in values:
        page.angle_amp_var.set(_text(values["angle_amp_deg"]))
    if "experiment" in values:
        page.set_experiment(values["experiment"])
    if "servo_tilt_deg" in values and hasattr(page, "servo_tilt_var"):
        page.servo_tilt_var.set(_text(values["servo_tilt_deg"]))
    for key, name in _STIFFNESS_VARS.items():
        if key in values and hasattr(page, name):
            getattr(page, name).set(_text(values[key]))
    if "alt_inject" in values and hasattr(page, "alt_inject_var"):
        page.alt_inject_var.set(values["alt_inject"])
    for key, name in _ALT_VARS.items():
        if key in values and hasattr(page, name):
            getattr(page, name).set(_text(values[key]))
    # 上面的「实验类型」会把角速度预设写进激励编排；ALT 轮要换回所选注入类型的默认激励。
    if values.get("mode") == "ALT" and hasattr(page, "apply_alt_inject"):
        page.apply_alt_inject()


def collect_from_page(page) -> dict:
    """只在开始前的输入检查通过之后调用，所以这里的解析不会失败。"""
    def optional(text):
        text = text.strip()
        return float(text) if text else None

    return {
        "psi_deg": float(page.psi_var.get()),
        "rod_to_fc_m": optional(page.rod_to_fc_var.get()),
        "axis_override_m": optional(page.axis_override_var.get()),
        "roll_pivot_to_fc_m": optional(page.roll_pivot_var.get()),
        "pitch_pivot_to_fc_m": optional(page.pitch_pivot_var.get()),
        "target_thrust_n": optional(page.target_thrust_var.get()),
        "max_throttle_pct": float(page.max_pct_var.get()),
        "manual_throttle": bool(page.manual_throttle_var.get()),
        "mode": page.mode_var.get(),
        "angle_amp_deg": float(page.angle_amp_var.get()),
        "experiment": page.experiment_key(),
        **_lenient(page, {"servo_tilt_deg": "servo_tilt_var", **_STIFFNESS_VARS, **_ALT_VARS}),
        **_alt_inject(page),
    }


def _alt_inject(page) -> dict:
    variable = getattr(page, "alt_inject_var", None)
    value = variable.get() if variable is not None else None
    return {"alt_inject": value} if value in INJECTS else {}


def _lenient(page, names: dict) -> dict:
    """可选项：填了合法数字才记，空或乱填就不记（开始前的输入检查不管它们）。"""
    out = {}
    for key, name in names.items():
        variable = getattr(page, name, None)
        if variable is None:
            continue
        text = str(variable.get()).strip()
        try:
            out[key] = _CHECKS[key](float(text)) if text else None
        except ValueError:
            continue
    for key in ("servo_tilt_deg", *_ALT_VARS):     # 这几项读的时候不认 null：空就不记
        if key in out and out[key] is None:
            del out[key]
    return out


def save_stiffness(page, path: Path | None = None) -> Path:
    """只更新挂砝码试验那几项，其余照文件里原样保留（文件没有或坏了就只写这几项）。"""
    path = path or settings_path()
    values, _note = load(path)
    values.update(_lenient(page, _STIFFNESS_VARS))
    return save(values, path)


__all__ = ["FILENAME", "MODES", "SETTINGS_VERSION", "STIFFNESS_KEYS", "apply_to_page",
           "collect_from_page", "load", "save", "save_stiffness", "settings_path"]
