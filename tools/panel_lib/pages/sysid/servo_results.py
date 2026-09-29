"""舵机单独轮的结论：反作用惯量、舵机二阶、延迟，折算成带桨轮的 ρ，并与同组带桨拟合对照。

不碰 Tk。读存档（同一天、同杆轴方向、同杆高的"一组"）是为了两件事：
* 找同组最近一次带桨 TWD 拟合，把它的 ρ 与这里预测的 ρ_pred 放在一起（只作交叉核对，不钉死）；
  它的杆摆频则拿来钉死本轮的 I_杆（`group_pendulum_hz`：舵机单独激不起杆摆）；
* 找同组其它舵机单独轮，幅值不同就报 J/延迟随幅值的变化与回差估计（方法见 `servo_fit.BACKLASH_METHOD`）。
  只和舵机回差补偿开关相同的轮次放一起（conditions 的 backlash）：补偿过的舵机回差应接近 0，
  和没补的混在一起拟，两边都不对。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from . import _core, settings_store

SERVO_FIT_FILE = "fit_servo.json"
GRAVITY_M_S2 = 9.80665


@dataclass
class ServoOutcome:
    fit: object
    #: 本轮固件沿杆轴的倾转力臂 |L| [m] 与悬停推力（机重）[N]；读不到 None。
    lever_m: float | None
    hover_thrust_n: float | None
    #: ρ_pred = −J/(|L|·T_悬停)（本轮固件力矩单位）[s²]。
    predicted_rho_s2: float
    #: 同组最近一次带桨 TWD 拟合的对照；没有时 None。
    comparison: dict | None = None
    #: 幅值依赖（同组 ≥ 2 个不同幅值时）；没有时 None。
    amplitude: object | None = None
    #: 静态偏心 H 的量级参考 (组件质量 kg, 质心到舵机轴 m, m·g·r)；读不到 None。
    pod_reference: tuple | None = None
    #: 钉死摆频用的 (Hz, 同组带桨拟合目录名)；没钉 None（见 `group_pendulum_hz`）。
    pendulum_pin: tuple | None = None
    blocks: dict = field(default_factory=dict)

    @property
    def warnings(self) -> list[str]:
        return list(getattr(self.fit, "warnings", []) or [])

    @property
    def structure(self) -> str:
        return "servo"


def _float(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def group_key(folder: Path, conditions: dict) -> tuple:
    """同一组：同一天、同杆轴方向、同杆高（推力与模式不限）。"""
    return (folder.parent.name, str(conditions.get("psi_mrad")), str(conditions.get("axis_off_um")))


def _archived(root: Path):
    for path in root.glob("*/rod_*/conditions.json"):
        try:
            conditions = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        end = conditions.get("end") or {}
        if isinstance(end, dict) and end.get("state") == "done":
            yield path.parent, conditions


def hover_thrust_n(conditions: dict) -> float | None:
    echo = conditions.get("parameter_echo") or {}
    weight = _float(echo.get("airframe.weight_n"))
    if weight and weight > 0.0:
        return weight
    mass = _float(conditions.get("mass_mg"))
    return mass * 1e-6 * GRAVITY_M_S2 if mass and mass > 0.0 else None


def run_thrust_n(conditions: dict) -> float | None:
    """带桨轮的推力：程序油门目标（start 行优先），没有就按机重。"""
    target = _float((conditions.get("start") or {}).get("target_cn", conditions.get("target_cn")))
    if target and target > 0.0:
        return target / 100.0
    return hover_thrust_n(conditions)


def rod_lever_m(conditions: dict, azimuth_rad: float) -> float | None:
    from sysid.servo_fit import rod_axis_lever

    return rod_axis_lever(_core.firmware_tilt_levers(conditions.get("parameter_echo") or {}),
                          azimuth_rad)


def pod_gravity_reference(conditions: dict, azimuth_rad: float) -> tuple[float, float, float] | None:
    """静态偏心 H 的量级参考 (组件质量 kg, 质心到舵机轴 r m, m·g·r N·m/rad)：取机体参数里的
    servo_motor 组件（质量与质心 z）和沿杆轴的舵机转轴（按 cos²ψ/sin²ψ 加权）。

    只是上限参考：部件表里的 servo_motor 未必整个随舵机倾转。读不到返回 None。
    """
    echo = conditions.get("parameter_echo") or {}
    mass = _float(echo.get("airframe.servo_motor_mass_g"))
    cg = _float(echo.get("airframe.servo_motor_cg_z_m"))
    axes = [(math.cos(azimuth_rad) ** 2, _float(echo.get("airframe.servo1_axis_z_m"))),
            (math.sin(azimuth_rad) ** 2, _float(echo.get("airframe.servo2_axis_z_m")))]
    used = [(weight, axis) for weight, axis in axes if weight >= 0.02]
    if not mass or mass <= 0.0 or cg is None or any(axis in (None, 0.0) for _w, axis in used):
        return None
    axis_z = sum(weight * axis for weight, axis in used) / sum(weight for weight, _a in used)
    offset = abs(axis_z - cg)
    return mass * 1e-3, offset, mass * 1e-3 * GRAVITY_M_S2 * offset


def latest_twd(key: tuple, root: Path) -> tuple[Path, dict, dict] | None:
    """同组最近一次带桨 TWD 拟合：(目录, conditions, 拟合 JSON)。联合拟合优先于单轮。"""
    runs = sorted(((folder, c) for folder, c in _archived(root)
                   if str(c.get("mode")) == "0" and group_key(folder, c) == key),
                  key=lambda item: item[0].name, reverse=True)
    for folder, conditions in runs:
        for name in ("fit_joint.json", "fit.json"):
            try:
                data = json.loads((folder / name).read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            if data.get("structure") == "twd" and _float(data.get("reaction_couple_s2")) is not None:
                return folder, conditions, data
    return None


def group_pendulum_hz(folder: Path | None, conditions: dict,
                      root: Path | None = None) -> tuple[float, str] | None:
    """同组最近一次带桨 TWD 拟合的摆频 (Hz, 目录名)，舵机单独轮拿它钉死 I_杆；没有 None。

    舵机单独的 2 Hz 双脉冲激不起 0.66 Hz 的杆摆，摆频在这类数据里不可辨：2026-09-27 同设置
    两轮 3° 自由拟合出 0.80 与 1.55 Hz、J 差 5 倍；按带桨轮的摆频钉死后两轮 J 差 8%。
    摆频是 K/I_杆 的实测量，不依赖那轮假定的 K，换到本轮的 K 上照样成立。
    `root` 默认取本轮所在的存档根（`<根>/<日期>/rod_*`）：同组的轮次一定在同一个存档里。
    """
    if folder is None:
        return None
    try:
        found = latest_twd(group_key(folder, conditions), root or Path(folder).parent.parent)
    except OSError:
        return None
    if found is None:
        return None
    hz = _float(found[2].get("natural_hz"))
    return (hz, found[0].name) if hz is not None and hz > 0.0 else None


def backlash_on(conditions: dict) -> bool:
    """这一轮开没开舵机回差补偿（开跑时固件回的 SYSID BACKLASH）；没有溯源的旧轮次按没开。"""
    backlash = conditions.get("backlash")
    return isinstance(backlash, dict) and str(backlash.get("en")) == "1"


def servo_points(key: tuple, root: Path, exclude: Path | None, stiffness: float,
                 backlash: bool | None = None) -> list[tuple]:
    """同组其它舵机单独轮的 (幅值, J, 纯延迟, 1 Hz 等效延迟)；J 按回中刚度换到本轮的 K。

    backlash 给了就只要回差补偿开关与它相同的轮次。
    """
    points = []
    for folder, conditions in _archived(root):
        if str(conditions.get("mode")) != "3" or group_key(folder, conditions) != key:
            continue
        if backlash is not None and backlash_on(conditions) != backlash:
            continue
        if exclude is not None and folder.resolve() == Path(exclude).resolve():
            continue
        try:
            data = json.loads((folder / SERVO_FIT_FILE).read_text(encoding="utf-8"))["fit"]
            amplitude = float(data["amplitude_rad"])
            reaction = float(data["reaction_inertia_kg_m2"])
            scale = stiffness / float(data["stiffness_n_m_rad"])
            points.append((amplitude, reaction * scale, float(data["dead_time_s"]),
                           float(data["equivalent_delay_1hz_s"])))
        except (OSError, ValueError, KeyError, TypeError, ZeroDivisionError):
            continue
    return points


def summarise(result, conditions: dict, folder: Path | None, *, azimuth_rad: float,
              root: Path | None = None, pendulum_pin: tuple | None = None) -> ServoOutcome:
    """拟合结果 + 本轮快照 → 折算、对照、幅值依赖与三块卡片文字。存档读不到的部分就不报。"""
    from sysid.servo_fit import backlash_estimate, predicted_reaction_couple

    root = root or settings_store.settings_path().parent
    lever = rod_lever_m(conditions, azimuth_rad)
    hover = hover_thrust_n(conditions)
    reaction = result.reaction_inertia_kg_m2
    outcome = ServoOutcome(fit=result, lever_m=lever, hover_thrust_n=hover,
                           predicted_rho_s2=predicted_reaction_couple(reaction, lever, hover),
                           pod_reference=pod_gravity_reference(conditions, azimuth_rad),
                           pendulum_pin=pendulum_pin)
    key = group_key(folder, conditions) if folder is not None else (
        "", str(conditions.get("psi_mrad")), str(conditions.get("axis_off_um")))
    try:
        found = latest_twd(key, root) if folder is not None else None
    except OSError:
        found = None
    if found is not None:
        twd_folder, twd_conditions, data = found
        twd_lever = rod_lever_m(twd_conditions, azimuth_rad)
        twd_thrust = run_thrust_n(twd_conditions)
        rho = float(data["reaction_couple_s2"])
        predicted = predicted_reaction_couple(reaction, twd_lever, twd_thrust)
        outcome.comparison = dict(
            folder=twd_folder.name, rho_s2=rho, predicted_rho_s2=predicted,
            ratio=predicted / rho if rho and math.isfinite(predicted) else float("nan"),
            lever_m=twd_lever, thrust_n=twd_thrust,
            servo_wn_rad_s=_float(data.get("servo_wn_rad_s")),
            servo_zeta=_float(data.get("servo_zeta")),
            dead_time_s=_float(data.get("dead_time_s")))
    if folder is not None:
        try:
            points = servo_points(key, root, folder, result.stiffness_n_m_rad,
                                  backlash=backlash_on(conditions))
        except OSError:
            points = []
        points.append((result.amplitude_rad, reaction, result.dead_time_s,
                       result.equivalent_delay_1hz_s))
        outcome.amplitude = backlash_estimate(points)
    outcome.blocks = card_blocks(outcome)
    return outcome


def _num(value, fmt: str, scale: float = 1.0, unit: str = "") -> str:
    number = _float(value)
    return "—" if number is None else f"{number * scale:{fmt}}{unit}"


def card_blocks(outcome: ServoOutcome) -> dict[str, str]:
    fit = outcome.fit
    sign = ("负号 = 舵机加速把机体推向推力力矩的同一方向，带桨时形成 TWD 凹口"
            if fit.reaction_inertia_kg_m2 < 0.0 else
            "正号 = 舵机加速把机体推向推力力矩的反方向，带桨时是实零点（非最小相位）")
    head = [
        "【舵机与反作用（电机不转）】",
        f"反作用惯量 J：{fit.reaction_inertia_kg_m2:+.5f} kg·m²（±{fit.reaction_uncertainty_pct:.0f}%；"
        f"模型 I_杆·θ̈ + c·θ̇ + K·θ = −J·s̈，{sign}）",
        f"舵机响应：{fit.servo_wn_rad_s / (2.0 * math.pi):.1f} Hz（ωs = {fit.servo_wn_rad_s:.1f} rad/s），"
        f"阻尼比 {fit.servo_zeta:.2f}（空载：电机不转，没有桨的气动负载）",
        f"纯延迟：{fit.dead_time_s * 1000.0:.1f} ms；1 Hz 等效延迟 "
        f"{fit.equivalent_delay_1hz_s * 1000.0:.1f} ms",
        f"预测带桨反作用力偶 ρ_pred = −J/(|L|·T) = {_num(outcome.predicted_rho_s2, '.4g', unit=' s²')}"
        f"（本轮固件沿杆轴力臂 |L| = {_num(outcome.lever_m, '.4f', unit=' m')}，悬停推力 T = "
        f"{_num(outcome.hover_thrust_n, '.2f', unit=' N')}；与带桨拟合的 ρ 同一约定）",
    ]
    comparison = outcome.comparison
    if comparison is None:
        head.append("同组（同一天、同杆轴、同杆高）还没有带桨 TWD 拟合可对照。")
    else:
        head.append(
            f"对照同组最近的带桨 TWD 拟合（{comparison['folder']}）：ρ = {comparison['rho_s2']:.4g} s²；"
            f"按那轮固件力臂 {_num(comparison['lever_m'], '.4f', unit=' m')}、推力 "
            f"{_num(comparison['thrust_n'], '.2f', unit=' N')} 预测 {_num(comparison['predicted_rho_s2'], '.4g', unit=' s²')}"
            f"（预测/拟合 = {_num(comparison['ratio'], '.2f')}）；舵机 "
            f"{_num(comparison['servo_wn_rad_s'], '.1f', unit=' rad/s')} / 阻尼比 "
            f"{_num(comparison['servo_zeta'], '.2f')} / 纯延迟 "
            f"{_num(comparison['dead_time_s'], '.1f', 1000.0, ' ms')}（带桨）。只作交叉核对，没有钉进带桨拟合。")
    pin = outcome.pendulum_pin
    plant = [
        "【台架】",
        f"绕杆惯量 I_杆 = K/(2π·f摆)² = {fit.inertia_rod_kg_m2:.5f} kg·m²；摆频 {fit.natural_hz:.3f} Hz"
        + (f"（取同组带桨拟合 {pin[1]}：舵机单独激不起杆摆，摆频不在这里拟合）" if pin else "")
        + f"；绕杆阻尼 c = {fit.damping_n_m_s:.4f} N·m·s",
        f"回中刚度 K = {fit.stiffness_n_m_rad:.4f} N·m/rad（挂砝码实测，或没填时 m·g·d）",
        pod_gravity_line(fit, outcome.pod_reference),
    ]
    amplitude = outcome.amplitude
    if amplitude is not None:
        plant.append("【幅值依赖（同组舵机单独轮）】")
        for a, j, dead, eq in amplitude.rows:
            plant.append(f"摆幅 {math.degrees(a):.1f}°：J = {j:+.5f} kg·m²，纯延迟 {dead * 1000.0:.1f} ms，"
                         f"1 Hz 等效延迟 {eq * 1000.0:.1f} ms")
        backlash = amplitude.backlash_rad
        plant.append(
            f"大幅值极限 J∞ = {amplitude.reaction_limit_kg_m2:+.5f} kg·m²；回差半宽 b ≈ "
            + (f"{math.degrees(backlash):.2f}°（{backlash * 1000.0:.1f} mrad）" if math.isfinite(backlash)
               else "—") + f"；纯延迟 T(A) = {amplitude.delay_limit_s * 1000.0:.1f} ms + "
            f"{amplitude.delay_slope_s_rad * 1000.0:.2f} ms·rad / A")
        plant.extend(amplitude.notes)
    diag = ["【诊断】",
            f"拟合度（12 Hz 以下，去掉开头 0.5 s）：{fit.fit_percent:.1f}%；样本 {fit.samples}；"
            f"指令摆幅 {math.degrees(fit.amplitude_rad):.2f}°"]
    if fit.warnings:
        diag.append("提醒：" + "；".join(str(w).rstrip("。") for w in fit.warnings) + "。")
    if fit.blockers:
        diag.append("不可信：" + "；".join(fit.blockers) + "。")
    return {"pid": "\n".join(head), "plant": "\n".join(plant), "diag": "\n".join(diag)}


def pod_gravity_line(fit, reference) -> str:
    """静态偏心 H 一行：拟合值 + 部件表的量级参考。旧存档没有 H 时说明一句。"""
    value = _float(getattr(fit, "pod_gravity_n_m_rad", None))
    if value is None:
        return "静态重力偏心 H：这份结果出自没拟合 H 的旧版本，J 与 I_杆 可能被它拽偏，重新分析一次。"
    text = (f"静态重力偏心 H = {value:+.4f} N·m/rad（倾转组件质心不在舵机轴上，倾转时绕杆多出 H·s；"
            "只在杆上有，已单独拟合，不混进 I_杆 与 J）")
    if reference is not None:
        mass, offset, moment = reference
        text += (f"；量级参考：servo_motor 组件 {mass * 1000.0:.1f} g、质心离舵机轴 {offset:.3f} m → "
                 f"m·g·r ≈ {moment:.3f} N·m/rad（整个组件都随舵机倾转时的上限）")
    return text


def servo_card(outcome: ServoOutcome) -> str:
    blocks = outcome.blocks or card_blocks(outcome)
    return "\n".join(blocks[key] for key in ("pid", "plant", "diag"))




def latest_servo(key: tuple, root: Path) -> tuple[Path, dict] | None:
    """同组最近一轮已拟合的舵机单独轮：(目录, fit_servo.json 里的 fit)。"""
    runs = sorted(((folder, c) for folder, c in _archived(root)
                   if str(c.get("mode")) == "3" and group_key(folder, c) == key),
                  key=lambda item: item[0].name, reverse=True)
    for folder, _conditions in runs:
        try:
            return folder, json.loads((folder / SERVO_FIT_FILE).read_text(encoding="utf-8"))["fit"]
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return None


def servo_cross_check(result, conditions: dict, folder: Path | None, *, azimuth_rad: float,
                      root: Path | None = None) -> str:
    """带桨 TWD 拟合的交叉核对：同组最近的舵机单独轮预测的 ρ，与本次拟合的 ρ 并排。

    只报不钉：舵机单独轮是空载（没有桨的气动负载），回差也让它随幅值变。J 按本次拟合用的
    回中刚度折算（J ∝ K）。没有同组舵机单独轮或读不到数时返回空串。
    """
    from sysid.servo_fit import predicted_reaction_couple

    if folder is None or getattr(result, "structure", "") != "twd":
        return ""
    root = root or settings_store.settings_path().parent
    try:
        found = latest_servo(group_key(folder, conditions), root)
    except OSError:
        return ""
    if found is None:
        return ""
    servo_folder, data = found
    try:
        reaction, servo_stiffness = float(data["reaction_inertia_kg_m2"]), float(data["stiffness_n_m_rad"])
        stiffness = _float(getattr(result, "rig_stiffness_n_m_rad", None))
        if stiffness is None:
            stiffness = (float(conditions["mass_mg"]) * 1e-6 * 9.81
                         * float(getattr(result, "pivot_above_cg_m")))
        scale = stiffness / servo_stiffness
        scaled = reaction * scale
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return ""
    lever, thrust = rod_lever_m(conditions, azimuth_rad), run_thrust_n(conditions)
    predicted = predicted_reaction_couple(scaled, lever, thrust)
    rho = _float(getattr(result, "reaction_couple_s2", None))
    ratio = predicted / rho if rho and math.isfinite(predicted) else float("nan")
    text = (f"舵机单独交叉核对（{servo_folder.name}，空载）：J = {scaled:+.5f} kg·m²（按本次回中刚度折算）"
            f"→ 预测 ρ_pred = {_num(predicted, '.4g', unit=' s²')}，本次拟合 ρ = {_num(rho, '.4g', unit=' s²')}"
            f"（预测/拟合 = {_num(ratio, '.2f')}）；舵机 {_num(data.get('servo_wn_rad_s'), '.1f', unit=' rad/s')} / "
            f"阻尼比 {_num(data.get('servo_zeta'), '.2f')} / 纯延迟 "
            f"{_num(data.get('dead_time_s'), '.1f', 1000.0, ' ms')}。只作核对，没有钉进拟合。")
    return text + _pod_gravity_share(_float(data.get("pod_gravity_n_m_rad")), scale, lever, thrust,
                                     result)


def _pod_gravity_share(pod_gravity, scale, lever, thrust, result) -> str:
    """带桨轮里 H·s = H/(|L|·T)·u 被并进推力增益 G：报它占 G 多少（k 的几何核对没扣它）。"""
    if pod_gravity is None or not lever or not thrust or lever <= 0.0 or thrust <= 0.0:
        return ""
    share = pod_gravity * scale / (lever * thrust)
    kappa = _float(getattr(result, "torque_model_scale", None))
    torque_scale = _float(getattr(result, "torque_scale", None))
    gain = kappa * torque_scale if kappa is not None and torque_scale is not None else None
    text = (f"静态重力偏心 H = {pod_gravity * scale:+.4f} N·m/rad 在带桨轮里折成 H/(|L|·T) = "
            f"{share:+.3f}（固件力矩单位），混在推力增益 G 里")
    if gain:
        text += f"：本次 G = κ·尺度 = {gain:.3f}，其中约 {100.0 * share / gain:+.0f}% 来自 H"
    return text + "；k 的几何核对没有扣掉这一项，看 k 时把它考虑进去。"


__all__ = ["SERVO_FIT_FILE", "ServoOutcome", "backlash_on", "card_blocks", "group_key", "hover_thrust_n",
           "latest_servo", "latest_twd", "pod_gravity_line", "pod_gravity_reference",
           "rod_lever_m", "run_thrust_n", "servo_card", "servo_cross_check", "servo_points",
           "summarise"]
