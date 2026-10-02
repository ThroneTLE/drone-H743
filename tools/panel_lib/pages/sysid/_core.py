"""面板到 `tools/sysid/` 分析核心的**唯一**桥。

为什么要有这一层薄封装：`panel_lib` 是 `tools/` 下的一个包，而 `sysid` 是它的
兄弟包；界面文件里散落十几处 `sys.path` 拼接，迟早有一处拼错而在某台机器上
静默退化成"图画不出来但也不报错"。路径只在这里拼一次。

它同时是一道**依赖边界**：界面只允许通过这里用分析核心，反过来分析核心里
不许出现任何 Tk——辨识结论必须能在没有飞控、没有窗口的情况下重跑一遍。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import TYPE_CHECKING, Mapping

_TOOLS = Path(__file__).resolve().parents[3]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from sysid.decode import (  # noqa: E402
    SchemaMismatch, SysIdBatch, SysIdSchema, parse_schema_lines,
)
from sysid import decode as _decode  # noqa: E402
from sysid.excitation import (  # noqa: E402
    PROFILE_CODES, PROFILE_NAMES, Excitation, ExcitationInvalid,
)
from sysid.profile import FitResult, Gains, IdentProfile  # noqa: E402
from sysid.rig import Rig  # noqa: E402
from sysid.rig_stiffness import (  # noqa: E402
    ASSUMPTIONS as STIFFNESS_ASSUMPTIONS, StiffnessEstimate, extra_stiffness, hanging_sides,
    implied_pivot_m, reading_axis, weight_test_stiffness,
)
from sysid import breakaway as _breakaway  # noqa: E402  纯 Python，导入不拉起 numpy
from sysid import xy_analysis as _xy  # noqa: E402  同上
from sysid import yaw_analysis as _yaw  # noqa: E402  同上（纯 Python，不依赖 numpy）

if TYPE_CHECKING:  # 拟合模块会拉起 numpy/scipy，只在真正拟合时才导入
    from sysid.fit import ModelFit

__all__ = [
    "Excitation", "ExcitationInvalid", "PROFILE_CODES", "PROFILE_NAMES",
    "SchemaMismatch", "SysIdBatch", "SysIdSchema", "decode_batch",
    "parse_schema_lines", "FitResult", "Gains", "IdentProfile", "Rig",
    "fit_inner_loop", "fit_inner_loop_multi", "synthesise_gains",
    "torque_model_signature", "firmware_tilt_levers", "fit_servo_only",
    "vibration_report", "STIFFNESS_ASSUMPTIONS", "StiffnessEstimate", "extra_stiffness",
    "implied_pivot_m", "weight_test_stiffness", "RECORD_VERSIONS", "hanging_sides",
    "reading_axis", "breakaway_analysis", "breakaway_summary",
    "xy_analysis", "xy_summary", "xy_rename_samples",
    "yaw_analysis", "yaw_summary", "yaw_rename_samples", "yaw_integrate_psi",
]

#: 页面接受的 SYSID 记录版本：v2（13 字段）与 v3（再加 erpm_lower、servo_tilt，舵机单独模式要它）。
RECORD_VERSIONS = (2, 3)


def decode_batch(data: bytes, schema: SysIdSchema) -> SysIdBatch:
    """`sysid.decode.decode_batch`：v1/v2/v3 逐字段按固件报的表解，schema_hash、帧长照旧逐项核对。"""
    return _decode.decode_batch(data, schema)

#: 旧固件（2026-09-27 之前）力矩模型的有效系数，源自 2026-07-25 的系统辨识。固件已把
#: DRV_COAX_CTRL_{ROLL,PITCH}_EFFECTIVENESS 删除，这里冻结原值，只用来解读旧记录。
LEGACY_ROLL_EFFECTIVENESS = 0.581
LEGACY_PITCH_EFFECTIVENESS = 0.569

_LEGACY_ARM_KEYS = ("airframe.roll_thrust_lever_arm_m", "airframe.pitch_thrust_lever_arm_m")
_GEOMETRIC_KEYS = ("airframe.servo1_axis_z_m", "airframe.servo2_axis_z_m", "airframe.cg_z_m")


def _echo_float(parameter_echo: Mapping[str, str], key: str) -> float | None:
    import math

    raw = parameter_echo.get(key)
    if raw in (None, ""):
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def torque_model_signature(parameter_echo: Mapping[str, str]) -> str:
    """"legacy"（echo 里有 airframe.roll/pitch_thrust_lever_arm_m：旧的 力臂×有效系数 模型）/
    "geometric"（没有这两项、有 servo1/2_axis_z_m 与 cg_z_m：2026-09-27 起的几何力臂）/ "unknown"。"""
    echo = parameter_echo or {}
    if any(key in echo for key in _LEGACY_ARM_KEYS):
        return "legacy"
    if all(key in echo for key in _GEOMETRIC_KEYS):
        return "geometric"
    return "unknown"


def firmware_tilt_levers(parameter_echo: Mapping[str, str]) -> tuple[float, float] | None:
    """(roll, pitch) 固件把舵机倾角换成记录力矩时用的带符号有效力臂 [m]：τ_fw = lever·T·sin(tilt)。
    legacy：lever = polarity·arm·EFF，EFF 冻结为 roll 0.581 / pitch 0.569（源自 2026-07-25，固件已删除），
            polarity = +1 if thrust_point_to_cg_z_m < 0 else −1。
    geometric：lever = cg_z_m − servoN_axis_z_m（轴在重心下方为正）。
    算不出来（缺项/为 0）返回 None。"""
    echo = parameter_echo or {}
    signature = torque_model_signature(echo)
    if signature == "legacy":
        roll_arm, pitch_arm = (_echo_float(echo, key) for key in _LEGACY_ARM_KEYS)
        thrust_point = _echo_float(echo, "airframe.thrust_point_to_cg_z_m")
        if None in (roll_arm, pitch_arm, thrust_point) or thrust_point == 0.0:
            return None
        polarity = 1.0 if thrust_point < 0.0 else -1.0
        levers = (polarity * roll_arm * LEGACY_ROLL_EFFECTIVENESS,
                  polarity * pitch_arm * LEGACY_PITCH_EFFECTIVENESS)
    elif signature == "geometric":
        roll_axis, pitch_axis, cg = (_echo_float(echo, key) for key in _GEOMETRIC_KEYS)
        # 舵机转轴 0 在固件里是"没填"（解锁闸门拒绝），不是"转轴在板子平面上"。
        if None in (roll_axis, pitch_axis, cg) or 0.0 in (roll_axis, pitch_axis):
            return None
        levers = (cg - roll_axis, cg - pitch_axis)
    else:
        return None
    if any(abs(lever) < 1e-6 for lever in levers):
        return None
    return levers


def fit_inner_loop(times_s, samples, *, azimuth_rad: float, mass_kg: float,
                   assumed_inertia_kg_m2: float,
                   thrust_point_to_cg_z_m: float | None = None,
                   pivot_above_cg_m: float | None = None,
                   pivot_above_cg_guess_m: float | None = None,
                   roll_pivot_to_cg_z_m: float | None = None,
                   pitch_pivot_to_cg_z_m: float | None = None,
                   pendulum_hz: float | None = None,
                   structure: str = "twd",
                   firmware_tilt_levers_m: tuple[float, float] | None = None,
                   rig_stiffness_n_m_rad: float | None = None) -> ModelFit:
    """把一次采集喂给拟合器，返回 `sysid.fit.ModelFit`。

    输入取固件按最终舵机脉宽反算的绕质心力矩估计，输出为陀螺沿杆轴投影。
    力矩依赖推力表、有效力臂和机械映射，是模型值不是测得值：真实力矩 = κ × 记录力矩。
    该低阶模型给出惯量与总延迟，不能分离两个舵机的各自延迟。

    台架是杆在质心上方 d 的单摆。`pivot_above_cg_m` 是**量出来的**杆高
    （`pivot_above_cg_guess_m` 为旧名，同义）：|d| ≥ 2 cm 时默认按 TWD 结构拟合——
    刚体单摆 + 二阶舵机 + 舵机甩动的反作用力偶 + 纯延迟，阻尼按 0——并按飞行对象
    回路整形给出裕度；`structure="rigid"` 是 4 Hz 刚体积分读数，只作对照。
    `pendulum_hz`（实测摆频）给了就按它钉死绕杆惯量。力臂参考点是舵机倾转轴：`roll_pivot_to_cg_z_m` /
    `pitch_pivot_to_cg_z_m`（+ 为轴在质心上方）给几何换算
    s(d) = cos²ψ·(h_roll − d)/h_roll + sin²ψ·(h_pitch − d)/h_pitch；缺了所需的高度照算，
    但 `tuning_blockers` 非空。`thrust_point_to_cg_z_m`（桨盘中点）只作兜底。
    不给 d 时按杆过质心的旧工况，κ 取 1，不需要倾转轴高度。
    `firmware_tilt_levers_m` 取 `firmware_tilt_levers(parameter_echo)`：给了就把 κ 与
    "实测倾转轴高度 / 固件力臂"的几何预测对拍（k = κ/κ_几何），否则 κ 只按宽范围检查。
    整定用 `result.tuning_inertia_kg_m2`（I_cg/κ）；`result.tuning_blockers` 非空时不要整定。
    `rig_stiffness_n_m_rad`（挂砝码实测的台架刚度，`weight_test_stiffness`）给了就代替 m·g·d。
    转角由陀螺积分得到，记录里的融合姿态在摆动中被 IMU 的切向/向心加速度带偏，
    不参与拟合。
    """
    from sysid import fit as fit_module

    options = _fit_options(assumed_inertia_kg_m2, mass_kg, thrust_point_to_cg_z_m,
                           pivot_above_cg_m, pivot_above_cg_guess_m)
    t_uniform, torque_uniform, rate_uniform = _uniform_record(times_s, samples, azimuth_rad)
    return fit_module.fit_model(t_uniform, torque_uniform, rate_uniform,
                                mass_kg=mass_kg, azimuth_rad=azimuth_rad,
                                roll_pivot_to_cg_z_m=roll_pivot_to_cg_z_m,
                                pitch_pivot_to_cg_z_m=pitch_pivot_to_cg_z_m,
                                pendulum_hz=pendulum_hz, structure=structure,
                                firmware_tilt_levers_m=firmware_tilt_levers_m,
                                rig_stiffness_n_m_rad=rig_stiffness_n_m_rad, **options)


def fit_inner_loop_multi(runs, *, azimuth_rad: float, mass_kg: float,
                         assumed_inertia_kg_m2: float,
                         thrust_point_to_cg_z_m: float | None = None,
                         pivot_above_cg_m: float | None = None,
                         pivot_above_cg_guess_m: float | None = None,
                         roll_pivot_to_cg_z_m: float | None = None,
                         pitch_pivot_to_cg_z_m: float | None = None,
                         pendulum_hz: float | None = None,
                         structure: str = "twd",
                         firmware_tilt_levers_m: tuple[float, float] | None = None,
                         rig_stiffness_n_m_rad: float | None = None) -> ModelFit:
    """多轮联合拟合：`runs` 为若干 `(times_s, samples)`，同一台架、同一几何。

    几轮共享一组模型参数（TWD：I_杆, G, T, ωs, ζs, ρ），每轮各自解初值与零偏，
    返回一个 `ModelFit`；
    `fit_percent` 是最差一轮的带内拟合优度，逐轮值在 `fit_percent_runs`。
    各轮必须是同一套固件力矩模型（按 `torque_model_signature` 分组），共用一个
    `firmware_tilt_levers_m`；不同力矩模型的轮次不能混在一起联合。
    只用于量了杆高（|d| ≥ 2 cm）的单摆台架，其余校验与 `fit_inner_loop` 相同。
    """
    from sysid import fit as fit_module

    options = _fit_options(assumed_inertia_kg_m2, mass_kg, thrust_point_to_cg_z_m,
                           pivot_above_cg_m, pivot_above_cg_guess_m)
    records = [_uniform_record(times_s, samples, azimuth_rad) for times_s, samples in runs]
    if not records:
        raise ValueError("没有可拟合的轮次")
    return fit_module.fit_model_multi(records, mass_kg=mass_kg, azimuth_rad=azimuth_rad,
                                      roll_pivot_to_cg_z_m=roll_pivot_to_cg_z_m,
                                      pitch_pivot_to_cg_z_m=pitch_pivot_to_cg_z_m,
                                      pendulum_hz=pendulum_hz, structure=structure,
                                      firmware_tilt_levers_m=firmware_tilt_levers_m,
                                      rig_stiffness_n_m_rad=rig_stiffness_n_m_rad,
                                      **options)


def _fit_options(assumed_inertia_kg_m2, mass_kg, thrust_point_to_cg_z_m,
                 pivot_above_cg_m, pivot_above_cg_guess_m) -> dict:
    import math

    if not (assumed_inertia_kg_m2 > 0.0):
        raise ValueError("假定惯量必须为正；先读回飞控状态拿到它")
    if not math.isfinite(mass_kg) or mass_kg <= 0:
        raise ValueError("质量或采样含无效值")
    if thrust_point_to_cg_z_m is not None:
        thrust_point_to_cg_z_m = float(thrust_point_to_cg_z_m)
        if not math.isfinite(thrust_point_to_cg_z_m) or abs(thrust_point_to_cg_z_m) < 1e-3:
            raise ValueError("推力点到质心的垂直距离必须是非零有限值；核对机体参数")
    if pivot_above_cg_m is None:
        pivot_above_cg_m = pivot_above_cg_guess_m
    if pivot_above_cg_m is not None:
        pivot_above_cg_m = float(pivot_above_cg_m)
        if not math.isfinite(pivot_above_cg_m):
            raise ValueError("杆高必须是有限值")
    return dict(inertia_guess=assumed_inertia_kg_m2,
                thrust_point_to_cg_z_m=thrust_point_to_cg_z_m,
                pivot_above_cg_m=pivot_above_cg_m)


def _uniform_record(times_s, samples, azimuth_rad):
    """一轮采样 → 等距 (t, torque, rate)。校验时间轴与字段，拒绝跨断点插值。"""
    import math

    import numpy as np

    from sysid import fit as fit_module

    if len(times_s) != len(samples):
        raise ValueError("时间轴与样本数不一致")
    t = np.asarray(times_s, dtype=float)
    if any("torque" not in s for s in samples):
        raise ValueError("旧记录没有限位后的力矩，不能用于惯量辨识；请更新固件重新采集")
    torque = np.asarray([s["torque"] for s in samples], dtype=float)
    rate = np.asarray(
        [s.get("gx", 0.0) * math.cos(azimuth_rad)
         + s.get("gy", 0.0) * math.sin(azimuth_rad) for s in samples], dtype=float)
    angle = np.asarray([s.get("angle", 0.0) for s in samples], dtype=float)
    if not all(np.all(np.isfinite(v)) for v in (t, torque, rate, angle)):
        raise ValueError("质量或采样含无效值")
    steps = np.diff(t)
    # 控制拍间隔 2.0–3.1 ms，250 Hz 抽样的最坏间隔约 6.1 ms（中位数的 1.5 倍以上）属正常；
    # 真丢样由固件的 GAP 标志和 dropped 计数兜底，这里只拦明显的断点。
    if not len(steps) or np.any(steps <= 0) or np.max(steps) > 2.5*np.median(steps):
        raise ValueError("时间轴存在重复、倒退或断点，拒绝跨断点插值拟合")

    fs = 1.0/float(np.median(steps))
    t_uniform, torque_uniform = fit_module.uniform_signal(t, torque, fs)
    _, rate_uniform = fit_module.uniform_signal(t, rate, fs)
    return t_uniform, torque_uniform, rate_uniform


def fit_servo_only(times_s, samples, *, azimuth_rad: float, mass_kg: float,
                   assumed_inertia_kg_m2: float, pivot_above_cg_m: float | None,
                   rig_stiffness_n_m_rad: float | None = None,
                   inertia_rod_kg_m2: float | None = None,
                   pendulum_hz: float | None = None):
    """舵机单独（电机不转）一轮 → `sysid.servo_fit.ServoFit`。

    输入是记录里的 `servo_tilt`（v3 记录，指令倾角沿杆轴、正向 = 杆轴正向力矩），输出是陀螺沿
    杆轴投影。回中刚度 K 取挂砝码实测值，没有就 m·g·d（杆要在质心上方至少 2 cm）。
    `pendulum_hz`（实测摆频）给了、又没直接给 I_杆 时，按 I_杆 = K/(2π·f)² 钉死。
    时间轴校验与 `fit_inner_loop` 相同（拒绝跨断点插值）。
    """
    import math

    import numpy as np

    from sysid import fit as fit_module
    from sysid import servo_fit

    if any("servo_tilt" not in s for s in samples):
        raise ValueError("这轮记录里没有舵机倾角 servo_tilt：舵机单独模式需要 SYSID 记录 v3 的固件")
    if not (math.isfinite(mass_kg) and mass_kg > 0.0):
        raise ValueError("质量或采样含无效值")
    if not (assumed_inertia_kg_m2 > 0.0):
        raise ValueError("假定惯量必须为正；先读回飞控状态拿到它")
    pivot = None if pivot_above_cg_m is None else float(pivot_above_cg_m)
    if rig_stiffness_n_m_rad is not None:
        stiffness = float(rig_stiffness_n_m_rad)
    elif pivot is not None and pivot >= fit_module.PIVOT_MEASURED_MIN_M:
        stiffness = mass_kg * 9.81 * pivot
    else:
        raise ValueError("舵机单独拟合要有回中刚度：量杆到飞控的距离（杆在质心上方至少 2 cm），"
                         "或在「准备」页填挂砝码试验")
    if (inertia_rod_kg_m2 is None and pendulum_hz is not None and math.isfinite(pendulum_hz)
            and pendulum_hz > 0.0):
        inertia_rod_kg_m2 = stiffness / (2.0 * math.pi * float(pendulum_hz)) ** 2
    tilt_samples = [dict(s, torque=s["servo_tilt"]) for s in samples]
    t_uniform, tilt_uniform, rate_uniform = _uniform_record(times_s, tilt_samples, azimuth_rad)
    guess = assumed_inertia_kg_m2 + mass_kg * (pivot or 0.0) ** 2
    result = servo_fit.fit_servo_only(t_uniform, np.asarray(tilt_uniform), rate_uniform,
                                      stiffness_n_m_rad=stiffness, inertia_rod_guess_kg_m2=guess,
                                      inertia_rod_kg_m2=inertia_rod_kg_m2)
    return result


def breakaway_analysis(times_s, samples, conditions) -> dict:
    """槽式台架 break 轮（ALT）的离地/滑落阈值：`sysid.breakaway.analyse_conditions`。

    `conditions` 是本轮快照（要 alt_phases 与移动质量）；输入不成立抛中文 ValueError。
    """
    return _breakaway.analyse_conditions(times_s, samples, conditions)


def breakaway_summary(result: dict) -> str:
    """阈值结果的中文摘要（`sysid.breakaway.summary_text`）。"""
    return _breakaway.summary_text(result)


def vibration_report(times_s, samples):
    """有真实 eRPM 时的振动主频与 eRPM→振动比（`sysid.vibration.VibrationReport`），否则 None。"""
    from sysid.vibration import erpm_vibration_report

    return erpm_vibration_report(times_s, samples)


#: 杆轴与某一机体轴夹角的余弦达到它，就认为这趟只辨到了那一个轴。
SINGLE_AXIS_COSINE = 0.9


def identified_axes(azimuth_rad: float | None) -> tuple[tuple[str, ...], str]:
    """(这一趟辨识到的轴, 说明)。杆沿 Y（|sin ψ| ≥ 0.9）只算俯仰，沿 X 只算横滚。"""
    import math

    if azimuth_rad is None:
        return ("roll", "pitch"), ""
    if abs(math.sin(azimuth_rad)) >= SINGLE_AXIS_COSINE:
        return ("pitch",), "杆沿俯仰轴：只给俯仰参数。"
    if abs(math.cos(azimuth_rad)) >= SINGLE_AXIS_COSINE:
        return ("roll",), "杆沿横滚轴：只给横滚参数。"
    return ("roll", "pitch"), ("斜杆辨识的是两轴混合：两轴写同一组值，"
                               "各轴单独上杆辨识之前不要当成各自的最优值。")


def synthesise_gains(result, *, azimuth_rad: float | None = None):
    """由拟合结果合成候选增益，并给出只写 RAM 的 `SYSID PARAM` 命令行（**不含 SAVE**）。

    TWD 结构（实测杆高的默认）：在飞行对象
    e^{−sT}·舵机二阶·(κ + ρs²)/(I_cg·s)（再乘反馈通路的转速陷波 `tune.DEPLOYED_RPM_NOTCH`）
    上回路整形，取满足 GM ≥ 6 dB、PM ≥ 45° 的最大
    kp，ki = kp·ωc/5；角度环 P 由速率环穿越频率按原比例给。其余结构沿用"延迟 → 带宽"
    公式（整定惯量 I_cg/κ、整定阻尼）。`azimuth_rad` 给出时只写这一趟辨识到的轴；
    None 时两轴都写（兼容旧调用）。是否可整定看 `result.tuning_blockers`，
    这里不替调用方判断。
    """
    from dataclasses import replace

    from sysid import tune

    axes, axis_note = identified_axes(azimuth_rad)
    kappa = result.torque_model_scale
    if getattr(result, "structure", "rigid") == "twd":
        rate = tune.synthesise_rate_shaped(tune.FlightPlant(
            inertia_cg_kg_m2=result.inertia_kg_m2, torque_model_scale=kappa,
            dead_time_s=result.dead_time_s, servo_wn_rad_s=result.servo_wn_rad_s,
            servo_zeta=result.servo_zeta, reaction_couple_s2=result.reaction_couple_s2,
            feedback_notch=tune.DEPLOYED_RPM_NOTCH))
        rate = replace(rate, rationale=(
            axis_note + f"整定惯量 I_cg/κ = {result.tuning_inertia_kg_m2:.5f} kg·m²（固件力矩单位），"
            "阻尼按 0。" + rate.rationale))
    else:
        rate = tune.synthesise_rate(result.tuning_inertia_kg_m2,
                                    result.tuning_damping_n_m_s, result.delay_s)
        rate = replace(rate, rationale=(
            axis_note +
            f"整定惯量 I_cg/κ = {result.inertia_kg_m2:.5f}/{kappa:.3f} = "
            f"{result.tuning_inertia_kg_m2:.5f} kg·m²；整定阻尼 {result.tuning_damping_n_m_s:.5f} N·m·s"
            f"（绕杆阻尼 c = {result.damping_n_m_s:.5f}；量了杆高时取 min(c, c·(h/(h−d))²)/κ："
            "桨气动阻尼换到质心要乘 (h/(h−d))²，轴承摩擦飞行时没有，取小者；旧工况直接用 c）。"
            "两者都按固件力矩单位（真实力矩 = κ × 固件力矩）。" + rate.rationale))
    attitude = tune.synthesise_attitude(rate)
    gains = Gains(rate_kp=rate.kp, rate_ki=rate.ki, rate_kd=rate.kd,
                  att_kp=attitude.kp)
    return rate, attitude, gains.param_commands(axes=axes)


def xy_rename_samples(samples):
    """`xy_analysis.rename_samples`：XY 轮的尾字段改成 XY 含义的名字（页面上不出现 height）。"""
    return _xy.rename_samples(samples)


def xy_analysis(times_s, samples, conditions: dict) -> dict:
    """`xy_analysis.analyse_conditions`：按本轮注入类型分析；输入不成立抛中文 ValueError。"""
    return _xy.analyse_conditions(times_s, samples, conditions)


def xy_summary(result: dict) -> str:
    return _xy.summary_text(result)


def yaw_rename_samples(samples, times_s=None):
    """`yaw_analysis.rename_samples`：YAW 轮的尾字段改成 YAW 含义的名字（页面上不出现 height）；
    给了时间轴就补一列由陀螺 z 积分得到的偏航角 psi。"""
    return _yaw.rename_samples(samples, times_s)


def yaw_integrate_psi(times_s, omega):
    """`yaw_analysis.integrate_psi`：陀螺 z 对时间的梯形积分（开跑清零）。"""
    return _yaw.integrate_psi(times_s, omega)


def yaw_analysis(times_s, samples, conditions: dict) -> dict:
    """`yaw_analysis.analyse_conditions`：按本轮注入类型分析；输入不成立抛中文 ValueError。"""
    return _yaw.analyse_conditions(times_s, samples, conditions)


def yaw_summary(result: dict) -> str:
    return _yaw.summary_text(result)
