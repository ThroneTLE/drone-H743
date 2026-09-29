"""结果卡片的文字：拟合结论、跟踪误差、候选参数。

只做排版与少量算术，不碰 Tk，也不做拟合本身（拟合在 `_core.fit_inner_loop`）。
新字段（杆上惯量、辨识出的杆到质心距离、固有频率）一律 `getattr` 取，
拟合核心还没升级时卡片照样能出，只是那几行显示“—”。
"""

from __future__ import annotations

import math

#: 可信门槛以拟合给的 `tuning_blockers` 为准；页面只留两条兜底（拟合核心旧版本也挡得住）：
#: 延迟低于 1 ms 等于没辨出延迟（拿接近 0 的延迟算带宽会得到天文数字的增益），
#: 整定惯量必须为正。
MIN_DELAY_S = 0.001


def _num(value, fmt: str, scale: float = 1.0, unit: str = "") -> str:
    try:
        number = float(value) * scale
    except (TypeError, ValueError):
        return "—"
    if not math.isfinite(number):
        return "—"
    return f"{number:{fmt}}{unit}"


def _finite(value) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def quality_problems(result, extra=()) -> list[str]:
    """不给建议参数的理由；空列表 = 可以给。以拟合的 `tuning_blockers` 为准。"""
    problems = list(extra)
    problems.extend(str(item) for item in (getattr(result, "tuning_blockers", ()) or ()))
    delay = _finite(getattr(result, "delay_s", None))
    inertia = _finite(getattr(result, "tuning_inertia_kg_m2", None))
    if inertia is None:
        inertia = _finite(getattr(result, "inertia_kg_m2", None))
    if (delay is None or delay < MIN_DELAY_S) and not any("延迟" in p for p in problems):
        problems.append("没辨出延迟（小于 1 ms）")
    if inertia is None or inertia <= 0:
        problems.append("整定用惯量不为正")
    return problems


def fit_passes(result, extra=()) -> bool:
    return not quality_problems(result, extra)


def advice_for(problems: list[str]) -> list[str]:
    """按拦下的原因给下一步；同类原因只说一次。"""
    advice = []

    def add(text):
        if text not in advice:
            advice.append(text)

    for problem in problems:
        if "倾转轴" in problem:
            axes = [axis for axis in ("俯仰", "横滚") if axis in problem] or ["俯仰"]
            add(f"到「1 · 准备」填{'、'.join(axes)}倾转轴到飞控板的距离，再点「重新分析」，不用重跑")
        elif "κ" in problem or "回转半径" in problem or "I_cg" in problem or "偏心" in problem \
                or "整定用惯量不为正" in problem:
            add("核对杆和倾转轴的尺量值")
        else:   # 拟合优度低、不确定度高、延迟辨不出、不可分离
            add("检查台架夹紧，或加长激励后重跑")
    return advice


def _hz_from_rad(value) -> str:
    number = _finite(value)
    return "—" if number is None else f"{number / (2.0 * math.pi):.1f} Hz"


def card_blocks(result, *, pivot_m: float | None, extra=(), header: str = "",
                geometry: str = "") -> dict[str, str]:
    """结果卡分三块：给 PID 用的 / 对象特性 / 诊断。缺字段一律显示"—"。"""
    get = lambda name: getattr(result, name, None)      # noqa: E731 - 只在这里用
    uncertainty = _finite(get("tuning_inertia_uncertainty_pct"))
    spread = "" if uncertainty is None else f"（不确定度 ±{uncertainty:.0f}%）"
    pid = [line for line in (header, geometry) if line] + [
        "【给 PID 用的】",
        f"整定惯量：{_num(get('tuning_inertia_kg_m2'), '.5g', unit=' kg·m²')}{spread}",
        f"1 Hz 等效延迟：{_num(get('delay_s'), '.1f', 1000.0, ' ms')}"
        "（舵机响应和纯延迟合起来，在 1 Hz 摆动时相当于晚了多久）",
        f"建议参数的裕度：增益裕度 {_num(get('gain_margin_db'), '.1f', unit=' dB')} / "
        f"相位裕度 {_num(get('phase_margin_deg'), '.0f', unit='°')} / "
        f"穿越 {_num(get('crossover_hz'), '.2f', unit=' Hz')}"
        "（增益裕度 ≥6 dB、相位裕度 ≥45° 才算稳）",
    ]
    findings = [str(n).rstrip("。") for n in (getattr(result, "notes", ()) or ())
                if "固件力矩模型" in str(n) and ("高估" in str(n) or "低估" in str(n))]
    plant = [
        "【对象特性】",
        *(f"固件算的倾转力矩和实际不符：{finding}。" for finding in findings),
        f"舵机响应：{_hz_from_rad(get('servo_wn_rad_s'))}，阻尼比 {_num(get('servo_zeta'), '.2f')}"
        "（舵机跟上指令角度有多快、会不会过冲）",
        f"纯延迟：{_num(get('dead_time_s'), '.1f', 1000.0, ' ms')}（指令发出到舵机开始动的时间）",
        f"反作用力矩系数 ρ：{_num(get('reaction_couple_s2'), '.3g', unit=' s²')}"
        "（舵机甩动电机组件时机体被反推的程度，飞行时也存在）",
        f"“尾巴摇狗”零点：台架 {_num(get('rig_zero_hz'), '.1f', unit=' Hz')} / "
        f"飞行 {_num(get('flight_zero_hz'), '.1f', unit=' Hz')}"
        "（比这更快的指令，机体会先往反方向动一下）",
        f"摆动固有频率：{_num(get('natural_hz'), '.2f', unit=' Hz')}（机体在杆上自己摆的快慢）",
        f"力矩模型标定系数 κ：{_num(get('torque_model_scale'), '.3f')}（1 = 固件的力矩模型准确）",
        f"杆上惯量（绕杆）：{_num(get('inertia_rod_kg_m2'), '.5g', unit=' kg·m²')}；"
        f"飞行惯量（绕质心）：{_num(get('inertia_kg_m2'), '.5g', unit=' kg·m²')}",
        f"杆到质心距离（你量的输入）：{_num(pivot_m, '.1f', 1000.0, ' mm')}",
    ]
    stiffness = _finite(get("rig_stiffness_n_m_rad"))
    if stiffness is not None:
        plant.append(
            f"台架刚度（挂砝码实测）：K = {stiffness:.4f} N·m/rad → 等效杆高 d_eff = "
            f"{_num(get('effective_pivot_m'), '.1f', 1000.0, ' mm')}（量的 d = "
            f"{_num(pivot_m, '.1f', 1000.0, ' mm')}），比只算重力多出 "
            f"{_num(get('extra_stiffness_n_m_rad'), '+.4f', unit=' N·m/rad')}（线缆等）；"
            "I_杆、κ 与 k 都按 K 标定")
    band = _finite(get("fit_band_hz"))
    runs = list(get("fit_percent_runs") or ())
    diag = [
        "【诊断】",
        f"拟合度（{band if band is not None else 4:g} Hz 以下，去掉开头 0.5 s）："
        f"{_num(get('fit_percent'), '.1f', unit='%')}",
    ]
    reference = _finite(get("fit_percent_15hz"))
    if reference is not None:
        diag.append(f"15 Hz 全频段 {reference:.1f}%（含碳杆约 9 Hz 台架抖动，只作参考）")
    if len(runs) > 1:
        diag.append("逐轮拟合度：" + " / ".join(_num(v, ".1f", unit="%") for v in runs))
    if any(_finite(get(name)) is not None for name in
           ("rigid_tuning_inertia_kg_m2", "rigid_delay_s", "rigid_fit_percent")):
        diag.append(
            f"刚体模型读数：整定惯量 {_num(get('rigid_tuning_inertia_kg_m2'), '.5g', unit=' kg·m²')}、"
            f"延迟 {_num(get('rigid_delay_s'), '.1f', 1000.0, ' ms')}、"
            f"拟合度 {_num(get('rigid_fit_percent'), '.1f', unit='%')}，仅供对比，不用于整定")
    warnings = [w for w in list(extra) + list(getattr(result, "warnings", []) or [])
                if str(w).rstrip("。") not in findings]         # 已写进【对象特性】的不重复
    if warnings:
        diag.append("提醒：" + "；".join(str(w).rstrip("。") for w in warnings) + "。")
    return {"pid": "\n".join(pid), "plant": "\n".join(plant), "diag": "\n".join(diag)}


def fit_card(result, *, pivot_m: float | None, extra=(), header: str = "",
             geometry: str = "") -> str:
    """三块拼成一段（写报告用）。"""
    blocks = card_blocks(result, pivot_m=pivot_m, extra=extra, header=header, geometry=geometry)
    return "\n".join(blocks[key] for key in ("pid", "plant", "diag"))


#: 验证轮跟踪误差只看这个频率以下；以上是桨/电机转频一类的振动，速率环不该也跟不上。
#: 与拟合的诊断频段同一个低通（`sysid.fit.lowpass_zero_phase`，二阶零相位）。
TRACKING_BAND_HZ = 15.0


def _rms(values) -> float:
    values = list(values)
    return math.sqrt(sum(v*v for v in values) / len(values)) if values else float("nan")


def tracking_summary(times, samples: list[dict], psi_rad: float, mode: int, *,
                     f1_hz: float | None = None,
                     crossover_hz: float | None = None) -> tuple[str, str]:
    """RATE/ANGLE 验证轮：`(正文, 灰字提示)`。

    跟踪误差 = 均匀重采样后、15 Hz 零相位低通的 (实测 − 指令)，沿杆轴；
    振动噪声 = 原始样本减去低通部分（在原始采样点上算，不经插值——线性插值本身会
    削弱接近奈奎斯特的振动）。两者平方和就是逐点原始均方根。相对值是对**指令均方根**。
    """
    if len(samples) < 16 or len(times) != len(samples):
        return "这一轮数据太少，算不出跟踪误差。", ""
    import importlib

    import numpy as np

    importlib.import_module("._core", __package__)   # 让 tools/ 在 sys.path 上（sysid 包）
    from sysid import fit as fit_module

    c, s = math.cos(psi_rad), math.sin(psi_rad)
    t = np.asarray(times, dtype=float)
    rate = np.asarray([x.get("gx", 0.0)*c + x.get("gy", 0.0)*s for x in samples])
    command = np.asarray([x.get("omega_sp", 0.0) for x in samples])
    error = rate - command
    steps = np.diff(t)
    fs = 1.0 / float(np.median(steps[steps > 0]))
    grid, error_u = fit_module.uniform_signal(t, error, fs)
    _, command_u = fit_module.uniform_signal(t, command, fs)
    low = fit_module.lowpass_zero_phase(error_u, fs, TRACKING_BAND_HZ, 2)
    inside = (t >= t[0] + grid[0]) & (t <= t[0] + grid[-1])
    vibration = error[inside] - np.interp(t[inside], t[0] + grid, low)
    spectrum = np.abs(np.fft.rfft((error_u - low) * np.hanning(len(low))))
    freqs = np.fft.rfftfreq(len(low), 1.0 / fs)
    band = freqs > TRACKING_BAND_HZ
    peak = float(freqs[band][int(np.argmax(spectrum[band]))]) if band.any() else float("nan")

    tracking, command_rms = _rms(low), _rms(command_u)
    relative = (f"（是指令均方根 {command_rms:.3f} rad/s 的 {100.0 * tracking / command_rms:.0f}%，"
                "不是相对幅值）") if command_rms > 1e-6 else ""
    lines = [f"跟踪误差（15 Hz 以下，均方根，沿杆轴）：{tracking:.3f} rad/s{relative}",
             f"振动噪声（15 Hz 以上，均方根）：{_rms(vibration):.3f} rad/s，主频约 {peak:.0f} Hz"
             "（桨/电机一类的振动，速率环不该也跟不上）"]
    if mode == 2 and all("angle_sp" in x and "angle" in x for x in samples):
        angle_error = np.asarray([x["angle_sp"] - x["angle"] for x in samples])
        _, angle_u = fit_module.uniform_signal(t, angle_error, fs)
        angle_low = fit_module.lowpass_zero_phase(angle_u, fs, TRACKING_BAND_HZ, 2)
        lines.append(f"角度跟踪误差（15 Hz 以下，均方根）：{math.degrees(_rms(angle_low)):.2f}°，"
                     f"最大 {math.degrees(float(np.max(np.abs(angle_low)))):.2f}°")
    lines.append("误差越小越好；看「看波形」页虚线（期望）和实线（实测）贴得紧不紧。"
                 "不满意可点「恢复原参数」。")
    note = ""
    if f1_hz is not None and crossover_hz is not None and f1_hz > 3.0 * crossover_hz:
        note = (f"本轮激励最高 {f1_hz:g} Hz，远超预测的速率环穿越频率 {crossover_hz:.2f} Hz，"
                "高频段本来就跟不上；验证速率环建议用双脉冲或低频扫频")
    return "\n".join(lines), note


def vibration_summary(times, samples) -> str:
    """有真实电调转速时：转速中位数、陀螺振动主频与 eRPM→振动比（一段文字）；没有就空串。"""
    if len(samples) < 64 or len(times) != len(samples):
        return ""
    try:
        from ._core import vibration_report
        report = vibration_report(times, samples)
    except (ImportError, ValueError) as error:
        return f"振动主频算不出来：{error}"
    if report is None:
        return ""
    return "【振动与转速】\n" + "\n".join(report.lines())


def gains_card(rate, attitude, commands: list[str]) -> str:
    return (f"建议参数（候选初值，还要在杆上用 RATE/ANGLE 验证）：\n"
            f"角速度环：kp={rate.kp:.4g}  ki={rate.ki:.4g}  kd={rate.kd:.4g}"
            f"（带宽 {rate.bandwidth_hz:.2f} Hz）\n"
            f"角度环：kp={attitude.kp:.4g}（带宽 {attitude.bandwidth_hz:.2f} Hz）\n"
            f"{rate.rationale}\n"
            "将写入：" + "；".join(commands))


__all__ = ["MIN_DELAY_S", "advice_for", "card_blocks", "fit_card", "fit_passes",
           "gains_card", "quality_problems", "tracking_summary", "vibration_summary"]
