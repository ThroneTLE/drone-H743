"""Thrust-bench calibration contract, checked against the legacy pressure GUI.

推力台标定的契约。

每一条都对着旧 `tools/pressure_rs485_gui.py`（1837 行单文件）的一个具体毛病：

* 转速是 `kv*V*pct/100` **算**的（`:250`），报告里和实测推力并排显示，
  看的人分不出哪个是量的。 -> 每个转速源自报 `measured`，没实测就不给 C_T。
* **不按电池电压分层**。 -> 扫描计划支持分层，不分层时报告必须明说。
* 只扫单向，测不出迟滞。 -> 计划默认带回程，`hysteresis()` 把它算出来。
* 标定用分段线性插值精确穿过每个点，把称量误差原样刻进曲线。 -> 改最小二乘
  直线并报残差。
* 停机写在正常流程末尾，中途抛异常就停不了。 -> `SafeRun` 上下文管理器。
"""
from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from thrust_bench import driver, emit, fit, rpm, scale, sweep  # noqa: E402
sys.path.pop(0)


# ---------------------------------------------------------------- 称重


def test_modbus_framing_matches_the_manual():
    request = scale.read_request(0x01, 0x0000, 2)
    assert request[:6] == bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x02])
    assert scale.check_crc(request)


def test_dip_switch_maps_to_the_station_address():
    assert scale.dip_to_addr("1000") == 1
    assert scale.dip_to_addr("0100") == 2
    assert scale.dip_to_addr("1111") == 15
    with pytest.raises(ValueError):
        scale.dip_to_addr("10")


def make_response(addr: int, value: int) -> bytes:
    unsigned = value & 0xFFFFFFFF
    body = bytes([addr, 0x03, 0x04]) + struct.pack(
        ">HH", unsigned & 0xFFFF, (unsigned >> 16) & 0xFFFF)
    return scale.add_crc(body)


def test_a_negative_reading_survives_the_round_trip():
    """秤可以读到负数（去皮之后往下压）。当成无符号会变成 40 亿。"""
    assert scale.parse_signed32(make_response(1, -12345), 1) == -12345


def test_a_corrupt_frame_raises_instead_of_reading_zero():
    """静默的 0 会被当成"秤上没东西"——那恰好是最像正常的读数。"""
    bad = bytearray(make_response(1, 5000))
    bad[-1] ^= 0xFF
    with pytest.raises(ValueError):
        scale.parse_signed32(bytes(bad), 1)
    with pytest.raises(ValueError):
        scale.parse_signed32(make_response(2, 5000), 1)  # 站号不符


def test_calibration_is_a_least_squares_line_not_an_interpolation():
    """分段线性会精确穿过每个点，把一次称量误差原样刻进曲线且看不出来。"""
    calibration = scale.ScaleCalibration(points=[
        (0.0, 0.0), (1000.0, 100.0), (2000.0, 200.0), (3000.0, 305.0),
    ]).fit()
    assert calibration.gain == pytest.approx(0.1013, rel=0.02)
    # 直线不穿过那个有偏差的点——残差正是要能看见的东西。
    assert abs(calibration.grams(3000.0) - 305.0) > 0.5
    assert calibration.residual_g > 0.5


def test_a_suspicious_calibration_says_which_way_it_is_wrong():
    good = scale.ScaleCalibration(points=[
        (0.0, 0.0), (1000.0, 100.0), (2000.0, 200.0)]).fit()
    assert good.suspicious() is None

    bad = scale.ScaleCalibration(points=[
        (0.0, 0.0), (1000.0, 100.0), (2000.0, 260.0)]).fit()
    assert "残差" in (bad.suspicious() or "")


def test_two_points_are_the_minimum():
    with pytest.raises(ValueError):
        scale.ScaleCalibration(points=[(0.0, 0.0)]).fit()


class FakeTransport:
    def __init__(self, values: list[int], addr: int = 1) -> None:
        self.values = list(values)
        self.addr = addr
        self.requests: list[bytes] = []

    def request(self, payload: bytes, expected: int) -> bytes:
        del expected
        self.requests.append(payload)
        return make_response(self.addr, self.values.pop(0))


def test_tare_averages_instead_of_taking_one_sample():
    """单点去皮会把一次抖动永久写进整条曲线。"""
    calibration = scale.ScaleCalibration(points=[(0.0, 0.0), (1000.0, 100.0)]).fit()
    transport = FakeTransport([100, 120, 80, 100])
    cell = scale.LoadCell(transport, 1, calibration)
    assert cell.tare(samples=4) == pytest.approx(100.0)


def test_tare_does_not_touch_the_calibration():
    """装上桨之后的自重每次都不一样；算进标定等于每换一次装配就重标传感器。"""
    calibration = scale.ScaleCalibration(points=[(0.0, 0.0), (1000.0, 100.0)]).fit()
    gain_before = calibration.gain
    transport = FakeTransport([500, 500, 1500])
    cell = scale.LoadCell(transport, 1, calibration)
    cell.tare(samples=2)
    assert calibration.gain == gain_before
    assert cell.read_grams() == pytest.approx(100.0, rel=1e-3)


# ---------------------------------------------------------------- 转速源


def test_no_rpm_returns_none_not_zero():
    """0 会让 F/(ρn²D⁴) 除零或给出 inf，而 inf 传几步就变成一个普通的大数。"""
    assert rpm.NoRpm().rpm(50, 12.0) is None


def test_the_kv_estimate_is_labelled_an_estimate_everywhere():
    source = rpm.KvEstimate(kv_rpm_per_volt=1300.0)
    assert source.measured is False
    assert "非实测" in source.quality
    assert "(est)" in rpm.annotate(source.rpm(50, 12.6), source)


def test_the_kv_estimate_needs_a_voltage():
    assert rpm.KvEstimate(kv_rpm_per_volt=1300.0).rpm(50) is None


def test_dshot_erpm_divides_by_pole_pairs():
    """极对数填错会让转速整体差一个整数倍，而曲线形状完全正常。"""
    source = rpm.DshotErpm(pole_pairs=7)
    source.note(50.0, 14000.0)
    assert source.rpm(50.0) == pytest.approx(2000.0)
    assert source.measured is True
    assert rpm.annotate(source.rpm(50.0), source) == "2000"


def test_dshot_refuses_a_nonsense_pole_pair_count():
    with pytest.raises(ValueError):
        rpm.DshotErpm(pole_pairs=0)


# ---------------------------------------------------------------- 扫描编排


def test_the_plan_includes_a_return_leg_for_hysteresis():
    plan = sweep.plan_sweep(minimum_percent=0, maximum_percent=20,
                            step_percent=10)
    directions = [point.direction for point in plan.points]
    assert directions == ["up", "up", "up", "down", "down"]
    # 最高点不重复：它已经在上行末尾量过，重复只会在迟滞图上多一个零差的点。
    percents = [p.percent for p in plan.points]
    assert percents == [0, 10, 20, 10, 0]


def test_the_plan_can_stratify_by_battery_voltage():
    """同一档油门在 12.6 V 和 11.1 V 下推力能差 20%。"""
    plan = sweep.plan_sweep(minimum_percent=0, maximum_percent=10,
                            step_percent=10, voltage_layers=(12.6, 11.1),
                            include_return=False)
    assert plan.voltage_layers() == (12.6, 11.1)
    assert len(plan.points) == 4


def test_every_point_waits_for_the_propeller_to_settle():
    """下发之后立刻读到的是过渡过程，而过渡过程的推力**总是偏小**。"""
    plan = sweep.plan_sweep(maximum_percent=10, step_percent=10)
    assert all(point.settle_s > 0.5 for point in plan.points)
    assert all(point.samples >= 4 for point in plan.points)
    assert plan.duration_s > 0.0


def test_a_nonsense_plan_is_refused():
    with pytest.raises(ValueError):
        sweep.plan_sweep(step_percent=0)
    with pytest.raises(ValueError):
        sweep.plan_sweep(minimum_percent=80, maximum_percent=20)


def measurement(percent, direction, thrust, **kwargs):
    return sweep.Measurement(percent=percent, direction=direction,
                             pulse_us=driver.percent_to_pulse(percent),
                             thrust_g=thrust, samples=8, **kwargs)


def test_hysteresis_is_the_difference_between_the_two_legs():
    data = [measurement(50, "up", 400.0), measurement(50, "down", 380.0)]
    assert sweep.hysteresis(data) == {50: pytest.approx(20.0)}


def test_voltage_sag_is_recorded_per_point():
    data = [measurement(80, "up", 900.0, voltage_v=11.6, voltage_layer_v=12.6)]
    assert sweep.voltage_sag(data) == {80: pytest.approx(1.0)}


def test_an_unsteady_reading_flags_itself():
    noisy = sweep.Measurement(percent=50, direction="up", pulse_us=1500,
                              thrust_g=400.0, samples=8, spread_g=40.0)
    assert "还没稳" in (noisy.suspicious or "")
    steady = sweep.Measurement(percent=50, direction="up", pulse_us=1500,
                               thrust_g=400.0, samples=8, spread_g=5.0)
    assert steady.suspicious is None


# ---------------------------------------------------------------- 驱动


class FakeLink:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def send_line(self, text: str) -> bool:
        self.lines.append(text)
        return True


def test_the_flight_controller_path_uses_the_heartbeat_protected_window():
    """裸电机命令发出去之后飞控会一直保持，哪怕对面的程序已经不在了。"""
    link = FakeLink()
    channel = driver.FlightControllerDriver(link)
    channel.arm()
    channel.set_percent(1, 42.4)
    assert link.lines[0] == "PROPCAL SPIN ARM confirm=safe"
    assert link.lines[1] == "PROPCAL SPIN SET ch=1 pct=42"


def test_the_confirm_token_matches_the_firmware():
    source = (ROOT / "App/Src/app_cmd_propcal.c").read_text(encoding="utf-8")
    assert f'"{driver.SPIN_CONFIRM_TOKEN}"' in source


def test_stopping_is_sent_twice():
    link = FakeLink()
    assert driver.FlightControllerDriver(link).stop_all()
    assert link.lines == ["PROPCAL SPIN STOP", "PROPCAL SPIN STOP"]


def test_an_exception_still_stops_the_motors():
    """旧那套的停机写在正常流程末尾，中途抛异常就停不了。

    而中途抛异常正是最需要停机的时候。
    """
    link = FakeLink()
    channel = driver.FlightControllerDriver(link)
    with pytest.raises(RuntimeError):
        with driver.SafeRun(channel):
            channel.set_percent(1, 80)
            raise RuntimeError("台架上出事了")
    assert link.lines[-1] == "PROPCAL SPIN STOP"


def test_percent_to_pulse_matches_the_firmware_range():
    assert driver.percent_to_pulse(0) == 1100
    assert driver.percent_to_pulse(100) == 1940
    assert driver.percent_to_pulse(50) == 1520
    assert driver.percent_to_pulse(-10) == 1100, "越界必须夹住"
    assert driver.percent_to_pulse(150) == 1940


# ---------------------------------------------------------------- 拟合


def test_trimmed_mean_ignores_a_knock_on_the_bench():
    values = [400.0, 401.0, 399.0, 400.5, 900.0]
    assert fit.trimmed_mean(values) == pytest.approx(400.5, abs=1.0)


def test_the_thrust_curve_is_quadratic_because_the_physics_is():
    """推力 ∝ 转速²、油门 ∝ 转速，所以推力对油门本来就是二次的。

    直线拟合会在两端各偏一截，而两端正是悬停点和满油。
    """
    truth = lambda p: 0.16 * p * p + 0.5 * p  # noqa: E731
    data = [measurement(p, "up", truth(p)) for p in range(0, 101, 5)]
    curve = fit.fit_thrust_curve(data)
    assert curve.a == pytest.approx(0.16, rel=1e-6)
    assert curve.b == pytest.approx(0.5, abs=1e-6)
    assert curve.r_squared > 0.999
    assert curve.newtons(100) == pytest.approx(truth(100) * 9.80665 / 1000.0)


def test_the_curve_only_uses_the_up_leg():
    """回程点带着迟滞，混进拟合会把曲线整体拉低一点，而且看不出来。"""
    data = [measurement(p, "up", 0.16 * p * p) for p in range(0, 101, 10)]
    data += [measurement(p, "down", 0.10 * p * p) for p in range(0, 101, 10)]
    curve = fit.fit_thrust_curve(data)
    assert curve.a == pytest.approx(0.16, rel=1e-6)


def test_three_points_is_the_minimum_for_a_quadratic():
    with pytest.raises(ValueError):
        fit.fit_thrust_curve([measurement(0, "up", 0.0),
                              measurement(50, "up", 400.0)])


def test_no_thrust_coefficient_without_a_measured_rpm():
    """转速是 C_T 里的平方项。kv 那个 0.8 的带载系数实际在 0.6~0.9 之间飘，

    于是 C_T 会差出两倍多——而两倍的 C_T 足以让一架按图纸能飞的飞机飞不起来。
    """
    estimate = rpm.KvEstimate(kv_rpm_per_volt=1300.0)
    assert fit.thrust_coefficient(400.0, 8000.0, 0.2286, estimate) is None

    measured = rpm.DshotErpm(pole_pairs=7)
    value = fit.thrust_coefficient(400.0, 8000.0, 0.2286, measured)
    assert value is not None and 0.0 < value < 1.0


def test_the_voltage_exponent_is_fitted_not_assumed_to_be_two():
    """电调限流、桨失速、电池内阻都会把指数拉低；写死 2 会在低电压端高估推力。"""
    exponent = 1.7
    reference = 12.6
    layers = {}
    for voltage in (12.6, 11.8, 11.1):
        thrust = 900.0 * (voltage / reference) ** exponent
        layers[voltage] = [measurement(80, "up", thrust, voltage_v=voltage,
                                       voltage_layer_v=voltage)]
    model = fit.fit_voltage_model(layers, 80)
    assert model is not None
    assert model.exponent == pytest.approx(exponent, rel=0.02)
    assert model.scale(11.1) == pytest.approx((11.1 / 12.6) ** exponent, rel=1e-6)


def test_one_voltage_layer_gives_no_model_instead_of_pretending():
    layers = {12.6: [measurement(80, "up", 900.0, voltage_v=12.6,
                                 voltage_layer_v=12.6)]}
    assert fit.fit_voltage_model(layers, 80) is None


# ---------------------------------------------------------------- 产出


def sample_run():
    return [measurement(p, "up", 0.16 * p * p, voltage_v=12.4,
                        voltage_layer_v=12.6) for p in range(0, 101, 10)]


def test_the_csv_records_whether_the_rpm_was_measured(tmp_path):
    """数据文件会被别的脚本读走，而那些脚本看不到报告。"""
    source = rpm.KvEstimate(kv_rpm_per_volt=1300.0)
    path = emit.write_csv(sample_run(), tmp_path / "s.csv", source)
    rows = path.read_text(encoding="utf-8").splitlines()
    assert "rpm_measured" in rows[0]
    assert "rpm_source" in rows[0]
    assert rows[1].split(",")[8] == "0"
    assert "非实测" in rows[1]


def test_the_report_says_the_rpm_is_an_estimate(tmp_path):
    source = rpm.KvEstimate(kv_rpm_per_volt=1300.0)
    text = emit.write_report(sample_run(), tmp_path / "r.md",
                             source=source).read_text(encoding="utf-8")
    assert "推算，非实测" in text
    assert "不得" in text and "C_T" in text


def test_the_report_complains_when_nothing_was_stratified(tmp_path):
    data = [measurement(p, "up", 0.16 * p * p) for p in range(0, 101, 10)]
    text = emit.write_report(data, tmp_path / "r.md",
                             source=rpm.NoRpm()).read_text(encoding="utf-8")
    assert "未做电压分层" in text


def test_the_report_says_when_there_is_no_return_leg(tmp_path):
    text = emit.write_report(sample_run(), tmp_path / "r.md",
                             source=rpm.NoRpm()).read_text(encoding="utf-8")
    assert "没有回程数据" in text


def test_the_report_shows_hysteresis_when_there_is_a_return_leg(tmp_path):
    data = sample_run() + [measurement(p, "down", 0.15 * p * p)
                           for p in range(0, 101, 10)]
    text = emit.write_report(data, tmp_path / "r.md",
                             source=rpm.NoRpm()).read_text(encoding="utf-8")
    assert "迟滞" in text
    assert "没有回程数据" not in text


def test_the_generated_header_records_the_calibration_conditions(tmp_path):
    """表本身只是一串数字；没有那行注释就没人知道它适不适用。"""
    curve = fit.fit_thrust_curve(sample_run())
    text = emit.write_c_header(curve, tmp_path / "t.h",
                               source=rpm.KvEstimate(kv_rpm_per_volt=1300.0),
                               voltage_layer_v=12.6).read_text(encoding="utf-8")
    assert "请勿手改" in text
    assert "推算，非实测" in text
    assert "12.60 V" in text
    assert "THRUST_TABLE_POINTS 21U" in text
    assert text.count("f,") >= 21


def test_the_generated_header_is_monotonic_and_starts_at_zero(tmp_path):
    curve = fit.fit_thrust_curve(sample_run())
    text = emit.write_c_header(curve, tmp_path / "t.h",
                               source=rpm.NoRpm()).read_text(encoding="utf-8")
    body = text.split("thrust_table_g[THRUST_TABLE_POINTS] = {")[1].split("};")[0]
    values = [float(token.strip().rstrip("f,")) for token in body.split()
              if token.strip().rstrip("f,")]
    assert len(values) == 21
    assert values[0] == pytest.approx(0.0, abs=1e-6)
    assert all(b >= a - 1e-6 for a, b in zip(values, values[1:]))


# ---------------------------------------------------------------- 迁移


def test_the_new_package_does_not_depend_on_the_old_single_file_gui():
    """旧文件保留做历史参考（注释里提它没问题），但代码上不许依赖它。"""
    import ast

    for module in ("scale", "sweep", "fit", "emit", "rpm", "driver", "ui"):
        tree = ast.parse((ROOT / f"tools/thrust_bench/{module}.py").read_text(
            encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            assert not any("pressure_rs485" in name for name in names), module


def test_no_module_outside_the_ui_imports_tkinter():
    """判据必须能在没有窗口、没有硬件的情况下复核。"""
    for module in ("scale", "sweep", "fit", "emit", "rpm", "driver"):
        source = (ROOT / f"tools/thrust_bench/{module}.py").read_text(encoding="utf-8")
        assert "tkinter" not in source, module
