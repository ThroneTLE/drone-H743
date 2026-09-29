from __future__ import annotations

import json
import math
from dataclasses import replace
from pathlib import Path
import sys

import pytest
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from thrust_bench.model import analyze_samples, predict_current, predict_thrust  # noqa: E402
from thrust_bench.model_report import write_analysis  # noqa: E402
from thrust_bench.records import BenchSample, FcSnapshot  # noqa: E402
from thrust_bench.acquisition import AcquisitionEngine  # noqa: E402
from thrust_bench.plans import dynamic_steps, grid  # noqa: E402
from thrust_bench import fit, sweep  # noqa: E402
sys.path.pop(0)


def sample(run: str, segment: str, upper: float, lower: float, thrust: float,
           *, index: int = 0, layer: float = 12.0, current: float | None = 4.0,
           calibrated: bool = True, phase: str = "steady", direction: str = "steady",
           upper_command: float | None = None, lower_command: float | None = None,
           time_s: float | None = None, quality: tuple[str, ...] = ()) -> BenchSample:
    timestamp = float(index) * 0.05 if time_s is None else time_s
    return BenchSample(
        run_id=run, segment_id=segment, host_time_s=timestamp,
        fc_time_ms=round(timestamp * 1000), scale_time_s=timestamp,
        direction=direction, phase=phase, upper_command_pct=upper_command,
        lower_command_pct=lower_command, upper_erpm=upper, lower_erpm=lower,
        thrust_n=thrust, voltage_v=layer,
        upper_esc_current_a=None if current is None else current * 0.4,
        lower_esc_current_a=None if current is None else current * 0.6,
        upper_erpm_age_ms=10, lower_erpm_age_ms=10, voltage_age_ms=10,
        upper_esc_current_age_ms=10, lower_esc_current_age_ms=10,
        board_current_a=current, board_current_age_ms=10,
        board_current_calibrated=calibrated,
        voltage_layer_v=layer, speed_source="dshot_erpm",
        current_source="dshot", esc_current_calibrated=False, quality=quality)


def identifiable_steady(*, calibrated: bool = True) -> list[BenchSample]:
    result: list[BenchSample] = []
    combinations = [(2000, 2500), (4000, 2500), (6500, 2500),
                    (2000, 5000), (4000, 5000), (6500, 5000),
                    (2000, 7500), (4000, 7500), (6500, 7500)]
    for run_index, run in enumerate(("run-a", "run-b")):
        layer = 12.6 if run_index == 0 else 11.1
        for point_index, (upper, lower) in enumerate(combinations):
            thrust = 0.2 + 2.0e-8 * upper ** 2 + 1.5e-8 * lower ** 2 + 0.8e-8 * upper * lower
            for repeat in range(4):
                result.append(sample(run, f"p{point_index}", upper, lower,
                                     thrust + (repeat - 1.5) * 0.001, index=len(result),
                                     layer=layer, current=1.0 + (upper + lower) / 3000.0,
                                     calibrated=calibrated))
    return result


def test_known_two_input_static_model_and_grouped_validation_are_recovered():
    analysis = analyze_samples(identifiable_steady())
    assert analysis["static"]["status"] == "available"
    assert analysis["static"]["rank"] == 4
    assert analysis["static"]["training_metrics"]["rmse_n"] < 0.005
    assert analysis["validation"]["status"] == "training_only"
    assert analysis["static"]["voltage_models"]["dual"]["status"] == "available"


def test_same_throttle_line_is_rejected_instead_of_pseudo_2d_fit():
    samples = []
    for point_index, erpm in enumerate(range(1000, 8000, 1000)):
        for repeat in range(3):
            samples.append(sample("one", str(point_index), erpm, erpm,
                                  3.0e-8 * erpm ** 2, index=len(samples)))
    static = analyze_samples(samples)["static"]
    assert static["status"] == "unavailable"
    assert static["erpm_only_baseline"]["reason"] == "inputs_not_independently_varied"


def test_uncalibrated_dshot_current_still_builds_observation_models():
    analysis = analyze_samples(identifiable_steady(calibrated=False))
    power = analysis["power"]
    assert power["status"] == "available"
    assert power["external_calibration_verified"] is False
    assert power["observations"]
    assert power["upper_current_model"]["status"] == "available"
    assert power["lower_current_model"]["status"] == "available"
    assert predict_current(analysis, upper_erpm=4000, lower_erpm=5000,
                           voltage_v=12.0, role="upper") > 0


def test_power_never_pairs_voltage_and_current_from_different_samples():
    samples = identifiable_steady()
    alternating = [replace(item, upper_esc_current_a=None) if index % 2 else
                   replace(item, lower_esc_current_a=None)
                   for index, item in enumerate(samples)]
    power = analyze_samples(alternating)["power"]
    assert power["combined_input_power_model"]["status"] == "unavailable"


def test_one_dshot_current_route_can_fit_without_faking_combined_power():
    samples = [replace(item, lower_esc_current_a=None) for item in identifiable_steady()]
    analysis = analyze_samples(samples)
    assert analysis["static"]["voltage_models"]["dual"]["status"] == "available"
    power = analysis["power"]
    assert power["upper_current_model"]["status"] == "available"
    assert power["lower_current_model"]["status"] == "unavailable"
    assert power["combined_input_power_model"]["status"] == "unavailable"


def test_dshot_current_models_include_loaded_voltage_and_require_cross_route_alignment():
    samples = [replace(item,
                       upper_esc_current_a=(2.0 if item.voltage_v < 12 else 3.0) +
                       float(item.upper_erpm) / 10000.0,
                       lower_esc_current_a=(1.0 if item.voltage_v < 12 else 2.0) +
                       float(item.lower_erpm) / 10000.0)
               for item in identifiable_steady()]
    analysis = analyze_samples(samples, {"current_resolution_a": 1.0})
    low = predict_current(analysis, upper_erpm=4000, lower_erpm=5000,
                          voltage_v=11.2, role="upper")
    high = predict_current(analysis, upper_erpm=4000, lower_erpm=5000,
                           voltage_v=12.4, role="upper")
    assert high > low
    assert analysis["power"]["current_resolution_a"] == 1.0
    skewed = [replace(item, lower_esc_current_age_ms=900) for item in samples]
    power = analyze_samples(skewed)["power"]
    assert power["upper_current_model"]["status"] == "available"
    assert power["lower_current_model"]["status"] == "unavailable"
    assert power["combined_input_power_model"]["status"] == "unavailable"


def test_current_response_coordinates_use_only_the_aligned_valid_subset():
    samples = []
    for index, item in enumerate(identifiable_steady()):
        within = index % 4
        samples.append(replace(
            item, upper_erpm=float(item.upper_erpm) + within * 10.0,
            voltage_v=float(item.voltage_v) + within * 0.01,
            upper_esc_current_a=item.upper_esc_current_a if within < 2 else None))
    analysis = analyze_samples(samples)
    point = analysis["static"]["operating_points"][0]
    response = point["response_points"]["upper_esc_current_a"]
    assert response["upper_erpm"] == pytest.approx(float(point["upper_erpm"]) - 10.0)
    assert response["voltage_v"] == pytest.approx(float(point["voltage_v"]) - 0.01)
    assert response["sample_count"] == 2


def test_saved_v1_analysis_is_rejected_by_public_predictors():
    legacy = {"schema_version": 1, "static": {}, "power": {}}
    with pytest.raises(ValueError, match="v2 electrical eRPM"):
        predict_thrust(legacy, upper_erpm=1, lower_erpm=1, voltage_v=12)
    with pytest.raises(ValueError, match="v2 electrical eRPM"):
        predict_current(legacy, upper_erpm=1, lower_erpm=1, voltage_v=12)


def test_board_adc_current_is_diagnostic_and_never_a_fitting_fallback():
    samples = [replace(item, upper_esc_current_a=None, lower_esc_current_a=None,
                       board_current_a=99.0, board_current_calibrated=True)
               for item in identifiable_steady()]
    analysis = analyze_samples(samples)
    assert analysis["static"]["voltage_models"]["dual"]["status"] == "available"
    assert analysis["power"]["status"] == "unavailable"
    assert analysis["power"]["board_current_diagnostic_only"] is True
    assert all(point["board_current_a_diagnostic"] == 99.0
               for point in analysis["power"]["observations"])


def test_wrong_domain_source_and_legacy_mechanical_fields_are_rejected():
    samples = identifiable_steady()
    with pytest.raises(ValueError, match="unsupported_speed_domain"):
        analyze_samples(samples, {"speed_domain": "mechanical_rpm"})
    with pytest.raises(ValueError, match="unsupported_current_source"):
        analyze_samples([replace(samples[0], current_source="board_adc")])
    legacy = SimpleNamespace(upper_rpm=1000.0, lower_rpm=1000.0,
                             speed_source="dshot_erpm", current_source="dshot")
    with pytest.raises(ValueError, match="unsupported_sample_schema"):
        analyze_samples([legacy])


def test_segment_power_is_mean_of_synchronised_products():
    samples = identifiable_steady()
    varied = [replace(item, voltage_v=10.0 if index % 2 else 20.0,
                      upper_esc_current_a=1.0 if index % 2 else 0.5,
                      lower_esc_current_a=1.0 if index % 2 else 0.5)
              for index, item in enumerate(samples)]
    power = analyze_samples(varied)["power"]
    assert power["observations"]
    assert all(point["input_power_w"] == pytest.approx(20.0)
               for point in power["observations"] if point["input_power_w"] is not None)


def test_repeated_old_source_updates_do_not_form_a_steady_point():
    repeated = [replace(sample("r", "s", 3000, 4000, 2.0, index=index),
                        host_time_s=1.0, fc_time_ms=100, scale_time_s=1.0)
                for index in range(4)]
    quality = analyze_samples(repeated)["static"]["data_quality"]
    assert quality["excluded_samples_by_reason"]["too_few_independent_source_updates"] == 4


def test_dirty_negative_erpm_age_and_nan_scale_time_are_rejected():
    dirty = [replace(sample("r", "s", 1000, 1000, 1.0, index=0), upper_erpm=-1),
             replace(sample("r", "s", 1000, 1000, 1.0, index=1), upper_erpm_age_ms=-1),
             replace(sample("r", "s", 1000, 1000, 1.0, index=2), scale_time_s=float("nan"))]
    excluded = analyze_samples(dirty)["static"]["data_quality"]["excluded_samples_by_reason"]
    assert excluded["negative_erpm"] == 1
    assert excluded["erpm_stale_or_age_missing"] == 1
    assert excluded["scale_not_synchronised"] == 1


def test_single_drive_installed_points_are_not_mixed_into_dual_surface():
    samples = [replace(item, mode="upper") for item in identifiable_steady()]
    static = analyze_samples(samples)["static"]
    assert static["status"] == "partial"
    assert static["coverage"]["points"] == 0
    assert static["single_drive_installed_baselines"]["status"] == "available"
    assert "windmill" in static["single_drive_installed_baselines"]["interpretation"]


def test_rank_deficient_result_is_strict_json_without_infinity():
    samples = []
    for point_index, erpm in enumerate(range(1000, 8000, 1000)):
        for repeat in range(3):
            samples.append(sample("one", str(point_index), erpm, erpm,
                                  3.0e-8 * erpm ** 2, index=len(samples)))
    json.dumps(analyze_samples(samples), allow_nan=False)


def dynamic_segment(run: str, segment: str, channel: str, *, rise: bool,
                    rate_hz: float = 20.0, tau: float = 0.18, delay: float = 0.08) -> list[BenchSample]:
    result = []
    dt = 1.0 / rate_hz
    y0, y1 = ((1200.0, 6200.0) if rise else (6200.0, 1200.0))
    step_time = 0.15
    for index in range(round(1.5 * rate_hz)):
        time_s = index * dt
        elapsed = max(time_s - step_time - delay, 0.0)
        erpm = y0 + (y1 - y0) * (1.0 - math.exp(-elapsed / tau))
        upper = erpm if channel == "upper" else 3000.0
        lower = erpm if channel == "lower" else 3000.0
        result.append(sample(run, segment, upper, lower, 5.0, phase="dynamic",
                             direction="up" if rise else "down", time_s=time_s,
                             upper_command=(70.0 if time_s >= step_time else 20.0)
                                           if channel == "upper" else 30.0,
                             lower_command=(70.0 if time_s >= step_time else 20.0)
                                           if channel == "lower" else 30.0))
    return result


def test_dynamic_parameters_recover_by_channel_and_rise_fall():
    samples = (dynamic_segment("r1", "upper-rise", "upper", rise=True) +
               dynamic_segment("r1", "lower-fall", "lower", rise=False))
    dynamics = analyze_samples(samples)["dynamics"]
    assert dynamics["status"] == "available"
    fits = {(item["channel"], item["direction"]): item for item in dynamics["fits"]}
    assert fits[("upper", "rise")]["time_constant_s"] == pytest.approx(0.18, abs=0.04)
    assert fits[("upper", "rise")]["delay_s"] == pytest.approx(0.08, abs=0.04)
    assert fits[("lower", "fall")]["time_constant_s"] == pytest.approx(0.18, abs=0.04)


def test_low_rate_dynamic_data_is_explicitly_rejected():
    dynamics = analyze_samples(dynamic_segment("r", "slow", "upper", rise=True,
                                                rate_hz=5.0))["dynamics"]
    assert dynamics["status"] == "unavailable"
    assert any(item["reason"] == "effective_sample_rate_below_10_hz"
               for item in dynamics["rejected"])


def test_response_faster_than_two_real_updates_is_resolution_limited():
    dynamics = analyze_samples(dynamic_segment("r", "too-fast", "upper", rise=True,
                                                rate_hz=20.0, tau=0.01,
                                                delay=0.0))["dynamics"]
    assert not any(item.get("channel") == "upper" for item in dynamics.get("fits", []))
    assert any(item["channel"] == "upper" and
               item["reason"] == "time_constant_below_2_update_intervals"
               for item in dynamics["rejected"])


def test_missing_or_stale_samples_are_counted_not_filled_with_zero():
    bad = sample("r", "s", 1000, 1000, 1.0, quality=("scale_crc_error",))
    analysis = analyze_samples([bad])
    assert analysis["static"]["status"] == "unavailable"
    assert analysis["static"]["data_quality"]["excluded_samples_by_reason"]["scale_crc_error"] == 1


def test_report_and_json_are_reviewable_and_never_emit_nan(tmp_path):
    output = tmp_path / "new-analysis"
    paths = write_analysis(identifiable_steady(), {
        "schema_version": 2, "firmware_id": "synthetic-unit-test",
        "both_propellers_installed": True, "undriven_propeller_state": "windmilling_unknown",
    }, output)
    parsed = json.loads(paths["model_json"].read_text(encoding="utf-8"))
    assert parsed["schema_version"] == 2
    text = paths["report"].read_text(encoding="utf-8")
    assert "两片桨均保持安装" in text
    assert "不能把任何系数解释为孤立单桨" in text
    assert "synthetic-unit-test" in text
    with pytest.raises(FileExistsError):
        write_analysis([], {}, output)


def legacy_measurement(percent: float, direction: str, thrust: float,
                       layer: float | None) -> sweep.Measurement:
    return sweep.Measurement(percent, direction, 1000 + round(percent * 10), thrust,
                             voltage_layer_v=layer, samples=8)


def test_legacy_curve_refuses_to_mix_voltage_layers():
    values = [legacy_measurement(p, "up", p * p, layer)
              for layer in (12.6, 11.1) for p in (10, 20, 30)]
    with pytest.raises(ValueError, match="多个电压层"):
        fit.fit_thrust_curve(values)


def test_legacy_hysteresis_keeps_voltage_layers_separate():
    values = [legacy_measurement(50, "up", 100, 12.6),
              legacy_measurement(50, "down", 90, 12.6),
              legacy_measurement(50, "up", 80, 11.1),
              legacy_measurement(50, "down", 75, 11.1)]
    assert sweep.hysteresis(values) == {(11.1, 50): 5, (12.6, 50): 10}


def voltage_dependent_dual(voltages=(10.0, 12.0)) -> list[BenchSample]:
    result = []
    combinations = [(2000, 2500), (4000, 2500), (6500, 2500),
                    (2000, 5000), (4000, 5000), (6500, 5000),
                    (2000, 7500), (4000, 7500), (6500, 7500)]
    for voltage in voltages:
        for run_repeat in range(2):
            run = f"v{voltage:g}-r{run_repeat}"
            for point_index, (upper, lower) in enumerate(combinations):
                base = 0.2 + 2.0e-8 * upper ** 2 + 1.5e-8 * lower ** 2 + 0.8e-8 * upper * lower
                thrust = base * (1.0 + 0.08 * (voltage - 10.0))
                for repeat in range(4):
                    result.append(sample(run, f"p{point_index}", upper, lower,
                                         thrust + repeat * 0.0001, index=len(result),
                                         layer=voltage,
                                         upper_command=upper / 100.0,
                                         lower_command=lower / 100.0))
    return result


def voltage_dependent_single(mode: str) -> list[BenchSample]:
    result = []
    for voltage in (10.0, 12.0):
        for erpm_index, erpm in enumerate((1500, 3000, 4500, 6000, 7500)):
            thrust = 0.1 + 2.2e-8 * erpm ** 2 * (1.0 + 0.1 * (voltage - 10.0))
            for repeat in range(4):
                item = sample(f"{mode}-v{voltage:g}", f"p{erpm_index}",
                              erpm if mode == "upper" else 2500,
                              erpm if mode == "lower" else 2500,
                              thrust, index=len(result), layer=voltage)
                if mode == "upper":
                    item = replace(item, mode=mode, lower_erpm=None,
                                   lower_erpm_age_ms=None, lower_command_pct=None)
                else:
                    item = replace(item, mode=mode, upper_erpm=None,
                                   upper_erpm_age_ms=None, upper_command_pct=None)
                result.append(item)
    return result


def test_explicit_voltage_model_changes_prediction_at_same_measured_erpm():
    analysis = analyze_samples(voltage_dependent_dual())
    model = analysis["static"]["voltage_models"]["dual"]
    assert model["status"] == "available"
    low = predict_thrust(analysis, upper_erpm=4000, lower_erpm=5000, voltage_v=10.0)
    high = predict_thrust(analysis, upper_erpm=4000, lower_erpm=5000, voltage_v=12.0)
    middle = predict_thrust(analysis, upper_erpm=4000, lower_erpm=5000, voltage_v=11.0)
    assert high > low
    assert middle == pytest.approx((low + high) / 2, rel=1e-6)
    assert analysis["validation"]["status"] == "available"
    assert all(item["group"] == "run" for item in analysis["validation"]["holdouts"])


def test_voltage_layer_centers_come_from_loaded_voltage_not_labels():
    relabelled = [replace(item, voltage_layer_v=100.0 if item.voltage_v == 10.0 else 200.0)
                  for item in voltage_dependent_dual()]
    model = analyze_samples(relabelled)["static"]["voltage_models"]["dual"]
    assert model["voltage_range_v"] == pytest.approx([10.0, 12.0])
    assert [layer["voltage_center_v"] for layer in model["layers"]] == pytest.approx([10.0, 12.0])


def test_whole_voltage_layer_holdout_is_reported_without_segment_leakage():
    analysis = analyze_samples(voltage_dependent_dual((10.0, 11.0, 12.0)))
    holdouts = analysis["static"]["voltage_models"]["dual"]["validation"]["holdouts"]
    layer_holdouts = [item for item in holdouts if item["group"] == "voltage_layer"]
    assert len(layer_holdouts) == 3
    middle = next(item for item in layer_holdouts if item["value"] == "layer:11")
    assert middle["status"] == "available"
    assert middle["test_points"] == 18


def test_report_presents_voltage_model_as_primary_with_chinese_usage(tmp_path):
    paths = write_analysis(voltage_dependent_dual(), {
        "schema_version": 2, "both_propellers_installed": True,
    }, tmp_path / "voltage-report")
    text = paths["report"].read_text(encoding="utf-8")
    assert "主要标定结果" in text
    assert "T_total = f(eRPM_upper, eRPM_lower, V_loaded)" in text
    assert "predict_thrust" in text
    assert "不使用 KV 估算兜底" in text


def test_voltage_prediction_rejects_extrapolation_and_missing_measured_erpm():
    analysis = analyze_samples(voltage_dependent_dual())
    with pytest.raises(ValueError, match="voltage_outside"):
        predict_thrust(analysis, upper_erpm=4000, lower_erpm=5000, voltage_v=9.0)
    with pytest.raises(ValueError, match="measured_dual_erpm_required"):
        predict_thrust(analysis, upper_erpm=None, lower_erpm=5000, voltage_v=11.0)
    estimated = [replace(item, speed_source="kv_estimate") for item in voltage_dependent_dual()]
    with pytest.raises(ValueError, match="unsupported_speed_source"):
        analyze_samples(estimated)


def test_single_voltage_layer_and_nonoverlapping_erpm_layers_are_unavailable():
    one_layer = [item for item in voltage_dependent_dual() if item.voltage_layer_v == 10.0]
    assert analyze_samples(one_layer)["static"]["voltage_models"]["dual"]["reason"] == (
        "fewer_than_2_measured_voltage_layers")
    separated = []
    for voltage, offset in ((10.0, 0), (12.0, 10000)):
        for i, (upper, lower) in enumerate(((1000, 1000), (2000, 1000), (3000, 1000),
                                            (1000, 2000), (2000, 2000), (3000, 2000))):
            for repeat in range(3):
                separated.append(sample(f"v{voltage}", str(i), upper + offset,
                                         lower + offset, 1.0 + i, index=len(separated),
                                         layer=voltage))
    result = analyze_samples(separated)["static"]["voltage_models"]["dual"]
    assert result["status"] == "unavailable"
    assert result["reason"] == "no_overlapping_erpm_coverage_across_voltage_layers"


def test_voltage_layers_reject_wide_span_and_overlapping_real_ranges():
    base = voltage_dependent_dual()
    wide = [replace(item, voltage_v=(9.6 if index % 2 else 10.4))
            if item.voltage_layer_v == 10.0 else item
            for index, item in enumerate(base)]
    assert analyze_samples(wide)["static"]["voltage_models"]["dual"]["reason"] == (
        "voltage_layer_span_too_wide")
    overlapping = []
    for index, item in enumerate(base):
        odd = int(item.segment_id[1:]) % 2
        actual = (10.0 if odd else 10.4) if item.voltage_layer_v == 10.0 else (
            10.3 if odd else 10.7)
        overlapping.append(replace(item, voltage_v=actual))
    model = analyze_samples(overlapping)["static"]["voltage_models"]["dual"]
    assert model["status"] == "unavailable"
    assert model["reason"] == "measured_voltage_layer_ranges_overlap"


def triangular_layers(second_triangle=False):
    first = [(1000, 1000), (9000, 1000), (1000, 9000),
             (5000, 1000), (1000, 5000), (4000, 4000)]
    second = ([(9000, 9000), (9000, 2000), (2000, 9000),
               (9000, 5000), (5000, 9000), (6000, 6000)]
              if second_triangle else first)
    samples = []
    for voltage, points in ((10.0, first), (12.0, second)):
        for index, (upper, lower) in enumerate(points):
            thrust = 0.1 + 2e-8 * upper ** 2 + 1e-8 * lower ** 2 + 1e-8 * upper * lower
            for repeat in range(3):
                samples.append(sample(f"v{voltage}", str(index), upper, lower,
                                      thrust, index=len(samples), layer=voltage,
                                      upper_command=30, lower_command=30))
    return samples


def test_rectangle_overlap_without_convex_hull_overlap_is_unavailable():
    result = analyze_samples(triangular_layers(second_triangle=True))["static"]["voltage_models"]["dual"]
    assert result["status"] == "unavailable"
    assert result["reason"] == "no_overlapping_erpm_coverage_across_voltage_layers"


def test_query_inside_minmax_rectangle_but_outside_hulls_is_rejected():
    analysis = analyze_samples(triangular_layers())
    assert analysis["static"]["voltage_models"]["dual"]["status"] == "available"
    with pytest.raises(ValueError, match="convex_hulls"):
        predict_thrust(analysis, upper_erpm=8000, lower_erpm=8000, voltage_v=11.0)


@pytest.mark.parametrize("mode", ["upper", "lower"])
def test_single_drive_voltage_model_uses_only_active_measured_erpm(mode):
    inactive="lower" if mode=="upper" else "upper"
    data=[replace(item,quality=(f"{inactive}_erpm_missing",)) for item in voltage_dependent_single(mode)]
    analysis = analyze_samples(data)
    model = analysis["static"]["voltage_models"][mode]
    assert model["status"] == "available"
    kwargs = {"upper_erpm": 4500 if mode == "upper" else None,
              "lower_erpm": 4500 if mode == "lower" else None,
              "voltage_v": 11.0, "mode": mode}
    assert predict_thrust(analysis, **kwargs) > 0
    assert "windmill" in analysis["static"]["single_drive_installed_baselines"]["interpretation"]
    power = analysis["power"]["current_models"][mode]
    assert analysis["power"]["status"] == "partial"
    assert power[mode]["status"] == "available"
    predicted = predict_current(
        analysis, upper_erpm=4500 if mode == "upper" else None,
        lower_erpm=4500 if mode == "lower" else None,
        voltage_v=11.0, role=mode, mode=mode)
    assert predicted > 0


def test_single_drive_nonzero_inactive_command_and_dual_zero_boundary_are_excluded():
    single = [replace(item, lower_command_pct=1.0)
              for item in voltage_dependent_single("upper")]
    quality = analyze_samples(single)["static"]["data_quality"]["excluded_samples_by_reason"]
    assert quality["single_drive_inactive_command_nonzero"] == len(single)
    boundary = [sample("r", "zero", 0, 3000, 1.0, index=index,
                       upper_command=0, lower_command=30) for index in range(3)]
    quality = analyze_samples(boundary)["static"]["data_quality"]["excluded_samples_by_reason"]
    assert quality["dual_undriven_zero_erpm_boundary"] == 3


def test_single_only_report_is_partial_and_shows_successful_mode(tmp_path):
    analysis = analyze_samples(voltage_dependent_single("upper"))
    assert analysis["static"]["status"] == "partial"
    paths = write_analysis(voltage_dependent_single("upper"), {},
                           tmp_path / "single-only")
    text = paths["report"].read_text(encoding="utf-8")
    assert "双驱动模型不可用" in text
    assert "- upper：available" in text
    assert "- lower：unavailable" in text


class _IntegrationConnection:
    generation = 1
    is_connected = True
    def send_command(self, text): return True
    def disconnect(self): pass
    def validate_snapshot_rate(self, hz):
        return {"snapshot_hz": float(hz), "rate_policy": "offline_fake_explicit"}


class _IntegrationStore:
    def event(self, *args, **kwargs): pass
    def sample(self, *args, **kwargs): pass
    def update_metadata(self, **kwargs): pass


def test_real_engine_current_quality_does_not_discard_valid_thrust_point():
    clock = [1.0]
    engine = AcquisitionEngine(_IntegrationConnection(), None, _IntegrationStore(),
                               snapshot_hz=20, clock=lambda: clock[0])
    point = grid(upper_values=(30,), lower_values=(40,)).points[0]
    samples = []
    for index in range(3):
        clock[0] += 0.05
        fc_ms = 1000 + index * 50
        fc = FcSnapshot(
            nonce=1, fc_time_ms=fc_ms, esc_protocol=1, upper_channel=1,
            lower_channel=2, command_us=(1352, 1436), erpm=(4000, 5000),
            erpm_age_ms=(0, 0), voltage_v=12.0, total_current_a=99.0,
            voltage_age_ms=0, current_age_ms=1000,
            esc_current_a=(3.0, None), esc_current_age_ms=(0, None))
        engine._latest_fc = (fc, clock[0], 1, fc_ms)
        engine._latest_scale = (500.0, clock[0])
        samples.append(engine._joined_sample("real-quality", "dual", point))
    assert "lower_esc_current_missing" in samples[0].quality
    assert "board_current_stale" in samples[0].quality
    analysis = analyze_samples(samples)
    assert analysis["static"]["data_quality"]["accepted_steady_points"] == 1
    accepted = analysis["static"]["operating_points"][0]
    assert accepted["response_points"]["upper_esc_current_a"]["upper_esc_current_a"] == 3.0
    assert accepted["response_points"]["lower_esc_current_a"] is None
    def update_metadata(self, **kwargs): pass


def test_real_dynamic_plan_and_acquisition_samples_feed_dynamic_model():
    clock = [0.0]
    engine = AcquisitionEngine(_IntegrationConnection(), None, _IntegrationStore(),
                               snapshot_hz=20, clock=lambda: clock[0])
    plan = dynamic_steps(baseline=(20, 20), axis="upper", delta=30,
                         repeats=1, hold_s=1.0)
    rise_points = [point for point in plan.points if point.segment_id.endswith("rise")]
    assert len(rise_points) == 2 and rise_points[0].segment_id == rise_points[1].segment_id
    samples = []
    fc_ms = 0
    for point_index, point in enumerate(rise_points):
        count = 6 if point_index == 0 else 24
        for index in range(count):
            clock[0] += 0.05
            fc_ms += 50
            elapsed = index * 0.05
            upper_erpm = 2000 if point_index == 0 else 2000 + 4000 * (1 - math.exp(-elapsed / 0.2))
            upper_erpm = round(upper_erpm)
            upper_us = round(1100 + point.upper_percent * 8.4)
            lower_us = round(1100 + point.lower_percent * 8.4)
            fc = FcSnapshot(1, fc_ms, 1, 1, 2, (upper_us, lower_us),
                            (upper_erpm, 21000), (0, 0), 11.0, 3.0, 0, 0)
            engine._latest_fc = (fc, clock[0], 1, fc_ms)
            engine._latest_scale = (500.0, clock[0])
            samples.append(engine._joined_sample("integration", "dual", point))
    dynamics = analyze_samples(samples)["dynamics"]
    assert any(item["channel"] == "upper" and item["direction"] == "rise"
               for item in dynamics.get("fits", []))


def test_holdout_validation_exports_prediction_rows_p95_and_voltage_errors():
    analysis = analyze_samples(voltage_dependent_dual())
    diagnostics = analysis["validation"]["diagnostics"]
    assert diagnostics["status"] == "available"
    assert diagnostics["points"] > 0
    assert diagnostics["metrics"]["p95_abs_error_n"] >= 0
    assert diagnostics["error_by_loaded_voltage"]
    row = diagnostics["predictions"][0]
    assert {"run_id", "segment_id", "upper_erpm", "lower_erpm", "voltage_v",
            "actual_thrust_n", "predicted_thrust_n", "residual_n",
            "holdout_group", "holdout_value"} <= set(row)
    assert all(item["group"] in {"run", "voltage_layer"}
               for item in analysis["validation"]["holdouts"])


def test_no_holdout_is_unvalidated_and_never_uses_training_error_as_validation():
    analysis = analyze_samples(identifiable_steady())
    assert analysis["static"]["training_diagnostics"]["status"] == "available"
    assert analysis["validation"]["diagnostics"]["status"] == "unvalidated"
    assert analysis["validation"]["diagnostics"]["metrics"] is None
    assert analysis["static"]["sufficiency"]["status"] == "unvalidated"
    assert "no_independent_holdout_validation" in analysis["static"]["sufficiency"]["gaps"]


def test_sufficiency_uses_independent_points_voltage_coverage_and_holdout_error():
    analysis = analyze_samples(voltage_dependent_dual((10.0, 11.0, 12.0)))
    result = analysis["static"]["sufficiency"]
    assert result["aggregated_steady_points"] == 54
    assert result["nonzero_steady_points"] == 54
    assert result["minimum_unique_nonzero_command_combinations_per_dual_voltage_group"] == 9
    assert all(item["unique_nonzero_command_combinations"] == 9
               for item in result["command_grid"]["groups"] if item["mode"] == "dual")
    assert len(result["actual_voltage_coverage"]) == 3
    assert result["holdout_available_groups"] > 0
    assert result["reference"]["advisory_only"] is True
    assert result["reference"]["does_not_authorize_flight"] is True


def test_repeated_runs_do_not_inflate_unique_recorded_command_grid():
    result = analyze_samples(voltage_dependent_dual((10.0, 11.0, 12.0)))["static"]["sufficiency"]
    assert result["aggregated_steady_points"] == 54
    groups = [item for item in result["command_grid"]["groups"] if item["mode"] == "dual"]
    assert [item["steady_points"] for item in groups] == [18, 18, 18]
    assert [item["unique_command_combinations"] for item in groups] == [9, 9, 9]
    assert result["status"] != "reference_ready"


def test_missing_commands_are_not_guessed_as_unique_grid_combinations():
    result = analyze_samples(identifiable_steady())["static"]["sufficiency"]
    assert result["command_grid"]["missing_command_points"] > 0
    assert result["minimum_unique_nonzero_command_combinations_per_dual_voltage_group"] == 0
    assert "recorded_command_fields_missing_for_some_steady_points" in result["gaps"]


def test_two_voltage_layers_have_run_holdout_but_no_layer_holdout():
    diagnostics = analyze_samples(voltage_dependent_dual((10.0, 12.0)))["validation"]["diagnostics"]
    assert diagnostics["groups"]["run"]["available"] == 4
    assert diagnostics["groups"]["voltage_layer"]["attempted"] == 0


def test_three_voltage_layers_validate_middle_and_reject_boundary_layers():
    diagnostics = analyze_samples(voltage_dependent_dual((10.0, 11.0, 12.0)))["validation"]["diagnostics"]
    assert diagnostics["groups"]["run"]["available"] == 6
    assert diagnostics["groups"]["voltage_layer"]["attempted"] == 3
    assert diagnostics["groups"]["voltage_layer"]["available"] == 1
    assert diagnostics["groups"]["voltage_layer"]["rejected"] == 2
    assert diagnostics["prediction_coverage_fraction"] < 1.0
    boundary = [item for item in diagnostics["unavailable_groups"]
                if item["group"] == "voltage_layer"]
    assert len(boundary) == 2
    assert all("边界" in item["explanation_zh"] for item in boundary)


def test_dashboard_is_local_escaped_and_separates_training_from_validation(tmp_path):
    output = tmp_path / "dashboard-analysis"
    paths = write_analysis(voltage_dependent_dual(), {
        "schema_version": 2, "speed_domain": "electrical_erpm",
        "firmware_id": "<script>alert('x')</script>",
    }, output)
    assert {"dashboard", "model", "model_json", "report", "fit_plot", "coverage_plot"} <= set(paths)
    assert paths["model"] == paths["model_json"]
    text = paths["dashboard"].read_text(encoding="utf-8")
    assert "<script>alert" not in text
    assert "&lt;script&gt;alert" in text
    assert "整轮留出可用" in text and "中间电压层留出可用" in text
    assert "训练拟合（仅诊断）" in text
    assert "5% 满量程 RMSE" in text and "不是飞行放行" in text
    assert "fit_diagnostics.png" in text and "erpm_voltage_coverage.png" in text
    assert "最低和最高电压边界层" in text
    assert "实际记录的指令组合数，不代表唯一 eRPM 工况" in text
    parsed = json.loads(paths["model"].read_text(encoding="utf-8"))
    assert isinstance(parsed["static"]["sufficiency"]["summary"], str)
    assert parsed["static"]["sufficiency"]["summary"]


def test_report_plot_labels_are_chinese_with_local_font_fallbacks():
    source = (ROOT / "tools/thrust_bench/model_report.py").read_text(encoding="utf-8")
    assert "Microsoft YaHei" in source and "SimHei" in source and "Noto Sans CJK SC" in source
    assert "实测推力 [N]" in source and "预测推力 [N]" in source
    assert "残差（预测-实测）[N]" in source
    assert "实际带载电压 [V]" in source


def test_dashboard_explains_battery_drift_layer_failure(tmp_path):
    samples = [replace(item, voltage_v=(9.6 if index % 2 else 10.4))
               if item.voltage_layer_v == 10.0 else item
               for index, item in enumerate(voltage_dependent_dual())]
    paths = write_analysis(samples, {}, tmp_path / "drift-report")
    model = json.loads(paths["model_json"].read_text(encoding="utf-8"))
    assert model["static"]["voltage_models"]["dual"]["reason"] == "voltage_layer_span_too_wide"
    assert "voltage_layer_span_too_wide" in model["static"]["sufficiency"]["gaps"]
    dashboard = paths["dashboard"].read_text(encoding="utf-8")
    assert "任意连续放电曲线" in dashboard


def test_current_surface_rejects_nearly_common_sweep_even_if_noise_adds_rank():
    from thrust_bench.model import _fit_response_surface
    # Synthetic regression fixture, not calibration evidence.
    points=[{"run_id":"r","voltage_layer_v":12.0,"upper_erpm":float(u),
             "lower_erpm":float(u+jitter),"upper_esc_current_a":1+u/3000}
            for u,jitter in zip((1000,2000,3000,4000,5000,6000),(0,10,-10,5,-5,0))]
    result=_fit_response_surface(points,"upper_esc_current_a","dshot_current","A")
    assert result["status"]=="unavailable"
    assert result["reason"]=="inputs_not_independently_varied"


@pytest.mark.parametrize("mode", ["upper", "lower"])
def test_passive_erpm_missing_quality_does_not_discard_single_drive(mode):
    from thrust_bench.plans import single_prop
    clock=[1.0]
    engine=AcquisitionEngine(_IntegrationConnection(),None,_IntegrationStore(),clock=lambda:clock[0])
    point=single_prop(mode,minimum=30,maximum=30,step=1).points[0]
    samples=[]
    active=0 if mode=="upper" else 1
    for index in range(3):
        clock[0]+=0.05;fc_ms=1000+index*50
        erpm=[None,None];currents=[None,None];ages=[None,None];commands=[0,0]
        erpm[active]=4000;currents[active]=3;ages[active]=0;commands[active]=1352
        fc=FcSnapshot(1,fc_ms,2,1,2,tuple(commands),tuple(erpm),tuple(ages),12.0,99.0,0,0,
                      esc_current_a=tuple(currents),esc_current_age_ms=tuple(ages))
        engine._latest_fc=(fc,clock[0],1,fc_ms);engine._latest_scale=(500.0,clock[0])
        samples.append(engine._joined_sample("single-real",mode,point))
    inactive="lower" if mode=="upper" else "upper"
    assert f"{inactive}_erpm_missing" in samples[0].quality
    result=analyze_samples(samples)
    assert result["static"]["data_quality"]["accepted_steady_points"]==1
    point=result["static"]["single_drive_installed_baselines"]["points"][0]
    assert point["response_points"][f"{mode}_esc_current_a"][f"{mode}_esc_current_a"]==3

    dual_result=analyze_samples([replace(item,mode="dual") for item in samples])
    assert dual_result["static"]["data_quality"]["accepted_steady_points"]==0
