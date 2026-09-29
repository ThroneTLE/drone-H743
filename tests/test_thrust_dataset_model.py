from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path

import pytest

from tools.thrust_bench.dataset_model import predict_dataset_thrust, train_dataset
from tools.thrust_bench.dataset_report import write_dataset_analysis
from tools.thrust_bench.records import BenchSample


def sample(run: str, segment: str, upper: float, lower: float, voltage: float,
           thrust: float, index: int, *, quality=()) -> BenchSample:
    time_s = index * 0.05
    return BenchSample(
        run_id=run, segment_id=segment, host_time_s=time_s,
        fc_time_ms=index * 50, scale_time_s=time_s, mode="dual",
        direction="steady", phase="steady",
        upper_command_pct=upper / 100.0, lower_command_pct=lower / 100.0,
        upper_erpm=upper, lower_erpm=lower, thrust_n=thrust,
        voltage_v=voltage, upper_erpm_age_ms=0, lower_erpm_age_ms=0,
        voltage_age_ms=0, speed_source="dshot_erpm", current_source="dshot",
        quality=tuple(quality))


def dataset(runs: tuple[str, ...], *, voltage_effect: float = 0.45,
            confounded_voltage: bool = False) -> list[BenchSample]:
    result = []
    upper_values = (2500.0, 5000.0, 7500.0)
    lower_values = (3000.0, 5500.0, 8000.0)
    voltages = (10.0, 11.0, 12.0)
    for run_index, run in enumerate(runs):
        for u_index, upper in enumerate(upper_values):
            for l_index, lower in enumerate(lower_values):
                selected_voltages = ((9.5 + 2.5 * (upper / 7500.0) ** 2,)
                                     if confounded_voltage else voltages)
                for v_index, voltage in enumerate(selected_voltages):
                    segment = f"p{u_index}{l_index}{v_index}"
                    base = 0.15 + 1.8e-8 * upper ** 2 + 1.3e-8 * lower ** 2 + 0.6e-8 * upper * lower
                    thrust = base + voltage_effect * (voltage - 11.0) + run_index * 0.002
                    for repeat in range(3):
                        result.append(sample(run, segment, upper, lower, voltage,
                                             thrust + (repeat - 1) * 0.0002,
                                             len(result)))
    return result


def metadata(train_runs, validation_runs):
    return {"schema_version": 2, "speed_domain": "electrical_erpm",
            "train_run_ids": list(train_runs), "validation_run_ids": list(validation_runs),
            "source_manifest": {"kind": "synthetic-unit-test"},
            "compatibility_identity": "fixture-v1"}


def test_train_and_validation_runs_must_be_disjoint():
    samples = dataset(("same",))
    with pytest.raises(ValueError, match="run_id overlap"):
        train_dataset(samples, samples, metadata(("same",), ("same",)))


def test_each_held_out_prediction_uses_the_50_gram_force_limit():
    train=dataset(("train-a","train-b"))
    clean=dataset(("validation-a",))
    good=train_dataset(train,clean,{})
    assert good["prediction_acceptance"]["status"]=="meets_selection_target"
    assert good["prediction_acceptance"]["max_allowed_abs_error_gf"]==50.0
    assert good["prediction_acceptance"]["observed_max_abs_error_gf"]<50.0
    outlier_segment=clean[0].segment_id
    changed=[replace(item,thrust_n=item.thrust_n+1.2)
             if item.segment_id==outlier_segment else item for item in clean]
    bad=train_dataset(train,changed,{})
    assert bad["prediction_acceptance"]["status"]=="above_tolerance"
    assert bad["prediction_acceptance"]["observed_max_abs_error_gf"]>50.0
    assert "未达到每点不超过 50" in bad["summary"]


def test_continuous_loaded_voltage_candidate_is_selected_on_whole_validation_runs():
    train = dataset(("train-a", "train-b"))
    validation = dataset(("validation-a",))
    analysis = train_dataset(train, validation,
                             metadata(("train-a", "train-b"), ("validation-a",)))
    assert analysis["selection"]["selected_candidate"] == "continuous_loaded_voltage"
    assert analysis["selection"]["validation_is_final_test"] is False
    assert analysis["selection"]["coefficients_refit_after_selection"] is False
    voltage = analysis["selection_validation"]["continuous_loaded_voltage"]
    rpm = analysis["selection_validation"]["erpm_only"]
    assert voltage["coverage_fraction"] == 1.0
    assert voltage["metrics"]["newton"]["rmse"] < rpm["metrics"]["newton"]["rmse"]
    assert voltage["metrics"]["gram_force"]["p95_abs_error"] >= 0
    assert analysis["selected_applicability"] == {
        "upper_erpm": [2500.0, 7500.0], "lower_erpm": [3000.0, 8000.0],
        "loaded_voltage_v": [10.0, 12.0]}
    assert analysis["selected_support_domain"]["dimension"] == 3


def test_validation_values_never_change_training_scaling_or_coefficients():
    train = dataset(("train-a", "train-b"))
    validation = dataset(("validation-a",))
    changed = [replace(item, thrust_n=float(item.thrust_n) + 50.0) for item in validation]
    first = train_dataset(train, validation, {})
    second = train_dataset(train, changed, {})
    for name in ("erpm_only", "continuous_loaded_voltage"):
        assert first["candidates"][name]["scale"] == second["candidates"][name]["scale"]
        assert first["candidates"][name]["coefficients"] == second["candidates"][name]["coefficients"]


def test_voltage_candidate_reports_confounding_instead_of_fabricating_signal():
    analysis = train_dataset(dataset(("train",), confounded_voltage=True), [], {})
    voltage = analysis["candidates"]["continuous_loaded_voltage"]
    assert voltage["status"] == "unavailable"
    assert voltage["reason"] in {"loaded_voltage_confounding_with_erpm",
                                  "joint_erpm_voltage_domain_rank_deficient",
                                  "rank_deficient_or_ill_conditioned"}
    assert analysis["selection"]["selected_candidate"] == "erpm_only"
    assert analysis["selection"]["basis"] == "training_only_default_simpler_candidate"


def test_erpm_only_declares_2d_plus_voltage_range_when_joint_domain_degenerates():
    linear_voltage = [replace(item, voltage_v=9.5 + float(item.upper_erpm) / 3000.0)
                      for item in dataset(("train",))]
    rpm = train_dataset(linear_voltage, [], {})["candidates"]["erpm_only"]
    assert rpm["coverage_claim"] == "two_dimensional_erpm_hull_plus_loaded_voltage_range_only"
    assert rpm["joint_domain"]["status"] == "unavailable"


def test_only_training_produces_diagnostics_without_claiming_validation():
    analysis = train_dataset(dataset(("train",)), [], {})
    assert analysis["selected_training_diagnostics"] is not None
    assert analysis["selected_validation_diagnostics"]["status"] == "unavailable"
    assert analysis["selection"]["basis"] == "training_only_default_simpler_candidate"
    assert any("只有训练诊断" in step for step in analysis["next_steps"])


def test_validation_outside_joint_support_is_rejected_and_coverage_reported():
    train = dataset(("train-a", "train-b"))
    validation = dataset(("validation-a",))
    outside = [replace(item, upper_erpm=float(item.upper_erpm) + 20000.0,
                       segment_id="outside-" + item.segment_id)
               for item in validation]
    analysis = train_dataset(train, outside, {})
    result = analysis["selection_validation"]["continuous_loaded_voltage"]
    assert result["coverage_fraction"] == 0.0
    assert result["rejected"]["outside_training_support_domain"] == result["input_points"]
    assert any("支持域外" in step for step in analysis["next_steps"])


def test_partial_validation_is_reported_as_local_and_not_used_for_selection():
    train = dataset(("train-a", "train-b"))
    validation = dataset(("validation-a",))
    partial = [replace(item, upper_erpm=float(item.upper_erpm) + 20000.0)
               if int(item.segment_id[1]) == 2 else item for item in validation]
    analysis = train_dataset(train, partial, {})
    assert analysis["selection"]["basis"] == "training_only_default_simpler_candidate"
    assert 0.0 < analysis["selected_validation_diagnostics"]["coverage_fraction"] < 1.0
    assert "仅局部可预测" in analysis["summary"]
    assert "未达到选模覆盖门" in analysis["summary"]


def test_erpm_only_rejects_validation_voltage_outside_training_range_even_inside_erpm_hull():
    train = dataset(("train-a", "train-b"), confounded_voltage=True)
    validation = dataset(("validation-a",), confounded_voltage=True)
    outside_voltage = [replace(item, voltage_v=float(item.voltage_v) + 5.0)
                       for item in validation]
    analysis = train_dataset(train, outside_voltage, {})
    result = analysis["selection_validation"]["erpm_only"]
    assert result["coverage_fraction"] == 0.0
    assert result["rejected"]["outside_training_loaded_voltage_range"] == result["input_points"]


def test_public_prediction_uses_selected_candidate_domain_and_voltage_limits():
    analysis = train_dataset(dataset(("train-a", "train-b")), [], {})
    assert math.isfinite(predict_dataset_thrust(
        analysis, upper_erpm=5000, lower_erpm=5500, voltage_v=11.0))
    with pytest.raises(ValueError, match="outside_training_loaded_voltage_range"):
        predict_dataset_thrust(analysis, upper_erpm=5000, lower_erpm=5500, voltage_v=20.0)


def test_current_quality_does_not_remove_valid_thrust_training_points():
    train = [replace(item, quality=("upper_esc_current_missing", "lower_esc_current_stale"))
             for item in dataset(("train",))]
    analysis = train_dataset(train, [], {})
    assert analysis["data"]["train_steady_points"] == 27
    assert analysis["candidates"]["erpm_only"]["status"] == "available"


def test_nondual_modes_are_counted_and_explicitly_excluded():
    upper_only = [replace(item, mode="upper", lower_command_pct=None,
                          lower_erpm=None, lower_erpm_age_ms=None)
                  for item in dataset(("train",))]
    analysis = train_dataset(upper_only, [], {})
    assert analysis["data"]["train_mode_points"] == {"upper": 27}
    assert analysis["data"]["train_unsupported_nondual_points"] == 27
    assert analysis["data"]["train_steady_points"] == 0
    assert "train_nondual_modes_are_not_supported" in " ".join(analysis["warnings"])


def test_report_answers_four_questions_escapes_metadata_and_keeps_local_plots(tmp_path):
    train = dataset(("train-a", "train-b"))
    validation = dataset(("validation-a",))
    output = tmp_path / "dataset-report"
    paths = write_dataset_analysis(train, validation, {
        **metadata(("train-a", "train-b"), ("validation-a",)),
        "operator_note": "<script>alert('x')</script>",
    }, output)
    assert {"model", "report", "dashboard", "fit_plot", "coverage_plot"} <= set(paths)
    dashboard = paths["dashboard"].read_text(encoding="utf-8")
    for text in ("用了几轮？", "误差多少克？", "适用范围？", "缺口与下一步？"):
        assert text in dashboard
    assert "<script>alert" not in dashboard
    assert "&lt;script&gt;alert" in dashboard
    assert "不是未触碰的最终测试集" in dashboard
    assert "平均相差" in dashboard and "95%的点误差不超过" in dashboard
    assert "上路 eRPM" in dashboard and "实际带载电压" in dashboard
    assert "候选比较与选择" in dashboard
    assert "dataset_fit.png" in dashboard and "dataset_coverage.png" in dashboard
    parsed = json.loads(paths["model"].read_text(encoding="utf-8"))
    assert parsed["runs"]["overlap"] == []
    assert isinstance(parsed["summary"], str) and parsed["summary"]


def test_unavailable_candidate_reason_is_chinese_with_actionable_next_step(tmp_path):
    train = dataset(("train",), confounded_voltage=True)
    paths = write_dataset_analysis(train, [], {}, tmp_path / "confounded")
    dashboard = paths["dashboard"].read_text(encoding="utf-8")
    assert "实际带载电压与 eRPM 变化混淆" in dashboard
    assert "跨不同电量重复相同指令组合" in dashboard


def test_training_only_report_still_draws_local_fit_and_labels_it_unvalidated(tmp_path):
    paths = write_dataset_analysis(dataset(("train",)), [], {}, tmp_path / "train-only")
    assert paths["fit_plot"].exists() and paths["coverage_plot"].exists()
    dashboard = paths["dashboard"].read_text(encoding="utf-8")
    assert "没有独立选择验证结果" in dashboard
    assert "仅训练诊断（未验证）" not in dashboard  # label lives in the generated PNG


def balanced_grid(runs: tuple[str, ...], voltages=(12.45, 12.15)) -> list[BenchSample]:
    """4x4 eRPM grid (commands = eRPM/100 %); the diagonal is the same-throttle curve."""
    result = []
    levels = (1000.0, 2000.0, 3000.0, 4000.0)
    for run_index, run in enumerate(runs):
        for v_index, voltage in enumerate(voltages):
            for upper in levels:
                for lower in levels:
                    thrust = 0.02 + 4.0e-8 * upper ** 2 + 3.0e-8 * lower ** 2 + run_index * 0.001
                    for repeat in range(3):
                        result.append(sample(run, f"v{v_index}-{upper:g}-{lower:g}", upper, lower,
                                             voltage, thrust + (repeat - 1) * 0.0002, len(result)))
    return result


def test_throttle_curves_keep_same_throttle_points_and_group_by_charge():
    from tools.thrust_bench.throttle_curve import throttle_curves
    analysis = train_dataset(balanced_grid(("train",)), balanced_grid(("validation",)),
                             metadata(("train",), ("validation",)))
    curves = throttle_curves(analysis)
    assert curves["band_width_v"] == pytest.approx(0.3)
    assert [band["label"] for band in curves["bands"]] == ["12.0–12.3 V", "12.3–12.6 V"]
    assert curves["balanced_points"] == 16 and curves["total_points"] == 64
    for band in curves["bands"]:
        assert [level["throttle_pct"] for level in band["levels"]] == [10.0, 20.0, 30.0, 40.0]
        assert {point["role"] for point in band["points"]} == {"train", "validation"}
        for level in band["levels"]:
            assert level["error_gf"] == pytest.approx(level["predicted_gf"] - level["measured_gf"], abs=1.0)


def test_throttle_bands_widen_instead_of_exceeding_five_ramp_steps():
    from tools.thrust_bench.throttle_curve import throttle_curves
    points = [{"upper_command_pct": 20.0, "lower_command_pct": 20.0, "upper_erpm": 100.0,
               "lower_erpm": 100.0, "voltage_v": 10.95 + 0.3 * index, "thrust_n": 1.0}
              for index in range(6)]
    curves = throttle_curves({"data": {"train_operating_points": points}})
    assert curves["band_width_v"] == pytest.approx(0.6) and len(curves["bands"]) <= 5
    assert curves["predicted_points"] == 0  # No trained model: nothing is extrapolated.


def test_report_shows_throttle_curve_with_table_view(tmp_path):
    paths = write_dataset_analysis(balanced_grid(("train",)), balanced_grid(("validation",)),
                                   metadata(("train",), ("validation",)), tmp_path / "report")
    assert paths["throttle_plot"].name == "dataset_throttle_curve.png" and paths["throttle_plot"].exists()
    dashboard = paths["dashboard"].read_text(encoding="utf-8")
    assert "推力—油门曲线（按电池电量分段）" in dashboard and "曲线数据表" in dashboard
    assert dashboard.count("dataset_throttle_curve.png") == 1
    report = paths["report"].read_text(encoding="utf-8")
    assert "![推力—油门曲线](dataset_throttle_curve.png)" in report
    assert "| 12.3–12.6 V | 40 |" in report


def throttle_points(n=120, offset=0.0, session="2026-09-23/050000-a"):
    """Synthetic truth thrust = 900*xu^2 + 700*xl^2 + 400*xu*xl + 150*(xu+xl), x = pct/100*V/12."""
    import numpy as np
    rng = np.random.default_rng(3)
    result = []
    for index in range(n):
        upper, lower = rng.uniform(2, 100, 2)
        charge = rng.uniform(10.8, 12.5)
        xu, xl = upper / 100 * charge / 12, lower / 100 * charge / 12
        truth = 900 * xu ** 2 + 700 * xl ** 2 + 400 * xu * xl + 150 * (xu + xl)
        result.append({"upper_command_pct": upper, "lower_command_pct": lower, "charge_v": charge,
                       "thrust_gf": truth + offset, "session": session, "time_s": float(index)})
    return result


def test_throttle_model_fits_predicts_inside_domain_and_inverts():
    from tools.thrust_bench import throttle_model as tm
    model = tm.fit(throttle_points())
    assert model["training"]["held_out_time_blocks"]["max_abs_gf"] < 1.0
    xu = xl = 50 / 100 * 12.0 / 12
    assert tm.predict_gf(model, 50, 50, 12.0) == pytest.approx(900 * xu ** 2 + 700 * xl ** 2 + 400 * xu * xl + 300 * xu, abs=1)
    assert tm.predict_gf(model, 50, 50, 12.9) is None and tm.predict_gf(model, 101, 50, 12.0) is None
    command = tm.balanced_throttle(model, 600.0, 11.6)
    assert tm.predict_gf(model, command, command, 11.6) == pytest.approx(600.0, abs=1)
    assert tm.balanced_throttle(model, 99999.0, 11.6) is None
    assert tm.balanced_throttle(model, 600.0, 13.5) is None


def test_throttle_data_is_post_kv_zero_checked_and_never_uses_validation_runs(tmp_path):
    from datetime import datetime, timezone
    from tools.thrust_bench import throttle_model as tm
    from tools.thrust_bench.session import SessionStore

    def session(when, plan, idle_gf, count=12):
        store = SessionStore({"schema_version": 2, "speed_domain": "electrical_erpm"}, root=tmp_path,
                             now=datetime(2026, 9, 23, *when, tzinfo=timezone.utc))
        index = 0
        for segment, pct in enumerate([2.0] + [10.0 + 7 * k for k in range(count - 1)]):
            thrust_n = (idle_gf + 8.0 * (pct - 2.0)) / tm.GRAMS_PER_NEWTON
            for _ in range(4):
                index += 1
                sample_value = sample("r1", f"s{segment}", pct * 700, pct * 690, 12.2, thrust_n, index)
                store.sample(replace(sample_value, upper_command_pct=pct, lower_command_pct=pct))
        store.record_run({"run_id": "r1", "plan_name": plan})
        return f"{store.path.parent.name}/{store.path.name}"

    before = session((4, 0, 0), "automatic_dataset_collection", 0.0)
    tm.record_esc_kv_write(tmp_path, before, 1300)
    good = session((5, 0, 0), "automatic_dataset_collection", 0.5)
    offset = session((5, 10, 0), "automatic_dataset_collection", 28.0)
    session((5, 20, 0), "model_validation", 0.0)
    points, sources, rejected = tm.load_points(tmp_path)
    assert [item["session"] for item in sources] == [good]
    assert {p["session"] for p in points} == {good} and len(points) == 12
    assert [item["session"] for item in rejected] == [offset] and "零点" in rejected[0]["reason"]
    assert "split_validation" in tm.EXCLUDED_PLANS  # bench A/B runs stay independent test data


def lut_points(n=300, keep=lambda upper, lower: True):
    """Synthetic truth in eRPM (1e4 units x, y): thrust = 16(x^2 + y^2) + 3xy; eRPM = 60 x throttle % x V."""
    import numpy as np
    rng = np.random.default_rng(5)
    result = []
    while len(result) < n:
        upper, lower = rng.uniform(2, 100, 2)
        charge = rng.uniform(10.8, 12.5)
        if not keep(upper, lower):
            continue
        u, l = 60 * upper * charge, 60 * lower * charge
        result.append({"upper_command_pct": upper, "lower_command_pct": lower, "charge_v": charge,
                       "upper_erpm": u, "lower_erpm": l, "thrust_gf": lut_truth(u, l),
                       "session": "2026-09-23/050000-a", "time_s": float(len(result))})
    return result


def lut_truth(u, l):
    return 16 * ((u / 1e4) ** 2 + (l / 1e4) ** 2) + 3 * u * l / 1e8


def test_lookup_tables_interpolate_inside_measured_range_and_refuse_outside(tmp_path):
    from tools.thrust_bench import thrust_lut as lut
    points = lut_points()
    table = lut.build(points)
    assert table["speed"]["held_out_time_blocks"]["p95_abs_gf"] < 3  # edge points can leave a block's hull
    assert lut.thrust_from_speed(table, 40000, 30000) == pytest.approx(lut_truth(40000, 30000), abs=3)
    assert lut.predict_gf(table, 50, 60, 12.0) == pytest.approx(lut_truth(36000, 43200), abs=3)
    assert lut.thrust_from_speed(table, 79000, 79000) is None
    assert lut.predict_gf(table, 50, 50, 12.9) is None and lut.predict_gf(table, 50, 50, 10.0) is None
    command = lut.balanced_throttle(table, 600.0, 11.6)
    assert lut.predict_gf(table, command, command, 11.6) == pytest.approx(600.0, abs=0.5)
    speed = lut.balanced_speed(table, 600.0)
    assert lut.thrust_from_speed(table, speed, speed) == pytest.approx(600.0, abs=0.5)
    assert lut.balanced_throttle(table, 99999.0, 11.6) is None
    path = lut.save(table, tmp_path)
    loaded, found = lut.latest(tmp_path)
    assert found == path and lut.thrust_from_speed(loaded, 40000, 30000) == lut.thrust_from_speed(table, 40000, 30000)
    assert lut.write_figure(table, points, path.parent).is_file()
    assert "查补表已建立" in lut.summary(table)


def test_unsupported_nodes_mark_where_to_measure_next():
    from tools.thrust_bench import thrust_lut as lut
    full = lut.build(lut_points())
    hole = lut.build(lut_points(keep=lambda upper, lower: not (40 <= upper <= 60 and 40 <= lower <= 60)))
    assert [35000.0, 35000.0] in hole["speed"]["unsupported_nodes"]
    assert [35000.0, 35000.0] not in full["speed"]["unsupported_nodes"]
    assert [50.0, 50.0] in hole["throttle"]["unsupported_nodes"]


def test_result_pages_show_lookup_fit_and_every_validation_target(tmp_path):
    from tools.thrust_bench import result_pages, thrust_lut as lut
    points = lut_points()
    table = lut.build(points)
    page = result_pages.lut_page(table, points, tmp_path)
    text = page.read_text(encoding="utf-8")
    assert (tmp_path / "thrust_lut_fit.png").is_file() and 'src="thrust_lut_fit.png"' in text
    assert "转速表" in text and "原油门×电压多项式（对照）" in text
    record = {"run_id": "r1", "stop_reason": "completed", "model_id": table["model_id"],
              "summary": {"measured": 2, "skipped": 1, "mae_gf": 12.0, "max_abs_gf": 20.0, "within_limit": 2,
                          "speed_checked": 1, "speed_mae_gf": 5.0, "speed_max_abs_gf": 5.0},
              "targets": [{"target_gf": 300, "command_pct": 31.4, "charge_v_before": 11.8, "measured_gf": 320.0,
                           "error_gf": 20.0, "speed_table_gf": 315.0, "speed_error_gf": 5.0},
                          {"target_gf": 900, "command_pct": 64.3, "charge_v_before": 11.7, "measured_gf": 896.0,
                           "error_gf": -4.0},
                          {"target_gf": 1500, "command_pct": None, "skipped": "当前电量下查不到"}]}
    page = result_pages.validation_page(record, tmp_path)
    text = page.read_text(encoding="utf-8")
    assert page.name == "validation-r1.html" and (tmp_path / "validation-r1.png").is_file()
    assert text.count("<tr>") == 4 and "当前电量下查不到" in text and "2/2 个在 ±50 克内" in text
