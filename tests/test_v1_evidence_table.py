"""V1 采集页的证据表格。

背景：2026-08-29 的会话里同时躺着三条 50 Hz 的降级采集和两条重复的 accel_pos_x，
表格是平铺的，既看不出哪些步骤还没采、也删不掉，点"分析"只会抛一句不知道说的是谁
的 "采样率仅 50 Hz"。这组测试锁住改造后的四件事：

  1. 表格按 11 个计划步骤成组，未采集的步骤也占一行；
  2. 降级/样本不足/重复采集在点分析之前就被标出来；
  3. 选中一行即切到该步骤，删除后证据可从 discarded/ 找回；
  4. 证据一变，上一次的分析结论立刻作废，不能拿去 apply。
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

import pytest

from tools import drone_tcp_panel as panel
from tools import v1_metrology_session as session_module
from tools.imu_metrology import MetrologyStage, MetrologyStatus

from test_v1_metrology_session import v4_header, v4_sample


def build_session(root: Path):
    return session_module.new_session(
        now=datetime(2026, 8, 29, tzinfo=timezone.utc), root=root)


def add(session, manifest, stage, *, count, step_us=1000, platform=None):
    samples = [v4_sample(1000 + index * step_us) for index in range(count)]
    return session_module.persist_capture(
        session, manifest, stage=stage, samples=samples,
        header=v4_header(total_samples=count, block_samples=count),
        temperature_platform=platform, supersede=platform is not None or step_us == 1000,
    )


@pytest.fixture(scope="module")
def _panel():
    """整个模块共用一个 Tk root。

    每条用例各建一个 DronePanel 会让 Tcl 在第十几次初始化时抛
    `invalid command name "tcl_findLibrary"`，测试随机变红。
    """
    import tkinter as tk

    try:
        instance = panel.DronePanel()
    except tk.TclError as exc:  # pragma: no cover - 无显示环境
        pytest.skip(f"Tk display unavailable: {exc}")
    try:
        yield instance
    finally:
        instance.destroy()


@pytest.fixture
def app(_panel):
    _panel.v1_session = None
    _panel.v1_manifest_path = None
    _panel.v1_analysis_summary = None
    _panel.v1_encoded_candidate = None
    _panel.v1_candidate_applied = False
    _panel.v1_probe_cache.clear()
    _panel.v1_row_plan.clear()
    _panel.v1_row_record.clear()
    _panel.validation_props_removed_var.set(False)
    _panel.validation_power_safe_var.set(False)
    for item in _panel.v1_capture_tree.get_children():
        _panel.v1_capture_tree.delete(item)
    return _panel


def rows(app) -> list[tuple[str, tuple, tuple]]:
    tree = app.v1_capture_tree
    return [
        (tree.item(iid, "text"), tree.item(iid, "values"), tree.item(iid, "tags"))
        for iid in tree.get_children()
    ]


def find(app, label: str):
    for iid in app.v1_capture_tree.get_children():
        if app.v1_capture_tree.item(iid, "text") == label:
            return iid
    raise AssertionError(f"表格里没有步骤 {label}")


def test_every_planned_step_gets_a_row_even_before_it_is_captured(app, tmp_path) -> None:
    app.v1_session, app.v1_manifest_path = build_session(tmp_path)
    app._v1_render_session()

    labels = [label for label, _values, _tags in rows(app)]
    assert labels == [plan.label for plan in session_module.CAPTURE_PLANS]
    assert all("未采集" in values[3] for _label, values, _tags in rows(app))
    assert "可用证据 0/7 步" in app.v1_counts_var.get()


def test_a_degraded_capture_is_flagged_before_the_user_clicks_analyse(app, tmp_path) -> None:
    session, manifest = build_session(tmp_path)
    # 20 ms 步进 = DRDY 失效退到轮询兜底，正是 2026-08-28 那三条数据。
    session, _record = add(session, manifest, MetrologyStage.ACCEL_POS_Z,
                           count=150, step_us=20_000)
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()

    _label, values, tags = next(row for row in rows(app) if row[0] == "+Z 水平")
    assert values[2] == "50 Hz"
    assert "删除重采" in values[3]
    assert "fail" in tags
    assert "1 条必须删除重采" in app.v1_counts_var.get()


def test_a_short_capture_is_warned_but_not_treated_as_corrupt(app, tmp_path) -> None:
    session, manifest = build_session(tmp_path)
    session, _record = add(session, manifest, MetrologyStage.ACCEL_POS_X, count=200)
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()

    _label, values, tags = next(row for row in rows(app) if row[0] == "+X 机头朝上")
    assert values[1] == "200 / 1500"
    assert "不会计入分析" in values[3]
    assert "warn" in tags and "fail" not in tags


def test_duplicate_captures_of_one_step_are_shown_and_flagged(app, tmp_path) -> None:
    session, manifest = build_session(tmp_path)
    session, _first = add(session, manifest, MetrologyStage.ACCEL_POS_X, count=1600)
    # 老会话（改造前写入的）可能有同一步骤的两份数据，不能悄悄藏掉一份。
    session = session_module.V1Session(
        session.session_id, session.created_at, session.updated_at,
        session.captures + (session.captures[0],))
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()

    duplicates = [row for row in rows(app) if row[0] == "+X 机头朝上"]
    assert len(duplicates) == 2
    assert "重复采集" in duplicates[1][1][3]
    assert "fail" in duplicates[1][2]


def test_selecting_a_row_switches_the_capture_step_so_recapture_is_one_click(app, tmp_path) -> None:
    session, manifest = build_session(tmp_path)
    session, _record = add(session, manifest, MetrologyStage.GYRO_STATIC, count=4200)
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()

    iid = find(app, "陀螺仪静止")
    app.v1_capture_tree.focus(iid)
    app.v1_capture_tree.selection_set(iid)
    app._v1_on_row_selected()

    assert app.v1_stage_var.get() == "陀螺仪静止"
    assert str(app.v1_discard_button["state"]) == "normal"


def test_an_uncaptured_row_cannot_be_deleted(app, tmp_path) -> None:
    app.v1_session, app.v1_manifest_path = build_session(tmp_path)
    app._v1_render_session()

    iid = find(app, "-Z 倒置")
    app.v1_capture_tree.focus(iid)
    app.v1_capture_tree.selection_set(iid)
    app._v1_on_row_selected()

    assert str(app.v1_discard_button["state"]) == "disabled"


def test_deleting_a_capture_invalidates_the_previous_analysis(app, tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(panel.messagebox, "askyesno", lambda *_a, **_k: True)
    session, manifest = build_session(tmp_path)
    session, record = add(session, manifest, MetrologyStage.GYRO_STATIC, count=4200)
    app.v1_session, app.v1_manifest_path = session, manifest
    app.validation_props_removed_var.set(True)
    app.validation_power_safe_var.set(True)
    app.v1_analysis_summary = session_module.AnalysisSummary(
        manifest.parent / "candidate.json", manifest.parent / "references.json",
        MetrologyStatus.PASS, MetrologyStatus.PASS, MetrologyStatus.PASS,
        MetrologyStatus.PASS, MetrologyStatus.PASS, "占位结论", 4200)
    app._v1_render_session()
    assert next(row for row in rows(app) if row[0] == "陀螺仪静止")[1][4] == "PASS"

    iid = find(app, "陀螺仪静止")
    app.v1_capture_tree.focus(iid)
    app.v1_capture_tree.selection_set(iid)
    app._v1_on_row_selected()
    app._v1_discard_selected()

    assert app.v1_session.captures == ()
    assert app.v1_analysis_summary is None, "证据被删了还留着旧结论，会被 apply 拿去用"
    assert str(app.v1_apply_button["state"]) == "disabled"
    assert (manifest.parent / session_module.DISCARDED_DIRNAME / record.csv_path).is_file()
    assert "重新执行" in app.v1_analysis_var.get()


def test_the_flow_no_longer_asks_for_hand_rotations_or_temperature(app, tmp_path) -> None:
    app.v1_session, app.v1_manifest_path = build_session(tmp_path)
    app._v1_render_session()

    labels = [label for label, _values, _tags in rows(app)]
    assert len(labels) == 7
    assert not [label for label in labels if "360" in label or "温度" in label]
    assert list(app.v1_stage_combo["values"]) == labels


def test_an_old_session_still_shows_its_retired_captures(app, tmp_path) -> None:
    """老会话的转动/温度证据不能凭空消失 —— 看得见才删得掉。"""
    session, manifest = build_session(tmp_path)
    session, kept = add(session, manifest, MetrologyStage.GYRO_STATIC, count=4200)
    retired = session_module.CaptureRecord(
        MetrologyStage.GYRO_POS_360_X.value, None, "manual",
        kept.csv_path, kept.meta_path, 6000, kept.captured_at)
    session = session_module.V1Session(
        session.session_id, session.created_at, session.updated_at,
        session.captures + (retired,))
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()

    label, _values, _tags = next(row for row in rows(app) if "360" in row[0])
    assert label == "已移除：+X 手动精确 360°"
    iid = find(app, label)
    app.v1_capture_tree.focus(iid)
    app.v1_capture_tree.selection_set(iid)
    app._v1_on_row_selected()
    assert str(app.v1_discard_button["state"]) == "normal"


def test_the_capture_gives_the_operator_time_to_get_ready(app) -> None:
    """6 s 的录制窗口按下就开录，手还没扶稳窗口已经用掉一截。"""
    source = Path(panel.__file__).read_text(encoding="utf-8")
    worker = source[source.index("def _v1_capture_worker"):]
    worker = worker[:worker.index("\n    def ")]
    prep = worker.index("V1_CAPTURE_PREP_SECONDS")
    start = worker.index('link.send(f"IMUCAP START')

    assert panel.V1_CAPTURE_PREP_SECONDS >= 3
    assert prep < start, "倒计时必须在 IMUCAP START 之前，否则等于没给准备时间"
    assert "剩余" in worker, "录制过程中必须有秒数在跳，否则体感就是'愣一下就结束了'"


def test_captures_from_a_different_firmware_image_are_flagged(app, tmp_path) -> None:
    """2026-08-29 实况：中途重烧了固件，前两面和后面几面不同源。

    分析阶段 _uniform_context 会整份拒收，但那时用户已经摆完六面了。
    """
    session, manifest = build_session(tmp_path)
    session, stale = add(session, manifest, MetrologyStage.ACCEL_POS_X, count=1600)
    meta_path = manifest.parent / stale.meta_path
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    meta["header"]["firmware_image_crc32"] = 0xDEADBEEF
    meta_path.write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    for stage in (MetrologyStage.ACCEL_NEG_X, MetrologyStage.ACCEL_POS_Y):
        session, _record = add(session, manifest, stage, count=1600)
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()

    _label, values, tags = next(row for row in rows(app) if row[0] == "+X 机头朝上")
    assert "固件版本" in values[3] and "必须重采" in values[3]
    assert "fail" in tags
    # 多数派不该被误伤。
    assert "pass" in next(row for row in rows(app) if row[0] == "-X 机头朝下")[2]


def test_the_group_verdict_is_shown_on_every_step_of_that_group(app, tmp_path) -> None:
    session, manifest = build_session(tmp_path)
    for stage, count in ((MetrologyStage.ACCEL_POS_X, 1600), (MetrologyStage.GYRO_STATIC, 4200)):
        session, _record = add(session, manifest, stage, count=count)
    app.v1_session, app.v1_manifest_path = session, manifest
    app.v1_analysis_summary = session_module.AnalysisSummary(
        manifest.parent / "candidate.json", manifest.parent / "references.json",
        MetrologyStatus.FAIL, MetrologyStatus.FAIL, MetrologyStatus.PASS,
        MetrologyStatus.NOT_RUN, MetrologyStatus.NOT_RUN, "占位结论", 8400)
    app._v1_render_session()

    # 六面是一起判的，单面没有独立结论；陀螺静止是另一组。
    assert next(row for row in rows(app) if row[0] == "+X 机头朝上")[1][4] == "FAIL"
    assert next(row for row in rows(app) if row[0] == "陀螺仪静止")[1][4] == "PASS"
    assert "+X 手动精确 360°" not in [row[0] for row in rows(app)]


def analysis_summary(manifest, **overrides):
    fields = dict(
        candidate_path=manifest.parent / "candidate.json",
        references_path=manifest.parent / "references.json",
        overall_status=MetrologyStatus.FAIL,
        accelerometer_status=MetrologyStatus.FAIL,
        gyro_static_status=MetrologyStatus.PASS,
        gyro_rotation_status=MetrologyStatus.NOT_RUN,
        temperature_status=MetrologyStatus.NOT_RUN,
        status_line="占位结论", sample_count=9600,
    )
    fields.update(overrides)
    return session_module.AnalysisSummary(**fields)


def test_the_verdict_column_refreshes_once_the_analysis_lands(app, tmp_path) -> None:
    """2026-08-29 实况：分析跑完了，表格"分析结论"整列还停在 "-"。"""
    session, manifest = build_session(tmp_path)
    for stage in (MetrologyStage.ACCEL_POS_Y, MetrologyStage.GYRO_STATIC):
        session, _record = add(session, manifest, stage, count=1600 if "accel" in stage.value else 4200)
    app.v1_session, app.v1_manifest_path = session, manifest
    app._v1_render_session()
    assert {row[1][4] for row in rows(app)} == {"-"}, "还没分析就有结论了"

    app.v1_event_queue.put(("analysis", analysis_summary(
        manifest, face_residuals={"accel_pos_y": (0.0921, 1.9)})))
    app._v1_drain_events()

    _label, values, tags = next(row for row in rows(app) if row[0] == "+Y 左侧朝上")
    assert values[4].startswith("FAIL"), "分析跑完了，结论列还是空的"
    assert "残差0.092g" in values[4] and "倾角1.9°" in values[4]
    # 采集本身没毛病，是这一面摆歪了 —— 两列各说各的，不能混为一谈。
    assert values[3] == "可用"
    assert "fail" in tags


def test_a_merely_imperfect_face_is_amber_not_red(app, tmp_path) -> None:
    """徒手摆到 0.025~0.075 g 是常态，标红只会让人以为白采了。"""
    session, manifest = build_session(tmp_path)
    session, _record = add(session, manifest, MetrologyStage.ACCEL_POS_Y, count=1600)
    app.v1_session, app.v1_manifest_path = session, manifest
    app.v1_event_queue.put(("analysis", analysis_summary(
        manifest, accelerometer_status=MetrologyStatus.WARN,
        overall_status=MetrologyStatus.WARN,
        face_residuals={"accel_pos_y": (0.0333, 2.4)})))
    app._v1_drain_events()

    _label, values, tags = next(row for row in rows(app) if row[0] == "+Y 左侧朝上")
    assert values[4].startswith("WARN") and "残差0.033g" in values[4]
    assert "warn" in tags and "fail" not in tags


def test_a_failed_analysis_says_why_and_which_face(app, tmp_path) -> None:
    summary = analysis_summary(
        tmp_path,
        findings=("校正后 RMS=0.053389 g，超过 0.025 g",),
        face_residuals={
            "accel_pos_y": (0.0731, 1.9), "accel_neg_y": (0.0722, 8.7),
            "accel_pos_z": (0.0044, 0.4),
        },
    )

    text = app._v1_describe_analysis(summary)

    assert "六面 accel=FAIL" in text and "陀螺静止=PASS" in text
    assert "超过 0.025 g" in text, "只报 FAIL 不报原因，用户只能六面全重采"
    assert "+Y 左侧朝上" in text and "-Y 右侧朝上" in text
    assert "+Z 水平" not in text, "残差最小的面不该被点名重采"
    # 0.073 g 还没到 FAIL 档：措辞是建议，不是命令。
    assert "想更准就重采" in text and "基准面" in text
    # 已移除的两组是 NOT_RUN，不该出现在结论里。
    assert "手动 +360°" not in text and "温度平台" not in text


def test_a_genuinely_bad_face_is_worded_as_mandatory(app, tmp_path) -> None:
    summary = analysis_summary(
        tmp_path, findings=("校正后 RMS=0.0921 g，超过 0.075 g；六面摆放差异过大",),
        face_residuals={"accel_neg_y": (0.0921, 22.0), "accel_pos_y": (0.0910, 1.5)})

    text = app._v1_describe_analysis(summary)

    assert "必须重采" in text and "摆放差异过大" in text


def test_a_passing_analysis_does_not_nag_about_recapture(app, tmp_path) -> None:
    summary = analysis_summary(
        tmp_path, overall_status=MetrologyStatus.PASS,
        accelerometer_status=MetrologyStatus.PASS,
        face_residuals={"accel_pos_y": (0.004, 1.2), "accel_neg_y": (0.004, 2.0)},
    )

    text = app._v1_describe_analysis(summary)

    assert "最该重采" not in text
    assert "六面 accel=PASS" in text
