from __future__ import annotations

from datetime import date, datetime
from pathlib import Path

from tools import organize_data
from tools import project_paths as paths


def test_date_directory_helpers_are_sortable_and_validate_dates(tmp_path: Path) -> None:
    # 规范布局问冻结记录，不问运行期常量（测试期间整棵 data/ 被改指到临时根，
    # 见 tests/conftest.py）。
    canonical = paths.canonical_path
    assert canonical("FIRMWARE_UPDATE_DIR") == canonical("DATA_ROOT") / "firmware_updates"
    assert canonical("IMU_METROLOGY_CALIBRATION_DIR") == (
        canonical("DATA_ROOT") / "calibration" / "imu_metrology"
    )
    assert canonical("FLIGHT_ACCEPTANCE_CALIBRATION_DIR") == (
        canonical("DATA_ROOT") / "calibration" / "flight_acceptance_v2"
    )
    assert paths.date_from_name("flightlog_20260725_161555.csv") == date(2026, 7, 25)
    assert paths.date_from_name("capture-2026-07-25.json") == date(2026, 7, 25)
    assert paths.date_from_name("capture_20261340.csv") is None
    assert paths.dated_directory(tmp_path, date(2026, 7, 25)) == tmp_path / "2026-07-25"
    assert paths.dated_directory(tmp_path, datetime(2026, 7, 26, 3, 4)) == tmp_path / "2026-07-26"

    (tmp_path / "2026-07-24").mkdir()
    (tmp_path / "2026-07-26").mkdir()
    (tmp_path / "undated").mkdir()
    assert paths.latest_dated_directory(tmp_path) == tmp_path / "2026-07-26"


def test_organizer_groups_dated_and_undated_items(tmp_path: Path) -> None:
    root = tmp_path / "captures"
    root.mkdir()
    (root / "capture_20260725.csv").write_text("dated", encoding="utf-8")
    (root / "filter_report.png").write_bytes(b"undated")
    (root / "README.md").write_text("static", encoding="utf-8")
    (root / "analysis").mkdir()

    rule = organize_data.CategoryRule(
        root,
        reserved_names=frozenset({"analysis"}),
        static_names=frozenset({"README.md"}),
    )
    plans = organize_data.plan_category(rule)
    organize_data.apply_plan(plans, allowed_root=tmp_path)

    assert (root / "2026-07-25" / "capture_20260725.csv").is_file()
    assert (root / "undated" / "filter_report.png").is_file()
    assert (root / "README.md").is_file()
    assert (root / "analysis").is_dir()


def test_canonical_categories_have_only_date_or_classification_children() -> None:
    assert organize_data.build_plan() == []
