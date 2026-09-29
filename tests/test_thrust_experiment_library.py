from __future__ import annotations

import csv
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields
from pathlib import Path

import pytest

from tools.thrust_bench.experiment_library import (
    ExperimentLibrary,
    compatibility_identity,
)
from tools.thrust_bench.records import BenchSample, SCHEMA_VERSION


def _metadata(*, spacing: str = "42 mm") -> dict:
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at": "2026-09-22T12:00:00+00:00",
        "motor_model": "AEO CRM2413-KV1300",
        "prop_installation": "upper-A/lower-B",
        "prop_spacing": spacing,
        "two_propellers_installed": True,
        "scale_calibration": {
            "kind": "piecewise_linear",
            "points": [[0, 0.0], [1000, 500.0]],
        },
        "runs": [],
    }


def _sample(run_id: str, segment_id: str, index: int, *, mode: str = "dual") -> BenchSample:
    timestamp = 100.0 + index * 0.02
    return BenchSample(
        run_id=run_id,
        segment_id=segment_id,
        host_time_s=timestamp,
        fc_time_ms=1000 + index * 20,
        scale_time_s=timestamp,
        mode=mode,
        direction="up",
        phase="steady",
        upper_command_pct=20.0,
        lower_command_pct=25.0,
        upper_erpm=12000.0,
        lower_erpm=13000.0,
        thrust_n=1.5,
        voltage_v=11.4,
        upper_erpm_age_ms=0,
        lower_erpm_age_ms=0,
        voltage_age_ms=0,
        speed_source="dshot_erpm",
    )


def _write_session(
    root: Path,
    name: str,
    samples: list[BenchSample],
    *,
    metadata: dict | None = None,
) -> Path:
    session = root / "2026-09-22" / name
    session.mkdir(parents=True, exist_ok=True)
    document = dict(metadata or _metadata())
    run_ids = list(dict.fromkeys(sample.run_id for sample in samples))
    document["runs"] = [
        {
            "run_id": run_id,
            "mode": next(sample.mode for sample in samples if sample.run_id == run_id),
            "completed": False,
            "cancelled": True,
        }
        for run_id in run_ids
    ]
    (session / "metadata.json").write_text(
        json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    names = ["schema_version", *[item.name for item in fields(BenchSample)]]
    with (session / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=names)
        writer.writeheader()
        for sample in samples:
            row = sample.to_dict()
            row["quality"] = "|".join(sample.quality)
            writer.writerow(row)
    return session


def _three(run_id: str, segment: str = "p1", *, start: int = 0) -> list[BenchSample]:
    return [_sample(run_id, segment, start + index) for index in range(3)]


def test_delete_hides_run_without_touching_sources_and_restore_requires_new_assignment(tmp_path: Path) -> None:
    source=tmp_path/"source"
    session=_write_session(source,"session-a",_three("run-a"))
    original=(session/"samples.csv").read_bytes()
    library=ExperimentLibrary(tmp_path/"library.sqlite3")
    library.import_sessions(source)
    run=library.list_runs()[0]
    library.set_split([run["id"]],"train")
    assert len(library.load_selection()[0])==3
    library.delete_runs([run["id"]])
    assert library.list_runs()==[]
    assert library.list_runs(include_deleted=True)[0]["deleted"] is True
    assert library.covered_points(_metadata(),include_unassigned=True)==[]
    with pytest.raises(ValueError,match="训练集为空"):
        library.load_selection()
    library.import_sessions(source)
    assert library.list_runs()==[]
    assert (session/"samples.csv").read_bytes()==original
    library.restore_runs([run["id"]])
    restored=library.list_runs()[0]
    assert not restored["deleted"] and restored["split"]=="excluded"
    library.set_split([run["id"]],"train")
    assert len(library.load_selection()[0])==3
    library.close()


def test_import_namespaces_duplicate_run_ids_and_loads_pinned_selection(tmp_path: Path) -> None:
    source = tmp_path / "source"
    first = _write_session(source, "session-a", _three("repeat"))
    _write_session(source, "session-b", _three("repeat", start=10))
    library = ExperimentLibrary(tmp_path / "library.sqlite3")

    summary = library.import_sessions(source)
    runs = library.list_runs()

    assert summary["runs_imported"] == 2
    assert {run["session_id"] for run in runs} == {
        "2026-09-22/session-a",
        "2026-09-22/session-b",
    }
    assert len({run["id"] for run in runs}) == 2
    assert all(run["run_id"] == "repeat" for run in runs)
    assert all(run["steady_point_count"] == 1 for run in runs)
    assert all(run["split"] == "excluded" for run in runs)
    assert all(run["assignment_explicit"] is False for run in runs)
    assert all(any("曾被停止" in warning for warning in run["warnings"]) for run in runs)
    assert all(any("未完整结束" in warning for warning in run["warnings"]) for run in runs)

    by_session = {run["session_id"]: run for run in runs}
    train_id = by_session["2026-09-22/session-a"]["id"]
    validation_id = by_session["2026-09-22/session-b"]["id"]
    library.set_split([train_id], "train")
    library.set_split([validation_id], "validation")
    assert all(run["assignment_explicit"] is True for run in library.list_runs())
    train, validation, metadata = library.load_selection()

    assert {sample.run_id for sample in train} == {train_id}
    assert {sample.run_id for sample in validation} == {validation_id}
    assert metadata["train_run_ids"] == [train_id]
    assert metadata["validation_run_ids"] == [validation_id]
    assert len(metadata["source_manifest"]) == 2
    first_manifest = next(
        item for item in metadata["source_manifest"] if item["library_run_id"] == train_id
    )
    assert first_manifest["source_file_sha256"] == hashlib.sha256(
        (first / "samples.csv").read_bytes()
    ).hexdigest()
    assert first_manifest["sample_count"] == 3
    assert metadata["snapshot_store"]["database"] == str(
        (tmp_path / "library.sqlite3").resolve()
    )
    assert metadata["snapshot_store"]["table"] == "run_snapshots"

    _write_session(source, "session-a", _three("repeat") + _three("repeat", "p2", start=3))
    refreshed = library.import_sessions(source)
    assert refreshed["runs_refreshed"] == 1
    current = next(run for run in library.list_runs() if run["id"] == train_id)
    assert current["sample_count"] == 3
    assert "有新记录，重新选择用途可更新" in current["warnings"]

    pinned_train, _, pinned_metadata = library.load_selection()
    assert len(pinned_train) == 3
    assert next(
        item for item in pinned_metadata["source_manifest"]
        if item["library_run_id"] == train_id
    )["sample_count"] == 3
    assert len(library.covered_points(_metadata())) == 2

    library.set_split([train_id], "train")
    refreshed_train, _, _ = library.load_selection()
    assert len(refreshed_train) == 6
    assert next(run for run in library.list_runs() if run["id"] == train_id)["sample_count"] == 6
    assert len(library.covered_points(_metadata())) == 3


def test_bad_empty_and_legacy_sessions_do_not_abort_the_batch(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_session(source, "good", _three("good-run"))
    empty = _write_session(source, "empty", [])
    assert empty.joinpath("samples.csv").is_file()

    broken = source / "2026-09-22" / "broken"
    broken.mkdir(parents=True)
    (broken / "metadata.json").write_text("{not-json", encoding="utf-8")
    (broken / "samples.csv").write_text("run_id,value\nbad,1\n", encoding="utf-8")

    legacy = source / "2026-09-22" / "legacy"
    legacy.mkdir(parents=True)
    legacy.joinpath("metadata.json").write_text(
        json.dumps({**_metadata(), "schema_version": 1}), encoding="utf-8"
    )
    legacy.joinpath("samples.csv").write_text(
        "schema_version,run_id,mechanical_rpm,thrust_n\n"
        "1,old,1000,1.0\n1,old,1100,1.1\n",
        encoding="utf-8",
    )

    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    summary = library.import_sessions(source)

    assert summary["sessions_scanned"] == 4
    assert len(summary["skipped"]) == 2
    assert {item["reason"].split(":", 1)[0] for item in summary["skipped"]} == {
        "empty_samples",
        "invalid_metadata",
    }
    runs = library.list_runs()
    assert {run["run_id"] for run in runs} == {"good-run", "old"}
    legacy_run = next(run for run in runs if run["run_id"] == "old")
    assert legacy_run["sample_count"] == 2
    assert any("旧版或不兼容样本格式" in warning for warning in legacy_run["warnings"])
    with pytest.raises(ValueError, match="不能进入 eRPM 模型"):
        library.set_split([legacy_run["id"]], "train")


def test_load_selection_rejects_empty_and_incompatible_splits(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_session(source, "spacing-a", _three("a"), metadata=_metadata(spacing="42 mm"))
    _write_session(source, "spacing-b", _three("b"), metadata=_metadata(spacing="55 mm"))
    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    library.import_sessions(source)
    runs = {run["run_id"]: run for run in library.list_runs()}

    with pytest.raises(ValueError, match="训练集为空"):
        library.load_selection()
    library.set_split([runs["a"]["id"]], "train")
    train, validation, metadata = library.load_selection()
    assert len(train) == 3
    assert validation == []
    assert metadata["validation_run_ids"] == []
    library.set_split([runs["b"]["id"]], "validation")
    with pytest.raises(ValueError, match="不同物理装配"):
        library.load_selection()


def test_load_selection_rejects_duplicate_sample_snapshots_across_splits(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_session(source, "copy-a", _three("same"))
    _write_session(source, "copy-b", _three("renamed"))
    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    library.import_sessions(source)
    runs = library.list_runs()
    library.set_split([runs[0]["id"]], "train")
    library.set_split([runs[1]["id"]], "validation")

    with pytest.raises(ValueError, match="内容相同的样本快照"):
        library.load_selection()


def test_identical_incomplete_identity_is_allowed_with_explicit_warning(tmp_path: Path) -> None:
    source = tmp_path / "source"
    incomplete = _metadata(spacing="")
    _write_session(source, "old-a", _three("a"), metadata=incomplete)
    _write_session(source, "old-b", _three("b", start=10), metadata=incomplete)
    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    library.import_sessions(source)
    runs = library.list_runs()
    assert all(any("装配记录不完整" in warning for warning in run["warnings"])
               for run in runs)
    library.set_split([run["id"] for run in runs], "train")

    train, validation, metadata = library.load_selection()

    assert len(train) == 6
    assert validation == []
    assert any("适用范围待确认" in warning for warning in metadata["warnings"])
    assert len(library.covered_points(incomplete)) == 2


def test_broken_growing_v2_file_keeps_last_good_index(tmp_path: Path) -> None:
    source = tmp_path / "source"
    session = _write_session(source, "growing", _three("run"))
    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    library.import_sessions(source)
    before = library.list_runs()[0]

    samples_path = session / "samples.csv"
    samples_path.write_text(
        samples_path.read_text(encoding="utf-8").replace("12000.0", "not-a-number", 1),
        encoding="utf-8",
    )
    summary = library.import_sessions(source)

    assert summary["runs_refreshed"] == 0
    assert summary["skipped"][0]["reason"].startswith("invalid_samples:")
    after = library.list_runs()[0]
    assert after["id"] == before["id"]
    assert after["sample_count"] == before["sample_count"] == 3


def test_covered_points_only_uses_compatible_non_excluded_erpm_runs(tmp_path: Path) -> None:
    source = tmp_path / "source"
    current = _metadata(spacing="42 mm")
    _write_session(source, "kept", _three("kept"), metadata=current)
    _write_session(source, "excluded", _three("excluded", start=10), metadata=current)
    _write_session(source, "unassigned", _three("unassigned", start=30), metadata=current)
    _write_session(
        source,
        "other-assembly",
        _three("other", start=20),
        metadata=_metadata(spacing="55 mm"),
    )
    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    library.import_sessions(source)
    runs = {run["run_id"]: run for run in library.list_runs()}
    library.set_split([runs["kept"]["id"]], "train")
    library.set_split([runs["other"]["id"]], "train")
    library.set_split([runs["excluded"]["id"]], "excluded")

    ui_metadata = dict(current)
    ui_metadata.pop("schema_version")
    points = library.covered_points(ui_metadata)

    assert len(points) == 1
    assert points[0]["library_run_id"] == runs["kept"]["id"]
    assert points[0]["upper_command_pct"] == pytest.approx(20.0)
    assert points[0]["lower_command_pct"] == pytest.approx(25.0)
    assert points[0]["voltage_v"] == pytest.approx(11.4)
    candidate_points = library.covered_points(ui_metadata, include_unassigned=True)
    assert {point["source_run_id"] for point in candidate_points} == {"kept", "unassigned"}


def test_compatibility_identity_is_stable_and_requires_calibration_snapshot() -> None:
    first = _metadata()
    reordered = {
        key: first[key]
        for key in reversed(list(first))
    }
    reordered["scale_calibration"] = {
        "points": [[0, 0.0], [1000, 500.0]],
        "kind": "piecewise_linear",
    }
    assert compatibility_identity(first) == compatibility_identity(reordered)
    changed = _metadata()
    changed["scale_calibration"] = {
        "kind": "piecewise_linear",
        "points": [[0, 0.0], [1000, 510.0]],
    }
    assert compatibility_identity(first) != compatibility_identity(changed)

    missing = _metadata()
    missing["scale_calibration"] = None
    assert compatibility_identity(missing)["scale_calibration"] is None


def test_independent_connections_allow_background_thread_reads(tmp_path: Path) -> None:
    source = tmp_path / "source"
    _write_session(source, "threaded", _three("run"))
    library = ExperimentLibrary(tmp_path / "library.sqlite3")
    library.import_sessions(source)

    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(lambda _: library.list_runs(), range(12)))

    assert all(result[0]["run_id"] == "run" for result in results)
    library.close()
    with pytest.raises(RuntimeError, match="已经关闭"):
        library.list_runs()
