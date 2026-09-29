"""Persistent experiment index and reproducible train/validation selections.

The SQLite database only contains an index and immutable per-run snapshots.  It
never moves, edits, or deletes the source session files.  Every public operation
opens its own connection so a Tk worker thread does not inherit a connection
created by the UI thread.
"""
from __future__ import annotations

from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import fields, replace
from datetime import datetime, timezone
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sqlite3
from threading import Lock
import tempfile
from typing import Any
from urllib.parse import quote

from .model import _steady_points
from .records import BenchSample, SCHEMA_VERSION
from .session import read_samples


_SPLITS = {"train", "validation", "excluded"}
_DATE_DIRECTORY = re.compile(r"\d{4}-\d{2}-\d{2}")
_V2_COLUMNS = {"schema_version", *[item.name for item in fields(BenchSample)]}
_IDENTITY_REQUIRED = (
    "motor_model",
    "prop_installation",
    "prop_spacing",
    "two_propellers_installed",
    "scale_calibration",
)
_WARNING_MESSAGES = {
    "metadata_missing": "缺少 metadata.json，装配身份可能不完整",
    "run_metadata_missing": "缺少该轮运行摘要，已按现有样本建立索引",
    "run_incomplete": "该轮实验未完整结束，保留已采集的有效样本",
    "run_cancelled": "该轮实验曾被停止，保留已采集的有效样本",
    "run_metadata_mode_mismatch": "运行摘要与样本中的实验模式不一致",
    "mixed_modes_within_run": "同一轮包含多个实验模式",
    "no_accepted_steady_points": "该轮暂时没有通过筛选的稳态点",
    "runs_metadata_invalid": "metadata.json 中的 runs 字段格式无效",
}


def _normalise_json(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _normalise_json(value[key]) for key in sorted(value, key=str)}
    if isinstance(value, (list, tuple)):
        return [_normalise_json(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("装配身份元数据包含非有限数字")
        return value
    raise ValueError(f"装配身份元数据包含不支持的值类型：{type(value).__name__}")


def _canonical_json(value: Any) -> str:
    return json.dumps(
        _normalise_json(value),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


def _sha256(value: bytes | str) -> str:
    if isinstance(value, str):
        value = value.encode("utf-8")
    return hashlib.sha256(value).hexdigest()


def compatibility_identity(metadata: Mapping[str, Any]) -> dict[str, Any]:
    """Return the stable physical-assembly identity used by every library query.

    Paths and live tare values are deliberately absent.  The actual scale
    calibration snapshot remains part of the identity; two files with the same
    name are not assumed to contain the same calibration.
    """

    spacing = metadata.get("prop_spacing", metadata.get("spacing"))
    installation = metadata.get("prop_installation", metadata.get("prop"))
    calibration = metadata.get(
        "scale_calibration", metadata.get("scale_calibration_snapshot")
    )
    identity = {
        "identity_version": 1,
        "sample_schema_version": metadata.get("schema_version", SCHEMA_VERSION),
        "motor_model": metadata.get("motor_model"),
        "upper_motor_model": metadata.get("upper_motor_model"),
        "lower_motor_model": metadata.get("lower_motor_model"),
        "prop_installation": installation,
        "prop_spacing": spacing,
        "upper_propeller_model": metadata.get("upper_propeller_model"),
        "lower_propeller_model": metadata.get("lower_propeller_model"),
        "two_propellers_installed": metadata.get("two_propellers_installed"),
        "bench_fixture_id": metadata.get("bench_fixture_id"),
        "scale_calibration": calibration,
    }
    return _normalise_json(identity)


def _identity_key(identity: Mapping[str, Any]) -> str:
    return _sha256(_canonical_json(identity))


def _identity_missing(identity: Mapping[str, Any]) -> list[str]:
    missing: list[str] = []
    for key in _IDENTITY_REQUIRED:
        value = identity.get(key)
        if value is None or value == "":
            missing.append(key)
    return missing


def _warning_text(code: str) -> str:
    if code.startswith("compatibility_identity_incomplete:"):
        fields_text = code.split(":", 1)[1]
        return f"装配记录不完整（缺少 {fields_text}），适用范围待确认"
    if code.startswith("unsupported_sample_schema:"):
        detail = code.split(":", 1)[1]
        return f"旧版或不兼容样本格式，仅保留索引，不能进入 eRPM 模型：{detail}"
    return _WARNING_MESSAGES.get(code, code)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _finite_values(samples: Sequence[BenchSample], field: str) -> list[float]:
    result: list[float] = []
    for sample in samples:
        value = getattr(sample, field)
        if value is not None and math.isfinite(float(value)):
            result.append(float(value))
    return result


def _sample_documents(samples: Sequence[BenchSample]) -> list[dict[str, Any]]:
    documents: list[dict[str, Any]] = []
    for sample in samples:
        document = sample.to_dict()
        document["quality"] = list(sample.quality)
        documents.append(document)
    return documents


def _content_sha256(documents: Sequence[Mapping[str, Any]]) -> str:
    """Hash recorded content while ignoring the user-renamable run label."""

    return _sha256(
        _canonical_json(
            [
                {key: value for key, value in document.items() if key != "run_id"}
                for document in documents
            ]
        )
    )


class ExperimentLibrary:
    """SQLite-backed index of append-only thrust-bench session files."""

    def __init__(self, db_path: str | Path) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._state_lock = Lock()
        self._closed = False
        connection = self._connect(check_open=False)
        try:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS run_snapshots (
                    snapshot_id TEXT PRIMARY KEY,
                    run_id TEXT NOT NULL,
                    source_file_sha256 TEXT NOT NULL,
                    metadata_sha256 TEXT NOT NULL,
                    sample_sha256 TEXT NOT NULL,
                    content_sha256 TEXT NOT NULL,
                    samples_json TEXT NOT NULL,
                    sample_count INTEGER NOT NULL,
                    steady_point_count INTEGER NOT NULL,
                    voltage_min REAL,
                    voltage_max REAL,
                    mode TEXT NOT NULL,
                    compatibility_key TEXT NOT NULL,
                    compatibility_json TEXT NOT NULL,
                    analysis_metadata_json TEXT NOT NULL,
                    warnings_json TEXT NOT NULL,
                    model_compatible INTEGER NOT NULL,
                    source_path TEXT NOT NULL,
                    source_relative_path TEXT NOT NULL,
                    imported_at TEXT NOT NULL
                );
                CREATE INDEX IF NOT EXISTS idx_snapshots_run
                    ON run_snapshots(run_id);

                CREATE TABLE IF NOT EXISTS runs (
                    id TEXT PRIMARY KEY,
                    session_id TEXT NOT NULL,
                    source_run_id TEXT NOT NULL,
                    source_path TEXT NOT NULL,
                    source_relative_path TEXT NOT NULL,
                    created_at TEXT,
                    sample_count INTEGER NOT NULL,
                    steady_point_count INTEGER NOT NULL,
                    voltage_min REAL,
                    voltage_max REAL,
                    mode TEXT NOT NULL,
                    split TEXT NOT NULL DEFAULT 'excluded'
                        CHECK(split IN ('train', 'validation', 'excluded')),
                    compatibility_key TEXT NOT NULL,
                    compatibility_json TEXT NOT NULL,
                    warnings_json TEXT NOT NULL,
                    model_compatible INTEGER NOT NULL,
                    current_snapshot_id TEXT NOT NULL,
                    selected_snapshot_id TEXT,
                    is_deleted INTEGER NOT NULL DEFAULT 0,
                    updated_at TEXT NOT NULL,
                    UNIQUE(session_id, source_run_id)
                );
                CREATE INDEX IF NOT EXISTS idx_runs_split ON runs(split);
                CREATE INDEX IF NOT EXISTS idx_runs_compatibility
                    ON runs(compatibility_key, model_compatible);
                """
            )
            snapshot_columns = {
                row[1] for row in connection.execute("PRAGMA table_info(run_snapshots)")
            }
            if "content_sha256" not in snapshot_columns:
                connection.execute(
                    "ALTER TABLE run_snapshots ADD COLUMN content_sha256 TEXT"
                )
                for row in connection.execute(
                    "SELECT snapshot_id, samples_json FROM run_snapshots"
                ):
                    documents = json.loads(row[1])
                    connection.execute(
                        "UPDATE run_snapshots SET content_sha256 = ? WHERE snapshot_id = ?",
                        (_content_sha256(documents), row[0]),
                    )
            connection.execute("UPDATE runs SET split = 'excluded' WHERE split IS NULL")
            run_columns={row[1] for row in connection.execute("PRAGMA table_info(runs)")}
            if "is_deleted" not in run_columns:
                connection.execute("ALTER TABLE runs ADD COLUMN is_deleted INTEGER NOT NULL DEFAULT 0")
            connection.commit()
        finally:
            connection.close()

    def _ensure_open(self) -> None:
        with self._state_lock:
            if self._closed:
                raise RuntimeError("实验库已经关闭")

    def _connect(self, *, check_open: bool = True) -> sqlite3.Connection:
        if check_open:
            self._ensure_open()
        connection = sqlite3.connect(self.db_path, timeout=30.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA busy_timeout=30000")
        return connection

    def close(self) -> None:
        with self._state_lock:
            self._closed = True

    def __enter__(self) -> "ExperimentLibrary":
        self._ensure_open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    @staticmethod
    def _snapshot_samples(raw: bytes) -> list[BenchSample]:
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as stream:
                stream.write(raw)
                temporary_path = Path(stream.name)
            return read_samples(temporary_path)
        finally:
            if temporary_path is not None:
                temporary_path.unlink(missing_ok=True)

    @staticmethod
    def _read_metadata(session_path: Path) -> tuple[dict[str, Any], list[str]]:
        metadata_path = session_path / "metadata.json"
        if not metadata_path.is_file():
            return {}, ["metadata_missing"]
        value = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("metadata.json 顶层不是对象")
        return value, []

    @staticmethod
    def _run_id(session_id: str, source_run_id: str) -> str:
        return f"{quote(session_id, safe='/-_.')}::{quote(source_run_id, safe='-_.')}"

    def _snapshot_record(
        self,
        *,
        root: Path,
        path: Path,
        session_id: str,
        source_run_id: str,
        samples: Sequence[BenchSample],
        metadata: Mapping[str, Any],
        source_sha256: str,
        metadata_warnings: Sequence[str],
        run_metadata: Mapping[str, Any] | None,
    ) -> dict[str, Any]:
        run_id = self._run_id(session_id, source_run_id)
        identity = compatibility_identity(metadata)
        identity_json = _canonical_json(identity)
        identity_key = _identity_key(identity)
        analysis_metadata_json = _canonical_json(metadata)
        metadata_sha256 = _sha256(analysis_metadata_json)
        sample_documents = _sample_documents(samples)
        samples_json = _canonical_json(sample_documents)
        sample_sha256 = _sha256(samples_json)
        content_sha256 = _content_sha256(sample_documents)
        snapshot_id = _sha256(
            _canonical_json(
                {
                    "run_id": run_id,
                    "source_file_sha256": source_sha256,
                    "metadata_sha256": metadata_sha256,
                    "sample_sha256": sample_sha256,
                    "content_sha256": content_sha256,
                    "compatibility_key": identity_key,
                }
            )
        )
        run_metadata = dict(run_metadata or {})
        warnings = list(metadata_warnings)
        if not run_metadata:
            warnings.append("run_metadata_missing")
        if run_metadata.get("completed") is False:
            warnings.append("run_incomplete")
        if run_metadata.get("cancelled") is True:
            warnings.append("run_cancelled")
        missing_identity = _identity_missing(identity)
        if missing_identity:
            warnings.append("compatibility_identity_incomplete:" + ",".join(missing_identity))
        modes = sorted({sample.mode for sample in samples})
        recorded_mode = run_metadata.get("mode")
        if recorded_mode and modes and recorded_mode not in modes:
            warnings.append("run_metadata_mode_mismatch")
        mode = str(recorded_mode or (modes[0] if len(modes) == 1 else "mixed"))
        if len(modes) > 1:
            warnings.append("mixed_modes_within_run")
        points, _ = _steady_points(samples, metadata)
        if not points:
            warnings.append("no_accepted_steady_points")
        voltages = _finite_values(samples, "voltage_v")
        relative_path = path.relative_to(root).as_posix()
        return {
            "id": run_id,
            "session_id": session_id,
            "source_run_id": source_run_id,
            "source_path": str(path.resolve()),
            "source_relative_path": relative_path,
            "created_at": run_metadata.get("created_at") or metadata.get("created_at"),
            "sample_count": len(samples),
            "steady_point_count": len(points),
            "voltage_min": min(voltages) if voltages else None,
            "voltage_max": max(voltages) if voltages else None,
            "mode": mode,
            "compatibility_key": identity_key,
            "compatibility_json": identity_json,
            "analysis_metadata_json": analysis_metadata_json,
            "warnings_json": _canonical_json(sorted(set(warnings))),
            "model_compatible": 1,
            "snapshot_id": snapshot_id,
            "source_file_sha256": source_sha256,
            "metadata_sha256": metadata_sha256,
            "sample_sha256": sample_sha256,
            "content_sha256": content_sha256,
            "samples_json": samples_json,
        }

    def _legacy_records(
        self,
        *,
        root: Path,
        path: Path,
        session_id: str,
        raw: bytes,
        metadata: Mapping[str, Any],
        source_sha256: str,
        reason: str,
        metadata_warnings: Sequence[str],
    ) -> list[dict[str, Any]]:
        text = raw.decode("utf-8-sig")
        rows = list(csv.DictReader(io.StringIO(text)))
        if not rows:
            return []
        grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
        for row in rows:
            grouped[(row.get("run_id") or "__legacy__").strip() or "__legacy__"].append(row)
        identity = compatibility_identity(metadata)
        identity_json = _canonical_json(identity)
        identity_key = _identity_key(identity)
        analysis_metadata_json = _canonical_json(metadata)
        metadata_sha256 = _sha256(analysis_metadata_json)
        relative_path = path.relative_to(root).as_posix()
        records: list[dict[str, Any]] = []
        for source_run_id, run_rows in sorted(grouped.items()):
            run_id = self._run_id(session_id, source_run_id)
            mode_values = sorted({row.get("mode", "").strip() for row in run_rows if row.get("mode", "").strip()})
            warnings = sorted(set([*metadata_warnings, "unsupported_sample_schema:" + reason]))
            sample_sha256 = _sha256(_canonical_json(run_rows))
            content_sha256 = _content_sha256(run_rows)
            snapshot_id = _sha256(
                _canonical_json(
                    {
                        "run_id": run_id,
                        "source_file_sha256": source_sha256,
                        "metadata_sha256": metadata_sha256,
                        "sample_sha256": sample_sha256,
                        "content_sha256": content_sha256,
                        "compatibility_key": identity_key,
                    }
                )
            )
            records.append(
                {
                    "id": run_id,
                    "session_id": session_id,
                    "source_run_id": source_run_id,
                    "source_path": str(path.resolve()),
                    "source_relative_path": relative_path,
                    "created_at": metadata.get("created_at"),
                    "sample_count": len(run_rows),
                    "steady_point_count": 0,
                    "voltage_min": None,
                    "voltage_max": None,
                    "mode": mode_values[0] if len(mode_values) == 1 else "legacy",
                    "compatibility_key": identity_key,
                    "compatibility_json": identity_json,
                    "analysis_metadata_json": analysis_metadata_json,
                    "warnings_json": _canonical_json(warnings),
                    "model_compatible": 0,
                    "snapshot_id": snapshot_id,
                    "source_file_sha256": source_sha256,
                    "metadata_sha256": metadata_sha256,
                    "sample_sha256": sample_sha256,
                    "content_sha256": content_sha256,
                    "samples_json": "[]",
                }
            )
        return records

    def import_sessions(self, root: str | Path) -> dict[str, Any]:
        """Refresh every direct ``root/YYYY-MM-DD/session/samples.csv`` source."""

        self._ensure_open()
        source_root = Path(root)
        if not source_root.is_dir():
            raise ValueError(f"实验会话根目录不存在或不是目录：{source_root}")
        paths = sorted(
            path
            for path in source_root.glob("*/*/samples.csv")
            if _DATE_DIRECTORY.fullmatch(path.parent.parent.name)
        )
        summary: dict[str, Any] = {
            "sessions_scanned": len(paths),
            "runs_imported": 0,
            "runs_refreshed": 0,
            "runs_unchanged": 0,
            "skipped": [],
        }
        records: list[dict[str, Any]] = []
        for path in paths:
            relative = path.relative_to(source_root).as_posix()
            session_id = path.parent.relative_to(source_root).as_posix()
            try:
                raw = path.read_bytes()
                source_sha256 = _sha256(raw)
                metadata, metadata_warnings = self._read_metadata(path.parent)
                # Validate the complete snapshot before any database changes.
                _canonical_json(metadata)
                compatibility_identity(metadata)
            except Exception as exc:
                summary["skipped"].append(
                    {"path": relative, "reason": f"invalid_metadata:元数据无效：{exc}"}
                )
                continue
            try:
                samples = self._snapshot_samples(raw)
            except Exception as exc:
                try:
                    decoded = raw.decode("utf-8-sig")
                    reader = csv.DictReader(io.StringIO(decoded))
                    header = reader.fieldnames or []
                    raw_rows = list(reader)
                except Exception:
                    header = []
                    raw_rows = []
                declared_versions = {
                    str(row.get("schema_version", "")).strip()
                    for row in raw_rows
                    if str(row.get("schema_version", "")).strip()
                }
                looks_like_broken_v2 = (
                    _V2_COLUMNS.issubset(header)
                    and (not declared_versions or declared_versions == {str(SCHEMA_VERSION)})
                )
                if looks_like_broken_v2:
                    summary["skipped"].append(
                        {"path": relative, "reason": f"invalid_samples:样本文件无效：{exc}"}
                    )
                    continue
                try:
                    legacy = self._legacy_records(
                        root=source_root,
                        path=path,
                        session_id=session_id,
                        raw=raw,
                        metadata=metadata,
                        source_sha256=source_sha256,
                        reason=str(exc),
                        metadata_warnings=metadata_warnings,
                    )
                except Exception as legacy_exc:
                    summary["skipped"].append(
                        {"path": relative, "reason": f"invalid_samples:样本文件无效：{legacy_exc}"}
                    )
                    continue
                if not legacy:
                    summary["skipped"].append(
                        {"path": relative, "reason": "empty_samples:没有样本行"}
                    )
                    continue
                records.extend(legacy)
                continue
            if not samples:
                summary["skipped"].append(
                    {"path": relative, "reason": "empty_samples:没有样本行"}
                )
                continue
            run_metadata: dict[str, Mapping[str, Any]] = {}
            metadata_runs = metadata.get("runs", [])
            if not isinstance(metadata_runs, list):
                metadata_warnings.append("runs_metadata_invalid")
                metadata_runs = []
            for item in metadata_runs:
                if isinstance(item, Mapping) and item.get("run_id") is not None:
                    run_metadata[str(item["run_id"])] = item
            grouped_samples: dict[str, list[BenchSample]] = defaultdict(list)
            for sample in samples:
                grouped_samples[sample.run_id].append(sample)
            for source_run_id, run_samples in sorted(grouped_samples.items()):
                records.append(
                    self._snapshot_record(
                        root=source_root,
                        path=path,
                        session_id=session_id,
                        source_run_id=source_run_id,
                        samples=run_samples,
                        metadata=metadata,
                        source_sha256=source_sha256,
                        metadata_warnings=metadata_warnings,
                        run_metadata=run_metadata.get(source_run_id),
                    )
                )

        connection = self._connect()
        try:
            with connection:
                for record in records:
                    previous = connection.execute(
                        "SELECT current_snapshot_id FROM runs WHERE id = ?", (record["id"],)
                    ).fetchone()
                    connection.execute(
                        """
                        INSERT OR IGNORE INTO run_snapshots (
                            snapshot_id, run_id, source_file_sha256, metadata_sha256,
                            sample_sha256, content_sha256,
                            samples_json, sample_count, steady_point_count,
                            voltage_min, voltage_max, mode, compatibility_key,
                            compatibility_json, analysis_metadata_json, warnings_json,
                            model_compatible, source_path, source_relative_path,
                            imported_at
                        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        """,
                        (
                            record["snapshot_id"],
                            record["id"],
                            record["source_file_sha256"],
                            record["metadata_sha256"],
                            record["sample_sha256"],
                            record["content_sha256"],
                            record["samples_json"],
                            record["sample_count"],
                            record["steady_point_count"],
                            record["voltage_min"],
                            record["voltage_max"],
                            record["mode"],
                            record["compatibility_key"],
                            record["compatibility_json"],
                            record["analysis_metadata_json"],
                            record["warnings_json"],
                            record["model_compatible"],
                            record["source_path"],
                            record["source_relative_path"],
                            _now(),
                        ),
                    )
                    connection.execute(
                        """
                        INSERT INTO runs (
                            id, session_id, source_run_id, source_path,
                            source_relative_path, created_at, sample_count,
                            steady_point_count, voltage_min, voltage_max, mode,
                            compatibility_key, compatibility_json, warnings_json,
                            model_compatible, current_snapshot_id, updated_at
                        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                        ON CONFLICT(id) DO UPDATE SET
                            session_id = excluded.session_id,
                            source_run_id = excluded.source_run_id,
                            source_path = excluded.source_path,
                            source_relative_path = excluded.source_relative_path,
                            created_at = excluded.created_at,
                            sample_count = excluded.sample_count,
                            steady_point_count = excluded.steady_point_count,
                            voltage_min = excluded.voltage_min,
                            voltage_max = excluded.voltage_max,
                            mode = excluded.mode,
                            compatibility_key = excluded.compatibility_key,
                            compatibility_json = excluded.compatibility_json,
                            warnings_json = excluded.warnings_json,
                            model_compatible = excluded.model_compatible,
                            current_snapshot_id = excluded.current_snapshot_id,
                            updated_at = excluded.updated_at
                        """,
                        (
                            record["id"],
                            record["session_id"],
                            record["source_run_id"],
                            record["source_path"],
                            record["source_relative_path"],
                            record["created_at"],
                            record["sample_count"],
                            record["steady_point_count"],
                            record["voltage_min"],
                            record["voltage_max"],
                            record["mode"],
                            record["compatibility_key"],
                            record["compatibility_json"],
                            record["warnings_json"],
                            record["model_compatible"],
                            record["snapshot_id"],
                            _now(),
                        ),
                    )
                    if previous is None:
                        summary["runs_imported"] += 1
                    elif previous["current_snapshot_id"] == record["snapshot_id"]:
                        summary["runs_unchanged"] += 1
                    else:
                        summary["runs_refreshed"] += 1
        finally:
            connection.close()
        return summary

    def list_runs(self, *, include_deleted: bool = False) -> list[dict[str, Any]]:
        connection = self._connect()
        try:
            rows = connection.execute(
                """
                SELECT r.id, r.session_id, r.source_run_id, r.created_at,
                       COALESCE(s.sample_count, r.sample_count) AS display_sample_count,
                       COALESCE(s.steady_point_count, r.steady_point_count) AS display_steady_point_count,
                       CASE WHEN s.snapshot_id IS NOT NULL
                            THEN s.voltage_min ELSE r.voltage_min END AS display_voltage_min,
                       CASE WHEN s.snapshot_id IS NOT NULL
                            THEN s.voltage_max ELSE r.voltage_max END AS display_voltage_max,
                       COALESCE(s.mode, r.mode) AS display_mode,
                       r.split,
                       COALESCE(s.compatibility_key, r.compatibility_key) AS display_compatibility_key,
                       COALESCE(s.compatibility_json, r.compatibility_json) AS display_compatibility_json,
                       COALESCE(s.warnings_json, r.warnings_json) AS display_warnings_json,
                       COALESCE(s.model_compatible, r.model_compatible) AS display_model_compatible,
                       r.current_snapshot_id, r.selected_snapshot_id, r.is_deleted
                FROM runs AS r
                LEFT JOIN run_snapshots AS s
                  ON s.snapshot_id = r.selected_snapshot_id
                WHERE (? = 1 OR r.is_deleted = 0)
                ORDER BY COALESCE(r.created_at, '') DESC, r.id
                """,
                (int(include_deleted),),
            ).fetchall()
        finally:
            connection.close()
        result: list[dict[str, Any]] = []
        for row in rows:
            warnings = [
                _warning_text(code)
                for code in json.loads(row["display_warnings_json"])
            ]
            if (
                row["split"] in {"train", "validation"}
                and row["selected_snapshot_id"] is not None
                and row["selected_snapshot_id"] != row["current_snapshot_id"]
            ):
                warnings.append("有新记录，重新选择用途可更新")
            result.append(
                {
                    "id": row["id"],
                    "session_id": row["session_id"],
                    "run_id": row["source_run_id"],
                    "created_at": row["created_at"],
                    "sample_count": row["display_sample_count"],
                    "steady_point_count": row["display_steady_point_count"],
                    "voltage_min": row["display_voltage_min"],
                    "voltage_max": row["display_voltage_max"],
                    "mode": row["display_mode"],
                    "split": row["split"],
                    "assignment_explicit": row["selected_snapshot_id"] is not None,
                    "deleted": bool(row["is_deleted"]),
                    "compatibility_key": row["display_compatibility_key"],
                    "compatibility_identity": json.loads(row["display_compatibility_json"]),
                    "warnings": sorted(set(warnings)),
                    "model_compatible": bool(row["display_model_compatible"]),
                }
            )
        return result

    def set_split(self, run_ids: Sequence[str], split: str) -> None:
        if split not in _SPLITS:
            raise ValueError("用途必须是 train、validation 或 excluded")
        unique_ids = list(dict.fromkeys(str(run_id) for run_id in run_ids))
        if not unique_ids:
            raise ValueError("没有选择任何实验轮次")
        placeholders = ",".join("?" for _ in unique_ids)
        connection = self._connect()
        try:
            with connection:
                rows = connection.execute(
                    f"SELECT id, model_compatible, current_snapshot_id, is_deleted FROM runs WHERE id IN ({placeholders})",
                    unique_ids,
                ).fetchall()
                found = {row["id"]: row for row in rows}
                missing = [run_id for run_id in unique_ids if run_id not in found]
                if missing:
                    raise ValueError("找不到这些实验轮次：" + ", ".join(missing))
                if any(found[run_id]["is_deleted"] for run_id in unique_ids):
                    raise ValueError("已删除的实验须先恢复，才能分配训练或验证用途")
                if split in {"train", "validation"}:
                    incompatible = [
                        run_id for run_id in unique_ids if not found[run_id]["model_compatible"]
                    ]
                    if incompatible:
                        raise ValueError(
                            "这些实验轮次不能进入 eRPM 模型："
                            + ", ".join(incompatible)
                        )
                connection.executemany(
                    "UPDATE runs SET split = ?, selected_snapshot_id = current_snapshot_id WHERE id = ?",
                    [(split, run_id) for run_id in unique_ids],
                )
        finally:
            connection.close()

    def delete_runs(self, run_ids: Sequence[str]) -> None:
        """Hide selected library runs and exclude them from training/coverage.

        Raw sessions and immutable snapshots remain available for restoration.
        """
        self._set_deleted(run_ids, deleted=True)

    def restore_runs(self, run_ids: Sequence[str]) -> None:
        self._set_deleted(run_ids, deleted=False)

    def _set_deleted(self, run_ids: Sequence[str], *, deleted: bool) -> None:
        unique_ids=list(dict.fromkeys(str(run_id) for run_id in run_ids))
        if not unique_ids: raise ValueError("请先选择实验轮次")
        placeholders=",".join("?" for _ in unique_ids)
        connection=self._connect()
        try:
            with connection:
                found={row["id"] for row in connection.execute(
                    f"SELECT id FROM runs WHERE id IN ({placeholders})",unique_ids)}
                missing=[run_id for run_id in unique_ids if run_id not in found]
                if missing: raise ValueError("找不到这些实验轮次："+", ".join(missing))
                if deleted:
                    connection.executemany(
                        "UPDATE runs SET is_deleted=1, split='excluded', "
                        "selected_snapshot_id=current_snapshot_id WHERE id=?",
                        [(run_id,) for run_id in unique_ids])
                else:
                    connection.executemany(
                        "UPDATE runs SET is_deleted=0, selected_snapshot_id=current_snapshot_id WHERE id=?",
                        [(run_id,) for run_id in unique_ids])
        finally:connection.close()

    @staticmethod
    def _decode_samples(row: sqlite3.Row) -> list[BenchSample]:
        samples = [BenchSample.from_dict(item) for item in json.loads(row["samples_json"])]
        return [replace(sample, run_id=row["id"]) for sample in samples]

    def load_selection(
        self,
    ) -> tuple[list[BenchSample], list[BenchSample], dict[str, Any]]:
        connection = self._connect()
        try:
            rows = connection.execute(
                """
                SELECT r.id, r.session_id, r.source_run_id, r.split,
                       s.model_compatible, r.selected_snapshot_id,
                       s.source_file_sha256, s.metadata_sha256,
                       s.sample_sha256, s.content_sha256, s.samples_json,
                       s.sample_count, s.compatibility_key, s.compatibility_json,
                       s.source_relative_path, s.snapshot_id
                FROM runs AS r
                LEFT JOIN run_snapshots AS s
                  ON s.snapshot_id = r.selected_snapshot_id
                WHERE r.split IN ('train', 'validation') AND r.is_deleted = 0
                ORDER BY CASE r.split WHEN 'train' THEN 0 ELSE 1 END, r.id
                """
            ).fetchall()
        finally:
            connection.close()
        train_rows = [row for row in rows if row["split"] == "train"]
        validation_rows = [row for row in rows if row["split"] == "validation"]
        if not train_rows:
            raise ValueError("训练集为空，请至少选择一轮 train")
        train_ids = {row["id"] for row in train_rows}
        validation_ids = {row["id"] for row in validation_rows}
        overlap = sorted(train_ids & validation_ids)
        if overlap:
            raise ValueError("训练集和验证集包含同一轮实验：" + ", ".join(overlap))
        if any(row["snapshot_id"] is None for row in rows):
            raise ValueError("选中的实验轮次缺少固定样本快照")
        if any(not row["model_compatible"] for row in rows):
            raise ValueError("选中的实验轮次包含不能进入 eRPM 模型的数据")
        compatibility_keys = {row["compatibility_key"] for row in rows}
        if len(compatibility_keys) != 1:
            raise ValueError("选中的实验来自不同物理装配或不同称重标定，不能混合训练")
        train_hashes = {row["content_sha256"] for row in train_rows}
        validation_hashes = {row["content_sha256"] for row in validation_rows}
        duplicate_hashes = sorted(train_hashes & validation_hashes)
        if duplicate_hashes:
            raise ValueError("训练集和验证集包含内容相同的样本快照")
        identity = json.loads(rows[0]["compatibility_json"])
        missing_identity = _identity_missing(identity)

        train_samples = [sample for row in train_rows for sample in self._decode_samples(row)]
        validation_samples = [
            sample for row in validation_rows for sample in self._decode_samples(row)
        ]
        manifest = [
            {
                "library_run_id": row["id"],
                "session_id": row["session_id"],
                "source_run_id": row["source_run_id"],
                "split": row["split"],
                "source_relative_path": row["source_relative_path"],
                "source_file_sha256": row["source_file_sha256"],
                "metadata_sha256": row["metadata_sha256"],
                "sample_snapshot_sha256": row["sample_sha256"],
                "sample_content_sha256": row["content_sha256"],
                "snapshot_id": row["snapshot_id"],
                "sample_count": row["sample_count"],
            }
            for row in rows
        ]
        selection_warnings = []
        if missing_identity:
            selection_warnings.append(
                "装配记录不完整（缺少 " + ", ".join(missing_identity)
                + "）；这些实验仅因现有身份字段完全相同而合并，适用范围待确认"
            )
        metadata = {
            "train_run_ids": sorted(train_ids),
            "validation_run_ids": sorted(validation_ids),
            "source_manifest": manifest,
            "compatibility_identity": identity,
            "warnings": selection_warnings,
            "snapshot_store": {
                "database": str(self.db_path.resolve()),
                "table": "run_snapshots",
            },
            "selection_sha256": _sha256(_canonical_json(manifest)),
        }
        return train_samples, validation_samples, metadata

    def covered_points(
        self,
        metadata: Mapping[str, Any],
        *,
        include_unassigned: bool = False,
    ) -> list[dict[str, Any]]:
        """Return accepted steady points for the current physical assembly.

        Runs assigned to ``excluded`` and non-eRPM legacy records are absent;
        coverage normally comes from pinned train/validation snapshots only.
        ``include_unassigned`` also reads the current snapshot of a newly
        imported run that still has the default excluded state and has never
        been explicitly assigned.  An explicitly excluded run stays absent.
        """

        identity = compatibility_identity(metadata)
        key = _identity_key(identity)
        connection = self._connect()
        try:
            rows = connection.execute(
                """
                SELECT r.id, r.source_run_id, s.analysis_metadata_json,
                       s.samples_json
                FROM runs AS r
                JOIN run_snapshots AS s
                  ON s.snapshot_id = CASE
                      WHEN r.selected_snapshot_id IS NOT NULL
                          THEN r.selected_snapshot_id
                      ELSE r.current_snapshot_id
                  END
                WHERE s.compatibility_key = ?
                  AND s.model_compatible = 1
                  AND r.is_deleted = 0
                  AND (
                      r.split <> 'excluded'
                      OR (? = 1 AND r.split = 'excluded'
                          AND r.selected_snapshot_id IS NULL)
                  )
                ORDER BY r.id
                """,
                (key, int(include_unassigned)),
            ).fetchall()
        finally:
            connection.close()
        result: list[dict[str, Any]] = []
        for row in rows:
            samples = self._decode_samples(row)
            run_metadata = json.loads(row["analysis_metadata_json"])
            points, _ = _steady_points(samples, run_metadata)
            for point in points:
                result.append(
                    {
                        **point,
                        "library_run_id": row["id"],
                        "source_run_id": row["source_run_id"],
                    }
                )
        return result
