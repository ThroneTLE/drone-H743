"""Unique, append-only thrust-bench session storage."""
from __future__ import annotations

import csv
import json
from dataclasses import MISSING, fields
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock
from typing import Any, Iterable
from uuid import uuid4

from .records import BenchSample, SCHEMA_VERSION


class SessionStore:
    def __init__(self, metadata: dict[str, Any], root: Path | None = None,
                 now: datetime | None = None) -> None:
        if root is None:
            from tools import project_paths
            root = project_paths.THRUST_IDENT_DIR
        stamp = (now or datetime.now(timezone.utc)).astimezone()
        day = root / stamp.date().isoformat()
        self.path = day / f"{stamp:%H%M%S}-{uuid4().hex[:8]}"
        self.raw_path = self.path / "raw"
        self.raw_path.mkdir(parents=True, exist_ok=False)
        self.samples_path = self.path / "samples.csv"
        self.events_path = self.raw_path / "events.jsonl"
        self.fc_raw_path = self.raw_path / "fc.jsonl"
        self.scale_raw_path = self.raw_path / "scale.jsonl"
        self._lock = Lock()
        document = {**metadata, "schema_version": SCHEMA_VERSION, "created_at": stamp.isoformat()}
        self.metadata = document
        (self.path / "metadata.json").write_text(
            json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
        names = ["schema_version", *[item.name for item in fields(BenchSample)]]
        with self.samples_path.open("x", newline="", encoding="utf-8") as stream:
            csv.DictWriter(stream, fieldnames=names).writeheader()

    def _jsonl(self, path: Path, record: dict[str, Any]) -> None:
        with self._lock, path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")

    def event(self, kind: str, **data: Any) -> None:
        self._jsonl(self.events_path, {"host_time_s": __import__("time").monotonic(),
                                      "kind": kind, **data})

    def raw_fc(self, **data: Any) -> None:
        self._jsonl(self.fc_raw_path, data)

    def raw_scale(self, **data: Any) -> None:
        self._jsonl(self.scale_raw_path, data)

    def sample(self, sample: BenchSample) -> None:
        row = sample.to_dict()
        row["quality"] = "|".join(sample.quality)
        with self._lock, self.samples_path.open("a", newline="", encoding="utf-8") as stream:
            csv.DictWriter(stream, fieldnames=row.keys()).writerow(row)

    def update_metadata(self, **values: Any) -> None:
        with self._lock:
            self.metadata.update(values)
            temporary = self.path / "metadata.json.tmp"
            temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(self.path / "metadata.json")

    def record_run(self, run: dict[str, Any]) -> None:
        """Append one run summary, including target labels and measured V coverage."""
        with self._lock:
            runs = list(self.metadata.get("runs", []))
            runs.append(dict(run))
            self.metadata["runs"] = runs
            temporary = self.path / "metadata.json.tmp"
            temporary.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2), encoding="utf-8")
            temporary.replace(self.path / "metadata.json")


def read_samples(path: Path) -> list[BenchSample]:
    result: list[BenchSample] = []
    required_columns = {"schema_version", *[item.name for item in fields(BenchSample)]}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        columns = set(reader.fieldnames or ())
        missing = required_columns - columns
        if missing:
            raise ValueError(
                "不是 thrust_bench v2 samples.csv（需要电转速eRPM和DShot分路电流字段；"
                "旧v1机械转速或KV/油门CSV不能直接当作eRPM，请使用历史工具）：缺 " + ",".join(sorted(missing)))
        for line_number, row in enumerate(reader, start=2):
            version = row.get("schema_version", "")
            if version != str(SCHEMA_VERSION):
                raise ValueError(f"samples.csv 第 {line_number} 行 schema_version={version!r} 不受支持")
            data: dict[str, Any] = {"schema_version": SCHEMA_VERSION}
            for field in fields(BenchSample):
                raw = row[field.name]
                if field.name == "quality":
                    data[field.name] = tuple(filter(None, raw.split("|")))
                elif field.name in {"run_id", "segment_id", "mode", "direction", "phase", "speed_source", "current_source"}:
                    if not raw:
                        raise ValueError(f"samples.csv 第 {line_number} 行必填字段 {field.name} 为空")
                    data[field.name] = raw
                elif field.name in {"board_current_calibrated", "esc_current_calibrated"}:
                    if raw not in {"True", "False"}:
                        raise ValueError(f"samples.csv 第 {line_number} 行布尔值非法：{raw!r}")
                    data[field.name] = raw == "True"
                elif raw == "":
                    if field.default is MISSING and field.default_factory is MISSING:
                        raise ValueError(f"samples.csv 第 {line_number} 行必填字段 {field.name} 为空")
                    data[field.name] = None
                elif field.name in {"fc_time_ms", "upper_erpm_age_ms", "lower_erpm_age_ms", "upper_esc_current_age_ms", "lower_esc_current_age_ms", "voltage_age_ms", "board_current_age_ms"}:
                    data[field.name] = int(raw)
                else:
                    data[field.name] = float(raw)
            result.append(BenchSample.from_dict(data))
    return result
