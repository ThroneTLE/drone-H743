#!/usr/bin/env python3
"""Resumable host-side V1 IMU metrology capture sessions."""

from __future__ import annotations

import csv
import json
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

try:
    from .imu_metrology import (
        CaptureMethod,
        MetrologySample,
        MetrologyStage,
        MetrologyStatus,
        build_metrology_candidate,
        candidate_to_json,
    )
    from .imu_vibration_capture import samples_to_rows, validate_v1_provenance, write_csv
    from .project_paths import IMU_METROLOGY_CALIBRATION_DIR, dated_directory, ensure_directory
except ImportError:
    from imu_metrology import CaptureMethod, MetrologySample, MetrologyStage, MetrologyStatus, build_metrology_candidate, candidate_to_json
    from imu_vibration_capture import samples_to_rows, validate_v1_provenance, write_csv
    from project_paths import IMU_METROLOGY_CALIBRATION_DIR, dated_directory, ensure_directory


SESSION_FORMAT = "drone-h743-v1-metrology-session"
SESSION_SCHEMA = 1
CAPTURE_SOURCE_PREFIX = "imucap-v4-session:"

# IMUCAP 走 DRDY 中断驱动的 1kHz 采样链。DRDY 失效时 Sensor_Task 会退到 20ms
# 轮询兜底，采集"成功"但只有 50Hz——2026-08-28 就真实发生过一次，150 个 50Hz
# 样本原样存进了标定目录，看起来和正常数据毫无区别。
#
# 这类数据不能用于标定：低通系数按 dt=1ms 烤死，50Hz 下等效截止频率掉到 4Hz；
# 且 213Hz 抗混叠滤波器远高于 25Hz 的 Nyquist，振动会折叠进信号带。
V1_MIN_SAMPLE_RATE_HZ = 800.0
V1_MAX_SAMPLE_RATE_HZ = 1200.0
# 允许极少量调度抖动造成的超长间隔；超过这个比例说明链路在丢帧。
V1_MAX_GAP_FRACTION = 0.02
V1_GAP_FACTOR = 3.0


def measure_capture_rate(rows: Sequence[dict[str, Any]]) -> tuple[float, float]:
    """返回 (中位采样率 Hz, 超长间隔占比)。样本不足时返回 (0.0, 1.0)。"""

    stamps: list[float] = []
    for row in rows:
        value = row.get("timestamp_us")
        if value in (None, ""):
            continue
        stamps.append(float(value))
    # 两个样本就有一个间隔，足以判定速率；再少则无从判断。
    if len(stamps) < 2:
        return 0.0, 1.0
    deltas = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    if not deltas:
        return 0.0, 1.0
    ordered = sorted(deltas)
    median_dt = ordered[len(ordered) // 2]
    if median_dt <= 0.0:
        return 0.0, 1.0
    gaps = sum(1 for delta in deltas if delta > median_dt * V1_GAP_FACTOR)
    return 1_000_000.0 / median_dt, gaps / len(deltas)


def validate_capture_sample_rate(rows: Sequence[dict[str, Any]]) -> float:
    """采样率护栏：不达标直接拒收，绝不让降级数据混进标定证据。"""

    rate_hz, gap_fraction = measure_capture_rate(rows)
    if rate_hz <= 0.0:
        raise ValueError("采集缺少可用时间戳，无法确认采样率，拒绝作为 V1 证据")
    if rate_hz < V1_MIN_SAMPLE_RATE_HZ:
        raise ValueError(
            f"采样率仅 {rate_hz:.0f} Hz（要求 ≥{V1_MIN_SAMPLE_RATE_HZ:.0f} Hz）。"
            "IMU DRDY 中断很可能失效、采样链已退到 20ms 轮询兜底；"
            "此状态下低通与抗混叠假设均不成立，数据不能用于标定。"
        )
    if rate_hz > V1_MAX_SAMPLE_RATE_HZ:
        raise ValueError(
            f"采样率 {rate_hz:.0f} Hz 高于预期上限 {V1_MAX_SAMPLE_RATE_HZ:.0f} Hz，"
            "时间戳可能不可信，拒绝作为 V1 证据"
        )
    if gap_fraction > V1_MAX_GAP_FRACTION:
        raise ValueError(
            f"{gap_fraction:.1%} 的采样间隔超过中位值 {V1_GAP_FACTOR:.0f} 倍，"
            "采集期间存在丢帧，拒绝作为 V1 证据"
        )
    return rate_hz


@dataclass(frozen=True)
class CapturePlan:
    stage: MetrologyStage
    label: str
    requested_samples: int
    maximum_samples: int
    capture_method: CaptureMethod
    temperature_platform_required: bool = False


CAPTURE_PLANS = (
    *(CapturePlan(stage, label, 2000, 2000, CaptureMethod.BENCH) for stage, label in (
        (MetrologyStage.ACCEL_POS_X, "+X 机头朝上"), (MetrologyStage.ACCEL_NEG_X, "-X 机头朝下"),
        (MetrologyStage.ACCEL_POS_Y, "+Y 左侧朝上"), (MetrologyStage.ACCEL_NEG_Y, "-Y 右侧朝上"),
        (MetrologyStage.ACCEL_POS_Z, "+Z 水平"), (MetrologyStage.ACCEL_NEG_Z, "-Z 倒置"),
    )),
    CapturePlan(MetrologyStage.GYRO_STATIC, "陀螺仪静止", 4500, 4500, CaptureMethod.BENCH),
    CapturePlan(MetrologyStage.GYRO_POS_360_X, "+X 手动精确 360°", 6000, 6000, CaptureMethod.MANUAL),
    CapturePlan(MetrologyStage.GYRO_POS_360_Y, "+Y 手动精确 360°", 6000, 6000, CaptureMethod.MANUAL),
    CapturePlan(MetrologyStage.GYRO_POS_360_Z, "+Z 手动精确 360°", 6000, 6000, CaptureMethod.MANUAL),
    CapturePlan(MetrologyStage.TEMPERATURE_STATIC, "温度平台静止水平", 4500, 4500, CaptureMethod.BENCH, True),
)
PLAN_BY_STAGE = {plan.stage: plan for plan in CAPTURE_PLANS}


@dataclass(frozen=True)
class CaptureRecord:
    stage: str
    temperature_platform: str | None
    capture_method: str
    csv_path: str
    meta_path: str
    sample_count: int
    captured_at: str


@dataclass(frozen=True)
class V1Session:
    session_id: str
    created_at: str
    updated_at: str
    captures: tuple[CaptureRecord, ...] = ()
    format: str = field(default=SESSION_FORMAT, init=False)
    schema: int = field(default=SESSION_SCHEMA, init=False)
    evidence_only: bool = field(default=True, init=False)
    target_apply_enabled: bool = field(default=False, init=False)
    target_commit_enabled: bool = field(default=False, init=False)


@dataclass(frozen=True)
class AnalysisSummary:
    candidate_path: Path
    references_path: Path
    overall_status: MetrologyStatus
    accelerometer_status: MetrologyStatus
    gyro_static_status: MetrologyStatus
    gyro_rotation_status: MetrologyStatus
    temperature_status: MetrologyStatus
    status_line: str
    sample_count: int


def _iso_now() -> str:
    return datetime.now().astimezone().isoformat()


def new_session(*, now: datetime | None = None, root: Path = IMU_METROLOGY_CALIBRATION_DIR) -> tuple[V1Session, Path]:
    value = now or datetime.now().astimezone()
    session_id = "v1-" + value.strftime("%Y%m%d-%H%M%S")
    directory = ensure_directory(dated_directory(root, value) / session_id)
    session = V1Session(session_id, value.isoformat(), value.isoformat())
    path = directory / "session.json"
    write_session(session, path)
    return session, path


def session_to_dict(session: V1Session) -> dict[str, Any]:
    return {
        "format": session.format, "schema": session.schema,
        "session_id": session.session_id, "created_at": session.created_at, "updated_at": session.updated_at,
        "captures": [record.__dict__ for record in session.captures],
        "evidence_only": session.evidence_only,
        "target_apply_enabled": session.target_apply_enabled,
        "target_commit_enabled": session.target_commit_enabled,
    }


def session_from_dict(value: Any) -> V1Session:
    if not isinstance(value, dict):
        raise TypeError("V1 session must be an object")
    expected = {"format", "schema", "session_id", "created_at", "updated_at", "captures", "evidence_only", "target_apply_enabled", "target_commit_enabled"}
    if set(value) != expected:
        raise ValueError("V1 session fields mismatch")
    if value["format"] != SESSION_FORMAT or type(value["schema"]) is not int or value["schema"] != SESSION_SCHEMA:
        raise ValueError("unsupported V1 session format/schema")
    if value["evidence_only"] is not True or value["target_apply_enabled"] is not False or value["target_commit_enabled"] is not False:
        raise ValueError("unsafe V1 session mutation fields")
    captures = value["captures"]
    if not isinstance(captures, list):
        raise TypeError("captures must be an array")
    records = []
    record_fields = {"stage", "temperature_platform", "capture_method", "csv_path", "meta_path", "sample_count", "captured_at"}
    for row in captures:
        if not isinstance(row, dict) or set(row) != record_fields or type(row["sample_count"]) is not int or row["sample_count"] <= 0:
            raise ValueError("invalid V1 capture record")
        MetrologyStage(row["stage"]); CaptureMethod(row["capture_method"])
        records.append(CaptureRecord(**row))
    return V1Session(str(value["session_id"]), str(value["created_at"]), str(value["updated_at"]), tuple(records))


def write_session(session: V1Session, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(session_to_dict(session), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)
    return path


def load_session(path: Path) -> V1Session:
    return session_from_dict(json.loads(path.read_text(encoding="utf-8")))


def latest_session_manifest(root: Path = IMU_METROLOGY_CALIBRATION_DIR) -> Path | None:
    candidates = list(root.glob("20??-??-??/v1-*/session.json")) if root.exists() else []
    return max(candidates, key=lambda path: (path.stat().st_mtime_ns, str(path)), default=None)


def _safe_token(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("_") or "capture"


def persist_capture(
    session: V1Session,
    manifest_path: Path,
    *,
    stage: MetrologyStage,
    samples: list[tuple],
    header: dict[str, Any],
    temperature_platform: str | None = None,
) -> tuple[V1Session, CaptureRecord]:
    validate_v1_provenance(header)
    plan = PLAN_BY_STAGE[stage]
    if len(samples) > plan.maximum_samples:
        raise ValueError(f"{stage.value} exceeds capture maximum {plan.maximum_samples}")
    platform = temperature_platform.strip() if temperature_platform else None
    if plan.temperature_platform_required and not platform:
        raise ValueError("temperature capture requires a platform label")
    if not plan.temperature_platform_required and platform is not None:
        raise ValueError("temperature_platform is only valid for temperature captures")
    rows = samples_to_rows(samples, header)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"_{_safe_token(platform)}" if platform else ""
    base = manifest_path.parent / f"{stamp}_{stage.value}{suffix}"
    csv_path = base.with_suffix(".csv")
    meta_path = base.with_name(base.name + "_meta.json")
    write_csv(rows, csv_path)
    metadata = {
        "format": "drone-h743-v1-imucap-reference", "schema": 1,
        "session_id": session.session_id, "stage": stage.value,
        "temperature_platform": platform, "capture_method": plan.capture_method.value,
        "header": header, "sample_count": len(rows), "csv_path": csv_path.name,
    }
    meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    record = CaptureRecord(stage.value, platform, plan.capture_method.value, csv_path.name, meta_path.name, len(rows), _iso_now())
    updated = V1Session(session.session_id, session.created_at, _iso_now(), session.captures + (record,))
    write_session(updated, manifest_path)
    return updated, record


def rows_to_metrology_samples(
    rows: Sequence[dict[str, Any]], header: dict[str, Any], *, stage: MetrologyStage,
    session_id: str, provenance: str, capture_method: CaptureMethod,
    temperature_platform: str | None = None,
) -> list[MetrologySample]:
    validate_v1_provenance(header)
    # 出处合法不等于数据可用：还必须确认这批样本真的是 1kHz 采到的。
    validate_capture_sample_rate(rows)
    firmware_hash = f"crc32:{int(header['firmware_image_crc32']):08x}"
    source = CAPTURE_SOURCE_PREFIX + session_id
    result = []
    for row in rows:
        if row.get("temperature_c") in (None, ""):
            raise ValueError("V1 requires v4 raw temperature")
        result.append(MetrologySample(
            timestamp_s=float(row["timestamp_us"]) / 1_000_000.0,
            accel_x_g=float(row["accel_filt_x_g"]), accel_y_g=float(row["accel_filt_y_g"]), accel_z_g=float(row["accel_filt_z_g"]),
            gyro_x_dps=float(row["gyro_filt_x_dps"]), gyro_y_dps=float(row["gyro_filt_y_dps"]), gyro_z_dps=float(row["gyro_filt_z_dps"]),
            temperature_c=float(row["temperature_c"]), stage=stage, frame_id="FLU", frame_contract_version=1,
            orientation_code=int(header["orientation_code"]), calibration_generation=int(header["calibration_generation"]),
            firmware_hash=firmware_hash, session_id=session_id, capture_source=source, provenance=provenance,
            capture_method=capture_method, temperature_platform=temperature_platform,
        ))
    return result


def load_session_samples(session: V1Session, manifest_path: Path) -> list[MetrologySample]:
    result: list[MetrologySample] = []
    for record in session.captures:
        meta = json.loads((manifest_path.parent / record.meta_path).read_text(encoding="utf-8"))
        with (manifest_path.parent / record.csv_path).open("r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        result.extend(rows_to_metrology_samples(
            rows, meta["header"], stage=MetrologyStage(record.stage), session_id=session.session_id,
            provenance=f"imucap-v4:{record.meta_path}", capture_method=CaptureMethod(record.capture_method),
            temperature_platform=record.temperature_platform,
        ))
    return result


def analyze_session(session: V1Session, manifest_path: Path) -> AnalysisSummary:
    samples = load_session_samples(session, manifest_path)
    candidate = build_metrology_candidate(samples, data_source=CAPTURE_SOURCE_PREFIX + session.session_id)
    candidate_path = manifest_path.parent / "room_temperature_candidate.json"
    candidate_path.write_text(candidate_to_json(candidate), encoding="utf-8")
    references_path = manifest_path.parent / "candidate_references.json"
    references_path.write_text(json.dumps({
        "format": "drone-h743-v1-candidate-references", "schema": 1,
        "session_manifest": manifest_path.name, "candidate": candidate_path.name,
        "raw_references": [record.__dict__ for record in session.captures],
        "evidence_only": True, "target_apply_enabled": False, "target_commit_enabled": False,
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    manual_incomplete = candidate.gyro_rotation.status is not MetrologyStatus.PASS
    temperature_incomplete = candidate.temperature.status is not MetrologyStatus.PASS
    if manual_incomplete or temperature_incomplete:
        status_line = "V1 室温基础候选：六面/静态项通过后可应用；手动 +360 仅作方向证据，精密转台和多温点尚未完成"
    else:
        status_line = "V1 完整候选已生成；可先应用到 RAM 复验，再写入参数 Flash"
    return AnalysisSummary(
        candidate_path, references_path, candidate.status, candidate.accelerometer.status,
        candidate.gyro_static.status, candidate.gyro_rotation.status, candidate.temperature.status,
        status_line, len(samples),
    )


__all__ = ["AnalysisSummary", "CAPTURE_PLANS", "CapturePlan", "CaptureRecord", "V1Session", "analyze_session", "latest_session_manifest", "load_session", "load_session_samples", "new_session", "persist_capture", "rows_to_metrology_samples", "session_from_dict", "session_to_dict", "write_session"]
