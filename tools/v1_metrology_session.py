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
        DEFAULT_THRESHOLDS,
        GYRO_ROTATION_MOTION_FLOOR_DPS,
        GYRO_ROTATION_STAGES,
        CaptureMethod,
        MetrologySample,
        MetrologyStage,
        MetrologyStatus,
        accelerometer_face_residuals,
        build_metrology_candidate,
        candidate_to_json,
        gyro_rotation_motion_window,
    )
    from .imu_metrology import _require_numpy
    from .imu_vibration_capture import samples_to_rows, validate_v1_provenance, write_csv
    from .project_paths import IMU_METROLOGY_CALIBRATION_DIR, dated_directory, ensure_directory
except ImportError:
    from imu_metrology import DEFAULT_THRESHOLDS, GYRO_ROTATION_MOTION_FLOOR_DPS, GYRO_ROTATION_STAGES, CaptureMethod, MetrologySample, MetrologyStage, MetrologyStatus, accelerometer_face_residuals, build_metrology_candidate, candidate_to_json, gyro_rotation_motion_window
    from imu_metrology import _require_numpy
    from imu_vibration_capture import samples_to_rows, validate_v1_provenance, write_csv
    from project_paths import IMU_METROLOGY_CALIBRATION_DIR, dated_directory, ensure_directory


SESSION_FORMAT = "drone-h743-v1-metrology-session"
SESSION_SCHEMA = 1
CAPTURE_SOURCE_PREFIX = "imucap-v4-session:"
# 被替换或手工删除的采集不物理销毁，只挪到这里：误删一次不至于让人重摆一遍机体。
DISCARDED_DIRNAME = "discarded"

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


# V1 只做手上真能做准的两组：六面加速度 + 陀螺静止零偏。这两项 PX4 也是强制项。
#
# 已移除的两组，以及为什么不是"改成可选"而是从流程里拿掉：
#   手动 +360° X/Y/Z —— 采集窗口被固件缓冲区限死在 6.1s，手转一圈实测只到 304°、
#     离轴 17.9%（2026-08-29）。据此解出的刻度修正是 1.18，而 MEMS 陀螺出厂刻度
#     误差本就只有 ~1%：标了比不标更差。PX4/ArduPilot 都没有陀螺刻度标定。
#   温度平台 —— 拟合斜率要求 ≥3 个平台、跨度 ≥15°C，没有温箱/冰箱根本凑不齐；
#     单个 room 点永远出不了结果，留在流程里只会一直挂着一行"未采集"。
#
# 两者的分析代码（analyze_gyro_rotations / analyze_temperature_drift）保留，老会话
# 里已有的证据仍能读出来；将来真有转台或温箱，在这里加回计划即可。
CAPTURE_PLANS = (
    *(CapturePlan(stage, label, 2000, 2000, CaptureMethod.BENCH) for stage, label in (
        (MetrologyStage.ACCEL_POS_X, "+X 机头朝上"), (MetrologyStage.ACCEL_NEG_X, "-X 机头朝下"),
        (MetrologyStage.ACCEL_POS_Y, "+Y 左侧朝上"), (MetrologyStage.ACCEL_NEG_Y, "-Y 右侧朝上"),
        (MetrologyStage.ACCEL_POS_Z, "+Z 水平"), (MetrologyStage.ACCEL_NEG_Z, "-Z 倒置"),
    )),
    CapturePlan(MetrologyStage.GYRO_STATIC, "陀螺仪静止", 4500, 4500, CaptureMethod.BENCH),
)
PLAN_BY_STAGE = {plan.stage: plan for plan in CAPTURE_PLANS}
# 从流程里拿掉、但老会话可能还存着的步骤：只读得出来，采不进去。
RETIRED_STAGE_LABELS = {
    MetrologyStage.GYRO_POS_360_X: "+X 手动精确 360°",
    MetrologyStage.GYRO_POS_360_Y: "+Y 手动精确 360°",
    MetrologyStage.GYRO_POS_360_Z: "+Z 手动精确 360°",
    MetrologyStage.TEMPERATURE_STATIC: "温度平台静止水平",
}


def minimum_samples(stage: MetrologyStage) -> int:
    """该步骤要想在分析里算数，最少需要多少样本（与 V1 验收门限同源）。"""

    if stage is MetrologyStage.GYRO_STATIC:
        return DEFAULT_THRESHOLDS.gyro_static_min_samples
    if stage is MetrologyStage.TEMPERATURE_STATIC:
        return DEFAULT_THRESHOLDS.temperature_min_samples_per_platform
    if stage in GYRO_ROTATION_STAGES:
        return DEFAULT_THRESHOLDS.gyro_rotation_min_samples_per_axis
    return DEFAULT_THRESHOLDS.accel_min_samples_per_face


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
    # 整体 FAIL 时必须说清"哪一步、为什么"，否则只能六面全重采。
    findings: tuple[str, ...] = ()
    face_residuals: dict[str, tuple[float, float]] = field(default_factory=dict)


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


def capture_key(record: CaptureRecord) -> tuple[str, str | None]:
    """同一步骤（温度步骤再按平台标签细分）只应保留一份证据。"""

    return record.stage, record.temperature_platform


def _archive_record(manifest_path: Path, record: CaptureRecord) -> None:
    """把一条采集的 CSV/meta 挪进 discarded/，manifest 之外仍可追溯。"""

    target = ensure_directory(manifest_path.parent / DISCARDED_DIRNAME)
    for name in (record.csv_path, record.meta_path):
        source = manifest_path.parent / name
        if not source.exists():
            continue
        destination = target / name
        counter = 1
        while destination.exists():
            destination = target / f"{source.stem}.{counter}{source.suffix}"
            counter += 1
        source.replace(destination)


def discard_capture(
    session: V1Session, manifest_path: Path, *, csv_path: str,
) -> tuple[V1Session, CaptureRecord]:
    """按 CSV 文件名删除一条采集；文件挪到 discarded/，不做物理销毁。"""

    matches = [record for record in session.captures if record.csv_path == csv_path]
    if not matches:
        raise ValueError(f"session {session.session_id} has no capture {csv_path}")
    record = matches[0]
    remaining = tuple(item for item in session.captures if item.csv_path != csv_path)
    _archive_record(manifest_path, record)
    updated = V1Session(session.session_id, session.created_at, _iso_now(), remaining)
    write_session(updated, manifest_path)
    return updated, record


@dataclass(frozen=True)
class CaptureHealth:
    """不跑完整分析就能给出的单条采集体检结论。"""

    level: str  # ok | short | rate | unreadable
    sample_count: int
    required_samples: int
    rate_hz: float
    gap_fraction: float
    detail: str


def summarise_rotation(rows: Sequence[dict[str, Any]], axis: int) -> tuple[str, str] | None:
    """把一次手转量成"转了多少度、离轴多少"；判据与 analyze_gyro_rotations 同源。

    零点用本段内静止样本的均值就地扣掉，不依赖陀螺静止那一步是否已经采过。
    numpy 缺席时返回 None，退回普通的样本量/采样率结论。
    """

    try:
        np = _require_numpy()
    except Exception:
        return None
    try:
        times = np.asarray([float(row["timestamp_us"]) for row in rows], dtype=float) / 1e6
        rates = np.asarray(
            [[float(row["gyro_filt_x_dps"]), float(row["gyro_filt_y_dps"]),
              float(row["gyro_filt_z_dps"])] for row in rows], dtype=float)
    except (KeyError, TypeError, ValueError):
        return None
    if times.size < 2:
        return None
    start, stop = gyro_rotation_motion_window(rates, axis)
    if stop - start < 2:
        return "angle", "整段都没有可辨识的转动：这一步没转起来，请重采"
    # 只有三轴同时静下来才算静止样本：主轴停住但机体还在晃时取零点，会把晃动
    # 当成 bias 减掉，离轴比例反而被算高。宁可不减零点。
    still = rates[np.max(np.abs(rates), axis=1) < GYRO_ROTATION_MOTION_FLOOR_DPS]
    zero = still.mean(axis=0) if still.shape[0] >= 100 else np.zeros(3)
    integrated = np.trapz(rates[start:stop] - zero, times[start:stop], axis=0)
    main = float(integrated[axis])
    off = max(abs(float(integrated[index])) for index in range(3) if index != axis)
    cross = off / abs(main) if main != 0.0 else 1.0
    window_s = float(times[stop - 1] - times[start])
    scale = 360.0 / main if main != 0.0 else 0.0
    problems = []
    if not DEFAULT_THRESHOLDS.gyro_singular_min <= scale <= DEFAULT_THRESHOLDS.gyro_singular_max:
        problems.append(f"缺 {360.0 - abs(main):+.0f}°" if abs(main) < 360.0 else f"多转 {abs(main) - 360.0:.0f}°")
    if cross > DEFAULT_THRESHOLDS.gyro_cross_axis_fail_fraction:
        problems.append(f"离轴 {cross:.1%} 超 {DEFAULT_THRESHOLDS.gyro_cross_axis_fail_fraction:.0%}")
    summary = f"转过 {main:+.0f}°（目标 360°）· 离轴 {cross:.1%} · 运动段 {window_s:.2f}s"
    if problems:
        return "angle", summary + " → " + "、".join(problems) + "，建议重采"
    return "ok", summary


CONTEXT_FIELDS = ("frame_contract", "orientation_code", "calibration_generation",
                  "base_frame", "firmware_image_crc32")
CONTEXT_LABELS = {
    "frame_contract": "坐标契约", "orientation_code": "安装朝向",
    "calibration_generation": "标定代次", "base_frame": "基准系",
    "firmware_image_crc32": "固件版本",
}


def context_conflicts(session: V1Session, manifest_path: Path) -> dict[str, str]:
    """找出采集上下文与多数派不一致的记录 —— 换过固件/改过标定代次都会分叉。

    返回 ``{csv_path: 说明}``。分析阶段这会让整份候选被拒，必须在点分析之前就看见。
    """

    contexts: dict[str, tuple] = {}
    for record in session.captures:
        try:
            meta = json.loads((manifest_path.parent / record.meta_path).read_text(encoding="utf-8"))
            header = meta["header"]
        except (OSError, KeyError, ValueError):
            continue
        contexts[record.csv_path] = tuple(header.get(name) for name in CONTEXT_FIELDS)
    if len(set(contexts.values())) < 2:
        return {}
    majority = max(set(contexts.values()), key=lambda value: list(contexts.values()).count(value))
    conflicts: dict[str, str] = {}
    for csv_path, context in contexts.items():
        differing = [
            f"{CONTEXT_LABELS[name]} {context[index]}≠{majority[index]}"
            for index, name in enumerate(CONTEXT_FIELDS) if context[index] != majority[index]
        ]
        if differing:
            conflicts[csv_path] = "、".join(differing) + "，与其余步骤不同源，必须重采"
    return conflicts


def probe_capture(manifest_path: Path, record: CaptureRecord) -> CaptureHealth:
    """读 CSV 判定采样率与样本量；分析前就把坏证据标红，别等分析整个炸掉。"""

    stage = MetrologyStage(record.stage)
    required = minimum_samples(stage)
    path = manifest_path.parent / record.csv_path
    try:
        with path.open("r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    except OSError as exc:
        return CaptureHealth("unreadable", 0, required, 0.0, 1.0, f"CSV 无法读取：{exc}")
    rate_hz, gap_fraction = measure_capture_rate(rows)
    if rate_hz <= 0.0:
        return CaptureHealth("unreadable", len(rows), required, 0.0, 1.0, "缺少可用时间戳")
    if rate_hz < V1_MIN_SAMPLE_RATE_HZ or rate_hz > V1_MAX_SAMPLE_RATE_HZ:
        return CaptureHealth(
            "rate", len(rows), required, rate_hz, gap_fraction,
            f"采样率 {rate_hz:.0f} Hz 不在 {V1_MIN_SAMPLE_RATE_HZ:.0f}~{V1_MAX_SAMPLE_RATE_HZ:.0f} Hz，"
            "DRDY 很可能失效；这条必须删除重采",
        )
    if gap_fraction > V1_MAX_GAP_FRACTION:
        return CaptureHealth(
            "rate", len(rows), required, rate_hz, gap_fraction,
            f"{gap_fraction:.1%} 的间隔超过中位 {V1_GAP_FACTOR:.0f} 倍，采集期间丢帧；这条必须删除重采",
        )
    if len(rows) < required:
        return CaptureHealth(
            "short", len(rows), required, rate_hz, gap_fraction,
            f"样本 {len(rows)} < 要求 {required}，本步不会计入分析",
        )
    if stage in GYRO_ROTATION_STAGES:
        # 转够了没有，必须当场知道：一次采集只有 6 s，等跑完整分析再发现差 56° 太晚了。
        angle = summarise_rotation(rows, GYRO_ROTATION_STAGES.index(stage))
        if angle is not None:
            level, detail = angle
            return CaptureHealth(level, len(rows), required, rate_hz, gap_fraction, detail)
    return CaptureHealth("ok", len(rows), required, rate_hz, gap_fraction, "可用")


def persist_capture(
    session: V1Session,
    manifest_path: Path,
    *,
    stage: MetrologyStage,
    samples: list[tuple],
    header: dict[str, Any],
    temperature_platform: str | None = None,
    supersede: bool = True,
) -> tuple[V1Session, CaptureRecord]:
    validate_v1_provenance(header)
    if stage not in PLAN_BY_STAGE:
        raise ValueError(
            f"{stage.value} 已从 V1 采集流程移除，不能再写入新证据"
            f"（{RETIRED_STAGE_LABELS.get(stage, stage.value)}）"
        )
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
    # 新证据已经落盘，此时才动旧的：中途失败不会两头落空。
    kept = session.captures
    if supersede:
        key = capture_key(record)
        superseded = tuple(item for item in kept if capture_key(item) == key)
        kept = tuple(item for item in kept if capture_key(item) != key)
        for old in superseded:
            _archive_record(manifest_path, old)
    updated = V1Session(session.session_id, session.created_at, _iso_now(), kept + (record,))
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
        try:
            result.extend(rows_to_metrology_samples(
                rows, meta["header"], stage=MetrologyStage(record.stage), session_id=session.session_id,
                provenance=f"imucap-v4:{record.meta_path}", capture_method=CaptureMethod(record.capture_method),
                temperature_platform=record.temperature_platform,
            ))
        except ValueError as exc:
            # 不说是哪一条，用户只能对着一句"采样率仅 50 Hz"干瞪眼。
            raise ValueError(f"{record.stage}（{record.csv_path}）：{exc}") from exc
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
    # V1 的产出就是室温基础候选；转动/温度两组已不在采集流程内，不再当作"待完成"。
    usable = {MetrologyStatus.PASS, MetrologyStatus.WARN}
    if (candidate.accelerometer.status in usable
            and candidate.gyro_static.status in usable):
        note = ("" if candidate.accelerometer.status is MetrologyStatus.PASS
                else "（六面摆放一致性只到 WARN，代价见下方说明，可用）")
        status_line = f"V1 室温基础候选已生成{note}：可先应用到 RAM 复验，再写入参数 Flash"
    else:
        status_line = (
            "V1 室温基础候选未通过："
            f"六面={candidate.accelerometer.status.value}、"
            f"陀螺静止={candidate.gyro_static.status.value}；请按表格提示重采后再分析"
        )
    return AnalysisSummary(
        candidate_path, references_path, candidate.status, candidate.accelerometer.status,
        candidate.gyro_static.status, candidate.gyro_rotation.status, candidate.temperature.status,
        status_line, len(samples),
        findings=tuple(candidate.accelerometer.findings) + tuple(candidate.gyro_static.findings),
        face_residuals=accelerometer_face_residuals(samples, candidate.accelerometer),
    )


__all__ = ["AnalysisSummary", "CAPTURE_PLANS", "CaptureHealth", "CapturePlan", "CaptureRecord", "DISCARDED_DIRNAME", "V1Session", "analyze_session", "capture_key", "context_conflicts", "discard_capture", "latest_session_manifest", "load_session", "load_session_samples", "minimum_samples", "new_session", "RETIRED_STAGE_LABELS", "persist_capture", "probe_capture", "rows_to_metrology_samples", "summarise_rotation", "session_from_dict", "session_to_dict", "write_session"]
