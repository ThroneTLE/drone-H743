"""V0 validation evidence persistence, restoration, and write guards."""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
import re
import time
from datetime import datetime
from tkinter import messagebox

from .proto import PROTO_REQ_IMU, PROTO_REQ_IMU_FRAME, safe_int

try:
    from ...flight_validation import (
        STAGE_DEFINITIONS,
        ImuSample,
        ValidationSession,
        ValidationStage,
        ValidationStatus,
        analyze_rotation_stage,
        analyze_six_face,
        analyze_static_stage,
        build_validation_report,
        load_session,
        write_csv_report,
        write_json_report,
        write_session,
    )
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.flight_validation import (
            STAGE_DEFINITIONS,
            ImuSample,
            ValidationSession,
            ValidationStage,
            ValidationStatus,
            analyze_rotation_stage,
            analyze_six_face,
            analyze_static_stage,
            build_validation_report,
            load_session,
            write_csv_report,
            write_json_report,
            write_session,
        )
    except ImportError:
        from flight_validation import (
            STAGE_DEFINITIONS,
            ImuSample,
            ValidationSession,
            ValidationStage,
            ValidationStatus,
            analyze_rotation_stage,
            analyze_six_face,
            analyze_static_stage,
            build_validation_report,
            load_session,
            write_csv_report,
            write_json_report,
            write_session,
        )

try:
    from ..project_paths import AIRFRAME_CALIBRATION_DIR, dated_directory, ensure_directory
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.project_paths import AIRFRAME_CALIBRATION_DIR, dated_directory, ensure_directory
    except ImportError:
        from project_paths import AIRFRAME_CALIBRATION_DIR, dated_directory, ensure_directory


VALIDATION_SAMPLE_FRESH_S = 1.5
VALIDATION_HEALTH_FRESH_S = 3.0
VALIDATION_MOTOR_SAFE_MAX_US = 1100
VALIDATION_STATIC_TARGET_SAMPLES = 50
VALIDATION_ROTATION_TARGET_SAMPLES = 30
VALIDATION_AUTOSAVE_PERIOD_S = 2.0
FIRMWARE_BOOT_COMMAND = "BOOT DFU CONFIRM"
VALIDATION_UI_STAGES = tuple(
    stage for stage, definition in STAGE_DEFINITIONS.items() if definition.required
)
VALIDATION_ALLOWED_COMMANDS = frozenset(
    {
        "PING",
        "CAPS?",
        "MODULES?",
        "STATUS?",
        "REGISTRY?",
        "IMU?",
        "CONFIG?",
        "PARAM?",
        "PID?",
        "AIRFRAME?",
        "BARO?",
        "FLOW?",
        "RANGE?",
        "GPS?",
        "MAG?",
        "RTOS?",
        "FLASH?",
        "WIFI?",
        "IDENT?",
        "IMUCAP?",
        "IMUCAP START",
        "IMUCAP STOP",
        "IMUCAP DUMP",
        "IMUCAP CANCEL",
        "SERVOCAL?",
    }
)
VALIDATION_SNAPSHOT_REQUIRED_FIELDS = (
    "valid", "source", "frame", "units", "contract", "migration", "ts_ms", "seq",
    "bias", "armed", "m1", "m2", "ax_mg", "ay_mg", "az_mg",
    "gx_mdps", "gy_mdps", "gz_mdps",
)


def validation_history_artifacts(root: Path) -> tuple[tuple[str, Path], ...]:
    """Catalog every V0 capture group and keep completed reports reachable."""

    if not root.is_dir():
        return ()
    groups: dict[str, dict[str, Path]] = {}
    for path in root.rglob("flu_acceptance_v0_*_*.json"):
        if path.name.endswith("_session.json"):
            prefix = path.name.removesuffix("_session.json")
            groups.setdefault(str(path.with_name(prefix)), {})["session"] = path
        elif path.name.endswith("_report.json"):
            prefix = path.name.removesuffix("_report.json")
            groups.setdefault(str(path.with_name(prefix)), {})["report"] = path

    rows: list[tuple[str, Path, str]] = []
    for prefix_path, artifacts in groups.items():
        session = artifacts.get("session")
        report = artifacts.get("report")
        session_has_samples = False
        if session is not None:
            try:
                payload = json.loads(session.read_text(encoding="utf-8"))
                stages = payload.get("samples_by_stage") if isinstance(payload, dict) else None
                session_has_samples = isinstance(stages, dict) and any(
                    isinstance(value, list) and bool(value) for value in stages.values())
            except (OSError, TypeError, ValueError, json.JSONDecodeError):
                session_has_samples = False
        selected = session if session_has_samples else report or session
        if selected is None:
            continue
        prefix = Path(prefix_path).name
        match = re.search(r"(20\d{6})_(\d{6})(?:_(\d{3}))?$", prefix)
        if match is not None:
            date_token, time_token, millis = match.groups()
            display_time = (
                f"{date_token[0:4]}-{date_token[4:6]}-{date_token[6:8]} "
                f"{time_token[0:2]}:{time_token[2:4]}:{time_token[4:6]}"
                + (f".{millis}" if millis else "")
            )
            sort_key = date_token + time_token + (millis or "")
        else:
            display_time = prefix
            sort_key = prefix
        if session_has_samples and report is not None:
            kind = "可恢复会话 + 报告"
        elif session_has_samples:
            kind = "可恢复会话"
        else:
            kind = "完成报告"
        rows.append((f"{display_time} · {kind}", selected, sort_key))
    rows.sort(key=lambda item: item[2], reverse=True)
    return tuple((label, path) for label, path, _sort in rows)


def validation_sample_from_snapshot(
    values: dict[str, str],
) -> tuple[ImuSample | None, str | None]:
    """Parse one complete target snapshot without inventing missing zero values."""

    missing = [key for key in VALIDATION_SNAPSHOT_REQUIRED_FIELDS if key not in values]
    if missing:
        return None, "incomplete:" + ",".join(missing)
    if values.get("valid") != "1":
        return None, "target snapshot is not valid"
    if values.get("source") != "stabilizer_snapshot":
        return None, f"unsupported source={values.get('source', '-')}"
    if values.get("units") != "mg_mdps_cdeg":
        return None, f"unsupported units={values.get('units', '-')}"
    try:
        timestamp_ms = int(values["ts_ms"], 0)
        sample = ImuSample(
            timestamp_s=timestamp_ms * 0.001,
            accel_x_g=float(values["ax_mg"]) * 0.001,
            accel_y_g=float(values["ay_mg"]) * 0.001,
            accel_z_g=float(values["az_mg"]) * 0.001,
            gyro_x_dps=float(values["gx_mdps"]) * 0.001,
            gyro_y_dps=float(values["gy_mdps"]) * 0.001,
            gyro_z_dps=float(values["gz_mdps"]) * 0.001,
            roll_deg=float(values["roll_cdeg"]) * 0.01 if "roll_cdeg" in values else None,
            pitch_deg=float(values["pitch_cdeg"]) * 0.01 if "pitch_cdeg" in values else None,
            yaw_deg=float(values["yaw_cdeg"]) * 0.01 if "yaw_cdeg" in values else None,
        )
    except (KeyError, ValueError, OverflowError):
        return None, "snapshot contains an invalid numeric field"
    return sample, None


def validation_samples_from_csv(
    path: Path | str,
) -> tuple[dict[ValidationStage, list[ImuSample]], list[dict[str, str]]]:
    """Restore samples from a V0 raw CSV without guessing missing measurements."""

    source = Path(path)
    samples = {stage: [] for stage in STAGE_DEFINITIONS}
    raw_rows: list[dict[str, str]] = []
    with source.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"验收样本 CSV 没有表头：{source}")
        for row_number, row in enumerate(reader, start=2):
            cleaned = {str(key): value or "" for key, value in row.items() if key is not None}
            raw_rows.append(cleaned)
            stage_text = cleaned.get("stage", "").strip()
            if not stage_text:
                continue
            try:
                stage = ValidationStage(stage_text)
            except ValueError as exc:
                raise ValueError(f"验收样本 CSV 第 {row_number} 行阶段未知：{stage_text}") from exc
            if cleaned.get("event", "").strip():
                continue

            timestamp_text = cleaned.get("target_timestamp_ms", "").strip()
            scale = 0.001
            if not timestamp_text:
                timestamp_text = cleaned.get("target_timestamp_us", "").strip()
                scale = 0.000001
            if not timestamp_text:
                timestamp_text = cleaned.get("timestamp_s", "").strip()
                scale = 1.0
            required_names = (
                "accel_x_g",
                "accel_y_g",
                "accel_z_g",
                "gyro_x_dps",
                "gyro_y_dps",
                "gyro_z_dps",
            )
            if not timestamp_text or any(not cleaned.get(name, "").strip() for name in required_names):
                raise ValueError(f"验收样本 CSV 第 {row_number} 行缺少原始测量字段")

            try:
                required_values = [float(cleaned[name]) for name in required_names]
                timestamp_s = float(timestamp_text) * scale
                optional_values = [
                    None if not cleaned.get(name, "").strip() else float(cleaned[name])
                    for name in ("roll_deg", "pitch_deg", "yaw_deg")
                ]
            except ValueError as exc:
                raise ValueError(f"验收样本 CSV 第 {row_number} 行数值无效") from exc
            numeric_values = [timestamp_s, *required_values, *(v for v in optional_values if v is not None)]
            if not all(math.isfinite(value) for value in numeric_values):
                raise ValueError(f"验收样本 CSV 第 {row_number} 行包含 NaN/Infinity")
            samples[stage].append(
                ImuSample(
                    timestamp_s=timestamp_s,
                    accel_x_g=required_values[0],
                    accel_y_g=required_values[1],
                    accel_z_g=required_values[2],
                    gyro_x_dps=required_values[3],
                    gyro_y_dps=required_values[4],
                    gyro_z_dps=required_values[5],
                    roll_deg=optional_values[0],
                    pitch_deg=optional_values[1],
                    yaw_deg=optional_values[2],
                )
            )
    return samples, raw_rows


def signed_permutation_descriptor(matrix: object) -> str | None:
    """Return the stable firmware descriptor for a proper 3-D rotation."""

    if not isinstance(matrix, (tuple, list)) or len(matrix) != 3:
        return None
    rows: list[tuple[int, int, int]] = []
    for row in matrix:
        if not isinstance(row, (tuple, list)) or len(row) != 3:
            return None
        try:
            values = tuple(int(value) for value in row)
        except (TypeError, ValueError):
            return None
        if any(value not in (-1, 0, 1) for value in values) or sum(value != 0 for value in values) != 1:
            return None
        rows.append(values)  # type: ignore[arg-type]
    if any(sum(rows[row][column] != 0 for row in range(3)) != 1 for column in range(3)):
        return None
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if determinant != 1:
        return None
    axes = "xyz"
    terms: list[str] = []
    for row in rows:
        index = next(index for index, value in enumerate(row) if value != 0)
        terms.append(("+" if row[index] > 0 else "-") + axes[index])
    return ",".join(terms)


class EvidenceMixin:
    def _validation_live_safety_gate(self) -> tuple[bool, str]:
        """V1/V2A 会真正驱动舵机，主机这一侧必须自己确认飞控 disarmed。

        这里和固件升级不同：固件升级有 BOOT 处理器兜底，而 ACCEPT/V1 命令没有等价
        的目标侧拒绝，所以必须要求一份新鲜快照。
        """
        ok, reason = self._firmware_link_gate()
        if not ok:
            return ok, reason
        level, advisory = self._firmware_safety_advisory()
        if level != "ok":
            return False, advisory
        return True, f"{reason} · {advisory}"

    def _validation_write_orientation_audit(self, event: str, *, flash_writes: int) -> Path:
        output_dir = dated_directory(AIRFRAME_CALIBRATION_DIR)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = output_dir / f"flu_orientation_{event}_{stamp}.json"
        payload = {
            "format": "drone-h743-flu-orientation-application",
            "schema": 1,
            "created_at": datetime.now().astimezone().isoformat(),
            "event": event,
            "base_frame": "legacy_intermediate_v1",
            "target_frame": "canonical_flu",
            "contract": 1,
            "candidate_matrix_flu_from_legacy": self.validation_candidate_matrix,
            "candidate_descriptor": self.validation_candidate_descriptor,
            "candidate_confidence": self.validation_candidate_confidence,
            "candidate_source": (
                str(self.validation_candidate_source_path)
                if self.validation_candidate_source_path is not None
                else None
            ),
            "target_report": dict(self.validation_orientation_values),
            "verification_mode": self.validation_verification_mode,
            "verification_passed": self.validation_candidate_verified,
            "required_stage_status": {
                stage.value: (
                    getattr(self.validation_stage_results.get(stage), "status").value
                    if self.validation_stage_results.get(stage) is not None
                    else ValidationStatus.NOT_RUN.value
                )
                for stage in VALIDATION_UI_STAGES
            },
            "ram_applied": self.validation_candidate_applied,
            "persisted": self.validation_candidate_committed,
            "flash_writes": flash_writes,
            "firmware_written": False,
            "flight_release": False,
        }
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
        except OSError as exc:
            self._append(f"[V0] 映射操作已执行，但PC审计文件保存失败：{exc}")
        self.validation_orientation_audit_path = path
        return path

    @staticmethod
    def _validation_raw_row_from_sample(
        stage: ValidationStage,
        sample: ImuSample,
    ) -> dict[str, object]:
        return {
            "stage": stage.value,
            "event": "",
            "target_timestamp_ms": round(sample.timestamp_s * 1000.0, 3),
            "sequence": "",
            "accel_x_g": sample.accel_x_g,
            "accel_y_g": sample.accel_y_g,
            "accel_z_g": sample.accel_z_g,
            "gyro_x_dps": sample.gyro_x_dps,
            "gyro_y_dps": sample.gyro_y_dps,
            "gyro_z_dps": sample.gyro_z_dps,
            "roll_deg": "" if sample.roll_deg is None else sample.roll_deg,
            "pitch_deg": "" if sample.pitch_deg is None else sample.pitch_deg,
            "yaw_deg": "" if sample.yaw_deg is None else sample.yaw_deg,
            "source": "restored_validation_session",
            "frame": "legacy_intermediate",
            "units": "g_dps_deg",
            "contract": 1,
            "migration": "0x00",
            "bias": "",
            "armed": "",
            "m1": "",
            "m2": "",
        }

    def _validation_autosave_session(self, *, force: bool = False) -> Path | None:
        if self.validation_loaded_history:
            # 只读浏览历史验收时绝不回写磁盘（历史证据不可变）。
            # “新建独立验收”/“继续历史会话”都会显式清除该标志后才允许保存。
            return None
        has_state = (
            any(self.validation_samples.values())
            or bool(self.validation_unsupported_stages)
            or bool(self.validation_skipped_stages)
            or bool(self.validation_finished_stages)
        )
        if not has_state:
            return None
        if self.validation_session_path is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self.validation_session_path = (
                dated_directory(AIRFRAME_CALIBRATION_DIR)
                / f"flu_acceptance_v0_{stamp}_session.json"
            )
        now_monotonic = time.monotonic()
        if (
            not force
            and self.validation_last_autosave_monotonic > 0.0
            and (now_monotonic - self.validation_last_autosave_monotonic)
            < VALIDATION_AUTOSAVE_PERIOD_S
        ):
            return self.validation_session_path
        timestamp = datetime.now().astimezone().isoformat()
        firmware_hash = self.validation_firmware_hash_var.get().strip() or None
        if firmware_hash and firmware_hash.startswith("unknown"):
            firmware_hash = "unknown"
        session_kwargs: dict[str, object] = {
            "samples_by_stage": {
                stage: tuple(samples) for stage, samples in self.validation_samples.items()
            },
            "unsupported_stages": frozenset(self.validation_unsupported_stages),
            "skipped_stages": frozenset(self.validation_skipped_stages),
            "finished_stages": frozenset(self.validation_finished_stages),
            "firmware_hash": firmware_hash,
            "updated_at": timestamp,
        }
        if self.validation_session_created_at is not None:
            session_kwargs["created_at"] = self.validation_session_created_at
        session = ValidationSession(**session_kwargs)
        self.validation_session_created_at = session.created_at
        try:
            output = write_session(session, self.validation_session_path)
        except (OSError, TypeError, ValueError) as exc:
            self._append(f"[V0] 会话自动保存失败：{exc}")
            return None
        self.validation_last_autosave_monotonic = now_monotonic
        self._validation_autosave_workflow()
        return output

    def _validation_autosave_workflow(self) -> Path | None:
        if self.validation_loaded_history:
            # 同上：workflow 载荷含实时目标状态快照；在只读浏览的历史工作区里
            # 回写会用可能为空的实时状态覆盖归档快照，属证据破坏。
            return None
        if (
            not self.validation_verification_mode
            or self.validation_session_path is None
            or self.validation_candidate_matrix is None
            or not self.validation_candidate_descriptor
        ):
            return None
        prefix = self.validation_session_path.name.removesuffix("_session.json")
        path = self.validation_session_path.with_name(prefix + "_workflow.json")
        temporary = path.with_name(path.name + ".tmp")
        payload = {
            "format": "drone-h743-flu-orientation-workflow",
            "schema": 1,
            "updated_at": datetime.now().astimezone().isoformat(),
            "phase": "ram_verification",
            "candidate_matrix_flu_from_legacy": self.validation_candidate_matrix,
            "candidate_descriptor": self.validation_candidate_descriptor,
            "candidate_confidence": self.validation_candidate_confidence,
            "candidate_source": (
                str(self.validation_candidate_source_path)
                if self.validation_candidate_source_path is not None
                else None
            ),
            "target_state_at_save": dict(self.validation_orientation_values),
            "target_state_requires_recheck": True,
            "flight_release": False,
        }
        try:
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            temporary.replace(path)
        except OSError as exc:
            self._append(f"[V0] 复验工作流自动保存失败：{exc}")
            return None
        return path

    @staticmethod
    def _validation_load_workflow(session_path: Path) -> dict[str, object] | None:
        prefix = session_path.name.removesuffix("_session.json")
        path = session_path.with_name(prefix + "_workflow.json")
        if not path.exists():
            return None
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("验收 workflow 根节点不是对象")
        if payload.get("format") != "drone-h743-flu-orientation-workflow":
            raise ValueError("验收 workflow format 不支持")
        if payload.get("schema") != 1 or payload.get("phase") != "ram_verification":
            raise ValueError("验收 workflow schema/phase 不支持")
        descriptor = payload.get("candidate_descriptor")
        matrix = payload.get("candidate_matrix_flu_from_legacy")
        if not isinstance(descriptor, str) or signed_permutation_descriptor(matrix) != descriptor:
            raise ValueError("验收 workflow 候选矩阵与descriptor不一致")
        confidence = payload.get("candidate_confidence")
        if isinstance(confidence, bool) or not isinstance(confidence, (int, float)):
            raise ValueError("验收 workflow confidence 无效")
        if not math.isfinite(float(confidence)) or not 0.0 <= float(confidence) <= 1.0:
            raise ValueError("验收 workflow confidence 越界")
        return payload

    def _validation_apply_loaded_session(
        self,
        session: ValidationSession,
        path: Path,
        *,
        raw_rows: list[dict[str, str]] | None = None,
        report_path: Path | None = None,
        workflow: dict[str, object] | None = None,
    ) -> None:
        self.validation_session_active = False
        self.validation_poll_requested = False
        self.validation_active_stage = None
        self.validation_samples = {
            stage: list(session.samples_by_stage.get(stage, ()))
            for stage in STAGE_DEFINITIONS
        }
        self.validation_unsupported_stages = set(session.unsupported_stages)
        self.validation_skipped_stages = set(session.skipped_stages)
        self.validation_finished_stages = set(session.finished_stages)
        self.validation_session_path = path
        self.validation_session_created_at = session.created_at
        self.validation_loaded_report_path = report_path
        self.validation_loaded_history = True
        self.validation_last_autosave_monotonic = 0.0
        self.validation_candidate_matrix = None
        self.validation_candidate_descriptor = ""
        self.validation_candidate_confidence = 0.0
        self.validation_candidate_source_path = None
        self.validation_candidate_applied = False
        self.validation_candidate_verified = False
        self.validation_candidate_committed = False
        self.validation_verification_mode = False
        if raw_rows is None:
            self.validation_raw_rows = [
                self._validation_raw_row_from_sample(stage, sample)
                for stage, samples in self.validation_samples.items()
                for sample in samples
            ]
        else:
            self.validation_raw_rows = [dict(row) for row in raw_rows]
        if workflow is not None:
            matrix = workflow["candidate_matrix_flu_from_legacy"]
            self.validation_candidate_matrix = tuple(  # type: ignore[assignment]
                tuple(int(value) for value in row) for row in matrix  # type: ignore[union-attr]
            )
            self.validation_candidate_descriptor = str(workflow["candidate_descriptor"])
            self.validation_candidate_confidence = float(workflow["candidate_confidence"])
            source = workflow.get("candidate_source")
            self.validation_candidate_source_path = Path(source) if isinstance(source, str) else None
            self.validation_verification_mode = True
            self.validation_candidate_applied = False
            self.validation_candidate_verified = False
            self.validation_candidate_committed = False
        if session.firmware_hash:
            suffix = "（历史记录，未与当前连接核对）"
            self.validation_firmware_hash_var.set(f"{session.firmware_hash}{suffix}")
        self._validation_rebuild_results()
        self._validation_refresh_candidate_summary()

        next_stage = next(
            (
                stage
                for stage in VALIDATION_UI_STAGES
                if stage not in self.validation_finished_stages
                and stage not in self.validation_skipped_stages
                and stage not in self.validation_unsupported_stages
            ),
            VALIDATION_UI_STAGES[0],
        )
        self.validation_stage_tree.selection_set(next_stage.value)
        self.validation_stage_tree.focus(next_stage.value)
        self.validation_stage_var.set(next_stage.value)
        self.validation_instruction_var.set(STAGE_DEFINITIONS[next_stage].prompt_zh)
        self._validation_refresh_stage_view(next_stage)
        self.validation_source_var.set(
            f"历史证据：{path}；尚未与当前连接、当前固件做身份核对"
        )
        if report_path is not None:
            prefix = report_path.name.removesuffix("_report.json")
            summary_path = report_path.with_name(prefix + "_summary.csv")
            raw_path = report_path.with_name(prefix + "_samples.csv")
            if summary_path.exists() and raw_path.exists():
                self.validation_report_paths = (report_path, summary_path, raw_path)
            self.validation_report_var.set(f"历史报告：{report_path}")
        else:
            self.validation_report_paths = None
            self.validation_report_var.set(f"自动保存会话：{path}")
        self._validation_set_status(
            ValidationStatus.WARN,
            "已加载历史验收；结果仅是PC证据，尚未写入固件",
        )
        self._validation_refresh_readiness()

    def _validation_apply_report_only(self, report_path: Path, payload: dict[str, object]) -> None:
        self.validation_loaded_report_path = report_path
        self.validation_loaded_history = True
        self.validation_report_var.set(f"历史报告（缺少原始样本，不能续测）：{report_path}")
        six_face = payload.get("six_face")
        rotations = payload.get("rotations")
        stage_payloads: dict[str, object] = {}
        if isinstance(six_face, dict) and isinstance(six_face.get("stages"), dict):
            stage_payloads.update(six_face["stages"])
        if isinstance(rotations, dict) and isinstance(rotations.get("stages"), dict):
            stage_payloads.update(rotations["stages"])
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            item = stage_payloads.get(stage.value)
            status = ValidationStatus.NOT_RUN.value
            count = 0
            if isinstance(item, dict):
                status = str(item.get("status", status))
                count = safe_int(str(item.get("sample_count", 0)), 0)
            label = definition.label_zh if definition.required else f"{definition.label_zh}（可选）"
            self.validation_stage_tree.item(stage.value, text=label, values=(status, count))
        if isinstance(six_face, dict) and isinstance(six_face.get("candidate"), dict):
            candidate = six_face["candidate"]
            matrix = candidate.get("matrix_flu_from_observed")
            confidence = candidate.get("confidence", 0.0)
            try:
                confidence_text = f"{float(confidence):.2%}"
            except (TypeError, ValueError):
                confidence_text = "unknown"
            self.validation_candidate_var.set(
                "历史轴映射候选："
                f"R_FLU←observed={matrix} · det={candidate.get('determinant')} · "
                f"confidence={confidence_text} · 未应用"
            )
        self.validation_source_var.set("仅加载历史报告摘要；原始 samples.csv 缺失")
        self._validation_set_status(
            ValidationStatus.WARN,
            "历史报告已加载，但缺少原始样本，不能继续采集或重新分析",
        )
        self._validation_refresh_readiness()

    def _validation_load_report_artifact(self, report_path: Path) -> None:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("验收报告根节点不是对象")
        prefix = report_path.name.removesuffix("_report.json")
        raw_path = report_path.with_name(prefix + "_samples.csv")
        if not raw_path.exists():
            self._validation_apply_report_only(report_path, payload)
            return
        samples, raw_rows = validation_samples_from_csv(raw_path)
        stage_payloads: dict[str, object] = {}
        for group_name in ("six_face", "rotations"):
            group = payload.get(group_name)
            if isinstance(group, dict) and isinstance(group.get("stages"), dict):
                stage_payloads.update(group["stages"])
        unsupported: set[ValidationStage] = set()
        skipped: set[ValidationStage] = set()
        finished: set[ValidationStage] = {
            stage for stage, stage_samples in samples.items() if stage_samples
        }
        for stage in STAGE_DEFINITIONS:
            item = stage_payloads.get(stage.value)
            status = item.get("status") if isinstance(item, dict) else None
            if status == ValidationStatus.UNSUPPORTED.value:
                unsupported.add(stage)
                finished.add(stage)
            elif status == ValidationStatus.SKIPPED.value:
                skipped.add(stage)
                finished.add(stage)
            elif status in {
                ValidationStatus.PASS.value,
                ValidationStatus.WARN.value,
                ValidationStatus.FAIL.value,
            }:
                finished.add(stage)
        created_at = payload.get("created_at")
        if not isinstance(created_at, str) or not created_at.strip():
            created_at = datetime.fromtimestamp(report_path.stat().st_mtime).astimezone().isoformat()
        firmware_hash = payload.get("firmware_hash")
        if not isinstance(firmware_hash, str):
            firmware_hash = None
        session = ValidationSession(
            samples_by_stage=samples,
            unsupported_stages=unsupported,
            skipped_stages=skipped,
            finished_stages=finished,
            firmware_hash=firmware_hash,
            created_at=created_at,
            updated_at=datetime.now().astimezone().isoformat(),
        )
        session_path = report_path.with_name(prefix + "_session.json")
        write_session(session, session_path)
        self._validation_apply_loaded_session(
            session,
            session_path,
            raw_rows=raw_rows,
            report_path=report_path,
        )

    def _validation_refresh_history_choices(self) -> None:
        catalog = validation_history_artifacts(Path(AIRFRAME_CALIBRATION_DIR))
        self.validation_history_paths = {label: path for label, path in catalog}
        values = tuple(self.validation_history_paths)
        if hasattr(self, "validation_history_combo"):
            self.validation_history_combo.configure(values=values)
        current = self.validation_history_var.get()
        if current not in self.validation_history_paths:
            self.validation_history_var.set(
                values[0] if values else "没有找到历史验收")

    def _validation_load_artifact(self, artifact: Path) -> None:
        if artifact.name.endswith("_session.json"):
            session = load_session(artifact)
            prefix = artifact.name.removesuffix("_session.json")
            report_path = artifact.with_name(prefix + "_report.json")
            raw_path = artifact.with_name(prefix + "_samples.csv")
            raw_rows: list[dict[str, str]] | None = None
            if raw_path.exists():
                _samples, raw_rows = validation_samples_from_csv(raw_path)
            self._validation_apply_loaded_session(
                session,
                artifact,
                raw_rows=raw_rows,
                report_path=report_path if report_path.exists() else None,
                workflow=self._validation_load_workflow(artifact),
            )
        elif artifact.name.endswith("_report.json"):
            self._validation_load_report_artifact(artifact)
        else:
            raise ValueError("只支持 V0 的 session.json 或 report.json")

    def _validation_load_selected_history(self) -> None:
        selected = self.validation_history_paths.get(
            self.validation_history_var.get())
        if selected is None:
            messagebox.showinfo("没有历史验收", "请先点击“刷新历史列表”。")
            return
        if self.validation_session_active:
            if not messagebox.askyesno(
                "切换历史验收",
                "当前验收正在运行。切换前会先自动保存当前样本，不会删除任何历史文件。继续？",
            ):
                return
            self._validation_autosave_session(force=True)
        try:
            self._validation_load_artifact(selected)
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            messagebox.showerror("历史验收加载失败", str(exc))
            return
        self.validation_history_var.set(next(
            (label for label, path in self.validation_history_paths.items()
             if path == selected),
            self.validation_history_var.get(),
        ))
        self._validation_refresh_readiness()

    def _validation_load_latest_artifact(self) -> None:
        try:
            self._validation_refresh_history_choices()
            catalog = validation_history_artifacts(Path(AIRFRAME_CALIBRATION_DIR))
            if not catalog:
                return
            label, latest = catalog[0]
            self.validation_history_var.set(label)
            self._validation_load_artifact(latest)
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.validation_report_var.set(f"历史验收加载失败：{exc}")
            self._validation_set_status(ValidationStatus.WARN, "历史验收文件无效，未自动使用")

    def _validation_resume_session(self) -> None:
        if not any(self.validation_samples.values()) and not (
            self.validation_skipped_stages or self.validation_unsupported_stages
        ):
            messagebox.showinfo("没有可继续的会话", "未加载到带原始样本的历史验收会话。")
            return
        if not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("安全门未通过", "继续验收前仍须确认已拆桨，并隔离动力或固定机体。")
            return
        if not self._transport_connected():
            messagebox.showwarning("尚未连接", "请先建立到飞控的连接，再继续历史会话。")
            return
        for transport in (self.tcp_transport, self.udp_transport, self.serial_transport):
            transport.cancel_pending_sends()
        self.validation_session_active = True
        self.validation_poll_requested = True
        self.validation_active_stage = None
        self.validation_loaded_history = False
        self.validation_latest_sequence = None
        self.validation_latest_values.clear()
        self.validation_latest_host_time = 0.0
        self._validation_set_status(
            ValidationStatus.WARN,
            "历史样本已保留；等待当前目标快照后继续未完成步骤",
        )
        self._send_proto(PROTO_REQ_IMU, "IMU?")
        self._send_proto(PROTO_REQ_IMU_FRAME, "IMUFRAME?")
        self._validation_refresh_readiness()

    def _validation_write_report(self) -> None:
        if self.validation_active_stage is not None:
            messagebox.showinfo("步骤仍在采集", "请先点击“停止并分析”，再生成一致的报告。")
            return
        if not any(self.validation_samples.values()) and not self.validation_unsupported_stages:
            messagebox.showinfo("没有数据", "至少完成一个验收步骤后才能生成报告。")
            return
        firmware_hash = self.validation_firmware_hash_var.get().split("（", 1)[0].strip() or "unknown"
        if firmware_hash.startswith("unknown"):
            firmware_hash = "unknown"
        report = build_validation_report(
            self._validation_stage_inputs(),
            firmware_hash=firmware_hash,
            data_source="stabilizer_snapshot_10hz_legacy_intermediate",
        )
        stamp = time.strftime("%Y%m%d_%H%M%S")
        output_dir = dated_directory(AIRFRAME_CALIBRATION_DIR)
        output_dir.mkdir(parents=True, exist_ok=True)
        base = output_dir / f"flu_acceptance_v0_{stamp}"
        json_path = write_json_report(report, base.with_name(base.name + "_report.json"))
        summary_csv_path = write_csv_report(report, base.with_name(base.name + "_summary.csv"))
        raw_path = base.with_name(base.name + "_samples.csv")
        fieldnames = [
            "stage", "event", "target_timestamp_ms", "sequence", "accel_x_g", "accel_y_g", "accel_z_g",
            "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "roll_deg", "pitch_deg", "yaw_deg",
            "source", "frame", "units", "contract", "migration", "orientation", "bias", "armed", "m1", "m2",
        ]
        with raw_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(
                {key: row.get(key, "") for key in fieldnames}
                for row in self.validation_raw_rows
            )
        self.validation_report_paths = (json_path, summary_csv_path, raw_path)
        self.validation_loaded_report_path = json_path
        self.validation_session_path = base.with_name(base.name + "_session.json")
        self.validation_report_var.set(f"报告：{json_path}")
        self._validation_autosave_session(force=True)
        self._validation_set_status(report.status, "PC 端只读证据已生成；flight_release=false")

    def _validation_open_report_dir(self) -> None:
        directory = (
            self.validation_report_paths[0].parent
            if self.validation_report_paths is not None
            else dated_directory(AIRFRAME_CALIBRATION_DIR)
        )
        directory.mkdir(parents=True, exist_ok=True)
        if hasattr(os, "startfile"):
            os.startfile(directory)  # type: ignore[attr-defined]
        else:
            messagebox.showinfo("报告目录", str(directory))

    def _validation_command_allowed(self, payload: str) -> bool:
        command = " ".join(payload.strip().upper().split())
        if command.startswith("BOOT"):
            return (
                command == FIRMWARE_BOOT_COMMAND
                and command in getattr(self, "firmware_authorized_commands", set())
            )
        if getattr(self, "firmware_update_pending", False) or getattr(
            self, "firmware_update_running", False
        ):
            return False
        if command == "IMUFRAME?":
            return True
        if command.startswith("IMUFRAME "):
            return command in getattr(self, "validation_authorized_orientation_commands", set())
        if not self.validation_session_active:
            return True
        if command in getattr(self, "validation_authorized_orientation_commands", set()):
            return True
        return command in VALIDATION_ALLOWED_COMMANDS

    def _validation_guard_command(self, payload: str) -> bool:
        if self._validation_command_allowed(payload):
            return True
        if " ".join(payload.strip().upper().split()).startswith("BOOT"):
            self._append(f"[FIRMWARE SAFETY] blocked command: {payload}")
            messagebox.showwarning(
                "固件升级安全门",
                "BOOT 命令只能由“固件升级”页在实时安全快照通过后单次授权。",
            )
            return False
        self._append(f"[V0 READ-ONLY] blocked command: {payload}")
        self._validation_set_status(ValidationStatus.FAIL, f"只读门阻止命令：{payload}")
        messagebox.showwarning("坐标系校准只读门", f"验收会话期间禁止执行：{payload}")
        return False


__all__ = [
    "EvidenceMixin",
    "FIRMWARE_BOOT_COMMAND",
    "VALIDATION_ALLOWED_COMMANDS",
    "VALIDATION_AUTOSAVE_PERIOD_S",
    "VALIDATION_HEALTH_FRESH_S",
    "VALIDATION_MOTOR_SAFE_MAX_US",
    "VALIDATION_ROTATION_TARGET_SAMPLES",
    "VALIDATION_SAMPLE_FRESH_S",
    "VALIDATION_SNAPSHOT_REQUIRED_FIELDS",
    "VALIDATION_STATIC_TARGET_SAMPLES",
    "VALIDATION_UI_STAGES",
    "signed_permutation_descriptor",
    "validation_history_artifacts",
    "validation_sample_from_snapshot",
    "validation_samples_from_csv",
]
