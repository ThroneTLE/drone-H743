#!/usr/bin/env python3
"""Ground-station panel for the drone-H743 Ai-WB2 transparent link."""

from __future__ import annotations

import csv
import json
import math
import os
import queue
import re
import socket
import statistics
import sys
import threading
import time
import tkinter as tk
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Callable, Sequence

try:
    from .flight_validation import (
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
    from .rom_dfu import (
        DfuCancelledError,
        FlashResult,
        FirmwareBuildError,
        FirmwareImageInfo,
        RomDfuError,
        build_firmware,
        firmware_image_staleness,
        flash_firmware,
        freeze_firmware_image,
        list_usb_dfu_ports,
        resolve_cubeprogrammer_cli,
        validate_firmware_image,
        wait_for_usb_dfu,
    )
    from .project_paths import (
        AIRFRAME_CALIBRATION_DIR,
        ATTITUDE_IDENT_DIR,
        FIRMWARE_UPDATE_DIR,
        FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
        PROJECT_ROOT,
        TELEMETRY_DIR,
        dated_directory,
        ensure_directory,
    )
except ImportError:  # Allows running as: python tools/drone_tcp_panel.py
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
        from tools.rom_dfu import (
            DfuCancelledError,
            FlashResult,
            FirmwareBuildError,
            FirmwareImageInfo,
            RomDfuError,
            build_firmware,
            firmware_image_staleness,
            flash_firmware,
            freeze_firmware_image,
            list_usb_dfu_ports,
            resolve_cubeprogrammer_cli,
            validate_firmware_image,
            wait_for_usb_dfu,
        )
        from tools.project_paths import (
            AIRFRAME_CALIBRATION_DIR,
            ATTITUDE_IDENT_DIR,
            FIRMWARE_UPDATE_DIR,
            FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
            PROJECT_ROOT,
            TELEMETRY_DIR,
            dated_directory,
            ensure_directory,
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
        from rom_dfu import (
            DfuCancelledError,
            FlashResult,
            FirmwareBuildError,
            FirmwareImageInfo,
            RomDfuError,
            build_firmware,
            firmware_image_staleness,
            flash_firmware,
            freeze_firmware_image,
            list_usb_dfu_ports,
            resolve_cubeprogrammer_cli,
            validate_firmware_image,
            wait_for_usb_dfu,
        )
        from project_paths import (
            AIRFRAME_CALIBRATION_DIR,
            ATTITUDE_IDENT_DIR,
            FIRMWARE_UPDATE_DIR,
            FLIGHT_ACCEPTANCE_CALIBRATION_DIR,
            PROJECT_ROOT,
            TELEMETRY_DIR,
            dated_directory,
            ensure_directory,
        )

try:
    from .imu_metrology import MetrologyStage, MetrologyStatus
    from .imucal_protocol import (
        EncodedV1Candidate,
        commit_candidate as imucal_commit_candidate,
        load_and_encode_v1_candidate,
        revert_candidate as imucal_revert_candidate,
        upload_and_apply as imucal_upload_and_apply,
        write_transaction_record as write_imucal_transaction_record,
    )
    from .imu_vibration_capture import CaptureLink
    from .v1_metrology_session import (
        CAPTURE_PLANS,
        V1Session,
        analyze_session,
        latest_session_manifest,
        load_session as load_v1_session,
        load_session_samples as load_v1_session_samples,
        new_session as new_v1_session,
        persist_capture as persist_v1_capture,
    )
except ImportError:
    try:
        from tools.imu_metrology import MetrologyStage, MetrologyStatus
        from tools.imucal_protocol import (
            EncodedV1Candidate,
            commit_candidate as imucal_commit_candidate,
            load_and_encode_v1_candidate,
            revert_candidate as imucal_revert_candidate,
            upload_and_apply as imucal_upload_and_apply,
            write_transaction_record as write_imucal_transaction_record,
        )
        from tools.imu_vibration_capture import CaptureLink
        from tools.v1_metrology_session import (
            CAPTURE_PLANS, V1Session, analyze_session, latest_session_manifest,
            load_session as load_v1_session,
            load_session_samples as load_v1_session_samples,
            new_session as new_v1_session,
            persist_capture as persist_v1_capture,
        )
    except ImportError:
        from imu_metrology import MetrologyStage, MetrologyStatus
        from imucal_protocol import (
            EncodedV1Candidate,
            commit_candidate as imucal_commit_candidate,
            load_and_encode_v1_candidate,
            revert_candidate as imucal_revert_candidate,
            upload_and_apply as imucal_upload_and_apply,
            write_transaction_record as write_imucal_transaction_record,
        )
        from imu_vibration_capture import CaptureLink
        from v1_metrology_session import (
            CAPTURE_PLANS, V1Session, analyze_session, latest_session_manifest,
            load_session as load_v1_session,
            load_session_samples as load_v1_session_samples,
            new_session as new_v1_session,
            persist_capture as persist_v1_capture,
        )


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6666
DEFAULT_MODULE_IP = "192.168.223.181"
DEFAULT_UDP_LOCAL_PORT = 6668
DEFAULT_UDP_MODULE_PORT = 7777
MAX_BARO_SAMPLES = 2000
MAX_GPS_TRACK_POINTS = 5000
MAX_IDENT_SAMPLES = 5000
BARO_STREAM_PERIOD_MS = 50
IMU_POLL_PERIOD_MS = 100
IMU_RENDER_PERIOD_MS = 50
RX_DRAIN_BATCH_SIZE = 200
RX_DRAIN_IDLE_MS = 100
RX_DRAIN_BUSY_MS = 10
LOG_FLUSH_PERIOD_MS = 100
MAX_PENDING_LOG_LINES = 500
VALIDATION_SAMPLE_FRESH_S = 1.5
VALIDATION_HEALTH_FRESH_S = 3.0
VALIDATION_MOTOR_SAFE_MAX_US = 1100
VALIDATION_STATIC_TARGET_SAMPLES = 50
VALIDATION_ROTATION_TARGET_SAMPLES = 30
VALIDATION_AUTOSAVE_PERIOD_S = 2.0
FIRMWARE_BOOT_COMMAND = "BOOT DFU CONFIRM"
FIRMWARE_BOOT_REPLY_TIMEOUT_MS = 5000
FIRMWARE_CDC_DISCONNECT_TIMEOUT_S = 8.0
FIRMWARE_RECOVERY_HINT = (
    "写入阶段取消或失败可能留下不完整应用，软件不会自动重试。"
    "若飞控没有重新出现，请用 BOOT0 进入 ROM DFU；"
    "仍无法连接时使用 ST-Link 恢复。"
)
VALIDATION_UI_STAGES = tuple(
    stage for stage, definition in STAGE_DEFINITIONS.items() if definition.required
)
VALIDATION_ALLOWED_COMMANDS = frozenset(
    {
        "PING",
        "CAPS?",
        "MODULES?",
        "STATUS?",
        "IMU?",
        "CONFIG?",
        "PARAM?",
        "PID?",
        "AIRFRAME?",
        "BARO?",
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
    }
)
CMD_REPLY_TIMEOUT_MS = 2500
# USB CDC 是纯命令/响应通道：飞控的周期性 VOFA 遥测只走 Ai-WB2 socket
# (App/Src/app_vofa.c)，USB 上只有命令回复镜像。空闲时链路必须由上位机主动
# 探活，否则 last_board_rx 永远不刷新，状态栏会把"没人问"误判成"掉线"。
LINK_KEEPALIVE_IDLE_S = 2.0
LINK_KEEPALIVE_MIN_INTERVAL_S = 1.5
LINK_STALE_S = 6.0

# 深色工程控制台配色。规则：全局只有一个强调色 accent，用于"当前该点的那一个
# 操作"；确认类用 amber、破坏类用 red，且都只做描边+微填充，不做大面积实心。
# 状态一律用指示灯圆点表达，不用彩色文字块，避免页面变成调色盘。
UI_PALETTE = {
    "canvas": "#171B22",
    "surface": "#1E232B",
    "panel": "#262C36",
    "raised": "#323945",
    "ink": "#EDF1F7",
    "ink_dim": "#C6CEDA",
    "muted": "#8D97A6",
    "border": "#39414E",
    "border_strong": "#4C5666",
    "navy": "#262C36",
    "accent": "#4DA3F5",
    "accent_hover": "#7BBEFF",
    "accent_ink": "#0A1A29",
    "accent_soft": "#28374A",
    "blue": "#4DA3F5",
    "blue_hover": "#7BBEFF",
    "blue_soft": "#28374A",
    "green": "#4ADE97",
    "green_soft": "#22352C",
    "amber": "#F2B441",
    "amber_soft": "#38311F",
    "amber_hover": "#FFC960",
    "red": "#FF8B82",
    "red_hover": "#FFA9A2",
    "red_soft": "#3B2729",
    "disabled": "#242A33",
    "disabled_ink": "#6B7482",
    "console": "#171B21",
}
UI_FONT = "Microsoft YaHei UI"
UI_MONO = "Consolas"
UI_SIZE = 10        # 正文
UI_SIZE_SM = 9      # 页签、eyebrow、表头
UI_SIZE_TITLE = 16
PROTO_COMPAT_FALLBACK_DELAY_MS = 400
PROTO_PROBE_SETTLE_MS = 900
SERIAL_ASCII_COMPAT_MODE = True
SERIAL_TX_DEBUG_ENABLED = True
PROTO_HEADER = b"$X"
PROTO_DIR_TO_FC = ord("<")
PROTO_DIR_FROM_FC = ord(">")
PROTO_REQ_PING = 0x1000
PROTO_REQ_STATUS = 0x1001
PROTO_REQ_CONFIG = 0x1002
PROTO_REQ_PARAMS = 0x1003
PROTO_REQ_PID = 0x1004
PROTO_REQ_BARO = 0x1005
PROTO_REQ_BARO_STREAM = 0x1006
PROTO_REQ_FLASH = 0x1007
PROTO_REQ_IMU = 0x1008
PROTO_REQ_MODULES = 0x1009
PROTO_REQ_CAPS = 0x100A
PROTO_REQ_SAVE = 0x100B
PROTO_REQ_LOAD = 0x100C
PROTO_REQ_DEFAULTS = 0x100D
PROTO_REQ_PARAM_SET = 0x100E
PROTO_REQ_PID_SET = 0x100F
PROTO_REQ_SERVO_MOVE = 0x1010
PROTO_REQ_SERVO_MOVE_ALL = 0x1011
PROTO_REQ_SERVO_ID = 0x1012
PROTO_REQ_SERVO_SETID = 0x1013
PROTO_REQ_SERVO_MODE = 0x1014
PROTO_REQ_SERVO_ENABLE = 0x1015
PROTO_REQ_SERVO_ACTION = 0x1016
PROTO_REQ_SERVO_RAW = 0x1017
PROTO_REQ_WIFI = 0x1018
PROTO_REQ_GPS = 0x1019
PROTO_REQ_MAG = 0x101A
PROTO_REQ_RTOS = 0x101B
PROTO_REQ_AIRFRAME = 0x101C
PROTO_REQ_IDENT = 0x101D
PROTO_REQ_IMU_FRAME = 0x101E
PROTO_REQ_IMU_CAL = 0x1020
PROTO_REQ_ACCEPTANCE = 0x1021
PROTO_MSG_CMD_LINE = 0x2000
PROTO_MSG_TEXT_LINE = 0x2001
PROTO_MSG_CMD_RX = 0x2100
PROTO_MSG_CMD_ACK = 0x2101
PROTO_MSG_CMD_ERR = 0x2102
PROTO_MSG_CMD_OK = 0x2103
PROTO_MSG_PONG = 0x2200
PROTO_MSG_HW_FLASH = 0x2201
PROTO_MSG_HW_BARO = 0x2202
PROTO_MSG_HW_IMU = 0x2203
PROTO_MSG_STATUS_FLASH = 0x2204
PROTO_MSG_STATUS_BARO = 0x2205
PROTO_MSG_STATUS_IMU = 0x2206
PROTO_MSG_UART_STATS = 0x2207
PROTO_MSG_CONFIG_SUMMARY = 0x2208
PROTO_MSG_CONFIG_SERVO = 0x2209
PROTO_MSG_PARAM_RECORD = 0x220A
PROTO_MSG_PID_RECORD = 0x220B
PROTO_MSG_FLASH_RECORD = 0x220C
PROTO_MSG_BARO_STATE = 0x220D
PROTO_MSG_BARO_DIAG = 0x220E
PROTO_MSG_BARO_RAW = 0x220F
PROTO_MSG_BARO_STREAM = 0x2210
PROTO_MSG_IMU_STATE = 0x2211
PROTO_MSG_IMU_SCALED = 0x2212
PROTO_MSG_MODULES_SUMMARY = 0x2213
PROTO_MSG_CAPS_RECORD = 0x2214
PROTO_MSG_READY = 0x2215
PROTO_MSG_SAVE_RESULT = 0x2216
PROTO_MSG_LOAD_RESULT = 0x2217
PROTO_MSG_DEFAULTS_RESULT = 0x2218
PROTO_MSG_SERVO_RESULT = 0x2219
PROTO_MSG_WIFI_RECORD = 0x221A
PROTO_MSG_GPS_RECORD = 0x221B
PROTO_MSG_MAG_RECORD = 0x221C
PROTO_MSG_RTOS_RECORD = 0x221D
PROTO_MSG_FLASH_BENCH = 0x221E
PROTO_MSG_AIRFRAME_RECORD = 0x221F

try:
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from matplotlib.figure import Figure

    HAS_MATPLOTLIB = True
    MATPLOTLIB_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on local optional package
    FigureCanvasTkAgg = None  # type: ignore[assignment]
    Figure = None  # type: ignore[assignment]
    HAS_MATPLOTLIB = False
    MATPLOTLIB_ERROR = str(exc)

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore

    HAS_PYSERIAL = True
    PYSERIAL_ERROR = ""
except Exception as exc:  # pragma: no cover - depends on local optional package
    serial = None  # type: ignore[assignment]
    HAS_PYSERIAL = False
    PYSERIAL_ERROR = str(exc)


MODULES = [
    ("FLASH", "GD25Q32 Flash", "STATUS?"),
    ("SPL06", "SPL06 气压计", "STATUS?"),
    ("ICM42688", "ICM42688 IMU", "STATUS?"),
    ("GPS", "M9N GPS", "GPS?"),
    ("MAG", "I2C1 Magnetometer", "MAG?"),
    ("UART1", "USART1 链路", "STATUS?"),
    ("WIFI", "Ai-WB2 WiFi", "WIFI?"),
]

MODULE_ALIASES = {
    "FLASH": "FLASH",
    "GD25Q32": "FLASH",
    "SPL06": "SPL06",
    "BARO": "SPL06",
    "ICM42688": "ICM42688",
    "IMU": "ICM42688",
    "GPS": "GPS",
    "GPS_USART2": "GPS",
    "M9N": "GPS",
    "UBX": "GPS",
    "MAG": "MAG",
    "MAG_I2C1": "MAG",
    "IST8310": "MAG",
    "HMC5883": "MAG",
    "QMC5883L": "MAG",
    "UART1": "UART1",
    "USART1": "UART1",
    "WIFI": "WIFI",
    "AIWB2": "WIFI",
}


def udp_payload_is_probably_text(data: bytes) -> bool:
    if not data:
        return False
    if data.startswith(PROTO_HEADER):
        return True
    if b"\x00" in data:
        return False

    allowed = 0
    for byte in data:
        if byte in (9, 10, 13) or 32 <= byte <= 126:
            allowed += 1
    return (allowed / len(data)) >= 0.95


def parse_kv(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z0-9_.-]+)=([^ \r\n]+)", line):
        result[match.group(1)] = match.group(2).rstrip(",")
    return result


def safe_int(value: str | None, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return int(value, 0)
    except ValueError:
        return default


def firmware_update_snapshot_gate(
    values: dict[str, str],
    *,
    connected: bool,
    serial_transport_selected: bool,
    sample_age_s: float,
) -> tuple[bool, str]:
    """Require a fresh disarmed, low-output snapshot before entering ROM DFU."""

    if not serial_transport_selected:
        return False, "请选择顶部 serial 通道；ROM DFU V0 只从 USB CDC 发起"
    if not connected:
        return False, "USB CDC 尚未连接"
    if (
        values.get("valid") != "1"
        or values.get("source") != "stabilizer_snapshot"
    ):
        return False, "尚无有效 stabilizer snapshot"
    if sample_age_s < 0.0 or sample_age_s > VALIDATION_SAMPLE_FRESH_S:
        return False, f"安全快照已过期（{sample_age_s:.1f}s）"
    try:
        armed = int(values["armed"], 0)
        motor_1 = int(values["m1"], 0)
        motor_2 = int(values["m2"], 0)
    except (KeyError, TypeError, ValueError):
        return False, "安全快照缺少 armed/m1/m2"
    if armed != 0:
        return False, f"飞控仍处于 armed={armed}，禁止进入 DFU"
    if motor_1 > VALIDATION_MOTOR_SAFE_MAX_US or motor_2 > VALIDATION_MOTOR_SAFE_MAX_US:
        return False, f"电机输出不安全：m1={motor_1} m2={motor_2}"
    return True, f"USB CDC ✓  snapshot ✓  armed=0  m1={motor_1} m2={motor_2}"


def v0_workflow_guidance(
    *,
    candidate_available: bool,
    candidate_applied: bool,
    verification_mode: bool,
    candidate_verified: bool,
    candidate_committed: bool,
    runtime_ready: bool,
) -> tuple[str, str]:
    """Explain V0 discovery versus post-apply verification without ambiguity."""

    if candidate_committed:
        return (
            "V0 已完成 · FLU 映射已写入参数 Flash",
            "当前 V0 结果已经持久化。下一步进入 02 · IMU V1；这仍不代表允许自由飞行。",
        )
    if candidate_verified:
        return (
            "阶段 3/3 · RAM 映射下的 6 步复验已经 PASS",
            "下一步：点击 C 写入 Flash；写入前继续保持拆桨、未解锁和安全输出。",
        )
    if verification_mode:
        return (
            "阶段 2/3 · 正在验证 RAM 中的 FLU 映射",
            "当前表格只看重新采集的 canonical FLU 样本；依次完成 6 个必要步骤，全部 PASS 后进入阶段 3。",
        )
    if candidate_applied:
        return (
            "阶段 2/3 · 候选已临时应用到 RAM，尚未写 Flash",
            "旧 FAIL 是映射前的发现证据，不会自动变成 PASS。下一步：点击 B 清空旧样本，并按新 FLU 坐标重新采集 6 步。",
        )
    if candidate_available:
        if runtime_ready:
            next_action = (
                "旧 FAIL 说明 legacy 轴与 FLU 不一致，它们用于推导候选，不是最终验收。"
                "下一步：完成两项物理确认，然后点击 A 临时应用到 RAM。"
            )
        else:
            next_action = (
                "旧 FAIL 说明 legacy 轴与 FLU 不一致，它们用于推导候选，不是最终验收。"
                "下一步：连接当前飞控；若加载的是历史会话，点击“继续已加载验收”；"
                "勾选拆桨/动力安全门并等待新鲜快照，再完成两项物理确认和 A。"
            )
        return "阶段 1/3 · 已发现 FLU 映射候选，尚未应用", next_action
    return (
        "阶段 1/3 · 采集姿态与正向旋转，发现坐标映射",
        "下一步：按步骤条依次完成必要动作。此阶段出现 FAIL 可以是坐标未映射的预期现象。",
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


def serial_device_identity_policy(identity: dict[str, object] | None) -> tuple[str, str]:
    """Classify a COM device for the application-CDC to ROM-DFU handoff."""

    if not identity:
        return "unknown", "没有该 COM 口的 USB 身份信息"
    combined = " ".join(
        str(identity.get(key) or "")
        for key in ("description", "hwid", "location", "serial_number")
    ).upper()
    rejected_tokens = (
        "STLINK", "ST-LINK", "CH340", "CH341", "CP210", "FTDI", "FT232",
        "FT4232", "BLUETOOTH", " BLE ",
    )
    matched = next((token for token in rejected_tokens if token in f" {combined} "), None)
    if matched is not None:
        return "rejected", f"检测到不允许的串口适配器：{matched.strip()}"
    vid = identity.get("vid")
    pid = identity.get("pid")
    if vid == 0x0483 and pid == 0x5740:
        return "allowed", "STM32 application USB CDC VID:PID=0483:5740"
    return "unknown", (
        f"未知 USB 串口身份 VID:PID="
        f"{int(vid):04X}:{int(pid):04X}" if isinstance(vid, int) and isinstance(pid, int)
        else "未知 USB 串口身份（无 VID/PID）"
    )


def serial_port_identity(port: object) -> dict[str, object]:
    """Copy the stable USB identity fields from a pyserial ListPortInfo."""

    return {
        "device": str(getattr(port, "device", "") or ""),
        "vid": getattr(port, "vid", None),
        "pid": getattr(port, "pid", None),
        "description": str(getattr(port, "description", "") or ""),
        "hwid": str(getattr(port, "hwid", "") or ""),
        "location": str(getattr(port, "location", "") or ""),
        "serial_number": str(getattr(port, "serial_number", "") or ""),
    }


def select_reenumerated_application_port(
    previous_port: str,
    previous_identity: dict[str, object] | None,
    candidates: Sequence[dict[str, object]],
) -> str | None:
    """Select only the same application CDC after ROM-DFU re-enumeration."""

    if not previous_identity:
        return None
    previous_vid = previous_identity.get("vid")
    previous_pid = previous_identity.get("pid")
    previous_serial = str(previous_identity.get("serial_number") or "").casefold()
    previous_location = str(previous_identity.get("location") or "").casefold()
    eligible = [
        row for row in candidates
        if serial_device_identity_policy(row)[0] == "allowed"
        and row.get("vid") == previous_vid
        and row.get("pid") == previous_pid
    ]
    if previous_serial:
        matches = [row for row in eligible if str(
            row.get("serial_number") or "").casefold() == previous_serial]
        if len(matches) == 1:
            return str(matches[0].get("device") or "") or None
    if previous_location:
        matches = [row for row in eligible if str(
            row.get("location") or "").casefold() == previous_location]
        if len(matches) == 1:
            return str(matches[0].get("device") or "") or None
    matches = [row for row in eligible if str(
        row.get("device") or "").casefold() == previous_port.casefold()]
    if len(matches) == 1:
        return str(matches[0].get("device") or "") or None
    return None


def wait_for_application_serial(
    previous_port: str,
    previous_identity: dict[str, object] | None,
    *,
    timeout_s: float = 15.0,
    poll_interval_s: float = 0.25,
    cancel_event: threading.Event | None = None,
    enumerate_ports: Callable[[], Sequence[object]] | None = None,
    on_log: Callable[[str], None] | None = None,
) -> str | None:
    """Wait for the same STM32 application CDC and return its COM device."""

    if timeout_s <= 0.0:
        return None
    cancel = cancel_event or threading.Event()
    if enumerate_ports is None:
        if not HAS_PYSERIAL or serial is None:
            return None
        enumerate_ports = lambda: tuple(  # type: ignore[union-attr]
            serial.tools.list_ports.comports())
    deadline = time.monotonic() + timeout_s
    if on_log is not None:
        on_log("等待飞控 application CDC 重新枚举…")
    while time.monotonic() < deadline:
        if cancel.is_set():
            return None
        try:
            identities = [
                row if isinstance(row, dict) else serial_port_identity(row)
                for row in enumerate_ports()
            ]
        except Exception as exc:
            if on_log is not None:
                on_log(f"枚举 application CDC 暂时失败：{exc}")
            identities = []
        matched = select_reenumerated_application_port(
            previous_port, previous_identity, identities)
        if matched is not None:
            if on_log is not None:
                on_log(f"已匹配原飞控 application CDC：{matched}")
            return matched
        cancel.wait(max(0.05, poll_interval_s))
    if on_log is not None:
        on_log("application CDC 自动重连等待超时")
    return None


def first_value(values: dict[str, str], *names: str) -> str:
    for name in names:
        if name in values:
            return values[name]
    return "-"


def first_float(values: dict[str, str], *names: str) -> float | None:
    for name in names:
        if name not in values:
            continue
        try:
            return float(values[name])
        except ValueError:
            return None
    return None


VALIDATION_SNAPSHOT_REQUIRED_FIELDS = (
    "valid", "source", "frame", "units", "contract", "migration", "ts_ms", "seq",
    "bias", "armed", "m1", "m2", "ax_mg", "ay_mg", "az_mg",
    "gx_mdps", "gy_mdps", "gz_mdps",
)


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


def airframe_record_from_line(line: str) -> dict[str, str | float] | None:
    if not line.startswith("AIRFRAME "):
        return None
    values = parse_kv(line)
    record: dict[str, str | float] = {"line": line}
    for key, value in values.items():
        parsed = first_float(values, key)
        record[key] = parsed if parsed is not None else value
    return record


def ident_record_from_line(line: str) -> dict[str, str | float | int] | None:
    if not line.startswith("IDENT sample "):
        return None
    values = parse_kv(line)
    record: dict[str, str | float | int] = {"line": line}
    for key, value in values.items():
        record[key] = value
    for key in ("id", "seq", "t_ms", "alpha_us", "beta_us", "rc_arm", "throttle_us"):
        if key in values:
            record[key] = safe_int(values[key])
    if "roll_mdeg" in values:
        record["roll"] = safe_int(values["roll_mdeg"]) / 1000.0
    elif "roll" in values:
        parsed = first_float(values, "roll")
        if parsed is not None:
            record["roll"] = parsed
    if "pitch_mdeg" in values:
        record["pitch"] = safe_int(values["pitch_mdeg"]) / 1000.0
    elif "pitch" in values:
        parsed = first_float(values, "pitch")
        if parsed is not None:
            record["pitch"] = parsed
    if "gx_cdps" in values:
        record["gx"] = safe_int(values["gx_cdps"]) / 100.0
    elif "gx" in values:
        parsed = first_float(values, "gx")
        if parsed is not None:
            record["gx"] = parsed
    if "gy_cdps" in values:
        record["gy"] = safe_int(values["gy_cdps"]) / 100.0
    elif "gy" in values:
        parsed = first_float(values, "gy")
        if parsed is not None:
            record["gy"] = parsed
    return record


def fit_ident_step(samples: list[dict[str, str | float | int]], axis: str) -> dict[str, float] | None:
    if len(samples) < 8:
        return None
    value_key = "roll" if axis == "roll" else "pitch"
    input_key = "alpha_us" if axis == "roll" else "beta_us"
    rows = [
        row for row in samples
        if isinstance(row.get("t_ms"), int)
        and isinstance(row.get(value_key), float)
        and isinstance(row.get(input_key), int)
    ]
    if len(rows) < 8:
        return None
    t0 = float(rows[0]["t_ms"]) / 1000.0
    times = [(float(row["t_ms"]) / 1000.0) - t0 for row in rows]
    outputs = [float(row[value_key]) for row in rows]
    inputs = [float(row[input_key]) for row in rows]
    baseline_n = max(1, min(5, len(outputs) // 5))
    baseline_y = sum(outputs[:baseline_n]) / baseline_n
    baseline_u = sum(inputs[:baseline_n]) / baseline_n
    final_n = max(1, min(8, len(outputs) // 4))
    final_y = sum(outputs[-final_n:]) / final_n
    final_u = sum(inputs[-final_n:]) / final_n
    du = final_u - baseline_u
    dy = final_y - baseline_y
    if abs(du) < 1.0 or abs(dy) < 0.01:
        return None
    target_10 = baseline_y + (0.10 * dy)
    target_63 = baseline_y + (0.632 * dy)

    def crossing(target: float) -> float | None:
        for t, y in zip(times, outputs):
            if (dy >= 0.0 and y >= target) or (dy < 0.0 and y <= target):
                return t
        return None

    t10 = crossing(target_10)
    t63 = crossing(target_63)
    delay = t10 if t10 is not None else 0.0
    tau = max(0.02, (t63 - delay) if t63 is not None else (times[-1] / 3.0))
    gain = dy / du
    kp = max(0.0, min(10.0, 0.35 / max(abs(gain), 0.001)))
    kd = max(0.0, min(5.0, kp * tau * 0.25))
    return {"K": gain, "tau": tau, "L": delay, "kp": kp, "kd": kd}


def normalize_module_key(key: str) -> str | None:
    return MODULE_ALIASES.get(key.upper())


def proto_crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_proto_frame(direction: int, function: int, payload: bytes) -> bytes:
    body = bytearray()
    body.append(0)
    body.append(function & 0xFF)
    body.append((function >> 8) & 0xFF)
    body.append(len(payload) & 0xFF)
    body.append((len(payload) >> 8) & 0xFF)
    body.extend(payload)
    crc = proto_crc8_dvb_s2(bytes(body))
    return PROTO_HEADER + bytes([direction]) + bytes(body) + bytes([crc])


class TransportBase(ABC):
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        self.rx_queue = rx_queue

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def start(self, *args, **kwargs) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        raise NotImplementedError

    def send_line(self, line: str) -> bool:
        return self.send_frame(
            PROTO_MSG_CMD_LINE,
            line.rstrip("\r\n").encode("utf-8"),
        )

    def cancel_pending_sends(self) -> None:
        """Invalidate queued writes before entering a read-only V0 session."""

        return

    def _consume_buffer(self, buffer: bytearray) -> None:
        while buffer:
            if len(buffer) >= 9 and buffer[0:2] == PROTO_HEADER and buffer[2] in (PROTO_DIR_TO_FC, PROTO_DIR_FROM_FC):
                payload_length = buffer[6] | (buffer[7] << 8)
                frame_length = 9 + payload_length
                if len(buffer) < frame_length:
                    break

                frame = bytes(buffer[:frame_length])
                body = frame[3:-1]
                if proto_crc8_dvb_s2(body) == frame[-1]:
                    function = frame[4] | (frame[5] << 8)
                    payload = frame[8:-1]
                    del buffer[:frame_length]

                    if frame[2] == PROTO_DIR_FROM_FC:
                        text = payload.decode("utf-8", errors="replace").rstrip("\r\n")
                        self.rx_queue.put(("proto", function, text))
                    else:
                        shown = payload.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                        self.rx_queue.put(f"RXRAW fn=0x{function:04X} len={len(payload)} data={shown}")
                    continue

                del buffer[0]
                continue

            newline_index = buffer.find(b"\n")
            frame_index = buffer.find(PROTO_HEADER)
            if newline_index != -1 and (frame_index == -1 or newline_index < frame_index):
                line = bytes(buffer[:newline_index]).rstrip(b"\r")
                del buffer[: newline_index + 1]
                self.rx_queue.put(line.decode("utf-8", errors="replace"))
                continue

            if frame_index > 0:
                raw = bytes(buffer[:frame_index])
                del buffer[:frame_index]
                shown = raw.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                self.rx_queue.put(f"RXRAW len={len(raw)} data={shown}")
                continue

            break


class TcpTransport(TransportBase):
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        super().__init__(rx_queue)
        self.sock: socket.socket | None = None
        self.client: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self._sender_thread: threading.Thread | None = None
        self._send_queue: "queue.Queue[tuple[int, bytes] | None]" = queue.Queue()
        self._send_generation = 0
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.client is not None

    def start(self, host: str, port: int) -> None:
        self.stop()
        self._send_queue = queue.Queue()
        with self.lock:
            self._send_generation += 1
        self.stop_event.clear()
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        self.thread = threading.Thread(target=self._run, args=(host, port), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self._send_queue.put_nowait(None)
        except queue.Full:
            pass
        with self.lock:
            sockets = [self.client, self.sock]
            self.client = None
            self.sock = None
        for item in sockets:
            if item is not None:
                try:
                    item.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    item.close()
                except OSError:
                    pass

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        frame = build_proto_frame(PROTO_DIR_TO_FC, function, payload)
        with self.lock:
            if self.client is None:
                return False
            generation = self._send_generation
        try:
            self._send_queue.put_nowait((generation, frame))
            return True
        except queue.Full:
            return False

    def cancel_pending_sends(self) -> None:
        with self.lock:
            self._send_generation += 1
        try:
            while True:
                self._send_queue.get_nowait()
        except queue.Empty:
            return

    def _sender_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                item = self._send_queue.get(timeout=0.3)
            except queue.Empty:
                continue
            if item is None:
                break
            generation, frame = item
            with self.lock:
                if generation != self._send_generation:
                    continue
                client = self.client
                if client is None:
                    continue
                try:
                    client.sendall(frame)
                except OSError as exc:
                    self.rx_queue.put(f"[上位机] 发送失败: {exc}")

    def _run(self, host: str, port: int) -> None:
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(1)
            server.settimeout(0.5)
            with self.lock:
                self.sock = server
            self.rx_queue.put(f"[上位机] 正在监听 {host}:{port}")
        except OSError as exc:
            self.rx_queue.put(f"[上位机] 监听失败: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with self.lock:
                if self.client is not None:
                    try:
                        self.client.close()
                    except OSError:
                        pass
                self.client = client
            self.rx_queue.put(f"[上位机] 板子已连接: {addr[0]}:{addr[1]}")
            self._read_client(client)

        self.rx_queue.put("[上位机] TCP 服务已停止")

    def _read_client(self, client: socket.socket) -> None:
        client.settimeout(0.5)
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                data = client.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            buffer += data
            self._consume_buffer(buffer)
        with self.lock:
            if self.client is client:
                self.client = None
        try:
            client.close()
        except OSError:
            pass
        self.rx_queue.put("[上位机] 板子已断开")


class UdpTransport(TransportBase):
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        super().__init__(rx_queue)
        self.sock: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self.remote: tuple[str, int] | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.sock is not None and self.remote is not None

    def start(self, bind_ip: str, local_port: int, module_ip: str, module_port: int) -> None:
        self.stop()
        self.stop_event.clear()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((bind_ip, local_port))
            sock.settimeout(0.5)
        except OSError as exc:
            self.rx_queue.put(f"[host] UDP open failed: {exc}")
            return

        with self.lock:
            self.sock = sock
            self.remote = (module_ip, module_port)
        self.rx_queue.put(f"[host] UDP ready local={bind_ip}:{local_port} module={module_ip}:{module_port}")
        self.thread = threading.Thread(target=self._read_loop, args=(sock,), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        with self.lock:
            sock = self.sock
            self.sock = None
            self.remote = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        del function
        try:
            text = payload.decode("utf-8") if payload else ""
        except UnicodeDecodeError:
            return False
        return self.send_line(text)

    def send_line(self, line: str) -> bool:
        data = (line.rstrip("\r\n") + "\r\n").encode("utf-8")
        with self.lock:
            sock = self.sock
            remote = self.remote
        if sock is None or remote is None:
            return False
        try:
            sock.sendto(data, remote)
            return True
        except OSError as exc:
            self.rx_queue.put(f"[host] UDP send failed: {exc}")
            return False

    def _read_loop(self, sock: socket.socket) -> None:
        while not self.stop_event.is_set():
            try:
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not udp_payload_is_probably_text(data):
                self.rx_queue.put(("udp_raw", addr[0], addr[1], len(data)))
                continue
            buffer = bytearray(data)
            self._consume_buffer(buffer)
            if buffer:
                shown = bytes(buffer).decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                self.rx_queue.put(f"RXRAW len={len(buffer)} data={shown}")
        with self.lock:
            if self.sock is sock:
                self.sock = None
                self.remote = None
        self.rx_queue.put("[host] UDP stopped")


class SerialTransport(TransportBase):
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        super().__init__(rx_queue)
        self.port: "serial.Serial | None" = None
        self.thread: threading.Thread | None = None
        self._sender_thread: threading.Thread | None = None
        self._send_queue: "queue.Queue[tuple[int, bytes] | None]" = queue.Queue()
        self._send_generation = 0
        self._connection_generation = 0
        self._active_port: str | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        with self.lock:
            return self.port is not None and bool(self.port.is_open)

    @property
    def active_port(self) -> str | None:
        with self.lock:
            return self._active_port

    @property
    def connection_generation(self) -> int:
        with self.lock:
            return self._connection_generation

    def start(self, port_name: str, baudrate: int) -> None:
        if not HAS_PYSERIAL or serial is None:
            self.rx_queue.put(f"[上位机] 串口模式不可用: {PYSERIAL_ERROR or '未安装 pyserial'}")
            return
        self.stop()
        self._send_queue = queue.Queue()
        with self.lock:
            self._send_generation += 1
        self.stop_event.clear()
        try:
            opened = serial.Serial(port_name, baudrate=baudrate, timeout=0.2, write_timeout=0.5)
        except Exception as exc:
            self.rx_queue.put(f"[上位机] 打开串口失败: {exc}")
            return
        with self.lock:
            self.port = opened
            self._active_port = str(port_name)
            self._connection_generation += 1
        self.rx_queue.put(f"[上位机] 串口已连接: {port_name} @ {baudrate}")
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()
        self.thread = threading.Thread(target=self._read_loop, args=(opened,), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        try:
            self._send_queue.put_nowait(None)
        except queue.Full:
            pass
        with self.lock:
            port = self.port
            self.port = None
            self._active_port = None
            self._connection_generation += 1
        if port is not None:
            self._safe_close(port)

    @staticmethod
    def _safe_close(port) -> None:
        try:
            if hasattr(port, 'cancel_read'):
                port.cancel_read()
        except Exception:
            pass
        try:
            if hasattr(port, 'cancel_write'):
                port.cancel_write()
        except Exception:
            pass
        try:
            port.close()
        except Exception:
            pass

    def send_frame(self, function: int, payload: bytes = b"") -> bool:
        if SERIAL_ASCII_COMPAT_MODE:
            try:
                text = payload.decode("utf-8") if payload else ""
            except UnicodeDecodeError:
                text = ""
            if text:
                return self.send_line(text)
        frame = build_proto_frame(PROTO_DIR_TO_FC, function, payload)
        with self.lock:
            if self.port is None or not self.port.is_open:
                return False
            generation = self._send_generation
        try:
            self._send_queue.put_nowait((generation, frame))
            return True
        except queue.Full:
            return False

    def send_line(self, line: str) -> bool:
        data = (line.rstrip("\r\n") + "\r\n").encode("utf-8")
        with self.lock:
            if self.port is None or not self.port.is_open:
                return False
            generation = self._send_generation
        try:
            self._send_queue.put_nowait((generation, data))
            return True
        except queue.Full:
            return False

    def cancel_pending_sends(self) -> None:
        with self.lock:
            self._send_generation += 1
        try:
            while True:
                self._send_queue.get_nowait()
        except queue.Empty:
            return

    def _sender_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                item = self._send_queue.get(timeout=0.3)
            except queue.Empty:
                continue
            if item is None:
                break
            generation, frame = item
            with self.lock:
                if generation != self._send_generation:
                    continue
                port = self.port
                if port is None or not port.is_open:
                    continue
                try:
                    written = port.write(frame)
                    port.flush()
                    if SERIAL_TX_DEBUG_ENABLED:
                        shown = frame.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                        self.rx_queue.put(f"[host] serial tx bytes={written} data={shown}")
                except Exception as exc:
                    self.rx_queue.put(f"[上位机] 串口发送失败: {exc}")

    def _read_loop(self, port: "serial.Serial") -> None:
        buffer = bytearray()
        while not self.stop_event.is_set():
            try:
                if not port.is_open:
                    break
                waiting = port.in_waiting
                chunk = port.read(waiting or 1)
            except Exception:
                break
            if not chunk:
                continue
            buffer += chunk
            self._consume_buffer(buffer)
        with self.lock:
            was_current = self.port is port
            if self.port is port:
                self.port = None
                self._active_port = None
                self._connection_generation += 1
        if was_current:
            self.rx_queue.put("[上位机] 串口已断开")


class VerticalScrolledFrame(ttk.Frame):
    """A width-following vertical viewport for long workflow pages."""

    def __init__(self, parent: tk.Misc) -> None:
        super().__init__(parent, style="Page.TFrame")
        self.canvas = tk.Canvas(
            self, background=UI_PALETTE["surface"], highlightthickness=0, borderwidth=0)
        self.scrollbar = ttk.Scrollbar(
            self, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.content = ttk.Frame(
            self.canvas, padding=(14, 14, 10, 18), style="Page.TFrame")
        self._window = self.canvas.create_window(
            (0, 0), window=self.content, anchor=tk.NW)
        self.content.bind("<Configure>", self._sync_scroll_region)
        self.content.bind("<Enter>", self._bind_wheel)
        self.content.bind("<Leave>", self._unbind_wheel)
        self.canvas.bind("<Configure>", self._sync_width)
        self.canvas.bind("<Enter>", self._bind_wheel)
        self.canvas.bind("<Leave>", self._unbind_wheel)

    def _sync_scroll_region(self, _event: tk.Event | None = None) -> None:
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _sync_width(self, event: tk.Event) -> None:
        self.canvas.itemconfigure(self._window, width=event.width)

    def _bind_wheel(self, _event: tk.Event) -> None:
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _unbind_wheel(self, _event: tk.Event) -> None:
        self.canvas.unbind_all("<MouseWheel>")

    def _on_mousewheel(self, event: tk.Event) -> None:
        delta = int(getattr(event, "delta", 0))
        if delta != 0:
            self.canvas.yview_scroll(-1 if delta > 0 else 1, "units")


def enable_hidpi_awareness() -> float:
    """声明 per-monitor DPI 感知，返回系统 DPI 缩放比（1.0 = 96 DPI）。

    Python 进程默认不是 DPI-aware，Windows 会对整个窗口做位图拉伸：在 2K/4K
    且系统缩放 >100% 的机器上，文字会明显发虚。必须在创建 Tk 根窗口之前调用，
    之后 Tk 拿到的是真实像素，再由 _configure_compact_scaling 按真实 DPI 放大字号。
    """
    if sys.platform != "win32":
        return 1.0
    import ctypes

    try:
        # PROCESS_PER_MONITOR_DPI_AWARE = 2
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except Exception:  # noqa: BLE001 - 老系统没有 shcore，退回进程级感知
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:  # noqa: BLE001
            return 1.0
    try:
        dc = ctypes.windll.user32.GetDC(0)
        dpi = ctypes.windll.gdi32.GetDeviceCaps(dc, 88)  # LOGPIXELSX
        ctypes.windll.user32.ReleaseDC(0, dc)
        return max(1.0, dpi / 96.0)
    except Exception:  # noqa: BLE001
        return 1.0


class DronePanel(tk.Tk):
    def __init__(self) -> None:
        # 必须早于 super().__init__()：Tk 根窗口一旦创建，DPI 感知就无法再改。
        self.ui_dpi_scale = enable_hidpi_awareness()
        super().__init__()
        self._configure_compact_scaling()
        self.title("drone-H743 地面站")
        width = min(int(1500 * self.ui_dpi_scale), self.winfo_screenwidth() - 80)
        height = min(int(900 * self.ui_dpi_scale), self.winfo_screenheight() - 120)
        self.geometry(f"{width}x{height}")
        self.minsize(1080, 700)

        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.tcp_transport = TcpTransport(self.rx_queue)
        self.udp_transport = UdpTransport(self.rx_queue)
        self.serial_transport = SerialTransport(self.rx_queue)
        self.transport: TransportBase = self.tcp_transport
        self.last_board_rx = 0.0
        self.last_reply_rx = 0.0
        self.last_keepalive_tx = 0.0
        self.selected_module = "FLASH"
        self.baro_capture_enabled = tk.BooleanVar(value=True)
        self.baro_buffer: list[dict[str, float | str]] = []
        self._baro_dirty = False
        self._last_baro_plot_ns = 0
        self.gps_track: list[dict[str, float | str | int]] = []
        self.gps_origin_lat: float | None = None
        self.gps_origin_lon: float | None = None
        self.gps_last_plot_ns = 0
        self.imu_poll_enabled = tk.BooleanVar(value=False)
        self.imu_last_poll = 0.0
        self.imu_last_sample_time = 0.0
        self.imu_last_count = -1
        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0
        self.imu_yaw_deg = 0.0
        self._imu_dirty = False
        self._last_imu_draw_ns = 0
        self._pending_log_lines: list[str] = []
        self.validation_session_active = False
        self.validation_poll_requested = False
        self.validation_active_stage: ValidationStage | None = None
        self.validation_samples: dict[ValidationStage, list[ImuSample]] = {
            stage: [] for stage in STAGE_DEFINITIONS
        }
        self.validation_stage_results: dict[ValidationStage, object] = {}
        self.validation_unsupported_stages: set[ValidationStage] = set()
        self.validation_skipped_stages: set[ValidationStage] = set()
        self.validation_finished_stages: set[ValidationStage] = set()
        self.validation_raw_rows: list[dict[str, object]] = []
        self.validation_latest_values: dict[str, str] = {}
        self.validation_latest_host_time = 0.0
        self.validation_latest_sequence: int | None = None
        self.validation_latest_transport_generation: int | None = None
        self.validation_stage_last_sequence: int | None = None
        self.validation_report_paths: tuple[Path, Path, Path] | None = None
        self.validation_session_path: Path | None = None
        self.validation_session_created_at: str | None = None
        self.validation_loaded_report_path: Path | None = None
        self.validation_loaded_history = False
        self.validation_last_autosave_monotonic = 0.0
        self.validation_candidate_matrix: tuple[tuple[int, int, int], ...] | None = None
        self.validation_candidate_descriptor = ""
        self.validation_candidate_confidence = 0.0
        self.validation_candidate_applied = False
        self.validation_candidate_verified = False
        self.validation_candidate_committed = False
        self.validation_verification_mode = False
        self.validation_orientation_pending = ""
        self.validation_orientation_poll_count = 0
        self.validation_orientation_values: dict[str, str] = {}
        self.validation_authorized_orientation_commands: set[str] = set()
        self.validation_candidate_source_path: Path | None = None
        self.validation_orientation_audit_path: Path | None = None
        self._last_validation_readiness_ns = 0
        self.params: dict[str, dict[str, str | bool]] = {}
        self.param_iids: dict[str, str] = {}
        self.param_names_by_iid: dict[str, str] = {}
        self.servo_widgets: list[dict[str, tk.Variable]] = []
        self.structured_protocol_supported: bool | None = None
        self.ident_samples: list[dict[str, str | float | int]] = []
        self.ident_csv_file = None
        self.ident_csv_writer: csv.DictWriter | None = None
        self.ident_current_path: Path | None = None
        self.ident_current_meta_path: Path | None = None
        self._last_ident_flush_ns = 0
        self._last_ident_plot_ns = 0
        self.ident_last_fit: dict[str, float] | None = None
        self.ident_current_command = ""
        self.airframe_info: dict[str, str | float] = {}
        self.udp_raw_hidden_count = 0
        self.udp_raw_last_note = ""

        self.link_var = tk.StringVar(value="未连接")
        self.show_log_var = tk.BooleanVar(value=False)
        self.last_cmd_var = tk.StringVar(value="-")
        self.config_summary_var = tk.StringVar(value="尚未读取配置")
        self.baro_count_var = tk.StringVar(value="暂存样本: 0")
        self.plot_var = tk.StringVar(value="pressure")
        self.gps_count_var = tk.StringVar(value="轨迹点: 0")
        self.gps_status_var = tk.StringVar(value="等待 GPS")
        self.transport_var = tk.StringVar(value="tcp")
        self.udp_bind_var = tk.StringVar(value=DEFAULT_HOST)
        self.udp_local_port_var = tk.IntVar(value=DEFAULT_UDP_LOCAL_PORT)
        self.udp_module_ip_var = tk.StringVar(value=DEFAULT_MODULE_IP)
        self.udp_module_port_var = tk.IntVar(value=DEFAULT_UDP_MODULE_PORT)
        self._serial_port_map: dict[str, str] = {}
        self._serial_port_identity: dict[str, dict[str, object]] = {}
        self.serial_port_var = tk.StringVar(value=self._default_serial_port())
        self.serial_baud_var = tk.IntVar(value=115200)
        self.ident_axis_var = tk.StringVar(value="roll")
        self.ident_mode_var = tk.StringVar(value="STEP")
        self.ident_pulse_var = tk.IntVar(value=20)
        self.ident_duration_var = tk.IntVar(value=3000)
        self.ident_hold_var = tk.IntVar(value=800)
        self.ident_repeat_var = tk.IntVar(value=2)
        self.ident_bit_var = tk.IntVar(value=250)
        self.ident_seed_var = tk.IntVar(value=1)
        self.ident_alpha_center_var = tk.IntVar(value=1500)
        self.ident_beta_center_var = tk.IntVar(value=1500)
        self.ident_status_var = tk.StringVar(value="idle")
        self.ident_sample_count_var = tk.StringVar(value="samples=0")
        self.ident_reason_var = tk.StringVar(value="-")
        self.ident_current_var = tk.StringVar(value="-")
        self.ident_fit_var = tk.StringVar(value="no fit")
        self.ident_airframe_var = tk.StringVar(value="AIRFRAME: not loaded")
        self.ident_save_dir = dated_directory(ATTITUDE_IDENT_DIR)
        self.ident_output_dir_var = tk.StringVar(value=f"save dir: {self.ident_save_dir}")
        self.ident_last_file_var = tk.StringVar(value="last file: none")
        self.ident_link_var = tk.StringVar(value="UDP text: waiting")
        self.ident_command_preview_var = tk.StringVar(value="")
        self.ident_mode_hint_var = tk.StringVar(value="")
        self.ident_safety_hint_var = tk.StringVar(
            value="建议第一次：STEP，小幅 20us，3s；确认方向后再加到 30-40us。"
        )
        self.ident_field_rows: dict[str, tuple[ttk.Label, ttk.Widget, ttk.Label]] = {}

        first_stage = VALIDATION_UI_STAGES[0]
        self.validation_stage_var = tk.StringVar(value=first_stage.value)
        self.validation_session_var = tk.StringVar(value="NOT_RUN · 尚未开始只读验收")
        self.validation_source_var = tk.StringVar(value="数据源：等待带 provenance 的实时快照")
        self.validation_instruction_var = tk.StringVar(
            value=STAGE_DEFINITIONS[first_stage].prompt_zh
        )
        self.validation_sample_var = tk.StringVar(value="当前步骤样本：0")
        self.validation_live_var = tk.StringVar(value="尚无有效样本")
        self.validation_safety_var = tk.StringVar(value="安全门：未确认")
        self.validation_report_var = tk.StringVar(value="报告：尚未生成")
        self.validation_candidate_var = tk.StringVar(
            value="轴映射候选：至少完成机头朝上/下、左/右侧朝上和水平静止后生成"
        )
        self.validation_firmware_hash_var = tk.StringVar(value="unknown（可选，不阻止验收）")
        # 就绪条件只存"值"，状态由指示灯圆点表达（见 _validation_set_gate）。
        self.validation_connection_gate_var = tk.StringVar(value="等待连接")
        self.validation_snapshot_gate_var = tk.StringVar(value="等待快照")
        self.validation_bias_gate_var = tk.StringVar(value="等待静止")
        self.validation_output_gate_var = tk.StringVar(value="等待快照")
        # 采样链健康：固件的 "IMU health" 行没有 seq，不走快照合并，单独缓存。
        self.validation_health_gate_var = tk.StringVar(value="等待固件上报")
        self.validation_imu_health: dict[str, str] = {}
        self.validation_imu_health_time = 0.0
        self.validation_gate_dots: dict[str, ttk.Label] = {}
        self.validation_step_dots: list[ttk.Label] = []
        self.validation_next_action_var = tk.StringVar(
            value="下一步：恢复已有历史，或勾选安全门后点击“新建验收”"
        )
        self.validation_props_removed_var = tk.BooleanVar(value=False)
        self.validation_power_safe_var = tk.BooleanVar(value=False)
        self.validation_axis_labels_confirmed_var = tk.BooleanVar(value=False)
        self.validation_mapping_confirmed_var = tk.BooleanVar(value=False)
        # 飞机回报的事实与本地推导的候选必须分开显示：候选摘要每 0.5s 重算一次，
        # 事件回执只在真的收到 IMUFRAME 回包时才变，否则提示会被刷新覆盖掉。
        self.validation_orientation_target_var = tk.StringVar(
            value="active=-  persisted=-  dirty=-  event=-"
        )
        self.validation_orientation_event_var = tk.StringVar(
            value="尚未与飞机交互；点击“读取飞机映射状态”可主动查询"
        )
        self.validation_orientation_var = tk.StringVar(
            value="应用状态：未应用；先由样本形成合法候选"
        )
        self.validation_phase_var = tk.StringVar(value="阶段 1/3 · 尚未开始坐标发现")
        self.validation_phase_action_var = tk.StringVar(
            value="下一步：连接飞控、确认拆桨和动力安全，然后开始只读采集。")
        self.validation_history_var = tk.StringVar(value="尚未扫描历史验收")
        self.validation_history_paths: dict[str, Path] = {}

        try:
            default_cubeprogrammer = str(resolve_cubeprogrammer_cli())
        except RomDfuError:
            default_cubeprogrammer = ""
        self.firmware_image_var = tk.StringVar(
            value=str(PROJECT_ROOT / "build" / "Debug" / "drone-H743.elf")
        )
        self.firmware_cli_var = tk.StringVar(value=default_cubeprogrammer)
        self.firmware_status_var = tk.StringVar(
            value="待机：连接飞控 USB CDC，安全快照通过后即可一键升级"
        )
        self.firmware_safety_var = tk.StringVar(value="安全门：等待 USB CDC 与实时快照")
        self.firmware_image_info_var = tk.StringVar(value="固件镜像：尚未验证")
        self.firmware_unknown_usb_override_var = tk.BooleanVar(value=False)
        self.firmware_progress_var = tk.DoubleVar(value=0.0)
        self.firmware_update_pending = False
        self.firmware_update_running = False
        self.firmware_programming = False
        self.firmware_building = False
        self.firmware_rebuild_var = tk.BooleanVar(value=True)
        self.firmware_staleness_var = tk.StringVar(
            value="烧录前会先执行一次编译，产物即为将要烧录的镜像。"
        )
        self.firmware_worker: threading.Thread | None = None
        self.firmware_cancel_event = threading.Event()
        self.firmware_event_queue: "queue.Queue[tuple[str, int, object]]" = queue.Queue()
        self.firmware_authorized_commands: set[str] = set()
        self.firmware_attempt_id = 0
        self.firmware_log_path: Path | None = None
        self.firmware_log_write_warning_sent = False
        self.firmware_target_abort_reason = ""
        self.firmware_reconnect_port = ""
        self.firmware_reconnect_identity: dict[str, object] | None = None
        self.firmware_reconnect_baud = 115200

        self.v1_session: V1Session | None = None
        self.v1_manifest_path: Path | None = None
        self.v1_worker: threading.Thread | None = None
        self.v1_finish_event = threading.Event()
        self.v1_cancel_event = threading.Event()
        self.v1_event_queue: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.v1_stage_by_label = {plan.label: plan for plan in CAPTURE_PLANS}
        self.v1_stage_var = tk.StringVar(value=CAPTURE_PLANS[0].label)
        self.v1_temperature_platform_var = tk.StringVar(value="room")
        self.v1_status_var = tk.StringVar(value="尚未开始：连接飞控并通过安全门，然后新建 V1 会话")
        self.v1_counts_var = tk.StringVar(value="尚无 V1 会话")
        self.v1_analysis_var = tk.StringVar(value="accel=NOT_RUN · gyro static=NOT_RUN · gyro +360=NOT_RUN · temperature=NOT_RUN")
        self.v1_analysis_summary = None
        self.v1_encoded_candidate: EncodedV1Candidate | None = None
        self.v1_candidate_applied = False
        self.v2_lease_id = 0
        self.v2_active = False
        self.v2_status_var = tk.StringVar(value="V2A 未启动：要求 V0 已保存、V1 室温基础参数有效、拆桨")
        self.v2_live_var = tk.StringVar(value="尚无 V2A 安全快照")
        self.v2_stage_var = tk.StringVar(value="rc_center")
        self.v2_tick_count = 0
        self.v2_session_dir: Path | None = None

        self.module_state: dict[str, dict[str, tk.StringVar]] = {}
        self.baro_vars: dict[str, tk.StringVar] = {}
        self.imu_vars: dict[str, tk.StringVar] = {}
        self.gps_vars: dict[str, tk.StringVar] = {}
        self.mag_vars: dict[str, tk.StringVar] = {}

        self._configure_style()
        self._build_ui()
        self.serial_port_var.trace_add("write", self._on_serial_port_selection_change)
        self.ident_axis_var.trace_add("write", self._on_ident_config_change)
        self.ident_mode_var.trace_add("write", self._on_ident_config_change)
        for var in (
            self.ident_pulse_var,
            self.ident_duration_var,
            self.ident_hold_var,
            self.ident_repeat_var,
            self.ident_bit_var,
            self.ident_seed_var,
            self.ident_alpha_center_var,
            self.ident_beta_center_var,
        ):
            var.trace_add("write", self._on_ident_config_change)
        self._on_ident_config_change()
        self.after(1000, self._check_link_health)
        self.after(RX_DRAIN_IDLE_MS, self._drain_rx)
        self.after(LOG_FLUSH_PERIOD_MS, self._flush_log)
        self.after(250, self._baro_tick)
        self.after(100, self._imu_poll_tick)
        self.after(100, self._firmware_drain_events)
        self.after(100, self._v1_drain_events)
        self.after(250, self._v2_tick)
        self.after_idle(self._validation_load_latest_artifact)
        self.after_idle(self._v1_load_latest_session)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _configure_compact_scaling(self) -> None:
        """按真实 DPI 设置点→像素换算，让文字在高分屏上原生清晰。

        进程已是 DPI-aware，因此 tk 报出来的屏幕尺寸是真实像素；点→像素必须
        显式乘以系统 DPI 比例，否则字号会缩成原来的 1/dpi 而且被系统再拉伸。
        紧凑系数只用来微调密度，不再用它把整体缩小到发虚。
        """
        try:
            native = float(self.tk.call("tk", "scaling"))
        except (tk.TclError, TypeError, ValueError):
            native = 1.333333333
        dpi_scale = getattr(self, "ui_dpi_scale", 1.0)
        # 96 DPI 基准下 1pt = 1.3333px；DPI 感知后要自己补上系统缩放。
        base = 1.3333333 * dpi_scale
        screen_height = self.winfo_screenheight()
        compact = 0.90 if screen_height <= 1080 else 0.95
        self.ui_native_scaling = native
        self.ui_scaling = max(1.0, base * compact)
        self.tk.call("tk", "scaling", self.ui_scaling)

    def _configure_style(self) -> None:
        palette = UI_PALETTE
        self.ui_palette = palette
        self.configure(background=palette["canvas"])
        self.option_add("*Font", (UI_FONT, UI_SIZE))
        self.option_add("*Text.Font", (UI_FONT, UI_SIZE))
        # 消息框/下拉列表等经典 Tk 部件不吃 ttk 样式，单独压深，避免深色页面里
        # 弹出一块刺眼的白。
        self.option_add("*Listbox.background", palette["panel"])
        self.option_add("*Listbox.foreground", palette["ink"])
        self.option_add("*Listbox.selectBackground", palette["accent"])
        self.option_add("*Listbox.selectForeground", palette["accent_ink"])
        style = ttk.Style(self)
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure(".", font=(UI_FONT, UI_SIZE), background=palette["surface"],
                        foreground=palette["ink"])

        # ---- 容器 ----
        style.configure("TFrame", background=palette["surface"])
        style.configure("Shell.TFrame", background=palette["canvas"])
        style.configure("Page.TFrame", background=palette["surface"])
        style.configure("Card.TFrame", background=palette["panel"])
        style.configure("AccentBar.TFrame", background=palette["accent"])
        style.configure("Rule.TFrame", background=palette["border"])

        # ---- 文字 ----
        style.configure("TLabel", background=palette["surface"], foreground=palette["ink"])
        style.configure(
            "PageTitle.TLabel", background=palette["surface"], foreground=palette["ink"],
            font=(UI_FONT, UI_SIZE_TITLE, "bold"),
        )
        style.configure(
            "Eyebrow.TLabel", background=palette["surface"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE_SM, "bold"),
        )
        style.configure(
            "SectionTitle.TLabel", background=palette["surface"], foreground=palette["ink_dim"],
            font=(UI_FONT, UI_SIZE, "bold"),
        )
        style.configure("Muted.TLabel", background=palette["surface"], foreground=palette["muted"])
        style.configure("CardMuted.TLabel", background=palette["panel"], foreground=palette["muted"])
        style.configure("Card.TLabel", background=palette["panel"], foreground=palette["ink"])
        # 数值一律等宽：工程上位机读数抖动时不会左右跳动。
        style.configure(
            "Mono.TLabel", background=palette["surface"], foreground=palette["ink_dim"],
            font=(UI_MONO, UI_SIZE),
        )
        style.configure(
            "CardMono.TLabel", background=palette["panel"], foreground=palette["ink_dim"],
            font=(UI_MONO, UI_SIZE),
        )
        # 指引不再是浅蓝气泡，而是一条左侧强调线的提示行。
        style.configure(
            "Guide.TLabel", background=palette["accent_soft"], foreground=palette["ink"],
            font=(UI_FONT, UI_SIZE), padding=(10, 6),
        )
        for name, color in (
            ("Pass", palette["green"]),
            ("Warn", palette["amber"]),
            ("Fail", palette["red"]),
        ):
            style.configure(
                f"{name}.TLabel", background=palette["surface"], foreground=color,
                font=(UI_FONT, UI_SIZE, "bold"), padding=(2, 2),
            )
            style.configure(
                f"Card{name}.TLabel", background=palette["panel"], foreground=color,
                font=(UI_FONT, UI_SIZE, "bold"), padding=(2, 2),
            )
        style.configure(
            "Idle.TLabel", background=palette["surface"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE), padding=(2, 2),
        )
        # 步骤条"当前步"用强调色，与 Pass/Warn/Fail 的语义色区分开。
        style.configure(
            "Active.TLabel", background=palette["surface"], foreground=palette["accent"],
            font=(UI_FONT, UI_SIZE, "bold"), padding=(2, 2),
        )
        style.configure(
            "CardIdle.TLabel", background=palette["panel"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE), padding=(2, 2),
        )

        # ---- 分组框 ----
        style.configure(
            "TLabelframe", background=palette["surface"], bordercolor=palette["border"],
            lightcolor=palette["border"], darkcolor=palette["border"], relief=tk.SOLID,
        )
        style.configure(
            "TLabelframe.Label", background=palette["surface"], foreground=palette["muted"],
            font=(UI_FONT, UI_SIZE_SM, "bold"), padding=(4, 0),
        )

        # ---- 按钮：全局只有 Primary 是实心，其余一律描边/幽灵 ----
        style.configure(
            "TButton", background=palette["surface"], foreground=palette["ink_dim"],
            bordercolor=palette["border_strong"], focusthickness=1,
            focuscolor=palette["accent"], relief=tk.SOLID,
            padding=(11, 5), font=(UI_FONT, UI_SIZE),
        )
        style.map(
            "TButton",
            background=[("disabled", palette["disabled"]), ("active", palette["raised"])],
            foreground=[("disabled", palette["disabled_ink"]), ("active", palette["ink"])],
            bordercolor=[("disabled", palette["border"])],
        )
        style.configure(
            "Primary.TButton", background=palette["accent"], foreground=palette["accent_ink"],
            bordercolor=palette["accent"], padding=(14, 6), font=(UI_FONT, UI_SIZE, "bold"),
        )
        style.map(
            "Primary.TButton",
            background=[("disabled", palette["disabled"]), ("pressed", palette["accent"]),
                        ("active", palette["accent_hover"])],
            foreground=[("disabled", palette["disabled_ink"]), ("!disabled", palette["accent_ink"])],
            bordercolor=[("disabled", palette["border"]), ("!disabled", palette["accent"])],
        )
        # 需二次确认(Warning)与破坏性(Danger)操作：描边 + 极淡填充，够醒目但不喧宾夺主。
        for name, color, soft, hover in (
            ("Success", palette["green"], palette["green_soft"], palette["green"]),
            ("Warning", palette["amber"], palette["amber_soft"], palette["amber_hover"]),
            ("Danger", palette["red"], palette["red_soft"], palette["red_hover"]),
        ):
            style.configure(
                f"{name}.TButton", background=soft, foreground=color,
                bordercolor=color, padding=(12, 5), font=(UI_FONT, UI_SIZE, "bold"),
            )
            style.map(
                f"{name}.TButton",
                background=[("disabled", palette["disabled"]), ("active", palette["raised"])],
                foreground=[("disabled", palette["disabled_ink"]), ("active", hover)],
                bordercolor=[("disabled", palette["border"]), ("!disabled", color)],
            )
        style.configure(
            "Secondary.TButton", background=palette["surface"], foreground=palette["muted"],
            bordercolor=palette["border"], padding=(11, 5), font=(UI_FONT, UI_SIZE),
        )
        style.map(
            "Secondary.TButton",
            background=[("active", palette["raised"]), ("disabled", palette["disabled"])],
            foreground=[("disabled", palette["disabled_ink"]), ("active", palette["ink_dim"])],
        )
        style.configure(
            "Link.TButton", background=palette["surface"], foreground=palette["accent"],
            bordercolor=palette["surface"], relief=tk.FLAT, padding=(6, 4),
        )
        style.map(
            "Link.TButton",
            background=[("active", palette["accent_soft"]), ("disabled", palette["surface"])],
            foreground=[("disabled", palette["disabled_ink"])],
        )

        # ---- 页签 ----
        style.configure(
            "TNotebook", background=palette["canvas"], bordercolor=palette["border"],
            tabmargins=(2, 4, 2, 0),
        )
        style.configure(
            "TNotebook.Tab", background=palette["canvas"], foreground=palette["muted"],
            bordercolor=palette["canvas"], padding=(11, 6), font=(UI_FONT, UI_SIZE_SM),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", palette["surface"]), ("active", palette["raised"])],
            foreground=[("selected", palette["accent"]), ("active", palette["ink_dim"])],
            font=[("selected", (UI_FONT, UI_SIZE_SM, "bold"))],
        )

        # ---- 表格 ----
        style.configure(
            "Treeview", background=palette["panel"], fieldbackground=palette["panel"],
            foreground=palette["ink_dim"], bordercolor=palette["border"],
            rowheight=21, font=(UI_MONO, UI_SIZE),
        )
        style.map(
            "Treeview",
            background=[("selected", palette["accent_soft"])],
            foreground=[("selected", palette["ink"])],
        )
        style.configure(
            "Treeview.Heading", background=palette["raised"], foreground=palette["muted"],
            bordercolor=palette["border"], relief=tk.FLAT, padding=(8, 5),
            font=(UI_FONT, UI_SIZE_SM, "bold"),
        )
        style.map("Treeview.Heading", background=[("active", palette["border_strong"])])

        # ---- 输入 ----
        style.configure(
            "TEntry", fieldbackground=palette["panel"], foreground=palette["ink"],
            insertcolor=palette["accent"], bordercolor=palette["border_strong"],
            lightcolor=palette["border"], darkcolor=palette["border"], padding=(6, 4),
        )
        style.map("TEntry", bordercolor=[("focus", palette["accent"])])
        style.configure(
            "TCombobox", fieldbackground=palette["panel"], background=palette["panel"],
            foreground=palette["ink"], arrowcolor=palette["muted"],
            bordercolor=palette["border_strong"], lightcolor=palette["border"],
            darkcolor=palette["border"], padding=(5, 3), arrowsize=13,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", palette["panel"])],
            foreground=[("readonly", palette["ink"])],
            selectbackground=[("readonly", palette["panel"])],
            selectforeground=[("readonly", palette["ink"])],
            bordercolor=[("focus", palette["accent"])],
        )
        style.configure(
            "Horizontal.TProgressbar", troughcolor=palette["panel"], background=palette["accent"],
            bordercolor=palette["border"], lightcolor=palette["accent"],
            darkcolor=palette["accent"], thickness=6,
        )
        style.configure(
            "TCheckbutton", background=palette["surface"], foreground=palette["ink_dim"],
            indicatorbackground=palette["panel"], indicatorforeground=palette["accent_ink"],
            bordercolor=palette["border_strong"],
            indicatormargin=(0, 0, 6, 0), padding=(3, 2),
        )
        style.map(
            "TCheckbutton",
            indicatorbackground=[("selected", palette["accent"]), ("active", palette["raised"])],
            foreground=[("disabled", palette["disabled_ink"]), ("active", palette["ink"])],
        )
        style.configure("TSeparator", background=palette["border"])
        style.configure("TPanedwindow", background=palette["canvas"], sashwidth=5)
        style.configure(
            "Vertical.TScrollbar", background=palette["raised"], troughcolor=palette["surface"],
            bordercolor=palette["surface"], arrowcolor=palette["muted"], width=11,
        )
        style.map("Vertical.TScrollbar", background=[("active", palette["border_strong"])])
        style.configure(
            "Horizontal.TScrollbar", background=palette["raised"], troughcolor=palette["surface"],
            bordercolor=palette["surface"], arrowcolor=palette["muted"],
        )

    def _build_port_label(self, p) -> str:
        desc = (p.description or "").upper()
        hwid = (p.hwid or "").upper()
        combined = f"{desc} {hwid}"
        if "CH340" in combined:
            return f"CH340 ({p.device})"
        if "CP210" in combined:
            return f"CP210x ({p.device})"
        if "FTDI" in combined or "FT232" in combined or "FT4232" in combined:
            return f"FTDI ({p.device})"
        if "BLUETOOTH" in combined or "BLE" in combined:
            return f"BLE ({p.device})"
        if "STLINK" in combined or "ST-LINK" in combined:
            return f"STLink ({p.device})"
        if desc and desc not in ("USB SERIAL DEVICE", "USB SERIAL", "SERIAL"):
            return f"{p.device} - {p.description}"
        return p.device

    def _refresh_serial_ports(self) -> list[str]:
        if not HAS_PYSERIAL or serial is None:
            return []
        try:
            ports = list(serial.tools.list_ports.comports())  # type: ignore[attr-defined]
        except Exception:
            return []
        self._serial_port_map.clear()
        self._serial_port_identity.clear()
        names: list[str] = []
        for p in ports:
            label = self._build_port_label(p)
            self._serial_port_map[label] = p.device
            self._serial_port_identity[str(p.device)] = serial_port_identity(p)
            names.append(label)
        return names

    def _default_serial_port(self) -> str:
        names = self._refresh_serial_ports()
        if names:
            return names[0]
        return "COM18"

    def _on_refresh_ports(self) -> None:
        if self.serial_transport.is_connected:
            return
        names = self._refresh_serial_ports()
        if hasattr(self, '_serial_port_combo'):
            self._serial_port_combo['values'] = names
        if names:
            self.serial_port_var.set(names[0])

    def _refresh_serial_selection_lock(self) -> None:
        if not hasattr(self, "_serial_port_combo"):
            return
        connected = self.serial_transport.is_connected
        self._serial_port_combo.configure(state=tk.DISABLED if connected else "readonly")
        self._serial_refresh_button.configure(state=tk.DISABLED if connected else tk.NORMAL)

    def _on_serial_port_selection_change(self, *_args: object) -> None:
        self.firmware_unknown_usb_override_var.set(False)
        self._firmware_refresh_safety()

    def _on_transport_mode_change(self, *args) -> None:
        self._tcp_frame.pack_forget()
        self._udp_frame.pack_forget()
        self._serial_frame.pack_forget()

        mode = self.transport_var.get()
        if mode == "serial":
            self._serial_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
            self._on_refresh_ports()
        elif mode == "udp":
            self._udp_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)
        else:
            self._tcp_frame.pack(side=tk.LEFT, padx=(0, 12), before=self._action_frame)

    def _current_transport(self) -> TransportBase:
        mode = self.transport_var.get()
        if mode == "serial":
            return self.serial_transport
        if mode == "udp":
            return self.udp_transport
        return self.tcp_transport

    def _transport_connected(self) -> bool:
        return self.transport.is_connected

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=12, style="Shell.TFrame")
        root.pack(fill=tk.BOTH, expand=True)

        self._build_connection_bar(root)

        body = ttk.PanedWindow(root, orient=tk.VERTICAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self._body_pane = body

        self.notebook = ttk.Notebook(body)
        body.add(self.notebook, weight=6)

        overview = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        validation_scroll = VerticalScrolledFrame(self.notebook)
        metrology_scroll = VerticalScrolledFrame(self.notebook)
        acceptance_v2_scroll = VerticalScrolledFrame(self.notebook)
        firmware_scroll = VerticalScrolledFrame(self.notebook)
        validation = validation_scroll.content
        metrology = metrology_scroll.content
        acceptance_v2 = acceptance_v2_scroll.content
        firmware = firmware_scroll.content
        baro = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        imu = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        gps = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        ident = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        params = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        servos = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        commands = ttk.Frame(self.notebook, padding=14, style="Page.TFrame")
        self.baro_tab = baro
        self.imu_tab = imu
        self.validation_tab = validation_scroll
        self.v1_tab = metrology_scroll
        self.v2_tab = acceptance_v2_scroll
        self.firmware_tab = firmware_scroll
        self.gps_tab = gps
        self.ident_tab = ident

        self.notebook.add(overview, text="总览")
        self.notebook.add(validation_scroll, text="01 · 坐标 V0")
        self.notebook.add(metrology_scroll, text="02 · IMU V1")
        self.notebook.add(acceptance_v2_scroll, text="03 · 链路 V2A")
        self.notebook.add(firmware_scroll, text="维护 · 固件升级")
        self.notebook.add(imu, text="IMU 监视（旧链）")
        self.notebook.add(baro, text="气压计")
        self.notebook.add(gps, text="GPS / 磁力计")
        self.notebook.add(servos, text="舵机")
        self.notebook.add(params, text="参数 / PID")
        self.notebook.add(ident, text="系统辨识")
        self.notebook.add(commands, text="诊断 / 命令")

        self._build_overview_page(overview)
        self._build_validation_page(validation)
        self._build_v1_page(metrology)
        self._build_v2_page(acceptance_v2)
        self._build_firmware_update_page(firmware)
        self._build_baro_page(baro)
        self._build_imu_page(imu)
        self._build_gps_page(gps)
        self._build_ident_page(ident)
        self._build_params_page(params)
        self._build_servo_page(servos)
        self._build_command_page(commands)

        self.log_box = ttk.LabelFrame(body, text="原始命令日志", padding=8)
        body.add(self.log_box, weight=1)
        self._build_log_area(self.log_box)
        self.after_idle(self._toggle_log_area)

    def _build_connection_bar(self, parent: ttk.Frame) -> None:
        conn = ttk.Frame(parent)
        conn.pack(fill=tk.X)

        ttk.Label(conn, text="通道").pack(side=tk.LEFT)
        ttk.Combobox(
            conn,
            textvariable=self.transport_var,
            values=("tcp", "udp", "serial"),
            width=8,
            state="readonly",
        ).pack(side=tk.LEFT, padx=(4, 12))

        # --- TCP controls ---
        self._tcp_frame = ttk.Frame(conn)
        ttk.Label(self._tcp_frame, text="监听地址").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(self._tcp_frame, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(self._tcp_frame, text="端口").pack(side=tk.LEFT)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(self._tcp_frame, textvariable=self.port_var, width=8).pack(side=tk.LEFT)

        # --- UDP controls ---
        self._udp_frame = ttk.Frame(conn)
        ttk.Label(self._udp_frame, text="本地").pack(side=tk.LEFT)
        ttk.Entry(self._udp_frame, textvariable=self.udp_bind_var, width=13).pack(side=tk.LEFT, padx=(4, 4))
        ttk.Entry(self._udp_frame, textvariable=self.udp_local_port_var, width=7).pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(self._udp_frame, text="模块").pack(side=tk.LEFT)
        ttk.Entry(self._udp_frame, textvariable=self.udp_module_ip_var, width=15).pack(side=tk.LEFT, padx=(4, 4))
        ttk.Entry(self._udp_frame, textvariable=self.udp_module_port_var, width=7).pack(side=tk.LEFT)

        # --- Serial controls (hidden by default) ---
        self._serial_frame = ttk.Frame(conn)
        ttk.Label(self._serial_frame, text="串口").pack(side=tk.LEFT)
        self._serial_port_combo = ttk.Combobox(
            self._serial_frame,
            textvariable=self.serial_port_var,
            values=self._refresh_serial_ports(),
            width=20,
            state="readonly",
        )
        self._serial_port_combo.pack(side=tk.LEFT, padx=(4, 2))
        self._serial_refresh_button = ttk.Button(
            self._serial_frame, text="刷新", command=self._on_refresh_ports
        )
        self._serial_refresh_button.pack(side=tk.LEFT)
        ttk.Label(self._serial_frame, text="波特率").pack(side=tk.LEFT)
        ttk.Entry(self._serial_frame, textvariable=self.serial_baud_var, width=8).pack(side=tk.LEFT, padx=(4, 0))

        # Show TCP frame initially
        self._tcp_frame.pack(side=tk.LEFT, padx=(0, 12))

        # --- Action buttons (always visible) ---
        self._action_frame = ttk.Frame(conn)
        self._action_frame.pack(side=tk.LEFT)
        ttk.Button(self._action_frame, text="启动连接", command=self._start,
                   style="Primary.TButton").pack(side=tk.LEFT)
        ttk.Button(self._action_frame, text="停止", command=self._stop,
                   style="Danger.TButton").pack(side=tk.LEFT, padx=6)

        utility = ttk.Frame(parent)
        utility.pack(fill=tk.X, pady=(7, 0))
        ttk.Label(utility, text="快捷诊断", style="Eyebrow.TLabel").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(utility, text="PING", command=lambda: self._send_proto(PROTO_REQ_PING, "PING"),
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(utility, text="硬件状态", command=self._request_overview_status,
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=4)
        ttk.Button(utility, text="读取配置", command=self._read_all_params,
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(
            utility,
            text="原始日志",
            variable=self.show_log_var,
            command=self._toggle_log_area,
        ).pack(side=tk.LEFT, padx=(10, 2))
        ttk.Label(utility, text="链路").pack(side=tk.LEFT, padx=(18, 4))
        self.link_status_label = ttk.Label(
            utility, textvariable=self.link_var, style="Fail.TLabel")
        self.link_status_label.pack(side=tk.LEFT)

        self.transport_var.trace_add("write", self._on_transport_mode_change)

    def _build_overview_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="FLIGHT CONTROL WORKBENCH", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="系统总览", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent, text="先看异常模块，再进入对应页面处理；绿色表示已就绪，灰色表示尚未取到数据，红色整行标注表示必须停止。",
            style="Muted.TLabel",
        ).pack(fill=tk.X, pady=(3, 8))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        toolbar = ttk.Frame(left)
        toolbar.pack(fill=tk.X, pady=(0, 8))
        ttk.Button(toolbar, text="刷新全部", command=self._request_overview_status,
                   style="Primary.TButton").pack(side=tk.LEFT)
        ttk.Button(toolbar, text="读取 UART 统计", command=lambda: self._send_proto(PROTO_REQ_STATUS, "STATUS?"),
                   style="Secondary.TButton").pack(side=tk.LEFT, padx=6)
        ttk.Label(toolbar, textvariable=self.last_cmd_var).pack(side=tk.LEFT, padx=(16, 0))

        columns = ("state", "stage", "value", "code", "hint")
        self.module_tree = ttk.Treeview(left, columns=columns, show="tree headings", height=10)
        self.module_tree.heading("#0", text="模块")
        self.module_tree.column("#0", width=160, anchor=tk.W)
        headings = {
            "state": "状态",
            "stage": "失败阶段",
            "value": "关键读数",
            "code": "返回码",
            "hint": "提示",
        }
        widths = {"state": 80, "stage": 120, "value": 230, "code": 170, "hint": 360}
        for col, label in headings.items():
            self.module_tree.heading(col, text=label)
            self.module_tree.column(col, width=widths[col], anchor=tk.W)

        self.module_tree.pack(fill=tk.BOTH, expand=True)
        self.module_tree.bind("<<TreeviewSelect>>", self._on_module_select)
        # 正常/等待只给文字上色；整行填充只留给真正的故障，否则一屏全是色块。
        self.module_tree.tag_configure("pass", foreground=UI_PALETTE["green"])
        self.module_tree.tag_configure("warn", foreground=UI_PALETTE["muted"])
        self.module_tree.tag_configure(
            "fail", background=UI_PALETTE["red_soft"], foreground=UI_PALETTE["red"])

        for key, title, _command in MODULES:
            values = {
                "title": tk.StringVar(value=title),
                "state": tk.StringVar(value="等待数据"),
                "stage": tk.StringVar(value="-"),
                "value": tk.StringVar(value="-"),
                "code": tk.StringVar(value="-"),
                "hint": tk.StringVar(value="点击“硬件状态”或等待心跳"),
                "last": tk.StringVar(value="-"),
            }
            self.module_state[key] = values
            self.module_tree.insert(
                "",
                tk.END,
                iid=key,
                values=(values["state"].get(), values["stage"].get(), values["value"].get(), values["code"].get(), values["hint"].get()),
                text=title,
                tags=("warn",),
            )
        self.module_tree.selection_set("FLASH")

        detail = ttk.LabelFrame(right, text="模块详情", padding=10)
        detail.pack(fill=tk.BOTH, expand=True)
        self.detail_vars = {
            "title": tk.StringVar(value="GD25Q32 Flash"),
            "state": tk.StringVar(value="等待数据"),
            "stage": tk.StringVar(value="-"),
            "value": tk.StringVar(value="-"),
            "code": tk.StringVar(value="-"),
            "hint": tk.StringVar(value="-"),
            "last": tk.StringVar(value="-"),
        }
        self._detail_row(detail, 0, "模块", self.detail_vars["title"])
        self._detail_row(detail, 1, "状态", self.detail_vars["state"])
        self._detail_row(detail, 2, "失败阶段", self.detail_vars["stage"])
        self._detail_row(detail, 3, "关键读数", self.detail_vars["value"])
        self._detail_row(detail, 4, "返回码", self.detail_vars["code"])
        self._detail_row(detail, 5, "提示", self.detail_vars["hint"], wrap=420)
        self._detail_row(detail, 6, "最近原始行", self.detail_vars["last"], wrap=420)

        actions = ttk.Frame(detail)
        actions.grid(row=7, column=0, columnspan=2, sticky=tk.EW, pady=(12, 0))
        ttk.Button(actions, text="请求该模块详情", command=self._request_selected_module).pack(side=tk.LEFT)
        ttk.Button(actions, text="打开气压计页", command=self._open_baro_tab).pack(side=tk.LEFT, padx=6)
        ttk.Button(actions, text="打开姿态页", command=self._open_imu_tab).pack(side=tk.LEFT)
        ttk.Button(actions, text="打开 GPS 页", command=self._open_gps_tab).pack(side=tk.LEFT, padx=6)
        detail.columnconfigure(1, weight=1)

    def _detail_row(self, parent: ttk.Frame, row: int, label: str, variable: tk.StringVar, wrap: int = 0) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.NW, padx=(0, 8), pady=4)
        ttk.Label(parent, textvariable=variable, wraplength=wrap).grid(row=row, column=1, sticky=tk.EW, pady=4)

    def _build_baro_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求状态", command=lambda: self._send_proto(PROTO_REQ_STATUS, "STATUS?")).pack(side=tk.LEFT)
        ttk.Button(top, text="请求气压计", command=lambda: self._send_proto(PROTO_REQ_BARO, "BARO?")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="开始流", command=lambda: self._send_proto(PROTO_REQ_BARO_STREAM, f"BARO STREAM 1 {BARO_STREAM_PERIOD_MS}", f"BARO STREAM 1 {BARO_STREAM_PERIOD_MS}")).pack(side=tk.LEFT)
        ttk.Button(top, text="停止流", command=lambda: self._send_proto(PROTO_REQ_BARO_STREAM, "BARO STREAM 0", "BARO STREAM 0")).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(top, text="暂存新数据", variable=self.baro_capture_enabled).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(top, text="清空暂存", command=self._clear_baro_buffer).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="导出 CSV", command=self._export_baro_csv).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.baro_count_var).pack(side=tk.LEFT, padx=(12, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=2)
        panes.add(right, weight=3)

        fields = [
            ("state", "状态"),
            ("pressure", "压力"),
            ("temperature", "温度"),
            ("altitude", "高度"),
            ("raw_pressure", "压力原始值"),
            ("raw_temperature", "温度原始值"),
            ("sample_count", "样本计数"),
            ("product_id", "Product ID"),
            ("split_id", "Split ID"),
            ("txrx_id", "TxRx ID"),
            ("bmp_id", "BMP280 ID"),
            ("init_status", "init 返回码"),
            ("split_status", "split 返回码"),
            ("txrx_status", "txrx 返回码"),
            ("cs_level", "CS 电平"),
            ("miso_level", "MISO 电平"),
            ("stage", "失败阶段"),
            ("last_update", "最近更新"),
        ]
        diagnostic = ttk.LabelFrame(left, text="实时 / 诊断字段", padding=10)
        diagnostic.pack(fill=tk.X)
        for row, (key, label) in enumerate(fields):
            self.baro_vars[key] = tk.StringVar(value="-")
            ttk.Label(diagnostic, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
            ttk.Label(diagnostic, textvariable=self.baro_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=2)
        diagnostic.columnconfigure(1, weight=1)

        sample_box = ttk.LabelFrame(left, text="暂存数据预览", padding=8)
        sample_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.baro_sample_tree = ttk.Treeview(sample_box, columns=("t", "p", "temp", "alt"), show="headings", height=8)
        for col, label, width in [
            ("t", "t(s)", 70),
            ("p", "pressure", 100),
            ("temp", "temp", 80),
            ("alt", "alt", 80),
        ]:
            self.baro_sample_tree.heading(col, text=label)
            self.baro_sample_tree.column(col, width=width, anchor=tk.W)
        self.baro_sample_tree.pack(fill=tk.BOTH, expand=True)

        plot_box = ttk.LabelFrame(right, text="曲线", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True)
        controls = ttk.Frame(plot_box)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="字段").pack(side=tk.LEFT)
        ttk.Combobox(
            controls,
            textvariable=self.plot_var,
            values=("pressure", "temperature", "altitude"),
            width=14,
            state="readonly",
        ).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="刷新曲线", command=self._update_baro_plot).pack(side=tk.LEFT)

        if HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None:
            self.baro_figure = Figure(figsize=(5, 3), dpi=100)
            self.baro_axis = self.baro_figure.add_subplot(111)
            self.baro_canvas = FigureCanvasTkAgg(self.baro_figure, master=plot_box)
            self.baro_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(8, 0))
            self._update_baro_plot()
        else:
            ttk.Label(
                plot_box,
                text=f"未安装 matplotlib，曲线区停用。安装后重启面板即可绘图。\npython -m pip install matplotlib\n{MATPLOTLIB_ERROR}",
                wraplength=520,
            ).pack(fill=tk.X, pady=(12, 0))
            self.baro_figure = None
            self.baro_axis = None
            self.baro_canvas = None

    VALIDATION_STEPS = ("坐标发现", "RAM 复验", "写入 Flash")

    def _build_validation_stepper(self, parent: ttk.Frame) -> None:
        """三段流程用步骤条表达顺序，因此按钮文案里不再需要 1/2/3/4 编号。"""
        strip = ttk.Frame(parent)
        strip.pack(fill=tk.X, pady=(0, 8))
        self.validation_step_dots = []
        for index, name in enumerate(self.VALIDATION_STEPS):
            if index:
                ttk.Label(strip, text="────", style="Muted.TLabel").pack(
                    side=tk.LEFT, padx=7)
            dot = ttk.Label(strip, text=f"○  {name}", style="Idle.TLabel")
            dot.pack(side=tk.LEFT)
            self.validation_step_dots.append(dot)
        self._validation_refresh_stepper()

    def _validation_current_step(self) -> int:
        """0-based 当前阶段；返回 3 表示三段全部完成。"""
        text = self.validation_phase_var.get()
        for index in range(1, len(self.VALIDATION_STEPS) + 1):
            if text.startswith(f"阶段 {index}/{len(self.VALIDATION_STEPS)}"):
                return index - 1
        return len(self.VALIDATION_STEPS) if text.startswith("V0 已完成") else 0

    def _validation_refresh_stepper(self) -> None:
        current = self._validation_current_step()
        for index, label in enumerate(self.validation_step_dots):
            name = self.VALIDATION_STEPS[index]
            if index < current:
                label.configure(text=f"●  {name}", style="Pass.TLabel")
            elif index == current:
                label.configure(text=f"●  {name}", style="Active.TLabel")
            else:
                label.configure(text=f"○  {name}", style="Idle.TLabel")

    _STAGE_ROW_TAGS = {
        "PASS": "pass",
        "FAIL": "fail",
        "WARN": "warn",
        "COLLECTING": "active",
    }

    def _validation_recolor_stage_rows(self) -> None:
        """按状态列给步骤表上色。

        状态由十来处调用点写入，逐处打标签容易漏；这里统一按当前状态列回填，
        因此新增写入点不会退化成"没颜色"。
        """
        tree = getattr(self, "validation_stage_tree", None)
        if tree is None:
            return
        for item in tree.get_children():
            values = tree.item(item, "values")
            status = str(values[0]).upper() if values else ""
            tree.item(item, tags=(self._STAGE_ROW_TAGS.get(status, "muted"),))

    _GATE_INDICATORS = {
        "ok": ("●", "Pass.TLabel"),
        "wait": ("◐", "Warn.TLabel"),
        "bad": ("●", "Fail.TLabel"),
    }

    def _validation_set_gate(self, key: str, state: str, value: str) -> None:
        """状态用指示灯圆点，文字栏只放可读的实测值。"""
        dot = self.validation_gate_dots.get(key)
        if dot is not None:
            glyph, style = self._GATE_INDICATORS.get(state, ("○", "Idle.TLabel"))
            dot.configure(text=glyph, style=style)
        variable = {
            "conn": self.validation_connection_gate_var,
            "snapshot": self.validation_snapshot_gate_var,
            "bias": self.validation_bias_gate_var,
            "output": self.validation_output_gate_var,
            "health": self.validation_health_gate_var,
        }.get(key)
        if variable is not None:
            variable.set(value)

    def _build_validation_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="01  /  建立唯一坐标与极性", style="Eyebrow.TLabel").pack(anchor=tk.W)
        header = ttk.Frame(parent)
        header.pack(fill=tk.X)
        ttk.Label(header, text="飞行器传感器验收 V0", style="PageTitle.TLabel").pack(side=tk.LEFT)
        self.validation_status_label = ttk.Label(
            header,
            textvariable=self.validation_session_var,
            style="Warn.TLabel",
        )
        self.validation_status_label.pack(side=tk.RIGHT)

        ttk.Label(
            parent,
            text=(
                "采集阶段只读；只有下方 A/B/C 映射流程可在双重确认后改 RAM 或写参数 Flash，"
                "不会重新烧录程序。支持 USB CDC；传感器映射通过仍不等于整机允许自由飞行。"
            ),
            style="Muted.TLabel",
            wraplength=1080,
        ).pack(fill=tk.X, pady=(3, 8))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        self._build_validation_stepper(parent)

        safety = ttk.LabelFrame(parent, text="开始前安全门", padding=8)
        safety.pack(fill=tk.X)
        ttk.Checkbutton(
            safety,
            text="已拆除全部桨叶",
            variable=self.validation_props_removed_var,
            command=self._validation_update_safety_text,
        ).pack(side=tk.LEFT)
        ttk.Checkbutton(
            safety,
            text="电调动力已断开或机体已可靠固定",
            variable=self.validation_power_safe_var,
            command=self._validation_update_safety_text,
        ).pack(side=tk.LEFT, padx=(16, 0))
        ttk.Label(safety, textvariable=self.validation_safety_var).pack(side=tk.RIGHT)

        # 采集操作与会话管理分两行：同一行里只允许存在一个实心主操作按钮，
        # 其余一律描边，避免出现"三色按钮并排"的玩具感。
        toolbar = ttk.Frame(parent)
        toolbar.pack(fill=tk.X, pady=(8, 0))
        self.validation_begin_button = ttk.Button(
            toolbar,
            text="开始采集当前步骤",
            command=self._validation_begin_stage,
            state=tk.DISABLED,
            style="Primary.TButton",
        )
        self.validation_begin_button.pack(side=tk.LEFT)
        self.validation_finish_button = ttk.Button(
            toolbar,
            text="停止并分析",
            command=self._validation_finish_stage,
            state=tk.DISABLED,
            style="Secondary.TButton",
        )
        self.validation_finish_button.pack(side=tk.LEFT, padx=(6, 0))
        self.validation_report_button = ttk.Button(
            toolbar,
            text="生成报告",
            command=self._validation_write_report,
            state=tk.DISABLED,
            style="Secondary.TButton",
        )
        self.validation_report_button.pack(side=tk.LEFT, padx=(18, 0))
        ttk.Button(toolbar, text="报告目录", command=self._validation_open_report_dir,
                   style="Link.TButton").pack(side=tk.LEFT, padx=(4, 0))
        ttk.Label(toolbar, text="固件备注", style="Muted.TLabel").pack(side=tk.LEFT, padx=(18, 6))
        ttk.Entry(toolbar, textvariable=self.validation_firmware_hash_var, width=30).pack(
            side=tk.LEFT)

        session_bar = ttk.Frame(parent)
        session_bar.pack(fill=tk.X, pady=(6, 0))
        ttk.Label(session_bar, text="会话", style="Eyebrow.TLabel").pack(side=tk.LEFT, padx=(0, 8))
        self.validation_session_button = ttk.Button(
            session_bar,
            text="新建验收",
            command=self._validation_start_session,
            style="Primary.TButton",
        )
        self.validation_session_button.pack(side=tk.LEFT)
        self.validation_resume_button = ttk.Button(
            session_bar,
            text="恢复/继续",
            command=self._validation_resume_session,
            state=tk.DISABLED,
            style="Secondary.TButton",
        )
        self.validation_resume_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(session_bar, text="暂停", command=self._validation_stop_session,
                   style="Danger.TButton").pack(side=tk.LEFT, padx=(6, 0))
        self.validation_history_combo = ttk.Combobox(
            session_bar, textvariable=self.validation_history_var,
            values=(), state="readonly", width=40,
        )
        self.validation_history_combo.pack(side=tk.LEFT, padx=(18, 6))
        ttk.Button(
            session_bar, text="载入",
            command=self._validation_load_selected_history,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT)
        ttk.Button(
            session_bar, text="刷新",
            command=self._validation_refresh_history_choices,
            style="Link.TButton",
        ).pack(side=tk.LEFT, padx=(4, 0))

        readiness = ttk.LabelFrame(parent, text="就绪条件", padding=(10, 7))
        readiness.pack(fill=tk.X, pady=(9, 0))
        # 两组条件左对齐成 2×2，右侧留一列吸收富余宽度，避免被拉到屏幕两端。
        readiness.columnconfigure(2, minsize=210)
        readiness.columnconfigure(6, minsize=210)
        readiness.columnconfigure(8, weight=1)
        ttk.Frame(readiness).grid(row=0, column=8, sticky=tk.EW)
        gates = (
            ("conn", "连接", self.validation_connection_gate_var),
            ("snapshot", "快照", self.validation_snapshot_gate_var),
            ("bias", "陀螺零偏", self.validation_bias_gate_var),
            ("output", "安全输出", self.validation_output_gate_var),
        )
        for index, (key, name, variable) in enumerate(gates):
            row, base = index % 2, (index // 2) * 4
            dot = ttk.Label(readiness, text="○", style="Idle.TLabel")
            dot.grid(row=row, column=base, sticky=tk.W, pady=1)
            self.validation_gate_dots[key] = dot
            ttk.Label(readiness, text=name, style="Muted.TLabel", width=9).grid(
                row=row, column=base + 1, sticky=tk.W, padx=(5, 0))
            ttk.Label(readiness, textvariable=variable, style="Mono.TLabel").grid(
                row=row, column=base + 2, sticky=tk.W, padx=(4, 26))
        # 采样链单独一行：它一旦降级，上面四项全绿也不代表数据可用。
        health_dot = ttk.Label(readiness, text="○", style="Idle.TLabel")
        health_dot.grid(row=2, column=0, sticky=tk.W, pady=1)
        self.validation_gate_dots["health"] = health_dot
        ttk.Label(readiness, text="采样链", style="Muted.TLabel", width=9).grid(
            row=2, column=1, sticky=tk.W, padx=(5, 0))
        ttk.Label(
            readiness, textvariable=self.validation_health_gate_var, style="Mono.TLabel"
        ).grid(row=2, column=2, columnspan=6, sticky=tk.W, padx=(4, 26))
        ttk.Label(
            readiness,
            textvariable=self.validation_next_action_var,
            style="Guide.TLabel",
            wraplength=1050,
        ).grid(row=3, column=0, columnspan=9, sticky=tk.EW, pady=(8, 0))

        phase_banner = ttk.Frame(parent)
        phase_banner.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            phase_banner, textvariable=self.validation_phase_var,
            style="SectionTitle.TLabel", wraplength=1080,
        ).pack(fill=tk.X)
        ttk.Label(
            phase_banner, textvariable=self.validation_phase_action_var,
            style="Muted.TLabel", wraplength=1080,
        ).pack(fill=tk.X, pady=(3, 0))

        mapping = ttk.LabelFrame(parent, text="坐标映射：A 临时应用 → B 重采复验 → C 写入 Flash", padding=8)
        mapping.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(
            mapping,
            textvariable=self.validation_orientation_var,
            wraplength=1040,
            style="SectionTitle.TLabel",
        ).grid(row=0, column=0, columnspan=6, sticky=tk.W)
        ttk.Checkbutton(
            mapping,
            text="我已确认机头、左侧、上方的物理标记",
            variable=self.validation_axis_labels_confirmed_var,
            command=self._validation_refresh_orientation_controls,
        ).grid(row=1, column=0, columnspan=2, sticky=tk.W, pady=(6, 0))
        ttk.Checkbutton(
            mapping,
            text="我已核对图示轴映射且桨叶已拆除",
            variable=self.validation_mapping_confirmed_var,
            command=self._validation_refresh_orientation_controls,
        ).grid(row=1, column=2, columnspan=2, sticky=tk.W, padx=(16, 0), pady=(6, 0))
        self.validation_apply_button = ttk.Button(
            mapping,
            text="A · 临时应用到 RAM",
            command=self._validation_apply_candidate,
            state=tk.DISABLED,
            style="Warning.TButton",
        )
        self.validation_apply_button.grid(row=2, column=0, sticky=tk.W, pady=(7, 0))
        self.validation_revert_button = ttk.Button(
            mapping,
            text="撤销临时映射",
            command=self._validation_revert_candidate,
            state=tk.DISABLED,
            style="Danger.TButton",
        )
        self.validation_revert_button.grid(row=2, column=1, sticky=tk.W, padx=(6, 0), pady=(7, 0))
        self.validation_verify_button = ttk.Button(
            mapping,
            text="B · 清空旧样本并重采 6 步",
            command=self._validation_begin_candidate_verification,
            state=tk.DISABLED,
            style="Primary.TButton",
        )
        self.validation_verify_button.grid(row=2, column=2, sticky=tk.W, padx=(16, 0), pady=(7, 0))
        self.validation_commit_button = ttk.Button(
            mapping,
            text="C · 复验通过后写入 Flash",
            command=self._validation_commit_candidate,
            state=tk.DISABLED,
            style="Warning.TButton",
        )
        self.validation_commit_button.grid(row=2, column=3, sticky=tk.W, padx=(6, 0), pady=(7, 0))
        ttk.Button(
            mapping,
            text="读取飞机映射状态",
            command=lambda: self._send_proto(PROTO_REQ_IMU_FRAME, "IMUFRAME?"),
            style="Secondary.TButton",
        ).grid(row=2, column=4, sticky=tk.W, padx=(16, 0), pady=(7, 0))
        ttk.Label(mapping, text="飞机回报", style="Muted.TLabel").grid(
            row=3, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Label(
            mapping,
            textvariable=self.validation_orientation_target_var,
            style="Mono.TLabel",
        ).grid(row=3, column=1, columnspan=5, sticky=tk.W, pady=(8, 0))
        ttk.Label(mapping, text="最近操作", style="Muted.TLabel").grid(
            row=4, column=0, sticky=tk.W, pady=(2, 0))
        self.validation_orientation_event_label = ttk.Label(
            mapping,
            textvariable=self.validation_orientation_event_var,
            style="Idle.TLabel",
            wraplength=980,
        )
        self.validation_orientation_event_label.grid(
            row=4, column=1, columnspan=5, sticky=tk.W, pady=(2, 0))
        ttk.Label(
            mapping,
            text="首次使用仍需烧录一次支持 IMUFRAME 的固件；之后改映射无需重新编程。",
            style="Muted.TLabel",
        ).grid(row=5, column=0, columnspan=6, sticky=tk.W, pady=(5, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        steps_box = ttk.LabelFrame(
            panes, text="坐标发现样本（旧 FAIL 用于推导映射）", padding=8)
        self.validation_steps_box = steps_box
        detail_box = ttk.LabelFrame(panes, text="当前步骤与只读证据", padding=10)
        panes.add(steps_box, weight=2)
        panes.add(detail_box, weight=5)

        self.validation_stage_tree = ttk.Treeview(
            steps_box,
            columns=("status", "samples"),
            show="tree headings",
            height=12,
            selectmode="browse",
        )
        self.validation_stage_tree.heading("#0", text="步骤")
        self.validation_stage_tree.heading("status", text="状态")
        self.validation_stage_tree.heading("samples", text="样本")
        self.validation_stage_tree.column("#0", width=170, anchor=tk.W)
        self.validation_stage_tree.column("status", width=100, anchor=tk.CENTER)
        self.validation_stage_tree.column("samples", width=70, anchor=tk.CENTER)
        self.validation_stage_tree.pack(fill=tk.BOTH, expand=True)
        for tag, color in (
            ("pass", UI_PALETTE["green"]),
            ("fail", UI_PALETTE["red"]),
            ("warn", UI_PALETTE["amber"]),
            ("active", UI_PALETTE["accent"]),
            ("muted", UI_PALETTE["muted"]),
        ):
            self.validation_stage_tree.tag_configure(tag, foreground=color)
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            self.validation_stage_tree.insert(
                "",
                tk.END,
                iid=stage.value,
                text=definition.label_zh,
                values=(ValidationStatus.NOT_RUN.value, 0),
            )
        first_iid = VALIDATION_UI_STAGES[0].value
        self.validation_stage_tree.selection_set(first_iid)
        self.validation_stage_tree.focus(first_iid)
        self.validation_stage_tree.bind("<<TreeviewSelect>>", self._on_validation_stage_select)

        ttk.Label(detail_box, text="动作说明", style="SectionTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            detail_box,
            textvariable=self.validation_instruction_var,
            wraplength=720,
        ).pack(fill=tk.X, pady=(2, 8))
        ttk.Label(detail_box, textvariable=self.validation_source_var, style="Muted.TLabel").pack(anchor=tk.W)
        ttk.Label(
            detail_box,
            textvariable=self.validation_candidate_var,
            style="Muted.TLabel",
            wraplength=720,
        ).pack(fill=tk.X, pady=(4, 0))
        ttk.Label(detail_box, textvariable=self.validation_sample_var).pack(anchor=tk.W, pady=(4, 0))
        self.validation_progress = ttk.Progressbar(detail_box, mode="determinate", maximum=100.0)
        self.validation_progress.pack(fill=tk.X, pady=(4, 8))

        live_box = ttk.LabelFrame(detail_box, text="最新完整快照", padding=8)
        live_box.pack(fill=tk.X)
        ttk.Label(
            live_box,
            textvariable=self.validation_live_var,
            font=("Consolas", 10),
            wraplength=720,
        ).pack(fill=tk.X)

        ttk.Label(detail_box, text="步骤判定", style="SectionTitle.TLabel").pack(anchor=tk.W, pady=(10, 3))
        self.validation_result_text = tk.Text(detail_box, height=10, wrap=tk.WORD)
        self.validation_result_text.configure(
            font=(UI_MONO, UI_SIZE), state=tk.DISABLED,
            background=UI_PALETTE["panel"], foreground=UI_PALETTE["ink_dim"],
            relief=tk.FLAT, highlightthickness=1,
            highlightbackground=UI_PALETTE["border"],
            padx=10, pady=8,
        )
        self.validation_result_text.pack(fill=tk.BOTH, expand=True)
        ttk.Label(detail_box, textvariable=self.validation_report_var, style="Muted.TLabel").pack(fill=tk.X, pady=(6, 0))

        self._validation_update_safety_text()
        self._validation_refresh_history_choices()
        self._validation_refresh_readiness()

    def _build_v1_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="02  /  修正传感器连续误差", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="IMU 传感器计量 V1", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=("IMUCAP v4 采集：六面各 2000、陀螺静止 4500、精确 +360° X/Y/Z 最多 6000、"
                  "带标签温度平台。六面+静态通过可应用室温基础参数；精密转台+多温点通过后升级为完整参数。"),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        safety = ttk.LabelFrame(parent, text="复用 V0/USB 安全门", padding=8)
        safety.pack(fill=tk.X)
        ttk.Checkbutton(safety, text="已拆除全部桨叶", variable=self.validation_props_removed_var, command=self._v1_refresh_controls).pack(side=tk.LEFT)
        ttk.Checkbutton(safety, text="电调动力已断开或机体已可靠固定", variable=self.validation_power_safe_var, command=self._v1_refresh_controls).pack(side=tk.LEFT, padx=(14, 0))
        ttk.Checkbutton(safety, text="未知 USB 身份人工确认", variable=self.firmware_unknown_usb_override_var, command=self._v1_refresh_controls).pack(side=tk.LEFT, padx=(14, 0))

        controls = ttk.Frame(parent); controls.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(controls, text="新建 V1 会话", command=self._v1_new_session,
                   style="Primary.TButton").pack(side=tk.LEFT)
        ttk.Label(controls, text="步骤").pack(side=tk.LEFT, padx=(14, 4))
        self.v1_stage_combo = ttk.Combobox(controls, textvariable=self.v1_stage_var, values=tuple(self.v1_stage_by_label), state="readonly", width=27)
        self.v1_stage_combo.pack(side=tk.LEFT)
        ttk.Label(controls, text="温度平台标签").pack(side=tk.LEFT, padx=(12, 4))
        ttk.Entry(controls, textvariable=self.v1_temperature_platform_var, width=14).pack(side=tk.LEFT)
        capture_actions = ttk.Frame(parent); capture_actions.pack(fill=tk.X, pady=(7, 0))
        self.v1_start_button = ttk.Button(capture_actions, text="开始采集", command=self._v1_start_capture,
                                          state=tk.DISABLED, style="Primary.TButton")
        self.v1_start_button.pack(side=tk.LEFT)
        self.v1_finish_button = ttk.Button(capture_actions, text="完成本步并导出", command=self._v1_finish_capture,
                                           state=tk.DISABLED, style="Warning.TButton")
        self.v1_finish_button.pack(side=tk.LEFT, padx=(6, 0))
        self.v1_analyze_button = ttk.Button(capture_actions, text="分析并保存候选", command=self._v1_start_analysis,
                                            state=tk.DISABLED, style="Secondary.TButton")
        self.v1_analyze_button.pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(
            capture_actions,
            text="采集完成会自动导出并保存；分析不会直接改飞机参数。",
            style="Muted.TLabel",
        ).pack(side=tk.LEFT, padx=(16, 0))

        ttk.Label(parent, textvariable=self.v1_status_var, style="Guide.TLabel", wraplength=1120).pack(fill=tk.X, pady=(8, 6))
        ttk.Label(parent, textvariable=self.v1_counts_var, style="Muted.TLabel").pack(fill=tk.X)
        ttk.Label(parent, textvariable=self.v1_analysis_var, wraplength=1120).pack(fill=tk.X, pady=(4, 8))
        self.v1_progress = ttk.Progressbar(parent, mode="determinate", maximum=100.0)
        self.v1_progress.pack(fill=tk.X)
        table = ttk.LabelFrame(parent, text="本会话原始证据", padding=8); table.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.v1_capture_tree = ttk.Treeview(table, columns=("stage", "platform", "samples", "file"), show="headings", height=10)
        for name, label, width in (("stage", "步骤", 220), ("platform", "温度平台", 130), ("samples", "样本", 90), ("file", "CSV", 520)):
            self.v1_capture_tree.heading(name, text=label); self.v1_capture_tree.column(name, width=width, anchor=tk.W)
        self.v1_capture_tree.pack(fill=tk.BOTH, expand=True)
        pending = ttk.Frame(parent); pending.pack(fill=tk.X, pady=(8, 0))
        self.v1_apply_button = ttk.Button(
            pending, text="应用基础/完整候选到 RAM",
            command=self._v1_apply_candidate, state=tk.DISABLED, style="Warning.TButton")
        self.v1_apply_button.pack(side=tk.LEFT)
        self.v1_revert_button = ttk.Button(
            pending, text="撤销 RAM 候选",
            command=self._v1_revert_candidate, state=tk.DISABLED, style="Danger.TButton")
        self.v1_revert_button.pack(side=tk.LEFT, padx=(8, 0))
        self.v1_commit_button = ttk.Button(
            pending, text="确认后写入参数 Flash",
            command=self._v1_commit_candidate, state=tk.DISABLED, style="Warning.TButton")
        self.v1_commit_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            pending, text="读取飞机 V1 状态",
            command=lambda: self._send_proto(PROTO_REQ_IMU_CAL, "IMUCAL?"),
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(16, 0))
        self._v1_refresh_controls()

    def _build_v2_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="03  /  验证控制链与执行方向", style="Eyebrow.TLabel").pack(anchor=tk.W)
        ttk.Label(parent, text="控制链路安全验收 V2A", style="PageTitle.TLabel").pack(anchor=tk.W)
        ttk.Label(
            parent,
            text=("这是拆桨地面模式：500 ms 租约，USB/链路停止续租即自动退出；进入模式后电机硬锁且 ESC CCR=0。"
                  "当前版本用于 RC、导航、恢复方向和舵机 ±50 µs 的可执行初始化，不构成自由飞行放行。"),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))
        safety = ttk.LabelFrame(parent, text="必要安全门", padding=8); safety.pack(fill=tk.X)
        ttk.Checkbutton(safety, text="已拆除全部桨叶", variable=self.validation_props_removed_var).pack(side=tk.LEFT)
        ttk.Checkbutton(safety, text="动力已隔离或机体已可靠固定", variable=self.validation_power_safe_var).pack(side=tk.LEFT, padx=(16, 0))
        controls = ttk.Frame(parent); controls.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(controls, text="启动 V2A 安全模式", command=self._v2_start,
                   style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Label(controls, text="步骤").pack(side=tk.LEFT, padx=(16, 4))
        stages = (
            "rc_center", "rc_positive_roll", "rc_positive_pitch", "rc_positive_yaw",
            "nav_static", "nav_forward", "nav_left", "restore_positive_roll",
            "restore_positive_pitch", "servo_alpha_positive_50us",
            "servo_alpha_negative_50us", "servo_beta_positive_50us",
            "servo_beta_negative_50us", "failsafe",
        )
        ttk.Combobox(controls, textvariable=self.v2_stage_var, values=stages,
                     state="readonly", width=31).pack(side=tk.LEFT)
        ttk.Button(controls, text="切换并观察步骤", command=self._v2_set_stage,
                   style="Primary.TButton").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="停止 V2A", command=self._v2_stop,
                   style="Danger.TButton").pack(side=tk.LEFT, padx=(16, 0))
        ttk.Button(controls, text="读取快照", command=lambda: self._send_proto(
            PROTO_REQ_ACCEPTANCE, "ACCEPT?", "ACCEPT?"),
            style="Secondary.TButton").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Label(parent, textvariable=self.v2_status_var, style="Guide.TLabel",
                  wraplength=1120).pack(fill=tk.X, pady=(12, 8))
        live = ttk.LabelFrame(parent, text="目标端只读快照", padding=10); live.pack(fill=tk.X)
        ttk.Label(live, textvariable=self.v2_live_var, font=("Consolas", 10),
                  wraplength=1080).pack(fill=tk.X)
        ttk.Label(
            parent,
            text=("舵机步骤会命令中心值 ±50 µs，但电机始终禁用。请按机体标记目视确认机械方向；"
                  "电机旋向和偏航反扭矩属于带动力测试，本 V2A 明确不执行。"),
            style="Muted.TLabel", wraplength=1120,
        ).pack(fill=tk.X, pady=(10, 0))

    def _v2_start(self) -> None:
        gate_ok, reason = self._firmware_snapshot_gate()
        if not gate_ok or not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("V2A 安全门未通过", reason if not gate_ok else "必须拆桨并隔离动力")
            return
        if not self.transport.send_line("ACCEPT V2 START props=1"):
            messagebox.showerror("V2A", "命令发送失败")
            return
        stamp = datetime.now()
        self.v2_session_dir = ensure_directory(
            dated_directory(FLIGHT_ACCEPTANCE_CALIBRATION_DIR, stamp) /
            ("v2a-" + stamp.strftime("%Y%m%d-%H%M%S")))
        (self.v2_session_dir / "session.json").write_text(
            json.dumps({
                "format": "drone-h743-v2a-live-session", "schema": 1,
                "created_at": stamp.astimezone().isoformat(),
                "props_removed": True, "power_safe_confirmed": True,
                "evidence_only": True, "flight_release": False,
                "raw_log": "acceptance_raw.log",
            }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.v2_status_var.set("正在等待目标端确认 ESC CCR=0 和 500 ms 租约…")

    def _v2_set_stage(self) -> None:
        if not self.v2_active or self.v2_lease_id == 0:
            messagebox.showwarning("V2A", "请先启动安全模式")
            return
        self.transport.send_line(
            f"ACCEPT V2 STAGE name={self.v2_stage_var.get()} lease={self.v2_lease_id}")

    def _v2_stop(self) -> None:
        if self.v2_lease_id:
            self.transport.send_line(f"ACCEPT V2 STOP lease={self.v2_lease_id}")
        self.v2_active = False; self.v2_lease_id = 0
        self.v2_status_var.set("V2A 已停止；目标端保持 ESC 禁用")

    def _v2_tick(self) -> None:
        if self.v2_active and self.v2_lease_id:
            if self.transport.is_connected:
                self.transport.send_line(
                    f"ACCEPT V2 KEEPALIVE lease={self.v2_lease_id}")
                self.v2_tick_count += 1
                if (self.v2_tick_count % 4) == 0:
                    self.transport.send_line("ACCEPT?")
            else:
                self.v2_active = False; self.v2_lease_id = 0
                self.v2_status_var.set("连接已断开；500 ms 租约将自动退出并保持 ESC 禁用")
        self.after(250, self._v2_tick)

    def _v2_handle_line(self, line: str) -> None:
        if self.v2_session_dir is not None:
            try:
                with (self.v2_session_dir / "acceptance_raw.log").open(
                    "a", encoding="utf-8") as stream:
                    stream.write(f"{datetime.now().astimezone().isoformat()} {line}\n")
            except OSError as exc:
                self.v2_status_var.set(f"V2A 日志写入失败：{exc}")
        values = parse_kv(line)
        if values.get("event") == "started":
            lease = safe_int(values.get("lease", "0"), 0)
            if lease > 0 and values.get("esc") == "0,0":
                self.v2_lease_id = lease; self.v2_active = True
                self.v2_status_var.set(
                    f"V2A 已启动：lease={lease}，ESC CCR=0；自动续租；证据={self.v2_session_dir}")
        elif values.get("event") == "stopped":
            self.v2_active = False; self.v2_lease_id = 0
            self.v2_status_var.set("V2A 已停止")
        elif "active" in values:
            if values.get("active") != "1" and self.v2_active:
                self.v2_active = False; self.v2_lease_id = 0
                self.v2_status_var.set("V2A 租约已失效，目标端已自动退出")
            else:
                self.v2_status_var.set(
                    f"V2A active={values.get('active')} stage={values.get('stage')} "
                    f"lease={values.get('lease')} sample={values.get('sample')} seq={values.get('seq')}")
        if line.startswith(("ACCEPT context", "ACCEPT motion", "ACCEPT control", "ACCEPT servo")):
            self.v2_live_var.set(line)

    def _v1_selected_plan(self):
        return self.v1_stage_by_label[self.v1_stage_var.get()]

    def _v1_refresh_controls(self) -> None:
        if not hasattr(self, "v1_start_button"):
            return
        gate_ok, _reason = self._firmware_snapshot_gate()
        safe = gate_ok and self.validation_props_removed_var.get() and self.validation_power_safe_var.get()
        busy = self.v1_worker is not None and self.v1_worker.is_alive()
        self.v1_start_button.configure(state=tk.NORMAL if safe and self.v1_session is not None and not busy else tk.DISABLED)
        self.v1_finish_button.configure(state=tk.NORMAL if busy else tk.DISABLED)
        self.v1_analyze_button.configure(state=tk.NORMAL if self.v1_session is not None and bool(self.v1_session.captures) and not busy else tk.DISABLED)
        candidate_pass = bool(
            self.v1_analysis_summary is not None
            and self.v1_analysis_summary.accelerometer_status is MetrologyStatus.PASS
            and self.v1_analysis_summary.gyro_static_status is MetrologyStatus.PASS
        )
        self.v1_apply_button.configure(
            state=tk.NORMAL if safe and candidate_pass and not busy and not self.v1_candidate_applied else tk.DISABLED)
        self.v1_revert_button.configure(
            state=tk.NORMAL if safe and self.v1_candidate_applied and not busy else tk.DISABLED)
        self.v1_commit_button.configure(
            state=tk.NORMAL if safe and self.v1_candidate_applied and not busy else tk.DISABLED)

    def _v1_new_session(self) -> None:
        if self.v1_worker is not None and self.v1_worker.is_alive():
            return
        self.v1_session, self.v1_manifest_path = new_v1_session()
        self.v1_analysis_summary = None
        self.v1_encoded_candidate = None
        self.v1_candidate_applied = False
        self.v1_status_var.set("V1 会话已建立：选择姿态步骤，然后开始采集")
        self._v1_render_session()

    def _v1_load_latest_session(self) -> None:
        path = latest_session_manifest()
        if path is None:
            return
        try:
            self.v1_session = load_v1_session(path); self.v1_manifest_path = path
            self.v1_status_var.set("已恢复最近一次 V1 会话：可继续未完成的采集步骤")
            self._v1_render_session()
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            self.v1_status_var.set(f"V1 会话恢复失败：{exc}")

    def _v1_render_session(self) -> None:
        if self.v1_session is None:
            return
        for item in self.v1_capture_tree.get_children(): self.v1_capture_tree.delete(item)
        for index, record in enumerate(self.v1_session.captures):
            self.v1_capture_tree.insert("", tk.END, iid=f"v1-{index}", values=(record.stage, record.temperature_platform or "-", record.sample_count, record.csv_path))
        self.v1_counts_var.set(f"session={self.v1_session.session_id} · captures={len(self.v1_session.captures)} · manifest={self.v1_manifest_path}")
        self._v1_refresh_controls()

    def _v1_start_capture(self) -> None:
        if self.v1_session is None or self.v1_manifest_path is None:
            self._v1_new_session()
        gate_ok, reason = self._firmware_snapshot_gate()
        if not gate_ok or not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("V1 安全门未通过", reason if not gate_ok else "必须拆桨并隔离动力")
            return
        port = self.serial_transport.active_port
        if not port:
            messagebox.showwarning("V1 USB CDC", "没有当前 active USB CDC")
            return
        plan = self._v1_selected_plan()
        platform = self.v1_temperature_platform_var.get().strip() if plan.temperature_platform_required else None
        if plan.temperature_platform_required and not platform:
            messagebox.showwarning("温度平台", "请输入明确的平台标签，例如 room/cold/warm")
            return
        self.v1_finish_event = threading.Event(); self.v1_cancel_event = threading.Event(); self.v1_progress.configure(value=0)
        baud = int(self.serial_baud_var.get())
        self.serial_transport.stop(); self._refresh_serial_selection_lock()
        self._serial_port_combo.configure(state=tk.DISABLED)
        self._serial_refresh_button.configure(state=tk.DISABLED)
        self.v1_status_var.set(f"V1 采集中：{plan.label}；USB CDC 已由面板安全让渡给 IMUCAP")
        self.v1_worker = threading.Thread(target=self._v1_capture_worker, args=(port, baud, plan, platform), daemon=True)
        self.v1_worker.start(); self._v1_refresh_controls()

    def _v1_finish_capture(self) -> None:
        self.v1_finish_event.set(); self.v1_status_var.set("正在 STOP/DUMP 并保存原始证据…")

    def _v1_capture_worker(self, port: str, baud: int, plan, platform: str | None) -> None:
        link = None
        try:
            time.sleep(0.25); link = CaptureLink(port, baud=baud, timeout=0.25)
            link.send(f"IMUCAP START {plan.requested_samples}")
            deadline = time.monotonic() + 2.0
            started = False
            while time.monotonic() < deadline:
                line = link.read_text_line(deadline)
                if line and "IMUCAP START" in line and "ok" in line.lower():
                    started = True
                    break
            if not started:
                raise RuntimeError("IMUCAP START did not return an explicit ok")
            self.v1_event_queue.put(("status", f"IMUCAP recording {plan.label}"))
            capture_deadline = time.monotonic() + plan.maximum_samples / 1000.0 + 0.75
            while time.monotonic() < capture_deadline and not self.v1_finish_event.is_set() and not self.v1_cancel_event.is_set():
                self.v1_cancel_event.wait(0.1)
            if self.v1_cancel_event.is_set(): raise RuntimeError("V1 capture cancelled")
            link.send("IMUCAP STOP")
            ready_deadline = time.monotonic() + 2.0
            capture_ready = False
            while time.monotonic() < ready_deadline:
                link.send("IMUCAP?")
                line = link.read_text_line(min(ready_deadline, time.monotonic() + 0.25))
                if line and "IMUCAP" in line and "state=ready" in line:
                    capture_ready = True
                    break
            if not capture_ready:
                raise RuntimeError("IMUCAP did not finish timestamp-matched annotations")
            link.send("IMUCAP DUMP")
            samples, header = link.read_blocks(timeout_s=45.0, verbose=False, progress=lambda text, fraction: self.v1_event_queue.put(("progress", (text, fraction))))
            assert self.v1_session is not None and self.v1_manifest_path is not None
            updated, record = persist_v1_capture(self.v1_session, self.v1_manifest_path, stage=plan.stage, samples=samples, header=header, temperature_platform=platform)
            self.v1_event_queue.put(("capture", (updated, record)))
        except Exception as exc:
            self.v1_event_queue.put(("error", f"V1 采集失败：{exc}"))
        finally:
            if link is not None:
                try: link.close()
                except Exception: pass
            self.v1_event_queue.put(("reconnect", (port, baud)))

    def _v1_start_analysis(self) -> None:
        if self.v1_session is None or self.v1_manifest_path is None: return
        self.v1_status_var.set("正在离线分析 V1 原始证据…")
        self.v1_worker = threading.Thread(target=self._v1_analysis_worker, args=(self.v1_session, self.v1_manifest_path), daemon=True)
        self.v1_worker.start(); self._v1_refresh_controls()

    def _v1_analysis_worker(self, session: V1Session, manifest: Path) -> None:
        try: self.v1_event_queue.put(("analysis", analyze_session(session, manifest)))
        except Exception as exc: self.v1_event_queue.put(("error", f"V1 分析失败：{exc}"))

    def _v1_begin_target_action(self, action: str) -> None:
        if self.v1_session is None or self.v1_manifest_path is None:
            return
        gate_ok, reason = self._firmware_snapshot_gate()
        if not gate_ok or not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("V1 安全门未通过", reason if not gate_ok else "必须拆桨并隔离动力")
            return
        port = self.serial_transport.active_port
        if not port:
            messagebox.showwarning("V1 USB CDC", "没有当前 active USB CDC")
            return
        try:
            if action == "apply":
                if self.v1_analysis_summary is None:
                    raise ValueError("请先完成步骤 4 离线分析")
                samples = load_v1_session_samples(self.v1_session, self.v1_manifest_path)
                encoded = load_and_encode_v1_candidate(
                    self.v1_analysis_summary.candidate_path, samples)
            else:
                if not self.v1_candidate_applied or self.v1_encoded_candidate is None:
                    raise ValueError("当前软件没有可确认的 RAM 候选")
                encoded = self.v1_encoded_candidate
        except (OSError, ValueError, TypeError) as exc:
            messagebox.showerror("V1 候选不允许应用", str(exc))
            return
        if action == "commit" and not messagebox.askyesno(
            "写入参数 Flash",
            "确认 RAM 候选的姿态、静止输出和方向均已二次验证？\n\n"
            "此操作只写外部参数双槽，不烧录程序固件。",
        ):
            return
        baud = int(self.serial_baud_var.get())
        self.serial_transport.stop(); self._refresh_serial_selection_lock()
        self._serial_port_combo.configure(state=tk.DISABLED)
        self._serial_refresh_button.configure(state=tk.DISABLED)
        self.v1_status_var.set({
            "apply": "正在重放证据并上传基础/完整候选到 RAM…",
            "revert": "正在撤销 RAM 候选…",
            "commit": "正在确认新代次并写入参数 Flash 双槽…",
        }[action])
        self.v1_worker = threading.Thread(
            target=self._v1_target_worker,
            args=(port, baud, action, encoded), daemon=True)
        self.v1_worker.start(); self._v1_refresh_controls()

    def _v1_apply_candidate(self) -> None:
        self._v1_begin_target_action("apply")

    def _v1_revert_candidate(self) -> None:
        self._v1_begin_target_action("revert")

    def _v1_commit_candidate(self) -> None:
        self._v1_begin_target_action("commit")

    def _v1_target_worker(self, port: str, baud: int, action: str,
                          encoded: EncodedV1Candidate) -> None:
        link = None
        try:
            time.sleep(0.25); link = CaptureLink(port, baud=baud, timeout=0.25)
            if action == "apply":
                result = imucal_upload_and_apply(link, encoded)
            elif action == "revert":
                result = imucal_revert_candidate(link)
            elif action == "commit":
                result = imucal_commit_candidate(link)
            else:
                raise ValueError(f"unknown V1 target action {action}")
            assert self.v1_manifest_path is not None
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            record_path = write_imucal_transaction_record(
                self.v1_manifest_path.parent / f"imucal_{action}_{stamp}.json",
                encoded=encoded, result=result)
            self.v1_event_queue.put(("target", (action, encoded, record_path)))
        except Exception as exc:
            self.v1_event_queue.put(("error", f"V1 {action} 失败：{exc}"))
        finally:
            if link is not None:
                try: link.close()
                except Exception: pass
            self.v1_event_queue.put(("reconnect", (port, baud)))

    def _v1_drain_events(self) -> None:
        try:
            while True:
                kind, payload = self.v1_event_queue.get_nowait()
                if kind == "status": self.v1_status_var.set(str(payload))
                elif kind == "progress":
                    text_value, fraction = payload; self.v1_status_var.set(str(text_value)); self.v1_progress.configure(value=float(fraction) * 100.0)
                elif kind == "capture":
                    self.v1_session, _record = payload; self.v1_progress.configure(value=100.0); self.v1_status_var.set("本步原始证据已保存：可继续下一步或执行离线分析"); self._v1_render_session()
                elif kind == "analysis":
                    summary = payload; self.v1_analysis_summary = summary; self.v1_encoded_candidate = None; self.v1_candidate_applied = False; self.v1_status_var.set(summary.status_line); self.v1_analysis_var.set(f"accel={summary.accelerometer_status.value} · gyro static={summary.gyro_static_status.value} · gyro +360={summary.gyro_rotation_status.value} · temperature={summary.temperature_status.value} · overall={summary.overall_status.value} · candidate={summary.candidate_path}")
                elif kind == "target":
                    action, encoded, record_path = payload
                    if action == "apply":
                        self.v1_encoded_candidate = encoded; self.v1_candidate_applied = True
                        self.v1_status_var.set(f"RAM 候选已应用且电机保持锁定；请二次验证。记录：{record_path}")
                    elif action == "revert":
                        self.v1_encoded_candidate = None; self.v1_candidate_applied = False
                        self.v1_status_var.set(f"RAM 候选已撤销，已恢复 Flash 确认参数。记录：{record_path}")
                    else:
                        self.v1_encoded_candidate = None; self.v1_candidate_applied = False
                        self.v1_status_var.set(f"V1 参数已写入并由 dirty=0 确认。记录：{record_path}")
                elif kind == "error": self.v1_status_var.set(str(payload)); messagebox.showerror("V1", str(payload))
                elif kind == "reconnect":
                    port, baud = payload
                    if self.transport is self.serial_transport:
                        self.serial_transport.start(port, baud); self._refresh_serial_selection_lock()
                if kind in {"capture", "analysis", "target", "error"}: self.v1_worker = None
        except queue.Empty: pass
        self._v1_refresh_controls(); self.after(100, self._v1_drain_events)

    def _build_firmware_update_page(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="维护工具  /  安全升级与自动重连", style="Eyebrow.TLabel").pack(anchor=tk.W)
        header = ttk.Frame(parent)
        header.pack(fill=tk.X)
        ttk.Label(header, text="USB 一键固件升级", style="PageTitle.TLabel").pack(side=tk.LEFT)
        ttk.Label(
            header,
            text="STM32 ROM DFU · 不依赖自定义 Bootloader",
            style="Muted.TLabel",
        ).pack(side=tk.RIGHT)

        ttk.Label(
            parent,
            text=(
                "同一根 USB 线完成：应用 CDC 授权进入 ROM DFU → Windows 枚举 STM32 BOOTLOADER → "
                "CubeProgrammer 写入、校验、启动应用并自动重连 COM。只接受 ELF/HEX，失败不会自动重试擦写。"
            ),
            wraplength=1120,
            style="Muted.TLabel",
        ).pack(fill=tk.X, pady=(4, 10))
        ttk.Frame(parent, height=1, style="Rule.TFrame").pack(fill=tk.X, pady=(0, 8))

        files = ttk.LabelFrame(parent, text="1 · 固件与烧录工具", padding=10)
        files.pack(fill=tk.X)
        files.columnconfigure(1, weight=1)
        ttk.Label(files, text="固件").grid(row=0, column=0, sticky=tk.W, padx=(0, 8), pady=3)
        ttk.Entry(files, textvariable=self.firmware_image_var).grid(
            row=0, column=1, sticky=tk.EW, pady=3
        )
        ttk.Button(files, text="选择 ELF/HEX", command=self._firmware_choose_image,
                   style="Secondary.TButton").grid(
            row=0, column=2, padx=(8, 0), pady=3
        )
        ttk.Label(files, text="CubeProgrammer CLI").grid(
            row=1, column=0, sticky=tk.W, padx=(0, 8), pady=3
        )
        ttk.Entry(files, textvariable=self.firmware_cli_var).grid(
            row=1, column=1, sticky=tk.EW, pady=3
        )
        ttk.Button(files, text="自动查找", command=self._firmware_find_cli,
                   style="Secondary.TButton").grid(
            row=1, column=2, padx=(8, 0), pady=3
        )
        ttk.Checkbutton(
            files,
            text="烧录前重新编译（cmake --build --preset Debug）",
            variable=self.firmware_rebuild_var,
            command=self._firmware_refresh_image_info,
        ).grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(5, 0))
        self.firmware_staleness_label = ttk.Label(
            files,
            textvariable=self.firmware_staleness_var,
            style="Idle.TLabel",
            wraplength=1000,
        )
        self.firmware_staleness_label.grid(
            row=3, column=0, columnspan=3, sticky=tk.W, pady=(2, 0))
        ttk.Label(
            files,
            textvariable=self.firmware_image_info_var,
            style="Muted.TLabel",
            wraplength=1000,
        ).grid(row=4, column=0, columnspan=3, sticky=tk.W, pady=(5, 0))

        safety = ttk.LabelFrame(parent, text="2 · 实时安全门", padding=10)
        safety.pack(fill=tk.X, pady=(9, 0))
        ttk.Label(
            safety,
            textvariable=self.firmware_safety_var,
            style="SectionTitle.TLabel",
        ).pack(side=tk.LEFT)
        ttk.Button(
            safety,
            text="立即读取安全快照",
            command=self._firmware_request_snapshot,
            style="Primary.TButton",
        ).pack(side=tk.RIGHT)
        ttk.Checkbutton(
            safety,
            text="我确认当前未知 USB 串口就是飞控 application CDC",
            variable=self.firmware_unknown_usb_override_var,
            command=self._firmware_refresh_safety,
        ).pack(side=tk.RIGHT, padx=(0, 12))

        action = ttk.LabelFrame(parent, text="3 · 进入 DFU 并烧录", padding=10)
        action.pack(fill=tk.X, pady=(9, 0))
        controls = ttk.Frame(action)
        controls.pack(fill=tk.X)
        self.firmware_start_button = ttk.Button(
            controls,
            text="进入 DFU 并烧录",
            command=self._firmware_start_update,
            state=tk.DISABLED,
            style="Warning.TButton",
        )
        self.firmware_start_button.pack(side=tk.LEFT)
        self.firmware_cancel_button = ttk.Button(
            controls,
            text="取消",
            command=self._firmware_cancel_update,
            state=tk.DISABLED,
            style="Danger.TButton",
        )
        self.firmware_cancel_button.pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(
            controls,
            text="打开升级日志目录",
            command=self._firmware_open_log_dir,
            style="Secondary.TButton",
        ).pack(side=tk.LEFT, padx=(18, 0))
        ttk.Label(
            action,
            textvariable=self.firmware_status_var,
            wraplength=1080,
            style="Guide.TLabel",
        ).pack(fill=tk.X, pady=(9, 0))
        self.firmware_progress = ttk.Progressbar(
            action,
            mode="determinate",
            maximum=100.0,
            variable=self.firmware_progress_var,
        )
        self.firmware_progress.pack(fill=tk.X, pady=(9, 0))

        log_box = ttk.LabelFrame(parent, text="升级进度与 CubeProgrammer 输出", padding=8)
        log_box.pack(fill=tk.BOTH, expand=True, pady=(9, 0))
        log_frame = ttk.Frame(log_box)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.firmware_log_text = tk.Text(
            log_frame,
            height=16,
            wrap=tk.WORD,
            font=("Consolas", 9),
            state=tk.DISABLED,
            background=UI_PALETTE["console"],
            foreground=UI_PALETTE["ink_dim"],
            insertbackground=UI_PALETTE["accent"],
            selectbackground=UI_PALETTE["accent_soft"],
            relief=tk.FLAT,
            padx=10,
            pady=8,
        )
        log_scroll = ttk.Scrollbar(
            log_frame,
            orient=tk.VERTICAL,
            command=self.firmware_log_text.yview,
        )
        self.firmware_log_text.configure(yscrollcommand=log_scroll.set)
        self.firmware_log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        ttk.Label(
            parent,
            text="恢复入口：" + FIRMWARE_RECOVERY_HINT,
            style="Warn.TLabel",
            wraplength=1120,
        ).pack(fill=tk.X, pady=(7, 0))
        self._firmware_refresh_image_info()
        self._firmware_refresh_safety()

    def _firmware_choose_image(self) -> None:
        current = Path(self.firmware_image_var.get().strip() or PROJECT_ROOT)
        selected = filedialog.askopenfilename(
            title="选择要烧录的 ELF/HEX 固件",
            initialdir=str(current.parent if current.suffix else current),
            filetypes=(("STM32 firmware", "*.elf *.hex"), ("ELF", "*.elf"), ("HEX", "*.hex")),
        )
        if selected:
            self.firmware_image_var.set(selected)
            self._firmware_refresh_image_info()

    def _firmware_refresh_image_info(self) -> FirmwareImageInfo | None:
        self._firmware_refresh_staleness()
        try:
            info = validate_firmware_image(self.firmware_image_var.get().strip())
        except RomDfuError as exc:
            self.firmware_image_info_var.set(f"固件镜像：✗ {exc}")
            return None
        self.firmware_image_info_var.set("固件镜像：✓ " + info.summary)
        return info

    def _firmware_refresh_staleness(self) -> None:
        """把"这个 ELF 比源码旧"直接摆在烧录按钮上方。

        自动重编译是主要保障，但用户可能关掉它；关掉之后必须仍然看得见风险，
        否则又会把上一次的旧固件刷进飞控。
        """
        label = getattr(self, "firmware_staleness_label", None)
        image = self.firmware_image_var.get().strip()
        if self.firmware_rebuild_var.get():
            self.firmware_staleness_var.set("烧录前会先执行一次编译，产物即为将要烧录的镜像。")
            if label is not None:
                label.configure(style="Idle.TLabel")
            return
        if not image:
            self.firmware_staleness_var.set("尚未选择固件镜像。")
            if label is not None:
                label.configure(style="Idle.TLabel")
            return
        stale, reason = firmware_image_staleness(image, PROJECT_ROOT)
        if stale:
            self.firmware_staleness_var.set(f"自动编译已关闭，且镜像可能是旧的：{reason}")
        else:
            self.firmware_staleness_var.set(f"自动编译已关闭；{reason}。")
        if label is not None:
            label.configure(style="Fail.TLabel" if stale else "Pass.TLabel")

    def _firmware_find_cli(self) -> None:
        try:
            cli = resolve_cubeprogrammer_cli()
        except RomDfuError as exc:
            self.firmware_status_var.set(str(exc))
            messagebox.showerror("未找到 CubeProgrammer", str(exc))
            return
        self.firmware_cli_var.set(str(cli))
        self.firmware_status_var.set(f"已找到 CubeProgrammer：{cli}")

    def _firmware_request_snapshot(self) -> None:
        if self.transport is not self.serial_transport or not self._transport_connected():
            messagebox.showwarning("USB CDC 未连接", "请在顶部选择 serial、选中飞控 USB CDC 并启动连接。")
            self._firmware_refresh_safety()
            return
        self._send_proto_once(PROTO_REQ_IMU, "IMU?")
        self.firmware_status_var.set("正在读取实时安全快照…")

    def _firmware_snapshot_gate(self) -> tuple[bool, str]:
        age_s = (
            time.monotonic() - self.validation_latest_host_time
            if self.validation_latest_host_time > 0.0
            else float("inf")
        )
        safe, reason = firmware_update_snapshot_gate(
            self.validation_latest_values,
            connected=self._transport_connected(),
            serial_transport_selected=self.transport is self.serial_transport,
            sample_age_s=age_s,
        )
        if not safe:
            return safe, reason
        selected_display = self.serial_port_var.get().strip()
        selected_port = self._serial_port_map.get(selected_display, selected_display)
        active_port = self.serial_transport.active_port
        if not active_port or selected_port.casefold() != active_port.casefold():
            return False, f"连接端口与下拉选择不一致：active={active_port or '-'} selected={selected_port or '-'}"
        if self.validation_latest_transport_generation != self.serial_transport.connection_generation:
            return False, "安全快照不属于当前 USB CDC 连接，请重新读取"
        identity = self._serial_port_identity.get(active_port)
        policy, identity_reason = serial_device_identity_policy(identity)
        if policy == "rejected":
            return False, identity_reason
        if policy == "unknown" and not self.firmware_unknown_usb_override_var.get():
            return False, identity_reason + "；必须在升级页人工确认未知 USB 身份"
        suffix = "（未知身份已人工确认）" if policy == "unknown" else ""
        return True, reason + f" · {identity_reason}{suffix}"

    def _firmware_refresh_safety(self) -> None:
        safe, reason = self._firmware_snapshot_gate()
        self.firmware_safety_var.set(("安全门：✓ " if safe else "安全门：✗ ") + reason)
        busy = (
            self.firmware_update_pending
            or self.firmware_update_running
            or self.firmware_building
        )
        if hasattr(self, "firmware_start_button"):
            self.firmware_start_button.configure(
                state=tk.NORMAL if safe and not busy else tk.DISABLED
            )
            self.firmware_cancel_button.configure(
                state=tk.NORMAL if busy and not self.firmware_programming else tk.DISABLED
            )

    def _firmware_clear_ui_log(self) -> None:
        self.firmware_log_text.configure(state=tk.NORMAL)
        self.firmware_log_text.delete("1.0", tk.END)
        self.firmware_log_text.configure(state=tk.DISABLED)

    def _firmware_append_ui_log(self, line: str) -> None:
        self.firmware_log_text.configure(state=tk.NORMAL)
        self.firmware_log_text.insert(tk.END, line.rstrip("\r\n") + "\n")
        self.firmware_log_text.see(tk.END)
        self.firmware_log_text.configure(state=tk.DISABLED)

    def _firmware_new_log_path(self, image: Path) -> Path:
        directory = ensure_directory(dated_directory(FIRMWARE_UPDATE_DIR))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", image.stem).strip("_") or "firmware"
        return directory / f"rom_dfu_{stamp}_{stem}.log"

    def _firmware_record_log(self, attempt: int, line: str) -> None:
        if attempt != self.firmware_attempt_id:
            return
        path = self.firmware_log_path
        if path is not None:
            stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
            try:
                with path.open("a", encoding="utf-8", newline="\n") as stream:
                    stream.write(f"[{stamp}] {line.rstrip()}\n")
            except OSError as exc:
                if not self.firmware_log_write_warning_sent:
                    self.firmware_log_write_warning_sent = True
                    self.firmware_event_queue.put(("log_write_error", attempt, str(exc)))
        self.firmware_event_queue.put(("log", attempt, line))

    def _firmware_send_boot_command(self) -> bool:
        command = FIRMWARE_BOOT_COMMAND
        self.firmware_authorized_commands.add(command)
        try:
            if not self._validation_guard_command(command):
                return False
            self.last_cmd_var.set(f"最近命令: {command}")
            self._append(f"> {command}")
            return self.transport.send_line(command)
        finally:
            # A one-shot grant: the diagnostics box cannot reuse it while the
            # target is preparing to detach USB CDC.
            self.firmware_authorized_commands.discard(command)

    def _firmware_start_update(self) -> None:
        if self.firmware_update_pending or self.firmware_update_running:
            return
        if self.firmware_building:
            return
        safe, reason = self._firmware_snapshot_gate()
        if not safe:
            messagebox.showwarning("固件升级安全门未通过", reason)
            self._firmware_refresh_safety()
            return
        if self.firmware_rebuild_var.get():
            # 先编译再烧：编译产物就是烧录对象，从源头杜绝"刷了旧固件"。
            self.firmware_building = True
            self.firmware_status_var.set("正在重新编译固件（cmake --build --preset Debug）…")
            self._firmware_clear_ui_log()
            self._firmware_refresh_safety()
            threading.Thread(
                target=self._firmware_build_worker,
                args=(self.firmware_attempt_id,),
                daemon=True,
            ).start()
            return
        self._firmware_start_update_after_build()

    def _firmware_build_worker(self, attempt: int) -> None:
        try:
            elf = build_firmware(
                PROJECT_ROOT,
                on_output=lambda line: self.firmware_event_queue.put(("log", attempt, line)),
            )
        except RomDfuError as exc:
            self.firmware_event_queue.put(("build_failed", attempt, str(exc)))
            return
        except Exception as exc:  # noqa: BLE001 - 构建异常绝不能静默变成"继续烧录"
            self.firmware_event_queue.put(("build_failed", attempt, repr(exc)))
            return
        self.firmware_event_queue.put(("build_ok", attempt, str(elf)))

    def _firmware_start_update_after_build(self) -> None:
        if self.firmware_update_pending or self.firmware_update_running:
            return
        safe, reason = self._firmware_snapshot_gate()
        if not safe:
            messagebox.showwarning("固件升级安全门未通过", reason)
            self._firmware_refresh_safety()
            return
        try:
            image_info = validate_firmware_image(self.firmware_image_var.get().strip())
            image = image_info.path
            cli = resolve_cubeprogrammer_cli(self.firmware_cli_var.get().strip() or None)
        except RomDfuError as exc:
            self.firmware_status_var.set(str(exc))
            messagebox.showerror("固件升级预检失败", str(exc))
            return
        if not messagebox.askyesno(
            "确认进入 ROM DFU",
            (
                f"将通过当前 USB CDC 向飞控发送唯一授权命令：\n{FIRMWARE_BOOT_COMMAND}\n\n"
                f"随后烧录并校验：\n{image}\n\n"
                "过程中不要拔 USB。确定继续吗？"
            ),
        ):
            return

        self.firmware_attempt_id += 1
        attempt = self.firmware_attempt_id
        self.firmware_cancel_event = threading.Event()
        self.firmware_selected_cli = cli
        try:
            self.firmware_log_path = self._firmware_new_log_path(image)
            self.firmware_log_path.write_text("", encoding="utf-8")
            self.firmware_log_write_warning_sent = False
            frozen_path = self.firmware_log_path.with_name(
                self.firmware_log_path.stem + "_image" + image.suffix.lower()
            )
            frozen_info = freeze_firmware_image(image_info, frozen_path)
        except OSError as exc:
            self.firmware_log_path = None
            self.firmware_status_var.set(f"无法创建升级日志：{exc}")
            messagebox.showerror("固件升级预检失败", f"无法创建升级日志：{exc}")
            return
        except RomDfuError as exc:
            self.firmware_status_var.set(f"无法冻结升级镜像：{exc}")
            messagebox.showerror("固件升级预检失败", f"无法冻结升级镜像：{exc}")
            return
        self.firmware_selected_original_image = image
        self.firmware_selected_original_image_info = image_info
        self.firmware_selected_image = frozen_info.path
        self.firmware_selected_image_info = frozen_info
        self._firmware_clear_ui_log()
        self.firmware_progress_var.set(0.0)
        self.firmware_update_pending = True
        self.firmware_status_var.set("安全预检：确认当前没有既存 USB DFU 设备…")
        if self.validation_session_active:
            self._validation_stop_session()
        self.validation_poll_requested = False
        self.transport.cancel_pending_sends()
        self._firmware_record_log(
            attempt, f"original_image={image} original_sha256={image_info.sha256}"
        )
        self._firmware_record_log(
            attempt,
            f"frozen_image={frozen_info.path} frozen_sha256={frozen_info.sha256} "
            f"image_info={frozen_info.summary}",
        )
        self._firmware_record_log(attempt, f"cubeprogrammer={cli}")
        active_port = self.serial_transport.active_port or ""
        identity = self._serial_port_identity.get(active_port)
        self.firmware_reconnect_port = active_port
        self.firmware_reconnect_identity = dict(identity) if identity is not None else None
        try:
            self.firmware_reconnect_baud = int(self.serial_baud_var.get())
        except (tk.TclError, ValueError):
            self.firmware_reconnect_baud = 115200
        self._firmware_record_log(
            attempt,
            f"usb_identity={json.dumps(identity, ensure_ascii=False, sort_keys=True)} "
            f"unknown_override={int(self.firmware_unknown_usb_override_var.get())} "
            f"connection_generation={self.serial_transport.connection_generation}",
        )
        self._firmware_record_log(attempt, "baseline: list USB DFU before sending BOOT")
        self.firmware_worker = threading.Thread(
            target=self._firmware_check_baseline_worker,
            args=(attempt, cli),
            daemon=True,
        )
        self.firmware_worker.start()
        self._firmware_refresh_safety()

    def _firmware_check_baseline_worker(self, attempt: int, cli: Path) -> None:
        try:
            ports = list_usb_dfu_ports(cli)
        except RomDfuError as exc:
            self._firmware_record_log(attempt, f"ERROR: baseline DFU enumeration failed: {exc}")
            self.firmware_event_queue.put(("preflight_error", attempt, str(exc)))
            return
        if ports:
            detail = ", ".join(ports)
            self._firmware_record_log(
                attempt,
                f"REFUSED: pre-existing DFU device(s) before BOOT: {detail}",
            )
            self.firmware_event_queue.put(("baseline_conflict", attempt, detail))
            return
        self._firmware_record_log(attempt, "baseline clear: no USB DFU device present")
        self.firmware_event_queue.put(("baseline_clear", attempt, ""))

    def _firmware_send_boot_after_baseline(self, attempt: int) -> None:
        if (
            attempt != self.firmware_attempt_id
            or not self.firmware_update_pending
            or self.firmware_cancel_event.is_set()
        ):
            return
        safe, reason = self._firmware_snapshot_gate()
        if not safe:
            self.firmware_update_pending = False
            self._firmware_record_log(attempt, f"REFUSED: safety snapshot changed: {reason}")
            self.firmware_status_var.set(f"发送 BOOT 前安全门失效：{reason}")
            self._firmware_refresh_safety()
            messagebox.showerror("固件升级安全门失效", reason)
            return
        self.firmware_status_var.set("基线无 DFU；已授权命令，等待飞控确认进入 DFU…")
        self._firmware_record_log(attempt, f"tx={FIRMWARE_BOOT_COMMAND}")
        if not self._firmware_send_boot_command():
            self.firmware_update_pending = False
            self._firmware_record_log(attempt, "ERROR: BOOT DFU authorization command send failed")
            self.firmware_status_var.set("发送 BOOT DFU 授权命令失败；未开始烧录")
            messagebox.showerror("无法进入 DFU", "USB CDC 命令发送失败。" + FIRMWARE_RECOVERY_HINT)
            self._firmware_refresh_safety()
            return
        self.after(
            FIRMWARE_BOOT_REPLY_TIMEOUT_MS,
            lambda current=attempt: self._firmware_boot_reply_timeout(current),
        )
        self._firmware_refresh_safety()

    def _firmware_boot_reply_timeout(self, attempt: int) -> None:
        if attempt != self.firmware_attempt_id or not self.firmware_update_pending:
            return
        self.firmware_update_pending = False
        self.firmware_cancel_event.set()
        self._firmware_record_log(
            attempt,
            "ERROR: expected `BOOT mode=dfu state=scheduled` reply was not received",
        )
        self.firmware_status_var.set("飞控未确认进入 DFU；未调用 CubeProgrammer")
        self._firmware_refresh_safety()
        messagebox.showerror("DFU 授权超时", "没有收到飞控的 scheduled 回包。" + FIRMWARE_RECOVERY_HINT)

    def _firmware_handle_boot_line(self, line: str) -> None:
        self.last_reply_rx = time.monotonic()
        attempt = self.firmware_attempt_id
        values = parse_kv(line)
        state = values.get("state", "").lower()
        if self.firmware_update_running and state in {"cancelled", "refused"}:
            reason = values.get("reason", "target_unspecified")
            self.firmware_target_abort_reason = f"飞控{state} ROM DFU：{reason}"
            self._firmware_record_log(attempt, f"TARGET_ABORT: state={state} reason={reason}")
            self.firmware_status_var.set(self.firmware_target_abort_reason)
            self.firmware_cancel_event.set()
            return
        if not self.firmware_update_pending:
            return
        self._firmware_record_log(attempt, f"rx={line}")
        if values.get("mode", "").lower() != "dfu" or values.get("state", "").lower() != "scheduled":
            self.firmware_update_pending = False
            self.firmware_cancel_event.set()
            self.firmware_status_var.set(f"飞控拒绝进入 DFU：{line}")
            self._firmware_record_log(attempt, "ERROR: target did not schedule ROM DFU")
            self._firmware_record_log(attempt, "terminal success=0 returncode=n/a duration_s=0 timed_out=0")
            self._firmware_refresh_safety()
            messagebox.showerror("飞控拒绝升级", f"目标回复：{line}\n\n{FIRMWARE_RECOVERY_HINT}")
            return

        self.firmware_update_pending = False
        self.firmware_update_running = True
        self.firmware_target_abort_reason = ""
        self.firmware_status_var.set("飞控已确认；等待 USB CDC 断开并枚举 DFU…")
        self.firmware_worker = threading.Thread(
            target=self._firmware_update_worker,
            args=(
                attempt,
                self.firmware_selected_cli,
                self.firmware_selected_image,
                self.firmware_selected_image_info.sha256,
                self.firmware_reconnect_port,
                self.firmware_reconnect_identity,
            ),
            daemon=True,
        )
        self.firmware_worker.start()
        self._firmware_refresh_safety()

    def _firmware_update_worker(
        self, attempt: int, cli: Path, image: Path, expected_sha256: str,
        reconnect_port: str, reconnect_identity: dict[str, object] | None,
    ) -> None:
        def emit(line: str) -> None:
            self._firmware_record_log(attempt, line)

        try:
            emit("飞控已安排 ROM DFU，等待 USB CDC 断开…")
            deadline = time.monotonic() + FIRMWARE_CDC_DISCONNECT_TIMEOUT_S
            while self.serial_transport.is_connected and time.monotonic() < deadline:
                if self.firmware_cancel_event.wait(0.1):
                    raise DfuCancelledError("等待 USB CDC 断开时已取消")
            if self.serial_transport.is_connected:
                raise RomDfuError("8 秒内 USB CDC 未断开；为避免刷错设备，本次未开始擦写")
            emit("USB CDC 已断开；开始查找 STM32 BOOTLOADER")
            port = wait_for_usb_dfu(
                cli,
                cancel_event=self.firmware_cancel_event,
                on_log=emit,
            )
            if port != "USB1":
                raise RomDfuError(f"V0 只允许唯一设备 USB1，实际枚举为 {port}")
            emit(f"执行一次写入/校验/启动应用：port=USB1 image={image}")
            self.firmware_programming = True
            self.firmware_event_queue.put(("phase", attempt, "programming"))
            result = flash_firmware(
                cli,
                image,
                port="USB1",
                expected_sha256=expected_sha256,
                cancel_event=self.firmware_cancel_event,
                on_output=emit,
                on_progress=lambda value: self.firmware_event_queue.put(
                    ("progress", attempt, value)
                ),
            )
            application_port = None
            if result.success or result.download_verified:
                self.firmware_event_queue.put(("phase", attempt, "reconnecting"))
                application_port = wait_for_application_serial(
                    reconnect_port,
                    reconnect_identity,
                    cancel_event=self.firmware_cancel_event,
                    on_log=emit,
                )
            self.firmware_event_queue.put(
                ("finished", attempt, (result, application_port)))
        except DfuCancelledError as exc:
            emit(f"CANCELLED: {exc}")
            self.firmware_event_queue.put(("cancelled", attempt, str(exc)))
        except RomDfuError as exc:
            emit(f"ERROR: {exc}")
            self.firmware_event_queue.put(("error", attempt, str(exc)))
        except Exception as exc:  # pragma: no cover - defensive UI boundary
            emit(f"ERROR: unexpected host failure: {exc}")
            self.firmware_event_queue.put(("error", attempt, f"主机端异常：{exc}"))

    def _firmware_cancel_update(self) -> None:
        if not (self.firmware_update_pending or self.firmware_update_running):
            return
        if self.firmware_programming:
            messagebox.showwarning(
                "正在写入/校验",
                "为避免留下半刷固件，program/verify 阶段不能从界面取消，也不能关闭窗口。请保持 USB 连接。",
            )
            return
        attempt = self.firmware_attempt_id
        self.firmware_cancel_event.set()
        self.transport.cancel_pending_sends()
        if self.firmware_update_pending:
            self.firmware_update_pending = False
            self.firmware_status_var.set("已取消；若 BOOT 命令已经送达，飞控可能停留在 ROM DFU")
            self._firmware_record_log(attempt, "CANCELLED before target scheduled reply")
        else:
            self.firmware_status_var.set("正在取消 CDC/DFU 等待…")
            self._firmware_record_log(attempt, "cancel requested before programming")
        self._firmware_refresh_safety()

    def _firmware_finish_ui(self, status: str, *, error: bool = False) -> None:
        self.firmware_update_pending = False
        self.firmware_update_running = False
        self.firmware_programming = False
        self.firmware_worker = None
        self.firmware_status_var.set(status)
        self._firmware_refresh_safety()
        if error:
            messagebox.showerror("固件升级未完成", status + "\n\n" + FIRMWARE_RECOVERY_HINT)

    def _firmware_auto_reconnect(self, port: str) -> bool:
        if not port:
            return False
        self.transport_var.set("serial")
        names = self._refresh_serial_ports()
        if hasattr(self, "_serial_port_combo"):
            self._serial_port_combo["values"] = names
        label = next(
            (name for name, device in self._serial_port_map.items()
             if str(device).casefold() == port.casefold()),
            None,
        )
        if label is None:
            self._firmware_record_log(
                self.firmware_attempt_id,
                f"auto_reconnect failed: matched device {port} disappeared before open",
            )
            return False
        self.serial_port_var.set(label)
        self.transport = self.serial_transport
        self.serial_transport.start(port, self.firmware_reconnect_baud)
        self._refresh_serial_selection_lock()
        if not self.serial_transport.is_connected:
            self._firmware_record_log(
                self.firmware_attempt_id,
                f"auto_reconnect failed: open {port} returned disconnected",
            )
            return False
        self.structured_protocol_supported = None
        self._firmware_record_log(
            self.firmware_attempt_id,
            f"auto_reconnect success port={port} baud={self.firmware_reconnect_baud}",
        )
        self.firmware_status_var.set(f"升级完成，已自动重连飞控 USB CDC：{port}")
        self.after(300, lambda: self.serial_transport.send_line("PING"))
        self.after(500, lambda: self.serial_transport.send_line("IMU?"))
        return True

    def _firmware_drain_events(self) -> None:
        try:
            while True:
                kind, attempt, payload = self.firmware_event_queue.get_nowait()
                if attempt != self.firmware_attempt_id:
                    continue
                if kind == "log":
                    self._firmware_append_ui_log(str(payload))
                elif kind == "build_ok":
                    self.firmware_building = False
                    # 烧录对象强制切到刚产出的 ELF，避免仍指向旧路径。
                    self.firmware_image_var.set(str(payload))
                    self._firmware_refresh_image_info()
                    self.firmware_status_var.set("编译完成；继续安全预检与烧录…")
                    self._firmware_start_update_after_build()
                elif kind == "build_failed":
                    self.firmware_building = False
                    self.firmware_status_var.set("编译失败，已取消烧录（未向飞控发送任何命令）")
                    self._firmware_refresh_safety()
                    messagebox.showerror(
                        "编译失败，未烧录",
                        f"{payload}\n\n飞控未被触碰，仍在运行原固件。",
                    )
                elif kind == "log_write_error":
                    messagebox.showwarning(
                        "升级日志写入失败",
                        f"升级仍在继续，但日志无法写入：{payload}",
                    )
                elif kind == "baseline_clear":
                    if self.firmware_update_pending:
                        self.firmware_worker = None
                        self._firmware_send_boot_after_baseline(attempt)
                elif kind == "baseline_conflict":
                    if self.firmware_update_pending:
                        self._firmware_finish_ui(
                            f"发现既存 DFU 设备 {payload}；为避免刷错 USB1，未发送 BOOT"
                        )
                        messagebox.showerror(
                            "DFU 基线冲突",
                            f"发送 BOOT 前已经存在 DFU 设备：{payload}\n请断开其它 DFU 设备后重试。",
                        )
                elif kind == "preflight_error":
                    if self.firmware_update_pending:
                        self._firmware_finish_ui(f"无法确认 DFU 基线：{payload}")
                        messagebox.showerror("DFU 基线检查失败", str(payload))
                elif kind == "progress":
                    self.firmware_progress_var.set(float(payload))
                elif kind == "phase":
                    if payload == "programming":
                        self.firmware_status_var.set(
                            "正在 program/verify/start；为避免半刷，取消和关闭窗口已锁定"
                        )
                        self._firmware_refresh_safety()
                    elif payload == "reconnecting":
                        self.firmware_programming = False
                        self.firmware_status_var.set(
                            "写入/校验完成；等待 application CDC 枚举并自动重连…"
                        )
                        self._firmware_refresh_safety()
                elif kind == "cancelled":
                    reason = self.firmware_target_abort_reason or f"升级已取消：{payload}"
                    self._firmware_record_log(
                        attempt,
                        "terminal success=0 returncode=n/a duration_s=n/a timed_out=0 cancelled=1",
                    )
                    self._firmware_finish_ui(reason)
                elif kind == "error":
                    self._firmware_record_log(
                        attempt,
                        "terminal success=0 returncode=n/a duration_s=n/a timed_out=0 cancelled=0",
                    )
                    self._firmware_finish_ui(str(payload), error=True)
                elif kind == "finished":
                    if (isinstance(payload, tuple) and len(payload) == 2):
                        result, application_port = payload
                    else:
                        result, application_port = payload, None
                    if not isinstance(result, FlashResult):
                        self._firmware_finish_ui("主机端返回了无效烧录结果", error=True)
                    elif result.success:
                        self._firmware_record_log(
                            attempt,
                            f"terminal success=1 returncode={result.returncode} "
                            f"duration_s={result.duration_s:.3f} timed_out={int(result.timed_out)} "
                            f"cancelled={int(result.cancelled)}",
                        )
                        self.firmware_progress_var.set(100.0)
                        frozen_path = self.firmware_selected_image
                        try:
                            frozen_path.unlink()
                            self._firmware_record_log(
                                attempt, f"cleanup frozen_image={frozen_path} removed=1"
                            )
                        except OSError as exc:
                            self._firmware_record_log(
                                attempt,
                                f"cleanup frozen_image={frozen_path} removed=0 error={exc}",
                            )
                        self._firmware_finish_ui(
                            f"烧录、校验并启动应用成功（{result.duration_s:.1f}s）；正在恢复连接"
                        )
                        if not self._firmware_auto_reconnect(str(application_port or "")):
                            self.firmware_status_var.set(
                                "烧录、校验和启动成功，但 15 秒内未能安全匹配原飞控 COM；请检查 USB 枚举"
                            )
                    elif result.cancelled:
                        self._firmware_record_log(
                            attempt,
                            f"terminal success=0 returncode={result.returncode} "
                            f"duration_s={result.duration_s:.3f} timed_out={int(result.timed_out)} "
                            f"cancelled={int(result.cancelled)}",
                        )
                        self._firmware_finish_ui("CubeProgrammer 已取消；未自动重试")
                    elif result.timed_out:
                        self._firmware_record_log(
                            attempt,
                            f"terminal success=0 returncode={result.returncode} "
                            f"duration_s={result.duration_s:.3f} timed_out=1 cancelled={int(result.cancelled)}",
                        )
                        self._firmware_finish_ui("CubeProgrammer 超时并已停止；未自动重试", error=True)
                    elif result.download_verified:
                        self._firmware_record_log(
                            attempt,
                            f"terminal success=0 verified=1 start_failed=1 returncode={result.returncode} "
                            f"duration_s={result.duration_s:.3f} timed_out=0 cancelled=0",
                        )
                        self.firmware_progress_var.set(100.0)
                        if application_port:
                            self._firmware_finish_ui(
                                "固件写入校验成功；CLI 返回非零，但 application CDC 已重新出现"
                            )
                            self._firmware_auto_reconnect(str(application_port))
                        else:
                            self._firmware_finish_ui(
                                "固件已经写入并校验成功，但 ROM DFU 未自动启动应用；请按复位键或断开 USB 后重新上电。"
                            )
                    else:
                        self._firmware_record_log(
                            attempt,
                            f"terminal success=0 returncode={result.returncode} "
                            f"duration_s={result.duration_s:.3f} timed_out={int(result.timed_out)} "
                            f"cancelled={int(result.cancelled)}",
                        )
                        self._firmware_finish_ui(
                            f"CubeProgrammer 失败，exit={result.returncode}；未自动重试",
                            error=True,
                        )
        except queue.Empty:
            pass
        self.after(100, self._firmware_drain_events)

    def _firmware_open_log_dir(self) -> None:
        directory = ensure_directory(dated_directory(FIRMWARE_UPDATE_DIR))
        if hasattr(os, "startfile"):
            os.startfile(directory)  # type: ignore[attr-defined]
        else:
            messagebox.showinfo("升级日志目录", str(directory))

    def _toggle_log_area(self) -> None:
        body = getattr(self, "_body_pane", None)
        log_box = getattr(self, "log_box", None)
        if body is None or log_box is None:
            return
        present = str(log_box) in body.panes()
        if self.show_log_var.get() and not present:
            body.add(log_box, weight=1)
        elif not self.show_log_var.get() and present:
            body.forget(log_box)

    def _validation_update_safety_text(self) -> None:
        ready = self.validation_props_removed_var.get() and self.validation_power_safe_var.get()
        self.validation_safety_var.set(
            "安全门：人工确认完成" if ready else "安全门：必须拆桨并隔离动力"
        )
        if hasattr(self, "validation_begin_button"):
            self._validation_refresh_readiness()

    def _validation_refresh_readiness(self) -> None:
        values = self.validation_latest_values
        now = time.monotonic()
        connected = self._transport_connected()
        fresh = (
            self.validation_latest_host_time > 0.0
            and (now - self.validation_latest_host_time) <= VALIDATION_SAMPLE_FRESH_S
        )
        snapshot_ok = (
            fresh
            and values.get("valid") == "1"
            and values.get("source") == "stabilizer_snapshot"
            and (
                values.get("frame", "").startswith("canonical_flu")
                if self.validation_candidate_applied
                else values.get("frame") == "legacy_intermediate"
            )
            and values.get("units") == "mg_mdps_cdeg"
            and safe_int(values.get("contract"), -1) == 1
            and safe_int(values.get("migration"), -1) == 0
            and (
                values.get("orientation")
                == self.validation_orientation_values.get("active_code")
                if self.validation_candidate_applied
                else True
            )
        )
        bias_ok = snapshot_ok and values.get("bias") == "1"
        motor_1 = safe_int(values.get("m1"), 65535)
        motor_2 = safe_int(values.get("m2"), 65535)
        output_ok = (
            snapshot_ok
            and safe_int(values.get("armed"), 1) == 0
            and motor_1 <= VALIDATION_MOTOR_SAFE_MAX_US
            and motor_2 <= VALIDATION_MOTOR_SAFE_MAX_US
        )
        manual_safe = (
            self.validation_props_removed_var.get()
            and self.validation_power_safe_var.get()
        )

        self._validation_set_gate(
            "conn",
            "ok" if connected else "bad",
            f"{self._transport_label()} 已连接" if connected else "未连接 · 点顶部“启动连接”",
        )
        if snapshot_ok:
            self._validation_set_gate(
                "snapshot", "ok", f"seq={self.validation_latest_sequence}")
        elif fresh:
            self._validation_set_gate(
                "snapshot", "bad", "provenance 不兼容 · 需烧录最新固件")
        else:
            self._validation_set_gate("snapshot", "wait", "自动请求中…")
        self._validation_set_gate(
            "bias",
            "ok" if bias_ok else "wait",
            "ready" if bias_ok else "保持机体静止…",
        )
        self._validation_set_gate(
            "output",
            "ok" if output_ok else "bad",
            f"armed=0  m1={motor_1}  m2={motor_2}"
            if output_ok
            else f"armed={values.get('armed', '-')}  m1={values.get('m1', '-')}  m2={values.get('m2', '-')}",
        )

        health_state, health_text, health_ok = self._validation_health_state()
        self._validation_set_gate("health", health_state, health_text)

        stage = self._validation_selected_stage()
        self.validation_begin_button.configure(
            text=f"开始采集 · {STAGE_DEFINITIONS[stage].label_zh}"
        )
        ready_to_collect = (
            self.validation_session_active
            and connected
            and snapshot_ok
            and bias_ok
            and output_ok
            and manual_safe
            and health_ok
            and self.validation_active_stage is None
        )
        self.validation_begin_button.configure(
            state=tk.NORMAL if ready_to_collect else tk.DISABLED
        )
        self.validation_finish_button.configure(
            state=tk.NORMAL if self.validation_active_stage is not None else tk.DISABLED
        )
        has_restorable_state = any(self.validation_samples.values()) or bool(
            self.validation_skipped_stages or self.validation_unsupported_stages
        )
        self.validation_resume_button.configure(
            state=tk.NORMAL
            if has_restorable_state and not self.validation_session_active
            else tk.DISABLED
        )
        has_evidence = any(self.validation_samples.values()) or bool(
            self.validation_unsupported_stages
        )
        self.validation_report_button.configure(
            state=tk.NORMAL
            if has_evidence and self.validation_active_stage is None
            else tk.DISABLED
        )

        if not self.validation_session_active and self.validation_loaded_history and has_restorable_state:
            next_action = (
                "已加载历史样本：若只查看可直接生成报告；若要补测，勾选安全门、连接飞控，"
                "再点击“继续已加载验收”"
            )
        elif not self.validation_session_active:
            next_action = "下一步：恢复已有历史，或勾选两项安全确认后点击“新建验收”"
        elif not connected:
            next_action = "下一步：点击顶部“启动连接”，V0会自动轮询IMU"
        elif not snapshot_ok:
            if fresh:
                next_action = "当前固件快照格式不兼容；请烧录最新固件后重新连接"
            else:
                next_action = "正在自动请求有效快照，请保持连接；超过2秒仍无数据请检查COM是否为飞控USB CDC"
        elif not bias_ok:
            next_action = "下一步：保持飞机完全静止，等待陀螺零偏ready"
        elif not output_ok:
            next_action = "安全输出未通过：确认未解锁，并断开电调动力"
        elif not health_ok:
            next_action = (
                f"采样链异常（{health_text}）：IMU DRDY 中断很可能失效，采样已退到 20ms 轮询。"
                "此状态下滤波与抗混叠假设均不成立，采到的数据不能用于标定，飞控也已禁止解锁。"
                "请检查 PC0 中断线与 IMU 排针，恢复后本项会自动转正常。"
            )
        elif not manual_safe:
            next_action = "下一步：勾选“已拆桨”和“动力已隔离/机体固定”"
        elif self.validation_active_stage is not None:
            next_action = (
                f"正在采集“{STAGE_DEFINITIONS[self.validation_active_stage].label_zh}”；"
                "保持动作，样本达到建议值后点击“停止并分析”"
            )
        else:
            next_action = (
                f"准备完成：按右侧说明摆放/转动飞机，然后点击“开始采集 · "
                f"{STAGE_DEFINITIONS[stage].label_zh}”"
            )
        self.validation_next_action_var.set(next_action)
        self._validation_refresh_orientation_controls()

    def _validation_set_status(self, status: ValidationStatus, text: str) -> None:
        self.validation_session_var.set(f"{status.value} · {text}")
        style = {
            ValidationStatus.PASS: "Pass.TLabel",
            ValidationStatus.WARN: "Warn.TLabel",
            ValidationStatus.SKIPPED: "Muted.TLabel",
            ValidationStatus.FAIL: "Fail.TLabel",
            ValidationStatus.UNSUPPORTED: "Fail.TLabel",
            ValidationStatus.NOT_RUN: "Warn.TLabel",
        }[status]
        self.validation_status_label.configure(style=style)

    def _validation_selected_stage(self) -> ValidationStage:
        selected = self.validation_stage_tree.selection()
        value = selected[0] if selected else self.validation_stage_var.get()
        try:
            return ValidationStage(value)
        except ValueError:
            return VALIDATION_UI_STAGES[0]

    def _on_validation_stage_select(self, _event: tk.Event | None = None) -> None:
        stage = self._validation_selected_stage()
        self.validation_stage_var.set(stage.value)
        self.validation_instruction_var.set(STAGE_DEFINITIONS[stage].prompt_zh)
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_readiness()

    def _validation_refresh_stage_view(self, stage: ValidationStage | None = None) -> None:
        stage = stage or self._validation_selected_stage()
        samples = self.validation_samples.get(stage, [])
        target = (
            VALIDATION_STATIC_TARGET_SAMPLES
            if STAGE_DEFINITIONS[stage].kind == "static"
            else VALIDATION_ROTATION_TARGET_SAMPLES
        )
        self.validation_sample_var.set(f"当前步骤样本：{len(samples)} / 建议 {target}")
        self.validation_progress.configure(value=min(100.0, len(samples) * 100.0 / target))
        result = self.validation_stage_results.get(stage)
        if stage in self.validation_skipped_stages:
            text = "该旧版可选步骤已跳过；不影响必要姿态与旋转验收。"
        elif stage in self.validation_unsupported_stages:
            text = "该步骤的数据源不受支持，已有样本不能作为通过证据。"
        elif result is None:
            text = "尚未分析。开始步骤后按提示缓慢操作，采集完成再点击“停止并分析”。"
        else:
            lines = [
                f"状态：{getattr(result, 'status').value}",
                f"样本：{getattr(result, 'sample_count', len(samples))}",
            ]
            if hasattr(result, "capture_quality_status"):
                lines.append(
                    f"采集质量：{getattr(result, 'capture_quality_status').value}"
                )
            if hasattr(result, "canonical_match_status"):
                match_status = getattr(result, "canonical_match_status")
                lines.append(f"当前数据直接符合FLU：{match_status.value}")
                if (
                    getattr(result, "capture_quality_status", None)
                    is ValidationStatus.PASS
                    and match_status is ValidationStatus.FAIL
                ):
                    lines.append(
                        "解释：采集本身稳定，但当前legacy轴与目标FLU不一致；"
                        "这通常用于推导候选映射，不代表你摆放错误。"
                    )
                if self.validation_verification_mode:
                    consistency, _error, reason = self._validation_static_fusion_consistency(stage)
                    lines.append(f"加速度倾角 ↔ Fusion：{consistency.value} · {reason}")
            if getattr(result, "accel_mean_g", None) is not None:
                lines.append(f"加速度均值 g：{getattr(result, 'accel_mean_g')}")
                lines.append(f"陀螺均值 dps：{getattr(result, 'gyro_mean_dps')}")
                lines.append(f"|a| 均值 g：{getattr(result, 'accel_norm_mean_g'):.4f}")
            if getattr(result, "integrated_angle_deg", None) is not None:
                lines.append(f"陀螺积分 deg：{getattr(result, 'integrated_angle_deg')}")
                lines.append(f"主轴：{getattr(result, 'dominant_axis', '-')}")
            findings = getattr(result, "findings", ())
            if findings:
                lines.append("\n".join(f"• {finding}" for finding in findings))
            text = "\n".join(lines)
        self.validation_result_text.configure(state=tk.NORMAL)
        self.validation_result_text.delete("1.0", tk.END)
        self.validation_result_text.insert("1.0", text)
        self.validation_result_text.configure(state=tk.DISABLED)

    def _validation_refresh_candidate_summary(self) -> None:
        stage_inputs = {
            stage: None if stage in self.validation_unsupported_stages else tuple(samples)
            for stage, samples in self.validation_samples.items()
            if samples or stage in self.validation_unsupported_stages
        }
        result = analyze_six_face(stage_inputs)
        candidate = result.candidate
        if self.validation_verification_mode:
            completed = sum(
                1 for stage in VALIDATION_UI_STAGES if stage in self.validation_finished_stages
            )
            self.validation_candidate_verified = self._validation_candidate_verification_passed()
            message = (
                f"映射后复验：必要步骤 {completed}/{len(VALIDATION_UI_STAGES)} · "
                f"当前数据直接符合FLU={result.canonical_match_status.value} · "
                f"{'已满足写入条件' if self.validation_candidate_verified else '尚未满足写入条件'}"
            )
        elif candidate.complete and candidate.matrix_flu_from_observed is not None:
            matrix = candidate.matrix_flu_from_observed
            descriptor = signed_permutation_descriptor(matrix)
            mapping = ", ".join(
                f"{key}={value}" for key, value in candidate.axis_mapping.items()
            )
            message = (
                f"轴映射候选：{candidate.status.value} · R_FLU←observed={matrix} · "
                f"det={candidate.determinant} · confidence={candidate.confidence:.2%} · {mapping} · 未应用"
            )
            if result.canonical_match_status is ValidationStatus.FAIL:
                message += "。采集质量可用但当前运行轴不直接符合FLU；请在下方二次确认后先做RAM试用。"
            if descriptor is not None:
                self.validation_candidate_matrix = tuple(tuple(row) for row in matrix)
                self.validation_candidate_descriptor = descriptor
                self.validation_candidate_confidence = candidate.confidence
                self.validation_candidate_source_path = (
                    self.validation_loaded_report_path or self.validation_session_path
                )
        else:
            required_static = tuple(
                stage
                for stage in VALIDATION_UI_STAGES
                if STAGE_DEFINITIONS[stage].kind == "static"
            )
            completed = sum(1 for stage in required_static if self.validation_samples.get(stage))
            message = f"轴映射候选：必要静态姿态已完成 {completed}/{len(required_static)}。"
        self.validation_candidate_var.set(message)
        if hasattr(self, "validation_apply_button"):
            self._validation_refresh_orientation_controls()

    def _validation_candidate_verification_passed(self) -> bool:
        if not self.validation_verification_mode or not self.validation_candidate_applied:
            return False
        for stage in VALIDATION_UI_STAGES:
            if stage not in self.validation_finished_stages:
                return False
            result = self.validation_stage_results.get(stage)
            if result is None or getattr(result, "status", None) is not ValidationStatus.PASS:
                return False
            if (
                hasattr(result, "canonical_match_status")
                and getattr(result, "canonical_match_status") is not ValidationStatus.PASS
            ):
                return False
            if STAGE_DEFINITIONS[stage].kind == "static":
                consistency, _error, _reason = self._validation_static_fusion_consistency(stage)
                if consistency is not ValidationStatus.PASS:
                    return False
            else:
                if getattr(result, "gyro_direction_matches_flu", None) is not True:
                    return False
                if getattr(result, "attitude_direction_matches_flu", None) is not True:
                    return False
        return True

    def _validation_static_fusion_consistency(
        self,
        stage: ValidationStage,
    ) -> tuple[ValidationStatus, float | None, str]:
        samples = self.validation_samples.get(stage, [])
        errors: list[float] = []

        def wrapped_error(actual_deg: float, expected_deg: float) -> float:
            return abs((actual_deg - expected_deg + 180.0) % 360.0 - 180.0)

        for sample in samples:
            horizontal = math.sqrt(
                sample.accel_y_g * sample.accel_y_g
                + sample.accel_z_g * sample.accel_z_g
            )
            roll_acc = math.degrees(math.atan2(sample.accel_y_g, sample.accel_z_g))
            pitch_acc = math.degrees(math.atan2(-sample.accel_x_g, horizontal))
            if stage is ValidationStage.LEVEL:
                if sample.roll_deg is None or sample.pitch_deg is None:
                    continue
                errors.extend(
                    (
                        wrapped_error(sample.roll_deg, roll_acc),
                        wrapped_error(sample.pitch_deg, pitch_acc),
                    )
                )
            elif stage is ValidationStage.NOSE_UP:
                if sample.pitch_deg is None:
                    continue
                errors.append(wrapped_error(sample.pitch_deg, pitch_acc))
            elif stage is ValidationStage.LEFT_SIDE_UP:
                if sample.roll_deg is None:
                    continue
                errors.append(wrapped_error(sample.roll_deg, roll_acc))

        minimum = 40 if stage is ValidationStage.LEVEL else 20
        if len(errors) < minimum:
            return ValidationStatus.FAIL, None, "缺少足够的Fusion姿态样本"
        median_error = statistics.median(errors)
        if median_error <= 8.0:
            status = ValidationStatus.PASS
        elif median_error <= 15.0:
            status = ValidationStatus.WARN
        else:
            status = ValidationStatus.FAIL
        return (
            status,
            median_error,
            f"加速度倾角与Fusion中位误差={median_error:.2f}°（PASS≤8°）",
        )

    def _validation_candidate_level_ready(self) -> tuple[bool, str]:
        matrix = self.validation_candidate_matrix
        values = self.validation_latest_values
        if matrix is None:
            return False, "候选矩阵尚未形成"
        try:
            observed = (
                float(values["ax_mg"]) * 0.001,
                float(values["ay_mg"]) * 0.001,
                float(values["az_mg"]) * 0.001,
            )
            gyro = tuple(
                float(values[name]) * 0.001
                for name in ("gx_mdps", "gy_mdps", "gz_mdps")
            )
        except (KeyError, ValueError):
            return False, "等待完整实时IMU快照"
        predicted = tuple(
            sum(matrix[row][column] * observed[column] for column in range(3))
            for row in range(3)
        )
        if (
            abs(predicted[0]) > 0.15
            or abs(predicted[1]) > 0.15
            or not 0.85 <= predicted[2] <= 1.15
        ):
            return False, (
                "临时应用前请把飞机水平放稳；候选换算后的比力应接近[0,0,+1]g，"
                f"当前为[{predicted[0]:+.3f},{predicted[1]:+.3f},{predicted[2]:+.3f}]g"
            )
        if max(abs(value) for value in gyro) > 3.0:
            return False, "临时应用前请保持飞机静止（gyro需小于3 dps）"
        return True, ""

    def _validation_refresh_orientation_controls(self) -> None:
        if not hasattr(self, "validation_apply_button"):
            return
        manual_confirmed = (
            self.validation_axis_labels_confirmed_var.get()
            and self.validation_mapping_confirmed_var.get()
            and self.validation_props_removed_var.get()
            and self.validation_power_safe_var.get()
        )
        connected = self._transport_connected()
        candidate_ready = bool(
            self.validation_candidate_descriptor
            and self.validation_candidate_matrix is not None
            and self.validation_candidate_confidence >= 0.75
        )
        safe, _reason = self._validation_latest_is_safe()
        level_ready, level_reason = self._validation_candidate_level_ready()
        can_apply = (
            candidate_ready
            and manual_confirmed
            and connected
            and safe
            and level_ready
            and not self.validation_candidate_applied
            and not self.validation_orientation_pending
        )
        self.validation_apply_button.configure(state=tk.NORMAL if can_apply else tk.DISABLED)
        self.validation_revert_button.configure(
            state=tk.NORMAL
            if self.validation_candidate_applied and connected and safe and not self.validation_orientation_pending
            else tk.DISABLED
        )
        self.validation_verify_button.configure(
            state=tk.NORMAL
            if self.validation_candidate_applied
            and not self.validation_verification_mode
            and not self.validation_orientation_pending
            else tk.DISABLED
        )
        self.validation_candidate_verified = self._validation_candidate_verification_passed()
        self.validation_commit_button.configure(
            state=tk.NORMAL
            if self.validation_candidate_verified
            and not self.validation_candidate_committed
            and connected
            and safe
            and not self.validation_orientation_pending
            else tk.DISABLED
        )
        if not candidate_ready:
            text = "应用状态：必要静态姿态尚未形成合法 det=+1 候选。"
        else:
            matrix = self.validation_candidate_matrix
            text = (
                f"候选图示：FLU[X前,Y左,Z上] ← observed[{self.validation_candidate_descriptor}] · "
                f"R={matrix} · confidence={self.validation_candidate_confidence:.2%}"
            )
            if self.validation_candidate_committed:
                text += " · 已由目标确认写入 Flash；仍受后续整机验收门限制"
            elif self.validation_candidate_verified:
                text += " · RAM复验全部PASS，可执行C写入Flash"
            elif self.validation_candidate_applied:
                text += " · 已临时应用到RAM，断电恢复；点击B开始必要步骤复验"
            else:
                text += " · 尚未应用"
                if not level_ready:
                    text += f" · {level_reason}"
        self.validation_orientation_var.set(text)
        phase, action = v0_workflow_guidance(
            candidate_available=candidate_ready,
            candidate_applied=self.validation_candidate_applied,
            verification_mode=self.validation_verification_mode,
            candidate_verified=self.validation_candidate_verified,
            candidate_committed=self.validation_candidate_committed,
            runtime_ready=(
                connected and safe and level_ready
                and self.validation_props_removed_var.get()
                and self.validation_power_safe_var.get()
            ),
        )
        self.validation_phase_var.set(phase)
        self.validation_phase_action_var.set(action)
        self._validation_refresh_stepper()
        self._validation_recolor_stage_rows()
        if hasattr(self, "validation_steps_box"):
            self.validation_steps_box.configure(
                text=(
                    "RAM 映射后复验步骤（只看重新采集的新样本）"
                    if self.validation_verification_mode
                    else "坐标发现样本（旧 FAIL 用于推导映射，不是最终失败）"
                )
            )

    def _validation_apply_candidate(self) -> None:
        self._validation_refresh_orientation_controls()
        if str(self.validation_apply_button["state"]) != str(tk.NORMAL):
            return
        descriptor = self.validation_candidate_descriptor
        if not messagebox.askyesno(
            "临时应用坐标映射",
            "将只在 RAM 中临时应用下列映射，断电会恢复；飞控必须保持拆桨/动力隔离。\n\n"
            f"FLU [X前,Y左,Z上] ← observed [{descriptor}]\n"
            f"置信度 {self.validation_candidate_confidence:.2%}\n\n继续？",
        ):
            return
        self.validation_orientation_pending = "apply"
        self._validation_send_orientation_command(f"IMUFRAME APPLY {descriptor}")
        self._validation_note_orientation_event("wait", "已发出 IMUFRAME APPLY；等待飞机确认 RAM 映射")
        self._validation_refresh_orientation_controls()

    def _validation_revert_candidate(self) -> None:
        if not messagebox.askyesno("撤销临时映射", "恢复飞机 Flash 中原有映射并清除本次复验状态？"):
            return
        self.validation_orientation_pending = "revert"
        self._validation_send_orientation_command("IMUFRAME REVERT")
        self._validation_refresh_orientation_controls()

    def _validation_begin_candidate_verification(self) -> None:
        if not self.validation_candidate_applied:
            return
        self._validation_autosave_session(force=True)
        self.validation_verification_mode = True
        self.validation_candidate_verified = False
        self.validation_session_active = True
        self.validation_poll_requested = True
        self.validation_active_stage = None
        self.validation_samples = {stage: [] for stage in STAGE_DEFINITIONS}
        self.validation_stage_results.clear()
        self.validation_unsupported_stages.clear()
        self.validation_skipped_stages.clear()
        self.validation_finished_stages.clear()
        self.validation_raw_rows.clear()
        self.validation_session_path = None
        self.validation_session_created_at = datetime.now().astimezone().isoformat()
        self.validation_report_paths = None
        for stage in VALIDATION_UI_STAGES:
            self.validation_stage_tree.item(
                stage.value,
                values=(ValidationStatus.NOT_RUN.value, 0),
            )
        first_stage = VALIDATION_UI_STAGES[0]
        self.validation_stage_tree.selection_set(first_stage.value)
        self.validation_stage_tree.focus(first_stage.value)
        self.validation_stage_var.set(first_stage.value)
        self.validation_instruction_var.set(STAGE_DEFINITIONS[first_stage].prompt_zh)
        self._validation_set_status(
            ValidationStatus.NOT_RUN,
            "RAM候选复验已开始；只采集6个必要步骤，全部PASS后才能写Flash",
        )
        self._validation_refresh_stage_view(first_stage)
        self._validation_refresh_candidate_summary()
        self._validation_refresh_readiness()

    def _validation_commit_candidate(self) -> None:
        self.validation_candidate_verified = self._validation_candidate_verification_passed()
        if not self.validation_candidate_verified:
            messagebox.showwarning("复验未通过", "6 个必要步骤必须全部 PASS，不能写入 Flash。")
            return
        if not messagebox.askyesno(
            "写入飞机参数 Flash",
            "RAM 映射已通过必要步骤复验。现在只持久化 IMU 坐标映射参数，"
            "不重新烧录固件；写入完成前请勿断电。继续？",
        ):
            return
        self.validation_orientation_pending = "commit"
        self.validation_orientation_poll_count = 0
        self._validation_send_orientation_command("IMUFRAME COMMIT")
        self._validation_note_orientation_event("wait", "已发出 IMUFRAME COMMIT；等待目标回报 dirty=0")
        self._validation_refresh_orientation_controls()

    _ORIENTATION_EVENT_STYLES = {
        "ok": "Pass.TLabel",
        "wait": "Warn.TLabel",
        "bad": "Fail.TLabel",
    }

    def _validation_note_orientation_event(self, state: str, text: str) -> None:
        """写映射操作回执。

        带时间戳，因此即使连续两次回包内容相同，也能看出"刚刚确实收到了"。
        这个变量不参与 _validation_refresh_orientation_controls 的重算，
        否则提示会在 0.5s 内被候选摘要覆盖掉。
        """
        self.validation_orientation_event_var.set(f"{time.strftime('%H:%M:%S')} · {text}")
        label = getattr(self, "validation_orientation_event_label", None)
        if label is not None:
            label.configure(
                style=self._ORIENTATION_EVENT_STYLES.get(state, "Idle.TLabel"))

    def _validation_send_orientation_command(self, command: str) -> None:
        normalized = " ".join(command.strip().upper().split())
        self.validation_authorized_orientation_commands.add(normalized)
        self._send_proto(PROTO_REQ_IMU_FRAME, command, command)

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

    def _validation_poll_orientation_commit(self) -> None:
        if self.validation_orientation_pending != "commit":
            return
        self.validation_orientation_poll_count += 1
        if self.validation_orientation_poll_count > 20:
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event(
                "bad",
                "Flash 写入确认超时：10 秒内没有收到 dirty=0；保持拆桨，点击“读取飞机映射状态”复查",
            )
            self._validation_refresh_orientation_controls()
            return
        self._send_proto_silent(PROTO_REQ_IMU_FRAME, "IMUFRAME?")
        self.after(500, self._validation_poll_orientation_commit)

    def _validation_handle_imu_frame_line(self, line: str) -> None:
        values = parse_kv(line)
        self.validation_orientation_values = dict(values)
        self.last_reply_rx = time.monotonic()
        active = values.get("active", "")
        persisted = values.get("persisted", "")
        event = values.get("event", "status")
        dirty = values.get("dirty", "-")
        base = values.get("base", "")
        commit_was_pending = self.validation_orientation_pending == "commit"
        apply_was_pending = self.validation_orientation_pending == "apply"
        # 无论后面走哪条分支，先把飞机原原本本回报的事实显示出来：点“读取飞机
        # 映射状态”至少要看得见这一行变化。
        self.validation_orientation_target_var.set(
            f"active={active or '-'}  persisted={persisted or '-'}  "
            f"dirty={dirty}  event={event}"
        )
        if base and base != "legacy_intermediate_v1":
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event(
                "bad", f"拒绝使用目标映射：base={base}，期望 legacy_intermediate_v1")
            self._validation_refresh_orientation_controls()
            return

        reported = False
        candidate_matches_active = bool(
            self.validation_candidate_descriptor
            and active == self.validation_candidate_descriptor
        )
        failure_events = {
            "safety_blocked": "飞机安全门拒绝：必须 disarmed 且电机输出≤1100us",
            "apply_busy": "参数后台写入中，暂不能切换 RAM 映射",
            "revert_busy": "参数后台写入中，暂不能撤销映射",
            "commit_busy": "已有另一映射正在写入，拒绝覆盖",
            "param_not_ready": "参数服务尚未就绪",
            "invalid_descriptor": "飞机拒绝了非法/镜像轴映射",
            "invalid_usage": "IMUFRAME 命令格式不兼容",
            "revert_failed": "飞机无法恢复已持久化映射",
            "commit_failed": "飞机无法准备映射参数",
            "commit_queue_failed": "后台 Flash 请求排队失败；尚未持久化，可稍后重试",
        }
        if event in failure_events:
            self.validation_authorized_orientation_commands.clear()
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event("bad", f"映射操作未完成：{failure_events[event]}")
            self._validation_set_status(ValidationStatus.FAIL, failure_events[event])
            self._validation_refresh_orientation_controls()
            return
        if self.validation_orientation_pending == "apply" and candidate_matches_active:
            self.validation_authorized_orientation_commands.discard(
                " ".join(
                    f"IMUFRAME APPLY {self.validation_candidate_descriptor}".upper().split()
                )
            )
        elif self.validation_orientation_pending == "revert" and event == "reverted":
            self.validation_authorized_orientation_commands.discard("IMUFRAME REVERT")
        elif self.validation_orientation_pending == "commit" and event in {
            "commit_queued",
            "committed",
            "status",
        }:
            self.validation_authorized_orientation_commands.discard("IMUFRAME COMMIT")
        if event == "reverted" or (
            self.validation_orientation_pending == "revert" and not candidate_matches_active
        ):
            self.validation_candidate_applied = False
            self.validation_candidate_verified = False
            self.validation_candidate_committed = False
            self.validation_verification_mode = False
            self.validation_orientation_pending = ""
            self._validation_write_orientation_audit("reverted", flash_writes=0)
            self._validation_set_status(ValidationStatus.WARN, "飞机已撤销 RAM 候选并恢复持久映射")
            self._validation_note_orientation_event(
                "wait", "飞机已撤销 RAM 候选并恢复持久映射")
            reported = True
        elif candidate_matches_active:
            first_apply = not self.validation_candidate_applied
            self.validation_candidate_applied = True
            if first_apply and apply_was_pending and event == "applied":
                self._validation_write_orientation_audit("ram_applied", flash_writes=0)
            if self.validation_orientation_pending == "apply":
                self.validation_orientation_pending = ""
                self._validation_set_status(
                    ValidationStatus.WARN,
                    "飞机已确认 RAM 映射；现在必须执行6步复验",
                )
                self._validation_note_orientation_event(
                    "ok", f"飞机已确认 RAM 映射 active={active}；接下来点 B 重采 6 步复验")
                reported = True
            if persisted == self.validation_candidate_descriptor and dirty == "0":
                first_commit = not self.validation_candidate_committed
                self.validation_candidate_committed = True
                self.validation_orientation_pending = ""
                if first_commit and commit_was_pending:
                    audit_path = self._validation_write_orientation_audit(
                        "flash_committed", flash_writes=1
                    )
                    self.validation_report_var.set(f"映射写入审计：{audit_path}")
                    self._validation_note_orientation_event(
                        "ok",
                        f"写入参数 Flash 成功并已读回确认：persisted={persisted}，dirty=0。"
                        f"审计已保存：{audit_path}",
                    )
                    # 写 Flash 是一次性里程碑，页头状态条在滚动后会移出视野，
                    # 因此这里额外弹一次确认，确保不会"写完了却不知道写没写"。
                    messagebox.showinfo(
                        "映射已写入参数 Flash",
                        f"飞机已读回确认：persisted={persisted}，dirty=0。\n\n"
                        f"审计文件：{audit_path}\n\n"
                        "注意：这只持久化了 IMU 坐标映射，flight_release 仍为 false，"
                        "不代表允许自由飞行。",
                    )
                else:
                    self._validation_note_orientation_event(
                        "ok", f"飞机映射已持久化：persisted={persisted}，dirty=0")
                self._validation_set_status(
                    ValidationStatus.PASS,
                    "目标确认映射已持久化；flight_release仍为false",
                )
                reported = True
            elif self.validation_orientation_pending == "commit" and dirty == "1":
                self._validation_note_orientation_event(
                    "wait", "后台正在写参数 Flash（dirty=1），请勿断电；正在轮询确认…")
                reported = True
                if self.validation_orientation_poll_count == 0:
                    self.after(500, self._validation_poll_orientation_commit)
        if not reported:
            # 例如手动点“读取飞机映射状态”：也必须看得见回包到达。
            self._validation_note_orientation_event(
                "wait" if dirty == "1" else "ok",
                f"已读取飞机映射状态：active={active or '-'}，"
                f"persisted={persisted or '-'}，dirty={dirty}",
            )
        self._validation_refresh_orientation_controls()

    def _validation_stage_inputs(self) -> dict[ValidationStage, tuple[ImuSample, ...] | None]:
        return {
            stage: None if stage in self.validation_unsupported_stages else tuple(samples)
            for stage, samples in self.validation_samples.items()
            if (
                (samples and stage not in self.validation_skipped_stages)
                or stage in self.validation_unsupported_stages
            )
        }

    def _validation_rebuild_results(self) -> None:
        self.validation_stage_results.clear()
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            samples = self.validation_samples.get(stage, [])
            label = definition.label_zh if definition.required else f"{definition.label_zh}（可选）"
            if stage in self.validation_skipped_stages:
                status_text = ValidationStatus.SKIPPED.value
            elif stage in self.validation_unsupported_stages:
                status_text = ValidationStatus.UNSUPPORTED.value
            elif stage in self.validation_finished_stages and samples:
                result = (
                    analyze_static_stage(stage, tuple(samples))
                    if definition.kind == "static"
                    else analyze_rotation_stage(stage, tuple(samples))
                )
                self.validation_stage_results[stage] = result
                status_text = result.status.value
            elif samples:
                status_text = "PARTIAL"
            else:
                status_text = ValidationStatus.NOT_RUN.value
            self.validation_stage_tree.item(
                stage.value,
                text=label,
                values=(status_text, len(samples)),
            )

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

    def _validation_skip_stage(self) -> None:
        if not self.validation_session_active or self.validation_active_stage is not None:
            return
        stage = self._validation_selected_stage()
        definition = STAGE_DEFINITIONS[stage]
        if definition.required:
            messagebox.showwarning("必需步骤", f"“{definition.label_zh}”不能跳过。")
            return
        self.validation_samples[stage] = []
        self.validation_stage_results.pop(stage, None)
        self.validation_unsupported_stages.discard(stage)
        self.validation_skipped_stages.add(stage)
        self.validation_finished_stages.add(stage)
        self.validation_stage_tree.item(
            stage.value,
            values=(ValidationStatus.SKIPPED.value, 0),
        )
        self._validation_set_status(
            ValidationStatus.SKIPPED,
            f"已跳过旧版可选步骤：{definition.label_zh}",
        )
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_candidate_summary()
        self._validation_autosave_session(force=True)
        if self.validation_candidate_descriptor and self.validation_candidate_source_path is None:
            self.validation_candidate_source_path = self.validation_session_path
        self._validation_refresh_readiness()

    def _validation_start_session(self) -> None:
        if self.validation_loaded_history and any(self.validation_samples.values()):
            if not messagebox.askyesno(
                "新建独立验收",
                "当前已加载一份历史验收。新建会清空当前页面工作区，但不会覆盖或删除历史报告；"
                "之后仍可从“历史验收”下拉框恢复。确定新建？",
            ):
                return
        if not self.validation_props_removed_var.get() or not self.validation_power_safe_var.get():
            messagebox.showwarning("安全门未通过", "请先确认已拆桨，并断开电调动力或可靠固定机体。")
            return
        if not self._transport_connected():
            messagebox.showwarning("尚未连接", "请先建立到飞控的连接。")
            return
        for transport in (self.tcp_transport, self.udp_transport, self.serial_transport):
            transport.cancel_pending_sends()
        self.validation_session_active = True
        self.validation_poll_requested = True
        self.validation_active_stage = None
        self.validation_samples = {stage: [] for stage in STAGE_DEFINITIONS}
        self.validation_stage_results.clear()
        self.validation_unsupported_stages.clear()
        self.validation_skipped_stages.clear()
        self.validation_finished_stages.clear()
        self.validation_raw_rows.clear()
        self.validation_report_paths = None
        self.validation_loaded_report_path = None
        self.validation_loaded_history = False
        self.validation_candidate_matrix = None
        self.validation_candidate_descriptor = ""
        self.validation_candidate_confidence = 0.0
        self.validation_candidate_source_path = None
        self.validation_candidate_applied = False
        self.validation_candidate_verified = False
        self.validation_candidate_committed = False
        self.validation_verification_mode = False
        self.validation_orientation_pending = ""
        self.validation_session_created_at = datetime.now().astimezone().isoformat()
        self.validation_session_path = None
        self.validation_last_autosave_monotonic = 0.0
        self.validation_latest_sequence = None
        self.validation_latest_values.clear()
        self.validation_latest_host_time = 0.0
        self.validation_stage_last_sequence = None
        first_stage = VALIDATION_UI_STAGES[0]
        self.validation_stage_tree.selection_set(first_stage.value)
        self.validation_stage_tree.focus(first_stage.value)
        self.validation_stage_var.set(first_stage.value)
        for stage in VALIDATION_UI_STAGES:
            definition = STAGE_DEFINITIONS[stage]
            self.validation_stage_tree.item(
                stage.value,
                text=definition.label_zh,
                values=(ValidationStatus.NOT_RUN.value, 0),
            )
        self._validation_set_status(ValidationStatus.NOT_RUN, "只读会话已开始，等待有效目标快照")
        self.validation_report_var.set("报告：尚未生成")
        self._validation_refresh_candidate_summary()
        self._send_proto(PROTO_REQ_IMU, "IMU?")
        self._send_proto(PROTO_REQ_IMU_FRAME, "IMUFRAME?")
        self._validation_refresh_stage_view()
        self._validation_refresh_readiness()

    def _validation_stop_session(self) -> None:
        if self.validation_active_stage is not None:
            self._validation_finish_stage()
        self.validation_session_active = False
        self.validation_poll_requested = False
        self.validation_active_stage = None
        self._validation_autosave_session(force=True)
        self._validation_set_status(ValidationStatus.WARN, "只读会话已结束；结果仍不构成飞行放行")
        self._validation_refresh_readiness()

    def _validation_latest_is_safe(self) -> tuple[bool, str]:
        values = self.validation_latest_values
        if not values or (time.monotonic() - self.validation_latest_host_time) > VALIDATION_SAMPLE_FRESH_S:
            return False, "没有新鲜的完整目标快照"
        if values.get("source") != "stabilizer_snapshot" or values.get("valid") != "1":
            return False, "固件不支持 V0 snapshot provenance；请重新编译并烧写本次固件"
        if safe_int(values.get("contract"), -1) != 1:
            return False, f"FLU contract version 不支持：{values.get('contract', '-')}"
        frame = values.get("frame", "")
        expected_frame = (
            frame.startswith("canonical_flu")
            if self.validation_candidate_applied
            else frame == "legacy_intermediate"
        )
        if not expected_frame:
            return False, f"frame provenance 不支持：{frame or '-'}"
        if self.validation_candidate_applied:
            expected_code = self.validation_orientation_values.get("active_code")
            if expected_code is None or values.get("orientation") != expected_code:
                return False, (
                    "orientation provenance 不匹配："
                    f"sample={values.get('orientation', '-')} target={expected_code or '-'}"
                )
        if safe_int(values.get("migration"), -1) != 0:
            return False, f"migration provenance 不支持：{values.get('migration', '-')}"
        if values.get("bias") != "1":
            return False, "陀螺零偏尚未完成"
        if safe_int(values.get("armed"), 1) != 0:
            return False, "目标报告 armed=1，禁止采集"
        motor_1 = safe_int(values.get("m1"), 65535)
        motor_2 = safe_int(values.get("m2"), 65535)
        if motor_1 > VALIDATION_MOTOR_SAFE_MAX_US or motor_2 > VALIDATION_MOTOR_SAFE_MAX_US:
            return False, f"电机输出不安全：m1={motor_1} m2={motor_2}"
        return True, ""

    def _validation_begin_stage(self) -> None:
        if not self.validation_session_active:
            messagebox.showwarning("尚未开始", "请先恢复历史验收，或点击“新建独立验收”。")
            return
        safe, reason = self._validation_latest_is_safe()
        if not safe:
            self._validation_set_status(ValidationStatus.UNSUPPORTED, reason)
            messagebox.showwarning("数据或安全门未通过", reason)
            return
        stage = self._validation_selected_stage()
        self.validation_samples[stage] = []
        self.validation_stage_results.pop(stage, None)
        self.validation_unsupported_stages.discard(stage)
        self.validation_skipped_stages.discard(stage)
        self.validation_finished_stages.discard(stage)
        self.validation_active_stage = stage
        self.validation_stage_last_sequence = self.validation_latest_sequence
        self.validation_stage_tree.item(stage.value, values=("COLLECTING", 0))
        self._validation_set_status(ValidationStatus.WARN, f"正在采集：{STAGE_DEFINITIONS[stage].label_zh}")
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_readiness()

    def _validation_finish_stage(self) -> None:
        stage = self.validation_active_stage
        if stage is None:
            return
        samples = tuple(self.validation_samples[stage])
        if STAGE_DEFINITIONS[stage].kind == "static":
            result = analyze_static_stage(stage, samples)
        else:
            result = analyze_rotation_stage(stage, samples)
        self.validation_stage_results[stage] = result
        self.validation_finished_stages.add(stage)
        self.validation_active_stage = None
        self.validation_stage_last_sequence = None
        self.validation_stage_tree.item(
            stage.value,
            values=(result.status.value, result.sample_count),
        )
        self._validation_set_status(result.status, f"{STAGE_DEFINITIONS[stage].label_zh} 分析完成")
        self._validation_refresh_stage_view(stage)
        self._validation_refresh_candidate_summary()
        self._validation_autosave_session(force=True)
        if self.validation_candidate_descriptor and self.validation_candidate_source_path is None:
            self.validation_candidate_source_path = self.validation_session_path
        stages = list(VALIDATION_UI_STAGES)
        current_index = stages.index(stage)
        if current_index + 1 < len(stages):
            next_stage = stages[current_index + 1]
            self.validation_stage_tree.selection_set(next_stage.value)
            self.validation_stage_tree.focus(next_stage.value)
            self.validation_stage_tree.see(next_stage.value)
            self.validation_stage_var.set(next_stage.value)
            self.validation_instruction_var.set(STAGE_DEFINITIONS[next_stage].prompt_zh)
        self._validation_refresh_readiness()

    def _validation_abort_active_stage(
        self,
        status: ValidationStatus,
        reason: str,
    ) -> None:
        stage = self.validation_active_stage
        if stage is None:
            return
        self.validation_unsupported_stages.add(stage)
        self.validation_finished_stages.add(stage)
        self.validation_active_stage = None
        self.validation_stage_last_sequence = None
        self.validation_stage_tree.item(
            stage.value,
            values=(status.value, len(self.validation_samples[stage])),
        )
        self.validation_raw_rows.append(
            {
                "stage": stage.value,
                "event": reason,
                "source": self.validation_latest_values.get("source", ""),
                "frame": self.validation_latest_values.get("frame", ""),
                "contract": self.validation_latest_values.get("contract", ""),
                "migration": self.validation_latest_values.get("migration", ""),
                "bias": self.validation_latest_values.get("bias", ""),
                "armed": self.validation_latest_values.get("armed", ""),
                "m1": self.validation_latest_values.get("m1", ""),
                "m2": self.validation_latest_values.get("m2", ""),
            }
        )
        self._validation_set_status(status, reason)
        self.validation_result_text.configure(state=tk.NORMAL)
        self.validation_result_text.delete("1.0", tk.END)
        self.validation_result_text.insert("1.0", reason)
        self.validation_result_text.configure(state=tk.DISABLED)
        self._validation_refresh_candidate_summary()
        self._validation_autosave_session(force=True)
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

    # 与固件 APP_ImuHealthLevel 一一对应。
    IMU_HEALTH_LABELS = {0: "正常", 1: "降级", 2: "失效"}

    def _validation_accept_imu_health(self, values: dict[str, str]) -> None:
        self.validation_imu_health = dict(values)
        self.validation_imu_health_time = time.monotonic()
        self.last_board_rx = time.monotonic()
        self._validation_refresh_readiness()

    def _validation_health_state(self) -> tuple[str, str, bool]:
        """返回 (指示灯状态, 显示文本, 是否允许采集)。"""
        health = self.validation_imu_health
        if not health or self.validation_imu_health_time == 0.0:
            return "wait", "等待固件上报（旧固件不带此项）", True
        if (time.monotonic() - self.validation_imu_health_time) > VALIDATION_HEALTH_FRESH_S:
            return "wait", "上报已过期", True
        level = safe_int(health.get("level"), -1)
        rate = safe_int(health.get("rate_hz"), 0)
        fault = safe_int(health.get("fault"), 0)
        fault_ever = safe_int(health.get("fault_ever"), 0)
        label = self.IMU_HEALTH_LABELS.get(level, f"未知({level})")
        text = f"{label}  {rate} Hz"
        if fault != 0:
            return "bad", f"{text} · DRDY 失效，已禁止解锁", False
        if level >= 1:
            return "bad", f"{text} · 采样链降级，标定数据不可信", False
        if fault_ever != 0:
            return "wait", f"{text} · 本次上电曾降级，建议重新采集", True
        return "ok", text, True

    def _validation_accept_imu_values(self, values: dict[str, str]) -> None:
        sequence = safe_int(values.get("seq"), -1)
        if sequence < 0:
            if values.get("valid") == "0" and values.get("source") == "stabilizer_snapshot":
                self.validation_latest_values.clear()
                self.validation_latest_host_time = 0.0
                self.validation_latest_transport_generation = None
                self.validation_source_var.set("数据源：UNSUPPORTED · 目标尚无有效 stabilizer snapshot")
                self._validation_refresh_readiness()
                self._firmware_refresh_safety()
            return
        cached_sequence = safe_int(self.validation_latest_values.get("seq"), -1)
        if cached_sequence != sequence:
            self.validation_latest_values = {"seq": str(sequence)}
            self.validation_latest_transport_generation = None
        self.validation_latest_values.update({key: value for key, value in values.items() if value != "-"})
        merged = self.validation_latest_values
        for short_name, explicit_name in (
            ("ax", "ax_mg"), ("ay", "ay_mg"), ("az", "az_mg"),
            ("gx", "gx_mdps"), ("gy", "gy_mdps"), ("gz", "gz_mdps"),
            ("roll", "roll_cdeg"), ("pitch", "pitch_cdeg"), ("yaw", "yaw_cdeg"),
        ):
            if short_name in merged and explicit_name not in merged:
                merged[explicit_name] = merged[short_name]
        required = (
            "valid", "source", "frame", "units", "contract", "migration", "ts_ms", "seq",
            "bias", "armed", "m1", "m2", "ax_mg", "ay_mg", "az_mg",
            "gx_mdps", "gy_mdps", "gz_mdps",
        )
        if any(key not in merged for key in required):
            return
        if merged.get("valid") != "1" or merged.get("source") != "stabilizer_snapshot":
            return
        if merged.get("units") != "mg_mdps_cdeg":
            self.validation_source_var.set(f"数据源：UNSUPPORTED units={merged.get('units', '-')}")
            return
        sample, parse_error = validation_sample_from_snapshot(merged)
        if sample is None:
            if parse_error is not None and not parse_error.startswith("incomplete:"):
                self.validation_source_var.set(f"数据源：UNSUPPORTED · {parse_error}")
            return
        timestamp_ms = int(merged["ts_ms"], 0)
        if self.validation_latest_sequence is not None and sequence <= self.validation_latest_sequence:
            return
        self.validation_latest_sequence = sequence
        self.validation_latest_host_time = time.monotonic()
        self.validation_latest_transport_generation = (
            self.serial_transport.connection_generation
            if self.transport is self.serial_transport
            else None
        )
        self.validation_source_var.set(
            f"数据源：snapshot · frame={merged['frame']} contract={merged['contract']} "
            f"migration={merged['migration']} seq={sequence}"
        )
        accel_norm = math.sqrt(sum(value * value for value in sample.accel_g))
        self.validation_live_var.set(
            f"a=({sample.accel_x_g:+.3f}, {sample.accel_y_g:+.3f}, {sample.accel_z_g:+.3f}) g  "
            f"|a|={accel_norm:.3f} g\n"
            f"gyro=({sample.gyro_x_dps:+.2f}, {sample.gyro_y_dps:+.2f}, {sample.gyro_z_dps:+.2f}) dps  "
            f"armed={merged['armed']} m1={merged['m1']} m2={merged['m2']} bias={merged['bias']}"
        )
        self._validation_refresh_readiness()
        self._firmware_refresh_safety()
        if self.validation_active_stage is None:
            return
        sample_safe, safety_reason = self._validation_latest_is_safe()
        if (
            not sample_safe
            or not self.validation_props_removed_var.get()
            or not self.validation_power_safe_var.get()
        ):
            if sample_safe:
                safety_reason = "人工安全确认在采集中被撤销"
            self._validation_abort_active_stage(
                ValidationStatus.FAIL,
                f"逐样本安全门触发：{safety_reason}",
            )
            return
        if self.validation_stage_last_sequence is not None and sequence <= self.validation_stage_last_sequence:
            return
        self.validation_stage_last_sequence = sequence
        stage = self.validation_active_stage
        self.validation_samples[stage].append(sample)
        self.validation_raw_rows.append(
            {
                "stage": stage.value,
                "target_timestamp_ms": timestamp_ms,
                "sequence": sequence,
                "accel_x_g": sample.accel_x_g,
                "accel_y_g": sample.accel_y_g,
                "accel_z_g": sample.accel_z_g,
                "gyro_x_dps": sample.gyro_x_dps,
                "gyro_y_dps": sample.gyro_y_dps,
                "gyro_z_dps": sample.gyro_z_dps,
                "roll_deg": sample.roll_deg,
                "pitch_deg": sample.pitch_deg,
                "yaw_deg": sample.yaw_deg,
                **{key: merged.get(key, "") for key in ("source", "frame", "units", "contract", "migration", "orientation", "bias", "armed", "m1", "m2")},
            }
        )
        self.validation_stage_tree.item(stage.value, values=("COLLECTING", len(self.validation_samples[stage])))
        target = (
            VALIDATION_STATIC_TARGET_SAMPLES
            if STAGE_DEFINITIONS[stage].kind == "static"
            else VALIDATION_ROTATION_TARGET_SAMPLES
        )
        sample_count = len(self.validation_samples[stage])
        self.validation_sample_var.set(f"当前步骤样本：{sample_count} / 建议 {target}")
        self.validation_progress.configure(value=min(100.0, sample_count * 100.0 / target))
        self._validation_autosave_session()

    def _build_imu_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求 IMU", command=lambda: self._send_proto(PROTO_REQ_IMU, "IMU?")).pack(side=tk.LEFT)
        ttk.Checkbutton(top, text=f"备用轮询 {IMU_POLL_PERIOD_MS} ms", variable=self.imu_poll_enabled).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(top, text="仅显示归零", command=self._reset_imu_attitude).pack(side=tk.LEFT, padx=8)
        ttk.Label(top, text="不参与 V0 验收", style="Muted.TLabel").pack(side=tk.LEFT)

        body = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        attitude = ttk.LabelFrame(body, text="人工地平仪", padding=8)
        values = ttk.LabelFrame(body, text="实时数值", padding=10)
        body.add(attitude, weight=3)
        body.add(values, weight=2)

        self.imu_canvas = tk.Canvas(attitude, width=520, height=380, bg="#101820", highlightthickness=0)
        self.imu_canvas.pack(fill=tk.BOTH, expand=True)
        self.imu_canvas.bind("<Configure>", lambda _event: self._draw_imu_attitude())

        fields = [
            ("roll", "Roll deg"),
            ("pitch", "Pitch deg"),
            ("yaw", "Yaw deg"),
            ("ax", "Accel X mg"),
            ("ay", "Accel Y mg"),
            ("az", "Accel Z mg"),
            ("gx", "Gyro X mdps"),
            ("gy", "Gyro Y mdps"),
            ("gz", "Gyro Z mdps"),
            ("temp", "Temp cdeg"),
            ("count", "Sample"),
            ("who", "WHO_AM_I"),
            ("age", "Age"),
        ]
        for row, (key, label) in enumerate(fields):
            self.imu_vars[key] = tk.StringVar(value="-")
            ttk.Label(values, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 10), pady=3)
            ttk.Label(values, textvariable=self.imu_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=3)
        values.columnconfigure(1, weight=1)
        self._draw_imu_attitude()

    def _build_gps_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求 GPS", command=lambda: self._send_proto(PROTO_REQ_GPS, "GPS?")).pack(side=tk.LEFT)
        ttk.Button(top, text="请求磁力计", command=lambda: self._send_proto(PROTO_REQ_MAG, "MAG?")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="清空轨迹", command=self._clear_gps_track).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="导出轨迹 CSV", command=self._export_gps_csv).pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.gps_count_var).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Label(top, textvariable=self.gps_status_var).pack(side=tk.LEFT, padx=(12, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        plot_box = ttk.LabelFrame(left, text="XY 轨迹图 (m)", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True)

        if HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None:
            self.gps_figure = Figure(figsize=(6, 4), dpi=100)
            self.gps_axis = self.gps_figure.add_subplot(111)
            self.gps_canvas = FigureCanvasTkAgg(self.gps_figure, master=plot_box)
            self.gps_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self._update_gps_plot()
        else:
            ttk.Label(
                plot_box,
                text=f"未安装 matplotlib，轨迹图区停用。\npython -m pip install matplotlib\n{MATPLOTLIB_ERROR}",
                wraplength=560,
            ).pack(fill=tk.X, pady=(12, 0))
            self.gps_figure = None
            self.gps_axis = None
            self.gps_canvas = None

        info = ttk.LabelFrame(right, text="GPS / MAG 状态", padding=10)
        info.pack(fill=tk.BOTH, expand=True)

        gps_fields = [
            ("state", "GPS 状态"),
            ("fix", "Fix"),
            ("sv", "卫星数"),
            ("lon", "经度"),
            ("lat", "纬度"),
            ("hmsl", "海拔 mm"),
            ("x", "X (m)"),
            ("y", "Y (m)"),
            ("spd", "地速 mm/s"),
            ("hdg", "航向 deg"),
            ("age", "数据时延"),
        ]
        for row, (key, label) in enumerate(gps_fields):
            self.gps_vars[key] = tk.StringVar(value="-")
            ttk.Label(info, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
            ttk.Label(info, textvariable=self.gps_vars[key]).grid(row=row, column=1, sticky=tk.W, pady=2)

        mag_title = ttk.Label(info, text="MAG", font=("Segoe UI", 10, "bold"))
        mag_title.grid(row=len(gps_fields), column=0, columnspan=2, sticky=tk.W, pady=(10, 2))

        mag_fields = [
            ("state", "磁力计状态"),
            ("type", "型号"),
            ("raw", "Raw xyz"),
            ("scaled", "mGauss xyz"),
        ]
        base = len(gps_fields) + 1
        for row, (key, label) in enumerate(mag_fields):
            self.mag_vars[key] = tk.StringVar(value="-")
            ttk.Label(info, text=label).grid(row=base + row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
            ttk.Label(info, textvariable=self.mag_vars[key]).grid(row=base + row, column=1, sticky=tk.W, pady=2)

        info.columnconfigure(1, weight=1)

    def _build_ident_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="ARM 辨识", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT ARM")).pack(side=tk.LEFT)
        ttk.Button(top, text="DISARM", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT DISARM")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="STOP", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT STOP")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="STATUS", command=lambda: self._send_proto(PROTO_REQ_IDENT, "IDENT?")).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="AIRFRAME", command=self._request_airframe).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, text="状态").pack(side=tk.LEFT, padx=(18, 4))
        ttk.Label(top, textvariable=self.ident_status_var).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.ident_sample_count_var).pack(side=tk.LEFT, padx=4)
        ttk.Label(top, textvariable=self.ident_reason_var).pack(side=tk.LEFT, padx=4)

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=2)
        panes.add(right, weight=3)

        cfg = ttk.LabelFrame(left, text="Experiment", padding=10)
        cfg.pack(fill=tk.X)
        cfg.columnconfigure(1, weight=1)
        ttk.Label(cfg, text="辨识轴").grid(row=0, column=0, sticky=tk.W, pady=3)
        ttk.Combobox(cfg, textvariable=self.ident_axis_var, values=("roll", "pitch"), width=12, state="readonly").grid(row=0, column=1, sticky=tk.W, pady=3)
        ttk.Label(cfg, text="波形").grid(row=1, column=0, sticky=tk.W, pady=3)
        ttk.Combobox(cfg, textvariable=self.ident_mode_var, values=("STEP", "DOUBLET", "PRBS"), width=12, state="readonly").grid(row=1, column=1, sticky=tk.W, pady=3)

        presets = ttk.Frame(cfg)
        presets.grid(row=2, column=0, columnspan=3, sticky=tk.EW, pady=(6, 8))
        ttk.Button(presets, text="小幅首测", command=lambda: self._ident_apply_preset("small")).pack(side=tk.LEFT)
        ttk.Button(presets, text="标准阶跃", command=lambda: self._ident_apply_preset("step")).pack(side=tk.LEFT, padx=4)
        ttk.Button(presets, text="标准双脉冲", command=lambda: self._ident_apply_preset("doublet")).pack(side=tk.LEFT)

        field_specs = [
            ("pulse", "舵机偏置", self.ident_pulse_var, "us，正负都可；越大响应越明显"),
            ("duration", "阶跃/PRBS 时长", self.ident_duration_var, "ms，最大 10000"),
            ("hold", "双脉冲保持", self.ident_hold_var, "ms，每段保持时间"),
            ("repeat", "双脉冲次数", self.ident_repeat_var, "次，默认 2"),
            ("bit", "PRBS 位宽", self.ident_bit_var, "ms，随机输入切换间隔"),
            ("seed", "PRBS seed", self.ident_seed_var, "相同 seed 可复现实验"),
            ("alpha_center", "alpha 中位", self.ident_alpha_center_var, "us，roll/1号左右舵机中心"),
            ("beta_center", "beta 中位", self.ident_beta_center_var, "us，pitch/2号前后舵机中心"),
        ]
        self.ident_field_rows.clear()
        for index, (key, label, var, hint) in enumerate(field_specs, start=3):
            label_widget = ttk.Label(cfg, text=label)
            entry = ttk.Entry(cfg, textvariable=var, width=12)
            hint_widget = ttk.Label(cfg, text=hint, style="Muted.TLabel")
            label_widget.grid(row=index, column=0, sticky=tk.W, pady=3)
            entry.grid(row=index, column=1, sticky=tk.W, pady=3)
            hint_widget.grid(row=index, column=2, sticky=tk.W, padx=(8, 0), pady=3)
            self.ident_field_rows[key] = (label_widget, entry, hint_widget)

        ttk.Label(cfg, textvariable=self.ident_mode_hint_var, wraplength=520, style="Muted.TLabel").grid(
            row=11, column=0, columnspan=3, sticky=tk.EW, pady=(8, 2)
        )
        ttk.Label(cfg, textvariable=self.ident_command_preview_var, wraplength=520, font=("Consolas", 9)).grid(
            row=12, column=0, columnspan=3, sticky=tk.EW, pady=(2, 6)
        )
        ttk.Button(cfg, text="Set Center", command=self._ident_send_center).grid(row=13, column=0, sticky=tk.EW, pady=(4, 0))
        ttk.Button(cfg, text="Run", command=self._ident_run).grid(row=13, column=1, sticky=tk.EW, pady=(4, 0))
        ttk.Label(cfg, textvariable=self.ident_safety_hint_var, wraplength=520, style="Warn.TLabel").grid(
            row=14, column=0, columnspan=3, sticky=tk.EW, pady=(8, 0)
        )

        fit = ttk.LabelFrame(left, text="Fit / PID", padding=10)
        fit.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(fit, textvariable=self.ident_fit_var, wraplength=320).pack(fill=tk.X)
        actions = ttk.Frame(fit)
        actions.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions, text="Fit", command=self._ident_fit).pack(side=tk.LEFT)
        ttk.Button(actions, text="Apply", command=self._ident_apply_fit).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Save", command=lambda: self._send_proto_once(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT)

        live = ttk.LabelFrame(left, text="Live", padding=10)
        live.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(live, textvariable=self.ident_link_var, wraplength=520).pack(fill=tk.X, pady=(0, 4))
        ttk.Label(live, textvariable=self.ident_airframe_var, wraplength=320).pack(fill=tk.X, pady=(0, 8))
        ttk.Label(live, textvariable=self.ident_current_var, wraplength=320).pack(fill=tk.X)
        ttk.Label(live, textvariable=self.ident_output_dir_var, wraplength=520).pack(fill=tk.X, pady=(8, 0))
        ttk.Label(live, textvariable=self.ident_last_file_var, wraplength=520).pack(fill=tk.X, pady=(2, 0))
        actions_live = ttk.Frame(live)
        actions_live.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(actions_live, text="Open CSV Folder", command=self._ident_open_folder).pack(side=tk.LEFT)
        ttk.Button(actions_live, text="Pause VOFA", command=lambda: self._send("Sensor_Data:0")).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions_live, text="Resume VOFA", command=lambda: self._send("Sensor_Data:1")).pack(side=tk.LEFT)

        plot_box = ttk.LabelFrame(right, text="Input / Response", padding=8)
        plot_box.pack(fill=tk.BOTH, expand=True)
        if HAS_MATPLOTLIB and Figure is not None and FigureCanvasTkAgg is not None:
            self.ident_figure = Figure(figsize=(6, 4), dpi=100)
            self.ident_axis_plot = self.ident_figure.add_subplot(111)
            self.ident_canvas = FigureCanvasTkAgg(self.ident_figure, master=plot_box)
            self.ident_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        else:
            ttk.Label(
                plot_box,
                text=f"matplotlib unavailable; CSV logging still works.\npython -m pip install matplotlib\n{MATPLOTLIB_ERROR}",
                wraplength=560,
            ).pack(fill=tk.X, pady=(12, 0))
            self.ident_figure = None
            self.ident_axis_plot = None
            self.ident_canvas = None

    def _build_params_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="读取参数/PID", command=self._read_all_params).pack(side=tk.LEFT)
        ttk.Button(top, text="保存到 Flash", command=lambda: self._send_proto_once(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="从 Flash 读取", command=lambda: self._send_proto_once(PROTO_REQ_LOAD, "LOAD")).pack(side=tk.LEFT)
        ttk.Button(top, text="恢复默认", command=lambda: self._send_proto(PROTO_REQ_DEFAULTS, "DEFAULTS")).pack(side=tk.LEFT, padx=6)
        ttk.Label(top, textvariable=self.config_summary_var).pack(side=tk.LEFT, padx=(14, 0))

        panes = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        panes.add(left, weight=3)
        panes.add(right, weight=2)

        self.param_tree = ttk.Treeview(left, columns=("value", "source", "dirty"), show="tree headings")
        self.param_tree.heading("#0", text="参数")
        self.param_tree.column("#0", width=220, anchor=tk.W)
        for col, label, width in [("value", "值", 150), ("source", "来源", 100), ("dirty", "待发送", 80)]:
            self.param_tree.heading(col, text=label)
            self.param_tree.column(col, width=width, anchor=tk.W)
        self.param_tree.pack(fill=tk.BOTH, expand=True)
        self.param_tree.bind("<<TreeviewSelect>>", self._on_param_select)

        edit = ttk.LabelFrame(right, text="参数编辑", padding=10)
        edit.pack(fill=tk.X)
        self.param_name_var = tk.StringVar()
        self.param_value_var = tk.StringVar()
        ttk.Label(edit, text="名称").grid(row=0, column=0, sticky=tk.W, pady=4)
        ttk.Entry(edit, textvariable=self.param_name_var).grid(row=0, column=1, sticky=tk.EW, pady=4)
        ttk.Label(edit, text="值").grid(row=1, column=0, sticky=tk.W, pady=4)
        ttk.Entry(edit, textvariable=self.param_value_var).grid(row=1, column=1, sticky=tk.EW, pady=4)
        actions = ttk.Frame(edit)
        actions.grid(row=2, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Button(actions, text="暂存修改", command=self._stage_param_edit).pack(side=tk.LEFT)
        ttk.Button(actions, text="发送修改", command=self._send_param_edit).pack(side=tk.LEFT, padx=6)
        edit.columnconfigure(1, weight=1)

        pid = ttk.LabelFrame(right, text="PID 快速编辑", padding=10)
        pid.pack(fill=tk.X, pady=(10, 0))
        self.pid_vars: dict[str, dict[str, tk.StringVar]] = {}
        ttk.Label(pid, text="轴").grid(row=0, column=0, sticky=tk.W)
        for col, term in enumerate(("kp", "ki", "kd"), start=1):
            ttk.Label(pid, text=term.upper()).grid(row=0, column=col, sticky=tk.W)
        for row, axis in enumerate(("roll", "pitch", "yaw"), start=1):
            ttk.Label(pid, text=axis).grid(row=row, column=0, sticky=tk.W, pady=4)
            self.pid_vars[axis] = {}
            for col, term in enumerate(("kp", "ki", "kd"), start=1):
                var = tk.StringVar(value="")
                self.pid_vars[axis][term] = var
                ttk.Entry(pid, textvariable=var, width=10).grid(row=row, column=col, sticky=tk.EW, padx=(4, 0), pady=4)
        ttk.Button(pid, text="发送 PID", command=self._send_pid_values).grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Button(pid, text="读取 PID", command=lambda: self._send_proto(PROTO_REQ_PID, "PID?")).grid(row=4, column=2, columnspan=2, sticky=tk.EW, padx=(6, 0), pady=(8, 0))
        for col in range(1, 4):
            pid.columnconfigure(col, weight=1)

    def _build_servo_page(self, parent: ttk.Frame) -> None:
        servo_notebook = ttk.Notebook(parent)
        servo_notebook.pack(fill=tk.BOTH, expand=True)
        for index in range(2):
            frame = ttk.Frame(servo_notebook, padding=10)
            servo_notebook.add(frame, text=f"舵机 {index}")
            self._build_servo_tab(frame, index)

        raw = ttk.LabelFrame(parent, text="手动原始舵机指令", padding=10)
        raw.pack(fill=tk.X, pady=(10, 0))
        self.raw_var = tk.StringVar(value="{#001P1500T0500!#002P1500T0500!}")
        ttk.Entry(raw, textvariable=self.raw_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(raw, text="发送原始指令", command=self._send_raw).pack(side=tk.LEFT, padx=(6, 0))

    def _build_command_page(self, parent: ttk.Frame) -> None:
        quick = ttk.LabelFrame(parent, text="兼容 / 调试命令", padding=10)
        quick.pack(fill=tk.X)
        for label, command in [
            ("PING", "PING"),
            ("STATUS?", "STATUS?"),
            ("CONFIG?", "CONFIG?"),
            ("PARAM?", "PARAM?"),
            ("AIRFRAME?", "AIRFRAME?"),
            ("PID?", "PID?"),
            ("BARO?", "BARO?"),
            ("GPS?", "GPS?"),
            ("MAG?", "MAG?"),
            ("SAVE", "SAVE"),
            ("LOAD", "LOAD"),
        ]:
            ttk.Button(quick, text=label, command=lambda c=command: self._send(c)).pack(side=tk.LEFT, padx=3)

        help_box = ttk.LabelFrame(parent, text="兼容格式", padding=10)
        help_box.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        text = tk.Text(help_box, height=10, wrap=tk.WORD)
        text.configure(
            background=UI_PALETTE["panel"], foreground=UI_PALETTE["ink_dim"],
            relief=tk.FLAT, highlightthickness=1,
            highlightbackground=UI_PALETTE["border"], padx=10, pady=8,
        )
        text.pack(fill=tk.BOTH, expand=True)
        text.insert(
            tk.END,
            "面板会解析现有固件行: READY, HW FLASH/SPL06/ICM42688, STATUS flash/baro/imu, UART1, CFG, OK/ERR。\n"
            "也预留解析: BARO pressure=... temp=... alt=..., PARAM name=... value=..., PID axis=roll kp=...。\n"
            "未知行不会报错，会保留在原始命令日志里，方便固件侧逐步补命令。\n",
        )
        text.configure(state=tk.DISABLED)

    def _build_log_area(self, parent: ttk.Frame) -> None:
        text_frame = ttk.Frame(parent)
        text_frame.pack(fill=tk.BOTH, expand=True)
        self.status_text = tk.Text(text_frame, height=9, wrap=tk.NONE)
        self.status_text.configure(
            font=(UI_MONO, UI_SIZE), background=UI_PALETTE["console"],
            foreground=UI_PALETTE["ink_dim"],
            insertbackground=UI_PALETTE["accent"],
            selectbackground=UI_PALETTE["accent_soft"], relief=tk.FLAT,
            padx=8, pady=6,
        )
        yscroll = ttk.Scrollbar(text_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        xscroll = ttk.Scrollbar(text_frame, orient=tk.HORIZONTAL, command=self.status_text.xview)
        self.status_text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        self.status_text.grid(row=0, column=0, sticky=tk.NSEW)
        yscroll.grid(row=0, column=1, sticky=tk.NS)
        xscroll.grid(row=1, column=0, sticky=tk.EW)
        text_frame.rowconfigure(0, weight=1)
        text_frame.columnconfigure(0, weight=1)

        cmd_frame = ttk.Frame(parent)
        cmd_frame.pack(fill=tk.X, pady=(8, 0))
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var)
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        cmd_entry.bind("<Return>", lambda _event: self._send_custom())
        ttk.Button(cmd_frame, text="发送", command=self._send_custom,
                   style="Primary.TButton").pack(side=tk.LEFT, padx=(6, 0))

    def _build_servo_tab(self, parent: ttk.Frame, index: int) -> None:
        values: dict[str, tk.Variable] = {
            "id": tk.IntVar(value=index + 1),
            "pulse": tk.IntVar(value=1500),
            "time": tk.IntVar(value=500),
            "mode": tk.IntVar(value=1),
            "enabled": tk.IntVar(value=1),
            "new_id": tk.IntVar(value=index + 1),
            "baud": tk.IntVar(value=4),
        }
        self.servo_widgets.append(values)

        row = 0
        ttk.Checkbutton(
            parent,
            text="启用此舵机槽位",
            variable=values["enabled"],
            command=lambda i=index: self._servo_enable(i),
        ).grid(row=row, column=0, sticky=tk.W)
        row += 1
        self._spin(parent, row, "当前舵机 ID", values["id"], 0, 255, lambda i=index: self._servo_set_id(i))
        row += 1
        self._scale(parent, row, "目标位置 us", values["pulse"], 500, 2500)
        row += 1
        self._spin(parent, row, "运行时间 ms", values["time"], 0, 9999, None)
        row += 1
        self._spin(parent, row, "模式 1-8", values["mode"], 1, 8, lambda i=index: self._servo_mode(i))
        row += 1
        ttk.Button(parent, text="移动此舵机", command=lambda i=index: self._servo_move(i)).grid(row=row, column=0, pady=6, sticky=tk.EW)
        ttk.Button(
            parent,
            text="按配置同时移动两路",
            command=lambda: self._send_proto(PROTO_REQ_SERVO_MOVE_ALL, "SERVO MOVEALL"),
        ).grid(row=row, column=1, pady=6, sticky=tk.EW)
        row += 1

        id_box = ttk.LabelFrame(parent, text="修改实体舵机 ID", padding=8)
        id_box.grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=(8, 4))
        ttk.Spinbox(id_box, from_=0, to=255, textvariable=values["new_id"], width=8).pack(side=tk.LEFT)
        ttk.Button(id_box, text="写入新 ID", command=lambda i=index: self._servo_set_physical_id(i)).pack(side=tk.LEFT, padx=6)
        row += 1

        actions = ttk.LabelFrame(parent, text="众灵手册动作指令", padding=8)
        actions.grid(row=row, column=0, columnspan=2, sticky=tk.EW)
        action_names = [
            ("读取版本", "VER"),
            ("检测 ID", "PID"),
            ("读取位置", "RAD"),
            ("读取模式", "MOD?"),
            ("释放扭力", "ULK"),
            ("恢复扭力", "ULR"),
            ("暂停", "DPT"),
            ("继续", "DCT"),
            ("停止", "DST"),
            ("当前位置设中位", "SCK"),
            ("设置启动位置", "CSD"),
            ("清除启动位置", "CSM"),
            ("恢复启动位置", "CSR"),
            ("设置最小值", "SMI"),
            ("设置最大值", "SMX"),
            ("半恢复出厂", "CLEO"),
            ("全恢复出厂", "CLE"),
        ]
        for n, (label, command) in enumerate(action_names):
            ttk.Button(actions, text=label, command=lambda i=index, c=command: self._servo_cmd(i, c)).grid(
                row=n // 2,
                column=n % 2,
                padx=3,
                pady=3,
                sticky=tk.EW,
            )
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)

        baud_box = ttk.Frame(parent)
        baud_box.grid(row=row + 1, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Label(baud_box, text="波特率代码").pack(side=tk.LEFT)
        ttk.Spinbox(baud_box, from_=0, to=7, textvariable=values["baud"], width=5).pack(side=tk.LEFT, padx=6)
        ttk.Button(baud_box, text="设置波特率", command=lambda i=index: self._servo_baud(i)).pack(side=tk.LEFT)

        parent.columnconfigure(1, weight=1)

    def _spin(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: int, maximum: int, command) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        box = ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=10)
        box.grid(row=row, column=1, sticky=tk.EW, pady=4)
        if command is not None:
            ttk.Button(parent, text="应用", command=command).grid(row=row, column=2, padx=(6, 0))

    def _scale(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: int, maximum: int) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        scale = ttk.Scale(parent, from_=minimum, to=maximum, variable=variable, orient=tk.HORIZONTAL)
        scale.grid(row=row, column=1, sticky=tk.EW, pady=4)
        ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=8).grid(row=row, column=2, padx=(6, 0))

    def _start(self) -> None:
        if self.v1_worker is not None and self.v1_worker.is_alive():
            messagebox.showwarning("V1 正在占用 USB CDC", "请先完成当前 V1 采集，再启动面板连接。")
            return
        self._stop()
        self.transport = self._current_transport()
        if self.transport is self.tcp_transport:
            try:
                port = int(self.port_var.get())
            except tk.TclError:
                messagebox.showerror("输入错误", "端口号无效")
                return
            self.transport.start(self.host_var.get(), port)
            return

        if self.transport is self.udp_transport:
            try:
                local_port = int(self.udp_local_port_var.get())
                module_port = int(self.udp_module_port_var.get())
            except tk.TclError:
                messagebox.showerror("输入错误", "UDP 端口号无效")
                return
            module_ip = self.udp_module_ip_var.get().strip()
            if not module_ip:
                messagebox.showerror("输入错误", "请输入 Ai-WB2 模块 IP")
                return
            self.transport.start(self.udp_bind_var.get(), local_port, module_ip, module_port)
            self.structured_protocol_supported = False
            self.after(250, lambda: self.transport.send_line("PING") if self.transport is self.udp_transport else None)
            self.after(450, lambda: self.transport.send_line("AIRFRAME?") if self.transport is self.udp_transport else None)
            return

        try:
            baud = int(self.serial_baud_var.get())
        except tk.TclError:
            messagebox.showerror("输入错误", "波特率无效")
            return
        port_display = self.serial_port_var.get().strip()
        if not port_display:
            messagebox.showerror("输入错误", "请输入串口号")
            return
        port_name = self._serial_port_map.get(port_display, port_display)
        self.transport.start(port_name, baud)
        self._refresh_serial_selection_lock()

    def _stop(self) -> None:
        if self.v1_worker is not None and self.v1_worker.is_alive():
            self.v1_cancel_event.set()
        self.tcp_transport.stop()
        self.udp_transport.stop()
        self.serial_transport.stop()
        self._refresh_serial_selection_lock()
        self.transport = self._current_transport()
        self.last_board_rx = 0.0
        self.last_reply_rx = 0.0
        self.last_keepalive_tx = 0.0
        self.imu_last_sample_time = 0.0
        self.validation_latest_host_time = 0.0
        self.validation_latest_values.clear()
        self.validation_latest_transport_generation = None
        self.validation_poll_requested = False
        if self.validation_session_active:
            if self.validation_active_stage is not None:
                self._validation_abort_active_stage(
                    ValidationStatus.UNSUPPORTED,
                    "连接停止，当前步骤证据作废",
                )
            self.validation_session_active = False
            self._validation_set_status(ValidationStatus.UNSUPPORTED, "连接已停止，验收会话暂停")
        self._validation_autosave_session(force=True)
        self._firmware_refresh_safety()

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
        messagebox.showwarning("V0 只读门", f"验收会话期间禁止执行：{payload}")
        return False

    def _send(self, line: str) -> None:
        if not self._validation_guard_command(line):
            return
        self.last_cmd_var.set(f"最近命令: {line}")
        self._append(f"> {line}")
        if not self.transport.send_line(line):
            self._append("[上位机] 发送失败")
        elif line in {"PING", "STATUS?", "CONFIG?", "PARAM?", "PID?", "BARO?", "GPS?", "MAG?", "AIRFRAME?"}:
            sent_at = time.monotonic()
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=line, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto(self, function: int, label: str, payload: str = "", expect_reply: bool = True) -> None:
        payload_text = payload if payload else label
        if not self._validation_guard_command(payload_text):
            return
        self.last_cmd_var.set(f"最近命令: {label}")
        self._append(f"> {label}")
        if not self.transport.send_frame(function, payload_text.encode("utf-8")):
            self._append("[上位机] 发送失败")
            return
        sent_at = time.monotonic()
        if self.transport is self.udp_transport and label in {"PING", "AIRFRAME?", "IDENT?", "STATUS?"}:
            self.ident_link_var.set(f"sent {label}; waiting for text reply")
        if self.transport is self.udp_transport:
            pass
        elif self.structured_protocol_supported is False and not (self.transport is self.serial_transport and SERIAL_ASCII_COMPAT_MODE):
            self.transport.send_line(payload_text)
        elif self.structured_protocol_supported is None and function != PROTO_REQ_CAPS and self.transport is not self.serial_transport:
            self.after(
                PROTO_COMPAT_FALLBACK_DELAY_MS,
                lambda legacy=payload_text, start=sent_at: self._fallback_proto_request(legacy, start),
            )
        if expect_reply:
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=label, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto_once(self, function: int, label: str, payload: str = "", expect_reply: bool = True) -> None:
        payload_text = payload if payload else label
        if not self._validation_guard_command(payload_text):
            return
        self.last_cmd_var.set(f"最近命令: {label}")
        self._append(f"> {label}")
        if not self.transport.send_frame(function, payload_text.encode("utf-8")):
            self._append("[上位机] 发送失败")
            return
        if expect_reply:
            sent_at = time.monotonic()
            self.after(CMD_REPLY_TIMEOUT_MS, lambda sent=label, start=sent_at: self._warn_if_no_reply(sent, start))

    def _send_proto_silent(self, function: int, payload: str) -> bool:
        if not self._transport_connected():
            return False
        if not self._validation_command_allowed(payload):
            return False
        return self.transport.send_frame(function, payload.encode("utf-8"))

    def _fallback_proto_request(self, legacy_line: str, started_at: float) -> None:
        if not self._transport_connected():
            return
        if self.structured_protocol_supported is True:
            return
        if self.last_reply_rx >= started_at:
            return
        if not self._validation_command_allowed(legacy_line):
            self._append(f"[V0 READ-ONLY] blocked delayed fallback: {legacy_line}")
            return
        self.transport.send_line(legacy_line)

    def _begin_protocol_probe(self) -> None:
        if self.transport is self.udp_transport:
            self.structured_protocol_supported = False
            return
        if self.transport is self.serial_transport and SERIAL_ASCII_COMPAT_MODE:
            self.structured_protocol_supported = False
            return
        self.structured_protocol_supported = None
        if not self._transport_connected():
            return

        started_at = time.monotonic()
        self.transport.send_frame(PROTO_REQ_CAPS, b"CAPS?")
        self.after(
            PROTO_COMPAT_FALLBACK_DELAY_MS,
            lambda start=started_at: self._fallback_proto_request("CAPS?", start),
        )
        self.after(PROTO_PROBE_SETTLE_MS, self._finalize_protocol_probe)

    def _finalize_protocol_probe(self) -> None:
        if not self._transport_connected():
            self.structured_protocol_supported = None
            return
        if self.structured_protocol_supported is None:
            self.structured_protocol_supported = False

    def _request_overview_status(self) -> None:
        self._send_proto(PROTO_REQ_MODULES, "MODULES?")
        self._send_proto(PROTO_REQ_STATUS, "STATUS?")

    def _send_custom(self) -> None:
        line = self.cmd_var.get().strip()
        if line:
            self._send(line)
            if self.transport is self.udp_transport and line in {"PING", "AIRFRAME?", "IDENT?", "STATUS?"}:
                self.ident_link_var.set(f"sent {line}; waiting for text reply")
            self.cmd_var.set("")

    def _read_all_params(self) -> None:
        self._send_proto(PROTO_REQ_CONFIG, "CONFIG?")
        self._send_proto(PROTO_REQ_PARAMS, "PARAM?")
        self._send_proto(PROTO_REQ_PID, "PID?")
        self._request_airframe()

    def _request_airframe(self) -> None:
        self._send_proto(PROTO_REQ_AIRFRAME, "AIRFRAME?")

    def _send_raw(self) -> None:
        payload = f"SERVO RAW {self.raw_var.get().strip()}"
        self._send_proto(PROTO_REQ_SERVO_RAW, payload, payload)

    def _servo_values(self, index: int) -> dict[str, int]:
        widgets = self.servo_widgets[index]
        return {key: int(var.get()) for key, var in widgets.items()}

    def _servo_move(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO MOVE {index} {values['pulse']} {values['time']}"
        self._send_proto(PROTO_REQ_SERVO_MOVE, payload, payload)

    def _servo_mode(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO MODE {index} {values['mode']}"
        self._send_proto(PROTO_REQ_SERVO_MODE, payload, payload)

    def _servo_enable(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO ENABLE {index} {values['enabled']}"
        self._send_proto(PROTO_REQ_SERVO_ENABLE, payload, payload)

    def _servo_set_id(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO ID {index} {values['id']}"
        self._send_proto(PROTO_REQ_SERVO_ID, payload, payload)

    def _servo_set_physical_id(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO SETID {index} {values['new_id']}"
        self._send_proto(PROTO_REQ_SERVO_SETID, payload, payload)

    def _servo_cmd(self, index: int, command: str) -> None:
        payload = f"SERVO CMD {index} {command}"
        self._send_proto(PROTO_REQ_SERVO_ACTION, payload, payload)

    def _servo_baud(self, index: int) -> None:
        values = self._servo_values(index)
        payload = f"SERVO CMD {index} BD {values['baud']}"
        self._send_proto(PROTO_REQ_SERVO_ACTION, payload, payload)

    MAX_LOG_LINES = 2000

    def _should_show_raw_log(self, line: str) -> bool:
        if line.startswith(("READY", "UART1 ")):
            return False
        if line.startswith(("GPS_USART2 ", "MAG_I2C1 ")):
            return False
        if line.startswith("BARO ok="):
            return False
        if line.startswith("[host] UDP RX "):
            return False
        if line.startswith("RXRAW "):
            return False
        if re.search(r"(^|[ ,])ax=", line) and re.search(r"(^|[ ,])az=", line):
            return False
        return True

    def _append(self, line: str) -> None:
        if not self._should_show_raw_log(line):
            return
        stamp = time.strftime("%H:%M:%S")
        self._pending_log_lines.append(f"[{stamp}] {line}\n")
        if len(self._pending_log_lines) > MAX_PENDING_LOG_LINES:
            del self._pending_log_lines[: len(self._pending_log_lines) - MAX_PENDING_LOG_LINES]

    def _flush_log(self) -> None:
        if self._pending_log_lines:
            batch = "".join(self._pending_log_lines)
            self._pending_log_lines.clear()
            self.status_text.insert(tk.END, batch)
            line_count = int(self.status_text.index("end-1c").split(".")[0])
            excess = line_count - self.MAX_LOG_LINES
            if excess > 0:
                self.status_text.delete("1.0", f"{excess + 1}.0")
            self.status_text.see(tk.END)
        self.after(LOG_FLUSH_PERIOD_MS, self._flush_log)

    def _transport_label(self) -> str:
        if self.transport is self.tcp_transport:
            return "TCP"
        if self.transport is self.udp_transport:
            return "UDP"
        return "串口"

    def _handle_proto_frame(self, function: int, text: str) -> None:
        self.last_board_rx = time.monotonic()
        self.link_var.set("已收到 STM32 数据，TCP 与串口透明链路正常")
        if function >= PROTO_MSG_PONG:
            self.structured_protocol_supported = True

        display = self._normalize_proto_line(function, text)

        self._append(display)

        if function in {
            PROTO_MSG_CMD_RX,
            PROTO_MSG_CMD_ACK,
            PROTO_MSG_CMD_ERR,
            PROTO_MSG_CMD_OK,
            PROTO_MSG_PONG,
        }:
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {display}")
            return

        if function == PROTO_MSG_READY:
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {display}")
            self._handle_ready_line(text)
            return

        typed_handlers = {
            PROTO_MSG_HW_FLASH: self._update_hardware_line,
            PROTO_MSG_HW_BARO: self._update_hardware_line,
            PROTO_MSG_HW_IMU: self._update_hardware_line,
            PROTO_MSG_STATUS_FLASH: self._update_status_line,
            PROTO_MSG_STATUS_BARO: self._update_status_line,
            PROTO_MSG_STATUS_IMU: self._update_status_line,
            PROTO_MSG_UART_STATS: self._update_uart_line,
            PROTO_MSG_CONFIG_SUMMARY: self._update_config_line,
            PROTO_MSG_CONFIG_SERVO: self._update_config_line,
            PROTO_MSG_PARAM_RECORD: self._update_param_line,
            PROTO_MSG_PID_RECORD: self._update_pid_line,
            PROTO_MSG_FLASH_RECORD: self._update_flash_line,
            PROTO_MSG_BARO_STATE: self._update_baro_line,
            PROTO_MSG_BARO_DIAG: self._handle_board_line,
            PROTO_MSG_BARO_RAW: self._handle_board_line,
            PROTO_MSG_BARO_STREAM: self._update_baro_line,
            PROTO_MSG_IMU_STATE: self._update_imu_line,
            PROTO_MSG_IMU_SCALED: self._update_imu_line,
            PROTO_MSG_MODULES_SUMMARY: self._handle_board_line,
            PROTO_MSG_CAPS_RECORD: self._handle_board_line,
            PROTO_MSG_SAVE_RESULT: self._handle_board_line,
            PROTO_MSG_LOAD_RESULT: self._handle_board_line,
            PROTO_MSG_DEFAULTS_RESULT: self._handle_board_line,
            PROTO_MSG_SERVO_RESULT: self._handle_board_line,
            PROTO_MSG_WIFI_RECORD: self._update_wifi_line,
            PROTO_MSG_GPS_RECORD: self._handle_board_line,
            PROTO_MSG_MAG_RECORD: self._handle_board_line,
            PROTO_MSG_AIRFRAME_RECORD: self._handle_board_line,
            PROTO_MSG_TEXT_LINE: self._handle_board_line,
        }

        handler = typed_handlers.get(function)
        if handler is not None:
            handler(display)
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {display}")

    def _normalize_proto_line(self, function: int, text: str) -> str:
        stripped = text.strip()
        if function == PROTO_MSG_TEXT_LINE:
            return stripped
        if function == PROTO_MSG_CMD_RX:
            return self._ensure_line_prefix(stripped, "RX")
        if function == PROTO_MSG_CMD_ACK:
            return self._ensure_line_prefix(stripped, "ACK")
        if function == PROTO_MSG_CMD_ERR:
            return self._ensure_line_prefix(stripped, "ERR")
        if function == PROTO_MSG_CMD_OK:
            return self._ensure_line_prefix(stripped, "OK")
        if function == PROTO_MSG_PONG:
            return self._ensure_line_prefix(stripped, "PONG")
        if function == PROTO_MSG_READY:
            return self._ensure_line_prefix(stripped, "READY")
        if function == PROTO_MSG_WIFI_RECORD:
            if stripped.startswith(("WIFI ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "WIFI")
        if function == PROTO_MSG_GPS_RECORD:
            if stripped.startswith(("GPS ", "GPS_USART2 ", "M9N ", "HW ", "STATUS ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "GPS")
        if function == PROTO_MSG_MAG_RECORD:
            if stripped.startswith(("MAG ", "MAG_I2C1 ", "HW ", "STATUS ", "RSP ")):
                return stripped
            return self._ensure_line_prefix(stripped, "MAG")

        prefix_map = {
            PROTO_MSG_HW_FLASH: "HW FLASH",
            PROTO_MSG_HW_BARO: "HW SPL06",
            PROTO_MSG_HW_IMU: "HW ICM42688",
            PROTO_MSG_STATUS_FLASH: "STATUS flash",
            PROTO_MSG_STATUS_BARO: "STATUS baro",
            PROTO_MSG_STATUS_IMU: "STATUS imu",
            PROTO_MSG_UART_STATS: "UART1",
            PROTO_MSG_CONFIG_SUMMARY: "CFG",
            PROTO_MSG_CONFIG_SERVO: "CFG",
            PROTO_MSG_PARAM_RECORD: "PARAM",
            PROTO_MSG_PID_RECORD: "PID",
            PROTO_MSG_FLASH_RECORD: "FLASH",
            PROTO_MSG_BARO_STATE: "BARO",
            PROTO_MSG_BARO_DIAG: "BARO",
            PROTO_MSG_BARO_RAW: "BARO",
            PROTO_MSG_BARO_STREAM: "BARO",
            PROTO_MSG_RTOS_RECORD: "RTOS",
            PROTO_MSG_FLASH_BENCH: "FLASH",
            PROTO_MSG_AIRFRAME_RECORD: "AIRFRAME",
        }
        prefix = prefix_map.get(function)
        if prefix is None:
            return stripped if stripped else f"fn=0x{function:04X}"
        return self._ensure_line_prefix(stripped, prefix)

    def _ensure_line_prefix(self, text: str, prefix: str) -> str:
        stripped = text.strip()
        if not stripped:
            return prefix
        if stripped.startswith(prefix):
            return stripped
        return f"{prefix} {stripped}"

    def _update_servo_ok_line(self, line: str) -> None:
        values = parse_kv(line)
        parts = line.split()
        if len(parts) < 2 or not parts[1].startswith("servo"):
            return
        index = safe_int(parts[1].replace("servo", ""), -1)
        if index < 0:
            return

        field_map = {
            "id": "id",
            "enabled": "enabled",
            "mode": "mode",
            "pulse": "pulse",
            "time": "time",
        }
        for src, dst in field_map.items():
            if src in values:
                self._set_param(f"servo{index}.{dst}", values[src], "OK", dirty=False)
        if 0 <= index < len(self.servo_widgets):
            widgets = self.servo_widgets[index]
            for src, dst in field_map.items():
                if src in values and dst in widgets:
                    widgets[dst].set(safe_int(values[src]))

    def _update_config_result_line(self, line: str) -> None:
        values = parse_kv(line)
        status = values.get("st", "-")
        if line.startswith("OK save"):
            if status == "0":
                self._set_param("config.valid", "1", "SAVE", dirty=False)
            self._set_param("config.last_flash_status", status, "SAVE", dirty=False)
        elif line.startswith("OK load"):
            if status == "0":
                self._set_param("config.loaded", "1", "LOAD", dirty=False)
            self._set_param("config.last_flash_status", status, "LOAD", dirty=False)
        elif line.startswith("OK defaults"):
            self._set_param("config.loaded", "0", "DEFAULTS", dirty=False)
            self._set_param("config.valid", "0", "DEFAULTS", dirty=False)
        self._send_proto_once(PROTO_REQ_CONFIG, "CONFIG?")

    def _on_module_select(self, _event: tk.Event) -> None:
        selection = self.module_tree.selection()
        if not selection:
            return
        self.selected_module = selection[0]
        self._refresh_detail()

    def _request_selected_module(self) -> None:
        if self.selected_module == "FLASH":
            self._send_proto(PROTO_REQ_FLASH, "FLASH?")
            return
        if self.selected_module == "SPL06":
            self._send_proto(PROTO_REQ_BARO, "BARO?")
            return
        if self.selected_module == "ICM42688":
            self._send_proto(PROTO_REQ_IMU, "IMU?")
            return
        if self.selected_module == "GPS":
            self._send_proto(PROTO_REQ_GPS, "GPS?")
            return
        if self.selected_module == "MAG":
            self._send_proto(PROTO_REQ_MAG, "MAG?")
            return
        if self.selected_module == "WIFI":
            self._send_proto(PROTO_REQ_WIFI, "WIFI?")
            return
        self._send_proto(PROTO_REQ_STATUS, "STATUS?")

    def _open_baro_tab(self) -> None:
        self.notebook.select(self.baro_tab)
        self._send_proto(PROTO_REQ_BARO, "BARO?")

    def _open_imu_tab(self) -> None:
        self.notebook.select(self.imu_tab)
        self._send_proto(PROTO_REQ_IMU, "IMU?")

    def _open_gps_tab(self) -> None:
        self.notebook.select(self.gps_tab)
        self._send_proto(PROTO_REQ_GPS, "GPS?")

    def _refresh_detail(self) -> None:
        state = self.module_state[self.selected_module]
        for name in self.detail_vars:
            self.detail_vars[name].set(state[name].get())

    def _update_module(self, key: str, *, state: str | None = None, stage: str | None = None, value: str | None = None, code: str | None = None, hint: str | None = None, line: str | None = None) -> None:
        module = self.module_state.get(key)
        if module is None:
            return
        updates = {
            "state": state,
            "stage": stage,
            "value": value,
            "code": code,
            "hint": hint,
            "last": line,
        }
        for name, new_value in updates.items():
            if new_value is not None:
                module[name].set(new_value)
        state_text = module["state"].get().casefold()
        if any(token in state_text for token in ("异常", "错误", "失败", "超时", "fault", "error", "fail")):
            row_tag = "fail"
        elif any(token in state_text for token in ("正常", "成功", "已初始化", "已使能", "就绪", " ok", "ready")):
            row_tag = "pass"
        else:
            row_tag = "warn"
        self.module_tree.item(
            key,
            values=(
                module["state"].get(),
                module["stage"].get(),
                module["value"].get(),
                module["code"].get(),
                module["hint"].get(),
            ),
            tags=(row_tag,),
        )
        if key == self.selected_module:
            self._refresh_detail()

    def _handle_board_line(self, line: str) -> None:
        if line.startswith("[上位机] 板子已连接") or line.startswith("[上位机] 串口已连接"):
            self._begin_protocol_probe()
            return
        if line.startswith("[上位机] 板子已断开") or line.startswith("[上位机] 串口已断开"):
            self.structured_protocol_supported = None
            return

        if not (line.startswith("[上位机]") or line.startswith("[host]")):
            self.last_board_rx = time.monotonic()
            mode = self._transport_label()
            self.link_var.set(f"已收到 STM32 数据，{mode} 链路正常")
            if line.startswith(("AIRFRAME ", "IDENT ", "OK ", "ERR ", "PONG", "READY")):
                self.ident_link_var.set(f"{mode} text OK: {line[:96]}")

        if line.startswith("READY"):
            self.last_board_rx = time.monotonic()
            self.link_var.set("STM32 已就绪")
            self._handle_ready_line(line)
        elif line.startswith("HW "):
            self._update_hardware_line(line)
        elif line.startswith("STATUS "):
            self._update_status_line(line)
        elif line.startswith("UART1 "):
            self._update_uart_line(line)
        elif line.startswith("CFG "):
            self._update_config_line(line)
        elif line.startswith("PARAM "):
            self._update_param_line(line)
        elif line.startswith("AIRFRAME "):
            self._update_airframe_line(line)
        elif line.startswith("PID "):
            self._update_pid_line(line)
        elif line.startswith("FLASH "):
            self._update_flash_line(line)
        elif line.startswith("BARO ") or line.startswith("SPL06 "):
            self._update_baro_line(line)
        elif line.startswith("BOOT "):
            self._firmware_handle_boot_line(line)
        elif line.startswith("IMUFRAME "):
            self._validation_handle_imu_frame_line(line)
        elif line.startswith("ACCEPT "):
            self._v2_handle_line(line)
        elif line.startswith("IMU "):
            self._update_imu_line(line)
        elif line.startswith(("GPS ", "GPS_USART2 ", "M9N ")):
            self._update_gps_line(line)
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {line}")
        elif line.startswith(("MAG ", "MAG_I2C1 ")):
            self._update_mag_line(line)
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近回包: {line}")
        elif re.search(r"(^|[ ,])ax=", line) and re.search(r"(^|[ ,])az=", line):
            self._update_imu_line("IMU " + line)
        elif line.startswith("WIFI "):
            self._update_wifi_line(line)
        elif line.startswith("IDENT "):
            self._ident_handle_line(line)
        elif line.startswith("RSP "):
            self._handle_rsp_line(line)
        elif line.startswith("OK servo"):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近命令: {line}")
            self._update_servo_ok_line(line)
        elif line.startswith(("OK save", "OK load", "OK defaults")):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近命令: {line}")
            self._update_config_result_line(line)
        elif line.startswith("ERR imuframe"):
            self.validation_authorized_orientation_commands.clear()
            self.validation_orientation_pending = ""
            self._validation_note_orientation_event("bad", f"飞机拒绝坐标映射命令：{line}")
            self._validation_refresh_orientation_controls()
            self.last_reply_rx = time.monotonic()
        elif line.startswith("OK ") or line.startswith("ERR ") or line.startswith("ACK ") or line.startswith("RX "):
            self.last_reply_rx = time.monotonic()
            self.last_cmd_var.set(f"最近命令: {line}")
        elif line.startswith(("PONG ", "READY ", "FLASH ", "BARO ", "IMU ", "GPS ", "GPS_USART2 ", "M9N ", "MAG ", "MAG_I2C1 ")):
            self.last_reply_rx = time.monotonic()

    def _handle_ready_line(self, line: str) -> None:
        values = parse_kv(line)
        if "cfg_valid" in values:
            self._set_param("system.cfg_valid", values["cfg_valid"], "READY", dirty=False)
        if "servo0_id" in values:
            self._set_param("servo0.id", values["servo0_id"], "READY", dirty=False)
        if "servo1_id" in values:
            self._set_param("servo1.id", values["servo1_id"], "READY", dirty=False)

    def _update_flash_line(self, line: str) -> None:
        values = parse_kv(line)
        if {"ok", "probe", "status", "sr_st", "read"} & values.keys():
            ok = values.get("ok") == "1"
            if "ok" not in values:
                ok = all(safe_int(values.get(name), 1) == 0 for name in ("probe", "status", "sr_st", "read"))
            self._update_module(
                "FLASH",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("stage", "ready" if ok else "-")),
                value=f"ID={values.get('id', '-')} 期望={values.get('exp', '-')} SR1={values.get('sr1', '-')}",
                code=f"probe={values.get('probe', '-')} status={first_value(values, 'status', 'sr', 'sr_st')} read={values.get('read', '-')}",
                hint=self._hardware_hint("FLASH", ok, values),
                line=line,
            )

        if "cfg_addr" in values or "cfg_valid" in values or "cfg_last" in values:
            for key, value in values.items():
                self._set_param(f"flash.{key}", value, "FLASH", dirty=False)
            if "cfg_valid" in values:
                self._set_param("config.valid", values["cfg_valid"], "FLASH", dirty=False)

    def _update_imu_line(self, line: str) -> None:
        values = parse_kv(line)
        if "rate_hz" in values and "level" in values:
            self._validation_accept_imu_health(values)
            return
        self._validation_accept_imu_values(values)
        has_sample = any(key in values for key in ("ax", "ax_mg", "gx", "gx_mdps"))
        if has_sample:
            self._update_imu_attitude(values, line)
        has_status = "ok" in values or "init" in values or "stage" in values
        if has_status:
            ok = values.get("ok") == "1" if "ok" in values else safe_int(values.get("init"), 0) != 0
            self._update_module(
                "ICM42688",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("stage", "-")),
                value=(
                    f"WHO={first_value(values, 'who', 'id')} n={first_value(values, 'n', 'count')} "
                    f"ax={first_value(values, 'ax', 'ax_mg')} ay={first_value(values, 'ay', 'ay_mg')} "
                    f"az={first_value(values, 'az', 'az_mg')}"
                ),
                code=f"st={first_value(values, 'st', 'code')} err={values.get('err', '-')}",
                hint=self._hardware_hint("ICM42688", ok, values),
                line=line,
            )
        elif has_sample:
            self._update_module(
                "ICM42688",
                value=(
                    f"n={first_value(values, 'seq', 'n', 'count')} "
                    f"ax={first_value(values, 'ax', 'ax_mg')} ay={first_value(values, 'ay', 'ay_mg')} "
                    f"az={first_value(values, 'az', 'az_mg')}"
                ),
                line=line,
            )

    def _update_imu_attitude(self, values: dict[str, str], line: str) -> None:
        ax = first_float(values, "ax", "ax_mg")
        ay = first_float(values, "ay", "ay_mg")
        az = first_float(values, "az", "az_mg")
        gx = first_float(values, "gx", "gx_mdps")
        gy = first_float(values, "gy", "gy_mdps")
        gz = first_float(values, "gz", "gz_mdps")
        now = time.monotonic()

        if ax is None or ay is None or az is None:
            return

        if gx is None:
            gx = 0.0
        if gy is None:
            gy = 0.0
        if gz is None:
            gz = 0.0

        roll_cdeg = first_float(values, "roll", "roll_cdeg")
        pitch_cdeg = first_float(values, "pitch", "pitch_cdeg")
        yaw_cdeg = first_float(values, "yaw", "yaw_cdeg")
        if roll_cdeg is not None and pitch_cdeg is not None and yaw_cdeg is not None:
            self.imu_roll_deg = roll_cdeg / 100.0
            self.imu_pitch_deg = pitch_cdeg / 100.0
            self.imu_yaw_deg = yaw_cdeg / 100.0
        else:
            roll_acc = math.degrees(math.atan2(ay, az))
            pitch_acc = math.degrees(math.atan2(-ax, math.sqrt((ay * ay) + (az * az))))
            dt = 0.0
            if self.imu_last_sample_time > 0.0:
                dt = max(0.0, min(now - self.imu_last_sample_time, 0.25))

            if dt <= 0.0:
                self.imu_roll_deg = roll_acc
                self.imu_pitch_deg = pitch_acc
            else:
                alpha = 0.96
                self.imu_roll_deg = alpha * (self.imu_roll_deg + (gx / 1000.0) * dt) + (1.0 - alpha) * roll_acc
                self.imu_pitch_deg = alpha * (self.imu_pitch_deg + (gy / 1000.0) * dt) + (1.0 - alpha) * pitch_acc
                self.imu_yaw_deg += (gz / 1000.0) * dt
                if self.imu_yaw_deg > 180.0 or self.imu_yaw_deg < -180.0:
                    self.imu_yaw_deg = ((self.imu_yaw_deg + 180.0) % 360.0) - 180.0

        self.imu_last_sample_time = now
        self.imu_last_count = safe_int(first_value(values, "n", "count"), self.imu_last_count)

        updates = {
            "roll": f"{self.imu_roll_deg:.1f}",
            "pitch": f"{self.imu_pitch_deg:.1f}",
            "yaw": f"{self.imu_yaw_deg:.1f}",
            "ax": f"{ax:.0f}",
            "ay": f"{ay:.0f}",
            "az": f"{az:.0f}",
            "gx": f"{gx:.0f}",
            "gy": f"{gy:.0f}",
            "gz": f"{gz:.0f}",
            "temp": first_value(values, "t", "temp_cdeg"),
            "count": first_value(values, "n", "count"),
            "who": first_value(values, "who", "id"),
            "age": "0 ms",
        }
        for key, value in updates.items():
            if key in self.imu_vars and value != "-":
                if self.imu_vars[key].get() != value:
                    self.imu_vars[key].set(value)
        self._imu_dirty = True

    def _reset_imu_attitude(self) -> None:
        self.imu_roll_deg = 0.0
        self.imu_pitch_deg = 0.0
        self.imu_yaw_deg = 0.0
        self.imu_last_sample_time = 0.0
        for key, value in {"roll": "0.0", "pitch": "0.0", "yaw": "0.0"}.items():
            if key in self.imu_vars:
                self.imu_vars[key].set(value)
        self._draw_imu_attitude()

    def _draw_imu_attitude(self) -> None:
        canvas = getattr(self, "imu_canvas", None)
        if canvas is None:
            return

        width = max(int(canvas.winfo_width()), 320)
        height = max(int(canvas.winfo_height()), 240)
        canvas.delete("all")

        cx = width / 2.0
        cy = height / 2.0
        radius = min(width, height) * 0.42
        roll = math.radians(self.imu_roll_deg)
        pitch_offset = max(-radius * 0.75, min(radius * 0.75, self.imu_pitch_deg * radius / 45.0))

        def rotate(point: tuple[float, float]) -> tuple[float, float]:
            x, y = point
            return (
                cx + (x * math.cos(roll)) - (y * math.sin(roll)),
                cy + (x * math.sin(roll)) + (y * math.cos(roll)),
            )

        span = radius * 2.8
        sky_poly = [
            rotate((-span, -span + pitch_offset)),
            rotate((span, -span + pitch_offset)),
            rotate((span, pitch_offset)),
            rotate((-span, pitch_offset)),
        ]
        ground_poly = [
            rotate((-span, pitch_offset)),
            rotate((span, pitch_offset)),
            rotate((span, span + pitch_offset)),
            rotate((-span, span + pitch_offset)),
        ]
        canvas.create_polygon(sky_poly, fill="#1D4F73", outline="")
        canvas.create_polygon(ground_poly, fill="#4A3320", outline="")
        canvas.create_line(*rotate((-span, pitch_offset)), *rotate((span, pitch_offset)), fill="#f7f3dc", width=3)

        for deg in range(-60, 75, 15):
            if deg == 0:
                continue
            y = pitch_offset - (deg * radius / 45.0)
            half = radius * (0.38 if deg % 30 == 0 else 0.22)
            x1, y1 = rotate((-half, y))
            x2, y2 = rotate((half, y))
            canvas.create_line(x1, y1, x2, y2, fill="#f7f3dc", width=2)
            if deg % 30 == 0:
                tx, ty = rotate((half + 14, y + 4))
                canvas.create_text(tx, ty, text=str(abs(deg)), fill="#f7f3dc", font=("Segoe UI", 9))

        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#e5e7eb", width=3)
        canvas.create_line(cx - radius * 0.55, cy, cx - radius * 0.15, cy, fill="#ffd166", width=5)
        canvas.create_line(cx + radius * 0.15, cy, cx + radius * 0.55, cy, fill="#ffd166", width=5)
        canvas.create_polygon(cx - 10, cy, cx, cy + 12, cx + 10, cy, fill="#ffd166", outline="")
        canvas.create_text(
            cx,
            height - 26,
            text=f"roll {self.imu_roll_deg:+.1f}   pitch {self.imu_pitch_deg:+.1f}   yaw {self.imu_yaw_deg:+.1f}",
            fill="#e5e7eb",
            font=("Segoe UI", 12, "bold"),
        )

    def _handle_rsp_line(self, line: str) -> None:
        values = parse_kv(line)
        mod = values.get("mod", "").upper()
        payload = {key: value for key, value in values.items() if key not in {"id", "mod", "op"}}

        if mod == "MODULES":
            self._update_modules_summary(payload, line)
            return

        if mod == "CAPS":
            for key, value in payload.items():
                self._set_param(f"caps.{key}", value, "CAPS", dirty=False)
            return

        if mod == "SPL06":
            rename_map = {
                "who": "product_id",
                "sid": "split_id",
                "tid": "txrx_id",
                "press_raw": "raw_pressure",
                "temp_raw": "raw_temperature",
            }
            for old, new in rename_map.items():
                if old in payload and new not in payload:
                    payload[new] = payload.pop(old)
            self._update_baro_line("BARO " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "ICM42688":
            if "ok" in payload and "init" not in payload:
                payload["init"] = payload["ok"]
            if "code" in payload and "err" not in payload:
                payload["err"] = payload["code"]
            self._update_imu_line("IMU " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod in {"GPS", "M9N"}:
            self._update_gps_line("GPS " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "MAG":
            self._update_mag_line("MAG " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "FLASH":
            self._update_flash_line("FLASH " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

        if mod == "WIFI":
            self._update_wifi_line("WIFI " + " ".join(f"{key}={value}" for key, value in payload.items()))
            return

    def _update_modules_summary(self, values: dict[str, str], line: str) -> None:
        if "flash" in values or "flash_stage" in values:
            ok = safe_int(values.get("flash"), 0) != 0
            self._update_module(
                "FLASH",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("flash_stage", "-")),
                hint=self._hardware_hint("FLASH", ok, values),
                line=line,
            )

        if "baro" in values or "baro_stage" in values:
            ok = safe_int(values.get("baro"), 0) != 0
            self._update_module(
                "SPL06",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("baro_stage", "-")),
                hint=self._hardware_hint("SPL06", ok, values),
                line=line,
            )

        if "imu" in values or "imu_stage" in values:
            ok = safe_int(values.get("imu"), 0) != 0
            self._update_module(
                "ICM42688",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("imu_stage", "-")),
                hint=self._hardware_hint("ICM42688", ok, values),
                line=line,
            )

        if "gps" in values or "gps_fix" in values or "gps_sv" in values:
            ok = safe_int(first_value(values, "gps", "gps_ok"), 0) != 0
            self._update_module(
                "GPS",
                state="已初始化" if ok else "等待",
                stage=f"fix={first_value(values, 'gps_fix', 'fix')}",
                value=f"sv={first_value(values, 'gps_sv', 'sv')}",
                code=f"init={first_value(values, 'gps', 'gps_ok')}",
                hint=self._hardware_hint("GPS", ok, values),
                line=line,
            )

        if "mag" in values or "mag_type" in values:
            ok = safe_int(first_value(values, "mag", "mag_ok"), 0) != 0
            self._update_module(
                "MAG",
                state="正常" if ok else "异常",
                stage=first_value(values, "mag_type", "type"),
                value=f"type={first_value(values, 'mag_type', 'type')}",
                code=f"init={first_value(values, 'mag', 'mag_ok')}",
                hint=self._hardware_hint("MAG", ok, values),
                line=line,
            )

        if "cfg_valid" in values:
            self._set_param("config.valid", values["cfg_valid"], "MODULES", dirty=False)
        if "cfg_loaded" in values:
            self._set_param("config.loaded", values["cfg_loaded"], "MODULES", dirty=False)
        if "servo_slots" in values:
            self._set_param("system.servo_slots", values["servo_slots"], "MODULES", dirty=False)
        if "wifi_en" in values:
            ok = safe_int(values.get("wifi_en"), 0) != 0
            self._update_module(
                "WIFI",
                state="已使能" if ok else "已关闭",
                stage="-",
                value=f"en={values.get('wifi_en', '-')}",
                code="-",
                hint="等待 Ai-WB2 透明 TCP 连接。" if ok else "PC6 当前关闭 WiFi EN。",
                line=line,
            )
        if {"cfg_valid", "cfg_loaded"} & values.keys():
            self.config_summary_var.set(
                f"配置 loaded={values.get('cfg_loaded', '-')} valid={values.get('cfg_valid', '-')} flash_st=-"
            )

    def _update_hardware_line(self, line: str) -> None:
        parts = line.split(maxsplit=2)
        if len(parts) < 2:
            return
        key = normalize_module_key(parts[1])
        if key is None:
            return
        values = parse_kv(line)
        ok = values.get("ok") == "1"
        state = "正常" if ok else "异常"
        stage = self._stage_text(values.get("stage", "-"))

        if key == "FLASH":
            value = f"ID={values.get('id', '-')} 期望={values.get('exp', '-')} SR1={values.get('sr1', '-')}"
            code = f"probe={values.get('probe', '-')} sr={values.get('sr', '-')} read={values.get('read', '-')}"
        elif key == "SPL06":
            value = f"ID={values.get('id', '-')} 期望={values.get('exp', '-')} split={values.get('split_id', '-')} txrx={values.get('txrx_id', '-')}"
            code = f"init={values.get('init', '-')} split={values.get('split', '-')} txrx={values.get('txrx', '-')}"
            self._update_baro_from_values(values, line)
        elif key == "ICM42688":
            value = f"WHO={values.get('who', '-')} 期望={values.get('exp', '-')} n={values.get('n', '-')}"
            code = f"st={values.get('st', '-')}"
        elif key == "GPS":
            value = f"fix={first_value(values, 'fix', 'fix_type')} sv={first_value(values, 'sv', 'num_sv')} lat={values.get('lat', '-')} lon={values.get('lon', '-')}"
            code = f"init={values.get('init', '-')} nav={values.get('nav', '-')}"
            self._update_gps_line("GPS " + " ".join(f"{name}={value}" for name, value in values.items()))
        elif key == "MAG":
            value = f"type={values.get('type', '-')} addr={values.get('addr', '-')} n={values.get('n', '-')}"
            code = f"init={values.get('init', '-')} st={values.get('st', '-')}"
            self._update_mag_line("MAG " + " ".join(f"{name}={value}" for name, value in values.items()))
        else:
            value = "-"
            code = "-"

        self._update_module(key, state=state, stage=stage, value=value, code=code, hint=self._hardware_hint(key, ok, values), line=line)

    def _update_status_line(self, line: str) -> None:
        parts = line.split(maxsplit=2)
        if len(parts) < 2:
            return
        subject = parts[1].lower()
        values = parse_kv(line)
        if subject == "flash":
            ok = all(safe_int(values.get(name), 1) == 0 for name in ("probe", "sr_st", "read"))
            self._update_module(
                "FLASH",
                state="正常" if ok else "异常",
                stage="初始化完成" if ok else "状态读取",
                value=f"ID={values.get('id', '-')} SR1={values.get('sr1', '-')}",
                code=f"probe={values.get('probe', '-')} sr_st={values.get('sr_st', '-')} read={values.get('read', '-')}",
                hint=self._hardware_hint("FLASH", ok, values),
                line=line,
            )
        elif subject == "baro":
            ok = safe_int(values.get("init"), 1) == 0
            self._update_module(
                "SPL06",
                state="正常" if ok else "异常",
                stage="初始化完成" if ok else "初始化/识别",
                value=f"ID={values.get('id', '-')} split={values.get('split_id', '-')} txrx={values.get('txrx_id', '-')} bmp={values.get('bmp', '-')}",
                code=f"init={values.get('init', '-')} split={values.get('split', '-')} txrx={values.get('txrx', '-')}",
                hint=self._hardware_hint("SPL06", ok, values),
                line=line,
            )
            self._update_baro_from_values(values, line)
        elif subject == "imu":
            ok = safe_int(values.get("init"), 0) != 0
            self._update_module(
                "ICM42688",
                state="正常" if ok else "异常",
                stage="初始化完成" if ok else "初始化/识别",
                value=f"WHO={values.get('who', '-')} n={values.get('n', '-')} ax={values.get('ax', '-')} ay={values.get('ay', '-')} az={values.get('az', '-')}",
                code=f"st={values.get('st', '-')}",
                hint=self._hardware_hint("ICM42688", ok, values),
                line=line,
            )
        elif subject == "gps":
            self._update_gps_line("GPS " + " ".join(f"{key}={value}" for key, value in values.items()))
        elif subject == "mag":
            self._update_mag_line("MAG " + " ".join(f"{key}={value}" for key, value in values.items()))

    def _update_wifi_line(self, line: str) -> None:
        values = parse_kv(line)
        enabled = safe_int(values.get("en"), 0) != 0
        transparent = safe_int(values.get("transparent"), 0) != 0
        cycling = safe_int(values.get("cycling"), 0) != 0
        state_text = values.get("state", "-")

        if transparent:
            state = "已连接"
            hint = "Ai-WB2 已进入 TCP 透明模式，可以通过 TCP 发送控制命令。"
        elif cycling:
            state = "重启中"
            hint = "PC6 正在重启 Ai-WB2，等待模块重新连接上位机 TCP 服务。"
        elif enabled:
            state = "等待连接"
            hint = "先保持上位机 TCP 监听；若连接失败，固件会通过 PC6 自动重启模块重试。"
        else:
            state = "已关闭"
            hint = "WiFi EN 为低；发送 WIFI EN 1 或 WIFI RESET 可重新拉起。"

        self._update_module(
            "WIFI",
            state=state,
            stage=state_text,
            value=f"en={values.get('en', '-')} trans={values.get('transparent', '-')} wait={values.get('wait_ms', '-')}",
            code=f"retry={values.get('retry', '-')} socket={values.get('socket', '-')} writes={values.get('writes', '-')}",
            hint=hint,
            line=line,
        )

    def _update_uart_line(self, line: str) -> None:
        values = parse_kv(line)
        rx_bytes = safe_int(values.get("rx_bytes"))
        rx_lines = safe_int(values.get("rx_lines"))
        rx_errors = safe_int(values.get("rx_errors"))
        rx_overflows = safe_int(values.get("rx_overflows"))
        ok = rx_errors == 0 and rx_overflows == 0

        if rx_bytes == 0:
            hint = "STM32 正在发心跳，但还没收到上位机命令；检查 Ai-WB2 到 USART1 的 TX/RX。"
        elif rx_lines == 0:
            hint = "已经收到字节但没有收到换行；检查上位机发送是否带 CR/LF。"
        elif ok:
            hint = "USART1 收发链路已打通。"
        else:
            hint = "串口出现溢出或错误；先降低发送频率，再检查波特率和接线。"

        self._update_module(
            "UART1",
            state="正常" if ok else "有错误",
            stage="接收统计",
            value=f"bytes={rx_bytes} lines={rx_lines}",
            code=f"overflow={rx_overflows} err={rx_errors}",
            hint=hint,
            line=line,
        )

    def _update_baro_line(self, line: str) -> None:
        values = parse_kv(line)
        ok_text = values.get("ok")
        if ok_text is not None:
            ok = ok_text == "1"
            self._update_module(
                "SPL06",
                state="正常" if ok else "异常",
                stage=self._stage_text(values.get("stage", "-")),
                value=f"P={first_value(values, 'pressure', 'pressure_pa', 'p', 'pa')} T={first_value(values, 'temperature', 'temp', 't')} Alt={first_value(values, 'altitude', 'alt')}",
                code=f"init={values.get('init', '-')} st={values.get('st', '-')}",
                hint=self._hardware_hint("SPL06", ok, values),
                line=line,
            )
        self._update_baro_from_values(values, line)

    def _update_baro_from_values(self, values: dict[str, str], line: str) -> None:
        is_raw_line = line.startswith("BARO raw")
        mapping = {
            "product_id": first_value(values, "product_id", "id", "who"),
            "split_id": first_value(values, "split_id", "sid"),
            "txrx_id": first_value(values, "txrx_id", "tid"),
            "bmp_id": first_value(values, "bmp", "bmp_id"),
            "init_status": first_value(values, "init"),
            "split_status": first_value(values, "split"),
            "txrx_status": first_value(values, "txrx"),
            "cs_level": first_value(values, "cs"),
            "miso_level": first_value(values, "miso"),
            "stage": self._stage_text(values.get("stage", "-")),
            "raw_pressure": first_value(values, "raw_pressure", "raw_p", "prs_raw", "press_raw"),
            "raw_temperature": first_value(values, "raw_temperature", "raw_t", "tmp_raw", "temp_raw"),
            "sample_count": first_value(values, "n", "count", "samples"),
            "last_update": time.strftime("%H:%M:%S"),
        }
        pressure = None if is_raw_line else first_float(values, "pressure", "pressure_pa", "p", "pa")
        temperature = None if is_raw_line else first_float(values, "temperature", "temp", "temp_c", "t")
        altitude = first_float(values, "altitude", "alt", "alt_m")
        if temperature is None and "temp_cdeg" in values:
            temperature = safe_int(values.get("temp_cdeg")) / 100.0
        if is_raw_line:
            if mapping["raw_pressure"] == "-":
                mapping["raw_pressure"] = first_value(values, "pressure")
            if mapping["raw_temperature"] == "-":
                mapping["raw_temperature"] = first_value(values, "temp")

        if pressure is not None:
            mapping["pressure"] = f"{pressure:g}"
        if temperature is not None:
            mapping["temperature"] = f"{temperature:g}"
        if altitude is not None:
            mapping["altitude"] = f"{altitude:g}"
        if "init" in values:
            mapping["state"] = "正常" if safe_int(values.get("init"), 1) == 0 else "异常"

        for key, value in mapping.items():
            if value != "-" and key in self.baro_vars:
                self.baro_vars[key].set(value)

        if pressure is None and temperature is None and altitude is None:
            return
        if not self.baro_capture_enabled.get():
            return

        sample: dict[str, float | str] = {"time": time.time(), "line": line}
        if pressure is not None:
            sample["pressure"] = pressure
        if temperature is not None:
            sample["temperature"] = temperature
        if altitude is not None:
            sample["altitude"] = altitude
        self.baro_buffer.append(sample)
        if len(self.baro_buffer) > MAX_BARO_SAMPLES:
            del self.baro_buffer[: len(self.baro_buffer) - MAX_BARO_SAMPLES]
        self._baro_dirty = True

    def _baro_tick(self) -> None:
        if self._baro_dirty:
            self._baro_dirty = False
            now_ns = time.monotonic_ns()
            self._refresh_baro_samples()
            if now_ns - self._last_baro_plot_ns > 300_000_000:
                self._last_baro_plot_ns = now_ns
                self._update_baro_plot()
        self.after(250, self._baro_tick)

    def _imu_poll_tick(self) -> None:
        now = time.monotonic()
        if self.imu_last_sample_time > 0.0 and "age" in self.imu_vars:
            age_ms = int((now - self.imu_last_sample_time) * 1000.0)
            age_text = f"{age_ms} ms"
            if self.imu_vars["age"].get() != age_text:
                self.imu_vars["age"].set(age_text)
        now_ns = time.monotonic_ns()
        imu_tab_visible = self.notebook.select() == str(self.imu_tab)
        firmware_tab_visible = self.notebook.select() == str(self.firmware_tab)
        v1_tab_visible = self.notebook.select() == str(self.v1_tab)
        validation_tab_visible = self.notebook.select() == str(self.validation_tab)
        if (
            self._imu_dirty
            and imu_tab_visible
            and (now_ns - self._last_imu_draw_ns) >= (IMU_RENDER_PERIOD_MS * 1_000_000)
        ):
            self._imu_dirty = False
            self._last_imu_draw_ns = now_ns
            self._draw_imu_attitude()
        if (
            self.validation_active_stage is not None
            and self.validation_latest_host_time > 0.0
            and (now - self.validation_latest_host_time) > VALIDATION_SAMPLE_FRESH_S
        ):
            self._validation_abort_active_stage(
                ValidationStatus.UNSUPPORTED,
                "目标快照超时，当前步骤证据作废",
            )
        if (now_ns - self._last_validation_readiness_ns) >= 500_000_000:
            self._last_validation_readiness_ns = now_ns
            self._validation_refresh_readiness()
            self._firmware_refresh_safety()
        v1_busy = self.v1_worker is not None and self.v1_worker.is_alive()
        firmware_busy = self.firmware_update_pending or self.firmware_update_running
        poll_requested = self.validation_poll_requested or (
            self.imu_poll_enabled.get() and imu_tab_visible
        ) or (
            firmware_tab_visible and not firmware_busy
        ) or (
            v1_tab_visible and not v1_busy
        ) or (
            # V0 页的四项准备清单（连接/快照/零偏/安全输出）和 A/B/C 按钮全部
            # 由 _validation_refresh_readiness 依据实时快照解锁。页面一打开就必须
            # 开始取快照，否则引导让用户"等待准备状态变 ✓"会永远等不到。
            validation_tab_visible and not firmware_busy and not v1_busy
        )
        if poll_requested and self._transport_connected():
            if now - self.imu_last_poll >= IMU_POLL_PERIOD_MS / 1000.0:
                self.imu_last_poll = now
                self._send_proto_silent(PROTO_REQ_IMU, "IMU?")
        self.after(50, self._imu_poll_tick)

    def _refresh_baro_samples(self) -> None:
        self.baro_count_var.set(f"暂存样本: {len(self.baro_buffer)}")
        latest = self.baro_buffer[-80:]
        children = self.baro_sample_tree.get_children()
        n_existing = len(children)
        n_needed = len(latest)
        base = float(latest[0]["time"]) if latest else 0.0
        for i, sample in enumerate(latest):
            t = float(sample["time"]) - base
            vals = (
                f"{t:.2f}",
                self._fmt_sample(sample, "pressure"),
                self._fmt_sample(sample, "temperature"),
                self._fmt_sample(sample, "altitude"),
            )
            if i < n_existing:
                self.baro_sample_tree.item(children[i], values=vals)
            else:
                self.baro_sample_tree.insert("", tk.END, values=vals)
        for i in range(n_needed, n_existing):
            self.baro_sample_tree.delete(children[i])

    def _fmt_sample(self, sample: dict[str, float | str], key: str) -> str:
        value = sample.get(key)
        if isinstance(value, float):
            return f"{value:g}"
        return "-"

    def _update_baro_plot(self) -> None:
        if not HAS_MATPLOTLIB or self.baro_axis is None or self.baro_canvas is None:
            return
        key = self.plot_var.get()
        points = [(float(s["time"]), float(s[key])) for s in self.baro_buffer if key in s]
        self.baro_axis.clear()
        self.baro_axis.set_title(f"SPL06 {key}")
        self.baro_axis.set_xlabel("time (s)")
        self.baro_axis.grid(True, alpha=0.3)
        if points:
            base = points[0][0]
            self.baro_axis.plot([t - base for t, _v in points], [v for _t, v in points], linewidth=1.4)
        else:
            self.baro_axis.text(0.5, 0.5, "waiting for samples", ha="center", va="center", transform=self.baro_axis.transAxes)
        self.baro_figure.tight_layout()
        self.baro_canvas.draw_idle()

    def _clear_baro_buffer(self) -> None:
        self.baro_buffer.clear()
        self._baro_dirty = False
        self._refresh_baro_samples()
        self._last_baro_plot_ns = time.monotonic_ns()
        self._update_baro_plot()

    def _export_baro_csv(self) -> None:
        if not self.baro_buffer:
            messagebox.showinfo("没有数据", "气压计暂存区为空")
            return
        initial = dated_directory(TELEMETRY_DIR) / f"baro_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        initial.parent.mkdir(parents=True, exist_ok=True)
        filename = filedialog.asksaveasfilename(
            title="导出气压计暂存数据",
            defaultextension=".csv",
            initialdir=str(initial.parent),
            initialfile=initial.name,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        with open(filename, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["time", "pressure", "temperature", "altitude", "line"])
            writer.writeheader()
            for sample in self.baro_buffer:
                writer.writerow(sample)
        self._append(f"[上位机] 已导出气压计暂存数据: {filename}")

    def _gps_lat_lon_from_values(self, values: dict[str, str]) -> tuple[float | None, float | None]:
        lat = first_float(values, "lat_deg", "latitude_deg", "latitude")
        lon = first_float(values, "lon_deg", "longitude_deg", "longitude")
        if lat is None:
            lat = first_float(values, "lat", "lat_deg_e7")
        if lon is None:
            lon = first_float(values, "lon", "lon_deg_e7")
        if lat is not None and abs(lat) > 90.0:
            lat /= 10000000.0
        if lon is not None and abs(lon) > 180.0:
            lon /= 10000000.0
        return lat, lon

    def _gps_xy_from_lat_lon(self, lat: float, lon: float) -> tuple[float, float]:
        if self.gps_origin_lat is None or self.gps_origin_lon is None:
            self.gps_origin_lat = lat
            self.gps_origin_lon = lon
        radius_m = 6378137.0
        lat0 = self.gps_origin_lat
        lon0 = self.gps_origin_lon
        x = math.radians(lon - lon0) * radius_m * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * radius_m
        return x, y

    def _update_gps_line(self, line: str) -> None:
        values = parse_kv(line)
        lat, lon = self._gps_lat_lon_from_values(values)
        fix_text = first_value(values, "fix", "fix_type")
        previous_fix = self.gps_vars.get("fix").get() if "fix" in self.gps_vars else "0"
        fix = safe_int(fix_text, safe_int(previous_fix, 0))
        valid = safe_int(first_value(values, "valid", "valid_fix"), 0) != 0
        if "ok" in values:
            initialized = safe_int(values.get("ok"), 0) != 0
        elif line.startswith("STATUS gps"):
            initialized = safe_int(values.get("init"), 0) != 0
        elif "init" in values:
            initialized = safe_int(values.get("init"), 1) == 0
        else:
            initialized = False
        sv_text = first_value(values, "sv", "num_sv")
        previous_sv = self.gps_vars.get("sv").get() if "sv" in self.gps_vars else "0"
        sv = safe_int(sv_text, safe_int(previous_sv, 0))
        now = time.time()
        x: float | None = None
        y: float | None = None

        if lat is not None and lon is not None and abs(lat) <= 90.0 and abs(lon) <= 180.0:
            if abs(lat) > 0.000001 or abs(lon) > 0.000001:
                x, y = self._gps_xy_from_lat_lon(lat, lon)
                self.gps_track.append(
                    {
                        "time": now,
                        "lat": lat,
                        "lon": lon,
                        "x_m": x,
                        "y_m": y,
                        "fix": fix,
                        "sv": sv,
                        "hmsl_mm": safe_int(first_value(values, "hmsl_mm", "hmsl"), 0),
                        "hacc_mm": safe_int(first_value(values, "hacc_mm", "hacc"), 0),
                        "line": line,
                    }
                )
                if len(self.gps_track) > MAX_GPS_TRACK_POINTS:
                    del self.gps_track[: len(self.gps_track) - MAX_GPS_TRACK_POINTS]

        previous_state = self.gps_vars.get("state").get() if "state" in self.gps_vars else "-"
        if valid or fix >= 2:
            state = "已定位"
        elif initialized:
            state = "已初始化"
        elif previous_state not in {"-", "等待 GPS"} and not ({"ok", "init", "valid", "valid_fix", "fix", "fix_type"} & values.keys()):
            state = previous_state
        else:
            state = "等待 GPS"
        age = first_value(values, "age_ms", "age")
        if age != "-" and age.isdigit():
            age = f"{age} ms"
        speed = first_value(values, "gspd", "ground_speed_mm_s", "speed_mm_s")
        if speed == "-":
            vn = first_float(values, "vn", "vel_n_mm_s")
            ve = first_float(values, "ve", "vel_e_mm_s")
            if vn is not None and ve is not None:
                speed = f"{math.hypot(vn, ve):.0f}"

        updates = {
            "state": state,
            "fix": str(fix) if fix_text != "-" else "-",
            "sv": str(sv) if sv_text != "-" else "-",
            "lon": f"{lon:.7f}" if lon is not None else "-",
            "lat": f"{lat:.7f}" if lat is not None else "-",
            "hmsl": first_value(values, "hmsl_mm", "hmsl"),
            "x": f"{x:.2f}" if x is not None else "-",
            "y": f"{y:.2f}" if y is not None else "-",
            "spd": speed,
            "hdg": "-",
            "age": age,
        }
        heading = first_float(values, "head_e5", "heading_motion_deg_e5", "heading_deg_e5")
        if heading is not None:
            updates["hdg"] = f"{heading / 100000.0:.2f}"
        for key, value in updates.items():
            if key in self.gps_vars and value != "-":
                self.gps_vars[key].set(value)

        value_text = f"fix={fix} sv={sv}"
        if lat is not None and lon is not None:
            value_text += f" lat={lat:.7f} lon={lon:.7f}"
        code_parts = [
            f"nav={values.get('nav', '-')}",
            f"pkts={first_value(values, 'pkts', 'packets')}",
        ]
        if "nmea" in values or "gga" in values:
            code_parts.append(f"nmea={values.get('nmea', '-')}")
            code_parts.append(f"gga={values.get('gga', '-')}")

        self._update_module(
            "GPS",
            state=state,
            stage=f"fix={fix}",
            value=value_text,
            code=" ".join(code_parts),
            hint=self._hardware_hint("GPS", valid or initialized, values),
            line=line,
        )
        self.gps_count_var.set(f"轨迹点: {len(self.gps_track)}")
        self.gps_status_var.set(f"{state}  fix={fix}  sv={sv}")

        now_ns = time.monotonic_ns()
        if now_ns - self.gps_last_plot_ns > 200_000_000:
            self.gps_last_plot_ns = now_ns
            self._update_gps_plot()

    def _update_mag_line(self, line: str) -> None:
        values = parse_kv(line)
        has_state_fields = bool({"ok", "init", "st", "raw", "mgauss", "x", "y", "z"} & values.keys())
        if "ok" in values:
            ok = safe_int(values.get("ok"), 0) != 0
        elif line.startswith("STATUS mag"):
            ok = safe_int(values.get("init"), 0) != 0
        elif "init" not in values:
            ok = self.mag_vars.get("state").get() == "正常" if "state" in self.mag_vars else False
        else:
            ok = safe_int(values.get("init"), 1) == 0
        raw = first_value(values, "raw")
        scaled = first_value(values, "mgauss")
        if raw == "-":
            raw = f"{first_value(values, 'raw_x', 'x_raw')},{first_value(values, 'raw_y', 'y_raw')},{first_value(values, 'raw_z', 'z_raw')}"
        if scaled == "-":
            scaled = f"{first_value(values, 'x_mgauss', 'x')},{first_value(values, 'y_mgauss', 'y')},{first_value(values, 'z_mgauss', 'z')}"
        if raw == "-,-,-":
            raw = "-"
        if scaled == "-,-,-":
            scaled = "-"

        updates = {
            "state": ("正常" if ok else "异常") if has_state_fields else "-",
            "type": first_value(values, "type"),
            "raw": raw,
            "scaled": scaled,
        }
        for key, value in updates.items():
            if key in self.mag_vars and value != "-":
                self.mag_vars[key].set(value)

        module_state = ("正常" if ok else "异常") if has_state_fields else None
        if {"ist", "hmc", "qmc"} & values.keys():
            value_text = f"probe ist={values.get('ist', '-')} hmc={values.get('hmc', '-')} qmc={values.get('qmc', '-')}"
            code_text = f"hmc_id={values.get('hmc_id', '-')}"
        else:
            value_text = f"type={first_value(values, 'type')} addr={values.get('addr', '-')} n={values.get('n', '-')}"
            code_text = f"init={values.get('init', '-')} st={values.get('st', '-')}"
        self._update_module(
            "MAG",
            state=module_state,
            stage=first_value(values, "type"),
            value=value_text,
            code=code_text,
            hint=self._hardware_hint("MAG", ok, values) if has_state_fields else None,
            line=line,
        )

    def _update_gps_plot(self) -> None:
        if not HAS_MATPLOTLIB or self.gps_axis is None or self.gps_canvas is None:
            return
        self.gps_axis.clear()
        self.gps_axis.set_title("M9N XY track")
        self.gps_axis.set_xlabel("X east (m)")
        self.gps_axis.set_ylabel("Y north (m)")
        self.gps_axis.grid(True, alpha=0.3)
        self.gps_axis.set_aspect("equal", adjustable="datalim")

        if self.gps_track:
            xs = [float(point["x_m"]) for point in self.gps_track]
            ys = [float(point["y_m"]) for point in self.gps_track]
            self.gps_axis.plot(xs, ys, linewidth=1.2, marker=".", markersize=3)
            self.gps_axis.scatter([xs[-1]], [ys[-1]], s=45, color="#d62728", zorder=3)
            pad = max(max(xs) - min(xs), max(ys) - min(ys), 2.0) * 0.08
            self.gps_axis.set_xlim(min(xs) - pad, max(xs) + pad)
            self.gps_axis.set_ylim(min(ys) - pad, max(ys) + pad)
        else:
            self.gps_axis.text(0.5, 0.5, "waiting for GPS points", ha="center", va="center", transform=self.gps_axis.transAxes)

        self.gps_figure.tight_layout()
        self.gps_canvas.draw_idle()

    def _clear_gps_track(self) -> None:
        self.gps_track.clear()
        self.gps_origin_lat = None
        self.gps_origin_lon = None
        self.gps_count_var.set("轨迹点: 0")
        self.gps_status_var.set("等待 GPS")
        self.gps_last_plot_ns = time.monotonic_ns()
        self._update_gps_plot()

    def _export_gps_csv(self) -> None:
        if not self.gps_track:
            messagebox.showinfo("没有数据", "GPS 轨迹为空")
            return
        initial = dated_directory(TELEMETRY_DIR) / f"gps_track_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        initial.parent.mkdir(parents=True, exist_ok=True)
        filename = filedialog.asksaveasfilename(
            title="导出 GPS 轨迹",
            defaultextension=".csv",
            initialdir=str(initial.parent),
            initialfile=initial.name,
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        with open(filename, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["time", "lat", "lon", "x_m", "y_m", "fix", "sv", "hmsl_mm", "hacc_mm", "line"])
            writer.writeheader()
            for point in self.gps_track:
                writer.writerow(point)
        self._append(f"[上位机] 已导出 GPS 轨迹: {filename}")

    def _ident_csv_fields(self) -> list[str]:
        return [
            "host_time", "id", "seq", "t_ms", "axis", "mode",
            "alpha_us", "beta_us", "roll", "pitch", "gx", "gy",
            "rc_arm", "throttle_us", "line",
        ]

    def _ident_meta_payload(self) -> dict[str, object]:
        return {
            "host_time": time.time(),
            "command": self.ident_current_command,
            "axis": self.ident_axis_var.get(),
            "mode": self.ident_mode_var.get(),
            "pulse_us": int(self.ident_pulse_var.get()),
            "duration_ms": int(self.ident_duration_var.get()),
            "hold_ms": int(self.ident_hold_var.get()),
            "repeat": int(self.ident_repeat_var.get()),
            "bit_ms": int(self.ident_bit_var.get()),
            "seed": int(self.ident_seed_var.get()),
            "center": {
                "alpha_us": int(self.ident_alpha_center_var.get()),
                "beta_us": int(self.ident_beta_center_var.get()),
            },
            "airframe": self.airframe_info,
        }

    def _ident_write_meta(self) -> None:
        if self.ident_current_path is None:
            return
        meta_path = self.ident_current_path.with_name(f"{self.ident_current_path.stem}_meta.json")
        self.ident_current_meta_path = meta_path
        meta_path.write_text(
            json.dumps(self._ident_meta_payload(), ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        if hasattr(self, "_ident_update_file_label"):
            self._ident_update_file_label()

    def _ident_update_file_label(self) -> None:
        csv_text = str(self.ident_current_path) if self.ident_current_path is not None else "none"
        meta_text = str(self.ident_current_meta_path) if self.ident_current_meta_path is not None else "none"
        self.ident_last_file_var.set(f"last csv: {csv_text}\nlast meta: {meta_text}")

    def _ident_begin_recording(self) -> None:
        self._ident_close_csv()
        path = self.ident_save_dir
        path.mkdir(parents=True, exist_ok=True)
        self.ident_current_path = path / f"ident_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self.ident_current_meta_path = self.ident_current_path.with_name(f"{self.ident_current_path.stem}_meta.json")
        self.ident_csv_file = self.ident_current_path.open("w", newline="", encoding="utf-8")
        self.ident_csv_writer = csv.DictWriter(self.ident_csv_file, fieldnames=self._ident_csv_fields(), extrasaction="ignore")
        self.ident_csv_writer.writeheader()
        self.ident_samples.clear()
        self.ident_last_fit = None
        self.ident_sample_count_var.set("samples=0")
        self.ident_fit_var.set("no fit")
        self._last_ident_flush_ns = time.monotonic_ns()
        self._last_ident_plot_ns = 0
        self.ident_output_dir_var.set(f"save dir: {path}")
        self._ident_update_file_label()
        self._ident_write_meta()

    def _ident_close_csv(self) -> None:
        if self.ident_csv_file is not None:
            self.ident_csv_file.close()
        self.ident_csv_file = None
        self.ident_csv_writer = None

    def _ident_payload(self) -> str:
        axis = self.ident_axis_var.get()
        mode = self.ident_mode_var.get().upper()
        pulse = int(self.ident_pulse_var.get())
        if mode == "STEP":
            return f"IDENT STEP {axis} pulse_us={pulse} duration_ms={int(self.ident_duration_var.get())}"
        if mode == "DOUBLET":
            return f"IDENT DOUBLET {axis} pulse_us={pulse} hold_ms={int(self.ident_hold_var.get())} repeat={int(self.ident_repeat_var.get())}"
        return f"IDENT PRBS {axis} pulse_us={pulse} bit_ms={int(self.ident_bit_var.get())} duration_ms={int(self.ident_duration_var.get())} seed={int(self.ident_seed_var.get())}"

    def _on_ident_config_change(self, *_args) -> None:
        try:
            payload = self._ident_payload()
        except Exception:
            payload = "参数未完整"
        self.ident_command_preview_var.set(f"将发送: {payload}")
        self._ident_update_mode_fields()

    def _ident_update_mode_fields(self) -> None:
        if not self.ident_field_rows:
            return
        mode = self.ident_mode_var.get().upper()
        visible = {
            "STEP": {"pulse", "duration", "alpha_center", "beta_center"},
            "DOUBLET": {"pulse", "hold", "repeat", "alpha_center", "beta_center"},
            "PRBS": {"pulse", "duration", "bit", "seed", "alpha_center", "beta_center"},
        }.get(mode, set(self.ident_field_rows.keys()))
        hints = {
            "STEP": "STEP 用来看最基础的阶跃响应，适合第一次确认方向和估计 K/tau/L。",
            "DOUBLET": "DOUBLET 正负各打一段，能减少持续偏置，适合绑绳台架。",
            "PRBS": "PRBS 信息量大，建议等 STEP/DOUBLET 都正常后再用。",
        }
        self.ident_mode_hint_var.set(hints.get(mode, ""))
        for key, widgets in self.ident_field_rows.items():
            for widget in widgets:
                if key in visible:
                    widget.grid()
                else:
                    widget.grid_remove()

    def _ident_apply_preset(self, name: str) -> None:
        if name == "small":
            self.ident_mode_var.set("STEP")
            self.ident_pulse_var.set(20)
            self.ident_duration_var.set(3000)
        elif name == "step":
            self.ident_mode_var.set("STEP")
            self.ident_pulse_var.set(30)
            self.ident_duration_var.set(4000)
        elif name == "doublet":
            self.ident_mode_var.set("DOUBLET")
            self.ident_pulse_var.set(30)
            self.ident_hold_var.set(700)
            self.ident_repeat_var.set(2)
        self._on_ident_config_change()

    def _ident_run(self) -> None:
        payload = self._ident_payload()
        self.ident_current_command = payload
        self._ident_begin_recording()
        self._send_proto(PROTO_REQ_IDENT, payload)

    def _ident_send_center(self) -> None:
        payload = f"IDENT CENTER alpha_us={int(self.ident_alpha_center_var.get())} beta_us={int(self.ident_beta_center_var.get())}"
        self._send_proto(PROTO_REQ_IDENT, payload)

    def _update_airframe_line(self, line: str) -> None:
        record = airframe_record_from_line(line)
        if record is None:
            return
        self.airframe_info = record
        self.ident_link_var.set("AIRFRAME received")
        self.ident_airframe_var.set(
            f"AIRFRAME m={record.get('mass_kg', '-')}kg cg_z={record.get('cg_z_m', '-')}m "
            f"attach_cg={record.get('tether_attach_to_cg_m', '-')}m rope={record.get('rope_m', '-')}m "
            f"maxF={record.get('max_total_force_n', '-')}N hover={record.get('hover_thrust_pct', '-')}%"
        )

    def _ident_handle_line(self, line: str) -> None:
        values = parse_kv(line)
        if line.startswith("IDENT start "):
            self.ident_status_var.set("running")
            self.ident_reason_var.set("running")
            self.ident_link_var.set(f"IDENT running: {line}")
            self._ident_write_meta()
            return
        if line.startswith("IDENT done "):
            self.ident_status_var.set("done")
            self.ident_reason_var.set(values.get("reason", "complete"))
            self.ident_link_var.set(f"IDENT done: {line}")
            self._ident_close_csv()
            return
        if line.startswith("IDENT abort "):
            self.ident_status_var.set("aborted")
            self.ident_reason_var.set(values.get("reason", "abort"))
            self.ident_link_var.set(f"IDENT aborted: {line}")
            self._ident_close_csv()
            return
        if line.startswith("IDENT state="):
            self.ident_status_var.set(values.get("state", "-"))
            self.ident_reason_var.set(values.get("reason", "-"))
            self.ident_link_var.set(f"IDENT status: {line}")
            return
        record = ident_record_from_line(line)
        if record is None:
            return
        record["host_time"] = f"{time.time():.6f}"
        self.ident_samples.append(record)
        if len(self.ident_samples) > MAX_IDENT_SAMPLES:
            del self.ident_samples[: len(self.ident_samples) - MAX_IDENT_SAMPLES]
        if self.ident_csv_writer is None:
            self._ident_begin_recording()
        if self.ident_csv_writer is not None:
            self.ident_csv_writer.writerow(record)
            now_ns = time.monotonic_ns()
            if (
                self.ident_csv_file is not None
                and (now_ns - self._last_ident_flush_ns) >= 1_000_000_000
            ):
                self.ident_csv_file.flush()
                self._last_ident_flush_ns = now_ns
                self._ident_update_file_label()
        self.ident_sample_count_var.set(f"samples={len(self.ident_samples)}")
        self.ident_current_var.set(
            f"seq={record.get('seq', '-')} alpha={record.get('alpha_us', '-')} beta={record.get('beta_us', '-')} "
            f"roll={record.get('roll', '-')} pitch={record.get('pitch', '-')} gx={record.get('gx', '-')} gy={record.get('gy', '-')}"
        )
        now_ns = time.monotonic_ns()
        if (
            self.notebook.select() == str(self.ident_tab)
            and (now_ns - self._last_ident_plot_ns) >= 200_000_000
        ):
            self._last_ident_plot_ns = now_ns
            self._ident_update_plot()

    def _ident_update_plot(self) -> None:
        if not HAS_MATPLOTLIB or getattr(self, "ident_axis_plot", None) is None or getattr(self, "ident_canvas", None) is None:
            return
        if not self.ident_samples:
            return
        axis_name = self.ident_axis_var.get()
        value_key = "roll" if axis_name == "roll" else "pitch"
        input_key = "alpha_us" if axis_name == "roll" else "beta_us"
        rows = [row for row in self.ident_samples if isinstance(row.get("t_ms"), int)]
        if not rows:
            return
        t0 = float(rows[0]["t_ms"]) / 1000.0
        xs = [(float(row["t_ms"]) / 1000.0) - t0 for row in rows]
        ys = [float(row.get(value_key, 0.0)) for row in rows]
        us = [float(row.get(input_key, 0.0)) for row in rows]
        axis = self.ident_axis_plot
        axis.clear()
        axis.plot(xs, ys, label=value_key)
        if us:
            base = us[0]
            axis.plot(xs, [(u - base) / 10.0 for u in us], label=f"{input_key} delta/10")
        axis.set_xlabel("s")
        axis.legend(loc="best")
        axis.grid(True, alpha=0.3)
        self.ident_canvas.draw_idle()

    def _ident_fit(self) -> None:
        fit = fit_ident_step(self.ident_samples, self.ident_axis_var.get())
        self.ident_last_fit = fit
        if fit is None:
            self.ident_fit_var.set("fit failed: need a clean step/doublet response")
            return
        self.ident_fit_var.set(
            f"K={fit['K']:.5f} tau={fit['tau']:.3f}s L={fit['L']:.3f}s "
            f"kp={fit['kp']:.5f} kd={fit['kd']:.5f}"
        )

    def _ident_apply_fit(self) -> None:
        if self.ident_last_fit is None:
            self._ident_fit()
        if self.ident_last_fit is None:
            return
        axis = self.ident_axis_var.get()
        fit = self.ident_last_fit
        payload = f"IDENT APPLY {axis} kp={fit['kp']:.6f} kd={fit['kd']:.6f}"
        self._send_proto(PROTO_REQ_IDENT, payload)

    def _ident_open_folder(self) -> None:
        path = self.ident_save_dir
        path.mkdir(parents=True, exist_ok=True)
        try:
            os.startfile(path)  # type: ignore[attr-defined]
        except Exception:
            messagebox.showinfo("IDENT", f"CSV folder:\n{path}")

    def _update_config_line(self, line: str) -> None:
        values = parse_kv(line)
        if "loaded" in values or "valid" in values:
            self.config_summary_var.set(
                f"配置 loaded={values.get('loaded', '-')} valid={values.get('valid', '-')} flash_st={values.get('flash_st', '-')}"
            )
            for key, value in values.items():
                self._set_param(f"config.{key}", value, "CFG", dirty=False)
            return

        parts = line.split()
        prefix: str | None = None
        index = -1
        if len(parts) >= 2 and parts[1].startswith("servo"):
            prefix = parts[1]
            index = safe_int(prefix.replace("servo", ""), -1)
        elif "slot" in values or "index" in values:
            index = safe_int(values.get("slot") or values.get("index"), -1)
            if index >= 0:
                prefix = f"servo{index}"

        if prefix is not None:
            for key, value in values.items():
                self._set_param(f"{prefix}.{key}", value, "CFG", dirty=False)
            if 0 <= index < len(self.servo_widgets):
                widgets = self.servo_widgets[index]
                servo_map = {"id": "id", "pulse": "pulse", "time": "time", "mode": "mode", "en": "enabled"}
                for src, dst in servo_map.items():
                    if src in values:
                        widgets[dst].set(safe_int(values[src]))

    def _update_param_line(self, line: str) -> None:
        values = parse_kv(line)
        tokens = line.split()
        record = tokens[1] if len(tokens) >= 2 and "=" not in tokens[1] else None
        name = values.get("name") or values.get("key")
        value = values.get("value") or values.get("val")
        if name is not None and value is not None:
            self._set_param(name, value, "PARAM", dirty=False)
            return

        if record is not None:
            for key, item_value in values.items():
                if key not in {"ok", "st", "count"}:
                    self._set_param(f"{record}.{key}", item_value, "PARAM", dirty=False)
            return

        for key, item_value in values.items():
            if key not in {"ok", "st", "count"}:
                self._set_param(key, item_value, "PARAM", dirty=False)

    def _update_pid_line(self, line: str) -> None:
        values = parse_kv(line)
        tokens = line.split()
        axis = values.get("axis")
        if axis is None and len(tokens) >= 2 and "=" not in tokens[1]:
            axis = tokens[1].lower()
        if axis is None:
            group = (values.get("group") or values.get("target") or "").lower()
            index = safe_int(values.get("index"), -1)
            if group in {"rate", "angle"} and 0 <= index < 3:
                axis = ("roll", "pitch", "yaw")[index]

        if axis in self.pid_vars:
            for term in ("kp", "ki", "kd"):
                if term in values:
                    self.pid_vars[axis][term].set(values[term])
                    self._set_param(f"pid.{axis}.{term}", values[term], "PID", dirty=False)
            return

        for key, value in values.items():
            if key in {"axis", "ok", "st"}:
                continue
            lowered = key.lower().replace("_", ".")
            if lowered.startswith("pid."):
                name = lowered
            elif "." in lowered:
                name = f"pid.{lowered}"
            else:
                name = f"pid.{key}"
            self._set_param(name, value, "PID", dirty=False)
            self._sync_pid_quick_var(name, value)

    def _sync_pid_quick_var(self, name: str, value: str) -> None:
        parts = name.lower().split(".")
        if len(parts) >= 3 and parts[-2] in self.pid_vars and parts[-1] in self.pid_vars[parts[-2]]:
            self.pid_vars[parts[-2]][parts[-1]].set(value)

    def _set_param(self, name: str, value: str, source: str, dirty: bool) -> None:
        self.params[name] = {"value": value, "source": source, "dirty": dirty}
        dirty_text = "yes" if dirty else ""
        iid = self.param_iids.get(name)
        if iid is None:
            iid = f"p{len(self.param_iids)}"
            self.param_iids[name] = iid
            self.param_names_by_iid[iid] = name
        if self.param_tree.exists(iid):
            self.param_tree.item(iid, text=name, values=(value, source, dirty_text))
        else:
            self.param_tree.insert("", tk.END, iid=iid, text=name, values=(value, source, dirty_text))
        self._sync_pid_quick_var(name, value)

    def _on_param_select(self, _event: tk.Event) -> None:
        selection = self.param_tree.selection()
        if not selection:
            return
        name = self.param_names_by_iid.get(selection[0], selection[0])
        self.param_name_var.set(name)
        self.param_value_var.set(str(self.params.get(name, {}).get("value", "")))

    def _stage_param_edit(self) -> None:
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        if not name:
            messagebox.showerror("参数错误", "参数名不能为空")
            return
        self._set_param(name, value, "local", dirty=True)

    def _param_value_for_servo(self, index: int, param_key: str, widget_key: str, fallback: str) -> str:
        param_value = str(self.params.get(f"servo{index}.{param_key}", {}).get("value", "")).strip()
        if param_value:
            return param_value
        if 0 <= index < len(self.servo_widgets):
            return str(self.servo_widgets[index][widget_key].get()).strip()
        return fallback

    def _payload_for_param_edit(self, name: str, value: str) -> tuple[int, str]:
        lowered = name.strip().lower()
        parts = lowered.split(".")
        pid_aliases = {
            "pid.roll.kp": "coax.roll_angle_kp",
            "pid.pitch.kp": "coax.pitch_angle_kp",
            "pid.roll.kd": "coax.roll_rate_kd",
            "pid.pitch.kd": "coax.pitch_rate_kd",
            "pid.yaw.kp": "coax.yaw_angle_kp",
            "pid.yaw.kd": "coax.yaw_rate_kd",
            "pid.pos.x.kp": "coax.pos_x_kp",
            "pid.pos.y.kp": "coax.pos_y_kp",
            "pid.pos.z.kp": "coax.pos_z_kp",
            "pid.pos.z.ki": "coax.pos_z_ki",
            "pid.vel.x.kd": "coax.vel_x_kd",
            "pid.vel.y.kd": "coax.vel_y_kd",
            "pid.vel.z.kd": "coax.vel_z_kd",
            "pid.vel_loop.enable": "coax.vel_loop_enable",
        }
        if lowered in pid_aliases:
            payload = f"PARAM SET {pid_aliases[lowered]} {value}"
            return PROTO_REQ_PARAM_SET, payload
        if len(parts) == 2 and parts[0].startswith("servo") and parts[0][5:].isdigit():
            index = int(parts[0][5:])
            field = parts[1]
            if field == "id":
                payload = f"SERVO ID {index} {value}"
                return PROTO_REQ_SERVO_ID, payload
            if field in {"enabled", "en"}:
                payload = f"SERVO ENABLE {index} {value}"
                return PROTO_REQ_SERVO_ENABLE, payload
            if field == "mode":
                payload = f"SERVO MODE {index} {value}"
                return PROTO_REQ_SERVO_MODE, payload
            if field in {"pulse", "pulse_us"}:
                time_ms = self._param_value_for_servo(index, "time", "time", "500")
                payload = f"SERVO MOVE {index} {value} {time_ms}"
                return PROTO_REQ_SERVO_MOVE, payload
            if field in {"time", "time_ms"}:
                pulse = self._param_value_for_servo(index, "pulse", "pulse", "1500")
                payload = f"SERVO MOVE {index} {pulse} {value}"
                return PROTO_REQ_SERVO_MOVE, payload

        payload = f"PARAM SET {name} {value}"
        return PROTO_REQ_PARAM_SET, payload

    def _send_param_edit(self) -> None:
        self._stage_param_edit()
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        if name:
            function, payload = self._payload_for_param_edit(name, value)
            self._send_proto_once(function, payload, payload)

    def _send_pid_values(self) -> None:
        for axis, terms in self.pid_vars.items():
            parts = []
            for term in ("kp", "ki", "kd"):
                value = terms[term].get().strip()
                if value:
                    parts.append(f"{term}={value}")
                    self._set_param(f"pid.{axis}.{term}", value, "local", dirty=True)
            if parts:
                payload = f"PID SET {axis} {' '.join(parts)}"
                self._send_proto(PROTO_REQ_PID_SET, payload, payload)

    def _stage_text(self, stage: str) -> str:
        mapping = {
            "ready": "初始化完成",
            "probe": "读取芯片 ID",
            "status": "读取状态寄存器",
            "read": "读取数据",
            "init": "初始化/识别",
            "txrx": "发送接收",
            "split": "分段读写",
        }
        return mapping.get(stage, stage)

    def _hardware_hint(self, key: str, ok: bool, _values: dict[str, str]) -> str:
        if ok:
            return "初始化成功，当前读数符合预期。"
        if key == "FLASH":
            return "优先看 SPI1/CS/MISO/MOSI 和 GD25Q32 供电；ID 应为 C84016。"
        if key == "SPL06":
            return "重点看 SPI4、CS、MISO、器件方向或焊接型号；SPL06 ID 通常应为 0x10。"
        if key == "ICM42688":
            return "重点看 SPI2/IMU 硬件链路；CHIP_ID ?? 0xA1。"
        if key == "GPS":
            return "确认 M9N 接 USART2: PD5 TX 到 GPS RX，PD6 RX 接 GPS TX，115200 8N1，且 USART2 IRQ 已开。"
        if key == "MAG":
            return "确认磁力计在 I2C1: PB6 SCL / PB7 SDA，外部上拉、电源和地址匹配。"
        if key == "UART1":
            return "确认 USART1: PB14 TX 接 Ai-WB2 RX，PB15 RX 接 Ai-WB2 TX，115200 8N1。"
        if key == "WIFI":
            return "确认上位机 TCP 服务先监听 6666；固件会用 PC6 自动重启 Ai-WB2 触发重连。"
        return "查看返回码和初始化阶段。"

    def _warn_if_no_reply(self, sent: str, started_at: float) -> None:
        if self.last_reply_rx >= started_at:
            return
        if self.last_board_rx >= started_at:
            return
        if (time.monotonic() - started_at) < (CMD_REPLY_TIMEOUT_MS / 1000.0):
            return
        if self._transport_connected():
            mode = "TCP" if self.transport is self.tcp_transport else "串口"
            self.link_var.set(f"{mode} 已连接但超时未收到 STM32 回复，请检查固件")
            self._append(f"[上位机] {sent} 超时: STM32 未通过{mode}回复，请检查固件是否运行")

    def _link_keepalive_suppressed_reason(self) -> str | None:
        """探活必须让位的场合；此时链路空闲不代表掉线。"""
        if getattr(self, "firmware_update_pending", False) or getattr(
            self, "firmware_update_running", False
        ) or getattr(self, "firmware_programming", False):
            return "固件升级进行中"
        if self.v1_worker is not None and self.v1_worker.is_alive():
            return "V1 正在独占 USB CDC"
        return None

    def _send_keepalive_probe(self) -> bool:
        """空闲探活：只发只读 PING，不写日志、不改"最近命令"。

        刻意不走 _send/_send_proto，避免每 2 秒往验收日志里灌一条命令记录。
        """
        if not self._validation_command_allowed("PING"):
            return False
        if not self.transport.send_frame(PROTO_REQ_PING, b"PING"):
            return False
        # 旧固件不认结构化帧时补一条文本；串口 ASCII 兼容模式下 send_frame
        # 本身已经降级成 send_line，再补会变成重复 PING。
        if (
            self.transport is not self.udp_transport
            and self.structured_protocol_supported is False
            and not (self.transport is self.serial_transport and SERIAL_ASCII_COMPAT_MODE)
        ):
            self.transport.send_line("PING")
        return True

    def _maybe_send_link_keepalive(self) -> None:
        if not self._transport_connected():
            return
        if self._link_keepalive_suppressed_reason() is not None:
            return
        now = time.monotonic()
        if self.last_board_rx != 0.0 and (now - self.last_board_rx) < LINK_KEEPALIVE_IDLE_S:
            return
        if (now - self.last_keepalive_tx) < LINK_KEEPALIVE_MIN_INTERVAL_S:
            return
        if self._send_keepalive_probe():
            self.last_keepalive_tx = now

    def _check_link_health(self) -> None:
        self._refresh_serial_selection_lock()
        self._maybe_send_link_keepalive()
        link_style = "Pass.TLabel"
        if not self._transport_connected():
            link_style = "Fail.TLabel"
            if self.transport is self.tcp_transport:
                self.link_var.set("等待 Ai-WB2 接入 TCP")
            else:
                self.link_var.set("串口未打开")
        elif self.last_board_rx == 0.0:
            link_style = "Warn.TLabel"
            if self.transport is self.tcp_transport:
                self.link_var.set("TCP 已连接，等待 STM32 数据")
            elif self.transport is self.udp_transport:
                self.link_var.set("UDP 已打开，等待 STM32 文本回复")
            else:
                self.link_var.set("串口已打开，等待 STM32 数据")
        else:
            idle = time.monotonic() - self.last_board_rx
            mode = self._transport_label()
            suppressed = self._link_keepalive_suppressed_reason()
            if suppressed is not None:
                if idle > LINK_KEEPALIVE_IDLE_S:
                    link_style = "Warn.TLabel"
                    self.link_var.set(f"{mode} 空闲（探活暂停：{suppressed}）")
            elif idle > LINK_STALE_S:
                link_style = "Fail.TLabel"
                self.link_var.set(
                    f"{mode} 已连接但探活 {LINK_STALE_S:.0f} 秒无回复，请检查固件")
            elif idle > LINK_KEEPALIVE_IDLE_S:
                self.link_var.set(f"{mode} 链路正常（空闲探活中）")
        if hasattr(self, "link_status_label"):
            self.link_status_label.configure(style=link_style)
        self.after(1000, self._check_link_health)

    def _drain_rx(self) -> None:
        processed = 0
        try:
            while processed < RX_DRAIN_BATCH_SIZE:
                item = self.rx_queue.get_nowait()
                processed += 1
                if isinstance(item, tuple) and len(item) == 3 and item[0] == "proto":
                    _tag, function, text = item
                    self._handle_proto_frame(int(function), str(text))
                    continue
                if isinstance(item, tuple) and len(item) == 4 and item[0] == "udp_raw":
                    _tag, host, port, size = item
                    self.udp_raw_hidden_count += 1
                    self.udp_raw_last_note = f"hidden UDP binary frames={self.udp_raw_hidden_count} last={host}:{port} {size}B"
                    self.ident_link_var.set(self.udp_raw_last_note)
                    continue

                line = str(item)
                self._append(line)
                self._handle_board_line(line)
        except queue.Empty:
            pass
        delay_ms = RX_DRAIN_BUSY_MS if not self.rx_queue.empty() else RX_DRAIN_IDLE_MS
        self.after(delay_ms, self._drain_rx)

    def _on_close(self) -> None:
        if self.firmware_programming:
            messagebox.showwarning(
                "正在写入/校验",
                "固件 program/verify 完成前不能关闭窗口；请保持 USB 连接。",
            )
            return
        self.firmware_cancel_event.set()
        self.v1_cancel_event.set()
        self._validation_autosave_session(force=True)
        self._stop()
        self._ident_close_csv()
        self.destroy()


def main() -> None:
    app = DronePanel()
    app.mainloop()


if __name__ == "__main__":
    main()
