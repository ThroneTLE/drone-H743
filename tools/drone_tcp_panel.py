#!/usr/bin/env python3
"""Ground-station panel for the drone-H743 Ai-WB2 transparent link."""

from __future__ import annotations

import csv
import json
import math
import os
import queue
import re
import statistics
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    from .panel_lib import arm_banner as _panel_arm_banner, evidence as _panel_evidence
    from .panel_lib.pages import acceptance_v2 as _panel_acceptance_v2
    from .panel_lib.pages import drift as _panel_drift
    from .panel_lib.pages import dashboard as _panel_dashboard, flow_monitor as _panel_flow_monitor
    from .panel_lib.pages import gps_validity as _panel_gps_validity
    from .panel_lib import parameter_editor as _panel_parameter_editor
    from .panel_lib.pages import flow_ranging as _panel_flow
    from .panel_lib.pages import mechanical as _panel_mechanical
    from .panel_lib.pages import rc_wizard as _panel_rc
    from .panel_lib.pages import servo_debug as _panel_servo_debug
    from .panel_lib.pages import v1_metrology as _panel_v1
    from .panel_lib.pages import vibration as _panel_vibration
    from .panel_lib.pages import validation_v0 as _panel_validation_v0
    from .panel_lib import plotting as _panel_plotting
    from .panel_lib import proto as _panel_proto
    from .panel_lib import state as _panel_state
    from .panel_lib import connection_controls as _panel_connection_controls
    from .panel_lib import theme as _panel_theme
    from .panel_lib import transport as _panel_transport, rx_dispatch as _panel_rx
    from .panel_lib import viewport as _panel_viewport
except ImportError:  # Allows direct import and: python tools/drone_tcp_panel.py
    try:
        from tools.panel_lib import arm_banner as _panel_arm_banner
        from tools.panel_lib import evidence as _panel_evidence
        from tools.panel_lib.pages import acceptance_v2 as _panel_acceptance_v2
        from tools.panel_lib.pages import drift as _panel_drift
        from tools.panel_lib.pages import dashboard as _panel_dashboard, flow_monitor as _panel_flow_monitor
        from tools.panel_lib.pages import gps_validity as _panel_gps_validity
        from tools.panel_lib import parameter_editor as _panel_parameter_editor
        from tools.panel_lib.pages import flow_ranging as _panel_flow
        from tools.panel_lib.pages import mechanical as _panel_mechanical
        from tools.panel_lib.pages import rc_wizard as _panel_rc
        from tools.panel_lib.pages import servo_debug as _panel_servo_debug
        from tools.panel_lib.pages import v1_metrology as _panel_v1
        from tools.panel_lib.pages import vibration as _panel_vibration
        from tools.panel_lib.pages import validation_v0 as _panel_validation_v0
        from tools.panel_lib import plotting as _panel_plotting
        from tools.panel_lib import proto as _panel_proto
        from tools.panel_lib import state as _panel_state
        from tools.panel_lib import connection_controls as _panel_connection_controls
        from tools.panel_lib import theme as _panel_theme
        from tools.panel_lib import transport as _panel_transport, rx_dispatch as _panel_rx
        from tools.panel_lib import viewport as _panel_viewport
    except ImportError:
        from panel_lib import arm_banner as _panel_arm_banner
        from panel_lib import evidence as _panel_evidence
        from panel_lib.pages import acceptance_v2 as _panel_acceptance_v2
        from panel_lib.pages import drift as _panel_drift
        from panel_lib.pages import dashboard as _panel_dashboard, flow_monitor as _panel_flow_monitor
        from panel_lib.pages import gps_validity as _panel_gps_validity
        from panel_lib import parameter_editor as _panel_parameter_editor
        from panel_lib.pages import flow_ranging as _panel_flow
        from panel_lib.pages import mechanical as _panel_mechanical
        from panel_lib.pages import rc_wizard as _panel_rc
        from panel_lib.pages import servo_debug as _panel_servo_debug
        from panel_lib.pages import v1_metrology as _panel_v1
        from panel_lib.pages import vibration as _panel_vibration
        from panel_lib.pages import validation_v0 as _panel_validation_v0
        from panel_lib import plotting as _panel_plotting
        from panel_lib import proto as _panel_proto
        from panel_lib import state as _panel_state
        from panel_lib import connection_controls as _panel_connection_controls
        from panel_lib import theme as _panel_theme
        from panel_lib import transport as _panel_transport, rx_dispatch as _panel_rx
        from panel_lib import viewport as _panel_viewport

# Compatibility forwarding: existing callers may keep importing these names from
# tools.drone_tcp_panel while transport.py owns their implementations.
HAS_PYSERIAL = _panel_transport.HAS_PYSERIAL
PYSERIAL_ERROR = _panel_transport.PYSERIAL_ERROR
SERIAL_ASCII_COMPAT_MODE = _panel_transport.SERIAL_ASCII_COMPAT_MODE
SERIAL_TX_DEBUG_ENABLED = _panel_transport.SERIAL_TX_DEBUG_ENABLED
SerialTransport = _panel_transport.SerialTransport
TcpTransport = _panel_transport.TcpTransport
TransportBase = _panel_transport.TransportBase
UdpTransport = _panel_transport.UdpTransport
build_proto_frame = _panel_transport.build_proto_frame
match_remembered_serial_port = _panel_transport.match_remembered_serial_port
proto_crc8_dvb_s2 = _panel_transport.proto_crc8_dvb_s2
select_reenumerated_application_port = _panel_transport.select_reenumerated_application_port
serial = _panel_transport.serial
serial_device_identity_policy = _panel_transport.serial_device_identity_policy
serial_port_fingerprint = _panel_transport.serial_port_fingerprint
serial_port_identity = _panel_transport.serial_port_identity
udp_payload_is_probably_text = _panel_transport.udp_payload_is_probably_text
wait_for_application_serial = _panel_transport.wait_for_application_serial

# Protocol compatibility forwarding follows the same rule: proto.py owns the
# table and pure parsers while legacy imports keep seeing the same objects.
PROTO_COMPAT_FALLBACK_DELAY_MS = _panel_proto.PROTO_COMPAT_FALLBACK_DELAY_MS
PROTO_PROBE_SETTLE_MS = _panel_proto.PROTO_PROBE_SETTLE_MS
PROTO_HEADER = _panel_proto.PROTO_HEADER
PROTO_DIR_TO_FC = _panel_proto.PROTO_DIR_TO_FC
PROTO_DIR_FROM_FC = _panel_proto.PROTO_DIR_FROM_FC
PROTO_REQ_PING = _panel_proto.PROTO_REQ_PING
PROTO_REQ_STATUS = _panel_proto.PROTO_REQ_STATUS
PROTO_REQ_CONFIG = _panel_proto.PROTO_REQ_CONFIG
PROTO_REQ_PARAMS = _panel_proto.PROTO_REQ_PARAMS
PROTO_REQ_PID = _panel_proto.PROTO_REQ_PID
PROTO_REQ_BARO = _panel_proto.PROTO_REQ_BARO
PROTO_REQ_BARO_STREAM = _panel_proto.PROTO_REQ_BARO_STREAM
PROTO_REQ_FLASH = _panel_proto.PROTO_REQ_FLASH
PROTO_REQ_IMU = _panel_proto.PROTO_REQ_IMU
PROTO_REQ_MODULES = _panel_proto.PROTO_REQ_MODULES
PROTO_REQ_CAPS = _panel_proto.PROTO_REQ_CAPS
PROTO_REQ_SAVE = _panel_proto.PROTO_REQ_SAVE
PROTO_REQ_LOAD = _panel_proto.PROTO_REQ_LOAD
PROTO_REQ_DEFAULTS = _panel_proto.PROTO_REQ_DEFAULTS
PROTO_REQ_PARAM_SET = _panel_proto.PROTO_REQ_PARAM_SET
PROTO_REQ_PID_SET = _panel_proto.PROTO_REQ_PID_SET
PROTO_REQ_SERVO_MOVE = _panel_proto.PROTO_REQ_SERVO_MOVE
PROTO_REQ_SERVO_MOVE_ALL = _panel_proto.PROTO_REQ_SERVO_MOVE_ALL
PROTO_REQ_SERVO_ID = _panel_proto.PROTO_REQ_SERVO_ID
PROTO_REQ_SERVO_SETID = _panel_proto.PROTO_REQ_SERVO_SETID
PROTO_REQ_SERVO_MODE = _panel_proto.PROTO_REQ_SERVO_MODE
PROTO_REQ_SERVO_ENABLE = _panel_proto.PROTO_REQ_SERVO_ENABLE
PROTO_REQ_SERVO_ACTION = _panel_proto.PROTO_REQ_SERVO_ACTION
PROTO_REQ_SERVO_RAW = _panel_proto.PROTO_REQ_SERVO_RAW
PROTO_REQ_WIFI = _panel_proto.PROTO_REQ_WIFI
PROTO_REQ_GPS = _panel_proto.PROTO_REQ_GPS
PROTO_REQ_MAG = _panel_proto.PROTO_REQ_MAG
PROTO_REQ_RTOS = _panel_proto.PROTO_REQ_RTOS
PROTO_REQ_AIRFRAME = _panel_proto.PROTO_REQ_AIRFRAME
PROTO_REQ_IDENT = _panel_proto.PROTO_REQ_IDENT
PROTO_REQ_IMU_FRAME = _panel_proto.PROTO_REQ_IMU_FRAME
PROTO_REQ_IMU_CAL = _panel_proto.PROTO_REQ_IMU_CAL
PROTO_REQ_ACCEPTANCE = _panel_proto.PROTO_REQ_ACCEPTANCE
PROTO_REQ_RC = _panel_proto.PROTO_REQ_RC
PROTO_REQ_RCMAP = _panel_proto.PROTO_REQ_RCMAP
PROTO_REQ_SERVOTYPE = _panel_proto.PROTO_REQ_SERVOTYPE
PROTO_REQ_SERVO_CAL = _panel_proto.PROTO_REQ_SERVO_CAL
PROTO_MSG_CMD_LINE = _panel_proto.PROTO_MSG_CMD_LINE
PROTO_MSG_TEXT_LINE = _panel_proto.PROTO_MSG_TEXT_LINE
PROTO_MSG_CMD_RX = _panel_proto.PROTO_MSG_CMD_RX
PROTO_MSG_CMD_ACK = _panel_proto.PROTO_MSG_CMD_ACK
PROTO_MSG_CMD_ERR = _panel_proto.PROTO_MSG_CMD_ERR
PROTO_MSG_CMD_OK = _panel_proto.PROTO_MSG_CMD_OK
PROTO_MSG_PONG = _panel_proto.PROTO_MSG_PONG
PROTO_MSG_HW_FLASH = _panel_proto.PROTO_MSG_HW_FLASH
PROTO_MSG_HW_BARO = _panel_proto.PROTO_MSG_HW_BARO
PROTO_MSG_HW_IMU = _panel_proto.PROTO_MSG_HW_IMU
PROTO_MSG_STATUS_FLASH = _panel_proto.PROTO_MSG_STATUS_FLASH
PROTO_MSG_STATUS_BARO = _panel_proto.PROTO_MSG_STATUS_BARO
PROTO_MSG_STATUS_IMU = _panel_proto.PROTO_MSG_STATUS_IMU
PROTO_MSG_UART_STATS = _panel_proto.PROTO_MSG_UART_STATS
PROTO_MSG_CONFIG_SUMMARY = _panel_proto.PROTO_MSG_CONFIG_SUMMARY
PROTO_MSG_CONFIG_SERVO = _panel_proto.PROTO_MSG_CONFIG_SERVO
PROTO_MSG_PARAM_RECORD = _panel_proto.PROTO_MSG_PARAM_RECORD
PROTO_MSG_PID_RECORD = _panel_proto.PROTO_MSG_PID_RECORD
PROTO_MSG_FLASH_RECORD = _panel_proto.PROTO_MSG_FLASH_RECORD
PROTO_MSG_BARO_STATE = _panel_proto.PROTO_MSG_BARO_STATE
PROTO_MSG_BARO_DIAG = _panel_proto.PROTO_MSG_BARO_DIAG
PROTO_MSG_BARO_RAW = _panel_proto.PROTO_MSG_BARO_RAW
PROTO_MSG_BARO_STREAM = _panel_proto.PROTO_MSG_BARO_STREAM
PROTO_MSG_IMU_STATE = _panel_proto.PROTO_MSG_IMU_STATE
PROTO_MSG_IMU_SCALED = _panel_proto.PROTO_MSG_IMU_SCALED
PROTO_MSG_MODULES_SUMMARY = _panel_proto.PROTO_MSG_MODULES_SUMMARY
PROTO_MSG_CAPS_RECORD = _panel_proto.PROTO_MSG_CAPS_RECORD
PROTO_MSG_READY = _panel_proto.PROTO_MSG_READY
PROTO_MSG_SAVE_RESULT = _panel_proto.PROTO_MSG_SAVE_RESULT
PROTO_MSG_LOAD_RESULT = _panel_proto.PROTO_MSG_LOAD_RESULT
PROTO_MSG_DEFAULTS_RESULT = _panel_proto.PROTO_MSG_DEFAULTS_RESULT
PROTO_MSG_SERVO_RESULT = _panel_proto.PROTO_MSG_SERVO_RESULT
PROTO_MSG_WIFI_RECORD = _panel_proto.PROTO_MSG_WIFI_RECORD
PROTO_MSG_GPS_RECORD = _panel_proto.PROTO_MSG_GPS_RECORD
PROTO_MSG_MAG_RECORD = _panel_proto.PROTO_MSG_MAG_RECORD
PROTO_MSG_RTOS_RECORD = _panel_proto.PROTO_MSG_RTOS_RECORD
PROTO_MSG_FLASH_BENCH = _panel_proto.PROTO_MSG_FLASH_BENCH
PROTO_MSG_AIRFRAME_RECORD = _panel_proto.PROTO_MSG_AIRFRAME_RECORD
PROTO_MSG_RC_LIVE = _panel_proto.PROTO_MSG_RC_LIVE
PROTO_MSG_RC_MAP = _panel_proto.PROTO_MSG_RC_MAP
PROTO_MSG_SERVO_CAL = _panel_proto.PROTO_MSG_SERVO_CAL
PROTO_MSG_SERVO_TYPE = _panel_proto.PROTO_MSG_SERVO_TYPE
PROTO_MSG_TELEM_FRAME = _panel_proto.PROTO_MSG_TELEM_FRAME
PROTO_MSG_COMPONENTS = _panel_proto.PROTO_MSG_COMPONENTS
PROTO_MSG_BATTERY = _panel_proto.PROTO_MSG_BATTERY
PROTO_MAX_FRAME_PAYLOAD = _panel_proto.PROTO_MAX_FRAME_PAYLOAD
PROTO_BINARY_FUNCTIONS = _panel_proto.PROTO_BINARY_FUNCTIONS
ProtocolLineMixin = _panel_proto.ProtocolLineMixin
first_float = _panel_proto.first_float
first_value = _panel_proto.first_value
parse_kv = _panel_proto.parse_kv
safe_float = _panel_proto.safe_float
safe_int = _panel_proto.safe_int

# Panel-local persistence and logs are owned by state.py; these aliases preserve
# the original module API for callers and existing Tk wiring.
LOG_DIR = _panel_state.LOG_DIR
PANEL_CRASH_LOG = _panel_state.PANEL_CRASH_LOG
PANEL_STATE_PATH = _panel_state.PANEL_STATE_PATH
PanelStateMixin = _panel_state.PanelStateMixin
RC_WIZARD_TRACE_LOG = _panel_state.RC_WIZARD_TRACE_LOG
append_log = _panel_state.append_log
record_panel_crash = _panel_state.record_panel_crash

# V0 validation evidence I/O and write guards live outside the page module so
# every panel page keeps using the same safety/evidence contract.
EvidenceMixin = _panel_evidence.EvidenceMixin
FIRMWARE_BOOT_COMMAND = _panel_evidence.FIRMWARE_BOOT_COMMAND
VALIDATION_ALLOWED_COMMANDS = _panel_evidence.VALIDATION_ALLOWED_COMMANDS
VALIDATION_AUTOSAVE_PERIOD_S = _panel_evidence.VALIDATION_AUTOSAVE_PERIOD_S
VALIDATION_HEALTH_FRESH_S = _panel_evidence.VALIDATION_HEALTH_FRESH_S
VALIDATION_MOTOR_SAFE_MAX_US = _panel_evidence.VALIDATION_MOTOR_SAFE_MAX_US
VALIDATION_ROTATION_TARGET_SAMPLES = _panel_evidence.VALIDATION_ROTATION_TARGET_SAMPLES
VALIDATION_SAMPLE_FRESH_S = _panel_evidence.VALIDATION_SAMPLE_FRESH_S
VALIDATION_SNAPSHOT_REQUIRED_FIELDS = _panel_evidence.VALIDATION_SNAPSHOT_REQUIRED_FIELDS
VALIDATION_STATIC_TARGET_SAMPLES = _panel_evidence.VALIDATION_STATIC_TARGET_SAMPLES
VALIDATION_UI_STAGES = _panel_evidence.VALIDATION_UI_STAGES
signed_permutation_descriptor = _panel_evidence.signed_permutation_descriptor
validation_history_artifacts = _panel_evidence.validation_history_artifacts
validation_sample_from_snapshot = _panel_evidence.validation_sample_from_snapshot
validation_samples_from_csv = _panel_evidence.validation_samples_from_csv

# The first extracted page follows the same Mixin forwarding pattern while the
# legacy module keeps the stationary_drift module object available to callers.
AcceptanceV2PageMixin = _panel_acceptance_v2.AcceptanceV2PageMixin
DriftPageMixin = _panel_drift.DriftPageMixin
drift = _panel_drift.drift
FLOW_CALIBRATION_STAGES = _panel_flow.FLOW_CALIBRATION_STAGES
FlowRangingPageMixin = _panel_flow.FlowRangingPageMixin
FLOW_MONITOR_MAX_INTEGRATION_DT_S = _panel_flow_monitor.FLOW_MONITOR_MAX_INTEGRATION_DT_S
FLOW_MONITOR_MAX_SAMPLES = _panel_flow_monitor.FLOW_MONITOR_MAX_SAMPLES
FLOW_MONITOR_MAX_TRACK_POINTS = _panel_flow_monitor.FLOW_MONITOR_MAX_TRACK_POINTS
FLOW_MONITOR_POLL_PERIOD_S = _panel_flow_monitor.FLOW_MONITOR_POLL_PERIOD_S
FLOW_MONITOR_QUALITY_FULL_SCALE = _panel_flow_monitor.FLOW_MONITOR_QUALITY_FULL_SCALE
FLOW_MONITOR_RENDER_PERIOD_NS = _panel_flow_monitor.FLOW_MONITOR_RENDER_PERIOD_NS
FlowMonitorPageMixin = _panel_flow_monitor.FlowMonitorPageMixin
GpsValidityPageMixin = _panel_gps_validity.GpsValidityPageMixin
MAX_GPS_TRACK_POINTS = _panel_gps_validity.MAX_GPS_TRACK_POINTS
ParameterEditorMixin = _panel_parameter_editor.ParameterEditorMixin
MechanicalPageMixin = _panel_mechanical.MechanicalPageMixin
ServoDebugPageMixin = _panel_servo_debug.ServoDebugPageMixin
V1PageMixin = _panel_v1.V1PageMixin
# 页面常量归页面模块所有；此前 V1 那四个在两边各留了一份副本，属 §11.1 禁止的
# 共存拷贝，这里一并改为转发，避免两处数值日后各改各的。
V1_CAPTURE_PREP_SECONDS = _panel_v1.V1_CAPTURE_PREP_SECONDS
V1_FACE_RESIDUAL_FAIL_G = _panel_v1.V1_FACE_RESIDUAL_FAIL_G
V1_FACE_RESIDUAL_WARN_G = _panel_v1.V1_FACE_RESIDUAL_WARN_G
V1_STAGE_LABELS = _panel_v1.V1_STAGE_LABELS
ValidationV0PageMixin = _panel_validation_v0.ValidationV0PageMixin
VibrationPageMixin = _panel_vibration.VibrationPageMixin
v0_workflow_guidance = _panel_validation_v0.v0_workflow_guidance

# RC mapping/wizard compatibility forwarding.  The page module owns both its
# pure calibration helpers and Tk handlers while legacy imports keep working.
RC_CHANNEL_COUNT = _panel_rc.RC_CHANNEL_COUNT
RC_DETECT_DOMINANCE = _panel_rc.RC_DETECT_DOMINANCE
RC_DETECT_MIN_TRAVEL_US = _panel_rc.RC_DETECT_MIN_TRAVEL_US
RC_FUNCTIONS = _panel_rc.RC_FUNCTIONS
RC_LIVE_FRESH_S = _panel_rc.RC_LIVE_FRESH_S
RC_MIN_SPAN_US = _panel_rc.RC_MIN_SPAN_US
RC_US_MAX = _panel_rc.RC_US_MAX
RC_US_MIN = _panel_rc.RC_US_MIN
RC_WIZARD_CENTER_MARGIN = _panel_rc.RC_WIZARD_CENTER_MARGIN
RC_WIZARD_DOMINANCE = _panel_rc.RC_WIZARD_DOMINANCE
RC_WIZARD_HOLD_FRAMES = _panel_rc.RC_WIZARD_HOLD_FRAMES
RC_WIZARD_HOLD_TOLERANCE_US = _panel_rc.RC_WIZARD_HOLD_TOLERANCE_US
RC_WIZARD_MIN_DEVIATION_US = _panel_rc.RC_WIZARD_MIN_DEVIATION_US
RC_WIZARD_NON_CENTERING = _panel_rc.RC_WIZARD_NON_CENTERING
RC_WIZARD_STEPS = _panel_rc.RC_WIZARD_STEPS
RcWizardPageMixin = _panel_rc.RcWizardPageMixin
rc_channel_travel = _panel_rc.rc_channel_travel
rc_detect_channel = _panel_rc.rc_detect_channel
rc_map_is_valid = _panel_rc.rc_map_is_valid
rc_normalize = _panel_rc.rc_normalize
rc_wizard_build_map = _panel_rc.rc_wizard_build_map
rc_wizard_dominant = _panel_rc.rc_wizard_dominant
rc_wizard_gate_open = _panel_rc.rc_wizard_gate_open
rc_wizard_step_ready = _panel_rc.rc_wizard_step_ready
rc_wizard_window_stable = _panel_rc.rc_wizard_window_stable

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
        FLOW_RANGE_CALIBRATION_DIR,
        PROJECT_ROOT,
        SERVO_MECHANICAL_CALIBRATION_DIR,
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
            FLOW_RANGE_CALIBRATION_DIR,
            PROJECT_ROOT,
            SERVO_MECHANICAL_CALIBRATION_DIR,
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
            FLOW_RANGE_CALIBRATION_DIR,
            PROJECT_ROOT,
            SERVO_MECHANICAL_CALIBRATION_DIR,
            TELEMETRY_DIR,
            dated_directory,
            ensure_directory,
        )

try:
    from .ground_calibration import (
        FlowRangeSample,
        GroundCalibrationError,
        analyze_flow_axis,
        analyze_flow_zero,
        analyze_rotation_compensation,
        fit_range_two_point,
        validate_servo_geometry,
    )
    from .imu_metrology import DEFAULT_THRESHOLDS, MetrologyStage, MetrologyStatus
    from .imucal_protocol import (
        EncodedV1Candidate,
        commit_candidate as imucal_commit_candidate,
        load_and_encode_v1_candidate,
        parse_imucal_line,
        revert_candidate as imucal_revert_candidate,
        upload_and_apply as imucal_upload_and_apply,
        write_transaction_record as write_imucal_transaction_record,
    )
    from .imu_vibration_capture import CaptureLink
    from .v1_metrology_session import (
        CAPTURE_PLANS,
        DISCARDED_DIRNAME,
        V1Session,
        RETIRED_STAGE_LABELS,
        analyze_session,
        context_conflicts as v1_context_conflicts,
        discard_capture as discard_v1_capture,
        latest_session_manifest,
        load_session as load_v1_session,
        load_session_samples as load_v1_session_samples,
        minimum_samples as v1_minimum_samples,
        new_session as new_v1_session,
        persist_capture as persist_v1_capture,
        probe_capture as probe_v1_capture,
    )
except ImportError:
    try:
        from tools.ground_calibration import (
            FlowRangeSample,
            GroundCalibrationError,
            analyze_flow_axis,
            analyze_flow_zero,
            analyze_rotation_compensation,
            fit_range_two_point,
            validate_servo_geometry,
        )
        from tools.imu_metrology import DEFAULT_THRESHOLDS, MetrologyStage, MetrologyStatus
        from tools.imucal_protocol import (
            EncodedV1Candidate,
            commit_candidate as imucal_commit_candidate,
            load_and_encode_v1_candidate,
            parse_imucal_line,
            revert_candidate as imucal_revert_candidate,
            upload_and_apply as imucal_upload_and_apply,
            write_transaction_record as write_imucal_transaction_record,
        )
        from tools.imu_vibration_capture import CaptureLink
        from tools.v1_metrology_session import (
            CAPTURE_PLANS, DISCARDED_DIRNAME, RETIRED_STAGE_LABELS, V1Session, analyze_session,
            context_conflicts as v1_context_conflicts,
            discard_capture as discard_v1_capture,
            latest_session_manifest,
            load_session as load_v1_session,
            load_session_samples as load_v1_session_samples,
            minimum_samples as v1_minimum_samples,
            new_session as new_v1_session,
            persist_capture as persist_v1_capture,
            probe_capture as probe_v1_capture,
        )
    except ImportError:
        from ground_calibration import (
            FlowRangeSample,
            GroundCalibrationError,
            analyze_flow_axis,
            analyze_flow_zero,
            analyze_rotation_compensation,
            fit_range_two_point,
            validate_servo_geometry,
        )
        from imu_metrology import DEFAULT_THRESHOLDS, MetrologyStage, MetrologyStatus
        from imucal_protocol import (
            EncodedV1Candidate,
            commit_candidate as imucal_commit_candidate,
            load_and_encode_v1_candidate,
            parse_imucal_line,
            revert_candidate as imucal_revert_candidate,
            upload_and_apply as imucal_upload_and_apply,
            write_transaction_record as write_imucal_transaction_record,
        )
        from imu_vibration_capture import CaptureLink
        from v1_metrology_session import (
            CAPTURE_PLANS, DISCARDED_DIRNAME, RETIRED_STAGE_LABELS, V1Session, analyze_session,
            context_conflicts as v1_context_conflicts,
            discard_capture as discard_v1_capture,
            latest_session_manifest,
            load_session as load_v1_session,
            load_session_samples as load_v1_session_samples,
            minimum_samples as v1_minimum_samples,
            new_session as new_v1_session,
            persist_capture as persist_v1_capture,
            probe_capture as probe_v1_capture,
        )


DEFAULT_HOST = _panel_connection_controls.DEFAULT_HOST
DEFAULT_PORT = _panel_connection_controls.DEFAULT_PORT
DEFAULT_MODULE_IP = "192.168.223.181"
DEFAULT_UDP_LOCAL_PORT = 6668
DEFAULT_UDP_MODULE_PORT = 7777
MAX_BARO_SAMPLES = 2000
MAX_GPS_TRACK_POINTS = 5000
BARO_STREAM_PERIOD_MS = 50
IMU_POLL_PERIOD_MS = 100
RC_POLL_PERIOD_MS = 50
IMU_RENDER_PERIOD_MS = 50
RX_DRAIN_BATCH_SIZE = 200
RX_DRAIN_IDLE_MS = 100
RX_DRAIN_BUSY_MS = 10
LOG_FLUSH_PERIOD_MS = 100
MAX_PENDING_LOG_LINES = 500
FIRMWARE_BOOT_REPLY_TIMEOUT_MS = 5000
FIRMWARE_CDC_DISCONNECT_TIMEOUT_S = 8.0
FIRMWARE_RECOVERY_HINT = (
    "写入阶段取消或失败可能留下不完整应用，软件不会自动重试。"
    "若飞控没有重新出现，请用 BOOT0 进入 ROM DFU；"
    "仍无法连接时使用 ST-Link 恢复。"
)
CMD_REPLY_TIMEOUT_MS = 2500
# USB CDC 是纯命令/响应通道：飞控的周期性 VOFA 遥测只走 Ai-WB2 socket
# (App/Src/app_vofa.c)，USB 上只有命令回复镜像。空闲时链路必须由上位机主动
# 探活，否则 last_board_rx 永远不刷新，状态栏会把"没人问"误判成"掉线"。
LINK_KEEPALIVE_IDLE_S = 2.0
LINK_KEEPALIVE_MIN_INTERVAL_S = 1.5
LINK_STALE_S = 6.0

# Shared theme remains available through this legacy module for callers that
# historically imported the palette from ``drone_tcp_panel``.
UI_PALETTE = _panel_theme.UI_PALETTE
UI_FONT = _panel_theme.UI_FONT
UI_MONO = _panel_theme.UI_MONO
UI_SIZE = _panel_theme.UI_SIZE
UI_SIZE_SM = _panel_theme.UI_SIZE_SM
UI_SIZE_TITLE = _panel_theme.UI_SIZE_TITLE
apply_matplotlib_theme = _panel_theme.apply_matplotlib_theme
# 可选 matplotlib 守卫归 panel_lib/plotting.py 所有，页面模块与大面板共用一份判定。
FigureCanvasTkAgg = _panel_plotting.FigureCanvasTkAgg
Figure = _panel_plotting.Figure
HAS_MATPLOTLIB = _panel_plotting.HAS_MATPLOTLIB
MATPLOTLIB_ERROR = _panel_plotting.MATPLOTLIB_ERROR

MODULES = ()  # Overview rows now arrive from the firmware registry.

MODULE_ALIASES = {
    "FLASH": "FLASH",
    "GD25Q32": "FLASH",
    "SPL06": "SPL06",
    "BARO": "SPL06",
    "ICM42688": "ICM42688",
    "IMU": "ICM42688",
    "FLOW": "FLOW",
    "OPTICAL_FLOW": "FLOW",
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




def firmware_update_link_gate(
    *,
    connected: bool,
    serial_transport_selected: bool,
) -> tuple[bool, str]:
    """Block only on facts the target cannot possibly check for us.

    解锁/油门/快照新鲜度由固件的 BOOT 处理器自己判定（APP_BOOT_REQUEST_ARMED、
    ESC_HIGH、NO_VALID_SNAPSHOT、SNAPSHOT_STALE，并在调度时再查一次），拒绝原因
    会原样回给上位机。主机再复制一份这套判断，只会在遥测缺一行时把烧录永久卡死，
    却一点安全性都不增加——所以主机这一侧只保留"命令根本发不出去"的情形。
    """

    if not serial_transport_selected:
        return False, "请选择顶部 serial 通道；ROM DFU 只从 USB CDC 发起"
    if not connected:
        return False, "USB CDC 尚未连接"
    return True, "USB CDC 已连接"


def firmware_update_snapshot_advisory(
    values: dict[str, str],
    *,
    sample_age_s: float,
) -> tuple[str, str]:
    """Describe the last snapshot for the operator; never gate on it.

    返回 (level, text)，level ∈ {"ok", "warn", "unknown"}。"unknown" 表示主机手上
    没有可信快照——这不是拒绝理由，固件会在收到 BOOT 时自己判断。
    """

    if values.get("valid") != "1" or values.get("source") != "stabilizer_snapshot":
        return "unknown", "飞控状态未知（等待遥测；固件会在收到 BOOT 时自行判定）"
    if sample_age_s < 0.0 or sample_age_s == float("inf"):
        return "unknown", "飞控状态未知（尚未收到快照）"
    if sample_age_s > VALIDATION_SAMPLE_FRESH_S:
        return "unknown", f"飞控状态已过期（{sample_age_s:.1f}s 前）"
    try:
        armed = int(values["armed"], 0)
        motor_1 = int(values["m1"], 0)
        motor_2 = int(values["m2"], 0)
    except (KeyError, TypeError, ValueError):
        return "unknown", "快照缺少 armed/m1/m2"
    if armed != 0:
        return "warn", f"飞控仍处于 armed={armed}，固件会拒绝进入 DFU"
    if motor_1 > VALIDATION_MOTOR_SAFE_MAX_US or motor_2 > VALIDATION_MOTOR_SAFE_MAX_US:
        return "warn", f"电机输出不安全：m1={motor_1} m2={motor_2}"
    return "ok", f"已解锁保护 armed=0 · m1={motor_1} m2={motor_2}"














def airframe_record_from_line(line: str) -> dict[str, str | float] | None:
    if not line.startswith("AIRFRAME "):
        return None
    values = parse_kv(line)
    record: dict[str, str | float] = {"line": line}
    for key, value in values.items():
        parsed = first_float(values, key)
        record[key] = parsed if parsed is not None else value
    return record


def normalize_module_key(key: str) -> str | None:
    return MODULE_ALIASES.get(key.upper())


VerticalScrolledFrame = _panel_viewport.VerticalScrolledFrame
FixedActionViewport = _panel_viewport.FixedActionViewport


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


class DronePanel(_panel_connection_controls.ConnectionControlsMixin, ValidationV0PageMixin, EvidenceMixin, AcceptanceV2PageMixin, VibrationPageMixin, ServoDebugPageMixin, MechanicalPageMixin, RcWizardPageMixin, V1PageMixin, FlowRangingPageMixin, FlowMonitorPageMixin, GpsValidityPageMixin, ParameterEditorMixin, _panel_dashboard.DashboardPageMixin, DriftPageMixin, PanelStateMixin, ProtocolLineMixin, tk.Tk):
    def __init__(self) -> None:
        # 必须早于 super().__init__()：Tk 根窗口一旦创建，DPI 感知就无法再改。
        self.ui_dpi_scale = enable_hidpi_awareness()
        super().__init__()
        self._init_gps_state()
        self._init_parameter_editor_state()
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
        self.selected_module = None
        self.baro_capture_enabled = tk.BooleanVar(value=True)
        self.baro_buffer: list[dict[str, float | str]] = []
        self._baro_dirty = False
        self._last_baro_plot_ns = 0
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
        self.validation_latest_timestamp_ms: int | None = None
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
        self.servo_widgets: list[dict[str, tk.Variable]] = []
        self.structured_protocol_supported: bool | None = None
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
        self._panel_state = self._load_panel_state()
        self.auto_connect_var = tk.BooleanVar(
            value=bool(self._panel_state.get("auto_connect", True))
        )
        self.autoconnect_var = tk.StringVar(value="上次连接：读取中…")
        self.ident_link_var = tk.StringVar(value="UDP text: waiting")

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
            value="待机：连接飞控 USB CDC 即可一键升级"
        )
        self.firmware_safety_var = tk.StringVar(value="链路：· 等待 USB CDC    飞控：· 状态未知")
        self.firmware_image_info_var = tk.StringVar(value="固件镜像：尚未验证")
        # --- 04 · 遥控 RC ---
        self.rc_link_var = tk.StringVar(value="遥控：等待飞控上报")
        self.rc_status_var = tk.StringVar(value="待机：连接飞控后点「从飞控读取」拉取当前映射")
        self.rc_detect_var = tk.StringVar(value="")
        self.rc_deadband_var = tk.StringVar(value="20")
        self.rc_channels: list[int] = [0] * RC_CHANNEL_COUNT
        self.rc_live_time = 0.0
        self.rc_last_poll = 0.0
        self.rc_map_generation: int | None = None
        self.rc_link_values: dict[str, str] = {}
        self.rc_map_values: dict[str, dict[str, int]] = {}
        self.rc_map_state = ""
        self.rc_detect_function: str | None = None
        self.rc_detect_samples: list[list[int]] = []
        self.rc_detect_deadline = 0.0
        self.rc_sweep_active = False
        self.rc_sweep_min: list[int] = []
        self.rc_sweep_max: list[int] = []
        self.rc_center: list[int] = []
        self.rc_wizard_step_var = tk.StringVar(value="")
        self.rc_wizard_prompt_var = tk.StringVar(
            value="点「开始引导校准」；开始前让会自动回中的杆松手，油门放哪都行"
        )
        self.rc_wizard_detect_var = tk.StringVar(value="")
        self.rc_wizard_result_var = tk.StringVar(value="")
        self.rc_wizard_progress_var = tk.DoubleVar(value=0.0)
        self.rc_wizard_active = False
        self.rc_wizard_index = 0
        self.rc_wizard_center: list[int] = []
        self.rc_wizard_min: list[int] = []
        self.rc_wizard_max: list[int] = []
        self.rc_wizard_window: list[list[int]] = []
        self.rc_wizard_results: dict[tuple[str, int], tuple[int, int]] = {}
        self.rc_wizard_armed = False
        self.rc_wizard_baseline: list[int] = []
        self.rc_wizard_release_channel: int | None = None
        self._rc_wizard_last_detail = ""
        self.firmware_unknown_usb_override_var = tk.BooleanVar(value=False)
        self.firmware_unknown_usb_confirmed_port = ""
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
        self.v1_status_var = tk.StringVar(value="尚未开始：连接飞控并通过安全门，然后新建 IMU 校准会话")
        self.v1_counts_var = tk.StringVar(value="尚无 IMU 校准会话")
        self.v1_analysis_var = tk.StringVar(value="六面 accel=NOT_RUN · 陀螺静止=NOT_RUN")
        self.v1_analysis_summary = None
        self.v1_encoded_candidate: EncodedV1Candidate | None = None
        self.v1_candidate_applied = False
        # 表格行 -> 计划/记录；体检结果按 (文件名, mtime) 缓存，避免每次渲染重读上万行 CSV。
        self.v1_row_plan: dict[str, object] = {}
        self.v1_row_record: dict[str, object] = {}
        self.v1_probe_cache: dict[tuple[str, int], object] = {}
        # 静止漂移自检：录制期间只往 buffer 里塞，分析全在停止之后做。
        self.drift_recording = False
        self.imu_calibration_generation = 0
        self.imu_calibration_firmware_crc32 = ""
        self._drift_pending_motion: dict[str, object] | None = None
        self.drift_samples: list[object] = []
        self.drift_deadline = 0.0
        self.drift_last_sequence = -1
        self.drift_report = None
        self.drift_baseline = None
        self.drift_baseline_path: Path | None = None
        self.drift_duration_var = tk.StringVar(value="60")
        self.drift_status_var = tk.StringVar(
            value="未开始：把飞机放在不会晃的桌面上，点开始后 30~60 秒内别碰它")
        self.drift_result_var = tk.StringVar(value="尚无录制结果")
        self.drift_compare_var = tk.StringVar(value="")
        self.v2_lease_id = 0
        self.v2_active = False
        self.v2_status_var = tk.StringVar(value="无桨验收未启动：要求坐标、IMU、舵机机械参数均已写入并拆桨")
        self.v2_live_var = tk.StringVar(value="尚无无桨控制链安全快照")
        self.v2_stage_var = tk.StringVar(value="rc_center")
        self.v2_tick_count = 0
        self.v2_session_dir: Path | None = None

        # 舵机机械校准只保存地面证据。目标端中心/方向/行程参数尚未参数化，
        # 所以这里不会把输入值伪装成已经写入固件的校准结果。
        self.mechanical_rows: list[dict[str, tk.Variable]] = []
        self.mechanical_status_var = tk.StringVar(
            value="未开始：当前飞控仍使用 1500 µs 硬编码中心；本页先完成机械测量与证据记录"
        )
        self.mechanical_last_report_var = tk.StringVar(value="尚未保存机械校准证据")
        self.mechanical_target_var = tk.StringVar(value="飞控参数：尚未读取")
        self.mechanical_target_records: dict[str, dict[str, str]] = {}
        self.mechanical_target_applied = False
        self.mechanical_target_commit_pending = False
        self.mechanical_commit_connection_generation: int | None = None
        self.mechanical_reboot_check_pending = False
        self.mechanical_reboot_verified = False
        self.mechanical_last_poll = 0.0

        # 光流/组合测距地面采样。不同步骤的原始样本分开保存，只有具备完整证据的
        # 项目才计算诊断值；旋转补偿若缺目标端补偿后速度则明确保持未验收。
        first_flow_stage = next(iter(FLOW_CALIBRATION_STAGES))
        self.flow_cal_stage_var = tk.StringVar(value=FLOW_CALIBRATION_STAGES[first_flow_stage])
        self.flow_cal_stage_by_label = {
            label: key for key, label in FLOW_CALIBRATION_STAGES.items()
        }
        self.flow_cal_reference_distance_var = tk.StringVar(value="0.50")
        self.flow_cal_near_height_var = tk.StringVar(value="0.30")
        self.flow_cal_far_height_var = tk.StringVar(value="1.00")
        self.flow_cal_status_var = tk.StringVar(value="未开始：先读取一次，确认光流质量和组合测距有效")
        self.flow_cal_live_var = tk.StringVar(value="FLOW / RANGE 尚无回包")
        self.flow_cal_result_var = tk.StringVar(value="尚无分析结果")
        self.flow_cal_collecting = False
        self.flow_cal_last_poll = 0.0
        self.flow_cal_active_stage: str | None = None
        self.flow_cal_last_target_sample_ms = -1
        self.flow_cal_samples: dict[str, list[FlowRangeSample]] = {
            stage: [] for stage in FLOW_CALIBRATION_STAGES
        }
        self.flow_diag_values: dict[str, str] = {}
        self.flow_latest_gyro_z_dps: float | None = None
        self.flow_cal_results: dict[str, dict[str, object]] = {}
        # “传感器 · 光流”监控页的状态全部归 FlowMonitorPageMixin 自己所有。
        self._init_flow_monitor_state()

        self.module_state: dict[str, dict[str, tk.StringVar]] = {}
        self.baro_vars: dict[str, tk.StringVar] = {}
        self.imu_vars: dict[str, tk.StringVar] = {}
        self.gps_vars: dict[str, tk.StringVar] = {}
        self.mag_vars: dict[str, tk.StringVar] = {}

        self._configure_style()
        self._build_ui()
        self._bind_gps_visibility_events()
        self.serial_port_var.trace_add("write", self._on_serial_port_selection_change)
        self.after(1000, self._check_link_health)
        self.after(RX_DRAIN_IDLE_MS, self._drain_rx)
        self.after(LOG_FLUSH_PERIOD_MS, self._flush_log)
        self.after(250, self._baro_tick)
        self.after(250, self._flow_monitor_tick)
        self.after(100, self._imu_poll_tick)
        self.after(100, self._firmware_drain_events)
        self.after(100, self._v1_drain_events)
        self.after(500, self._drift_tick)
        self.after(250, self._v2_tick)
        self.after_idle(self._validation_load_latest_artifact)
        self.after_idle(self._v1_load_latest_session)
        self.after_idle(self._restore_last_connection)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        # Tk 默认把回调异常打到 stderr；从资源管理器启动时没有 stderr，界面就"突然
        # 没了"。这里改成落盘 + 弹窗，并且让窗口活下来，不丢失已经做完的校准步骤。
        sys.excepthook = self._report_uncaught_exception

    def report_callback_exception(self, exc_type, exc_value, exc_tb) -> None:
        summary = record_panel_crash(exc_type, exc_value, exc_tb)
        self._crash_count = getattr(self, "_crash_count", 0) + 1
        try:
            self.autoconnect_var.set(
                f"发生内部错误（第 {self._crash_count} 次），详情见 {PANEL_CRASH_LOG}"
            )
        except tk.TclError:
            pass
        # 同一个错误往往每帧都触发，只在第一次弹窗，之后靠状态栏和日志。
        if self._crash_count == 1:
            try:
                messagebox.showerror(
                    "面板内部错误",
                    f"操作没有完成，但窗口已保留。\n\n{summary}\n\n"
                    f"完整堆栈：{PANEL_CRASH_LOG}",
                )
            except tk.TclError:
                pass

    def _report_uncaught_exception(self, exc_type, exc_value, exc_tb) -> None:
        record_panel_crash(exc_type, exc_value, exc_tb)

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
        _panel_theme.PanelThemeMixin._configure_style(self)

    # ------------------------------------------------------------------
    # 上次连接记录
    # ------------------------------------------------------------------











    def _build_ui(self) -> None:
        try:
            from .panel_lib.shell import build_ui
        except ImportError:
            from panel_lib.shell import build_ui
        build_ui(self)


    def _build_overview_page(self, parent: ttk.Frame) -> None:
        try:
            from .panel_lib.pages.overview import mount_overview
        except ImportError:
            from panel_lib.pages.overview import mount_overview
        mount_overview(self, parent)

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
            apply_matplotlib_theme(self.baro_figure, self.ui_palette)
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

    # ------------------------------------------------------------------
    # 04 · 遥控 RC
    # ------------------------------------------------------------------


    # --- RC：状态读写 -------------------------------------------------


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

        action = ttk.LabelFrame(parent, text="2 · 进入 DFU 并烧录", padding=10)
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
        # 解锁/油门由固件的 BOOT 处理器判定并回拒绝原因，这里只是把已知状态摆出来，
        # 不再充当第二道闸门。
        self.firmware_safety_label = ttk.Label(
            action,
            textvariable=self.firmware_safety_var,
            wraplength=1080,
            style="Muted.TLabel",
        )
        self.firmware_safety_label.pack(fill=tk.X, pady=(8, 0))
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

    def _firmware_link_gate(self) -> tuple[bool, str]:
        """能不能把 BOOT 命令送到正确的目标——只判断这个。"""
        ok, reason = firmware_update_link_gate(
            connected=self._transport_connected(),
            serial_transport_selected=self.transport is self.serial_transport,
        )
        if not ok:
            return ok, reason
        selected_display = self.serial_port_var.get().strip()
        selected_port = self._serial_port_map.get(selected_display, selected_display)
        active_port = self.serial_transport.active_port
        if not active_port or selected_port.casefold() != active_port.casefold():
            return False, f"连接端口与下拉选择不一致：active={active_port or '-'} selected={selected_port or '-'}"
        identity = self._serial_port_identity.get(active_port)
        policy, identity_reason = serial_device_identity_policy(identity)
        if policy == "rejected":
            # 刷错设备是固件无从知晓的风险，这一条必须留在主机侧。
            return False, identity_reason
        return True, f"{active_port} · {identity_reason}"

    def _firmware_safety_advisory(self) -> tuple[str, str]:
        if not _panel_rx.snapshot_is_current(self):
            return "unknown", "飞控状态未知（快照来自已断开的连接）"
        age_s = (
            time.monotonic() - self.validation_latest_host_time
            if self.validation_latest_host_time > 0.0
            else float("inf")
        )
        if (
            self.transport is self.serial_transport
            and self.validation_latest_host_time > 0.0
            and self.validation_latest_transport_generation
            != self.serial_transport.connection_generation
        ):
            return "unknown", "飞控状态未知（快照来自上一次连接）"
        return firmware_update_snapshot_advisory(
            self.validation_latest_values,
            sample_age_s=age_s,
        )


    def _firmware_refresh_safety(self) -> None:
        ok, reason = self._firmware_link_gate()
        level, advisory = self._firmware_safety_advisory()
        mark = {"ok": "✓", "warn": "!", "unknown": "·"}[level]
        self.firmware_safety_var.set(
            ("链路：✓ " if ok else "链路：✗ ") + reason + f"    飞控：{mark} {advisory}"
        )
        label = getattr(self, "firmware_safety_label", None)
        if label is not None:
            label.configure(
                style="Fail.TLabel" if not ok else
                ("Warn.TLabel" if level == "warn" else "Muted.TLabel")
            )
        busy = (
            self.firmware_update_pending
            or self.firmware_update_running
            or self.firmware_building
        )
        if hasattr(self, "firmware_start_button"):
            self.firmware_start_button.configure(
                state=tk.NORMAL if ok and not busy else tk.DISABLED
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
        ok, reason = self._firmware_link_gate()
        if not ok:
            messagebox.showwarning("无法向飞控发送 BOOT", reason)
            self._firmware_refresh_safety()
            return
        if not self._firmware_confirm_unknown_identity():
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

    def _firmware_confirm_unknown_identity(self) -> bool:
        """未知 USB 身份：一次性确认，不再让用户长期挂着一个勾选框。"""
        active_port = self.serial_transport.active_port or ""
        identity = self._serial_port_identity.get(active_port)
        policy, identity_reason = serial_device_identity_policy(identity)
        if policy != "unknown":
            self.firmware_unknown_usb_override_var.set(False)
            self.firmware_unknown_usb_confirmed_port = ""
            return True
        if (
            self.firmware_unknown_usb_override_var.get()
            and self.firmware_unknown_usb_confirmed_port.casefold() == active_port.casefold()
        ):
            return True
        confirmed = messagebox.askyesno(
            "无法确认这是飞控",
            f"{identity_reason}\n\n端口：{active_port or '-'}\n\n"
            "继续会把固件写入这个设备。确认它就是飞控的 application CDC 吗？",
        )
        self.firmware_unknown_usb_override_var.set(confirmed)
        self.firmware_unknown_usb_confirmed_port = active_port if confirmed else ""
        return confirmed

    def _firmware_start_update_after_build(self) -> None:
        if self.firmware_update_pending or self.firmware_update_running:
            return
        ok, reason = self._firmware_link_gate()
        if not ok:
            messagebox.showwarning("无法向飞控发送 BOOT", reason)
            self._firmware_refresh_safety()
            return
        level, advisory = self._firmware_safety_advisory()
        if level == "warn" and not messagebox.askyesno(
            "飞控状态不安全",
            f"{advisory}\n\n固件预计会拒绝这次 BOOT。仍要发送吗？",
        ):
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
        ok, reason = self._firmware_link_gate()
        if not ok:
            self.firmware_update_pending = False
            self._firmware_record_log(attempt, f"REFUSED: link lost before BOOT: {reason}")
            self.firmware_status_var.set(f"发送 BOOT 前链路失效：{reason}")
            self._firmware_refresh_safety()
            messagebox.showerror("无法向飞控发送 BOOT", reason)
            return
        level, advisory = self._firmware_safety_advisory()
        self._firmware_record_log(attempt, f"host_advisory level={level} detail={advisory}")
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
        # 刚烧完的飞控一定是重启过的：seqlock 序号回到很小的值。不清基线的话，
        # 后续每一帧快照都会被单调守卫丢弃，升级页会一直停在"状态未知"。
        self.validation_latest_sequence = None
        self.validation_latest_timestamp_ms = None
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













































    # 与固件 APP_ImuHealthLevel 一一对应。

    def _build_imu_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="请求 IMU", command=lambda: self._send_proto(PROTO_REQ_IMU, "IMU?")).pack(side=tk.LEFT)
        ttk.Checkbutton(top, text=f"备用轮询 {IMU_POLL_PERIOD_MS} ms", variable=self.imu_poll_enabled).pack(side=tk.LEFT, padx=(10, 0))
        ttk.Button(top, text="仅显示归零", command=self._reset_imu_attitude).pack(side=tk.LEFT, padx=8)
        ttk.Label(top, text="不参与坐标系校准", style="Muted.TLabel").pack(side=tk.LEFT)

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
            apply_matplotlib_theme(self.gps_figure, self.ui_palette)
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

    def _build_params_page(self, parent: ttk.Frame) -> None:
        top = ttk.Frame(parent)
        top.pack(fill=tk.X)
        ttk.Button(top, text="读取参数/PID", command=self._read_all_params).pack(side=tk.LEFT)
        ttk.Button(top, text="保存到 Flash", command=lambda: self._send_proto_once(PROTO_REQ_SAVE, "SAVE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="从 Flash 读取", command=lambda: self._send_proto_once(PROTO_REQ_LOAD, "LOAD")).pack(side=tk.LEFT)
        ttk.Button(top, text="恢复默认", command=lambda: self._send_proto(PROTO_REQ_DEFAULTS, "DEFAULTS")).pack(side=tk.LEFT, padx=6)
        ttk.Label(top, textvariable=self.config_summary_var).pack(side=tk.LEFT, padx=(14, 0))

        panes = ttk.Frame(parent)
        panes.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        left = ttk.Frame(panes)
        right = ttk.Frame(panes)
        left.pack(fill=tk.X)
        right.pack(fill=tk.X, pady=(10, 0))

        self.param_tree = ttk.Treeview(left, columns=("target", "draft", "source", "status"), show="tree headings")
        self.param_tree.heading("#0", text="参数")
        self.param_tree.column("#0", width=220, anchor=tk.W)
        for col, label, width in [
            ("target", "目标值", 110), ("draft", "本地草稿", 110),
            ("source", "来源/单位", 135), ("status", "状态", 150),
        ]:
            self.param_tree.heading(col, text=label)
            self.param_tree.column(col, width=width, anchor=tk.W)
        self.param_tree.pack(fill=tk.BOTH, expand=True)
        self.param_tree.bind("<<TreeviewSelect>>", self._on_param_select)

        edit = ttk.LabelFrame(right, text="参数编辑", padding=10)
        edit.pack(fill=tk.X)
        self.param_name_var = tk.StringVar()
        self.param_value_var = tk.StringVar()
        ttk.Label(edit, text="名称").grid(row=0, column=0, sticky=tk.W, pady=4)
        self.param_name_entry = ttk.Entry(edit, textvariable=self.param_name_var)
        self.param_name_entry.grid(row=0, column=1, sticky=tk.EW, pady=4)
        ttk.Label(edit, text="值").grid(row=1, column=0, sticky=tk.W, pady=4)
        self.param_value_entry = ttk.Entry(edit, textvariable=self.param_value_var)
        self.param_value_entry.grid(row=1, column=1, sticky=tk.EW, pady=4)
        ttk.Label(edit, textvariable=self.param_value_error_var, style="Fail.TLabel").grid(
            row=2, column=0, columnspan=2, sticky=tk.W, pady=(0, 4)
        )
        actions = ttk.Frame(edit)
        actions.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Button(actions, text="暂存修改", command=self._stage_param_edit).pack(side=tk.LEFT)
        ttk.Button(actions, text="发送修改", command=self._send_param_edit).pack(side=tk.LEFT, padx=6)
        ttk.Button(actions, text="撤销草稿", command=self._discard_param_draft).pack(side=tk.LEFT)
        ttk.Label(edit, textvariable=self.param_editor_status_var, style="Muted.TLabel").grid(
            row=4, column=0, columnspan=2, sticky=tk.W, pady=(6, 0)
        )
        edit.columnconfigure(1, weight=1)

        self._build_cascade_editor(right)

    def _build_command_page(self, parent: ttk.Frame) -> None:
        quick = ttk.LabelFrame(parent, text="兼容 / 调试命令", padding=10)
        quick.pack(fill=tk.X)
        for index, (label, command) in enumerate([
            ("PING", "PING"),
            ("STATUS?", "STATUS?"),
            ("CONFIG?", "CONFIG?"),
            ("PARAM?", "PARAM?"),
            ("AIRFRAME?", "AIRFRAME?"),
            ("BARO?", "BARO?"),
            ("GPS?", "GPS?"),
            ("MAG?", "MAG?"),
            ("SAVE", "SAVE"),
            ("LOAD", "LOAD"),
        ]):
            ttk.Button(
                quick, text=label, command=lambda c=command: self._send(c)
            ).grid(row=index // 4, column=index % 4, padx=3, pady=3, sticky=tk.W)
        for column in range(4):
            quick.columnconfigure(column, weight=1)

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
            "也预留解析: BARO pressure=... temp=... alt=..., PARAM name=... value=..., 当前四环 PARAM 参数。\n"
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

    def _spin(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: int, maximum: int, command) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        box = ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=10,
                          style="Numeric.TSpinbox")
        box.grid(row=row, column=1, sticky=tk.EW, pady=4)
        if command is not None:
            ttk.Button(parent, text="应用", command=command).grid(row=row, column=2, padx=(6, 0))

    def _scale(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable, minimum: int, maximum: int) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        scale = ttk.Scale(parent, from_=minimum, to=maximum, variable=variable,
                          orient=tk.HORIZONTAL, style="Numeric.Horizontal.TScale")
        scale.grid(row=row, column=1, sticky=tk.EW, pady=4)
        ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=8,
                    style="Numeric.TSpinbox").grid(row=row, column=2, padx=(6, 0))


    def _stop(self) -> None:
        self._cancel_bluetooth()
        self._parameter_on_disconnect()
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
        # 断开后重连的可能是刚重启过的飞控，序号基线必须一起丢掉。
        self.validation_latest_sequence = None
        self.validation_latest_timestamp_ms = None
        # 重连后重新拉一次遥控映射：接的可能已经是另一台飞控。
        self.rc_map_generation = None
        self.rc_live_time = 0.0
        self.mechanical_target_records.clear()
        self.mechanical_target_applied = False
        self.mechanical_target_commit_pending = False
        self.mechanical_reboot_check_pending = False
        self.mechanical_reboot_verified = False
        self.mechanical_last_poll = 0.0
        self.mechanical_target_var.set("飞控参数：连接已停止，等待重新读取")
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

    def _send(self, line: str) -> None:
        if not self._validation_guard_command(line):
            return
        self.last_cmd_var.set(f"最近命令: {line}")
        self._append(f"> {line}")
        if not self.transport.send_line(line):
            self._append("[上位机] 发送失败")
        elif line in {"PING", "STATUS?", "CONFIG?", "PARAM?", "BARO?", "FLOW?", "GPS?", "MAG?", "AIRFRAME?"}:
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
        self.overview_page.request()

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
        self._request_airframe()

    def _request_airframe(self) -> None:
        self._send_proto(PROTO_REQ_AIRFRAME, "AIRFRAME?")

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
            PROTO_MSG_SERVO_TYPE: self._update_servo_type_line,
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
        self.overview_page.request()

    def _select_sensor_tab(self, tab: tk.Misc) -> None:
        # 气压计/IMU/GPS 现在挂在二级“传感器” Notebook 下，顶层 notebook.select(子页)
        # 会因为子页不是它的 tab 而抛 TclError，必须先选中分组再选子页。
        sensor_group_tab = getattr(self, "sensor_group_tab", None)
        if sensor_group_tab is not None and hasattr(self, "sensor_notebook"):
            self.notebook.select(sensor_group_tab)
            self.sensor_notebook.select(tab)
            return
        self.notebook.select(tab)

    def _open_baro_tab(self) -> None:
        self._select_sensor_tab(self.baro_tab)
        self._send_proto(PROTO_REQ_BARO, "BARO?")

    def _open_imu_tab(self) -> None:
        self._select_sensor_tab(self.imu_tab)
        self._send_proto(PROTO_REQ_IMU, "IMU?")

    def _open_gps_tab(self) -> None:
        self._select_sensor_tab(self.gps_tab)
        self._send_proto(PROTO_REQ_GPS, "GPS?")

    def _refresh_detail(self) -> None:
        page = getattr(self, "overview_page", None)
        if page is not None:
            page.refresh_detail()

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
        self._parameter_handle_result_line(line)
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
        elif line.startswith("IMUCAL "):
            self._v1_sync_target_state(line)
        elif line.startswith("SERVOCAL "):
            self._mechanical_handle_target_line(line)
        elif line.startswith("SERVOTYPE "):
            self._update_servo_type_line(line)
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
        elif line.startswith("FLOW "):
            self._update_flow_line(line)
            # 监控页自己收一份：标定页那条解析是标定专用的，两边不共用状态。
            self._flow_monitor_handle_line(line)
        elif line.startswith("TELEM "):
            self._dashboard_handle_line(line)
        elif line.startswith("RCMAP "):
            self._rc_handle_map_line(line)
        elif line.startswith("RC "):
            self._rc_handle_live_line(line)
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
        if "gz_dps" in values:
            self.flow_latest_gyro_z_dps = safe_float(values.get("gz_dps"))
        elif "gz_mdps" in values:
            self.flow_latest_gyro_z_dps = safe_float(values.get("gz_mdps")) * 0.001
        elif "gz" in values:
            # 固件的 `IMU sample seq=` 行按 units=mg_mdps_cdeg 发**裸 gz**（mdps）。
            # 上面两个名字固件从来没发过，所以旋转补偿阶一直拿不到偏航角速度，
            # 299 个实采样本无一带 gyro_z（2026-08-30 台架实测）。下面 has_sample
            # 早就在认裸 "gx" 了，这里只是把漏掉的一半补上。
            self.flow_latest_gyro_z_dps = safe_float(values.get("gz")) * 0.001
        if "rate_hz" in values and "level" in values:
            self._validation_accept_imu_health(values)
            return
        if "cal_generation" in values:
            self.imu_calibration_generation = safe_int(values.get("cal_generation"), 0)
            self.imu_calibration_firmware_crc32 = values.get("firmware_crc32", "")
            return
        self._drift_accept_sample(values)
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

        if mod == "ARM":
            _panel_arm_banner.handle_arm_rsp(self, payload)
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
        outer_selection = self.notebook.select()
        sensor_group_tab = getattr(self, "sensor_group_tab", None)
        if sensor_group_tab is not None and hasattr(self, "sensor_notebook"):
            # IMU 监视页移进二级“传感器”分组后，顶层选中的是分组本身，
            # 拿顶层 selection 比 imu_tab 会永远为假，轮询/重绘就此停摆。
            sensor_visible = outer_selection == str(sensor_group_tab)
            imu_tab_visible = sensor_visible and (
                self.sensor_notebook.select() == str(self.imu_tab)
            )
        else:
            imu_tab_visible = outer_selection == str(self.imu_tab)
        firmware_tab_visible = _panel_viewport.leaf_tab_visible(self, self.firmware_tab, outer_selection)
        calibration_group_tab = getattr(self, "calibration_group_tab", None)
        if calibration_group_tab is not None and hasattr(self, "calibration_notebook"):
            calibration_visible = outer_selection == str(calibration_group_tab)
            calibration_selection = self.calibration_notebook.select() if calibration_visible else ""
            v1_tab_visible = calibration_visible and calibration_selection == str(self.v1_tab)
            validation_tab_visible = calibration_visible and calibration_selection == str(self.validation_tab)
            rc_tab_visible = calibration_visible and calibration_selection == str(self.rc_tab)
            mechanical_tab_visible = calibration_visible and calibration_selection == str(self.mechanical_tab)
            flow_range_tab_visible = calibration_visible and calibration_selection == str(self.flow_range_tab)
            v2_tab_visible = calibration_visible and calibration_selection == str(self.v2_tab)
        else:
            # 保留纯逻辑测试/旧嵌入者的扁平 Notebook 兼容；真实界面走上面的二级“校准”。
            v1_tab_visible = outer_selection == str(self.v1_tab)
            validation_tab_visible = outer_selection == str(self.validation_tab)
            rc_tab_visible = outer_selection == str(self.rc_tab)
            mechanical_tab_visible = False
            flow_range_tab_visible = False
            v2_tab_visible = outer_selection == str(getattr(self, "v2_tab", ""))
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
        poll_requested = self.drift_recording or self.validation_poll_requested or (
            self.imu_poll_enabled.get() and imu_tab_visible
        ) or (
            firmware_tab_visible and not firmware_busy
        ) or (
            v1_tab_visible and not v1_busy
        ) or (
            (mechanical_tab_visible or flow_range_tab_visible or v2_tab_visible)
            and not firmware_busy and not v1_busy
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
        if mechanical_tab_visible and self._transport_connected():
            if now - self.mechanical_last_poll >= 1.0:
                self.mechanical_last_poll = now
                self._send_proto_silent(PROTO_REQ_SERVO_CAL, "SERVOCAL?")
        self._flow_monitor_poll_tick(now)
        self._dashboard_poll_tick(now)
        if getattr(self, "flow_cal_collecting", False):
            if not self._transport_connected():
                self.flow_cal_collecting = False
                self.flow_cal_active_stage = None
                self.flow_cal_status_var.set("连接已断开，采样已停止；本步证据未分析")
                self.flow_cal_start_button.configure(state=tk.NORMAL)
                self.flow_cal_stop_button.configure(state=tk.DISABLED)
            elif now - self.flow_cal_last_poll >= 0.20:
                self.flow_cal_last_poll = now
                if self._validation_command_allowed("FLOW?"):
                    self.transport.send_line("FLOW?")
                self._send_proto_silent(PROTO_REQ_IMU, "IMU?")
        # RC 页要看摇杆实时位置，10Hz 的 IMU 轮询节奏不够跟手；这里单独按 20Hz 拉。
        # 自动识别和行程标定都靠这条流采样，降频会直接让标定采不到端点。
        if rc_tab_visible and self._transport_connected():
            # 页面一打开就把飞控里的映射拉回来。不自动拉的话表格是空的，用户得先
            # 猜到要点"从飞控读取"——和 V0 页那个"等一个永远不会亮的 ✓"是同一类坑。
            if self.rc_map_generation != self.serial_transport.connection_generation:
                self.rc_map_generation = self.serial_transport.connection_generation
                self._send_proto_silent(PROTO_REQ_RCMAP, "RCMAP?")
            if now - self.rc_last_poll >= RC_POLL_PERIOD_MS / 1000.0:
                self.rc_last_poll = now
                self._send_proto_silent(PROTO_REQ_RC, "RC?")
        if rc_tab_visible:
            self._rc_render()
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
        try:
            from .panel_lib.link_activity import keepalive_suppressed_reason
        except ImportError:
            from panel_lib.link_activity import keepalive_suppressed_reason
        return keepalive_suppressed_reason(self)

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
        _panel_rx.drain_rx(self, RX_DRAIN_BATCH_SIZE, RX_DRAIN_BUSY_MS, RX_DRAIN_IDLE_MS)

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
        self.destroy()


def main() -> None:
    app = DronePanel()
    try:
        app.mainloop()
    except Exception:  # noqa: BLE001 - 顶层兜底：宁可留下日志也不要静默消失
        record_panel_crash(*sys.exc_info())
        raise


if __name__ == "__main__":
    main()
