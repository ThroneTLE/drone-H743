"""Threaded acquisition/state machine; worker threads never call Tk."""
from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import asdict, dataclass, replace
from typing import Any
from uuid import uuid4

from .plans import ExperimentPlan, PlanPoint
from .records import BenchSample, FcSnapshot


@dataclass(frozen=True)
class UiEvent:
    kind: str
    data: Any = None


class StopVoltageReached(RuntimeError):
    """Auto collection requested an immediate low-voltage stop."""


@dataclass
class AckTransaction:
    state: str
    expected: dict[str, str]
    sent_at: float
    generation: int
    outcome: str = "pending"


@dataclass(frozen=True)
class LatestObservation:
    snapshot: FcSnapshot | None
    scale_grams: float | None
    quality: tuple[str, ...]
    generation: int
    unfolded_fc_time_ms: int | None = None
    fc_received_at: float | None = None
    scale_received_at: float | None = None


class AcquisitionEngine:
    HEARTBEAT_S = 0.10
    ACK_TIMEOUT_S = 0.25
    SNAPSHOT_TIMEOUT_S = 0.5
    SOURCE_FRESH_MS = 250
    ESC_CURRENT_FRESH_MS = 1000
    SCALE_FRESH_S = 0.25
    STARTUP_STREAM_GRACE_S = 0.60
    TELEMETRY_GAP_GRACE_S = 2.0
    MAX_PENDING = 32

    def __init__(self, connection, load_cell, store, *,
                 snapshot_hz: float = 15.0, clock=time.monotonic) -> None:
        if not 1 <= snapshot_hz <= 200:
            raise ValueError("快照频率必须在 1..200 Hz")
        self.connection = connection
        self.load_cell = load_cell
        self.store = store
        self.snapshot_period = 1.0 / snapshot_hz
        validator = getattr(connection, "validate_snapshot_rate", None)
        if validator is None:
            raise ValueError("连接对象必须显式提供快照速率能力验证")
        self.rate_decision = validator(snapshot_hz)
        store.update_metadata(fc_link_rate=self.rate_decision)
        self.clock = clock
        self.ui_events: queue.Queue[UiEvent] = queue.Queue()
        self.propcal_events: queue.Queue = queue.Queue(maxsize=128)
        self.esc_events: queue.Queue = queue.Queue(maxsize=32)
        self.last_rejection_reason = ""
        self.cancel = threading.Event()
        self._shutdown = threading.Event()
        self._state_lock = threading.Lock()
        self._observation_lock = threading.Lock()
        self._ack = threading.Condition()
        self._snapshot_condition = threading.Condition()
        self._running = False
        self._operation_owner: str | None = None
        self._armed = False
        self._command = (0.0, 0.0)
        self._max_pct = 0
        self._generation = connection.generation
        self._nonce = 0
        self._pending_snapshots: dict[int, tuple[float, int]] = {}
        self._latest_fc = None  # snapshot, receive_s, generation, unfolded_fc_ms
        self._latest_scale = None  # grams, receive_s
        self._pending_ack: AckTransaction | None = None
        self._plan_set_waiters = 0
        self._window_token = 0
        self._request_id = 0
        self._last_set_send_time = -math.inf
        self._threads: list[threading.Thread] = []
        self._plan_thread: threading.Thread | None = None
        self._source_times = [set(), set()]
        self._last_emitted_source_times: tuple[int, int] | None = None
        self._fc_epoch = 0
        self._last_fc_raw = None
        self._last_snapshot_request_time = -math.inf
        self._mapping_recorded = False
        self._recorded_mapping = None
        self._startup_grace_used = False
        self._startup_grace_until = 0.0
        self._stream_established = False
        self._telemetry_gap_started: float | None = None

    def start_io(self) -> None:
        if any(thread.is_alive() for thread in self._threads):
            return
        self._shutdown.clear()
        self._generation = self.connection.generation
        self._threads = []
        for target, name in (
            (self._fc_loop, "tbench-fc"),
            (self._scale_loop, "tbench-scale"),
            (self._heartbeat_loop, "tbench-heartbeat"),
        ):
            thread = threading.Thread(
                target=self._guarded_loop, args=(target, name),
                name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def _guarded_loop(self, target, name: str) -> None:
        try:
            target()
        except Exception as exc:
            self.cancel.set()
            self._armed = False
            self._shutdown.set()
            self._cancel_transaction(f"{name}_failed")
            try:
                from .fc_protocol import stop_command
                self.connection.send_command(stop_command())
                self.connection.send_command(stop_command())
            finally:
                self.ui_events.put(UiEvent("error", f"{name} 后台失败，已紧急 STOP：{exc}"))

    def close(self) -> None:
        self.cancel.set()
        self.stop()
        self._shutdown.set()
        self._cancel_transaction("shutdown")
        with self._snapshot_condition:
            self._snapshot_condition.notify_all()
        for thread in (*self._threads, self._plan_thread):
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=0.6)
        self.connection.disconnect()
        self._threads = []
        self._plan_thread = None
        with self._state_lock:
            self._operation_owner = None

    def run_plan(self, plan: ExperimentPlan, *, max_percent: int) -> None:
        if plan.reference_voltage_v is not None:
            observation = self.latest_observation()
            voltage = (observation.snapshot.voltage_v
                       if observation.snapshot is not None else None)
            if (voltage is None or abs(voltage - plan.reference_voltage_v)
                    > plan.voltage_tolerance_v):
                raise RuntimeError(
                    "智能计划预览后的带载电压已变化，请重新预览")
        if not self.claim_operation("plan"):
            raise RuntimeError("台架正由其他操作占用")
        with self._state_lock:
            self._running = True
        self._plan_thread = threading.Thread(
            target=self._run_plan, args=(plan, max_percent),
            name="tbench-plan", daemon=True)
        self._plan_thread.start()

    def claim_operation(self, owner: str) -> bool:
        if not owner:
            raise ValueError("operation owner must not be empty")
        with self._state_lock:
            if (self._operation_owner is not None or self._running
                    or self._shutdown.is_set()
                    or not self.connection.is_connected):
                return False
            self._operation_owner = owner
            self.last_rejection_reason = ""
            self.cancel.clear()
            return True

    def release_operation(self, owner: str) -> None:
        with self._state_lock:
            if self._operation_owner == owner:
                self._operation_owner = None

    @property
    def operation_owner(self) -> str | None:
        with self._state_lock:
            return self._operation_owner

    @property
    def active_operation(self) -> str | None:
        return self.operation_owner

    @property
    def armed(self) -> bool:
        with self._state_lock:
            return self._armed

    def set_targets(self, upper: float, lower: float) -> bool:
        transaction = self._begin_set(upper, lower, wait_for_slot=True)
        return self._wait_transaction(transaction)

    def latest_observation(self) -> LatestObservation:
        now = self.clock()
        live_generation = self.connection.generation
        connected = bool(self.connection.is_connected)
        with self._observation_lock:
            fc_value = self._latest_fc
            scale_value = self._latest_scale
        quality: list[str] = []
        snapshot = None
        unfolded = None
        fc_received = None
        if not connected:
            quality.append("fc_disconnected")
        elif fc_value is None:
            quality.append("fc_missing")
        elif fc_value[2] != live_generation:
            quality.append("fc_generation_stale")
        elif now - fc_value[1] < 0:
            quality.append("fc_time_invalid")
        elif now - fc_value[1] > self.SNAPSHOT_TIMEOUT_S:
            quality.append("fc_stale")
        else:
            source, fc_received, _generation, unfolded = fc_value
            held_ms = (now - fc_received) * 1000.0
            erpm = list(source.erpm)
            erpm_age = list(source.erpm_age_ms)
            for index in range(2):
                age = erpm_age[index]
                if erpm[index] is None:
                    quality.append(f"erpm_{index + 1}_missing")
                if erpm[index] is not None and (
                        age is None or age < 0
                        or age + held_ms > self.SOURCE_FRESH_MS):
                    erpm[index] = None
                    erpm_age[index] = None
                    quality.append(f"erpm_{index + 1}_stale")
                elif age is not None:
                    erpm_age[index] = int(round(age + held_ms))
            voltage = source.voltage_v
            voltage_age = source.voltage_age_ms
            if voltage is None:
                quality.append("voltage_missing")
            if voltage is not None and (
                    voltage_age is None
                    or voltage_age < 0
                    or voltage_age + held_ms > self.SOURCE_FRESH_MS):
                voltage = None
                voltage_age = None
                quality.append("voltage_stale")
            elif voltage_age is not None:
                voltage_age = int(round(voltage_age + held_ms))
            current = source.total_current_a
            current_age = source.current_age_ms
            if current is None:
                quality.append("board_current_missing")
            if current is not None and (
                    current_age is None
                    or current_age < 0
                    or current_age + held_ms > self.SOURCE_FRESH_MS):
                current = None
                current_age = None
                quality.append("board_current_stale")
            elif current_age is not None:
                current_age = int(round(current_age + held_ms))
            esc_current = list(source.esc_current_a)
            esc_current_age = list(source.esc_current_age_ms)
            for index in range(2):
                age = esc_current_age[index]
                if esc_current[index] is None:
                    quality.append(f"esc_current_{index + 1}_missing")
                if esc_current[index] is not None and (
                        age is None or age < 0
                        or age + held_ms > self.ESC_CURRENT_FRESH_MS):
                    esc_current[index] = None
                    esc_current_age[index] = None
                    quality.append(f"esc_current_{index + 1}_stale")
                elif age is not None:
                    esc_current_age[index] = int(round(age + held_ms))
            snapshot = replace(
                source, erpm=tuple(erpm), erpm_age_ms=tuple(erpm_age),
                voltage_v=voltage, voltage_age_ms=voltage_age,
                total_current_a=current, current_age_ms=current_age,
                esc_current_a=tuple(esc_current),
                esc_current_age_ms=tuple(esc_current_age))
        scale_grams = None
        scale_received = None
        if scale_value is None:
            quality.append("scale_missing")
        elif now - scale_value[1] < 0:
            quality.append("scale_time_invalid")
        elif now - scale_value[1] > self.SCALE_FRESH_S:
            quality.append("scale_stale")
        else:
            scale_grams, scale_received = scale_value
        return LatestObservation(
            snapshot=snapshot, scale_grams=scale_grams,
            quality=tuple(quality), generation=live_generation,
            unfolded_fc_time_ms=unfolded, fc_received_at=fc_received,
            scale_received_at=scale_received)

    def capture_latest_sample(self, run_id: str, segment_id: str, *,
                              mode: str = "dual", phase: str = "steady",
                              direction: str = "steady",persist: bool = True) -> BenchSample | None:
        with self._ack:
            upper,lower=self._command
        point = PlanPoint(segment_id, upper, lower, phase, direction)
        sample = self._joined_sample(run_id, mode, point)
        missing = self._required_missing(sample)
        if missing:
            raise RuntimeError("核心采样失效：" + ",".join(missing))
        source_times=self._sample_source_times(sample)
        source_status=self._classify_sample_sources(source_times)
        if source_status=="duplicate": return None
        if source_status=="regression": raise RuntimeError("eRPM来源时刻倒退")
        self._last_emitted_source_times=source_times
        if persist:
            self.store.sample(sample)
            self.ui_events.put(UiEvent("sample", sample))
        return sample

    def commit_steady_samples(self,samples) -> None:
        """Publish a complete accepted segment after independent-source checks."""
        for sample in samples:
            self.store.sample(sample)
            self.ui_events.put(UiEvent("sample",sample))

    def wait_for_adaptive_stability(self, run_id: str, point: PlanPoint,
                                    *, mode: str = "dual",
                                    stop_voltage_v: float | None = None):
        if not point.adaptive:
            raise ValueError("adaptive stability requires an adaptive PlanPoint")
        return self._await_adaptive_stability(
            run_id, mode, point, stop_voltage_v=stop_voltage_v)

    @staticmethod
    def _next_request_id(previous: int) -> int:
        value = (int(previous) + 1) & 0xFFFFFFFF
        return value or 1

    def _fresh_snapshot(self) -> bool:
        return (self._latest_fc is not None
                and self._latest_fc[2] == self._generation
                and 0 <= self.clock() - self._latest_fc[1] <= self.SNAPSHOT_TIMEOUT_S)

    def arm(self, max_percent: int) -> bool:
        from .fc_protocol import arm_command
        deadline = self.clock() + self.SNAPSHOT_TIMEOUT_S
        with self._snapshot_condition:
            while not self._fresh_snapshot() and not self.cancel.is_set() and self.clock() < deadline:
                self._snapshot_condition.wait(max(0.005, deadline - self.clock()))
        if not self._fresh_snapshot() or self.cancel.is_set():
            self.store.event("arm_refused", reason="fresh_snapshot_required_or_cancelled")
            return False
        if self._latest_fc[0].esc_protocol != 2:
            self.store.event("arm_refused", reason="DSHOT300_BIDIR_required",
                             esc_protocol=self._latest_fc[0].esc_protocol)
            self.ui_events.put(UiEvent(
                "error", "当前飞控不是 DSHOT300_BIDIR，拒绝台架 ARM"))
            return False
        self._request_id = self._next_request_id(self._latest_fc[0].last_request_id)
        self._window_token = self._request_id
        self._max_pct = max_percent
        transaction = self._begin_transaction(
            arm_command(max_percent, request_id=self._request_id), "armed",
            {"request_id": str(self._request_id), "token": str(self._window_token),
             "max_pct": str(max_percent)})
        success = self._wait_transaction(transaction)
        self._armed = success
        if success:
            self._telemetry_gap_started = None
        if success and not self._startup_grace_used:
            self._startup_grace_used = True
            self._startup_grace_until = self.clock() + self.STARTUP_STREAM_GRACE_S
            self.store.event(
                "startup_stream_grace", duration_s=self.STARTUP_STREAM_GRACE_S)
        return success

    def _begin_set(self, upper: float, lower: float, *, wait_for_slot: bool) -> AckTransaction | None:
        transaction, _reason = self._try_begin_set(
            upper, lower, wait_for_slot=wait_for_slot)
        return transaction

    def _try_begin_set(self, upper: float | None, lower: float | None, *,
                       wait_for_slot: bool) -> tuple[AckTransaction | None, str]:
        from .fc_protocol import set_command
        if not self._armed or self.cancel.is_set():
            return None, "inactive_or_cancelled"
        if upper is not None and lower is not None and max(upper, lower) > self._max_pct:
            raise ValueError("指令超过本次 ARM 上限")
        deadline = self.clock() + self.ACK_TIMEOUT_S
        with self._ack:
            if not wait_for_slot:
                if self._plan_set_waiters:
                    return None, "yield_to_plan"
                if self._pending_ack is not None:
                    return None, "transaction_pending"
                if self.clock() - self._last_set_send_time < self.HEARTBEAT_S:
                    return None, "rate_limited"
            else:
                self._plan_set_waiters += 1
            try:
                while wait_for_slot and not self.cancel.is_set():
                    pending = self._pending_ack
                    due_in = self.HEARTBEAT_S - (
                        self.clock() - self._last_set_send_time)
                    if pending is None and due_in <= 0:
                        break
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        return None, "send_slot_timeout"
                    self._ack.wait(max(0.005, min(remaining,
                                                   due_in if pending is None else remaining)))
                if self.cancel.is_set():
                    return None, "cancelled_before_send"
                if self._pending_ack is not None:
                    return None, "transaction_pending"
                if upper is None or lower is None:
                    upper, lower = self._command
                # The board keeps 0.01 % rounded half-up and echoes that; send the
                # same value so the ACK compares equal (8.125 vs "8.13" fails by 1e-15).
                upper, lower = (math.floor(float(value) * 100.0 + 0.5) / 100.0
                                for value in (upper, lower))
                self._request_id = self._next_request_id(self._request_id)
                command = set_command(
                    upper, lower, request_id=self._request_id,
                    window_token=self._window_token)
                transaction = AckTransaction(
                    "set",
                    {"request_id": str(self._request_id), "token": str(self._window_token),
                     "upper_pct": format(float(upper), ".6g"),
                     "lower_pct": format(float(lower), ".6g")},
                    self.clock(), self._generation)
                if not self.connection.send_command(command):
                    transaction.outcome = "send_failed"
                    return transaction, "send_failed"
                self._pending_ack = transaction
                self._last_set_send_time = transaction.sent_at
                self._command = (float(upper), float(lower))
                return transaction, "sent"
            finally:
                if wait_for_slot:
                    self._plan_set_waiters -= 1
                    self._ack.notify_all()

    def _begin_transaction(self, command: str, state: str,
                           expected: dict[str, str]) -> AckTransaction:
        with self._ack:
            transaction = AckTransaction(
                state, expected, self.clock(), self._generation)
            if not self.connection.send_command(command):
                transaction.outcome = "send_failed"
            else:
                self._pending_ack = transaction
            return transaction

    def _wait_transaction(self, transaction: AckTransaction | None) -> bool:
        if transaction is None:
            return False
        deadline = transaction.sent_at + self.ACK_TIMEOUT_S
        with self._ack:
            while transaction.outcome == "pending" and not self.cancel.is_set():
                remaining = deadline - self.clock()
                if remaining <= 0:
                    transaction.outcome = "timeout"
                    if self._pending_ack is transaction:
                        self._pending_ack = None
                    self._ack.notify_all()
                    break
                self._ack.wait(max(0.005, remaining))
        self.store.event(
            "command_ack", state=transaction.state,
            request_id=transaction.expected.get("request_id"),
            result=transaction.outcome)
        return transaction.outcome == "matched"

    def _cancel_transaction(self, reason: str) -> None:
        with self._ack:
            transaction = self._pending_ack
            if transaction is not None:
                transaction.outcome = reason
                self._pending_ack = None
            self._ack.notify_all()

    def stop(self) -> bool:
        from .fc_protocol import stop_command
        self.cancel.set()
        with self._state_lock:
            self._armed = False
        self._telemetry_gap_started = None
        self._cancel_transaction("cancelled")
        first = self.connection.send_command(stop_command())
        second = self.connection.send_command(stop_command())
        try:
            self.store.event("stop", generation=self.connection.generation,
                             sent=first or second)
        except Exception as exc:
            self.ui_events.put(UiEvent("error", f"STOP 已发送，但日志写入失败：{exc}"))
        return first or second

    def _run_plan(self, plan: ExperimentPlan, max_percent: int) -> None:
        run_id = uuid4().hex[:12]
        self._source_times = [set(), set()]
        self._last_emitted_source_times = None
        measured_voltages: list[float] = []
        layer_v = plan.points[0].voltage_layer_v if plan.points else None
        layer_label = plan.points[0].voltage_layer_label if plan.points else ""
        self.ui_events.put(UiEvent("run_started", {
            "run_id": run_id, "voltage_layer_label": layer_label,
            "voltage_layer_target_v": layer_v,
            "total_points": len(plan.points),
            "adaptive_points": sum(1 for point in plan.points
                                   if point.adaptive)}))
        completed = False
        skipped_points = 0
        try:
            if not self.arm(max_percent):
                raise TimeoutError("TBENCH ARM 未获匹配确认")
            for point_index, point in enumerate(plan.points, start=1):
                if self.cancel.is_set():
                    break
                while self._health_missing() and not self.cancel.is_set():
                    self.cancel.wait(self.snapshot_period)
                if self.cancel.is_set():
                    break
                self.ui_events.put(UiEvent("point_progress", {
                    "run_id": run_id, "segment_id": point.segment_id,
                    "index": point_index, "total": len(plan.points),
                    "adaptive": point.adaptive,
                    "message": f"开始第{point_index}/{len(plan.points)}点"}))
                transaction, begin_reason = self._try_begin_set(
                    point.upper_percent, point.lower_percent,
                    wait_for_slot=True)
                if transaction is None:
                    self.store.event(
                        "set_not_sent", run_id=run_id,
                        segment_id=point.segment_id, reason=begin_reason)
                    raise RuntimeError(
                        f"TBENCH SET 未取得发送位置：{begin_reason}")
                if transaction.outcome == "send_failed":
                    raise ConnectionError("TBENCH SET 命令未入队")
                if not self._wait_transaction(transaction):
                    raise TimeoutError(
                        f"TBENCH SET 已发送但ACK未匹配：{transaction.outcome}")
                self.store.event(
                    "segment", run_id=run_id, segment_id=point.segment_id,
                    phase=point.phase, direction=point.direction,
                    upper=point.upper_percent, lower=point.lower_percent)
                if self.cancel.wait(point.settle_s):
                    break
                if point.adaptive:
                    stable, probe_voltages = self._await_adaptive_stability(
                        run_id, plan.mode, point)
                    measured_voltages.extend(probe_voltages)
                    if not stable:
                        skipped_points += 1
                        continue
                deadline = self.clock() + point.duration_s
                while self.clock() < deadline and not self.cancel.is_set():
                    sample = self._joined_sample(run_id, plan.mode, point)
                    missing = self._required_missing(sample)
                    if missing:
                        # A missing measurement is not a failed motor command.
                        # Keep the command heartbeat alive and discard this interval.
                        self.cancel.wait(self.snapshot_period)
                        continue
                    if sample.voltage_v is not None:
                        measured_voltages.append(sample.voltage_v)
                    source_times = self._sample_source_times(sample)
                    source_status = self._classify_sample_sources(source_times)
                    if source_status == "duplicate":
                        self.store.event(
                            "duplicate_source_time", run_id=run_id,
                            segment_id=point.segment_id,
                            upper_source_ms=source_times[0],
                            lower_source_ms=source_times[1])
                        self.cancel.wait(self.snapshot_period)
                        continue
                    if source_status == "regression":
                        raise RuntimeError("eRPM 来源时刻倒退")
                    self._last_emitted_source_times = source_times
                    self.store.sample(sample)
                    self.ui_events.put(UiEvent("sample", sample))
                    self.cancel.wait(self.snapshot_period)
            completed = not self.cancel.is_set()
        except Exception as exc:
            self.store.event("run_error", run_id=run_id, error=repr(exc))
            self.ui_events.put(UiEvent("error", str(exc)))
        finally:
            self.stop()
            rates = []
            for values in self._source_times:
                span = (max(values) - min(values)) / 1000 if len(values) > 1 else 0
                rates.append((len(values) - 1) / span if span > 0 else 0)
            try:
                self.store.event(
                    "effective_erpm_source_rate", run_id=run_id,
                    upper_hz=rates[0], lower_hz=rates[1])
                self.store.record_run({
                    "run_id": run_id, "plan_name": plan.name, "mode": plan.mode,
                    "voltage_layer_label": layer_label,
                    "voltage_layer_target_v": layer_v,
                    "voltage_target_is_measured": False,
                    "loaded_voltage_min_v": min(measured_voltages) if measured_voltages else None,
                    "loaded_voltage_max_v": max(measured_voltages) if measured_voltages else None,
                    "loaded_voltage_samples": len(measured_voltages),
                    "skipped_points": skipped_points,
                    "completed": completed,
                    "cancelled": not completed})
            except Exception as exc:
                self.ui_events.put(UiEvent("error", f"run 元数据写入失败：{exc}"))
            finally:
                with self._state_lock:
                    self._running = False
                self.release_operation("plan")
                self.ui_events.put(UiEvent(
                    "run_finished", {"run_id": run_id,
                                     "completed": completed,
                                     "skipped_points": skipped_points,
                                     "total_points": len(plan.points)}))

    def _await_adaptive_stability(self, run_id: str, mode: str,
                                  point: PlanPoint, *,
                                  stop_voltage_v: float | None = None
                                  ) -> tuple[bool, list[float]]:
        started = self.clock()
        deadline = started + max(0.1, point.max_wait_s)
        required_span = max(0.1, point.stable_window_s)
        window: list[tuple[tuple[int, int], BenchSample]] = []
        last_source: tuple[int, int] | None = None
        voltages: list[float] = []
        while self.clock() < deadline and not self.cancel.is_set():
            sample = self._joined_sample(run_id, mode, point)
            if (stop_voltage_v is not None and sample.voltage_v is not None
                    and sample.voltage_v <= stop_voltage_v):
                raise StopVoltageReached("带载电压达到停止阈值")
            missing = self._required_missing(sample)
            if missing:
                window.clear()
                last_source = None
                self.cancel.wait(self.snapshot_period)
                continue
            source_times = self._sample_source_times(sample)
            if last_source is not None:
                progress = self._source_progress(source_times, last_source)
                if progress == "waiting":
                    self.cancel.wait(self.snapshot_period)
                    continue
                if progress == "regression":
                    raise RuntimeError("智能稳态eRPM来源时刻倒退")
            last_source = source_times
            if sample.voltage_v is not None:
                voltages.append(sample.voltage_v)
            settling = replace(
                sample, phase="settle",
                quality=tuple(sample.quality) + ("adaptive_settling",))
            self.store.sample(settling)
            self.ui_events.put(UiEvent("sample", settling))
            window.append((source_times, sample))
            cutoff = sample.host_time_s - required_span
            # Keep the nearest sample immediately before the cutoff so an
            # irregularly sampled window can actually span required_span.
            while (len(window) > 1
                   and window[1][1].host_time_s <= cutoff):
                window.pop(0)
            if self._adaptive_window_stable(
                    window, required_span, point.min_scale_updates):
                # The first valid steady sample must be a newer source update
                # than the evidence already saved during settling.
                self._last_emitted_source_times = source_times
                self.store.event(
                    "adaptive_point_stable", run_id=run_id,
                    segment_id=point.segment_id,
                    samples=len(window), wait_s=self.clock() - started)
                self.ui_events.put(UiEvent(
                    "point_stable", {"run_id": run_id,
                                     "segment_id": point.segment_id,
                                     "samples": len(window),
                                     "wait_s": self.clock() - started,
                                     "message": "转速与推力已稳定"}))
                return True, voltages
            self.cancel.wait(self.snapshot_period)
        reason = "已取消" if self.cancel.is_set() else "等待稳定超时"
        self.store.event(
            "adaptive_point_skipped", run_id=run_id,
            segment_id=point.segment_id, reason=reason,
            samples=len(window), max_wait_s=point.max_wait_s)
        self.ui_events.put(UiEvent(
            "point_skipped", {"run_id": run_id,
                              "segment_id": point.segment_id,
                              "reason": reason,
                              "message": ("已取消当前点" if reason=="已取消"
                                          else "等待稳定超时，已跳过当前点")}))
        return False, voltages

    @staticmethod
    def _adaptive_window_stable(
            window: list[tuple[tuple[int, int], BenchSample]],
            required_span: float, min_scale_updates: int = 3) -> bool:
        if len(window) < 4:
            return False
        samples = [item[1] for item in window]
        if samples[-1].host_time_s - samples[0].host_time_s < required_span:
            return False
        scale_times = {sample.scale_time_s for sample in samples
                       if sample.scale_time_s is not None}
        if len(scale_times) < max(3, int(min_scale_updates)):
            return False
        series = ([float(sample.upper_erpm) for sample in samples],
                  [float(sample.lower_erpm) for sample in samples],
                  [float(sample.thrust_n) for sample in samples])
        for index, values in enumerate(series):
            mean = sum(values) / len(values)
            tolerance = (max(100.0, abs(mean) * 0.02)
                         if index < 2 else max(0.05, abs(mean) * 0.03))
            if max(values) - min(values) > tolerance:
                return False
        return True

    @staticmethod
    def _required_missing(sample: BenchSample) -> list[str]:
        required = {
            "upper_erpm": sample.upper_erpm,
            "lower_erpm": sample.lower_erpm,
            "thrust": sample.thrust_n,
            "voltage": sample.voltage_v,
        }
        return [name for name, value in required.items() if value is None]

    @staticmethod
    def _sample_source_times(sample: BenchSample) -> tuple[int, int]:
        if (sample.fc_time_ms is None or sample.upper_erpm_age_ms is None
                or sample.lower_erpm_age_ms is None):
            raise RuntimeError("eRPM 来源时刻不可用")
        return (sample.fc_time_ms - sample.upper_erpm_age_ms,
                sample.fc_time_ms - sample.lower_erpm_age_ms)

    def _classify_sample_sources(self, source_times: tuple[int, int]) -> str:
        previous = self._last_emitted_source_times
        if previous is None:
            return "new"
        progress = self._source_progress(source_times, previous)
        if progress == "waiting":
            return "duplicate"
        if progress == "regression":
            return "regression"
        return "new"

    @staticmethod
    def _source_progress(current: tuple[int, int],
                         previous: tuple[int, int]) -> str:
        if any(now < old for now, old in zip(current, previous)):
            return "regression"
        if any(now == old for now, old in zip(current, previous)):
            return "waiting"
        return "new"

    def _health_missing(self, now: float | None = None) -> list[str]:
        current = self.clock() if now is None else now
        point = PlanPoint("health", 0.0, 0.0)
        sample = self._joined_sample("health", "dual", point)
        return self._required_missing(sample)

    def _monitor_active_health(self) -> bool:
        missing = self._health_missing()
        if missing and self._stream_established:
            # A reply may already be queued while the FC reader is finishing
            # its decode/log step. Give that owner one short chance to publish
            # a fresh snapshot, then apply the same source-age limits again.
            with self._snapshot_condition:
                self._snapshot_condition.wait(timeout=min(0.010,self.snapshot_period/2))
            missing = self._health_missing()
        if not missing:
            if self._telemetry_gap_started is not None:
                self.store.event("telemetry_gap_recovered",
                                 duration_s=round(self.clock()-self._telemetry_gap_started,3))
                self._telemetry_gap_started = None
            self._stream_established = True
            return True
        if (not self._stream_established
                and self._startup_grace_used
                and self.clock() <= self._startup_grace_until):
            return True
        now=self.clock()
        if self._telemetry_gap_started is None:
            self._telemetry_gap_started=now
            self.store.event("telemetry_gap_started", missing=missing)
        if now-self._telemetry_gap_started < self.TELEMETRY_GAP_GRACE_S:
            return True
        with self._observation_lock:
            latest_fc=self._latest_fc
        snapshot=latest_fc[0] if latest_fc else None
        self.store.event(
            "active_stream_lost", missing=missing,
            gap_duration_s=round(now-self._telemetry_gap_started,3),
            fc_receive_age_ms=(round((now-latest_fc[1])*1000,1)
                               if latest_fc else None),
            fc_erpm_source_age_ms=(list(snapshot.erpm_age_ms) if snapshot else None),
            fc_voltage_source_age_ms=(snapshot.voltage_age_ms if snapshot else None),
            pending_snapshots=len(self._pending_snapshots),
            link_queue_depth=self.connection.events.qsize())
        self.ui_events.put(UiEvent(
            "error", "活动期数据流失效，已停止：" + ",".join(missing)))
        self.stop()
        return False

    def _heartbeat_loop(self) -> None:
        while not self._shutdown.wait(self.HEARTBEAT_S):
            if not self._armed:
                continue
            if not self._monitor_active_health():
                continue
            timed_out = None
            with self._ack:
                pending = self._pending_ack
                if (pending is not None
                        and pending.outcome == "pending"
                        and self._pending_ack is pending
                        and self.clock() - pending.sent_at > self.ACK_TIMEOUT_S):
                    pending.outcome = "timeout"
                    self._pending_ack = None
                    # Cancel the window before waking a planned SET waiter.
                    # stop() will still send the redundant wire STOP below.
                    self.cancel.set()
                    self._armed = False
                    self._ack.notify_all()
                    timed_out = pending
            if timed_out is not None:
                self.store.event(
                    "command_ack", state="set",
                    request_id=timed_out.expected.get("request_id"),
                    result="timeout", source="heartbeat_monitor")
                self.ui_events.put(UiEvent("error", "心跳SET已发送但ACK超时，已停止"))
                self.stop()
                continue
            if pending is not None:
                continue
            transaction, reason = self._try_begin_set(
                None, None, wait_for_slot=False)
            if transaction is None:
                if reason in {"yield_to_plan", "transaction_pending",
                              "rate_limited", "inactive_or_cancelled"}:
                    continue
                self.ui_events.put(UiEvent(
                    "error", f"心跳SET未取得发送位置：{reason}"))
                self.stop()
                continue
            if transaction.outcome == "send_failed":
                self.ui_events.put(UiEvent("error", "心跳发送失败，已停止"))
                self.stop()

    def _fc_loop(self) -> None:
        next_poll = self.clock()
        while not self._shutdown.is_set():
            now = self.clock()
            if self.connection.generation != self._generation:
                self._generation = self.connection.generation
                with self._observation_lock:
                    self._latest_fc = None
                self._pending_snapshots.clear()
                self.cancel.set()
                self._armed = False
                self._fc_epoch = 0
                self._last_fc_raw = None
                self._last_snapshot_request_time = -math.inf
                self._mapping_recorded = False
                self._cancel_transaction("generation_changed")
                self.store.event("generation", generation=self._generation)
            expired = [
                nonce for nonce, (sent, _) in self._pending_snapshots.items()
                if now - sent > self.SNAPSHOT_TIMEOUT_S]
            for nonce in expired:
                self._pending_snapshots.pop(nonce)
                self.store.event("snapshot_timeout", nonce=nonce)
            if (self.connection.is_connected and now >= next_poll
                    and len(self._pending_snapshots) < self.MAX_PENDING):
                from .fc_protocol import snapshot_command
                self._nonce = (self._nonce + 1) & 0xFFFFFFFF
                if self.connection.send_command(snapshot_command(self._nonce)):
                    self._pending_snapshots[self._nonce] = (now, self._generation)
                next_poll = now + self.snapshot_period
            self._drain_events()
            self._shutdown.wait(0.003)

    def _scale_loop(self) -> None:
        while not self._shutdown.is_set():
            now = self.clock()
            if self.load_cell is not None:
                try:
                    raw = self.load_cell.read_raw()
                    grams = self.load_cell.grams_from_raw(raw)
                    with self._observation_lock:
                        self._latest_scale = (grams, now)
                    self.store.raw_scale(host_time_s=now, raw=raw, grams=grams)
                except Exception as exc:
                    with self._observation_lock:
                        self._latest_scale = None
                    self.store.event("scale_error", error=repr(exc))
            self._shutdown.wait(0.05)

    @staticmethod
    def _kv(text: str) -> dict[str, str]:
        return {
            token.split("=", 1)[0]: token.split("=", 1)[1]
            for token in text.split() if "=" in token}

    @staticmethod
    def _ack_matches(values: dict[str, str], expected: dict[str, str]) -> bool:
        for key, wanted in expected.items():
            actual = values.get(key)
            if key in {"upper_pct", "lower_pct"}:
                try:
                    if not math.isclose(float(actual), float(wanted), abs_tol=0.005):
                        return False
                except (TypeError, ValueError):
                    return False
            elif actual != wanted:
                return False
        return True

    def _drain_events(self) -> None:
        from .fc_protocol import decode_snapshot
        while True:
            try:
                event = self.connection.events.get_nowait()
            except queue.Empty:
                return
            if event.generation != self._generation:
                self.store.event(
                    "stale_generation", received=event.generation,
                    current=self._generation)
                continue
            if event.kind == "fc_payload":
                try:
                    snapshot = decode_snapshot(event.data)
                except Exception as exc:
                    self.store.raw_fc(
                        host_time_s=event.host_time_s,
                        generation=event.generation,
                        payload_hex=bytes(event.data).hex(),
                        decode_error=repr(exc))
                    self.store.event("fc_decode_error", error=repr(exc))
                    continue
                pending = self._pending_snapshots.pop(snapshot.nonce, None)
                if (pending is None
                        or event.host_time_s - pending[0] > self.SNAPSHOT_TIMEOUT_S):
                    self.store.raw_fc(
                        host_time_s=event.host_time_s,
                        generation=event.generation,
                        payload_hex=bytes(event.data).hex(),
                        classification="stale_or_duplicate_nonce",
                        **asdict(snapshot))
                    self.store.event("stale_nonce", nonce=snapshot.nonce)
                    continue
                if self._last_fc_raw is not None:
                    classification = self._classify_fc_time(
                        snapshot.fc_time_ms, pending[0])
                    if classification == "wrap":
                        self._fc_epoch += 1 << 32
                    elif classification == "out_of_order":
                        self.store.raw_fc(
                            host_time_s=event.host_time_s,
                            generation=event.generation,
                            payload_hex=bytes(event.data).hex(),
                            classification="out_of_order_old_frame",
                            request_time_s=pending[0], **asdict(snapshot))
                        self.store.event(
                            "out_of_order_snapshot", nonce=snapshot.nonce)
                        continue
                    elif classification == "reset":
                        self.store.raw_fc(
                            host_time_s=event.host_time_s,
                            generation=event.generation,
                            payload_hex=bytes(event.data).hex(),
                            classification="fc_time_reset",
                            request_time_s=pending[0], **asdict(snapshot))
                        self.store.event(
                            "fc_time_reset", previous=self._last_fc_raw,
                            received=snapshot.fc_time_ms)
                        self.cancel.set()
                        self.stop()
                        continue
                self._last_fc_raw = snapshot.fc_time_ms
                self._last_snapshot_request_time = pending[0]
                unfolded_fc = self._fc_epoch + snapshot.fc_time_ms
                with self._observation_lock:
                    self._latest_fc = (
                        snapshot, event.host_time_s, event.generation, unfolded_fc)
                with self._snapshot_condition:
                    self._snapshot_condition.notify_all()
                mapping = (snapshot.upper_channel, snapshot.lower_channel)
                if mapping in ((1, 2), (2, 1)) and mapping != self._recorded_mapping:
                    self.store.update_metadata(
                        upper_channel=snapshot.upper_channel,
                        lower_channel=snapshot.lower_channel,
                        channel_mapping_source="firmware_snapshot")
                    self._mapping_recorded = True
                    self._recorded_mapping = mapping
                for role, index in (
                    (0, snapshot.upper_channel - 1),
                    (1, snapshot.lower_channel - 1),
                ):
                    if (index in (0, 1) and snapshot.erpm[index] is not None
                            and snapshot.erpm_age_ms[index] is not None):
                        self._source_times[role].add(
                            unfolded_fc - int(snapshot.erpm_age_ms[index]))
                self.store.raw_fc(
                    host_time_s=event.host_time_s,
                    generation=event.generation,
                    payload_hex=bytes(event.data).hex(),
                    unfolded_fc_time_ms=unfolded_fc,
                    request_time_s=pending[0], **asdict(snapshot))
            elif event.kind == "fc_text":
                text = str(event.data)
                self.store.event("fc_text", text=text)
                if text.startswith("PROPCAL "):
                    try:
                        self.propcal_events.put_nowait(event)
                    except queue.Full:
                        # Bounded diagnostic backlog; callers must still match
                        # generation, receipt time and the expected reply fields.
                        try: self.propcal_events.get_nowait()
                        except queue.Empty: pass
                        self.propcal_events.put_nowait(event)
                if text.startswith("ESC "):
                    try:
                        self.esc_events.put_nowait(event)
                    except queue.Full:
                        try: self.esc_events.get_nowait()
                        except queue.Empty: pass
                        self.esc_events.put_nowait(event)
                if not text.startswith("TBENCH "):
                    continue
                values = self._kv(text)
                state = values.get("state", "")
                with self._ack:
                    transaction = self._pending_ack
                    if (transaction is not None
                            and event.host_time_s >= transaction.sent_at
                            and event.generation == transaction.generation
                            and state == transaction.state
                            and self._ack_matches(values, transaction.expected)):
                        transaction.outcome = "matched"
                        self._pending_ack = None
                        self._ack.notify_all()
                if state in {"idle", "rejected"}:
                    if state == "rejected":
                        self.last_rejection_reason = values.get("reason", "unknown")
                    self.cancel.set()
                    self._armed = False
                    self._cancel_transaction(state)
                    self.ui_events.put(UiEvent("rejected" if state == "rejected" else "stopped_confirmed", values))
            elif event.kind == "fc_disconnected":
                self.cancel.set()
                self._armed = False
                self._cancel_transaction("disconnected")
                self.ui_events.put(UiEvent("error", "飞控连接已断开"))
            elif event.kind == "fc_frame_error":
                self.store.event(
                    "fc_frame_error", generation=event.generation,
                    received_at=event.host_time_s, **event.data)

    def _classify_fc_time(self, received_raw: int,
                          request_time: float) -> str:
        if request_time < self._last_snapshot_request_time:
            return "out_of_order"
        if self._last_fc_raw is None or received_raw >= self._last_fc_raw:
            return "forward"
        if self._last_fc_raw - received_raw > 0x80000000:
            return "wrap"
        return "reset"

    def _joined_sample(self, run_id: str, mode: str,
                       point: PlanPoint) -> BenchSample:
        now = self.clock()
        quality: list[str] = []
        fc = self._latest_fc[0] if self._latest_fc else None
        receive_s = self._latest_fc[1] if self._latest_fc else None
        unfolded_fc = self._latest_fc[3] if self._latest_fc else None
        held_ms = (now - receive_s) * 1000 if receive_s is not None else math.inf
        if fc is None:
            quality.append("fc_missing")
        elif held_ms > self.SNAPSHOT_TIMEOUT_S * 1000:
            quality.append("fc_stale")
            fc = None
            unfolded_fc = None
        scale = self._latest_scale
        if scale is None:
            quality.append("scale_missing")
        elif now - scale[1] > self.SCALE_FRESH_S:
            quality.append("scale_stale")
            scale = None
        indices = (fc.upper_channel - 1, fc.lower_channel - 1) if fc else (-1, -1)
        if fc and set(indices) != {0, 1}:
            quality.append("channel_mapping_invalid")
            fc = None
            unfolded_fc = None
            indices = (-1, -1)

        def source_value(role: int, values, ages, name: str,
                         max_age_ms: int):
            index = indices[role]
            if fc is None:
                return None
            if values[index] is None or ages[index] is None:
                quality.append(f"{name}_missing")
                return None
            if ages[index] + held_ms > max_age_ms:
                quality.append(f"{name}_stale")
                return None
            return values[index]

        def erpm(role: int) -> float | None:
            value = source_value(role, fc.erpm if fc else (),
                                 fc.erpm_age_ms if fc else (),
                                 "upper_erpm" if role == 0 else "lower_erpm",
                                 self.SOURCE_FRESH_MS)
            return None if value is None else float(value)

        def erpm_age(role: int) -> int | None:
            index = indices[role]
            if fc is None or fc.erpm_age_ms[index] is None:
                return None
            return int(fc.erpm_age_ms[index])

        def esc_current(role: int) -> float | None:
            return source_value(
                role, fc.esc_current_a if fc else (),
                fc.esc_current_age_ms if fc else (),
                "upper_esc_current" if role == 0 else "lower_esc_current",
                self.ESC_CURRENT_FRESH_MS)

        def esc_current_age(role: int) -> int | None:
            index = indices[role]
            if fc is None or fc.esc_current_age_ms[index] is None:
                return None
            return int(fc.esc_current_age_ms[index])

        def command(role: int) -> float | None:
            index = indices[role]
            if fc is None or fc.command_us[index] == 0:
                return None
            return (fc.command_us[index] - 1100) * 100.0 / 840.0

        voltage = None
        board_current = None
        voltage_age = None
        board_current_age = None
        if fc is not None and fc.voltage_v is not None and fc.voltage_age_ms is not None:
            if fc.voltage_age_ms + held_ms <= self.SOURCE_FRESH_MS:
                voltage = fc.voltage_v
                voltage_age = int(fc.voltage_age_ms)
            else:
                quality.append("voltage_stale")
        if fc is not None and fc.total_current_a is not None and fc.current_age_ms is not None:
            if fc.current_age_ms + held_ms <= self.SOURCE_FRESH_MS:
                board_current = fc.total_current_a
                board_current_age = int(fc.current_age_ms)
            else:
                quality.append("board_current_stale")
        return BenchSample(
            run_id=run_id, segment_id=point.segment_id, host_time_s=now,
            fc_time_ms=unfolded_fc, scale_time_s=scale[1] if scale else None,
            mode=mode, direction=point.direction, phase=point.phase,
            upper_command_pct=command(0), lower_command_pct=command(1),
            upper_erpm=erpm(0), lower_erpm=erpm(1),
            thrust_n=scale[0] * 0.00980665 if scale else None,
            voltage_v=voltage,
            upper_esc_current_a=esc_current(0),
            lower_esc_current_a=esc_current(1),
            upper_erpm_age_ms=erpm_age(0), lower_erpm_age_ms=erpm_age(1),
            upper_esc_current_age_ms=esc_current_age(0),
            lower_esc_current_age_ms=esc_current_age(1),
            voltage_age_ms=voltage_age,
            board_current_a=board_current,
            board_current_age_ms=board_current_age,
            board_current_calibrated=bool(fc.current_calibrated) if fc else False,
            voltage_layer_v=point.voltage_layer_v,
            speed_source="dshot_erpm", current_source="dshot",
            esc_current_calibrated=False,
            quality=tuple(quality))
