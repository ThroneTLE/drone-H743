"""Parameter/PID editor semantics for the ground-station panel.

The page builder owns Tk geometry.  This mixin owns only parameter meaning:
target versus draft, pending confirmation, validation and wire payloads.
"""

from __future__ import annotations

import re
import tkinter as tk

from .proto import (
    PROTO_REQ_PARAM_SET,
    PROTO_REQ_SERVO_ENABLE,
    PROTO_REQ_SERVO_ID,
    PROTO_REQ_SERVO_MODE,
    PROTO_REQ_SERVO_MOVE,
    PROTO_REQ_PID_SET,
    parse_kv,
)
from .parameter_model import (
    PID_QUICK_ALIASES,
    ParameterState,
    canonical_parameter_name,
    parameter_capability,
    validate_parameter_text,
)


_INTEGER_RE = re.compile(r"^[+-]?\d+$")
PID_QUICK_TERMS = ("kp", "kd")
PID_DISPLAY_TERMS = ("kp", "ki", "kd")


class ParameterEditorMixin:
    """State and command boundary for the parameter page."""

    def _init_parameter_editor_state(self) -> None:
        self.params: dict[str, dict[str, str | bool]] = {}
        self.param_states: dict[str, ParameterState] = {}
        self.param_iids: dict[str, str] = {}
        self.param_names_by_iid: dict[str, str] = {}
        self.param_editor_status_var = tk.StringVar(value="目标值来自飞控；草稿未发送")
        self.param_value_error_var = tk.StringVar(value="")
        self.pid_widgets: dict[str, dict[str, tk.Widget]] = {}
        self.pid_ki_status_var = tk.StringVar(value="KI：当前固件 PID SET 不支持，保持只读")
        self._last_param_stage_ok = False
        self._last_param_edit_name = ""

    def _parameter_state(self, name: str) -> ParameterState:
        state = self.param_states.get(name)
        if state is None:
            state = ParameterState(name=name)
            self.param_states[name] = state
        return state

    def _parameter_status_text(self, state: ParameterState) -> str:
        capability = parameter_capability(state.name)
        if capability is not None and not capability.writable:
            return "只读"
        if state.error:
            return f"失败：{state.error}"
        if state.pending is not None:
            return f"发送中（待回读 {state.pending}）"
        if state.dirty:
            return "本地草稿"
        if state.target is not None:
            return "目标已回读"
        return "未读取"

    def _render_param_state(self, state: ParameterState) -> None:
        self.params[state.name] = state.as_legacy_view(parameter_capability(state.name))
        tree = getattr(self, "param_tree", None)
        if tree is None:
            return
        iid = self.param_iids.get(state.name)
        if iid is None:
            iid = f"p{len(self.param_iids)}"
            self.param_iids[state.name] = iid
            self.param_names_by_iid[iid] = state.name
        capability = parameter_capability(state.name)
        source = state.draft_source if state.draft is not None else state.target_source
        source_text = f"{source or '-'} / {capability.unit if capability else '未知单位'}"
        columns = tuple(tree["columns"])
        if len(columns) >= 4:
            row = (
                state.target or "-",
                state.draft or "-",
                source_text,
                self._parameter_status_text(state),
            )
        else:
            row = (state.value, source_text, "yes" if state.dirty else "")
        if tree.exists(iid):
            tree.item(iid, text=state.name, values=row)
        else:
            tree.insert("", tk.END, iid=iid, text=state.name, values=row)

    def _parameter_set_status(self, text: str, *, error: bool = False) -> None:
        self.param_editor_status_var.set(text)
        self.param_value_error_var.set(text if error else "")

    def _parameter_mark_error(self, name: str, message: str, *, focus: bool = False) -> None:
        state = self._parameter_state(name)
        state.pending = None
        state.error = message
        self._render_param_state(state)
        self._parameter_set_status(f"{name}：{message}", error=True)
        if focus:
            entry = getattr(self, "param_value_entry", None)
            if entry is not None:
                entry.focus_set()
                try:
                    entry.selection_range(0, tk.END)
                except tk.TclError:
                    pass

    def _set_param(self, name: str, value: str, source: str, dirty: bool) -> None:
        name = name.strip()
        if not name:
            return
        state = self._parameter_state(name)
        value = str(value)
        if dirty:
            state.draft = value
            state.draft_source = source
            state.error = None
            self._last_param_edit_name = name
        else:
            state.target = value
            state.target_source = source
            if state.pending == value:
                state.pending = None
                state.error = None
                if state.draft == value:
                    state.draft = None
                    state.draft_source = ""
            elif state.draft == value:
                state.draft = None
                state.draft_source = ""
            state.error = state.error if state.pending is not None else None
        self._render_param_state(state)
        self._sync_pid_quick_var(name, value)

    def _mark_param_pending(self, name: str, value: str) -> None:
        state = self._parameter_state(name)
        state.pending = value
        state.error = None
        self._render_param_state(state)
        self._parameter_set_status(f"{name} 已排队，等待目标回读；尚未确认应用")

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
            try:
                index = int(values.get("index", "-1"))
            except ValueError:
                index = -1
            if group in {"rate", "angle"} and 0 <= index < 3:
                axis = ("roll", "pitch", "yaw")[index]

        if axis in getattr(self, "pid_vars", {}):
            for term in PID_DISPLAY_TERMS:
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
        lowered = name.lower()
        alias = lowered if lowered in PID_QUICK_ALIASES else None
        if alias is None:
            alias = next((item for item, actual in PID_QUICK_ALIASES.items() if actual == lowered), None)
        if alias is None:
            return
        parts = alias.split(".")
        if len(parts) == 3 and parts[1] in getattr(self, "pid_vars", {}) and parts[2] in self.pid_vars[parts[1]]:
            self.pid_vars[parts[1]][parts[2]].set(value)

    def _on_param_select(self, _event: tk.Event) -> None:
        selection = self.param_tree.selection()
        if not selection:
            return
        name = self.param_names_by_iid.get(selection[0], selection[0])
        state = self._parameter_state(name)
        self.param_name_var.set(name)
        self.param_value_var.set(state.value)
        capability = parameter_capability(name)
        unit = capability.unit if capability is not None else "未知单位"
        target = state.target or "未读取"
        draft = state.draft or "无"
        self._parameter_set_status(
            f"目标={target} {unit}；草稿={draft}；{self._parameter_status_text(state)}"
        )

    @staticmethod
    def _is_servo_editor_name(name: str) -> bool:
        parts = name.lower().split(".")
        return len(parts) == 2 and parts[0].startswith("servo") and parts[0][5:].isdigit()

    @staticmethod
    def _validate_integer(value: str, minimum: int, maximum: int) -> tuple[bool, str]:
        text = value.strip()
        if not text:
            return False, "值不能为空"
        if not _INTEGER_RE.fullmatch(text):
            return False, "请输入整数"
        try:
            parsed = int(text)
        except ValueError:
            return False, "数值过大"
        if parsed < minimum or parsed > maximum:
            return False, f"范围应为 {minimum}..{maximum}"
        return True, ""

    def _validate_servo_editor_value(self, name: str, value: str) -> tuple[bool, str]:
        field = name.lower().split(".")[-1]
        bounds = {
            "id": (0, 255),
            "new_id": (0, 255),
            "enabled": (0, 1),
            "en": (0, 1),
            "mode": (1, 8),
            "baud": (0, 7),
            "pulse": (500, 2500),
            "pulse_us": (500, 2500),
            "time": (0, 9999),
            "time_ms": (0, 9999),
        }
        if field not in bounds:
            return False, "该舵机字段不可写"
        return self._validate_integer(value, *bounds[field])

    def _stage_param_edit(self) -> bool:
        self._last_param_stage_ok = False
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        if not name:
            self._parameter_set_status("参数名不能为空", error=True)
            entry = getattr(self, "param_name_entry", None)
            if entry is not None:
                entry.focus_set()
            return False

        canonical = canonical_parameter_name(name)
        if self._is_servo_editor_name(canonical):
            valid, reason = self._validate_servo_editor_value(canonical, value)
        else:
            valid, reason = validate_parameter_text(canonical, value)
        if not valid:
            self._parameter_mark_error(canonical, reason, focus=True)
            return False

        self.param_name_var.set(canonical)
        self._set_param(canonical, value, "local", dirty=True)
        self._last_param_stage_ok = True
        self._parameter_set_status(f"{canonical} 已暂存本地草稿，尚未发送")
        return True

    def _discard_param_draft(self) -> None:
        name = self.param_name_var.get().strip()
        if not name:
            self._parameter_set_status("请选择一个参数后再撤销草稿", error=True)
            return
        state = self._parameter_state(canonical_parameter_name(name))
        state.draft = None
        state.draft_source = ""
        state.error = None
        self._render_param_state(state)
        self.param_value_var.set(state.target or "")
        self._parameter_set_status(f"{state.name} 草稿已撤销；目标值未改变")

    def _param_value_for_servo(self, index: int, param_key: str, widget_key: str, fallback: str) -> str:
        state = self.param_states.get(f"servo{index}.{param_key}")
        if state is not None and state.value:
            return state.value
        if 0 <= index < len(self.servo_widgets):
            return str(self.servo_widgets[index][widget_key].get()).strip()
        return fallback

    def _payload_for_param_edit(self, name: str, value: str) -> tuple[int, str]:
        lowered = canonical_parameter_name(name)
        parts = lowered.split(".")
        if self._is_servo_editor_name(lowered):
            index = int(parts[0][5:])
            field = parts[1]
            valid, reason = self._validate_servo_editor_value(lowered, value)
            if not valid:
                raise ValueError(reason)
            if field == "id":
                return PROTO_REQ_SERVO_ID, f"SERVO ID {index} {value}"
            if field in {"enabled", "en"}:
                return PROTO_REQ_SERVO_ENABLE, f"SERVO ENABLE {index} {value}"
            if field == "mode":
                return PROTO_REQ_SERVO_MODE, f"SERVO MODE {index} {value}"
            if field in {"pulse", "pulse_us"}:
                duration = self._param_value_for_servo(index, "time", "time", "500")
                return PROTO_REQ_SERVO_MOVE, f"SERVO MOVE {index} {value} {duration}"
            if field in {"time", "time_ms"}:
                pulse = self._param_value_for_servo(index, "pulse", "pulse", "1500")
                return PROTO_REQ_SERVO_MOVE, f"SERVO MOVE {index} {pulse} {value}"
            raise ValueError("该舵机字段不可写")

        capability = parameter_capability(lowered)
        valid, reason = validate_parameter_text(lowered, value)
        if capability is None or not valid:
            raise ValueError(reason or "参数不可写")
        return PROTO_REQ_PARAM_SET, f"PARAM SET {capability.name} {value}"

    def _send_param_edit(self) -> None:
        if not self._stage_param_edit():
            return
        name = self.param_name_var.get().strip()
        value = self.param_value_var.get().strip()
        try:
            function, payload = self._payload_for_param_edit(name, value)
        except ValueError as exc:
            self._parameter_mark_error(name, str(exc), focus=True)
            return
        if not self._transport_connected():
            self._parameter_mark_error(name, "连接不可用，草稿保留且未发送")
            return
        self._mark_param_pending(name, value)
        self._send_proto_once(function, payload, payload)

    def _send_pid_values(self) -> None:
        for axis, terms in self.pid_vars.items():
            parts: list[str] = []
            for term in PID_QUICK_TERMS:
                value = terms[term].get().strip()
                if not value:
                    continue
                alias = f"pid.{axis}.{term}"
                valid, reason = validate_parameter_text(alias, value)
                if not valid:
                    self._parameter_mark_error(alias, reason, focus=True)
                    continue
                parts.append(f"{term}={value}")
                self._set_param(alias, value, "local", dirty=True)
            ki_value = terms["ki"].get().strip()
            if ki_value:
                self._parameter_mark_error(
                    f"pid.{axis}.ki",
                    "固件 PID SET 只接受 kp/kd；KI 当前不可写",
                    focus=not parts,
                )
            if not parts:
                continue
            payload = f"PID SET {axis} {' '.join(parts)}"
            for term in PID_QUICK_TERMS:
                value = terms[term].get().strip()
                if value and validate_parameter_text(f"pid.{axis}.{term}", value)[0]:
                    self._mark_param_pending(f"pid.{axis}.{term}", value)
            self._send_proto(PROTO_REQ_PID_SET, payload, payload)

    def _parameter_handle_result_line(self, line: str) -> None:
        """Consume existing OK/ERR text without inventing request IDs."""
        if not (line.startswith("ERR ") or line.startswith("OK ")):
            return
        tokens = line.split()
        values = parse_kv(line)
        if line.startswith("ERR param target") and len(tokens) >= 4:
            name = tokens[3]
            self._parameter_mark_error(name, "飞控拒绝目标值")
            # 同一条 ERR 也要送到 Dashboard 的滑块：参数页和 Dashboard 是两套
            # 控件，只在参数页标红等于让 Dashboard 继续显示一个飞控没接受的值。
            notify = getattr(self, "_dashboard_note_param_error", None)
            if callable(notify):
                notify(name, "飞控拒绝了这个值")
            return
        if line.startswith("ERR usage PARAM SET"):
            name = self._last_param_edit_name or self.param_name_var.get().strip()
            if name:
                self._parameter_mark_error(name, "PARAM SET 参数格式被飞控拒绝")
            return
        if line.startswith("ERR pid "):
            axis = values.get("axis") or (tokens[3] if len(tokens) > 3 else "")
            if axis in getattr(self, "pid_vars", {}):
                for term in PID_QUICK_TERMS:
                    self._parameter_mark_error(
                        f"pid.{axis}.{term}", "PID SET 被飞控拒绝"
                    )

    def _parameter_on_disconnect(self) -> None:
        for state in self.param_states.values():
            if state.pending is not None:
                state.error = "连接已断开，目标未确认"
                state.pending = None
                self._render_param_state(state)


__all__ = ["PID_DISPLAY_TERMS", "PID_QUICK_TERMS", "ParameterEditorMixin"]
