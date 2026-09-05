"""Host-side parameter capabilities and the target/draft transaction model.

The firmware remains the authority for parameter names and storage.  This module
only describes the current host-facing vocabulary so the editor can reject an
unknown or read-only field before constructing a command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ParameterCapability:
    """The part of the firmware parameter contract visible to the host."""

    name: str
    unit: str
    writable: bool = True
    minimum: float | None = 0.0
    maximum: float | None = None
    value_kind: str = "float"
    storage: str = "RAM；Flash 持久化需单独确认"


@dataclass
class ParameterState:
    """Separate target, local draft and write confirmation state."""

    name: str
    target: str | None = None
    draft: str | None = None
    pending: str | None = None
    error: str | None = None
    target_source: str = ""
    draft_source: str = ""

    @property
    def dirty(self) -> bool:
        return self.draft is not None and self.draft != self.target

    @property
    def value(self) -> str:
        """Compatibility value: prefer the unconfirmed local draft."""
        return self.draft if self.draft is not None else (self.target or "")

    def as_legacy_view(self, capability: ParameterCapability | None) -> dict[str, str | bool]:
        """Expose old dict keys while keeping the four states explicit."""
        writable = capability.writable if capability is not None else False
        unit = capability.unit if capability is not None else "未知"
        storage = capability.storage if capability is not None else "未知"
        source = self.draft_source if self.draft is not None else self.target_source
        return {
            "value": self.value,
            "source": source,
            "dirty": self.dirty,
            "target": self.target or "",
            "draft": self.draft or "",
            "pending": self.pending or "",
            "error": self.error or "",
            "unit": unit,
            "writable": writable,
            "storage": storage,
        }


# These names are copied from DRV_COAX_CTRL's real parameter table.  The test
# suite compares the set against Driver/Src/drv_coax_ctrl.c; this table is not a
# second firmware source of truth, it is the host's unit/capability annotation.
_CAPABILITY_ROWS = (
    ("pos_x_kp", "gain"),
    ("pos_y_kp", "gain"),
    ("pos_z_kp", "gain"),
    ("pos_xy_vel_max_m_s", "m/s"),
    ("pos_z_vel_up_max_m_s", "m/s"),
    ("pos_z_vel_down_max_m_s", "m/s"),
    ("vel_x_kp", "gain"),
    ("vel_y_kp", "gain"),
    ("vel_z_kp", "gain"),
    ("vel_x_ki", "gain"),
    ("vel_y_ki", "gain"),
    ("vel_z_ki", "gain"),
    ("vel_x_kd", "gain"),
    ("vel_y_kd", "gain"),
    ("vel_z_kd", "gain"),
    ("vel_x_i_limit_m_s2", "m/s²"),
    ("vel_y_i_limit_m_s2", "m/s²"),
    ("vel_z_i_limit_m_s2", "m/s²"),
    ("accel_lpf_cutoff_hz", "Hz"),
    ("accel_xy_max_m_s2", "m/s²"),
    ("accel_z_up_max_m_s2", "m/s²"),
    ("accel_z_down_max_m_s2", "m/s²"),
    ("att_roll_kp", "gain"),
    ("att_pitch_kp", "gain"),
    ("att_yaw_kp", "gain"),
    ("roll_rate_limit_rad_s", "rad/s"),
    ("pitch_rate_limit_rad_s", "rad/s"),
    ("yaw_rate_limit_rad_s", "rad/s"),
    ("rate_roll_kp", "gain"),
    ("rate_pitch_kp", "gain"),
    ("rate_yaw_kp", "gain"),
    ("rate_roll_ki", "gain"),
    ("rate_pitch_ki", "gain"),
    ("rate_yaw_ki", "gain"),
    ("rate_roll_kd", "gain"),
    ("rate_pitch_kd", "gain"),
    ("rate_yaw_kd", "gain"),
    ("rate_roll_i_limit_n_m", "N·m"),
    ("rate_pitch_i_limit_n_m", "N·m"),
    ("rate_yaw_i_limit_n_m", "N·m"),
    ("angular_accel_lpf_cutoff_rad_s", "rad/s"),
    ("rate_roll_ff", "gain"),
    ("rate_pitch_ff", "gain"),
    ("rate_yaw_ff", "gain"),
    ("vel_loop_enable", "bool", "bool"),
    ("tilt_limit_rad", "rad"),
)


PARAMETER_CAPABILITIES: dict[str, ParameterCapability] = {}
for _row in _CAPABILITY_ROWS:
    _name, _unit = _row[:2]
    _kind = _row[2] if len(_row) > 2 else "float"
    PARAMETER_CAPABILITIES[f"coax.{_name}"] = ParameterCapability(
        name=f"coax.{_name}", unit=_unit, value_kind=_kind
    )

# PID SET exposes six derived controller values through DRV_COAX_CTRL_GetParam/
# SetParam.  They are real firmware names, but intentionally do not appear in
# the raw offset table because each is computed from more than one field.
PID_VIRTUAL_CAPABILITIES = {
    name: ParameterCapability(name=name, unit="gain")
    for name in (
        "coax.roll_angle_kp", "coax.pitch_angle_kp", "coax.yaw_angle_kp",
        "coax.roll_rate_kd", "coax.pitch_rate_kd", "coax.yaw_rate_kd",
    )
}


# PID's legacy quick editor uses friendly names, but its wire command only has
# kp/kd.  KI is intentionally absent from this map: it is not a PID SET field.
PID_PARAMETER_ALIASES = {
    "pid.roll.kp": "coax.roll_angle_kp",
    "pid.pitch.kp": "coax.pitch_angle_kp",
    "pid.roll.kd": "coax.roll_rate_kd",
    "pid.pitch.kd": "coax.pitch_rate_kd",
    "pid.yaw.kp": "coax.yaw_angle_kp",
    "pid.yaw.kd": "coax.yaw_rate_kd",
    "pid.pos.x.kp": "coax.pos_x_kp",
    "pid.pos.y.kp": "coax.pos_y_kp",
    "pid.pos.z.kp": "coax.pos_z_kp",
    # This is the old position integrator, not a new velocity I term.
    "pid.pos.z.ki": "coax.pos_z_ki",
    "pid.vel.x.kp": "coax.vel_x_kp",
    "pid.vel.y.kp": "coax.vel_y_kp",
    "pid.vel.z.kp": "coax.vel_z_kp",
    "pid.vel.x.ki": "coax.vel_x_ki",
    "pid.vel.y.ki": "coax.vel_y_ki",
    "pid.vel.z.ki": "coax.vel_z_ki",
    "pid.vel.x.kd": "coax.vel_x_kd",
    "pid.vel.y.kd": "coax.vel_y_kd",
    "pid.vel.z.kd": "coax.vel_z_kd",
    "pid.vel_loop.enable": "coax.vel_loop_enable",
}

PID_QUICK_ALIASES = {
    "pid.roll.kp": "coax.roll_angle_kp",
    "pid.roll.kd": "coax.roll_rate_kd",
    "pid.pitch.kp": "coax.pitch_angle_kp",
    "pid.pitch.kd": "coax.pitch_rate_kd",
    "pid.yaw.kp": "coax.yaw_angle_kp",
    "pid.yaw.kd": "coax.yaw_rate_kd",
}


def canonical_parameter_name(name: str) -> str:
    lowered = name.strip().lower()
    return PID_PARAMETER_ALIASES.get(lowered, lowered)


def parameter_capability(name: str) -> ParameterCapability | None:
    canonical = canonical_parameter_name(name)
    return PARAMETER_CAPABILITIES.get(canonical) or PID_VIRTUAL_CAPABILITIES.get(canonical)


def validate_parameter_text(
    name: str, value: str, *, allow_read_only: bool = False
) -> tuple[bool, str]:
    """Validate a value without changing the firmware's range or clamping rules."""
    capability = parameter_capability(name)
    if capability is None:
        return False, "参数不在当前固件能力表中，请先读取 PARAM?"
    if not capability.writable and not allow_read_only:
        return False, "该字段只读"
    text = value.strip()
    if not text:
        return False, "值不能为空"
    if capability.value_kind == "bool":
        if text not in {"0", "1"}:
            return False, "布尔参数只能为 0 或 1"
        return True, ""
    try:
        parsed = float(text)
    except ValueError:
        return False, "请输入有限数字"
    if not math.isfinite(parsed):
        return False, "不接受 NaN 或无穷大"
    if capability.minimum is not None and parsed < capability.minimum:
        return False, f"不能小于 {capability.minimum:g} {capability.unit}"
    if capability.maximum is not None and parsed > capability.maximum:
        return False, f"不能大于 {capability.maximum:g} {capability.unit}"
    return True, ""


__all__ = [
    "PARAMETER_CAPABILITIES",
    "PID_VIRTUAL_CAPABILITIES",
    "PID_PARAMETER_ALIASES",
    "PID_QUICK_ALIASES",
    "ParameterCapability",
    "ParameterState",
    "canonical_parameter_name",
    "parameter_capability",
    "validate_parameter_text",
]
