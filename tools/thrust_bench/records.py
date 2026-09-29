"""Versioned observations shared by acquisition, replay and offline modelling.

Missing or stale measurements are None. The model uses eRPM directly from a
valid electrical RPM observation without converting to mechanical RPM.
Host monotonic seconds and MCU milliseconds are separate clock domains.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

SCHEMA_VERSION = 2


@dataclass(frozen=True)
class FcSnapshot:
    """Read-only MCU observation. Paired tuples are ESC channels 1 and 2."""

    nonce: int
    fc_time_ms: int
    esc_protocol: int
    upper_channel: int
    lower_channel: int
    command_us: tuple[int, int]
    erpm: tuple[int | None, int | None]
    erpm_age_ms: tuple[int | None, int | None]
    voltage_v: float | None
    total_current_a: float | None
    voltage_age_ms: int | None
    current_age_ms: int | None
    current_calibrated: bool = False
    bench_active: bool = False
    bench_max_percent: int = 0
    armed: bool = False
    esc_current_a: tuple[float | None, float | None] = (None, None)
    esc_current_age_ms: tuple[int | None, int | None] = (None, None)
    not_spinning: tuple[bool, bool] = (False, False)
    flags: int = 0
    last_request_id: int = 0  # MCU-wide last accepted ARM/SET id; survives window close.


@dataclass(frozen=True)
class BenchSample:
    """One joined observation; raw asynchronous source records are also saved.

run_id names an independent sweep/repetition, segment_id one operating point
or step. direction is up/down/steady; phase is settle/steady/dynamic. Roles
come from the confirmed firmware mapping. ESC current and board ADC diagnostics
are separate. esc_current_calibrated is not inferred from the ADC calibration flag. quality contains machine-readable exclusions.
"""

    run_id: str
    segment_id: str
    host_time_s: float
    fc_time_ms: int | None = None
    scale_time_s: float | None = None
    mode: str = "dual"
    direction: str = "steady"
    phase: str = "steady"
    upper_command_pct: float | None = None
    lower_command_pct: float | None = None
    upper_erpm: float | None = None
    lower_erpm: float | None = None
    thrust_n: float | None = None
    voltage_v: float | None = None
    upper_esc_current_a: float | None = None
    lower_esc_current_a: float | None = None
    upper_erpm_age_ms: int | None = None
    lower_erpm_age_ms: int | None = None
    upper_esc_current_age_ms: int | None = None
    lower_esc_current_age_ms: int | None = None
    voltage_age_ms: int | None = None
    board_current_a: float | None = None
    board_current_age_ms: int | None = None
    board_current_calibrated: bool = False
    voltage_layer_v: float | None = None
    speed_source: str = "dshot_erpm"
    current_source: str = "dshot"
    esc_current_calibrated: bool = False
    quality: tuple[str, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {"schema_version": SCHEMA_VERSION, **asdict(self)}

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BenchSample":
        data = dict(value)
        if data.pop("schema_version", None) != SCHEMA_VERSION:
            raise ValueError("unsupported thrust-bench sample schema: v2 electrical eRPM required; v1 mechanical RPM cannot be relabelled")
        data["quality"] = tuple(data.get("quality", ()))
        return cls(**data)
