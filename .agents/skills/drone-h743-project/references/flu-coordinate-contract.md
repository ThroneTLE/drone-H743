# FLU Coordinate Contract

Read this reference before changing IMU axes, magnetometer mounting axes, attitude/Fusion adapters, navigation transforms, controller signs, RC direction, optical-flow axes, actuator allocation, servo polarity, or motor/yaw polarity.

## Authority

The sole normative, machine-readable body-frame definition is `Driver/Inc/drv_frame_contract.h`. If comments, documents, tests, captured metadata, or legacy sign macros disagree with that header, treat them as migration evidence—not as another convention.

The canonical body frame is right-handed FLU:

- `+X`: forward.
- `+Y`: left.
- `+Z`: up.
- Positive roll: right wing down.
- Positive pitch: nose down.
- Positive yaw: nose left.
- Canonical angular rates are positive in the same roll/pitch/yaw directions.
- Canonical control-boundary angles use radians and angular rates use radians per second; diagnostic degree values must carry a `_deg`/`_dps` suffix.
- Canonical accelerometer data is specific force; stationary and level is approximately `[0, 0, +1] g`.
- Canonical magnetometer data is the local magnetic field vector in body FLU axes, in milligauss (mGa), the same scale `Driver/Src/drv_mag.c` already produces. It is a polar vector (like specific force), not an axial vector (like angular rate); the proper, determinant `+1` mounting rotation applies identically to both. This header only fixes frame and unit -- hard-/soft-iron calibration and axis-verification gating live in `Services/Inc/svc_mag.h` (and the not-yet-written `Services/svc_mag_cal`), and an uncalibrated or axis-unverified magnetometer must not influence attitude.

Navigation/world frames are separate. Name them explicitly; do not infer NED, ENU, or a local-level frame from the body-frame convention.

## Adapter Rules

- Apply one proper sensor-mount rotation to every sensor vector. A signed permutation used as a physical mounting rotation must have determinant `+1`.
- Keep gravity-vector versus specific-force semantics explicit. A semantic negation is not an axis rotation.
- Isolate third-party estimator conventions at one adapter boundary. FLU to FRD is `diag(+1, -1, -1)` for both polar and axial vectors.
- Controllers consume canonical FLU attitude and body rates. Do not compensate frame errors with negative gains.
- Keep RC intent mapping and actuator/mechanical polarity outside the estimator and controller frame definitions.

## Current Migration Status

`DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK` is `0x3FU`, so
`DRV_FRAME_RUNTIME_MIGRATION_COMPLETE` evaluates to `1` and the migration arm
lock is removed. This is an **Author-directed props-off validation override**
(2026-09-06): the author explicitly required setting the mask before the V2
physical pass in order to arm and manually validate the chain. It is **not
flight release**, not proof that the props-off checks passed, and does not
unfreeze M7. PIPELINE keeps those physical checks open until their observations
are recorded.

Known legacy boundaries include:

- `App/Src/app_sensor.c`: IMU chip axes to the current intermediate axes.
- `App/Src/app_stabilizer.c`: Fusion input signs, startup attitude zero, and controller input assembly. Optical-flow mounting is converted exactly once, at the exit of `stabilizer_compensate_flow_rotation`; nothing downstream may re-adapt it. RC intent no longer lives here — see `App/Src/app_rc_intent.c`.
- `App/Src/app_rc_intent.c`: the only stick-to-FLU Adapter. Each sign is derived from the calibration wizard's physical prompt (`RC_WIZARD_STEPS`) plus this contract, not from bench trial and error.
- `Core/Src/freertos.c`: current sensor-task alignment and legacy NED/FRD comments; this file remains CubeMX-owned.
- `Driver/Src/drv_attitude_fusion.c`: x-io Fusion NED convention.
- `Driver/Src/drv_coax_ctrl.c`: the force-frame and rate-frame sign macros are deleted; the controller consumes canonical FLU directly. Tilt-to-moment polarity and magnitude are derived from measured geometry (since 2026-09-27 the signed lever `cg_z_m - servoN_axis_z_m` filled in `coax_ctrl_apply_fixed_model_params()`, no empirical effectiveness factor; the thrust line passes through the servo tilt axis, so `airframe.thrust_point_to_cg_z_m` is only an arming cross-check), yaw polarity is derived from rotor handedness (`coax_ctrl_yaw_torque_polarity()`, from the ground-station rotor calibration `Driver/Inc/drv_prop_map.h`), the 90-degree servo mount map is fixed kinematics, and servo polarity/travel come from the runtime `ServoCalibration` (`pulse_sign`/`center_us`/`min_us`/`max_us`). No compile-time sign macro may reappear on this path.

Tilt polarity has come from the runtime airframe model (`Driver/Inc/drv_airframe_params.h`) since 2026-09-11; its only source is Flash written by the ground station, there is no compile-time copy left, and an aircraft with no measured model is refused arming (`APP_LED_ARM_BLOCK_AIRFRAME`). Since 2026-09-27 the servo axis heights are part of that gate: zero (unfilled), closer than 1 cm to the CG, or on the opposite side of the CG from the thrust point all refuse arming.

Yaw polarity moved out of that model on 2026-09-13. It had been `airframe.lower_rotor_spin_sense`, a value **back-solved from tuning behaviour** (raising the yaw rate gain produced oscillation rather than divergence, so the closed yaw loop is negative feedback). That inference shows the chain is self-consistent; it does not show which way the rotors turn — a different gain set, or a second sign flipped alongside it, reproduces the same symptom with the aircraft yawing the other way.

The authority is now `Driver/Inc/drv_prop_map.h`: which ESC channel carries the upper rotor, which carries the lower, and each rotor's spin sense seen from above, entered by an operator who spun the motors and watched them (`PROPCAL` command family, ground-station page "桨叶与电机方向"). Polarity is `-lower_rotor_spin_sense`; uncalibrated yields **0**, not a default direction, so the allocator produces no yaw moment and the actuator commit — which routes upper/lower thrust by calibrated role rather than by channel index — finds no channel and disables output. Records written before v22 are **never** migrated from the retired field; doing so would let the inferred value keep deciding yaw direction under the name of a measurement. If the direction turns out reversed, redo that calibration — never the allocator or the RC mapping.

This calibration is deliberately **not** an arming gate (author's decision, 2026-09-13): whether a given session needs it is the author's call, not the firmware's. Do not add one back. Missing calibration means "no yaw authority", which is a different and much safer state than "yaw may be reversed".
- `tools/drone_tcp_panel.py`: artificial-horizon integration and accelerometer-angle display.
- `tools/flight_log_rerun_replay.py`: current X-forward/Y-right/Z-down replay geometry.
- Historical design documents under `doc/history/` may use legacy FRD/local-frame terminology; ask the user which dataset and firmware revision applies before using their values.

`Services/Inc/svc_mag.h` is a new magnetometer mounting adapter, not a legacy
boundary, and it deliberately carries no `DRV_FRAME_RUNTIME_MIGRATION_*` bit.
The mask tracks migration progress of pre-existing legacy runtime seams
(decoupling-spec D1-4); a newly introduced module that is already FLU-compliant
on day one has nothing to migrate, and adding it a bit would only re-arm the
migration lock for everyone else without adding evidence. Do not add one. The
attitude-fusion gate for magnetometer data (calibrated, axis-verified, fresh,
and within a plausible field-strength envelope) is a separate contract from
frame correctness; an axis-correct vector that fails any of those gates is
still refused, and the default state (`SVC_MAG_AXIS_UNVERIFIED`, uncalibrated)
must produce attitude output bit-identical to today's magnetometer-free path.

Telemetry is no longer an adapter boundary. Schema v3 still declares `frame=body_flu` plus the
frame-contract version, but `App/Src/app_telem_port.c` now publishes velocity, position and the
controller setpoint/error channels verbatim: the values reaching it are already canonical FLU, so a
second `DRV_FRAME_FrdToFlu()` there would double-convert. Any reintroduced sign or conversion on the
observation path is a defect, not an exception.

`DRV_FRAME_RUNTIME_MIGRATION_DONE_MASK` tracks sensor, estimator, navigation, controller, RC/actuator, and telemetry/log seams independently. Completion is derived from that mask; do not set a bit without its matching test. Do not claim runtime FLU compliance or authorize free-flight testing while `DRV_FRAME_RUNTIME_MIGRATION_COMPLETE` evaluates to `0`.

Telemetry, captures, calibration records, and flight logs created after migration must carry `DRV_FRAME_CONTRACT_VERSION` plus an explicit frame identifier. Never reinterpret historical NED/FRD data as FLU merely because the current source has migrated.

Before exporting estimator attitude, define quaternion direction (for example body-to-navigation), component order, and both named frames. Test the full quaternion basis change; do not implement migration by flipping only displayed Euler angles.

## Required Validation

Run the executable contract first:

```powershell
python -m pytest tests\test_flu_frame_contract.py -q
```

For changes to the runtime chain, also run the focused sensor/Fusion/controller contracts and add a real pipeline test at the changed seam. Separate estimator-only and controller-only tests are insufficient evidence of physical sign consistency.

When intentionally changing the contract itself, update `drv_frame_contract.h`, this reference, the Skill routing paragraph, and `test_flu_frame_contract.py` in the same task.

## Staged Physical Acceptance

Use these names consistently; do not inflate V0 with later calibration work:

- **V0 frame acceptance:** infer `R_FLU<-legacy_intermediate_v1` from the three positive static bases (level/+Z, nose-up/+X, left-side-up/+Y), require `det=+1`, then verify `+roll/+pitch/+yaw`. The host may apply the result to RAM only after explicit physical-axis confirmation. A second 6-step run must directly match FLU, and accelerometer tilt must agree with Fusion, before `IMUFRAME COMMIT` writes the Param dual-slot Flash record. Reverse static poses are legacy-compatible optional evidence, not normal workflow steps.
- **V1 sensor metrology:** the room-temperature flow is exactly six-face accelerometer calibration plus stationary gyro bias/noise (7 capture stages). Hand-rotation scale checks and multi-plateau temperature drift are retired from V1 for lack of fixtures (rate table, thermal chamber) and live on the frozen sideline until those exist. V0 signed permutations are not substitutes for these continuous calibration parameters.
- **V2 end-to-end actuation:** validate navigation transforms, controller error signs, RC intent, servo/motor polarity, mixer response and failsafe with props removed, then constrained/tethered power tests. Only matching executable and physical evidence may advance the corresponding migration bits.

`IMUFRAME APPLY` is a preview and must reset frame-dependent estimator/navigation state. While the runtime migration mask is incomplete, any active FLU candidate remains arm-locked. `COMMIT` persists only the mapping; it never implies `flight_release=true`.
