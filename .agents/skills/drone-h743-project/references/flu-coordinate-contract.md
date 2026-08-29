# FLU Coordinate Contract

Read this reference before changing IMU axes, attitude/Fusion adapters, navigation transforms, controller signs, RC direction, optical-flow axes, actuator allocation, servo polarity, or motor/yaw polarity.

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

Navigation/world frames are separate. Name them explicitly; do not infer NED, ENU, or a local-level frame from the body-frame convention.

## Adapter Rules

- Apply one proper sensor-mount rotation to every sensor vector. A signed permutation used as a physical mounting rotation must have determinant `+1`.
- Keep gravity-vector versus specific-force semantics explicit. A semantic negation is not an axis rotation.
- Isolate third-party estimator conventions at one adapter boundary. FLU to FRD is `diag(+1, -1, -1)` for both polar and axial vectors.
- Controllers consume canonical FLU attitude and body rates. Do not compensate frame errors with negative gains.
- Keep RC intent mapping and actuator/mechanical polarity outside the estimator and controller frame definitions.

## Current Migration Status

Runtime migration is incomplete while `DRV_FRAME_RUNTIME_MIGRATION_COMPLETE` is `0`.

Known legacy boundaries include:

- `App/Src/app_sensor.c`: IMU chip axes to the current intermediate axes.
- `App/Src/app_stabilizer.c`: Fusion input signs, startup attitude zero, RC intent, and controller input assembly.
- `Core/Src/freertos.c`: current sensor-task alignment and legacy NED/FRD comments; this file remains CubeMX-owned.
- `Driver/Src/drv_attitude_fusion.c`: x-io Fusion NED convention.
- `Driver/Src/drv_imu_nav.c`: current local-level acceleration basis and Z-down assumptions.
- `Driver/Src/drv_coax_ctrl.c`: force-frame, rate-frame, 90-degree servo mount, and actuator polarity adapters.
- `tools/drone_tcp_panel.py`: artificial-horizon integration and accelerometer-angle display.
- `tools/flight_log_rerun_replay.py`: current X-forward/Y-right/Z-down replay geometry.
- `doc/controller_walkthrough.md`: as-implemented descriptions that may still use legacy FRD/local-frame terminology.

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
- **V1 sensor metrology:** estimate repeatable gyro/accelerometer bias, scale, non-orthogonality, temperature drift, noise and powered vibration. V0 signed permutations are not substitutes for these continuous calibration parameters.
- **V2 end-to-end actuation:** validate navigation transforms, controller error signs, RC intent, servo/motor polarity, mixer response and failsafe with props removed, then constrained/tethered power tests. Only matching executable and physical evidence may advance the corresponding migration bits.

`IMUFRAME APPLY` is a preview and must reset frame-dependent estimator/navigation state. While the runtime migration mask is incomplete, any active FLU candidate remains arm-locked. `COMMIT` persists only the mapping; it never implies `flight_release=true`.
