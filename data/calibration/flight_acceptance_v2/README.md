# V2A Flight Acceptance Data

Store each V2A session below `YYYY-MM-DD/<session>/`. A session may contain
target snapshots, operator-bound servo direction confirmations, the immutable
threshold set, and the strict report produced by `tools/flight_acceptance_v2.py`.

V2A is props-removed evidence only. Reports keep `flight_release=false`; powered
motor rotation and yaw-torque tests do not belong in this directory or mode.
