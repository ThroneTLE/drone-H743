# Closed-loop Attitude Identification

Source: `D:\stm32hal\drone-H743\flightlog_20260724_200832.csv`

## Data and controller reconstruction

- Rows: 12207; sequence coverage: 68.26%; missing records: 5677.
- Complete experiment segments: 10; total tilt saturation: 13.76%.
- Capture model: roll/pitch frame signs `+1 / -1`, attitude-force sign `-1`, legacy rate lever `0.201 m`.
- Force RMSE X/Y/Z: `0.200994 / 0.167842 / 0.036245 N`.
- Tilt output RMSE pitch/roll: `0.0025176 / 0.0019112 rad`.

- Old-to-corrected frame force delta RMS X/Y: `1.8593 / 1.3890 N`; 95th percentile `4.0810 / 2.8749 N`.
- Force delta / angle-P RMS ratio pitch/roll: `3.402 / 3.706`; tilt delta RMS `6.765 / 5.309 deg`.

## PITCH

- `checkFeedback`: 7/7 experiments detected feedback.
- Best fixed delay: `100 ms`; held-out angle/rate fit: `-12.13% / -2.28%`.
- Final grey-box: servo tau `5.00 ms`, effectiveness `+0.6723`, constrained damping `0.0000 1/s`, tether stiffness `0.0000 1/s^2`.
- Previous simplified PID Tuner comparison: Kp force `0.417965 N/rad`, Kd `-0.175677 N*m*s/rad` (not directly transferable).
- No gain pair passed every robust and nonlinear constraint.
- Reason: Nominal candidates failed at least one uncertainty corner (delay, servo tau, inertia, force, or effectiveness).

- Restrained commissioning candidate: internal Kp/Kd `-12.000 / -0.300`, Synex `+12.000 / +0.300`.
- Worst-corner PM/GM `39.12 deg / 8.27 dB`; nominal crossover `3.411 rad/s`; margin requirement passed `0`.
- Nonlinear 10 deg final angle nominal/worst-delay `0.010 / 0.000 deg`; settling time `3.708 / 1.637 s`.

## ROLL

- `checkFeedback`: 7/7 experiments detected feedback.
- Best fixed delay: `40 ms`; held-out angle/rate fit: `-22.10% / -4.39%`.
- Final grey-box: servo tau `200.00 ms`, effectiveness `-1.5000`, constrained damping `26.6451 1/s`, tether stiffness `0.0000 1/s^2`.
- Previous simplified PID Tuner comparison: Kp force `0.399996 N/rad`, Kd `-0.150658 N*m*s/rad` (not directly transferable).
- No gain pair passed every robust and nonlinear constraint.
- Reason: Nominal candidates failed at least one uncertainty corner (delay, servo tau, inertia, force, or effectiveness).

- Restrained commissioning candidate: internal Kp/Kd `-12.000 / -0.250`, Synex `+12.000 / +0.250`.
- Worst-corner PM/GM `43.93 deg / 9.81 dB`; nominal crossover `2.905 rad/s`; margin requirement passed `0`.
- Nonlinear 10 deg final angle nominal/worst-delay `0.024 / 0.000 deg`; settling time `4.234 / 1.859 s`.

## Transfer boundary

This is a simulation candidate, not a flight-approved tune. The log was recorded under a carbon-rod constraint, has no measured servo position, and the capture firmware used a legacy shared `0.201 m` damping lever. Validate polarity with motors disabled, then use a restrained low-thrust test before free flight.
