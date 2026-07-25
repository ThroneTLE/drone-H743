# MATLAB PID Toolbox Analysis

Source: `D:\stm32hal\drone-H743\flightlog_20260724_200832.csv`

## Data quality

- Rows: 12207
- Sequence coverage: 68.26%
- Missing records: 5677
- Large experiment gaps: 9
- Tilt saturation: 13.76%

## PITCH

- Usable low-saturation experiments: 7
- Direct tfest status: `diagnostic_only`
- Direct tfest validation fit: 46.13%
- Direct tfest structure: 4 poles, 0 zeros, 0 ms delay
- Conservative analysis crossover: 2.00 rad/s
- Conservative firmware angle Kp magnitude: 0.417965 N/rad
- Conservative firmware rate Kd: -0.175677 N m s/rad
- Worst-case phase margin: 62.84 deg
- Worst-case gain margin: 15.85 dB
- Nominal settling time: 6.701 s
- Nominal overshoot: 11.41%
- Upper-edge crossover: 3.25 rad/s
- Upper-edge Kp/Kd: 1.182599 N/rad, -0.284998 N m s/rad

## ROLL

- Usable low-saturation experiments: 7
- Direct tfest status: `diagnostic_only`
- Direct tfest validation fit: 39.00%
- Direct tfest structure: 2 poles, 0 zeros, 20 ms delay
- Conservative analysis crossover: 1.75 rad/s
- Conservative firmware angle Kp magnitude: 0.399996 N/rad
- Conservative firmware rate Kd: -0.150658 N m s/rad
- Worst-case phase margin: 62.50 deg
- Worst-case gain margin: 14.48 dB
- Nominal settling time: 7.966 s
- Nominal overshoot: 11.14%
- Upper-edge crossover: 2.75 rad/s
- Upper-edge Kp/Kd: 1.036666 N/rad, -0.237105 N m s/rad

## Interpretation

The direct identified models are diagnostic because the input is a closed-loop servo command and the log has no measured servo position. The physical-model PID candidates use the measured inertia, lever arms, thrust range, and the previous grey-box actuator delay/effectiveness estimates. They are simulation candidates, not flight-approved gains.
