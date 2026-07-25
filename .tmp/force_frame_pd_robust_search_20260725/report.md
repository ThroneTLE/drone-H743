# Corrected Force-Frame Robust PD Search

Force range: `5.50..17.50 N`; inertia: `0.051 kg*m^2`.
Uncertainty: effectiveness +/-20%, inertia +/-10%, servo tau 10/30 ms, delay 40/60/80/100 ms.
Design gates: PM >= 56 deg, GM >= 12 dB, nominal zeta 0.90..1.05.

## PITCH

### fixed

- Status: `margin_failed`
- Worst PM/GM: `6.25 deg / 1.63 dB`; nominal crossover: `5.761 rad/s`.
- Internal Kp/Kd: `-5.250 / -0.475`; Synex: `+5.250 / +0.475`.

### scheduled

- Status: `passed`
- Worst PM/GM: `56.31 deg / 12.31 dB`; nominal crossover: `2.302 rad/s`.
- Effective stiffness/Kd: `0.800 / -0.200`; Kp schedule: `Kp_internal = stiffness - ctrl_total_force_n`.
- Hover internal/Synex Kp: `-12.609 / +12.609`; Synex Kp range: `4.700..16.700`.
- Rigid-body wn/zeta: `1.138 / 0.981`.

## ROLL

### fixed

- Status: `margin_failed`
- Worst PM/GM: `14.29 deg / 3.42 dB`; nominal crossover: `5.185 rad/s`.
- Internal Kp/Kd: `-5.250 / -0.425`; Synex: `+5.250 / +0.425`.

### scheduled

- Status: `passed`
- Worst PM/GM: `56.11 deg / 12.56 dB`; nominal crossover: `2.243 rad/s`.
- Effective stiffness/Kd: `1.100 / -0.190`; Kp schedule: `Kp_internal = stiffness - ctrl_total_force_n`.
- Hover internal/Synex Kp: `-12.309 / +12.309`; Synex Kp range: `4.400..16.400`.
- Rigid-body wn/zeta: `1.147 / 0.943`.
