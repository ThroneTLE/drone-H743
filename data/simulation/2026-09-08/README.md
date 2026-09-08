# R-SIM-1 host-only evidence

This directory is for the 2026-09-08 host-only teaching simulator run. It is not a flight, calibration, or hardware acceptance record.

The implementation and launch instructions are in `tools/sim_xz/README.md`. The UI preview is `tools/sim_xz/assets/r_sim_ui_preview.svg`.

Verified in this worktree:

- Focused simulation contracts: `21 passed`, including the real `DronePanel + TcpTransport + SimulatorDevice` parameter-control and telemetry-decoder route, four Kp readbacks, and the controller dependency fingerprint test.
- Full repository pytest: `1307 passed, 4 skipped, 1 failed`; the remaining failure is an unrelated existing FLU regression that requires the missing historical CSV `data/flight_logs/2026-09-02/rm1_3_block_queue/flightlog_20260902_202552.csv`.
- Debug firmware build: no work after the successful link, with last linked FLASH usage `400388 B` and zero warnings.
- Physical serial open attempts: `0`.
- Flashing/probe tool invocation attempts: `0`.
- Real Tk window screenshot: `r_sim_ui_screenshot.png`.
- The screenshot was captured after running A/B; all five rows are visible with names, units, A solid lines, and B dashed lines.
