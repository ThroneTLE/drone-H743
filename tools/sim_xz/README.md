# R-SIM-1 X/Z teaching simulator

This is a host-only teaching tool. It does not provide flight evidence, calibration evidence, or a hardware substitute.

## Start

1. Start the existing ground-station panel as a TCP server on `127.0.0.1:6666`.
2. From the repository root, run:

   ```powershell
   python -m tools.sim_xz
   ```

3. Use `Start / pause` in the simulator window. The simulator is a TCP client and does not open a serial port.
4. In the ground station, request `CAPS?`, `PARAM?`, and `TELEM?`. The simulator answers with the existing `$X` function IDs and CRC.
5. Change one of the four gain channels in the ground station, wait for the parameter echo, click `Save A parameter snapshot`, then click `Run B vs saved A` after the next parameter change.

## Experiments

The defaults are the approved teaching values: position step `0.5 m`, X velocity step `0.2 m/s`, and pitch step `3 deg`. The X velocity experiment keeps Z at `1 m` and bypasses only the horizontal position target by using the existing cascade's velocity feed-forward path. The pitch experiment fixes total thrust to the C bridge's hover value.

The A/B artifact contains a complete parameter snapshot and five comparison channels: `pitch`, `pitch_rate`, `vx`, `x`, and `z`. Artifacts are written below `data/simulation/YYYY-MM-DD/` with `simulation=true` metadata.

## Boundaries

The C bridge is compiled from the real host-pure controller sources. Python owns only the deterministic plant, actuator assumptions, experiment orchestration, TCP adaptation, and Tk snapshots. Drag and thrust time constant are teaching assumptions; tilt time constant, mass, gravity, inertia, and lever arm come from the C-side airframe model. No serial, flash, reset, or target-board command is used.
