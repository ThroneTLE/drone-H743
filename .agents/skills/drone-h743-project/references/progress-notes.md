# drone-H743 Current Bring-Up Notes

Read this reference only when current wiring, diagnostic commands, or known board observations matter. Keep it short and update facts instead of appending a chronological log.

## FLASH / GD25Q32

- External flash: GD25Q32; expected JEDEC ID `C8 40 16`.
- SPI1 wiring: `FLASH_CS=PA4`, `SPI1_SCK=PA5`, `SPI1_MISO=PA6`, `SPI1_MOSI=PA7`.
- Useful UART commands:
  - `RTOS?`
  - `FLASH?`
  - `FLASH VERIFY 0x000000 16`
  - `FLASH VERIFY 0x000000 32`
  - `FLASH VERIFY 0x001000 4096`
  - `FLASH BENCH READ <addr> <len> <loops>`
- Host verification: `python tools/flash_diag_test.py --serial COMx --baud 115200 --final-rtos`.
- During dedicated FLASH tests, high-rate IMU or heartbeat UART output may be disabled to avoid flooding command responses.

## Other Bring-Up Facts

- ICM-42688 on SPI2 previously returned an all-zero `WHO_AM_I`; treat this as a separate SPI2/IMU hardware or wiring issue, not evidence that SPI1 FLASH is broken.
- SPL06-001 on SPI4 previously returned an all-zero ID; check footprint, soldering, wiring, and the actual part before changing protocol logic.
- Ai-WB2 UART/TCP work commonly uses 115200 8N1 and TCP port `6666`; do not assume AT mode while transparent transmission is active.

## IMU Frame V0

- The 2026-08-28 V0 evidence resolved `R_FLU<-legacy_intermediate_v1` to `diag(-1,-1,+1)` / descriptor `-x,-y,+z`; the required positive-basis confidence was about 99.53%. This is historical evidence only: after the current firmware changes the V0 loop must be re-run on target (PIPELINE.md keeps M2 incomplete until then).
- `IMUFRAME APPLY/REVERT/COMMIT` is available over USB CDC, serial and the text link. APPLY is RAM-only; COMMIT uses the `SVC_Param` dual-slot background save.
- A canonical candidate is arm-locked while `DRV_FRAME_RUNTIME_MIGRATION_COMPLETE==0`. V0 persistence is not free-flight approval.

## USB DFU / V1 / V2A

- Application command `BOOT DFU CONFIRM` enters the STM32H743 factory ROM USB DFU after a fresh disarmed/ESC-safe snapshot and USB reply completion. The ROM entry is `0x1FF09800`; future ELF/HEX updates use the ground-station firmware page, but installing this application once still requires ST-Link or physical BOOT0. CubeProgrammer USB DFU must finish with `-s 0x08000000`; `-rst` is JTAG/SWD-only and produces exit 1 after an otherwise successful verify.
- After DFU start, the panel waits up to 15 seconds for the same application CDC and matches VID/PID plus USB serial/location before automatically selecting and reconnecting its COM port; never fall back to the first arbitrary serial device.
- IMUCAP v4 is 48-byte sample / 56-byte header with firmware, frame, orientation, calibration-generation and raw-temperature provenance. Export exposes only timestamp-matched samples whose filtered and control annotations are complete; v3 remains offline read-only.
- `IMUCAL BEGIN/DATA/END/APPLY/REVERT/COMMIT` accepts the bounded 128-byte V1 payload. Room-temperature V1 applies six-face accel calibration plus stationary gyro residual bias; the host applies candidates whose six-face/static-gyro verdict is PASS or WARN (two-tier residual gates), while FAIL/INCOMPLETE and `V0_IMPORTED` evidence are refused. Gyro matrix/temperature bits require reference-fixture and multi-temperature PASS. APPLY is RAM-only and arm-locked; COMMIT preserves the whole FCAL record and uses Param dual-slot persistence.
- `ACCEPT V2` is a props-removed ground mode with a 500 ms lease, mandatory persisted V0 + V1 base bits, ESC CCR=0, passive RC/navigation/controller snapshots, and bounded ±50 us servo steps around the runtime-calibrated centers (`ServoCalibration.center_us`, no fixed 1500 us constant). Lease expiry/disconnect exits safely. V2A remains evidence-only and never sets flight release.

## Ground Calibration Subsystems (2026-08-29 wave)

- RC mapping is runtime-calibrated in `App/Src/app_rc_config.c` (`RCMAP` protocol, Flash record V16→V17); factory defaults reproduce the previous hardcoded CH1..CH6 / 1000-1500-2000 mapping, and controllers read `frame->rc.norm[APP_RC_FUNC_*]`.
- Servo mechanical calibration (`SERVOCAL` preview/revert/commit) stores center/min/max/pulse_sign in the FCAL reserved area (record stays 160 bytes); driver, stabilizer, acceptance, ident and control all consume the single runtime `ServoCalibration`.
- Optical-flow/rangefinder ground calibration lives in `tools/ground_calibration.py` (host-side math only) with an FLU-exported flow compensation snapshot on target; evidence goes to `data/calibration/servo_mechanical/` and `data/calibration/flow_range/`.
- Stationary drift self-check (`tools/stationary_drift.py`) grades 10 Hz telemetry windows (attitude drift, |a|, gyro bias, temperature span) and supports pre/post-APPLY A/B comparison.
