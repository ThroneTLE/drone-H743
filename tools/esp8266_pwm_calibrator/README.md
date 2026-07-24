# ESP8266 PWM Calibrator

This is a replacement for the lost H743 when calibrating ESCs, motors, and propellers on the RS485 thrust stand. It is intentionally a two-output bench tool, not flight-controller firmware. The normal operator entry point is `tools/pressure_rs485_gui.py`; keep manual PWM, identification, saved CSV review, and dual-prop loss analysis there.

## Wiring

- `GPIO4` / NodeMCU `D2` -> motor 1 ESC signal wire.
- `GPIO5` / NodeMCU `D1` -> motor 2 ESC signal wire.
- ESP8266 `GND` -> both ESC signal grounds.
- Existing SSD1309 128x64 display stays on `D3/GPIO0` as SDA and `D4/GPIO2` as SCL.
- Power the NodeMCU from its USB cable.
- Power the ESC and motor from the normal battery or bench supply.
- Do not use the ESC BEC to power the USB-connected NodeMCU. Leave its 5 V wire insulated unless the setup is deliberately redesigned.

The ESC must accept a 3.3 V logic signal. Check that before testing; if it requires a 5 V signal, add a suitable level shifter.

Important ESP8266 board note: `S3/S2/S1/SC/SO/SK` are flash-bus pads on common ESP8266 modules. Do not use them for PWM on this fixture; a firmware that touched S2/S3 failed to reach the application status prompt on this board.

## Build And Upload

The repository already has PlatformIO with the ESP8266 Arduino platform installed:

```powershell
D:\ANACON\Scripts\pio.exe run -d tools\esp8266_pwm_calibrator
D:\ANACON\Scripts\pio.exe run -d tools\esp8266_pwm_calibrator -t upload --upload-port COMx
```

The default target is `nodemcuv2`; the firmware uses raw ESP8266 GPIO numbers, with motor 1 on GPIO4/D2 and motor 2 on GPIO5/D1. For a different ESP8266 board, change the PlatformIO `board` setting and `kEscPins` in `src/main.cpp` before building.

## Serial Protocol

Use `115200 8N1`, newline-terminated ASCII commands. The H743 PWM range is preserved exactly: `0% = 1100 us`, `100% = 1940 us`, at 50 Hz.

```text
STATUS?
ARM
PULSE 1150 1000
PCT 10 1000
PULSE 2 1150 1000
PCT 0 10 1000
DISARM
```

`ARM` produces only 1100 us on both outputs and requires a three-second ESC settle time before a non-minimum command. `PULSE` and `PCT` require an explicit hold time of 1 to 5000 ms, then the outputs are disconnected automatically. The optional first argument is the motor: `0` means both, `1` means GPIO4/D2, `2` means GPIO5/D1. Without that motor argument, manual commands default to motor 1 for quick smoke tests.

The pressure GUI uses this identification protocol:

```text
ARM
IDENT START 1 0 100 5 2000
IDENT START 2 0 100 5 2000
IDENT START 0 0 100 5 2000
IDENT KEEPALIVE
IDENT STOP
```

During identification, motor `1` drives GPIO4/D2, motor `2` drives GPIO5/D1, and motor `0` drives both outputs with the same pulse. The PC must send `IDENT KEEPALIVE` at least once every 1.5 s. Loss of the serial program, a stop command, or completion disconnects both PWM outputs.

## GUI Workflow

Run:

```powershell
python tools\pressure_rs485_gui.py
```

In the GUI:

- Select the RS485 pressure transmitter port in `Connection`.
- Select `PWM Controller = ESP12E`, then select the ESP12E USB serial port and `115200` baud.
- Click `Open ESP` once before sending ESP commands. Opening the CH340 serial port can reset the ESP12E, so the GUI waits for the ready banner and then keeps that same serial port open.
- Use `ESP12E Manual PWM` to arm, query actual ESP status, or command M1/M2 percentages with sliders. During manual output, the GUI reuses the already-open serial port, polls `STATUS?`, and shows the ESP-reported mode, arm state, settle state, and M1/M2 pulse widths.
- Use `Thrust Identification` for saved sweeps. `Run=AUTO` performs the full sequence in one CSV: single prop M1, single prop M2, then dual prop. After each stage finishes, the GUI waits for the fixture to settle, sends the pressure transmitter `Tare` command, verifies the after-tare reading, and logs the before/after offset. Select `1`, `2`, or `0` only when you deliberately want to repeat one stage.
- Use `History And Loss` to refresh saved `thrust_ident_*.csv` data, view historical maximum thrust, and export `dual_prop_loss_report.csv` when M1/M2/Dual runs are available.

For offline viewing and comparison of saved AUTO files:

```powershell
python tools\thrust_ident_auto_viewer.py
python tools\thrust_ident_auto_viewer.py tools\thrust_ident_auto_20260723_171830.csv tools\thrust_ident_auto_20260723_165915.csv
```

The viewer can import one or more `thrust_ident_auto_*.csv` files, align M1/M2/Dual points, subtract each stage's `0%` baseline by default, show the loss-coefficient table, plot thrust curves, plot dual-prop loss coefficient, and export the aligned table or PNG plot. Clear `0% baseline` in the viewer when you need to inspect raw transmitter offsets.

## OLED Status

The SSD1309 128x64 display uses I2C on `D3/GPIO0` as SDA and `D4/GPIO2` as SCL. The firmware follows the Zhongjingyuan `ZJY242I0400WG01` sample code: software IIC, 8-bit write address `0x78` (`0x3C` as a 7-bit address), command control byte `0x00`, and data control byte `0x40`. The vendor sample only clocks the ACK bit and does not require an ACK readback, so the ESP8266 firmware no longer treats a failed `Wire` scan as a display fault.

The status screen refreshes about twice per second and shows:

- current mode: `DISARMED`, `ARMED_IDLE`, `MANUAL`, or `IDENT`
- motor 1 and motor 2 pulse widths in microseconds
- current identification motor, percent, and sequence number
- ARM settle countdown before non-minimum PWM is accepted
- last high-level event such as `ARM`, `MANUAL`, `IDENT`, `DISARM`, or `ERR`

Serial display checks:

```text
OLED?
OLED TEST
```

`OLED?` reports the manual no-ACK mode and the active pins. `OLED TEST` shows a 3-second block pattern on the display, then returns to the normal status screen.

## Test Flow And Loss Coefficient

Use the same PWM schedule for all three runs:

1. single prop on motor 1: GUI `Motor=1`
2. single prop on motor 2: GUI `Motor=2`
3. dual prop: GUI `Motor=0`

Then calculate the coaxial/dual-prop loss by aligning the same `pct` or `pulse_us` points:

```text
loss_coeff = dual_g / (single1_g + single2_g)
loss_pct = (1 - loss_coeff) * 100
```

The GUI `History And Loss` section also adds RPM-style estimates for KV1300, 12.6 V, and a 9050 prop. The RPM, tip-speed, and pitch-speed fields are engineering estimates only. They are not measured motor speed unless a tachometer or ESC telemetry is added.

## Bench Gates

1. Remove the propellers and verify `STATUS?` reports `mode=disarmed m1_pulse_us=0 m2_pulse_us=0`.
2. Connect the ESC signal ground, power the ESC, then send `ARM`. Confirm the ESC sees low throttle only.
3. After three seconds, command `PULSE 1100 1000`. It must stay stopped and return to `mode=disarmed`.
4. Only after those checks, mount the propeller in a rigid thrust fixture with a guard and run the GUI at 0 to 20 percent first.
