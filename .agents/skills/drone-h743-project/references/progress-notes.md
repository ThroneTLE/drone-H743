# Progress Notes

## 2026-05-05 IMU bring-up status

Current repo path:
- `drone-H743`

Module:
- `ICM-42688` on current project `SPI2`

Current project path references:
- driver: `BSP/Inc/bsp_icm42688.h`, `BSP/Src/bsp_icm42688.c`
- board wrapper: `BSP/Inc/bsp_imu.h`, `BSP/Src/bsp_imu.c`
- task integration: `App/Src/app_imu.c`, `App/Src/app_message.c`, `App/Src/app_uart.c`

CubeMX-generated hardware state at time of test:
- `SPI2_SCK = PA9`
- `SPI2_MOSI = PC1`
- `SPI2_MISO = PC2_C`
- `IMU_CS = PB12`
- `USART1_TX = PB14`
- `USART1_RX = PB15`

Observed result:
- UART1 debug path on `PB14/PB15` works
- IMU init debug ended at `st=3 probe=0 id=0x00`
- Meaning at that stage:
  - software reached SPI transaction path
  - HAL transaction returned without transport error
  - `WHO_AM_I` readback stayed `0x00`

Temporary diagnostic attempts that did not solve it:
- disabled `soft_reset_on_init`
- changed SPI read/write transaction shape to single transfer style
- enabled H7 `PC2_C` analog switch close in `main.c`
- added forced read path and UART diagnostics

Current decision:
- revert temporary IMU diagnostics and keep only baseline IMU driver/task framework
- postpone IMU hardware root-cause until after other peripherals are validated and board rework/re-solder is done

Important reminder for next IMU revisit:
- prioritize checking hardware continuity and soldering for `PC2_C` / MISO, `PB12` / CS, power, and ground
- because `id=0x00` is more consistent with no effective MISO data than with a pure protocol mismatch

## 2026-05-05 SPI flash bring-up preparation

Current repo path:
- `drone-H743`

Module:
- `GD25Q32E` on current project `SPI1`

Reference path:
- datasheet: `driver_doc/C2832998_NOR+FLASH_GD25Q32ESIGR_规格书_WJ123231.PDF`

Current project hardware assumptions from schematic and user notes:
- `FLASH_CS = PA4`
- `SPI1_SCK = PA5`
- `SPI1_MISO = PA6`
- `SPI1_MOSI = PA7`

Current CubeMX/generated mismatch found before driver coding:
- `FLASH_CS` is generated as output low at startup in `Core/Src/gpio.c`
- `SPI1` is generated as `4-bit` data size in `Core/Src/spi.c`
- `SPI1` baud is currently about `60 Mbit/s` for initial bring-up
- `SPI1_MOSI` is currently mapped to `PD7`, while schematic shows `PA7`
- `PA7` is currently occupied by `SPI6_MOSI`

Generated-code check after first CubeMX adjustment:
- `SPI1_MOSI` is now correctly mapped to `PA7`
- `PD7` is no longer used as `SPI1_MOSI`
- `FLASH_CS` is still generated as output low at startup
- `SPI1` is still generated as `4-bit` data size
- `SPI1` baud is still about `60 Mbit/s`

Bring-up direction:
- first fix CubeMX pin mapping and SPI1 basic mode
- then verify flash ID read with `RDID (0x9F)` and standard `READ (0x03)`
- keep standard SPI mode with `WP#` and `HOLD#` tied high; do not enable quad mode first

Current implementation after generated-code check passed:
- chip driver added at `BSP/Inc/bsp_gd25q32.h` and `BSP/Src/bsp_gd25q32.c`
- board binding added at `BSP/Inc/bsp_flash.h` and `BSP/Src/bsp_flash.c`
- startup probe report added at `App/Inc/app_flash.h` and `App/Src/app_flash.c`
- startup report is triggered once from `App/Src/app_message.c`

Current startup test behavior on UART1:
- first probe `RDID (0x9F)`
- then read `SR1 (0x05)`
- then read first 16 bytes from address `0x000000` using `READ (0x03)`

Expected UART examples:
- success:
  - `FLASH ok mid=0xC8 type=0x40 cap=0x16 sr1=0x..`
  - `FLASH [0x000000] .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. ..`
- probe failure:
  - `FLASH probe st=... mid=0x.. type=0x.. cap=0x..`

Observed after first flash test:
- Flash also returned `st=3` and all-zero ID, similar to IMU's all-zero `WHO_AM_I`
- This suggests the next check should focus on SPI bus behavior, signal routing, and MISO levels instead of adding more chip logic

Temporary SPI diagnostic added:
- `BSP_GD25Q32_ReadJedecIdTxRx()` reads `RDID (0x9F)` using one `HAL_SPI_TransmitReceive()` transaction
- startup UART now prints:
  - `FLASH dbg cs=... miso=... split(st=... .. .. ..) txrx(st=... .. .. ..)`
- use this to compare split transmit+receive against single full-duplex transaction

Verified flash result:
- UART reported `FLASH ok mid=0xC8 type=0x40 cap=0x16 sr1=0x00`
- UART reported first 16 bytes at `0x000000` as all `0xFF`
- Meaning `SPI1` plus `GD25Q32E` read path is working
- All-`0xFF` at address zero is normal for erased NOR flash
- The earlier IMU all-zero result should now be treated as a separate `SPI2`/IMU hardware, wiring, soldering, or chip-specific bring-up issue rather than proof that all SPI peripherals are broken

## 2026-05-05 SPL06-001 barometer bring-up preparation

Current repo path:
- `drone-H743`

Module:
- `SPL06-001` on current project `SPI4`

Reference path:
- datasheet: `driver_doc/C2684428_压力传感器_SPL06-001_规格书_WJ83860.PDF`

Current project hardware assumptions from schematic and user notes:
- actual soldered part is `SPL06-001`
- `SPI4_SCK = PE2`
- `SPI4_CS = PE4`
- `SPI4_MISO = PE5`
- `SPI4_MOSI = PE6`

Datasheet facts for first probe:
- SPI 4-wire is default
- `CSB` is active low
- SPI maximum clock is `10 MHz`
- SPI mode 0 is suitable: input latched on SCK rising edge, output changes on SCK falling edge
- ID register address is `0x0D`
- expected ID reset value is `0x10`

Generated-code check after CubeMX adjustment:
- `SPI4` is generated as `8-bit`
- `SPI4` uses software NSS
- `SPI4` baud is about `3.75 Mbit/s`
- `PE4` is generated as GPIO output high with label `Press_cs`
- `PE2/PE5/PE6` are generated as `SPI4_SCK/MISO/MOSI`

Next manual-code direction after user approval:
- add SPL06 chip driver under `BSP/*`
- add board binding for `SPI4 + Press_cs`
- add one-shot UART startup probe that reads `ID (0x0D)` and expects `0x10`

Current implementation after user approval:
- chip driver added at `BSP/Inc/bsp_spl06.h` and `BSP/Src/bsp_spl06.c`
- board binding added at `BSP/Inc/bsp_baro.h` and `BSP/Src/bsp_baro.c`
- startup report added at `App/Inc/app_baro.h` and `App/Src/app_baro.c`
- startup report is triggered once from `App/Src/app_message.c`

Current startup test behavior on UART1:
- `FLASH` report remains once at boot
- `BARO` report now probes `SPL06-001` via `SPI4`
- expected output:
  - `BARO ok id=0x10`

Notes:
- actual soldered part is `SPL06-001`, even if schematic text may still mention `BMP280`
- `SPI4` is now validated in CubeMX with `PE2/PE4/PE5/PE6`

Observed after first SPL06 test:
- user reported `baro0x00`
- this means the `SPI4` transaction path ran, but the SPL06 ID value was not read as `0x10`
- key hardware concern: schematic symbol still says `BMP280`; SPL06-001 pinout must be checked against the actual PCB footprint and orientation

Temporary SPL06 diagnostic added:
- startup UART now prints:
  - `BARO dbg cs=... miso=... split(st=... id=0x..) txrx(st=... id=0x..)`
- split read uses transmit command then receive data
- txrx read uses one full-duplex `HAL_SPI_TransmitReceive()` transaction

Observed after deeper SPL06 test:
- user reported:
  - `BARO dbg cs=1 miso=0 split(st=3 id=0x00) txrx(st=3 id=0x00)`
- meaning at that stage:
  - SPI4 startup path and UART report path were alive
  - both split and txrx probe styles still read all-zero ID
  - this is more consistent with board-side wiring, soldering, footprint/pinout mismatch, or device mismatch than with a simple register-access bug

Current decision:
- pause SPL06 bring-up for now
- keep current minimal driver and diagnostic code in repo as a checkpoint
- revisit after more peripherals are validated or after hardware rework

## 2026-05-05 Ai-WB2-12F Wi-Fi module bring-up preparation

Current repo path:
- `drone-H743`

Module:
- `Ai-WB2-12F` Wi-Fi + BLE module based on `BL602`

Reference path:
- datasheet: `driver_doc/A82852E3370C90F4DD91995C889100A2.pdf`

Goal:
- bring the module up in current project
- make it connect to a PC-side Wi-Fi network or hotspot and transmit data

Datasheet facts for first bring-up:
- supports generic `AT` command workflow for quick start
- default UART baud rate is `115200 bps`
- supply range is `2.7V ~ 3.6V`
- recommended external supply capability is at least `500mA`
- module UART pins:
  - pin 21 `RXD`
  - pin 22 `TXD`
- `EN` is active high when exposed and used
- `GPIO8` bootstrap high at power-on enters programming mode; low at power-on is normal boot

Current current-project UART resources already enabled in CubeMX:
- `UART4`: `PD0/PD1`
- `UART5`: `PD2/PB13`
- `UART7`: `PE7/PE8`
- `UART8`: `PE0/PE1`
- `USART1`: `PB14/PB15` currently used for debug output
- `USART2`: `PD5/PD6`

Current decision for first software path:
- do not start with BL602 SDK porting or SPI/SDIO integration
- first use plain UART `AT` probe and response forwarding
- keep `USART1` as debug uplink to the PC
- choose one dedicated UART to the module after confirming real board wiring

Open hardware questions to resolve before coding:
- which MCU UART instance is physically wired to the module `RXD/TXD`
- whether module `EN`, `RST`, or bootstrap-related pins are wired to MCU GPIOs
- whether the module is intended to join an existing PC hotspot/router or expose its own AP

User-selected first test path:
- test the module directly with a `CH340` USB-UART adapter before adding STM32-side code
- keep STM32-side Wi-Fi driver work paused until the AT command sequence is verified manually
- first target is to join a PC-provided 2.4GHz Wi-Fi hotspot and send UDP/TCP test data to the PC

Manual CH340 test status:
- STM32-side `USART1` output was temporarily disabled in current project code so it does not drive the shared UART line while CH340 talks to the module
- user observed `AT` returns `OK`, so the module UART path is alive
- sending multiple commands in one block produced `[Busy]Cmd running` and partial `Unknown cmd` responses
- conclusion: send one AT command at a time, wait for `OK`/`ERROR` before sending the next command
- module firmware identified as:
  - `at version: release/V4.18_P2.19.1`
  - `sdk version: release_bl_iot_sdk_1.6.36`
  - `firmware version: V4.18_P1.4.4-e15d67b`

Automatic module-side configuration completed from the PC:
- CH340 port used: `/dev/cu.usbserial-1110`
- PC hotspot-side IP used for UDP target: `192.168.223.205`
- Wi-Fi auto-connect was set and verified:
  - `AT+WAUTOCONN=1`
  - readback: `+WAUTOCONN:1`
- auto socket transparent transmission was set and verified:
  - `AT+SOCKETAUTOTT=2,192.168.223.205,6666`
  - readback: `+SOCKETAUTOTT:2,192.168.223.205,6666`

Power-on persistence and auto-traffic result:
- after `AT+RST`, module booted and automatically produced:
  - `+EVENT:WIFI_CONNECT`
  - `+EVENT:WIFI_GOT_IP`
  - `connect success ConID=1`
  - prompt `>` indicating transparent mode entered
- PC-side UDP listener received:
  - `boot-auto-udp-test-01\\r\\n`
- therefore current confirmed working path is:
  - module auto-connects to hotspot `misakachiyo3`
  - module auto-connects as UDP client to PC `192.168.223.205:6666`
  - serial payload after boot is forwarded to PC without manual AT steps

Bidirectional UDP transparent test result:
- PC UDP listener received serial payload from module:
  - source: `192.168.223.181:56056`
  - payload: `wb2-to-pc-probe-01\\r\\n`
- PC replied to that exact source address and source port
- CH340 serial side received:
  - `pc-to-wb2-reply-01\\r\\n`
- therefore bidirectional traffic is confirmed in the current auto UDP-client transparent mode

Important PC-side rule:
- because the module is configured as UDP client transparent mode, the PC should reply to the module's observed source port, not to a fixed `7777`
- helper command added:
  - `python3 tools/aiwb2_net_tool.py udp-reply-console --port 6666`

VOFA workflow added:
- helper bridge added:
  - `python3 tools/vofa_udp_bridge.py`
- default bridge behavior:
  - receives module UDP packets on PC port `6666`
  - remembers the module's latest dynamic source endpoint
  - forwards module data to VOFA at `127.0.0.1:6668`
  - receives VOFA data on bridge port `6667`
  - forwards VOFA data back to the remembered module endpoint
- suggested VOFA UDP settings:
  - remote IP: `127.0.0.1`
  - remote port: `6667`
  - local port: `6668`

TCP transparent mode update:
- user confirmed VOFA supports both TCP client and TCP server
- module was reconfigured from UDP auto transparent mode to TCP-client auto transparent mode:
  - `AT+SOCKETAUTOTT=4,192.168.223.205,6666`
  - readback: `+SOCKETAUTOTT:4,192.168.223.205,6666`
- PC-side Python TCP server verified bidirectional traffic:
  - module connected from `192.168.223.181:52674`
  - PC received serial payload `tcp-auto-test-01\\r\\n`
  - PC sent `pc-tcp-reply-01\\r\\n`
  - CH340 serial side received `pc-tcp-reply-01\\r\\n`
- recommended final VOFA path:
  - VOFA runs as TCP server on local port `6666`
  - Ai-WB2 auto-connects as TCP client to `192.168.223.205:6666`
  - once connected, serial payload is bidirectional through the TCP stream

H7 resource note:
- this first AT-command probe is light control-plane traffic only
- no DMA is required yet
- no special cache handling is needed yet
- if a long-running Wi-Fi uplink task is added later, prefer a dedicated communication task rather than folding it into an existing sensor or message task

## Bus servo / Zhongling serial servo

Source references:
- external reference project: `/Users/misakachiyo/STM32VScodeProject/duojitest`
- current target project: `/Users/misakachiyo/STM32VScodeProject/drone-H743`
- local manual: `driver_doc/众灵舵机使用手册-250508.pdf`

Hardware assumption for current board:
- single-wire servo UART signal is on `PE8 / UART7_TX`
- current CubeMX-generated UART7 config already exists:
  - `PE8 = UART7_TX`
  - `PE7 = UART7_RX`
  - `huart7`, 115200, 8N1, no flow control
- first driver pass uses transmit-only commands, so no CubeMX change was needed
- if servo feedback/readback is required on the physical single-wire bus, review whether UART7 needs half-duplex/open-drain or external direction/diode handling before writing RX code

Protocol notes from the manual:
- default baud rate: 115200
- command format example: `#000P1500T1000!`
- multi-servo command can be wrapped as `{#001P0500T1000!#002P2500T1000!}`
- position range used by the manual: `0500` to `2500`
- time field range: `0000` to `9999`
- normal servo IDs cover `0..254`; `255` is broadcast
- default ID may be `0`, so test one servo at a time and change IDs before chaining servos on the same bus

Current implementation:
- added BSP driver:
  - `BSP/Inc/bsp_bus_servo.h`
  - `BSP/Src/bsp_bus_servo.c`
- added build and umbrella include wiring:
  - `CMakeLists.txt`
  - `BSP/Inc/bsp.h`
- temporary demo hook:
  - `App/Src/app_led.c`
  - sends alternating two-servo commands every few LED-task cycles
- build verified with Debug CMake build

Next recommended cleanup:
- for real control logic, add a dedicated `servoTask` in CubeMX instead of keeping servo traffic in the LED task
- use a queue/message buffer from control logic to the servo task
- keep UART7 command formatting in BSP, not in generated `Core/*`

## TCP control panel and two-servo control protocol

Current goal:
- PC-side upper computer connects to the board through Ai-WB2 TCP transparent mode
- upper computer can view hardware init/error status
- upper computer can control two Zhongling serial bus servos
- servo parameters can be saved to GD25Q32 and loaded after power-on

Current default servo mapping:
- servo slot `0` -> physical bus servo ID `1`
- servo slot `1` -> physical bus servo ID `2`
- both default to `pulse=1500`, `time=500`, `mode=1`, `enabled=1`

Board-side line protocol over transparent TCP:
- `PING`
- `STATUS?`
- `CONFIG?`
- `SAVE`
- `LOAD`
- `DEFAULTS`
- `SERVO ID <slot> <id>`
- `SERVO SETID <slot> <new_id>`
- `SERVO MOVE <slot> <pulse_us> <time_ms>`
- `SERVO MOVEALL`
- `SERVO MODE <slot> <mode_1_to_8>`
- `SERVO ENABLE <slot> <0_or_1>`
- `SERVO CMD <slot> <VER|PID|RAD|MOD?|ULK|ULR|DPT|DCT|DST|SCK|CSD|CSM|CSR|SMI|SMX|CLEO|CLE>`
- `SERVO CMD <slot> BD <baud_code>`
- `SERVO RAW <raw_servo_command>`

PC-side tool:
- `tools/drone_tcp_panel.py`
- default listens as TCP server on `0.0.0.0:6666`
- expects Ai-WB2 to connect as TCP client transparent mode

Implementation notes:
- current receive parser is temporarily folded into existing `UARTTask` because `USART1` is already generated as TX/RX
- if control traffic grows, add a dedicated CubeMX `controlTask` and move protocol work out of `UARTTask`
- GD25Q32 persistent config uses the last 4KB sector: `0x3FF000`
- config record has magic, version, size, and checksum

2026-05-05 link-debug update:
- user reported the Python TCP panel sees the Ai-WB2 TCP client connect, but board commands such as `PING`, `STATUS?`, `CONFIG?`, `SAVE`, and `LOAD` do not get responses
- important distinction: TCP connection only proves Ai-WB2 reached the PC; it does not prove the STM32 USART1 transparent path is alive
- firmware now sends:
  - `READY ...` heartbeat every 2 seconds
  - first heartbeat also emits `HW FLASH`, `HW SPL06`, `HW ICM42688`
  - `UART1 rx_bytes=... rx_lines=... rx_overflows=... rx_errors=...` every 2 seconds
  - `ACK <cmd>` immediately after a complete line reaches the STM32 parser
- GUI now shows `USART1 透明链路` health so we can tell whether bytes are arriving at PB15 and whether newline parsing is working
- next physical check if `READY` appears but `rx_bytes` stays 0 after clicking buttons: Ai-WB2 TX must go to STM32 PB15, Ai-WB2 RX must go to STM32 PB14, both at 115200 8N1

2026-05-05 Ai-WB2 TCP loop test with CH340:
- CH340 port: `/dev/cu.usbserial-130`
- PC IP on phone hotspot: `192.168.223.205`
- Ai-WB2 IP observed from TCP connection: `192.168.223.181`
- after Wi-Fi module power was restored and the module was power-cycled, it printed boot banner, `+EVENT:WIFI_CONNECT`, `+EVENT:WIFI_GOT_IP`, then `connect success ConID=1`, `OK`, `>`
- PC TCP server on `0.0.0.0:6666` accepted the module connection
- downlink verified:
  - PC TCP sent `TCP_TO_CH340...` / `DOWNLINK_CHECK...`
  - CH340 serial received those strings from the module
- uplink failed:
  - CH340 serial sent repeated `CH340_TO_TCP...` / `UPLINK_ONLY...`
  - PC TCP received 0 bytes
- conclusion: TCP connection and socket-to-UART direction work; UART-to-socket direction does not currently pass data in the tested state
- important: this test used CH340 connected only to Ai-WB2, not to STM32
