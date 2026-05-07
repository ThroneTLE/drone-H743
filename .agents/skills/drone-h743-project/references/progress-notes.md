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

2026-05-06 Ai-WB2 Xiaomi_11FA TCP bring-up and `+SOCKET:97` fix:
- requested target WiFi changed to:
  - SSID: `Xiaomi_11FA`
  - password: `2325972824`
- PC is connected to `Xiaomi_11FA` through wired Ethernet; verified PC LAN IP for the module target:
  - `192.168.31.189`
- current TCP target:
  - PC listens on `0.0.0.0:6666`
  - Ai-WB2 auto transparent TCP client connects to `192.168.31.189:6666`
- manual AT command rules confirmed:
  - serial assistant must append `CRLF`
  - send one AT command at a time; do not paste the whole command block
  - if commands are pasted without `CRLF`, module may receive one concatenated command and report `Unknown cmd:ATE0AT+WMODE...`
- optional environment scan before joining WiFi:
  - `AT+WSCAN`
  - if needed: `AT+WSCANOPT=1`, then `AT+WSCAN`
- working manual configuration sequence:
  - `AT`
  - `ATE0`
  - `AT+WMODE=1,1`
  - `AT+WJAP="Xiaomi_11FA","2325972824"`
  - `AT+WAUTOCONN=1`
  - `AT+SOCKETAUTOTT=4,192.168.31.189,6666`
  - start PC TCP server before reset:
    `python D:\stm32hal\drone-H743\tools\aiwb2_net_tool.py tcp-server --bind 0.0.0.0 --port 6666`
  - `AT+RST`
- observed failure before PC listener was started:
  - module booted normally
  - `+EVENT:WIFI_CONNECT`
  - `+EVENT:WIFI_GOT_IP`
  - after timeout: `+SOCKET:97` then `ERROR`
- conclusion for `+SOCKET:97`:
  - when it appears after `WIFI_GOT_IP`, WiFi association is already OK
  - first check that the PC TCP server is listening on `192.168.31.189:6666`
  - local PC test showed `Test-NetConnection 192.168.31.189 -Port 6666` failed when no listener was running
  - starting `tools/aiwb2_net_tool.py tcp-server --bind 0.0.0.0 --port 6666` resolved it
- success evidence:
  - PC output: `TCP client connected from 192.168.31.203:50221`
- next bidirectional test steps after connection:
  - serial assistant sends normal payload such as `CH340_TO_TCP_001`; PC TCP server should print it
  - PowerShell TCP server window sends normal payload such as `TCP_TO_CH340_001`; serial assistant should receive it
  - once in transparent mode, serial `AT...` commands are forwarded as TCP data; use `+++` with guard time to return to AT mode

2026-05-06 STM32-side Ai-WB2 order/retry state machine:
- goal:
  - do not depend on Ai-WB2 automatic transparent mode blindly
  - make STM32 handle ordering: AT mode detection, WiFi join, TCP auto transparent config, reset, connect wait, and retry
  - recover from `+SOCKET:97` instead of leaving the application logically stuck
- implementation files:
  - `App/Inc/app_aiwb2.h`
  - `App/Src/app_aiwb2.c`
  - `App/Src/app_uart.c`
  - `BSP/Inc/bsp_uart.h`
  - `CMakeLists.txt`
- no CubeMX regeneration was required because the existing `UARTTask` and generated `USART1` are reused
- `BSP_UART_USART1_OUTPUT_ENABLED` was changed back to `1U`; STM32 now actively drives Ai-WB2 RX through USART1 TX
- important hardware implication:
  - do not leave CH340 and STM32 TX connected to Ai-WB2 RX at the same time during normal firmware operation
  - CH340 is for isolated manual module debugging; normal operation should be STM32 USART1 <-> Ai-WB2
- state machine behavior:
  - first sends `AT`
  - if no response, uses guarded `+++` then probes `AT` again
  - sends one AT command at a time:
    - `ATE0`
    - `AT+WMODE=1,1`
    - `AT+WJAP="Xiaomi_11FA","2325972824"`
    - `AT+WAUTOCONN=1`
    - `AT+SOCKETDEL=1`
    - `AT+SOCKETAUTOTT=4,192.168.31.189,6666`
    - `AT+RST`
  - waits for `connect success` after reset before enabling the normal TCP control protocol
  - consumes Ai-WB2 status/error lines before transparent mode so they do not reach `APP_Control_ProcessLine`
  - after transparent mode, only module event/error lines are consumed by the WiFi manager; normal TCP lines go to the existing board control parser
  - on `+SOCKET:97`, `ERROR`, WiFi disconnect, socket event, or connection timeout, enters retry delay and repeats the sequence
- resource note:
  - this is low-rate USART control traffic inside existing `UARTTask`
  - no DMA is used
  - no cache maintenance is needed
  - state and line parsing live in normal static data; no H7 linker/MPU changes were made
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 RX line normalization/debug:
- observed:
  - `rx_lines` and `rx_idle` now increase, proving DMA and idle-line fallback are working
  - control protocol still did not accept `PING`, so the parsed line likely contains prompt/spacing such as `> PING` or another non-exact form
- implementation:
  - `App/Src/app_uart.c` now prints the first few raw received lines as:
    `BOOT uart_line len=... hex=...`
  - before matching protocol commands, received lines are normalized:
    - leading control/space characters removed
    - optional leading `>` prompt removed
    - trailing control/space characters removed
  - normalized lines are used for both `APP_AiWB2_IsControlPayload()` and `APP_Control_ProcessLine()`
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 USART1 RX DMA receive implementation:
- CubeMX generation check:
  - `.ioc`: `USART1_RX` on `DMA1_Stream0`, `DMA_CIRCULAR`, byte/byte alignment, memory increment enabled, priority high
  - `Core/Src/dma.c`: enables DMA1 clock and `DMA1_Stream0_IRQn`
  - `Core/Src/usart.c`: declares `hdma_usart1_rx`, configures `DMA_REQUEST_USART1_RX`, links it with `__HAL_LINKDMA(uartHandle, hdmarx, hdma_usart1_rx)`, enables `USART1_IRQn`
  - `Core/Src/stm32h7xx_it.c`: has `DMA1_Stream0_IRQHandler()` and `USART1_IRQHandler()`
  - `Core/Src/main.c`: calls `MX_DMA_Init()` before `MX_USART1_UART_Init()`
- manual implementation:
  - `STM32H743XX_FLASH.ld`: added `.dma_buffer (NOLOAD)` section in `RAM_D2`
  - `App/Src/app_uart.c`: added 256-byte `app_uart_dma_rx_buffer` in `.dma_buffer`, starts `HAL_UARTEx_ReceiveToIdle_DMA()` in `UARTTask` init, polls the circular DMA write position, and feeds bytes into the existing line parser
  - half-transfer DMA interrupt is disabled for this stream to reduce IRQ noise; circular DMA and task polling handle the byte stream
  - removed the earlier manual USART1 FIFO enable hook so CubeMX's FIFO-disabled UART configuration remains the effective configuration
- H7 resource/cache note:
  - DMA buffer is in D2 SRAM because default `.bss` lives in DTCM, which DMA1 should not be expected to access
  - DCache is not enabled in current startup code, so no cache invalidate is needed yet
  - if DCache is enabled later, this buffer needs MPU non-cacheable attributes or explicit cache maintenance
- build verification:
  - `cmake --build --preset Debug` passed
  - memory report shows `RAM_D2: 256 B`, confirming the DMA buffer placement

2026-05-06 USART1 RX idle-line fallback:
- observed after DMA receive:
  - `rx_bytes` increases when the panel sends `PING` / `STATUS?` / `CONFIG?`
  - `rx_lines` stays `0`
  - example: `PING` increased `rx_bytes` by 6 bytes, so the downlink physically reaches STM32
- conclusion:
  - Ai-WB2 TX -> STM32 PB15 is no longer the primary issue
  - line termination is not being recognized reliably in the current stream, so relying only on `\r`/`\n` leaves commands buffered forever
- fix:
  - `App/Src/app_uart.c` now treats RX idle as a line boundary
  - if bytes are pending and no new byte arrives for `60 ms`, the pending buffer is terminated and routed through the same line parser
  - boot diagnostics now include `rx_idle=...` to show how many lines were accepted through this fallback
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 passive Ai-WB2 policy after hardware EN/RST clarification:
- clarified hardware fact:
  - earlier assumption was that Ai-WB2 EN/RST was not connected to STM32
  - later corrected: Ai-WB2 EN is connected to `PC6` for debugging
  - Ai-WB2 may still reset/connect on its own timeline unless firmware actively toggles PC6
  - STM32 boot logs sent before Ai-WB2 TCP connection are simply lost from the PC panel, so absence of early `BOOT ...` lines does not prove the task did not run
- observed problem:
  - STM32 active AT manager sent `AT` while Ai-WB2 was already in transparent/auto-connect flow
  - repeated `AT` strings appeared in the PC panel
  - manual logs showed `[Busy]Cmd running` and malformed `Unknown cmd:+++AT+SOCKET...`
- final policy change:
  - default `APP_AIWB2_PASSIVE_ONLY` is now `1U`
  - STM32 no longer sends `AT`, `+++`, `AT+SOCKETAUTOTT`, or `AT+RST` by default
  - firmware passively listens for Ai-WB2 events such as `connect success` / `>`
  - if a known upper-computer command arrives (`PING`, `STATUS?`, `CONFIG?`, `SAVE`, `LOAD`, `DEFAULTS`, `SERVO ...`), firmware assumes transparent mode and immediately routes it to `APP_Control_ProcessLine()`
  - disconnect/socket/error events move the manager back to wait state, but passive mode does not attempt active recovery
- implication:
  - module persistent configuration must be prepared manually or by a separate tool
  - normal run order is PC TCP panel starts listening, Ai-WB2 connects using its saved auto-transparent configuration, STM32 passively accepts the transparent channel
- build verification:
  - `cmake --build --preset Debug` passed without warnings

2026-05-06 passive-mode downlink diagnostics:
- observed after passive firmware:
  - panel receives STM32 `BOOT ...` logs, proving STM32 USART1 TX -> Ai-WB2 RX -> TCP -> PC works
  - panel `PING` still gets no STM32 reply
- current leading suspect:
  - TCP downlink/Ai-WB2 TX -> STM32 PB15 USART1_RX is not reaching the parser, or data arrives without line termination / with UART errors
- added diagnostics in `App/Src/app_uart.c`:
  - before control protocol is accepted, firmware periodically prints:
    `BOOT uart_wait_control rx_bytes=... rx_lines=... rx_overflows=... rx_errors=...`
  - when a known control payload is actually parsed, firmware prints:
    `BOOT control_payload`
  - around `APP_Control_Init()`, firmware prints:
    `BOOT control_init_begin`
    `BOOT control_init_done`
- interpretation:
  - if `rx_bytes` stays 0 after clicking `PING`, check Ai-WB2 TX -> STM32 PB15 and GND
  - if `rx_bytes` increases but `rx_lines` stays 0, check line endings / byte stream framing
  - if `BOOT control_payload` appears but `control_init_done` does not, investigate control init / flash probing path
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 Ai-WB2 EN control on PC6:
- hardware clarification:
  - `PC6`, previously generated/labeled as `LED1`, has been wired to the Ai-WB2 module enable pin for debugging
  - the module enable line is pulled high externally, and `PC6` can actively control module on/off
  - because CubeMX currently still initializes `LED1 / PC6` as GPIO output low, firmware must drive it high early after `MX_GPIO_Init()`
- implementation:
  - added `BSP/Inc/bsp_aiwb2_power.h`
  - added `BSP/Src/bsp_aiwb2_power.c`
  - `BSP_Init()` now calls `BSP_AiWB2_PowerInit()` to set `PC6` high by default
  - `BSP_LED_*()` ignores `LED_1 / PC6` so LED logic cannot toggle WiFi EN
  - `LED_1` remains in the legacy LED enum only as a compatibility name; treat it as Ai-WB2 EN in this hardware setup
- control protocol additions:
  - text:
    - `WIFI?`
    - `WIFI EN 0|1`
    - `WIFI ENABLE 0|1`
    - `WIFI RESET [ms]`
    - compatibility: `WIFI_EN?`, `WIFI_EN 0|1`
  - request/response frame IDs:
    - `APP_PROTO_REQ_WIFI = 0x1018`
    - `APP_PROTO_MSG_WIFI_RECORD = 0x221A`
  - `STATUS?`, `CONFIG?`, `MODULES?`, and `CAPS?` now report WiFi enable/status information
- reset behavior:
  - `WIFI RESET [ms]` is non-blocking
  - command pulls `PC6` low, queues a deadline, and `APP_Control_Tick()` restores `PC6` high later
  - default low pulse is `500 ms`, capped at `5000 ms`
- recommended CubeMX cleanup later:
  - rename `PC6` label from `LED1` to `WIFI_EN`
  - set generated output level to High if the project keeps PC6 as a managed output
- build verification:
  - `cmake --build --preset Debug` passed
  - warning remains from existing disabled heartbeat variables in `App/Src/app_control.c`

2026-05-06 USART1 direct protocol debug mode:
- question answered:
  - current firmware does not actively keep probing Ai-WB2 transparent mode when `APP_AIWB2_PASSIVE_ONLY = 1U`
  - `APP_AiWB2_Tick()` returns immediately in passive mode, so it does not periodically send `AT`, `+++`, or provisioning commands
  - before this update, UART control init and TX were still gated by the Ai-WB2 transparent-state flag, and legacy ASCII command routing was disabled
- implementation:
  - `App/Src/app_uart.c` now sets `APP_UART_DIRECT_CONTROL_ENABLED = 1U`
  - `APP_UART_LEGACY_ASCII_CONTROL_ENABLED` is enabled again
  - USART1 receipt of a known text command such as `PING`, `STATUS?`, `CONFIG?`, `WIFI?`, or `SERVO ...` now calls `APP_AiWB2_AssumeTransparent()`, initializes `APP_Control`, and routes the command immediately
  - binary `$X<...` frame requests also initialize and route the control protocol without waiting for an Ai-WB2 `connect success` or `>` prompt
  - UART TX only requires `APP_Control` initialization in direct mode; it no longer waits for a real WiFi transparent-mode event
- important serial-debug note:
  - accepted text commands can be sent over USART1 for bring-up
  - responses still use the current framed protocol (`$X>` with typed function IDs), not plain ASCII lines
  - a temporary serial upper-computer should decode `App/Inc/app_proto.h`; plain serial assistants may display binary-looking data
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 LED debug semantics cleanup:
- previous `APP_LED_Task_Step()` toggled `LED_RED`, `LED_2`, `LED_3`, and `LED_4` together every second, which made link debugging ambiguous
- updated LED meanings:
  - `LED_RED` / PC13: FreeRTOS heartbeat, slow toggle once per second
  - `LED_1`: USART1 TX activity pulse, driven from `BSP_UART_Transmit_USART1()` and auto-cleared by `UARTTask`
  - `LED_2`: Ai-WB2 TCP transparent mode is accepted by the firmware
  - `LED_3` and `LED_4`: kept off by default
- implementation files:
  - `App/Src/app_led.c`
  - `App/Src/app_uart.c`
  - `BSP/Inc/bsp_uart.h`
  - `BSP/Src/bsp_uart.c`
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 Ai-WB2 busy/autoconnect backoff:
- observed manual/module log:
  - repeated `[Busy]Cmd running`
  - later `+EVENT:WIFI_CONNECT` and `+EVENT:WIFI_GOT_IP`
  - malformed `Unknown cmd:+++AT+SOCKET=4,192.168.31.189,6666`
  - several delayed `OK` responses
- interpretation:
  - commands were being sent while Ai-WB2 was still executing a previous command or its persisted auto-connect flow
  - `+++` was concatenated with the next AT command because the guard-time idle window was not preserved
  - the module was not dead; it was busy and command ordering was being violated
- firmware adjustment:
  - `APP_AIWB2_START_DELAY_MS` increased to `8000 ms` so persisted auto-connect can finish before STM32 starts probing
  - `APP_AIWB2_PROBE_TIMEOUT_MS` increased to `8000 ms`
  - `[Busy]Cmd running` now extends the current wait by `6000 ms` instead of causing immediate further commands
  - `+EVENT:WIFI_CONNECT` / `+EVENT:WIFI_GOT_IP` now moves the state machine into boot-connect wait for `25000 ms`
  - `connect success` or `>` now forces transparent mode from any non-transparent state
  - `Unknown cmd:` now backs off into retry delay instead of continuing to send commands
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 already-transparent Ai-WB2 detection:
- observed after the scheduler fix:
  - `BOOT os_start`
  - `BOOT uart_task_init`
  - panel received `AT`
  - panel could send `PING`, `STATUS?`, `CONFIG?`, but no STM32 control reply was produced
- interpretation:
  - Ai-WB2 may already be connected in TCP transparent mode from its previous persistent configuration
  - the STM32 state machine's initial `AT` probe is transparently forwarded to the PC, so the state machine still thinks it is not in transparent mode
  - incoming panel commands such as `PING` prove that the TCP transparent path is already available
- fix:
  - `App/Src/app_aiwb2.c`: added `APP_AiWB2_IsControlPayload()` and `APP_AiWB2_AssumeTransparent()`
  - `App/Src/app_uart.c`: while the WiFi manager has not yet declared transparent mode, an incoming known control line (`PING`, `STATUS?`, `CONFIG?`, `SAVE`, `LOAD`, `DEFAULTS`, `SERVO ...`) now forces transparent mode and is immediately handed to `APP_Control_ProcessLine()`
- build verification:
  - `cmake --build --preset Debug` passed
  - observed memory after build: DTCMRAM `35088 B / 128 KB`, FLASH `102496 B / 2 MB`

2026-05-06 boot log freeze before scheduler fix:
- observed from `drone_tcp_panel.py`:
  - TCP client connected from Ai-WB2
  - STM32 boot logs reached `BOOT freertos_init_done`
  - no later `BOOT os_start`, no `READY`, and no `ACK PING`
- conclusion:
  - PC <-> Ai-WB2 TCP and Ai-WB2 -> PC transparent path are alive
  - issue is not `PING` text format; panel sends `PING\r\n`
  - firmware was stopping before `osKernelStart()`, so `UARTTask` and Ai-WB2 state machine never ran
- fix:
  - `Core/Src/main.c`: `Main_StagePulse()` now returns immediately after the kernel is initialized, avoiding blocking `HAL_Delay()` during the startup path before `osKernelStart()`
  - `App/Src/app_uart.c`: `APP_UART_Task_Init()` now emits `BOOT uart_task_init\r\n` so the panel can verify the scheduler and UART task are running
- build verification:
  - `cmake --build --preset Debug` passed

2026-05-06 USART1 RX DMA restart after first line:
- observed issue:
  - user reported that the first serial entry works, but the second entry no longer appears to reach the DMA IRQ path
  - this is more likely HAL receive state not being re-armed after the first IDLE-driven receive event than a missing DMA IRQ enable
- generated-code confirmation:
  - `Core/Src/dma.c` enables `DMA1_Stream0_IRQn` and `DMA1_Stream1_IRQn`
  - `Core/Src/stm32h7xx_it.c` routes `DMA1_Stream0_IRQHandler()` to `HAL_DMA_IRQHandler(&hdma_usart1_rx)`
  - `Core/Src/usart.c` links USART1 RX to `DMA1_Stream0` with `DMA_REQUEST_USART1_RX`
  - `.ioc` has `NVIC.DMA1_Stream0_IRQn=true` and `NVIC.USART1_IRQn=true`
- implementation fix:
  - `App/Src/app_uart.c` now checks whether USART1 RX DMA still looks active after draining the circular buffer
  - if `huart1.RxState` is not `HAL_UART_STATE_BUSY_RX`, or the DMA stream is no longer enabled, the task restarts `HAL_UARTEx_ReceiveToIdle_DMA()`
  - this keeps the second and later serial entries alive even when the previous IDLE or error path caused HAL to drop receive state
- practical note:
  - short text packets usually wake through USART1 IDLE events, not DMA transfer-complete IRQs
  - with half-transfer IRQ disabled, DMA1_Stream0 may not fire for every short command
  - only the circular buffer reaching the end should produce a real DMA transfer-complete interrupt
- build verification:
  - `cmake --build --preset Debug` passed
