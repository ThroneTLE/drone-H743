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
