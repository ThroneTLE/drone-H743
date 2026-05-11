# drone-H743 Current Notes

Keep this file short. Record only facts that help future agents avoid repeating hardware or architecture mistakes.

## Current Architecture

- Intended layering: `App Tasks -> Services -> Driver -> BSP -> HAL/MCU`.
- BSP is below Driver in this project.
- BSP should bind board resources: bus handles, CS pins, DMA callback registration, cache maintenance hooks, board locks.
- Driver should implement reusable chip/device protocol logic.
- App should call application-facing services or drivers through stable public boundaries.
- Services are no-task API modules, not App execution bodies.
- Runtime slow work uses `backgroundReqQueue` and `backgroundRespQueue`.
- `backgroundTask` owns low-priority asynchronous work and directly calls synchronous service/flash APIs below it.
- `Param` lives in `Services/Inc/svc_param.h` and `Services/Src/svc_param.c`; it has no task.
- Avoid queue nesting below background work; do not bring back `storageTask`, `App_Storage`, `flashReqQueueHandle`, or `flashTaskHandle`.

## FLASH / GD25Q32

- External flash chip: GD25Q32, expected JEDEC ID `C8 40 16`.
- SPI wiring: `FLASH_CS=PA4`, `SPI1_SCK=PA5`, `SPI1_MISO=PA6`, `SPI1_MOSI=PA7`.
- Current layer names:
  - App service: `App/Inc/app_flash_service.h`, `App/Src/app_flash_service.c`
  - Driver: `Driver/Inc/drv_gd25q32.h`, `Driver/Src/drv_gd25q32.c`
  - BSP bus binding: `BSP/Inc/bsp_flash_bus.h`, `BSP/Src/bsp_flash_bus.c`
- App code should use `APP_FlashService_*`.
- Chip command code should use `DRV_GD25Q32_*`.
- `BSP_FlashBus_*` is only for board binding, locking, DMA callback registration, and cache hooks.
- Old names to avoid: `bsp_flash.*`, `bsp_gd25q32.*`, `drv_flash.*`, `BSP_FLASH_*`, `BSP_GD25Q32_*`.
- Flash DMA and blocking read paths are currently expected to verify with `FLASH VERIFY`.
- Host verification tool: `python tools/flash_diag_test.py --serial COMx --baud 115200 --final-rtos`.

## H743 DMA / Cache

- Do not place DMA buffers in DTCM.
- Align DMA buffers to cache line size where practical.
- Clean cache before DMA reads memory.
- Invalidate cache after DMA writes memory.
- Flash DMA cache maintenance is passed as callbacks in `DRV_GD25Q32_Bus` and supplied by BSP.

## RTOS Diagnostics

- Useful UART commands:
  - `RTOS?`
  - `FLASH?`
  - `FLASH VERIFY 0x000000 16`
  - `FLASH VERIFY 0x000000 32`
  - `FLASH VERIFY 0x001000 4096`
- During dedicated flash tests, high-rate IMU/heartbeat UART output may be disabled to avoid flooding command responses.

## Other Bring-Up Facts

- ICM-42688 IMU on SPI2 previously read all-zero `WHO_AM_I`; treat as separate SPI2/IMU hardware or wiring issue, not proof that SPI1 flash is broken.
- SPL06-001 barometer on SPI4 previously read all-zero ID; check footprint, soldering, wiring, and actual part before changing protocol logic.
- Ai-WB2 UART/TCP work commonly uses 115200 8N1 and TCP port `6666`; avoid assuming the module is in AT mode while transparent transmission is active.

## Validation Commands

```powershell
python -m pytest tests\test_service_param_background_contract.py tests\test_flash_layering.py tests\test_flash_bdd.py -q
cmake --build --preset Debug
python -m py_compile tools\flash_diag_test.py
```
