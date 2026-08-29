# FLASH / GD25Q32 Architecture

Read this reference only when a task touches external FLASH APIs, layering, names, diagnostics, or tests.

## Required Layering

```text
App/Inc/app_flash_service.h
App/Src/app_flash_service.c
  -> Driver/Inc/drv_gd25q32.h
  -> Driver/Src/drv_gd25q32.c
  -> BSP/Inc/bsp_flash_bus.h
  -> BSP/Src/bsp_flash_bus.c
  -> SPI1 / CS / DMA / cache board binding
```

Ownership rules:

- App code calls `APP_FlashService_*`, not BSP FLASH APIs.
- `DRV_GD25Q32_*` implements GD25Q32 chip commands and protocol behavior.
- `BSP_FlashBus_*` owns only board bus binding, locking, DMA callback registration, and cache hooks.
- Flash DMA cache maintenance is supplied by BSP through `DRV_GD25Q32_Bus` callbacks.

Do not restore these legacy names or boundaries:

- `bsp_gd25q32.*`
- `bsp_flash.*`
- `drv_flash.*`
- `BSP_FLASH_*`
- `BSP_GD25Q32_*`

For the current chip, pins, UART commands, and host diagnostic tool, also read [progress-notes.md](progress-notes.md).

## Focused Validation

Use the checks relevant to the change:

```powershell
python -m pytest tests\test_flash_layering.py tests\test_flash_bdd.py -q
cmake --build --preset Debug
python -m py_compile tools\flash_diag_test.py
```

Architecture tests should describe intended dependencies and prevent legacy symbols from returning. Hardware verification should exercise both blocking and DMA paths when the changed behavior affects them.
