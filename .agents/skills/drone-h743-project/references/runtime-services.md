# Runtime Services And Background Work

Read this reference only when a task touches `Services/*`, `Param`, `backgroundTask`, slow operations, or their request/response boundaries.

## Ownership

- `Services/*` contains synchronous, no-task service APIs. A service owns data rules and synchronous operations; it is not an App task.
- `Services/Inc/svc_param.h` and `Services/Src/svc_param.c` own the RAM parameter mirror, boot-time FLASH load, dirty state, and slow-save request.
- `Param` is currently the only module under `Services/*`.
- `backgroundTask` is a low-priority App task that executes slow runtime work.
- `APP_FlashService_*` remains a synchronous lower-level boundary used by Param, boot-time load, and maintenance diagnostics.

Runtime slow work follows this boundary:

```text
other App
  -> backgroundReqQueue / APP_BackgroundRequest
  -> backgroundTask / APP_Task_Background_Step()
  -> Param API or APP_FlashService_* synchronous call
  -> backgroundRespQueue / APP_BackgroundResponse
```

Avoid queue nesting below `backgroundTask`. Do not restore:

- `storageReqQueue`, `storageRespQueue`, `storageTask`, or `storageTaskHandle`
- `App_Storage`, `APP_Storage`, or `APP_STORAGE_*`
- `flashReqQueueHandle` or `flashTaskHandle`
- `APP_FLASH_Request` or `APP_FLASH_REQ_*`

## Focused Validation

For service/background architecture changes, run the relevant contract test and a firmware build:

```powershell
python -m pytest tests\test_service_param_background_contract.py -q
cmake --build --preset Debug
```
