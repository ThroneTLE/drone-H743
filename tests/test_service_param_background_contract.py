from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_services_layer_contains_only_param_service() -> None:
    cmake = read("CMakeLists.txt")
    param_h = read("Services/Inc/svc_param.h")
    param_c = read("Services/Src/svc_param.c")

    assert "Services/Inc" in cmake
    assert "Services/Src/svc_param.c" in cmake
    assert "Services/Src/svc_background.c" not in cmake
    assert not (ROOT / "Services/Inc/svc_background.h").exists()
    assert not (ROOT / "Services/Src/svc_background.c").exists()

    for text in (param_h, param_c):
        assert "功能" in text
        assert "参数" in text

    assert "SVC_Param_Init" in param_h
    assert "SVC_Param_RequestSaveBlob" in param_h
    assert "SVC_Param_IsReady" in param_h
    assert "APP_Storage" not in param_h
    assert "APP_Storage" not in param_c


def test_background_task_and_queues_are_named_as_background_not_storage() -> None:
    freertos = read("Core/Src/freertos.c")
    ioc = read("drone-H743.ioc")
    tasks = read("App/Inc/app_tasks.h")
    task_impl = read("App/Src/app_tasks.c")

    assert "backgroundReqQueueHandle" in freertos
    assert "backgroundRespQueueHandle" in freertos
    assert "sizeof(APP_BackgroundRequest)" in freertos
    assert "sizeof(APP_BackgroundResponse)" in freertos
    assert "backgroundReqQueue,8,APP_BackgroundRequest" in ioc
    assert "backgroundRespQueue,8,APP_BackgroundResponse" in ioc

    forbidden = (
        "storageReqQueue",
        "storageRespQueue",
        "storageTaskHandle",
        "APP_STORAGE_",
        "APP_FLASH_Request",
        "flashReqQueueHandle",
        "flashTaskHandle",
    )
    for token in forbidden:
        assert token not in freertos
        assert token not in ioc
        assert token not in tasks

    assert "extern osMessageQueueId_t backgroundReqQueueHandle;" in tasks
    assert "extern osMessageQueueId_t backgroundRespQueueHandle;" in tasks
    assert "extern osThreadId_t backgroundTaskHandle;" in tasks
    assert "APP_Task_Background_Init" in task_impl
    assert "APP_Task_Background_Step" in task_impl


def test_freertos_objects_are_synchronized_with_ioc() -> None:
    freertos = read("Core/Src/freertos.c")
    ioc = read("drone-H743.ioc")
    tasks = read("App/Inc/app_tasks.h")
    flash_bus = read("BSP/Src/bsp_flash_bus.c")
    rtos_objects = read("Core/Inc/rtos_objects.h")

    assert "RecursiveMutexes01=flashBusMutex,Dynamic,NULL" in ioc
    assert "configUSE_MUTEXES=1" in ioc
    assert "configUSE_RECURSIVE_MUTEXES=1" in ioc
    assert "osMutexId_t flashBusMutexHandle;" in freertos
    assert ".attr_bits = osMutexRecursive" in freertos
    assert "flashBusMutexHandle = osMutexNew(&flashBusMutex_attributes);" in freertos
    assert '#include "rtos_objects.h"' in tasks
    assert "extern osMutexId_t flashBusMutexHandle;" in rtos_objects

    assert "osMutexAcquire(flashBusMutexHandle" in flash_bus
    assert "osMutexRelease(flashBusMutexHandle" in flash_bus
    assert '#include "app_tasks.h"' not in flash_bus
    assert "xSemaphoreCreateRecursiveMutex" not in flash_bus
    assert "StaticSemaphore_t" not in flash_bus


def test_background_task_comment_documents_low_priority_purpose() -> None:
    freertos = read("Core/Src/freertos.c")

    assert "后台低优先级任务" in freertos
    assert "参数" in freertos
    assert "FLASH" in freertos
    assert "APP_Task_Background_Init();" in freertos
    assert "APP_Task_Background_Step();" in freertos


def test_background_app_calls_param_service_for_slow_save_only() -> None:
    param = read("Services/Src/svc_param.c")
    background = read("App/Src/app_background.c")
    background_h = read("App/Inc/app_background.h")

    assert "APP_Background_RequestParamSave" in param
    assert "SVC_Param_SavePendingToFlash" in background
    assert "功能" in background_h
    assert "后台低优先级任务" in background_h
    assert "APP_FlashService_ReadData" in param
    assert "APP_FlashService_WriteData" in param
    assert "APP_FlashService_EraseSector" in param
    assert "osMessageQueueGet(backgroundReqQueueHandle" in background
    assert "osMessageQueuePut(backgroundRespQueueHandle" in background
