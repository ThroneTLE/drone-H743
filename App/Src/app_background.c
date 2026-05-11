#include "app_background.h"

#include "app_flash_service.h"
#include "app_tasks.h"
#include "svc_param.h"

/*
 * 功能：后台低优先级任务实现。
 *
 * 这里是 App 层任务逻辑，负责消费 backgroundReqQueue。
 * 它会调用 Param 服务实现参数慢存储；Param 仍然是唯一服务层模块。
 */

static uint32_t background_next_request_id;

static APP_BackgroundStatus background_status_from_flash(APP_FlashService_Status status)
{
    switch (status) {
    case APP_FLASH_SERVICE_OK:
        return APP_BACKGROUND_STATUS_OK;
    case APP_FLASH_SERVICE_TIMEOUT:
        return APP_BACKGROUND_STATUS_TIMEOUT;
    case APP_FLASH_SERVICE_INVALID_ARG:
        return APP_BACKGROUND_STATUS_INVALID_ARG;
    case APP_FLASH_SERVICE_BUSY:
        return APP_BACKGROUND_STATUS_BUSY;
    default:
        return APP_BACKGROUND_STATUS_ERROR;
    }
}

static APP_BackgroundStatus background_status_from_param(SVC_ParamStatus status)
{
    switch (status) {
    case SVC_PARAM_STATUS_OK:
    case SVC_PARAM_STATUS_NO_VALID_RECORD:
        return APP_BACKGROUND_STATUS_OK;
    case SVC_PARAM_STATUS_INVALID_ARG:
    case SVC_PARAM_STATUS_NOT_READY:
        return APP_BACKGROUND_STATUS_INVALID_ARG;
    default:
        return APP_BACKGROUND_STATUS_ERROR;
    }
}

static uint32_t background_alloc_request_id(void)
{
    background_next_request_id++;
    if (background_next_request_id == 0U) {
        background_next_request_id = 1U;
    }
    return background_next_request_id;
}

static uint32_t background_submit_request(APP_BackgroundRequest *request)
{
    if ((request == NULL) || (backgroundReqQueueHandle == NULL)) {
        return 0U;
    }

    request->request_id = background_alloc_request_id();
    if (osMessageQueuePut(backgroundReqQueueHandle, request, 0U, 0U) != osOK) {
        return 0U;
    }

    return request->request_id;
}

static void background_publish_response(const APP_BackgroundRequest *request,
                                        APP_BackgroundStatus status,
                                        uint32_t completed_length)
{
    if ((request == NULL) || (backgroundRespQueueHandle == NULL)) {
        return;
    }

    APP_BackgroundResponse response = {
        .request_id = request->request_id,
        .op = request->op,
        .status = status,
        .address = request->address,
        .length = completed_length,
        .data = request->data,
    };

    (void)osMessageQueuePut(backgroundRespQueueHandle, &response, 0U, 0U);
}

void APP_Background_Init(void)
{
    background_next_request_id = 0U;
    SVC_Param_Init();
    (void)background_status_from_param(SVC_Param_LoadFromFlash());
}

void APP_Background_Step(void)
{
    APP_BackgroundRequest request;
    APP_BackgroundStatus result = APP_BACKGROUND_STATUS_OK;

    if (osMessageQueueGet(backgroundReqQueueHandle, &request, NULL, osWaitForever) != osOK) {
        return;
    }

    switch (request.op) {
    case APP_BACKGROUND_OP_READ_BLOCK:
        result = background_status_from_flash(APP_FlashService_ReadDataFast(request.address,
                                                                            request.data,
                                                                            request.length));
        background_publish_response(&request,
                                    result,
                                    (result == APP_BACKGROUND_STATUS_OK) ? request.length : 0U);
        break;
    case APP_BACKGROUND_OP_WRITE_BLOCK:
        result = background_status_from_flash(APP_FlashService_WriteData(request.address,
                                                                         request.data,
                                                                         request.length));
        background_publish_response(&request,
                                    result,
                                    (result == APP_BACKGROUND_STATUS_OK) ? request.length : 0U);
        break;
    case APP_BACKGROUND_OP_ERASE_SECTOR:
        result = background_status_from_flash(APP_FlashService_EraseSector(request.address));
        background_publish_response(&request, result, 0U);
        break;
    case APP_BACKGROUND_OP_PARAM_SAVE:
        result = background_status_from_param(SVC_Param_SavePendingToFlash());
        background_publish_response(&request, result, 0U);
        break;
    default:
        background_publish_response(&request, APP_BACKGROUND_STATUS_INVALID_ARG, 0U);
        break;
    }
}

uint32_t APP_Background_RequestReadBlock(uint32_t address, uint8_t *data, uint32_t size)
{
    if ((data == NULL) || (size == 0U)) {
        return 0U;
    }

    APP_BackgroundRequest request = {
        .request_id = 0U,
        .op = APP_BACKGROUND_OP_READ_BLOCK,
        .address = address,
        .length = size,
        .data = data,
    };

    return background_submit_request(&request);
}

uint32_t APP_Background_RequestWriteBlock(uint32_t address, uint8_t *data, uint32_t size)
{
    if ((data == NULL) || (size == 0U)) {
        return 0U;
    }

    APP_BackgroundRequest request = {
        .request_id = 0U,
        .op = APP_BACKGROUND_OP_WRITE_BLOCK,
        .address = address,
        .length = size,
        .data = data,
    };

    return background_submit_request(&request);
}

uint32_t APP_Background_RequestEraseSector(uint32_t address)
{
    APP_BackgroundRequest request = {
        .request_id = 0U,
        .op = APP_BACKGROUND_OP_ERASE_SECTOR,
        .address = address,
        .length = 0U,
        .data = NULL,
    };

    return background_submit_request(&request);
}

uint32_t APP_Background_RequestParamSave(void)
{
    APP_BackgroundRequest request = {
        .request_id = 0U,
        .op = APP_BACKGROUND_OP_PARAM_SAVE,
        .address = 0U,
        .length = 0U,
        .data = NULL,
    };

    return background_submit_request(&request);
}

uint8_t APP_Background_PollResponse(APP_BackgroundResponse *response)
{
    if ((response == NULL) || (backgroundRespQueueHandle == NULL)) {
        return 0U;
    }

    return (osMessageQueueGet(backgroundRespQueueHandle, response, NULL, 0U) == osOK) ? 1U : 0U;
}
