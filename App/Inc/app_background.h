#ifndef APP_BACKGROUND_H
#define APP_BACKGROUND_H

#include "cmsis_os2.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 功能：后台低优先级任务的 App 层接口。
 *
 * backgroundTask 调用本模块的 Init/Step。它不是参数服务本身，
 * 而是负责执行慢操作队列，例如调用 Param API 把参数保存到 FLASH，
 * 或在维护模式下执行大块 FLASH 读写。
 */

typedef enum {
    APP_BACKGROUND_OP_READ_BLOCK = 0,
    APP_BACKGROUND_OP_WRITE_BLOCK,
    APP_BACKGROUND_OP_ERASE_SECTOR,
    APP_BACKGROUND_OP_PARAM_SAVE,
} APP_BackgroundOp;

typedef enum {
    APP_BACKGROUND_STATUS_OK = 0,
    APP_BACKGROUND_STATUS_ERROR,
    APP_BACKGROUND_STATUS_TIMEOUT,
    APP_BACKGROUND_STATUS_INVALID_ARG,
    APP_BACKGROUND_STATUS_BUSY,
    APP_BACKGROUND_STATUS_QUEUE_FULL,
} APP_BackgroundStatus;

typedef struct {
    uint32_t request_id;
    APP_BackgroundOp op;
    uint32_t address;
    uint32_t length;
    uint8_t *data;
} APP_BackgroundRequest;

typedef struct {
    uint32_t request_id;
    APP_BackgroundOp op;
    APP_BackgroundStatus status;
    uint32_t address;
    uint32_t length;
    uint8_t *data;
} APP_BackgroundResponse;

void APP_Background_Init(void);
void APP_Background_Step(void);

uint32_t APP_Background_RequestReadBlock(uint32_t address, uint8_t *data, uint32_t size);
uint32_t APP_Background_RequestWriteBlock(uint32_t address, uint8_t *data, uint32_t size);
uint32_t APP_Background_RequestEraseSector(uint32_t address);
uint32_t APP_Background_RequestParamSave(void);
uint8_t APP_Background_PollResponse(APP_BackgroundResponse *response);

#ifdef __cplusplus
}
#endif

#endif
