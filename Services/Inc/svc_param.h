#ifndef SVC_PARAM_H
#define SVC_PARAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 功能：参数服务层 API。
 *
 * 这个模块没有自己的任务，不属于 App 执行体。它维护 RAM 中的参数镜像，
 * 提供启动时从 FLASH 加载参数、运行期请求后台慢保存的 API。
 * 后续 PID/EKF 参数表可以继续放到这里，但控制环应该读本地缓存，不要每周期读 FLASH。
 */

#define SVC_PARAM_MAX_PAYLOAD 2048U

typedef enum {
    SVC_PARAM_STATUS_OK = 0,
    SVC_PARAM_STATUS_ERROR,
    SVC_PARAM_STATUS_INVALID_ARG,
    SVC_PARAM_STATUS_NOT_READY,
    SVC_PARAM_STATUS_NO_VALID_RECORD,
} SVC_ParamStatus;

void SVC_Param_Init(void);
uint8_t SVC_Param_IsReady(void);
uint8_t SVC_Param_IsDirty(void);
uint32_t SVC_Param_GetGeneration(void);

SVC_ParamStatus SVC_Param_LoadFromFlash(void);
SVC_ParamStatus SVC_Param_GetBlob(uint8_t *data, uint32_t max_size, uint32_t *out_size);
SVC_ParamStatus SVC_Param_SetBlob(const uint8_t *data, uint32_t size);

uint32_t SVC_Param_RequestSaveBlob(void);
SVC_ParamStatus SVC_Param_SavePendingToFlash(void);

#ifdef __cplusplus
}
#endif

#endif
