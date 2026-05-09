#ifndef APP_AIWB2_H
#define APP_AIWB2_H

#include <stdint.h>

typedef enum {
    APP_AIWB2_STATE_START_DELAY = 0,
    APP_AIWB2_STATE_WAIT_PROBE,
    APP_AIWB2_STATE_ESCAPE_BEFORE,
    APP_AIWB2_STATE_ESCAPE_AFTER,
    APP_AIWB2_STATE_SEND_COMMAND,
    APP_AIWB2_STATE_WAIT_COMMAND,
    APP_AIWB2_STATE_WAIT_BOOT_CONNECT,
    APP_AIWB2_STATE_TRANSPARENT,
    APP_AIWB2_STATE_RETRY_DELAY
} APP_AiWB2_State;

void APP_AiWB2_Init(void);
void APP_AiWB2_Tick(void);
void APP_AiWB2_ProcessLine(const char *line);
uint8_t APP_AiWB2_IsTransparent(void);
uint8_t APP_AiWB2_IsControlPayload(const char *line);
uint8_t APP_AiWB2_ShouldConsumeTransparentLine(const char *line);
void APP_AiWB2_AssumeTransparent(void);
uint8_t APP_AiWB2_StartProvision(const char *ssid,
                                 const char *password,
                                 const char *host,
                                 const char *port);
uint8_t APP_AiWB2_SendRawCommand(const char *command);
void APP_AiWB2_SendDiagCommands(void);
APP_AiWB2_State APP_AiWB2_GetState(void);
uint32_t APP_AiWB2_GetRetryCount(void);
int32_t APP_AiWB2_GetLastSocketError(void);
uint8_t APP_AiWB2_IsPowerRecycleActive(void);
uint32_t APP_AiWB2_GetDeadlineRemainingMs(void);
uint8_t APP_AiWB2_IsProvisionActive(void);
uint32_t APP_AiWB2_GetCommandIndex(void);
uint32_t APP_AiWB2_GetCommandCount(void);
const char *APP_AiWB2_GetCurrentCommand(void);

#endif
