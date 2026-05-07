#ifndef BSP_AIWB2_POWER_H
#define BSP_AIWB2_POWER_H

#include <stdint.h>

void BSP_AiWB2_PowerInit(void);
void BSP_AiWB2_SetEnabled(uint8_t enabled);
uint8_t BSP_AiWB2_IsEnabled(void);
uint8_t BSP_AiWB2_GetLastWrittenState(void);
uint32_t BSP_AiWB2_GetWriteCount(void);

#endif
