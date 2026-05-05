#include "bsp_system.h"
#include "stm32h7xx_hal.h"

void BSP_System_Init(void)
{
    /* Clocks and MPU already configured by CubeMX in main.c */
    BSP_Cache_Enable();
}

void BSP_Cache_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

void BSP_Cache_Disable(void)
{
    SCB_DisableICache();
    SCB_DisableDCache();
}

BSP_ClockInfo BSP_System_GetClockInfo(void)
{
    BSP_ClockInfo info = {0};
    info.sysclk = HAL_RCC_GetSysClockFreq();
    info.hclk   = HAL_RCC_GetHCLKFreq();
    info.pclk1  = HAL_RCC_GetPCLK1Freq();
    info.pclk2  = HAL_RCC_GetPCLK2Freq();
    return info;
}
