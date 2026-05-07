#include "bsp_aiwb2_power.h"

#include "bsp_gpio.h"

static uint8_t aiwb2_last_written_state = 1U;
static uint32_t aiwb2_write_count;

void BSP_AiWB2_PowerInit(void)
{
    BSP_AiWB2_SetEnabled(1U);
}

void BSP_AiWB2_SetEnabled(uint8_t enabled)
{
    aiwb2_last_written_state = (enabled != 0U) ? 1U : 0U;
    ++aiwb2_write_count;
    BSP_GPIO_Write(BSP_GPIO_PC6, aiwb2_last_written_state);
}

uint8_t BSP_AiWB2_IsEnabled(void)
{
    return BSP_GPIO_Read(BSP_GPIO_PC6);
}

uint8_t BSP_AiWB2_GetLastWrittenState(void)
{
    return aiwb2_last_written_state;
}

uint32_t BSP_AiWB2_GetWriteCount(void)
{
    return aiwb2_write_count;
}
