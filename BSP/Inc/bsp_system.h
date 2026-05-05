#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include <stdint.h>

typedef struct {
    uint32_t sysclk;   /* SYSCLK frequency in Hz */
    uint32_t hclk;     /* HCLK frequency in Hz   */
    uint32_t pclk1;    /* APB1 clock in Hz       */
    uint32_t pclk2;    /* APB2 clock in Hz       */
} BSP_ClockInfo;

void BSP_System_Init(void);
void BSP_Cache_Enable(void);
void BSP_Cache_Disable(void);
BSP_ClockInfo BSP_System_GetClockInfo(void);

#endif
