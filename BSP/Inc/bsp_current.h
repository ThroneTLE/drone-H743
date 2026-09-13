#ifndef BSP_CURRENT_H
#define BSP_CURRENT_H
#include <stdint.h>
typedef enum { BSP_CURRENT_OK=0, BSP_CURRENT_NOT_READY, BSP_CURRENT_TIMEOUT, BSP_CURRENT_ERROR } BSP_CurrentStatus;
/* messageTask only. ADC1/PC1 current + PC0 voltage, 16-bit polling, no DMA or IRQ.
 * ADC internal calibration is not sensor/current scale calibration.
 */
BSP_CurrentStatus BSP_Current_Init(void);
/* Bounded polling (1 ms); failure leaves raw unchanged. No automatic re-init. */
BSP_CurrentStatus BSP_Current_Read(uint32_t *raw);
typedef struct { uint32_t raw, sequence; BSP_CurrentStatus status; } BSP_VoltageSample;
/* Same task reads the voltage from the latest completed ADC pair; never starts ADC. */
void BSP_Current_GetVoltageSample(BSP_VoltageSample *out);
#endif
