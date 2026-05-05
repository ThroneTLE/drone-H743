#ifndef BSP_BARO_H
#define BSP_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_spl06.h"

BSP_SPL06_Status BSP_BARO_Init(void);
BSP_SPL06_Status BSP_BARO_ProbeId(uint8_t *product_id);
BSP_SPL06_Status BSP_BARO_ProbeIdTxRx(uint8_t *product_id);
BSP_SPL06_Status BSP_BARO_ReadId(uint8_t *product_id);
BSP_SPL06_Status BSP_BARO_ReadRawRegister(uint8_t reg, uint8_t *value);
const BSP_SPL06_Device *BSP_BARO_GetDevice(void);
void BSP_BARO_DebugReadLevels(uint8_t *cs_level, uint8_t *miso_level);
void BSP_BARO_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
