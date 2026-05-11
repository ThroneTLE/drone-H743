#ifndef BSP_BARO_H
#define BSP_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "drv_baro.h"

typedef DRV_BARO_Status BSP_SPL06_Status;
typedef DRV_BARO_Bus    BSP_SPL06_Bus;
typedef DRV_BARO_Device BSP_SPL06_Device;

#define BSP_SPL06_ID_VALUE DRV_BARO_ID_VALUE
#define BSP_SPL06_OK       DRV_BARO_OK
#define BSP_SPL06_ERROR    DRV_BARO_ERROR
#define BSP_SPL06_TIMEOUT  DRV_BARO_TIMEOUT
#define BSP_SPL06_BAD_ID   DRV_BARO_BAD_ID
#define BSP_SPL06_INVALID_ARG DRV_BARO_INVALID_ARG

DRV_BARO_Status BSP_BARO_Init(void);
DRV_BARO_Status BSP_BARO_ProbeId(uint8_t *product_id);
DRV_BARO_Status BSP_BARO_ProbeIdTxRx(uint8_t *product_id);
DRV_BARO_Status BSP_BARO_ReadId(uint8_t *product_id);
DRV_BARO_Status BSP_BARO_ReadRawRegister(uint8_t reg, uint8_t *value);
DRV_BARO_Status BSP_BARO_ReadRawRegisters(uint8_t reg, uint8_t *data, uint16_t len);
const DRV_BARO_Device *BSP_BARO_GetDevice(void);
void BSP_BARO_DebugReadLevels(uint8_t *cs_level, uint8_t *miso_level);
void BSP_BARO_Invalidate(void);

#ifdef __cplusplus
}
#endif

#endif
