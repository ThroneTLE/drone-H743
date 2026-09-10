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

/*
 * BSP_BARO_DebugReadLevels 在 I2C 板子上返回这个值。
 *
 * 这两个电平是给 SPI 用的（片选拉住了没有、MISO 有没有被拽死），I2C 上根本没有
 * 对应的东西。这时候返回 0 会读起来像"片选一直是低"——一个看着合理、其实是编的
 * 结论；诊断宁可说"不适用"也不能撒谎（decoupling-spec D5-3）。GPIO 电平只有 0/1，
 * 所以 0xFF 不可能被误认成真实读数。
 *
 * I2C 上要判断气压计死活，用 BSP_BARO_ProbeId 直接读芯片 ID，比看线电平更直接。
 */
#define BSP_BARO_LEVEL_NOT_APPLICABLE 0xFFU

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
