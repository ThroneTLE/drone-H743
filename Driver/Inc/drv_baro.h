#ifndef DRV_BARO_H
#define DRV_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define DRV_BARO_ID_VALUE 0x10U

typedef enum {
    DRV_BARO_OK = 0,
    DRV_BARO_ERROR,
    DRV_BARO_TIMEOUT,
    DRV_BARO_BAD_ID,
    DRV_BARO_INVALID_ARG
} DRV_BARO_Status;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} DRV_BARO_Bus;

typedef struct {
    DRV_BARO_Bus bus;
    uint8_t      product_id;
} DRV_BARO_Device;

DRV_BARO_Status DRV_BARO_Init(DRV_BARO_Device *dev, const DRV_BARO_Bus *bus);
DRV_BARO_Status DRV_BARO_ReadId(DRV_BARO_Device *dev, uint8_t *product_id);
DRV_BARO_Status DRV_BARO_ReadIdTxRx(DRV_BARO_Device *dev, uint8_t *product_id);
DRV_BARO_Status DRV_BARO_ReadRegister(DRV_BARO_Device *dev, uint8_t reg, uint8_t *value);
DRV_BARO_Status DRV_BARO_ReadRegisters(DRV_BARO_Device *dev, uint8_t reg,
                                       uint8_t *data, uint16_t len);
DRV_BARO_Status DRV_BARO_WriteRegister(DRV_BARO_Device *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
