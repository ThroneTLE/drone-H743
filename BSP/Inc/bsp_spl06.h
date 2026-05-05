#ifndef BSP_SPL06_H
#define BSP_SPL06_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define BSP_SPL06_ID_VALUE 0x10U

typedef enum {
    BSP_SPL06_OK = 0,
    BSP_SPL06_ERROR,
    BSP_SPL06_TIMEOUT,
    BSP_SPL06_BAD_ID,
    BSP_SPL06_INVALID_ARG
} BSP_SPL06_Status;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} BSP_SPL06_Bus;

typedef struct {
    BSP_SPL06_Bus bus;
    uint8_t       product_id;
} BSP_SPL06_Device;

BSP_SPL06_Status BSP_SPL06_Init(BSP_SPL06_Device *dev, const BSP_SPL06_Bus *bus);
BSP_SPL06_Status BSP_SPL06_ReadId(BSP_SPL06_Device *dev, uint8_t *product_id);
BSP_SPL06_Status BSP_SPL06_ReadIdTxRx(BSP_SPL06_Device *dev, uint8_t *product_id);
BSP_SPL06_Status BSP_SPL06_ReadRegister(BSP_SPL06_Device *dev,
                                        uint8_t reg,
                                        uint8_t *value);
BSP_SPL06_Status BSP_SPL06_ReadRegisters(BSP_SPL06_Device *dev,
                                         uint8_t reg,
                                         uint8_t *data,
                                         uint16_t len);
BSP_SPL06_Status BSP_SPL06_WriteRegister(BSP_SPL06_Device *dev,
                                         uint8_t reg,
                                         uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
