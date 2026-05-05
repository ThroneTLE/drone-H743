#ifndef BSP_GD25Q32_H
#define BSP_GD25Q32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define BSP_GD25Q32_JEDEC_MANUFACTURER_ID 0xC8U
#define BSP_GD25Q32_JEDEC_MEMORY_TYPE    0x40U
#define BSP_GD25Q32_JEDEC_CAPACITY_ID    0x16U
#define BSP_GD25Q32_FLASH_SIZE_BYTES     (4UL * 1024UL * 1024UL)

typedef enum {
    BSP_GD25Q32_OK = 0,
    BSP_GD25Q32_ERROR,
    BSP_GD25Q32_TIMEOUT,
    BSP_GD25Q32_BAD_ID,
    BSP_GD25Q32_INVALID_ARG
} BSP_GD25Q32_Status;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} BSP_GD25Q32_Bus;

typedef struct {
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity_id;
} BSP_GD25Q32_JedecId;

typedef struct {
    BSP_GD25Q32_Bus     bus;
    BSP_GD25Q32_JedecId jedec_id;
} BSP_GD25Q32_Device;

BSP_GD25Q32_Status BSP_GD25Q32_Init(BSP_GD25Q32_Device *dev,
                                    const BSP_GD25Q32_Bus *bus);
BSP_GD25Q32_Status BSP_GD25Q32_ReleaseFromPowerDown(BSP_GD25Q32_Device *dev);
BSP_GD25Q32_Status BSP_GD25Q32_ReadJedecId(BSP_GD25Q32_Device *dev,
                                           BSP_GD25Q32_JedecId *jedec_id);
BSP_GD25Q32_Status BSP_GD25Q32_ReadStatus1(BSP_GD25Q32_Device *dev,
                                           uint8_t *status1);
BSP_GD25Q32_Status BSP_GD25Q32_ReadData(BSP_GD25Q32_Device *dev,
                                        uint32_t address,
                                        uint8_t *data,
                                        uint32_t length);

#ifdef __cplusplus
}
#endif

#endif
