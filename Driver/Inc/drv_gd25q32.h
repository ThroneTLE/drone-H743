#ifndef DRV_GD25Q32_H
#define DRV_GD25Q32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define DRV_GD25Q32_JEDEC_MANUFACTURER_ID 0xC8U
#define DRV_GD25Q32_JEDEC_MEMORY_TYPE     0x40U
#define DRV_GD25Q32_JEDEC_CAPACITY_ID     0x16U
#define DRV_GD25Q32_SIZE_BYTES            (4UL * 1024UL * 1024UL)

#define DRV_GD25Q32_SECTOR_SIZE           4096U
#define DRV_GD25Q32_BLOCK32K_SIZE         32768U
#define DRV_GD25Q32_BLOCK64K_SIZE         65536U
#define DRV_GD25Q32_PAGE_SIZE             256U
#define DRV_GD25Q32_DMA_CHUNK             512U

typedef enum {
    DRV_GD25Q32_OK = 0,
    DRV_GD25Q32_ERROR,
    DRV_GD25Q32_TIMEOUT,
    DRV_GD25Q32_BAD_ID,
    DRV_GD25Q32_INVALID_ARG,
    DRV_GD25Q32_BUSY,
    DRV_GD25Q32_DMA_ERROR
} DRV_GD25Q32_Status;

typedef enum {
    DRV_GD25Q32_DMA_IDLE = 0,
    DRV_GD25Q32_DMA_BUSY,
    DRV_GD25Q32_DMA_DONE,
    DRV_GD25Q32_DMA_STATE_ERROR,
    DRV_GD25Q32_DMA_TIMEOUT
} DRV_GD25Q32_DmaState;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
    void             (*cache_clean)(const void *addr, uint32_t len);
    void             (*cache_invalidate)(const void *addr, uint32_t len);
} DRV_GD25Q32_Bus;

typedef struct {
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity_id;
} DRV_GD25Q32_JedecId;

typedef struct {
    DRV_GD25Q32_Bus      bus;
    DRV_GD25Q32_JedecId  jedec_id;
    volatile DRV_GD25Q32_DmaState dma_state;
    volatile uint32_t   dma_error_code;
} DRV_GD25Q32_Device;

DRV_GD25Q32_Status DRV_GD25Q32_Init(DRV_GD25Q32_Device *dev, const DRV_GD25Q32_Bus *bus);
DRV_GD25Q32_Status DRV_GD25Q32_ReleaseFromPowerDown(DRV_GD25Q32_Device *dev);
DRV_GD25Q32_Status DRV_GD25Q32_ReadJedecId(DRV_GD25Q32_Device *dev,
                                       DRV_GD25Q32_JedecId *jedec_id);
DRV_GD25Q32_Status DRV_GD25Q32_ReadStatus1(DRV_GD25Q32_Device *dev, uint8_t *status1);
DRV_GD25Q32_Status DRV_GD25Q32_ReadData(DRV_GD25Q32_Device *dev, uint32_t address,
                                    uint8_t *data, uint32_t length);
DRV_GD25Q32_Status DRV_GD25Q32_ReadDataFast(DRV_GD25Q32_Device *dev, uint32_t address,
                                        uint8_t *data, uint32_t length);
DRV_GD25Q32_Status DRV_GD25Q32_EraseSector(DRV_GD25Q32_Device *dev, uint32_t address);
DRV_GD25Q32_Status DRV_GD25Q32_EraseBlock32K(DRV_GD25Q32_Device *dev, uint32_t address);
DRV_GD25Q32_Status DRV_GD25Q32_EraseBlock64K(DRV_GD25Q32_Device *dev, uint32_t address);
DRV_GD25Q32_Status DRV_GD25Q32_PageProgram(DRV_GD25Q32_Device *dev, uint32_t address,
                                       const uint8_t *data, uint16_t length);
DRV_GD25Q32_Status DRV_GD25Q32_WriteData(DRV_GD25Q32_Device *dev, uint32_t address,
                                     const uint8_t *data, uint32_t length);

void DRV_GD25Q32_DmaTxCplt(DRV_GD25Q32_Device *dev);
void DRV_GD25Q32_DmaRxCplt(DRV_GD25Q32_Device *dev);
void DRV_GD25Q32_DmaError(DRV_GD25Q32_Device *dev);

#ifdef __cplusplus
}
#endif

#endif
