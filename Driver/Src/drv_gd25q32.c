#include "drv_gd25q32.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <string.h>

#define GD25Q32_CMD_RELEASE_POWER_DOWN 0xABU
#define GD25Q32_CMD_READ_STATUS1       0x05U
#define GD25Q32_CMD_WRITE_ENABLE       0x06U
#define GD25Q32_CMD_READ_DATA          0x03U
#define GD25Q32_CMD_READ_DATA_FAST     0x0BU
#define GD25Q32_CMD_PAGE_PROGRAM       0x02U
#define GD25Q32_CMD_SECTOR_ERASE       0x20U
#define GD25Q32_CMD_BLOCK_ERASE_32K    0x52U
#define GD25Q32_CMD_BLOCK_ERASE_64K    0xD8U
#define GD25Q32_CMD_READ_JEDEC_ID      0x9FU

#define GD25Q32_DEFAULT_TIMEOUT_MS     100U
#define GD25Q32_WAKE_DELAY_MS          1U
#define GD25Q32_BUSY_POLL_DELAY_MS     1U
#define GD25Q32_PROGRAM_TIMEOUT_MS     500U
#define GD25Q32_ERASE_TIMEOUT_MS       5000U
#define GD25Q32_BLOCK_ERASE_32K_TIMEOUT_MS 2500U
#define GD25Q32_BLOCK_ERASE_64K_TIMEOUT_MS 4000U
#define GD25Q32_STATUS1_BUSY           0x01U
#define GD25Q32_FAST_READ_DUMMY_BYTES  1U
#define GD25Q32_FAST_READ_CMD_LEN      5U

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t flash_dma_tx[DRV_GD25Q32_DMA_CHUNK + GD25Q32_FAST_READ_CMD_LEN];

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t flash_dma_rx[DRV_GD25Q32_DMA_CHUNK + GD25Q32_FAST_READ_CMD_LEN];

static osThreadId_t dma_wait_thread;

#define FLASH_DMA_DONE_FLAG  0x00000001U
#define FLASH_DMA_ERROR_FLAG 0x00000002U

static uint32_t gd25q32_timeout_ms(const DRV_GD25Q32_Device *dev)
{
    return (dev->bus.timeout_ms != 0U) ? dev->bus.timeout_ms : GD25Q32_DEFAULT_TIMEOUT_MS;
}

static void gd25q32_delay_ms(DRV_GD25Q32_Device *dev, uint32_t delay_ms)
{
    if (dev->bus.delay_ms != NULL) {
        dev->bus.delay_ms(delay_ms);
    } else {
        HAL_Delay(delay_ms);
    }
}

static void gd25q32_cs_low(DRV_GD25Q32_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_RESET);
}

static void gd25q32_cs_high(DRV_GD25Q32_Device *dev)
{
    HAL_GPIO_WritePin(dev->bus.cs_port, dev->bus.cs_pin, GPIO_PIN_SET);
}

static void gd25q32_cache_clean(DRV_GD25Q32_Device *dev, const void *addr, uint32_t len)
{
    if (dev->bus.cache_clean != NULL) {
        dev->bus.cache_clean(addr, len);
    }
}

static void gd25q32_cache_invalidate(DRV_GD25Q32_Device *dev, const void *addr, uint32_t len)
{
    if (dev->bus.cache_invalidate != NULL) {
        dev->bus.cache_invalidate(addr, len);
    }
}

static DRV_GD25Q32_Status gd25q32_from_hal_status(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:      return DRV_GD25Q32_OK;
    case HAL_TIMEOUT:  return DRV_GD25Q32_TIMEOUT;
    case HAL_ERROR:
    case HAL_BUSY:
    default:           return DRV_GD25Q32_ERROR;
    }
}

static DRV_GD25Q32_Status gd25q32_spi_blocking_tx(DRV_GD25Q32_Device *dev,
                                                const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef hal_status;

    gd25q32_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi, (uint8_t *)data, len,
                                  gd25q32_timeout_ms(dev));
    gd25q32_cs_high(dev);
    return gd25q32_from_hal_status(hal_status);
}

static DRV_GD25Q32_Status gd25q32_read_after_command(DRV_GD25Q32_Device *dev,
                                                   const uint8_t *command,
                                                   uint16_t command_length,
                                                   uint8_t *data,
                                                   uint16_t data_length)
{
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (command == NULL) || (command_length == 0U) ||
        (data == NULL) || (data_length == 0U)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    gd25q32_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi, (uint8_t *)(uintptr_t)command,
                                  command_length, gd25q32_timeout_ms(dev));
    if (hal_status == HAL_OK) {
        hal_status = HAL_SPI_Receive(dev->bus.hspi, data, data_length,
                                     gd25q32_timeout_ms(dev));
    }
    gd25q32_cs_high(dev);

    return gd25q32_from_hal_status(hal_status);
}

static DRV_GD25Q32_Status gd25q32_write_enable(DRV_GD25Q32_Device *dev)
{
    uint8_t command = GD25Q32_CMD_WRITE_ENABLE;

    if (dev == NULL) { return DRV_GD25Q32_INVALID_ARG; }

    return gd25q32_spi_blocking_tx(dev, &command, 1U);
}

static DRV_GD25Q32_Status gd25q32_wait_while_busy(DRV_GD25Q32_Device *dev,
                                                 uint32_t timeout_ms)
{
    uint32_t start_tick;
    DRV_GD25Q32_Status status;
    uint8_t status1;

    if (dev == NULL) { return DRV_GD25Q32_INVALID_ARG; }

    start_tick = HAL_GetTick();
    do {
        status = DRV_GD25Q32_ReadStatus1(dev, &status1);
        if (status != DRV_GD25Q32_OK) { return status; }
        if ((status1 & GD25Q32_STATUS1_BUSY) == 0U) { return DRV_GD25Q32_OK; }
        gd25q32_delay_ms(dev, GD25Q32_BUSY_POLL_DELAY_MS);
    } while ((HAL_GetTick() - start_tick) < timeout_ms);

    return DRV_GD25Q32_TIMEOUT;
}

static void gd25q32_dma_prepare(DRV_GD25Q32_Device *dev)
{
    dev->dma_state = DRV_GD25Q32_DMA_BUSY;
    if (osKernelGetState() == osKernelRunning) {
        dma_wait_thread = osThreadGetId();
        osThreadFlagsClear(FLASH_DMA_DONE_FLAG | FLASH_DMA_ERROR_FLAG);
    } else {
        dma_wait_thread = NULL;
    }
}

static DRV_GD25Q32_Status gd25q32_dma_wait(DRV_GD25Q32_Device *dev, uint32_t timeout_ms)
{
    uint32_t start_tick;
    uint32_t flags;

    if (osKernelGetState() == osKernelRunning) {
        flags = osThreadFlagsWait(FLASH_DMA_DONE_FLAG | FLASH_DMA_ERROR_FLAG,
                                   osFlagsWaitAny, timeout_ms);
        dma_wait_thread = NULL;
        if ((flags & osFlagsError) != 0U) {
            dev->dma_state = DRV_GD25Q32_DMA_TIMEOUT;
            (void)HAL_SPI_Abort(dev->bus.hspi);
            gd25q32_cs_high(dev);
            dev->dma_state = DRV_GD25Q32_DMA_IDLE;
            return DRV_GD25Q32_TIMEOUT;
        }
        if (flags & FLASH_DMA_DONE_FLAG) {
            dev->dma_state = DRV_GD25Q32_DMA_IDLE;
            return DRV_GD25Q32_OK;
        }
        if (flags & FLASH_DMA_ERROR_FLAG) {
            dev->dma_state = DRV_GD25Q32_DMA_IDLE;
            return DRV_GD25Q32_DMA_ERROR;
        }
        dev->dma_state = DRV_GD25Q32_DMA_TIMEOUT;
        (void)HAL_SPI_Abort(dev->bus.hspi);
        gd25q32_cs_high(dev);
        dev->dma_state = DRV_GD25Q32_DMA_IDLE;
        return DRV_GD25Q32_TIMEOUT;
    }

    start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        if (dev->dma_state == DRV_GD25Q32_DMA_DONE) {
            dev->dma_state = DRV_GD25Q32_DMA_IDLE;
            return DRV_GD25Q32_OK;
        }
        if (dev->dma_state == DRV_GD25Q32_DMA_STATE_ERROR) {
            dev->dma_state = DRV_GD25Q32_DMA_IDLE;
            return DRV_GD25Q32_DMA_ERROR;
        }
    }

    dev->dma_state = DRV_GD25Q32_DMA_TIMEOUT;
    (void)HAL_SPI_Abort(dev->bus.hspi);
    gd25q32_cs_high(dev);
    dev->dma_state = DRV_GD25Q32_DMA_IDLE;
    return DRV_GD25Q32_TIMEOUT;
}

void DRV_GD25Q32_DmaTxCplt(DRV_GD25Q32_Device *dev)
{
    if (dev == NULL) { return; }
    dev->dma_state = DRV_GD25Q32_DMA_DONE;
    if ((osKernelGetState() == osKernelRunning) && (dma_wait_thread != NULL)) {
        osThreadFlagsSet(dma_wait_thread, FLASH_DMA_DONE_FLAG);
    }
}

void DRV_GD25Q32_DmaRxCplt(DRV_GD25Q32_Device *dev)
{
    if (dev == NULL) { return; }
    dev->dma_state = DRV_GD25Q32_DMA_DONE;
    if ((osKernelGetState() == osKernelRunning) && (dma_wait_thread != NULL)) {
        osThreadFlagsSet(dma_wait_thread, FLASH_DMA_DONE_FLAG);
    }
}

void DRV_GD25Q32_DmaError(DRV_GD25Q32_Device *dev)
{
    if (dev == NULL) { return; }
    dev->dma_state = DRV_GD25Q32_DMA_STATE_ERROR;
    dev->dma_error_code = HAL_SPI_GetError(dev->bus.hspi);
    if ((osKernelGetState() == osKernelRunning) && (dma_wait_thread != NULL)) {
        osThreadFlagsSet(dma_wait_thread, FLASH_DMA_ERROR_FLAG);
    }
}

static bool gd25q32_addr_valid(uint32_t address, uint32_t length)
{
    if (length > DRV_GD25Q32_SIZE_BYTES) { return false; }
    if (address >= DRV_GD25Q32_SIZE_BYTES) { return false; }
    if (length > DRV_GD25Q32_SIZE_BYTES - address) { return false; }
    return true;
}

static bool gd25q32_addr_aligned(uint32_t address, uint32_t align)
{
    return (address % align) == 0U;
}

DRV_GD25Q32_Status DRV_GD25Q32_Init(DRV_GD25Q32_Device *dev, const DRV_GD25Q32_Bus *bus)
{
    DRV_GD25Q32_Status status;
    DRV_GD25Q32_JedecId jedec_id;

    if ((dev == NULL) || (bus == NULL) || (bus->hspi == NULL) ||
        (bus->cs_port == NULL)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    dev->dma_state = DRV_GD25Q32_DMA_IDLE;

    gd25q32_cs_high(dev);

    status = DRV_GD25Q32_ReleaseFromPowerDown(dev);
    if (status != DRV_GD25Q32_OK) { return status; }

    status = DRV_GD25Q32_ReadJedecId(dev, &jedec_id);
    if (status != DRV_GD25Q32_OK) { return status; }

    dev->jedec_id = jedec_id;
    if ((jedec_id.manufacturer_id != DRV_GD25Q32_JEDEC_MANUFACTURER_ID) ||
        (jedec_id.memory_type != DRV_GD25Q32_JEDEC_MEMORY_TYPE) ||
        (jedec_id.capacity_id != DRV_GD25Q32_JEDEC_CAPACITY_ID)) {
        return DRV_GD25Q32_BAD_ID;
    }

    return DRV_GD25Q32_OK;
}

DRV_GD25Q32_Status DRV_GD25Q32_ReleaseFromPowerDown(DRV_GD25Q32_Device *dev)
{
    uint8_t command = GD25Q32_CMD_RELEASE_POWER_DOWN;

    if (dev == NULL) { return DRV_GD25Q32_INVALID_ARG; }

    DRV_GD25Q32_Status st = gd25q32_spi_blocking_tx(dev, &command, 1U);
    if (st != DRV_GD25Q32_OK) { return st; }

    gd25q32_delay_ms(dev, GD25Q32_WAKE_DELAY_MS);
    return DRV_GD25Q32_OK;
}

DRV_GD25Q32_Status DRV_GD25Q32_ReadJedecId(DRV_GD25Q32_Device *dev,
                                        DRV_GD25Q32_JedecId *jedec_id)
{
    uint8_t command = GD25Q32_CMD_READ_JEDEC_ID;
    uint8_t response[3];
    DRV_GD25Q32_Status status;

    if ((dev == NULL) || (jedec_id == NULL)) { return DRV_GD25Q32_INVALID_ARG; }

    status = gd25q32_read_after_command(dev, &command, 1U, response, sizeof(response));
    if (status != DRV_GD25Q32_OK) { return status; }

    jedec_id->manufacturer_id = response[0];
    jedec_id->memory_type = response[1];
    jedec_id->capacity_id = response[2];
    dev->jedec_id = *jedec_id;

    return DRV_GD25Q32_OK;
}

DRV_GD25Q32_Status DRV_GD25Q32_ReadStatus1(DRV_GD25Q32_Device *dev, uint8_t *status1)
{
    uint8_t command = GD25Q32_CMD_READ_STATUS1;
    return gd25q32_read_after_command(dev, &command, 1U, status1, 1U);
}

DRV_GD25Q32_Status DRV_GD25Q32_ReadData(DRV_GD25Q32_Device *dev, uint32_t address,
                                    uint8_t *data, uint32_t length)
{
    uint8_t command[4];
    uint32_t remaining;
    uint32_t offset = 0U;
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) || (data == NULL) || (length == 0U)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    if (!gd25q32_addr_valid(address, length)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    command[0] = GD25Q32_CMD_READ_DATA;
    command[1] = (uint8_t)(address >> 16U);
    command[2] = (uint8_t)(address >> 8U);
    command[3] = (uint8_t)address;

    gd25q32_cs_low(dev);
    hal_status = HAL_SPI_Transmit(dev->bus.hspi, command, sizeof(command),
                                  gd25q32_timeout_ms(dev));
    remaining = length;
    while ((hal_status == HAL_OK) && (remaining > 0U)) {
        uint16_t chunk = (remaining > DRV_GD25Q32_DMA_CHUNK) ?
                         (uint16_t)DRV_GD25Q32_DMA_CHUNK :
                         (uint16_t)remaining;

        memset(flash_dma_tx, 0xFF, chunk);
        hal_status = HAL_SPI_TransmitReceive(dev->bus.hspi,
                                             flash_dma_tx,
                                             &data[offset],
                                             chunk,
                                             gd25q32_timeout_ms(dev));
        offset += chunk;
        remaining -= chunk;
    }
    if (hal_status != HAL_OK) {
        (void)HAL_SPI_Abort(dev->bus.hspi);
    }
    gd25q32_cs_high(dev);

    return gd25q32_from_hal_status(hal_status);
}

DRV_GD25Q32_Status DRV_GD25Q32_ReadDataFast(DRV_GD25Q32_Device *dev, uint32_t address,
                                        uint8_t *data, uint32_t length)
{
    uint32_t offset = 0U;
    DRV_GD25Q32_Status status;

    if ((dev == NULL) || (data == NULL) || (length == 0U)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    if (!gd25q32_addr_valid(address, length)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    if (dev->dma_state != DRV_GD25Q32_DMA_IDLE) {
        return DRV_GD25Q32_BUSY;
    }

    while (offset < length) {
        uint16_t chunk = (uint16_t)((length - offset) > DRV_GD25Q32_DMA_CHUNK
                                    ? DRV_GD25Q32_DMA_CHUNK
                                    : (length - offset));
        uint16_t dma_len = chunk + GD25Q32_FAST_READ_CMD_LEN;

        flash_dma_tx[0] = GD25Q32_CMD_READ_DATA_FAST;
        flash_dma_tx[1] = (uint8_t)((address + offset) >> 16U);
        flash_dma_tx[2] = (uint8_t)((address + offset) >> 8U);
        flash_dma_tx[3] = (uint8_t)(address + offset);
        flash_dma_tx[4] = 0x00U;
        memset(&flash_dma_tx[GD25Q32_FAST_READ_CMD_LEN], 0xFF, chunk);

        gd25q32_cache_clean(dev, flash_dma_tx, dma_len);
        gd25q32_cache_invalidate(dev, flash_dma_rx, dma_len);

        gd25q32_dma_prepare(dev);
        gd25q32_cs_low(dev);
        HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive_DMA(
            dev->bus.hspi, flash_dma_tx, flash_dma_rx, dma_len);
        if (hal_status != HAL_OK) {
            gd25q32_cs_high(dev);
            dma_wait_thread = NULL;
            dev->dma_state = DRV_GD25Q32_DMA_IDLE;
            return gd25q32_from_hal_status(hal_status);
        }

        status = gd25q32_dma_wait(dev, GD25Q32_DEFAULT_TIMEOUT_MS);
        gd25q32_cs_high(dev);

        if (status != DRV_GD25Q32_OK) { return status; }

        gd25q32_cache_invalidate(dev, flash_dma_rx, dma_len);
        memcpy(&data[offset], &flash_dma_rx[GD25Q32_FAST_READ_CMD_LEN], chunk);
        offset += chunk;
    }

    return DRV_GD25Q32_OK;
}

static DRV_GD25Q32_Status gd25q32_block_erase(DRV_GD25Q32_Device *dev,
                                            uint8_t erase_command,
                                            uint32_t address,
                                            uint32_t align_size,
                                            uint32_t timeout_ms)
{
    uint8_t command[4];
    DRV_GD25Q32_Status status;

    if (dev == NULL) { return DRV_GD25Q32_INVALID_ARG; }

    if (!gd25q32_addr_valid(address, align_size)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    if (!gd25q32_addr_aligned(address, align_size)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    status = gd25q32_wait_while_busy(dev, GD25Q32_DEFAULT_TIMEOUT_MS);
    if (status != DRV_GD25Q32_OK) { return status; }

    status = gd25q32_write_enable(dev);
    if (status != DRV_GD25Q32_OK) { return status; }

    command[0] = erase_command;
    command[1] = (uint8_t)(address >> 16U);
    command[2] = (uint8_t)(address >> 8U);
    command[3] = (uint8_t)address;

    status = gd25q32_spi_blocking_tx(dev, command, sizeof(command));
    if (status != DRV_GD25Q32_OK) { return status; }

    return gd25q32_wait_while_busy(dev, timeout_ms);
}

DRV_GD25Q32_Status DRV_GD25Q32_EraseSector(DRV_GD25Q32_Device *dev, uint32_t address)
{
    return gd25q32_block_erase(dev, GD25Q32_CMD_SECTOR_ERASE, address,
                               DRV_GD25Q32_SECTOR_SIZE, GD25Q32_ERASE_TIMEOUT_MS);
}

DRV_GD25Q32_Status DRV_GD25Q32_EraseBlock32K(DRV_GD25Q32_Device *dev, uint32_t address)
{
    return gd25q32_block_erase(dev, GD25Q32_CMD_BLOCK_ERASE_32K, address,
                               DRV_GD25Q32_BLOCK32K_SIZE,
                               GD25Q32_BLOCK_ERASE_32K_TIMEOUT_MS);
}

DRV_GD25Q32_Status DRV_GD25Q32_EraseBlock64K(DRV_GD25Q32_Device *dev, uint32_t address)
{
    return gd25q32_block_erase(dev, GD25Q32_CMD_BLOCK_ERASE_64K, address,
                               DRV_GD25Q32_BLOCK64K_SIZE,
                               GD25Q32_BLOCK_ERASE_64K_TIMEOUT_MS);
}

DRV_GD25Q32_Status DRV_GD25Q32_PageProgram(DRV_GD25Q32_Device *dev, uint32_t address,
                                       const uint8_t *data, uint16_t length)
{
    DRV_GD25Q32_Status status;
    uint32_t page_remaining;

    if ((dev == NULL) || (data == NULL) || (length == 0U) ||
        (length > DRV_GD25Q32_PAGE_SIZE)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    if (!gd25q32_addr_valid(address, length)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    page_remaining = DRV_GD25Q32_PAGE_SIZE - (address % DRV_GD25Q32_PAGE_SIZE);
    if (length > page_remaining) { return DRV_GD25Q32_INVALID_ARG; }

    if (dev->dma_state != DRV_GD25Q32_DMA_IDLE) {
        return DRV_GD25Q32_BUSY;
    }

    status = gd25q32_wait_while_busy(dev, GD25Q32_DEFAULT_TIMEOUT_MS);
    if (status != DRV_GD25Q32_OK) { return status; }

    status = gd25q32_write_enable(dev);
    if (status != DRV_GD25Q32_OK) { return status; }

    flash_dma_tx[0] = GD25Q32_CMD_PAGE_PROGRAM;
    flash_dma_tx[1] = (uint8_t)(address >> 16U);
    flash_dma_tx[2] = (uint8_t)(address >> 8U);
    flash_dma_tx[3] = (uint8_t)address;
    memcpy(&flash_dma_tx[4], data, length);

    uint16_t dma_len = 4U + length;

    gd25q32_cache_clean(dev, flash_dma_tx, dma_len);

    gd25q32_dma_prepare(dev);
    gd25q32_cs_low(dev);
    HAL_StatusTypeDef hal_status = HAL_SPI_Transmit_DMA(
        dev->bus.hspi, flash_dma_tx, dma_len);
    if (hal_status != HAL_OK) {
        gd25q32_cs_high(dev);
        dma_wait_thread = NULL;
        dev->dma_state = DRV_GD25Q32_DMA_IDLE;
        return gd25q32_from_hal_status(hal_status);
    }

    status = gd25q32_dma_wait(dev, GD25Q32_PROGRAM_TIMEOUT_MS);
    gd25q32_cs_high(dev);

    if (status != DRV_GD25Q32_OK) { return status; }

    return gd25q32_wait_while_busy(dev, GD25Q32_PROGRAM_TIMEOUT_MS);
}

DRV_GD25Q32_Status DRV_GD25Q32_WriteData(DRV_GD25Q32_Device *dev, uint32_t address,
                                     const uint8_t *data, uint32_t length)
{
    uint32_t offset = 0U;

    if ((dev == NULL) || (data == NULL) || (length == 0U)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    if (!gd25q32_addr_valid(address, length)) {
        return DRV_GD25Q32_INVALID_ARG;
    }

    while (offset < length) {
        uint32_t page_remaining = DRV_GD25Q32_PAGE_SIZE -
                                  ((address + offset) % DRV_GD25Q32_PAGE_SIZE);
        uint32_t remaining = length - offset;
        uint16_t chunk = (remaining < page_remaining) ? (uint16_t)remaining
                                                      : (uint16_t)page_remaining;
        DRV_GD25Q32_Status status = DRV_GD25Q32_PageProgram(dev, address + offset,
                                                         &data[offset], chunk);
        if (status != DRV_GD25Q32_OK) { return status; }
        offset += chunk;
    }

    return DRV_GD25Q32_OK;
}
