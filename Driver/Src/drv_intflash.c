/*
 * STM32H743 片内 Flash 驱动实现。
 * 与 NOR 的差异和调用约束写在 drv_intflash.h，此处只讲实现上的取舍。
 */

#include "drv_intflash.h"

#include <string.h>

static DRV_INTFLASH_Bus intflash_bus;

void DRV_INTFLASH_SetBus(const DRV_INTFLASH_Bus *bus)
{
    if (bus != NULL) {
        intflash_bus = *bus;
    } else {
        memset(&intflash_bus, 0, sizeof(intflash_bus));
    }
}

static void intflash_invalidate(uint32_t address, uint32_t length)
{
    if (intflash_bus.cache_invalidate != NULL) {
        intflash_bus.cache_invalidate((const void *)(uintptr_t)address, length);
    }
}

uint8_t DRV_INTFLASH_IsParamAddress(uint32_t address)
{
    return ((address >= DRV_INTFLASH_PARAM_BASE) &&
            (address < (DRV_INTFLASH_PARAM_BASE + DRV_INTFLASH_PARAM_SIZE)))
           ? 1U : 0U;
}

static uint8_t intflash_sector_of(uint32_t address, uint32_t *sector_base)
{
    const uint32_t index = (address - DRV_INTFLASH_PARAM_BASE) /
                           DRV_INTFLASH_SECTOR_SIZE;

    if (sector_base != NULL) {
        *sector_base = DRV_INTFLASH_PARAM_SECTOR_ADDR(index);
    }
    return (uint8_t)(DRV_INTFLASH_PARAM_FIRST_SECTOR + index);
}

DRV_INTFLASH_Status DRV_INTFLASH_EraseSector(uint32_t address)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0xFFFFFFFFUL;
    uint32_t sector_base = 0U;
    uint8_t sector;
    HAL_StatusTypeDef hal_status;

    if (DRV_INTFLASH_IsParamAddress(address) == 0U) {
        return DRV_INTFLASH_INVALID_ARG;
    }

    sector = intflash_sector_of(address, &sector_base);

    /*
     * 只接受扇区起始地址。擦除是 128 KB 粒度的破坏性操作，如果允许"扇区内任意地址"
     * 就会出现调用者以为只擦了 4 KB、实际抹掉整个槽的情况。宁可返回参数错误。
     */
    if (address != sector_base) {
        return DRV_INTFLASH_ALIGN_ERROR;
    }

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = DRV_INTFLASH_PARAM_BANK;
    erase.Sector       = sector;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return DRV_INTFLASH_ERROR;
    }

    /* 残留的错误标志会让本次操作直接失败，先清掉再动手。 */
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);

    hal_status = HAL_FLASHEx_Erase(&erase, &sector_error);
    (void)HAL_FLASH_Lock();

    if ((hal_status != HAL_OK) || (sector_error != 0xFFFFFFFFUL)) {
        return DRV_INTFLASH_ERROR;
    }

    intflash_invalidate(sector_base, DRV_INTFLASH_SECTOR_SIZE);
    return DRV_INTFLASH_OK;
}

DRV_INTFLASH_Status DRV_INTFLASH_Write(uint32_t address, const uint8_t *data,
                                       uint32_t length)
{
    uint32_t written = 0U;
    HAL_StatusTypeDef hal_status = HAL_OK;

    if ((data == NULL) || (length == 0U)) { return DRV_INTFLASH_INVALID_ARG; }
    if (DRV_INTFLASH_IsParamAddress(address) == 0U) {
        return DRV_INTFLASH_INVALID_ARG;
    }
    if ((address % DRV_INTFLASH_WORD_SIZE) != 0U) {
        return DRV_INTFLASH_ALIGN_ERROR;
    }
    if (DRV_INTFLASH_IsParamAddress(address + length - 1U) == 0U) {
        return DRV_INTFLASH_INVALID_ARG;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return DRV_INTFLASH_ERROR;
    }
    __HAL_FLASH_CLEAR_FLAG_BANK2(FLASH_FLAG_ALL_ERRORS_BANK2);

    while (written < length) {
        /*
         * 每次必须整字（32 字节）编程。尾部不足一整字时补 0xFF——
         * 0xFF 是擦除态，补它等于"这几个字节没写过"，不会破坏后续数据；
         * 补 0x00 则会把那些位永久拉低，且无法再写。
         */
        uint8_t word_buf[DRV_INTFLASH_WORD_SIZE] __attribute__((aligned(4)));
        uint32_t chunk = length - written;

        if (chunk > DRV_INTFLASH_WORD_SIZE) { chunk = DRV_INTFLASH_WORD_SIZE; }

        memset(word_buf, 0xFF, sizeof(word_buf));
        memcpy(word_buf, &data[written], chunk);

        hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                       (uint32_t)(address + written),
                                       (uint32_t)(uintptr_t)word_buf);
        if (hal_status != HAL_OK) { break; }

        written += DRV_INTFLASH_WORD_SIZE;
    }

    (void)HAL_FLASH_Lock();

    if (hal_status != HAL_OK) { return DRV_INTFLASH_ERROR; }

    intflash_invalidate(address, written);
    return DRV_INTFLASH_OK;
}

DRV_INTFLASH_Status DRV_INTFLASH_Read(uint32_t address, uint8_t *data,
                                      uint32_t length)
{
    if ((data == NULL) || (length == 0U)) { return DRV_INTFLASH_INVALID_ARG; }
    if (DRV_INTFLASH_IsParamAddress(address) == 0U) {
        return DRV_INTFLASH_INVALID_ARG;
    }
    if (DRV_INTFLASH_IsParamAddress(address + length - 1U) == 0U) {
        return DRV_INTFLASH_INVALID_ARG;
    }

    /* 片内 Flash 可直接寻址；先失效缓存，否则可能读到擦/写前的旧内容。 */
    intflash_invalidate(address, length);
    memcpy(data, (const void *)(uintptr_t)address, length);

    return DRV_INTFLASH_OK;
}
