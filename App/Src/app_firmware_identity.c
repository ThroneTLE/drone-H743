#include "app_firmware_identity.h"

#include <stddef.h>

/* End of all Flash loadable bytes, provided by STM32H743XX_FLASH.ld. */
extern const uint8_t __data_source_end;

static APP_FirmwareIdentity app_firmware_identity_cached;
static uint8_t app_firmware_identity_ready;

uint8_t APP_FirmwareIdentity_IsRangeValid(uintptr_t start, uintptr_t end)
{
    return ((start == (uintptr_t)APP_FIRMWARE_FLASH_START) &&
            (end > start) &&
            (end <= (uintptr_t)APP_FIRMWARE_FLASH_LIMIT) &&
            ((end - start) <= UINT32_MAX)) ? 1U : 0U;
}

uint32_t APP_FirmwareIdentity_ComputeCrc32(const uint8_t *data,
                                           uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint32_t bit;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }
    for (index = 0U; index < length; ++index) {
        crc ^= (uint32_t)data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 1UL) != 0UL) ?
                  ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
}

uint8_t APP_FirmwareIdentity_Get(APP_FirmwareIdentity *identity)
{
    const uintptr_t start = (uintptr_t)APP_FIRMWARE_FLASH_START;
    const uintptr_t end = (uintptr_t)&__data_source_end;

    if (identity == NULL) {
        return 0U;
    }
    if (app_firmware_identity_ready == 0U) {
        if (APP_FirmwareIdentity_IsRangeValid(start, end) == 0U) {
            return 0U;
        }
        app_firmware_identity_cached.image_size = (uint32_t)(end - start);
        app_firmware_identity_cached.image_crc32 =
            APP_FirmwareIdentity_ComputeCrc32((const uint8_t *)start,
                                              app_firmware_identity_cached.image_size);
        app_firmware_identity_ready = 1U;
    }
    *identity = app_firmware_identity_cached;
    return 1U;
}

uint32_t APP_FirmwareIdentity_GetCrc32(void)
{
    APP_FirmwareIdentity identity;

    return (APP_FirmwareIdentity_Get(&identity) != 0U) ?
           identity.image_crc32 : 0U;
}
