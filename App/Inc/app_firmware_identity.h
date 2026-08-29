#ifndef APP_FIRMWARE_IDENTITY_H
#define APP_FIRMWARE_IDENTITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_FIRMWARE_FLASH_START 0x08000000UL
#define APP_FIRMWARE_FLASH_LIMIT 0x08200000UL

typedef struct {
    uint32_t image_crc32;
    uint32_t image_size;
} APP_FirmwareIdentity;

/* Pure helpers are public so host tests can verify the exact identity rule. */
uint8_t APP_FirmwareIdentity_IsRangeValid(uintptr_t start, uintptr_t end);
uint32_t APP_FirmwareIdentity_ComputeCrc32(const uint8_t *data,
                                           uint32_t length);

/*
 * CRC covers the linked load image [0x08000000, __data_source_end). The first
 * successful calculation is cached because immutable internal Flash cannot
 * change while this image is executing.
 */
uint8_t APP_FirmwareIdentity_Get(APP_FirmwareIdentity *identity);
uint32_t APP_FirmwareIdentity_GetCrc32(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FIRMWARE_IDENTITY_H */
