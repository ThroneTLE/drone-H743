#include "svc_param.h"

#include "app_background.h"
#include "app_flash_service.h"

#include <stdbool.h>
#include <string.h>

/*
 * 功能：参数服务实现。
 *
 * 这里保存参数 RAM 镜像，并把参数块以双槽记录格式持久化到外部 FLASH。
 * 读 RAM 参数是同步 API；写 FLASH 属于慢操作，运行期通过后台任务触发。
 */

#define SVC_PARAM_RECORD_MAGIC        0x44524F4EU
#define SVC_PARAM_RECORD_EMPTY        0xFFFFFFFFUL
#define SVC_PARAM_RECORD_WRITING      0xFFFFFFFEUL
#define SVC_PARAM_RECORD_VALID        0xFFFFFFFCUL
#define SVC_PARAM_SECTOR_SIZE         APP_FLASH_SERVICE_SECTOR_SIZE
#define SVC_PARAM_SLOT_A_OFFSET       (APP_FLASH_SERVICE_SIZE_BYTES - 3U * SVC_PARAM_SECTOR_SIZE)
#define SVC_PARAM_SLOT_B_OFFSET       (APP_FLASH_SERVICE_SIZE_BYTES - 2U * SVC_PARAM_SECTOR_SIZE)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t payload_size;
    uint8_t payload[SVC_PARAM_MAX_PAYLOAD];
    uint32_t payload_crc;
    uint32_t header_crc;
    uint32_t state;
} SVC_ParamRecord;

static uint32_t param_crc32_table[256];
static uint8_t param_crc32_initialized;
static uint8_t param_ready;
static uint8_t param_dirty;
static uint32_t param_generation;
static uint32_t param_payload_size;

__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t param_payload[SVC_PARAM_MAX_PAYLOAD];

__attribute__((section(".dma_buffer"), aligned(32)))
static SVC_ParamRecord param_read_record;
__attribute__((section(".dma_buffer"), aligned(32)))
static SVC_ParamRecord param_write_record;
__attribute__((section(".dma_buffer"), aligned(32)))
static SVC_ParamRecord param_verify_record;

static void param_crc32_init(void)
{
    if (param_crc32_initialized != 0U) {
        return;
    }

    for (uint32_t i = 0U; i < 256U; ++i) {
        uint32_t crc = i;
        for (uint32_t j = 0U; j < 8U; ++j) {
            crc = (crc & 1U) ? (0xEDB88320U ^ (crc >> 1U)) : (crc >> 1U);
        }
        param_crc32_table[i] = crc;
    }
    param_crc32_initialized = 1U;
}

static uint32_t param_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;

    param_crc32_init();
    for (uint32_t i = 0U; i < len; ++i) {
        crc = param_crc32_table[(crc ^ data[i]) & 0xFFU] ^ (crc >> 8U);
    }
    return crc ^ 0xFFFFFFFFU;
}

static uint32_t param_record_header_crc(const SVC_ParamRecord *rec)
{
    uint32_t offset = (uint32_t)((uintptr_t)&rec->payload_crc - (uintptr_t)rec);

    return param_crc32((const uint8_t *)rec, offset);
}

static bool param_find_valid_slot(uint32_t *slot_offset,
                                  uint32_t *out_sequence,
                                  uint8_t *out_payload,
                                  uint32_t *out_size)
{
    SVC_ParamRecord *rec = &param_read_record;
    uint32_t slots[2] = { SVC_PARAM_SLOT_A_OFFSET, SVC_PARAM_SLOT_B_OFFSET };
    bool found = false;
    uint32_t best_seq = 0U;
    uint32_t best_offset = 0U;

    for (uint32_t i = 0U; i < 2U; ++i) {
        APP_FlashService_Status st = APP_FlashService_ReadData(slots[i],
                                                               (uint8_t *)rec,
                                                               sizeof(*rec));
        if (st != APP_FLASH_SERVICE_OK) {
            continue;
        }

        if (rec->magic != SVC_PARAM_RECORD_MAGIC) {
            continue;
        }
        if (rec->state != SVC_PARAM_RECORD_VALID) {
            continue;
        }
        if (rec->payload_size > SVC_PARAM_MAX_PAYLOAD) {
            continue;
        }
        if (param_record_header_crc(rec) != rec->header_crc) {
            continue;
        }
        if (param_crc32(rec->payload, rec->payload_size) != rec->payload_crc) {
            continue;
        }

        if (rec->sequence >= best_seq) {
            best_seq = rec->sequence;
            best_offset = slots[i];
            if ((out_payload != NULL) && (out_size != NULL)) {
                *out_size = rec->payload_size;
                memcpy(out_payload, rec->payload, rec->payload_size);
            }
            found = true;
        }
    }

    if (slot_offset != NULL) {
        *slot_offset = best_offset;
    }
    if (out_sequence != NULL) {
        *out_sequence = best_seq;
    }
    return found;
}

void SVC_Param_Init(void)
{
    param_crc32_init();
    memset(param_payload, 0, sizeof(param_payload));
    param_payload_size = 0U;
    param_generation = 0U;
    param_dirty = 0U;
    param_ready = 0U;
}

uint8_t SVC_Param_IsReady(void)
{
    return param_ready;
}

uint8_t SVC_Param_IsDirty(void)
{
    return param_dirty;
}

uint32_t SVC_Param_GetGeneration(void)
{
    return param_generation;
}

SVC_ParamStatus SVC_Param_LoadFromFlash(void)
{
    uint32_t size = 0U;

    if (!param_find_valid_slot(NULL, NULL, param_payload, &size)) {
        param_payload_size = 0U;
        param_ready = 1U;
        param_dirty = 0U;
        param_generation++;
        return SVC_PARAM_STATUS_NO_VALID_RECORD;
    }

    param_payload_size = size;
    param_ready = 1U;
    param_dirty = 0U;
    param_generation++;
    return SVC_PARAM_STATUS_OK;
}

SVC_ParamStatus SVC_Param_GetBlob(uint8_t *data, uint32_t max_size, uint32_t *out_size)
{
    if ((data == NULL) || (out_size == NULL)) {
        return SVC_PARAM_STATUS_INVALID_ARG;
    }
    if (param_ready == 0U) {
        return SVC_PARAM_STATUS_NOT_READY;
    }
    if (max_size < param_payload_size) {
        return SVC_PARAM_STATUS_INVALID_ARG;
    }

    memcpy(data, param_payload, param_payload_size);
    *out_size = param_payload_size;
    return SVC_PARAM_STATUS_OK;
}

SVC_ParamStatus SVC_Param_SetBlob(const uint8_t *data, uint32_t size)
{
    if ((data == NULL) || (size > SVC_PARAM_MAX_PAYLOAD)) {
        return SVC_PARAM_STATUS_INVALID_ARG;
    }

    memcpy(param_payload, data, size);
    param_payload_size = size;
    param_ready = 1U;
    param_dirty = 1U;
    param_generation++;
    return SVC_PARAM_STATUS_OK;
}

uint32_t SVC_Param_RequestSaveBlob(void)
{
    if ((param_ready == 0U) || (param_dirty == 0U)) {
        return 0U;
    }

    return APP_Background_RequestParamSave();
}

SVC_ParamStatus SVC_Param_SavePendingToFlash(void)
{
    SVC_ParamRecord *rec = &param_write_record;
    SVC_ParamRecord *verify = &param_verify_record;
    uint32_t cur_offset = 0U;
    uint32_t cur_seq = 0U;
    uint32_t target_offset;
    uint32_t state_offset;

    if (param_ready == 0U) {
        return SVC_PARAM_STATUS_NOT_READY;
    }
    if (param_dirty == 0U) {
        return SVC_PARAM_STATUS_OK;
    }

    (void)param_find_valid_slot(&cur_offset, &cur_seq, NULL, NULL);
    target_offset = (cur_offset == SVC_PARAM_SLOT_A_OFFSET) ?
                    SVC_PARAM_SLOT_B_OFFSET :
                    SVC_PARAM_SLOT_A_OFFSET;

    APP_FlashService_Status st = APP_FlashService_EraseSector(target_offset);
    if (st != APP_FLASH_SERVICE_OK) {
        return SVC_PARAM_STATUS_ERROR;
    }

    memset(rec, 0xFF, sizeof(*rec));
    rec->magic = SVC_PARAM_RECORD_MAGIC;
    rec->version = 1U;
    rec->sequence = cur_seq + 1U;
    rec->payload_size = param_payload_size;
    memcpy(rec->payload, param_payload, param_payload_size);
    rec->payload_crc = param_crc32(rec->payload, rec->payload_size);
    rec->state = SVC_PARAM_RECORD_WRITING;
    rec->header_crc = param_record_header_crc(rec);

    st = APP_FlashService_WriteData(target_offset, (const uint8_t *)rec, sizeof(*rec));
    if (st != APP_FLASH_SERVICE_OK) {
        return SVC_PARAM_STATUS_ERROR;
    }

    st = APP_FlashService_ReadData(target_offset, (uint8_t *)verify, sizeof(*verify));
    if (st != APP_FLASH_SERVICE_OK) {
        return SVC_PARAM_STATUS_ERROR;
    }
    if (memcmp(rec, verify, sizeof(*rec)) != 0) {
        return SVC_PARAM_STATUS_ERROR;
    }

    rec->state = SVC_PARAM_RECORD_VALID;
    state_offset = (uint32_t)((uintptr_t)&rec->state - (uintptr_t)rec);
    st = APP_FlashService_WriteData(target_offset + state_offset,
                                    (const uint8_t *)&rec->state,
                                    sizeof(rec->state));
    if (st != APP_FLASH_SERVICE_OK) {
        return SVC_PARAM_STATUS_ERROR;
    }

    param_dirty = 0U;
    return SVC_PARAM_STATUS_OK;
}
