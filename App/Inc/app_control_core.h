#ifndef APP_CONTROL_CORE_H
#define APP_CONTROL_CORE_H

#include <stdint.h>
#include <stddef.h>
#include "app_flight_calibration.h"
#include "svc_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Common String & Number Parsers ──────────────────────────────────────── */

uint32_t app_control_core_tokenize(char *buffer, char **tokens, uint32_t max_tokens);

const char *app_control_core_token_value(char **tokens, uint32_t count, const char *key);

uint8_t app_control_core_parse_u32(const char *text, uint32_t *value);

uint8_t app_control_core_parse_i32(const char *text, int32_t *value);

uint8_t app_control_core_parse_u32_auto(const char *text, uint32_t *value);

uint8_t app_control_core_token_u32(char **tokens,
                                   uint32_t count,
                                   uint32_t index,
                                   uint32_t default_value,
                                   uint32_t *value);

const char *app_control_core_after_param_separator(const char *text);

uint32_t app_control_core_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len);

uint32_t app_control_core_crc32(const uint8_t *data, uint32_t len);

/* ── Protocol & Text Formatting Output ──────────────────────────────────── */

void app_control_core_queue_proto_text(uint16_t function, const char *format, ...);

/* ── Calibration & Param Core Observer State ────────────────────────────── */

typedef struct {
    APP_FlightCalibration record;
    uint32_t generation;
    uint8_t valid;
    uint8_t dirty;
} AppControlCorePersistedSnapshot;

void app_control_core_get_persisted_snapshot(AppControlCorePersistedSnapshot *snapshot);

const APP_FlightCalibration *app_control_core_get_persisted_record(void);

uint32_t app_control_core_get_persisted_generation(void);

uint8_t app_control_core_is_persisted_valid(void);

uint8_t app_control_core_is_param_dirty(void);

/* ── Calibration Common Safety Check ────────────────────────────────────── */

#define APP_CONTROL_CORE_SNAPSHOT_MAX_AGE_US 1000000ULL
#define APP_CONTROL_CORE_ESC_SAFE_MAX_US     1100U

const char *app_control_core_check_calibration_safety(
    uint8_t require_sequence_progress,
    uint32_t apply_sequence);

#ifdef __cplusplus
}
#endif

#endif /* APP_CONTROL_CORE_H */
