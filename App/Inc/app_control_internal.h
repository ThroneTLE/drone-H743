#ifndef APP_CONTROL_INTERNAL_H
#define APP_CONTROL_INTERNAL_H

#include <stdint.h>

/* Keep the synchronous USB text budget byte-for-byte aligned with the legacy path. */
#define APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS 10U

uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens);
uint8_t app_control_parse_u32(const char *text, uint32_t *value);
uint8_t app_control_parse_i32(const char *text, int32_t *value);
const char *app_control_token_value(char **tokens,
                                    uint32_t count,
                                    const char *key);
uint32_t app_control_crc32_update(uint32_t crc,
                                  const uint8_t *data,
                                  uint32_t len);
uint32_t app_control_crc32(const uint8_t *data, uint32_t len);
void app_control_queue_proto_text(uint16_t function, const char *format, ...);

/* Narrow bridge for the one legacy transport-mode flag read by QueueProtoText. */
uint8_t app_control_internal_maint_output_active(void);

/* Read-only calibration mirrors and pure delegates for extracted command domains. */
const void *app_control_internal_imucal_confirmed_record(void);
uint32_t app_control_internal_imucal_confirmed_generation(void);
uint8_t app_control_internal_imucal_confirmed_valid(void);
void app_control_internal_imuframe_sync_param(void);
const char *app_control_internal_imucal_safety(
    void *snapshot,
    uint8_t require_sequence_progress);
uint8_t app_control_internal_imucal_upload_state(void);
uint8_t app_control_internal_imucal_applied(void);
uint8_t app_control_internal_imucal_commit_pending(void);
uint8_t app_control_internal_imuframe_confirmed_code(void);

#endif /* APP_CONTROL_INTERNAL_H */
