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
uint32_t *app_control_internal_imucal_confirmed_generation_slot(void);
uint32_t *app_control_internal_imucal_apply_sequence_slot(void);

void app_control_handle_imucal(char **tokens, uint32_t count);
void app_control_service_imucal(void);
void app_cmd_imucal_clear_candidate(void);
void app_cmd_imucal_set_event(const char *event, const char *reason);
void *app_cmd_imucal_upload_slot(void);
void *app_cmd_imucal_preview_slot(void);
void *app_cmd_imucal_pending_record_slot(void);
uint32_t *app_cmd_imucal_preview_generation_slot(void);
uint32_t *app_cmd_imucal_last_request_slot(void);
uint8_t *app_cmd_imucal_applied_slot(void);
uint8_t *app_cmd_imucal_commit_pending_slot(void);
const char **app_cmd_imucal_last_event_slot(void);
const char **app_cmd_imucal_last_reason_slot(void);

void app_control_handle_servocal(char **tokens, uint32_t count);
void app_control_service_servocal(void);
void app_cmd_servocal_init(void);
void app_cmd_servocal_on_persisted(const void *record);
uint8_t app_cmd_servocal_is_busy(void);

void app_cmd_servotype_init(void);
void app_cmd_servotype_on_persisted(const void *record);
void app_control_handle_servotype(char **tokens, uint32_t count);
void app_control_service_servotype(void);

void app_control_report_rc_live(void);
void app_control_handle_rc_map(char *tokens[], uint32_t count);
void app_cmd_rcmap_apply_config(const void *config);
const void *app_cmd_rcmap_config(void);
uint8_t app_control_internal_commit_config_persist(void);

int32_t app_control_internal_acceptance_milli(float value);
void app_control_handle_flow(char **tokens, uint32_t count);
void app_control_report_flow(void);

/* TELEM 命令族（App/Src/app_cmd_telem.c）。 */
void app_control_handle_telem(char **tokens, uint32_t count);

/*
 * 通用探针命令族（App/Src/app_cmd_probe.c）：MEM / SPI / I2C / UART。
 * 认领了这个 mod 返回 1，否则返回 0 让调用方继续往下匹配。
 */
uint8_t app_control_req_probe(uint32_t id, const char *mod, const char *op,
                              char **tokens, uint32_t count);

/* IMUSEL 命令族（App/Src/app_cmd_imusel.c）。 */
void app_control_req_imusel(uint32_t id, const char *op);

void app_control_report_caps(void);
void app_control_report_wifi(void);
void app_control_report_rtos(void);
void app_control_report_modules(void);
void app_control_report_status(void);
void app_control_handle_req(char **tokens, uint32_t count);

const void *app_control_internal_config_view(void);
const char *app_control_internal_imu_stage_name(uint8_t stage);
uint8_t app_control_internal_flash_ok(const void *status);
const char *app_control_internal_flash_stage(const void *status);
uint8_t app_control_internal_baro_ok(const void *status);
const char *app_control_internal_baro_stage(const void *status);
const char *app_control_internal_aiwb2_state_name(uint32_t state);
uint8_t app_control_internal_parse_u32_auto(const char *text, uint32_t *value);

#endif /* APP_CONTROL_INTERNAL_H */
