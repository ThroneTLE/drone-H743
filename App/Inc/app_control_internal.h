#ifndef APP_CONTROL_INTERNAL_H
#define APP_CONTROL_INTERNAL_H

#include <stdint.h>

/* Keep the synchronous USB text budget byte-for-byte aligned with the legacy path. */
#define APP_CONTROL_USB_TEXT_TX_TIMEOUT_MS 10U

uint32_t app_control_tokenize(char *buffer, char **tokens, uint32_t max_tokens);
/*
 * 一行命令最多多少段（含命令字）。原为 10：SYSID EXC 有 13 段，末尾的 bit_ms/seed/
 * servo_tilt_mrad 被静默丢弃，舵机单独改摆幅后回读永远不符（2026-09-27 实机）。
 */
#define APP_CONTROL_MAX_TOKENS 24U
void APP_Control_QueueText(const char *format, ...);   /* 同 app_control.h；拆行报错要用 */
/*
 * 复制一行进 buffer 并拆段。行比 buffer 长、或段数超过 max_tokens − 1 时回
 * "ERR line too long" / "ERR too many tokens" 并返回 0：宁可整行拒绝，不静默截断。
 * tokens 数组要比允许的段数多一格（用来判断是否超了）。
 */
uint32_t app_control_split_line(const char *line, char *buffer, uint32_t buffer_size,
                                char **tokens, uint32_t max_tokens);
uint8_t app_control_parse_u32(const char *text, uint32_t *value);
uint8_t app_control_parse_i32(const char *text, int32_t *value);
/* 严格浮点解析：整串必须被吃完，且结果有限。"1.2abc" 与 "inf" 都判失败。 */
uint8_t app_control_parse_f32(const char *text, float *value);
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
 * 参数名路由（App/Src/app_cmd_airframe.c）：coax.* 与 airframe.* 两张表。
 * app_control.c 里的调用点做等量替换即可，不新增行。
 */
uint32_t app_control_param_count_any(void);
const char *app_control_param_name_any(uint32_t index);
uint8_t app_control_param_get_any(const char *name, float *value);
uint8_t app_control_param_set_any(const char *name, float value);
void app_control_report_airframe_record(void);

/*
 * 命令兜底链（App/Src/app_cmd_fallback.c）。`app_control.c` 的 if-else 全都没
 * 认领时落到这里；新命令族挂在它的链上，不再往 app_control.c 加分支（只减不增）。
 */
void app_control_handle_unclaimed(char **tokens, uint32_t count);

/* LED 命令族（App/Src/app_cmd_led.c）：认领返回 1，否则 0。 */
uint8_t app_control_handle_led(char **tokens, uint32_t count);

/*
 * LEDMAP 命令族（App/Src/app_cmd_ledmap.c）：状态灯颜色绑定表的读写。
 * 后两个是给 app_control_config_store.c 存取 CFG 记录里那一块用的。
 */
uint8_t app_control_handle_ledmap(char **tokens, uint32_t count);
void app_cmd_ledmap_apply_config(const void *config);
const void *app_cmd_ledmap_config(void);

/*
 * PROPCAL 命令族（App/Src/app_cmd_propcal.c）：桨叶与电机接线标定（偏航极性的
 * 唯一来源），外加受心跳保护的点电机窗口。后两个是给 app_control_config_store.c
 * 存取 CFG 记录里那一块用的。
 */
uint8_t app_control_handle_propcal(char **tokens, uint32_t count);
uint8_t app_control_handle_thrust_bench(char **tokens, uint32_t count);
void app_cmd_propcal_apply_config(const void *config);
const void *app_cmd_propcal_config(void);

/*
 * MAGCAL / MAGFRAME 命令族（App/Src/app_cmd_magcal.c）：磁力计硬磁/软磁校准
 * 与轴向验证。一个函数认领两个命令字（认领返回 1，否则 0）。后两个是给
 * app_control_config_store.c 存取 CFG 记录里那一块用的。
 */
uint8_t app_control_handle_magcal(char **tokens, uint32_t count);

/*
 * `CURRENT PROBE`：电流输入脚接线自检（见 App/Src/app_cmd_currprobe.c）。
 * 解锁状态下拒绝执行；自检期间电流采样暂停若干拍。
 */
uint8_t app_control_handle_currprobe(char **tokens, uint32_t count);

/*
 * `ESC EDT ON|OFF` / `ESC ?`：打开电调扩展遥测（见 App/Src/app_cmd_esc_edt.c）。
 * 逐路电流属于 EDT，而 EDT 只能由飞控发 DShot 特殊命令打开。解锁状态下拒绝。
 * 回包说的是"发出去了"，不是"生效了"——DShot 不回 ACK，生效只能看收到没收到 EDT 帧。
 */
uint8_t app_control_handle_esc_edt(char **tokens, uint32_t count);
uint8_t app_control_handle_esc_kv(char **tokens, uint32_t count);
uint8_t app_control_handle_thrust_lut(char **tokens, uint32_t count);
uint8_t app_control_handle_esc_edt_set(char **tokens, uint32_t count);
void app_cmd_magcal_apply_config(const void *config);
const void *app_cmd_magcal_config(void);

/*
 * SYSID 命令族（App/Src/app_cmd_sysid.c）：光杆台架上的内环辨识。
 *
 * 第二个是从 app_control.c 搬出来的 `IDENT` 那一组（START 除外，它耦合
 * app_control.c 的 ident_* 静态量与服务拍）。认领返回 1。
 */
uint8_t app_control_handle_sysid(char **tokens, uint32_t count);
uint8_t app_cmd_sysid_handle_ident(char **tokens, uint32_t count);

/* ARM 命令族（App/Src/app_cmd_arm.c）：解锁状态与被拒原因。 */
void app_control_report_arm(void);
uint8_t app_control_req_arm(uint32_t id, const char *mod, const char *op);

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
void app_control_report_pwm(void);
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
