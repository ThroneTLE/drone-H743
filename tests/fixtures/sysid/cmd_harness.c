/*
 * 宿主装置（命令面）：让 App/Src/app_cmd_sysid.c 的 `SYSID ...` 解析能在 gcc 上跑。
 *
 * 与 app_harness.c 一起链接。这里只补命令面依赖的这几类东西：
 *   - 令牌解析小工具：逐字照抄 App/Src/app_control_core.c / app_control.c 的严格解析
 *     （整串必须吃完、浮点必须有限）。那两个文件拖着 USB/UART/任务头，搬不上宿主；
 *     照抄的语义一旦和原件不一致，命令面测试就会在"原件会拒、这里放行"处漏判，
 *     所以只抄、不改；
 *   - 老 IDENT 族的入口：命令面测试不走这些分支，给空桩。
 *
 * `SYSID PARAM` 的写入走真的 App/Src/app_param_trial.c（链接时一并加进来）与真的
 * DRV_COAX_CTRL_GetParam/SetParam，这里不替。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_ident.h"
#include "app_rpm_notch.h"
#include "app_servo_backlash.h"
#include "app_sysid.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

uint8_t app_control_handle_sysid(char **tokens, uint32_t count);

uint8_t app_control_parse_u32(const char *text, uint32_t *value)
{
    char *end_ptr;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }
    parsed = strtoul(text, &end_ptr, 10);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

uint8_t app_control_parse_i32(const char *text, int32_t *value)
{
    char *end_ptr;
    long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }
    parsed = strtol(text, &end_ptr, 10);
    if ((end_ptr == text) || (*end_ptr != '\0')) {
        return 0U;
    }
    *value = (int32_t)parsed;
    return 1U;
}

uint8_t app_control_parse_f32(const char *text, float *value)
{
    char *end_ptr;
    float parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) {
        return 0U;
    }
    parsed = strtof(text, &end_ptr);
    if ((end_ptr == text) || (*end_ptr != '\0') || !isfinite(parsed)) {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

const char *app_control_token_value(char **tokens, uint32_t count, const char *key)
{
    size_t key_len;

    if ((tokens == NULL) || (key == NULL)) {
        return NULL;
    }
    key_len = strlen(key);
    for (uint32_t index = 1U; index < count; ++index) {
        if ((strncmp(tokens[index], key, key_len) == 0) &&
            (tokens[index][key_len] == '=')) {
            return &tokens[index][key_len + 1U];
        }
    }
    return NULL;
}

/*
 * 转速陷波的溯源行：真实现在 app_cmd_rpmnotch.c（它拖着陷波策略层）。这里只回一行同名前缀，
 * 让命令面测试看得到"开跑成功才报、带本轮 run"。
 */
void APP_RpmNotch_ReportProvenance(uint16_t run_id)
{
    APP_Control_QueueText("SYSID NOTCH run=%u en=0 state=off src=stub\r\n", (unsigned int)run_id);
}

/* 舵机回差补偿的溯源行：真实现在 app_cmd_backlash.c（它拖着补偿策略层），理由同上。 */
void APP_ServoBacklash_ReportProvenance(uint16_t run_id)
{
    APP_Control_QueueText("SYSID BACKLASH run=%u en=0 alpha_mrad=20 beta_mrad=20 thr_mrad=5\r\n",
                          (unsigned int)run_id);
}

/* 老 IDENT 族：命令面测试只走 SYSID，这些分支给空桩。 */
void APP_Ident_ReportStatus(void) {}
uint8_t APP_Ident_Arm(void) { return 0U; }
void APP_Ident_Disarm(void) {}
void APP_Ident_Stop(const char *reason) { (void)reason; }
uint8_t APP_Ident_SetCenter(uint16_t alpha_us, uint16_t beta_us)
{ (void)alpha_us; (void)beta_us; return 0U; }
uint8_t APP_Ident_StartStep(const char *axis, int32_t pulse_us, uint32_t duration_ms)
{ (void)axis; (void)pulse_us; (void)duration_ms; return 0U; }
uint8_t APP_Ident_StartDoublet(const char *axis, int32_t pulse_us, uint32_t hold_ms,
                               uint32_t repeat)
{ (void)axis; (void)pulse_us; (void)hold_ms; (void)repeat; return 0U; }
uint8_t APP_Ident_StartPrbs(const char *axis, int32_t pulse_us, uint32_t bit_ms,
                            uint32_t duration_ms, uint32_t seed)
{ (void)axis; (void)pulse_us; (void)bit_ms; (void)duration_ms; (void)seed; return 0U; }
uint8_t APP_Ident_ApplyRateKp(const char *axis, const char *value_text)
{ (void)axis; (void)value_text; return 0U; }
uint8_t APP_IdentAtt_StartPrbs(const char *axis, int32_t amp_mdeg, uint32_t bit_ms,
                               uint32_t duration_ms, uint32_t seed)
{ (void)axis; (void)amp_mdeg; (void)bit_ms; (void)duration_ms; (void)seed; return 0U; }
void APP_IdentAtt_Stop(const char *reason) { (void)reason; }

/* 一行命令 → 空格切分 → app_control_handle_sysid。返回处理器的返回值。 */
uint8_t harness_command(const char *line)
{
    static char buffer[256];
    char *tokens[24];
    uint32_t count = 0U;
    char *cursor;

    if (line == NULL) {
        return 0U;
    }
    strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    cursor = strtok(buffer, " ");
    while ((cursor != NULL) && (count < (uint32_t)(sizeof(tokens) / sizeof(tokens[0])))) {
        tokens[count++] = cursor;
        cursor = strtok(NULL, " ");
    }
    return app_control_handle_sysid(tokens, count);
}
