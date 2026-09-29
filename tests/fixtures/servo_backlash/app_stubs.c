/*
 * 宿主装置（策略层 + 命令面）：让 App/Src/app_servo_backlash.c 与 app_cmd_backlash.c 在 gcc 上跑。
 *
 * 只替换平台边界，全部可由测试控制：
 *   - 舵机标定（DRV_COAX_CTRL_GetServoCalibration：中位、端点、pulse_sign）；
 *   - 临界区（空操作，单线程）；
 *   - 解锁 / 辨识占用两个拒绝条件；
 *   - 文本回复（捕获成行）与严格数字解析（照抄 tests/fixtures/rpm_notch/app_stubs.c，
 *     那份又是照抄 app_control_core.c 的语义）。
 * 策略、补偿算法、命令解析一行不替。
 */

#include "app_servo_backlash.h"
#include "app_control_internal.h"
#include "drv_coax_ctrl.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STUB_TEXT_LINES 32
#define STUB_TEXT_BYTES 512

static DRV_COAX_CTRL_ServoCalibration stub_cal;
static uint8_t stub_armed;
static uint8_t stub_sysid;
static uint32_t stub_critical_depth;
static uint32_t stub_critical_max;
static char stub_text[STUB_TEXT_LINES][STUB_TEXT_BYTES];
static uint32_t stub_text_count;
static uint32_t stub_text_longest;

/* ------------------------------------------------------------ 平台边界 */

void DRV_COAX_CTRL_GetServoCalibration(DRV_COAX_CTRL_ServoCalibration *calibration)
{
    *calibration = stub_cal;
}

uint32_t BSP_Critical_Enter(void)
{
    stub_critical_depth++;
    if (stub_critical_depth > stub_critical_max) {
        stub_critical_max = stub_critical_depth;
    }
    return 0U;
}

void BSP_Critical_Exit(uint32_t state)
{
    (void)state;
    stub_critical_depth--;
}

void BSP_Critical_MemoryBarrier(void) {}
uint8_t APP_Stabilizer_IsArmed(void) { return stub_armed; }
uint32_t BSP_PWM_GetServoFrameHz(void) { return 50U; }   /* SYSID BACKLASH 溯源行尾的舵机帧率 */
uint8_t APP_SysId_IsEngaged(void) { return stub_sysid; }

void APP_Control_QueueText(const char *format, ...)
{
    va_list args;
    int written;

    if (stub_text_count >= STUB_TEXT_LINES) {
        return;
    }
    va_start(args, format);
    written = vsnprintf(stub_text[stub_text_count], STUB_TEXT_BYTES, format, args);
    va_end(args);
    if ((written > 0) && ((uint32_t)written > stub_text_longest)) {
        stub_text_longest = (uint32_t)written;
    }
    stub_text_count++;
}

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

/* ------------------------------------------------------------ 测试控制面 */

void stub_set_calibration(uint16_t center_a, uint16_t center_b, uint16_t min_a, uint16_t min_b,
                          uint16_t max_a, uint16_t max_b, int8_t sign_a, int8_t sign_b)
{
    stub_cal.center_us[0] = center_a;
    stub_cal.center_us[1] = center_b;
    stub_cal.min_us[0] = min_a;
    stub_cal.min_us[1] = min_b;
    stub_cal.max_us[0] = max_a;
    stub_cal.max_us[1] = max_b;
    stub_cal.pulse_sign[0] = sign_a;
    stub_cal.pulse_sign[1] = sign_b;
}

void stub_reset(void)
{
    stub_set_calibration(1500U, 1500U, 1000U, 1000U, 2000U, 2000U, -1, -1);
    stub_armed = 0U;
    stub_sysid = 0U;
    stub_critical_depth = 0U;
    stub_critical_max = 0U;
    stub_text_count = 0U;
    stub_text_longest = 0U;
}

void stub_set_armed(uint8_t armed) { stub_armed = armed; }
void stub_set_sysid(uint8_t engaged) { stub_sysid = engaged; }
uint32_t stub_critical_balance(void) { return stub_critical_depth; }
uint32_t stub_critical_nesting(void) { return stub_critical_max; }
void stub_clear_text(void) { stub_text_count = 0U; stub_text_longest = 0U; }
uint32_t stub_text_lines(void) { return stub_text_count; }
const char *stub_text_line(uint32_t index) { return (index < stub_text_count) ? stub_text[index] : ""; }
uint32_t stub_text_max(void) { return stub_text_longest; }

/* 一行命令 → 空格切分 → APP_ServoBacklash_Command。返回处理器的返回值。 */
uint8_t stub_command(const char *line)
{
    static char buffer[256];
    char *tokens[APP_CONTROL_MAX_TOKENS];
    uint32_t count = 0U;
    char *cursor;

    strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    cursor = strtok(buffer, " ");
    while ((cursor != NULL) && (count < (uint32_t)APP_CONTROL_MAX_TOKENS)) {
        tokens[count++] = cursor;
        cursor = strtok(NULL, " ");
    }
    return APP_ServoBacklash_Command(tokens, count);
}

/*
 * 按拍喂：第 i 拍的两路补偿前脉宽 in[2i..]、时刻 ms[i]、来源/解锁/接管，补偿后写 out[2i..]。
 * 与 stabilizer_control_commit 的调用形状一致（原地改两路脉宽）。
 */
void stub_run(uint32_t n, const uint16_t *in, uint16_t *out, const uint32_t *ms,
              const uint8_t *source, const uint8_t *armed, const uint8_t *override)
{
    for (uint32_t i = 0U; i < n; ++i) {
        uint16_t alpha = in[2U * i];
        uint16_t beta = in[(2U * i) + 1U];

        APP_ServoBacklash_Apply(ms[i], (APP_ServoBacklashSource)source[i], armed[i], override[i],
                                &alpha, &beta);
        out[2U * i] = alpha;
        out[(2U * i) + 1U] = beta;
    }
}
