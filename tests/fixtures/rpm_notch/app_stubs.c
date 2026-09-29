/*
 * 宿主装置（策略层 + 命令面）：让 App/Src/app_rpm_notch.c 与 app_cmd_rpmnotch.c 在 gcc 上跑。
 *
 * 只替换平台边界，全部可由测试控制：
 *   - 电调回传快照（BSP_DShotRx_GetSnapshot）、陀螺名义 ODR（BSP_IMU_GetGyroOdrHz）；
 *   - 时钟（SVC_Timestamp_Ms/Us）、临界区（空操作，单线程）、内存屏障（可在第 N 次
 *     插一个控制拍，模拟读者拷副本时被切走）；
 *   - 解锁 / 辨识占用两个拒绝条件；
 *   - 文本回复（捕获成行）与严格数字解析（照抄 tests/fixtures/sysid/cmd_harness.c，
 *     那份又是逐字照抄 app_control_core.c 的语义）。
 * 策略、滤波、命令解析一行不替。
 */

#include "app_rpm_notch.h"
#include "app_control_internal.h"
#include "bsp_dshot_rx.h"
#include "bsp_imu_rate.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define STUB_TEXT_LINES 64
#define STUB_TEXT_BYTES 512

static BSP_DShotRxSnapshot stub_rx;
static uint16_t stub_odr_hz = 1000U;
static uint32_t stub_ms;
static uint64_t stub_us;
static uint32_t stub_us_step;      /* 每次读 Us() 后自动前进（给耗时统计一个非零值） */
static uint8_t stub_armed;
static uint8_t stub_sysid;
static char stub_text[STUB_TEXT_LINES][STUB_TEXT_BYTES];
static uint32_t stub_text_count;
static uint32_t stub_text_longest;

/* ------------------------------------------------------------ 平台边界 */

void BSP_DShotRx_GetSnapshot(BSP_DShotRxSnapshot *out) { *out = stub_rx; }
uint16_t BSP_IMU_GetGyroOdrHz(void) { return stub_odr_hz; }
uint32_t SVC_Timestamp_Ms(void) { return stub_ms; }
uint64_t SVC_Timestamp_Us(void)
{
    const uint64_t now = stub_us;

    stub_us += stub_us_step;
    return now;
}
uint32_t BSP_Critical_Enter(void) { return 0U; }
void BSP_Critical_Exit(uint32_t state) { (void)state; }

/*
 * 屏障是读者"读序号 → 拷 → 再读序号"之间唯一的外部调用点，拿它模拟时间片切换：
 * 倒数到 0 的那一次屏障里插一个完整的控制拍（发布一次）。Tick 自己也过屏障，
 * 倒数先清零再调，不会递归。
 */
static uint32_t stub_barrier_countdown;

void BSP_Critical_MemoryBarrier(void)
{
    if (stub_barrier_countdown != 0U) {
        stub_barrier_countdown--;
        if (stub_barrier_countdown == 0U) {
            APP_RpmNotch_Tick();
        }
    }
}
void stub_tick_at_barrier(uint32_t nth) { stub_barrier_countdown = nth; }
uint8_t APP_Stabilizer_IsArmed(void) { return stub_armed; }
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

/* ------------------------------------------------------------ 测试控制面 */

void stub_reset(void)
{
    memset(&stub_rx, 0, sizeof(stub_rx));
    stub_odr_hz = 1000U;
    stub_ms = 100000U;
    stub_us = 100000000ULL;
    stub_us_step = 0U;
    stub_armed = 0U;
    stub_sysid = 0U;
    stub_barrier_countdown = 0U;
    stub_text_count = 0U;
    stub_text_longest = 0U;
}

void stub_set_available(uint8_t available) { stub_rx.available = available; }
void stub_set_rotor(uint32_t index, uint32_t erpm, uint32_t sample_ms, uint8_t valid,
                    uint8_t not_spinning)
{
    stub_rx.erpm[index] = erpm;
    stub_rx.sample_ms[index] = sample_ms;
    stub_rx.valid[index] = valid;
    stub_rx.not_spinning[index] = not_spinning;
}
void stub_set_odr(uint16_t hz) { stub_odr_hz = hz; }
void stub_set_ms(uint32_t ms) { stub_ms = ms; }
uint32_t stub_get_ms(void) { return stub_ms; }
void stub_set_us(uint64_t us) { stub_us = us; }
void stub_set_us_step(uint32_t step) { stub_us_step = step; }
void stub_set_armed(uint8_t armed) { stub_armed = armed; }
void stub_set_sysid(uint8_t engaged) { stub_sysid = engaged; }
void stub_clear_text(void) { stub_text_count = 0U; stub_text_longest = 0U; }
uint32_t stub_text_lines(void) { return stub_text_count; }
const char *stub_text_line(uint32_t index) { return (index < stub_text_count) ? stub_text[index] : ""; }
uint32_t stub_text_max(void) { return stub_text_longest; }

/* 一行命令 → 空格切分 → APP_RpmNotch_Command。返回处理器的返回值。 */
uint8_t stub_command(const char *line)
{
    static char buffer[256];
    char *tokens[16];
    uint32_t count = 0U;
    char *cursor;

    strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    cursor = strtok(buffer, " ");
    while ((cursor != NULL) && (count < (uint32_t)(sizeof(tokens) / sizeof(tokens[0])))) {
        tokens[count++] = cursor;
        cursor = strtok(NULL, " ");
    }
    return APP_RpmNotch_Command(tokens, count);
}

/*
 * 批量喂样本：第 i 个样本时间戳 ts[i]，输入 in[3i..]，输出 out[3i..]；tick[i] 非 0 时在该样本
 * 之后调一次 Tick（与 StabilizerTask 的顺序一致：先处理样本，再跑控制拍）。
 * ms[i] 是那一拍的 HAL 毫秒（电调回包的 sample_ms 由 rx_ms 给，fresh 由调用方控制）。
 * erpm 非空时每个 tick 前把两路 eRPM 与 sample_ms（= ms[i]）写进快照。
 */
void stub_run(uint32_t n, const uint64_t *ts, const float *in, float *out, const uint8_t *tick,
              const uint32_t *ms, const uint32_t *erpm)
{
    for (uint32_t i = 0U; i < n; ++i) {
        stub_ms = ms[i];
        APP_RpmNotch_ApplySample(&in[3U * i], &out[3U * i], ts[i]);
        if (tick[i] != 0U) {
            if (erpm != NULL) {
                for (uint32_t m = 0U; m < 2U; ++m) {
                    stub_rx.erpm[m] = erpm[(2U * i) + m];
                    stub_rx.sample_ms[m] = ms[i];
                }
            }
            stub_us = ts[i] + 300U;
            APP_RpmNotch_Tick();
        }
    }
}
