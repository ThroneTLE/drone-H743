#include "app_current.h"
#include "app_current_format.h"
#include "drv_current_filter.h"
#include "app_control.h"
#include "bsp_current.h"
#include "bsp_critical.h"
#include "svc_timestamp.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CURRENT_STALE_MS 250U

/*
 * 一个块 25 个有效样本 = 20 ms × 25 = 0.5 s。
 *
 * 取舍：块越长均值越稳（纹波按 1/sqrt(N) 收敛），但读数更新越慢。0.5 s 对
 * "看一眼耗了多少电"和"跟外部电流表对拍"都够用，对"抓瞬时峰值"不够——后者要
 * 的是 min/max，那两个值本来就一起报出来了，不靠均值去抓。
 */
#define APP_CURRENT_FILTER_WINDOW 25U

static DRV_CurrentConfig current_config;
static DRV_CurrentFilter current_filter;
static APP_CurrentSnapshot current_snapshot = {.adc_status=BSP_CURRENT_NOT_READY};
static uint32_t last_attempt_ms;
static uint8_t ready;
static uint8_t has_sample;

void APP_Current_Init(void)
{
    current_config = DRV_Current_Am32_55A_Default();
    DRV_CurrentFilter_Init(&current_filter, APP_CURRENT_FILTER_WINDOW);
    memset(&current_snapshot, 0, sizeof(current_snapshot));
    current_snapshot.reading.adc_v = NAN;
    current_snapshot.reading.current_a = NAN;
    current_snapshot.adc_status = (uint8_t)BSP_Current_Init();
    ready = current_snapshot.adc_status == BSP_CURRENT_OK;
    has_sample = 0U;
    current_snapshot.errors = ready ? 0U : 1U;
    last_attempt_ms = SVC_Timestamp_Ms() - APP_CURRENT_PERIOD_MS;
}

void APP_Current_Step(void)
{
    uint32_t now = SVC_Timestamp_Ms();
    if (!ready || (uint32_t)(now - last_attempt_ms) < APP_CURRENT_PERIOD_MS) { return; }
    last_attempt_ms = now;
    uint32_t raw;
    BSP_CurrentStatus status = BSP_Current_Read(&raw);
    DRV_CurrentReading reading;
    DRV_CurrentStatus converted = DRV_CURRENT_INVALID;
    if (status == BSP_CURRENT_OK) { converted = DRV_Current_Convert(&current_config, raw, &reading); }
    /* Timestamp the start of the pair. A preempted conversion must not make an
     * old current sample appear fresh merely because rank 2 finished later. */
    uint32_t sampled_ms = now;
    uint32_t lock = BSP_Critical_Enter();
    if (status == BSP_CURRENT_OK && converted != DRV_CURRENT_INVALID) {
        current_snapshot.reading = reading;
        current_snapshot.sample_ms = sampled_ms;
        current_snapshot.samples++;
        current_snapshot.adc_status = BSP_CURRENT_OK;
        has_sample = 1U;
        /*
         * 只把**这一拍确实有效**的读数送进平均。饱和样本的 current_a 已经是
         * NaN（DRV_Current_Convert 的约定），滤波器会自己拒掉它，这里不重复判断。
         */
        DRV_CurrentFilter_Push(&current_filter, reading.current_a);
    } else {
        current_snapshot.errors++;
        current_snapshot.adc_status = (uint8_t)(status == BSP_CURRENT_OK ? BSP_CURRENT_ERROR : status);
        current_snapshot.reading.valid = 0U;
        current_snapshot.reading.current_a = NAN;
        /* 采样失败也要记一笔，否则 rejected 计数看不出链路在掉样本。 */
        DRV_CurrentFilter_Push(&current_filter, NAN);
    }
    current_snapshot.mean_a = current_filter.mean_a;
    current_snapshot.min_a = current_filter.min_a;
    current_snapshot.max_a = current_filter.max_a;
    current_snapshot.mean_valid = current_filter.valid;
    current_snapshot.blocks = current_filter.blocks;
    current_snapshot.rejected = current_filter.rejected;
    BSP_Critical_Exit(lock);
}

void APP_Current_GetSnapshot(APP_CurrentSnapshot *out)
{
    if (out == NULL) { return; }
    uint32_t lock = BSP_Critical_Enter();
    *out = current_snapshot;
    uint32_t now = SVC_Timestamp_Ms();
    out->age_ms = has_sample ? (uint32_t)(now - out->sample_ms) : UINT32_MAX;
    if (!has_sample || out->age_ms > CURRENT_STALE_MS || out->adc_status != BSP_CURRENT_OK) {
        out->reading.valid = 0U;
        out->reading.current_a = NAN;
        if (!has_sample) { out->reading.adc_v = NAN; }
    }
    BSP_Critical_Exit(lock);
}

void APP_Current_Report(void)
{
    APP_CurrentSnapshot s;
    char adc_text[16], current_text[16];
    APP_Current_GetSnapshot(&s);
    uint8_t adc_ok = APP_Current_FormatFixed(adc_text, s.reading.adc_v, 5U);
    uint8_t current_ok = APP_Current_FormatFixed(current_text, s.reading.current_a, 3U);
    /*
     * 报**实际生效**的灵敏度，不是写死的 12.75。这一行原本是字面量字符串，
     * 灵敏度一旦被改（正是 2026-09-21 标定要做的事），它会继续报旧值——
     * 一个和实现脱节的诊断字段比没有这个字段更坏。
     *
     * 用 APP_Current_FormatFixed 而不是 %f：本文件的报文刻意不依赖 printf 的
     * 浮点支持（tests/test_current_format.py 钉着这条），这正是那个函数存在的理由。
     */
    char sens_text[16];
    (void)APP_Current_FormatFixed(sens_text, current_config.sensitivity_v_per_a * 1000.0f, 3U);
    if (!adc_ok || !current_ok) { s.reading.valid = 0U; }
    APP_Control_QueueText(
        "CURRENT raw=%lu adc_v=%s current_a=%s valid=%u calibrated=%u saturated=%u age_ms=%lu samples=%lu errors=%lu adc_status=%u source=AM32_55A_CURR nominal_mv_per_a=%s\r\n",
        (unsigned long)s.reading.raw, adc_text, current_text,
        (unsigned)s.reading.valid, (unsigned)s.reading.calibrated, (unsigned)s.reading.saturated,
        (unsigned long)s.age_ms, (unsigned long)s.samples, (unsigned long)s.errors, (unsigned)s.adc_status,
        sens_text);

    /*
     * 平均值单独一行：上面那行已接近 190 字符，再追加会顶到发送缓冲上限，而且
     * 改动既有字段有破坏上位机解析的风险。新增一行对老解析器是透明的。
     */
    char mean_text[16], min_text[16], max_text[16];
    uint8_t mean_ok = APP_Current_FormatFixed(mean_text, s.mean_valid ? s.mean_a : NAN, 3U);
    uint8_t min_ok = APP_Current_FormatFixed(min_text, s.mean_valid ? s.min_a : NAN, 3U);
    uint8_t max_ok = APP_Current_FormatFixed(max_text, s.mean_valid ? s.max_a : NAN, 3U);
    APP_Control_QueueText(
        /*
         * 键名一律带 avg_ 前缀：`valid` 在上面那行已经有了，含义完全不同
         * （那条是"这一拍的采样有效"，这条是"块平均已就绪"）。同一命令族里
         * 撞名的后果不是"看着乱"——两行一旦被拼进同一个缓冲区解析，后面的
         * valid 会盖掉前面的，"valid=1 但 adc_status=2"这类矛盾检查就此失效，
         * 非法回包被当成合法接受。2026-09-21 实测撞过这个坑。
         */
        "CURRENT AVG mean_a=%s min_a=%s max_a=%s avg_valid=%u avg_window=%u "
        "avg_blocks=%lu avg_rejected=%lu\r\n",
        mean_text, min_text, max_text,
        (unsigned)(s.mean_valid && mean_ok && min_ok && max_ok),
        (unsigned)APP_CURRENT_FILTER_WINDOW,
        (unsigned long)s.blocks, (unsigned long)s.rejected);
}
