#ifndef APP_CURRENT_H
#define APP_CURRENT_H
#include "drv_current.h"
#include <stdint.h>

#define APP_CURRENT_PERIOD_MS 20U

typedef struct {
    DRV_CurrentReading reading;
    uint32_t sample_ms; /* SVC_Timestamp_Ms(), uint32 wrap-safe */
    uint32_t age_ms;    /* UINT32_MAX when no sample has arrived */
    uint32_t samples;
    uint32_t errors;
    uint8_t adc_status; /* 0=OK, 1=not ready, 2=timeout, 3=error */
    /*
     * 块平均结果（drv_current_filter.h）。`reading.current_a` 是单点瞬时值，对
     * 带 PWM 纹波的电调电流输出**没有定义**；要显示、要和外部电流表对照，用这里。
     * mean_valid 为 0 表示还没凑满第一个块，此时三个值都没有意义。
     * min_a/max_a 是该块内的纹波范围，故意暴露出来——不给它，看的人无法区分
     * "信号干净"和"纹波很大被抹平了"。
     */
    float mean_a;
    float min_a;
    float max_a;
    uint8_t mean_valid;
    uint32_t blocks;    /* 已完成的块数 */
    uint32_t rejected;  /* 被丢弃的非有限样本数 */
} APP_CurrentSnapshot;

void APP_Current_Init(void); /* low-rate messageTask startup; sole ADC owner */
void APP_Current_Step(void); /* target 20 ms, independent of storage worker */
/* valid means fresh ADC + unsaturated conversion, not calibrated accuracy or
 * electrical connection detection. Invalid/stale current is NaN, never a fake 0 A.
 */
void APP_Current_GetSnapshot(APP_CurrentSnapshot *out);
void APP_Current_Report(void); /* communication task; snapshots only, no ADC I/O */
#endif
