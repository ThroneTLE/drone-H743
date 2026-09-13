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
} APP_CurrentSnapshot;

void APP_Current_Init(void); /* low-rate messageTask startup; sole ADC owner */
void APP_Current_Step(void); /* target 20 ms, independent of storage worker */
/* valid means fresh ADC + unsaturated conversion, not calibrated accuracy or
 * electrical connection detection. Invalid/stale current is NaN, never a fake 0 A.
 */
void APP_Current_GetSnapshot(APP_CurrentSnapshot *out);
void APP_Current_Report(void); /* communication task; snapshots only, no ADC I/O */
#endif
