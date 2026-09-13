#include "app_current.h"
#include "app_current_format.h"
#include "app_control.h"
#include "bsp_current.h"
#include "bsp_critical.h"
#include "svc_timestamp.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define CURRENT_PERIOD_MS 20U
#define CURRENT_STALE_MS 250U

static DRV_CurrentConfig current_config;
static APP_CurrentSnapshot current_snapshot = {.adc_status=BSP_CURRENT_NOT_READY};
static uint32_t last_attempt_ms;
static uint8_t ready;
static uint8_t has_sample;

void APP_Current_Init(void)
{
    current_config = DRV_Current_Am32_55A_Default();
    memset(&current_snapshot, 0, sizeof(current_snapshot));
    current_snapshot.reading.adc_v = NAN;
    current_snapshot.reading.current_a = NAN;
    current_snapshot.adc_status = (uint8_t)BSP_Current_Init();
    ready = current_snapshot.adc_status == BSP_CURRENT_OK;
    has_sample = 0U;
    current_snapshot.errors = ready ? 0U : 1U;
    last_attempt_ms = SVC_Timestamp_Ms() - CURRENT_PERIOD_MS;
}

void APP_Current_Step(void)
{
    uint32_t now = SVC_Timestamp_Ms();
    if (!ready || (uint32_t)(now - last_attempt_ms) < CURRENT_PERIOD_MS) { return; }
    last_attempt_ms = now;
    uint32_t raw;
    BSP_CurrentStatus status = BSP_Current_Read(&raw);
    DRV_CurrentReading reading;
    DRV_CurrentStatus converted = DRV_CURRENT_INVALID;
    if (status == BSP_CURRENT_OK) { converted = DRV_Current_Convert(&current_config, raw, &reading); }
    uint32_t sampled_ms = SVC_Timestamp_Ms();
    uint32_t lock = BSP_Critical_Enter();
    if (status == BSP_CURRENT_OK && converted != DRV_CURRENT_INVALID) {
        current_snapshot.reading = reading;
        current_snapshot.sample_ms = sampled_ms;
        current_snapshot.samples++;
        current_snapshot.adc_status = BSP_CURRENT_OK;
        has_sample = 1U;
    } else {
        current_snapshot.errors++;
        current_snapshot.adc_status = (uint8_t)(status == BSP_CURRENT_OK ? BSP_CURRENT_ERROR : status);
        current_snapshot.reading.valid = 0U;
        current_snapshot.reading.current_a = NAN;
    }
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
    if (!adc_ok || !current_ok) { s.reading.valid = 0U; }
    APP_Control_QueueText(
        "CURRENT raw=%lu adc_v=%s current_a=%s valid=%u calibrated=%u saturated=%u age_ms=%lu samples=%lu errors=%lu adc_status=%u source=AM32_55A_CURR nominal_mv_per_a=12.75\r\n",
        (unsigned long)s.reading.raw, adc_text, current_text,
        (unsigned)s.reading.valid, (unsigned)s.reading.calibrated, (unsigned)s.reading.saturated,
        (unsigned long)s.age_ms, (unsigned long)s.samples, (unsigned long)s.errors, (unsigned)s.adc_status);
}
