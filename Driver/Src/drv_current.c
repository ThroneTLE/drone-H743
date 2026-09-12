#include "drv_current.h"
#include <math.h>
#include <stddef.h>

DRV_CurrentConfig DRV_Current_Am32_55A_Default(void)
{
    DRV_CurrentConfig c = {3.3f, 0.01275f, 0.0f, 1.0f, 65535U, 0U};
    return c;
}

DRV_CurrentStatus DRV_Current_Convert(const DRV_CurrentConfig *c,
                                    uint32_t raw, DRV_CurrentReading *out)
{
    if (c == NULL || out == NULL || c->adc_max == 0U || c->adc_max > 65535U ||
        raw > c->adc_max || !isfinite(c->reference_v) || c->reference_v <= 0.0f ||
        c->reference_v > 3.6f || !isfinite(c->sensitivity_v_per_a) ||
        c->sensitivity_v_per_a <= 0.0f || !isfinite(c->zero_offset_v) ||
        !isfinite(c->input_scale) || c->input_scale <= 0.0f || c->calibrated > 1U) {
        return DRV_CURRENT_INVALID;
    }
    DRV_CurrentReading r = {0};
    r.raw = raw;
    r.adc_v = ((float)raw / (float)c->adc_max) * c->reference_v;
    r.current_a = (r.adc_v * c->input_scale - c->zero_offset_v) / c->sensitivity_v_per_a;
    if (!isfinite(r.current_a)) { return DRV_CURRENT_INVALID; }
    r.calibrated = c->calibrated;
    r.saturated = raw == c->adc_max;
    r.valid = !r.saturated;
    if (r.saturated) { r.current_a = NAN; }
    *out = r;
    return r.saturated ? DRV_CURRENT_SATURATED : DRV_CURRENT_OK;
}
