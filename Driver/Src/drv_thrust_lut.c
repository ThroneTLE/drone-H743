#include "drv_thrust_lut.h"

#include <stddef.h>

/* 生成的表数据（python -m tools.thrust_bench.thrust_lut_export），勿手改。 */
#include "drv_thrust_lut_table.inc"

static float lut_clamp(float value, float lo, float hi)
{
    if (!(value >= lo)) { return lo; } /* 也挡住 NaN */
    if (value > hi) { return hi; }
    return value;
}

static float lut_cell(float value, float origin, float step, uint16_t count, uint16_t *index)
{
    const float last = (float)(count - 1U);
    float position = lut_clamp((value - origin) / step, 0.0f, last);
    uint16_t i = (uint16_t)position;

    if (i >= (uint16_t)(count - 1U)) {
        i = (uint16_t)(count - 2U);
    }
    *index = i;
    return position - (float)i;
}

float DRV_ThrustLut_ReadGrid(const DRV_ThrustLutGrid *grid, float x, float y)
{
    uint16_t ix;
    uint16_t iy;
    float fx;
    float fy;
    const float *row0;
    const float *row1;

    if ((grid == NULL) || (grid->values == NULL) || (grid->nx < 2U) || (grid->ny < 2U)) {
        return 0.0f;
    }
    fx = lut_cell(x, grid->x0, grid->dx, grid->nx, &ix);
    fy = lut_cell(y, grid->y0, grid->dy, grid->ny, &iy);
    row0 = &grid->values[(uint32_t)iy * grid->nx];
    row1 = row0 + grid->nx;
    return (1.0f - fy) * ((1.0f - fx) * row0[ix] + fx * row0[ix + 1U]) +
           fy * ((1.0f - fx) * row1[ix] + fx * row1[ix + 1U]);
}

/* 实测以外的网格角（如零转速处）由平滑外推，可能略小于 0：推力不会为负。 */
float DRV_ThrustLut_ThrustFromSpeed(float upper_erpm, float lower_erpm)
{
    return lut_clamp(DRV_ThrustLut_ReadGrid(&DRV_ThrustLut_Table.speed, upper_erpm, lower_erpm), 0.0f, 1.0e6f);
}

float DRV_ThrustLut_ThrustFromEffective(float upper_effective, float lower_effective)
{
    return lut_clamp(DRV_ThrustLut_ReadGrid(&DRV_ThrustLut_Table.effective, upper_effective, lower_effective),
                     0.0f, 1.0e6f);
}

float DRV_ThrustLut_BalancedThrustForEffective(float effective)
{
    const DRV_ThrustLutTable *t = &DRV_ThrustLut_Table;
    const uint16_t n = t->diag_count;

    if (!(effective > 0.0f) || (n < 2U)) {
        return 0.0f;
    }
    /* 实测最低油门以下按直线收到 0：那里推力本来就只有几克。 */
    if (effective <= t->diag_effective[0]) {
        return t->diag_thrust_g[0] * effective / t->diag_effective[0];
    }
    if (effective >= t->diag_effective[n - 1U]) {
        return t->diag_thrust_g[n - 1U];
    }
    for (uint16_t i = 1U; i < n; ++i) {
        if (effective <= t->diag_effective[i]) {
            const float span = t->diag_effective[i] - t->diag_effective[i - 1U];
            const float ratio = (effective - t->diag_effective[i - 1U]) / span;
            return t->diag_thrust_g[i - 1U] + ratio * (t->diag_thrust_g[i] - t->diag_thrust_g[i - 1U]);
        }
    }
    return t->diag_thrust_g[n - 1U];
}

float DRV_ThrustLut_BalancedEffectiveForThrust(float total_g)
{
    const DRV_ThrustLutTable *t = &DRV_ThrustLut_Table;
    const uint16_t n = t->diag_count;

    if (!(total_g > 0.0f) || (n < 2U)) {
        return 0.0f;
    }
    if (total_g <= t->diag_thrust_g[0]) {
        return (t->diag_thrust_g[0] > 0.0f) ?
            t->diag_effective[0] * total_g / t->diag_thrust_g[0] : t->diag_effective[0];
    }
    for (uint16_t i = 1U; i < n; ++i) {
        if (total_g <= t->diag_thrust_g[i]) {
            const float rise = t->diag_thrust_g[i] - t->diag_thrust_g[i - 1U];
            const float ratio = (rise > 0.0f) ? (total_g - t->diag_thrust_g[i - 1U]) / rise : 0.0f;
            return t->diag_effective[i - 1U] + ratio * (t->diag_effective[i] - t->diag_effective[i - 1U]);
        }
    }
    return t->diag_effective[n - 1U]; /* 超出实测最大推力：饱和 */
}

float DRV_ThrustLut_PairShift(float e_upper, float e_lower, float total_g, float e_max)
{
    const float floor_e = (DRV_ThrustLut_Table.diag_count > 0U) ? DRV_ThrustLut_Table.diag_effective[0] : 0.0f;
    const float low_e = (e_upper < e_lower) ? e_upper : e_lower;
    const float high_e = (e_upper > e_lower) ? e_upper : e_lower;
    float lo = floor_e - low_e;
    float hi = e_max - high_e;

    /* 有一桨已饱和或低于实测最低油门：不修正，照逐桨换算（饱和时绝不能压低另一桨）。 */
    if (!(total_g > 0.0f) || (low_e < floor_e) || (high_e > e_max)) {
        return 0.0f;
    }
    if (lo < -DRV_THRUST_LUT_PAIR_SHIFT_LIMIT) { lo = -DRV_THRUST_LUT_PAIR_SHIFT_LIMIT; }
    if (hi > DRV_THRUST_LUT_PAIR_SHIFT_LIMIT) { hi = DRV_THRUST_LUT_PAIR_SHIFT_LIMIT; }
    if (DRV_ThrustLut_ThrustFromEffective(e_upper + hi, e_lower + hi) < total_g) {
        return hi;
    }
    if (DRV_ThrustLut_ThrustFromEffective(e_upper + lo, e_lower + lo) >= total_g) {
        return lo;
    }
    for (uint32_t i = 0U; i < 24U; ++i) { /* 24 次二分：分辨率远小于 0.001% */
        const float mid = 0.5f * (lo + hi);
        if (DRV_ThrustLut_ThrustFromEffective(e_upper + mid, e_lower + mid) < total_g) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5f * (lo + hi);
}

float DRV_ThrustLut_ChargeVoltage(float loaded_v, float erpm_a, float erpm_b)
{
    const float a = erpm_a / 10000.0f;
    const float b = erpm_b / 10000.0f;

    return loaded_v + DRV_ThrustLut_Table.sag_k * (a * a * a + b * b * b);
}

static float lut_compensation_voltage(float charge_v)
{
    const DRV_ThrustLutTable *t = &DRV_ThrustLut_Table;

    if (!(charge_v > 0.0f)) {
        return t->v_ref;
    }
    return lut_clamp(charge_v, t->charge_usable_v[0], t->charge_usable_v[1]);
}

float DRV_ThrustLut_PercentForEffective(float effective, float charge_v)
{
    const float percent = effective * DRV_ThrustLut_Table.v_ref / lut_compensation_voltage(charge_v);

    return lut_clamp(percent, 0.0f, 100.0f);
}

float DRV_ThrustLut_EffectiveForPercent(float percent, float charge_v)
{
    return lut_clamp(percent, 0.0f, 100.0f) * lut_compensation_voltage(charge_v) / DRV_ThrustLut_Table.v_ref;
}
