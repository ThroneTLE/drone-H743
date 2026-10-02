#include "app_hover_thrust.h"

#include <math.h>
#include <string.h>

/* 状态放 AXI SRAM（DTCM 已用 97%）；noinit 段上电不清零，靠 APP_HoverThrust_Init 显式清。 */
#if defined(__GNUC__) && defined(__arm__)
#define HOVER_AXI_NOINIT __attribute__((section(".ram_d1_noinit"), aligned(32)))
#else
#define HOVER_AXI_NOINIT
#endif

typedef struct {
    DRV_HoverEst est;
    uint8_t  initialized;
    uint8_t  ground_valid;
    uint8_t  range_lpf_valid;
    uint8_t  airborne;
    uint8_t  gate_mask;
    uint8_t  lift_pending;
    float    ground_m;
    float    range_lpf_m;
    float    above_ground_m;
    uint32_t lift_since_ms;      /* 测距首次越过 LIFT_ON 的时刻 */
} HoverState;

static HoverState hover_state HOVER_AXI_NOINIT;
static APP_HoverThrustSnapshot hover_snapshot HOVER_AXI_NOINIT;
static volatile uint32_t hover_seq HOVER_AXI_NOINIT;
static volatile uint8_t hover_reset_request HOVER_AXI_NOINIT;

static void hover_publish(void)
{
    APP_HoverThrustSnapshot snap;

    memset(&snap, 0, sizeof(snap));
    if (hover_state.initialized != 0U) {
        DRV_HoverEst_GetOutput(&hover_state.est, &snap.est);
    }
    snap.initialized = hover_state.initialized;
    snap.gate_mask = hover_state.gate_mask;
    snap.airborne = hover_state.airborne;
    snap.ground_valid = hover_state.ground_valid;
    snap.ground_m = hover_state.ground_m;
    snap.above_ground_m = hover_state.above_ground_m;
    hover_seq++;                  /* 奇数 = 写入中 */
    hover_snapshot = snap;
    hover_seq++;
}

void APP_HoverThrust_Init(void)
{
    memset(&hover_state, 0, sizeof(hover_state));
    memset(&hover_snapshot, 0, sizeof(hover_snapshot));
    hover_seq = 0U;
    hover_reset_request = 0U;
}

void APP_HoverThrust_RequestReset(void)
{
    hover_reset_request = 1U;
}

uint8_t APP_HoverThrust_GetSnapshot(APP_HoverThrustSnapshot *out)
{
    uint32_t before;
    uint32_t after;
    uint8_t tries;

    if (out == NULL) {
        return 0U;
    }
    for (tries = 0U; tries < 8U; ++tries) {
        before = hover_seq;
        *out = hover_snapshot;
        after = hover_seq;
        if ((before == after) && ((before & 1U) == 0U)) {
            return out->initialized;
        }
    }
    return 0U;
}

/* 支撑面高度：未解锁滑动平均；解锁后只往下追。 */
static void hover_track_ground(const APP_HoverThrustInput *in)
{
    if (in->range_valid == 0U) {
        return;
    }
    if (hover_state.range_lpf_valid == 0U) {
        hover_state.range_lpf_m = in->range_m;
        hover_state.range_lpf_valid = 1U;
    } else {
        hover_state.range_lpf_m += APP_HOVER_RANGE_ALPHA * (in->range_m - hover_state.range_lpf_m);
    }
    if (hover_state.ground_valid == 0U) {
        hover_state.ground_m = in->range_m;       /* 首个有效读数起步（含开机即解锁） */
        hover_state.ground_valid = 1U;
    } else if (in->armed == 0U) {
        hover_state.ground_m += APP_HOVER_GROUND_ALPHA * (in->range_m - hover_state.ground_m);
    } else if (hover_state.range_lpf_m < hover_state.ground_m) {
        hover_state.ground_m = hover_state.range_lpf_m;
    }
}

static void hover_update_airborne(const APP_HoverThrustInput *in)
{
    if (in->range_valid != 0U) {
        hover_state.above_ground_m = in->range_m - hover_state.ground_m;
    }
    if ((in->armed == 0U) || (in->range_valid == 0U) || (hover_state.ground_valid == 0U)) {
        hover_state.airborne = 0U;
        hover_state.lift_pending = 0U;
        return;
    }
    if (hover_state.above_ground_m <= APP_HOVER_LIFT_OFF_M) {
        hover_state.airborne = 0U;
        hover_state.lift_pending = 0U;
    } else if (hover_state.airborne == 0U) {
        if (hover_state.above_ground_m > APP_HOVER_LIFT_ON_M) {
            if (hover_state.lift_pending == 0U) {
                hover_state.lift_pending = 1U;
                hover_state.lift_since_ms = in->now_ms;
            } else if ((uint32_t)(in->now_ms - hover_state.lift_since_ms) >=
                       APP_HOVER_LIFT_HOLD_MS) {
                hover_state.airborne = 1U;
            }
        } else {
            hover_state.lift_pending = 0U;
        }
    }
}

static float hover_init_value(const APP_HoverThrustInput *in)
{
    if ((isfinite(in->hover_cfg_n) != 0) && (in->hover_cfg_n > 0.0f)) {
        return in->hover_cfg_n;
    }
    if ((isfinite(in->mass_kg) != 0) && (in->mass_kg > 0.0f) && (in->gravity_m_s2 > 1.0f)) {
        return in->mass_kg * in->gravity_m_s2;
    }
    return 0.0f;
}

void APP_HoverThrust_Step(const APP_HoverThrustInput *in)
{
    uint8_t mask = 0U;
    DRV_HoverEstInput est_in;

    if (in == NULL) {
        return;
    }
    if (hover_reset_request != 0U) {
        hover_reset_request = 0U;
        hover_state.initialized = 0U;       /* 下面按当前 coax.hover_thrust_n / m·g 重新起步 */
    }
    /* 一个样本都还没学时初值跟着配置走：coax.hover_thrust_n 只在 RAM，上电为 0 → 起步按 m·g，
     * 之后才被设成台架值（2026-09-30 首次上板 init_n=11.235 而配置已是 14.25）。学过就不再动。 */
    if ((hover_state.initialized != 0U) && (hover_state.est.samples == 0U)) {
        const float cfg_init_n = hover_init_value(in);

        if ((cfg_init_n > 0.0f) && (fabsf(cfg_init_n - hover_state.est.init_n) > 0.01f)) {
            hover_state.initialized = 0U;
        }
    }
    if (hover_state.initialized == 0U) {
        const float init_n = hover_init_value(in);

        if (init_n <= 0.0f) {
            return;                          /* airframe 还没就绪 */
        }
        (void)DRV_HoverEst_Init(&hover_state.est, NULL, init_n);
        hover_state.initialized = 1U;
    }

    hover_track_ground(in);
    hover_update_airborne(in);

    if (in->armed != 0U) {
        mask |= APP_HOVER_GATE_ARMED;
    }
    if ((in->thrust_valid != 0U) && (isfinite(in->thrust_n) != 0) &&
        (in->thrust_n >= APP_HOVER_THRUST_MIN_N)) {
        mask |= APP_HOVER_GATE_THRUST;
    }
    if (in->saturated == 0U) {
        mask |= APP_HOVER_GATE_UNSAT;
    }
    if ((fabsf(in->roll_rad) < APP_HOVER_TILT_MAX_RAD) &&
        (fabsf(in->pitch_rad) < APP_HOVER_TILT_MAX_RAD)) {
        mask |= APP_HOVER_GATE_TILT;
    }
    if (in->accel_valid != 0U) {
        mask |= APP_HOVER_GATE_ACCEL;
    }
    if (hover_state.airborne != 0U) {
        mask |= APP_HOVER_GATE_AIRBORNE;
    }
    hover_state.gate_mask = mask;

    est_in.dt_s = in->dt_s;
    est_in.thrust_n = (in->thrust_valid != 0U) ? in->thrust_n : 0.0f;
    est_in.roll_rad = in->roll_rad;
    est_in.pitch_rad = in->pitch_rad;
    est_in.a_up_m_s2 = in->a_up_m_s2;
    est_in.gravity_m_s2 = in->gravity_m_s2;
    est_in.learn = (mask == APP_HOVER_GATE_ALL) ? 1U : 0U;
    (void)DRV_HoverEst_Step(&hover_state.est, &est_in);
    hover_publish();
}
