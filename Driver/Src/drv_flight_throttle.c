#include "drv_flight_throttle.h"

#include <math.h>
#include <stddef.h>

/* 全部静态状态约 40 字节。 */
static volatile uint8_t ft_mode = DRV_FLIGHT_THROTTLE_MODE_FLIGHT;
static volatile uint8_t ft_state = DRV_FLIGHT_THROTTLE_STATE_DISARMED;
static volatile uint8_t ft_zone = DRV_FLIGHT_THROTTLE_ZONE_LOW;
static volatile uint8_t ft_xy_mode = DRV_FLIGHT_THROTTLE_XY_POSITION;
static volatile uint8_t ft_height_valid = 0U;
static volatile float ft_throttle = 0.0f;
static volatile float ft_height = 0.0f;
static uint8_t ft_lockout = 1U;      /* 1 = 起飞闭锁：杆必须先回到 CLIMB 区以下 */
static uint8_t ft_manual_low_seen = 0U; /* 角度档：LANDED 后杆到过 LOW 区，才允许按杆直接起转 */
static float ft_takeoff_s = 0.0f;
static float ft_ramp_s = 0.0f;
static float ft_land1_s = 0.0f;
static float ft_land2_s = 0.0f;

static float ft_clamp01(float v)
{
    if (!(v > 0.0f)) { return 0.0f; }   /* NaN 也落到 0 */
    if (v > 1.0f) { return 1.0f; }
    return v;
}

static void ft_reset_timers(void)
{
    ft_takeoff_s = 0.0f;
    ft_ramp_s = 0.0f;
    ft_land1_s = 0.0f;
    ft_land2_s = 0.0f;
}

void DRV_FlightThrottle_Init(void)
{
    ft_mode = DRV_FLIGHT_THROTTLE_MODE_FLIGHT;
    ft_state = DRV_FLIGHT_THROTTLE_STATE_DISARMED;
    ft_zone = DRV_FLIGHT_THROTTLE_ZONE_LOW;
    ft_xy_mode = DRV_FLIGHT_THROTTLE_XY_POSITION;
    ft_height_valid = 0U;
    ft_throttle = 0.0f;
    ft_height = 0.0f;
    ft_lockout = 1U;
    ft_manual_low_seen = 0U;
    ft_reset_timers();
}

DRV_FlightThrottleZone DRV_FlightThrottle_ZoneOf(float t)
{
    t = ft_clamp01(t);
    if (t < DRV_FLIGHT_THROTTLE_LOW_MAX) { return DRV_FLIGHT_THROTTLE_ZONE_LOW; }
    if (t < DRV_FLIGHT_THROTTLE_DESCEND_MAX) { return DRV_FLIGHT_THROTTLE_ZONE_DESCEND; }
    if (t <= DRV_FLIGHT_THROTTLE_HOLD_MAX) { return DRV_FLIGHT_THROTTLE_ZONE_HOLD; }
    return DRV_FLIGHT_THROTTLE_ZONE_CLIMB;
}

float DRV_FlightThrottle_ClimbRate(float t)
{
    return DRV_FlightThrottle_ClimbRateWith(t, 0.0f, 0.0f);
}

float DRV_FlightThrottle_ClimbRateWith(float t, float v_up, float v_dn)
{
    t = ft_clamp01(t);
    if (!(isfinite(v_up) && (v_up > 0.0f))) { v_up = DRV_FLIGHT_THROTTLE_V_UP_M_S; }
    if (!(isfinite(v_dn) && (v_dn > 0.0f))) { v_dn = DRV_FLIGHT_THROTTLE_V_DN_M_S; }
    switch (DRV_FlightThrottle_ZoneOf(t)) {
    case DRV_FLIGHT_THROTTLE_ZONE_LOW:
        return -v_dn;
    case DRV_FLIGHT_THROTTLE_ZONE_DESCEND:
        return -v_dn *
               (DRV_FLIGHT_THROTTLE_DESCEND_MAX - t) /
               (DRV_FLIGHT_THROTTLE_DESCEND_MAX - DRV_FLIGHT_THROTTLE_LOW_MAX);
    case DRV_FLIGHT_THROTTLE_ZONE_HOLD:
        return 0.0f;
    default:
        return v_up *
               (t - DRV_FLIGHT_THROTTLE_HOLD_MAX) /
               (1.0f - DRV_FLIGHT_THROTTLE_HOLD_MAX);
    }
}

float DRV_FlightThrottle_GroundSpin01(float t)
{
    t = ft_clamp01(t);
    if (t < DRV_FLIGHT_THROTTLE_LOW_MAX) { return 0.0f; }
    if (t >= DRV_FLIGHT_THROTTLE_HOLD_MAX) { return DRV_FLIGHT_THROTTLE_GROUND_MAX_01; }
    return DRV_FLIGHT_THROTTLE_GROUND_MIN_01 +
           (DRV_FLIGHT_THROTTLE_GROUND_MAX_01 - DRV_FLIGHT_THROTTLE_GROUND_MIN_01) *
           (t - DRV_FLIGHT_THROTTLE_LOW_MAX) /
           (DRV_FLIGHT_THROTTLE_HOLD_MAX - DRV_FLIGHT_THROTTLE_LOW_MAX);
}

DRV_FlightThrottleXyMode DRV_FlightThrottle_XyModeOf(float s)
{
    if (!(s >= DRV_FLIGHT_THROTTLE_SW_POSITION_MAX)) { return DRV_FLIGHT_THROTTLE_XY_POSITION; } /* 含 NaN */
    if (s > DRV_FLIGHT_THROTTLE_SW_ANGLE_MIN) { return DRV_FLIGHT_THROTTLE_XY_ANGLE; }
    return DRV_FLIGHT_THROTTLE_XY_VELOCITY;
}

float DRV_FlightThrottle_AngleModeForce(float t, float hover_n, float max_n)
{
    t = ft_clamp01(t);
    if (max_n < hover_n) { max_n = hover_n; }
    if (t <= DRV_FLIGHT_THROTTLE_ANGLE_MID) {
        return hover_n * t / DRV_FLIGHT_THROTTLE_ANGLE_MID;
    }
    return hover_n + (max_n - hover_n) * (t - DRV_FLIGHT_THROTTLE_ANGLE_MID) /
                     (1.0f - DRV_FLIGHT_THROTTLE_ANGLE_MID);
}

float DRV_FlightThrottle_ManualForce(float t, float hover_n, float max_n)
{
    t = ft_clamp01(t);
    if (t < DRV_FLIGHT_THROTTLE_LOW_MAX) { t = DRV_FLIGHT_THROTTLE_LOW_MAX; }
    return DRV_FlightThrottle_AngleModeForce(t, hover_n, max_n);
}

void DRV_FlightThrottle_SetMode(DRV_FlightThrottleMode mode)
{
    ft_mode = (uint8_t)((mode == DRV_FLIGHT_THROTTLE_MODE_LEGACY) ?
                        DRV_FLIGHT_THROTTLE_MODE_LEGACY : DRV_FLIGHT_THROTTLE_MODE_FLIGHT);
}

DRV_FlightThrottleMode DRV_FlightThrottle_GetMode(void) { return (DRV_FlightThrottleMode)ft_mode; }
DRV_FlightThrottleState DRV_FlightThrottle_GetState(void) { return (DRV_FlightThrottleState)ft_state; }
DRV_FlightThrottleZone DRV_FlightThrottle_GetZone(void) { return (DRV_FlightThrottleZone)ft_zone; }
DRV_FlightThrottleXyMode DRV_FlightThrottle_GetXyMode(void) { return (DRV_FlightThrottleXyMode)ft_xy_mode; }
float DRV_FlightThrottle_GetThrottle01(void) { return ft_throttle; }

uint8_t DRV_FlightThrottle_GetHeight(float *height_m)
{
    if (height_m != NULL) { *height_m = ft_height; }
    return ft_height_valid;
}

void DRV_FlightThrottle_Update(const DRV_FlightThrottleInput *in,
                               DRV_FlightThrottleOutput *out)
{
    float t;
    float dt;
    DRV_FlightThrottleState st;
    uint8_t legacy;

    if ((in == NULL) || (out == NULL)) { return; }
    t = ft_clamp01(in->throttle_01);
    dt = (in->dt_s > 0.0f) ? in->dt_s : 0.0f;
    if (dt > 0.1f) { dt = 0.1f; }   /* 长间隔不能一拍跨过多个时间阈值 */

    legacy = (uint8_t)((ft_mode == DRV_FLIGHT_THROTTLE_MODE_LEGACY) || (in->bypass != 0U));
    st = (DRV_FlightThrottleState)ft_state;

    out->zone = DRV_FlightThrottle_ZoneOf(t);
    out->xy_mode = DRV_FlightThrottle_XyModeOf(in->mode_switch_01);
    out->climb_rate_m_s = DRV_FlightThrottle_ClimbRateWith(t, in->v_up_m_s, in->v_dn_m_s);
    out->manual_force_valid = 0U;
    out->manual_force_n = 0.0f;
    out->ground_spin_01 = 0.0f;
    out->manual_integrate = 0U;
    out->legacy_semantics = legacy;

    ft_throttle = t;
    ft_zone = (uint8_t)out->zone;
    ft_xy_mode = (uint8_t)out->xy_mode;
    ft_height_valid = in->height_valid;
    ft_height = in->height_m;

    if ((in->armed == 0U) || (in->link_ok == 0U)) {
        st = DRV_FLIGHT_THROTTLE_STATE_DISARMED;
        ft_lockout = 1U;
        ft_manual_low_seen = 0U;
        ft_reset_timers();
    } else if (legacy != 0U) {
        /* 旧语义：状态只记"已解锁=LANDED"；旁路结束后从 LANDED 起步并闭锁起飞。 */
        st = DRV_FLIGHT_THROTTLE_STATE_LANDED;
        ft_lockout = 1U;
        ft_manual_low_seen = 0U;
        ft_reset_timers();
    } else {
        if (st == DRV_FLIGHT_THROTTLE_STATE_DISARMED) {
            st = DRV_FLIGHT_THROTTLE_STATE_LANDED;
            ft_reset_timers();
        }
        switch (st) {
        case DRV_FLIGHT_THROTTLE_STATE_LANDED:
            if (out->xy_mode == DRV_FLIGHT_THROTTLE_XY_ANGLE) {
                /* 角度档手控油门：杆到过底之后，一过 LOW_MAX 就直接起转（推力按杆，见下）。 */
                ft_takeoff_s = 0.0f;
                if (out->zone != DRV_FLIGHT_THROTTLE_ZONE_CLIMB) { ft_lockout = 0U; }
                if (t < DRV_FLIGHT_THROTTLE_LOW_MAX) {
                    ft_manual_low_seen = 1U;
                } else if ((ft_manual_low_seen != 0U) && (in->imu_valid != 0U)) {
                    st = DRV_FLIGHT_THROTTLE_STATE_FLYING;
                    ft_reset_timers();
                }
                break;
            }
            if (out->zone != DRV_FLIGHT_THROTTLE_ZONE_CLIMB) {
                ft_lockout = 0U;
                ft_takeoff_s = 0.0f;
            } else if ((ft_lockout == 0U) && (in->imu_valid != 0U)) {
                ft_takeoff_s += dt;
                if (ft_takeoff_s >= DRV_FLIGHT_THROTTLE_TAKEOFF_HOLD_S) {
                    st = DRV_FLIGHT_THROTTLE_STATE_SPOOLUP;
                    ft_reset_timers();
                }
            } else {
                ft_takeoff_s = 0.0f;
            }
            break;
        case DRV_FLIGHT_THROTTLE_STATE_SPOOLUP:
            if ((out->zone != DRV_FLIGHT_THROTTLE_ZONE_CLIMB) || (in->imu_valid == 0U)) {
                st = DRV_FLIGHT_THROTTLE_STATE_LANDED;   /* 中止，电机回最小 */
                ft_reset_timers();
            } else {
                ft_ramp_s += dt;
                if (ft_ramp_s >= DRV_FLIGHT_THROTTLE_SPOOLUP_S) {
                    st = DRV_FLIGHT_THROTTLE_STATE_FLYING;
                    ft_reset_timers();
                }
            }
            break;
        case DRV_FLIGHT_THROTTLE_STATE_FLYING:
        default:
            if (in->imu_valid == 0U) {
                st = DRV_FLIGHT_THROTTLE_STATE_LANDED;
                ft_reset_timers();
                break;
            }
            /* 判据 1 需要定高在跑：定点与速度保持都定高，角度模式不定高。 */
            if ((out->xy_mode != DRV_FLIGHT_THROTTLE_XY_ANGLE) && (in->height_valid != 0U) &&
                (in->height_m < DRV_FLIGHT_THROTTLE_LAND1_H_M) &&
                (out->climb_rate_m_s <= 0.0f) &&
                (in->vz_m_s < DRV_FLIGHT_THROTTLE_LAND1_VZ_M_S) &&
                (in->vz_m_s > -DRV_FLIGHT_THROTTLE_LAND1_VZ_M_S)) {
                ft_land1_s += dt;
            } else {
                ft_land1_s = 0.0f;
            }
            if ((t < DRV_FLIGHT_THROTTLE_LOW_MAX) && (in->height_valid != 0U) &&
                (in->height_m < DRV_FLIGHT_THROTTLE_LAND2_H_M)) {
                ft_land2_s += dt;
            } else {
                ft_land2_s = 0.0f;
            }
            if ((ft_land1_s >= DRV_FLIGHT_THROTTLE_LAND1_S) ||
                (ft_land2_s >= DRV_FLIGHT_THROTTLE_LAND2_S)) {
                st = DRV_FLIGHT_THROTTLE_STATE_LANDED;
                ft_reset_timers();
            }
            break;
        }
    }

    if (st == DRV_FLIGHT_THROTTLE_STATE_SPOOLUP) {
        float frac = ft_ramp_s / DRV_FLIGHT_THROTTLE_SPOOLUP_S;
        const float top = DRV_FLIGHT_THROTTLE_SPOOLUP_FRAC * in->hover_thrust_n;
        float start = (in->spool_start_force_n > 0.0f) ? in->spool_start_force_n : 0.0f;
        if (frac > 1.0f) { frac = 1.0f; }
        if (start > top) { start = top; }
        out->manual_force_valid = 1U;
        out->manual_force_n = start + (top - start) * frac;   /* 从地面慢转上限平滑爬到 0.85·hover */
    }
    if ((st == DRV_FLIGHT_THROTTLE_STATE_FLYING) && (legacy == 0U) &&
        (out->xy_mode == DRV_FLIGHT_THROTTLE_XY_ANGLE)) {
        out->manual_force_valid = 1U;
        out->manual_force_n = DRV_FlightThrottle_ManualForce(t, in->hover_thrust_n, in->max_thrust_n);
        out->manual_integrate = (uint8_t)(out->manual_force_n >=
                                          DRV_FLIGHT_THROTTLE_MANUAL_I_MIN_FRAC * in->hover_thrust_n);
    }
    if (st != DRV_FLIGHT_THROTTLE_STATE_LANDED) {
        ft_manual_low_seen = 0U;      /* 每次落地后都要重新把杆拉到底 */
    }
    if ((st == DRV_FLIGHT_THROTTLE_STATE_LANDED) && (legacy == 0U)) {
        out->ground_spin_01 = DRV_FlightThrottle_GroundSpin01(t);
    }
    ft_state = (uint8_t)st;
    out->state = st;
    out->active = (uint8_t)((legacy == 0U) &&
                            ((st == DRV_FLIGHT_THROTTLE_STATE_SPOOLUP) ||
                             (st == DRV_FLIGHT_THROTTLE_STATE_FLYING)));
}
