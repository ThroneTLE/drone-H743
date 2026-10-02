/*
 * THRMODE? / THRMODE FLIGHT|LEGACY —— 飞行油门模式开关与状态查询（契约 doc/flight-throttle-contract.md）。
 *
 *   THRMODE?   -> THRMODE mode=flight|legacy state=disarmed|landed|spoolup|flying
 *                 zone=low|descend|hold|climb xy=pos|vel|angle（CH6 三档：定点/速度保持/角度）
 *                 thr_x1000=<t01*1000> h_mm=<相对高度mm> range=ok|none
 *                 tmax_mn=<单桨此刻可用最大推力 mN> hover_mn=<悬停推力 mN>（两者看推力余量：
 *                 2*tmax 与 hover 之差就是爬升/偏航差速能用的全部余量）
 *   THRMODE FLIGHT|LEGACY -> 同一行再加 state2=set（只存 RAM，上电 FLIGHT）
 *                 解锁时拒绝 -> THRMODE state=rejected reason=armed
 *
 * 全部整数输出（newlib-nano 无浮点 printf）。切换只在未解锁时允许。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_stabilizer.h"
#include "drv_airframe_params.h"
#include "drv_coax_ctrl.h"
#include "drv_flight_throttle.h"

#include <string.h>

static void thrmode_report(const char *suffix)
{
    static const char *const state_name[] = { "disarmed", "landed", "spoolup", "flying" };
    static const char *const zone_name[] = { "low", "descend", "hold", "climb" };
    static const char *const xy_name[] = { "pos", "vel", "angle" };
    float height_m = 0.0f;
    uint8_t range_ok = DRV_FlightThrottle_GetHeight(&height_m);
    unsigned st = (unsigned)DRV_FlightThrottle_GetState();
    unsigned zn = (unsigned)DRV_FlightThrottle_GetZone();
    unsigned xy = (unsigned)DRV_FlightThrottle_GetXyMode();
    long thr_x1000 = (long)(DRV_FlightThrottle_GetThrottle01() * 1000.0f + 0.5f);
    long h_mm = (long)(height_m * 1000.0f + ((height_m >= 0.0f) ? 0.5f : -0.5f));
    const DRV_Airframe_Params *airframe = DRV_Airframe_Get();
    long tmax_mn = (long)(DRV_COAX_CTRL_SingleMaxThrustN() * 1000.0f + 0.5f);
    long hover_mn = (long)(DRV_COAX_CTRL_EffectiveMassKg(airframe->mass_kg) *
                           airframe->gravity_m_s2 * 1000.0f + 0.5f);

    if (st > 3U) { st = 0U; }
    if (zn > 3U) { zn = 0U; }
    if (xy > 2U) { xy = 0U; }
    APP_Control_QueueText("THRMODE mode=%s state=%s zone=%s xy=%s thr_x1000=%ld h_mm=%ld range=%s"
                          " tmax_mn=%ld hover_mn=%ld%s\r\n",
                          (DRV_FlightThrottle_GetMode() == DRV_FLIGHT_THROTTLE_MODE_LEGACY) ?
                              "legacy" : "flight",
                          state_name[st], zone_name[zn], xy_name[xy], thr_x1000,
                          (range_ok != 0U) ? h_mm : 0L,
                          (range_ok != 0U) ? "ok" : "none", tmax_mn, hover_mn, suffix);
}

uint8_t app_control_handle_thrmode(char **tokens, uint32_t count)
{
    DRV_FlightThrottleMode mode;

    if ((count < 1U) || (tokens == NULL) || (tokens[0] == NULL)) {
        return 0U;
    }
    if (strcmp(tokens[0], "THRMODE?") == 0) {
        if (count != 1U) {
            return 0U;
        }
        thrmode_report("");
        return 1U;
    }
    if ((strcmp(tokens[0], "THRMODE") != 0) || (count != 2U) || (tokens[1] == NULL)) {
        return 0U;
    }
    if (strcmp(tokens[1], "FLIGHT") == 0) {
        mode = DRV_FLIGHT_THROTTLE_MODE_FLIGHT;
    } else if (strcmp(tokens[1], "LEGACY") == 0) {
        mode = DRV_FLIGHT_THROTTLE_MODE_LEGACY;
    } else {
        APP_Control_QueueText("THRMODE state=rejected reason=arg\r\n");
        return 1U;
    }
    /* 解锁时拒绝：飞行中途换油门语义会让电机输出跳变。 */
    if ((APP_Stabilizer_IsArmed() != 0U) ||
        (DRV_FlightThrottle_GetState() != DRV_FLIGHT_THROTTLE_STATE_DISARMED)) {
        APP_Control_QueueText("THRMODE state=rejected reason=armed\r\n");
        return 1U;
    }
    DRV_FlightThrottle_SetMode(mode);
    thrmode_report(" state2=set");
    return 1U;
}
