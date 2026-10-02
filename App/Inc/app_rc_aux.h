/*
 * 遥控辅助开关：不进 RCMAP 的单一功能键（目前只有"手动 IMU 归零"）。
 *
 * 作者 2026-10-01："CH9  2000的时候手动IMU标定归零"。动作与 `IMUZERO` 命令相同
 * （app_cmd_imuzero.c：陀螺零偏 + 姿态零点按上电规则重新采样，机体需静止约 3 s），只在上锁时生效。
 *
 * 通道绑定只在本文件：IMU 归零 = CH9（CRSF 下标 8）。要做成可标定时并入 RCMAP（需升配置版本），
 * 不要在别处再写通道下标。
 *
 * 触发规则（防误触）：
 *   - 拨到高位（> HIGH_US）的**上升沿**才触发；必须先见过低位（< LOW_US），所以上电/重连时
 *     开关本来就在高位不会触发，拨住不放也只触发一次。
 *   - 已解锁时拨高：不触发，回 `IMUZERO state=armed_blocked source=rc`，且要拨回低位再拨高才算下一次。
 *   - 遥控链路不正常：清"见过低位"，恢复后要重新见到低位。
 *   - 两次触发至少隔 HOLDOFF_MS（一次归零的采样窗口）。
 */
#ifndef APP_RC_AUX_H
#define APP_RC_AUX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_RC_AUX_IMUZERO_CHANNEL     8U      /* CH9 */
#define APP_RC_AUX_HIGH_US             1800U
#define APP_RC_AUX_LOW_US              1600U
#define APP_RC_AUX_IMUZERO_HOLDOFF_MS  3000U

#define APP_RC_AUX_ACTION_NONE          0U
#define APP_RC_AUX_ACTION_IMUZERO       1U
#define APP_RC_AUX_ACTION_ARMED_BLOCKED 2U

typedef struct {
    uint8_t  low_seen;
    uint8_t  holdoff_active;
    uint32_t holdoff_until_ms;
} APP_RcAuxState;

/* 纯判定（不调任何外部函数），宿主可测。返回 APP_RC_AUX_ACTION_*。 */
uint8_t APP_RcAux_Step(APP_RcAuxState *state, uint32_t now_ms, uint16_t channel_us,
                       uint8_t link_ok, uint8_t armed);

/* 控制任务每拍调用：ch 为 CRSF 16 通道原始 us。触发时发起陀螺零偏与姿态零点重采样并回一行文本。 */
void APP_RcAux_Update(uint32_t now_ms, const uint16_t ch[16], uint8_t link_ok, uint8_t armed);

#ifdef __cplusplus
}
#endif

#endif
