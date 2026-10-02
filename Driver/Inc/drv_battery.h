#ifndef DRV_BATTERY_H
#define DRV_BATTERY_H
#include <stdint.h>
#define DRV_BATTERY_STALE_MS 250U
/* 默认门限（每节，3S）。2026-10-01 起为 11.4/11.45 V；2026-10-02 作者："电池限制可以降低一些了，你根据我们实机
 * 电池电压来稍微往下降一降电池的封锁阈值"。10-01 晚三份自由飞日志：起转前静止 11.32–11.37 V 飞行正常、带载起步约
 * 10.2–10.3 V；带载低于约 9.9 V 后满油门（1935 µs+）占比 14%→47%、推力顶死（长航时结尾事故即在此区）；静止→带载
 * 压降约 1.0 V，末段带载每分钟约掉 0.5 V。按"带载 ≥9.95 V ⇔ 静止 ≥10.95 V，再留 30–40 s 可用"取 11.2 V 判低、
 * 11.25 V 恢复（小回差防抖）。辨识页 11.4 V 以下提示"推力可能到不了目标"。 */
#define DRV_BATTERY_DEFAULT_LOW_CELL_MV 3733U
#define DRV_BATTERY_DEFAULT_RECOVER_CELL_MV 3750U
typedef struct { uint8_t cells; uint16_t low_cell_mv, recover_cell_mv; } DRV_BatteryConfig;
typedef struct {
    DRV_BatteryConfig config;
    uint32_t raw, voltage_mv, sample_ms, samples, errors;
    uint8_t valid, low, saturated, adc_status;
} DRV_BatteryState;
void DRV_Battery_Init(DRV_BatteryState *state);
uint8_t DRV_Battery_Configure(DRV_BatteryState *state, DRV_BatteryConfig config);
void DRV_Battery_Update(DRV_BatteryState *state, uint32_t raw, uint8_t adc_status, uint32_t now_ms);
uint8_t DRV_Battery_IsFresh(const DRV_BatteryState *state, uint32_t now_ms);
uint8_t DRV_Battery_CanArm(const DRV_BatteryState *state, uint32_t now_ms);
#endif
