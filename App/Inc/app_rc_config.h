#ifndef APP_RC_CONFIG_H
#define APP_RC_CONFIG_H

#include <stdint.h>

#include "drv_elrs.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 遥控通道映射与端点标定。
 *
 * 在这之前，通道号是 app_stabilizer.c 里的一组 #define（CH1..CH6 写死），端点是
 * 1000/1500/2000 三个字面量。换一台发射机、改一次通道顺序、或者摇杆行程不是标准
 * 1000~2000，都得改代码重烧固件。这个模块把这两件事变成可标定、可持久化的配置。
 *
 * 分工：
 *   - 本模块只做纯函数（默认值、校验、归一化）和一份 seqlock 运行期快照；
 *   - Flash 读写归 app_control.c（和舵机/PID 参数同一条 CFG 记录）；
 *   - 控制环每周期读一次快照，不碰 Flash。
 */

#define APP_RC_CONFIG_MAGIC   0x52434D50UL /* "RCMP" */
#define APP_RC_CONFIG_SCHEMA  1U

/* 功能位序即协议里的 func_id，追加新功能只能往后加，不能重排。 */
#define APP_RC_FUNC_ROLL      0U
#define APP_RC_FUNC_PITCH     1U
#define APP_RC_FUNC_THROTTLE  2U
#define APP_RC_FUNC_YAW       3U
#define APP_RC_FUNC_ARM       4U
#define APP_RC_FUNC_MODE      5U
#define APP_RC_FUNC_COUNT     6U

#define APP_RC_CHANNEL_UNBOUND 0xFFU

/*
 * CRSF 的 us 值域是 [988, 2012]，留出余量后仍要挡住明显异常的标定结果：
 * 端点跨度太小会让归一化增益爆炸，中位偏出端点区间会让摇杆方向反常。
 */
#define APP_RC_US_MIN          800U
#define APP_RC_US_MAX          2200U
#define APP_RC_MIN_SPAN_US     200U
#define APP_RC_DEFAULT_MIN_US  1000U
#define APP_RC_DEFAULT_MID_US  1500U
#define APP_RC_DEFAULT_MAX_US  2000U
#define APP_RC_DEFAULT_DEADBAND_US 20U
#define APP_RC_MAX_DEADBAND_US 200U

typedef struct {
    uint8_t  channel;   /* 0..CRSF_CHANNEL_COUNT-1，或 APP_RC_CHANNEL_UNBOUND */
    uint8_t  reversed;
    uint16_t min_us;
    uint16_t mid_us;
    uint16_t max_us;
} APP_RcFunctionMap;

typedef struct {
    uint32_t magic;
    uint16_t schema;
    uint16_t size;
    APP_RcFunctionMap function[APP_RC_FUNC_COUNT];
    uint16_t deadband_us;
    uint8_t  calibrated;   /* 端点是实测的，不是默认值 */
    uint8_t  reserved0;
    uint32_t generation;   /* 每次写入递增，上位机据此判断"是否已生效" */
    uint32_t reserved1[3];
} APP_RcConfig;

/* 控制环每周期解析一次的结果，避免在多处重复查表。 */
typedef struct {
    uint16_t us[APP_RC_FUNC_COUNT];    /* 映射后的原始脉宽，未反向 */
    float    norm[APP_RC_FUNC_COUNT];  /* [-1,+1]，已反向/死区/标定 */
    float    throttle_01;              /* [0,1]，按 THROTTLE 的 min..max */
    uint8_t  bound_mask;               /* bit i = 功能 i 已绑定且通道有效 */
} APP_RcInputs;

void APP_RcConfig_Defaults(APP_RcConfig *config);

/*
 * 校验的是"能不能安全地拿来飞"，不是"好不好用"：
 * 通道号越界、端点跨度不足、中位不在端点之间、同一通道绑给多个功能，都拒绝。
 * 通道可以留空（UNBOUND），此时该功能输出恒 0 / 视为低电平。
 */
uint8_t APP_RcConfig_Validate(const APP_RcConfig *config);

const char *APP_RcConfig_FunctionName(uint8_t function);
/* 返回 APP_RC_FUNC_COUNT 表示无法识别。 */
uint8_t APP_RcConfig_FunctionFromName(const char *name);

/* 单个功能的归一化，供测试和协议直接调用。 */
float APP_RcConfig_Normalize(const APP_RcConfig *config,
                             uint8_t function,
                             uint16_t channel_us);
float APP_RcConfig_Throttle01(const APP_RcConfig *config,
                              uint16_t channel_us);

/* 把 16 路原始通道整体折算成功能输入。config 为 NULL 时用默认映射。 */
void APP_RcConfig_Resolve(const APP_RcConfig *config,
                          const uint16_t channels_us[CRSF_CHANNEL_COUNT],
                          APP_RcInputs *inputs);

/*
 * 运行期快照。写入方（控制/参数任务）调用 Publish，控制环调用 Read。
 * 与 app_flight_calibration 相同的 odd/even seqlock，读方永远拿到一致副本。
 */
void APP_RcConfig_ResetActive(void);
uint8_t APP_RcConfig_PublishActive(const APP_RcConfig *config);
uint8_t APP_RcConfig_ReadActive(APP_RcConfig *config);
uint32_t APP_RcConfig_GetActiveGeneration(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RC_CONFIG_H */
