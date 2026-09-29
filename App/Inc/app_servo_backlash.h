#ifndef APP_SERVO_BACKLASH_H
#define APP_SERVO_BACKLASH_H

#include <stdint.h>

/*
 * 舵机回差补偿：策略层（作者 2026-09-27 决定："加回差补偿，舵机换向时多走约 1.1°，
 * 先把空程吃掉"）。
 *
 * 解决什么：电机不转的舵机单独三轮（杆上 3°/5°/10°，data/identification/attitude/
 * 2026-09-27/rod_224531_d363ddd2、rod_223614_af74f7cd、rod_224556_f922902b）里，
 * 俯仰倾转舵机的表观反作用惯量按 J(A) = J∞(1 − b/A) 随幅值变小、纯延迟随幅值变长
 * （95/75/43 ms），拟合出回差半宽 b ≈ 0.0199 rad（1.14°）。悬停时的小修正正好落在
 * 这段空程里，带积分就可能极限环。横滚舵机同型号，没测过，暂按同值。
 *
 * 分工（decoupling-spec）：补偿算法在 Driver/drv_servo_backlash.c；这里只管
 *   - **补谁**：只补在飞控制器（已解锁）和光杆辨识（FF/RATE/ANGLE/SERVO 在跑）产生的
 *     舵机指令。舵机标定、地面点动、验收覆盖、舵机反馈台架、老 IDENT、上锁/低油门回中
 *     一律原样——回中必须是标定中位本身。
 *   - **什么时候从头定向**：不补的那一拍、来源切换、配置或舵机标定变了、两次调用
 *     间隔超过 GAP_RESET_MS（标定手势期间整段不调用）都复位回 d = 0。
 *   - 脉宽 ↔ 倾转角的换算与夹限：用分配器同一条律（2000 µs 对 180°，按标定中位与
 *     pulse_sign），补完夹回标定 min/max。
 *   - 配置的跨任务交接（命令任务写待生效配置，控制拍取用）与计数，供 `BACKLASH ?`。
 *
 * 运行在 StabilizerTask：Apply 每个控制拍一次，在舵机仲裁（验收/反馈台架/点动）之后、
 * 写硬件之前原地改脉宽。调用方在 Apply 之前把补偿前的目标记下来（保持目标、飞行日志
 * servo_*_us 与辨识记录都是补偿前的口径；servo_*_sent_us 是补偿后实际发出的）。
 * **本模块不许包含 app_control.h、不许调 APP_Control_QueueText**（控制拍不打字）；
 * 文字回复全在 app_cmd_backlash.c，由命令任务出。
 *
 * 阶段一只在 RAM：重启回到下面的编译期默认值（2026-09-28 起为开）。
 */

#define APP_SERVO_BACKLASH_DEFAULT_ENABLE     1U    /* 2026-09-28 杆上 A/B 通过（3° 等效延迟 100→66 ms），作者定为开机即开 */
#define APP_SERVO_BACKLASH_DEFAULT_HALF_MRAD  20U   /* 俯仰舵机实测 19.9 mrad；横滚未测，暂同值 */
#define APP_SERVO_BACKLASH_DEFAULT_THR_MRAD   5U    /* 与 DRV_SERVO_BACKLASH_THRESHOLD_DEFAULT_RAD 同值 */
#define APP_SERVO_BACKLASH_HALF_MRAD_MAX      87U   /* 与 DRV_SERVO_BACKLASH_HALF_GAP_MAX_RAD 同值 */
#define APP_SERVO_BACKLASH_THR_MRAD_MIN       1U
#define APP_SERVO_BACKLASH_THR_MRAD_MAX       50U   /* 与 DRV_SERVO_BACKLASH_THRESHOLD_MAX_RAD 同值 */
#define APP_SERVO_BACKLASH_GAP_RESET_MS       50U   /* 控制拍 2 ms；断这么久说明中间有别人接管过舵机 */

#define APP_SERVO_BACKLASH_ALPHA  0U   /* PWM ch1，横滚倾转（DRV_COAX_CTRL_SERVO_ALPHA_INDEX） */
#define APP_SERVO_BACKLASH_BETA   1U   /* PWM ch2，俯仰倾转（DRV_COAX_CTRL_SERVO_BETA_INDEX） */

/* 这一拍舵机脉宽是谁产生的。由稳定环在算出脉宽的那一支里填，默认（0）= 不补。 */
typedef enum {
    APP_SERVO_BACKLASH_SRC_NONE = 0,     /* 回中、上电无姿态、老 IDENT …… */
    APP_SERVO_BACKLASH_SRC_CONTROLLER,   /* 在飞控制器（含 IMU 短暂失效时保持上一目标） */
    APP_SERVO_BACKLASH_SRC_SYSID         /* 光杆辨识正在跑（APP_SysId_IsRunning） */
} APP_ServoBacklashSource;

typedef struct {
    uint8_t  enable;
    uint16_t half_mrad[2];   /* 下标 APP_SERVO_BACKLASH_ALPHA/BETA，0..87 */
    uint16_t thr_mrad;       /* 1..50 */
} APP_ServoBacklashConfig;

typedef struct {
    APP_ServoBacklashConfig cfg;
    uint8_t  active;          /* 最近一拍在补偿 */
    uint8_t  source;          /* 最近一拍的来源（APP_ServoBacklashSource） */
    int8_t   direction[2];    /* 最近一拍各舵机的 d：+1/−1/0 */
    int16_t  offset_us[2];    /* 最近一拍加在脉宽上的量（夹限之后） */
    uint32_t reversals[2];    /* 判出的换向次数（首次定向不算） */
    uint32_t resets;          /* 丢掉已有方向的次数：退出补偿、来源切换、改配置/标定、断档 */
    uint32_t nonfinite;       /* 非有限输入（直通并复位） */
    uint32_t ticks;           /* 补偿过的控制拍 */
    uint32_t clamped;         /* 补完被夹回标定端点的舵机·拍 */
} APP_ServoBacklashStatus;

void    APP_ServoBacklash_Init(void);
void    APP_ServoBacklash_DefaultConfig(APP_ServoBacklashConfig *cfg);
uint8_t APP_ServoBacklash_ConfigValid(const APP_ServoBacklashConfig *cfg);
uint8_t APP_ServoBacklash_RequestConfig(const APP_ServoBacklashConfig *cfg); /* 命令任务；下一拍生效 */
void    APP_ServoBacklash_GetConfig(APP_ServoBacklashConfig *out);          /* 待生效的那份（若有） */
void    APP_ServoBacklash_GetStatus(APP_ServoBacklashStatus *out);          /* 任意任务 */
/* 纯策略：这个来源、这个解锁状态、有没有人接管，本拍该不该补（不看开关）。 */
uint8_t APP_ServoBacklash_SourceCompensated(APP_ServoBacklashSource source, uint8_t armed,
                                            uint8_t override);
/*
 * StabilizerTask，每个控制拍一次，写硬件之前。override = 验收覆盖 / 反馈台架 / 地面点动
 * 接管了这一拍。不补时两路脉宽一个字节都不动。
 */
void    APP_ServoBacklash_Apply(uint32_t now_ms, APP_ServoBacklashSource source, uint8_t armed,
                                uint8_t override, uint16_t *alpha_pulse_us, uint16_t *beta_pulse_us);
/* 来源名（none/controller/sysid），命令面与溯源行共用。 */
const char *APP_ServoBacklash_SourceName(uint8_t source);
/* implemented in app_cmd_backlash.c */
uint8_t APP_ServoBacklash_Command(char **tokens, uint32_t count);
void    APP_ServoBacklash_ReportProvenance(uint16_t run_id);

#endif /* APP_SERVO_BACKLASH_H */
