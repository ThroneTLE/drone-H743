#ifndef DRV_SERVO_BACKLASH_H
#define DRV_SERVO_BACKLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 舵机 / 连杆回差（空程）的逆补偿：**纯算法**，没有寄存器、没有 RTOS、没有文件级
 * 可变量；一个舵机通道一份状态，由调用者持有。策略（哪些指令补、什么时候复位、
 * 参数怎么交接）在 App/Src/app_servo_backlash.c。
 *
 * ──────────────── 信号契约（D4-3） ────────────────
 *
 * 输入 cmd 与输出都是同一个舵机通道的倾转角 [rad]，零点与符号由调用者定（本工程
 * 接的是"舵机标定中位为 0、按标定 pulse_sign 折成机构倾转"的角度），本模块只要求
 * 两者同一口径。回差对称，所以符号取反整体不影响结果。
 *
 * ──────────────── 对象模型 ────────────────
 *
 * 舵盘 h 与负载 y 之间有 ±b 的空程（b = 半宽）：舵盘推着负载走时 y = h ∓ b，
 * 舵盘在空程里时负载不动。要让负载停在 cmd，舵盘就得多走 b：正向推时 h = cmd + b，
 * 反向推时 h = cmd − b。输出 = cmd + d·b，d 是"当前在往哪边推"。
 *
 * ──────────────── 为什么换向那一拍故意跳 2b ────────────────
 *
 * 换向时舵盘要穿过整条空程才重新贴上负载；这段路上它**推不动负载**，也就不产生
 * 任何反作用力矩。越快穿过去，负载越早跟上新指令，所以直接一拍跳 2b——跳的这段
 * 是"空走"，不会在机体上激起冲击。不补偿时同一段空程是靠指令慢慢爬过去的：
 * 小幅修正全落在空程里，舵机在动、负载不动，加上积分就是极限环。
 *
 * ──────────────── 为什么要迟滞 ────────────────
 *
 * 判换向不看"这一拍比上一拍小"，看"从本方向走到的极值 r 往回退了超过阈值"。
 * 控制指令里带着陀螺噪声：没有迟滞，噪声每抖一下方向就翻一次，舵机被 ±b 地来回砸
 * （本机 2b ≈ 0.04 rad，约 25 µs 脉宽），比回差本身还糟。阈值要大于指令噪声的峰峰值——
 * 峰峰值低于阈值的噪声叠在任何单调走势上都不会判出换向；代价是真换向晚阈值那么一点。
 *
 * ──────────────── 状态机 ────────────────
 *
 *   复位后第一个样本：d = 0，输出 = cmd（不加偏置），r = cmd。
 *   d = 0：cmd 离 r 超过阈值才定向（d = ±1，r = cmd）；之前一直原样输出。
 *   d = +1：cmd > r 则 r = cmd；cmd < r − 阈值则 d = −1、r = cmd（记一次换向）。
 *   d = −1：对称。
 *   输出 = cmd + d·b。
 *
 * 关（enable = 0）时逐位直通并清状态；非有限输入原样直通、清状态并计数。
 */

#define DRV_SERVO_BACKLASH_HALF_GAP_MAX_RAD       0.087f  /* 5°：再大就是连杆松脱，不是回差 */
#define DRV_SERVO_BACKLASH_THRESHOLD_DEFAULT_RAD  0.005f  /* ≈0.29° */
#define DRV_SERVO_BACKLASH_THRESHOLD_MAX_RAD      0.05f   /* ≈2.9°：再大真换向就晚得离谱 */

typedef struct {
    float   half_gap_rad;    /* b，[0, HALF_GAP_MAX]；0 = 只跟踪方向不加偏置 */
    float   threshold_rad;   /* 换向迟滞，(0, THRESHOLD_MAX] */
    uint8_t enable;          /* 0 = 逐位直通 */
} DRV_ServoBacklashConfig;

typedef struct {
    DRV_ServoBacklashConfig cfg;
    int8_t   direction;        /* +1 / −1；0 = 未定向（复位后） */
    uint8_t  primed;           /* 复位后见过第一个样本（reference_rad 有效） */
    float    reference_rad;    /* r：本方向走到的极值；未定向时是复位后的第一个样本 */
    uint32_t reversal_count;   /* 定向之后真正判出的换向次数（首次定向不算） */
    uint32_t nonfinite_count;
} DRV_ServoBacklash;

void    DRV_ServoBacklash_DefaultConfig(DRV_ServoBacklashConfig *cfg);   /* b=0、阈值 5 mrad、关 */
uint8_t DRV_ServoBacklash_ConfigValid(const DRV_ServoBacklashConfig *cfg);
/* 配置不合法时装默认配置（关）并返回 0；计数清零，状态复位。 */
uint8_t DRV_ServoBacklash_Init(DRV_ServoBacklash *bl, const DRV_ServoBacklashConfig *cfg);
/* 回到 d = 0：下一个样本原样输出并重新定向。计数不动。 */
void    DRV_ServoBacklash_Reset(DRV_ServoBacklash *bl);
/* 一拍：返回发给舵机的角度 [rad]。bl == NULL 时原样返回 cmd。 */
float   DRV_ServoBacklash_Step(DRV_ServoBacklash *bl, float cmd_rad);

#ifdef __cplusplus
}
#endif

#endif /* DRV_SERVO_BACKLASH_H */
