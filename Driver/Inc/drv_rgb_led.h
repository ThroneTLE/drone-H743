#ifndef DRV_RGB_LED_H
#define DRV_RGB_LED_H

#include <stdint.h>

/*
 * RGB 状态灯的**效果算法**：给一个图案和"现在几点"，算出此刻三个通道各该多亮。
 *
 * 本文件不含 HAL、不含 RTOS、不知道灯接在哪根脚上，也不知道什么颜色代表什么。
 * 引脚与极性在 `BSP/Src/bsp_rgb_led.c`，"什么状态亮什么"在 `App/Src/app_led.c`，
 * "同时有好几件事要说时听谁的"在 `Services/Src/svc_led.c`。
 *
 * 分开的理由很具体：颜色和节奏是会被反复调的（调到顺眼为止），而调它不该有
 * 碰错引脚、写错极性的风险；反过来换一块板、换成共阴接法，也不该动效果曲线。
 */

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} DRV_RgbColor;

typedef enum {
    DRV_RGB_EFFECT_OFF = 0,
    DRV_RGB_EFFECT_SOLID,    /* 常亮 */
    DRV_RGB_EFFECT_BLINK,    /* 方波：亮 on_ms、灭 off_ms，无限循环 */
    DRV_RGB_EFFECT_PULSES,   /* 闪 count 下，然后黑 gap_ms —— "数灯"用的就是它 */
    DRV_RGB_EFFECT_BREATHE   /* 呼吸：period_ms 一个来回 */
} DRV_RgbEffect;

typedef struct {
    DRV_RgbColor  color;
    DRV_RgbEffect effect;
    uint16_t      on_ms;      /* BLINK / PULSES：单次亮多久 */
    uint16_t      off_ms;     /* BLINK / PULSES：单次灭多久 */
    uint16_t      gap_ms;     /* PULSES：一组闪完之后黑多久 */
    uint16_t      period_ms;  /* BREATHE：一个完整来回多久 */
    uint8_t       count;      /* PULSES：一组闪几下 */
    uint8_t       dim;        /* BREATHE：最暗处的亮度，0 = 呼吸到全黑 */
} DRV_RgbPattern;

/*
 * 相位是 `now_ms % 周期` 算出来的，不保存上一次的时间，所以：
 *   - 同一个 now_ms 调多少次结果都一样（可测）；
 *   - 换图案时不需要"复位相位"，也就不会有一个忘记复位就卡死的状态。
 * 代价是换图案时节奏可能从中间接上，对状态灯无所谓。
 */
DRV_RgbColor DRV_RgbLed_Sample(const DRV_RgbPattern *pattern, uint32_t now_ms);

/*
 * 一阶 Σ-Δ 调制：把 0..255 的亮度铺到只有"亮/灭"两种状态的引脚上。
 *
 * 为什么不用普通 PWM：这三根脚（PE2/PE3/PE4）没有定时器复用，只能靠软件按固定
 * 节拍翻转，而节拍就 1 kHz。普通 PWM 在 1 kHz 下要 8 位分辨率意味着载波只有
 * 1000/256 ≈ 3.9 Hz——那不叫调光，那叫闪烁。Σ-Δ 把同样的平均占空比**尽量均匀地
 * 摊开**：亮度 128 就是一拍亮一拍灭（等效 500 Hz），亮度 255 恒亮、0 恒灭，
 * 只有很暗的时候脉冲才稀疏，而那时候灯本来就快看不见了。
 *
 * 返回位：bit0=R、bit1=G、bit2=B，1 表示"这一拍该亮"。
 */
typedef struct {
    uint16_t acc[3];
} DRV_RgbDither;

void DRV_RgbLed_DitherReset(DRV_RgbDither *state);
uint8_t DRV_RgbLed_Modulate(DRV_RgbDither *state, DRV_RgbColor color);

#define DRV_RGB_BIT_R 0x01U
#define DRV_RGB_BIT_G 0x02U
#define DRV_RGB_BIT_B 0x04U

#endif
