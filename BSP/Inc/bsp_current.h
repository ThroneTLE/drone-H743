#ifndef BSP_CURRENT_H
#define BSP_CURRENT_H
#include <stdint.h>
typedef enum { BSP_CURRENT_OK=0, BSP_CURRENT_NOT_READY, BSP_CURRENT_TIMEOUT, BSP_CURRENT_ERROR } BSP_CurrentStatus;
/* messageTask only. ADC1/PC1 current + PC0 voltage, 16-bit polling, no DMA or IRQ.
 * ADC internal calibration is not sensor/current scale calibration.
 */
BSP_CurrentStatus BSP_Current_Init(void);
/* Bounded polling (1 ms); failure leaves raw unchanged. No automatic re-init. */
BSP_CurrentStatus BSP_Current_Read(uint32_t *raw);
typedef struct { uint32_t raw, sequence; BSP_CurrentStatus status; } BSP_VoltageSample;
/* Same task reads the voltage from the latest completed ADC pair; never starts ADC. */
void BSP_Current_GetVoltageSample(BSP_VoltageSample *out);

/*
 * 电流输入脚（PC1）接线自检。
 *
 * 为什么需要它：ADC 读数为 0 **无法区分**两件事——"电流真的是 0" 和
 * "Curr 信号压根没接到这个脚"。doc/current-sensor.md 一开始就写明了这条缺口。
 * 浮空脚在电机 PWM 下拾取几 mV EMI，看起来就像"有一点点电流在抖"。
 *
 * 判据：把脚临时切成数字输入，分别带上拉和下拉各读一次电平。
 *   - 两次电平**不同** → 脚跟着拉电阻走，没有东西驱动它 = 浮空。
 *   - 两次电平**相同** → 被低阻源（电调的电流检测运放输出）钉住 = 接线通。
 * 这是二值判断，不依赖"几 mV 算不算信号"的阈值。
 *
 * **不能用"模拟模式 + 内部上拉"来做这件事**：STM32 在 analog mode 下 PUPDR 的
 * 上下拉是被禁用的，那样测出来恒为浮空，等于自欺。
 *
 * 时序（稳定等待）归调用方：Curr 线上通常有滤波电容，几十 kΩ 的内部拉电阻配
 * 100 nF 的时间常数是毫秒级，等不够会把"浮空"误判成"被驱动"。BSP 这层不做延时、
 * 不碰 RTOS。
 *
 * RESTORE 会把脚恢复成模拟输入并重新初始化 ADC。进入 PULLUP/PULLDOWN 之后到
 * RESTORE 之前，BSP_Current_Read() 一律返回 NOT_READY——那段时间 PC1 不是模拟脚，
 * 让它继续产出"读数"就是在编数据。
 */
typedef enum {
    BSP_CURRENT_PIN_PROBE_PULLUP = 0,
    BSP_CURRENT_PIN_PROBE_PULLDOWN,
    BSP_CURRENT_PIN_PROBE_RESTORE
} BSP_CurrentPinProbeMode;

BSP_CurrentStatus BSP_Current_SetPinProbeMode(BSP_CurrentPinProbeMode mode);

/* 只在 PULLUP/PULLDOWN 模式下有意义；其余情况返回 0。 */
uint8_t BSP_Current_ReadPinLevel(void);

/*
 * 连续转换取证：一次序列里连读 count 次，把每一次的原始值原样返回。
 *
 * 为什么需要它：2026-09-21 出厂 PX4 基准测出 PC1 有 1226 LSB，而本模块读到约
 * 5 LSB——同一引脚、同一 ADC，差 240 倍，且**电压（第二次转换）完全正确**。
 * 首要嫌疑是"每对采样后 HAL_ADC_Stop() 禁用 ADC，重新使能后的第一次转换不可靠"，
 * 而那次恰好是电流通道。但那是推断不是判据。
 *
 * 本函数把推断变成可观测的事实：2 个 rank 配 discontinuous=1 时，序列应当是
 * rank1,rank2,rank1,rank2,…，即 `[电流, 电压, 电流, 电压, …]`。于是
 *   - 第 1 个异常而第 3 个正常 → "重新使能后首次转换不可靠"成立；
 *   - 奇数位全都异常 → 问题跟通道走，不跟次序走，另找原因；
 *   - 读出来根本不是交替 → rank 假设本身错了。
 * 三种结果指向三个完全不同的修法，猜是猜不出来的。
 *
 * 只读诊断，但会让序列停在半路，所以返回前标记位置不明，下次正常采样自行复位。
 */
#define BSP_CURRENT_SEQ_BURST_MAX 8U
BSP_CurrentStatus BSP_Current_SeqBurst(uint32_t *out, uint32_t count,
                                       uint32_t *out_done);

#endif
