#ifndef DRV_CURRENT_FILTER_H
#define DRV_CURRENT_FILTER_H

#include <stdint.h>

/*
 * 电流块平均。
 *
 * 为什么需要它：AM32 的电流检测输出是**未滤波的分流器运放输出**，带着电机 PWM
 * 的大纹波；而采样是 50 Hz 异步单点。两者之间没有任何同步关系，所以每一拍采到
 * 的是纹波上的一个随机点——2026-09-21 实测表现为读数在 0~0.4 A 之间乱跳，而外部
 * 电流表同一时刻读 0.15 A。这不是"读数不准"，是**单点瞬时值对这个信号没有定义**。
 *
 * 为什么同时报 min/max 而不只报均值：只给平滑后的数，读数会立刻变好看，但看的人
 * 无法区分"信号本来就干净"和"纹波很大被抹平了"。纹波幅度是判断接线/滤波是否正常
 * 的一手信息，抹掉它等于把问题藏起来。均值供显示与标定，min/max 供诊断。
 *
 * 块平均而不是滑动平均/IIR：块平均的每个输出**只依赖一段互不重叠的样本**，
 * 拿它去和外部电流表对照时，"这个数对应哪段时间"是确定的。IIR 的输出永远拖着
 * 一条无限长的尾巴，标定时说不清它在平均什么。
 *
 * 本模块是纯函数 + 调用方持有的状态（decoupling-spec D3-3）：不含 HAL、不含 RTOS、
 * 不做 I/O、无文件级可变静态量，可在 PC 上直接单测。
 */

typedef struct {
    /* 累积中的块 */
    float    sum_a;
    float    block_min_a;
    float    block_max_a;
    uint16_t block_count;

    /* 最近一个**完整**块的结果；valid 为 0 时三者都没有意义 */
    float    mean_a;
    float    min_a;
    float    max_a;
    uint8_t  valid;

    uint16_t window;   /* 一个块需要多少个**有效**样本 */
    uint32_t blocks;   /* 已完成的块数，供诊断判断"这个均值是第几块" */
    uint32_t rejected; /* 被丢弃的非有限样本数 */
} DRV_CurrentFilter;

/*
 * window 为 0 时按 1 处理（等价于不平均），不会除零。
 * Init 会清空一切，包括历史结果。
 */
void DRV_CurrentFilter_Init(DRV_CurrentFilter *filter, uint16_t window);

/*
 * 送入一个样本。
 *
 * **非有限样本（NaN/Inf）不计入块**，只累加 rejected 计数：上游用 NaN 表示
 * "这一拍没有有效读数"（见 drv_current.h 的无效语义），把它当成 0 混进均值
 * 会让"传感器坏了"表现为"电流变小了"，这正是最该避免的伪装。
 *
 * 于是一个块的含义是"凑满 window 个**有效**样本"，而不是"window 个采样周期"。
 * 传感器时好时坏时，块会变长而不是变脏——这是有意的取舍。
 */
void DRV_CurrentFilter_Push(DRV_CurrentFilter *filter, float current_a);

/*
 * 丢弃累积中的块并让上一次结果失效。
 * 量程/比例/接线变化之后必须调用：跨越配置变更的平均没有物理意义。
 */
void DRV_CurrentFilter_Reset(DRV_CurrentFilter *filter);

#endif /* DRV_CURRENT_FILTER_H */
