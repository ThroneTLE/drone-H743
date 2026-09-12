#ifndef SVC_LED_H
#define SVC_LED_H

#include <stdint.h>

#include "drv_rgb_led.h"

/*
 * 状态灯服务：谁都可以说"我想让灯这样亮"，由本服务决定此刻听谁的。
 *
 * 为什么需要仲裁，而不是各处直接写灯：一盏灯同一时刻只能表达一件事，而想说话的
 * 至少有五处（解锁状态、标定流程、传感器健康、故障、上位机点名）。原来的做法是在
 * `app_led.c` 里排一串 if-else，谁压谁写死在语句顺序里——加一种状态就要去读懂整串
 * 链条，而且插错位置的后果是"某个状态永远显示不出来"，不报错、不崩溃，只是看不见。
 *
 * 现在改成：**源即优先级**。下面枚举的顺序就是仲裁顺序，越靠前越优先。每个源各占
 * 一格，互不覆盖；发布 NULL 表示"我没话说"，让位给后面的源。
 *
 * 新增源只能加在语义正确的位置上，加完必须重看一遍——插在中间会改变已有源之间的
 * 相对顺序，而那正是这里唯一的仲裁依据。`tests/test_led_service.py` 钉住了顺序。
 */
typedef enum {
    /* 上位机 / 维护口点名。压过一切自动逻辑：人正盯着这盏灯找是哪块板。 */
    SVC_LED_SOURCE_IDENTIFY = 0,
    /* 已解锁。桨随时可能转，这是整块板上最该被看见的事实。 */
    SVC_LED_SOURCE_ARMED,
    /* 标定 / 验收流程占用中，灯在跟流程走，不是在报飞行状态。 */
    SVC_LED_SOURCE_CALIBRATION,
    /* 解锁被拒 + 原因码。 */
    SVC_LED_SOURCE_BLOCKED,
    /* 不挡解锁、但值得知道的告警（例如光流没数据）。 */
    SVC_LED_SOURCE_WARNING,
    /* 常态：就绪、待命。 */
    SVC_LED_SOURCE_STATUS,
    /* 兜底心跳。永远有话说，所以放最后——它亮着只说明固件还在跑。 */
    SVC_LED_SOURCE_HEARTBEAT,
    SVC_LED_SOURCE_COUNT
} SVC_LedSource;

/*
 * 输出口子由调用方注册，服务自己不认识引脚。
 * 这样 `svc_led.c` 在 PC 上就能整份跑起来（见 tests/test_led_service.py），
 * 而不是只能靠读代码判断仲裁对不对。
 */
typedef void (*SVC_LedSink)(uint8_t bits);

void SVC_Led_Init(SVC_LedSink sink);

/* pattern = NULL 表示撤销本源的发布。 */
void SVC_Led_Publish(SVC_LedSource source, const DRV_RgbPattern *pattern);

/*
 * 发布并在 hold_ms 之后自动撤销。给"保存成功绿闪两秒"这种一次性提示用：
 * 不留定时撤销的话，每个发布方都要自己记一个到期时间，而漏掉一个的表现是
 * 灯**永远停在**那次提示上，把之后所有状态都盖住。
 */
void SVC_Led_PublishFor(SVC_LedSource source, const DRV_RgbPattern *pattern,
                        uint32_t hold_ms, uint32_t now_ms);

/* 按节拍调用：算出此刻该亮什么，调制成引脚电平，送给 sink。 */
void SVC_Led_Tick(uint32_t now_ms);

/* 诊断用：当前是谁在说话、算出来的颜色是什么。没人说话时源返回 COUNT。 */
SVC_LedSource SVC_Led_ActiveSource(void);
DRV_RgbColor SVC_Led_ActiveColor(void);

#endif
