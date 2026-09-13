#include "app_tasks.h"
#include "app_background.h"
#include "app_message.h"
#include "app_led.h"
#include "app_sensor.h"
#include "app_gps.h"
#include "app_mag.h"
#include "app_optical_flow.h"
#include "app_uart.h"
#include "app_maint_uart.h"

osThreadId_t LEDTaskHandle;

/*
 * 栈 256 字 = 1 KB。先按 128 字写，实测 `RTOS?` 报 free_stack_words=18——
 * 只剩 72 字节余量。任务体本身只有一个 Σ-Δ 调制，但每 20 ms 要问一次光流健康，
 * `APP_OPTICAL_FLOW_Status` 是个整结构体、按值放在栈上。余量太薄的代价不是
 * 偶尔出错，是栈溢出钩子里的死循环，而那时灯已经不亮了、串口也没了。
 *
 * **优先级必须是 BelowNormal，不能是 Low。** 2026-09-12 先按 Low 写，实测
 * `LED? ticks=0` —— 任务建起来了、`RTOS?` 报 state=1(Ready)，但**一整拍都没跑过**。
 * Low 低于 messageTask / backgroundTask / TELEM（都是 BelowNormal），而
 * backgroundTask 在 `RTOS?` 里长期是 Ready，于是 Low 永远轮不到。
 *
 * 这与 2026-09-11 遥测任务那次是同一个坑（见 freertos.c 里 VOFA_Task 的注释：
 * "stream=1 却一个字节都不来"）。按固定节拍产出的任务不该靠捡剩余时间片——
 * 灯尤其如此：它停了只会僵在某个电平上，看起来完全像一盏正常亮着的灯。
 */
static const osThreadAttr_t LEDTask_attributes = {
    .name = "LEDTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal,
};

void APP_Task_LED_Init(void)
{
    APP_LED_Task_Init();
}

void APP_Task_LED_Step(void)
{
    APP_LED_Task_Step();
}

static void APP_Task_LED_Run(void *argument)
{
    uint32_t next = osKernelGetTickCount();

    (void)argument;
    for (;;) {
        /* osDelayUntil 而不是 osDelay：后者是"从现在起再等 1 ms"，本任务被更高
         * 优先级挤掉多久，误差就累积多久。调光靠的是节拍密度，节拍慢了就是暗了。 */
        next += 1U;
        (void)osDelayUntil(next);
        APP_LED_Task_Step();
    }
}

void APP_Task_LED_Start(void)
{
    LEDTaskHandle = osThreadNew(APP_Task_LED_Run, NULL, &LEDTask_attributes);
}

void APP_Task_GPS_Init(void)
{
    APP_GPS_Init();
}

void APP_Task_GPS_Step(void)
{
    APP_GPS_Step();
}

void APP_Task_OpticalFlow_Init(void)
{
    APP_OpticalFlow_Init();
}

void APP_Task_OpticalFlow_Step(void)
{
    APP_OpticalFlow_Step();
}

void APP_Task_MAG_Init(void)
{
    APP_MAG_Init();
}

void APP_Task_MAG_Step(void)
{
    APP_MAG_Step();
}

void APP_Task_Message_Init(void)
{
    APP_Message_Task_Init();
}

void APP_Task_Message_Step(void)
{
    APP_Message_Task_Step();
}

void APP_Task_UART_Init(void)
{
    APP_UART_Task_Init();
}

void APP_Task_UART_Step(void)
{
    APP_UART_Task_Step();
}

void APP_Task_MaintUART_Init(void)
{
    APP_MaintUART_Init();
}

void APP_Task_MaintUART_Step(void)
{
    APP_MaintUART_Step();
}

void APP_Task_Background_Init(void)
{
    APP_Background_Init();
}

void APP_Task_Background_Step(void)
{
    APP_Background_Step();
}
