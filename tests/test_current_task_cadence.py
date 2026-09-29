"""Actual task wrappers/monitor keep sampling while the storage worker is occupied."""
import ast
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def test_real_task_chain_current_survives_slow_storage(tmp_path):
    # Reuse the existing hardware seam, not a second ADC implementation.
    nodes=ast.parse((ROOT/'tests/test_current_runtime.py').read_text(encoding='utf-8')).body
    adc=next(n.value.value for n in nodes if isinstance(n,ast.Assign)
             and any(isinstance(t,ast.Name) and t.id=='ADC_STUB' for t in n.targets))
    (tmp_path/'adc.h').write_text(adc)
    (tmp_path/'cmsis_os2.h').write_text('#include "app_tasks.h"\n')
    (tmp_path/'app_control.h').write_text('void APP_Control_QueueText(const char*,...);\n')
    (tmp_path/'app_tasks.h').write_text(r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void *osThreadId_t;
typedef int osPriority_t;
typedef struct {const char *name;uint32_t stack_size;osPriority_t priority;} osThreadAttr_t;
#define osPriorityBelowNormal 16
#define osOK 0
void *osThreadNew(void (*)(void*),void*,const osThreadAttr_t*);
uint32_t osKernelGetTickCount(void);
uint32_t osKernelGetTickFreq(void);
int osDelayUntil(uint32_t);
int osDelay(uint32_t);
int osMessageQueueGet(void*,void*,uint8_t*,uint32_t);
int osMessageQueuePut(void*,const void*,uint8_t,uint32_t);
extern void *backgroundReqQueueHandle,*backgroundRespQueueHandle;
void APP_Task_Message_Init(void);
void APP_Task_Message_Step(void);
void APP_Task_Background_Init(void);
void APP_Task_Background_Step(void);
''')
    for name,prefix in (('led','APP_LED_Task'),('gps','APP_GPS'),('mag','APP_MAG'),
                        ('optical_flow','APP_OpticalFlow'),('uart','APP_UART_Task'),
                        ('maint_uart','APP_MaintUART')):
        (tmp_path/f'app_{name}.h').write_text(f'void {prefix}_Init(void);\nvoid {prefix}_Step(void);\n')
    for name in ('app_sensor.h','app_baro.h','app_flash.h','app_messages.h','app_proto.h'):
        (tmp_path/name).write_text('/* Unused in the default message worker. */\n')
    (tmp_path/'app_flight_log.h').write_text('#define APP_FLIGHT_LOG_BACKGROUND_IDLE_MS 5U\nvoid APP_FlightLog_BackgroundStep(void);\n')
    (tmp_path/'app_flash_service.h').write_text(r'''
#include <stdint.h>
typedef enum {APP_FLASH_SERVICE_OK,APP_FLASH_SERVICE_TIMEOUT,APP_FLASH_SERVICE_INVALID_ARG,APP_FLASH_SERVICE_BUSY,APP_FLASH_SERVICE_ERROR} APP_FlashService_Status;
APP_FlashService_Status APP_FlashService_ReadDataFast(uint32_t,uint8_t*,uint32_t);
APP_FlashService_Status APP_FlashService_WriteData(uint32_t,uint8_t*,uint32_t);
APP_FlashService_Status APP_FlashService_EraseSector(uint32_t);
''')
    (tmp_path/'svc_param.h').write_text(r'''
typedef enum {SVC_PARAM_STATUS_OK,SVC_PARAM_STATUS_NO_VALID_RECORD,SVC_PARAM_STATUS_INVALID_ARG,SVC_PARAM_STATUS_NOT_READY} SVC_ParamStatus;
void SVC_Param_Init(void);
SVC_ParamStatus SVC_Param_LoadFromFlash(void);
SVC_ParamStatus SVC_Param_SavePendingToFlash(void);
''')
    source=r'''
#include "adc.h"
#include "app_current.h"
#include "app_tasks.h"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
int test_adc;ADC_HandleTypeDef hadc1={ADC1,{16,2,0,0,0,0,1,1,1,1}};
void *backgroundReqQueueHandle=(void*)1,*backgroundRespQueueHandle=(void*)2;
static uint32_t now,raw,calibrations,adc_calls,max_delay,hold_until;
static uint32_t delays,battery_inits,battery_steps,battery_telemetry,battery_saw_samples,thrust_lut_steps;
static int read_error;
#define UNUSED_TASK(name) void name(void){assert(!"unexpected task call");}
UNUSED_TASK(APP_LED_Task_Init) UNUSED_TASK(APP_LED_Task_Step)
UNUSED_TASK(APP_GPS_Init) UNUSED_TASK(APP_GPS_Step)
UNUSED_TASK(APP_MAG_Init) UNUSED_TASK(APP_MAG_Step)
UNUSED_TASK(APP_OpticalFlow_Init) UNUSED_TASK(APP_OpticalFlow_Step)
UNUSED_TASK(APP_UART_Task_Init) UNUSED_TASK(APP_UART_Task_Step)
UNUSED_TASK(APP_MaintUART_Init) UNUSED_TASK(APP_MaintUART_Step)
void *osThreadNew(void (*fn)(void*),void *arg,const osThreadAttr_t *attr){(void)fn;(void)arg;(void)attr;assert(0);return NULL;}
int osDelayUntil(uint32_t ticks){(void)ticks;assert(0);return 0;}
uint32_t SVC_Timestamp_Ms(void){return now;}
uint32_t BSP_Critical_Enter(void){return 0;}
void BSP_Critical_Exit(uint32_t x){(void)x;}
uint32_t osKernelGetTickCount(void){return now;}
uint32_t osKernelGetTickFreq(void){return 1000;}
int osDelay(uint32_t ticks){assert(ticks&&ticks<1000);if(ticks>max_delay)max_delay=ticks;now+=ticks;delays++;return 0;}
int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*a,uint32_t b,uint32_t c){(void)a;(void)b;(void)c;calibrations++;return 0;}
int HAL_ADC_Start(ADC_HandleTypeDef*a){(void)a;adc_calls++;return 0;}
int HAL_ADC_PollForConversion(ADC_HandleTypeDef*a,uint32_t timeout){(void)a;assert(timeout==1);return read_error;}
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*a){(void)a;return 0;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*a){(void)a;return raw;}
int HAL_ADC_Stop(ADC_HandleTypeDef*a){(void)a;return 0;}
void APP_Control_QueueText(const char *fmt,...){va_list a;va_start(a,fmt);vprintf(fmt,a);va_end(a);}
void SVC_Param_Init(void){}
/* 这三个桩不是空的：电池的每个模块各有自己的单元测试，但**没人测装配**——
 * 把 app_message.c 里那两行调用删掉，整套回归照样全绿，而电压永远不更新、
 * 遥测一帧都不发。所以在这里数调用次数和顺序，把"接线"本身钉住。 */
void APP_Battery_Init(void){battery_inits++;}
void APP_Battery_Step(void){
    APP_CurrentSnapshot c;APP_Current_GetSnapshot(&c);battery_saw_samples=c.samples;battery_steps++;}
void APP_Battery_TelemetryStep(void){assert(battery_telemetry<battery_steps);battery_telemetry++;}
/* 推力查补表的电量电压估计紧跟每次电池采样（app_thrust_lut.c）。 */
void APP_ThrustLut_Step(void){assert(thrust_lut_steps<battery_steps);thrust_lut_steps++;}
int SVC_Param_LoadFromFlash(void){return 0;}
int SVC_Param_SavePendingToFlash(void){return 0;}
int APP_FlashService_ReadDataFast(uint32_t a,uint8_t*b,uint32_t c){(void)a;(void)b;(void)c;return 0;}
int APP_FlashService_WriteData(uint32_t a,uint8_t*b,uint32_t c){(void)a;(void)b;(void)c;return 0;}
int APP_FlashService_EraseSector(uint32_t a){(void)a;return 0;}
int osMessageQueueGet(void*q,void*p,uint8_t*r,uint32_t ticks){(void)q;(void)p;(void)r;assert(ticks==5);now+=ticks;return -1;}
int osMessageQueuePut(void*q,const void*p,uint8_t r,uint32_t ticks){(void)q;(void)p;(void)r;assert(!ticks);return 0;}
void APP_FlightLog_BackgroundStep(void){
    /* The storage worker stays inside slow I/O; the scheduler runs messageTask.
     * No board storage or scheduler is used. Both task entry chains are real C. */
    while(now<hold_until)APP_Task_Message_Step();
}
int main(void){
    APP_Task_Message_Init();APP_Task_Background_Init();
    assert(calibrations==1);hold_until=51282;
    APP_Task_Background_Step();
    APP_CurrentSnapshot s;APP_Current_GetSnapshot(&s);
    printf("slow-storage duration=%lu samples=%lu age=%lu valid=%u errors=%lu\n",
      (unsigned long)now,(unsigned long)s.samples,(unsigned long)s.age_ms,s.reading.valid,(unsigned long)s.errors);
    fflush(stdout);
    assert(s.samples>=2500 && s.reading.valid && s.age_ms<=25 && s.errors==0);
    assert(s.reading.raw==0 && s.reading.adc_v==0 && s.reading.current_a==0);
    assert(max_delay==20 && calibrations==1);
    uint32_t calls=adc_calls;APP_Current_Report();assert(adc_calls==calls);
    now+=251;APP_Current_GetSnapshot(&s);assert(!s.reading.valid && isnan(s.reading.current_a));
    read_error=HAL_TIMEOUT;APP_Task_Message_Step();APP_Current_GetSnapshot(&s);assert(!s.reading.valid&&s.errors==1);
    read_error=0;raw=12000;APP_Task_Message_Step();APP_Current_GetSnapshot(&s);assert(s.reading.valid && s.samples>2500);
    /* 装配闸门：messageTask 每一拍都必须走电池采样和遥测，Init 恰好一次；
     * battery_saw_samples 等于本拍采样后的计数，说明电池步跑在电流步之后。 */
    assert(battery_inits==1 && battery_steps==delays && battery_telemetry==delays && thrust_lut_steps==delays);
    assert(battery_saw_samples==s.samples);
    calls=adc_calls;APP_Task_Background_Step();assert(adc_calls==calls); /* single ADC owner */
    puts("cadence, real zero, timeout recovery, stale gate, single owner: passed");return 0;
}
'''
    (tmp_path/'test.c').write_text(source)
    command=[shutil.which('gcc'),'-std=c11','-O2','-Wall','-Wextra','-Werror',
             '-ffunction-sections','-fdata-sections','-Wl,--gc-sections']
    for path in (tmp_path,ROOT/'App/Inc',ROOT/'Driver/Inc',ROOT/'BSP/Inc',ROOT/'Services/Inc'):
        command+=['-I',str(path)]
    command+=[str(tmp_path/'test.c')]+[str(ROOT/path) for path in (
        'App/Src/app_tasks.c','App/Src/app_message.c','App/Src/app_background.c',
        'App/Src/app_current.c','BSP/Src/bsp_current.c','Driver/Src/drv_current.c','Driver/Src/drv_current_filter.c')]
    exe=tmp_path/'cadence.exe';command+=['-lm','-o',str(exe)]
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    print(result.stdout, end='')
