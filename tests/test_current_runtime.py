"""Compile actual ADC BSP + current monitor against a software-only HAL seam."""
from pathlib import Path
import shutil
import subprocess
import re

ROOT=Path(__file__).resolve().parents[1]
ADC_STUB=r"""
#ifndef TEST_CURRENT_ADC_H
#define TEST_CURRENT_ADC_H
#include <stdint.h>
typedef struct { uint32_t Resolution,NbrOfConversion,ContinuousConvMode,ConversionDataManagement,LeftBitShift,OversamplingMode; } ADC_InitTypeDef;
typedef struct { void *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
extern ADC_HandleTypeDef hadc1;
extern int test_adc;
#define ADC1 (&test_adc)
#define ADC_RESOLUTION_16B 16U
#define DISABLE 0U
#define ADC_CONVERSIONDATA_DR 0U
#define ADC_LEFTBITSHIFT_NONE 0U
#define ADC_CALIB_OFFSET_LINEARITY 1U
#define ADC_SINGLE_ENDED 0U
#define HAL_ADC_ERROR_NONE 0U
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_TIMEOUT 3
typedef int HAL_StatusTypeDef;
int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*,uint32_t,uint32_t);
int HAL_ADC_Start(ADC_HandleTypeDef*);
int HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*);
int HAL_ADC_Stop(ADC_HandleTypeDef*);
#endif
"""
HARNESS=r"""
#include "adc.h"
#include "app_current.h"
#include "bsp_current.h"
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
int test_adc;
ADC_HandleTypeDef hadc1={ADC1,{16,1,0,0,0,0}};
static uint32_t raw=12000,now,starts,stops,error;
static int calibration_rc,start_rc,poll_rc,stop_rc;
static char report[512];
#define CHECK(c) do { if(!(c)){fprintf(stderr,"line %d: %s\n",__LINE__,#c);return 1;} } while(0)
int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*a,uint32_t b,uint32_t c){(void)a;(void)b;(void)c;return calibration_rc;}
int HAL_ADC_Start(ADC_HandleTypeDef*a){(void)a;starts++;return start_rc;}
int HAL_ADC_PollForConversion(ADC_HandleTypeDef*a,uint32_t timeout){(void)a;if(timeout!=1)return HAL_ERROR;return poll_rc;}
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*a){(void)a;return error;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*a){(void)a;return raw;}
int HAL_ADC_Stop(ADC_HandleTypeDef*a){(void)a;stops++;return stop_rc;}
uint32_t SVC_Timestamp_Ms(void){return now;}
uint32_t BSP_Critical_Enter(void){return 0;}
void BSP_Critical_Exit(uint32_t x){(void)x;}
void APP_Control_QueueText(const char*fmt,...){va_list a;va_start(a,fmt);vsnprintf(report,sizeof(report),fmt,a);va_end(a);}
int main(void){
    APP_CurrentSnapshot s;
    APP_Current_GetSnapshot(&s);CHECK(!s.reading.valid && isnan(s.reading.current_a));
    APP_Current_Init();APP_Current_GetSnapshot(&s);CHECK(!s.reading.valid && s.age_ms==UINT32_MAX);
    APP_Current_Step();APP_Current_GetSnapshot(&s);
    CHECK(s.reading.valid && s.samples==1 && !s.reading.calibrated && s.reading.raw==12000);
    CHECK(starts==1 && stops==1);
    APP_Current_Step();CHECK(starts==1);
    now=19;APP_Current_Step();CHECK(starts==1);
    now=20;APP_Current_Step();CHECK(starts==2);
    now=271;APP_Current_GetSnapshot(&s);CHECK(!s.reading.valid && isnan(s.reading.current_a) && s.age_ms==251);
    APP_Current_Report();CHECK(strstr(report,"valid=0") && strstr(report,"calibrated=0") && strstr(report,"source=AM32_55A_CURR"));
    poll_rc=HAL_TIMEOUT;APP_Current_Step();APP_Current_GetSnapshot(&s);
    CHECK(s.adc_status==BSP_CURRENT_TIMEOUT && !s.reading.valid && stops==3 && s.samples==2);
    poll_rc=0;now+=20;raw=0;APP_Current_Step();APP_Current_GetSnapshot(&s);CHECK(s.reading.valid && s.reading.current_a==0);
    now+=20;raw=65535;APP_Current_Step();APP_Current_GetSnapshot(&s);CHECK(s.reading.saturated && !s.reading.valid && isnan(s.reading.current_a));
    uint32_t out=123;stop_rc=HAL_ERROR;CHECK(BSP_Current_Read(&out)==BSP_CURRENT_ERROR && out==123);stop_rc=0;
    start_rc=HAL_ERROR;CHECK(BSP_Current_Read(&out)==BSP_CURRENT_ERROR && out==123);start_rc=0;
    error=1;CHECK(BSP_Current_Read(&out)==BSP_CURRENT_ERROR && out==123);error=0;
    CHECK(BSP_Current_Read(NULL)==BSP_CURRENT_ERROR);
    raw=65536;CHECK(BSP_Current_Read(&out)==BSP_CURRENT_ERROR && out==123);
    now=UINT32_MAX-10;raw=12000;APP_Current_Init();APP_Current_Step();
    now=9;APP_Current_Step();APP_Current_GetSnapshot(&s);CHECK(s.samples==2 && s.reading.valid && s.age_ms==0);
    calibration_rc=HAL_ERROR;APP_Current_Init();uint32_t old=starts;now+=100;APP_Current_Step();APP_Current_GetSnapshot(&s);
    CHECK(!s.reading.valid && s.errors==1 && starts==old);
    CHECK(BSP_Current_Read(&out)==BSP_CURRENT_NOT_READY);
    calibration_rc=0;hadc1.Init.Resolution=12;CHECK(BSP_Current_Init()==BSP_CURRENT_ERROR);
    hadc1.Init.Resolution=16;hadc1.Init.OversamplingMode=1;CHECK(BSP_Current_Init()==BSP_CURRENT_ERROR);
    puts("current ADC/monitor: cadence, stale, timeout, clipping, calibration failure and wrap passed");return 0;
}
"""


def test_real_adc_and_monitor_on_host(tmp_path):
    (tmp_path/"adc.h").write_text(ADC_STUB)
    (tmp_path/"app_control.h").write_text('void APP_Control_QueueText(const char*,...);\n')
    (tmp_path/"current_test.c").write_text(HARNESS)
    exe=tmp_path/"current.exe"
    cmd=[shutil.which("gcc"),"-std=c11","-O2","-Wall","-Wextra","-Werror"]
    for inc in [tmp_path,ROOT/"Driver/Inc",ROOT/"BSP/Inc",ROOT/"App/Inc",ROOT/"Services/Inc"]:
        cmd += ["-I",str(inc)]
    cmd += [str(tmp_path/"current_test.c"),str(ROOT/"Driver/Src/drv_current.c"),str(ROOT/"BSP/Src/bsp_current.c"),str(ROOT/"App/Src/app_current.c"),"-lm","-o",str(exe)]
    r=subprocess.run(cmd,capture_output=True,text=True)
    assert r.returncode==0,r.stdout+r.stderr
    r=subprocess.run([str(exe)],capture_output=True,text=True)
    assert r.returncode==0,r.stdout+r.stderr


def test_current_polling_is_outside_control_and_report_is_snapshot_only():
    source=(ROOT/"App/Src/app_current.c").read_text(encoding="utf-8")
    report=source.split("void APP_Current_Report(void)")[1]
    assert "BSP_Current_Read(" not in report and "APP_Current_GetSnapshot(&s)" in report
    for name in ("app_stabilizer.c","app_sensor.c"):
        assert "BSP_Current_Read" not in (ROOT/"App/Src"/name).read_text(encoding="utf-8")
    assert "HAL_" not in source
