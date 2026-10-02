"""Actual ADC pair, battery service, diagnostic encoder and periodic CRSF adapter."""
import ast
from pathlib import Path
import shutil
import struct
import subprocess
import pytest
from tools.panel_lib.battery_monitor import decode_battery

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def firmware_battery(tmp_path_factory):
    d=tmp_path_factory.mktemp('battery-c')
    nodes=ast.parse((ROOT/'tests/test_current_runtime.py').read_text(encoding='utf-8')).body
    adc=next(n.value.value for n in nodes if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='ADC_STUB' for t in n.targets))
    (d/'adc.h').write_text(adc)
    (d/'app_control.h').write_text('void APP_Control_QueueText(const char*,...);\n')
    (d/'app_stabilizer.h').write_text('#include <stdint.h>\ntypedef struct {uint8_t armed,published;} APP_Stabilizer_ArmStatus;\nuint8_t APP_Stabilizer_IsArmed(void);\nvoid APP_Stabilizer_GetArmStatus(APP_Stabilizer_ArmStatus*);\n')
    code=r'''
#include "adc.h"
#include "app_battery.h"
#include "app_current.h"
#include "app_proto.h"
#include "app_stabilizer.h"
#include "bsp_current.h"
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
int test_adc;ADC_HandleTypeDef hadc1={ADC1,{16,2,0,0,0,0,1,1,1,1}};
static uint32_t now,rank,next_rank,raws[2]={12000,11283},depth,starts,stops,calibrations;
static int fail_rank=-1,armed,tx_ok=1,preempt,stop_error;static unsigned sent;static uint16_t sent_v,sent_i;
static uint8_t packet[256];static uint16_t packet_size;
uint32_t SVC_Timestamp_Ms(void){return now;}
uint32_t BSP_Critical_Enter(void){return depth++;}
void BSP_Critical_Exit(uint32_t old){depth=old;}
uint8_t APP_Stabilizer_IsArmed(void){return armed;}
void APP_Stabilizer_GetArmStatus(APP_Stabilizer_ArmStatus *s){s->armed=armed;s->published=1;}
int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*a,uint32_t b,uint32_t c){(void)a;(void)b;(void)c;calibrations++;return 0;}
int HAL_ADC_Start(ADC_HandleTypeDef*a){(void)a;assert(!depth);rank=next_rank;next_rank=(next_rank+1)%2;starts++;return 0;}
int HAL_ADC_PollForConversion(ADC_HandleTypeDef*a,uint32_t timeout){(void)a;assert(timeout==1);return fail_rank==(int)rank?HAL_TIMEOUT:0;}
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef*a){(void)a;return 0;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*a){(void)a;if(preempt&&rank==0)now+=300;return raws[rank];}
int HAL_ADC_Stop(ADC_HandleTypeDef*a){(void)a;next_rank=stop_error?1:0;stops++;return stop_error?HAL_ERROR:0;}
void APP_Control_QueueText(const char*fmt,...){(void)fmt;assert(!depth);}
uint8_t APP_Diag_SendBinary(uint16_t function,const uint8_t *p,uint16_t n){
    assert(!depth&&function==0x2232&&n==60);assert(APP_Proto_BuildFrame('>',function,p,n,packet,sizeof(packet),&packet_size));return 1;
}
uint8_t APP_ELRS_SendTelemetryBattery(uint16_t voltage,uint16_t current,uint32_t used,uint8_t remaining){
    assert(!depth&&used==0xFFFFFF&&remaining==0xFF);sent++;sent_v=voltage;sent_i=current;return tx_ok;
}
static void sample(void){APP_Current_Step();APP_Battery_Step();}
int main(int argc,char **argv){
    assert(argc==2);APP_BatterySnapshot b;APP_CurrentSnapshot c;
    APP_Battery_GetSnapshot(&b);assert(!b.state.valid&&!b.can_arm&&b.state.config.cells==3);
    assert(!APP_Battery_Configure((DRV_BatteryConfig){3,3300,3500}));
    APP_Current_Init();APP_Battery_Init();sample();APP_Battery_GetSnapshot(&b);APP_Current_GetSnapshot(&c);
    assert(calibrations==1&&starts==2&&stops==1);
    assert(c.reading.raw==12000&&b.state.raw==11283&&b.state.voltage_mv>=11998&&b.state.voltage_mv<=12001);
    assert(b.state.valid&&b.can_arm&&b.state.samples==1);
    /* 2026-10-02 起默认 11.2/11.25 V（10-01 为 11.4/11.45）；归档黄金帧录于旧默认 3500/3600，帧前显式配回以核对线格式。 */
    assert(b.state.config.low_cell_mv==3733&&b.state.config.recover_cell_mv==3750);
    assert(APP_Battery_Configure((DRV_BatteryConfig){3,3500,3600}));
    APP_Battery_Step();APP_Battery_GetSnapshot(&b);assert(b.state.samples==1);
    char *query[]={"BATTERY?","42"};assert(APP_Battery_Command(query,2));
    FILE *f=fopen(argv[1],"wb");assert(f);fwrite(packet,1,packet_size,f);fclose(f);
    APP_Battery_TelemetryStep();assert(sent==1&&sent_v==120&&sent_i==474);
    now=499;APP_Battery_TelemetryStep();assert(sent==1);
    now=500;tx_ok=0;APP_Battery_TelemetryStep();assert(sent==2&&sent_v==65535&&sent_i==65535);
    uint32_t ok,bad;APP_Battery_GetTxStats(&ok,&bad);assert(ok==1&&bad==1);
    APP_Battery_GetSnapshot(&b);assert(!b.state.valid&&!b.can_arm&&b.age_ms==500);
    fail_rank=1;sample();APP_Current_GetSnapshot(&c);APP_Battery_GetSnapshot(&b);
    assert(!c.reading.valid&&!b.state.valid&&b.state.adc_status==2&&b.state.errors==1);
    assert(c.samples==1&&b.state.samples==1); /* No partial/current-only pair published. */
    fail_rank=-1;now+=20;sample();APP_Battery_GetSnapshot(&b);assert(b.state.valid&&b.can_arm&&b.state.samples==2);
    char *set[]={"BATTERY","SET","43","3","3300","3500"};assert(APP_Battery_Command(set,6));
    assert(packet[9]&32);APP_Battery_GetSnapshot(&b);assert(b.state.config.low_cell_mv==3300);
    armed=1;set[4]="3400";assert(APP_Battery_Command(set,6));assert(packet[9]&64);
    APP_Battery_GetSnapshot(&b);assert(b.state.config.low_cell_mv==3300);armed=0;
    set[3]="259";assert(APP_Battery_Command(set,6));assert(packet[9]&64);
    assert(!APP_Battery_Command(NULL,0));
    now+=20;preempt=1;sample();APP_Current_GetSnapshot(&c);APP_Battery_GetSnapshot(&b);
    assert(!c.reading.valid&&!b.state.valid&&!b.can_arm&&b.age_ms==300);preempt=0;
    now+=20;raws[1]=65535;sample();APP_Battery_GetSnapshot(&b);assert(!b.state.valid&&b.state.saturated&&!b.can_arm);
    now+=20;raws[1]=11283;stop_error=1;sample();APP_Battery_GetSnapshot(&b);assert(!b.can_arm);
    uint32_t before_starts=starts;stop_error=0;now+=20;sample();assert(starts==before_starts);
    APP_Battery_GetSnapshot(&b);assert(!b.state.valid&&!b.can_arm);
    hadc1.Init.DiscontinuousConvMode=0;assert(BSP_Current_Init()==BSP_CURRENT_ERROR);
    puts("battery ADC pairs/age/errors/RAM ACK/CRSF cadence: passed");return 0;
}
'''
    (d/'test.c').write_text(code)
    command=[shutil.which('gcc'),'-std=c11','-O2','-Wall','-Wextra','-Werror']
    for path in (d,ROOT/'App/Inc',ROOT/'Driver/Inc',ROOT/'BSP/Inc',ROOT/'Services/Inc'):command+=['-I',str(path)]
    command+=[str(d/'test.c')]+[str(ROOT/path) for path in ('App/Src/app_current.c','BSP/Src/bsp_current.c',
        'Driver/Src/drv_current.c','Driver/Src/drv_current_filter.c','Driver/Src/drv_battery.c','App/Src/app_battery.c','App/Src/app_battery_proto.c',
        'App/Src/app_battery_telemetry.c','App/Src/app_proto.c')]
    command+=['-lm','-o',str(d/'test.exe')]
    result=subprocess.run(command,capture_output=True,text=True);assert result.returncode==0,result.stdout+result.stderr
    result=subprocess.run([str(d/'test.exe'),str(d/'wire.bin')],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    return (d/'wire.bin').read_bytes()


def test_actual_c_battery_frame(firmware_battery):
    snapshot=decode_battery(firmware_battery[8:-1])
    assert snapshot.nonce==42 and snapshot.cells==3 and snapshot.can_arm and snapshot.valid
    assert snapshot.raw==11283 and abs(snapshot.voltage_mv-12000)<=2
    assert snapshot.arm_state==1 and snapshot.current_ma==47393
    assert firmware_battery[:8]==b'$X>\0\x32\x22\x3c\0'
    # 同上：把归档的 69B 黄金帧接进断言，让 60B payload 的字段顺序和 CRC 有一份
    # 已提交的线格式基准，而不是只靠本文件里的几条字段断言。
    golden=ROOT/'data/analysis/bluetooth-battery/2026-09-13/battery-diagnostic.golden.bin'
    assert firmware_battery==golden.read_bytes(),'电池诊断线格式与归档黄金帧不符'


def test_battery_decoder_rejects_truncation_and_inconsistent_flags(firmware_battery):
    payload=firmware_battery[8:-1]
    for count in range(len(payload)):
        with pytest.raises(ValueError):decode_battery(payload[:count])
    for flags in (0x80,1|2|4,4,32|64):
        with pytest.raises(ValueError):decode_battery(payload[:1]+bytes([flags])+payload[2:])


GATE_HARNESS=r'''
#include "app_battery.h"
#include "app_current.h"
#include "bsp_current.h"
#include <assert.h>
#include <stdio.h>
static uint32_t now=100;
uint32_t SVC_Timestamp_Ms(void){return now;}
uint32_t BSP_Critical_Enter(void){return 0;}
void BSP_Critical_Exit(uint32_t x){(void)x;}
uint8_t APP_Stabilizer_IsArmed(void){return 0;}
void BSP_Current_GetVoltageSample(BSP_VoltageSample *o){o->raw=0;o->sequence=0;o->status=BSP_CURRENT_NOT_READY;}
void APP_Current_GetSnapshot(APP_CurrentSnapshot *o){(void)o;}
int main(void){
    APP_BatterySnapshot b;
    APP_Battery_GetSnapshot(&b);
    assert(!b.can_arm && !b.state.valid);
    assert(!APP_Battery_CanArm());
    puts("pre-init arm refusal survives a healthy-looking static initialiser");
    return 0;
}
'''


def test_pre_init_arm_refusal_is_a_gate_not_an_accident(tmp_path):
    """未初始化时拒绝解锁必须由 initialized 判据兜住，不能只靠静态初值恰好是"坏"的。

    稳定环优先级 24、messageTask 16，而 APP_Battery_Init() 排在 messageTask 里
    APP_Current_Init() 的阻塞式 ADC 校准之后——所以第一次 APP_Battery_CanArm()
    必然早于初始化。这里把 app_battery.c 的静态初始化式改成"看起来健康"的样子
    （低压位清零、已有有效样本），sample_ms 仍是 0，在 now=100ms 时会通过 250ms
    新鲜度判定。若没有 initialized 兜底，这一改就等于开机瞬间允许解锁。
    """
    source=(ROOT/'App/Src/app_battery.c').read_text(encoding='utf-8')
    original='.low=1U,.adc_status=1U};'
    assert original in source,'静态初始化式已改写，请同步本测试'
    mutated=source.replace(original,'.low=0U,.adc_status=0U,.valid=1U,.samples=1U};',1)
    (tmp_path/'app_battery_mutated.c').write_text(mutated,encoding='utf-8')
    (tmp_path/'gate.c').write_text(GATE_HARNESS,encoding='utf-8')
    (tmp_path/'app_stabilizer.h').write_text('#include <stdint.h>\nuint8_t APP_Stabilizer_IsArmed(void);\n')
    command=[shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror']
    for path in (tmp_path,ROOT/'App/Inc',ROOT/'Driver/Inc',ROOT/'BSP/Inc',ROOT/'Services/Inc'):command+=['-I',str(path)]
    command+=[str(tmp_path/'gate.c'),str(tmp_path/'app_battery_mutated.c'),str(ROOT/'Driver/Src/drv_battery.c'),
              '-o',str(tmp_path/'gate.exe')]
    result=subprocess.run(command,capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    result=subprocess.run([str(tmp_path/'gate.exe')],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
