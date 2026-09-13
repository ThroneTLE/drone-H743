"""Actual LED policy keeps an eight-pulse battery alarm visible while armed."""
from pathlib import Path
import subprocess
import shutil
from test_battery_crsf import c_function

ROOT=Path(__file__).resolve().parents[1]


def test_battery_warning_overrides_armed_solid_without_changing_arm_state(tmp_path):
    source=(ROOT/'App/Src/app_led.c').read_text(encoding='utf-8')
    actual=c_function(source,'static void app_led_publish_arm(')
    code=r'''
#include "app_led.h"
#include "app_battery.h"
#include "svc_led.h"
#include <assert.h>
static uint8_t app_led_armed=1,app_led_arm_published=1;
static APP_LED_ArmBlockReason app_led_arm_block_reason=APP_LED_ARM_BLOCK_BATTERY;
static const DRV_RgbColor app_led_red={255,0,0},app_led_green={0,255,0},app_led_amber={255,110,0};
static unsigned armed_cleared,status_cleared,alarm;
static DRV_RgbPattern app_led_solid(DRV_RgbColor c){(void)c;assert(0);return (DRV_RgbPattern){0};}
static DRV_RgbPattern app_led_breathe(DRV_RgbColor c,uint16_t p,uint8_t d){(void)c;(void)p;(void)d;assert(0);return (DRV_RgbPattern){0};}
static DRV_RgbPattern app_led_pulses(DRV_RgbColor c,uint8_t n){(void)c;(void)n;assert(0);return (DRV_RgbPattern){0};}
void SVC_Led_Publish(SVC_LedSource source,const DRV_RgbPattern *pattern){
    if(source==SVC_LED_SOURCE_ARMED){assert(!pattern);armed_cleared++;}
    else if(source==SVC_LED_SOURCE_STATUS){assert(!pattern);status_cleared++;}
    else {assert(source==SVC_LED_SOURCE_BLOCKED&&pattern&&pattern->count==8&&pattern->effect==DRV_RGB_EFFECT_PULSES);alarm++;}
}
''' + actual + r'''
int main(void){app_led_publish_arm();assert(app_led_armed==1&&armed_cleared==1&&status_cleared==1&&alarm==1);return 0;}
'''
    (tmp_path/'test.c').write_text(code,encoding='utf-8')
    cmd=[shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror']
    for directory in ('App/Inc','Driver/Inc','Services/Inc'):cmd+=['-I',str(ROOT/directory)]
    cmd+=[str(tmp_path/'test.c'),str(ROOT/'App/Src/app_battery_led.c'),'-o',str(tmp_path/'test.exe')]
    result=subprocess.run(cmd,capture_output=True,text=True);assert result.returncode==0,result.stdout+result.stderr
    subprocess.run([str(tmp_path/'test.exe')],check=True)
