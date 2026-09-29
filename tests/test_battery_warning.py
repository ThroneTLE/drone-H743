"""Actual LED policy: the eight-pulse battery alarm explains a *blocked* arm only.

已解锁时红灯是地面上唯一能判断"电机带电"的信号。3S 带载掉到 10.5V 以下是常态，
若低压告警能盖掉红灯，飞行中/地面试车时这盏灯大半时间是灭的，旁人会当成已上锁。
所以这里同时钉死两件事：解锁后红灯压过低压告警；未解锁时低压告警必须出现。
"""
from pathlib import Path
import subprocess
import shutil
from test_battery_crsf import c_function

ROOT=Path(__file__).resolve().parents[1]

HARNESS=r'''
#include "app_led.h"
#include "app_battery.h"
#include "svc_led.h"
#include <assert.h>
static uint8_t app_led_armed,app_led_arm_published=1;
static APP_LED_ArmBlockReason app_led_arm_block_reason=APP_LED_ARM_BLOCK_BATTERY;
const DRV_RgbColor app_led_red={255,0,0},app_led_green={0,255,0},app_led_amber={255,110,0};
static unsigned armed_cleared,armed_shown,status_cleared,blocked_cleared,alarm,identify_cleared;
DRV_RgbPattern app_led_solid(DRV_RgbColor c){(void)c;assert(0);return (DRV_RgbPattern){0};}
DRV_RgbPattern app_led_breathe(DRV_RgbColor c,uint16_t p,uint8_t d){(void)c;(void)p;(void)d;assert(0);return (DRV_RgbPattern){0};}
DRV_RgbPattern app_led_pulses(DRV_RgbColor c,uint8_t n){(void)c;(void)n;assert(0);return (DRV_RgbPattern){0};}
void app_led_publish(SVC_LedSource source,uint8_t binding){
    assert(source==SVC_LED_SOURCE_ARMED&&binding==APP_LED_BIND_ARMED);armed_shown++;}
uint8_t APP_LedConfig_BindingForBlockReason(uint8_t reason){(void)reason;assert(0);return 0;}
void SVC_Led_Publish(SVC_LedSource source,const DRV_RgbPattern *pattern){
    if(source==SVC_LED_SOURCE_ARMED){assert(!pattern);armed_cleared++;}
    else if(source==SVC_LED_SOURCE_STATUS){assert(!pattern);status_cleared++;}
    else if(source==SVC_LED_SOURCE_IDENTIFY){assert(!pattern);identify_cleared++;}
    else{assert(source==SVC_LED_SOURCE_BLOCKED);
        if(!pattern){blocked_cleared++;return;}
        assert(pattern->count==8&&pattern->effect==DRV_RGB_EFFECT_PULSES);alarm++;}
}
'''

MAIN=r'''
int main(void){
    /* 已解锁 + 低压：红灯必须照常点亮，告警不得接管这盏灯；
     * 同时必须撤销上位机点名——IDENTIFY 在 svc_led 里优先级高于 ARMED。 */
    app_led_armed=1;app_led_publish_arm();
    assert(app_led_armed==1);
    assert(armed_shown==1&&armed_cleared==0&&alarm==0);
    assert(blocked_cleared==1&&status_cleared==1&&identify_cleared==1);

    /* 未解锁 + 低压：8 闪告警必须出现，并清掉解锁/就绪两路；
     * 未解锁时点名是正当用途，不能去动它。 */
    armed_shown=armed_cleared=status_cleared=blocked_cleared=alarm=identify_cleared=0;
    app_led_armed=0;app_led_publish_arm();
    assert(alarm==1&&armed_shown==0&&armed_cleared>=1&&status_cleared>=1);
    assert(identify_cleared==0);
    return 0;
}
'''


def test_armed_solid_outranks_battery_alarm_which_still_shows_when_disarmed(tmp_path):
    source=(ROOT/'App/Src/app_led.c').read_text(encoding='utf-8')
    code=HARNESS+c_function(source,'static void app_led_publish_arm(')+MAIN
    if (ROOT/'App/Inc/app_led_config.h').exists():
        code='#include "app_led_config.h"\n'+code
    (tmp_path/'test.c').write_text(code,encoding='utf-8')
    cmd=[shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror']
    for directory in ('App/Inc','Driver/Inc','Services/Inc'):cmd+=['-I',str(ROOT/directory)]
    cmd+=[str(tmp_path/'test.c'),str(ROOT/'App/Src/app_battery_led.c'),'-o',str(tmp_path/'test.exe')]
    result=subprocess.run(cmd,capture_output=True,text=True);assert result.returncode==0,result.stdout+result.stderr
    subprocess.run([str(tmp_path/'test.exe')],check=True)
