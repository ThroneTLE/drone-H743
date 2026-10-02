"""Execute the actual arming state machine with the real battery voltage policy."""
from pathlib import Path
import re
import subprocess
import shutil

ROOT=Path(__file__).resolve().parents[1]


def test_real_arming_edges_low_voltage_stale_and_existing_gates(tmp_path):
    source=(ROOT/'App/Src/app_stabilizer.c').read_text(encoding='utf-8')
    start=source.index('  static uint8_t stabilizer_rc_update_armed(')
    end=source.index('\n  }',start)+4
    actual=source[start:end]
    code=r'''
#include "drv_battery.h"
#include <assert.h>
#include <stdio.h>
static uint8_t stabilizer_rc_arm_latched,stabilizer_rc_switch_seen_low,stabilizer_rc_switch_prev_high;
static uint8_t frame_locked,imu_blocked,airframe_valid=1;
static uint32_t now;
static DRV_BatteryState battery;
uint8_t APP_Stabilizer_IsImuFrameArmLocked(void){return frame_locked;}
uint8_t APP_ImuHealth_IsArmBlocked(void){return imu_blocked;}
uint8_t DRV_Airframe_IsValid(void){return airframe_valid;}
uint8_t APP_Battery_CanArm(void){return DRV_Battery_CanArm(&battery,now);}
static void voltage(uint32_t mv){DRV_Battery_Update(&battery,(mv*65535U+34848U)/69696U,0,now);}
''' + actual + r'''
static void edge(void){assert(!stabilizer_rc_update_armed(0,1,1));}
int main(void){
    DRV_Battery_Init(&battery);edge();
    assert(!stabilizer_rc_update_armed(1,1,1)); /* Missing voltage refuses arming. */
    voltage(12000);assert(!stabilizer_rc_update_armed(1,1,1)); /* No auto-arm on recovery. */
    edge();assert(stabilizer_rc_update_armed(1,1,1));
    voltage(10000);assert(!APP_Battery_CanArm());
    assert(stabilizer_rc_update_armed(1,1,1)); /* Low in flight does not cut motors. */
    now+=251;assert(stabilizer_rc_update_armed(1,1,1)); /* Nor does stale voltage. */
    assert(!stabilizer_rc_update_armed(1,1,0)); /* RC loss STILL disarms. */
    voltage(12000);assert(!stabilizer_rc_update_armed(1,1,1));edge();
    assert(!stabilizer_rc_update_armed(1,0,1)); /* High throttle still refuses. */
    assert(!stabilizer_rc_update_armed(1,1,1));edge();assert(stabilizer_rc_update_armed(1,1,1));
    for(unsigned gate=0;gate<3;gate++){
        frame_locked=gate==0;imu_blocked=gate==1;airframe_valid=gate!=2;
        assert(!stabilizer_rc_update_armed(1,1,1));
        frame_locked=imu_blocked=0;airframe_valid=1;
        assert(!stabilizer_rc_update_armed(1,1,1));edge();assert(stabilizer_rc_update_armed(1,1,1));
    }
    /* 2026-10-02 默认门限：11.2 V 判低、11.25 V 恢复（DRV_BATTERY_DEFAULT_*，10-01 为 11.4/11.45）。 */
    edge();voltage(11100);assert(!APP_Battery_CanArm());
    assert(!stabilizer_rc_update_armed(1,1,1));
    voltage(11220);assert(!APP_Battery_CanArm());
    voltage(11300);assert(APP_Battery_CanArm());assert(!stabilizer_rc_update_armed(1,1,1));
    edge();assert(stabilizer_rc_update_armed(1,1,1));edge();now+=251;
    assert(!stabilizer_rc_update_armed(1,1,1));
    puts("actual arming: voltage missing/low/stale refuse; fresh switch edge required; flight and legacy gates preserved");
    return 0;
}
'''
    (tmp_path/'test.c').write_text(code,encoding='utf-8')
    command=[shutil.which('gcc'),'-std=c11','-Wall','-Wextra','-Werror','-I',str(ROOT/'Driver/Inc'),
             str(tmp_path/'test.c'),str(ROOT/'Driver/Src/drv_battery.c'),'-o',str(tmp_path/'test.exe')]
    compiled=subprocess.run(command,capture_output=True,text=True)
    assert compiled.returncode==0,compiled.stdout+compiled.stderr
    ran=subprocess.run([str(tmp_path/'test.exe')],capture_output=True,text=True)
    assert ran.returncode==0,ran.stdout+ran.stderr


def test_the_rotor_calibration_is_not_an_arming_gate():
    """桨叶接线标定**不**参与解锁判定。

    作者 2026-09-13 明确要求这一条：什么时候需要标定由他决定，固件不替他拦。
    这条断言的作用是防止它被"顺手"加回来——加回来不会让任何测试变红，
    而现场的症状是一块升级后的板子突然解不了锁，且闪灯报的是别的原因。

    未标定时的真实行为写在 drv_prop_map.h：偏航极性为 0（没有方向），
    分配式给不出偏航力矩；那是"没有偏航"，不是"偏航可能反"。
    """
    source=(ROOT/'App/Src/app_stabilizer.c').read_text(encoding='utf-8')
    start=source.index('  static uint8_t stabilizer_rc_update_armed(')
    end=source.index('\n  }',start)+4
    body=re.sub(r'/\*.*?\*/',' ',source[start:end],flags=re.S)
    assert 'DRV_PropMap' not in body, '解锁函数里不该出现桨叶标定判定'

    chain_start=source.index('if (frame->rc_link_seen == 0U) {')
    chain=re.sub(r'/\*.*?\*/',' ',
                 source[chain_start:source.index('APP_LED_SetArmStatus(',chain_start)],
                 flags=re.S)
    assert 'DRV_PropMap' not in chain, 'LED 原因链里也不该有它'
    assert 'APP_LED_ARM_BLOCK_PROPCAL' not in (ROOT/'App/Inc/app_led.h').read_text(encoding='utf-8')
