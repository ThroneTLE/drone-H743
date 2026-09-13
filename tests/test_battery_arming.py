"""Execute the actual arming state machine with the real battery voltage policy."""
from pathlib import Path
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
    edge();voltage(10400);assert(!APP_Battery_CanArm());
    assert(!stabilizer_rc_update_armed(1,1,1));
    voltage(10700);assert(!APP_Battery_CanArm());
    voltage(11000);assert(APP_Battery_CanArm());assert(!stabilizer_rc_update_armed(1,1,1));
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
