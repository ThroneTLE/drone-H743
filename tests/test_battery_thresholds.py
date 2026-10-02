"""电池门限（2026-10-01 作者："按照之前高度辨识的经验，静止时11.4V一下电池电压代表几乎推不动飞机了……
你可以把这个电压11.6V视为没电警告可能会影响结果11.4V设置为完全没电不让解锁"）。

解锁门：固件默认每节 3733 mV（3S 11.2 V）判低、3750 mV（11.25 V）恢复（2026-10-02 按实机飞行电压下调，原 11.4/11.45 V）。11.4 V 告警只在辨识页提示
（THR 行 vbat_mv，上锁时判，带载电压本来就会掉 1 V 以上）。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

HARNESS = r"""
#include "drv_battery.h"
#include <stdio.h>
static DRV_BatteryState s;
static void v(uint32_t mv, uint32_t now){DRV_Battery_Update(&s,(mv*65535U+34848U)/69696U,0,now);}
int main(void){
    DRV_Battery_Init(&s);
    v(11190,10); printf("%u ",DRV_Battery_CanArm(&s,10));   /* 低于 11.2：不能解锁 */
    v(11220,20); printf("%u ",DRV_Battery_CanArm(&s,20));   /* 回差内：仍不能 */
    v(11260,30); printf("%u ",DRV_Battery_CanArm(&s,30));   /* ≥11.25：恢复 */
    v(11210,40); printf("%u ",DRV_Battery_CanArm(&s,40));   /* 恢复后 ≥11.2：可以 */
    v(11180,50); printf("%u\n",DRV_Battery_CanArm(&s,50));  /* 再低于 11.2：不能 */
    return 0;
}
"""


def test_firmware_defaults_block_arming_below_11_2_v(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("host gcc unavailable")
    src = tmp_path / "bat.c"
    exe = tmp_path / "bat.exe"
    src.write_text(HARNESS, encoding="utf-8")
    built = subprocess.run([gcc, "-std=c11", "-Wall", "-Werror", f"-I{ROOT / 'Driver' / 'Inc'}",
                            str(ROOT / "Driver/Src/drv_battery.c"), str(src), "-o", str(exe)],
                           capture_output=True, text=True)
    assert built.returncode == 0, built.stderr
    out = subprocess.run([str(exe)], capture_output=True, text=True).stdout.split()
    assert out == ["0", "0", "1", "1", "0"]


def test_both_default_sites_use_the_shared_thresholds() -> None:
    header = (ROOT / "Driver/Inc/drv_battery.h").read_text(encoding="utf-8")
    assert "#define DRV_BATTERY_DEFAULT_LOW_CELL_MV 3733U" in header
    assert "#define DRV_BATTERY_DEFAULT_RECOVER_CELL_MV 3750U" in header
    for rel in ("Driver/Src/drv_battery.c", "App/Src/app_battery.c"):
        text = (ROOT / rel).read_text(encoding="utf-8")
        assert "DRV_BATTERY_DEFAULT_LOW_CELL_MV" in text and "3500U" not in text, rel


def test_sysid_banner_warns_below_11_4_v_only_while_disarmed() -> None:
    import sys
    sys.path.insert(0, str(ROOT / "tools"))
    from panel_lib.pages.sysid import banner

    rest = banner.rc_summary({"armed": "0", "thr_low": "1", "vbat_mv": "11320"})
    assert "电池 11.32 V" in rest and "低于 11.4 V" in rest and "11.2 V 以下不能解锁" in rest
    assert "低于" not in banner.rc_summary({"armed": "0", "thr_low": "1", "vbat_mv": "12300"})
    loaded = banner.rc_summary({"armed": "1", "thr_low": "1", "vbat_mv": "10400"})
    assert "电池 10.40 V" in loaded and "低于" not in loaded
    assert "电池" not in banner.rc_summary({"armed": "0", "thr_low": "1"})     # 旧固件不报
