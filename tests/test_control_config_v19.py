"""CFG V19 cascade migration contract; synthetic records are unit tests only."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r"""
#include "app_control_config_compat.h"
#include <math.h>
#include <string.h>

#define CHECK(x, n) do { if (!(x)) return (n); } while (0)
#define NEAR(a,b) (fabsf((a)-(b)) < 1.0e-5f)

int main(void) {
    APP_ControlCoaxTunableParamsV18 old = {0};
    APP_ControlCoaxTunableParams now;
    old.pos_x_kp = 0.30f; old.pos_y_kp = 0.30f; old.pos_z_kp = 3.8f;
    old.pos_z_ki = 0.25f;
    old.vel_x_kd = 0.80f; old.vel_y_kd = 0.80f; old.vel_z_kd = 0.0f;
    old.vel_loop_enable = 1.0f;
    old.roll_angle_kp = 0.0671f; old.pitch_angle_kp = 0.0660f;
    old.roll_rate_kd = 0.1104f; old.pitch_rate_kd = 0.1138f;
    old.tilt_limit_rad = 0.4886922f;
    old.yaw_angle_kp = 1.0f; old.yaw_rate_kd = 0.15f;
    CHECK(APP_ControlConfigCompat_V18ToCurrent(&old, &now), 1);
    CHECK(NEAR(now.pos_x_kp, 0.375f) && NEAR(now.vel_x_kp, 0.8f), 2);
    CHECK(NEAR(now.pos_x_kp * now.vel_x_kp, old.pos_x_kp), 3);
    CHECK(NEAR(now.pos_y_kp * now.vel_y_kp, old.pos_y_kp), 4);
    CHECK(NEAR(now.vel_z_kp, 1.0f) && NEAR(now.pos_z_kp, 3.8f), 5);
    CHECK(now.vel_x_ki == 0.0f && now.vel_y_ki == 0.0f &&
          now.vel_z_ki == 0.0f && now.vel_x_kd == 0.0f &&
          now.vel_y_kd == 0.0f && now.vel_z_kd == 0.0f, 6);
    CHECK(NEAR(now.rate_roll_kp * now.att_roll_kp, old.roll_angle_kp), 7);
    CHECK(NEAR(now.rate_pitch_kp * now.att_pitch_kp, old.pitch_angle_kp), 8);
    /* The divisor is FROZEN at the I_zz this record format was written with.
     * It used to track the live constant; since 2026-09-11 the airframe model
     * is runtime data from Flash, and letting a migration follow it would mean
     * the same stored bytes decode to a different gain after the aircraft is
     * re-measured -- a silent handling change with no message. The Python side
     * cross-checks this literal against the constant in the compat source. */
    CHECK(NEAR((now.rate_yaw_kp * now.att_yaw_kp) / 0.005f,
               old.yaw_angle_kp), 9);

    APP_ControlCoaxTunableParamsV17 v17 = {0};
    memcpy(&v17, &old, 8U * sizeof(float));
    v17.roll_angle_kp = old.roll_angle_kp; v17.pitch_angle_kp = old.pitch_angle_kp;
    v17.roll_rate_kd = old.roll_rate_kd; v17.pitch_rate_kd = old.pitch_rate_kd;
    v17.tilt_limit_rad = old.tilt_limit_rad;
    v17.yaw_angle_kp = old.yaw_angle_kp; v17.yaw_rate_kd = old.yaw_rate_kd;
    CHECK(APP_ControlConfigCompat_V17ToCurrent(&v17, &now), 10);

    APP_ControlCoaxTunableParamsV15 v15 = {0};
    v15.pos_x_kp = old.pos_x_kp; v15.pos_y_kp = old.pos_y_kp;
    v15.pos_z_kp = old.pos_z_kp; v15.vel_x_kd = old.vel_x_kd;
    v15.vel_y_kd = old.vel_y_kd; v15.roll_angle_kp = old.roll_angle_kp;
    v15.pitch_angle_kp = old.pitch_angle_kp; v15.roll_rate_kd = old.roll_rate_kd;
    v15.pitch_rate_kd = old.pitch_rate_kd; v15.tilt_limit_rad = old.tilt_limit_rad;
    v15.yaw_angle_kp = old.yaw_angle_kp; v15.yaw_rate_kd = old.yaw_rate_kd;
    CHECK(APP_ControlConfigCompat_V15ToCurrent(&v15, &now), 11);
    return 0;
}
"""


def test_v18_v17_v15_migration_math_on_host(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    if gcc is None:
        pytest.skip("gcc is required")
    harness = tmp_path / "cfg_v19.c"
    exe = tmp_path / "cfg_v19.exe"
    harness.write_text(HARNESS, encoding="ascii")
    result = subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{ROOT / 'App' / 'Inc'}",
            f"-I{ROOT / 'Driver' / 'Inc'}",
            str(ROOT / "App" / "Src" / "app_control_config_compat.c"),
            str(harness),
            "-lm",
            "-o",
            str(exe),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, check=False)
    assert run.returncode == 0, run.stdout + run.stderr


def test_cfg_v19_store_is_extracted_and_backward_compatible() -> None:
    store = (ROOT / "App" / "Src" / "app_control_config_store.c").read_text(
        encoding="utf-8"
    )
    header = (ROOT / "App" / "Inc" / "app_control_config_store.h").read_text(
        encoding="utf-8"
    )
    control = (ROOT / "App" / "Src" / "app_control.c").read_text(encoding="utf-8")
    # v19 已不是当前版本（v20 追加了机体模型块），但它必须仍在迁移链上。
    # v21 起记录里多了状态灯颜色绑定块（APP_LedConfig）；v20 的读取器
    # 仍在（config_read_v20），所以旧记录照样读得回来。
    # 2026-09-20（R-MAG-1）：v23 在记录尾部追加磁力计校准块，当前版本号随之
    # 推进到 23；v19 仍在迁移链上不受影响，v22 → v23 的覆盖见下一条测试。
    # 2026-09-28：v24 在记录尾部追加横滚/俯仰指令整形与出口陷波块（默认关），
    # v23 → v24 由 config_read_v23 读取、新块落回"关"（tests/test_attitude_shaping.py 实跑覆盖）。
    # 同日晚 v25 在整形块尾部追加第二级出口陷波，v24 由 config_read_v24 读取、第二级落回"关"。
    assert "APP_CONTROL_CFG_VERSION     25U" in header
    assert "APP_CONTROL_CFG_VERSION_V24 24U" in header
    assert "APP_CONTROL_CFG_VERSION_V23 23U" in header
    assert "config_read_v24" in store
    assert "APP_CONTROL_CFG_VERSION_V19 19U" in header
    for version in ("V18", "V17", "V16", "V15"):
        assert f"APP_CONTROL_CFG_VERSION_{version}" in header
        assert f"config_read_{version.lower()}" in store
    assert "config_checksum" in store
    assert "APP_ControlConfigStore_Load(&control_config)" in control
    assert "APP_ControlConfigStore_Save(&control_config)" in control
    assert "APP_ControlFlashRecordV17" not in control
    assert "pos_z_ki" not in store


def test_cfg_v22_migrates_into_v23_keeping_old_blocks_and_defaulting_the_new_mag_block() -> None:
    """v22 → v23 迁移覆盖，钉法与上面 v19 一节同构：旧记录只读不丢。

    2026-09-20（R-MAG-1）：v23 在记录尾部追加磁力计校准块。config_read_v22
    读一条 v22 记录时，必须把它已有的四块（遥控映射/机体模型/LED/桨叶标定）
    按记录里的真实字段原样应用——不能因为加了新块就退化成像它们自己都没有
    过的版本那样落回默认。新出现的磁力计块在 v22 记录里不存在，必须显式落回
    "未校准"（NULL），不能留着 RAM 里上一次的系数。
    """
    store = (ROOT / "App" / "Src" / "app_control_config_store.c").read_text(
        encoding="utf-8"
    )
    reader = store[store.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v22"):]
    reader = reader[:reader.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v21")]

    assert "app_cmd_rcmap_apply_config(&record.rc_config);" in reader
    assert "DRV_Airframe_SetParams(&record.airframe);" in reader
    assert "app_cmd_ledmap_apply_config(&record.led);" in reader
    assert "app_cmd_propcal_apply_config(&record.prop);" in reader
    assert "app_cmd_magcal_apply_config(NULL);" in reader


def test_v18_yaw_migration_divisor_is_frozen_not_the_live_airframe_model() -> None:
    """迁移必须是确定的函数：同样的字节进去，同样的数出来。

    V18 记录里的 yaw_rate_kd 单位是 1/s，当前记录是 N·m/(rad/s)，两者差一个
    I_zz。历史上这里直接引用 I_zz 常量——那时它是编译期常量，"跟着它走"没问题。
    2026-09-11 机体模型改成 Flash 运行时数据之后，再跟着走就意味着：重新量过
    惯量以后，同一条旧记录会被解成另一个增益，而用户只看到"载入旧配置后手感
    变了"，没有任何提示。所以这个除数被冻结成字面常量。

    本条测试同时钉两件事：冻结值必须与 harness 里的 0.005f 一致；迁移模块
    不许再去读运行时机体模型。
    """
    compat = (ROOT / "App" / "Src" / "app_control_config_compat.c").read_text(
        encoding="utf-8"
    )

    assert "#define APP_CONTROL_COMPAT_V18_YAW_INERTIA_KGM2    0.005f" in compat
    assert (
        "current->rate_yaw_kp = APP_CONTROL_COMPAT_V18_YAW_INERTIA_KGM2 *" in compat
    )
    assert "DRV_Airframe_" not in compat, "迁移不能跟着运行时机体模型走"
    assert "drv_airframe_params.h" not in compat
