"""`SYSID PARAM` 的试用增益不许被别处触发的 Flash 保存带走 —— 在宿主上跑真源码。

2026-09-27 实机：杆上"临时应用 50%"之后，为改积分限幅发了一条普通
`PARAM SET coax.rate_pitch_i_limit_n_m 0.05`，1.5 s 后的自动保存把 RAM 里的 50% 试用增益
一并永久化了。`SYSID PARAM` 自己不排保存挡不住这件事：保存存的是**整份**配置。

契约见 App/Inc/app_param_trial.h。这里分两套装置验：

* **保存语义**（C 装置，build_and_run）：真的 app_param_trial.c、app_cmd_airframe.c
  （`PARAM SET` 两种写法共用的写入口 app_control_param_set_any）、
  app_control_config_store.c（Save/Load）与真驱动；Flash 用 test_config_store_ab_slots 的
  RAM 假 Flash。"存进去的是什么"靠 Load 读回——RAM 里的增益被整份换成 Flash 里的值。
* **命令面**（sysid 宿主装置，ctypes）：真的 app_cmd_sysid.c 解析 `SYSID PARAM`，
  回显、查询行（`SYSID TRIAL`）与"满了就拒绝"的 ERR。

反向验证（2026-09-28）：注释掉 app_control_config_store.c 里的
`APP_ParamTrial_RestorePersistent(...)`，C 装置里 2026-09-27 那一幕（15）、
重复试用（55）、全部名字往返（64，每个名字一条）、读取失败后再存（87/88）与满表（108/109）变红；
注释掉 app_cmd_airframe.c 里的 `APP_ParamTrial_Clear(name);`，同名 PARAM SET 即定下来
（21/22/24/25）与满表腾格（106/107）变红。
"""

from __future__ import annotations

import ctypes
import re
from pathlib import Path

import pytest

from _micoair_hostfakes import CHECK_MACRO, ROOT, build_and_run, write_fakes
from test_config_store_ab_slots import (
    FAKE_CMSIS_OS2_H,
    FAKE_FLASH_C,
    FAKE_MAIN_H,
    HOST_STUBS_C,
    SOURCES as CONFIG_STORE_SOURCES,
    includes as config_store_includes,
)
from test_sysid_runtime_contract import build_lib, texts


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_c_comments(source: str) -> str:
    source = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    return re.sub(r"//[^\n]*", "", source)


# ------------------------------------------------------------------ 名单与接线


# 高度环 z 通道（R-ALTID-1）：光杆台架高度辨识用 SYSID PARAM 只写 RAM 试用。
Z_LOOP_NAMES = {"pos_z_kp", "vel_z_kp", "vel_z_ki", "vel_z_kd", "vel_z_i_limit_m_s2"}
# 水平 x 通道（R-XYID-1）：水平槽 XY 辨识只跑 x 通道，同样只写 RAM 试用；y 通道不开放。
X_LOOP_NAMES = {"pos_x_kp", "vel_x_kp", "vel_x_ki", "vel_x_kd", "vel_x_i_limit_m_s2"}
LOOP_NAMES = Z_LOOP_NAMES | X_LOOP_NAMES


def test_the_trial_list_is_exactly_the_driver_rate_and_att_names() -> None:
    """可试用名单 = 驱动具名表里全部 coax.rate_* / coax.att_* + 高度环 z 通道五项 + 水平 x 通道五项（契约 7：每个都放得下）。

    名单外的名字会被拒绝试用，名单里多出驱动没有的名字则永远用不上——两边从源码里抓，
    不手抄。容量默认就是名单长度。z 通道五项必须既在驱动具名表里、也是 Flash 增益块
    APP_ControlCoaxTunableParams 的同名字段（PARAM_TRIAL_GAIN 用 offsetof 取，写错编译不过）。
    """
    driver_source = read("Driver/Src/drv_coax_ctrl.c")
    driver = set(re.findall(r'DRV_COAX_CTRL_NAMED_PARAM_ENTRY\("((?:rate|att)_\w+)"',
                            driver_source))
    driver |= set(re.findall(r"DRV_COAX_CTRL_PARAM_ENTRY\(((?:rate|att)_\w+)\)",
                             driver_source))
    named = set(re.findall(r'DRV_COAX_CTRL_NAMED_PARAM_ENTRY\("(\w+)"', driver_source))
    tunables = read("App/Inc/app_control_config_compat.h")
    tunables = tunables[:tunables.index("} APP_ControlCoaxTunableParams;")]
    for name in LOOP_NAMES:
        assert name in named, name
        assert f"    float {name};" in tunables, name
    driver |= LOOP_NAMES
    trial_source = read("App/Src/app_param_trial.c")
    trial = re.findall(r"^\s*PARAM_TRIAL_(?:GAIN|SHAPING)\((\w+)\),", trial_source, flags=re.M)

    assert driver, "没从驱动具名表里抓到 rate_/att_ 名字，正则过期了"
    assert len(trial) == len(set(trial))
    assert set(trial) == driver
    assert "#define APP_PARAM_TRIAL_CAPACITY PARAM_TRIAL_FIELD_COUNT" in trial_source


def test_every_save_load_and_explicit_write_path_is_wired() -> None:
    """保存/读取/显式写三类路径都接到试用记录上（运行时行为见下面的 C 装置）。"""
    store = strip_c_comments(read("App/Src/app_control_config_store.c"))
    save = store[store.index("APP_FlashService_Status APP_ControlConfigStore_Save("):]
    # 两块（增益、v24 指令整形/出口陷波）都从 RAM 捕获完，才换回持久值，然后才算校验和。
    capture = save.index("APP_ControlConfigStore_CaptureTunables(&record.coax_tunables);")
    shaping = save.index("config_capture_shaping(&record.shaping);")
    restore = save.index("APP_ParamTrial_RestorePersistent(&record.coax_tunables, &record.shaping);")
    assert capture < restore and shaping < restore < save.index("record.checksum = config_checksum(")
    load = store[store.index("APP_FlashService_Status APP_ControlConfigStore_Load("):]
    load = load[:load.index("APP_FlashService_Status APP_ControlConfigStore_Save(")]
    assert load.index("APP_ParamTrial_ClearAll();") > load.index("switch (header.version)")

    # 飞行日志记的是控制器正在用的增益（RAM），不许换成持久值。
    assert "APP_ParamTrial" not in strip_c_comments(read("App/Src/app_flight_log.c"))

    route = strip_c_comments(read("App/Src/app_cmd_airframe.c"))
    set_any = route[route.index("uint8_t app_control_param_set_any("):]
    coax_branch = set_any[:set_any.index("return DRV_Airframe_SetParam(")]
    assert "APP_ParamTrial_Clear(name);" in coax_branch

    control = strip_c_comments(read("App/Src/app_control.c"))
    defaults = control[control.index("static void app_control_defaults(APP_ControlConfig *config)"):]
    defaults = defaults[:defaults.index("\n}\n")]
    assert defaults.index("APP_ParamTrial_ClearAll();") > defaults.index("DRV_COAX_CTRL_ResetParams();")


def test_the_trial_report_uses_integer_formatting_only() -> None:
    """newlib-nano 没有浮点 printf。"""
    code = strip_c_comments(read("App/Src/app_param_trial.c"))
    assert not re.search(r"%[-+ #0-9.]*[efgEFG]", code)


# ------------------------------------------------------------------ 保存语义（C 装置）


TRIAL_HARNESS = (
    CHECK_MACRO
    + FAKE_FLASH_C
    + HOST_STUBS_C
    + r"""
#include "app_param_trial.h"

#include <math.h>

/* app_cmd_airframe.c 的 AIRFRAME 报文出口；本装置不看它。 */
void app_control_queue_proto_text(uint16_t function, const char *format, ...)
{
    (void)function;
    (void)format;
}

extern void fake_flash_reset(void);

static float ram(const char *name)
{
    float value = -1.0f;

    CHECK(DRV_COAX_CTRL_GetParam(name, &value) == 1U, 900);
    return value;
}

/* `PARAM SET n v` 与 `n=v` 两种写法共用的写入口；app_control.c 随后排自动保存。 */
static void param_set(const char *name, float value)
{
    CHECK(app_control_param_set_any(name, value) == 1U, 901);
}

/* `SYSID PARAM n v` 解析完之后调的就是它。 */
static APP_ParamTrialStatus trial(const char *name, float value)
{
    float applied = -1.0f;

    return APP_ParamTrial_Apply(name, value, &applied);
}

/* 自动保存、SAVE、RCMAP/LEDMAP/PROPCAL/MAGCAL COMMIT 最终都调这一个入口。 */
static void save(void)
{
    APP_ControlConfig config;

    memset(&config, 0, sizeof(config));
    CHECK(APP_ControlConfigStore_Save(&config) == APP_FLASH_SERVICE_OK, 902);
}

/* = 重新上电：RAM 里的增益整份换成 Flash 里存的，由此看出存进去的是什么。 */
static void load(void)
{
    APP_ControlConfig config;

    memset(&config, 0, sizeof(config));
    CHECK(APP_ControlConfigStore_Load(&config) == APP_FLASH_SERVICE_OK, 903);
}

/* Flash 与 RAM 里都是这一组"正式配置"，没有试用。 */
static void fresh(void)
{
    fake_flash_reset();
    APP_ParamTrial_ClearAll();
    DRV_COAX_CTRL_ResetParams();
    param_set("coax.rate_roll_kp", 0.2f);
    param_set("coax.rate_pitch_kp", 0.2914f);
    param_set("coax.rate_pitch_i_limit_n_m", 0.01f);
    param_set("coax.att_pitch_kp", 1.632f);
    param_set("coax.pos_y_kp", 0.7f);
    save();
    CHECK(APP_ParamTrial_Count() == 0U, 904);
}

#ifndef APP_PARAM_TRIAL_CAPACITY

static uint8_t trialable(const char *name)
{
    return ((name != NULL) &&
            ((strncmp(name, "coax.rate_", 10U) == 0) ||
             (strncmp(name, "coax.att_", 9U) == 0) ||
             (strcmp(name, "coax.pos_z_kp") == 0) ||
             (strcmp(name, "coax.vel_z_kp") == 0) ||
             (strcmp(name, "coax.vel_z_ki") == 0) ||
             (strcmp(name, "coax.vel_z_kd") == 0) ||
             (strcmp(name, "coax.vel_z_i_limit_m_s2") == 0) ||
             (strcmp(name, "coax.pos_x_kp") == 0) ||
             (strcmp(name, "coax.vel_x_kp") == 0) ||
             (strcmp(name, "coax.vel_x_ki") == 0) ||
             (strcmp(name, "coax.vel_x_kd") == 0) ||
             (strcmp(name, "coax.vel_x_i_limit_m_s2") == 0))) ? 1U : 0U;
}

/* 2026-09-27 实机那一幕原样重放。 */
static void check_a_plain_param_set_does_not_persist_a_trial(void)
{
    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 10);   /* 临时应用 50% */
    CHECK(APP_ParamTrial_Count() == 1U, 11);
    param_set("coax.rate_pitch_i_limit_n_m", 0.05f);   /* 改积分限幅，1.5 s 后自动保存 */
    save();

    /* 控制器照旧用试用值：保存不许动 RAM，也不结束试用。 */
    CHECK(ram("coax.rate_pitch_kp") == 0.1457f, 12);
    CHECK(ram("coax.rate_pitch_i_limit_n_m") == 0.05f, 13);
    CHECK(APP_ParamTrial_Count() == 1U, 14);

    load();
    CHECK(ram("coax.rate_pitch_kp") == 0.2914f, 15);            /* 缺陷时这里是 0.1457 */
    CHECK(ram("coax.rate_pitch_i_limit_n_m") == 0.05f, 16);     /* 显式改的照常存 */
    CHECK(ram("coax.att_pitch_kp") == 1.632f, 17);
    CHECK(APP_ParamTrial_Count() == 0U, 18);                     /* LOAD 结束全部试用 */
}

static void check_param_set_on_the_same_name_adopts_it(void)
{
    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 20);
    param_set("coax.rate_pitch_kp", 0.2f);
    CHECK(APP_ParamTrial_Count() == 0U, 21);
    save();
    load();
    CHECK(ram("coax.rate_pitch_kp") == 0.2f, 22);

    /* 定下来最常见的写法：把正在试用的值原样 PARAM SET 一次。 */
    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 23);
    param_set("coax.rate_pitch_kp", 0.1457f);
    CHECK(APP_ParamTrial_Count() == 0U, 24);
    save();
    load();
    CHECK(ram("coax.rate_pitch_kp") == 0.1457f, 25);

    /* 别的名字的 PARAM SET 不结束这个名字的试用（上面第一组已验存盘结果）。 */
    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 26);
    param_set("coax.rate_roll_kp", 0.3f);
    param_set("airframe.board_mass_g", 80.0f);   /* 机体表那一路也走同一个入口 */
    CHECK(APP_ParamTrial_Count() == 1U, 27);
}

static void check_writing_back_the_original_ends_the_trial(void)
{
    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 30);
    CHECK(trial("coax.rate_pitch_kp", 0.2914f) == APP_PARAM_TRIAL_OK, 31);   /* 恢复原参数 */
    CHECK(APP_ParamTrial_Count() == 0U, 32);

    /* 上位机按六位小数回写：<1 的值差 1e-6 以内算写回原值，差得多就还是试用。 */
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 33);
    CHECK(trial("coax.rate_pitch_kp", 0.2914f + 9.0e-7f) == APP_PARAM_TRIAL_OK, 34);
    CHECK(APP_ParamTrial_Count() == 0U, 35);
    CHECK(trial("coax.rate_pitch_kp", 0.2914f + 3.0e-6f) == APP_PARAM_TRIAL_OK, 36);
    CHECK(APP_ParamTrial_Count() == 1U, 37);

    /* >1 的值按相对 1e-6。 */
    CHECK(trial("coax.att_pitch_kp", 0.816f) == APP_PARAM_TRIAL_OK, 38);
    CHECK(trial("coax.att_pitch_kp", 1.632f * (1.0f + 8.0e-7f)) == APP_PARAM_TRIAL_OK, 39);
    CHECK(APP_ParamTrial_Count() == 1U, 40);

    /* 发一个和当前 RAM 相同的值不是试用。 */
    CHECK(trial("coax.rate_roll_kp", 0.2f) == APP_PARAM_TRIAL_OK, 41);
    CHECK(APP_ParamTrial_Count() == 1U, 42);
}

static void check_a_repeated_trial_keeps_the_first_original(void)
{
    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 50);
    CHECK(trial("coax.rate_pitch_kp", 0.2185f) == APP_PARAM_TRIAL_OK, 51);   /* 50% → 75% */
    CHECK(APP_ParamTrial_Count() == 1U, 52);
    CHECK(ram("coax.rate_pitch_kp") == 0.2185f, 53);
    save();
    CHECK(ram("coax.rate_pitch_kp") == 0.2185f, 54);
    load();
    CHECK(ram("coax.rate_pitch_kp") == 0.2914f, 55);   /* 不是 50% 那一档 */
}

/*
 * 名字→Flash 字段的对应一个都不能错：错一个，就会有别的字段被写成持久值、试用值漏进去。
 * 名单从驱动具名表现取，含 v24/v25 的指令整形/出口陷波六项（另一块）。取值 2.0～5.3 同时落在
 * 增益（≥0）、陷波中心 [1,100] Hz、Q [0.3,10]、参考模型 [0.5,30] rad/s、延后 [0,80] ms 的合法区间里。
 */
static void check_every_trialable_name_round_trips(void)
{
    uint32_t n = 0U;

    fresh();
    for (uint32_t i = 0U; i < DRV_COAX_CTRL_ParamCount(); ++i) {
        const char *name = DRV_COAX_CTRL_ParamName(i);
        if (trialable(name) != 0U) {
            param_set(name, 2.0f + 0.1f * (float)n);
            n++;
        }
    }
    CHECK(n > 0U, 60);
    save();

    n = 0U;
    for (uint32_t i = 0U; i < DRV_COAX_CTRL_ParamCount(); ++i) {
        const char *name = DRV_COAX_CTRL_ParamName(i);
        if (trialable(name) != 0U) {
            CHECK(trial(name, 3.0f + 0.1f * (float)n) == APP_PARAM_TRIAL_OK, 61);
            n++;
        }
    }
    CHECK(APP_ParamTrial_Count() == n, 62);   /* 每个名字都放得下 */
    param_set("coax.pos_y_kp", 0.9f);         /* 名单外的照常按 RAM 存 */
    save();

    n = 0U;
    for (uint32_t i = 0U; i < DRV_COAX_CTRL_ParamCount(); ++i) {
        const char *name = DRV_COAX_CTRL_ParamName(i);
        if (trialable(name) != 0U) {
            CHECK(ram(name) == 3.0f + 0.1f * (float)n, 63);
            n++;
        }
    }
    load();
    n = 0U;
    for (uint32_t i = 0U; i < DRV_COAX_CTRL_ParamCount(); ++i) {
        const char *name = DRV_COAX_CTRL_ParamName(i);
        if (trialable(name) != 0U) {
            CHECK(ram(name) == 2.0f + 0.1f * (float)n, 64);
            n++;
        }
    }
    CHECK(ram("coax.pos_y_kp") == 0.9f, 65);
}

static void check_refused_trials_leave_ram_and_records_alone(void)
{
    fresh();
    CHECK(trial("coax.pos_y_kp", 1.5f) == APP_PARAM_TRIAL_NOT_TRIALABLE, 70);
    CHECK(ram("coax.pos_y_kp") == 0.7f, 71);
    CHECK(trial("coax.rate_bogus", 1.0f) == APP_PARAM_TRIAL_NOT_TRIALABLE, 72);
    CHECK(trial(NULL, 1.0f) == APP_PARAM_TRIAL_NOT_TRIALABLE, 73);
    CHECK(trial("coax.rate_pitch_kp", -1.0f) == APP_PARAM_TRIAL_REJECTED, 74);
    CHECK(trial("coax.rate_pitch_kp", NAN) == APP_PARAM_TRIAL_REJECTED, 75);
    CHECK(ram("coax.rate_pitch_kp") == 0.2914f, 76);
    CHECK(APP_ParamTrial_Count() == 0U, 77);
}

static void check_only_a_successful_load_ends_the_trials(void)
{
    APP_ControlConfig config;

    fresh();
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 80);
    CHECK(trial("coax.att_pitch_kp", 0.816f) == APP_PARAM_TRIAL_OK, 81);
    CHECK(APP_ParamTrial_Count() == 2U, 82);

    /* Flash 里读不出东西：RAM 没被换掉，试用还在。 */
    fake_flash_reset();
    memset(&config, 0, sizeof(config));
    CHECK(APP_ControlConfigStore_Load(&config) != APP_FLASH_SERVICE_OK, 83);
    CHECK(APP_ParamTrial_Count() == 2U, 84);
    CHECK(ram("coax.rate_pitch_kp") == 0.1457f, 85);

    /* 这时存一次：Flash 里的仍是试用前的值。 */
    save();
    load();
    CHECK(APP_ParamTrial_Count() == 0U, 86);
    CHECK(ram("coax.rate_pitch_kp") == 0.2914f, 87);
    CHECK(ram("coax.att_pitch_kp") == 1.632f, 88);
}

int main(void)
{
    DRV_COAX_CTRL_Init();
    check_a_plain_param_set_does_not_persist_a_trial();
    check_param_set_on_the_same_name_adopts_it();
    check_writing_back_the_original_ends_the_trial();
    check_a_repeated_trial_keeps_the_first_original();
    check_every_trialable_name_round_trips();
    check_refused_trials_leave_ram_and_records_alone();
    check_only_a_successful_load_ends_the_trials();
    REPORT();
}

#else /* APP_PARAM_TRIAL_CAPACITY：调小容量，专验满表 */

static void check_a_full_table_refuses_new_trials_without_touching_ram(void)
{
    fresh();
    CHECK(trial("coax.rate_roll_kp", 0.5f) == APP_PARAM_TRIAL_OK, 100);
    CHECK(trial("coax.rate_pitch_kp", 0.1457f) == APP_PARAM_TRIAL_OK, 101);
    CHECK(trial("coax.att_pitch_kp", 0.816f) == APP_PARAM_TRIAL_FULL, 102);
    CHECK(ram("coax.att_pitch_kp") == 1.632f, 103);   /* 拒绝就不写 RAM */
    CHECK(APP_ParamTrial_Count() == 2U, 104);

    /* 已在表里的名字照常改，不占新格。 */
    CHECK(trial("coax.rate_roll_kp", 0.6f) == APP_PARAM_TRIAL_OK, 105);
    /* 定下一个，腾出一格，新的才进得来。 */
    param_set("coax.rate_roll_kp", 0.6f);
    CHECK(trial("coax.att_pitch_kp", 0.816f) == APP_PARAM_TRIAL_OK, 106);

    save();
    load();
    CHECK(ram("coax.rate_roll_kp") == 0.6f, 107);
    CHECK(ram("coax.rate_pitch_kp") == 0.2914f, 108);
    CHECK(ram("coax.att_pitch_kp") == 1.632f, 109);
}

int main(void)
{
    DRV_COAX_CTRL_Init();
    check_a_full_table_refuses_new_trials_without_touching_ram();
    REPORT();
}

#endif
"""
)

TRIAL_SOURCES = [*CONFIG_STORE_SOURCES, ROOT / "App" / "Src" / "app_cmd_airframe.c"]


def _run_trial_harness(tmp_path: Path, name: str, flags: tuple[str, ...] = ()):
    fakes = write_fakes(tmp_path, {"cmsis_os2.h": FAKE_CMSIS_OS2_H, "main.h": FAKE_MAIN_H})
    return build_and_run(tmp_path, name, TRIAL_HARNESS, sources=TRIAL_SOURCES,
                         includes=config_store_includes(fakes), flags=flags)


def test_trial_values_never_reach_flash_through_another_save(tmp_path: Path) -> None:
    result = _run_trial_harness(tmp_path, "param_trial")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout


def test_a_full_trial_table_refuses_instead_of_dropping_a_record(tmp_path: Path) -> None:
    result = _run_trial_harness(tmp_path, "param_trial_full", ("-DAPP_PARAM_TRIAL_CAPACITY=2U",))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "all checks passed" in result.stdout


# ------------------------------------------------------------------ 命令面（sysid 宿主装置）


CMD_SOURCES = ("App/Src/app_cmd_sysid.c", "App/Src/app_param_trial.c",
               "tests/fixtures/sysid/cmd_harness.c")


def _prepare(handle):
    handle.harness_command.argtypes = [ctypes.c_char_p]
    handle.harness_command.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_GetParam.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_float)]
    handle.DRV_COAX_CTRL_GetParam.restype = ctypes.c_uint8
    handle.DRV_COAX_CTRL_SetParam.argtypes = [ctypes.c_char_p, ctypes.c_float]
    handle.DRV_COAX_CTRL_SetParam.restype = ctypes.c_uint8
    handle.APP_ParamTrial_Count.restype = ctypes.c_uint32
    return handle


@pytest.fixture(scope="module")
def trial_cmd_lib(tmp_path_factory):
    return _prepare(build_lib(tmp_path_factory, CMD_SOURCES, "sysid-param-trial"))


@pytest.fixture(scope="module")
def trial_cmd_lib_one_slot(tmp_path_factory):
    return _prepare(build_lib(tmp_path_factory, CMD_SOURCES, "sysid-param-trial-1",
                              flags=("-DAPP_PARAM_TRIAL_CAPACITY=1U",)))


def fmt6(value: float) -> str:
    """与固件同一写法：|x|·1e6 + 0.5 取整，六位小数。"""
    micro = int(abs(value) * 1_000_000.0 + 0.5)
    return f"{'-' if value < 0 else ''}{micro // 1_000_000}.{micro % 1_000_000:06d}"


def ram(lib, name: str) -> float:
    value = ctypes.c_float()
    assert lib.DRV_COAX_CTRL_GetParam(name.encode(), ctypes.byref(value)) == 1
    return value.value


def command(lib, line: str) -> list[str]:
    lib.harness_reset()
    assert lib.harness_command(line.encode()) == 1
    return texts(lib)


def baseline(lib) -> dict[str, float]:
    """一组正式配置（不经试用直接写驱动），返回它们在 RAM 里的 float 值。"""
    lib.APP_ParamTrial_ClearAll()
    for name, value in (("coax.rate_pitch_kp", 0.2914), ("coax.att_pitch_kp", 1.632),
                        ("coax.rate_roll_kp", 0.2)):
        assert lib.DRV_COAX_CTRL_SetParam(name.encode(), value) == 1
    return {name: ram(lib, name) for name in ("coax.rate_pitch_kp", "coax.att_pitch_kp",
                                              "coax.rate_roll_kp")}


def test_sysid_param_reports_the_trials_and_forgets_a_written_back_original(trial_cmd_lib) -> None:
    lib = trial_cmd_lib
    saved = baseline(lib)
    assert command(lib, "SYSID PARAM ?") == ["SYSID TRIAL n=0\r\n"]
    assert command(lib, "SYSID PARAM") == ["SYSID TRIAL n=0\r\n"]   # 不带参数 = 读回

    assert command(lib, "SYSID PARAM coax.rate_pitch_kp 0.1457") == [
        "OK sysid param name=coax.rate_pitch_kp value=0.145700 ram=1\r\n"]
    assert command(lib, "SYSID PARAM coax.att_pitch_kp 0.816") == [
        "OK sysid param name=coax.att_pitch_kp value=0.816000 ram=1\r\n"]
    # 同名再试用：RAM 跟着变，记下的原值不变；顺序按第一次试用的先后。
    command(lib, "SYSID PARAM coax.rate_pitch_kp 0.2185")
    lines = command(lib, "SYSID PARAM ?")
    assert lines == [
        "SYSID TRIAL n=2\r\n",
        f"SYSID TRIAL name=coax.rate_pitch_kp ram=0.218500 saved={fmt6(saved['coax.rate_pitch_kp'])}\r\n",
        f"SYSID TRIAL name=coax.att_pitch_kp ram=0.816000 saved={fmt6(saved['coax.att_pitch_kp'])}\r\n",
    ]
    assert all(len(line) < 255 for line in lines)

    # 上位机「恢复原参数」：按回显的六位小数写回原值 → 这一条试用结束。
    restore = f"SYSID PARAM coax.rate_pitch_kp {fmt6(saved['coax.rate_pitch_kp'])}"
    assert command(lib, restore)[0].startswith("OK sysid param name=coax.rate_pitch_kp ")
    assert command(lib, "SYSID PARAM ?") == [
        "SYSID TRIAL n=1\r\n",
        f"SYSID TRIAL name=coax.att_pitch_kp ram=0.816000 saved={fmt6(saved['coax.att_pitch_kp'])}\r\n",
    ]
    command(lib, f"SYSID PARAM coax.att_pitch_kp {fmt6(saved['coax.att_pitch_kp'])}")
    assert command(lib, "SYSID PARAM ?") == ["SYSID TRIAL n=0\r\n"]


def test_sysid_param_refusals_do_not_touch_ram(trial_cmd_lib) -> None:
    lib = trial_cmd_lib
    saved = baseline(lib)
    only = ("ERR sysid param only coax.rate_* / coax.att_* / coax.pos_z_kp / coax.vel_z_* / "
            "coax.pos_x_kp / coax.vel_x_*\r\n")
    # 水平槽只开放 x 通道；y 通道照样拒绝、不写 RAM。
    assert command(lib, "SYSID PARAM coax.pos_y_kp 1") == [only]
    assert command(lib, "SYSID PARAM coax.vel_y_kp 1") == [only]
    # z 通道里只有增益与积分限幅可试用；限速这类不在名单里的照样拒绝、不写 RAM。
    assert command(lib, "SYSID PARAM coax.pos_z_vel_up_max_m_s 1") == [only]
    assert command(lib, "SYSID PARAM coax.pos_xy_vel_max_m_s 1") == [only]
    assert command(lib, "SYSID PARAM coax.rate_bogus 1") == ["ERR sysid param coax.rate_bogus\r\n"]
    assert command(lib, "SYSID PARAM coax.rate_pitch_kp -1") == [
        "ERR sysid param coax.rate_pitch_kp\r\n"]
    assert command(lib, "SYSID PARAM coax.rate_pitch_kp") == [
        "ERR usage SYSID PARAM coax.<rate_*|att_*|pos_z_kp|vel_z_*|pos_x_kp|vel_x_*> <value>\r\n"]
    assert ram(lib, "coax.rate_pitch_kp") == saved["coax.rate_pitch_kp"]
    assert lib.APP_ParamTrial_Count() == 0


def test_sysid_param_trials_the_height_loop_names_in_ram_only(trial_cmd_lib) -> None:
    """高度环 z 通道与水平 x 通道各五项走同一条试用路径：写 RAM、进试用记录（保存时换回原值），写回原值即结束。"""
    lib = trial_cmd_lib
    lib.APP_ParamTrial_ClearAll()
    for name in sorted(LOOP_NAMES):
        original = ram(lib, f"coax.{name}")
        trial = original + 0.25
        assert command(lib, f"SYSID PARAM coax.{name} {fmt6(trial)}") == [
            f"OK sysid param name=coax.{name} value={fmt6(trial)} ram=1\r\n"]
        assert ram(lib, f"coax.{name}") == pytest.approx(trial, abs=1e-6)
        report = command(lib, "SYSID PARAM ?")
        assert f"SYSID TRIAL name=coax.{name} ram={fmt6(trial)} saved={fmt6(original)}\r\n" in report
        command(lib, f"SYSID PARAM coax.{name} {fmt6(original)}")
        assert lib.APP_ParamTrial_Count() == 0


def test_sysid_param_refuses_a_new_name_when_the_table_is_full(trial_cmd_lib_one_slot) -> None:
    lib = trial_cmd_lib_one_slot
    saved = baseline(lib)
    assert command(lib, "SYSID PARAM coax.rate_pitch_kp 0.1457")[0].startswith("OK sysid param ")
    assert command(lib, "SYSID PARAM coax.att_pitch_kp 0.816") == [
        "ERR sysid param trial full coax.att_pitch_kp\r\n"]
    assert ram(lib, "coax.att_pitch_kp") == saved["coax.att_pitch_kp"]
    # 已在表里的名字不占新格，照常改。
    assert command(lib, "SYSID PARAM coax.rate_pitch_kp 0.2185")[0].startswith("OK sysid param ")
    assert lib.APP_ParamTrial_Count() == 1
