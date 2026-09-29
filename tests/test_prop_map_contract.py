"""桨叶与电机接线标定（`Driver/*/drv_prop_map.*`）的契约。

这份标定是**偏航极性的唯一来源**，而且它整体嵌在 CFG 记录里存在用户板子上，
所以本文件钉的不是"代码长什么样"，而是四件出了错不会报警的事：

1. **字节布局冻结。** 它是 Flash ABI。改了尺寸/顺序，用户板子上已经存着的那份
   会整体错位——错位后的 `role`/`spin_sense` 仍然是合法枚举值，飞控照飞，
   只是偏航方向可能反了。

2. **半份标定必须被拒收。** 只填了一路的记录看起来像"标过了"，而另一路按 0 算。
   宁可整份作废、挡住解锁，也不要一个自称已标定的半成品。

3. **未标定时极性是 0，不是 +1。** "没有方向"和"默认正方向"是两回事。猜一个
   方向的后果是偏航正反馈，而它的现象和增益过大几乎一样，会被当成调参问题
   一路查下去——那正是这次改造要消灭的失效模式。

4. **共轴必反转。** 两路同向的组合在这架飞机上物理不存在，收下它等于让分配式
   按一个不存在的机构工作。

harness 用宿主 gcc **真编译固件那份 .c**，不在 Python 里重写一遍模型。
"""

from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
DRIVER_INC = ROOT / "Driver" / "Inc"
DRIVER_SRC = ROOT / "Driver" / "Src"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def strip_c_comments(text: str) -> str:
    """去掉 C 注释。

    断言"源码里**没有**某个标识符"时，注释是假阳性的主要来源：解释"为什么没有它"
    的那段话必然会提到它，于是断言永远为真。
    """
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


HARNESS = r"""
#include "drv_prop_map.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition, code) do { \
    if (!(condition)) { \
        fprintf(stderr, "check %d failed at line %d\n", (code), __LINE__); \
        return (code); \
    } \
} while (0)

static void calibrated(DRV_PropMap *out, uint8_t upper_channel, int8_t lower_spin)
{
    DRV_PropMap_Defaults(out);
    out->channel[upper_channel - 1U].role = (uint8_t)DRV_PROP_ROLE_UPPER;
    out->channel[upper_channel - 1U].spin_sense = (int8_t)(-lower_spin);
    out->channel[2U - upper_channel].role = (uint8_t)DRV_PROP_ROLE_LOWER;
    out->channel[2U - upper_channel].spin_sense = lower_spin;
    out->calibrated = 1U;
}

/* ---------------------------------------------------- 布局是 Flash ABI */

static int test_layout_is_frozen(void)
{
    CHECK(sizeof(DRV_PropChannel) == 4U, 10);
    CHECK(sizeof(DRV_PropMap) == 32U, 11);
    CHECK(offsetof(DRV_PropMap, channel) == 8U, 12);
    CHECK(offsetof(DRV_PropMap, calibrated) == 16U, 13);
    CHECK(offsetof(DRV_PropMap, generation) == 20U, 14);
    CHECK(DRV_PROP_ESC_CHANNEL_COUNT == 2U, 15);
    /* 旋向的取值必须是 +1/-1/0 本身，不是"某个枚举的序号"：推导式里它直接
     * 参与乘法，换成序号会安静地算出别的极性。 */
    CHECK(DRV_PROP_SPIN_CCW == 1, 16);
    CHECK(DRV_PROP_SPIN_CW == -1, 17);
    CHECK(DRV_PROP_SPIN_NONE == 0, 18);
    return 0;
}

/* ---------------------------------------------------- 出厂 = 未标定 */

static int test_defaults_are_empty_not_a_guess(void)
{
    DRV_PropMap map;

    DRV_PropMap_Defaults(&map);
    CHECK(map.magic == DRV_PROP_MAP_MAGIC, 20);
    CHECK(map.size == (uint16_t)sizeof(DRV_PropMap), 21);
    CHECK(map.calibrated == 0U, 22);
    /* 角色摆着是为了结构体里没有非法枚举值，但两路旋向必须都是 0：
     * 出厂状态如果带着一个"看起来像标过"的旋向，就等于替没量过的飞机编了个方向。 */
    CHECK(map.channel[0].spin_sense == DRV_PROP_SPIN_NONE, 23);
    CHECK(map.channel[1].spin_sense == DRV_PROP_SPIN_NONE, 24);
    CHECK(DRV_PropMap_Validate(&map) == 1U, 25);
    return 0;
}

/* ---------------------------------------------------- 校验 */

static int test_validate_rejects_half_a_calibration(void)
{
    DRV_PropMap map;

    /* 没标定却带着一路旋向：界面会显示"这一路已经填好了"，另一路按 0 算。 */
    DRV_PropMap_Defaults(&map);
    map.channel[0].spin_sense = DRV_PROP_SPIN_CW;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 30);

    /* 自称标定完了，却有一路没填旋向。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.channel[1].spin_sense = DRV_PROP_SPIN_NONE;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 31);
    return 0;
}

static int test_validate_rejects_physically_impossible_pairs(void)
{
    DRV_PropMap map;

    /* 两路都说自己是上桨。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.channel[1].role = (uint8_t)DRV_PROP_ROLE_UPPER;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 40);

    /* 共轴同向转：这架飞机上不存在这种机构。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.channel[0].spin_sense = DRV_PROP_SPIN_CW;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 41);

    /* 非法角色枚举。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.channel[0].role = 7U;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 42);

    /* 头部损坏。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.magic = 0xDEADBEEFUL;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 43);
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.schema = 99U;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 44);
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.size = 4U;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 45);
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    map.calibrated = 2U;
    CHECK(DRV_PropMap_Validate(&map) == 0U, 46);

    CHECK(DRV_PropMap_Validate(NULL) == 0U, 47);

    /* 合法的两种接法都要收。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    CHECK(DRV_PropMap_Validate(&map) == 1U, 48);
    calibrated(&map, 2U, DRV_PROP_SPIN_CCW);
    CHECK(DRV_PropMap_Validate(&map) == 1U, 49);
    return 0;
}

/* -------------------------------- 未标定 = 没有方向，不是默认正方向 */

static int test_uncalibrated_has_no_direction_at_all(void)
{
    DRV_PropMap_ResetActive();

    CHECK(DRV_PropMap_IsCalibrated() == 0U, 50);
    CHECK(DRV_PropMap_LowerSpinSense() == 0.0f, 51);
    CHECK(DRV_PropMap_YawTorquePolarity() == 0.0f, 52);
    /* 查不到通道时返回 0，调用方据此禁用输出。**绝不能**退回下标顺序——
     * 退回去正好是接反时最危险的那种行为。 */
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 0U, 53);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_LOWER) == 0U, 54);
    CHECK(DRV_PropMap_GetActiveGeneration() == 0U, 55);
    return 0;
}

/* -------------------------------- 极性推导：极性 = -下桨旋向 */

static int test_yaw_polarity_is_minus_the_lower_rotor_spin(void)
{
    DRV_PropMap map;

    /* 下桨俯视顺时针 -> 极性 +1：正偏航力矩靠**加大下桨**获得。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    CHECK(DRV_PropMap_PublishActive(&map) == 1U, 60);
    CHECK(DRV_PropMap_LowerSpinSense() == -1.0f, 61);
    CHECK(DRV_PropMap_YawTorquePolarity() == 1.0f, 62);

    /* 换成逆时针，极性必须跟着翻，且只由这一个事实决定。 */
    calibrated(&map, 1U, DRV_PROP_SPIN_CCW);
    CHECK(DRV_PropMap_PublishActive(&map) == 1U, 63);
    CHECK(DRV_PropMap_LowerSpinSense() == 1.0f, 64);
    CHECK(DRV_PropMap_YawTorquePolarity() == -1.0f, 65);

    /* 换通道归属不影响极性——极性只看旋向，不看谁接在哪。 */
    calibrated(&map, 2U, DRV_PROP_SPIN_CCW);
    CHECK(DRV_PropMap_PublishActive(&map) == 1U, 66);
    CHECK(DRV_PropMap_YawTorquePolarity() == -1.0f, 67);
    return 0;
}

static int test_channel_lookup_follows_the_declared_wiring(void)
{
    DRV_PropMap map;

    calibrated(&map, 1U, DRV_PROP_SPIN_CW);
    CHECK(DRV_PropMap_PublishActive(&map) == 1U, 70);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 1U, 71);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_LOWER) == 2U, 72);

    /* 反着接：控制器算出的"上桨推力"必须去通道 2。 */
    calibrated(&map, 2U, DRV_PROP_SPIN_CW);
    CHECK(DRV_PropMap_PublishActive(&map) == 1U, 73);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 2U, 74);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_LOWER) == 1U, 75);

    /* 认不出的角色返回 0，不是"随便给一个"。 */
    CHECK(DRV_PropMap_EscChannelForRole(9U) == 0U, 76);
    return 0;
}

/* -------------------------------- 运行期快照 */

static int test_publish_read_and_generation(void)
{
    DRV_PropMap written;
    DRV_PropMap readback;
    uint32_t before;

    DRV_PropMap_ResetActive();
    CHECK(DRV_PropMap_GetActiveGeneration() == 0U, 80);

    calibrated(&written, 2U, DRV_PROP_SPIN_CW);
    before = DRV_PropMap_GetActiveGeneration();
    CHECK(DRV_PropMap_PublishActive(&written) == 1U, 81);
    CHECK(DRV_PropMap_GetActiveGeneration() == before + 1U, 82);

    CHECK(DRV_PropMap_ReadActive(&readback) == 1U, 83);
    CHECK(readback.calibrated == 1U, 84);
    CHECK(readback.channel[1].role == (uint8_t)DRV_PROP_ROLE_UPPER, 85);
    CHECK(readback.generation == DRV_PropMap_GetActiveGeneration(), 86);
    return 0;
}

static int test_publishing_an_invalid_map_changes_nothing(void)
{
    /* 拒收之后 active 必须还是上一份好的，不能变成半新半旧——更不能变成
     * "还标着已标定，但通道查不到了"。 */
    DRV_PropMap good;
    DRV_PropMap bad;
    DRV_PropMap readback;

    calibrated(&good, 1U, DRV_PROP_SPIN_CW);
    CHECK(DRV_PropMap_PublishActive(&good) == 1U, 90);

    calibrated(&bad, 1U, DRV_PROP_SPIN_CW);
    bad.channel[0].spin_sense = DRV_PROP_SPIN_CW;    /* 两路同向 */
    CHECK(DRV_PropMap_PublishActive(&bad) == 0U, 91);

    CHECK(DRV_PropMap_ReadActive(&readback) == 1U, 92);
    CHECK(readback.channel[0].role == (uint8_t)DRV_PROP_ROLE_UPPER, 93);
    CHECK(DRV_PropMap_YawTorquePolarity() == 1.0f, 94);
    CHECK(DRV_PropMap_IsCalibrated() == 1U, 95);
    return 0;
}

static int test_publishing_an_empty_map_takes_the_direction_away(void)
{
    /*
     * 上位机改到一半（只填了一路）时，运行期会被推成"未标定"。这一步必须真的
     * 把方向拿走：留着上一份会让"我正在改"和"飞控正在用"不是同一件事。
     */
    DRV_PropMap good;
    DRV_PropMap blank;

    calibrated(&good, 1U, DRV_PROP_SPIN_CW);
    CHECK(DRV_PropMap_PublishActive(&good) == 1U, 100);
    CHECK(DRV_PropMap_IsCalibrated() == 1U, 101);

    DRV_PropMap_Defaults(&blank);
    CHECK(DRV_PropMap_PublishActive(&blank) == 1U, 102);
    CHECK(DRV_PropMap_IsCalibrated() == 0U, 103);
    CHECK(DRV_PropMap_YawTorquePolarity() == 0.0f, 104);
    CHECK(DRV_PropMap_EscChannelForRole((uint8_t)DRV_PROP_ROLE_UPPER) == 0U, 105);
    return 0;
}

static int test_reading_before_any_publish_reports_uncalibrated(void)
{
    /* 静态区全零：magic 不对。交"未标定"，不要交一份 magic 为 0 的坏记录——
     * 上位机看到坏记录会以为 Flash 损坏，而实际只是还没标定过。 */
    DRV_PropMap readback;

    DRV_PropMap_ResetActive();
    CHECK(DRV_PropMap_ReadActive(&readback) == 1U, 110);
    CHECK(readback.magic == DRV_PROP_MAP_MAGIC, 111);
    CHECK(readback.calibrated == 0U, 112);
    CHECK(DRV_PropMap_Validate(&readback) == 1U, 113);
    return 0;
}

/* -------------------------------- 名字表 */

static int test_names_round_trip(void)
{
    uint8_t role;
    int8_t spin;

    CHECK(DRV_PropMap_RoleFromName("upper", &role) == 1U, 120);
    CHECK(role == (uint8_t)DRV_PROP_ROLE_UPPER, 121);
    CHECK(DRV_PropMap_RoleFromName("lower", &role) == 1U, 122);
    CHECK(role == (uint8_t)DRV_PROP_ROLE_LOWER, 123);
    CHECK(DRV_PropMap_RoleFromName("middle", &role) == 0U, 124);
    CHECK(DRV_PropMap_RoleFromName(NULL, &role) == 0U, 125);
    CHECK(strcmp(DRV_PropMap_RoleName((uint8_t)DRV_PROP_ROLE_UPPER), "upper") == 0, 126);
    CHECK(strcmp(DRV_PropMap_RoleName(9U), "-") == 0, 127);

    CHECK(DRV_PropMap_SpinFromName("cw", &spin) == 1U, 130);
    CHECK(spin == DRV_PROP_SPIN_CW, 131);
    CHECK(DRV_PropMap_SpinFromName("ccw", &spin) == 1U, 132);
    CHECK(spin == DRV_PROP_SPIN_CCW, 133);
    CHECK(DRV_PropMap_SpinFromName("clockwise", &spin) == 0U, 134);
    CHECK(strcmp(DRV_PropMap_SpinName(DRV_PROP_SPIN_NONE), "-") == 0, 135);
    return 0;
}

int main(void)
{
    int rc;

    rc = test_layout_is_frozen(); if (rc) { return rc; }
    rc = test_defaults_are_empty_not_a_guess(); if (rc) { return rc; }
    rc = test_validate_rejects_half_a_calibration(); if (rc) { return rc; }
    rc = test_validate_rejects_physically_impossible_pairs(); if (rc) { return rc; }
    rc = test_uncalibrated_has_no_direction_at_all(); if (rc) { return rc; }
    rc = test_yaw_polarity_is_minus_the_lower_rotor_spin(); if (rc) { return rc; }
    rc = test_channel_lookup_follows_the_declared_wiring(); if (rc) { return rc; }
    rc = test_publish_read_and_generation(); if (rc) { return rc; }
    rc = test_publishing_an_invalid_map_changes_nothing(); if (rc) { return rc; }
    rc = test_publishing_an_empty_map_takes_the_direction_away(); if (rc) { return rc; }
    rc = test_reading_before_any_publish_reports_uncalibrated(); if (rc) { return rc; }
    rc = test_names_round_trip(); if (rc) { return rc; }

    printf("prop map harness ok\n");
    return 0;
}
"""


def test_prop_map_on_host_gcc(tmp_path) -> None:
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("host gcc is unavailable")

    harness = tmp_path / "harness.c"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = tmp_path / "harness.exe"
    subprocess.run(
        [
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror",
            f"-I{DRIVER_INC}",
            str(DRIVER_SRC / "drv_prop_map.c"),
            str(harness), "-o", str(executable),
        ],
        check=True, capture_output=True, text=True,
    )
    result = subprocess.run([str(executable)], check=True, capture_output=True)
    assert "prop map harness ok" in result.stdout.decode("ascii", "replace")


def test_the_channel_count_matches_the_board(tmp_path) -> None:
    """`DRV_PROP_ESC_CHANNEL_COUNT` 必须等于板子真有的 ESC 路数。

    本层不许包含 BSP 头（否则上面那个 harness 直接编不过），所以两边只能各写
    一份常量。写岔了不会有任何编译错误：多出来的那一路永远查不到通道，
    少掉的那一路永远标不上——都表现为"标定页某一行怎么点都没反应"。
    """
    prop = read("Driver/Inc/drv_prop_map.h")
    pwm = read("BSP/Inc/bsp_pwm.h")

    theirs = re.search(r"#define BSP_PWM_ESC_CHANNEL_COUNT (\d+)U", pwm)
    ours = re.search(r"#define DRV_PROP_ESC_CHANNEL_COUNT (\d+)U", prop)
    assert theirs is not None and ours is not None
    assert ours.group(1) == theirs.group(1)


def test_the_driver_stays_free_of_hardware() -> None:
    """它在 Driver 层，禁止碰 HAL/BSP——否则上面那个 harness 直接编不过。"""
    # 先去注释：头文件里正当地提到过 `BSP/Inc/bsp_pwm.h`（说明通道 1/2 对应
    # 哪个焊盘）。不去注释的话这条会被那句话误伤，而真正的 #include 反而查不到。
    for path in ("Driver/Inc/drv_prop_map.h", "Driver/Src/drv_prop_map.c"):
        text = strip_c_comments(read(path))
        for banned in ("stm32h7xx_hal", "main.h", "cmsis_os", "HAL_", "bsp_"):
            assert banned not in text, f"{path} 引入了 {banned}"


def test_the_controller_reads_the_calibration_and_nothing_else() -> None:
    """偏航极性只有一个来源。

    钉两头：`drv_coax_ctrl.c` 必须调标定；退役的机体字段必须没人再读。
    只钉前者的话，把旧字段当成"兜底"再读一次会静默通过——而两个来源打架时，
    谁也不会报错，只有飞机知道。
    """
    coax = strip_c_comments(read("Driver/Src/drv_coax_ctrl.c"))
    assert "DRV_PropMap_YawTorquePolarity()" in coax
    assert "lower_rotor_spin_sense" not in coax

    stabilizer = strip_c_comments(read("App/Src/app_stabilizer.c"))
    assert "lower_rotor_spin_sense" not in stabilizer


def test_the_flash_record_counts_the_prop_block() -> None:
    """CFG 记录的校验跨度必须把桨叶标定块算进去。

    v20 加机体模型块时就漏在这里：Save 写的 size 含新块、读回来的校验式不含，
    于是每一条自己写的记录都过不了自己的检查，而且**没有任何报错**。
    """
    store = read("App/Src/app_control_config_store.c")

    size_macro = store[store.index("#define APP_CONTROL_CFG_CURRENT_SIZE"):]
    size_macro = size_macro[:size_macro.index("_Static_assert")]
    assert "->prop)" in size_macro, "CURRENT_SIZE 漏算了桨叶标定块"

    assert "app_cmd_propcal_apply_config(&record.prop)" in store
    assert "record.prop = *(const DRV_PropMap *)app_cmd_propcal_config()" in store


def test_every_legacy_reader_explicitly_handles_the_new_block() -> None:
    """每条迁移路径都必须显式把这一块落回"未标定"。

    什么都不做的后果不是"用默认值"，而是**留着 RAM 里上一次的标定**：
    读进来一条 v21 旧记录之后，飞控会继续用上一架飞机的旋向飞，
    而 `PROPCAL?` 报的和 Flash 里存的不是一回事。LED 块当年就是这么定的规矩。

    这里逐个读取器地数，而不是只看"文件里出现过 NULL 调用"：漏掉的往往只是
    其中一条路径，而那条路径平时走不到。
    """
    store = read("App/Src/app_control_config_store.c")
    body = store[store.index("APP_CONTROL_DEFINE_LEGACY_READER(config_read_v21"):]
    body = body[:body.index("APP_FlashService_Status APP_ControlConfigStore_Load")]

    readers = re.findall(r"APP_CONTROL_DEFINE_LEGACY_READER\((config_read_v\d+)", body)
    assert readers == ["config_read_v21", "config_read_v20", "config_read_v19",
                       "config_read_v18", "config_read_v17", "config_read_v16",
                       "config_read_v15"], readers
    assert body.count("app_cmd_propcal_apply_config(NULL)") == len(readers)
    # 宏体里不许再留"无条件兜底"：v21 已经带了 LED 块，一句无脑的
    # `app_cmd_ledmap_apply_config(NULL)` 会把刚读出来的颜色当场抹掉。
    macro = store[store.index("#define APP_CONTROL_DEFINE_LEGACY_READER"):]
    macro = macro[:macro.index("\nAPP_CONTROL_DEFINE_LEGACY_READER(")]
    assert "app_cmd_ledmap_apply_config" not in macro
    assert "app_cmd_propcal_apply_config" not in macro


def test_the_calibration_is_never_seeded_from_the_retired_field() -> None:
    """**绝不能**从 `retired_lower_rotor_spin_sense` 迁移。

    那个值是从调参现象反推的。搬进来就等于让一份从没量过的旋向顶着"已标定"的
    名义决定偏航方向——这次改造的全部意义就是不再让那种值决定偏航方向。
    """
    propcal = strip_c_comments(read("App/Src/app_cmd_propcal.c"))
    assert "retired_lower_rotor_spin_sense" not in propcal
    assert "DRV_Airframe" not in propcal
