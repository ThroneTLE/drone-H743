#include "drv_prop_map.h"

#include <string.h>

/* ────────────────────────────────────────────── 出厂状态与校验 */

void DRV_PropMap_Defaults(DRV_PropMap *out)
{
    uint32_t i;

    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->magic = DRV_PROP_MAP_MAGIC;
    out->schema = (uint16_t)DRV_PROP_MAP_SCHEMA;
    out->size = (uint16_t)sizeof(DRV_PropMap);

    /*
     * 角色摆成"通道1=上、通道2=下"只是为了让结构体里没有非法枚举值，
     * **不是默认接线**：calibrated 为 0 时谁也不许用它。按下标猜接线正是
     * 这个模块要消灭的东西。
     */
    for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
        out->channel[i].role = (uint8_t)((i == 0U) ? DRV_PROP_ROLE_UPPER
                                                   : DRV_PROP_ROLE_LOWER);
        out->channel[i].spin_sense = DRV_PROP_SPIN_NONE;
    }
    out->calibrated = 0U;
}

static uint8_t prop_map_spin_is_defined(int8_t spin_sense)
{
    return ((spin_sense == DRV_PROP_SPIN_CCW) ||
            (spin_sense == DRV_PROP_SPIN_CW)) ? 1U : 0U;
}

uint8_t DRV_PropMap_Validate(const DRV_PropMap *map)
{
    uint32_t i;
    uint8_t role_seen[DRV_PROP_ROLE_COUNT];

    if (map == NULL) {
        return 0U;
    }
    if ((map->magic != DRV_PROP_MAP_MAGIC) ||
        (map->schema != (uint16_t)DRV_PROP_MAP_SCHEMA) ||
        (map->size != (uint16_t)sizeof(DRV_PropMap))) {
        return 0U;
    }
    if (map->calibrated > 1U) {
        return 0U;
    }

    if (map->calibrated == 0U) {
        /*
         * 半份标定比没标定更危险：界面会显示"这一路已经填好了"，而另一路
         * 悄悄按 0 算。要么整份有效，要么整份是空的。
         */
        for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
            if (map->channel[i].spin_sense != DRV_PROP_SPIN_NONE) {
                return 0U;
            }
        }
        return 1U;
    }

    memset(role_seen, 0, sizeof(role_seen));
    for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
        const uint8_t role = map->channel[i].role;

        if (role >= (uint8_t)DRV_PROP_ROLE_COUNT) {
            return 0U;
        }
        if (role_seen[role] != 0U) {
            return 0U;      /* 两路都说自己是上桨 */
        }
        role_seen[role] = 1U;

        if (prop_map_spin_is_defined(map->channel[i].spin_sense) == 0U) {
            return 0U;
        }
    }

    /*
     * 共轴必反转。两路同向的组合在这架飞机上物理不存在，而它算出来的
     * 偏航力矩是 `Mz = s*(ku*T_upper - kl*T_lower)` 之外的另一回事——
     * 与其让分配式按一个不存在的机构工作，不如当场拒收。
     */
    if ((int)map->channel[0].spin_sense + (int)map->channel[1].spin_sense != 0) {
        return 0U;
    }
    return 1U;
}

/* ──────────────────────────────────── 运行期快照 */

static DRV_PropMap prop_active;
static volatile uint32_t prop_active_sequence;
static volatile uint32_t prop_active_generation;

/*
 * 500 Hz 提交点用的派生字。一个 32 位量，单次读写在 Cortex-M7 与宿主上都是
 * 原子的，所以控制环不必走 seqlock，也不可能读到半新半旧的组合。
 *
 *   bit 0..1 : 上桨接在哪个 ESC 通道（1..2），0 = 未标定
 *   bit 2..3 : 下桨接在哪个 ESC 通道（1..2），0 = 未标定
 *   bit 4..5 : 下桨旋向  0 = 未标定，1 = +1（俯视逆时针），2 = -1（俯视顺时针）
 */
#define PROP_DERIVED_UPPER_SHIFT 0U
#define PROP_DERIVED_LOWER_SHIFT 2U
#define PROP_DERIVED_SPIN_SHIFT  4U
#define PROP_DERIVED_FIELD_MASK  0x3U
#define PROP_DERIVED_SPIN_CCW    1U
#define PROP_DERIVED_SPIN_CW     2U

static volatile uint32_t prop_active_derived;

static uint32_t prop_map_build_derived(const DRV_PropMap *map)
{
    uint32_t derived = 0U;
    uint32_t i;

    if ((map == NULL) || (map->calibrated == 0U)) {
        return 0U;
    }

    for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
        const uint32_t channel_number = i + 1U;
        const uint32_t shift = (map->channel[i].role == (uint8_t)DRV_PROP_ROLE_UPPER)
                                   ? PROP_DERIVED_UPPER_SHIFT
                                   : PROP_DERIVED_LOWER_SHIFT;

        derived |= (channel_number & PROP_DERIVED_FIELD_MASK) << shift;

        if (map->channel[i].role == (uint8_t)DRV_PROP_ROLE_LOWER) {
            const uint32_t spin =
                (map->channel[i].spin_sense == DRV_PROP_SPIN_CCW)
                    ? PROP_DERIVED_SPIN_CCW
                    : PROP_DERIVED_SPIN_CW;

            derived |= spin << PROP_DERIVED_SPIN_SHIFT;
        }
    }
    return derived;
}

void DRV_PropMap_ResetActive(void)
{
    /* 先断掉派生字。控制环在这之后的任何一拍都读到"未标定"，不会用到正在被
     * 改写的那份结构体。 */
    prop_active_derived = 0U;
    prop_active_sequence++;
    DRV_PropMap_Defaults(&prop_active);
    prop_active_generation = 0U;
    prop_active.generation = 0U;
    prop_active_sequence++;
}

uint8_t DRV_PropMap_PublishActive(const DRV_PropMap *map)
{
    if (DRV_PropMap_Validate(map) == 0U) {
        return 0U;
    }

    prop_active_derived = 0U;
    prop_active_sequence++;
    prop_active = *map;
    prop_active_generation++;
    prop_active.generation = prop_active_generation;
    prop_active_sequence++;
    /* 派生字最后写：写到它的那一拍起，控制环才看得见这份新标定。 */
    prop_active_derived = prop_map_build_derived(&prop_active);
    return 1U;
}

uint8_t DRV_PropMap_ReadActive(DRV_PropMap *out)
{
    uint32_t before;
    uint32_t after;
    uint8_t attempt;

    if (out == NULL) {
        return 0U;
    }
    for (attempt = 0U; attempt < 4U; attempt++) {
        before = prop_active_sequence;
        if ((before & 1U) != 0U) {
            continue;           /* 写者正在中间，重读 */
        }
        *out = prop_active;
        after = prop_active_sequence;
        if (before == after) {
            if (out->magic != DRV_PROP_MAP_MAGIC) {
                /* 首次 Publish 之前静态区还是全零。全零的 magic/size 过不了
                 * 校验，交出来会让上位机以为读到了一份坏记录；交"未标定"。 */
                DRV_PropMap_Defaults(out);
            }
            return 1U;
        }
    }
    DRV_PropMap_Defaults(out);
    return 0U;
}

uint32_t DRV_PropMap_GetActiveGeneration(void)
{
    return prop_active_generation;
}

/* ──────────────────────────────────── 500 Hz 提交点读到的东西 */

uint8_t DRV_PropMap_IsCalibrated(void)
{
    return (prop_active_derived != 0U) ? 1U : 0U;
}

float DRV_PropMap_LowerSpinSense(void)
{
    const uint32_t spin =
        (prop_active_derived >> PROP_DERIVED_SPIN_SHIFT) & PROP_DERIVED_FIELD_MASK;

    if (spin == PROP_DERIVED_SPIN_CCW) {
        return 1.0f;
    }
    if (spin == PROP_DERIVED_SPIN_CW) {
        return -1.0f;
    }
    return 0.0f;
}

float DRV_PropMap_YawTorquePolarity(void)
{
    return -DRV_PropMap_LowerSpinSense();
}

uint8_t DRV_PropMap_EscChannelForRole(uint8_t role)
{
    uint32_t shift;

    if (role == (uint8_t)DRV_PROP_ROLE_UPPER) {
        shift = PROP_DERIVED_UPPER_SHIFT;
    } else if (role == (uint8_t)DRV_PROP_ROLE_LOWER) {
        shift = PROP_DERIVED_LOWER_SHIFT;
    } else {
        return 0U;
    }
    return (uint8_t)((prop_active_derived >> shift) & PROP_DERIVED_FIELD_MASK);
}

/* ──────────────────────────────────── 名字表 */

const char *DRV_PropMap_RoleName(uint8_t role)
{
    if (role == (uint8_t)DRV_PROP_ROLE_UPPER) {
        return "upper";
    }
    if (role == (uint8_t)DRV_PROP_ROLE_LOWER) {
        return "lower";
    }
    return "-";
}

uint8_t DRV_PropMap_RoleFromName(const char *name, uint8_t *role)
{
    if ((name == NULL) || (role == NULL)) {
        return 0U;
    }
    if (strcmp(name, "upper") == 0) {
        *role = (uint8_t)DRV_PROP_ROLE_UPPER;
        return 1U;
    }
    if (strcmp(name, "lower") == 0) {
        *role = (uint8_t)DRV_PROP_ROLE_LOWER;
        return 1U;
    }
    return 0U;
}

const char *DRV_PropMap_SpinName(int8_t spin_sense)
{
    if (spin_sense == DRV_PROP_SPIN_CCW) {
        return "ccw";
    }
    if (spin_sense == DRV_PROP_SPIN_CW) {
        return "cw";
    }
    return "-";
}

uint8_t DRV_PropMap_SpinFromName(const char *name, int8_t *spin_sense)
{
    if ((name == NULL) || (spin_sense == NULL)) {
        return 0U;
    }
    if (strcmp(name, "ccw") == 0) {
        *spin_sense = DRV_PROP_SPIN_CCW;
        return 1U;
    }
    if (strcmp(name, "cw") == 0) {
        *spin_sense = DRV_PROP_SPIN_CW;
        return 1U;
    }
    return 0U;
}
