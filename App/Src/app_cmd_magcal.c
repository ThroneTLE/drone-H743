/*
 * MAGCAL / MAGFRAME 命令族：磁力计硬磁/软磁校准系数与轴向验证状态的读写。
 *
 * 状态机与 `app_cmd_ledmap.c` / `app_cmd_propcal.c` 同构（同一类问题：一份存
 * Flash 的配置，要能在线改、立刻看到效果、确认满意了再落盘），细节上有一处
 * 不同：校准系数是 12 个浮点数（3 硬磁零偏 + 9 软磁矩阵），单条命令塞不下
 * （`app_control_tokenize` 只给 `tokens[10]`，见 app_cmd_ledmap.c 顶部注释同一个
 * 坑），所以拆成 BIAS 一条、MATRIX 一行一条，分批填进一份草稿（draft），
 * 全部到齐再用 APPLY 一次性校验、生效：
 *
 *   SET BIAS/MATRIX   只改草稿，不影响当前生效的校准
 *   APPLY             草稿凑齐 + 校验通过 → 覆盖当前生效的 RAM 工作副本
 *   CLEAR             工作副本（和草稿）一起回到出厂未校准状态
 *   COMMIT            才真正写 Flash（同 LEDMAP，落盘是同步擦扇区，别在这之前做）
 *
 * MAGFRAME 是同一份 RAM 工作副本里的轴向验证位；VERIFY 需要 CONFIRM 字面量，
 * 避免误敲导致一条从未验证过的贴装朝向被标记为"验证过"。
 *
 * 校准系数 + 轴向验证状态都存进同一个 DRV_MAG_Calibration 记录（C3/C6），
 * 持久化形态见 app_control_config_store.c 里新增的 v23 mag 块——和 LEDMAP/
 * PROPCAL 一样,"旧记录没有这一块"与"存的那份过不了校验"都必须显式落回
 * 出厂未校准状态，不能让 RAM 里留着上一次的值。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_current_format.h"
#include "app_flash_service.h"
#include "app_magcal.h"
#include "app_magxy.h"
#include "app_stabilizer.h"
#include "bsp_critical.h"
#include "drv_frame_contract.h"
#include "drv_mag_calibration.h"

#include <stddef.h>
#include <string.h>

/*
 * `magcal_config` 是 App/Src/app_mag.c 在 Sensor_Task（20Hz）里通过
 * APP_MagCal_GetEffective() 读取的跨任务状态，而命令族在 UARTTask/messageTask
 * 上下文里改它。几十字节的结构体，临界区拷贝足够（decoupling-spec D2-5），
 * 不必上双缓冲——但和 LEDMAP 不同，这份数据直接喂进姿态融合，torn read 不是
 * "颜色抖一下"那种无害后果，所以读写两侧都包临界区，LEDMAP 没做这一步。
 */

static DRV_MAG_Calibration magcal_config;
static uint8_t magcal_loaded;
static uint8_t magcal_dirty;

/* SET 的暂存草稿：BIAS 与 MATRIX 各行分开到达，APPLY 时一次性校验+生效。 */
static DRV_MAG_Calibration magcal_draft;
static uint8_t magcal_draft_bias_set;
static uint8_t magcal_draft_row_set[3];

static void magcal_defaults(DRV_MAG_Calibration *calibration)
{
    memset(calibration, 0, sizeof(*calibration));
    calibration->soft_iron_matrix[0][0] = 1.0f;
    calibration->soft_iron_matrix[1][1] = 1.0f;
    calibration->soft_iron_matrix[2][2] = 1.0f;
    /*
     * calibrated=0, axis_verified=DRV_MAG_CAL_AXIS_UNVERIFIED(0),
     * frame_contract_version=0, hard_iron_bias_mgauss={0,0,0}：正是 C3/C4
     * 要求的默认/未校准/未验证状态。
     */
}

static void magcal_clear_draft(void)
{
    magcal_defaults(&magcal_draft);
    magcal_draft_bias_set = 0U;
    magcal_draft_row_set[0] = 0U;
    magcal_draft_row_set[1] = 0U;
    magcal_draft_row_set[2] = 0U;
}

static void magcal_ensure_loaded(void)
{
    if (magcal_loaded == 0U) {
        magcal_defaults(&magcal_config);
        magcal_clear_draft();
        magcal_loaded = 1U;
    }
}

static uint8_t magcal_write_allowed(void)
{
    /*
     * 改校准系数或轴向验证位，等于改"这个磁力计读数该怎么解释"。解锁状态下
     * 改一次，就可能让姿态融合在桨正在转的时候突然多吃一路此前没有的观测——
     * 和 LEDMAP / IMUFRAME 的安全门同源。
     */
    return (APP_Stabilizer_IsArmed() == 0U) ? 1U : 0U;
}

static const char *magcal_status_text(DRV_MAG_CalibrationStatus status)
{
    switch (status) {
    case DRV_MAG_CAL_VALID:
        return "valid";
    case DRV_MAG_CAL_INVALID_ARGS:
        return "invalid_args";
    case DRV_MAG_CAL_INVALID_NONFINITE:
        return "nonfinite";
    case DRV_MAG_CAL_INVALID_DETERMINANT:
        return "bad_determinant";
    case DRV_MAG_CAL_INVALID_SCALE:
        return "bad_scale";
    default:
        return "unknown";
    }
}

/* ─────────────────────────────────────────────────────────── 回包 */

static void magcal_report_status(void)
{
    const DRV_MAG_Calibration *c = &magcal_config;
    uint8_t axis_effective = (c->frame_contract_version ==
                              (uint32_t)DRV_FRAME_CONTRACT_VERSION) ?
        c->axis_verified : DRV_MAG_CAL_AXIS_UNVERIFIED;

    APP_Control_QueueText(
        "MAGCAL calibrated=%u axis_verified=%u axis_effective=%u "
        "contract_stored=%lu contract_live=%lu dirty=%u\r\n",
        (unsigned int)c->calibrated,
        (unsigned int)c->axis_verified,
        (unsigned int)axis_effective,
        (unsigned long)c->frame_contract_version,
        (unsigned long)DRV_FRAME_CONTRACT_VERSION,
        (unsigned int)magcal_dirty);
    /* 固件链接的是 newlib-nano，%f 输出为空；小数一律走定点格式化。 */
    char b[3][16];
    char m[9][16];
    for (uint32_t i = 0U; i < 3U; i++) {
        (void)APP_Current_FormatFixed(b[i], c->hard_iron_bias_mgauss[i], 3U);
    }
    for (uint32_t i = 0U; i < 9U; i++) {
        (void)APP_Current_FormatFixed(m[i], c->soft_iron_matrix[i / 3U][i % 3U], 4U);
    }
    APP_Control_QueueText("MAGCAL bias_mgauss=%s,%s,%s\r\n", b[0], b[1], b[2]);
    APP_Control_QueueText(
        "MAGCAL matrix row0=%s,%s,%s row1=%s,%s,%s row2=%s,%s,%s\r\n",
        m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    APP_Control_QueueText(
        "MAGCAL draft bias_set=%u row_set=%u,%u,%u\r\n",
        (unsigned int)magcal_draft_bias_set,
        (unsigned int)magcal_draft_row_set[0],
        (unsigned int)magcal_draft_row_set[1],
        (unsigned int)magcal_draft_row_set[2]);
    {
        /*
         * 区分"没装磁力计"和"装了但被拒了"：subsystem_enabled 是门控 1-3 条
         * 的合取，used 是这一拍真的喂进了 Fusion。二者都为 0 时看
         * field_rejected/ignored 具体是哪一层拒的（Driver 的场强门控，还是
         * Fusion 库自己的内部磁力拒绝）。
         */
        APP_Stabilizer_MagFusionStatus fusion_status;
        char error_deg[16];

        APP_Stabilizer_GetMagFusionStatus(&fusion_status);
        (void)APP_Current_FormatFixed(error_deg, fusion_status.error_deg, 2U);
        APP_Control_QueueText(
            "MAGCAL fusion subsystem_enabled=%u used=%u field_rejected=%u "
            "ignored=%u recovery=%u error_deg=%s\r\n",
            (unsigned int)fusion_status.subsystem_enabled,
            (unsigned int)fusion_status.used,
            (unsigned int)fusion_status.field_rejected,
            (unsigned int)fusion_status.ignored,
            (unsigned int)fusion_status.recovery,
            error_deg);
    }
}

static void magcal_report_frame(void)
{
    const DRV_MAG_Calibration *c = &magcal_config;
    uint8_t axis_effective = (c->frame_contract_version ==
                              (uint32_t)DRV_FRAME_CONTRACT_VERSION) ?
        c->axis_verified : DRV_MAG_CAL_AXIS_UNVERIFIED;

    APP_Control_QueueText(
        "MAGFRAME axis_verified=%u axis_effective=%u contract_stored=%lu "
        "contract_live=%lu derivation=svc_mag_default_rotation_unverified\r\n",
        (unsigned int)c->axis_verified,
        (unsigned int)axis_effective,
        (unsigned long)c->frame_contract_version,
        (unsigned long)DRV_FRAME_CONTRACT_VERSION);
}

/* ─────────────────────────────────────────────────── MAGCAL 子命令 */

static void magcal_handle_set(char **tokens, uint32_t count)
{
    if (count < 3U) {
        APP_Control_QueueText("ERR usage MAGCAL SET BIAS|MATRIX ...\r\n");
        return;
    }

    if (strcmp(tokens[2], "BIAS") == 0) {
        float bx;
        float by;
        float bz;

        if ((count != 6U) ||
            (app_control_parse_f32(tokens[3], &bx) == 0U) ||
            (app_control_parse_f32(tokens[4], &by) == 0U) ||
            (app_control_parse_f32(tokens[5], &bz) == 0U)) {
            APP_Control_QueueText("ERR usage MAGCAL SET BIAS <x> <y> <z>\r\n");
            return;
        }
        magcal_draft.hard_iron_bias_mgauss[0] = bx;
        magcal_draft.hard_iron_bias_mgauss[1] = by;
        magcal_draft.hard_iron_bias_mgauss[2] = bz;
        magcal_draft_bias_set = 1U;
        APP_Control_QueueText("MAGCAL state=draft_bias_set\r\n");
        return;
    }

    if (strcmp(tokens[2], "MATRIX") == 0) {
        uint32_t row;
        float c0;
        float c1;
        float c2;

        if ((count != 7U) ||
            (app_control_parse_u32(tokens[3], &row) == 0U) || (row > 2U) ||
            (app_control_parse_f32(tokens[4], &c0) == 0U) ||
            (app_control_parse_f32(tokens[5], &c1) == 0U) ||
            (app_control_parse_f32(tokens[6], &c2) == 0U)) {
            APP_Control_QueueText(
                "ERR usage MAGCAL SET MATRIX <row0-2> <c0> <c1> <c2>\r\n");
            return;
        }
        magcal_draft.soft_iron_matrix[row][0] = c0;
        magcal_draft.soft_iron_matrix[row][1] = c1;
        magcal_draft.soft_iron_matrix[row][2] = c2;
        magcal_draft_row_set[row] = 1U;
        APP_Control_QueueText("MAGCAL state=draft_matrix_row row=%lu\r\n",
                              (unsigned long)row);
        return;
    }

    APP_Control_QueueText("ERR usage MAGCAL SET BIAS|MATRIX ...\r\n");
}

static void magcal_handle_apply(void)
{
    DRV_MAG_Calibration candidate;
    DRV_MAG_CalibrationStatus status;

    if ((magcal_draft_bias_set == 0U) ||
        (magcal_draft_row_set[0] == 0U) ||
        (magcal_draft_row_set[1] == 0U) ||
        (magcal_draft_row_set[2] == 0U)) {
        APP_Control_QueueText(
            "MAGCAL state=apply_rejected reason=incomplete_draft\r\n");
        return;
    }

    /*
     * 轴向验证出处不受数值重标定影响：SET 只提供新的硬磁/软磁系数，不构成
     * 新的贴装朝向证据。calibrated 之外的字段（axis_verified、
     * frame_contract_version）原样继承当前生效值。
     */
    candidate = magcal_config;
    candidate.calibrated = 1U;
    candidate.hard_iron_bias_mgauss[0] = magcal_draft.hard_iron_bias_mgauss[0];
    candidate.hard_iron_bias_mgauss[1] = magcal_draft.hard_iron_bias_mgauss[1];
    candidate.hard_iron_bias_mgauss[2] = magcal_draft.hard_iron_bias_mgauss[2];
    memcpy(candidate.soft_iron_matrix, magcal_draft.soft_iron_matrix,
           sizeof(candidate.soft_iron_matrix));

    status = DRV_MAG_Calibration_Validate(&candidate);
    if (status != DRV_MAG_CAL_VALID) {
        APP_Control_QueueText("MAGCAL state=apply_rejected reason=%s\r\n",
                              magcal_status_text(status));
        return;
    }

    {
        uint32_t lock = BSP_Critical_Enter();
        magcal_config = candidate;
        BSP_Critical_Exit(lock);
    }
    magcal_dirty = 1U;
    magcal_clear_draft();
    APP_Control_QueueText("MAGCAL state=applied_ram\r\n");
}

static void magcal_handle_clear(void)
{
    uint32_t lock = BSP_Critical_Enter();
    magcal_defaults(&magcal_config);
    BSP_Critical_Exit(lock);
    magcal_dirty = 1U;
    magcal_clear_draft();
    APP_Control_QueueText("MAGCAL state=cleared_ram\r\n");
}

static void magcal_handle_commit(void)
{
    APP_FlashService_Status save_status;

    if (DRV_MAG_Calibration_Validate(&magcal_config) != DRV_MAG_CAL_VALID) {
        APP_Control_QueueText(
            "MAGCAL state=commit_failed reason=invalid_record\r\n");
        return;
    }
    save_status = app_control_internal_commit_config_persist();
    if (save_status != APP_FLASH_SERVICE_OK) {
        APP_Control_QueueText("MAGCAL state=commit_failed st=%d\r\n",
                              (int)save_status);
        return;
    }
    magcal_dirty = 0U;
    APP_Control_QueueText("MAGCAL state=committed\r\n");
}

static void magcal_handle_magcal(char **tokens, uint32_t count)
{
    magcal_ensure_loaded();

    if ((strcmp(tokens[0], "MAGCAL?") == 0) || (count == 1U)) {
        magcal_report_status();
        return;
    }

    if (magcal_write_allowed() == 0U) {
        APP_Control_QueueText("MAGCAL state=armed_blocked\r\n");
        return;
    }

    if (strcmp(tokens[1], "SET") == 0) {
        magcal_handle_set(tokens, count);
    } else if (strcmp(tokens[1], "APPLY") == 0) {
        magcal_handle_apply();
    } else if (strcmp(tokens[1], "COMMIT") == 0) {
        magcal_handle_commit();
    } else if (strcmp(tokens[1], "CLEAR") == 0) {
        magcal_handle_clear();
    } else {
        APP_Control_QueueText("ERR usage MAGCAL SET|APPLY|COMMIT|CLEAR\r\n");
    }
}

/* ────────────────────────────────────────────────── MAGFRAME 子命令 */

static void magcal_handle_magframe(char **tokens, uint32_t count)
{
    magcal_ensure_loaded();

    if ((strcmp(tokens[0], "MAGFRAME?") == 0) || (count == 1U)) {
        magcal_report_frame();
        return;
    }

    if (magcal_write_allowed() == 0U) {
        APP_Control_QueueText("MAGFRAME state=armed_blocked\r\n");
        return;
    }

    if ((count == 3U) && (strcmp(tokens[1], "VERIFY") == 0) &&
        (strcmp(tokens[2], "CONFIRM") == 0)) {
        uint32_t lock = BSP_Critical_Enter();
        magcal_config.axis_verified = DRV_MAG_CAL_AXIS_VERIFIED;
        magcal_config.frame_contract_version =
            (uint32_t)DRV_FRAME_CONTRACT_VERSION;
        BSP_Critical_Exit(lock);
        magcal_dirty = 1U;
        APP_Control_QueueText("MAGFRAME state=verified\r\n");
        return;
    }

    APP_Control_QueueText("ERR usage MAGFRAME VERIFY CONFIRM\r\n");
}

/* ─────────────────────────────────────────────────────────── 入口 */

uint8_t app_control_handle_magcal(char **tokens, uint32_t count)
{
    if ((tokens == NULL) || (count == 0U)) {
        return 0U;
    }
    if (APP_MagXY_HandleCommand(tokens, count) != 0U) {
        return 1U;
    }
    if ((strcmp(tokens[0], "MAGCAL") == 0) ||
        (strcmp(tokens[0], "MAGCAL?") == 0)) {
        magcal_handle_magcal(tokens, count);
        return 1U;
    }
    if ((strcmp(tokens[0], "MAGFRAME") == 0) ||
        (strcmp(tokens[0], "MAGFRAME?") == 0)) {
        magcal_handle_magframe(tokens, count);
        return 1U;
    }
    return 0U;
}

/* ───────────────────────── 与配置记录（CFG）的接口，供 config_store 存取 */

void app_cmd_magcal_apply_config(const void *config)
{
    const DRV_MAG_Calibration *loaded = (const DRV_MAG_Calibration *)config;
    uint32_t lock;

    lock = BSP_Critical_Enter();
    if ((loaded == NULL) ||
        (DRV_MAG_Calibration_Validate(loaded) != DRV_MAG_CAL_VALID)) {
        /*
         * 旧版本记录里没有 mag 块，或者存的那份过不了校验。必须显式落回
         * 出厂未校准状态，不能什么都不做——否则 RAM 里留着上一次的系数，
         * MAGCAL? 报的和 Flash 里存的不是一回事，姿态融合却已经在用旧系数。
         */
        magcal_defaults(&magcal_config);
    } else {
        magcal_config = *loaded;
    }
    BSP_Critical_Exit(lock);
    magcal_loaded = 1U;
    magcal_dirty = 0U;
}

const void *app_cmd_magcal_config(void)
{
    magcal_ensure_loaded();
    return &magcal_config;
}

/* ───────────────────────────── 供 App/Src/app_mag.c 读取的"有效"校准视图 */

void APP_MagCal_GetEffective(DRV_MAG_Calibration *out)
{
    uint32_t lock;

    if (out == NULL) {
        return;
    }
    magcal_ensure_loaded();
    lock = BSP_Critical_Enter();
    *out = magcal_config;
    BSP_Critical_Exit(lock);
    if (out->frame_contract_version != (uint32_t)DRV_FRAME_CONTRACT_VERSION) {
        /* C6：轴向一变，校准系数（这里具体是"轴向已验证"这个前提）立即失效。 */
        out->axis_verified = DRV_MAG_CAL_AXIS_UNVERIFIED;
    }
}
