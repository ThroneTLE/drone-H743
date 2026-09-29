/*
 * PROPCAL 命令族：桨叶与电机接线标定，外加一个受心跳保护的"点电机"窗口。
 *
 * 状态机与 `app_cmd_ledmap.c` 同构（SET 改 RAM 并立刻发布，COMMIT 才落 Flash），
 * 理由也一样：落盘是同步擦一个 128 KB 片内扇区，跑在 UARTTask 上忙等，
 * 边改边存就是几十次秒级卡顿。
 *
 * 与 LEDMAP 不同的是**两条额外的规矩**：
 *
 * 1. 这份配置决定偏航极性和 ESC 通道归属，半份是危险的。所以草稿要两路都
 *    declare 过、且整份自洽，才会 PublishActive；只要还没凑齐或者互相矛盾，
 *    就发布一份"未标定"上去——飞控因此拒绝解锁，而不是带着半份映射待命。
 *
 * 2. `PROPCAL SPIN` 能让电机转。它是全仓库唯一一条能从上位机让桨转起来的路径，
 *    所以它要求显式确认词、只在未解锁时可用、一次只转一路、油门有固件侧上限，
 *    而且**必须靠心跳续命**——超时判定跑在 500 Hz 控制环里（`app_prop_spin.c`）。
 *    `APP_CONTROL_ALLOW_RAW_MOTOR_COMMANDS` 仍然是 0，本模块没有放宽它。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_acceptance.h"
#include "app_flash_service.h"
#include "app_ident.h"
#include "app_prop_spin.h"
#include "app_thrust_bench.h"
#include "app_servo_cal.h"
#include "app_stabilizer.h"
#include "bsp_dshot.h"
#include "bsp_dshot_rx.h"
#include "bsp_pwm.h"
#include "drv_prop_map.h"
#include "svc_timestamp.h"

#include <stdio.h>
#include <stddef.h>
#include <string.h>

/* 开窗必须带上这个词。`PROPCAL SPIN` 手滑敲一半不会让电机转起来。 */
#define PROPCAL_SPIN_CONFIRM_TOKEN "safe"

/*
 * 电调回传数据的新鲜度门。
 *
 * 双向 DShot 下两路轮流采，500 Hz 提交对应每路 250 Hz，所以 500 ms 是连续丢掉
 * 一百多帧——那不是抖动，是这一路已经不说话了。EDT 是**插在转速帧之间**偶尔发
 * 一次的，频率低得多，所以电流用 1000 ms，和台架那边（app_thrust_bench.h）取同
 * 一个数量级。
 *
 * 过期一律报 `-` 而不是报最后一个值：这一行是给人盯着看转速变化的，一个僵住不
 * 动的数字比一个"没有"更容易被当成真的。
 */
#define PROPCAL_ESC_ERPM_FRESH_MS     500U
/*
 * EDT 电流的新鲜度窗口。2026-09-21 实机实测：链路通了之后转速帧以约 500 Hz
 * 稳定到达，而 **EDT 帧是低速插在转速帧之间**的——按 1000 ms 判过期时，电流
 * 字段有一多半时间显示 `-`，看起来像"时有时无的故障"，实际是协议本身的节奏。
 *
 * 放宽到 4 秒是按实测节奏定的，不是拍脑袋。代价是显示的电流可能是几秒前的，
 * 所以**真实年龄一并报出来**（escdiag 的 edt_age*），让读数的人自己判断够不够
 * 新——只放宽窗口而不报年龄，就是把陈旧值伪装成实时值。
 *
 * 这条也定义了这条链路的能力边界：EDT 电流适合台架上"稳住一个工作点数秒"的
 * 用法，不适合看瞬态。
 */
#define PROPCAL_ESC_CURRENT_FRESH_MS 4000U

static DRV_PropMap propcal_draft;
static uint8_t propcal_declared[DRV_PROP_ESC_CHANNEL_COUNT];
static uint8_t propcal_loaded;
static uint8_t propcal_dirty;

/* ──────────────────────────────────────────── 草稿 → 运行期 */

static void propcal_publish_uncalibrated(void)
{
    DRV_PropMap blank;

    DRV_PropMap_Defaults(&blank);
    (void)DRV_PropMap_PublishActive(&blank);
}

static uint8_t propcal_draft_complete(void)
{
    uint32_t i;

    for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
        if (propcal_declared[i] == 0U) {
            return 0U;
        }
    }
    return 1U;
}

/*
 * 把草稿推到运行期。返回的字符串就是回包里的 state=，三种结局：
 *   applied_ram —— 两路都填了且自洽，已经生效（未落盘）
 *   partial     —— 还没填全；运行期置为未标定，解锁被挡住
 *   conflict    —— 填全了但物理上不可能（两路都说自己是上桨 / 两路同向）；
 *                  同样置为未标定，并把草稿留着让人看见矛盾在哪
 */
static const char *propcal_publish_draft(void)
{
    DRV_PropMap candidate;

    if (propcal_draft_complete() == 0U) {
        propcal_draft.calibrated = 0U;
        propcal_publish_uncalibrated();
        return "partial";
    }

    candidate = propcal_draft;
    candidate.calibrated = 1U;
    if (DRV_PropMap_Validate(&candidate) == 0U) {
        propcal_draft.calibrated = 0U;
        propcal_publish_uncalibrated();
        return "conflict";
    }

    propcal_draft = candidate;
    (void)DRV_PropMap_PublishActive(&propcal_draft);
    return "applied_ram";
}

static void propcal_ensure_loaded(void)
{
    if (propcal_loaded == 0U) {
        DRV_PropMap_Defaults(&propcal_draft);
        memset(propcal_declared, 0, sizeof(propcal_declared));
        propcal_loaded = 1U;
        /*
         * 点电机窗口在这里归零，而不是在 app_control 的初始化里：那个文件是
         * 硬约束的"只减不增"。静态区本来就是全零（= 窗口关着、两路油门 0），
         * 这一句是把"安全初值靠的是 C 的静态初始化规则"变成明写出来的事实——
         * 靠巧合成立的安全属性，下一个人改动时不会注意到它。
         */
        APP_PropSpin_Reset();
        propcal_publish_uncalibrated();
    }
}

/* ──────────────────────────────────────────── 回包 */

static const char *propcal_yaw_polarity_text(void)
{
    const float polarity = DRV_PropMap_YawTorquePolarity();

    if (polarity > 0.5f) {
        return "+1";
    }
    if (polarity < -0.5f) {
        return "-1";
    }
    /* 未标定。这里报 0 而不是 +1：极性不是"默认正"，是**还没有方向**。 */
    return "0";
}

static void propcal_report_channel(uint32_t index)
{
    const DRV_PropChannel *channel = &propcal_draft.channel[index];

    APP_Control_QueueText(
        "PROPCAL ch=%u pad=%s role=%s spin=%s declared=%u\r\n",
        (unsigned int)(index + 1U),
        BSP_PWM_EscChannelLabel(index + 1U),
        DRV_PropMap_RoleName(channel->role),
        DRV_PropMap_SpinName(channel->spin_sense),
        (unsigned int)propcal_declared[index]);
}

/*
 * 把一个"可能没有"的整数写成字段值：有就是十进制，没有就是 `-`。
 *
 * 用 `-` 而不是 0 / -1 之类的哨兵，是因为这几路的合法值里**本来就包含 0**
 * （电调回报"未旋转"时 eRPM 就是 0，小电流下 EDT 电流也是 0）。拿一个合法值当
 * "没有"，上位机就再也分不清"电调说它停着"和"根本没收到回传"。
 */
static void propcal_format_optional(char *out, size_t size, uint8_t present,
                                    uint32_t value)
{
    if (present == 0U) {
        (void)snprintf(out, size, "-");
        return;
    }
    (void)snprintf(out, size, "%lu", (unsigned long)value);
}

/*
 * 心跳回包顺带捎上电调回传。
 *
 * 为什么挂在这一行上：这条命令链路每 100 ms 必定往返一次（心跳本身是安全机制，
 * 不是为了取数才有的），所以在这里多报几个字段**不增加任何一次往返**，只多几十
 * 个字节。单独给电调转速/电流开一路轮询或者加遥测通道，都是在为已经存在的往返
 * 重复付一遍钱。
 *
 * 总电压/总电流不在这里：它们已经是遥测通道 `batt_v` / `batt_i`，由上位机按可见
 * 性订阅推送，40 Hz 比这里的 10 Hz 更适合看采样噪声。
 */
static void propcal_report_spin(void)
{
    APP_PropSpinOutput spin;
    BSP_DShotRxSnapshot rx;
    const uint32_t now_ms = SVC_Timestamp_Ms();
    char current_text[2][12];
    char erpm_text[2][12];
    uint8_t telem_available;
    uint32_t i;

    APP_PropSpin_GetOutput(&spin);
    BSP_DShotRx_GetSnapshot(&rx);
    telem_available = (rx.available != 0U) ? 1U : 0U;

    for (i = 0U; i < 2U; ++i) {
        /* 无符号差值，跨 49 天回绕安全；和台架那边取数的写法一致。 */
        const uint32_t erpm_age = (uint32_t)(now_ms - rx.sample_ms[i]);
        const uint32_t current_age = (uint32_t)(now_ms - rx.current_sample_ms[i]);
        const uint8_t erpm_ok = (uint8_t)((telem_available != 0U) &&
                                          (rx.valid[i] != 0U) &&
                                          (erpm_age <= PROPCAL_ESC_ERPM_FRESH_MS));
        const uint8_t current_ok = (uint8_t)((telem_available != 0U) &&
                                             (rx.current_valid[i] != 0U) &&
                                             (current_age <= PROPCAL_ESC_CURRENT_FRESH_MS));
        /* 电调明确回报"未旋转"是**有效回包**，报 0；这和解码失败是两回事。 */
        propcal_format_optional(erpm_text[i], sizeof(erpm_text[i]), erpm_ok,
                                (rx.not_spinning[i] != 0U) ? 0UL : rx.erpm[i]);
        propcal_format_optional(current_text[i], sizeof(current_text[i]),
                                current_ok, (uint32_t)rx.current_a[i]);
    }

    APP_Control_QueueText(
        "PROPCAL spin=%s ch=%u pct=%u age_ms=%lu max_pct=%u timeout_ms=%u "
        "stop=%s esc_telem=%u esc_i1=%s esc_i2=%s esc_erpm1=%s esc_erpm2=%s\r\n",
        (spin.active != 0U) ? "active" : "idle",
        (unsigned int)spin.channel,
        (unsigned int)((spin.channel != 0U) ? spin.percent[spin.channel - 1U] : 0U),
        (unsigned long)APP_PropSpin_HeartbeatAgeMs(now_ms),
        (unsigned int)spin.max_percent,
        (unsigned int)APP_PROP_SPIN_TIMEOUT_MS,
        APP_PropSpin_StopReasonName(APP_PropSpin_LastStopReason()),
        (unsigned int)telem_available,
        current_text[0], current_text[1], erpm_text[0], erpm_text[1]);
}

/*
 * 电调回传的**解码计数**，与上面那行分开发。
 *
 * 为什么必须有：双向 DShot 这条链 2026-09-21 才在 bitbang 后端实机打通（TIMER 后端
 * 实机失败，见 doc/esc-output.md「已验证 / 未验证」表）。换后端或换电调固件后一旦回不来
 * 数，上位机只会显示 `—`，而"根本没收到"和"收到了解不开"要查的是完全不同的东西：
 *   - to(timeouts) 在涨、fr/crc 不动 → 线上压根没有回传：电调多半还在单向模式；
 *   - crc 在涨       → 有回传但解不开：极性/校验/GCR 表/位宽有一个不对；
 *   - fr 在涨        → 链路通，剩下的是数值对不对。
 * 三种指向三个完全不同的修法，靠"显示不出来"这一个现象是分不开的。
 *
 * 不挂在心跳那一行上：计数是慢变量，不需要 10 Hz，而那一行已经接近
 * APP_UART_TX_TEXT_SIZE 的一半，再塞六个 32 位计数会把它顶到溢出边上。
 * 因此只在开窗 / 停止 / 裸查询时发一次——正好够"跑一轮前后对比"。
 *
 * `proto=` 报的是本次构建的档位：烧完之后第一件事就是确认烧进去的是不是双向档，
 * 而这件事没有别的地方能看。
 */
/*
 * 在一整个帧周期上统计两路信号脚的高电平占比。
 *
 * 单次采样没用：帧本身只占约 2.6%（DShot300 一帧约 53 us，周期 2 ms），一次读到
 * 高既可能是空闲、也可能是数据位之间。要回答"空闲电平到底是高还是低"，必须在
 * **至少一个完整周期**上看占比——空闲高时应当压倒性是高，空闲低时压倒性是低。
 *
 * 用时间戳卡窗口而不是数循环次数：循环次数随优化等级和总线争用飘，而这条结论
 * 是要拿去判断硬件的，不能建立在一个会飘的采样跨度上。
 */
#define PROPCAL_PIN_SAMPLE_MS 4U

static void propcal_sample_esc_pins(uint32_t *out_high1, uint32_t *out_high2,
                                    uint32_t *out_total)
{
    const uint32_t start = SVC_Timestamp_Ms();
    uint32_t high1 = 0U, high2 = 0U, total = 0U;

    while ((uint32_t)(SVC_Timestamp_Ms() - start) < PROPCAL_PIN_SAMPLE_MS) {
        const uint8_t levels = BSP_DShot_ReadEscPinLevels();
        if ((levels & 1U) != 0U) { high1++; }
        if ((levels & 2U) != 0U) { high2++; }
        total++;
    }
    *out_high1 = high1; *out_high2 = high2; *out_total = total;
}

static void propcal_report_esc_diag(void)
{
    BSP_DShotRxSnapshot rx;
    uint32_t pin_high1 = 0U, pin_high2 = 0U, pin_total = 0U;
    char edt_age_text[2][12];
    uint32_t now_ms;

    BSP_DShotRx_GetSnapshot(&rx);
    now_ms = SVC_Timestamp_Ms();
    propcal_sample_esc_pins(&pin_high1, &pin_high2, &pin_total);

    /*
     * EDT 电流的**真实年龄**，单位 ms。放宽新鲜度窗口而不报年龄，等于把几秒前的
     * 读数伪装成实时值；报了年龄，读数的人才能自己判断够不够新。
     * 从未收到过就报 `-`——0 是"刚刚收到"，拿它表示"从来没有"会正好反过来。
     */
    for (uint32_t i = 0U; i < 2U; ++i) {
        propcal_format_optional(edt_age_text[i], sizeof(edt_age_text[i]),
                                rx.current_valid[i],
                                (uint32_t)(now_ms - rx.current_sample_ms[i]));
    }
    APP_Control_QueueText(
        "PROPCAL escdiag avail=%u proto=%u grace=%u fr1=%lu crc1=%lu to1=%lu "
        "fr2=%lu crc2=%lu to2=%lu pin1=%lu/%lu pin2=%lu/%lu "
        "edt_age1=%s edt_age2=%s\r\n",
        (unsigned int)((rx.available != 0U) ? 1U : 0U),
        (unsigned int)BSP_ESC_PROTOCOL,
        (unsigned int)rx.detect_grace,
        (unsigned long)rx.frames[0], (unsigned long)rx.crc_errors[0],
        (unsigned long)rx.timeouts[0],
        (unsigned long)rx.frames[1], (unsigned long)rx.crc_errors[1],
        (unsigned long)rx.timeouts[1],
        (unsigned long)pin_high1, (unsigned long)pin_total,
        (unsigned long)pin_high2, (unsigned long)pin_total,
        edt_age_text[0], edt_age_text[1]);
}

static void propcal_report(const char *state, const char *reason)
{
    const uint8_t upper_ch = DRV_PropMap_EscChannelForRole(
        (uint8_t)DRV_PROP_ROLE_UPPER);
    const uint8_t lower_ch = DRV_PropMap_EscChannelForRole(
        (uint8_t)DRV_PROP_ROLE_LOWER);
    float lower_spin = DRV_PropMap_LowerSpinSense();
    int8_t lower_spin_code = DRV_PROP_SPIN_NONE;

    if (lower_spin > 0.5f) {
        lower_spin_code = DRV_PROP_SPIN_CCW;
    } else if (lower_spin < -0.5f) {
        lower_spin_code = DRV_PROP_SPIN_CW;
    } else {
        lower_spin_code = DRV_PROP_SPIN_NONE;
    }

    APP_Control_QueueText(
        "PROPCAL state=%s calibrated=%u gen=%lu dirty=%u upper_ch=%u "
        "lower_ch=%u lower_spin=%s yaw_polarity=%s reason=%s\r\n",
        state,
        (unsigned int)DRV_PropMap_IsCalibrated(),
        (unsigned long)DRV_PropMap_GetActiveGeneration(),
        (unsigned int)propcal_dirty,
        (unsigned int)upper_ch,
        (unsigned int)lower_ch,
        DRV_PropMap_SpinName(lower_spin_code),
        propcal_yaw_polarity_text(),
        (reason != NULL) ? reason : "-");
}

static void propcal_report_all(const char *state, const char *reason)
{
    uint32_t i;

    propcal_report(state, reason);
    for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
        propcal_report_channel(i);
    }
    propcal_report_spin();
}

/* ──────────────────────────────────────────── 安全门 */

/*
 * 改映射 = 改偏航极性和通道归属。解锁状态下改一次，桨正在转的飞机会在下一拍
 * 换一套分配式——和 RCMAP / LEDMAP 的解锁门同源。点电机窗口开着时同样不许改：
 * 正在转的那一路的含义不能在人眼盯着它的时候被改掉。
 */
static uint8_t propcal_write_allowed(void)
{
    return ((APP_Stabilizer_IsArmed() == 0U) &&
            (APP_PropSpin_IsActive() == 0U) &&
            (APP_ThrustBench_IsActive() == 0U)) ? 1U : 0U;
}

/* 点电机的前置条件。任何一条不满足都不开窗，且回包说明是哪一条。 */
static const char *propcal_spin_block_reason(void)
{
    if (APP_Stabilizer_IsArmed() != 0U) {
        return "armed";
    }
    if (APP_Acceptance_IsActive() != 0U) {
        return "acceptance_active";
    }
    if (APP_ServoCal_IsActive() != 0U) {
        return "servo_cal_active";
    }
    if (APP_Ident_IsRunning() != 0U) {
        return "ident_running";
    }
    if (APP_ThrustBench_IsActive() != 0U) {
        return "thrust_bench_active";
    }
    return NULL;
}

/* ──────────────────────────────────────────── 子命令 */

static void propcal_handle_set(char **tokens, uint32_t count)
{
    const char *ch_text = app_control_token_value(tokens, count, "ch");
    const char *role_text = app_control_token_value(tokens, count, "role");
    const char *spin_text = app_control_token_value(tokens, count, "spin");
    uint32_t channel_number;
    uint32_t index;
    uint8_t role;
    int8_t spin;

    if ((ch_text == NULL) ||
        (app_control_parse_u32(ch_text, &channel_number) == 0U) ||
        (channel_number == 0U) ||
        (channel_number > DRV_PROP_ESC_CHANNEL_COUNT)) {
        propcal_report("rejected", "bad_channel");
        return;
    }
    if ((role_text == NULL) && (spin_text == NULL)) {
        propcal_report("invalid_usage", "need_role_or_spin");
        return;
    }
    index = channel_number - 1U;

    if (role_text != NULL) {
        if (DRV_PropMap_RoleFromName(role_text, &role) == 0U) {
            propcal_report("rejected", "bad_role");
            return;
        }
        propcal_draft.channel[index].role = role;
    }
    if (spin_text != NULL) {
        if (DRV_PropMap_SpinFromName(spin_text, &spin) == 0U) {
            propcal_report("rejected", "bad_spin");
            return;
        }
        propcal_draft.channel[index].spin_sense = spin;
    }

    /*
     * declared 只有在这一路的旋向真的填过之后才置起来。角色单独填不算数：
     * 没有旋向就推不出偏航极性，而一个"填了一半却显示已标定"的界面，
     * 正是这次改造要消灭的东西。
     */
    propcal_declared[index] =
        (propcal_draft.channel[index].spin_sense != DRV_PROP_SPIN_NONE) ? 1U : 0U;
    propcal_dirty = 1U;

    propcal_report_all(propcal_publish_draft(), NULL);
}

static void propcal_handle_reset(void)
{
    DRV_PropMap_Defaults(&propcal_draft);
    memset(propcal_declared, 0, sizeof(propcal_declared));
    propcal_dirty = 1U;
    propcal_publish_uncalibrated();
    propcal_report_all("reset_ram", NULL);
}

static void propcal_handle_commit(void)
{
    uint8_t save_status;

    if (propcal_draft_complete() == 0U) {
        propcal_report("commit_rejected", "incomplete");
        return;
    }
    if (DRV_PropMap_IsCalibrated() == 0U) {
        /* 草稿填全了但运行期仍是未标定 = publish 被 Validate 拒了。
         * 不把一份连自己都不敢用的映射写进 Flash。 */
        propcal_report("commit_rejected", "conflict");
        return;
    }
    save_status = app_control_internal_commit_config_persist();
    if (save_status != (uint8_t)APP_FLASH_SERVICE_OK) {
        APP_Control_QueueText("PROPCAL state=commit_failed st=%u\r\n",
                              (unsigned int)save_status);
        return;
    }
    propcal_dirty = 0U;
    propcal_report_all("committed", NULL);
}

static void propcal_handle_spin(char **tokens, uint32_t count)
{
    const uint32_t now_ms = SVC_Timestamp_Ms();
    const char *blocked;

    if (count < 3U) {
        propcal_report_spin();
        propcal_report_esc_diag();
        return;
    }

    if (strcmp(tokens[2], "STOP") == 0) {
        APP_PropSpin_Close(now_ms, (uint8_t)APP_PROP_SPIN_STOP_REQUEST);
        propcal_report_spin();
        propcal_report_esc_diag();
        return;
    }

    if (strcmp(tokens[2], "ARM") == 0) {
        const char *confirm = app_control_token_value(tokens, count, "confirm");
        const char *max_text = app_control_token_value(tokens, count, "max_pct");
        uint32_t max_percent = (uint32_t)APP_PROP_SPIN_DEFAULT_MAX_PERCENT;

        if ((confirm == NULL) ||
            (strcmp(confirm, PROPCAL_SPIN_CONFIRM_TOKEN) != 0)) {
            APP_Control_QueueText(
                "PROPCAL state=spin_rejected reason=need_confirm "
                "usage=PROPCAL SPIN ARM confirm=" PROPCAL_SPIN_CONFIRM_TOKEN
                " [max_pct=1..100]\r\n");
            return;
        }
        /*
         * 不写 `max_pct` 就是 20%——看旋向的那条老路径一个字都不用改，保护也一点
         * 没少。写了才可能更高，而且这个上限跟着窗口走，停一次就回到 20%。
         */
        if (max_text != NULL) {
            if ((app_control_parse_u32(max_text, &max_percent) == 0U) ||
                (max_percent == 0U) ||
                (max_percent > (uint32_t)APP_PROP_SPIN_HARD_MAX_PERCENT)) {
                APP_Control_QueueText(
                    "PROPCAL state=spin_rejected reason=bad_max_pct\r\n");
                return;
            }
        }
        blocked = propcal_spin_block_reason();
        if (blocked != NULL) {
            APP_Control_QueueText("PROPCAL state=spin_rejected reason=%s\r\n",
                                  blocked);
            return;
        }
        if (APP_PropSpin_Open(now_ms, (uint8_t)max_percent) == 0U) {
            APP_Control_QueueText(
                "PROPCAL state=spin_rejected reason=bad_max_pct\r\n");
            return;
        }
        propcal_report_spin();
        propcal_report_esc_diag();
        return;
    }

    if (strcmp(tokens[2], "SET") == 0) {
        const char *ch_text = app_control_token_value(tokens, count, "ch");
        const char *pct_text = app_control_token_value(tokens, count, "pct");
        uint32_t channel_number;
        uint32_t percent;

        if (APP_PropSpin_IsActive() == 0U) {
            APP_Control_QueueText(
                "PROPCAL state=spin_rejected reason=not_open\r\n");
            return;
        }
        if ((ch_text == NULL) || (pct_text == NULL) ||
            (app_control_parse_u32(ch_text, &channel_number) == 0U) ||
            (app_control_parse_u32(pct_text, &percent) == 0U) ||
            (channel_number > 255U) || (percent > 255U)) {
            /* 解析不出来的命令不能当成心跳。窗口按"乱发命令"关掉，
             * 理由见 app_prop_spin.c 的 Command()。 */
            APP_PropSpin_Close(now_ms, (uint8_t)APP_PROP_SPIN_STOP_REJECTED);
            APP_Control_QueueText(
                "PROPCAL state=spin_rejected reason=bad_number\r\n");
            return;
        }
        blocked = propcal_spin_block_reason();
        if (blocked != NULL) {
            APP_PropSpin_Close(now_ms, (uint8_t)APP_PROP_SPIN_STOP_INHIBIT);
            APP_Control_QueueText("PROPCAL state=spin_rejected reason=%s\r\n",
                                  blocked);
            return;
        }
        (void)APP_PropSpin_Command(now_ms, (uint8_t)channel_number,
                                   (uint8_t)percent);
        propcal_report_spin();
        return;
    }

    APP_Control_QueueText(
        "PROPCAL state=invalid_usage reason=PROPCAL SPIN ARM|SET|STOP\r\n");
}

/* ──────────────────────────────────────────── 入口 */

uint8_t app_control_handle_propcal(char **tokens, uint32_t count)
{
    if ((tokens == NULL) || (count == 0U)) {
        return 0U;
    }
    if ((strcmp(tokens[0], "PROPCAL?") != 0) &&
        (strcmp(tokens[0], "PROPCAL") != 0)) {
        return 0U;
    }
    propcal_ensure_loaded();

    if ((strcmp(tokens[0], "PROPCAL?") == 0) || (count == 1U)) {
        propcal_report_all("status", NULL);
        return 1U;
    }

    if (strcmp(tokens[1], "SPIN") == 0) {
        propcal_handle_spin(tokens, count);
        return 1U;
    }

    if (propcal_write_allowed() == 0U) {
        propcal_report("write_blocked",
                       (APP_Stabilizer_IsArmed() != 0U) ? "armed" : "spin_open");
        return 1U;
    }

    if (strcmp(tokens[1], "SET") == 0) {
        propcal_handle_set(tokens, count);
    } else if (strcmp(tokens[1], "RESET") == 0) {
        propcal_handle_reset();
    } else if (strcmp(tokens[1], "COMMIT") == 0) {
        propcal_handle_commit();
    } else {
        propcal_report("invalid_usage", "SET|RESET|COMMIT|SPIN");
    }
    return 1U;
}

/* ──────────────────── 与配置记录（CFG）的接口 */

void app_cmd_propcal_apply_config(const void *config)
{
    const DRV_PropMap *loaded = (const DRV_PropMap *)config;
    uint32_t i;

    if ((loaded == NULL) || (DRV_PropMap_Validate(loaded) == 0U)) {
        /*
         * 旧版本记录里没有这一块，或存的那份过不了校验。
         *
         * **绝不从 airframe.lower_rotor_spin_sense 迁移过来。** 那个值是从调参
         * 现象反推的，把它搬进来会让一份从没量过的旋向顶着"已标定"的名义生效，
         * 而这次改造的全部意义就是不再让那种值决定偏航方向。宁可退回未标定、
         * 挡住解锁，逼一次真的通电确认。
         */
        DRV_PropMap_Defaults(&propcal_draft);
        memset(propcal_declared, 0, sizeof(propcal_declared));
    } else {
        propcal_draft = *loaded;
        for (i = 0U; i < DRV_PROP_ESC_CHANNEL_COUNT; i++) {
            propcal_declared[i] = propcal_draft.calibrated;
        }
    }
    propcal_loaded = 1U;
    propcal_dirty = 0U;
    /* 重新装载配置 = 映射的含义可能整个换了。正在转的那一路必须先停：
     * 窗口开着时换一套通道归属，操作者眼睛盯着的电机和界面说的会对不上。 */
    APP_PropSpin_Reset();
    (void)propcal_publish_draft();
}

const void *app_cmd_propcal_config(void)
{
    propcal_ensure_loaded();
    return &propcal_draft;
}
