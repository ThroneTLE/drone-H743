#include "app_telem_stream.h"

#include "app_telemetry.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*
 * 流开关。历史名保留（见头文件说明），定义搬到这里之后 freertos.c 的
 * USER CODE 段少了一个全局变量，所有权也回到了真正使用它的模块。
 */
volatile uint8_t vofaStreamActive = 0U;

/*
 * 上电默认掩码里的实时通道。选择依据是带宽，不是口味：
 * 数传 57600 baud = 5760 B/s，类别模式规定任何默认配置不得超过它的 60%
 * （3456 B/s）。
 *   稳态帧 = 12 路实时通道 -> 9 + 24 + 48 = 81 B，× 40 Hz = 3240 B/s（56.3%）。
 *   参数通道也在默认掩码里，但稳态不置位（只有值变了才带上），所以不占稳态带宽；
 *   它们在场是为了让 1 Hz 的全量刷新帧能把 13 个兼容滑块一次喂给上位机。
 *   刷新帧 = 25 路 -> 133 B，**替换**当拍的稳态帧而不是额外加一帧，
 *   合计 39×81 + 133 = 3292 B/s（57.2%），仍在 60% 以内。
 * 两个累计计数类 fusion 通道（corrections / norm_rej）默认不发：40 Hz 重复推
 * 一个单调计数器信息量极低，需要时用 `TELEM MASK` 单独打开。
 */
static const uint8_t app_telem_stream_default_channels[] = {
    (uint8_t)APP_TELEM_CH_ROLL,
    (uint8_t)APP_TELEM_CH_PITCH,
    (uint8_t)APP_TELEM_CH_YAW,
    (uint8_t)APP_TELEM_CH_FLOW_HEIGHT,
    (uint8_t)APP_TELEM_CH_TIME,
    (uint8_t)APP_TELEM_CH_VEL_EST_X,
    (uint8_t)APP_TELEM_CH_VEL_EST_Y,
    (uint8_t)APP_TELEM_CH_POS_EST_X,
    (uint8_t)APP_TELEM_CH_POS_EST_Y,
    (uint8_t)APP_TELEM_CH_FUSION_ACC_ERR,
    (uint8_t)APP_TELEM_CH_FUSION_ACC_IGNORED,
    (uint8_t)APP_TELEM_CH_FUSION_ACC_RECOVERY,
};

typedef struct {
    uint32_t        rate_hz;
    APP_TelemMask   mask;
    uint32_t        refresh_s;
    APP_TelemFormat format;
    APP_TelemSink   sink;
    APP_TelemSink   auto_sink;          /* STREAM on 那一刻锁定的来源链路 */
    APP_TelemSink   last_command_sink;  /* 最近一条命令是从哪条链路进来的 */
    uint16_t        seq;
    uint32_t        frames;
    uint32_t        drops;
    uint32_t        usb_lost;
    uint32_t        refresh_accum_ms;
    APP_TelemMask   dirty;
    uint32_t        shadow[APP_TELEM_CH_COUNT];
    uint8_t         shadow_valid;
    uint8_t         encode_error_latched;
    uint8_t         initialised;
    /* 定时基准：下一拍**应该**在什么时刻发，而不是"睡完这段再说"。 */
    uint32_t        pace_deadline_us;
    uint8_t         pace_valid;
} APP_TelemStreamState;

static APP_TelemStreamState app_telem_stream;

/*
 * 每拍耗时画像。**固定开着**，不做"诊断模式"开关。
 *
 * 这条流上已经栽过的两个跟头（遥测任务优先级太低一拍没跑过、周期把干活时间也
 * 算了进去）都不是读代码能读出来的，全靠量。而要是诊断得先切模式才准，真出问题
 * 那一刻常常已经来不及切了。所以一直记，`TELEM PROF` 只负责把它读出来。
 * 代价是每拍多读 5 次时间戳——40 Hz 下完全可以忽略。
 *
 * 累加用 64 位：25 ms 一拍的话 32 位和只够 71 分钟，一次长测就溢出了；
 * 打印前先在 C 里除成平均值，所以不需要 newlib-nano 的 %llu。
 */
typedef struct {
    uint32_t ticks;
    uint32_t sent;
    uint32_t skipped;
    uint32_t late;           /* pace 进来时截止时刻已经过了 */
    uint32_t entry_us;
    uint8_t  entry_valid;
    uint32_t period_last_us;
    uint32_t period_min_us;
    uint32_t period_max_us;
    uint64_t period_sum_us;
    uint32_t sleep_max_us;
    uint64_t sleep_sum_us;
    uint32_t sample_max_us;
    uint64_t sample_sum_us;
    uint32_t encode_max_us;
    uint64_t encode_sum_us;
    uint32_t send_max_us;
    uint64_t send_sum_us;
    /* 本拍的分段耗时，由 tick 主体逐段写入，收尾时一次性并入统计。 */
    uint32_t stage_sleep_us;
    uint32_t stage_sample_us;
    uint32_t stage_encode_us;
    uint32_t stage_send_us;
} APP_TelemStreamProfile;

static APP_TelemStreamProfile app_telem_prof;

/*
 * 帧缓冲与取值数组都放静态区：遥测任务栈只有 2 KB，265 B 的成帧缓冲加两个
 * 通道数组压栈会把余量吃掉一半以上。只有遥测任务一个写者。
 */
static uint8_t app_telem_stream_frame[APP_TELEM_FRAME_OVERHEAD +
                                      APP_TELEM_FRAME_MAX_PAYLOAD];
static float   app_telem_stream_values[APP_TELEM_CH_COUNT];
static float   app_telem_stream_packed[APP_TELEM_CH_COUNT];

/* ------------------------------------------------------------------ */
/* 内部工具                                                            */
/* ------------------------------------------------------------------ */

static APP_TelemMask telem_stream_all_mask(void)
{
    APP_TelemMask mask = APP_TelemMask_Zero();
    uint32_t index;

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        mask = APP_TelemMask_Or(mask, APP_TelemMask_FromBit(index));
    }

    return mask;
}

/* 参数回显通道的位集合。这些通道按变化发，不进稳态帧。 */
static APP_TelemMask telem_stream_param_mask(void)
{
    APP_TelemMask mask = APP_TelemMask_Zero();
    uint32_t index;

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        if (APP_Telemetry_ChannelHasParam(index) != 0U) {
            mask = APP_TelemMask_Or(mask, APP_TelemMask_FromBit(index));
        }
    }

    return mask;
}

static uint32_t telem_stream_period_ms(void)
{
    uint32_t hz = app_telem_stream.rate_hz;

    if (hz == 0U) {
        hz = APP_TELEM_RATE_HZ;
    }

    return (1000U / hz) > 0U ? (1000U / hz) : 1U;
}

/*
 * 按**绝对时刻**对齐每一拍，而不是"干完活再睡一个周期"。
 *
 * 原来是 `PortDelayMs(period)` 打头、后面才采样/编码/发送，于是实际周期 =
 * 周期 + 干活时间。USB 上干活只有几十微秒，看不出来；蓝牙上那次发送是阻塞的
 * （84 字节 / 115200 ≈ 7.3 ms），40 Hz 设下去实测只跑出 28.3 Hz——而带宽才用了
 * 2.4 kB/s，离 115200 的上限远得很。也就是说卡的从来不是链路，是这行代码。
 *
 * 改成按截止时刻对齐之后，发送时间落在本来就要睡的那一段里，不再往周期上加。
 *
 * 基准是从**上一个截止时刻**递推的，不是从"现在"，所以每拍那点取整误差不会
 * 累积成越走越慢。
 */
static void telem_stream_prof_accum(uint32_t *peak, uint64_t *sum, uint32_t value)
{
    *sum += (uint64_t)value;
    if (value > *peak) {
        *peak = value;
    }
}

/* 两个 32 位微秒时刻之差。约 71 分钟回绕，无符号减法自然成立。 */
static uint32_t telem_stream_elapsed_us(uint32_t from_us, uint32_t to_us)
{
    return to_us - from_us;
}

static void telem_stream_pace(void)
{
    const uint32_t period_us = telem_stream_period_ms() * 1000U;
    const uint32_t now_us    = APP_TelemStream_PortNowUs();
    int32_t  remaining_us;
    uint32_t sleep_ms;

    if (app_telem_stream.pace_valid == 0U) {
        app_telem_stream.pace_deadline_us = now_us;
        app_telem_stream.pace_valid = 1U;
    }

    app_telem_stream.pace_deadline_us += period_us;
    /* 有符号差值：PortNowUs 是 32 位微秒，约 71 分钟回绕，这样写回绕也成立。 */
    remaining_us = (int32_t)(app_telem_stream.pace_deadline_us - now_us);

    if (remaining_us <= 0) {
        /*
         * 上一拍超时了（链路堵、或者刚发过一个全量刷新大帧）。**不补发**：
         * 连发几帧去追进度会在链路上挤成一团，看到的波形反而更抖。
         * 直接把基准挪到现在重新起算，宁可少一帧也不要一串挤在一起。
         */
        app_telem_stream.pace_deadline_us = now_us + period_us;
        remaining_us = (int32_t)period_us;
        app_telem_prof.late++;
    }

    /* 四舍五入到毫秒；睡 0 毫秒在有些 RTOS 上不让出 CPU，所以下限是 1。 */
    sleep_ms = ((uint32_t)remaining_us + 500U) / 1000U;
    if (sleep_ms == 0U) {
        sleep_ms = 1U;
    }
    /*
     * 一拍绝不会需要睡超过一个周期。真算出更大的值，只可能是时间基准出了岔子
     * （回绕、时钟毛刺），那时宁可多发一帧也不要整条流停在一个超长的 osDelay 里
     * ——那是"看起来 stream=1、实际一个字节都不来"的安静失败。
     */
    if (sleep_ms > telem_stream_period_ms()) {
        sleep_ms = telem_stream_period_ms();
        app_telem_stream.pace_deadline_us = now_us + period_us;
    }
    APP_TelemStream_PortDelayMs(sleep_ms);
    /*
     * 记的是**实际**睡了多久，不是要求睡多久：osDelay(n) 在 1 kHz tick 上本来
     * 就落在 (n-1, n] 之间，再叠上被同优先级任务抢占的时间。"要求 18 实际 22"
     * 正是要能看见的东西，写回要求值就把它抹掉了。
     */
    app_telem_prof.stage_sleep_us =
        telem_stream_elapsed_us(now_us, APP_TelemStream_PortNowUs());
}

APP_TelemMask APP_TelemStream_DefaultMask(void)
{
    APP_TelemMask mask = telem_stream_param_mask();
    uint32_t index;

    for (index = 0U;
         index < (sizeof(app_telem_stream_default_channels) /
                  sizeof(app_telem_stream_default_channels[0]));
         ++index) {
        mask = APP_TelemMask_Or(
            mask, APP_TelemMask_FromBit(app_telem_stream_default_channels[index]));
    }

    return mask;
}

void APP_TelemStream_Init(void)
{
    if (app_telem_stream.initialised != 0U) {
        return;
    }

    memset(&app_telem_stream, 0, sizeof(app_telem_stream));
    app_telem_stream.rate_hz           = APP_TELEM_RATE_HZ;
    app_telem_stream.mask              = APP_TelemStream_DefaultMask();
    app_telem_stream.refresh_s         = APP_TELEM_STREAM_REFRESH_DEFAULT_S;
    app_telem_stream.format            = APP_TELEM_FORMAT_BIN;
    app_telem_stream.sink              = APP_TELEM_SINK_AUTO;
    app_telem_stream.auto_sink         = APP_TELEM_SINK_UART;
    app_telem_stream.last_command_sink = APP_TELEM_SINK_UART;
    app_telem_stream.initialised       = 1U;
    /* 流配置只存 RAM，上电一律 stream=0（规划文档 §2.4）。 */
    vofaStreamActive = 0U;
}

void APP_TelemStream_Reset(void)
{
    app_telem_stream.initialised = 0U;
    APP_TelemStream_ResetProfile();
    APP_TelemStream_Init();
}

APP_TelemSink APP_TelemStream_ActiveSink(void)
{
    APP_TelemStream_Init();

    if (app_telem_stream.sink != APP_TELEM_SINK_AUTO) {
        return app_telem_stream.sink;
    }

    return app_telem_stream.auto_sink;
}

void APP_TelemStream_NoteCommandSource(APP_TelemSink source)
{
    APP_TelemStream_Init();

    if ((source == APP_TELEM_SINK_UART) || (source == APP_TELEM_SINK_USB) ||
        (source == APP_TELEM_SINK_BT)) {
        app_telem_stream.last_command_sink = source;
    }
}

/*
 * 一个配置在某个出口上到底放不放得下。最坏一帧是全量刷新帧（掩码全置位），
 * 所以按 popcount(mask) 算，不按稳态帧算——否则会出现"平时好好的，刷新那一拍
 * 突然超长"这种最难查的间歇故障。
 */
static APP_TelemStreamStatus telem_stream_check_capacity(APP_TelemMask mask,
                                                         APP_TelemSink sink)
{
    uint32_t payload = APP_TelemFrame_PayloadLength(1U, mask);

    if (payload == 0U) {
        return APP_TELEM_STREAM_ERR_MASK;
    }

    if (payload > (uint32_t)APP_TelemStream_PortMaxPayload(sink)) {
        return APP_TELEM_STREAM_ERR_TOO_LARGE;
    }

    return APP_TELEM_STREAM_OK;
}

/* ------------------------------------------------------------------ */
/* 命令面                                                              */
/* ------------------------------------------------------------------ */

APP_TelemStreamStatus APP_TelemStream_SetActive(uint8_t active)
{
    APP_TelemStream_Init();

    if (active == 0U) {
        vofaStreamActive = 0U;
        return APP_TELEM_STREAM_OK;
    }

    /*
     * `auto` 的语义是"命令从哪条链路进来就往哪条发"，锚点取 STREAM on 这条命令
     * 本身。锁定在开流的这一刻而不是每拍重算：每拍重算的话，飞行中随便一条从
     * 数传发进来的命令都会把正在跑的 USB 流甩到数传上去，占满 5760 B/s。
     */
    if (app_telem_stream.sink == APP_TELEM_SINK_AUTO) {
        app_telem_stream.auto_sink = app_telem_stream.last_command_sink;
    }

    /* 重新起算定时基准：留着上次关流前的截止时刻会让第一拍白等一大段。 */
    app_telem_stream.pace_valid = 0U;

    if ((APP_TelemStream_ActiveSink() == APP_TELEM_SINK_USB) &&
        (APP_TelemStream_PortUsbReady() == 0U)) {
        return APP_TELEM_STREAM_ERR_SINK;
    }

    if ((app_telem_stream.format == APP_TELEM_FORMAT_JF) &&
        (APP_TelemStream_ActiveSink() != APP_TELEM_SINK_UART)) {
        return APP_TELEM_STREAM_ERR_SINK;
    }

    {
        APP_TelemStreamStatus status =
            telem_stream_check_capacity(app_telem_stream.mask,
                                        APP_TelemStream_ActiveSink());
        if (status != APP_TELEM_STREAM_OK) {
            return status;
        }
    }

    app_telem_stream.refresh_accum_ms     = 0U;
    app_telem_stream.encode_error_latched = 0U;
    vofaStreamActive = 1U;
    return APP_TELEM_STREAM_OK;
}

APP_TelemStreamStatus APP_TelemStream_SetRate(uint32_t hz)
{
    APP_TelemStream_Init();

    if ((hz < APP_TELEM_STREAM_RATE_MIN_HZ) || (hz > APP_TELEM_STREAM_RATE_MAX_HZ)) {
        return APP_TELEM_STREAM_ERR_RANGE;
    }

    app_telem_stream.rate_hz = hz;
    return APP_TELEM_STREAM_OK;
}

APP_TelemStreamStatus APP_TelemStream_SetMask(APP_TelemMask mask)
{
    APP_TelemStreamStatus status;

    APP_TelemStream_Init();

    if ((APP_TelemMask_IsEmpty(mask) != 0U) ||
        (APP_TelemMask_IsEmpty(
             APP_TelemMask_AndNot(mask, telem_stream_all_mask())) == 0U)) {
        return APP_TELEM_STREAM_ERR_MASK;
    }

    status = telem_stream_check_capacity(mask, APP_TelemStream_ActiveSink());
    if (status != APP_TELEM_STREAM_OK) {
        return status;
    }

    app_telem_stream.mask = mask;
    /*
     * 掩码变了就把参数通道的影子作废：新选进来的增益通道必须在下一帧被带上，
     * 否则上位机会对着一个永远不更新的滑块等下去（而这看起来像"参数没生效"）。
     */
    app_telem_stream.shadow_valid = 0U;
    return APP_TELEM_STREAM_OK;
}

APP_TelemStreamStatus APP_TelemStream_SetRefresh(uint32_t seconds)
{
    APP_TelemStream_Init();

    if (seconds > APP_TELEM_STREAM_REFRESH_MAX_S) {
        return APP_TELEM_STREAM_ERR_RANGE;
    }

    app_telem_stream.refresh_s        = seconds;
    app_telem_stream.refresh_accum_ms = 0U;
    return APP_TELEM_STREAM_OK;
}

APP_TelemStreamStatus APP_TelemStream_SetFormat(APP_TelemFormat format)
{
    APP_TelemStream_Init();

    if ((format == APP_TELEM_FORMAT_JF) &&
        (APP_TelemStream_ActiveSink() != APP_TELEM_SINK_UART)) {
        /* JustFloat 是 Synex 过渡格式，只有数传那条路认它。 */
        return APP_TELEM_STREAM_ERR_SINK;
    }

    app_telem_stream.format = format;
    return APP_TELEM_STREAM_OK;
}

APP_TelemStreamStatus APP_TelemStream_SetSink(APP_TelemSink sink)
{
    APP_TelemSink resolved;
    APP_TelemStreamStatus status;

    APP_TelemStream_Init();

    resolved = (sink == APP_TELEM_SINK_AUTO) ? app_telem_stream.last_command_sink : sink;

    if ((app_telem_stream.format == APP_TELEM_FORMAT_JF) &&
        (resolved != APP_TELEM_SINK_UART)) {
        return APP_TELEM_STREAM_ERR_SINK;
    }

    status = telem_stream_check_capacity(app_telem_stream.mask, resolved);
    if (status != APP_TELEM_STREAM_OK) {
        return status;
    }

    app_telem_stream.sink = sink;
    if (sink == APP_TELEM_SINK_AUTO) {
        app_telem_stream.auto_sink = app_telem_stream.last_command_sink;
    }
    return APP_TELEM_STREAM_OK;
}

static const char *telem_stream_sink_name(APP_TelemSink sink)
{
    switch (sink) {
    case APP_TELEM_SINK_UART:
        return "uart";
    case APP_TELEM_SINK_USB:
        return "usb";
    case APP_TELEM_SINK_BT:
        return "bt";
    default:
        return "auto";
    }
}

void APP_TelemStream_ReportStatus(void)
{
    char text[224];
    APP_TelemMask mask;

    APP_TelemStream_Init();
    mask = app_telem_stream.mask;

    /*
     * 掩码按 32 位半打印，不用 %llX：目标端是 newlib-nano 的精简 printf，
     * 长长整型转换在那里不保证可用，而 hash 和 mask 一旦打歪上位机就重建错表。
     * 通道号越过 63 之后掩码是 128 位，这里固定发 32 个十六进制字符、高位在前，
     * 上位机按整串解析，不靠长度猜宽度。
     */
    (void)snprintf(text, sizeof(text),
                   "TELEM STREAM stream=%u rate=%lu mask=%08lX%08lX%08lX%08lX "
                   "refresh=%lu "
                   "fmt=%s sink=%s active=%s seq=%u frames=%lu drop=%lu "
                   "usb_lost=%lu\r\n",
                   (unsigned int)((vofaStreamActive != 0U) ? 1U : 0U),
                   (unsigned long)app_telem_stream.rate_hz,
                   (unsigned long)((mask.hi >> 32) & 0xFFFFFFFFULL),
                   (unsigned long)(mask.hi & 0xFFFFFFFFULL),
                   (unsigned long)((mask.lo >> 32) & 0xFFFFFFFFULL),
                   (unsigned long)(mask.lo & 0xFFFFFFFFULL),
                   (unsigned long)app_telem_stream.refresh_s,
                   (app_telem_stream.format == APP_TELEM_FORMAT_JF) ? "jf" : "bin",
                   telem_stream_sink_name(app_telem_stream.sink),
                   (vofaStreamActive != 0U)
                       ? telem_stream_sink_name(APP_TelemStream_ActiveSink())
                       : "-",
                   (unsigned int)app_telem_stream.seq,
                   (unsigned long)app_telem_stream.frames,
                   (unsigned long)app_telem_stream.drops,
                   (unsigned long)app_telem_stream.usb_lost);
    APP_TelemStream_PortReply(text);
}

/* ------------------------------------------------------------------ */
/* 任务面                                                              */
/* ------------------------------------------------------------------ */

/*
 * 脏位检测：把参数通道这一拍的值与影子按**位**比较。
 *
 * 按位而不是按数值比：`!=` 对 NaN 恒真，会让一条坏掉的增益通道每帧都置位，
 * 把"参数不变不回显"这条设计目标悄悄废掉。
 *
 * 检测放在这里而不是给 DRV_COAX_CTRL_SetParam 挂钩子：挂钩子要碰控制路径，
 * 而且漏掉 LOAD / DEFAULTS 这些不走 SetParam 的写入来源；这里本来就每拍取一
 * 遍全部通道，比较是零新增开销的，且对写入来源无差别。
 */
static void telem_stream_update_dirty(const float *values)
{
    uint32_t index;
    uint8_t  first = (app_telem_stream.shadow_valid == 0U) ? 1U : 0U;

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        uint32_t bits;

        if (APP_Telemetry_ChannelHasParam(index) == 0U) {
            continue;
        }

        memcpy(&bits, &values[index], sizeof(bits));
        if ((first != 0U) || (bits != app_telem_stream.shadow[index])) {
            app_telem_stream.dirty =
                APP_TelemMask_Or(app_telem_stream.dirty,
                                 APP_TelemMask_FromBit(index));
        }
        app_telem_stream.shadow[index] = bits;
    }

    app_telem_stream.shadow_valid = 1U;
}

static uint32_t telem_stream_pack(APP_TelemMask mask, const float *values, float *out)
{
    uint32_t index;
    uint32_t used = 0U;

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        if (APP_TelemMask_Test(mask, index) != 0U) {
            out[used++] = values[index];
        }
    }

    return used;
}

static void telem_stream_report_encode_error(void)
{
    /*
     * 只在状态翻转时报一次。40 Hz 逐帧刷 ERR 会把命令回复挤出 uartTxQueue
     * （深度 32、满时丢最旧的一条），本来只是"配置不合法"，结果连带把操作者
     * 用来诊断它的回包也吃掉了。计数留在 `TELEM?` 的 drop= 里，不会丢信息。
     */
    if (app_telem_stream.encode_error_latched != 0U) {
        return;
    }

    app_telem_stream.encode_error_latched = 1U;
    APP_TelemStream_PortReply("ERR telem frame too large\r\n");
}

/*
 * 一拍的实际工作。返回 1 表示这一拍真把帧送出去了。
 *
 * 限速与画像收尾都在外面的 APP_TelemStream_Tick 里：这个函数有七个提前返回，
 * 每个出口都手写一遍统计收尾，迟早会漏掉一个，而漏掉的那个恰好就是要查的那拍。
 */
static uint8_t telem_stream_tick_body(void)
{
    APP_TelemSink        sink;
    APP_TelemFrameDesc   desc;
    APP_TelemFrameStatus encoded;
    APP_TelemMask        send_mask;
    uint32_t             packed;
    uint32_t             mark_us;
    uint16_t             frame_length = 0U;
    uint8_t              full_refresh = 0U;
    uint8_t              sent;

    /* IMUCAP / FLOG 导出独占 CDC 链路，导出期间一帧都不发。 */
    if (APP_TelemStream_PortServiceExports() != 0U) {
        return 0U;
    }

    if (vofaStreamActive == 0U) {
        return 0U;
    }

    sink = APP_TelemStream_ActiveSink();
    if ((sink == APP_TELEM_SINK_USB) && (APP_TelemStream_PortUsbReady() == 0U)) {
        /*
         * USB 拔了。这不是"这一拍没发出去"，是出口没有了：直接关流并计数。
         * 继续假装在发的话，上位机重连后会对着一个自称 stream=1、却永远收不到
         * 帧的飞控排查半天——安静失败正是本次改造要消灭的东西。
         */
        vofaStreamActive = 0U;
        app_telem_stream.usb_lost++;
        return 0U;
    }

    mark_us = APP_TelemStream_PortNowUs();
    if (APP_TelemStream_PortSample(app_telem_stream_values,
                                   (uint32_t)APP_TELEM_CH_COUNT) == 0U) {
        /* 这一拍没有新样本。宁可不发，也不把上一拍的旧值再推一遍。 */
        return 0U;
    }
    app_telem_prof.stage_sample_us =
        telem_stream_elapsed_us(mark_us, APP_TelemStream_PortNowUs());

    mark_us = APP_TelemStream_PortNowUs();
    telem_stream_update_dirty(app_telem_stream_values);

    if (app_telem_stream.refresh_s > 0U) {
        app_telem_stream.refresh_accum_ms += telem_stream_period_ms();
        if (app_telem_stream.refresh_accum_ms >=
            (app_telem_stream.refresh_s * 1000U)) {
            app_telem_stream.refresh_accum_ms = 0U;
            full_refresh = 1U;
        }
    }

    if (app_telem_stream.format == APP_TELEM_FORMAT_JF) {
        /* 旧 JustFloat：定长全表帧，掩码与脏位对它没有意义。 */
        app_telem_prof.stage_encode_us =
            telem_stream_elapsed_us(mark_us, APP_TelemStream_PortNowUs());
        mark_us = APP_TelemStream_PortNowUs();
        sent = APP_TelemStream_PortSendJustFloat(app_telem_stream_values,
                                                 (uint32_t)APP_TELEM_CH_COUNT);
        app_telem_prof.stage_send_us =
            telem_stream_elapsed_us(mark_us, APP_TelemStream_PortNowUs());
        if (sent != 0U) {
            app_telem_stream.seq++;
            app_telem_stream.frames++;
            return 1U;
        }
        app_telem_stream.drops++;
        return 0U;
    }

    if (full_refresh != 0U) {
        send_mask = app_telem_stream.mask;
    } else {
        send_mask = APP_TelemMask_Or(
            APP_TelemMask_AndNot(app_telem_stream.mask, telem_stream_param_mask()),
            APP_TelemMask_And(app_telem_stream.mask, app_telem_stream.dirty));
    }

    if (APP_TelemMask_IsEmpty(send_mask) != 0U) {
        /* 只选了参数通道且这一拍没有变化：本来就没什么要说的。 */
        return 0U;
    }

    packed = telem_stream_pack(send_mask, app_telem_stream_values,
                               app_telem_stream_packed);

    desc.count  = 1U;
    desc.seq    = app_telem_stream.seq;
    desc.schema = APP_Telemetry_SchemaHash();
    desc.t_us   = APP_TelemStream_PortNowUs();
    desc.dt_us  = 0U;
    desc.flags  = (full_refresh != 0U) ? APP_TELEM_FRAME_FLAG_FULL_REFRESH : 0U;
    desc.mask   = send_mask;

    encoded = APP_TelemFrame_Encode(&desc,
                                    app_telem_stream_packed,
                                    packed,
                                    APP_TelemStream_PortMaxPayload(sink),
                                    app_telem_stream_frame,
                                    (uint16_t)sizeof(app_telem_stream_frame),
                                    &frame_length);
    app_telem_prof.stage_encode_us =
        telem_stream_elapsed_us(mark_us, APP_TelemStream_PortNowUs());
    if (encoded != APP_TELEM_FRAME_OK) {
        app_telem_stream.drops++;
        telem_stream_report_encode_error();
        return 0U;
    }
    app_telem_stream.encode_error_latched = 0U;

    mark_us = APP_TelemStream_PortNowUs();
    if (sink == APP_TELEM_SINK_USB) {
        sent = APP_TelemStream_PortSendUsb(app_telem_stream_frame, frame_length);
    } else if (sink == APP_TELEM_SINK_BT) {
        sent = APP_TelemStream_PortSendBt(app_telem_stream_frame, frame_length);
    } else {
        sent = APP_TelemStream_PortSendUart(app_telem_stream_frame, frame_length);
    }
    app_telem_prof.stage_send_us =
        telem_stream_elapsed_us(mark_us, APP_TelemStream_PortNowUs());

    if (sent == 0U) {
        app_telem_stream.drops++;
        return 0U;
    }

    /*
     * 只有真发出去了才清脏位并推进 seq。发失败还清脏位的话，那次参数变化就
     * 永远不会再上线，滑块会一直停在 pending。
     */
    app_telem_stream.dirty = APP_TelemMask_AndNot(app_telem_stream.dirty, send_mask);
    app_telem_stream.seq++;
    app_telem_stream.frames++;
    return 1U;
}

void APP_TelemStream_Tick(void)
{
    uint32_t entry_us;
    uint8_t  sent;

    APP_TelemStream_Init();

    app_telem_prof.stage_sleep_us  = 0U;
    app_telem_prof.stage_sample_us = 0U;
    app_telem_prof.stage_encode_us = 0U;
    app_telem_prof.stage_send_us   = 0U;

    entry_us = APP_TelemStream_PortNowUs();
    telem_stream_pace();
    sent = telem_stream_tick_body();

    /*
     * 周期量的是**两次进入之间**的间隔，也就是睡眠 + 干活的总和，与上位机在
     * 线上看到的帧间隔同口径。只量睡眠会把"周期里还夹着 7 ms 阻塞发送"这类
     * 问题量没了——那恰好是这条流上真出过的事故。
     */
    if (app_telem_prof.entry_valid != 0U) {
        uint32_t period_us =
            telem_stream_elapsed_us(app_telem_prof.entry_us, entry_us);

        app_telem_prof.period_last_us = period_us;
        if ((app_telem_prof.ticks == 0U) ||
            (period_us < app_telem_prof.period_min_us)) {
            app_telem_prof.period_min_us = period_us;
        }
        telem_stream_prof_accum(&app_telem_prof.period_max_us,
                                &app_telem_prof.period_sum_us, period_us);
        app_telem_prof.ticks++;
    }
    app_telem_prof.entry_us    = entry_us;
    app_telem_prof.entry_valid = 1U;

    telem_stream_prof_accum(&app_telem_prof.sleep_max_us,
                            &app_telem_prof.sleep_sum_us,
                            app_telem_prof.stage_sleep_us);
    telem_stream_prof_accum(&app_telem_prof.sample_max_us,
                            &app_telem_prof.sample_sum_us,
                            app_telem_prof.stage_sample_us);
    telem_stream_prof_accum(&app_telem_prof.encode_max_us,
                            &app_telem_prof.encode_sum_us,
                            app_telem_prof.stage_encode_us);
    telem_stream_prof_accum(&app_telem_prof.send_max_us,
                            &app_telem_prof.send_sum_us,
                            app_telem_prof.stage_send_us);

    if (sent != 0U) {
        app_telem_prof.sent++;
    } else {
        app_telem_prof.skipped++;
    }
}

void APP_TelemStream_ResetProfile(void)
{
    memset(&app_telem_prof, 0, sizeof(app_telem_prof));
}

static uint32_t telem_stream_prof_avg(uint64_t sum, uint32_t count)
{
    return (count == 0U) ? 0U : (uint32_t)(sum / (uint64_t)count);
}

void APP_TelemStream_ReportProfile(void)
{
    char     text[224];
    uint32_t ticks   = app_telem_prof.ticks;
    uint32_t avg_us  = telem_stream_prof_avg(app_telem_prof.period_sum_us, ticks);
    /*
     * 实测速率同时用 x100 定点给出来。上位机要判"设 40 实际多少"，自己拿平均
     * 周期去倒数是能算，但每个脚本都要再写一遍同样的换算，写歪一次就得重测；
     * 固件这边本来就有这两个数，顺手算完更不容易错。
     */
    uint32_t rate_x100 = (avg_us == 0U) ? 0U
                                        : (uint32_t)((100000000ULL + (avg_us / 2U)) /
                                                     (uint64_t)avg_us);

    (void)snprintf(text, sizeof(text),
                   "TELEM PROF n=%lu sent=%lu skip=%lu late=%lu "
                   "period_us=%lu/%lu/%lu last=%lu rate_x100=%lu\r\n",
                   (unsigned long)ticks,
                   (unsigned long)app_telem_prof.sent,
                   (unsigned long)app_telem_prof.skipped,
                   (unsigned long)app_telem_prof.late,
                   (unsigned long)((ticks == 0U) ? 0U : app_telem_prof.period_min_us),
                   (unsigned long)avg_us,
                   (unsigned long)app_telem_prof.period_max_us,
                   (unsigned long)app_telem_prof.period_last_us,
                   (unsigned long)rate_x100);
    APP_TelemStream_PortReply(text);

    /*
     * 分段耗时的分母用 sent+skip 而不是 ticks：每一拍都会走一遍这些分段，
     * 而 ticks 少算了第一拍（那拍没有上一次进入时刻，算不出周期）。
     */
    {
        uint32_t stages = app_telem_prof.sent + app_telem_prof.skipped;

        (void)snprintf(text, sizeof(text),
                       "TELEM PROF stage_us sleep=%lu/%lu sample=%lu/%lu "
                       "encode=%lu/%lu send=%lu/%lu avg/max\r\n",
                       (unsigned long)telem_stream_prof_avg(
                           app_telem_prof.sleep_sum_us, stages),
                       (unsigned long)app_telem_prof.sleep_max_us,
                       (unsigned long)telem_stream_prof_avg(
                           app_telem_prof.sample_sum_us, stages),
                       (unsigned long)app_telem_prof.sample_max_us,
                       (unsigned long)telem_stream_prof_avg(
                           app_telem_prof.encode_sum_us, stages),
                       (unsigned long)app_telem_prof.encode_max_us,
                       (unsigned long)telem_stream_prof_avg(
                           app_telem_prof.send_sum_us, stages),
                       (unsigned long)app_telem_prof.send_max_us);
        APP_TelemStream_PortReply(text);
    }
}
