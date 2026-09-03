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
 *   它们在场是为了让 1 Hz 的全量刷新帧能把 14 个滑块一次喂给刚连上的上位机。
 *   刷新帧 = 26 路 -> 137 B，**替换**当拍的稳态帧而不是额外加一帧，
 *   合计 39×81 + 137 = 3296 B/s（57.2%），仍在 60% 以内。
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
    uint64_t        mask;
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
    uint64_t        dirty;
    uint32_t        shadow[APP_TELEM_CH_COUNT];
    uint8_t         shadow_valid;
    uint8_t         encode_error_latched;
    uint8_t         initialised;
} APP_TelemStreamState;

static APP_TelemStreamState app_telem_stream;

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

static uint64_t telem_stream_all_mask(void)
{
    uint32_t count = (uint32_t)APP_TELEM_CH_COUNT;

    if (count >= 64U) {
        return ~0ULL;
    }

    return (1ULL << count) - 1ULL;
}

/* 参数回显通道的位集合。这些通道按变化发，不进稳态帧。 */
static uint64_t telem_stream_param_mask(void)
{
    uint64_t mask = 0ULL;
    uint32_t index;

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        if (APP_Telemetry_ChannelHasParam(index) != 0U) {
            mask |= (1ULL << index);
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

uint64_t APP_TelemStream_DefaultMask(void)
{
    uint64_t mask = telem_stream_param_mask();
    uint32_t index;

    for (index = 0U;
         index < (sizeof(app_telem_stream_default_channels) /
                  sizeof(app_telem_stream_default_channels[0]));
         ++index) {
        mask |= (1ULL << app_telem_stream_default_channels[index]);
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

    if ((source == APP_TELEM_SINK_UART) || (source == APP_TELEM_SINK_USB)) {
        app_telem_stream.last_command_sink = source;
    }
}

/*
 * 一个配置在某个出口上到底放不放得下。最坏一帧是全量刷新帧（掩码全置位），
 * 所以按 popcount(mask) 算，不按稳态帧算——否则会出现"平时好好的，刷新那一拍
 * 突然超长"这种最难查的间歇故障。
 */
static APP_TelemStreamStatus telem_stream_check_capacity(uint64_t mask,
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

APP_TelemStreamStatus APP_TelemStream_SetMask(uint64_t mask)
{
    APP_TelemStreamStatus status;

    APP_TelemStream_Init();

    if ((mask == 0ULL) || ((mask & ~telem_stream_all_mask()) != 0ULL)) {
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
    default:
        return "auto";
    }
}

void APP_TelemStream_ReportStatus(void)
{
    char text[192];
    uint64_t mask;

    APP_TelemStream_Init();
    mask = app_telem_stream.mask;

    /*
     * 掩码按两个 32 位半打印，不用 %llX：目标端是 newlib-nano 的精简 printf，
     * 长长整型转换在那里不保证可用，而 hash 和 mask 一旦打歪上位机就重建错表。
     */
    (void)snprintf(text, sizeof(text),
                   "TELEM STREAM stream=%u rate=%lu mask=%08lX%08lX refresh=%lu "
                   "fmt=%s sink=%s active=%s seq=%u frames=%lu drop=%lu "
                   "usb_lost=%lu\r\n",
                   (unsigned int)((vofaStreamActive != 0U) ? 1U : 0U),
                   (unsigned long)app_telem_stream.rate_hz,
                   (unsigned long)((mask >> 32) & 0xFFFFFFFFULL),
                   (unsigned long)(mask & 0xFFFFFFFFULL),
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
            app_telem_stream.dirty |= (1ULL << index);
        }
        app_telem_stream.shadow[index] = bits;
    }

    app_telem_stream.shadow_valid = 1U;
}

static uint32_t telem_stream_pack(uint64_t mask, const float *values, float *out)
{
    uint32_t index;
    uint32_t used = 0U;

    for (index = 0U; index < (uint32_t)APP_TELEM_CH_COUNT; ++index) {
        if ((mask & (1ULL << index)) != 0ULL) {
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

void APP_TelemStream_Tick(void)
{
    APP_TelemSink        sink;
    APP_TelemFrameDesc   desc;
    APP_TelemFrameStatus encoded;
    uint64_t             send_mask;
    uint32_t             packed;
    uint16_t             frame_length = 0U;
    uint8_t              full_refresh = 0U;
    uint8_t              sent;

    APP_TelemStream_Init();
    APP_TelemStream_PortDelayMs(telem_stream_period_ms());

    /* IMUCAP / FLOG 导出独占 CDC 链路，导出期间一帧都不发。 */
    if (APP_TelemStream_PortServiceExports() != 0U) {
        return;
    }

    if (vofaStreamActive == 0U) {
        return;
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
        return;
    }

    if (APP_TelemStream_PortSample(app_telem_stream_values,
                                   (uint32_t)APP_TELEM_CH_COUNT) == 0U) {
        /* 这一拍没有新样本。宁可不发，也不把上一拍的旧值再推一遍。 */
        return;
    }

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
        sent = APP_TelemStream_PortSendJustFloat(app_telem_stream_values,
                                                 (uint32_t)APP_TELEM_CH_COUNT);
        if (sent != 0U) {
            app_telem_stream.seq++;
            app_telem_stream.frames++;
        } else {
            app_telem_stream.drops++;
        }
        return;
    }

    if (full_refresh != 0U) {
        send_mask = app_telem_stream.mask;
    } else {
        send_mask = (app_telem_stream.mask & ~telem_stream_param_mask()) |
                    (app_telem_stream.mask & app_telem_stream.dirty);
    }

    if (send_mask == 0ULL) {
        /* 只选了参数通道且这一拍没有变化：本来就没什么要说的。 */
        return;
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
    if (encoded != APP_TELEM_FRAME_OK) {
        app_telem_stream.drops++;
        telem_stream_report_encode_error();
        return;
    }
    app_telem_stream.encode_error_latched = 0U;

    sent = (sink == APP_TELEM_SINK_USB)
               ? APP_TelemStream_PortSendUsb(app_telem_stream_frame, frame_length)
               : APP_TelemStream_PortSendUart(app_telem_stream_frame, frame_length);

    if (sent == 0U) {
        app_telem_stream.drops++;
        return;
    }

    /*
     * 只有真发出去了才清脏位并推进 seq。发失败还清脏位的话，那次参数变化就
     * 永远不会再上线，滑块会一直停在 pending。
     */
    app_telem_stream.dirty &= ~send_mask;
    app_telem_stream.seq++;
    app_telem_stream.frames++;
}
