/*
 * app_cmd_telem.c —— `TELEM` 命令族。
 *
 * 从 app_control.c 搬出来（R-T1-1，仿 app_cmd_flow.c）。原来那里只有
 * `TELEM?` / `TELEM CH from=`；本次新增流控制子命令，按仓库规矩不往那个
 * 4000 行的文件里追加，整族迁到这里，app_control.c 的分发行原样指向本文件。
 *
 * 子命令见 doc/telemetry-protocol.md。流配置**只存 RAM**：调参用的
 * 流不该在下次上电时自己跑起来占满数传。
 */

#include "app_control.h"
#include "app_control_internal.h"

#include "app_telem_stream.h"
#include "app_telemetry.h"

#include <stddef.h>
#include <string.h>

static void app_cmd_telem_report_status(APP_TelemStreamStatus status,
                                        const char *subcommand)
{
    switch (status) {
    case APP_TELEM_STREAM_OK:
        APP_TelemStream_ReportStatus();
        break;
    case APP_TELEM_STREAM_ERR_RANGE:
        APP_Control_QueueText("ERR telem %s out of range\r\n", subcommand);
        break;
    case APP_TELEM_STREAM_ERR_MASK:
        APP_Control_QueueText("ERR telem mask empty or unknown channel n=%u\r\n",
                              (unsigned int)APP_TELEM_CH_COUNT);
        break;
    case APP_TELEM_STREAM_ERR_TOO_LARGE:
        APP_Control_QueueText("ERR telem frame too large\r\n");
        break;
    case APP_TELEM_STREAM_ERR_SINK:
    default:
        APP_Control_QueueText("ERR telem sink unavailable\r\n");
        break;
    }
}

/*
 * 16 位十六进制掩码。不用 strtoull：目标端的精简 C 库对 long long 转换不保证
 * 可用，而这里解错一位就会让上位机收到一张与它请求的完全不同的通道集合——
 * 帧自描述能挡住错位，但挡不住"我要的通道根本没发"。所以自己逐字符解，
 * 并且拒绝任何非法字符与超长输入，不做"能解多少算多少"的宽容处理。
 */
static uint8_t app_cmd_telem_parse_mask(const char *text, uint64_t *value)
{
    uint64_t parsed = 0ULL;
    uint32_t digits = 0U;

    if ((text == NULL) || (value == NULL) || (text[0] == '\0')) {
        return 0U;
    }

    if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X'))) {
        text += 2;
    }

    while (*text != '\0') {
        char c = *text;
        uint32_t nibble;

        if ((c >= '0') && (c <= '9')) {
            nibble = (uint32_t)(c - '0');
        } else if ((c >= 'a') && (c <= 'f')) {
            nibble = (uint32_t)(c - 'a') + 10U;
        } else if ((c >= 'A') && (c <= 'F')) {
            nibble = (uint32_t)(c - 'A') + 10U;
        } else {
            return 0U;
        }

        digits++;
        if (digits > 16U) {
            return 0U;
        }

        parsed = (parsed << 4) | (uint64_t)nibble;
        text++;
    }

    if (digits == 0U) {
        return 0U;
    }

    *value = parsed;
    return 1U;
}

static void app_cmd_telem_usage(void)
{
    APP_Control_QueueText(
        "ERR usage TELEM? | TELEM CH from=<n> | TELEM STREAM on|off | "
        "TELEM RATE <hz> | TELEM MASK <hex> | TELEM REFRESH <s> | "
        "TELEM FORMAT bin|jf | TELEM SINK usb|uart|auto\r\n");
}

void app_control_handle_telem(char **tokens, uint32_t count)
{
    const char *from_text;
    uint32_t    from;
    uint32_t    value;

    if (tokens == NULL) {
        return;
    }

    if (strcmp(tokens[0], "TELEM?") == 0) {
        if (count != 1U) {
            APP_Control_QueueText("ERR usage TELEM?\r\n");
            return;
        }
        APP_Telemetry_ReportHeader();
        APP_TelemStream_ReportStatus();
        return;
    }

    if (count < 2U) {
        app_cmd_telem_usage();
        return;
    }

    if (strcmp(tokens[1], "CH") == 0) {
        from_text = app_control_token_value(tokens, count, "from");
        if ((count != 3U) || (from_text == NULL) ||
            (app_control_parse_u32(from_text, &from) == 0U)) {
            APP_Control_QueueText("ERR usage TELEM CH from=<n>\r\n");
            return;
        }
        APP_Telemetry_ReportPage(from);
        return;
    }

    if (strcmp(tokens[1], "STREAM") == 0) {
        if (count != 3U) {
            APP_Control_QueueText("ERR usage TELEM STREAM on|off\r\n");
            return;
        }
        if (strcmp(tokens[2], "on") == 0) {
            app_cmd_telem_report_status(APP_TelemStream_SetActive(1U), "stream");
        } else if (strcmp(tokens[2], "off") == 0) {
            app_cmd_telem_report_status(APP_TelemStream_SetActive(0U), "stream");
        } else {
            APP_Control_QueueText("ERR usage TELEM STREAM on|off\r\n");
        }
        return;
    }

    if (strcmp(tokens[1], "RATE") == 0) {
        if ((count != 3U) || (app_control_parse_u32(tokens[2], &value) == 0U)) {
            APP_Control_QueueText("ERR usage TELEM RATE %u..%u\r\n",
                                  (unsigned int)APP_TELEM_STREAM_RATE_MIN_HZ,
                                  (unsigned int)APP_TELEM_STREAM_RATE_MAX_HZ);
            return;
        }
        app_cmd_telem_report_status(APP_TelemStream_SetRate(value), "rate");
        return;
    }

    if (strcmp(tokens[1], "MASK") == 0) {
        uint64_t mask = 0ULL;

        if ((count != 3U) || (app_cmd_telem_parse_mask(tokens[2], &mask) == 0U)) {
            APP_Control_QueueText("ERR usage TELEM MASK <hex up to 16 digits>\r\n");
            return;
        }
        app_cmd_telem_report_status(APP_TelemStream_SetMask(mask), "mask");
        return;
    }

    if (strcmp(tokens[1], "REFRESH") == 0) {
        if ((count != 3U) || (app_control_parse_u32(tokens[2], &value) == 0U)) {
            APP_Control_QueueText("ERR usage TELEM REFRESH 0..%u\r\n",
                                  (unsigned int)APP_TELEM_STREAM_REFRESH_MAX_S);
            return;
        }
        app_cmd_telem_report_status(APP_TelemStream_SetRefresh(value), "refresh");
        return;
    }

    if (strcmp(tokens[1], "FORMAT") == 0) {
        if (count != 3U) {
            APP_Control_QueueText("ERR usage TELEM FORMAT bin|jf\r\n");
            return;
        }
        if (strcmp(tokens[2], "bin") == 0) {
            app_cmd_telem_report_status(
                APP_TelemStream_SetFormat(APP_TELEM_FORMAT_BIN), "format");
        } else if (strcmp(tokens[2], "jf") == 0) {
            app_cmd_telem_report_status(
                APP_TelemStream_SetFormat(APP_TELEM_FORMAT_JF), "format");
        } else {
            APP_Control_QueueText("ERR usage TELEM FORMAT bin|jf\r\n");
        }
        return;
    }

    if (strcmp(tokens[1], "SINK") == 0) {
        if (count != 3U) {
            APP_Control_QueueText("ERR usage TELEM SINK usb|uart|auto\r\n");
            return;
        }
        if (strcmp(tokens[2], "usb") == 0) {
            app_cmd_telem_report_status(
                APP_TelemStream_SetSink(APP_TELEM_SINK_USB), "sink");
        } else if (strcmp(tokens[2], "uart") == 0) {
            app_cmd_telem_report_status(
                APP_TelemStream_SetSink(APP_TELEM_SINK_UART), "sink");
        } else if (strcmp(tokens[2], "auto") == 0) {
            app_cmd_telem_report_status(
                APP_TelemStream_SetSink(APP_TELEM_SINK_AUTO), "sink");
        } else {
            APP_Control_QueueText("ERR usage TELEM SINK usb|uart|auto\r\n");
        }
        return;
    }

    app_cmd_telem_usage();
}
