#include "app_aiwb2.h"

#include "app_maint_uart.h"
#include "bsp_aiwb2_power.h"
#include "bsp_uart.h"

#include "main.h"

#include <stdio.h>
#include <string.h>

/* These values are only used if APP_AIWB2_PASSIVE_ONLY is set to 0.
   Keep real WiFi credentials out of git; override them from a private build
   config if active module provisioning is re-enabled. */
#ifndef APP_AIWB2_WIFI_SSID
#define APP_AIWB2_WIFI_SSID       "CHANGE_ME"
#endif

#ifndef APP_AIWB2_WIFI_PASSWORD
#define APP_AIWB2_WIFI_PASSWORD   "CHANGE_ME"
#endif

#ifndef APP_AIWB2_UDP_SERVER_PORT
#define APP_AIWB2_UDP_SERVER_PORT "7777"
#endif

/* Keep AT provisioning passive by default so firmware does not collide with
   the module's persisted auto-transparent flow. PC6 now controls Ai-WB2 EN,
   so passive mode may power-cycle the module to make it retry TCP connect. */
#define APP_AIWB2_PASSIVE_ONLY        1U
#define APP_AIWB2_START_DELAY_MS     8000U
#define APP_AIWB2_PROBE_TIMEOUT_MS   8000U
#define APP_AIWB2_ESCAPE_GUARD_MS    1200U
#define APP_AIWB2_BOOT_TIMEOUT_MS   35000U
#define APP_AIWB2_TRANSPARENT_OK_TIMEOUT_MS 3000U
#define APP_AIWB2_BUSY_BACKOFF_MS    6000U
#define APP_AIWB2_AUTOCONNECT_MS    25000U
#define APP_AIWB2_RETRY_MIN_MS       3000U
#define APP_AIWB2_RETRY_MAX_MS      15000U
#define APP_AIWB2_PASSIVE_ERROR_RETRY_MS 3000U
#define APP_AIWB2_PASSIVE_POWER_RECYCLE_ENABLED 1U
#define APP_AIWB2_PASSIVE_POWER_OFF_MS 800U
#define APP_AIWB2_UART8_MIRROR_ENABLED 1U
#define APP_AIWB2_MANUAL_AT_GUARD_MS 2500U

typedef struct {
    const char *text;
    uint32_t timeout_ms;
    uint8_t allow_error;
    uint8_t causes_reset;
} APP_AiWB2Command;

static const APP_AiWB2Command aiwb2_commands[] = {
    { "ATE0", 1500U, 0U, 0U },
    { "AT+WMODE=1,1", 1500U, 0U, 0U },
    { "AT+WJAP=\"" APP_AIWB2_WIFI_SSID "\",\"" APP_AIWB2_WIFI_PASSWORD "\"", 25000U, 0U, 0U },
    { "AT+WAUTOCONN=1", 1500U, 0U, 0U },
    { "AT+SOCKETDEL=1", 2000U, 1U, 0U },
    { "AT+SOCKETAUTOTT=1," APP_AIWB2_UDP_SERVER_PORT, 2500U, 0U, 0U },
    { "AT+RST", 8000U, 0U, 1U },
};

static APP_AiWB2_State aiwb2_state;
static uint32_t aiwb2_deadline_ms;
static uint32_t aiwb2_retry_count;
static uint32_t aiwb2_command_index;
static uint8_t aiwb2_probe_escape_used;
static uint8_t aiwb2_power_recycle_active;
static int32_t aiwb2_last_socket_error;
static APP_AiWB2Command aiwb2_provision_commands[7];
static char aiwb2_provision_text[7][128];
static uint32_t aiwb2_provision_command_count;
static uint8_t aiwb2_provision_active;
static uint32_t aiwb2_manual_at_deadline_ms;

static uint8_t aiwb2_starts_with(const char *line, const char *prefix)
{
    return (strncmp(line, prefix, strlen(prefix)) == 0) ? 1U : 0U;
}

static uint8_t aiwb2_contains(const char *line, const char *needle)
{
    return (strstr(line, needle) != 0) ? 1U : 0U;
}

static uint8_t aiwb2_should_mirror_to_maint(const char *line)
{
    if ((line == 0) || (*line == '\0')) {
        return 0U;
    }

    if ((aiwb2_state != APP_AIWB2_STATE_TRANSPARENT) ||
        (aiwb2_provision_active != 0U)) {
        return 1U;
    }

    if ((aiwb2_starts_with(line, "+EVENT:") != 0U) ||
        (aiwb2_starts_with(line, "+SOCKET:") != 0U) ||
        (aiwb2_contains(line, "[Busy]Cmd") != 0U) ||
        (aiwb2_contains(line, "Unknown cmd:") != 0U) ||
        (aiwb2_contains(line, "connect success") != 0U) ||
        (strcmp(line, "OK") == 0) ||
        (strcmp(line, "ERROR") == 0) ||
        (strcmp(line, ">") == 0)) {
        return 1U;
    }

    return 0U;
}

static void aiwb2_mirror_to_maint(const char *line)
{
#if (APP_AIWB2_UART8_MIRROR_ENABLED != 0U)
    if (aiwb2_should_mirror_to_maint(line) == 0U) {
        return;
    }

    APP_MaintUART_Write("WIFI_RX ", 8U);
    APP_MaintUART_Write(line, (uint16_t)strlen(line));
    APP_MaintUART_Write("\r\n", 2U);
#else
    (void)line;
#endif
}

static void aiwb2_send_raw(const char *text)
{
    (void)BSP_UART_Transmit_USART1((const uint8_t *)text,
                                   (uint16_t)strlen(text),
                                   100U);
}

static void aiwb2_send_command(const char *text)
{
    aiwb2_send_raw(text);
    aiwb2_send_raw("\r\n");
}

static uint8_t aiwb2_time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static uint8_t aiwb2_manual_at_active(void)
{
    return (aiwb2_time_reached(HAL_GetTick(), aiwb2_manual_at_deadline_ms) == 0U) ? 1U : 0U;
}

#if (APP_AIWB2_PASSIVE_ONLY == 0U)
static uint32_t aiwb2_retry_delay_ms(void)
{
    uint32_t delay_ms = APP_AIWB2_RETRY_MIN_MS + (aiwb2_retry_count * 2000U);

    if (delay_ms > APP_AIWB2_RETRY_MAX_MS) {
        delay_ms = APP_AIWB2_RETRY_MAX_MS;
    }

    return delay_ms;
}
#endif

static void aiwb2_enter_retry_delay(void)
{
    uint32_t now_ms = HAL_GetTick();

#if (APP_AIWB2_PASSIVE_ONLY != 0U)
    aiwb2_state = APP_AIWB2_STATE_WAIT_BOOT_CONNECT;
    aiwb2_deadline_ms = now_ms + APP_AIWB2_PASSIVE_ERROR_RETRY_MS;
    return;
#else
    if (aiwb2_retry_count < 1000U) {
        ++aiwb2_retry_count;
    }

    aiwb2_state = APP_AIWB2_STATE_RETRY_DELAY;
    aiwb2_deadline_ms = now_ms + aiwb2_retry_delay_ms();
#endif
}

static void aiwb2_begin_probe(void)
{
    uint32_t now_ms = HAL_GetTick();

    aiwb2_send_command("AT");
    aiwb2_state = APP_AIWB2_STATE_WAIT_PROBE;
    aiwb2_deadline_ms = now_ms + APP_AIWB2_PROBE_TIMEOUT_MS;
}

static void aiwb2_begin_config(void)
{
    aiwb2_command_index = 0U;
    aiwb2_probe_escape_used = 0U;
    aiwb2_state = APP_AIWB2_STATE_SEND_COMMAND;
}

static const APP_AiWB2Command *aiwb2_active_commands(uint32_t *count)
{
    if (aiwb2_provision_active != 0U) {
        if (count != 0) {
            *count = aiwb2_provision_command_count;
        }
        return aiwb2_provision_commands;
    }

    if (count != 0) {
        *count = (uint32_t)(sizeof(aiwb2_commands) / sizeof(aiwb2_commands[0]));
    }
    return aiwb2_commands;
}

static void aiwb2_command_done(void)
{
    ++aiwb2_command_index;
    aiwb2_state = APP_AIWB2_STATE_SEND_COMMAND;
}

static void aiwb2_wait_from_now(uint32_t delay_ms)
{
    aiwb2_deadline_ms = HAL_GetTick() + delay_ms;
}

static void aiwb2_enter_transparent(void)
{
    aiwb2_state = APP_AIWB2_STATE_TRANSPARENT;
    aiwb2_retry_count = 0U;
}

static void aiwb2_wait_transparent_ok(void)
{
    aiwb2_state = APP_AIWB2_STATE_WAIT_TRANSPARENT_OK;
    aiwb2_deadline_ms = HAL_GetTick() + APP_AIWB2_TRANSPARENT_OK_TIMEOUT_MS;
}

static void aiwb2_parse_socket_error(const char *line)
{
    const char *cursor = line;
    int32_t value = 0;
    uint8_t have_digit = 0U;

    cursor = strstr(line, "+SOCKET:");
    if (cursor == 0) {
        return;
    }

    cursor += strlen("+SOCKET:");
    while ((*cursor >= '0') && (*cursor <= '9')) {
        have_digit = 1U;
        value = (value * 10) + (int32_t)(*cursor - '0');
        ++cursor;
    }

    if (have_digit != 0U) {
        aiwb2_last_socket_error = value;
    }
}

void APP_AiWB2_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

#if (APP_AIWB2_PASSIVE_ONLY != 0U)
    aiwb2_state = APP_AIWB2_STATE_WAIT_BOOT_CONNECT;
    aiwb2_deadline_ms = now_ms + APP_AIWB2_BOOT_TIMEOUT_MS;
#else
    aiwb2_state = APP_AIWB2_STATE_START_DELAY;
    aiwb2_deadline_ms = now_ms + APP_AIWB2_START_DELAY_MS;
#endif
    aiwb2_retry_count = 0U;
    aiwb2_command_index = 0U;
    aiwb2_probe_escape_used = 0U;
    aiwb2_power_recycle_active = 0U;
    aiwb2_last_socket_error = -1;
    aiwb2_provision_active = 0U;
    aiwb2_provision_command_count = 0U;
    aiwb2_manual_at_deadline_ms = 0U;
}

void APP_AiWB2_Tick(void)
{
    uint32_t now_ms = HAL_GetTick();
    const APP_AiWB2Command *commands;
    const APP_AiWB2Command *command;
    uint32_t command_count;

#if (APP_AIWB2_PASSIVE_ONLY != 0U)
    if (aiwb2_provision_active == 0U) {
#if (APP_AIWB2_PASSIVE_POWER_RECYCLE_ENABLED != 0U)
        if (aiwb2_state == APP_AIWB2_STATE_TRANSPARENT) {
            return;
        }

        if (aiwb2_power_recycle_active != 0U) {
            if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
                aiwb2_power_recycle_active = 0U;
                BSP_AiWB2_SetEnabled(1U);
                aiwb2_state = APP_AIWB2_STATE_WAIT_BOOT_CONNECT;
                aiwb2_deadline_ms = now_ms + APP_AIWB2_BOOT_TIMEOUT_MS;
            }
            return;
        }

        if (BSP_AiWB2_IsEnabled() == 0U) {
            aiwb2_deadline_ms = now_ms + APP_AIWB2_BOOT_TIMEOUT_MS;
            return;
        }

        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            if (aiwb2_retry_count < 1000U) {
                ++aiwb2_retry_count;
            }
            BSP_AiWB2_SetEnabled(0U);
            aiwb2_power_recycle_active = 1U;
            aiwb2_state = APP_AIWB2_STATE_RETRY_DELAY;
            aiwb2_deadline_ms = now_ms + APP_AIWB2_PASSIVE_POWER_OFF_MS;
        }
#endif
        return;
    }
#endif

    switch (aiwb2_state) {
    case APP_AIWB2_STATE_START_DELAY:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_begin_probe();
        }
        break;

    case APP_AIWB2_STATE_WAIT_PROBE:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) == 0U) {
            break;
        }

        if (aiwb2_probe_escape_used == 0U) {
            aiwb2_probe_escape_used = 1U;
            aiwb2_state = APP_AIWB2_STATE_ESCAPE_BEFORE;
            aiwb2_deadline_ms = now_ms + APP_AIWB2_ESCAPE_GUARD_MS;
        } else {
            aiwb2_enter_retry_delay();
        }
        break;

    case APP_AIWB2_STATE_ESCAPE_BEFORE:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_send_raw("+++");
            aiwb2_state = APP_AIWB2_STATE_ESCAPE_AFTER;
            aiwb2_deadline_ms = now_ms + APP_AIWB2_ESCAPE_GUARD_MS;
        }
        break;

    case APP_AIWB2_STATE_ESCAPE_AFTER:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_begin_probe();
        }
        break;

    case APP_AIWB2_STATE_SEND_COMMAND:
        commands = aiwb2_active_commands(&command_count);
        if (aiwb2_command_index >= command_count) {
            aiwb2_state = APP_AIWB2_STATE_WAIT_BOOT_CONNECT;
            aiwb2_deadline_ms = now_ms + APP_AIWB2_BOOT_TIMEOUT_MS;
            aiwb2_provision_active = 0U;
            break;
        }

        command = &commands[aiwb2_command_index];
        aiwb2_send_command(command->text);
        if (command->causes_reset != 0U) {
            ++aiwb2_command_index;
            aiwb2_provision_active = 0U;
            aiwb2_state = APP_AIWB2_STATE_WAIT_BOOT_CONNECT;
            aiwb2_deadline_ms = now_ms + APP_AIWB2_BOOT_TIMEOUT_MS;
        } else {
            aiwb2_state = APP_AIWB2_STATE_WAIT_COMMAND;
            aiwb2_deadline_ms = now_ms + command->timeout_ms;
        }
        break;

    case APP_AIWB2_STATE_WAIT_COMMAND:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_enter_retry_delay();
        }
        break;

    case APP_AIWB2_STATE_WAIT_BOOT_CONNECT:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_enter_retry_delay();
        }
        break;

    case APP_AIWB2_STATE_WAIT_TRANSPARENT_OK:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_enter_retry_delay();
        }
        break;

    case APP_AIWB2_STATE_RETRY_DELAY:
        if (aiwb2_time_reached(now_ms, aiwb2_deadline_ms) != 0U) {
            aiwb2_begin_probe();
        }
        break;

    case APP_AIWB2_STATE_TRANSPARENT:
    default:
        break;
    }
}

void APP_AiWB2_ProcessLine(const char *line)
{
    const APP_AiWB2Command *command;

    if ((line == 0) || (*line == '\0')) {
        return;
    }

    aiwb2_mirror_to_maint(line);

    if (aiwb2_contains(line, "+SOCKET:") != 0U) {
        aiwb2_parse_socket_error(line);
    }

    if ((aiwb2_manual_at_active() != 0U) &&
        ((strcmp(line, "OK") == 0) ||
         (strcmp(line, "ERROR") == 0) ||
         (aiwb2_contains(line, "Unknown cmd:") != 0U))) {
        return;
    }

    if (strcmp(line, ">") == 0) {
        aiwb2_enter_transparent();
        return;
    }

    if ((aiwb2_state == APP_AIWB2_STATE_WAIT_TRANSPARENT_OK) &&
        (strcmp(line, "OK") == 0)) {
        aiwb2_enter_transparent();
        return;
    }

    if (aiwb2_contains(line, "connect success") != 0U) {
        aiwb2_wait_transparent_ok();
        return;
    }

    if ((aiwb2_contains(line, "[Busy]Cmd running") != 0U) ||
        (aiwb2_contains(line, "[Busy]Cmd") != 0U)) {
        if (aiwb2_state != APP_AIWB2_STATE_TRANSPARENT) {
            aiwb2_wait_from_now(APP_AIWB2_BUSY_BACKOFF_MS);
        }
        return;
    }

    if (aiwb2_state == APP_AIWB2_STATE_WAIT_COMMAND) {
        uint32_t command_count;
        const APP_AiWB2Command *commands = aiwb2_active_commands(&command_count);

        if (aiwb2_command_index >= command_count) {
            return;
        }
        command = &commands[aiwb2_command_index];
        if (strcmp(line, "OK") == 0) {
            aiwb2_command_done();
            return;
        }
        if ((strcmp(line, "ERROR") == 0) || (aiwb2_contains(line, "+SOCKET:97") != 0U)) {
            if (command->allow_error != 0U) {
                aiwb2_command_done();
            } else {
                aiwb2_enter_retry_delay();
            }
            return;
        }
    }

    if ((aiwb2_state != APP_AIWB2_STATE_TRANSPARENT) &&
        (aiwb2_provision_active == 0U) &&
        ((aiwb2_contains(line, "+EVENT:WIFI_CONNECT") != 0U) ||
         (aiwb2_contains(line, "+EVENT:WIFI_GOT_IP") != 0U))) {
        aiwb2_state = APP_AIWB2_STATE_WAIT_BOOT_CONNECT;
        aiwb2_wait_from_now(APP_AIWB2_AUTOCONNECT_MS);
        return;
    }

    if ((aiwb2_state != APP_AIWB2_STATE_TRANSPARENT) &&
        (aiwb2_contains(line, "Unknown cmd:") != 0U)) {
        aiwb2_enter_retry_delay();
        return;
    }

    if ((aiwb2_state == APP_AIWB2_STATE_WAIT_PROBE) &&
        ((strcmp(line, "OK") == 0) || (aiwb2_contains(line, "OK") != 0U))) {
        aiwb2_begin_config();
        return;
    }

    if (aiwb2_state == APP_AIWB2_STATE_WAIT_BOOT_CONNECT) {
        if ((aiwb2_contains(line, "connect success") != 0U) ||
            (strcmp(line, ">") == 0)) {
            if (strcmp(line, ">") == 0) {
                aiwb2_enter_transparent();
            } else {
                aiwb2_wait_transparent_ok();
            }
            return;
        }

        if ((aiwb2_contains(line, "+SOCKET:97") != 0U) ||
            (aiwb2_contains(line, "+EVENT:WIFI_DISCONNECT") != 0U) ||
            (strcmp(line, "ERROR") == 0)) {
            aiwb2_enter_retry_delay();
            return;
        }
    }

    if (aiwb2_state == APP_AIWB2_STATE_TRANSPARENT) {
        if ((aiwb2_contains(line, "+SOCKET:97") != 0U) ||
            (aiwb2_contains(line, "+EVENT:WIFI_DISCONNECT") != 0U) ||
            (strcmp(line, "ERROR") == 0)) {
            aiwb2_enter_retry_delay();
        }
    }
}

uint8_t APP_AiWB2_IsTransparent(void)
{
    return (aiwb2_state == APP_AIWB2_STATE_TRANSPARENT) ? 1U : 0U;
}

uint8_t APP_AiWB2_IsControlPayload(const char *line)
{
    if (line == 0) {
        return 0U;
    }

    if ((strcmp(line, "PING") == 0) ||
        (strcmp(line, "MODULES?") == 0) ||
        (strcmp(line, "CAPS?") == 0) ||
        (strcmp(line, "STATUS?") == 0) ||
        (strcmp(line, "CONFIG?") == 0) ||
        (strcmp(line, "FLASH?") == 0) ||
        (strcmp(line, "RTOS?") == 0) ||
        (aiwb2_starts_with(line, "FLASH ") != 0U) ||
        (strcmp(line, "BARO?") == 0) ||
        (strcmp(line, "IMU?") == 0) ||
        (strcmp(line, "PARAM?") == 0) ||
        (strcmp(line, "PID?") == 0) ||
        (strcmp(line, "WIFI?") == 0) ||
        (strcmp(line, "WIFI_EN?") == 0) ||
        (strcmp(line, "SAVE") == 0) ||
        (strcmp(line, "LOAD") == 0) ||
        (strcmp(line, "DEFAULTS") == 0) ||
        (aiwb2_starts_with(line, "REQ ") != 0U) ||
        (aiwb2_starts_with(line, "BARO ") != 0U) ||
        (aiwb2_starts_with(line, "WIFI ") != 0U) ||
        (aiwb2_starts_with(line, "WIFI_EN ") != 0U) ||
        (aiwb2_starts_with(line, "PARAM ") != 0U) ||
        (aiwb2_starts_with(line, "PID ") != 0U) ||
        (aiwb2_starts_with(line, "SERVO ") != 0U) ||
        (aiwb2_starts_with(line, "Servor") != 0U) ||
        (aiwb2_starts_with(line, "PWM") != 0U) ||
        (strcmp(line, "Sensor_Data:1") == 0) ||
        (strcmp(line, "Sensor_Data:0") == 0)) {
        return 1U;
    }

    return 0U;
}

uint8_t APP_AiWB2_ShouldConsumeTransparentLine(const char *line)
{
    if (line == 0) {
        return 0U;
    }

    if ((aiwb2_starts_with(line, "+EVENT:") != 0U) ||
        (aiwb2_starts_with(line, "+SOCKET:") != 0U) ||
        (aiwb2_contains(line, "[Busy]Cmd") != 0U) ||
        (aiwb2_contains(line, "Unknown cmd:") != 0U) ||
        (strcmp(line, "ERROR") == 0) ||
        (aiwb2_contains(line, "connect success") != 0U)) {
        return 1U;
    }

    return 0U;
}

void APP_AiWB2_AssumeTransparent(void)
{
    aiwb2_enter_transparent();
}

static uint8_t aiwb2_build_socket_command(APP_AiWB2_LinkMode mode,
                                          const char *host,
                                          const char *port,
                                          char *buffer,
                                          uint32_t buffer_size)
{
    int written;

    if ((port == 0) || (*port == '\0') || (buffer == 0) || (buffer_size == 0U)) {
        return 0U;
    }

    if ((mode == APP_AIWB2_LINK_TCP_CLIENT) &&
        ((host == 0) || (*host == '\0'))) {
        return 0U;
    }

    if (mode == APP_AIWB2_LINK_TCP_CLIENT) {
        written = snprintf(buffer,
                           buffer_size,
                           "AT+SOCKETAUTOTT=4,%s,%s",
                           host,
                           port);
    } else if (mode == APP_AIWB2_LINK_UDP_SERVER) {
        written = snprintf(buffer,
                           buffer_size,
                           "AT+SOCKETAUTOTT=1,%s",
                           port);
    } else {
        return 0U;
    }

    return ((written >= 0) && ((uint32_t)written < buffer_size)) ? 1U : 0U;
}

uint8_t APP_AiWB2_StartProvision(const char *ssid,
                                 const char *password,
                                 APP_AiWB2_LinkMode mode,
                                 const char *host,
                                 const char *port)
{
    int written;

    if ((ssid == 0) || (password == 0) || (port == 0) ||
        (*ssid == '\0') || (*port == '\0')) {
        return 0U;
    }

    if ((mode == APP_AIWB2_LINK_TCP_CLIENT) &&
        ((host == 0) || (*host == '\0'))) {
        return 0U;
    }

    if ((mode != APP_AIWB2_LINK_UDP_SERVER) &&
        (mode != APP_AIWB2_LINK_TCP_CLIENT)) {
        return 0U;
    }

    (void)snprintf(aiwb2_provision_text[0], sizeof(aiwb2_provision_text[0]), "ATE0");
    (void)snprintf(aiwb2_provision_text[1], sizeof(aiwb2_provision_text[1]), "AT+WMODE=1,1");
    written = snprintf(aiwb2_provision_text[2],
                       sizeof(aiwb2_provision_text[2]),
                       "AT+WJAP=\"%s\",\"%s\"",
                       ssid,
                       password);
    if ((written < 0) || ((uint32_t)written >= sizeof(aiwb2_provision_text[2]))) {
        return 0U;
    }
    (void)snprintf(aiwb2_provision_text[3], sizeof(aiwb2_provision_text[3]), "AT+WAUTOCONN=1");
    (void)snprintf(aiwb2_provision_text[4], sizeof(aiwb2_provision_text[4]), "AT+SOCKETDEL=1");
    (void)written;
    if (aiwb2_build_socket_command(mode,
                                   host,
                                   port,
                                   aiwb2_provision_text[5],
                                   sizeof(aiwb2_provision_text[5])) == 0U) {
        return 0U;
    }
    (void)snprintf(aiwb2_provision_text[6], sizeof(aiwb2_provision_text[6]), "AT+RST");

    aiwb2_provision_commands[0] = (APP_AiWB2Command){ aiwb2_provision_text[0], 1500U, 0U, 0U };
    aiwb2_provision_commands[1] = (APP_AiWB2Command){ aiwb2_provision_text[1], 1500U, 0U, 0U };
    aiwb2_provision_commands[2] = (APP_AiWB2Command){ aiwb2_provision_text[2], 25000U, 0U, 0U };
    aiwb2_provision_commands[3] = (APP_AiWB2Command){ aiwb2_provision_text[3], 1500U, 0U, 0U };
    aiwb2_provision_commands[4] = (APP_AiWB2Command){ aiwb2_provision_text[4], 2000U, 1U, 0U };
    aiwb2_provision_commands[5] = (APP_AiWB2Command){ aiwb2_provision_text[5], 2500U, 0U, 0U };
    aiwb2_provision_commands[6] = (APP_AiWB2Command){ aiwb2_provision_text[6], 8000U, 0U, 1U };
    aiwb2_provision_command_count = 7U;

    if (BSP_AiWB2_IsEnabled() == 0U) {
        BSP_AiWB2_SetEnabled(1U);
    }

    aiwb2_provision_active = 1U;
    aiwb2_power_recycle_active = 0U;
    aiwb2_command_index = 0U;
    aiwb2_probe_escape_used = 0U;
    aiwb2_begin_probe();

    return 1U;
}

uint8_t APP_AiWB2_StartSocketConfig(APP_AiWB2_LinkMode mode,
                                    const char *host,
                                    const char *port)
{
    if (aiwb2_build_socket_command(mode,
                                   host,
                                   port,
                                   aiwb2_provision_text[0],
                                   sizeof(aiwb2_provision_text[0])) == 0U) {
        return 0U;
    }

    (void)snprintf(aiwb2_provision_text[1], sizeof(aiwb2_provision_text[1]), "AT+RST");

    aiwb2_provision_commands[0] = (APP_AiWB2Command){ aiwb2_provision_text[0], 2500U, 0U, 0U };
    aiwb2_provision_commands[1] = (APP_AiWB2Command){ aiwb2_provision_text[1], 8000U, 0U, 1U };
    aiwb2_provision_command_count = 2U;

    if (BSP_AiWB2_IsEnabled() == 0U) {
        BSP_AiWB2_SetEnabled(1U);
    }

    aiwb2_provision_active = 1U;
    aiwb2_power_recycle_active = 0U;
    aiwb2_command_index = 0U;
    aiwb2_probe_escape_used = 0U;
    aiwb2_begin_probe();

    return 1U;
}

uint8_t APP_AiWB2_SendRawCommand(const char *command)
{
    uint32_t now_ms;

    if ((command == 0) || (*command == '\0')) {
        return 0U;
    }

    if (BSP_AiWB2_IsEnabled() == 0U) {
        BSP_AiWB2_SetEnabled(1U);
    }

    now_ms = HAL_GetTick();
    aiwb2_manual_at_deadline_ms = now_ms + APP_AIWB2_MANUAL_AT_GUARD_MS;
    if ((aiwb2_provision_active == 0U) &&
        (aiwb2_state != APP_AIWB2_STATE_TRANSPARENT)) {
        aiwb2_power_recycle_active = 0U;
        aiwb2_deadline_ms = now_ms + APP_AIWB2_BOOT_TIMEOUT_MS;
    }
    aiwb2_send_command(command);
    return 1U;
}

void APP_AiWB2_SendDiagCommands(void)
{
    static const char *const commands[] = {
        "AT",
        "AT+WMODE?",
        "AT+WJAP?",
        "AT+WAUTOCONN?",
        "AT+SOCKETAUTOTT?",
        "AT+SOCKET?",
    };

    for (uint32_t i = 0U; i < (uint32_t)(sizeof(commands) / sizeof(commands[0])); ++i) {
        (void)APP_AiWB2_SendRawCommand(commands[i]);
        HAL_Delay(150U);
    }
}

APP_AiWB2_State APP_AiWB2_GetState(void)
{
    return aiwb2_state;
}

uint32_t APP_AiWB2_GetRetryCount(void)
{
    return aiwb2_retry_count;
}

int32_t APP_AiWB2_GetLastSocketError(void)
{
    return aiwb2_last_socket_error;
}

uint8_t APP_AiWB2_IsPowerRecycleActive(void)
{
    return aiwb2_power_recycle_active;
}

uint32_t APP_AiWB2_GetDeadlineRemainingMs(void)
{
    uint32_t now_ms = HAL_GetTick();

    if ((int32_t)(aiwb2_deadline_ms - now_ms) <= 0) {
        return 0U;
    }

    return aiwb2_deadline_ms - now_ms;
}

uint8_t APP_AiWB2_IsProvisionActive(void)
{
    return aiwb2_provision_active;
}

uint32_t APP_AiWB2_GetCommandIndex(void)
{
    return aiwb2_command_index;
}

uint32_t APP_AiWB2_GetCommandCount(void)
{
    uint32_t count = 0U;

    (void)aiwb2_active_commands(&count);
    return count;
}

const char *APP_AiWB2_GetCurrentCommand(void)
{
    uint32_t count = 0U;
    const APP_AiWB2Command *commands = aiwb2_active_commands(&count);

    if ((commands == 0) || (aiwb2_command_index >= count)) {
        return "";
    }

    return commands[aiwb2_command_index].text;
}
