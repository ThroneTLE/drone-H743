#include "drv_elrs.h"

#include <string.h>

static uint8_t  g_frame[CRSF_MAX_FRAME_SIZE];
static uint8_t  g_frame_index;
static uint8_t  g_frame_len;

static uint16_t g_channels_raw[CRSF_CHANNEL_COUNT];
static uint16_t g_channels_us[CRSF_CHANNEL_COUNT];
static DRV_ELRS_LinkStats g_link_stats;

static uint32_t g_total_frames;
static uint32_t g_crc_errors;
static uint32_t g_length_errors;
static uint32_t g_rc_frames;
static uint32_t g_last_rc_tick;
static uint32_t g_last_fps_tick;
static uint32_t g_last_fps_frames;
static uint16_t g_fps_x10;
static uint8_t  g_rc_updated;

/* ---------- helpers ---------- */

static uint8_t Crsf_IsCommonAddress(uint8_t address)
{
    if (address == 0x00U || address == 0x10U || address == 0x80U)
        return 1U;
    return (address >= 0xC0U) ? 1U : 0U;
}

static uint16_t Crsf_ReadPackedChannel(const uint8_t *payload, uint8_t channel)
{
    uint16_t bit_offset  = (uint16_t)channel * 11U;
    uint16_t byte_offset = bit_offset >> 3;
    uint8_t  shift       = (uint8_t)(bit_offset & 0x07U);
    uint32_t value       = (uint32_t)payload[byte_offset] |
                           ((uint32_t)payload[byte_offset + 1U] << 8U) |
                           ((uint32_t)payload[byte_offset + 2U] << 16U);
    return (uint16_t)((value >> shift) & 0x07FFU);
}

static uint16_t Crsf_ChannelToUs(uint16_t raw)
{
    int32_t centered = (int32_t)raw - 992;
    int32_t us = 1500 + ((centered * 5) / 8);
    if (us < 800)  us = 800;
    if (us > 2200) us = 2200;
    return (uint16_t)us;
}

/* ---------- CRC ---------- */

uint8_t DRV_ELRS_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0xD5U) : (uint8_t)(crc << 1U);
    }
    return crc;
}

uint8_t DRV_ELRS_BuildTelemetry(uint8_t type, const uint8_t *payload,
                                uint8_t payload_len,
                                uint8_t out_frame[CRSF_MAX_FRAME_SIZE])
{
    uint8_t len = (uint8_t)(payload_len + 2U);
    if (len > CRSF_MAX_LEN)
        return 0U;

    out_frame[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;  /* 0xC8 */
    out_frame[1] = len;
    out_frame[2] = type;
    if (payload_len > 0U)
        memcpy(&out_frame[3], payload, payload_len);
    out_frame[3U + payload_len] = DRV_ELRS_Crc8(&out_frame[2], (uint8_t)(payload_len + 1U));
    return (uint8_t)(payload_len + 4U);
}

/* ---------- frame handling ---------- */

static void Crsf_HandleRcChannels(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len != CRSF_RC_PAYLOAD_LEN)
        return;

    for (uint8_t i = 0U; i < CRSF_CHANNEL_COUNT; i++) {
        g_channels_raw[i] = Crsf_ReadPackedChannel(payload, i);
        g_channels_us[i]  = Crsf_ChannelToUs(g_channels_raw[i]);
    }

    g_rc_frames++;
    g_rc_updated = 1U;
}

static void Crsf_HandleLinkStats(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len < 10U)
        return;

    g_link_stats.valid          = 1U;
    g_link_stats.uplink_rssi_1  = payload[0];
    g_link_stats.uplink_rssi_2  = payload[1];
    g_link_stats.uplink_lq      = payload[2];
    g_link_stats.uplink_snr     = (int8_t)payload[3];
    g_link_stats.active_antenna = payload[4];
    g_link_stats.rf_mode        = payload[5];
    g_link_stats.tx_power       = payload[6];
    g_link_stats.downlink_rssi  = payload[7];
    g_link_stats.downlink_lq    = payload[8];
    g_link_stats.downlink_snr   = (int8_t)payload[9];
}

static void Crsf_HandleFrame(const uint8_t *frame, uint8_t total_len)
{
    uint8_t len         = frame[1];
    uint8_t type        = frame[2];
    const uint8_t *payload = &frame[3];
    uint8_t payload_len = (uint8_t)(len - 2U);
    uint8_t crc_rx      = frame[total_len - 1U];
    uint8_t crc_calc    = DRV_ELRS_Crc8(&frame[2], (uint8_t)(len - 1U));

    if (crc_rx != crc_calc) {
        g_crc_errors++;
        return;
    }

    g_total_frames++;

    if (type == CRSF_FRAME_RC_CHANNELS_PACKED)
        Crsf_HandleRcChannels(payload, payload_len);
    else if (type == CRSF_FRAME_LINK_STATISTICS)
        Crsf_HandleLinkStats(payload, payload_len);
}

/* ---------- byte-level parser ---------- */

uint8_t DRV_ELRS_ProcessByte(uint8_t byte)
{
    if (g_frame_index == 0U) {
        if (Crsf_IsCommonAddress(byte))
            g_frame[g_frame_index++] = byte;
        return 0U;
    }

    if (g_frame_index == 1U) {
        if (byte < CRSF_MIN_LEN || byte > CRSF_MAX_LEN) {
            g_length_errors++;
            g_frame_index = 0U;
            if (Crsf_IsCommonAddress(byte))
                g_frame[g_frame_index++] = byte;
            return 0U;
        }
        g_frame_len = byte;
        g_frame[g_frame_index++] = byte;
        return 0U;
    }

    g_frame[g_frame_index++] = byte;
    if (g_frame_index >= (uint8_t)(g_frame_len + 2U)) {
        uint8_t handled = (g_frame[2] == CRSF_FRAME_RC_CHANNELS_PACKED ||
                           g_frame[2] == CRSF_FRAME_LINK_STATISTICS) ? 1U : 0U;
        Crsf_HandleFrame(g_frame, g_frame_index);
        g_frame_index = 0U;
        return handled;
    }
    return 0U;
}

/* ---------- FPS ---------- */

static void UpdateFps(uint32_t now)
{
    uint32_t dt = now - g_last_fps_tick;
    if (dt < 1000U) return;

    uint32_t frames = g_rc_frames - g_last_fps_frames;
    g_fps_x10 = (uint16_t)((frames * 10000U + (dt / 2U)) / dt);
    g_last_fps_frames = g_rc_frames;
    g_last_fps_tick = now;
}

/* ---------- public API ---------- */

void DRV_ELRS_Init(void)
{
    g_frame_index = 0U;
    g_frame_len   = 0U;
    g_total_frames = 0U;
    g_crc_errors   = 0U;
    g_length_errors = 0U;
    g_rc_frames    = 0U;
    g_rc_updated   = 0U;
    g_fps_x10      = 0U;
    g_last_fps_tick = 0U;
    g_last_fps_frames = 0U;
    memset(&g_link_stats, 0, sizeof(g_link_stats));

    for (uint8_t i = 0U; i < CRSF_CHANNEL_COUNT; i++) {
        g_channels_raw[i] = 992U;
        g_channels_us[i]  = 1500U;
    }
}

void DRV_ELRS_GetChannels(uint16_t raw_out[CRSF_CHANNEL_COUNT],
                          uint16_t us_out[CRSF_CHANNEL_COUNT])
{
    if (raw_out != NULL)
        memcpy(raw_out, g_channels_raw, sizeof(g_channels_raw));
    if (us_out != NULL)
        memcpy(us_out, g_channels_us, sizeof(g_channels_us));

    uint32_t now = g_last_rc_tick;
    if (now == 0U) now = 1U;
    UpdateFps(now);
}

const DRV_ELRS_LinkStats *DRV_ELRS_GetLinkStats(void)
{
    return &g_link_stats;
}

uint8_t DRV_ELRS_IsRcUpdated(void)
{
    return g_rc_updated;
}

void DRV_ELRS_ClearRcUpdated(void)
{
    g_rc_updated = 0U;
}

uint32_t DRV_ELRS_GetRcFrames(void)       { return g_rc_frames; }
uint32_t DRV_ELRS_GetCrcErrors(void)      { return g_crc_errors; }
uint32_t DRV_ELRS_GetLengthErrors(void)   { return g_length_errors; }
uint32_t DRV_ELRS_GetFpsX10(void)         { return g_fps_x10; }
