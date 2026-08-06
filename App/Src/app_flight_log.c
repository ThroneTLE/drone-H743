#include "app_flight_log.h"

#include "app_flash_service.h"
#include "app_messages.h"
#include "app_tasks.h"
#include "app_uart.h"
#include "app_usb_cdc.h"
#include "svc_timestamp.h"

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_FLIGHT_LOG_SECTOR_MAGIC       0x31534C46UL /* FLS1 */
#define APP_FLIGHT_LOG_RECORD_MAGIC       0x31524C46UL /* FLR1 */
#define APP_FLIGHT_LOG_EXPORT_BLOCK_MAGIC 0x31424C46UL /* FLB1 */
#define APP_FLIGHT_LOG_VERSION            7U
#define APP_FLIGHT_LOG_EXPORT_VERSION     1U
#define APP_FLIGHT_LOG_REGION_SIZE \
    (APP_FLIGHT_LOG_REGION_END_EXCL - APP_FLIGHT_LOG_REGION_START)
#define APP_FLIGHT_LOG_SECTOR_COUNT \
    (APP_FLIGHT_LOG_REGION_SIZE / APP_FLASH_SERVICE_SECTOR_SIZE)
#define APP_FLIGHT_LOG_QUEUE_CAPACITY     32U
#define APP_FLIGHT_LOG_WRITE_BATCH_RECORDS 4U
#define APP_FLIGHT_LOG_UART_EXPORT_PAYLOAD_MAX 48U
#define APP_FLIGHT_LOG_USB_EXPORT_PAYLOAD_MAX 1024U
#define APP_FLIGHT_LOG_EXPORT_FLAG_LAST   0x0001U
#define APP_FLIGHT_LOG_TESTFILL_MAX_SECTORS 64U
#define APP_FLIGHT_LOG_EXPORT_BLOCK_GAP_MS 40U
#define APP_FLIGHT_LOG_USB_TX_TIMEOUT_MS 200U
#define APP_FLIGHT_LOG_RAM_D1_NOINIT \
    __attribute__((section(".ram_d1_noinit"), aligned(32)))

typedef enum {
    APP_FLIGHT_LOG_EXPORT_UART_TEXT = 0,
    APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY = 1,
} APP_FlightLogExportTransport;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t sector_size;
    uint32_t record_size;
    uint32_t session_id;
    uint32_t sector_seq;
    uint32_t sector_index;
    uint32_t log_rate_hz;
    uint32_t region_start;
    uint32_t region_end_excl;
    uint64_t created_us;
    uint32_t params_size;
    uint32_t header_crc32;
    DRV_COAX_CTRL_Params params;
    uint8_t reserved[84];
} APP_FlightLogSectorHeader;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t dropped_records;
    uint64_t timestamp_us;
    uint32_t tick_ms;
    uint32_t imu_sequence;
    DRV_IMU_RawData imu_raw;
    DRV_IMU_ScaledData imu;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint16_t rc_channels[8];
    uint16_t throttle_us;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
    uint16_t motor_upper_us;
    uint16_t motor_lower_us;
    uint16_t servo_alpha_feedback_us;
    uint16_t servo_beta_feedback_us;
    uint16_t servo_alpha_feedback_age_ms;
    uint16_t servo_beta_feedback_age_ms;
    uint16_t servo_alpha_feedback_sequence;
    uint16_t servo_beta_feedback_sequence;
    uint8_t servo_feedback_valid_mask;
    uint8_t servo_feedback_reserved[3];
    uint8_t rc_armed;
    uint8_t rc_link_ok;
    uint8_t throttle_over_20;
    uint8_t imu_valid;
    uint8_t motor_output_reason;
    uint8_t rc_link_seen;
    uint8_t arm_switch_high;
    uint8_t arm_throttle_low;
    uint8_t arm_switch_seen_low;
    uint8_t arm_switch_prev_high;
    uint8_t imu_fault_latched;
    uint8_t imu_fault_reason;
    float acc_nav_m_s2[3];
    float vel_est_m_s[3];
    float vel_ref_m_s[2];
    float vel_err_m_s[2];
    float vel_pid_out_m_s2[2];
    float vel_pid_p_m_s2[2];
    float vel_pid_i_m_s2[2];
    float vel_pid_d_m_s2[2];
    float vel_loop_active;
    DRV_COAX_CTRL_Debug ctrl_debug;
    float z_ref_m;
    APP_IdentAttLog ident_att;
    uint16_t servo_alpha_sent_us;
    uint16_t servo_beta_sent_us;
    int16_t flow_raw_x;
    int16_t flow_raw_y;
    uint16_t flow_sample_age_ms;
    uint16_t flow_height_age_ms;
    uint8_t flow_quality;
    uint8_t flow_valid;
    uint8_t flow_velocity_valid;
    uint8_t flow_height_valid;
    float flow_height_raw_m;
    float flow_height_m;
    float flow_sensor_velocity_m_s[2];
    float flow_optical_rot_comp_m_s[2];
    float flow_offset_rot_comp_m_s[2];
    float flow_corrected_velocity_m_s[2];
    uint32_t servo_move_attempt_count;
    uint32_t servo_move_sent_count;
    uint32_t servo_move_busy_count;
    uint32_t servo_move_error_count;
    uint32_t servo_feedback_request_count;
    uint32_t servo_feedback_response_count;
    uint32_t servo_feedback_timeout_count;
    uint32_t servo_feedback_parse_error_count;
    uint32_t servo_feedback_uart_error_count;
    uint32_t servo_feedback_busy_count;
    uint32_t record_crc32;
} APP_FlightLogRecord;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t seq;
    uint32_t offset;
    uint16_t length;
    uint16_t flags;
    uint32_t payload_crc32;
} APP_FlightLogExportBlockHeader;

_Static_assert(sizeof(APP_FlightLogSectorHeader) == APP_FLIGHT_LOG_SECTOR_HEADER_SIZE,
               "flight log sector header must stay 256 bytes");
_Static_assert(sizeof(APP_FlightLogRecord) == 528U,
               "flight log record must match tools/flight_log_receive.py");
_Static_assert(sizeof(APP_FlightLogExportBlockHeader) == 24U,
               "flight log export header must match tools/flight_log_receive.py");
_Static_assert((APP_FLIGHT_LOG_REGION_START % APP_FLASH_SERVICE_SECTOR_SIZE) == 0U,
               "flight log region start must be sector aligned");
_Static_assert((APP_FLIGHT_LOG_REGION_END_EXCL % APP_FLASH_SERVICE_SECTOR_SIZE) == 0U,
               "flight log region end must be sector aligned");
_Static_assert(APP_FLIGHT_LOG_REGION_END_EXCL <=
               (APP_FLASH_SERVICE_SIZE_BYTES - (4UL * APP_FLASH_SERVICE_SECTOR_SIZE)),
               "flight log must not overlap the last four reserved sectors");
_Static_assert((64U + (APP_FLIGHT_LOG_UART_EXPORT_PAYLOAD_MAX * 2U)) <=
               APP_UART_TX_TEXT_SIZE,
               "UART text flight log block must fit the USART1 TX queue frame");

static APP_FlightLogStatus flight_log_status;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogRecord flight_log_queue[APP_FLIGHT_LOG_QUEUE_CAPACITY];
static uint32_t flight_log_queue_head;
static uint32_t flight_log_queue_tail;
static uint32_t flight_log_queue_count;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogRecord flight_log_batch[APP_FLIGHT_LOG_WRITE_BATCH_RECORDS];
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogSectorHeader flight_log_test_header;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogSectorHeader flight_log_test_verify_header;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogSnapshot flight_log_test_snapshot;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogRecord flight_log_test_record;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static APP_FlightLogRecord flight_log_test_verify_record;
APP_FLIGHT_LOG_RAM_D1_NOINIT
static uint16_t flight_log_sector_order[APP_FLIGHT_LOG_SECTOR_COUNT];
APP_FLIGHT_LOG_RAM_D1_NOINIT
static uint32_t flight_log_sector_order_seq[APP_FLIGHT_LOG_SECTOR_COUNT];
static uint32_t flight_log_next_sector_index;
static uint32_t flight_log_current_sector_index;
static uint32_t flight_log_sector_write_offset;
static uint32_t flight_log_record_sequence;
static uint32_t flight_log_export_sector_pos;
static uint32_t flight_log_export_sector_offset;
static uint32_t flight_log_export_seq;
static uint32_t flight_log_export_next_ms;
static APP_FlightLogExportTransport flight_log_export_transport;
static uint8_t flight_log_flush_requested;
static uint8_t flight_log_export_pending;
static uint8_t flight_log_export_restore_vofa;
static uint8_t flight_log_export_cancel_requested;
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t flight_log_export_payload[APP_FLIGHT_LOG_USB_EXPORT_PAYLOAD_MAX];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t flight_log_export_usb_frame[sizeof(APP_FlightLogExportBlockHeader) +
                                           APP_FLIGHT_LOG_USB_EXPORT_PAYLOAD_MAX];
APP_FLIGHT_LOG_RAM_D1_NOINIT
static char flight_log_export_text_frame[APP_UART_TX_TEXT_SIZE];

static uint32_t flight_log_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    while (length-- > 0U) {
        crc ^= (uint32_t)(*data++);
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            uint32_t mask = 0UL - (crc & 1UL);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }

    return ~crc;
}

static uint32_t flight_log_sector_address(uint32_t sector_index)
{
    return APP_FLIGHT_LOG_REGION_START +
           (sector_index * APP_FLASH_SERVICE_SECTOR_SIZE);
}

static uint8_t flight_log_queue_message(const uint8_t *data, uint16_t length)
{
    APP_UART_TxMessage tx_message;

    if ((data == NULL) || (length == 0U) || (length > APP_UART_TX_TEXT_SIZE) ||
        (uartTxQueueHandle == NULL)) {
        return 0U;
    }

    tx_message.function = APP_UART_TX_FUNCTION_TEXT;
    tx_message.length = length;
    memcpy(tx_message.text, data, length);

    if (osMessageQueuePut(uartTxQueueHandle, &tx_message, 0U, 0U) != osOK) {
        return 0U;
    }

    APP_UART_NotifyTxPending();
    return 1U;
}

static uint16_t flight_log_export_payload_max(void)
{
    return (flight_log_export_transport == APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY) ?
        APP_FLIGHT_LOG_USB_EXPORT_PAYLOAD_MAX :
        APP_FLIGHT_LOG_UART_EXPORT_PAYLOAD_MAX;
}

static uint8_t flight_log_send_message(const uint8_t *data, uint16_t length)
{
    if ((flight_log_export_transport == APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY) &&
        (APP_USB_CDC_IsReady() != 0U)) {
        return APP_USB_CDC_Write(data, length, APP_FLIGHT_LOG_USB_TX_TIMEOUT_MS);
    }

    return flight_log_queue_message(data, length);
}

static uint8_t flight_log_queue_printf(const char *format, ...)
{
    char line[APP_UART_TX_TEXT_SIZE];
    va_list args;
    int written;

    if (format == NULL) {
        return 0U;
    }

    va_start(args, format);
    written = vsnprintf(line, sizeof(line), format, args);
    va_end(args);

    if (written <= 0) {
        return 0U;
    }
    if ((uint32_t)written >= sizeof(line)) {
        written = (int)(sizeof(line) - 1U);
    }

    return flight_log_send_message((const uint8_t *)line, (uint16_t)written);
}

static char flight_log_hex_digit(uint8_t value)
{
    value &= 0x0FU;
    return (char)((value < 10U) ? ('0' + value) : ('A' + (value - 10U)));
}

static uint8_t flight_log_whiten_byte(uint32_t offset)
{
    uint32_t x = offset + 0xA5A5A5A5UL;

    x ^= (x >> 7U);
    x *= 0x45D9F3BUL;
    x ^= (x >> 11U);
    return (uint8_t)(x | 0x01U);
}

static uint16_t flight_log_hex_encode(char *dst,
                                      uint16_t dst_size,
                                      const uint8_t *src,
                                      uint16_t src_size,
                                      uint32_t src_offset)
{
    uint16_t needed = (uint16_t)(src_size * 2U);

    if ((dst == NULL) || (src == NULL) || (dst_size < needed)) {
        return 0U;
    }

    for (uint16_t index = 0U; index < src_size; ++index) {
        uint8_t encoded = (uint8_t)(src[index] ^
                                    flight_log_whiten_byte(src_offset + index));

        dst[index * 2U] = flight_log_hex_digit((uint8_t)(encoded >> 4U));
        dst[(index * 2U) + 1U] = flight_log_hex_digit(encoded);
    }

    return needed;
}

static uint8_t flight_log_sector_header_valid(APP_FlightLogSectorHeader *header)
{
    uint32_t expected_crc;
    uint32_t saved_crc;

    if (header == NULL) {
        return 0U;
    }
    if ((header->magic != APP_FLIGHT_LOG_SECTOR_MAGIC) ||
        (header->version != APP_FLIGHT_LOG_VERSION) ||
        (header->header_size != APP_FLIGHT_LOG_SECTOR_HEADER_SIZE) ||
        (header->sector_size != APP_FLASH_SERVICE_SECTOR_SIZE) ||
        (header->record_size != sizeof(APP_FlightLogRecord)) ||
        (header->region_start != APP_FLIGHT_LOG_REGION_START) ||
        (header->region_end_excl != APP_FLIGHT_LOG_REGION_END_EXCL)) {
        return 0U;
    }

    saved_crc = header->header_crc32;
    header->header_crc32 = 0U;
    expected_crc = flight_log_crc32((const uint8_t *)header, sizeof(*header));
    header->header_crc32 = saved_crc;

    return (expected_crc == saved_crc) ? 1U : 0U;
}

static void flight_log_order_insert(uint16_t sector_index, uint32_t sector_seq)
{
    uint32_t pos = flight_log_status.used_sectors;

    if (pos >= APP_FLIGHT_LOG_SECTOR_COUNT) {
        return;
    }

    while ((pos > 0U) && (flight_log_sector_order_seq[pos - 1U] > sector_seq)) {
        flight_log_sector_order[pos] = flight_log_sector_order[pos - 1U];
        flight_log_sector_order_seq[pos] = flight_log_sector_order_seq[pos - 1U];
        --pos;
    }

    flight_log_sector_order[pos] = sector_index;
    flight_log_sector_order_seq[pos] = sector_seq;
    flight_log_status.used_sectors++;
}

static void flight_log_order_remove_sector(uint32_t sector_index)
{
    for (uint32_t i = 0U; i < flight_log_status.used_sectors; ++i) {
        if (flight_log_sector_order[i] == sector_index) {
            for (uint32_t j = i; (j + 1U) < flight_log_status.used_sectors; ++j) {
                flight_log_sector_order[j] = flight_log_sector_order[j + 1U];
                flight_log_sector_order_seq[j] = flight_log_sector_order_seq[j + 1U];
            }
            flight_log_status.used_sectors--;
            return;
        }
    }
}

static void flight_log_scan_existing(void)
{
    APP_FlightLogSectorHeader header;
    uint32_t max_seq = 0U;
    uint32_t newest_sector = 0U;
    uint8_t have_sector = 0U;

    flight_log_status.used_sectors = 0U;
    memset(flight_log_sector_order, 0, sizeof(flight_log_sector_order));
    memset(flight_log_sector_order_seq, 0, sizeof(flight_log_sector_order_seq));

    for (uint32_t sector = 0U; sector < APP_FLIGHT_LOG_SECTOR_COUNT; ++sector) {
        APP_FlashService_Status st =
            APP_FlashService_ReadData(flight_log_sector_address(sector),
                                      (uint8_t *)&header,
                                      sizeof(header));

        if (st != APP_FLASH_SERVICE_OK) {
            flight_log_status.last_flash_status = (uint32_t)st;
            continue;
        }

        if (flight_log_sector_header_valid(&header) != 0U) {
            flight_log_order_insert((uint16_t)sector, header.sector_seq);
            if ((have_sector == 0U) || (header.sector_seq >= max_seq)) {
                max_seq = header.sector_seq;
                newest_sector = sector;
                have_sector = 1U;
            }
        }
    }

    flight_log_status.sector_seq = have_sector ? (max_seq + 1U) : 1U;
    flight_log_next_sector_index =
        have_sector ? ((newest_sector + 1U) % APP_FLIGHT_LOG_SECTOR_COUNT) : 0U;
    flight_log_current_sector_index = 0U;
    flight_log_sector_write_offset = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    flight_log_status.sector_open = 0U;
}

static void flight_log_fill_sector_header(APP_FlightLogSectorHeader *header,
                                          uint32_t sector_index)
{
    memset(header, 0, sizeof(*header));
    header->magic = APP_FLIGHT_LOG_SECTOR_MAGIC;
    header->version = APP_FLIGHT_LOG_VERSION;
    header->header_size = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    header->sector_size = APP_FLASH_SERVICE_SECTOR_SIZE;
    header->record_size = sizeof(APP_FlightLogRecord);
    header->session_id = flight_log_status.session_id;
    header->sector_seq = flight_log_status.sector_seq;
    header->sector_index = sector_index;
    header->log_rate_hz = APP_FLIGHT_LOG_RATE_HZ;
    header->region_start = APP_FLIGHT_LOG_REGION_START;
    header->region_end_excl = APP_FLIGHT_LOG_REGION_END_EXCL;
    header->created_us = SVC_Timestamp_Us();
    header->params_size = sizeof(header->params);
    {
        DRV_COAX_CTRL_Params params;

        DRV_COAX_CTRL_GetParams(&params);
        memcpy(&header->params, &params, sizeof(params));
    }
    header->header_crc32 = 0U;
    header->header_crc32 = flight_log_crc32((const uint8_t *)header,
                                            sizeof(*header));
}

static uint8_t flight_log_open_sector(void)
{
    APP_FlightLogSectorHeader header;
    uint32_t sector = flight_log_next_sector_index;
    uint32_t address = flight_log_sector_address(sector);
    APP_FlashService_Status st;

    st = APP_FlashService_EraseSector(address);
    flight_log_status.last_flash_status = (uint32_t)st;
    if (st != APP_FLASH_SERVICE_OK) {
        return 0U;
    }

    flight_log_fill_sector_header(&header, sector);
    st = APP_FlashService_WriteData(address, (const uint8_t *)&header, sizeof(header));
    flight_log_status.last_flash_status = (uint32_t)st;
    if (st != APP_FLASH_SERVICE_OK) {
        return 0U;
    }

    flight_log_order_remove_sector(sector);
    flight_log_order_insert((uint16_t)sector, flight_log_status.sector_seq);
    flight_log_status.sector_seq++;
    flight_log_current_sector_index = sector;
    flight_log_next_sector_index = (sector + 1U) % APP_FLIGHT_LOG_SECTOR_COUNT;
    flight_log_sector_write_offset = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    flight_log_status.sector_open = 1U;
    return 1U;
}

static void flight_log_record_from_snapshot(APP_FlightLogRecord *record,
                                            const APP_FlightLogSnapshot *snapshot)
{
    memset(record, 0, sizeof(*record));
    record->magic = APP_FLIGHT_LOG_RECORD_MAGIC;
    record->version = APP_FLIGHT_LOG_VERSION;
    record->size = sizeof(*record);
    record->sequence = ++flight_log_record_sequence;
    record->dropped_records = flight_log_status.dropped_records;
    record->timestamp_us = snapshot->timestamp_us;
    record->tick_ms = snapshot->tick_ms;
    record->imu_sequence = snapshot->imu_sequence;
    record->imu_raw = snapshot->imu_raw;
    record->imu = snapshot->imu;
    record->roll_deg = snapshot->roll_deg;
    record->pitch_deg = snapshot->pitch_deg;
    record->yaw_deg = snapshot->yaw_deg;
    memcpy(record->rc_channels, snapshot->rc_channels, sizeof(record->rc_channels));
    record->throttle_us = snapshot->throttle_us;
    record->servo_alpha_us = snapshot->servo_alpha_us;
    record->servo_beta_us = snapshot->servo_beta_us;
    record->motor_upper_us = snapshot->motor_upper_us;
    record->motor_lower_us = snapshot->motor_lower_us;
    record->servo_alpha_feedback_us = snapshot->servo_alpha_feedback_us;
    record->servo_beta_feedback_us = snapshot->servo_beta_feedback_us;
    record->servo_alpha_feedback_age_ms =
        snapshot->servo_alpha_feedback_age_ms;
    record->servo_beta_feedback_age_ms =
        snapshot->servo_beta_feedback_age_ms;
    record->servo_alpha_feedback_sequence =
        snapshot->servo_alpha_feedback_sequence;
    record->servo_beta_feedback_sequence =
        snapshot->servo_beta_feedback_sequence;
    record->servo_feedback_valid_mask = snapshot->servo_feedback_valid_mask;
    record->rc_armed = snapshot->rc_armed;
    record->rc_link_ok = snapshot->rc_link_ok;
    record->throttle_over_20 = snapshot->throttle_over_20;
    record->imu_valid = snapshot->imu_valid;
    record->motor_output_reason = snapshot->motor_output_reason;
    record->rc_link_seen = snapshot->rc_link_seen;
    record->arm_switch_high = snapshot->arm_switch_high;
    record->arm_throttle_low = snapshot->arm_throttle_low;
    record->arm_switch_seen_low = snapshot->arm_switch_seen_low;
    record->arm_switch_prev_high = snapshot->arm_switch_prev_high;
    record->imu_fault_latched = snapshot->imu_fault_latched;
    record->imu_fault_reason = snapshot->imu_fault_reason;
    memcpy(record->acc_nav_m_s2, snapshot->acc_nav_m_s2, sizeof(record->acc_nav_m_s2));
    memcpy(record->vel_est_m_s, snapshot->vel_est_m_s, sizeof(record->vel_est_m_s));
    memcpy(record->vel_ref_m_s, snapshot->vel_ref_m_s, sizeof(record->vel_ref_m_s));
    memcpy(record->vel_err_m_s, snapshot->vel_err_m_s, sizeof(record->vel_err_m_s));
    memcpy(record->vel_pid_out_m_s2, snapshot->vel_pid_out_m_s2,
           sizeof(record->vel_pid_out_m_s2));
    memcpy(record->vel_pid_p_m_s2, snapshot->vel_pid_p_m_s2,
           sizeof(record->vel_pid_p_m_s2));
    memcpy(record->vel_pid_i_m_s2, snapshot->vel_pid_i_m_s2,
           sizeof(record->vel_pid_i_m_s2));
    memcpy(record->vel_pid_d_m_s2, snapshot->vel_pid_d_m_s2,
           sizeof(record->vel_pid_d_m_s2));
    record->vel_loop_active = snapshot->vel_loop_active;
    record->ctrl_debug = snapshot->ctrl_debug;
    record->z_ref_m = snapshot->z_ref_m;
    record->ident_att = snapshot->ident_att;
    record->servo_alpha_sent_us = snapshot->servo_alpha_sent_us;
    record->servo_beta_sent_us = snapshot->servo_beta_sent_us;
    record->flow_raw_x = snapshot->flow_raw_x;
    record->flow_raw_y = snapshot->flow_raw_y;
    record->flow_sample_age_ms = snapshot->flow_sample_age_ms;
    record->flow_height_age_ms = snapshot->flow_height_age_ms;
    record->flow_quality = snapshot->flow_quality;
    record->flow_valid = snapshot->flow_valid;
    record->flow_velocity_valid = snapshot->flow_velocity_valid;
    record->flow_height_valid = snapshot->flow_height_valid;
    record->flow_height_raw_m = snapshot->flow_height_raw_m;
    record->flow_height_m = snapshot->flow_height_m;
    memcpy(record->flow_sensor_velocity_m_s,
           snapshot->flow_sensor_velocity_m_s,
           sizeof(record->flow_sensor_velocity_m_s));
    memcpy(record->flow_optical_rot_comp_m_s,
           snapshot->flow_optical_rot_comp_m_s,
           sizeof(record->flow_optical_rot_comp_m_s));
    memcpy(record->flow_offset_rot_comp_m_s,
           snapshot->flow_offset_rot_comp_m_s,
           sizeof(record->flow_offset_rot_comp_m_s));
    memcpy(record->flow_corrected_velocity_m_s,
           snapshot->flow_corrected_velocity_m_s,
           sizeof(record->flow_corrected_velocity_m_s));
    record->servo_move_attempt_count = snapshot->servo_move_attempt_count;
    record->servo_move_sent_count = snapshot->servo_move_sent_count;
    record->servo_move_busy_count = snapshot->servo_move_busy_count;
    record->servo_move_error_count = snapshot->servo_move_error_count;
    record->servo_feedback_request_count = snapshot->servo_feedback_request_count;
    record->servo_feedback_response_count = snapshot->servo_feedback_response_count;
    record->servo_feedback_timeout_count = snapshot->servo_feedback_timeout_count;
    record->servo_feedback_parse_error_count =
        snapshot->servo_feedback_parse_error_count;
    record->servo_feedback_uart_error_count =
        snapshot->servo_feedback_uart_error_count;
    record->servo_feedback_busy_count = snapshot->servo_feedback_busy_count;
    record->record_crc32 = 0U;
    record->record_crc32 = flight_log_crc32((const uint8_t *)record, sizeof(*record));
}

static void flight_log_clear_record_queue(void)
{
    taskENTER_CRITICAL();
    flight_log_queue_head = 0U;
    flight_log_queue_tail = 0U;
    flight_log_queue_count = 0U;
    flight_log_status.buffered_records = 0U;
    taskEXIT_CRITICAL();
}

static uint32_t flight_log_copy_batch(uint32_t max_records)
{
    uint32_t count = 0U;

    taskENTER_CRITICAL();
    while ((count < max_records) && (flight_log_queue_count > 0U)) {
        flight_log_batch[count] = flight_log_queue[flight_log_queue_tail];
        flight_log_queue_tail =
            (flight_log_queue_tail + 1U) % APP_FLIGHT_LOG_QUEUE_CAPACITY;
        flight_log_queue_count--;
        count++;
    }
    flight_log_status.buffered_records = flight_log_queue_count;
    taskEXIT_CRITICAL();

    return count;
}

static uint8_t flight_log_write_records(void)
{
    uint32_t available_records;
    uint32_t batch_records;
    uint32_t write_address;
    uint32_t write_bytes;
    APP_FlashService_Status st;

    if (flight_log_queue_count == 0U) {
        flight_log_flush_requested = 0U;
        return 1U;
    }

    if ((flight_log_status.recording != 0U) &&
        (flight_log_flush_requested == 0U) &&
        (flight_log_queue_count < APP_FLIGHT_LOG_WRITE_BATCH_RECORDS)) {
        return 1U;
    }

    if ((flight_log_status.sector_open == 0U) ||
        ((flight_log_sector_write_offset + sizeof(APP_FlightLogRecord)) >
         APP_FLASH_SERVICE_SECTOR_SIZE)) {
        if (flight_log_open_sector() == 0U) {
            return 0U;
        }
    }

    available_records =
        (APP_FLASH_SERVICE_SECTOR_SIZE - flight_log_sector_write_offset) /
        sizeof(APP_FlightLogRecord);
    if (available_records == 0U) {
        flight_log_status.sector_open = 0U;
        return 1U;
    }

    batch_records = APP_FLIGHT_LOG_WRITE_BATCH_RECORDS;
    if (batch_records > available_records) {
        batch_records = available_records;
    }
    if (batch_records > flight_log_queue_count) {
        batch_records = flight_log_queue_count;
    }

    batch_records = flight_log_copy_batch(batch_records);
    if (batch_records == 0U) {
        return 1U;
    }

    write_bytes = batch_records * sizeof(APP_FlightLogRecord);
    write_address = flight_log_sector_address(flight_log_current_sector_index) +
                    flight_log_sector_write_offset;
    st = APP_FlashService_WriteData(write_address,
                                    (const uint8_t *)flight_log_batch,
                                    write_bytes);
    flight_log_status.last_flash_status = (uint32_t)st;
    if (st != APP_FLASH_SERVICE_OK) {
        flight_log_status.dropped_records += flight_log_queue_count + batch_records;
        flight_log_status.recording = 0U;
        flight_log_status.sector_open = 0U;
        flight_log_clear_record_queue();
        return 0U;
    }

    flight_log_sector_write_offset += write_bytes;
    flight_log_status.total_records += batch_records;
    if ((flight_log_sector_write_offset + sizeof(APP_FlightLogRecord)) >
        APP_FLASH_SERVICE_SECTOR_SIZE) {
        flight_log_status.sector_open = 0U;
    }

    return 1U;
}

static uint32_t flight_log_used_bytes(void)
{
    uint32_t used_sectors = flight_log_status.used_sectors;

    if (used_sectors == 0U) {
        return 0U;
    }

    if (flight_log_status.sector_open != 0U) {
        return ((used_sectors - 1U) * APP_FLASH_SERVICE_SECTOR_SIZE) +
               flight_log_sector_write_offset;
    }

    return used_sectors * APP_FLASH_SERVICE_SECTOR_SIZE;
}

static uint8_t flight_log_export_finish(const char *reason)
{
    if (reason == NULL) {
        reason = "done";
    }

    if (flight_log_queue_printf("FLOG END reason=%s sent=%lu total=%lu\r\n",
                                reason,
                                (unsigned long)flight_log_status.export_bytes_sent,
                                (unsigned long)flight_log_status.export_total_bytes) == 0U) {
        return 0U;
    }
    vofaStreamActive = flight_log_export_restore_vofa;
    flight_log_status.export_active = 0U;
    flight_log_export_pending = 0U;
    flight_log_status.export_pending = 0U;
    flight_log_export_cancel_requested = 0U;
    return 1U;
}

static APP_FlightLogCommandStatus flight_log_start_export_from_background(void)
{
    flight_log_status.used_bytes = flight_log_used_bytes();
    flight_log_status.export_total_bytes =
        flight_log_status.used_sectors * APP_FLASH_SERVICE_SECTOR_SIZE;
    flight_log_status.export_bytes_sent = 0U;
    flight_log_export_sector_pos = 0U;
    flight_log_export_sector_offset = 0U;
    flight_log_export_seq = 0U;
    flight_log_export_next_ms = 0U;
    flight_log_export_cancel_requested = 0U;
    vofaStreamActive = 0U;

    if (flight_log_queue_printf("FLOG BEGIN version=%u block_magic=0x%08lX "
                                "transport=%s encoding=%s total=%lu "
                                "sectors=%lu sector_size=%u header_size=%u "
                                "record_size=%u log_rate=%u baud=%u session=%lu payload=%u\r\n",
                                (unsigned int)APP_FLIGHT_LOG_EXPORT_VERSION,
                                (unsigned long)APP_FLIGHT_LOG_EXPORT_BLOCK_MAGIC,
                                (flight_log_export_transport ==
                                 APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY) ? "usbcdc" : "uart",
                                (flight_log_export_transport ==
                                 APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY) ? "binary" : "xorhex",
                                (unsigned long)flight_log_status.export_total_bytes,
                                (unsigned long)flight_log_status.used_sectors,
                                (unsigned int)APP_FLASH_SERVICE_SECTOR_SIZE,
                                (unsigned int)APP_FLIGHT_LOG_SECTOR_HEADER_SIZE,
                                (unsigned int)sizeof(APP_FlightLogRecord),
                                (unsigned int)APP_FLIGHT_LOG_RATE_HZ,
                                (unsigned int)APP_FLIGHT_LOG_EXPORT_BAUD,
                                (unsigned long)flight_log_status.session_id,
                                (unsigned int)flight_log_export_payload_max()) == 0U) {
        return APP_FLIGHT_LOG_CMD_BUSY;
    }

    flight_log_status.export_active = 1U;
    flight_log_export_pending = 0U;
    flight_log_status.export_pending = 0U;
    return APP_FLIGHT_LOG_CMD_OK;
}

static void flight_log_export_step(void)
{
    uint8_t *payload = flight_log_export_payload;
    char *frame = flight_log_export_text_frame;
    uint32_t remaining;
    uint32_t physical_sector;
    uint32_t address;
    uint32_t payload_crc;
    uint16_t flags;
    uint16_t payload_max;
    uint16_t chunk;
    uint16_t used;
    int written;
    APP_FlashService_Status st;

    if (flight_log_status.export_active == 0U) {
        return;
    }

    if (flight_log_export_cancel_requested != 0U) {
        (void)flight_log_export_finish("cancel");
        return;
    }

    if (flight_log_status.export_bytes_sent >=
        flight_log_status.export_total_bytes) {
        (void)flight_log_export_finish("done");
        return;
    }

    if ((flight_log_export_transport == APP_FLIGHT_LOG_EXPORT_UART_TEXT) &&
        (uartTxQueueHandle != NULL) &&
        (osMessageQueueGetSpace(uartTxQueueHandle) == 0U)) {
        return;
    }
    if ((flight_log_export_next_ms != 0U) &&
        ((int32_t)(HAL_GetTick() - flight_log_export_next_ms) < 0)) {
        return;
    }

    while (flight_log_export_sector_offset >= APP_FLASH_SERVICE_SECTOR_SIZE) {
        flight_log_export_sector_offset -= APP_FLASH_SERVICE_SECTOR_SIZE;
        flight_log_export_sector_pos++;
    }

    if (flight_log_export_sector_pos >= flight_log_status.used_sectors) {
        (void)flight_log_export_finish("short");
        return;
    }

    physical_sector = flight_log_sector_order[flight_log_export_sector_pos];
    remaining = flight_log_status.export_total_bytes -
                flight_log_status.export_bytes_sent;
    payload_max = flight_log_export_payload_max();
    chunk = payload_max;
    if (chunk > remaining) {
        chunk = (uint16_t)remaining;
    }
    if ((flight_log_export_sector_offset + chunk) > APP_FLASH_SERVICE_SECTOR_SIZE) {
        chunk = (uint16_t)(APP_FLASH_SERVICE_SECTOR_SIZE -
                           flight_log_export_sector_offset);
    }

    address = flight_log_sector_address(physical_sector) +
              flight_log_export_sector_offset;
    /*
     * Export is host-link limited; blocking SPI keeps the dump state machine
     * simple and avoids aborting on a lost background DMA completion event.
     */
    st = APP_FlashService_ReadData(address, payload, chunk);
    flight_log_status.last_flash_status = (uint32_t)st;
    if (st != APP_FLASH_SERVICE_OK) {
        (void)flight_log_queue_printf("FLOG ERROR export_read status=%lu offset=%lu\r\n",
                                      (unsigned long)st,
                                      (unsigned long)flight_log_status.export_bytes_sent);
        (void)flight_log_export_finish("error");
        return;
    }

    payload_crc = flight_log_crc32(payload, chunk);
    flags =
        ((flight_log_status.export_bytes_sent + chunk) >=
         flight_log_status.export_total_bytes) ? APP_FLIGHT_LOG_EXPORT_FLAG_LAST : 0U;

    if (flight_log_export_transport == APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY) {
        APP_FlightLogExportBlockHeader *header =
            (APP_FlightLogExportBlockHeader *)flight_log_export_usb_frame;

        if (APP_USB_CDC_IsReady() == 0U) {
            (void)flight_log_export_finish("usb_lost");
            return;
        }

        header->magic = APP_FLIGHT_LOG_EXPORT_BLOCK_MAGIC;
        header->version = APP_FLIGHT_LOG_EXPORT_VERSION;
        header->header_size = (uint16_t)sizeof(APP_FlightLogExportBlockHeader);
        header->seq = flight_log_export_seq;
        header->offset = flight_log_status.export_bytes_sent;
        header->length = chunk;
        header->flags = flags;
        header->payload_crc32 = payload_crc;
        memcpy(&flight_log_export_usb_frame[sizeof(APP_FlightLogExportBlockHeader)],
               payload,
               chunk);

        if (APP_USB_CDC_Write(flight_log_export_usb_frame,
                              (uint16_t)(sizeof(APP_FlightLogExportBlockHeader) +
                                         chunk),
                              APP_FLIGHT_LOG_USB_TX_TIMEOUT_MS) == 0U) {
            return;
        }
    } else {
        /*
         * USART1 export is link-speed limited; keep the legacy small xorhex
         * text block so old telemetry receivers can still dump logs.
         */
        written = snprintf(frame,
                           APP_UART_TX_TEXT_SIZE,
                           "FLOG BLK seq=%lu offset=%lu len=%u flags=%u crc=%08lX data=",
                           (unsigned long)flight_log_export_seq,
                           (unsigned long)flight_log_status.export_bytes_sent,
                           (unsigned int)chunk,
                           (unsigned int)flags,
                           (unsigned long)payload_crc);
        if ((written <= 0) || ((uint32_t)written >= APP_UART_TX_TEXT_SIZE)) {
            (void)flight_log_export_finish("format");
            return;
        }
        used = (uint16_t)written;
        written = (int)flight_log_hex_encode(&frame[used],
                                             (uint16_t)(APP_UART_TX_TEXT_SIZE -
                                                        used - 2U),
                                             payload,
                                             chunk,
                                             flight_log_status.export_bytes_sent);
        if (written <= 0) {
            (void)flight_log_export_finish("format");
            return;
        }
        used = (uint16_t)(used + (uint16_t)written);
        frame[used++] = '\r';
        frame[used++] = '\n';

        if (flight_log_queue_message((const uint8_t *)frame, used) == 0U) {
            return;
        }
    }

    flight_log_export_seq++;
    flight_log_status.export_bytes_sent += chunk;
    flight_log_export_sector_offset += chunk;
    flight_log_export_next_ms =
        (flight_log_export_transport == APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY) ?
        0U : (HAL_GetTick() + APP_FLIGHT_LOG_EXPORT_BLOCK_GAP_MS);
}

void APP_FlightLog_Init(void)
{
    if (flight_log_status.initialized != 0U) {
        return;
    }

    memset(&flight_log_status, 0, sizeof(flight_log_status));
    flight_log_clear_record_queue();
    flight_log_status.session_id =
        (uint32_t)(HAL_GetTick() ^ (uint32_t)SVC_Timestamp_Us() ^ 0xF10A2501UL);
    flight_log_status.last_flash_status = (uint32_t)APP_FlashService_Init();
    flight_log_scan_existing();
    flight_log_status.used_bytes = flight_log_used_bytes();
    flight_log_status.initialized = 1U;
}

void APP_FlightLog_BackgroundStep(void)
{
    if (flight_log_status.initialized == 0U) {
        APP_FlightLog_Init();
    }

    if (flight_log_status.export_active != 0U) {
        flight_log_export_step();
        return;
    }

    if (flight_log_export_pending != 0U) {
        if (flight_log_queue_count > 0U) {
            if (flight_log_write_records() == 0U) {
                flight_log_export_pending = 0U;
                flight_log_status.export_pending = 0U;
                vofaStreamActive = flight_log_export_restore_vofa;
                (void)flight_log_queue_printf("FLOG ERROR flush status=%lu\r\n",
                                              (unsigned long)flight_log_status.last_flash_status);
            }
            flight_log_status.used_bytes = flight_log_used_bytes();
            return;
        }

        (void)flight_log_start_export_from_background();
        return;
    }

    if (flight_log_write_records() == 0U) {
        flight_log_flush_requested = 0U;
    }
    flight_log_status.used_bytes = flight_log_used_bytes();
}

void APP_FlightLog_Observe(const APP_FlightLogSnapshot *snapshot,
                           uint8_t should_record)
{
    APP_FlightLogRecord record;

    if (flight_log_status.initialized == 0U) {
        return;
    }

    if ((should_record == 0U) || (snapshot == NULL) ||
        (flight_log_status.export_active != 0U) ||
        (flight_log_export_pending != 0U)) {
        if (flight_log_status.recording != 0U) {
            flight_log_status.recording = 0U;
            flight_log_flush_requested = 1U;
        }
        return;
    }

    flight_log_status.recording = 1U;
    flight_log_record_from_snapshot(&record, snapshot);

    taskENTER_CRITICAL();
    if (flight_log_queue_count >= APP_FLIGHT_LOG_QUEUE_CAPACITY) {
        flight_log_status.dropped_records++;
        taskEXIT_CRITICAL();
        return;
    }

    flight_log_queue[flight_log_queue_head] = record;
    flight_log_queue_head =
        (flight_log_queue_head + 1U) % APP_FLIGHT_LOG_QUEUE_CAPACITY;
    flight_log_queue_count++;
    flight_log_status.buffered_records = flight_log_queue_count;
    taskEXIT_CRITICAL();
}

void APP_FlightLog_GetStatus(APP_FlightLogStatus *status)
{
    if (status == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    flight_log_status.used_bytes = flight_log_used_bytes();
    flight_log_status.buffered_records = flight_log_queue_count;
    flight_log_status.export_pending = flight_log_export_pending;
    *status = flight_log_status;
    taskEXIT_CRITICAL();
}

APP_FlightLogCommandStatus APP_FlightLog_StartDump(void)
{
    if (flight_log_status.initialized == 0U) {
        APP_FlightLog_Init();
    }

    if (flight_log_status.recording != 0U) {
        return APP_FLIGHT_LOG_CMD_RECORDING;
    }
    if ((flight_log_status.export_active != 0U) ||
        (flight_log_export_pending != 0U)) {
        return APP_FLIGHT_LOG_CMD_BUSY;
    }

    flight_log_flush_requested = 1U;
    flight_log_export_transport = (APP_USB_CDC_IsReady() != 0U) ?
        APP_FLIGHT_LOG_EXPORT_USB_CDC_BINARY :
        APP_FLIGHT_LOG_EXPORT_UART_TEXT;
    flight_log_export_restore_vofa = vofaStreamActive;
    vofaStreamActive = 0U;
    flight_log_export_pending = 1U;
    flight_log_status.export_pending = 1U;
    return APP_FLIGHT_LOG_CMD_OK;
}

APP_FlightLogCommandStatus APP_FlightLog_CancelDump(void)
{
    if ((flight_log_status.export_active == 0U) &&
        (flight_log_export_pending == 0U)) {
        return APP_FLIGHT_LOG_CMD_NOT_EXPORTING;
    }

    if (flight_log_export_pending != 0U) {
        flight_log_export_pending = 0U;
        flight_log_status.export_pending = 0U;
        flight_log_flush_requested = 1U;
        vofaStreamActive = flight_log_export_restore_vofa;
        return APP_FLIGHT_LOG_CMD_OK;
    }

    flight_log_export_cancel_requested = 1U;
    return APP_FLIGHT_LOG_CMD_OK;
}

APP_FlightLogCommandStatus APP_FlightLog_TestFill(uint32_t sectors)
{
    APP_FlashService_Status st;
    uint32_t erase_probe;
    uint32_t records_per_sector =
        (APP_FLASH_SERVICE_SECTOR_SIZE - APP_FLIGHT_LOG_SECTOR_HEADER_SIZE) /
        sizeof(APP_FlightLogRecord);

    if (flight_log_status.initialized == 0U) {
        APP_FlightLog_Init();
    }
    if (flight_log_status.recording != 0U) {
        return APP_FLIGHT_LOG_CMD_RECORDING;
    }
    if ((flight_log_status.export_active != 0U) ||
        (flight_log_export_pending != 0U)) {
        return APP_FLIGHT_LOG_CMD_BUSY;
    }
    if (sectors == 0U) {
        sectors = 1U;
    }
    if (sectors > APP_FLIGHT_LOG_SECTOR_COUNT) {
        sectors = APP_FLIGHT_LOG_SECTOR_COUNT;
    }
    if (sectors > APP_FLIGHT_LOG_TESTFILL_MAX_SECTORS) {
        sectors = APP_FLIGHT_LOG_TESTFILL_MAX_SECTORS;
    }

    flight_log_clear_record_queue();
    memset(flight_log_sector_order, 0, sizeof(flight_log_sector_order));
    memset(flight_log_sector_order_seq, 0, sizeof(flight_log_sector_order_seq));
    flight_log_status.used_sectors = 0U;
    flight_log_status.used_bytes = 0U;
    flight_log_status.total_records = 0U;
    flight_log_status.dropped_records = 0U;
    flight_log_status.sector_open = 0U;
    flight_log_status.sector_seq = 1U;
    flight_log_record_sequence = 0U;
    flight_log_status.session_id =
        (uint32_t)(HAL_GetTick() ^ (uint32_t)SVC_Timestamp_Us() ^ 0xF10A7E57UL);

    memset(&flight_log_test_snapshot, 0, sizeof(flight_log_test_snapshot));
    flight_log_test_snapshot.imu_raw.accel_z = 16384;
    flight_log_test_snapshot.imu.accel_z_g = 1.0f;
    flight_log_test_snapshot.rc_armed = 1U;
    flight_log_test_snapshot.rc_link_ok = 1U;
    flight_log_test_snapshot.throttle_over_20 = 1U;
    flight_log_test_snapshot.imu_valid = 1U;
    flight_log_test_snapshot.motor_output_reason = APP_FLIGHT_LOG_MOTOR_REASON_STABILIZED_MIX;
    flight_log_test_snapshot.rc_link_seen = 1U;
    flight_log_test_snapshot.arm_switch_high = 1U;
    flight_log_test_snapshot.arm_throttle_low = 1U;

    for (uint32_t sector = 0U; sector < sectors; ++sector) {
        uint32_t address = flight_log_sector_address(sector);

        st = APP_FlashService_EraseSector(address);
        flight_log_status.last_flash_status = (uint32_t)st;
        if (st != APP_FLASH_SERVICE_OK) {
            flight_log_scan_existing();
            return APP_FLIGHT_LOG_CMD_ERROR;
        }

        st = APP_FlashService_ReadData(address,
                                       (uint8_t *)&erase_probe,
                                       sizeof(erase_probe));
        flight_log_status.last_flash_status = (uint32_t)st;
        if ((st != APP_FLASH_SERVICE_OK) || (erase_probe != 0xFFFFFFFFUL)) {
            flight_log_status.last_flash_status = (uint32_t)APP_FLASH_SERVICE_ERROR;
            flight_log_scan_existing();
            return APP_FLIGHT_LOG_CMD_ERROR;
        }

        flight_log_fill_sector_header(&flight_log_test_header, sector);
        st = APP_FlashService_WriteData(address,
                                        (const uint8_t *)&flight_log_test_header,
                                        sizeof(flight_log_test_header));
        flight_log_status.last_flash_status = (uint32_t)st;
        if (st != APP_FLASH_SERVICE_OK) {
            flight_log_scan_existing();
            return APP_FLIGHT_LOG_CMD_ERROR;
        }

        st = APP_FlashService_ReadData(address,
                                       (uint8_t *)&flight_log_test_verify_header,
                                       sizeof(flight_log_test_verify_header));
        flight_log_status.last_flash_status = (uint32_t)st;
        if ((st != APP_FLASH_SERVICE_OK) ||
            (flight_log_sector_header_valid(&flight_log_test_verify_header) == 0U)) {
            flight_log_status.last_flash_status = (uint32_t)APP_FLASH_SERVICE_ERROR;
            flight_log_scan_existing();
            return APP_FLIGHT_LOG_CMD_ERROR;
        }

        for (uint32_t index = 0U; index < records_per_sector; ++index) {
            uint32_t record_address = address +
                                      APP_FLIGHT_LOG_SECTOR_HEADER_SIZE +
                                      (index * sizeof(flight_log_test_record));

            flight_log_test_snapshot.timestamp_us =
                ((uint64_t)sector * 1000000ULL) + ((uint64_t)index * 4000ULL);
            flight_log_test_snapshot.tick_ms = (sector * 1000U) + (index * 4U);
            flight_log_test_snapshot.imu_sequence = flight_log_record_sequence + 1U;
            flight_log_test_snapshot.roll_deg = (float)sector;
            flight_log_test_snapshot.pitch_deg = (float)index;
            flight_log_test_snapshot.yaw_deg = (float)(sector + index);
            for (uint32_t ch = 0U; ch < 8U; ++ch) {
                flight_log_test_snapshot.rc_channels[ch] =
                    (uint16_t)(1000U + (ch * 50U));
            }
            flight_log_test_snapshot.throttle_us = 1300U;
            flight_log_test_snapshot.servo_alpha_us = 1500U;
            flight_log_test_snapshot.servo_beta_us = 1500U;
            flight_log_test_snapshot.motor_upper_us = 1300U;
            flight_log_test_snapshot.motor_lower_us = 1300U;
            flight_log_test_snapshot.z_ref_m = 0.1f;

            flight_log_record_from_snapshot(&flight_log_test_record,
                                            &flight_log_test_snapshot);
            st = APP_FlashService_WriteData(record_address,
                                            (const uint8_t *)&flight_log_test_record,
                                            sizeof(flight_log_test_record));
            flight_log_status.last_flash_status = (uint32_t)st;
            if (st != APP_FLASH_SERVICE_OK) {
                flight_log_scan_existing();
                return APP_FLIGHT_LOG_CMD_ERROR;
            }
            st = APP_FlashService_ReadData(record_address,
                                           (uint8_t *)&flight_log_test_verify_record,
                                           sizeof(flight_log_test_verify_record));
            flight_log_status.last_flash_status = (uint32_t)st;
            if ((st != APP_FLASH_SERVICE_OK) ||
                (memcmp(&flight_log_test_verify_record,
                        &flight_log_test_record,
                        sizeof(flight_log_test_record)) != 0)) {
                flight_log_status.last_flash_status = (uint32_t)APP_FLASH_SERVICE_ERROR;
                flight_log_scan_existing();
                return APP_FLIGHT_LOG_CMD_ERROR;
            }
            flight_log_status.total_records++;
        }

        flight_log_order_insert((uint16_t)sector, flight_log_status.sector_seq);
        flight_log_status.sector_seq++;
    }

    flight_log_next_sector_index = sectors % APP_FLIGHT_LOG_SECTOR_COUNT;
    flight_log_current_sector_index = 0U;
    flight_log_sector_write_offset = APP_FLIGHT_LOG_SECTOR_HEADER_SIZE;
    flight_log_status.sector_open = 0U;
    flight_log_status.used_bytes = flight_log_used_bytes();
    return APP_FLIGHT_LOG_CMD_OK;
}

uint8_t APP_FlightLog_IsExportActive(void)
{
    return (flight_log_status.export_active != 0U) ? 1U : 0U;
}

const char *APP_FlightLog_CommandStatusText(APP_FlightLogCommandStatus status)
{
    switch (status) {
    case APP_FLIGHT_LOG_CMD_OK:
        return "ok";
    case APP_FLIGHT_LOG_CMD_BUSY:
        return "busy";
    case APP_FLIGHT_LOG_CMD_RECORDING:
        return "recording";
    case APP_FLIGHT_LOG_CMD_NOT_EXPORTING:
        return "not_exporting";
    default:
        return "error";
    }
}
