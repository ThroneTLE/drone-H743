#include "app_imu_capture.h"

#include "app_usb_cdc.h"
#include "bsp_imu.h"

#include <string.h>

/*
 * Full-rate raw IMU capture. See app_imu_capture.h for why this exists and why
 * the producer side must stay non-blocking.
 */

#define APP_IMU_CAPTURE_USB_TX_TIMEOUT_MS 50U
/*
 * Block size is bounded by APP_USB_CDC_TX_SIZE (1536 B): APP_USB_CDC_Write()
 * rejects anything larger outright, which silently stalls the whole export.
 * 32 samples x 46 B = 1472 B payload + 40 B header = 1512 B, just inside it.
 * The static assert below keeps this honest if either size changes.
 */
#define APP_IMU_CAPTURE_BLOCK_SAMPLES 32U

typedef struct {
    APP_IMU_CaptureSample samples[APP_IMU_CAPTURE_CAPACITY];
} APP_IMU_CaptureStorage;

/*
 * .ram_d1_noinit maps to the 512 KB AXI SRAM, which is only lightly used. The
 * buffer is far too large for the 128 KB DTCM that holds the RTOS stacks, and
 * it needs no DMA reachability because only the CPU touches it.
 */
#if defined(__GNUC__)
__attribute__((section(".ram_d1_noinit"), aligned(32)))
#endif
static APP_IMU_CaptureStorage imu_capture_storage;

/*
 * volatile: written by Sensor_Task, read by the export task. Only the producer
 * writes write_index/dropped and only the consumer reads them once recording has
 * stopped, so a lock is unnecessary — but the compiler must not cache them.
 */
static volatile APP_IMU_CaptureState imu_capture_state = APP_IMU_CAPTURE_IDLE;
static volatile uint32_t imu_capture_write_index;
static volatile uint32_t imu_capture_dropped;
static uint32_t imu_capture_requested;
static uint32_t imu_capture_session_id;
static uint32_t imu_capture_export_sent;
static uint32_t imu_capture_export_total;
static uint32_t imu_capture_export_retries;

/* Roughly a second of retries at the VOFA task cadence before giving up. */
#define APP_IMU_CAPTURE_EXPORT_RETRY_LIMIT 200U

static uint8_t imu_capture_tx_frame[sizeof(APP_IMU_CaptureBlockHeader) +
                                    (APP_IMU_CAPTURE_BLOCK_SAMPLES *
                                     sizeof(APP_IMU_CaptureSample))];

/*
 * APP_USB_CDC_Write() returns failure for any length above its TX buffer, and
 * ExportStep() treats failure as "retry this block", so an oversized frame
 * stalls the export forever with no error surfaced to the host. Catch it here.
 */
_Static_assert(sizeof(imu_capture_tx_frame) <= APP_USB_CDC_TX_SIZE,
               "IMU capture block exceeds the USB CDC TX buffer");

static uint32_t imu_capture_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t bit;

    for (i = 0U; i < length; i++) {
        crc ^= (uint32_t)data[i];
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 1UL) != 0UL) ? ((crc >> 1) ^ 0xEDB88320UL)
                                       : (crc >> 1);
        }
    }
    return ~crc;
}

static void imu_capture_fill_ranges(APP_IMU_CaptureStatus *status)
{
    const DRV_IMU_Device *dev = BSP_IMU_GetDevice();

    if (dev == NULL) {
        status->accel_aaf_hz = 0U;
        status->gyro_aaf_hz = 0U;
        status->accel_range_g = 0U;
        status->gyro_range_dps = 0U;
        return;
    }

    status->accel_aaf_hz = dev->accel_aaf_actual_hz;
    status->gyro_aaf_hz = dev->gyro_aaf_actual_hz;

    switch (dev->config.accel_range) {
    case DRV_IMU_ACCEL_RANGE_2G:  status->accel_range_g = 2U;  break;
    case DRV_IMU_ACCEL_RANGE_4G:  status->accel_range_g = 4U;  break;
    case DRV_IMU_ACCEL_RANGE_8G:  status->accel_range_g = 8U;  break;
    case DRV_IMU_ACCEL_RANGE_16G:
    default:                      status->accel_range_g = 16U; break;
    }

    switch (dev->config.gyro_range) {
    case DRV_IMU_GYRO_RANGE_250DPS:  status->gyro_range_dps = 250U;  break;
    case DRV_IMU_GYRO_RANGE_500DPS:  status->gyro_range_dps = 500U;  break;
    case DRV_IMU_GYRO_RANGE_1000DPS: status->gyro_range_dps = 1000U; break;
    case DRV_IMU_GYRO_RANGE_2000DPS: status->gyro_range_dps = 2000U; break;
    default:                         status->gyro_range_dps = 0U;    break;
    }
}

void APP_IMU_Capture_Init(void)
{
    imu_capture_state = APP_IMU_CAPTURE_IDLE;
    imu_capture_write_index = 0U;
    imu_capture_dropped = 0U;
    imu_capture_requested = 0U;
    imu_capture_session_id = 0U;
    imu_capture_export_sent = 0U;
    imu_capture_export_total = 0U;
}

void APP_IMU_Capture_Push(uint32_t timestamp_us,
                          const DRV_IMU_RawData *raw,
                          uint16_t motor_upper_us,
                          uint16_t motor_lower_us)
{
    uint32_t index;
    APP_IMU_CaptureSample *slot;

    /* Hot path: bail out immediately unless armed for capture. */
    if ((imu_capture_state != APP_IMU_CAPTURE_RECORDING) || (raw == NULL)) {
        return;
    }

    index = imu_capture_write_index;
    if (index >= imu_capture_requested) {
        /* Stop rather than wrap: a contiguous window is what FFT needs. */
        imu_capture_state = APP_IMU_CAPTURE_FULL;
        return;
    }

    slot = &imu_capture_storage.samples[index];
    /*
     * Clear first: the buffer is NOLOAD and annotators are optional, so stale
     * bytes from an earlier session could otherwise masquerade as real data.
     */
    memset(slot, 0, sizeof(*slot));
    slot->timestamp_us = timestamp_us;
    slot->accel[0] = raw->accel_x;
    slot->accel[1] = raw->accel_y;
    slot->accel[2] = raw->accel_z;
    slot->gyro[0] = raw->gyro_x;
    slot->gyro[1] = raw->gyro_y;
    slot->gyro[2] = raw->gyro_z;
    slot->motor_upper_us = motor_upper_us;
    slot->motor_lower_us = motor_lower_us;

    imu_capture_write_index = index + 1U;
    if (imu_capture_write_index >= imu_capture_requested) {
        imu_capture_state = APP_IMU_CAPTURE_FULL;
    }
}

static int16_t imu_capture_saturate(float value)
{
    if (value > 32767.0f)  { return (int16_t)32767; }
    if (value < -32768.0f) { return (int16_t)-32768; }
    return (int16_t)value;
}

/*
 * Returns the slot just filled by Push(), or NULL when nothing is pending.
 * Annotators run after Push() in the same or the following task, so the target
 * is always write_index - 1.
 */
static APP_IMU_CaptureSample *imu_capture_pending_slot(void)
{
    uint32_t index = imu_capture_write_index;

    if ((imu_capture_state != APP_IMU_CAPTURE_RECORDING) &&
        (imu_capture_state != APP_IMU_CAPTURE_FULL)) {
        return NULL;
    }
    if ((index == 0U) || (index > APP_IMU_CAPTURE_CAPACITY)) {
        return NULL;
    }
    return &imu_capture_storage.samples[index - 1U];
}

void APP_IMU_Capture_AnnotateFiltered(const float accel_g[3],
                                      const float gyro_dps[3],
                                      uint8_t gyro_bias_ready)
{
    APP_IMU_CaptureSample *slot = imu_capture_pending_slot();
    uint32_t axis;

    if ((slot == NULL) || (accel_g == NULL) || (gyro_dps == NULL)) {
        return;
    }

    for (axis = 0U; axis < 3U; axis++) {
        /* milli-g and centi-dps keep full useful range inside int16. */
        slot->accel_filt[axis] = imu_capture_saturate(accel_g[axis] * 1000.0f);
        slot->gyro_filt[axis] = imu_capture_saturate(gyro_dps[axis] * 100.0f);
    }
    if (gyro_bias_ready != 0U) {
        slot->flight_flags |= APP_IMU_CAPTURE_FLIGHT_GYRO_BIAS_READY;
    }
}

void APP_IMU_Capture_AnnotateControl(float roll_deg,
                                     float pitch_deg,
                                     float yaw_deg,
                                     uint16_t servo_alpha_us,
                                     uint16_t servo_beta_us,
                                     float accel_error_deg,
                                     uint8_t fusion_flags,
                                     uint8_t armed)
{
    APP_IMU_CaptureSample *slot = imu_capture_pending_slot();

    if (slot == NULL) {
        return;
    }

    slot->roll_cdeg = imu_capture_saturate(roll_deg * 100.0f);
    slot->pitch_cdeg = imu_capture_saturate(pitch_deg * 100.0f);
    slot->yaw_cdeg = imu_capture_saturate(yaw_deg * 100.0f);
    slot->servo_alpha_us = servo_alpha_us;
    slot->servo_beta_us = servo_beta_us;
    slot->accel_error_cdeg = imu_capture_saturate(accel_error_deg * 100.0f);
    slot->fusion_flags = fusion_flags;
    if (armed != 0U) {
        slot->flight_flags |= APP_IMU_CAPTURE_FLIGHT_ARMED;
    }
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_Start(uint32_t sample_count)
{
    if ((imu_capture_state == APP_IMU_CAPTURE_RECORDING) ||
        (imu_capture_state == APP_IMU_CAPTURE_EXPORTING)) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (sample_count == 0U) {
        sample_count = APP_IMU_CAPTURE_CAPACITY;
    }
    if (sample_count > APP_IMU_CAPTURE_CAPACITY) {
        sample_count = APP_IMU_CAPTURE_CAPACITY;
    }

    imu_capture_write_index = 0U;
    imu_capture_dropped = 0U;
    imu_capture_export_sent = 0U;
    imu_capture_export_total = 0U;
    imu_capture_export_retries = 0U;
    imu_capture_requested = sample_count;
    ++imu_capture_session_id;
    /* Publish the arming flag last so the producer never sees a half-set run. */
    imu_capture_state = APP_IMU_CAPTURE_RECORDING;
    return APP_IMU_CAPTURE_CMD_OK;
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_Stop(void)
{
    if (imu_capture_state == APP_IMU_CAPTURE_RECORDING) {
        imu_capture_state = (imu_capture_write_index > 0U) ?
            APP_IMU_CAPTURE_FULL : APP_IMU_CAPTURE_IDLE;
        return APP_IMU_CAPTURE_CMD_OK;
    }
    return APP_IMU_CAPTURE_CMD_INVALID;
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_StartDump(void)
{
    if (imu_capture_state == APP_IMU_CAPTURE_RECORDING) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (imu_capture_state == APP_IMU_CAPTURE_EXPORTING) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (imu_capture_write_index == 0U) {
        return APP_IMU_CAPTURE_CMD_EMPTY;
    }
    if (APP_USB_CDC_IsReady() == 0U) {
        return APP_IMU_CAPTURE_CMD_NO_LINK;
    }

    imu_capture_export_sent = 0U;
    imu_capture_export_total = imu_capture_write_index;
    imu_capture_export_retries = 0U;
    imu_capture_state = APP_IMU_CAPTURE_EXPORTING;
    return APP_IMU_CAPTURE_CMD_OK;
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_CancelDump(void)
{
    if (imu_capture_state != APP_IMU_CAPTURE_EXPORTING) {
        return APP_IMU_CAPTURE_CMD_INVALID;
    }
    /* Samples survive a cancelled dump so the host can retry. */
    imu_capture_state = APP_IMU_CAPTURE_FULL;
    return APP_IMU_CAPTURE_CMD_OK;
}

uint8_t APP_IMU_Capture_IsExportActive(void)
{
    return (imu_capture_state == APP_IMU_CAPTURE_EXPORTING) ? 1U : 0U;
}

void APP_IMU_Capture_GetStatus(APP_IMU_CaptureStatus *status)
{
    if (status == NULL) {
        return;
    }

    memset(status, 0, sizeof(*status));
    status->state = imu_capture_state;
    status->stored = imu_capture_write_index;
    status->capacity = APP_IMU_CAPTURE_CAPACITY;
    status->requested = imu_capture_requested;
    status->dropped = imu_capture_dropped;
    status->export_sent = imu_capture_export_sent;
    status->session_id = imu_capture_session_id;
    imu_capture_fill_ranges(status);
}

void APP_IMU_Capture_ExportStep(void)
{
    APP_IMU_CaptureBlockHeader *header;
    uint8_t *payload;
    uint32_t remaining;
    uint32_t block;
    uint32_t payload_bytes;
    uint32_t flags;

    if (imu_capture_state != APP_IMU_CAPTURE_EXPORTING) {
        return;
    }
    if (APP_USB_CDC_IsReady() == 0U) {
        /* Link dropped mid-dump: keep the samples, let the host restart. */
        imu_capture_state = APP_IMU_CAPTURE_FULL;
        return;
    }

    remaining = imu_capture_export_total - imu_capture_export_sent;
    block = (remaining > APP_IMU_CAPTURE_BLOCK_SAMPLES) ?
        APP_IMU_CAPTURE_BLOCK_SAMPLES : remaining;
    payload_bytes = block * (uint32_t)sizeof(APP_IMU_CaptureSample);
    flags = (block == remaining) ? APP_IMU_CAPTURE_FLAG_LAST : 0UL;

    header = (APP_IMU_CaptureBlockHeader *)imu_capture_tx_frame;
    payload = &imu_capture_tx_frame[sizeof(APP_IMU_CaptureBlockHeader)];
    memcpy(payload,
           &imu_capture_storage.samples[imu_capture_export_sent],
           payload_bytes);

    header->magic = APP_IMU_CAPTURE_MAGIC;
    header->version = APP_IMU_CAPTURE_VERSION;
    header->header_size = (uint16_t)sizeof(APP_IMU_CaptureBlockHeader);
    header->session_id = imu_capture_session_id;
    header->total_samples = imu_capture_export_total;
    header->offset_samples = imu_capture_export_sent;
    header->block_samples = (uint16_t)block;
    header->sample_size = (uint16_t)sizeof(APP_IMU_CaptureSample);
    header->flags = flags;
    header->payload_crc32 = imu_capture_crc32(payload, payload_bytes);
    {
        APP_IMU_CaptureStatus ranges;
        memset(&ranges, 0, sizeof(ranges));
        imu_capture_fill_ranges(&ranges);
        header->accel_aaf_hz = ranges.accel_aaf_hz;
        header->gyro_aaf_hz = ranges.gyro_aaf_hz;
        header->accel_range_g = ranges.accel_range_g;
        header->gyro_range_dps = ranges.gyro_range_dps;
    }

    if (APP_USB_CDC_Write(imu_capture_tx_frame,
                          (uint16_t)(sizeof(APP_IMU_CaptureBlockHeader) +
                                     payload_bytes),
                          APP_IMU_CAPTURE_USB_TX_TIMEOUT_MS) == 0U) {
        /*
         * Retry the same block; offset is unchanged. Bound the retries so a
         * permanently failing write (rather than transient USB back-pressure)
         * ends the dump instead of spinning silently forever.
         */
        ++imu_capture_export_retries;
        if (imu_capture_export_retries >= APP_IMU_CAPTURE_EXPORT_RETRY_LIMIT) {
            imu_capture_state = APP_IMU_CAPTURE_FULL;
        }
        return;
    }
    imu_capture_export_retries = 0U;

    imu_capture_export_sent += block;
    if (imu_capture_export_sent >= imu_capture_export_total) {
        imu_capture_state = APP_IMU_CAPTURE_FULL;
    }
}

const char *APP_IMU_Capture_CommandStatusText(
    APP_IMU_CaptureCommandStatus status)
{
    switch (status) {
    case APP_IMU_CAPTURE_CMD_OK:      return "ok";
    case APP_IMU_CAPTURE_CMD_BUSY:    return "busy";
    case APP_IMU_CAPTURE_CMD_EMPTY:   return "empty";
    case APP_IMU_CAPTURE_CMD_NO_LINK: return "no_usb";
    case APP_IMU_CAPTURE_CMD_INVALID:
    default:                          return "invalid";
    }
}

const char *APP_IMU_Capture_StateText(APP_IMU_CaptureState state)
{
    switch (state) {
    case APP_IMU_CAPTURE_IDLE:      return "idle";
    case APP_IMU_CAPTURE_RECORDING: return "recording";
    case APP_IMU_CAPTURE_FULL:      return "ready";
    case APP_IMU_CAPTURE_EXPORTING: return "exporting";
    default:                        return "unknown";
    }
}
