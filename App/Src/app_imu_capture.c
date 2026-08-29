#include "app_imu_capture.h"

#include "app_firmware_identity.h"
#include "app_flight_calibration.h"
#include "app_sensor.h"
#include "app_usb_cdc.h"
#include "bsp_imu.h"
#include "drv_frame_contract.h"

#include <string.h>

/*
 * Full-rate raw IMU capture. See app_imu_capture.h for why this exists and why
 * the producer side must stay non-blocking.
 */

#define APP_IMU_CAPTURE_USB_TX_TIMEOUT_MS 50U
/*
 * Block size is bounded by APP_USB_CDC_TX_SIZE (1536 B): APP_USB_CDC_Write()
 * rejects anything larger outright, which silently stalls the whole export.
 * V4 uses 30 samples x 48 B = 1440 B payload + 56 B header = 1496 B.
 * The static assert below keeps this honest if either size changes.
 */
#define APP_IMU_CAPTURE_BLOCK_SAMPLES 30U

_Static_assert(sizeof(APP_IMU_CaptureSample) ==
                   APP_IMU_CAPTURE_V4_SAMPLE_SIZE,
               "IMUCAP v4 sample ABI must remain 48 bytes");
_Static_assert(sizeof(APP_IMU_CaptureBlockHeader) ==
                   APP_IMU_CAPTURE_V4_HEADER_SIZE,
               "IMUCAP v4 header ABI must remain 56 bytes");

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
static volatile uint32_t imu_capture_committed_index;
static volatile uint32_t imu_capture_control_index;
static volatile uint32_t imu_capture_dropped;
static volatile uint32_t imu_capture_sensor_pending_index;
static volatile uint8_t imu_capture_sensor_pending_valid;
static volatile uint8_t imu_capture_control_started;
static volatile uint8_t imu_capture_annotation_mask[APP_IMU_CAPTURE_CAPACITY];
static uint32_t imu_capture_requested;
static uint32_t imu_capture_session_id;
static uint32_t imu_capture_export_sent;
static uint32_t imu_capture_export_total;
static uint32_t imu_capture_export_retries;
static uint32_t imu_capture_calibration_generation;
static uint16_t imu_capture_frame_contract;
static uint8_t imu_capture_orientation_code;
static uint8_t imu_capture_calibration_valid_mask;
static uint8_t imu_capture_base_frame;
static uint8_t imu_capture_provenance_invalid;
static uint32_t imu_capture_firmware_image_crc32;

/* Roughly a second of retries at the VOFA task cadence before giving up. */
#define APP_IMU_CAPTURE_EXPORT_RETRY_LIMIT 200U
#define APP_IMU_CAPTURE_ANNOTATION_FILTERED 0x01U
#define APP_IMU_CAPTURE_ANNOTATION_CONTROL  0x02U
#define APP_IMU_CAPTURE_ANNOTATION_COMPLETE \
    (APP_IMU_CAPTURE_ANNOTATION_FILTERED | APP_IMU_CAPTURE_ANNOTATION_CONTROL)

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

static void imu_capture_memory_barrier(void)
{
#if defined(__GNUC__) || defined(__clang__)
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
#endif
}

static void imu_capture_finish_draining_if_complete(void)
{
    if ((imu_capture_state == APP_IMU_CAPTURE_DRAINING) &&
        (imu_capture_committed_index == imu_capture_write_index)) {
        imu_capture_state = (imu_capture_committed_index > 0U) ?
            APP_IMU_CAPTURE_FULL : APP_IMU_CAPTURE_IDLE;
    }
}

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
    imu_capture_committed_index = 0U;
    imu_capture_control_index = 0U;
    imu_capture_dropped = 0U;
    imu_capture_sensor_pending_index = 0U;
    imu_capture_sensor_pending_valid = 0U;
    imu_capture_control_started = 0U;
    imu_capture_requested = 0U;
    imu_capture_session_id = 0U;
    imu_capture_export_sent = 0U;
    imu_capture_export_total = 0U;
    imu_capture_calibration_generation = 0U;
    imu_capture_frame_contract = DRV_FRAME_CONTRACT_VERSION;
    imu_capture_orientation_code = APP_SENSOR_FLU_ORIENTATION_LEGACY;
    imu_capture_calibration_valid_mask = 0U;
    imu_capture_base_frame =
        APP_IMU_CAPTURE_BASE_RAW_ICM42688_FILTERED_LEGACY_V1;
    imu_capture_provenance_invalid = 0U;
    imu_capture_firmware_image_crc32 = 0U;
}

void APP_IMU_Capture_Push(uint32_t timestamp_us,
                          const DRV_IMU_RawData *raw,
                          uint16_t motor_upper_us,
                          uint16_t motor_lower_us)
{
    uint32_t index;
    uint32_t current_calibration_generation;
    APP_IMU_CaptureSample *slot;

    /* Never let the following filter callback reuse a slot from an old frame. */
    imu_capture_sensor_pending_valid = 0U;

    /* Hot path: bail out immediately unless armed for capture. */
    if ((imu_capture_state != APP_IMU_CAPTURE_RECORDING) || (raw == NULL)) {
        return;
    }

    /* A capture is one immutable calibration epoch; never mix epochs. */
    if ((APP_FlightCalibration_ReadActiveGeneration(
             &current_calibration_generation) == 0U) ||
        (APP_Sensor_GetFluOrientation() != imu_capture_orientation_code) ||
        (current_calibration_generation !=
         imu_capture_calibration_generation)) {
        imu_capture_provenance_invalid = 1U;
        imu_capture_state = APP_IMU_CAPTURE_DRAINING;
        imu_capture_finish_draining_if_complete();
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
    slot->temperature_raw = raw->temperature;
    slot->accel[0] = raw->accel_x;
    slot->accel[1] = raw->accel_y;
    slot->accel[2] = raw->accel_z;
    slot->gyro[0] = raw->gyro_x;
    slot->gyro[1] = raw->gyro_y;
    slot->gyro[2] = raw->gyro_z;
    slot->motor_upper_us = motor_upper_us;
    slot->motor_lower_us = motor_lower_us;

    imu_capture_annotation_mask[index] = 0U;
    imu_capture_memory_barrier();
    imu_capture_write_index = index + 1U;
    imu_capture_sensor_pending_index = index;
    imu_capture_sensor_pending_valid = 1U;
    if (imu_capture_write_index >= imu_capture_requested) {
        /* The final raw frame still needs filtered and control annotations. */
        imu_capture_state = APP_IMU_CAPTURE_DRAINING;
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
static APP_IMU_CaptureSample *imu_capture_pending_filtered_slot(
    uint32_t *slot_index)
{
    uint32_t index = imu_capture_sensor_pending_index;

    if ((imu_capture_state != APP_IMU_CAPTURE_RECORDING) &&
        (imu_capture_state != APP_IMU_CAPTURE_DRAINING)) {
        return NULL;
    }
    if ((imu_capture_sensor_pending_valid == 0U) ||
        (index >= imu_capture_write_index) ||
        (index >= APP_IMU_CAPTURE_CAPACITY)) {
        return NULL;
    }
    *slot_index = index;
    return &imu_capture_storage.samples[index];
}

void APP_IMU_Capture_AnnotateFiltered(const float accel_g[3],
                                      const float gyro_dps[3],
                                      uint8_t gyro_bias_ready)
{
    uint32_t slot_index = 0U;
    APP_IMU_CaptureSample *slot =
        imu_capture_pending_filtered_slot(&slot_index);
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
    imu_capture_memory_barrier();
    imu_capture_annotation_mask[slot_index] |=
        APP_IMU_CAPTURE_ANNOTATION_FILTERED;
    imu_capture_sensor_pending_valid = 0U;
}

void APP_IMU_Capture_AnnotateControl(uint32_t timestamp_us,
                                     float roll_deg,
                                     float pitch_deg,
                                     float yaw_deg,
                                     uint16_t servo_alpha_us,
                                     uint16_t servo_beta_us,
                                     float accel_error_deg,
                                     uint8_t fusion_flags,
                                     uint8_t armed)
{
    uint32_t slot_index;
    APP_IMU_CaptureSample *slot;

    if ((imu_capture_state != APP_IMU_CAPTURE_RECORDING) &&
        (imu_capture_state != APP_IMU_CAPTURE_DRAINING)) {
        return;
    }
    slot_index = imu_capture_control_index;
    if (slot_index >= imu_capture_write_index) {
        return;
    }
    slot = &imu_capture_storage.samples[slot_index];
    if (slot->timestamp_us != timestamp_us) {
        /* Ignore pre-capture queue residue until the first exact match. */
        if (imu_capture_control_started == 0U) {
            return;
        }
        /* Once matched, any gap would make every later association ambiguous. */
        imu_capture_provenance_invalid = 1U;
        imu_capture_state = (imu_capture_committed_index > 0U) ?
            APP_IMU_CAPTURE_FULL : APP_IMU_CAPTURE_IDLE;
        return;
    }
    if ((imu_capture_annotation_mask[slot_index] &
         APP_IMU_CAPTURE_ANNOTATION_FILTERED) == 0U) {
        imu_capture_provenance_invalid = 1U;
        imu_capture_state = (imu_capture_committed_index > 0U) ?
            APP_IMU_CAPTURE_FULL : APP_IMU_CAPTURE_IDLE;
        return;
    }
    imu_capture_control_started = 1U;

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
    imu_capture_memory_barrier();
    imu_capture_annotation_mask[slot_index] |=
        APP_IMU_CAPTURE_ANNOTATION_CONTROL;
    if ((imu_capture_annotation_mask[slot_index] &
         APP_IMU_CAPTURE_ANNOTATION_COMPLETE) ==
        APP_IMU_CAPTURE_ANNOTATION_COMPLETE) {
        imu_capture_control_index = slot_index + 1U;
        imu_capture_committed_index = slot_index + 1U;
    }
    imu_capture_finish_draining_if_complete();
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_Start(uint32_t sample_count)
{
    APP_FlightCalibrationSnapshot calibration_snapshot;
    APP_FirmwareIdentity firmware_identity;
    DRV_IMU_Calibration imu_calibration;
    uint8_t orientation_before;
    uint8_t orientation_after;
    uint8_t calibration_valid_mask;

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

    orientation_before = APP_Sensor_GetFluOrientation();
    if (APP_FlightCalibration_ReadActive(&calibration_snapshot) == 0U) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    orientation_after = APP_Sensor_GetFluOrientation();
    if (orientation_before != orientation_after) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (APP_FirmwareIdentity_Get(&firmware_identity) == 0U) {
        return APP_IMU_CAPTURE_CMD_INVALID;
    }
    calibration_valid_mask = APP_FlightCalibration_BuildImuCalibration(
        &calibration_snapshot.calibration,
        orientation_after,
        &imu_calibration);

    imu_capture_write_index = 0U;
    imu_capture_committed_index = 0U;
    imu_capture_control_index = 0U;
    imu_capture_dropped = 0U;
    imu_capture_export_sent = 0U;
    imu_capture_export_total = 0U;
    imu_capture_export_retries = 0U;
    imu_capture_requested = sample_count;
    imu_capture_sensor_pending_valid = 0U;
    imu_capture_control_started = 0U;
    ++imu_capture_session_id;
    imu_capture_frame_contract = DRV_FRAME_CONTRACT_VERSION;
    imu_capture_orientation_code = orientation_after;
    imu_capture_calibration_generation = calibration_snapshot.generation;
    imu_capture_calibration_valid_mask = calibration_valid_mask;
    imu_capture_base_frame =
        APP_IMU_CAPTURE_BASE_RAW_ICM42688_FILTERED_LEGACY_V1;
    imu_capture_provenance_invalid = 0U;
    imu_capture_firmware_image_crc32 = firmware_identity.image_crc32;
    /* Publish the arming flag last so the producer never sees a half-set run. */
    imu_capture_state = APP_IMU_CAPTURE_RECORDING;
    return APP_IMU_CAPTURE_CMD_OK;
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_Stop(void)
{
    if (imu_capture_state == APP_IMU_CAPTURE_RECORDING) {
        imu_capture_state = APP_IMU_CAPTURE_DRAINING;
        imu_capture_finish_draining_if_complete();
        return APP_IMU_CAPTURE_CMD_OK;
    }
    return APP_IMU_CAPTURE_CMD_INVALID;
}

APP_IMU_CaptureCommandStatus APP_IMU_Capture_StartDump(void)
{
    if (imu_capture_state == APP_IMU_CAPTURE_RECORDING) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (imu_capture_state == APP_IMU_CAPTURE_DRAINING) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (imu_capture_state == APP_IMU_CAPTURE_EXPORTING) {
        return APP_IMU_CAPTURE_CMD_BUSY;
    }
    if (imu_capture_committed_index == 0U) {
        return APP_IMU_CAPTURE_CMD_EMPTY;
    }
    if (APP_USB_CDC_IsReady() == 0U) {
        return APP_IMU_CAPTURE_CMD_NO_LINK;
    }

    imu_capture_export_sent = 0U;
    imu_capture_export_total = imu_capture_committed_index;
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
    status->stored = imu_capture_committed_index;
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
    if (imu_capture_provenance_invalid != 0U) {
        flags |= APP_IMU_CAPTURE_FLAG_INVALID_PROVENANCE;
    }

    header = (APP_IMU_CaptureBlockHeader *)imu_capture_tx_frame;
    payload = &imu_capture_tx_frame[sizeof(APP_IMU_CaptureBlockHeader)];
    memset(header, 0, sizeof(*header));
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
        header->frame_contract = imu_capture_frame_contract;
        header->orientation_code = imu_capture_orientation_code;
        header->calibration_valid_mask =
            imu_capture_calibration_valid_mask;
        header->calibration_generation =
            imu_capture_calibration_generation;
        header->base_frame = imu_capture_base_frame;
        header->firmware_image_crc32 =
            imu_capture_firmware_image_crc32;
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
    case APP_IMU_CAPTURE_DRAINING:  return "finalizing";
    default:                        return "unknown";
    }
}
