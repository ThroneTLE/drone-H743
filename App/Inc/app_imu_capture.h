#ifndef APP_IMU_CAPTURE_H
#define APP_IMU_CAPTURE_H

#include "drv_imu.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Full-rate raw IMU capture for vibration spectrum analysis.
 *
 * Purpose: the flight log and VOFA streams are decimated (~250 Hz and 40 Hz),
 * and both carry post-LPF values. Neither can show the 150-300 Hz coaxial rotor
 * band, so neither can size the anti-alias or software filters. This module
 * records undecimated pre-filter samples straight from the sensor.
 *
 * Design constraint: the producer runs in Sensor_Task at 1 kHz. It must never
 * block, because stalling that task distorts the very timing being measured and
 * starves the stabilizer. So the sample hook only writes into a RAM ring buffer
 * and the slow USB export runs from a low-priority task.
 */

/*
 * Raw sensor frame plus attitude, actuator and control state, all at the full
 * 1 kHz sample rate.
 *
 * The raw accel/gyro pair is what the spectrum analysis needs. The rest is here
 * because the USB link has bandwidth to spare and because correlating vibration
 * against actuator commands and the estimator output in the *same* full-rate
 * record is what separates a sensor problem from a control problem — the
 * existing 40 Hz VOFA stream aliases all of it away.
 */
/* Packed: this struct goes on the wire verbatim, so no implicit tail padding. */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;   /* truncated microsecond stamp; deltas are exact  */
    int16_t  accel[3];       /* sensor-frame LSB, before axis align and LPF    */
    int16_t  gyro[3];
    /* Filtered/aligned values, so the LPF's real effect is measurable. */
    int16_t  accel_filt[3];  /* milli-g                                       */
    int16_t  gyro_filt[3];   /* centi-dps, +-327 dps                          */
    /* Estimator output, centi-degrees. */
    int16_t  roll_cdeg;
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    /* Actuator commands, microseconds. */
    uint16_t motor_upper_us;
    uint16_t motor_lower_us;
    uint16_t servo_alpha_us;
    uint16_t servo_beta_us;
    /* Fusion health, to catch gate lockout recurring under vibration. */
    uint8_t  fusion_flags;   /* see APP_IMU_CAPTURE_FUSION_* below            */
    uint8_t  flight_flags;   /* see APP_IMU_CAPTURE_FLIGHT_* below            */
    int16_t  accel_error_cdeg; /* Fusion gravity innovation, centi-degrees     */
} APP_IMU_CaptureSample;

/* fusion_flags bits */
#define APP_IMU_CAPTURE_FUSION_ACCEL_IGNORED   0x01U
#define APP_IMU_CAPTURE_FUSION_NORM_REJECTED   0x02U
#define APP_IMU_CAPTURE_FUSION_ACCEL_RECOVERY  0x04U
#define APP_IMU_CAPTURE_FUSION_RATE_RECOVERY   0x08U
#define APP_IMU_CAPTURE_FUSION_STARTUP         0x10U

/* flight_flags bits */
#define APP_IMU_CAPTURE_FLIGHT_ARMED           0x01U
#define APP_IMU_CAPTURE_FLIGHT_GYRO_BIAS_READY 0x02U

/*
 * Block magic. Deliberately NOT four printable characters: the previous value
 * spelled "IMUC", which is exactly how the "IMUCAP DUMP ok ..." status line
 * begins, so a host resynchronising on the magic would lock onto that text and
 * parse ASCII as header fields. The two high bytes here cannot occur in the
 * 7-bit status text, so binary blocks are unambiguous.
 */
#define APP_IMU_CAPTURE_MAGIC        0xA5C3494DUL /* 'M','I',0xC3,0xA5 */
#define APP_IMU_CAPTURE_VERSION      3U
/*
 * 6144 samples = 6.14 s at 1 kHz. At 48 B/sample that is 288 KB, held in the
 * 512 KB AXI SRAM via .ram_d1_noinit (CPU-only, no DMA reachability needed);
 * DTCM is far too small and is where the RTOS stacks live. 6 s gives 0.16 Hz
 * FFT resolution, well beyond what is needed to separate rotor tones, and
 * leaves roughly 190 KB of the region free. Each throttle step is captured
 * separately anyway, so a longer single window buys nothing.
 */
#define APP_IMU_CAPTURE_CAPACITY     6144U

typedef enum {
    APP_IMU_CAPTURE_IDLE = 0,
    APP_IMU_CAPTURE_RECORDING = 1,
    APP_IMU_CAPTURE_FULL = 2,
    APP_IMU_CAPTURE_EXPORTING = 3,
} APP_IMU_CaptureState;

typedef enum {
    APP_IMU_CAPTURE_CMD_OK = 0,
    APP_IMU_CAPTURE_CMD_BUSY,
    APP_IMU_CAPTURE_CMD_EMPTY,
    APP_IMU_CAPTURE_CMD_NO_LINK,
    APP_IMU_CAPTURE_CMD_INVALID,
} APP_IMU_CaptureCommandStatus;

typedef struct {
    APP_IMU_CaptureState state;
    uint32_t stored;          /* samples currently held                       */
    uint32_t capacity;
    uint32_t requested;       /* target sample count for this run             */
    uint32_t dropped;         /* samples lost because the buffer filled       */
    uint32_t export_sent;
    uint32_t session_id;
    uint16_t accel_aaf_hz;    /* AAF actually programmed, for provenance      */
    uint16_t gyro_aaf_hz;
    uint16_t accel_range_g;
    uint16_t gyro_range_dps;
} APP_IMU_CaptureStatus;

/* Wire header prefixed to every exported block. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint32_t session_id;
    uint32_t total_samples;
    uint32_t offset_samples;
    uint16_t block_samples;
    uint16_t sample_size;
    uint16_t accel_aaf_hz;
    uint16_t gyro_aaf_hz;
    uint16_t accel_range_g;
    uint16_t gyro_range_dps;
    uint32_t flags;
    uint32_t payload_crc32;
} APP_IMU_CaptureBlockHeader;

#define APP_IMU_CAPTURE_FLAG_LAST 0x00000001UL

void APP_IMU_Capture_Init(void);

/*
 * Producer hook for the raw sensor half. Called from Sensor_Task once per IMU
 * frame, before scaling/alignment/LPF. Never blocks and never allocates; a
 * no-op unless recording. This call is what advances the write index, so it
 * must come first for each frame.
 */
void APP_IMU_Capture_Push(uint32_t timestamp_us,
                          const DRV_IMU_RawData *raw,
                          uint16_t motor_upper_us,
                          uint16_t motor_lower_us);

/*
 * Annotates the sample most recently pushed with filtered sensor values. Called
 * from Sensor_Task after the LPF so the filter's measured effect is recorded
 * alongside its input. Safe to omit; fields stay zero.
 */
void APP_IMU_Capture_AnnotateFiltered(const float accel_g[3],
                                      const float gyro_dps[3],
                                      uint8_t gyro_bias_ready);

/*
 * Annotates the most recent sample with estimator output, servo commands and
 * fusion health. Called from StabilizerTask, which runs one frame behind the
 * sensor task, so this lands on the sample it actually corresponds to.
 */
void APP_IMU_Capture_AnnotateControl(float roll_deg,
                                     float pitch_deg,
                                     float yaw_deg,
                                     uint16_t servo_alpha_us,
                                     uint16_t servo_beta_us,
                                     float accel_error_deg,
                                     uint8_t fusion_flags,
                                     uint8_t armed);

APP_IMU_CaptureCommandStatus APP_IMU_Capture_Start(uint32_t sample_count);
APP_IMU_CaptureCommandStatus APP_IMU_Capture_Stop(void);
APP_IMU_CaptureCommandStatus APP_IMU_Capture_StartDump(void);
APP_IMU_CaptureCommandStatus APP_IMU_Capture_CancelDump(void);

void APP_IMU_Capture_GetStatus(APP_IMU_CaptureStatus *status);
uint8_t APP_IMU_Capture_IsExportActive(void);

/* Drives one export block. Call from a low-priority task, never from the
 * sample producer. */
void APP_IMU_Capture_ExportStep(void);

const char *APP_IMU_Capture_CommandStatusText(
    APP_IMU_CaptureCommandStatus status);
const char *APP_IMU_Capture_StateText(APP_IMU_CaptureState state);

#ifdef __cplusplus
}
#endif

#endif /* APP_IMU_CAPTURE_H */
