#ifndef APP_BOOT_H
#define APP_BOOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* STM32H743 system-memory vector table for the factory USB DFU loader. */
#define APP_BOOT_ROM_DFU_VECTOR_ADDRESS 0x1FF09800UL
#define APP_BOOT_ROM_ADDRESS_MIN        0x1FF00000UL
#define APP_BOOT_ROM_ADDRESS_MAX        0x1FF20000UL

/* Leave enough time for the scheduled reply to leave the current CDC link. */
#define APP_BOOT_DFU_SCHEDULE_DELAY_MS 750U
#define APP_BOOT_ESC_SAFE_MAX_US       1100U
#define APP_BOOT_SNAPSHOT_MAX_AGE_US   100000ULL
#define APP_BOOT_DFU_REQUEST_MAGIC     0x44465531UL /* "DFU1" */

typedef enum {
    APP_BOOT_STATE_IDLE = 0U,
    APP_BOOT_STATE_WAITING_USB_REPLY = 1U,
    APP_BOOT_STATE_SCHEDULED = 2U,
    APP_BOOT_STATE_ENTERING = 3U,
} APP_BootState;

typedef enum {
    APP_BOOT_SAFETY_OK = 0U,
    APP_BOOT_SAFETY_NO_VALID_SNAPSHOT = 1U,
    APP_BOOT_SAFETY_ARMED = 2U,
    APP_BOOT_SAFETY_ESC_HIGH = 3U,
    APP_BOOT_SAFETY_SNAPSHOT_STALE = 4U,
} APP_BootSafety;

typedef enum {
    APP_BOOT_REQUEST_READY_FOR_USB = 0U,
    APP_BOOT_REQUEST_ALREADY_PENDING = 1U,
    APP_BOOT_REQUEST_NO_VALID_SNAPSHOT = 2U,
    APP_BOOT_REQUEST_ARMED = 3U,
    APP_BOOT_REQUEST_ESC_HIGH = 4U,
    APP_BOOT_REQUEST_VECTOR_INVALID = 5U,
    APP_BOOT_REQUEST_SNAPSHOT_STALE = 6U,
} APP_BootRequestResult;

typedef enum {
    APP_BOOT_EVENT_NONE = 0U,
    APP_BOOT_EVENT_CANCELLED_NO_SNAPSHOT = 1U,
    APP_BOOT_EVENT_CANCELLED_ARMED = 2U,
    APP_BOOT_EVENT_CANCELLED_ESC_HIGH = 3U,
    APP_BOOT_EVENT_VECTOR_INVALID = 4U,
    APP_BOOT_EVENT_MAGIC_WRITE_FAILED = 5U,
    APP_BOOT_EVENT_CANCELLED_SNAPSHOT_STALE = 6U,
    APP_BOOT_EVENT_CANCELLED_SEQUENCE_STALLED = 7U,
    APP_BOOT_EVENT_ESC_DISABLE_FAILED = 8U,
} APP_BootEvent;

typedef struct {
    APP_BootState state;
    APP_BootSafety safety;
    uint32_t deadline_ms;
    uint32_t remaining_ms;
    uint32_t snapshot_age_us;
    uint32_t snapshot_sequence;
    uint32_t request_sequence;
    uint32_t vector_msp;
    uint32_t vector_reset;
    uint16_t esc_pulse_us[2];
    uint8_t snapshot_valid;
    uint8_t armed;
    uint8_t vector_valid;
} APP_BootStatus;

void APP_Boot_Init(void);

/*
 * Call from main() USER CODE BEGIN 1, before MPU/cache/HAL/RTOS setup.  It
 * returns during an ordinary boot.  With a valid reset-persistent request it
 * clears the request and branches to ROM DFU from reset-time MSP context.
 */
void APP_Boot_TryRomDfu(void);

/*
 * Pure validation helpers are public so the safety contract can be exercised
 * by host tests without jumping to system memory.
 */
APP_BootSafety APP_Boot_EvaluateSafety(uint8_t snapshot_valid,
                                       uint8_t armed,
                                       uint16_t esc_1_us,
                                       uint16_t esc_2_us);
uint8_t APP_Boot_IsVectorReasonable(uint32_t initial_msp,
                                    uint32_t reset_handler);
uint8_t APP_Boot_IsSnapshotFresh(uint64_t now_us, uint64_t timestamp_us);
uint8_t APP_Boot_HasSequenceAdvanced(uint32_t current_sequence,
                                     uint32_t request_sequence);

void APP_Boot_GetStatus(APP_BootStatus *status);
/* Reserves one request and records its snapshot sequence; no countdown yet. */
APP_BootRequestResult APP_Boot_RequestDfu(void);
/* Call only after the scheduled USB CDC reply reports transmit completion. */
uint8_t APP_Boot_ConfirmDfuScheduled(void);
void APP_Boot_CancelDfuRequest(void);

/*
 * This service never jumps on the command-processing task stack.  A successful
 * request remains scheduled for at least APP_BOOT_DFU_SCHEDULE_DELAY_MS, is
 * safety-checked again, writes a reset-persistent request, then resets.  The
 * early main hook performs the actual ROM entry from reset-time MSP context.
 */
APP_BootEvent APP_Boot_Tick(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_BOOT_H */
