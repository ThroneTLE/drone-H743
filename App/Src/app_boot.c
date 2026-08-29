#include "app_boot.h"

#include "app_stabilizer.h"
#include "app_usb_cdc.h"
#include "bsp_pwm.h"
#include "svc_timestamp.h"
#include "usb_device.h"
#include "usbd_core.h"

#include "stm32h7xx_hal.h"

#include <stddef.h>
#include <string.h>

/* CubeMX defines this handle in USB_DEVICE/App/usb_device.c. */
extern USBD_HandleTypeDef hUsbDeviceFS;

static volatile APP_BootState app_boot_state;
static volatile uint64_t app_boot_deadline_us;
static volatile uint32_t app_boot_request_sequence;

typedef struct {
    uint64_t timestamp_us;
    uint64_t age_us;
    uint32_t sequence;
    uint16_t esc_pulse_us[2];
    uint8_t snapshot_valid;
    uint8_t armed;
} APP_BootSafetySample;

typedef struct {
    uint8_t dbp_was_enabled;
    uint8_t rtc_apb_was_enabled;
} APP_BootBackupAccessState;

static uint8_t app_boot_time_reached(uint64_t now_us, uint64_t deadline_us)
{
    return (now_us >= deadline_us) ? 1U : 0U;
}

static uint8_t app_boot_msp_in_sram(uint32_t initial_msp)
{
    if ((initial_msp & 0x7U) != 0U) {
        return 0U;
    }

    /* DTCM, D1 AXI SRAM, D2 SRAM1/2/3 and D3 SRAM4 on STM32H743. */
    if (((initial_msp >= 0x20000000UL) && (initial_msp <= 0x20020000UL)) ||
        ((initial_msp >= 0x24000000UL) && (initial_msp <= 0x24080000UL)) ||
        ((initial_msp >= 0x30000000UL) && (initial_msp <= 0x30048000UL)) ||
        ((initial_msp >= 0x38000000UL) && (initial_msp <= 0x38010000UL))) {
        return 1U;
    }

    return 0U;
}

APP_BootSafety APP_Boot_EvaluateSafety(uint8_t snapshot_valid,
                                       uint8_t armed,
                                       uint16_t esc_1_us,
                                       uint16_t esc_2_us)
{
    if (snapshot_valid == 0U) {
        return APP_BOOT_SAFETY_NO_VALID_SNAPSHOT;
    }
    if (armed != 0U) {
        return APP_BOOT_SAFETY_ARMED;
    }
    if ((esc_1_us > APP_BOOT_ESC_SAFE_MAX_US) ||
        (esc_2_us > APP_BOOT_ESC_SAFE_MAX_US)) {
        return APP_BOOT_SAFETY_ESC_HIGH;
    }
    return APP_BOOT_SAFETY_OK;
}

uint8_t APP_Boot_IsSnapshotFresh(uint64_t now_us, uint64_t timestamp_us)
{
    if ((timestamp_us == 0ULL) || (now_us < timestamp_us)) {
        return 0U;
    }
    return ((now_us - timestamp_us) <= APP_BOOT_SNAPSHOT_MAX_AGE_US) ? 1U : 0U;
}

uint8_t APP_Boot_HasSequenceAdvanced(uint32_t current_sequence,
                                     uint32_t request_sequence)
{
    return ((int32_t)(current_sequence - request_sequence) > 0) ? 1U : 0U;
}

uint8_t APP_Boot_IsVectorReasonable(uint32_t initial_msp,
                                    uint32_t reset_handler)
{
    uint32_t reset_address = reset_handler & ~1UL;

    if ((initial_msp == 0U) || (initial_msp == 0xFFFFFFFFUL) ||
        (reset_handler == 0U) || (reset_handler == 0xFFFFFFFFUL)) {
        return 0U;
    }
    if (app_boot_msp_in_sram(initial_msp) == 0U) {
        return 0U;
    }
    if ((reset_handler & 1U) == 0U) {
        return 0U;
    }
    if ((reset_address < APP_BOOT_ROM_ADDRESS_MIN) ||
        (reset_address >= APP_BOOT_ROM_ADDRESS_MAX)) {
        return 0U;
    }
    return 1U;
}

static void app_boot_read_vector(uint32_t *initial_msp,
                                 uint32_t *reset_handler)
{
    const volatile uint32_t *const vector =
        (const volatile uint32_t *)APP_BOOT_ROM_DFU_VECTOR_ADDRESS;

    *initial_msp = vector[0];
    *reset_handler = vector[1];
}

static APP_BootSafety app_boot_read_safety(uint64_t now_us,
                                           APP_BootSafetySample *sample)
{
    StabilizerValidationImuSnapshot snapshot;
    APP_BootSafety safety;

    memset(&snapshot, 0, sizeof(snapshot));
    memset(sample, 0, sizeof(*sample));
    sample->snapshot_valid = APP_Stabilizer_ReadValidationImuSnapshot(&snapshot);
    sample->timestamp_us = snapshot.timestamp_us;
    sample->sequence = snapshot.sequence;
    sample->armed = snapshot.armed;
    sample->esc_pulse_us[0] = snapshot.esc_pulse_us[0];
    sample->esc_pulse_us[1] = snapshot.esc_pulse_us[1];

    safety = APP_Boot_EvaluateSafety(sample->snapshot_valid,
                                     sample->armed,
                                     sample->esc_pulse_us[0],
                                     sample->esc_pulse_us[1]);
    if (safety != APP_BOOT_SAFETY_OK) {
        return safety;
    }
    if ((sample->timestamp_us != 0ULL) && (now_us >= sample->timestamp_us)) {
        sample->age_us = now_us - sample->timestamp_us;
    }
    if (APP_Boot_IsSnapshotFresh(now_us, sample->timestamp_us) == 0U) {
        return APP_BOOT_SAFETY_SNAPSHOT_STALE;
    }
    return APP_BOOT_SAFETY_OK;
}

static APP_BootRequestResult app_boot_request_from_safety(APP_BootSafety safety)
{
    switch (safety) {
    case APP_BOOT_SAFETY_NO_VALID_SNAPSHOT:
        return APP_BOOT_REQUEST_NO_VALID_SNAPSHOT;
    case APP_BOOT_SAFETY_ARMED:
        return APP_BOOT_REQUEST_ARMED;
    case APP_BOOT_SAFETY_ESC_HIGH:
        return APP_BOOT_REQUEST_ESC_HIGH;
    case APP_BOOT_SAFETY_SNAPSHOT_STALE:
        return APP_BOOT_REQUEST_SNAPSHOT_STALE;
    case APP_BOOT_SAFETY_OK:
    default:
        return APP_BOOT_REQUEST_READY_FOR_USB;
    }
}

static APP_BootEvent app_boot_event_from_safety(APP_BootSafety safety)
{
    switch (safety) {
    case APP_BOOT_SAFETY_NO_VALID_SNAPSHOT:
        return APP_BOOT_EVENT_CANCELLED_NO_SNAPSHOT;
    case APP_BOOT_SAFETY_ARMED:
        return APP_BOOT_EVENT_CANCELLED_ARMED;
    case APP_BOOT_SAFETY_ESC_HIGH:
        return APP_BOOT_EVENT_CANCELLED_ESC_HIGH;
    case APP_BOOT_SAFETY_SNAPSHOT_STALE:
        return APP_BOOT_EVENT_CANCELLED_SNAPSHOT_STALE;
    case APP_BOOT_SAFETY_OK:
    default:
        return APP_BOOT_EVENT_NONE;
    }
}

static uint8_t app_boot_enable_backup_register_access(
    APP_BootBackupAccessState *state)
{
    volatile uint32_t readback;
    uint32_t attempts;

    /*
     * STM32H743 keeps PWR active at startup and its RCC has no PWREN bit.  Keep
     * the conditional call for source compatibility with STM32 variants that
     * do expose a PWR clock gate, without inventing a nonexistent H743 bit.
     */
#if defined(__HAL_RCC_PWR_CLK_ENABLE)
    __HAL_RCC_PWR_CLK_ENABLE();
#endif
    state->dbp_was_enabled = ((PWR->CR1 & PWR_CR1_DBP) != 0U) ? 1U : 0U;
    state->rtc_apb_was_enabled =
        ((RCC->APB4ENR & RCC_APB4ENR_RTCAPBEN) != 0U) ? 1U : 0U;
    HAL_PWR_EnableBkUpAccess();
    attempts = 1024U;
    do {
        readback = PWR->CR1 & PWR_CR1_DBP;
        --attempts;
    } while ((readback == 0U) && (attempts != 0U));
    if (readback == 0U) {
        return 0U;
    }
    __DSB();
    __HAL_RCC_RTC_CLK_ENABLE();
    readback = RCC->APB4ENR & RCC_APB4ENR_RTCAPBEN;
    __DSB();
    return (readback != 0U) ? 1U : 0U;
}

static void app_boot_restore_backup_register_access(
    const APP_BootBackupAccessState *state)
{
    if (state->dbp_was_enabled == 0U) {
        HAL_PWR_DisableBkUpAccess();
    }
    if (state->rtc_apb_was_enabled == 0U) {
        __HAL_RCC_RTC_CLK_DISABLE();
    }
    __DSB();
}

static uint8_t app_boot_request_magic_present(void)
{
    return ((RTC->BKP0R == APP_BOOT_DFU_REQUEST_MAGIC) &&
            (RTC->BKP1R == (uint32_t)~APP_BOOT_DFU_REQUEST_MAGIC)) ? 1U : 0U;
}

static void app_boot_clear_request_magic(void)
{
    RTC->BKP0R = 0U;
    RTC->BKP1R = 0U;
    __DSB();
}

static uint8_t app_boot_write_request_magic(void)
{
    APP_BootBackupAccessState access_state;
    uint8_t written;

    if (app_boot_enable_backup_register_access(&access_state) == 0U) {
        return 0U;
    }
    RTC->BKP0R = APP_BOOT_DFU_REQUEST_MAGIC;
    RTC->BKP1R = (uint32_t)~APP_BOOT_DFU_REQUEST_MAGIC;
    __DSB();
    written = app_boot_request_magic_present();
    app_boot_restore_backup_register_access(&access_state);
    return written;
}

static void app_boot_disable_nvic_and_systick(void)
{
    uint32_t index;

    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;

    for (index = 0U; index < 8U; ++index) {
        NVIC->ICER[index] = 0xFFFFFFFFUL;
        NVIC->ICPR[index] = 0xFFFFFFFFUL;
    }
    __DSB();
    __ISB();
}

static void app_boot_disable_cache_and_mpu(void)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
        SCB_CleanInvalidateDCache();
        SCB_DisableDCache();
    }
    if ((SCB->CCR & SCB_CCR_IC_Msk) != 0U) {
        SCB_InvalidateICache();
        SCB_DisableICache();
    }
    HAL_MPU_Disable();
    __DSB();
    __ISB();
}

/*
 * r0/r1 arrive as MSP/reset-handler.  Naked assembly prevents a compiler
 * epilogue or stack access after MSP changes.  This function is called only
 * from the earliest main hook, where reset has selected MSP (never an RTOS
 * PSP), and explicitly unmasks interrupts before entering the ROM handler.
 */
static void __attribute__((naked, noreturn)) app_boot_branch_to_rom(
    uint32_t initial_msp __attribute__((unused)),
    uint32_t reset_handler __attribute__((unused)))
{
    __asm volatile(
        "msr msp, r0\n"
        "movs r2, #0\n"
        "msr control, r2\n"
        "isb\n"
        "cpsie i\n"
        "bx r1\n");
}

void APP_Boot_TryRomDfu(void)
{
    APP_BootBackupAccessState access_state;
    uint32_t initial_msp;
    uint32_t reset_handler;

    if (app_boot_enable_backup_register_access(&access_state) == 0U) {
        return;
    }
    if (app_boot_request_magic_present() == 0U) {
        app_boot_restore_backup_register_access(&access_state);
        return;
    }

    /* Clear before validating/jumping so any failure boots the app, not a loop. */
    app_boot_clear_request_magic();
    app_boot_restore_backup_register_access(&access_state);
    app_boot_read_vector(&initial_msp, &reset_handler);
    if (APP_Boot_IsVectorReasonable(initial_msp, reset_handler) == 0U) {
        return;
    }

    __disable_irq();
    app_boot_disable_nvic_and_systick();
    app_boot_disable_cache_and_mpu();
    SCB->VTOR = APP_BOOT_ROM_DFU_VECTOR_ADDRESS;
    __DSB();
    __ISB();
    app_boot_branch_to_rom(initial_msp, reset_handler);
}

void APP_Boot_Init(void)
{
    app_boot_deadline_us = 0ULL;
    app_boot_request_sequence = 0U;
    app_boot_state = APP_BOOT_STATE_IDLE;
}

void APP_Boot_GetStatus(APP_BootStatus *status)
{
    APP_BootSafetySample sample;
    uint64_t now_us;

    if (status == NULL) {
        return;
    }

    now_us = SVC_Timestamp_Us();
    memset(status, 0, sizeof(*status));
    status->state = app_boot_state;
    status->deadline_ms = (uint32_t)(app_boot_deadline_us / 1000ULL);
    status->safety = app_boot_read_safety(now_us, &sample);
    status->snapshot_valid = sample.snapshot_valid;
    status->armed = sample.armed;
    status->esc_pulse_us[0] = sample.esc_pulse_us[0];
    status->esc_pulse_us[1] = sample.esc_pulse_us[1];
    status->snapshot_age_us = (uint32_t)sample.age_us;
    status->snapshot_sequence = sample.sequence;
    status->request_sequence = app_boot_request_sequence;
    app_boot_read_vector(&status->vector_msp, &status->vector_reset);
    status->vector_valid = APP_Boot_IsVectorReasonable(status->vector_msp,
                                                       status->vector_reset);

    if ((status->state == APP_BOOT_STATE_SCHEDULED) &&
        (app_boot_time_reached(now_us, app_boot_deadline_us) == 0U)) {
        status->remaining_ms =
            (uint32_t)((app_boot_deadline_us - now_us + 999ULL) / 1000ULL);
    }
}

APP_BootRequestResult APP_Boot_RequestDfu(void)
{
    APP_BootSafetySample sample;
    APP_BootSafety safety;
    uint64_t now_us;
    uint32_t initial_msp;
    uint32_t reset_handler;
    uint32_t primask;

    now_us = SVC_Timestamp_Us();
    safety = app_boot_read_safety(now_us, &sample);
    if (safety != APP_BOOT_SAFETY_OK) {
        return app_boot_request_from_safety(safety);
    }

    app_boot_read_vector(&initial_msp, &reset_handler);
    if (APP_Boot_IsVectorReasonable(initial_msp, reset_handler) == 0U) {
        return APP_BOOT_REQUEST_VECTOR_INVALID;
    }

    /* The command can arrive from USB, USART1 or maintenance UART tasks. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (app_boot_state != APP_BOOT_STATE_IDLE) {
        if (primask == 0U) {
            __enable_irq();
        }
        return APP_BOOT_REQUEST_ALREADY_PENDING;
    }
    app_boot_request_sequence = sample.sequence;
    app_boot_deadline_us = 0ULL;
    app_boot_state = APP_BOOT_STATE_WAITING_USB_REPLY;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }

    return APP_BOOT_REQUEST_READY_FOR_USB;
}

uint8_t APP_Boot_ConfirmDfuScheduled(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if (app_boot_state != APP_BOOT_STATE_WAITING_USB_REPLY) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }
    app_boot_deadline_us = SVC_Timestamp_Us() +
                           ((uint64_t)APP_BOOT_DFU_SCHEDULE_DELAY_MS * 1000ULL);
    app_boot_state = APP_BOOT_STATE_SCHEDULED;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void APP_Boot_CancelDfuRequest(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (app_boot_state == APP_BOOT_STATE_WAITING_USB_REPLY) {
        app_boot_deadline_us = 0ULL;
        app_boot_request_sequence = 0U;
        app_boot_state = APP_BOOT_STATE_IDLE;
        __DMB();
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

static APP_BootEvent app_boot_fail_locked(APP_BootEvent event, uint32_t primask)
{
    app_boot_state = APP_BOOT_STATE_IDLE;
    app_boot_deadline_us = 0ULL;
    app_boot_request_sequence = 0U;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
    return event;
}

APP_BootEvent APP_Boot_Tick(void)
{
    APP_BootSafetySample sample;
    APP_BootSafety safety;
    APP_BootEvent event;
    BSP_PWM_Status esc_1_status;
    BSP_PWM_Status esc_2_status;
    uint64_t now_us;
    uint32_t initial_msp;
    uint32_t reset_handler;
    uint32_t primask;

    now_us = SVC_Timestamp_Us();
    if ((app_boot_state != APP_BOOT_STATE_SCHEDULED) ||
        (app_boot_time_reached(now_us, app_boot_deadline_us) == 0U)) {
        return APP_BOOT_EVENT_NONE;
    }

    /* From this point success never unmasks interrupts before system reset. */
    primask = __get_PRIMASK();
    __disable_irq();
    now_us = SVC_Timestamp_Us();
    if ((app_boot_state != APP_BOOT_STATE_SCHEDULED) ||
        (app_boot_time_reached(now_us, app_boot_deadline_us) == 0U)) {
        return app_boot_fail_locked(APP_BOOT_EVENT_NONE, primask);
    }

    safety = app_boot_read_safety(now_us, &sample);
    event = app_boot_event_from_safety(safety);
    if (event != APP_BOOT_EVENT_NONE) {
        return app_boot_fail_locked(event, primask);
    }
    if (APP_Boot_HasSequenceAdvanced(sample.sequence,
                                     app_boot_request_sequence) == 0U) {
        return app_boot_fail_locked(
            APP_BOOT_EVENT_CANCELLED_SEQUENCE_STALLED, primask);
    }
    app_boot_read_vector(&initial_msp, &reset_handler);
    if (APP_Boot_IsVectorReasonable(initial_msp, reset_handler) == 0U) {
        return app_boot_fail_locked(APP_BOOT_EVENT_VECTOR_INVALID, primask);
    }

    app_boot_state = APP_BOOT_STATE_ENTERING;
    __DMB();
    esc_1_status = BSP_PWM_DisableEsc(1U);
    esc_2_status = BSP_PWM_DisableEsc(2U);
    if ((esc_1_status != BSP_PWM_OK) || (esc_2_status != BSP_PWM_OK) ||
        (BSP_PWM_GetEscPulse(1U) != 0U) ||
        (BSP_PWM_GetEscPulse(2U) != 0U)) {
        return app_boot_fail_locked(APP_BOOT_EVENT_ESC_DISABLE_FAILED, primask);
    }

    if (app_boot_write_request_magic() == 0U) {
        return app_boot_fail_locked(APP_BOOT_EVENT_MAGIC_WRITE_FAILED, primask);
    }

    /* A hardware reset provides a clean MSP context for APP_Boot_TryRomDfu. */
    APP_USB_CDC_SetConfigured(0U);
    (void)USBD_Stop(&hUsbDeviceFS);
    (void)USBD_DeInit(&hUsbDeviceFS);
    __DSB();
    NVIC_SystemReset();

    for (;;) {
        __NOP();
    }
}
