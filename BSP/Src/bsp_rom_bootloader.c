#include "bsp_rom_bootloader.h"

#include "stm32h7xx_hal.h"

typedef struct {
    uint8_t dbp_was_enabled;
    uint8_t rtc_apb_was_enabled;
} BSP_RomBootloaderBackupState;

static uint8_t rom_bootloader_open_backup(BSP_RomBootloaderBackupState *state)
{
    volatile uint32_t readback;
    uint32_t attempts;

    /*
     * STM32H743 上 PWR 复位后就是通的，RCC 也没有 PWREN 位。这里保留条件调用，
     * 是为了让源码在确实有 PWR 时钟门的 STM32 变种上也能直接编过，
     * 而不是去发明一个 H743 上并不存在的位。
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

static void rom_bootloader_close_backup(
    const BSP_RomBootloaderBackupState *state)
{
    if (state->dbp_was_enabled == 0U) {
        HAL_PWR_DisableBkUpAccess();
    }
    if (state->rtc_apb_was_enabled == 0U) {
        __HAL_RCC_RTC_CLK_DISABLE();
    }
    __DSB();
}

static uint8_t rom_bootloader_magic_present(uint32_t magic)
{
    return ((RTC->BKP0R == magic) &&
            (RTC->BKP1R == (uint32_t)~magic)) ? 1U : 0U;
}

static void rom_bootloader_clear_magic(void)
{
    RTC->BKP0R = 0U;
    RTC->BKP1R = 0U;
    __DSB();
}

void BSP_RomBootloader_ReadVector(uint32_t vector_address,
                                  uint32_t *initial_msp,
                                  uint32_t *reset_handler)
{
    const volatile uint32_t *const vector =
        (const volatile uint32_t *)vector_address;

    if ((initial_msp == NULL) || (reset_handler == NULL)) {
        return;
    }

    *initial_msp = vector[0];
    *reset_handler = vector[1];
}

uint8_t BSP_RomBootloader_WriteRequestMagic(uint32_t magic)
{
    BSP_RomBootloaderBackupState state;
    uint8_t written;

    if (rom_bootloader_open_backup(&state) == 0U) {
        return 0U;
    }
    RTC->BKP0R = magic;
    RTC->BKP1R = (uint32_t)~magic;
    __DSB();
    written = rom_bootloader_magic_present(magic);
    rom_bootloader_close_backup(&state);
    return written;
}

uint8_t BSP_RomBootloader_TakeRequestMagic(uint32_t magic)
{
    BSP_RomBootloaderBackupState state;
    uint8_t present;

    if (rom_bootloader_open_backup(&state) == 0U) {
        return 0U;
    }

    present = rom_bootloader_magic_present(magic);
    if (present != 0U) {
        /*
         * 先清再让调用方去校验向量、去跳。任何一步失败都会正常启动应用，
         * 而不是又跳一次——"永远进 DFU"是要拆机才救得回来的状态。
         */
        rom_bootloader_clear_magic();
    }
    rom_bootloader_close_backup(&state);
    return present;
}

static void rom_bootloader_quiesce(void)
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
 * r0/r1 进来分别是 MSP 与复位向量。用裸函数是为了让编译器不要在换掉 MSP 之后
 * 再生成任何尾声代码或栈访问。只允许从 main 最早的钩子里调用——那时复位刚把
 * 栈指针选成 MSP（绝不会是 RTOS 的 PSP）；进入 ROM 之前显式开中断。
 */
static void __attribute__((naked, noreturn)) rom_bootloader_branch(
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

void BSP_RomBootloader_Jump(uint32_t vector_address,
                            uint32_t initial_msp,
                            uint32_t reset_handler)
{
    __disable_irq();
    rom_bootloader_quiesce();
    SCB->VTOR = vector_address;
    __DSB();
    __ISB();
    rom_bootloader_branch(initial_msp, reset_handler);
}

void BSP_RomBootloader_SystemReset(void)
{
    NVIC_SystemReset();
}
