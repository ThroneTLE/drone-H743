#include "drv_gd25q32_timing_probe.h"

#include "drv_gd25q32.h"
#include "main.h"

#include <stddef.h>

_Static_assert(sizeof(DRV_GD25Q32_TimingProbe) == 1680U,
               "OpenOCD decoder requires a 420-word probe block");
_Static_assert(offsetof(DRV_GD25Q32_TimingProbe, tight_poll_enabled) == 8U,
               "OpenOCD tight-poll switch offset must stay stable");
_Static_assert(offsetof(DRV_GD25Q32_TimingProbe, suspend_resume_enabled) == 16U,
               "OpenOCD suspend/resume switch offset must stay stable");

volatile DRV_GD25Q32_TimingProbe g_drv_gd25q32_timing_probe = {
    .magic = DRV_GD25Q32_TIMING_PROBE_MAGIC,
    .version = DRV_GD25Q32_TIMING_PROBE_VERSION,
    .tight_poll_enabled = 0U,
    .pending_page_block_kb = 0U,
    .suspend_resume_enabled = 0U,
};

static void timing_probe_record(volatile DRV_GD25Q32_TimingSeries *series,
                                uint32_t elapsed_us)
{
    uint32_t index = series->count % DRV_GD25Q32_TIMING_SAMPLE_CAPACITY;

    series->samples_us[index] = elapsed_us;
    if ((series->count == 0U) || (elapsed_us < series->min_us)) {
        series->min_us = elapsed_us;
    }
    if ((series->count == 0U) || (elapsed_us > series->max_us)) {
        series->max_us = elapsed_us;
    }
    series->sum_us += elapsed_us;
    series->count++;
}

uint8_t DRV_GD25Q32_TimingProbe_TightPollEnabled(void)
{
    return (g_drv_gd25q32_timing_probe.tight_poll_enabled != 0U) ? 1U : 0U;
}

uint8_t DRV_GD25Q32_TimingProbe_SuspendResumeEnabled(void)
{
    return (g_drv_gd25q32_timing_probe.suspend_resume_enabled != 0U) ? 1U : 0U;
}

uint32_t DRV_GD25Q32_TimingProbe_StartCycles(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    return DWT->CYCCNT;
}

uint32_t DRV_GD25Q32_TimingProbe_ElapsedUs(uint32_t start_cycles)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000UL;
    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;

    return (cycles_per_us != 0U) ? (elapsed_cycles / cycles_per_us) : 0U;
}

void DRV_GD25Q32_TimingProbe_RecordBlock(uint32_t block_size,
                                         uint32_t elapsed_us)
{
    if (block_size == DRV_GD25Q32_BLOCK32K_SIZE) {
        timing_probe_record(&g_drv_gd25q32_timing_probe.erase_32k,
                            elapsed_us);
        g_drv_gd25q32_timing_probe.pending_page_block_kb = 32U;
    } else if (block_size == DRV_GD25Q32_BLOCK64K_SIZE) {
        timing_probe_record(&g_drv_gd25q32_timing_probe.erase_64k,
                            elapsed_us);
        g_drv_gd25q32_timing_probe.pending_page_block_kb = 64U;
    }
}

void DRV_GD25Q32_TimingProbe_RecordPage(uint32_t elapsed_us)
{
    if (g_drv_gd25q32_timing_probe.pending_page_block_kb == 32U) {
        timing_probe_record(&g_drv_gd25q32_timing_probe.page_after_32k,
                            elapsed_us);
    } else if (g_drv_gd25q32_timing_probe.pending_page_block_kb == 64U) {
        timing_probe_record(&g_drv_gd25q32_timing_probe.page_after_64k,
                            elapsed_us);
    } else {
        return;
    }
    g_drv_gd25q32_timing_probe.pending_page_block_kb = 0U;
}

void DRV_GD25Q32_TimingProbe_RecordSuspend(uint32_t elapsed_us,
                                           uint8_t status1,
                                           uint8_t status2)
{
    timing_probe_record(&g_drv_gd25q32_timing_probe.suspend_to_ready,
                        elapsed_us);
    g_drv_gd25q32_timing_probe.last_suspend_status1 = status1;
    g_drv_gd25q32_timing_probe.last_suspend_status2 = status2;
    g_drv_gd25q32_timing_probe.suspend_success_count++;
}

void DRV_GD25Q32_TimingProbe_RecordResume(uint32_t elapsed_us,
                                          uint8_t status1,
                                          uint8_t status2)
{
    timing_probe_record(&g_drv_gd25q32_timing_probe.resume_to_running,
                        elapsed_us);
    g_drv_gd25q32_timing_probe.last_resume_status1 = status1;
    g_drv_gd25q32_timing_probe.last_resume_status2 = status2;
    g_drv_gd25q32_timing_probe.resume_success_count++;
}

void DRV_GD25Q32_TimingProbe_RecordHandshakeError(void)
{
    g_drv_gd25q32_timing_probe.handshake_error_count++;
}
