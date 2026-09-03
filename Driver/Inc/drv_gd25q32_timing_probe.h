#ifndef DRV_GD25Q32_TIMING_PROBE_H
#define DRV_GD25Q32_TIMING_PROBE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_GD25Q32_TIMING_PROBE_MAGIC 0x544C4646UL /* FFLT */
#define DRV_GD25Q32_TIMING_PROBE_VERSION 3U
#define DRV_GD25Q32_TIMING_SAMPLE_CAPACITY 64U

typedef struct {
    uint32_t count;
    uint32_t min_us;
    uint32_t max_us;
    uint32_t sum_us;
    uint32_t samples_us[DRV_GD25Q32_TIMING_SAMPLE_CAPACITY];
} DRV_GD25Q32_TimingSeries;

/*
 * Debugger-readable measurement block.  Every field is 32-bit so OpenOCD
 * `mdw` can decode it without target-ABI padding assumptions.
 */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t tight_poll_enabled;
    uint32_t pending_page_block_kb;
    uint32_t suspend_resume_enabled;
    uint32_t suspend_success_count;
    uint32_t resume_success_count;
    uint32_t handshake_error_count;
    uint32_t last_suspend_status1;
    uint32_t last_suspend_status2;
    uint32_t last_resume_status1;
    uint32_t last_resume_status2;
    DRV_GD25Q32_TimingSeries erase_32k;
    DRV_GD25Q32_TimingSeries erase_64k;
    DRV_GD25Q32_TimingSeries page_after_32k;
    DRV_GD25Q32_TimingSeries page_after_64k;
    DRV_GD25Q32_TimingSeries suspend_to_ready;
    DRV_GD25Q32_TimingSeries resume_to_running;
    DRV_GD25Q32_TimingSeries erase_4k;
} DRV_GD25Q32_TimingProbe;

extern volatile DRV_GD25Q32_TimingProbe g_drv_gd25q32_timing_probe;

uint8_t DRV_GD25Q32_TimingProbe_TightPollEnabled(void);
uint8_t DRV_GD25Q32_TimingProbe_SuspendResumeEnabled(void);
uint32_t DRV_GD25Q32_TimingProbe_StartCycles(void);
uint32_t DRV_GD25Q32_TimingProbe_ElapsedUs(uint32_t start_cycles);
void DRV_GD25Q32_TimingProbe_RecordBlock(uint32_t block_size,
                                         uint32_t elapsed_us);
void DRV_GD25Q32_TimingProbe_RecordPage(uint32_t elapsed_us);
void DRV_GD25Q32_TimingProbe_RecordSuspend(uint32_t elapsed_us,
                                           uint8_t status1,
                                           uint8_t status2);
void DRV_GD25Q32_TimingProbe_RecordResume(uint32_t elapsed_us,
                                          uint8_t status1,
                                          uint8_t status2);
void DRV_GD25Q32_TimingProbe_RecordHandshakeError(void);

#ifdef __cplusplus
}
#endif

#endif
