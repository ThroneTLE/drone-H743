#ifndef BSP_DSHOT_H
#define BSP_DSHOT_H
#include <stdint.h>

typedef enum {
    BSP_DSHOT_OK = 0,
    BSP_DSHOT_BUSY,
    BSP_DSHOT_INVALID,
    BSP_DSHOT_ERROR
} BSP_DShotStatus;

typedef struct {
    uint32_t submitted;
    uint32_t completed; /* DMA completed, not ESC acknowledgement */
    uint32_t busy_rejected;
    uint32_t errors;
    uint32_t cancelled;
    uint32_t timer_clock_hz;
    uint16_t code[2];   /* last accepted code; unit=DShot code, not us */
    uint8_t enabled_mask;
    uint8_t busy;
    uint8_t fault;
} BSP_DShotSnapshot;

/* Init only at boot or explicit reinitialization. Never call from Set/Submit.
 * M4/PE9 = channel 1; M3/PE11 = channel 2. Init holds both outputs inactive.
 */
BSP_DShotStatus BSP_DShot_Init(void);
/* Atomic two-channel frame, no queue; mask bit0/1 correspond to channel1/2.
 * mask=0 means hard disable, code=0 with an enabled bit means a STOP frame.
 * Caller is the 500 Hz final actuator commit. Busy does not touch DMA memory.
 */
BSP_DShotStatus BSP_DShot_Submit(const uint16_t code[2], uint8_t enabled_mask);
/* Immediate physical disable; usable with interrupts masked, never waits for IRQ.
 * Pending completion cannot re-enable output. Returns ERROR if DMA won't stop.
 */
BSP_DShotStatus BSP_DShot_Disable(uint8_t channel_mask);
void BSP_DShot_GetSnapshot(BSP_DShotSnapshot *out);

#endif
