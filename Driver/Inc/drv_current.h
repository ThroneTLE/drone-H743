#ifndef DRV_CURRENT_H
#define DRV_CURRENT_H
#include <stdint.h>

/* Pure conversion; no HAL, device access or mutable global state.
 * Voltages are V, current is A. Offset belongs to the ESC Curr output, not ADC.
 * input_scale = ESC Curr voltage / ADC pin voltage; nominal direct input = 1.
 */
typedef struct {
    float reference_v;
    float sensitivity_v_per_a;
    float zero_offset_v;
    float input_scale;
    uint32_t adc_max;
    uint8_t calibrated; /* external measurement confirmed; never inferred from raw ADC */
} DRV_CurrentConfig;

typedef struct {
    uint32_t raw;
    float adc_v;
    float current_a;
    uint8_t valid;
    uint8_t saturated;
    uint8_t calibrated;
} DRV_CurrentReading;

typedef enum { DRV_CURRENT_OK = 0, DRV_CURRENT_INVALID, DRV_CURRENT_SATURATED } DRV_CurrentStatus;

/* Nominal values only: MicoAir AM32 55A manual specifies 12.75 mV/A.
 * https://micoair.cn/zh/docs/esc/55a-am32-esc
 * 3.3 V reference and direct analog input still require physical verification.
 */
DRV_CurrentConfig DRV_Current_Am32_55A_Default(void);
/* Invalid config/raw leaves out untouched. ADC high rail returns a reading with
 * saturated=1, valid=0, current_a=NaN; raw/adc_v remain available for diagnosis.
 * Low rail is not proof of disconnect: true zero current is legitimate.
 * Negative offset-corrected current is not silently clamped to zero.
 */
DRV_CurrentStatus DRV_Current_Convert(const DRV_CurrentConfig *config,
                                    uint32_t raw, DRV_CurrentReading *out);
#endif
