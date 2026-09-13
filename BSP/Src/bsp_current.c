#include "bsp_current.h"
#include "adc.h"
#include <stddef.h>

static uint8_t current_initialized;
static BSP_VoltageSample voltage_sample={.status=BSP_CURRENT_NOT_READY};

BSP_CurrentStatus BSP_Current_Init(void)
{
    current_initialized = 0U;
    voltage_sample = (BSP_VoltageSample){.status=BSP_CURRENT_ERROR};
    if (hadc1.Instance != ADC1 || hadc1.Init.Resolution != ADC_RESOLUTION_16B ||
        hadc1.Init.NbrOfConversion != 2U || hadc1.Init.ContinuousConvMode != DISABLE ||
        hadc1.Init.ScanConvMode != ADC_SCAN_ENABLE || hadc1.Init.DiscontinuousConvMode != ENABLE ||
        hadc1.Init.NbrOfDiscConversion != 1U || hadc1.Init.EOCSelection != ADC_EOC_SINGLE_CONV ||
        hadc1.Init.ConversionDataManagement != ADC_CONVERSIONDATA_DR ||
        hadc1.Init.LeftBitShift != ADC_LEFTBITSHIFT_NONE || hadc1.Init.OversamplingMode != DISABLE) {
        return BSP_CURRENT_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                                   ADC_SINGLE_ENDED) != HAL_OK) {
        return BSP_CURRENT_ERROR;
    }
    current_initialized = 1U;
    voltage_sample.status = BSP_CURRENT_NOT_READY;
    return BSP_CURRENT_OK;
}

static BSP_CurrentStatus read_rank(uint32_t *value)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) { return BSP_CURRENT_ERROR; }
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 1U);
    uint32_t error = HAL_ADC_GetError(&hadc1);
    if (error != HAL_ADC_ERROR_NONE) { return BSP_CURRENT_ERROR; }
    if (status == HAL_TIMEOUT) { return BSP_CURRENT_TIMEOUT; }
    if (status != HAL_OK) { return BSP_CURRENT_ERROR; }
    *value = HAL_ADC_GetValue(&hadc1);
    return *value <= 65535U ? BSP_CURRENT_OK : BSP_CURRENT_ERROR;
}

BSP_CurrentStatus BSP_Current_Read(uint32_t *raw)
{
    if (raw == NULL) { return BSP_CURRENT_ERROR; }
    if (!current_initialized) { return BSP_CURRENT_NOT_READY; }
    uint32_t current_raw=0U, voltage_raw=0U;
    /* Discontinuous group size=1 holds DR between ranks even if preempted.
     * Stop only after the pair (or error), so every new pair starts at rank 1. */
    BSP_CurrentStatus status=read_rank(&current_raw);
    if (status==BSP_CURRENT_OK) { status=read_rank(&voltage_raw); }
    if (HAL_ADC_Stop(&hadc1)!=HAL_OK) { status=BSP_CURRENT_ERROR; }
    voltage_sample.sequence++;
    voltage_sample.status=status;
    if (status!=BSP_CURRENT_OK) { return status; }
    voltage_sample.raw=voltage_raw;
    *raw=current_raw;
    return BSP_CURRENT_OK;
}

void BSP_Current_GetVoltageSample(BSP_VoltageSample *out)
{
    if (out!=NULL) { *out=voltage_sample; }
}
