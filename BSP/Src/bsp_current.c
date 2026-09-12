#include "bsp_current.h"
#include "adc.h"
#include <stddef.h>

static uint8_t current_initialized;

BSP_CurrentStatus BSP_Current_Init(void)
{
    current_initialized = 0U;
    if (hadc1.Instance != ADC1 || hadc1.Init.Resolution != ADC_RESOLUTION_16B ||
        hadc1.Init.NbrOfConversion != 1U || hadc1.Init.ContinuousConvMode != DISABLE ||
        hadc1.Init.ConversionDataManagement != ADC_CONVERSIONDATA_DR ||
        hadc1.Init.LeftBitShift != ADC_LEFTBITSHIFT_NONE || hadc1.Init.OversamplingMode != DISABLE) {
        return BSP_CURRENT_ERROR;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                                   ADC_SINGLE_ENDED) != HAL_OK) {
        return BSP_CURRENT_ERROR;
    }
    current_initialized = 1U;
    return BSP_CURRENT_OK;
}

BSP_CurrentStatus BSP_Current_Read(uint32_t *raw)
{
    if (raw == NULL) { return BSP_CURRENT_ERROR; }
    if (!current_initialized) { return BSP_CURRENT_NOT_READY; }
    if (HAL_ADC_Start(&hadc1) != HAL_OK) { return BSP_CURRENT_ERROR; }
    HAL_StatusTypeDef status = HAL_ADC_PollForConversion(&hadc1, 1U);
    uint32_t value = 0U;
    uint32_t error = HAL_ADC_GetError(&hadc1);
    if (status == HAL_OK && error == HAL_ADC_ERROR_NONE) { value = HAL_ADC_GetValue(&hadc1); }
    HAL_StatusTypeDef stop = HAL_ADC_Stop(&hadc1);
    if (stop != HAL_OK || error != HAL_ADC_ERROR_NONE) { return BSP_CURRENT_ERROR; }
    if (status == HAL_TIMEOUT) { return BSP_CURRENT_TIMEOUT; }
    if (status != HAL_OK || value > 65535U) { return BSP_CURRENT_ERROR; }
    *raw = value;
    return BSP_CURRENT_OK;
}
