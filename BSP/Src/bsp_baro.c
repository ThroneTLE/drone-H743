#include "bsp_baro.h"
#include "bsp_board.h"

#include "main.h"

#include <string.h>

static DRV_BARO_Device baro_dev;
static uint8_t         baro_initialized;
static uint8_t         baro_bound;

static void baro_bind_bus(void)
{
    if (baro_bound != 0U) { return; }
    memset(&baro_dev, 0, sizeof(baro_dev));
    baro_dev.bus = *BSP_Board_GetBaroBus();
    baro_bound = 1U;
}

DRV_BARO_Status BSP_BARO_Init(void)
{
    DRV_BARO_Status status;

    if (baro_initialized != 0U) { return DRV_BARO_OK; }

    status = DRV_BARO_Init(&baro_dev, BSP_Board_GetBaroBus());
    if (status == DRV_BARO_OK) { baro_initialized = 1U; baro_bound = 1U; }
    return status;
}

DRV_BARO_Status BSP_BARO_ProbeId(uint8_t *product_id)
{
    DRV_BARO_Status status;
    baro_bind_bus();
    status = DRV_BARO_ReadId(&baro_dev, product_id);
    if (status != DRV_BARO_OK) { return status; }
    return (*product_id == DRV_BARO_ID_VALUE) ? DRV_BARO_OK : DRV_BARO_BAD_ID;
}

DRV_BARO_Status BSP_BARO_ProbeIdTxRx(uint8_t *product_id)
{
    DRV_BARO_Status status;
    baro_bind_bus();
    status = DRV_BARO_ReadIdTxRx(&baro_dev, product_id);
    if (status != DRV_BARO_OK) { return status; }
    return (*product_id == DRV_BARO_ID_VALUE) ? DRV_BARO_OK : DRV_BARO_BAD_ID;
}

DRV_BARO_Status BSP_BARO_ReadId(uint8_t *product_id)
{
    if (baro_initialized == 0U) { return DRV_BARO_ERROR; }
    return DRV_BARO_ReadId(&baro_dev, product_id);
}

DRV_BARO_Status BSP_BARO_ReadRawRegister(uint8_t reg, uint8_t *value)
{
    baro_bind_bus();
    return DRV_BARO_ReadRegister(&baro_dev, reg, value);
}

DRV_BARO_Status BSP_BARO_ReadRawRegisters(uint8_t reg, uint8_t *data, uint16_t len)
{
    baro_bind_bus();
    return DRV_BARO_ReadRegisters(&baro_dev, reg, data, len);
}

const DRV_BARO_Device *BSP_BARO_GetDevice(void)
{
    baro_bind_bus();
    return &baro_dev;
}

void BSP_BARO_DebugReadLevels(uint8_t *cs_level, uint8_t *miso_level)
{
    if (cs_level != NULL) {
        *cs_level = (uint8_t)HAL_GPIO_ReadPin(Press_cs_GPIO_Port, Press_cs_Pin);
    }
    if (miso_level != NULL) {
        *miso_level = (uint8_t)HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);
    }
}

void BSP_BARO_Invalidate(void)
{
    memset(&baro_dev, 0, sizeof(baro_dev));
    baro_initialized = 0U;
    baro_bound = 0U;
}
