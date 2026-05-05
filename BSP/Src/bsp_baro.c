#include "bsp_baro.h"

#include "main.h"
#include "spi.h"

#include <string.h>

static BSP_SPL06_Device baro_dev;
static uint8_t baro_initialized;
static uint8_t baro_bound;

static BSP_SPL06_Bus baro_make_bus(void)
{
    BSP_SPL06_Bus bus = {
        .hspi = &hspi4,
        .cs_port = Press_cs_GPIO_Port,
        .cs_pin = Press_cs_Pin,
        .timeout_ms = 100U,
        .delay_ms = HAL_Delay,
    };

    return bus;
}

static void baro_bind_bus(void)
{
    BSP_SPL06_Bus bus;

    if (baro_bound != 0U) {
        return;
    }

    bus = baro_make_bus();

    memset(&baro_dev, 0, sizeof(baro_dev));
    baro_dev.bus = bus;
    baro_bound = 1U;
}

static BSP_SPL06_Status baro_validate_id(uint8_t product_id)
{
    return (product_id == BSP_SPL06_ID_VALUE) ? BSP_SPL06_OK : BSP_SPL06_BAD_ID;
}

BSP_SPL06_Status BSP_BARO_Init(void)
{
    BSP_SPL06_Bus bus = baro_make_bus();
    BSP_SPL06_Status status;

    if (baro_initialized != 0U) {
        return BSP_SPL06_OK;
    }

    status = BSP_SPL06_Init(&baro_dev, &bus);
    if (status == BSP_SPL06_OK) {
        baro_initialized = 1U;
        baro_bound = 1U;
    }

    return status;
}

BSP_SPL06_Status BSP_BARO_ProbeId(uint8_t *product_id)
{
    BSP_SPL06_Status status;

    baro_bind_bus();

    status = BSP_SPL06_ReadId(&baro_dev, product_id);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    return baro_validate_id(*product_id);
}

BSP_SPL06_Status BSP_BARO_ProbeIdTxRx(uint8_t *product_id)
{
    BSP_SPL06_Status status;

    baro_bind_bus();

    status = BSP_SPL06_ReadIdTxRx(&baro_dev, product_id);
    if (status != BSP_SPL06_OK) {
        return status;
    }

    return baro_validate_id(*product_id);
}

BSP_SPL06_Status BSP_BARO_ReadId(uint8_t *product_id)
{
    if (baro_initialized == 0U) {
        return BSP_SPL06_ERROR;
    }

    return BSP_SPL06_ReadId(&baro_dev, product_id);
}

BSP_SPL06_Status BSP_BARO_ReadRawRegister(uint8_t reg, uint8_t *value)
{
    baro_bind_bus();
    return BSP_SPL06_ReadRegister(&baro_dev, reg, value);
}

const BSP_SPL06_Device *BSP_BARO_GetDevice(void)
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
