#include "bsp_baro.h"
#include "bsp_board.h"

#include "main.h"

#include <string.h>

static DRV_BARO_Device baro_dev;
static uint8_t         baro_initialized;
static uint8_t         baro_bound;

uint8_t BSP_BARO_IsInitialized(void) { return baro_initialized; }

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

/*
 * 电平取自**当前绑定的总线**，不再写死某块板子的引脚标签。
 * 以前这里直接用 Press_cs_* 宏，那是老板子 SPI4 的 CubeMX 标签；MicoAir 上气压计
 * 改挂 I2C2，这个标签不再生成，函数会连编译都过不去。
 */
void BSP_BARO_DebugReadLevels(uint8_t *cs_level, uint8_t *miso_level)
{
    uint8_t cs = BSP_BARO_LEVEL_NOT_APPLICABLE;
    uint8_t miso = BSP_BARO_LEVEL_NOT_APPLICABLE;

    baro_bind_bus();

    if (baro_dev.bus.hi2c == NULL) {
        if (baro_dev.bus.cs_port != NULL) {
            cs = (uint8_t)HAL_GPIO_ReadPin(baro_dev.bus.cs_port,
                                           baro_dev.bus.cs_pin);
        }
        if (baro_dev.bus.miso_port != NULL) {
            miso = (uint8_t)HAL_GPIO_ReadPin(baro_dev.bus.miso_port,
                                             baro_dev.bus.miso_pin);
        }
    }

    if (cs_level != NULL)   { *cs_level = cs; }
    if (miso_level != NULL) { *miso_level = miso; }
}

void BSP_BARO_Invalidate(void)
{
    memset(&baro_dev, 0, sizeof(baro_dev));
    baro_initialized = 0U;
    baro_bound = 0U;
}
