#ifndef DRV_BARO_H
#define DRV_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdint.h>

#define DRV_BARO_ID_VALUE 0x10U

typedef enum {
    DRV_BARO_OK = 0,
    DRV_BARO_ERROR,
    DRV_BARO_TIMEOUT,
    DRV_BARO_BAD_ID,
    DRV_BARO_INVALID_ARG
} DRV_BARO_Status;

/*
 * 同一颗 SPL06 在两块板子上挂在不同总线：
 *   老板子   —— SPI4，片选 Press_cs
 *   MicoAir  —— I2C2，地址 0x77（见 doc/micoair743v2/vendor/ardupilot-hwdef.dat）
 * 寄存器逻辑完全相同，只有搬运方式不同，所以这里两套句柄并存：
 * hi2c 非空走 I2C，否则走 SPI。
 *
 * 注：上游对芯片型号有分歧（ArduPilot 记 SPL06、Betaflight 与 INAV 记 DPS310），
 * 但两者的 PROD_ID 都是 0x10、寄存器布局兼容，因此同一份驱动通吃，不必分支。
 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_address;   /* 7 位地址，0 表示不用 I2C */
    uint32_t           timeout_ms;
    void             (*delay_ms)(uint32_t ms);
} DRV_BARO_Bus;

typedef struct {
    DRV_BARO_Bus bus;
    uint8_t      product_id;
} DRV_BARO_Device;

DRV_BARO_Status DRV_BARO_Init(DRV_BARO_Device *dev, const DRV_BARO_Bus *bus);
DRV_BARO_Status DRV_BARO_ReadId(DRV_BARO_Device *dev, uint8_t *product_id);
DRV_BARO_Status DRV_BARO_ReadIdTxRx(DRV_BARO_Device *dev, uint8_t *product_id);
DRV_BARO_Status DRV_BARO_ReadRegister(DRV_BARO_Device *dev, uint8_t reg, uint8_t *value);
DRV_BARO_Status DRV_BARO_ReadRegisters(DRV_BARO_Device *dev, uint8_t reg,
                                       uint8_t *data, uint16_t len);
DRV_BARO_Status DRV_BARO_WriteRegister(DRV_BARO_Device *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
