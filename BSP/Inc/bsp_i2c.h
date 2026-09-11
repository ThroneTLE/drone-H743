#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "main.h"

#include <stdint.h>

/*
 * I2C 的板级出口。
 *
 * 这个文件原先只是一句占位注释——两颗 I2C 器件（SPL06 气压计、QMC5883L 磁罗盘）
 * 各自在自己的驱动里直接拿 hi2c2 用，没人需要板级抽象。现在需要了：诊断要能
 * 对任意地址打一笔事务、要能扫总线，而这些都得有人统一看着"哪条总线是哪个句柄"。
 *
 * 总线号用板上的实例编号（1 = I2C1 外接口，2 = I2C2 板载传感器），
 * 与丝印和 hwdef 一致，不另造一套编号。
 */

#define BSP_I2C_XFER_MAX 32U

typedef enum {
    BSP_I2C_OK = 0,
    BSP_I2C_INVALID_ARG,
    BSP_I2C_ERROR,
    BSP_I2C_TIMEOUT,
    BSP_I2C_NACK
} BSP_I2C_Status;

/* 总线号 → HAL 句柄。未知总线返回 NULL。 */
I2C_HandleTypeDef *BSP_I2C_GetHandle(uint8_t bus_index);

/*
 * 一笔"先写后读"事务：tx_len 为 0 时退化成纯读，rx_len 为 0 时退化成纯写。
 * addr 是 **7 位**地址，左移由本函数负责——调用方按数据手册上的地址填即可。
 */
BSP_I2C_Status BSP_I2C_DebugXfer(uint8_t bus_index, uint8_t addr7,
                                 const uint8_t *tx, uint16_t tx_len,
                                 uint8_t *rx, uint16_t rx_len);

/*
 * 扫总线：对 0x08~0x77 逐个发一次零长度写，把应答的地址填进 found[]。
 * 返回找到的个数；found_max 限制写入个数。等价于 Linux 的 i2cdetect。
 */
uint8_t BSP_I2C_DebugScan(uint8_t bus_index, uint8_t *found, uint8_t found_max);

#endif
