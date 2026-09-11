#include "bsp_i2c.h"

#include "i2c.h"

/* 每笔诊断事务的超时。比传感器路径宽松：卡住也只影响这条命令。 */
#define BSP_I2C_DEBUG_TIMEOUT_MS 10U

/* 扫描时每个地址只给很短的时间——总线上大多数地址是空的，等待会被放大 118 倍。 */
#define BSP_I2C_SCAN_TIMEOUT_MS   2U
#define BSP_I2C_SCAN_FIRST_ADDR   0x08U
#define BSP_I2C_SCAN_LAST_ADDR    0x77U

I2C_HandleTypeDef *BSP_I2C_GetHandle(uint8_t bus_index)
{
    switch (bus_index) {
    case 1U: return &hi2c1;
    case 2U: return &hi2c2;
    default: return NULL;
    }
}

static BSP_I2C_Status bsp_i2c_from_hal(I2C_HandleTypeDef *hi2c,
                                       HAL_StatusTypeDef status)
{
    if (status == HAL_OK) { return BSP_I2C_OK; }
    if (status == HAL_TIMEOUT) { return BSP_I2C_TIMEOUT; }

    /*
     * 把 NACK 单独分出来：地址无人应答（器件不在/地址填错）与总线本身出错
     * （SCL 被拉死、仲裁丢失）排查方向完全不同，合并成一个 ERROR 等于没说。
     */
    if ((hi2c != NULL) &&
        ((hi2c->ErrorCode & HAL_I2C_ERROR_AF) != 0U)) {
        return BSP_I2C_NACK;
    }
    return BSP_I2C_ERROR;
}

BSP_I2C_Status BSP_I2C_DebugXfer(uint8_t bus_index, uint8_t addr7,
                                 const uint8_t *tx, uint16_t tx_len,
                                 uint8_t *rx, uint16_t rx_len)
{
    I2C_HandleTypeDef *hi2c = BSP_I2C_GetHandle(bus_index);
    uint16_t addr8 = (uint16_t)((uint16_t)addr7 << 1U);
    HAL_StatusTypeDef hal;

    if ((hi2c == NULL) || (addr7 > 0x7FU) ||
        (tx_len > (uint16_t)BSP_I2C_XFER_MAX) ||
        (rx_len > (uint16_t)BSP_I2C_XFER_MAX) ||
        ((tx_len != 0U) && (tx == NULL)) ||
        ((rx_len != 0U) && (rx == NULL)) ||
        ((tx_len == 0U) && (rx_len == 0U))) {
        return BSP_I2C_INVALID_ARG;
    }

    /*
     * "写 1~2 字节再读"是读寄存器的标准形态，交给 HAL_I2C_Mem_Read——它发的是
     * 写地址 → **重复起始** → 读。中间若插一个 STOP，有些器件会把地址指针丢掉，
     * 读回来就是错的。所以这条路径单独走，不用 Transmit + Receive 拼。
     */
    if ((tx_len != 0U) && (rx_len != 0U) && (tx_len <= 2U)) {
        uint16_t mem_addr = (tx_len == 1U)
            ? (uint16_t)tx[0]
            : (uint16_t)(((uint16_t)tx[0] << 8U) | (uint16_t)tx[1]);
        uint16_t mem_size = (tx_len == 1U) ? I2C_MEMADD_SIZE_8BIT
                                           : I2C_MEMADD_SIZE_16BIT;

        hal = HAL_I2C_Mem_Read(hi2c, addr8, mem_addr, mem_size, rx, rx_len,
                               BSP_I2C_DEBUG_TIMEOUT_MS);
        return bsp_i2c_from_hal(hi2c, hal);
    }

    if (tx_len != 0U) {
        hal = HAL_I2C_Master_Transmit(hi2c, addr8, (uint8_t *)(uintptr_t)tx,
                                      tx_len, BSP_I2C_DEBUG_TIMEOUT_MS);
        if (hal != HAL_OK) { return bsp_i2c_from_hal(hi2c, hal); }
    }

    if (rx_len != 0U) {
        /*
         * 走到这里要么是纯读，要么是"写超过 2 字节再读"——后者中间会有 STOP。
         * 这是有意的取舍：超过 2 字节的地址不是寄存器读的常见形态，与其猜，
         * 不如让行为可预测，需要重复起始时把写长度压到 2 以内即可。
         */
        hal = HAL_I2C_Master_Receive(hi2c, addr8, rx, rx_len,
                                     BSP_I2C_DEBUG_TIMEOUT_MS);
        return bsp_i2c_from_hal(hi2c, hal);
    }

    return BSP_I2C_OK;
}

uint8_t BSP_I2C_DebugScan(uint8_t bus_index, uint8_t *found, uint8_t found_max)
{
    I2C_HandleTypeDef *hi2c = BSP_I2C_GetHandle(bus_index);
    uint8_t count = 0U;
    uint8_t addr;

    if ((hi2c == NULL) || (found == NULL) || (found_max == 0U)) { return 0U; }

    for (addr = BSP_I2C_SCAN_FIRST_ADDR; addr <= BSP_I2C_SCAN_LAST_ADDR; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)((uint16_t)addr << 1U), 1U,
                                  BSP_I2C_SCAN_TIMEOUT_MS) == HAL_OK) {
            if (count < found_max) {
                found[count] = addr;
                count++;
            }
        }
    }

    return count;
}
