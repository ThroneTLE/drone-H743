#include "bsp_board.h"

#include "bsp_cache.h"

#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "usart.h"
#include "cmsis_os2.h"

static DRV_IMU_Bus imu_bus;
static DRV_BARO_Bus baro_bus;
static DRV_GD25Q32_Bus flash_bus;
static DRV_MAG_Bus mag_bus;
static DRV_GPS_Bus gps_bus;
static DRV_SERVO_Bus servo_bus;

void BSP_DelayMs(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

void BSP_Board_Init(void)
{
    imu_bus.hspi       = &hspi2;
    imu_bus.cs_port    = IMU_CS_GPIO_Port;
    imu_bus.cs_pin     = IMU_CS_Pin;
    imu_bus.timeout_ms = 100U;
    imu_bus.delay_ms   = BSP_DelayMs;

    baro_bus.hspi       = &hspi4;
    baro_bus.cs_port    = Press_cs_GPIO_Port;
    baro_bus.cs_pin     = Press_cs_Pin;
    baro_bus.timeout_ms = 100U;
    baro_bus.delay_ms   = BSP_DelayMs;

    flash_bus.hspi       = &hspi1;
    flash_bus.cs_port    = FLASH_CS_GPIO_Port;
    flash_bus.cs_pin     = FLASH_CS_Pin;
    flash_bus.timeout_ms = 100U;
    flash_bus.delay_ms   = BSP_DelayMs;
    flash_bus.cache_clean = BSP_Cache_CleanDCache;
    flash_bus.cache_invalidate = BSP_Cache_InvalidateDCache;

    mag_bus.hi2c       = &hi2c1;
    mag_bus.timeout_ms = 20U;

    gps_bus.huart      = &huart2;
    gps_bus.baud_rate  = 38400U;
    gps_bus.delay_ms   = BSP_DelayMs;

    servo_bus.huart      = &huart7;
    servo_bus.timeout_ms = 100U;
}

const DRV_IMU_Bus *BSP_Board_GetImuBus(void)     { return &imu_bus; }
const DRV_BARO_Bus *BSP_Board_GetBaroBus(void)   { return &baro_bus; }
const DRV_GD25Q32_Bus *BSP_Board_GetFlashBus(void) { return &flash_bus; }
const DRV_MAG_Bus *BSP_Board_GetMagBus(void)     { return &mag_bus; }
const DRV_GPS_Bus *BSP_Board_GetGpsBus(void)     { return &gps_bus; }
const DRV_SERVO_Bus *BSP_Board_GetServoBus(void) { return &servo_bus; }
