/*
 * 板级总线绑定 —— 换板子时唯一需要动的地方。
 *
 * 目标板：MicoAir743v2（STM32H743VIT6）。
 * 引脚与外设归属的权威是 drone-H743.ioc；本文件只做"哪个器件挂在哪条总线上"的绑定，
 * 上游板级定义见 doc/micoair743v2/vendor/ardupilot-hwdef.dat。
 *
 * 器件登记：
 *   BMI088   SPI2  CS: 陀螺 PD5 / 加计 PD4   DRDY: PC15 / PC14
 *   BMI270   SPI3  CS: PA15                  DRDY: PB7
 *   SPL06    I2C2  地址 0x77
 *   QMC5883L I2C2  地址 0x0D（板载；外接磁罗盘走 I2C1）
 *   MTF 光流 USART2（MicoLink 协议）
 *   GPS      USART3
 *   总线舵机 UART7（半双工单线，PE8）
 *   参数     片内 Flash Bank2 尾两个扇区
 *   飞行日志 SDMMC1 裸块
 *   外部 NOR 本板没有；SPI1 上的绑定保留，探测会如实失败
 */

#include "bsp_board.h"

#include "bsp_cache.h"

#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "sdmmc.h"
#include "usart.h"
#include "cmsis_os2.h"

static DRV_IMU_Bus imu_bus;
static DRV_BMI088_Bus bmi088_bus;
static DRV_BMI270_Bus bmi270_bus;
static DRV_BARO_Bus baro_bus;
static DRV_GD25Q32_Bus flash_bus;
static DRV_INTFLASH_Bus intflash_bus;
static DRV_SDBLOCK_Bus sdblock_bus;
static DRV_MAG_Bus mag_bus;
static DRV_GPS_Bus gps_bus;
static DRV_OPTICAL_FLOW_Bus optical_flow_bus;
static DRV_SERVO_Bus servo_bus;

#define BSP_IMU_SPI_TIMEOUT_MS 5U
#define BSP_BARO_I2C_ADDRESS   0x77U

void BSP_DelayMs(uint32_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

/*
 * H7 的 PC2 / PC3 在 LQFP100 封装上只引出 PC2_C / PC3_C，中间隔着一个模拟开关，
 * 默认**断开**：不闭合就只有 ADC 能用，数字外设看到的是悬空脚。
 *
 * SPI2 的 MISO 走 PC2_C、MOSI 走 PC3_C，所以两个开关都必须闭合，
 * 否则 BMI088 读回来的是随机值（而不是报错）。
 *
 * 放在 BSP 而不是 CubeMX 生成文件的 USER CODE 区：换板/重生成时这段逻辑跟着
 * 板级走，不会被 Generate Code 的行为影响，也不违反"不手改生成代码"的约束。
 */
static void BSP_Board_CloseAnalogSwitches(void)
{
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);
}

void BSP_Board_Init(void)
{
    BSP_Board_CloseAnalogSwitches();

    /*
     * BMI088：加计与陀螺是同封装里的两颗独立芯片，各有片选，共用 SPI2。
     * 两个 CS 都必须给，缺一个探测就会读到另一颗的寄存器空间。
     */
    bmi088_bus.hspi          = &hspi2;
    bmi088_bus.acc_cs_port   = BMI088_A_CS_GPIO_Port;
    bmi088_bus.acc_cs_pin    = BMI088_A_CS_Pin;
    bmi088_bus.gyro_cs_port  = BMI088_G_CS_GPIO_Port;
    bmi088_bus.gyro_cs_pin   = BMI088_G_CS_Pin;
    bmi088_bus.timeout_ms    = BSP_IMU_SPI_TIMEOUT_MS;
    bmi088_bus.delay_ms      = BSP_DelayMs;

    bmi270_bus.hspi          = &hspi3;
    bmi270_bus.cs_port       = BMI270_CS_GPIO_Port;
    bmi270_bus.cs_pin        = BMI270_CS_Pin;
    bmi270_bus.timeout_ms    = BSP_IMU_SPI_TIMEOUT_MS;
    bmi270_bus.delay_ms      = BSP_DelayMs;

    /*
     * ICM-42688 不在这块板上，绑定保留是为了老板子仍能用同一份固件。
     * 片选借用 IMU_CS 标签（本板上是 AT7456E 的 OSD 片选，挂在 SPI1）。
     * 探测时拉动它不会干扰 SPI2 上的 BMI088——OSD 不在这条总线上——
     * 而 SPI2 上没有 ICM 应答，探测会如实失败并跳过。
     */
    imu_bus.hspi       = &hspi2;
    imu_bus.cs_port    = IMU_CS_GPIO_Port;
    imu_bus.cs_pin     = IMU_CS_Pin;
    imu_bus.timeout_ms = BSP_IMU_SPI_TIMEOUT_MS;
    imu_bus.delay_ms   = BSP_DelayMs;

    /* SPL06 从老板子的 SPI4 换到 I2C2；寄存器逻辑不变，只是搬运方式变了。 */
    baro_bus.hspi        = NULL;
    baro_bus.cs_port     = NULL;
    baro_bus.cs_pin      = 0U;
    baro_bus.miso_port   = NULL;   /* I2C 没有 MISO，诊断电平报"不适用" */
    baro_bus.miso_pin    = 0U;
    baro_bus.hi2c        = &hi2c2;
    baro_bus.i2c_address = BSP_BARO_I2C_ADDRESS;
    baro_bus.timeout_ms  = 100U;
    baro_bus.delay_ms    = BSP_DelayMs;

    /*
     * 外部 SPI NOR：本板没有。保留绑定让诊断页能如实报告"探测失败"，
     * 而不是编译期把整条链路挖掉——老板子还要用。
     */
    flash_bus.hspi       = &hspi1;
    flash_bus.cs_port    = FLASH_CS_GPIO_Port;
    flash_bus.cs_pin     = FLASH_CS_Pin;
    flash_bus.timeout_ms = 100U;
    flash_bus.delay_ms   = BSP_DelayMs;
    flash_bus.cache_clean = BSP_Cache_CleanDCache;
    flash_bus.cache_invalidate = BSP_Cache_InvalidateDCache;

    /* 参数落片内 Flash：擦写后必须失效 D-Cache，否则读回旧内容。 */
    intflash_bus.cache_invalidate = BSP_Cache_InvalidateDCache;

    /*
     * 飞行日志落 SD 裸块。这里**故意不挂缓存维护回调**：阻塞版 HAL_SD_ReadBlocks
     * 是 CPU 轮询 FIFO 搬运，不是 DMA，做维护反而会丢数据。原委见 drv_sdblock.h。
     */
    sdblock_bus.hsd              = &hsd1;
    sdblock_bus.timeout_ms       = 1000U;

    /*
     * 磁罗盘接 I2C2：板载 QMC5883L(0x0D) 与气压计(0x77) 同总线，地址不冲突。
     * 需要外接 IST8310 时改绑 &hi2c1（板上 I2C1 就是外部磁罗盘口，PB8/PB9），
     * drv_mag 会自动探测型号，不用改驱动。
     */
    mag_bus.hi2c       = &hi2c2;
    mag_bus.timeout_ms = 20U;

    /* GPS 从 USART2 分家到 USART3（本板的默认 GPS 口），光流独占 USART2。 */
    gps_bus.huart      = &huart3;
    gps_bus.baud_rate  = 38400U;
    gps_bus.delay_ms   = BSP_DelayMs;

    optical_flow_bus.huart = &huart2;
    optical_flow_bus.baud_rate = DRV_OPTICAL_FLOW_BAUD_RATE;
    optical_flow_bus.timeout_ms = 100U;
    optical_flow_bus.delay_ms = BSP_DelayMs;
    optical_flow_bus.cache_invalidate = BSP_Cache_InvalidateDCache;

    /* 总线舵机：UART7 半双工单线，PE8。这是新旧两块板唯一引脚同址的外设。 */
    servo_bus.huart      = &huart7;
    servo_bus.timeout_ms = 100U;
}

const DRV_IMU_Bus *BSP_Board_GetImuBus(void)     { return &imu_bus; }
const DRV_BMI088_Bus *BSP_Board_GetBmi088Bus(void) { return &bmi088_bus; }
const DRV_BMI270_Bus *BSP_Board_GetBmi270Bus(void) { return &bmi270_bus; }
const DRV_BARO_Bus *BSP_Board_GetBaroBus(void)   { return &baro_bus; }
const DRV_GD25Q32_Bus *BSP_Board_GetFlashBus(void) { return &flash_bus; }
const DRV_INTFLASH_Bus *BSP_Board_GetIntFlashBus(void) { return &intflash_bus; }
const DRV_SDBLOCK_Bus *BSP_Board_GetSdBlockBus(void) { return &sdblock_bus; }
const DRV_MAG_Bus *BSP_Board_GetMagBus(void)     { return &mag_bus; }
const DRV_GPS_Bus *BSP_Board_GetGpsBus(void)     { return &gps_bus; }
const DRV_OPTICAL_FLOW_Bus *BSP_Board_GetOpticalFlowBus(void) { return &optical_flow_bus; }
const DRV_SERVO_Bus *BSP_Board_GetServoBus(void) { return &servo_bus; }
