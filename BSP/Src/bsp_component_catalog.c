#include "bsp_component_catalog.h"
#include "drv_component_proto.h"
#include "drv_imu_types.h"

/* Same binding as bsp_board.c and the generated .ioc; no bus probe here. */
const char *BSP_Component_Interface(uint16_t id, uint8_t variant)
{
    switch (id) {
    case DRV_COMPONENT_IMU:
        return variant==DRV_IMU_CHIP_BMI088?"SPI2":
               (variant==DRV_IMU_CHIP_BMI270?"SPI3":"unselected");
    case DRV_COMPONENT_BARO:return "I2C2 / 0x77";
    case DRV_COMPONENT_MAG:return "I2C2";
    case DRV_COMPONENT_FLOW:return "UART4";
    case DRV_COMPONENT_CURRENT:return "ADC1 / PC1";
    case DRV_COMPONENT_BATTERY:return "ADC1 / PC0";
    case DRV_COMPONENT_PARAMS:return "Flash Bank2";
    case DRV_COMPONENT_LOG:return "SDMMC1";
    case DRV_COMPONENT_ESC:return "TIM1 / M4,M3";
    case DRV_COMPONENT_SERVO:return variant?"TIM4 / M7,M8":"UART7";
    case DRV_COMPONENT_RC:return "USART6";
    case DRV_COMPONENT_UART:return "USART1";
    case DRV_COMPONENT_BT:return "UART8";
    case DRV_COMPONENT_LED:return "PE3,PE2,PE4";
    case DRV_COMPONENT_GPS:return "USART3";
    default:return "";
    }
}
