#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_gpio.h"
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_spi.h"
#include "bsp_gd25q32.h"
#include "bsp_flash.h"
#include "bsp_spl06.h"
#include "bsp_baro.h"
#include "bsp_icm42688.h"
#include "bsp_imu.h"
#include "bsp_pwm.h"
#include "bsp_bus_servo.h"
#include "bsp_system.h"
#include "bsp_aiwb2_power.h"

void BSP_Init(void);

#ifdef __cplusplus
}
#endif

#endif
