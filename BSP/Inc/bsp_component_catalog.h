#ifndef BSP_COMPONENT_CATALOG_H
#define BSP_COMPONENT_CATALOG_H
#include <stdint.h>
/* MicoAir743v2 binding names; variant is selected IMU kind or servo PWM mode. */
const char *BSP_Component_Interface(uint16_t id, uint8_t variant);
#endif
