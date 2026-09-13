#ifndef APP_COMPONENTS_H
#define APP_COMPONENTS_H
#include "drv_component_proto.h"
typedef void (*APP_ComponentSample)(DRV_ComponentRecord *out);
/* Register cached-state samplers (no I/O/blocking). Same ID+callback is idempotent;
 * duplicate IDs with another callback/full registry are rejected. */
uint8_t APP_Components_Register(uint16_t id, APP_ComponentSample sample);
void APP_Components_Init(void);
uint8_t APP_Components_Command(char **tokens, uint32_t count);
void APP_Components_Report(uint32_t nonce);
/* Board/application binding and transport seams; command task only. */
void APP_Components_RegisterBoard(void);
uint8_t APP_Components_Send(const uint8_t *payload, uint16_t length);
uint8_t APP_Components_ExportBusy(void);
uint32_t APP_Components_NowMs(void);
void APP_Components_RegisterGps(void);
#endif
