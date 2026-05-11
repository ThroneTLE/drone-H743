#ifndef APP_FLASH_H
#define APP_FLASH_H

#include <stdint.h>

typedef struct {
    uint8_t  report_done;
    int32_t  probe_status;
    int32_t  status1_status;
    int32_t  read_status;
    uint8_t  manufacturer_id;
    uint8_t  memory_type;
    uint8_t  capacity_id;
    uint8_t  status1;
} APP_Flash_Status;

void APP_Flash_ReportStartup(void);
void APP_Flash_RefreshStatus(void);
void APP_Flash_GetStatus(APP_Flash_Status *status);

#endif
