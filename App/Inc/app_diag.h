#ifndef APP_DIAG_H
#define APP_DIAG_H

#include <stdint.h>

typedef struct {
    uint8_t stack_overflow_seen;
    uint8_t malloc_failed_seen;
    char    stack_overflow_task[16];
    uint32_t malloc_failed_count;
} APP_DiagFaultInfo;

void APP_Diag_RecordStackOverflow(const char *task_name);
void APP_Diag_RecordMallocFailed(void);
void APP_Diag_GetFaultInfo(APP_DiagFaultInfo *info);

#endif
