#include "app_diag.h"

#include <string.h>

static volatile uint8_t diag_stack_overflow_seen;
static volatile uint8_t diag_malloc_failed_seen;
static volatile uint32_t diag_malloc_failed_count;
static char diag_stack_overflow_task[16];

void APP_Diag_RecordStackOverflow(const char *task_name)
{
    diag_stack_overflow_seen = 1U;
    memset(diag_stack_overflow_task, 0, sizeof(diag_stack_overflow_task));
    if (task_name != NULL) {
        (void)strncpy(diag_stack_overflow_task,
                      task_name,
                      sizeof(diag_stack_overflow_task) - 1U);
    }
}

void APP_Diag_RecordMallocFailed(void)
{
    diag_malloc_failed_seen = 1U;
    diag_malloc_failed_count++;
}

void APP_Diag_GetFaultInfo(APP_DiagFaultInfo *info)
{
    if (info == NULL) {
        return;
    }

    info->stack_overflow_seen = diag_stack_overflow_seen;
    info->malloc_failed_seen = diag_malloc_failed_seen;
    info->malloc_failed_count = diag_malloc_failed_count;
    memcpy(info->stack_overflow_task,
           diag_stack_overflow_task,
           sizeof(info->stack_overflow_task));
}
