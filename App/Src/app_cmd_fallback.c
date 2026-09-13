/*
 * 命令兜底链：`app_control.c` 那串 if-else 全都没认领时，落到这里。
 *
 * 存在的理由是硬约束：`App/Src/app_control.c` **只减不增**。在它里面再加一条
 * `else if (strcmp(tokens[0], "LED") == 0)` 就是往一个已经超限的文件里加行。
 * 原来那条末尾的 else 只有一句"报个未知命令"，把它换成一次调用，行数不变，
 * 而之后**任何**新命令族都只要在本文件的链上挂一个函数，再不用碰 app_control.c。
 *
 * 链上每个处理函数认领了就返回 1，没认领返回 0 继续往下走。顺序无所谓——
 * 各家认的是自己的命令字，不存在谁盖谁。
 */

#include "app_control.h"
#include "app_control_internal.h"
#include "app_components.h"

#include <stddef.h>

void app_control_handle_unclaimed(char **tokens, uint32_t count)
{
    if ((tokens == NULL) || (count == 0U)) {
        return;
    }
    if (app_control_handle_led(tokens, count) != 0U) {
        return;
    }
    if (APP_Components_Command(tokens, count) != 0U) { return; }
    APP_Control_QueueText("ERR unknown cmd %s\r\n", tokens[0]);
}
