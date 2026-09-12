#include "bsp_critical.h"

#include "main.h"

uint32_t BSP_Critical_Enter(void)
{
    uint32_t state = __get_PRIMASK();

    __disable_irq();
    return state;
}

void BSP_Critical_Exit(uint32_t state)
{
    /*
     * 恢复进入前的 PRIMASK，而不是无条件 `__enable_irq()`。
     * 无条件开中断在嵌套时会把外层临界区一起拆掉——本函数被调用的地方就有
     * 这种可能（例如中断里调了同样用临界区的发送队列）。
     */
    __set_PRIMASK(state);
}

void BSP_Critical_MemoryBarrier(void)
{
    __DMB();
}
