#include "bsp_cache.h"
#include "stm32h7xx_hal.h"

void BSP_Cache_Enable(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
}

void BSP_Cache_Disable(void)
{
    SCB_DisableICache();
    SCB_DisableDCache();
}

uint32_t BSP_Cache_AlignDown32(const void *addr)
{
    return ((uint32_t)(uintptr_t)addr) & ~31UL;
}

uint32_t BSP_Cache_AlignedSize32(const void *addr, uint32_t len)
{
    uint32_t start = BSP_Cache_AlignDown32(addr);
    uint32_t end   = ((uint32_t)(uintptr_t)addr + len + 31UL) & ~31UL;

    return end - start;
}

void BSP_Cache_CleanDCache(const void *addr, uint32_t len)
{
    if (len == 0U) { return; }

    SCB_CleanDCache_by_Addr(
        (uint32_t *)BSP_Cache_AlignDown32(addr),
        (int32_t)BSP_Cache_AlignedSize32(addr, len));
}

void BSP_Cache_InvalidateDCache(const void *addr, uint32_t len)
{
    if (len == 0U) { return; }

    SCB_InvalidateDCache_by_Addr(
        (uint32_t *)BSP_Cache_AlignDown32(addr),
        (int32_t)BSP_Cache_AlignedSize32(addr, len));
}
