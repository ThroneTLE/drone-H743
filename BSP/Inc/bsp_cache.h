#ifndef BSP_CACHE_H
#define BSP_CACHE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Cache_Enable(void);
void BSP_Cache_Disable(void);

uint32_t BSP_Cache_AlignDown32(const void *addr);
uint32_t BSP_Cache_AlignedSize32(const void *addr, uint32_t len);
void BSP_Cache_CleanDCache(const void *addr, uint32_t len);
void BSP_Cache_InvalidateDCache(const void *addr, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
