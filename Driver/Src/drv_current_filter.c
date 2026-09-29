#include "drv_current_filter.h"

#include <math.h>
#include <stddef.h>

static void filter_start_block(DRV_CurrentFilter *filter)
{
    filter->sum_a = 0.0f;
    filter->block_min_a = 0.0f;
    filter->block_max_a = 0.0f;
    filter->block_count = 0U;
}

void DRV_CurrentFilter_Init(DRV_CurrentFilter *filter, uint16_t window)
{
    if (filter == NULL) { return; }
    filter_start_block(filter);
    filter->mean_a = 0.0f;
    filter->min_a = 0.0f;
    filter->max_a = 0.0f;
    filter->valid = 0U;
    /* 0 会让下面的"凑满就出块"永远不触发；按 1 处理等价于不平均。 */
    filter->window = (window == 0U) ? 1U : window;
    filter->blocks = 0U;
    filter->rejected = 0U;
}

void DRV_CurrentFilter_Reset(DRV_CurrentFilter *filter)
{
    if (filter == NULL) { return; }
    filter_start_block(filter);
    /* 结果一并失效：调用方是在配置变了之后调它的，旧均值不再代表当前量程。 */
    filter->valid = 0U;
}

void DRV_CurrentFilter_Push(DRV_CurrentFilter *filter, float current_a)
{
    if (filter == NULL) { return; }

    if (!isfinite(current_a)) {
        /* 见头文件：NaN 是"这一拍没有有效读数"，不是 0 安培。 */
        filter->rejected++;
        return;
    }

    if (filter->block_count == 0U) {
        filter->block_min_a = current_a;
        filter->block_max_a = current_a;
    } else if (current_a < filter->block_min_a) {
        filter->block_min_a = current_a;
    } else if (current_a > filter->block_max_a) {
        filter->block_max_a = current_a;
    } else {
        /* 落在区间内，min/max 不变。 */
    }

    filter->sum_a += current_a;
    filter->block_count++;

    if (filter->block_count < filter->window) {
        return;
    }

    filter->mean_a = filter->sum_a / (float)filter->block_count;
    filter->min_a = filter->block_min_a;
    filter->max_a = filter->block_max_a;
    filter->valid = 1U;
    filter->blocks++;
    filter_start_block(filter);
}
