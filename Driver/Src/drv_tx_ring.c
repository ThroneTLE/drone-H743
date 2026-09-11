#include "drv_tx_ring.h"

#include <string.h>

void DRV_TxRing_Init(DRV_TxRing *ring, uint8_t *buffer, uint32_t size)
{
    if (ring == NULL) {
        return;
    }

    if ((buffer == NULL) || (size == 0U)) {
        ring->buffer = NULL;
        ring->size   = 0U;
    } else {
        ring->buffer = buffer;
        ring->size   = size;
    }
    ring->head = 0U;
    ring->tail = 0U;
    ring->used = 0U;
}

uint32_t DRV_TxRing_Used(const DRV_TxRing *ring)
{
    if ((ring == NULL) || (ring->buffer == NULL)) {
        return 0U;
    }

    return ring->used;
}

uint32_t DRV_TxRing_Free(const DRV_TxRing *ring)
{
    if ((ring == NULL) || (ring->buffer == NULL)) {
        return 0U;
    }

    return ring->size - ring->used;
}

uint8_t DRV_TxRing_Push(DRV_TxRing *ring, const uint8_t *data, uint32_t length)
{
    uint32_t first;

    if ((ring == NULL) || (ring->buffer == NULL) || (data == NULL)) {
        return 0U;
    }
    if (length == 0U) {
        return 1U;   /* 没有要写的东西也算成功，调用方不必到处判空 */
    }
    if (length > (ring->size - ring->used)) {
        return 0U;   /* 全有或全无：塞不下就一个字节都不写 */
    }

    /* head 到缓冲末尾能放多少，剩下的从头绕回去。 */
    first = ring->size - ring->head;
    if (first > length) {
        first = length;
    }
    memcpy(&ring->buffer[ring->head], data, first);
    if (length > first) {
        memcpy(&ring->buffer[0], &data[first], length - first);
    }

    ring->head = ring->head + length;
    if (ring->head >= ring->size) {
        ring->head -= ring->size;
    }
    ring->used += length;
    return 1U;
}

uint32_t DRV_TxRing_PeekContiguous(const DRV_TxRing *ring, const uint8_t **chunk)
{
    uint32_t length;

    if ((ring == NULL) || (ring->buffer == NULL) || (chunk == NULL) ||
        (ring->used == 0U)) {
        return 0U;
    }

    length = ring->size - ring->tail;
    if (length > ring->used) {
        length = ring->used;
    }

    *chunk = &ring->buffer[ring->tail];
    return length;
}

void DRV_TxRing_Release(DRV_TxRing *ring, uint32_t length)
{
    if ((ring == NULL) || (ring->buffer == NULL) || (length == 0U)) {
        return;
    }

    if (length > ring->used) {
        length = ring->used;
    }

    ring->tail = ring->tail + length;
    if (ring->tail >= ring->size) {
        ring->tail -= ring->size;
    }
    ring->used -= length;
}
