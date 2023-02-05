#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "ringbuffer.h"

uint32_t Min(uint32_t a, uint32_t b)
{
    uint32_t _a = a;
    uint32_t _b = b;
    return (_a < _b) ? _a : _b;
}

uint8_t RingBuffer_Init(RingBuffer_t *pRingBuffer, uint8_t *pBuffer, size_t buffer_size)
{
    if (pRingBuffer == NULL || pBuffer == NULL || buffer_size == 0)
    {
        return 1;
    }

    pRingBuffer->buffer = pBuffer;
    pRingBuffer->size   = buffer_size;
    pRingBuffer->head   = 0;
    pRingBuffer->tail   = 0;
    pRingBuffer->count  = 0;

    return 0;
}

uint8_t RingBuffer_Put(RingBuffer_t *pRingBuffer, uint8_t *pData, size_t data_size)
{
    if (pRingBuffer->size - pRingBuffer->count < data_size)
    {
        return 1;
    }

    uint32_t first_part     = Min(data_size, pRingBuffer->size - pRingBuffer->head);
    uint32_t second_part    = data_size - first_part;

    memcpy(pRingBuffer->buffer + pRingBuffer->head, pData,              first_part);
    memcpy(pRingBuffer->buffer,                     pData + first_part, second_part);

    pRingBuffer->head = (pRingBuffer->head + data_size) % pRingBuffer->size;
    pRingBuffer->count += data_size;

    return 0;
}


uint8_t RingBuffer_Get(RingBuffer_t *pRingBuffer, uint8_t *pData, size_t data_size)
{
    if (pRingBuffer->count < data_size)
    {
        return 1;
    }

    uint32_t first_part     = Min(data_size, pRingBuffer->size - pRingBuffer->tail);
    uint32_t second_part    = data_size - first_part;

    memcpy(pData,               pRingBuffer->buffer + pRingBuffer->tail,    first_part);
    memcpy(pData + first_part,  pRingBuffer->buffer,                        second_part);

    pRingBuffer->tail = (pRingBuffer->tail + data_size) % pRingBuffer->size;
    pRingBuffer->count -= data_size;

    return 0;
}

