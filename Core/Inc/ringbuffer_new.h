#ifndef RINGBUFFER_NEW_H
#define RINGBUFFER_NEW_H

typedef struct
{
    uint8_t *buffer;
    size_t   size;
    size_t   head;
    size_t   tail;
    size_t   count;
} RingBuffer_t;

uint8_t RingBuffer_Init(RingBuffer_t *pRingBuffer, uint8_t *pBuffer, size_t buffer_size);
uint8_t RingBuffer_Put(RingBuffer_t *pRingBuffer, uint8_t *pData, size_t data_size);
uint8_t RingBuffer_Get(RingBuffer_t *pRingBuffer, uint8_t *pData, size_t data_size);

#endif /* RINGBUFFER_NEW_H */

