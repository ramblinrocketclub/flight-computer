#include <stdint.h>
#include <stddef.h>
#include "ringbuffer.h"

uint8_t ringbuf_init(ringbuf_t *ringbuf, uint8_t *buf_data, size_t size)
{
    if (ringbuf == NULL || buf_data == NULL || size == 0)
    {
        return 0;
    }

    ringbuf->size = size;
    ringbuf->start = 0;
    ringbuf->end = 0;
    ringbuf->buf = buf_data;

    return 1;
}

void ringbuf_put(ringbuf_t *ringbuf, uint8_t data)
{
   ringbuf->buf[ringbuf->end++] = data;
   ringbuf->end %= ringbuf->size;
}

uint8_t ringbuf_get(ringbuf_t *ringbuf)
{
    uint8_t data = ringbuf->buf[ringbuf->start++];
    ringbuf->start %= ringbuf->size;
    return data;
}

