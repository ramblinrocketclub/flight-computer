/**
  ******************************************************************************
  * @file    ringbuffer.h
  * @brief   This file contains the rinbuffer struct and functions.
  ******************************************************************************
  */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

typedef struct ringbuf {
    volatile uint8_t *buf;
    uint32_t size;
    uint32_t end;
    uint32_t start;
} ringbuf_t;

uint8_t ringbuf_init(ringbuf_t *ringbuf, uint8_t *buf_data, size_t size);

/**
 * @brief Inserts an element into a specified ring buffer
 *
 * @param ringbuf   a pointer to a ring buffer
 * @param data      byte to be inserted into the ring buffer
 */

void    ringbuf_put(ringbuf_t *ringbuf, uint8_t data);

/**
 * @brief Gets the current element pointed to by the read pointer
 *
 * @param ringbuf   a pointer to a ring buffer
 */

uint8_t ringbuf_get(ringbuf_t *ringbuf);

#endif /* RINGBUFFER_H */

