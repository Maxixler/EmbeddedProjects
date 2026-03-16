/**
 * @file    ring_buffer.c
 * @brief   Thread-safe ring buffer (circular buffer) implementation.
 *
 * @details Implements all ring buffer operations declared in ring_buffer.h.
 *          This implementation is designed for embedded systems where one context
 *          (e.g., ISR) writes data and another context (e.g., main loop) reads it.
 *
 *          Thread-safety notes:
 *          - The 'count' variable is marked volatile and used as the single
 *            source of truth for buffer fullness/emptiness.
 *          - Single-producer, single-consumer (SPSC) scenarios are inherently
 *            safe because head is only modified by the producer and tail is
 *            only modified by the consumer.
 *          - For block operations that may be called from non-ISR context while
 *            an ISR modifies head/count, a brief critical section (interrupt
 *            disable) may be needed at the application level.
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "ring_buffer.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialize a ring buffer instance.
 */
ring_buffer_status_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, size_t size)
{
    if (rb == NULL || buffer == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    if (size == 0U)
    {
        return RING_BUFFER_ERROR;
    }

    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0U;
    rb->tail = 0U;
    rb->count = 0U;

    return RING_BUFFER_OK;
}

/**
 * @brief   Write a single byte into the ring buffer.
 */
ring_buffer_status_t ring_buffer_write(ring_buffer_t *rb, uint8_t byte)
{
    if (rb == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    if (rb->count >= rb->size)
    {
        return RING_BUFFER_FULL;
    }

    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1U) % rb->size;
    rb->count++;

    return RING_BUFFER_OK;
}

/**
 * @brief   Read a single byte from the ring buffer.
 */
ring_buffer_status_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte)
{
    if (rb == NULL || byte == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    if (rb->count == 0U)
    {
        return RING_BUFFER_EMPTY;
    }

    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1U) % rb->size;
    rb->count--;

    return RING_BUFFER_OK;
}

/**
 * @brief   Write a block of data into the ring buffer.
 *
 * @details This is an atomic-style operation: either all bytes are written,
 *          or none are. This prevents partial writes that could corrupt
 *          message framing.
 */
ring_buffer_status_t ring_buffer_write_block(ring_buffer_t *rb,
                                             const uint8_t *data,
                                             size_t length)
{
    if (rb == NULL || data == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    if (length == 0U)
    {
        return RING_BUFFER_OK;
    }

    /* Check if there is enough space for the entire block. */
    size_t available = rb->size - rb->count;
    if (length > available)
    {
        return RING_BUFFER_NO_SPACE;
    }

    /*
     * Copy data into the ring buffer, handling the wrap-around case.
     * Two scenarios:
     *   1. Data fits without wrapping: single memcpy.
     *   2. Data wraps around: two memcpy calls.
     */
    size_t bytes_to_end = rb->size - rb->head;

    if (length <= bytes_to_end)
    {
        /* Case 1: No wrap-around needed. */
        memcpy(&rb->buffer[rb->head], data, length);
    }
    else
    {
        /* Case 2: Data wraps around the end of the buffer. */
        memcpy(&rb->buffer[rb->head], data, bytes_to_end);
        memcpy(&rb->buffer[0], &data[bytes_to_end], length - bytes_to_end);
    }

    rb->head = (rb->head + length) % rb->size;
    rb->count += length;

    return RING_BUFFER_OK;
}

/**
 * @brief   Read a block of data from the ring buffer.
 *
 * @details This is an atomic-style operation: either all requested bytes are
 *          read, or none are consumed from the buffer.
 */
ring_buffer_status_t ring_buffer_read_block(ring_buffer_t *rb,
                                            uint8_t *data,
                                            size_t length)
{
    if (rb == NULL || data == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    if (length == 0U)
    {
        return RING_BUFFER_OK;
    }

    /* Check if there is enough data available. */
    if (length > rb->count)
    {
        return RING_BUFFER_NO_DATA;
    }

    /*
     * Copy data from the ring buffer, handling the wrap-around case.
     */
    size_t bytes_to_end = rb->size - rb->tail;

    if (length <= bytes_to_end)
    {
        /* Case 1: No wrap-around needed. */
        memcpy(data, &rb->buffer[rb->tail], length);
    }
    else
    {
        /* Case 2: Data wraps around the end of the buffer. */
        memcpy(data, &rb->buffer[rb->tail], bytes_to_end);
        memcpy(&data[bytes_to_end], &rb->buffer[0], length - bytes_to_end);
    }

    rb->tail = (rb->tail + length) % rb->size;
    rb->count -= length;

    return RING_BUFFER_OK;
}

/**
 * @brief   Peek at a byte in the ring buffer without consuming it.
 *
 * @details Allows inspection of data at any offset from the tail (oldest data).
 *          Offset 0 returns the oldest byte (next to be read).
 */
ring_buffer_status_t ring_buffer_peek(const ring_buffer_t *rb,
                                      uint8_t *byte,
                                      size_t offset)
{
    if (rb == NULL || byte == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    if (offset >= rb->count)
    {
        return RING_BUFFER_EMPTY;
    }

    size_t index = (rb->tail + offset) % rb->size;
    *byte = rb->buffer[index];

    return RING_BUFFER_OK;
}

/**
 * @brief   Check if the ring buffer is full.
 */
bool ring_buffer_is_full(const ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return true; /* Fail-safe: treat NULL as full to prevent writes. */
    }

    return (rb->count >= rb->size);
}

/**
 * @brief   Check if the ring buffer is empty.
 */
bool ring_buffer_is_empty(const ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return true; /* Fail-safe: treat NULL as empty to prevent reads. */
    }

    return (rb->count == 0U);
}

/**
 * @brief   Get the number of bytes currently stored in the ring buffer.
 */
size_t ring_buffer_available_data(const ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return 0U;
    }

    return rb->count;
}

/**
 * @brief   Get the number of free bytes available for writing.
 */
size_t ring_buffer_available_space(const ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return 0U;
    }

    return (rb->size - rb->count);
}

/**
 * @brief   Flush (reset) the ring buffer, discarding all data.
 */
ring_buffer_status_t ring_buffer_flush(ring_buffer_t *rb)
{
    if (rb == NULL)
    {
        return RING_BUFFER_NULL_PTR;
    }

    rb->head = 0U;
    rb->tail = 0U;
    rb->count = 0U;

    return RING_BUFFER_OK;
}
