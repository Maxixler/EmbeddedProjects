/**
 * @file    ring_buffer.h
 * @brief   Thread-safe ring buffer (circular buffer) data structure for embedded systems.
 *
 * @details This module provides a generic, thread-safe ring buffer implementation
 *          suitable for use in interrupt-driven and DMA-based communication systems.
 *          The ring buffer supports both single-byte and block read/write operations.
 *
 *          Key features:
 *          - O(1) read and write operations
 *          - No dynamic memory allocation (buffer must be provided externally)
 *          - Thread-safe through volatile qualifiers on shared state
 *          - Supports peek operation without consuming data
 *          - Block read/write for efficient bulk data transfers
 *
 * @note    This implementation assumes a single-producer, single-consumer model.
 *          For multi-producer or multi-consumer scenarios, additional synchronization
 *          (e.g., critical sections) is required.
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* -------------------------------------------------------------------------- */
    /*                                  Includes                                  */
    /* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Ring buffer status codes.
     */
    typedef enum
    {
        RING_BUFFER_OK = 0,        /**< Operation completed successfully      */
        RING_BUFFER_ERROR = -1,    /**< General error                         */
        RING_BUFFER_FULL = -2,     /**< Buffer is full, cannot write          */
        RING_BUFFER_EMPTY = -3,    /**< Buffer is empty, cannot read          */
        RING_BUFFER_NULL_PTR = -4, /**< Null pointer argument                 */
        RING_BUFFER_NO_SPACE = -5, /**< Not enough space for the operation    */
        RING_BUFFER_NO_DATA = -6,  /**< Not enough data available             */
    } ring_buffer_status_t;

    /**
     * @brief   Ring buffer instance structure.
     *
     * @details Contains all state needed to manage a circular buffer.
     *          The head index points to the next write position.
     *          The tail index points to the next read position.
     *          The count field tracks the number of bytes currently stored.
     *
     * @note    The 'volatile' qualifier on head, tail, and count ensures correct
     *          behavior when accessed from both interrupt context and main loop.
     */
    typedef struct
    {
        uint8_t *buffer;       /**< Pointer to the underlying data buffer     */
        volatile size_t head;  /**< Write index (next position to write)      */
        volatile size_t tail;  /**< Read index (next position to read)        */
        size_t size;           /**< Total capacity of the buffer in bytes     */
        volatile size_t count; /**< Number of bytes currently stored          */
    } ring_buffer_t;

    /* -------------------------------------------------------------------------- */
    /*                            Function Prototypes                             */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize a ring buffer instance.
     *
     * @param[out]  rb      Pointer to the ring buffer instance to initialize.
     * @param[in]   buffer  Pointer to the memory to use as the backing store.
     * @param[in]   size    Size of the backing buffer in bytes. Must be > 0.
     *
     * @return  RING_BUFFER_OK on success, RING_BUFFER_NULL_PTR if rb or buffer is NULL,
     *          RING_BUFFER_ERROR if size is 0.
     */
    ring_buffer_status_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, size_t size);

    /**
     * @brief   Write a single byte into the ring buffer.
     *
     * @param[in,out]   rb      Pointer to the ring buffer instance.
     * @param[in]       byte    The byte to write.
     *
     * @return  RING_BUFFER_OK on success, RING_BUFFER_FULL if the buffer is full,
     *          RING_BUFFER_NULL_PTR if rb is NULL.
     */
    ring_buffer_status_t ring_buffer_write(ring_buffer_t *rb, uint8_t byte);

    /**
     * @brief   Read a single byte from the ring buffer.
     *
     * @param[in,out]   rb      Pointer to the ring buffer instance.
     * @param[out]      byte    Pointer to store the read byte.
     *
     * @return  RING_BUFFER_OK on success, RING_BUFFER_EMPTY if no data available,
     *          RING_BUFFER_NULL_PTR if rb or byte is NULL.
     */
    ring_buffer_status_t ring_buffer_read(ring_buffer_t *rb, uint8_t *byte);

    /**
     * @brief   Write a block of data into the ring buffer.
     *
     * @param[in,out]   rb      Pointer to the ring buffer instance.
     * @param[in]       data    Pointer to the source data to write.
     * @param[in]       length  Number of bytes to write.
     *
     * @return  RING_BUFFER_OK if all bytes were written,
     *          RING_BUFFER_NO_SPACE if insufficient space (no data is written),
     *          RING_BUFFER_NULL_PTR if rb or data is NULL.
     */
    ring_buffer_status_t ring_buffer_write_block(ring_buffer_t *rb,
                                                 const uint8_t *data,
                                                 size_t length);

    /**
     * @brief   Read a block of data from the ring buffer.
     *
     * @param[in,out]   rb      Pointer to the ring buffer instance.
     * @param[out]      data    Pointer to the destination buffer.
     * @param[in]       length  Number of bytes to read.
     *
     * @return  RING_BUFFER_OK if all bytes were read,
     *          RING_BUFFER_NO_DATA if insufficient data available (no data is consumed),
     *          RING_BUFFER_NULL_PTR if rb or data is NULL.
     */
    ring_buffer_status_t ring_buffer_read_block(ring_buffer_t *rb,
                                                uint8_t *data,
                                                size_t length);

    /**
     * @brief   Peek at a byte in the ring buffer without consuming it.
     *
     * @param[in]   rb      Pointer to the ring buffer instance.
     * @param[out]  byte    Pointer to store the peeked byte.
     * @param[in]   offset  Offset from the tail (0 = oldest byte in buffer).
     *
     * @return  RING_BUFFER_OK on success, RING_BUFFER_EMPTY if no data at offset,
     *          RING_BUFFER_NULL_PTR if rb or byte is NULL.
     */
    ring_buffer_status_t ring_buffer_peek(const ring_buffer_t *rb,
                                          uint8_t *byte,
                                          size_t offset);

    /**
     * @brief   Check if the ring buffer is full.
     *
     * @param[in]   rb  Pointer to the ring buffer instance.
     *
     * @return  true if the buffer is full, false otherwise.
     *          Returns true if rb is NULL (fail-safe).
     */
    bool ring_buffer_is_full(const ring_buffer_t *rb);

    /**
     * @brief   Check if the ring buffer is empty.
     *
     * @param[in]   rb  Pointer to the ring buffer instance.
     *
     * @return  true if the buffer is empty, false otherwise.
     *          Returns true if rb is NULL (fail-safe).
     */
    bool ring_buffer_is_empty(const ring_buffer_t *rb);

    /**
     * @brief   Get the number of bytes currently stored in the ring buffer.
     *
     * @param[in]   rb  Pointer to the ring buffer instance.
     *
     * @return  Number of bytes available for reading. Returns 0 if rb is NULL.
     */
    size_t ring_buffer_available_data(const ring_buffer_t *rb);

    /**
     * @brief   Get the number of free bytes available for writing.
     *
     * @param[in]   rb  Pointer to the ring buffer instance.
     *
     * @return  Number of bytes that can be written. Returns 0 if rb is NULL.
     */
    size_t ring_buffer_available_space(const ring_buffer_t *rb);

    /**
     * @brief   Flush (reset) the ring buffer, discarding all data.
     *
     * @param[in,out]   rb  Pointer to the ring buffer instance.
     *
     * @return  RING_BUFFER_OK on success, RING_BUFFER_NULL_PTR if rb is NULL.
     *
     * @note    This does NOT zero-out the underlying memory, it only resets the indices.
     */
    ring_buffer_status_t ring_buffer_flush(ring_buffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
