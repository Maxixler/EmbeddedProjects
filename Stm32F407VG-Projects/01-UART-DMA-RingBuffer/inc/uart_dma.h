/**
 * @file    uart_dma.h
 * @brief   UART driver with DMA support and ring buffer integration for STM32F4.
 *
 * @details This module provides a high-performance UART communication driver that
 *          uses DMA for data transfer and ring buffers for data management.
 *
 *          Architecture:
 *          - RX: DMA circular mode captures incoming data into a DMA buffer.
 *                IDLE line detection and DMA half/complete transfer interrupts
 *                trigger data transfer from the DMA buffer into the RX ring buffer.
 *          - TX: Data is written to a TX ring buffer. When a DMA transfer is not
 *                active, data is copied from the ring buffer to a DMA buffer and
 *                a DMA normal-mode transfer is initiated.
 *
 *          Supported UART: USART2 (PA2=TX, PA3=RX) on STM32F407VG Discovery
 *          DMA mapping:
 *            - USART2_RX: DMA1 Stream5 Channel4 (Circular mode)
 *            - USART2_TX: DMA1 Stream6 Channel4 (Normal mode)
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef UART_DMA_H
#define UART_DMA_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* -------------------------------------------------------------------------- */
    /*                                  Includes                                  */
    /* -------------------------------------------------------------------------- */

#include "stm32f4xx_hal.h"
#include "ring_buffer.h"
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** @defgroup UART_DMA_Buffer_Sizes Buffer size configuration
 *  @brief   Defines the sizes of internal buffers. Must be powers of 2 for
 *           optimal modulo performance, though this implementation does not
 *           strictly require it.
 *  @{
 */
#define UART_DMA_RX_DMA_BUF_SIZE 256U   /**< DMA RX circular buffer size (bytes) */
#define UART_DMA_TX_DMA_BUF_SIZE 256U   /**< DMA TX transfer buffer size (bytes) */
#define UART_DMA_RX_RING_BUF_SIZE 1024U /**< RX ring buffer size (bytes)         */
#define UART_DMA_TX_RING_BUF_SIZE 1024U /**< TX ring buffer size (bytes)         */
/** @} */

/** @defgroup UART_DMA_Default_Config Default UART configuration
 *  @{
 */
#define UART_DMA_DEFAULT_BAUD_RATE 115200U /**< Default baud rate                   */
    /** @} */

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   UART DMA driver status codes.
     */
    typedef enum
    {
        UART_DMA_OK = 0,           /**< Operation successful                  */
        UART_DMA_ERROR = -1,       /**< General error                         */
        UART_DMA_BUSY = -2,        /**< DMA transfer in progress              */
        UART_DMA_TIMEOUT = -3,     /**< Operation timed out                   */
        UART_DMA_NULL_PTR = -4,    /**< Null pointer argument                 */
        UART_DMA_BUFFER_FULL = -5, /**< TX ring buffer is full                */
        UART_DMA_NO_DATA = -6,     /**< No data available in RX ring buffer   */
    } uart_dma_status_t;

    /**
     * @brief   UART error flags (bitmask).
     */
    typedef enum
    {
        UART_DMA_ERR_NONE = 0x00,    /**< No error                              */
        UART_DMA_ERR_PARITY = 0x01,  /**< Parity error detected                 */
        UART_DMA_ERR_FRAMING = 0x02, /**< Framing error detected                */
        UART_DMA_ERR_NOISE = 0x04,   /**< Noise error detected                  */
        UART_DMA_ERR_OVERRUN = 0x08, /**< Overrun error (data loss)             */
        UART_DMA_ERR_DMA = 0x10,     /**< DMA transfer error                    */
    } uart_dma_error_t;

    /**
     * @brief   Callback function type for RX data available notification.
     *
     * @param[in]   data    Pointer to the received data.
     * @param[in]   length  Number of bytes received.
     */
    typedef void (*uart_dma_rx_callback_t)(const uint8_t *data, size_t length);

    /**
     * @brief   Callback function type for TX complete notification.
     */
    typedef void (*uart_dma_tx_callback_t)(void);

    /**
     * @brief   Callback function type for error notification.
     *
     * @param[in]   errors  Bitmask of uart_dma_error_t flags.
     */
    typedef void (*uart_dma_error_callback_t)(uint32_t errors);

    /**
     * @brief   UART DMA configuration structure.
     */
    typedef struct
    {
        uint32_t baud_rate;    /**< Baud rate (e.g., 9600, 115200)            */
        uint32_t word_length;  /**< Word length: UART_WORDLENGTH_8B / _9B     */
        uint32_t stop_bits;    /**< Stop bits: UART_STOPBITS_1 / _2           */
        uint32_t parity;       /**< Parity: UART_PARITY_NONE / _EVEN / _ODD   */
        uint32_t flow_control; /**< Flow control: UART_HWCONTROL_NONE / etc.  */
    } uart_dma_config_t;

    /**
     * @brief   UART DMA communication statistics.
     */
    typedef struct
    {
        volatile uint32_t tx_bytes;         /**< Total bytes transmitted            */
        volatile uint32_t rx_bytes;         /**< Total bytes received               */
        volatile uint32_t tx_dma_transfers; /**< Number of DMA TX transfers         */
        volatile uint32_t rx_events;        /**< Number of RX events (IDLE/HT/TC)  */
        volatile uint32_t error_count;      /**< Total number of errors             */
        volatile uint32_t overrun_count;    /**< Overrun error count                */
        volatile uint32_t buffer_overflow;  /**< Ring buffer overflow count         */
    } uart_dma_stats_t;

    /**
     * @brief   UART DMA driver handle structure.
     *
     * @details Contains all state, HAL handles, DMA buffers, ring buffers,
     *          and callback pointers needed to manage the UART+DMA subsystem.
     */
    typedef struct
    {
        /* HAL peripheral handles */
        UART_HandleTypeDef huart;  /**< HAL UART handle                   */
        DMA_HandleTypeDef hdma_rx; /**< HAL DMA handle for RX stream      */
        DMA_HandleTypeDef hdma_tx; /**< HAL DMA handle for TX stream      */

        /* DMA buffers (directly accessed by DMA hardware) */
        uint8_t rx_dma_buf[UART_DMA_RX_DMA_BUF_SIZE]; /**< DMA RX circular buffer */
        uint8_t tx_dma_buf[UART_DMA_TX_DMA_BUF_SIZE]; /**< DMA TX transfer buffer */

        /* Ring buffer instances */
        ring_buffer_t rx_ring; /**< RX ring buffer instance           */
        ring_buffer_t tx_ring; /**< TX ring buffer instance           */

        /* Ring buffer backing storage */
        uint8_t rx_ring_data[UART_DMA_RX_RING_BUF_SIZE]; /**< RX ring buffer storage */
        uint8_t tx_ring_data[UART_DMA_TX_RING_BUF_SIZE]; /**< TX ring buffer storage */

        /* State tracking */
        volatile size_t rx_dma_old_pos; /**< Previous DMA RX read position     */
        volatile bool tx_dma_busy;      /**< TX DMA transfer in progress flag  */
        volatile uint32_t last_error;   /**< Last error flags (bitmask)        */
        volatile bool initialized;      /**< Driver initialization flag        */

        /* User callbacks */
        uart_dma_rx_callback_t rx_callback;       /**< RX data available callback    */
        uart_dma_tx_callback_t tx_callback;       /**< TX complete callback          */
        uart_dma_error_callback_t error_callback; /**< Error notification callback   */

        /* Statistics */
        uart_dma_stats_t stats; /**< Communication statistics          */
    } uart_dma_handle_t;

    /* -------------------------------------------------------------------------- */
    /*                            Function Prototypes                             */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize the UART DMA driver with the specified configuration.
     *
     * @details Configures USART2 peripheral, associated GPIO pins (PA2/PA3),
     *          DMA1 streams (Stream5 for RX, Stream6 for TX), and ring buffers.
     *          Starts DMA RX in circular mode with IDLE line detection enabled.
     *
     * @param[out]  handle  Pointer to the UART DMA handle to initialize.
     * @param[in]   config  Pointer to the configuration. If NULL, default
     *                      configuration (115200-8N1, no flow control) is used.
     *
     * @return  UART_DMA_OK on success, UART_DMA_ERROR on failure.
     */
    uart_dma_status_t uart_dma_init(uart_dma_handle_t *handle,
                                    const uart_dma_config_t *config);

    /**
     * @brief   Deinitialize the UART DMA driver and release resources.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     *
     * @return  UART_DMA_OK on success, UART_DMA_NULL_PTR if handle is NULL.
     */
    uart_dma_status_t uart_dma_deinit(uart_dma_handle_t *handle);

    /**
     * @brief   Send data through UART using the TX ring buffer and DMA.
     *
     * @details Data is first placed into the TX ring buffer. If no DMA transfer
     *          is currently active, a new DMA transfer is initiated immediately.
     *          Otherwise, remaining data will be sent when the current transfer
     *          completes (chained from the TC interrupt handler).
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     * @param[in]       data    Pointer to the data to send.
     * @param[in]       length  Number of bytes to send.
     *
     * @return  UART_DMA_OK on success, UART_DMA_BUFFER_FULL if insufficient space,
     *          UART_DMA_NULL_PTR if handle or data is NULL.
     */
    uart_dma_status_t uart_dma_send(uart_dma_handle_t *handle,
                                    const uint8_t *data,
                                    size_t length);

    /**
     * @brief   Send a null-terminated string through UART.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     * @param[in]       str     Pointer to the null-terminated string.
     *
     * @return  UART_DMA_OK on success, error code on failure.
     */
    uart_dma_status_t uart_dma_send_string(uart_dma_handle_t *handle,
                                           const char *str);

    /**
     * @brief   Initiate a DMA TX transfer from the TX ring buffer.
     *
     * @details Copies available data from the TX ring buffer to the DMA TX buffer
     *          and starts a DMA transfer. Called internally after writing to the
     *          ring buffer, and from the DMA TC ISR to chain transfers.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     *
     * @return  UART_DMA_OK if transfer started, UART_DMA_BUSY if a transfer is
     *          already in progress, UART_DMA_NO_DATA if the TX ring buffer is empty.
     */
    uart_dma_status_t uart_dma_start_tx_dma(uart_dma_handle_t *handle);

    /**
     * @brief   Read a single byte from the RX ring buffer.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     * @param[out]      byte    Pointer to store the received byte.
     *
     * @return  UART_DMA_OK on success, UART_DMA_NO_DATA if no data available.
     */
    uart_dma_status_t uart_dma_receive_byte(uart_dma_handle_t *handle,
                                            uint8_t *byte);

    /**
     * @brief   Read a block of bytes from the RX ring buffer.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     * @param[out]      data    Pointer to the destination buffer.
     * @param[in]       max_len Maximum number of bytes to read.
     * @param[out]      actual  Pointer to store the actual number of bytes read.
     *                          May be NULL if not needed.
     *
     * @return  UART_DMA_OK on success (actual contains the count),
     *          UART_DMA_NO_DATA if no data available.
     */
    uart_dma_status_t uart_dma_receive_block(uart_dma_handle_t *handle,
                                             uint8_t *data,
                                             size_t max_len,
                                             size_t *actual);

    /**
     * @brief   Register a callback for RX data available events.
     *
     * @param[in,out]   handle      Pointer to the UART DMA handle.
     * @param[in]       callback    Callback function pointer, or NULL to unregister.
     */
    void uart_dma_register_rx_callback(uart_dma_handle_t *handle,
                                       uart_dma_rx_callback_t callback);

    /**
     * @brief   Register a callback for TX complete events.
     *
     * @param[in,out]   handle      Pointer to the UART DMA handle.
     * @param[in]       callback    Callback function pointer, or NULL to unregister.
     */
    void uart_dma_register_tx_callback(uart_dma_handle_t *handle,
                                       uart_dma_tx_callback_t callback);

    /**
     * @brief   Register a callback for error events.
     *
     * @param[in,out]   handle      Pointer to the UART DMA handle.
     * @param[in]       callback    Callback function pointer, or NULL to unregister.
     */
    void uart_dma_register_error_callback(uart_dma_handle_t *handle,
                                          uart_dma_error_callback_t callback);

    /**
     * @brief   Get the number of bytes available for reading in the RX ring buffer.
     *
     * @param[in]   handle  Pointer to the UART DMA handle.
     *
     * @return  Number of bytes available for reading.
     */
    size_t uart_dma_get_rx_count(const uart_dma_handle_t *handle);

    /**
     * @brief   Get the current communication statistics.
     *
     * @param[in]   handle  Pointer to the UART DMA handle.
     * @param[out]  stats   Pointer to the statistics structure to fill.
     *
     * @return  UART_DMA_OK on success, UART_DMA_NULL_PTR if a pointer is NULL.
     */
    uart_dma_status_t uart_dma_get_stats(const uart_dma_handle_t *handle,
                                         uart_dma_stats_t *stats);

    /**
     * @brief   Reset the communication statistics counters.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     */
    void uart_dma_reset_stats(uart_dma_handle_t *handle);

    /**
     * @brief   Get the last error flags and clear them.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     *
     * @return  Bitmask of uart_dma_error_t flags representing accumulated errors.
     */
    uint32_t uart_dma_get_and_clear_errors(uart_dma_handle_t *handle);

    /**
     * @brief   Process the DMA RX data (called from ISR context).
     *
     * @details Reads the current DMA NDTR to determine how much new data has arrived,
     *          copies it into the RX ring buffer, and optionally invokes the RX callback.
     *          This function handles DMA buffer wrap-around correctly.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     *
     * @note    This function is intended to be called from:
     *          - UART IDLE line interrupt handler
     *          - DMA RX half-transfer complete interrupt handler
     *          - DMA RX transfer complete interrupt handler
     */
    void uart_dma_rx_check(uart_dma_handle_t *handle);

    /**
     * @brief   Handle DMA TX transfer complete (called from ISR context).
     *
     * @details Checks if more data is in the TX ring buffer and starts a new
     *          DMA transfer if needed (chaining). Otherwise, clears the busy flag.
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     */
    void uart_dma_tx_complete_handler(uart_dma_handle_t *handle);

    /**
     * @brief   Handle UART error interrupts (called from ISR context).
     *
     * @param[in,out]   handle  Pointer to the UART DMA handle.
     */
    void uart_dma_error_handler(uart_dma_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H */
