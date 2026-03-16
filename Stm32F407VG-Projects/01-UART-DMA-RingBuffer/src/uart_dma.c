/**
 * @file    uart_dma.c
 * @brief   UART DMA driver implementation for STM32F407VG.
 *
 * @details This file implements the UART+DMA driver using the STM32 HAL library.
 *          It configures USART2 with DMA1 for both reception and transmission.
 *
 *          RX path:
 *          1. DMA1 Stream5 Channel4 runs in circular mode, continuously filling
 *             rx_dma_buf[] from USART2->DR.
 *          2. On IDLE line detection, DMA half-transfer, or DMA transfer complete
 *             interrupts, uart_dma_rx_check() is called.
 *          3. New data is identified by comparing the current DMA NDTR with the
 *             previously recorded position, then copied into the RX ring buffer.
 *
 *          TX path:
 *          1. Application writes data into the TX ring buffer via uart_dma_send().
 *          2. Data is copied from the ring buffer into tx_dma_buf[] and a DMA
 *             transfer is started in normal mode.
 *          3. On DMA TX transfer complete, the handler checks for remaining data
 *             and chains another transfer if needed.
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "uart_dma.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Global pointer to the active UART DMA handle.
 *
 * @details Used by HAL callbacks to access the driver instance. This limits
 *          the driver to a single UART instance (USART2). For multi-instance
 *          support, a lookup table indexed by UART peripheral address would
 *          be needed.
 */
static uart_dma_handle_t *s_active_handle = NULL;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void uart_dma_gpio_init(void);
static void uart_dma_gpio_deinit(void);
static HAL_StatusTypeDef uart_dma_configure_dma_rx(uart_dma_handle_t *handle);
static HAL_StatusTypeDef uart_dma_configure_dma_tx(uart_dma_handle_t *handle);
static void uart_dma_enable_idle_interrupt(uart_dma_handle_t *handle);

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialize the UART DMA driver.
 */
uart_dma_status_t uart_dma_init(uart_dma_handle_t *handle,
                                const uart_dma_config_t *config)
{
    if (handle == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    /* Clear the entire handle structure to known state. */
    memset(handle, 0, sizeof(uart_dma_handle_t));

    /* Store the global handle pointer for HAL callbacks. */
    s_active_handle = handle;

    /* ---- Initialize ring buffers ---- */
    ring_buffer_init(&handle->rx_ring, handle->rx_ring_data, UART_DMA_RX_RING_BUF_SIZE);
    ring_buffer_init(&handle->tx_ring, handle->tx_ring_data, UART_DMA_TX_RING_BUF_SIZE);

    /* ---- Enable peripheral clocks ---- */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* ---- Configure GPIO pins for USART2 (PA2=TX, PA3=RX) ---- */
    uart_dma_gpio_init();

    /* ---- Configure USART2 ---- */
    handle->huart.Instance = USART2;

    if (config != NULL)
    {
        handle->huart.Init.BaudRate = config->baud_rate;
        handle->huart.Init.WordLength = config->word_length;
        handle->huart.Init.StopBits = config->stop_bits;
        handle->huart.Init.Parity = config->parity;
        handle->huart.Init.HwFlowCtl = config->flow_control;
    }
    else
    {
        /* Default configuration: 115200-8N1, no flow control. */
        handle->huart.Init.BaudRate = UART_DMA_DEFAULT_BAUD_RATE;
        handle->huart.Init.WordLength = UART_WORDLENGTH_8B;
        handle->huart.Init.StopBits = UART_STOPBITS_1;
        handle->huart.Init.Parity = UART_PARITY_NONE;
        handle->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }

    handle->huart.Init.Mode = UART_MODE_TX_RX;
    handle->huart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&handle->huart) != HAL_OK)
    {
        return UART_DMA_ERROR;
    }

    /* ---- Configure DMA streams ---- */
    if (uart_dma_configure_dma_rx(handle) != HAL_OK)
    {
        return UART_DMA_ERROR;
    }

    if (uart_dma_configure_dma_tx(handle) != HAL_OK)
    {
        return UART_DMA_ERROR;
    }

    /* ---- Link DMA handles to UART handle ---- */
    __HAL_LINKDMA(&handle->huart, hdmarx, handle->hdma_rx);
    __HAL_LINKDMA(&handle->huart, hdmatx, handle->hdma_tx);

    /* ---- Configure NVIC for DMA and UART interrupts ---- */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0); /* RX DMA - high priority */
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 2, 0); /* TX DMA - medium priority */
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    HAL_NVIC_SetPriority(USART2_IRQn, 1, 1); /* UART - high priority */
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* ---- Start DMA RX in circular mode ---- */
    handle->rx_dma_old_pos = 0U;
    handle->tx_dma_busy = false;

    if (HAL_UART_Receive_DMA(&handle->huart, handle->rx_dma_buf,
                             UART_DMA_RX_DMA_BUF_SIZE) != HAL_OK)
    {
        return UART_DMA_ERROR;
    }

    /* ---- Enable IDLE line interrupt for variable-length packet detection ---- */
    uart_dma_enable_idle_interrupt(handle);

    handle->initialized = true;

    return UART_DMA_OK;
}

/**
 * @brief   Deinitialize the UART DMA driver.
 */
uart_dma_status_t uart_dma_deinit(uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    /* Abort any ongoing DMA transfers. */
    HAL_UART_DMAStop(&handle->huart);

    /* Deinitialize peripherals. */
    HAL_DMA_DeInit(&handle->hdma_rx);
    HAL_DMA_DeInit(&handle->hdma_tx);
    HAL_UART_DeInit(&handle->huart);

    /* Disable interrupts. */
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    /* Release GPIO pins. */
    uart_dma_gpio_deinit();

    /* Flush ring buffers. */
    ring_buffer_flush(&handle->rx_ring);
    ring_buffer_flush(&handle->tx_ring);

    handle->initialized = false;
    s_active_handle = NULL;

    return UART_DMA_OK;
}

/**
 * @brief   Send data through UART using the TX ring buffer and DMA.
 */
uart_dma_status_t uart_dma_send(uart_dma_handle_t *handle,
                                const uint8_t *data,
                                size_t length)
{
    if (handle == NULL || data == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    if (length == 0U)
    {
        return UART_DMA_OK;
    }

    /* Write data to the TX ring buffer. */
    ring_buffer_status_t rb_status = ring_buffer_write_block(&handle->tx_ring,
                                                             data, length);
    if (rb_status != RING_BUFFER_OK)
    {
        handle->stats.buffer_overflow++;
        return UART_DMA_BUFFER_FULL;
    }

    /* If DMA TX is not currently busy, start a new transfer. */
    if (!handle->tx_dma_busy)
    {
        uart_dma_start_tx_dma(handle);
    }

    return UART_DMA_OK;
}

/**
 * @brief   Send a null-terminated string.
 */
uart_dma_status_t uart_dma_send_string(uart_dma_handle_t *handle,
                                       const char *str)
{
    if (handle == NULL || str == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    size_t len = strlen(str);
    if (len == 0U)
    {
        return UART_DMA_OK;
    }

    return uart_dma_send(handle, (const uint8_t *)str, len);
}

/**
 * @brief   Start a DMA TX transfer from the TX ring buffer.
 */
uart_dma_status_t uart_dma_start_tx_dma(uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    /* If a DMA transfer is already in progress, do nothing. */
    if (handle->tx_dma_busy)
    {
        return UART_DMA_BUSY;
    }

    /* Check if there is data to send. */
    size_t available = ring_buffer_available_data(&handle->tx_ring);
    if (available == 0U)
    {
        return UART_DMA_NO_DATA;
    }

    /*
     * Determine how many bytes to transfer in this batch.
     * Limited by the DMA TX buffer size and available data.
     */
    size_t tx_len = (available > UART_DMA_TX_DMA_BUF_SIZE)
                        ? UART_DMA_TX_DMA_BUF_SIZE
                        : available;

    /* Copy data from the TX ring buffer to the DMA buffer. */
    ring_buffer_status_t rb_status = ring_buffer_read_block(&handle->tx_ring,
                                                            handle->tx_dma_buf,
                                                            tx_len);
    if (rb_status != RING_BUFFER_OK)
    {
        return UART_DMA_ERROR;
    }

    /* Mark TX DMA as busy BEFORE starting the transfer. */
    handle->tx_dma_busy = true;

    /* Start the DMA transfer. */
    if (HAL_UART_Transmit_DMA(&handle->huart, handle->tx_dma_buf,
                              (uint16_t)tx_len) != HAL_OK)
    {
        handle->tx_dma_busy = false;
        return UART_DMA_ERROR;
    }

    /* Update statistics. */
    handle->stats.tx_bytes += (uint32_t)tx_len;
    handle->stats.tx_dma_transfers++;

    return UART_DMA_OK;
}

/**
 * @brief   Read a single byte from the RX ring buffer.
 */
uart_dma_status_t uart_dma_receive_byte(uart_dma_handle_t *handle,
                                        uint8_t *byte)
{
    if (handle == NULL || byte == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    ring_buffer_status_t rb_status = ring_buffer_read(&handle->rx_ring, byte);
    if (rb_status != RING_BUFFER_OK)
    {
        return UART_DMA_NO_DATA;
    }

    return UART_DMA_OK;
}

/**
 * @brief   Read a block of bytes from the RX ring buffer.
 *
 * @details Reads up to max_len bytes. Unlike ring_buffer_read_block, this
 *          function performs a partial read, returning whatever is available
 *          up to max_len.
 */
uart_dma_status_t uart_dma_receive_block(uart_dma_handle_t *handle,
                                         uint8_t *data,
                                         size_t max_len,
                                         size_t *actual)
{
    if (handle == NULL || data == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    size_t available = ring_buffer_available_data(&handle->rx_ring);
    if (available == 0U)
    {
        if (actual != NULL)
        {
            *actual = 0U;
        }
        return UART_DMA_NO_DATA;
    }

    /* Read up to max_len bytes (partial read). */
    size_t to_read = (available > max_len) ? max_len : available;

    ring_buffer_status_t rb_status = ring_buffer_read_block(&handle->rx_ring,
                                                            data, to_read);
    if (rb_status != RING_BUFFER_OK)
    {
        if (actual != NULL)
        {
            *actual = 0U;
        }
        return UART_DMA_ERROR;
    }

    if (actual != NULL)
    {
        *actual = to_read;
    }

    return UART_DMA_OK;
}

/**
 * @brief   Register RX data callback.
 */
void uart_dma_register_rx_callback(uart_dma_handle_t *handle,
                                   uart_dma_rx_callback_t callback)
{
    if (handle != NULL)
    {
        handle->rx_callback = callback;
    }
}

/**
 * @brief   Register TX complete callback.
 */
void uart_dma_register_tx_callback(uart_dma_handle_t *handle,
                                   uart_dma_tx_callback_t callback)
{
    if (handle != NULL)
    {
        handle->tx_callback = callback;
    }
}

/**
 * @brief   Register error callback.
 */
void uart_dma_register_error_callback(uart_dma_handle_t *handle,
                                      uart_dma_error_callback_t callback)
{
    if (handle != NULL)
    {
        handle->error_callback = callback;
    }
}

/**
 * @brief   Get available RX data count.
 */
size_t uart_dma_get_rx_count(const uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return 0U;
    }

    return ring_buffer_available_data(&handle->rx_ring);
}

/**
 * @brief   Get communication statistics.
 */
uart_dma_status_t uart_dma_get_stats(const uart_dma_handle_t *handle,
                                     uart_dma_stats_t *stats)
{
    if (handle == NULL || stats == NULL)
    {
        return UART_DMA_NULL_PTR;
    }

    memcpy(stats, &handle->stats, sizeof(uart_dma_stats_t));
    return UART_DMA_OK;
}

/**
 * @brief   Reset statistics counters.
 */
void uart_dma_reset_stats(uart_dma_handle_t *handle)
{
    if (handle != NULL)
    {
        memset(&handle->stats, 0, sizeof(uart_dma_stats_t));
    }
}

/**
 * @brief   Get and clear accumulated error flags.
 */
uint32_t uart_dma_get_and_clear_errors(uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return 0U;
    }

    uint32_t errors = handle->last_error;
    handle->last_error = UART_DMA_ERR_NONE;
    return errors;
}

/**
 * @brief   Process RX data from the DMA circular buffer.
 *
 * @details This is the core RX processing function. It compares the current
 *          DMA write position (derived from NDTR) with the previously recorded
 *          position to determine how much new data has arrived.
 *
 *          Called from ISR context on:
 *          - UART IDLE line interrupt
 *          - DMA half-transfer complete interrupt
 *          - DMA transfer complete interrupt
 */
void uart_dma_rx_check(uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    /*
     * Calculate the current write position in the DMA buffer.
     * NDTR counts DOWN from the buffer size, so:
     *   current_pos = buffer_size - NDTR
     *
     * For example, with a 256-byte buffer:
     *   NDTR=256 -> pos=0   (DMA hasn't written anything yet, or just wrapped)
     *   NDTR=200 -> pos=56  (DMA has written 56 bytes)
     *   NDTR=1   -> pos=255 (DMA is about to wrap)
     */
    size_t dma_buf_size = UART_DMA_RX_DMA_BUF_SIZE;
    size_t current_pos = dma_buf_size -
                         (size_t)__HAL_DMA_GET_COUNTER(&handle->hdma_rx);

    if (current_pos != handle->rx_dma_old_pos)
    {
        size_t old_pos = handle->rx_dma_old_pos;

        if (current_pos > old_pos)
        {
            /*
             * Case 1: No wrap-around.
             * New data is contiguous from old_pos to current_pos.
             *
             *  [....old_pos|NEW DATA|current_pos....]
             */
            size_t new_data_len = current_pos - old_pos;
            ring_buffer_status_t status = ring_buffer_write_block(
                &handle->rx_ring,
                &handle->rx_dma_buf[old_pos],
                new_data_len);

            if (status == RING_BUFFER_OK)
            {
                handle->stats.rx_bytes += (uint32_t)new_data_len;
            }
            else
            {
                handle->stats.buffer_overflow++;
            }

            /* Invoke RX callback if registered. */
            if (handle->rx_callback != NULL)
            {
                handle->rx_callback(&handle->rx_dma_buf[old_pos], new_data_len);
            }
        }
        else
        {
            /*
             * Case 2: Wrap-around occurred.
             * Data is split into two segments:
             *   Segment A: old_pos to end of buffer
             *   Segment B: start of buffer to current_pos
             *
             *  [NEW B|current_pos....old_pos|NEW A]
             */

            /* Segment A: from old_pos to end of DMA buffer. */
            size_t seg_a_len = dma_buf_size - old_pos;
            ring_buffer_status_t status_a = ring_buffer_write_block(
                &handle->rx_ring,
                &handle->rx_dma_buf[old_pos],
                seg_a_len);

            if (status_a == RING_BUFFER_OK)
            {
                handle->stats.rx_bytes += (uint32_t)seg_a_len;
            }
            else
            {
                handle->stats.buffer_overflow++;
            }

            /* Segment B: from start of DMA buffer to current_pos. */
            if (current_pos > 0U)
            {
                ring_buffer_status_t status_b = ring_buffer_write_block(
                    &handle->rx_ring,
                    &handle->rx_dma_buf[0],
                    current_pos);

                if (status_b == RING_BUFFER_OK)
                {
                    handle->stats.rx_bytes += (uint32_t)current_pos;
                }
                else
                {
                    handle->stats.buffer_overflow++;
                }
            }

            /* Invoke RX callback with the first segment pointer. */
            if (handle->rx_callback != NULL)
            {
                handle->rx_callback(&handle->rx_dma_buf[old_pos],
                                    seg_a_len + current_pos);
            }
        }

        /* Update the tracked position for next call. */
        handle->rx_dma_old_pos = current_pos;
        handle->stats.rx_events++;
    }
}

/**
 * @brief   Handle DMA TX transfer complete.
 */
void uart_dma_tx_complete_handler(uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    /* Mark DMA TX as no longer busy. */
    handle->tx_dma_busy = false;

    /* Invoke TX complete callback if registered. */
    if (handle->tx_callback != NULL)
    {
        handle->tx_callback();
    }

    /*
     * Chain the next transfer if there is more data in the TX ring buffer.
     * This ensures continuous DMA TX without gaps.
     */
    if (!ring_buffer_is_empty(&handle->tx_ring))
    {
        uart_dma_start_tx_dma(handle);
    }
}

/**
 * @brief   Handle UART errors.
 */
void uart_dma_error_handler(uart_dma_handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    uint32_t hal_error = HAL_UART_GetError(&handle->huart);
    uint32_t our_error = UART_DMA_ERR_NONE;

    if (hal_error & HAL_UART_ERROR_PE)
    {
        our_error |= UART_DMA_ERR_PARITY;
    }
    if (hal_error & HAL_UART_ERROR_FE)
    {
        our_error |= UART_DMA_ERR_FRAMING;
    }
    if (hal_error & HAL_UART_ERROR_NE)
    {
        our_error |= UART_DMA_ERR_NOISE;
    }
    if (hal_error & HAL_UART_ERROR_ORE)
    {
        our_error |= UART_DMA_ERR_OVERRUN;
        handle->stats.overrun_count++;
    }
    if (hal_error & HAL_UART_ERROR_DMA)
    {
        our_error |= UART_DMA_ERR_DMA;
    }

    handle->last_error |= our_error;
    handle->stats.error_count++;

    /* Invoke error callback if registered. */
    if (handle->error_callback != NULL)
    {
        handle->error_callback(our_error);
    }

    /*
     * After an error, the DMA RX may have been stopped by HAL.
     * Restart it to continue receiving data.
     */
    if (hal_error & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE |
                     HAL_UART_ERROR_NE | HAL_UART_ERROR_PE))
    {
        /* Clear error flags by reading SR then DR (per reference manual). */
        volatile uint32_t tmp;
        tmp = handle->huart.Instance->SR;
        tmp = handle->huart.Instance->DR;
        (void)tmp;

        /*
         * If the DMA RX is no longer active (HAL may have aborted it),
         * restart it.
         */
        if (handle->huart.RxState == HAL_UART_STATE_READY ||
            handle->huart.RxState == HAL_UART_STATE_ERROR)
        {
            handle->rx_dma_old_pos = 0U;
            HAL_UART_Receive_DMA(&handle->huart, handle->rx_dma_buf,
                                 UART_DMA_RX_DMA_BUF_SIZE);
            uart_dma_enable_idle_interrupt(handle);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialize GPIO pins for USART2 (PA2=TX, PA3=RX).
 */
static void uart_dma_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PA2 = USART2_TX, PA3 = USART2_RX */
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief   Deinitialize USART2 GPIO pins.
 */
static void uart_dma_gpio_deinit(void)
{
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
}

/**
 * @brief   Configure DMA1 Stream5 Channel4 for USART2 RX (circular mode).
 */
static HAL_StatusTypeDef uart_dma_configure_dma_rx(uart_dma_handle_t *handle)
{
    handle->hdma_rx.Instance = DMA1_Stream5;
    handle->hdma_rx.Init.Channel = DMA_CHANNEL_4;
    handle->hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle->hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    handle->hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    handle->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    handle->hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    handle->hdma_rx.Init.Mode = DMA_CIRCULAR;
    handle->hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;
    handle->hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    return HAL_DMA_Init(&handle->hdma_rx);
}

/**
 * @brief   Configure DMA1 Stream6 Channel4 for USART2 TX (normal mode).
 */
static HAL_StatusTypeDef uart_dma_configure_dma_tx(uart_dma_handle_t *handle)
{
    handle->hdma_tx.Instance = DMA1_Stream6;
    handle->hdma_tx.Init.Channel = DMA_CHANNEL_4;
    handle->hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle->hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    handle->hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    handle->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    handle->hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    handle->hdma_tx.Init.Mode = DMA_NORMAL;
    handle->hdma_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    handle->hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    return HAL_DMA_Init(&handle->hdma_tx);
}

/**
 * @brief   Enable the UART IDLE line interrupt.
 *
 * @details The IDLE interrupt fires when the RX line has been idle for one
 *          frame duration after receiving data. This is used to detect the
 *          end of variable-length packets.
 *
 *          Sequence to enable:
 *          1. Read SR to check/clear any pending IDLE flag.
 *          2. Read DR to complete the clear sequence.
 *          3. Set IDLEIE bit in CR1 to enable the interrupt.
 */
static void uart_dma_enable_idle_interrupt(uart_dma_handle_t *handle)
{
    /* Clear any pending IDLE flag (read SR then DR per reference manual). */
    volatile uint32_t tmp;
    tmp = handle->huart.Instance->SR;
    tmp = handle->huart.Instance->DR;
    (void)tmp;

    /* Enable IDLE line interrupt. */
    __HAL_UART_ENABLE_IT(&handle->huart, UART_IT_IDLE);
}

/* -------------------------------------------------------------------------- */
/*                        HAL Callback Implementations                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   HAL UART RX complete callback (DMA circular mode full transfer).
 *
 * @details Called by HAL when the DMA has filled the entire RX DMA buffer
 *          and wrapped around to the beginning (in circular mode).
 *
 * @param[in]   huart   Pointer to the UART handle that triggered the callback.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 && s_active_handle != NULL)
    {
        uart_dma_rx_check(s_active_handle);
    }
}

/**
 * @brief   HAL UART RX half-complete callback (DMA circular mode half transfer).
 *
 * @details Called by HAL when the DMA has filled the first half of the RX DMA
 *          buffer. Processing at half-transfer prevents data loss that could
 *          occur if we only processed at full transfer.
 *
 * @param[in]   huart   Pointer to the UART handle that triggered the callback.
 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 && s_active_handle != NULL)
    {
        uart_dma_rx_check(s_active_handle);
    }
}

/**
 * @brief   HAL UART TX complete callback.
 *
 * @details Called by HAL when a DMA TX transfer has completed. This is where
 *          we chain the next transfer if more data is available.
 *
 * @param[in]   huart   Pointer to the UART handle that triggered the callback.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 && s_active_handle != NULL)
    {
        uart_dma_tx_complete_handler(s_active_handle);
    }
}

/**
 * @brief   HAL UART error callback.
 *
 * @param[in]   huart   Pointer to the UART handle that triggered the error.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2 && s_active_handle != NULL)
    {
        uart_dma_error_handler(s_active_handle);
    }
}

/* -------------------------------------------------------------------------- */
/*                          IRQ Handler Implementations                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief   USART2 global interrupt handler.
 *
 * @details Handles the IDLE line interrupt, which is not managed by the HAL
 *          UART IRQ handler. After checking for IDLE, delegates to HAL for
 *          standard UART interrupt processing.
 */
void USART2_IRQHandler(void)
{
    if (s_active_handle != NULL)
    {
        UART_HandleTypeDef *huart = &s_active_handle->huart;

        /*
         * Check for IDLE line interrupt (not handled by HAL_UART_IRQHandler).
         * IDLE flag is set when the RX line is idle for one frame duration
         * after receiving data.
         */
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
        {
            /* Clear the IDLE flag: read SR then read DR (per reference manual). */
            volatile uint32_t tmp;
            tmp = huart->Instance->SR;
            tmp = huart->Instance->DR;
            (void)tmp;

            /* Process any new data that arrived before the line went idle. */
            uart_dma_rx_check(s_active_handle);
        }

        /* Let HAL handle other UART interrupts (TC, RXNE, errors, etc.). */
        HAL_UART_IRQHandler(huart);
    }
}

/**
 * @brief   DMA1 Stream5 global interrupt handler (USART2 RX).
 */
void DMA1_Stream5_IRQHandler(void)
{
    if (s_active_handle != NULL)
    {
        HAL_DMA_IRQHandler(&s_active_handle->hdma_rx);
    }
}

/**
 * @brief   DMA1 Stream6 global interrupt handler (USART2 TX).
 */
void DMA1_Stream6_IRQHandler(void)
{
    if (s_active_handle != NULL)
    {
        HAL_DMA_IRQHandler(&s_active_handle->hdma_tx);
    }
}
