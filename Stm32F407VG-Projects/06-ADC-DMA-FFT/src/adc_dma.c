/**
 * @file    adc_dma.c
 * @brief   ADC with DMA driver implementation for STM32F407VG
 * @details Implements ADC1 Channel 0 (PA0) with Timer2 TRGO triggered conversions
 *          and DMA2 Stream0 circular mode transfers. Provides double buffering
 *          through half-transfer and transfer-complete DMA interrupts.
 *
 *          Hardware configuration:
 *          - ADC1 Channel 0: PA0 (analog input)
 *          - DMA2 Stream 0, Channel 0: Peripheral-to-memory circular transfer
 *          - TIM2: TRGO trigger source with configurable period for sampling rate
 *          - ADC clock: APB2 (84 MHz) / 4 = 21 MHz
 *          - Timer clock: APB1 * 2 = 84 MHz
 *
 * @author  Embedded Systems Project
 * @version 1.0
 */

/* ========================== Includes ========================== */
#include "adc_dma.h"
#include <string.h>

/* ========================== Private Variables ========================== */

/**
 * @brief Global handle pointer for ISR access
 * @details Since HAL DMA callbacks don't carry user context, we store
 *          the handle pointer globally. This limits us to one ADC-DMA
 *          instance, which is acceptable for this application.
 */
static ADC_DMA_Handle_t *g_adc_dma_handle = NULL;

/* ========================== Private Function Prototypes ========================== */

static HAL_StatusTypeDef ADC_DMA_ConfigGPIO(void);
static HAL_StatusTypeDef ADC_DMA_ConfigDMA(ADC_DMA_Handle_t *handle);
static HAL_StatusTypeDef ADC_DMA_ConfigADC(ADC_DMA_Handle_t *handle,
                                           const ADC_DMA_Config_t *config);
static HAL_StatusTypeDef ADC_DMA_ConfigTimer(ADC_DMA_Handle_t *handle,
                                             uint32_t sampling_rate);
static uint32_t ADC_DMA_CalculateTimerPeriod(uint32_t sampling_rate);

/* ========================== Public Functions ========================== */

/**
 * @brief  Initialize the ADC-DMA system
 *
 * Initialization sequence:
 * 1. Enable peripheral clocks (GPIOA, ADC1, DMA2, TIM2)
 * 2. Configure PA0 as analog input
 * 3. Configure DMA2 Stream0 for circular peripheral-to-memory transfer
 * 4. Configure ADC1 with external trigger from TIM2 TRGO
 * 5. Configure TIM2 with calculated period for desired sampling rate
 * 6. Set up double buffer pointers
 */
HAL_StatusTypeDef ADC_DMA_Init(ADC_DMA_Handle_t *handle,
                               const ADC_DMA_Config_t *config,
                               uint16_t *buffer)
{
    HAL_StatusTypeDef status;

    /* Validate parameters */
    if (handle == NULL || config == NULL || buffer == NULL)
    {
        return HAL_ERROR;
    }

    if (config->sampling_rate < ADC_DMA_MIN_SAMPLING_RATE ||
        config->sampling_rate > ADC_DMA_MAX_SAMPLING_RATE)
    {
        return HAL_ERROR;
    }

    if (config->buffer_size < 2 || config->buffer_size > ADC_DMA_MAX_BUFFER_SIZE)
    {
        return HAL_ERROR;
    }

    /* Buffer size must be even for double buffering */
    if (config->buffer_size % 2 != 0)
    {
        return HAL_ERROR;
    }

    /* Clear the handle structure */
    memset(handle, 0, sizeof(ADC_DMA_Handle_t));

    /* Store configuration */
    memcpy(&handle->config, config, sizeof(ADC_DMA_Config_t));

    /* Set up buffer pointers for double buffering */
    handle->dma_buffer = buffer;
    handle->half_size = config->buffer_size / 2;
    handle->buffer_half0 = &buffer[0];                 /* First half  */
    handle->buffer_half1 = &buffer[handle->half_size]; /* Second half */

    /* Clear status flags */
    handle->half0_ready = false;
    handle->half1_ready = false;
    handle->is_running = false;
    handle->overrun = false;
    handle->conversion_count = 0;
    handle->callback = NULL;

    /* Store global handle pointer for ISR context */
    g_adc_dma_handle = handle;

    /* ---- Enable peripheral clocks ---- */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* ---- Step 1: Configure GPIO (PA0 as analog input) ---- */
    status = ADC_DMA_ConfigGPIO();
    if (status != HAL_OK)
    {
        return status;
    }

    /* ---- Step 2: Configure DMA2 Stream0 (circular mode) ---- */
    status = ADC_DMA_ConfigDMA(handle);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ---- Step 3: Configure ADC1 (Timer2 TRGO triggered) ---- */
    status = ADC_DMA_ConfigADC(handle, config);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ---- Step 4: Configure Timer2 (sampling rate trigger) ---- */
    status = ADC_DMA_ConfigTimer(handle, config->sampling_rate);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief  Start ADC-DMA continuous acquisition
 *
 * Startup sequence:
 * 1. Clear all status flags and buffer contents
 * 2. Start ADC with DMA in circular mode
 * 3. Start Timer2 to begin triggering conversions
 *
 * After calling this function, DMA will continuously fill the circular
 * buffer and generate half-transfer / transfer-complete interrupts.
 */
HAL_StatusTypeDef ADC_DMA_Start(ADC_DMA_Handle_t *handle)
{
    HAL_StatusTypeDef status;

    if (handle == NULL)
    {
        return HAL_ERROR;
    }

    /* Don't start if already running */
    if (handle->is_running)
    {
        return HAL_OK;
    }

    /* Clear status flags */
    handle->half0_ready = false;
    handle->half1_ready = false;
    handle->overrun = false;
    handle->conversion_count = 0;

    /* Clear the DMA buffer */
    memset(handle->dma_buffer, 0, handle->config.buffer_size * sizeof(uint16_t));

    /* Start ADC with DMA (circular mode, full buffer length) */
    status = HAL_ADC_Start_DMA(&handle->hadc,
                               (uint32_t *)handle->dma_buffer,
                               handle->config.buffer_size);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Start Timer2 to begin triggering ADC conversions */
    status = HAL_TIM_Base_Start(&handle->htim);
    if (status != HAL_OK)
    {
        HAL_ADC_Stop_DMA(&handle->hadc);
        return status;
    }

    handle->is_running = true;
    return HAL_OK;
}

/**
 * @brief  Stop ADC-DMA acquisition
 *
 * Shutdown sequence:
 * 1. Stop Timer2 (no more trigger events)
 * 2. Stop ADC and DMA transfers
 * 3. Clear running flag
 */
HAL_StatusTypeDef ADC_DMA_Stop(ADC_DMA_Handle_t *handle)
{
    if (handle == NULL)
    {
        return HAL_ERROR;
    }

    if (!handle->is_running)
    {
        return HAL_OK;
    }

    /* Stop timer first (no more triggers) */
    HAL_TIM_Base_Stop(&handle->htim);

    /* Stop ADC and DMA */
    HAL_ADC_Stop_DMA(&handle->hadc);

    handle->is_running = false;
    return HAL_OK;
}

/**
 * @brief  Change the sampling rate at runtime
 *
 * Updates the Timer2 auto-reload register to achieve the new sampling rate.
 * The timer period is calculated as: period = (timer_clock / sampling_rate) - 1
 *
 * This can be called while acquisition is running. The new rate takes
 * effect on the next timer update event.
 */
HAL_StatusTypeDef ADC_DMA_SetSamplingRate(ADC_DMA_Handle_t *handle,
                                          uint32_t sampling_rate)
{
    uint32_t period;

    if (handle == NULL)
    {
        return HAL_ERROR;
    }

    /* Validate sampling rate range */
    if (sampling_rate < ADC_DMA_MIN_SAMPLING_RATE ||
        sampling_rate > ADC_DMA_MAX_SAMPLING_RATE)
    {
        return HAL_ERROR;
    }

    /* Calculate new timer period */
    period = ADC_DMA_CalculateTimerPeriod(sampling_rate);

    /* Update timer auto-reload register */
    __HAL_TIM_SET_AUTORELOAD(&handle->htim, period);

    /* Force an update event to load the new period immediately */
    HAL_TIM_GenerateEvent(&handle->htim, TIM_EVENTSOURCE_UPDATE);

    /* Store actual achieved rate */
    handle->actual_sampling_rate = ADC_DMA_TIMER_CLOCK / (period + 1);
    handle->config.sampling_rate = sampling_rate;

    return HAL_OK;
}

/**
 * @brief  Check if a buffer half is ready for processing
 *
 * Checks half0 first, then half1. Returns the first ready buffer.
 * The caller must process the data and then call ADC_DMA_AcknowledgeBuffer().
 *
 * @note This is a non-blocking, polling-style interface. For interrupt-driven
 *       processing, use ADC_DMA_RegisterCallback() instead.
 */
bool ADC_DMA_IsBufferReady(ADC_DMA_Handle_t *handle,
                           uint16_t **buffer_out,
                           uint32_t *length_out)
{
    if (handle == NULL)
    {
        return false;
    }

    /* Check first half-buffer */
    if (handle->half0_ready)
    {
        if (buffer_out != NULL)
        {
            *buffer_out = handle->buffer_half0;
        }
        if (length_out != NULL)
        {
            *length_out = handle->half_size;
        }
        return true;
    }

    /* Check second half-buffer */
    if (handle->half1_ready)
    {
        if (buffer_out != NULL)
        {
            *buffer_out = handle->buffer_half1;
        }
        if (length_out != NULL)
        {
            *length_out = handle->half_size;
        }
        return true;
    }

    return false;
}

/**
 * @brief  Acknowledge that a buffer has been processed
 *
 * Clears the ready flag for whichever half-buffer is currently marked ready.
 * This must be called after processing to prevent false overrun detection.
 */
void ADC_DMA_AcknowledgeBuffer(ADC_DMA_Handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    /* Clear the first ready flag found (priority: half0 first) */
    if (handle->half0_ready)
    {
        handle->half0_ready = false;
    }
    else if (handle->half1_ready)
    {
        handle->half1_ready = false;
    }
}

/**
 * @brief  Register a callback for buffer-ready events
 *
 * The callback is invoked from DMA interrupt context (ISR). It receives
 * a pointer to the ready half-buffer and the number of samples.
 *
 * @warning Keep callback execution time as short as possible to avoid
 *          blocking other interrupts. Ideally, just set a flag or copy
 *          data and process in the main loop.
 */
void ADC_DMA_RegisterCallback(ADC_DMA_Handle_t *handle,
                              ADC_DMA_Callback_t callback)
{
    if (handle == NULL)
    {
        return;
    }
    handle->callback = callback;
}

/**
 * @brief  Get the actual achieved sampling rate
 *
 * Due to integer division when calculating the timer period, the actual
 * sampling rate may differ slightly from the requested rate.
 *
 * Example: Requested 44100 Hz
 *   period = 84000000/44100 - 1 = 1904
 *   actual = 84000000/1905 = 44094.49 Hz (0.01% error)
 */
uint32_t ADC_DMA_GetActualSamplingRate(const ADC_DMA_Handle_t *handle)
{
    if (handle == NULL)
    {
        return 0;
    }
    return handle->actual_sampling_rate;
}

/**
 * @brief  Check if an overrun condition occurred
 *
 * Overrun is detected when DMA fills a new half-buffer before the
 * application has processed the previous one. This indicates that
 * the processing cannot keep up with the sampling rate.
 *
 * The overrun flag is automatically cleared after reading.
 */
bool ADC_DMA_CheckOverrun(ADC_DMA_Handle_t *handle)
{
    bool was_overrun;

    if (handle == NULL)
    {
        return false;
    }

    was_overrun = handle->overrun;
    handle->overrun = false; /* Clear on read */
    return was_overrun;
}

/**
 * @brief  DMA half-transfer complete callback
 *
 * Called when DMA has filled the first half of the circular buffer.
 * At this point:
 * - buffer_half0 contains valid, complete data
 * - DMA is now writing to buffer_half1
 * - Safe to read/process buffer_half0
 */
void ADC_DMA_HalfTransferCallback(ADC_DMA_Handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    /* Check for overrun: previous half0 data was not processed */
    if (handle->half0_ready)
    {
        handle->overrun = true;
    }

    /* Mark first half as ready */
    handle->half0_ready = true;
    handle->conversion_count += handle->half_size;

    /* Invoke user callback if registered */
    if (handle->callback != NULL)
    {
        handle->callback(handle->buffer_half0, handle->half_size);
    }
}

/**
 * @brief  DMA transfer complete callback
 *
 * Called when DMA has filled the entire buffer (second half complete).
 * At this point:
 * - buffer_half1 contains valid, complete data
 * - DMA wraps around and starts writing to buffer_half0 again
 * - Safe to read/process buffer_half1
 */
void ADC_DMA_TransferCompleteCallback(ADC_DMA_Handle_t *handle)
{
    if (handle == NULL)
    {
        return;
    }

    /* Check for overrun: previous half1 data was not processed */
    if (handle->half1_ready)
    {
        handle->overrun = true;
    }

    /* Mark second half as ready */
    handle->half1_ready = true;
    handle->conversion_count += handle->half_size;

    /* Invoke user callback if registered */
    if (handle->callback != NULL)
    {
        handle->callback(handle->buffer_half1, handle->half_size);
    }
}

/* ========================== Private Functions ========================== */

/**
 * @brief  Configure PA0 as analog input for ADC
 *
 * GPIO configuration for analog mode:
 * - No pull-up / pull-down (floating)
 * - Analog mode (GPIO_MODE_ANALOG)
 * - Speed setting is irrelevant for analog mode
 */
static HAL_StatusTypeDef ADC_DMA_ConfigGPIO(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    gpio_init.Pin = ADC_DMA_GPIO_PIN;  /* PA0 */
    gpio_init.Mode = GPIO_MODE_ANALOG; /* Analog input mode */
    gpio_init.Pull = GPIO_NOPULL;      /* No pull-up/pull-down */
    HAL_GPIO_Init(ADC_DMA_GPIO_PORT, &gpio_init);

    return HAL_OK;
}

/**
 * @brief  Configure DMA2 Stream0 for ADC1 transfers
 *
 * DMA configuration:
 * - DMA2 Stream 0, Channel 0 (mapped to ADC1)
 * - Direction: Peripheral (ADC DR) -> Memory (buffer)
 * - Circular mode: auto-restart when buffer is full
 * - Data width: Half-word (16-bit) for both peripheral and memory
 * - Peripheral address fixed (ADC_DR), memory address incrementing
 * - Half-transfer and transfer-complete interrupts enabled
 * - Priority: High (time-critical data acquisition)
 */
static HAL_StatusTypeDef ADC_DMA_ConfigDMA(ADC_DMA_Handle_t *handle)
{
    HAL_StatusTypeDef status;

    /* Configure DMA2 Stream0 Channel0 for ADC1 */
    handle->hdma.Instance = DMA2_Stream0;
    handle->hdma.Init.Channel = DMA_CHANNEL_0;
    handle->hdma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle->hdma.Init.PeriphInc = DMA_PINC_DISABLE;                  /* ADC DR is fixed */
    handle->hdma.Init.MemInc = DMA_MINC_ENABLE;                      /* Buffer increments */
    handle->hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* 16-bit ADC data */
    handle->hdma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* 16-bit buffer */
    handle->hdma.Init.Mode = DMA_CIRCULAR;                           /* Continuous circular */
    handle->hdma.Init.Priority = DMA_PRIORITY_HIGH;
    handle->hdma.Init.FIFOMode = DMA_FIFOMODE_DISABLE; /* Direct mode */
    handle->hdma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    handle->hdma.Init.MemBurst = DMA_MBURST_SINGLE;
    handle->hdma.Init.PeriphBurst = DMA_PBURST_SINGLE;

    status = HAL_DMA_Init(&handle->hdma);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Link DMA handle to ADC handle */
    __HAL_LINKDMA(&handle->hadc, DMA_Handle, handle->hdma);

    /* Configure DMA interrupt priority and enable it */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    return HAL_OK;
}

/**
 * @brief  Configure ADC1 with Timer2 TRGO external trigger
 *
 * ADC configuration:
 * - Single channel (Channel 0 on PA0), single conversion per trigger
 * - External trigger: Timer2 TRGO on rising edge
 * - Continuous conversion mode disabled (timer controls the rate)
 * - DMA request enabled with continuous DMA requests
 * - Right-aligned data, configurable resolution
 * - Scan mode disabled (single channel)
 * - EOC flag set after each conversion
 */
static HAL_StatusTypeDef ADC_DMA_ConfigADC(ADC_DMA_Handle_t *handle,
                                           const ADC_DMA_Config_t *config)
{
    HAL_StatusTypeDef status;
    ADC_ChannelConfTypeDef channel_config = {0};

    /* ---- ADC base configuration ---- */
    handle->hadc.Instance = ADC1;
    handle->hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; /* 84/4 = 21 MHz */
    handle->hadc.Init.Resolution = (uint32_t)config->resolution;
    handle->hadc.Init.ScanConvMode = DISABLE;       /* Single channel */
    handle->hadc.Init.ContinuousConvMode = DISABLE; /* Timer triggered */
    handle->hadc.Init.DiscontinuousConvMode = DISABLE;
    handle->hadc.Init.NbrOfDiscConversion = 0;
    handle->hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    handle->hadc.Init.NbrOfConversion = 1;            /* One conversion per trigger */
    handle->hadc.Init.DMAContinuousRequests = ENABLE; /* Keep DMA running */
    handle->hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    /* Set external trigger source */
    if (config->trigger == ADC_DMA_TRIGGER_SOFTWARE)
    {
        handle->hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
        handle->hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    }
    else
    {
        handle->hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
        handle->hadc.Init.ExternalTrigConv = (uint32_t)config->trigger;
    }

    status = HAL_ADC_Init(&handle->hadc);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ---- Configure ADC channel ---- */
    channel_config.Channel = config->channel;
    channel_config.Rank = 1; /* First (and only) in sequence */
    channel_config.SamplingTime = (uint32_t)config->sample_time;
    channel_config.Offset = 0;

    status = HAL_ADC_ConfigChannel(&handle->hadc, &channel_config);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief  Configure Timer2 as ADC trigger source
 *
 * Timer2 generates TRGO (Trigger Output) events at the desired sampling rate.
 * The timer runs in up-counting mode with update event as trigger output.
 *
 * Timer period calculation:
 *   period = (timer_clock / sampling_rate) - 1
 *
 * Example for 10 kHz sampling:
 *   period = (84,000,000 / 10,000) - 1 = 8399
 *   actual_rate = 84,000,000 / 8400 = 10,000 Hz (exact)
 *
 * Example for 44.1 kHz sampling:
 *   period = (84,000,000 / 44,100) - 1 = 1904
 *   actual_rate = 84,000,000 / 1905 = 44,094.49 Hz (0.01% error)
 */
static HAL_StatusTypeDef ADC_DMA_ConfigTimer(ADC_DMA_Handle_t *handle,
                                             uint32_t sampling_rate)
{
    HAL_StatusTypeDef status;
    TIM_MasterConfigTypeDef master_config = {0};
    uint32_t period;

    /* Calculate timer period for desired sampling rate */
    period = ADC_DMA_CalculateTimerPeriod(sampling_rate);

    /* ---- Timer base configuration ---- */
    handle->htim.Instance = TIM2;
    handle->htim.Init.Prescaler = ADC_DMA_TIMER_PRESCALER; /* No prescaling */
    handle->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    handle->htim.Init.Period = period;
    handle->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handle->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    status = HAL_TIM_Base_Init(&handle->htim);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ---- Configure TRGO output: update event triggers ADC ---- */
    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    status = HAL_TIMEx_MasterConfigSynchronization(&handle->htim, &master_config);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Store actual achieved sampling rate */
    handle->actual_sampling_rate = ADC_DMA_TIMER_CLOCK / (period + 1);

    return HAL_OK;
}

/**
 * @brief  Calculate Timer2 auto-reload period for a given sampling rate
 *
 * @param  sampling_rate  Desired rate in Hz
 * @retval Timer period value (auto-reload register value)
 */
static uint32_t ADC_DMA_CalculateTimerPeriod(uint32_t sampling_rate)
{
    /* period = (clock / rate) - 1 */
    /* Clamp to valid range to prevent timer issues */
    uint32_t period = (ADC_DMA_TIMER_CLOCK / sampling_rate) - 1;

    if (period < 1)
    {
        period = 1; /* Minimum period */
    }

    return period;
}

/* ========================== HAL Callback Overrides ========================== */

/**
 * @brief  HAL ADC conversion complete callback (called by HAL from DMA TC ISR)
 * @param  hadc  Pointer to HAL ADC handle
 *
 * This function is called by the HAL framework when DMA transfer-complete
 * interrupt fires. It delegates to our handle-based callback.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && g_adc_dma_handle != NULL)
    {
        ADC_DMA_TransferCompleteCallback(g_adc_dma_handle);
    }
}

/**
 * @brief  HAL ADC conversion half-complete callback (called by HAL from DMA HT ISR)
 * @param  hadc  Pointer to HAL ADC handle
 *
 * This function is called by the HAL framework when DMA half-transfer
 * interrupt fires. It delegates to our handle-based callback.
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && g_adc_dma_handle != NULL)
    {
        ADC_DMA_HalfTransferCallback(g_adc_dma_handle);
    }
}

/* ========================== IRQ Handler ========================== */

/**
 * @brief  DMA2 Stream0 global interrupt handler
 *
 * This ISR must be present for DMA interrupts to work. It delegates
 * to the HAL DMA IRQ handler which in turn calls the appropriate
 * half-transfer or transfer-complete callbacks above.
 */
void DMA2_Stream0_IRQHandler(void)
{
    if (g_adc_dma_handle != NULL)
    {
        HAL_DMA_IRQHandler(&g_adc_dma_handle->hdma);
    }
}
