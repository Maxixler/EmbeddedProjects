/**
 * @file    adc_dma.h
 * @brief   ADC with DMA driver for STM32F407VG
 * @details Provides ADC1 Channel 0 (PA0) sampling with DMA2 Stream0 circular
 *          mode transfer. Supports Timer2 TRGO triggered conversions with
 *          configurable sampling rates from 1 kHz to 100 kHz. Implements
 *          double buffering using DMA half-transfer and transfer-complete
 *          interrupts for continuous, loss-free data acquisition.
 *
 * @author  Embedded Systems Project
 * @version 1.0
 */

#ifndef ADC_DMA_H
#define ADC_DMA_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================== Includes ========================== */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================== Defines =========================== */

/** @defgroup ADC_DMA_Defaults Default configuration values */
/** @{ */
#define ADC_DMA_DEFAULT_SAMPLING_RATE 10000U /**< Default sampling rate: 10 kHz */
#define ADC_DMA_MIN_SAMPLING_RATE 1000U      /**< Minimum sampling rate: 1 kHz */
#define ADC_DMA_MAX_SAMPLING_RATE 100000U    /**< Maximum sampling rate: 100 kHz */
#define ADC_DMA_DEFAULT_BUFFER_SIZE 2048U    /**< Default DMA buffer size (samples) */
#define ADC_DMA_MAX_BUFFER_SIZE 8192U        /**< Maximum DMA buffer size */
/** @} */

/** @defgroup ADC_DMA_Timer Timer2 configuration for ADC triggering */
/** @{ */
#define ADC_DMA_TIMER_CLOCK 84000000U /**< TIM2 clock: APB1*2 = 84 MHz */
#define ADC_DMA_TIMER_PRESCALER 0U    /**< No prescaling: 84 MHz tick */
/** @} */

/** @defgroup ADC_DMA_Pins ADC input pin definitions */
/** @{ */
#define ADC_DMA_GPIO_PORT GPIOA           /**< ADC input GPIO port */
#define ADC_DMA_GPIO_PIN GPIO_PIN_0       /**< ADC input pin: PA0 */
#define ADC_DMA_ADC_CHANNEL ADC_CHANNEL_0 /**< ADC channel: CH0 */
    /** @} */

    /* ========================== Typedefs ========================== */

    /**
     * @brief ADC resolution options
     */
    typedef enum
    {
        ADC_DMA_RES_12BIT = ADC_RESOLUTION_12B, /**< 12-bit resolution (0-4095) */
        ADC_DMA_RES_10BIT = ADC_RESOLUTION_10B, /**< 10-bit resolution (0-1023) */
        ADC_DMA_RES_8BIT = ADC_RESOLUTION_8B,   /**< 8-bit resolution (0-255)   */
        ADC_DMA_RES_6BIT = ADC_RESOLUTION_6B    /**< 6-bit resolution (0-63)    */
    } ADC_DMA_Resolution_t;

    /**
     * @brief ADC sample time options
     */
    typedef enum
    {
        ADC_DMA_SAMPLETIME_3 = ADC_SAMPLETIME_3CYCLES,     /**< 3 cycles   */
        ADC_DMA_SAMPLETIME_15 = ADC_SAMPLETIME_15CYCLES,   /**< 15 cycles  */
        ADC_DMA_SAMPLETIME_28 = ADC_SAMPLETIME_28CYCLES,   /**< 28 cycles  */
        ADC_DMA_SAMPLETIME_56 = ADC_SAMPLETIME_56CYCLES,   /**< 56 cycles  */
        ADC_DMA_SAMPLETIME_84 = ADC_SAMPLETIME_84CYCLES,   /**< 84 cycles  */
        ADC_DMA_SAMPLETIME_112 = ADC_SAMPLETIME_112CYCLES, /**< 112 cycles */
        ADC_DMA_SAMPLETIME_144 = ADC_SAMPLETIME_144CYCLES, /**< 144 cycles */
        ADC_DMA_SAMPLETIME_480 = ADC_SAMPLETIME_480CYCLES  /**< 480 cycles */
    } ADC_DMA_SampleTime_t;

    /**
     * @brief ADC trigger source options
     */
    typedef enum
    {
        ADC_DMA_TRIGGER_TIM2_TRGO = ADC_EXTERNALTRIGCONV_T2_TRGO, /**< Timer2 TRGO */
        ADC_DMA_TRIGGER_TIM3_TRGO = ADC_EXTERNALTRIGCONV_T3_TRGO, /**< Timer3 TRGO */
        ADC_DMA_TRIGGER_SOFTWARE = 0xFFU                          /**< Software trigger */
    } ADC_DMA_TriggerSource_t;

    /**
     * @brief Callback function pointer type for buffer-ready events
     * @param buffer   Pointer to the ready half-buffer data
     * @param length   Number of samples in the ready buffer
     */
    typedef void (*ADC_DMA_Callback_t)(uint16_t *buffer, uint32_t length);

    /**
     * @brief ADC-DMA configuration structure
     * @details Contains all user-configurable parameters for the ADC-DMA system.
     *          Passed to ADC_DMA_Init() to set up the hardware.
     */
    typedef struct
    {
        uint32_t channel;                 /**< ADC channel number (ADC_CHANNEL_x)       */
        ADC_DMA_SampleTime_t sample_time; /**< ADC sample time per conversion            */
        ADC_DMA_Resolution_t resolution;  /**< ADC resolution (6/8/10/12 bit)            */
        ADC_DMA_TriggerSource_t trigger;  /**< Conversion trigger source                 */
        uint32_t sampling_rate;           /**< Desired sampling rate in Hz (1k-100k)     */
        uint32_t buffer_size;             /**< Total DMA buffer size (samples, must be even) */
    } ADC_DMA_Config_t;

    /**
     * @brief ADC-DMA handle structure
     * @details Maintains the runtime state of the ADC-DMA driver including
     *          HAL handles, buffer pointers, and status flags.
     */
    typedef struct
    {
        /* HAL peripheral handles */
        ADC_HandleTypeDef hadc; /**< HAL ADC handle                            */
        DMA_HandleTypeDef hdma; /**< HAL DMA handle                            */
        TIM_HandleTypeDef htim; /**< HAL Timer handle (trigger source)          */

        /* Double buffer management */
        uint16_t *dma_buffer;   /**< Full DMA circular buffer                   */
        uint16_t *buffer_half0; /**< Pointer to first half of DMA buffer        */
        uint16_t *buffer_half1; /**< Pointer to second half of DMA buffer       */
        uint32_t half_size;     /**< Size of each half-buffer (buffer_size / 2) */

        /* Configuration */
        ADC_DMA_Config_t config;       /**< Active configuration                       */
        uint32_t actual_sampling_rate; /**< Actual achieved sampling rate (Hz)    */

        /* Status flags */
        volatile bool half0_ready;          /**< Flag: first half-buffer ready for processing  */
        volatile bool half1_ready;          /**< Flag: second half-buffer ready for processing */
        volatile bool is_running;           /**< Flag: ADC-DMA acquisition is active           */
        volatile bool overrun;              /**< Flag: buffer overrun detected                  */
        volatile uint32_t conversion_count; /**< Total number of completed conversions       */

        /* User callback */
        ADC_DMA_Callback_t callback; /**< User callback for buffer-ready notification   */
    } ADC_DMA_Handle_t;

    /* ========================== Function Prototypes ========================== */

    /**
     * @brief  Initialize the ADC-DMA system
     * @details Configures ADC1 on PA0 (Channel 0), DMA2 Stream0 in circular mode,
     *          and Timer2 as the conversion trigger. Sets up GPIO, clocks, and
     *          interrupt priorities.
     *
     * @param  handle  Pointer to ADC_DMA handle structure (must be pre-allocated)
     * @param  config  Pointer to configuration structure with desired parameters
     * @param  buffer  Pointer to DMA buffer (must be aligned, size = config->buffer_size)
     * @retval HAL_OK on success, HAL_ERROR on failure
     */
    HAL_StatusTypeDef ADC_DMA_Init(ADC_DMA_Handle_t *handle,
                                   const ADC_DMA_Config_t *config,
                                   uint16_t *buffer);

    /**
     * @brief  Start ADC-DMA continuous acquisition
     * @details Enables DMA interrupts, starts the timer trigger, and begins
     *          ADC conversions. Data flows continuously into the circular buffer.
     *
     * @param  handle  Pointer to initialized ADC_DMA handle
     * @retval HAL_OK on success, HAL_ERROR on failure
     */
    HAL_StatusTypeDef ADC_DMA_Start(ADC_DMA_Handle_t *handle);

    /**
     * @brief  Stop ADC-DMA acquisition
     * @details Stops the timer trigger, halts ADC conversions, and disables
     *          DMA transfers. Buffer contents remain valid.
     *
     * @param  handle  Pointer to active ADC_DMA handle
     * @retval HAL_OK on success, HAL_ERROR on failure
     */
    HAL_StatusTypeDef ADC_DMA_Stop(ADC_DMA_Handle_t *handle);

    /**
     * @brief  Change the sampling rate at runtime
     * @details Updates Timer2 auto-reload register to achieve the new rate.
     *          Can be called while acquisition is running (glitch-free).
     *
     * @param  handle         Pointer to ADC_DMA handle
     * @param  sampling_rate  New sampling rate in Hz (1000-100000)
     * @retval HAL_OK on success, HAL_ERROR if rate is out of range
     */
    HAL_StatusTypeDef ADC_DMA_SetSamplingRate(ADC_DMA_Handle_t *handle,
                                              uint32_t sampling_rate);

    /**
     * @brief  Check if a buffer half is ready for processing
     * @details Non-blocking check. After processing, call ADC_DMA_AcknowledgeBuffer()
     *          to clear the ready flag.
     *
     * @param  handle      Pointer to ADC_DMA handle
     * @param  buffer_out  Output: pointer to the ready half-buffer data
     * @param  length_out  Output: number of samples in the ready buffer
     * @retval true if a buffer is ready, false otherwise
     */
    bool ADC_DMA_IsBufferReady(ADC_DMA_Handle_t *handle,
                               uint16_t **buffer_out,
                               uint32_t *length_out);

    /**
     * @brief  Acknowledge that a buffer has been processed
     * @details Clears the ready flag for the buffer that was most recently returned
     *          by ADC_DMA_IsBufferReady(). Must be called after processing to
     *          prevent overrun detection.
     *
     * @param  handle  Pointer to ADC_DMA handle
     */
    void ADC_DMA_AcknowledgeBuffer(ADC_DMA_Handle_t *handle);

    /**
     * @brief  Register a callback for buffer-ready events
     * @details The callback is invoked from DMA ISR context when a half-buffer
     *          becomes available. Keep callback execution time minimal.
     *
     * @param  handle    Pointer to ADC_DMA handle
     * @param  callback  Function pointer to the callback (NULL to disable)
     */
    void ADC_DMA_RegisterCallback(ADC_DMA_Handle_t *handle,
                                  ADC_DMA_Callback_t callback);

    /**
     * @brief  Get the actual achieved sampling rate
     * @details Due to integer division of timer clock, the actual rate may differ
     *          slightly from the requested rate.
     *
     * @param  handle  Pointer to ADC_DMA handle
     * @retval Actual sampling rate in Hz
     */
    uint32_t ADC_DMA_GetActualSamplingRate(const ADC_DMA_Handle_t *handle);

    /**
     * @brief  Check if an overrun condition occurred
     * @details Overrun happens when DMA fills a half-buffer before the previous
     *          one was processed. Clears the overrun flag after reading.
     *
     * @param  handle  Pointer to ADC_DMA handle
     * @retval true if overrun occurred since last check, false otherwise
     */
    bool ADC_DMA_CheckOverrun(ADC_DMA_Handle_t *handle);

    /**
     * @brief  DMA half-transfer complete ISR handler
     * @details Called from DMA2_Stream0_IRQHandler. Sets half0_ready flag
     *          and invokes user callback if registered.
     *
     * @param  handle  Pointer to ADC_DMA handle
     */
    void ADC_DMA_HalfTransferCallback(ADC_DMA_Handle_t *handle);

    /**
     * @brief  DMA transfer complete ISR handler
     * @details Called from DMA2_Stream0_IRQHandler. Sets half1_ready flag
     *          and invokes user callback if registered.
     *
     * @param  handle  Pointer to ADC_DMA handle
     */
    void ADC_DMA_TransferCompleteCallback(ADC_DMA_Handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* ADC_DMA_H */
