/**
 * @file    main.c
 * @brief   ADC-DMA-FFT Real-Time Spectrum Analyzer for STM32F407VG
 * @details Main application implementing a complete real-time spectrum analyzer:
 *
 *          System Architecture:
 *          +---------+     +------+     +---------+     +----------+
 *          | PA0     | --> | ADC1 | --> | DMA2    | --> | Buffer   |
 *          | (Analog)|     | 12bit|     | Stream0 |     | (double) |
 *          +---------+     +------+     +---------+     +----------+
 *               ^             ^                              |
 *               |             |                              v
 *          Signal        TIM2 TRGO                    +-----------+
 *          Source        (fs trigger)                  | FFT       |
 *                                                     | Analyzer  |
 *                                                     +-----------+
 *                                                          |
 *                                                          v
 *                                                     +-----------+
 *                                                     | UART2     |
 *                                                     | 115200    |
 *                                                     +-----------+
 *
 *          Features:
 *          - Configurable sampling rate: 1 kHz - 100 kHz (default 10 kHz)
 *          - Configurable FFT size: 256/512/1024/2048/4096 (default 1024)
 *          - Window functions: Rectangular, Hanning, Hamming, Blackman
 *          - Output modes: Spectrum, Peak, Raw, Stats
 *          - UART command interface for runtime configuration
 *          - LED status indicators on PD12-PD15
 *          - Double-buffered DMA for zero data loss
 *
 *          Clock configuration:
 *          - SYSCLK: 168 MHz (HSE 8MHz -> PLL)
 *          - AHB: 168 MHz, APB1: 42 MHz, APB2: 84 MHz
 *          - ADC clock: 21 MHz (APB2/4)
 *          - Timer2 clock: 84 MHz (APB1 * 2)
 *
 * @author  Embedded Systems Project
 * @version 1.0
 */

/* ========================== Includes ========================== */
#include "stm32f4xx_hal.h"
#include "adc_dma.h"
#include "fft_analyzer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

/* ========================== Defines ========================== */

/** @defgroup LED_Pins LED indicator pin definitions (Discovery board) */
/** @{ */
#define LED_GREEN_PIN GPIO_PIN_12  /**< PD12: System ready / idle */
#define LED_ORANGE_PIN GPIO_PIN_13 /**< PD13: ADC sampling active */
#define LED_RED_PIN GPIO_PIN_14    /**< PD14: FFT processing active */
#define LED_BLUE_PIN GPIO_PIN_15   /**< PD15: Data transmitting */
#define LED_GPIO_PORT GPIOD
/** @} */

/** @defgroup Default_Config Default system configuration */
/** @{ */
#define DEFAULT_SAMPLING_RATE 10000U           /**< Default: 10 kHz sampling */
#define DEFAULT_FFT_SIZE 1024U                 /**< Default: 1024-point FFT */
#define DEFAULT_WINDOW_TYPE FFT_WINDOW_HANNING /**< Default: Hanning window */
/** @} */

/** @defgroup Buffer_Sizes Memory allocation sizes */
/** @{ */
#define ADC_BUFFER_SIZE (FFT_MAX_SIZE * 2) /**< DMA buffer: 2x max FFT size */
#define UART_RX_BUFFER_SIZE 64             /**< UART receive buffer */
#define UART_TX_BUFFER_SIZE 256            /**< UART transmit line buffer */
/** @} */

/**
 * @brief Output mode enumeration
 */
typedef enum
{
    OUTPUT_MODE_SPECTRUM = 0, /**< Full spectrum (bin, freq, mag_dB) */
    OUTPUT_MODE_PEAK,         /**< Peak frequencies and THD/SNR */
    OUTPUT_MODE_RAW,          /**< Raw magnitude values */
    OUTPUT_MODE_STATS         /**< Statistical summary */
} OutputMode_t;

/* ========================== Global Variables ========================== */

/* Peripheral handles */
static UART_HandleTypeDef huart2; /**< UART2 handle for debug/commands */

/* ADC-DMA system */
static ADC_DMA_Handle_t adc_dma_handle; /**< ADC-DMA driver handle */
static uint16_t adc_buffer[ADC_BUFFER_SIZE] __attribute__((aligned(4)));

/* FFT analyzer */
static FFT_Analyzer_t fft_analyzer;               /**< FFT analyzer handle */
static float32_t fft_input_buf[FFT_MAX_SIZE];     /**< FFT input buffer */
static float32_t fft_output_buf[FFT_MAX_SIZE];    /**< FFT output buffer */
static float32_t window_buf[FFT_MAX_SIZE];        /**< Window coefficients */
static float32_t mag_buf[FFT_MAX_SIZE / 2];       /**< Magnitude buffer */
static float32_t mag_db_buf[FFT_MAX_SIZE / 2];    /**< Magnitude dB buffer */
static float32_t freq_axis_buf[FFT_MAX_SIZE / 2]; /**< Frequency axis */

/* FFT result structure */
static FFT_Result_t fft_result;

/* Processing buffer: copy from DMA buffer for safe processing */
static uint16_t process_buffer[FFT_MAX_SIZE];

/* UART communication */
static char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_byte;                 /**< Single byte for interrupt RX */
static volatile uint8_t uart_rx_index = 0;   /**< Current position in RX buffer */
static volatile bool uart_cmd_ready = false; /**< Flag: command received */

/* System state */
static OutputMode_t output_mode = OUTPUT_MODE_PEAK;
static bool continuous_mode = false;      /**< Continuous output flag */
static volatile bool single_shot = false; /**< Single measurement flag */
static uint32_t current_fft_size = DEFAULT_FFT_SIZE;
static uint32_t current_sampling_rate = DEFAULT_SAMPLING_RATE;

/* ========================== Private Function Prototypes ========================== */

/* System initialization */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART2_Init(void);
static void System_Init(void);

/* Processing pipeline */
static void ProcessFFTData(uint16_t *data, uint32_t length);
static void OutputSpectrum(const FFT_Result_t *result);
static void OutputPeaks(const FFT_Result_t *result);
static void OutputRaw(const FFT_Result_t *result);
static void OutputStats(const FFT_Result_t *result);

/* UART communication */
static void UART_SendString(const char *str);
static void UART_Printf(const char *fmt, ...);
static void ProcessCommand(const char *cmd);
static void PrintHelp(void);
static void PrintConfig(void);

/* Command handlers */
static void CMD_SetFFTSize(const char *param);
static void CMD_SetSamplingRate(const char *param);
static void CMD_SetWindowType(const char *param);
static void CMD_SetOutputMode(const char *param);

/* LED control */
static void LED_SetState(uint16_t pin, bool state);
static void LED_Toggle(uint16_t pin);

/* ========================== Main Function ========================== */

/**
 * @brief  Application entry point
 *
 * Main loop operation:
 * 1. Initialize all peripherals (clock, GPIO, UART, ADC-DMA, FFT)
 * 2. Start ADC-DMA acquisition
 * 3. Poll for:
 *    a. Buffer-ready events from DMA (double buffering)
 *    b. UART commands from user
 * 4. When buffer is ready and output is requested:
 *    - Copy data from DMA buffer (safe snapshot)
 *    - Run FFT analysis pipeline
 *    - Output results via UART
 * 5. Process any pending UART commands
 */
int main(void)
{
    uint16_t *ready_buffer = NULL;
    uint32_t ready_length = 0;

    /* ---- System Initialization ---- */
    HAL_Init();
    SystemClock_Config();
    System_Init();

    /* Print startup banner */
    UART_SendString("\r\n");
    UART_SendString("============================================\r\n");
    UART_SendString("  STM32F407VG ADC-DMA-FFT Spectrum Analyzer\r\n");
    UART_SendString("============================================\r\n");
    PrintConfig();
    UART_SendString("Type HELP for command list.\r\n\r\n");

    /* Start ADC-DMA acquisition */
    if (ADC_DMA_Start(&adc_dma_handle) != HAL_OK)
    {
        UART_SendString("ERROR: Failed to start ADC-DMA!\r\n");
        /* Error indication: blink red LED */
        while (1)
        {
            LED_Toggle(LED_RED_PIN);
            HAL_Delay(200);
        }
    }

    /* Green LED on: system ready */
    LED_SetState(LED_GREEN_PIN, true);
    UART_SendString("ADC-DMA started. System ready.\r\n");

    /* Start UART receive interrupt for command input */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    /* ========================== Main Loop ========================== */
    while (1)
    {
        /* ---- Check for ready ADC buffer ---- */
        if (ADC_DMA_IsBufferReady(&adc_dma_handle, &ready_buffer, &ready_length))
        {

            /* Orange LED on: sampling active indication */
            LED_SetState(LED_ORANGE_PIN, true);

            /* Process data if continuous mode or single-shot requested */
            if (continuous_mode || single_shot)
            {

                /* Ensure we have enough samples for the current FFT size */
                if (ready_length >= current_fft_size)
                {
                    ProcessFFTData(ready_buffer, current_fft_size);

                    /* Clear single-shot flag after processing */
                    if (single_shot)
                    {
                        single_shot = false;
                    }
                }
            }

            /* Acknowledge the buffer so DMA can reuse it */
            ADC_DMA_AcknowledgeBuffer(&adc_dma_handle);

            /* Check for overrun (processing too slow) */
            if (ADC_DMA_CheckOverrun(&adc_dma_handle))
            {
                UART_SendString("WARNING: Buffer overrun detected!\r\n");
            }

            LED_SetState(LED_ORANGE_PIN, false);
        }

        /* ---- Process UART commands ---- */
        if (uart_cmd_ready)
        {
            uart_cmd_ready = false;
            ProcessCommand(uart_rx_buffer);
            /* Reset RX buffer index for next command */
            uart_rx_index = 0;
            memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
        }
    }
}

/* ========================== Initialization Functions ========================== */

/**
 * @brief  Configure system clock to 168 MHz using HSE + PLL
 *
 * Clock tree:
 *   HSE (8 MHz) -> PLL_M (/8) -> VCO_IN (1 MHz)
 *                -> PLL_N (*336) -> VCO_OUT (336 MHz)
 *                -> PLL_P (/2) -> SYSCLK (168 MHz)
 *                -> PLL_Q (/7) -> USB_CLK (48 MHz)
 *
 *   SYSCLK (168 MHz) -> AHB  (/1) = 168 MHz
 *                     -> APB1 (/4) = 42 MHz (TIM2, USART2)
 *                     -> APB2 (/2) = 84 MHz (ADC1)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure HSE and PLL */
    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_ON;
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc_init.PLL.PLLM = 8;             /* VCO input: 8 MHz / 8 = 1 MHz */
    osc_init.PLL.PLLN = 336;           /* VCO output: 1 MHz * 336 = 336 MHz */
    osc_init.PLL.PLLP = RCC_PLLP_DIV2; /* SYSCLK: 336 / 2 = 168 MHz */
    osc_init.PLL.PLLQ = 7;             /* USB: 336 / 7 = 48 MHz */

    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)
    {
        while (1)
            ; /* Clock configuration failed */
    }

    /* Configure bus clocks */
    clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1; /* AHB = 168 MHz */
    clk_init.APB1CLKDivider = RCC_HCLK_DIV4;  /* APB1 = 42 MHz */
    clk_init.APB2CLKDivider = RCC_HCLK_DIV2;  /* APB2 = 84 MHz */

    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5) != HAL_OK)
    {
        while (1)
            ; /* Clock configuration failed */
    }
}

/**
 * @brief  Initialize GPIO for LEDs and UART pins
 *
 * LED pins (PD12-PD15): Push-pull output, no pull-up/down
 * UART pins (PA2, PA3): Alternate function (AF7 = USART2)
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* ---- LED pins: PD12 (Green), PD13 (Orange), PD14 (Red), PD15 (Blue) ---- */
    gpio_init.Pin = LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &gpio_init);

    /* Turn off all LEDs initially */
    HAL_GPIO_WritePin(LED_GPIO_PORT,
                      LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN,
                      GPIO_PIN_RESET);

    /* ---- UART2 pins: PA2 (TX), PA3 (RX) ---- */
    gpio_init.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio_init);
}

/**
 * @brief  Initialize UART2 at 115200 baud for debug output and commands
 *
 * Configuration: 115200-8-N-1, no hardware flow control
 * Interrupt-based receive for command processing
 */
static void UART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while (1)
            ; /* UART init failed */
    }

    /* Enable UART receive interrupt */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief  Initialize all system components
 *
 * Initialization order:
 * 1. GPIO (LEDs, UART pins)
 * 2. UART2 (debug/commands)
 * 3. ADC-DMA (data acquisition)
 * 4. FFT analyzer (signal processing)
 */
static void System_Init(void)
{
    ADC_DMA_Config_t adc_config;
    FFT_Config_t fft_config;

    /* Initialize GPIO and UART */
    GPIO_Init();
    UART2_Init();

    UART_SendString("Initializing system...\r\n");

    /* ---- Initialize ADC-DMA ---- */
    adc_config.channel = ADC_DMA_ADC_CHANNEL;       /* Channel 0 (PA0) */
    adc_config.sample_time = ADC_DMA_SAMPLETIME_15; /* 15 cycles */
    adc_config.resolution = ADC_DMA_RES_12BIT;      /* 12-bit */
    adc_config.trigger = ADC_DMA_TRIGGER_TIM2_TRGO; /* Timer2 trigger */
    adc_config.sampling_rate = current_sampling_rate;
    adc_config.buffer_size = current_fft_size * 2; /* Double buffer */

    if (ADC_DMA_Init(&adc_dma_handle, &adc_config, adc_buffer) != HAL_OK)
    {
        UART_SendString("ERROR: ADC-DMA init failed!\r\n");
        while (1)
            ;
    }
    UART_SendString("  ADC-DMA initialized.\r\n");

    /* ---- Initialize FFT Analyzer ---- */
    fft_config.fft_size = current_fft_size;
    fft_config.fs = (float32_t)current_sampling_rate;
    fft_config.window_type = DEFAULT_WINDOW_TYPE;

    /* Set up result structure buffer pointers */
    memset(&fft_result, 0, sizeof(FFT_Result_t));
    fft_result.magnitude = mag_buf;
    fft_result.magnitude_db = mag_db_buf;
    fft_result.frequency = freq_axis_buf;

    if (FFT_Analyzer_Init(&fft_analyzer, &fft_config,
                          fft_input_buf, fft_output_buf,
                          window_buf, mag_buf) != ARM_MATH_SUCCESS)
    {
        UART_SendString("ERROR: FFT analyzer init failed!\r\n");
        while (1)
            ;
    }
    UART_SendString("  FFT analyzer initialized.\r\n");

    UART_Printf("  Actual sampling rate: %lu Hz\r\n",
                ADC_DMA_GetActualSamplingRate(&adc_dma_handle));
    UART_Printf("  Frequency resolution: %.2f Hz\r\n",
                fft_analyzer.freq_resolution);
}

/* ========================== Processing Pipeline ========================== */

/**
 * @brief  Process a buffer of ADC data through the FFT pipeline
 *
 * Processing steps:
 * 1. Copy data from DMA buffer (atomic snapshot while DMA writes to other half)
 * 2. Red LED on (processing indicator)
 * 3. Run FFT analysis (window -> FFT -> magnitude -> peaks -> THD/SNR)
 * 4. Blue LED on (transmitting indicator)
 * 5. Output results according to selected mode
 * 6. LEDs off
 *
 * @param data    Pointer to ADC sample data
 * @param length  Number of samples to process
 */
static void ProcessFFTData(uint16_t *data, uint32_t length)
{
    /* Step 1: Copy data from DMA buffer to processing buffer
     * This ensures the DMA can safely overwrite the original while we process */
    memcpy(process_buffer, data, length * sizeof(uint16_t));

    /* Step 2: Red LED on - FFT processing */
    LED_SetState(LED_RED_PIN, true);

    /* Step 3: Run the complete FFT analysis pipeline */
    if (!FFT_Analyzer_Process(&fft_analyzer, process_buffer, length, &fft_result))
    {
        UART_SendString("ERROR: FFT processing failed!\r\n");
        LED_SetState(LED_RED_PIN, false);
        return;
    }

    LED_SetState(LED_RED_PIN, false);

    /* Step 4: Blue LED on - transmitting results */
    LED_SetState(LED_BLUE_PIN, true);

    /* Step 5: Output results based on selected mode */
    switch (output_mode)
    {
    case OUTPUT_MODE_SPECTRUM:
        OutputSpectrum(&fft_result);
        break;

    case OUTPUT_MODE_PEAK:
        OutputPeaks(&fft_result);
        break;

    case OUTPUT_MODE_RAW:
        OutputRaw(&fft_result);
        break;

    case OUTPUT_MODE_STATS:
        OutputStats(&fft_result);
        break;
    }

    LED_SetState(LED_BLUE_PIN, false);
}

/* ========================== Output Functions ========================== */

/**
 * @brief  Output full spectrum data via UART
 *
 * Format:
 *   --- SPECTRUM ---
 *   BIN,FREQ_HZ,MAG_DB
 *   0,0.00,-45.2
 *   1,9.77,-42.1
 *   ...
 *   --- END ---
 */
static void OutputSpectrum(const FFT_Result_t *result)
{
    uint32_t i;
    uint32_t half_size = result->fft_size / 2;

    UART_SendString("--- SPECTRUM ---\r\n");
    UART_SendString("BIN,FREQ_HZ,MAG_DB\r\n");

    for (i = 0; i < half_size; i++)
    {
        UART_Printf("%lu,%.2f,%.1f\r\n",
                    i,
                    result->frequency[i],
                    result->magnitude_db[i]);
    }

    UART_SendString("--- END ---\r\n\r\n");
}

/**
 * @brief  Output peak detection results via UART
 *
 * Format:
 *   --- PEAKS ---
 *   PEAK1: 1000.0 Hz, -0.5 dB
 *   PEAK2: 2000.0 Hz, -25.3 dB
 *   THD: 3.45%
 *   SNR: 62.3 dB
 *   --- END ---
 */
static void OutputPeaks(const FFT_Result_t *result)
{
    uint32_t i;

    UART_SendString("--- PEAKS ---\r\n");

    if (result->num_peaks == 0)
    {
        UART_SendString("No peaks detected.\r\n");
    }
    else
    {
        for (i = 0; i < result->num_peaks && i < 5; i++)
        {
            UART_Printf("PEAK%lu: %.1f Hz, %.1f dB\r\n",
                        i + 1,
                        result->peaks[i].frequency,
                        result->peaks[i].magnitude_db);
        }
    }

    UART_Printf("THD: %.2f%%\r\n", result->thd);
    UART_Printf("SNR: %.1f dB\r\n", result->snr);
    UART_SendString("--- END ---\r\n\r\n");
}

/**
 * @brief  Output raw magnitude values via UART
 *
 * Format:
 *   --- RAW ---
 *   0.00123,0.00456,0.78901,...
 *   --- END ---
 */
static void OutputRaw(const FFT_Result_t *result)
{
    uint32_t i;
    uint32_t half_size = result->fft_size / 2;

    UART_SendString("--- RAW ---\r\n");

    for (i = 0; i < half_size; i++)
    {
        if (i > 0 && i % 16 == 0)
        {
            UART_SendString("\r\n");
        }
        UART_Printf("%.5f", result->magnitude[i]);
        if (i < half_size - 1)
        {
            UART_SendString(",");
        }
    }
    UART_SendString("\r\n--- END ---\r\n\r\n");
}

/**
 * @brief  Output statistical summary via UART
 *
 * Format:
 *   --- STATS ---
 *   Fs: 10000 Hz
 *   N: 1024
 *   df: 9.77 Hz
 *   Window: HANNING
 *   Peak Freq: 1000.0 Hz
 *   Peak Mag: -0.5 dB
 *   RMS: 0.452 V
 *   DC Offset: 1.648 V
 *   THD: 3.45%
 *   SNR: 62.3 dB
 *   --- END ---
 */
static void OutputStats(const FFT_Result_t *result)
{
    UART_SendString("--- STATS ---\r\n");
    UART_Printf("Fs: %lu Hz\r\n", (uint32_t)result->fs);
    UART_Printf("N: %lu\r\n", result->fft_size);
    UART_Printf("df: %.2f Hz\r\n", result->freq_resolution);
    UART_Printf("Window: %s\r\n",
                FFT_Analyzer_GetWindowName(fft_analyzer.config.window_type));
    UART_Printf("Peak Freq: %.1f Hz\r\n", result->peak_frequency);
    UART_Printf("Peak Mag: %.1f dB\r\n", result->peak_magnitude_db);
    UART_Printf("RMS: %.4f V\r\n", result->rms_value);
    UART_Printf("DC Offset: %.3f V\r\n", result->dc_offset);
    UART_Printf("THD: %.2f%%\r\n", result->thd);
    UART_Printf("SNR: %.1f dB\r\n", result->snr);
    UART_SendString("--- END ---\r\n\r\n");
}

/* ========================== UART Communication ========================== */

/**
 * @brief  Send a null-terminated string via UART2 (blocking)
 */
static void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief  Printf-style formatted output via UART2
 * @details Uses a local buffer with vsnprintf for safe formatting.
 *          Maximum output length is UART_TX_BUFFER_SIZE characters.
 */
static void UART_Printf(const char *fmt, ...)
{
    char buf[UART_TX_BUFFER_SIZE];
    va_list args;

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    UART_SendString(buf);
}

/**
 * @brief  Process a received UART command
 *
 * Supported commands:
 *   FFT <size>      - Set FFT size (256/512/1024/2048/4096)
 *   FS <rate>       - Set sampling rate in Hz (1000-100000)
 *   WIN <type>      - Set window (RECT/HANN/HAMM/BLACK)
 *   MODE <mode>     - Set output mode (SPEC/PEAK/RAW/STATS)
 *   START           - Start continuous output
 *   STOP            - Stop continuous output
 *   SINGLE          - Single-shot measurement
 *   CONFIG          - Show current configuration
 *   HELP            - Show command list
 */
static void ProcessCommand(const char *cmd)
{
    char cmd_upper[UART_RX_BUFFER_SIZE];
    char *param;
    uint32_t i;

    /* Convert command to uppercase for case-insensitive matching */
    for (i = 0; i < strlen(cmd) && i < UART_RX_BUFFER_SIZE - 1; i++)
    {
        cmd_upper[i] = (cmd[i] >= 'a' && cmd[i] <= 'z') ? (cmd[i] - 'a' + 'A') : cmd[i];
    }
    cmd_upper[i] = '\0';

    /* Remove trailing whitespace */
    while (i > 0 && (cmd_upper[i - 1] == ' ' || cmd_upper[i - 1] == '\r' ||
                     cmd_upper[i - 1] == '\n'))
    {
        cmd_upper[--i] = '\0';
    }

    /* Find parameter (first space separates command from parameter) */
    param = strchr(cmd_upper, ' ');
    if (param != NULL)
    {
        *param = '\0'; /* Null-terminate the command part */
        param++;       /* Point to parameter */
        /* Skip leading spaces in parameter */
        while (*param == ' ')
            param++;
    }

    /* ---- Dispatch commands ---- */
    if (strcmp(cmd_upper, "FFT") == 0 && param != NULL)
    {
        CMD_SetFFTSize(param);
    }
    else if (strcmp(cmd_upper, "FS") == 0 && param != NULL)
    {
        CMD_SetSamplingRate(param);
    }
    else if (strcmp(cmd_upper, "WIN") == 0 && param != NULL)
    {
        CMD_SetWindowType(param);
    }
    else if (strcmp(cmd_upper, "MODE") == 0 && param != NULL)
    {
        CMD_SetOutputMode(param);
    }
    else if (strcmp(cmd_upper, "START") == 0)
    {
        continuous_mode = true;
        UART_SendString("Continuous mode started.\r\n");
    }
    else if (strcmp(cmd_upper, "STOP") == 0)
    {
        continuous_mode = false;
        UART_SendString("Continuous mode stopped.\r\n");
    }
    else if (strcmp(cmd_upper, "SINGLE") == 0)
    {
        single_shot = true;
        UART_SendString("Single measurement triggered.\r\n");
    }
    else if (strcmp(cmd_upper, "CONFIG") == 0)
    {
        PrintConfig();
    }
    else if (strcmp(cmd_upper, "HELP") == 0)
    {
        PrintHelp();
    }
    else if (strlen(cmd_upper) > 0)
    {
        UART_Printf("Unknown command: %s\r\n", cmd_upper);
        UART_SendString("Type HELP for command list.\r\n");
    }
}

/**
 * @brief  Print available commands
 */
static void PrintHelp(void)
{
    UART_SendString("\r\n=== COMMAND LIST ===\r\n");
    UART_SendString("FFT <256|512|1024|2048|4096>  Set FFT size\r\n");
    UART_SendString("FS <1000-100000>              Set sampling rate (Hz)\r\n");
    UART_SendString("WIN <RECT|HANN|HAMM|BLACK>    Set window function\r\n");
    UART_SendString("MODE <SPEC|PEAK|RAW|STATS>    Set output mode\r\n");
    UART_SendString("START                         Start continuous output\r\n");
    UART_SendString("STOP                          Stop continuous output\r\n");
    UART_SendString("SINGLE                        Single measurement\r\n");
    UART_SendString("CONFIG                        Show configuration\r\n");
    UART_SendString("HELP                          Show this help\r\n");
    UART_SendString("====================\r\n\r\n");
}

/**
 * @brief  Print current system configuration
 */
static void PrintConfig(void)
{
    UART_SendString("\r\n--- CONFIGURATION ---\r\n");
    UART_Printf("FFT Size: %lu\r\n", current_fft_size);
    UART_Printf("Sampling Rate: %lu Hz (actual: %lu Hz)\r\n",
                current_sampling_rate,
                ADC_DMA_GetActualSamplingRate(&adc_dma_handle));
    UART_Printf("Freq Resolution: %.2f Hz\r\n", fft_analyzer.freq_resolution);
    UART_Printf("Nyquist Freq: %lu Hz\r\n", current_sampling_rate / 2);
    UART_Printf("Window: %s\r\n",
                FFT_Analyzer_GetWindowName(fft_analyzer.config.window_type));

    const char *mode_names[] = {"SPECTRUM", "PEAK", "RAW", "STATS"};
    UART_Printf("Output Mode: %s\r\n", mode_names[output_mode]);
    UART_Printf("Continuous: %s\r\n", continuous_mode ? "ON" : "OFF");
    UART_SendString("---------------------\r\n\r\n");
}

/* ========================== Command Handlers ========================== */

/**
 * @brief  Handle FFT size change command
 *
 * Valid sizes: 256, 512, 1024, 2048, 4096
 * Requires reinitializing the FFT analyzer and adjusting the DMA buffer.
 */
static void CMD_SetFFTSize(const char *param)
{
    uint32_t new_size = (uint32_t)atoi(param);

    /* Validate FFT size */
    if (new_size != 256 && new_size != 512 && new_size != 1024 &&
        new_size != 2048 && new_size != 4096)
    {
        UART_SendString("ERROR: Invalid FFT size. Use 256/512/1024/2048/4096\r\n");
        return;
    }

    /* Check if DMA buffer is large enough */
    if (new_size * 2 > ADC_BUFFER_SIZE)
    {
        UART_SendString("ERROR: FFT size too large for buffer.\r\n");
        return;
    }

    /* Stop acquisition during reconfiguration */
    bool was_continuous = continuous_mode;
    continuous_mode = false;
    ADC_DMA_Stop(&adc_dma_handle);

    /* Update FFT analyzer size */
    if (FFT_Analyzer_SetSize(&fft_analyzer, new_size) != ARM_MATH_SUCCESS)
    {
        UART_SendString("ERROR: Failed to set FFT size.\r\n");
        ADC_DMA_Start(&adc_dma_handle);
        continuous_mode = was_continuous;
        return;
    }

    /* Update sampling frequency in analyzer config */
    fft_analyzer.config.fs = (float32_t)current_sampling_rate;
    fft_analyzer.freq_resolution = (float32_t)current_sampling_rate / (float32_t)new_size;

    current_fft_size = new_size;

    /* Reconfigure ADC-DMA with new buffer size */
    ADC_DMA_Config_t adc_config;
    adc_config.channel = ADC_DMA_ADC_CHANNEL;
    adc_config.sample_time = ADC_DMA_SAMPLETIME_15;
    adc_config.resolution = ADC_DMA_RES_12BIT;
    adc_config.trigger = ADC_DMA_TRIGGER_TIM2_TRGO;
    adc_config.sampling_rate = current_sampling_rate;
    adc_config.buffer_size = new_size * 2;

    if (ADC_DMA_Init(&adc_dma_handle, &adc_config, adc_buffer) != HAL_OK)
    {
        UART_SendString("ERROR: ADC-DMA reinit failed!\r\n");
        return;
    }

    /* Restart acquisition */
    ADC_DMA_Start(&adc_dma_handle);
    continuous_mode = was_continuous;

    UART_Printf("FFT size set to %lu. Resolution: %.2f Hz\r\n",
                new_size, fft_analyzer.freq_resolution);
}

/**
 * @brief  Handle sampling rate change command
 *
 * Valid range: 1000 - 100000 Hz
 * Updates Timer2 period for new rate. Can be done while running.
 */
static void CMD_SetSamplingRate(const char *param)
{
    uint32_t new_rate = (uint32_t)atoi(param);

    if (new_rate < ADC_DMA_MIN_SAMPLING_RATE ||
        new_rate > ADC_DMA_MAX_SAMPLING_RATE)
    {
        UART_Printf("ERROR: Rate out of range (%u-%u Hz)\r\n",
                    ADC_DMA_MIN_SAMPLING_RATE, ADC_DMA_MAX_SAMPLING_RATE);
        return;
    }

    /* Update sampling rate (can be done while running) */
    if (ADC_DMA_SetSamplingRate(&adc_dma_handle, new_rate) != HAL_OK)
    {
        UART_SendString("ERROR: Failed to set sampling rate.\r\n");
        return;
    }

    current_sampling_rate = new_rate;

    /* Update FFT analyzer frequency parameters */
    fft_analyzer.config.fs = (float32_t)new_rate;
    fft_analyzer.freq_resolution = (float32_t)new_rate / (float32_t)current_fft_size;

    UART_Printf("Sampling rate set to %lu Hz (actual: %lu Hz)\r\n",
                new_rate, ADC_DMA_GetActualSamplingRate(&adc_dma_handle));
    UART_Printf("Nyquist: %lu Hz, Resolution: %.2f Hz\r\n",
                new_rate / 2, fft_analyzer.freq_resolution);
}

/**
 * @brief  Handle window type change command
 *
 * Supported types: RECT, HANN, HAMM, BLACK
 */
static void CMD_SetWindowType(const char *param)
{
    FFT_WindowType_t new_window;

    if (strcmp(param, "RECT") == 0)
    {
        new_window = FFT_WINDOW_RECTANGULAR;
    }
    else if (strcmp(param, "HANN") == 0)
    {
        new_window = FFT_WINDOW_HANNING;
    }
    else if (strcmp(param, "HAMM") == 0)
    {
        new_window = FFT_WINDOW_HAMMING;
    }
    else if (strcmp(param, "BLACK") == 0)
    {
        new_window = FFT_WINDOW_BLACKMAN;
    }
    else
    {
        UART_SendString("ERROR: Invalid window. Use RECT/HANN/HAMM/BLACK\r\n");
        return;
    }

    if (!FFT_Analyzer_SetWindowType(&fft_analyzer, new_window))
    {
        UART_SendString("ERROR: Failed to set window type.\r\n");
        return;
    }

    UART_Printf("Window set to %s\r\n", FFT_Analyzer_GetWindowName(new_window));
}

/**
 * @brief  Handle output mode change command
 *
 * Supported modes: SPEC, PEAK, RAW, STATS
 */
static void CMD_SetOutputMode(const char *param)
{
    if (strcmp(param, "SPEC") == 0)
    {
        output_mode = OUTPUT_MODE_SPECTRUM;
        UART_SendString("Output mode: SPECTRUM\r\n");
    }
    else if (strcmp(param, "PEAK") == 0)
    {
        output_mode = OUTPUT_MODE_PEAK;
        UART_SendString("Output mode: PEAK\r\n");
    }
    else if (strcmp(param, "RAW") == 0)
    {
        output_mode = OUTPUT_MODE_RAW;
        UART_SendString("Output mode: RAW\r\n");
    }
    else if (strcmp(param, "STATS") == 0)
    {
        output_mode = OUTPUT_MODE_STATS;
        UART_SendString("Output mode: STATS\r\n");
    }
    else
    {
        UART_SendString("ERROR: Invalid mode. Use SPEC/PEAK/RAW/STATS\r\n");
    }
}

/* ========================== LED Control ========================== */

/**
 * @brief  Set an LED to on or off state
 */
static void LED_SetState(uint16_t pin, bool state)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief  Toggle an LED state
 */
static void LED_Toggle(uint16_t pin)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, pin);
}

/* ========================== HAL Callbacks & IRQ Handlers ========================== */

/**
 * @brief  UART receive complete callback (interrupt-driven, byte-by-byte)
 *
 * Builds up a command string in uart_rx_buffer until CR or LF is received,
 * at which point uart_cmd_ready flag is set for main loop processing.
 * Echoes each character back for terminal feedback.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Echo the received character */
        HAL_UART_Transmit(&huart2, &uart_rx_byte, 1, 10);

        /* Check for end-of-line (command complete) */
        if (uart_rx_byte == '\r' || uart_rx_byte == '\n')
        {
            if (uart_rx_index > 0)
            {
                uart_rx_buffer[uart_rx_index] = '\0';
                uart_cmd_ready = true;
            }
            /* Send newline for terminal formatting */
            uint8_t nl[] = "\r\n";
            HAL_UART_Transmit(&huart2, nl, 2, 10);
        }
        /* Handle backspace */
        else if (uart_rx_byte == '\b' || uart_rx_byte == 0x7F)
        {
            if (uart_rx_index > 0)
            {
                uart_rx_index--;
                /* Erase character on terminal */
                uint8_t erase[] = " \b";
                HAL_UART_Transmit(&huart2, erase, 2, 10);
            }
        }
        /* Normal character: add to buffer */
        else if (uart_rx_index < UART_RX_BUFFER_SIZE - 1)
        {
            uart_rx_buffer[uart_rx_index++] = (char)uart_rx_byte;
        }

        /* Re-enable UART receive interrupt for next byte */
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/**
 * @brief  USART2 global interrupt handler
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief  SysTick interrupt handler (required by HAL for HAL_Delay)
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
