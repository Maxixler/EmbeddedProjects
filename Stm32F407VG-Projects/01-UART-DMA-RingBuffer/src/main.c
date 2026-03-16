/**
 * @file    main.c
 * @brief   UART DMA Ring Buffer demonstration application for STM32F407VG Discovery.
 *
 * @details This application demonstrates efficient UART communication using DMA
 *          and ring buffers. It provides:
 *          - AT command interface for controlling board peripherals
 *          - Echo mode for unrecognized commands
 *          - LED status indicators on the Discovery board LEDs (PD12-PD15)
 *          - Internal temperature sensor readout
 *          - Communication statistics and system information
 *
 *          Hardware:
 *          - MCU:    STM32F407VGT6 @ 168 MHz (HSE 8 MHz + PLL)
 *          - UART:   USART2 @ 115200 baud (PA2=TX, PA3=RX)
 *          - LEDs:   PD12 (Green), PD13 (Orange), PD14 (Red), PD15 (Blue)
 *
 *          AT Commands:
 *          - AT              -> OK
 *          - AT+LED=ON       -> LED ON  (Green LED PD12)
 *          - AT+LED=OFF      -> LED OFF (Green LED PD12)
 *          - AT+TEMP         -> TEMP=xx.x C
 *          - AT+INFO         -> System information
 *          - AT+STATS        -> Communication statistics
 *          - (other)         -> ECHO: <input>
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "stm32f4xx_hal.h"
#include "uart_dma.h"
#include "ring_buffer.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** @defgroup LED_Pins Discovery board LED pin definitions
 *  @{
 */
#define LED_GREEN_PIN GPIO_PIN_12  /**< PD12 - User controlled LED    */
#define LED_ORANGE_PIN GPIO_PIN_13 /**< PD13 - Data activity LED      */
#define LED_RED_PIN GPIO_PIN_14    /**< PD14 - Error indicator LED    */
#define LED_BLUE_PIN GPIO_PIN_15   /**< PD15 - Heartbeat LED          */
#define LED_GPIO_PORT GPIOD        /**< All LEDs on port D            */
#define LED_ALL_PINS (LED_GREEN_PIN | LED_ORANGE_PIN | LED_RED_PIN | LED_BLUE_PIN)
/** @} */

/** @defgroup Command_Defs Command parsing definitions
 *  @{
 */
#define CMD_BUFFER_SIZE 128U      /**< Maximum command length         */
#define RESPONSE_BUFFER_SIZE 256U /**< Maximum response length        */
#define CMD_TERMINATOR_CR '\r'    /**< Carriage return                */
#define CMD_TERMINATOR_LF '\n'    /**< Line feed                     */
/** @} */

/** @defgroup Timing_Defs Timing definitions
 *  @{
 */
#define HEARTBEAT_INTERVAL_MS 500U   /**< Heartbeat LED toggle interval */
#define ACTIVITY_LED_DURATION_MS 50U /**< Activity LED flash duration   */
/** @} */

/** @defgroup Temp_Sensor Internal temperature sensor definitions
 *  @{
 */
#define TEMP_SENSOR_AVG_SLOPE 2.5f  /**< mV per degree C (typical)     */
#define TEMP_SENSOR_V25 0.76f       /**< Voltage at 25C (typical, V)   */
#define TEMP_SENSOR_VREF 3.3f       /**< Reference voltage (V)         */
#define TEMP_SENSOR_ADC_MAX 4095.0f /**< 12-bit ADC max value          */
/** @} */

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

/** @brief  UART DMA driver handle (global instance). */
static uart_dma_handle_t g_uart_handle;

/** @brief  ADC handle for internal temperature sensor. */
static ADC_HandleTypeDef g_hadc1;

/** @brief  Command receive buffer (accumulates bytes until terminator). */
static char g_cmd_buffer[CMD_BUFFER_SIZE];

/** @brief  Current position in the command buffer. */
static size_t g_cmd_index = 0U;

/** @brief  Response formatting buffer. */
static char g_response[RESPONSE_BUFFER_SIZE];

/** @brief  System uptime counter (incremented by SysTick). */
static volatile uint32_t g_uptime_seconds = 0U;

/** @brief  Timestamp of last heartbeat toggle. */
static uint32_t g_last_heartbeat_ms = 0U;

/** @brief  Timestamp of last activity LED activation. */
static uint32_t g_activity_led_off_time_ms = 0U;

/** @brief  Flag indicating activity LED is currently on. */
static bool g_activity_led_on = false;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void SystemClock_Config(void);
static void GPIO_LED_Init(void);
static void ADC_TempSensor_Init(void);
static float ADC_ReadTemperature(void);
static void process_command(const char *cmd);
static void send_welcome_message(void);
static void send_response(const char *resp);
static void update_heartbeat_led(void);
static void update_activity_led(void);
static void flash_activity_led(void);
static void str_to_upper(char *str);
static void Error_Handler(void);

/* -------------------------------------------------------------------------- */
/*                              Main Function                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Application entry point.
 *
 * @details Initializes all peripherals and enters the main loop.
 *          The main loop:
 *          1. Checks for received UART data via the RX ring buffer
 *          2. Accumulates characters until a command terminator is found
 *          3. Processes complete commands through the AT command parser
 *          4. Manages LED status indicators
 */
int main(void)
{
    /* ---- HAL and system initialization ---- */
    HAL_Init();
    SystemClock_Config();

    /* ---- Peripheral initialization ---- */
    GPIO_LED_Init();
    ADC_TempSensor_Init();

    /* ---- Turn on blue LED to indicate system is alive ---- */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_SET);

    /* ---- Initialize UART DMA driver with default configuration ---- */
    uart_dma_config_t uart_config = {
        .baud_rate = 115200U,
        .word_length = UART_WORDLENGTH_8B,
        .stop_bits = UART_STOPBITS_1,
        .parity = UART_PARITY_NONE,
        .flow_control = UART_HWCONTROL_NONE,
    };

    if (uart_dma_init(&g_uart_handle, &uart_config) != UART_DMA_OK)
    {
        /* Initialization failed - turn on red LED and halt. */
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);
        Error_Handler();
    }

    /* ---- Send welcome banner ---- */
    send_welcome_message();

    /* ---- Initialize timing variables ---- */
    g_last_heartbeat_ms = HAL_GetTick();

    /* ===================================================================== */
    /*                            MAIN LOOP                                  */
    /* ===================================================================== */
    while (1)
    {
        uint8_t byte;

        /*
         * Process all available bytes from the RX ring buffer.
         * The DMA+IDLE mechanism fills the ring buffer in the background.
         * Here we just read from it at our own pace.
         */
        while (uart_dma_receive_byte(&g_uart_handle, &byte) == UART_DMA_OK)
        {
            /* Flash the activity LED on data reception. */
            flash_activity_led();

            /*
             * Command accumulation:
             * Collect characters until we see CR or LF, which marks
             * the end of a command. Ignore standalone LF after CR
             * (handles both CR, LF, and CR+LF line endings).
             */
            if (byte == CMD_TERMINATOR_CR || byte == CMD_TERMINATOR_LF)
            {
                if (g_cmd_index > 0U)
                {
                    /* Null-terminate the command string. */
                    g_cmd_buffer[g_cmd_index] = '\0';

                    /* Process the complete command. */
                    process_command(g_cmd_buffer);

                    /* Reset the command buffer for the next command. */
                    g_cmd_index = 0U;
                }
                /* If cmd_index is 0, this is a standalone LF after CR - ignore. */
            }
            else
            {
                /* Accumulate the character if there is room. */
                if (g_cmd_index < (CMD_BUFFER_SIZE - 1U))
                {
                    g_cmd_buffer[g_cmd_index] = (char)byte;
                    g_cmd_index++;
                }
                else
                {
                    /*
                     * Command buffer overflow - discard and reset.
                     * This prevents processing a truncated command.
                     */
                    g_cmd_index = 0U;
                    send_response("ERROR: Command too long\r\n");
                }
            }
        }

        /* ---- Check for UART errors ---- */
        uint32_t errors = uart_dma_get_and_clear_errors(&g_uart_handle);
        if (errors != UART_DMA_ERR_NONE)
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);

            /* Briefly turn on red LED for errors - will be turned off by heartbeat. */
            if (errors & UART_DMA_ERR_OVERRUN)
            {
                send_response("ERROR: Overrun detected\r\n");
            }
            if (errors & UART_DMA_ERR_FRAMING)
            {
                send_response("ERROR: Framing error\r\n");
            }
        }

        /* ---- Update LED states ---- */
        update_heartbeat_led();
        update_activity_led();
    }
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Configure the system clock to 168 MHz using HSE and PLL.
 *
 * @details Clock tree configuration:
 *          - HSE = 8 MHz (on-board crystal)
 *          - PLL input = HSE / PLLM(8) = 1 MHz
 *          - VCO = PLL input * PLLN(336) = 336 MHz
 *          - SYSCLK = VCO / PLLP(2) = 168 MHz
 *          - AHB = SYSCLK / 1 = 168 MHz
 *          - APB1 = AHB / 4 = 42 MHz (max for APB1)
 *          - APB2 = AHB / 2 = 84 MHz (max for APB2)
 *
 *          APB1 timer clocks = APB1 * 2 = 84 MHz (prescaler != 1)
 *          APB2 timer clocks = APB2 * 2 = 168 MHz (prescaler != 1)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    /* ---- Enable Power Control clock and set voltage scaling ---- */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* ---- Configure HSE and PLL ---- */
    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_ON;
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc_init.PLL.PLLM = 8U;            /* VCO input = 8 MHz / 8 = 1 MHz    */
    osc_init.PLL.PLLN = 336U;          /* VCO output = 1 MHz * 336 = 336 MHz */
    osc_init.PLL.PLLP = RCC_PLLP_DIV2; /* SYSCLK = 336 / 2 = 168 MHz */
    osc_init.PLL.PLLQ = 7U;            /* USB clock = 336 / 7 = 48 MHz     */

    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---- Configure clock dividers ---- */
    clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1; /* HCLK = 168 MHz       */
    clk_init.APB1CLKDivider = RCC_HCLK_DIV4;  /* PCLK1 = 42 MHz       */
    clk_init.APB2CLKDivider = RCC_HCLK_DIV2;  /* PCLK2 = 84 MHz       */

    /*
     * Set flash latency to 5 wait states for 168 MHz operation
     * at 3.3V supply (per datasheet Table 10).
     */
    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---- Update SystemCoreClock variable ---- */
    SystemCoreClockUpdate();
}

/**
 * @brief   Initialize GPIO pins for the Discovery board LEDs (PD12-PD15).
 */
static void GPIO_LED_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_ALL_PINS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_GPIO_PORT, &gpio);

    /* Start with all LEDs off. */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ALL_PINS, GPIO_PIN_RESET);
}

/**
 * @brief   Initialize ADC1 for internal temperature sensor reading.
 *
 * @details The internal temperature sensor is connected to ADC1 Channel 18
 *          on STM32F407. It requires a minimum sampling time of 10 us.
 *          At 84 MHz APB2 (ADC clock = APB2/4 = 21 MHz), we use 480 cycles
 *          sampling time: 480 / 21 MHz = 22.9 us (sufficient).
 */
static void ADC_TempSensor_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    g_hadc1.Instance = ADC1;
    g_hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    g_hadc1.Init.ScanConvMode = DISABLE;
    g_hadc1.Init.ContinuousConvMode = DISABLE;
    g_hadc1.Init.DiscontinuousConvMode = DISABLE;
    g_hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_hadc1.Init.NbrOfConversion = 1U;

    if (HAL_ADC_Init(&g_hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief   Read the internal temperature sensor value.
 *
 * @details Configures ADC1 to read the internal temperature sensor channel,
 *          performs a single conversion, and calculates the temperature in
 *          degrees Celsius using the formula from the reference manual:
 *
 *          Temperature (C) = ((V_SENSE - V_25) / Avg_Slope) + 25
 *
 *          Where:
 *          - V_SENSE = ADC_value * V_REF / 4095
 *          - V_25 = 0.76 V (voltage at 25 C, typical)
 *          - Avg_Slope = 2.5 mV/C (typical)
 *
 * @return  Temperature in degrees Celsius.
 */
static float ADC_ReadTemperature(void)
{
    ADC_ChannelConfTypeDef ch_config = {0};

    /* Configure the temperature sensor channel. */
    ch_config.Channel = ADC_CHANNEL_TEMPSENSOR;
    ch_config.Rank = 1U;
    ch_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;

    if (HAL_ADC_ConfigChannel(&g_hadc1, &ch_config) != HAL_OK)
    {
        return -999.0f; /* Error indicator. */
    }

    /* Start conversion. */
    HAL_ADC_Start(&g_hadc1);

    /* Wait for conversion to complete (timeout 100 ms). */
    if (HAL_ADC_PollForConversion(&g_hadc1, 100U) != HAL_OK)
    {
        HAL_ADC_Stop(&g_hadc1);
        return -999.0f;
    }

    /* Read the ADC value. */
    uint32_t adc_value = HAL_ADC_GetValue(&g_hadc1);
    HAL_ADC_Stop(&g_hadc1);

    /* Convert ADC reading to temperature. */
    float v_sense = ((float)adc_value * TEMP_SENSOR_VREF) / TEMP_SENSOR_ADC_MAX;
    float temperature = ((v_sense - TEMP_SENSOR_V25) / (TEMP_SENSOR_AVG_SLOPE / 1000.0f)) + 25.0f;

    return temperature;
}

/**
 * @brief   Process a complete AT command.
 *
 * @details Parses the command string and executes the corresponding action.
 *          Supported commands:
 *          - "AT"         : Connection test, responds "OK"
 *          - "AT+LED=ON"  : Turn on green LED (PD12), responds "LED ON"
 *          - "AT+LED=OFF" : Turn off green LED (PD12), responds "LED OFF"
 *          - "AT+TEMP"    : Read internal temperature, responds "TEMP=xx.x C"
 *          - "AT+INFO"    : System information
 *          - "AT+STATS"   : Communication statistics
 *          - Other        : Echo back the input
 *
 * @param[in]   cmd     Null-terminated command string (without terminator).
 */
static void process_command(const char *cmd)
{
    /* Make a mutable uppercase copy for case-insensitive comparison. */
    char cmd_upper[CMD_BUFFER_SIZE];
    strncpy(cmd_upper, cmd, CMD_BUFFER_SIZE - 1U);
    cmd_upper[CMD_BUFFER_SIZE - 1U] = '\0';
    str_to_upper(cmd_upper);

    /* ---- AT: Basic connection test ---- */
    if (strcmp(cmd_upper, "AT") == 0)
    {
        send_response("OK\r\n");
    }
    /* ---- AT+LED=ON: Turn on green LED ---- */
    else if (strcmp(cmd_upper, "AT+LED=ON") == 0)
    {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
        send_response("LED ON\r\n");
    }
    /* ---- AT+LED=OFF: Turn off green LED ---- */
    else if (strcmp(cmd_upper, "AT+LED=OFF") == 0)
    {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
        send_response("LED OFF\r\n");
    }
    /* ---- AT+TEMP: Read internal temperature sensor ---- */
    else if (strcmp(cmd_upper, "AT+TEMP") == 0)
    {
        float temp = ADC_ReadTemperature();
        int temp_int = (int)temp;
        int temp_frac = (int)((temp - (float)temp_int) * 10.0f);
        if (temp_frac < 0)
        {
            temp_frac = -temp_frac;
        }
        snprintf(g_response, RESPONSE_BUFFER_SIZE,
                 "TEMP=%d.%d C\r\n", temp_int, temp_frac);
        send_response(g_response);
    }
    /* ---- AT+INFO: System information ---- */
    else if (strcmp(cmd_upper, "AT+INFO") == 0)
    {
        uint32_t uptime_s = HAL_GetTick() / 1000U;
        uint32_t uptime_m = uptime_s / 60U;
        uint32_t uptime_h = uptime_m / 60U;

        size_t rx_used = ring_buffer_available_data(&g_uart_handle.rx_ring);
        size_t rx_free = ring_buffer_available_space(&g_uart_handle.rx_ring);
        size_t tx_used = ring_buffer_available_data(&g_uart_handle.tx_ring);
        size_t tx_free = ring_buffer_available_space(&g_uart_handle.tx_ring);

        snprintf(g_response, RESPONSE_BUFFER_SIZE,
                 "=== System Info ===\r\n"
                 "MCU: STM32F407VG\r\n"
                 "Clock: %lu MHz\r\n"
                 "Uptime: %luh %lum %lus\r\n"
                 "RX Buffer: %u/%u bytes used\r\n"
                 "TX Buffer: %u/%u bytes used\r\n"
                 "===================\r\n",
                 (unsigned long)(SystemCoreClock / 1000000U),
                 (unsigned long)uptime_h,
                 (unsigned long)(uptime_m % 60U),
                 (unsigned long)(uptime_s % 60U),
                 (unsigned int)rx_used,
                 (unsigned int)(rx_used + rx_free),
                 (unsigned int)tx_used,
                 (unsigned int)(tx_used + tx_free));
        send_response(g_response);
    }
    /* ---- AT+STATS: Communication statistics ---- */
    else if (strcmp(cmd_upper, "AT+STATS") == 0)
    {
        uart_dma_stats_t stats;
        uart_dma_get_stats(&g_uart_handle, &stats);

        snprintf(g_response, RESPONSE_BUFFER_SIZE,
                 "=== UART Stats ===\r\n"
                 "TX Bytes: %lu\r\n"
                 "RX Bytes: %lu\r\n"
                 "TX DMA Transfers: %lu\r\n"
                 "RX Events: %lu\r\n"
                 "Errors: %lu\r\n"
                 "Overruns: %lu\r\n"
                 "Buffer Overflows: %lu\r\n"
                 "==================\r\n",
                 (unsigned long)stats.tx_bytes,
                 (unsigned long)stats.rx_bytes,
                 (unsigned long)stats.tx_dma_transfers,
                 (unsigned long)stats.rx_events,
                 (unsigned long)stats.error_count,
                 (unsigned long)stats.overrun_count,
                 (unsigned long)stats.buffer_overflow);
        send_response(g_response);
    }
    /* ---- Unknown command: echo back ---- */
    else
    {
        snprintf(g_response, RESPONSE_BUFFER_SIZE, "ECHO: %s\r\n", cmd);
        send_response(g_response);
    }

    /* Print prompt for next command. */
    send_response("> ");
}

/**
 * @brief   Send the welcome banner message.
 */
static void send_welcome_message(void)
{
    const char *banner =
        "\r\n"
        "========================================\r\n"
        "  UART DMA Ring Buffer Demo\r\n"
        "  STM32F407VG Discovery Board\r\n"
        "  Baud: 115200, Clock: 168 MHz\r\n"
        "========================================\r\n"
        "System ready. Type 'AT' to test.\r\n"
        "> ";

    uart_dma_send_string(&g_uart_handle, banner);
}

/**
 * @brief   Send a response string via UART.
 *
 * @param[in]   resp    Null-terminated response string.
 */
static void send_response(const char *resp)
{
    uart_dma_send_string(&g_uart_handle, resp);
}

/**
 * @brief   Update the heartbeat LED (PD15 Blue).
 *
 * @details Toggles the blue LED at a fixed interval to indicate the system
 *          is alive and the main loop is running. Also clears the error LED
 *          after some time.
 */
static void update_heartbeat_led(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - g_last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS)
    {
        g_last_heartbeat_ms = now;
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_BLUE_PIN);

        /*
         * Clear the red error LED on each heartbeat toggle.
         * This makes the error LED flash briefly on errors rather
         * than staying permanently on.
         */
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    }
}

/**
 * @brief   Update the data activity LED (PD13 Orange).
 *
 * @details Turns off the activity LED after its display duration has elapsed.
 */
static void update_activity_led(void)
{
    if (g_activity_led_on)
    {
        uint32_t now = HAL_GetTick();
        if (now >= g_activity_led_off_time_ms)
        {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
            g_activity_led_on = false;
        }
    }
}

/**
 * @brief   Flash the activity LED briefly.
 *
 * @details Turns on the orange LED and schedules it to turn off after
 *          ACTIVITY_LED_DURATION_MS milliseconds.
 */
static void flash_activity_led(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
    g_activity_led_off_time_ms = HAL_GetTick() + ACTIVITY_LED_DURATION_MS;
    g_activity_led_on = true;
}

/**
 * @brief   Convert a string to uppercase in-place.
 *
 * @param[in,out]   str     Null-terminated string to convert.
 */
static void str_to_upper(char *str)
{
    if (str == NULL)
    {
        return;
    }

    while (*str != '\0')
    {
        if (*str >= 'a' && *str <= 'z')
        {
            *str = *str - ('a' - 'A');
        }
        str++;
    }
}

/**
 * @brief   Error handler - enters infinite loop with red LED on.
 *
 * @details Called when an unrecoverable error occurs during initialization.
 *          In a production system, this would trigger a watchdog reset.
 */
static void Error_Handler(void)
{
    /* Disable interrupts to prevent further processing. */
    __disable_irq();

    /* Turn on red LED to indicate error. */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);

    /* Infinite loop - system is halted. */
    while (1)
    {
        /* In production, a watchdog timeout would reset the MCU here. */
    }
}

/* -------------------------------------------------------------------------- */
/*                        HAL Required Callbacks                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief   SysTick interrupt handler.
 *
 * @details Increments the HAL tick counter (1 ms resolution).
 *          Required by HAL_GetTick() and HAL_Delay().
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief   Non-maskable interrupt handler.
 */
void NMI_Handler(void)
{
    /* Nothing to do. */
}

/**
 * @brief   Hard fault handler.
 */
void HardFault_Handler(void)
{
    /* Turn on all LEDs to indicate hard fault. */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ALL_PINS, GPIO_PIN_SET);

    while (1)
    {
        /* Halt. */
    }
}

/**
 * @brief   Memory management fault handler.
 */
void MemManage_Handler(void)
{
    while (1)
    {
        /* Halt. */
    }
}

/**
 * @brief   Bus fault handler.
 */
void BusFault_Handler(void)
{
    while (1)
    {
        /* Halt. */
    }
}

/**
 * @brief   Usage fault handler.
 */
void UsageFault_Handler(void)
{
    while (1)
    {
        /* Halt. */
    }
}
