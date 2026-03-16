/*******************************************************************************
 * @file    main.c
 * @brief   FreeRTOS Task Scheduler - Main Entry Point for STM32F407VG
 * @details Initializes all hardware peripherals (system clock at 168 MHz,
 *          GPIO for LEDs, ADC1 with DMA, UART2, TIM5 for runtime stats),
 *          creates all FreeRTOS objects, and starts the scheduler.
 *
 *          Hardware Configuration:
 *          - System Clock: 168 MHz (HSE 8 MHz -> PLL)
 *          - AHB: 168 MHz, APB1: 42 MHz, APB2: 84 MHz
 *          - PD12-PD15: Output LEDs (Green, Orange, Red, Blue)
 *          - PA0 (ADC1_CH0): Temperature sensor input
 *          - PA1 (ADC1_CH1): Light sensor input
 *          - PA2 (USART2_TX), PA3 (USART2_RX): Debug UART @ 115200
 *          - TIM5: 32-bit free-running counter at 10 kHz for runtime stats
 *          - DMA2 Stream 0: ADC1 DMA transfer
 *
 *          FreeRTOS Hook Functions:
 *          - vApplicationIdleHook: WFI for low power, idle counter
 *          - vApplicationStackOverflowHook: Error LED, halt
 *          - vApplicationMallocFailedHook: Error LED, halt
 *
 * @author  Embedded Systems Portfolio Project
 ******************************************************************************/

/*===========================================================================*/
/*                        INCLUDE FILES                                      */
/*===========================================================================*/

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include <stdio.h>
#include <string.h>

/*===========================================================================*/
/*                    PERIPHERAL HANDLE DEFINITIONS                          */
/*===========================================================================*/

ADC_HandleTypeDef hadc1;     /**< ADC1 handle for sensor channels        */
DMA_HandleTypeDef hdma_adc1; /**< DMA handle for ADC1 transfers          */
UART_HandleTypeDef huart2;   /**< USART2 handle for debug output         */
TIM_HandleTypeDef htim5;     /**< TIM5 handle for runtime stats counter  */

/*===========================================================================*/
/*                    PRIVATE FUNCTION PROTOTYPES                            */
/*===========================================================================*/

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void Error_Handler(void);

/*===========================================================================*/
/*                    RUNTIME STATS TIMER FUNCTIONS                          */
/*===========================================================================*/

/**
 * @brief Configure TIM5 as the runtime statistics timer.
 *
 * This function is called by the FreeRTOS kernel via the
 * portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() macro during scheduler startup.
 * TIM5 is a 32-bit timer configured to count at 10 kHz (100 us resolution),
 * which is 10x the RTOS tick rate for adequate measurement granularity.
 */
void vConfigureTimerForRunTimeStats(void)
{
    /* TIM5 is already initialized in MX_TIM5_Init() and started below.
     * Reset the counter to zero when stats collection begins. */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}

/**
 * @brief Read the runtime statistics counter value.
 *
 * Called by the FreeRTOS kernel via the portGET_RUN_TIME_COUNTER_VALUE()
 * macro on every context switch to measure task execution time.
 *
 * @return Current TIM5 counter value (32-bit, increments every 100 us)
 */
uint32_t ulGetRunTimeCounterValue(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

/*===========================================================================*/
/*                    MAIN FUNCTION                                          */
/*===========================================================================*/

/**
 * @brief Application entry point.
 *
 * Initialization sequence:
 * 1. HAL initialization (SysTick, Flash latency)
 * 2. System clock configuration (168 MHz)
 * 3. Peripheral initialization (GPIO, DMA, ADC, UART, TIM)
 * 4. Startup banner via UART
 * 5. FreeRTOS object creation (tasks, queues, semaphores, etc.)
 * 6. Start the FreeRTOS scheduler (never returns on success)
 *
 * @retval int  Never returns (scheduler takes over)
 */
int main(void)
{
    /*-----------------------------------------------------------------------*/
    /* Step 1: HAL Initialization                                            */
    /*-----------------------------------------------------------------------*/

    /* Reset all peripherals, initialize Flash interface and SysTick.
     * HAL_Init() sets SysTick to 1ms interrupt, which will be reconfigured
     * by FreeRTOS when the scheduler starts. */
    HAL_Init();

    /*-----------------------------------------------------------------------*/
    /* Step 2: System Clock Configuration                                    */
    /*-----------------------------------------------------------------------*/

    /* Configure the system clock to 168 MHz using HSE and PLL.
     * This also configures AHB, APB1, and APB2 bus clocks. */
    SystemClock_Config();

    /*-----------------------------------------------------------------------*/
    /* Step 3: Peripheral Initialization                                     */
    /*-----------------------------------------------------------------------*/

    /* IMPORTANT: DMA must be initialized BEFORE ADC, as ADC DMA mode
     * depends on the DMA controller being ready. */
    MX_GPIO_Init();        /* LED outputs (PD12-PD15)                      */
    MX_DMA_Init();         /* DMA2 Stream 0 for ADC1                       */
    MX_ADC1_Init();        /* ADC1 Ch0 + Ch1, scan mode, DMA              */
    MX_USART2_UART_Init(); /* USART2 @ 115200 baud                        */
    MX_TIM5_Init();        /* TIM5 @ 10 kHz for runtime stats             */

    /*-----------------------------------------------------------------------*/
    /* Step 4: Start the runtime statistics timer                            */
    /*-----------------------------------------------------------------------*/

    /* Start TIM5 before the scheduler so it's counting from the beginning.
     * The counter runs continuously (free-running, 32-bit). */
    HAL_TIM_Base_Start(&htim5);

    /*-----------------------------------------------------------------------*/
    /* Step 5: Startup Banner                                                */
    /*-----------------------------------------------------------------------*/

    /* Print a startup message to UART before the scheduler takes over.
     * After vTaskStartScheduler(), all UART access must go through the mutex. */
    const char *pcBanner =
        "\r\n"
        "======================================================\r\n"
        "  STM32F407VG FreeRTOS Task Scheduler Demo\r\n"
        "  System Clock: 168 MHz | Tick Rate: 1000 Hz\r\n"
        "  UART: 115200 8N1\r\n"
        "======================================================\r\n"
        "Initializing FreeRTOS objects...\r\n";

    HAL_UART_Transmit(&huart2, (uint8_t *)pcBanner,
                      (uint16_t)strlen(pcBanner), HAL_MAX_DELAY);

    /*-----------------------------------------------------------------------*/
    /* Step 6: Create All FreeRTOS Objects                                   */
    /*-----------------------------------------------------------------------*/

    /* Create all tasks, queues, semaphores, mutexes, event groups,
     * and software timers. This allocates memory from the FreeRTOS heap. */
    if (xAppTasksInit() != pdPASS)
    {
        /* Fatal: insufficient heap memory for FreeRTOS objects */
        const char *pcError = "[FATAL] Failed to create FreeRTOS objects!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)pcError,
                          (uint16_t)strlen(pcError), HAL_MAX_DELAY);
        Error_Handler();
    }

    const char *pcReady = "All FreeRTOS objects created. Starting scheduler...\r\n\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)pcReady,
                      (uint16_t)strlen(pcReady), HAL_MAX_DELAY);

    /*-----------------------------------------------------------------------*/
    /* Step 7: Turn on system status LED briefly to indicate ready           */
    /*-----------------------------------------------------------------------*/

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); /* Blue LED on */

    /*-----------------------------------------------------------------------*/
    /* Step 8: Start the FreeRTOS Scheduler                                  */
    /*-----------------------------------------------------------------------*/

    /* This call never returns if everything is configured correctly.
     * The scheduler takes control of the CPU and begins running tasks.
     * From this point, SysTick is managed by FreeRTOS (not HAL).
     *
     * If vTaskStartScheduler() returns, it means there was not enough
     * heap memory to create the idle task or timer task. */
    vTaskStartScheduler();

    /*-----------------------------------------------------------------------*/
    /* ERROR: We should never reach here                                     */
    /*-----------------------------------------------------------------------*/

    /* If the scheduler returns, it's a fatal error (insufficient heap) */
    const char *pcFatal = "[FATAL] Scheduler returned! Insufficient heap.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)pcFatal,
                      (uint16_t)strlen(pcFatal), HAL_MAX_DELAY);

    for (;;)
    {
        /* Trap: blink all LEDs rapidly to indicate fatal error */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 |
                                      GPIO_PIN_14 | GPIO_PIN_15);
        HAL_Delay(100);
    }
}

/*===========================================================================*/
/*                    FREERTOS APPLICATION HOOK FUNCTIONS                     */
/*===========================================================================*/

/**
 * @brief Idle hook function - called on each idle task iteration.
 *
 * The idle task is created automatically by the scheduler at priority 0.
 * This hook runs whenever no application task is ready to execute.
 *
 * Uses:
 * 1. Enter low-power mode (WFI) to reduce power consumption
 * 2. Increment idle counter for rough CPU utilization estimation
 *
 * CRITICAL RULES:
 * - Must NOT call any blocking FreeRTOS API (vTaskDelay, xQueueReceive, etc.)
 * - Must NOT call vTaskSuspend() on itself
 * - Should be as short as possible
 */
void vApplicationIdleHook(void)
{
    /* Increment idle counter for CPU utilization estimation.
     * The monitor task reads and resets this periodically. */
    ulIdleCounter++;

    /* Enter sleep mode until the next interrupt.
     * WFI (Wait For Interrupt) halts the CPU core while keeping
     * peripherals and interrupts active. The CPU wakes on any interrupt
     * (SysTick, DMA, UART, etc.), consuming minimal power while idle.
     *
     * Power savings: ~30-50% reduction in active mode power consumption. */
    __WFI();
}

/**
 * @brief Stack overflow hook - called when a task exceeds its stack.
 *
 * This is a fatal error condition. The task's stack has been corrupted,
 * and the system is in an unstable state. We halt the system and indicate
 * the error visually.
 *
 * Method 2 detection: FreeRTOS fills the stack with 0xA5A5A5A5 pattern
 * and checks the last 20 bytes at each context switch. If the pattern
 * is corrupted, this hook is called.
 *
 * @param xTask      Handle of the task that overflowed
 * @param pcTaskName Name string of the offending task
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; /* Suppress unused parameter warning */

    /* Disable all interrupts - system is unstable */
    taskDISABLE_INTERRUPTS();

    /* Attempt to send an error message via UART (no mutex - we're in crisis).
     * This may or may not work depending on system state. */
    char pcErrorMsg[80];
    int iLen = snprintf(pcErrorMsg, sizeof(pcErrorMsg),
                        "\r\n[FATAL] Stack overflow in task: %s\r\n", pcTaskName);
    HAL_UART_Transmit(&huart2, (uint8_t *)pcErrorMsg,
                      (uint16_t)iLen, 1000);

    /* Turn on red LED to indicate fatal error */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

    /* Turn off all other LEDs */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15,
                      GPIO_PIN_RESET);

    /* Halt: infinite loop. Debugger can inspect pcTaskName to identify
     * which task caused the overflow. Increase that task's stack size. */
    for (;;)
    {
        /* Toggle red LED to make the error visible */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        /* Simple delay without HAL_Delay (SysTick may not be working) */
        for (volatile uint32_t i = 0; i < 1000000; i++)
            ;
    }
}

/**
 * @brief Malloc failed hook - called when pvPortMalloc() returns NULL.
 *
 * This indicates the FreeRTOS heap is exhausted. No more tasks, queues,
 * semaphores, or other dynamically allocated objects can be created.
 *
 * Common causes:
 * - configTOTAL_HEAP_SIZE is too small
 * - Memory leak (objects created but never deleted)
 * - Too many tasks or too large stack sizes
 *
 * Resolution: Increase configTOTAL_HEAP_SIZE or reduce task stack sizes.
 */
void vApplicationMallocFailedHook(void)
{
    /* Disable all interrupts */
    taskDISABLE_INTERRUPTS();

    /* Send error message */
    const char *pcError = "\r\n[FATAL] FreeRTOS malloc failed! Heap exhausted.\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)pcError,
                      (uint16_t)strlen(pcError), 1000);

    /* Turn on red and orange LEDs to indicate memory error */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

    /* Halt in infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 500000; i++)
            ;
    }
}

/*===========================================================================*/
/*                    SYSTEM CLOCK CONFIGURATION                             */
/*===========================================================================*/

/**
 * @brief Configure the system clock to 168 MHz.
 *
 * Clock tree:
 *   HSE (8 MHz external crystal)
 *   -> PLL: PLLM=8, PLLN=336, PLLP=2, PLLQ=7
 *   -> SYSCLK = 168 MHz
 *   -> AHB = 168 MHz (prescaler = 1)
 *   -> APB1 = 42 MHz (prescaler = 4, max 42 MHz for APB1)
 *   -> APB2 = 84 MHz (prescaler = 2, max 84 MHz for APB2)
 *
 * Flash configuration:
 *   5 wait states for 168 MHz (per datasheet Table 10)
 *   Instruction cache, data cache, and prefetch enabled
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage.
     * Scale 1 is required for 168 MHz operation. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure HSE and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;             /* VCO input = 8 MHz / 8 = 1 MHz  */
    RCC_OscInitStruct.PLL.PLLN = 336;           /* VCO output = 1 MHz * 336 = 336 MHz */
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; /* SYSCLK = 336 / 2 = 168 MHz */
    RCC_OscInitStruct.PLL.PLLQ = 7;             /* USB OTG FS = 336 / 7 = 48 MHz  */

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure bus clocks: AHB, APB1, APB2 */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* 168 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  /* 42 MHz  */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  /* 84 MHz  */

    /* 5 wait states required for 168 MHz at 3.3V (refer to datasheet) */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/*===========================================================================*/
/*                    GPIO INITIALIZATION                                    */
/*===========================================================================*/

/**
 * @brief Initialize GPIO pins for LEDs.
 *
 * Configures PD12-PD15 as push-pull outputs for the four on-board LEDs:
 *   PD12 = Green  (Heartbeat LED, toggled by software timer)
 *   PD13 = Orange (Temperature alarm indicator)
 *   PD14 = Red    (Light alarm / error indicator)
 *   PD15 = Blue   (System status indicator)
 *
 * All LEDs start in the OFF state.
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO port D clock */
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Enable GPIO port A clock (needed for ADC and UART pins) */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Set all LED pins to LOW (LEDs off) before configuring as output */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

    /* Configure PD12-PD15 as push-pull outputs, no pull-up/pull-down */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; /* Low speed is fine for LEDs */
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/*===========================================================================*/
/*                    DMA INITIALIZATION                                     */
/*===========================================================================*/

/**
 * @brief Initialize DMA2 for ADC1 data transfer.
 *
 * DMA2 Stream 0, Channel 0 is used for ADC1.
 * Configured for:
 * - Peripheral to Memory transfer direction
 * - Circular mode (auto-restart after transfer complete)
 * - No peripheral address increment (ADC DR is fixed)
 * - Memory address increment (store each channel to successive buffer locations)
 * - Half-word (16-bit) data width for both peripheral and memory
 *
 * IMPORTANT: DMA initialization must be called BEFORE ADC initialization.
 */
static void MX_DMA_Init(void)
{
    /* Enable DMA2 clock */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA2 Stream 0 for ADC1 */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Link DMA handle to ADC handle */
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* Configure DMA2 Stream 0 interrupt priority.
     * Must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5)
     * to safely call FreeRTOS "FromISR" functions in the callback. */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/*===========================================================================*/
/*                    ADC INITIALIZATION                                     */
/*===========================================================================*/

/**
 * @brief Initialize ADC1 for dual-channel sensor reading.
 *
 * ADC1 Configuration:
 * - 12-bit resolution (0-4095 counts, 0-3.3V)
 * - Scan mode enabled (multiple channels in sequence)
 * - Continuous conversion disabled (triggered by software)
 * - DMA continuous request enabled (works with circular DMA)
 * - End of conversion flag after full sequence
 *
 * Channel Configuration:
 * - Rank 1: Channel 0 (PA0) - Temperature sensor
 * - Rank 2: Channel 1 (PA1) - Light sensor
 * - Sample time: 84 cycles (adequate for most sensors)
 */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Enable ADC1 clock (on APB2 bus) */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* ADC1 global configuration */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; /* 84/4 = 21 MHz */
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;        /* Scan multiple channels        */
    hadc1.Init.ContinuousConvMode = DISABLE; /* Single conversion per trigger */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_NUM_CHANNELS; /* 2 channels */
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV; /* EOC after sequence */

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure ADC channel 0 (PA0) - Temperature sensor, Rank 1 */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure ADC channel 1 (PA1) - Light sensor, Rank 2 */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 2;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure PA0 and PA1 as analog inputs */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*===========================================================================*/
/*                    USART2 INITIALIZATION                                  */
/*===========================================================================*/

/**
 * @brief Initialize USART2 for debug/display output.
 *
 * USART2 is connected to the ST-Link Virtual COM Port on the Discovery board.
 * Configuration: 115200 baud, 8 data bits, no parity, 1 stop bit, no flow control.
 *
 * Pin mapping:
 *   PA2 = USART2_TX (Alternate Function 7)
 *   PA3 = USART2_RX (Alternate Function 7)
 */
static void MX_USART2_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable USART2 clock (on APB1 bus, 42 MHz) */
    __HAL_RCC_USART2_CLK_ENABLE();

    /* Configure PA2 (TX) and PA3 (RX) as alternate function */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 configuration */
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
        Error_Handler();
    }
}

/*===========================================================================*/
/*                    TIM5 INITIALIZATION (RUNTIME STATS)                    */
/*===========================================================================*/

/**
 * @brief Initialize TIM5 as a free-running counter for runtime statistics.
 *
 * TIM5 is a 32-bit timer clocked from APB1 (42 MHz, timer clock = 84 MHz
 * due to APB1 prescaler > 1).
 *
 * Configuration:
 * - Prescaler: 8399 -> Counter clock = 84 MHz / 8400 = 10 kHz
 * - Period: 0xFFFFFFFF (maximum, 32-bit free-running)
 * - Resolution: 100 microseconds per count
 * - Overflow: ~119.3 hours at 10 kHz (sufficient for any session)
 *
 * The 10 kHz rate is 10x the RTOS tick rate (1 kHz), providing adequate
 * resolution for CPU usage measurement as recommended by FreeRTOS docs.
 */
static void MX_TIM5_Init(void)
{
    /* Enable TIM5 clock (APB1 timer) */
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 8400 - 1; /* 84 MHz / 8400 = 10 kHz      */
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF; /* 32-bit max, free-running     */
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
}

/*===========================================================================*/
/*                    INTERRUPT HANDLERS                                      */
/*===========================================================================*/

/**
 * @brief DMA2 Stream 0 global interrupt handler.
 *
 * Handles the DMA transfer complete interrupt for ADC1.
 * The HAL_DMA_IRQHandler processes the interrupt flags and calls
 * HAL_ADC_ConvCpltCallback() (defined in app_tasks.c), which gives
 * the binary semaphore to unblock Sensor_Task.
 *
 * NVIC Priority: 6 (within FreeRTOS managed range: 5-15)
 */
void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
 * @brief Hard Fault handler.
 *
 * Enters an infinite loop on hard fault. In a production system,
 * this would log fault information and potentially reset.
 */
void HardFault_Handler(void)
{
    /* Turn on red LED */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

    for (;;)
        ;
}

/**
 * @brief Memory Management fault handler.
 */
void MemManage_Handler(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    for (;;)
        ;
}

/**
 * @brief Bus Fault handler.
 */
void BusFault_Handler(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    for (;;)
        ;
}

/**
 * @brief Usage Fault handler.
 */
void UsageFault_Handler(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    for (;;)
        ;
}

/*===========================================================================*/
/*                    ERROR HANDLER                                          */
/*===========================================================================*/

/**
 * @brief General error handler for peripheral initialization failures.
 *
 * Called when HAL_xxx_Init() returns HAL_ERROR. Indicates a hardware
 * configuration problem. Halts with all LEDs blinking.
 */
static void Error_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Blink all LEDs to indicate initialization error */
    for (;;)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 |
                                      GPIO_PIN_14 | GPIO_PIN_15);
        for (volatile uint32_t i = 0; i < 800000; i++)
            ;
    }
}

/*===========================================================================*/
/*                    ASSERT FAILED (HAL DEBUG)                              */
/*===========================================================================*/

#ifdef USE_FULL_ASSERT
/**
 * @brief HAL assert failed callback.
 *
 * Called when a HAL function receives invalid parameters and USE_FULL_ASSERT
 * is defined. Reports the source file and line number.
 *
 * @param file  Source file name where the error occurred
 * @param line  Line number in the source file
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    char pcMsg[128];
    int iLen = snprintf(pcMsg, sizeof(pcMsg),
                        "[ASSERT] Failed in %s at line %lu\r\n",
                        (char *)file, (unsigned long)line);
    HAL_UART_Transmit(&huart2, (uint8_t *)pcMsg, (uint16_t)iLen, 1000);

    /* Halt */
    for (;;)
        ;
}
#endif /* USE_FULL_ASSERT */
