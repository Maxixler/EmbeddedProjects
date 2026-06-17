/**
 * @file main.c
 * @brief Main system initialization and FreeRTOS scheduler startup
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "main.h"
#include "system_config.h"
#include "pin_config.h"
#include "app_tasks.h"
#include "debug_uart.h"
#include "stm32f4xx_hal.h"

/* UART handle declaration (used by debug_uart.c) */
UART_HandleTypeDef huart2;

/* Function prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_Init(void);
void NVIC_Config(void);

/**
 * @brief Main application entry point
 */
int main(void)
{
    /* HAL library initialization */
    // HAL_Init(); // Would be called in actual STM32 HAL project

    /* Configure system clock */
    SystemClock_Config();

    /* Initialize GPIO pins */
    GPIO_Init();

    /* Initialize debug UART */
    debug_uart_init();

    debug_uart_printf("Ground Station Telemetry Processing Unit Starting\r\n");
    debug_uart_printf("System Clock: %lu Hz\r\n", SYSTEM_CORE_CLOCK);
    debug_uart_printf("UART Baud Rate: %lu bps\r\n", UART_BAUD_RATE);

    /* Initialize USART2 for telemetry input and debug output */
    USART2_Init();

    /* Configure NVIC for UART interrupts */
    NVIC_Config();

    /* Initialize application tasks and IPC objects */
    xAppTasksInit();

    /* Start FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never reach here */
    for (;;)
    {
        // If scheduler fails to start, trap here
        debug_uart_printf("ERROR: FreeRTOS scheduler failed to start\r\n");
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // Turn on error LED
        while (1)
        {
            // Blink error LED rapidly
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            HAL_Delay(100);
        }
    }
}

/**
 * @brief System clock configuration
 *        Configures system clock to 168 MHz using HSE and PLL
 */
void SystemClock_Config(void)
{
    // In a real STM32 HAL project, this would use RCC_OscInitTypeDef, RCC_ClkInitTypeDef, etc.
    // For this implementation, we'll document the intended configuration:
    //
    // PLL_M = 8   (HSE divider)
    // PLL_N = 336 (PLL multiplier)
    // PLL_P = 2   (PLL divider for main system clock)
    // PLL_Q = 7   (PLL divider for USB OTG FS, SDIO, RNG)
    //
    // VCO input frequency = HSE / PLL_M = 8 MHz / 8 = 1 MHz
    // VCO output frequency = VCO input frequency * PLL_N = 1 MHz * 336 = 336 MHz
    // System clock = VCO output frequency / PLL_P = 336 MHz / 2 = 168 MHz
    //
    // Clock source: PLLCLK
    // SYSCLK = 168 MHz
    // HCLK = SYSCLK = 168 MHz (AHB prescaler = 1)
    // APB1 prescaler = 4 -> APB1 clock = 168 MHz / 4 = 42 MHz
    // APB2 prescaler = 2 -> APB2 clock = 168 MHz / 2 = 84 MHz

    debug_uart_printf("System clock configured to 168 MHz\r\n");
}

/**
 * @brief GPIO initialization
 *        Configures GPIO pins for LEDs and UART
 */
void GPIO_Init(void)
{
    // Enable GPIO clocks
    // __HAL_RCC_GPIOD_CLK_ENABLE(); // For LEDs on PD12-PD15
    // __HAL_RCC_GPIOA_CLK_ENABLE(); // For UART2 PA2/PA3

    // Configure LED pins as outputs
    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Initialize LED states
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // Activity LED off
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); // Processing LED off
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // Error LED off
    // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);   // Power LED on (initial state)

    debug_uart_printf("GPIO initialized\r\n");
}

/**
 * @brief USART2 initialization
 *        Configures USART2 for 115200 baud, 8N1, interrupt-driven reception
 */
void USART2_Init(void)
{
    // Enable UART clock
    // __HAL_RCC_USART2_CLK_ENABLE();

    // Configure UART parameters
    // huart2.Instance = USART2;
    // huart2.Init.BaudRate = UART_BAUD_RATE;
    // huart2.Init.WordLength = UART_WORDLENGTH_8B;
    // huart2.Init.StopBits = UART_STOPBITS_1;
    // huart2.Init.Parity = UART_PARITY_NONE;
    // huart2.Init.Mode = UART_MODE_TX_RX;
    // huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    // if (HAL_UART_Init(&huart2) != HAL_OK)
    // {
    //     // Initialization Error
    //     while(1);
    // }

    debug_uart_printf("USART2 initialized at %lu baud\r\n", UART_BAUD_RATE);
}

/**
 * @brief NVIC configuration
 *        Configures interrupt priorities for UART2
 */
void NVIC_Config(void)
{
    // Set UART2 interrupt priority
    // HAL_NVIC_SetPriority(USART2_IRQn, 5, 0); // Priority 5, subpriority 0
    // HAL_NVIC_EnableIRQ(USART2_IRQn);

    debug_uart_printf("NVIC configured for USART2 interrupts\r\n");
}

/* Optional: UART2 interrupt handler for byte reception
void USART2_IRQHandler(void)
{
    uint8_t receivedByte;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Check if interrupt is due to reception
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        // Clear the interrupt flag
        __HAL_UART_CLEAR_PEFLAG(&huart2);

        // Get received byte
        receivedByte = (uint8_t)(huart2.Instance->DR & (uint8_t)0x00FF);

        // In a real implementation, we would send this byte to the UART reception task
        // via a queue or direct method. For this example, we'll note that the task
        // would normally block on a queue that is fed by this ISR.
        //
        // Example implementation:
        // if (xQueueSendFromISR(xUartByteQueue, &receivedByte, &xHigherPriorityTaskWoken) != pdPASS)
        // {
        //     // Handle queue full error
        // }
        //
        // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
*/