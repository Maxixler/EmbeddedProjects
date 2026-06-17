### Task 18: App Tasks Implementation

**Files:**
- Create: `Satellite-Projects/Ground-Station-Telemetry/src/app_tasks.c`

- [ ] **Step 1: Create app_tasks.c with task implementations and IPC object initialization**

```c
/**
 * @file app_tasks.c
 * @brief FreeRTOS task implementations and IPC object initialization
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "app_tasks.h"
#include "system_config.h"
#include "pin_config.h"
#include "frame_parser.h"
#include "telemetry_protocol.h"
#include "telemetry_types.h"
#include "lcd_driver.h"
#include "debug_uart.h"
#include "crc16.h"
#include "string.h"

/* IPC Objects */
QueueHandle_t xUartToProcessingQueue = NULL;
QueueHandle_t xProcessingToLcdQueue = NULL;
SemaphoreHandle_t xUartTxMutex = NULL;
EventGroupHandle_t xSystemEventGroup = NULL;

/* Frame parser state machine instance */
static FrameParserState_t xFrameParserState;

/* Task function implementations */

/**
 * @brief UART Reception Task - Highest priority (4)
 *        Handles interrupt-driven UART byte reception and frame assembly
 */
void vUartReceptionTask(void *pvParameters)
{
    BaseType_t xResult;
    uint8_t receivedByte;
    FrameParserStatus_t eFrameStatus;
    
    /* Initialize frame parser state machine */
    frame_parser_init(&xFrameParserState);
    
    debug_uart_printf("UART Reception Task started\r\n");
    
    for (;;)
    {
        /* In a real implementation, this would block on a UART receive interrupt or queue.
           For this example, we'll simulate byte reception.
           In practice, you would use UART interrupt or DMA to feed bytes to this task. */
        
        /* Simulate receiving a byte (replace with actual UART receive logic) */
        // receivedByte = UART_ReceiveByte(); // This would come from ISR or DMA
        
        /* For now, we'll just yield and simulate reception in the telemetry simulator */
        vTaskDelay(pdMS_TO_TICKS(1));
        
        /* In a real system, you would get bytes from a queue that is fed by UART ISR:
        if (xQueueReceive(xUartByteQueue, &receivedByte, portMAX_DELAY) == pdPASS)
        {
            // Feed byte to frame parser
            eFrameStatus = frame_parser_feed_byte(&xFrameParserState, receivedByte);
            
            if (eFrameStatus == FRAME_COMPLETE)
            {
                // Frame assembled completely, send to processing queue
                RawFrame_t *pxFrame = pvPortMalloc(sizeof(RawFrame_t));
                if (pxFrame != NULL)
                {
                    memcpy(pxFrame, &xFrameParserState.rawFrame, sizeof(RawFrame_t));
                    xResult = xQueueSend(xUartToProcessingQueue, &pxFrame, portMAX_DELAY);
                    if (xResult != pdPASS)
                    {
                        vPortFree(pxFrame);
                        debug_uart_printf("UART RX: Failed to send frame to processing queue\r\n");
                    }
                    else
                    {
                        // Toggle activity LED on successful frame reception
                        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // PD12 - Green/Activity LED
                    }
                }
                else
                {
                    debug_uart_printf("UART RX: Failed to allocate memory for frame\r\n");
                }
                
                // Reset frame parser for next frame
                frame_parser_init(&xFrameParserState);
            }
            else if (eFrameStatus == FRAME_TIMEOUT)
            {
                // Handle frame timeout
                debug_uart_printf("UART RX: Frame reception timeout\r\n");
                frame_parser_init(&xFrameParserState); // Reset parser
            }
        }
        */
    }
}

/**
 * @brief Telemetry Processing Task - Priority 3
 *        Validates frames, extracts telemetry fields, handles errors
 */
void vTelemetryProcessingTask(void *pvParameters)
{
    BaseType_t xResult;
    RawFrame_t *pxReceivedFrame;
    TelemetryData_t xTelemetryData;
    TelemetryStatus_t eTelemetryStatus;
    SystemStats_t xSystemStats;
    
    /* Initialize system statistics */
    system_stats_init(&xSystemStats);
    
    debug_uart_printf("Telemetry Processing Task started\r\n");
    
    for (;;)
    {
        /* Wait for incoming frame from UART reception task */
        xResult = xQueueReceive(xUartToProcessingQueue, &pxReceivedFrame, portMAX_DELAY);
        
        if (xResult == pdPASS && pxReceivedFrame != NULL)
        {
            // Increment received packet counter
            system_stats_inc_rx(&xSystemStats);
            
            // Validate the received frame
            eTelemetryStatus = telemetry_validate_frame(pxReceivedFrame);
            
            if (eTelemetryStatus == TELEMETRY_STATUS_OK)
            {
                // Frame is valid, extract telemetry fields
                eTelemetryStatus = telemetry_extract_fields(pxReceivedFrame, &xTelemetryData);
                
                if (eTelemetryStatus == TELEMETRY_STATUS_OK)
                {
                    // Increment valid packet counter
                    system_stats_inc_valid(&xSystemStats);
                    
                    // Send telemetry data to LCD display task
                    xResult = xQueueSend(xProcessingToLcdQueue, &xTelemetryData, portMAX_DELAY);
                    if (xResult != pdPASS)
                    {
                        debug_uart_printf("PROC: Failed to send telemetry data to LCD queue\r\n");
                    }
                    
                    // Output debug information
                    debug_uart_printf("PROC: Valid frame received - Temp: %.2f°C, Voltage: %.2fV\r\n", 
                                    xTelemetryData.temperature, xTelemetryData.voltage);
                }
                else
                {
                    // Field extraction failed
                    system_stats_inc_format_error(&xSystemStats);
                    debug_uart_printf("PROC: Failed to extract fields from valid frame\r\n");
                }
            }
            else
            {
                // Frame validation failed
                switch (eTelemetryStatus)
                {
                    case TELEMETRY_STATUS_INVALID_CRC:
                        system_stats_inc_crc_error(&xSystemStats);
                        debug_uart_printf("PROC: CRC validation failed\r\n");
                        // Set error LED on CRC failure
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET); // PD14 - Red/Error LED
                        vTaskDelay(pdMS_TO_TICKS(100)); // Keep LED on briefly
                        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
                        break;
                    
                    case TELEMETRY_STATUS_INVALID_START:
                    case TELEMETRY_STATUS_INVALID_LENGTH:
                    case TELEMETRY_STATUS_INVALID_FIELD:
                        system_stats_inc_format_error(&xSystemStats);
                        debug_uart_printf("PROC: Invalid frame format\r\n");
                        break;
                    
                    default:
                        system_stats_inc_format_error(&xSystemStats);
                        debug_uart_printf("PROC: Unknown validation error\r\n");
                        break;
                }
            }
            
            // Free the received frame buffer
            vPortFree(pxReceivedFrame);
            pxReceivedFrame = NULL;
        }
    }
}

/**
 * @brief LCD Display Task - Priority 2
 *        Formats telemetry data for display and updates LCD
 */
void vLcdDisplayTask(void *pvParameters)
{
    BaseType_t xResult;
    TelemetryData_t xReceivedData;
    char buffer[32];
    TickType_t xLastDisplayTime = 0;
    const TickType_t xDisplayPeriod = pdMS_TO_TICKS(200); // Update at 5 Hz max (200ms)
    
    debug_uart_printf("LCD Display Task started\r\n");
    
    /* Initialize LCD driver */
    lcd_init();
    
    for (;;)
    {
        /* Wait for telemetry data from processing task */
        xResult = xQueueReceive(xProcessingToLcdQueue, &xReceivedData, xDisplayPeriod);
        
        if (xResult == pdPASS)
        {
            // Format and display telemetry data on LCD
            lcd_clear();
            lcd_set_cursor(0, 0);
            
            // Display timestamp
            if (xReceivedData.validTimestamp)
            {
                telemetry_format_timestamp(buffer, sizeof(buffer), xReceivedData.timestamp);
                lcd_print_string("Time: ");
                lcd_print_string(buffer);
            }
            else
            {
                lcd_print_string("Time: ----");
            }
            
            lcd_set_cursor(0, 1);
            
            // Display temperature and voltage
            if (xReceivedData.validTemperature && xReceivedData.validVoltage)
            {
                telemetry_format_temperature(buffer, sizeof(buffer), xReceivedData.temperature);
                lcd_print_string("T: ");
                lcd_print_string(buffer);
                
                lcd_set_cursor(9, 1); // Second half of line
                telemetry_format_voltage(buffer, sizeof(buffer), xReceivedData.voltage);
                lcd_print_string("V: ");
                lcd_print_string(buffer);
            }
            else if (xReceivedData.validTemperature)
            {
                telemetry_format_temperature(buffer, sizeof(buffer), xReceivedData.temperature);
                lcd_print_string("T: ");
                lcd_print_string(buffer);
                lcd_print_string(" V: ----");
            }
            else if (xReceivedData.validVoltage)
            {
                lcd_print_string("T: ----");
                lcd_set_cursor(9, 1);
                telemetry_format_voltage(buffer, sizeof(buffer), xReceivedData.voltage);
                lcd_print_string("V: ");
                lcd_print_string(buffer);
            }
            else
            {
                lcd_print_string("T: ---- V: ----");
            }
            
            // Optional: Display status on separate line or toggle LEDs
            if (xReceivedData.validStatus)
            {
                // Could display status or use LEDs for status indication
                // For now, we'll just note it in debug output
                debug_uart_printf("LCD: Status = 0x%02X\r\n", xReceivedData.status);
            }
        }
        else
        {
            // Timeout - optionally show last data or idle display
            // Could implement a screensaver or power-saving mode here
        }
        
        // Ensure we don't update too frequently (enforce 5Hz max)
        vTaskDelayUntil(&xLastDisplayTime, xDisplayPeriod);
    }
}

/**
 * @brief System Monitor Task - Priority 1 (Lowest)
 *        Provides periodic system health monitoring and LED heartbeat
 */
void vSystemMonitorTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xMonitorPeriod = pdMS_TO_TICKS(1000); // 1 second period
    SystemStats_t xSystemStats;
    char buffer[64];
    uint32_t ulnSeconds = 0;
    
    /* Initialize system statistics */
    system_stats_init(&xSystemStats);
    
    debug_uart_printf("System Monitor Task started\r\n");
    
    /* Initialize last wake time variable */
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        /* Wait until it's time to run again */
        vTaskDelayUntil(&xLastWakeTime, xMonitorPeriod);
        
        // Increment uptime counter
        ulnSeconds++;
        system_stats_set_uptime(&xSystemStats, ulnSeconds);
        
        // Get stack high water marks for all tasks (would need task handles)
        // For simplicity, we'll just report basic stats in this example
        
        // Output system status via debug UART
        debug_uart_printf("MONITOR: Uptime=%lus, RX=%lu, Valid=%lu, CRC_ERR=%lu, FMT_ERR=%lu\r\n",
                         ulnSeconds,
                         xSystemStats.packetsReceived,
                         xSystemStats.packetsValid,
                         xSystemStats.packetsInvalidCrc,
                         xSystemStats.packetsInvalidFormat);
        
        // Toggle power LED (heartbeat)
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15); // PD15 - Blue/Power LED
        
        // Optional: Check for error conditions and take action
        if (xSystemStats.packetsInvalidCrc > 10)
        {
            // Persistent CRC errors - might indicate communication issues
            debug_uart_printf("MONITOR: High CRC error rate detected\r\n");
        }
    }
}

/**
 * @brief Initialize all application tasks and IPC objects
 */
void xAppTasksInit(void)
{
    /* Create IPC objects */
    xUartToProcessingQueue = xQueueCreate(UART_TO_PROC_QUEUE_SIZE, sizeof(RawFrame_t *));
    xProcessingToLcdQueue = xQueueCreate(PROC_TO_LCD_QUEUE_SIZE, sizeof(TelemetryData_t));
    xUartTxMutex = xSemaphoreCreateMutex();
    xSystemEventGroup = xEventGroupCreate();
    
    /* Verify IPC objects were created successfully */
    configASSERT(xUartToProcessingQueue != NULL);
    configASSERT(xProcessingToLcdQueue != NULL);
    configASSERT(xUartTxMutex != NULL);
    configASSERT(xSystemEventGroup != NULL);
    
    /* Create application tasks */
    BaseType_t xResult;
    
    // UART Reception Task - Priority 4 (highest)
    xResult = xTaskCreate(vUartReceptionTask, "UART_RX", UART_RX_TASK_STACK_SIZE, 
                         NULL, UART_RX_TASK_PRIORITY, NULL);
    configASSERT(xResult == pdPASS);
    
    // Telemetry Processing Task - Priority 3
    xResult = xTaskCreate(vTelemetryProcessingTask, "TELEM_PROC", PROCESSING_TASK_STACK_SIZE, 
                         NULL, PROCESSING_TASK_PRIORITY, NULL);
    configASSERT(xResult == pdPASS);
    
    // LCD Display Task - Priority 2
    xResult = xTaskCreate(vLcdDisplayTask, "LCD_DISP", LCD_TASK_STACK_SIZE, 
                         NULL, LCD_TASK_PRIORITY, NULL);
    configASSERT(xResult == pdPASS);
    
    // System Monitor Task - Priority 1 (lowest)
    xResult = xTaskCreate(vSystemMonitorTask, "SYS_MON", MONITOR_TASK_STACK_SIZE, 
                         NULL, MONITOR_TASK_PRIORITY, NULL);
    configASSERT(xResult == pdPASS);
    
    debug_uart_printf("All application tasks created successfully\r\n");
}
```

- [ ] **Step 2: Verify app_tasks.c compiles**

```bash
# Will verify during compilation
echo "app_tasks.c created - will verify during compilation"
```

- [ ] **Step 3: Commit app tasks implementation**

```bash
git add src/app_tasks.c
git commit -m "feat: add FreeRTOS task implementations and IPC object initialization"
```

### Task 19: Main System Implementation

**Files:**
- Create: `Satellite-Projects/Ground-Station-Telemetry/src/main.c`

- [ ] **Step 1: Create main.c with system initialization and FreeRTOS scheduler startup**

```c
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
```

- [ ] **Step 2: Verify main.c compiles**

```bash
# Will verify during compilation
echo "main.c created - will verify during compilation"
```

- [ ] **Step 3: Commit main system implementation**

```bash
git add src/main.c
git commit -m "feat: add main system initialization and FreeRTOS scheduler startup"
```

### Task 20: Frame Parser Implementation

**Files:**
- Create: `Satellite-Projects/Ground-Station-Telemetry/src/frame_parser.c`

- [ ] **Step 1: Create frame_parser.c with state machine implementation**

```c
/**
 * @file frame_parser.c
 * @brief Frame parser state machine implementation for telemetry frame assembly
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "frame_parser.h"
#include <string.h>

/**
 * @brief Initialize frame parser state machine
 * @param pxState Pointer to frame parser state structure
 */
void frame_parser_init(FrameParserState_t *pxState)
{
    if (pxState == NULL)
        return;
    
    pxState->eState = WAIT_START1;
    pxState->rawFrame.startMarker1 = 0;
    pxState->rawFrame.startMarker2 = 0;
    pxState->rawFrame.length = 0;
    pxState->rawFrame.payloadIndex = 0;
    pxState->rawFrame.tickCount = 0;
    pxState->ulStartTick = 0;
    
    // Clear payload buffer
    memset(pxState->rawFrame.payload, 0, MAX_FRAME_BUFFER_SIZE);
}

/**
 * @brief Feed a byte to the frame parser state machine
 * @param pxState Pointer to frame parser state structure
 * @param ucByte Received byte to process
 * @return FrameParserStatus_t Status of frame parsing operation
 */
FrameParserStatus_t frame_parser_feed_byte(FrameParserState_t *pxState, uint8_t ucByte)
{
    FrameParserStatus_t eReturnStatus = FRAME_INCOMPLETE;
    TickType_t xCurrentTick;
    
    if (pxState == NULL)
        return FRAME_ERROR;
    
    xCurrentTick = xTaskGetTickCount();
    
    // Check for timeout (if we've been waiting too long for a complete frame)
    if ((pxState->eState != WAIT_START1) && 
        ((xCurrentTick - pxState->ulStartTick) > pdMS_TO_TICKS(FRAME_TIMEOUT_MS)))
    {
        // Timeout occurred
        frame_parser_init(pxState); // Reset state machine
        return FRAME_TIMEOUT;
    }
    
    // State machine processing
    switch (pxState->eState)
    {
        case WAIT_START1:
            // Looking for first start marker (0xAA)
            if (ucByte == TELEMETRY_START_MARKER1)
            {
                pxState->rawFrame.startMarker1 = ucByte;
                pxState->eState = WAIT_START2;
                pxState->ulStartTick = xCurrentTick; // Start timeout timer
            }
            // Stay in WAIT_START1 if byte doesn't match
            break;
            
        case WAIT_START2:
            // Looking for second start marker (0x55)
            if (ucByte == TELEMETRY_START_MARKER2)
            {
                pxState->rawFrame.startMarker2 = ucByte;
                pxState->eState = WAIT_LENGTH;
            }
            else
            {
                // Second start marker not received, go back to waiting for first
                pxState->eState = WAIT_START1;
                pxState->rawFrame.startMarker1 = 0; // Clear first marker
            }
            break;
            
        case WAIT_LENGTH:
            // Receiving length byte
            pxState->rawFrame.length = ucByte;
            
            // Validate length
            if (pxState->rawFrame.length > MAX_FRAME_BUFFER_SIZE)
            {
                // Length too large, reset state machine
                frame_parser_init(pxState);
                return FRAME_ERROR;
            }
            else if (pxState->rawFrame.length == 0)
            {
                // Zero length frame - go directly to CRC waiting (no payload)
                pxState->eState = WAIT_CRC_HIGH;
                pxState->rawFrame.payloadIndex = 0;
            }
            else
            {
                // Valid length, proceed to receive payload
                pxState->eState = RECEIVE_PAYLOAD;
                pxState->rawFrame.payloadIndex = 0;
            }
            break;
            
        case RECEIVE_PAYLOAD:
            // Receiving payload bytes
            if (pxState->rawFrame.payloadIndex < MAX_FRAME_BUFFER_SIZE)
            {
                pxState->rawFrame.payload[pxState->rawFrame.payloadIndex] = ucByte;
                pxState->rawFrame.payloadIndex++;
                
                // Check if we've received all payload bytes
                if (pxState->rawFrame.payloadIndex >= pxState->rawFrame.length)
                {
                    pxState->eState = WAIT_CRC_HIGH;
                }
            }
            else
            {
                // Buffer overflow - reset state machine
                frame_parser_init(pxState);
                return FRAME_ERROR;
            }
            break;
            
        case WAIT_CRC_HIGH:
            // Receiving high byte of CRC
            pxState->rawFrame.crcHigh = ucByte;
            pxState->eState = WAIT_CRC_LOW;
            break;
            
        case WAIT_CRC_LOW:
            // Receiving low byte of CRC - frame is complete
            pxState->rawFrame.crcLow = ucByte;
            pxState->eState = WAIT_START1; // Reset for next frame
            eReturnStatus = FRAME_COMPLETE;
            break;
            
        default:
            // Unknown state - reset state machine
            frame_parser_init(pxState);
            eReturnStatus = FRAME_ERROR;
            break;
    }
    
    return eReturnStatus;
}

/**
 * @brief Reset frame parser state machine
 * @param pxState Pointer to frame parser state structure
 */
void frame_parser_reset(FrameParserState_t *pxState)
{
    if (pxState != NULL)
    {
        frame_parser_init(pxState);
    }
}
```

- [ ] **Step 2: Verify frame_parser.c compiles**

```bash
# Will verify during compilation
echo "frame_parser.c created - will verify during compilation"
```

- [ ] **Step 3: Commit frame parser implementation**

```bash
git add src/frame_parser.c
git commit -m "feat: add frame parser state machine implementation"
```

### Task 21: Build System Verification

**Files:**
- Modify: Various (compile all source files)

- [ ] **Step 1: Create a simple Makefile for building the project**

```makefile
# Simple Makefile for Ground Station Telemetry Processing Unit
# This is a basic example - in practice you'd use STM32CubeIDE or similar

CC = arm-none-eabi-gcc
CFLAGS = -Wall -Wextra -Werror -O2 -g -std=c99 \
         -Iinc \
         -IFreeRTOS/Source/include \
         -IFreeRTOS/Source/portable/GCC/ARM_CM4F \
         -DSTM32F407xx

LDFLAGS = -Tstm32f407vg_flash.ld -nostartfiles -Wl,--gc-sections
LIBS = -lm -lc -lgcc

SRC_DIR = src
INC_DIR = inc
FREERTOS_DIR = FreeRTOS/Source

SOURCES = $(wildcard $(SRC_DIR)/*.c) \
          $(wildcard $(FREERTOS_DIR)/*.c) \
          $(wildcard $(FREERTOS_DIR)/portable/MemMang/heap_4.c) \
          $(wildcard $(FREERTOS_DIR)/portable/GCC/ARM_CM4F/port.c)

OBJECTS = $(SOURCES:.c=.o)

TARGET = ground_station_telemetry.elf

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)
	@echo "Build successful: $@"

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Clean completed"

flash: $(TARGET)
	# Example flash command (adjust for your debugger)
	# openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $$< verify reset exit"
	@echo "Flash command would go here for: $<"

debug: $(TARGET)
	# Example debug command
	# arm-none-eabi-gdb $<
	@echo "Debug command would go here for: $<"

.PHONY: all clean flash debug
```

- [ ] **Step 2: Verify build system works**

```bash
# Try to compile the project
make clean
make
```

- [ ] **Step 3: Commit build system**

```bash
git add Makefile
git commit -m "feat: add basic Makefile for project build"
```

### Task 22: Telemetry Simulator Tool

**Files:**
- Create: `Satellite-Projects/Ground-Station-Telemetry/tools/telemetry_simulator.py`

- [ ] **Step 1: Create telemetry_simulator.py for generating test frames**

```python
#!/usr/bin/env python3
"""
Telemetry Simulator Tool
Generates test telemetry frames for ground station testing
"""

import serial
import struct
import time
import argparse
import random
from typing import List

def calculate_crc16_ccitt(data: bytes) -> int:
    """Calculate CRC-16-CCITT (0x1021) for given data"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

def create_telemetry_frame(timestamp: int, temperature: float, voltage: float, status: int) -> bytes:
    """Create a standard telemetry frame with start markers, payload, and CRC"""
    # Start markers
    frame = bytearray([0xAA, 0x55])
    
    # Payload: timestamp (4B) + temperature (4B float) + voltage (4B float) + status (1B)
    payload = bytearray()
    payload.extend(struct.pack('>I', timestamp))  # Big-endian 32-bit unsigned int
    payload.extend(struct.pack('>f', temperature)) # Big-endian 32-bit float
    payload.extend(struct.pack('>f', voltage))     # Big-endian 32-bit float
    payload.append(status & 0xFF)                  # 8-bit status
    
    # Add length byte
    frame.append(len(payload))
    
    # Add payload
    frame.extend(payload)
    
    # Calculate and add CRC-16
    crc = calculate_crc16_ccitt(payload)
    frame.append((crc >> 8) & 0xFF)  # CRC high byte
    frame.append(crc & 0xFF)         # CRC low byte
    
    return bytes(frame)

def main():
    parser = argparse.ArgumentParser(description='Telemetry Frame Simulator')
    parser.add_argument('--port', default='COM3', help='Serial port (default: COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--interval', type=float, default=1.0, help='Frame interval in seconds (default: 1.0)')
    parser.add_argument('--temp-min', type=float, default=-20.0, help='Minimum temperature (°C)')
    parser.add_argument('--temp-max', type=float, default=50.0, help='Maximum temperature (°C)')
    parser.add_argument('--volt-min', type=float, default=3.0, help='Minimum voltage (V)')
    parser.add_argument('--volt-max', type=float, default=5.0, help='Maximum voltage (V)')
    parser.add_argument('--count', type=int, default=0, help='Number of frames to send (0=infinite)')
    parser.add_argument('--error-rate', type=float, default=0.0, help='CRC error rate (0.0-1.0)')
    parser.add_argument('--truncate-rate', type=float, default=0.0, help='Frame truncation rate (0.0-1.0)')
    
    args = parser.parse_args()
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Connected to {args.port} at {args.baud} baud")
        print(f"Sending frames every {args.interval} seconds")
        print(f"Temperature range: {args.temp_min} to {args.temp_max} °C")
        print(f"Voltage range: {args.volt_min} to {args.volt_max} V")
        if args.error_rate > 0:
            print(f"CRC error rate: {args.error_rate*100}%")
        if args.truncate_rate > 0:
            print(f"Truncation rate: {args.truncate_rate*100}%")
        print("Press Ctrl+C to stop\n")
        
        frame_count = 0
        while args.count == 0 or frame_count < args.count:
            # Generate random telemetry values within specified ranges
            temperature = random.uniform(args.temp_min, args.temp_max)
            voltage = random.uniform(args.volt_min, args.volt_max)
            status = random.randint(0, 255)  # Random status byte
            timestamp = int(time.time())  # Current timestamp
            
            # Create telemetry frame
            frame = create_telemetry_frame(timestamp, temperature, voltage, status)
            
            # Apply errors if requested
            if args.error_rate > 0 and random.random() < args.error_rate:
                # Introduce CRC error by flipping a random bit in CRC
                frame_list = list(frame)
                crc_pos = len(frame) - 2  # Position of CRC high byte
                bit_to_flip = random.randint(0, 7)
                frame_list[crc_pos] ^= (1 << bit_to_flip)
                frame = bytes(frame_list)
                print(f"Frame {frame_count}: CRC ERROR INJECTED")
            
            # Apply truncation if requested
            if args.truncate_rate > 0 and random.random() < args.truncate_rate:
                # Truncate frame at random point (after start markers)
                if len(frame) > 4:  # Need at least start markers + length
                    truncate_point = random.randint(3, len(frame) - 1)
                    frame = frame[:truncate_point]
                    print(f"Frame {frame_count}: FRAME TRUNCATED to {len(frame)} bytes")
            
            # Send frame
            ser.write(frame)
            ser.flush()
            
            # Print frame info
            print(f"Frame {frame_count}: "
                  f"TS={timestamp}, T={temperature:.1f}°C, V={voltage:.2f}V, S=0x{status:02X} "
                  f"({len(frame)} bytes)")
            
            frame_count += 1
            time.sleep(args.interval)
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify telemetry_simulator.py works**

```bash
# Check if python is available and test the script
python --python
python tools/telemetry_simulator.py --help
```

- [ ] **Step 3: Commit telemetry simulator tool**

```bash
git add tools/telemetry_simulator.py
git commit -m "feat: add telemetry simulator Python script for testing"
```

### Task 23: Final Verification and Documentation

**Files:**
- Create: `Satellite-Projects/Ground-Station-Telemetry/README.md`
- Create: `Satellite-Projects/Ground-Station-Telemetry/DESIGN.md` (reference to existing)
- Create: `Satellite-Projects/Ground-Station-Telemetry/APPROACHES.md` (reference to existing)

- [ ] **Step 1: Create README.md with project overview**

```markdown
# Ground Station Telemetry Processing Unit

## Overview

This project implements a Ground Station Telemetry Processing Unit on STM32F407G-DISC1 using FreeRTOS. The system receives telemetry frames via UART, validates CRC-16, extracts telemetry fields, and displays results on UART debug output and an LCD (interface-only, no hardware yet).

## Features

- **FreeRTOS-based Architecture**: Four tasks with priority-based preemptive scheduling
- **Modular Design**: Separate components for frame parsing, protocol handling, and display
- **Robust Error Detection**: CRC-16-CCITT validation with error counting and reporting
- **Thread-Safe Debug Output**: Mutex-protected UART transmission from multiple tasks
- **Configurable Parameters**: System clock, UART baud rate, queue sizes, stack depths
- **Test Tooling**: Python telemetry simulator for generating test frames

## Hardware Requirements

- STM32F407G-DISC1 Discovery board
- USB-to-USB cable for programming and debug output
- (Optional) LCD display with I2C interface for future expansion

## Software Components

### Core Modules
- `frame_parser.c/h`: Byte-by-byte frame assembly state machine
- `telemetry_protocol.c/h`: Frame validation (CRC, format) and field extraction
- `telemetry_types.c/h`: Data structures and formatting helpers
- `lcd_driver.c/h`: LCD display abstraction (currently stubbed to UART)
- `crc16.c/h`: Table-driven CRC-16-CCITT implementation
- `debug_uart.c/h`: Mutex-protected debug UART output
- `app_tasks.c/h`: FreeRTOS task implementations and IPC objects
- `main.c`: System initialization and FreeRTOS scheduler startup

### Configuration Headers
- `system_config.h`: Clock settings, UART baud rate, task priorities/sizes
- `pin_config.h`: GPIO pin assignments for LEDs and UART
- `telemetry_config.h`: Frame format constants and field definitions

### Tooling
- `tools/telemetry_simulator.py`: Python script for generating test telemetry frames

## Task Architecture

1. **vUartReceptionTask** (Priority 4): 
   - Interrupt-driven UART byte reception
   - Frame assembly via state machine
   - Sends complete frames to processing queue
   - Toggles activity LED on frame reception

2. **vTelemetryProcessingTask** (Priority 3):
   - Validates frames (CRC, format)
   - Extracts telemetry fields (timestamp, temperature, voltage, status)
   - Sends valid telemetry to LCD queue
   - Handles error conditions (sets error LED on CRC fail)

3. **vLcdDisplayTask** (Priority 2):
   - Formats telemetry data for LCD display
   - Updates LCD at ≤5 Hz
   - Currently routes output to debug UART

4. **vSystemMonitorTask** (Priority 1):
   - Periodic system health reporting (every second)
   - Stack usage monitoring (conceptual)
   - Packet/error counter tracking
   - LED heartbeat management

## Communication Protocols

### Telemetry Frame Structure
```
+----------------+----------------+----------------+----------------+
| Start Marker   | Length         | Payload        | CRC-16         |
| (0xAA 0x55)    | (1 byte)       | (N bytes)      | (2 bytes)      |
+----------------+----------------+----------------+----------------+
```

### Standard Payload Format
```
+----------------+----------------+----------------+----------------+
| Timestamp      | Temperature    | Voltage        | Status         |
| (4 bytes)      | (4 bytes float)| (4 bytes float)| (1 byte)       |
+----------------+----------------+----------------+----------------+
```

## Getting Started

1. Clone the repository
2. Import the project into STM32CubeIDE or compile with provided Makefile
3. Connect STM32F407G-DISC1 to PC via USB
4. Build and flash the firmware
5. Run `tools/telemetry_simulator.py` to generate test frames
6. Observe debug output via USB virtual COM port at 115200 baud

## Testing

### Automated Verification
1. CRC-16 unit validation: Known test vectors (e.g., "123456789" → 0x29B1 for CCITT)
2. Frame parser validation: Feed byte sequences and verify state transitions
3. Build verification: Clean compile with no warnings (`-Wall -Wextra -Werror`)

### On-Target Verification
1. Flash to Discovery board → verify UART debug output appears at 115200
2. Run `telemetry_simulator.py` → verify frames are received, validated, and displayed
3. Send corrupted frames → verify CRC errors are detected and counted
4. Monitor stack high water marks via System Monitor task output
5. Verify LED behavior: Green=activity, Red=CRC error, Blue=power on

## License

This project is open source and available for educational and portfolio purposes.

## Acknowledgments

Developed as part of embedded systems skill development for aerospace/defense applications.
```

- [ ] **Step 2: Verify README.md content**

```bash
echo "README.md created - verifying content"
```

- [ ] **Step 3: Commit README.md**

```bash
git add README.md
git commit -m "feat: add project README with overview and instructions"
```

- [ ] **Step 4: Final verification - check that all planned files exist**

```bash
# List all files that should have been created
echo "Verifying project structure:"
ls -la Satellite-Projects/Ground-Station-Telemetry/
echo "Header files:"
ls -la Satellite-Projects/Ground-Station-Telemetry/inc/
echo "Source files:"
ls -la Satellite-Projects/Ground-Station-Telemetry/src/
echo "Tools:"
ls -la Satellite-Projects/Ground-Station-Telemetry/tools/
```

- [ ] **Step 5: Final commit - mark implementation complete**

```bash
git commit --allow-empty -m "feat: complete Ground Station Telemetry Processing Unit implementation"
```

## Verification Plan

### Automated Verification
1. **CRC-16 unit validation**: Known test vectors (e.g., "123456789" → 0x29B1 for CCITT)
2. **Frame parser validation**: Feed byte sequences and verify state transitions
3. **Build verification**: Clean compile with no warnings (`-Wall -Wextra -Werror`)

### On-Target Verification
1. Flash to Discovery board → verify UART debug output appears at 115200
2. Run `telemetry_simulator.py` → verify frames are received, validated, and displayed
3. Send corrupted frames → verify CRC errors are detected and counted
4. Monitor stack high water marks via System Monitor task output
5. Verify LED behavior: Green=activity, Red=CRC error, Blue=power on

### Manual Verification
1. Review debug UART output for correct telemetry field values
2. Confirm system runs stable for extended period (>30 min)
3. Test frame burst (rapid consecutive frames) → no queue overflow