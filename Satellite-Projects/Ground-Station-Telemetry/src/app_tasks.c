/**
 * @file app_tasks.c
 * @brief FreeRTOS task implementations for the Ground Station Telemetry Processing Unit.
 *
 * This file contains the implementation of the UART reception task, telemetry processing task,
 * LCD display task, and system monitor task, along with the required IPC objects and
 * initialization function.
 */

/*=== Includes ============================================================*/
#include "app_tasks.h"
#include "telemetry_protocol.h"
#include "frame_parser.h"
#include "lcd_driver.h"
#include "debug_uart.h"
#include "crc16.h"
#include "telemetry_types.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include <stdio.h>

/*=== Defines =============================================================*/
/* Task priorities */
#define UART_RX_TASK_PRIO          (4U)
#define PROCESSING_TASK_PRIO       (3U)
#define LCD_TASK_PRIO              (2U)
#define MONITOR_TASK_PRIO          (1U)

/* Task stack sizes */
#define UART_RX_TASK_STACK_SIZE    (256U)
#define PROCESSING_TASK_STACK_SIZE (512U)
#define LCD_TASK_STACK_SIZE        (256U)
#define MONITOR_TASK_STACK_SIZE    (256U)

/* Queue lengths */
#define UART_TO_PROC_QUEUE_LEN     (10U)
#define PROC_TO_LCD_QUEUE_LEN      (5U)

/*=== IPC Objects =========================================================*/
/* Queues */
static QueueHandle_t xUartToProcessingQueue = NULL;
static QueueHandle_t xProcessingToLcdQueue = NULL;

/* Mutex for UART TX (debug output) */
static SemaphoreHandle_t xUartTxMutex = NULL;

/* Event group for error/status signaling */
static EventGroupHandle_t xSystemEventGroup = NULL;

/* Task handles (optional, for debugging) */
static TaskHandle_t xUartRxTaskHandle = NULL;
static TaskHandle_t xTelemetryProcTaskHandle = NULL;
static TaskHandle_t xLcdDisplayTaskHandle = NULL;
static TaskHandle_t xSystemMonitorTaskHandle = NULL;

/*=== Task Function Prototypes ============================================*/
static void vUartReceptionTask(void *pvParameters);
static void vTelemetryProcessingTask(void *pvParameters);
static void vLcdDisplayTask(void *pvParameters);
static void vSystemMonitorTask(void *pvParameters);

/*=== External Variables ==================================================*/
/* Assume these are defined elsewhere (e.g., in main.c or system_config.h) */
extern UART_HandleTypeDef huart2; /* USART2 handle */

/*=== Function Implementations ============================================*/

/**
 * @brief Initialize application tasks, IPC objects, and create tasks.
 *
 * @return BaseType_t pdPASS if successful, pdFAIL otherwise.
 */
BaseType_t xAppTasksInit(void)
{
    BaseType_t xReturn = pdFAIL;

    /*=== Create IPC objects ===*/
    xUartToProcessingQueue = xQueueCreate(UART_TO_PROC_QUEUE_LEN, sizeof(RawFrame_t));
    if (xUartToProcessingQueue == NULL)
    {
        goto cleanup;
    }

    xProcessingToLcdQueue = xQueueCreate(PROC_TO_LCD_QUEUE_LEN, sizeof(TelemetryData_t));
    if (xProcessingToLcdQueue == NULL)
    {
        goto cleanup;
    }

    xUartTxMutex = xSemaphoreCreateMutex();
    if (xUartTxMutex == NULL)
    {
        goto cleanup;
    }

    xSystemEventGroup = xEventGroupCreate();
    if (xSystemEventGroup == NULL)
    {
        goto cleanup;
    }

    /*=== Create tasks ===*/
    if (xTaskCreate(vUartReceptionTask, "UART_RX", UART_RX_TASK_STACK_SIZE,
                    NULL, UART_RX_TASK_PRIO, &xUartRxTaskHandle) != pdPASS)
    {
        goto cleanup;
    }

    if (xTaskCreate(vTelemetryProcessingTask, "TELEMETRY_PROC", PROCESSING_TASK_STACK_SIZE,
                    NULL, PROCESSING_TASK_PRIO, &xTelemetryProcTaskHandle) != pdPASS)
    {
        goto cleanup;
    }

    if (xTaskCreate(vLcdDisplayTask, "LCD_DISPLAY", LCD_TASK_STACK_SIZE,
                    NULL, LCD_TASK_PRIO, &xLcdDisplayTaskHandle) != pdPASS)
    {
        goto cleanup;
    }

    if (xTaskCreate(vSystemMonitorTask, "SYSTEM_MONITOR", MONITOR_TASK_STACK_SIZE,
                    NULL, MONITOR_TASK_PRIO, &xSystemMonitorTaskHandle) != pdPASS)
    {
        goto cleanup;
    }

    xReturn = pdPASS;

cleanup:
    if (xReturn != pdPASS)
    {
        /* Cleanup on failure */
        if (xUartToProcessingQueue != NULL) { vQueueDelete(xUartToProcessingQueue); xUartToProcessingQueue = NULL; }
        if (xProcessingToLcdQueue != NULL) { vQueueDelete(xProcessingToLcdQueue); xProcessingToLcdQueue = NULL; }
        if (xUartTxMutex != NULL) { vSemaphoreDelete(xUartTxMutex); xUartTxMutex = NULL; }
        if (xSystemEventGroup != NULL) { vEventGroupDelete(xSystemEventGroup); xSystemEventGroup = NULL; }
        /* Note: Task deletion cleanup is more complex and omitted for brevity */
    }

    return xReturn;
}

/*=== UART Reception Task =================================================*/
/**
 * @brief UART Reception Task: Receives bytes via USART2, assembles frames,
 *        and sends complete frames to the processing queue.
 *
 * @note In a production system, this task would be driven by UART interrupts.
 *       For simplicity, this implementation uses polling with a timeout.
 *
 * @param pvParameters Unused task parameter.
 */
static void vUartReceptionTask(void *pvParameters)
{
    (void) pvParameters;
    FrameParserState_t xParserState;
    BaseType_t xQueueSendResult;
    uint8_t ucReceivedByte;

    /* Initialize the frame parser state machine */
    frame_parser_init(&xParserState);

    for (;;)
    {
        /* Wait for a byte to be received via UART (polling with timeout) */
        if (HAL_UART_Receive(&huart2, &ucReceivedByte, 1, 10) == HAL_OK)
        {
            /* Feed the byte to the frame parser */
            FrameParserStatus_t eResult = frame_parser_feed_byte(&xParserState, ucReceivedByte);

            if (eResult == FRAME_COMPLETE)
            {
                /* Get the received frame from the parser state */
                RawFrame_t xReceivedFrame = xParserState.rawFrame;
                /* Send the complete frame to the processing queue */
                xQueueSendResult = xQueueSend(xUartToProcessingQueue, &xReceivedFrame, 10);
                if (xQueueSendResult != pdPASS)
                {
                    /* Handle queue full error (e.g., increment error counter, set event group bit) */
                    xEventGroupSetBits(xSystemEventGroup, (1UL << 0)); /* Example bit 0 for UART queue full */
                }
                else
                {
                    /* Toggle Activity LED (PD12) */
                    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
                }
            }
            else if (eResult == FRAME_ERROR)
            {
                /* Handle frame error (e.g., timeout, invalid start marker) */
                xEventGroupSetBits(xSystemEventGroup, (1UL << 1)); /* Example bit 1 for frame error */
                frame_parser_reset(&xParserState);
            }
            /* Otherwise, continue receiving bytes */
        }
        /* Optional: add a small delay to prevent tight loop if no data */
        vTaskDelay(1);
    }
}

/*=== Telemetry Processing Task ===========================================*/
/**
 * @brief Telemetry Processing Task: Validates frames, extracts telemetry data,
 *        and sends it to the LCD display queue.
 *
 * @param pvParameters Unused task parameter.
 */
static void vTelemetryProcessingTask(void *pvParameters)
{
    (void) pvParameters;
    RawFrame_t xReceivedFrame;
    TelemetryData_t xTelemetryData;
    BaseType_t xQueueReceiveResult;
    BaseType_t xQueueSendResult;

    for (;;)
    {
        /* Wait for a frame from the UART reception queue */
        xQueueReceiveResult = xQueueReceive(xUartToProcessingQueue, &xReceivedFrame, portMAX_DELAY);
        if (xQueueReceiveResult == pdPASS)
        {
            /* Validate the frame (CRC, length, etc.) */
            if (telemetry_validate_frame(&xReceivedFrame) == TELEMETRY_STATUS_SUCCESS)
            {
                /* Extract telemetry fields */
                if (telemetry_extract_fields(&xReceivedFrame, &xTelemetryData) == TELEMETRY_STATUS_SUCCESS)
                {
                    /* Send the telemetry data to the LCD queue */
                    xQueueSendResult = xQueueSend(xProcessingToLcdQueue, &xTelemetryData, 10);
                    if (xQueueSendResult != pdPASS)
                    {
                        /* Handle queue full error */
                        xEventGroupSetBits(xSystemEventGroup, (1UL << 2)); /* Example bit 2 for LCD queue full */
                    }
                    else
                    {
                        /* Output debug information via UART (with mutex protection) */
                        if (xSemaphoreTake(xUartTxMutex, 100) == pdTRUE)
                        {
                            debug_uart_printf("Telemetry: Timestamp=%lu, Temp=%.2fC, Voltage=%.2fV, Status=%u\r\n",
                                              xTelemetryData.timestamp,
                                              xTelemetryData.temperature,
                                              xTelemetryData.voltage,
                                              xTelemetryData.status);
                            xSemaphoreGive(xUartTxMutex);
                        }
                    }
                }
                else
                {
                    /* Field extraction failed */
                    xEventGroupSetBits(xSystemEventGroup, (1UL << 3)); /* Example bit 3 for extraction failure */
                }
            }
            else
            {
                /* CRC or validation failed */
                xEventGroupSetBits(xSystemEventGroup, (1UL << 4)); /* Example bit 4 for CRC error */
                /* Increment error counter (could be stored in a global stats structure) */
                /* Toggle Error LED (PD14) */
                HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
            }
        }
    }
}

/*=== LCD Display Task ====================================================*/
/**
 * @brief LCD Display Task: Receives telemetry data and updates the LCD display.
 *
 * @note The LCD driver is currently stubbed to output via debug UART.
 *
 * @param pvParameters Unused task parameter.
 */
static void vLcdDisplayTask(void *pvParameters)
{
    (void) pvParameters;
    TelemetryData_t xTelemetryData;
    BaseType_t xQueueReceiveResult;
    char cDisplayBuffer[32];

    for (;;)
    {
        /* Wait for telemetry data from the processing queue */
        xQueueReceiveResult = xQueueReceive(xProcessingToLcdQueue, &xTelemetryData, portMAX_DELAY);
        if (xQueueReceiveResult == pdPASS)
        {
            /* Format the telemetry data for display (using helper functions) */
            /* For now, we'll use the debug UART output as per the LCD driver stub */
            /* In a real implementation, we would call lcd_driver functions */
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print_string("Temp: ");
            /* Convert float to string with 2 decimal places */
            snprintf(cDisplayBuffer, sizeof(cDisplayBuffer), "%.2fC", xTelemetryData.temperature);
            lcd_print_string(cDisplayBuffer);

            lcd_set_cursor(0, 1);
            lcd_print_string("Volt: ");
            snprintf(cDisplayBuffer, sizeof(cDisplayBuffer), "%.2fV", xTelemetryData.voltage);
            lcd_print_string(cDisplayBuffer);

            /* Update display at <=5 Hz (delay 200ms) */
            vTaskDelay(200);
        }
    }
}

/*=== System Monitor Task =================================================*/
/**
 * @brief System Monitor Task: Reports system health, stack usage, and error counters.
 *
 * @param pvParameters Unused task parameter.
 */
static void vSystemMonitorTask(void *pvParameters)
{
    (void) pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); /* 1 second */
    UBaseType_t uxHighWaterMark;

    /* Initialize the last wake time */
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /* Wait until the next cycle */
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /* Check stack high water marks for each task */
        if (xUartRxTaskHandle != NULL)
        {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(xUartRxTaskHandle);
            debug_uart_printf("UART_RX Task High Water Mark: %lu\r\n", (unsigned long)uxHighWaterMark);
        }
        if (xTelemetryProcTaskHandle != NULL)
        {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(xTelemetryProcTaskHandle);
            debug_uart_printf("TELEMETRY_PROC Task High Water Mark: %lu\r\n", (unsigned long)uxHighWaterMark);
        }
        if (xLcdDisplayTaskHandle != NULL)
        {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(xLcdDisplayTaskHandle);
            debug_uart_printf("LCD_DISPLAY Task High Water Mark: %lu\r\n", (unsigned long)uxHighWaterMark);
        }
        if (xSystemMonitorTaskHandle != NULL)
        {
            uxHighWaterMark = uxTaskGetStackHighWaterMark(xSystemMonitorTaskHandle);
            debug_uart_printf("SYSTEM_MONITOR Task High Water Mark: %lu\r\n", (unsigned long)uxHighWaterMark);
        }

        /* Report packet and error counters (if available) */
        /* We would typically have a global stats structure, but for now we output placeholder */
        debug_uart_printf("System Monitor: Heartbeat\r\n");

        /* Toggle Power LED (PD15) as a heartbeat */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    }
}