/*******************************************************************************
 * @file    app_tasks.c
 * @brief   FreeRTOS Application Tasks - Implementation
 * @details Implements a realistic multi-task sensor data acquisition system
 *          demonstrating FreeRTOS concepts: tasks, queues, semaphores, mutexes,
 *          software timers, and event groups.
 *
 *          Task Architecture:
 *          - Sensor_Task    (Pri 4, 100ms)  : ADC+DMA read, queue send
 *          - Processing_Task (Pri 3)        : Filter, alarms, queue send
 *          - Display_Task   (Pri 2, 500ms)  : UART output, mutex protected
 *          - Logger_Task    (Pri 1)         : Event logging, event group wait
 *          - Monitor_Task   (Pri 0, 1000ms) : Stack/heap/CPU stats
 *          - LED_Timer      (SW Timer)      : Heartbeat LED, state-based pattern
 *
 * @author  Embedded Systems Portfolio Project
 ******************************************************************************/

/*===========================================================================*/
/*                        INCLUDE FILES                                      */
/*===========================================================================*/

#include "app_tasks.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/*===========================================================================*/
/*                    EXTERNAL PERIPHERAL HANDLES                            */
/*===========================================================================*/

/* These handles are defined in main.c and used here for hardware access */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_adc1;

/*===========================================================================*/
/*                    TASK HANDLE DEFINITIONS                                */
/*===========================================================================*/

TaskHandle_t xSensorTaskHandle = NULL;
TaskHandle_t xProcessingTaskHandle = NULL;
TaskHandle_t xDisplayTaskHandle = NULL;
TaskHandle_t xLoggerTaskHandle = NULL;
TaskHandle_t xMonitorTaskHandle = NULL;

/*===========================================================================*/
/*                    IPC OBJECT DEFINITIONS                                 */
/*===========================================================================*/

QueueHandle_t xSensorDataQueue = NULL;       /* Sensor -> Processing       */
QueueHandle_t xDisplayQueue = NULL;          /* Processing -> Display      */
SemaphoreHandle_t xDmaSemaphore = NULL;      /* DMA completion sync        */
SemaphoreHandle_t xUartMutex = NULL;         /* UART access protection     */
TimerHandle_t xLedTimer = NULL;              /* Heartbeat LED timer        */
EventGroupHandle_t xSystemEventGroup = NULL; /* System-wide event flags    */

/*===========================================================================*/
/*                    SHARED STATE VARIABLES                                 */
/*===========================================================================*/

volatile SystemState_t eCurrentSystemState = SYS_STATE_INIT;
volatile uint32_t ulIdleCounter = 0;

/* ADC DMA buffer - written by DMA, read by Sensor_Task */
volatile uint16_t usADC_DMA_Buffer[ADC_NUM_CHANNELS] = {0};

/*===========================================================================*/
/*                    PRIVATE (FILE-SCOPE) VARIABLES                         */
/*===========================================================================*/

/* Moving average filter buffers for temperature and light channels */
static uint16_t usTempFilterBuffer[FILTER_WINDOW_SIZE] = {0};
static uint16_t usLightFilterBuffer[FILTER_WINDOW_SIZE] = {0};
static uint8_t ucFilterIndex = 0;
static bool bFilterBufferFull = false;

/* Sensor data sequence counter */
static uint32_t ulSensorSequence = 0;

/* Previous alarm states for edge detection in logger */
static bool bPrevTempAlarm = false;
static bool bPrevLightAlarm = false;

/* LED timer toggle counter for pattern generation */
static uint32_t ulLedToggleCount = 0;

/* Transmit buffer for UART messages (shared via mutex) */
static char pcUartTxBuffer[256];

/*===========================================================================*/
/*                    PRIVATE FUNCTION PROTOTYPES                            */
/*===========================================================================*/

static uint16_t prvApplyMovingAverage(uint16_t *pusBuffer, uint16_t usNewSample,
                                      uint8_t ucIndex);
static float prvAdcToVoltage(uint16_t usAdcValue);
static void prvFormatUptime(uint32_t ulTicks, char *pcBuffer, size_t xBufSize);
static const char *prvGetSeverityString(LogSeverity_t eSeverity);

/*===========================================================================*/
/*                    INITIALIZATION                                         */
/*===========================================================================*/

/**
 * @brief Create all FreeRTOS objects for the application.
 *
 * Creates tasks, queues, semaphores, mutexes, event groups, and timers
 * in a specific order. Returns pdFAIL if any creation fails.
 */
BaseType_t xAppTasksInit(void)
{
    BaseType_t xResult = pdPASS;

    /*-----------------------------------------------------------------------*/
    /* Step 1: Create IPC Objects (before tasks, so tasks can use them)       */
    /*-----------------------------------------------------------------------*/

    /* Binary semaphore for ADC DMA completion synchronization.
     * Created in "empty" state - Sensor_Task will block until DMA ISR gives it. */
    xDmaSemaphore = xSemaphoreCreateBinary();
    if (xDmaSemaphore == NULL)
    {
        return pdFAIL;
    }

    /* Mutex for protecting shared UART2 access between Display and Logger tasks.
     * Created in "available" state (unlike binary semaphore). */
    xUartMutex = xSemaphoreCreateMutex();
    if (xUartMutex == NULL)
    {
        return pdFAIL;
    }

    /* Queue for raw sensor data: Sensor_Task -> Processing_Task.
     * Depth of 10 allows Sensor_Task to run ahead if Processing_Task is busy. */
    xSensorDataQueue = xQueueCreate(SENSOR_DATA_QUEUE_SIZE, sizeof(SensorData_t));
    if (xSensorDataQueue == NULL)
    {
        return pdFAIL;
    }

    /* Queue for processed data: Processing_Task -> Display_Task.
     * Depth of 5 is sufficient since Display_Task runs at 500ms. */
    xDisplayQueue = xQueueCreate(DISPLAY_QUEUE_SIZE, sizeof(ProcessedData_t));
    if (xDisplayQueue == NULL)
    {
        return pdFAIL;
    }

    /* Event group for system-wide alarm and status signaling.
     * Bits are defined in app_tasks.h (EVT_TEMP_HIGH_ALARM_BIT, etc.). */
    xSystemEventGroup = xEventGroupCreate();
    if (xSystemEventGroup == NULL)
    {
        return pdFAIL;
    }

    /*-----------------------------------------------------------------------*/
    /* Step 2: Create Application Tasks                                      */
    /*-----------------------------------------------------------------------*/

    /* Sensor_Task: Highest priority - must not miss ADC sampling deadlines */
    xResult = xTaskCreate(
        vSensorTask,            /* Task function                     */
        "Sensor",               /* Human-readable name (debug)       */
        SENSOR_TASK_STACK_SIZE, /* Stack depth in words              */
        NULL,                   /* Task parameter (unused)           */
        SENSOR_TASK_PRIORITY,   /* Priority (4 = highest)            */
        &xSensorTaskHandle      /* Output: task handle               */
    );
    if (xResult != pdPASS)
        return pdFAIL;

    /* Processing_Task: Above normal - data must be processed promptly */
    xResult = xTaskCreate(
        vProcessingTask,
        "Process",
        PROCESSING_TASK_STACK_SIZE,
        NULL,
        PROCESSING_TASK_PRIORITY,
        &xProcessingTaskHandle);
    if (xResult != pdPASS)
        return pdFAIL;

    /* Display_Task: Normal priority - human-visible output can tolerate delay */
    xResult = xTaskCreate(
        vDisplayTask,
        "Display",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        DISPLAY_TASK_PRIORITY,
        &xDisplayTaskHandle);
    if (xResult != pdPASS)
        return pdFAIL;

    /* Logger_Task: Below normal - logging is important but not time-critical */
    xResult = xTaskCreate(
        vLoggerTask,
        "Logger",
        LOGGER_TASK_STACK_SIZE,
        NULL,
        LOGGER_TASK_PRIORITY,
        &xLoggerTaskHandle);
    if (xResult != pdPASS)
        return pdFAIL;

    /* Monitor_Task: Lowest priority - runs only when nothing else needs CPU */
    xResult = xTaskCreate(
        vMonitorTask,
        "Monitor",
        MONITOR_TASK_STACK_SIZE,
        NULL,
        MONITOR_TASK_PRIORITY,
        &xMonitorTaskHandle);
    if (xResult != pdPASS)
        return pdFAIL;

    /*-----------------------------------------------------------------------*/
    /* Step 3: Create Software Timers                                        */
    /*-----------------------------------------------------------------------*/

    /* LED heartbeat timer: auto-reload timer for periodic LED toggle */
    xLedTimer = xTimerCreate(
        "LEDTimer",                         /* Timer name (debug)        */
        pdMS_TO_TICKS(LED_TIMER_PERIOD_MS), /* Period: 500ms             */
        pdTRUE,                             /* Auto-reload: yes          */
        (void *)0,                          /* Timer ID (unused)         */
        vLedTimerCallback                   /* Callback function         */
    );
    if (xLedTimer == NULL)
    {
        return pdFAIL;
    }

    /* Start the LED timer. It won't actually fire until the scheduler starts. */
    xResult = xTimerStart(xLedTimer, 0);
    if (xResult != pdPASS)
    {
        return pdFAIL;
    }

    /* Update system state to indicate initialization is complete */
    eCurrentSystemState = SYS_STATE_NORMAL;

    return pdPASS;
}

/*===========================================================================*/
/*                    TASK IMPLEMENTATIONS                                   */
/*===========================================================================*/

/**
 * @brief Sensor data acquisition task (Priority 4, 100ms period).
 *
 * This task performs the following in each cycle:
 * 1. Triggers an ADC conversion with DMA
 * 2. Waits for DMA completion via binary semaphore (given by DMA ISR)
 * 3. Reads the DMA buffer values
 * 4. Packages data into SensorData_t structure
 * 5. Sends the data to the processing queue
 *
 * Uses vTaskDelayUntil() for precise periodic timing, ensuring consistent
 * sampling regardless of task execution time variations.
 */
void vSensorTask(void *pvParameters)
{
    (void)pvParameters; /* Suppress unused parameter warning */

    TickType_t xLastWakeTime;
    SensorData_t xSensorData;

    /* Initialize the xLastWakeTime variable with the current time.
     * This is the reference point for vTaskDelayUntil(). */
    xLastWakeTime = xTaskGetTickCount();

    /* Log task startup */
    vLogEvent(LOG_INFO, "Sensor_Task started");

    /* Infinite task loop - FreeRTOS tasks must never return */
    for (;;)
    {
        /*-------------------------------------------------------------------*/
        /* Step 1: Start ADC conversion with DMA                             */
        /*-------------------------------------------------------------------*/

        /* Start ADC conversion in DMA mode. The DMA controller will
         * transfer results to usADC_DMA_Buffer[] and trigger an interrupt
         * upon completion. */
        HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)usADC_DMA_Buffer,
                          ADC_NUM_CHANNELS);

        /*-------------------------------------------------------------------*/
        /* Step 2: Wait for DMA completion (binary semaphore)                */
        /*-------------------------------------------------------------------*/

        /* Block until the DMA ISR gives the semaphore (max wait: 50ms).
         * If DMA doesn't complete in time, we handle the timeout as an error. */
        if (xSemaphoreTake(xDmaSemaphore, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            /*---------------------------------------------------------------*/
            /* Step 3: Package raw sensor data                               */
            /*---------------------------------------------------------------*/

            xSensorData.temperature_raw = usADC_DMA_Buffer[ADC_CHANNEL_TEMPERATURE];
            xSensorData.light_raw = usADC_DMA_Buffer[ADC_CHANNEL_LIGHT];
            xSensorData.timestamp = xTaskGetTickCount();
            xSensorData.sequence_number = ulSensorSequence++;

            /*---------------------------------------------------------------*/
            /* Step 4: Send to processing queue                              */
            /*---------------------------------------------------------------*/

            /* Send data to queue. If queue is full, wait up to 10ms.
             * A full queue indicates Processing_Task is overloaded. */
            if (xQueueSend(xSensorDataQueue, &xSensorData,
                           pdMS_TO_TICKS(10)) != pdPASS)
            {
                /* Queue full - data is dropped. This is a design decision:
                 * we prefer to drop old data rather than block the sensor. */
                xEventGroupSetBits(xSystemEventGroup, EVT_SYSTEM_ERROR_BIT);
            }
        }
        else
        {
            /* DMA timeout - ADC/DMA hardware may have an issue */
            xEventGroupSetBits(xSystemEventGroup, EVT_SENSOR_TIMEOUT_BIT);
            vLogEvent(LOG_ERROR, "Sensor DMA timeout");

            /* Stop the ADC to reset the DMA state */
            HAL_ADC_Stop_DMA(&hadc1);
        }

        /*-------------------------------------------------------------------*/
        /* Step 5: Wait for next period                                      */
        /*-------------------------------------------------------------------*/

        /* vTaskDelayUntil provides precise periodic timing.
         * Unlike vTaskDelay(), it compensates for execution time, ensuring
         * the task runs exactly every SENSOR_TASK_PERIOD_MS milliseconds. */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_TASK_PERIOD_MS));
    }
}

/**
 * @brief Data processing and filtering task (Priority 3).
 *
 * Receives raw sensor data from the queue and applies:
 * 1. Moving average filter (window size = FILTER_WINDOW_SIZE)
 * 2. ADC-to-voltage conversion (12-bit ADC, 3.3V reference)
 * 3. Threshold-based alarm detection
 * 4. Event group bit setting for alarm conditions
 *
 * This task blocks on xQueueReceive() and runs whenever new data arrives.
 */
void vProcessingTask(void *pvParameters)
{
    (void)pvParameters;

    SensorData_t xRawData;
    ProcessedData_t xProcessedData;

    vLogEvent(LOG_INFO, "Processing_Task started");

    for (;;)
    {
        /*-------------------------------------------------------------------*/
        /* Step 1: Wait for raw data from sensor queue                       */
        /*-------------------------------------------------------------------*/

        /* Block indefinitely until data is available.
         * This is efficient - the task consumes zero CPU while waiting. */
        if (xQueueReceive(xSensorDataQueue, &xRawData, portMAX_DELAY) == pdTRUE)
        {
            /*---------------------------------------------------------------*/
            /* Step 2: Apply moving average filter                           */
            /*---------------------------------------------------------------*/

            /* Store new samples in circular buffer */
            usTempFilterBuffer[ucFilterIndex] = xRawData.temperature_raw;
            usLightFilterBuffer[ucFilterIndex] = xRawData.light_raw;

            /* Calculate filtered values */
            xProcessedData.temperature_filtered =
                prvApplyMovingAverage(usTempFilterBuffer,
                                      xRawData.temperature_raw,
                                      ucFilterIndex);

            xProcessedData.light_filtered =
                prvApplyMovingAverage(usLightFilterBuffer,
                                      xRawData.light_raw,
                                      ucFilterIndex);

            /* Advance circular buffer index */
            ucFilterIndex++;
            if (ucFilterIndex >= FILTER_WINDOW_SIZE)
            {
                ucFilterIndex = 0;
                bFilterBufferFull = true;
            }

            /*---------------------------------------------------------------*/
            /* Step 3: Convert to voltage and copy raw values                */
            /*---------------------------------------------------------------*/

            xProcessedData.temperature_raw = xRawData.temperature_raw;
            xProcessedData.light_raw = xRawData.light_raw;
            xProcessedData.temperature_voltage = prvAdcToVoltage(xProcessedData.temperature_filtered);
            xProcessedData.light_voltage = prvAdcToVoltage(xProcessedData.light_filtered);
            xProcessedData.timestamp = xTaskGetTickCount();
            xProcessedData.sequence_number = xRawData.sequence_number;

            /*---------------------------------------------------------------*/
            /* Step 4: Check alarm thresholds                                */
            /*---------------------------------------------------------------*/

            /* Temperature alarm check */
            if (xProcessedData.temperature_filtered > TEMP_HIGH_THRESHOLD ||
                xProcessedData.temperature_filtered < TEMP_LOW_THRESHOLD)
            {
                xProcessedData.temp_alarm_active = true;

                /* Set the appropriate alarm bit in the event group */
                if (xProcessedData.temperature_filtered > TEMP_HIGH_THRESHOLD)
                {
                    xEventGroupSetBits(xSystemEventGroup, EVT_TEMP_HIGH_ALARM_BIT);
                }
                else
                {
                    xEventGroupSetBits(xSystemEventGroup, EVT_TEMP_LOW_ALARM_BIT);
                }

                /* Turn on temperature alarm LED (PD13 - Orange) */
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

                /* Update system state */
                if (eCurrentSystemState < SYS_STATE_WARNING)
                {
                    eCurrentSystemState = SYS_STATE_WARNING;
                }
            }
            else
            {
                xProcessedData.temp_alarm_active = false;

                /* Clear temperature alarm bits */
                xEventGroupClearBits(xSystemEventGroup, EVT_TEMP_ALARM_MASK);

                /* Turn off temperature alarm LED */
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
            }

            /* Light alarm check */
            if (xProcessedData.light_filtered > LIGHT_HIGH_THRESHOLD ||
                xProcessedData.light_filtered < LIGHT_LOW_THRESHOLD)
            {
                xProcessedData.light_alarm_active = true;

                if (xProcessedData.light_filtered > LIGHT_HIGH_THRESHOLD)
                {
                    xEventGroupSetBits(xSystemEventGroup, EVT_LIGHT_HIGH_ALARM_BIT);
                }
                else
                {
                    xEventGroupSetBits(xSystemEventGroup, EVT_LIGHT_LOW_ALARM_BIT);
                }

                /* Turn on light alarm LED (PD14 - Red) */
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

                if (eCurrentSystemState < SYS_STATE_WARNING)
                {
                    eCurrentSystemState = SYS_STATE_WARNING;
                }
            }
            else
            {
                xProcessedData.light_alarm_active = false;
                xEventGroupClearBits(xSystemEventGroup, EVT_LIGHT_ALARM_MASK);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
            }

            /* Update system state based on combined alarm status */
            if (!xProcessedData.temp_alarm_active && !xProcessedData.light_alarm_active)
            {
                eCurrentSystemState = SYS_STATE_NORMAL;
            }

            /* Set data ready bit for any task waiting on processed data */
            xEventGroupSetBits(xSystemEventGroup, EVT_DATA_READY_BIT);

            /*---------------------------------------------------------------*/
            /* Step 5: Send processed data to display queue                  */
            /*---------------------------------------------------------------*/

            /* Non-blocking send: if display queue is full, drop this update.
             * Display updates are not critical - losing one is acceptable. */
            if (xQueueSend(xDisplayQueue, &xProcessedData, 0) != pdPASS)
            {
                /* Display queue full - display task is behind.
                 * This is normal if display period is much longer than
                 * sensor period (500ms vs 100ms). */
            }
        }
    }
}

/**
 * @brief Display output task via UART (Priority 2, 500ms period).
 *
 * Formats processed sensor data into a human-readable table and transmits
 * via UART2. Uses a mutex to prevent output corruption when the Logger_Task
 * also writes to UART.
 */
void vDisplayTask(void *pvParameters)
{
    (void)pvParameters;

    ProcessedData_t xDisplayData;
    TickType_t xLastWakeTime;
    char pcTimeStr[16];
    int iLen;

    xLastWakeTime = xTaskGetTickCount();

    vLogEvent(LOG_INFO, "Display_Task started");

    for (;;)
    {
        /*-------------------------------------------------------------------*/
        /* Step 1: Check for processed data in the queue                     */
        /*-------------------------------------------------------------------*/

        /* Try to receive the latest processed data.
         * Non-blocking: if no data, we display "no data" message. */
        if (xQueueReceive(xDisplayQueue, &xDisplayData, 0) == pdTRUE)
        {
            /*---------------------------------------------------------------*/
            /* Step 2: Format the display output                             */
            /*---------------------------------------------------------------*/

            prvFormatUptime(xDisplayData.timestamp, pcTimeStr, sizeof(pcTimeStr));

            iLen = snprintf(pcUartTxBuffer, sizeof(pcUartTxBuffer),
                            "\r\n"
                            "--- Sensor Data [%s] Seq#%lu ---\r\n"
                            "Temperature: Raw=%4u  Filt=%4u  %.2fV %s\r\n"
                            "Light:       Raw=%4u  Filt=%4u  %.2fV %s\r\n"
                            "System State: %s\r\n",
                            pcTimeStr,
                            (unsigned long)xDisplayData.sequence_number,
                            xDisplayData.temperature_raw,
                            xDisplayData.temperature_filtered,
                            (double)xDisplayData.temperature_voltage,
                            xDisplayData.temp_alarm_active ? "[ALARM]" : "[OK]",
                            xDisplayData.light_raw,
                            xDisplayData.light_filtered,
                            (double)xDisplayData.light_voltage,
                            xDisplayData.light_alarm_active ? "[ALARM]" : "[OK]",
                            (eCurrentSystemState == SYS_STATE_NORMAL) ? "NORMAL" : (eCurrentSystemState == SYS_STATE_WARNING) ? "WARNING"
                                                                               : (eCurrentSystemState == SYS_STATE_ERROR)     ? "ERROR"
                                                                                                                              : "INIT");

            /*---------------------------------------------------------------*/
            /* Step 3: Transmit via UART with mutex protection               */
            /*---------------------------------------------------------------*/

            /* Acquire the UART mutex before transmitting.
             * Timeout of 100ms prevents indefinite blocking. */
            if (xSemaphoreTake(xUartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                HAL_UART_Transmit(&huart2, (uint8_t *)pcUartTxBuffer,
                                  (uint16_t)iLen, HAL_MAX_DELAY);

                /* ALWAYS release the mutex after use */
                xSemaphoreGive(xUartMutex);
            }
            /* If mutex not acquired, this display cycle is skipped.
             * This prevents deadlock if Logger_Task holds the mutex too long. */
        }

        /*-------------------------------------------------------------------*/
        /* Step 4: Wait for next display period                              */
        /*-------------------------------------------------------------------*/

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(DISPLAY_TASK_PERIOD_MS));
    }
}

/**
 * @brief Event logging task (Priority 1).
 *
 * Monitors the system event group for alarm condition changes and logs
 * transitions. Detects both alarm onset and clearance by tracking
 * previous alarm states.
 *
 * Uses xEventGroupWaitBits() to efficiently block until any alarm bit
 * is set, rather than polling.
 */
void vLoggerTask(void *pvParameters)
{
    (void)pvParameters;

    EventBits_t uxBits;
    bool bCurrentTempAlarm;
    bool bCurrentLightAlarm;

    vLogEvent(LOG_INFO, "Logger_Task started");

    for (;;)
    {
        /*-------------------------------------------------------------------*/
        /* Step 1: Wait for any alarm or status event                        */
        /*-------------------------------------------------------------------*/

        /* Wait for any alarm bit to be set, OR timeout after 500ms.
         * The timeout allows periodic status checks even without events.
         * pdFALSE = don't clear bits on read (we'll clear manually).
         * pdFALSE = OR logic (any single bit triggers wake-up). */
        uxBits = xEventGroupWaitBits(
            xSystemEventGroup,
            EVT_ALL_ALARM_BITS | EVT_DATA_READY_BIT,
            pdFALSE,           /* Don't auto-clear bits       */
            pdFALSE,           /* Wait for ANY bit (OR)       */
            pdMS_TO_TICKS(500) /* Timeout: 500ms              */
        );

        /*-------------------------------------------------------------------*/
        /* Step 2: Check for temperature alarm transitions                   */
        /*-------------------------------------------------------------------*/

        bCurrentTempAlarm = (uxBits & EVT_TEMP_ALARM_MASK) != 0;

        /* Rising edge: alarm just activated */
        if (bCurrentTempAlarm && !bPrevTempAlarm)
        {
            if (uxBits & EVT_TEMP_HIGH_ALARM_BIT)
            {
                vLogEvent(LOG_WARNING, "TEMP ALARM: High threshold exceeded");
            }
            else
            {
                vLogEvent(LOG_WARNING, "TEMP ALARM: Low threshold exceeded");
            }
        }
        /* Falling edge: alarm just cleared */
        else if (!bCurrentTempAlarm && bPrevTempAlarm)
        {
            vLogEvent(LOG_INFO, "TEMP ALARM: Cleared - back to normal");
        }

        bPrevTempAlarm = bCurrentTempAlarm;

        /*-------------------------------------------------------------------*/
        /* Step 3: Check for light alarm transitions                         */
        /*-------------------------------------------------------------------*/

        bCurrentLightAlarm = (uxBits & EVT_LIGHT_ALARM_MASK) != 0;

        if (bCurrentLightAlarm && !bPrevLightAlarm)
        {
            if (uxBits & EVT_LIGHT_HIGH_ALARM_BIT)
            {
                vLogEvent(LOG_WARNING, "LIGHT ALARM: High threshold exceeded");
            }
            else
            {
                vLogEvent(LOG_WARNING, "LIGHT ALARM: Low light detected");
            }
        }
        else if (!bCurrentLightAlarm && bPrevLightAlarm)
        {
            vLogEvent(LOG_INFO, "LIGHT ALARM: Cleared - back to normal");
        }

        bPrevLightAlarm = bCurrentLightAlarm;

        /*-------------------------------------------------------------------*/
        /* Step 4: Check for system error events                             */
        /*-------------------------------------------------------------------*/

        if (uxBits & EVT_SYSTEM_ERROR_BIT)
        {
            vLogEvent(LOG_ERROR, "SYSTEM: Queue overflow detected");
            /* Clear the error bit after logging */
            xEventGroupClearBits(xSystemEventGroup, EVT_SYSTEM_ERROR_BIT);
        }

        if (uxBits & EVT_SENSOR_TIMEOUT_BIT)
        {
            vLogEvent(LOG_ERROR, "SYSTEM: Sensor DMA timeout detected");
            xEventGroupClearBits(xSystemEventGroup, EVT_SENSOR_TIMEOUT_BIT);
        }

        /*-------------------------------------------------------------------*/
        /* Step 5: Clear the data ready bit                                  */
        /*-------------------------------------------------------------------*/

        if (uxBits & EVT_DATA_READY_BIT)
        {
            xEventGroupClearBits(xSystemEventGroup, EVT_DATA_READY_BIT);
        }
    }
}

/**
 * @brief System health monitoring task (Priority 0, 1000ms period).
 *
 * Reports comprehensive system health information via UART:
 * - Stack high water marks for all tasks (overflow risk detection)
 * - Heap usage and minimum ever free heap
 * - CPU utilization per task (via runtime stats)
 * - Current alarm states
 * - System uptime
 *
 * Runs at the lowest priority, so it only gets CPU time when no other
 * task needs it. This is appropriate since monitoring is non-critical.
 */
void vMonitorTask(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime;
    SystemStatus_t xStatus;
    char pcTimeStr[16];
    int iLen;

    /* Buffer for runtime stats - needs to be large enough for all tasks */
    static char pcStatsBuffer[512];

    xLastWakeTime = xTaskGetTickCount();

    vLogEvent(LOG_INFO, "Monitor_Task started");

    for (;;)
    {
        /*-------------------------------------------------------------------*/
        /* Step 1: Collect stack high water marks                             */
        /*-------------------------------------------------------------------*/

        /* uxTaskGetStackHighWaterMark returns the minimum number of words
         * that have ever been free on the task's stack. A value approaching
         * zero indicates the task is close to overflowing its stack. */
        if (xSensorTaskHandle != NULL)
        {
            xStatus.sensor_stack_hwm = uxTaskGetStackHighWaterMark(xSensorTaskHandle);
        }
        if (xProcessingTaskHandle != NULL)
        {
            xStatus.processing_stack_hwm = uxTaskGetStackHighWaterMark(xProcessingTaskHandle);
        }
        if (xDisplayTaskHandle != NULL)
        {
            xStatus.display_stack_hwm = uxTaskGetStackHighWaterMark(xDisplayTaskHandle);
        }
        if (xLoggerTaskHandle != NULL)
        {
            xStatus.logger_stack_hwm = uxTaskGetStackHighWaterMark(xLoggerTaskHandle);
        }
        if (xMonitorTaskHandle != NULL)
        {
            xStatus.monitor_stack_hwm = uxTaskGetStackHighWaterMark(xMonitorTaskHandle);
        }

        /*-------------------------------------------------------------------*/
        /* Step 2: Collect heap usage                                         */
        /*-------------------------------------------------------------------*/

        xStatus.heap_free = xPortGetFreeHeapSize();
        xStatus.heap_min_free = xPortGetMinimumEverFreeHeapSize();

        /*-------------------------------------------------------------------*/
        /* Step 3: Collect system information                                 */
        /*-------------------------------------------------------------------*/

        xStatus.uptime_seconds = xTaskGetTickCount() / configTICK_RATE_HZ;
        xStatus.total_tasks = uxTaskGetNumberOfTasks();
        xStatus.temp_alarm = bPrevTempAlarm;
        xStatus.light_alarm = bPrevLightAlarm;
        xStatus.system_error = (eCurrentSystemState == SYS_STATE_ERROR);

        /*-------------------------------------------------------------------*/
        /* Step 4: Format and transmit the monitoring report                  */
        /*-------------------------------------------------------------------*/

        prvFormatUptime(xTaskGetTickCount(), pcTimeStr, sizeof(pcTimeStr));

        /* Build the monitoring report string */
        iLen = snprintf(pcUartTxBuffer, sizeof(pcUartTxBuffer),
                        "\r\n"
                        "========== SYSTEM MONITOR ==========\r\n"
                        "[Uptime: %s] [Tasks: %lu]\r\n"
                        "\r\n"
                        "--- Stack Usage (words remaining) ---\r\n"
                        "  Sensor:     %3lu / %d\r\n"
                        "  Processing: %3lu / %d\r\n"
                        "  Display:    %3lu / %d\r\n"
                        "  Logger:     %3lu / %d\r\n"
                        "  Monitor:    %3lu / %d\r\n"
                        "\r\n"
                        "--- Heap Usage ---\r\n"
                        "  Total:    %lu bytes\r\n"
                        "  Free:     %lu bytes (%lu%%)\r\n"
                        "  Min Free: %lu bytes\r\n"
                        "\r\n"
                        "--- Alarms ---\r\n"
                        "  Temperature: %s\r\n"
                        "  Light:       %s\r\n"
                        "  System:      %s\r\n"
                        "=====================================\r\n",
                        pcTimeStr,
                        (unsigned long)xStatus.total_tasks,
                        (unsigned long)xStatus.sensor_stack_hwm, SENSOR_TASK_STACK_SIZE,
                        (unsigned long)xStatus.processing_stack_hwm, PROCESSING_TASK_STACK_SIZE,
                        (unsigned long)xStatus.display_stack_hwm, DISPLAY_TASK_STACK_SIZE,
                        (unsigned long)xStatus.logger_stack_hwm, LOGGER_TASK_STACK_SIZE,
                        (unsigned long)xStatus.monitor_stack_hwm, MONITOR_TASK_STACK_SIZE,
                        (unsigned long)configTOTAL_HEAP_SIZE,
                        (unsigned long)xStatus.heap_free,
                        (unsigned long)((xStatus.heap_free * 100) / configTOTAL_HEAP_SIZE),
                        (unsigned long)xStatus.heap_min_free,
                        xStatus.temp_alarm ? "ALARM" : "NORMAL",
                        xStatus.light_alarm ? "ALARM" : "NORMAL",
                        xStatus.system_error ? "ERROR" : "OK");

        /* Transmit with mutex protection */
        if (xSemaphoreTake(xUartMutex, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)pcUartTxBuffer,
                              (uint16_t)iLen, HAL_MAX_DELAY);

            /*---------------------------------------------------------------*/
            /* Step 5: Print runtime statistics (CPU usage per task)          */
            /*---------------------------------------------------------------*/

            /* Generate runtime statistics table.
             * Output format: TaskName\t\tRunTime\t\tPercentage\r\n */
            vTaskGetRunTimeStats(pcStatsBuffer);

            iLen = snprintf(pcUartTxBuffer, sizeof(pcUartTxBuffer),
                            "\r\n--- CPU Usage ---\r\n"
                            "Task            Abs Time     %%Time\r\n"
                            "----------------------------------\r\n"
                            "%s"
                            "----------------------------------\r\n",
                            pcStatsBuffer);

            HAL_UART_Transmit(&huart2, (uint8_t *)pcUartTxBuffer,
                              (uint16_t)iLen, HAL_MAX_DELAY);

            xSemaphoreGive(xUartMutex);
        }

        /*-------------------------------------------------------------------*/
        /* Step 6: Wait for next monitoring period                            */
        /*-------------------------------------------------------------------*/

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MONITOR_TASK_PERIOD_MS));
    }
}

/*===========================================================================*/
/*                    SOFTWARE TIMER CALLBACK                                */
/*===========================================================================*/

/**
 * @brief LED heartbeat timer callback.
 *
 * Called automatically by the timer service task at the configured period.
 * The LED blink pattern indicates the system state:
 *
 * - NORMAL:  Steady toggle every 500ms (1 Hz blink)
 * - WARNING: Fast toggle every 250ms (by using a counter-based pattern)
 * - ERROR:   Rapid double-blink pattern
 *
 * IMPORTANT: Timer callbacks execute in the context of the timer service
 * task. They must be short and must not call blocking API functions.
 */
void vLedTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    ulLedToggleCount++;

    switch (eCurrentSystemState)
    {
    case SYS_STATE_NORMAL:
        /* Steady heartbeat: toggle every callback (500ms period) */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

        /* System status LED (blue) solid ON during normal operation */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
        break;

    case SYS_STATE_WARNING:
        /* Fast blink: toggle every callback but timer period is shorter
         * perception. We toggle the heartbeat LED every callback. */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

        /* Blue LED blinks in warning state */
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        break;

    case SYS_STATE_ERROR:
        /* Rapid double-blink pattern using counter:
         * ON-OFF-ON-OFF-OFF-OFF (repeating every 6 callbacks)  */
        if ((ulLedToggleCount % 6) < 4)
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        }

        /* Blue LED OFF during error state */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
        break;

    case SYS_STATE_INIT:
    default:
        /* During init: all LEDs off, blue LED blinks slowly */
        if (ulLedToggleCount % 4 == 0)
        {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
        }
        break;
    }
}

/*===========================================================================*/
/*                    UTILITY FUNCTIONS                                       */
/*===========================================================================*/

/**
 * @brief Send a string via UART with mutex protection.
 *
 * Thread-safe UART transmission. Acquires the UART mutex, transmits
 * the message, then releases the mutex. If the mutex cannot be acquired
 * within the specified timeout, the message is silently dropped.
 *
 * @param pcMessage  Null-terminated string to transmit
 * @param xTimeout   Maximum ticks to wait for the UART mutex
 */
void vUartPrintProtected(const char *pcMessage, TickType_t xTimeout)
{
    if (pcMessage == NULL)
    {
        return;
    }

    uint16_t usLen = (uint16_t)strlen(pcMessage);

    if (usLen == 0)
    {
        return;
    }

    /* Attempt to acquire the UART mutex */
    if (xSemaphoreTake(xUartMutex, xTimeout) == pdTRUE)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)pcMessage, usLen, HAL_MAX_DELAY);
        xSemaphoreGive(xUartMutex);
    }
    /* If mutex not acquired within timeout, message is dropped.
     * This is preferable to blocking indefinitely. */
}

/**
 * @brief Log a system event with severity level and timestamp.
 *
 * Formats a log entry with timestamp, severity tag, and message,
 * then transmits via UART with mutex protection.
 *
 * Output format: [HH:MM:SS.mmm] [SEVERITY] message\r\n
 *
 * @param eSeverity  Log severity level
 * @param pcMessage  Log message (max 63 characters, truncated if longer)
 */
void vLogEvent(LogSeverity_t eSeverity, const char *pcMessage)
{
    char pcLogBuffer[128];
    char pcTimeStr[16];

    if (pcMessage == NULL)
    {
        return;
    }

    /* Get current timestamp and format it */
    prvFormatUptime(xTaskGetTickCount(), pcTimeStr, sizeof(pcTimeStr));

    /* Format the complete log entry */
    int iLen = snprintf(pcLogBuffer, sizeof(pcLogBuffer),
                        "[%s] [%s] %s\r\n",
                        pcTimeStr,
                        prvGetSeverityString(eSeverity),
                        pcMessage);

    /* Transmit with mutex protection (100ms timeout) */
    if (xUartMutex != NULL)
    {
        if (xSemaphoreTake(xUartMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            HAL_UART_Transmit(&huart2, (uint8_t *)pcLogBuffer,
                              (uint16_t)iLen, HAL_MAX_DELAY);
            xSemaphoreGive(xUartMutex);
        }
    }
    else
    {
        /* Mutex not yet created (during early init) - send unprotected */
        HAL_UART_Transmit(&huart2, (uint8_t *)pcLogBuffer,
                          (uint16_t)iLen, HAL_MAX_DELAY);
    }
}

/*===========================================================================*/
/*                    PRIVATE HELPER FUNCTIONS                               */
/*===========================================================================*/

/**
 * @brief Calculate moving average from a circular buffer.
 *
 * Computes the arithmetic mean of all valid samples in the buffer.
 * Before the buffer is fully populated, only filled entries are averaged.
 *
 * @param pusBuffer   Pointer to the circular buffer array
 * @param usNewSample The newest sample (already stored in buffer)
 * @param ucIndex     Current write index in the circular buffer
 * @return Filtered (averaged) value
 */
static uint16_t prvApplyMovingAverage(uint16_t *pusBuffer, uint16_t usNewSample,
                                      uint8_t ucIndex)
{
    uint32_t ulSum = 0;
    uint8_t ucCount;

    /* Determine how many valid samples are in the buffer */
    if (bFilterBufferFull)
    {
        ucCount = FILTER_WINDOW_SIZE;
    }
    else
    {
        ucCount = ucIndex + 1;
    }

    /* Sum all valid samples */
    for (uint8_t i = 0; i < ucCount; i++)
    {
        ulSum += pusBuffer[i];
    }

    /* Return the arithmetic mean */
    return (uint16_t)(ulSum / ucCount);
}

/**
 * @brief Convert a 12-bit ADC value to voltage.
 *
 * Assumes 3.3V reference voltage and 12-bit resolution (0-4095).
 * Formula: voltage = (adc_value / 4095) * 3.3
 *
 * @param usAdcValue  Raw ADC value (0-4095)
 * @return Voltage in volts (0.0 - 3.3)
 */
static float prvAdcToVoltage(uint16_t usAdcValue)
{
    return ((float)usAdcValue / 4095.0f) * 3.3f;
}

/**
 * @brief Format a tick count into a human-readable time string.
 *
 * Converts FreeRTOS tick count to HH:MM:SS.mmm format.
 *
 * @param ulTicks   Tick count to convert
 * @param pcBuffer  Output buffer (must be at least 13 bytes)
 * @param xBufSize  Size of the output buffer
 */
static void prvFormatUptime(uint32_t ulTicks, char *pcBuffer, size_t xBufSize)
{
    uint32_t ulMillis = ulTicks; /* At 1000 Hz, ticks == milliseconds */
    uint32_t ulSeconds = ulMillis / 1000;
    uint32_t ulMinutes = ulSeconds / 60;
    uint32_t ulHours = ulMinutes / 60;

    snprintf(pcBuffer, xBufSize, "%02lu:%02lu:%02lu.%03lu",
             (unsigned long)(ulHours % 100),
             (unsigned long)(ulMinutes % 60),
             (unsigned long)(ulSeconds % 60),
             (unsigned long)(ulMillis % 1000));
}

/**
 * @brief Get a string representation of a log severity level.
 *
 * @param eSeverity  Log severity enum value
 * @return Constant string pointer (e.g., "INFO", "WARNING")
 */
static const char *prvGetSeverityString(LogSeverity_t eSeverity)
{
    switch (eSeverity)
    {
    case LOG_INFO:
        return "INFO    ";
    case LOG_WARNING:
        return "WARNING ";
    case LOG_ERROR:
        return "ERROR   ";
    case LOG_CRITICAL:
        return "CRITICAL";
    default:
        return "UNKNOWN ";
    }
}

/*===========================================================================*/
/*                    DMA CALLBACK (ISR CONTEXT)                             */
/*===========================================================================*/

/**
 * @brief ADC DMA transfer complete callback.
 *
 * Called from the DMA ISR when all ADC channels have been converted and
 * transferred to the DMA buffer. Gives the binary semaphore to unblock
 * the Sensor_Task.
 *
 * IMPORTANT: This function runs in ISR context. Only "FromISR" FreeRTOS
 * API functions are allowed. The portYIELD_FROM_ISR macro triggers a
 * context switch if a higher-priority task was unblocked.
 *
 * @param hadc  Pointer to the ADC handle that triggered the callback
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        /* Give the semaphore to unblock Sensor_Task */
        xSemaphoreGiveFromISR(xDmaSemaphore, &xHigherPriorityTaskWoken);

        /* If giving the semaphore unblocked a higher-priority task,
         * request a context switch so that task runs immediately
         * after the ISR returns. */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
