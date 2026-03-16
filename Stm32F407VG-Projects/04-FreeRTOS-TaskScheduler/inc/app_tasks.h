/*******************************************************************************
 * @file    app_tasks.h
 * @brief   FreeRTOS Application Tasks - Declarations and Shared Objects
 * @details Defines all task handles, IPC object handles, data structures,
 *          event group bit definitions, and function prototypes for the
 *          multi-task sensor data acquisition system.
 *
 * @note    This header is included by both main.c and app_tasks.c
 *
 * @author  Embedded Systems Portfolio Project
 ******************************************************************************/

#ifndef APP_TASKS_H
#define APP_TASKS_H

/*===========================================================================*/
/*                        INCLUDE FILES                                      */
/*===========================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "event_groups.h"
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/*                    TASK CONFIGURATION CONSTANTS                           */
/*===========================================================================*/

/**
 * @defgroup TaskPriorities Task Priority Assignments
 * @brief Priority levels for each application task.
 *        Higher number = higher priority (0 is lowest, used by idle task).
 * @{
 */
#define SENSOR_TASK_PRIORITY 4     /**< Highest - critical data acquisition    */
#define PROCESSING_TASK_PRIORITY 3 /**< Above normal - data filtering/alarms   */
#define DISPLAY_TASK_PRIORITY 2    /**< Normal - UART display output           */
#define LOGGER_TASK_PRIORITY 1     /**< Below normal - event logging           */
#define MONITOR_TASK_PRIORITY 0    /**< Lowest - system health monitoring      */
/** @} */

/**
 * @defgroup TaskStackSizes Task Stack Sizes (in words, 1 word = 4 bytes)
 * @brief Stack allocation for each task. Sized based on local variables,
 *        function call depth, and safety margin.
 * @{
 */
#define SENSOR_TASK_STACK_SIZE 256     /**< 1024 bytes - ADC/DMA operations      */
#define PROCESSING_TASK_STACK_SIZE 512 /**< 2048 bytes - filter calculations     */
#define DISPLAY_TASK_STACK_SIZE 256    /**< 1024 bytes - UART formatting         */
#define LOGGER_TASK_STACK_SIZE 256     /**< 1024 bytes - log formatting          */
#define MONITOR_TASK_STACK_SIZE 512    /**< 2048 bytes - runtime stats strings   */
/** @} */

/**
 * @defgroup TaskPeriods Task Execution Periods (in milliseconds)
 * @{
 */
#define SENSOR_TASK_PERIOD_MS 100   /**< Sensor sampling period               */
#define DISPLAY_TASK_PERIOD_MS 500  /**< Display update period                */
#define MONITOR_TASK_PERIOD_MS 1000 /**< System monitor report period         */
/** @} */

/**
 * @defgroup QueueSizes Queue Capacities
 * @{
 */
#define SENSOR_DATA_QUEUE_SIZE 10 /**< Sensor -> Processing queue depth     */
#define DISPLAY_QUEUE_SIZE 5      /**< Processing -> Display queue depth    */
/** @} */

/**
 * @defgroup FilterConfig Moving Average Filter Configuration
 * @{
 */
#define FILTER_WINDOW_SIZE 8 /**< Number of samples in moving average  */
/** @} */

/**
 * @defgroup AlarmThresholds Alarm Threshold Values (ADC counts, 12-bit: 0-4095)
 * @{
 */
#define TEMP_HIGH_THRESHOLD 3000  /**< Temperature high alarm threshold     */
#define TEMP_LOW_THRESHOLD 500    /**< Temperature low alarm threshold      */
#define LIGHT_HIGH_THRESHOLD 3500 /**< Light sensor high threshold          */
#define LIGHT_LOW_THRESHOLD 300   /**< Light sensor low alarm threshold     */
/** @} */

/**
 * @defgroup LEDTimerConfig LED Software Timer Configuration
 * @{
 */
#define LED_TIMER_PERIOD_MS 500 /**< Heartbeat LED toggle period          */
/** @} */

/**
 * @defgroup ADCConfig ADC Channel Configuration
 * @{
 */
#define ADC_NUM_CHANNELS 2        /**< Number of ADC channels scanned       */
#define ADC_CHANNEL_TEMPERATURE 0 /**< Index for temperature ADC channel    */
#define ADC_CHANNEL_LIGHT 1       /**< Index for light sensor ADC channel   */
/** @} */

/*===========================================================================*/
/*                    EVENT GROUP BIT DEFINITIONS                            */
/*===========================================================================*/

/**
 * @defgroup EventBits Event Group Bit Definitions
 * @brief Bits used in the system event group for alarm and status signaling.
 *        Multiple bits can be set simultaneously.
 * @{
 */
#define EVT_TEMP_HIGH_ALARM_BIT (1 << 0)  /**< Bit 0: Temperature high alarm     */
#define EVT_TEMP_LOW_ALARM_BIT (1 << 1)   /**< Bit 1: Temperature low alarm      */
#define EVT_LIGHT_HIGH_ALARM_BIT (1 << 2) /**< Bit 2: Light level high alarm     */
#define EVT_LIGHT_LOW_ALARM_BIT (1 << 3)  /**< Bit 3: Light level low alarm      */
#define EVT_SYSTEM_ERROR_BIT (1 << 4)     /**< Bit 4: System error condition     */
#define EVT_DATA_READY_BIT (1 << 5)       /**< Bit 5: New processed data ready   */
#define EVT_SENSOR_TIMEOUT_BIT (1 << 6)   /**< Bit 6: Sensor read timeout        */

/** Combined mask for all temperature alarm bits */
#define EVT_TEMP_ALARM_MASK (EVT_TEMP_HIGH_ALARM_BIT | EVT_TEMP_LOW_ALARM_BIT)

/** Combined mask for all light alarm bits */
#define EVT_LIGHT_ALARM_MASK (EVT_LIGHT_HIGH_ALARM_BIT | EVT_LIGHT_LOW_ALARM_BIT)

/** Combined mask for all alarm bits */
#define EVT_ALL_ALARM_BITS (EVT_TEMP_ALARM_MASK | EVT_LIGHT_ALARM_MASK | \
                            EVT_SYSTEM_ERROR_BIT | EVT_SENSOR_TIMEOUT_BIT)
/** @} */

/*===========================================================================*/
/*                    DATA STRUCTURES                                        */
/*===========================================================================*/

/**
 * @brief Raw sensor data structure.
 *        Populated by Sensor_Task and sent via sensorDataQueue.
 */
typedef struct
{
    uint16_t temperature_raw; /**< Raw ADC value from temperature channel (0-4095) */
    uint16_t light_raw;       /**< Raw ADC value from light sensor channel (0-4095) */
    uint32_t timestamp;       /**< Tick count when sample was taken               */
    uint32_t sequence_number; /**< Monotonic sequence counter for tracking        */
} SensorData_t;

/**
 * @brief Processed sensor data structure.
 *        Populated by Processing_Task and sent via displayQueue.
 */
typedef struct
{
    uint16_t temperature_filtered; /**< Filtered temperature value (moving avg)    */
    uint16_t light_filtered;       /**< Filtered light value (moving avg)          */
    uint16_t temperature_raw;      /**< Last raw temperature for reference         */
    uint16_t light_raw;            /**< Last raw light value for reference         */
    float temperature_voltage;     /**< Temperature converted to voltage (0-3.3V) */
    float light_voltage;           /**< Light converted to voltage (0-3.3V)       */
    bool temp_alarm_active;        /**< True if temperature is out of range       */
    bool light_alarm_active;       /**< True if light level is out of range       */
    uint32_t timestamp;            /**< Tick count of processing                  */
    uint32_t sequence_number;      /**< Inherited from raw data                   */
} ProcessedData_t;

/**
 * @brief System status structure for monitoring.
 *        Updated by Monitor_Task.
 */
typedef struct
{
    /* Stack high water marks (in words) - lower means less margin */
    UBaseType_t sensor_stack_hwm;     /**< Sensor task stack high water mark     */
    UBaseType_t processing_stack_hwm; /**< Processing task stack high water mark */
    UBaseType_t display_stack_hwm;    /**< Display task stack high water mark    */
    UBaseType_t logger_stack_hwm;     /**< Logger task stack high water mark     */
    UBaseType_t monitor_stack_hwm;    /**< Monitor task stack high water mark    */

    /* Heap usage */
    size_t heap_free;     /**< Current free heap in bytes            */
    size_t heap_min_free; /**< Minimum ever free heap in bytes       */

    /* Uptime */
    uint32_t uptime_seconds; /**< System uptime in seconds              */

    /* Task counts */
    UBaseType_t total_tasks; /**< Total number of tasks in the system   */

    /* Alarm states */
    bool temp_alarm;   /**< Current temperature alarm state       */
    bool light_alarm;  /**< Current light alarm state             */
    bool system_error; /**< Current system error state            */
} SystemStatus_t;

/**
 * @brief Log entry structure for Logger_Task.
 */
typedef struct
{
    uint32_t timestamp; /**< Tick count when event occurred         */
    char message[64];   /**< Log message string                    */
    uint8_t severity;   /**< 0=INFO, 1=WARNING, 2=ERROR, 3=CRITICAL */
} LogEntry_t;

/**
 * @brief Severity levels for log entries.
 */
typedef enum
{
    LOG_INFO = 0,    /**< Informational message     */
    LOG_WARNING = 1, /**< Warning condition          */
    LOG_ERROR = 2,   /**< Error condition            */
    LOG_CRITICAL = 3 /**< Critical/fatal condition   */
} LogSeverity_t;

/**
 * @brief System operating states for LED pattern control.
 */
typedef enum
{
    SYS_STATE_INIT = 0,    /**< System initializing       */
    SYS_STATE_NORMAL = 1,  /**< Normal operation           */
    SYS_STATE_WARNING = 2, /**< Warning condition active   */
    SYS_STATE_ERROR = 3    /**< Error condition active     */
} SystemState_t;

/*===========================================================================*/
/*                    EXTERNAL TASK HANDLES                                  */
/*===========================================================================*/

/**
 * @defgroup TaskHandles FreeRTOS Task Handles
 * @brief Handles for all application tasks. Used for task notifications,
 *        priority queries, stack monitoring, and task management.
 * @{
 */
extern TaskHandle_t xSensorTaskHandle;     /**< Sensor data acquisition task     */
extern TaskHandle_t xProcessingTaskHandle; /**< Data processing/filtering task   */
extern TaskHandle_t xDisplayTaskHandle;    /**< Display output task              */
extern TaskHandle_t xLoggerTaskHandle;     /**< Event logging task               */
extern TaskHandle_t xMonitorTaskHandle;    /**< System monitor task              */
/** @} */

/*===========================================================================*/
/*                    EXTERNAL IPC OBJECT HANDLES                            */
/*===========================================================================*/

/**
 * @defgroup QueueHandles Queue Handles
 * @{
 */
extern QueueHandle_t xSensorDataQueue; /**< Sensor -> Processing queue       */
extern QueueHandle_t xDisplayQueue;    /**< Processing -> Display queue      */
/** @} */

/**
 * @defgroup SemaphoreHandles Semaphore and Mutex Handles
 * @{
 */
extern SemaphoreHandle_t xDmaSemaphore; /**< Binary semaphore for DMA sync    */
extern SemaphoreHandle_t xUartMutex;    /**< Mutex protecting UART2 access    */
/** @} */

/**
 * @defgroup TimerHandles Software Timer Handles
 * @{
 */
extern TimerHandle_t xLedTimer; /**< Heartbeat LED software timer     */
/** @} */

/**
 * @defgroup EventGroupHandles Event Group Handles
 * @{
 */
extern EventGroupHandle_t xSystemEventGroup; /**< System-wide event/alarm group   */
/** @} */

/*===========================================================================*/
/*                    EXTERNAL SHARED DATA                                   */
/*===========================================================================*/

extern volatile SystemState_t eCurrentSystemState; /**< Current system state    */
extern volatile uint32_t ulIdleCounter;            /**< Idle task iteration count */

/*===========================================================================*/
/*                    DMA BUFFER (shared with main.c)                        */
/*===========================================================================*/

extern volatile uint16_t usADC_DMA_Buffer[ADC_NUM_CHANNELS]; /**< ADC DMA buffer */

/*===========================================================================*/
/*                    TASK FUNCTION PROTOTYPES                               */
/*===========================================================================*/

/**
 * @brief High-priority sensor data acquisition task.
 *
 * Reads ADC values via DMA at 100ms intervals. Waits for DMA completion
 * semaphore, packages raw data into SensorData_t, and sends to the
 * processing queue. Handles DMA timeout errors.
 *
 * @param pvParameters Unused (NULL)
 */
void vSensorTask(void *pvParameters);

/**
 * @brief Data processing and filtering task.
 *
 * Receives raw sensor data from the sensor queue, applies a moving average
 * filter, converts ADC values to voltages, checks alarm thresholds, and
 * sends processed data to the display queue. Sets event group bits for
 * alarm conditions.
 *
 * @param pvParameters Unused (NULL)
 */
void vProcessingTask(void *pvParameters);

/**
 * @brief Display output task via UART.
 *
 * Receives processed data from the display queue and formats a human-readable
 * output string. Acquires the UART mutex before transmitting to prevent
 * interleaved output with the logger task.
 *
 * @param pvParameters Unused (NULL)
 */
void vDisplayTask(void *pvParameters);

/**
 * @brief Event logging task.
 *
 * Monitors the system event group for alarm conditions and logs events
 * with timestamps via UART. Uses the UART mutex for safe access.
 * Detects both alarm activation and deactivation transitions.
 *
 * @param pvParameters Unused (NULL)
 */
void vLoggerTask(void *pvParameters);

/**
 * @brief System health monitoring task.
 *
 * Periodically reports stack high water marks for all tasks, heap usage,
 * CPU utilization statistics, and alarm states via UART. Runs at the
 * lowest priority (1000ms period).
 *
 * @param pvParameters Unused (NULL)
 */
void vMonitorTask(void *pvParameters);

/**
 * @brief Heartbeat LED software timer callback.
 *
 * Toggles the heartbeat LED (PD12) at a fixed rate. The blink pattern
 * changes based on the current system state:
 * - NORMAL: Slow steady blink
 * - WARNING: Fast blink
 * - ERROR: Rapid flash
 *
 * @param xTimer Handle of the timer that triggered this callback
 */
void vLedTimerCallback(TimerHandle_t xTimer);

/*===========================================================================*/
/*                    INITIALIZATION FUNCTION                                */
/*===========================================================================*/

/**
 * @brief Create all FreeRTOS objects (tasks, queues, semaphores, etc.).
 *
 * This function creates all application tasks, queues, semaphores, mutexes,
 * event groups, and software timers. It should be called from main() before
 * vTaskStartScheduler().
 *
 * @retval pdPASS   All objects created successfully
 * @retval pdFAIL   One or more objects failed to create (insufficient heap)
 */
BaseType_t xAppTasksInit(void);

/*===========================================================================*/
/*                    UTILITY FUNCTION PROTOTYPES                            */
/*===========================================================================*/

/**
 * @brief Send a formatted string via UART with mutex protection.
 *
 * Acquires the UART mutex, transmits the string, then releases the mutex.
 * If the mutex cannot be acquired within the timeout, the message is dropped.
 *
 * @param pcMessage   Null-terminated string to transmit
 * @param xTimeout    Maximum ticks to wait for UART mutex
 */
void vUartPrintProtected(const char *pcMessage, TickType_t xTimeout);

/**
 * @brief Log a system event with severity and timestamp.
 *
 * Creates a log entry and sends it via UART. Thread-safe via UART mutex.
 *
 * @param eSeverity   Log severity level (INFO, WARNING, ERROR, CRITICAL)
 * @param pcMessage   Log message string (max 63 characters)
 */
void vLogEvent(LogSeverity_t eSeverity, const char *pcMessage);

#endif /* APP_TASKS_H */
