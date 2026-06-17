#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>
#include <stdbool.h>

/* FreeRTOS configuration */
#define configTICK_RATE_HZ ((TickType_t)1000)

/* Basic FreeRTOS types */
typedef void *TaskHandle_t;
typedef void *QueueHandle_t;
typedef void *SemaphoreHandle_t;
typedef void *EventGroupHandle_t;
typedef uint32_t TickType_t;
typedef uint8_t BaseType_t;
typedef uint8_t UBaseType_t;

#define pdFALSE ((BaseType_t)0)
#define pdTRUE  ((BaseType_t)1)
#define pdPASS  pdTRUE
#define pdFAIL  pdFALSE

#define portMAX_DELAY ((TickType_t)0xFFFFFFFFU)
#define pdMS_TO_TICKS(xTimeInMs) ((TickType_t)(((uint32_t)(xTimeInMs) * (uint32_t)configTICK_RATE_HZ) / (uint32_t)1000))

/* Mock FreeRTOS API functions */
static inline BaseType_t xTaskCreate(
    void (*pvTaskCode)(void *),
    const char * const pcName,
    const uint16_t usStackDepth,
    void *pvParameters,
    UBaseType_t uxPriority,
    TaskHandle_t *pxCreatedTask)
{
    (void)pvTaskCode;
    (void)pcName;
    (void)usStackDepth;
    (void)pvParameters;
    (void)uxPriority;
    (void)pxCreatedTask;
    return pdPASS;
}

static inline QueueHandle_t xQueueCreate(UBaseType_t uxQueueLength, UBaseType_t uxItemSize)
{
    (void)uxQueueLength;
    (void)uxItemSize;
    return (QueueHandle_t)0x1; /* Non-NULL handle */
}

static inline SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return (SemaphoreHandle_t)0x1; /* Non-NULL handle */
}

static inline EventGroupHandle_t xEventGroupCreate(void)
{
    return (EventGroupHandle_t)0x1; /* Non-NULL handle */
}

static inline BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait)
{
    (void)xQueue;
    (void)pvItemToQueue;
    (void)xTicksToWait;
    return pdPASS;
}

static inline BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait)
{
    (void)xQueue;
    (void)pvBuffer;
    (void)xTicksToWait;
    return pdPASS;
}

static inline void vTaskDelay(TickType_t xTicksToDelay)
{
    (void)xTicksToDelay;
}

static inline void vTaskDelayUntil(TickType_t *pxPreviousWakeTime, TickType_t xTimeIncrement)
{
    (void)pxPreviousWakeTime;
    (void)xTimeIncrement;
}

static inline TickType_t xTaskGetTickCount(void)
{
    return 0; /* Stub implementation */
}

static inline UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask)
{
    (void)xTask;
    return 100; /* Stub implementation */
}

static inline void vTaskStartScheduler(void)
{
    /* Stub implementation */
}

static inline void vTaskEndScheduler(void)
{
    /* Stub implementation */
}

static inline void vTaskSuspendAll(void)
{
    /* Stub implementation */
}

static inline BaseType_t xTaskResumeAll(void)
{
    return pdTRUE;
}

/* Additional FreeRTOS functions needed */
static inline void vQueueDelete(QueueHandle_t xQueue)
{
    (void)xQueue;
}

static inline void vSemaphoreDelete(SemaphoreHandle_t xSemaphore)
{
    (void)xSemaphore;
}

static inline void vEventGroupDelete(EventGroupHandle_t xEventGroup)
{
    (void)xEventGroup;
}

static inline BaseType_t xEventGroupSetBits(EventGroupHandle_t xEventGroup, TickType_t uxBitsToSet)
{
    (void)xEventGroup;
    (void)uxBitsToSet;
    return pdPASS;
}

static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore, TickType_t xBlockTime)
{
    (void)xSemaphore;
    (void)xBlockTime;
    return pdPASS;
}

static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore)
{
    (void)xSemaphore;
    return pdPASS;
}

#endif /* FREERTOS_H */