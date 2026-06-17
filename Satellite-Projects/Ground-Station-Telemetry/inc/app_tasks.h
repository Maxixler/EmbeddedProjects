#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Initializes all application tasks, IPC objects, and starts the scheduler.
 *
 * This function creates all queues, mutexes, event groups, and tasks.
 * It should be called from main() before starting the scheduler.
 *
 * @return BaseType_t pdPASS if initialization successful, pdFAIL otherwise.
 */
BaseType_t xAppTasksInit(void);

#endif /* APP_TASKS_H */