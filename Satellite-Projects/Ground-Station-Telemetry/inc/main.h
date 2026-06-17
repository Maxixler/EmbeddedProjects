#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"

/* Forward declarations */
void SystemClock_Config(void);
void GPIO_Init(void);
void USART2_Init(void);
void NVIC_Config(void);

/* FreeRTOS hook functions */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

#endif /* MAIN_H */