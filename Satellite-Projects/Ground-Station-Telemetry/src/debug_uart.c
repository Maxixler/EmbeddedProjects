/**
 * @file debug_uart.c
 * @brief Debug UART implementation - printf-style output with mutex protection
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "debug_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* UART handle declaration (would be initialized in actual STM32 HAL project) */
extern UART_HandleTypeDef huart2;

/* Mutex for UART TX protection (would be created by FreeRTOS) */
extern SemaphoreHandle_t xUartTxMutex;

/**
 * @brief Initialize debug UART
 * @return void
 */
void debug_uart_init(void)
{
    /* In actual implementation, this would initialize the UART peripheral */
    /* For this stub, we just indicate initialization */
    /* huart2 initialization would happen here */
}

/**
 * @brief Formatted print function for debug UART (thread-safe)
 * @param pcFormat Format string (printf-style)
 */
void debug_uart_printf(const char *pcFormat, ...)
{
    va_list xArgs;
    char acBuffer[256];

    if (pcFormat == NULL)
        return;

    va_start(xArgs, pcFormat);
    vsnprintf(acBuffer, sizeof(acBuffer), pcFormat, xArgs);
    va_end(xArgs);

    /* In actual implementation, this would transmit via UART with mutex protection */
    /* For this stub, we just print to console */
    printf("%s", acBuffer);
}

/**
 * @brief Send raw data via debug UART (thread-safe)
 * @param pucData Pointer to data buffer
 * @param usLength Length of data buffer in bytes
 * @return Number of bytes transmitted
 */
size_t debug_uart_send(const uint8_t *pucData, size_t usLength)
{
    if (pucData == NULL || usLength == 0)
        return 0;

    /* In actual implementation, this would transmit via UART with mutex protection */
    /* For this stub, we just print to console */
    for (size_t i = 0; i < usLength; i++)
    {
        printf("%02X ", pucData[i]);
    }
    printf("\r\n");

    return usLength;
}