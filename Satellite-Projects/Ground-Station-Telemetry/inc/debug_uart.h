#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

/**
 * @brief Initialize debug UART
 * @return void
 */
void debug_uart_init(void);

/**
 * @brief Debug UART printf-like function.
 *
 * This function sends formatted output to the debug UART.
 * It is intended to be used similarly to the standard printf function.
 *
 * @param format Format string (see printf for details).
 * @param ...    Variable arguments corresponding to the format string.
 */
void debug_uart_printf(const char *format, ...);

#endif /* DEBUG_UART_H */