/**
 * @file lcd_driver.c
 * @brief LCD driver implementation - stub version that routes to debug UART
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "lcd_driver.h"
#include "debug_uart.h"
#include <stddef.h>
#include <stdio.h>

/**
 * @brief Initialize LCD display
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_init(void)
{
    debug_uart_printf("LCD: Initialized (stub implementation)\r\n");
    return LCD_STATUS_SUCCESS;
}

/**
 * @brief Clear LCD display
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_clear(void)
{
    debug_uart_printf("LCD: Cleared display\r\n");
    return LCD_STATUS_SUCCESS;
}

/**
 * @brief Set cursor position on LCD
 * @param ucRow Row index (0-based)
 * @param ucColumn Column index (0-based)
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_set_cursor(uint8_t ucRow, uint8_t ucColumn)
{
    debug_uart_printf("LCD: Set cursor to (%u, %u)\r\n", ucRow, ucColumn);
    return LCD_STATUS_SUCCESS;
}

/**
 * @brief Print string to LCD at current cursor position
 * @param pcString Null-terminated string to print
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_print_string(const char *pcString)
{
    if (pcString == NULL)
        return LCD_STATUS_INVALID_PARAM;

    debug_uart_printf("LCD: \"%s\"\r\n", pcString);
    return LCD_STATUS_SUCCESS;
}

/**
 * @brief Print formatted telemetry data to LCD
 * @param pxData Pointer to telemetry data structure
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_print_telemetry(const TelemetryData_t *pxData)
{
    if (pxData == NULL)
        return LCD_STATUS_INVALID_PARAM;

    if (!pxData->valid)
    {
        lcd_print_string("INVALID TELEMETRY");
        return LCD_STATUS_SUCCESS;
    }

    char acBuffer[32];
    lcd_set_cursor(0, 0);
    snprintf(acBuffer, sizeof(acBuffer), "Temp: %.1fC", pxData->temperature);
    lcd_print_string(acBuffer);

    lcd_set_cursor(0, 1);
    snprintf(acBuffer, sizeof(acBuffer), "Volt: %.2fV", pxData->voltage);
    lcd_print_string(acBuffer);

    return LCD_STATUS_SUCCESS;
}