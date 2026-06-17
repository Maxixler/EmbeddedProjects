#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_types.h"

/**
 * @brief LCD status codes
 */
typedef enum {
    LCD_STATUS_SUCCESS = 0,
    LCD_STATUS_INVALID_PARAM,
    LCD_STATUS_ERROR
} lcd_status_t;

/**
 * @brief Initialize LCD display
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_init(void);

/**
 * @brief Clear LCD display
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_clear(void);

/**
 * @brief Set cursor position on LCD
 * @param ucRow Row index (0-based)
 * @param ucColumn Column index (0-based)
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_set_cursor(uint8_t ucRow, uint8_t ucColumn);

/**
 * @brief Print string to LCD at current cursor position
 * @param pcString Null-terminated string to print
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_print_string(const char *pcString);

/**
 * @brief Print formatted telemetry data to LCD
 * @param pxData Pointer to telemetry data structure
 * @return LCD_STATUS_SUCCESS if successful, error code otherwise
 */
lcd_status_t lcd_print_telemetry(const TelemetryData_t *pxData);

#endif /* LCD_DRIVER_H */