/**
 * @file    display_driver.h
 * @brief   SSD1306 OLED display driver (128x64, I2C) with frame buffer,
 *          text rendering, and dashboard screen layout.
 *
 * @details Provides a complete driver for the SSD1306-based 0.96" OLED
 *          display connected via I2C.  Features include:
 *
 *          - Hardware initialisation and power management
 *          - 128x64 pixel frame buffer (1024 bytes)
 *          - Pixel-level drawing primitives
 *          - 5x7 bitmap font character rendering (ASCII 0x20 - 0x7E)
 *          - Pre-defined dashboard layout with labelled sections:
 *              Row 0: Temperature
 *              Row 1: Humidity
 *              Row 2: Pressure
 *              Row 3: Tilt angle
 *              Row 4: Status / alerts
 *
 *          The driver is designed to coexist on the same I2C bus as the
 *          BME280 and MPU6050 sensors.  Callers must acquire the I2C mutex
 *          before calling any function that performs I2C transactions
 *          (init, flush, send_command).
 *
 *          Memory map (SSD1306 page addressing):
 *          +---Page 0---+  rows 0 - 7
 *          +---Page 1---+  rows 8 - 15
 *          +---Page 2---+  rows 16 - 23
 *          +---Page 3---+  rows 24 - 31
 *          +---Page 4---+  rows 32 - 39
 *          +---Page 5---+  rows 40 - 47
 *          +---Page 6---+  rows 48 - 55
 *          +---Page 7---+  rows 56 - 63
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** @name Display Dimensions
 *  @{ */
#define SSD1306_WIDTH (128)                                 /**< Horizontal pixel count. */
#define SSD1306_HEIGHT (64)                                 /**< Vertical pixel count. */
#define SSD1306_PAGES (SSD1306_HEIGHT / 8)                  /**< 8 pages. */
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_PAGES) /**< 1024 bytes. */
/** @} */

/** @name I2C Configuration
 *  @{ */
#define SSD1306_I2C_ADDR (0x3C)      /**< Default 7-bit I2C address. */
#define SSD1306_I2C_PORT I2C_NUM_0   /**< I2C peripheral number. */
#define SSD1306_I2C_TIMEOUT_MS (100) /**< I2C transaction timeout. */
/** @} */

/** @name SSD1306 Command Bytes
 *  @{ */
#define SSD1306_CMD_DISPLAY_OFF (0xAE)     /**< Turn display off. */
#define SSD1306_CMD_DISPLAY_ON (0xAF)      /**< Turn display on. */
#define SSD1306_CMD_SET_MUX_RATIO (0xA8)   /**< Set multiplex ratio. */
#define SSD1306_CMD_SET_OFFSET (0xD3)      /**< Set display offset. */
#define SSD1306_CMD_SET_START_LINE (0x40)  /**< Set display start line. */
#define SSD1306_CMD_SEG_REMAP (0xA1)       /**< Segment re-map (col 127 = SEG0). */
#define SSD1306_CMD_COM_SCAN_DEC (0xC8)    /**< COM output scan: decremental. */
#define SSD1306_CMD_SET_COM_PINS (0xDA)    /**< Set COM pins hardware config. */
#define SSD1306_CMD_SET_CONTRAST (0x81)    /**< Set contrast control. */
#define SSD1306_CMD_ENTIRE_ON_RES (0xA4)   /**< Entire display on (resume). */
#define SSD1306_CMD_SET_NORMAL (0xA6)      /**< Normal display (not inverted). */
#define SSD1306_CMD_SET_INVERT (0xA7)      /**< Inverted display. */
#define SSD1306_CMD_SET_CLK_DIV (0xD5)     /**< Set clock divide ratio. */
#define SSD1306_CMD_SET_CHARGE_PUMP (0x8D) /**< Charge pump setting. */
#define SSD1306_CMD_SET_MEM_ADDR (0x20)    /**< Set memory addressing mode. */
#define SSD1306_CMD_SET_COL_ADDR (0x21)    /**< Set column address range. */
#define SSD1306_CMD_SET_PAGE_ADDR (0x22)   /**< Set page address range. */
/** @} */

/** @name Font Dimensions
 *  @{ */
#define FONT_WIDTH (5)                                    /**< Character width in pixels. */
#define FONT_HEIGHT (7)                                   /**< Character height in pixels. */
#define FONT_SPACING (1)                                  /**< Pixels between characters. */
#define FONT_CHAR_TOTAL_WIDTH (FONT_WIDTH + FONT_SPACING) /**< 6 px stride. */
#define FONT_FIRST_CHAR (0x20)                            /**< First printable ASCII (space). */
#define FONT_LAST_CHAR (0x7E)                             /**< Last printable ASCII (~). */
/** @} */

/** @name Dashboard Layout (pixel coordinates)
 *  @{ */
#define DASH_LABEL_X (0)     /**< Label column start. */
#define DASH_VALUE_X (48)    /**< Value column start. */
#define DASH_ROW_HEIGHT (10) /**< Row height in pixels. */
#define DASH_ROW_TEMP (0)    /**< Temperature row Y. */
#define DASH_ROW_HUMID (10)  /**< Humidity row Y. */
#define DASH_ROW_PRESS (20)  /**< Pressure row Y. */
#define DASH_ROW_TILT (30)   /**< Tilt row Y. */
#define DASH_ROW_ADC (40)    /**< ADC / potentiometer row Y. */
#define DASH_ROW_STATUS (54) /**< Status / alert row Y. */
    /** @} */

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Display driver context (opaque to callers in normal use).
     */
    typedef struct
    {
        uint8_t buffer[SSD1306_BUFFER_SIZE]; /**< Frame buffer. */
        bool initialized;                    /**< True after successful init. */
    } ssd1306_t;

    /* -------------------------------------------------------------------------- */
    /*                   Initialisation - Public Function Prototypes              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialise the SSD1306 display over I2C.
     *
     * @details Sends the full initialisation command sequence, clears the frame
     *          buffer, and turns the display on.  The I2C peripheral must already
     *          be configured (done in main.c during system init).
     *
     * @param[out]  dev     Pointer to the display context.
     * @return  ESP_OK on success, ESP_FAIL on I2C communication error.
     *
     * @note    Caller must hold the I2C mutex.
     */
    esp_err_t ssd1306_init(ssd1306_t *dev);

    /**
     * @brief   Turn the display off (sleep mode).
     *
     * @param[in]   dev     Pointer to the display context.
     * @return  ESP_OK on success.
     */
    esp_err_t ssd1306_display_off(ssd1306_t *dev);

    /**
     * @brief   Turn the display on (normal mode).
     *
     * @param[in]   dev     Pointer to the display context.
     * @return  ESP_OK on success.
     */
    esp_err_t ssd1306_display_on(ssd1306_t *dev);

    /* -------------------------------------------------------------------------- */
    /*                    Frame Buffer - Public Function Prototypes               */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Clear the entire frame buffer (all pixels off).
     *
     * @param[in,out]   dev     Pointer to the display context.
     */
    void ssd1306_clear(ssd1306_t *dev);

    /**
     * @brief   Fill the entire frame buffer (all pixels on).
     *
     * @param[in,out]   dev     Pointer to the display context.
     */
    void ssd1306_fill(ssd1306_t *dev);

    /**
     * @brief   Flush the frame buffer to the SSD1306 over I2C.
     *
     * @details Transfers all 1024 bytes of the frame buffer to the display
     *          using horizontal addressing mode.
     *
     * @param[in]   dev     Pointer to the display context.
     * @return  ESP_OK on success, ESP_FAIL on I2C error.
     *
     * @note    Caller must hold the I2C mutex.
     */
    esp_err_t ssd1306_flush(ssd1306_t *dev);

    /* -------------------------------------------------------------------------- */
    /*                  Drawing Primitives - Public Function Prototypes           */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Set or clear a single pixel in the frame buffer.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       x       X coordinate (0 .. SSD1306_WIDTH-1).
     * @param[in]       y       Y coordinate (0 .. SSD1306_HEIGHT-1).
     * @param[in]       on      True = pixel on, false = pixel off.
     */
    void ssd1306_set_pixel(ssd1306_t *dev, int16_t x, int16_t y, bool on);

    /**
     * @brief   Draw a horizontal line in the frame buffer.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       x       Starting X coordinate.
     * @param[in]       y       Y coordinate.
     * @param[in]       width   Line length in pixels.
     * @param[in]       on      True = pixels on.
     */
    void ssd1306_draw_hline(ssd1306_t *dev, int16_t x, int16_t y,
                            int16_t width, bool on);

    /**
     * @brief   Draw a rectangle outline in the frame buffer.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       x       Top-left X coordinate.
     * @param[in]       y       Top-left Y coordinate.
     * @param[in]       w       Width in pixels.
     * @param[in]       h       Height in pixels.
     * @param[in]       on      True = pixels on.
     */
    void ssd1306_draw_rect(ssd1306_t *dev, int16_t x, int16_t y,
                           int16_t w, int16_t h, bool on);

    /* -------------------------------------------------------------------------- */
    /*                   Text Rendering - Public Function Prototypes              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Draw a single character at the specified pixel position.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       x       X coordinate of top-left corner.
     * @param[in]       y       Y coordinate of top-left corner.
     * @param[in]       c       ASCII character to render (0x20 - 0x7E).
     * @param[in]       on      True = foreground on, false = foreground off.
     */
    void ssd1306_draw_char(ssd1306_t *dev, int16_t x, int16_t y,
                           char c, bool on);

    /**
     * @brief   Draw a null-terminated string at the specified position.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       x       X coordinate of the first character.
     * @param[in]       y       Y coordinate of the first character.
     * @param[in]       str     Null-terminated string.
     * @param[in]       on      True = foreground on.
     */
    void ssd1306_draw_string(ssd1306_t *dev, int16_t x, int16_t y,
                             const char *str, bool on);

    /**
     * @brief   Draw a formatted string (printf-style) at the specified position.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       x       X coordinate.
     * @param[in]       y       Y coordinate.
     * @param[in]       on      True = foreground on.
     * @param[in]       fmt     printf-style format string.
     * @param[in]       ...     Format arguments.
     */
    void ssd1306_draw_printf(ssd1306_t *dev, int16_t x, int16_t y,
                             bool on, const char *fmt, ...)
        __attribute__((format(printf, 5, 6)));

    /* -------------------------------------------------------------------------- */
    /*                   Dashboard Layout - Public Function Prototypes            */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Draw the static dashboard frame (labels, separator lines).
     *
     * @param[in,out]   dev     Pointer to the display context.
     */
    void ssd1306_draw_dashboard_frame(ssd1306_t *dev);

    /**
     * @brief   Update the temperature value on the dashboard.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       temp_c  Temperature in degrees Celsius.
     */
    void ssd1306_dash_set_temperature(ssd1306_t *dev, float temp_c);

    /**
     * @brief   Update the humidity value on the dashboard.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       humidity_pct    Relative humidity in percent.
     */
    void ssd1306_dash_set_humidity(ssd1306_t *dev, float humidity_pct);

    /**
     * @brief   Update the pressure value on the dashboard.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       pressure_hpa    Pressure in hPa.
     */
    void ssd1306_dash_set_pressure(ssd1306_t *dev, float pressure_hpa);

    /**
     * @brief   Update the tilt angle value on the dashboard.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       tilt_deg    Tilt angle in degrees.
     */
    void ssd1306_dash_set_tilt(ssd1306_t *dev, float tilt_deg);

    /**
     * @brief   Update the ADC / potentiometer value on the dashboard.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       voltage ADC voltage in volts.
     */
    void ssd1306_dash_set_adc(ssd1306_t *dev, float voltage);

    /**
     * @brief   Update the status / alert line on the dashboard.
     *
     * @param[in,out]   dev     Pointer to the display context.
     * @param[in]       msg     Status message string (max ~21 characters).
     */
    void ssd1306_dash_set_status(ssd1306_t *dev, const char *msg);

    /* -------------------------------------------------------------------------- */
    /*                   Low-Level I2C - Public Function Prototypes               */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Send a single command byte to the SSD1306.
     *
     * @param[in]   cmd     Command byte.
     * @return  ESP_OK on success.
     *
     * @note    Caller must hold the I2C mutex.
     */
    esp_err_t ssd1306_send_command(uint8_t cmd);

    /**
     * @brief   Send a buffer of data bytes to the SSD1306.
     *
     * @param[in]   data    Pointer to data buffer.
     * @param[in]   len     Number of bytes.
     * @return  ESP_OK on success.
     *
     * @note    Caller must hold the I2C mutex.
     */
    esp_err_t ssd1306_send_data(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAY_DRIVER_H */
