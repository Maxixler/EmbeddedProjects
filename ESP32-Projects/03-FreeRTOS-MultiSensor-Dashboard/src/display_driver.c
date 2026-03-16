/**
 * @file    display_driver.c
 * @brief   SSD1306 OLED display driver implementation (128x64, I2C) with
 *          frame buffer, 5x7 font rendering, and dashboard layout.
 *
 * @details Implements the complete driver for an SSD1306-based 0.96" OLED
 *          display connected via I2C on the same bus as the BME280 and
 *          MPU6050 sensors.
 *
 *          Frame Buffer Architecture:
 *          The SSD1306 uses page-based addressing.  The 64-pixel-high
 *          display is divided into 8 pages of 8 rows each.  Each byte
 *          in the frame buffer represents a vertical column of 8 pixels
 *          within a page, with bit 0 at the top:
 *
 *              Byte layout within one page column:
 *                  bit 0  ->  top pixel
 *                  bit 1
 *                  bit 2
 *                  bit 3
 *                  bit 4
 *                  bit 5
 *                  bit 6
 *                  bit 7  ->  bottom pixel
 *
 *          The 1024-byte buffer is organised as:
 *              buffer[page * SSD1306_WIDTH + column]
 *
 *          Font:
 *          A minimal 5x7 pixel bitmap font covers printable ASCII
 *          characters 0x20 (space) through 0x7E (~).  Each character is
 *          stored as 5 column bytes; a 1-pixel spacing column is added
 *          during rendering for a total stride of 6 pixels.
 *
 *          Dashboard Layout (128x64):
 *          +---------------------------+
 *          | Temp:   25.3 C            |  Row 0  (y = 0)
 *          | Humid:  48.2 %            |  Row 1  (y = 10)
 *          | Press:  1013 hPa          |  Row 2  (y = 20)
 *          | Tilt:   12.5 deg          |  Row 3  (y = 30)
 *          | ADC:    1.65 V            |  Row 4  (y = 40)
 *          |---------------------------|
 *          | Status: OK                |  Row 5  (y = 54)
 *          +---------------------------+
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "display_driver.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "ssd1306";

/* -------------------------------------------------------------------------- */
/*                          5x7 Bitmap Font Table                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief   5x7 pixel font data for printable ASCII characters (0x20 - 0x7E).
 *
 * @details Each character is represented by 5 bytes (one per column).
 *          Bit 0 of each byte corresponds to the top pixel row; bit 6
 *          corresponds to the bottom pixel row.  Bit 7 is unused.
 *
 *          Total: 95 characters * 5 bytes = 475 bytes.
 */
static const uint8_t s_font_5x7[][FONT_WIDTH] = {
    /* 0x20 ' ' */ {0x00, 0x00, 0x00, 0x00, 0x00},
    /* 0x21 '!' */ {0x00, 0x00, 0x5F, 0x00, 0x00},
    /* 0x22 '"' */ {0x00, 0x07, 0x00, 0x07, 0x00},
    /* 0x23 '#' */ {0x14, 0x7F, 0x14, 0x7F, 0x14},
    /* 0x24 '$' */ {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    /* 0x25 '%' */ {0x23, 0x13, 0x08, 0x64, 0x62},
    /* 0x26 '&' */ {0x36, 0x49, 0x55, 0x22, 0x50},
    /* 0x27 ''' */ {0x00, 0x05, 0x03, 0x00, 0x00},
    /* 0x28 '(' */ {0x00, 0x1C, 0x22, 0x41, 0x00},
    /* 0x29 ')' */ {0x00, 0x41, 0x22, 0x1C, 0x00},
    /* 0x2A '*' */ {0x14, 0x08, 0x3E, 0x08, 0x14},
    /* 0x2B '+' */ {0x08, 0x08, 0x3E, 0x08, 0x08},
    /* 0x2C ',' */ {0x00, 0x50, 0x30, 0x00, 0x00},
    /* 0x2D '-' */ {0x08, 0x08, 0x08, 0x08, 0x08},
    /* 0x2E '.' */ {0x00, 0x60, 0x60, 0x00, 0x00},
    /* 0x2F '/' */ {0x20, 0x10, 0x08, 0x04, 0x02},
    /* 0x30 '0' */ {0x3E, 0x51, 0x49, 0x45, 0x3E},
    /* 0x31 '1' */ {0x00, 0x42, 0x7F, 0x40, 0x00},
    /* 0x32 '2' */ {0x42, 0x61, 0x51, 0x49, 0x46},
    /* 0x33 '3' */ {0x21, 0x41, 0x45, 0x4B, 0x31},
    /* 0x34 '4' */ {0x18, 0x14, 0x12, 0x7F, 0x10},
    /* 0x35 '5' */ {0x27, 0x45, 0x45, 0x45, 0x39},
    /* 0x36 '6' */ {0x3C, 0x4A, 0x49, 0x49, 0x30},
    /* 0x37 '7' */ {0x01, 0x71, 0x09, 0x05, 0x03},
    /* 0x38 '8' */ {0x36, 0x49, 0x49, 0x49, 0x36},
    /* 0x39 '9' */ {0x06, 0x49, 0x49, 0x29, 0x1E},
    /* 0x3A ':' */ {0x00, 0x36, 0x36, 0x00, 0x00},
    /* 0x3B ';' */ {0x00, 0x56, 0x36, 0x00, 0x00},
    /* 0x3C '<' */ {0x08, 0x14, 0x22, 0x41, 0x00},
    /* 0x3D '=' */ {0x14, 0x14, 0x14, 0x14, 0x14},
    /* 0x3E '>' */ {0x00, 0x41, 0x22, 0x14, 0x08},
    /* 0x3F '?' */ {0x02, 0x01, 0x51, 0x09, 0x06},
    /* 0x40 '@' */ {0x32, 0x49, 0x79, 0x41, 0x3E},
    /* 0x41 'A' */ {0x7E, 0x11, 0x11, 0x11, 0x7E},
    /* 0x42 'B' */ {0x7F, 0x49, 0x49, 0x49, 0x36},
    /* 0x43 'C' */ {0x3E, 0x41, 0x41, 0x41, 0x22},
    /* 0x44 'D' */ {0x7F, 0x41, 0x41, 0x22, 0x1C},
    /* 0x45 'E' */ {0x7F, 0x49, 0x49, 0x49, 0x41},
    /* 0x46 'F' */ {0x7F, 0x09, 0x09, 0x09, 0x01},
    /* 0x47 'G' */ {0x3E, 0x41, 0x49, 0x49, 0x7A},
    /* 0x48 'H' */ {0x7F, 0x08, 0x08, 0x08, 0x7F},
    /* 0x49 'I' */ {0x00, 0x41, 0x7F, 0x41, 0x00},
    /* 0x4A 'J' */ {0x20, 0x40, 0x41, 0x3F, 0x01},
    /* 0x4B 'K' */ {0x7F, 0x08, 0x14, 0x22, 0x41},
    /* 0x4C 'L' */ {0x7F, 0x40, 0x40, 0x40, 0x40},
    /* 0x4D 'M' */ {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    /* 0x4E 'N' */ {0x7F, 0x04, 0x08, 0x10, 0x7F},
    /* 0x4F 'O' */ {0x3E, 0x41, 0x41, 0x41, 0x3E},
    /* 0x50 'P' */ {0x7F, 0x09, 0x09, 0x09, 0x06},
    /* 0x51 'Q' */ {0x3E, 0x41, 0x51, 0x21, 0x5E},
    /* 0x52 'R' */ {0x7F, 0x09, 0x19, 0x29, 0x46},
    /* 0x53 'S' */ {0x46, 0x49, 0x49, 0x49, 0x31},
    /* 0x54 'T' */ {0x01, 0x01, 0x7F, 0x01, 0x01},
    /* 0x55 'U' */ {0x3F, 0x40, 0x40, 0x40, 0x3F},
    /* 0x56 'V' */ {0x1F, 0x20, 0x40, 0x20, 0x1F},
    /* 0x57 'W' */ {0x3F, 0x40, 0x38, 0x40, 0x3F},
    /* 0x58 'X' */ {0x63, 0x14, 0x08, 0x14, 0x63},
    /* 0x59 'Y' */ {0x07, 0x08, 0x70, 0x08, 0x07},
    /* 0x5A 'Z' */ {0x61, 0x51, 0x49, 0x45, 0x43},
    /* 0x5B '[' */ {0x00, 0x7F, 0x41, 0x41, 0x00},
    /* 0x5C '\' */ {0x02, 0x04, 0x08, 0x10, 0x20},
    /* 0x5D ']' */ {0x00, 0x41, 0x41, 0x7F, 0x00},
    /* 0x5E '^' */ {0x04, 0x02, 0x01, 0x02, 0x04},
    /* 0x5F '_' */ {0x40, 0x40, 0x40, 0x40, 0x40},
    /* 0x60 '`' */ {0x00, 0x01, 0x02, 0x04, 0x00},
    /* 0x61 'a' */ {0x20, 0x54, 0x54, 0x54, 0x78},
    /* 0x62 'b' */ {0x7F, 0x48, 0x44, 0x44, 0x38},
    /* 0x63 'c' */ {0x38, 0x44, 0x44, 0x44, 0x20},
    /* 0x64 'd' */ {0x38, 0x44, 0x44, 0x48, 0x7F},
    /* 0x65 'e' */ {0x38, 0x54, 0x54, 0x54, 0x18},
    /* 0x66 'f' */ {0x08, 0x7E, 0x09, 0x01, 0x02},
    /* 0x67 'g' */ {0x0C, 0x52, 0x52, 0x52, 0x3E},
    /* 0x68 'h' */ {0x7F, 0x08, 0x04, 0x04, 0x78},
    /* 0x69 'i' */ {0x00, 0x44, 0x7D, 0x40, 0x00},
    /* 0x6A 'j' */ {0x20, 0x40, 0x44, 0x3D, 0x00},
    /* 0x6B 'k' */ {0x7F, 0x10, 0x28, 0x44, 0x00},
    /* 0x6C 'l' */ {0x00, 0x41, 0x7F, 0x40, 0x00},
    /* 0x6D 'm' */ {0x7C, 0x04, 0x18, 0x04, 0x78},
    /* 0x6E 'n' */ {0x7C, 0x08, 0x04, 0x04, 0x78},
    /* 0x6F 'o' */ {0x38, 0x44, 0x44, 0x44, 0x38},
    /* 0x70 'p' */ {0x7C, 0x14, 0x14, 0x14, 0x08},
    /* 0x71 'q' */ {0x08, 0x14, 0x14, 0x18, 0x7C},
    /* 0x72 'r' */ {0x7C, 0x08, 0x04, 0x04, 0x08},
    /* 0x73 's' */ {0x48, 0x54, 0x54, 0x54, 0x20},
    /* 0x74 't' */ {0x04, 0x3F, 0x44, 0x40, 0x20},
    /* 0x75 'u' */ {0x3C, 0x40, 0x40, 0x20, 0x7C},
    /* 0x76 'v' */ {0x1C, 0x20, 0x40, 0x20, 0x1C},
    /* 0x77 'w' */ {0x3C, 0x40, 0x30, 0x40, 0x3C},
    /* 0x78 'x' */ {0x44, 0x28, 0x10, 0x28, 0x44},
    /* 0x79 'y' */ {0x0C, 0x50, 0x50, 0x50, 0x3C},
    /* 0x7A 'z' */ {0x44, 0x64, 0x54, 0x4C, 0x44},
    /* 0x7B '{' */ {0x00, 0x08, 0x36, 0x41, 0x00},
    /* 0x7C '|' */ {0x00, 0x00, 0x7F, 0x00, 0x00},
    /* 0x7D '}' */ {0x00, 0x41, 0x36, 0x08, 0x00},
    /* 0x7E '~' */ {0x10, 0x08, 0x08, 0x10, 0x08},
};

/* -------------------------------------------------------------------------- */
/*                   Low-Level I2C - Public Function Definitions              */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Send a single command byte to the SSD1306 over I2C.
 *
 * @details The I2C write sequence for a command is:
 *          [START][ADDR+W][0x00 (Co=0, D/C#=0)][command byte][STOP]
 *
 *          The control byte 0x00 tells the SSD1306 that the following
 *          byte is a command (D/C# = 0) and that this is the last
 *          control byte (Co = 0).
 *
 * @note    Caller must hold the I2C mutex.
 */
esp_err_t ssd1306_send_command(uint8_t cmd)
{
    uint8_t buf[2];
    buf[0] = 0x00; /* Control byte: Co=0, D/C#=0 (command mode). */
    buf[1] = cmd;

    esp_err_t ret = i2c_master_write_to_device(
        SSD1306_I2C_PORT,
        SSD1306_I2C_ADDR,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(SSD1306_I2C_TIMEOUT_MS));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Command 0x%02X send failed: %s", cmd, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief   Send a buffer of data bytes to the SSD1306 over I2C.
 *
 * @details The I2C write sequence for data is:
 *          [START][ADDR+W][0x40 (Co=0, D/C#=1)][data0][data1]...[STOP]
 *
 *          The control byte 0x40 tells the SSD1306 that all following
 *          bytes are GDDRAM data (D/C# = 1).
 *
 *          To avoid allocating a large temporary buffer (1025 bytes for
 *          a full frame), this function uses the ESP-IDF I2C command
 *          link builder to send the control byte followed by the data
 *          in a single I2C transaction.
 *
 * @note    Caller must hold the I2C mutex.
 */
esp_err_t ssd1306_send_data(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x40, true); /* Control byte: Co=0, D/C#=1 (data). */
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        SSD1306_I2C_PORT,
        cmd,
        pdMS_TO_TICKS(SSD1306_I2C_TIMEOUT_MS));

    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Data send (%u bytes) failed: %s", len, esp_err_to_name(ret));
    }

    return ret;
}

/* -------------------------------------------------------------------------- */
/*                   Initialisation - Public Function Definitions             */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise the SSD1306 display over I2C.
 *
 * @details Sends the complete initialisation command sequence as specified
 *          in the SSD1306 application note.  The sequence configures:
 *
 *          1. Display off during setup.
 *          2. Clock divide ratio and oscillator frequency.
 *          3. Multiplex ratio (64 lines).
 *          4. Display offset (no offset).
 *          5. Display start line (line 0).
 *          6. Charge pump enable (internal VCC).
 *          7. Memory addressing mode (horizontal).
 *          8. Segment remap and COM scan direction (normal orientation).
 *          9. COM pin configuration (alternative, no remap).
 *         10. Contrast (0x7F, medium).
 *         11. Entire display on (follow RAM content).
 *         12. Normal display (not inverted).
 *         13. Column and page address ranges (full screen).
 *         14. Display on.
 *
 *          After initialisation, the frame buffer is cleared and flushed
 *          so the display starts with a blank screen.
 *
 * @note    Caller must hold the I2C mutex.
 */
esp_err_t ssd1306_init(ssd1306_t *dev)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(ssd1306_t));

    /*
     * SSD1306 Initialisation Command Sequence
     */
    static const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF, /* 0xAE: Display OFF. */
        SSD1306_CMD_SET_CLK_DIV,
        0x80, /* 0xD5, 0x80: Default clock. */
        SSD1306_CMD_SET_MUX_RATIO,
        0x3F, /* 0xA8, 0x3F: 64 MUX. */
        SSD1306_CMD_SET_OFFSET,
        0x00,                       /* 0xD3, 0x00: No offset. */
        SSD1306_CMD_SET_START_LINE, /* 0x40: Start line 0. */
        SSD1306_CMD_SET_CHARGE_PUMP,
        0x14, /* 0x8D, 0x14: Enable charge pump. */
        SSD1306_CMD_SET_MEM_ADDR,
        0x00,                     /* 0x20, 0x00: Horizontal addressing. */
        SSD1306_CMD_SEG_REMAP,    /* 0xA1: Segment remap. */
        SSD1306_CMD_COM_SCAN_DEC, /* 0xC8: COM scan decremental. */
        SSD1306_CMD_SET_COM_PINS,
        0x12, /* 0xDA, 0x12: Alternative COM pins. */
        SSD1306_CMD_SET_CONTRAST,
        0x7F,                      /* 0x81, 0x7F: Medium contrast. */
        SSD1306_CMD_ENTIRE_ON_RES, /* 0xA4: Display follows RAM. */
        SSD1306_CMD_SET_NORMAL,    /* 0xA6: Normal (not inverted). */
    };

    /* Send each command byte. */
    for (size_t i = 0; i < sizeof(init_cmds); i++)
    {
        esp_err_t ret = ssd1306_send_command(init_cmds[i]);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Init command [%zu]=0x%02X failed", i, init_cmds[i]);
            return ESP_FAIL;
        }
    }

    /* Set column address range: 0 to 127. */
    ssd1306_send_command(SSD1306_CMD_SET_COL_ADDR);
    ssd1306_send_command(0x00);
    ssd1306_send_command(0x7F);

    /* Set page address range: 0 to 7. */
    ssd1306_send_command(SSD1306_CMD_SET_PAGE_ADDR);
    ssd1306_send_command(0x00);
    ssd1306_send_command(0x07);

    /* Clear the frame buffer and flush to display. */
    ssd1306_clear(dev);
    esp_err_t ret = ssd1306_flush(dev);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }

    /* Turn display on. */
    ret = ssd1306_send_command(SSD1306_CMD_DISPLAY_ON);
    if (ret != ESP_OK)
    {
        return ESP_FAIL;
    }

    dev->initialized = true;
    ESP_LOGI(TAG, "SSD1306 initialized (128x64, addr 0x%02X)", SSD1306_I2C_ADDR);

    return ESP_OK;
}

/**
 * @brief   Turn the display off (sleep mode).
 */
esp_err_t ssd1306_display_off(ssd1306_t *dev)
{
    (void)dev;
    return ssd1306_send_command(SSD1306_CMD_DISPLAY_OFF);
}

/**
 * @brief   Turn the display on (normal mode).
 */
esp_err_t ssd1306_display_on(ssd1306_t *dev)
{
    (void)dev;
    return ssd1306_send_command(SSD1306_CMD_DISPLAY_ON);
}

/* -------------------------------------------------------------------------- */
/*                    Frame Buffer - Public Function Definitions              */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Clear the entire frame buffer (all pixels off).
 */
void ssd1306_clear(ssd1306_t *dev)
{
    if (dev == NULL)
    {
        return;
    }

    memset(dev->buffer, 0x00, SSD1306_BUFFER_SIZE);
}

/**
 * @brief   Fill the entire frame buffer (all pixels on).
 */
void ssd1306_fill(ssd1306_t *dev)
{
    if (dev == NULL)
    {
        return;
    }

    memset(dev->buffer, 0xFF, SSD1306_BUFFER_SIZE);
}

/**
 * @brief   Flush the frame buffer to the SSD1306 over I2C.
 *
 * @details Sets the column and page address ranges to cover the full
 *          display, then transfers all 1024 bytes of the frame buffer
 *          in a single I2C data transaction using horizontal addressing
 *          mode.
 *
 * @note    Caller must hold the I2C mutex.
 */
esp_err_t ssd1306_flush(ssd1306_t *dev)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Reset column and page address pointers to start of display. */
    ssd1306_send_command(SSD1306_CMD_SET_COL_ADDR);
    ssd1306_send_command(0x00); /* Column start: 0. */
    ssd1306_send_command(0x7F); /* Column end: 127. */

    ssd1306_send_command(SSD1306_CMD_SET_PAGE_ADDR);
    ssd1306_send_command(0x00); /* Page start: 0. */
    ssd1306_send_command(0x07); /* Page end: 7. */

    /* Transfer the entire frame buffer. */
    return ssd1306_send_data(dev->buffer, SSD1306_BUFFER_SIZE);
}

/* -------------------------------------------------------------------------- */
/*                  Drawing Primitives - Public Function Definitions          */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Set or clear a single pixel in the frame buffer.
 *
 * @details Computes the page index and bit position from the (x, y)
 *          coordinate and sets or clears the corresponding bit in the
 *          frame buffer.
 *
 *          Buffer index:  page * SSD1306_WIDTH + x
 *          Bit position:  y % 8
 *
 *          Out-of-bounds coordinates are silently ignored.
 */
void ssd1306_set_pixel(ssd1306_t *dev, int16_t x, int16_t y, bool on)
{
    if (dev == NULL)
    {
        return;
    }

    /* Bounds check. */
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT)
    {
        return;
    }

    uint16_t page = (uint16_t)(y / 8);
    uint8_t bit = (uint8_t)(y % 8);
    uint16_t idx = page * SSD1306_WIDTH + (uint16_t)x;

    if (on)
    {
        dev->buffer[idx] |= (1U << bit);
    }
    else
    {
        dev->buffer[idx] &= ~(1U << bit);
    }
}

/**
 * @brief   Draw a horizontal line in the frame buffer.
 *
 * @details Iterates from x to x+width-1, setting each pixel via
 *          ssd1306_set_pixel().  Negative width is ignored.
 */
void ssd1306_draw_hline(ssd1306_t *dev, int16_t x, int16_t y,
                        int16_t width, bool on)
{
    for (int16_t i = 0; i < width; i++)
    {
        ssd1306_set_pixel(dev, x + i, y, on);
    }
}

/**
 * @brief   Draw a rectangle outline in the frame buffer.
 *
 * @details Draws four lines: top, bottom, left, and right edges.
 */
void ssd1306_draw_rect(ssd1306_t *dev, int16_t x, int16_t y,
                       int16_t w, int16_t h, bool on)
{
    /* Top and bottom horizontal lines. */
    ssd1306_draw_hline(dev, x, y, w, on);
    ssd1306_draw_hline(dev, x, (int16_t)(y + h - 1), w, on);

    /* Left and right vertical lines. */
    for (int16_t i = 0; i < h; i++)
    {
        ssd1306_set_pixel(dev, x, (int16_t)(y + i), on);
        ssd1306_set_pixel(dev, (int16_t)(x + w - 1), (int16_t)(y + i), on);
    }
}

/* -------------------------------------------------------------------------- */
/*                   Text Rendering - Public Function Definitions             */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Draw a single character at the specified pixel position.
 *
 * @details Looks up the character in the 5x7 font table and renders it
 *          pixel by pixel into the frame buffer.  Characters outside the
 *          printable ASCII range (0x20 - 0x7E) are rendered as a space.
 *
 *          The character is 5 pixels wide and 7 pixels tall.  A 1-pixel
 *          gap column is NOT drawn here; it is handled by the string
 *          rendering function via FONT_CHAR_TOTAL_WIDTH stride.
 */
void ssd1306_draw_char(ssd1306_t *dev, int16_t x, int16_t y,
                       char c, bool on)
{
    if (dev == NULL)
    {
        return;
    }

    /* Clamp to printable range. */
    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR)
    {
        c = ' ';
    }

    uint8_t char_index = (uint8_t)(c - FONT_FIRST_CHAR);
    const uint8_t *glyph = s_font_5x7[char_index];

    /* Render each column of the glyph. */
    for (int16_t col = 0; col < FONT_WIDTH; col++)
    {
        uint8_t col_data = glyph[col];

        for (int16_t row = 0; row < FONT_HEIGHT; row++)
        {
            if (col_data & (1U << row))
            {
                ssd1306_set_pixel(dev, (int16_t)(x + col),
                                  (int16_t)(y + row), on);
            }
            else
            {
                /* Clear the pixel (write background). */
                ssd1306_set_pixel(dev, (int16_t)(x + col),
                                  (int16_t)(y + row), !on);
            }
        }
    }

    /* Draw the 1-pixel spacing column (clear it). */
    for (int16_t row = 0; row < FONT_HEIGHT; row++)
    {
        ssd1306_set_pixel(dev, (int16_t)(x + FONT_WIDTH),
                          (int16_t)(y + row), !on);
    }
}

/**
 * @brief   Draw a null-terminated string at the specified position.
 *
 * @details Iterates through each character in the string, rendering it
 *          with ssd1306_draw_char() and advancing the X position by
 *          FONT_CHAR_TOTAL_WIDTH (6 pixels) per character.
 *
 *          Characters that would start beyond the right edge of the
 *          display are not rendered (no automatic line wrapping).
 */
void ssd1306_draw_string(ssd1306_t *dev, int16_t x, int16_t y,
                         const char *str, bool on)
{
    if (dev == NULL || str == NULL)
    {
        return;
    }

    int16_t cursor_x = x;

    while (*str != '\0')
    {
        /* Stop if the character would start off-screen. */
        if (cursor_x + FONT_WIDTH > SSD1306_WIDTH)
        {
            break;
        }

        ssd1306_draw_char(dev, cursor_x, y, *str, on);
        cursor_x += FONT_CHAR_TOTAL_WIDTH;
        str++;
    }
}

/**
 * @brief   Draw a formatted string (printf-style) at the specified position.
 *
 * @details Uses vsnprintf() to format the string into a local buffer,
 *          then renders it with ssd1306_draw_string().
 *
 *          Maximum formatted string length: 31 characters (128 pixels /
 *          6 pixels per character = 21 visible characters, but we allow
 *          a larger buffer for safety).
 */
void ssd1306_draw_printf(ssd1306_t *dev, int16_t x, int16_t y,
                         bool on, const char *fmt, ...)
{
    if (dev == NULL || fmt == NULL)
    {
        return;
    }

    char buf[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    ssd1306_draw_string(dev, x, y, buf, on);
}

/* -------------------------------------------------------------------------- */
/*                   Dashboard Layout - Public Function Definitions           */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Draw the static dashboard frame (labels and separator lines).
 *
 * @details Renders the fixed text labels for each data row and draws
 *          a horizontal separator line above the status row.
 *
 *          Layout:
 *          ```
 *          Temp:                          y = 0
 *          Humid:                         y = 10
 *          Press:                         y = 20
 *          Tilt:                          y = 30
 *          ADC:                           y = 40
 *          -------------------------------- y = 52
 *          Status:                        y = 54
 *          ```
 */
void ssd1306_draw_dashboard_frame(ssd1306_t *dev)
{
    if (dev == NULL)
    {
        return;
    }

    ssd1306_clear(dev);

    /* Draw row labels. */
    ssd1306_draw_string(dev, DASH_LABEL_X, DASH_ROW_TEMP, "Temp:", true);
    ssd1306_draw_string(dev, DASH_LABEL_X, DASH_ROW_HUMID, "Humid:", true);
    ssd1306_draw_string(dev, DASH_LABEL_X, DASH_ROW_PRESS, "Press:", true);
    ssd1306_draw_string(dev, DASH_LABEL_X, DASH_ROW_TILT, "Tilt:", true);
    ssd1306_draw_string(dev, DASH_LABEL_X, DASH_ROW_ADC, "ADC:", true);

    /* Draw separator line above status row. */
    ssd1306_draw_hline(dev, 0, 52, SSD1306_WIDTH, true);

    /* Draw status label. */
    ssd1306_draw_string(dev, DASH_LABEL_X, DASH_ROW_STATUS, "Stat:", true);
}

/**
 * @brief   Update the temperature value on the dashboard.
 *
 * @details Renders the temperature value at the DASH_VALUE_X column
 *          on the DASH_ROW_TEMP row, formatted as "XX.X C".
 *
 *          The value area is first cleared by drawing spaces, then
 *          the new value is rendered.
 */
void ssd1306_dash_set_temperature(ssd1306_t *dev, float temp_c)
{
    if (dev == NULL)
    {
        return;
    }

    ssd1306_draw_printf(dev, DASH_VALUE_X, DASH_ROW_TEMP,
                        true, "%5.1f C", temp_c);
}

/**
 * @brief   Update the humidity value on the dashboard.
 */
void ssd1306_dash_set_humidity(ssd1306_t *dev, float humidity_pct)
{
    if (dev == NULL)
    {
        return;
    }

    ssd1306_draw_printf(dev, DASH_VALUE_X, DASH_ROW_HUMID,
                        true, "%5.1f %%", humidity_pct);
}

/**
 * @brief   Update the pressure value on the dashboard.
 */
void ssd1306_dash_set_pressure(ssd1306_t *dev, float pressure_hpa)
{
    if (dev == NULL)
    {
        return;
    }

    ssd1306_draw_printf(dev, DASH_VALUE_X, DASH_ROW_PRESS,
                        true, "%6.0fhPa", pressure_hpa);
}

/**
 * @brief   Update the tilt angle value on the dashboard.
 */
void ssd1306_dash_set_tilt(ssd1306_t *dev, float tilt_deg)
{
    if (dev == NULL)
    {
        return;
    }

    ssd1306_draw_printf(dev, DASH_VALUE_X, DASH_ROW_TILT,
                        true, "%5.1f dg", tilt_deg);
}

/**
 * @brief   Update the ADC / potentiometer value on the dashboard.
 */
void ssd1306_dash_set_adc(ssd1306_t *dev, float voltage)
{
    if (dev == NULL)
    {
        return;
    }

    ssd1306_draw_printf(dev, DASH_VALUE_X, DASH_ROW_ADC,
                        true, "%4.2f V", voltage);
}

/**
 * @brief   Update the status / alert line on the dashboard.
 *
 * @details Draws the status message at the DASH_ROW_STATUS row.
 *          The message is truncated if it exceeds the available
 *          screen width (~13 characters after the "Stat:" label).
 */
void ssd1306_dash_set_status(ssd1306_t *dev, const char *msg)
{
    if (dev == NULL || msg == NULL)
    {
        return;
    }

    /* Clear the value area of the status row first. */
    ssd1306_draw_string(dev, DASH_VALUE_X, DASH_ROW_STATUS,
                        "             ", true);

    /* Draw the new status message. */
    ssd1306_draw_string(dev, DASH_VALUE_X, DASH_ROW_STATUS, msg, true);
}
