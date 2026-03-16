/**
 * @file    sensor_reader.c
 * @brief   Multi-sensor reader implementation for DHT22 and BH1750.
 *
 * @details Implements sensor reading routines for:
 *
 *          DHT22 (AM2302) - Temperature & Humidity:
 *          - One-wire protocol with strict timing requirements.
 *          - Communication sequence: Start signal (MCU pulls low 1ms),
 *            Response (DHT pulls low 80us + high 80us), then 40 data bits.
 *          - Each bit: 50us low + 26-28us high (bit 0) or 70us high (bit 1).
 *          - Data format: [Humidity_H][Humidity_L][Temp_H][Temp_L][Checksum]
 *          - Resolution: 0.1C / 0.1% RH.
 *
 *          BH1750 - Ambient Light:
 *          - I2C digital light sensor, no analog conversion needed.
 *          - Continuous H-Resolution mode: 1 lux resolution, 120ms measurement.
 *          - Raw value conversion: lux = raw_value / 1.2
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "sensor_reader.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "sensor";

/** Active configuration. */
static sensor_reader_config_t s_config;

/** Whether the sensor reader has been initialized. */
static bool s_initialized = false;

/* -------------------------------------------------------------------------- */
/*                              BH1750 Commands                               */
/* -------------------------------------------------------------------------- */

#define BH1750_CMD_POWER_ON 0x01   /**< Power on the sensor. */
#define BH1750_CMD_RESET 0x07      /**< Reset data register. */
#define BH1750_CMD_CONT_H_RES 0x10 /**< Continuous H-Resolution mode. */

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static esp_err_t dht22_read(float *temperature, float *humidity);
static esp_err_t bh1750_init_sensor(void);
static esp_err_t bh1750_read_lux(float *lux);
static esp_err_t i2c_master_init(void);
static int64_t get_elapsed_us(int64_t start);

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

esp_err_t sensor_reader_init(const sensor_reader_config_t *config)
{
    if (config != NULL)
    {
        memcpy(&s_config, config, sizeof(sensor_reader_config_t));
    }
    else
    {
        /* Use default configuration. */
        s_config.dht22_gpio = SENSOR_DHT22_GPIO;
        s_config.i2c_port = SENSOR_I2C_PORT;
        s_config.i2c_sda_gpio = SENSOR_I2C_SDA_GPIO;
        s_config.i2c_scl_gpio = SENSOR_I2C_SCL_GPIO;
        s_config.i2c_freq_hz = SENSOR_I2C_FREQ_HZ;
        s_config.bh1750_addr = SENSOR_BH1750_ADDR;
    }

    /* Initialize I2C master for BH1750. */
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize BH1750 sensor. */
    ret = bh1750_init_sensor();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "BH1750 initialization failed (sensor may not be connected)");
        /* Non-fatal: continue without light sensor. */
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Sensor reader initialized (DHT22: GPIO%d, BH1750: I2C addr 0x%02X)",
             s_config.dht22_gpio, s_config.bh1750_addr);

    return ESP_OK;
}

esp_err_t sensor_reader_deinit(void)
{
    i2c_driver_delete(s_config.i2c_port);
    s_initialized = false;
    ESP_LOGI(TAG, "Sensor reader deinitialized");
    return ESP_OK;
}

esp_err_t sensor_reader_read_all(sensor_data_t *data)
{
    if (data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    memset(data, 0, sizeof(sensor_data_t));
    data->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    /* Read DHT22. */
    esp_err_t dht_ret = dht22_read(&data->temperature_c, &data->humidity_pct);
    data->dht22_valid = (dht_ret == ESP_OK);
    if (dht_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "DHT22 read failed: %s", esp_err_to_name(dht_ret));
    }

    /* Read BH1750. */
    esp_err_t bh_ret = bh1750_read_lux(&data->light_lux);
    data->bh1750_valid = (bh_ret == ESP_OK);
    if (bh_ret != ESP_OK)
    {
        ESP_LOGW(TAG, "BH1750 read failed: %s", esp_err_to_name(bh_ret));
    }

    /* Return OK if at least one sensor provided valid data. */
    return (data->dht22_valid || data->bh1750_valid) ? ESP_OK : ESP_FAIL;
}

int sensor_reader_to_json(const sensor_data_t *data, char *json_buf, size_t buf_len)
{
    if (data == NULL || json_buf == NULL || buf_len == 0)
    {
        return -1;
    }

    int written = snprintf(json_buf, buf_len,
                           "{"
                           "\"temperature\":%.1f,"
                           "\"humidity\":%.1f,"
                           "\"light\":%.1f,"
                           "\"timestamp\":%lu,"
                           "\"dht22_valid\":%s,"
                           "\"bh1750_valid\":%s"
                           "}",
                           data->temperature_c,
                           data->humidity_pct,
                           data->light_lux,
                           (unsigned long)data->timestamp_ms,
                           data->dht22_valid ? "true" : "false",
                           data->bh1750_valid ? "true" : "false");

    return (written >= (int)buf_len) ? -1 : written;
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Read temperature and humidity from the DHT22 sensor.
 *
 * @details DHT22 one-wire protocol timing:
 *
 *          Start Signal (MCU -> DHT):
 *            MCU pulls data line LOW for >= 1 ms, then releases (HIGH).
 *
 *          Response (DHT -> MCU):
 *            DHT pulls LOW for ~80 us, then HIGH for ~80 us.
 *
 *          Data Transfer (40 bits, MSB first):
 *            Each bit starts with ~50 us LOW, followed by:
 *              - HIGH for 26-28 us -> bit '0'
 *              - HIGH for ~70 us   -> bit '1'
 *
 *          Timing diagram:
 *
 *            MCU Start     DHT Response     Bit 0          Bit 1
 *          ___      ____________      __    __      __    ________
 *             |    |            |    |  |  |  |    |  |  |        |
 *             |____|            |____|  |__|  |____|  |__|        |____
 *           >1ms     ~80us ~80us  50us 26us   50us     70us
 *
 *          Data format (5 bytes):
 *            Byte 0: Humidity integer part
 *            Byte 1: Humidity decimal part
 *            Byte 2: Temperature integer part
 *            Byte 3: Temperature decimal part
 *            Byte 4: Checksum (sum of bytes 0-3)
 */
static esp_err_t dht22_read(float *temperature, float *humidity)
{
    uint8_t data[5] = {0};
    gpio_num_t pin = s_config.dht22_gpio;

    /*
     * Step 1: Send start signal.
     * Pull the data line LOW for at least 1 ms to wake the DHT22.
     */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
    ets_delay_us(1100); /* 1.1 ms LOW */
    gpio_set_level(pin, 1);
    ets_delay_us(30); /* 30 us HIGH (release) */

    /* Switch to input mode to read DHT response. */
    gpio_set_direction(pin, GPIO_MODE_INPUT);

    /*
     * Step 2: Wait for DHT response.
     * DHT pulls LOW for ~80 us, then HIGH for ~80 us.
     */
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(pin) == 0)
    {
        if (get_elapsed_us(start) > 100)
        {
            return ESP_ERR_TIMEOUT;
        }
    }

    start = esp_timer_get_time();
    while (gpio_get_level(pin) == 1)
    {
        if (get_elapsed_us(start) > 100)
        {
            return ESP_ERR_TIMEOUT;
        }
    }

    /*
     * Step 3: Read 40 data bits.
     * Each bit: ~50 us LOW followed by variable HIGH duration.
     */
    for (int i = 0; i < 40; i++)
    {
        /* Wait for the LOW period to end (~50 us). */
        start = esp_timer_get_time();
        while (gpio_get_level(pin) == 0)
        {
            if (get_elapsed_us(start) > 70)
            {
                return ESP_ERR_TIMEOUT;
            }
        }

        /* Measure the HIGH period duration. */
        start = esp_timer_get_time();
        while (gpio_get_level(pin) == 1)
        {
            if (get_elapsed_us(start) > 100)
            {
                return ESP_ERR_TIMEOUT;
            }
        }

        int64_t high_duration = get_elapsed_us(start);

        /* HIGH > 40 us indicates bit '1', otherwise bit '0'. */
        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8);

        if (high_duration > 40)
        {
            data[byte_idx] |= (1 << bit_idx);
        }
    }

    /*
     * Step 4: Verify checksum.
     * Checksum = (byte[0] + byte[1] + byte[2] + byte[3]) & 0xFF
     */
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (checksum != data[4])
    {
        ESP_LOGE(TAG, "DHT22 checksum mismatch: expected 0x%02X, got 0x%02X",
                 checksum, data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    /*
     * Step 5: Convert raw data to physical values.
     * Humidity: ((byte[0] << 8) | byte[1]) / 10.0
     * Temperature: ((byte[2] << 8) | byte[3]) / 10.0
     *   - Bit 15 of temperature indicates negative value.
     */
    uint16_t raw_hum = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_temp = ((uint16_t)data[2] << 8) | data[3];

    *humidity = raw_hum / 10.0f;

    if (raw_temp & 0x8000)
    {
        /* Negative temperature. */
        raw_temp &= 0x7FFF;
        *temperature = -(raw_temp / 10.0f);
    }
    else
    {
        *temperature = raw_temp / 10.0f;
    }

    ESP_LOGD(TAG, "DHT22: T=%.1fC, H=%.1f%%", *temperature, *humidity);
    return ESP_OK;
}

/**
 * @brief   Initialize the BH1750 ambient light sensor.
 *
 * @details Sends Power On command followed by Continuous H-Resolution Mode
 *          (0x10) command. In this mode, the sensor continuously measures
 *          ambient light at 1 lux resolution with ~120 ms measurement time.
 */
static esp_err_t bh1750_init_sensor(void)
{
    uint8_t cmd;

    /* Power on the sensor. */
    cmd = BH1750_CMD_POWER_ON;
    esp_err_t ret = i2c_master_write_to_device(
        s_config.i2c_port, s_config.bh1750_addr,
        &cmd, 1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Set Continuous H-Resolution Mode. */
    cmd = BH1750_CMD_CONT_H_RES;
    ret = i2c_master_write_to_device(
        s_config.i2c_port, s_config.bh1750_addr,
        &cmd, 1, pdMS_TO_TICKS(100));

    /* Wait for the first measurement to complete. */
    vTaskDelay(pdMS_TO_TICKS(180));

    return ret;
}

/**
 * @brief   Read ambient light level from BH1750.
 *
 * @details Reads 2 bytes from the sensor (MSB first) and converts to lux.
 *          Conversion formula: lux = raw_value / 1.2
 *
 *          The sensor automatically starts the next measurement after
 *          being read in continuous mode.
 */
static esp_err_t bh1750_read_lux(float *lux)
{
    uint8_t raw[2] = {0};

    esp_err_t ret = i2c_master_read_from_device(
        s_config.i2c_port, s_config.bh1750_addr,
        raw, 2, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    uint16_t raw_value = ((uint16_t)raw[0] << 8) | raw[1];
    *lux = raw_value / 1.2f;

    ESP_LOGD(TAG, "BH1750: %.1f lux (raw: %u)", *lux, raw_value);
    return ESP_OK;
}

/**
 * @brief   Initialize I2C master driver.
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = s_config.i2c_sda_gpio,
        .scl_io_num = s_config.i2c_scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = s_config.i2c_freq_hz,
    };

    esp_err_t ret = i2c_param_config(s_config.i2c_port, &i2c_cfg);
    if (ret != ESP_OK)
    {
        return ret;
    }

    return i2c_driver_install(s_config.i2c_port, I2C_MODE_MASTER, 0, 0, 0);
}

/**
 * @brief   Calculate elapsed microseconds from a start time.
 */
static int64_t get_elapsed_us(int64_t start)
{
    return esp_timer_get_time() - start;
}
