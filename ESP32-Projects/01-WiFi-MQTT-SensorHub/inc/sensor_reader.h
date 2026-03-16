/**
 * @file    sensor_reader.h
 * @brief   Multi-sensor reader for DHT22 (temperature/humidity) and BH1750 (light).
 *
 * @details Provides a unified interface for reading environmental sensor data.
 *          - DHT22: One-wire protocol for temperature (-40~80C) and humidity (0~100%).
 *          - BH1750: I2C digital ambient light sensor (1~65535 lux).
 *
 *          Sensor data is packaged into a single structure for convenient
 *          transmission over MQTT.
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef SENSOR_READER_H
#define SENSOR_READER_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** GPIO pin connected to DHT22 data line. */
#define SENSOR_DHT22_GPIO GPIO_NUM_4

/** I2C master port for BH1750. */
#define SENSOR_I2C_PORT I2C_NUM_0

/** I2C SDA pin. */
#define SENSOR_I2C_SDA_GPIO GPIO_NUM_21

/** I2C SCL pin. */
#define SENSOR_I2C_SCL_GPIO GPIO_NUM_22

/** I2C clock frequency in Hz. */
#define SENSOR_I2C_FREQ_HZ 100000

/** BH1750 I2C slave address (ADDR pin LOW). */
#define SENSOR_BH1750_ADDR 0x23

/** Sensor reading interval in milliseconds. */
#define SENSOR_READ_INTERVAL_MS 5000

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Sensor data packet structure.
     */
    typedef struct
    {
        float temperature_c;   /**< Temperature in Celsius (DHT22). */
        float humidity_pct;    /**< Relative humidity in percent (DHT22). */
        float light_lux;       /**< Ambient light in lux (BH1750). */
        uint32_t timestamp_ms; /**< Timestamp when data was read (ms since boot). */
        bool dht22_valid;      /**< True if DHT22 reading is valid. */
        bool bh1750_valid;     /**< True if BH1750 reading is valid. */
    } sensor_data_t;

    /**
     * @brief   Sensor reader configuration.
     */
    typedef struct
    {
        gpio_num_t dht22_gpio;   /**< GPIO pin for DHT22 data line. */
        i2c_port_t i2c_port;     /**< I2C port number for BH1750. */
        gpio_num_t i2c_sda_gpio; /**< I2C SDA pin. */
        gpio_num_t i2c_scl_gpio; /**< I2C SCL pin. */
        uint32_t i2c_freq_hz;    /**< I2C clock frequency. */
        uint8_t bh1750_addr;     /**< BH1750 I2C address. */
    } sensor_reader_config_t;

    /* -------------------------------------------------------------------------- */
    /*                          Public Function Prototypes                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize all sensors with default or custom configuration.
     *
     * @param[in]   config  Pointer to configuration, or NULL for defaults.
     * @return  ESP_OK on success.
     */
    esp_err_t sensor_reader_init(const sensor_reader_config_t *config);

    /**
     * @brief   Deinitialize sensors and release resources.
     *
     * @return  ESP_OK on success.
     */
    esp_err_t sensor_reader_deinit(void);

    /**
     * @brief   Read all sensor values into a sensor_data_t structure.
     *
     * @param[out]  data    Pointer to the data structure to fill.
     * @return  ESP_OK if at least one sensor read succeeded.
     */
    esp_err_t sensor_reader_read_all(sensor_data_t *data);

    /**
     * @brief   Format sensor data as a JSON string for MQTT publishing.
     *
     * @param[in]   data        Pointer to the sensor data structure.
     * @param[out]  json_buf    Buffer to store the JSON string.
     * @param[in]   buf_len     Length of the buffer.
     * @return  Number of characters written, or -1 on error.
     */
    int sensor_reader_to_json(const sensor_data_t *data, char *json_buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_READER_H */
