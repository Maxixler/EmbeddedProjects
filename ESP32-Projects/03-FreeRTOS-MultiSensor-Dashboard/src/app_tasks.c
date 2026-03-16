/**
 * @file    app_tasks.c
 * @brief   FreeRTOS task implementations for the Multi-Sensor Dashboard.
 *
 * @details Implements four application tasks and a software timer callback
 *          that together form a real-time sensor acquisition, processing,
 *          display, and alerting pipeline running on ESP32 under FreeRTOS.
 *
 *          Task Data Flow:
 *
 *          +----------------+  raw_queue  +----------------+ proc_queue +----------+
 *          | sensor_read    | ==========> | data_process   | =========> | display  |
 *          | (Pri 5, 100ms) |             | (Pri 4, blocks)|            | (Pri 2)  |
 *          +------+---------+             +-------+--------+            +----------+
 *                 |                               |
 *            I2C mutex                     event_group bits
 *            (BME280,                             |
 *             MPU6050)                    +-------+--------+
 *                 |                       | alert_task     |
 *                 v                       | (Pri 3, 250ms) |
 *          +-----------+                  +----------------+
 *          | I2C Bus   |                        |
 *          | BME280    |                  LED toggle
 *          | MPU6050   |
 *          | SSD1306   |
 *          +-----------+
 *
 *          Synchronisation Primitives:
 *          - g_i2c_mutex:          Protects the shared I2C bus (BME280,
 *                                  MPU6050, SSD1306 all on I2C_NUM_0).
 *          - g_sync_semaphore:     Binary semaphore used to gate the first
 *                                  sensor read until hardware init is done.
 *          - g_data_mutex:         Protects g_latest_processed so that
 *                                  alert_task can safely read the latest
 *                                  processed data snapshot.
 *          - g_alert_event_group:  Carries per-alert-condition bits from
 *                                  data_process_task to alert_task.
 *          - g_watchdog_timer:     Software timer that fires every 5 s to
 *                                  verify that the sensor pipeline is alive.
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "app_tasks.h"
#include "sensor_fusion.h"
#include "display_driver.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

/* -------------------------------------------------------------------------- */
/*                              Configuration                                 */
/* -------------------------------------------------------------------------- */

/** @name GPIO Pin Assignments
 *  @{ */
#define PIN_I2C_SDA (GPIO_NUM_21)    /**< I2C data line. */
#define PIN_I2C_SCL (GPIO_NUM_22)    /**< I2C clock line. */
#define PIN_LED_ALERT (GPIO_NUM_2)   /**< On-board LED for alerts. */
#define PIN_ADC_POT (ADC1_CHANNEL_6) /**< GPIO34 - potentiometer. */
/** @} */

/** @name I2C Configuration
 *  @{ */
#define I2C_MASTER_PORT (I2C_NUM_0)          /**< I2C peripheral number. */
#define I2C_MASTER_FREQ_HZ (100000)          /**< I2C clock frequency. */
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(100) /**< I2C operation timeout. */
/** @} */

/** @name Sensor I2C Addresses
 *  @{ */
#define BME280_I2C_ADDR (0x76)  /**< BME280 default address. */
#define MPU6050_I2C_ADDR (0x68) /**< MPU6050 default address. */
/** @} */

/** @name BME280 Register Addresses
 *  @{ */
#define BME280_REG_CHIP_ID (0xD0)   /**< Chip ID register. */
#define BME280_REG_CTRL_HUM (0xF2)  /**< Humidity control. */
#define BME280_REG_CTRL_MEAS (0xF4) /**< Measurement control. */
#define BME280_REG_CONFIG (0xF5)    /**< Configuration. */
#define BME280_REG_PRESS_MSB (0xF7) /**< Pressure data start. */
#define BME280_CHIP_ID_VALUE (0x60) /**< Expected chip ID. */
#define BME280_DATA_LEN (8)         /**< Burst read length. */
/** @} */

/** @name MPU6050 Register Addresses
 *  @{ */
#define MPU6050_REG_WHO_AM_I (0x75)     /**< WHO_AM_I register. */
#define MPU6050_REG_PWR_MGMT_1 (0x6B)   /**< Power management 1. */
#define MPU6050_REG_ACCEL_XOUT_H (0x3B) /**< Accel X high byte. */
#define MPU6050_REG_GYRO_CONFIG (0x1B)  /**< Gyroscope config. */
#define MPU6050_REG_ACCEL_CONFIG (0x1C) /**< Accelerometer config. */
#define MPU6050_WHO_AM_I_VALUE (0x68)   /**< Expected WHO_AM_I. */
#define MPU6050_DATA_LEN (14)           /**< Burst read length. */
/** @} */

/** @name ADC Configuration
 *  @{ */
#define ADC_ATTEN ADC_ATTEN_DB_11  /**< Full-scale 3.3V. */
#define ADC_WIDTH ADC_WIDTH_BIT_12 /**< 12-bit resolution. */
#define ADC_VREF_MV (3300)         /**< Reference voltage (mV). */
/** @} */

/** @name Alert Threshold Defaults
 *  @{ */
#define DEFAULT_TEMP_HIGH_C (35.0f)
#define DEFAULT_TEMP_LOW_C (5.0f)
#define DEFAULT_HUMIDITY_HIGH_PCT (80.0f)
#define DEFAULT_PRESSURE_LOW_HPA (980.0f)
#define DEFAULT_TILT_MAX_DEG (45.0f)
#define DEFAULT_HYST_TEMP_C (1.0f)
#define DEFAULT_HYST_HUMIDITY_PCT (2.0f)
#define DEFAULT_HYST_PRESSURE_HPA (5.0f)
#define DEFAULT_HYST_TILT_DEG (3.0f)
/** @} */

/** Sensor fusion moving average window size. */
#define FUSION_WINDOW_SIZE (10)

/** Complementary filter alpha coefficient. */
#define FUSION_ALPHA (0.96f)

/** MPU6050 sensitivity scale factors (default full-scale range). */
#define ACCEL_SENSITIVITY (16384.0f) /**< LSB/g   for +-2g. */
#define GYRO_SENSITIVITY (131.0f)    /**< LSB/dps for +-250 dps. */

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "app_tasks";

/* -------------------------------------------------------------------------- */
/*                         Global Handle Definitions                          */
/* -------------------------------------------------------------------------- */

/** @name Inter-task Communication Handles
 *  @{ */
QueueHandle_t g_raw_data_queue = NULL;
QueueHandle_t g_processed_data_queue = NULL;
SemaphoreHandle_t g_i2c_mutex = NULL;
SemaphoreHandle_t g_sync_semaphore = NULL;
EventGroupHandle_t g_alert_event_group = NULL;
TimerHandle_t g_watchdog_timer = NULL;
/** @} */

/** @name Latest Processed Data (protected by g_data_mutex)
 *  @{ */
processed_sensor_data_t g_latest_processed = {0};
SemaphoreHandle_t g_data_mutex = NULL;
/** @} */

/* -------------------------------------------------------------------------- */
/*                    Sensor Fusion Filter Contexts (Static)                  */
/* -------------------------------------------------------------------------- */

/** Moving average filters for each scalar sensor value. */
static moving_avg_t s_avg_temp;
static moving_avg_t s_avg_humid;
static moving_avg_t s_avg_press;
static moving_avg_t s_avg_adc;

/** Complementary filter for roll/pitch orientation. */
static comp_filter_t s_comp_filter;

/** Threshold detectors with hysteresis. */
static threshold_detector_t s_thresh_temp_high;
static threshold_detector_t s_thresh_temp_low;
static threshold_detector_t s_thresh_humid_high;
static threshold_detector_t s_thresh_press_low;
static threshold_detector_t s_thresh_tilt;

/** Running sample counter (for watchdog monitoring). */
static volatile uint32_t s_sample_counter = 0;

/** Previous sample counter snapshot (used by watchdog). */
static uint32_t s_last_watchdog_count = 0;

/** Display driver context. */
static ssd1306_t s_display;

/** Previous timestamp for dt calculation in complementary filter. */
static uint32_t s_prev_timestamp_ms = 0;

/** Alert threshold configuration. */
static alert_thresholds_t s_thresholds = {
    .temp_high_c = DEFAULT_TEMP_HIGH_C,
    .temp_low_c = DEFAULT_TEMP_LOW_C,
    .humidity_high_pct = DEFAULT_HUMIDITY_HIGH_PCT,
    .pressure_low_hpa = DEFAULT_PRESSURE_LOW_HPA,
    .tilt_max_deg = DEFAULT_TILT_MAX_DEG,
    .hysteresis_c = DEFAULT_HYST_TEMP_C,
    .hysteresis_pct = DEFAULT_HYST_HUMIDITY_PCT,
    .hysteresis_hpa = DEFAULT_HYST_PRESSURE_HPA,
    .hysteresis_deg = DEFAULT_HYST_TILT_DEG,
};

/** Task handles for stack monitoring. */
static TaskHandle_t s_sensor_task_handle = NULL;
static TaskHandle_t s_process_task_handle = NULL;
static TaskHandle_t s_display_task_handle = NULL;
static TaskHandle_t s_alert_task_handle = NULL;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static esp_err_t prv_i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t data);
static esp_err_t prv_i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t *data, size_t len);
static esp_err_t prv_bme280_init(void);
static esp_err_t prv_bme280_read(float *temp, float *humid, float *press);
static esp_err_t prv_mpu6050_init(void);
static esp_err_t prv_mpu6050_read(float *ax, float *ay, float *az,
                                  float *gx, float *gy, float *gz);
static void prv_adc_init(void);
static uint16_t prv_adc_read_raw(void);
static void prv_led_init(void);
static void prv_led_toggle(void);
static void prv_init_fusion_filters(void);

/* -------------------------------------------------------------------------- */
/*                     Low-Level I2C Helper Functions                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Write a single byte to a register on an I2C slave device.
 *
 * @param[in]   dev_addr    7-bit I2C slave address.
 * @param[in]   reg_addr    Register address to write.
 * @param[in]   data        Byte value to write.
 * @return  ESP_OK on success.
 *
 * @note    The caller must already hold the I2C mutex.
 */
static esp_err_t prv_i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t data)
{
    uint8_t buf[2] = {reg_addr, data};

    return i2c_master_write_to_device(
        I2C_MASTER_PORT, dev_addr,
        buf, sizeof(buf),
        I2C_TIMEOUT_TICKS);
}

/**
 * @brief   Read multiple bytes from consecutive registers on an I2C device.
 *
 * @param[in]   dev_addr    7-bit I2C slave address.
 * @param[in]   reg_addr    Starting register address.
 * @param[out]  data        Buffer to store read bytes.
 * @param[in]   len         Number of bytes to read.
 * @return  ESP_OK on success.
 *
 * @note    The caller must already hold the I2C mutex.
 */
static esp_err_t prv_i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_PORT, dev_addr,
        &reg_addr, 1,
        data, len,
        I2C_TIMEOUT_TICKS);
}

/* -------------------------------------------------------------------------- */
/*                     BME280 Driver (Simplified)                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise the BME280 temperature/humidity/pressure sensor.
 *
 * @details Verifies the chip ID, then configures forced mode with 1x
 *          oversampling for all three measurements.  A simplified
 *          compensation approach is used: the raw 20-bit ADC values are
 *          converted using approximate linear scaling.
 *
 *          For production use, the full factory calibration data from
 *          registers 0x88-0xA1 and 0xE1-0xE7 should be read and applied.
 *
 * @return  ESP_OK on success, ESP_FAIL if chip ID mismatch.
 *
 * @note    Caller must hold the I2C mutex.
 */
static esp_err_t prv_bme280_init(void)
{
    uint8_t chip_id = 0;
    esp_err_t ret = prv_i2c_read_bytes(BME280_I2C_ADDR, BME280_REG_CHIP_ID,
                                       &chip_id, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BME280: Failed to read chip ID");
        return ret;
    }

    if (chip_id != BME280_CHIP_ID_VALUE)
    {
        ESP_LOGW(TAG, "BME280: Unexpected chip ID 0x%02X (expected 0x%02X)",
                 chip_id, BME280_CHIP_ID_VALUE);
        /* Continue anyway - could be a BMP280 (0x58). */
    }
    else
    {
        ESP_LOGI(TAG, "BME280: Chip ID verified (0x%02X)", chip_id);
    }

    /* Configure humidity oversampling: 1x. */
    ret = prv_i2c_write_byte(BME280_I2C_ADDR, BME280_REG_CTRL_HUM, 0x01);
    if (ret != ESP_OK)
        return ret;

    /* Configure measurement: temp 1x OS, press 1x OS, forced mode. */
    ret = prv_i2c_write_byte(BME280_I2C_ADDR, BME280_REG_CTRL_MEAS, 0x27);
    if (ret != ESP_OK)
        return ret;

    /* Configuration: standby 1000ms, filter off. */
    ret = prv_i2c_write_byte(BME280_I2C_ADDR, BME280_REG_CONFIG, 0xA0);

    ESP_LOGI(TAG, "BME280: Configured (temp+humid+press, normal mode)");
    return ret;
}

/**
 * @brief   Read temperature, humidity, and pressure from the BME280.
 *
 * @details Performs a burst read of 8 bytes starting at register 0xF7
 *          (press_msb through hum_lsb).  Raw 20-bit pressure and
 *          temperature values and 16-bit humidity are extracted and
 *          converted using approximate linear formulas.
 *
 *          Note: For accurate readings, the full compensation algorithm
 *          from the BME280 datasheet (using factory trim values) should
 *          be implemented.  The simplified conversion here provides
 *          reasonable results for demonstration purposes.
 *
 * @note    Caller must hold the I2C mutex.
 */
static esp_err_t prv_bme280_read(float *temp, float *humid, float *press)
{
    uint8_t data[BME280_DATA_LEN];

    /* Trigger a forced measurement. */
    esp_err_t ret = prv_i2c_write_byte(BME280_I2C_ADDR,
                                       BME280_REG_CTRL_MEAS, 0x25);
    if (ret != ESP_OK)
        return ret;

    /* Wait for measurement to complete. */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Burst read: press[19:0], temp[19:0], hum[15:0]. */
    ret = prv_i2c_read_bytes(BME280_I2C_ADDR, BME280_REG_PRESS_MSB,
                             data, BME280_DATA_LEN);
    if (ret != ESP_OK)
        return ret;

    /*
     * Extract raw ADC values.
     *
     * Pressure:    data[0..2] -> 20-bit unsigned (MSB, LSB, XLSB>>4)
     * Temperature: data[3..5] -> 20-bit unsigned
     * Humidity:    data[6..7] -> 16-bit unsigned
     */
    int32_t raw_press = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    int32_t raw_temp = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    int32_t raw_humid = ((int32_t)data[6] << 8) | (int32_t)data[7];

    /*
     * Simplified conversion (approximate, for demonstration).
     *
     * These formulas are linear approximations.  For production:
     * use the official Bosch compensation routines with the
     * factory calibration data stored in NVM registers.
     */
    *temp = (float)(raw_temp - 409600) / 8192.0f;
    *press = (float)raw_press / 256.0f + 300.0f;
    *humid = (float)raw_humid / 512.0f;

    /* Clamp humidity to valid range. */
    if (*humid > 100.0f)
        *humid = 100.0f;
    if (*humid < 0.0f)
        *humid = 0.0f;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                     MPU6050 Driver (Simplified)                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise the MPU6050 6-axis IMU.
 *
 * @details Wakes the device from sleep mode (clears bit 6 of PWR_MGMT_1)
 *          and configures the default full-scale ranges:
 *            - Accelerometer: +/-2g (sensitivity = 16384 LSB/g)
 *            - Gyroscope:     +/-250 deg/s (sensitivity = 131 LSB/dps)
 *
 * @return  ESP_OK on success, ESP_FAIL on WHO_AM_I mismatch.
 *
 * @note    Caller must hold the I2C mutex.
 */
static esp_err_t prv_mpu6050_init(void)
{
    uint8_t who_am_i = 0;
    esp_err_t ret = prv_i2c_read_bytes(MPU6050_I2C_ADDR,
                                       MPU6050_REG_WHO_AM_I,
                                       &who_am_i, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050: Failed to read WHO_AM_I");
        return ret;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE)
    {
        ESP_LOGW(TAG, "MPU6050: Unexpected WHO_AM_I 0x%02X (expected 0x%02X)",
                 who_am_i, MPU6050_WHO_AM_I_VALUE);
    }
    else
    {
        ESP_LOGI(TAG, "MPU6050: WHO_AM_I verified (0x%02X)", who_am_i);
    }

    /* Wake up the device (clear sleep bit). */
    ret = prv_i2c_write_byte(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK)
        return ret;

    /* Configure gyroscope: +/-250 deg/s. */
    ret = prv_i2c_write_byte(MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK)
        return ret;

    /* Configure accelerometer: +/-2g. */
    ret = prv_i2c_write_byte(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00);

    ESP_LOGI(TAG, "MPU6050: Configured (accel +/-2g, gyro +/-250dps)");
    return ret;
}

/**
 * @brief   Read accelerometer and gyroscope data from the MPU6050.
 *
 * @details Performs a 14-byte burst read starting at register 0x3B:
 *          ACCEL_XOUT[H,L], ACCEL_YOUT[H,L], ACCEL_ZOUT[H,L],
 *          TEMP_OUT[H,L] (skipped),
 *          GYRO_XOUT[H,L], GYRO_YOUT[H,L], GYRO_ZOUT[H,L].
 *
 *          Raw 16-bit signed values are converted to physical units
 *          using the sensitivity scale factors for the configured
 *          full-scale range.
 *
 * @note    Caller must hold the I2C mutex.
 */
static esp_err_t prv_mpu6050_read(float *ax, float *ay, float *az,
                                  float *gx, float *gy, float *gz)
{
    uint8_t data[MPU6050_DATA_LEN];

    esp_err_t ret = prv_i2c_read_bytes(MPU6050_I2C_ADDR,
                                       MPU6050_REG_ACCEL_XOUT_H,
                                       data, MPU6050_DATA_LEN);
    if (ret != ESP_OK)
        return ret;

    /* Extract raw 16-bit signed values (big-endian). */
    int16_t raw_ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t raw_ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t raw_az = (int16_t)((data[4] << 8) | data[5]);
    /* data[6..7] = temperature (skipped). */
    int16_t raw_gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t raw_gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t raw_gz = (int16_t)((data[12] << 8) | data[13]);

    /* Convert to physical units. */
    *ax = (float)raw_ax / ACCEL_SENSITIVITY;
    *ay = (float)raw_ay / ACCEL_SENSITIVITY;
    *az = (float)raw_az / ACCEL_SENSITIVITY;
    *gx = (float)raw_gx / GYRO_SENSITIVITY;
    *gy = (float)raw_gy / GYRO_SENSITIVITY;
    *gz = (float)raw_gz / GYRO_SENSITIVITY;

    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                         ADC Helper Functions                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise the ADC for potentiometer reading.
 *
 * @details Configures ADC1 channel 6 (GPIO34) with 12-bit resolution
 *          and 11dB attenuation for full 0-3.3V range.
 */
static void prv_adc_init(void)
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(PIN_ADC_POT, ADC_ATTEN);
    ESP_LOGI(TAG, "ADC: Configured (channel 6, GPIO34, 12-bit, 11dB atten)");
}

/**
 * @brief   Read the raw 12-bit ADC value from the potentiometer.
 *
 * @return  12-bit ADC value (0 - 4095).
 */
static uint16_t prv_adc_read_raw(void)
{
    return (uint16_t)adc1_get_raw(PIN_ADC_POT);
}

/* -------------------------------------------------------------------------- */
/*                         LED Helper Functions                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise the alert LED GPIO as output.
 */
static void prv_led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LED_ALERT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_LED_ALERT, 0);
    ESP_LOGI(TAG, "LED: Alert LED configured (GPIO%d)", PIN_LED_ALERT);
}

/**
 * @brief   Toggle the alert LED state.
 */
static void prv_led_toggle(void)
{
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level(PIN_LED_ALERT, led_state ? 1 : 0);
}

/* -------------------------------------------------------------------------- */
/*                     Sensor Fusion Initialisation                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise all sensor fusion filter contexts.
 *
 * @details Sets up:
 *          - Four moving average filters (temp, humidity, pressure, ADC)
 *            with FUSION_WINDOW_SIZE samples.
 *          - One complementary filter for roll/pitch with FUSION_ALPHA.
 *          - Five threshold detectors with configured thresholds and
 *            hysteresis bands.
 */
static void prv_init_fusion_filters(void)
{
    /* Moving average filters. */
    moving_avg_init(&s_avg_temp, FUSION_WINDOW_SIZE);
    moving_avg_init(&s_avg_humid, FUSION_WINDOW_SIZE);
    moving_avg_init(&s_avg_press, FUSION_WINDOW_SIZE);
    moving_avg_init(&s_avg_adc, FUSION_WINDOW_SIZE);

    /* Complementary filter. */
    comp_filter_init(&s_comp_filter, FUSION_ALPHA);

    /* Threshold detectors. */
    threshold_init(&s_thresh_temp_high, s_thresholds.temp_high_c,
                   s_thresholds.hysteresis_c, true);

    threshold_init(&s_thresh_temp_low, s_thresholds.temp_low_c,
                   s_thresholds.hysteresis_c, false);

    threshold_init(&s_thresh_humid_high, s_thresholds.humidity_high_pct,
                   s_thresholds.hysteresis_pct, true);

    threshold_init(&s_thresh_press_low, s_thresholds.pressure_low_hpa,
                   s_thresholds.hysteresis_hpa, false);

    threshold_init(&s_thresh_tilt, s_thresholds.tilt_max_deg,
                   s_thresholds.hysteresis_deg, true);

    ESP_LOGI(TAG, "Fusion: Filters initialised (window=%d, alpha=%.2f)",
             FUSION_WINDOW_SIZE, FUSION_ALPHA);
}

/* -------------------------------------------------------------------------- */
/*                     Task Implementations                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Sensor reading task - acquires data from ADC, BME280, and MPU6050.
 *
 * @details Runs periodically at SENSOR_READ_PERIOD_MS (100 ms = 10 Hz).
 *
 *          Each iteration:
 *          1. Read the potentiometer via ADC (no mutex needed).
 *          2. Acquire g_i2c_mutex.
 *          3. Read BME280 (temperature, humidity, pressure).
 *          4. Read MPU6050 (accelerometer, gyroscope).
 *          5. Release g_i2c_mutex.
 *          6. Package all readings into a raw_sensor_data_t structure.
 *          7. Send the packet to g_raw_data_queue (non-blocking).
 *          8. Increment the sample counter for watchdog monitoring.
 *          9. Delay until the next period.
 */
void sensor_read_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "sensor_read_task: Started (period=%d ms, pri=%d)",
             SENSOR_READ_PERIOD_MS, TASK_PRIORITY_SENSOR_READ);

    /* Wait for sync semaphore (hardware init complete). */
    xSemaphoreTake(g_sync_semaphore, portMAX_DELAY);
    ESP_LOGI(TAG, "sensor_read_task: Sync semaphore received, starting reads");

    while (1)
    {
        raw_sensor_data_t raw = {0};
        raw.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /*
         * Step 1: Read ADC (potentiometer).
         * ADC does not use I2C, so no mutex needed.
         */
        raw.adc_raw = prv_adc_read_raw();
        raw.adc_voltage = (float)raw.adc_raw * 3.3f / 4095.0f;

        /*
         * Step 2: Acquire I2C mutex for sensor reads.
         */
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            /*
             * Step 3: Read BME280.
             */
            esp_err_t bme_ret = prv_bme280_read(&raw.temperature_c,
                                                &raw.humidity_pct,
                                                &raw.pressure_hpa);
            raw.bme280_valid = (bme_ret == ESP_OK);
            if (bme_ret != ESP_OK)
            {
                ESP_LOGW(TAG, "BME280 read failed: %s", esp_err_to_name(bme_ret));
            }

            /*
             * Step 4: Read MPU6050.
             */
            esp_err_t mpu_ret = prv_mpu6050_read(&raw.accel_x, &raw.accel_y,
                                                 &raw.accel_z,
                                                 &raw.gyro_x, &raw.gyro_y,
                                                 &raw.gyro_z);
            raw.mpu6050_valid = (mpu_ret == ESP_OK);
            if (mpu_ret != ESP_OK)
            {
                ESP_LOGW(TAG, "MPU6050 read failed: %s", esp_err_to_name(mpu_ret));
            }

            /*
             * Step 5: Release I2C mutex.
             */
            xSemaphoreGive(g_i2c_mutex);
        }
        else
        {
            ESP_LOGW(TAG, "sensor_read_task: I2C mutex timeout");
            raw.bme280_valid = false;
            raw.mpu6050_valid = false;
        }

        /*
         * Step 6-7: Send to raw data queue (non-blocking).
         */
        if (xQueueSend(g_raw_data_queue, &raw, 0) != pdTRUE)
        {
            ESP_LOGW(TAG, "sensor_read_task: Raw queue full, sample dropped");
        }

        /*
         * Step 8: Increment sample counter for watchdog.
         */
        s_sample_counter++;

        /*
         * Step 9: Delay until next period.
         */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_READ_PERIOD_MS));
    }
}

/**
 * @brief   Data processing task - applies sensor fusion algorithms.
 *
 * @details Blocks on g_raw_data_queue waiting for raw sensor packets.
 *          For each packet received:
 *
 *          1. Feed scalar values into their respective moving average
 *             filters (temperature, humidity, pressure, ADC voltage).
 *          2. Compute the time delta (dt) from the previous sample and
 *             update the complementary filter with accelerometer and
 *             gyroscope data to produce fused roll/pitch/tilt angles.
 *          3. Evaluate each threshold detector with the filtered values
 *             and build the alert flag bitmask.
 *          4. Set corresponding bits in g_alert_event_group so that
 *             alert_task can respond.
 *          5. Pack the results into a processed_sensor_data_t and send
 *             to g_processed_data_queue for the display task.
 *          6. Copy the latest processed data to g_latest_processed under
 *             g_data_mutex for the alert task to inspect.
 */
void data_process_task(void *pvParameters)
{
    (void)pvParameters;

    raw_sensor_data_t raw;
    uint32_t sample_num = 0;

    ESP_LOGI(TAG, "data_process_task: Started (pri=%d)",
             TASK_PRIORITY_DATA_PROCESS);

    /* Initialise fusion filters. */
    prv_init_fusion_filters();

    while (1)
    {
        /* Block until a raw packet arrives. */
        if (xQueueReceive(g_raw_data_queue, &raw, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        processed_sensor_data_t proc = {0};
        proc.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        proc.sample_count = ++sample_num;

        /*
         * Step 1: Update moving average filters.
         */
        if (raw.bme280_valid)
        {
            proc.temperature_avg = moving_avg_update(&s_avg_temp,
                                                     raw.temperature_c);
            proc.humidity_avg = moving_avg_update(&s_avg_humid,
                                                  raw.humidity_pct);
            proc.pressure_avg = moving_avg_update(&s_avg_press,
                                                  raw.pressure_hpa);
        }
        else
        {
            proc.temperature_avg = moving_avg_get(&s_avg_temp);
            proc.humidity_avg = moving_avg_get(&s_avg_humid);
            proc.pressure_avg = moving_avg_get(&s_avg_press);
        }

        proc.adc_voltage_avg = moving_avg_update(&s_avg_adc, raw.adc_voltage);

        /*
         * Step 2: Update complementary filter for orientation.
         */
        if (raw.mpu6050_valid)
        {
            /* Compute dt in seconds. */
            float dt;
            if (s_prev_timestamp_ms == 0)
            {
                dt = (float)SENSOR_READ_PERIOD_MS / 1000.0f;
            }
            else
            {
                dt = (float)(raw.timestamp_ms - s_prev_timestamp_ms) / 1000.0f;
            }
            s_prev_timestamp_ms = raw.timestamp_ms;

            comp_filter_update(&s_comp_filter,
                               raw.accel_x, raw.accel_y, raw.accel_z,
                               raw.gyro_x, raw.gyro_y,
                               dt);

            proc.roll_deg = comp_filter_get_roll(&s_comp_filter);
            proc.pitch_deg = comp_filter_get_pitch(&s_comp_filter);
            proc.tilt_deg = comp_filter_get_tilt(&s_comp_filter);
        }
        else
        {
            proc.roll_deg = comp_filter_get_roll(&s_comp_filter);
            proc.pitch_deg = comp_filter_get_pitch(&s_comp_filter);
            proc.tilt_deg = comp_filter_get_tilt(&s_comp_filter);
        }

        /*
         * Step 3: Evaluate alert thresholds.
         */
        proc.alert_flags = 0;

        if (threshold_evaluate(&s_thresh_temp_high, proc.temperature_avg))
        {
            proc.alert_flags |= ALERT_BIT_TEMP_HIGH;
        }
        if (threshold_evaluate(&s_thresh_temp_low, proc.temperature_avg))
        {
            proc.alert_flags |= ALERT_BIT_TEMP_LOW;
        }
        if (threshold_evaluate(&s_thresh_humid_high, proc.humidity_avg))
        {
            proc.alert_flags |= ALERT_BIT_HUMIDITY_HIGH;
        }
        if (threshold_evaluate(&s_thresh_press_low, proc.pressure_avg))
        {
            proc.alert_flags |= ALERT_BIT_PRESSURE_LOW;
        }
        if (threshold_evaluate(&s_thresh_tilt, proc.tilt_deg))
        {
            proc.alert_flags |= ALERT_BIT_TILT_EXCEED;
        }

        /*
         * Step 4: Signal alert_task via event group.
         */
        if (proc.alert_flags != 0)
        {
            xEventGroupSetBits(g_alert_event_group, proc.alert_flags);
        }

        /*
         * Step 5: Send processed data to display queue.
         */
        if (xQueueSend(g_processed_data_queue, &proc, 0) != pdTRUE)
        {
            ESP_LOGD(TAG, "data_process_task: Processed queue full");
        }

        /*
         * Step 6: Update global latest processed data snapshot.
         */
        if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(&g_latest_processed, &proc, sizeof(processed_sensor_data_t));
            xSemaphoreGive(g_data_mutex);
        }

        ESP_LOGD(TAG, "Proc #%lu: T=%.1f H=%.1f P=%.0f Tilt=%.1f Flags=0x%02lX",
                 (unsigned long)sample_num,
                 proc.temperature_avg, proc.humidity_avg,
                 proc.pressure_avg, proc.tilt_deg,
                 (unsigned long)proc.alert_flags);
    }
}

/**
 * @brief   Display task - renders processed data on SSD1306 OLED.
 *
 * @details Blocks on g_processed_data_queue for new processed packets.
 *          For each packet:
 *
 *          1. Update the dashboard values in the frame buffer
 *             (temperature, humidity, pressure, tilt, ADC, status).
 *          2. Acquire g_i2c_mutex.
 *          3. Flush the frame buffer to the SSD1306 over I2C.
 *          4. Release g_i2c_mutex.
 *
 *          The display is initialised once on the first iteration.
 *          A minimum refresh interval of DISPLAY_REFRESH_PERIOD_MS
 *          is enforced by discarding intermediate packets if they
 *          arrive faster than the display can be updated.
 */
void display_task(void *pvParameters)
{
    (void)pvParameters;

    bool display_ready = false;
    processed_sensor_data_t proc;
    TickType_t last_refresh = 0;

    ESP_LOGI(TAG, "display_task: Started (refresh=%d ms, pri=%d)",
             DISPLAY_REFRESH_PERIOD_MS, TASK_PRIORITY_DISPLAY);

    /* Initialise the OLED display. */
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
    {
        esp_err_t ret = ssd1306_init(&s_display);
        if (ret == ESP_OK)
        {
            ssd1306_draw_dashboard_frame(&s_display);
            ssd1306_dash_set_status(&s_display, "Init...");
            ssd1306_flush(&s_display);
            display_ready = true;
            ESP_LOGI(TAG, "display_task: OLED initialised");
        }
        else
        {
            ESP_LOGE(TAG, "display_task: OLED init failed");
        }
        xSemaphoreGive(g_i2c_mutex);
    }

    while (1)
    {
        /* Block until processed data is available. */
        if (xQueueReceive(g_processed_data_queue, &proc,
                          portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        if (!display_ready)
        {
            continue;
        }

        /* Enforce minimum refresh interval. */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_refresh) < pdMS_TO_TICKS(DISPLAY_REFRESH_PERIOD_MS))
        {
            /* Skip this frame - display cannot keep up. */
            continue;
        }
        last_refresh = now;

        /*
         * Update dashboard values in the frame buffer.
         */
        ssd1306_dash_set_temperature(&s_display, proc.temperature_avg);
        ssd1306_dash_set_humidity(&s_display, proc.humidity_avg);
        ssd1306_dash_set_pressure(&s_display, proc.pressure_avg);
        ssd1306_dash_set_tilt(&s_display, proc.tilt_deg);
        ssd1306_dash_set_adc(&s_display, proc.adc_voltage_avg);

        /* Update status line based on alert flags. */
        if (proc.alert_flags & ALERT_BIT_TEMP_HIGH)
        {
            ssd1306_dash_set_status(&s_display, "TEMP HIGH!");
        }
        else if (proc.alert_flags & ALERT_BIT_TEMP_LOW)
        {
            ssd1306_dash_set_status(&s_display, "TEMP LOW!");
        }
        else if (proc.alert_flags & ALERT_BIT_HUMIDITY_HIGH)
        {
            ssd1306_dash_set_status(&s_display, "HUMID HIGH!");
        }
        else if (proc.alert_flags & ALERT_BIT_PRESSURE_LOW)
        {
            ssd1306_dash_set_status(&s_display, "PRESS LOW!");
        }
        else if (proc.alert_flags & ALERT_BIT_TILT_EXCEED)
        {
            ssd1306_dash_set_status(&s_display, "TILT WARN!");
        }
        else
        {
            ssd1306_dash_set_status(&s_display, "OK");
        }

        /* Flush frame buffer to OLED (requires I2C mutex). */
        if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            ssd1306_flush(&s_display);
            xSemaphoreGive(g_i2c_mutex);
        }
        else
        {
            ESP_LOGW(TAG, "display_task: I2C mutex timeout during flush");
        }
    }
}

/**
 * @brief   Alert monitoring task - evaluates event group and drives LED.
 *
 * @details Waits on g_alert_event_group for any ALERT_BIT_xxx bits to be
 *          set.  Uses a timeout of ALERT_CHECK_PERIOD_MS so that the task
 *          also runs periodically even when no alerts are active.
 *
 *          When any alert bit is set:
 *          1. Identify which alerts are active and log them.
 *          2. Toggle the alert LED to create a visual warning.
 *          3. Clear the handled bits in the event group.
 *
 *          When no alerts are active:
 *          1. Ensure the LED is turned off.
 */
void alert_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "alert_task: Started (period=%d ms, pri=%d)",
             ALERT_CHECK_PERIOD_MS, TASK_PRIORITY_ALERT);

    while (1)
    {
        /*
         * Wait for any alert bit, with timeout for periodic check.
         * xClearOnExit = pdFALSE so we can inspect which bits are set
         * before clearing them manually.
         */
        EventBits_t bits = xEventGroupWaitBits(
            g_alert_event_group,
            ALERT_BIT_ANY,
            pdFALSE, /* Do not auto-clear on exit. */
            pdFALSE, /* Wait for ANY bit, not all. */
            pdMS_TO_TICKS(ALERT_CHECK_PERIOD_MS));

        if (bits & ALERT_BIT_ANY)
        {
            /* At least one alert is active. */
            if (bits & ALERT_BIT_TEMP_HIGH)
            {
                ESP_LOGW(TAG, "ALERT: Temperature HIGH exceeded threshold");
            }
            if (bits & ALERT_BIT_TEMP_LOW)
            {
                ESP_LOGW(TAG, "ALERT: Temperature LOW below threshold");
            }
            if (bits & ALERT_BIT_HUMIDITY_HIGH)
            {
                ESP_LOGW(TAG, "ALERT: Humidity HIGH exceeded threshold");
            }
            if (bits & ALERT_BIT_PRESSURE_LOW)
            {
                ESP_LOGW(TAG, "ALERT: Pressure LOW below threshold");
            }
            if (bits & ALERT_BIT_TILT_EXCEED)
            {
                ESP_LOGW(TAG, "ALERT: Tilt angle exceeded threshold");
            }
            if (bits & ALERT_BIT_WATCHDOG)
            {
                ESP_LOGE(TAG, "ALERT: Watchdog - sensor pipeline stalled!");
            }

            /* Toggle the alert LED. */
            prv_led_toggle();

            /* Clear the handled bits. */
            xEventGroupClearBits(g_alert_event_group, bits & ALERT_BIT_ANY);
        }
        else
        {
            /* No alerts - ensure LED is off. */
            gpio_set_level(PIN_LED_ALERT, 0);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                      Software Timer Callback                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Watchdog timer callback - monitors the sensor pipeline health.
 *
 * @details Called every WATCHDOG_TIMER_PERIOD_MS (5000 ms) by the FreeRTOS
 *          timer daemon task.  Checks that the sample counter has been
 *          incremented since the last invocation.  If the count has not
 *          changed, the sensor_read_task may be stalled, so the
 *          ALERT_BIT_WATCHDOG bit is set in the event group.
 *
 *          This also logs the current free heap size for diagnostics.
 */
void watchdog_timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;

    uint32_t current_count = s_sample_counter;

    if (current_count == s_last_watchdog_count)
    {
        ESP_LOGE(TAG, "Watchdog: Sensor pipeline stalled! (count=%lu)",
                 (unsigned long)current_count);
        xEventGroupSetBits(g_alert_event_group, ALERT_BIT_WATCHDOG);
    }
    else
    {
        ESP_LOGD(TAG, "Watchdog: OK (samples=%lu, heap=%lu bytes)",
                 (unsigned long)current_count,
                 (unsigned long)esp_get_free_heap_size());
    }

    s_last_watchdog_count = current_count;
}

/* -------------------------------------------------------------------------- */
/*                    Public Initialisation Functions                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Create all FreeRTOS primitives (queues, semaphores, etc.).
 *
 * @details Allocates and initialises:
 *          - g_raw_data_queue:         Queue of raw_sensor_data_t packets.
 *          - g_processed_data_queue:   Queue of processed_sensor_data_t packets.
 *          - g_i2c_mutex:              Mutex for I2C bus arbitration.
 *          - g_data_mutex:             Mutex for g_latest_processed access.
 *          - g_sync_semaphore:         Binary semaphore for init sync.
 *          - g_alert_event_group:      Event group for alert signalling.
 *          - g_watchdog_timer:         Auto-reload software timer.
 *
 * @return  ESP_OK on success, ESP_ERR_NO_MEM if any allocation fails.
 */
esp_err_t app_tasks_create_primitives(void)
{
    ESP_LOGI(TAG, "Creating FreeRTOS primitives...");

    /* Queues. */
    g_raw_data_queue = xQueueCreate(RAW_DATA_QUEUE_LENGTH,
                                    sizeof(raw_sensor_data_t));
    if (g_raw_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create raw data queue");
        return ESP_ERR_NO_MEM;
    }

    g_processed_data_queue = xQueueCreate(PROCESSED_DATA_QUEUE_LENGTH,
                                          sizeof(processed_sensor_data_t));
    if (g_processed_data_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create processed data queue");
        return ESP_ERR_NO_MEM;
    }

    /* Mutexes. */
    g_i2c_mutex = xSemaphoreCreateMutex();
    if (g_i2c_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return ESP_ERR_NO_MEM;
    }

    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_ERR_NO_MEM;
    }

    /* Binary semaphore (created empty - must be given to unblock). */
    g_sync_semaphore = xSemaphoreCreateBinary();
    if (g_sync_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create sync semaphore");
        return ESP_ERR_NO_MEM;
    }

    /* Event group. */
    g_alert_event_group = xEventGroupCreate();
    if (g_alert_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create alert event group");
        return ESP_ERR_NO_MEM;
    }

    /* Software timer (auto-reload). */
    g_watchdog_timer = xTimerCreate(
        "watchdog",
        pdMS_TO_TICKS(WATCHDOG_TIMER_PERIOD_MS),
        pdTRUE, /* Auto-reload. */
        NULL,
        watchdog_timer_callback);

    if (g_watchdog_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create watchdog timer");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "FreeRTOS primitives created successfully");
    ESP_LOGI(TAG, "  Raw queue:       depth=%d, item=%u bytes",
             RAW_DATA_QUEUE_LENGTH, (unsigned)sizeof(raw_sensor_data_t));
    ESP_LOGI(TAG, "  Processed queue: depth=%d, item=%u bytes",
             PROCESSED_DATA_QUEUE_LENGTH,
             (unsigned)sizeof(processed_sensor_data_t));

    return ESP_OK;
}

/**
 * @brief   Start all application tasks.
 *
 * @details Creates the four application tasks with their configured
 *          priorities and stack sizes.  Also starts the watchdog
 *          software timer.
 *
 *          Tasks are pinned to CPU core 1 (APP_CPU) where available,
 *          leaving core 0 for WiFi/BT protocol stack.
 *
 * @return  ESP_OK on success, ESP_FAIL if any task creation fails.
 */
esp_err_t app_tasks_start(void)
{
    ESP_LOGI(TAG, "Starting application tasks...");
    BaseType_t ret;

    /* Sensor read task - highest priority. */
    ret = xTaskCreatePinnedToCore(
        sensor_read_task,
        "sensor_read",
        TASK_STACK_SENSOR_READ,
        NULL,
        TASK_PRIORITY_SENSOR_READ,
        &s_sensor_task_handle,
        1); /* Pin to APP_CPU (core 1). */

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensor_read_task");
        return ESP_FAIL;
    }

    /* Data process task. */
    ret = xTaskCreatePinnedToCore(
        data_process_task,
        "data_process",
        TASK_STACK_DATA_PROCESS,
        NULL,
        TASK_PRIORITY_DATA_PROCESS,
        &s_process_task_handle,
        1);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create data_process_task");
        return ESP_FAIL;
    }

    /* Alert task. */
    ret = xTaskCreatePinnedToCore(
        alert_task,
        "alert",
        TASK_STACK_ALERT,
        NULL,
        TASK_PRIORITY_ALERT,
        &s_alert_task_handle,
        1);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create alert_task");
        return ESP_FAIL;
    }

    /* Display task - lowest priority. */
    ret = xTaskCreatePinnedToCore(
        display_task,
        "display",
        TASK_STACK_DISPLAY,
        NULL,
        TASK_PRIORITY_DISPLAY,
        &s_display_task_handle,
        1);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create display_task");
        return ESP_FAIL;
    }

    /* Start the watchdog timer. */
    if (xTimerStart(g_watchdog_timer, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGW(TAG, "Failed to start watchdog timer");
    }

    ESP_LOGI(TAG, "All tasks started:");
    ESP_LOGI(TAG, "  %-16s  Pri=%d  Stack=%d words",
             "sensor_read", TASK_PRIORITY_SENSOR_READ,
             TASK_STACK_SENSOR_READ);
    ESP_LOGI(TAG, "  %-16s  Pri=%d  Stack=%d words",
             "data_process", TASK_PRIORITY_DATA_PROCESS,
             TASK_STACK_DATA_PROCESS);
    ESP_LOGI(TAG, "  %-16s  Pri=%d  Stack=%d words",
             "alert", TASK_PRIORITY_ALERT,
             TASK_STACK_ALERT);
    ESP_LOGI(TAG, "  %-16s  Pri=%d  Stack=%d words",
             "display", TASK_PRIORITY_DISPLAY,
             TASK_STACK_DISPLAY);

    return ESP_OK;
}
