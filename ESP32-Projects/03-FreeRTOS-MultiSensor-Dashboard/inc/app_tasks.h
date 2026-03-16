/**
 * @file    app_tasks.h
 * @brief   FreeRTOS task definitions, inter-task communication handles,
 *          and shared data structures for the Multi-Sensor Dashboard.
 *
 * @details Declares all FreeRTOS primitives used across the application:
 *          - Task function prototypes and configuration (priority, stack)
 *          - Queue handles for raw and processed sensor data pipelines
 *          - Mutex handle for I2C bus arbitration (BME280 + MPU6050)
 *          - Binary semaphore for synchronisation events
 *          - Event group for alert signalling
 *          - Software timer handle for watchdog-like monitoring
 *
 *          Data flow overview:
 *
 *          +----------+   raw_queue   +-----------+  proc_queue  +---------+
 *          | Sensor   |  ==========>  |  Data     | ===========> | Display |
 *          | Read     |               |  Process  |              | Task    |
 *          +----------+               +-----------+              +---------+
 *               |                          |
 *               |  (I2C mutex)             |  event_group
 *               v                          v
 *          +---------+              +-----------+
 *          | BME280  |              |  Alert    |
 *          | MPU6050 |              |  Task     |
 *          +---------+              +-----------+
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef APP_TASKS_H
#define APP_TASKS_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                         Task Configuration Macros                          */
/* -------------------------------------------------------------------------- */

/** @name Task Priorities (higher number = higher priority)
 *  @{ */
#define TASK_PRIORITY_SENSOR_READ (5)  /**< Highest - time-critical sampling. */
#define TASK_PRIORITY_DATA_PROCESS (4) /**< Fusion must keep up with sensor rate. */
#define TASK_PRIORITY_ALERT (3)        /**< Alert evaluation runs after processing. */
#define TASK_PRIORITY_DISPLAY (2)      /**< Display refresh is lowest priority. */
/** @} */

/** @name Task Stack Sizes (in words, 1 word = 4 bytes on ESP32)
 *  @{ */
#define TASK_STACK_SENSOR_READ (4096)  /**< ADC + I2C transactions. */
#define TASK_STACK_DATA_PROCESS (4096) /**< Floating-point fusion. */
#define TASK_STACK_ALERT (2048)        /**< Threshold checks, LED toggle. */
#define TASK_STACK_DISPLAY (4096)      /**< Frame buffer + I2C OLED writes. */
/** @} */

/** @name Task Periods
 *  @{ */
#define SENSOR_READ_PERIOD_MS (100)     /**< 10 Hz sensor sampling. */
#define DISPLAY_REFRESH_PERIOD_MS (500) /**< 2 Hz screen refresh. */
#define ALERT_CHECK_PERIOD_MS (250)     /**< 4 Hz threshold check. */
#define WATCHDOG_TIMER_PERIOD_MS (5000) /**< 5 s watchdog timer. */
/** @} */

/** @name Queue Lengths
 *  @{ */
#define RAW_DATA_QUEUE_LENGTH (10)      /**< Sensor -> Process pipeline. */
#define PROCESSED_DATA_QUEUE_LENGTH (5) /**< Process -> Display pipeline. */
/** @} */

/* -------------------------------------------------------------------------- */
/*                        Event Group Bit Definitions                         */
/* -------------------------------------------------------------------------- */

/** @name Alert Event Bits
 *  @{ */
#define ALERT_BIT_TEMP_HIGH (1 << 0)     /**< Temperature exceeds upper limit. */
#define ALERT_BIT_TEMP_LOW (1 << 1)      /**< Temperature below lower limit. */
#define ALERT_BIT_HUMIDITY_HIGH (1 << 2) /**< Humidity exceeds upper limit. */
#define ALERT_BIT_PRESSURE_LOW (1 << 3)  /**< Pressure below lower limit. */
#define ALERT_BIT_TILT_EXCEED (1 << 4)   /**< Tilt angle exceeds threshold. */
#define ALERT_BIT_WATCHDOG (1 << 5)      /**< Watchdog timer expired. */
#define ALERT_BIT_ANY (0x3F)             /**< Mask for any alert bit. */
    /** @} */

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Raw sensor reading packet sent from sensor_read_task to
     *          data_process_task via the raw data queue.
     */
    typedef struct
    {
        /* ADC */
        uint16_t adc_raw;  /**< 12-bit ADC reading from potentiometer. */
        float adc_voltage; /**< Converted voltage (0 - 3.3 V). */

        /* BME280 */
        float temperature_c; /**< Temperature in degrees Celsius. */
        float humidity_pct;  /**< Relative humidity in percent. */
        float pressure_hpa;  /**< Barometric pressure in hPa. */

        /* MPU6050 */
        float accel_x; /**< Accelerometer X-axis (g). */
        float accel_y; /**< Accelerometer Y-axis (g). */
        float accel_z; /**< Accelerometer Z-axis (g). */
        float gyro_x;  /**< Gyroscope X-axis (deg/s). */
        float gyro_y;  /**< Gyroscope Y-axis (deg/s). */
        float gyro_z;  /**< Gyroscope Z-axis (deg/s). */

        /* Metadata */
        uint32_t timestamp_ms; /**< Tick count at acquisition (ms). */
        bool bme280_valid;     /**< True if BME280 read succeeded. */
        bool mpu6050_valid;    /**< True if MPU6050 read succeeded. */
    } raw_sensor_data_t;

    /**
     * @brief   Processed (fused) sensor data sent from data_process_task to
     *          display_task and evaluated by alert_task.
     */
    typedef struct
    {
        /* Filtered values */
        float temperature_avg; /**< Moving-average temperature (C). */
        float humidity_avg;    /**< Moving-average humidity (%). */
        float pressure_avg;    /**< Moving-average pressure (hPa). */
        float adc_voltage_avg; /**< Moving-average ADC voltage (V). */

        /* Orientation from complementary filter */
        float roll_deg;  /**< Fused roll angle (degrees). */
        float pitch_deg; /**< Fused pitch angle (degrees). */
        float tilt_deg;  /**< Combined tilt magnitude (degrees). */

        /* Alert flags (set by data_process_task) */
        uint32_t alert_flags; /**< Bitmask using ALERT_BIT_xxx. */

        /* Metadata */
        uint32_t timestamp_ms; /**< Tick count after processing (ms). */
        uint32_t sample_count; /**< Running sample counter. */
    } processed_sensor_data_t;

    /**
     * @brief   Alert threshold configuration.
     */
    typedef struct
    {
        float temp_high_c;       /**< Upper temperature limit (C). */
        float temp_low_c;        /**< Lower temperature limit (C). */
        float humidity_high_pct; /**< Upper humidity limit (%). */
        float pressure_low_hpa;  /**< Lower pressure limit (hPa). */
        float tilt_max_deg;      /**< Maximum allowable tilt (deg). */
        float hysteresis_c;      /**< Temperature hysteresis band (C). */
        float hysteresis_pct;    /**< Humidity hysteresis band (%). */
        float hysteresis_hpa;    /**< Pressure hysteresis band (hPa). */
        float hysteresis_deg;    /**< Tilt hysteresis band (deg). */
    } alert_thresholds_t;

    /* -------------------------------------------------------------------------- */
    /*                             Extern Declarations                            */
    /* -------------------------------------------------------------------------- */

    /** @name Inter-task Communication Handles
     *  @{ */
    extern QueueHandle_t g_raw_data_queue;         /**< Sensor -> Process. */
    extern QueueHandle_t g_processed_data_queue;   /**< Process -> Display. */
    extern SemaphoreHandle_t g_i2c_mutex;          /**< I2C bus mutex. */
    extern SemaphoreHandle_t g_sync_semaphore;     /**< Binary sync semaphore. */
    extern EventGroupHandle_t g_alert_event_group; /**< Alert event group. */
    extern TimerHandle_t g_watchdog_timer;         /**< Software watchdog timer. */
    /** @} */

    /** @name Latest Processed Data (protected by mutex)
     *  @{ */
    extern processed_sensor_data_t g_latest_processed; /**< Snapshot for alert_task. */
    extern SemaphoreHandle_t g_data_mutex;             /**< Protects g_latest_processed. */
    /** @} */

    /* -------------------------------------------------------------------------- */
    /*                          Public Function Prototypes                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Sensor reading task - acquires data from ADC, BME280, and MPU6050.
     *
     * @details Periodically samples all sensors at SENSOR_READ_PERIOD_MS interval.
     *          Uses g_i2c_mutex to arbitrate I2C bus access between BME280 and
     *          MPU6050.  Packages readings into raw_sensor_data_t and sends to
     *          g_raw_data_queue.
     *
     * @param[in]   pvParameters    Unused (NULL).
     */
    void sensor_read_task(void *pvParameters);

    /**
     * @brief   Data processing task - applies sensor fusion algorithms.
     *
     * @details Blocks on g_raw_data_queue.  For each raw packet:
     *          1. Updates moving-average filters for temp, humidity, pressure, ADC.
     *          2. Runs complementary filter on accel + gyro to compute roll/pitch.
     *          3. Evaluates alert thresholds with hysteresis.
     *          4. Sends processed_sensor_data_t to g_processed_data_queue.
     *          5. Copies latest result to g_latest_processed under g_data_mutex.
     *
     * @param[in]   pvParameters    Unused (NULL).
     */
    void data_process_task(void *pvParameters);

    /**
     * @brief   Display task - renders processed data on SSD1306 OLED.
     *
     * @details Blocks on g_processed_data_queue.  Updates the 128x64 OLED with
     *          temperature, humidity, pressure, and tilt angle sections.
     *          Uses g_i2c_mutex for OLED I2C transactions.
     *
     * @param[in]   pvParameters    Unused (NULL).
     */
    void display_task(void *pvParameters);

    /**
     * @brief   Alert monitoring task - evaluates thresholds and drives LED.
     *
     * @details Waits on g_alert_event_group for any ALERT_BIT_xxx.  When an
     *          alert fires, toggles an LED and logs the alert condition.
     *          Runs at ALERT_CHECK_PERIOD_MS when no events are pending.
     *
     * @param[in]   pvParameters    Unused (NULL).
     */
    void alert_task(void *pvParameters);

    /**
     * @brief   Software timer callback for watchdog-like monitoring.
     *
     * @details Called every WATCHDOG_TIMER_PERIOD_MS.  Checks that the sensor
     *          read task is still producing data by inspecting the sample counter.
     *          Sets ALERT_BIT_WATCHDOG if the count has not incremented.
     *
     * @param[in]   xTimer  Timer handle (unused).
     */
    void watchdog_timer_callback(TimerHandle_t xTimer);

    /**
     * @brief   Create all FreeRTOS primitives (queues, semaphores, etc.).
     *
     * @return  ESP_OK on success, ESP_ERR_NO_MEM if allocation fails.
     */
    esp_err_t app_tasks_create_primitives(void);

    /**
     * @brief   Start all application tasks.
     *
     * @return  ESP_OK on success, ESP_FAIL if a task could not be created.
     */
    esp_err_t app_tasks_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TASKS_H */
