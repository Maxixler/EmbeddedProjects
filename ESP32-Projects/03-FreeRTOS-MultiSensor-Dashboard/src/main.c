/**
 * @file    main.c
 * @brief   FreeRTOS Multi-Sensor Dashboard - Main application entry point.
 *
 * @details This application demonstrates a comprehensive FreeRTOS-based
 *          sensor dashboard running on the ESP32.  It integrates:
 *
 *          - **ADC** (potentiometer on GPIO34)
 *          - **BME280** (temperature, humidity, pressure via I2C)
 *          - **MPU6050** (6-axis accelerometer/gyroscope via I2C)
 *          - **SSD1306** (128x64 OLED display via I2C)
 *          - **LED** (GPIO2 alert indicator)
 *
 *          Four FreeRTOS tasks form a pipelined architecture:
 *
 *          +---------------+  raw_queue  +---------------+ proc_queue +----------+
 *          | sensor_read   | ==========> | data_process  | =========> | display  |
 *          | Task (Pri 5)  |             | Task (Pri 4)  |            | (Pri 2)  |
 *          +------+--------+             +------+--------+            +----------+
 *                 |                             |
 *            I2C mutex                   event_group bits
 *                 |                             |
 *          +------+--------+             +------+--------+
 *          | I2C Bus       |             | alert_task    |
 *          | (BME280,      |             | (Pri 3)       |
 *          |  MPU6050,     |             | LED toggle    |
 *          |  SSD1306)     |             +---------------+
 *          +---------------+
 *
 *          FreeRTOS Primitives Used:
 *          - 2 Queues (raw data pipeline, processed data pipeline)
 *          - 2 Mutexes (I2C bus arbitration, shared data protection)
 *          - 1 Binary Semaphore (init synchronisation)
 *          - 1 Event Group (alert signalling)
 *          - 1 Software Timer (watchdog monitoring)
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "app_tasks.h"
#include "sensor_fusion.h"
#include "display_driver.h"

/* -------------------------------------------------------------------------- */
/*                              Configuration                                 */
/* -------------------------------------------------------------------------- */

/** @name I2C Bus Configuration
 *  @{ */
#define MAIN_I2C_PORT I2C_NUM_0       /**< I2C peripheral number. */
#define MAIN_I2C_SDA_GPIO GPIO_NUM_21 /**< I2C SDA pin. */
#define MAIN_I2C_SCL_GPIO GPIO_NUM_22 /**< I2C SCL pin. */
#define MAIN_I2C_FREQ_HZ 100000       /**< I2C bus frequency. */
/** @} */

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "main";

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static esp_err_t prv_i2c_master_init(void);
static void prv_print_system_info(void);
static void prv_print_task_table(void);

/* -------------------------------------------------------------------------- */
/*                          Application Entry Point                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Main application entry point (called by FreeRTOS after boot).
 *
 * @details Execution sequence:
 *
 *          1. Print system information (chip, heap, IDF version).
 *          2. Initialise the I2C master driver (shared bus for all devices).
 *          3. Create all FreeRTOS primitives (queues, mutexes, semaphores,
 *             event group, software timer).
 *          4. Create and start all application tasks.
 *          5. Release the sync semaphore to unblock sensor_read_task.
 *          6. Print the task table summary.
 *          7. Return (app_main runs in the main task context).
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " FreeRTOS Multi-Sensor Dashboard");
    ESP_LOGI(TAG, " Version 1.0.0");
    ESP_LOGI(TAG, "========================================");

    /*
     * Step 1: Print system information.
     */
    prv_print_system_info();

    /*
     * Step 2: Initialise I2C master bus.
     *
     * A single I2C bus (I2C_NUM_0) is shared among BME280, MPU6050,
     * and SSD1306.  Access is arbitrated via g_i2c_mutex.
     */
    esp_err_t ret = prv_i2c_master_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C master init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted - I2C bus is required for all sensors");
        return;
    }

    /*
     * Step 3: Create FreeRTOS primitives.
     */
    ret = app_tasks_create_primitives();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Primitive creation failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "System halted - insufficient memory");
        return;
    }

    /*
     * Step 4: Start all application tasks.
     */
    ret = app_tasks_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Task creation failed: %s", esp_err_to_name(ret));
        return;
    }

    /*
     * Step 5: Release sync semaphore.
     *
     * The sensor_read_task blocks on g_sync_semaphore until hardware
     * initialisation is complete.  Giving the semaphore here unblocks
     * the task and starts the sensor acquisition pipeline.
     */
    vTaskDelay(pdMS_TO_TICKS(100)); /* Brief delay for tasks to initialise. */
    xSemaphoreGive(g_sync_semaphore);
    ESP_LOGI(TAG, "Sync semaphore released - sensor pipeline started");

    /*
     * Step 6: Print task table.
     */
    prv_print_task_table();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " System initialisation complete");
    ESP_LOGI(TAG, " Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    /*
     * app_main returns here.  The FreeRTOS scheduler continues to run
     * the application tasks.  The main task (which called app_main)
     * is automatically deleted by the ESP-IDF framework.
     */
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise the I2C master driver.
 *
 * @details Configures I2C_NUM_0 as a master with internal pull-ups
 *          enabled on SDA (GPIO21) and SCL (GPIO22).  The bus runs
 *          at 100 kHz (standard mode) which is compatible with all
 *          connected devices (BME280, MPU6050, SSD1306).
 *
 * @return  ESP_OK on success.
 */
static esp_err_t prv_i2c_master_init(void)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MAIN_I2C_SDA_GPIO,
        .scl_io_num = MAIN_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MAIN_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(MAIN_I2C_PORT, &i2c_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(MAIN_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C master initialised (port=%d, SDA=GPIO%d, SCL=GPIO%d, %d Hz)",
             MAIN_I2C_PORT, MAIN_I2C_SDA_GPIO, MAIN_I2C_SCL_GPIO,
             MAIN_I2C_FREQ_HZ);

    return ESP_OK;
}

/**
 * @brief   Print system information at startup.
 *
 * @details Logs:
 *          - ESP-IDF version
 *          - Chip model, revision, and core count
 *          - Available features (WiFi, BT, BLE)
 *          - Free heap memory
 *          - Minimum free heap since boot
 */
static void prv_print_system_info(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "--- System Information ---");
    ESP_LOGI(TAG, "ESP-IDF version : %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Chip model      : %s (rev %d)",
             (chip_info.model == CHIP_ESP32) ? "ESP32" : (chip_info.model == CHIP_ESP32S2) ? "ESP32-S2"
                                                     : (chip_info.model == CHIP_ESP32S3)   ? "ESP32-S3"
                                                     : (chip_info.model == CHIP_ESP32C3)   ? "ESP32-C3"
                                                                                           : "Unknown",
             chip_info.revision);
    ESP_LOGI(TAG, "CPU cores       : %d", chip_info.cores);
    ESP_LOGI(TAG, "Features        : WiFi%s%s",
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(TAG, "Free heap       : %lu bytes",
             (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap   : %lu bytes",
             (unsigned long)esp_get_minimum_free_heap_size());
}

/**
 * @brief   Print the application task table.
 *
 * @details Displays a formatted table of all application tasks with
 *          their priorities, stack sizes, and periods.  Also shows
 *          the FreeRTOS primitives and their configurations.
 */
static void prv_print_task_table(void)
{
    ESP_LOGI(TAG, "--- Task Configuration ---");
    ESP_LOGI(TAG, "+------------------+-----+-------+--------+");
    ESP_LOGI(TAG, "| Task             | Pri | Stack | Period |");
    ESP_LOGI(TAG, "+------------------+-----+-------+--------+");
    ESP_LOGI(TAG, "| sensor_read      |  %d  | %5d | %3d ms |",
             TASK_PRIORITY_SENSOR_READ,
             TASK_STACK_SENSOR_READ,
             SENSOR_READ_PERIOD_MS);
    ESP_LOGI(TAG, "| data_process     |  %d  | %5d | (queue)|",
             TASK_PRIORITY_DATA_PROCESS,
             TASK_STACK_DATA_PROCESS);
    ESP_LOGI(TAG, "| alert            |  %d  | %5d | %3d ms |",
             TASK_PRIORITY_ALERT,
             TASK_STACK_ALERT,
             ALERT_CHECK_PERIOD_MS);
    ESP_LOGI(TAG, "| display          |  %d  | %5d | %3d ms |",
             TASK_PRIORITY_DISPLAY,
             TASK_STACK_DISPLAY,
             DISPLAY_REFRESH_PERIOD_MS);
    ESP_LOGI(TAG, "+------------------+-----+-------+--------+");

    ESP_LOGI(TAG, "--- FreeRTOS Primitives ---");
    ESP_LOGI(TAG, "  Raw data queue     : depth=%d", RAW_DATA_QUEUE_LENGTH);
    ESP_LOGI(TAG, "  Processed queue    : depth=%d", PROCESSED_DATA_QUEUE_LENGTH);
    ESP_LOGI(TAG, "  I2C mutex          : created");
    ESP_LOGI(TAG, "  Data mutex         : created");
    ESP_LOGI(TAG, "  Sync semaphore     : binary");
    ESP_LOGI(TAG, "  Alert event group  : 6 bits");
    ESP_LOGI(TAG, "  Watchdog timer     : %d ms (auto-reload)",
             WATCHDOG_TIMER_PERIOD_MS);
}
