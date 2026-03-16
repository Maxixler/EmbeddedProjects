/**
 * @file    main.c
 * @brief   BLE OTA Update - Main application entry point.
 *
 * @details This application demonstrates a BLE GATT server on ESP32 that
 *          supports Over-The-Air (OTA) firmware updates over Bluetooth Low
 *          Energy. Additionally, it simulates sensor data and sends periodic
 *          BLE notifications to the connected client.
 *
 *          System architecture:
 *
 *          +---------------------------------------------+
 *          |                ESP32 Device                  |
 *          |                                             |
 *          |  +----------------+   +------------------+  |
 *          |  | BLE GATT       |   | OTA Manager      |  |
 *          |  | Server         |   |                  |  |
 *          |  |                |   | State Machine:   |  |
 *          |  | Sensor Service |   | IDLE->RECEIVING  |  |
 *          |  |   - Data [R|N] |   | ->VALIDATING     |  |
 *          |  |                |   | ->COMPLETE       |  |
 *          |  | OTA Service    +-->|                  |  |
 *          |  |   - Data [W]   |   | SHA-256 verify   |  |
 *          |  |   - Ctrl [W|N] |   | Partition mgmt   |  |
 *          |  +-------+--------+   +------------------+  |
 *          |          |                                   |
 *          |  +-------+--------+                          |
 *          |  | Sensor Task    |  (FreeRTOS)              |
 *          |  | Simulated data |                          |
 *          |  | BLE notify     |                          |
 *          |  +----------------+                          |
 *          +---------------------------------------------+
 *                     |
 *                  BLE Link
 *                     |
 *          +---------------------------------------------+
 *          |         Mobile Device (nRF Connect)         |
 *          |  - Read sensor data                         |
 *          |  - Send firmware chunks via OTA Data char.  |
 *          |  - Control OTA via OTA Control char.        |
 *          +---------------------------------------------+
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "ble_gatt_server.h"
#include "ota_manager.h"

/* -------------------------------------------------------------------------- */
/*                              Configuration                                 */
/* -------------------------------------------------------------------------- */

/** Sensor data notification interval in milliseconds. */
#define SENSOR_NOTIFY_INTERVAL_MS 2000

/** Sensor simulation task stack size in bytes. */
#define SENSOR_TASK_STACK_SIZE 4096

/** Sensor simulation task priority. */
#define SENSOR_TASK_PRIORITY 5

/** Firmware version string (embedded in the binary by ESP-IDF). */
#define FIRMWARE_VERSION "1.0.0"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

/** Logging tag for ESP_LOGx macros. */
static const char *TAG = "main";

/** Handle for the sensor simulation task. */
static TaskHandle_t s_sensor_task_handle = NULL;

/** Simulated sensor tick counter (used to generate periodic data). */
static uint32_t s_sensor_tick = 0;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void sensor_simulation_task(void *pvParameters);
static int ota_data_callback(const uint8_t *data, uint16_t len);
static int ota_ctrl_callback(ble_ota_cmd_t cmd, const uint8_t *data, uint16_t len);
static void ota_state_change_callback(const ota_progress_t *progress);
static void ble_conn_state_callback(ble_conn_state_t state, uint16_t conn_handle);
static esp_err_t nvs_init(void);
static void log_boot_info(void);

/* -------------------------------------------------------------------------- */
/*                          Application Entry Point                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Application main entry point.
 *
 * @details Initializes all subsystems and creates the sensor simulation task.
 *          The initialization sequence is:
 *          1. Initialize NVS (required by BLE stack).
 *          2. Log firmware version and partition information.
 *          3. Initialize OTA manager.
 *          4. Confirm the running image (anti-rollback protection).
 *          5. Initialize BLE GATT server.
 *          6. Register BLE and OTA callbacks.
 *          7. Create the sensor simulation FreeRTOS task.
 */
void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  BLE GATT Server + OTA Firmware Update ");
    ESP_LOGI(TAG, "  Firmware version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "========================================");

    /*
     * Step 1: Initialize Non-Volatile Storage (NVS).
     *
     * NVS is required by the NimBLE host stack to store bonding
     * information and other persistent BLE configuration.
     */
    ret = nvs_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    /*
     * Step 2: Log boot information.
     *
     * Display the running firmware version, partition details, and
     * system information for debugging and verification.
     */
    log_boot_info();

    /*
     * Step 3: Initialize the OTA manager.
     *
     * The OTA manager queries the partition table and prepares for
     * firmware updates. It must be initialized before the BLE server
     * so that OTA callbacks are ready when the first client connects.
     */
    ret = ota_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA manager initialization failed: %s",
                 esp_err_to_name(ret));
        ESP_LOGW(TAG, "OTA functionality will be unavailable");
    }
    else
    {
        /* Register the OTA state change callback for logging/notifications. */
        ota_manager_register_state_callback(ota_state_change_callback);

        /*
         * Step 3a: Confirm the current firmware image.
         *
         * After an OTA update, the new firmware boots with state
         * ESP_OTA_IMG_NEW. We confirm it here to prevent automatic
         * rollback on the next reboot. In a production system, you
         * should perform additional self-tests before confirming.
         */
        ret = ota_manager_confirm_image();
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Image confirmation failed (may not be OTA boot)");
        }
    }

    /*
     * Step 4: Initialize the BLE GATT server.
     *
     * This starts the NimBLE host stack, registers GATT services,
     * and begins advertising the device to nearby BLE scanners.
     */
    ret = ble_gatt_server_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "BLE GATT server initialization failed: %s",
                 esp_err_to_name(ret));
        return;
    }

    /*
     * Step 5: Register application callbacks.
     *
     * The BLE GATT server invokes these callbacks when:
     *   - ota_data_cb:    Firmware data chunk received on OTA Data char.
     *   - ota_ctrl_cb:    OTA command received on OTA Control char.
     *   - conn_state_cb:  BLE connection state changes (connect/disconnect).
     */
    ble_gatt_callbacks_t ble_cbs = {
        .ota_data_cb = ota_data_callback,
        .ota_ctrl_cb = ota_ctrl_callback,
        .conn_state_cb = ble_conn_state_callback,
    };

    ret = ble_gatt_server_register_callbacks(&ble_cbs);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register BLE callbacks");
    }

    /*
     * Step 6: Create the sensor simulation task.
     *
     * This FreeRTOS task generates simulated sensor readings (temperature,
     * humidity) and sends BLE notifications to the connected client at
     * regular intervals.
     */
    BaseType_t task_ret = xTaskCreate(
        sensor_simulation_task,
        "sensor_sim",
        SENSOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_TASK_PRIORITY,
        &s_sensor_task_handle);

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create sensor simulation task");
    }

    ESP_LOGI(TAG, "System initialization complete");
    ESP_LOGI(TAG, "Waiting for BLE connection from client...");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   FreeRTOS task that simulates sensor data and sends BLE notifications.
 *
 * @details Generates sinusoidal temperature and humidity values that vary
 *          over time, packages them into a compact binary format, and sends
 *          the data as a BLE notification on the Sensor Data characteristic.
 *
 *          Sensor data packet format (12 bytes):
 *          +--------+--------+--------+--------+--------+--------+
 *          | Temp   | Temp   | Humid  | Humid  | Tick   | Tick   |
 *          | (int)  | (frac) | (int)  | (frac) | (MSB)  | (3B)   |
 *          | 2 B LE | 2 B LE | 2 B LE | 2 B LE | 4 B LE          |
 *          +--------+--------+---------+--------+-----------------+
 *
 *          Data generation uses sinusoidal functions to simulate realistic
 *          sensor behavior with gradual changes over time.
 *
 * @param[in]   pvParameters    FreeRTOS task parameter (unused).
 */
static void sensor_simulation_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor simulation task started (interval: %d ms)",
             SENSOR_NOTIFY_INTERVAL_MS);

    while (1)
    {
        /*
         * Generate simulated sensor data.
         *
         * Temperature: 20.0 + 5.0 * sin(tick / 50.0)  -> Range: 15.0 - 25.0 C
         * Humidity:    50.0 + 15.0 * cos(tick / 30.0)  -> Range: 35.0 - 65.0 %
         */
        float temperature = 20.0f + 5.0f * sinf((float)s_sensor_tick / 50.0f);
        float humidity = 50.0f + 15.0f * cosf((float)s_sensor_tick / 30.0f);

        /*
         * Pack sensor data into a binary payload.
         *
         * We use fixed-point representation with 2 decimal places:
         *   temp_raw = (int16_t)(temperature * 100)
         *   humid_raw = (int16_t)(humidity * 100)
         *
         * Example: 23.45 C -> 2345 (0x0929 in little-endian)
         */
        int16_t temp_raw = (int16_t)(temperature * 100.0f);
        int16_t humid_raw = (int16_t)(humidity * 100.0f);

        uint8_t payload[8];
        /* Temperature: 2 bytes, little-endian */
        payload[0] = (uint8_t)(temp_raw & 0xFF);
        payload[1] = (uint8_t)((temp_raw >> 8) & 0xFF);
        /* Humidity: 2 bytes, little-endian */
        payload[2] = (uint8_t)(humid_raw & 0xFF);
        payload[3] = (uint8_t)((humid_raw >> 8) & 0xFF);
        /* Tick counter: 4 bytes, little-endian */
        payload[4] = (uint8_t)(s_sensor_tick & 0xFF);
        payload[5] = (uint8_t)((s_sensor_tick >> 8) & 0xFF);
        payload[6] = (uint8_t)((s_sensor_tick >> 16) & 0xFF);
        payload[7] = (uint8_t)((s_sensor_tick >> 24) & 0xFF);

        /*
         * Send BLE notification.
         *
         * The notification is only sent if a client is connected and has
         * enabled notifications on the Sensor Data characteristic by
         * writing 0x0001 to the CCCD descriptor.
         */
        esp_err_t ret = ble_gatt_server_notify_sensor_data(payload, sizeof(payload));
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Sensor notify: temp=%.2f C, humid=%.2f %%, tick=%lu",
                     temperature, humidity, (unsigned long)s_sensor_tick);
        }

        s_sensor_tick++;

        vTaskDelay(pdMS_TO_TICKS(SENSOR_NOTIFY_INTERVAL_MS));
    }
}

/**
 * @brief   Callback for OTA firmware data chunks received over BLE.
 *
 * @details This function is called by the BLE GATT server each time a
 *          firmware data chunk is written to the OTA Data characteristic.
 *          The chunk is forwarded to the OTA manager for writing to flash.
 *
 * @param[in]   data    Pointer to the firmware data chunk.
 * @param[in]   len     Length of the data chunk in bytes.
 * @return  0 on success, -1 on error.
 */
static int ota_data_callback(const uint8_t *data, uint16_t len)
{
    esp_err_t ret = ota_manager_write_chunk(data, len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA write chunk failed: %s", esp_err_to_name(ret));

        /* Notify the BLE client of the error via OTA Control notification. */
        ota_progress_t progress;
        ota_manager_get_progress(&progress);

        uint8_t status_buf[3] = {
            (uint8_t)progress.state,
            (uint8_t)progress.error,
            progress.progress_percent};
        ble_gatt_server_notify_ota_status(status_buf, sizeof(status_buf));

        return -1;
    }

    return 0;
}

/**
 * @brief   Callback for OTA control commands received over BLE.
 *
 * @details Handles the following OTA commands from the BLE client:
 *
 *          CMD 0x01 (START):
 *            Payload: 4 bytes (uint32_t total_size, little-endian)
 *            Action:  Start a new OTA session with the specified firmware size.
 *
 *          CMD 0x02 (STOP):
 *            Payload: None
 *            Action:  Abort the current OTA session.
 *
 *          CMD 0x03 (COMMIT):
 *            Payload: Optional 32 bytes (SHA-256 hash for verification)
 *            Action:  Finish receiving, validate, and commit the firmware.
 *
 *          CMD 0x04 (ROLLBACK):
 *            Payload: None
 *            Action:  Rollback to the previous firmware version.
 *
 *          CMD 0x05 (VERSION_REQ):
 *            Payload: None
 *            Action:  Send the current firmware version via notification.
 *
 * @param[in]   cmd     The OTA command received.
 * @param[in]   data    Optional command parameter data.
 * @param[in]   len     Length of the parameter data.
 * @return  0 on success, -1 on error.
 */
static int ota_ctrl_callback(ble_ota_cmd_t cmd, const uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    ota_progress_t progress;
    uint8_t status_buf[3];

    switch (cmd)
    {
    case BLE_OTA_CMD_START:
    {
        /*
         * START command: Begin OTA session.
         *
         * The client sends the total firmware size as a 4-byte
         * little-endian unsigned integer in the command payload.
         */
        if (data == NULL || len < 4)
        {
            ESP_LOGE(TAG, "OTA START: missing firmware size parameter");
            return -1;
        }

        uint32_t total_size = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);

        ESP_LOGI(TAG, "OTA START: firmware size = %lu bytes",
                 (unsigned long)total_size);

        ret = ota_manager_start(total_size);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start OTA session");
            return -1;
        }

        /* Notify client of state change. */
        ota_manager_get_progress(&progress);
        status_buf[0] = (uint8_t)progress.state;
        status_buf[1] = (uint8_t)progress.error;
        status_buf[2] = progress.progress_percent;
        ble_gatt_server_notify_ota_status(status_buf, sizeof(status_buf));
        break;
    }

    case BLE_OTA_CMD_STOP:
    {
        /* STOP command: Abort the current OTA session. */
        ESP_LOGW(TAG, "OTA STOP: aborting session");

        ret = ota_manager_abort();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to abort OTA session");
        }

        /* Notify client. */
        ota_manager_get_progress(&progress);
        status_buf[0] = (uint8_t)progress.state;
        status_buf[1] = (uint8_t)progress.error;
        status_buf[2] = 0;
        ble_gatt_server_notify_ota_status(status_buf, sizeof(status_buf));
        break;
    }

    case BLE_OTA_CMD_COMMIT:
    {
        /*
         * COMMIT command: Finish and commit the firmware.
         *
         * Optional payload: 32 bytes SHA-256 hash for verification.
         * If provided, the OTA manager compares it against the
         * incrementally computed hash of the received firmware.
         */
        const uint8_t *expected_sha256 = NULL;
        if (data != NULL && len >= OTA_SHA256_DIGEST_LEN)
        {
            expected_sha256 = data;
            ESP_LOGI(TAG, "OTA COMMIT: SHA-256 verification requested");
        }
        else
        {
            ESP_LOGW(TAG, "OTA COMMIT: no SHA-256 provided, skipping hash check");
        }

        /* Finalize and validate the firmware image. */
        ret = ota_manager_finish(expected_sha256);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Firmware validation failed");

            ota_manager_get_progress(&progress);
            status_buf[0] = (uint8_t)progress.state;
            status_buf[1] = (uint8_t)progress.error;
            status_buf[2] = progress.progress_percent;
            ble_gatt_server_notify_ota_status(status_buf, sizeof(status_buf));
            return -1;
        }

        /* Notify client that validation passed before committing. */
        ota_manager_get_progress(&progress);
        status_buf[0] = (uint8_t)progress.state;
        status_buf[1] = (uint8_t)progress.error;
        status_buf[2] = 100;
        ble_gatt_server_notify_ota_status(status_buf, sizeof(status_buf));

        /* Commit the firmware (this will restart the device). */
        ESP_LOGI(TAG, "Committing new firmware and restarting...");
        ret = ota_manager_commit();
        /* NOTE: If commit succeeds, this line is never reached. */
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Firmware commit failed");
            return -1;
        }
        break;
    }

    case BLE_OTA_CMD_ROLLBACK:
    {
        /* ROLLBACK command: Revert to previous firmware. */
        ESP_LOGW(TAG, "OTA ROLLBACK: reverting to previous firmware");

        ret = ota_manager_rollback();
        /* NOTE: If rollback succeeds, this line is never reached (reboot). */
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Rollback failed");
            return -1;
        }
        break;
    }

    case BLE_OTA_CMD_VERSION_REQ:
    {
        /*
         * VERSION_REQ command: Report the current firmware version.
         *
         * We send the version string as a notification on the OTA
         * Control characteristic.
         */
        ota_manager_get_progress(&progress);

        ESP_LOGI(TAG, "OTA VERSION_REQ: current version = '%s'",
                 progress.current_version);

        /* Send the version string as notification (up to 32 bytes). */
        uint16_t ver_len = (uint16_t)strlen(progress.current_version);
        if (ver_len > OTA_VERSION_STRING_MAX_LEN)
        {
            ver_len = OTA_VERSION_STRING_MAX_LEN;
        }

        ble_gatt_server_notify_ota_status(
            (const uint8_t *)progress.current_version, ver_len);
        break;
    }

    default:
        ESP_LOGW(TAG, "Unknown OTA command: 0x%02X", cmd);
        return -1;
    }

    return 0;
}

/**
 * @brief   Callback for OTA state machine transitions.
 *
 * @details Called by the OTA manager whenever the state changes. Logs the
 *          transition and sends a BLE notification to inform the client.
 *
 * @param[in]   progress    Pointer to the current OTA progress information.
 */
static void ota_state_change_callback(const ota_progress_t *progress)
{
    static const char *state_names[] = {
        "IDLE", "RECEIVING", "VALIDATING", "COMPLETE", "ERROR"};

    ESP_LOGI(TAG, "OTA state: %s, progress: %d%%, error: %d",
             state_names[progress->state],
             progress->progress_percent,
             progress->error);

    /* Send state update to BLE client via notification. */
    uint8_t status_buf[3] = {
        (uint8_t)progress->state,
        (uint8_t)progress->error,
        progress->progress_percent};

    ble_gatt_server_notify_ota_status(status_buf, sizeof(status_buf));
}

/**
 * @brief   Callback for BLE connection state changes.
 *
 * @details Called when a BLE client connects to or disconnects from
 *          the GATT server. On disconnection during an active OTA
 *          session, the session is automatically aborted.
 *
 * @param[in]   state       New connection state.
 * @param[in]   conn_handle Connection handle.
 */
static void ble_conn_state_callback(ble_conn_state_t state, uint16_t conn_handle)
{
    switch (state)
    {
    case BLE_STATE_CONNECTED:
        ESP_LOGI(TAG, "BLE client connected (handle=%d)", conn_handle);
        ESP_LOGI(TAG, "Free heap: %lu bytes",
                 (unsigned long)esp_get_free_heap_size());
        break;

    case BLE_STATE_IDLE:
        ESP_LOGI(TAG, "BLE client disconnected");

        /* Abort any in-progress OTA session on disconnect. */
        if (ota_manager_get_state() != OTA_STATE_IDLE)
        {
            ESP_LOGW(TAG, "Aborting OTA session due to BLE disconnect");
            ota_manager_abort();
        }
        break;

    case BLE_STATE_ADVERTISING:
        ESP_LOGI(TAG, "BLE advertising started");
        break;

    default:
        break;
    }
}

/**
 * @brief   Initialize Non-Volatile Storage (NVS).
 *
 * @details NVS is required by the NimBLE host stack for storing bonding
 *          data, CCCD state, and other persistent BLE configuration.
 *          If the NVS partition is corrupted or has an incompatible
 *          version, it is erased and re-initialized.
 *
 * @return  ESP_OK on success.
 */
static esp_err_t nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition issue, erasing and re-initializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "NVS initialized successfully");
    }

    return ret;
}

/**
 * @brief   Log boot-time information for debugging.
 *
 * @details Displays the firmware version, running partition details, boot
 *          partition, chip information, and available heap memory. This
 *          information is invaluable for debugging OTA issues and verifying
 *          that the correct firmware is running after an update.
 */
static void log_boot_info(void)
{
    ESP_LOGI(TAG, "--- Boot Information ---");

    /* Firmware version from the app description. */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    if (app_desc != NULL)
    {
        ESP_LOGI(TAG, "App name:    %s", app_desc->project_name);
        ESP_LOGI(TAG, "App version: %s", app_desc->version);
        ESP_LOGI(TAG, "Compile time: %s %s", app_desc->date, app_desc->time);
        ESP_LOGI(TAG, "IDF version: %s", app_desc->idf_ver);
    }

    /* Running partition info. */
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running != NULL)
    {
        ESP_LOGI(TAG, "Running partition: '%s' at 0x%08lx (size: 0x%08lx)",
                 running->label,
                 (unsigned long)running->address,
                 (unsigned long)running->size);
    }

    /* Boot partition info. */
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    if (boot != NULL)
    {
        ESP_LOGI(TAG, "Boot partition:    '%s' at 0x%08lx",
                 boot->label, (unsigned long)boot->address);
    }

    /* System information. */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip: ESP32 rev %d, %d CPU cores, WiFi%s%s",
             chip_info.revision, chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());
    ESP_LOGI(TAG, "Min free heap: %lu bytes",
             (unsigned long)esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "------------------------");
}
