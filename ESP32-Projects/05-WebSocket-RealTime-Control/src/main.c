/**
 * @file main.c
 * @brief Main application for ESP32 WebSocket Real-Time Motor Control.
 *
 * This is the entry point for the ESP32 motor/servo control system.
 * It initializes:
 *  1. Non-volatile storage (NVS)
 *  2. WiFi in STA mode with AP fallback
 *  3. mDNS for local network discovery
 *  4. Motor controller (LEDC PWM + PCNT encoder)
 *  5. HTTP web server with WebSocket support
 *  6. FreeRTOS tasks for periodic status broadcasting
 *
 * The system provides real-time bidirectional control of a DC motor
 * and servo via a web-based interface using WebSocket communication.
 *
 * @author EmbeddedProjects
 * @date 2026
 * @version 1.0.0
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "mdns.h"

#include "web_server.h"
#include "motor_controller.h"

/*******************************************************************************
 * Preprocessor Definitions
 ******************************************************************************/

/** @brief Logging tag for this module. */
static const char *TAG = "main";

/** @brief WiFi SSID for STA mode (station). */
#define WIFI_STA_SSID CONFIG_WIFI_STA_SSID

/** @brief WiFi password for STA mode. */
#define WIFI_STA_PASS CONFIG_WIFI_STA_PASS

/** @brief WiFi SSID for AP mode (fallback access point). */
#define WIFI_AP_SSID "ESP32-Control"

/** @brief WiFi password for AP mode. */
#define WIFI_AP_PASS "esp32control"

/** @brief Maximum STA connection retry count before switching to AP mode. */
#define WIFI_STA_MAX_RETRY 5

/** @brief WiFi AP maximum number of simultaneous connections. */
#define WIFI_AP_MAX_CONN 4

/** @brief WiFi AP channel. */
#define WIFI_AP_CHANNEL 6

/** @brief mDNS hostname (accessible as http://esp32-control.local). */
#define MDNS_HOSTNAME "esp32-control"

/** @brief mDNS instance name for service discovery. */
#define MDNS_INSTANCE_NAME "ESP32 Motor Control Panel"

/** @brief Stack size for the status broadcast task (bytes). */
#define STATUS_TASK_STACK_SIZE 4096

/** @brief Priority of the status broadcast task. */
#define STATUS_TASK_PRIORITY 5

/** @brief Interval between status broadcasts in milliseconds. */
#define STATUS_BROADCAST_INTERVAL_MS 200

/** @brief Interval between encoder readings in milliseconds. */
#define ENCODER_READ_INTERVAL_MS 50

/*******************************************************************************
 * FreeRTOS Event Group Bits
 ******************************************************************************/

/** @brief Event bit: WiFi connected to AP (STA mode). */
#define WIFI_CONNECTED_BIT BIT0

/** @brief Event bit: WiFi connection failed. */
#define WIFI_FAIL_BIT BIT1

/** @brief Event bit: AP mode started. */
#define WIFI_AP_STARTED_BIT BIT2

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

/** @brief FreeRTOS event group for WiFi events. */
static EventGroupHandle_t s_wifi_event_group = NULL;

/** @brief WiFi STA connection retry counter. */
static int s_retry_count = 0;

/** @brief Flag indicating if we are in AP fallback mode. */
static bool s_ap_mode_active = false;

/** @brief Handle for the status broadcast task. */
static TaskHandle_t s_status_task_handle = NULL;

/** @brief Flag to control the status broadcast task lifecycle. */
static volatile bool s_status_task_running = false;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static esp_err_t wifi_init_sta(void);
static esp_err_t wifi_init_ap(void);
static esp_err_t mdns_init_service(void);
static void status_broadcast_task(void *arg);

/*******************************************************************************
 * WiFi Event Handler
 ******************************************************************************/

/**
 * @brief WiFi and IP event handler.
 *
 * Handles WiFi station connect/disconnect events, IP acquisition,
 * and AP mode events.
 *
 * @param[in] arg           Unused handler argument.
 * @param[in] event_base    Event base (WIFI_EVENT or IP_EVENT).
 * @param[in] event_id      Specific event ID.
 * @param[in] event_data    Event-specific data.
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started, connecting...");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            if (s_retry_count < WIFI_STA_MAX_RETRY)
            {
                s_retry_count++;
                ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d",
                         s_retry_count, WIFI_STA_MAX_RETRY);
                esp_wifi_connect();
            }
            else
            {
                ESP_LOGE(TAG, "WiFi connection failed after %d retries, "
                              "switching to AP mode",
                         WIFI_STA_MAX_RETRY);
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;

        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "WiFi AP started");
            xEventGroupSetBits(s_wifi_event_group, WIFI_AP_STARTED_BIT);
            break;

        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event =
                (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG, "Station connected to AP: " MACSTR ", AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t *event =
                (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG, "Station disconnected from AP: " MACSTR ", AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }

        default:
            break;
        }
    }
    else if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR,
                     IP2STR(&event->ip_info.ip));
            s_retry_count = 0;
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}

/*******************************************************************************
 * WiFi Initialization
 ******************************************************************************/

/**
 * @brief Initialize WiFi in Station (STA) mode.
 *
 * Connects to the configured WiFi access point. If the connection
 * fails after WIFI_STA_MAX_RETRY attempts, returns an error so the
 * caller can fall back to AP mode.
 *
 * @return
 *  - ESP_OK if connected successfully.
 *  - ESP_FAIL if connection failed after retries.
 */
static esp_err_t wifi_init_sta(void)
{
    ESP_LOGI(TAG, "Initializing WiFi STA mode...");
    ESP_LOGI(TAG, "SSID: %s", WIFI_STA_SSID);

    /* Create default STA network interface. */
    esp_netif_create_default_wifi_sta();

    /* Initialize WiFi with default configuration. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers. */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
        NULL, &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler,
        NULL, &instance_got_ip));

    /* Configure STA. */
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    strlcpy((char *)wifi_config.sta.ssid, WIFI_STA_SSID,
            sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_STA_PASS,
            sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Wait for connection or failure. */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WiFi STA connected successfully");
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGW(TAG, "WiFi STA connection failed");
        /* Stop WiFi to reconfigure for AP mode. */
        esp_wifi_stop();
        return ESP_FAIL;
    }

    return ESP_FAIL;
}

/**
 * @brief Initialize WiFi in Access Point (AP) mode (fallback).
 *
 * Creates a WiFi access point that clients can connect to directly.
 * This is used when the STA connection to the configured network fails.
 *
 * @return ESP_OK on success.
 */
static esp_err_t wifi_init_ap(void)
{
    ESP_LOGI(TAG, "Initializing WiFi AP mode (fallback)...");
    ESP_LOGI(TAG, "AP SSID: %s, Password: %s", WIFI_AP_SSID, WIFI_AP_PASS);

    /* Create default AP network interface. */
    esp_netif_create_default_wifi_ap();

    /* Re-initialize WiFi if needed. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg); /* May already be initialized. */

    /* Register event handlers if not already registered. */
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
        NULL, &instance_any_id);

    /* Configure AP. */
    wifi_config_t wifi_config = {
        .ap = {
            .channel = WIFI_AP_CHANNEL,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    strlcpy((char *)wifi_config.ap.ssid, WIFI_AP_SSID,
            sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    strlcpy((char *)wifi_config.ap.password, WIFI_AP_PASS,
            sizeof(wifi_config.ap.password));

    /* If password is empty, use open authentication. */
    if (strlen(WIFI_AP_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    s_ap_mode_active = true;

    ESP_LOGI(TAG, "WiFi AP started. Connect to '%s' and navigate to "
                  "http://192.168.4.1",
             WIFI_AP_SSID);

    return ESP_OK;
}

/*******************************************************************************
 * mDNS Initialization
 ******************************************************************************/

/**
 * @brief Initialize mDNS service for local network discovery.
 *
 * Registers the ESP32 with the hostname "esp32-control" so it can
 * be accessed at http://esp32-control.local from any device that
 * supports mDNS (most modern operating systems).
 *
 * @return ESP_OK on success.
 */
static esp_err_t mdns_init_service(void)
{
    esp_err_t ret = mdns_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_instance_name_set(MDNS_INSTANCE_NAME);

    /* Add HTTP service to mDNS. */
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);

    ESP_LOGI(TAG, "mDNS initialized: http://%s.local", MDNS_HOSTNAME);

    return ESP_OK;
}

/*******************************************************************************
 * Status Broadcast Task
 ******************************************************************************/

/**
 * @brief FreeRTOS task for periodic status broadcasting.
 *
 * This task periodically:
 *  1. Reads the encoder count from the PCNT peripheral.
 *  2. Calculates RPM from encoder delta.
 *  3. Retrieves the current motor/servo state.
 *  4. Broadcasts a JSON status update to all connected WebSocket clients.
 *
 * The task runs at STATUS_BROADCAST_INTERVAL_MS intervals and includes
 * separate encoder reading at ENCODER_READ_INTERVAL_MS intervals.
 *
 * @param[in] arg    Unused task argument.
 */
static void status_broadcast_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Status broadcast task started");

    TickType_t last_broadcast = xTaskGetTickCount();
    TickType_t last_encoder_read = xTaskGetTickCount();

    while (s_status_task_running)
    {
        TickType_t now = xTaskGetTickCount();

        /* Read encoder at higher frequency. */
        if ((now - last_encoder_read) >= pdMS_TO_TICKS(ENCODER_READ_INTERVAL_MS))
        {
            motor_controller_read_encoder(NULL);
            last_encoder_read = now;
        }

        /* Broadcast status at configured interval. */
        if ((now - last_broadcast) >= pdMS_TO_TICKS(STATUS_BROADCAST_INTERVAL_MS))
        {
            /* Calculate RPM. */
            motor_controller_calc_rpm(NULL);

            /* Get current state. */
            motor_state_t state;
            esp_err_t ret = motor_controller_get_state(&state);
            if (ret == ESP_OK)
            {
                /* Only broadcast if there are connected clients. */
                if (web_server_get_client_count() > 0)
                {
                    web_server_broadcast_status(
                        state.current_speed,
                        motor_direction_to_str(state.direction),
                        state.servo_angle,
                        state.encoder_count,
                        state.rpm);
                }
            }

            last_broadcast = now;
        }

        /* Yield to other tasks. */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "Status broadcast task stopped");
    vTaskDelete(NULL);
}

/*******************************************************************************
 * Application Entry Point
 ******************************************************************************/

/**
 * @brief Main application entry point.
 *
 * Initializes all subsystems in the following order:
 *  1. NVS flash (required by WiFi)
 *  2. TCP/IP stack and event loop
 *  3. WiFi (STA with AP fallback)
 *  4. mDNS service
 *  5. Motor controller
 *  6. Web server
 *  7. Status broadcast task
 */
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " ESP32 WebSocket Motor Control System");
    ESP_LOGI(TAG, " Version 1.0.0");
    ESP_LOGI(TAG, "========================================");

    /*
     * Step 1: Initialize NVS (Non-Volatile Storage).
     * Required by WiFi driver for storing calibration data.
     */
    ESP_LOGI(TAG, "[1/7] Initializing NVS...");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition error, erasing and re-initializing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /*
     * Step 2: Initialize TCP/IP stack and default event loop.
     */
    ESP_LOGI(TAG, "[2/7] Initializing network stack...");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Network stack initialized");

    /*
     * Step 3: Initialize WiFi (STA mode with AP fallback).
     */
    ESP_LOGI(TAG, "[3/7] Initializing WiFi...");
    s_wifi_event_group = xEventGroupCreate();

    ret = wifi_init_sta();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "STA mode failed, falling back to AP mode");
        ret = wifi_init_ap();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "WiFi AP mode also failed! System cannot operate.");
            return;
        }
    }

    /*
     * Step 4: Initialize mDNS service.
     */
    ESP_LOGI(TAG, "[4/7] Initializing mDNS...");
    ret = mdns_init_service();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "mDNS init failed (non-fatal): %s", esp_err_to_name(ret));
    }

    /*
     * Step 5: Initialize motor controller.
     */
    ESP_LOGI(TAG, "[5/7] Initializing motor controller...");
    motor_config_t motor_cfg = MOTOR_CONFIG_DEFAULT();
    ret = motor_controller_init(&motor_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor controller init failed: %s", esp_err_to_name(ret));
        ESP_LOGW(TAG, "Continuing without motor control...");
    }
    else
    {
        ESP_LOGI(TAG, "Motor controller initialized");
    }

    /*
     * Step 6: Start web server.
     */
    ESP_LOGI(TAG, "[6/7] Starting web server...");
    ret = web_server_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Web server start failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Web server started");

    /*
     * Step 7: Create status broadcast task.
     */
    ESP_LOGI(TAG, "[7/7] Starting status broadcast task...");
    s_status_task_running = true;
    BaseType_t task_ret = xTaskCreate(
        status_broadcast_task,
        "status_bcast",
        STATUS_TASK_STACK_SIZE,
        NULL,
        STATUS_TASK_PRIORITY,
        &s_status_task_handle);

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create status broadcast task");
        s_status_task_running = false;
    }
    else
    {
        ESP_LOGI(TAG, "Status broadcast task started");
    }

    /*
     * Print access information.
     */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " System Ready!");
    if (s_ap_mode_active)
    {
        ESP_LOGI(TAG, " Mode: Access Point (AP)");
        ESP_LOGI(TAG, " SSID: %s", WIFI_AP_SSID);
        ESP_LOGI(TAG, " URL:  http://192.168.4.1");
    }
    else
    {
        ESP_LOGI(TAG, " Mode: Station (STA)");
        ESP_LOGI(TAG, " URL:  http://%s.local", MDNS_HOSTNAME);
    }
    ESP_LOGI(TAG, " WebSocket: ws://<ip>/ws");
    ESP_LOGI(TAG, " API Status: GET /api/status");
    ESP_LOGI(TAG, " API Motor:  POST /api/motor");
    ESP_LOGI(TAG, "========================================");

    /* Main task is no longer needed; FreeRTOS tasks handle everything. */
}
