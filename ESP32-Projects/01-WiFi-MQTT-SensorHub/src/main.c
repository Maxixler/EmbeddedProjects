/**
 * @file    main.c
 * @brief   WiFi MQTT Sensor Hub - Main application entry point.
 *
 * @details This application demonstrates a complete IoT sensor hub that:
 *          1. Connects to a WiFi access point (STA mode).
 *          2. Connects to an MQTT broker.
 *          3. Periodically reads DHT22 (temp/humidity) and BH1750 (light) sensors.
 *          4. Publishes sensor data as JSON to MQTT topics.
 *          5. Subscribes to a control topic for remote configuration.
 *
 *          Architecture:
 *          +----------+     WiFi      +--------+     MQTT     +--------+
 *          |  DHT22   |--+           |        |             |  MQTT  |
 *          +----------+  |  +-----+  |  ESP32 |<----------->| Broker |
 *          +----------+  +->| ESP |->|  WiFi  |             +--------+
 *          |  BH1750  |--+  | IDF |  | Station|                 |
 *          +----------+     +-----+  +--------+            +--------+
 *                                                           | Client |
 *                                                           +--------+
 *
 *          MQTT Topics:
 *          - Publish:   sensor/data      (JSON sensor readings)
 *          - Publish:   sensor/status    (device status)
 *          - Subscribe: sensor/control   (remote commands)
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

#include "wifi_manager.h"
#include "mqtt_client_handler.h"
#include "sensor_reader.h"

/* -------------------------------------------------------------------------- */
/*                              Configuration                                 */
/* -------------------------------------------------------------------------- */

/** WiFi credentials - replace with your network settings. */
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

/** MQTT broker settings. */
#define MQTT_BROKER_URI "mqtt://192.168.1.100:1883"
#define MQTT_CLIENT_ID "esp32_sensor_hub_01"
#define MQTT_USERNAME NULL
#define MQTT_PASSWORD NULL

/** MQTT topic definitions. */
#define MQTT_TOPIC_DATA "sensor/data"
#define MQTT_TOPIC_STATUS "sensor/status"
#define MQTT_TOPIC_CONTROL "sensor/control"

/** Sensor reading interval in milliseconds. */
#define SENSOR_PUBLISH_INTERVAL_MS 5000

/** JSON buffer size. */
#define JSON_BUFFER_SIZE 256

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "main";

/** Current sensor reading interval (can be changed via MQTT control). */
static uint32_t s_publish_interval_ms = SENSOR_PUBLISH_INTERVAL_MS;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void sensor_publish_task(void *pvParameters);
static void mqtt_control_callback(const char *topic, int topic_len,
                                  const char *data, int data_len);
static void wifi_status_callback(wifi_manager_status_t status);

/* -------------------------------------------------------------------------- */
/*                          Application Entry Point                           */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "=== WiFi MQTT Sensor Hub ===");
    ESP_LOGI(TAG, "Firmware version: 1.0.0");
    ESP_LOGI(TAG, "Free heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    /*
     * Step 1: Initialize WiFi and connect to the access point.
     */
    wifi_manager_config_t wifi_cfg = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
    };

    wifi_manager_register_callback(wifi_status_callback);

    esp_err_t ret = wifi_manager_init(&wifi_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Wait for WiFi connection (30 second timeout). */
    ret = wifi_manager_wait_connected(30000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi connection timeout");
        return;
    }

    char ip_str[16];
    wifi_manager_get_ip_string(ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "WiFi connected, IP: %s", ip_str);

    /*
     * Step 2: Initialize MQTT client and connect to the broker.
     */
    mqtt_handler_config_t mqtt_cfg = {
        .broker_uri = MQTT_BROKER_URI,
        .client_id = MQTT_CLIENT_ID,
        .username = MQTT_USERNAME,
        .password = MQTT_PASSWORD,
        .keepalive_sec = 60,
    };

    ret = mqtt_handler_init(&mqtt_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MQTT initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Wait for MQTT connection (15 second timeout). */
    ret = mqtt_handler_wait_connected(15000);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MQTT connection timeout");
        return;
    }

    /* Subscribe to the control topic for remote commands. */
    mqtt_handler_subscribe(MQTT_TOPIC_CONTROL, 1, mqtt_control_callback);

    /* Publish device status as online. */
    mqtt_handler_publish(MQTT_TOPIC_STATUS, "{\"status\":\"online\"}", 0, 1, true);

    /*
     * Step 3: Initialize sensors.
     */
    ret = sensor_reader_init(NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Sensor initialization issue: %s", esp_err_to_name(ret));
    }

    /*
     * Step 4: Create the sensor publish task.
     */
    xTaskCreate(sensor_publish_task, "sensor_pub", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System initialized. Publishing every %lu ms.",
             (unsigned long)s_publish_interval_ms);
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   FreeRTOS task that periodically reads sensors and publishes data.
 *
 * @details Runs in a loop:
 *          1. Read all sensor data via sensor_reader_read_all().
 *          2. Convert to JSON format via sensor_reader_to_json().
 *          3. Publish JSON to the MQTT data topic.
 *          4. Wait for the configured interval before repeating.
 *
 *          The publish interval can be changed at runtime via MQTT control.
 */
static void sensor_publish_task(void *pvParameters)
{
    sensor_data_t sensor_data;
    char json_buf[JSON_BUFFER_SIZE];

    ESP_LOGI(TAG, "Sensor publish task started");

    while (1)
    {
        /* Read all sensors. */
        esp_err_t ret = sensor_reader_read_all(&sensor_data);

        if (ret == ESP_OK)
        {
            /* Convert to JSON. */
            int len = sensor_reader_to_json(&sensor_data, json_buf,
                                            sizeof(json_buf));

            if (len > 0)
            {
                /* Publish to MQTT. */
                if (mqtt_handler_get_status() == MQTT_STATUS_CONNECTED)
                {
                    mqtt_handler_publish(MQTT_TOPIC_DATA, json_buf, len,
                                         MQTT_DEFAULT_QOS, false);
                    ESP_LOGI(TAG, "Published: %s", json_buf);
                }
                else
                {
                    ESP_LOGW(TAG, "MQTT not connected, data skipped");
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "All sensor reads failed");
        }

        vTaskDelay(pdMS_TO_TICKS(s_publish_interval_ms));
    }
}

/**
 * @brief   Callback for MQTT control messages.
 *
 * @details Handles commands received on the sensor/control topic:
 *          - {"interval": <ms>}  : Change the publish interval.
 *          - {"restart": true}   : Restart the ESP32.
 *
 *          A simple sscanf-based parser is used to extract integer values
 *          from the JSON payload. A full JSON parser (e.g., cJSON) is
 *          recommended for production use.
 */
static void mqtt_control_callback(const char *topic, int topic_len,
                                  const char *data, int data_len)
{
    ESP_LOGI(TAG, "Control message: %.*s", data_len, data);

    /* Parse interval command. */
    int interval = 0;
    if (sscanf(data, "{\"interval\":%d}", &interval) == 1 && interval >= 1000)
    {
        s_publish_interval_ms = (uint32_t)interval;
        ESP_LOGI(TAG, "Publish interval changed to %lu ms",
                 (unsigned long)s_publish_interval_ms);

        /* Acknowledge the change. */
        char ack[64];
        snprintf(ack, sizeof(ack), "{\"interval_ack\":%lu}",
                 (unsigned long)s_publish_interval_ms);
        mqtt_handler_publish(MQTT_TOPIC_STATUS, ack, 0, 1, false);
    }

    /* Parse restart command. */
    if (strstr(data, "\"restart\":true") != NULL)
    {
        ESP_LOGW(TAG, "Restart command received, restarting...");
        mqtt_handler_publish(MQTT_TOPIC_STATUS, "{\"status\":\"restarting\"}", 0, 1, true);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}

/**
 * @brief   WiFi status change callback.
 */
static void wifi_status_callback(wifi_manager_status_t status)
{
    switch (status)
    {
    case WIFI_MGR_CONNECTED:
        ESP_LOGI(TAG, "WiFi: Connected");
        break;
    case WIFI_MGR_DISCONNECTED:
        ESP_LOGW(TAG, "WiFi: Disconnected");
        break;
    case WIFI_MGR_FAILED:
        ESP_LOGE(TAG, "WiFi: Connection failed");
        break;
    default:
        break;
    }
}
