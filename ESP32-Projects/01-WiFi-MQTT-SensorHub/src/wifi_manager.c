/**
 * @file    wifi_manager.c
 * @brief   ESP32 WiFi Station mode manager implementation.
 *
 * @details Implements WiFi STA mode connection with automatic reconnection
 *          using exponential back-off. Uses ESP-IDF's event loop for handling
 *          WiFi and IP events asynchronously.
 *
 *          Connection flow:
 *          1. Initialize NVS, TCP/IP adapter, and default event loop.
 *          2. Configure WiFi in STA mode with provided SSID/password.
 *          3. Register event handlers for WIFI_EVENT and IP_EVENT.
 *          4. Start WiFi and initiate connection.
 *          5. On disconnect, retry with exponential back-off.
 *          6. On IP obtained, set CONNECTED bit in event group.
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "wifi_manager.h"
#include <string.h>
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "wifi_mgr";

/** Event group for signaling WiFi connection status. */
static EventGroupHandle_t s_wifi_event_group = NULL;

/** Current connection status. */
static wifi_manager_status_t s_status = WIFI_MGR_DISCONNECTED;

/** Reconnection attempt counter. */
static uint32_t s_retry_count = 0;

/** Current reconnection delay (exponential back-off). */
static uint32_t s_retry_delay_ms = WIFI_RETRY_DELAY_MS;

/** User-registered status change callback. */
static wifi_manager_event_cb_t s_event_callback = NULL;

/** Stored IP address. */
static esp_netif_ip_info_t s_ip_info;

/** Network interface handle. */
static esp_netif_t *s_netif = NULL;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data);
static esp_err_t nvs_init(void);

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

esp_err_t wifi_manager_init(const wifi_manager_config_t *config)
{
    if (config == NULL || config->ssid == NULL)
    {
        ESP_LOGE(TAG, "Invalid configuration: SSID is required");
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize NVS (required by WiFi driver). */
    esp_err_t ret = nvs_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Create the event group for connection signaling. */
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_ERR_NO_MEM;
    }

    /* Initialize TCP/IP stack and create default event loop. */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Create default WiFi STA network interface. */
    s_netif = esp_netif_create_default_wifi_sta();

    /* Initialize WiFi with default configuration. */
    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_cfg));

    /* Register event handlers. */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, NULL));

    /* Configure WiFi STA parameters. */
    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, config->ssid,
            sizeof(wifi_cfg.sta.ssid) - 1);

    if (config->password != NULL)
    {
        strncpy((char *)wifi_cfg.sta.password, config->password,
                sizeof(wifi_cfg.sta.password) - 1);
    }

    wifi_cfg.sta.threshold.authmode = (config->password != NULL)
                                          ? WIFI_AUTH_WPA2_PSK
                                          : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));

    /* Reset retry state. */
    s_retry_count = 0;
    s_retry_delay_ms = WIFI_RETRY_DELAY_MS;
    s_status = WIFI_MGR_CONNECTING;

    /* Start WiFi (connection is initiated in the event handler). */
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi manager initialized, connecting to '%s'...", config->ssid);

    return ESP_OK;
}

esp_err_t wifi_manager_deinit(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_netif != NULL)
    {
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }

    if (s_wifi_event_group != NULL)
    {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    s_status = WIFI_MGR_DISCONNECTED;
    ESP_LOGI(TAG, "WiFi manager deinitialized");

    return ESP_OK;
}

wifi_manager_status_t wifi_manager_get_status(void)
{
    return s_status;
}

esp_err_t wifi_manager_wait_connected(uint32_t timeout_ms)
{
    if (s_wifi_event_group == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, /* Don't clear bits on exit. */
        pdFALSE, /* Wait for any bit, not all. */
        pdMS_TO_TICKS(timeout_ms));

    if (bits & WIFI_CONNECTED_BIT)
    {
        return ESP_OK;
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        return ESP_FAIL;
    }

    return ESP_ERR_TIMEOUT;
}

void wifi_manager_register_callback(wifi_manager_event_cb_t callback)
{
    s_event_callback = callback;
}

esp_err_t wifi_manager_get_ip_string(char *ip_buf, size_t buf_len)
{
    if (ip_buf == NULL || buf_len < 16)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_status != WIFI_MGR_CONNECTED)
    {
        return ESP_FAIL;
    }

    snprintf(ip_buf, buf_len, IPSTR, IP2STR(&s_ip_info.ip));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   WiFi event handler.
 *
 * @details Handles STA_START (initiate connection) and DISCONNECTED events.
 *          On disconnection, implements exponential back-off retry logic:
 *            delay = min(initial_delay * 2^retry_count, max_delay)
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "WiFi STA started, connecting...");
        esp_wifi_connect();
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        wifi_event_sta_disconnected_t *event =
            (wifi_event_sta_disconnected_t *)event_data;

        ESP_LOGW(TAG, "Disconnected from AP (reason: %d)", event->reason);

        s_status = WIFI_MGR_DISCONNECTED;

        if (s_retry_count < WIFI_MAX_RETRY)
        {
            ESP_LOGI(TAG, "Retrying connection (%lu/%d) in %lu ms...",
                     (unsigned long)(s_retry_count + 1), WIFI_MAX_RETRY,
                     (unsigned long)s_retry_delay_ms);

            vTaskDelay(pdMS_TO_TICKS(s_retry_delay_ms));

            /* Exponential back-off: double the delay each attempt. */
            s_retry_delay_ms *= 2;
            if (s_retry_delay_ms > WIFI_MAX_RETRY_DELAY_MS)
            {
                s_retry_delay_ms = WIFI_MAX_RETRY_DELAY_MS;
            }

            s_retry_count++;
            s_status = WIFI_MGR_CONNECTING;
            esp_wifi_connect();
        }
        else
        {
            ESP_LOGE(TAG, "Max retries reached, connection failed");
            s_status = WIFI_MGR_FAILED;
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);

            if (s_event_callback != NULL)
            {
                s_event_callback(WIFI_MGR_FAILED);
            }
        }
    }
}

/**
 * @brief   IP event handler.
 *
 * @details Called when the STA obtains an IP address from the AP's DHCP server.
 *          Resets retry counters and sets the CONNECTED bit.
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

    ESP_LOGI(TAG, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));

    /* Store IP info for later queries. */
    memcpy(&s_ip_info, &event->ip_info, sizeof(esp_netif_ip_info_t));

    /* Reset retry state on successful connection. */
    s_retry_count = 0;
    s_retry_delay_ms = WIFI_RETRY_DELAY_MS;

    s_status = WIFI_MGR_CONNECTED;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    if (s_event_callback != NULL)
    {
        s_event_callback(WIFI_MGR_CONNECTED);
    }
}

/**
 * @brief   Initialize Non-Volatile Storage (NVS).
 *
 * @details NVS is required by the WiFi driver to store calibration data.
 *          If the NVS partition is full or has a new version, it is erased
 *          and re-initialized.
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

    return ret;
}
