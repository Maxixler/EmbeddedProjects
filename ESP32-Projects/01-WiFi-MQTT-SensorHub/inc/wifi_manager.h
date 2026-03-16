/**
 * @file    wifi_manager.h
 * @brief   ESP32 WiFi Station mode manager with auto-reconnect.
 *
 * @details Provides a high-level API for connecting to a WiFi access point
 *          using ESP-IDF's WiFi driver. Implements automatic reconnection
 *          with exponential back-off on disconnection events.
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** Maximum number of reconnection attempts before giving up. */
#define WIFI_MAX_RETRY 10

/** Initial reconnection delay in milliseconds. */
#define WIFI_RETRY_DELAY_MS 1000

/** Maximum reconnection delay (back-off cap) in milliseconds. */
#define WIFI_MAX_RETRY_DELAY_MS 30000

/** Event group bit: WiFi connected and IP obtained. */
#define WIFI_CONNECTED_BIT BIT0

/** Event group bit: WiFi connection failed after max retries. */
#define WIFI_FAIL_BIT BIT1

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   WiFi manager configuration structure.
     */
    typedef struct
    {
        const char *ssid;           /**< Access point SSID (max 32 chars). */
        const char *password;       /**< Access point password (max 64 chars). */
        wifi_auth_mode_t auth_mode; /**< Authentication mode (default: WPA2). */
    } wifi_manager_config_t;

    /**
     * @brief   WiFi connection status enumeration.
     */
    typedef enum
    {
        WIFI_MGR_DISCONNECTED = 0, /**< Not connected. */
        WIFI_MGR_CONNECTING,       /**< Connection in progress. */
        WIFI_MGR_CONNECTED,        /**< Connected with valid IP. */
        WIFI_MGR_FAILED            /**< Connection failed after retries. */
    } wifi_manager_status_t;

    /**
     * @brief   WiFi event callback type.
     *
     * @param[in]   status  Current WiFi connection status.
     */
    typedef void (*wifi_manager_event_cb_t)(wifi_manager_status_t status);

    /* -------------------------------------------------------------------------- */
    /*                          Public Function Prototypes                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize the WiFi manager and connect to the configured AP.
     *
     * @param[in]   config  Pointer to the WiFi configuration structure.
     * @return  ESP_OK on success, ESP_ERR_xxx on failure.
     */
    esp_err_t wifi_manager_init(const wifi_manager_config_t *config);

    /**
     * @brief   Disconnect from the access point and deinitialize WiFi.
     *
     * @return  ESP_OK on success.
     */
    esp_err_t wifi_manager_deinit(void);

    /**
     * @brief   Get the current WiFi connection status.
     *
     * @return  Current status as wifi_manager_status_t.
     */
    wifi_manager_status_t wifi_manager_get_status(void);

    /**
     * @brief   Block until WiFi is connected or connection fails.
     *
     * @param[in]   timeout_ms  Maximum time to wait in milliseconds.
     * @return  ESP_OK if connected, ESP_ERR_TIMEOUT on timeout.
     */
    esp_err_t wifi_manager_wait_connected(uint32_t timeout_ms);

    /**
     * @brief   Register a callback for WiFi status change events.
     *
     * @param[in]   callback    Function pointer to the event callback.
     */
    void wifi_manager_register_callback(wifi_manager_event_cb_t callback);

    /**
     * @brief   Get the current IP address as a string.
     *
     * @param[out]  ip_buf      Buffer to store the IP address string.
     * @param[in]   buf_len     Length of the buffer.
     * @return  ESP_OK on success, ESP_FAIL if not connected.
     */
    esp_err_t wifi_manager_get_ip_string(char *ip_buf, size_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */
