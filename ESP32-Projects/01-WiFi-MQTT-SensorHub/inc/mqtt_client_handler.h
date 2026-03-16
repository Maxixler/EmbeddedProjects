/**
 * @file    mqtt_client_handler.h
 * @brief   MQTT client wrapper for ESP32 using ESP-IDF MQTT library.
 *
 * @details Provides a simplified MQTT publish/subscribe interface built on top
 *          of the ESP-IDF esp_mqtt_client component. Supports QoS 0/1/2,
 *          automatic reconnection, and topic-based message routing via callbacks.
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef MQTT_CLIENT_HANDLER_H
#define MQTT_CLIENT_HANDLER_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "mqtt_client.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** Maximum number of topic subscriptions. */
#define MQTT_MAX_SUBSCRIPTIONS 8

/** Maximum topic string length. */
#define MQTT_MAX_TOPIC_LEN 128

/** Default MQTT keep-alive interval in seconds. */
#define MQTT_KEEPALIVE_SEC 60

/** Default MQTT QoS level for publishing. */
#define MQTT_DEFAULT_QOS 1

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   MQTT connection status.
     */
    typedef enum
    {
        MQTT_STATUS_DISCONNECTED = 0,
        MQTT_STATUS_CONNECTING,
        MQTT_STATUS_CONNECTED,
        MQTT_STATUS_ERROR
    } mqtt_handler_status_t;

    /**
     * @brief   MQTT message received callback type.
     *
     * @param[in]   topic       Topic string the message was received on.
     * @param[in]   topic_len   Length of the topic string.
     * @param[in]   data        Pointer to the message payload.
     * @param[in]   data_len    Length of the message payload.
     */
    typedef void (*mqtt_message_cb_t)(const char *topic, int topic_len,
                                      const char *data, int data_len);

    /**
     * @brief   MQTT client configuration structure.
     */
    typedef struct
    {
        const char *broker_uri; /**< MQTT broker URI (e.g., "mqtt://192.168.1.100"). */
        const char *client_id;  /**< Client identifier string. */
        const char *username;   /**< Authentication username (NULL if unused). */
        const char *password;   /**< Authentication password (NULL if unused). */
        uint16_t keepalive_sec; /**< Keep-alive interval in seconds. */
    } mqtt_handler_config_t;

    /**
     * @brief   MQTT topic subscription entry.
     */
    typedef struct
    {
        char topic[MQTT_MAX_TOPIC_LEN]; /**< Subscribed topic string. */
        int qos;                        /**< QoS level for this subscription. */
        mqtt_message_cb_t callback;     /**< Callback for messages on this topic. */
        bool active;                    /**< Whether this slot is in use. */
    } mqtt_subscription_t;

    /* -------------------------------------------------------------------------- */
    /*                          Public Function Prototypes                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize and start the MQTT client.
     *
     * @param[in]   config  Pointer to MQTT configuration structure.
     * @return  ESP_OK on success.
     */
    esp_err_t mqtt_handler_init(const mqtt_handler_config_t *config);

    /**
     * @brief   Stop and deinitialize the MQTT client.
     *
     * @return  ESP_OK on success.
     */
    esp_err_t mqtt_handler_deinit(void);

    /**
     * @brief   Publish a message to a topic.
     *
     * @param[in]   topic   Topic string to publish to.
     * @param[in]   data    Pointer to the payload data.
     * @param[in]   len     Length of the payload data.
     * @param[in]   qos     QoS level (0, 1, or 2).
     * @param[in]   retain  Whether to set the retain flag.
     * @return  Message ID on success (>= 0), -1 on failure.
     */
    int mqtt_handler_publish(const char *topic, const char *data,
                             int len, int qos, bool retain);

    /**
     * @brief   Subscribe to a topic with a message callback.
     *
     * @param[in]   topic       Topic string to subscribe to.
     * @param[in]   qos         QoS level for the subscription.
     * @param[in]   callback    Function to call when a message arrives on this topic.
     * @return  ESP_OK on success, ESP_ERR_NO_MEM if subscription table is full.
     */
    esp_err_t mqtt_handler_subscribe(const char *topic, int qos,
                                     mqtt_message_cb_t callback);

    /**
     * @brief   Unsubscribe from a topic.
     *
     * @param[in]   topic   Topic string to unsubscribe from.
     * @return  ESP_OK on success.
     */
    esp_err_t mqtt_handler_unsubscribe(const char *topic);

    /**
     * @brief   Get the current MQTT connection status.
     *
     * @return  Current status as mqtt_handler_status_t.
     */
    mqtt_handler_status_t mqtt_handler_get_status(void);

    /**
     * @brief   Block until MQTT is connected or timeout expires.
     *
     * @param[in]   timeout_ms  Maximum wait time in milliseconds.
     * @return  ESP_OK if connected, ESP_ERR_TIMEOUT on timeout.
     */
    esp_err_t mqtt_handler_wait_connected(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_CLIENT_HANDLER_H */
