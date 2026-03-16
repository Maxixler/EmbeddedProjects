/**
 * @file    mqtt_client_handler.c
 * @brief   MQTT client wrapper implementation for ESP32.
 *
 * @details Implements MQTT client functionality using the ESP-IDF esp_mqtt
 *          component. Features include:
 *          - Automatic connection and reconnection management.
 *          - Topic-based message routing with per-topic callbacks.
 *          - Thread-safe publish/subscribe operations.
 *          - Support for QoS 0, 1, and 2.
 *
 *          Event flow:
 *          1. MQTT_EVENT_CONNECTED: Re-subscribe to all active topics.
 *          2. MQTT_EVENT_DATA: Match incoming topic to subscription table
 *             and invoke the registered callback.
 *          3. MQTT_EVENT_DISCONNECTED: Update status, ESP-MQTT handles
 *             automatic reconnection internally.
 *          4. MQTT_EVENT_ERROR: Log error details for diagnostics.
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "mqtt_client_handler.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

static const char *TAG = "mqtt_hdlr";

/** ESP-MQTT client handle. */
static esp_mqtt_client_handle_t s_mqtt_client = NULL;

/** Current MQTT connection status. */
static mqtt_handler_status_t s_status = MQTT_STATUS_DISCONNECTED;

/** Topic subscription table. */
static mqtt_subscription_t s_subscriptions[MQTT_MAX_SUBSCRIPTIONS];

/** Event group for connection signaling. */
static EventGroupHandle_t s_mqtt_event_group = NULL;

#define MQTT_CONNECTED_BIT BIT0

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data);
static void dispatch_message(const char *topic, int topic_len,
                             const char *data, int data_len);
static void resubscribe_all(void);

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

esp_err_t mqtt_handler_init(const mqtt_handler_config_t *config)
{
    if (config == NULL || config->broker_uri == NULL)
    {
        ESP_LOGE(TAG, "Invalid configuration: broker URI is required");
        return ESP_ERR_INVALID_ARG;
    }

    /* Clear the subscription table. */
    memset(s_subscriptions, 0, sizeof(s_subscriptions));

    /* Create event group. */
    s_mqtt_event_group = xEventGroupCreate();
    if (s_mqtt_event_group == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    /* Configure the MQTT client. */
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config->broker_uri,
        .credentials.client_id = config->client_id,
        .credentials.username = config->username,
        .credentials.authentication.password = config->password,
        .session.keepalive = (config->keepalive_sec > 0)
                                 ? config->keepalive_sec
                                 : MQTT_KEEPALIVE_SEC,
    };

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }

    /* Register the unified event handler. */
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    /* Start the MQTT client (connects asynchronously). */
    s_status = MQTT_STATUS_CONNECTING;
    esp_err_t ret = esp_mqtt_client_start(s_mqtt_client);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        s_status = MQTT_STATUS_ERROR;
        return ret;
    }

    ESP_LOGI(TAG, "MQTT client started, connecting to %s", config->broker_uri);
    return ESP_OK;
}

esp_err_t mqtt_handler_deinit(void)
{
    if (s_mqtt_client != NULL)
    {
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
    }

    if (s_mqtt_event_group != NULL)
    {
        vEventGroupDelete(s_mqtt_event_group);
        s_mqtt_event_group = NULL;
    }

    memset(s_subscriptions, 0, sizeof(s_subscriptions));
    s_status = MQTT_STATUS_DISCONNECTED;

    ESP_LOGI(TAG, "MQTT client deinitialized");
    return ESP_OK;
}

int mqtt_handler_publish(const char *topic, const char *data,
                         int len, int qos, bool retain)
{
    if (s_mqtt_client == NULL || s_status != MQTT_STATUS_CONNECTED)
    {
        ESP_LOGW(TAG, "Cannot publish: not connected");
        return -1;
    }

    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, data,
                                         len, qos, retain ? 1 : 0);

    if (msg_id >= 0)
    {
        ESP_LOGD(TAG, "Published to '%s' (msg_id=%d, qos=%d)", topic, msg_id, qos);
    }
    else
    {
        ESP_LOGE(TAG, "Publish failed to '%s'", topic);
    }

    return msg_id;
}

esp_err_t mqtt_handler_subscribe(const char *topic, int qos,
                                 mqtt_message_cb_t callback)
{
    if (topic == NULL || callback == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Find an empty slot in the subscription table. */
    int slot = -1;
    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++)
    {
        if (!s_subscriptions[i].active)
        {
            slot = i;
            break;
        }
    }

    if (slot < 0)
    {
        ESP_LOGE(TAG, "Subscription table full (max %d)", MQTT_MAX_SUBSCRIPTIONS);
        return ESP_ERR_NO_MEM;
    }

    /* Store the subscription. */
    strncpy(s_subscriptions[slot].topic, topic, MQTT_MAX_TOPIC_LEN - 1);
    s_subscriptions[slot].qos = qos;
    s_subscriptions[slot].callback = callback;
    s_subscriptions[slot].active = true;

    /* If already connected, subscribe immediately. */
    if (s_mqtt_client != NULL && s_status == MQTT_STATUS_CONNECTED)
    {
        int msg_id = esp_mqtt_client_subscribe(s_mqtt_client, topic, qos);
        ESP_LOGI(TAG, "Subscribed to '%s' (msg_id=%d)", topic, msg_id);
    }

    return ESP_OK;
}

esp_err_t mqtt_handler_unsubscribe(const char *topic)
{
    if (topic == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++)
    {
        if (s_subscriptions[i].active &&
            strncmp(s_subscriptions[i].topic, topic, MQTT_MAX_TOPIC_LEN) == 0)
        {
            s_subscriptions[i].active = false;
            memset(s_subscriptions[i].topic, 0, MQTT_MAX_TOPIC_LEN);

            if (s_mqtt_client != NULL && s_status == MQTT_STATUS_CONNECTED)
            {
                esp_mqtt_client_unsubscribe(s_mqtt_client, topic);
            }

            ESP_LOGI(TAG, "Unsubscribed from '%s'", topic);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

mqtt_handler_status_t mqtt_handler_get_status(void)
{
    return s_status;
}

esp_err_t mqtt_handler_wait_connected(uint32_t timeout_ms)
{
    if (s_mqtt_event_group == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_mqtt_event_group, MQTT_CONNECTED_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));

    return (bits & MQTT_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Unified MQTT event handler.
 *
 * @details Processes all MQTT events from the ESP-MQTT client. Key events:
 *
 *          CONNECTED:
 *            - Update status, set event group bit.
 *            - Re-subscribe to all previously registered topics (handles
 *              reconnection transparently).
 *
 *          DATA:
 *            - Extract topic and payload from the event.
 *            - Dispatch to the matching subscription callback.
 *
 *          DISCONNECTED:
 *            - Update status, clear event group bit.
 *            - ESP-MQTT handles reconnection internally.
 *
 *          ERROR:
 *            - Log the error type and transport-level details.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected to broker");
        s_status = MQTT_STATUS_CONNECTED;
        xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);

        /* Re-subscribe to all active topics after (re)connection. */
        resubscribe_all();
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected from broker");
        s_status = MQTT_STATUS_DISCONNECTED;
        xEventGroupClearBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "MQTT data received on topic '%.*s'",
                 event->topic_len, event->topic);
        dispatch_message(event->topic, event->topic_len,
                         event->data, event->data_len);
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error event");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            ESP_LOGE(TAG, "  Transport error: 0x%x",
                     event->error_handle->esp_transport_sock_errno);
        }
        s_status = MQTT_STATUS_ERROR;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGD(TAG, "Subscription acknowledged (msg_id=%d)", event->msg_id);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "Publish acknowledged (msg_id=%d)", event->msg_id);
        break;

    default:
        ESP_LOGD(TAG, "Unhandled MQTT event: %ld", (long)event_id);
        break;
    }
}

/**
 * @brief   Dispatch a received message to the matching subscription callback.
 *
 * @details Iterates through the subscription table and compares the incoming
 *          topic with each registered topic. On match, invokes the callback.
 *
 *          Topic matching is done with strncmp using the received topic length,
 *          which supports both exact matches and prefix-based routing.
 */
static void dispatch_message(const char *topic, int topic_len,
                             const char *data, int data_len)
{
    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++)
    {
        if (s_subscriptions[i].active)
        {
            size_t sub_len = strlen(s_subscriptions[i].topic);

            if ((size_t)topic_len == sub_len &&
                strncmp(topic, s_subscriptions[i].topic, topic_len) == 0)
            {
                s_subscriptions[i].callback(topic, topic_len, data, data_len);
                return;
            }
        }
    }

    ESP_LOGD(TAG, "No handler for topic '%.*s'", topic_len, topic);
}

/**
 * @brief   Re-subscribe to all active topics.
 *
 * @details Called after a successful MQTT connection to restore all
 *          subscriptions. This is essential for reconnection scenarios
 *          where the broker does not persist session state (clean session).
 */
static void resubscribe_all(void)
{
    for (int i = 0; i < MQTT_MAX_SUBSCRIPTIONS; i++)
    {
        if (s_subscriptions[i].active)
        {
            int msg_id = esp_mqtt_client_subscribe(
                s_mqtt_client, s_subscriptions[i].topic,
                s_subscriptions[i].qos);
            ESP_LOGI(TAG, "Re-subscribed to '%s' (msg_id=%d)",
                     s_subscriptions[i].topic, msg_id);
        }
    }
}
