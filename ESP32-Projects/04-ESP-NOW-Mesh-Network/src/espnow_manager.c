/**
 * @file espnow_manager.c
 * @brief ESP-NOW Manager Implementation.
 *
 * Provides ESP-NOW initialization with WiFi in STA mode (no AP connection),
 * peer management (add/remove/list), send and receive callbacks, broadcast
 * and unicast messaging, encryption support (PMK/LMK), and channel
 * configuration.
 *
 * @version 1.0.0
 * @date 2026-03-16
 */

/* -------------------------------------------------------------------------
 * Includes
 * ---------------------------------------------------------------------- */
#include "espnow_manager.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

/* -------------------------------------------------------------------------
 * Private Defines
 * ---------------------------------------------------------------------- */

/** @brief Log tag for this module. */
static const char *TAG = "ESPNOW_MGR";

/** @brief Stack size for the RX processing task. */
#define ESPNOW_RX_TASK_STACK_SIZE (4096)

/** @brief Priority for the RX processing task. */
#define ESPNOW_RX_TASK_PRIORITY (5)

/* -------------------------------------------------------------------------
 * Private Data
 * ---------------------------------------------------------------------- */

/** @brief Whether the manager has been initialized. */
static bool s_initialized = false;

/** @brief Peer information table. */
static espnow_peer_info_t s_peer_table[ESPNOW_MAX_PEERS];

/** @brief Mutex for peer table access. */
static SemaphoreHandle_t s_peer_mutex = NULL;

/** @brief Receive message queue. */
static QueueHandle_t s_rx_queue = NULL;

/** @brief RX processing task handle. */
static TaskHandle_t s_rx_task_handle = NULL;

/** @brief Application-level receive callback. */
static espnow_rx_callback_t s_rx_callback = NULL;

/** @brief Application-level send callback. */
static espnow_tx_callback_t s_tx_callback = NULL;

/** @brief Runtime statistics. */
static espnow_stats_t s_stats = {0};

/** @brief Current WiFi channel. */
static uint8_t s_current_channel = ESPNOW_DEFAULT_CHANNEL;

/** @brief Local MAC address cache. */
static uint8_t s_local_mac[ESPNOW_MAC_ADDR_LEN] = {0};

/* -------------------------------------------------------------------------
 * Private Function Prototypes
 * ---------------------------------------------------------------------- */

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int data_len);
static void espnow_rx_task(void *arg);
static esp_err_t wifi_init_sta(uint8_t channel);
static int find_peer_slot(const uint8_t *mac_addr);
static int find_free_slot(void);

/* -------------------------------------------------------------------------
 * WiFi Initialization (STA mode, no connection)
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialize WiFi in STA mode without connecting to any AP.
 *
 * ESP-NOW operates on top of WiFi and requires the WiFi radio to be
 * active. We configure STA mode but do not call esp_wifi_connect().
 *
 * @param[in] channel WiFi channel to use (1-14).
 * @return ESP_OK on success.
 */
static esp_err_t wifi_init_sta(uint8_t channel)
{
    esp_err_t ret;

    /* Initialize NVS (required by WiFi) */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize the TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Create default event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* WiFi initialization with default configuration */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Store WiFi configuration in RAM only */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    /* Set WiFi mode to STA */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* Start WiFi */
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Set the channel after WiFi has started */
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));

    /* Enable long-range mode for better mesh coverage (optional) */
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
                                          WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                              WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));

    /* Cache the local MAC address */
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, s_local_mac));

    char mac_str[18];
    espnow_mac_to_str(s_local_mac, mac_str);
    ESP_LOGI(TAG, "WiFi STA initialized. MAC: %s, Channel: %d", mac_str, channel);

    return ESP_OK;
}

/* -------------------------------------------------------------------------
 * ESP-NOW Callbacks
 * ---------------------------------------------------------------------- */

/**
 * @brief ESP-NOW send callback (called from WiFi task context).
 *
 * Reports delivery confirmation for each transmitted frame.
 *
 * @param[in] mac_addr Destination MAC address.
 * @param[in] status   Delivery status (success or fail).
 */
static void espnow_send_cb(const uint8_t *mac_addr,
                           esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send callback: NULL MAC address");
        return;
    }

    if (status == ESP_NOW_SEND_SUCCESS)
    {
        s_stats.tx_success++;
    }
    else
    {
        s_stats.tx_fail++;
    }

    /* Notify the application callback if registered */
    if (s_tx_callback != NULL)
    {
        espnow_tx_event_t event;
        memcpy(event.dest_mac, mac_addr, ESPNOW_MAC_ADDR_LEN);
        event.status = (status == ESP_NOW_SEND_SUCCESS) ? ESPNOW_SEND_OK : ESPNOW_SEND_FAIL;
        s_tx_callback(&event);
    }

    char mac_str[18];
    espnow_mac_to_str(mac_addr, mac_str);
    ESP_LOGD(TAG, "TX to %s: %s", mac_str,
             (status == ESP_NOW_SEND_SUCCESS) ? "OK" : "FAIL");
}

/**
 * @brief ESP-NOW receive callback (called from WiFi task context).
 *
 * Copies the received data into a queue for processing by the
 * application task (avoids blocking the WiFi task).
 *
 * @param[in] recv_info  Receive information (source MAC, etc.).
 * @param[in] data       Pointer to received data.
 * @param[in] data_len   Length of received data.
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info,
                           const uint8_t *data, int data_len)
{
    if (recv_info == NULL || data == NULL || data_len <= 0)
    {
        ESP_LOGE(TAG, "Receive callback: invalid parameters");
        return;
    }

    if (data_len > ESPNOW_MAX_PAYLOAD_SIZE)
    {
        ESP_LOGW(TAG, "Received frame too large: %d bytes", data_len);
        return;
    }

    espnow_rx_msg_t msg;
    memcpy(msg.src_mac, recv_info->src_addr, ESPNOW_MAC_ADDR_LEN);
    memcpy(msg.data, data, data_len);
    msg.data_len = (uint16_t)data_len;
    msg.rssi = 0; /* RSSI can be obtained from rx_ctrl if available */
    msg.timestamp = esp_timer_get_time();

    /* Update last-seen time for the peer */
    if (s_peer_mutex != NULL && xSemaphoreTake(s_peer_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        int idx = find_peer_slot(recv_info->src_addr);
        if (idx >= 0)
        {
            s_peer_table[idx].last_seen = msg.timestamp;
        }
        xSemaphoreGive(s_peer_mutex);
    }

    /* Enqueue the message for processing */
    if (s_rx_queue != NULL)
    {
        if (xQueueSend(s_rx_queue, &msg, 0) != pdTRUE)
        {
            s_stats.rx_dropped++;
            ESP_LOGW(TAG, "RX queue full - message dropped");
        }
        else
        {
            s_stats.rx_count++;
        }
    }
}

/* -------------------------------------------------------------------------
 * RX Processing Task
 * ---------------------------------------------------------------------- */

/**
 * @brief Task that dequeues received messages and dispatches them.
 *
 * Runs in application context (not ISR/WiFi context), making it safe
 * to perform blocking operations and call application callbacks.
 *
 * @param[in] arg Unused.
 */
static void espnow_rx_task(void *arg)
{
    espnow_rx_msg_t msg;

    ESP_LOGI(TAG, "RX processing task started");

    while (true)
    {
        if (xQueueReceive(s_rx_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            /* Dispatch to the application callback */
            if (s_rx_callback != NULL)
            {
                s_rx_callback(&msg);
            }
        }
    }

    /* Should never reach here */
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Peer Table Helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Find the index of a peer by MAC address.
 *
 * @param[in] mac_addr 6-byte MAC address.
 * @return Index (>= 0) if found, -1 otherwise.
 */
static int find_peer_slot(const uint8_t *mac_addr)
{
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++)
    {
        if (s_peer_table[i].active &&
            memcmp(s_peer_table[i].mac_addr, mac_addr,
                   ESPNOW_MAC_ADDR_LEN) == 0)
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find the first free slot in the peer table.
 *
 * @return Index (>= 0) if found, -1 if full.
 */
static int find_free_slot(void)
{
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++)
    {
        if (!s_peer_table[i].active)
        {
            return i;
        }
    }
    return -1;
}

/* -------------------------------------------------------------------------
 * Public API Implementation
 * ---------------------------------------------------------------------- */

esp_err_t espnow_manager_init(const espnow_config_t *config)
{
    if (s_initialized)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    if (config == NULL)
    {
        ESP_LOGE(TAG, "Configuration is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing ESP-NOW Manager...");

    /* Clear state */
    memset(s_peer_table, 0, sizeof(s_peer_table));
    memset(&s_stats, 0, sizeof(s_stats));

    s_current_channel = (config->channel > 0 && config->channel <= 14) ? config->channel : ESPNOW_DEFAULT_CHANNEL;

    /* Store callbacks */
    s_rx_callback = config->rx_callback;
    s_tx_callback = config->tx_callback;

    /* Initialize WiFi in STA mode */
    esp_err_t ret = wifi_init_sta(s_current_channel);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "WiFi initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Initialize ESP-NOW */
    ret = esp_now_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_init() failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register ESP-NOW callbacks */
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Configure PMK for encryption */
    if (config->enable_encryption)
    {
        ESP_ERROR_CHECK(esp_now_set_pmk(config->pmk));
        ESP_LOGI(TAG, "Encryption enabled (PMK set)");
    }

    /* Add broadcast peer (required for broadcast transmissions) */
    esp_now_peer_info_t broadcast_peer = {0};
    memset(broadcast_peer.peer_addr, 0xFF, ESPNOW_MAC_ADDR_LEN);
    broadcast_peer.channel = s_current_channel;
    broadcast_peer.ifidx = WIFI_IF_STA;
    broadcast_peer.encrypt = false;
    ret = esp_now_add_peer(&broadcast_peer);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST)
    {
        ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create the mutex */
    s_peer_mutex = xSemaphoreCreateMutex();
    if (s_peer_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create peer mutex");
        return ESP_FAIL;
    }

    /* Create the receive queue */
    s_rx_queue = xQueueCreate(ESPNOW_RX_QUEUE_SIZE, sizeof(espnow_rx_msg_t));
    if (s_rx_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_FAIL;
    }

    /* Create the RX processing task */
    BaseType_t xret = xTaskCreate(espnow_rx_task,
                                  "espnow_rx",
                                  ESPNOW_RX_TASK_STACK_SIZE,
                                  NULL,
                                  ESPNOW_RX_TASK_PRIORITY,
                                  &s_rx_task_handle);
    if (xret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create RX task");
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ESP-NOW Manager initialized successfully");

    return ESP_OK;
}

esp_err_t espnow_manager_deinit(void)
{
    if (!s_initialized)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing ESP-NOW Manager...");

    /* Delete the RX task */
    if (s_rx_task_handle != NULL)
    {
        vTaskDelete(s_rx_task_handle);
        s_rx_task_handle = NULL;
    }

    /* Delete the RX queue */
    if (s_rx_queue != NULL)
    {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }

    /* Delete the peer mutex */
    if (s_peer_mutex != NULL)
    {
        vSemaphoreDelete(s_peer_mutex);
        s_peer_mutex = NULL;
    }

    /* Deinitialize ESP-NOW */
    esp_now_deinit();

    /* Clear state */
    memset(s_peer_table, 0, sizeof(s_peer_table));
    s_rx_callback = NULL;
    s_tx_callback = NULL;
    s_initialized = false;

    ESP_LOGI(TAG, "ESP-NOW Manager deinitialized");

    return ESP_OK;
}

esp_err_t espnow_manager_add_peer(const uint8_t *mac_addr,
                                  uint8_t channel,
                                  bool encrypted,
                                  const uint8_t *lmk)
{
    if (mac_addr == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized)
    {
        ESP_LOGE(TAG, "Manager not initialized");
        return ESP_FAIL;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    /* Check for duplicate */
    if (find_peer_slot(mac_addr) >= 0)
    {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGW(TAG, "Peer already exists");
        return ESP_ERR_ESPNOW_EXIST;
    }

    /* Find a free slot */
    int slot = find_free_slot();
    if (slot < 0)
    {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGE(TAG, "Peer table full");
        return ESP_ERR_ESPNOW_FULL;
    }

    /* Add to ESP-NOW peer list */
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, mac_addr, ESPNOW_MAC_ADDR_LEN);
    peer_info.channel = (channel > 0) ? channel : s_current_channel;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = encrypted;
    if (encrypted && lmk != NULL)
    {
        memcpy(peer_info.lmk, lmk, ESPNOW_LMK_LEN);
    }

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGE(TAG, "esp_now_add_peer() failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Store in our table */
    memcpy(s_peer_table[slot].mac_addr, mac_addr, ESPNOW_MAC_ADDR_LEN);
    s_peer_table[slot].channel = peer_info.channel;
    s_peer_table[slot].encrypted = encrypted;
    if (lmk != NULL)
    {
        memcpy(s_peer_table[slot].lmk, lmk, ESPNOW_LMK_LEN);
    }
    s_peer_table[slot].active = true;
    s_peer_table[slot].last_seen = esp_timer_get_time();

    xSemaphoreGive(s_peer_mutex);

    char mac_str[18];
    espnow_mac_to_str(mac_addr, mac_str);
    ESP_LOGI(TAG, "Peer added: %s (ch=%d, enc=%d)", mac_str,
             peer_info.channel, encrypted);

    return ESP_OK;
}

esp_err_t espnow_manager_remove_peer(const uint8_t *mac_addr)
{
    if (mac_addr == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized)
    {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    int slot = find_peer_slot(mac_addr);
    if (slot < 0)
    {
        xSemaphoreGive(s_peer_mutex);
        return ESP_ERR_ESPNOW_NOT_FOUND;
    }

    /* Remove from ESP-NOW */
    esp_err_t ret = esp_now_del_peer(mac_addr);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(s_peer_mutex);
        ESP_LOGE(TAG, "esp_now_del_peer() failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Clear our table entry */
    memset(&s_peer_table[slot], 0, sizeof(espnow_peer_info_t));

    xSemaphoreGive(s_peer_mutex);

    char mac_str[18];
    espnow_mac_to_str(mac_addr, mac_str);
    ESP_LOGI(TAG, "Peer removed: %s", mac_str);

    return ESP_OK;
}

bool espnow_manager_peer_exists(const uint8_t *mac_addr)
{
    if (mac_addr == NULL || !s_initialized)
    {
        return false;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    bool exists = (find_peer_slot(mac_addr) >= 0);
    xSemaphoreGive(s_peer_mutex);

    return exists;
}

int espnow_manager_get_peer_count(void)
{
    if (!s_initialized)
    {
        return 0;
    }

    int count = 0;
    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++)
    {
        if (s_peer_table[i].active)
        {
            count++;
        }
    }
    xSemaphoreGive(s_peer_mutex);

    return count;
}

esp_err_t espnow_manager_get_peer_info(int index, espnow_peer_info_t *info)
{
    if (info == NULL || index < 0 || index >= ESPNOW_MAX_PEERS)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized)
    {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_peer_mutex, portMAX_DELAY);

    /* Find the n-th active peer */
    int count = 0;
    for (int i = 0; i < ESPNOW_MAX_PEERS; i++)
    {
        if (s_peer_table[i].active)
        {
            if (count == index)
            {
                memcpy(info, &s_peer_table[i], sizeof(espnow_peer_info_t));
                xSemaphoreGive(s_peer_mutex);
                return ESP_OK;
            }
            count++;
        }
    }

    xSemaphoreGive(s_peer_mutex);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t espnow_manager_send_unicast(const uint8_t *dest_mac,
                                      const uint8_t *data,
                                      uint16_t len)
{
    if (dest_mac == NULL || data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > ESPNOW_MAX_PAYLOAD_SIZE)
    {
        ESP_LOGE(TAG, "Payload too large: %d > %d", len, ESPNOW_MAX_PAYLOAD_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!s_initialized)
    {
        return ESP_FAIL;
    }

    /* Ensure the peer is registered before sending */
    if (!esp_now_is_peer_exist(dest_mac))
    {
        /* Auto-add as an unencrypted peer on the current channel */
        ESP_LOGW(TAG, "Auto-adding peer for unicast send");
        esp_err_t ret = espnow_manager_add_peer(dest_mac, 0, false, NULL);
        if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST)
        {
            return ret;
        }
    }

    esp_err_t ret = esp_now_send(dest_mac, data, len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_now_send() failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t espnow_manager_send_broadcast(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (len > ESPNOW_MAX_PAYLOAD_SIZE)
    {
        ESP_LOGE(TAG, "Payload too large: %d > %d", len, ESPNOW_MAX_PAYLOAD_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!s_initialized)
    {
        return ESP_FAIL;
    }

    const uint8_t broadcast_addr[ESPNOW_MAC_ADDR_LEN] = ESPNOW_BROADCAST_ADDR;

    esp_err_t ret = esp_now_send(broadcast_addr, data, len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Broadcast send failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t espnow_manager_set_channel(uint8_t channel)
{
    if (channel < 1 || channel > 14)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret == ESP_OK)
    {
        s_current_channel = channel;
        ESP_LOGI(TAG, "Channel set to %d", channel);
    }

    return ret;
}

uint8_t espnow_manager_get_channel(void)
{
    return s_current_channel;
}

esp_err_t espnow_manager_get_local_mac(uint8_t *mac_addr)
{
    if (mac_addr == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(mac_addr, s_local_mac, ESPNOW_MAC_ADDR_LEN);
    return ESP_OK;
}

esp_err_t espnow_manager_get_stats(espnow_stats_t *stats)
{
    if (stats == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(stats, &s_stats, sizeof(espnow_stats_t));
    return ESP_OK;
}

void espnow_manager_reset_stats(void)
{
    memset(&s_stats, 0, sizeof(espnow_stats_t));
}

QueueHandle_t espnow_manager_get_rx_queue(void)
{
    return s_rx_queue;
}

void espnow_manager_register_rx_callback(espnow_rx_callback_t cb)
{
    s_rx_callback = cb;
}

void espnow_manager_register_tx_callback(espnow_tx_callback_t cb)
{
    s_tx_callback = cb;
}

void espnow_mac_to_str(const uint8_t *mac, char *buf)
{
    if (mac == NULL || buf == NULL)
    {
        return;
    }

    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
