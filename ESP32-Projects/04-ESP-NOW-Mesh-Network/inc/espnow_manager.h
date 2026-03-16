/**
 * @file espnow_manager.h
 * @brief ESP-NOW Manager Header - Peer management, send/receive callbacks,
 *        message types, broadcast/unicast support.
 *
 * This module provides a high-level API for ESP-NOW peer-to-peer communication
 * on the ESP32 platform. It handles WiFi initialization in STA mode, peer
 * registration, encrypted communication via PMK/LMK, and asynchronous
 * message delivery with callback-based notification.
 *
 * @version 1.0.0
 * @date 2026-03-16
 *
 * @note Requires ESP-IDF v5.x or later.
 * @note Maximum 20 peers (ESP-NOW hardware limit).
 * @note Maximum payload size is 250 bytes per frame.
 */

#ifndef ESPNOW_MANAGER_H
#define ESPNOW_MANAGER_H

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------
 * Includes
 * ---------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* -------------------------------------------------------------------------
 * Compile-Time Configuration
 * ---------------------------------------------------------------------- */

/** @brief Maximum number of managed peers (ESP-NOW limit is 20). */
#define ESPNOW_MAX_PEERS (20)

/** @brief Maximum ESP-NOW payload size in bytes. */
#define ESPNOW_MAX_PAYLOAD_SIZE (250)

/** @brief Length of a MAC address in bytes. */
#define ESPNOW_MAC_ADDR_LEN (6)

/** @brief Default WiFi channel for ESP-NOW communication. */
#define ESPNOW_DEFAULT_CHANNEL (1)

/** @brief Size of the receive queue (number of pending messages). */
#define ESPNOW_RX_QUEUE_SIZE (32)

/** @brief Primary Master Key length (16 bytes). */
#define ESPNOW_PMK_LEN (16)

/** @brief Local Master Key length (16 bytes). */
#define ESPNOW_LMK_LEN (16)

/** @brief Broadcast MAC address. */
#define ESPNOW_BROADCAST_ADDR {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

    /* -------------------------------------------------------------------------
     * Type Definitions
     * ---------------------------------------------------------------------- */

    /**
     * @brief ESP-NOW message types used in the protocol header.
     */
    typedef enum
    {
        ESPNOW_MSG_TYPE_DATA = 0x01,      /**< Generic data message.          */
        ESPNOW_MSG_TYPE_BEACON = 0x02,    /**< Node discovery beacon.         */
        ESPNOW_MSG_TYPE_ACK = 0x03,       /**< Acknowledgment message.        */
        ESPNOW_MSG_TYPE_ROUTE_REQ = 0x04, /**< Route request (RREQ).          */
        ESPNOW_MSG_TYPE_ROUTE_REP = 0x05, /**< Route reply (RREP).            */
        ESPNOW_MSG_TYPE_MESH = 0x06,      /**< Mesh-layer forwarded message.  */
        ESPNOW_MSG_TYPE_COMMAND = 0x07,   /**< Remote command / control.      */
        ESPNOW_MSG_TYPE_SENSOR = 0x08,    /**< Sensor data payload.           */
        ESPNOW_MSG_TYPE_MAX               /**< Sentinel (do not use).          */
    } espnow_msg_type_t;

    /**
     * @brief Delivery status reported by the send callback.
     */
    typedef enum
    {
        ESPNOW_SEND_OK = 0,  /**< Frame was acknowledged by the receiver. */
        ESPNOW_SEND_FAIL = 1 /**< Frame delivery failed.                  */
    } espnow_send_status_t;

    /**
     * @brief Peer information stored in the manager's peer table.
     */
    typedef struct
    {
        uint8_t mac_addr[ESPNOW_MAC_ADDR_LEN]; /**< Peer MAC address.        */
        uint8_t channel;                       /**< WiFi channel.            */
        bool encrypted;                        /**< Encryption enabled flag. */
        uint8_t lmk[ESPNOW_LMK_LEN];           /**< Local Master Key.        */
        bool active;                           /**< Slot in use flag.        */
        int64_t last_seen;                     /**< Last activity timestamp. */
    } espnow_peer_info_t;

    /**
     * @brief Structure representing a received ESP-NOW message.
     *
     * Placed into the RX queue by the receive callback for processing
     * in application context (not ISR).
     */
    typedef struct
    {
        uint8_t src_mac[ESPNOW_MAC_ADDR_LEN];  /**< Source MAC address.       */
        uint8_t data[ESPNOW_MAX_PAYLOAD_SIZE]; /**< Received payload.        */
        uint16_t data_len;                     /**< Length of payload.        */
        int rssi;                              /**< RSSI of received frame.  */
        int64_t timestamp;                     /**< Reception timestamp (us).*/
    } espnow_rx_msg_t;

    /**
     * @brief Structure representing a send-complete event.
     */
    typedef struct
    {
        uint8_t dest_mac[ESPNOW_MAC_ADDR_LEN]; /**< Destination MAC.  */
        espnow_send_status_t status;           /**< Delivery status.  */
    } espnow_tx_event_t;

    /**
     * @brief Application-level callback invoked when a message is received.
     *
     * @param[in] msg Pointer to the received message structure.
     */
    typedef void (*espnow_rx_callback_t)(const espnow_rx_msg_t *msg);

    /**
     * @brief Application-level callback invoked on send completion.
     *
     * @param[in] event Pointer to the send event structure.
     */
    typedef void (*espnow_tx_callback_t)(const espnow_tx_event_t *event);

    /**
     * @brief ESP-NOW manager configuration structure.
     */
    typedef struct
    {
        uint8_t channel;                  /**< WiFi channel (1-14).       */
        uint8_t pmk[ESPNOW_PMK_LEN];      /**< Primary Master Key.     */
        bool enable_encryption;           /**< Enable PMK encryption.     */
        espnow_rx_callback_t rx_callback; /**< Receive callback (or NULL).*/
        espnow_tx_callback_t tx_callback; /**< Send callback (or NULL).   */
    } espnow_config_t;

    /**
     * @brief ESP-NOW manager runtime statistics.
     */
    typedef struct
    {
        uint32_t tx_success; /**< Frames sent successfully.    */
        uint32_t tx_fail;    /**< Frames that failed to send.  */
        uint32_t rx_count;   /**< Frames received.             */
        uint32_t rx_dropped; /**< Frames dropped (queue full). */
    } espnow_stats_t;

    /* -------------------------------------------------------------------------
     * Public API
     * ---------------------------------------------------------------------- */

    /**
     * @brief Initialize the ESP-NOW manager.
     *
     * Sets up WiFi in STA mode (no AP connection), initialises ESP-NOW,
     * configures the PMK if encryption is enabled, and starts the
     * internal receive-processing task.
     *
     * @param[in] config Pointer to the configuration structure.
     * @return
     *   - ESP_OK on success
     *   - ESP_ERR_INVALID_ARG if config is NULL
     *   - ESP_FAIL on internal error
     */
    esp_err_t espnow_manager_init(const espnow_config_t *config);

    /**
     * @brief Deinitialize the ESP-NOW manager and release all resources.
     *
     * @return ESP_OK on success.
     */
    esp_err_t espnow_manager_deinit(void);

    /**
     * @brief Add a peer to the ESP-NOW peer list.
     *
     * @param[in] mac_addr  6-byte MAC address of the peer.
     * @param[in] channel   WiFi channel (0 = use current channel).
     * @param[in] encrypted true to enable encryption for this peer.
     * @param[in] lmk       16-byte Local Master Key (NULL if unencrypted).
     * @return
     *   - ESP_OK on success
     *   - ESP_ERR_ESPNOW_FULL if peer table is full
     *   - ESP_ERR_ESPNOW_EXIST if peer already exists
     */
    esp_err_t espnow_manager_add_peer(const uint8_t *mac_addr,
                                      uint8_t channel,
                                      bool encrypted,
                                      const uint8_t *lmk);

    /**
     * @brief Remove a peer from the ESP-NOW peer list.
     *
     * @param[in] mac_addr 6-byte MAC address of the peer to remove.
     * @return
     *   - ESP_OK on success
     *   - ESP_ERR_ESPNOW_NOT_FOUND if peer is not registered
     */
    esp_err_t espnow_manager_remove_peer(const uint8_t *mac_addr);

    /**
     * @brief Check whether a peer is registered.
     *
     * @param[in] mac_addr 6-byte MAC address.
     * @return true if the peer exists, false otherwise.
     */
    bool espnow_manager_peer_exists(const uint8_t *mac_addr);

    /**
     * @brief Get the number of currently registered peers.
     *
     * @return Number of peers.
     */
    int espnow_manager_get_peer_count(void);

    /**
     * @brief Retrieve information about a peer by index.
     *
     * @param[in]  index Index (0-based) into the peer table.
     * @param[out] info  Pointer to structure to fill.
     * @return
     *   - ESP_OK on success
     *   - ESP_ERR_INVALID_ARG if index is out of range or info is NULL
     */
    esp_err_t espnow_manager_get_peer_info(int index, espnow_peer_info_t *info);

    /**
     * @brief Send data to a specific peer (unicast).
     *
     * @param[in] dest_mac 6-byte destination MAC address.
     * @param[in] data     Pointer to payload data.
     * @param[in] len      Length of payload (max ESPNOW_MAX_PAYLOAD_SIZE).
     * @return
     *   - ESP_OK if the frame was queued for transmission
     *   - ESP_ERR_INVALID_ARG on invalid parameters
     */
    esp_err_t espnow_manager_send_unicast(const uint8_t *dest_mac,
                                          const uint8_t *data,
                                          uint16_t len);

    /**
     * @brief Broadcast data to all registered peers.
     *
     * The broadcast address FF:FF:FF:FF:FF:FF is used, which means
     * all ESP-NOW receivers on the same channel will pick it up.
     *
     * @param[in] data Pointer to payload data.
     * @param[in] len  Length of payload (max ESPNOW_MAX_PAYLOAD_SIZE).
     * @return ESP_OK on success.
     */
    esp_err_t espnow_manager_send_broadcast(const uint8_t *data, uint16_t len);

    /**
     * @brief Set the WiFi channel used for ESP-NOW communication.
     *
     * @param[in] channel WiFi channel (1-14).
     * @return ESP_OK on success.
     */
    esp_err_t espnow_manager_set_channel(uint8_t channel);

    /**
     * @brief Get the current WiFi channel.
     *
     * @return Current channel number.
     */
    uint8_t espnow_manager_get_channel(void);

    /**
     * @brief Get the local device MAC address.
     *
     * @param[out] mac_addr Buffer to receive the 6-byte MAC address.
     * @return ESP_OK on success.
     */
    esp_err_t espnow_manager_get_local_mac(uint8_t *mac_addr);

    /**
     * @brief Get runtime statistics.
     *
     * @param[out] stats Pointer to statistics structure to fill.
     * @return ESP_OK on success.
     */
    esp_err_t espnow_manager_get_stats(espnow_stats_t *stats);

    /**
     * @brief Reset runtime statistics counters to zero.
     */
    void espnow_manager_reset_stats(void);

    /**
     * @brief Get the internal receive queue handle (for external consumers).
     *
     * @return FreeRTOS queue handle, or NULL if not initialised.
     */
    QueueHandle_t espnow_manager_get_rx_queue(void);

    /**
     * @brief Register (or replace) the application-level receive callback.
     *
     * @param[in] cb Callback function pointer (NULL to unregister).
     */
    void espnow_manager_register_rx_callback(espnow_rx_callback_t cb);

    /**
     * @brief Register (or replace) the application-level send callback.
     *
     * @param[in] cb Callback function pointer (NULL to unregister).
     */
    void espnow_manager_register_tx_callback(espnow_tx_callback_t cb);

    /**
     * @brief Format a MAC address into a human-readable string.
     *
     * @param[in]  mac MAC address (6 bytes).
     * @param[out] buf Output buffer (at least 18 bytes: "XX:XX:XX:XX:XX:XX\0").
     */
    void espnow_mac_to_str(const uint8_t *mac, char *buf);

#ifdef __cplusplus
}
#endif

#endif /* ESPNOW_MANAGER_H */
