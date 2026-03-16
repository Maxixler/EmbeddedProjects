/**
 * @file mesh_protocol.h
 * @brief Custom Mesh Protocol Header - Routing table, hop counting,
 *        message forwarding, node discovery, and packet structure definition.
 *
 * Implements a simplified AODV-like (Ad-hoc On-demand Distance Vector)
 * routing protocol on top of ESP-NOW for multi-hop mesh networking.
 *
 * Packet Structure:
 * @code
 *  +--------+------+------+-----+-----+-----------+---------+
 *  | Type   | Src  | Dst  | Seq | TTL | Hop Count | Payload |
 *  | 1 byte | 6 B  | 6 B  | 2 B | 1 B | 1 byte   | N bytes |
 *  +--------+------+------+-----+-----+-----------+---------+
 *  |<---------- Header (17 bytes) ---------->|<-- Variable -->|
 * @endcode
 *
 * @version 1.0.0
 * @date 2026-03-16
 */

#ifndef MESH_PROTOCOL_H
#define MESH_PROTOCOL_H

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------
 * Includes
 * ---------------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "espnow_manager.h"

/* -------------------------------------------------------------------------
 * Compile-Time Configuration
 * ---------------------------------------------------------------------- */

/** @brief Maximum number of nodes in the routing table. */
#define MESH_MAX_ROUTES (40)

/** @brief Maximum Time-To-Live for mesh packets. */
#define MESH_DEFAULT_TTL (10)

/** @brief Beacon interval in milliseconds. */
#define MESH_BEACON_INTERVAL_MS (5000)

/** @brief Route entry expiration time in milliseconds. */
#define MESH_ROUTE_TIMEOUT_MS (30000)

/** @brief Maximum payload size within a mesh packet. */
#define MESH_MAX_PAYLOAD_SIZE (ESPNOW_MAX_PAYLOAD_SIZE - sizeof(mesh_header_t))

/** @brief Size of the duplicate-detection cache. */
#define MESH_DUP_CACHE_SIZE (64)

/** @brief Mesh packet queue depth. */
#define MESH_PACKET_QUEUE_SIZE (32)

/** @brief Maximum number of sequence entries for duplicate detection. */
#define MESH_SEQ_CACHE_SIZE (128)

/** @brief Route request timeout in milliseconds. */
#define MESH_RREQ_TIMEOUT_MS (3000)

/** @brief Maximum route request retries. */
#define MESH_RREQ_MAX_RETRIES (3)

/** @brief Mesh protocol header size in bytes. */
#define MESH_HEADER_SIZE (17)

    /* -------------------------------------------------------------------------
     * Type Definitions
     * ---------------------------------------------------------------------- */

    /**
     * @brief Mesh packet types.
     */
    typedef enum
    {
        MESH_PKT_DATA = 0x01,    /**< User data packet.               */
        MESH_PKT_BEACON = 0x02,  /**< Node discovery beacon.          */
        MESH_PKT_RREQ = 0x03,    /**< Route Request (RREQ).           */
        MESH_PKT_RREP = 0x04,    /**< Route Reply (RREP).             */
        MESH_PKT_RERR = 0x05,    /**< Route Error.                    */
        MESH_PKT_ACK = 0x06,     /**< Mesh-level acknowledgment.      */
        MESH_PKT_SENSOR = 0x07,  /**< Sensor data (application).      */
        MESH_PKT_COMMAND = 0x08, /**< Control command.                */
    } mesh_pkt_type_t;

    /**
     * @brief Mesh packet header (17 bytes, packed).
     *
     * Placed at the beginning of every mesh-layer frame.
     */
    typedef struct __attribute__((packed))
    {
        uint8_t type;                          /**< Packet type (mesh_pkt_type_t). */
        uint8_t src_addr[ESPNOW_MAC_ADDR_LEN]; /**< Original source MAC address.   */
        uint8_t dst_addr[ESPNOW_MAC_ADDR_LEN]; /**< Final destination MAC address.  */
        uint16_t seq_num;                      /**< Sequence number.               */
        uint8_t ttl;                           /**< Time-to-live (decremented each hop). */
        uint8_t hop_count;                     /**< Number of hops traversed.      */
    } mesh_header_t;

    /**
     * @brief Complete mesh packet (header + payload).
     */
    typedef struct
    {
        mesh_header_t header;                   /**< Packet header.                 */
        uint8_t payload[MESH_MAX_PAYLOAD_SIZE]; /**< Payload data.               */
        uint16_t payload_len;                   /**< Actual payload length.         */
    } mesh_packet_t;

    /**
     * @brief Beacon payload sent during node discovery.
     */
    typedef struct __attribute__((packed))
    {
        uint8_t node_mac[ESPNOW_MAC_ADDR_LEN]; /**< Beacon sender MAC.            */
        uint8_t hop_count;                     /**< Hops from originator.         */
        uint8_t peer_count;                    /**< Number of known peers.        */
        uint8_t channel;                       /**< WiFi channel in use.          */
        int8_t rssi;                           /**< Reserved / RSSI placeholder.  */
        uint32_t uptime_sec;                   /**< Node uptime in seconds.       */
    } mesh_beacon_payload_t;

    /**
     * @brief Route Request (RREQ) payload.
     */
    typedef struct __attribute__((packed))
    {
        uint8_t origin[ESPNOW_MAC_ADDR_LEN]; /**< Request originator.           */
        uint8_t target[ESPNOW_MAC_ADDR_LEN]; /**< Desired destination.          */
        uint16_t rreq_id;                    /**< Unique RREQ identifier.       */
        uint8_t hop_count;                   /**< Hops from originator.         */
    } mesh_rreq_payload_t;

    /**
     * @brief Route Reply (RREP) payload.
     */
    typedef struct __attribute__((packed))
    {
        uint8_t origin[ESPNOW_MAC_ADDR_LEN]; /**< Original RREQ originator.     */
        uint8_t target[ESPNOW_MAC_ADDR_LEN]; /**< Destination that was found.   */
        uint8_t hop_count;                   /**< Hops to reach destination.    */
        uint16_t rreq_id;                    /**< Matching RREQ identifier.     */
    } mesh_rrep_payload_t;

    /**
     * @brief Routing table entry.
     */
    typedef struct
    {
        uint8_t dest_addr[ESPNOW_MAC_ADDR_LEN]; /**< Destination node MAC.         */
        uint8_t next_hop[ESPNOW_MAC_ADDR_LEN];  /**< Next hop MAC towards dest.    */
        uint8_t hop_count;                      /**< Total hops to destination.    */
        uint16_t seq_num;                       /**< Destination sequence number.  */
        int64_t last_updated;                   /**< Timestamp of last update (us).*/
        bool valid;                             /**< Entry validity flag.          */
    } mesh_route_entry_t;

    /**
     * @brief Duplicate detection cache entry.
     */
    typedef struct
    {
        uint8_t src_addr[ESPNOW_MAC_ADDR_LEN]; /**< Packet originator MAC.        */
        uint16_t seq_num;                      /**< Sequence number.              */
        int64_t timestamp;                     /**< When this entry was recorded. */
        bool valid;                            /**< Slot in use.                  */
    } mesh_dup_entry_t;

    /**
     * @brief Mesh protocol runtime statistics.
     */
    typedef struct
    {
        uint32_t packets_sent;      /**< Total mesh packets originated.     */
        uint32_t packets_received;  /**< Total mesh packets received for us.*/
        uint32_t packets_forwarded; /**< Total mesh packets forwarded.      */
        uint32_t packets_dropped;   /**< Packets dropped (TTL, dup, etc.).  */
        uint32_t beacons_sent;      /**< Beacons transmitted.               */
        uint32_t beacons_received;  /**< Beacons received.                  */
        uint32_t rreq_sent;         /**< RREQ packets sent.                 */
        uint32_t rrep_sent;         /**< RREP packets sent.                 */
        uint32_t route_errors;      /**< Route errors encountered.          */
        uint32_t dup_detected;      /**< Duplicate packets detected.        */
    } mesh_stats_t;

    /**
     * @brief Mesh node state.
     */
    typedef enum
    {
        MESH_STATE_IDLE = 0,        /**< Not initialised.                   */
        MESH_STATE_DISCOVERING = 1, /**< Actively discovering neighbours.   */
        MESH_STATE_ACTIVE = 2,      /**< Normal operation.                  */
        MESH_STATE_ERROR = 3,       /**< Error state.                       */
    } mesh_state_t;

    /**
     * @brief Application-level callback for received mesh data.
     *
     * @param[in] packet Pointer to the received mesh packet.
     */
    typedef void (*mesh_data_callback_t)(const mesh_packet_t *packet);

    /**
     * @brief Mesh protocol configuration.
     */
    typedef struct
    {
        uint8_t default_ttl;                /**< Default TTL for outgoing packets. */
        uint32_t beacon_interval_ms;        /**< Beacon interval (ms).             */
        uint32_t route_timeout_ms;          /**< Route expiration time (ms).       */
        mesh_data_callback_t data_callback; /**< Callback for received data.       */
        bool enable_forwarding;             /**< Enable multi-hop forwarding.      */
    } mesh_config_t;

    /* -------------------------------------------------------------------------
     * Public API
     * ---------------------------------------------------------------------- */

    /**
     * @brief Initialize the mesh protocol layer.
     *
     * Must be called after espnow_manager_init().
     *
     * @param[in] config Pointer to mesh configuration (NULL for defaults).
     * @return
     *   - ESP_OK on success
     *   - ESP_FAIL if ESP-NOW manager is not initialised
     */
    esp_err_t mesh_protocol_init(const mesh_config_t *config);

    /**
     * @brief Deinitialize the mesh protocol and release resources.
     *
     * @return ESP_OK on success.
     */
    esp_err_t mesh_protocol_deinit(void);

    /**
     * @brief Send a data packet through the mesh to the given destination.
     *
     * If a route exists, the packet is forwarded via the next hop.
     * If no route exists, a RREQ is initiated.
     *
     * @param[in] dest_mac  6-byte destination MAC (use broadcast for flooding).
     * @param[in] data      Pointer to payload data.
     * @param[in] data_len  Length of payload data.
     * @param[in] type      Mesh packet type.
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if route discovery is pending.
     */
    esp_err_t mesh_protocol_send(const uint8_t *dest_mac,
                                 const uint8_t *data,
                                 uint16_t data_len,
                                 mesh_pkt_type_t type);

    /**
     * @brief Initiate a route request to the given destination.
     *
     * @param[in] dest_mac 6-byte destination MAC.
     * @return ESP_OK on success.
     */
    esp_err_t mesh_protocol_route_request(const uint8_t *dest_mac);

    /**
     * @brief Look up a route to the given destination.
     *
     * @param[in]  dest_mac 6-byte destination MAC.
     * @param[out] entry    Pointer to route entry to fill (may be NULL).
     * @return true if a valid route exists, false otherwise.
     */
    bool mesh_protocol_route_lookup(const uint8_t *dest_mac,
                                    mesh_route_entry_t *entry);

    /**
     * @brief Manually add or update a route in the routing table.
     *
     * @param[in] dest_mac  Destination MAC address.
     * @param[in] next_hop  Next hop MAC address.
     * @param[in] hop_count Number of hops to the destination.
     * @return ESP_OK on success, ESP_ERR_NO_MEM if table is full.
     */
    esp_err_t mesh_protocol_route_add(const uint8_t *dest_mac,
                                      const uint8_t *next_hop,
                                      uint8_t hop_count);

    /**
     * @brief Remove a route from the routing table.
     *
     * @param[in] dest_mac Destination MAC address.
     * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not present.
     */
    esp_err_t mesh_protocol_route_remove(const uint8_t *dest_mac);

    /**
     * @brief Get the number of valid routes in the routing table.
     *
     * @return Number of valid route entries.
     */
    int mesh_protocol_route_count(void);

    /**
     * @brief Retrieve a route entry by index.
     *
     * @param[in]  index  0-based index.
     * @param[out] entry  Pointer to route entry to fill.
     * @return ESP_OK on success.
     */
    esp_err_t mesh_protocol_route_get(int index, mesh_route_entry_t *entry);

    /**
     * @brief Purge expired routes from the routing table.
     *
     * Called automatically by the beacon task but can also be
     * invoked manually.
     */
    void mesh_protocol_purge_routes(void);

    /**
     * @brief Print the routing table to the serial console (for debugging).
     */
    void mesh_protocol_print_routes(void);

    /**
     * @brief Get mesh protocol statistics.
     *
     * @param[out] stats Pointer to statistics structure to fill.
     * @return ESP_OK on success.
     */
    esp_err_t mesh_protocol_get_stats(mesh_stats_t *stats);

    /**
     * @brief Reset mesh statistics counters to zero.
     */
    void mesh_protocol_reset_stats(void);

    /**
     * @brief Get the current mesh state.
     *
     * @return Current mesh_state_t value.
     */
    mesh_state_t mesh_protocol_get_state(void);

    /**
     * @brief Get the local node's next sequence number (and increment).
     *
     * @return Current sequence number (pre-increment).
     */
    uint16_t mesh_protocol_next_seq(void);

    /**
     * @brief Register a callback for received mesh data.
     *
     * @param[in] cb Callback function pointer (NULL to unregister).
     */
    void mesh_protocol_register_data_callback(mesh_data_callback_t cb);

    /**
     * @brief Process an incoming ESP-NOW frame at the mesh layer.
     *
     * This is called internally by the ESP-NOW receive callback but
     * is exposed for testing and manual injection.
     *
     * @param[in] src_mac  6-byte MAC of the immediate sender.
     * @param[in] data     Raw frame payload.
     * @param[in] data_len Length of the frame payload.
     */
    void mesh_protocol_process_incoming(const uint8_t *src_mac,
                                        const uint8_t *data,
                                        uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* MESH_PROTOCOL_H */
