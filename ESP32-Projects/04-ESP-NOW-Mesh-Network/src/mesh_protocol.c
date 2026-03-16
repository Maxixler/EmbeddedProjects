/**
 * @file mesh_protocol.c
 * @brief Mesh Protocol Implementation.
 *
 * Implements a simplified AODV-like mesh networking protocol on top of
 * ESP-NOW. Features include:
 *   - Node discovery via periodic broadcast beacons
 *   - Routing table management (add/remove/update/purge)
 *   - Multi-hop message forwarding with TTL
 *   - Route request/reply mechanism (RREQ/RREP)
 *   - Duplicate message detection via sequence numbers
 *   - Statistics tracking (sent, received, forwarded, dropped)
 *
 * @version 1.0.0
 * @date 2026-03-16
 */

/* -------------------------------------------------------------------------
 * Includes
 * ---------------------------------------------------------------------- */
#include "mesh_protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"

/* -------------------------------------------------------------------------
 * Private Defines
 * ---------------------------------------------------------------------- */

/** @brief Log tag for this module. */
static const char *TAG = "MESH_PROTO";

/** @brief Stack size for the beacon task. */
#define BEACON_TASK_STACK_SIZE (4096)

/** @brief Priority for the beacon task. */
#define BEACON_TASK_PRIORITY (4)

/** @brief Stack size for the mesh processing task. */
#define MESH_PROC_TASK_STACK_SIZE (8192)

/** @brief Priority for the mesh processing task. */
#define MESH_PROC_TASK_PRIORITY (5)

/** @brief Broadcast MAC constant for comparisons. */
static const uint8_t BROADCAST_MAC[ESPNOW_MAC_ADDR_LEN] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* -------------------------------------------------------------------------
 * Private Data
 * ---------------------------------------------------------------------- */

/** @brief Whether the mesh protocol has been initialized. */
static bool s_mesh_initialized = false;

/** @brief Current mesh state. */
static mesh_state_t s_mesh_state = MESH_STATE_IDLE;

/** @brief Routing table. */
static mesh_route_entry_t s_route_table[MESH_MAX_ROUTES];

/** @brief Mutex for routing table access. */
static SemaphoreHandle_t s_route_mutex = NULL;

/** @brief Duplicate detection cache. */
static mesh_dup_entry_t s_dup_cache[MESH_DUP_CACHE_SIZE];

/** @brief Mutex for duplicate cache access. */
static SemaphoreHandle_t s_dup_mutex = NULL;

/** @brief Mesh statistics. */
static mesh_stats_t s_mesh_stats = {0};

/** @brief Local node's sequence number counter. */
static uint16_t s_local_seq = 0;

/** @brief RREQ ID counter. */
static uint16_t s_rreq_id = 0;

/** @brief Mesh configuration (runtime copy). */
static mesh_config_t s_mesh_config = {0};

/** @brief Local MAC address cache. */
static uint8_t s_local_mac[ESPNOW_MAC_ADDR_LEN] = {0};

/** @brief Beacon task handle. */
static TaskHandle_t s_beacon_task_handle = NULL;

/** @brief Mesh processing task handle. */
static TaskHandle_t s_mesh_proc_task_handle = NULL;

/** @brief Application data callback. */
static mesh_data_callback_t s_data_callback = NULL;

/** @brief Boot time for uptime calculation. */
static int64_t s_boot_time = 0;

/* -------------------------------------------------------------------------
 * Private Function Prototypes
 * ---------------------------------------------------------------------- */

static void mesh_beacon_task(void *arg);
static void mesh_espnow_rx_handler(const espnow_rx_msg_t *msg);
static void handle_beacon(const uint8_t *sender_mac,
                          const mesh_header_t *header,
                          const uint8_t *payload, uint16_t payload_len);
static void handle_data_packet(const uint8_t *sender_mac,
                               const mesh_header_t *header,
                               const uint8_t *payload, uint16_t payload_len);
static void handle_rreq(const uint8_t *sender_mac,
                        const mesh_header_t *header,
                        const uint8_t *payload, uint16_t payload_len);
static void handle_rrep(const uint8_t *sender_mac,
                        const mesh_header_t *header,
                        const uint8_t *payload, uint16_t payload_len);
static bool is_duplicate(const uint8_t *src_addr, uint16_t seq_num);
static void add_to_dup_cache(const uint8_t *src_addr, uint16_t seq_num);
static bool is_local_mac(const uint8_t *mac);
static bool is_broadcast_mac(const uint8_t *mac);
static esp_err_t forward_packet(const uint8_t *sender_mac,
                                const mesh_header_t *header,
                                const uint8_t *payload, uint16_t payload_len);
static int find_route_slot(const uint8_t *dest_mac);
static int find_free_route_slot(void);
static void send_beacon(void);
static esp_err_t send_mesh_packet(const uint8_t *next_hop_mac,
                                  const mesh_header_t *header,
                                  const uint8_t *payload, uint16_t payload_len);

/* -------------------------------------------------------------------------
 * Public API Implementation
 * ---------------------------------------------------------------------- */

esp_err_t mesh_protocol_init(const mesh_config_t *config)
{
    if (s_mesh_initialized)
    {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing mesh protocol...");

    /* Apply configuration (use defaults if NULL) */
    if (config != NULL)
    {
        memcpy(&s_mesh_config, config, sizeof(mesh_config_t));
    }
    else
    {
        s_mesh_config.default_ttl = MESH_DEFAULT_TTL;
        s_mesh_config.beacon_interval_ms = MESH_BEACON_INTERVAL_MS;
        s_mesh_config.route_timeout_ms = MESH_ROUTE_TIMEOUT_MS;
        s_mesh_config.data_callback = NULL;
        s_mesh_config.enable_forwarding = true;
    }

    s_data_callback = s_mesh_config.data_callback;

    /* Get local MAC */
    esp_err_t ret = espnow_manager_get_local_mac(s_local_mac);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get local MAC");
        return ESP_FAIL;
    }

    char mac_str[18];
    espnow_mac_to_str(s_local_mac, mac_str);
    ESP_LOGI(TAG, "Local mesh node MAC: %s", mac_str);

    /* Clear tables */
    memset(s_route_table, 0, sizeof(s_route_table));
    memset(s_dup_cache, 0, sizeof(s_dup_cache));
    memset(&s_mesh_stats, 0, sizeof(s_mesh_stats));
    s_local_seq = 0;
    s_rreq_id = 0;
    s_boot_time = esp_timer_get_time();

    /* Create mutexes */
    s_route_mutex = xSemaphoreCreateMutex();
    if (s_route_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create route mutex");
        return ESP_FAIL;
    }

    s_dup_mutex = xSemaphoreCreateMutex();
    if (s_dup_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create dup mutex");
        return ESP_FAIL;
    }

    /* Register our handler with the ESP-NOW manager */
    espnow_manager_register_rx_callback(mesh_espnow_rx_handler);

    /* Create beacon task */
    BaseType_t xret = xTaskCreate(mesh_beacon_task,
                                  "mesh_beacon",
                                  BEACON_TASK_STACK_SIZE,
                                  NULL,
                                  BEACON_TASK_PRIORITY,
                                  &s_beacon_task_handle);
    if (xret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create beacon task");
        return ESP_FAIL;
    }

    s_mesh_state = MESH_STATE_DISCOVERING;
    s_mesh_initialized = true;

    ESP_LOGI(TAG, "Mesh protocol initialized (TTL=%d, beacon=%dms)",
             s_mesh_config.default_ttl,
             (int)s_mesh_config.beacon_interval_ms);

    return ESP_OK;
}

esp_err_t mesh_protocol_deinit(void)
{
    if (!s_mesh_initialized)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing mesh protocol...");

    /* Delete tasks */
    if (s_beacon_task_handle != NULL)
    {
        vTaskDelete(s_beacon_task_handle);
        s_beacon_task_handle = NULL;
    }

    if (s_mesh_proc_task_handle != NULL)
    {
        vTaskDelete(s_mesh_proc_task_handle);
        s_mesh_proc_task_handle = NULL;
    }

    /* Unregister callback */
    espnow_manager_register_rx_callback(NULL);

    /* Delete mutexes */
    if (s_route_mutex != NULL)
    {
        vSemaphoreDelete(s_route_mutex);
        s_route_mutex = NULL;
    }
    if (s_dup_mutex != NULL)
    {
        vSemaphoreDelete(s_dup_mutex);
        s_dup_mutex = NULL;
    }

    s_mesh_state = MESH_STATE_IDLE;
    s_mesh_initialized = false;

    ESP_LOGI(TAG, "Mesh protocol deinitialized");

    return ESP_OK;
}

esp_err_t mesh_protocol_send(const uint8_t *dest_mac,
                             const uint8_t *data,
                             uint16_t data_len,
                             mesh_pkt_type_t type)
{
    if (dest_mac == NULL || data == NULL || data_len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (data_len > MESH_MAX_PAYLOAD_SIZE)
    {
        ESP_LOGE(TAG, "Payload exceeds mesh maximum: %d > %d",
                 data_len, (int)MESH_MAX_PAYLOAD_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    if (!s_mesh_initialized)
    {
        return ESP_FAIL;
    }

    /* Build the mesh header */
    mesh_header_t header = {0};
    header.type = (uint8_t)type;
    memcpy(header.src_addr, s_local_mac, ESPNOW_MAC_ADDR_LEN);
    memcpy(header.dst_addr, dest_mac, ESPNOW_MAC_ADDR_LEN);
    header.seq_num = mesh_protocol_next_seq();
    header.ttl = s_mesh_config.default_ttl;
    header.hop_count = 0;

    /* Add to duplicate cache (so we don't process our own packet) */
    add_to_dup_cache(s_local_mac, header.seq_num);

    /* Determine next hop */
    if (is_broadcast_mac(dest_mac))
    {
        /* Broadcast: send to all */
        esp_err_t ret = send_mesh_packet(NULL, &header, data, data_len);
        if (ret == ESP_OK)
        {
            s_mesh_stats.packets_sent++;
        }
        return ret;
    }

    /* Unicast: look up route */
    mesh_route_entry_t route;
    if (mesh_protocol_route_lookup(dest_mac, &route))
    {
        esp_err_t ret = send_mesh_packet(route.next_hop, &header, data, data_len);
        if (ret == ESP_OK)
        {
            s_mesh_stats.packets_sent++;
        }
        return ret;
    }

    /* No route found - initiate route discovery */
    ESP_LOGW(TAG, "No route to destination, initiating RREQ");
    mesh_protocol_route_request(dest_mac);

    return ESP_ERR_NOT_FOUND;
}

esp_err_t mesh_protocol_route_request(const uint8_t *dest_mac)
{
    if (dest_mac == NULL || !s_mesh_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Build RREQ payload */
    mesh_rreq_payload_t rreq = {0};
    memcpy(rreq.origin, s_local_mac, ESPNOW_MAC_ADDR_LEN);
    memcpy(rreq.target, dest_mac, ESPNOW_MAC_ADDR_LEN);
    rreq.rreq_id = s_rreq_id++;
    rreq.hop_count = 0;

    /* Build mesh header for RREQ (broadcast) */
    mesh_header_t header = {0};
    header.type = MESH_PKT_RREQ;
    memcpy(header.src_addr, s_local_mac, ESPNOW_MAC_ADDR_LEN);
    memset(header.dst_addr, 0xFF, ESPNOW_MAC_ADDR_LEN); /* broadcast */
    header.seq_num = mesh_protocol_next_seq();
    header.ttl = s_mesh_config.default_ttl;
    header.hop_count = 0;

    /* Add to dup cache */
    add_to_dup_cache(s_local_mac, header.seq_num);

    /* Broadcast the RREQ */
    esp_err_t ret = send_mesh_packet(NULL, &header,
                                     (const uint8_t *)&rreq, sizeof(rreq));
    if (ret == ESP_OK)
    {
        s_mesh_stats.rreq_sent++;
        char mac_str[18];
        espnow_mac_to_str(dest_mac, mac_str);
        ESP_LOGI(TAG, "RREQ sent for destination %s (id=%d)",
                 mac_str, rreq.rreq_id);
    }

    return ret;
}

bool mesh_protocol_route_lookup(const uint8_t *dest_mac,
                                mesh_route_entry_t *entry)
{
    if (dest_mac == NULL || !s_mesh_initialized)
    {
        return false;
    }

    xSemaphoreTake(s_route_mutex, portMAX_DELAY);

    int idx = find_route_slot(dest_mac);
    if (idx >= 0 && s_route_table[idx].valid)
    {
        /* Check if route is expired */
        int64_t now = esp_timer_get_time();
        int64_t age = (now - s_route_table[idx].last_updated) / 1000; /* ms */

        if (age < (int64_t)s_mesh_config.route_timeout_ms)
        {
            if (entry != NULL)
            {
                memcpy(entry, &s_route_table[idx], sizeof(mesh_route_entry_t));
            }
            xSemaphoreGive(s_route_mutex);
            return true;
        }

        /* Route expired */
        s_route_table[idx].valid = false;
    }

    xSemaphoreGive(s_route_mutex);
    return false;
}

esp_err_t mesh_protocol_route_add(const uint8_t *dest_mac,
                                  const uint8_t *next_hop,
                                  uint8_t hop_count)
{
    if (dest_mac == NULL || next_hop == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_mesh_initialized)
    {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_route_mutex, portMAX_DELAY);

    int idx = find_route_slot(dest_mac);
    if (idx < 0)
    {
        /* New entry */
        idx = find_free_route_slot();
        if (idx < 0)
        {
            xSemaphoreGive(s_route_mutex);
            ESP_LOGW(TAG, "Routing table full");
            return ESP_ERR_NO_MEM;
        }
    }
    else
    {
        /* Update only if the new route is better (fewer hops) */
        if (s_route_table[idx].valid &&
            s_route_table[idx].hop_count <= hop_count)
        {
            xSemaphoreGive(s_route_mutex);
            return ESP_OK; /* Existing route is at least as good */
        }
    }

    memcpy(s_route_table[idx].dest_addr, dest_mac, ESPNOW_MAC_ADDR_LEN);
    memcpy(s_route_table[idx].next_hop, next_hop, ESPNOW_MAC_ADDR_LEN);
    s_route_table[idx].hop_count = hop_count;
    s_route_table[idx].seq_num = 0;
    s_route_table[idx].last_updated = esp_timer_get_time();
    s_route_table[idx].valid = true;

    xSemaphoreGive(s_route_mutex);

    char dest_str[18], next_str[18];
    espnow_mac_to_str(dest_mac, dest_str);
    espnow_mac_to_str(next_hop, next_str);
    ESP_LOGI(TAG, "Route added/updated: %s via %s (hops=%d)",
             dest_str, next_str, hop_count);

    /* Ensure the next hop is registered as an ESP-NOW peer */
    if (!espnow_manager_peer_exists(next_hop))
    {
        espnow_manager_add_peer(next_hop, 0, false, NULL);
    }

    return ESP_OK;
}

esp_err_t mesh_protocol_route_remove(const uint8_t *dest_mac)
{
    if (dest_mac == NULL || !s_mesh_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_route_mutex, portMAX_DELAY);

    int idx = find_route_slot(dest_mac);
    if (idx < 0)
    {
        xSemaphoreGive(s_route_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    memset(&s_route_table[idx], 0, sizeof(mesh_route_entry_t));

    xSemaphoreGive(s_route_mutex);
    return ESP_OK;
}

int mesh_protocol_route_count(void)
{
    if (!s_mesh_initialized)
    {
        return 0;
    }

    int count = 0;
    xSemaphoreTake(s_route_mutex, portMAX_DELAY);
    for (int i = 0; i < MESH_MAX_ROUTES; i++)
    {
        if (s_route_table[i].valid)
        {
            count++;
        }
    }
    xSemaphoreGive(s_route_mutex);

    return count;
}

esp_err_t mesh_protocol_route_get(int index, mesh_route_entry_t *entry)
{
    if (entry == NULL || index < 0 || !s_mesh_initialized)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_route_mutex, portMAX_DELAY);

    int count = 0;
    for (int i = 0; i < MESH_MAX_ROUTES; i++)
    {
        if (s_route_table[i].valid)
        {
            if (count == index)
            {
                memcpy(entry, &s_route_table[i], sizeof(mesh_route_entry_t));
                xSemaphoreGive(s_route_mutex);
                return ESP_OK;
            }
            count++;
        }
    }

    xSemaphoreGive(s_route_mutex);
    return ESP_ERR_INVALID_ARG;
}

void mesh_protocol_purge_routes(void)
{
    if (!s_mesh_initialized)
    {
        return;
    }

    int64_t now = esp_timer_get_time();
    int purged = 0;

    xSemaphoreTake(s_route_mutex, portMAX_DELAY);

    for (int i = 0; i < MESH_MAX_ROUTES; i++)
    {
        if (s_route_table[i].valid)
        {
            int64_t age_ms = (now - s_route_table[i].last_updated) / 1000;
            if (age_ms > (int64_t)s_mesh_config.route_timeout_ms)
            {
                char mac_str[18];
                espnow_mac_to_str(s_route_table[i].dest_addr, mac_str);
                ESP_LOGD(TAG, "Purging expired route to %s", mac_str);
                memset(&s_route_table[i], 0, sizeof(mesh_route_entry_t));
                purged++;
            }
        }
    }

    xSemaphoreGive(s_route_mutex);

    if (purged > 0)
    {
        ESP_LOGI(TAG, "Purged %d expired routes", purged);
    }
}

void mesh_protocol_print_routes(void)
{
    if (!s_mesh_initialized)
    {
        printf("Mesh protocol not initialized.\n");
        return;
    }

    xSemaphoreTake(s_route_mutex, portMAX_DELAY);

    printf("\n");
    printf("+-----+-------------------+-------------------+------+\n");
    printf("| Idx | Destination       | Next Hop          | Hops |\n");
    printf("+-----+-------------------+-------------------+------+\n");

    int count = 0;
    for (int i = 0; i < MESH_MAX_ROUTES; i++)
    {
        if (s_route_table[i].valid)
        {
            char dest_str[18], next_str[18];
            espnow_mac_to_str(s_route_table[i].dest_addr, dest_str);
            espnow_mac_to_str(s_route_table[i].next_hop, next_str);
            printf("| %3d | %s | %s | %4d |\n",
                   count, dest_str, next_str, s_route_table[i].hop_count);
            count++;
        }
    }

    if (count == 0)
    {
        printf("|              (routing table empty)               |\n");
    }

    printf("+-----+-------------------+-------------------+------+\n");
    printf("Total routes: %d\n\n", count);

    xSemaphoreGive(s_route_mutex);
}

esp_err_t mesh_protocol_get_stats(mesh_stats_t *stats)
{
    if (stats == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(stats, &s_mesh_stats, sizeof(mesh_stats_t));
    return ESP_OK;
}

void mesh_protocol_reset_stats(void)
{
    memset(&s_mesh_stats, 0, sizeof(mesh_stats_t));
}

mesh_state_t mesh_protocol_get_state(void)
{
    return s_mesh_state;
}

uint16_t mesh_protocol_next_seq(void)
{
    return s_local_seq++;
}

void mesh_protocol_register_data_callback(mesh_data_callback_t cb)
{
    s_data_callback = cb;
}

void mesh_protocol_process_incoming(const uint8_t *src_mac,
                                    const uint8_t *data,
                                    uint16_t data_len)
{
    mesh_espnow_rx_handler(&(espnow_rx_msg_t){
        .data_len = data_len,
        .timestamp = esp_timer_get_time()});
    /* Note: the actual handler reads from the msg struct, so this
     * public wrapper creates a synthetic message. For full fidelity,
     * use the internal handler directly via the RX callback chain. */
}

/* -------------------------------------------------------------------------
 * Private Function Implementations
 * ---------------------------------------------------------------------- */

/**
 * @brief Periodic beacon transmission task.
 *
 * Sends discovery beacons at the configured interval and purges
 * expired routes.
 *
 * @param[in] arg Unused.
 */
static void mesh_beacon_task(void *arg)
{
    ESP_LOGI(TAG, "Beacon task started (interval=%dms)",
             (int)s_mesh_config.beacon_interval_ms);

    TickType_t interval = pdMS_TO_TICKS(s_mesh_config.beacon_interval_ms);

    while (true)
    {
        vTaskDelay(interval);

        /* Send a beacon */
        send_beacon();

        /* Purge expired routes */
        mesh_protocol_purge_routes();

        /* Transition to ACTIVE state once we have discovered peers */
        if (s_mesh_state == MESH_STATE_DISCOVERING &&
            mesh_protocol_route_count() > 0)
        {
            s_mesh_state = MESH_STATE_ACTIVE;
            ESP_LOGI(TAG, "Mesh state -> ACTIVE");
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Send a discovery beacon via broadcast.
 */
static void send_beacon(void)
{
    mesh_header_t header = {0};
    header.type = MESH_PKT_BEACON;
    memcpy(header.src_addr, s_local_mac, ESPNOW_MAC_ADDR_LEN);
    memset(header.dst_addr, 0xFF, ESPNOW_MAC_ADDR_LEN);
    header.seq_num = mesh_protocol_next_seq();
    header.ttl = 1; /* Beacons are single-hop only */
    header.hop_count = 0;

    /* Build beacon payload */
    mesh_beacon_payload_t beacon = {0};
    memcpy(beacon.node_mac, s_local_mac, ESPNOW_MAC_ADDR_LEN);
    beacon.hop_count = 0;
    beacon.peer_count = (uint8_t)espnow_manager_get_peer_count();
    beacon.channel = espnow_manager_get_channel();
    beacon.rssi = 0;
    beacon.uptime_sec = (uint32_t)((esp_timer_get_time() - s_boot_time) / 1000000);

    /* Add to dup cache */
    add_to_dup_cache(s_local_mac, header.seq_num);

    /* Broadcast */
    esp_err_t ret = send_mesh_packet(NULL, &header,
                                     (const uint8_t *)&beacon, sizeof(beacon));
    if (ret == ESP_OK)
    {
        s_mesh_stats.beacons_sent++;
        ESP_LOGD(TAG, "Beacon sent (seq=%d, uptime=%ds)",
                 header.seq_num, (int)beacon.uptime_sec);
    }
}

/**
 * @brief ESP-NOW receive handler registered with the manager.
 *
 * Parses the mesh header and dispatches to the appropriate handler
 * based on packet type.
 *
 * @param[in] msg Received ESP-NOW message.
 */
static void mesh_espnow_rx_handler(const espnow_rx_msg_t *msg)
{
    if (msg == NULL || msg->data_len < sizeof(mesh_header_t))
    {
        ESP_LOGD(TAG, "RX: too short for mesh header (%d bytes)",
                 msg ? msg->data_len : 0);
        return;
    }

    /* Parse the mesh header */
    const mesh_header_t *header = (const mesh_header_t *)msg->data;

    /* Skip our own packets */
    if (is_local_mac(header->src_addr))
    {
        return;
    }

    /* Check for duplicates */
    if (is_duplicate(header->src_addr, header->seq_num))
    {
        s_mesh_stats.dup_detected++;
        ESP_LOGD(TAG, "Duplicate packet detected (seq=%d)", header->seq_num);
        return;
    }

    /* Record in duplicate cache */
    add_to_dup_cache(header->src_addr, header->seq_num);

    /* Calculate payload pointer and length */
    const uint8_t *payload = msg->data + sizeof(mesh_header_t);
    uint16_t payload_len = msg->data_len - sizeof(mesh_header_t);

    /* Auto-add the immediate sender as a direct route (1 hop) */
    mesh_protocol_route_add(msg->src_mac, msg->src_mac, 1);

    /* Dispatch based on packet type */
    switch (header->type)
    {
    case MESH_PKT_BEACON:
        handle_beacon(msg->src_mac, header, payload, payload_len);
        break;

    case MESH_PKT_DATA:
    case MESH_PKT_SENSOR:
    case MESH_PKT_COMMAND:
        handle_data_packet(msg->src_mac, header, payload, payload_len);
        break;

    case MESH_PKT_RREQ:
        handle_rreq(msg->src_mac, header, payload, payload_len);
        break;

    case MESH_PKT_RREP:
        handle_rrep(msg->src_mac, header, payload, payload_len);
        break;

    default:
        ESP_LOGW(TAG, "Unknown mesh packet type: 0x%02X", header->type);
        break;
    }
}

/**
 * @brief Handle a received beacon packet.
 *
 * Updates the routing table with information about the beacon sender.
 *
 * @param[in] sender_mac   Immediate sender MAC.
 * @param[in] header       Mesh header.
 * @param[in] payload      Beacon payload.
 * @param[in] payload_len  Length of payload.
 */
static void handle_beacon(const uint8_t *sender_mac,
                          const mesh_header_t *header,
                          const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(mesh_beacon_payload_t))
    {
        ESP_LOGW(TAG, "Beacon payload too short");
        return;
    }

    const mesh_beacon_payload_t *beacon = (const mesh_beacon_payload_t *)payload;

    s_mesh_stats.beacons_received++;

    /* Add or update the direct route to the beacon sender */
    mesh_protocol_route_add(beacon->node_mac, sender_mac, 1);

    /* Ensure the sender is registered as an ESP-NOW peer */
    if (!espnow_manager_peer_exists(sender_mac))
    {
        espnow_manager_add_peer(sender_mac, beacon->channel, false, NULL);
    }

    char mac_str[18];
    espnow_mac_to_str(beacon->node_mac, mac_str);
    ESP_LOGD(TAG, "Beacon from %s (peers=%d, ch=%d, uptime=%ds)",
             mac_str, beacon->peer_count, beacon->channel,
             (int)beacon->uptime_sec);
}

/**
 * @brief Handle a received data/sensor/command packet.
 *
 * If the packet is addressed to us, deliver it to the application.
 * If it is addressed to another node and forwarding is enabled,
 * forward it via the routing table.
 *
 * @param[in] sender_mac   Immediate sender MAC.
 * @param[in] header       Mesh header.
 * @param[in] payload      Data payload.
 * @param[in] payload_len  Length of payload.
 */
static void handle_data_packet(const uint8_t *sender_mac,
                               const mesh_header_t *header,
                               const uint8_t *payload, uint16_t payload_len)
{
    /* Is this packet for us? */
    if (is_local_mac(header->dst_addr) || is_broadcast_mac(header->dst_addr))
    {
        s_mesh_stats.packets_received++;

        /* Deliver to application callback */
        if (s_data_callback != NULL)
        {
            mesh_packet_t pkt;
            memcpy(&pkt.header, header, sizeof(mesh_header_t));
            if (payload_len > 0 && payload_len <= MESH_MAX_PAYLOAD_SIZE)
            {
                memcpy(pkt.payload, payload, payload_len);
            }
            pkt.payload_len = payload_len;
            s_data_callback(&pkt);
        }

        char src_str[18];
        espnow_mac_to_str(header->src_addr, src_str);
        ESP_LOGI(TAG, "Data received from %s (%d bytes, hops=%d)",
                 src_str, payload_len, header->hop_count);

        /* If broadcast, continue forwarding */
        if (is_broadcast_mac(header->dst_addr) && s_mesh_config.enable_forwarding)
        {
            forward_packet(sender_mac, header, payload, payload_len);
        }

        return;
    }

    /* Not for us - forward if enabled */
    if (s_mesh_config.enable_forwarding)
    {
        forward_packet(sender_mac, header, payload, payload_len);
    }
    else
    {
        s_mesh_stats.packets_dropped++;
    }
}

/**
 * @brief Handle a Route Request (RREQ) packet.
 *
 * If we are the target, send a Route Reply (RREP).
 * Otherwise, re-broadcast the RREQ.
 *
 * @param[in] sender_mac   Immediate sender MAC.
 * @param[in] header       Mesh header.
 * @param[in] payload      RREQ payload.
 * @param[in] payload_len  Length of payload.
 */
static void handle_rreq(const uint8_t *sender_mac,
                        const mesh_header_t *header,
                        const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(mesh_rreq_payload_t))
    {
        return;
    }

    const mesh_rreq_payload_t *rreq = (const mesh_rreq_payload_t *)payload;

    char origin_str[18], target_str[18];
    espnow_mac_to_str(rreq->origin, origin_str);
    espnow_mac_to_str(rreq->target, target_str);
    ESP_LOGD(TAG, "RREQ: origin=%s target=%s id=%d hops=%d",
             origin_str, target_str, rreq->rreq_id, rreq->hop_count);

    /* Add reverse route to the RREQ originator via the sender */
    mesh_protocol_route_add(rreq->origin, sender_mac, rreq->hop_count + 1);

    /* Are we the target? */
    if (is_local_mac(rreq->target))
    {
        /* Send RREP back towards the originator */
        ESP_LOGI(TAG, "We are the RREQ target - sending RREP");

        mesh_rrep_payload_t rrep = {0};
        memcpy(rrep.origin, rreq->origin, ESPNOW_MAC_ADDR_LEN);
        memcpy(rrep.target, s_local_mac, ESPNOW_MAC_ADDR_LEN);
        rrep.hop_count = 0;
        rrep.rreq_id = rreq->rreq_id;

        mesh_header_t rrep_header = {0};
        rrep_header.type = MESH_PKT_RREP;
        memcpy(rrep_header.src_addr, s_local_mac, ESPNOW_MAC_ADDR_LEN);
        memcpy(rrep_header.dst_addr, rreq->origin, ESPNOW_MAC_ADDR_LEN);
        rrep_header.seq_num = mesh_protocol_next_seq();
        rrep_header.ttl = s_mesh_config.default_ttl;
        rrep_header.hop_count = 0;

        add_to_dup_cache(s_local_mac, rrep_header.seq_num);

        /* Send RREP back via the reverse route (sender) */
        send_mesh_packet(sender_mac, &rrep_header,
                         (const uint8_t *)&rrep, sizeof(rrep));
        s_mesh_stats.rrep_sent++;
        return;
    }

    /* Not the target - forward the RREQ if TTL allows */
    if (header->ttl > 1)
    {
        mesh_header_t fwd_header;
        memcpy(&fwd_header, header, sizeof(mesh_header_t));
        fwd_header.ttl--;
        fwd_header.hop_count++;

        mesh_rreq_payload_t fwd_rreq;
        memcpy(&fwd_rreq, rreq, sizeof(mesh_rreq_payload_t));
        fwd_rreq.hop_count++;

        /* Re-broadcast */
        send_mesh_packet(NULL, &fwd_header,
                         (const uint8_t *)&fwd_rreq, sizeof(fwd_rreq));
        s_mesh_stats.packets_forwarded++;
    }
    else
    {
        s_mesh_stats.packets_dropped++;
    }
}

/**
 * @brief Handle a Route Reply (RREP) packet.
 *
 * Install the forward route and forward the RREP towards the originator.
 *
 * @param[in] sender_mac   Immediate sender MAC.
 * @param[in] header       Mesh header.
 * @param[in] payload      RREP payload.
 * @param[in] payload_len  Length of payload.
 */
static void handle_rrep(const uint8_t *sender_mac,
                        const mesh_header_t *header,
                        const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len < sizeof(mesh_rrep_payload_t))
    {
        return;
    }

    const mesh_rrep_payload_t *rrep = (const mesh_rrep_payload_t *)payload;

    char origin_str[18], target_str[18];
    espnow_mac_to_str(rrep->origin, origin_str);
    espnow_mac_to_str(rrep->target, target_str);
    ESP_LOGD(TAG, "RREP: origin=%s target=%s hops=%d",
             origin_str, target_str, rrep->hop_count);

    /* Install forward route to the target via the sender */
    mesh_protocol_route_add(rrep->target, sender_mac, rrep->hop_count + 1);

    /* Are we the RREQ originator? */
    if (is_local_mac(rrep->origin))
    {
        ESP_LOGI(TAG, "RREP received - route to %s established (%d hops)",
                 target_str, rrep->hop_count + 1);
        return;
    }

    /* Forward RREP towards the originator */
    if (header->ttl > 1)
    {
        mesh_route_entry_t route;
        if (mesh_protocol_route_lookup(rrep->origin, &route))
        {
            mesh_header_t fwd_header;
            memcpy(&fwd_header, header, sizeof(mesh_header_t));
            fwd_header.ttl--;
            fwd_header.hop_count++;

            mesh_rrep_payload_t fwd_rrep;
            memcpy(&fwd_rrep, rrep, sizeof(mesh_rrep_payload_t));
            fwd_rrep.hop_count++;

            send_mesh_packet(route.next_hop, &fwd_header,
                             (const uint8_t *)&fwd_rrep, sizeof(fwd_rrep));
            s_mesh_stats.packets_forwarded++;
        }
        else
        {
            ESP_LOGW(TAG, "No reverse route for RREP forwarding");
            s_mesh_stats.route_errors++;
        }
    }
}

/**
 * @brief Forward a mesh packet to the next hop.
 *
 * Decrements TTL and increments hop count. Drops the packet
 * if TTL reaches zero.
 *
 * @param[in] sender_mac   MAC of the node that sent us this packet.
 * @param[in] header       Original mesh header.
 * @param[in] payload      Payload data.
 * @param[in] payload_len  Length of payload.
 * @return ESP_OK if forwarded, error code otherwise.
 */
static esp_err_t forward_packet(const uint8_t *sender_mac,
                                const mesh_header_t *header,
                                const uint8_t *payload, uint16_t payload_len)
{
    if (header->ttl <= 1)
    {
        s_mesh_stats.packets_dropped++;
        ESP_LOGD(TAG, "Dropping packet (TTL expired)");
        return ESP_ERR_TIMEOUT;
    }

    /* Build forwarded header */
    mesh_header_t fwd_header;
    memcpy(&fwd_header, header, sizeof(mesh_header_t));
    fwd_header.ttl--;
    fwd_header.hop_count++;

    esp_err_t ret;

    if (is_broadcast_mac(header->dst_addr))
    {
        /* Flood: broadcast to all except the sender */
        ret = send_mesh_packet(NULL, &fwd_header, payload, payload_len);
    }
    else
    {
        /* Unicast: look up the route */
        mesh_route_entry_t route;
        if (mesh_protocol_route_lookup(header->dst_addr, &route))
        {
            ret = send_mesh_packet(route.next_hop, &fwd_header,
                                   payload, payload_len);
        }
        else
        {
            ESP_LOGW(TAG, "No route for forwarding");
            s_mesh_stats.route_errors++;
            return ESP_ERR_NOT_FOUND;
        }
    }

    if (ret == ESP_OK)
    {
        s_mesh_stats.packets_forwarded++;
        ESP_LOGD(TAG, "Packet forwarded (ttl=%d, hops=%d)",
                 fwd_header.ttl, fwd_header.hop_count);
    }

    return ret;
}

/**
 * @brief Serialize and send a mesh packet via ESP-NOW.
 *
 * Assembles the header and payload into a contiguous buffer
 * and sends it via broadcast or unicast.
 *
 * @param[in] next_hop_mac Next hop MAC (NULL for broadcast).
 * @param[in] header       Mesh header.
 * @param[in] payload      Payload data.
 * @param[in] payload_len  Length of payload.
 * @return ESP_OK on success.
 */
static esp_err_t send_mesh_packet(const uint8_t *next_hop_mac,
                                  const mesh_header_t *header,
                                  const uint8_t *payload, uint16_t payload_len)
{
    /* Assemble the full frame: header + payload */
    uint8_t frame[ESPNOW_MAX_PAYLOAD_SIZE];
    uint16_t frame_len = sizeof(mesh_header_t) + payload_len;

    if (frame_len > ESPNOW_MAX_PAYLOAD_SIZE)
    {
        ESP_LOGE(TAG, "Frame too large: %d bytes", frame_len);
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(frame, header, sizeof(mesh_header_t));
    if (payload != NULL && payload_len > 0)
    {
        memcpy(frame + sizeof(mesh_header_t), payload, payload_len);
    }

    if (next_hop_mac == NULL)
    {
        /* Broadcast */
        return espnow_manager_send_broadcast(frame, frame_len);
    }
    else
    {
        /* Unicast */
        return espnow_manager_send_unicast(next_hop_mac, frame, frame_len);
    }
}

/* -------------------------------------------------------------------------
 * Duplicate Detection
 * ---------------------------------------------------------------------- */

/**
 * @brief Check whether a packet is a duplicate.
 *
 * @param[in] src_addr Source MAC address.
 * @param[in] seq_num  Sequence number.
 * @return true if the packet is a duplicate.
 */
static bool is_duplicate(const uint8_t *src_addr, uint16_t seq_num)
{
    xSemaphoreTake(s_dup_mutex, portMAX_DELAY);

    for (int i = 0; i < MESH_DUP_CACHE_SIZE; i++)
    {
        if (s_dup_cache[i].valid &&
            s_dup_cache[i].seq_num == seq_num &&
            memcmp(s_dup_cache[i].src_addr, src_addr,
                   ESPNOW_MAC_ADDR_LEN) == 0)
        {
            xSemaphoreGive(s_dup_mutex);
            return true;
        }
    }

    xSemaphoreGive(s_dup_mutex);
    return false;
}

/**
 * @brief Add a packet to the duplicate detection cache.
 *
 * Uses a circular replacement strategy when the cache is full.
 *
 * @param[in] src_addr Source MAC address.
 * @param[in] seq_num  Sequence number.
 */
static void add_to_dup_cache(const uint8_t *src_addr, uint16_t seq_num)
{
    xSemaphoreTake(s_dup_mutex, portMAX_DELAY);

    /* Find a free slot or the oldest entry */
    int oldest_idx = 0;
    int64_t oldest_time = INT64_MAX;
    int free_idx = -1;

    for (int i = 0; i < MESH_DUP_CACHE_SIZE; i++)
    {
        if (!s_dup_cache[i].valid)
        {
            free_idx = i;
            break;
        }
        if (s_dup_cache[i].timestamp < oldest_time)
        {
            oldest_time = s_dup_cache[i].timestamp;
            oldest_idx = i;
        }
    }

    int idx = (free_idx >= 0) ? free_idx : oldest_idx;

    memcpy(s_dup_cache[idx].src_addr, src_addr, ESPNOW_MAC_ADDR_LEN);
    s_dup_cache[idx].seq_num = seq_num;
    s_dup_cache[idx].timestamp = esp_timer_get_time();
    s_dup_cache[idx].valid = true;

    xSemaphoreGive(s_dup_mutex);
}

/* -------------------------------------------------------------------------
 * Route Table Helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Find a route slot by destination MAC.
 *
 * @param[in] dest_mac Destination MAC address.
 * @return Index (>= 0) if found, -1 otherwise.
 */
static int find_route_slot(const uint8_t *dest_mac)
{
    for (int i = 0; i < MESH_MAX_ROUTES; i++)
    {
        if (s_route_table[i].valid &&
            memcmp(s_route_table[i].dest_addr, dest_mac,
                   ESPNOW_MAC_ADDR_LEN) == 0)
        {
            return i;
        }
    }
    return -1;
}

/**
 * @brief Find a free slot in the routing table.
 *
 * @return Index (>= 0) if found, -1 if full.
 */
static int find_free_route_slot(void)
{
    for (int i = 0; i < MESH_MAX_ROUTES; i++)
    {
        if (!s_route_table[i].valid)
        {
            return i;
        }
    }
    return -1;
}

/* -------------------------------------------------------------------------
 * MAC Address Helpers
 * ---------------------------------------------------------------------- */

/**
 * @brief Check if a MAC address is the local device MAC.
 *
 * @param[in] mac 6-byte MAC address.
 * @return true if it matches the local MAC.
 */
static bool is_local_mac(const uint8_t *mac)
{
    return (memcmp(mac, s_local_mac, ESPNOW_MAC_ADDR_LEN) == 0);
}

/**
 * @brief Check if a MAC address is the broadcast address.
 *
 * @param[in] mac 6-byte MAC address.
 * @return true if it is FF:FF:FF:FF:FF:FF.
 */
static bool is_broadcast_mac(const uint8_t *mac)
{
    return (memcmp(mac, BROADCAST_MAC, ESPNOW_MAC_ADDR_LEN) == 0);
}
