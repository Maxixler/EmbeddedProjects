/**
 * @file main.c
 * @brief Main Application - ESP-NOW Mesh Network Demo.
 *
 * Demonstrates ESP-NOW mesh networking on ESP32 with:
 *   - ESP-NOW and mesh protocol initialization
 *   - Beacon sending and message processing tasks
 *   - Sensor data exchange through the mesh
 *   - LED indication for TX/RX activity
 *   - Serial command interface for test messages
 *
 * @version 1.0.0
 * @date 2026-03-16
 */

/* -------------------------------------------------------------------------
 * Includes
 * ---------------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/temperature_sensor.h"

#include "espnow_manager.h"
#include "mesh_protocol.h"

/* -------------------------------------------------------------------------
 * Configuration
 * ---------------------------------------------------------------------- */

/** @brief Log tag for this module. */
static const char *TAG = "MESH_APP";

/** @brief GPIO pin for the activity LED. */
#define LED_GPIO GPIO_NUM_2

/** @brief LED blink duration in milliseconds. */
#define LED_BLINK_DURATION_MS (50)

/** @brief WiFi channel for the mesh network. */
#define MESH_WIFI_CHANNEL (1)

/** @brief Sensor data broadcast interval in milliseconds. */
#define SENSOR_SEND_INTERVAL_MS (10000)

/** @brief Serial command buffer size. */
#define CMD_BUF_SIZE (256)

/** @brief UART number for serial commands. */
#define CMD_UART_NUM UART_NUM_0

/** @brief Stack size for the serial command task. */
#define CMD_TASK_STACK_SIZE (4096)

/** @brief Stack size for the sensor task. */
#define SENSOR_TASK_STACK_SIZE (4096)

/** @brief Stack size for the LED task. */
#define LED_TASK_STACK_SIZE (2048)

/* -------------------------------------------------------------------------
 * Sensor Data Structure
 * ---------------------------------------------------------------------- */

/**
 * @brief Sensor data payload exchanged between mesh nodes.
 */
typedef struct __attribute__((packed))
{
    uint8_t node_mac[ESPNOW_MAC_ADDR_LEN]; /**< Source node MAC.          */
    float temperature;                     /**< Temperature in Celsius.   */
    float humidity;                        /**< Humidity percentage.       */
    uint32_t uptime_sec;                   /**< Node uptime (seconds).    */
    uint16_t battery_mv;                   /**< Battery voltage (mV).     */
    uint32_t free_heap;                    /**< Free heap memory (bytes). */
    uint16_t msg_count;                    /**< Message counter.          */
} sensor_data_t;

/* -------------------------------------------------------------------------
 * Private Data
 * ---------------------------------------------------------------------- */

/** @brief LED blink notification queue. */
static QueueHandle_t s_led_queue = NULL;

/** @brief Sensor message counter. */
static uint16_t s_sensor_msg_count = 0;

/** @brief Boot timestamp. */
static int64_t s_boot_time = 0;

/** @brief Temperature sensor handle. */
static temperature_sensor_handle_t s_temp_sensor = NULL;

/* -------------------------------------------------------------------------
 * Private Function Prototypes
 * ---------------------------------------------------------------------- */

static void led_init(void);
static void led_blink(void);
static void led_task(void *arg);
static void sensor_data_task(void *arg);
static void serial_cmd_task(void *arg);
static void mesh_data_handler(const mesh_packet_t *packet);
static void tx_status_handler(const espnow_tx_event_t *event);
static void process_command(const char *cmd);
static void print_help(void);
static void print_stats(void);
static void print_node_info(void);
static float read_temperature(void);
static bool parse_mac(const char *str, uint8_t *mac);

/* -------------------------------------------------------------------------
 * LED Control
 * ---------------------------------------------------------------------- */

/**
 * @brief Initialize the activity LED GPIO.
 */
static void led_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO, 0);
}

/**
 * @brief Request an LED blink (non-blocking).
 */
static void led_blink(void)
{
    if (s_led_queue != NULL)
    {
        uint8_t val = 1;
        xQueueSend(s_led_queue, &val, 0);
    }
}

/**
 * @brief LED blink task - toggles the LED briefly on each notification.
 *
 * @param[in] arg Unused.
 */
static void led_task(void *arg)
{
    uint8_t val;

    while (true)
    {
        if (xQueueReceive(s_led_queue, &val, portMAX_DELAY) == pdTRUE)
        {
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_DURATION_MS));
            gpio_set_level(LED_GPIO, 0);
        }
    }

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Temperature Sensor
 * ---------------------------------------------------------------------- */

/**
 * @brief Read the internal temperature sensor.
 *
 * @return Temperature in Celsius (or a simulated value if unavailable).
 */
static float read_temperature(void)
{
    float temp = 0.0f;

    if (s_temp_sensor != NULL)
    {
        if (temperature_sensor_get_celsius(s_temp_sensor, &temp) == ESP_OK)
        {
            return temp;
        }
    }

    /* Fallback: simulate a temperature value */
    temp = 20.0f + (float)(esp_random() % 150) / 10.0f; /* 20.0 - 35.0 */
    return temp;
}

/* -------------------------------------------------------------------------
 * Mesh Data Handler
 * ---------------------------------------------------------------------- */

/**
 * @brief Application callback for received mesh data packets.
 *
 * Processes sensor data and command packets received via the mesh.
 *
 * @param[in] packet Pointer to the received mesh packet.
 */
static void mesh_data_handler(const mesh_packet_t *packet)
{
    if (packet == NULL)
    {
        return;
    }

    /* Blink LED on receive */
    led_blink();

    char src_str[18];
    espnow_mac_to_str(packet->header.src_addr, src_str);

    switch (packet->header.type)
    {
    case MESH_PKT_SENSOR:
    {
        if (packet->payload_len >= sizeof(sensor_data_t))
        {
            const sensor_data_t *sensor =
                (const sensor_data_t *)packet->payload;

            ESP_LOGI(TAG, "--- Sensor Data from %s ---", src_str);
            ESP_LOGI(TAG, "  Temperature : %.1f C", sensor->temperature);
            ESP_LOGI(TAG, "  Humidity    : %.1f %%", sensor->humidity);
            ESP_LOGI(TAG, "  Uptime      : %d s", (int)sensor->uptime_sec);
            ESP_LOGI(TAG, "  Battery     : %d mV", sensor->battery_mv);
            ESP_LOGI(TAG, "  Free Heap   : %d bytes", (int)sensor->free_heap);
            ESP_LOGI(TAG, "  Msg Count   : %d", sensor->msg_count);
            ESP_LOGI(TAG, "  Hops        : %d", packet->header.hop_count);
        }
        break;
    }

    case MESH_PKT_DATA:
    {
        /* Print raw data as string */
        char buf[MESH_MAX_PAYLOAD_SIZE + 1];
        int len = (packet->payload_len < MESH_MAX_PAYLOAD_SIZE) ? packet->payload_len : MESH_MAX_PAYLOAD_SIZE;
        memcpy(buf, packet->payload, len);
        buf[len] = '\0';
        ESP_LOGI(TAG, "Data from %s (hops=%d): %s",
                 src_str, packet->header.hop_count, buf);
        break;
    }

    case MESH_PKT_COMMAND:
    {
        char buf[MESH_MAX_PAYLOAD_SIZE + 1];
        int len = (packet->payload_len < MESH_MAX_PAYLOAD_SIZE) ? packet->payload_len : MESH_MAX_PAYLOAD_SIZE;
        memcpy(buf, packet->payload, len);
        buf[len] = '\0';
        ESP_LOGI(TAG, "Command from %s: %s", src_str, buf);
        break;
    }

    default:
        ESP_LOGD(TAG, "Received packet type 0x%02X from %s",
                 packet->header.type, src_str);
        break;
    }
}

/**
 * @brief TX status callback - blinks LED on successful send.
 *
 * @param[in] event Pointer to the TX event.
 */
static void tx_status_handler(const espnow_tx_event_t *event)
{
    if (event == NULL)
    {
        return;
    }

    if (event->status == ESPNOW_SEND_OK)
    {
        led_blink();
    }
    else
    {
        char mac_str[18];
        espnow_mac_to_str(event->dest_mac, mac_str);
        ESP_LOGW(TAG, "TX failed to %s", mac_str);
    }
}

/* -------------------------------------------------------------------------
 * Sensor Data Task
 * ---------------------------------------------------------------------- */

/**
 * @brief Periodically broadcasts sensor data through the mesh.
 *
 * Reads the temperature sensor, builds a sensor data payload, and
 * broadcasts it to all mesh nodes.
 *
 * @param[in] arg Unused.
 */
static void sensor_data_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor data task started (interval=%dms)",
             SENSOR_SEND_INTERVAL_MS);

    /* Wait for mesh to discover neighbours */
    vTaskDelay(pdMS_TO_TICKS(SENSOR_SEND_INTERVAL_MS));

    while (true)
    {
        /* Build sensor data payload */
        sensor_data_t data = {0};
        espnow_manager_get_local_mac(data.node_mac);
        data.temperature = read_temperature();
        data.humidity = 40.0f + (float)(esp_random() % 400) / 10.0f;
        data.uptime_sec = (uint32_t)((esp_timer_get_time() - s_boot_time) / 1000000);
        data.battery_mv = 3000 + (esp_random() % 700); /* 3.0V - 3.7V */
        data.free_heap = (uint32_t)esp_get_free_heap_size();
        data.msg_count = s_sensor_msg_count++;

        /* Broadcast sensor data to all mesh nodes */
        const uint8_t broadcast[ESPNOW_MAC_ADDR_LEN] = ESPNOW_BROADCAST_ADDR;
        esp_err_t ret = mesh_protocol_send(broadcast,
                                           (const uint8_t *)&data,
                                           sizeof(data),
                                           MESH_PKT_SENSOR);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Sensor data broadcast: temp=%.1fC, hum=%.1f%%, heap=%d",
                     data.temperature, data.humidity, (int)data.free_heap);
        }
        else
        {
            ESP_LOGW(TAG, "Failed to broadcast sensor data: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_SEND_INTERVAL_MS));
    }

    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------
 * Serial Command Interface
 * ---------------------------------------------------------------------- */

/**
 * @brief Serial command processing task.
 *
 * Reads commands from UART and processes them. Supported commands:
 *   - help       : Print command list
 *   - info       : Print node information
 *   - routes     : Print routing table
 *   - stats      : Print mesh statistics
 *   - peers      : List ESP-NOW peers
 *   - send <mac> <message> : Send a message to a specific node
 *   - broadcast <message>  : Broadcast a message to all nodes
 *   - rreq <mac> : Send a route request
 *
 * @param[in] arg Unused.
 */
static void serial_cmd_task(void *arg)
{
    char cmd_buf[CMD_BUF_SIZE];
    int cmd_pos = 0;

    ESP_LOGI(TAG, "Serial command interface ready. Type 'help' for commands.");

    while (true)
    {
        /* Read one byte at a time */
        uint8_t byte;
        int len = uart_read_bytes(CMD_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));

        if (len <= 0)
        {
            continue;
        }

        /* Echo the character */
        uart_write_bytes(CMD_UART_NUM, (const char *)&byte, 1);

        if (byte == '\n' || byte == '\r')
        {
            if (cmd_pos > 0)
            {
                cmd_buf[cmd_pos] = '\0';
                printf("\n");
                process_command(cmd_buf);
                cmd_pos = 0;
            }
            printf("\nmesh> ");
            fflush(stdout);
        }
        else if (byte == 0x7F || byte == '\b')
        {
            /* Backspace */
            if (cmd_pos > 0)
            {
                cmd_pos--;
                printf("\b \b");
                fflush(stdout);
            }
        }
        else
        {
            if (cmd_pos < CMD_BUF_SIZE - 1)
            {
                cmd_buf[cmd_pos++] = (char)byte;
            }
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief Process a serial command string.
 *
 * @param[in] cmd Null-terminated command string.
 */
static void process_command(const char *cmd)
{
    if (cmd == NULL || strlen(cmd) == 0)
    {
        return;
    }

    /* Trim leading spaces */
    while (*cmd == ' ')
        cmd++;

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
    {
        print_help();
    }
    else if (strcmp(cmd, "info") == 0)
    {
        print_node_info();
    }
    else if (strcmp(cmd, "routes") == 0 || strcmp(cmd, "rt") == 0)
    {
        mesh_protocol_print_routes();
    }
    else if (strcmp(cmd, "stats") == 0)
    {
        print_stats();
    }
    else if (strcmp(cmd, "peers") == 0)
    {
        int count = espnow_manager_get_peer_count();
        printf("\nRegistered Peers (%d):\n", count);
        printf("+-----+-------------------+---------+------+\n");
        printf("| Idx | MAC Address       | Channel | Enc  |\n");
        printf("+-----+-------------------+---------+------+\n");

        for (int i = 0; i < count; i++)
        {
            espnow_peer_info_t info;
            if (espnow_manager_get_peer_info(i, &info) == ESP_OK)
            {
                char mac_str[18];
                espnow_mac_to_str(info.mac_addr, mac_str);
                printf("| %3d | %s | %7d | %4s |\n",
                       i, mac_str, info.channel,
                       info.encrypted ? "yes" : "no");
            }
        }
        printf("+-----+-------------------+---------+------+\n\n");
    }
    else if (strncmp(cmd, "send ", 5) == 0)
    {
        /* send XX:XX:XX:XX:XX:XX message */
        const char *args = cmd + 5;
        uint8_t dest_mac[ESPNOW_MAC_ADDR_LEN];

        if (strlen(args) < 18)
        {
            printf("Usage: send XX:XX:XX:XX:XX:XX <message>\n");
            return;
        }

        char mac_str[18];
        strncpy(mac_str, args, 17);
        mac_str[17] = '\0';

        if (!parse_mac(mac_str, dest_mac))
        {
            printf("Invalid MAC address format. Use XX:XX:XX:XX:XX:XX\n");
            return;
        }

        const char *msg = args + 18;
        if (strlen(msg) == 0)
        {
            msg = "Hello from mesh!";
        }

        esp_err_t ret = mesh_protocol_send(dest_mac,
                                           (const uint8_t *)msg,
                                           strlen(msg),
                                           MESH_PKT_DATA);
        if (ret == ESP_OK)
        {
            printf("Message sent to %s: %s\n", mac_str, msg);
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            printf("No route to %s. Route discovery initiated.\n", mac_str);
        }
        else
        {
            printf("Send failed: %s\n", esp_err_to_name(ret));
        }
    }
    else if (strncmp(cmd, "broadcast ", 10) == 0)
    {
        const char *msg = cmd + 10;
        if (strlen(msg) == 0)
        {
            msg = "Broadcast test!";
        }

        const uint8_t broadcast[ESPNOW_MAC_ADDR_LEN] = ESPNOW_BROADCAST_ADDR;
        esp_err_t ret = mesh_protocol_send(broadcast,
                                           (const uint8_t *)msg,
                                           strlen(msg),
                                           MESH_PKT_DATA);
        if (ret == ESP_OK)
        {
            printf("Broadcast sent: %s\n", msg);
        }
        else
        {
            printf("Broadcast failed: %s\n", esp_err_to_name(ret));
        }
    }
    else if (strncmp(cmd, "rreq ", 5) == 0)
    {
        const char *mac_arg = cmd + 5;
        uint8_t dest_mac[ESPNOW_MAC_ADDR_LEN];

        if (!parse_mac(mac_arg, dest_mac))
        {
            printf("Usage: rreq XX:XX:XX:XX:XX:XX\n");
            return;
        }

        esp_err_t ret = mesh_protocol_route_request(dest_mac);
        if (ret == ESP_OK)
        {
            printf("Route request sent for %s\n", mac_arg);
        }
        else
        {
            printf("RREQ failed: %s\n", esp_err_to_name(ret));
        }
    }
    else if (strcmp(cmd, "reset") == 0)
    {
        printf("Resetting statistics...\n");
        espnow_manager_reset_stats();
        mesh_protocol_reset_stats();
        printf("Done.\n");
    }
    else if (strcmp(cmd, "reboot") == 0)
    {
        printf("Rebooting...\n");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }
    else
    {
        printf("Unknown command: '%s'. Type 'help' for available commands.\n",
               cmd);
    }
}

/**
 * @brief Print the help / command list.
 */
static void print_help(void)
{
    printf("\n");
    printf("=== ESP-NOW Mesh Network - Serial Commands ===\n");
    printf("\n");
    printf("  help, ?                       - Show this help\n");
    printf("  info                          - Show node information\n");
    printf("  routes, rt                    - Show routing table\n");
    printf("  stats                         - Show mesh statistics\n");
    printf("  peers                         - List ESP-NOW peers\n");
    printf("  send <MAC> <message>          - Send unicast message\n");
    printf("  broadcast <message>           - Broadcast message\n");
    printf("  rreq <MAC>                    - Send route request\n");
    printf("  reset                         - Reset statistics\n");
    printf("  reboot                        - Reboot the device\n");
    printf("\n");
    printf("  MAC format: XX:XX:XX:XX:XX:XX\n");
    printf("\n");
}

/**
 * @brief Print mesh statistics to the console.
 */
static void print_stats(void)
{
    mesh_stats_t mesh_st;
    espnow_stats_t espnow_st;

    mesh_protocol_get_stats(&mesh_st);
    espnow_manager_get_stats(&espnow_st);

    printf("\n");
    printf("=== Mesh Protocol Statistics ===\n");
    printf("  Packets Sent       : %lu\n", (unsigned long)mesh_st.packets_sent);
    printf("  Packets Received   : %lu\n", (unsigned long)mesh_st.packets_received);
    printf("  Packets Forwarded  : %lu\n", (unsigned long)mesh_st.packets_forwarded);
    printf("  Packets Dropped    : %lu\n", (unsigned long)mesh_st.packets_dropped);
    printf("  Beacons Sent       : %lu\n", (unsigned long)mesh_st.beacons_sent);
    printf("  Beacons Received   : %lu\n", (unsigned long)mesh_st.beacons_received);
    printf("  RREQ Sent          : %lu\n", (unsigned long)mesh_st.rreq_sent);
    printf("  RREP Sent          : %lu\n", (unsigned long)mesh_st.rrep_sent);
    printf("  Route Errors       : %lu\n", (unsigned long)mesh_st.route_errors);
    printf("  Duplicates Detected: %lu\n", (unsigned long)mesh_st.dup_detected);
    printf("\n");
    printf("=== ESP-NOW Statistics ===\n");
    printf("  TX Success         : %lu\n", (unsigned long)espnow_st.tx_success);
    printf("  TX Fail            : %lu\n", (unsigned long)espnow_st.tx_fail);
    printf("  RX Count           : %lu\n", (unsigned long)espnow_st.rx_count);
    printf("  RX Dropped         : %lu\n", (unsigned long)espnow_st.rx_dropped);
    printf("\n");
    printf("  Active Routes      : %d\n", mesh_protocol_route_count());
    printf("  Registered Peers   : %d\n", espnow_manager_get_peer_count());
    printf("  Free Heap          : %lu bytes\n",
           (unsigned long)esp_get_free_heap_size());
    printf("\n");
}

/**
 * @brief Print node information.
 */
static void print_node_info(void)
{
    uint8_t mac[ESPNOW_MAC_ADDR_LEN];
    char mac_str[18];

    espnow_manager_get_local_mac(mac);
    espnow_mac_to_str(mac, mac_str);

    uint32_t uptime = (uint32_t)((esp_timer_get_time() - s_boot_time) / 1000000);
    const char *state_str;

    switch (mesh_protocol_get_state())
    {
    case MESH_STATE_IDLE:
        state_str = "IDLE";
        break;
    case MESH_STATE_DISCOVERING:
        state_str = "DISCOVERING";
        break;
    case MESH_STATE_ACTIVE:
        state_str = "ACTIVE";
        break;
    case MESH_STATE_ERROR:
        state_str = "ERROR";
        break;
    default:
        state_str = "UNKNOWN";
        break;
    }

    printf("\n");
    printf("=== Node Information ===\n");
    printf("  MAC Address  : %s\n", mac_str);
    printf("  WiFi Channel : %d\n", espnow_manager_get_channel());
    printf("  Mesh State   : %s\n", state_str);
    printf("  Uptime       : %lu seconds\n", (unsigned long)uptime);
    printf("  Free Heap    : %lu bytes\n",
           (unsigned long)esp_get_free_heap_size());
    printf("  Temperature  : %.1f C\n", read_temperature());
    printf("  Routes       : %d\n", mesh_protocol_route_count());
    printf("  Peers        : %d\n", espnow_manager_get_peer_count());
    printf("\n");
}

/**
 * @brief Parse a MAC address string "XX:XX:XX:XX:XX:XX" into bytes.
 *
 * @param[in]  str MAC address string.
 * @param[out] mac 6-byte output buffer.
 * @return true if parsing succeeded.
 */
static bool parse_mac(const char *str, uint8_t *mac)
{
    if (str == NULL || mac == NULL || strlen(str) < 17)
    {
        return false;
    }

    unsigned int values[6];
    int count = sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
                       &values[0], &values[1], &values[2],
                       &values[3], &values[4], &values[5]);

    if (count != 6)
    {
        return false;
    }

    for (int i = 0; i < 6; i++)
    {
        mac[i] = (uint8_t)values[i];
    }

    return true;
}

/* -------------------------------------------------------------------------
 * Application Entry Point
 * ---------------------------------------------------------------------- */

/**
 * @brief Main application entry point.
 *
 * Initializes the ESP-NOW manager and mesh protocol, creates
 * application tasks for sensor data broadcasting, LED activity
 * indication, and serial command processing.
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  ESP-NOW Mesh Network - Starting...");
    ESP_LOGI(TAG, "============================================");

    s_boot_time = esp_timer_get_time();

    /* ---- Initialize LED ---- */
    led_init();
    s_led_queue = xQueueCreate(16, sizeof(uint8_t));

    /* ---- Initialize Temperature Sensor ---- */
    temperature_sensor_config_t temp_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t ret = temperature_sensor_install(&temp_cfg, &s_temp_sensor);
    if (ret == ESP_OK)
    {
        temperature_sensor_enable(s_temp_sensor);
        ESP_LOGI(TAG, "Internal temperature sensor initialized");
    }
    else
    {
        ESP_LOGW(TAG, "Temperature sensor init failed (using simulated values)");
        s_temp_sensor = NULL;
    }

    /* ---- Initialize ESP-NOW Manager ---- */
    espnow_config_t espnow_cfg = {
        .channel = MESH_WIFI_CHANNEL,
        .pmk = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10},
        .enable_encryption = false, /* Set to true for encrypted communication */
        .rx_callback = NULL,        /* Will be set by mesh protocol */
        .tx_callback = tx_status_handler,
    };

    ret = espnow_manager_init(&espnow_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP-NOW Manager init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* ---- Initialize Mesh Protocol ---- */
    mesh_config_t mesh_cfg = {
        .default_ttl = MESH_DEFAULT_TTL,
        .beacon_interval_ms = MESH_BEACON_INTERVAL_MS,
        .route_timeout_ms = MESH_ROUTE_TIMEOUT_MS,
        .data_callback = mesh_data_handler,
        .enable_forwarding = true,
    };

    ret = mesh_protocol_init(&mesh_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Mesh protocol init failed: %s", esp_err_to_name(ret));
        return;
    }

    /* ---- Print Node Info ---- */
    uint8_t local_mac[ESPNOW_MAC_ADDR_LEN];
    char mac_str[18];
    espnow_manager_get_local_mac(local_mac);
    espnow_mac_to_str(local_mac, mac_str);

    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  Node MAC    : %s", mac_str);
    ESP_LOGI(TAG, "  WiFi Channel: %d", MESH_WIFI_CHANNEL);
    ESP_LOGI(TAG, "  Mesh TTL    : %d", MESH_DEFAULT_TTL);
    ESP_LOGI(TAG, "  Beacon Intv : %d ms", MESH_BEACON_INTERVAL_MS);
    ESP_LOGI(TAG, "============================================");

    /* ---- Create Application Tasks ---- */

    /* LED blink task */
    xTaskCreate(led_task,
                "led_task",
                LED_TASK_STACK_SIZE,
                NULL,
                2,
                NULL);

    /* Sensor data broadcasting task */
    xTaskCreate(sensor_data_task,
                "sensor_task",
                SENSOR_TASK_STACK_SIZE,
                NULL,
                3,
                NULL);

    /* Serial command interface task */
    xTaskCreate(serial_cmd_task,
                "cmd_task",
                CMD_TASK_STACK_SIZE,
                NULL,
                1,
                NULL);

    ESP_LOGI(TAG, "All tasks started. Type 'help' for commands.");
    printf("\nmesh> ");
    fflush(stdout);
}
