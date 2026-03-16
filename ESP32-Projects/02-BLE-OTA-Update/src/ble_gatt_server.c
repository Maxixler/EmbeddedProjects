/**
 * @file    ble_gatt_server.c
 * @brief   BLE GATT server implementation using NimBLE stack on ESP32.
 *
 * @details Implements a fully functional BLE GATT server with two custom
 *          services: a Sensor Data service for exposing sensor readings and
 *          an OTA Update service for receiving firmware updates over BLE.
 *
 *          Implementation overview:
 *          1. NimBLE host stack is initialized via nimble_port_init().
 *          2. GAP and GATT are configured with device name, appearance,
 *             preferred connection parameters, and service definitions.
 *          3. BLE advertising uses connectable undirected mode with the
 *             device name included in the advertising data.
 *          4. GAP event handling manages connect/disconnect/MTU exchange.
 *          5. GATT access callbacks route read/write operations to the
 *             appropriate handler based on characteristic UUID.
 *
 *          Data flow for OTA update:
 *          Client (nRF Connect)              ESP32 GATT Server
 *              |                                    |
 *              |-- Write OTA Control (0x01=START) ->|  -> ota_manager_start()
 *              |-- Write OTA Data (chunk 1) ------->|  -> ota_manager_write()
 *              |-- Write OTA Data (chunk 2) ------->|  -> ota_manager_write()
 *              |          ...                       |
 *              |-- Write OTA Data (chunk N) ------->|  -> ota_manager_write()
 *              |-- Write OTA Control (0x03=COMMIT)->|  -> ota_manager_finish()
 *              |<- Notify OTA Control (status) -----|     ota_manager_commit()
 *              |                                    |  -> esp_restart()
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "ble_gatt_server.h"

#include <string.h>
#include <assert.h>

#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

/** Logging tag for ESP_LOGx macros. */
static const char *TAG = "ble_gatt";

/** Current BLE connection handle. BLE_HS_CONN_HANDLE_NONE when disconnected. */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/** Current connection state. */
static ble_conn_state_t s_conn_state = BLE_STATE_IDLE;

/** Current negotiated MTU size. */
static uint16_t s_current_mtu = 23; /* BLE default MTU */

/** GATT attribute handles assigned by NimBLE during service registration. */
static ble_gatt_handles_t s_gatt_handles = {0};

/** Application-registered callbacks. */
static ble_gatt_callbacks_t s_callbacks = {0};

/** Sensor data buffer for read access. */
static uint8_t s_sensor_data[BLE_SENSOR_DATA_MAX_LEN] = {0};
static uint16_t s_sensor_data_len = 0;

/** Flag indicating whether the NimBLE stack has been initialized. */
static bool s_nimble_initialized = false;

/** Own address type (public or random). */
static uint8_t s_own_addr_type;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

/* GAP event handler */
static int gap_event_handler(struct ble_gap_event *event, void *arg);

/* GATT access callbacks */
static int gatt_sensor_data_access(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_ota_data_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_ota_ctrl_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Advertising configuration */
static void ble_advertise(void);

/* NimBLE host task and sync callback */
static void ble_host_task(void *param);
static void ble_on_sync(void);
static void ble_on_reset(int reason);

/* -------------------------------------------------------------------------- */
/*                        GATT Service Definitions                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief   GATT service table definition.
 *
 * @details Defines two custom services with their characteristics:
 *
 *          Service 1: Sensor Data Service (0x00FF)
 *            - Sensor Data (0xFF01): Read | Notify
 *
 *          Service 2: OTA Update Service (0x00FE)
 *            - OTA Data   (0xFE01): Write | Write No Response
 *            - OTA Control(0xFE02): Write | Read | Notify
 *
 *          The table is terminated with a {0} entry.
 */
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        /* ----- Sensor Data Service ----- */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SENSOR_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* Sensor Data Characteristic: Read + Notify */
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_SENSOR_DATA_UUID16),
                .access_cb = gatt_sensor_data_access,
                .val_handle = &s_gatt_handles.sensor_data_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, /* Terminator */
            },
        },
    },
    {
        /* ----- OTA Update Service ----- */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_OTA_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /* OTA Data Characteristic: Write + Write No Response */
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_OTA_DATA_UUID16),
                .access_cb = gatt_ota_data_access,
                .val_handle = &s_gatt_handles.ota_data_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* OTA Control Characteristic: Write + Read + Notify */
                .uuid = BLE_UUID16_DECLARE(BLE_CHR_OTA_CONTROL_UUID16),
                .access_cb = gatt_ota_ctrl_access,
                .val_handle = &s_gatt_handles.ota_ctrl_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, /* Terminator */
            },
        },
    },
    {
        0, /* End of service table */
    },
};

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

esp_err_t ble_gatt_server_init(void)
{
    esp_err_t ret;
    int rc;

    if (s_nimble_initialized)
    {
        ESP_LOGW(TAG, "BLE GATT server already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing BLE GATT server...");

    /*
     * Step 1: Initialize the NimBLE host stack.
     *
     * nimble_port_init() initializes the BLE controller and the NimBLE host.
     * This must be called before any other NimBLE API.
     */
    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nimble_port_init() failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /*
     * Step 2: Configure the NimBLE host callbacks.
     *
     * - on_sync: Called when the host and controller are synchronized and
     *   ready to accept commands. We start advertising here.
     * - on_reset: Called when the host resets due to an unrecoverable error.
     */
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    /* Set the preferred MTU for data exchange. */
    ble_att_set_preferred_mtu(BLE_PREFERRED_MTU);

    /*
     * Step 3: Initialize GATT and GAP services.
     *
     * ble_svc_gap_init()  - Registers the mandatory GAP service.
     * ble_svc_gatt_init() - Registers the mandatory GATT service.
     */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /*
     * Step 4: Register custom GATT services.
     *
     * ble_gatts_count_cfg() counts all attributes in the service table.
     * ble_gatts_add_svcs()  registers services with the GATT server.
     */
    rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_count_cfg() failed: rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gatts_add_svcs() failed: rc=%d", rc);
        return ESP_FAIL;
    }

    /*
     * Step 5: Set the device name in the GAP service.
     */
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to set device name: rc=%d", rc);
    }

    /*
     * Step 6: Start the NimBLE host task.
     *
     * The NimBLE host runs in its own FreeRTOS task. The task processes
     * BLE events and calls our registered callbacks.
     */
    nimble_port_freertos_init(ble_host_task);

    s_nimble_initialized = true;
    ESP_LOGI(TAG, "BLE GATT server initialized successfully");

    return ESP_OK;
}

esp_err_t ble_gatt_server_deinit(void)
{
    if (!s_nimble_initialized)
    {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Deinitializing BLE GATT server...");

    /* Stop the NimBLE host task. */
    int rc = nimble_port_stop();
    if (rc != 0)
    {
        ESP_LOGE(TAG, "nimble_port_stop() failed: rc=%d", rc);
        return ESP_FAIL;
    }

    nimble_port_deinit();

    s_nimble_initialized = false;
    s_conn_state = BLE_STATE_IDLE;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

    ESP_LOGI(TAG, "BLE GATT server deinitialized");
    return ESP_OK;
}

esp_err_t ble_gatt_server_register_callbacks(const ble_gatt_callbacks_t *callbacks)
{
    if (callbacks == NULL)
    {
        ESP_LOGE(TAG, "Callbacks pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(&s_callbacks, callbacks, sizeof(ble_gatt_callbacks_t));
    ESP_LOGI(TAG, "Application callbacks registered");

    return ESP_OK;
}

esp_err_t ble_gatt_server_notify_sensor_data(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_conn_state != BLE_STATE_CONNECTED)
    {
        return ESP_FAIL;
    }

    /* Clamp length to maximum. */
    if (len > BLE_SENSOR_DATA_MAX_LEN)
    {
        len = BLE_SENSOR_DATA_MAX_LEN;
    }

    /* Update the internal sensor data buffer for subsequent reads. */
    memcpy(s_sensor_data, data, len);
    s_sensor_data_len = len;

    /*
     * Send notification.
     *
     * ble_gatts_notify_custom() sends a notification with a custom payload
     * to the connected client. The client must have enabled notifications
     * by writing 0x0001 to the CCCD (Client Characteristic Configuration
     * Descriptor).
     */
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate mbuf for sensor notification");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle,
                                     s_gatt_handles.sensor_data_handle, om);
    if (rc != 0)
    {
        ESP_LOGD(TAG, "Sensor notification failed: rc=%d (client may not have enabled)", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t ble_gatt_server_notify_ota_status(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_conn_state != BLE_STATE_CONNECTED)
    {
        return ESP_FAIL;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate mbuf for OTA status notification");
        return ESP_ERR_NO_MEM;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle,
                                     s_gatt_handles.ota_ctrl_handle, om);
    if (rc != 0)
    {
        ESP_LOGD(TAG, "OTA status notification failed: rc=%d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

ble_conn_state_t ble_gatt_server_get_state(void)
{
    return s_conn_state;
}

uint16_t ble_gatt_server_get_mtu(void)
{
    return s_current_mtu;
}

const ble_gatt_handles_t *ble_gatt_server_get_handles(void)
{
    return &s_gatt_handles;
}

esp_err_t ble_gatt_server_start_advertising(void)
{
    ble_advertise();
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Configure and start BLE advertising.
 *
 * @details Configures connectable undirected advertising with the following
 *          advertising data:
 *          - Flags: General Discoverable + BR/EDR Not Supported
 *          - TX Power Level (automatically set by stack)
 *          - Complete Local Name: BLE_DEVICE_NAME
 *
 *          Advertising parameters:
 *          - Type: Connectable undirected (ADV_IND)
 *          - Interval: BLE_ADV_ITVL_MIN to BLE_ADV_ITVL_MAX
 *          - Channel map: All channels (37, 38, 39)
 *          - Filter policy: No whitelist
 */
static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    /*
     * Advertising data fields:
     *
     * Flags (AD Type 0x01):
     *   BLE_HS_ADV_F_DISC_GEN - General discoverable mode.
     *   BLE_HS_ADV_F_BREDR_UNSUP - Classic Bluetooth not supported.
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * TX Power Level (AD Type 0x0A):
     *   Include the TX power level so scanners can estimate distance.
     *   The actual value is filled in by the stack.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    /*
     * Complete Local Name (AD Type 0x09):
     *   Include the full device name in the advertising data.
     */
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields() failed: rc=%d", rc);
        return;
    }

    /*
     * Advertising parameters:
     *   - conn_mode: BLE_GAP_CONN_MODE_UND (undirected connectable)
     *   - disc_mode: BLE_GAP_DISC_MODE_GEN (general discoverable)
     *   - itvl_min/itvl_max: Advertising interval range in 0.625 ms units
     */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_ADV_ITVL_MIN;
    adv_params.itvl_max = BLE_ADV_ITVL_MAX;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "ble_gap_adv_start() failed: rc=%d", rc);
        return;
    }

    s_conn_state = BLE_STATE_ADVERTISING;
    ESP_LOGI(TAG, "Advertising started as '%s'", BLE_DEVICE_NAME);
}

/**
 * @brief   GAP event handler.
 *
 * @details Handles the following GAP events:
 *          - BLE_GAP_EVENT_CONNECT:      A central device has connected.
 *          - BLE_GAP_EVENT_DISCONNECT:   The central device has disconnected.
 *          - BLE_GAP_EVENT_MTU:          MTU exchange completed.
 *          - BLE_GAP_EVENT_ADV_COMPLETE: Advertising completed (timeout).
 *          - BLE_GAP_EVENT_SUBSCRIBE:    Client toggled CCCD (notifications).
 *
 * @param[in]   event   Pointer to the GAP event structure.
 * @param[in]   arg     User argument (unused).
 * @return  0 on success.
 */
static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;

    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        /*
         * Connection event.
         *
         * event->connect.status == 0 indicates a successful connection.
         * We store the connection handle for later use (notifications, etc.).
         */
        if (event->connect.status == 0)
        {
            s_conn_handle = event->connect.conn_handle;
            s_conn_state = BLE_STATE_CONNECTED;

            /* Query the connection descriptor for peer address info. */
            int rc = ble_gap_conn_find(s_conn_handle, &desc);
            if (rc == 0)
            {
                ESP_LOGI(TAG, "Connected: handle=%d, peer_addr=%02x:%02x:%02x:%02x:%02x:%02x",
                         s_conn_handle,
                         desc.peer_ota_addr.val[5], desc.peer_ota_addr.val[4],
                         desc.peer_ota_addr.val[3], desc.peer_ota_addr.val[2],
                         desc.peer_ota_addr.val[1], desc.peer_ota_addr.val[0]);
            }

            /* Notify the application of the connection. */
            if (s_callbacks.conn_state_cb != NULL)
            {
                s_callbacks.conn_state_cb(BLE_STATE_CONNECTED, s_conn_handle);
            }
        }
        else
        {
            ESP_LOGW(TAG, "Connection failed: status=%d", event->connect.status);

            /* Restart advertising on connection failure. */
            ble_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        /*
         * Disconnection event.
         *
         * event->disconnect.reason contains the HCI disconnect reason code:
         *   0x13 = Remote user terminated connection
         *   0x08 = Connection timeout
         *   0x16 = Connection terminated by local host
         */
        ESP_LOGI(TAG, "Disconnected: handle=%d, reason=0x%02x",
                 event->disconnect.conn.conn_handle,
                 event->disconnect.reason);

        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_conn_state = BLE_STATE_IDLE;
        s_current_mtu = 23; /* Reset to default MTU */

        /* Notify the application. */
        if (s_callbacks.conn_state_cb != NULL)
        {
            s_callbacks.conn_state_cb(BLE_STATE_IDLE, 0);
        }

        /* Restart advertising to accept new connections. */
        ble_advertise();
        break;

    case BLE_GAP_EVENT_MTU:
        /*
         * MTU exchange event.
         *
         * The central device initiates an MTU exchange to negotiate a larger
         * MTU for data transfer. A larger MTU allows bigger GATT write
         * payloads, which significantly improves OTA throughput.
         *
         * Effective payload = MTU - 3 (ATT header overhead)
         */
        s_current_mtu = ble_att_mtu(event->mtu.conn_handle);
        ESP_LOGI(TAG, "MTU updated: conn_handle=%d, mtu=%d (payload=%d)",
                 event->mtu.conn_handle, s_current_mtu,
                 s_current_mtu - 3);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        /*
         * Advertising completed (e.g., timeout or stopped).
         * Restart advertising unless we are connected.
         */
        if (s_conn_state != BLE_STATE_CONNECTED)
        {
            ESP_LOGI(TAG, "Advertising completed, restarting...");
            ble_advertise();
        }
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        /*
         * Subscribe event.
         *
         * Indicates a client has written to a CCCD (Client Characteristic
         * Configuration Descriptor) to enable or disable notifications/
         * indications for a characteristic.
         */
        ESP_LOGI(TAG, "Subscribe: conn_handle=%d, attr_handle=%d, "
                      "cur_notify=%d, cur_indicate=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle,
                 event->subscribe.cur_notify,
                 event->subscribe.cur_indicate);
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        /*
         * Connection parameter update.
         *
         * The central device may request updated connection parameters
         * (interval, latency, timeout) for power optimization.
         */
        ESP_LOGD(TAG, "Connection parameters updated: status=%d",
                 event->conn_update.status);
        break;

    default:
        ESP_LOGD(TAG, "Unhandled GAP event: type=%d", event->type);
        break;
    }

    return 0;
}

/**
 * @brief   GATT access callback for the Sensor Data characteristic.
 *
 * @details Handles read operations on the Sensor Data characteristic (0xFF01).
 *          Returns the latest sensor data from the internal buffer.
 *
 *          ATT Read Response format:
 *          +--------+-----------+---...---+
 *          | Opcode | Att Handle|  Value  |
 *          | (0x0B) | (2 bytes) | (N bytes)|
 *          +--------+-----------+---...---+
 *
 * @param[in]   conn_handle     Connection handle.
 * @param[in]   attr_handle     Attribute handle being accessed.
 * @param[in]   ctxt            GATT access context (op, om, etc.).
 * @param[in]   arg             User argument (unused).
 * @return  0 on success, BLE_ATT_ERR_xxx on error.
 */
static int gatt_sensor_data_access(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /*
         * Read operation: Return the current sensor data.
         *
         * os_mbuf_append() appends data to the output mbuf chain which
         * NimBLE uses to construct the ATT Read Response PDU.
         */
        ESP_LOGD(TAG, "Sensor Data READ: len=%d", s_sensor_data_len);

        if (s_sensor_data_len == 0)
        {
            /* No sensor data available yet; return empty. */
            return 0;
        }

        rc = os_mbuf_append(ctxt->om, s_sensor_data, s_sensor_data_len);
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    default:
        ESP_LOGW(TAG, "Unexpected op on Sensor Data: op=%d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * @brief   GATT access callback for the OTA Data characteristic.
 *
 * @details Handles write operations on the OTA Data characteristic (0xFE01).
 *          Each write delivers a firmware binary chunk that is forwarded to
 *          the OTA manager via the registered callback.
 *
 *          For optimal throughput, the client should use "Write Without
 *          Response" (ATT opcode 0x52) after negotiating a large MTU.
 *
 *          Throughput estimation:
 *            MTU = 512, payload = 509 bytes/write
 *            Connection interval = 7.5ms, 6 packets/interval
 *            Throughput ~ 509 * 6 / 0.0075 = ~407 KB/s (theoretical max)
 *
 * @param[in]   conn_handle     Connection handle.
 * @param[in]   attr_handle     Attribute handle being accessed.
 * @param[in]   ctxt            GATT access context.
 * @param[in]   arg             User argument (unused).
 * @return  0 on success, BLE_ATT_ERR_xxx on error.
 */
static int gatt_ota_data_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
    {
        /*
         * Write operation: Receive a firmware data chunk.
         *
         * The data may span multiple mbufs in the chain, so we need to
         * flatten the mbuf chain into a contiguous buffer before passing
         * it to the OTA data callback.
         */
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0)
        {
            ESP_LOGW(TAG, "OTA Data write: empty payload");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        /* Flatten mbuf chain into a stack buffer. */
        uint8_t buf[OTA_CHUNK_SIZE_MAX];
        uint16_t copy_len = (om_len > sizeof(buf)) ? sizeof(buf) : om_len;

        int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, copy_len, NULL);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "OTA Data: mbuf flatten failed: rc=%d", rc);
            return BLE_ATT_ERR_UNLIKELY;
        }

        ESP_LOGD(TAG, "OTA Data write: %d bytes", copy_len);

        /* Forward to the application callback. */
        if (s_callbacks.ota_data_cb != NULL)
        {
            int cb_rc = s_callbacks.ota_data_cb(buf, copy_len);
            if (cb_rc != 0)
            {
                ESP_LOGE(TAG, "OTA Data callback error: %d", cb_rc);
                return BLE_ATT_ERR_UNLIKELY;
            }
        }
        else
        {
            ESP_LOGW(TAG, "OTA Data received but no callback registered");
        }

        return 0;
    }

    default:
        ESP_LOGW(TAG, "Unexpected op on OTA Data: op=%d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * @brief   GATT access callback for the OTA Control characteristic.
 *
 * @details Handles read and write operations on the OTA Control
 *          characteristic (0xFE02).
 *
 *          Write format (from client):
 *          +--------+------------------+
 *          | CMD    | Payload          |
 *          | (1 B)  | (0-N bytes)      |
 *          +--------+------------------+
 *
 *          CMD byte values:
 *            0x01 = START   (payload: 4 bytes total_size, little-endian)
 *            0x02 = STOP    (no payload)
 *            0x03 = COMMIT  (no payload)
 *            0x04 = ROLLBACK(no payload)
 *            0x05 = VERSION_REQ (no payload)
 *
 *          Read format (to client):
 *          +--------+---------+--------+
 *          | State  | Error   | Progress|
 *          | (1 B)  | (1 B)   | (1 B)  |
 *          +--------+---------+--------+
 *
 * @param[in]   conn_handle     Connection handle.
 * @param[in]   attr_handle     Attribute handle being accessed.
 * @param[in]   ctxt            GATT access context.
 * @param[in]   arg             User argument (unused).
 * @return  0 on success, BLE_ATT_ERR_xxx on error.
 */
static int gatt_ota_ctrl_access(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
    {
        /*
         * Read operation: Return the current OTA state.
         *
         * The client reads this characteristic to query the OTA status
         * without waiting for a notification.
         */
        uint8_t status_buf[3] = {0};
        /* status_buf[0] = current state (filled by OTA manager externally) */
        /* status_buf[1] = last error code */
        /* status_buf[2] = progress percent */

        rc = os_mbuf_append(ctxt->om, status_buf, sizeof(status_buf));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
    {
        /*
         * Write operation: Receive an OTA control command.
         *
         * The first byte of the payload is the command. Remaining bytes
         * (if any) are command-specific parameters.
         */
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len < 1)
        {
            ESP_LOGW(TAG, "OTA Control write: payload too short");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        uint8_t buf[BLE_OTA_CTRL_MAX_LEN];
        uint16_t copy_len = (om_len > sizeof(buf)) ? sizeof(buf) : om_len;

        rc = ble_hs_mbuf_to_flat(ctxt->om, buf, copy_len, NULL);
        if (rc != 0)
        {
            ESP_LOGE(TAG, "OTA Control: mbuf flatten failed: rc=%d", rc);
            return BLE_ATT_ERR_UNLIKELY;
        }

        ble_ota_cmd_t cmd = (ble_ota_cmd_t)buf[0];
        const uint8_t *param_data = (copy_len > 1) ? &buf[1] : NULL;
        uint16_t param_len = (copy_len > 1) ? (copy_len - 1) : 0;

        ESP_LOGI(TAG, "OTA Control command: 0x%02X, param_len=%d",
                 cmd, param_len);

        /* Forward to the application callback. */
        if (s_callbacks.ota_ctrl_cb != NULL)
        {
            int cb_rc = s_callbacks.ota_ctrl_cb(cmd, param_data, param_len);
            if (cb_rc != 0)
            {
                ESP_LOGE(TAG, "OTA Control callback error: %d", cb_rc);
                return BLE_ATT_ERR_UNLIKELY;
            }
        }
        else
        {
            ESP_LOGW(TAG, "OTA Control command received but no callback registered");
        }

        return 0;
    }

    default:
        ESP_LOGW(TAG, "Unexpected op on OTA Control: op=%d", ctxt->op);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * @brief   NimBLE host task entry point.
 *
 * @details This is the main loop for the NimBLE host stack. The function
 *          nimble_port_run() blocks and processes BLE events until the
 *          host is stopped via nimble_port_stop().
 *
 *          The task runs at a high priority to ensure timely processing
 *          of BLE events (connection requests, data transfers, etc.).
 *
 * @param[in]   param   FreeRTOS task parameter (unused).
 */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");

    /* This function returns only when nimble_port_stop() is called. */
    nimble_port_run();

    /* Clean up after host task exits. */
    nimble_port_freertos_deinit();
    ESP_LOGI(TAG, "NimBLE host task ended");
}

/**
 * @brief   NimBLE host sync callback.
 *
 * @details Called by the NimBLE host when it has synchronized with the
 *          controller and is ready to accept commands. This is where we
 *          determine the device address type and start advertising.
 *
 *          Address types:
 *          - Public address:  Globally unique, burned into the chip.
 *          - Random address:  Generated at runtime (static or resolvable).
 */
static void ble_on_sync(void)
{
    int rc;

    ESP_LOGI(TAG, "BLE host synchronized with controller");

    /*
     * Determine the best address type to use for advertising.
     *
     * ble_hs_id_infer_auto() checks if a public address is available
     * (from eFuse or configured). If not, it falls back to a random
     * static address.
     */
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to infer address type: rc=%d", rc);
        return;
    }

    /* Log the device address. */
    uint8_t addr[6] = {0};
    rc = ble_hs_id_copy_addr(s_own_addr_type, addr, NULL);
    if (rc == 0)
    {
        ESP_LOGI(TAG, "Device address: %02x:%02x:%02x:%02x:%02x:%02x (type=%d)",
                 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0],
                 s_own_addr_type);
    }

    /* Start advertising. */
    ble_advertise();
}

/**
 * @brief   NimBLE host reset callback.
 *
 * @details Called when the NimBLE host resets due to an unrecoverable error.
 *          Common reasons include controller communication failure or resource
 *          exhaustion. After the reset callback returns, the host will
 *          automatically re-sync and call ble_on_sync() again.
 *
 * @param[in]   reason  The reset reason code.
 */
static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset: reason=%d", reason);

    s_conn_state = BLE_STATE_IDLE;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    s_current_mtu = 23;
}
