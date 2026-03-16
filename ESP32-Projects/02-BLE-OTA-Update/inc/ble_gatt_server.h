/**
 * @file    ble_gatt_server.h
 * @brief   BLE GATT server with custom service for sensor data and OTA control.
 *
 * @details Provides a high-level API for initializing and managing a BLE GATT
 *          server on ESP32 using the NimBLE host stack. Exposes two custom
 *          GATT services:
 *
 *          1. **Sensor Data Service** (UUID: 0x00FF)
 *             - Sensor Data Characteristic (UUID: 0xFF01) [Read | Notify]
 *               Allows a central device to read the latest sensor reading or
 *               receive asynchronous notifications when data changes.
 *
 *          2. **OTA Update Service** (UUID: 0x00FE)
 *             - OTA Data Characteristic  (UUID: 0xFE01) [Write | Write No Rsp]
 *               Receives firmware binary chunks from the central device.
 *             - OTA Control Characteristic (UUID: 0xFE02) [Write | Read | Notify]
 *               Accepts OTA commands (start, commit, rollback) and reports
 *               the current OTA state back to the client.
 *
 *          GATT Server Architecture:
 *          +-------------------------------------------+
 *          |           NimBLE Host Stack                |
 *          +-------------------------------------------+
 *          |  +-- Sensor Data Service (0x00FF) ------+ |
 *          |  |   Sensor Data  [R|N]  (0xFF01)       | |
 *          |  +--------------------------------------+ |
 *          |  +-- OTA Update Service (0x00FE) -------+ |
 *          |  |   OTA Data     [W|WNR] (0xFE01)      | |
 *          |  |   OTA Control  [W|R|N] (0xFE02)      | |
 *          |  +--------------------------------------+ |
 *          +-------------------------------------------+
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef BLE_GATT_SERVER_H
#define BLE_GATT_SERVER_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gatt.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/**
 * @defgroup BLE_UUIDs  Custom GATT Service and Characteristic UUIDs
 * @{
 */

/** Sensor Data Service - 16-bit UUID. */
#define BLE_SVC_SENSOR_UUID16 0x00FF

/** Sensor Data Characteristic - 16-bit UUID. */
#define BLE_CHR_SENSOR_DATA_UUID16 0xFF01

/** OTA Update Service - 16-bit UUID. */
#define BLE_SVC_OTA_UUID16 0x00FE

/** OTA Data Characteristic (firmware binary chunks) - 16-bit UUID. */
#define BLE_CHR_OTA_DATA_UUID16 0xFE01

/** OTA Control Characteristic (start/commit/rollback commands) - 16-bit UUID. */
#define BLE_CHR_OTA_CONTROL_UUID16 0xFE02

/** @} */

/** BLE device name advertised to central devices. */
#define BLE_DEVICE_NAME "ESP32-OTA-GATT"

/** Maximum supported MTU size for BLE data transfer.
 *  Larger MTU improves OTA throughput by reducing per-packet overhead. */
#define BLE_PREFERRED_MTU 512

/** Maximum length of sensor data payload in bytes. */
#define BLE_SENSOR_DATA_MAX_LEN 64

/** Maximum length of OTA control command/response in bytes. */
#define BLE_OTA_CTRL_MAX_LEN 32

/** Advertising interval minimum (in 0.625 ms units). 160 = 100 ms. */
#define BLE_ADV_ITVL_MIN 160

/** Advertising interval maximum (in 0.625 ms units). 800 = 500 ms. */
#define BLE_ADV_ITVL_MAX 800

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   OTA control commands written by the BLE client.
     *
     * @details These commands are sent by the central device to the OTA Control
     *          characteristic to manage the firmware update process.
     */
    typedef enum
    {
        BLE_OTA_CMD_START = 0x01,       /**< Start a new OTA update session. */
        BLE_OTA_CMD_STOP = 0x02,        /**< Abort the current OTA update. */
        BLE_OTA_CMD_COMMIT = 0x03,      /**< Commit (validate & activate) the new firmware. */
        BLE_OTA_CMD_ROLLBACK = 0x04,    /**< Rollback to the previous firmware. */
        BLE_OTA_CMD_VERSION_REQ = 0x05, /**< Request current firmware version info. */
    } ble_ota_cmd_t;

    /**
     * @brief   BLE connection state enumeration.
     */
    typedef enum
    {
        BLE_STATE_IDLE = 0,    /**< No active connection. */
        BLE_STATE_ADVERTISING, /**< Advertising and waiting for connection. */
        BLE_STATE_CONNECTED,   /**< Central device is connected. */
    } ble_conn_state_t;

    /**
     * @brief   GATT server attribute handle set.
     *
     * @details Stores the attribute handles assigned by the NimBLE stack during
     *          service registration. These handles are needed for sending
     *          notifications and accessing characteristic values.
     */
    typedef struct
    {
        uint16_t sensor_data_handle; /**< Handle for Sensor Data characteristic value. */
        uint16_t ota_data_handle;    /**< Handle for OTA Data characteristic value. */
        uint16_t ota_ctrl_handle;    /**< Handle for OTA Control characteristic value. */
    } ble_gatt_handles_t;

    /**
     * @brief   Callback type for OTA data reception.
     *
     * @details Called each time a firmware chunk is received on the OTA Data
     *          characteristic. The implementation should forward the data to
     *          the OTA manager for writing to the update partition.
     *
     * @param[in]   data    Pointer to the received firmware data chunk.
     * @param[in]   len     Length of the data chunk in bytes.
     * @return  0 on success, non-zero on error (will be sent as GATT error).
     */
    typedef int (*ble_ota_data_cb_t)(const uint8_t *data, uint16_t len);

    /**
     * @brief   Callback type for OTA control commands.
     *
     * @details Called when the BLE client writes a command to the OTA Control
     *          characteristic (e.g., start, commit, rollback).
     *
     * @param[in]   cmd     The OTA command received from the client.
     * @param[in]   data    Optional command parameter data (may be NULL).
     * @param[in]   len     Length of the parameter data.
     * @return  0 on success, non-zero on error.
     */
    typedef int (*ble_ota_ctrl_cb_t)(ble_ota_cmd_t cmd, const uint8_t *data, uint16_t len);

    /**
     * @brief   Callback type for BLE connection state changes.
     *
     * @param[in]   state       New connection state.
     * @param[in]   conn_handle Connection handle (valid only when connected).
     */
    typedef void (*ble_conn_state_cb_t)(ble_conn_state_t state, uint16_t conn_handle);

    /**
     * @brief   BLE GATT server callback registration structure.
     *
     * @details Groups all application-level callbacks that the GATT server
     *          invokes on various BLE events.
     */
    typedef struct
    {
        ble_ota_data_cb_t ota_data_cb;     /**< Firmware data chunk received. */
        ble_ota_ctrl_cb_t ota_ctrl_cb;     /**< OTA control command received. */
        ble_conn_state_cb_t conn_state_cb; /**< Connection state changed. */
    } ble_gatt_callbacks_t;

    /* -------------------------------------------------------------------------- */
    /*                          Public Function Prototypes                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize the BLE GATT server and start advertising.
     *
     * @details Performs the following initialization sequence:
     *          1. Initialize the NimBLE host stack and controller.
     *          2. Configure GAP parameters (device name, appearance).
     *          3. Register GATT services and characteristics.
     *          4. Start advertising with connectable undirected mode.
     *
     * @return  ESP_OK on success, ESP_ERR_xxx on failure.
     */
    esp_err_t ble_gatt_server_init(void);

    /**
     * @brief   Deinitialize the BLE GATT server and stop all BLE activity.
     *
     * @return  ESP_OK on success.
     */
    esp_err_t ble_gatt_server_deinit(void);

    /**
     * @brief   Register application callbacks for BLE events.
     *
     * @details Must be called before or after ble_gatt_server_init(). The
     *          callbacks can be updated at any time.
     *
     * @param[in]   callbacks   Pointer to the callback structure. The structure
     *                          is copied internally; the caller need not keep
     *                          the pointer valid after the call.
     * @return  ESP_OK on success, ESP_ERR_INVALID_ARG if callbacks is NULL.
     */
    esp_err_t ble_gatt_server_register_callbacks(const ble_gatt_callbacks_t *callbacks);

    /**
     * @brief   Send a sensor data notification to the connected client.
     *
     * @details Sends a BLE GATT notification on the Sensor Data characteristic.
     *          If no client is connected or notifications are not enabled by the
     *          client (via CCCD), this function returns silently.
     *
     * @param[in]   data    Pointer to the sensor data payload.
     * @param[in]   len     Length of the payload in bytes (max BLE_SENSOR_DATA_MAX_LEN).
     * @return  ESP_OK on success, ESP_FAIL if not connected or notification disabled.
     */
    esp_err_t ble_gatt_server_notify_sensor_data(const uint8_t *data, uint16_t len);

    /**
     * @brief   Send an OTA status notification to the connected client.
     *
     * @details Sends a BLE GATT notification on the OTA Control characteristic
     *          to inform the client about the current OTA state (e.g., progress,
     *          completion, error).
     *
     * @param[in]   data    Pointer to the status payload.
     * @param[in]   len     Length of the payload in bytes.
     * @return  ESP_OK on success, ESP_FAIL if not connected.
     */
    esp_err_t ble_gatt_server_notify_ota_status(const uint8_t *data, uint16_t len);

    /**
     * @brief   Get the current BLE connection state.
     *
     * @return  Current connection state as ble_conn_state_t.
     */
    ble_conn_state_t ble_gatt_server_get_state(void);

    /**
     * @brief   Get the current negotiated MTU size.
     *
     * @details Returns the MTU negotiated with the connected central device.
     *          If no connection is active, returns the default MTU (23).
     *
     * @return  Current MTU size in bytes.
     */
    uint16_t ble_gatt_server_get_mtu(void);

    /**
     * @brief   Get the GATT attribute handles.
     *
     * @details Returns a pointer to the internal handle structure. Handles are
     *          valid only after ble_gatt_server_init() has been called.
     *
     * @return  Pointer to the ble_gatt_handles_t structure.
     */
    const ble_gatt_handles_t *ble_gatt_server_get_handles(void);

    /**
     * @brief   Restart BLE advertising after a disconnection.
     *
     * @details Called internally on disconnect events, but may also be invoked
     *          by the application to manually restart advertising.
     *
     * @return  ESP_OK on success.
     */
    esp_err_t ble_gatt_server_start_advertising(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_SERVER_H */
