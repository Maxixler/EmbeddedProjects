/**
 * @file    ota_manager.h
 * @brief   OTA firmware update manager for ESP32 over BLE.
 *
 * @details Provides a state-machine-based OTA firmware update engine that
 *          receives firmware binary data in chunks (typically over BLE GATT),
 *          writes the data to the next OTA partition, verifies the image
 *          integrity via SHA-256 hash, and manages commit/rollback operations.
 *
 *          The OTA manager operates as a state machine:
 *
 *          +--------+   start()   +-----------+   all chunks   +------------+
 *          |  IDLE  |------------>| RECEIVING |--------------->| VALIDATING |
 *          +--------+            +-----------+                +------+-----+
 *              ^                      |                              |
 *              |                      | abort()               valid? |
 *              |                      v                    +---------+---------+
 *              |                 +--------+           YES  |                   | NO
 *              +<----------------| ERROR  |<-----------+   v                   v
 *              |                 +--------+            | +----------+    +--------+
 *              |                                       | | COMPLETE |    | ERROR  |
 *              +<--------------------------------------+ +----------+    +--------+
 *                         commit() / rollback()
 *
 *          Flash Partition Layout (OTA-enabled):
 *          +---------- 0x000000 ----------+
 *          |   Bootloader (0x1000)        |
 *          +--------- 0x008000 ----------+
 *          |   Partition Table            |
 *          +--------- 0x009000 ----------+
 *          |   NVS (Non-Volatile Storage) |
 *          +--------- 0x00F000 ----------+
 *          |   OTA Data Partition         |
 *          +--------- 0x010000 ----------+
 *          |   OTA_0 (app0) - Factory     |
 *          |   (up to ~1.5 MB)            |
 *          +--------- 0x190000 ----------+
 *          |   OTA_1 (app1) - Update      |
 *          |   (up to ~1.5 MB)            |
 *          +--------- 0x310000 ----------+
 *          |   Remaining flash            |
 *          +------------------------------+
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** Maximum size of a single OTA data chunk in bytes.
 *  Should be aligned with the BLE MTU minus ATT overhead (3 bytes). */
#define OTA_CHUNK_SIZE_MAX 512

/** Minimum firmware image size in bytes (sanity check). */
#define OTA_IMAGE_SIZE_MIN (32 * 1024) /* 32 KB */

/** Maximum firmware image size in bytes (partition size limit). */
#define OTA_IMAGE_SIZE_MAX (1536 * 1024) /* 1.5 MB */

/** SHA-256 digest length in bytes. */
#define OTA_SHA256_DIGEST_LEN 32

/** Firmware version string maximum length. */
#define OTA_VERSION_STRING_MAX_LEN 32

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   OTA update state machine states.
     */
    typedef enum
    {
        OTA_STATE_IDLE = 0,   /**< No update in progress, ready to start. */
        OTA_STATE_RECEIVING,  /**< Receiving firmware data chunks. */
        OTA_STATE_VALIDATING, /**< All data received, validating image. */
        OTA_STATE_COMPLETE,   /**< Validation passed, ready to commit. */
        OTA_STATE_ERROR,      /**< An error occurred during the update. */
    } ota_state_t;

    /**
     * @brief   OTA error codes for detailed failure reporting.
     */
    typedef enum
    {
        OTA_ERR_NONE = 0,            /**< No error. */
        OTA_ERR_ALREADY_IN_PROGRESS, /**< OTA session already active. */
        OTA_ERR_NOT_IN_PROGRESS,     /**< No active OTA session. */
        OTA_ERR_PARTITION_NOT_FOUND, /**< Next OTA partition not found. */
        OTA_ERR_BEGIN_FAILED,        /**< esp_ota_begin() failed. */
        OTA_ERR_WRITE_FAILED,        /**< esp_ota_write() failed. */
        OTA_ERR_IMAGE_TOO_SMALL,     /**< Received image smaller than minimum. */
        OTA_ERR_IMAGE_TOO_LARGE,     /**< Declared image exceeds partition size. */
        OTA_ERR_VALIDATION_FAILED,   /**< SHA-256 hash mismatch. */
        OTA_ERR_END_FAILED,          /**< esp_ota_end() failed (image invalid). */
        OTA_ERR_SET_BOOT_FAILED,     /**< esp_ota_set_boot_partition() failed. */
        OTA_ERR_ROLLBACK_FAILED,     /**< esp_ota_mark_app_invalid_rollback() failed. */
        OTA_ERR_INVALID_STATE,       /**< Operation not allowed in current state. */
    } ota_error_t;

    /**
     * @brief   OTA progress information structure.
     *
     * @details Provides real-time status of the firmware update process for
     *          reporting to the BLE client or serial console.
     */
    typedef struct
    {
        ota_state_t state;                                /**< Current state machine state. */
        ota_error_t error;                                /**< Last error code (if state == ERROR). */
        uint32_t total_size;                              /**< Total firmware image size in bytes. */
        uint32_t received_size;                           /**< Number of bytes received so far. */
        uint8_t progress_percent;                         /**< Download progress (0-100%). */
        char current_version[OTA_VERSION_STRING_MAX_LEN]; /**< Running firmware version. */
    } ota_progress_t;

    /**
     * @brief   Callback type for OTA state change notifications.
     *
     * @details Called whenever the OTA state machine transitions to a new state.
     *          The application can use this to update the BLE client or log status.
     *
     * @param[in]   progress    Pointer to the current progress information.
     */
    typedef void (*ota_state_change_cb_t)(const ota_progress_t *progress);

    /* -------------------------------------------------------------------------- */
    /*                          Public Function Prototypes                        */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialize the OTA manager.
     *
     * @details Queries the partition table for OTA partitions, reads the current
     *          running partition and boot partition information, and initializes
     *          the internal state machine to IDLE.
     *
     * @return  ESP_OK on success, ESP_ERR_NOT_FOUND if OTA partitions are missing.
     */
    esp_err_t ota_manager_init(void);

    /**
     * @brief   Start a new OTA firmware update session.
     *
     * @details Transitions the state machine from IDLE to RECEIVING. Selects the
     *          next OTA partition and calls esp_ota_begin() to prepare for writing.
     *
     * @param[in]   total_size  Total firmware image size in bytes. The manager uses
     *                          this to track progress and validate completeness.
     * @return  ESP_OK on success, ESP_FAIL if already in progress or partition error.
     */
    esp_err_t ota_manager_start(uint32_t total_size);

    /**
     * @brief   Write a firmware data chunk to the OTA partition.
     *
     * @details Called repeatedly as firmware chunks arrive over BLE. Each chunk is
     *          written sequentially to the OTA partition. The internal SHA-256
     *          context is updated incrementally with each chunk.
     *
     * @param[in]   data    Pointer to the firmware data chunk.
     * @param[in]   len     Length of the data chunk in bytes.
     * @return  ESP_OK on success, ESP_FAIL on write error or invalid state.
     */
    esp_err_t ota_manager_write_chunk(const uint8_t *data, uint16_t len);

    /**
     * @brief   Finalize and validate the received firmware image.
     *
     * @details Called after all chunks have been received. Transitions to
     *          VALIDATING state, calls esp_ota_end() to validate the image
     *          header and checksum, and optionally verifies the SHA-256 hash
     *          against the expected digest.
     *
     * @param[in]   expected_sha256     Pointer to the expected SHA-256 digest
     *                                  (32 bytes). Pass NULL to skip hash
     *                                  verification (not recommended).
     * @return  ESP_OK if image is valid, ESP_FAIL on validation error.
     */
    esp_err_t ota_manager_finish(const uint8_t *expected_sha256);

    /**
     * @brief   Commit the validated firmware and set it as the boot partition.
     *
     * @details Sets the newly written OTA partition as the next boot partition.
     *          After reboot, the new firmware will execute. The first successful
     *          boot should call esp_ota_mark_app_valid_cancel_rollback() to
     *          confirm the update, or the bootloader will automatically rollback.
     *
     * @return  ESP_OK on success, ESP_FAIL if no valid image to commit.
     */
    esp_err_t ota_manager_commit(void);

    /**
     * @brief   Rollback to the previously running firmware.
     *
     * @details Marks the current running firmware as invalid, causing the
     *          bootloader to revert to the previous OTA partition on next boot.
     *          This function triggers an automatic restart.
     *
     * @return  ESP_OK on success (device will restart), ESP_FAIL on error.
     */
    esp_err_t ota_manager_rollback(void);

    /**
     * @brief   Abort the current OTA session and return to IDLE.
     *
     * @details Cancels any in-progress firmware update. If data has been partially
     *          written, the OTA partition contents are discarded (via esp_ota_abort).
     *
     * @return  ESP_OK on success.
     */
    esp_err_t ota_manager_abort(void);

    /**
     * @brief   Get the current OTA progress information.
     *
     * @param[out]  progress    Pointer to a progress structure to be filled.
     * @return  ESP_OK on success, ESP_ERR_INVALID_ARG if progress is NULL.
     */
    esp_err_t ota_manager_get_progress(ota_progress_t *progress);

    /**
     * @brief   Get the current OTA state.
     *
     * @return  Current state as ota_state_t.
     */
    ota_state_t ota_manager_get_state(void);

    /**
     * @brief   Register a callback for OTA state change events.
     *
     * @param[in]   callback    Function pointer to the state change callback.
     */
    void ota_manager_register_state_callback(ota_state_change_cb_t callback);

    /**
     * @brief   Get information about the running firmware partition.
     *
     * @param[out]  label       Buffer to receive the partition label (e.g., "ota_0").
     * @param[in]   label_len   Length of the label buffer.
     * @param[out]  address     Pointer to receive the partition start address.
     * @param[out]  size        Pointer to receive the partition size in bytes.
     * @return  ESP_OK on success.
     */
    esp_err_t ota_manager_get_partition_info(char *label, size_t label_len,
                                             uint32_t *address, uint32_t *size);

    /**
     * @brief   Confirm that the current firmware is valid (anti-rollback).
     *
     * @details Should be called once the application has verified that the new
     *          firmware is functioning correctly. Prevents the bootloader from
     *          automatically rolling back on the next reboot.
     *
     * @return  ESP_OK on success.
     */
    esp_err_t ota_manager_confirm_image(void);

#ifdef __cplusplus
}
#endif

#endif /* OTA_MANAGER_H */
