/**
 * @file    ota_manager.c
 * @brief   OTA firmware update manager implementation for ESP32.
 *
 * @details Implements a state-machine-based OTA firmware update engine that:
 *          1. Selects the next available OTA partition for writing.
 *          2. Receives firmware binary data in chunks and writes to flash.
 *          3. Computes an incremental SHA-256 hash for integrity verification.
 *          4. Validates the firmware image via esp_ota_end().
 *          5. Supports commit (set as boot partition) and rollback operations.
 *
 *          State machine transitions:
 *
 *          [IDLE] --start()--> [RECEIVING] --finish()--> [VALIDATING]
 *                                  |                        |
 *                                  | abort()         +------+------+
 *                                  v                 |             |
 *                              [IDLE]           [COMPLETE]    [ERROR]
 *                                                    |
 *                                              commit() --> reboot
 *
 *          ESP-IDF OTA API call sequence:
 *          1. esp_ota_get_next_update_partition()  - Find target partition
 *          2. esp_ota_begin()                      - Erase and prepare partition
 *          3. esp_ota_write() [repeated]           - Write firmware chunks
 *          4. esp_ota_end()                        - Validate image
 *          5. esp_ota_set_boot_partition()         - Set as next boot
 *          6. esp_restart()                        - Reboot into new firmware
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "ota_manager.h"

#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_app_format.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_flash_partitions.h"
#include "esp_image_format.h"
#include "mbedtls/sha256.h"

/* -------------------------------------------------------------------------- */
/*                            Private Variables                               */
/* -------------------------------------------------------------------------- */

/** Logging tag for ESP_LOGx macros. */
static const char *TAG = "ota_mgr";

/** Current state of the OTA state machine. */
static ota_state_t s_state = OTA_STATE_IDLE;

/** Last error code. */
static ota_error_t s_last_error = OTA_ERR_NONE;

/** OTA handle returned by esp_ota_begin(). */
static esp_ota_handle_t s_ota_handle = 0;

/** Pointer to the target (next update) OTA partition. */
static const esp_partition_t *s_update_partition = NULL;

/** Pointer to the currently running partition. */
static const esp_partition_t *s_running_partition = NULL;

/** Total firmware image size declared at start. */
static uint32_t s_total_size = 0;

/** Number of firmware bytes received so far. */
static uint32_t s_received_size = 0;

/** SHA-256 hash context for incremental digest computation. */
static mbedtls_sha256_context s_sha256_ctx;

/** Computed SHA-256 digest of the received firmware. */
static uint8_t s_computed_sha256[OTA_SHA256_DIGEST_LEN] = {0};

/** Application-registered state change callback. */
static ota_state_change_cb_t s_state_callback = NULL;

/** Flag indicating whether the OTA manager has been initialized. */
static bool s_initialized = false;

/* -------------------------------------------------------------------------- */
/*                        Private Function Prototypes                         */
/* -------------------------------------------------------------------------- */

static void set_state(ota_state_t new_state);
static void set_error(ota_error_t error);
static void notify_state_change(void);
static void get_firmware_version(char *version_buf, size_t buf_len);

/* -------------------------------------------------------------------------- */
/*                          Public Function Definitions                       */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_init(void)
{
    if (s_initialized)
    {
        ESP_LOGW(TAG, "OTA manager already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing OTA manager...");

    /*
     * Step 1: Get the currently running partition.
     *
     * esp_ota_get_running_partition() returns a pointer to the partition
     * descriptor of the application firmware that is currently executing.
     * This is typically "ota_0" or "ota_1".
     */
    s_running_partition = esp_ota_get_running_partition();
    if (s_running_partition == NULL)
    {
        ESP_LOGE(TAG, "Failed to get running partition");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Running partition: label='%s', type=%d, subtype=%d, "
                  "offset=0x%08lx, size=0x%08lx",
             s_running_partition->label,
             s_running_partition->type,
             s_running_partition->subtype,
             (unsigned long)s_running_partition->address,
             (unsigned long)s_running_partition->size);

    /*
     * Step 2: Identify the next OTA update partition.
     *
     * esp_ota_get_next_update_partition() selects the OTA partition that
     * is NOT currently running. For a two-OTA-slot configuration:
     *   - If running from ota_0, the next update partition is ota_1.
     *   - If running from ota_1, the next update partition is ota_0.
     */
    s_update_partition = esp_ota_get_next_update_partition(NULL);
    if (s_update_partition == NULL)
    {
        ESP_LOGE(TAG, "Failed to get next update partition. "
                      "Check partition table for OTA slots.");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Next update partition: label='%s', offset=0x%08lx, size=0x%08lx",
             s_update_partition->label,
             (unsigned long)s_update_partition->address,
             (unsigned long)s_update_partition->size);

    /*
     * Step 3: Log the boot partition info.
     *
     * The boot partition is the partition the bootloader will load on
     * next reboot. It may differ from the running partition if an OTA
     * update was committed but the device hasn't rebooted yet.
     */
    const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
    if (boot_partition != NULL)
    {
        ESP_LOGI(TAG, "Boot partition: label='%s'", boot_partition->label);
    }

    /* Initialize state. */
    s_state = OTA_STATE_IDLE;
    s_last_error = OTA_ERR_NONE;
    s_received_size = 0;
    s_total_size = 0;

    s_initialized = true;
    ESP_LOGI(TAG, "OTA manager initialized successfully");

    return ESP_OK;
}

esp_err_t ota_manager_start(uint32_t total_size)
{
    if (!s_initialized)
    {
        ESP_LOGE(TAG, "OTA manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state != OTA_STATE_IDLE)
    {
        ESP_LOGE(TAG, "Cannot start OTA: already in state %d", s_state);
        set_error(OTA_ERR_ALREADY_IN_PROGRESS);
        return ESP_FAIL;
    }

    /*
     * Validate the declared firmware size.
     *
     * The total_size is provided by the BLE client before sending firmware
     * chunks. We validate it against the minimum image size and the
     * partition capacity.
     */
    if (total_size < OTA_IMAGE_SIZE_MIN)
    {
        ESP_LOGE(TAG, "Firmware size too small: %lu bytes (min: %d)",
                 (unsigned long)total_size, OTA_IMAGE_SIZE_MIN);
        set_error(OTA_ERR_IMAGE_TOO_SMALL);
        return ESP_FAIL;
    }

    if (total_size > s_update_partition->size)
    {
        ESP_LOGE(TAG, "Firmware size (%lu) exceeds partition capacity (%lu)",
                 (unsigned long)total_size,
                 (unsigned long)s_update_partition->size);
        set_error(OTA_ERR_IMAGE_TOO_LARGE);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting OTA update: total_size=%lu bytes, "
                  "target_partition='%s'",
             (unsigned long)total_size, s_update_partition->label);

    /*
     * Step 1: Begin the OTA update.
     *
     * esp_ota_begin() performs the following:
     *   1. Erases the target OTA partition.
     *   2. Prepares the partition for sequential writes.
     *   3. Returns an OTA handle for subsequent write/end calls.
     *
     * OTA_WITH_SEQUENTIAL_WRITES (or OTA_SIZE_UNKNOWN) tells the API
     * that we will write data sequentially in chunks rather than
     * providing the entire image at once.
     */
    esp_err_t ret = esp_ota_begin(s_update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                                  &s_ota_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin() failed: %s", esp_err_to_name(ret));
        set_error(OTA_ERR_BEGIN_FAILED);
        return ESP_FAIL;
    }

    /*
     * Step 2: Initialize the SHA-256 hash context.
     *
     * We compute the SHA-256 hash incrementally as chunks arrive.
     * This avoids needing to buffer the entire firmware image in RAM.
     *
     * mbedTLS SHA-256 API:
     *   init()   -> Initialize context
     *   starts() -> Begin hash computation (0 = SHA-256, 1 = SHA-224)
     *   update() -> Feed data incrementally
     *   finish() -> Finalize and output 32-byte digest
     */
    mbedtls_sha256_init(&s_sha256_ctx);
    mbedtls_sha256_starts(&s_sha256_ctx, 0); /* 0 = SHA-256 (not SHA-224) */

    /* Reset counters. */
    s_total_size = total_size;
    s_received_size = 0;
    memset(s_computed_sha256, 0, sizeof(s_computed_sha256));

    set_state(OTA_STATE_RECEIVING);
    ESP_LOGI(TAG, "OTA session started, ready to receive firmware chunks");

    return ESP_OK;
}

esp_err_t ota_manager_write_chunk(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state != OTA_STATE_RECEIVING)
    {
        ESP_LOGE(TAG, "Cannot write chunk: not in RECEIVING state (state=%d)",
                 s_state);
        set_error(OTA_ERR_INVALID_STATE);
        return ESP_FAIL;
    }

    /*
     * Check if this chunk would exceed the declared total size.
     */
    if ((s_received_size + len) > s_total_size)
    {
        ESP_LOGE(TAG, "Chunk would exceed total size: received=%lu + chunk=%d > total=%lu",
                 (unsigned long)s_received_size, len,
                 (unsigned long)s_total_size);
        set_error(OTA_ERR_IMAGE_TOO_LARGE);
        set_state(OTA_STATE_ERROR);
        return ESP_FAIL;
    }

    /*
     * Write the chunk to the OTA partition.
     *
     * esp_ota_write() writes data sequentially to the partition at the
     * current write offset (maintained internally by the OTA API).
     * The data is written directly to flash via SPI.
     *
     * Performance note: Flash write speed is typically 8-40 KB/s depending
     * on the SPI bus configuration and flash chip. BLE throughput (~50 KB/s
     * practical) may occasionally outpace flash writes, so the NimBLE stack
     * handles flow control via the BLE connection event mechanism.
     */
    esp_err_t ret = esp_ota_write(s_ota_handle, data, len);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_write() failed: %s (offset=%lu, len=%d)",
                 esp_err_to_name(ret), (unsigned long)s_received_size, len);
        set_error(OTA_ERR_WRITE_FAILED);
        set_state(OTA_STATE_ERROR);
        return ESP_FAIL;
    }

    /*
     * Update the SHA-256 hash with the new chunk.
     *
     * The hash is computed incrementally so we never need to buffer the
     * entire firmware image. After all chunks are received, we call
     * mbedtls_sha256_finish() to get the final 32-byte digest.
     */
    mbedtls_sha256_update(&s_sha256_ctx, data, len);

    s_received_size += len;

    /* Calculate and log progress at 10% intervals. */
    uint8_t progress = (uint8_t)((s_received_size * 100) / s_total_size);
    static uint8_t last_logged_progress = 0;

    if (progress >= last_logged_progress + 10 || progress == 100)
    {
        ESP_LOGI(TAG, "OTA progress: %lu/%lu bytes (%d%%)",
                 (unsigned long)s_received_size,
                 (unsigned long)s_total_size, progress);
        last_logged_progress = progress;
    }

    /* Notify state change for progress update. */
    notify_state_change();

    return ESP_OK;
}

esp_err_t ota_manager_finish(const uint8_t *expected_sha256)
{
    if (s_state != OTA_STATE_RECEIVING)
    {
        ESP_LOGE(TAG, "Cannot finish: not in RECEIVING state (state=%d)",
                 s_state);
        set_error(OTA_ERR_INVALID_STATE);
        return ESP_FAIL;
    }

    /*
     * Verify that we received the expected number of bytes.
     */
    if (s_received_size != s_total_size)
    {
        ESP_LOGE(TAG, "Incomplete firmware: received %lu of %lu bytes",
                 (unsigned long)s_received_size,
                 (unsigned long)s_total_size);
        set_error(OTA_ERR_IMAGE_TOO_SMALL);
        set_state(OTA_STATE_ERROR);
        return ESP_FAIL;
    }

    set_state(OTA_STATE_VALIDATING);
    ESP_LOGI(TAG, "All firmware data received, validating image...");

    /*
     * Step 1: Finalize the SHA-256 hash.
     */
    mbedtls_sha256_finish(&s_sha256_ctx, s_computed_sha256);
    mbedtls_sha256_free(&s_sha256_ctx);

    /* Log the computed hash. */
    ESP_LOGI(TAG, "Computed SHA-256: %02x%02x%02x%02x...%02x%02x%02x%02x",
             s_computed_sha256[0], s_computed_sha256[1],
             s_computed_sha256[2], s_computed_sha256[3],
             s_computed_sha256[28], s_computed_sha256[29],
             s_computed_sha256[30], s_computed_sha256[31]);

    /*
     * Step 2: Verify SHA-256 hash (if expected digest is provided).
     *
     * The BLE client can send the expected SHA-256 hash of the firmware
     * image as part of the OTA COMMIT command. We compare it against
     * the incrementally computed hash.
     */
    if (expected_sha256 != NULL)
    {
        if (memcmp(s_computed_sha256, expected_sha256, OTA_SHA256_DIGEST_LEN) != 0)
        {
            ESP_LOGE(TAG, "SHA-256 verification FAILED!");
            ESP_LOGE(TAG, "Expected: %02x%02x%02x%02x...%02x%02x%02x%02x",
                     expected_sha256[0], expected_sha256[1],
                     expected_sha256[2], expected_sha256[3],
                     expected_sha256[28], expected_sha256[29],
                     expected_sha256[30], expected_sha256[31]);

            set_error(OTA_ERR_VALIDATION_FAILED);
            set_state(OTA_STATE_ERROR);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "SHA-256 verification PASSED");
    }
    else
    {
        ESP_LOGW(TAG, "No expected SHA-256 provided, skipping hash verification");
    }

    /*
     * Step 3: Finalize the OTA update via esp_ota_end().
     *
     * esp_ota_end() performs the following validation:
     *   1. Checks the ESP-IDF image header magic byte (0xE9).
     *   2. Validates the segment count and chip ID.
     *   3. Verifies the image checksum.
     *   4. Optionally checks the secure boot signature.
     *
     * If validation fails, the written data is discarded and the
     * OTA handle is invalidated.
     */
    esp_err_t ret = esp_ota_end(s_ota_handle);
    if (ret != ESP_OK)
    {
        if (ret == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG, "Image validation failed (corrupt or incompatible firmware)");
        }
        else
        {
            ESP_LOGE(TAG, "esp_ota_end() failed: %s", esp_err_to_name(ret));
        }

        set_error(OTA_ERR_END_FAILED);
        set_state(OTA_STATE_ERROR);
        return ESP_FAIL;
    }

    set_state(OTA_STATE_COMPLETE);
    ESP_LOGI(TAG, "Firmware validation passed. Ready to commit.");

    return ESP_OK;
}

esp_err_t ota_manager_commit(void)
{
    if (s_state != OTA_STATE_COMPLETE)
    {
        ESP_LOGE(TAG, "Cannot commit: not in COMPLETE state (state=%d)",
                 s_state);
        set_error(OTA_ERR_INVALID_STATE);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Committing new firmware: setting boot partition to '%s'",
             s_update_partition->label);

    /*
     * Set the newly written partition as the boot partition.
     *
     * esp_ota_set_boot_partition() updates the OTA data partition to
     * instruct the bootloader to load from the specified app partition
     * on the next boot.
     *
     * OTA data partition layout (16 bytes per entry, 2 entries):
     *   +--------+--------+--------+-----------+
     *   | SeqNum | Label  | State  | SHA-256   |
     *   | (4 B)  | (16 B) | (4 B)  | (32 B)   |
     *   +--------+--------+--------+-----------+
     *
     * The bootloader selects the entry with the highest sequence number
     * that has a valid state (ESP_OTA_IMG_NEW or ESP_OTA_IMG_VALID).
     */
    esp_err_t ret = esp_ota_set_boot_partition(s_update_partition);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition() failed: %s",
                 esp_err_to_name(ret));
        set_error(OTA_ERR_SET_BOOT_FAILED);
        set_state(OTA_STATE_ERROR);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Boot partition updated. New firmware will be active after reboot.");
    ESP_LOGI(TAG, "Restarting device in 1 second...");

    /* Notify the application of the successful commit. */
    notify_state_change();

    /*
     * Delay before restarting to allow:
     *   1. BLE notification of success to be sent to the client.
     *   2. Log messages to be flushed to serial output.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Restart the device to boot the new firmware. */
    esp_restart();

    /* This line is never reached. */
    return ESP_OK;
}

esp_err_t ota_manager_rollback(void)
{
    ESP_LOGW(TAG, "Rolling back to previous firmware...");

    /*
     * Mark the currently running firmware as invalid.
     *
     * esp_ota_mark_app_invalid_rollback_and_reboot() sets the current
     * OTA app state to ESP_OTA_IMG_INVALID and triggers an immediate
     * reboot. The bootloader will then load the previous valid OTA
     * partition.
     *
     * Rollback protection flow:
     *   1. New firmware boots with state ESP_OTA_IMG_NEW.
     *   2. If the app calls esp_ota_mark_app_valid_cancel_rollback(),
     *      the state changes to ESP_OTA_IMG_VALID.
     *   3. If the app does NOT confirm (e.g., crashes), the bootloader
     *      detects the failure and rolls back automatically.
     *   4. This function allows manual rollback at any time.
     */
    esp_err_t ret = esp_ota_mark_app_invalid_rollback_and_reboot();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Rollback failed: %s", esp_err_to_name(ret));
        set_error(OTA_ERR_ROLLBACK_FAILED);
        return ESP_FAIL;
    }

    /* This line is never reached (device reboots). */
    return ESP_OK;
}

esp_err_t ota_manager_abort(void)
{
    if (s_state == OTA_STATE_IDLE)
    {
        ESP_LOGW(TAG, "No OTA session to abort");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "Aborting OTA session (received %lu/%lu bytes)",
             (unsigned long)s_received_size,
             (unsigned long)s_total_size);

    /*
     * Abort the OTA session.
     *
     * esp_ota_abort() discards any partially written data and
     * invalidates the OTA handle. The target partition contents
     * are left in an undefined state (partially erased/written).
     */
    if (s_state == OTA_STATE_RECEIVING || s_state == OTA_STATE_VALIDATING)
    {
        esp_err_t ret = esp_ota_abort(s_ota_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "esp_ota_abort() returned: %s", esp_err_to_name(ret));
        }
    }

    /* Clean up SHA-256 context. */
    mbedtls_sha256_free(&s_sha256_ctx);

    /* Reset state. */
    s_received_size = 0;
    s_total_size = 0;
    s_last_error = OTA_ERR_NONE;
    set_state(OTA_STATE_IDLE);

    ESP_LOGI(TAG, "OTA session aborted, returning to IDLE");

    return ESP_OK;
}

esp_err_t ota_manager_get_progress(ota_progress_t *progress)
{
    if (progress == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    progress->state = s_state;
    progress->error = s_last_error;
    progress->total_size = s_total_size;
    progress->received_size = s_received_size;

    if (s_total_size > 0)
    {
        progress->progress_percent =
            (uint8_t)((s_received_size * 100) / s_total_size);
    }
    else
    {
        progress->progress_percent = 0;
    }

    get_firmware_version(progress->current_version,
                         sizeof(progress->current_version));

    return ESP_OK;
}

ota_state_t ota_manager_get_state(void)
{
    return s_state;
}

void ota_manager_register_state_callback(ota_state_change_cb_t callback)
{
    s_state_callback = callback;
    ESP_LOGI(TAG, "State change callback registered");
}

esp_err_t ota_manager_get_partition_info(char *label, size_t label_len,
                                         uint32_t *address, uint32_t *size)
{
    if (s_running_partition == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (label != NULL && label_len > 0)
    {
        strncpy(label, s_running_partition->label, label_len - 1);
        label[label_len - 1] = '\0';
    }

    if (address != NULL)
    {
        *address = s_running_partition->address;
    }

    if (size != NULL)
    {
        *size = s_running_partition->size;
    }

    return ESP_OK;
}

esp_err_t ota_manager_confirm_image(void)
{
    ESP_LOGI(TAG, "Confirming current firmware image as valid...");

    /*
     * Mark the current app as valid to cancel automatic rollback.
     *
     * After an OTA update, the new firmware boots with state
     * ESP_OTA_IMG_NEW. If the app does not call this function before
     * the next reboot (e.g., due to a crash or watchdog reset), the
     * bootloader will automatically rollback to the previous firmware.
     *
     * This mechanism provides "A/B update" safety:
     *   - If the new firmware works: confirm it.
     *   - If the new firmware crashes: bootloader rolls back automatically.
     */
    esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to confirm image: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Firmware image confirmed as valid (rollback cancelled)");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                        Private Function Definitions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Update the OTA state machine state.
 *
 * @details Logs the state transition and notifies the registered callback.
 *
 * @param[in]   new_state   The new state to transition to.
 */
static void set_state(ota_state_t new_state)
{
    static const char *state_names[] = {
        "IDLE", "RECEIVING", "VALIDATING", "COMPLETE", "ERROR"};

    ota_state_t old_state = s_state;
    s_state = new_state;

    if (old_state != new_state)
    {
        ESP_LOGI(TAG, "State: %s -> %s",
                 state_names[old_state], state_names[new_state]);
        notify_state_change();
    }
}

/**
 * @brief   Set the last error code and transition to ERROR state.
 *
 * @param[in]   error   The error code to set.
 */
static void set_error(ota_error_t error)
{
    s_last_error = error;

    static const char *error_names[] = {
        "NONE", "ALREADY_IN_PROGRESS", "NOT_IN_PROGRESS",
        "PARTITION_NOT_FOUND", "BEGIN_FAILED", "WRITE_FAILED",
        "IMAGE_TOO_SMALL", "IMAGE_TOO_LARGE", "VALIDATION_FAILED",
        "END_FAILED", "SET_BOOT_FAILED", "ROLLBACK_FAILED",
        "INVALID_STATE"};

    if (error < sizeof(error_names) / sizeof(error_names[0]))
    {
        ESP_LOGE(TAG, "OTA error set: %s (%d)", error_names[error], error);
    }
}

/**
 * @brief   Notify the application of a state change via the registered callback.
 */
static void notify_state_change(void)
{
    if (s_state_callback != NULL)
    {
        ota_progress_t progress;
        ota_manager_get_progress(&progress);
        s_state_callback(&progress);
    }
}

/**
 * @brief   Get the firmware version string from the app description.
 *
 * @details Reads the ESP-IDF application description, which is embedded
 *          in the firmware binary header. The version string is set at
 *          build time via the PROJECT_VER macro in CMakeLists.txt.
 *
 * @param[out]  version_buf     Buffer to receive the version string.
 * @param[in]   buf_len         Size of the buffer.
 */
static void get_firmware_version(char *version_buf, size_t buf_len)
{
    if (version_buf == NULL || buf_len == 0)
    {
        return;
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    if (app_desc != NULL)
    {
        strncpy(version_buf, app_desc->version, buf_len - 1);
        version_buf[buf_len - 1] = '\0';
    }
    else
    {
        strncpy(version_buf, "unknown", buf_len - 1);
        version_buf[buf_len - 1] = '\0';
    }
}
