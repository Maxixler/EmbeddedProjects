/**
 * @file rtc_storage.c
 * @brief RTC Memory Storage - Implementation
 *
 * Manages persistent data across ESP32 deep sleep cycles using
 * RTC_DATA_ATTR variables in RTC slow memory. Provides boot counting,
 * sensor data buffering, wake-up history logging, CRC32 integrity
 * verification, and NVS backup functionality.
 *
 * @author Embedded Systems Developer
 * @date 2026-03-16
 * @version 1.0.0
 *
 * @note RTC slow memory (8 KB) is retained during deep sleep but
 *       cleared on power-on reset. NVS is used for cross-power-cycle
 *       persistence.
 */

/*============================================================================
 *                              INCLUDES
 *============================================================================*/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "rtc_storage.h"

/*============================================================================
 *                          PRIVATE DEFINITIONS
 *============================================================================*/

/** @brief Module log tag */
static const char *TAG = "RTCSTORE";

/** @brief Total RTC slow memory size (bytes) */
#define RTC_SLOW_MEM_TOTAL_BYTES 8192

/*============================================================================
 *                     RTC_DATA_ATTR VARIABLES
 *
 * These variables persist across deep sleep cycles. They are placed in
 * RTC slow memory using the RTC_DATA_ATTR attribute. On power-on reset,
 * they are initialized to zero by the bootloader.
 *============================================================================*/

/**
 * @brief Boot metadata stored in RTC memory
 *
 * Contains boot counters, timestamps, and cumulative statistics.
 * Validated via magic number on each boot.
 */
RTC_DATA_ATTR static rtcstore_metadata_t s_metadata;

/**
 * @brief Sensor data buffer state
 *
 * Tracks the circular buffer write position and count.
 */
RTC_DATA_ATTR static rtcstore_buffer_state_t s_buffer_state;

/**
 * @brief Sensor readings circular buffer
 *
 * Stores up to RTCSTORE_MAX_SENSOR_READINGS measurements in RTC memory.
 * Each reading is ~16 bytes, total ~1024 bytes for 64 readings.
 */
RTC_DATA_ATTR static rtcstore_sensor_reading_t s_sensor_buffer[RTCSTORE_MAX_SENSOR_READINGS];

/**
 * @brief Wake-up history buffer state
 */
RTC_DATA_ATTR static rtcstore_history_state_t s_history_state;

/**
 * @brief Wake-up history circular buffer
 *
 * Records the last RTCSTORE_MAX_WAKEUP_HISTORY wake-up events with
 * cause, timestamp, battery level, and active duration.
 */
RTC_DATA_ATTR static rtcstore_wakeup_entry_t s_wakeup_history[RTCSTORE_MAX_WAKEUP_HISTORY];

/*============================================================================
 *                         PRIVATE VARIABLES
 *============================================================================*/

/** @brief NVS handle for persistent storage */
static nvs_handle_t s_nvs_handle = 0;

/** @brief NVS initialization flag */
static bool s_nvs_initialized = false;

/*============================================================================
 *                     PRIVATE FUNCTION PROTOTYPES
 *============================================================================*/

static esp_err_t prv_init_nvs(void);
static void prv_close_nvs(void);
static void prv_reset_metadata(void);
static void prv_reset_sensor_buffer(void);
static void prv_reset_history_buffer(void);
static uint32_t prv_calculate_sensor_crc(void);
static uint32_t prv_calculate_history_crc(void);
static uint32_t prv_get_rtc_timestamp(void);

/*============================================================================
 *                     PUBLIC FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Initialize the RTC storage subsystem
 */
esp_err_t rtcstore_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing RTC storage...");

    /* Initialize NVS for persistent backup */
    ret = prv_init_nvs();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS initialization failed: %s", esp_err_to_name(ret));
        /* Continue without NVS - RTC storage still works */
    }

    /* Check if this is a fresh boot or wake from deep sleep */
    if (s_metadata.magic != RTCSTORE_MAGIC_NUMBER)
    {
        /*
         * First boot (power-on reset) or RTC memory was corrupted.
         * Initialize all RTC data structures from scratch.
         */
        ESP_LOGI(TAG, "First boot detected (magic=0x%08lX, expected=0x%08lX)",
                 (unsigned long)s_metadata.magic,
                 (unsigned long)RTCSTORE_MAGIC_NUMBER);

        prv_reset_metadata();
        prv_reset_sensor_buffer();
        prv_reset_history_buffer();

        s_metadata.magic = RTCSTORE_MAGIC_NUMBER;
        s_metadata.first_boot_time = prv_get_rtc_timestamp();

        /* Load total boot count from NVS if available */
        if (s_nvs_initialized)
        {
            uint32_t total = 0;
            ret = nvs_get_u32(s_nvs_handle, RTCSTORE_NVS_KEY_TOTAL_BOOTS, &total);
            if (ret == ESP_OK)
            {
                s_metadata.total_boot_count = total;
                ESP_LOGI(TAG, "Loaded total boot count from NVS: %lu",
                         (unsigned long)total);
            }
            else
            {
                s_metadata.total_boot_count = 0;
                ESP_LOGI(TAG, "No previous boot count in NVS (first ever boot)");
            }
        }

        ESP_LOGI(TAG, "RTC storage initialized (fresh boot)");
        return ESP_OK;
    }

    /*
     * Wake from deep sleep - verify data integrity.
     */
    ESP_LOGI(TAG, "Deep sleep wake-up detected, verifying data integrity...");

    bool sensor_ok, history_ok;
    bool all_ok = rtcstore_verify_integrity(&sensor_ok, &history_ok);

    if (!all_ok)
    {
        ESP_LOGW(TAG, "Data integrity check failed (sensor:%s, history:%s)",
                 sensor_ok ? "OK" : "FAIL",
                 history_ok ? "OK" : "FAIL");

        if (!sensor_ok)
        {
            ESP_LOGW(TAG, "Resetting corrupted sensor buffer");
            prv_reset_sensor_buffer();
        }
        if (!history_ok)
        {
            ESP_LOGW(TAG, "Resetting corrupted history buffer");
            prv_reset_history_buffer();
        }

        return ESP_ERR_INVALID_CRC;
    }

    s_metadata.last_boot_time = prv_get_rtc_timestamp();

    ESP_LOGI(TAG, "RTC storage verified (boot #%lu, %lu sensor readings buffered)",
             (unsigned long)s_metadata.boot_count,
             (unsigned long)s_buffer_state.count);

    return ESP_OK;
}

/**
 * @brief Increment the boot counter
 */
uint32_t rtcstore_increment_boot_count(void)
{
    s_metadata.boot_count++;
    s_metadata.total_boot_count++;
    s_metadata.last_boot_time = prv_get_rtc_timestamp();

    /* Periodically save total boot count to NVS */
    if (s_nvs_initialized && (s_metadata.total_boot_count % 10 == 0))
    {
        nvs_set_u32(s_nvs_handle, RTCSTORE_NVS_KEY_TOTAL_BOOTS,
                    s_metadata.total_boot_count);
        nvs_commit(s_nvs_handle);
    }

    ESP_LOGI(TAG, "Boot count: %lu (total: %lu)",
             (unsigned long)s_metadata.boot_count,
             (unsigned long)s_metadata.total_boot_count);

    return s_metadata.boot_count;
}

/**
 * @brief Get the current boot count
 */
uint32_t rtcstore_get_boot_count(void)
{
    return s_metadata.boot_count;
}

/**
 * @brief Get the total boot count across power cycles
 */
uint32_t rtcstore_get_total_boot_count(void)
{
    return s_metadata.total_boot_count;
}

/**
 * @brief Store a sensor reading in the RTC buffer
 */
esp_err_t rtcstore_add_sensor_reading(const rtcstore_sensor_reading_t *reading)
{
    if (reading == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Store reading at current write position */
    uint32_t idx = s_buffer_state.write_index;
    memcpy(&s_sensor_buffer[idx], reading, sizeof(rtcstore_sensor_reading_t));

    /* Advance write index (circular) */
    s_buffer_state.write_index = (idx + 1) % RTCSTORE_MAX_SENSOR_READINGS;
    s_buffer_state.total_readings++;

    /* Update count (cap at buffer size) */
    if (s_buffer_state.count < RTCSTORE_MAX_SENSOR_READINGS)
    {
        s_buffer_state.count++;
    }
    else
    {
        s_buffer_state.buffer_full = true;
    }

    /* Update CRC */
    s_buffer_state.crc32 = prv_calculate_sensor_crc();

    ESP_LOGD(TAG, "Sensor reading stored at index %lu (count: %lu, total: %lu)",
             (unsigned long)idx,
             (unsigned long)s_buffer_state.count,
             (unsigned long)s_buffer_state.total_readings);

    /* Check if flush is needed */
    if (s_buffer_state.count >= RTCSTORE_FLUSH_THRESHOLD)
    {
        ESP_LOGW(TAG, "Sensor buffer near full (%lu/%d), flush recommended",
                 (unsigned long)s_buffer_state.count,
                 RTCSTORE_MAX_SENSOR_READINGS);
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

/**
 * @brief Get all buffered sensor readings
 */
esp_err_t rtcstore_get_sensor_readings(rtcstore_sensor_reading_t *readings,
                                       uint32_t max_readings,
                                       uint32_t *count)
{
    if (readings == NULL || count == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Verify CRC before returning data */
    uint32_t calc_crc = prv_calculate_sensor_crc();
    if (calc_crc != s_buffer_state.crc32)
    {
        ESP_LOGE(TAG, "Sensor buffer CRC mismatch (stored: 0x%08lX, calc: 0x%08lX)",
                 (unsigned long)s_buffer_state.crc32,
                 (unsigned long)calc_crc);
        *count = 0;
        return ESP_ERR_INVALID_CRC;
    }

    /* Calculate how many readings to copy */
    uint32_t n = s_buffer_state.count;
    if (n > max_readings)
    {
        n = max_readings;
    }

    /*
     * Copy readings in chronological order.
     * If the buffer has wrapped, the oldest entry is at write_index.
     * If not wrapped, entries start at index 0.
     */
    uint32_t start_idx;
    if (s_buffer_state.buffer_full || s_buffer_state.count >= RTCSTORE_MAX_SENSOR_READINGS)
    {
        start_idx = s_buffer_state.write_index;
    }
    else
    {
        start_idx = 0;
    }

    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t src_idx = (start_idx + i) % RTCSTORE_MAX_SENSOR_READINGS;
        memcpy(&readings[i], &s_sensor_buffer[src_idx],
               sizeof(rtcstore_sensor_reading_t));
    }

    *count = n;
    return ESP_OK;
}

/**
 * @brief Get the number of buffered sensor readings
 */
uint32_t rtcstore_get_sensor_count(void)
{
    return s_buffer_state.count;
}

/**
 * @brief Check if sensor buffer needs flushing
 */
bool rtcstore_needs_flush(void)
{
    return (s_buffer_state.count >= RTCSTORE_FLUSH_THRESHOLD);
}

/**
 * @brief Clear the sensor reading buffer
 */
void rtcstore_clear_sensor_buffer(void)
{
    ESP_LOGI(TAG, "Clearing sensor buffer (%lu readings discarded)",
             (unsigned long)s_buffer_state.count);

    memset(s_sensor_buffer, 0, sizeof(s_sensor_buffer));
    prv_reset_sensor_buffer();
    s_buffer_state.crc32 = prv_calculate_sensor_crc();
}

/**
 * @brief Record a wake-up event in the history log
 */
esp_err_t rtcstore_add_wakeup_entry(const rtcstore_wakeup_entry_t *entry)
{
    if (entry == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Store entry at current write position */
    uint32_t idx = s_history_state.write_index;
    memcpy(&s_wakeup_history[idx], entry, sizeof(rtcstore_wakeup_entry_t));

    /* Advance write index (circular) */
    s_history_state.write_index = (idx + 1) % RTCSTORE_MAX_WAKEUP_HISTORY;

    /* Update count (cap at buffer size) */
    if (s_history_state.count < RTCSTORE_MAX_WAKEUP_HISTORY)
    {
        s_history_state.count++;
    }

    /* Update CRC */
    s_history_state.crc32 = prv_calculate_history_crc();

    ESP_LOGD(TAG, "Wake-up entry recorded at index %lu (count: %lu)",
             (unsigned long)idx,
             (unsigned long)s_history_state.count);

    return ESP_OK;
}

/**
 * @brief Get the wake-up history log
 */
esp_err_t rtcstore_get_wakeup_history(rtcstore_wakeup_entry_t *entries,
                                      uint32_t max_entries,
                                      uint32_t *count)
{
    if (entries == NULL || count == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t n = s_history_state.count;
    if (n > max_entries)
    {
        n = max_entries;
    }

    /* Copy entries in chronological order */
    uint32_t start_idx;
    if (s_history_state.count >= RTCSTORE_MAX_WAKEUP_HISTORY)
    {
        start_idx = s_history_state.write_index;
    }
    else
    {
        start_idx = 0;
    }

    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t src_idx = (start_idx + i) % RTCSTORE_MAX_WAKEUP_HISTORY;
        memcpy(&entries[i], &s_wakeup_history[src_idx],
               sizeof(rtcstore_wakeup_entry_t));
    }

    *count = n;
    return ESP_OK;
}

/**
 * @brief Flush all RTC data to NVS
 */
esp_err_t rtcstore_flush_to_nvs(void)
{
    if (!s_nvs_initialized)
    {
        esp_err_t ret = prv_init_nvs();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Cannot flush: NVS not available");
            return ESP_FAIL;
        }
    }

    esp_err_t ret;

    /* Save total boot count */
    ret = nvs_set_u32(s_nvs_handle, RTCSTORE_NVS_KEY_TOTAL_BOOTS,
                      s_metadata.total_boot_count);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to save boot count: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /* Save sensor data as blob */
    if (s_buffer_state.count > 0)
    {
        size_t data_size = s_buffer_state.count * sizeof(rtcstore_sensor_reading_t);
        ret = nvs_set_blob(s_nvs_handle, RTCSTORE_NVS_KEY_SENSOR_DATA,
                           s_sensor_buffer, data_size);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to save sensor data: %s", esp_err_to_name(ret));
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "Saved %lu sensor readings to NVS (%u bytes)",
                 (unsigned long)s_buffer_state.count, (unsigned)data_size);
    }

    /* Save wake-up history as blob */
    if (s_history_state.count > 0)
    {
        size_t hist_size = s_history_state.count * sizeof(rtcstore_wakeup_entry_t);
        ret = nvs_set_blob(s_nvs_handle, RTCSTORE_NVS_KEY_WAKEUP_HIST,
                           s_wakeup_history, hist_size);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to save wake-up history: %s", esp_err_to_name(ret));
            return ESP_FAIL;
        }
    }

    /* Commit all changes */
    ret = nvs_commit(s_nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    s_metadata.nvs_flush_count++;
    ESP_LOGI(TAG, "NVS flush #%lu complete",
             (unsigned long)s_metadata.nvs_flush_count);

    return ESP_OK;
}

/**
 * @brief Load previously saved data from NVS
 */
esp_err_t rtcstore_load_from_nvs(void)
{
    if (!s_nvs_initialized)
    {
        esp_err_t ret = prv_init_nvs();
        if (ret != ESP_OK)
        {
            return ESP_FAIL;
        }
    }

    /* Load total boot count */
    uint32_t total = 0;
    esp_err_t ret = nvs_get_u32(s_nvs_handle, RTCSTORE_NVS_KEY_TOTAL_BOOTS, &total);
    if (ret == ESP_OK)
    {
        s_metadata.total_boot_count = total;
    }
    else if (ret != ESP_ERR_NVS_NOT_FOUND)
    {
        return ESP_FAIL;
    }

    /* Load sensor data */
    size_t data_size = sizeof(s_sensor_buffer);
    ret = nvs_get_blob(s_nvs_handle, RTCSTORE_NVS_KEY_SENSOR_DATA,
                       s_sensor_buffer, &data_size);
    if (ret == ESP_OK)
    {
        s_buffer_state.count = data_size / sizeof(rtcstore_sensor_reading_t);
        s_buffer_state.write_index = s_buffer_state.count % RTCSTORE_MAX_SENSOR_READINGS;
        s_buffer_state.crc32 = prv_calculate_sensor_crc();
        ESP_LOGI(TAG, "Loaded %lu sensor readings from NVS",
                 (unsigned long)s_buffer_state.count);
    }
    else if (ret == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No saved sensor data in NVS");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to load sensor data: %s", esp_err_to_name(ret));
    }

    /* Load wake-up history */
    size_t hist_size = sizeof(s_wakeup_history);
    ret = nvs_get_blob(s_nvs_handle, RTCSTORE_NVS_KEY_WAKEUP_HIST,
                       s_wakeup_history, &hist_size);
    if (ret == ESP_OK)
    {
        s_history_state.count = hist_size / sizeof(rtcstore_wakeup_entry_t);
        s_history_state.write_index = s_history_state.count % RTCSTORE_MAX_WAKEUP_HISTORY;
        s_history_state.crc32 = prv_calculate_history_crc();
        ESP_LOGI(TAG, "Loaded %lu wake-up history entries from NVS",
                 (unsigned long)s_history_state.count);
    }

    return ESP_OK;
}

/**
 * @brief Verify data integrity using CRC32
 */
bool rtcstore_verify_integrity(bool *sensor_ok, bool *history_ok)
{
    bool s_ok = true;
    bool h_ok = true;

    /* Verify sensor buffer CRC */
    if (s_buffer_state.count > 0)
    {
        uint32_t calc_crc = prv_calculate_sensor_crc();
        s_ok = (calc_crc == s_buffer_state.crc32);
        if (!s_ok)
        {
            ESP_LOGW(TAG, "Sensor CRC mismatch: stored=0x%08lX, calc=0x%08lX",
                     (unsigned long)s_buffer_state.crc32,
                     (unsigned long)calc_crc);
        }
    }

    /* Verify history buffer CRC */
    if (s_history_state.count > 0)
    {
        uint32_t calc_crc = prv_calculate_history_crc();
        h_ok = (calc_crc == s_history_state.crc32);
        if (!h_ok)
        {
            ESP_LOGW(TAG, "History CRC mismatch: stored=0x%08lX, calc=0x%08lX",
                     (unsigned long)s_history_state.crc32,
                     (unsigned long)calc_crc);
        }
    }

    if (sensor_ok != NULL)
        *sensor_ok = s_ok;
    if (history_ok != NULL)
        *history_ok = h_ok;

    return (s_ok && h_ok);
}

/**
 * @brief Get a complete storage summary
 */
esp_err_t rtcstore_get_summary(rtcstore_summary_t *summary)
{
    if (summary == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Copy metadata */
    memcpy(&summary->metadata, &s_metadata, sizeof(rtcstore_metadata_t));

    /* Buffer counts */
    summary->sensor_count = s_buffer_state.count;
    summary->history_count = s_history_state.count;

    /* Integrity check */
    summary->data_valid = rtcstore_verify_integrity(
        &summary->sensor_crc_ok, &summary->history_crc_ok);

    /* Memory usage estimate */
    summary->rtc_memory_used =
        sizeof(s_metadata) +
        sizeof(s_buffer_state) +
        (s_buffer_state.count * sizeof(rtcstore_sensor_reading_t)) +
        sizeof(s_history_state) +
        (s_history_state.count * sizeof(rtcstore_wakeup_entry_t));

    summary->rtc_memory_free = RTC_SLOW_MEM_TOTAL_BYTES - summary->rtc_memory_used;

    return ESP_OK;
}

/**
 * @brief Print storage status report to console
 */
void rtcstore_print_status(void)
{
    rtcstore_summary_t summary;
    rtcstore_get_summary(&summary);

    printf("\n");
    printf("========================================================\n");
    printf("          RTC STORAGE STATUS REPORT\n");
    printf("========================================================\n");
    printf("\n");
    printf("  Boot Information:\n");
    printf("    Boot count (this power cycle): %lu\n",
           (unsigned long)summary.metadata.boot_count);
    printf("    Total boot count (all time):   %lu\n",
           (unsigned long)summary.metadata.total_boot_count);
    printf("    First boot timestamp:          %lu s\n",
           (unsigned long)summary.metadata.first_boot_time);
    printf("    Last boot timestamp:           %lu s\n",
           (unsigned long)summary.metadata.last_boot_time);
    printf("    Total sleep time:              %lu s\n",
           (unsigned long)summary.metadata.total_sleep_time_s);
    printf("    Total active time:             %lu ms\n",
           (unsigned long)summary.metadata.total_active_time_ms);
    printf("    Data transmissions:            %lu\n",
           (unsigned long)summary.metadata.data_send_count);
    printf("    NVS flush operations:          %lu\n",
           (unsigned long)summary.metadata.nvs_flush_count);
    printf("\n");
    printf("  Sensor Data Buffer:\n");
    printf("    Readings buffered:             %lu / %d\n",
           (unsigned long)summary.sensor_count,
           RTCSTORE_MAX_SENSOR_READINGS);
    printf("    Total readings taken:          %lu\n",
           (unsigned long)s_buffer_state.total_readings);
    printf("    Buffer full:                   %s\n",
           s_buffer_state.buffer_full ? "YES" : "NO");
    printf("    Needs flush:                   %s\n",
           rtcstore_needs_flush() ? "YES" : "NO");
    printf("    CRC32:                         0x%08lX (%s)\n",
           (unsigned long)s_buffer_state.crc32,
           summary.sensor_crc_ok ? "VALID" : "INVALID");
    printf("\n");
    printf("  Wake-up History:\n");
    printf("    Entries logged:                %lu / %d\n",
           (unsigned long)summary.history_count,
           RTCSTORE_MAX_WAKEUP_HISTORY);
    printf("    CRC32:                         0x%08lX (%s)\n",
           (unsigned long)s_history_state.crc32,
           summary.history_crc_ok ? "VALID" : "INVALID");
    printf("\n");
    printf("  Memory Usage:\n");
    printf("    RTC memory used:               %lu bytes\n",
           (unsigned long)summary.rtc_memory_used);
    printf("    RTC memory free:               %lu bytes\n",
           (unsigned long)summary.rtc_memory_free);
    printf("    Data integrity:                %s\n",
           summary.data_valid ? "ALL OK" : "ERRORS DETECTED");
    printf("\n");
    printf("========================================================\n");
    printf("\n");
}

/**
 * @brief Update cumulative timing statistics
 */
void rtcstore_update_timing(uint32_t active_time_ms, uint32_t sleep_duration_s)
{
    s_metadata.total_active_time_ms += active_time_ms;
    s_metadata.total_sleep_time_s += sleep_duration_s;
}

/**
 * @brief Reset all RTC storage data
 */
void rtcstore_reset(void)
{
    ESP_LOGW(TAG, "Resetting all RTC storage data");
    prv_reset_metadata();
    prv_reset_sensor_buffer();
    prv_reset_history_buffer();
    memset(s_sensor_buffer, 0, sizeof(s_sensor_buffer));
    memset(s_wakeup_history, 0, sizeof(s_wakeup_history));
    s_metadata.magic = RTCSTORE_MAGIC_NUMBER;
}

/**
 * @brief Reset all data including NVS
 */
esp_err_t rtcstore_factory_reset(void)
{
    ESP_LOGW(TAG, "Factory reset: clearing RTC storage and NVS");

    /* Reset RTC data */
    rtcstore_reset();

    /* Erase NVS namespace */
    if (!s_nvs_initialized)
    {
        esp_err_t ret = prv_init_nvs();
        if (ret != ESP_OK)
        {
            return ESP_FAIL;
        }
    }

    esp_err_t ret = nvs_erase_all(s_nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS erase failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ret = nvs_commit(s_nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Factory reset complete");
    return ESP_OK;
}

/**
 * @brief Calculate CRC32 for a data buffer
 */
uint32_t rtcstore_calculate_crc32(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0)
    {
        return 0;
    }

    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 1)
            {
                crc = (crc >> 1) ^ RTCSTORE_CRC32_POLYNOMIAL;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

/*============================================================================
 *                    PRIVATE FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Initialize NVS flash and open the storage namespace
 *
 * @return ESP_OK on success
 */
static esp_err_t prv_init_nvs(void)
{
    if (s_nvs_initialized)
    {
        return ESP_OK;
    }

    /* Initialize NVS flash if not already done */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition issue, erasing...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Open NVS namespace */
    ret = nvs_open(RTCSTORE_NVS_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_nvs_initialized = true;
    return ESP_OK;
}

/**
 * @brief Close NVS handle
 */
static void prv_close_nvs(void)
{
    if (s_nvs_initialized && s_nvs_handle != 0)
    {
        nvs_close(s_nvs_handle);
        s_nvs_handle = 0;
        s_nvs_initialized = false;
    }
}

/**
 * @brief Reset metadata to default values
 */
static void prv_reset_metadata(void)
{
    memset(&s_metadata, 0, sizeof(s_metadata));
}

/**
 * @brief Reset sensor buffer state
 */
static void prv_reset_sensor_buffer(void)
{
    memset(&s_buffer_state, 0, sizeof(s_buffer_state));
}

/**
 * @brief Reset history buffer state
 */
static void prv_reset_history_buffer(void)
{
    memset(&s_history_state, 0, sizeof(s_history_state));
}

/**
 * @brief Calculate CRC32 of the sensor data buffer
 *
 * @return CRC32 value
 */
static uint32_t prv_calculate_sensor_crc(void)
{
    if (s_buffer_state.count == 0)
    {
        return 0;
    }

    size_t data_size = s_buffer_state.count * sizeof(rtcstore_sensor_reading_t);
    if (data_size > sizeof(s_sensor_buffer))
    {
        data_size = sizeof(s_sensor_buffer);
    }

    return rtcstore_calculate_crc32((const uint8_t *)s_sensor_buffer, data_size);
}

/**
 * @brief Calculate CRC32 of the wake-up history buffer
 *
 * @return CRC32 value
 */
static uint32_t prv_calculate_history_crc(void)
{
    if (s_history_state.count == 0)
    {
        return 0;
    }

    size_t data_size = s_history_state.count * sizeof(rtcstore_wakeup_entry_t);
    if (data_size > sizeof(s_wakeup_history))
    {
        data_size = sizeof(s_wakeup_history);
    }

    return rtcstore_calculate_crc32((const uint8_t *)s_wakeup_history, data_size);
}

/**
 * @brief Get a timestamp from the RTC clock
 *
 * Returns a relative timestamp in seconds since boot. On ESP32,
 * the RTC clock runs at approximately 150 kHz and persists during
 * deep sleep.
 *
 * @return Relative timestamp in seconds
 */
static uint32_t prv_get_rtc_timestamp(void)
{
    /*
     * Use esp_timer_get_time() which returns microseconds since boot.
     * During deep sleep, the RTC clock continues to run, so this
     * provides a monotonic timestamp.
     */
    int64_t time_us = esp_timer_get_time();
    return (uint32_t)(time_us / 1000000LL);
}
