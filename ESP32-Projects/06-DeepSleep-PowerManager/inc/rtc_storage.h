/**
 * @file rtc_storage.h
 * @brief RTC Memory Storage - Persistent Data Across Deep Sleep Cycles
 *
 * This module manages data persistence across ESP32 deep sleep cycles
 * using RTC_DATA_ATTR variables stored in RTC slow memory (8 KB).
 * RTC memory is retained during deep sleep but lost on power-on reset.
 *
 * Features:
 *  - Boot counter tracking
 *  - Circular buffer for accumulated sensor readings
 *  - Wake-up history log (last N events with timestamps)
 *  - CRC32 data integrity verification
 *  - Automatic flush to NVS when buffers are full
 *
 * Memory Layout (RTC Slow Memory - 8 KB total):
 *  - Boot metadata:      ~32 bytes
 *  - Sensor buffer:      ~2048 bytes (64 readings x 32 bytes)
 *  - Wake-up history:    ~512 bytes (32 entries x 16 bytes)
 *  - CRC + reserved:     ~64 bytes
 *
 * @author Embedded Systems Developer
 * @date 2026-03-16
 * @version 1.0.0
 *
 * @note RTC slow memory is shared with the ULP coprocessor.
 *       Ensure no address conflicts if ULP programs are used.
 * @warning Data is lost on power-on reset, brownout reset, or
 *          software reset. Only retained during deep sleep.
 */

#ifndef RTC_STORAGE_H
#define RTC_STORAGE_H

#ifdef __cplusplus
extern "C"
{
#endif

    /*============================================================================
     *                              INCLUDES
     *============================================================================*/

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*============================================================================
 *                         MACRO DEFINITIONS
 *============================================================================*/

/**
 * @defgroup RTCSTORE_CONFIG RTC Storage Configuration
 * @{
 */

/** @brief Maximum number of sensor readings in the RTC buffer */
#define RTCSTORE_MAX_SENSOR_READINGS 64

/** @brief Maximum number of wake-up history entries */
#define RTCSTORE_MAX_WAKEUP_HISTORY 32

/** @brief NVS namespace for persistent data backup */
#define RTCSTORE_NVS_NAMESPACE "rtc_store"

/** @brief NVS key for total boot count (across power cycles) */
#define RTCSTORE_NVS_KEY_TOTAL_BOOTS "total_boots"

/** @brief NVS key for stored sensor data blob */
#define RTCSTORE_NVS_KEY_SENSOR_DATA "sensor_data"

/** @brief NVS key for wake-up history blob */
#define RTCSTORE_NVS_KEY_WAKEUP_HIST "wakeup_hist"

/** @brief CRC32 polynomial for data integrity checks */
#define RTCSTORE_CRC32_POLYNOMIAL 0xEDB88320UL

/** @brief Magic number to verify RTC memory validity */
#define RTCSTORE_MAGIC_NUMBER 0xDEEP5LP1UL

/** @brief Threshold for auto-flushing sensor buffer to NVS */
#define RTCSTORE_FLUSH_THRESHOLD (RTCSTORE_MAX_SENSOR_READINGS - 4)

    /** @} */

    /*============================================================================
     *                         TYPE DEFINITIONS
     *============================================================================*/

    /**
     * @brief Sensor reading data structure
     *
     * Stores a single sensor measurement with timestamp and metadata.
     * Designed to be compact for efficient RTC memory usage.
     */
    typedef struct
    {
        uint32_t timestamp;  /**< Relative timestamp in seconds since first boot */
        int16_t temperature; /**< Temperature in 0.1 degree C units (e.g., 235 = 23.5C) */
        uint16_t humidity;   /**< Humidity in 0.1% units (e.g., 650 = 65.0%) */
        uint16_t pressure;   /**< Pressure in 0.1 hPa units (offset from 9000) */
        uint16_t battery_mv; /**< Battery voltage in millivolts */
        int16_t light_lux;   /**< Light level in lux (0-65535) */
        uint8_t wake_cause;  /**< Wake-up cause that triggered this reading */
        uint8_t reserved;    /**< Reserved for alignment / future use */
    } __attribute__((packed)) rtcstore_sensor_reading_t;

    /**
     * @brief Wake-up history entry
     *
     * Records details of each wake-up event for diagnostic purposes.
     */
    typedef struct
    {
        uint32_t timestamp;        /**< Relative timestamp in seconds */
        uint8_t cause;             /**< Wake-up cause (pwrmgr_wakeup_cause_t) */
        uint8_t battery_pct;       /**< Battery percentage at wake-up */
        uint16_t active_time_ms;   /**< Time spent active before sleeping (ms) */
        uint32_t sleep_duration_s; /**< Sleep duration that was configured (s) */
        uint32_t gpio_state;       /**< GPIO state at wake-up (EXT0/EXT1 pin values) */
    } __attribute__((packed)) rtcstore_wakeup_entry_t;

    /**
     * @brief RTC storage metadata (boot information)
     *
     * Core metadata stored in RTC memory for tracking boot cycles
     * and data buffer state.
     */
    typedef struct
    {
        uint32_t magic;                /**< Magic number for validity check */
        uint32_t boot_count;           /**< Boot counter (since last power-on) */
        uint32_t total_boot_count;     /**< Total boots (loaded from NVS on first boot) */
        uint32_t first_boot_time;      /**< Timestamp of first boot (from RTC clock) */
        uint32_t last_boot_time;       /**< Timestamp of most recent boot */
        uint32_t total_sleep_time_s;   /**< Cumulative sleep time in seconds */
        uint32_t total_active_time_ms; /**< Cumulative active time in milliseconds */
        uint32_t data_send_count;      /**< Number of successful data transmissions */
        uint32_t nvs_flush_count;      /**< Number of NVS flush operations */
    } rtcstore_metadata_t;

    /**
     * @brief Sensor data buffer state
     *
     * Manages the circular buffer of sensor readings in RTC memory.
     */
    typedef struct
    {
        uint32_t count;          /**< Number of valid readings in buffer */
        uint32_t write_index;    /**< Next write position (circular) */
        uint32_t total_readings; /**< Total readings taken (may exceed buffer size) */
        bool buffer_full;        /**< Flag indicating buffer has wrapped */
        uint32_t crc32;          /**< CRC32 of the sensor data array */
    } rtcstore_buffer_state_t;

    /**
     * @brief Wake-up history buffer state
     */
    typedef struct
    {
        uint32_t count;       /**< Number of entries in history */
        uint32_t write_index; /**< Next write position (circular) */
        uint32_t crc32;       /**< CRC32 of the history data array */
    } rtcstore_history_state_t;

    /**
     * @brief Complete RTC storage summary for reporting
     */
    typedef struct
    {
        rtcstore_metadata_t metadata; /**< Copy of current metadata */
        uint32_t sensor_count;        /**< Number of buffered sensor readings */
        uint32_t history_count;       /**< Number of wake-up history entries */
        bool data_valid;              /**< Overall data integrity status */
        bool sensor_crc_ok;           /**< Sensor buffer CRC check result */
        bool history_crc_ok;          /**< History buffer CRC check result */
        uint32_t rtc_memory_used;     /**< Estimated RTC memory usage in bytes */
        uint32_t rtc_memory_free;     /**< Estimated free RTC memory in bytes */
    } rtcstore_summary_t;

    /*============================================================================
     *                       FUNCTION DECLARATIONS
     *============================================================================*/

    /**
     * @brief Initialize the RTC storage subsystem
     *
     * Checks RTC memory validity using the magic number. On first boot
     * (power-on reset), initializes all RTC data structures and loads
     * persistent counters from NVS. On subsequent deep sleep wake-ups,
     * verifies data integrity via CRC32.
     *
     * @return
     *  - ESP_OK:               Success (data valid or freshly initialized)
     *  - ESP_ERR_INVALID_CRC:  Data corruption detected, storage was reset
     *  - ESP_FAIL:             NVS initialization failed
     */
    esp_err_t rtcstore_init(void);

    /**
     * @brief Increment the boot counter
     *
     * Should be called once per boot after rtcstore_init(). Updates both
     * the RTC boot counter and the NVS-backed total boot counter.
     *
     * @return uint32_t Current boot count (since last power-on)
     */
    uint32_t rtcstore_increment_boot_count(void);

    /**
     * @brief Get the current boot count
     *
     * @return uint32_t Boot count since last power-on reset
     */
    uint32_t rtcstore_get_boot_count(void);

    /**
     * @brief Get the total boot count across power cycles
     *
     * @return uint32_t Total boot count (from NVS)
     */
    uint32_t rtcstore_get_total_boot_count(void);

    /**
     * @brief Store a sensor reading in the RTC buffer
     *
     * Adds a sensor reading to the circular buffer in RTC memory.
     * Updates the CRC32 checksum after writing. If the buffer reaches
     * the flush threshold, the auto_flush flag is set.
     *
     * @param[in] reading Pointer to the sensor reading to store
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     *  - ESP_ERR_NO_MEM:       Buffer full and flush needed
     */
    esp_err_t rtcstore_add_sensor_reading(const rtcstore_sensor_reading_t *reading);

    /**
     * @brief Get all buffered sensor readings
     *
     * Copies all valid sensor readings from the RTC buffer to the
     * provided output array. Readings are returned in chronological order.
     *
     * @param[out] readings     Output array for sensor readings
     * @param[in]  max_readings Maximum number of readings to copy
     * @param[out] count        Actual number of readings copied
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer(s)
     *  - ESP_ERR_INVALID_CRC:  Buffer CRC check failed
     */
    esp_err_t rtcstore_get_sensor_readings(rtcstore_sensor_reading_t *readings,
                                           uint32_t max_readings,
                                           uint32_t *count);

    /**
     * @brief Get the number of buffered sensor readings
     *
     * @return uint32_t Number of sensor readings currently in the buffer
     */
    uint32_t rtcstore_get_sensor_count(void);

    /**
     * @brief Check if sensor buffer needs flushing
     *
     * Returns true if the buffer has reached the flush threshold and
     * data should be sent or saved to NVS.
     *
     * @return true  Buffer is at or above flush threshold
     * @return false Buffer has space for more readings
     */
    bool rtcstore_needs_flush(void);

    /**
     * @brief Clear the sensor reading buffer
     *
     * Resets the sensor buffer after successful data transmission
     * or NVS backup. Does not affect boot count or history.
     */
    void rtcstore_clear_sensor_buffer(void);

    /**
     * @brief Record a wake-up event in the history log
     *
     * Adds an entry to the circular wake-up history buffer.
     * Automatically captures timestamp and updates CRC32.
     *
     * @param[in] entry Pointer to the wake-up history entry
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     */
    esp_err_t rtcstore_add_wakeup_entry(const rtcstore_wakeup_entry_t *entry);

    /**
     * @brief Get the wake-up history log
     *
     * Copies all wake-up history entries from the RTC buffer.
     * Entries are returned in chronological order.
     *
     * @param[out] entries     Output array for history entries
     * @param[in]  max_entries Maximum number of entries to copy
     * @param[out] count       Actual number of entries copied
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer(s)
     */
    esp_err_t rtcstore_get_wakeup_history(rtcstore_wakeup_entry_t *entries,
                                          uint32_t max_entries,
                                          uint32_t *count);

    /**
     * @brief Flush all RTC data to NVS
     *
     * Saves the current sensor buffer, wake-up history, and metadata
     * to NVS flash for persistence across power cycles. Should be called
     * before data transmission or when the buffer is full.
     *
     * @return
     *  - ESP_OK:   Success
     *  - ESP_FAIL: NVS write failed
     */
    esp_err_t rtcstore_flush_to_nvs(void);

    /**
     * @brief Load previously saved data from NVS
     *
     * Restores sensor data and history from NVS. Typically called on
     * power-on reset to recover data from the last session.
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_NOT_FOUND:    No saved data in NVS
     *  - ESP_FAIL:             NVS read failed
     */
    esp_err_t rtcstore_load_from_nvs(void);

    /**
     * @brief Verify data integrity using CRC32
     *
     * Recalculates CRC32 checksums for both the sensor buffer and
     * wake-up history, comparing against stored values.
     *
     * @param[out] sensor_ok  True if sensor buffer CRC matches (can be NULL)
     * @param[out] history_ok True if history buffer CRC matches (can be NULL)
     *
     * @return true  All checked data is valid
     * @return false One or more CRC mismatches detected
     */
    bool rtcstore_verify_integrity(bool *sensor_ok, bool *history_ok);

    /**
     * @brief Get a complete storage summary
     *
     * Fills a summary structure with current metadata, buffer states,
     * integrity status, and memory usage estimates.
     *
     * @param[out] summary Pointer to summary structure to fill
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     */
    esp_err_t rtcstore_get_summary(rtcstore_summary_t *summary);

    /**
     * @brief Print storage status report to console
     *
     * Outputs a formatted report showing boot count, buffer usage,
     * data integrity status, and memory utilization.
     */
    void rtcstore_print_status(void);

    /**
     * @brief Update cumulative timing statistics
     *
     * Records the active time for the current wake cycle and adds
     * the sleep duration to the cumulative total.
     *
     * @param[in] active_time_ms  Time spent active in milliseconds
     * @param[in] sleep_duration_s Sleep duration that was configured in seconds
     */
    void rtcstore_update_timing(uint32_t active_time_ms, uint32_t sleep_duration_s);

    /**
     * @brief Reset all RTC storage data
     *
     * Clears all RTC memory structures and reinitializes with default
     * values. Does NOT clear NVS data. Use this for a soft reset.
     */
    void rtcstore_reset(void);

    /**
     * @brief Reset all data including NVS
     *
     * Performs a complete reset of both RTC memory and NVS stored data.
     * Use this for a factory reset of the storage subsystem.
     *
     * @return
     *  - ESP_OK:   Success
     *  - ESP_FAIL: NVS erase failed
     */
    esp_err_t rtcstore_factory_reset(void);

    /**
     * @brief Calculate CRC32 for a data buffer
     *
     * Utility function to compute CRC32 checksum using the standard
     * polynomial (0xEDB88320).
     *
     * @param[in] data Pointer to data buffer
     * @param[in] len  Length of data in bytes
     *
     * @return uint32_t Calculated CRC32 value
     */
    uint32_t rtcstore_calculate_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* RTC_STORAGE_H */
