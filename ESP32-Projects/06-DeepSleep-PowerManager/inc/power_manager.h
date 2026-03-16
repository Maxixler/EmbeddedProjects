/**
 * @file power_manager.h
 * @brief ESP32 Deep Sleep Power Manager - Header
 *
 * This module provides comprehensive power management for battery-powered
 * ESP32 IoT applications. It supports multiple sleep modes, various wake-up
 * sources, adaptive sleep intervals based on battery level, and GPIO
 * isolation for minimum deep sleep current consumption.
 *
 * Supported Power Modes:
 *  - ACTIVE:       Full CPU and peripherals running (~240 mA at 240 MHz)
 *  - MODEM_SLEEP:  WiFi/BT modem powered down (~20 mA)
 *  - LIGHT_SLEEP:  CPU paused, RTC and ULP running (~0.8 mA)
 *  - DEEP_SLEEP:   Only RTC controller + RTC memory (~10 uA)
 *  - HIBERNATE:    Only RTC timer running (~5 uA)
 *
 * Supported Wake-up Sources:
 *  - Timer:        Periodic wake-up at configurable intervals
 *  - EXT0:         Single RTC GPIO level trigger
 *  - EXT1:         Multiple RTC GPIOs with AND/OR logic
 *  - Touch Pad:    Capacitive touch sensor threshold trigger
 *  - ULP:          ULP coprocessor program trigger
 *
 * @author Embedded Systems Developer
 * @date 2026-03-16
 * @version 1.0.0
 *
 * @note This project targets ESP32 (not ESP32-S2/S3/C3) using ESP-IDF v5.x
 * @warning Deep sleep current depends heavily on GPIO configuration.
 *          Floating GPIOs can cause significant current leakage.
 */

#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

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
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"

/*============================================================================
 *                         MACRO DEFINITIONS
 *============================================================================*/

/**
 * @defgroup PWRMGR_DEFAULTS Default Configuration Values
 * @{
 */

/** @brief Default deep sleep duration in microseconds (60 seconds) */
#define PWRMGR_DEFAULT_SLEEP_DURATION_US (60ULL * 1000000ULL)

/** @brief Minimum deep sleep duration in microseconds (1 second) */
#define PWRMGR_MIN_SLEEP_DURATION_US (1ULL * 1000000ULL)

/** @brief Maximum deep sleep duration in microseconds (24 hours) */
#define PWRMGR_MAX_SLEEP_DURATION_US (24ULL * 3600ULL * 1000000ULL)

/** @brief Default wake-up GPIO pin for EXT0 (GPIO 33, RTC GPIO 8) */
#define PWRMGR_DEFAULT_WAKEUP_GPIO GPIO_NUM_33

/** @brief Default wake-up level for EXT0 (active LOW with pull-up) */
#define PWRMGR_DEFAULT_WAKEUP_LEVEL 0

/** @brief Default touch pad channel for wake-up */
#define PWRMGR_DEFAULT_TOUCH_PAD TOUCH_PAD_NUM7

/** @brief Touch pad wake-up threshold (lower = more sensitive) */
#define PWRMGR_TOUCH_THRESHOLD 400

/** @} */

/**
 * @defgroup PWRMGR_BATTERY Battery Monitoring Configuration
 * @{
 */

/** @brief ADC channel for battery voltage measurement (GPIO 34) */
#define PWRMGR_BATTERY_ADC_CHANNEL ADC_CHANNEL_6

/** @brief ADC unit for battery measurement */
#define PWRMGR_BATTERY_ADC_UNIT ADC_UNIT_1

/** @brief Battery voltage divider upper resistor (kohm) */
#define PWRMGR_VDIV_R1_KOHM 100

/** @brief Battery voltage divider lower resistor (kohm) */
#define PWRMGR_VDIV_R2_KOHM 100

/** @brief ADC reference voltage in millivolts */
#define PWRMGR_ADC_VREF_MV 1100

/** @brief Full battery voltage in millivolts (4.2V Li-Ion) */
#define PWRMGR_BATTERY_FULL_MV 4200

/** @brief Empty battery voltage in millivolts (3.0V Li-Ion) */
#define PWRMGR_BATTERY_EMPTY_MV 3000

/** @brief Critical battery voltage in millivolts (2.8V) */
#define PWRMGR_BATTERY_CRITICAL_MV 2800

/** @brief Low battery percentage threshold for extended sleep */
#define PWRMGR_BATTERY_LOW_PERCENT 20

/** @brief Critical battery percentage threshold for indefinite sleep */
#define PWRMGR_BATTERY_CRITICAL_PERCENT 5

/** @brief Number of ADC samples to average for battery reading */
#define PWRMGR_BATTERY_ADC_SAMPLES 64

/** @} */

/**
 * @defgroup PWRMGR_ADAPTIVE Adaptive Sleep Configuration
 * @{
 */

/** @brief Sleep interval at full battery (seconds) */
#define PWRMGR_SLEEP_INTERVAL_FULL_S 60

/** @brief Sleep interval at low battery (seconds) */
#define PWRMGR_SLEEP_INTERVAL_LOW_S 300

/** @brief Sleep interval at critical battery (seconds) */
#define PWRMGR_SLEEP_INTERVAL_CRITICAL_S 900

/** @brief Number of wake cycles before data transmission */
#define PWRMGR_CYCLES_BEFORE_SEND 5

/** @brief Maximum active time before forced sleep (milliseconds) */
#define PWRMGR_MAX_ACTIVE_TIME_MS 5000

/** @} */

/**
 * @defgroup PWRMGR_CURRENT Current Consumption Estimates
 * @{
 */

/** @brief Estimated active mode current (mA) at 240 MHz with WiFi */
#define PWRMGR_CURRENT_ACTIVE_MA 240.0f

/** @brief Estimated active mode current (mA) at 240 MHz without WiFi */
#define PWRMGR_CURRENT_ACTIVE_NO_WIFI_MA 50.0f

/** @brief Estimated modem sleep current (mA) */
#define PWRMGR_CURRENT_MODEM_SLEEP_MA 20.0f

/** @brief Estimated light sleep current (mA) */
#define PWRMGR_CURRENT_LIGHT_SLEEP_MA 0.8f

/** @brief Estimated deep sleep current (uA) */
#define PWRMGR_CURRENT_DEEP_SLEEP_UA 10.0f

/** @brief Estimated hibernate current (uA) */
#define PWRMGR_CURRENT_HIBERNATE_UA 5.0f

    /** @} */

    /*============================================================================
     *                         TYPE DEFINITIONS
     *============================================================================*/

    /**
     * @brief Power mode enumeration
     *
     * Defines all available power modes from highest to lowest power consumption.
     * Each mode trades off between functionality and current draw.
     */
    typedef enum
    {
        PWRMGR_MODE_ACTIVE = 0,      /**< Full operation, all peripherals available */
        PWRMGR_MODE_MODEM_SLEEP = 1, /**< CPU active, WiFi/BT modem off */
        PWRMGR_MODE_LIGHT_SLEEP = 2, /**< CPU paused, fast wake-up, memory retained */
        PWRMGR_MODE_DEEP_SLEEP = 3,  /**< Only RTC + RTC memory, full reboot on wake */
        PWRMGR_MODE_HIBERNATE = 4,   /**< Minimal: only RTC timer, no RTC memory */
    } pwrmgr_mode_t;

    /**
     * @brief Wake-up source flags (bitmask)
     *
     * Multiple wake-up sources can be enabled simultaneously by ORing flags.
     * On wake-up, the actual cause can be determined via pwrmgr_get_wakeup_cause().
     */
    typedef enum
    {
        PWRMGR_WAKE_NONE = 0x00,     /**< No wake-up source (indefinite sleep) */
        PWRMGR_WAKE_TIMER = 0x01,    /**< RTC timer wake-up */
        PWRMGR_WAKE_EXT0 = 0x02,     /**< External wake-up on single GPIO (EXT0) */
        PWRMGR_WAKE_EXT1 = 0x04,     /**< External wake-up on multiple GPIOs (EXT1) */
        PWRMGR_WAKE_TOUCHPAD = 0x08, /**< Touch pad wake-up */
        PWRMGR_WAKE_ULP = 0x10,      /**< ULP coprocessor wake-up */
        PWRMGR_WAKE_GPIO = 0x20,     /**< GPIO wake-up (light sleep only) */
        PWRMGR_WAKE_ALL = 0x3F,      /**< All wake-up sources enabled */
    } pwrmgr_wake_source_t;

    /**
     * @brief Wake-up cause enumeration
     *
     * Returned by pwrmgr_get_wakeup_cause() to identify what triggered the
     * most recent wake-up from sleep.
     */
    typedef enum
    {
        PWRMGR_WAKEUP_UNDEFINED = 0, /**< Not a wake-up from sleep (power-on/reset) */
        PWRMGR_WAKEUP_TIMER = 1,     /**< Woke up due to RTC timer expiry */
        PWRMGR_WAKEUP_EXT0 = 2,      /**< Woke up due to EXT0 (single GPIO) */
        PWRMGR_WAKEUP_EXT1 = 3,      /**< Woke up due to EXT1 (multiple GPIO) */
        PWRMGR_WAKEUP_TOUCHPAD = 4,  /**< Woke up due to touch pad activation */
        PWRMGR_WAKEUP_ULP = 5,       /**< Woke up due to ULP coprocessor */
        PWRMGR_WAKEUP_GPIO = 6,      /**< Woke up due to GPIO (light sleep) */
        PWRMGR_WAKEUP_UNKNOWN = 7,   /**< Unknown wake-up cause */
    } pwrmgr_wakeup_cause_t;

    /**
     * @brief EXT1 wake-up logic mode
     *
     * When using EXT1 (multiple GPIO) wake-up, determines whether ANY pin
     * or ALL pins must be at the trigger level to generate a wake-up event.
     */
    typedef enum
    {
        PWRMGR_EXT1_ANY_HIGH = ESP_EXT1_WAKEUP_ANY_HIGH, /**< Wake if ANY pin is HIGH */
        PWRMGR_EXT1_ALL_LOW = ESP_EXT1_WAKEUP_ALL_LOW,   /**< Wake if ALL pins are LOW */
    } pwrmgr_ext1_mode_t;

    /**
     * @brief Battery status enumeration
     *
     * Indicates the current battery charge level category, used for
     * adaptive sleep interval calculation.
     */
    typedef enum
    {
        PWRMGR_BAT_FULL = 0,     /**< Battery >= 80% */
        PWRMGR_BAT_NORMAL = 1,   /**< Battery 20-79% */
        PWRMGR_BAT_LOW = 2,      /**< Battery 5-19% */
        PWRMGR_BAT_CRITICAL = 3, /**< Battery < 5% */
    } pwrmgr_battery_status_t;

    /**
     * @brief EXT0 wake-up configuration
     */
    typedef struct
    {
        gpio_num_t gpio; /**< RTC GPIO pin number */
        int level;       /**< Trigger level (0 = LOW, 1 = HIGH) */
    } pwrmgr_ext0_config_t;

    /**
     * @brief EXT1 wake-up configuration
     */
    typedef struct
    {
        uint64_t gpio_mask;      /**< Bitmask of RTC GPIO pins */
        pwrmgr_ext1_mode_t mode; /**< Logic mode (ANY_HIGH or ALL_LOW) */
    } pwrmgr_ext1_config_t;

    /**
     * @brief Touch pad wake-up configuration
     */
    typedef struct
    {
        touch_pad_t touch_num; /**< Touch pad channel number */
        uint16_t threshold;    /**< Wake-up threshold value */
    } pwrmgr_touch_config_t;

    /**
     * @brief Complete power manager configuration structure
     *
     * Encapsulates all settings for sleep mode, wake-up sources, battery
     * monitoring, and adaptive behavior. Initialize with pwrmgr_config_default()
     * before modifying individual fields.
     */
    typedef struct
    {
        /** @brief Target power mode */
        pwrmgr_mode_t mode;

        /** @brief Bitmask of enabled wake-up sources (OR of pwrmgr_wake_source_t) */
        uint32_t wake_sources;

        /** @brief Timer wake-up duration in microseconds */
        uint64_t sleep_duration_us;

        /** @brief EXT0 (single GPIO) wake-up configuration */
        pwrmgr_ext0_config_t ext0;

        /** @brief EXT1 (multiple GPIO) wake-up configuration */
        pwrmgr_ext1_config_t ext1;

        /** @brief Touch pad wake-up configuration */
        pwrmgr_touch_config_t touch;

        /** @brief Enable GPIO isolation for minimum sleep current */
        bool isolate_gpio;

        /** @brief Enable adaptive sleep interval based on battery level */
        bool adaptive_sleep;

        /** @brief Enable battery voltage monitoring */
        bool monitor_battery;

        /** @brief Number of wake cycles before data transmission */
        uint32_t cycles_before_send;

        /** @brief Maximum active time before forced sleep (ms) */
        uint32_t max_active_time_ms;
    } pwrmgr_config_t;

    /**
     * @brief Battery measurement result
     */
    typedef struct
    {
        uint32_t raw_adc;               /**< Raw ADC reading (12-bit) */
        uint32_t voltage_mv;            /**< Calculated battery voltage in mV */
        uint8_t percentage;             /**< Estimated charge percentage (0-100) */
        pwrmgr_battery_status_t status; /**< Battery status category */
    } pwrmgr_battery_info_t;

    /**
     * @brief Current consumption estimation per mode
     */
    typedef struct
    {
        float active_ma;      /**< Active mode current (mA) */
        float modem_sleep_ma; /**< Modem sleep current (mA) */
        float light_sleep_ma; /**< Light sleep current (mA) */
        float deep_sleep_ua;  /**< Deep sleep current (uA) */
        float hibernate_ua;   /**< Hibernate current (uA) */
        float avg_current_ua; /**< Calculated average current (uA) */
        float estimated_days; /**< Estimated battery life (days) */
    } pwrmgr_current_estimate_t;

    /*============================================================================
     *                       FUNCTION DECLARATIONS
     *============================================================================*/

    /**
     * @brief Get a default power manager configuration
     *
     * Returns a configuration structure populated with safe default values:
     *  - Mode: DEEP_SLEEP
     *  - Wake source: TIMER only
     *  - Sleep duration: 60 seconds
     *  - GPIO isolation: enabled
     *  - Adaptive sleep: enabled
     *  - Battery monitoring: enabled
     *
     * @return pwrmgr_config_t Default configuration structure
     *
     * @code
     * pwrmgr_config_t config = pwrmgr_config_default();
     * config.sleep_duration_us = 30 * 1000000ULL; // 30 seconds
     * config.wake_sources |= PWRMGR_WAKE_EXT0;    // Add button wake-up
     * @endcode
     */
    pwrmgr_config_t pwrmgr_config_default(void);

    /**
     * @brief Initialize the power manager subsystem
     *
     * Sets up all configured wake-up sources, initializes the battery ADC
     * (if enabled), and prepares GPIO states for sleep. Must be called once
     * during startup before any other power manager function.
     *
     * @param[in] config Pointer to power manager configuration
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL config or invalid parameters
     *  - ESP_ERR_INVALID_STATE: Already initialized
     *  - ESP_FAIL:             Hardware initialization failed
     */
    esp_err_t pwrmgr_init(const pwrmgr_config_t *config);

    /**
     * @brief Enter the configured sleep mode
     *
     * Configures all enabled wake-up sources and enters the selected power
     * mode. For deep sleep and hibernate modes, this function does not return;
     * the CPU will reset on wake-up and execution resumes from app_main().
     *
     * For light sleep, this function returns after wake-up.
     *
     * @return
     *  - ESP_OK:           Successfully returned from light sleep
     *  - ESP_FAIL:         Failed to enter sleep mode
     *
     * @warning For DEEP_SLEEP and HIBERNATE modes, this function never returns.
     *          Ensure all data is saved before calling.
     *
     * @note Call pwrmgr_init() before this function.
     */
    esp_err_t pwrmgr_enter_sleep(void);

    /**
     * @brief Enter deep sleep with specific duration
     *
     * Convenience function to enter deep sleep for a specific duration
     * without modifying the stored configuration. Uses currently configured
     * wake-up sources in addition to the timer.
     *
     * @param[in] duration_us Sleep duration in microseconds
     *
     * @warning This function does not return. CPU resets on wake-up.
     */
    void pwrmgr_deep_sleep_for(uint64_t duration_us);

    /**
     * @brief Enter indefinite deep sleep with GPIO wake-up only
     *
     * Enters deep sleep with no timer wake-up. Only an external GPIO event
     * can wake the device. Used for critical battery situations where the
     * device should remain dormant until physically attended.
     *
     * @param[in] gpio_num RTC GPIO pin to use for wake-up
     * @param[in] level    Trigger level (0 = LOW, 1 = HIGH)
     *
     * @warning This function does not return. CPU resets on wake-up.
     */
    void pwrmgr_enter_indefinite_sleep(gpio_num_t gpio_num, int level);

    /**
     * @brief Get the cause of the most recent wake-up
     *
     * Determines why the ESP32 woke from sleep. On first boot (power-on reset
     * or software reset), returns PWRMGR_WAKEUP_UNDEFINED.
     *
     * @return pwrmgr_wakeup_cause_t The wake-up cause
     *
     * @note Can be called without pwrmgr_init() as it only reads hardware
     *       registers.
     */
    pwrmgr_wakeup_cause_t pwrmgr_get_wakeup_cause(void);

    /**
     * @brief Get a human-readable string for a wake-up cause
     *
     * @param[in] cause Wake-up cause value
     *
     * @return const char* Static string describing the wake-up cause
     */
    const char *pwrmgr_wakeup_cause_to_str(pwrmgr_wakeup_cause_t cause);

    /**
     * @brief Get the GPIO number that triggered EXT0/EXT1 wake-up
     *
     * After an EXT0 or EXT1 wake-up, returns the specific GPIO(s) that
     * triggered the event. For EXT1, returns a bitmask.
     *
     * @param[out] gpio_mask Pointer to store the GPIO bitmask
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     *  - ESP_ERR_INVALID_STATE: Last wake-up was not GPIO-triggered
     */
    esp_err_t pwrmgr_get_wakeup_gpio(uint64_t *gpio_mask);

    /**
     * @brief Get the touch pad that triggered wake-up
     *
     * After a touch pad wake-up, returns the specific touch pad channel
     * that triggered the event.
     *
     * @param[out] touch_num Pointer to store the touch pad number
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     *  - ESP_ERR_INVALID_STATE: Last wake-up was not touchpad-triggered
     */
    esp_err_t pwrmgr_get_wakeup_touchpad(touch_pad_t *touch_num);

    /**
     * @brief Read battery voltage and calculate charge level
     *
     * Performs multiple ADC readings of the battery voltage through the
     * voltage divider, averages them, applies calibration, and calculates
     * the estimated charge percentage.
     *
     * @param[out] info Pointer to battery info structure to fill
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     *  - ESP_ERR_INVALID_STATE: Battery monitoring not enabled
     *  - ESP_FAIL:             ADC read failed
     */
    esp_err_t pwrmgr_read_battery(pwrmgr_battery_info_t *info);

    /**
     * @brief Get adaptive sleep duration based on battery level
     *
     * Calculates the optimal sleep interval considering the current battery
     * level. Lower battery = longer sleep intervals to conserve power.
     *
     * @param[in] battery_info Current battery information
     *
     * @return uint64_t Recommended sleep duration in microseconds
     */
    uint64_t pwrmgr_get_adaptive_sleep_duration(const pwrmgr_battery_info_t *battery_info);

    /**
     * @brief Check if battery level is critical
     *
     * Returns true if the battery voltage is below the critical threshold,
     * indicating the device should enter indefinite deep sleep.
     *
     * @param[in] battery_info Current battery information
     *
     * @return true  Battery is critically low
     * @return false Battery level is acceptable
     */
    bool pwrmgr_is_battery_critical(const pwrmgr_battery_info_t *battery_info);

    /**
     * @brief Isolate all GPIOs for minimum sleep current
     *
     * Configures all GPIOs to a low-power state before entering deep sleep.
     * Preserves only the RTC GPIOs configured as wake-up sources.
     *
     * This can reduce deep sleep current from ~150 uA to ~10 uA by
     * eliminating leakage through floating GPIOs.
     */
    void pwrmgr_isolate_gpio(void);

    /**
     * @brief Configure RTC GPIO for wake-up
     *
     * Sets up an RTC GPIO pin with the appropriate pull-up/pull-down
     * resistors for use as a deep sleep wake-up source.
     *
     * @param[in] gpio_num RTC GPIO pin number
     * @param[in] level    Expected trigger level (0 or 1)
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  Not an RTC GPIO pin
     */
    esp_err_t pwrmgr_configure_rtc_gpio(gpio_num_t gpio_num, int level);

    /**
     * @brief Estimate current consumption for each power mode
     *
     * Calculates estimated current draw for the current configuration,
     * including average current considering duty cycle (active time vs
     * sleep time) and estimated battery life.
     *
     * @param[out] estimate Pointer to current estimate structure
     * @param[in]  battery_capacity_mah Battery capacity in milliamp-hours
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  NULL pointer
     */
    esp_err_t pwrmgr_estimate_current(pwrmgr_current_estimate_t *estimate,
                                      uint32_t battery_capacity_mah);

    /**
     * @brief Print detailed power consumption report
     *
     * Outputs a formatted report to the serial console showing current
     * consumption estimates, duty cycle analysis, and battery life
     * projections.
     *
     * @param[in] battery_capacity_mah Battery capacity in milliamp-hours
     */
    void pwrmgr_print_power_report(uint32_t battery_capacity_mah);

    /**
     * @brief Set the power mode
     *
     * Updates the target power mode for the next sleep cycle.
     *
     * @param[in] mode Target power mode
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  Invalid mode value
     */
    esp_err_t pwrmgr_set_mode(pwrmgr_mode_t mode);

    /**
     * @brief Update sleep duration
     *
     * Changes the timer wake-up interval for subsequent sleep cycles.
     *
     * @param[in] duration_us New sleep duration in microseconds
     *
     * @return
     *  - ESP_OK:               Success
     *  - ESP_ERR_INVALID_ARG:  Duration out of valid range
     */
    esp_err_t pwrmgr_set_sleep_duration(uint64_t duration_us);

    /**
     * @brief Prepare ULP coprocessor for wake-up
     *
     * Initializes the ULP coprocessor with a simple monitoring program
     * that can wake the main CPU when a condition is met. This is a
     * placeholder for user-defined ULP programs.
     *
     * @return
     *  - ESP_OK:   Success
     *  - ESP_FAIL: ULP initialization failed
     *
     * @note Requires ULP coprocessor support enabled in sdkconfig.
     */
    esp_err_t pwrmgr_prepare_ulp(void);

    /**
     * @brief Deinitialize the power manager
     *
     * Releases any allocated resources. Not typically needed as the system
     * resets on deep sleep wake-up, but useful for clean shutdown.
     */
    void pwrmgr_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* POWER_MANAGER_H */
