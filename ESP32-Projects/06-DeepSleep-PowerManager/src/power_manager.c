/**
 * @file power_manager.c
 * @brief ESP32 Deep Sleep Power Manager - Implementation
 *
 * Implements comprehensive power management for battery-powered ESP32
 * IoT applications. Handles deep sleep configuration, wake-up source
 * management, battery monitoring via ADC, adaptive sleep intervals,
 * and GPIO isolation for minimum sleep current.
 *
 * @author Embedded Systems Developer
 * @date 2026-03-16
 * @version 1.0.0
 */

/*============================================================================
 *                              INCLUDES
 *============================================================================*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/rtc.h"
#include "soc/sens_reg.h"

#include "power_manager.h"

/*============================================================================
 *                          PRIVATE DEFINITIONS
 *============================================================================*/

/** @brief Module log tag */
static const char *TAG = "PWRMGR";

/** @brief Voltage divider ratio: Vbat = Vadc * (R1 + R2) / R2 */
#define VDIV_RATIO ((float)(PWRMGR_VDIV_R1_KOHM + PWRMGR_VDIV_R2_KOHM) / \
                    (float)PWRMGR_VDIV_R2_KOHM)

/*============================================================================
 *                         PRIVATE VARIABLES
 *============================================================================*/

/** @brief Module initialization flag */
static bool s_initialized = false;

/** @brief Stored power manager configuration */
static pwrmgr_config_t s_config;

/** @brief ADC handle for battery monitoring */
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

/** @brief ADC calibration handle */
static adc_cali_handle_t s_adc_cali_handle = NULL;

/** @brief Flag indicating ADC calibration is available */
static bool s_adc_cali_valid = false;

/*============================================================================
 *                     PRIVATE FUNCTION PROTOTYPES
 *============================================================================*/

static esp_err_t prv_configure_timer_wakeup(void);
static esp_err_t prv_configure_ext0_wakeup(void);
static esp_err_t prv_configure_ext1_wakeup(void);
static esp_err_t prv_configure_touch_wakeup(void);
static esp_err_t prv_init_battery_adc(void);
static void prv_deinit_battery_adc(void);
static uint32_t prv_read_adc_averaged(uint32_t samples);
static uint8_t prv_voltage_to_percentage(uint32_t voltage_mv);

/*============================================================================
 *                     PUBLIC FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Get a default power manager configuration
 */
pwrmgr_config_t pwrmgr_config_default(void)
{
    pwrmgr_config_t config = {
        .mode = PWRMGR_MODE_DEEP_SLEEP,
        .wake_sources = PWRMGR_WAKE_TIMER,
        .sleep_duration_us = PWRMGR_DEFAULT_SLEEP_DURATION_US,
        .ext0 = {
            .gpio = PWRMGR_DEFAULT_WAKEUP_GPIO,
            .level = PWRMGR_DEFAULT_WAKEUP_LEVEL,
        },
        .ext1 = {
            .gpio_mask = (1ULL << GPIO_NUM_32) | (1ULL << GPIO_NUM_33),
            .mode = PWRMGR_EXT1_ANY_HIGH,
        },
        .touch = {
            .touch_num = PWRMGR_DEFAULT_TOUCH_PAD,
            .threshold = PWRMGR_TOUCH_THRESHOLD,
        },
        .isolate_gpio = true,
        .adaptive_sleep = true,
        .monitor_battery = true,
        .cycles_before_send = PWRMGR_CYCLES_BEFORE_SEND,
        .max_active_time_ms = PWRMGR_MAX_ACTIVE_TIME_MS,
    };

    return config;
}

/**
 * @brief Initialize the power manager subsystem
 */
esp_err_t pwrmgr_init(const pwrmgr_config_t *config)
{
    if (config == NULL)
    {
        ESP_LOGE(TAG, "Configuration pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized)
    {
        ESP_LOGW(TAG, "Power manager already initialized, reinitializing");
        pwrmgr_deinit();
    }

    /* Store configuration */
    memcpy(&s_config, config, sizeof(pwrmgr_config_t));

    /* Validate sleep duration */
    if (s_config.sleep_duration_us < PWRMGR_MIN_SLEEP_DURATION_US)
    {
        ESP_LOGW(TAG, "Sleep duration too short, clamping to minimum");
        s_config.sleep_duration_us = PWRMGR_MIN_SLEEP_DURATION_US;
    }
    if (s_config.sleep_duration_us > PWRMGR_MAX_SLEEP_DURATION_US)
    {
        ESP_LOGW(TAG, "Sleep duration too long, clamping to maximum");
        s_config.sleep_duration_us = PWRMGR_MAX_SLEEP_DURATION_US;
    }

    /* Initialize battery ADC if monitoring is enabled */
    if (s_config.monitor_battery)
    {
        esp_err_t ret = prv_init_battery_adc();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to initialize battery ADC: %s",
                     esp_err_to_name(ret));
            /* Continue without battery monitoring */
            s_config.monitor_battery = false;
        }
    }

    /* Configure wake-up sources */
    ESP_LOGI(TAG, "Configuring wake-up sources (mask: 0x%02lX)",
             (unsigned long)s_config.wake_sources);

    if (s_config.wake_sources & PWRMGR_WAKE_TIMER)
    {
        esp_err_t ret = prv_configure_timer_wakeup();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Timer wake-up configuration failed");
            return ret;
        }
    }

    if (s_config.wake_sources & PWRMGR_WAKE_EXT0)
    {
        esp_err_t ret = prv_configure_ext0_wakeup();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "EXT0 wake-up configuration failed");
            return ret;
        }
    }

    if (s_config.wake_sources & PWRMGR_WAKE_EXT1)
    {
        esp_err_t ret = prv_configure_ext1_wakeup();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "EXT1 wake-up configuration failed");
            return ret;
        }
    }

    if (s_config.wake_sources & PWRMGR_WAKE_TOUCHPAD)
    {
        esp_err_t ret = prv_configure_touch_wakeup();
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Touch pad wake-up configuration failed");
            return ret;
        }
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Power manager initialized successfully");
    ESP_LOGI(TAG, "  Mode: %s",
             s_config.mode == PWRMGR_MODE_ACTIVE ? "ACTIVE" : s_config.mode == PWRMGR_MODE_MODEM_SLEEP ? "MODEM_SLEEP"
                                                          : s_config.mode == PWRMGR_MODE_LIGHT_SLEEP   ? "LIGHT_SLEEP"
                                                          : s_config.mode == PWRMGR_MODE_DEEP_SLEEP    ? "DEEP_SLEEP"
                                                          : s_config.mode == PWRMGR_MODE_HIBERNATE     ? "HIBERNATE"
                                                                                                       : "UNKNOWN");
    ESP_LOGI(TAG, "  Sleep duration: %llu ms",
             s_config.sleep_duration_us / 1000ULL);
    ESP_LOGI(TAG, "  GPIO isolation: %s",
             s_config.isolate_gpio ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  Adaptive sleep: %s",
             s_config.adaptive_sleep ? "enabled" : "disabled");
    ESP_LOGI(TAG, "  Battery monitor: %s",
             s_config.monitor_battery ? "enabled" : "disabled");

    return ESP_OK;
}

/**
 * @brief Enter the configured sleep mode
 */
esp_err_t pwrmgr_enter_sleep(void)
{
    if (!s_initialized)
    {
        ESP_LOGE(TAG, "Power manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Preparing to enter sleep mode...");

    /* Reconfigure wake-up sources (may have been modified) */
    if (s_config.wake_sources & PWRMGR_WAKE_TIMER)
    {
        prv_configure_timer_wakeup();
    }

    switch (s_config.mode)
    {
    case PWRMGR_MODE_ACTIVE:
        ESP_LOGW(TAG, "Active mode requested, not entering sleep");
        return ESP_OK;

    case PWRMGR_MODE_MODEM_SLEEP:
        ESP_LOGI(TAG, "Entering modem sleep (WiFi/BT will be disabled)");
        /* Modem sleep is automatic when WiFi is connected with */
        /* power save mode enabled. Here we just disable WiFi.  */
        ESP_LOGI(TAG, "Modem sleep: disable WiFi/BT for power saving");
        return ESP_OK;

    case PWRMGR_MODE_LIGHT_SLEEP:
        ESP_LOGI(TAG, "Entering light sleep...");
        esp_err_t ret = esp_light_sleep_start();
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Woke up from light sleep");
        }
        else
        {
            ESP_LOGE(TAG, "Light sleep failed: %s", esp_err_to_name(ret));
        }
        return ret;

    case PWRMGR_MODE_DEEP_SLEEP:
        ESP_LOGI(TAG, "Entering deep sleep for %llu seconds...",
                 s_config.sleep_duration_us / 1000000ULL);

        /* Isolate GPIOs for minimum current if configured */
        if (s_config.isolate_gpio)
        {
            pwrmgr_isolate_gpio();
        }

        /* Deinitialize ADC before sleep */
        if (s_config.monitor_battery)
        {
            prv_deinit_battery_adc();
        }

        /* Enter deep sleep - does not return */
        esp_deep_sleep_start();
        /* Never reached */
        return ESP_OK;

    case PWRMGR_MODE_HIBERNATE:
        ESP_LOGI(TAG, "Entering hibernate mode (minimal power)...");

        /* Disable all RTC memory in hibernate for minimum current */
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

        if (s_config.isolate_gpio)
        {
            pwrmgr_isolate_gpio();
        }

        if (s_config.monitor_battery)
        {
            prv_deinit_battery_adc();
        }

        /* Enter deep sleep (hibernate) - does not return */
        esp_deep_sleep_start();
        /* Never reached */
        return ESP_OK;

    default:
        ESP_LOGE(TAG, "Invalid power mode: %d", s_config.mode);
        return ESP_ERR_INVALID_ARG;
    }
}

/**
 * @brief Enter deep sleep with specific duration
 */
void pwrmgr_deep_sleep_for(uint64_t duration_us)
{
    ESP_LOGI(TAG, "Entering deep sleep for %llu us", duration_us);

    /* Configure timer wake-up with the specified duration */
    esp_sleep_enable_timer_wakeup(duration_us);

    /* Isolate GPIOs if configured */
    if (s_initialized && s_config.isolate_gpio)
    {
        pwrmgr_isolate_gpio();
    }

    /* Deinitialize ADC */
    if (s_initialized && s_config.monitor_battery)
    {
        prv_deinit_battery_adc();
    }

    /* Enter deep sleep - does not return */
    esp_deep_sleep_start();
}

/**
 * @brief Enter indefinite deep sleep with GPIO wake-up only
 */
void pwrmgr_enter_indefinite_sleep(gpio_num_t gpio_num, int level)
{
    ESP_LOGW(TAG, "===========================================");
    ESP_LOGW(TAG, "  ENTERING INDEFINITE DEEP SLEEP");
    ESP_LOGW(TAG, "  Wake-up: GPIO %d, level %d", gpio_num, level);
    ESP_LOGW(TAG, "  Battery critically low or manual request");
    ESP_LOGW(TAG, "===========================================");

    /* Disable all other wake-up sources */
    /* Only EXT0 GPIO wake-up will be active */
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    /* Configure EXT0 wake-up on the specified GPIO */
    pwrmgr_configure_rtc_gpio(gpio_num, level);
    esp_sleep_enable_ext0_wakeup(gpio_num, level);

    /* Isolate all other GPIOs */
    pwrmgr_isolate_gpio();

    /* Deinitialize ADC */
    if (s_initialized && s_config.monitor_battery)
    {
        prv_deinit_battery_adc();
    }

    /* Small delay to ensure log output is flushed */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Enter deep sleep - does not return */
    esp_deep_sleep_start();
}

/**
 * @brief Get the cause of the most recent wake-up
 */
pwrmgr_wakeup_cause_t pwrmgr_get_wakeup_cause(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        return PWRMGR_WAKEUP_UNDEFINED;
    case ESP_SLEEP_WAKEUP_TIMER:
        return PWRMGR_WAKEUP_TIMER;
    case ESP_SLEEP_WAKEUP_EXT0:
        return PWRMGR_WAKEUP_EXT0;
    case ESP_SLEEP_WAKEUP_EXT1:
        return PWRMGR_WAKEUP_EXT1;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        return PWRMGR_WAKEUP_TOUCHPAD;
    case ESP_SLEEP_WAKEUP_ULP:
        return PWRMGR_WAKEUP_ULP;
    case ESP_SLEEP_WAKEUP_GPIO:
        return PWRMGR_WAKEUP_GPIO;
    default:
        return PWRMGR_WAKEUP_UNKNOWN;
    }
}

/**
 * @brief Get a human-readable string for a wake-up cause
 */
const char *pwrmgr_wakeup_cause_to_str(pwrmgr_wakeup_cause_t cause)
{
    switch (cause)
    {
    case PWRMGR_WAKEUP_UNDEFINED:
        return "Power-on reset / Software reset";
    case PWRMGR_WAKEUP_TIMER:
        return "RTC timer";
    case PWRMGR_WAKEUP_EXT0:
        return "EXT0 (single GPIO)";
    case PWRMGR_WAKEUP_EXT1:
        return "EXT1 (multiple GPIO)";
    case PWRMGR_WAKEUP_TOUCHPAD:
        return "Touch pad";
    case PWRMGR_WAKEUP_ULP:
        return "ULP coprocessor";
    case PWRMGR_WAKEUP_GPIO:
        return "GPIO (light sleep)";
    case PWRMGR_WAKEUP_UNKNOWN:
    default:
        return "Unknown";
    }
}

/**
 * @brief Get the GPIO number that triggered EXT0/EXT1 wake-up
 */
esp_err_t pwrmgr_get_wakeup_gpio(uint64_t *gpio_mask)
{
    if (gpio_mask == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    pwrmgr_wakeup_cause_t cause = pwrmgr_get_wakeup_cause();

    if (cause == PWRMGR_WAKEUP_EXT0)
    {
        /* EXT0 uses a single GPIO configured in ext0 settings */
        *gpio_mask = (1ULL << s_config.ext0.gpio);
        return ESP_OK;
    }
    else if (cause == PWRMGR_WAKEUP_EXT1)
    {
        /* EXT1 can identify which GPIO(s) triggered the wake-up */
        *gpio_mask = esp_sleep_get_ext1_wakeup_status();
        return ESP_OK;
    }

    *gpio_mask = 0;
    return ESP_ERR_INVALID_STATE;
}

/**
 * @brief Get the touch pad that triggered wake-up
 */
esp_err_t pwrmgr_get_wakeup_touchpad(touch_pad_t *touch_num)
{
    if (touch_num == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    pwrmgr_wakeup_cause_t cause = pwrmgr_get_wakeup_cause();

    if (cause != PWRMGR_WAKEUP_TOUCHPAD)
    {
        *touch_num = TOUCH_PAD_MAX;
        return ESP_ERR_INVALID_STATE;
    }

    *touch_num = esp_sleep_get_touchpad_wakeup_status();
    return ESP_OK;
}

/**
 * @brief Read battery voltage and calculate charge level
 */
esp_err_t pwrmgr_read_battery(pwrmgr_battery_info_t *info)
{
    if (info == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_config.monitor_battery || s_adc_handle == NULL)
    {
        ESP_LOGE(TAG, "Battery monitoring not enabled or ADC not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Read averaged ADC value */
    info->raw_adc = prv_read_adc_averaged(PWRMGR_BATTERY_ADC_SAMPLES);

    /* Convert to voltage using calibration if available */
    int voltage_mv = 0;
    if (s_adc_cali_valid && s_adc_cali_handle != NULL)
    {
        esp_err_t ret = adc_cali_raw_to_voltage(s_adc_cali_handle,
                                                (int)info->raw_adc,
                                                &voltage_mv);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "ADC calibration conversion failed, using raw");
            /* Fallback: approximate conversion for 12-bit ADC, 0-3.3V */
            voltage_mv = (int)((info->raw_adc * 3300) / 4095);
        }
    }
    else
    {
        /* No calibration: approximate conversion */
        voltage_mv = (int)((info->raw_adc * 3300) / 4095);
    }

    /* Apply voltage divider ratio to get actual battery voltage */
    info->voltage_mv = (uint32_t)(voltage_mv * VDIV_RATIO);

    /* Calculate percentage */
    info->percentage = prv_voltage_to_percentage(info->voltage_mv);

    /* Determine status category */
    if (info->percentage >= 80)
    {
        info->status = PWRMGR_BAT_FULL;
    }
    else if (info->percentage >= PWRMGR_BATTERY_LOW_PERCENT)
    {
        info->status = PWRMGR_BAT_NORMAL;
    }
    else if (info->percentage >= PWRMGR_BATTERY_CRITICAL_PERCENT)
    {
        info->status = PWRMGR_BAT_LOW;
    }
    else
    {
        info->status = PWRMGR_BAT_CRITICAL;
    }

    ESP_LOGI(TAG, "Battery: %lu mV (%u%%), status: %s",
             (unsigned long)info->voltage_mv, info->percentage,
             info->status == PWRMGR_BAT_FULL ? "FULL" : info->status == PWRMGR_BAT_NORMAL ? "NORMAL"
                                                    : info->status == PWRMGR_BAT_LOW      ? "LOW"
                                                    : info->status == PWRMGR_BAT_CRITICAL ? "CRITICAL"
                                                                                          : "UNKNOWN");

    return ESP_OK;
}

/**
 * @brief Get adaptive sleep duration based on battery level
 */
uint64_t pwrmgr_get_adaptive_sleep_duration(const pwrmgr_battery_info_t *battery_info)
{
    if (battery_info == NULL || !s_config.adaptive_sleep)
    {
        return s_config.sleep_duration_us;
    }

    uint32_t sleep_seconds;

    switch (battery_info->status)
    {
    case PWRMGR_BAT_FULL:
    case PWRMGR_BAT_NORMAL:
        sleep_seconds = PWRMGR_SLEEP_INTERVAL_FULL_S;
        break;

    case PWRMGR_BAT_LOW:
        sleep_seconds = PWRMGR_SLEEP_INTERVAL_LOW_S;
        ESP_LOGW(TAG, "Low battery - extending sleep interval to %lu s",
                 (unsigned long)sleep_seconds);
        break;

    case PWRMGR_BAT_CRITICAL:
        sleep_seconds = PWRMGR_SLEEP_INTERVAL_CRITICAL_S;
        ESP_LOGW(TAG, "Critical battery - maximum sleep interval %lu s",
                 (unsigned long)sleep_seconds);
        break;

    default:
        sleep_seconds = PWRMGR_SLEEP_INTERVAL_FULL_S;
        break;
    }

    return (uint64_t)sleep_seconds * 1000000ULL;
}

/**
 * @brief Check if battery level is critical
 */
bool pwrmgr_is_battery_critical(const pwrmgr_battery_info_t *battery_info)
{
    if (battery_info == NULL)
    {
        return false;
    }

    return (battery_info->voltage_mv <= PWRMGR_BATTERY_CRITICAL_MV) ||
           (battery_info->percentage <= PWRMGR_BATTERY_CRITICAL_PERCENT);
}

/**
 * @brief Isolate all GPIOs for minimum sleep current
 */
void pwrmgr_isolate_gpio(void)
{
    ESP_LOGI(TAG, "Isolating GPIOs for minimum sleep current");

    /*
     * Isolate GPIO12 (MTDI) to prevent flash voltage interference.
     * This is important for modules using 3.3V flash.
     */
    rtc_gpio_isolate(GPIO_NUM_12);

    /*
     * Hold GPIO pins that might source/sink current through
     * external pull-up/pull-down resistors.
     * Isolate SPI flash pins and other common leakage sources.
     */
    rtc_gpio_isolate(GPIO_NUM_0);
    rtc_gpio_isolate(GPIO_NUM_2);
    rtc_gpio_isolate(GPIO_NUM_4);
    rtc_gpio_isolate(GPIO_NUM_15);

    /*
     * Note: Do NOT isolate GPIOs configured as wake-up sources.
     * The RTC controller needs them to detect the wake-up event.
     */

    ESP_LOGI(TAG, "GPIO isolation complete");
}

/**
 * @brief Configure RTC GPIO for wake-up
 */
esp_err_t pwrmgr_configure_rtc_gpio(gpio_num_t gpio_num, int level)
{
    /* Check if this is a valid RTC GPIO */
    if (!rtc_gpio_is_valid_gpio(gpio_num))
    {
        ESP_LOGE(TAG, "GPIO %d is not an RTC GPIO", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize as RTC GPIO */
    esp_err_t ret = rtc_gpio_init(gpio_num);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init RTC GPIO %d: %s",
                 gpio_num, esp_err_to_name(ret));
        return ret;
    }

    /* Set direction to input */
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);

    /* Configure pull-up/pull-down based on trigger level */
    if (level == 0)
    {
        /* Trigger on LOW: use internal pull-up so pin is normally HIGH */
        rtc_gpio_pullup_en(gpio_num);
        rtc_gpio_pulldown_dis(gpio_num);
        ESP_LOGI(TAG, "RTC GPIO %d configured: pull-up, trigger LOW", gpio_num);
    }
    else
    {
        /* Trigger on HIGH: use internal pull-down so pin is normally LOW */
        rtc_gpio_pullup_dis(gpio_num);
        rtc_gpio_pulldown_en(gpio_num);
        ESP_LOGI(TAG, "RTC GPIO %d configured: pull-down, trigger HIGH", gpio_num);
    }

    return ESP_OK;
}

/**
 * @brief Estimate current consumption for each power mode
 */
esp_err_t pwrmgr_estimate_current(pwrmgr_current_estimate_t *estimate,
                                  uint32_t battery_capacity_mah)
{
    if (estimate == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* Fill in per-mode estimates */
    estimate->active_ma = PWRMGR_CURRENT_ACTIVE_NO_WIFI_MA;
    estimate->modem_sleep_ma = PWRMGR_CURRENT_MODEM_SLEEP_MA;
    estimate->light_sleep_ma = PWRMGR_CURRENT_LIGHT_SLEEP_MA;
    estimate->deep_sleep_ua = PWRMGR_CURRENT_DEEP_SLEEP_UA;
    estimate->hibernate_ua = PWRMGR_CURRENT_HIBERNATE_UA;

    /*
     * Calculate average current based on duty cycle.
     *
     * Duty cycle: active for max_active_time_ms, then sleep for sleep_duration.
     *
     * I_avg = (I_active * t_active + I_sleep * t_sleep) / (t_active + t_sleep)
     */
    float t_active_s = (float)s_config.max_active_time_ms / 1000.0f;
    float t_sleep_s = (float)(s_config.sleep_duration_us / 1000000ULL);
    float t_total_s = t_active_s + t_sleep_s;

    float i_sleep_ma = 0.0f;
    switch (s_config.mode)
    {
    case PWRMGR_MODE_MODEM_SLEEP:
        i_sleep_ma = PWRMGR_CURRENT_MODEM_SLEEP_MA;
        break;
    case PWRMGR_MODE_LIGHT_SLEEP:
        i_sleep_ma = PWRMGR_CURRENT_LIGHT_SLEEP_MA;
        break;
    case PWRMGR_MODE_DEEP_SLEEP:
        i_sleep_ma = PWRMGR_CURRENT_DEEP_SLEEP_UA / 1000.0f;
        break;
    case PWRMGR_MODE_HIBERNATE:
        i_sleep_ma = PWRMGR_CURRENT_HIBERNATE_UA / 1000.0f;
        break;
    default:
        i_sleep_ma = PWRMGR_CURRENT_ACTIVE_NO_WIFI_MA;
        break;
    }

    float avg_current_ma = (PWRMGR_CURRENT_ACTIVE_NO_WIFI_MA * t_active_s +
                            i_sleep_ma * t_sleep_s) /
                           t_total_s;

    estimate->avg_current_ua = avg_current_ma * 1000.0f;

    /* Estimate battery life in days */
    if (avg_current_ma > 0.0f && battery_capacity_mah > 0)
    {
        float hours = (float)battery_capacity_mah / avg_current_ma;
        estimate->estimated_days = hours / 24.0f;
    }
    else
    {
        estimate->estimated_days = 0.0f;
    }

    return ESP_OK;
}

/**
 * @brief Print detailed power consumption report
 */
void pwrmgr_print_power_report(uint32_t battery_capacity_mah)
{
    pwrmgr_current_estimate_t estimate;

    if (pwrmgr_estimate_current(&estimate, battery_capacity_mah) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to calculate power estimates");
        return;
    }

    float t_active_s = (float)s_config.max_active_time_ms / 1000.0f;
    float t_sleep_s = (float)(s_config.sleep_duration_us / 1000000ULL);
    float duty_cycle = (t_active_s / (t_active_s + t_sleep_s)) * 100.0f;

    printf("\n");
    printf("========================================================\n");
    printf("          POWER CONSUMPTION REPORT\n");
    printf("========================================================\n");
    printf("\n");
    printf("  Current per mode:\n");
    printf("    Active (no WiFi):   %.1f mA\n", estimate.active_ma);
    printf("    Active (WiFi):      %.1f mA\n", PWRMGR_CURRENT_ACTIVE_MA);
    printf("    Modem Sleep:        %.1f mA\n", estimate.modem_sleep_ma);
    printf("    Light Sleep:        %.2f mA\n", estimate.light_sleep_ma);
    printf("    Deep Sleep:         %.1f uA\n", estimate.deep_sleep_ua);
    printf("    Hibernate:          %.1f uA\n", estimate.hibernate_ua);
    printf("\n");
    printf("  Duty cycle analysis:\n");
    printf("    Active time:        %.1f s\n", t_active_s);
    printf("    Sleep time:         %.1f s\n", t_sleep_s);
    printf("    Duty cycle:         %.2f%%\n", duty_cycle);
    printf("    Sleep mode:         %s\n",
           s_config.mode == PWRMGR_MODE_DEEP_SLEEP ? "DEEP_SLEEP" : s_config.mode == PWRMGR_MODE_LIGHT_SLEEP ? "LIGHT_SLEEP"
                                                                : s_config.mode == PWRMGR_MODE_HIBERNATE     ? "HIBERNATE"
                                                                : s_config.mode == PWRMGR_MODE_MODEM_SLEEP   ? "MODEM_SLEEP"
                                                                                                             : "ACTIVE");
    printf("\n");
    printf("  Average current:      %.2f uA (%.4f mA)\n",
           estimate.avg_current_ua, estimate.avg_current_ua / 1000.0f);
    printf("\n");
    printf("  Battery life estimate:\n");
    printf("    Battery capacity:   %lu mAh\n", (unsigned long)battery_capacity_mah);
    printf("    Estimated life:     %.1f days (%.1f months)\n",
           estimate.estimated_days, estimate.estimated_days / 30.0f);
    printf("\n");
    printf("========================================================\n");
    printf("\n");
}

/**
 * @brief Set the power mode
 */
esp_err_t pwrmgr_set_mode(pwrmgr_mode_t mode)
{
    if (mode > PWRMGR_MODE_HIBERNATE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    s_config.mode = mode;
    ESP_LOGI(TAG, "Power mode set to %d", mode);
    return ESP_OK;
}

/**
 * @brief Update sleep duration
 */
esp_err_t pwrmgr_set_sleep_duration(uint64_t duration_us)
{
    if (duration_us < PWRMGR_MIN_SLEEP_DURATION_US ||
        duration_us > PWRMGR_MAX_SLEEP_DURATION_US)
    {
        ESP_LOGE(TAG, "Sleep duration out of range");
        return ESP_ERR_INVALID_ARG;
    }

    s_config.sleep_duration_us = duration_us;

    /* Reconfigure timer wake-up if it is an active source */
    if (s_config.wake_sources & PWRMGR_WAKE_TIMER)
    {
        prv_configure_timer_wakeup();
    }

    ESP_LOGI(TAG, "Sleep duration updated to %llu us", duration_us);
    return ESP_OK;
}

/**
 * @brief Prepare ULP coprocessor for wake-up
 */
esp_err_t pwrmgr_prepare_ulp(void)
{
    /*
     * NOTE: Full ULP programming requires the ULP coprocessor toolchain
     * and is beyond the scope of this power manager module. This function
     * provides the framework for ULP wake-up integration.
     *
     * To use ULP wake-up:
     * 1. Write a ULP assembly program (.S file)
     * 2. Load it into RTC slow memory
     * 3. Configure the ULP timer period
     * 4. Start the ULP program
     * 5. Enable ULP wake-up: esp_sleep_enable_ulp_wakeup()
     */

    ESP_LOGW(TAG, "ULP coprocessor wake-up preparation:");
    ESP_LOGW(TAG, "  ULP program must be loaded separately.");
    ESP_LOGW(TAG, "  Call esp_sleep_enable_ulp_wakeup() after ULP setup.");

    /* Enable ULP wake-up source */
    esp_err_t ret = esp_sleep_enable_ulp_wakeup();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable ULP wake-up: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ULP wake-up source enabled");
    return ESP_OK;
}

/**
 * @brief Deinitialize the power manager
 */
void pwrmgr_deinit(void)
{
    if (s_config.monitor_battery)
    {
        prv_deinit_battery_adc();
    }

    s_initialized = false;
    ESP_LOGI(TAG, "Power manager deinitialized");
}

/*============================================================================
 *                    PRIVATE FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Configure timer-based wake-up
 *
 * @return ESP_OK on success
 */
static esp_err_t prv_configure_timer_wakeup(void)
{
    esp_err_t ret = esp_sleep_enable_timer_wakeup(s_config.sleep_duration_us);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure timer wake-up: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Timer wake-up configured: %llu us (%llu s)",
             s_config.sleep_duration_us,
             s_config.sleep_duration_us / 1000000ULL);
    return ESP_OK;
}

/**
 * @brief Configure EXT0 (single GPIO) wake-up
 *
 * EXT0 uses a single RTC GPIO to trigger wake-up when the pin
 * reaches the specified level.
 *
 * @return ESP_OK on success
 */
static esp_err_t prv_configure_ext0_wakeup(void)
{
    /* Configure the RTC GPIO with appropriate pull resistors */
    esp_err_t ret = pwrmgr_configure_rtc_gpio(s_config.ext0.gpio,
                                              s_config.ext0.level);
    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Enable EXT0 wake-up */
    ret = esp_sleep_enable_ext0_wakeup(s_config.ext0.gpio,
                                       s_config.ext0.level);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure EXT0 wake-up: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "EXT0 wake-up configured: GPIO %d, level %d",
             s_config.ext0.gpio, s_config.ext0.level);
    return ESP_OK;
}

/**
 * @brief Configure EXT1 (multiple GPIO) wake-up
 *
 * EXT1 uses multiple RTC GPIOs with configurable logic (ANY or ALL)
 * to trigger wake-up.
 *
 * @return ESP_OK on success
 */
static esp_err_t prv_configure_ext1_wakeup(void)
{
    if (s_config.ext1.gpio_mask == 0)
    {
        ESP_LOGW(TAG, "EXT1 wake-up requested but no GPIOs configured");
        return ESP_ERR_INVALID_ARG;
    }

    /* Configure each GPIO in the mask */
    for (int i = 0; i < GPIO_NUM_MAX; i++)
    {
        if (s_config.ext1.gpio_mask & (1ULL << i))
        {
            gpio_num_t gpio = (gpio_num_t)i;
            if (rtc_gpio_is_valid_gpio(gpio))
            {
                int level = (s_config.ext1.mode == PWRMGR_EXT1_ANY_HIGH) ? 1 : 0;
                pwrmgr_configure_rtc_gpio(gpio, level);
            }
        }
    }

    /* Enable EXT1 wake-up */
    esp_err_t ret = esp_sleep_enable_ext1_wakeup(s_config.ext1.gpio_mask,
                                                 s_config.ext1.mode);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure EXT1 wake-up: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "EXT1 wake-up configured: mask 0x%llX, mode %s",
             s_config.ext1.gpio_mask,
             s_config.ext1.mode == PWRMGR_EXT1_ANY_HIGH ? "ANY_HIGH" : "ALL_LOW");
    return ESP_OK;
}

/**
 * @brief Configure touch pad wake-up
 *
 * Sets up a capacitive touch pad as a deep sleep wake-up source.
 * The touch pad triggers a wake-up when the capacitance change
 * exceeds the configured threshold.
 *
 * @return ESP_OK on success
 */
static esp_err_t prv_configure_touch_wakeup(void)
{
    /* Initialize touch pad peripheral */
    esp_err_t ret = touch_pad_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Touch pad init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set reference voltage for charging/discharging */
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    /* Configure the touch pad channel */
    touch_pad_config(s_config.touch.touch_num, s_config.touch.threshold);

    /* Enable touch pad wake-up */
    esp_sleep_enable_touchpad_wakeup();

    ESP_LOGI(TAG, "Touch pad wake-up configured: pad %d, threshold %u",
             s_config.touch.touch_num, s_config.touch.threshold);
    return ESP_OK;
}

/**
 * @brief Initialize ADC for battery voltage measurement
 *
 * @return ESP_OK on success
 */
static esp_err_t prv_init_battery_adc(void)
{
    /* Configure ADC unit */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = PWRMGR_BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &s_adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure ADC channel */
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_oneshot_config_channel(s_adc_handle,
                                     PWRMGR_BATTERY_ADC_CHANNEL,
                                     &chan_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    /* Try to initialize ADC calibration */
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = PWRMGR_BATTERY_ADC_UNIT,
        .chan = PWRMGR_BATTERY_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = PWRMGR_BATTERY_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &s_adc_cali_handle);
#else
    ret = ESP_ERR_NOT_SUPPORTED;
#endif

    if (ret == ESP_OK)
    {
        s_adc_cali_valid = true;
        ESP_LOGI(TAG, "ADC calibration initialized successfully");
    }
    else
    {
        s_adc_cali_valid = false;
        ESP_LOGW(TAG, "ADC calibration not available, using raw conversion");
    }

    ESP_LOGI(TAG, "Battery ADC initialized (channel %d, 12-bit, 12dB atten)",
             PWRMGR_BATTERY_ADC_CHANNEL);
    return ESP_OK;
}

/**
 * @brief Deinitialize battery ADC
 */
static void prv_deinit_battery_adc(void)
{
    if (s_adc_cali_handle != NULL)
    {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(s_adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(s_adc_cali_handle);
#endif
        s_adc_cali_handle = NULL;
        s_adc_cali_valid = false;
    }

    if (s_adc_handle != NULL)
    {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
}

/**
 * @brief Read ADC with averaging for noise reduction
 *
 * @param[in] samples Number of samples to average
 * @return Averaged ADC reading (12-bit, 0-4095)
 */
static uint32_t prv_read_adc_averaged(uint32_t samples)
{
    if (s_adc_handle == NULL || samples == 0)
    {
        return 0;
    }

    uint64_t sum = 0;
    uint32_t valid_samples = 0;

    for (uint32_t i = 0; i < samples; i++)
    {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(s_adc_handle,
                                         PWRMGR_BATTERY_ADC_CHANNEL,
                                         &raw);
        if (ret == ESP_OK)
        {
            sum += (uint64_t)raw;
            valid_samples++;
        }
    }

    if (valid_samples == 0)
    {
        ESP_LOGE(TAG, "All ADC readings failed");
        return 0;
    }

    return (uint32_t)(sum / valid_samples);
}

/**
 * @brief Convert battery voltage to charge percentage
 *
 * Uses a simplified Li-Ion discharge curve approximation.
 * The curve is linearized between empty (3.0V) and full (4.2V).
 *
 * @param[in] voltage_mv Battery voltage in millivolts
 * @return Estimated charge percentage (0-100)
 */
static uint8_t prv_voltage_to_percentage(uint32_t voltage_mv)
{
    if (voltage_mv >= PWRMGR_BATTERY_FULL_MV)
    {
        return 100;
    }
    if (voltage_mv <= PWRMGR_BATTERY_EMPTY_MV)
    {
        return 0;
    }

    /*
     * Simplified Li-Ion discharge curve approximation:
     *
     * 4.20V = 100%
     * 4.10V =  90%   (steep initial drop)
     * 3.90V =  70%   (long plateau)
     * 3.80V =  50%   (mid-range)
     * 3.70V =  30%   (plateau ending)
     * 3.50V =  15%   (steep drop begins)
     * 3.30V =   5%   (nearly empty)
     * 3.00V =   0%   (cutoff)
     *
     * For simplicity, we use linear interpolation:
     */
    uint32_t range = PWRMGR_BATTERY_FULL_MV - PWRMGR_BATTERY_EMPTY_MV;
    uint32_t offset = voltage_mv - PWRMGR_BATTERY_EMPTY_MV;

    return (uint8_t)((offset * 100) / range);
}
