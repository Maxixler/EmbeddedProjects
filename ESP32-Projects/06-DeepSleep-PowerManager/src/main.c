/**
 * @file main.c
 * @brief ESP32 Deep Sleep Power Manager - Main Application
 *
 * Demonstrates a complete battery-powered IoT application using deep sleep
 * for maximum power efficiency. The application follows a duty-cycle pattern:
 *
 *   1. Wake up (timer, GPIO, or touchpad)
 *   2. Detect wake-up cause
 *   3. Read sensors quickly (minimize active time)
 *   4. Store data in RTC memory
 *   5. Every N cycles or on button press: connect WiFi, send data
 *   6. Check battery level (critical = indefinite sleep)
 *   7. Go back to deep sleep
 *
 * Duty Cycle Parameters:
 *   - Active time:   ~5 seconds maximum per wake cycle
 *   - Sleep time:    60 seconds (adaptive based on battery)
 *   - Data send:     Every 5 wake cycles
 *   - Battery check: Every wake cycle
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
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "power_manager.h"
#include "rtc_storage.h"

/*============================================================================
 *                          DEFINITIONS
 *============================================================================*/

/** @brief Main application log tag */
static const char *TAG = "MAIN";

/** @brief Battery capacity in mAh (typical 18650 Li-Ion) */
#define BATTERY_CAPACITY_MAH 2500

/** @brief LED indicator GPIO (onboard LED on most ESP32 devkits) */
#define LED_GPIO GPIO_NUM_2

/** @brief Button GPIO for manual wake-up (same as EXT0 default) */
#define BUTTON_GPIO GPIO_NUM_33

/** @brief Simulated temperature offset for demo */
#define TEMP_SIM_BASE 220     /* 22.0 C */
#define TEMP_SIM_VARIATION 30 /* +/- 3.0 C */

/** @brief Simulated humidity offset for demo */
#define HUMID_SIM_BASE 600      /* 60.0% */
#define HUMID_SIM_VARIATION 100 /* +/- 10.0% */

/** @brief Simulated pressure offset for demo */
#define PRESS_SIM_BASE 1130    /* 1013.0 hPa (offset from 9000) */
#define PRESS_SIM_VARIATION 50 /* +/- 5.0 hPa */

/*============================================================================
 *                     PRIVATE FUNCTION PROTOTYPES
 *============================================================================*/

static void app_handle_first_boot(void);
static void app_handle_timer_wakeup(void);
static void app_handle_gpio_wakeup(void);
static void app_handle_touch_wakeup(void);
static void app_handle_unknown_wakeup(void);
static void app_read_sensors(rtcstore_sensor_reading_t *reading);
static void app_send_data(void);
static void app_check_battery_and_sleep(void);
static void app_enter_sleep(uint64_t duration_us);
static void app_blink_led(int count, int delay_ms);
static void app_print_wakeup_info(pwrmgr_wakeup_cause_t cause);
static void app_print_startup_banner(void);

/*============================================================================
 *                         STATIC VARIABLES
 *============================================================================*/

/** @brief Power manager configuration */
static pwrmgr_config_t s_pm_config;

/** @brief Boot start timestamp for active time calculation */
static int64_t s_boot_start_us;

/*============================================================================
 *                        MAIN APPLICATION
 *============================================================================*/

/**
 * @brief Main application entry point
 *
 * This function is called on every boot - both initial power-on and
 * deep sleep wake-up. The wake-up cause determines the application
 * flow for each cycle.
 */
void app_main(void)
{
    /* Record boot start time for active duration calculation */
    s_boot_start_us = esp_timer_get_time();

    /* Print startup banner */
    app_print_startup_banner();

    /*------------------------------------------------------------------------
     * Step 1: Initialize RTC Storage
     *------------------------------------------------------------------------*/
    ESP_LOGI(TAG, "[1/7] Initializing RTC storage...");
    esp_err_t ret = rtcstore_init();
    if (ret == ESP_ERR_INVALID_CRC)
    {
        ESP_LOGW(TAG, "RTC data corruption detected, storage was reset");
    }

    /* Increment boot counter */
    uint32_t boot_count = rtcstore_increment_boot_count();
    ESP_LOGI(TAG, "Boot #%lu (total: %lu)",
             (unsigned long)boot_count,
             (unsigned long)rtcstore_get_total_boot_count());

    /*------------------------------------------------------------------------
     * Step 2: Detect Wake-up Cause
     *------------------------------------------------------------------------*/
    ESP_LOGI(TAG, "[2/7] Detecting wake-up cause...");
    pwrmgr_wakeup_cause_t wakeup_cause = pwrmgr_get_wakeup_cause();
    app_print_wakeup_info(wakeup_cause);

    /*------------------------------------------------------------------------
     * Step 3: Initialize Power Manager
     *------------------------------------------------------------------------*/
    ESP_LOGI(TAG, "[3/7] Initializing power manager...");
    s_pm_config = pwrmgr_config_default();

    /* Configure wake-up sources */
    s_pm_config.mode = PWRMGR_MODE_DEEP_SLEEP;
    s_pm_config.wake_sources = PWRMGR_WAKE_TIMER | PWRMGR_WAKE_EXT0 | PWRMGR_WAKE_TOUCHPAD;
    s_pm_config.sleep_duration_us = PWRMGR_DEFAULT_SLEEP_DURATION_US; /* 60s */
    s_pm_config.cycles_before_send = PWRMGR_CYCLES_BEFORE_SEND;       /* 5 */
    s_pm_config.max_active_time_ms = PWRMGR_MAX_ACTIVE_TIME_MS;       /* 5000 */

    /* EXT0: Button on GPIO33, wake on LOW (button press pulls LOW) */
    s_pm_config.ext0.gpio = BUTTON_GPIO;
    s_pm_config.ext0.level = 0;

    /* Touch: Touch pad 7 (GPIO 27) */
    s_pm_config.touch.touch_num = TOUCH_PAD_NUM7;
    s_pm_config.touch.threshold = PWRMGR_TOUCH_THRESHOLD;

    ret = pwrmgr_init(&s_pm_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Power manager init failed: %s", esp_err_to_name(ret));
    }

    /*------------------------------------------------------------------------
     * Step 4: Configure LED indicator
     *------------------------------------------------------------------------*/
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    /*------------------------------------------------------------------------
     * Step 5: Handle Wake-up Cause
     *------------------------------------------------------------------------*/
    ESP_LOGI(TAG, "[4/7] Processing wake-up event...");

    switch (wakeup_cause)
    {
    case PWRMGR_WAKEUP_UNDEFINED:
        /* First boot (power-on or software reset) */
        app_handle_first_boot();
        break;

    case PWRMGR_WAKEUP_TIMER:
        /* Periodic timer wake-up */
        app_handle_timer_wakeup();
        break;

    case PWRMGR_WAKEUP_EXT0:
    case PWRMGR_WAKEUP_EXT1:
        /* Button press / external GPIO */
        app_handle_gpio_wakeup();
        break;

    case PWRMGR_WAKEUP_TOUCHPAD:
        /* Touch pad activation */
        app_handle_touch_wakeup();
        break;

    default:
        app_handle_unknown_wakeup();
        break;
    }

    /*------------------------------------------------------------------------
     * Step 6: Record wake-up in history
     *------------------------------------------------------------------------*/
    ESP_LOGI(TAG, "[5/7] Recording wake-up event in history...");

    int64_t active_us = esp_timer_get_time() - s_boot_start_us;
    uint32_t active_ms = (uint32_t)(active_us / 1000);

    pwrmgr_battery_info_t bat_info = {0};
    if (s_pm_config.monitor_battery)
    {
        pwrmgr_read_battery(&bat_info);
    }

    rtcstore_wakeup_entry_t wake_entry = {
        .timestamp = (uint32_t)(esp_timer_get_time() / 1000000LL),
        .cause = (uint8_t)wakeup_cause,
        .battery_pct = bat_info.percentage,
        .active_time_ms = (uint16_t)(active_ms > 65535 ? 65535 : active_ms),
        .sleep_duration_s = (uint32_t)(s_pm_config.sleep_duration_us / 1000000ULL),
        .gpio_state = 0,
    };

    /* Capture GPIO state for EXT0/EXT1 wake-ups */
    if (wakeup_cause == PWRMGR_WAKEUP_EXT0 || wakeup_cause == PWRMGR_WAKEUP_EXT1)
    {
        uint64_t gpio_mask = 0;
        pwrmgr_get_wakeup_gpio(&gpio_mask);
        wake_entry.gpio_state = (uint32_t)(gpio_mask & 0xFFFFFFFF);
    }

    rtcstore_add_wakeup_entry(&wake_entry);

    /*------------------------------------------------------------------------
     * Step 7: Battery check and sleep
     *------------------------------------------------------------------------*/
    ESP_LOGI(TAG, "[6/7] Checking battery level...");
    app_check_battery_and_sleep();

    /* Should not reach here for deep sleep mode */
    ESP_LOGI(TAG, "[7/7] Application cycle complete");
}

/*============================================================================
 *                    PRIVATE FUNCTION IMPLEMENTATIONS
 *============================================================================*/

/**
 * @brief Handle first boot (power-on reset)
 *
 * On initial power-up, perform full system diagnostics, print
 * configuration, and take the first sensor reading.
 */
static void app_handle_first_boot(void)
{
    ESP_LOGI(TAG, "=== FIRST BOOT (Power-On Reset) ===");
    app_blink_led(3, 200); /* Triple blink = first boot */

    /* Print system information */
    printf("\n");
    printf("  ESP32 Deep Sleep Power Manager\n");
    printf("  ==============================\n");
    printf("  Chip:       ESP32\n");
    printf("  Free heap:  %lu bytes\n", (unsigned long)esp_get_free_heap_size());
    printf("  IDF ver:    %s\n", esp_get_idf_version());
    printf("\n");

    /* Print power consumption report */
    pwrmgr_print_power_report(BATTERY_CAPACITY_MAH);

    /* Print RTC storage status */
    rtcstore_print_status();

    /* Take initial sensor reading */
    rtcstore_sensor_reading_t reading;
    app_read_sensors(&reading);
    rtcstore_add_sensor_reading(&reading);

    ESP_LOGI(TAG, "First boot complete. Entering first sleep cycle...");

    /* Enter sleep for the first time */
    app_enter_sleep(s_pm_config.sleep_duration_us);
}

/**
 * @brief Handle timer wake-up
 *
 * Periodic wake-up: read sensors quickly, store in RTC memory.
 * Every N cycles, send accumulated data.
 */
static void app_handle_timer_wakeup(void)
{
    ESP_LOGI(TAG, "=== TIMER WAKE-UP ===");
    app_blink_led(1, 100); /* Single short blink = timer */

    uint32_t boot_count = rtcstore_get_boot_count();

    /* Read sensors and store */
    rtcstore_sensor_reading_t reading;
    app_read_sensors(&reading);
    rtcstore_add_sensor_reading(&reading);

    ESP_LOGI(TAG, "Sensor reading stored (buffer: %lu/%d readings)",
             (unsigned long)rtcstore_get_sensor_count(),
             RTCSTORE_MAX_SENSOR_READINGS);

    /*
     * Check if it's time to send data:
     *   - Every cycles_before_send wake cycles
     *   - Or if the buffer is nearly full
     */
    bool should_send = false;

    if ((boot_count % s_pm_config.cycles_before_send) == 0)
    {
        ESP_LOGI(TAG, "Cycle %lu: time to send data (every %lu cycles)",
                 (unsigned long)boot_count,
                 (unsigned long)s_pm_config.cycles_before_send);
        should_send = true;
    }

    if (rtcstore_needs_flush())
    {
        ESP_LOGW(TAG, "Sensor buffer near full, forcing data send");
        should_send = true;
    }

    if (should_send)
    {
        app_send_data();
    }

    /* Determine sleep duration (adaptive based on battery) */
    pwrmgr_battery_info_t bat_info = {0};
    uint64_t sleep_duration_us = s_pm_config.sleep_duration_us;

    if (s_pm_config.monitor_battery)
    {
        pwrmgr_read_battery(&bat_info);
        if (s_pm_config.adaptive_sleep)
        {
            sleep_duration_us = pwrmgr_get_adaptive_sleep_duration(&bat_info);
        }
    }

    /* Update timing statistics */
    int64_t active_us = esp_timer_get_time() - s_boot_start_us;
    rtcstore_update_timing((uint32_t)(active_us / 1000),
                           (uint32_t)(sleep_duration_us / 1000000ULL));

    /* Go back to sleep */
    app_enter_sleep(sleep_duration_us);
}

/**
 * @brief Handle GPIO (button) wake-up
 *
 * Button press: immediately send all accumulated data, then sleep.
 */
static void app_handle_gpio_wakeup(void)
{
    ESP_LOGI(TAG, "=== GPIO WAKE-UP (Button Press) ===");
    app_blink_led(2, 150); /* Double blink = button */

    /* Identify which GPIO triggered the wake-up */
    uint64_t gpio_mask = 0;
    if (pwrmgr_get_wakeup_gpio(&gpio_mask) == ESP_OK)
    {
        ESP_LOGI(TAG, "Wake-up GPIO mask: 0x%llX", gpio_mask);
        for (int i = 0; i < 40; i++)
        {
            if (gpio_mask & (1ULL << i))
            {
                ESP_LOGI(TAG, "  -> GPIO %d triggered wake-up", i);
            }
        }
    }

    /* Read sensors */
    rtcstore_sensor_reading_t reading;
    app_read_sensors(&reading);
    rtcstore_add_sensor_reading(&reading);

    /* Button press = send all data immediately */
    ESP_LOGI(TAG, "Button pressed: sending all accumulated data...");
    app_send_data();

    /* Print status report */
    rtcstore_print_status();
    pwrmgr_print_power_report(BATTERY_CAPACITY_MAH);

    /* Resume normal sleep cycle */
    app_enter_sleep(s_pm_config.sleep_duration_us);
}

/**
 * @brief Handle touch pad wake-up
 *
 * Touch activation: read sensors and send data.
 */
static void app_handle_touch_wakeup(void)
{
    ESP_LOGI(TAG, "=== TOUCH PAD WAKE-UP ===");
    app_blink_led(4, 80); /* Rapid quad blink = touch */

    /* Identify which touch pad triggered the wake-up */
    touch_pad_t touch_num;
    if (pwrmgr_get_wakeup_touchpad(&touch_num) == ESP_OK)
    {
        ESP_LOGI(TAG, "Wake-up touch pad: T%d", touch_num);
    }

    /* Read sensors */
    rtcstore_sensor_reading_t reading;
    app_read_sensors(&reading);
    rtcstore_add_sensor_reading(&reading);

    /* Touch = send data like button press */
    ESP_LOGI(TAG, "Touch detected: sending data...");
    app_send_data();

    /* Resume normal sleep */
    app_enter_sleep(s_pm_config.sleep_duration_us);
}

/**
 * @brief Handle unknown/unhandled wake-up cause
 */
static void app_handle_unknown_wakeup(void)
{
    ESP_LOGW(TAG, "=== UNKNOWN WAKE-UP CAUSE ===");
    app_blink_led(5, 100);

    /* Take a reading anyway */
    rtcstore_sensor_reading_t reading;
    app_read_sensors(&reading);
    rtcstore_add_sensor_reading(&reading);

    /* Go back to sleep */
    app_enter_sleep(s_pm_config.sleep_duration_us);
}

/**
 * @brief Read sensors and fill a sensor reading structure
 *
 * In a real application, this would interface with I2C/SPI sensors.
 * For demonstration, we generate simulated sensor data based on
 * boot count to show realistic-looking values.
 *
 * @param[out] reading Pointer to sensor reading to fill
 */
static void app_read_sensors(rtcstore_sensor_reading_t *reading)
{
    if (reading == NULL)
        return;

    ESP_LOGI(TAG, "Reading sensors...");

    /*
     * Simulated sensor readings for demonstration.
     * In a real application, replace with actual sensor I2C/SPI reads:
     *   - BME280/BMP280 for temperature, humidity, pressure
     *   - BH1750 for light level
     */
    uint32_t boot = rtcstore_get_boot_count();

    /* Simulate temperature with slight variation per boot */
    int16_t temp_variation = (int16_t)((boot * 7) % TEMP_SIM_VARIATION) - (TEMP_SIM_VARIATION / 2);
    reading->temperature = TEMP_SIM_BASE + temp_variation;

    /* Simulate humidity */
    int16_t humid_variation = (int16_t)((boot * 13) % HUMID_SIM_VARIATION) - (HUMID_SIM_VARIATION / 2);
    reading->humidity = (uint16_t)(HUMID_SIM_BASE + humid_variation);

    /* Simulate pressure */
    int16_t press_variation = (int16_t)((boot * 3) % PRESS_SIM_VARIATION) - (PRESS_SIM_VARIATION / 2);
    reading->pressure = (uint16_t)(PRESS_SIM_BASE + press_variation);

    /* Simulate light level (varies with "time of day" approximated by boot count) */
    reading->light_lux = (int16_t)((boot * 100) % 1000);

    /* Battery voltage from actual ADC */
    pwrmgr_battery_info_t bat_info = {0};
    if (pwrmgr_read_battery(&bat_info) == ESP_OK)
    {
        reading->battery_mv = (uint16_t)bat_info.voltage_mv;
    }
    else
    {
        reading->battery_mv = 0;
    }

    /* Timestamp and wake cause */
    reading->timestamp = (uint32_t)(esp_timer_get_time() / 1000000LL);
    reading->wake_cause = (uint8_t)pwrmgr_get_wakeup_cause();
    reading->reserved = 0;

    ESP_LOGI(TAG, "Sensor data: T=%.1fC, H=%.1f%%, P=%.1f hPa, L=%d lux, Bat=%u mV",
             reading->temperature / 10.0f,
             reading->humidity / 10.0f,
             (9000 + reading->pressure) / 10.0f,
             reading->light_lux,
             reading->battery_mv);
}

/**
 * @brief Send accumulated data over WiFi
 *
 * In a real application, this would:
 *   1. Connect to WiFi
 *   2. Send data via MQTT/HTTP to a server
 *   3. Disconnect WiFi
 *
 * For demonstration, we simulate the data transmission and print
 * the data that would be sent.
 */
static void app_send_data(void)
{
    ESP_LOGI(TAG, "--- Data Transmission ---");

    /* Get all buffered sensor readings */
    rtcstore_sensor_reading_t readings[RTCSTORE_MAX_SENSOR_READINGS];
    uint32_t count = 0;

    esp_err_t ret = rtcstore_get_sensor_readings(readings,
                                                 RTCSTORE_MAX_SENSOR_READINGS,
                                                 &count);
    if (ret != ESP_OK || count == 0)
    {
        ESP_LOGW(TAG, "No sensor data to send (ret=%s, count=%lu)",
                 esp_err_to_name(ret), (unsigned long)count);
        return;
    }

    ESP_LOGI(TAG, "Preparing to send %lu sensor readings...",
             (unsigned long)count);

    /*
     * === WiFi Connection (Simulated) ===
     *
     * In production code, this section would contain:
     *
     *   wifi_init_sta();
     *   esp_wifi_start();
     *   // Wait for connection...
     *   mqtt_client_publish(readings, count);
     *   esp_wifi_stop();
     *
     * WiFi connection typically takes 2-5 seconds and draws ~240mA.
     * This is the largest power consumer in the duty cycle.
     */

    ESP_LOGI(TAG, "[SIM] Connecting to WiFi...");
    vTaskDelay(pdMS_TO_TICKS(500)); /* Simulate WiFi connection time */
    ESP_LOGI(TAG, "[SIM] WiFi connected");

    /* Print data summary */
    printf("\n  --- Sensor Data Batch (%lu readings) ---\n", (unsigned long)count);
    printf("  %-6s %-8s %-8s %-10s %-8s %-8s %-10s\n",
           "Index", "Temp(C)", "Hum(%)", "Press(hPa)", "Lux", "Bat(mV)", "Cause");
    printf("  %-6s %-8s %-8s %-10s %-8s %-8s %-10s\n",
           "-----", "-------", "------", "---------", "----", "------", "-----");

    for (uint32_t i = 0; i < count && i < 10; i++)
    {
        printf("  %-6lu %-8.1f %-8.1f %-10.1f %-8d %-8u %-10s\n",
               (unsigned long)i,
               readings[i].temperature / 10.0f,
               readings[i].humidity / 10.0f,
               (9000 + readings[i].pressure) / 10.0f,
               readings[i].light_lux,
               readings[i].battery_mv,
               pwrmgr_wakeup_cause_to_str((pwrmgr_wakeup_cause_t)readings[i].wake_cause));
    }
    if (count > 10)
    {
        printf("  ... and %lu more readings\n", (unsigned long)(count - 10));
    }
    printf("\n");

    ESP_LOGI(TAG, "[SIM] Data transmitted successfully");
    ESP_LOGI(TAG, "[SIM] Disconnecting WiFi...");
    vTaskDelay(pdMS_TO_TICKS(100)); /* Simulate WiFi disconnect */

    /* Flush to NVS as backup */
    rtcstore_flush_to_nvs();

    /* Clear the sensor buffer after successful send */
    rtcstore_clear_sensor_buffer();

    ESP_LOGI(TAG, "Data transmission complete. Buffer cleared.");
}

/**
 * @brief Check battery level and enter appropriate sleep mode
 *
 * If battery is critically low, enters indefinite deep sleep
 * with only GPIO wake-up. Otherwise, enters normal deep sleep
 * with the configured duration.
 */
static void app_check_battery_and_sleep(void)
{
    pwrmgr_battery_info_t bat_info = {0};

    if (s_pm_config.monitor_battery)
    {
        esp_err_t ret = pwrmgr_read_battery(&bat_info);
        if (ret != ESP_OK)
        {
            ESP_LOGW(TAG, "Battery read failed, proceeding with normal sleep");
            app_enter_sleep(s_pm_config.sleep_duration_us);
            return;
        }

        ESP_LOGI(TAG, "Battery: %lu mV (%u%%)",
                 (unsigned long)bat_info.voltage_mv,
                 bat_info.percentage);

        /* Check for critical battery */
        if (pwrmgr_is_battery_critical(&bat_info))
        {
            ESP_LOGW(TAG, "!!! CRITICAL BATTERY LEVEL !!!");
            ESP_LOGW(TAG, "Voltage: %lu mV (threshold: %d mV)",
                     (unsigned long)bat_info.voltage_mv,
                     PWRMGR_BATTERY_CRITICAL_MV);
            ESP_LOGW(TAG, "Entering indefinite deep sleep...");
            ESP_LOGW(TAG, "Press button on GPIO %d to wake up", BUTTON_GPIO);

            /* Save data before indefinite sleep */
            rtcstore_flush_to_nvs();

            /* Indefinite sleep - only button can wake us */
            pwrmgr_enter_indefinite_sleep(BUTTON_GPIO, 0);
            /* Never returns */
        }

        /* Adaptive sleep duration */
        uint64_t sleep_us = s_pm_config.sleep_duration_us;
        if (s_pm_config.adaptive_sleep)
        {
            sleep_us = pwrmgr_get_adaptive_sleep_duration(&bat_info);
            ESP_LOGI(TAG, "Adaptive sleep: %llu s (battery %u%%)",
                     sleep_us / 1000000ULL, bat_info.percentage);
        }

        app_enter_sleep(sleep_us);
    }
    else
    {
        /* No battery monitoring - use fixed sleep interval */
        app_enter_sleep(s_pm_config.sleep_duration_us);
    }
}

/**
 * @brief Enter deep sleep with the specified duration
 *
 * Updates timing statistics, prints a summary, and enters deep sleep.
 *
 * @param[in] duration_us Sleep duration in microseconds
 */
static void app_enter_sleep(uint64_t duration_us)
{
    /* Calculate active time */
    int64_t active_us = esp_timer_get_time() - s_boot_start_us;
    uint32_t active_ms = (uint32_t)(active_us / 1000);

    /* Update timing statistics */
    rtcstore_update_timing(active_ms, (uint32_t)(duration_us / 1000000ULL));

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Active time: %lu ms", (unsigned long)active_ms);
    ESP_LOGI(TAG, "  Sleep duration: %llu s", duration_us / 1000000ULL);
    ESP_LOGI(TAG, "  Sensor readings buffered: %lu",
             (unsigned long)rtcstore_get_sensor_count());
    ESP_LOGI(TAG, "  Next data send in: %lu cycles",
             (unsigned long)(s_pm_config.cycles_before_send -
                             (rtcstore_get_boot_count() % s_pm_config.cycles_before_send)));
    ESP_LOGI(TAG, "  Entering deep sleep...");
    ESP_LOGI(TAG, "========================================");

    /* Ensure UART output is flushed before sleep */
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Update sleep duration and enter sleep */
    pwrmgr_set_sleep_duration(duration_us);
    pwrmgr_enter_sleep();

    /* If we reach here, it was light sleep and we woke up */
    ESP_LOGI(TAG, "Returned from light sleep");
}

/**
 * @brief Blink the onboard LED
 *
 * @param[in] count     Number of blinks
 * @param[in] delay_ms  Delay between blinks in milliseconds
 */
static void app_blink_led(int count, int delay_ms)
{
    for (int i = 0; i < count; i++)
    {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(LED_GPIO, 0);
        if (i < count - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

/**
 * @brief Print detailed wake-up cause information
 *
 * @param[in] cause Wake-up cause
 */
static void app_print_wakeup_info(pwrmgr_wakeup_cause_t cause)
{
    printf("\n");
    printf("  +----------------------------------------------+\n");
    printf("  |           WAKE-UP INFORMATION                |\n");
    printf("  +----------------------------------------------+\n");
    printf("  |  Cause:    %-33s|\n", pwrmgr_wakeup_cause_to_str(cause));

    if (cause == PWRMGR_WAKEUP_EXT0 || cause == PWRMGR_WAKEUP_EXT1)
    {
        uint64_t gpio_mask = 0;
        if (pwrmgr_get_wakeup_gpio(&gpio_mask) == ESP_OK)
        {
            printf("  |  GPIO:     0x%08llX                       |\n", gpio_mask);
        }
    }

    if (cause == PWRMGR_WAKEUP_TOUCHPAD)
    {
        touch_pad_t tp;
        if (pwrmgr_get_wakeup_touchpad(&tp) == ESP_OK)
        {
            printf("  |  Touch:    T%-32d|\n", tp);
        }
    }

    printf("  |  Boot #:   %-33lu|\n", (unsigned long)rtcstore_get_boot_count());
    printf("  +----------------------------------------------+\n");
    printf("\n");
}

/**
 * @brief Print application startup banner
 */
static void app_print_startup_banner(void)
{
    printf("\n");
    printf("  ============================================\n");
    printf("    ESP32 Deep Sleep Power Manager v1.0.0\n");
    printf("    Battery-Powered IoT Application Demo\n");
    printf("  ============================================\n");
    printf("\n");
}
