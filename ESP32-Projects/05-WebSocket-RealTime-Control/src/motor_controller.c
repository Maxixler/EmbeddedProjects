/**
 * @file motor_controller.c
 * @brief Motor and servo controller implementation using ESP32 LEDC PWM.
 *
 * This file implements the motor controller module providing:
 *  - DC motor control via L298N H-bridge (LEDC PWM)
 *  - Servo control via direct PWM (50Hz, 500-2500us)
 *  - Smooth speed ramping (acceleration/deceleration)
 *  - Encoder position feedback via PCNT peripheral
 *  - RPM calculation from encoder pulse counting
 *
 * @author EmbeddedProjects
 * @date 2026
 * @version 1.0.0
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "motor_controller.h"

/*******************************************************************************
 * Private Definitions
 ******************************************************************************/

/** @brief Logging tag for this module. */
static const char *TAG = "motor_ctrl";

/** @brief Servo PWM period in microseconds (1/50Hz = 20000us). */
#define SERVO_PERIOD_US 20000

/** @brief Stack size for the speed ramp task (bytes). */
#define RAMP_TASK_STACK_SIZE 2048

/** @brief Priority of the speed ramp task. */
#define RAMP_TASK_PRIORITY 4

/*******************************************************************************
 * Private Variables
 ******************************************************************************/

/** @brief Motor controller state (singleton). */
static motor_state_t s_motor_state = {
    .target_speed = 0,
    .current_speed = 0,
    .direction = MOTOR_DIR_COAST,
    .servo_angle = 90,
    .encoder_count = 0,
    .prev_encoder_count = 0,
    .rpm = 0.0f,
    .last_rpm_calc_time = 0,
    .initialized = false,
    .motor_enabled = false};

/** @brief Active motor configuration. */
static motor_config_t s_config;

/** @brief Mutex for protecting motor state access. */
static SemaphoreHandle_t s_state_mutex = NULL;

/** @brief PCNT unit handle for encoder. */
static pcnt_unit_handle_t s_pcnt_unit = NULL;

/** @brief PCNT channel handle for encoder channel A. */
static pcnt_channel_handle_t s_pcnt_chan_a = NULL;

/** @brief PCNT channel handle for encoder channel B. */
static pcnt_channel_handle_t s_pcnt_chan_b = NULL;

/** @brief Handle for the speed ramp FreeRTOS task. */
static TaskHandle_t s_ramp_task_handle = NULL;

/** @brief Flag to signal the ramp task to stop. */
static volatile bool s_ramp_task_running = false;

/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/

static esp_err_t configure_motor_pwm(void);
static esp_err_t configure_servo_pwm(void);
static esp_err_t configure_encoder_pcnt(void);
static esp_err_t configure_enable_pin(void);
static esp_err_t apply_motor_pwm(int32_t speed_pct, motor_direction_t dir);
static uint32_t angle_to_duty(int32_t angle);
static void speed_ramp_task(void *arg);
static int32_t clamp_i32(int32_t value, int32_t min_val, int32_t max_val);

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Clamp an integer value to a specified range.
 *
 * @param[in] value    Value to clamp.
 * @param[in] min_val  Minimum allowed value.
 * @param[in] max_val  Maximum allowed value.
 *
 * @return Clamped value.
 */
static int32_t clamp_i32(int32_t value, int32_t min_val, int32_t max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

/**
 * @brief Configure LEDC PWM channels for DC motor control.
 *
 * Sets up two LEDC channels (for IN1 and IN2 of the L298N H-bridge)
 * with the motor PWM frequency and resolution.
 *
 * @return ESP_OK on success.
 */
static esp_err_t configure_motor_pwm(void)
{
    esp_err_t ret;

    /* Configure LEDC timer for motor. */
    ledc_timer_config_t timer_conf = {
        .speed_mode = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num = MOTOR_LEDC_TIMER,
        .freq_hz = s_config.motor_pwm_freq,
        .clk_cfg = LEDC_AUTO_CLK};

    ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure LEDC channel for IN1 (forward). */
    ledc_channel_config_t ch_in1 = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_IN1,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = s_config.pin_in1,
        .duty = 0,
        .hpoint = 0};

    ret = ledc_channel_config(&ch_in1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor LEDC channel IN1 config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure LEDC channel for IN2 (reverse). */
    ledc_channel_config_t ch_in2 = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_IN2,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = s_config.pin_in2,
        .duty = 0,
        .hpoint = 0};

    ret = ledc_channel_config(&ch_in2);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Motor LEDC channel IN2 config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Install LEDC fade service for smooth transitions. */
    ledc_fade_func_install(0);

    ESP_LOGI(TAG, "Motor PWM configured: IN1=GPIO%d, IN2=GPIO%d, freq=%luHz",
             s_config.pin_in1, s_config.pin_in2, (unsigned long)s_config.motor_pwm_freq);

    return ESP_OK;
}

/**
 * @brief Configure LEDC PWM channel for servo motor control.
 *
 * Sets up a LEDC channel at 50Hz with 16-bit resolution for
 * precise servo pulse width control (500-2500us).
 *
 * @return ESP_OK on success.
 */
static esp_err_t configure_servo_pwm(void)
{
    esp_err_t ret;

    /* Configure LEDC timer for servo (50Hz). */
    ledc_timer_config_t timer_conf = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_PWM_RESOLUTION,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK};

    ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Servo LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure LEDC channel for servo. */
    ledc_channel_config_t ch_servo = {
        .speed_mode = SERVO_LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = s_config.pin_servo,
        .duty = angle_to_duty(90), /* Start at center (90 degrees). */
        .hpoint = 0};

    ret = ledc_channel_config(&ch_servo);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Servo LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Servo PWM configured: GPIO%d, 50Hz, center position",
             s_config.pin_servo);

    return ESP_OK;
}

/**
 * @brief Configure the PCNT peripheral for rotary encoder reading.
 *
 * Sets up one PCNT unit with two channels for quadrature decoding
 * of the encoder signals (channels A and B).
 *
 * @return ESP_OK on success.
 */
static esp_err_t configure_encoder_pcnt(void)
{
    esp_err_t ret;

    /* Create PCNT unit. */
    pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
    };

    ret = pcnt_new_unit(&unit_config, &s_pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT unit creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure glitch filter. */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENCODER_GLITCH_FILTER_NS,
    };
    ret = pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_config);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "PCNT glitch filter config failed: %s", esp_err_to_name(ret));
        /* Non-fatal; continue without filter. */
    }

    /* Configure channel A: count on edge of A, direction from B. */
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = s_config.pin_enc_a,
        .level_gpio_num = s_config.pin_enc_b,
    };

    ret = pcnt_new_channel(s_pcnt_unit, &chan_a_config, &s_pcnt_chan_a);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT channel A creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Rising edge of A: if B is high, count down; if B is low, count up. */
    pcnt_channel_set_edge_action(s_pcnt_chan_a,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(s_pcnt_chan_a,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Configure channel B for quadrature decoding. */
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = s_config.pin_enc_b,
        .level_gpio_num = s_config.pin_enc_a,
    };

    ret = pcnt_new_channel(s_pcnt_unit, &chan_b_config, &s_pcnt_chan_b);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT channel B creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    pcnt_channel_set_edge_action(s_pcnt_chan_b,
                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(s_pcnt_chan_b,
                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Enable and start PCNT unit. */
    ret = pcnt_unit_enable(s_pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT unit enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_clear_count(s_pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "PCNT clear count failed: %s", esp_err_to_name(ret));
    }

    ret = pcnt_unit_start(s_pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "PCNT unit start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Encoder PCNT configured: A=GPIO%d, B=GPIO%d, PPR=%lu",
             s_config.pin_enc_a, s_config.pin_enc_b,
             (unsigned long)s_config.encoder_ppr);

    return ESP_OK;
}

/**
 * @brief Configure the motor enable pin (L298N ENA) as a GPIO output.
 *
 * @return ESP_OK on success.
 */
static esp_err_t configure_enable_pin(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_config.pin_ena),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ENA GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start with motor disabled. */
    gpio_set_level(s_config.pin_ena, 0);

    ESP_LOGI(TAG, "Motor ENA configured: GPIO%d", s_config.pin_ena);
    return ESP_OK;
}

/**
 * @brief Apply PWM duty cycle to motor H-bridge channels.
 *
 * Sets the LEDC duty for IN1 and IN2 based on the requested
 * speed percentage and direction.
 *
 * H-bridge truth table (L298N):
 * | Direction | IN1  | IN2  | ENA  |
 * |-----------|------|------|------|
 * | Forward   | PWM  | LOW  | HIGH |
 * | Reverse   | LOW  | PWM  | HIGH |
 * | Brake     | HIGH | HIGH | HIGH |
 * | Coast     | LOW  | LOW  | LOW  |
 *
 * @param[in] speed_pct   Speed percentage (0-100).
 * @param[in] dir         Motor direction.
 *
 * @return ESP_OK on success.
 */
static esp_err_t apply_motor_pwm(int32_t speed_pct, motor_direction_t dir)
{
    uint32_t duty = (uint32_t)((speed_pct * MOTOR_PWM_MAX_DUTY) / 100);
    uint32_t duty_in1 = 0;
    uint32_t duty_in2 = 0;
    int ena_level = 1;

    switch (dir)
    {
    case MOTOR_DIR_FORWARD:
        duty_in1 = duty;
        duty_in2 = 0;
        ena_level = 1;
        break;

    case MOTOR_DIR_REVERSE:
        duty_in1 = 0;
        duty_in2 = duty;
        ena_level = 1;
        break;

    case MOTOR_DIR_BRAKE:
        duty_in1 = MOTOR_PWM_MAX_DUTY;
        duty_in2 = MOTOR_PWM_MAX_DUTY;
        ena_level = 1;
        break;

    case MOTOR_DIR_COAST:
    default:
        duty_in1 = 0;
        duty_in2 = 0;
        ena_level = 0;
        break;
    }

    /* Apply duty cycles. */
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_IN1, duty_in1);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_IN1);

    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_IN2, duty_in2);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_IN2);

    /* Set enable pin. */
    gpio_set_level(s_config.pin_ena, ena_level);

    ESP_LOGD(TAG, "Motor PWM: duty_in1=%lu, duty_in2=%lu, ena=%d",
             (unsigned long)duty_in1, (unsigned long)duty_in2, ena_level);

    return ESP_OK;
}

/**
 * @brief Convert a servo angle (0-180) to LEDC duty cycle value.
 *
 * Maps the angle to a pulse width between SERVO_MIN_PULSE_US (500us)
 * and SERVO_MAX_PULSE_US (2500us), then converts to a duty cycle
 * value for the 16-bit LEDC resolution at 50Hz.
 *
 * Calculation:
 *   pulse_us = SERVO_MIN_PULSE_US + (angle / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)
 *   duty = (pulse_us / SERVO_PERIOD_US) * SERVO_PWM_MAX_DUTY
 *
 * @param[in] angle    Servo angle in degrees (0-180).
 *
 * @return LEDC duty cycle value (16-bit range).
 */
static uint32_t angle_to_duty(int32_t angle)
{
    angle = clamp_i32(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    /* Calculate pulse width in microseconds. */
    uint32_t pulse_us = SERVO_MIN_PULSE_US +
                        (uint32_t)((angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
                                   SERVO_MAX_ANGLE);

    /* Convert to duty cycle. */
    uint32_t duty = (uint32_t)(((uint64_t)pulse_us * SERVO_PWM_MAX_DUTY) / SERVO_PERIOD_US);

    ESP_LOGD(TAG, "Servo: angle=%ld -> pulse=%luus -> duty=%lu",
             (long)angle, (unsigned long)pulse_us, (unsigned long)duty);

    return duty;
}

/**
 * @brief FreeRTOS task for smooth speed ramping.
 *
 * This task runs continuously and gradually adjusts the current motor
 * speed toward the target speed, providing smooth acceleration and
 * deceleration.
 *
 * @param[in] arg    Unused task argument.
 */
static void speed_ramp_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Speed ramp task started");

    while (s_ramp_task_running)
    {
        if (s_state_mutex != NULL)
        {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        }

        bool need_update = false;

        if (s_motor_state.current_speed < s_motor_state.target_speed)
        {
            /* Accelerate. */
            s_motor_state.current_speed += MOTOR_RAMP_STEP;
            if (s_motor_state.current_speed > s_motor_state.target_speed)
            {
                s_motor_state.current_speed = s_motor_state.target_speed;
            }
            need_update = true;
        }
        else if (s_motor_state.current_speed > s_motor_state.target_speed)
        {
            /* Decelerate. */
            s_motor_state.current_speed -= MOTOR_RAMP_STEP;
            if (s_motor_state.current_speed < s_motor_state.target_speed)
            {
                s_motor_state.current_speed = s_motor_state.target_speed;
            }
            need_update = true;
        }

        if (need_update)
        {
            apply_motor_pwm(s_motor_state.current_speed, s_motor_state.direction);
        }

        if (s_state_mutex != NULL)
        {
            xSemaphoreGive(s_state_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_RAMP_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "Speed ramp task stopped");
    vTaskDelete(NULL);
}

/*******************************************************************************
 * Public Function Implementations
 ******************************************************************************/

esp_err_t motor_controller_init(const motor_config_t *config)
{
    if (s_motor_state.initialized)
    {
        ESP_LOGW(TAG, "Motor controller already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Apply configuration. */
    if (config != NULL)
    {
        memcpy(&s_config, config, sizeof(motor_config_t));
    }
    else
    {
        motor_config_t default_config = MOTOR_CONFIG_DEFAULT();
        memcpy(&s_config, &default_config, sizeof(motor_config_t));
    }

    /* Create state mutex. */
    if (s_state_mutex == NULL)
    {
        s_state_mutex = xSemaphoreCreateMutex();
        if (s_state_mutex == NULL)
        {
            ESP_LOGE(TAG, "Failed to create state mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t ret;

    /* Configure motor enable pin. */
    ret = configure_enable_pin();
    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Configure motor PWM channels. */
    ret = configure_motor_pwm();
    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Configure servo PWM channel. */
    ret = configure_servo_pwm();
    if (ret != ESP_OK)
    {
        return ret;
    }

    /* Configure encoder PCNT. */
    ret = configure_encoder_pcnt();
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Encoder config failed (may not be connected): %s",
                 esp_err_to_name(ret));
        /* Non-fatal: encoder may not be connected. */
    }

    /* Initialize state. */
    s_motor_state.target_speed = 0;
    s_motor_state.current_speed = 0;
    s_motor_state.direction = MOTOR_DIR_COAST;
    s_motor_state.servo_angle = 90;
    s_motor_state.encoder_count = 0;
    s_motor_state.prev_encoder_count = 0;
    s_motor_state.rpm = 0.0f;
    s_motor_state.last_rpm_calc_time = esp_timer_get_time();
    s_motor_state.initialized = true;
    s_motor_state.motor_enabled = false;

    /* Start speed ramp task. */
    s_ramp_task_running = true;
    BaseType_t task_ret = xTaskCreate(
        speed_ramp_task,
        "motor_ramp",
        RAMP_TASK_STACK_SIZE,
        NULL,
        RAMP_TASK_PRIORITY,
        &s_ramp_task_handle);

    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create ramp task");
        s_ramp_task_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Motor controller initialized successfully");
    return ESP_OK;
}

esp_err_t motor_controller_deinit(void)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* Stop ramp task. */
    s_ramp_task_running = false;
    if (s_ramp_task_handle != NULL)
    {
        /* Wait for task to finish. */
        vTaskDelay(pdMS_TO_TICKS(MOTOR_RAMP_INTERVAL_MS * 2));
        s_ramp_task_handle = NULL;
    }

    /* Stop motor. */
    apply_motor_pwm(0, MOTOR_DIR_COAST);

    /* Stop servo. */
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, 0);
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);

    /* Disable PCNT. */
    if (s_pcnt_unit != NULL)
    {
        pcnt_unit_stop(s_pcnt_unit);
        pcnt_unit_disable(s_pcnt_unit);
        if (s_pcnt_chan_a != NULL)
        {
            pcnt_del_channel(s_pcnt_chan_a);
            s_pcnt_chan_a = NULL;
        }
        if (s_pcnt_chan_b != NULL)
        {
            pcnt_del_channel(s_pcnt_chan_b);
            s_pcnt_chan_b = NULL;
        }
        pcnt_del_unit(s_pcnt_unit);
        s_pcnt_unit = NULL;
    }

    /* Uninstall LEDC fade service. */
    ledc_fade_func_uninstall();

    /* Reset state. */
    s_motor_state.initialized = false;
    s_motor_state.motor_enabled = false;

    /* Clean up mutex. */
    if (s_state_mutex != NULL)
    {
        vSemaphoreDelete(s_state_mutex);
        s_state_mutex = NULL;
    }

    ESP_LOGI(TAG, "Motor controller deinitialized");
    return ESP_OK;
}

esp_err_t motor_controller_set_speed(int32_t speed_pct)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    speed_pct = clamp_i32(speed_pct, MOTOR_MIN_SPEED_PCT, MOTOR_MAX_SPEED_PCT);

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    s_motor_state.target_speed = speed_pct;
    s_motor_state.motor_enabled = (speed_pct > 0);

    ESP_LOGI(TAG, "Motor target speed set to %ld%%", (long)speed_pct);

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    return ESP_OK;
}

esp_err_t motor_controller_set_direction(motor_direction_t direction)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (direction > MOTOR_DIR_COAST)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    motor_direction_t old_dir = s_motor_state.direction;
    s_motor_state.direction = direction;

    /* If changing between forward/reverse while running, ramp down first. */
    if ((old_dir == MOTOR_DIR_FORWARD && direction == MOTOR_DIR_REVERSE) ||
        (old_dir == MOTOR_DIR_REVERSE && direction == MOTOR_DIR_FORWARD))
    {
        if (s_motor_state.current_speed > 0)
        {
            ESP_LOGI(TAG, "Direction change while running: ramping to 0 first");
            int32_t saved_target = s_motor_state.target_speed;
            s_motor_state.target_speed = 0;

            if (s_state_mutex != NULL)
            {
                xSemaphoreGive(s_state_mutex);
            }

            /* Wait for speed to reach zero. */
            while (s_motor_state.current_speed > 0)
            {
                vTaskDelay(pdMS_TO_TICKS(MOTOR_RAMP_INTERVAL_MS));
            }

            if (s_state_mutex != NULL)
            {
                xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            }

            s_motor_state.target_speed = saved_target;
        }
    }

    /* Apply direction immediately for brake/coast. */
    if (direction == MOTOR_DIR_BRAKE || direction == MOTOR_DIR_COAST)
    {
        apply_motor_pwm(s_motor_state.current_speed, direction);
    }

    ESP_LOGI(TAG, "Motor direction set to: %s", motor_direction_to_str(direction));

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    return ESP_OK;
}

esp_err_t motor_controller_set_servo_angle(int32_t angle)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    angle = clamp_i32(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    uint32_t duty = angle_to_duty(angle);

    esp_err_t ret = ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set servo duty: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to update servo duty: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    s_motor_state.servo_angle = angle;

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    ESP_LOGI(TAG, "Servo angle set to %ld degrees", (long)angle);

    return ESP_OK;
}

esp_err_t motor_controller_emergency_stop(void)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "EMERGENCY STOP activated!");

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    /* Immediately stop motor without ramping. */
    s_motor_state.target_speed = 0;
    s_motor_state.current_speed = 0;
    s_motor_state.direction = MOTOR_DIR_BRAKE;
    s_motor_state.motor_enabled = false;

    /* Apply brake immediately. */
    apply_motor_pwm(0, MOTOR_DIR_BRAKE);

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    /* Move servo to center. */
    motor_controller_set_servo_angle(90);

    return ESP_OK;
}

esp_err_t motor_controller_ramp_update(void)
{
    /* This is handled by the ramp task internally.
     * This function is provided for manual ramp control if the task is not used.
     */
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    if (s_motor_state.current_speed < s_motor_state.target_speed)
    {
        s_motor_state.current_speed += MOTOR_RAMP_STEP;
        if (s_motor_state.current_speed > s_motor_state.target_speed)
        {
            s_motor_state.current_speed = s_motor_state.target_speed;
        }
        apply_motor_pwm(s_motor_state.current_speed, s_motor_state.direction);
    }
    else if (s_motor_state.current_speed > s_motor_state.target_speed)
    {
        s_motor_state.current_speed -= MOTOR_RAMP_STEP;
        if (s_motor_state.current_speed < s_motor_state.target_speed)
        {
            s_motor_state.current_speed = s_motor_state.target_speed;
        }
        apply_motor_pwm(s_motor_state.current_speed, s_motor_state.direction);
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    return ESP_OK;
}

esp_err_t motor_controller_read_encoder(int32_t *count)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_pcnt_unit == NULL)
    {
        /* Encoder not configured. */
        if (count != NULL)
        {
            *count = 0;
        }
        return ESP_OK;
    }

    int pulse_count = 0;
    esp_err_t ret = pcnt_unit_get_count(s_pcnt_unit, &pulse_count);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read PCNT: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    s_motor_state.encoder_count = pulse_count;

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    if (count != NULL)
    {
        *count = pulse_count;
    }

    return ESP_OK;
}

esp_err_t motor_controller_reset_encoder(void)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_pcnt_unit == NULL)
    {
        return ESP_OK;
    }

    esp_err_t ret = pcnt_unit_clear_count(s_pcnt_unit);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear PCNT: %s", esp_err_to_name(ret));
        return ret;
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    s_motor_state.encoder_count = 0;
    s_motor_state.prev_encoder_count = 0;

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    return ESP_OK;
}

esp_err_t motor_controller_calc_rpm(float *rpm)
{
    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t now = esp_timer_get_time();

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    int64_t elapsed_us = now - s_motor_state.last_rpm_calc_time;

    if (elapsed_us <= 0)
    {
        if (s_state_mutex != NULL)
        {
            xSemaphoreGive(s_state_mutex);
        }
        if (rpm != NULL)
        {
            *rpm = s_motor_state.rpm;
        }
        return ESP_OK;
    }

    /* Calculate pulses since last measurement. */
    int32_t delta_pulses = s_motor_state.encoder_count -
                           s_motor_state.prev_encoder_count;

    /* Convert to RPM: (pulses / PPR) * (60,000,000 / elapsed_us). */
    float elapsed_seconds = (float)elapsed_us / 1000000.0f;
    float revolutions = (float)abs(delta_pulses) / (float)s_config.encoder_ppr;
    float calculated_rpm = (elapsed_seconds > 0.0f)
                               ? (revolutions / elapsed_seconds) * 60.0f
                               : 0.0f;

    s_motor_state.rpm = calculated_rpm;
    s_motor_state.prev_encoder_count = s_motor_state.encoder_count;
    s_motor_state.last_rpm_calc_time = now;

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    if (rpm != NULL)
    {
        *rpm = calculated_rpm;
    }

    return ESP_OK;
}

esp_err_t motor_controller_get_state(motor_state_t *state)
{
    if (state == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_motor_state.initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state_mutex != NULL)
    {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    }

    memcpy(state, &s_motor_state, sizeof(motor_state_t));

    if (s_state_mutex != NULL)
    {
        xSemaphoreGive(s_state_mutex);
    }

    return ESP_OK;
}

const char *motor_direction_to_str(motor_direction_t dir)
{
    switch (dir)
    {
    case MOTOR_DIR_FORWARD:
        return "forward";
    case MOTOR_DIR_REVERSE:
        return "reverse";
    case MOTOR_DIR_BRAKE:
        return "brake";
    case MOTOR_DIR_COAST:
        return "coast";
    default:
        return "unknown";
    }
}

motor_direction_t motor_direction_from_str(const char *str)
{
    if (str == NULL)
    {
        return MOTOR_DIR_COAST;
    }

    if (strcmp(str, "forward") == 0)
        return MOTOR_DIR_FORWARD;
    if (strcmp(str, "reverse") == 0)
        return MOTOR_DIR_REVERSE;
    if (strcmp(str, "brake") == 0)
        return MOTOR_DIR_BRAKE;
    if (strcmp(str, "coast") == 0)
        return MOTOR_DIR_COAST;

    ESP_LOGW(TAG, "Unknown direction string: %s", str);
    return MOTOR_DIR_COAST;
}
