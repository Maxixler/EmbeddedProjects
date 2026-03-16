/**
 * @file motor_controller.h
 * @brief Motor and servo controller using ESP32 LEDC PWM peripheral.
 *
 * This module provides a hardware abstraction layer for controlling:
 *  - DC motor via L298N H-bridge driver (speed and direction control)
 *  - Servo motor (SG90) via direct PWM signal (angle control)
 *  - Rotary encoder position feedback via PCNT peripheral
 *
 * The LEDC peripheral is used for PWM generation:
 *  - DC motor: Two PWM channels driving IN1/IN2 of L298N at configurable frequency.
 *  - Servo: One PWM channel at 50Hz with 500-2500us pulse width range.
 *
 * Features:
 *  - Smooth speed ramping (acceleration/deceleration)
 *  - Four motor states: forward, reverse, brake, coast
 *  - Servo angle control with configurable min/max pulse widths
 *  - Encoder pulse counting using hardware PCNT unit
 *  - RPM calculation from encoder readings
 *
 * @note GPIO pin assignments are configurable via preprocessor defines.
 *
 * @author EmbeddedProjects
 * @date 2026
 * @version 1.0.0
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*******************************************************************************
 * Preprocessor Definitions - GPIO Pin Assignments
 ******************************************************************************/

/** @defgroup motor_pins DC Motor GPIO Pins (L298N)
 *  @{
 */
/** @brief GPIO pin for L298N IN1 (motor forward PWM). */
#define MOTOR_PIN_IN1 GPIO_NUM_25

/** @brief GPIO pin for L298N IN2 (motor reverse PWM). */
#define MOTOR_PIN_IN2 GPIO_NUM_26

/** @brief GPIO pin for L298N ENA (enable, active high). */
#define MOTOR_PIN_ENA GPIO_NUM_27
/** @} */

/** @defgroup servo_pins Servo Motor GPIO Pins (SG90)
 *  @{
 */
/** @brief GPIO pin for servo PWM signal. */
#define SERVO_PIN_PWM GPIO_NUM_18
/** @} */

/** @defgroup encoder_pins Rotary Encoder GPIO Pins
 *  @{
 */
/** @brief GPIO pin for encoder channel A. */
#define ENCODER_PIN_A GPIO_NUM_34

/** @brief GPIO pin for encoder channel B. */
#define ENCODER_PIN_B GPIO_NUM_35
/** @} */

/*******************************************************************************
 * Preprocessor Definitions - PWM Configuration
 ******************************************************************************/

/** @defgroup motor_pwm DC Motor PWM Configuration
 *  @{
 */
/** @brief PWM frequency for DC motor control (Hz). */
#define MOTOR_PWM_FREQ_HZ 1000

/** @brief PWM resolution for DC motor (bits). 13-bit = 0-8191 range. */
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_13_BIT

/** @brief Maximum duty cycle value based on resolution. */
#define MOTOR_PWM_MAX_DUTY ((1 << 13) - 1)

/** @brief LEDC timer used for DC motor PWM. */
#define MOTOR_LEDC_TIMER LEDC_TIMER_0

/** @brief LEDC speed mode for DC motor. */
#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE

/** @brief LEDC channel for motor IN1. */
#define MOTOR_LEDC_CHANNEL_IN1 LEDC_CHANNEL_0

/** @brief LEDC channel for motor IN2. */
#define MOTOR_LEDC_CHANNEL_IN2 LEDC_CHANNEL_1
/** @} */

/** @defgroup servo_pwm Servo PWM Configuration
 *  @{
 */
/** @brief PWM frequency for servo control (Hz). Standard servo = 50Hz. */
#define SERVO_PWM_FREQ_HZ 50

/** @brief PWM resolution for servo (bits). 16-bit for fine granularity. */
#define SERVO_PWM_RESOLUTION LEDC_TIMER_16_BIT

/** @brief Maximum duty cycle value for servo based on resolution. */
#define SERVO_PWM_MAX_DUTY ((1 << 16) - 1)

/** @brief LEDC timer used for servo PWM. */
#define SERVO_LEDC_TIMER LEDC_TIMER_1

/** @brief LEDC speed mode for servo. */
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE

/** @brief LEDC channel for servo. */
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_2

/** @brief Minimum pulse width for servo in microseconds (0 degrees). */
#define SERVO_MIN_PULSE_US 500

/** @brief Maximum pulse width for servo in microseconds (180 degrees). */
#define SERVO_MAX_PULSE_US 2500

/** @brief Minimum servo angle in degrees. */
#define SERVO_MIN_ANGLE 0

/** @brief Maximum servo angle in degrees. */
#define SERVO_MAX_ANGLE 180
/** @} */

/** @defgroup speed_ramp Speed Ramping Configuration
 *  @{
 */
/** @brief Ramp step increment per tick (percentage points). */
#define MOTOR_RAMP_STEP 2

/** @brief Ramp update interval in milliseconds. */
#define MOTOR_RAMP_INTERVAL_MS 20

/** @brief Minimum speed percentage (below this, motor is stopped). */
#define MOTOR_MIN_SPEED_PCT 0

/** @brief Maximum speed percentage. */
#define MOTOR_MAX_SPEED_PCT 100
/** @} */

/** @defgroup encoder_config Encoder Configuration
 *  @{
 */
/** @brief PCNT unit number for encoder. */
#define ENCODER_PCNT_UNIT PCNT_UNIT_0

/** @brief Encoder pulses per revolution (PPR). */
#define ENCODER_PPR 20

/** @brief PCNT high limit for counter overflow. */
#define ENCODER_PCNT_HIGH_LIMIT 10000

/** @brief PCNT low limit for counter underflow. */
#define ENCODER_PCNT_LOW_LIMIT (-10000)

/** @brief Glitch filter threshold in APB clock cycles (12.5ns each at 80MHz). */
#define ENCODER_GLITCH_FILTER_NS 1000
    /** @} */

    /*******************************************************************************
     * Type Definitions
     ******************************************************************************/

    /**
     * @brief Motor direction states.
     */
    typedef enum
    {
        MOTOR_DIR_FORWARD = 0, /**< Motor spinning forward (IN1=PWM, IN2=LOW). */
        MOTOR_DIR_REVERSE,     /**< Motor spinning reverse (IN1=LOW, IN2=PWM). */
        MOTOR_DIR_BRAKE,       /**< Active braking (IN1=HIGH, IN2=HIGH). */
        MOTOR_DIR_COAST        /**< Coast / free-running (IN1=LOW, IN2=LOW). */
    } motor_direction_t;

    /**
     * @brief Motor controller runtime state.
     *
     * This structure maintains the current state of the motor controller
     * including target and actual speed values for smooth ramping.
     */
    typedef struct
    {
        int32_t target_speed;        /**< Target speed percentage (0-100). */
        int32_t current_speed;       /**< Current (ramped) speed percentage. */
        motor_direction_t direction; /**< Current motor direction. */
        int32_t servo_angle;         /**< Current servo angle (0-180 degrees). */
        int32_t encoder_count;       /**< Current encoder pulse count. */
        int32_t prev_encoder_count;  /**< Previous encoder count for RPM calc. */
        float rpm;                   /**< Calculated RPM. */
        int64_t last_rpm_calc_time;  /**< Timestamp of last RPM calculation (us). */
        bool initialized;            /**< True if controller has been initialized. */
        bool motor_enabled;          /**< True if motor output is enabled. */
    } motor_state_t;

    /**
     * @brief Motor controller configuration structure.
     *
     * Passed to motor_controller_init() to override default pin assignments
     * and PWM parameters if needed.
     */
    typedef struct
    {
        int pin_in1;             /**< GPIO for L298N IN1. */
        int pin_in2;             /**< GPIO for L298N IN2. */
        int pin_ena;             /**< GPIO for L298N ENA. */
        int pin_servo;           /**< GPIO for servo PWM. */
        int pin_enc_a;           /**< GPIO for encoder channel A. */
        int pin_enc_b;           /**< GPIO for encoder channel B. */
        uint32_t motor_pwm_freq; /**< Motor PWM frequency (Hz). */
        uint32_t encoder_ppr;    /**< Encoder pulses per revolution. */
    } motor_config_t;

/*******************************************************************************
 * Macros
 ******************************************************************************/

/**
 * @brief Default motor controller configuration using predefined pin macros.
 */
#define MOTOR_CONFIG_DEFAULT() {         \
    .pin_in1 = MOTOR_PIN_IN1,            \
    .pin_in2 = MOTOR_PIN_IN2,            \
    .pin_ena = MOTOR_PIN_ENA,            \
    .pin_servo = SERVO_PIN_PWM,          \
    .pin_enc_a = ENCODER_PIN_A,          \
    .pin_enc_b = ENCODER_PIN_B,          \
    .motor_pwm_freq = MOTOR_PWM_FREQ_HZ, \
    .encoder_ppr = ENCODER_PPR}

    /*******************************************************************************
     * Public Function Prototypes
     ******************************************************************************/

    /**
     * @brief Initialize the motor controller subsystem.
     *
     * Configures LEDC PWM channels for DC motor and servo control,
     * sets up PCNT for encoder reading, and enables the motor driver.
     *
     * @param[in] config    Pointer to configuration structure. Pass NULL to use defaults.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if already initialized.
     *  - ESP_ERR_NO_MEM if memory allocation failed.
     *  - ESP_FAIL on peripheral configuration error.
     */
    esp_err_t motor_controller_init(const motor_config_t *config);

    /**
     * @brief Deinitialize the motor controller and release all resources.
     *
     * Stops motor and servo outputs, disables PCNT, and releases GPIO pins.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_deinit(void);

    /**
     * @brief Set the target motor speed with smooth ramping.
     *
     * The actual speed will ramp toward the target value at the rate
     * defined by MOTOR_RAMP_STEP and MOTOR_RAMP_INTERVAL_MS.
     *
     * @param[in] speed_pct    Target speed percentage (0-100). Values outside
     *                          this range will be clamped.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_set_speed(int32_t speed_pct);

    /**
     * @brief Set the motor direction.
     *
     * Changes the direction of the DC motor. If the motor is currently
     * running, it will smoothly decelerate to zero before changing direction
     * and then accelerate to the target speed.
     *
     * @param[in] direction    Desired motor direction.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_ARG if direction is invalid.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_set_direction(motor_direction_t direction);

    /**
     * @brief Set the servo angle.
     *
     * Converts the angle to the appropriate LEDC duty cycle based on
     * the configured pulse width range (500-2500us at 50Hz).
     *
     * @param[in] angle    Servo angle in degrees (0-180). Values outside
     *                      this range will be clamped.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_set_servo_angle(int32_t angle);

    /**
     * @brief Emergency stop - immediately halt motor and reset servo to center.
     *
     * Sets motor speed to zero without ramping, applies brake, and moves
     * servo to 90 degrees (center position).
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_emergency_stop(void);

    /**
     * @brief Update the speed ramp (call periodically from a task).
     *
     * This function should be called at regular intervals (MOTOR_RAMP_INTERVAL_MS)
     * to smoothly adjust the current speed toward the target speed.
     *
     * @return
     *  - ESP_OK if ramp update was applied.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_ramp_update(void);

    /**
     * @brief Read the current encoder count.
     *
     * Reads the hardware PCNT counter value and updates the internal state.
     *
     * @param[out] count    Pointer to store the encoder count. Can be NULL
     *                       if only internal state update is needed.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_read_encoder(int32_t *count);

    /**
     * @brief Reset the encoder count to zero.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_reset_encoder(void);

    /**
     * @brief Calculate RPM from encoder readings.
     *
     * Uses the difference in encoder counts over the elapsed time since
     * the last call to compute the motor RPM.
     *
     * @param[out] rpm    Pointer to store the calculated RPM. Can be NULL
     *                     if only internal state update is needed.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_calc_rpm(float *rpm);

    /**
     * @brief Get the current motor controller state.
     *
     * Returns a snapshot of the current motor state including speed,
     * direction, servo angle, and encoder readings.
     *
     * @param[out] state    Pointer to a motor_state_t structure to fill.
     *
     * @return
     *  - ESP_OK on success.
     *  - ESP_ERR_INVALID_ARG if state is NULL.
     *  - ESP_ERR_INVALID_STATE if not initialized.
     */
    esp_err_t motor_controller_get_state(motor_state_t *state);

    /**
     * @brief Convert a motor direction enum to a human-readable string.
     *
     * @param[in] dir    Motor direction enum value.
     *
     * @return Null-terminated string representation (e.g., "forward", "reverse").
     */
    const char *motor_direction_to_str(motor_direction_t dir);

    /**
     * @brief Convert a direction string to the corresponding enum value.
     *
     * @param[in] str    Null-terminated direction string ("forward", "reverse",
     *                    "brake", or "coast").
     *
     * @return Corresponding motor_direction_t value, or MOTOR_DIR_COAST if
     *         the string is not recognized.
     */
    motor_direction_t motor_direction_from_str(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROLLER_H */
