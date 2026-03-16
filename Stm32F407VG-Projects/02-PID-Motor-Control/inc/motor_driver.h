/**
 * @file    motor_driver.h
 * @brief   DC Motor Driver Interface using H-Bridge (L298N / TB6612FNG)
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details This module provides a hardware abstraction layer for DC motor control
 *          via an H-bridge motor driver. It generates PWM using TIM1 Channel 1
 *          (PA8) and controls direction via GPIO pins PB12 (IN1) and PB13 (IN2).
 *
 *          Features:
 *          - 20kHz PWM output (inaudible switching frequency)
 *          - Bidirectional motor control (forward, reverse, brake, coast)
 *          - Soft-start ramp for mechanical protection
 *          - Dead-time insertion via TIM1 BDTR register
 *          - Emergency stop function
 *          - Duty cycle range: 0-100%
 *
 * @note    Hardware connections:
 *          - PA8  -> TIM1_CH1 -> H-Bridge ENA (PWM input)
 *          - PB12 -> GPIO     -> H-Bridge IN1 (direction)
 *          - PB13 -> GPIO     -> H-Bridge IN2 (direction)
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* -------------------------------------------------------------------------- */
    /*                              Includes                                      */
    /* -------------------------------------------------------------------------- */

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                              Definitions                                   */
/* -------------------------------------------------------------------------- */

/** PWM Configuration */
#define MOTOR_PWM_FREQUENCY_HZ 20000U                                               /**< 20kHz PWM (inaudible) */
#define MOTOR_PWM_TIMER_CLOCK_HZ 168000000U                                         /**< TIM1 clock (APB2*2) */
#define MOTOR_PWM_PRESCALER 0U                                                      /**< No prescaling */
#define MOTOR_PWM_PERIOD ((MOTOR_PWM_TIMER_CLOCK_HZ / MOTOR_PWM_FREQUENCY_HZ) - 1U) /* 8399 */

/** Dead-time configuration for H-bridge safety (TIM1 BDTR) */
#define MOTOR_DEADTIME_VALUE 32U /**< ~190ns at 168MHz */

/** Direction control GPIO pins */
#define MOTOR_IN1_PORT GPIOB
#define MOTOR_IN1_PIN GPIO_PIN_12
#define MOTOR_IN2_PORT GPIOB
#define MOTOR_IN2_PIN GPIO_PIN_13

/** Soft-start configuration */
#define MOTOR_SOFT_START_STEP 2.0f   /**< Duty cycle increment per ms */
#define MOTOR_SOFT_START_INTERVAL 1U /**< Ramp step interval in ms */

/** Safety limits */
#define MOTOR_MAX_DUTY_CYCLE 100.0f        /**< Maximum duty cycle (%) */
#define MOTOR_MIN_DUTY_CYCLE 0.0f          /**< Minimum duty cycle (%) */
#define MOTOR_STALL_CURRENT_THRESHOLD 4.5f /**< Stall current threshold (A) */

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Motor operating state enumeration
     */
    typedef enum
    {
        MOTOR_STATE_STOPPED = 0, /**< Motor is stopped (PWM = 0)                  */
        MOTOR_STATE_FORWARD = 1, /**< Motor spinning forward (IN1=1, IN2=0, PWM)  */
        MOTOR_STATE_REVERSE = 2, /**< Motor spinning reverse (IN1=0, IN2=1, PWM)  */
        MOTOR_STATE_BRAKE = 3,   /**< Active braking (IN1=1, IN2=1)               */
        MOTOR_STATE_COAST = 4,   /**< Coasting / free-running (IN1=0, IN2=0)      */
        MOTOR_STATE_FAULT = 5    /**< Fault condition (overcurrent, stall, etc.)   */
    } Motor_State_t;

    /**
     * @brief Motor direction enumeration
     */
    typedef enum
    {
        MOTOR_DIR_FORWARD = 0, /**< Forward direction (CW)  */
        MOTOR_DIR_REVERSE = 1  /**< Reverse direction (CCW) */
    } Motor_Direction_t;

    /**
     * @brief Motor driver handle structure
     *
     * Contains all configuration and runtime state for one motor channel.
     */
    typedef struct
    {
        /* ---- Hardware Handles ---- */
        TIM_HandleTypeDef *htim_pwm; /**< Timer handle for PWM generation     */
        uint32_t pwm_channel;        /**< Timer channel (TIM_CHANNEL_1, etc.) */

        /* ---- Configuration ---- */
        uint32_t pwm_frequency; /**< PWM frequency in Hz                 */
        uint32_t pwm_period;    /**< Auto-reload value (ARR)             */
        float max_duty;         /**< Maximum allowed duty cycle (%)      */

        /* ---- Runtime State ---- */
        Motor_State_t state;         /**< Current motor state                 */
        Motor_Direction_t direction; /**< Current motor direction             */
        float duty_cycle;            /**< Current duty cycle (0..100%)        */
        float target_duty;           /**< Target duty for soft-start ramp     */
        bool soft_start_active;      /**< Soft-start ramp in progress      */
        bool emergency_stop;         /**< Emergency stop flag                 */
        bool initialized;            /**< Initialization complete flag        */
    } Motor_Handle_t;

    /* -------------------------------------------------------------------------- */
    /*                          Function Prototypes                               */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief  Initialize the motor driver
     *
     * Configures TIM1 CH1 for PWM output at 20kHz, sets up direction
     * control GPIO pins (PB12, PB13), and configures dead-time.
     * Motor starts in STOPPED state with 0% duty cycle.
     *
     * @param  hmotor  Pointer to motor handle structure
     * @param  htim    Pointer to HAL TIM handle (must be TIM1)
     * @param  channel Timer channel (typically TIM_CHANNEL_1)
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Motor_Init(Motor_Handle_t *hmotor,
                                 TIM_HandleTypeDef *htim,
                                 uint32_t channel);

    /**
     * @brief  Set motor speed (duty cycle) and direction
     *
     * Sets the PWM duty cycle for motor speed control. Positive values
     * drive forward, negative values drive reverse. Uses soft-start
     * if enabled.
     *
     * @param  hmotor      Pointer to motor handle
     * @param  duty_cycle  Duty cycle percentage (-100.0 to +100.0)
     *                     Positive = forward, Negative = reverse
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Motor_SetSpeed(Motor_Handle_t *hmotor, float duty_cycle);

    /**
     * @brief  Set motor direction without changing speed
     *
     * @param  hmotor     Pointer to motor handle
     * @param  direction  MOTOR_DIR_FORWARD or MOTOR_DIR_REVERSE
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Motor_SetDirection(Motor_Handle_t *hmotor,
                                         Motor_Direction_t direction);

    /**
     * @brief  Apply active brake (both H-bridge outputs high)
     *
     * Active braking shorts the motor terminals, providing maximum
     * braking torque. The motor decelerates quickly.
     *
     * @param  hmotor  Pointer to motor handle
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Motor_Brake(Motor_Handle_t *hmotor);

    /**
     * @brief  Set motor to coast (free-run, no braking)
     *
     * Both H-bridge outputs are set low, allowing the motor to
     * spin freely until friction stops it.
     *
     * @param  hmotor  Pointer to motor handle
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Motor_Coast(Motor_Handle_t *hmotor);

    /**
     * @brief  Emergency stop - immediately disable all outputs
     *
     * Sets PWM to 0 and applies brake. This function is designed to
     * be called from fault handlers or when immediate stop is required.
     * After emergency stop, Motor_Init() or Motor_ClearFault() must
     * be called to resume operation.
     *
     * @param  hmotor  Pointer to motor handle
     */
    void Motor_EmergencyStop(Motor_Handle_t *hmotor);

    /**
     * @brief  Clear fault condition and return to stopped state
     *
     * @param  hmotor  Pointer to motor handle
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Motor_ClearFault(Motor_Handle_t *hmotor);

    /**
     * @brief  Get current motor state
     *
     * @param  hmotor  Pointer to motor handle
     * @return Motor_State_t  Current motor state
     */
    Motor_State_t Motor_GetState(const Motor_Handle_t *hmotor);

    /**
     * @brief  Get current duty cycle
     *
     * @param  hmotor  Pointer to motor handle
     * @return float   Current duty cycle percentage (0..100)
     */
    float Motor_GetDutyCycle(const Motor_Handle_t *hmotor);

    /**
     * @brief  Get current direction
     *
     * @param  hmotor  Pointer to motor handle
     * @return Motor_Direction_t  Current direction
     */
    Motor_Direction_t Motor_GetDirection(const Motor_Handle_t *hmotor);

    /**
     * @brief  Update soft-start ramp (call from periodic timer, e.g., 1ms SysTick)
     *
     * If soft-start is active, this function gradually ramps the duty cycle
     * from the current value to the target value. Should be called periodically
     * at MOTOR_SOFT_START_INTERVAL rate.
     *
     * @param  hmotor  Pointer to motor handle
     */
    void Motor_SoftStartUpdate(Motor_Handle_t *hmotor);

    /**
     * @brief  Set maximum allowed duty cycle
     *
     * @param  hmotor    Pointer to motor handle
     * @param  max_duty  Maximum duty cycle percentage (0..100)
     */
    void Motor_SetMaxDuty(Motor_Handle_t *hmotor, float max_duty);

    /**
     * @brief  Check if motor driver is initialized
     *
     * @param  hmotor  Pointer to motor handle
     * @return bool    true if initialized
     */
    bool Motor_IsInitialized(const Motor_Handle_t *hmotor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
