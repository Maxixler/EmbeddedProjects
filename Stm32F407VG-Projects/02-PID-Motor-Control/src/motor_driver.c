/**
 * @file    motor_driver.c
 * @brief   DC Motor Driver Implementation using H-Bridge with TIM1 PWM
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details Implements DC motor control through an H-bridge driver circuit:
 *
 *          Hardware configuration:
 *          - TIM1 CH1 (PA8): PWM output at 20kHz
 *          - PB12: H-Bridge IN1 (direction control)
 *          - PB13: H-Bridge IN2 (direction control)
 *
 *          Direction truth table (L298N / TB6612FNG):
 *          | IN1 | IN2 | ENA(PWM) | Action     |
 *          |-----|-----|----------|------------|
 *          |  1  |  0  |   PWM    | Forward    |
 *          |  0  |  1  |   PWM    | Reverse    |
 *          |  1  |  1  |   X      | Brake      |
 *          |  0  |  0  |   X      | Coast      |
 *
 *          Safety features:
 *          - Dead-time via TIM1 BDTR (190ns, prevents H-bridge shoot-through)
 *          - Soft-start ramp (gradual duty cycle increase)
 *          - Emergency stop (immediate PWM disable + brake)
 *          - Maximum duty cycle limiting
 */

/* -------------------------------------------------------------------------- */
/*                              Includes                                      */
/* -------------------------------------------------------------------------- */

#include "motor_driver.h"
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                          Private Helper Functions                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Set H-bridge direction pins
 *
 * @param  in1_state  State for IN1 pin (GPIO_PIN_SET or GPIO_PIN_RESET)
 * @param  in2_state  State for IN2 pin (GPIO_PIN_SET or GPIO_PIN_RESET)
 */
static void Motor_SetDirectionPins(GPIO_PinState in1_state, GPIO_PinState in2_state)
{
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, in1_state);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, in2_state);
}

/**
 * @brief  Set PWM compare value from duty cycle percentage
 *
 * @param  hmotor      Pointer to motor handle
 * @param  duty_cycle  Duty cycle percentage (0..100)
 */
static void Motor_SetPWMDuty(Motor_Handle_t *hmotor, float duty_cycle)
{
    /* Clamp duty cycle to valid range */
    if (duty_cycle < MOTOR_MIN_DUTY_CYCLE)
    {
        duty_cycle = MOTOR_MIN_DUTY_CYCLE;
    }
    if (duty_cycle > hmotor->max_duty)
    {
        duty_cycle = hmotor->max_duty;
    }

    /* Convert percentage to timer compare value */
    uint32_t compare_value = (uint32_t)((duty_cycle / 100.0f) * (float)(hmotor->pwm_period + 1));

    /* Ensure compare value does not exceed period */
    if (compare_value > hmotor->pwm_period)
    {
        compare_value = hmotor->pwm_period;
    }

    /* Set the capture/compare register */
    __HAL_TIM_SET_COMPARE(hmotor->htim_pwm, hmotor->pwm_channel, compare_value);

    /* Update stored duty cycle */
    hmotor->duty_cycle = duty_cycle;
}

/* -------------------------------------------------------------------------- */
/*                          Public Function Implementations                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize the motor driver
 */
HAL_StatusTypeDef Motor_Init(Motor_Handle_t *hmotor,
                             TIM_HandleTypeDef *htim,
                             uint32_t channel)
{
    if (hmotor == NULL || htim == NULL)
    {
        return HAL_ERROR;
    }

    /* Store hardware handles */
    hmotor->htim_pwm = htim;
    hmotor->pwm_channel = channel;

    /* Store configuration */
    hmotor->pwm_frequency = MOTOR_PWM_FREQUENCY_HZ;
    hmotor->pwm_period = MOTOR_PWM_PERIOD;
    hmotor->max_duty = MOTOR_MAX_DUTY_CYCLE;

    /* Initialize state */
    hmotor->state = MOTOR_STATE_STOPPED;
    hmotor->direction = MOTOR_DIR_FORWARD;
    hmotor->duty_cycle = 0.0f;
    hmotor->target_duty = 0.0f;
    hmotor->soft_start_active = false;
    hmotor->emergency_stop = false;
    hmotor->initialized = false;

    /* -------------------------------------------------------------------- */
    /* Configure TIM1 for PWM output                                        */
    /* -------------------------------------------------------------------- */

    /*
     * TIM1 Configuration (done via HAL / CubeMX, verified here):
     * - Clock source: Internal (168 MHz from APB2)
     * - Prescaler: 0 (no prescaling)
     * - Counter mode: Up
     * - Period (ARR): 8399 -> 168MHz / 8400 = 20kHz
     * - PWM Mode 1: Active when CNT < CCR
     * - Auto-reload preload: Enabled
     */

    /* Ensure timer period matches our expected value */
    __HAL_TIM_SET_AUTORELOAD(hmotor->htim_pwm, MOTOR_PWM_PERIOD);

    /* Set initial duty cycle to 0 */
    __HAL_TIM_SET_COMPARE(hmotor->htim_pwm, hmotor->pwm_channel, 0);

    /* -------------------------------------------------------------------- */
    /* Configure direction control GPIO pins                                */
    /* -------------------------------------------------------------------- */

    /*
     * GPIO Configuration (PB12 and PB13):
     * - Mode: Output Push-Pull
     * - Pull: No pull-up, no pull-down
     * - Speed: High
     *
     * Note: GPIO clock and pin configuration should be done in
     *       HAL_MspInit() or CubeMX-generated code. Here we just
     *       set the initial state.
     */

    /* Set motor to stopped state (both direction pins low = coast) */
    Motor_SetDirectionPins(GPIO_PIN_RESET, GPIO_PIN_RESET);

    /* -------------------------------------------------------------------- */
    /* Start PWM output                                                     */
    /* -------------------------------------------------------------------- */

    /*
     * For TIM1 (advanced timer), we need to enable the main output
     * using __HAL_TIM_MOE_ENABLE() in addition to starting PWM.
     * The dead-time is configured in CubeMX via the BDTR register.
     */
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(hmotor->htim_pwm, hmotor->pwm_channel);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Enable main output for TIM1 (advanced timer requirement) */
    __HAL_TIM_MOE_ENABLE(hmotor->htim_pwm);

    /* Mark as initialized */
    hmotor->initialized = true;

    return HAL_OK;
}

/**
 * @brief  Set motor speed (duty cycle) and direction
 *
 * Positive duty_cycle values drive the motor forward.
 * Negative duty_cycle values drive the motor in reverse.
 * Zero stops the motor (coast mode).
 */
HAL_StatusTypeDef Motor_SetSpeed(Motor_Handle_t *hmotor, float duty_cycle)
{
    if (hmotor == NULL || !hmotor->initialized)
    {
        return HAL_ERROR;
    }

    /* Do not allow speed changes during emergency stop */
    if (hmotor->emergency_stop)
    {
        return HAL_ERROR;
    }

    /* Determine direction from sign of duty_cycle */
    Motor_Direction_t new_direction;
    float abs_duty;

    if (duty_cycle >= 0.0f)
    {
        new_direction = MOTOR_DIR_FORWARD;
        abs_duty = duty_cycle;
    }
    else
    {
        new_direction = MOTOR_DIR_REVERSE;
        abs_duty = -duty_cycle;
    }

    /* Clamp absolute duty cycle */
    if (abs_duty > hmotor->max_duty)
    {
        abs_duty = hmotor->max_duty;
    }

    /* Handle near-zero duty cycle as stop */
    if (abs_duty < 0.5f)
    {
        /* Duty cycle too small to drive motor; coast */
        Motor_SetDirectionPins(GPIO_PIN_RESET, GPIO_PIN_RESET);
        Motor_SetPWMDuty(hmotor, 0.0f);
        hmotor->state = MOTOR_STATE_STOPPED;
        hmotor->soft_start_active = false;
        return HAL_OK;
    }

    /*
     * If direction is changing while motor is running, briefly brake
     * to avoid abrupt mechanical stress. In a real application, you
     * might want a more sophisticated transition (decelerate, stop,
     * then accelerate in new direction).
     */
    if (hmotor->state == MOTOR_STATE_FORWARD && new_direction == MOTOR_DIR_REVERSE)
    {
        /* Brief brake before direction change */
        Motor_SetDirectionPins(GPIO_PIN_SET, GPIO_PIN_SET);
        Motor_SetPWMDuty(hmotor, 0.0f);
        /* Small delay would be ideal here, but we avoid blocking delays.
         * The soft-start ramp will handle the gradual transition. */
    }
    else if (hmotor->state == MOTOR_STATE_REVERSE && new_direction == MOTOR_DIR_FORWARD)
    {
        Motor_SetDirectionPins(GPIO_PIN_SET, GPIO_PIN_SET);
        Motor_SetPWMDuty(hmotor, 0.0f);
    }

    /* Set direction pins */
    if (new_direction == MOTOR_DIR_FORWARD)
    {
        Motor_SetDirectionPins(GPIO_PIN_SET, GPIO_PIN_RESET); /* IN1=1, IN2=0 */
        hmotor->state = MOTOR_STATE_FORWARD;
    }
    else
    {
        Motor_SetDirectionPins(GPIO_PIN_RESET, GPIO_PIN_SET); /* IN1=0, IN2=1 */
        hmotor->state = MOTOR_STATE_REVERSE;
    }

    hmotor->direction = new_direction;

    /*
     * Check if soft-start is needed:
     * Soft-start is used when the duty cycle increase is large (> 20%)
     * to protect the motor and mechanical coupling from sudden torque.
     */
    float duty_delta = abs_duty - hmotor->duty_cycle;

    if (duty_delta > 20.0f && hmotor->duty_cycle < 5.0f)
    {
        /* Large step from near-zero: enable soft-start ramp */
        hmotor->target_duty = abs_duty;
        hmotor->soft_start_active = true;
        /* Soft-start will be handled by Motor_SoftStartUpdate() */
    }
    else
    {
        /* Small change or already running: apply immediately */
        Motor_SetPWMDuty(hmotor, abs_duty);
        hmotor->soft_start_active = false;
    }

    return HAL_OK;
}

/**
 * @brief  Set motor direction without changing speed
 */
HAL_StatusTypeDef Motor_SetDirection(Motor_Handle_t *hmotor,
                                     Motor_Direction_t direction)
{
    if (hmotor == NULL || !hmotor->initialized)
    {
        return HAL_ERROR;
    }

    if (hmotor->emergency_stop)
    {
        return HAL_ERROR;
    }

    hmotor->direction = direction;

    if (direction == MOTOR_DIR_FORWARD)
    {
        Motor_SetDirectionPins(GPIO_PIN_SET, GPIO_PIN_RESET);
        if (hmotor->duty_cycle > 0.0f)
        {
            hmotor->state = MOTOR_STATE_FORWARD;
        }
    }
    else
    {
        Motor_SetDirectionPins(GPIO_PIN_RESET, GPIO_PIN_SET);
        if (hmotor->duty_cycle > 0.0f)
        {
            hmotor->state = MOTOR_STATE_REVERSE;
        }
    }

    return HAL_OK;
}

/**
 * @brief  Apply active brake
 *
 * Active braking: both direction pins HIGH, shorting the motor terminals.
 * This provides maximum electromagnetic braking torque.
 */
HAL_StatusTypeDef Motor_Brake(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL || !hmotor->initialized)
    {
        return HAL_ERROR;
    }

    /* Set direction pins to brake mode */
    Motor_SetDirectionPins(GPIO_PIN_SET, GPIO_PIN_SET);

    /* Apply full duty cycle for maximum braking */
    Motor_SetPWMDuty(hmotor, 100.0f);

    /* Update state */
    hmotor->state = MOTOR_STATE_BRAKE;
    hmotor->soft_start_active = false;

    return HAL_OK;
}

/**
 * @brief  Set motor to coast (free-run)
 *
 * Coast mode: both direction pins LOW, motor disconnected from supply.
 * Motor spins freely until friction stops it.
 */
HAL_StatusTypeDef Motor_Coast(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL || !hmotor->initialized)
    {
        return HAL_ERROR;
    }

    /* Set direction pins to coast mode */
    Motor_SetDirectionPins(GPIO_PIN_RESET, GPIO_PIN_RESET);

    /* Set PWM to 0 */
    Motor_SetPWMDuty(hmotor, 0.0f);

    /* Update state */
    hmotor->state = MOTOR_STATE_COAST;
    hmotor->soft_start_active = false;

    return HAL_OK;
}

/**
 * @brief  Emergency stop - immediately disable all outputs
 *
 * This function:
 * 1. Sets PWM to 0% immediately
 * 2. Applies brake (IN1=1, IN2=1)
 * 3. Sets emergency_stop flag (prevents further speed commands)
 * 4. Sets motor state to FAULT
 *
 * After calling this, Motor_ClearFault() must be called to resume.
 */
void Motor_EmergencyStop(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return;
    }

    /* Immediately set PWM compare to 0 (register-level for speed) */
    __HAL_TIM_SET_COMPARE(hmotor->htim_pwm, hmotor->pwm_channel, 0);

    /* Apply brake */
    Motor_SetDirectionPins(GPIO_PIN_SET, GPIO_PIN_SET);

    /* Update state */
    hmotor->duty_cycle = 0.0f;
    hmotor->target_duty = 0.0f;
    hmotor->state = MOTOR_STATE_FAULT;
    hmotor->emergency_stop = true;
    hmotor->soft_start_active = false;
}

/**
 * @brief  Clear fault condition and return to stopped state
 */
HAL_StatusTypeDef Motor_ClearFault(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL || !hmotor->initialized)
    {
        return HAL_ERROR;
    }

    /* Clear emergency stop flag */
    hmotor->emergency_stop = false;

    /* Release brake (coast mode) */
    Motor_SetDirectionPins(GPIO_PIN_RESET, GPIO_PIN_RESET);

    /* Ensure PWM is at 0 */
    Motor_SetPWMDuty(hmotor, 0.0f);

    /* Set state to stopped */
    hmotor->state = MOTOR_STATE_STOPPED;

    return HAL_OK;
}

/**
 * @brief  Get current motor state
 */
Motor_State_t Motor_GetState(const Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return MOTOR_STATE_FAULT;
    }
    return hmotor->state;
}

/**
 * @brief  Get current duty cycle
 */
float Motor_GetDutyCycle(const Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return 0.0f;
    }
    return hmotor->duty_cycle;
}

/**
 * @brief  Get current direction
 */
Motor_Direction_t Motor_GetDirection(const Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return MOTOR_DIR_FORWARD;
    }
    return hmotor->direction;
}

/**
 * @brief  Update soft-start ramp
 *
 * This function should be called periodically (e.g., every 1ms from SysTick)
 * when soft-start is active. It gradually ramps the duty cycle from the
 * current value to the target value.
 *
 * Ramp rate: MOTOR_SOFT_START_STEP percent per call (default: 2%/ms = 50ms to 100%)
 */
void Motor_SoftStartUpdate(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL || !hmotor->soft_start_active)
    {
        return;
    }

    if (hmotor->emergency_stop)
    {
        hmotor->soft_start_active = false;
        return;
    }

    /* Calculate difference to target */
    float diff = hmotor->target_duty - hmotor->duty_cycle;

    if (fabsf(diff) <= MOTOR_SOFT_START_STEP)
    {
        /* Close enough to target: set final value and stop ramping */
        Motor_SetPWMDuty(hmotor, hmotor->target_duty);
        hmotor->soft_start_active = false;
    }
    else if (diff > 0.0f)
    {
        /* Ramp up */
        Motor_SetPWMDuty(hmotor, hmotor->duty_cycle + MOTOR_SOFT_START_STEP);
    }
    else
    {
        /* Ramp down */
        Motor_SetPWMDuty(hmotor, hmotor->duty_cycle - MOTOR_SOFT_START_STEP);
    }
}

/**
 * @brief  Set maximum allowed duty cycle
 */
void Motor_SetMaxDuty(Motor_Handle_t *hmotor, float max_duty)
{
    if (hmotor == NULL)
    {
        return;
    }

    /* Clamp to valid range */
    if (max_duty < 0.0f)
        max_duty = 0.0f;
    if (max_duty > MOTOR_MAX_DUTY_CYCLE)
        max_duty = MOTOR_MAX_DUTY_CYCLE;

    hmotor->max_duty = max_duty;

    /* If current duty exceeds new max, reduce it */
    if (hmotor->duty_cycle > max_duty)
    {
        Motor_SetPWMDuty(hmotor, max_duty);
    }
}

/**
 * @brief  Check if motor driver is initialized
 */
bool Motor_IsInitialized(const Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return false;
    }
    return hmotor->initialized;
}
