/**
 * @file    encoder.c
 * @brief   Quadrature Encoder Implementation using TIM3 Encoder Mode
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details Implements quadrature encoder reading using the STM32 timer
 *          hardware encoder interface:
 *
 *          Hardware configuration:
 *          - TIM3 in Encoder Mode 3 (both edges of TI1 and TI2 = 4x resolution)
 *          - PA6 (TIM3_CH1): Encoder Phase A input
 *          - PA7 (TIM3_CH2): Encoder Phase B input
 *
 *          Position tracking:
 *          - TIM3 is a 16-bit timer (CNT range: 0..65535)
 *          - For a 400 CPR encoder in 4x mode: 1600 counts/rev
 *          - Timer overflow is handled via TIM3 update interrupt
 *          - 32-bit extended position = overflow_count * 65536 + CNT
 *
 *          Velocity calculation:
 *          - Uses pulse counting method: RPM = (delta_pos / CPR_4x) * (60/dt)
 *          - First-order low-pass filter for noise reduction
 *          - Called at 100Hz from the PID control loop timer
 *
 *          Direction detection:
 *          - TIM3->CR1.DIR bit indicates current counting direction
 *          - 0 = upcounting (forward), 1 = downcounting (reverse)
 */

/* -------------------------------------------------------------------------- */
/*                              Includes                                      */
/* -------------------------------------------------------------------------- */

#include "encoder.h"
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                          Public Function Implementations                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize the encoder interface
 *
 * This function assumes TIM3 has been configured for encoder mode
 * via CubeMX or HAL initialization code. It starts the encoder
 * counter and enables the update interrupt for overflow detection.
 */
HAL_StatusTypeDef Encoder_Init(Encoder_Handle_t *henc,
                               TIM_HandleTypeDef *htim,
                               uint32_t cpr)
{
    if (henc == NULL || htim == NULL || cpr == 0)
    {
        return HAL_ERROR;
    }

    /* Store hardware handle and parameters */
    henc->htim = htim;
    henc->cpr = cpr;
    henc->counts_per_rev = cpr * ENCODER_QUADRATURE_MULT;

    /* Initialize position tracking */
    henc->position = 0;
    henc->overflow_count = 0;
    henc->last_counter = 0;

    /* Initialize velocity calculation */
    henc->velocity_rpm = 0.0f;
    henc->velocity_raw = 0.0f;
    henc->last_position = 0;
    henc->velocity_filter_alpha = ENCODER_VELOCITY_FILTER_ALPHA;

    /* Initialize direction */
    henc->direction = ENCODER_DIR_FORWARD;

    /* Initialize state */
    henc->initialized = false;

    /* -------------------------------------------------------------------- */
    /* Configure TIM3 for Encoder Mode                                      */
    /*                                                                      */
    /* TIM3 encoder mode configuration (via CubeMX/HAL):                    */
    /*   SMCR.SMS  = 011  (Encoder mode 3: count on both TI1 and TI2)      */
    /*   CCMR1.CC1S = 01  (IC1 mapped to TI1)                              */
    /*   CCMR1.CC2S = 01  (IC2 mapped to TI2)                              */
    /*   CCMR1.IC1F = ENCODER_INPUT_FILTER (noise filtering)               */
    /*   CCMR1.IC2F = ENCODER_INPUT_FILTER (noise filtering)               */
    /*   CCER.CC1P  = 0   (TI1 non-inverted: rising edge)                  */
    /*   CCER.CC2P  = 0   (TI2 non-inverted: rising edge)                  */
    /*   ARR        = 65535 (full 16-bit range)                             */
    /* -------------------------------------------------------------------- */

    /* Set auto-reload to maximum 16-bit value */
    __HAL_TIM_SET_AUTORELOAD(henc->htim, ENCODER_TIMER_PERIOD);

    /* Reset counter to 0 */
    __HAL_TIM_SET_COUNTER(henc->htim, 0);

    /* Start encoder mode (both channels) */
    HAL_StatusTypeDef status = HAL_TIM_Encoder_Start(henc->htim,
                                                     TIM_CHANNEL_ALL);
    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Enable update interrupt for overflow/underflow detection.
     * The TIM3 update interrupt fires when the counter wraps around
     * (0->65535 on underflow, or 65535->0 on overflow).
     */
    __HAL_TIM_ENABLE_IT(henc->htim, TIM_IT_UPDATE);

    /* Store initial counter value */
    henc->last_counter = (uint16_t)__HAL_TIM_GET_COUNTER(henc->htim);

    /* Mark as initialized */
    henc->initialized = true;

    return HAL_OK;
}

/**
 * @brief  Get absolute position in encoder counts
 *
 * Combines the overflow count (multiples of 65536) with the current
 * timer counter value to produce a 32-bit signed position.
 *
 * Position range: approximately +/- 2 billion counts
 * For a 1600 counts/rev encoder: +/- 1.3 million revolutions
 */
int32_t Encoder_GetPosition(Encoder_Handle_t *henc)
{
    if (henc == NULL || !henc->initialized)
    {
        return 0;
    }

    /* Read current counter value (atomic 16-bit read) */
    uint16_t current_counter = (uint16_t)__HAL_TIM_GET_COUNTER(henc->htim);

    /*
     * Calculate extended 32-bit position:
     * position = (overflow_count * 65536) + current_counter
     *
     * Note: overflow_count is updated in the TIM3 update interrupt.
     * The counter value between interrupts is always valid (0..65535).
     */
    henc->position = (henc->overflow_count * (int32_t)(ENCODER_TIMER_PERIOD + 1)) + (int32_t)current_counter;

    return henc->position;
}

/**
 * @brief  Get position in degrees within one revolution
 */
float Encoder_GetPositionDegrees(Encoder_Handle_t *henc)
{
    if (henc == NULL || !henc->initialized || henc->counts_per_rev == 0)
    {
        return 0.0f;
    }

    int32_t pos = Encoder_GetPosition(henc);

    /* Get position within one revolution */
    int32_t pos_in_rev = pos % (int32_t)henc->counts_per_rev;

    /* Handle negative modulo */
    if (pos_in_rev < 0)
    {
        pos_in_rev += (int32_t)henc->counts_per_rev;
    }

    /* Convert to degrees */
    return ((float)pos_in_rev / (float)henc->counts_per_rev) * 360.0f;
}

/**
 * @brief  Get velocity in RPM using pulse counting method
 *
 * Algorithm:
 *   1. Read current position
 *   2. delta_position = current_position - last_position
 *   3. RPM_raw = (delta_position / counts_per_rev) * (60.0 / dt)
 *   4. Apply low-pass filter: RPM = alpha * RPM_raw + (1-alpha) * RPM_prev
 *   5. Store current position for next call
 *
 * @param  dt  Time since last call in seconds (e.g., 0.01 for 100Hz)
 */
float Encoder_GetVelocityRPM(Encoder_Handle_t *henc, float dt)
{
    if (henc == NULL || !henc->initialized || dt <= 0.0f)
    {
        return 0.0f;
    }

    /* Get current extended position */
    int32_t current_position = Encoder_GetPosition(henc);

    /* Calculate position change since last call */
    int32_t delta_position = current_position - henc->last_position;

    /* Convert to RPM:
     *   revolutions = delta_position / counts_per_rev
     *   rev_per_second = revolutions / dt
     *   RPM = rev_per_second * 60
     */
    float raw_rpm = ((float)delta_position / (float)henc->counts_per_rev) * (60.0f / dt);

    /* Store raw velocity */
    henc->velocity_raw = raw_rpm;

    /* Apply first-order low-pass filter to reduce noise:
     *   filtered = alpha * raw + (1 - alpha) * previous_filtered
     *
     * alpha close to 1.0 = less filtering (more responsive)
     * alpha close to 0.0 = more filtering (smoother but laggier)
     */
    henc->velocity_rpm = henc->velocity_filter_alpha * raw_rpm + (1.0f - henc->velocity_filter_alpha) * henc->velocity_rpm;

    /* Update direction based on velocity sign */
    if (henc->velocity_rpm >= 0.0f)
    {
        henc->direction = ENCODER_DIR_FORWARD;
    }
    else
    {
        henc->direction = ENCODER_DIR_REVERSE;
    }

    /* Store position for next call */
    henc->last_position = current_position;

    return henc->velocity_rpm;
}

/**
 * @brief  Get raw (unfiltered) velocity in RPM
 */
float Encoder_GetRawVelocityRPM(const Encoder_Handle_t *henc)
{
    if (henc == NULL)
    {
        return 0.0f;
    }
    return henc->velocity_raw;
}

/**
 * @brief  Get current rotation direction
 *
 * Direction can be determined two ways:
 * 1. From the velocity sign (used here for consistency)
 * 2. From TIM3->CR1.DIR bit (hardware direction)
 */
Encoder_Direction_t Encoder_GetDirection(const Encoder_Handle_t *henc)
{
    if (henc == NULL)
    {
        return ENCODER_DIR_FORWARD;
    }
    return henc->direction;
}

/**
 * @brief  Reset encoder position to zero
 */
void Encoder_ResetPosition(Encoder_Handle_t *henc)
{
    if (henc == NULL || !henc->initialized)
    {
        return;
    }

    /*
     * Disable interrupts briefly to ensure atomic reset of all
     * position-related variables. This prevents the overflow handler
     * from modifying overflow_count while we're resetting.
     */
    __disable_irq();

    /* Reset timer counter to 0 */
    __HAL_TIM_SET_COUNTER(henc->htim, 0);

    /* Reset all position tracking state */
    henc->position = 0;
    henc->overflow_count = 0;
    henc->last_counter = 0;
    henc->last_position = 0;

    /* Reset velocity */
    henc->velocity_rpm = 0.0f;
    henc->velocity_raw = 0.0f;

    __enable_irq();
}

/**
 * @brief  Handle timer overflow/underflow interrupt
 *
 * This function MUST be called from the HAL_TIM_PeriodElapsedCallback()
 * when htim matches the encoder timer (TIM3).
 *
 * The TIM3 update interrupt fires when:
 * - Counter overflows: 65535 -> 0 (forward direction)
 * - Counter underflows: 0 -> 65535 (reverse direction)
 *
 * We determine the direction by reading TIM3->CR1.DIR:
 * - DIR = 0: Upcounting (forward) -> overflow occurred -> increment
 * - DIR = 1: Downcounting (reverse) -> underflow occurred -> decrement
 *
 * Important: In encoder mode, the DIR bit reflects the current counting
 * direction determined by the encoder signals, not a software setting.
 */
void Encoder_OverflowHandler(Encoder_Handle_t *henc)
{
    if (henc == NULL || !henc->initialized)
    {
        return;
    }

    /*
     * Determine overflow direction from the timer's direction bit.
     *
     * In encoder mode:
     * - If counting UP (DIR=0), an update event means we overflowed
     *   from 65535 to 0. This means forward rotation, so increment
     *   the overflow counter.
     * - If counting DOWN (DIR=1), an update event means we underflowed
     *   from 0 to 65535. This means reverse rotation, so decrement
     *   the overflow counter.
     */
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(henc->htim))
    {
        /* Underflow: counting down, passed through 0 -> 65535 */
        henc->overflow_count--;
    }
    else
    {
        /* Overflow: counting up, passed through 65535 -> 0 */
        henc->overflow_count++;
    }
}

/**
 * @brief  Set velocity filter coefficient
 */
void Encoder_SetVelocityFilter(Encoder_Handle_t *henc, float alpha)
{
    if (henc == NULL)
    {
        return;
    }

    /* Clamp to valid range */
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;

    henc->velocity_filter_alpha = alpha;
}

/**
 * @brief  Set counts per revolution
 */
void Encoder_SetCPR(Encoder_Handle_t *henc, uint32_t cpr)
{
    if (henc == NULL || cpr == 0)
    {
        return;
    }

    henc->cpr = cpr;
    henc->counts_per_rev = cpr * ENCODER_QUADRATURE_MULT;
}

/**
 * @brief  Get the raw timer counter value
 */
uint16_t Encoder_GetRawCounter(const Encoder_Handle_t *henc)
{
    if (henc == NULL || henc->htim == NULL)
    {
        return 0;
    }
    return (uint16_t)__HAL_TIM_GET_COUNTER(henc->htim);
}

/**
 * @brief  Check if encoder is initialized
 */
bool Encoder_IsInitialized(const Encoder_Handle_t *henc)
{
    if (henc == NULL)
    {
        return false;
    }
    return henc->initialized;
}
