/**
 * @file    pid_controller.c
 * @brief   PID Controller Implementation for DC Motor Speed Control
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details Complete PID controller implementation featuring:
 *          - Standard parallel-form PID algorithm
 *          - Derivative on measurement (prevents derivative kick on setpoint change)
 *          - First-order low-pass filter on derivative term (noise rejection)
 *          - Integral anti-windup using clamping method
 *          - Output saturation with configurable limits
 *          - Bumpless gain changes via proper state management
 *
 * @note    This implementation uses single-precision float. The STM32F407VG
 *          has a hardware FPU (Cortex-M4F) that makes float operations efficient.
 *          Typical PID_Compute() execution time: < 2 microseconds at 168MHz.
 */

/* -------------------------------------------------------------------------- */
/*                              Includes                                      */
/* -------------------------------------------------------------------------- */

#include "pid_controller.h"
#include <string.h> /* For memset */
#include <math.h>   /* For fabsf  */

/* -------------------------------------------------------------------------- */
/*                          Private Helper Functions                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Clamp a floating-point value between min and max
 *
 * @param  value  Input value
 * @param  min    Minimum bound
 * @param  max    Maximum bound
 * @return float  Clamped value
 */
static inline float clamp_f(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

/**
 * @brief  Check if a float value is finite (not NaN or Inf)
 *
 * @param  value  Value to check
 * @return bool   true if value is finite
 */
static inline bool is_finite_f(float value)
{
    /* IEEE 754: NaN != NaN, and isinf checks for infinity */
    return (value == value) && (fabsf(value) < 1.0e30f);
}

/* -------------------------------------------------------------------------- */
/*                          Public Function Implementations                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize PID controller with default parameters
 */
void PID_Init(PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    /* Clear entire structure to zero first */
    memset(pid, 0, sizeof(PID_Controller_t));

    /* Set default gains */
    pid->Kp = PID_DEFAULT_KP;
    pid->Ki = PID_DEFAULT_KI;
    pid->Kd = PID_DEFAULT_KD;

    /* Set default setpoint */
    pid->setpoint = 0.0f;

    /* Set default output limits */
    pid->output_min = PID_DEFAULT_OUTPUT_MIN;
    pid->output_max = PID_DEFAULT_OUTPUT_MAX;

    /* Set default anti-windup configuration */
    pid->antiwindup_mode = PID_ANTIWINDUP_CLAMPING;
    pid->integral_min = PID_DEFAULT_INTEGRAL_MIN;
    pid->integral_max = PID_DEFAULT_INTEGRAL_MAX;
    pid->Ka = 1.0f; /* Default back-calculation gain */

    /* Set default derivative filter */
    pid->deriv_filter_alpha = PID_DEFAULT_DERIV_FILTER_ALPHA;
    pid->deriv_filter_enabled = true;

    /* Set default sampling period */
    pid->dt = PID_DEFAULT_DT;

    /* Initialize internal state */
    pid->integral_sum = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->is_first_run = true;
    pid->output_saturated = false;
    pid->state = PID_STATE_IDLE;
}

/**
 * @brief  Initialize PID controller with custom gains
 */
void PID_InitWithGains(PID_Controller_t *pid, float kp, float ki, float kd, float dt)
{
    /* Initialize with defaults first, then override gains */
    PID_Init(pid);

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
}

/**
 * @brief  Compute PID output from current measurement
 *
 * Algorithm summary (per sample):
 *
 *   error = setpoint - measurement
 *
 *   P_term = Kp * error
 *
 *   I_term = Ki * integral_sum   (integral_sum += error * dt, with anti-windup)
 *
 *   D_term = -Kd * d(measurement)/dt   (derivative on measurement, filtered)
 *
 *   output = clamp(P_term + I_term + D_term, output_min, output_max)
 */
float PID_Compute(PID_Controller_t *pid, float measurement)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    /* Safety check: ensure dt is valid */
    if (pid->dt <= 0.0f || !is_finite_f(pid->dt))
    {
        pid->state = PID_STATE_ERROR;
        return pid->output;
    }

    /* Safety check: ensure measurement is valid */
    if (!is_finite_f(measurement))
    {
        pid->state = PID_STATE_ERROR;
        return pid->output;
    }

    /* Mark controller as running */
    pid->state = PID_STATE_RUNNING;

    /* ---------------------------------------------------------------------- */
    /* Step 1: Calculate Error                                                 */
    /* ---------------------------------------------------------------------- */
    float error = pid->setpoint - measurement;

    /* ---------------------------------------------------------------------- */
    /* Step 2: Proportional Term                                               */
    /*   P_out = Kp * error                                                   */
    /* ---------------------------------------------------------------------- */
    pid->p_term = pid->Kp * error;

    /* ---------------------------------------------------------------------- */
    /* Step 3: Integral Term with Anti-Windup                                  */
    /*   I_out = Ki * sum(error * dt)                                         */
    /*                                                                        */
    /*   Anti-windup (clamping): do not accumulate integral if output is       */
    /*   saturated AND the error would push it further into saturation.        */
    /* ---------------------------------------------------------------------- */
    bool should_integrate = true;

    if (pid->antiwindup_mode == PID_ANTIWINDUP_CLAMPING)
    {
        /*
         * Clamping anti-windup logic:
         * If output was saturated on the previous cycle, only allow
         * integration if the error sign would reduce the integral
         * (i.e., move away from saturation).
         */
        if (pid->output_saturated)
        {
            /* Check if error would push integral further into saturation */
            if ((error > 0.0f && pid->output >= pid->output_max) ||
                (error < 0.0f && pid->output <= pid->output_min))
            {
                should_integrate = false;
            }
        }
    }

    if (should_integrate)
    {
        pid->integral_sum += error * pid->dt;

        /* Apply integral limits (hard clamp as an additional safety net) */
        pid->integral_sum = clamp_f(pid->integral_sum,
                                    pid->integral_min,
                                    pid->integral_max);
    }

    pid->i_term = pid->Ki * pid->integral_sum;

    /* ---------------------------------------------------------------------- */
    /* Step 4: Derivative Term                                                 */
    /*                                                                        */
    /*   Derivative on MEASUREMENT (not on error) to prevent derivative kick  */
    /*   when setpoint changes abruptly.                                      */
    /*                                                                        */
    /*   D_out = -Kd * d(measurement) / dt                                    */
    /*                                                                        */
    /*   The negative sign is because:                                        */
    /*     d(error)/dt = d(setpoint)/dt - d(measurement)/dt                   */
    /*   If setpoint is constant: d(error)/dt = -d(measurement)/dt            */
    /*                                                                        */
    /*   Low-pass filter applied to reduce noise amplification:               */
    /*     filtered = alpha * raw + (1-alpha) * prev_filtered                 */
    /* ---------------------------------------------------------------------- */
    float raw_derivative = 0.0f;

    if (pid->is_first_run)
    {
        /*
         * On the first computation, we have no previous measurement to
         * differentiate against. Set derivative to zero and store the
         * current measurement for the next cycle.
         */
        raw_derivative = 0.0f;
        pid->is_first_run = false;
    }
    else
    {
        /* Compute rate of change of measurement */
        raw_derivative = (measurement - pid->prev_measurement) / pid->dt;
    }

    /* Apply low-pass filter to derivative if enabled */
    float filtered_derivative;
    if (pid->deriv_filter_enabled)
    {
        filtered_derivative = pid->deriv_filter_alpha * raw_derivative +
                              (1.0f - pid->deriv_filter_alpha) * pid->prev_derivative;
    }
    else
    {
        filtered_derivative = raw_derivative;
    }

    /* Store filtered derivative for next cycle */
    pid->prev_derivative = filtered_derivative;

    /* Derivative term: negative because we use derivative on measurement */
    pid->d_term = -pid->Kd * filtered_derivative;

    /* ---------------------------------------------------------------------- */
    /* Step 5: Compute Total Output                                            */
    /* ---------------------------------------------------------------------- */
    float raw_output = pid->p_term + pid->i_term + pid->d_term;

    /* ---------------------------------------------------------------------- */
    /* Step 6: Back-Calculation Anti-Windup (if enabled)                       */
    /*                                                                        */
    /*   Adjust integral based on the difference between saturated and        */
    /*   unsaturated output.                                                  */
    /* ---------------------------------------------------------------------- */
    if (pid->antiwindup_mode == PID_ANTIWINDUP_BACK_CALC)
    {
        float saturated_output = clamp_f(raw_output, pid->output_min, pid->output_max);
        float saturation_error = saturated_output - raw_output;

        /* Adjust integral accumulator */
        pid->integral_sum += pid->Ka * saturation_error * pid->dt;
        pid->integral_sum = clamp_f(pid->integral_sum,
                                    pid->integral_min,
                                    pid->integral_max);

        /* Recompute I term after adjustment */
        pid->i_term = pid->Ki * pid->integral_sum;

        /* Recompute raw output */
        raw_output = pid->p_term + pid->i_term + pid->d_term;
    }

    /* ---------------------------------------------------------------------- */
    /* Step 7: Output Saturation (Clamping)                                    */
    /* ---------------------------------------------------------------------- */
    pid->output = clamp_f(raw_output, pid->output_min, pid->output_max);

    /* Track whether output is saturated (used by clamping anti-windup) */
    pid->output_saturated = (fabsf(pid->output - raw_output) > 0.001f);

    /* ---------------------------------------------------------------------- */
    /* Step 8: Store State for Next Cycle                                      */
    /* ---------------------------------------------------------------------- */
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return pid->output;
}

/**
 * @brief  Reset PID controller internal state
 */
void PID_Reset(PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    /* Reset all internal state variables */
    pid->integral_sum = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output = 0.0f;
    pid->p_term = 0.0f;
    pid->i_term = 0.0f;
    pid->d_term = 0.0f;
    pid->is_first_run = true;
    pid->output_saturated = false;
    pid->state = PID_STATE_IDLE;
}

/**
 * @brief  Set PID gains at runtime
 *
 * When changing gains while the controller is running, the integral
 * accumulator is preserved. This provides smoother transitions (bumpless
 * transfer) compared to resetting the entire controller.
 */
void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    /*
     * Bumpless gain change for integral term:
     * When Ki changes, adjust integral_sum so that the I contribution
     * (Ki * integral_sum) remains the same:
     *   old_Ki * old_integral = new_Ki * new_integral
     *   new_integral = old_integral * (old_Ki / new_Ki)
     *
     * Only do this if both old and new Ki are nonzero.
     */
    if (fabsf(ki) > 1.0e-10f && fabsf(pid->Ki) > 1.0e-10f)
    {
        pid->integral_sum = pid->integral_sum * (pid->Ki / ki);
        pid->integral_sum = clamp_f(pid->integral_sum,
                                    pid->integral_min,
                                    pid->integral_max);
    }
    else if (fabsf(ki) < 1.0e-10f)
    {
        /* New Ki is zero; clear integral accumulator */
        pid->integral_sum = 0.0f;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

/**
 * @brief  Set output limits
 */
void PID_SetOutputLimits(PID_Controller_t *pid, float min, float max)
{
    if (pid == NULL)
    {
        return;
    }

    /* Ensure min < max */
    if (min >= max)
    {
        return;
    }

    pid->output_min = min;
    pid->output_max = max;

    /* Clamp current output to new limits */
    pid->output = clamp_f(pid->output, min, max);
}

/**
 * @brief  Set integral limits for anti-windup
 */
void PID_SetIntegralLimits(PID_Controller_t *pid, float min, float max)
{
    if (pid == NULL)
    {
        return;
    }

    /* Ensure min < max */
    if (min >= max)
    {
        return;
    }

    pid->integral_min = min;
    pid->integral_max = max;

    /* Clamp current integral to new limits */
    pid->integral_sum = clamp_f(pid->integral_sum, min, max);
}

/**
 * @brief  Set the target setpoint
 */
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint)
{
    if (pid == NULL)
    {
        return;
    }

    pid->setpoint = setpoint;
}

/**
 * @brief  Enable and configure anti-windup
 */
void PID_EnableAntiWindup(PID_Controller_t *pid, PID_AntiWindupMode_t mode)
{
    if (pid == NULL)
    {
        return;
    }

    pid->antiwindup_mode = mode;
}

/**
 * @brief  Set derivative filter coefficient
 */
void PID_SetDerivativeFilter(PID_Controller_t *pid, float alpha, bool enable)
{
    if (pid == NULL)
    {
        return;
    }

    /* Clamp alpha to valid range */
    pid->deriv_filter_alpha = clamp_f(alpha, 0.0f, 1.0f);
    pid->deriv_filter_enabled = enable;
}

/**
 * @brief  Set sampling period
 */
void PID_SetSamplingPeriod(PID_Controller_t *pid, float dt)
{
    if (pid == NULL || dt <= 0.0f)
    {
        return;
    }

    pid->dt = dt;
}

/**
 * @brief  Get the last computed output value
 */
float PID_GetOutput(const PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    return pid->output;
}

/**
 * @brief  Get the current error
 */
float PID_GetError(const PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    return pid->prev_error;
}

/**
 * @brief  Get individual PID term contributions
 */
void PID_GetTerms(const PID_Controller_t *pid, float *p_term, float *i_term, float *d_term)
{
    if (pid == NULL)
    {
        return;
    }

    if (p_term != NULL)
        *p_term = pid->p_term;
    if (i_term != NULL)
        *i_term = pid->i_term;
    if (d_term != NULL)
        *d_term = pid->d_term;
}

/**
 * @brief  Check if PID output is saturated
 */
bool PID_IsSaturated(const PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return false;
    }
    return pid->output_saturated;
}

/**
 * @brief  Get current controller state
 */
PID_State_t PID_GetState(const PID_Controller_t *pid)
{
    if (pid == NULL)
    {
        return PID_STATE_ERROR;
    }
    return pid->state;
}
