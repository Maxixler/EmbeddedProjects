/**
 * @file    pid_controller.h
 * @brief   PID Controller Interface for DC Motor Speed Control
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details This module implements a production-quality discrete PID controller
 *          with the following features:
 *          - Configurable P, I, D gains
 *          - Integral anti-windup (clamping method)
 *          - Derivative kick prevention (derivative on measurement)
 *          - First-order low-pass filter on derivative term
 *          - Output saturation with configurable limits
 *          - Proper delta-time handling
 *
 * @note    Designed for STM32F407VG running at 168MHz with hardware FPU.
 *          All computations use single-precision floating point.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* -------------------------------------------------------------------------- */
    /*                              Includes                                      */
    /* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                              Definitions                                   */
/* -------------------------------------------------------------------------- */

/** Default PID gains (suitable starting point for DC motor speed control) */
#define PID_DEFAULT_KP 1.0f
#define PID_DEFAULT_KI 0.1f
#define PID_DEFAULT_KD 0.05f

/** Default output limits (PWM duty cycle percentage: -100% to +100%) */
#define PID_DEFAULT_OUTPUT_MIN -100.0f
#define PID_DEFAULT_OUTPUT_MAX 100.0f

/** Default integral limits for anti-windup */
#define PID_DEFAULT_INTEGRAL_MIN -500.0f
#define PID_DEFAULT_INTEGRAL_MAX 500.0f

/** Default derivative filter coefficient
 *  alpha = dt / (tau + dt), where tau = 1/(2*pi*fc)
 *  For fc = 10Hz, dt = 0.01s: alpha ~ 0.386
 */
#define PID_DEFAULT_DERIV_FILTER_ALPHA 0.386f

/** Default sampling period in seconds (100Hz control loop) */
#define PID_DEFAULT_DT 0.01f

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Anti-windup mode enumeration
     */
    typedef enum
    {
        PID_ANTIWINDUP_NONE = 0,     /**< No anti-windup (integral can grow unbounded)   */
        PID_ANTIWINDUP_CLAMPING = 1, /**< Clamping: stop integrating when output saturates */
        PID_ANTIWINDUP_BACK_CALC = 2 /**< Back-calculation with tracking gain Ka          */
    } PID_AntiWindupMode_t;

    /**
     * @brief PID controller state/status enumeration
     */
    typedef enum
    {
        PID_STATE_IDLE = 0,    /**< Controller is idle, not computing output         */
        PID_STATE_RUNNING = 1, /**< Controller is actively computing output          */
        PID_STATE_ERROR = 2    /**< Controller encountered an error                  */
    } PID_State_t;

    /**
     * @brief PID Controller handle structure
     *
     * Contains all parameters, state variables, and configuration for one
     * instance of a PID controller. Multiple instances can be created for
     * controlling different loops.
     */
    typedef struct
    {
        /* ---- Tunable Gains ---- */
        float Kp; /**< Proportional gain                         */
        float Ki; /**< Integral gain                             */
        float Kd; /**< Derivative gain                           */

        /* ---- Setpoint ---- */
        float setpoint; /**< Desired target value (e.g., RPM)          */

        /* ---- Output Limits ---- */
        float output_min; /**< Minimum output value (e.g., -100.0)       */
        float output_max; /**< Maximum output value (e.g., +100.0)       */

        /* ---- Anti-Windup Configuration ---- */
        PID_AntiWindupMode_t antiwindup_mode; /**< Selected anti-windup strategy       */
        float integral_min;                   /**< Minimum integral accumulator limit         */
        float integral_max;                   /**< Maximum integral accumulator limit         */
        float Ka;                             /**< Back-calculation tracking gain (if used)   */

        /* ---- Derivative Filter ---- */
        float deriv_filter_alpha;  /**< Low-pass filter coefficient (0..1)        */
        bool deriv_filter_enabled; /**< Enable/disable derivative filtering       */

        /* ---- Timing ---- */
        float dt; /**< Sampling period in seconds                */

        /* ---- Internal State (do not modify directly) ---- */
        float integral_sum;     /**< Accumulated integral term                 */
        float prev_error;       /**< Previous error value (for D on error)     */
        float prev_measurement; /**< Previous measurement (for D on measurement) */
        float prev_derivative;  /**< Previous filtered derivative              */
        float output;           /**< Last computed output                      */
        float p_term;           /**< Last proportional contribution            */
        float i_term;           /**< Last integral contribution                */
        float d_term;           /**< Last derivative contribution              */
        bool is_first_run;      /**< Flag for first computation cycle          */
        bool output_saturated;  /**< Flag indicating output is at limit        */
        PID_State_t state;      /**< Current controller state                  */
    } PID_Controller_t;

    /* -------------------------------------------------------------------------- */
    /*                          Function Prototypes                               */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief  Initialize PID controller with default parameters
     *
     * Sets all gains, limits, and internal state to default values.
     * Must be called before using PID_Compute().
     *
     * @param  pid  Pointer to PID controller handle
     */
    void PID_Init(PID_Controller_t *pid);

    /**
     * @brief  Initialize PID controller with custom gains
     *
     * @param  pid  Pointer to PID controller handle
     * @param  kp   Proportional gain
     * @param  ki   Integral gain
     * @param  kd   Derivative gain
     * @param  dt   Sampling period in seconds
     */
    void PID_InitWithGains(PID_Controller_t *pid, float kp, float ki, float kd, float dt);

    /**
     * @brief  Compute PID output from current measurement
     *
     * This is the main PID computation function. It should be called at a
     * fixed rate defined by dt (e.g., in a timer interrupt at 100Hz).
     *
     * Features applied:
     * - Derivative on measurement (prevents derivative kick)
     * - Low-pass filter on derivative term (reduces noise amplification)
     * - Anti-windup on integral term (prevents integral saturation)
     * - Output clamping to configured limits
     *
     * @param  pid          Pointer to PID controller handle
     * @param  measurement  Current process variable (e.g., measured RPM)
     * @return float        Control output (e.g., PWM duty cycle)
     */
    float PID_Compute(PID_Controller_t *pid, float measurement);

    /**
     * @brief  Reset PID controller internal state
     *
     * Clears integral accumulator, previous error/measurement, and
     * derivative filter state. Does NOT reset gains or limits.
     * Use this when restarting the control loop or after an emergency stop.
     *
     * @param  pid  Pointer to PID controller handle
     */
    void PID_Reset(PID_Controller_t *pid);

    /**
     * @brief  Set PID gains
     *
     * Allows runtime adjustment of Kp, Ki, Kd via UART or other interface.
     *
     * @param  pid  Pointer to PID controller handle
     * @param  kp   Proportional gain
     * @param  ki   Integral gain
     * @param  kd   Derivative gain
     */
    void PID_SetGains(PID_Controller_t *pid, float kp, float ki, float kd);

    /**
     * @brief  Set output limits
     *
     * Configures the minimum and maximum values the PID output can take.
     * For motor control, typically -100.0 to +100.0 (percent duty cycle).
     *
     * @param  pid  Pointer to PID controller handle
     * @param  min  Minimum output value
     * @param  max  Maximum output value
     */
    void PID_SetOutputLimits(PID_Controller_t *pid, float min, float max);

    /**
     * @brief  Set integral limits for anti-windup
     *
     * Configures the maximum absolute value the integral accumulator can reach.
     *
     * @param  pid  Pointer to PID controller handle
     * @param  min  Minimum integral value
     * @param  max  Maximum integral value
     */
    void PID_SetIntegralLimits(PID_Controller_t *pid, float min, float max);

    /**
     * @brief  Set the target setpoint
     *
     * @param  pid       Pointer to PID controller handle
     * @param  setpoint  Desired target value (e.g., target RPM)
     */
    void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);

    /**
     * @brief  Enable and configure anti-windup
     *
     * @param  pid   Pointer to PID controller handle
     * @param  mode  Anti-windup mode (NONE, CLAMPING, or BACK_CALC)
     */
    void PID_EnableAntiWindup(PID_Controller_t *pid, PID_AntiWindupMode_t mode);

    /**
     * @brief  Set derivative filter coefficient
     *
     * @param  pid     Pointer to PID controller handle
     * @param  alpha   Filter coefficient (0..1). Lower = more filtering.
     * @param  enable  true to enable filtering, false to disable
     */
    void PID_SetDerivativeFilter(PID_Controller_t *pid, float alpha, bool enable);

    /**
     * @brief  Set sampling period
     *
     * @param  pid  Pointer to PID controller handle
     * @param  dt   Sampling period in seconds
     */
    void PID_SetSamplingPeriod(PID_Controller_t *pid, float dt);

    /**
     * @brief  Get the last computed output value
     *
     * @param  pid  Pointer to PID controller handle
     * @return float  Last computed PID output
     */
    float PID_GetOutput(const PID_Controller_t *pid);

    /**
     * @brief  Get the current error (setpoint - measurement)
     *
     * @param  pid  Pointer to PID controller handle
     * @return float  Last computed error
     */
    float PID_GetError(const PID_Controller_t *pid);

    /**
     * @brief  Get individual PID term contributions
     *
     * Useful for debugging and tuning. Returns the last computed
     * P, I, and D term values.
     *
     * @param  pid     Pointer to PID controller handle
     * @param  p_term  Pointer to store proportional contribution (can be NULL)
     * @param  i_term  Pointer to store integral contribution (can be NULL)
     * @param  d_term  Pointer to store derivative contribution (can be NULL)
     */
    void PID_GetTerms(const PID_Controller_t *pid, float *p_term, float *i_term, float *d_term);

    /**
     * @brief  Check if PID output is saturated
     *
     * @param  pid  Pointer to PID controller handle
     * @return bool true if output is at min or max limit
     */
    bool PID_IsSaturated(const PID_Controller_t *pid);

    /**
     * @brief  Get current controller state
     *
     * @param  pid  Pointer to PID controller handle
     * @return PID_State_t  Current state (IDLE, RUNNING, or ERROR)
     */
    PID_State_t PID_GetState(const PID_Controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
