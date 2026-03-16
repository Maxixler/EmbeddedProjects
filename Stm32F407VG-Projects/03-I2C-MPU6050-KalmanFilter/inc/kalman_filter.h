/**
 * @file    kalman_filter.h
 * @brief   2-State Kalman Filter and Complementary Filter for IMU Sensor Fusion
 * @author  STM32 Embedded Systems Portfolio
 * @version 2.0
 *
 * @details Implements a 2-state (angle + gyroscope bias) Kalman filter designed
 *          specifically for fusing accelerometer and gyroscope data to produce
 *          accurate, drift-free angle estimates.
 *
 *          The filter state vector is:
 *            x = [angle]   (degrees)
 *                [bias ]   (degrees/second, gyroscope bias estimate)
 *
 *          The system model:
 *            angle(k) = angle(k-1) + (gyro_rate - bias(k-1)) * dt
 *            bias(k)  = bias(k-1)
 *
 *          The measurement:
 *            z(k) = accelerometer_angle
 *
 *          Also provides a simple complementary filter for comparison.
 *
 *          Typical usage:
 *            1. Initialize with Kalman_Init()
 *            2. Each sample period, call Kalman_Update() with gyro rate and
 *               accelerometer angle
 *            3. Read the filtered angle from the state structure
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========================================================================== */
    /*                              INCLUDES                                      */
    /* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                         DEFAULT PARAMETERS                                 */
/* ========================================================================== */

/**
 * @defgroup Kalman_Defaults Default Kalman Filter Parameters
 * @brief Tuning parameters that control filter behavior
 *
 * Q_angle:  Process noise variance for the angle state.
 *           Higher values make the filter respond faster to changes
 *           but increase output noise. Lower values produce smoother
 *           output but slower response.
 *           Typical range: 0.0001 to 0.01
 *
 * Q_bias:   Process noise variance for the gyroscope bias state.
 *           Higher values allow the bias estimate to change faster.
 *           Lower values assume the bias is more stable over time.
 *           Typical range: 0.001 to 0.01
 *
 * R_measure: Measurement noise variance (accelerometer noise).
 *            Higher values place more trust in the gyroscope (model),
 *            producing smoother output but slower convergence to the
 *            true angle. Lower values trust the accelerometer more,
 *            giving faster convergence but more vibration sensitivity.
 *            Typical range: 0.01 to 0.5
 *
 * @{
 */
#define KALMAN_DEFAULT_Q_ANGLE 0.001f  /**< Angle process noise     */
#define KALMAN_DEFAULT_Q_BIAS 0.003f   /**< Bias process noise      */
#define KALMAN_DEFAULT_R_MEASURE 0.03f /**< Measurement noise       */
/** @} */

/**
 * @brief Default complementary filter coefficient
 *
 * alpha = 0.96 means 96% weight on gyroscope integration (high-pass)
 * and 4% weight on accelerometer angle (low-pass).
 *
 * Higher alpha -> smoother output, slower response to accelerometer
 * Lower alpha  -> noisier output, faster response to accelerometer
 *
 * Typical range: 0.90 to 0.99
 */
#define COMPLEMENTARY_DEFAULT_ALPHA 0.96f

    /* ========================================================================== */
    /*                            DATA STRUCTURES                                 */
    /* ========================================================================== */

    /**
     * @brief Kalman filter state structure
     *
     * Contains the complete state of one Kalman filter instance.
     * Each axis (roll, pitch) requires its own instance.
     *
     * Internal state:
     *   angle  - Current estimated angle (degrees)
     *   bias   - Current estimated gyroscope bias (degrees/second)
     *   P[2][2]- Error covariance matrix (2x2)
     *
     * Tuning parameters:
     *   Q_angle   - Process noise for angle
     *   Q_bias    - Process noise for bias
     *   R_measure - Measurement noise (accelerometer)
     */
    typedef struct
    {
        /* --- Estimated states --- */
        float angle; /**< Estimated angle (degrees)               */
        float bias;  /**< Estimated gyroscope bias (deg/s)        */

        /* --- Error covariance matrix P (2x2) --- */
        float P[2][2]; /**< Error covariance matrix                 */

        /* --- Tuning parameters --- */
        float Q_angle;   /**< Process noise variance for angle        */
        float Q_bias;    /**< Process noise variance for bias         */
        float R_measure; /**< Measurement noise variance              */

        /* --- Diagnostic outputs --- */
        float K[2];       /**< Last computed Kalman gain vector         */
        float innovation; /**< Last measurement innovation (residual)  */
        float S;          /**< Last innovation covariance              */
    } Kalman_State_t;

    /**
     * @brief Complementary filter state structure
     *
     * Simple first-order complementary filter that combines high-pass filtered
     * gyroscope data with low-pass filtered accelerometer data.
     */
    typedef struct
    {
        float angle;      /**< Estimated angle (degrees)               */
        float alpha;      /**< Filter coefficient (0.0 to 1.0)         */
        bool initialized; /**< True after first update                 */
    } ComplementaryFilter_State_t;

    /* ========================================================================== */
    /*                     KALMAN FILTER FUNCTION PROTOTYPES                      */
    /* ========================================================================== */

    /**
     * @defgroup Kalman_Functions Kalman Filter Functions
     * @{
     */

    /**
     * @brief  Initialize a Kalman filter instance with default parameters
     *
     * Sets the initial state to zero angle and zero bias, and initializes
     * the error covariance matrix with large diagonal values (high initial
     * uncertainty).
     *
     * Default parameters: Q_angle=0.001, Q_bias=0.003, R_measure=0.03
     *
     * @param  kf  Pointer to Kalman filter state structure
     */
    void Kalman_Init(Kalman_State_t *kf);

    /**
     * @brief  Initialize a Kalman filter with custom parameters
     *
     * Allows specifying the three tuning parameters that control filter behavior.
     *
     * @param  kf         Pointer to Kalman filter state structure
     * @param  Q_angle    Process noise variance for angle state
     * @param  Q_bias     Process noise variance for bias state
     * @param  R_measure  Measurement noise variance
     */
    void Kalman_InitCustom(Kalman_State_t *kf,
                           float Q_angle,
                           float Q_bias,
                           float R_measure);

    /**
     * @brief  Set the initial angle of the Kalman filter
     *
     * Use this to seed the filter with the first accelerometer reading
     * before starting the regular predict/update cycle. This avoids a
     * large initial transient as the filter converges from zero.
     *
     * @param  kf             Pointer to Kalman filter state structure
     * @param  initial_angle  Initial angle in degrees (from accelerometer)
     */
    void Kalman_SetAngle(Kalman_State_t *kf, float initial_angle);

    /**
     * @brief  Run one complete Kalman filter cycle (predict + update)
     *
     * This function performs both the prediction step (using gyroscope data)
     * and the update step (using accelerometer angle) in a single call.
     *
     * Call this function once per sample period.
     *
     * @param  kf          Pointer to Kalman filter state structure
     * @param  gyro_rate   Gyroscope angular rate (degrees/second)
     * @param  accel_angle Angle computed from accelerometer data (degrees)
     * @param  dt          Time step in seconds (e.g., 0.005 for 200 Hz)
     * @retval Filtered angle in degrees
     */
    float Kalman_Update(Kalman_State_t *kf,
                        float gyro_rate,
                        float accel_angle,
                        float dt);

    /**
     * @brief  Get the current estimated angle
     * @param  kf  Pointer to Kalman filter state structure
     * @retval Current angle estimate in degrees
     */
    float Kalman_GetAngle(const Kalman_State_t *kf);

    /**
     * @brief  Get the current estimated gyroscope bias
     * @param  kf  Pointer to Kalman filter state structure
     * @retval Current bias estimate in degrees/second
     */
    float Kalman_GetBias(const Kalman_State_t *kf);

    /**
     * @brief  Get the unbiased gyroscope rate (rate - estimated bias)
     * @param  kf         Pointer to Kalman filter state structure
     * @param  gyro_rate  Raw gyroscope rate in degrees/second
     * @retval Bias-compensated rate in degrees/second
     */
    float Kalman_GetRate(const Kalman_State_t *kf, float gyro_rate);

    /**
     * @brief  Update the process noise variance for the angle state
     * @param  kf       Pointer to Kalman filter state structure
     * @param  Q_angle  New Q_angle value
     */
    void Kalman_SetQAngle(Kalman_State_t *kf, float Q_angle);

    /**
     * @brief  Update the process noise variance for the bias state
     * @param  kf      Pointer to Kalman filter state structure
     * @param  Q_bias  New Q_bias value
     */
    void Kalman_SetQBias(Kalman_State_t *kf, float Q_bias);

    /**
     * @brief  Update the measurement noise variance
     * @param  kf         Pointer to Kalman filter state structure
     * @param  R_measure  New R_measure value
     */
    void Kalman_SetRMeasure(Kalman_State_t *kf, float R_measure);

    /**
     * @brief  Reset the Kalman filter state to initial conditions
     *
     * Resets angle and bias to zero, reinitializes the error covariance
     * matrix, but preserves the current tuning parameters.
     *
     * @param  kf  Pointer to Kalman filter state structure
     */
    void Kalman_Reset(Kalman_State_t *kf);

    /** @} */ /* End of Kalman_Functions */

    /* ========================================================================== */
    /*                 COMPLEMENTARY FILTER FUNCTION PROTOTYPES                   */
    /* ========================================================================== */

    /**
     * @defgroup Comp_Functions Complementary Filter Functions
     * @{
     */

    /**
     * @brief  Initialize a complementary filter with default alpha
     *
     * Default alpha = 0.96 (96% gyro, 4% accelerometer)
     *
     * @param  cf  Pointer to complementary filter state structure
     */
    void CompFilter_Init(ComplementaryFilter_State_t *cf);

    /**
     * @brief  Initialize a complementary filter with custom alpha
     * @param  cf     Pointer to complementary filter state structure
     * @param  alpha  Filter coefficient (0.0 to 1.0)
     */
    void CompFilter_InitCustom(ComplementaryFilter_State_t *cf, float alpha);

    /**
     * @brief  Set the initial angle of the complementary filter
     * @param  cf             Pointer to complementary filter state structure
     * @param  initial_angle  Initial angle in degrees (from accelerometer)
     */
    void CompFilter_SetAngle(ComplementaryFilter_State_t *cf, float initial_angle);

    /**
     * @brief  Run one complementary filter update cycle
     *
     * Implements the formula:
     *   angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
     *
     * This is equivalent to:
     *   High-pass filter on gyroscope + Low-pass filter on accelerometer
     *
     * @param  cf          Pointer to complementary filter state
     * @param  gyro_rate   Gyroscope angular rate (degrees/second)
     * @param  accel_angle Angle from accelerometer (degrees)
     * @param  dt          Time step in seconds
     * @retval Filtered angle in degrees
     */
    float CompFilter_Update(ComplementaryFilter_State_t *cf,
                            float gyro_rate,
                            float accel_angle,
                            float dt);

    /**
     * @brief  Get the current estimated angle
     * @param  cf  Pointer to complementary filter state
     * @retval Current angle estimate in degrees
     */
    float CompFilter_GetAngle(const ComplementaryFilter_State_t *cf);

    /**
     * @brief  Reset the complementary filter to initial state
     * @param  cf  Pointer to complementary filter state
     */
    void CompFilter_Reset(ComplementaryFilter_State_t *cf);

    /** @} */ /* End of Comp_Functions */

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H */
