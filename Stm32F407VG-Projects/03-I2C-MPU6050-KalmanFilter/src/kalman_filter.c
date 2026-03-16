/**
 * @file    kalman_filter.c
 * @brief   2-State Kalman Filter and Complementary Filter Implementation
 * @author  STM32 Embedded Systems Portfolio
 * @version 2.0
 *
 * @details Implements a discrete-time Kalman filter with a 2-element state
 *          vector [angle, bias] for fusing accelerometer and gyroscope data.
 *
 *          Mathematical foundation:
 *
 *          State Transition Model:
 *            x(k|k-1) = F * x(k-1|k-1) + B * u(k)
 *
 *            Where:
 *              F = [1  -dt]    State transition matrix
 *                  [0   1 ]
 *
 *              B = [dt]        Control input matrix
 *                  [0 ]
 *
 *              u = gyro_rate   Control input (gyroscope measurement)
 *
 *          Measurement Model:
 *            z(k) = H * x(k) + v(k)
 *
 *            Where:
 *              H = [1  0]      Measurement matrix
 *              v ~ N(0, R)     Measurement noise
 *
 *          The filter operates in two phases per sample:
 *            1. PREDICT: Project state and covariance forward using gyro data
 *            2. UPDATE:  Correct the prediction using accelerometer angle
 *
 *          Also includes a complementary filter implementation for comparison.
 */

/* ========================================================================== */
/*                              INCLUDES                                      */
/* ========================================================================== */

#include "kalman_filter.h"

/* ========================================================================== */
/*                       KALMAN FILTER IMPLEMENTATION                         */
/* ========================================================================== */

void Kalman_Init(Kalman_State_t *kf)
{
    Kalman_InitCustom(kf,
                      KALMAN_DEFAULT_Q_ANGLE,
                      KALMAN_DEFAULT_Q_BIAS,
                      KALMAN_DEFAULT_R_MEASURE);
}

void Kalman_InitCustom(Kalman_State_t *kf,
                       float Q_angle,
                       float Q_bias,
                       float R_measure)
{
    /* Initialize state estimates to zero */
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    /*
     * Initialize error covariance matrix P.
     *
     * We start with relatively large values on the diagonal to indicate
     * high initial uncertainty. The filter will converge quickly as
     * measurements arrive.
     *
     * P = [1  0]
     *     [0  1]
     *
     * Off-diagonal elements are zero (no initial correlation between
     * angle and bias uncertainty).
     */
    kf->P[0][0] = 1.0f; /* Angle variance      */
    kf->P[0][1] = 0.0f; /* Angle-bias covariance */
    kf->P[1][0] = 0.0f; /* Bias-angle covariance */
    kf->P[1][1] = 1.0f; /* Bias variance        */

    /* Store tuning parameters */
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    /* Clear diagnostic outputs */
    kf->K[0] = 0.0f;
    kf->K[1] = 0.0f;
    kf->innovation = 0.0f;
    kf->S = 0.0f;
}

void Kalman_SetAngle(Kalman_State_t *kf, float initial_angle)
{
    kf->angle = initial_angle;
}

float Kalman_Update(Kalman_State_t *kf,
                    float gyro_rate,
                    float accel_angle,
                    float dt)
{
    /*
     * ================================================================
     *  STEP 1: PREDICT (Time Update / A Priori Estimate)
     * ================================================================
     *
     * Project the state estimate forward in time using the system model
     * and the gyroscope measurement as the control input.
     */

    /*
     * 1a. State prediction:
     *
     *   angle_predicted = angle + (gyro_rate - bias) * dt
     *   bias_predicted  = bias   (bias is modeled as constant)
     *
     * We subtract the estimated bias from the gyroscope rate to get
     * the corrected angular velocity, then integrate it.
     */
    float rate = gyro_rate - kf->bias; /* Bias-corrected gyro rate */
    kf->angle += rate * dt;

    /* Bias prediction: bias remains unchanged (random walk model) */
    /* kf->bias = kf->bias; */

    /*
     * 1b. Error covariance prediction:
     *
     *   P(k|k-1) = F * P(k-1|k-1) * F^T + Q
     *
     * Expanding the matrix multiplication with:
     *   F = [1  -dt]    F^T = [1    0]    Q = [Q_angle*dt  0        ]
     *       [0   1 ]          [-dt  1]        [0           Q_bias*dt]
     *
     * The full expansion:
     *   P00_new = P00 + dt*(P11*dt - P01 - P10) + Q_angle*dt
     *   P01_new = P01 - P11*dt
     *   P10_new = P10 - P11*dt
     *   P11_new = P11 + Q_bias*dt
     *
     * Note: We scale Q by dt because the process noise accumulates
     * proportionally to the time step.
     */
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    float P10_temp = kf->P[1][0];
    float P11_temp = kf->P[1][1];

    kf->P[0][0] = P00_temp + dt * (dt * P11_temp - P01_temp - P10_temp + kf->Q_angle);
    kf->P[0][1] = P01_temp - dt * P11_temp;
    kf->P[1][0] = P10_temp - dt * P11_temp;
    kf->P[1][1] = P11_temp + kf->Q_bias * dt;

    /*
     * ================================================================
     *  STEP 2: UPDATE (Measurement Update / A Posteriori Estimate)
     * ================================================================
     *
     * Correct the predicted state using the accelerometer measurement.
     */

    /*
     * 2a. Innovation (measurement residual):
     *
     *   y = z - H * x_predicted
     *   y = accel_angle - angle_predicted
     *
     * The innovation represents the difference between what we measured
     * and what we expected to measure given our prediction.
     */
    float y = accel_angle - kf->angle;
    kf->innovation = y;

    /*
     * 2b. Innovation covariance:
     *
     *   S = H * P(k|k-1) * H^T + R
     *
     * Since H = [1  0], this simplifies to:
     *   S = P[0][0] + R_measure
     *
     * S represents the total uncertainty in the innovation.
     */
    float S = kf->P[0][0] + kf->R_measure;
    kf->S = S;

    /*
     * 2c. Kalman gain:
     *
     *   K = P(k|k-1) * H^T * S^-1
     *
     * Since H = [1  0] and S is scalar:
     *   K[0] = P[0][0] / S
     *   K[1] = P[1][0] / S
     *
     * The Kalman gain determines how much to trust the measurement
     * versus the prediction:
     *   K near 0: Trust the prediction (gyroscope)
     *   K near 1: Trust the measurement (accelerometer)
     */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    kf->K[0] = K0;
    kf->K[1] = K1;

    /*
     * 2d. State update:
     *
     *   x(k|k) = x(k|k-1) + K * y
     *
     *   angle = angle_predicted + K[0] * innovation
     *   bias  = bias_predicted  + K[1] * innovation
     *
     * The state is corrected by adding the Kalman gain weighted innovation.
     * The bias estimate is also updated, allowing the filter to track and
     * compensate for gyroscope drift over time.
     */
    kf->angle += K0 * y;
    kf->bias += K1 * y;

    /*
     * 2e. Error covariance update:
     *
     *   P(k|k) = (I - K * H) * P(k|k-1)
     *
     * Expanding with H = [1  0]:
     *   P00_new = P00 - K[0] * P00
     *   P01_new = P01 - K[0] * P01
     *   P10_new = P10 - K[1] * P00
     *   P11_new = P11 - K[1] * P01
     *
     * We must use the predicted P values (before this update) for all terms.
     */
    float P00_updated = kf->P[0][0];
    float P01_updated = kf->P[0][1];

    kf->P[0][0] -= K0 * P00_updated;
    kf->P[0][1] -= K0 * P01_updated;
    kf->P[1][0] -= K1 * P00_updated;
    kf->P[1][1] -= K1 * P01_updated;

    return kf->angle;
}

float Kalman_GetAngle(const Kalman_State_t *kf)
{
    return kf->angle;
}

float Kalman_GetBias(const Kalman_State_t *kf)
{
    return kf->bias;
}

float Kalman_GetRate(const Kalman_State_t *kf, float gyro_rate)
{
    return gyro_rate - kf->bias;
}

void Kalman_SetQAngle(Kalman_State_t *kf, float Q_angle)
{
    kf->Q_angle = Q_angle;
}

void Kalman_SetQBias(Kalman_State_t *kf, float Q_bias)
{
    kf->Q_bias = Q_bias;
}

void Kalman_SetRMeasure(Kalman_State_t *kf, float R_measure)
{
    kf->R_measure = R_measure;
}

void Kalman_Reset(Kalman_State_t *kf)
{
    float q_a = kf->Q_angle;
    float q_b = kf->Q_bias;
    float r_m = kf->R_measure;

    /* Reinitialize with preserved parameters */
    Kalman_InitCustom(kf, q_a, q_b, r_m);
}

/* ========================================================================== */
/*                   COMPLEMENTARY FILTER IMPLEMENTATION                      */
/* ========================================================================== */

void CompFilter_Init(ComplementaryFilter_State_t *cf)
{
    CompFilter_InitCustom(cf, COMPLEMENTARY_DEFAULT_ALPHA);
}

void CompFilter_InitCustom(ComplementaryFilter_State_t *cf, float alpha)
{
    cf->angle = 0.0f;
    cf->alpha = alpha;
    cf->initialized = false;
}

void CompFilter_SetAngle(ComplementaryFilter_State_t *cf, float initial_angle)
{
    cf->angle = initial_angle;
    cf->initialized = true;
}

float CompFilter_Update(ComplementaryFilter_State_t *cf,
                        float gyro_rate,
                        float accel_angle,
                        float dt)
{
    /*
     * On the first call, initialize the angle from the accelerometer
     * to avoid a large initial transient.
     */
    if (!cf->initialized)
    {
        cf->angle = accel_angle;
        cf->initialized = true;
        return cf->angle;
    }

    /*
     * Complementary filter equation:
     *
     *   angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle
     *
     * This is equivalent to:
     *   angle = alpha * angle_from_gyro + (1 - alpha) * angle_from_accel
     *
     * Where:
     *   - alpha * (angle + gyro_rate * dt) is a HIGH-PASS filter on gyro data
     *     (passes fast changes, blocks slow drift)
     *
     *   - (1 - alpha) * accel_angle is a LOW-PASS filter on accel data
     *     (passes stable orientation, blocks vibration/noise)
     *
     * The complementary property: the two filters sum to unity gain at
     * all frequencies, preserving the signal without gaps or overlaps.
     *
     * With alpha = 0.96 and dt = 0.005s:
     *   Time constant tau = dt * alpha / (1 - alpha) = 0.005 * 0.96 / 0.04 = 0.12s
     *
     * This means changes faster than ~0.12s come from the gyroscope,
     * and slower changes come from the accelerometer.
     */
    cf->angle = cf->alpha * (cf->angle + gyro_rate * dt) + (1.0f - cf->alpha) * accel_angle;

    return cf->angle;
}

float CompFilter_GetAngle(const ComplementaryFilter_State_t *cf)
{
    return cf->angle;
}

void CompFilter_Reset(ComplementaryFilter_State_t *cf)
{
    cf->angle = 0.0f;
    cf->initialized = false;
    /* alpha is preserved */
}
