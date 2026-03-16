/**
 * @file    sensor_fusion.c
 * @brief   Sensor fusion module implementation - moving average filter,
 *          complementary filter, and threshold detection with hysteresis.
 *
 * @details Implements three signal-processing building blocks used by the
 *          data_process_task to convert raw sensor readings into stable,
 *          filtered outputs:
 *
 *          1. **Moving Average Filter**
 *             O(1) per-sample circular-buffer implementation.  A running sum
 *             is maintained so that each update only requires one addition
 *             and one subtraction regardless of window size.
 *
 *             Initialisation:  sum = 0, count = 0, index = 0
 *             Update:
 *               if (count == window_size)
 *                   sum -= buffer[index];   // remove oldest sample
 *               buffer[index] = sample;
 *               sum += sample;
 *               index = (index + 1) % window_size;
 *               if (count < window_size) count++;
 *               return sum / count;
 *
 *          2. **Complementary Filter**
 *             Fuses accelerometer (low-frequency trust) and gyroscope
 *             (high-frequency trust) to produce a stable orientation
 *             estimate.  The discrete-time equation is:
 *
 *               angle[k] = alpha * (angle[k-1] + gyro_rate * dt)
 *                        + (1 - alpha) * accel_angle
 *
 *             Roll is computed from atan2(ay, az) and pitch from
 *             atan2(-ax, sqrt(ay^2 + az^2)).
 *
 *          3. **Threshold Detector with Hysteresis**
 *             Uses a Schmitt-trigger-style mechanism:
 *               - Upper detect:  trigger when value > (threshold + hysteresis),
 *                                clear  when value < (threshold - hysteresis).
 *               - Lower detect:  trigger when value < (threshold - hysteresis),
 *                                clear  when value > (threshold + hysteresis).
 *
 * @version 1.0
 * @date    2026-03-16
 */

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include "sensor_fusion.h"
#include <string.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                    Moving Average - Public Function Definitions             */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise a moving average filter context.
 *
 * @details Clears the circular buffer, resets the running sum and sample
 *          count to zero, and clamps the requested window size to the
 *          valid range [1 .. MOVING_AVG_MAX_WINDOW].
 */
void moving_avg_init(moving_avg_t *ctx, uint16_t window_size)
{
    if (ctx == NULL)
    {
        return;
    }

    memset(ctx, 0, sizeof(moving_avg_t));

    /* Clamp window_size to valid range. */
    if (window_size == 0)
    {
        window_size = MOVING_AVG_DEFAULT_WINDOW;
    }
    else if (window_size > MOVING_AVG_MAX_WINDOW)
    {
        window_size = MOVING_AVG_MAX_WINDOW;
    }

    ctx->window_size = window_size;
    ctx->index = 0;
    ctx->count = 0;
    ctx->sum = 0.0f;
}

/**
 * @brief   Reset the filter to its initial (empty) state.
 *
 * @details Preserves the configured window_size but clears all samples,
 *          the running sum, and the write index.
 */
void moving_avg_reset(moving_avg_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    uint16_t saved_window = ctx->window_size;
    memset(ctx, 0, sizeof(moving_avg_t));
    ctx->window_size = saved_window;
}

/**
 * @brief   Feed a new sample into the filter and return the current average.
 *
 * @details Algorithm (O(1) per sample):
 *          1. If the buffer is full, subtract the oldest sample from the
 *             running sum before overwriting it.
 *          2. Store the new sample at the current write index.
 *          3. Add the new sample to the running sum.
 *          4. Advance the write index (wrapping via modulo).
 *          5. Increment the count if the buffer is not yet full.
 *          6. Return sum / count.
 *
 * @param[in,out]   ctx     Pointer to the filter context.
 * @param[in]       sample  New sample value.
 * @return  Current moving average after inserting the sample.
 */
float moving_avg_update(moving_avg_t *ctx, float sample)
{
    if (ctx == NULL)
    {
        return 0.0f;
    }

    /* If the buffer is already full, remove the oldest value from the sum. */
    if (ctx->count >= ctx->window_size)
    {
        ctx->sum -= ctx->buffer[ctx->index];
    }

    /* Insert the new sample. */
    ctx->buffer[ctx->index] = sample;
    ctx->sum += sample;

    /* Advance the circular index. */
    ctx->index = (uint16_t)((ctx->index + 1U) % ctx->window_size);

    /* Track how many samples have been inserted (up to window_size). */
    if (ctx->count < ctx->window_size)
    {
        ctx->count++;
    }

    return ctx->sum / (float)ctx->count;
}

/**
 * @brief   Get the current average without adding a new sample.
 *
 * @details Simply returns sum / count.  If no samples have been inserted
 *          yet (count == 0), returns 0.0f to avoid division by zero.
 */
float moving_avg_get(const moving_avg_t *ctx)
{
    if (ctx == NULL || ctx->count == 0)
    {
        return 0.0f;
    }

    return ctx->sum / (float)ctx->count;
}

/**
 * @brief   Check whether the filter buffer is fully populated.
 *
 * @return  true if count >= window_size, false otherwise.
 */
bool moving_avg_is_full(const moving_avg_t *ctx)
{
    if (ctx == NULL)
    {
        return false;
    }

    return (ctx->count >= ctx->window_size);
}

/* -------------------------------------------------------------------------- */
/*                 Complementary Filter - Public Function Definitions          */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise a complementary filter for roll and pitch.
 *
 * @details Sets the filter coefficient (alpha) for both axes and marks
 *          the filter as uninitialised.  On the first call to
 *          comp_filter_update() the angle will be seeded directly from
 *          the accelerometer rather than applying the recursive equation.
 */
void comp_filter_init(comp_filter_t *filter, float alpha)
{
    if (filter == NULL)
    {
        return;
    }

    memset(filter, 0, sizeof(comp_filter_t));

    /* Clamp alpha to valid range [0.0, 1.0]. */
    if (alpha < 0.0f)
    {
        alpha = 0.0f;
    }
    else if (alpha > 1.0f)
    {
        alpha = 1.0f;
    }

    filter->roll.alpha = alpha;
    filter->roll.angle = 0.0f;
    filter->roll.initialized = false;

    filter->pitch.alpha = alpha;
    filter->pitch.angle = 0.0f;
    filter->pitch.initialized = false;
}

/**
 * @brief   Reset the filter state (angles set to zero, uninitialised).
 */
void comp_filter_reset(comp_filter_t *filter)
{
    if (filter == NULL)
    {
        return;
    }

    float saved_alpha = filter->roll.alpha;
    memset(filter, 0, sizeof(comp_filter_t));
    filter->roll.alpha = saved_alpha;
    filter->pitch.alpha = saved_alpha;
}

/**
 * @brief   Update the complementary filter with new accelerometer and
 *          gyroscope readings.
 *
 * @details Computation steps:
 *
 *          1. Compute the accelerometer-derived angles:
 *               roll_accel  = atan2(ay, az)             * RAD_TO_DEG
 *               pitch_accel = atan2(-ax, sqrt(ay^2+az^2)) * RAD_TO_DEG
 *
 *          2. If this is the first update (not yet initialised), seed the
 *             fused angle directly from the accelerometer angles.
 *
 *          3. Otherwise, apply the complementary filter equation:
 *               angle = alpha * (prev_angle + gyro_rate * dt)
 *                     + (1 - alpha) * accel_angle
 *
 *          4. Clamp dt to [COMP_FILTER_MIN_DT, COMP_FILTER_MAX_DT] to
 *             prevent integration blow-up or division issues.
 *
 * @param[in,out]   filter  Pointer to the filter state.
 * @param[in]       ax      Accelerometer X (g).
 * @param[in]       ay      Accelerometer Y (g).
 * @param[in]       az      Accelerometer Z (g).
 * @param[in]       gx      Gyroscope X (deg/s) - roll rate.
 * @param[in]       gy      Gyroscope Y (deg/s) - pitch rate.
 * @param[in]       dt      Time step in seconds since last update.
 */
void comp_filter_update(comp_filter_t *filter,
                        float ax, float ay, float az,
                        float gx, float gy,
                        float dt)
{
    if (filter == NULL)
    {
        return;
    }

    /* Clamp dt to prevent numerical issues. */
    if (dt < COMP_FILTER_MIN_DT)
    {
        dt = COMP_FILTER_MIN_DT;
    }
    else if (dt > COMP_FILTER_MAX_DT)
    {
        dt = COMP_FILTER_MAX_DT;
    }

    /*
     * Step 1: Compute accelerometer-derived angles.
     *
     * Roll  = atan2(ay, az)  -- rotation about the X-axis.
     * Pitch = atan2(-ax, sqrt(ay^2 + az^2))  -- rotation about the Y-axis.
     */
    float roll_accel = atan2f(ay, az) * FUSION_RAD_TO_DEG;
    float pitch_accel = atan2f(-ax, sqrtf(ay * ay + az * az)) * FUSION_RAD_TO_DEG;

    /*
     * Step 2 / 3: Seed or apply the complementary filter.
     */
    if (!filter->roll.initialized)
    {
        /* First update: seed from accelerometer. */
        filter->roll.angle = roll_accel;
        filter->roll.initialized = true;
    }
    else
    {
        /* Complementary filter equation for roll. */
        float alpha = filter->roll.alpha;
        filter->roll.angle = alpha * (filter->roll.angle + gx * dt) + (1.0f - alpha) * roll_accel;
    }

    if (!filter->pitch.initialized)
    {
        filter->pitch.angle = pitch_accel;
        filter->pitch.initialized = true;
    }
    else
    {
        float alpha = filter->pitch.alpha;
        filter->pitch.angle = alpha * (filter->pitch.angle + gy * dt) + (1.0f - alpha) * pitch_accel;
    }
}

/**
 * @brief   Get the current roll angle estimate.
 *
 * @return  Roll angle in degrees, or 0.0f if the filter is NULL.
 */
float comp_filter_get_roll(const comp_filter_t *filter)
{
    if (filter == NULL)
    {
        return 0.0f;
    }

    return filter->roll.angle;
}

/**
 * @brief   Get the current pitch angle estimate.
 *
 * @return  Pitch angle in degrees, or 0.0f if the filter is NULL.
 */
float comp_filter_get_pitch(const comp_filter_t *filter)
{
    if (filter == NULL)
    {
        return 0.0f;
    }

    return filter->pitch.angle;
}

/**
 * @brief   Compute the combined tilt magnitude from roll and pitch.
 *
 * @details Uses the Euclidean norm:
 *            tilt = sqrt(roll^2 + pitch^2)
 *
 *          This gives a single scalar indicating how far the device
 *          is tilted from the horizontal plane.
 *
 * @return  Tilt magnitude in degrees, or 0.0f if the filter is NULL.
 */
float comp_filter_get_tilt(const comp_filter_t *filter)
{
    if (filter == NULL)
    {
        return 0.0f;
    }

    float roll = filter->roll.angle;
    float pitch = filter->pitch.angle;

    return sqrtf(roll * roll + pitch * pitch);
}

/* -------------------------------------------------------------------------- */
/*                Threshold Detector - Public Function Definitions             */
/* -------------------------------------------------------------------------- */

/**
 * @brief   Initialise a threshold detector with hysteresis.
 *
 * @details Sets the trigger threshold, hysteresis band, and detection
 *          direction.  The detector starts in the non-triggered state.
 *
 *          For upper_detect == true:
 *            - Triggers  when value > (threshold + hysteresis)
 *            - Clears    when value < (threshold - hysteresis)
 *
 *          For upper_detect == false:
 *            - Triggers  when value < (threshold - hysteresis)
 *            - Clears    when value > (threshold + hysteresis)
 */
void threshold_init(threshold_detector_t *det,
                    float threshold,
                    float hysteresis,
                    bool upper_detect)
{
    if (det == NULL)
    {
        return;
    }

    det->threshold = threshold;
    det->hysteresis = (hysteresis >= 0.0f) ? hysteresis : 0.0f;
    det->is_triggered = false;
    det->upper_detect = upper_detect;
}

/**
 * @brief   Evaluate a new sample against the threshold with hysteresis.
 *
 * @details Implements a Schmitt-trigger-style state machine:
 *
 *          Upper detect mode (trigger on HIGH):
 *          ```
 *                       +---- trigger level (threshold + hyst)
 *                       |
 *           value ------+---->  is_triggered = true
 *                       |
 *                       +---- clear level   (threshold - hyst)
 *                       |
 *           value ------+---->  is_triggered = false
 *          ```
 *
 *          Lower detect mode (trigger on LOW) uses the mirrored logic.
 *
 * @return  Current triggered state after evaluating the sample.
 */
bool threshold_evaluate(threshold_detector_t *det, float value)
{
    if (det == NULL)
    {
        return false;
    }

    if (det->upper_detect)
    {
        /* Trigger when value exceeds the upper band. */
        if (!det->is_triggered)
        {
            if (value > (det->threshold + det->hysteresis))
            {
                det->is_triggered = true;
            }
        }
        else
        {
            /* Clear when value drops below the lower band. */
            if (value < (det->threshold - det->hysteresis))
            {
                det->is_triggered = false;
            }
        }
    }
    else
    {
        /* Trigger when value drops below the lower band. */
        if (!det->is_triggered)
        {
            if (value < (det->threshold - det->hysteresis))
            {
                det->is_triggered = true;
            }
        }
        else
        {
            /* Clear when value exceeds the upper band. */
            if (value > (det->threshold + det->hysteresis))
            {
                det->is_triggered = false;
            }
        }
    }

    return det->is_triggered;
}

/**
 * @brief   Check the current triggered state without providing a new sample.
 *
 * @return  true if currently triggered, false otherwise.
 */
bool threshold_is_triggered(const threshold_detector_t *det)
{
    if (det == NULL)
    {
        return false;
    }

    return det->is_triggered;
}

/**
 * @brief   Reset the detector to the non-triggered state.
 *
 * @details Clears the is_triggered flag without changing the threshold,
 *          hysteresis, or detection direction configuration.
 */
void threshold_reset(threshold_detector_t *det)
{
    if (det == NULL)
    {
        return;
    }

    det->is_triggered = false;
}
