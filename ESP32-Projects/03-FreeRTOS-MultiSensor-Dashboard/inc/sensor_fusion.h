/**
 * @file    sensor_fusion.h
 * @brief   Sensor fusion module - moving average, complementary filter,
 *          and threshold-based alarm detection.
 *
 * @details Provides three core signal-processing building blocks:
 *
 *          1. **Moving Average Filter**
 *             Configurable-window FIR filter for smoothing noisy sensor
 *             readings (temperature, humidity, pressure, ADC).
 *
 *          2. **Complementary Filter**
 *             Fuses accelerometer and gyroscope data to produce stable
 *             roll and pitch estimates.  The filter equation is:
 *
 *                 angle = alpha * (angle + gyro * dt)
 *                       + (1 - alpha) * accel_angle
 *
 *             where alpha is typically 0.96 - 0.98.
 *
 *          3. **Threshold Detector with Hysteresis**
 *             Prevents rapid toggling around a set-point by requiring the
 *             signal to exceed (threshold + hysteresis) to trigger, and
 *             drop below (threshold - hysteresis) to clear.
 *
 * @version 1.0
 * @date    2026-03-16
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

/* -------------------------------------------------------------------------- */
/*                                  Includes                                  */
/* -------------------------------------------------------------------------- */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/** Maximum window size for the moving average filter. */
#define MOVING_AVG_MAX_WINDOW (32)

/** Default window size if none is specified. */
#define MOVING_AVG_DEFAULT_WINDOW (10)

/** Default complementary filter coefficient (alpha). */
#define COMP_FILTER_DEFAULT_ALPHA (0.96f)

/** Minimum allowable dt for complementary filter (prevents division issues). */
#define COMP_FILTER_MIN_DT (0.001f)

/** Maximum allowable dt for complementary filter (prevents integration drift). */
#define COMP_FILTER_MAX_DT (1.0f)

/** Pi constant for angle conversions. */
#define FUSION_PI (3.14159265358979f)

/** Radians to degrees conversion factor. */
#define FUSION_RAD_TO_DEG (180.0f / FUSION_PI)

/** Degrees to radians conversion factor. */
#define FUSION_DEG_TO_RAD (FUSION_PI / 180.0f)

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Moving average filter context.
     *
     * @details Maintains a circular buffer of the last N samples and a running
     *          sum for O(1) per-sample computation.
     */
    typedef struct
    {
        float buffer[MOVING_AVG_MAX_WINDOW]; /**< Circular sample buffer. */
        uint16_t window_size;                /**< Active window size (1..MAX). */
        uint16_t index;                      /**< Current write index. */
        uint16_t count;                      /**< Number of samples inserted. */
        float sum;                           /**< Running sum of buffer. */
    } moving_avg_t;

    /**
     * @brief   Complementary filter state for one axis.
     *
     * @details Stores the fused angle estimate and the filter coefficient.
     */
    typedef struct
    {
        float angle;      /**< Current fused angle estimate (degrees). */
        float alpha;      /**< Filter coefficient (0.0 - 1.0). */
        bool initialized; /**< True after first update. */
    } comp_filter_axis_t;

    /**
     * @brief   Complementary filter state for roll and pitch.
     */
    typedef struct
    {
        comp_filter_axis_t roll;  /**< Roll axis filter state. */
        comp_filter_axis_t pitch; /**< Pitch axis filter state. */
    } comp_filter_t;

    /**
     * @brief   Threshold detector state (with hysteresis).
     */
    typedef struct
    {
        float threshold;   /**< Trigger threshold value. */
        float hysteresis;  /**< Hysteresis band (must be >= 0). */
        bool is_triggered; /**< True if currently in triggered state. */
        bool upper_detect; /**< True = trigger when value > threshold,
                                False = trigger when value < threshold. */
    } threshold_detector_t;

    /* -------------------------------------------------------------------------- */
    /*                    Moving Average - Public Function Prototypes             */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialise a moving average filter context.
     *
     * @param[out]  ctx         Pointer to the filter context to initialise.
     * @param[in]   window_size Desired window size (1 .. MOVING_AVG_MAX_WINDOW).
     *                          Clamped if out of range.
     */
    void moving_avg_init(moving_avg_t *ctx, uint16_t window_size);

    /**
     * @brief   Reset the filter to its initial (empty) state.
     *
     * @param[out]  ctx     Pointer to the filter context.
     */
    void moving_avg_reset(moving_avg_t *ctx);

    /**
     * @brief   Feed a new sample into the filter and return the average.
     *
     * @param[in,out]   ctx     Pointer to the filter context.
     * @param[in]       sample  New sample value.
     * @return  Current moving average.
     */
    float moving_avg_update(moving_avg_t *ctx, float sample);

    /**
     * @brief   Get the current average without adding a new sample.
     *
     * @param[in]   ctx     Pointer to the filter context.
     * @return  Current moving average, or 0.0 if no samples.
     */
    float moving_avg_get(const moving_avg_t *ctx);

    /**
     * @brief   Check whether the filter buffer is fully populated.
     *
     * @param[in]   ctx     Pointer to the filter context.
     * @return  True if count >= window_size.
     */
    bool moving_avg_is_full(const moving_avg_t *ctx);

    /* -------------------------------------------------------------------------- */
    /*                 Complementary Filter - Public Function Prototypes          */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialise a complementary filter for roll and pitch.
     *
     * @param[out]  filter  Pointer to the filter state to initialise.
     * @param[in]   alpha   Filter coefficient (0.0 - 1.0).  A higher alpha
     *                      trusts the gyroscope more; a lower alpha trusts
     *                      the accelerometer more.  Typical: 0.96 - 0.98.
     */
    void comp_filter_init(comp_filter_t *filter, float alpha);

    /**
     * @brief   Reset the filter state (angles set to zero, uninitialised).
     *
     * @param[out]  filter  Pointer to the filter state.
     */
    void comp_filter_reset(comp_filter_t *filter);

    /**
     * @brief   Update the complementary filter with new sensor readings.
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
                            float dt);

    /**
     * @brief   Get the current roll angle estimate.
     *
     * @param[in]   filter  Pointer to the filter state.
     * @return  Roll angle in degrees.
     */
    float comp_filter_get_roll(const comp_filter_t *filter);

    /**
     * @brief   Get the current pitch angle estimate.
     *
     * @param[in]   filter  Pointer to the filter state.
     * @return  Pitch angle in degrees.
     */
    float comp_filter_get_pitch(const comp_filter_t *filter);

    /**
     * @brief   Compute the combined tilt magnitude from roll and pitch.
     *
     * @details tilt = sqrt(roll^2 + pitch^2)
     *
     * @param[in]   filter  Pointer to the filter state.
     * @return  Tilt magnitude in degrees.
     */
    float comp_filter_get_tilt(const comp_filter_t *filter);

    /* -------------------------------------------------------------------------- */
    /*                Threshold Detector - Public Function Prototypes             */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief   Initialise a threshold detector.
     *
     * @param[out]  det             Pointer to the detector to initialise.
     * @param[in]   threshold       Trigger threshold value.
     * @param[in]   hysteresis      Hysteresis band (>= 0).
     * @param[in]   upper_detect    True to trigger when value > threshold,
     *                              false to trigger when value < threshold.
     */
    void threshold_init(threshold_detector_t *det,
                        float threshold,
                        float hysteresis,
                        bool upper_detect);

    /**
     * @brief   Evaluate a new sample against the threshold.
     *
     * @param[in,out]   det     Pointer to the detector.
     * @param[in]       value   Current sample value.
     * @return  True if the detector is currently in the triggered state.
     */
    bool threshold_evaluate(threshold_detector_t *det, float value);

    /**
     * @brief   Check the current triggered state without providing a new sample.
     *
     * @param[in]   det     Pointer to the detector.
     * @return  True if currently triggered.
     */
    bool threshold_is_triggered(const threshold_detector_t *det);

    /**
     * @brief   Reset the detector to the non-triggered state.
     *
     * @param[out]  det     Pointer to the detector.
     */
    void threshold_reset(threshold_detector_t *det);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_FUSION_H */
