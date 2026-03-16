/**
 * @file    encoder.h
 * @brief   Quadrature Encoder Interface for DC Motor Position/Speed Feedback
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details This module provides a complete interface for reading a quadrature
 *          incremental encoder using the STM32 timer hardware encoder mode.
 *
 *          Features:
 *          - TIM3 in encoder mode (quadrature decoding, both edges = 4x resolution)
 *          - PA6 (TIM3_CH1) = Encoder Phase A
 *          - PA7 (TIM3_CH2) = Encoder Phase B
 *          - 32-bit extended position tracking (overflow handling via interrupt)
 *          - Velocity calculation in RPM using pulse counting method
 *          - Direction detection
 *          - Configurable counts per revolution (CPR)
 *          - Input noise filtering
 *
 * @note    The timer counter (TIM3->CNT) is 16-bit (0..65535). To track
 *          absolute position beyond 65535 counts, the timer update interrupt
 *          is used to maintain a 32-bit overflow counter.
 */

#ifndef ENCODER_H
#define ENCODER_H

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

/** Encoder hardware configuration */
#define ENCODER_TIMER_PERIOD 65535U /**< 16-bit timer max count    */
#define ENCODER_TIMER_HALF 32768U   /**< Half of timer period      */

/** Default encoder parameters */
#define ENCODER_DEFAULT_CPR 400U   /**< Counts per revolution (before quadrature) */
#define ENCODER_QUADRATURE_MULT 4U /**< 4x mode (both edges, both channels) */

/**
 * Total counts per revolution in 4x mode:
 * ENCODER_DEFAULT_CPR * ENCODER_QUADRATURE_MULT = 400 * 4 = 1600 counts/rev
 */
#define ENCODER_DEFAULT_COUNTS_PER_REV (ENCODER_DEFAULT_CPR * ENCODER_QUADRATURE_MULT)

/** Velocity calculation parameters */
#define ENCODER_VELOCITY_SAMPLE_HZ 100U    /**< Velocity sampling rate (Hz) */
#define ENCODER_VELOCITY_FILTER_ALPHA 0.3f /**< Velocity low-pass filter coefficient */

/** Input filter value for TIM3 CCMR (noise rejection)
 *  ICxF = 0110b -> sampling at fDTS/4, N=6
 *  This filters out pulses shorter than ~200ns at 84MHz timer clock
 */
#define ENCODER_INPUT_FILTER 6U

    /* -------------------------------------------------------------------------- */
    /*                              Type Definitions                              */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief Encoder rotation direction
     */
    typedef enum
    {
        ENCODER_DIR_FORWARD = 0, /**< Forward (counter incrementing)   */
        ENCODER_DIR_REVERSE = 1  /**< Reverse (counter decrementing)   */
    } Encoder_Direction_t;

    /**
     * @brief Encoder handle structure
     *
     * Contains hardware configuration, encoder parameters, and runtime state
     * for one encoder instance.
     */
    typedef struct
    {
        /* ---- Hardware Handle ---- */
        TIM_HandleTypeDef *htim; /**< Timer handle (TIM3 in encoder mode) */

        /* ---- Encoder Parameters ---- */
        uint32_t cpr;            /**< Counts per revolution (base, without 4x) */
        uint32_t counts_per_rev; /**< Total counts per revolution (with 4x) */

        /* ---- Position Tracking ---- */
        int32_t position;       /**< Extended 32-bit absolute position (counts) */
        int32_t overflow_count; /**< Number of timer overflows (+/-) */
        uint16_t last_counter;  /**< Previous timer counter snapshot */

        /* ---- Velocity ---- */
        float velocity_rpm;          /**< Current velocity in RPM (filtered) */
        float velocity_raw;          /**< Raw (unfiltered) velocity in RPM */
        int32_t last_position;       /**< Position at last velocity calculation */
        float velocity_filter_alpha; /**< Low-pass filter coefficient */

        /* ---- Direction ---- */
        Encoder_Direction_t direction; /**< Current rotation direction */

        /* ---- State ---- */
        bool initialized; /**< Initialization complete flag */
    } Encoder_Handle_t;

    /* -------------------------------------------------------------------------- */
    /*                          Function Prototypes                               */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief  Initialize the encoder interface
     *
     * Configures TIM3 in encoder mode with quadrature decoding (4x resolution).
     * The encoder starts counting from position 0.
     *
     * @param  henc  Pointer to encoder handle structure
     * @param  htim  Pointer to HAL TIM handle (must be TIM3, configured for encoder mode)
     * @param  cpr   Counts per revolution of the encoder (before 4x multiplication)
     * @return HAL_StatusTypeDef  HAL_OK on success
     */
    HAL_StatusTypeDef Encoder_Init(Encoder_Handle_t *henc,
                                   TIM_HandleTypeDef *htim,
                                   uint32_t cpr);

    /**
     * @brief  Get absolute position in encoder counts
     *
     * Returns the 32-bit extended position that accounts for timer overflow.
     * The position is signed: positive values for forward rotation,
     * negative for reverse.
     *
     * @param  henc  Pointer to encoder handle
     * @return int32_t  Current position in encoder counts
     */
    int32_t Encoder_GetPosition(Encoder_Handle_t *henc);

    /**
     * @brief  Get position in degrees (0..360)
     *
     * @param  henc  Pointer to encoder handle
     * @return float  Position in degrees within one revolution
     */
    float Encoder_GetPositionDegrees(Encoder_Handle_t *henc);

    /**
     * @brief  Get velocity in RPM
     *
     * Calculates motor speed using the pulse counting method:
     *   RPM = (delta_position / counts_per_rev) * (60 / dt)
     *
     * This function should be called at a fixed rate (e.g., 100Hz from
     * the PID control timer). It applies a low-pass filter to reduce noise.
     *
     * @param  henc  Pointer to encoder handle
     * @param  dt    Time elapsed since last call, in seconds
     * @return float  Current velocity in RPM (positive = forward, negative = reverse)
     */
    float Encoder_GetVelocityRPM(Encoder_Handle_t *henc, float dt);

    /**
     * @brief  Get raw (unfiltered) velocity in RPM
     *
     * @param  henc  Pointer to encoder handle
     * @return float  Raw velocity in RPM
     */
    float Encoder_GetRawVelocityRPM(const Encoder_Handle_t *henc);

    /**
     * @brief  Get current rotation direction
     *
     * @param  henc  Pointer to encoder handle
     * @return Encoder_Direction_t  ENCODER_DIR_FORWARD or ENCODER_DIR_REVERSE
     */
    Encoder_Direction_t Encoder_GetDirection(const Encoder_Handle_t *henc);

    /**
     * @brief  Reset encoder position to zero
     *
     * Clears the position counter, overflow count, and timer counter.
     * Useful when establishing a new reference position.
     *
     * @param  henc  Pointer to encoder handle
     */
    void Encoder_ResetPosition(Encoder_Handle_t *henc);

    /**
     * @brief  Handle timer overflow/underflow interrupt
     *
     * This function MUST be called from the TIM3 update interrupt handler
     * (TIM3_IRQHandler -> HAL_TIM_PeriodElapsedCallback). It tracks the
     * number of overflows to maintain the 32-bit extended position.
     *
     * @param  henc  Pointer to encoder handle
     */
    void Encoder_OverflowHandler(Encoder_Handle_t *henc);

    /**
     * @brief  Set velocity filter coefficient
     *
     * @param  henc   Pointer to encoder handle
     * @param  alpha  Filter coefficient (0..1). Lower = more smoothing.
     */
    void Encoder_SetVelocityFilter(Encoder_Handle_t *henc, float alpha);

    /**
     * @brief  Set counts per revolution
     *
     * @param  henc  Pointer to encoder handle
     * @param  cpr   New counts per revolution (before 4x multiplication)
     */
    void Encoder_SetCPR(Encoder_Handle_t *henc, uint32_t cpr);

    /**
     * @brief  Get the raw timer counter value (for debugging)
     *
     * @param  henc  Pointer to encoder handle
     * @return uint16_t  Current TIM3->CNT value (0..65535)
     */
    uint16_t Encoder_GetRawCounter(const Encoder_Handle_t *henc);

    /**
     * @brief  Check if encoder is initialized
     *
     * @param  henc  Pointer to encoder handle
     * @return bool  true if initialized
     */
    bool Encoder_IsInitialized(const Encoder_Handle_t *henc);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
