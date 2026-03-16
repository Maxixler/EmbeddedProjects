/**
 * @file    fft_analyzer.h
 * @brief   FFT spectrum analyzer using ARM CMSIS-DSP library
 * @details Provides a complete FFT analysis pipeline for real-valued signals:
 *          - Configurable FFT sizes: 256, 512, 1024, 2048, 4096
 *          - Window functions: Rectangular, Hanning, Hamming, Blackman
 *          - DC offset removal before FFT computation
 *          - Magnitude spectrum via arm_cmplx_mag_f32
 *          - Peak frequency detection with interpolation
 *          - THD (Total Harmonic Distortion) calculation
 *          - SNR (Signal-to-Noise Ratio) estimation
 *          - Magnitude to dB conversion
 *
 *          Uses ARM CMSIS-DSP arm_rfft_fast_f32 for efficient FFT computation
 *          optimized for Cortex-M4 with FPU.
 *
 * @author  Embedded Systems Project
 * @version 1.0
 */

#ifndef FFT_ANALYZER_H
#define FFT_ANALYZER_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================== Includes ========================== */
#include "arm_math.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================== Defines =========================== */

/** @defgroup FFT_Sizes Supported FFT point sizes */
/** @{ */
#define FFT_SIZE_256 256U
#define FFT_SIZE_512 512U
#define FFT_SIZE_1024 1024U
#define FFT_SIZE_2048 2048U
#define FFT_SIZE_4096 4096U
/** @} */

/** @defgroup FFT_Limits Analysis limits and thresholds */
/** @{ */
#define FFT_MAX_SIZE 4096U             /**< Maximum supported FFT size */
#define FFT_MAX_HARMONICS 10U          /**< Maximum harmonics for THD calc */
#define FFT_MIN_DB (-120.0f)           /**< Floor value for dB conversion */
#define FFT_PEAK_THRESHOLD_DB (-60.0f) /**< Minimum dB level for peak detection */
#define FFT_MAX_PEAKS 10U              /**< Maximum number of peaks to detect */
    /** @} */

    /* ========================== Typedefs ========================== */

    /**
     * @brief Window function type enumeration
     */
    typedef enum
    {
        FFT_WINDOW_RECTANGULAR = 0, /**< No windowing (w[n] = 1) */
        FFT_WINDOW_HANNING,         /**< Hanning/Hann window (good general purpose) */
        FFT_WINDOW_HAMMING,         /**< Hamming window (better sidelobe rejection) */
        FFT_WINDOW_BLACKMAN,        /**< Blackman window (best sidelobe rejection) */
        FFT_WINDOW_COUNT            /**< Number of window types (sentinel) */
    } FFT_WindowType_t;

    /**
     * @brief Single peak detection result
     */
    typedef struct
    {
        float32_t frequency;    /**< Peak frequency in Hz */
        float32_t magnitude;    /**< Peak magnitude (linear) */
        float32_t magnitude_db; /**< Peak magnitude in dB */
        uint32_t bin_index;     /**< FFT bin index of the peak */
    } FFT_Peak_t;

    /**
     * @brief FFT analysis configuration
     * @details Specifies the FFT size, sampling rate, and window function.
     *          Passed to FFT_Analyzer_Init() to configure the analyzer.
     */
    typedef struct
    {
        uint32_t fft_size;            /**< FFT point count: 256/512/1024/2048/4096 */
        float32_t fs;                 /**< Sampling frequency in Hz */
        FFT_WindowType_t window_type; /**< Window function to apply before FFT */
    } FFT_Config_t;

    /**
     * @brief FFT analysis result structure
     * @details Contains all computed results from a single FFT analysis frame.
     *          Populated by FFT_Analyzer_Process() after a complete analysis.
     */
    typedef struct
    {
        /* Spectrum arrays (allocated by user, filled by analyzer) */
        float32_t *magnitude;    /**< Magnitude spectrum array [fft_size/2] */
        float32_t *magnitude_db; /**< Magnitude in dB array [fft_size/2] */
        float32_t *frequency;    /**< Frequency axis array [fft_size/2] in Hz */

        /* Peak detection results */
        FFT_Peak_t peaks[FFT_MAX_PEAKS]; /**< Detected peaks sorted by magnitude */
        uint32_t num_peaks;              /**< Number of detected peaks */
        float32_t peak_frequency;        /**< Frequency of the strongest peak (Hz) */
        float32_t peak_magnitude;        /**< Magnitude of the strongest peak (linear) */
        float32_t peak_magnitude_db;     /**< Magnitude of the strongest peak (dB) */

        /* Signal quality metrics */
        float32_t thd;       /**< Total Harmonic Distortion (%) */
        float32_t snr;       /**< Signal-to-Noise Ratio (dB) */
        float32_t dc_offset; /**< DC offset of input signal (ADC units) */
        float32_t rms_value; /**< RMS value of input signal */

        /* Analysis metadata */
        float32_t freq_resolution; /**< Frequency resolution: fs/N (Hz) */
        uint32_t fft_size;         /**< FFT size used for this analysis */
        float32_t fs;              /**< Sampling rate used for this analysis */
        bool valid;                /**< True if results are valid */
    } FFT_Result_t;

    /**
     * @brief FFT analyzer handle structure
     * @details Maintains the internal state and working buffers for the FFT analyzer.
     *          Must be initialized with FFT_Analyzer_Init() before use.
     */
    typedef struct
    {
        /* CMSIS-DSP FFT instance */
        arm_rfft_fast_instance_f32 fft_instance; /**< CMSIS-DSP RFFT instance */

        /* Configuration */
        FFT_Config_t config;       /**< Active configuration */
        float32_t freq_resolution; /**< Frequency resolution: fs / fft_size */

        /* Working buffers (pointers to user-allocated memory) */
        float32_t *fft_input;     /**< FFT input buffer [fft_size] */
        float32_t *fft_output;    /**< FFT output buffer [fft_size] */
        float32_t *window_coeffs; /**< Window function coefficients [fft_size] */
        float32_t *mag_buffer;    /**< Magnitude working buffer [fft_size/2] */

        /* State */
        bool initialized;      /**< True if analyzer is properly initialized */
        bool window_generated; /**< True if window coefficients are computed */
    } FFT_Analyzer_t;

    /* ========================== Function Prototypes ========================== */

    /**
     * @brief  Initialize the FFT analyzer
     * @details Sets up the CMSIS-DSP RFFT instance, generates window function
     *          coefficients, and prepares internal buffers. Must be called
     *          before any other FFT_Analyzer function.
     *
     * @param  analyzer     Pointer to FFT analyzer handle (pre-allocated)
     * @param  config       Pointer to configuration with FFT size, fs, window type
     * @param  fft_input    Working buffer for FFT input [fft_size floats]
     * @param  fft_output   Working buffer for FFT output [fft_size floats]
     * @param  window_buf   Buffer for window coefficients [fft_size floats]
     * @param  mag_buf      Buffer for magnitude values [fft_size/2 floats]
     * @retval ARM_MATH_SUCCESS on success, error code on failure
     */
    arm_status FFT_Analyzer_Init(FFT_Analyzer_t *analyzer,
                                 const FFT_Config_t *config,
                                 float32_t *fft_input,
                                 float32_t *fft_output,
                                 float32_t *window_buf,
                                 float32_t *mag_buf);

    /**
     * @brief  Run the complete FFT analysis pipeline
     * @details Performs the full analysis chain on raw ADC samples:
     *          1. Convert uint16 ADC data to float32
     *          2. Remove DC offset (mean subtraction)
     *          3. Apply window function
     *          4. Compute FFT using arm_rfft_fast_f32
     *          5. Compute magnitude spectrum
     *          6. Detect peaks
     *          7. Calculate THD and SNR
     *          8. Convert magnitudes to dB
     *
     * @param  analyzer     Pointer to initialized FFT analyzer
     * @param  adc_data     Raw ADC sample data (uint16_t array)
     * @param  num_samples  Number of ADC samples (must equal fft_size)
     * @param  result       Pointer to result structure (with pre-allocated arrays)
     * @retval true on success, false on error
     */
    bool FFT_Analyzer_Process(FFT_Analyzer_t *analyzer,
                              const uint16_t *adc_data,
                              uint32_t num_samples,
                              FFT_Result_t *result);

    /**
     * @brief  Apply the configured window function to a signal buffer
     * @details Multiplies each sample by the corresponding window coefficient.
     *          The window coefficients are pre-computed during init.
     *
     * @param  analyzer  Pointer to initialized FFT analyzer
     * @param  signal    Signal buffer to window [fft_size floats] (modified in-place)
     */
    void FFT_Analyzer_ApplyWindow(FFT_Analyzer_t *analyzer, float32_t *signal);

    /**
     * @brief  Compute the FFT of a real-valued signal
     * @details Wraps arm_rfft_fast_f32. Input is modified during computation.
     *          Output is in interleaved complex format: [Re0, Im0, Re1, Im1, ...]
     *
     * @param  analyzer  Pointer to initialized FFT analyzer
     * @param  input     Real input signal [fft_size floats] (modified in-place!)
     * @param  output    Complex FFT output [fft_size floats]
     */
    void FFT_Analyzer_ComputeFFT(FFT_Analyzer_t *analyzer,
                                 float32_t *input,
                                 float32_t *output);

    /**
     * @brief  Compute magnitude spectrum from complex FFT output
     * @details Calculates |X[k]| = sqrt(Re[k]^2 + Im[k]^2) for each bin
     *          using arm_cmplx_mag_f32 for SIMD-optimized computation.
     *
     * @param  analyzer       Pointer to initialized FFT analyzer
     * @param  fft_output     Complex FFT output [fft_size floats]
     * @param  magnitude_out  Magnitude values [fft_size/2 floats]
     */
    void FFT_Analyzer_ComputeMagnitude(FFT_Analyzer_t *analyzer,
                                       const float32_t *fft_output,
                                       float32_t *magnitude_out);

    /**
     * @brief  Find peaks in the magnitude spectrum
     * @details Identifies local maxima above the threshold in the magnitude
     *          spectrum. Peaks are sorted by magnitude (strongest first).
     *
     * @param  analyzer   Pointer to initialized FFT analyzer
     * @param  magnitude  Magnitude spectrum [fft_size/2 floats]
     * @param  peaks_out  Array to store detected peaks
     * @param  max_peaks  Maximum number of peaks to detect
     * @retval Number of peaks found
     */
    uint32_t FFT_Analyzer_FindPeaks(FFT_Analyzer_t *analyzer,
                                    const float32_t *magnitude,
                                    FFT_Peak_t *peaks_out,
                                    uint32_t max_peaks);

    /**
     * @brief  Calculate Total Harmonic Distortion (THD)
     * @details THD is the ratio of the sum of harmonic magnitudes to the
     *          fundamental magnitude, expressed as a percentage.
     *
     *          THD(%) = 100 * sqrt(V2^2 + V3^2 + ... + Vn^2) / V1
     *
     *          where V1 is the fundamental and V2..Vn are harmonics.
     *
     * @param  analyzer   Pointer to initialized FFT analyzer
     * @param  magnitude  Magnitude spectrum [fft_size/2 floats]
     * @param  fund_bin   Bin index of the fundamental frequency
     * @retval THD value in percent (0-100+)
     */
    float32_t FFT_Analyzer_ComputeTHD(FFT_Analyzer_t *analyzer,
                                      const float32_t *magnitude,
                                      uint32_t fund_bin);

    /**
     * @brief  Calculate Signal-to-Noise Ratio (SNR)
     * @details Estimates SNR by comparing the signal power (fundamental + harmonics)
     *          to the noise floor power across the spectrum.
     *
     * @param  analyzer   Pointer to initialized FFT analyzer
     * @param  magnitude  Magnitude spectrum [fft_size/2 floats]
     * @param  fund_bin   Bin index of the fundamental frequency
     * @retval SNR value in dB
     */
    float32_t FFT_Analyzer_ComputeSNR(FFT_Analyzer_t *analyzer,
                                      const float32_t *magnitude,
                                      uint32_t fund_bin);

    /**
     * @brief  Convert magnitude values to dB scale
     * @details Computes 20*log10(magnitude/reference) for each bin.
     *          Values below FFT_MIN_DB are clamped to FFT_MIN_DB.
     *
     * @param  magnitude     Input magnitude array [length floats]
     * @param  magnitude_db  Output dB array [length floats]
     * @param  length        Number of elements to convert
     * @param  reference     Reference value for 0 dB (typically max magnitude)
     */
    void FFT_Analyzer_MagnitudeToDb(const float32_t *magnitude,
                                    float32_t *magnitude_db,
                                    uint32_t length,
                                    float32_t reference);

    /**
     * @brief  Generate frequency axis values
     * @details Fills an array with the center frequency of each FFT bin.
     *          freq[k] = k * fs / fft_size, for k = 0 to fft_size/2 - 1
     *
     * @param  analyzer   Pointer to initialized FFT analyzer
     * @param  freq_out   Output frequency array [fft_size/2 floats]
     */
    void FFT_Analyzer_GenerateFreqAxis(FFT_Analyzer_t *analyzer,
                                       float32_t *freq_out);

    /**
     * @brief  Change the window function type at runtime
     * @details Regenerates window coefficients for the new window type.
     *          Can be called while the analyzer is active; takes effect
     *          on the next call to FFT_Analyzer_Process().
     *
     * @param  analyzer     Pointer to initialized FFT analyzer
     * @param  window_type  New window type to apply
     * @retval true on success, false on invalid type
     */
    bool FFT_Analyzer_SetWindowType(FFT_Analyzer_t *analyzer,
                                    FFT_WindowType_t window_type);

    /**
     * @brief  Change the FFT size at runtime
     * @details Reinitializes the CMSIS-DSP FFT instance and regenerates
     *          window coefficients for the new size. Buffers must be large
     *          enough to accommodate the new size.
     *
     * @param  analyzer  Pointer to initialized FFT analyzer
     * @param  new_size  New FFT size (256/512/1024/2048/4096)
     * @retval ARM_MATH_SUCCESS on success, error code on invalid size
     */
    arm_status FFT_Analyzer_SetSize(FFT_Analyzer_t *analyzer, uint32_t new_size);

    /**
     * @brief  Get the name string for a window type
     * @param  type  Window type enumeration value
     * @retval Pointer to a constant string name (e.g., "HANNING")
     */
    const char *FFT_Analyzer_GetWindowName(FFT_WindowType_t type);

#ifdef __cplusplus
}
#endif

#endif /* FFT_ANALYZER_H */
