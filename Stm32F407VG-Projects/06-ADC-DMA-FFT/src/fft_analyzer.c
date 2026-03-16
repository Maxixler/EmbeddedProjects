/**
 * @file    fft_analyzer.c
 * @brief   FFT spectrum analyzer implementation using ARM CMSIS-DSP
 * @details Implements a complete real-time FFT analysis pipeline:
 *
 *          Processing chain:
 *          ADC uint16 data -> float32 conversion -> DC offset removal ->
 *          Window function application -> FFT (arm_rfft_fast_f32) ->
 *          Complex magnitude (arm_cmplx_mag_f32) -> Peak detection ->
 *          THD/SNR calculation -> dB conversion
 *
 *          The CMSIS-DSP library exploits the Cortex-M4 FPU and SIMD
 *          instructions for maximum throughput. A 1024-point FFT
 *          completes in approximately 250 microseconds at 168 MHz.
 *
 * @author  Embedded Systems Project
 * @version 1.0
 */

/* ========================== Includes ========================== */
#include "fft_analyzer.h"
#include <string.h>
#include <math.h>

/* ========================== Private Defines ========================== */

/** @brief Pi constant for window function calculations */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/** @brief Two-pi constant */
#define TWO_PI (2.0f * M_PI)

/** @brief Four-pi constant (used in Blackman window) */
#define FOUR_PI (4.0f * M_PI)

/** @brief ADC reference voltage (3.3V for STM32F407) */
#define ADC_VREF 3.3f

/** @brief ADC maximum digital value (12-bit) */
#define ADC_MAX_VALUE 4095.0f

/** @brief Minimum magnitude for log10 calculation to avoid -infinity */
#define MIN_MAGNITUDE 1.0e-10f

/** @brief Number of bins around a peak to exclude for noise floor estimation */
#define PEAK_EXCLUSION_BINS 3

/* ========================== Private Function Prototypes ========================== */

static void FFT_GenerateWindow(float32_t *coeffs, uint32_t size,
                               FFT_WindowType_t type);
static void FFT_GenerateHanning(float32_t *coeffs, uint32_t size);
static void FFT_GenerateHamming(float32_t *coeffs, uint32_t size);
static void FFT_GenerateBlackman(float32_t *coeffs, uint32_t size);
static void FFT_SortPeaksByMagnitude(FFT_Peak_t *peaks, uint32_t count);
static bool FFT_IsValidSize(uint32_t size);

/* ========================== Window Type Names ========================== */

/**
 * @brief String names for window types (used for UART output)
 */
static const char *window_names[] = {
    "RECTANGULAR",
    "HANNING",
    "HAMMING",
    "BLACKMAN"};

/* ========================== Public Functions ========================== */

/**
 * @brief  Initialize the FFT analyzer
 *
 * Initialization steps:
 * 1. Validate FFT size (must be power of 2, 32-4096)
 * 2. Initialize CMSIS-DSP arm_rfft_fast_f32 instance
 * 3. Store buffer pointers
 * 4. Generate window function coefficients
 * 5. Compute frequency resolution
 */
arm_status FFT_Analyzer_Init(FFT_Analyzer_t *analyzer,
                             const FFT_Config_t *config,
                             float32_t *fft_input,
                             float32_t *fft_output,
                             float32_t *window_buf,
                             float32_t *mag_buf)
{
    arm_status status;

    /* Validate parameters */
    if (analyzer == NULL || config == NULL ||
        fft_input == NULL || fft_output == NULL ||
        window_buf == NULL || mag_buf == NULL)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    /* Validate FFT size */
    if (!FFT_IsValidSize(config->fft_size))
    {
        return ARM_MATH_LENGTH_ERROR;
    }

    /* Validate window type */
    if (config->window_type >= FFT_WINDOW_COUNT)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    /* Clear the analyzer structure */
    memset(analyzer, 0, sizeof(FFT_Analyzer_t));

    /* Store configuration */
    memcpy(&analyzer->config, config, sizeof(FFT_Config_t));

    /* Store buffer pointers */
    analyzer->fft_input = fft_input;
    analyzer->fft_output = fft_output;
    analyzer->window_coeffs = window_buf;
    analyzer->mag_buffer = mag_buf;

    /* Initialize the CMSIS-DSP RFFT instance for the given size */
    /* arm_rfft_fast_init_f32 supports sizes: 32, 64, 128, 256, 512, 1024, 2048, 4096 */
    status = arm_rfft_fast_init_f32(&analyzer->fft_instance, config->fft_size);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    /* Calculate frequency resolution: df = fs / N */
    analyzer->freq_resolution = config->fs / (float32_t)config->fft_size;

    /* Generate window function coefficients */
    FFT_GenerateWindow(analyzer->window_coeffs, config->fft_size, config->window_type);
    analyzer->window_generated = true;

    analyzer->initialized = true;
    return ARM_MATH_SUCCESS;
}

/**
 * @brief  Run the complete FFT analysis pipeline
 *
 * Processing pipeline:
 *
 * Step 1: Convert ADC uint16 samples to float32
 *         float_val = (adc_val / 4095.0) * 3.3
 *
 * Step 2: Calculate and remove DC offset
 *         DC = mean(signal)
 *         signal[n] -= DC
 *
 * Step 3: Apply window function
 *         signal[n] *= window[n]
 *
 * Step 4: Compute FFT using CMSIS-DSP
 *         arm_rfft_fast_f32 produces interleaved complex output
 *
 * Step 5: Compute magnitude spectrum
 *         |X[k]| = sqrt(Re[k]^2 + Im[k]^2)
 *
 * Step 6: Normalize magnitudes
 *         Divide by N/2 for proper amplitude scaling
 *
 * Step 7: Find peaks in magnitude spectrum
 *
 * Step 8: Calculate THD and SNR
 *
 * Step 9: Convert magnitudes to dB scale
 */
bool FFT_Analyzer_Process(FFT_Analyzer_t *analyzer,
                          const uint16_t *adc_data,
                          uint32_t num_samples,
                          FFT_Result_t *result)
{
    uint32_t i;
    uint32_t half_size;
    float32_t mean_val;
    float32_t max_mag;
    uint32_t max_index;

    /* Validate parameters */
    if (analyzer == NULL || !analyzer->initialized ||
        adc_data == NULL || result == NULL)
    {
        return false;
    }

    if (num_samples != analyzer->config.fft_size)
    {
        return false;
    }

    half_size = analyzer->config.fft_size / 2;

    /* ---- Step 1: Convert ADC uint16 data to float32 ---- */
    /* Scale ADC values (0-4095) to voltage (0-3.3V) */
    for (i = 0; i < analyzer->config.fft_size; i++)
    {
        analyzer->fft_input[i] = ((float32_t)adc_data[i] / ADC_MAX_VALUE) * ADC_VREF;
    }

    /* ---- Step 2: Calculate DC offset and remove it ---- */
    /* Compute the mean value (DC component) */
    arm_mean_f32(analyzer->fft_input, analyzer->config.fft_size, &mean_val);
    result->dc_offset = mean_val;

    /* Subtract the DC offset from all samples */
    /* This prevents a large DC spike in bin 0 that could mask signals */
    for (i = 0; i < analyzer->config.fft_size; i++)
    {
        analyzer->fft_input[i] -= mean_val;
    }

    /* Calculate RMS of the AC-coupled signal */
    arm_rms_f32(analyzer->fft_input, analyzer->config.fft_size, &result->rms_value);

    /* ---- Step 3: Apply window function ---- */
    FFT_Analyzer_ApplyWindow(analyzer, analyzer->fft_input);

    /* ---- Step 4: Compute FFT ---- */
    /* arm_rfft_fast_f32 computes real FFT, output is interleaved complex:
     * [Re(0), Im(0), Re(1), Im(1), ..., Re(N/2-1), Im(N/2-1)]
     * Note: For a real FFT, Im(0) actually contains Re(N/2) (Nyquist bin)
     * The input array is modified (used as scratch space) */
    FFT_Analyzer_ComputeFFT(analyzer, analyzer->fft_input, analyzer->fft_output);

    /* ---- Step 5: Compute magnitude spectrum ---- */
    FFT_Analyzer_ComputeMagnitude(analyzer, analyzer->fft_output, analyzer->mag_buffer);

    /* ---- Step 6: Normalize magnitudes ---- */
    /* Scale by 2/N to get correct amplitude values
     * Factor of 2 because real FFT only computes positive frequencies
     * Factor of 1/N from the DFT definition */
    float32_t scale = 2.0f / (float32_t)analyzer->config.fft_size;
    for (i = 0; i < half_size; i++)
    {
        analyzer->mag_buffer[i] *= scale;
    }
    /* DC bin (index 0) should not be doubled */
    analyzer->mag_buffer[0] *= 0.5f;

    /* Copy magnitude to result if user provided the array */
    if (result->magnitude != NULL)
    {
        memcpy(result->magnitude, analyzer->mag_buffer,
               half_size * sizeof(float32_t));
    }

    /* ---- Step 7: Find the strongest peak ---- */
    /* Skip DC bin (index 0) for peak detection */
    arm_max_f32(&analyzer->mag_buffer[1], half_size - 1, &max_mag, &max_index);
    max_index += 1; /* Adjust for skipped DC bin */

    result->peak_frequency = (float32_t)max_index * analyzer->freq_resolution;
    result->peak_magnitude = max_mag;
    result->peak_magnitude_db = 20.0f * log10f(max_mag > MIN_MAGNITUDE ? max_mag : MIN_MAGNITUDE);

    /* ---- Step 8: Detect multiple peaks ---- */
    result->num_peaks = FFT_Analyzer_FindPeaks(analyzer, analyzer->mag_buffer,
                                               result->peaks, FFT_MAX_PEAKS);

    /* ---- Step 9: Calculate THD ---- */
    result->thd = FFT_Analyzer_ComputeTHD(analyzer, analyzer->mag_buffer, max_index);

    /* ---- Step 10: Calculate SNR ---- */
    result->snr = FFT_Analyzer_ComputeSNR(analyzer, analyzer->mag_buffer, max_index);

    /* ---- Step 11: Convert to dB scale ---- */
    if (result->magnitude_db != NULL)
    {
        FFT_Analyzer_MagnitudeToDb(analyzer->mag_buffer, result->magnitude_db,
                                   half_size, max_mag);
    }

    /* ---- Step 12: Generate frequency axis ---- */
    if (result->frequency != NULL)
    {
        FFT_Analyzer_GenerateFreqAxis(analyzer, result->frequency);
    }

    /* Store metadata in result */
    result->freq_resolution = analyzer->freq_resolution;
    result->fft_size = analyzer->config.fft_size;
    result->fs = analyzer->config.fs;
    result->valid = true;

    return true;
}

/**
 * @brief  Apply the configured window function to a signal
 *
 * Element-wise multiplication: signal[n] = signal[n] * window[n]
 *
 * Uses arm_mult_f32 for SIMD-optimized multiplication when available.
 */
void FFT_Analyzer_ApplyWindow(FFT_Analyzer_t *analyzer, float32_t *signal)
{
    if (analyzer == NULL || signal == NULL || !analyzer->window_generated)
    {
        return;
    }

    /* For rectangular window, coefficients are all 1.0 -- skip multiplication */
    if (analyzer->config.window_type == FFT_WINDOW_RECTANGULAR)
    {
        return;
    }

    /* Element-wise multiply: signal = signal .* window_coeffs */
    arm_mult_f32(signal, analyzer->window_coeffs, signal,
                 analyzer->config.fft_size);
}

/**
 * @brief  Compute the FFT of a real-valued signal
 *
 * Wraps the CMSIS-DSP arm_rfft_fast_f32 function.
 *
 * Input format:  Real values [x0, x1, x2, ..., x(N-1)]
 * Output format: Interleaved complex [Re0, Im0, Re1, Im1, ..., Re(N/2-1), Im(N/2-1)]
 *
 * Special case: output[0] = Re(DC), output[1] = Re(Nyquist)
 * (Both DC and Nyquist components are purely real for real input)
 *
 * @warning The input array is modified during computation (used as scratch)
 */
void FFT_Analyzer_ComputeFFT(FFT_Analyzer_t *analyzer,
                             float32_t *input,
                             float32_t *output)
{
    if (analyzer == NULL || input == NULL || output == NULL)
    {
        return;
    }

    /* ifftFlag = 0 means forward FFT (time -> frequency) */
    arm_rfft_fast_f32(&analyzer->fft_instance, input, output, 0);
}

/**
 * @brief  Compute magnitude spectrum from complex FFT output
 *
 * For the standard complex bins (indices 1 to N/2-1):
 *   |X[k]| = sqrt(Re[k]^2 + Im[k]^2)
 *
 * Special handling for bin 0:
 *   output[0] = DC component (real only)
 *   output[1] = Nyquist component (real only, packed by CMSIS-DSP)
 */
void FFT_Analyzer_ComputeMagnitude(FFT_Analyzer_t *analyzer,
                                   const float32_t *fft_output,
                                   float32_t *magnitude_out)
{
    uint32_t half_size;

    if (analyzer == NULL || fft_output == NULL || magnitude_out == NULL)
    {
        return;
    }

    half_size = analyzer->config.fft_size / 2;

    /* Handle DC and Nyquist bins specially
     * CMSIS-DSP packs them as: output[0] = Re(DC), output[1] = Re(Nyquist) */
    magnitude_out[0] = fabsf(fft_output[0]); /* DC magnitude */

    /* Compute magnitude for bins 1 to N/2-1 using CMSIS-DSP optimized function
     * arm_cmplx_mag_f32 expects interleaved complex: [Re, Im, Re, Im, ...]
     * Starting from index 2 in the FFT output (bin 1 complex data) */
    arm_cmplx_mag_f32(&fft_output[2], &magnitude_out[1], half_size - 1);
}

/**
 * @brief  Find peaks in the magnitude spectrum
 *
 * A peak is defined as a bin where:
 * 1. magnitude[k] > magnitude[k-1]  AND  magnitude[k] > magnitude[k+1]
 * 2. magnitude[k] is above the detection threshold
 *
 * Found peaks are sorted by magnitude in descending order.
 */
uint32_t FFT_Analyzer_FindPeaks(FFT_Analyzer_t *analyzer,
                                const float32_t *magnitude,
                                FFT_Peak_t *peaks_out,
                                uint32_t max_peaks)
{
    uint32_t half_size;
    uint32_t peak_count = 0;
    uint32_t i;
    float32_t mag_db;
    float32_t max_val;
    uint32_t max_idx;

    if (analyzer == NULL || magnitude == NULL || peaks_out == NULL)
    {
        return 0;
    }

    half_size = analyzer->config.fft_size / 2;

    /* Find the overall maximum for dB reference */
    arm_max_f32(magnitude, half_size, &max_val, &max_idx);

    if (max_val < MIN_MAGNITUDE)
    {
        return 0; /* No significant signal */
    }

    /* Scan for local maxima (skip DC bin at index 0 and last bin) */
    for (i = 1; i < half_size - 1 && peak_count < max_peaks; i++)
    {
        /* Check if this bin is a local maximum */
        if (magnitude[i] > magnitude[i - 1] &&
            magnitude[i] > magnitude[i + 1])
        {

            /* Check if magnitude is above threshold */
            mag_db = 20.0f * log10f(magnitude[i] / max_val);

            if (mag_db > FFT_PEAK_THRESHOLD_DB)
            {
                peaks_out[peak_count].bin_index = i;
                peaks_out[peak_count].frequency = (float32_t)i * analyzer->freq_resolution;
                peaks_out[peak_count].magnitude = magnitude[i];
                peaks_out[peak_count].magnitude_db = mag_db;
                peak_count++;
            }
        }
    }

    /* Sort peaks by magnitude (descending) */
    if (peak_count > 1)
    {
        FFT_SortPeaksByMagnitude(peaks_out, peak_count);
    }

    return peak_count;
}

/**
 * @brief  Calculate Total Harmonic Distortion (THD)
 *
 * THD formula:
 *   THD(%) = 100 * sqrt(V2^2 + V3^2 + V4^2 + ... + Vn^2) / V1
 *
 * Where V1 is the fundamental magnitude and V2..Vn are harmonics.
 *
 * The function searches for harmonics at integer multiples of the
 * fundamental frequency (2f, 3f, 4f, ..., up to FFT_MAX_HARMONICS*f).
 * Each harmonic is found by searching for the maximum near the
 * expected bin location (+/- 2 bins tolerance).
 */
float32_t FFT_Analyzer_ComputeTHD(FFT_Analyzer_t *analyzer,
                                  const float32_t *magnitude,
                                  uint32_t fund_bin)
{
    uint32_t half_size;
    float32_t fund_mag;
    float32_t harmonic_sum_sq = 0.0f;
    uint32_t harmonic;
    uint32_t harm_bin;
    uint32_t search_start, search_end;
    float32_t max_harm_mag;
    uint32_t max_harm_idx;
    uint32_t j;

    if (analyzer == NULL || magnitude == NULL)
    {
        return 0.0f;
    }

    half_size = analyzer->config.fft_size / 2;

    /* Get fundamental magnitude */
    fund_mag = magnitude[fund_bin];

    if (fund_mag < MIN_MAGNITUDE)
    {
        return 0.0f; /* No fundamental detected */
    }

    /* Sum the squared magnitudes of harmonics (2nd through Nth) */
    for (harmonic = 2; harmonic <= FFT_MAX_HARMONICS; harmonic++)
    {
        harm_bin = fund_bin * harmonic;

        /* Check if harmonic is within Nyquist range */
        if (harm_bin >= half_size)
        {
            break;
        }

        /* Search for the harmonic peak near the expected bin (+/- 2 bins) */
        search_start = (harm_bin > 2) ? (harm_bin - 2) : 0;
        search_end = (harm_bin + 2 < half_size) ? (harm_bin + 2) : (half_size - 1);

        max_harm_mag = 0.0f;
        max_harm_idx = harm_bin;

        for (j = search_start; j <= search_end; j++)
        {
            if (magnitude[j] > max_harm_mag)
            {
                max_harm_mag = magnitude[j];
                max_harm_idx = j;
            }
        }
        (void)max_harm_idx; /* Used for debugging; suppress unused warning */

        /* Accumulate squared harmonic magnitude */
        harmonic_sum_sq += max_harm_mag * max_harm_mag;
    }

    /* THD(%) = 100 * sqrt(sum of harmonic powers) / fundamental magnitude */
    float32_t thd = 100.0f * sqrtf(harmonic_sum_sq) / fund_mag;

    return thd;
}

/**
 * @brief  Calculate Signal-to-Noise Ratio (SNR)
 *
 * SNR estimation method:
 * 1. Identify signal bins: fundamental + harmonics (+/- PEAK_EXCLUSION_BINS)
 * 2. Sum signal power = sum of squared magnitudes at signal bins
 * 3. Sum noise power = sum of squared magnitudes at all other bins
 * 4. SNR = 10 * log10(signal_power / noise_power)
 */
float32_t FFT_Analyzer_ComputeSNR(FFT_Analyzer_t *analyzer,
                                  const float32_t *magnitude,
                                  uint32_t fund_bin)
{
    uint32_t half_size;
    float32_t signal_power = 0.0f;
    float32_t noise_power = 0.0f;
    uint32_t i, h;
    bool is_signal_bin;

    if (analyzer == NULL || magnitude == NULL)
    {
        return 0.0f;
    }

    half_size = analyzer->config.fft_size / 2;

    /* Calculate power in each bin and classify as signal or noise */
    for (i = 1; i < half_size; i++)
    { /* Skip DC bin */
        float32_t bin_power = magnitude[i] * magnitude[i];

        /* Check if this bin is near the fundamental or any harmonic */
        is_signal_bin = false;
        for (h = 1; h <= FFT_MAX_HARMONICS; h++)
        {
            uint32_t expected_bin = fund_bin * h;
            if (expected_bin >= half_size)
                break;

            /* Check if within exclusion zone around this harmonic */
            if (i >= (expected_bin > PEAK_EXCLUSION_BINS ? expected_bin - PEAK_EXCLUSION_BINS : 0) &&
                i <= expected_bin + PEAK_EXCLUSION_BINS)
            {
                is_signal_bin = true;
                break;
            }
        }

        if (is_signal_bin)
        {
            signal_power += bin_power;
        }
        else
        {
            noise_power += bin_power;
        }
    }

    /* Calculate SNR in dB */
    if (noise_power < MIN_MAGNITUDE)
    {
        return 120.0f; /* Very high SNR (essentially no noise) */
    }

    float32_t snr = 10.0f * log10f(signal_power / noise_power);
    return snr;
}

/**
 * @brief  Convert magnitude values to dB scale
 *
 * Formula: dB[k] = 20 * log10(magnitude[k] / reference)
 *
 * - Values below MIN_MAGNITUDE are clamped to avoid log10(0)
 * - Output is clamped to FFT_MIN_DB (-120 dB) at the lower end
 * - Reference value sets the 0 dB point (typically the peak magnitude)
 */
void FFT_Analyzer_MagnitudeToDb(const float32_t *magnitude,
                                float32_t *magnitude_db,
                                uint32_t length,
                                float32_t reference)
{
    uint32_t i;
    float32_t val;

    if (magnitude == NULL || magnitude_db == NULL || length == 0)
    {
        return;
    }

    /* Prevent division by zero */
    if (reference < MIN_MAGNITUDE)
    {
        reference = MIN_MAGNITUDE;
    }

    for (i = 0; i < length; i++)
    {
        val = magnitude[i];

        /* Clamp to minimum to avoid log10(0) */
        if (val < MIN_MAGNITUDE)
        {
            val = MIN_MAGNITUDE;
        }

        /* 20*log10 for voltage/amplitude quantities */
        magnitude_db[i] = 20.0f * log10f(val / reference);

        /* Clamp to minimum dB value */
        if (magnitude_db[i] < FFT_MIN_DB)
        {
            magnitude_db[i] = FFT_MIN_DB;
        }
    }
}

/**
 * @brief  Generate frequency axis values
 *
 * Fills the output array with the center frequency of each FFT bin:
 *   freq[k] = k * (fs / N)   for k = 0, 1, ..., N/2 - 1
 *
 * Example (fs=10kHz, N=1024):
 *   freq[0]   = 0 Hz      (DC)
 *   freq[1]   = 9.77 Hz
 *   freq[511] = 4995.12 Hz (near Nyquist)
 */
void FFT_Analyzer_GenerateFreqAxis(FFT_Analyzer_t *analyzer,
                                   float32_t *freq_out)
{
    uint32_t i;
    uint32_t half_size;

    if (analyzer == NULL || freq_out == NULL)
    {
        return;
    }

    half_size = analyzer->config.fft_size / 2;

    for (i = 0; i < half_size; i++)
    {
        freq_out[i] = (float32_t)i * analyzer->freq_resolution;
    }
}

/**
 * @brief  Change the window function type at runtime
 *
 * Regenerates the window coefficient array for the new window type.
 * The change takes effect on the next call to FFT_Analyzer_Process().
 */
bool FFT_Analyzer_SetWindowType(FFT_Analyzer_t *analyzer,
                                FFT_WindowType_t window_type)
{
    if (analyzer == NULL || !analyzer->initialized)
    {
        return false;
    }

    if (window_type >= FFT_WINDOW_COUNT)
    {
        return false;
    }

    analyzer->config.window_type = window_type;
    FFT_GenerateWindow(analyzer->window_coeffs, analyzer->config.fft_size,
                       window_type);
    analyzer->window_generated = true;

    return true;
}

/**
 * @brief  Change the FFT size at runtime
 *
 * This reinitializes the CMSIS-DSP FFT instance and regenerates window
 * coefficients. The caller must ensure that all buffers (fft_input, fft_output,
 * window_coeffs, mag_buffer) are large enough for the new size.
 *
 * Supported sizes: 256, 512, 1024, 2048, 4096
 */
arm_status FFT_Analyzer_SetSize(FFT_Analyzer_t *analyzer, uint32_t new_size)
{
    arm_status status;

    if (analyzer == NULL || !analyzer->initialized)
    {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    if (!FFT_IsValidSize(new_size))
    {
        return ARM_MATH_LENGTH_ERROR;
    }

    /* Reinitialize the CMSIS-DSP FFT instance */
    status = arm_rfft_fast_init_f32(&analyzer->fft_instance, new_size);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }

    /* Update configuration */
    analyzer->config.fft_size = new_size;
    analyzer->freq_resolution = analyzer->config.fs / (float32_t)new_size;

    /* Regenerate window coefficients for new size */
    FFT_GenerateWindow(analyzer->window_coeffs, new_size,
                       analyzer->config.window_type);

    return ARM_MATH_SUCCESS;
}

/**
 * @brief  Get the name string for a window type
 */
const char *FFT_Analyzer_GetWindowName(FFT_WindowType_t type)
{
    if (type >= FFT_WINDOW_COUNT)
    {
        return "UNKNOWN";
    }
    return window_names[type];
}

/* ========================== Private Functions ========================== */

/**
 * @brief  Generate window function coefficients
 * @details Dispatches to the appropriate window generation function.
 *          For rectangular window, all coefficients are set to 1.0.
 */
static void FFT_GenerateWindow(float32_t *coeffs, uint32_t size,
                               FFT_WindowType_t type)
{
    uint32_t i;

    switch (type)
    {
    case FFT_WINDOW_HANNING:
        FFT_GenerateHanning(coeffs, size);
        break;

    case FFT_WINDOW_HAMMING:
        FFT_GenerateHamming(coeffs, size);
        break;

    case FFT_WINDOW_BLACKMAN:
        FFT_GenerateBlackman(coeffs, size);
        break;

    case FFT_WINDOW_RECTANGULAR:
    default:
        /* Rectangular window: all coefficients = 1.0 */
        for (i = 0; i < size; i++)
        {
            coeffs[i] = 1.0f;
        }
        break;
    }
}

/**
 * @brief  Generate Hanning (Hann) window coefficients
 *
 * Formula: w[n] = 0.5 * (1 - cos(2*pi*n / (N-1)))
 *
 * Properties:
 * - First and last samples are zero (exact zero endpoints)
 * - Main lobe width: 4 * fs/N
 * - First sidelobe: -31 dB
 * - Sidelobe rolloff: -18 dB/octave
 * - Good general-purpose window for spectral analysis
 */
static void FFT_GenerateHanning(float32_t *coeffs, uint32_t size)
{
    uint32_t i;
    float32_t n_minus_1 = (float32_t)(size - 1);

    for (i = 0; i < size; i++)
    {
        coeffs[i] = 0.5f * (1.0f - cosf(TWO_PI * (float32_t)i / n_minus_1));
    }
}

/**
 * @brief  Generate Hamming window coefficients
 *
 * Formula: w[n] = 0.54 - 0.46 * cos(2*pi*n / (N-1))
 *
 * Properties:
 * - First and last samples are 0.08 (non-zero endpoints)
 * - Main lobe width: 4 * fs/N
 * - First sidelobe: -42 dB (better than Hanning)
 * - Sidelobe rolloff: -6 dB/octave (slower than Hanning)
 * - Good for frequency measurement applications
 */
static void FFT_GenerateHamming(float32_t *coeffs, uint32_t size)
{
    uint32_t i;
    float32_t n_minus_1 = (float32_t)(size - 1);

    for (i = 0; i < size; i++)
    {
        coeffs[i] = 0.54f - 0.46f * cosf(TWO_PI * (float32_t)i / n_minus_1);
    }
}

/**
 * @brief  Generate Blackman window coefficients
 *
 * Formula: w[n] = 0.42 - 0.5*cos(2*pi*n/(N-1)) + 0.08*cos(4*pi*n/(N-1))
 *
 * Properties:
 * - Main lobe width: 6 * fs/N (wider than Hanning/Hamming)
 * - First sidelobe: -58 dB (excellent)
 * - Best sidelobe rejection among common windows
 * - Ideal for detecting low-level signals near strong ones
 * - Good for harmonic analysis and THD measurement
 */
static void FFT_GenerateBlackman(float32_t *coeffs, uint32_t size)
{
    uint32_t i;
    float32_t n_minus_1 = (float32_t)(size - 1);

    for (i = 0; i < size; i++)
    {
        float32_t phase = TWO_PI * (float32_t)i / n_minus_1;
        coeffs[i] = 0.42f - 0.5f * cosf(phase) + 0.08f * cosf(2.0f * phase);
    }
}

/**
 * @brief  Sort peaks by magnitude in descending order (strongest first)
 * @details Uses simple insertion sort - efficient for small arrays (max 10 peaks)
 */
static void FFT_SortPeaksByMagnitude(FFT_Peak_t *peaks, uint32_t count)
{
    uint32_t i, j;
    FFT_Peak_t temp;

    for (i = 1; i < count; i++)
    {
        temp = peaks[i];
        j = i;
        while (j > 0 && peaks[j - 1].magnitude < temp.magnitude)
        {
            peaks[j] = peaks[j - 1];
            j--;
        }
        peaks[j] = temp;
    }
}

/**
 * @brief  Validate that an FFT size is supported
 * @retval true if size is a valid power of 2 in range [256, 4096]
 */
static bool FFT_IsValidSize(uint32_t size)
{
    return (size == FFT_SIZE_256 ||
            size == FFT_SIZE_512 ||
            size == FFT_SIZE_1024 ||
            size == FFT_SIZE_2048 ||
            size == FFT_SIZE_4096);
}
