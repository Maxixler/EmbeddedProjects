/**
 * @file    mpu6050.c
 * @brief   MPU6050 6-Axis IMU Sensor Driver Implementation
 * @author  STM32 Embedded Systems Portfolio
 * @version 2.0
 *
 * @details Full-featured driver for the MPU6050 accelerometer/gyroscope sensor.
 *          Uses STM32 HAL I2C memory read/write functions for register access.
 *          Implements 14-byte burst reads for efficient, time-coherent data
 *          acquisition, automatic endian conversion, calibration, and self-test.
 */

/* ========================================================================== */
/*                              INCLUDES                                      */
/* ========================================================================== */

#include "mpu6050.h"
#include <string.h>
#include <math.h>

/* ========================================================================== */
/*                          PRIVATE CONSTANTS                                 */
/* ========================================================================== */

/**
 * @brief Accelerometer sensitivity scale factors in LSB/g
 *
 * Index corresponds to MPU6050_AccelRange_t enum values.
 */
static const float ACCEL_SENSITIVITY[4] = {
    16384.0f, /* +/- 2g  */
    8192.0f,  /* +/- 4g  */
    4096.0f,  /* +/- 8g  */
    2048.0f   /* +/- 16g */
};

/**
 * @brief Gyroscope sensitivity scale factors in LSB/(deg/s)
 *
 * Index corresponds to MPU6050_GyroRange_t enum values.
 */
static const float GYRO_SENSITIVITY[4] = {
    131.0f, /* +/- 250  dps */
    65.5f,  /* +/- 500  dps */
    32.8f,  /* +/- 1000 dps */
    16.4f   /* +/- 2000 dps */
};

/**
 * @brief Factory trim values for self-test calculation
 *
 * From the MPU6050 register map document. These are used to compute
 * the expected self-test response for comparison.
 */
static const float SELF_TEST_ACCEL_FT[32] = {
    0.0f, 1392.64f, 1404.352f, 1416.192f, 1428.16f, 1440.256f,
    1452.48f, 1464.832f, 1477.312f, 1489.92f, 1502.656f, 1515.52f,
    1528.512f, 1541.632f, 1554.88f, 1568.256f, 1581.76f, 1595.392f,
    1609.152f, 1623.04f, 1637.056f, 1651.2f, 1665.472f, 1679.872f,
    1694.4f, 1709.056f, 1723.84f, 1738.752f, 1753.792f, 1768.96f,
    1784.256f, 1799.68f};

/** @brief Burst read buffer size: 3*accel + temp + 3*gyro = 14 bytes */
#define MPU6050_BURST_READ_SIZE 14

/* ========================================================================== */
/*                       PRIVATE HELPER FUNCTIONS                             */
/* ========================================================================== */

/**
 * @brief  Write a single byte to an MPU6050 register
 * @param  hmpu     Pointer to MPU6050 handle
 * @param  reg      Register address
 * @param  data     Byte to write
 * @retval MPU6050_Status_t
 */
static MPU6050_Status_t MPU6050_WriteReg(MPU6050_Handle_t *hmpu,
                                         uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_I2C_Mem_Write(hmpu->hi2c,
                                   hmpu->dev_addr,
                                   reg,
                                   I2C_MEMADD_SIZE_8BIT,
                                   &data,
                                   1,
                                   MPU6050_I2C_TIMEOUT);

    if (hal_status != HAL_OK)
    {
        return (hal_status == HAL_TIMEOUT) ? MPU6050_TIMEOUT : MPU6050_ERROR;
    }

    return MPU6050_OK;
}

/**
 * @brief  Read one or more bytes from consecutive MPU6050 registers
 * @param  hmpu     Pointer to MPU6050 handle
 * @param  reg      Starting register address
 * @param  data     Pointer to receive buffer
 * @param  length   Number of bytes to read
 * @retval MPU6050_Status_t
 */
static MPU6050_Status_t MPU6050_ReadRegs(MPU6050_Handle_t *hmpu,
                                         uint8_t reg,
                                         uint8_t *data,
                                         uint16_t length)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_I2C_Mem_Read(hmpu->hi2c,
                                  hmpu->dev_addr,
                                  reg,
                                  I2C_MEMADD_SIZE_8BIT,
                                  data,
                                  length,
                                  MPU6050_I2C_TIMEOUT);

    if (hal_status != HAL_OK)
    {
        return (hal_status == HAL_TIMEOUT) ? MPU6050_TIMEOUT : MPU6050_ERROR;
    }

    return MPU6050_OK;
}

/**
 * @brief  Combine two bytes into a signed 16-bit value (big-endian)
 *
 * MPU6050 transmits data in big-endian format (MSB first).
 *
 * @param  msb  Most significant byte
 * @param  lsb  Least significant byte
 * @retval Signed 16-bit value
 */
static inline int16_t MPU6050_CombineBytes(uint8_t msb, uint8_t lsb)
{
    return (int16_t)((uint16_t)msb << 8 | (uint16_t)lsb);
}

/* ========================================================================== */
/*                    INITIALIZATION AND CONFIGURATION                        */
/* ========================================================================== */

MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu,
                              I2C_HandleTypeDef *hi2c,
                              uint16_t dev_addr,
                              MPU6050_AccelRange_t accel_range,
                              MPU6050_GyroRange_t gyro_range,
                              MPU6050_DLPF_t dlpf,
                              uint8_t sample_div)
{
    MPU6050_Status_t status;
    uint8_t who_am_i;

    /* Validate parameters */
    if (hmpu == NULL || hi2c == NULL)
    {
        return MPU6050_INVALID_PARAM;
    }

    /* Store configuration in handle */
    hmpu->hi2c = hi2c;
    hmpu->dev_addr = dev_addr;
    hmpu->initialized = false;

    /* Clear calibration data */
    memset(&hmpu->calibration, 0, sizeof(MPU6050_Calibration_t));

    /*
     * Step 1: Verify device identity
     * Read WHO_AM_I register to confirm the MPU6050 is present and responding.
     * The expected value is 0x68 regardless of the AD0 pin state.
     */
    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (status != MPU6050_OK)
    {
        return MPU6050_NOT_FOUND;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE)
    {
        return MPU6050_NOT_FOUND;
    }

    /*
     * Step 2: Reset the device
     * Setting the DEVICE_RESET bit resets all internal registers to their
     * default values. We must wait for the reset to complete.
     */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1,
                              MPU6050_PWR1_DEVICE_RESET);
    if (status != MPU6050_OK)
    {
        return status;
    }

    HAL_Delay(MPU6050_RESET_DELAY);

    /*
     * Step 3: Wake up from sleep mode and select clock source
     * After reset, the device enters sleep mode by default.
     * We clear the SLEEP bit and set CLKSEL to PLL with X-axis gyro
     * reference, which provides better stability than the internal RC.
     */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1,
                              MPU6050_CLKSEL_PLL_XGYRO);
    if (status != MPU6050_OK)
    {
        return status;
    }

    HAL_Delay(10); /* Allow PLL to stabilize */

    /*
     * Step 4: Configure the Digital Low-Pass Filter
     * This must be set before the sample rate divider, as the DLPF
     * configuration affects the base sample rate.
     */
    status = MPU6050_SetDLPF(hmpu, dlpf);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /*
     * Step 5: Set sample rate divider
     * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
     * With DLPF enabled: Gyroscope Output Rate = 1 kHz
     */
    status = MPU6050_SetSampleRate(hmpu, sample_div);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /*
     * Step 6: Configure accelerometer full-scale range
     */
    status = MPU6050_SetAccelRange(hmpu, accel_range);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /*
     * Step 7: Configure gyroscope full-scale range
     */
    status = MPU6050_SetGyroRange(hmpu, gyro_range);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /*
     * Step 8: Configure interrupt pin (optional but useful for data ready)
     * Set INT pin to active high, push-pull, 50us pulse, clear on any read
     */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_INT_PIN_CFG,
                              MPU6050_INT_RD_CLEAR);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /*
     * Step 9: Enable data ready interrupt
     */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_INT_ENABLE,
                              MPU6050_INT_DATA_RDY);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /* Mark as successfully initialized */
    hmpu->initialized = true;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetAccelRange(MPU6050_Handle_t *hmpu,
                                       MPU6050_AccelRange_t range)
{
    MPU6050_Status_t status;

    if (range > MPU6050_ACCEL_RANGE_16G)
    {
        return MPU6050_INVALID_PARAM;
    }

    /*
     * Write the full-scale selection to bits [4:3] of ACCEL_CONFIG.
     * Other bits (self-test and high-pass filter) are cleared.
     */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG,
                              (uint8_t)(range << 3));
    if (status != MPU6050_OK)
    {
        return status;
    }

    hmpu->accel_range = range;
    hmpu->accel_scale = ACCEL_SENSITIVITY[range];

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetGyroRange(MPU6050_Handle_t *hmpu,
                                      MPU6050_GyroRange_t range)
{
    MPU6050_Status_t status;

    if (range > MPU6050_GYRO_RANGE_2000DPS)
    {
        return MPU6050_INVALID_PARAM;
    }

    /*
     * Write the full-scale selection to bits [4:3] of GYRO_CONFIG.
     * Other bits (self-test) are cleared.
     */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG,
                              (uint8_t)(range << 3));
    if (status != MPU6050_OK)
    {
        return status;
    }

    hmpu->gyro_range = range;
    hmpu->gyro_scale = GYRO_SENSITIVITY[range];

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetDLPF(MPU6050_Handle_t *hmpu, MPU6050_DLPF_t dlpf)
{
    MPU6050_Status_t status;

    if (dlpf > MPU6050_DLPF_5HZ)
    {
        return MPU6050_INVALID_PARAM;
    }

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_CONFIG, (uint8_t)dlpf);
    if (status != MPU6050_OK)
    {
        return status;
    }

    hmpu->dlpf_cfg = dlpf;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_SetSampleRate(MPU6050_Handle_t *hmpu, uint8_t divider)
{
    MPU6050_Status_t status;

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_SMPLRT_DIV, divider);
    if (status != MPU6050_OK)
    {
        return status;
    }

    hmpu->sample_div = divider;

    return MPU6050_OK;
}

/* ========================================================================== */
/*                          DATA READING FUNCTIONS                            */
/* ========================================================================== */

MPU6050_Status_t MPU6050_ReadAccel(MPU6050_Handle_t *hmpu)
{
    MPU6050_Status_t status;
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    /*
     * Read 6 bytes starting from ACCEL_XOUT_H (0x3B).
     * MPU6050 supports auto-increment, so bytes are read in order:
     *   [XOUT_H, XOUT_L, YOUT_H, YOUT_L, ZOUT_H, ZOUT_L]
     */
    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H, buf, 6);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /* Combine MSB and LSB (big-endian to native) */
    raw_x = MPU6050_CombineBytes(buf[0], buf[1]);
    raw_y = MPU6050_CombineBytes(buf[2], buf[3]);
    raw_z = MPU6050_CombineBytes(buf[4], buf[5]);

    /* Store raw values */
    hmpu->raw_accel.x = raw_x;
    hmpu->raw_accel.y = raw_y;
    hmpu->raw_accel.z = raw_z;

    /* Apply calibration offsets if available */
    if (hmpu->calibration.is_valid)
    {
        raw_x -= (int16_t)hmpu->calibration.accel_x;
        raw_y -= (int16_t)hmpu->calibration.accel_y;
        raw_z -= (int16_t)hmpu->calibration.accel_z;
    }

    /* Convert to g using the sensitivity scale factor */
    hmpu->accel.x = (float)raw_x / hmpu->accel_scale;
    hmpu->accel.y = (float)raw_y / hmpu->accel_scale;
    hmpu->accel.z = (float)raw_z / hmpu->accel_scale;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadGyro(MPU6050_Handle_t *hmpu)
{
    MPU6050_Status_t status;
    uint8_t buf[6];
    int16_t raw_x, raw_y, raw_z;

    /*
     * Read 6 bytes starting from GYRO_XOUT_H (0x43).
     *   [XOUT_H, XOUT_L, YOUT_H, YOUT_L, ZOUT_H, ZOUT_L]
     */
    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_GYRO_XOUT_H, buf, 6);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /* Combine MSB and LSB */
    raw_x = MPU6050_CombineBytes(buf[0], buf[1]);
    raw_y = MPU6050_CombineBytes(buf[2], buf[3]);
    raw_z = MPU6050_CombineBytes(buf[4], buf[5]);

    /* Store raw values */
    hmpu->raw_gyro.x = raw_x;
    hmpu->raw_gyro.y = raw_y;
    hmpu->raw_gyro.z = raw_z;

    /* Apply calibration offsets */
    if (hmpu->calibration.is_valid)
    {
        raw_x -= (int16_t)hmpu->calibration.gyro_x;
        raw_y -= (int16_t)hmpu->calibration.gyro_y;
        raw_z -= (int16_t)hmpu->calibration.gyro_z;
    }

    /* Convert to degrees per second */
    hmpu->gyro.x = (float)raw_x / hmpu->gyro_scale;
    hmpu->gyro.y = (float)raw_y / hmpu->gyro_scale;
    hmpu->gyro.z = (float)raw_z / hmpu->gyro_scale;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadTemperature(MPU6050_Handle_t *hmpu)
{
    MPU6050_Status_t status;
    uint8_t buf[2];

    /* Read 2 bytes from TEMP_OUT_H (0x41) */
    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_TEMP_OUT_H, buf, 2);
    if (status != MPU6050_OK)
    {
        return status;
    }

    hmpu->raw_temp = MPU6050_CombineBytes(buf[0], buf[1]);

    /*
     * Temperature conversion formula from the datasheet:
     *   Temperature (deg C) = TEMP_OUT / 340.0 + 36.53
     */
    hmpu->temperature = (float)hmpu->raw_temp / 340.0f + 36.53f;

    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadAll(MPU6050_Handle_t *hmpu)
{
    MPU6050_Status_t status;
    uint8_t buf[MPU6050_BURST_READ_SIZE];
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;

    /*
     * Burst read 14 bytes starting from ACCEL_XOUT_H (0x3B).
     *
     * This reads all sensor data in a single I2C transaction:
     *   Bytes  0- 1: ACCEL_XOUT_H, ACCEL_XOUT_L
     *   Bytes  2- 3: ACCEL_YOUT_H, ACCEL_YOUT_L
     *   Bytes  4- 5: ACCEL_ZOUT_H, ACCEL_ZOUT_L
     *   Bytes  6- 7: TEMP_OUT_H,   TEMP_OUT_L
     *   Bytes  8- 9: GYRO_XOUT_H,  GYRO_XOUT_L
     *   Bytes 10-11: GYRO_YOUT_H,  GYRO_YOUT_L
     *   Bytes 12-13: GYRO_ZOUT_H,  GYRO_ZOUT_L
     *
     * Advantages of burst read:
     *   - Minimal I2C overhead (single start/stop)
     *   - All data sampled at nearly the same instant
     *   - Ensures temporal coherence between accel and gyro
     */
    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H,
                              buf, MPU6050_BURST_READ_SIZE);
    if (status != MPU6050_OK)
    {
        return status;
    }

    /* --- Parse accelerometer data (bytes 0-5) --- */
    raw_ax = MPU6050_CombineBytes(buf[0], buf[1]);
    raw_ay = MPU6050_CombineBytes(buf[2], buf[3]);
    raw_az = MPU6050_CombineBytes(buf[4], buf[5]);

    hmpu->raw_accel.x = raw_ax;
    hmpu->raw_accel.y = raw_ay;
    hmpu->raw_accel.z = raw_az;

    /* --- Parse temperature data (bytes 6-7) --- */
    hmpu->raw_temp = MPU6050_CombineBytes(buf[6], buf[7]);
    hmpu->temperature = (float)hmpu->raw_temp / 340.0f + 36.53f;

    /* --- Parse gyroscope data (bytes 8-13) --- */
    raw_gx = MPU6050_CombineBytes(buf[8], buf[9]);
    raw_gy = MPU6050_CombineBytes(buf[10], buf[11]);
    raw_gz = MPU6050_CombineBytes(buf[12], buf[13]);

    hmpu->raw_gyro.x = raw_gx;
    hmpu->raw_gyro.y = raw_gy;
    hmpu->raw_gyro.z = raw_gz;

    /* --- Apply calibration offsets --- */
    if (hmpu->calibration.is_valid)
    {
        raw_ax -= (int16_t)hmpu->calibration.accel_x;
        raw_ay -= (int16_t)hmpu->calibration.accel_y;
        raw_az -= (int16_t)hmpu->calibration.accel_z;
        raw_gx -= (int16_t)hmpu->calibration.gyro_x;
        raw_gy -= (int16_t)hmpu->calibration.gyro_y;
        raw_gz -= (int16_t)hmpu->calibration.gyro_z;
    }

    /* --- Scale accelerometer to g --- */
    hmpu->accel.x = (float)raw_ax / hmpu->accel_scale;
    hmpu->accel.y = (float)raw_ay / hmpu->accel_scale;
    hmpu->accel.z = (float)raw_az / hmpu->accel_scale;

    /* --- Scale gyroscope to degrees per second --- */
    hmpu->gyro.x = (float)raw_gx / hmpu->gyro_scale;
    hmpu->gyro.y = (float)raw_gy / hmpu->gyro_scale;
    hmpu->gyro.z = (float)raw_gz / hmpu->gyro_scale;

    return MPU6050_OK;
}

/* ========================================================================== */
/*                         CALIBRATION FUNCTIONS                              */
/* ========================================================================== */

MPU6050_Status_t MPU6050_Calibrate(MPU6050_Handle_t *hmpu)
{
    uint8_t buf[MPU6050_BURST_READ_SIZE];
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint32_t valid_samples = 0;

    if (!hmpu->initialized)
    {
        return MPU6050_ERROR;
    }

    /*
     * Calibration procedure:
     *
     * The sensor must be stationary on a flat surface with Z-axis pointing up.
     * We collect many samples and compute the average offset for each axis.
     *
     * Expected values when stationary and level:
     *   Accelerometer: X=0, Y=0, Z=+1g (16384 at +/-2g range)
     *   Gyroscope:     X=0, Y=0, Z=0
     *
     * The offsets are computed as:
     *   offset = measured_average - expected_value
     */

    /* Invalidate current calibration during the process */
    hmpu->calibration.is_valid = false;

    /* Discard initial samples to allow sensor to settle */
    for (uint16_t i = 0; i < 100; i++)
    {
        MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H,
                         buf, MPU6050_BURST_READ_SIZE);
        HAL_Delay(MPU6050_CALIBRATION_DELAY);
    }

    /* Collect calibration samples */
    for (uint16_t i = 0; i < MPU6050_CALIBRATION_SAMPLES; i++)
    {
        MPU6050_Status_t status;

        status = MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H,
                                  buf, MPU6050_BURST_READ_SIZE);

        if (status == MPU6050_OK)
        {
            /* Accumulate accelerometer values */
            sum_ax += (int32_t)MPU6050_CombineBytes(buf[0], buf[1]);
            sum_ay += (int32_t)MPU6050_CombineBytes(buf[2], buf[3]);
            sum_az += (int32_t)MPU6050_CombineBytes(buf[4], buf[5]);

            /* Accumulate gyroscope values */
            sum_gx += (int32_t)MPU6050_CombineBytes(buf[8], buf[9]);
            sum_gy += (int32_t)MPU6050_CombineBytes(buf[10], buf[11]);
            sum_gz += (int32_t)MPU6050_CombineBytes(buf[12], buf[13]);

            valid_samples++;
        }

        HAL_Delay(MPU6050_CALIBRATION_DELAY);
    }

    /* Ensure we got enough valid samples */
    if (valid_samples < (MPU6050_CALIBRATION_SAMPLES / 2))
    {
        return MPU6050_ERROR;
    }

    /*
     * Compute average offsets.
     *
     * For accelerometer:
     *   X and Y offsets = average (expecting 0 when level)
     *   Z offset = average - 1g_raw_value (expecting +1g when level)
     *
     * For gyroscope:
     *   All offsets = average (expecting 0 when stationary)
     */
    hmpu->calibration.accel_x = sum_ax / (int32_t)valid_samples;
    hmpu->calibration.accel_y = sum_ay / (int32_t)valid_samples;
    hmpu->calibration.accel_z = sum_az / (int32_t)valid_samples - (int32_t)hmpu->accel_scale; /* Subtract 1g */

    hmpu->calibration.gyro_x = sum_gx / (int32_t)valid_samples;
    hmpu->calibration.gyro_y = sum_gy / (int32_t)valid_samples;
    hmpu->calibration.gyro_z = sum_gz / (int32_t)valid_samples;

    /* Mark calibration as valid */
    hmpu->calibration.is_valid = true;

    return MPU6050_OK;
}

void MPU6050_ResetCalibration(MPU6050_Handle_t *hmpu)
{
    memset(&hmpu->calibration, 0, sizeof(MPU6050_Calibration_t));
    hmpu->calibration.is_valid = false;
}

/* ========================================================================== */
/*                          SELF-TEST FUNCTION                                */
/* ========================================================================== */

MPU6050_Status_t MPU6050_SelfTest(MPU6050_Handle_t *hmpu,
                                  MPU6050_SelfTestResult_t *result)
{
    MPU6050_Status_t status;
    uint8_t self_test_regs[4];
    uint8_t buf[6];

    int16_t accel_st_on[3], accel_st_off[3];
    int16_t gyro_st_on[3], gyro_st_off[3];
    float accel_st_response[3], gyro_st_response[3];
    float factory_trim;
    uint8_t test_val;

    if (result == NULL)
    {
        return MPU6050_INVALID_PARAM;
    }

    /*
     * Self-test procedure (from MPU6050 register map document):
     *
     * 1. Read sensor data with self-test disabled (normal operation)
     * 2. Enable self-test for all axes
     * 3. Read sensor data with self-test enabled
     * 4. Self-test response = ST_enabled - ST_disabled
     * 5. Compare response against factory trim values
     * 6. If deviation < 14%, the axis passes
     */

    /* --- Step 1: Read with self-test disabled --- */

    /* Ensure self-test bits are cleared */
    status = MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG,
                              (uint8_t)(hmpu->accel_range << 3));
    if (status != MPU6050_OK)
        return status;

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG,
                              (uint8_t)(hmpu->gyro_range << 3));
    if (status != MPU6050_OK)
        return status;

    HAL_Delay(50);

    /* Average multiple readings for stability */
    int32_t sum[6] = {0};
    for (int i = 0; i < 100; i++)
    {
        status = MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H, buf, 6);
        if (status != MPU6050_OK)
            return status;
        sum[0] += MPU6050_CombineBytes(buf[0], buf[1]);
        sum[1] += MPU6050_CombineBytes(buf[2], buf[3]);
        sum[2] += MPU6050_CombineBytes(buf[4], buf[5]);

        status = MPU6050_ReadRegs(hmpu, MPU6050_REG_GYRO_XOUT_H, buf, 6);
        if (status != MPU6050_OK)
            return status;
        sum[3] += MPU6050_CombineBytes(buf[0], buf[1]);
        sum[4] += MPU6050_CombineBytes(buf[2], buf[3]);
        sum[5] += MPU6050_CombineBytes(buf[4], buf[5]);

        HAL_Delay(2);
    }

    accel_st_off[0] = (int16_t)(sum[0] / 100);
    accel_st_off[1] = (int16_t)(sum[1] / 100);
    accel_st_off[2] = (int16_t)(sum[2] / 100);
    gyro_st_off[0] = (int16_t)(sum[3] / 100);
    gyro_st_off[1] = (int16_t)(sum[4] / 100);
    gyro_st_off[2] = (int16_t)(sum[5] / 100);

    /* --- Step 2: Enable self-test for all axes --- */

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG,
                              MPU6050_ACCEL_SELF_TEST_X |
                                  MPU6050_ACCEL_SELF_TEST_Y |
                                  MPU6050_ACCEL_SELF_TEST_Z |
                                  (uint8_t)(hmpu->accel_range << 3));
    if (status != MPU6050_OK)
        return status;

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG,
                              MPU6050_GYRO_SELF_TEST_X |
                                  MPU6050_GYRO_SELF_TEST_Y |
                                  MPU6050_GYRO_SELF_TEST_Z |
                                  (uint8_t)(hmpu->gyro_range << 3));
    if (status != MPU6050_OK)
        return status;

    HAL_Delay(100); /* Allow self-test actuation to stabilize */

    /* --- Step 3: Read with self-test enabled --- */

    memset(sum, 0, sizeof(sum));
    for (int i = 0; i < 100; i++)
    {
        status = MPU6050_ReadRegs(hmpu, MPU6050_REG_ACCEL_XOUT_H, buf, 6);
        if (status != MPU6050_OK)
            return status;
        sum[0] += MPU6050_CombineBytes(buf[0], buf[1]);
        sum[1] += MPU6050_CombineBytes(buf[2], buf[3]);
        sum[2] += MPU6050_CombineBytes(buf[4], buf[5]);

        status = MPU6050_ReadRegs(hmpu, MPU6050_REG_GYRO_XOUT_H, buf, 6);
        if (status != MPU6050_OK)
            return status;
        sum[3] += MPU6050_CombineBytes(buf[0], buf[1]);
        sum[4] += MPU6050_CombineBytes(buf[2], buf[3]);
        sum[5] += MPU6050_CombineBytes(buf[4], buf[5]);

        HAL_Delay(2);
    }

    accel_st_on[0] = (int16_t)(sum[0] / 100);
    accel_st_on[1] = (int16_t)(sum[1] / 100);
    accel_st_on[2] = (int16_t)(sum[2] / 100);
    gyro_st_on[0] = (int16_t)(sum[3] / 100);
    gyro_st_on[1] = (int16_t)(sum[4] / 100);
    gyro_st_on[2] = (int16_t)(sum[5] / 100);

    /* --- Step 4: Disable self-test (restore normal configuration) --- */

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_ACCEL_CONFIG,
                              (uint8_t)(hmpu->accel_range << 3));
    if (status != MPU6050_OK)
        return status;

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_GYRO_CONFIG,
                              (uint8_t)(hmpu->gyro_range << 3));
    if (status != MPU6050_OK)
        return status;

    /* --- Step 5: Calculate self-test response --- */

    accel_st_response[0] = (float)(accel_st_on[0] - accel_st_off[0]);
    accel_st_response[1] = (float)(accel_st_on[1] - accel_st_off[1]);
    accel_st_response[2] = (float)(accel_st_on[2] - accel_st_off[2]);

    gyro_st_response[0] = (float)(gyro_st_on[0] - gyro_st_off[0]);
    gyro_st_response[1] = (float)(gyro_st_on[1] - gyro_st_off[1]);
    gyro_st_response[2] = (float)(gyro_st_on[2] - gyro_st_off[2]);

    /* --- Step 6: Read factory trim values --- */

    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_SELF_TEST_X,
                              self_test_regs, 4);
    if (status != MPU6050_OK)
        return status;

    /*
     * Calculate factory trim for each accelerometer axis.
     * The formula uses the self-test register values and the lookup table
     * from the register map document.
     */
    result->passed = true;

    /* Accelerometer X */
    test_val = ((self_test_regs[0] >> 3) & 0x1C) |
               ((self_test_regs[3] >> 4) & 0x03);
    factory_trim = (test_val != 0) ? SELF_TEST_ACCEL_FT[test_val] : 0.0f;
    if (factory_trim != 0.0f)
    {
        result->accel_x_deviation = 100.0f *
                                    (accel_st_response[0] - factory_trim) / factory_trim;
    }
    else
    {
        result->accel_x_deviation = 0.0f;
    }
    if (fabsf(result->accel_x_deviation) > MPU6050_SELF_TEST_TOLERANCE)
    {
        result->passed = false;
    }

    /* Accelerometer Y */
    test_val = ((self_test_regs[1] >> 3) & 0x1C) |
               ((self_test_regs[3] >> 2) & 0x03);
    factory_trim = (test_val != 0) ? SELF_TEST_ACCEL_FT[test_val] : 0.0f;
    if (factory_trim != 0.0f)
    {
        result->accel_y_deviation = 100.0f *
                                    (accel_st_response[1] - factory_trim) / factory_trim;
    }
    else
    {
        result->accel_y_deviation = 0.0f;
    }
    if (fabsf(result->accel_y_deviation) > MPU6050_SELF_TEST_TOLERANCE)
    {
        result->passed = false;
    }

    /* Accelerometer Z */
    test_val = ((self_test_regs[2] >> 3) & 0x1C) |
               (self_test_regs[3] & 0x03);
    factory_trim = (test_val != 0) ? SELF_TEST_ACCEL_FT[test_val] : 0.0f;
    if (factory_trim != 0.0f)
    {
        result->accel_z_deviation = 100.0f *
                                    (accel_st_response[2] - factory_trim) / factory_trim;
    }
    else
    {
        result->accel_z_deviation = 0.0f;
    }
    if (fabsf(result->accel_z_deviation) > MPU6050_SELF_TEST_TOLERANCE)
    {
        result->passed = false;
    }

    /*
     * Gyroscope self-test uses a simplified check.
     * The factory trim for gyroscope is computed from the self-test register
     * bits using the formula: FT = 25 * 131 * 1.046^(value - 1)
     */

    /* Gyroscope X */
    test_val = self_test_regs[0] & 0x1F;
    if (test_val != 0)
    {
        factory_trim = 25.0f * 131.0f * powf(1.046f, (float)test_val - 1.0f);
    }
    else
    {
        factory_trim = 0.0f;
    }
    if (factory_trim != 0.0f)
    {
        result->gyro_x_deviation = 100.0f *
                                   (gyro_st_response[0] - factory_trim) / factory_trim;
    }
    else
    {
        result->gyro_x_deviation = 0.0f;
    }
    if (fabsf(result->gyro_x_deviation) > MPU6050_SELF_TEST_TOLERANCE)
    {
        result->passed = false;
    }

    /* Gyroscope Y */
    test_val = self_test_regs[1] & 0x1F;
    if (test_val != 0)
    {
        factory_trim = -25.0f * 131.0f * powf(1.046f, (float)test_val - 1.0f);
    }
    else
    {
        factory_trim = 0.0f;
    }
    if (factory_trim != 0.0f)
    {
        result->gyro_y_deviation = 100.0f *
                                   (gyro_st_response[1] - factory_trim) / factory_trim;
    }
    else
    {
        result->gyro_y_deviation = 0.0f;
    }
    if (fabsf(result->gyro_y_deviation) > MPU6050_SELF_TEST_TOLERANCE)
    {
        result->passed = false;
    }

    /* Gyroscope Z */
    test_val = self_test_regs[2] & 0x1F;
    if (test_val != 0)
    {
        factory_trim = 25.0f * 131.0f * powf(1.046f, (float)test_val - 1.0f);
    }
    else
    {
        factory_trim = 0.0f;
    }
    if (factory_trim != 0.0f)
    {
        result->gyro_z_deviation = 100.0f *
                                   (gyro_st_response[2] - factory_trim) / factory_trim;
    }
    else
    {
        result->gyro_z_deviation = 0.0f;
    }
    if (fabsf(result->gyro_z_deviation) > MPU6050_SELF_TEST_TOLERANCE)
    {
        result->passed = false;
    }

    return MPU6050_OK;
}

/* ========================================================================== */
/*                          UTILITY FUNCTIONS                                 */
/* ========================================================================== */

bool MPU6050_IsConnected(MPU6050_Handle_t *hmpu)
{
    uint8_t who_am_i;
    MPU6050_Status_t status;

    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1);

    return (status == MPU6050_OK && who_am_i == MPU6050_WHO_AM_I_VALUE);
}

MPU6050_Status_t MPU6050_ReadWhoAmI(MPU6050_Handle_t *hmpu, uint8_t *who_am_i)
{
    if (who_am_i == NULL)
    {
        return MPU6050_INVALID_PARAM;
    }

    return MPU6050_ReadRegs(hmpu, MPU6050_REG_WHO_AM_I, who_am_i, 1);
}

bool MPU6050_IsDataReady(MPU6050_Handle_t *hmpu)
{
    uint8_t int_status;
    MPU6050_Status_t status;

    status = MPU6050_ReadRegs(hmpu, MPU6050_REG_INT_STATUS, &int_status, 1);

    if (status != MPU6050_OK)
    {
        return false;
    }

    return (int_status & MPU6050_INT_DATA_RDY) != 0;
}

MPU6050_Status_t MPU6050_Reset(MPU6050_Handle_t *hmpu)
{
    MPU6050_Status_t status;

    status = MPU6050_WriteReg(hmpu, MPU6050_REG_PWR_MGMT_1,
                              MPU6050_PWR1_DEVICE_RESET);
    if (status != MPU6050_OK)
    {
        return status;
    }

    HAL_Delay(MPU6050_RESET_DELAY);

    hmpu->initialized = false;

    return MPU6050_OK;
}
