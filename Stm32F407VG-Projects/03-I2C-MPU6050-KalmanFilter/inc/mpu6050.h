/**
 * @file    mpu6050.h
 * @brief   MPU6050 6-Axis IMU Sensor Driver for STM32F407VG
 * @author  STM32 Embedded Systems Portfolio
 * @version 2.0
 *
 * @details Complete driver for the InvenSense MPU6050 accelerometer/gyroscope
 *          sensor communicating over I2C. Supports configurable ranges, burst
 *          reads, calibration, self-test, and temperature measurement.
 *
 *          Features:
 *          - Full register map definitions
 *          - Configurable accelerometer range (2g/4g/8g/16g)
 *          - Configurable gyroscope range (250/500/1000/2000 dps)
 *          - 14-byte burst read for synchronized sensor data
 *          - Offset calibration routine
 *          - Factory self-test verification
 *          - Digital low-pass filter configuration
 *          - Temperature sensor readout
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* ========================================================================== */
    /*                              INCLUDES                                      */
    /* ========================================================================== */

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/*                         I2C ADDRESS DEFINITIONS                            */
/* ========================================================================== */

/**
 * @brief MPU6050 I2C slave addresses
 *
 * The address depends on the AD0 pin state:
 *   AD0 = LOW  -> Address = 0x68
 *   AD0 = HIGH -> Address = 0x69
 *
 * HAL library expects the address left-shifted by 1 bit.
 */
#define MPU6050_ADDR_AD0_LOW (0x68 << 1)  /**< 0xD0 when AD0 = GND */
#define MPU6050_ADDR_AD0_HIGH (0x69 << 1) /**< 0xD2 when AD0 = VCC */
#define MPU6050_DEFAULT_ADDR MPU6050_ADDR_AD0_LOW

/** @brief Expected value of the WHO_AM_I register */
#define MPU6050_WHO_AM_I_VALUE 0x68

/* ========================================================================== */
/*                         REGISTER MAP DEFINITIONS                           */
/* ========================================================================== */

/**
 * @defgroup MPU6050_Registers MPU6050 Register Addresses
 * @brief Complete register map for the MPU6050 sensor
 * @{
 */

/* Self-test registers */
#define MPU6050_REG_SELF_TEST_X 0x0D /**< X-axis self-test register       */
#define MPU6050_REG_SELF_TEST_Y 0x0E /**< Y-axis self-test register       */
#define MPU6050_REG_SELF_TEST_Z 0x0F /**< Z-axis self-test register       */
#define MPU6050_REG_SELF_TEST_A 0x10 /**< Accelerometer self-test register*/

/* Configuration registers */
#define MPU6050_REG_SMPLRT_DIV 0x19   /**< Sample rate divider             */
#define MPU6050_REG_CONFIG 0x1A       /**< General configuration           */
#define MPU6050_REG_GYRO_CONFIG 0x1B  /**< Gyroscope configuration         */
#define MPU6050_REG_ACCEL_CONFIG 0x1C /**< Accelerometer configuration     */

/* FIFO registers */
#define MPU6050_REG_FIFO_EN 0x23 /**< FIFO enable                     */

/* I2C master registers */
#define MPU6050_REG_I2C_MST_CTRL 0x24  /**< I2C master control              */
#define MPU6050_REG_I2C_SLV0_ADDR 0x25 /**< I2C slave 0 address             */
#define MPU6050_REG_I2C_SLV0_REG 0x26  /**< I2C slave 0 register            */
#define MPU6050_REG_I2C_SLV0_CTRL 0x27 /**< I2C slave 0 control             */

/* Interrupt registers */
#define MPU6050_REG_INT_PIN_CFG 0x37 /**< Interrupt pin configuration     */
#define MPU6050_REG_INT_ENABLE 0x38  /**< Interrupt enable                */
#define MPU6050_REG_INT_STATUS 0x3A  /**< Interrupt status                */

/* Accelerometer output registers (big-endian, high byte first) */
#define MPU6050_REG_ACCEL_XOUT_H 0x3B /**< Accelerometer X-axis high byte  */
#define MPU6050_REG_ACCEL_XOUT_L 0x3C /**< Accelerometer X-axis low byte   */
#define MPU6050_REG_ACCEL_YOUT_H 0x3D /**< Accelerometer Y-axis high byte  */
#define MPU6050_REG_ACCEL_YOUT_L 0x3E /**< Accelerometer Y-axis low byte   */
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F /**< Accelerometer Z-axis high byte  */
#define MPU6050_REG_ACCEL_ZOUT_L 0x40 /**< Accelerometer Z-axis low byte   */

/* Temperature output registers */
#define MPU6050_REG_TEMP_OUT_H 0x41 /**< Temperature high byte           */
#define MPU6050_REG_TEMP_OUT_L 0x42 /**< Temperature low byte            */

/* Gyroscope output registers (big-endian, high byte first) */
#define MPU6050_REG_GYRO_XOUT_H 0x43 /**< Gyroscope X-axis high byte      */
#define MPU6050_REG_GYRO_XOUT_L 0x44 /**< Gyroscope X-axis low byte       */
#define MPU6050_REG_GYRO_YOUT_H 0x45 /**< Gyroscope Y-axis high byte      */
#define MPU6050_REG_GYRO_YOUT_L 0x46 /**< Gyroscope Y-axis low byte       */
#define MPU6050_REG_GYRO_ZOUT_H 0x47 /**< Gyroscope Z-axis high byte      */
#define MPU6050_REG_GYRO_ZOUT_L 0x48 /**< Gyroscope Z-axis low byte       */

/* User control */
#define MPU6050_REG_USER_CTRL 0x6A /**< User control register           */

/* Power management registers */
#define MPU6050_REG_PWR_MGMT_1 0x6B /**< Power management 1              */
#define MPU6050_REG_PWR_MGMT_2 0x6C /**< Power management 2              */

/* FIFO count and data registers */
#define MPU6050_REG_FIFO_COUNT_H 0x72 /**< FIFO count high byte            */
#define MPU6050_REG_FIFO_COUNT_L 0x73 /**< FIFO count low byte             */
#define MPU6050_REG_FIFO_R_W 0x74     /**< FIFO read/write                 */

/* Identity register */
#define MPU6050_REG_WHO_AM_I 0x75 /**< Device identity register        */

/** @} */ /* End of MPU6050_Registers */

/* ========================================================================== */
/*                        REGISTER BIT DEFINITIONS                            */
/* ========================================================================== */

/**
 * @defgroup MPU6050_Bits MPU6050 Register Bit Definitions
 * @{
 */

/* PWR_MGMT_1 register bits */
#define MPU6050_PWR1_DEVICE_RESET (1 << 7) /**< Device reset bit            */
#define MPU6050_PWR1_SLEEP (1 << 6)        /**< Sleep mode bit              */
#define MPU6050_PWR1_CYCLE (1 << 5)        /**< Cycle mode bit              */
#define MPU6050_PWR1_TEMP_DIS (1 << 3)     /**< Temperature disable bit     */
#define MPU6050_PWR1_CLKSEL_MASK 0x07      /**< Clock select mask           */

/* Clock source selections for PWR_MGMT_1 CLKSEL field */
#define MPU6050_CLKSEL_INTERNAL 0x00  /**< Internal 8MHz oscillator        */
#define MPU6050_CLKSEL_PLL_XGYRO 0x01 /**< PLL with X-axis gyro reference */
#define MPU6050_CLKSEL_PLL_YGYRO 0x02 /**< PLL with Y-axis gyro reference */
#define MPU6050_CLKSEL_PLL_ZGYRO 0x03 /**< PLL with Z-axis gyro reference */
#define MPU6050_CLKSEL_STOP 0x07      /**< Stops the clock                 */

/* CONFIG register bits */
#define MPU6050_CFG_DLPF_MASK 0x07 /**< DLPF configuration mask         */

/* GYRO_CONFIG register bits */
#define MPU6050_GYRO_SELF_TEST_X (1 << 7)    /**< Gyro X self-test enable     */
#define MPU6050_GYRO_SELF_TEST_Y (1 << 6)    /**< Gyro Y self-test enable     */
#define MPU6050_GYRO_SELF_TEST_Z (1 << 5)    /**< Gyro Z self-test enable     */
#define MPU6050_GYRO_FS_SEL_MASK (0x03 << 3) /**< Gyro full-scale mask      */

/* ACCEL_CONFIG register bits */
#define MPU6050_ACCEL_SELF_TEST_X (1 << 7)    /**< Accel X self-test enable    */
#define MPU6050_ACCEL_SELF_TEST_Y (1 << 6)    /**< Accel Y self-test enable    */
#define MPU6050_ACCEL_SELF_TEST_Z (1 << 5)    /**< Accel Z self-test enable    */
#define MPU6050_ACCEL_FS_SEL_MASK (0x03 << 3) /**< Accel full-scale mask     */

/* INT_PIN_CFG register bits */
#define MPU6050_INT_LEVEL (1 << 7)    /**< INT pin active low          */
#define MPU6050_INT_OPEN (1 << 6)     /**< INT pin open drain          */
#define MPU6050_LATCH_INT_EN (1 << 5) /**< Latch INT pin               */
#define MPU6050_INT_RD_CLEAR (1 << 4) /**< Clear INT on any read       */

/* INT_ENABLE / INT_STATUS register bits */
#define MPU6050_INT_DATA_RDY (1 << 0)   /**< Data ready interrupt         */
#define MPU6050_INT_FIFO_OFLOW (1 << 4) /**< FIFO overflow interrupt      */

/* USER_CTRL register bits */
#define MPU6050_USERCTRL_FIFO_EN (1 << 6)    /**< FIFO enable                  */
#define MPU6050_USERCTRL_I2C_MST_EN (1 << 5) /**< I2C master mode enable       */
#define MPU6050_USERCTRL_FIFO_RST (1 << 2)   /**< FIFO reset                   */
#define MPU6050_USERCTRL_SIG_RST (1 << 0)    /**< Signal path reset            */

    /** @} */ /* End of MPU6050_Bits */

    /* ========================================================================== */
    /*                             ENUMERATIONS                                   */
    /* ========================================================================== */

    /**
     * @brief MPU6050 driver status codes
     */
    typedef enum
    {
        MPU6050_OK = 0x00,             /**< Operation successful               */
        MPU6050_ERROR = 0x01,          /**< General error                      */
        MPU6050_TIMEOUT = 0x02,        /**< Communication timeout              */
        MPU6050_INVALID_PARAM = 0x03,  /**< Invalid parameter                  */
        MPU6050_NOT_FOUND = 0x04,      /**< Device not found on I2C bus        */
        MPU6050_SELF_TEST_FAIL = 0x05, /**< Self-test failed                   */
        MPU6050_BUSY = 0x06,           /**< Device or bus is busy              */
        MPU6050_CALIBRATING = 0x07     /**< Calibration in progress            */
    } MPU6050_Status_t;

    /**
     * @brief Accelerometer full-scale range selection
     *
     * These values correspond to the AFS_SEL bits [4:3] in ACCEL_CONFIG register.
     */
    typedef enum
    {
        MPU6050_ACCEL_RANGE_2G = 0x00, /**< +/- 2g,  sensitivity: 16384 LSB/g */
        MPU6050_ACCEL_RANGE_4G = 0x01, /**< +/- 4g,  sensitivity:  8192 LSB/g */
        MPU6050_ACCEL_RANGE_8G = 0x02, /**< +/- 8g,  sensitivity:  4096 LSB/g */
        MPU6050_ACCEL_RANGE_16G = 0x03 /**< +/- 16g, sensitivity:  2048 LSB/g */
    } MPU6050_AccelRange_t;

    /**
     * @brief Gyroscope full-scale range selection
     *
     * These values correspond to the FS_SEL bits [4:3] in GYRO_CONFIG register.
     */
    typedef enum
    {
        MPU6050_GYRO_RANGE_250DPS = 0x00,  /**< +/- 250 dps,  sensitivity: 131.0 LSB/dps  */
        MPU6050_GYRO_RANGE_500DPS = 0x01,  /**< +/- 500 dps,  sensitivity:  65.5 LSB/dps  */
        MPU6050_GYRO_RANGE_1000DPS = 0x02, /**< +/- 1000 dps, sensitivity:  32.8 LSB/dps  */
        MPU6050_GYRO_RANGE_2000DPS = 0x03  /**< +/- 2000 dps, sensitivity:  16.4 LSB/dps  */
    } MPU6050_GyroRange_t;

    /**
     * @brief Digital Low-Pass Filter (DLPF) configuration
     *
     * Corresponds to the DLPF_CFG bits [2:0] in CONFIG register.
     * Bandwidth values are for the accelerometer; gyroscope values differ slightly.
     */
    typedef enum
    {
        MPU6050_DLPF_260HZ = 0x00, /**< Accel BW: 260 Hz, Gyro BW: 256 Hz      */
        MPU6050_DLPF_184HZ = 0x01, /**< Accel BW: 184 Hz, Gyro BW: 188 Hz      */
        MPU6050_DLPF_94HZ = 0x02,  /**< Accel BW:  94 Hz, Gyro BW:  98 Hz      */
        MPU6050_DLPF_44HZ = 0x03,  /**< Accel BW:  44 Hz, Gyro BW:  42 Hz      */
        MPU6050_DLPF_21HZ = 0x04,  /**< Accel BW:  21 Hz, Gyro BW:  20 Hz      */
        MPU6050_DLPF_10HZ = 0x05,  /**< Accel BW:  10 Hz, Gyro BW:  10 Hz      */
        MPU6050_DLPF_5HZ = 0x06    /**< Accel BW:   5 Hz, Gyro BW:   5 Hz      */
    } MPU6050_DLPF_t;

    /* ========================================================================== */
    /*                              DATA STRUCTURES                               */
    /* ========================================================================== */

    /**
     * @brief Raw 16-bit sensor data for all three axes
     */
    typedef struct
    {
        int16_t x; /**< X-axis raw value */
        int16_t y; /**< Y-axis raw value */
        int16_t z; /**< Z-axis raw value */
    } MPU6050_RawData_t;

    /**
     * @brief Scaled floating-point sensor data for all three axes
     */
    typedef struct
    {
        float x; /**< X-axis scaled value (g for accel, dps for gyro) */
        float y; /**< Y-axis scaled value (g for accel, dps for gyro) */
        float z; /**< Z-axis scaled value (g for accel, dps for gyro) */
    } MPU6050_ScaledData_t;

    /**
     * @brief Calibration offset data (stored as accumulated raw values)
     */
    typedef struct
    {
        int32_t accel_x; /**< Accelerometer X-axis offset (raw)    */
        int32_t accel_y; /**< Accelerometer Y-axis offset (raw)    */
        int32_t accel_z; /**< Accelerometer Z-axis offset (raw)    */
        int32_t gyro_x;  /**< Gyroscope X-axis offset (raw)        */
        int32_t gyro_y;  /**< Gyroscope Y-axis offset (raw)        */
        int32_t gyro_z;  /**< Gyroscope Z-axis offset (raw)        */
        bool is_valid;   /**< True if calibration has been performed */
    } MPU6050_Calibration_t;

    /**
     * @brief Self-test results
     */
    typedef struct
    {
        float accel_x_deviation; /**< X accel factory trim deviation (%) */
        float accel_y_deviation; /**< Y accel factory trim deviation (%) */
        float accel_z_deviation; /**< Z accel factory trim deviation (%) */
        float gyro_x_deviation;  /**< X gyro factory trim deviation (%)  */
        float gyro_y_deviation;  /**< Y gyro factory trim deviation (%)  */
        float gyro_z_deviation;  /**< Z gyro factory trim deviation (%)  */
        bool passed;             /**< True if all axes within tolerance  */
    } MPU6050_SelfTestResult_t;

    /**
     * @brief Complete MPU6050 device handle structure
     *
     * This structure holds all state and configuration for one MPU6050 device.
     * Initialize with MPU6050_Init() before use.
     */
    typedef struct
    {
        /* Hardware interface */
        I2C_HandleTypeDef *hi2c; /**< Pointer to HAL I2C handle         */
        uint16_t dev_addr;       /**< I2C device address (left-shifted) */

        /* Configuration */
        MPU6050_AccelRange_t accel_range; /**< Current accelerometer range       */
        MPU6050_GyroRange_t gyro_range;   /**< Current gyroscope range           */
        MPU6050_DLPF_t dlpf_cfg;          /**< Current DLPF setting              */
        uint8_t sample_div;               /**< Sample rate divider value         */

        /* Sensitivity scale factors (derived from range settings) */
        float accel_scale; /**< Accelerometer: LSB per g          */
        float gyro_scale;  /**< Gyroscope: LSB per deg/s          */

        /* Calibration data */
        MPU6050_Calibration_t calibration; /**< Calibration offsets               */

        /* Latest readings (updated by read functions) */
        MPU6050_RawData_t raw_accel; /**< Latest raw accelerometer data     */
        MPU6050_RawData_t raw_gyro;  /**< Latest raw gyroscope data         */
        int16_t raw_temp;            /**< Latest raw temperature data       */

        MPU6050_ScaledData_t accel; /**< Latest scaled accel data (g)      */
        MPU6050_ScaledData_t gyro;  /**< Latest scaled gyro data (dps)     */
        float temperature;          /**< Latest temperature (Celsius)      */

        /* Status */
        bool initialized; /**< True after successful init        */
    } MPU6050_Handle_t;

/* ========================================================================== */
/*                        CONFIGURATION DEFAULTS                              */
/* ========================================================================== */

/** @brief Default I2C communication timeout in milliseconds */
#define MPU6050_I2C_TIMEOUT 100

/** @brief Number of samples for calibration averaging */
#define MPU6050_CALIBRATION_SAMPLES 1000

/** @brief Delay between calibration samples in milliseconds */
#define MPU6050_CALIBRATION_DELAY 2

/** @brief Self-test deviation tolerance in percent */
#define MPU6050_SELF_TEST_TOLERANCE 14.0f

/** @brief Startup delay after reset in milliseconds */
#define MPU6050_RESET_DELAY 100

/** @brief Expected raw accelerometer value for 1g (at +/-2g range) */
#define MPU6050_ACCEL_1G_RAW 16384

    /* ========================================================================== */
    /*                         FUNCTION PROTOTYPES                                */
    /* ========================================================================== */

    /**
     * @defgroup MPU6050_Functions MPU6050 Driver Functions
     * @{
     */

    /* ---- Initialization and Configuration ----------------------------------- */

    /**
     * @brief  Initialize the MPU6050 sensor
     *
     * Performs the following steps:
     *   1. Verifies device identity via WHO_AM_I register
     *   2. Resets the device and waits for startup
     *   3. Wakes the device from sleep mode
     *   4. Configures clock source to PLL with X-axis gyro
     *   5. Sets accelerometer and gyroscope ranges
     *   6. Configures the digital low-pass filter
     *   7. Sets the sample rate divider
     *
     * @param  hmpu       Pointer to MPU6050 handle structure
     * @param  hi2c       Pointer to HAL I2C handle
     * @param  dev_addr   I2C device address (use MPU6050_DEFAULT_ADDR)
     * @param  accel_range Accelerometer full-scale range
     * @param  gyro_range  Gyroscope full-scale range
     * @param  dlpf       Digital low-pass filter setting
     * @param  sample_div Sample rate divider (Sample Rate = 1kHz / (1 + div))
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hmpu,
                                  I2C_HandleTypeDef *hi2c,
                                  uint16_t dev_addr,
                                  MPU6050_AccelRange_t accel_range,
                                  MPU6050_GyroRange_t gyro_range,
                                  MPU6050_DLPF_t dlpf,
                                  uint8_t sample_div);

    /**
     * @brief  Set the accelerometer full-scale range
     * @param  hmpu   Pointer to MPU6050 handle
     * @param  range  Desired accelerometer range
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_SetAccelRange(MPU6050_Handle_t *hmpu,
                                           MPU6050_AccelRange_t range);

    /**
     * @brief  Set the gyroscope full-scale range
     * @param  hmpu   Pointer to MPU6050 handle
     * @param  range  Desired gyroscope range
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_SetGyroRange(MPU6050_Handle_t *hmpu,
                                          MPU6050_GyroRange_t range);

    /**
     * @brief  Set the digital low-pass filter configuration
     * @param  hmpu  Pointer to MPU6050 handle
     * @param  dlpf  Desired DLPF setting
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_SetDLPF(MPU6050_Handle_t *hmpu, MPU6050_DLPF_t dlpf);

    /**
     * @brief  Set the sample rate divider
     * @param  hmpu       Pointer to MPU6050 handle
     * @param  divider    Sample rate divider (0-255)
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_SetSampleRate(MPU6050_Handle_t *hmpu, uint8_t divider);

    /* ---- Data Reading Functions --------------------------------------------- */

    /**
     * @brief  Read accelerometer data from the sensor
     *
     * Reads 6 bytes (X, Y, Z high and low) and converts to scaled values in g.
     * Results are stored in hmpu->accel and hmpu->raw_accel.
     * Calibration offsets are applied if calibration is valid.
     *
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_ReadAccel(MPU6050_Handle_t *hmpu);

    /**
     * @brief  Read gyroscope data from the sensor
     *
     * Reads 6 bytes (X, Y, Z high and low) and converts to scaled values in dps.
     * Results are stored in hmpu->gyro and hmpu->raw_gyro.
     * Calibration offsets are applied if calibration is valid.
     *
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_ReadGyro(MPU6050_Handle_t *hmpu);

    /**
     * @brief  Read temperature from the sensor
     *
     * Reads 2 bytes and converts to degrees Celsius using the formula:
     *   Temperature = (raw_value / 340.0) + 36.53
     *
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_ReadTemperature(MPU6050_Handle_t *hmpu);

    /**
     * @brief  Read all sensor data in a single burst transfer
     *
     * Reads 14 consecutive bytes starting from ACCEL_XOUT_H (0x3B):
     *   Bytes  0-5:  Accelerometer X, Y, Z
     *   Bytes  6-7:  Temperature
     *   Bytes  8-13: Gyroscope X, Y, Z
     *
     * This is the most efficient method as it minimizes I2C overhead and
     * ensures temporal coherence between accelerometer and gyroscope readings.
     *
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_ReadAll(MPU6050_Handle_t *hmpu);

    /* ---- Calibration Functions ---------------------------------------------- */

    /**
     * @brief  Perform sensor calibration
     *
     * The sensor must be placed on a flat, vibration-free surface with the
     * Z-axis pointing upward. This function:
     *   1. Collects MPU6050_CALIBRATION_SAMPLES samples
     *   2. Computes average offsets for each axis
     *   3. Accounts for 1g on the Z accelerometer axis
     *   4. Stores offsets in hmpu->calibration
     *
     * Duration: approximately (SAMPLES * DELAY) milliseconds = ~2 seconds
     *
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_Calibrate(MPU6050_Handle_t *hmpu);

    /**
     * @brief  Reset calibration offsets to zero
     * @param  hmpu  Pointer to MPU6050 handle
     */
    void MPU6050_ResetCalibration(MPU6050_Handle_t *hmpu);

    /* ---- Self-Test Function ------------------------------------------------- */

    /**
     * @brief  Run the built-in self-test
     *
     * Activates the internal self-test actuators and compares the response
     * against factory trim values. Results are stored in the provided
     * result structure.
     *
     * @param  hmpu    Pointer to MPU6050 handle
     * @param  result  Pointer to self-test result structure
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_SelfTest(MPU6050_Handle_t *hmpu,
                                      MPU6050_SelfTestResult_t *result);

    /* ---- Utility Functions -------------------------------------------------- */

    /**
     * @brief  Check if the MPU6050 is connected and responding
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval true if device responds correctly, false otherwise
     */
    bool MPU6050_IsConnected(MPU6050_Handle_t *hmpu);

    /**
     * @brief  Read the WHO_AM_I register
     * @param  hmpu      Pointer to MPU6050 handle
     * @param  who_am_i  Pointer to store the read value
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_ReadWhoAmI(MPU6050_Handle_t *hmpu, uint8_t *who_am_i);

    /**
     * @brief  Check if new data is available (data ready interrupt status)
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval true if new data is available
     */
    bool MPU6050_IsDataReady(MPU6050_Handle_t *hmpu);

    /**
     * @brief  Reset the MPU6050 device
     *
     * Writes the DEVICE_RESET bit in PWR_MGMT_1 and waits for the
     * device to complete its reset sequence.
     *
     * @param  hmpu  Pointer to MPU6050 handle
     * @retval MPU6050_Status_t
     */
    MPU6050_Status_t MPU6050_Reset(MPU6050_Handle_t *hmpu);

    /** @} */ /* End of MPU6050_Functions */

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
