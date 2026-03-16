/**
 * @file    main.c
 * @brief   MPU6050 IMU with Kalman Filter - Main Application
 * @author  STM32 Embedded Systems Portfolio
 * @version 2.0
 *
 * @details Main application for Project 3: I2C-MPU6050-KalmanFilter
 *
 *          This application:
 *          - Initializes I2C1 (PB6/PB7) at 400 kHz for MPU6050 communication
 *          - Initializes USART2 (PA2/PA3) at 115200 baud for data output
 *          - Configures TIM7 for 200 Hz periodic sampling
 *          - Reads accelerometer and gyroscope data via 14-byte burst reads
 *          - Computes roll and pitch angles from accelerometer using atan2
 *          - Applies dual Kalman filters (one for roll, one for pitch)
 *          - Also runs complementary filters for comparison
 *          - Outputs data in CSV format over UART
 *          - Provides a command interface for calibration and parameter tuning
 *
 *          Hardware connections:
 *            PB6 -> I2C1_SCL -> MPU6050 SCL
 *            PB7 -> I2C1_SDA -> MPU6050 SDA
 *            PA2 -> USART2_TX -> USB-TTL converter RX
 *            PA3 -> USART2_RX -> USB-TTL converter TX
 *            PD12 -> Green LED  (system running indicator)
 *            PD13 -> Orange LED (data read indicator)
 *            PD14 -> Red LED    (error indicator)
 *            PD15 -> Blue LED   (calibration indicator)
 */

/* ========================================================================== */
/*                              INCLUDES                                      */
/* ========================================================================== */

#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "kalman_filter.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              DEFINES                                       */
/* ========================================================================== */

/** @brief Sampling frequency in Hz (must match TIM7 configuration) */
#define SAMPLE_FREQ_HZ 200

/** @brief Sampling period in seconds */
#define SAMPLE_PERIOD_S (1.0f / (float)SAMPLE_FREQ_HZ)

/** @brief Mathematical constant PI */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/** @brief Conversion factor: radians to degrees */
#define RAD_TO_DEG (180.0f / M_PI)

/** @brief UART transmit buffer size */
#define UART_TX_BUF_SIZE 256

/** @brief UART receive buffer size */
#define UART_RX_BUF_SIZE 16

/** @brief Parameter adjustment step multiplier (multiplicative) */
#define PARAM_ADJUST_FACTOR 1.5f

/* LED pin definitions (STM32F407 Discovery board) */
#define LED_GREEN_PIN GPIO_PIN_12
#define LED_ORANGE_PIN GPIO_PIN_13
#define LED_RED_PIN GPIO_PIN_14
#define LED_BLUE_PIN GPIO_PIN_15
#define LED_GPIO_PORT GPIOD

/* ========================================================================== */
/*                         OUTPUT MODE ENUMERATION                            */
/* ========================================================================== */

/**
 * @brief Data output mode selection
 */
typedef enum
{
    OUTPUT_MODE_KALMAN = 0,    /**< Kalman-filtered angles (default)    */
    OUTPUT_MODE_COMPLEMENTARY, /**< Complementary-filtered angles       */
    OUTPUT_MODE_RAW,           /**< Raw sensor data (g and dps)         */
    OUTPUT_MODE_ALL            /**< All data: Kalman + Comp + Raw       */
} OutputMode_t;

/* ========================================================================== */
/*                          GLOBAL VARIABLES                                  */
/* ========================================================================== */

/* --- Peripheral handles --- */
I2C_HandleTypeDef hi2c1;   /**< I2C1 handle for MPU6050             */
UART_HandleTypeDef huart2; /**< USART2 handle for data output       */
TIM_HandleTypeDef htim7;   /**< TIM7 handle for 200 Hz sampling     */

/* --- Sensor and filter instances --- */
MPU6050_Handle_t mpu6050;               /**< MPU6050 sensor handle               */
Kalman_State_t kalman_roll;             /**< Kalman filter for roll axis         */
Kalman_State_t kalman_pitch;            /**< Kalman filter for pitch axis        */
ComplementaryFilter_State_t comp_roll;  /**< Comp filter for roll           */
ComplementaryFilter_State_t comp_pitch; /**< Comp filter for pitch          */

/* --- Application state --- */
volatile bool data_ready = false;           /**< Flag set by TIM7 ISR          */
volatile uint32_t sample_counter = 0;       /**< Total samples taken           */
bool output_enabled = true;                 /**< UART output on/off           */
bool filters_initialized = false;           /**< First reading done?    */
OutputMode_t output_mode = OUTPUT_MODE_ALL; /**< Current output mode  */

/* --- Computed angles --- */
float roll_kalman = 0.0f;  /**< Kalman-filtered roll (deg)    */
float pitch_kalman = 0.0f; /**< Kalman-filtered pitch (deg)   */
float roll_comp = 0.0f;    /**< Complementary-filtered roll   */
float pitch_comp = 0.0f;   /**< Complementary-filtered pitch  */
float roll_accel = 0.0f;   /**< Roll from accelerometer only  */
float pitch_accel = 0.0f;  /**< Pitch from accelerometer only */

/* --- UART buffers --- */
char uart_tx_buf[UART_TX_BUF_SIZE]; /**< Transmit buffer     */
uint8_t uart_rx_byte;               /**< Single byte receive */

/* ========================================================================== */
/*                     PERIPHERAL INITIALIZATION                              */
/* ========================================================================== */

/**
 * @brief  Configure the system clock to 168 MHz using HSE and PLL
 *
 * Clock tree:
 *   HSE (8 MHz) -> PLL (PLLM=8, PLLN=336, PLLP=2) -> SYSCLK = 168 MHz
 *   AHB  = SYSCLK / 1  = 168 MHz
 *   APB1 = SYSCLK / 4  = 42 MHz  (timer clock = 84 MHz)
 *   APB2 = SYSCLK / 2  = 84 MHz
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable Power Control clock and set voltage scaling for 168 MHz */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure HSE and PLL */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure bus clock dividers */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Initialize GPIO pins for onboard LEDs
 *
 * STM32F407 Discovery board LEDs:
 *   PD12 = Green, PD13 = Orange, PD14 = Red, PD15 = Blue
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure LED pins as push-pull outputs */
    GPIO_InitStruct.Pin = LED_GREEN_PIN | LED_ORANGE_PIN |
                          LED_RED_PIN | LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    /* All LEDs off initially */
    HAL_GPIO_WritePin(LED_GPIO_PORT,
                      LED_GREEN_PIN | LED_ORANGE_PIN |
                          LED_RED_PIN | LED_BLUE_PIN,
                      GPIO_PIN_RESET);
}

/**
 * @brief  Initialize I2C1 for MPU6050 communication
 *
 * Configuration:
 *   - I2C1 on PB6 (SCL) and PB7 (SDA)
 *   - Fast Mode: 400 kHz
 *   - 7-bit addressing
 *   - Analog filter enabled for noise rejection
 *   - Clock stretching enabled (MPU6050 may use it)
 */
static void I2C1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable peripheral clocks */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * Configure PB6 (SCL) and PB7 (SDA) for I2C1 alternate function.
     * Open-drain mode is required for I2C.
     * Internal pull-ups can be used if no external pull-ups are present,
     * but external 4.7k pull-ups are recommended for reliable operation.
     */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure I2C1 peripheral */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;         /* 400 kHz Fast Mode      */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; /* T_low/T_high = 2      */
    hi2c1.Init.OwnAddress1 = 0x00;          /* Not used in master    */
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable analog filter for better noise immunity */
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

/**
 * @brief  Initialize USART2 for data output and command input
 *
 * Configuration:
 *   - USART2 on PA2 (TX) and PA3 (RX)
 *   - 115200 baud, 8N1
 *   - Receive interrupt enabled for command processing
 */
static void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable peripheral clocks */
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Configure PA2 (TX) and PA3 (RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure USART2 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable USART2 receive interrupt for command processing */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* Start receiving one byte at a time (interrupt-driven) */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
}

/**
 * @brief  Initialize TIM7 for 200 Hz periodic interrupts
 *
 * TIM7 is a basic timer on APB1. Since APB1 prescaler > 1,
 * the timer clock is 2x APB1 = 84 MHz.
 *
 * For 200 Hz interrupts:
 *   Timer Clock = 84 MHz
 *   Prescaler   = 8400 - 1 = 8399  (84 MHz / 8400 = 10 kHz)
 *   Period      = 50 - 1   = 49    (10 kHz / 50 = 200 Hz)
 */
static void TIM7_Init(void)
{
    /* Enable TIM7 clock */
    __HAL_RCC_TIM7_CLK_ENABLE();

    /* Configure TIM7 */
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 8400 - 1; /* 84 MHz / 8400 = 10 kHz */
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 50 - 1; /* 10 kHz / 50 = 200 Hz  */
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure and enable TIM7 interrupt */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* ========================================================================== */
/*                         ANGLE COMPUTATION                                  */
/* ========================================================================== */

/**
 * @brief  Compute roll and pitch angles from accelerometer data
 *
 * Roll  = atan2(Ay, sqrt(Ax^2 + Az^2))  [rotation about X-axis]
 * Pitch = atan2(-Ax, sqrt(Ay^2 + Az^2)) [rotation about Y-axis]
 *
 * These formulas avoid singularity issues at +/-90 degrees for the
 * non-computed axis and handle all four quadrants correctly via atan2.
 *
 * @param  ax  Accelerometer X reading in g
 * @param  ay  Accelerometer Y reading in g
 * @param  az  Accelerometer Z reading in g
 * @param  roll   Pointer to store computed roll in degrees
 * @param  pitch  Pointer to store computed pitch in degrees
 */
static void ComputeAccelAngles(float ax, float ay, float az,
                               float *roll, float *pitch)
{
    /*
     * Roll: rotation about the X-axis
     *
     * Using atan2(Ay, Az) provides a full 360-degree range but is
     * susceptible to issues when Az is near zero. The formula below
     * uses sqrt(Ax^2 + Az^2) which limits roll to +/-90 degrees but
     * is more stable for typical applications.
     *
     * For a full-range roll (useful for aerial applications):
     *   *roll = atan2f(ay, az) * RAD_TO_DEG;
     */
    *roll = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;

    /*
     * Pitch: rotation about the Y-axis
     *
     * The negative sign on Ax accounts for the sensor coordinate system
     * convention where tilting the nose up produces a positive pitch.
     */
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

/* ========================================================================== */
/*                          UART OUTPUT FUNCTIONS                             */
/* ========================================================================== */

/**
 * @brief  Transmit a string over UART2 (blocking mode)
 * @param  str  Null-terminated string to transmit
 */
static void UART_Print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 100);
}

/**
 * @brief  Print the startup banner and system information
 */
static void PrintBanner(void)
{
    UART_Print("\r\n");
    UART_Print("========================================================\r\n");
    UART_Print("  STM32F407VG - MPU6050 IMU with Kalman Filter\r\n");
    UART_Print("  Project 3: I2C-MPU6050-KalmanFilter\r\n");
    UART_Print("========================================================\r\n");
    UART_Print("\r\n");

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  Sample Rate : %d Hz\r\n", SAMPLE_FREQ_HZ);
    UART_Print(uart_tx_buf);

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  I2C Speed   : 400 kHz (Fast Mode)\r\n");
    UART_Print(uart_tx_buf);

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  UART Baud   : 115200\r\n");
    UART_Print(uart_tx_buf);

    UART_Print("\r\n");
    UART_Print("  Press 'H' for help menu\r\n");
    UART_Print("========================================================\r\n");
    UART_Print("\r\n");
}

/**
 * @brief  Print the help menu with available commands
 */
static void PrintHelp(void)
{
    UART_Print("\r\n--- Command Menu ---\r\n");
    UART_Print("  C : Start calibration\r\n");
    UART_Print("  S : Run self-test\r\n");
    UART_Print("  R : Raw data output mode\r\n");
    UART_Print("  K : Kalman filter output mode\r\n");
    UART_Print("  F : Complementary filter output mode\r\n");
    UART_Print("  A : All data output mode\r\n");
    UART_Print("  D : Toggle data output on/off\r\n");
    UART_Print("  Q/q : Increase/decrease Q_angle\r\n");
    UART_Print("  W/w : Increase/decrease Q_bias\r\n");
    UART_Print("  E/e : Increase/decrease R_measure\r\n");
    UART_Print("  P : Print current parameters\r\n");
    UART_Print("  H : This help menu\r\n");
    UART_Print("--------------------\r\n\r\n");
}

/**
 * @brief  Print current Kalman filter parameters
 */
static void PrintParameters(void)
{
    UART_Print("\r\n--- Kalman Filter Parameters ---\r\n");

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  Q_angle   = %.6f\r\n", kalman_roll.Q_angle);
    UART_Print(uart_tx_buf);

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  Q_bias    = %.6f\r\n", kalman_roll.Q_bias);
    UART_Print(uart_tx_buf);

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  R_measure = %.6f\r\n", kalman_roll.R_measure);
    UART_Print(uart_tx_buf);

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  Comp alpha = %.4f\r\n", comp_roll.alpha);
    UART_Print(uart_tx_buf);

    UART_Print("\r\n--- Current Bias Estimates ---\r\n");

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  Roll bias  = %.4f deg/s\r\n", kalman_roll.bias);
    UART_Print(uart_tx_buf);

    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "  Pitch bias = %.4f deg/s\r\n", kalman_pitch.bias);
    UART_Print(uart_tx_buf);

    UART_Print("--------------------------------\r\n\r\n");
}

/**
 * @brief  Print CSV header line matching the current output mode
 */
static void PrintCSVHeader(void)
{
    switch (output_mode)
    {
    case OUTPUT_MODE_KALMAN:
        UART_Print("time_ms,roll_kal,pitch_kal\r\n");
        break;

    case OUTPUT_MODE_COMPLEMENTARY:
        UART_Print("time_ms,roll_comp,pitch_comp\r\n");
        break;

    case OUTPUT_MODE_RAW:
        UART_Print("time_ms,ax,ay,az,gx,gy,gz,temp\r\n");
        break;

    case OUTPUT_MODE_ALL:
        UART_Print("time_ms,roll_kal,pitch_kal,roll_comp,"
                   "pitch_comp,ax,ay,az,gx,gy,gz,temp\r\n");
        break;
    }
}

/**
 * @brief  Output one line of CSV data based on current output mode
 */
static void OutputData(void)
{
    uint32_t timestamp_ms = HAL_GetTick();
    int len = 0;

    switch (output_mode)
    {
    case OUTPUT_MODE_KALMAN:
        len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                       "%lu,%.2f,%.2f\r\n",
                       timestamp_ms,
                       roll_kalman, pitch_kalman);
        break;

    case OUTPUT_MODE_COMPLEMENTARY:
        len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                       "%lu,%.2f,%.2f\r\n",
                       timestamp_ms,
                       roll_comp, pitch_comp);
        break;

    case OUTPUT_MODE_RAW:
        len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                       "%lu,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f\r\n",
                       timestamp_ms,
                       mpu6050.accel.x, mpu6050.accel.y, mpu6050.accel.z,
                       mpu6050.gyro.x, mpu6050.gyro.y, mpu6050.gyro.z,
                       mpu6050.temperature);
        break;

    case OUTPUT_MODE_ALL:
        len = snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                       "%lu,%.2f,%.2f,%.2f,%.2f,"
                       "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.1f\r\n",
                       timestamp_ms,
                       roll_kalman, pitch_kalman,
                       roll_comp, pitch_comp,
                       mpu6050.accel.x, mpu6050.accel.y, mpu6050.accel.z,
                       mpu6050.gyro.x, mpu6050.gyro.y, mpu6050.gyro.z,
                       mpu6050.temperature);
        break;
    }

    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buf, len, 50);
    }
}

/* ========================================================================== */
/*                         COMMAND PROCESSING                                 */
/* ========================================================================== */

/**
 * @brief  Process a received UART command character
 * @param  cmd  The command character received
 */
static void ProcessCommand(uint8_t cmd)
{
    switch (cmd)
    {

    /* --- Calibration --- */
    case 'C':
    case 'c':
    {
        UART_Print("\r\n[CAL] Starting calibration...\r\n");
        UART_Print("[CAL] Keep sensor flat and still!\r\n");

        /* Visual indicator: blue LED on during calibration */
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_SET);

        /* Stop timer during calibration to prevent contention */
        HAL_TIM_Base_Stop_IT(&htim7);

        MPU6050_Status_t cal_status = MPU6050_Calibrate(&mpu6050);

        if (cal_status == MPU6050_OK)
        {
            UART_Print("[CAL] Calibration complete!\r\n");

            snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                     "[CAL] Accel offsets: X=%ld, Y=%ld, Z=%ld\r\n",
                     mpu6050.calibration.accel_x,
                     mpu6050.calibration.accel_y,
                     mpu6050.calibration.accel_z);
            UART_Print(uart_tx_buf);

            snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                     "[CAL] Gyro offsets:  X=%ld, Y=%ld, Z=%ld\r\n",
                     mpu6050.calibration.gyro_x,
                     mpu6050.calibration.gyro_y,
                     mpu6050.calibration.gyro_z);
            UART_Print(uart_tx_buf);

            /* Reset filters after calibration for clean start */
            Kalman_Reset(&kalman_roll);
            Kalman_Reset(&kalman_pitch);
            CompFilter_Reset(&comp_roll);
            CompFilter_Reset(&comp_pitch);
            filters_initialized = false;
        }
        else
        {
            UART_Print("[CAL] Calibration FAILED!\r\n");
        }

        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);

        /* Restart timer */
        HAL_TIM_Base_Start_IT(&htim7);
        break;
    }

    /* --- Self-Test --- */
    case 'S':
    case 's':
    {
        UART_Print("\r\n[ST] Running self-test...\r\n");

        HAL_TIM_Base_Stop_IT(&htim7);

        MPU6050_SelfTestResult_t st_result;
        MPU6050_Status_t st_status = MPU6050_SelfTest(&mpu6050, &st_result);

        if (st_status == MPU6050_OK)
        {
            snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                     "[ST] Accel deviation: X=%.1f%%, Y=%.1f%%, Z=%.1f%%\r\n",
                     st_result.accel_x_deviation,
                     st_result.accel_y_deviation,
                     st_result.accel_z_deviation);
            UART_Print(uart_tx_buf);

            snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                     "[ST] Gyro deviation:  X=%.1f%%, Y=%.1f%%, Z=%.1f%%\r\n",
                     st_result.gyro_x_deviation,
                     st_result.gyro_y_deviation,
                     st_result.gyro_z_deviation);
            UART_Print(uart_tx_buf);

            UART_Print(st_result.passed ? "[ST] Result: PASS\r\n" : "[ST] Result: FAIL\r\n");
        }
        else
        {
            UART_Print("[ST] Self-test error!\r\n");
        }

        HAL_TIM_Base_Start_IT(&htim7);
        break;
    }

    /* --- Output mode selection --- */
    case 'R':
        output_mode = OUTPUT_MODE_RAW;
        UART_Print("\r\n[MODE] Raw data output\r\n");
        PrintCSVHeader();
        break;

    case 'K':
        output_mode = OUTPUT_MODE_KALMAN;
        UART_Print("\r\n[MODE] Kalman filter output\r\n");
        PrintCSVHeader();
        break;

    case 'F':
        output_mode = OUTPUT_MODE_COMPLEMENTARY;
        UART_Print("\r\n[MODE] Complementary filter output\r\n");
        PrintCSVHeader();
        break;

    case 'A':
        output_mode = OUTPUT_MODE_ALL;
        UART_Print("\r\n[MODE] All data output\r\n");
        PrintCSVHeader();
        break;

    /* --- Toggle output --- */
    case 'D':
    case 'd':
        output_enabled = !output_enabled;
        UART_Print(output_enabled ? "\r\n[OUT] Data output ENABLED\r\n" : "\r\n[OUT] Data output DISABLED\r\n");
        if (output_enabled)
        {
            PrintCSVHeader();
        }
        break;

    /* --- Kalman parameter tuning: Q_angle --- */
    case 'Q':
        Kalman_SetQAngle(&kalman_roll,
                         kalman_roll.Q_angle * PARAM_ADJUST_FACTOR);
        Kalman_SetQAngle(&kalman_pitch,
                         kalman_pitch.Q_angle * PARAM_ADJUST_FACTOR);
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "\r\n[TUNE] Q_angle = %.6f\r\n", kalman_roll.Q_angle);
        UART_Print(uart_tx_buf);
        break;

    case 'q':
        Kalman_SetQAngle(&kalman_roll,
                         kalman_roll.Q_angle / PARAM_ADJUST_FACTOR);
        Kalman_SetQAngle(&kalman_pitch,
                         kalman_pitch.Q_angle / PARAM_ADJUST_FACTOR);
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "\r\n[TUNE] Q_angle = %.6f\r\n", kalman_roll.Q_angle);
        UART_Print(uart_tx_buf);
        break;

    /* --- Kalman parameter tuning: Q_bias --- */
    case 'W':
        Kalman_SetQBias(&kalman_roll,
                        kalman_roll.Q_bias * PARAM_ADJUST_FACTOR);
        Kalman_SetQBias(&kalman_pitch,
                        kalman_pitch.Q_bias * PARAM_ADJUST_FACTOR);
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "\r\n[TUNE] Q_bias = %.6f\r\n", kalman_roll.Q_bias);
        UART_Print(uart_tx_buf);
        break;

    case 'w':
        Kalman_SetQBias(&kalman_roll,
                        kalman_roll.Q_bias / PARAM_ADJUST_FACTOR);
        Kalman_SetQBias(&kalman_pitch,
                        kalman_pitch.Q_bias / PARAM_ADJUST_FACTOR);
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "\r\n[TUNE] Q_bias = %.6f\r\n", kalman_roll.Q_bias);
        UART_Print(uart_tx_buf);
        break;

    /* --- Kalman parameter tuning: R_measure --- */
    case 'E':
        Kalman_SetRMeasure(&kalman_roll,
                           kalman_roll.R_measure * PARAM_ADJUST_FACTOR);
        Kalman_SetRMeasure(&kalman_pitch,
                           kalman_pitch.R_measure * PARAM_ADJUST_FACTOR);
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "\r\n[TUNE] R_measure = %.6f\r\n", kalman_roll.R_measure);
        UART_Print(uart_tx_buf);
        break;

    case 'e':
        Kalman_SetRMeasure(&kalman_roll,
                           kalman_roll.R_measure / PARAM_ADJUST_FACTOR);
        Kalman_SetRMeasure(&kalman_pitch,
                           kalman_pitch.R_measure / PARAM_ADJUST_FACTOR);
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "\r\n[TUNE] R_measure = %.6f\r\n", kalman_roll.R_measure);
        UART_Print(uart_tx_buf);
        break;

    /* --- Print parameters --- */
    case 'P':
    case 'p':
        PrintParameters();
        break;

    /* --- Help --- */
    case 'H':
    case 'h':
        PrintHelp();
        break;

    default:
        /* Ignore unknown commands */
        break;
    }
}

/* ========================================================================== */
/*                        INTERRUPT CALLBACKS                                 */
/* ========================================================================== */

/**
 * @brief  TIM7 period elapsed callback (200 Hz)
 *
 * This ISR sets the data_ready flag which is polled in the main loop.
 * Actual sensor reading and filtering are done in the main loop context
 * to avoid long ISR execution times.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {
        data_ready = true;
        sample_counter++;

        /* Toggle green LED every 100 samples (0.5 Hz blink = system alive) */
        if ((sample_counter % 100) == 0)
        {
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GREEN_PIN);
        }
    }
}

/**
 * @brief  UART receive complete callback
 *
 * Called when one byte is received via UART interrupt.
 * Processes the command and re-arms the receive.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        ProcessCommand(uart_rx_byte);

        /* Re-arm receive for next byte */
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/* ========================================================================== */
/*                       INTERRUPT HANDLERS                                   */
/* ========================================================================== */

/**
 * @brief TIM7 global interrupt handler
 */
void TIM7_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim7);
}

/**
 * @brief USART2 global interrupt handler
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* ========================================================================== */
/*                         MAIN APPLICATION                                   */
/* ========================================================================== */

/**
 * @brief  Main entry point
 *
 * Initialization sequence:
 *   1. HAL init and system clock configuration
 *   2. GPIO, I2C, UART, Timer initialization
 *   3. MPU6050 initialization and connectivity check
 *   4. Kalman and complementary filter initialization
 *   5. Start timer and enter main processing loop
 *
 * Main loop:
 *   - Waits for TIM7 interrupt (200 Hz)
 *   - Reads all MPU6050 data via burst read
 *   - Computes angles from accelerometer
 *   - Updates Kalman and complementary filters
 *   - Outputs data via UART
 */
int main(void)
{
    /* ---- Step 1: HAL Initialization ---- */
    HAL_Init();
    SystemClock_Config();

    /* ---- Step 2: Peripheral Initialization ---- */
    GPIO_Init();
    I2C1_Init();
    USART2_Init();
    TIM7_Init();

    /* ---- Step 3: Print startup banner ---- */
    PrintBanner();

    /* ---- Step 4: Initialize MPU6050 ----
     *
     * Configuration:
     *   - Accelerometer: +/- 2g range (maximum sensitivity)
     *   - Gyroscope:     +/- 250 dps range (maximum sensitivity)
     *   - DLPF:          44 Hz bandwidth (good for 200 Hz sampling)
     *   - Sample divider: 4 (1 kHz / 5 = 200 Hz internal rate)
     */
    UART_Print("[INIT] Initializing MPU6050...\r\n");

    MPU6050_Status_t mpu_status = MPU6050_Init(&mpu6050,
                                               &hi2c1,
                                               MPU6050_DEFAULT_ADDR,
                                               MPU6050_ACCEL_RANGE_2G,
                                               MPU6050_GYRO_RANGE_250DPS,
                                               MPU6050_DLPF_44HZ,
                                               4);

    if (mpu_status != MPU6050_OK)
    {
        /* MPU6050 initialization failed */
        UART_Print("[ERROR] MPU6050 init failed! Check I2C connections.\r\n");
        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "[ERROR] Status code: %d\r\n", mpu_status);
        UART_Print(uart_tx_buf);

        /* Blink red LED to indicate error */
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);
        while (1)
        {
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_RED_PIN);
            HAL_Delay(200);
        }
    }

    /* Verify connection by reading WHO_AM_I */
    uint8_t who_am_i;
    MPU6050_ReadWhoAmI(&mpu6050, &who_am_i);
    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "[INIT] MPU6050 WHO_AM_I = 0x%02X (expected 0x68)\r\n", who_am_i);
    UART_Print(uart_tx_buf);
    UART_Print("[INIT] MPU6050 initialized successfully!\r\n");

    /* ---- Step 5: Initialize Kalman Filters ----
     *
     * Two independent Kalman filter instances: one for roll, one for pitch.
     * Both use the same tuning parameters initially.
     */
    UART_Print("[INIT] Initializing Kalman filters...\r\n");

    Kalman_Init(&kalman_roll);
    Kalman_Init(&kalman_pitch);

    /* ---- Step 6: Initialize Complementary Filters ---- */
    CompFilter_Init(&comp_roll);
    CompFilter_Init(&comp_pitch);

    UART_Print("[INIT] Filters initialized.\r\n");

    /* ---- Step 7: Optional automatic calibration ---- */
    UART_Print("[INIT] Starting auto-calibration (keep sensor still)...\r\n");
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_SET);

    if (MPU6050_Calibrate(&mpu6050) == MPU6050_OK)
    {
        UART_Print("[INIT] Calibration successful!\r\n");

        snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
                 "[INIT] Gyro bias: X=%.2f, Y=%.2f, Z=%.2f dps\r\n",
                 (float)mpu6050.calibration.gyro_x / mpu6050.gyro_scale,
                 (float)mpu6050.calibration.gyro_y / mpu6050.gyro_scale,
                 (float)mpu6050.calibration.gyro_z / mpu6050.gyro_scale);
        UART_Print(uart_tx_buf);
    }
    else
    {
        UART_Print("[INIT] Calibration failed, proceeding without.\r\n");
    }

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);

    /* ---- Step 8: Print CSV header and start ---- */
    UART_Print("\r\n[RUN] Starting data acquisition...\r\n");
    PrintCSVHeader();

    /* Start the 200 Hz timer */
    HAL_TIM_Base_Start_IT(&htim7);

    /* ================================================================== */
    /*                         MAIN PROCESSING LOOP                       */
    /* ================================================================== */

    while (1)
    {
        /*
         * Wait for the TIM7 interrupt to set the data_ready flag.
         * This ensures precise 200 Hz sampling regardless of processing time.
         * The CPU can sleep between samples to save power.
         */
        if (!data_ready)
        {
            /* Optional: enter sleep mode to save power */
            /* __WFI(); */
            continue;
        }

        /* Clear the flag */
        data_ready = false;

        /* Toggle orange LED to indicate sensor read activity */
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);

        /*
         * ---- Read all sensor data ----
         * Burst read 14 bytes: accel(6) + temp(2) + gyro(6)
         * This is the most efficient method and ensures all data
         * is from the same measurement cycle.
         */
        MPU6050_Status_t read_status = MPU6050_ReadAll(&mpu6050);

        if (read_status != MPU6050_OK)
        {
            /* I2C read error - turn on red LED and skip this sample */
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
            continue;
        }

        /* Clear error LED on successful read */
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_RESET);

        /*
         * ---- Compute angles from accelerometer ----
         * These are the "measurement" input to the Kalman filter.
         * Accurate in static conditions but noisy during motion.
         */
        ComputeAccelAngles(mpu6050.accel.x, mpu6050.accel.y, mpu6050.accel.z,
                           &roll_accel, &pitch_accel);

        /*
         * ---- Initialize filters on first reading ----
         * Seed the filters with the initial accelerometer angle
         * to avoid a large startup transient.
         */
        if (!filters_initialized)
        {
            Kalman_SetAngle(&kalman_roll, roll_accel);
            Kalman_SetAngle(&kalman_pitch, pitch_accel);
            CompFilter_SetAngle(&comp_roll, roll_accel);
            CompFilter_SetAngle(&comp_pitch, pitch_accel);
            filters_initialized = true;

            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
            continue; /* Skip output for the first sample */
        }

        /*
         * ---- Update Kalman filters ----
         *
         * Roll filter:
         *   - Gyro rate: X-axis gyroscope reading (deg/s)
         *   - Measurement: roll angle from accelerometer (deg)
         *
         * Pitch filter:
         *   - Gyro rate: Y-axis gyroscope reading (deg/s)
         *   - Measurement: pitch angle from accelerometer (deg)
         *
         * The Kalman filter internally:
         *   1. Predicts the new angle using gyro data
         *   2. Corrects the prediction using the accelerometer angle
         *   3. Estimates and compensates for gyroscope bias
         */
        roll_kalman = Kalman_Update(&kalman_roll,
                                    mpu6050.gyro.x,
                                    roll_accel,
                                    SAMPLE_PERIOD_S);

        pitch_kalman = Kalman_Update(&kalman_pitch,
                                     mpu6050.gyro.y,
                                     pitch_accel,
                                     SAMPLE_PERIOD_S);

        /*
         * ---- Update complementary filters ----
         * Run in parallel for comparison with the Kalman filter.
         */
        roll_comp = CompFilter_Update(&comp_roll,
                                      mpu6050.gyro.x,
                                      roll_accel,
                                      SAMPLE_PERIOD_S);

        pitch_comp = CompFilter_Update(&comp_pitch,
                                       mpu6050.gyro.y,
                                       pitch_accel,
                                       SAMPLE_PERIOD_S);

        /* ---- Output data over UART ---- */
        if (output_enabled)
        {
            /*
             * Output at a reduced rate to avoid UART buffer overflow.
             * At 115200 baud, we can transmit about 11520 chars/sec.
             * With ~80 chars per line, max ~144 lines/sec.
             * Output every 2nd sample = 100 Hz is safe.
             */
            if ((sample_counter % 2) == 0)
            {
                OutputData();
            }
        }

        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
    }
}

/* ========================================================================== */
/*                          ERROR HANDLER                                     */
/* ========================================================================== */

/**
 * @brief  This function is executed in case of error occurrence
 *
 * Turns on the red LED and enters an infinite loop.
 * In a production system, this would log the error and attempt recovery.
 */
void Error_Handler(void)
{
    __disable_irq();

    /* Turn on red LED to indicate fatal error */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);

    while (1)
    {
        /* Stay here */
    }
}

/**
 * @brief  Reports the name of the source file and line number
 *         where the assert_param error has occurred (debug builds only)
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    snprintf(uart_tx_buf, UART_TX_BUF_SIZE,
             "[ASSERT] File: %s, Line: %lu\r\n", file, line);
    UART_Print(uart_tx_buf);
}
#endif /* USE_FULL_ASSERT */
