/**
 * @file    main.c
 * @brief   PID Motor Speed Control - Main Application
 * @author  STM32F407VG Portfolio Project
 * @version 2.0
 * @date    2026
 *
 * @details Main application for DC motor speed control using PID algorithm.
 *
 *          System overview:
 *          - STM32F407VG running at 168MHz (HSE + PLL)
 *          - TIM1 CH1 (PA8): 20kHz PWM output to H-bridge ENA
 *          - TIM3 Encoder Mode (PA6, PA7): Quadrature encoder input (4x)
 *          - TIM6: PID control loop timer at 100Hz (10ms period)
 *          - USART2 (PA2/PA3): Command interface at 115200 baud
 *          - PB12/PB13: H-bridge direction control (IN1/IN2)
 *          - PD12-PD14: LED status indicators
 *          - PA0 (ADC1_IN0): Current sensing (ACS712)
 *
 *          UART Commands:
 *          - SET RPM <value>   : Set target speed
 *          - SET KP <value>    : Set proportional gain
 *          - SET KI <value>    : Set integral gain
 *          - SET KD <value>    : Set derivative gain
 *          - GET STATUS        : Print current state
 *          - STREAM ON/OFF     : Enable/disable data streaming
 *          - STOP              : Emergency stop
 *          - RESET PID         : Reset PID controller state
 *
 *          LED Indicators:
 *          - PD12 (Green):  Motor running
 *          - PD13 (Orange): Target RPM reached (within +/-5 RPM)
 *          - PD14 (Red):    Error / overcurrent / stall fault
 */

/* -------------------------------------------------------------------------- */
/*                              Includes                                      */
/* -------------------------------------------------------------------------- */

#include "stm32f4xx_hal.h"
#include "pid_controller.h"
#include "motor_driver.h"
#include "encoder.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <math.h>

/* -------------------------------------------------------------------------- */
/*                              Definitions                                   */
/* -------------------------------------------------------------------------- */

/* ---- System Clock ---- */
#define SYSTEM_CLOCK_HZ 168000000U /**< 168 MHz system clock        */

/* ---- UART ---- */
#define UART_BAUD_RATE 115200U
#define UART_RX_BUFFER_SIZE 128U /**< UART receive buffer size    */
#define UART_TX_BUFFER_SIZE 256U /**< UART transmit buffer size   */

/* ---- Control Loop ---- */
#define CONTROL_LOOP_FREQ_HZ 100U                            /**< PID loop frequency (Hz)     */
#define CONTROL_LOOP_DT (1.0f / (float)CONTROL_LOOP_FREQ_HZ) /* 0.01s */

/* ---- TIM6 Configuration (Control Loop Timer) ---- */
#define TIM6_PRESCALER 839U /**< 84MHz / 840 = 100kHz        */
#define TIM6_PERIOD 999U    /**< 100kHz / 1000 = 100Hz       */

/* ---- Safety Limits ---- */
#define MAX_RPM 3000.0f               /**< Maximum allowed RPM         */
#define MIN_RPM -3000.0f              /**< Minimum allowed RPM         */
#define RPM_TOLERANCE 5.0f            /**< Target reached tolerance     */
#define STALL_DETECTION_TIME_MS 2000U /**< Time before stall is declared (ms) */
#define STALL_DUTY_THRESHOLD 50.0f    /**< Duty above which stall is checked */
#define STALL_RPM_THRESHOLD 10.0f     /**< RPM below which stall is detected */
#define OVERCURRENT_THRESHOLD 4.0f    /**< Overcurrent threshold (A)   */
#define STREAM_INTERVAL_MS 50U        /**< Data streaming interval (ms) */

/* ---- LED Pins ---- */
#define LED_GREEN_PORT GPIOD
#define LED_GREEN_PIN GPIO_PIN_12
#define LED_ORANGE_PORT GPIOD
#define LED_ORANGE_PIN GPIO_PIN_13
#define LED_RED_PORT GPIOD
#define LED_RED_PIN GPIO_PIN_14
#define LED_BLUE_PORT GPIOD
#define LED_BLUE_PIN GPIO_PIN_15

/* ---- Current Sensing (ADC) ---- */
#define ADC_VREF 3.3f              /**< ADC reference voltage        */
#define ADC_RESOLUTION 4096.0f     /**< 12-bit ADC resolution        */
#define ACS712_SENSITIVITY 0.185f  /**< ACS712-05B: 185mV/A          */
#define ACS712_ZERO_CURRENT_V 2.5f /**< ACS712 zero-current output   */

/* ---- Encoder ---- */
#define ENCODER_CPR 400U /**< Encoder counts per revolution */

/* -------------------------------------------------------------------------- */
/*                          Private Type Definitions                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Application state enumeration
 */
typedef enum
{
    APP_STATE_INIT = 0,    /**< Initializing                         */
    APP_STATE_IDLE = 1,    /**< Waiting for commands, motor stopped  */
    APP_STATE_RUNNING = 2, /**< PID control active, motor running    */
    APP_STATE_FAULT = 3    /**< Fault condition, motor stopped       */
} App_State_t;

/**
 * @brief Application context structure
 */
typedef struct
{
    /* ---- Module Handles ---- */
    PID_Controller_t pid;     /**< PID controller instance       */
    Motor_Handle_t motor;     /**< Motor driver instance         */
    Encoder_Handle_t encoder; /**< Encoder interface instance    */

    /* ---- Application State ---- */
    App_State_t state;   /**< Current application state     */
    float target_rpm;    /**< Target speed setpoint (RPM)   */
    float current_rpm;   /**< Measured speed (RPM)          */
    float current_duty;  /**< Current PWM duty cycle (%)    */
    float current_error; /**< Current PID error (RPM)       */
    float motor_current; /**< Measured motor current (A)    */

    /* ---- Streaming ---- */
    bool stream_enabled;   /**< Data streaming active flag    */
    uint32_t stream_timer; /**< Stream interval counter (ms)  */

    /* ---- Safety ---- */
    uint32_t stall_timer; /**< Stall detection timer (ms)    */
    bool stall_detected;  /**< Stall condition flag          */
    bool overcurrent;     /**< Overcurrent condition flag    */

    /* ---- Timing ---- */
    volatile bool control_flag; /**< Set by TIM6 ISR, cleared by main loop */
    uint32_t uptime_ms;         /**< System uptime in milliseconds */
} App_Context_t;

/* -------------------------------------------------------------------------- */
/*                          Private Variables                                 */
/* -------------------------------------------------------------------------- */

/** HAL Peripheral Handles */
static TIM_HandleTypeDef htim1;   /**< TIM1: PWM generation              */
static TIM_HandleTypeDef htim3;   /**< TIM3: Encoder mode                */
static TIM_HandleTypeDef htim6;   /**< TIM6: Control loop timer          */
static UART_HandleTypeDef huart2; /**< USART2: Command interface         */
static ADC_HandleTypeDef hadc1;   /**< ADC1: Current sensing             */

/** Application context (global singleton) */
static App_Context_t app;

/** UART receive buffer */
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_byte;       /**< Single byte for interrupt receive */
static uint16_t uart_rx_index = 0; /**< Current position in rx buffer    */

/** UART transmit buffer */
static char uart_tx_buffer[UART_TX_BUFFER_SIZE];

/* -------------------------------------------------------------------------- */
/*                          Private Function Prototypes                       */
/* -------------------------------------------------------------------------- */

/* System initialization */
static void SystemClock_Config(void);
static void GPIO_Init(void);
static void TIM1_PWM_Init(void);
static void TIM3_Encoder_Init(void);
static void TIM6_ControlLoop_Init(void);
static void USART2_Init(void);
static void ADC1_Init(void);

/* Application logic */
static void App_Init(void);
static void App_ProcessControlLoop(void);
static void App_ProcessUARTCommand(const char *cmd);
static void App_UpdateLEDs(void);
static void App_CheckSafety(void);
static void App_StreamData(void);

/* UART helpers */
static void UART_SendString(const char *str);
static void UART_Printf(const char *fmt, ...);
static float UART_ParseFloat(const char *str);

/* ADC helpers */
static float ADC_ReadMotorCurrent(void);

/* -------------------------------------------------------------------------- */
/*                          Main Function                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Main application entry point
 *
 * Initializes all peripherals, configures the PID controller, motor driver,
 * and encoder, then enters the main loop where it processes the control
 * loop flag (set by TIM6 interrupt at 100Hz) and UART commands.
 */
int main(void)
{
    /* ---- HAL Initialization ---- */
    HAL_Init();

    /* ---- Configure System Clock to 168MHz ---- */
    SystemClock_Config();

    /* ---- Initialize Peripherals ---- */
    GPIO_Init();
    USART2_Init();
    ADC1_Init();
    TIM1_PWM_Init();
    TIM3_Encoder_Init();
    TIM6_ControlLoop_Init();

    /* ---- Initialize Application ---- */
    App_Init();

    /* ---- Print startup message ---- */
    UART_SendString("\r\n========================================\r\n");
    UART_SendString("  PID Motor Speed Control System\r\n");
    UART_SendString("  STM32F407VG @ 168MHz\r\n");
    UART_SendString("  Control Loop: 100Hz | PWM: 20kHz\r\n");
    UART_SendString("========================================\r\n");
    UART_SendString("Type 'GET STATUS' for current state.\r\n");
    UART_SendString("Type 'SET RPM <value>' to set target.\r\n\r\n");

    /* ---- Start UART receive interrupt ---- */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    /* ---- Main Loop ---- */
    while (1)
    {
        /*
         * The main loop is event-driven:
         * 1. control_flag is set by TIM6 ISR at 100Hz
         * 2. UART commands are processed in the UART RX callback
         * 3. LED updates and safety checks run every control cycle
         */

        if (app.control_flag)
        {
            app.control_flag = false;

            /* Run the PID control loop */
            App_ProcessControlLoop();

            /* Check safety conditions */
            App_CheckSafety();

            /* Update LED indicators */
            App_UpdateLEDs();

            /* Stream data if enabled */
            App_StreamData();
        }

        /* Motor soft-start update (called at SysTick rate, ~1ms) */
        /* Note: This is handled in HAL_SYSTICK_Callback for precise timing */
    }
}

/* -------------------------------------------------------------------------- */
/*                      System Clock Configuration                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Configure system clock to 168MHz using HSE + PLL
 *
 * Clock tree:
 *   HSE (8MHz) -> PLL (M=8, N=336, P=2) -> SYSCLK = 168MHz
 *   AHB  = 168 MHz
 *   APB1 = 42 MHz  (timer clock = 84 MHz due to x2 multiplier)
 *   APB2 = 84 MHz  (timer clock = 168 MHz due to x2 multiplier)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Enable Power Control clock */
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
        /* Oscillator configuration error - enter error handler */
        while (1)
        {
            HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
            HAL_Delay(100);
        }
    }

    /* Configure bus clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; /* AHB  = 168 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  /* APB1 = 42 MHz  */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  /* APB2 = 84 MHz  */

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
            HAL_Delay(100);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                          GPIO Initialization                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize GPIO pins for LEDs and motor direction control
 */
static void GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* ---- LED Pins (PD12-PD15): Output Push-Pull ---- */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* All LEDs off initially */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      GPIO_PIN_RESET);

    /* ---- Motor Direction Pins (PB12, PB13): Output Push-Pull ---- */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Direction pins low initially (motor coast) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/*                      TIM1 PWM Initialization                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize TIM1 Channel 1 for PWM output at 20kHz
 *
 * PA8 -> TIM1_CH1 -> H-Bridge ENA
 *
 * Timer clock: 168 MHz (APB2 timer clock)
 * Prescaler: 0 (no prescaling)
 * Period: 8399  -> 168MHz / 8400 = 20kHz
 * Dead-time: 32 ticks = ~190ns
 */
static void TIM1_PWM_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable TIM1 clock */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* Configure PA8 as TIM1_CH1 alternate function */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM1 Base configuration */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = MOTOR_PWM_PRESCALER;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = MOTOR_PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        while (1)
            ; /* Configuration error */
    }

    /* PWM Mode 1 configuration for Channel 1 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; /* Start with 0% duty */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        while (1)
            ;
    }

    /* Configure dead-time for H-bridge safety */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = MOTOR_DEADTIME_VALUE;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        while (1)
            ;
    }
}

/* -------------------------------------------------------------------------- */
/*                      TIM3 Encoder Mode Initialization                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize TIM3 in Encoder Mode 3 (quadrature, 4x resolution)
 *
 * PA6 -> TIM3_CH1 -> Encoder Phase A
 * PA7 -> TIM3_CH2 -> Encoder Phase B
 *
 * Encoder Mode 3: Count on both edges of both channels (4x resolution)
 * Counter range: 0..65535 (16-bit)
 * Input filter: 6 (noise rejection)
 */
static void TIM3_Encoder_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable TIM3 clock */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure PA6 and PA7 as TIM3_CH1 and TIM3_CH2 */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; /* Pull-up for noise immunity */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM3 Base configuration */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = ENCODER_TIMER_PERIOD; /* 65535 */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Encoder mode configuration */
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;  /* Mode 3: both channels */
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING; /* TI1: non-inverted */
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = ENCODER_INPUT_FILTER;    /* Noise filter */
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING; /* TI2: non-inverted */
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = ENCODER_INPUT_FILTER;

    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        while (1)
            ;
    }

    /* Enable TIM3 interrupt for overflow detection */
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* -------------------------------------------------------------------------- */
/*                      TIM6 Control Loop Timer Initialization                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize TIM6 as the PID control loop timer (100Hz)
 *
 * Timer clock: 84 MHz (APB1 timer clock = 42MHz * 2)
 * Prescaler: 839 -> 84MHz / 840 = 100kHz
 * Period: 999   -> 100kHz / 1000 = 100Hz
 *
 * Generates an update interrupt every 10ms to trigger the PID computation.
 */
static void TIM6_ControlLoop_Init(void)
{
    /* Enable TIM6 clock */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 Base configuration */
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = TIM6_PRESCALER;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = TIM6_PERIOD;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        while (1)
            ;
    }

    /* Configure NVIC for TIM6 interrupt */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    /* Start TIM6 with interrupt */
    HAL_TIM_Base_Start_IT(&htim6);
}

/* -------------------------------------------------------------------------- */
/*                          USART2 Initialization                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize USART2 for command interface (115200, 8N1)
 *
 * PA2 -> USART2_TX
 * PA3 -> USART2_RX
 */
static void USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable USART2 clock */
    __HAL_RCC_USART2_CLK_ENABLE();

    /* Configure PA2 (TX) and PA3 (RX) as USART2 alternate function */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 configuration */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = UART_BAUD_RATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        while (1)
            ;
    }

    /* Enable USART2 interrupt */
    HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* -------------------------------------------------------------------------- */
/*                          ADC1 Initialization                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize ADC1 Channel 0 (PA0) for motor current measurement
 *
 * ACS712 current sensor output connected to PA0.
 * 12-bit resolution, single conversion mode.
 */
static void ADC1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* Enable ADC1 clock */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* Configure PA0 as analog input */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 configuration */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        while (1)
            ;
    }

    /* Configure ADC channel 0 */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        while (1)
            ;
    }
}

/* -------------------------------------------------------------------------- */
/*                      Application Initialization                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize application modules and state
 */
static void App_Init(void)
{
    /* Clear application context */
    memset(&app, 0, sizeof(App_Context_t));

    /* ---- Initialize PID Controller ---- */
    PID_InitWithGains(&app.pid,
                      PID_DEFAULT_KP,   /* Kp = 1.0 */
                      PID_DEFAULT_KI,   /* Ki = 0.1 */
                      PID_DEFAULT_KD,   /* Kd = 0.05 */
                      CONTROL_LOOP_DT); /* dt = 0.01s */

    PID_SetOutputLimits(&app.pid, -100.0f, 100.0f);
    PID_EnableAntiWindup(&app.pid, PID_ANTIWINDUP_CLAMPING);
    PID_SetDerivativeFilter(&app.pid, PID_DEFAULT_DERIV_FILTER_ALPHA, true);
    PID_SetSetpoint(&app.pid, 0.0f);

    /* ---- Initialize Motor Driver ---- */
    Motor_Init(&app.motor, &htim1, TIM_CHANNEL_1);

    /* ---- Initialize Encoder ---- */
    Encoder_Init(&app.encoder, &htim3, ENCODER_CPR);

    /* ---- Set application state ---- */
    app.state = APP_STATE_IDLE;
    app.target_rpm = 0.0f;
    app.stream_enabled = false;
    app.stall_detected = false;
    app.overcurrent = false;
}

/* -------------------------------------------------------------------------- */
/*                      PID Control Loop Processing                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Process one iteration of the PID control loop
 *
 * Called at 100Hz from the main loop when control_flag is set by TIM6.
 *
 * Flow:
 *   1. Read encoder velocity (RPM)
 *   2. Read motor current (ADC)
 *   3. Compute PID output
 *   4. Apply output to motor driver
 *   5. Update application state
 */
static void App_ProcessControlLoop(void)
{
    /* ---- Step 1: Read encoder velocity ---- */
    app.current_rpm = Encoder_GetVelocityRPM(&app.encoder, CONTROL_LOOP_DT);

    /* ---- Step 2: Read motor current ---- */
    app.motor_current = ADC_ReadMotorCurrent();

    /* ---- Step 3: Compute PID output ---- */
    if (app.state == APP_STATE_RUNNING)
    {
        float pid_output = PID_Compute(&app.pid, app.current_rpm);
        app.current_duty = pid_output;
        app.current_error = PID_GetError(&app.pid);

        /* ---- Step 4: Apply to motor ---- */
        Motor_SetSpeed(&app.motor, pid_output);
    }
    else if (app.state == APP_STATE_IDLE)
    {
        /* Motor stopped, ensure PID output is zero */
        app.current_duty = 0.0f;
        app.current_error = 0.0f;
    }
    /* In FAULT state, motor is already stopped by safety handler */
}

/* -------------------------------------------------------------------------- */
/*                          UART Command Processing                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Process a received UART command string
 *
 * Supported commands:
 *   SET RPM <float>   - Set target speed in RPM
 *   SET KP <float>    - Set proportional gain
 *   SET KI <float>    - Set integral gain
 *   SET KD <float>    - Set derivative gain
 *   GET STATUS        - Print system status
 *   STREAM ON         - Enable data streaming
 *   STREAM OFF        - Disable data streaming
 *   STOP              - Emergency stop
 *   RESET PID         - Reset PID internal state
 */
static void App_ProcessUARTCommand(const char *cmd)
{
    /* Skip leading whitespace */
    while (*cmd == ' ' || *cmd == '\t')
        cmd++;

    /* ---- SET RPM ---- */
    if (strncmp(cmd, "SET RPM ", 8) == 0)
    {
        float rpm = UART_ParseFloat(cmd + 8);

        /* Clamp to safety limits */
        if (rpm > MAX_RPM)
            rpm = MAX_RPM;
        if (rpm < MIN_RPM)
            rpm = MIN_RPM;

        app.target_rpm = rpm;
        PID_SetSetpoint(&app.pid, rpm);

        /* If currently idle and RPM != 0, start running */
        if (app.state == APP_STATE_IDLE && fabsf(rpm) > 0.1f)
        {
            app.state = APP_STATE_RUNNING;
            PID_Reset(&app.pid);
            PID_SetSetpoint(&app.pid, rpm);
        }
        /* If RPM is zero, go to idle */
        if (fabsf(rpm) < 0.1f)
        {
            app.state = APP_STATE_IDLE;
            Motor_Coast(&app.motor);
            PID_Reset(&app.pid);
        }

        UART_Printf("OK: Target RPM = %.1f\r\n", rpm);
    }
    /* ---- SET KP ---- */
    else if (strncmp(cmd, "SET KP ", 7) == 0)
    {
        float kp = UART_ParseFloat(cmd + 7);
        PID_SetGains(&app.pid, kp, app.pid.Ki, app.pid.Kd);
        UART_Printf("OK: Kp = %.4f\r\n", kp);
    }
    /* ---- SET KI ---- */
    else if (strncmp(cmd, "SET KI ", 7) == 0)
    {
        float ki = UART_ParseFloat(cmd + 7);
        PID_SetGains(&app.pid, app.pid.Kp, ki, app.pid.Kd);
        UART_Printf("OK: Ki = %.4f\r\n", ki);
    }
    /* ---- SET KD ---- */
    else if (strncmp(cmd, "SET KD ", 7) == 0)
    {
        float kd = UART_ParseFloat(cmd + 7);
        PID_SetGains(&app.pid, app.pid.Kp, app.pid.Ki, kd);
        UART_Printf("OK: Kd = %.4f\r\n", kd);
    }
    /* ---- GET STATUS ---- */
    else if (strncmp(cmd, "GET STATUS", 10) == 0)
    {
        const char *state_str;
        switch (app.state)
        {
        case APP_STATE_INIT:
            state_str = "INIT";
            break;
        case APP_STATE_IDLE:
            state_str = "IDLE";
            break;
        case APP_STATE_RUNNING:
            state_str = "RUNNING";
            break;
        case APP_STATE_FAULT:
            state_str = "FAULT";
            break;
        default:
            state_str = "UNKNOWN";
            break;
        }

        UART_SendString("\r\n--- Motor Status ---\r\n");
        UART_Printf("Setpoint: %.1f RPM\r\n", app.target_rpm);
        UART_Printf("Current:  %.1f RPM\r\n", app.current_rpm);
        UART_Printf("Error:    %.1f RPM\r\n", app.current_error);
        UART_Printf("Duty:     %.1f%%\r\n", app.current_duty);
        UART_Printf("Current:  %.2f A\r\n", app.motor_current);
        UART_Printf("PID: Kp=%.4f Ki=%.4f Kd=%.4f\r\n",
                    app.pid.Kp, app.pid.Ki, app.pid.Kd);
        UART_Printf("State: %s\r\n", state_str);
        UART_Printf("Uptime: %lu ms\r\n", app.uptime_ms);
        UART_SendString("--------------------\r\n");
    }
    /* ---- STREAM ON ---- */
    else if (strncmp(cmd, "STREAM ON", 9) == 0)
    {
        app.stream_enabled = true;
        app.stream_timer = 0;
        UART_SendString("OK: Streaming ON\r\n");
        UART_SendString("time_ms,setpoint,rpm,duty,error,current\r\n");
    }
    /* ---- STREAM OFF ---- */
    else if (strncmp(cmd, "STREAM OFF", 10) == 0)
    {
        app.stream_enabled = false;
        UART_SendString("OK: Streaming OFF\r\n");
    }
    /* ---- STOP ---- */
    else if (strncmp(cmd, "STOP", 4) == 0)
    {
        Motor_EmergencyStop(&app.motor);
        PID_Reset(&app.pid);
        app.target_rpm = 0.0f;
        app.state = APP_STATE_IDLE;
        app.stream_enabled = false;
        UART_SendString("OK: EMERGENCY STOP\r\n");

        /* Clear fault after emergency stop */
        Motor_ClearFault(&app.motor);
    }
    /* ---- RESET PID ---- */
    else if (strncmp(cmd, "RESET PID", 9) == 0)
    {
        PID_Reset(&app.pid);
        UART_SendString("OK: PID Reset\r\n");
    }
    /* ---- Unknown Command ---- */
    else
    {
        UART_Printf("ERR: Unknown command: %s\r\n", cmd);
        UART_SendString("Commands: SET RPM/KP/KI/KD <val>, "
                        "GET STATUS, STREAM ON/OFF, STOP, RESET PID\r\n");
    }
}

/* -------------------------------------------------------------------------- */
/*                          LED Update                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Update LED indicators based on application state
 *
 * PD12 (Green):  ON when motor is running
 * PD13 (Orange): ON when target RPM is reached (within tolerance)
 * PD14 (Red):    ON when fault condition exists
 */
static void App_UpdateLEDs(void)
{
    /* Green LED: Motor running */
    if (app.state == APP_STATE_RUNNING)
    {
        HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    }

    /* Orange LED: Target reached */
    if (app.state == APP_STATE_RUNNING &&
        fabsf(app.current_error) < RPM_TOLERANCE)
    {
        HAL_GPIO_WritePin(LED_ORANGE_PORT, LED_ORANGE_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_ORANGE_PORT, LED_ORANGE_PIN, GPIO_PIN_RESET);
    }

    /* Red LED: Fault condition */
    if (app.state == APP_STATE_FAULT ||
        app.stall_detected || app.overcurrent)
    {
        HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    }
}

/* -------------------------------------------------------------------------- */
/*                          Safety Checks                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Check for safety conditions: overcurrent and stall detection
 *
 * Overcurrent: Motor current exceeds OVERCURRENT_THRESHOLD
 * Stall: Motor duty > STALL_DUTY_THRESHOLD but RPM < STALL_RPM_THRESHOLD
 *        for longer than STALL_DETECTION_TIME_MS
 */
static void App_CheckSafety(void)
{
    if (app.state != APP_STATE_RUNNING)
    {
        /* Reset safety timers when not running */
        app.stall_timer = 0;
        app.stall_detected = false;
        app.overcurrent = false;
        return;
    }

    /* ---- Overcurrent Detection ---- */
    if (app.motor_current > OVERCURRENT_THRESHOLD)
    {
        app.overcurrent = true;
        app.state = APP_STATE_FAULT;
        Motor_EmergencyStop(&app.motor);
        PID_Reset(&app.pid);
        UART_SendString("FAULT: Overcurrent detected!\r\n");
        return;
    }

    /* ---- Stall Detection ---- */
    if (fabsf(app.current_duty) > STALL_DUTY_THRESHOLD &&
        fabsf(app.current_rpm) < STALL_RPM_THRESHOLD)
    {
        /* Motor is being driven hard but not spinning */
        app.stall_timer += (uint32_t)(CONTROL_LOOP_DT * 1000.0f); /* Add 10ms */

        if (app.stall_timer >= STALL_DETECTION_TIME_MS)
        {
            app.stall_detected = true;
            app.state = APP_STATE_FAULT;
            Motor_EmergencyStop(&app.motor);
            PID_Reset(&app.pid);
            UART_SendString("FAULT: Motor stall detected!\r\n");
        }
    }
    else
    {
        /* Motor is spinning, reset stall timer */
        app.stall_timer = 0;
    }
}

/* -------------------------------------------------------------------------- */
/*                          Data Streaming                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Stream CSV data if streaming is enabled
 *
 * Output format: time_ms,setpoint,rpm,duty,error,current
 * Interval: STREAM_INTERVAL_MS (50ms)
 */
static void App_StreamData(void)
{
    if (!app.stream_enabled)
    {
        return;
    }

    app.stream_timer += (uint32_t)(CONTROL_LOOP_DT * 1000.0f);

    if (app.stream_timer >= STREAM_INTERVAL_MS)
    {
        app.stream_timer = 0;

        UART_Printf("%lu,%.1f,%.1f,%.1f,%.1f,%.2f\r\n",
                    app.uptime_ms,
                    app.target_rpm,
                    app.current_rpm,
                    app.current_duty,
                    app.current_error,
                    app.motor_current);
    }
}

/* -------------------------------------------------------------------------- */
/*                          UART Helper Functions                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Send a null-terminated string via UART (blocking)
 */
static void UART_SendString(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), 100);
}

/**
 * @brief  Formatted print via UART (like printf)
 *
 * Uses a static buffer. NOT reentrant - only call from main context.
 */
static void UART_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(uart_tx_buffer, UART_TX_BUFFER_SIZE, fmt, args);
    va_end(args);

    if (len > 0)
    {
        if (len > (int)UART_TX_BUFFER_SIZE - 1)
        {
            len = UART_TX_BUFFER_SIZE - 1;
        }
        HAL_UART_Transmit(&huart2, (uint8_t *)uart_tx_buffer, (uint16_t)len, 100);
    }
}

/**
 * @brief  Parse a floating-point number from a string
 *
 * Simple implementation using strtof. Returns 0.0 on parse error.
 */
static float UART_ParseFloat(const char *str)
{
    char *endptr;
    float value = strtof(str, &endptr);

    /* Check if parsing was successful */
    if (endptr == str)
    {
        return 0.0f;
    }
    return value;
}

/* -------------------------------------------------------------------------- */
/*                          ADC Helper Functions                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Read motor current from ACS712 sensor via ADC
 *
 * ACS712-05B characteristics:
 * - Zero current output: 2.5V (VCC/2)
 * - Sensitivity: 185 mV/A
 *
 * Current = (ADC_Voltage - 2.5V) / 0.185 V/A
 *
 * @return float  Motor current in Amperes (signed)
 */
static float ADC_ReadMotorCurrent(void)
{
    /* Start ADC conversion */
    HAL_ADC_Start(&hadc1);

    /* Wait for conversion (timeout 10ms) */
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        return 0.0f;
    }

    /* Read ADC value */
    uint32_t adc_raw = HAL_ADC_GetValue(&hadc1);

    /* Stop ADC */
    HAL_ADC_Stop(&hadc1);

    /* Convert to voltage */
    float voltage = ((float)adc_raw / ADC_RESOLUTION) * ADC_VREF;

    /* Convert to current (ACS712) */
    float current = (voltage - ACS712_ZERO_CURRENT_V) / ACS712_SENSITIVITY;

    return current;
}

/* -------------------------------------------------------------------------- */
/*                          Interrupt Handlers                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  TIM6 DAC interrupt handler (PID control loop timer)
 *
 * Fires at 100Hz. Sets the control_flag for the main loop to process.
 */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

/**
 * @brief  TIM3 interrupt handler (encoder overflow)
 *
 * Fires when TIM3 counter overflows or underflows.
 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/**
 * @brief  USART2 interrupt handler
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/* -------------------------------------------------------------------------- */
/*                          HAL Callbacks                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Timer period elapsed callback (called by HAL from IRQ handlers)
 *
 * Routes to the appropriate handler based on which timer generated the interrupt.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* PID control loop timer (100Hz) */
        app.control_flag = true;
        app.uptime_ms += (uint32_t)(CONTROL_LOOP_DT * 1000.0f);
    }
    else if (htim->Instance == TIM3)
    {
        /* Encoder overflow/underflow */
        Encoder_OverflowHandler(&app.encoder);
    }
}

/**
 * @brief  UART receive complete callback (single byte interrupt mode)
 *
 * Builds up a command string byte-by-byte until CR or LF is received,
 * then processes the complete command.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* Echo received character */
        HAL_UART_Transmit(&huart2, &uart_rx_byte, 1, 10);

        if (uart_rx_byte == '\r' || uart_rx_byte == '\n')
        {
            if (uart_rx_index > 0)
            {
                /* Null-terminate the received string */
                uart_rx_buffer[uart_rx_index] = '\0';

                /* Process the command */
                App_ProcessUARTCommand((char *)uart_rx_buffer);

                /* Reset buffer */
                uart_rx_index = 0;
                memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
            }
        }
        else
        {
            /* Store character in buffer */
            if (uart_rx_index < UART_RX_BUFFER_SIZE - 1)
            {
                uart_rx_buffer[uart_rx_index++] = uart_rx_byte;
            }
        }

        /* Re-enable receive interrupt for next byte */
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/**
 * @brief  SysTick hook for soft-start motor ramp
 *
 * Called every 1ms by the HAL SysTick interrupt.
 * Used for non-critical periodic tasks like soft-start ramping.
 */
void HAL_SYSTICK_Callback(void)
{
    /* Update soft-start ramp (if active) */
    Motor_SoftStartUpdate(&app.motor);
}

/**
 * @brief  System error handler
 *
 * Called when an unrecoverable error occurs. Blinks the red LED rapidly.
 */
void Error_Handler(void)
{
    /* Disable interrupts */
    __disable_irq();

    /* Stop motor immediately */
    Motor_EmergencyStop(&app.motor);

    /* Blink red LED */
    while (1)
    {
        HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
        /* Simple busy-wait delay since SysTick may not be running */
        for (volatile uint32_t i = 0; i < 1000000; i++)
            ;
    }
}

/**
 * @brief  Hard Fault Handler - catches critical processor faults
 */
void HardFault_Handler(void)
{
    /* Stop motor on hard fault */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(MOTOR_IN1_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);

    /* Turn on red LED */
    HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, GPIO_PIN_SET);

    while (1)
        ;
}

/* -------------------------------------------------------------------------- */
/*                          Required HAL Stubs                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  SysTick interrupt handler (required by HAL)
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

/**
 * @brief  NMI Handler
 */
void NMI_Handler(void)
{
    /* Nothing to do */
}

/**
 * @brief  Memory Management Fault Handler
 */
void MemManage_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief  Bus Fault Handler
 */
void BusFault_Handler(void)
{
    while (1)
        ;
}

/**
 * @brief  Usage Fault Handler
 */
void UsageFault_Handler(void)
{
    while (1)
        ;
}
