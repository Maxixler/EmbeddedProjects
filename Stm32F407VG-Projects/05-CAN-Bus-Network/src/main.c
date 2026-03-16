/**
 * @file    main.c
 * @brief   STM32F407VG CAN Bus Network - Main Application
 * @details Demonstrates CAN bus communication with three operating modes:
 *
 *          1. LOOPBACK - Internal loopback test (no external hardware needed)
 *          2. TX       - Periodically transmits automotive CAN messages
 *          3. RX       - Receives and displays CAN messages
 *
 *          UART2 (PA2/PA3 at 115200 baud) provides a command-line monitor
 *          interface for runtime control and diagnostics.
 *
 *          Supported UART commands:
 *            MODE LOOPBACK | TX | RX   - Switch operating mode
 *            SEND id dlc hexdata       - Transmit a CAN message
 *            FILTER bank id mask type  - Configure acceptance filter
 *            STATS                     - Display communication statistics
 *            ERRORS                    - Display error counters
 *            BAUD 125|250|500|1000     - Change baud rate
 *
 *          LED indicators (STM32F4 Discovery):
 *            PD12 Green  = TX activity
 *            PD13 Orange = RX activity
 *            PD14 Red    = Error condition
 *            PD15 Blue   = Heartbeat (system running)
 *
 * @version 1.0
 * @date    2026
 *
 * Hardware:
 *   - CAN1_RX : PD0  (AF9)
 *   - CAN1_TX : PD1  (AF9)
 *   - USART2_TX : PA2
 *   - USART2_RX : PA3
 *   - LEDs : PD12, PD13, PD14, PD15
 *   - User button : PA0
 */

/* ------------------------------------------------------------------ */
/*  Includes                                                          */
/* ------------------------------------------------------------------ */
#include "stm32f4xx_hal.h"
#include "can_driver.h"
#include "can_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>

/* ------------------------------------------------------------------ */
/*  Pin / Peripheral Definitions                                      */
/* ------------------------------------------------------------------ */

/* LED GPIO definitions (active high on STM32F4 Discovery) */
#define LED_TX_PIN GPIO_PIN_12  /* Green  - TX activity     */
#define LED_RX_PIN GPIO_PIN_13  /* Orange - RX activity     */
#define LED_ERR_PIN GPIO_PIN_14 /* Red    - Error           */
#define LED_HB_PIN GPIO_PIN_15  /* Blue   - Heartbeat       */
#define LED_GPIO_PORT GPIOD

/* User button */
#define USER_BTN_PIN GPIO_PIN_0
#define USER_BTN_PORT GPIOA

/* UART command buffer size */
#define UART_RX_BUF_SIZE 128
#define UART_TX_BUF_SIZE 512

/* Periodic TX interval in milliseconds */
#define TX_PERIOD_MS 500

/* LED blink duration for activity indication (ms) */
#define LED_BLINK_MS 50

/* Heartbeat toggle period (ms) */
#define HEARTBEAT_MS 500

/* ------------------------------------------------------------------ */
/*  Application Operating Modes                                       */
/* ------------------------------------------------------------------ */

typedef enum
{
    APP_MODE_LOOPBACK = 0, /**< Internal loopback test          */
    APP_MODE_TX = 1,       /**< Periodic transmitter            */
    APP_MODE_RX = 2        /**< Receiver / monitor              */
} AppMode_t;

/* ------------------------------------------------------------------ */
/*  Private Variables                                                 */
/* ------------------------------------------------------------------ */

/* HAL peripheral handles */
static UART_HandleTypeDef huart2;

/* Application state */
static AppMode_t g_AppMode = APP_MODE_LOOPBACK;
static volatile bool g_RxFlag = false;
static volatile CAN_Message_t g_LastRxMsg;

/* UART receive buffer (byte-by-byte interrupt reception) */
static uint8_t g_UartRxBuf[UART_RX_BUF_SIZE];
static uint16_t g_UartRxIdx = 0;
static uint8_t g_UartRxByte;
static volatile bool g_UartCmdReady = false;

/* printf redirect buffer */
static char g_PrintBuf[UART_TX_BUF_SIZE];

/* Timing variables */
static uint32_t g_LastTxTick = 0;
static uint32_t g_LedTxOff = 0;
static uint32_t g_LedRxOff = 0;
static uint32_t g_LedErrOff = 0;
static uint32_t g_LastHBTick = 0;

/* Rolling message counter for TX mode */
static uint8_t g_MsgCounter = 0;

/* ------------------------------------------------------------------ */
/*  Forward Declarations                                              */
/* ------------------------------------------------------------------ */

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART2_Init(void);
static void UART_Print(const char *str);
static void UART_Printf(const char *fmt, ...);
static void ProcessCommand(const char *cmd);
static void PrintBanner(void);
static void PrintHelp(void);
static void PrintStats(void);
static void PrintErrors(void);
static void PrintMessage(const char *prefix, const CAN_Message_t *msg);
static void CAN_RxHandler(const CAN_Message_t *msg, uint8_t fifo);
static void CAN_ErrorHandler(uint32_t errorCode);
static void Mode_Loopback_Process(void);
static void Mode_TX_Process(void);
static void Mode_RX_Process(void);
static void LED_Update(void);
static uint8_t HexCharToNibble(char c);
static bool ParseHexString(const char *str, uint8_t *buf, uint8_t maxLen, uint8_t *outLen);

/* ------------------------------------------------------------------ */
/*  Main Entry Point                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief  Application entry point.
 *
 * Initialises the system clock to 168 MHz, configures GPIO for LEDs,
 * sets up UART2 for debug output, initialises CAN1 at 500 Kbit/s in
 * loopback mode, and enters the main loop.
 */
int main(void)
{
    /* ---- HAL and system initialisation ---- */
    HAL_Init();
    SystemClock_Config();

    /* ---- Peripheral initialisation ---- */
    GPIO_Init();
    UART2_Init();

    /* ---- Print startup banner ---- */
    PrintBanner();

    /* ---- Initialise CAN driver at 500 Kbit/s in loopback mode ---- */
    CAN_DrvStatus_t status = CAN_Driver_Init(CAN_BAUDRATE_500K, CAN_OPMODE_LOOPBACK);
    if (status != CAN_DRV_OK)
    {
        UART_Printf("[ERROR] CAN init failed (code %d)\r\n", status);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ERR_PIN, GPIO_PIN_SET);
        while (1)
        { /* Halt on init failure */
        }
    }

    /* Register callbacks */
    CAN_Driver_RegisterRxCallback(CAN_RxHandler);
    CAN_Driver_RegisterErrorCallback(CAN_ErrorHandler);

    UART_Printf("[CAN] Initialised: %s @ %s\r\n",
                CAN_Driver_ModeToString(g_CanDriver.OpMode),
                CAN_Driver_BaudRateToString(g_CanDriver.BaudRate));
    UART_Print("[SYS] Type HELP for available commands\r\n\r\n");

    /* Start receiving UART bytes via interrupt */
    HAL_UART_Receive_IT(&huart2, &g_UartRxByte, 1);

    /* ---- Main loop ---- */
    while (1)
    {
        /* Process UART commands */
        if (g_UartCmdReady)
        {
            g_UartCmdReady = false;
            ProcessCommand((char *)g_UartRxBuf);
            g_UartRxIdx = 0;
            memset(g_UartRxBuf, 0, sizeof(g_UartRxBuf));
        }

        /* Process mode-specific logic */
        switch (g_AppMode)
        {
        case APP_MODE_LOOPBACK:
            Mode_Loopback_Process();
            break;
        case APP_MODE_TX:
            Mode_TX_Process();
            break;
        case APP_MODE_RX:
            Mode_RX_Process();
            break;
        }

        /* Update LED indicators */
        LED_Update();
    }
}

/* ================================================================== */
/*  System Clock Configuration  (168 MHz from 8 MHz HSE)              */
/* ================================================================== */

/**
 * @brief  Configure the system clock to 168 MHz.
 *
 *   HSE (8 MHz) -> PLL -> SYSCLK = 168 MHz
 *   AHB  = 168 MHz (/1)
 *   APB1 =  42 MHz (/4)  <-- CAN peripheral clock
 *   APB2 =  84 MHz (/2)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscInit = {0};
    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState = RCC_HSE_ON;
    oscInit.PLL.PLLState = RCC_PLL_ON;
    oscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscInit.PLL.PLLM = 8;             /* VCO input  = 8 MHz / 8 = 1 MHz     */
    oscInit.PLL.PLLN = 336;           /* VCO output = 1 MHz * 336 = 336 MHz  */
    oscInit.PLL.PLLP = RCC_PLLP_DIV2; /* SYSCLK = 336 / 2 = 168 MHz  */
    oscInit.PLL.PLLQ = 7;             /* USB OTG FS = 336 / 7 = 48 MHz      */

    if (HAL_RCC_OscConfig(&oscInit) != HAL_OK)
    {
        while (1)
            ; /* Clock configuration failure */
    }

    RCC_ClkInitTypeDef clkInit = {0};
    clkInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                        RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkInit.AHBCLKDivider = RCC_SYSCLK_DIV1; /* HCLK  = 168 MHz */
    clkInit.APB1CLKDivider = RCC_HCLK_DIV4;  /* APB1  =  42 MHz */
    clkInit.APB2CLKDivider = RCC_HCLK_DIV2;  /* APB2  =  84 MHz */

    if (HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_5) != HAL_OK)
    {
        while (1)
            ;
    }
}

/* ================================================================== */
/*  GPIO Initialisation  (LEDs + User Button)                         */
/* ================================================================== */

/**
 * @brief  Configure GPIO pins for LEDs (PD12-PD15) and user button (PA0).
 */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* LEDs: PD12, PD13, PD14, PD15 as push-pull outputs */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = LED_TX_PIN | LED_RX_PIN | LED_ERR_PIN | LED_HB_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &gpio);

    /* Turn all LEDs off initially */
    HAL_GPIO_WritePin(LED_GPIO_PORT,
                      LED_TX_PIN | LED_RX_PIN | LED_ERR_PIN | LED_HB_PIN,
                      GPIO_PIN_RESET);

    /* User button: PA0 as input (external pull-down on Discovery board) */
    gpio.Pin = USER_BTN_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_BTN_PORT, &gpio);
}

/* ================================================================== */
/*  UART2 Initialisation  (PA2 TX, PA3 RX, 115200 8N1)               */
/* ================================================================== */

/**
 * @brief  Configure USART2 for 115200 baud, 8N1 (debug console).
 */
static void UART2_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2 = USART2_TX, PA3 = USART2_RX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart2);

    /* Enable UART RX interrupt */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* ================================================================== */
/*  UART Utility Functions                                            */
/* ================================================================== */

/**
 * @brief  Send a null-terminated string over UART2.
 */
static void UART_Print(const char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief  Formatted print to UART2 (printf-style).
 */
static void UART_Printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(g_PrintBuf, sizeof(g_PrintBuf), fmt, args);
    va_end(args);
    UART_Print(g_PrintBuf);
}

/* ================================================================== */
/*  UART Receive Interrupt Handling                                   */
/* ================================================================== */

/**
 * @brief  USART2 IRQ handler - forwards to HAL.
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief  HAL UART RX complete callback.
 *
 * Accumulates characters into g_UartRxBuf until a carriage return or
 * newline is received, then sets the command-ready flag.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t ch = g_UartRxByte;

        if (ch == '\r' || ch == '\n')
        {
            if (g_UartRxIdx > 0)
            {
                g_UartRxBuf[g_UartRxIdx] = '\0';
                g_UartCmdReady = true;
            }
        }
        else if (ch == '\b' || ch == 0x7F) /* Backspace / DEL */
        {
            if (g_UartRxIdx > 0)
                g_UartRxIdx--;
        }
        else
        {
            if (g_UartRxIdx < UART_RX_BUF_SIZE - 1)
            {
                g_UartRxBuf[g_UartRxIdx++] = ch;
            }
        }

        /* Re-arm the single-byte receive interrupt */
        HAL_UART_Receive_IT(&huart2, &g_UartRxByte, 1);
    }
}

/* ================================================================== */
/*  CAN Callbacks                                                     */
/* ================================================================== */

/**
 * @brief  Called from CAN RX ISR when a message is received.
 *
 * Copies the message into a global buffer and sets a flag for the
 * main loop to process.
 */
static void CAN_RxHandler(const CAN_Message_t *msg, uint8_t fifo)
{
    (void)fifo;
    memcpy((void *)&g_LastRxMsg, msg, sizeof(CAN_Message_t));
    g_RxFlag = true;

    /* Blink the RX LED */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RX_PIN, GPIO_PIN_SET);
    g_LedRxOff = HAL_GetTick() + LED_BLINK_MS;
}

/**
 * @brief  Called from CAN error ISR.
 */
static void CAN_ErrorHandler(uint32_t errorCode)
{
    /* Light the error LED */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ERR_PIN, GPIO_PIN_SET);
    g_LedErrOff = HAL_GetTick() + LED_BLINK_MS * 10; /* Longer blink for errors */

    UART_Printf("[CAN ERR] Code=0x%08lX  TEC=%u  REC=%u\r\n",
                errorCode,
                g_CanDriver.Stats.TEC,
                g_CanDriver.Stats.REC);
}

/* ================================================================== */
/*  Command Processing                                                */
/* ================================================================== */

/**
 * @brief  Parse and execute a UART command.
 */
static void ProcessCommand(const char *cmd)
{
    /* Make a mutable upper-case copy for keyword matching */
    char upper[UART_RX_BUF_SIZE];
    strncpy(upper, cmd, sizeof(upper) - 1);
    upper[sizeof(upper) - 1] = '\0';
    for (int i = 0; upper[i]; i++)
        upper[i] = (char)toupper((unsigned char)upper[i]);

    UART_Printf("\r\n> %s\r\n", cmd);

    /* ---- MODE command ---- */
    if (strncmp(upper, "MODE", 4) == 0)
    {
        const char *modeStr = upper + 4;
        while (*modeStr == ' ')
            modeStr++;

        if (strncmp(modeStr, "LOOPBACK", 8) == 0)
        {
            CAN_Driver_SetMode(CAN_OPMODE_LOOPBACK);
            g_AppMode = APP_MODE_LOOPBACK;
            UART_Print("[CAN] Loopback mode activated\r\n");
        }
        else if (strncmp(modeStr, "TX", 2) == 0)
        {
            CAN_Driver_SetMode(CAN_OPMODE_NORMAL);
            g_AppMode = APP_MODE_TX;
            UART_Print("[CAN] TX mode activated\r\n");
        }
        else if (strncmp(modeStr, "RX", 2) == 0)
        {
            CAN_Driver_SetMode(CAN_OPMODE_NORMAL);
            g_AppMode = APP_MODE_RX;
            UART_Print("[CAN] RX mode activated\r\n");
        }
        else
        {
            UART_Print("[ERROR] Usage: MODE LOOPBACK | TX | RX\r\n");
        }
    }
    /* ---- SEND command ---- */
    else if (strncmp(upper, "SEND", 4) == 0)
    {
        /*
         * Format: SEND <hex_id> <dlc> <hex_data>
         * Example: SEND 100 8 DEADBEEFCAFEBABE
         */
        const char *p = cmd + 4;
        while (*p == ' ')
            p++;

        uint32_t id = (uint32_t)strtoul(p, (char **)&p, 16);
        while (*p == ' ')
            p++;
        uint8_t dlc = (uint8_t)strtoul(p, (char **)&p, 10);
        while (*p == ' ')
            p++;

        CAN_Message_t msg;
        memset(&msg, 0, sizeof(msg));
        msg.ID = id;
        msg.IDE = (id > 0x7FF) ? 1 : 0;
        msg.RTR = 0;
        msg.DLC = (dlc > 8) ? 8 : dlc;

        /* Parse hex data string */
        uint8_t dataLen = 0;
        ParseHexString(p, msg.Data, msg.DLC, &dataLen);

        CAN_DrvStatus_t st = CAN_Driver_Send(&msg);
        if (st == CAN_DRV_OK)
        {
            PrintMessage("[CAN TX]", &msg);
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_TX_PIN, GPIO_PIN_SET);
            g_LedTxOff = HAL_GetTick() + LED_BLINK_MS;
        }
        else
        {
            UART_Printf("[ERROR] CAN send failed (code %d)\r\n", st);
        }
    }
    /* ---- FILTER command ---- */
    else if (strncmp(upper, "FILTER", 6) == 0)
    {
        /*
         * Format: FILTER <bank> <id> <mask> <MASK|LIST>
         * Example: FILTER 0 100 7F0 MASK
         */
        const char *p = cmd + 6;
        while (*p == ' ')
            p++;

        uint8_t bank = (uint8_t)strtoul(p, (char **)&p, 10);
        while (*p == ' ')
            p++;
        uint32_t id = (uint32_t)strtoul(p, (char **)&p, 16);
        while (*p == ' ')
            p++;
        uint32_t mask = (uint32_t)strtoul(p, (char **)&p, 16);
        while (*p == ' ')
            p++;

        /* Check mode keyword in the uppercased copy */
        const char *modeP = upper + (p - cmd);
        CAN_FilterMode_t filterMode = CAN_FILTER_MODE_MASK;
        if (strncmp(modeP, "LIST", 4) == 0)
            filterMode = CAN_FILTER_MODE_LIST;

        CAN_FilterCfg_t cfg;
        cfg.FilterBank = bank;
        cfg.Mode = filterMode;
        cfg.Scale = 1; /* 32-bit */
        cfg.IdHigh = id;
        cfg.IdLow = mask;
        cfg.FIFOAssignment = 0;
        cfg.Enabled = 1;

        CAN_DrvStatus_t st = CAN_Driver_AddFilter(&cfg);
        if (st == CAN_DRV_OK)
        {
            UART_Printf("[CAN] Filter bank %u: ID=0x%03lX  Mask=0x%03lX  Mode=%s\r\n",
                        bank, id, mask,
                        (filterMode == CAN_FILTER_MODE_LIST) ? "LIST" : "MASK");
        }
        else
        {
            UART_Printf("[ERROR] Filter config failed (code %d)\r\n", st);
        }
    }
    /* ---- STATS command ---- */
    else if (strncmp(upper, "STATS", 5) == 0)
    {
        PrintStats();
    }
    /* ---- ERRORS command ---- */
    else if (strncmp(upper, "ERRORS", 6) == 0)
    {
        PrintErrors();
    }
    /* ---- BAUD command ---- */
    else if (strncmp(upper, "BAUD", 4) == 0)
    {
        const char *p = cmd + 4;
        while (*p == ' ')
            p++;
        uint32_t baud = (uint32_t)strtoul(p, NULL, 10);

        CAN_BaudRate_t br;
        switch (baud)
        {
        case 125:
            br = CAN_BAUDRATE_125K;
            break;
        case 250:
            br = CAN_BAUDRATE_250K;
            break;
        case 500:
            br = CAN_BAUDRATE_500K;
            break;
        case 1000:
            br = CAN_BAUDRATE_1M;
            break;
        default:
            UART_Print("[ERROR] Usage: BAUD 125 | 250 | 500 | 1000\r\n");
            return;
        }

        CAN_DrvStatus_t st = CAN_Driver_SetBaudRate(br);
        if (st == CAN_DRV_OK)
        {
            UART_Printf("[CAN] Baud rate changed to %s\r\n",
                        CAN_Driver_BaudRateToString(br));
        }
        else
        {
            UART_Printf("[ERROR] Baud rate change failed (code %d)\r\n", st);
        }
    }
    /* ---- HELP command ---- */
    else if (strncmp(upper, "HELP", 4) == 0)
    {
        PrintHelp();
    }
    else
    {
        UART_Print("[ERROR] Unknown command. Type HELP for usage.\r\n");
    }
}

/* ================================================================== */
/*  Mode Processing Functions                                         */
/* ================================================================== */

/**
 * @brief  Loopback mode processing.
 *
 * In loopback mode, received messages (sent by ourselves) are
 * printed to the UART console to verify driver operation without
 * external CAN hardware.
 */
static void Mode_Loopback_Process(void)
{
    if (g_RxFlag)
    {
        g_RxFlag = false;
        CAN_Message_t rxCopy;
        memcpy(&rxCopy, (const void *)&g_LastRxMsg, sizeof(CAN_Message_t));
        PrintMessage("[CAN RX]", &rxCopy);
    }
}

/**
 * @brief  TX mode processing.
 *
 * Periodically transmits sample automotive CAN messages:
 *   - Engine RPM  (ID 0x100) every TX_PERIOD_MS
 *   - Vehicle Speed (ID 0x101)
 *   - Temperatures (ID 0x102)
 *   - Brake Status (ID 0x200)
 *
 * Uses the protocol layer builders for correct encoding.
 */
static void Mode_TX_Process(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_LastTxTick) < TX_PERIOD_MS)
        return;

    g_LastTxTick = now;
    g_MsgCounter++;

    CAN_Message_t msg;

    /* ---- Engine RPM message ---- */
    EngineRPM_Data_t rpmData;
    rpmData.RPM = 1750.0f + (float)(g_MsgCounter % 50) * 25.0f;
    rpmData.EngineLoad = 45 + (g_MsgCounter % 20);
    rpmData.ThrottlePos = 30 + (g_MsgCounter % 30);
    rpmData.StatusFlags = 0x01; /* Engine running */
    rpmData.Counter = g_MsgCounter;

    CAN_Protocol_BuildEngineRPM(&msg, &rpmData);
    if (CAN_Driver_Send(&msg) == CAN_DRV_OK)
    {
        PrintMessage("[CAN TX]", &msg);
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_TX_PIN, GPIO_PIN_SET);
        g_LedTxOff = HAL_GetTick() + LED_BLINK_MS;
    }

    /* ---- Vehicle Speed message ---- */
    VehicleSpeed_Data_t speedData;
    speedData.VehicleSpeed = 60.0f + (float)(g_MsgCounter % 40);
    speedData.WheelSpeedFL = speedData.VehicleSpeed + 0.5f;
    speedData.WheelSpeedFR = speedData.VehicleSpeed - 0.3f;
    speedData.GearPosition = 3;
    speedData.Counter = g_MsgCounter;

    CAN_Protocol_BuildVehicleSpeed(&msg, &speedData);
    CAN_Driver_Send(&msg);

    /* ---- Temperatures message ---- */
    Temperature_Data_t tempData;
    tempData.CoolantTemp = 85;
    tempData.IntakeAirTemp = 35;
    tempData.OilTemp = 95;
    tempData.AmbientTemp = 22;
    tempData.TransmissionTemp = 78;
    tempData.WarningFlags = 0x00;
    tempData.Counter = g_MsgCounter;

    CAN_Protocol_BuildTemperatures(&msg, &tempData);
    CAN_Driver_Send(&msg);

    /* ---- Brake Status message ---- */
    BrakeStatus_Data_t brakeData;
    brakeData.BrakePedalPressed = false;
    brakeData.BrakePressure = 0.0f;
    brakeData.ABSActive = false;
    brakeData.ESPActive = false;
    brakeData.ParkingBrake = false;
    brakeData.BrakeWearWarning = false;
    brakeData.Counter = g_MsgCounter;

    CAN_Protocol_BuildBrakeStatus(&msg, &brakeData);
    CAN_Driver_Send(&msg);
}

/**
 * @brief  RX mode processing.
 *
 * Processes received messages and prints them to the UART console.
 * Also dispatches through the protocol layer for decoding.
 */
static void Mode_RX_Process(void)
{
    if (g_RxFlag)
    {
        g_RxFlag = false;
        CAN_Message_t rxCopy;
        memcpy(&rxCopy, (const void *)&g_LastRxMsg, sizeof(CAN_Message_t));

        PrintMessage("[CAN RX]", &rxCopy);

        /* Attempt protocol-level decoding */
        switch (rxCopy.ID)
        {
        case CAN_MSG_ID_ENGINE_RPM:
        {
            EngineRPM_Data_t rpmData;
            if (CAN_Protocol_ParseEngineRPM(&rxCopy, &rpmData))
            {
                UART_Printf("  -> RPM=%.1f  Load=%u%%  Throttle=%u%%\r\n",
                            (double)rpmData.RPM,
                            rpmData.EngineLoad,
                            rpmData.ThrottlePos);
            }
            break;
        }
        case CAN_MSG_ID_VEHICLE_SPEED:
        {
            VehicleSpeed_Data_t speedData;
            if (CAN_Protocol_ParseVehicleSpeed(&rxCopy, &speedData))
            {
                UART_Printf("  -> Speed=%.2f km/h  Gear=%u\r\n",
                            (double)speedData.VehicleSpeed,
                            speedData.GearPosition);
            }
            break;
        }
        case CAN_MSG_ID_TEMPERATURES:
        {
            Temperature_Data_t tempData;
            if (CAN_Protocol_ParseTemperatures(&rxCopy, &tempData))
            {
                UART_Printf("  -> Coolant=%dC  Oil=%dC  Intake=%dC  Ambient=%dC\r\n",
                            tempData.CoolantTemp,
                            tempData.OilTemp,
                            tempData.IntakeAirTemp,
                            tempData.AmbientTemp);
            }
            break;
        }
        case CAN_MSG_ID_BRAKE_STATUS:
        {
            BrakeStatus_Data_t brakeData;
            if (CAN_Protocol_ParseBrakeStatus(&rxCopy, &brakeData))
            {
                UART_Printf("  -> Brake=%s  Pressure=%.1f bar  ABS=%s  ESP=%s\r\n",
                            brakeData.BrakePedalPressed ? "ON" : "OFF",
                            (double)brakeData.BrakePressure,
                            brakeData.ABSActive ? "ON" : "OFF",
                            brakeData.ESPActive ? "ON" : "OFF");
            }
            break;
        }
        default:
            break;
        }
    }
}

/* ================================================================== */
/*  LED Management                                                    */
/* ================================================================== */

/**
 * @brief  Update LED states (turn off activity LEDs after blink period,
 *         toggle heartbeat).
 */
static void LED_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* TX LED auto-off */
    if (g_LedTxOff && now >= g_LedTxOff)
    {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_TX_PIN, GPIO_PIN_RESET);
        g_LedTxOff = 0;
    }

    /* RX LED auto-off */
    if (g_LedRxOff && now >= g_LedRxOff)
    {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RX_PIN, GPIO_PIN_RESET);
        g_LedRxOff = 0;
    }

    /* Error LED auto-off */
    if (g_LedErrOff && now >= g_LedErrOff)
    {
        HAL_GPIO_WritePin(LED_GPIO_PORT, LED_ERR_PIN, GPIO_PIN_RESET);
        g_LedErrOff = 0;
    }

    /* Heartbeat: toggle blue LED periodically */
    if ((now - g_LastHBTick) >= HEARTBEAT_MS)
    {
        g_LastHBTick = now;
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_HB_PIN);
    }
}

/* ================================================================== */
/*  Print Helpers                                                     */
/* ================================================================== */

/**
 * @brief  Print the startup banner to UART.
 */
static void PrintBanner(void)
{
    UART_Print("\r\n");
    UART_Print("================================================\r\n");
    UART_Print("  STM32F407VG - CAN Bus Network Project\r\n");
    UART_Print("  CAN1 on PD0 (RX) / PD1 (TX)\r\n");
    UART_Print("  UART2 Debug Console @ 115200 baud\r\n");
    UART_Print("================================================\r\n");
    UART_Print("\r\n");
}

/**
 * @brief  Print the help / command reference to UART.
 */
static void PrintHelp(void)
{
    UART_Print("Available commands:\r\n");
    UART_Print("  MODE LOOPBACK     - Internal loopback test mode\r\n");
    UART_Print("  MODE TX           - Periodic transmitter mode\r\n");
    UART_Print("  MODE RX           - Receiver / monitor mode\r\n");
    UART_Print("  SEND id dlc data  - Send CAN message\r\n");
    UART_Print("                      Example: SEND 100 8 DEADBEEFCAFEBABE\r\n");
    UART_Print("  FILTER bank id mask type\r\n");
    UART_Print("                    - Configure acceptance filter\r\n");
    UART_Print("                      Example: FILTER 0 100 7F0 MASK\r\n");
    UART_Print("  STATS             - Show communication statistics\r\n");
    UART_Print("  ERRORS            - Show error counters\r\n");
    UART_Print("  BAUD rate         - Change baud rate\r\n");
    UART_Print("                      Values: 125, 250, 500, 1000 (Kbit/s)\r\n");
    UART_Print("  HELP              - Show this help message\r\n");
}

/**
 * @brief  Print communication statistics to UART.
 */
static void PrintStats(void)
{
    CAN_Stats_t stats;
    CAN_Driver_GetStats(&stats);

    UART_Print("\r\n--- CAN Communication Statistics ---\r\n");
    UART_Printf("  Mode       : %s\r\n", CAN_Driver_ModeToString(g_CanDriver.OpMode));
    UART_Printf("  Baud Rate  : %s\r\n", CAN_Driver_BaudRateToString(g_CanDriver.BaudRate));
    UART_Printf("  TX Count   : %lu\r\n", stats.TxCount);
    UART_Printf("  RX Count   : %lu\r\n", stats.RxCount);
    UART_Printf("  TX Errors  : %lu\r\n", stats.TxErrors);
    UART_Printf("  RX Errors  : %lu\r\n", stats.RxErrors);
    UART_Printf("  Bus-Off    : %lu\r\n", stats.BusOffCount);
    UART_Printf("  FIFO Overrn: %lu\r\n", stats.FifoOverruns);
    UART_Printf("  Arb. Lost  : %lu\r\n", stats.ArbitrationLost);

    const char *stateStr;
    switch (stats.BusState)
    {
    case 0:
        stateStr = "Error-Active (Normal)";
        break;
    case 1:
        stateStr = "Error-Warning";
        break;
    case 2:
        stateStr = "Error-Passive";
        break;
    case 3:
        stateStr = "Bus-Off";
        break;
    default:
        stateStr = "Unknown";
        break;
    }
    UART_Printf("  Bus State  : %s\r\n", stateStr);
    UART_Print("------------------------------------\r\n\r\n");
}

/**
 * @brief  Print detailed error counters to UART.
 */
static void PrintErrors(void)
{
    CAN_Stats_t stats;
    CAN_Driver_GetStats(&stats);

    UART_Print("\r\n--- CAN Error Counters ---\r\n");
    UART_Printf("  TEC (Transmit Error Counter) : %u\r\n", stats.TEC);
    UART_Printf("  REC (Receive Error Counter)  : %u\r\n", stats.REC);
    UART_Printf("  Stuff Errors    : %lu\r\n", stats.StuffErrors);
    UART_Printf("  Form Errors     : %lu\r\n", stats.FormErrors);
    UART_Printf("  ACK Errors      : %lu\r\n", stats.AckErrors);
    UART_Printf("  CRC Errors      : %lu\r\n", stats.CrcErrors);
    UART_Printf("  Error Warnings  : %lu\r\n", stats.ErrorWarnings);
    UART_Printf("  Error Passive   : %lu\r\n", stats.ErrorPassive);
    UART_Printf("  Bus-Off Events  : %lu\r\n", stats.BusOffCount);
    UART_Print("---------------------------\r\n\r\n");
}

/**
 * @brief  Print a CAN message in a readable format.
 *
 * Output: [prefix] ID=0x100 DLC=8 Data=DE AD BE EF CA FE BA BE
 */
static void PrintMessage(const char *prefix, const CAN_Message_t *msg)
{
    UART_Printf("%s ID=0x%03lX DLC=%u Data=",
                prefix, msg->ID, msg->DLC);

    for (uint8_t i = 0; i < msg->DLC; i++)
    {
        UART_Printf("%02X", msg->Data[i]);
        if (i < msg->DLC - 1)
            UART_Print(" ");
    }
    UART_Print("\r\n");
}

/* ================================================================== */
/*  Hex String Parsing Utility                                        */
/* ================================================================== */

/**
 * @brief  Convert a single hex character to its 4-bit value.
 */
static uint8_t HexCharToNibble(char c)
{
    if (c >= '0' && c <= '9')
        return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F')
        return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f')
        return (uint8_t)(c - 'a' + 10);
    return 0;
}

/**
 * @brief  Parse a hex string (e.g. "DEADBEEF") into a byte array.
 * @param  str     Input hex string (no 0x prefix, no spaces).
 * @param  buf     Output byte buffer.
 * @param  maxLen  Maximum number of bytes to parse.
 * @param  outLen  [out] Actual number of bytes parsed.
 * @retval true    if at least one byte was parsed.
 */
static bool ParseHexString(const char *str, uint8_t *buf, uint8_t maxLen, uint8_t *outLen)
{
    uint8_t len = 0;
    while (*str && *(str + 1) && len < maxLen)
    {
        if (!isxdigit((unsigned char)*str) || !isxdigit((unsigned char)*(str + 1)))
            break;

        buf[len] = (uint8_t)((HexCharToNibble(*str) << 4) | HexCharToNibble(*(str + 1)));
        str += 2;
        len++;
    }
    if (outLen)
        *outLen = len;
    return (len > 0);
}

/* ================================================================== */
/*  SysTick Handler  (required by HAL)                                */
/* ================================================================== */

/**
 * @brief  SysTick interrupt handler - increments HAL tick counter.
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
