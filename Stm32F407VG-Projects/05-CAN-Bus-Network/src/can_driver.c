/**
 * @file    can_driver.c
 * @brief   CAN Bus Driver Implementation for STM32F407VG
 * @details Implements the CAN1 peripheral driver using the STM32 HAL library.
 *
 *          Hardware mapping:
 *            - CAN1_RX : PD0  (AF9)
 *            - CAN1_TX : PD1  (AF9)
 *
 *          Clock source:
 *            - APB1 = 42 MHz  (SYSCLK 168 MHz / 4)
 *
 *          Features:
 *            - Configurable baud rate (125K / 250K / 500K / 1M)
 *            - Hardware filter bank configuration (mask & list modes)
 *            - TX with automatic mailbox selection
 *            - RX via FIFO0 and FIFO1 interrupts
 *            - HAL callback integration
 *            - Error handling with bus-off auto-recovery (ABOM)
 *            - Runtime statistics collection
 *
 * @version 1.0
 * @date    2026
 */

/* ------------------------------------------------------------------ */
/*  Includes                                                          */
/* ------------------------------------------------------------------ */
#include "can_driver.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  Global Driver Instance                                            */
/* ------------------------------------------------------------------ */

/** Single global CAN driver instance (CAN1). */
CAN_Driver_t g_CanDriver;

/* ------------------------------------------------------------------ */
/*  Private Helper: look up prescaler from enum                       */
/* ------------------------------------------------------------------ */

/**
 * @brief  Map a CAN_BaudRate_t enum to the corresponding prescaler value.
 * @param  br  Baud rate enum.
 * @return Prescaler value or 0 on invalid input.
 */
static uint32_t BaudRate_ToPrescaler(CAN_BaudRate_t br)
{
    switch (br)
    {
    case CAN_BAUDRATE_125K:
        return CAN_BAUD_125K_PRESCALER;
    case CAN_BAUDRATE_250K:
        return CAN_BAUD_250K_PRESCALER;
    case CAN_BAUDRATE_500K:
        return CAN_BAUD_500K_PRESCALER;
    case CAN_BAUDRATE_1M:
        return CAN_BAUD_1M_PRESCALER;
    default:
        return 0;
    }
}

/**
 * @brief  Map a CAN_OpMode_t to the HAL mode constant.
 * @param  opMode  Operating mode enum.
 * @return HAL CAN operating-mode constant.
 */
static uint32_t OpMode_ToHAL(CAN_OpMode_t opMode)
{
    switch (opMode)
    {
    case CAN_OPMODE_LOOPBACK:
        return CAN_MODE_LOOPBACK;
    case CAN_OPMODE_SILENT:
        return CAN_MODE_SILENT;
    case CAN_OPMODE_LOOPBACK_SILENT:
        return CAN_MODE_SILENT_LOOPBACK;
    case CAN_OPMODE_NORMAL:
    default:
        return CAN_MODE_NORMAL;
    }
}

/* ------------------------------------------------------------------ */
/*  HAL MSP Callbacks  (called from HAL_CAN_Init / HAL_CAN_DeInit)    */
/* ------------------------------------------------------------------ */

/**
 * @brief  CAN MSP Initialisation.
 *         Enables clocks, configures GPIO pins PD0/PD1 for CAN1 AF9,
 *         and enables NVIC interrupts.
 * @param  hcan  HAL CAN handle.
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        /* ---- Enable peripheral clocks ---- */
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* ---- Configure PD0 = CAN1_RX, PD1 = CAN1_TX ---- */
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &gpio);

        /* ---- Enable NVIC interrupts for CAN1 ---- */
        HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);

        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

        HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

        HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
    }
}

/**
 * @brief  CAN MSP De-Initialisation.
 *         Disables clocks, resets GPIO pins, disables NVIC interrupts.
 * @param  hcan  HAL CAN handle.
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_DISABLE();

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);

        HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
        HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
    }
}

/* ------------------------------------------------------------------ */
/*  Public API: Initialisation / De-Initialisation                    */
/* ------------------------------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_Init(CAN_BaudRate_t baudRate, CAN_OpMode_t opMode)
{
    CAN_Driver_t *drv = &g_CanDriver;

    /* Validate prescaler look-up */
    uint32_t prescaler = BaudRate_ToPrescaler(baudRate);
    if (prescaler == 0)
        return CAN_DRV_PARAM;

    /* Clear the driver structure */
    memset(drv, 0, sizeof(CAN_Driver_t));
    drv->BaudRate = baudRate;
    drv->OpMode = opMode;

    /* ---- HAL CAN Init structure ---- */
    CAN_HandleTypeDef *hcan = &drv->hcan;
    hcan->Instance = CAN1;

    hcan->Init.Prescaler = prescaler;
    hcan->Init.Mode = OpMode_ToHAL(opMode);
    hcan->Init.SyncJumpWidth = CAN_DEFAULT_SJW;
    hcan->Init.TimeSeg1 = CAN_DEFAULT_BS1;
    hcan->Init.TimeSeg2 = CAN_DEFAULT_BS2;
    hcan->Init.TimeTriggeredMode = DISABLE;
    hcan->Init.AutoBusOff = ENABLE; /* ABOM: auto bus-off recovery */
    hcan->Init.AutoWakeUp = ENABLE;
    hcan->Init.AutoRetransmission = ENABLE;    /* NART = 0: auto retransmit   */
    hcan->Init.ReceiveFifoLocked = DISABLE;    /* Overwrite on FIFO full      */
    hcan->Init.TransmitFifoPriority = DISABLE; /* Mailbox priority by ID      */

    if (HAL_CAN_Init(hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    /* ---- Install a default accept-all filter on bank 0 / FIFO 0 ---- */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000; /* Mask = 0 => accept all */
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = CAN_MAX_FILTER_BANKS; /* All banks to CAN1 */

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
        return CAN_DRV_ERROR;

    /* ---- Start the CAN peripheral ---- */
    if (HAL_CAN_Start(hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    /* ---- Enable interrupt notifications ---- */
    HAL_CAN_ActivateNotification(hcan,
                                 CAN_IT_RX_FIFO0_MSG_PENDING |     /* FIFO 0 message pending      */
                                     CAN_IT_RX_FIFO1_MSG_PENDING | /* FIFO 1 message pending      */
                                     CAN_IT_TX_MAILBOX_EMPTY |     /* TX mailbox empty             */
                                     CAN_IT_RX_FIFO0_OVERRUN |     /* FIFO 0 overrun              */
                                     CAN_IT_RX_FIFO1_OVERRUN |     /* FIFO 1 overrun              */
                                     CAN_IT_ERROR_WARNING |        /* Error-warning threshold      */
                                     CAN_IT_ERROR_PASSIVE |        /* Error-passive state          */
                                     CAN_IT_BUSOFF |               /* Bus-off state                */
                                     CAN_IT_LAST_ERROR_CODE |      /* Last error code change       */
                                     CAN_IT_ERROR);                /* General error                */

    drv->Initialized = true;
    return CAN_DRV_OK;
}

CAN_DrvStatus_t CAN_Driver_DeInit(void)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;

    HAL_CAN_Stop(&drv->hcan);
    HAL_CAN_DeInit(&drv->hcan);

    drv->Initialized = false;
    return CAN_DRV_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API: Transmit                                              */
/* ------------------------------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_Send(const CAN_Message_t *msg)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;
    if (msg == NULL || msg->DLC > CAN_MAX_DATA_LEN)
        return CAN_DRV_PARAM;

    /* Build the HAL TX header */
    CAN_TxHeaderTypeDef txHeader;
    if (msg->IDE)
    {
        txHeader.IDE = CAN_ID_EXT;
        txHeader.ExtId = msg->ID;
        txHeader.StdId = 0;
    }
    else
    {
        txHeader.IDE = CAN_ID_STD;
        txHeader.StdId = msg->ID;
        txHeader.ExtId = 0;
    }
    txHeader.RTR = msg->RTR ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    txHeader.DLC = msg->DLC;
    txHeader.TransmitGlobalTime = DISABLE;

    /* Check for a free TX mailbox */
    uint32_t freeMbox = HAL_CAN_GetTxMailboxesFreeLevel(&drv->hcan);
    if (freeMbox == 0)
    {
        drv->Stats.TxErrors++;
        return CAN_DRV_BUSY;
    }

    /* Request transmission */
    uint32_t usedMailbox = 0;
    if (HAL_CAN_AddTxMessage(&drv->hcan, &txHeader, (uint8_t *)msg->Data, &usedMailbox) != HAL_OK)
    {
        drv->Stats.TxErrors++;
        return CAN_DRV_ERROR;
    }

    drv->Stats.TxCount++;
    return CAN_DRV_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API: Receive (polled)                                      */
/* ------------------------------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_Receive(CAN_Message_t *msg, uint8_t fifo)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;
    if (msg == NULL || fifo > 1)
        return CAN_DRV_PARAM;

    uint32_t rxFifo = (fifo == 0) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;

    /* Check if there is at least one pending message */
    if (HAL_CAN_GetRxFifoFillLevel(&drv->hcan, rxFifo) == 0)
        return CAN_DRV_ERROR;

    CAN_RxHeaderTypeDef rxHeader;
    if (HAL_CAN_GetRxMessage(&drv->hcan, rxFifo, &rxHeader, msg->Data) != HAL_OK)
    {
        drv->Stats.RxErrors++;
        return CAN_DRV_ERROR;
    }

    /* Populate the driver message structure */
    msg->IDE = (rxHeader.IDE == CAN_ID_EXT) ? 1 : 0;
    msg->ID = msg->IDE ? rxHeader.ExtId : rxHeader.StdId;
    msg->RTR = (rxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
    msg->DLC = (uint8_t)rxHeader.DLC;
    msg->Timestamp = rxHeader.Timestamp;
    msg->FilterMatchIndex = (uint8_t)rxHeader.FilterMatchIndex;

    drv->Stats.RxCount++;
    return CAN_DRV_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API: Filter Configuration                                  */
/* ------------------------------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_AddFilter(const CAN_FilterCfg_t *cfg)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;
    if (cfg == NULL || cfg->FilterBank >= CAN_MAX_FILTER_BANKS)
        return CAN_DRV_PARAM;

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = cfg->FilterBank;
    filter.FilterMode = (cfg->Mode == CAN_FILTER_MODE_LIST)
                            ? CAN_FILTERMODE_IDLIST
                            : CAN_FILTERMODE_IDMASK;
    filter.FilterScale = cfg->Scale
                             ? CAN_FILTERSCALE_32BIT
                             : CAN_FILTERSCALE_16BIT;

    /*
     * For 32-bit scale mask mode with standard IDs the ID must be shifted
     * into bits [31:21] of the filter register (STID[10:0] in high word).
     *
     * HAL splits the 32-bit value into FilterIdHigh (bits 31..16) and
     * FilterIdLow (bits 15..0).
     *
     * Quick mapping for standard 11-bit IDs:
     *   Register value = (StdId << 5)  -- shift into STID position
     *   FilterIdHigh   = (value >> 16) & 0xFFFF
     *   FilterIdLow    = value & 0xFFFF
     */
    if (filter.FilterScale == CAN_FILTERSCALE_32BIT)
    {
        uint32_t idReg = cfg->IdHigh << 5; /* Shift Standard ID */
        uint32_t maskReg = cfg->IdLow << 5;

        filter.FilterIdHigh = (idReg >> 16) & 0xFFFF;
        filter.FilterIdLow = (idReg) & 0xFFFF;
        filter.FilterMaskIdHigh = (maskReg >> 16) & 0xFFFF;
        filter.FilterMaskIdLow = (maskReg) & 0xFFFF;
    }
    else
    {
        /* 16-bit scale: ID and mask packed directly */
        filter.FilterIdHigh = (uint16_t)(cfg->IdHigh << 5);
        filter.FilterIdLow = (uint16_t)(cfg->IdLow << 5);
        filter.FilterMaskIdHigh = 0x0000;
        filter.FilterMaskIdLow = 0x0000;
    }

    filter.FilterFIFOAssignment = (cfg->FIFOAssignment == 1)
                                      ? CAN_FILTER_FIFO1
                                      : CAN_FILTER_FIFO0;
    filter.FilterActivation = cfg->Enabled ? ENABLE : DISABLE;
    filter.SlaveStartFilterBank = CAN_MAX_FILTER_BANKS;

    if (HAL_CAN_ConfigFilter(&drv->hcan, &filter) != HAL_OK)
        return CAN_DRV_ERROR;

    return CAN_DRV_OK;
}

CAN_DrvStatus_t CAN_Driver_SetAcceptAllFilter(uint8_t filterBank, uint8_t fifo)
{
    CAN_FilterCfg_t cfg = {0};
    cfg.FilterBank = filterBank;
    cfg.Mode = CAN_FILTER_MODE_MASK;
    cfg.Scale = 1; /* 32-bit */
    cfg.IdHigh = 0x000;
    cfg.IdLow = 0x000; /* Mask = 0 => accept everything */
    cfg.FIFOAssignment = fifo;
    cfg.Enabled = 1;

    return CAN_Driver_AddFilter(&cfg);
}

/* ------------------------------------------------------------------ */
/*  Public API: Mode & Baud Rate Changes                              */
/* ------------------------------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_SetMode(CAN_OpMode_t opMode)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;

    /* Stop, reconfigure, restart */
    HAL_CAN_Stop(&drv->hcan);

    drv->hcan.Init.Mode = OpMode_ToHAL(opMode);
    if (HAL_CAN_Init(&drv->hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    if (HAL_CAN_Start(&drv->hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    /* Re-enable notifications */
    HAL_CAN_ActivateNotification(&drv->hcan,
                                 CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_RX_FIFO1_MSG_PENDING |
                                     CAN_IT_TX_MAILBOX_EMPTY |
                                     CAN_IT_RX_FIFO0_OVERRUN |
                                     CAN_IT_RX_FIFO1_OVERRUN |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE |
                                     CAN_IT_ERROR);

    drv->OpMode = opMode;
    return CAN_DRV_OK;
}

CAN_DrvStatus_t CAN_Driver_EnableLoopback(void)
{
    return CAN_Driver_SetMode(CAN_OPMODE_LOOPBACK);
}

CAN_DrvStatus_t CAN_Driver_DisableLoopback(void)
{
    return CAN_Driver_SetMode(CAN_OPMODE_NORMAL);
}

CAN_DrvStatus_t CAN_Driver_SetBaudRate(CAN_BaudRate_t baudRate)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;

    uint32_t prescaler = BaudRate_ToPrescaler(baudRate);
    if (prescaler == 0)
        return CAN_DRV_PARAM;

    HAL_CAN_Stop(&drv->hcan);

    drv->hcan.Init.Prescaler = prescaler;
    if (HAL_CAN_Init(&drv->hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    if (HAL_CAN_Start(&drv->hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    /* Re-enable notifications */
    HAL_CAN_ActivateNotification(&drv->hcan,
                                 CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_RX_FIFO1_MSG_PENDING |
                                     CAN_IT_TX_MAILBOX_EMPTY |
                                     CAN_IT_RX_FIFO0_OVERRUN |
                                     CAN_IT_RX_FIFO1_OVERRUN |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE |
                                     CAN_IT_ERROR);

    drv->BaudRate = baudRate;
    return CAN_DRV_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API: Statistics & Error Counters                           */
/* ------------------------------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_GetStats(CAN_Stats_t *stats)
{
    if (stats == NULL)
        return CAN_DRV_PARAM;

    /* Update the live error counters before returning the snapshot */
    CAN_Driver_GetErrorCounters(NULL, NULL);

    *stats = g_CanDriver.Stats;
    return CAN_DRV_OK;
}

void CAN_Driver_ResetStats(void)
{
    memset(&g_CanDriver.Stats, 0, sizeof(CAN_Stats_t));
}

void CAN_Driver_GetErrorCounters(uint8_t *tec, uint8_t *rec)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return;

    /*
     * The ESR register contains the error counters:
     *   Bits [23:16] = REC (Receive Error Counter)
     *   Bits [31:24] = TEC (Transmit Error Counter)
     */
    uint32_t esr = drv->hcan.Instance->ESR;
    uint8_t t = (uint8_t)((esr >> 24) & 0xFF);
    uint8_t r = (uint8_t)((esr >> 16) & 0xFF);

    drv->Stats.TEC = t;
    drv->Stats.REC = r;

    /* Determine bus state from error counters */
    if (t >= 256)
        drv->Stats.BusState = 3; /* Bus-Off */
    else if (t >= 128 || r >= 128)
        drv->Stats.BusState = 2; /* Error-Passive */
    else if (t >= 96 || r >= 96)
        drv->Stats.BusState = 1; /* Error-Warning */
    else
        drv->Stats.BusState = 0; /* Error-Active (normal) */

    if (tec)
        *tec = t;
    if (rec)
        *rec = r;
}

CAN_DrvStatus_t CAN_Driver_RecoverBusOff(void)
{
    CAN_Driver_t *drv = &g_CanDriver;
    if (!drv->Initialized)
        return CAN_DRV_NOT_INIT;

    /*
     * With ABOM enabled the hardware will auto-recover.
     * If manual recovery is needed we can stop and restart the peripheral.
     */
    HAL_CAN_Stop(&drv->hcan);

    if (HAL_CAN_Start(&drv->hcan) != HAL_OK)
        return CAN_DRV_ERROR;

    /* Re-enable notifications */
    HAL_CAN_ActivateNotification(&drv->hcan,
                                 CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_RX_FIFO1_MSG_PENDING |
                                     CAN_IT_TX_MAILBOX_EMPTY |
                                     CAN_IT_RX_FIFO0_OVERRUN |
                                     CAN_IT_RX_FIFO1_OVERRUN |
                                     CAN_IT_ERROR_WARNING |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_LAST_ERROR_CODE |
                                     CAN_IT_ERROR);

    return CAN_DRV_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API: Callback Registration                                 */
/* ------------------------------------------------------------------ */

void CAN_Driver_RegisterRxCallback(void (*cb)(const CAN_Message_t *msg, uint8_t fifo))
{
    g_CanDriver.RxCallback = cb;
}

void CAN_Driver_RegisterErrorCallback(void (*cb)(uint32_t errorCode))
{
    g_CanDriver.ErrorCallback = cb;
}

/* ------------------------------------------------------------------ */
/*  Public API: String Helpers                                        */
/* ------------------------------------------------------------------ */

const char *CAN_Driver_BaudRateToString(CAN_BaudRate_t baudRate)
{
    switch (baudRate)
    {
    case CAN_BAUDRATE_125K:
        return "125 Kbit/s";
    case CAN_BAUDRATE_250K:
        return "250 Kbit/s";
    case CAN_BAUDRATE_500K:
        return "500 Kbit/s";
    case CAN_BAUDRATE_1M:
        return "1 Mbit/s";
    default:
        return "Unknown";
    }
}

const char *CAN_Driver_ModeToString(CAN_OpMode_t opMode)
{
    switch (opMode)
    {
    case CAN_OPMODE_NORMAL:
        return "NORMAL";
    case CAN_OPMODE_LOOPBACK:
        return "LOOPBACK";
    case CAN_OPMODE_SILENT:
        return "SILENT";
    case CAN_OPMODE_LOOPBACK_SILENT:
        return "LOOPBACK+SILENT";
    default:
        return "UNKNOWN";
    }
}

/* ================================================================== */
/*  HAL CAN Callbacks  (invoked from ISR context)                     */
/* ================================================================== */

/**
 * @brief  Internal helper: read a message from the given FIFO and
 *         invoke the user callback.
 * @param  hcan  HAL CAN handle.
 * @param  fifo  CAN_RX_FIFO0 or CAN_RX_FIFO1.
 * @param  fifoIndex  0 or 1 (passed to callback).
 */
static void CAN_ISR_ReceiveFromFifo(CAN_HandleTypeDef *hcan,
                                    uint32_t fifo,
                                    uint8_t fifoIndex)
{
    CAN_Driver_t *drv = &g_CanDriver;

    CAN_RxHeaderTypeDef rxHeader;
    CAN_Message_t msg;

    if (HAL_CAN_GetRxMessage(hcan, fifo, &rxHeader, msg.Data) != HAL_OK)
    {
        drv->Stats.RxErrors++;
        return;
    }

    /* Fill the driver message struct */
    msg.IDE = (rxHeader.IDE == CAN_ID_EXT) ? 1 : 0;
    msg.ID = msg.IDE ? rxHeader.ExtId : rxHeader.StdId;
    msg.RTR = (rxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
    msg.DLC = (uint8_t)rxHeader.DLC;
    msg.Timestamp = rxHeader.Timestamp;
    msg.FilterMatchIndex = (uint8_t)rxHeader.FilterMatchIndex;

    drv->Stats.RxCount++;

    /* Invoke user callback if registered */
    if (drv->RxCallback)
        drv->RxCallback(&msg, fifoIndex);
}

/* ---------- FIFO 0 message pending ---------- */

/**
 * @brief  HAL callback: message pending in FIFO 0.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_ISR_ReceiveFromFifo(hcan, CAN_RX_FIFO0, 0);
}

/* ---------- FIFO 1 message pending ---------- */

/**
 * @brief  HAL callback: message pending in FIFO 1.
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_ISR_ReceiveFromFifo(hcan, CAN_RX_FIFO1, 1);
}

/* ---------- FIFO overrun ---------- */

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    g_CanDriver.Stats.FifoOverruns++;
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    g_CanDriver.Stats.FifoOverruns++;
}

/* ---------- TX mailbox complete ---------- */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    /* TX success already counted in CAN_Driver_Send */
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
}

/* ---------- Error & status-change ---------- */

/**
 * @brief  HAL callback: error detected.
 *
 * Classifies the error from the Last Error Code (LEC) bits of the ESR
 * register and updates the appropriate statistics counters.
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Driver_t *drv = &g_CanDriver;
    uint32_t error = HAL_CAN_GetError(hcan);

    /* ---- Classify error flags ---- */
    if (error & HAL_CAN_ERROR_EWG)
    {
        drv->Stats.ErrorWarnings++;
    }
    if (error & HAL_CAN_ERROR_EPV)
    {
        drv->Stats.ErrorPassive++;
    }
    if (error & HAL_CAN_ERROR_BOF)
    {
        drv->Stats.BusOffCount++;
    }
    if (error & HAL_CAN_ERROR_STF)
    {
        drv->Stats.StuffErrors++;
    }
    if (error & HAL_CAN_ERROR_FOR)
    {
        drv->Stats.FormErrors++;
    }
    if (error & HAL_CAN_ERROR_ACK)
    {
        drv->Stats.AckErrors++;
    }
    if (error & HAL_CAN_ERROR_CRC)
    {
        drv->Stats.CrcErrors++;
    }
    if (error & HAL_CAN_ERROR_RX_FOV0)
    {
        drv->Stats.FifoOverruns++;
    }
    if (error & HAL_CAN_ERROR_RX_FOV1)
    {
        drv->Stats.FifoOverruns++;
    }

    /* Update error counters from the ESR register */
    CAN_Driver_GetErrorCounters(NULL, NULL);

    /* Invoke user error callback if registered */
    if (drv->ErrorCallback)
        drv->ErrorCallback(error);
}

/* ================================================================== */
/*  NVIC IRQ Handlers  (forward to HAL)                               */
/* ================================================================== */

/**
 * @brief CAN1 TX interrupt handler.
 */
void CAN1_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&g_CanDriver.hcan);
}

/**
 * @brief CAN1 RX FIFO 0 interrupt handler.
 */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&g_CanDriver.hcan);
}

/**
 * @brief CAN1 RX FIFO 1 interrupt handler.
 */
void CAN1_RX1_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&g_CanDriver.hcan);
}

/**
 * @brief CAN1 Status Change / Error interrupt handler.
 */
void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&g_CanDriver.hcan);
}
