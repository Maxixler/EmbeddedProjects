/**
 * @file    can_driver.h
 * @brief   CAN Bus Driver for STM32F407VG
 * @details Provides a complete CAN bus driver layer on top of the STM32 HAL.
 *          Supports CAN1 on PD0 (RX) / PD1 (TX), configurable baud rates,
 *          hardware filter management, interrupt-driven reception on FIFO0/FIFO1,
 *          mailbox-managed transmission, error handling with bus-off recovery,
 *          and runtime statistics collection.
 *
 * @version 1.0
 * @date    2026
 *
 * @note    CAN1 is clocked from APB1 at 42 MHz (SYSCLK 168 MHz / 4).
 *          All baud-rate presets assume this APB1 frequency.
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/*  Includes                                                          */
/* ------------------------------------------------------------------ */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Public Macros & Constants                                         */
/* ------------------------------------------------------------------ */

/** Maximum CAN data payload length in bytes (CAN 2.0). */
#define CAN_MAX_DATA_LEN 8U

/** Maximum number of hardware filter banks available to CAN1. */
#define CAN_MAX_FILTER_BANKS 14U

/** Maximum number of TX mailboxes in the bCAN peripheral. */
#define CAN_TX_MAILBOX_COUNT 3U

/* ------------------------------------------------------------------ */
/*  Baud Rate Presets  (APB1 = 42 MHz)                                */
/*                                                                    */
/*  Formula:                                                          */
/*    BaudRate = APB1_CLK / (Prescaler * (1 + BS1 + BS2))             */
/*    Sample point = (1 + BS1) / (1 + BS1 + BS2)                      */
/*                                                                    */
/*  All presets use BS1 = 11, BS2 = 2  =>  14 tq per bit              */
/*  Sample point = 12/14 = 85.7 %                                     */
/* ------------------------------------------------------------------ */

/** @defgroup CAN_BaudRate_Presets Baud Rate Prescaler Values
 *  @brief   Prescaler values for common CAN baud rates at 42 MHz APB1.
 *  @{
 */
#define CAN_BAUD_125K_PRESCALER 24U /**< 42 MHz / (24 * 14) = 125 Kbit/s  */
#define CAN_BAUD_250K_PRESCALER 12U /**< 42 MHz / (12 * 14) = 250 Kbit/s  */
#define CAN_BAUD_500K_PRESCALER 6U  /**< 42 MHz / ( 6 * 14) = 500 Kbit/s  */
#define CAN_BAUD_1M_PRESCALER 3U    /**< 42 MHz / ( 3 * 14) =   1 Mbit/s  */

/** Common bit-segment values shared by all presets. */
#define CAN_DEFAULT_BS1 CAN_BS1_11TQ
#define CAN_DEFAULT_BS2 CAN_BS2_2TQ
#define CAN_DEFAULT_SJW CAN_SJW_1TQ
    /** @} */

    /* ------------------------------------------------------------------ */
    /*  Enumerated Types                                                  */
    /* ------------------------------------------------------------------ */

    /**
     * @brief Supported CAN baud rates.
     */
    typedef enum
    {
        CAN_BAUDRATE_125K = 0, /**< 125 Kbit/s */
        CAN_BAUDRATE_250K = 1, /**< 250 Kbit/s */
        CAN_BAUDRATE_500K = 2, /**< 500 Kbit/s */
        CAN_BAUDRATE_1M = 3    /**<   1 Mbit/s */
    } CAN_BaudRate_t;

    /**
     * @brief CAN operating modes.
     */
    typedef enum
    {
        CAN_OPMODE_NORMAL = 0,         /**< Normal bus operation                 */
        CAN_OPMODE_LOOPBACK = 1,       /**< Internal loopback (no ext. HW req.)  */
        CAN_OPMODE_SILENT = 2,         /**< Silent / listen-only mode            */
        CAN_OPMODE_LOOPBACK_SILENT = 3 /**< Combined loopback + silent       */
    } CAN_OpMode_t;

    /**
     * @brief CAN filter mode selection.
     */
    typedef enum
    {
        CAN_FILTER_MODE_MASK = 0, /**< ID / Mask filtering                  */
        CAN_FILTER_MODE_LIST = 1  /**< Exact-match ID list filtering        */
    } CAN_FilterMode_t;

    /**
     * @brief CAN driver status / return codes.
     */
    typedef enum
    {
        CAN_DRV_OK = 0,       /**< Operation succeeded                  */
        CAN_DRV_ERROR = -1,   /**< Generic error                        */
        CAN_DRV_BUSY = -2,    /**< All TX mailboxes are full             */
        CAN_DRV_TIMEOUT = -3, /**< Timeout waiting for operation        */
        CAN_DRV_PARAM = -4,   /**< Invalid parameter                    */
        CAN_DRV_NOT_INIT = -5 /**< Driver not initialised               */
    } CAN_DrvStatus_t;

    /* ------------------------------------------------------------------ */
    /*  Data Structures                                                   */
    /* ------------------------------------------------------------------ */

    /**
     * @brief CAN message descriptor (TX and RX).
     */
    typedef struct
    {
        uint32_t ID;                    /**< Message identifier (11-bit or 29-bit) */
        uint8_t IDE;                    /**< 0 = Standard (11-bit), 1 = Extended (29-bit) */
        uint8_t RTR;                    /**< 0 = Data frame, 1 = Remote request frame */
        uint8_t DLC;                    /**< Data Length Code (0 .. 8)             */
        uint8_t Data[CAN_MAX_DATA_LEN]; /**< Payload bytes                */
        uint32_t Timestamp;             /**< Reception timestamp (from hardware)   */
        uint8_t FilterMatchIndex;       /**< Index of the filter that matched (RX only) */
    } CAN_Message_t;

    /**
     * @brief CAN hardware filter configuration.
     */
    typedef struct
    {
        uint8_t FilterBank;     /**< Filter bank number (0..13 for CAN1) */
        CAN_FilterMode_t Mode;  /**< MASK or LIST mode                   */
        uint8_t Scale;          /**< 0 = 16-bit scale, 1 = 32-bit scale  */
        uint32_t IdHigh;        /**< Filter ID (or first list entry)     */
        uint32_t IdLow;         /**< Mask    (or second list entry)      */
        uint8_t FIFOAssignment; /**< 0 = FIFO0, 1 = FIFO1               */
        uint8_t Enabled;        /**< 1 = filter active, 0 = inactive     */
    } CAN_FilterCfg_t;

    /**
     * @brief Runtime statistics collected by the driver.
     */
    typedef struct
    {
        uint32_t TxCount;         /**< Total messages transmitted            */
        uint32_t RxCount;         /**< Total messages received               */
        uint32_t TxErrors;        /**< Transmission error count              */
        uint32_t RxErrors;        /**< Reception error count                 */
        uint32_t BusOffCount;     /**< Number of bus-off events              */
        uint32_t ErrorWarnings;   /**< Error-warning state transitions       */
        uint32_t ErrorPassive;    /**< Error-passive state transitions       */
        uint32_t ArbitrationLost; /**< Arbitration lost events               */
        uint32_t StuffErrors;     /**< Bit-stuffing error count              */
        uint32_t FormErrors;      /**< Form error count                      */
        uint32_t AckErrors;       /**< Acknowledge error count               */
        uint32_t CrcErrors;       /**< CRC error count                       */
        uint32_t FifoOverruns;    /**< FIFO overrun events                   */
        uint8_t TEC;              /**< Last read Transmit Error Counter      */
        uint8_t REC;              /**< Last read Receive Error Counter       */
        uint8_t BusState;         /**< 0=active, 1=warning, 2=passive, 3=off */
    } CAN_Stats_t;

    /**
     * @brief CAN driver handle.
     *
     * Holds all state needed by the driver: HAL handle, operating mode,
     * baud rate selection, message callback, and statistics.
     */
    typedef struct
    {
        CAN_HandleTypeDef hcan;  /**< HAL CAN handle (CAN1)          */
        CAN_BaudRate_t BaudRate; /**< Current baud rate selection     */
        CAN_OpMode_t OpMode;     /**< Current operating mode          */
        bool Initialized;        /**< true after successful init      */
        CAN_Stats_t Stats;       /**< Runtime statistics              */

        /**
         * @brief User callback invoked from RX ISR when a message arrives.
         * @param msg  Pointer to the received message (copied from FIFO).
         * @param fifo FIFO number that sourced the message (0 or 1).
         *
         * Set to NULL if no callback is needed.
         */
        void (*RxCallback)(const CAN_Message_t *msg, uint8_t fifo);

        /**
         * @brief User callback invoked when an error is detected (from SCE ISR).
         * @param errorCode  Combined HAL error flags (see HAL_CAN_ERROR_xxx).
         */
        void (*ErrorCallback)(uint32_t errorCode);
    } CAN_Driver_t;

    /* ------------------------------------------------------------------ */
    /*  Global Driver Instance (extern)                                   */
    /* ------------------------------------------------------------------ */

    /**
     * @brief Global CAN driver instance used throughout the application.
     *
     * Defined in can_driver.c.  Users may access statistics and state
     * through this handle but should use the API functions for operations.
     */
    extern CAN_Driver_t g_CanDriver;

    /* ------------------------------------------------------------------ */
    /*  Public API Function Prototypes                                    */
    /* ------------------------------------------------------------------ */

    /**
     * @brief  Initialise the CAN1 peripheral and driver state.
     * @param  baudRate  Desired baud rate (see @ref CAN_BaudRate_t).
     * @param  opMode    Operating mode   (see @ref CAN_OpMode_t).
     * @retval CAN_DRV_OK on success, negative error code otherwise.
     *
     * Configures CAN1 on PD0 (RX) / PD1 (TX), sets bit timing for the
     * requested baud rate, enables FIFO0/FIFO1 receive interrupts, the
     * TX complete interrupt, and the status-change / error interrupt.
     * A default accept-all filter is installed on bank 0 / FIFO0.
     */
    CAN_DrvStatus_t CAN_Driver_Init(CAN_BaudRate_t baudRate, CAN_OpMode_t opMode);

    /**
     * @brief  De-initialise the CAN driver and release the peripheral.
     * @retval CAN_DRV_OK on success.
     */
    CAN_DrvStatus_t CAN_Driver_DeInit(void);

    /**
     * @brief  Transmit a CAN message.
     * @param  msg  Pointer to the message to send.
     * @retval CAN_DRV_OK if the message was placed in a TX mailbox,
     *         CAN_DRV_BUSY if all mailboxes are occupied,
     *         CAN_DRV_PARAM if the message pointer is NULL or DLC > 8.
     *
     * The function selects the first free TX mailbox, loads the header
     * and payload, and requests transmission.  The actual result is
     * reported asynchronously via the TX-complete interrupt.
     */
    CAN_DrvStatus_t CAN_Driver_Send(const CAN_Message_t *msg);

    /**
     * @brief  Poll-receive a message from the specified RX FIFO.
     * @param  msg   [out] Buffer to store the received message.
     * @param  fifo  FIFO number: 0 or 1.
     * @retval CAN_DRV_OK if a message was available and copied to @p msg,
     *         CAN_DRV_ERROR if the FIFO is empty.
     *
     * @note   In interrupt-driven mode the RxCallback is the preferred
     *         reception path.  This function is useful for polled designs
     *         or for draining the FIFO in the callback context.
     */
    CAN_DrvStatus_t CAN_Driver_Receive(CAN_Message_t *msg, uint8_t fifo);

    /**
     * @brief  Configure a hardware acceptance filter.
     * @param  cfg  Pointer to the filter configuration structure.
     * @retval CAN_DRV_OK on success, CAN_DRV_PARAM on invalid parameters.
     *
     * The driver enters filter-init mode, programs the requested bank,
     * and returns to active filtering.
     */
    CAN_DrvStatus_t CAN_Driver_AddFilter(const CAN_FilterCfg_t *cfg);

    /**
     * @brief  Install an accept-all filter on the given bank / FIFO.
     * @param  filterBank  Bank number (0..13).
     * @param  fifo        Target FIFO (0 or 1).
     * @retval CAN_DRV_OK on success.
     */
    CAN_DrvStatus_t CAN_Driver_SetAcceptAllFilter(uint8_t filterBank, uint8_t fifo);

    /**
     * @brief  Change the operating mode at run time.
     * @param  opMode  New operating mode.
     * @retval CAN_DRV_OK on success.
     *
     * The peripheral is briefly stopped, reconfigured, and restarted.
     */
    CAN_DrvStatus_t CAN_Driver_SetMode(CAN_OpMode_t opMode);

    /**
     * @brief  Enable internal loopback mode (convenience wrapper).
     * @retval CAN_DRV_OK on success.
     */
    CAN_DrvStatus_t CAN_Driver_EnableLoopback(void);

    /**
     * @brief  Return to normal operating mode (convenience wrapper).
     * @retval CAN_DRV_OK on success.
     */
    CAN_DrvStatus_t CAN_Driver_DisableLoopback(void);

    /**
     * @brief  Change the baud rate at run time.
     * @param  baudRate  New baud rate selection.
     * @retval CAN_DRV_OK on success.
     *
     * The peripheral is stopped, bit timing is reconfigured, and the
     * peripheral is restarted.
     */
    CAN_DrvStatus_t CAN_Driver_SetBaudRate(CAN_BaudRate_t baudRate);

    /**
     * @brief  Retrieve a snapshot of the current statistics.
     * @param  stats  [out] Destination structure.
     * @retval CAN_DRV_OK on success.
     */
    CAN_DrvStatus_t CAN_Driver_GetStats(CAN_Stats_t *stats);

    /**
     * @brief  Reset all statistics counters to zero.
     */
    void CAN_Driver_ResetStats(void);

    /**
     * @brief  Read the current Transmit/Receive Error Counters from the
     *         CAN peripheral and update the stats structure.
     * @param  tec  [out] Transmit Error Counter (may be NULL).
     * @param  rec  [out] Receive Error Counter  (may be NULL).
     */
    void CAN_Driver_GetErrorCounters(uint8_t *tec, uint8_t *rec);

    /**
     * @brief  Attempt manual bus-off recovery.
     * @retval CAN_DRV_OK if recovery was initiated.
     *
     * @note   If ABOM (Automatic Bus-Off Management) is enabled in the
     *         MCR register the hardware recovers automatically after
     *         128 occurrences of 11 consecutive recessive bits.
     */
    CAN_DrvStatus_t CAN_Driver_RecoverBusOff(void);

    /**
     * @brief  Register a message-received callback.
     * @param  cb  Function pointer (NULL to disable).
     */
    void CAN_Driver_RegisterRxCallback(void (*cb)(const CAN_Message_t *msg, uint8_t fifo));

    /**
     * @brief  Register an error callback.
     * @param  cb  Function pointer (NULL to disable).
     */
    void CAN_Driver_RegisterErrorCallback(void (*cb)(uint32_t errorCode));

    /**
     * @brief  Return a human-readable baud rate label.
     * @param  baudRate  Baud rate enum value.
     * @return Pointer to a constant string such as "500 Kbit/s".
     */
    const char *CAN_Driver_BaudRateToString(CAN_BaudRate_t baudRate);

    /**
     * @brief  Return a human-readable mode label.
     * @param  opMode  Operating mode enum value.
     * @return Pointer to a constant string such as "LOOPBACK".
     */
    const char *CAN_Driver_ModeToString(CAN_OpMode_t opMode);

#ifdef __cplusplus
}
#endif

#endif /* CAN_DRIVER_H */
