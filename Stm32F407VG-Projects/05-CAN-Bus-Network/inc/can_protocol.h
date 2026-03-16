/**
 * @file    can_protocol.h
 * @brief   Automotive CAN Protocol Layer - Message Definitions & Codec
 * @details Defines standard automotive CAN message IDs, data structures for
 *          common vehicle signals (engine, speed, temperature, brake, OBD-II),
 *          and prototypes for big-endian signal encoding/decoding functions.
 *
 *          Message ID map (Standard 11-bit):
 *            0x100  Engine RPM
 *            0x101  Vehicle Speed
 *            0x102  Temperatures (coolant, intake air, oil)
 *            0x200  Brake Status
 *            0x7DF  OBD-II Diagnostic Request  (broadcast)
 *            0x7E8  OBD-II Diagnostic Response  (ECU #1)
 *
 * @version 1.0
 * @date    2026
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ------------------------------------------------------------------ */
/*  Includes                                                          */
/* ------------------------------------------------------------------ */
#include "can_driver.h"
#include <stdint.h>
#include <stdbool.h>

/* ================================================================== */
/*  Automotive CAN Message IDs                                        */
/* ================================================================== */

/** @defgroup CAN_MSG_IDS  Standard Automotive Message IDs
 *  @{
 */
#define CAN_MSG_ID_ENGINE_RPM 0x100U    /**< Engine RPM data               */
#define CAN_MSG_ID_VEHICLE_SPEED 0x101U /**< Vehicle speed data            */
#define CAN_MSG_ID_TEMPERATURES 0x102U  /**< Coolant, intake, oil temps    */
#define CAN_MSG_ID_BRAKE_STATUS 0x200U  /**< Brake pedal & pressure        */
#define CAN_MSG_ID_OBD2_REQUEST 0x7DFU  /**< OBD-II diagnostic request     */
#define CAN_MSG_ID_OBD2_RESPONSE 0x7E8U /**< OBD-II diagnostic response    */
/** @} */

/* ================================================================== */
/*  OBD-II Service Modes                                              */
/* ================================================================== */

/** @defgroup OBD2_MODES  OBD-II Service / Mode Identifiers
 *  @{
 */
#define OBD2_MODE_CURRENT_DATA 0x01U /**< Mode 01 - Show current data   */
#define OBD2_MODE_FREEZE_FRAME 0x02U /**< Mode 02 - Freeze frame data   */
#define OBD2_MODE_DTC_STORED 0x03U   /**< Mode 03 - Stored DTCs         */
#define OBD2_MODE_CLEAR_DTC 0x04U    /**< Mode 04 - Clear DTCs          */
#define OBD2_MODE_VEHICLE_INFO 0x09U /**< Mode 09 - Vehicle info        */
/** @} */

/* ================================================================== */
/*  OBD-II Parameter IDs (PIDs)                                       */
/* ================================================================== */

/** @defgroup OBD2_PIDS  Common OBD-II PIDs (Mode 01)
 *  @{
 */
#define OBD2_PID_ENGINE_RPM 0x0CU      /**< Engine RPM                    */
#define OBD2_PID_VEHICLE_SPEED 0x0DU   /**< Vehicle speed (km/h)          */
#define OBD2_PID_COOLANT_TEMP 0x05U    /**< Engine coolant temperature    */
#define OBD2_PID_INTAKE_AIR_TEMP 0x0FU /**< Intake air temperature        */
#define OBD2_PID_THROTTLE_POS 0x11U    /**< Throttle position             */
#define OBD2_PID_ENGINE_LOAD 0x04U     /**< Calculated engine load        */
#define OBD2_PID_FUEL_PRESSURE 0x0AU   /**< Fuel pressure                 */
#define OBD2_PID_MAF_FLOW 0x10U        /**< MAF air flow rate             */
/** @} */

/* ================================================================== */
/*  Signal Encoding Constants (factor / offset)                       */
/* ================================================================== */

/**
 * @defgroup SIGNAL_ENCODING  Signal Conversion Factors & Offsets
 *
 * Physical = (RawValue * Factor) + Offset
 * RawValue = (Physical - Offset) / Factor
 *
 * @{
 */

/* Engine RPM: raw value in 0.25 RPM increments (OBD-II standard) */
#define RPM_FACTOR 0.25f
#define RPM_OFFSET 0.0f

/* Vehicle speed: 1 km/h per count, no offset */
#define SPEED_FACTOR 1.0f
#define SPEED_OFFSET 0.0f

/* Temperature: 1 degree per count, offset = -40 (OBD-II standard) */
#define TEMP_FACTOR 1.0f
#define TEMP_OFFSET (-40.0f)

/* Brake pressure: 0.1 bar per count */
#define BRAKE_PRESSURE_FACTOR 0.1f
#define BRAKE_PRESSURE_OFFSET 0.0f

/* Throttle position: 100/255 % per count */
#define THROTTLE_FACTOR (100.0f / 255.0f)
#define THROTTLE_OFFSET 0.0f

    /** @} */

    /* ================================================================== */
    /*  Data Structures                                                   */
    /* ================================================================== */

    /**
     * @brief Engine RPM message data (ID 0x100).
     *
     * CAN frame layout (8 bytes, big-endian):
     *   Byte 0-1 : RPM raw value (uint16, factor = 0.25 RPM)
     *   Byte 2   : Engine load (0..100 %)
     *   Byte 3   : Throttle position (0..100 %)
     *   Byte 4-5 : Reserved
     *   Byte 6   : Engine status flags
     *   Byte 7   : Message counter (rolling 0..255)
     */
    typedef struct
    {
        float RPM;           /**< Engine speed in RPM           */
        uint8_t EngineLoad;  /**< Calculated engine load (%)    */
        uint8_t ThrottlePos; /**< Throttle position (%)         */
        uint8_t StatusFlags; /**< Bit-mapped status flags       */
        uint8_t Counter;     /**< Rolling message counter       */
    } EngineRPM_Data_t;

    /**
     * @brief Vehicle speed message data (ID 0x101).
     *
     * CAN frame layout (8 bytes, big-endian):
     *   Byte 0-1 : Vehicle speed (uint16, km/h * 100 for 0.01 resolution)
     *   Byte 2-3 : Wheel speed FL (uint16, km/h * 100)
     *   Byte 4-5 : Wheel speed FR (uint16, km/h * 100)
     *   Byte 6   : Gear position (0=Park, 1-6=Gears, 7=Reverse)
     *   Byte 7   : Message counter
     */
    typedef struct
    {
        float VehicleSpeed;   /**< Vehicle speed in km/h         */
        float WheelSpeedFL;   /**< Front-left wheel speed km/h   */
        float WheelSpeedFR;   /**< Front-right wheel speed km/h  */
        uint8_t GearPosition; /**< Current gear position         */
        uint8_t Counter;      /**< Rolling message counter       */
    } VehicleSpeed_Data_t;

    /**
     * @brief Temperature message data (ID 0x102).
     *
     * CAN frame layout (8 bytes, big-endian):
     *   Byte 0   : Coolant temperature (uint8, offset = -40, in C)
     *   Byte 1   : Intake air temperature (uint8, offset = -40, in C)
     *   Byte 2   : Oil temperature (uint8, offset = -40, in C)
     *   Byte 3   : Ambient temperature (uint8, offset = -40, in C)
     *   Byte 4   : Transmission temperature (uint8, offset = -40, in C)
     *   Byte 5   : Status / warning flags
     *   Byte 6   : Reserved
     *   Byte 7   : Message counter
     */
    typedef struct
    {
        int16_t CoolantTemp;      /**< Engine coolant temp (C)       */
        int16_t IntakeAirTemp;    /**< Intake air temp (C)           */
        int16_t OilTemp;          /**< Engine oil temp (C)           */
        int16_t AmbientTemp;      /**< Ambient / outside temp (C)    */
        int16_t TransmissionTemp; /**< Transmission fluid temp (C)   */
        uint8_t WarningFlags;     /**< Bit-mapped warning flags      */
        uint8_t Counter;          /**< Rolling message counter       */
    } Temperature_Data_t;

    /**
     * @brief Brake status message data (ID 0x200).
     *
     * CAN frame layout (8 bytes, big-endian):
     *   Byte 0   : Brake pedal pressed (0 or 1)
     *   Byte 1-2 : Brake pressure (uint16, 0.1 bar per count)
     *   Byte 3   : ABS active flag
     *   Byte 4   : ESP/ESC active flag
     *   Byte 5   : Parking brake engaged flag
     *   Byte 6   : Brake wear warning flag
     *   Byte 7   : Message counter
     */
    typedef struct
    {
        bool BrakePedalPressed; /**< Brake pedal state             */
        float BrakePressure;    /**< Brake line pressure (bar)     */
        bool ABSActive;         /**< ABS currently intervening     */
        bool ESPActive;         /**< ESP/ESC currently intervening */
        bool ParkingBrake;      /**< Parking brake engaged         */
        bool BrakeWearWarning;  /**< Brake pad wear warning        */
        uint8_t Counter;        /**< Rolling message counter       */
    } BrakeStatus_Data_t;

    /**
     * @brief OBD-II request message data (ID 0x7DF).
     *
     * ISO 15765-2 single-frame format:
     *   Byte 0   : Number of additional data bytes
     *   Byte 1   : Service / Mode (e.g. 0x01)
     *   Byte 2   : PID (e.g. 0x0C for RPM)
     *   Byte 3-7 : Padding (0x00 or 0x55)
     */
    typedef struct
    {
        uint8_t ServiceMode; /**< OBD-II service mode           */
        uint8_t PID;         /**< Parameter ID                  */
    } OBD2_Request_t;

    /**
     * @brief OBD-II response message data (ID 0x7E8).
     *
     * ISO 15765-2 single-frame format:
     *   Byte 0   : Number of additional data bytes
     *   Byte 1   : Service mode + 0x40 (positive response)
     *   Byte 2   : PID echo
     *   Byte 3-6 : Data bytes (up to 4 bytes, PID-dependent)
     *   Byte 7   : Padding (0x00 or 0x55)
     */
    typedef struct
    {
        uint8_t ServiceMode; /**< Response mode (request + 0x40) */
        uint8_t PID;         /**< Echoed PID                     */
        uint8_t DataLength;  /**< Number of data bytes (1..4)    */
        uint8_t Data[4];     /**< Response data bytes            */
    } OBD2_Response_t;

    /* ================================================================== */
    /*  Public API: Message Builders (encode struct -> CAN frame)         */
    /* ================================================================== */

    /**
     * @brief  Build a CAN message from EngineRPM data.
     * @param  msg   [out] CAN message to populate.
     * @param  data  Pointer to the engine RPM data structure.
     */
    void CAN_Protocol_BuildEngineRPM(CAN_Message_t *msg, const EngineRPM_Data_t *data);

    /**
     * @brief  Build a CAN message from VehicleSpeed data.
     * @param  msg   [out] CAN message to populate.
     * @param  data  Pointer to the vehicle speed data structure.
     */
    void CAN_Protocol_BuildVehicleSpeed(CAN_Message_t *msg, const VehicleSpeed_Data_t *data);

    /**
     * @brief  Build a CAN message from Temperature data.
     * @param  msg   [out] CAN message to populate.
     * @param  data  Pointer to the temperature data structure.
     */
    void CAN_Protocol_BuildTemperatures(CAN_Message_t *msg, const Temperature_Data_t *data);

    /**
     * @brief  Build a CAN message from BrakeStatus data.
     * @param  msg   [out] CAN message to populate.
     * @param  data  Pointer to the brake status data structure.
     */
    void CAN_Protocol_BuildBrakeStatus(CAN_Message_t *msg, const BrakeStatus_Data_t *data);

    /**
     * @brief  Build an OBD-II request CAN message.
     * @param  msg   [out] CAN message to populate.
     * @param  req   Pointer to the OBD-II request structure.
     */
    void CAN_Protocol_BuildOBD2Request(CAN_Message_t *msg, const OBD2_Request_t *req);

    /**
     * @brief  Build an OBD-II response CAN message.
     * @param  msg   [out] CAN message to populate.
     * @param  resp  Pointer to the OBD-II response structure.
     */
    void CAN_Protocol_BuildOBD2Response(CAN_Message_t *msg, const OBD2_Response_t *resp);

    /* ================================================================== */
    /*  Public API: Message Parsers (CAN frame -> decode into struct)     */
    /* ================================================================== */

    /**
     * @brief  Parse a CAN message into EngineRPM data.
     * @param  msg   Pointer to the received CAN message.
     * @param  data  [out] Parsed engine RPM data.
     * @retval true  if the message ID matched and was parsed successfully.
     * @retval false if the message ID did not match.
     */
    bool CAN_Protocol_ParseEngineRPM(const CAN_Message_t *msg, EngineRPM_Data_t *data);

    /**
     * @brief  Parse a CAN message into VehicleSpeed data.
     * @param  msg   Pointer to the received CAN message.
     * @param  data  [out] Parsed vehicle speed data.
     * @retval true  on success, false if ID mismatch.
     */
    bool CAN_Protocol_ParseVehicleSpeed(const CAN_Message_t *msg, VehicleSpeed_Data_t *data);

    /**
     * @brief  Parse a CAN message into Temperature data.
     * @param  msg   Pointer to the received CAN message.
     * @param  data  [out] Parsed temperature data.
     * @retval true  on success, false if ID mismatch.
     */
    bool CAN_Protocol_ParseTemperatures(const CAN_Message_t *msg, Temperature_Data_t *data);

    /**
     * @brief  Parse a CAN message into BrakeStatus data.
     * @param  msg   Pointer to the received CAN message.
     * @param  data  [out] Parsed brake status data.
     * @retval true  on success, false if ID mismatch.
     */
    bool CAN_Protocol_ParseBrakeStatus(const CAN_Message_t *msg, BrakeStatus_Data_t *data);

    /**
     * @brief  Parse a CAN message into an OBD-II request.
     * @param  msg   Pointer to the received CAN message.
     * @param  req   [out] Parsed OBD-II request.
     * @retval true  on success, false if ID mismatch.
     */
    bool CAN_Protocol_ParseOBD2Request(const CAN_Message_t *msg, OBD2_Request_t *req);

    /**
     * @brief  Parse a CAN message into an OBD-II response.
     * @param  msg   Pointer to the received CAN message.
     * @param  resp  [out] Parsed OBD-II response.
     * @retval true  on success, false if ID mismatch.
     */
    bool CAN_Protocol_ParseOBD2Response(const CAN_Message_t *msg, OBD2_Response_t *resp);

    /* ================================================================== */
    /*  Public API: Generic Signal Encoding / Decoding                    */
    /* ================================================================== */

    /**
     * @brief  Encode a physical value into a raw integer using factor & offset.
     * @param  physical  Physical value (e.g. 1750.0 RPM).
     * @param  factor    Scaling factor.
     * @param  offset    Offset.
     * @return Raw integer value suitable for packing into the CAN frame.
     *
     * Formula: raw = (uint32_t)((physical - offset) / factor)
     */
    uint32_t CAN_Protocol_EncodeSignal(float physical, float factor, float offset);

    /**
     * @brief  Decode a raw integer value into a physical value.
     * @param  raw     Raw value from the CAN frame.
     * @param  factor  Scaling factor.
     * @param  offset  Offset.
     * @return Physical value (float).
     *
     * Formula: physical = (raw * factor) + offset
     */
    float CAN_Protocol_DecodeSignal(uint32_t raw, float factor, float offset);

    /**
     * @brief  Pack a 16-bit value into a byte array in big-endian order.
     * @param  buf    Destination buffer (at least 2 bytes).
     * @param  value  16-bit value to pack.
     */
    void CAN_Protocol_PackU16BE(uint8_t *buf, uint16_t value);

    /**
     * @brief  Unpack a 16-bit value from a byte array in big-endian order.
     * @param  buf  Source buffer (at least 2 bytes).
     * @return Unpacked 16-bit value.
     */
    uint16_t CAN_Protocol_UnpackU16BE(const uint8_t *buf);

    /**
     * @brief  Pack a 32-bit value into a byte array in big-endian order.
     * @param  buf    Destination buffer (at least 4 bytes).
     * @param  value  32-bit value to pack.
     */
    void CAN_Protocol_PackU32BE(uint8_t *buf, uint32_t value);

    /**
     * @brief  Unpack a 32-bit value from a byte array in big-endian order.
     * @param  buf  Source buffer (at least 4 bytes).
     * @return Unpacked 32-bit value.
     */
    uint32_t CAN_Protocol_UnpackU32BE(const uint8_t *buf);

    /**
     * @brief  Dispatch a received CAN message to the appropriate parser
     *         based on its ID.
     * @param  msg  Pointer to the received CAN message.
     * @note   This is a convenience function that calls the individual
     *         parsers and can be extended with a user callback table.
     */
    void CAN_Protocol_DispatchMessage(const CAN_Message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
