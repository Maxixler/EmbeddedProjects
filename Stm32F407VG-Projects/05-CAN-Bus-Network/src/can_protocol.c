/**
 * @file    can_protocol.c
 * @brief   Automotive CAN Protocol Layer - Encoder / Decoder Implementation
 * @details Implements big-endian data packing, signal encoding with
 *          factor/offset conversion, and message builders/parsers for
 *          standard automotive CAN messages and OBD-II diagnostics.
 *
 *          All multi-byte values are stored in big-endian (Motorola) byte
 *          order, which is the standard byte order for automotive CAN.
 *
 * @version 1.0
 * @date    2026
 */

/* ------------------------------------------------------------------ */
/*  Includes                                                          */
/* ------------------------------------------------------------------ */
#include "can_protocol.h"
#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Generic Signal Encoding / Decoding                                */
/* ================================================================== */

/**
 * @brief  Encode a physical value to a raw integer.
 *
 * Formula:  raw = (uint32_t)((physical - offset) / factor)
 *
 * Example:  1750 RPM with factor 0.25 => raw = (1750 - 0) / 0.25 = 7000
 */
uint32_t CAN_Protocol_EncodeSignal(float physical, float factor, float offset)
{
    if (fabsf(factor) < 1e-9f)
        return 0;

    float raw = (physical - offset) / factor;
    if (raw < 0.0f)
        return 0;

    return (uint32_t)(raw + 0.5f); /* Round to nearest integer */
}

/**
 * @brief  Decode a raw integer to a physical value.
 *
 * Formula:  physical = (raw * factor) + offset
 *
 * Example:  raw 7000 with factor 0.25 => physical = 7000 * 0.25 + 0 = 1750 RPM
 */
float CAN_Protocol_DecodeSignal(uint32_t raw, float factor, float offset)
{
    return ((float)raw * factor) + offset;
}

/* ================================================================== */
/*  Big-Endian Packing / Unpacking Helpers                            */
/* ================================================================== */

/**
 * @brief  Pack a uint16 into buf[0..1] in big-endian order.
 *
 *   buf[0] = MSB,  buf[1] = LSB
 */
void CAN_Protocol_PackU16BE(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)((value >> 8) & 0xFF);
    buf[1] = (uint8_t)(value & 0xFF);
}

/**
 * @brief  Unpack a uint16 from buf[0..1] in big-endian order.
 */
uint16_t CAN_Protocol_UnpackU16BE(const uint8_t *buf)
{
    return ((uint16_t)((uint16_t)buf[0] << 8)) | ((uint16_t)buf[1]);
}

/**
 * @brief  Pack a uint32 into buf[0..3] in big-endian order.
 *
 *   buf[0] = MSB,  buf[3] = LSB
 */
void CAN_Protocol_PackU32BE(uint8_t *buf, uint32_t value)
{
    buf[0] = (uint8_t)((value >> 24) & 0xFF);
    buf[1] = (uint8_t)((value >> 16) & 0xFF);
    buf[2] = (uint8_t)((value >> 8) & 0xFF);
    buf[3] = (uint8_t)(value & 0xFF);
}

/**
 * @brief  Unpack a uint32 from buf[0..3] in big-endian order.
 */
uint32_t CAN_Protocol_UnpackU32BE(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) |
           ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) |
           (uint32_t)buf[3];
}

/* ================================================================== */
/*  Message Builders  (struct -> CAN frame)                           */
/* ================================================================== */

/**
 * @brief  Build Engine RPM message (ID 0x100).
 *
 * Frame layout (8 bytes, big-endian):
 *   [0..1] RPM raw (uint16, factor 0.25)
 *   [2]    Engine load (%)
 *   [3]    Throttle position (%)
 *   [4..5] Reserved (0x00)
 *   [6]    Status flags
 *   [7]    Message counter
 */
void CAN_Protocol_BuildEngineRPM(CAN_Message_t *msg, const EngineRPM_Data_t *data)
{
    if (!msg || !data)
        return;

    memset(msg, 0, sizeof(CAN_Message_t));
    msg->ID = CAN_MSG_ID_ENGINE_RPM;
    msg->IDE = 0; /* Standard 11-bit ID */
    msg->RTR = 0; /* Data frame */
    msg->DLC = 8;

    /* Encode RPM: physical RPM -> raw with factor 0.25 */
    uint16_t rpmRaw = (uint16_t)CAN_Protocol_EncodeSignal(data->RPM, RPM_FACTOR, RPM_OFFSET);
    CAN_Protocol_PackU16BE(&msg->Data[0], rpmRaw);

    msg->Data[2] = data->EngineLoad;
    msg->Data[3] = data->ThrottlePos;
    msg->Data[4] = 0x00; /* Reserved */
    msg->Data[5] = 0x00; /* Reserved */
    msg->Data[6] = data->StatusFlags;
    msg->Data[7] = data->Counter;
}

/**
 * @brief  Build Vehicle Speed message (ID 0x101).
 *
 * Frame layout (8 bytes, big-endian):
 *   [0..1] Vehicle speed (uint16, km/h * 100)
 *   [2..3] Wheel speed FL (uint16, km/h * 100)
 *   [4..5] Wheel speed FR (uint16, km/h * 100)
 *   [6]    Gear position
 *   [7]    Message counter
 */
void CAN_Protocol_BuildVehicleSpeed(CAN_Message_t *msg, const VehicleSpeed_Data_t *data)
{
    if (!msg || !data)
        return;

    memset(msg, 0, sizeof(CAN_Message_t));
    msg->ID = CAN_MSG_ID_VEHICLE_SPEED;
    msg->IDE = 0;
    msg->RTR = 0;
    msg->DLC = 8;

    /* Speed encoded as km/h * 100 for 0.01 km/h resolution */
    uint16_t speedRaw = (uint16_t)(data->VehicleSpeed * 100.0f + 0.5f);
    uint16_t wheelFLRaw = (uint16_t)(data->WheelSpeedFL * 100.0f + 0.5f);
    uint16_t wheelFRRaw = (uint16_t)(data->WheelSpeedFR * 100.0f + 0.5f);

    CAN_Protocol_PackU16BE(&msg->Data[0], speedRaw);
    CAN_Protocol_PackU16BE(&msg->Data[2], wheelFLRaw);
    CAN_Protocol_PackU16BE(&msg->Data[4], wheelFRRaw);
    msg->Data[6] = data->GearPosition;
    msg->Data[7] = data->Counter;
}

/**
 * @brief  Build Temperatures message (ID 0x102).
 *
 * Frame layout (8 bytes):
 *   [0] Coolant temp       (uint8, offset -40)
 *   [1] Intake air temp    (uint8, offset -40)
 *   [2] Oil temp           (uint8, offset -40)
 *   [3] Ambient temp       (uint8, offset -40)
 *   [4] Transmission temp  (uint8, offset -40)
 *   [5] Warning flags
 *   [6] Reserved
 *   [7] Message counter
 */
void CAN_Protocol_BuildTemperatures(CAN_Message_t *msg, const Temperature_Data_t *data)
{
    if (!msg || !data)
        return;

    memset(msg, 0, sizeof(CAN_Message_t));
    msg->ID = CAN_MSG_ID_TEMPERATURES;
    msg->IDE = 0;
    msg->RTR = 0;
    msg->DLC = 8;

    /* Encode temperatures: physical + 40 (inverse of offset -40) */
    msg->Data[0] = (uint8_t)CAN_Protocol_EncodeSignal((float)data->CoolantTemp, TEMP_FACTOR, TEMP_OFFSET);
    msg->Data[1] = (uint8_t)CAN_Protocol_EncodeSignal((float)data->IntakeAirTemp, TEMP_FACTOR, TEMP_OFFSET);
    msg->Data[2] = (uint8_t)CAN_Protocol_EncodeSignal((float)data->OilTemp, TEMP_FACTOR, TEMP_OFFSET);
    msg->Data[3] = (uint8_t)CAN_Protocol_EncodeSignal((float)data->AmbientTemp, TEMP_FACTOR, TEMP_OFFSET);
    msg->Data[4] = (uint8_t)CAN_Protocol_EncodeSignal((float)data->TransmissionTemp, TEMP_FACTOR, TEMP_OFFSET);
    msg->Data[5] = data->WarningFlags;
    msg->Data[6] = 0x00; /* Reserved */
    msg->Data[7] = data->Counter;
}

/**
 * @brief  Build Brake Status message (ID 0x200).
 *
 * Frame layout (8 bytes):
 *   [0]    Brake pedal pressed (0/1)
 *   [1..2] Brake pressure (uint16, 0.1 bar per count, big-endian)
 *   [3]    ABS active (0/1)
 *   [4]    ESP active (0/1)
 *   [5]    Parking brake (0/1)
 *   [6]    Brake wear warning (0/1)
 *   [7]    Message counter
 */
void CAN_Protocol_BuildBrakeStatus(CAN_Message_t *msg, const BrakeStatus_Data_t *data)
{
    if (!msg || !data)
        return;

    memset(msg, 0, sizeof(CAN_Message_t));
    msg->ID = CAN_MSG_ID_BRAKE_STATUS;
    msg->IDE = 0;
    msg->RTR = 0;
    msg->DLC = 8;

    msg->Data[0] = data->BrakePedalPressed ? 0x01 : 0x00;

    /* Brake pressure: encode with factor 0.1 bar */
    uint16_t pressRaw = (uint16_t)CAN_Protocol_EncodeSignal(
        data->BrakePressure, BRAKE_PRESSURE_FACTOR, BRAKE_PRESSURE_OFFSET);
    CAN_Protocol_PackU16BE(&msg->Data[1], pressRaw);

    msg->Data[3] = data->ABSActive ? 0x01 : 0x00;
    msg->Data[4] = data->ESPActive ? 0x01 : 0x00;
    msg->Data[5] = data->ParkingBrake ? 0x01 : 0x00;
    msg->Data[6] = data->BrakeWearWarning ? 0x01 : 0x00;
    msg->Data[7] = data->Counter;
}

/**
 * @brief  Build an OBD-II diagnostic request message (ID 0x7DF).
 *
 * ISO 15765-2 single-frame:
 *   [0] = 0x02  (2 additional data bytes)
 *   [1] = Service mode
 *   [2] = PID
 *   [3..7] = 0x00 padding
 */
void CAN_Protocol_BuildOBD2Request(CAN_Message_t *msg, const OBD2_Request_t *req)
{
    if (!msg || !req)
        return;

    memset(msg, 0, sizeof(CAN_Message_t));
    msg->ID = CAN_MSG_ID_OBD2_REQUEST;
    msg->IDE = 0;
    msg->RTR = 0;
    msg->DLC = 8;

    msg->Data[0] = 0x02; /* Number of additional bytes */
    msg->Data[1] = req->ServiceMode;
    msg->Data[2] = req->PID;
    /* Bytes 3..7 remain 0x00 (padding) */
}

/**
 * @brief  Build an OBD-II diagnostic response message (ID 0x7E8).
 *
 * ISO 15765-2 single-frame:
 *   [0] = Number of additional data bytes (2 + DataLength)
 *   [1] = Service mode + 0x40
 *   [2] = PID echo
 *   [3..6] = Response data
 *   [7] = 0x00 padding
 */
void CAN_Protocol_BuildOBD2Response(CAN_Message_t *msg, const OBD2_Response_t *resp)
{
    if (!msg || !resp)
        return;

    memset(msg, 0, sizeof(CAN_Message_t));
    msg->ID = CAN_MSG_ID_OBD2_RESPONSE;
    msg->IDE = 0;
    msg->RTR = 0;
    msg->DLC = 8;

    uint8_t dataLen = (resp->DataLength > 4) ? 4 : resp->DataLength;

    msg->Data[0] = 2 + dataLen;       /* Byte count */
    msg->Data[1] = resp->ServiceMode; /* Already includes +0x40 */
    msg->Data[2] = resp->PID;

    for (uint8_t i = 0; i < dataLen; i++)
    {
        msg->Data[3 + i] = resp->Data[i];
    }
    /* Remaining bytes stay 0x00 */
}

/* ================================================================== */
/*  Message Parsers  (CAN frame -> struct)                            */
/* ================================================================== */

/**
 * @brief  Parse Engine RPM message (ID 0x100).
 */
bool CAN_Protocol_ParseEngineRPM(const CAN_Message_t *msg, EngineRPM_Data_t *data)
{
    if (!msg || !data)
        return false;
    if (msg->ID != CAN_MSG_ID_ENGINE_RPM)
        return false;
    if (msg->DLC < 8)
        return false;

    /* Decode RPM from big-endian uint16 with factor 0.25 */
    uint16_t rpmRaw = CAN_Protocol_UnpackU16BE(&msg->Data[0]);
    data->RPM = CAN_Protocol_DecodeSignal(rpmRaw, RPM_FACTOR, RPM_OFFSET);

    data->EngineLoad = msg->Data[2];
    data->ThrottlePos = msg->Data[3];
    data->StatusFlags = msg->Data[6];
    data->Counter = msg->Data[7];

    return true;
}

/**
 * @brief  Parse Vehicle Speed message (ID 0x101).
 */
bool CAN_Protocol_ParseVehicleSpeed(const CAN_Message_t *msg, VehicleSpeed_Data_t *data)
{
    if (!msg || !data)
        return false;
    if (msg->ID != CAN_MSG_ID_VEHICLE_SPEED)
        return false;
    if (msg->DLC < 8)
        return false;

    /* Speed is km/h * 100 -> divide by 100 */
    uint16_t speedRaw = CAN_Protocol_UnpackU16BE(&msg->Data[0]);
    uint16_t wheelFLRaw = CAN_Protocol_UnpackU16BE(&msg->Data[2]);
    uint16_t wheelFRRaw = CAN_Protocol_UnpackU16BE(&msg->Data[4]);

    data->VehicleSpeed = (float)speedRaw / 100.0f;
    data->WheelSpeedFL = (float)wheelFLRaw / 100.0f;
    data->WheelSpeedFR = (float)wheelFRRaw / 100.0f;
    data->GearPosition = msg->Data[6];
    data->Counter = msg->Data[7];

    return true;
}

/**
 * @brief  Parse Temperatures message (ID 0x102).
 */
bool CAN_Protocol_ParseTemperatures(const CAN_Message_t *msg, Temperature_Data_t *data)
{
    if (!msg || !data)
        return false;
    if (msg->ID != CAN_MSG_ID_TEMPERATURES)
        return false;
    if (msg->DLC < 8)
        return false;

    /* Decode temperatures: raw - 40 = physical (C) */
    data->CoolantTemp = (int16_t)(CAN_Protocol_DecodeSignal(msg->Data[0], TEMP_FACTOR, TEMP_OFFSET));
    data->IntakeAirTemp = (int16_t)(CAN_Protocol_DecodeSignal(msg->Data[1], TEMP_FACTOR, TEMP_OFFSET));
    data->OilTemp = (int16_t)(CAN_Protocol_DecodeSignal(msg->Data[2], TEMP_FACTOR, TEMP_OFFSET));
    data->AmbientTemp = (int16_t)(CAN_Protocol_DecodeSignal(msg->Data[3], TEMP_FACTOR, TEMP_OFFSET));
    data->TransmissionTemp = (int16_t)(CAN_Protocol_DecodeSignal(msg->Data[4], TEMP_FACTOR, TEMP_OFFSET));
    data->WarningFlags = msg->Data[5];
    data->Counter = msg->Data[7];

    return true;
}

/**
 * @brief  Parse Brake Status message (ID 0x200).
 */
bool CAN_Protocol_ParseBrakeStatus(const CAN_Message_t *msg, BrakeStatus_Data_t *data)
{
    if (!msg || !data)
        return false;
    if (msg->ID != CAN_MSG_ID_BRAKE_STATUS)
        return false;
    if (msg->DLC < 8)
        return false;

    data->BrakePedalPressed = (msg->Data[0] != 0);

    /* Brake pressure: raw * 0.1 bar */
    uint16_t pressRaw = CAN_Protocol_UnpackU16BE(&msg->Data[1]);
    data->BrakePressure = CAN_Protocol_DecodeSignal(pressRaw,
                                                    BRAKE_PRESSURE_FACTOR, BRAKE_PRESSURE_OFFSET);

    data->ABSActive = (msg->Data[3] != 0);
    data->ESPActive = (msg->Data[4] != 0);
    data->ParkingBrake = (msg->Data[5] != 0);
    data->BrakeWearWarning = (msg->Data[6] != 0);
    data->Counter = msg->Data[7];

    return true;
}

/**
 * @brief  Parse OBD-II diagnostic request (ID 0x7DF).
 */
bool CAN_Protocol_ParseOBD2Request(const CAN_Message_t *msg, OBD2_Request_t *req)
{
    if (!msg || !req)
        return false;
    if (msg->ID != CAN_MSG_ID_OBD2_REQUEST)
        return false;
    if (msg->DLC < 3)
        return false;

    req->ServiceMode = msg->Data[1];
    req->PID = msg->Data[2];

    return true;
}

/**
 * @brief  Parse OBD-II diagnostic response (ID 0x7E8).
 */
bool CAN_Protocol_ParseOBD2Response(const CAN_Message_t *msg, OBD2_Response_t *resp)
{
    if (!msg || !resp)
        return false;
    if (msg->ID != CAN_MSG_ID_OBD2_RESPONSE)
        return false;
    if (msg->DLC < 3)
        return false;

    uint8_t numBytes = msg->Data[0];
    resp->ServiceMode = msg->Data[1];
    resp->PID = msg->Data[2];

    /* Data bytes follow after mode + PID (2 header bytes subtracted) */
    resp->DataLength = (numBytes > 2) ? (numBytes - 2) : 0;
    if (resp->DataLength > 4)
        resp->DataLength = 4;

    for (uint8_t i = 0; i < resp->DataLength; i++)
    {
        resp->Data[i] = msg->Data[3 + i];
    }

    return true;
}

/* ================================================================== */
/*  Message Dispatcher                                                */
/* ================================================================== */

/**
 * @brief  Dispatch a received CAN message by ID to the correct parser.
 *
 * This function serves as a central routing point.  It can be extended
 * with a user-defined callback table for application-specific handling.
 *
 * Currently it simply parses the message into the appropriate struct
 * as a demonstration.  In a real application you would forward the
 * parsed data to the relevant subsystem or store it in a shared data
 * model.
 */
void CAN_Protocol_DispatchMessage(const CAN_Message_t *msg)
{
    if (!msg)
        return;

    switch (msg->ID)
    {
    case CAN_MSG_ID_ENGINE_RPM:
    {
        EngineRPM_Data_t rpmData;
        CAN_Protocol_ParseEngineRPM(msg, &rpmData);
        /* Application would process rpmData here */
        break;
    }

    case CAN_MSG_ID_VEHICLE_SPEED:
    {
        VehicleSpeed_Data_t speedData;
        CAN_Protocol_ParseVehicleSpeed(msg, &speedData);
        break;
    }

    case CAN_MSG_ID_TEMPERATURES:
    {
        Temperature_Data_t tempData;
        CAN_Protocol_ParseTemperatures(msg, &tempData);
        break;
    }

    case CAN_MSG_ID_BRAKE_STATUS:
    {
        BrakeStatus_Data_t brakeData;
        CAN_Protocol_ParseBrakeStatus(msg, &brakeData);
        break;
    }

    case CAN_MSG_ID_OBD2_REQUEST:
    {
        OBD2_Request_t obdReq;
        CAN_Protocol_ParseOBD2Request(msg, &obdReq);
        break;
    }

    case CAN_MSG_ID_OBD2_RESPONSE:
    {
        OBD2_Response_t obdResp;
        CAN_Protocol_ParseOBD2Response(msg, &obdResp);
        break;
    }

    default:
        /* Unknown message ID - ignore or log */
        break;
    }
}
