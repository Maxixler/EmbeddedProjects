/**
 * @file telemetry_protocol.c
 * @brief Telemetry protocol implementation - CRC validation and field extraction
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "telemetry_protocol.h"
#include "telemetry_types.h"
#include "crc16.h"
#include <string.h>

/**
 * @brief Validate a telemetry frame (CRC, length, etc.)
 * @param pxFrame Pointer to raw frame structure
 * @return TELEMETRY_STATUS_SUCCESS if valid, error code otherwise
 */
telemetry_status_t telemetry_validate_frame(const RawFrame_t *pxFrame)
{
    if (pxFrame == NULL)
        return TELEMETRY_STATUS_INVALID_PARAM;

    /* Check start markers */
    if (pxFrame->startMarker1 != TELEMETRY_START_MARKER1 ||
        pxFrame->startMarker2 != TELEMETRY_START_MARKER2)
    {
        return TELEMETRY_STATUS_INVALID_START_MARKER;
    }

    /* Check payload length */
    if (pxFrame->length > TELEMETRY_MAX_PAYLOAD_LEN)
        return TELEMETRY_STATUS_INVALID_LENGTH;

    /* Calculate CRC over frame (excluding CRC bytes themselves) */
    uint8_t aucFrameBuffer[sizeof(RawFrame_t) - 2]; /* Exclude crcHigh and crcLow */
    size_t xBufferSize = 0;

    /* Copy frame fields except CRC */
    memcpy(aucFrameBuffer, &pxFrame->startMarker1, 1);
    memcpy(aucFrameBuffer + 1, &pxFrame->startMarker2, 1);
    memcpy(aucFrameBuffer + 2, &pxFrame->length, 1);
    memcpy(aucFrameBuffer + 3, pxFrame->payload, pxFrame->length);
    xBufferSize = 3 + pxFrame->length;

    /* Calculate CRC */
    uint16_t usCalculatedCrc = crc16_calculate(aucFrameBuffer, xBufferSize);

    /* Compare with received CRC */
    uint16_t usReceivedCrc = (pxFrame->crcHigh << 8) | pxFrame->crcLow;
    if (usCalculatedCrc != usReceivedCrc)
        return TELEMETRY_STATUS_CRC_ERROR;

    return TELEMETRY_STATUS_SUCCESS;
}

/**
 * @brief Extract telemetry fields from a validated frame
 * @param pxFrame Pointer to validated raw frame structure
 * @param pxData Pointer to telemetry data structure to populate
 * @return TELEMETRY_STATUS_SUCCESS if successful, error code otherwise
 */
telemetry_status_t telemetry_extract_fields(const RawFrame_t *pxFrame, TelemetryData_t *pxData)
{
    if (pxFrame == NULL || pxData == NULL)
        return TELEMETRY_STATUS_INVALID_PARAM;

    /* Validate frame first */
    if (telemetry_validate_frame(pxFrame) != TELEMETRY_STATUS_SUCCESS)
        return TELEMETRY_STATUS_INVALID_FRAME;

    /* Extract timestamp (assuming big-endian) */
    pxData->timestamp =
        ((uint32_t)pxFrame->payload[0] << 24) |
        ((uint32_t)pxFrame->payload[1] << 16) |
        ((uint32_t)pxFrame->payload[2] << 8)  |
        ((uint32_t)pxFrame->payload[3]);

    /* Extract temperature (IEEE 754 float, big-endian) */
    uint8_t aucTempBytes[4] = {
        pxFrame->payload[4],
        pxFrame->payload[5],
        pxFrame->payload[6],
        pxFrame->payload[7]
    };
    memcpy(&pxData->temperature, aucTempBytes, sizeof(float));

    /* Extract voltage (IEEE 754 float, big-endian) */
    uint8_t aucVoltageBytes[4] = {
        pxFrame->payload[8],
        pxFrame->payload[9],
        pxFrame->payload[10],
        pxFrame->payload[11]
    };
    memcpy(&pxData->voltage, aucVoltageBytes, sizeof(float));

    /* Extract status */
    pxData->status = pxFrame->payload[12];

    /* Mark as valid */
    pxData->valid = true;

    return TELEMETRY_STATUS_SUCCESS;
}