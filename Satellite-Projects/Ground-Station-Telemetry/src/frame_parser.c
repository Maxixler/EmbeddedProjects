/**
 * @file frame_parser.c
 * @brief Frame parser state machine implementation for telemetry frame assembly
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "frame_parser.h"
#include "telemetry_protocol.h"
#include "telemetry_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/**
 * @brief Initialize frame parser state machine
 * @param pxState Pointer to frame parser state structure
 */
void frame_parser_init(FrameParserState_t *pxState)
{
    if (pxState == NULL)
        return;

    pxState->eState = WAIT_START1;
    pxState->rawFrame.startMarker1 = 0;
    pxState->rawFrame.startMarker2 = 0;
    pxState->rawFrame.length = 0;
    pxState->payloadIndex = 0;
    pxState->tickCount = 0;
    pxState->ulStartTick = 0;

    // Clear payload buffer
    memset(pxState->rawFrame.payload, 0, MAX_FRAME_BUFFER_SIZE);
}

/**
 * @brief Feed a byte to the frame parser state machine
 * @param pxState Pointer to frame parser state structure
 * @param ucByte Received byte to process
 * @return FrameParserStatus_t Status of frame parsing operation
 */
FrameParserStatus_t frame_parser_feed_byte(FrameParserState_t *pxState, uint8_t ucByte)
{
    FrameParserStatus_t eReturnStatus = FRAME_INCOMPLETE;
    TickType_t xCurrentTick;

    if (pxState == NULL)
        return FRAME_ERROR;

    xCurrentTick = xTaskGetTickCount();

    // Check for timeout (if we've been waiting too long for a complete frame)
    if ((pxState->eState != WAIT_START1) &&
        ((xCurrentTick - pxState->ulStartTick) > (TickType_t)pdMS_TO_TICKS(FRAME_TIMEOUT_MS)))
    {
        // Timeout occurred
        frame_parser_init(pxState); // Reset state machine
        return FRAME_TIMEOUT;
    }

    // State machine processing
    switch (pxState->eState)
    {
        case WAIT_START1:
            // Looking for first start marker (0xAA)
            if (ucByte == TELEMETRY_START_MARKER1)
            {
                pxState->rawFrame.startMarker1 = ucByte;
                pxState->eState = WAIT_START2;
                pxState->ulStartTick = xCurrentTick; // Start timeout timer
            }
            // Stay in WAIT_START1 if byte doesn't match
            break;

        case WAIT_START2:
            // Looking for second start marker (0x55)
            if (ucByte == TELEMETRY_START_MARKER2)
            {
                pxState->rawFrame.startMarker2 = ucByte;
                pxState->eState = WAIT_LENGTH;
            }
            else
            {
                // Second start marker not received, go back to waiting for first
                pxState->eState = WAIT_START1;
                pxState->rawFrame.startMarker1 = 0; // Clear first marker
            }
            break;

        case WAIT_LENGTH:
            // Receiving length byte
            pxState->rawFrame.length = ucByte;

            // Validate length
            if (pxState->rawFrame.length > MAX_FRAME_BUFFER_SIZE)
            {
                // Length too large, reset state machine
                frame_parser_init(pxState);
                return FRAME_ERROR;
            }
            else if (pxState->rawFrame.length == 0)
            {
                // Zero length frame - go directly to CRC waiting (no payload)
                pxState->eState = WAIT_CRC_HIGH;
                pxState->payloadIndex = 0;
            }
            else
            {
                // Valid length, proceed to receive payload
                pxState->eState = RECEIVE_PAYLOAD;
                pxState->payloadIndex = 0;
            }
            break;

        case RECEIVE_PAYLOAD:
            // Receiving payload bytes
            if (pxState->payloadIndex < MAX_FRAME_BUFFER_SIZE)
            {
                pxState->rawFrame.payload[pxState->payloadIndex] = ucByte;
                pxState->payloadIndex++;

                // Check if we've received all payload bytes
                if (pxState->payloadIndex >= pxState->rawFrame.length)
                {
                    pxState->eState = WAIT_CRC_HIGH;
                }
            }
            else
            {
                // Buffer overflow - reset state machine
                frame_parser_init(pxState);
                return FRAME_ERROR;
            }
            break;

        case WAIT_CRC_HIGH:
            // Receiving high byte of CRC
            pxState->rawFrame.crcHigh = ucByte;
            pxState->eState = WAIT_CRC_LOW;
            break;

        case WAIT_CRC_LOW:
            // Receiving low byte of CRC - frame is complete
            pxState->rawFrame.crcLow = ucByte;
            pxState->eState = WAIT_START1; // Reset for next frame
            eReturnStatus = FRAME_COMPLETE;
            break;

        default:
            // Unknown state - reset state machine
            frame_parser_init(pxState);
            eReturnStatus = FRAME_ERROR;
            break;
    }

    return eReturnStatus;
}

/**
 * @brief Reset frame parser state machine
 * @param pxState Pointer to frame parser state structure
 */
void frame_parser_reset(FrameParserState_t *pxState)
{
    if (pxState != NULL)
    {
        frame_parser_init(pxState);
    }
}