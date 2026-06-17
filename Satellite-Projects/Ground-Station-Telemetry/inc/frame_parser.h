#ifndef FRAME_PARSER_H
#define FRAME_PARSER_H

#include <stdint.h>
#include "telemetry_types.h"

/**
 * @brief Frame parser state machine states.
 */
typedef enum {
    WAIT_START1,   // Waiting for first start marker (0xAA)
    WAIT_START2,   // Waiting for second start marker (0x55)
    WAIT_LENGTH,   // Waiting for payload length byte
    RECEIVE_PAYLOAD, // Receiving payload bytes
    WAIT_CRC_HIGH, // Waiting for high byte of CRC
    WAIT_CRC_LOW   // Waiting for low byte of CRC
} FrameParserStateEnum_t;

/**
 * @brief Frame parser status returned by feed_byte.
 */
typedef enum {
    FRAME_INCOMPLETE,      // Frame reception in progress, no frame yet
    FRAME_COMPLETE,     // A complete frame has been received
    FRAME_ERROR,        // Error in frame (e.g., length too big, unexpected start marker)
    FRAME_TIMEOUT       // Timeout waiting for frame completion
} FrameParserStatus_t;

/**
 * @brief Frame parser context structure.
 */
typedef struct {
    FrameParserStateEnum_t eState;        // Current state machine state
    RawFrame_t rawFrame;                  // Frame being assembled
    uint8_t payloadIndex;                 // Index of next byte to write in payload
    uint32_t tickCount;                   // Tick count
    uint32_t ulStartTick;                 // Tick count when frame reception started
} FrameParserState_t;

/**
 * @brief Initialize frame parser state machine.
 * @param pxState Pointer to frame parser state structure.
 */
void frame_parser_init(FrameParserState_t *pxState);

/**
 * @brief Feed a byte to the frame parser state machine.
 * @param pxState Pointer to frame parser state structure.
 * @param ucByte Received byte to process.
 * @return FrameParserStatus_t Status of frame parsing operation.
 */
FrameParserStatus_t frame_parser_feed_byte(FrameParserState_t *pxState, uint8_t ucByte);

/**
 * @brief Reset frame parser state machine.
 * @param pxState Pointer to frame parser state structure.
 */
void frame_parser_reset(FrameParserState_t *pxState);

#endif /* FRAME_PARSER_H */