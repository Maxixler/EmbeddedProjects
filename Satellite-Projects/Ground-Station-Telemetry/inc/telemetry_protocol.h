#ifndef TELEMETRY_PROTOCOL_H
#define TELEMETRY_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_config.h"
#include "telemetry_types.h"

/**
 * @brief Frame parser configuration constants.
 */

/** Maximum frame buffer size for payload (matches TELEMETRY_MAX_PAYLOAD_LEN) */
#define MAX_FRAME_BUFFER_SIZE TELEMETRY_MAX_PAYLOAD_LEN

/** Frame timeout in milliseconds */
#define FRAME_TIMEOUT_MS 1000U

/**
 * @brief Validate a telemetry frame (CRC, length, etc.)
 * @param pxFrame Pointer to raw frame structure
 * @return TELEMETRY_STATUS_SUCCESS if valid, error code otherwise
 */
telemetry_status_t telemetry_validate_frame(const RawFrame_t *pxFrame);

/**
 * @brief Extract telemetry fields from a validated frame
 * @param pxFrame Pointer to validated raw frame structure
 * @param pxData Pointer to telemetry data structure to populate
 * @return TELEMETRY_STATUS_SUCCESS if successful, error code otherwise
 */
telemetry_status_t telemetry_extract_fields(const RawFrame_t *pxFrame, TelemetryData_t *pxData);

#endif /* TELEMETRY_PROTOCOL_H */