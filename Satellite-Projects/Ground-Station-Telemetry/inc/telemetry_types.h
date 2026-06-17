#ifndef TELEMETRY_TYPES_H
#define TELEMETRY_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "telemetry_config.h"

/**
 * @brief Telemetry status codes
 */
typedef enum {
    TELEMETRY_STATUS_SUCCESS = 0,
    TELEMETRY_STATUS_INVALID_PARAM,
    TELEMETRY_STATUS_INVALID_START_MARKER,
    TELEMETRY_STATUS_INVALID_LENGTH,
    TELEMETRY_STATUS_CRC_ERROR,
    TELEMETRY_STATUS_INVALID_FRAME
} telemetry_status_t;

/**
 * @brief Raw telemetry frame as received from UART.
 */
typedef struct {
    uint8_t startMarker1;
    uint8_t startMarker2;
    uint8_t length;          // Length of payload (0 to TELEMETRY_MAX_PAYLOAD_LEN)
    uint8_t payload[TELEMETRY_MAX_PAYLOAD_LEN];
    uint8_t crcHigh;
    uint8_t crcLow;
} RawFrame_t;

/**
 * @brief Extracted telemetry data from a valid frame.
 */
typedef struct {
    uint32_t timestamp;
    float temperature;
    float voltage;
    uint8_t status;
    bool valid;              // Indicates if all fields were successfully extracted
} TelemetryData_t;

#endif /* TELEMETRY_TYPES_H */