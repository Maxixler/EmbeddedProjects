#ifndef TELEMETRY_CONFIG_H
#define TELEMETRY_CONFIG_H

/**
 * @brief Telemetry frame configuration.
 */

/** Start of frame markers */
#define TELEMETRY_START_MARKER1 0xAA
#define TELEMETRY_START_MARKER2 0x55

/** Maximum payload length in bytes */
#define TELEMETRY_MAX_PAYLOAD_LEN 240

/** CRC-16-CCITT polynomial */
#define TELEMETRY_CRC_POLYNOMIAL 0x1021

#endif /* TELEMETRY_CONFIG_H */