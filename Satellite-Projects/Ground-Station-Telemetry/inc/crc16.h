#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Calculate CRC-16-CCITT for a data buffer
 * @param pucData Pointer to data buffer
 * @param usLength Length of data buffer in bytes
 * @return CRC-16-CCITT value
 */
uint16_t crc16_calculate(const uint8_t *pucData, size_t usLength);

/**
 * @brief Validate data against expected CRC-16-CCITT value
 * @param pucData Pointer to data buffer
 * @param usLength Length of data buffer in bytes
 * @param usExpectedCrc Expected CRC-16-CCITT value
 * @return true if CRC matches, false otherwise
 */
bool crc16_validate(const uint8_t *pucData, size_t usLength, uint16_t usExpectedCrc);

#endif /* CRC16_H */