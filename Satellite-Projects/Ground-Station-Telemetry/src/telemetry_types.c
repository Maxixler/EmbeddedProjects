/**
 * @file telemetry_types.c
 * @brief Telemetry types implementation - helper functions and formatting
 * @author Embedded Systems Developer
 * @date 2026
 */

#include "telemetry_types.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief Initialize a telemetry data structure to default values
 * @param pxData Pointer to telemetry data structure to initialize
 */
void vTelemetryDataInit(TelemetryData_t *pxData)
{
    if (pxData != NULL)
    {
        pxData->timestamp = 0;
        pxData->temperature = 0.0f;
        pxData->voltage = 0.0f;
        pxData->status = 0;
        pxData->valid = false;
    }
}

/**
 * @brief Format telemetry data for display
 * @param pxData Pointer to telemetry data structure
 * @param pcBuffer Buffer to store formatted string
 * @param xBufferSize Size of the buffer
 * @return Number of characters written (excluding null terminator)
 */
size_t xTelemetryFormatForDisplay(const TelemetryData_t *pxData, char *pcBuffer, size_t xBufferSize)
{
    if (pxData == NULL || pcBuffer == NULL || xBufferSize == 0)
        return 0;

    if (!pxData->valid)
    {
        snprintf(pcBuffer, xBufferSize, "INVALID TELEMETRY");
        return strlen(pcBuffer);
    }

    return snprintf(pcBuffer, xBufferSize,
                   "T:%lu Temp:%.1fC Volt:%.2fV Stat:%u",
                   pxData->timestamp,
                   pxData->temperature,
                   pxData->voltage,
                   pxData->status);
}