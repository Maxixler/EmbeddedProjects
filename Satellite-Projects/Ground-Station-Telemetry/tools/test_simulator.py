#!/usr/bin/env python3
"""Test script to verify telemetry frame generation"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__)))

from telemetry_simulator import create_telemetry_frame, create_truncated_frame, crc16_ccitt

def test_frame_creation():
    print("Testing telemetry frame creation...")

    # Test normal frame
    frame = create_telemetry_frame(timestamp=1000, temperature=25.5, voltage=3.3, status=0x01)
    print(f"Normal frame length: {len(frame)} bytes")
    print(f"Frame hex: {frame.hex()}")

    # Verify start marker
    assert frame[0] == 0xAA and frame[1] == 0x55, "Invalid start marker"

    # Verify length
    payload_length = frame[2]
    print(f"Payload length: {payload_length}")
    assert payload_length == 13, f"Expected payload length 13, got {payload_length}"

    # Verify total length
    expected_length = 2 + 1 + payload_length + 2  # start + length + payload + crc
    assert len(frame) == expected_length, f"Expected length {expected_length}, got {len(frame)}"

    # Extract payload and verify CRC
    payload = frame[3:3+payload_length]
    received_crc = int.from_bytes(frame[3+payload_length:3+payload_length+2], byteorder='big')
    calculated_crc = crc16_ccitt(payload)

    print(f"Payload: {payload.hex()}")
    print(f"Received CRC: 0x{received_crc:04X}")
    print(f"Calculated CRC: 0x{calculated_crc:04X}")

    assert received_crc == calculated_crc, "CRC mismatch"

    # Test truncated frame
    truncated = create_truncated_frame()
    print(f"\nTruncated frame length: {len(truncated)} bytes")
    assert len(truncated) == len(frame) - 2, "Truncated frame should be missing CRC bytes"

    # Test error injection
    error_frame = create_telemetry_frame(timestamp=1000, temperature=25.5, voltage=3.3, status=0x01, inject_error=True)
    error_payload = error_frame[3:3+payload_length]
    error_received_crc = int.from_bytes(error_frame[3+payload_length:3+payload_length+2], byteorder='big')
    error_calculated_crc = crc16_ccitt(error_payload)

    print(f"\nError frame - Received CRC: 0x{error_received_crc:04X}")
    print(f"Error frame - Calculated CRC: 0x{error_calculated_crc:04X}")
    assert error_received_crc != error_calculated_crc, "Error frame should have incorrect CRC"
    assert (error_received_crc ^ error_calculated_crc) == 0x0001, "Error should flip LSB of CRC"

    print("\nAll tests passed!")

if __name__ == "__main__":
    test_frame_creation()