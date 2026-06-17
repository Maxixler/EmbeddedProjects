#!/usr/bin/env python3
"""Test script to verify telemetry frame generation (without serial dependency)"""

import struct

# Copy the essential functions for testing
def _generate_crc16_table():
    table = []
    for i in range(256):
        crc = 0
        for j in range(8):
            if (crc ^ (i << (8 - j))) & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
        table.append(crc & 0xFFFF)
    return table

crc16_table = _generate_crc16_table()

def crc16_ccitt(data: bytes) -> int:
    """Calculate CRC-16-CCITT (0x1021)"""
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ crc16_table[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF

def create_telemetry_frame(timestamp: int = 0, temperature: float = 25.0,
                          voltage: float = 3.3, status: int = 0x01,
                          inject_error: bool = False) -> bytes:
    """
    Create a telemetry frame according to the specification

    Frame format:
    - Start marker: 0xAA 0x55 (2 bytes)
    - Length: payload length (1 byte)
    - Payload: timestamp(4) + temperature(4) + voltage(4) + status(1) = 13 bytes
    - CRC: CRC-16-CCITT of payload (2 bytes)

    Total: 2 + 1 + 13 + 2 = 18 bytes
    """
    # Pack the payload (big-endian as used in the receiver)
    payload = struct.pack('>IffB', timestamp, temperature, voltage, status)

    # Calculate CRC
    crc = crc16_ccitt(payload)

    # Inject error if requested (flip a bit in the CRC)
    if inject_error:
        crc ^= 0x0001  # Flip LSB

    # Build frame: start marker + length + payload + CRC
    frame = bytearray()
    frame.extend([0xAA, 0x55])  # Start marker
    frame.append(len(payload))  # Length
    frame.extend(payload)       # Payload
    frame.extend(struct.pack('>H', crc))  # CRC (big-endian)

    return bytes(frame)

def create_truncated_frame() -> bytes:
    """Create a truncated frame (missing CRC)"""
    frame = create_telemetry_frame()
    # Return frame without the last 2 bytes (CRC)
    return frame[:-2]

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