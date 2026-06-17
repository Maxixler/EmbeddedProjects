#!/usr/bin/env python3
"""
Telemetry Simulator for Ground Station Telemetry Processing Unit
Generates test telemetry frames for verification and testing

Usage:
    python telemetry_simulator.py [options]

Options:
    --port PORT         Serial port to use (default: COM3)
    --baud BAUD         Baud rate (default: 115200)
    --rate RATE         Frames per second (default: 1)
    --count COUNT       Number of frames to send (default: infinity)
    --error-rate RATE   CRC error injection rate (0.0-1.0, default: 0.0)
    --truncate-rate RATE Truncated frame injection rate (0.0-1.0, default: 0.0)
    --burst-count COUNT Number of frames in burst (default: 1)
    --burst-interval MS Interval between bursts in milliseconds (default: 1000)
    --help              Show this help message
"""

import serial
import struct
import time
import argparse
import sys
import binascii
from typing import Optional

# CRC-16-CCITT implementation (matches the one in crc16.c)
def crc16_ccitt(data: bytes) -> int:
    """Calculate CRC-16-CCITT (0x1021)"""
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ crc16_table[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF

# Pre-calculate CRC table for performance
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
    payload = struct.Struct('>ffB').pack(temperature, voltage, status)  # Wait, let me check the actual order

    # Looking at the telemetry extraction code, it's: timestamp (ul), temperature (f), voltage (f), status (uc)
    # And they use ntohl/ntohs for conversion, meaning big-endian
    payload = struct.Struct('>IffB').pack(timestamp, temperature, voltage, status)

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

def main():
    parser = argparse.ArgumentParser(description='Telemetry Simulator for Ground Station')
    parser.add_argument('--port', default='COM3', help='Serial port to use (default: COM3)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--rate', type=float, default=1.0, help='Frames per second (default: 1)')
    parser.add_argument('--count', type=int, default=0, help='Number of frames to send (0=infinity)')
    parser.add_argument('--error-rate', type=float, default=0.0,
                       help='CRC error injection rate (0.0-1.0, default: 0.0)')
    parser.add_argument('--truncate-rate', type=float, default=0.0,
                       help='Truncated frame injection rate (0.0-1.0, default: 0.0)')
    parser.add_argument('--burst-count', type=int, default=1,
                       help='Number of frames in burst (default: 1)')
    parser.add_argument('--burst-interval', type=int, default=1000,
                       help='Interval between bursts in milliseconds (default: 1000)')

    args = parser.parse_args()

    # Validate rates
    if not 0.0 <= args.error_rate <= 1.0:
        print("Error: error-rate must be between 0.0 and 1.0")
        sys.exit(1)

    if not 0.0 <= args.truncate_rate <= 1.0:
        print("Error: truncate-rate must be between 0.0 and 1.0")
        sys.exit(1)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Opened serial port {args.port} at {args.baud} baud")
        print(f"Sending frames at {args.rate} fps")
        if args.error_rate > 0:
            print(f"Injecting CRC errors at rate {args.error_rate*100:.1f}%")
        if args.truncate_rate > 0:
            print(f"Injecting truncated frames at rate {args.truncate_rate*100:.1f}%")
        if args.burst_count > 1:
            print(f"Burst mode: {args.burst_count} frames every {args.burst_interval}ms")
        print("Press Ctrl+C to stop\n")

        frame_count = 0
        burst_count = 0
        last_burst_time = time.time()

        while True:
            # Check if we've reached the frame limit
            if args.count > 0 and frame_count >= args.count:
                break

            # Check if we should send a burst
            current_time = time.time()
            if args.burst_count > 1:
                if (current_time - last_burst_time) * 1000 < args.burst_interval:
                    time.sleep(0.001)  # Small sleep to prevent busy-wait
                    continue

                # Send burst
                for i in range(args.burst_count):
                    # Check frame limit again
                    if args.count > 0 and frame_count >= args.count:
                        break

                    # Determine frame type
                    if args.truncate_rate > 0 and (frame_count % 100) < (args.truncate_rate * 100):
                        frame = create_truncated_frame()
                        frame_type = "TRUNCATED"
                    elif args.error_rate > 0 and (frame_count % 100) < (args.error_rate * 100):
                        frame = create_telemetry_frame(
                            timestamp=frame_count,
                            temperature=20.0 + (frame_count % 10),  # 20-30°C
                            voltage=3.0 + (frame_count % 5) * 0.1,  # 3.0-3.4V
                            status=0x01 if (frame_count % 2 == 0) else 0x02,
                            inject_error=True
                        )
                        frame_type = "ERROR"
                    else:
                        frame = create_telemetry_frame(
                            timestamp=frame_count,
                            temperature=20.0 + (frame_count % 10),  # 20-30°C
                            voltage=3.0 + (frame_count % 5) * 0.1,  # 3.0-3.4V
                            status=0x01 if (frame_count % 2 == 0) else 0x02
                        )
                        frame_type = "NORMAL"

                    ser.write(frame)
                    ser.flush()

                    # Print frame info occasionally
                    if frame_count % 10 == 0:
                        print(f"Frame {frame_count}: {frame_type} "
                              f"[{' '.join(f'{b:02X}' for b in frame[:6])}...] "
                              f"({len(frame)} bytes)")

                    frame_count += 1

                    # Small delay between frames in burst
                    if i < args.burst_count - 1:
                        time.sleep(0.01)

                last_burst_time = current_time
                time.sleep(0.001)  # Prevent tight loop
            else:
                # Regular mode (not burst)
                # Determine frame type
                if args.truncate_rate > 0 and (frame_count % 100) < (args.truncate_rate * 100):
                    frame = create_truncated_frame()
                    frame_type = "TRUNCATED"
                elif args.error_rate > 0 and (frame_count % 100) < (args.error_rate * 100):
                    frame = create_telemetry_frame(
                        timestamp=frame_count,
                        temperature=20.0 + (frame_count % 10),  # 20-30°C
                        voltage=3.0 + (frame_count % 5) * 0.1,  # 3.0-3.4V
                        status=0x01 if (frame_count % 2 == 0) else 0x02,
                        inject_error=True
                    )
                    frame_type = "ERROR"
                else:
                    frame = create_telemetry_frame(
                        timestamp=frame_count,
                        temperature=20.0 + (frame_count % 10),  # 20-30°C
                        voltage=3.0 + (frame_count % 5) * 0.1,  # 3.0-3.4V
                        status=0x01 if (frame_count % 2 == 0) else 0x02
                    )
                    frame_type = "NORMAL"

                ser.write(frame)
                ser.flush()

                # Print frame info occasionally
                if frame_count % 10 == 0:
                    print(f"Frame {frame_count}: {frame_type} "
                          f"[{' '.join(f'{b:02X}' for b in frame[:6])}...] "
                          f"({len(frame)} bytes)")

                frame_count += 1

                # Wait for next frame
                time.sleep(1.0 / args.rate if args.rate > 0 else 0)

    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print(f"\n\nSent {frame_count} frames")
        ser.close()
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()