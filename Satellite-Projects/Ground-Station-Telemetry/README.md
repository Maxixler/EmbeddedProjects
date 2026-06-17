# Ground Station Telemetry Processing Unit

This project implements a telemetry processing unit for ground station applications using an STM32F407G Discovery board and FreeRTOS.

## Overview

The system receives telemetry frames via UART, validates CRC-16, extracts telemetry fields, and displays results on UART debug output and an LCD interface.

## Features

- **Frame Parser**: State machine-based assembly of telemetry frames from UART byte stream
- **CRC Validation**: Table-driven CRC-16-CCITT for error detection
- **Field Extraction**: Parses timestamp, temperature, voltage, and status fields
- **FreeRTOS Architecture**: Four prioritized tasks with inter-task communication
- **Thread-Safe Debug Output**: Mutex-protected UART printf functionality
- **System Monitoring**: Stack usage reporting and health monitoring
- **LCD Abstraction**: Interface-layer for future LCD hardware integration
- **Test Tools**: Python-based telemetry frame generator for verification

## Project Structure

```
Ground-Station-Telemetry/
├── DESIGN.md                    (Design document)
├── APPROACHES.md               (Implementation approaches)
├── README.md                   (This file)
├── inc/                        (Header files)
│   ├── system_config.h         (System-wide constants)
│   ├── pin_config.h            (Pin assignments)
│   ├── telemetry_config.h      (Frame format parameters)
│   ├── frame_parser.h          (Frame assembly state machine)
│   ├── telemetry_protocol.h    (Packet validation, CRC, field extraction)
│   ├── telemetry_types.h       (Data structures for telemetry payloads)
│   ├── lcd_driver.h            (LCD display abstraction interface)
│   ├── crc16.h                 (CRC-16-CCITT implementation)
│   ├── debug_uart.h            (Debug output utilities)
│   └── app_tasks.h             (FreeRTOS task definitions, IPC objects)
├── src/                        (Source files)
│   ├── main.c                  (System init, clock config, scheduler start)
│   ├── app_tasks.c             (Task implementations)
│   ├── frame_parser.c          (Frame assembly state machine)
│   ├── telemetry_protocol.c    (CRC validation, field extraction)
│   ├── telemetry_types.c       (Unit conversion, formatting helpers)
│   ├── lcd_driver.c            (LCD stub/abstraction implementation)
│   ├── crc16.c                 (CRC-16-CCITT implementation)
│   └── debug_uart.c            (Printf-style debug output with mutex)
└── tools/                      (Utilities)
    └── telemetry_simulator.py  (Python script to generate test frames)
```

## Building

The project uses a standard Makefile for building:

```bash
make clean
make
```

This will produce `ground_station_telemetry.elf` in the project directory.

## Flashing

Use STM32CubeIDE or OpenOCD to flash the resulting `.elf` file to the Discovery board.

## Usage

1. Flash the board with the built firmware
2. Connect USART2 (PA2/PA3) to your computer's serial port
3. Run the telemetry simulator to send test frames:
   ```bash
   python tools/telemetry_simulator.py --port COM3 --baud 115200
   ```
4. Observe the debug output on the UART connection at 115200 baud

## Telemetry Frame Format

Frames follow this structure:
- Start Marker: 0xAA 0x55 (2 bytes)
- Length: Payload length in bytes (1 byte)
- Payload: 
  - Timestamp: 4 bytes (unsigned long, big-endian)
  - Temperature: 4 bytes (float, big-endian)
  - Voltage: 4 bytes (float, big-endian)
  - Status: 1 byte (unsigned char)
- CRC: 2 bytes (CRC-16-CCITT of payload, big-endian)

Total frame size: 2 + 1 + 13 + 2 = 18 bytes

## Task Overview

The system consists of four FreeRTOS tasks:

1. **UART Reception Task** (Priority 4):
   - Receives bytes via USART2
   - Assembles frames using frame parser state machine
   - Sends complete frames to processing queue
   - Toggles Activity LED (PD12) on frame reception

2. **Telemetry Processing Task** (Priority 3):
   - Validates frames (CRC, length)
   - Extracts telemetry fields
   - Sends telemetry data to LCD queue
   - Outputs debug information via UART
   - Toggles Error LED (PD14) on CRC failure

3. **LCD Display Task** (Priority 2):
   - Receives telemetry data from processing queue
   - Formats and displays on LCD (currently routed to debug UART)
   - Updates at ≤5 Hz rate

4. **System Monitor Task** (Priority 1):
   - Reports stack high water marks
   - Monitors packet/error counters
   - Provides system heartbeat
   - Toggles Power LED (PD15) as heartbeat indicator

## Inter-Process Communication

- **Queues**:
  - `xUartToProcessingQueue`: RawFrame_t (10 items)
  - `xProcessingToLcdQueue`: TelemetryData_t (5 items)
- **Mutex**:
  - `xUartTxMutex`: Protects UART TX from multiple tasks
- **Event Group**:
  - `xSystemEventGroup`: Error/status signaling

## LED Indicators

- PD12 (Green): Activity - toggles on each received frame
- PD13 (Orange): Processing - reserved for future use
- PD14 (Red): Error - toggles on CRC validation failure
- PD15 (Blue): Power - system heartbeat (1 Hz toggle)

## Testing

The `tools/telemetry_simulator.py` script can generate:
- Valid telemetry frames with realistic values
- Configurable CRC error injection
- Configurable truncated frame injection
- Burst mode testing
- Adjustable frame rates

## License

This project is provided for educational and experimentation purposes.