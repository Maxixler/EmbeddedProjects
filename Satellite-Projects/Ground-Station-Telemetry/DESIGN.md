# Ground Station Telemetry Processing Unit - Design Specification

## 1. Project Overview

### Purpose
To develop a satellite ground station telemetry processing unit that receives UDP-formatted telemetry packets, validates their integrity, extracts telemetry data, and displays it on both UART debug output and LCD display. This project focuses on building RTOS and embedded C skills while creating a portfolio piece aligned with Profen's satellite communications work.

### Success Criteria (MVP)
- Successfully receive UDP telemetry packets (simulated via UART for initial development)
- Validate packet integrity using CRC-16 checksum
- Extract and decode telemetry data fields
- Display telemetry values on both UART debug console and LCD display
- Handle basic error conditions (checksum failures, frame errors)
- Implement using FreeRTOS task architecture on STM32F407VG

### Extended Goals (Time Permitting)
- Implement CCSDS-compliant Space Packet Protocol structure
- Add telemetry packet sequencing and gap detection
- Implement basic ground station command generation capability
- Add system health monitoring (stack usage, task timing)
- Support multiple telemetry packet types

## 2. System Architecture

### High-Level Block Diagram
```
[UART Input] → [UART Reception Task] → [UDP Queue] → 
[UDP Processing Task] → [LCD Queue] → [LCD Display Task]
                              ↓
                     [Debug UART Output]
                              ↓
                    [Error Handling/LED]
```

### Task Architecture
1. **UART Reception Task**:
   - Configures UART peripheral for asynchronous reception
   - Uses interrupt-driven or DMA-based byte reception
   - Assembles incoming bytes into frames based on delimiters or length
   - Performs basic framing validation (start/end bytes, length checks)
   - Sends complete frames to UDP Processing Queue

2. **UDP Processing Task**:
   - Receives frames from UART Reception Queue
   - Validates UDP packet structure (simplified or CCSDS)
   - Calculates and verifies CRC-16 checksum
   - Extracts telemetry data fields (timestamp, parameters)
   - Sends processed telemetry to LCD Display Queue
   - Outputs debug information via UART
   - Handles error conditions (invalid CRC, malformed packets)

3. **LCD Display Task**:
   - Receives telemetry data from UDP Processing Queue
   - Formats data for LCD display (fixed-point conversion, units)
   - Manages LCD refresh and display routines
   - Handles LCD initialization and configuration
   - Implements basic display layouts (telemetry values, status indicators)

4. **System Monitor Task** (Extended):
   - Monitors system health (stack usage, task timing)
   - Implements watchdog functionality
   - Provides system status via LEDs or additional UART messages

### Inter-Task Communication
- **Queues**: 
  - UART_TO_UDP_QUEUE: Frame data from UART to UDP processing
  - UDP_TO_LCD_QUEUE: Processed telemetry from UDP to LCD display
- **Mutexes**: Protect shared resources (LCD controller, UART transmit)
- **Event Groups**: For synchronization and signal handling (extended features)

## 3. Hardware Abstraction & Peripherals

### STM32F407VG Peripheral Usage
- **USART1**: Primary UART for telemetry input (receive) and debug output (transmit)
- **I2C1**: Interface to LCD display (using PCF8574 I2C expander or direct LCD controller)
- **GPIO**: LEDs for status indication (power, error, activity)
- **SysTick**: FreeRTOS tick timer
- **NVIC**: Interrupt configuration for UART reception

### Peripheral Abstraction Strategy
- Direct register access for critical timing operations (UART receive)
- STM32 HAL drivers for peripheral initialization and configuration
- Thin abstraction layers for testability and portability
- Configurable pin assignments via header files

## 4. Telemetry Protocol Design

### Packet Structure (Simplified UDP-like)
```
+----------------+----------------+----------------+----------------+
| Start Marker   | Length         | Payload        | CRC-16         |
| (0xAA 0x55)    | (1 byte)       | (N bytes)      | (2 bytes)      |
+----------------+----------------+----------------+----------------+
```

### Payload Structure (Telemetry Data)
```
+----------------+----------------+----------------+----------------+
| Timestamp      | Parameter 1    | Parameter 2    | ...            |
| (4 bytes)      | (4 bytes float)| (4 bytes float)| ...            |
+----------------+----------------+----------------+----------------+
```

### Error Detection
- **CRC-16**: Standard CRC-16-CCITT (0x1021) for payload integrity
- **Length Validation**: Ensures payload length matches expectations
- **Marker Validation**: Verifies start/end frame markers
- **Timeout Detection**: Identifies stalled communication

### Data Types Supported
- 32-bit unsigned integers (counters, status)
- 32-bit IEEE 754 floating point (sensor readings)
- 8-bit enumerated values (system states)
- Bit-packed status flags

## 5. Software Components & Modules

### Core Modules
1. **uart_driver.c/h**: UART configuration, interrupt handling, byte reception
2. **frame_parser.c/h**: Frame assembly, start/end marker detection, length validation
3. **udp_processor.c/h**: UDP/CCSDS-like packet validation, CRC verification, field extraction
4. **telemetry_types.c/h**: Telemetry data structures, unit conversion, formatting
5. **lcd_driver.c/h**: LCD initialization, character graphics, display routines
6. **crc16.c/h**: CRC-16-CCITT implementation
7. **freertos_tasks.c/h**: Task definitions, queue creation, synchronization
8. **main.cpp/c**: System initialization, FreeRTOS scheduler start

### Configuration Modules
1. **system_config.h**: System-wide parameters (baud rates, queue sizes, stack depths)
2. **pin_config.h**: Hardware pin assignments
3. **telemetry_config.h**: Telemetry format parameters, field definitions

### Utility Modules
1. **time_utils.c/h**: Time conversion, timestamp handling
2. **math_utils.c/h**: Fixed-point math, scaling functions
3. **debug.c/h**: Conditional debug output macros

## 6. Error Handling & Fault Tolerance

### Error Detection Mechanisms
- **Communication Errors**: Framing errors, overflow, noise detection via UART status
- **Protocol Errors**: Invalid start/end markers, length mismatches
- **Data Integrity Errors**: CRC-16 validation failures
- **Processing Errors**: Invalid telemetry values, out-of-range parameters

### Error Response Strategies
- **UART Errors**: Reset receiver state machine, flush buffers
- **CRC Failures**: Discard packet, increment error counter, optional retransmission request (future)
- **LCD Errors**: Fallback to UART-only display, visual error indication via LEDs
- **Queue Overflows**: Drop oldest packets, implement queue monitoring
- **Task Failures**: System monitor task can restart problematic tasks (advanced)

### Debugging & Diagnostics
- **UART Debug Output**: Packet reception, validation results, telemetry values
- **LED Indicators**: Power (solid), Activity (blinking on valid packet), Error (blinking on CRC fail)
- **Error Counters**: Track different error types for system health assessment
- **Timestamp Logging**: Optional packet reception timing for jitter analysis

## 7. Real-Time Considerations & Performance

### Timing Requirements
- **UART Reception**: Must handle incoming byte stream at expected baud rate (e.g., 115200)
- **Frame Processing**: Complete validation within reasonable time (<10ms per frame)
- **LCD Update**: Refresh rate suitable for human readability (>5Hz)
- **System Responsiveness**: Visible response to input within 100ms

### FreeRTOS Configuration
- **Tick Rate**: 1kHz (1ms tick) for good resolution
- **Task Priorities**: 
  - UART Reception: Highest (time-critical input)
  - UDP Processing: Medium (data processing)
  - LCD Display: Lower (human interface)
  - System Monitor: Lowest (background)
- **Stack Sizes**: Appropriately sized for each task's needs
- **Queue Sizes**: Sized to handle expected bursts (e.g., 10-20 frames)

### Performance Optimization
- **Minimize Interrupt Latency**: Short UART ISR, defer processing to task
- **Efficient Memory Use**: Static allocation where possible, avoid fragmentation
- **Optimized CRC**: Table-driven or hardware-assisted CRC calculation
- ** LCD Updates**: Batch updates, minimize I2C transactions
- **Polling vs Interrupts**: Interrupt-driven UART reception for efficiency

## 8. Testing & Verification Strategy

### Unit Testing Approach
- **Mock Hardware**: Abstract hardware interfaces for host-based testing
- **CRC Validation**: Test against known good/bad values
- **Frame Parsing**: Test with various frame types (valid, invalid, partial)
- **Telemetry Conversion**: Validate scaling, formatting, edge cases
- **LCD Formatting**: Verify string generation for different value ranges

### Integration Testing
- **Frame Injection**: Synthetic frame injection via UART to test full pipeline
- **Error Injection**: Intentional errors to verify error handling
- **Timing Validation**: Measure task execution times, queue latencies
- **Stress Testing**: Continuous high-rate data injection

### System Testing
- **End-to-End**: Simulated telemetry source → UART input → processing → LCD output
- **Boundary Conditions**: Minimum/maximum values, special cases (NaN, infinity)
- **Long-Run Stability**: Extended operation to detect memory leaks, stack overflow
- **Environmental**: Temperature/voltage variations (if hardware available)

### Test Equipment & Tools
- **Logic Analyzer**: For UART timing validation
- **Oscilloscope**: For signal integrity, timing measurements
- **USB-to-UART Adapter**: For test signal injection
- **STM32CubeIDE/Project Tools**: For debugging and tracing
- **FreeRTOS Trace Tools**: For task timing analysis (if available)

## 9. Development Milestones & Timeline (20 Days)

### Week 1: Foundation & Basic Reception
- **Days 1-2**: Project setup, STM32F407VG configuration, basic UART hello world
- **Days 3-4**: FreeRTOS task creation, inter-task communication basics
- **Days 5-7**: UART reception task implementation, basic frame parsing

### Week 2: Core Processing & Display
- **Days 8-9**: UDP packet validation, CRC-16 implementation
- **Days 10-11**: Telemetry field extraction, data type handling
- **Days 12-13**: LCD driver implementation, basic display output
- **Day 14**: Integration of UART → UDP → LCD pipeline

### Week 3: Robustness & Features
- **Days 15-16**: Error handling implementation, CRC failure recovery
- **Days 17-18**: Debug output enhancement, status indicators (LEDs)
- **Days 19-20**: System optimization, final testing, documentation

### Stretch Goals (If Time Permits)
- CCSDS packet structure implementation
- Telemetry sequencing and gap detection
- Basic command generation capability
- System health monitoring extensions

## 10. Dependencies & Resources

### Hardware Requirements
- STM32F407VG Development Board (e.g., Discovery, Nucleo, or custom)
- UART-to-USB adapter (for PC communication and testing)
- LCD Display (16x2 or 20x4 character LCD with I2C or parallel interface)
- Breadboard and jumper wires (for prototyping)
- LEDs and resistors (for status indication)
- Logic analyzer or oscilloscope (recommended for debugging)

### Software Tools
- STM32CubeIDE or equivalent STM32 development environment
- STM32CubeMX (for pinout and clock configuration)
- FreeRTOS (included via STM32Cube or manual inclusion)
- GNU Arm Embedded Toolchain (compiler, debugger)
- OpenOCD or ST-Link utilities (for flashing and debugging)
- Serial terminal emulator (PuTTY, Tera Term, or similar)

### Reference Materials
- STM32F407VG Reference Manual and Datasheet
- FreeRTOS Documentation and API Reference
- CCSDS Space Packet Protocol Standards (for extended work)
- UART, I2C, and SPI communication protocols
- CRC-16-CCITT algorithm specifications

## 11. Future Extensions & Enhancements

### Phase 2: Enhanced Telemetry Support
- Multiple telemetry packet types with dynamic routing
- Telemetry history and trending capabilities
- Alarm and threshold monitoring with alerts
- Data logging to external storage (SD card)

### Phase 3: Command & Control
- Ground station command generation and transmission
- Command acknowledgment and retry mechanisms
- Command sequencing and scripting capabilities
- Equipment control interfaces (GPIO, DACs, etc.)

### Phase 4: Network Integration
- Actual UDP/IP over Ethernet (using external MAC/PHY)
- TCP/IP stack integration for reliable communication
- Network time synchronization (NTP/PTP)
- Remote monitoring and control capabilities

### Phase 5: Advanced Features
- Adaptive data rate based on link quality
- Forward Error Correction (FEC) implementation
- Encryption and authentication for secure telemetry
- Multiple antenna tracking integration