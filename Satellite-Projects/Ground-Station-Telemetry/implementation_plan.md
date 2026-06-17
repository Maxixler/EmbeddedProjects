# Ground Station Telemetry Processing Unit — Implementation Plan

## Goal

Implement the **MVP** of the Ground Station Telemetry Processing Unit on STM32F407G-DISC1 using FreeRTOS. The system receives telemetry frames via UART, validates CRC-16, extracts telemetry fields, and displays results on UART debug output and an LCD (interface-only, no hardware yet).

## Key Design Decisions

> [!IMPORTANT]
> **Renamed "UDP" to "Telemetry Frame"** throughout the codebase, as noted in the evaluation. The protocol is a custom serial framing protocol, not actual UDP.

- **Board**: STM32F407G-DISC1 (Discovery)
- **Build System**: STM32CubeIDE-style project (source-only, no `.ioc` — hand-written register/HAL init matching your existing projects)
- **LCD**: Abstract driver interface only (no hardware yet)
- **UART**: USART2 (PA2/PA3) for telemetry input + debug output, matching your existing Discovery pin usage
- **Code Style**: Matches your existing conventions — Doxygen `@` comments, `/*===*/` section banners, `PascalCase_t` for types, `snake_case` for functions, `UPPER_CASE` for macros, `x`/`v`/`ul` Hungarian prefixes for FreeRTOS objects

## Proposed Changes

### Component 1: Project Structure

#### [NEW] Project directory under `Satellite-Projects/Ground-Station-Telemetry/`

```
Ground-Station-Telemetry/
├── DESIGN.md                    (existing)
├── APPROACHES.md                (existing)
├── README.md                    (new - project overview)
├── inc/
│   ├── system_config.h          — System-wide constants (baud, queue sizes, stack depths)
│   ├── pin_config.h             — Discovery board pin assignments
│   ├── telemetry_config.h       — Frame format parameters, field definitions
│   ├── frame_parser.h           — Frame assembly state machine
│   ├── telemetry_protocol.h     — Packet validation, CRC, field extraction
│   ├── telemetry_types.h        — Data structures for telemetry payloads
│   ├── lcd_driver.h             — LCD display abstraction interface
│   ├── crc16.h                  — CRC-16-CCITT implementation
│   ├── debug_uart.h             — Debug output utilities
│   └── app_tasks.h              — FreeRTOS task definitions, IPC objects
├── src/
│   ├── main.c                   — System init, clock config, scheduler start
│   ├── app_tasks.c              — Task implementations (UART RX, Processing, LCD, Monitor)
│   ├── frame_parser.c           — Byte-by-byte frame assembly state machine
│   ├── telemetry_protocol.c     — CRC validation, field extraction
│   ├── telemetry_types.c        — Unit conversion, formatting helpers
│   ├── lcd_driver.c             — LCD stub/abstraction implementation
│   ├── crc16.c                  — CRC-16-CCITT (table-driven)
│   └── debug_uart.c             — Printf-style debug output with mutex
└── tools/
    └── telemetry_simulator.py   — Python script to generate test frames over serial
```

---

### Component 2: Configuration Headers

#### [NEW] [system_config.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/system_config.h)
- System clock: 168 MHz (HSE 8 MHz → PLL), APB1 42 MHz, APB2 84 MHz
- UART baud rate: 115200
- FreeRTOS tick: 1 kHz
- Task priorities: UART_RX=4, Processing=3, LCD=2, Monitor=1
- Task stack sizes: UART_RX=256, Processing=512, LCD=256, Monitor=256
- Queue sizes: UART_TO_PROC=10, PROC_TO_LCD=5
- Frame buffer: max 256 bytes

#### [NEW] [pin_config.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/pin_config.h)
- USART2: PA2 (TX), PA3 (RX)
- Status LEDs: PD12 (Green/Activity), PD13 (Orange/Processing), PD14 (Red/Error), PD15 (Blue/Power)
- I2C1: PB6 (SCL), PB7 (SDA) — reserved for future LCD

#### [NEW] [telemetry_config.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/telemetry_config.h)
- Start marker: `0xAA 0x55`
- Max payload length: 240 bytes
- CRC polynomial: 0x1021 (CRC-16-CCITT)
- Telemetry field IDs and types
- Defines the standard telemetry payload: timestamp (4B) + temperature (4B float) + voltage (4B float) + status (1B)

---

### Component 3: Core Protocol Implementation

#### [NEW] [crc16.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/crc16.h) / [crc16.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/crc16.c)
- Table-driven CRC-16-CCITT implementation
- `uint16_t crc16_calculate(const uint8_t *data, size_t length)`
- `bool crc16_validate(const uint8_t *data, size_t length, uint16_t expected)`
- Pre-computed 256-entry lookup table for performance

#### [NEW] [frame_parser.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/frame_parser.h) / [frame_parser.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/frame_parser.c)
- State machine with states: `WAIT_START1`, `WAIT_START2`, `WAIT_LENGTH`, `RECEIVE_PAYLOAD`, `WAIT_CRC_HIGH`, `WAIT_CRC_LOW`
- `frame_parser_init()`, `frame_parser_feed_byte()`, `frame_parser_reset()`
- Returns `FRAME_COMPLETE` when a full frame is assembled
- Timeout detection via tick counter

#### [NEW] [telemetry_protocol.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/telemetry_protocol.h) / [telemetry_protocol.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/telemetry_protocol.c)
- `telemetry_status_t telemetry_validate_frame(const RawFrame_t *frame)`
- `telemetry_status_t telemetry_extract_fields(const RawFrame_t *frame, TelemetryData_t *data)`
- CRC verification, length validation, field extraction with endianness handling

#### [NEW] [telemetry_types.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/telemetry_types.h) / [telemetry_types.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/telemetry_types.c)
- `RawFrame_t`: start marker, length, payload buffer, CRC
- `TelemetryData_t`: timestamp, temperature, voltage, status, valid flags
- `SystemStats_t`: packet counts, error counts, uptime
- Formatting helpers for display output

---

### Component 4: FreeRTOS Tasks & Application Logic

#### [NEW] [app_tasks.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/app_tasks.h) / [app_tasks.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/app_tasks.c)

**Task 1 — vUartReceptionTask (Priority 4)**:
- Interrupt-driven UART byte reception via USART2
- Feeds bytes into frame_parser state machine
- On `FRAME_COMPLETE`: sends `RawFrame_t` to processing queue
- Toggles Activity LED (PD12) on each received frame

**Task 2 — vTelemetryProcessingTask (Priority 3)**:
- Blocks on processing queue (`xQueueReceive`)
- Validates frame via `telemetry_validate_frame()`
- On CRC fail: increment error counter, set Error LED, discard
- On success: extract fields, send `TelemetryData_t` to LCD queue
- Outputs debug info via `debug_uart_printf()`

**Task 3 — vLcdDisplayTask (Priority 2)**:
- Blocks on LCD queue
- Formats telemetry for LCD (abstracted — prints to debug UART for now)
- Calls `lcd_driver` interface functions
- Updates display at ≤5 Hz

**Task 4 — vSystemMonitorTask (Priority 1)**:
- Periodic (1000ms) system health report
- Stack high water marks for all tasks
- Packet/error counters, uptime
- LED heartbeat management

**IPC Objects**:
- `xUartToProcessingQueue` (QueueHandle_t, 10 × RawFrame_t)
- `xProcessingToLcdQueue` (QueueHandle_t, 5 × TelemetryData_t)
- `xUartTxMutex` (SemaphoreHandle_t — protects debug UART TX)
- `xSystemEventGroup` (EventGroupHandle_t — error/status signaling)

#### [NEW] [main.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/main.c)
- SystemClock_Config (168 MHz, matching your existing projects)
- GPIO init (PD12-PD15 LEDs)
- USART2 init (115200-8N1, interrupt-driven RX)
- NVIC configuration
- `xAppTasksInit()` call
- `vTaskStartScheduler()`
- FreeRTOS hook functions (idle, stack overflow, malloc failed)

---

### Component 5: Display & Debug

#### [NEW] [lcd_driver.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/lcd_driver.h) / [lcd_driver.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/lcd_driver.c)
- Abstract interface: `lcd_init()`, `lcd_clear()`, `lcd_set_cursor()`, `lcd_print_string()`, `lcd_print_telemetry()`
- Current implementation: stubs that route to debug UART output
- Ready for future I2C PCF8574 implementation without changing callers

#### [NEW] [debug_uart.h](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/inc/debug_uart.h) / [debug_uart.c](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/src/debug_uart.c)
- `debug_uart_init()`, `debug_uart_printf()`, `debug_uart_send()`
- Mutex-protected UART TX (thread-safe from multiple tasks)
- Conditional compilation via `DEBUG_ENABLED` macro

---

### Component 6: Test Tooling

#### [NEW] [telemetry_simulator.py](file:///c:/Users/Administrator/Desktop/yazilim/EmbeddedProjects/Satellite-Projects/Ground-Station-Telemetry/tools/telemetry_simulator.py)
- Python script using `pyserial` to send test telemetry frames
- Generates valid frames with realistic temperature/voltage values
- Can inject CRC errors, truncated frames, and burst traffic
- Configurable COM port and baud rate via CLI arguments

---

## Verification Plan

### Automated Verification
1. **CRC-16 unit validation**: Known test vectors (e.g., "123456789" → 0x29B1 for CCITT)
2. **Frame parser validation**: Feed byte sequences and verify state transitions
3. **Build verification**: Clean compile with no warnings (`-Wall -Wextra -Werror`)

### On-Target Verification
1. Flash to Discovery board → verify UART debug output appears at 115200
2. Run `telemetry_simulator.py` → verify frames are received, validated, and displayed
3. Send corrupted frames → verify CRC errors are detected and counted
4. Monitor stack high water marks via System Monitor task output
5. Verify LED behavior: Green=activity, Red=CRC error, Blue=power on

### Manual Verification
1. Review debug UART output for correct telemetry field values
2. Confirm system runs stable for extended period (>30 min)
3. Test frame burst (rapid consecutive frames) → no queue overflow
