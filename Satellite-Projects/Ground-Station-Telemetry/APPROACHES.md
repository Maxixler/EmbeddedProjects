# Ground Station Telemetry Processing Unit - Architectural Approaches

## Approach 1: Bare-metal Superloop with State Machines

### Description
Traditional bare-metal approach using a main superloop that polls for events and executes state machine functions for each subsystem.

### Components
- **Main Loop**: Continuous polling of UART, timers, and flags
- **UART Receiver State Machine**: Handles byte-by-byte reception, frame assembly
- **UDP Processor State Machine**: Validates packets, extracts telemetry data
- **LCD Display State Machine**: Manages LCD updates and refresh
- **Timer Module**: Provides timeouts and periodic tasks
- **CRC Module**: Handles error detection

### Trade-offs
**Pros:**
- Minimal resource overhead (no RTOS kernel)
- Deterministic timing and predictable behavior
- Deep understanding of hardware and timing
- Simple to debug and trace
- Minimal dependencies

**Cons:**
- More complex to manage as features grow
- Manual timing management required
- Limited scalability for concurrent operations
- More boilerplate code for task management

### Best For
Learning fundamental embedded concepts, simple applications with few concurrent tasks, when deterministic timing is critical.

## Approach 2: FreeRTOS-based Task Architecture

### Description
Using FreeRTOS to create separate tasks for each subsystem, communicating via queues and synchronization primitives.

### Components
- **UART Reception Task**: Receives bytes via UART DMA/interrupt, assembles frames, sends to UDP queue
- **UDP Processing Task**: Validates packets from queue, extracts telemetry, sends to display queue
- **LCD Display Task**: Updates LCD based on telemetry data from queue
- **Command Handling Task**: (Optional) Processes incoming commands
- **System Monitor Task**: (Optional) Monitors system health, stack usage
- **Queues**: For inter-task communication (UART→UDP, UDP→LCD)
- **Mutexes/Semaphores**: For shared resource protection (LCD, UART transmit)
- **Timers**: For periodic tasks and timeouts

### Trade-offs
**Pros:**
- Natural separation of concerns
- Easy to add new features/tasks
- Built-in timing and delay functions
- Better resource utilization
- Industry-standard approach for complex embedded systems
- Easier to maintain and extend

**Cons:**
- RTOS overhead (memory and CPU)
- Learning curve for RTOS concepts
- Potential priority inversion issues
- More complex debugging (task interactions)
- Slightly less predictable timing than bare-metal

### Best For
Applications with multiple concurrent processes, when scalability and maintainability are important, industry-standard embedded development.

## Approach 3: Hybrid HAL + RTOS with Application Layer State Machines

### Description
Combines STM32 HAL drivers for peripheral initialization with FreeRTOS for task management, while keeping protocol handling in application-layer state machines.

### Components
- **STM32 HAL/UART Drivers**: Handle low-level UART configuration and DMA/interrupts
- **FreeRTOS Tasks**: As in Approach 2, but with thinner abstraction layers
- **Application State Machines**: For UDP frame validation, telemetry parsing
- **Driver Abstraction Layer**: Thin wrapper around HAL for testability
- **Service Layer**: For LCD display, timer services
- **Configuration Module**: For system parameters (baud rates, timeouts, etc.)

### Trade-offs
**Pros:**
- Leverages vendor HAL for reliable peripheral setup
- Maintains RTOS benefits for task management
- Clear separation between hardware, OS, and application layers
- Easier to port to different MCUs (change HAL layer)
- Good balance of control and productivity
- Testable application logic

**Cons:**
- Additional abstraction layer complexity
- Dependency on HAL libraries
- Slightly more indirection in code
- Need to understand both HAL and RTOS

### Best For
Projects where hardware abstraction is valuable, when leveraging vendor peripheral libraries, for applications that may need MCU portability in future.

## Recommendation

**Recommended Approach: Approach 2 (FreeRTOS-based Task Architecture)**

### Rationale:
1. **Skill Development Focus**: Directly addresses your goal of developing RTOS skills specifically
2. **Industry Relevance**: RTOS usage is standard in aerospace/defense embedded systems
3. **Scalability**: Easy to extend with additional features (command handling, monitoring, multiple telemetry types)
4. **Learning Value**: Teaches inter-task communication, synchronization, and real-time concepts
5. **Matches Existing Experience**: Builds on your existing FreeRTOS exposure from ESP32 projects
6. **Appropriate Complexity**: Well-suited for the 20-day timeline with clear milestones

This approach will give you demonstrable RTOS experience while building a system that closely resembles real satellite ground station architectures.