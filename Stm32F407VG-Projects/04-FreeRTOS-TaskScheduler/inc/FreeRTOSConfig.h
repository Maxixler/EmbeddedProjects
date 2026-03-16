/*******************************************************************************
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS Kernel Configuration for STM32F407VG
 * @details This configuration file tailors FreeRTOS to the STM32F407VG
 *          running at 168 MHz with Cortex-M4F core. It enables preemptive
 *          scheduling, mutexes, counting semaphores, software timers, event
 *          groups, runtime statistics, and stack overflow detection (Method 2).
 *
 * @note    Interrupt priority configuration is critical for correct operation:
 *          - STM32F407 uses 4 priority bits (16 levels, 0 = highest)
 *          - FreeRTOS kernel interrupts (SysTick, PendSV) run at lowest priority
 *          - ISRs calling FreeRTOS API must have priority >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 *
 * @author  Embedded Systems Portfolio Project
 ******************************************************************************/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*===========================================================================*/
/*                        INCLUDE FILES                                      */
/*===========================================================================*/

/* Required for SystemCoreClock and __NVIC_PRIO_BITS */
#include "stm32f4xx.h"

/* Ensure definitions exist for the STM32F407 interrupt handlers */
extern uint32_t SystemCoreClock;

/*===========================================================================*/
/*                    BASIC KERNEL CONFIGURATION                             */
/*===========================================================================*/

/**
 * @brief Enable preemptive scheduling.
 *        1 = Preemptive: Higher priority tasks preempt lower priority tasks.
 *        0 = Cooperative: Tasks must explicitly yield.
 *        Preemptive is the standard choice for most real-time applications.
 */
#define configUSE_PREEMPTION 1

/**
 * @brief CPU clock frequency in Hz.
 *        STM32F407VG runs at 168 MHz with the PLL configuration:
 *        HSE (8 MHz) -> PLL (x336 / 2) -> 168 MHz SYSCLK
 */
#define configCPU_CLOCK_HZ ((uint32_t)168000000)

/**
 * @brief RTOS tick frequency in Hz.
 *        1000 Hz = 1 ms tick period, providing 1 ms time resolution.
 *        Higher values give finer resolution but increase overhead.
 *        Common values: 100 Hz (10 ms), 250 Hz (4 ms), 1000 Hz (1 ms).
 */
#define configTICK_RATE_HZ ((TickType_t)1000)

/**
 * @brief Maximum number of priority levels.
 *        Tasks can have priorities from 0 (lowest, idle) to
 *        configMAX_PRIORITIES - 1 (highest).
 *        5 levels: 0=Idle/Monitor, 1=Logger, 2=Display, 3=Processing, 4=Sensor
 *        More priorities = more RAM for ready lists.
 */
#define configMAX_PRIORITIES 5

/**
 * @brief Minimum stack size for any task (in words, not bytes).
 *        128 words = 512 bytes on 32-bit architecture.
 *        Used by the Idle task and as a default minimum.
 *        Actual tasks should use larger stacks based on their requirements.
 */
#define configMINIMAL_STACK_SIZE ((uint16_t)128)

/**
 * @brief Total heap size available for FreeRTOS dynamic allocation (in bytes).
 *        32 KB allocated for FreeRTOS objects (tasks, queues, semaphores, etc.).
 *        STM32F407VG has 192 KB SRAM total (128 KB main + 64 KB CCM).
 *        Heap is allocated from the main SRAM region.
 */
#define configTOTAL_HEAP_SIZE ((size_t)(32 * 1024))

/**
 * @brief Maximum length of a task name string (including null terminator).
 *        Used for debugging and runtime statistics display.
 */
#define configMAX_TASK_NAME_LEN 16

/**
 * @brief Use 32-bit tick counter for extended uptime range.
 *        0 = 32-bit: Up to ~49.7 days at 1000 Hz tick rate.
 *        1 = 16-bit: Up to ~65.5 seconds (rarely used).
 */
#define configUSE_16_BIT_TICKS 0

/**
 * @brief Allow the idle task to yield to same-priority application tasks.
 *        1 = Idle task yields when application tasks at priority 0 are ready.
 *        This prevents idle task from consuming an entire time slice.
 */
#define configIDLE_SHOULD_YIELD 1

/**
 * @brief Enable time slicing for tasks at the same priority level.
 *        1 = Round-robin scheduling among equal-priority tasks.
 *        Each task gets one tick period (1 ms) before switching.
 */
#define configUSE_TIME_SLICING 1

/*===========================================================================*/
/*                    FEATURE ENABLE/DISABLE                                 */
/*===========================================================================*/

/**
 * @brief Enable mutex support.
 *        Mutexes provide mutual exclusion with priority inheritance to
 *        prevent unbounded priority inversion. Required for protecting
 *        shared resources like UART.
 */
#define configUSE_MUTEXES 1

/**
 * @brief Enable recursive mutex support.
 *        Allows the same task to take a mutex multiple times without
 *        deadlocking. The mutex must be given the same number of times.
 */
#define configUSE_RECURSIVE_MUTEXES 1

/**
 * @brief Enable counting semaphore support.
 *        Used for resource counting (e.g., available buffer slots)
 *        and event counting.
 */
#define configUSE_COUNTING_SEMAPHORES 1

/**
 * @brief Enable task notification support.
 *        Task notifications are a lightweight alternative to binary
 *        semaphores, counting semaphores, event groups, and mailboxes.
 *        ~45% faster than binary semaphores with zero RAM overhead.
 */
#define configUSE_TASK_NOTIFICATIONS 1

/**
 * @brief Number of notification indexes per task.
 *        Each index is an independent notification value/state.
 *        Default is 1 for backward compatibility.
 */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 1

/**
 * @brief Enable queue set support.
 *        Queue sets allow a task to block on multiple queues/semaphores
 *        simultaneously (similar to select() in POSIX).
 *        Disabled to save code size as not used in this project.
 */
#define configUSE_QUEUE_SETS 0

/**
 * @brief Enable dynamic memory allocation API (pvPortMalloc/vPortFree).
 *        Required for xTaskCreate(), xQueueCreate(), etc.
 */
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/**
 * @brief Disable static memory allocation API.
 *        When enabled, xTaskCreateStatic(), xQueueCreateStatic(), etc.
 *        are available, allowing objects to be created without heap.
 *        Disabled to keep this project simpler.
 */
#define configSUPPORT_STATIC_ALLOCATION 0

/*===========================================================================*/
/*                    SOFTWARE TIMER CONFIGURATION                           */
/*===========================================================================*/

/**
 * @brief Enable software timer support.
 *        Creates a timer service (daemon) task that processes timer commands.
 *        Used for the heartbeat LED timer in this project.
 */
#define configUSE_TIMERS 1

/**
 * @brief Priority of the timer service/daemon task.
 *        Set to maximum priority minus 1 to ensure timers are serviced
 *        promptly, but still below the highest-priority application task
 *        if needed. Using (configMAX_PRIORITIES - 1) = 4.
 */
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)

/**
 * @brief Length of the timer command queue.
 *        Determines how many timer commands can be queued before the
 *        calling task blocks. 10 is sufficient for this project.
 */
#define configTIMER_QUEUE_LENGTH 10

/**
 * @brief Stack depth (in words) for the timer service task.
 *        The timer task executes timer callback functions, so the stack
 *        must be large enough for the deepest callback. 256 words = 1 KB.
 */
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE * 2)

/*===========================================================================*/
/*                    HOOK (CALLBACK) FUNCTIONS                              */
/*===========================================================================*/

/**
 * @brief Enable the idle hook callback.
 *        vApplicationIdleHook() is called on each iteration of the idle task.
 *        Used for entering low-power modes (WFI instruction) and
 *        incrementing an idle counter for CPU utilization estimation.
 *        WARNING: The idle hook must NEVER call a blocking function.
 */
#define configUSE_IDLE_HOOK 1

/**
 * @brief Disable the tick hook callback.
 *        vApplicationTickHook() would be called from the tick interrupt.
 *        Not needed in this project.
 */
#define configUSE_TICK_HOOK 0

/**
 * @brief Enable the malloc failed hook callback.
 *        vApplicationMallocFailedHook() is called if pvPortMalloc() returns
 *        NULL. Critical for catching heap exhaustion at development time.
 */
#define configUSE_MALLOC_FAILED_HOOK 1

/**
 * @brief Enable stack overflow detection using Method 2 (pattern check).
 *        Method 1: Checks if SP is within stack bounds at context switch.
 *        Method 2: Fills the stack with a known pattern (0xA5) and checks
 *                  if the last 20 bytes are intact at context switch.
 *        Method 2 is more reliable but slightly slower.
 *        vApplicationStackOverflowHook() is called on detection.
 */
#define configCHECK_FOR_STACK_OVERFLOW 2

/**
 * @brief Enable the daemon (timer) task startup hook.
 *        Called once when the timer task starts. Disabled as not needed.
 */
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0

/*===========================================================================*/
/*                    RUNTIME STATISTICS CONFIGURATION                       */
/*===========================================================================*/

/**
 * @brief Enable collection of runtime statistics (CPU usage per task).
 *        Requires a free-running counter with resolution at least 10x
 *        the tick rate. TIM5 is configured at 10 kHz (100 us resolution)
 *        for this purpose.
 */
#define configGENERATE_RUN_TIME_STATS 1

/**
 * @brief Enable the trace facility.
 *        Required for vTaskList() and vTaskGetRunTimeStats() functions
 *        that provide formatted task information.
 */
#define configUSE_TRACE_FACILITY 1

/**
 * @brief Enable the formatted runtime statistics functions.
 *        vTaskGetRunTimeStats() generates a table of task CPU usage.
 *        vTaskList() generates a table of task states and stack usage.
 */
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/**
 * @brief Macro to initialize the runtime stats timer.
 *        Called once during scheduler startup. TIM5 is started in main.c
 *        before the scheduler, so this is a no-op here.
 *        The actual timer initialization is done in main.c via HAL_TIM_Base_Start().
 */
extern void vConfigureTimerForRunTimeStats(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vConfigureTimerForRunTimeStats()

/**
 * @brief Macro to read the runtime stats timer counter.
 *        Returns the current TIM5 counter value (32-bit, counts at 10 kHz).
 *        Each count = 100 microseconds.
 */
extern uint32_t ulGetRunTimeCounterValue(void);
#define portGET_RUN_TIME_COUNTER_VALUE() ulGetRunTimeCounterValue()

/*===========================================================================*/
/*                    CO-ROUTINE CONFIGURATION                               */
/*===========================================================================*/

/**
 * @brief Disable co-routine support.
 *        Co-routines are a legacy FreeRTOS feature that share a single stack.
 *        They are deprecated in favor of tasks and should not be used in
 *        new designs.
 */
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES 2

/*===========================================================================*/
/*                    INTERRUPT PRIORITY CONFIGURATION                       */
/*===========================================================================*/

/*
 * CRITICAL: Cortex-M interrupt priority configuration for FreeRTOS
 *
 * STM32F407VG uses 4 priority bits (__NVIC_PRIO_BITS = 4), giving 16 levels.
 * ARM Cortex-M uses the UPPER bits of the 8-bit priority register:
 *   - Priority 0 (0x00) = HIGHEST priority (most urgent)
 *   - Priority 15 (0xF0) = LOWEST priority (least urgent)
 *
 * FreeRTOS splits interrupts into two groups:
 *
 *   Priority 0-4:  ABOVE FreeRTOS - Cannot use FreeRTOS API calls!
 *                  Use for ultra-low-latency interrupts (motor control, safety).
 *
 *   Priority 5-14: MANAGED BY FreeRTOS - Can use "FromISR" API functions.
 *                  DMA, UART, ADC, SPI, I2C interrupts go here.
 *
 *   Priority 15:   KERNEL - SysTick and PendSV (context switching).
 *                  Must be the LOWEST priority.
 */

/**
 * @brief Number of priority bits implemented in hardware.
 *        STM32F407VG: 4 bits -> 16 priority levels (0-15).
 *        The upper 4 bits of the 8-bit priority field are used.
 */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

/**
 * @brief Lowest interrupt priority (numerically highest value).
 *        Used for SysTick and PendSV handlers.
 *        15 << 4 = 0xF0 (shifted to upper bits of priority register).
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

/**
 * @brief Maximum priority for ISRs that can call FreeRTOS API.
 *        ISRs with priority 5-15 (inclusive) can call "FromISR" functions.
 *        ISRs with priority 0-4 MUST NOT call any FreeRTOS API.
 *        5 << 4 = 0x50 (shifted to upper bits of priority register).
 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/**
 * @brief Kernel interrupt priority (SysTick, PendSV).
 *        Must be the lowest priority to ensure all other ISRs can preempt
 *        the context switch mechanism.
 */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/**
 * @brief Maximum interrupt priority for FreeRTOS-managed ISRs.
 *        Interrupts at or below this priority (numerically >= 5) can safely
 *        call FreeRTOS "FromISR" API functions.
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*===========================================================================*/
/*                    ASSERT CONFIGURATION                                   */
/*===========================================================================*/

/**
 * @brief FreeRTOS assertion macro.
 *        Called throughout the kernel to catch configuration errors,
 *        invalid parameters, and impossible conditions during development.
 *        In production, this can be removed for performance.
 *
 *        Common assertions triggered by:
 *        - Calling API from ISR with priority above MAX_SYSCALL
 *        - Invalid priority values
 *        - NULL handle parameters
 *        - Stack overflow conditions
 */
#define configASSERT(x)           \
    if ((x) == 0)                 \
    {                             \
        taskDISABLE_INTERRUPTS(); \
        for (;;)                  \
            ;                     \
    }

/*===========================================================================*/
/*                    INCLUDE API FUNCTIONS                                   */
/*===========================================================================*/

/* Set to 1 to include the API function, 0 to exclude it.
 * Excluding unused functions reduces code size. */

#define INCLUDE_vTaskPrioritySet 1            /* xTaskPrioritySet()          */
#define INCLUDE_uxTaskPriorityGet 1           /* uxTaskPriorityGet()         */
#define INCLUDE_vTaskDelete 1                 /* vTaskDelete()               */
#define INCLUDE_vTaskSuspend 1                /* vTaskSuspend/Resume()       */
#define INCLUDE_xResumeFromISR 1              /* xTaskResumeFromISR()        */
#define INCLUDE_vTaskDelayUntil 1             /* vTaskDelayUntil()           */
#define INCLUDE_vTaskDelay 1                  /* vTaskDelay()                */
#define INCLUDE_xTaskGetSchedulerState 1      /* xTaskGetSchedulerState()    */
#define INCLUDE_xTaskGetCurrentTaskHandle 1   /* xTaskGetCurrentTaskHandle() */
#define INCLUDE_uxTaskGetStackHighWaterMark 1 /* uxTaskGetStackHighWaterMark() */
#define INCLUDE_xTaskGetIdleTaskHandle 0      /* xTaskGetIdleTaskHandle()    */
#define INCLUDE_eTaskGetState 1               /* eTaskGetState()             */
#define INCLUDE_xEventGroupSetBitFromISR 1    /* xEventGroupSetBitFromISR()  */
#define INCLUDE_xTimerPendFunctionCall 1      /* xTimerPendFunctionCall()    */
#define INCLUDE_xTaskAbortDelay 0             /* xTaskAbortDelay()           */
#define INCLUDE_xTaskGetHandle 1              /* xTaskGetHandle()            */
#define INCLUDE_xTaskResumeFromISR 1          /* xTaskResumeFromISR()        */

/*===========================================================================*/
/*                    CORTEX-M4 HANDLER MAPPING                              */
/*===========================================================================*/

/**
 * @brief Map FreeRTOS port handlers to STM32 CMSIS handler names.
 *        This ensures FreeRTOS handlers are called when SVC, PendSV,
 *        and SysTick interrupts fire.
 *
 *        IMPORTANT: If using STM32CubeMX-generated code, these handlers
 *        must NOT be defined in stm32f4xx_it.c (remove or comment them out).
 */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
