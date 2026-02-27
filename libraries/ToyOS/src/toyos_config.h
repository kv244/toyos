/**
 * ToyOS Configuration Header
 *
 * This file contains user-configurable parameters for the ToyOS RTOS.
 * Customize these values based on your application requirements and
 * available hardware resources.
 *
 * To override defaults:
 * 1. Copy this file to your project directory as "toyos_user_config.h"
 * 2. Uncomment and modify the desired parameters
 * 3. Define TOYOS_USER_CONFIG before including toyos.h:
 *    #define TOYOS_USER_CONFIG
 *    #include <toyos.h>
 *
 * Author: [Your Name]
 * Version: 2.4
 * Date: January 2026
 */

#ifndef TOYOS_CONFIG_H
#define TOYOS_CONFIG_H

/* ========================================================================
 * TASK CONFIGURATION
 * ======================================================================== */

/**
 * Maximum number of tasks that can be created in the system.
 * Each task consumes one TaskNode in the static pool.
 *
 * Memory impact: ~40 bytes per task in static pool
 * Recommended values:
 * - Arduino UNO (2KB SRAM): 4-8 tasks
 * - Arduino Mega (8KB SRAM): 16-32 tasks
 *
 * Default: 6 tasks
 */
#ifndef TOYOS_MAX_TASKS
#define TOYOS_MAX_TASKS 4
#endif

/**
 * Default stack size for tasks (in bytes).
 * Used when no explicit stack size is specified in os_create_task().
 *
 * Guidelines:
 * - Minimum: 64 bytes (simple tasks with minimal local variables)
 * - Typical: 96-128 bytes (moderate complexity, some function calls)
 * - Large: 192-256 bytes (deep call chains, large local arrays)
 *
 * Stack grows downward on AVR.
 * Stack requirements depend on:
 * - Local variable usage
 * - Function call depth
 * - Interrupt nesting
 *
 * Default: 96 bytes
 */
#ifndef TOYOS_DEFAULT_STACK_SIZE
#define TOYOS_DEFAULT_STACK_SIZE 192
#endif

/**
 * Minimum stack size allowed (safety check).
 * Enforced in os_create_task() to prevent crashes.
 *
 * Must be at least large enough for:
 * - Stack canary (4 bytes)
 * - Initial context (35 bytes: 32 registers + SREG + 2-byte PC)
 * - Working space for library calls (Serial, etc.)
 *
 * Default: 160 bytes
 */
#ifndef TOYOS_MIN_STACK_SIZE
#define TOYOS_MIN_STACK_SIZE 160
#endif

/* ========================================================================
 * PRIORITY CONFIGURATION
 * ======================================================================== */

/**
 * Default priority for tasks if not specified.
 * Used for tasks created without explicit priority.
 *
 * Priority range: 0-255 (higher = more important)
 *
 * Priority guidelines:
 * - 0-3: Lowest (background tasks, logging, LED blinking)
 * - 4-7: Normal (most application tasks) <-- DEFAULT
 * - 8-15: High (sensor reading, control loops)
 * - 16-31: Very High (communication protocols)
 * - 32+: Critical (safety, hard real-time requirements)
 *
 * Default: 5
 */
#ifndef TOYOS_DEFAULT_PRIORITY
#define TOYOS_DEFAULT_PRIORITY 5
#endif

/**
 * Idle task priority.
 * Should be lowest priority in the system.
 *
 * The idle task runs when no other tasks are ready.
 * Typically used for:
 * - Watchdog feeding
 * - Stack overflow checking
 * - Power management (sleep modes)
 *
 * Default: 0 (lowest priority)
 */
#ifndef TOYOS_IDLE_TASK_PRIORITY
#define TOYOS_IDLE_TASK_PRIORITY 0
#endif

/* ========================================================================
 * MEMORY CONFIGURATION
 * ======================================================================== */

/**
 * Size of memory pool for dynamic allocation (in bytes).
 * Used for task stacks, message queues, and system structures.
 *
 * Recommended sizing:
 * - Minimum: (MAX_TASKS * DEFAULT_STACK_SIZE) + 128
 * - Typical: (MAX_TASKS * 128) + 256 for overhead
 *
 * Example for 6 tasks with 96-byte stacks:
 * - Stacks: 6 * 96 = 576 bytes
 * - Overhead: ~128 bytes
 * - Total: 704 bytes (round up to 768 or 1024)
 *
 * Default: 512 bytes
 * Note: This is just a suggested default - define in your application!
 */
#ifndef TOYOS_MEMORY_POOL_SIZE
#define TOYOS_MEMORY_POOL_SIZE 1024
#endif

/**
 * Memory alignment boundary (in bytes).
 * All allocations are aligned to this boundary.
 *
 * AVR requires 2-byte alignment for optimal performance.
 * Do not change unless you know what you're doing!
 *
 * Default: 2 bytes
 */
#ifndef TOYOS_MEM_ALIGNMENT
#define TOYOS_MEM_ALIGNMENT 2
#endif

/* ========================================================================
 * TIMING CONFIGURATION
 * ======================================================================== */

/**
 * System tick frequency in Hz.
 * Determines scheduler quantum and delay resolution.
 *
 * Higher frequency = better responsiveness, more overhead
 * Lower frequency = less overhead, coarser timing
 *
 * Supported values (at 16MHz):
 * - 1000 Hz (1ms tick) - Default, good balance
 * - 500 Hz (2ms tick) - Lower overhead
 * - 100 Hz (10ms tick) - Very low overhead, coarse timing
 *
 * Default: 1000 Hz (1ms tick)
 */
#ifndef TOYOS_TICK_RATE_HZ
#define TOYOS_TICK_RATE_HZ 1000
#endif

/**
 * Timer prescaler value.
 * Determines timer clock divider.
 *
 * For 16MHz CPU with 1ms tick:
 * - Prescaler 64: OCR1A = 249
 * - Prescaler 256: OCR1A = 62
 *
 * Default: 64
 */
#ifndef TOYOS_TIMER_PRESCALER
#define TOYOS_TIMER_PRESCALER 64
#endif

/* ========================================================================
 * STACK OVERFLOW DETECTION
 * ======================================================================== */

/**
 * Stack canary value for overflow detection.
 * Placed at the bottom (lowest address) of each task's stack.
 * If this value is corrupted, stack overflow is detected.
 *
 * Value chosen to be unlikely to occur naturally.
 * 0xDEADBEEF = 3735928559 decimal
 *
 * You can customize this, but use a distinctive pattern.
 *
 * Default: 0xDEADBEEF
 */
#ifndef TOYOS_STACK_CANARY
#define TOYOS_STACK_CANARY 0xDEADBEEF
#endif

/**
 * Enable automatic stack checking on every context switch.
 * If enabled, os_check_stack_overflow() is called automatically.
 *
 * Warning: Adds overhead to context switches (~10-20 cycles)
 * Recommended: Enable during development, disable in production
 *
 * Default: 0 (disabled)
 */
#ifndef TOYOS_AUTO_STACK_CHECK
#define TOYOS_AUTO_STACK_CHECK 0
#endif

/* ========================================================================
 * WATCHDOG CONFIGURATION
 * ======================================================================== */

/**
 * Enable watchdog timer support.
 * If enabled, includes watchdog initialization and feeding functions.
 *
 * Recommended: Enable for production systems
 *
 * Default: 1 (enabled)
 */
#ifndef TOYOS_ENABLE_WATCHDOG
#define TOYOS_ENABLE_WATCHDOG 1
#endif

/**
 * Default watchdog timeout period.
 * Valid values: WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS,
 *               WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S,
 *               WDTO_4S, WDTO_8S
 *
 * Default: WDTO_4S (4 seconds)
 */
#ifndef TOYOS_WATCHDOG_TIMEOUT
#define WDTO_4S 8 // AVR watchdog constant
#define TOYOS_WATCHDOG_TIMEOUT WDTO_4S
#endif

/* ========================================================================
 * DEBUG AND DIAGNOSTICS
 * ======================================================================== */

/**
 * Enable debug assertions.
 * If enabled, ASSERT() macros perform runtime checks.
 * If disabled, ASSERT() macros compile to nothing (zero overhead).
 *
 * Recommended: Enable during development, disable in production
 *
 * Default: 0 (disabled)
 */
#ifndef TOYOS_DEBUG
#define TOYOS_DEBUG 0
#endif

/* ========================================================================
 * MEMORY PROTECTION (ARM ONLY)
 * ======================================================================== */

/**
 * Enable MPU-based memory protection (ARM Cortex-M only).
 * Provides hardware-enforced task isolation and stack protection.
 *
 * Features:
 * - Task stack isolation (prevents stack overflow corruption)
 * - Kernel data protection (user tasks cannot modify kernel structures)
 * - Heap protection with guard pages
 * - Immediate fault detection with precise error location
 *
 * Requirements:
 * - ARM Cortex-M with MPU (M3/M4/M7)
 * - Minimum 8 MPU regions
 *
 * Overhead:
 * - Code size: ~300 bytes
 * - Context switch: ~10-20 cycles
 * - Runtime: Zero (hardware enforcement)
 *
 * Note: Automatically disabled on AVR (no MPU hardware)
 *
 * Default: 1 on ARM, 0 on AVR
 */
#ifndef TOYOS_USE_MPU
#define TOYOS_USE_MPU 0
#endif

/**
 * Enable runtime statistics collection.
 * Tracks task execution time, context switches, etc.
 *
 * Warning: Adds memory and CPU overhead
 *
 * Default: 0 (disabled)
 */
#ifndef TOYOS_ENABLE_STATS
#define TOYOS_ENABLE_STATS 0
#endif

/* ========================================================================
 * FEATURE TOGGLES
 * ======================================================================== */

/**
 * Enable priority inheritance for mutexes.
 * Prevents priority inversion by temporarily boosting
 * mutex owner's priority.
 *
 * Adds overhead to mutex lock/unlock operations.
 *
 * Default: 1 (enabled)
 */
#ifndef TOYOS_ENABLE_PRIORITY_INHERITANCE
#define TOYOS_ENABLE_PRIORITY_INHERITANCE 0
#endif

/**
 * Enable message queues.
 * If disabled, message queue functions are not included,
 * saving flash memory.
 *
 * Default: 1 on ARM, 0 on AVR (saves ~500 bytes Flash)
 */
#ifndef TOYOS_ENABLE_MESSAGE_QUEUES
#if defined(__arm__) || defined(__ARM_ARCH)
#define TOYOS_ENABLE_MESSAGE_QUEUES 1
#else
#define TOYOS_ENABLE_MESSAGE_QUEUES 0
#endif
#endif

/**
 * Maximum message queue capacity.
 * Limits the size of message queues to prevent excessive memory use.
 *
 * Default: 16 messages
 */
#ifndef TOYOS_MAX_QUEUE_CAPACITY
#define TOYOS_MAX_QUEUE_CAPACITY 16
#endif

/* ========================================================================
 * VALIDATION AND DERIVED CONSTANTS
 * ======================================================================== */

/* Compile-time validation of configuration */
#if TOYOS_MAX_TASKS < 1 || TOYOS_MAX_TASKS > 32
#error "TOYOS_MAX_TASKS must be between 1 and 32"
#endif

#if TOYOS_MIN_STACK_SIZE < 48
#error "TOYOS_MIN_STACK_SIZE must be at least 48 bytes"
#endif

#if TOYOS_DEFAULT_STACK_SIZE < TOYOS_MIN_STACK_SIZE
#error "TOYOS_DEFAULT_STACK_SIZE must be >= TOYOS_MIN_STACK_SIZE"
#endif

#if TOYOS_TIMER_PRESCALER != 1 && TOYOS_TIMER_PRESCALER != 8 &&                \
    TOYOS_TIMER_PRESCALER != 64 && TOYOS_TIMER_PRESCALER != 256 &&             \
    TOYOS_TIMER_PRESCALER != 1024
#error "Invalid TOYOS_TIMER_PRESCALER value"
#endif

/* Calculate Timer1 compare value based on configuration */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define TOYOS_TIMER_COMPARE_VALUE                                              \
  ((F_CPU / TOYOS_TIMER_PRESCALER / TOYOS_TICK_RATE_HZ) - 1)

/* Derive timer prescaler bits */
#if TOYOS_TIMER_PRESCALER == 1
#define TOYOS_TIMER_PRESCALER_BITS (1 << CS10)
#elif TOYOS_TIMER_PRESCALER == 8
#define TOYOS_TIMER_PRESCALER_BITS (1 << CS11)
#elif TOYOS_TIMER_PRESCALER == 64
#define TOYOS_TIMER_PRESCALER_BITS ((1 << CS11) | (1 << CS10))
#elif TOYOS_TIMER_PRESCALER == 256
#define TOYOS_TIMER_PRESCALER_BITS (1 << CS12)
#elif TOYOS_TIMER_PRESCALER == 1024
#define TOYOS_TIMER_PRESCALER_BITS ((1 << CS12) | (1 << CS10))
#endif

/* Tick conversion macros */
#define TOYOS_MS_TO_TICKS(ms) ((uint16_t)((ms) * TOYOS_TICK_RATE_HZ / 1000))
#define TOYOS_SEC_TO_TICKS(sec) ((uint16_t)((sec) * TOYOS_TICK_RATE_HZ))
#define TOYOS_TICKS_TO_MS(ticks)                                               \
  ((uint32_t)((ticks) * 1000 / TOYOS_TICK_RATE_HZ))

/* ========================================================================
 * COMPILER OPTIMIZATION MACROS
 * ======================================================================== */

#ifndef TOYOS_NORETURN
#if defined(__GNUC__) || defined(__clang__)
#define TOYOS_NORETURN __attribute__((noreturn))
#define TOYOS_HOT __attribute__((hot))
#define TOYOS_COLD __attribute__((cold))
#define TOYOS_ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define TOYOS_NORETURN
#define TOYOS_HOT
#define TOYOS_COLD
#define TOYOS_ALWAYS_INLINE inline
#endif
#endif

/**
 * Enable/Disable general kernel debugging (Serial output)
 * 0: Disabled
 * 1: Enabled
 */
#ifndef TOYOS_DEBUG_ENABLED
#define TOYOS_DEBUG_ENABLED 0
#endif

#endif /* TOYOS_CONFIG_H */
