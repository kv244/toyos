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

#ifndef TOYOS_WATCHDOG_TIMEOUT
#define TOYOS_WATCHDOG_TIMEOUT 4000
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
