/**
 * ToyOS Platform Portability Layer
 *
 * This header defines the hardware abstraction layer (HAL) for ToyOS,
 * allowing the RTOS to run on multiple architectures.
 *
 * Supported platforms:
 * - AVR (ATmega328P - Arduino UNO)
 * - ARM Cortex-M (Renesas RA4M1 - Arduino UNO R4)
 *
 *  Author: [Your Name]
 * Version: 2.5
 * Date: January 2026
 */

#ifndef TOYOS_PORT_H
#define TOYOS_PORT_H

#include <stdbool.h>
#include <stdint.h>

/* ========================================================================
 * PLATFORM DETECTION
 * ======================================================================== */

/**
 * Automatic platform detection based on compiler defines.
 * Users can override by defining PORT_PLATFORM before including this header.
 */
#ifndef PORT_PLATFORM

#if defined(__AVR_ATmega328P__) || defined(__AVR__)
#define PORT_PLATFORM_AVR 1
#define PORT_PLATFORM PORT_PLATFORM_AVR
#define PORT_ARCH_NAME "AVR"
#define PORT_DEVICE_NAME "ATmega328P"

#elif defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) ||                 \
    defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_8M_MAIN__) ||               \
    defined(__ARM_ARCH_8M_BASE__)
#define PORT_PLATFORM_ARM_CORTEX_M 1
#define PORT_PLATFORM PORT_PLATFORM_ARM_CORTEX_M
#define PORT_ARCH_NAME "ARM Cortex-M"

/* Detect specific Cortex-M variant */
#if defined(__ARM_ARCH_7M__)
#define PORT_DEVICE_NAME "Cortex-M3"
#elif defined(__ARM_ARCH_7EM__)
#define PORT_DEVICE_NAME "Cortex-M4/M7"
#elif defined(__ARM_ARCH_6M__)
#define PORT_DEVICE_NAME "Cortex-M0/M0+"
#elif defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8M_BASE__)
#define PORT_DEVICE_NAME "Cortex-M23/M33"
#else
#define PORT_DEVICE_NAME "ARM Cortex-M"
#endif

#else
#error "Unsupported platform! Define PORT_PLATFORM manually."
#endif

#endif /* PORT_PLATFORM */

/* ========================================================================
 * PLATFORM-SPECIFIC INCLUDES
 * ======================================================================== */

/* Include platform-specific implementation header */
#if PORT_PLATFORM == PORT_PLATFORM_AVR
#include "port/avr/port_avr.h"
#elif PORT_PLATFORM == PORT_PLATFORM_ARM_CORTEX_M
#include "port/arm/port_arm.h"
#else
#error "No port implementation for selected platform"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * PLATFORM-SPECIFIC CONFIGURATION
 * ======================================================================== */

#if PORT_PLATFORM == PORT_PLATFORM_AVR
/* AVR Configuration */
#define PORT_STACK_GROWS_DOWN 1
#define PORT_POINTER_SIZE 2
#define PORT_HAS_NESTED_INTERRUPTS 0
#define PORT_CONTEXT_SIZE 35 /* 32 regs + SREG + 2-byte PC */
#define PORT_MIN_STACK_SIZE 48

#elif PORT_PLATFORM == PORT_PLATFORM_ARM_CORTEX_M
/* ARM Cortex-M Configuration */
#define PORT_STACK_GROWS_DOWN 1
#define PORT_POINTER_SIZE 4
#define PORT_HAS_NESTED_INTERRUPTS 1
#define PORT_CONTEXT_SIZE 64 /* R0-R12, LR, PC, xPSR + FPU if applicable */
#define PORT_MIN_STACK_SIZE 128

#else
#error "PORT_PLATFORM not properly configured"
#endif

/* ========================================================================
 * CRITICAL SECTION MANAGEMENT
 * ======================================================================== */

/**
 * Begin critical section (disable interrupts).
 * Platform-specific implementation saves interrupt state and disables
 * interrupts.
 */
/* Delegated to platform header */
// void port_enter_critical(void);
// void port_exit_critical(void);

/**
 * Globally enable interrupts.
 */
// void port_enable_interrupts(void);
// void port_disable_interrupts(void);

/* ========================================================================
 * CONTEXT SWITCHING
 * ======================================================================== */

/**
 * Initialize a task's stack.
 *
 * Creates initial stack frame for a new task, including:
 * - Return address (task_func)
 * - Initial register values
 * - Status register with interrupts enabled
 *
 * @param stack_top Pointer to top of allocated stack (highest address)
 * @param stack_size Size of stack in bytes
 * @param task_func Pointer to task entry function
 * @return Stack pointer ready for context restore (adjusted downward)
 */
// uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
//                          void (*task_func)(void));

/**
 * Perform context switch.
 *
 * Called from:
 * - Voluntary yield (os_task_yield)
 * - Blocking operations (os_delay, semaphore/mutex wait)
 * - Timer ISR (preemptive scheduling)
 *
 * Implementation:
 * 1. Save current task's context to its stack
 * 2. Update os_current_task_ptr->stack_ptr
 * 3. Call os_scheduler() to select next task
 * 4. Load new task's context from its stack
 * 5. Return to new task
 *
 * @note This function never returns to the same task!
 */
// void port_context_switch(void);

/**
 * Start the first task (called from os_start).
 *
 * Loads the context of the first task and begins execution.
 * Never returns.
 *
 * @param task_stack_ptr Stack pointer of first task
 */
// void port_start_first_task(uint8_t *task_stack_ptr)
// __attribute__((noreturn));

/* ========================================================================
 * TIMER / SYSTEM TICK
 * ======================================================================== */

/**
 * Initialize system tick timer.
 *
 * Configures hardware timer to generate interrupts at OS_TICK_RATE_HZ
 * frequency. Timer ISR should call os_system_tick() and trigger context switch.
 *
 * Platform-specific implementation:
 * - AVR: Timer1 in CTC mode
 * - ARM: SysTick timer
 */
// void port_timer_init(void);

/**
 * Get current system tick count.
 *
 * Returns number of timer ticks since os_start() was called.
 * Must be implemented with atomic access for multi-byte counters.
 *
 * @return Current tick count
 */
// uint32_t port_get_tick(void);

/* ========================================================================
 * ATOMIC OPERATIONS
 * ======================================================================== */

/**
 * Atomic increment of uint8_t variable.
 * @param ptr Pointer to variable
 * @return New value after increment
 */
// uint8_t port_atomic_inc_u8(volatile uint8_t *ptr);
// uint8_t port_atomic_dec_u8(volatile uint8_t *ptr);
// bool port_atomic_cas_u8(volatile uint8_t *ptr, uint8_t expected, uint8_t
// desired);

/* ========================================================================
 * STACK UTILITIES
 * ======================================================================== */

/**
 * Get current stack pointer value.
 * Used for debugging and stack overflow detection.
 *
 * @return Current stack pointer
 */
// void *port_get_stack_pointer(void);

/**
 * Fast stack zeroing (optional optimization).
 *
 * Some platforms provide optimized assembly for bulk zeroing.
 * If not implemented, falls back to C loop.
 *
 * @param sp Stack pointer to start zeroing from
 * @param count Number of bytes to zero
 * @return Updated stack pointer after zeroing
 */
// uint8_t *port_fast_zero_stack(uint8_t *sp, uint8_t count);

/* ========================================================================
 * LOW POWER / IDLE
 * ======================================================================== */

/**
 * Enter idle/sleep mode.
 *
 * Called from idle task when no other tasks are ready.
 * Should enable interrupts and enter low-power mode.
 * CPU will wake on next interrupt (timer tick).
 */
// void port_enter_idle(void);

/* ========================================================================
 * WATCHDOG TIMER
 * ======================================================================== */

/**
 * Initialize watchdog timer.
 *
 * @param timeout_ms Timeout period in milliseconds
 */
// void port_wdt_init(uint16_t timeout_ms);
// void port_wdt_feed(void);
// void port_wdt_disable(void);

/* ========================================================================
 * PLATFORM INFORMATION
 * ======================================================================== */

/**
 * Get platform name string.
 * @return String describing the platform (e.g., "AVR ATmega328P")
 */
// const char *port_get_platform_name(void);

/**
 * Get CPU frequency in Hz.
 * @return CPU clock frequency
 */
// uint32_t port_get_cpu_freq(void);

/* ========================================================================
 * PLATFORM-SPECIFIC INCLUDES
 * ======================================================================== */

/* Include platform-specific implementation header */
/* Platform includes moved to top */

#ifdef __cplusplus
}
#endif

#endif /* TOYOS_PORT_H */
