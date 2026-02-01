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

/* Unified CPU Abstraction Layer */
#include "port/cpu_port.h"

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
 * UNIFIED PORTING INTERFACE (Delegated to cpu_port.h or Port Impl)
 * ======================================================================== */

/* Inlined for performance where possible */
static inline void port_enter_critical(void) { cpu_enter_critical(); }
static inline void port_exit_critical(void) { cpu_exit_critical(); }
static inline void port_enable_interrupts(void) { cpu_enable_interrupts(); }
static inline void port_disable_interrupts(void) { cpu_disable_interrupts(); }

/* Target-specific implementations */
void port_context_switch(void);
static inline void port_enter_idle(void) { cpu_idle(); }
static inline void *port_get_stack_pointer(void) { return cpu_get_sp(); }

/* Atomics (Inlined) */
static inline uint8_t port_atomic_inc_u8(volatile uint8_t *ptr) {
  return cpu_atomic_inc_u8(ptr);
}
static inline uint8_t port_atomic_dec_u8(volatile uint8_t *ptr) {
  return cpu_atomic_dec_u8(ptr);
}
static inline bool port_atomic_cas_u8(volatile uint8_t *ptr, uint8_t exp,
                                      uint8_t des) {
  return cpu_atomic_cas_u8(ptr, exp, des);
}

/* ========================================================================
 * WATCHDOG TIMER
 * ======================================================================== */

/**
 * Initialize watchdog timer.
 *
 * @param timeout_ms Timeout period in milliseconds
 */
void port_wdt_init(uint16_t timeout_ms);
void port_wdt_feed(void);
void port_wdt_disable(void);

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
