/**
 * ToyOS CPU Porting Layer
 *
 * This file abstracts low-level CPU primitives for different architectures.
 */

#ifndef TOYOS_CPU_PORT_H
#define TOYOS_CPU_PORT_H

/* Ensure base types are available */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * ARM CORTEX-M SECTION
 * ======================================================================== */
#if defined(__arm__) || defined(__ARM_ARCH) || defined(ARDUINO_ARCH_RENESAS)

/* External state variables (defined in port_arm.c) */
extern volatile uint32_t critical_nesting;

#include <Arduino.h>

#define cpu_enable_interrupts() __enable_irq()
#define cpu_disable_interrupts() __disable_irq()

static inline void cpu_enter_critical(void) {
  __disable_irq();
  critical_nesting++;
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb" ::: "memory");
}

static inline void cpu_exit_critical(void) {
  critical_nesting--;
  if (critical_nesting == 0) {
    __enable_irq();
  }
}

#define cpu_yield() port_context_switch()

#define cpu_idle() __asm volatile("wfi")
#define cpu_nop() __asm volatile("nop")

static inline uint8_t cpu_atomic_inc_u8(volatile uint8_t *ptr) {
  return __atomic_add_fetch(ptr, 1, __ATOMIC_SEQ_CST);
}
static inline uint8_t cpu_atomic_dec_u8(volatile uint8_t *ptr) {
  return __atomic_sub_fetch(ptr, 1, __ATOMIC_SEQ_CST);
}
static inline bool cpu_atomic_cas_u8(volatile uint8_t *ptr, uint8_t exp,
                                     uint8_t des) {
  return __atomic_compare_exchange_n(ptr, &exp, des, false, __ATOMIC_SEQ_CST,
                                     __ATOMIC_SEQ_CST);
}

#else
#error "Unsupported Architecture - ARM Cortex-M required"
#endif

#ifdef __cplusplus
}
#endif

#endif /* TOYOS_CPU_PORT_H */
