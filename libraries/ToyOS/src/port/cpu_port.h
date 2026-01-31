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
 * AVR (ATmega328P) SECTION
 * ======================================================================== */
#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#define cpu_enable_interrupts() sei()
#define cpu_disable_interrupts() cli()

static inline void cpu_enter_critical(void) {
  uint8_t sreg = SREG;
  cli();
  if (port_critical_nesting == 0) {
    port_saved_sreg = sreg;
  }
  port_critical_nesting++;
}

static inline void cpu_exit_critical(void) {
  port_critical_nesting--;
  if (port_critical_nesting == 0) {
    SREG = port_saved_sreg;
  }
}

static inline void *cpu_get_sp(void) {
  uint16_t sp = SPL | (SPH << 8);
  return (void *)sp;
}

extern void port_context_switch(void);
#define cpu_yield() port_context_switch()

static inline void cpu_idle(void) {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
}

#define cpu_nop() __asm__ __volatile__("nop")

static inline uint8_t cpu_atomic_inc_u8(volatile uint8_t *ptr) {
  uint8_t res;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { res = ++(*ptr); }
  return res;
}
static inline uint8_t cpu_atomic_dec_u8(volatile uint8_t *ptr) {
  uint8_t res;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { res = --(*ptr); }
  return res;
}
static inline bool cpu_atomic_cas_u8(volatile uint8_t *ptr, uint8_t exp,
                                     uint8_t des) {
  bool success = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (*ptr == exp) {
      *ptr = des;
      success = true;
    }
  }
  return success;
}

/* ========================================================================
 * ARM CORTEX-M SECTION
 * ======================================================================== */
#elif defined(__arm__) || defined(__ARM_ARCH) || defined(ARDUINO_ARCH_RENESAS)
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

static inline void *cpu_get_sp(void) {
  uint32_t sp;
  __asm__ __volatile__("mrs %0, psp" : "=r"(sp));
  return (void *)sp;
}

#define cpu_yield()                                                            \
  do {                                                                         \
    (*(volatile uint32_t *)0xE000ED04) = (1UL << 28);                          \
    __asm volatile("dsb" ::: "memory");                                        \
    __asm volatile("isb" ::: "memory");                                        \
  } while (0)

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
#error "Unsupported Architecture"
#endif

#ifdef __cplusplus
}
#endif

#endif /* TOYOS_CPU_PORT_H */
