/**
 * ToyOS AVR Port - Platform-Specific Definitions
 *
 * ATmega328P (Arduino UNO) implementation of the portability layer.
 *
 * Author: [Your Name]
 * Version: 2.5
 * Date: January 2026
 */

#ifndef PORT_AVR_H
#define PORT_AVR_H

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * AVR-SPECIFIC CONSTANTS
 * ======================================================================== */

/**
 * Stack frame size for AVR context.
 * 32 registers (R0-R31) + SREG + 2-byte PC = 35 bytes
 */
#define PORT_AVR_CONTEXT_SIZE 35

/**
 * AVR SREG interrupt enable bit (bit 7)
 */
#define PORT_AVR_SREG_I_BIT 0x80

/**
 * Offset of stack_ptr in TaskControlBlock.
 * CRITICAL: Must match toyos.h TCB definition!
 */
#define PORT_AVR_TCB_STACK_PTR_OFFSET 0

/**
 * AVR Timer1 configuration constants
 */
#define TIMER1_PRESCALER_64 ((1 << CS11) | (1 << CS10))
#define TIMER1_1MS_AT_16MHZ 249 /* (16MHz / 64 / 1000) - 1 */

/**
 * AVR ATmega328P Memory Characteristics
 */
#define PORT_AVR_FLASH_SIZE 32768 /* 32KB Flash */
#define PORT_AVR_SRAM_SIZE 2048   /* 2KB SRAM */
#define PORT_AVR_EEPROM_SIZE 1024 /* 1KB EEPROM */

/* ========================================================================
 * CRITICAL SECTION (INLINE FOR PERFORMANCE)
 * ======================================================================== */

/**
 * Critical section state storage.
 * Stores SREG to allow nested critical sections.
 */
extern volatile uint8_t port_critical_nesting;
extern volatile uint8_t port_saved_sreg;

/**
 * Enter critical section - disable interrupts and save state.
 */
static inline void port_enter_critical(void) {
  uint8_t sreg;
  __asm__ __volatile__("in %0, %1 \n\t"
                       "cli \n\t"
                       : "=r"(sreg)
                       : "I"(_SFR_IO_ADDR(SREG))
                       : "memory");

  if (port_critical_nesting == 0) {
    port_saved_sreg = sreg;
  }
  port_critical_nesting++;
}

/**
 * Exit critical section - restore interrupt state.
 */
static inline void port_exit_critical(void) {
  port_critical_nesting--;

  if (port_critical_nesting == 0) {
    __asm__ __volatile__("out %0, %1 \n\t"
                         :
                         : "I"(_SFR_IO_ADDR(SREG)), "r"(port_saved_sreg)
                         : "memory");
  }
}

/**
 * Enable interrupts globally.
 */
static inline void port_enable_interrupts(void) { sei(); }

/**
 * Disable interrupts globally.
 */
static inline void port_disable_interrupts(void) { cli(); }

/* ========================================================================
 * ATOMIC OPERATIONS
 * ======================================================================== */

static inline uint8_t port_atomic_inc_u8(volatile uint8_t *ptr) {
  uint8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    (*ptr)++;
    result = *ptr;
  }
  return result;
}

static inline uint8_t port_atomic_dec_u8(volatile uint8_t *ptr) {
  uint8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    (*ptr)--;
    result = *ptr;
  }
  return result;
}

static inline bool port_atomic_cas_u8(volatile uint8_t *ptr, uint8_t expected,
                                      uint8_t desired) {
  bool success = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (*ptr == expected) {
      *ptr = desired;
      success = true;
    }
  }
  return success;
}

/* ========================================================================
 * STACK UTILITIES
 * ======================================================================== */

/**
 * Get current stack pointer.
 */
static inline void *port_get_stack_pointer(void) {
  uint16_t sp = SPL | (SPH << 8);
  return (void *)sp;
}

/* ========================================================================
 * WATCHDOG TIMER MAPPINGS
 * ======================================================================== */

/* Map generic timeout values to AVR WDT constants */
#define PORT_WDT_15MS WDTO_15MS
#define PORT_WDT_30MS WDTO_30MS
#define PORT_WDT_60MS WDTO_60MS
#define PORT_WDT_120MS WDTO_120MS
#define PORT_WDT_250MS WDTO_250MS
#define PORT_WDT_500MS WDTO_500MS
#define PORT_WDT_1S WDTO_1S
#define PORT_WDT_2S WDTO_2S
#define PORT_WDT_4S WDTO_4S
#define PORT_WDT_8S WDTO_8S

/* ========================================================================
 * FUNCTION DECLARATIONS (Implemented in port_avr.c)
 * ======================================================================== */

/**
 * Initialize task stack for AVR.
 * See port.h for details.
 */
uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
                         void (*task_func)(void));

/**
 * Context switching for AVR.
 * Implemented in port_avr_asm.S (assembly file).
 */
void port_context_switch(void);

/**
 * Start first task (AVR-specific).
 */
void port_start_first_task(uint8_t *task_stack_ptr) __attribute__((noreturn));

/**
 * Initialize Timer1 for system tick.
 */
void port_timer_init(void);

/**
 * Get system tick count (implemented in C with atomic protection).
 */
uint32_t port_get_tick(void);

/**
 * Fast stack zeroing (AVR assembly optimization).
 */
uint8_t *port_fast_zero_stack(uint8_t *sp, uint8_t count);

/**
 * Enter idle mode (AVR sleep).
 */
void port_enter_idle(void);

/**
 * Watchdog timer operations.
 */
void port_wdt_init(uint16_t timeout_ms);
void port_wdt_feed(void);
void port_wdt_disable(void);

/**
 * Platform information.
 */
const char *port_get_platform_name(void);
uint32_t port_get_cpu_freq(void);

/**
 * Platform memory characteristics.
 */
uint32_t port_get_flash_size(void);
uint32_t port_get_sram_size(void);
uint32_t port_get_eeprom_size(void);
const char *port_get_mcu_name(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_AVR_H */
