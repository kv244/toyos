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
 * CRITICAL SECTION & ATOMICS - NOW CENTRALIZED IN CPU_PORT.H
 * ======================================================================== */

extern volatile uint8_t port_critical_nesting;
extern volatile uint8_t port_saved_sreg;

/* ========================================================================
 * STACK UTILITIES
 * ======================================================================== */

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
 * Watchdog timer operations.
 */
void port_wdt_init(uint16_t timeout_ms);
void port_wdt_feed(void);
void port_wdt_disable(void);

/**
 * Platform information.
 */
extern const char port_platform_name[];
extern const char port_mcu_atmega328p[];
extern const char port_mcu_atmega2560[];
extern const char port_mcu_atmega32u4[];
extern const char port_mcu_unknown[];

static inline const char *port_get_platform_name(void) {
  return port_platform_name;
}

static inline const char *port_get_mcu_name(void) {
#if defined(__AVR_ATmega328P__)
  return port_mcu_atmega328p;
#elif defined(__AVR_ATmega2560__)
  return port_mcu_atmega2560;
#elif defined(__AVR_ATmega32U4__)
  return port_mcu_atmega32u4;
#else
  return port_mcu_unknown;
#endif
}

static inline uint32_t port_get_cpu_freq(void) {
#ifdef F_CPU
  return F_CPU;
#else
  return 16000000UL;
#endif
}

/**
 * Platform memory characteristics.
 */
static inline uint32_t port_get_flash_size(void) { return PORT_AVR_FLASH_SIZE; }
static inline uint32_t port_get_sram_size(void) { return PORT_AVR_SRAM_SIZE; }
static inline uint32_t port_get_eeprom_size(void) {
  return PORT_AVR_EEPROM_SIZE;
}

#ifdef __cplusplus
}
#endif

#endif /* PORT_AVR_H */
