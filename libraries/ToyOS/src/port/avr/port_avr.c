/**
 * ToyOS AVR Port - C Implementation
 *
 * ATmega328P (Arduino UNO) implementation of portability layer.
 *
 * Author: [Your Name]
 * Version: 2.5
 * Date: January 2026
 */

/* Guard entire file based on architecture */
#if defined(__AVR__)

#include "port_avr.h"
#include "../../port.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Global states for nesting and interrupts */
volatile uint8_t port_critical_nesting = 0;
volatile uint8_t port_saved_sreg = 0;

/* System tick counter */
static volatile uint32_t port_system_tick = 0;

/* External reference to scheduler and current task */
extern void os_scheduler(void);
extern void os_system_tick(void);
extern void *os_current_task_ptr;

/* ========================================================================
 * STACK INITIALIZATION
 * ======================================================================== */

/**
 * Initialize AVR task stack.
 *
 * Stack layout (from high to low address):
 * - PC Low byte (return address for 'ret')
 * - PC High byte
 * - R0 (temporary storage for SREG during restore)
 * - SREG (status register with I bit set)
 * - R1 (zero register)
 * - R2-R31 (general purpose registers)
 */
uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
                         void (*task_func)(void)) {
  uint8_t *sp = stack_top - 1;

  /* Store return address (task function) */
  /* Store return address (task function) */
  uint16_t func_addr = (uint16_t)(uintptr_t)task_func;
  *sp-- = (uint8_t)(func_addr & 0xFF);        /* PC Low byte */
  *sp-- = (uint8_t)((func_addr >> 8) & 0xFF); /* PC High byte */

  /* Initial register values */
  /* Order: R0 pushed first, SREG, then R1 pushed last (Top of stack) */
  *sp-- = 0x00;                /* R0 */
  *sp-- = PORT_AVR_SREG_I_BIT; /* SREG (interrupts enabled) */
  *sp-- = 0x00;                /* R1 (zero register) */

  /* R2-R31: Initialize to zero */
  for (int i = 0; i < 30; i++) {
    *sp-- = 0x00;
  }

  return sp;
}

/* ========================================================================
 * TIMER / SYSTEM TICK
 * ======================================================================== */

/**
 * Initialize Timer1 for 1ms system tick at 16MHz.
 */
void port_timer_init(void) {
  cli(); /* Disable interrupts during configuration */

  /* Configure Timer1 in CTC mode with prescaler 64 */
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | TIMER1_PRESCALER_64;

  /* Set compare value for 1ms tick
   * OCR1A = (F_CPU / prescaler / tick_rate) - 1
   * = (16000000 / 64 / 1000) - 1 = 249
   */
  OCR1A = TIMER1_1MS_AT_16MHZ;

  /* Enable Timer1 compare match A interrupt */
  TIMSK1 = (1 << OCIE1A);

  /* Reset tick counter */
  port_system_tick = 0;
}

/**
 * Get current system tick count with atomic access.
 */
uint32_t port_get_tick(void) {
  uint32_t tick;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tick = port_system_tick; }
  return tick;
}

/**
 * Timer1 Compare Match A ISR - System Tick Handler
 *
 * This is a NAKED ISR that manually handles context saving/restoring.
 * It calls os_system_tick() and triggers scheduling via port_context_switch().
 */
ISR(TIMER1_COMPA_vect, ISR_NAKED) {
  /* Save context (inline assembly) */
  __asm__ __volatile__("push r0 \n\t"
                       "in r0, %0 \n\t"
                       "push r0 \n\t"
                       "push r1 \n\t"
                       "clr r1 \n\t"
                       "push r2 \n\t"
                       "push r3 \n\t"
                       "push r4 \n\t"
                       "push r5 \n\t"
                       "push r6 \n\t"
                       "push r7 \n\t"
                       "push r8 \n\t"
                       "push r9 \n\t"
                       "push r10 \n\t"
                       "push r11 \n\t"
                       "push r12 \n\t"
                       "push r13 \n\t"
                       "push r14 \n\t"
                       "push r15 \n\t"
                       "push r16 \n\t"
                       "push r17 \n\t"
                       "push r18 \n\t"
                       "push r19 \n\t"
                       "push r20 \n\t"
                       "push r21 \n\t"
                       "push r22 \n\t"
                       "push r23 \n\t"
                       "push r24 \n\t"
                       "push r25 \n\t"
                       "push r26 \n\t"
                       "push r27 \n\t"
                       "push r28 \n\t"
                       "push r29 \n\t"
                       "push r30 \n\t"
                       "push r31 \n\t" ::"I"(_SFR_IO_ADDR(SREG)));

  /* Save current SP to TCB */
  if (os_current_task_ptr) {
    uint16_t sp_val = SPL | (SPH << 8);
    *(uint8_t **)os_current_task_ptr = (uint8_t *)sp_val;
  }

  /* Increment system tick */
  port_system_tick++;

  /* Call OS tick handler */
  os_system_tick();

  /* Call scheduler to pick next task */
  os_scheduler();

  /* Load new SP from TCB */
  if (os_current_task_ptr) {
    uint16_t sp_val = (uint16_t)(*(uint8_t **)os_current_task_ptr);
    SPH = sp_val >> 8;
    SPL = sp_val & 0xFF;
  }

  /* Restore context (inline assembly) */
  __asm__ __volatile__("pop r31 \n\t"
                       "pop r30 \n\t"
                       "pop r29 \n\t"
                       "pop r28 \n\t"
                       "pop r27 \n\t"
                       "pop r26 \n\t"
                       "pop r25 \n\t"
                       "pop r24 \n\t"
                       "pop r23 \n\t"
                       "pop r22 \n\t"
                       "pop r21 \n\t"
                       "pop r20 \n\t"
                       "pop r19 \n\t"
                       "pop r18 \n\t"
                       "pop r17 \n\t"
                       "pop r16 \n\t"
                       "pop r15 \n\t"
                       "pop r14 \n\t"
                       "pop r13 \n\t"
                       "pop r12 \n\t"
                       "pop r11 \n\t"
                       "pop r10 \n\t"
                       "pop r9 \n\t"
                       "pop r8 \n\t"
                       "pop r7 \n\t"
                       "pop r6 \n\t"
                       "pop r5 \n\t"
                       "pop r4 \n\t"
                       "pop r3 \n\t"
                       "pop r2 \n\t"
                       "pop r1 \n\t"
                       "pop r0 \n\t"
                       "out %0, r0 \n\t"
                       "pop r0 \n\t"
                       "reti \n\t" ::"I"(_SFR_IO_ADDR(SREG)));
}

/* ========================================================================
 * LOW POWER / IDLE
 * ======================================================================== */

/* ========================================================================
 * WATCHDOG TIMER
 * ======================================================================== */

/**
 * Initialize watchdog timer with timeout.
 *
 * Note: timeout_ms is mapped to nearest AVR WDT constant.
 */
void port_wdt_init(uint16_t timeout_ms) {
  uint8_t wdt_timeout;

  /* Map milliseconds to AVR WDT constants */
  if (timeout_ms <= 15)
    wdt_timeout = WDTO_15MS;
  else if (timeout_ms <= 30)
    wdt_timeout = WDTO_30MS;
  else if (timeout_ms <= 60)
    wdt_timeout = WDTO_60MS;
  else if (timeout_ms <= 120)
    wdt_timeout = WDTO_120MS;
  else if (timeout_ms <= 250)
    wdt_timeout = WDTO_250MS;
  else if (timeout_ms <= 500)
    wdt_timeout = WDTO_500MS;
  else if (timeout_ms <= 1000)
    wdt_timeout = WDTO_1S;
  else if (timeout_ms <= 2000)
    wdt_timeout = WDTO_2S;
  else if (timeout_ms <= 4000)
    wdt_timeout = WDTO_4S;
  else
    wdt_timeout = WDTO_8S;

  wdt_enable(wdt_timeout);
}

/**
 * Feed/reset the watchdog timer.
 */
void port_wdt_feed(void) { wdt_reset(); }

/**
 * Disable watchdog timer.
 */
void port_wdt_disable(void) { wdt_disable(); }

/**
 * Early watchdog disable (AVR specific).
 *
 * This function is placed in the .init3 section, which runs immediately
 * after reset, before the main() function and before C global constructors.
 * This is the most robust way to prevent watchdog-induced boot loops on AVR.
 */
void port_early_wdt_disable(void) __attribute__((naked))
__attribute__((section(".init3")));
void port_early_wdt_disable(void) {
  MCUSR = 0;
  wdt_disable();
}

/* ========================================================================
 * PLATFORM INFORMATION
 * ======================================================================== */

#include <avr/pgmspace.h>

const char port_platform_name[] PROGMEM = "AVR";
const char port_mcu_atmega328p[] PROGMEM = "ATmega328P";
const char port_mcu_atmega2560[] PROGMEM = "ATmega2560";
const char port_mcu_atmega32u4[] PROGMEM = "ATmega32U4";
const char port_mcu_unknown[] PROGMEM = "Unknown AVR";

#endif /* __AVR__ */
