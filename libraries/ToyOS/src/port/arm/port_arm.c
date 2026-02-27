/**
 * ToyOS ARM Cortex-M4 Port (Arduino Uno R4 / Renesas RA4M1)
 *
 * IMPORTANT: The Renesas FSP (FreeRTOS port) defines non-weak SVC_Handler
 * AND PendSV_Handler. We CANNOT override them. Therefore this port uses:
 *   - Cooperative context switching via direct register save/restore
 *   - Arduino millis() for time tracking (no SysTick override)
 *   - No PendSV, no SVC
 */

#if defined(__arm__) || defined(__ARM_ARCH) || defined(ARDUINO_ARCH_RENESAS)

#include <stdbool.h>
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "../../port.h"
#include "../../toyos.h"
#include "../../toyos_config.h"

/* -----------------------------------------------------------------------
 * Globals
 * --------------------------------------------------------------------- */
extern volatile TaskControlBlock *os_current_task_ptr;
extern void os_system_tick(void);
extern void os_scheduler(void);

volatile uint32_t critical_nesting = 0;

/* FPU lazy stacking control */
#define FPU_FPCCR (*((volatile uint32_t *)0xE000EF34u))
#define LSPEN_BIT (1UL << 30)
#define ASPEN_BIT (1UL << 31)

/* Track last tick time for polling-based tick */
static volatile uint32_t last_tick_ms = 0;

/* -----------------------------------------------------------------------
 * Stack frame for cooperative context switch
 *
 * When a task yields, we push R4-R11 and LR onto its PSP stack.
 * That's 9 words = 36 bytes. No HW frame needed.
 * --------------------------------------------------------------------- */

#define COOP_FRAME_WORDS 9 /* R4-R11 + LR */

/* -----------------------------------------------------------------------
 * port_init_stack
 *
 * Set up initial stack so that when port_context_switch restores it,
 * the task function starts executing.
 *
 * Layout (grows down):
 *   [stack_top]
 *     LR = task_func   <- when restored, "return" jumps here
 *     R11 = 0
 *     R10 = 0
 *     R9  = 0
 *     R8  = 0
 *     R7  = 0
 *     R6  = 0
 *     R5  = 0
 *     R4  = 0
 *   [stack_ptr in TCB]
 * --------------------------------------------------------------------- */
uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
                         void (*task_func)(void)) {
  (void)stack_size;

  /* Align to 8 bytes */
  uintptr_t addr = ((uintptr_t)stack_top) & ~0x7u;
  uint32_t *stk = (uint32_t *)addr;

  /* Push LR (return address = task entry point) */
  *(--stk) = (uint32_t)task_func;

  /* Push R11 down to R4 (8 registers, all zero) */
  for (int i = 0; i < 8; i++) {
    *(--stk) = 0;
  }

  /* Push dummy word (mapped to R0 in pop {r0, r4-r11, lr}) */
  *(--stk) = 0;

  return (uint8_t *)stk;
}

/* -----------------------------------------------------------------------
 * port_context_switch  (naked)
 *
 * Cooperative context switch. Called from k_yield().
 * Saves R4-R11 + LR onto current PSP, stores PSP into TCB.
 * Calls os_scheduler to pick next task.
 * Loads next task's PSP, restores R4-R11 + LR, returns.
 *
 * When returning, we "return" into the next task's yield point
 * (or into the task_func for a freshly created task).
 * --------------------------------------------------------------------- */
__attribute__((naked)) void port_context_switch(void) {
  __asm volatile(
      /* Save current context onto PSP */
      "push  {r0, r4-r11, lr} \n" /* Save callee-saved regs + LR + dummy (R0)
                                     for alignment */

      /* Store current SP into TCB->stack_ptr */
      "ldr   r2, =os_current_task_ptr \n"
      "ldr   r1, [r2]         \n" /* r1 = &TCB */
      "mov   r0, sp           \n"
      "str   r0, [r1]         \n" /* TCB->stack_ptr = SP */

      /* Call scheduler to select next task */
      "bl    os_scheduler     \n"

      /* Restore new task's context */
      "ldr   r2, =os_current_task_ptr \n"
      "ldr   r1, [r2]         \n" /* r1 = &new TCB */
      "ldr   r0, [r1]         \n" /* r0 = new TCB->stack_ptr */
      "mov   sp, r0           \n" /* Restore SP */

      "pop   {r0, r4-r11, lr} \n" /* Restore callee-saved regs + LR + dummy (R0)
                                   */
      "bx    lr               \n" /* "Return" into next task */
      ::
          : "memory");
}

/* -----------------------------------------------------------------------
 * port_timer_init
 *
 * We do NOT override SysTick (the FSP owns it for FreeRTOS).
 * Instead we use a polling approach: check millis() in os_update_ticks().
 * --------------------------------------------------------------------- */
void port_timer_init(void) {
  /* Disable lazy FP stacking */
  FPU_FPCCR &= ~(ASPEN_BIT | LSPEN_BIT);

  /* Record starting time */
  last_tick_ms = millis();
}

/* -----------------------------------------------------------------------
 * os_update_ticks (called before scheduling decisions)
 *
 * Polls millis() and calls os_system_tick() for each ms elapsed.
 * --------------------------------------------------------------------- */
void os_update_ticks(void) {
  uint32_t now = millis();
  if (last_tick_ms >= now)
    return;

  uint32_t saved_primask;
  __asm volatile("mrs %0, primask; cpsid i" : "=r"(saved_primask)::"memory");

  while (last_tick_ms < now) {
    last_tick_ms++;
    os_system_tick();
  }

  __asm volatile("msr primask, %0" ::"r"(saved_primask) : "memory");
}

/* -----------------------------------------------------------------------
 * port_start_first_task
 *
 * Switch to PSP, start timer, load first task's context and "return" to it.
 * --------------------------------------------------------------------- */
void port_start_first_task(uint8_t *task_stack_ptr) {
  FPU_FPCCR &= ~(ASPEN_BIT | LSPEN_BIT);

  /* Set PSP and switch to it */
  __asm volatile("msr psp, %0" ::"r"(task_stack_ptr) : "memory");
  __asm volatile("mov r0, #2; msr control, r0; isb" ::: "r0", "memory");

  port_timer_init();

  /* Load the task's initial stack into SP and pop context */
  __asm volatile(
      "mov   sp, %0           \n"
      "pop   {r0, r4-r11, lr} \n" /* r0=dummy, r4-r11=context, LR=entry */
      "cpsie i                \n"
      "cpsie f                \n"
      "bx    lr               \n" /* Jump to task_func */
      ::"r"(task_stack_ptr)
      : "memory");

  while (1) {
  }
}

/* -----------------------------------------------------------------------
 * Platform info
 * --------------------------------------------------------------------- */
const char *port_get_platform_name(void) {
  return "ARM Cortex-M4 (Renesas RA4M1)";
}
const char *port_get_mcu_name(void) { return "RA4M1"; }

uint32_t port_get_cpu_freq(void) {
#ifdef F_CPU
  return (uint32_t)F_CPU;
#else
  return 48000000UL;
#endif
}

uint32_t port_get_flash_size(void) { return 256u * 1024u; }
uint32_t port_get_sram_size(void) { return 32u * 1024u; }
uint32_t port_get_eeprom_size(void) { return 8u * 1024u; }

uint32_t port_get_tick(void) { return (uint32_t)millis(); }

void port_wdt_init(uint16_t timeout_ms) { (void)timeout_ms; }
void port_wdt_feed(void) {}
void port_wdt_disable(void) {}

uint8_t *port_fast_zero_stack(uint8_t *sp, uint8_t count) {
  while (count--) {
    *(--sp) = 0;
  }
  return sp;
}

#endif /* ARM */
