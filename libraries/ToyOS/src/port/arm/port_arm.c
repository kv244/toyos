/**
 * ToyOS Portability Layer - ARM Cortex-M Implementation
 */

/* Guard entire file based on architecture to prevent compilation on AVR */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) ||                   \
    defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_8M_MAIN__) ||               \
    defined(__ARM_ARCH_8M_BASE__)

#include "port_arm.h"
#include "../../port.h"
#include "../../toyos.h"        /* For TaskControlBlock */
#include "../../toyos_config.h" /* For TOYOS_USE_MPU */

#if TOYOS_USE_MPU
#include "port_arm_mpu.h"
#endif

#ifdef PORT_PLATFORM_ARM_CORTEX_M

/* Global pointer to current task (defined inos_kernel_fixed.cpp) */
extern volatile TaskControlBlock *os_current_task_ptr;

/* Scheduler entry point */
extern void os_scheduler(void);

/* Interrupt nesting counter */
volatile uint32_t critical_nesting = 0;

/* Exception Frame Layout (Pushed by Hardware) */
typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
} hw_stack_frame_t;

/* Software Context Layout (Pushed by PendSV) */
typedef struct {
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
} sw_stack_frame_t;

#define xPSR_T_BIT (1UL << 24)

/**
 * Initialize stack for a new task.
 */
uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
                         void (*task_func)(void)) {
  /* ARM stack grows down. stack_top points to the highest valid address + 1?
     Usually user passes array end.
     Ensure 8-byte alignment.
  */
  uintptr_t addr = (uintptr_t)stack_top;
  addr &= ~0x7; /* Alien to 8 bytes */
  uint8_t *stk = (uint8_t *)addr;

  /* Reserve space for Hardware Stack Frame */
  stk -= sizeof(hw_stack_frame_t);
  hw_stack_frame_t *hw_frame = (hw_stack_frame_t *)stk;

  hw_frame->r0 = 0;
  hw_frame->r1 = 0;
  hw_frame->r2 = 0;
  hw_frame->r3 = 0;
  hw_frame->r12 = 0;
  hw_frame->lr =
      0xFFFFFFFF; /* Return to invalid address (trap) if task returns */
  hw_frame->pc = (uint32_t)task_func; /* Task Entry Point */
  hw_frame->xpsr = xPSR_T_BIT;        /* EPSR Thumb bit must be set! */

  /* Reserve space for Software Stack Frame (R4-R11) */
  stk -= sizeof(sw_stack_frame_t);
  /* No need to initialize R4-R11, but useful for debug */

  return stk;
}

/**
 * Start First Task
 */
/**
 * Start First Task
 * Triggers SVC to switch to handler mode and restore first context.
 */
void port_start_first_task(uint8_t *task_stack_ptr) {
  (void)task_stack_ptr;
  __asm volatile(" cpsie i \n" /* Enable interrupts globally */
                 " svc 0 \n"   /* Supervisor Call to restore context */
                 " nop \n");
  while (1)
    ; /* Should not be reached */
}

/**
 * Context Switch (Trigger PendSV)
 */
void port_context_switch(void) {
  /* Set PendSV to pending */
  /* SCB_ICSR = PENDSVSET_Msk */
  /* 0xE000ED04 |= (1 << 28) */
  (*(volatile uint32_t *)0xE000ED04) = (1UL << 28);

  /* Ensure memory ops completed */
  __asm volatile("dsb");
  __asm volatile("isb");
}

/* MPU Reconfiguration Helper (called from PendSV) */
void port_mpu_reconfigure(TaskControlBlock *task) {
#if TOYOS_USE_MPU
  if (task) {
    extern void port_mpu_configure_task_stack(void *stack_base,
                                              uint32_t stack_size);
    /* Stack base corresponds to the canary location (lowest address) */
    port_mpu_configure_task_stack((void *)task->canary_ptr, task->stack_size);
  }
#endif
}

/**
 * PendSV Handler - The actual context switch
 */
__attribute__((naked)) void PendSV_Handler(void) {
  __asm volatile(
      "mrs r0, psp \n"
      "isb \n"

      /* Save Context */
      "stmdb r0!, {r4-r11} \n" // Save R4-R11

      /* Save SP to TCB */
      "ldr r2, =os_current_task_ptr \n"
      "ldr r1, [r2] \n"
      "str r0, [r1] \n"

      /* Save LR (EXC_RETURN) on stack? No need unless we call C func */
      "push {r14} \n"

      /* Select Next Task */
      "bl os_scheduler \n"

#if TOYOS_USE_MPU
      "ldr r0, =os_current_task_ptr \n"
      "ldr r0, [r0] \n"
      "bl port_mpu_reconfigure \n"
#endif

      "pop {r14} \n"

      /* Restore Context */
      "ldr r2, =os_current_task_ptr \n"
      "ldr r1, [r2] \n"
      "ldr r0, [r1] \n" // psp = stack_ptr

      "ldmia r0!, {r4-r11} \n" // Restore R4-R11
      "msr psp, r0 \n"
      "isb \n"

      "bx r14 \n"
      ".align 4 \n");
}

/**
 * Timer Setup (SysTick)
 */
void port_timer_init(void) {
  /* Arduino init() usually enables SysTick for millis().
     We can hook SysTick or use a different timer.
     Better to hook SysTick if possible?
     Compiling for Arduino, the core defines SysTick_Handler!
     Collision!

     Solution: define os_systick_hook() and call it from the core's handler?
     Or use `yield()`?

     ToyOS expects to OWN the tick handler.
     If running on top of Arduino Core, we might break `millis()`.

     Alternative: `port_get_tick` uses `millis()`. `port_context_switch` is
     manual. Preemption requires an interrupt.

     On R4, we can override SysTick_Handler if it's weak?
     Usually it is.
     If we override it, we must increment Arduino tick manually if we want
     `millis` to work.

     Let's assume we own SysTick for now.
  */

  /* Configure SysTick to interrupt at OS_TICK_RATE_HZ (1kHz) */
  /* SystemCoreClock is standard CMSIS variable */
  /* SysTick_Config(SystemCoreClock / 1000); */

  /* On Arduino R4, F_CPU is defined. */
  /* But we want to avoid conflict. Assuming users accept "ToyOS takes over". */

  SysTick_Config(F_CPU / 1000);
  NVIC_SetPriority(SysTick_IRQn, 15); /* Lowest priority */
  NVIC_SetPriority(PendSV_IRQn, 15);  /* Lowest priority for PendSV too */
}

void SysTick_Handler(void) {
  /* Call OS Tick processing */
  os_system_tick();

  /* Trigger potential context switch via PendSV */
  port_context_switch();
}

/* Optional: Increment Arduino millis counter if we stole the vector?
   On R4, generic clock usually handles millis? No, SysTick.
   If we steal it, `delay()` inside Arduino libs might break.
   Ideally we call invalid global?
*/

/* SVC Handler - Restores first task context */
__attribute__((naked)) void SVC_Handler(void) {
  __asm volatile("ldr r0, =os_current_task_ptr \n"
                 "ldr r1, [r0] \n"
                 "ldr r0, [r1] \n" /* r0 = stack_ptr from TCB */

                 "ldmia r0!, {r4-r11} \n" /* Pop SW context (R4-R11) */
                 "msr psp, r0 \n"         /* Set PSP to current SP */
                 "mov r0, #2 \n" /* Set CONTROL: Use PSP, Unprivileged? (or
                                    Privileged Thread) */
                 /* Bit 1=1 (PSP), Bit 0=0 (Privileged) usually */
                 /* For Arduino R4/M4, we run Privileged Thread usually. */
                 "msr control, r0 \n"
                 "isb \n"

                 "orr r14, #0xd \n" /* Force EXC_RETURN to Thread Mode, PSP */
                 "bx r14 \n" /* Returns -> pops HW frame from PSP -> Task PC */
                 ".align 4 \n");
}

/* Platform Info */
const char *port_get_platform_name(void) { return "ARM Cortex-M4 (Renesas)"; }

const char *port_get_mcu_name(void) { return "RA4M1"; }

uint32_t port_get_cpu_freq(void) { return F_CPU; }

uint32_t port_get_flash_size(void) { return 256 * 1024; /* 256KB */ }

uint32_t port_get_sram_size(void) { return 32 * 1024; /* 32KB */ }

uint32_t port_get_eeprom_size(void) {
  return 8 * 1024; /* 8KB Data Flash acts as EEPROM */
}

/* Atomic and Critical Section functions are now inlined in port.h via
 * cpu_port.h */

/* Ticks */
uint32_t port_get_tick(void) {
  /* os_system_tick returns count? No, os_system_tick updates kernel state.
     We need access to kernel.system_tick?
     Or rely on SysTick->VAL?

     ToyOS generic `os_get_time()` usually returns `kernel.system_tick`.
     `port_get_tick` is usually hard to implement without kernel internals.

     Wait, `port.h` defines `port_get_tick`.
     If we use `kernel.system_tick` (exposed via `os_get_time`?), we don't need
     port impl? Ah, `toyos.h` says `port_get_tick()` returns milliseconds.

     Let's use the Arduino `millis()` as fallback if we didn't hook SysTick
     perfectly? No, let's return `os_get_time()` from kernel? But that's
     circular.

     Let's assume `extern volatile uint32_t system_tick` from kernel?
     Actually, `port_get_tick` is intended to simply return the SysTick counter
     or global variable.

     Let's leave it as returning `ms` derived from kernel.
     Wait, `os_kernel_fixed.cpp` manages `kernel.system_tick`.
     `port.h` interface requires `port_get_tick`.

     I'll implement it as:
     return (uint32_t)os_get_time();

  */
  extern uint32_t os_get_time(void);
  return os_get_time();
}

/* Power & WDT */

void port_wdt_init(uint16_t timeout_ms) {
  (void)timeout_ms;
  /* Stub: Implement R4 WDT init here */
}

void port_wdt_feed(void) { /* Stub: Implement R4 WDT feed here */ }

void port_wdt_disable(void) { /* Stub */ }

uint8_t *port_fast_zero_stack(uint8_t *sp, uint8_t count) {
  /* Fallback C implementation or optimized assembly */
  while (count--) {
    *(--sp) = 0;
  }
  return sp;
}

#endif /* PORT_PLATFORM_ARM_CORTEX_M */

#endif /* __ARM_ARCH... */
