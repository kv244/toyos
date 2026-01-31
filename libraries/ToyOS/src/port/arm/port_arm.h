/**
 * ToyOS Portability Layer - ARM Cortex-M Implementation
 * Supports M0/M0+/M3/M4/M7
 */

#ifndef KEY_OS_PORT_ARM_H
#define KEY_OS_PORT_ARM_H

#ifdef ARDUINO
#include <Arduino.h>
#else
/* For non-Arduino CMSIS environment, include appropriate headers */
/* e.g., #include "stm32f4xx.h" */
#endif

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cortex-M Interrupt Management */

/**
 * Enable global interrupts
 * Uses CMSIS intrinsic
 */
static inline void port_enable_interrupts(void) { __enable_irq(); }

/**
 * Disable global interrupts
 * Uses CMSIS intrinsic
 */
static inline void port_disable_interrupts(void) { __disable_irq(); }

/* Critical section counter to handle nested calls */
extern volatile uint32_t critical_nesting;

/* These functions are implemented in port_arm.c */
void port_enter_critical(void);
void port_exit_critical(void);

/* Timer & Stack */
void port_timer_init(void);
uint32_t port_get_tick(void);
uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
                         void (*task_func)(void));
void port_context_switch(void);
void port_start_first_task(uint8_t *task_stack_ptr) __attribute__((noreturn));
void *port_get_stack_pointer(void);
uint8_t *port_fast_zero_stack(uint8_t *sp, uint8_t count);

/* Atomic */
uint8_t port_atomic_inc_u8(volatile uint8_t *ptr);
uint8_t port_atomic_dec_u8(volatile uint8_t *ptr);
bool port_atomic_cas_u8(volatile uint8_t *ptr, uint8_t expected,
                        uint8_t desired);

/* Power & WDT */
void port_enter_idle(void);
void port_wdt_init(uint16_t timeout_ms);
void port_wdt_feed(void);
void port_wdt_disable(void);

/* Info */
const char *port_get_platform_name(void);
uint32_t port_get_cpu_freq(void);
uint32_t port_get_flash_size(void);
uint32_t port_get_sram_size(void);
uint32_t port_get_eeprom_size(void);
const char *port_get_mcu_name(void);

/* Context Switching Helper */
void PendSV_Handler(void);
void SysTick_Handler(void);
void SVC_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* KEY_OS_PORT_ARM_H */
