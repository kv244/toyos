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

/* Critical section counter to handle nested calls (Implemented in port_arm.c)
 */
extern volatile uint32_t critical_nesting;

/* Critical section and atomic functions are now inlined via cpu_port.h */

/* Timer & Stack */
void port_timer_init(void);
uint32_t port_get_tick(void);
uint8_t *port_init_stack(uint8_t *stack_top, uint16_t stack_size,
                         void (*task_func)(void));
void port_context_switch(void);
void port_start_first_task(uint8_t *task_stack_ptr) __attribute__((noreturn));

uint8_t *port_fast_zero_stack(uint8_t *sp, uint8_t count);

/* Power & WDT */
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
