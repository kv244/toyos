/**
 * ToyOS User Configuration Example
 *
 * Copy this file to your project directory and modify it to customize
 * ToyOS behavior for your specific application.
 *
 * To use this configuration:
 * 1. Copy this file to your Arduino sketch directory
 * 2. Rename to "toyos_user_config.h"
 * 3. Add this line at the top of your sketch BEFORE #include <toyos.h>:
 *    #define TOYOS_USER_CONFIG
 * 4. Uncomment and modify the parameters you want to change
 *
 * Example:
 *   #define TOYOS_USER_CONFIG
 *   #include <toyos.h>
 *
 * Author: [Your Name]
 * Date: January 2026
 */

#ifndef TOYOS_USER_CONFIG_H
#define TOYOS_USER_CONFIG_H

/* ========================================================================
 * EXAMPLE CONFIGURATIONS
 * Uncomment and modify the sections you need
 * ======================================================================== */

/* ------------------------------------------------------------------------
 * MINIMAL CONFIGURATION (Arduino UNO with tight memory)
 * ------------------------------------------------------------------------ */
/*
#define TOYOS_MAX_TASKS 4
#define TOYOS_DEFAULT_STACK_SIZE 80
#define TOYOS_MEMORY_POOL_SIZE 384
#define TOYOS_ENABLE_MESSAGE_QUEUES 0
*/

/* ------------------------------------------------------------------------
 * STANDARD CONFIGURATION (Balanced for most applications)
 * ------------------------------------------------------------------------ */
/*
#define TOYOS_MAX_TASKS 6
#define TOYOS_DEFAULT_STACK_SIZE 96
#define TOYOS_MEMORY_POOL_SIZE 512
#define TOYOS_ENABLE_PRIORITY_INHERITANCE 1
#define TOYOS_ENABLE_MESSAGE_QUEUES 1
*/

/* ------------------------------------------------------------------------
 * PERFORMANCE CONFIGURATION (More tasks, larger stacks)
 * ------------------------------------------------------------------------ */
/*
#define TOYOS_MAX_TASKS 8
#define TOYOS_DEFAULT_STACK_SIZE 128
#define TOYOS_MEMORY_POOL_SIZE 1024
#define TOYOS_TICK_RATE_HZ 1000
*/

/* ------------------------------------------------------------------------
 * DEBUG CONFIGURATION (Extensive checking and diagnostics)
 * ------------------------------------------------------------------------ */
/*
#define TOYOS_DEBUG 1
#define TOYOS_AUTO_STACK_CHECK 1
#define TOYOS_ENABLE_STATS 1
#define TOYOS_MIN_STACK_SIZE 64
*/

/* ------------------------------------------------------------------------
 * LOW POWER CONFIGURATION (Slower tick, less overhead)
 * ------------------------------------------------------------------------ */
/*
#define TOYOS_TICK_RATE_HZ 100
#define TOYOS_TIMER_PRESCALER 256
#define TOYOS_ENABLE_WATCHDOG 1
#define TOYOS_WATCHDOG_TIMEOUT WDTO_4S
*/

/* ------------------------------------------------------------------------
 * CUSTOM CONFIGURATION
 * Uncomment individual parameters you want to override
 * ------------------------------------------------------------------------ */

/* Task Configuration */
// #define TOYOS_MAX_TASKS 6
// #define TOYOS_DEFAULT_STACK_SIZE 96
// #define TOYOS_MIN_STACK_SIZE 48

/* Priority Configuration */
// #define TOYOS_DEFAULT_PRIORITY 5
// #define TOYOS_IDLE_TASK_PRIORITY 0

/* Memory Configuration */
// #define TOYOS_MEMORY_POOL_SIZE 512
// #define TOYOS_MEM_ALIGNMENT 2

/* Timing Configuration */
// #define TOYOS_TICK_RATE_HZ 1000
// #define TOYOS_TIMER_PRESCALER 64

/* Stack Overflow Detection */
// #define TOYOS_STACK_CANARY 0xDEADBEEF
// #define TOYOS_AUTO_STACK_CHECK 0

/* Watchdog Configuration */
// #define TOYOS_ENABLE_WATCHDOG 1
// #define TOYOS_WATCHDOG_TIMEOUT WDTO_4S

/* Debug and Diagnostics */
// #define TOYOS_DEBUG 0
// #define TOYOS_ENABLE_STATS 0

/* Feature Toggles */
// #define TOYOS_ENABLE_PRIORITY_INHERITANCE 1
// #define TOYOS_ENABLE_MESSAGE_QUEUES 1
// #define TOYOS_MAX_QUEUE_CAPACITY 16

#endif /* TOYOS_USER_CONFIG_H */
