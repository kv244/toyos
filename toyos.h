#ifndef TOYOS_H
#define TOYOS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef enum { TASK_READY, TASK_RUNNING, TASK_BLOCKED } TaskState;

/* Task Control Block */
typedef struct {
  uint8_t id;
  TaskState state;
  void (*task_func)(void);
  uint16_t stack_size;
  uint8_t priority;
  uint16_t tick_count;
} TaskControlBlock;

/* Task Queue Node */
typedef struct TaskNode {
  TaskControlBlock task;
  struct TaskNode *next;
} TaskNode;

/* Simple Queue for Ready Tasks */
typedef struct {
  TaskNode *head;
  TaskNode *tail;
  uint8_t count;
} TaskQueue;

/* Memory Manager */
typedef struct {
  uint8_t *memory_pool;
  uint16_t total_size;
  uint16_t allocated;
} MemoryManager;

/* Kernel Structure */
typedef struct {
  TaskQueue ready_queue;
  TaskQueue blocked_queue; /* Separate queue for blocked tasks */
  TaskControlBlock *current_task;
  MemoryManager mem_manager;
  volatile uint16_t system_tick;
  uint8_t task_count;
} Kernel;

/* Public API */
void os_init(uint8_t *mem_pool, uint16_t mem_size);
void os_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                    uint16_t stack_size);
void os_start(void);
void os_scheduler(void);
void os_delay(uint16_t ticks);
void *os_malloc(uint16_t size);
void os_free(void *ptr);
void os_task_yield(void);
void os_system_tick(void);  /* Called from Timer ISR */
void os_timer_init(void);   /* Initialize Timer1 for system tick */
uint16_t os_get_tick(void); /* Get current system tick count */

#ifdef __cplusplus
}
#endif

#endif
