#ifndef TOYOS_H
#define TOYOS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration */
#define MAX_TASKS 8

/* Task States */
typedef enum { TASK_READY, TASK_RUNNING, TASK_BLOCKED } TaskState;

/* Task Control Block */
typedef struct {
  uint8_t id;
  TaskState state;
  void (*task_func)(void);
  uint16_t stack_size;
  uint8_t priority;     /* Higher = more important */
  uint16_t delta_ticks; /* Delta time for blocked queue */
  uint8_t *stack_ptr;   /* Current stack pointer for context switch */
} TaskControlBlock;

/* External assembly functions */
extern "C" void os_context_switch(void);

/* Task Queue Node */
typedef struct TaskNode {
  TaskControlBlock task;
  struct TaskNode *next;
} TaskNode;

/* Simple Queue */
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

/* Binary Heap for Priority Queue */
typedef struct {
  TaskNode *nodes[MAX_TASKS];
  uint8_t size;
} BinaryHeap;

/* Semaphore */
typedef struct {
  volatile uint8_t count;
  TaskQueue blocked_tasks;
} Semaphore;

/* Mutex */
typedef struct {
  volatile uint8_t locked;
  TaskControlBlock *owner;
  TaskQueue blocked_tasks;
} Mutex;

/* Message Queue */
typedef struct {
  void **buffer;
  uint8_t capacity;
  uint8_t head;
  uint8_t tail;
  uint8_t count;
  Semaphore sem_read;
  Semaphore sem_write;
  Mutex mutex;
} MessageQueue;

/* Kernel Structure */
typedef struct {
  BinaryHeap ready_heap;
  TaskQueue blocked_queue; /* Delta queue for blocked tasks */
  TaskControlBlock *current_task;
  TaskNode *current_node; /* Direct pointer to current task's node */
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

/* Semaphore API */
void os_sem_init(Semaphore *sem, uint8_t initial_count);
void os_sem_wait(Semaphore *sem);
void os_sem_post(Semaphore *sem);

/* Mutex API */
void os_mutex_init(Mutex *mutex);
void os_mutex_lock(Mutex *mutex);
void os_mutex_unlock(Mutex *mutex);

/* Message Queue API */
MessageQueue *os_mq_create(uint8_t capacity);
void os_mq_send(MessageQueue *mq, void *msg);
void *os_mq_receive(MessageQueue *mq);

/* Stack Detection & Idle */
void os_check_stack_overflow(void);
void os_enter_idle(void);

#ifdef __cplusplus
}
#endif

#endif
