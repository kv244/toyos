#include "toyos.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

/* Static Task Pool - no dynamic allocation overhead */
static TaskNode task_pool[MAX_TASKS];
static uint8_t task_pool_index = 0;

/* Global Kernel Instance */
static Kernel kernel = {0};

/* Inline helper: get task node from static pool */
static inline TaskNode *get_task_node(void) __attribute__((always_inline));
static inline TaskNode *get_task_node(void) {
  if (task_pool_index >= MAX_TASKS)
    return NULL;
  return &task_pool[task_pool_index++];
}

/* Initialize the OS */
void os_init(uint8_t *mem_pool, uint16_t mem_size) {
  kernel.ready_queue.head = NULL;
  kernel.ready_queue.tail = NULL;
  kernel.ready_queue.count = 0;
  kernel.blocked_queue.head = NULL;
  kernel.blocked_queue.tail = NULL;
  kernel.blocked_queue.count = 0;
  kernel.current_task = NULL;
  kernel.current_node = NULL;
  kernel.system_tick = 0;
  kernel.task_count = 0;

  /* Initialize memory manager (still available for user allocations) */
  kernel.mem_manager.memory_pool = mem_pool;
  kernel.mem_manager.total_size = mem_size;
  kernel.mem_manager.allocated = 0;

  /* Reset task pool index */
  task_pool_index = 0;
}

/* Initialize Timer1 for ~1ms system tick at 16MHz */
void os_timer_init(void) {
  cli(); /* Disable interrupts */

  /* Timer1 CTC mode, prescaler 64 */
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); /* CTC, prescaler 64 */

  /* For 1ms tick at 16MHz with prescaler 64: 16000000/64/1000 = 250 */
  OCR1A = 249; /* 1ms interval */

  /* Enable Timer1 compare match A interrupt */
  TIMSK1 = (1 << OCIE1A);

  sei(); /* Enable interrupts */
}

/* Timer1 Compare Match A ISR */
ISR(TIMER1_COMPA_vect) { os_system_tick(); }

/* Get current tick count - atomic read */
uint16_t os_get_tick(void) {
  uint16_t tick;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tick = kernel.system_tick; }
  return tick;
}

/* Inline: Add task node to queue tail (for round-robin fallback) */
static inline void queue_add_tail(TaskQueue *queue, TaskNode *node)
    __attribute__((always_inline));
static inline void queue_add_tail(TaskQueue *queue, TaskNode *node) {
  node->next = NULL;
  if (queue->tail == NULL) {
    queue->head = node;
    queue->tail = node;
  } else {
    queue->tail->next = node;
    queue->tail = node;
  }
  queue->count++;
}

/* Priority-based insertion: higher priority tasks at front */
static void queue_add_priority(TaskQueue *queue, TaskNode *node) {
  node->next = NULL;

  if (queue->head == NULL) {
    /* Empty queue */
    queue->head = node;
    queue->tail = node;
  } else if (node->task.priority > queue->head->task.priority) {
    /* Insert at front (highest priority) */
    node->next = queue->head;
    queue->head = node;
  } else {
    /* Find insertion point */
    TaskNode *curr = queue->head;
    while (curr->next && curr->next->task.priority >= node->task.priority) {
      curr = curr->next;
    }
    node->next = curr->next;
    curr->next = node;
    if (node->next == NULL) {
      queue->tail = node;
    }
  }
  queue->count++;
}

/* Inline: Remove task node from front of queue */
static inline TaskNode *queue_remove(TaskQueue *queue)
    __attribute__((always_inline));
static inline TaskNode *queue_remove(TaskQueue *queue) {
  if (queue->head == NULL)
    return NULL;

  TaskNode *node = queue->head;
  queue->head = node->next;
  if (queue->head == NULL) {
    queue->tail = NULL;
  }
  node->next = NULL;
  queue->count--;
  return node;
}

/* Create a new task - uses static pool */
void os_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                    uint16_t stack_size) {
  if (kernel.task_count >= MAX_TASKS)
    return;

  TaskNode *new_node = get_task_node();
  if (!new_node)
    return;

  new_node->task.id = id;
  new_node->task.state = TASK_READY;
  new_node->task.task_func = task_func;
  new_node->task.priority = priority;
  new_node->task.stack_size = stack_size;
  new_node->task.delta_ticks = 0;
  new_node->next = NULL;

  queue_add_priority(&kernel.ready_queue, new_node);
  kernel.task_count++;
}

/* Priority-Based Scheduler */
void os_scheduler(void) {
  if (kernel.ready_queue.head == NULL)
    return;

  /* Get highest priority ready task (always at head) */
  TaskNode *next_node = queue_remove(&kernel.ready_queue);
  if (next_node && next_node->task.state == TASK_READY) {
    kernel.current_task = &next_node->task;
    kernel.current_node = next_node; /* Store direct pointer for O(1) delay */
    kernel.current_task->state = TASK_RUNNING;

    /* Re-queue the task by priority */
    queue_add_priority(&kernel.ready_queue, next_node);
  }
}

/* Execute current task - inlined for performance */
static inline void os_run_task(void) __attribute__((always_inline));
static inline void os_run_task(void) {
  register TaskControlBlock *task = kernel.current_task;
  if (task && task->task_func) {
    task->task_func();
  }
}

/* Delta Queue insertion for blocked tasks */
static void blocked_queue_add(TaskNode *node, uint16_t ticks) {
  node->next = NULL;
  node->task.delta_ticks = ticks;

  if (kernel.blocked_queue.head == NULL) {
    /* Empty queue */
    kernel.blocked_queue.head = node;
    kernel.blocked_queue.tail = node;
  } else {
    /* Find insertion point using delta encoding */
    TaskNode *prev = NULL;
    TaskNode *curr = kernel.blocked_queue.head;
    uint16_t accumulated = 0;

    while (curr && (accumulated + curr->task.delta_ticks) <= ticks) {
      accumulated += curr->task.delta_ticks;
      prev = curr;
      curr = curr->next;
    }

    /* Store remaining delta */
    node->task.delta_ticks = ticks - accumulated;

    /* Adjust next node's delta */
    if (curr) {
      curr->task.delta_ticks -= node->task.delta_ticks;
    }

    /* Insert node */
    if (prev == NULL) {
      node->next = kernel.blocked_queue.head;
      kernel.blocked_queue.head = node;
    } else {
      node->next = curr;
      prev->next = node;
      if (curr == NULL) {
        kernel.blocked_queue.tail = node;
      }
    }
  }
  kernel.blocked_queue.count++;
}

/* Task Delay - O(1) using stored current_node pointer */
void os_delay(uint16_t ticks) {
  if (kernel.current_task && kernel.current_node && ticks > 0) {
    kernel.current_task->state = TASK_BLOCKED;

    /* O(1) removal: we have direct pointer to current node */
    TaskNode *node = kernel.current_node;

    /* Remove from ready queue */
    TaskNode *prev = NULL;
    TaskNode *curr = kernel.ready_queue.head;
    while (curr && curr != node) {
      prev = curr;
      curr = curr->next;
    }

    if (curr == node) {
      if (prev) {
        prev->next = curr->next;
      } else {
        kernel.ready_queue.head = curr->next;
      }
      if (curr == kernel.ready_queue.tail) {
        kernel.ready_queue.tail = prev;
      }
      kernel.ready_queue.count--;

      /* Add to blocked queue using delta encoding */
      blocked_queue_add(node, ticks);
    }

    kernel.current_task = NULL;
    kernel.current_node = NULL;
  }
}

/* Task Yield - voluntary context switch */
void os_task_yield(void) {
  if (kernel.current_task) {
    kernel.current_task->state = TASK_READY;
    kernel.current_task = NULL;
    kernel.current_node = NULL;
  }
}

/* Simple Memory Allocation (bump allocator) - for user data */
void *os_malloc(uint16_t size) {
  /* Align to 2-byte boundary */
  size = (size + 1) & ~1;

  if (kernel.mem_manager.allocated + size > kernel.mem_manager.total_size) {
    return NULL; /* Out of memory */
  }

  uint8_t *ptr = kernel.mem_manager.memory_pool + kernel.mem_manager.allocated;
  kernel.mem_manager.allocated += size;
  return ptr;
}

/* Simple Memory Deallocation (no-op in bump allocator) */
void os_free(void *ptr) { (void)ptr; /* Unused in bump allocator */ }

/* System tick - O(1) delta queue processing */
void os_system_tick(void) {
  kernel.system_tick++;

  /* Delta queue: only decrement first node */
  if (kernel.blocked_queue.head == NULL)
    return;

  kernel.blocked_queue.head->task.delta_ticks--;

  /* Unblock all tasks with zero delta */
  while (kernel.blocked_queue.head &&
         kernel.blocked_queue.head->task.delta_ticks == 0) {
    TaskNode *node = kernel.blocked_queue.head;
    kernel.blocked_queue.head = node->next;
    if (kernel.blocked_queue.head == NULL) {
      kernel.blocked_queue.tail = NULL;
    }
    kernel.blocked_queue.count--;

    /* Move to ready queue by priority */
    node->task.state = TASK_READY;
    queue_add_priority(&kernel.ready_queue, node);
  }
}

/* Start the OS - infinite loop */
void os_start(void) {
  os_timer_init(); /* Start system tick timer */

  while (1) {
    os_scheduler();
    os_run_task();
  }
}
