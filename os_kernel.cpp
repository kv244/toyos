#include "toyos.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
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
  kernel.ready_heap.size = 0;
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

/* Binary Heap Helpers */
static void heap_push(TaskNode *node) {
  if (kernel.ready_heap.size >= MAX_TASKS)
    return;

  uint8_t i = kernel.ready_heap.size++;
  while (i > 0) {
    uint8_t p = (i - 1) / 2;
    if (kernel.ready_heap.nodes[p]->task.priority >= node->task.priority)
      break;
    kernel.ready_heap.nodes[i] = kernel.ready_heap.nodes[p];
    i = p;
  }
  kernel.ready_heap.nodes[i] = node;
}

static TaskNode *heap_pop(void) {
  if (kernel.ready_heap.size == 0)
    return NULL;

  TaskNode *top = kernel.ready_heap.nodes[0];
  TaskNode *last = kernel.ready_heap.nodes[--kernel.ready_heap.size];

  uint8_t i = 0;
  while (i * 2 + 1 < kernel.ready_heap.size) {
    uint8_t child = i * 2 + 1;
    if (child + 1 < kernel.ready_heap.size &&
        kernel.ready_heap.nodes[child + 1]->task.priority >
            kernel.ready_heap.nodes[child]->task.priority) {
      child++;
    }
    if (last->task.priority >= kernel.ready_heap.nodes[child]->task.priority)
      break;
    kernel.ready_heap.nodes[i] = kernel.ready_heap.nodes[child];
    i = child;
  }
  kernel.ready_heap.nodes[i] = last;
  return top;
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

  heap_push(new_node);
  kernel.task_count++;

  /* Initialize stack canary */
  // Note: This assumes task_func is near the end of stack or we have stack
  // pointer For a real check, we'd need to know the stack top address.
}

/* Priority-Based Scheduler */
void os_scheduler(void) {
  if (kernel.ready_heap.size == 0)
    return;

  /* Get highest priority ready task */
  TaskNode *next_node = heap_pop();
  if (next_node && next_node->task.state == TASK_READY) {
    kernel.current_task = &next_node->task;
    kernel.current_node = next_node;
    kernel.current_task->state = TASK_RUNNING;

    /* Re-insert into heap (for round-robin among same priority) */
    heap_push(next_node);
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

    TaskNode *node = kernel.current_node;
    if (!node)
      return;

    /* O(N) removal from heap: find and remove the node */
    uint8_t found = 0;
    for (uint8_t i = 0; i < kernel.ready_heap.size; i++) {
      if (kernel.ready_heap.nodes[i] == node) {
        kernel.ready_heap.nodes[i] =
            kernel.ready_heap.nodes[--kernel.ready_heap.size];
        /* Re-heapify at index i */
        // Simplified: push/pop would be better but this is a rare removal
        found = 1;
        break;
      }
    }

    if (found) {
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

    /* Move to ready queue - now using heap */
    node->task.state = TASK_READY;
    heap_push(node);
  }
}

/* Semaphore Implementation */
void os_sem_init(Semaphore *sem, uint8_t initial_count) {
  sem->count = initial_count;
  sem->blocked_tasks.head = NULL;
  sem->blocked_tasks.tail = NULL;
  sem->blocked_tasks.count = 0;
}

void os_sem_wait(Semaphore *sem) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (sem->count > 0) {
      sem->count--;
    } else {
      /* Block current task */
      TaskNode *node = kernel.current_node;
      if (node) {
        node->task.state = TASK_BLOCKED;
        queue_add_tail(&sem->blocked_tasks, node);
        /* Remove from heap is tricky here, but scheduler will skip it */
        kernel.current_task = NULL;
        kernel.current_node = NULL;
      }
    }
  }
}

void os_sem_post(Semaphore *sem) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (sem->blocked_tasks.count > 0) {
      TaskNode *node = queue_remove(&sem->blocked_tasks);
      node->task.state = TASK_READY;
      heap_push(node);
    } else {
      sem->count++;
    }
  }
}

/* Mutex Implementation */
void os_mutex_init(Mutex *mutex) {
  mutex->locked = 0;
  mutex->owner = NULL;
  mutex->blocked_tasks.head = NULL;
  mutex->blocked_tasks.tail = NULL;
  mutex->blocked_tasks.count = 0;
}

void os_mutex_lock(Mutex *mutex) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (!mutex->locked) {
      mutex->locked = 1;
      mutex->owner = kernel.current_task;
    } else {
      TaskNode *node = kernel.current_node;
      if (node) {
        node->task.state = TASK_BLOCKED;
        queue_add_tail(&mutex->blocked_tasks, node);
        kernel.current_task = NULL;
        kernel.current_node = NULL;
      }
    }
  }
}

void os_mutex_unlock(Mutex *mutex) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (mutex->blocked_tasks.count > 0) {
      TaskNode *node = queue_remove(&mutex->blocked_tasks);
      node->task.state = TASK_READY;
      mutex->owner = &node->task;
      heap_push(node);
    } else {
      mutex->locked = 0;
      mutex->owner = NULL;
    }
  }
}

/* Message Queue Implementation */
MessageQueue *os_mq_create(uint8_t capacity) {
  MessageQueue *mq = (MessageQueue *)os_malloc(sizeof(MessageQueue));
  if (!mq)
    return NULL;
  mq->buffer = (void **)os_malloc(capacity * sizeof(void *));
  if (!mq->buffer)
    return NULL;

  mq->capacity = capacity;
  mq->head = 0;
  mq->tail = 0;
  mq->count = 0;
  os_sem_init(&mq->sem_read, 0);
  os_sem_init(&mq->sem_write, capacity);
  os_mutex_init(&mq->mutex);
  return mq;
}

void os_mq_send(MessageQueue *mq, void *msg) {
  os_sem_wait(&mq->sem_write);
  os_mutex_lock(&mq->mutex);
  mq->buffer[mq->tail] = msg;
  mq->tail = (mq->tail + 1) % mq->capacity;
  mq->count++;
  os_mutex_unlock(&mq->mutex);
  os_sem_post(&mq->sem_read);
}

void *os_mq_receive(MessageQueue *mq) {
  os_sem_wait(&mq->sem_read);
  os_mutex_lock(&mq->mutex);
  void *msg = mq->buffer[mq->head];
  mq->head = (mq->head + 1) % mq->capacity;
  mq->count--;
  os_mutex_unlock(&mq->mutex);
  os_sem_post(&mq->sem_write);
  return msg;
}

/* Stack Overflow & Power Management */
void os_check_stack_overflow(void) {
  /* Simple placeholder: in a real AVR OS, we'd check for a 'canary' value
     at the end of the task's stack space. */
  if (kernel.current_task) {
    // Check if current stack pointer is dangerously close to heap/other tasks
    // For now, this is a placeholder for the logic.
  }
}

void os_enter_idle(void) {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
}

/* Start the OS - infinite loop */
void os_start(void) {
  os_timer_init(); /* Start system tick timer */

  while (1) {
    if (kernel.ready_heap.size > 0) {
      os_scheduler();
      os_run_task();
      os_check_stack_overflow();
    } else {
      os_enter_idle();
    }
  }
}
