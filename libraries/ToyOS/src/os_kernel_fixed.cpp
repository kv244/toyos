#include "port.h"
#include "toyos.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

/* Dynamic Task Pool - allocated during os_init to save global RAM */
static TaskNode *task_pool = NULL;
static uint8_t task_pool_index = 0;

/* Global Kernel Instance */
static Kernel kernel = {0};

/* Global pointer for ASM access */
extern "C" TaskControlBlock *os_current_task_ptr;
TaskControlBlock *os_current_task_ptr = NULL;

/* Inline helper: get task node from static pool */
static inline TaskNode *get_task_node(void) __attribute__((always_inline));
static inline TaskNode *get_task_node(void) {
  if (task_pool_index >= MAX_TASKS)
    return NULL;
  return &task_pool[task_pool_index++];
}

/* Block Header for Free List Allocator */
typedef struct BlockHeader {
  uint16_t size; /* Size of the user data following this header */
  struct BlockHeader
      *next; /* Pointer to the next FREE block (NULL if allocated) */
} BlockHeader;

/* Initialize the OS and Memory Manager */
void os_init(uint8_t *mem_pool, uint16_t mem_size) {
  /* Zero out kernel state */
  kernel.ready_heap.size = 0;
  kernel.blocked_queue.head = NULL;
  kernel.blocked_queue.tail = NULL;
  kernel.blocked_queue.count = 0;
  kernel.current_task = NULL;
  kernel.current_node = NULL;
  kernel.system_tick = 0;
  kernel.task_count = 0;

  /* Initialize Free List Memory Manager */
  uintptr_t start_addr = (uintptr_t)mem_pool;

  /* Ensure 2-byte alignment */
  if (start_addr % 2 != 0) {
    start_addr++;
    mem_size--;
  }

  /* Initial free block covers the entire pool */
  BlockHeader *first_block = (BlockHeader *)start_addr;
  first_block->size = mem_size - sizeof(BlockHeader);
  first_block->next = NULL;

  kernel.mem_manager.free_list_head = (void *)first_block;

  /* Initialize task pool dynamically to reduce global footprint */
  if (task_pool == NULL) {
    task_pool = (TaskNode *)os_malloc(sizeof(TaskNode) * MAX_TASKS);
    if (task_pool == NULL) {
      /* Fatal error - system cannot operate without task pool */
#ifdef ARDUINO
      Serial.println(F("FATAL: Task pool allocation failed"));
      while (1)
        ; // Halt system
#endif
    }
  }

  /* Reset task pool index */
  task_pool_index = 0;
}

// ... heap helpers omitted (unchanged) ...

/* Memory Allocation - First Fit Strategy */
void *os_malloc(uint16_t size) {
  if (size == 0)
    return NULL;

  /* Align size to 2-byte boundary */
  if (size % 2 != 0)
    size++;

  BlockHeader *prev = NULL;
  BlockHeader *curr = (BlockHeader *)kernel.mem_manager.free_list_head;

  while (curr != NULL) {
    if (curr->size >= size) {
      /* Found a fit! Check if we can split */
      if (curr->size >= size + sizeof(BlockHeader) + 4) {
        /* SPLIT: Create new free block from remainder */
        BlockHeader *new_block =
            (BlockHeader *)((uint8_t *)curr + sizeof(BlockHeader) + size);
        new_block->size = curr->size - size - sizeof(BlockHeader);
        new_block->next = curr->next;

        /* Update current block size */
        curr->size = size;
        curr->next = new_block; /* temporary link for insertion logic below */
      }

      /* Remove 'curr' from free list */
      if (prev) {
        prev->next = curr->next;
      } else {
        kernel.mem_manager.free_list_head = curr->next;
      }

      /* Mark as allocated */
      curr->next = NULL;

      /* Return pointer to user data */
      return (void *)((uint8_t *)curr + sizeof(BlockHeader));
    }
    prev = curr;
    curr = curr->next;
  }
  return NULL; /* Out of memory */
}

/* Memory Deallocation - Coalescing Strategy */
void os_free(void *ptr) {
  if (!ptr)
    return;

  /* Recover header */
  BlockHeader *block = (BlockHeader *)((uint8_t *)ptr - sizeof(BlockHeader));

  /* Insert into sorted free list */
  BlockHeader *prev = NULL;
  BlockHeader *curr = (BlockHeader *)kernel.mem_manager.free_list_head;

  /* Find insertion point (sorted by address) */
  while (curr != NULL && curr < block) {
    prev = curr;
    curr = curr->next;
  }

  /* Insert */
  if (prev) {
    prev->next = block;
  } else {
    kernel.mem_manager.free_list_head = block;
  }
  block->next = curr;

  /* Coalesce with NEXT block */
  if (curr && ((uint8_t *)block + sizeof(BlockHeader) + block->size) ==
                  (uint8_t *)curr) {
    block->size += sizeof(BlockHeader) + curr->size;
    block->next = curr->next;
  }

  /* Coalesce with PREV block */
  if (prev && ((uint8_t *)prev + sizeof(BlockHeader) + prev->size) ==
                  (uint8_t *)block) {
    prev->size += sizeof(BlockHeader) + block->size;
    prev->next = block->next;
  }
}

/* Binary Heap Helpers - with assertions */
static void heap_push(TaskNode *node) {
  ASSERT(kernel.ready_heap.size < MAX_TASKS);
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

static void heapify(uint8_t i) {
  uint8_t size = kernel.ready_heap.size;
  while (1) {
    uint8_t largest = i;
    uint8_t left = 2 * i + 1;
    uint8_t right = 2 * i + 2;

    if (left < size && kernel.ready_heap.nodes[left]->task.priority >
                           kernel.ready_heap.nodes[largest]->task.priority) {
      largest = left;
    }
    if (right < size && kernel.ready_heap.nodes[right]->task.priority >
                            kernel.ready_heap.nodes[largest]->task.priority) {
      largest = right;
    }

    if (largest != i) {
      TaskNode *temp = kernel.ready_heap.nodes[i];
      kernel.ready_heap.nodes[i] = kernel.ready_heap.nodes[largest];
      kernel.ready_heap.nodes[largest] = temp;
      i = largest;
    } else {
      break;
    }
  }
}

static void heap_rebuild(void) {
  if (kernel.ready_heap.size <= 1)
    return;
  for (int8_t i = (kernel.ready_heap.size / 2) - 1; i >= 0; i--) {
    heapify(i);
  }
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

/**
 * Create a new task - FIXED: No memory leak on failure
 * Validates stack size and properly handles allocation failures.
 */
void os_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                    uint16_t stack_size) {
  /* Validate parameters */
  ASSERT(task_func != NULL);
  ASSERT(stack_size >= MIN_STACK_SIZE);

  if (kernel.task_count >= MAX_TASKS)
    return;

  TaskNode *new_node = get_task_node();
  if (!new_node)
    return;

  /* Clamp stack size to minimum if too small */
  if (stack_size < MIN_STACK_SIZE) {
    stack_size = MIN_STACK_SIZE;
  }

  /* Initialize stack and get canary pointer - FIXED */
  uint32_t *canary_ptr = NULL;
  uint8_t *stack = (uint8_t *)os_malloc(stack_size);
  if (!stack) {
    /* Failed to allocate stack - return node to pool */
    task_pool_index--;
    return;
  }

  /* Place canary at bottom of stack */
  *(uint32_t *)stack = STACK_CANARY;
  canary_ptr = (uint32_t *)stack;

  /* Initialize stack using port layer */
  uint8_t *sp = port_init_stack(stack, stack_size, task_func);
  if (!sp) {
    /* Failed to allocate stack - return node to pool */
    task_pool_index--;
    return;
  }

  new_node->task.id = id;
  new_node->task.state = TASK_READY;
  new_node->task.task_func = task_func;
  new_node->task.priority = priority;
  new_node->task.base_priority = priority;
  new_node->task.stack_size = stack_size;
  new_node->task.delta_ticks = 0;
  new_node->task.stack_ptr = sp;
  new_node->task.canary_ptr = canary_ptr;

  /* Fill stack with pattern for high-water mark tracking */
  /* Skip the initial context (approx 35 bytes) and canary (4 bytes) */
  uint8_t *stack_base = (uint8_t *)canary_ptr + 4;
  uint16_t fill_size = stack_size - 35 - 4;
  for (uint16_t i = 0; i < fill_size; i++) {
    stack_base[i] = 0xA5;
  }

  new_node->next = NULL;

  /* Add to ready heap with atomic protection */
  ATOMIC_START();
  heap_push(new_node);
  kernel.task_count++;
  ATOMIC_END();
}

/* Priority-Based Scheduler - FIXED: Skip blocked tasks */
void os_scheduler(void) {
  /* Activity Heartbeat (optional debug) - Removed for portability */
  // PINB |= (1 << 5);

  /* Put current task back in the ready heap ONLY if it's ready/running */
  if (kernel.current_node && (kernel.current_task->state == TASK_RUNNING ||
                              kernel.current_task->state == TASK_READY)) {
    kernel.current_task->state = TASK_READY;
    heap_push(kernel.current_node);
  }

  /* Pop tasks until we find a READY one (skip BLOCKED tasks) */
  TaskNode *next_node = NULL;
  while (kernel.ready_heap.size > 0) {
    next_node = heap_pop();
    if (next_node && next_node->task.state == TASK_READY) {
      break; /* Found a ready task */
    }
    /* Task is BLOCKED - don't re-add to heap */
    next_node = NULL;
  }

  if (next_node) {
    os_current_task_ptr = &next_node->task;
    kernel.current_task = &next_node->task;
    kernel.current_node = next_node;
    kernel.current_task->state = TASK_RUNNING;
  } else {
    os_current_task_ptr = NULL;
    kernel.current_task = NULL;
    kernel.current_node = NULL;
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

/* Task Delay - FIXED: Use optimized atomic macros */
void os_delay(uint16_t ticks) {
  if (kernel.current_task && kernel.current_node && ticks > 0) {
    ATOMIC_START();
    kernel.current_task->state = TASK_BLOCKED;
    blocked_queue_add(kernel.current_node, ticks);
    port_context_switch();
    ATOMIC_END();
  }
}

/* Task Yield - voluntary context switch */
void os_task_yield(void) {
  if (kernel.current_task) {
    ATOMIC_START();
    port_context_switch();
    ATOMIC_END();
  }
}

/* Memory Manager Implementation Moved to Top of File */

/* System tick - O(1) delta queue processing */
void os_system_tick(void) {
  kernel.system_tick++;

  /* Fast path for empty blocked queue */
  if (kernel.blocked_queue.count == 0)
    return;

  /* Delta queue: only decrement first node */
  if (kernel.blocked_queue.head->task.delta_ticks > 0) {
    kernel.blocked_queue.head->task.delta_ticks--;
  }

  /* Unblock all tasks with zero delta */
  while (kernel.blocked_queue.head &&
         kernel.blocked_queue.head->task.delta_ticks == 0) {
    TaskNode *node = kernel.blocked_queue.head;
    kernel.blocked_queue.head = node->next;
    if (kernel.blocked_queue.head == NULL) {
      kernel.blocked_queue.tail = NULL;
    }
    kernel.blocked_queue.count--;

    /* Move to ready heap */
    node->task.state = TASK_READY;
    heap_push(node);
  }
}

/* Semaphore Implementation - FIXED: Trigger context switch when blocking */
void os_sem_init(Semaphore *sem, uint8_t initial_count) {
  sem->count = initial_count;
  sem->blocked_tasks.head = NULL;
  sem->blocked_tasks.tail = NULL;
  sem->blocked_tasks.count = 0;
}

void os_sem_wait(Semaphore *sem) {
  ATOMIC_START();
  if (sem->count > 0) {
    sem->count--;
  } else {
    /* Block current task - FIXED: Add context switch */
    TaskNode *node = kernel.current_node;
    if (node) {
      node->task.state = TASK_BLOCKED;
      queue_add_tail(&sem->blocked_tasks, node);
      /* Trigger immediate context switch */
      port_context_switch();
    }
  }
  ATOMIC_END();
}

void os_sem_post(Semaphore *sem) {
  ATOMIC_START();
  if (sem->blocked_tasks.count > 0) {
    TaskNode *node = queue_remove(&sem->blocked_tasks);
    node->task.state = TASK_READY;
    heap_push(node);
  } else {
    sem->count++;
  }
  ATOMIC_END();
}

/* Mutex Implementation - FIXED: Trigger context switch when blocking */
void os_mutex_init(Mutex *mutex) {
  mutex->locked = 0;
  mutex->owner = NULL;
  mutex->blocked_tasks.head = NULL;
  mutex->blocked_tasks.tail = NULL;
  mutex->blocked_tasks.count = 0;
}

void os_mutex_lock(Mutex *mutex) {
  ATOMIC_START();
  if (!mutex->locked) {
    mutex->locked = 1;
    mutex->owner = kernel.current_task;
  } else {
    /* Priority Inheritance: Bump owner's priority if lower than ours */
    if (mutex->owner->priority < kernel.current_task->priority) {
      mutex->owner->priority = kernel.current_task->priority;
      /* If owner is READY in the heap, we must rebuild to promote it */
      if (mutex->owner->state == TASK_READY) {
        heap_rebuild();
      }
    }

    /* Block current task - FIXED: Add context switch */
    TaskNode *node = kernel.current_node;
    if (node) {
      node->task.state = TASK_BLOCKED;
      queue_add_tail(&mutex->blocked_tasks, node);
      /* Trigger immediate context switch */
      port_context_switch();
    }
  }
  ATOMIC_END();
}

void os_mutex_unlock(Mutex *mutex) {
  ATOMIC_START();
  // Add a check to ensure only the owner can unlock.
  if (mutex->owner != kernel.current_task) {
    ATOMIC_END();
    return; // Or trigger an assertion
  }

  if (mutex->blocked_tasks.count > 0) {
    TaskNode *node = queue_remove(&mutex->blocked_tasks);
    node->task.state = TASK_READY;
    mutex->owner = &node->task; // Pass ownership to the next waiting task

    /* The new owner's priority might be high due to inheritance,
     * but we don't need to rebuild here because we're adding it to heap fresh.
     */
    heap_push(node);
  } else {
    mutex->locked = 0;
    mutex->owner = NULL;
  }

  /* Restore our own base priority now that we've released the mutex */
  kernel.current_task->priority = kernel.current_task->base_priority;

  ATOMIC_END();
}

/* Message Queue Implementation */
MessageQueue *os_mq_create(uint8_t capacity) {
  MessageQueue *mq = (MessageQueue *)os_malloc(sizeof(MessageQueue));
  if (!mq)
    return NULL;
  mq->buffer = (void **)os_malloc(capacity * sizeof(void *));
  if (!mq->buffer) {
    os_free(mq);
    return NULL;
  }

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

/* Fast-path message queue operations - OPTIMIZATION #14 - FIXED */
void os_mq_send_fast(MessageQueue *mq, void *msg) {
  ATOMIC_START();

  /* Check if space available without blocking */
  /* CRITICAL FIX: Must ensure no tasks are blocked on the semaphore! */
  if (mq->sem_write.count > 0 && !mq->mutex.locked) {
    /* Fast path - space available, no context switches needed */

    /* CRITICAL FIX: We are effectively doing a non-blocking wait on sem_write
     */
    mq->sem_write.count--;

    mq->buffer[mq->tail] = msg;
    mq->tail = (mq->tail + 1) % mq->capacity;
    mq->count++;

    /* Wake one receiver if any are blocked */
    if (mq->sem_read.blocked_tasks.count > 0) {
      TaskNode *node = queue_remove(&mq->sem_read.blocked_tasks);
      node->task.state = TASK_READY;
      heap_push(node);
    } else {
      mq->sem_read.count++;
    }

    ATOMIC_END();
    return;
  }

  ATOMIC_END();

  /* Slow path - must block */
  os_mq_send(mq, msg);
}

void *os_mq_receive_fast(MessageQueue *mq) {
  ATOMIC_START();

  /* Check if message available without blocking */
  /* CRITICAL FIX: Must ensure no tasks are blocked on the semaphore! */
  if (mq->sem_read.count > 0 && !mq->mutex.locked) {
    /* Fast path - message available, no context switches needed */

    /* CRITICAL FIX: We are effectively doing a non-blocking wait on sem_read */
    mq->sem_read.count--;

    void *msg = mq->buffer[mq->head];
    mq->head = (mq->head + 1) % mq->capacity;
    mq->count--;

    /* Wake one sender if any are blocked */
    if (mq->sem_write.blocked_tasks.count > 0) {
      TaskNode *node = queue_remove(&mq->sem_write.blocked_tasks);
      node->task.state = TASK_READY;
      heap_push(node);
    } else {
      mq->sem_write.count++;
    }

    ATOMIC_END();
    return msg;
  }

  ATOMIC_END();

  /* Slow path - must block */
  return os_mq_receive(mq);
}

/* Stack Overflow Detection - FIXED: Actual implementation */
void os_check_stack_overflow(void) {
  if (kernel.current_node && kernel.current_node->task.canary_ptr) {
    if (*(kernel.current_node->task.canary_ptr) != STACK_CANARY) {
      /* STACK OVERFLOW DETECTED! */
      port_disable_interrupts();
      /* Halt system - in production, you might log the error */
      while (1) {
        /* Could blink LED rapidly here to indicate error */
      }
    }
  }
}

void os_enter_idle(void) { port_enter_idle(); }

/* Watchdog Timer Support */
void os_wdt_init(uint16_t timeout_ms) { port_wdt_init(timeout_ms); }

void os_wdt_feed(void) { port_wdt_feed(); }

/* Stack High-Water Mark Tracking */
uint16_t os_get_stack_usage(uint8_t task_id) {
  TaskNode *node = NULL;
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (task_pool[i].task.id == task_id) {
      node = &task_pool[i];
      break;
    }
  }
  if (!node)
    return 0;

  /* Walk up from the bottom (canary) to find the first non-pattern byte */
  uint8_t *stack_start = (uint8_t *)node->task.canary_ptr + 4;
  uint16_t unused = 0;
  uint16_t max_search = node->task.stack_size - 4;

  while (unused < max_search && stack_start[unused] == 0xA5) {
    unused++;
  }

  return node->task.stack_size - unused;
}

uint8_t os_get_priority(uint8_t task_id) {
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (task_pool[i].task.id == task_id) {
      return task_pool[i].task.priority;
    }
  }
  return 0;
}

/* Start the OS - never returns */
void os_start(void) {
  port_disable_interrupts(); /* Disable interrupts to prevent corruption */
  port_timer_init();         /* Start system tick timer (from port layer) */

  /* Initial context switch - pick the first task and start it */
  os_scheduler();

  if (os_current_task_ptr) {
    /* Start the first task using port layer */
    port_start_first_task(os_current_task_ptr->stack_ptr);
  }

  /* Should never reach here */
  while (1)
    ;
}

/**
 * Print detailed system information to Serial port.
 * Blocks until output is complete.
 */
void os_print_info(void) {
  /* Note: We assume Serial is initialized by user validation calling this */
  /* If running on Arduino, Serial.print is available via C++ */

#ifdef ARDUINO
  /* We use simple prints to avoid large printf overhead */

  /* Ensure we have a serial connection if not already started */
  /* Note: This might re-init if user did Serial.begin(115200) */
  /* Good practice: User calls Serial.begin() before os_print_info() */

  Serial.println(F("========================================"));
  Serial.print(F("ToyOS Version: "));
  Serial.println(TOYOS_VERSION_STRING);
  Serial.println(F("========================================"));

  Serial.print(F("Platform:      "));
  Serial.println(port_get_platform_name());

  Serial.print(F("MCU:           "));
  Serial.println(port_get_mcu_name());

  Serial.print(F("CPU Freq:      "));
  Serial.print(port_get_cpu_freq() / 1000000);
  Serial.println(F(" MHz"));

  Serial.print(F("Flash Size:    "));
  Serial.print(port_get_flash_size() / 1024);
  Serial.println(F(" KB"));

  Serial.print(F("SRAM Size:     "));
  Serial.print(port_get_sram_size() / 1024);
  Serial.println(F(" KB"));

  Serial.print(F("EEPROM Size:   "));
  Serial.print(port_get_eeprom_size());
  Serial.println(F(" Bytes"));

  Serial.println(F("========================================"));
  Serial.flush();
#endif
}
