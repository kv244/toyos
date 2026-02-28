#include "port.h"
/**
 * @file os_kernel.cpp
 * @brief Core implementation of the ToyOS RTOS kernel.
 *
 * Implements the scheduler, task management, synchronization primitives,
 * and memory management for the operating system.
 */

#include "toyos.h"
#include "toyos_config.h"
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

/* Enable allocator debug prints for troubleshooting */
#ifndef TOYOS_DEBUG_ALLOC
#define TOYOS_DEBUG_ALLOC 0
#endif

#ifndef TOYOS_MAX_TASKS
#define TOYOS_MAX_TASKS 4
#endif

/* Static Task Pool */
static TaskNode static_task_pool[MAX_TASKS];
static TaskNode *task_pool = static_task_pool;
static uint8_t task_pool_index = 0;

/* Global Kernel */
static Kernel kernel = {0};

/* Global pointer for ASM access */
extern "C" TaskControlBlock *os_current_task_ptr;
TaskControlBlock *os_current_task_ptr = NULL;

/* Forward Decls of Internal Kernel Functions */
extern "C" {
void k_yield(void);
void k_delay(uint16_t ticks);
void k_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                   uint16_t stack_size);
void *k_malloc(size_t size);
void k_free(void *ptr);
void k_mutex_init(Mutex *mutex);
void k_mutex_lock(Mutex *mutex);
void k_mutex_unlock(Mutex *mutex);
void k_sem_init(Semaphore *sem, uint8_t initial_count);
void k_sem_wait(Semaphore *sem);
void k_sem_post(Semaphore *sem);
uint32_t k_get_time(void);
}

/* Inline helper */
static inline TaskNode *get_task_node(void) __attribute__((always_inline));
static inline TaskNode *get_task_node(void) {
  TaskNode *node = NULL;
  if (task_pool_index < MAX_TASKS) {
    node = &task_pool[task_pool_index++];
  }
  return node;
}

/* Heap Helpers from MISRA refactor */
/* Heap Helpers from MISRA refactor */
static inline void heap_push(TaskNode *node) __attribute__((always_inline));
static inline void heap_push(TaskNode *node) {
  ASSERT(kernel.ready_heap.size < MAX_TASKS);
  if (kernel.ready_heap.size < MAX_TASKS) {
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
}

static inline TaskNode *heap_pop(void) __attribute__((always_inline));
static inline TaskNode *heap_pop(void) {
  TaskNode *root = NULL;
  if (kernel.ready_heap.size > 0) {
    root = kernel.ready_heap.nodes[0];
    kernel.ready_heap.size--;
    if (kernel.ready_heap.size > 0) {
      TaskNode *last = kernel.ready_heap.nodes[kernel.ready_heap.size];
      uint8_t i = 0;
      bool done = false;
      while (!done && (2 * i + 1) < kernel.ready_heap.size) {
        uint8_t left = 2 * i + 1;
        uint8_t right = 2 * i + 2;
        uint8_t largest = left;
        if (right < kernel.ready_heap.size &&
            kernel.ready_heap.nodes[right]->task.priority >
                kernel.ready_heap.nodes[left]->task.priority) {
          largest = right;
        }
        if (last->task.priority >=
            kernel.ready_heap.nodes[largest]->task.priority) {
          done = true;
        } else {
          kernel.ready_heap.nodes[i] = kernel.ready_heap.nodes[largest];
          i = largest;
        }
      }
      kernel.ready_heap.nodes[i] = last;
    }
  }
  return root;
}

/* --- KERNEL IMPLEMENTATION (k_ functions) --- */

void k_init(uint8_t *mem_pool, uint16_t mem_size) {
  memset(&kernel, 0, sizeof(Kernel));

  /* Init Allocator */
  uintptr_t start_addr = (uintptr_t)mem_pool;
  if (start_addr % 2 != 0) {
    start_addr++;
    mem_size--;
  }

  typedef struct BlockHeader {
    uint16_t size;
    struct BlockHeader *next;
  } BlockHeader;
  BlockHeader *first_block = (BlockHeader *)start_addr;
  first_block->size = mem_size - sizeof(BlockHeader);
  first_block->next = NULL;
  kernel.mem_manager.free_list_head = (void *)first_block;

  /* silent init */

  /* Static Task Pool already set up */
  task_pool_index = 0;

#if TOYOS_USE_MPU
  port_mpu_init();
#endif
}

void k_start(void) {
  if (kernel.task_count > 0) {
    os_scheduler();
    if (kernel.current_task != NULL) {
      port_start_first_task((uint8_t *)kernel.current_task->stack_ptr);
    }
  }
}

void k_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                   uint16_t stack_size) {
  TaskNode *new_node = get_task_node();
  if (new_node == NULL) {
    return;
  }

  void *stack_mem = k_malloc(stack_size);
  if (stack_mem == NULL) {
    return;
  }

  uint32_t *canary_ptr = (uint32_t *)stack_mem;
  *canary_ptr = 0xDEADBEEF;

  uint8_t *stack_top = (uint8_t *)stack_mem + stack_size;

  new_node->task.id = id;
  new_node->task.state = TASK_READY;
  new_node->task.priority = priority;
  new_node->task.base_priority = priority;
  new_node->task.stack_size = stack_size;
  new_node->task.canary_ptr = canary_ptr;
  new_node->task.stack_ptr = port_init_stack(stack_top, stack_size, task_func);
  if (new_node->task.stack_ptr == NULL) {
    k_free(stack_mem);
    task_pool_index--;
    return;
  }

  port_enter_critical();
  heap_push(new_node);
  kernel.task_count++;
  port_exit_critical();
}

extern "C" void os_update_ticks(void);

void k_yield(void) {
  os_update_ticks();
  port_context_switch();
}

/* Optimized Scheduler (Hot Path) */
TOYOS_HOT void os_scheduler(void) {
  port_enter_critical();
  bool needs_switch = true;

  if (kernel.current_node && (kernel.current_task->state == TASK_RUNNING ||
                              kernel.current_task->state == TASK_READY)) {
    /* Fast Path Check */
    if (kernel.ready_heap.size == 0 ||
        (kernel.ready_heap.size > 0 &&
         kernel.current_task->priority >=
             kernel.ready_heap.nodes[0]->task.priority)) {
      needs_switch = false;
    }

    if (needs_switch) {
      kernel.current_task->state = TASK_READY;
      heap_push(kernel.current_node);
    }
  }

  if (needs_switch) {
    TaskNode *next_node = heap_pop();
    if (next_node != NULL) {
      next_node->task.state = TASK_RUNNING;
      os_current_task_ptr = &next_node->task;
      kernel.current_task = &next_node->task;
      kernel.current_node = next_node;
    }
  }
  port_exit_critical();
}

TOYOS_HOT void os_system_tick(void) {
  kernel.system_tick++;
  if (kernel.blocked_queue.head) {
    if (kernel.blocked_queue.head->task.delta_ticks > 0)
      kernel.blocked_queue.head->task.delta_ticks--;

    while (kernel.blocked_queue.head &&
           kernel.blocked_queue.head->task.delta_ticks == 0) {
      TaskNode *waking = kernel.blocked_queue.head;
      kernel.blocked_queue.head = waking->next;
      if (kernel.blocked_queue.head == NULL)
        kernel.blocked_queue.tail = NULL;

      waking->task.state = TASK_READY;
      if (waking != kernel.current_node) {
        heap_push(waking);
      }
    }
  }
}

void k_delay(uint16_t ticks) {
  if (ticks == 0)
    return;
  port_enter_critical();
  TaskNode *node = kernel.current_node;
  if (node) {
    node->task.state = TASK_BLOCKED;

    /* Orderly Insert into Delta Queue */
    TaskNode **curr = &kernel.blocked_queue.head;
    TaskNode *entry = kernel.blocked_queue.head;
    uint16_t delay = ticks;

    while (entry && delay >= entry->task.delta_ticks) {
      delay -= entry->task.delta_ticks;
      curr = &entry->next;
      entry = entry->next;
    }

    node->task.delta_ticks = delay;
    node->next = entry;
    *curr = node;

    if (entry) {
      entry->task.delta_ticks -= delay;
    }
  }
  port_exit_critical();

  /* Cooperative spin: yield repeatedly until we are unblocked.
     os_update_ticks (called inside port_context_switch) will
     decrement delta_ticks and move us back to READY state. */
  while (kernel.current_task->state == TASK_BLOCKED) {
    k_yield();
  }
}

void *k_malloc(size_t size) {
  void *allocated_buffer = NULL;
  /* Simple first fit (assuming mem_manager initialized) */
  typedef struct BlockHeader {
    uint16_t size;
    struct BlockHeader *next;
  } BlockHeader;

#ifdef __arm__
#define TOYOS_MEM_ALIGN_MASK 0x7
#else
#define TOYOS_MEM_ALIGN_MASK 0x1
#endif

  size = (size + TOYOS_MEM_ALIGN_MASK) & ~TOYOS_MEM_ALIGN_MASK;
  uint16_t total_size = size + sizeof(BlockHeader);

  port_enter_critical();
  BlockHeader *prev = NULL;
  BlockHeader *curr = (BlockHeader *)kernel.mem_manager.free_list_head;

  while (curr != NULL && allocated_buffer == NULL) {
    if (curr->size >= total_size) {
      /* Split logic */
      if (curr->size >= total_size + sizeof(BlockHeader) + 4) {
        BlockHeader *new_block = (BlockHeader *)((uint8_t *)curr + total_size);
        new_block->size = curr->size - total_size;
        new_block->next = curr->next;
        curr->size = size;
        curr->next = new_block;
      }
      /* Allocate */
      if (prev != NULL) {
        prev->next = curr->next;
      } else {
        kernel.mem_manager.free_list_head = curr->next;
      }
      allocated_buffer = (void *)((uint8_t *)curr + sizeof(BlockHeader));
    } else {
      prev = curr;
      curr = curr->next;
    }
  }

  port_exit_critical();
  return allocated_buffer;
}

void k_free(void *ptr) {
  if (!ptr)
    return;

  typedef struct BlockHeader {
    uint16_t size;
    struct BlockHeader *next;
  } BlockHeader;

  BlockHeader *block = (BlockHeader *)((uint8_t *)ptr - sizeof(BlockHeader));
  port_enter_critical();

  /* Insert sorted by Addr */
  BlockHeader *prev = NULL;
  BlockHeader *curr = (BlockHeader *)kernel.mem_manager.free_list_head;

  while (curr && curr < block) {
    prev = curr;
    curr = curr->next;
  }

  /* Insert */
  block->next = curr;
  if (prev)
    prev->next = block;
  else
    kernel.mem_manager.free_list_head = (void *)block;

  /* Coalesce Next */
  if (curr &&
      (uint8_t *)block + sizeof(BlockHeader) + block->size == (uint8_t *)curr) {
    block->size += sizeof(BlockHeader) + curr->size;
    block->next = curr->next;
  }

  /* Coalesce Prev */
  if (prev &&
      (uint8_t *)prev + sizeof(BlockHeader) + prev->size == (uint8_t *)block) {
    prev->size += sizeof(BlockHeader) + block->size;
    prev->next = block->next;
  }

  port_exit_critical();
}

void k_mutex_init(Mutex *m) {
  m->locked = 0;
  m->owner = NULL;
  m->blocked_tasks.head = NULL;
  m->blocked_tasks.tail = NULL;
  m->blocked_tasks.count = 0;
}

void k_mutex_lock(Mutex *m) {
  port_enter_critical();
  if (m->locked) {
    /* Priority Inheritance */
    TaskControlBlock *owner = m->owner;
    if (owner && owner->priority < kernel.current_task->priority) {
      owner->priority = kernel.current_task->priority;
    }

    /* Block */
    TaskNode *node = kernel.current_node;
    node->next = NULL;
    if (m->blocked_tasks.tail) {
      m->blocked_tasks.tail->next = node;
    } else {
      m->blocked_tasks.head = node;
    }
    m->blocked_tasks.tail = node;
    m->blocked_tasks.count++;

    node->task.state = TASK_BLOCKED;
    port_exit_critical();
    k_yield();
  } else {
    m->locked = 1;
    m->owner = kernel.current_task;
    port_exit_critical();
  }
}

void k_mutex_unlock(Mutex *m) {
  port_enter_critical();

  /* Enforce Ownership */
  if (m->owner != kernel.current_task) {
    port_exit_critical();
    return;
  }

  /* Restore Priority */
  kernel.current_task->priority = kernel.current_task->base_priority;

  if (m->blocked_tasks.head) {
    TaskNode *waking = m->blocked_tasks.head;
    m->blocked_tasks.head = waking->next;
    if (!m->blocked_tasks.head)
      m->blocked_tasks.tail = NULL;
    m->blocked_tasks.count--;

    m->owner = &waking->task;
    waking->task.state = TASK_READY;
    heap_push(waking);
  } else {
    m->locked = 0;
    m->owner = NULL;
  }
  port_exit_critical();
}

/* Semaphores with Direct Handoff */
void k_sem_init(Semaphore *s, uint8_t c) {
  s->count = c;
  s->blocked_tasks.head = NULL;
  s->blocked_tasks.tail = NULL;
  s->blocked_tasks.count = 0;
}

void k_sem_wait(Semaphore *s) {
  port_enter_critical();
  if (s->count > 0) {
    s->count--;
  } else {
    /* Block */
    TaskNode *node = kernel.current_node;
    node->next = NULL;
    if (s->blocked_tasks.tail) {
      s->blocked_tasks.tail->next = node;
    } else {
      s->blocked_tasks.head = node;
    }
    s->blocked_tasks.tail = node;
    s->blocked_tasks.count++;

    node->task.state = TASK_BLOCKED;
    port_exit_critical();
    k_yield();
    return; /* On wakeup, resource is handed off */
  }
  port_exit_critical();
}

void k_sem_post(Semaphore *s) {
  port_enter_critical();
  if (s->blocked_tasks.head) {
    /* Direct Handoff - Wake one task, do not increment count */
    TaskNode *waking = s->blocked_tasks.head;
    s->blocked_tasks.head = waking->next;
    if (!s->blocked_tasks.head)
      s->blocked_tasks.tail = NULL;
    s->blocked_tasks.count--;

    waking->task.state = TASK_READY;
    heap_push(waking);
  } else {
    s->count++;
  }
  port_exit_critical();
}
#if TOYOS_ENABLE_MESSAGE_QUEUES
/* Message Queues currently not implemented in this port */
#endif

uint32_t k_get_time(void) { return kernel.system_tick; }

/* --- PUBLIC API WRAPPERS --- */

void os_init(uint8_t *pool, uint16_t size) { k_init(pool, size); }
void os_start(void) { k_start(); }

void os_yield(void) { k_yield(); }
void os_delay(uint16_t ticks) { k_delay(ticks); }
void os_create_task(uint8_t id, void (*f)(void), uint8_t p, uint16_t s) {
  k_create_task(id, f, p, s);
}
void *os_malloc(uint16_t size) { return k_malloc(size); }
void os_free(void *p) { k_free(p); }
void os_mutex_init(Mutex *m) { k_mutex_init(m); }
void os_mutex_lock(Mutex *m) { k_mutex_lock(m); }
void os_mutex_unlock(Mutex *m) { k_mutex_unlock(m); }

/* Stubs for others */
void os_sem_init(Semaphore *s, uint8_t c) { k_sem_init(s, c); }
void os_sem_wait(Semaphore *s) { k_sem_wait(s); }
void os_sem_post(Semaphore *s) { k_sem_post(s); }

/* --- UTILITY FUNCTIONS --- */

void os_enter_idle(void) { __asm volatile("wfi"); }

void os_wdt_feed(void) { port_wdt_feed(); }

void os_wdt_init(uint16_t timeout_ms) { port_wdt_init(timeout_ms); }

void os_print(const char *msg) { Serial.println(msg); }

void os_print_p(const char *msg_p) { os_print(msg_p); }
