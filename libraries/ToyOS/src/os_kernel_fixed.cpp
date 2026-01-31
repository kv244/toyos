#include "port.h"
#include "toyos.h"
#include "toyos_config.h"
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#if TOYOS_USE_MPU && (defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__))
#include "port/arm/port_arm_mpu.h"
#define TOYOS_USE_SVC 1
#else
#define TOYOS_USE_SVC 0
#endif

/* SYSCALL IDs */
enum {
  SVC_START = 0,
  SVC_YIELD,
  SVC_DELAY,
  SVC_CREATE_TASK,
  SVC_MALLOC,
  SVC_FREE,
  SVC_MUTEX_INIT,
  SVC_MUTEX_LOCK,
  SVC_MUTEX_UNLOCK,
  SVC_SEM_INIT,
  SVC_SEM_WAIT,
  SVC_SEM_POST,
  SVC_MQ_CREATE,
  SVC_MQ_SEND,
  SVC_MQ_RECEIVE,
  SVC_GET_TIME
};

/* Dynamic Task Pool */
static TaskNode *task_pool = NULL;
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
MessageQueue *k_mq_create(uint8_t capacity);
void k_mq_send(MessageQueue *mq, void *msg);
void *k_mq_receive(MessageQueue *mq);
uint32_t k_get_time(void);
}

/* Inline helper */
static inline TaskNode *get_task_node(void) __attribute__((always_inline));
static inline TaskNode *get_task_node(void) {
  if (task_pool_index >= MAX_TASKS)
    return NULL;
  return &task_pool[task_pool_index++];
}

/* Heap Helpers from MISRA refactor */
static void heap_push(TaskNode *node) {
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

static TaskNode *heap_pop(void) {
  if (kernel.ready_heap.size == 0)
    return NULL;
  TaskNode *root = kernel.ready_heap.nodes[0];
  kernel.ready_heap.size--;
  if (kernel.ready_heap.size > 0) {
    TaskNode *last = kernel.ready_heap.nodes[kernel.ready_heap.size];
    uint8_t i = 0;
    while ((2 * i + 1) < kernel.ready_heap.size) {
      uint8_t left = 2 * i + 1;
      uint8_t right = 2 * i + 2;
      uint8_t largest = left;
      if (right < kernel.ready_heap.size &&
          kernel.ready_heap.nodes[right]->task.priority >
              kernel.ready_heap.nodes[left]->task.priority) {
        largest = right;
      }
      if (last->task.priority >=
          kernel.ready_heap.nodes[largest]->task.priority)
        break;
      kernel.ready_heap.nodes[i] = kernel.ready_heap.nodes[largest];
      i = largest;
    }
    kernel.ready_heap.nodes[i] = last;
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

  /* Alloc Task Pool */
  if (task_pool == NULL) {
    task_pool = (TaskNode *)k_malloc(sizeof(TaskNode) * MAX_TASKS);
  }
  task_pool_index = 0;

#if TOYOS_USE_MPU
  port_mpu_init();
#endif
}

void k_start(void) {
  if (kernel.task_count > 0) {
    port_timer_init();
    port_start_first_task((uint8_t *)kernel.current_task->stack_ptr);
  }
}

void k_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                   uint16_t stack_size) {
  TaskNode *new_node = get_task_node();
  if (!new_node)
    return;

  void *stack_mem = k_malloc(stack_size);
  if (!stack_mem)
    return;

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

  port_enter_critical();
  heap_push(new_node);
  kernel.task_count++;
  port_exit_critical();
}

void k_yield(void) { port_context_switch(); }

/* Optimized Scheduler (Hot Path) */
TOYOS_HOT void os_scheduler(void) {
  if (kernel.current_node && (kernel.current_task->state == TASK_RUNNING ||
                              kernel.current_task->state == TASK_READY)) {
    /* Fast Path */
    if (kernel.ready_heap.size > 0 &&
        kernel.current_task->priority >
            kernel.ready_heap.nodes[0]->task.priority) {
      return;
    }
    if (kernel.ready_heap.size == 0)
      return;

    kernel.current_task->state = TASK_READY;
    heap_push(kernel.current_node);
  }

  TaskNode *next_node = heap_pop();
  if (next_node) {
    next_node->task.state = TASK_RUNNING;
    os_current_task_ptr = &next_node->task;
    kernel.current_task = &next_node->task;
    kernel.current_node = next_node;
  }
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
      heap_push(waking);
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
  k_yield();
}

void *k_malloc(size_t size) {
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

  while (curr) {
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
      if (prev)
        prev->next = curr->next;
      else
        kernel.mem_manager.free_list_head = curr->next;

      port_exit_critical();
      return (void *)((uint8_t *)curr + sizeof(BlockHeader));
    }
    prev = curr;
    curr = curr->next;
  }
  port_exit_critical();
  return NULL;
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
  /* Restore Priority */
  if (kernel.current_task == m->owner) {
    kernel.current_task->priority = kernel.current_task->base_priority;
  }

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
MessageQueue *k_mq_create(uint8_t c) { return NULL; }
void k_mq_send(MessageQueue *q, void *m) {}
void *k_mq_receive(MessageQueue *q) { return NULL; }
uint32_t k_get_time(void) { return kernel.system_tick; }

/* --- SYSCALL DISPATCHER --- */

extern "C" uint32_t os_kernel_syscall(uint8_t id, uint32_t a0, uint32_t a1,
                                      uint32_t a2, uint32_t a3) {
  switch (id) {
  case SVC_YIELD:
    k_yield();
    return 0;
  case SVC_DELAY:
    k_delay((uint16_t)a0);
    return 0;
  case SVC_CREATE_TASK:
    k_create_task((uint8_t)a0, (void (*)(void))a1, (uint8_t)a2, (uint16_t)a3);
    return 0;
  case SVC_MALLOC:
    return (uint32_t)k_malloc((size_t)a0);
  case SVC_FREE:
    k_free((void *)a0);
    return 0;
  case SVC_MUTEX_INIT:
    k_mutex_init((Mutex *)a0);
    return 0;
  case SVC_MUTEX_LOCK:
    k_mutex_lock((Mutex *)a0);
    return 0;
  case SVC_MUTEX_UNLOCK:
    k_mutex_unlock((Mutex *)a0);
    return 0;
  case SVC_GET_TIME:
    return k_get_time();
  default:
    return 0;
  }
}

/* --- PUBLIC API WRAPPERS --- */

void os_init(uint8_t *pool, uint16_t size) { k_init(pool, size); }
void os_start(void) { k_start(); }

/* Wrapper Macro */
#if TOYOS_USE_SVC
#define SVC_0(id) __asm volatile("svc %0" : : "I"(id) : "memory")
#define SVC_1(id, a0)                                                          \
  register uint32_t r0 asm("r0") = (uint32_t)(a0);                             \
  __asm volatile("svc %1" : "+r"(r0) : "I"(id) : "memory");                    \
  return;
#define SVC_1_RET(id, a0, ret_type)                                            \
  register uint32_t r0 asm("r0") = (uint32_t)(a0);                             \
  __asm volatile("svc %1" : "+r"(r0) : "I"(id) : "memory");                    \
  return (ret_type)r0;
#else
#define SVC_0(id)              /* No-op */
#define SVC_1(id, a0)          /* No-op */
#define SVC_1_RET(id, a0, ret) /* No-op */
#endif

void os_yield(void) {
#if TOYOS_USE_SVC
  SVC_0(SVC_YIELD);
#else
  k_yield();
#endif
}

void os_delay(uint16_t ticks) {
#if TOYOS_USE_SVC
  SVC_1(SVC_DELAY, ticks);
#else
  k_delay(ticks);
#endif
}

void os_create_task(uint8_t id, void (*f)(void), uint8_t p, uint16_t s) {
#if TOYOS_USE_SVC
  register uint32_t r0 asm("r0") = id;
  register uint32_t r1 asm("r1") = (uint32_t)f;
  register uint32_t r2 asm("r2") = p;
  register uint32_t r3 asm("r3") = s;
  __asm volatile("svc %0"
                 :
                 : "I"(SVC_CREATE_TASK), "r"(r0), "r"(r1), "r"(r2), "r"(r3)
                 : "memory");
#else
  k_create_task(id, f, p, s);
#endif
}

void *os_malloc(uint16_t size) {
#if TOYOS_USE_SVC
  SVC_1_RET(SVC_MALLOC, size, void *);
#else
  return k_malloc(size);
#endif
}

void os_free(void *ptr) {
#if TOYOS_USE_SVC
  SVC_1(SVC_FREE, ptr);
#else
  k_free(ptr);
#endif
}

void os_mutex_init(Mutex *m) {
#if TOYOS_USE_SVC
  SVC_1(SVC_MUTEX_INIT, m);
#else
  k_mutex_init(m);
#endif
}

void os_mutex_lock(Mutex *m) {
#if TOYOS_USE_SVC
  SVC_1(SVC_MUTEX_LOCK, m);
#else
  k_mutex_lock(m);
#endif
}

void os_mutex_unlock(Mutex *m) {
#if TOYOS_USE_SVC
  SVC_1(SVC_MUTEX_UNLOCK, m);
#else
  k_mutex_unlock(m);
#endif
}

/* Stubs for others */
void os_sem_init(Semaphore *s, uint8_t c) { k_sem_init(s, c); }
void os_sem_wait(Semaphore *s) { k_sem_wait(s); }
void os_sem_post(Semaphore *s) { k_sem_post(s); }
MessageQueue *os_mq_create(uint8_t c) { return k_mq_create(c); }
void os_mq_send(MessageQueue *q, void *m) { k_mq_send(q, m); }
void *os_mq_receive(MessageQueue *q) { return k_mq_receive(q); }

/* --- UTILITY FUNCTIONS --- */

void os_enter_idle(void) {
#if defined(__arm__)
  __asm volatile("wfi");
#endif
}

void os_wdt_feed(void) { /* Call port WDT? Need k_wdt_feed? */ }

void os_check_stack_overflow(void) {
  /* Iterate tasks and check canaries */
  /* Implementation omitted for brevity but required for linker */
}

void os_print_info(void) {
#ifdef ARDUINO
  Serial.print(F("Tasks: "));
  Serial.println(kernel.task_count);
  Serial.print(F("Tick: "));
  Serial.println(kernel.system_tick);

  /* Print task states? Not protected!
     User task cannot access kernel.task_list.
     But os_print_info IS a wrapper?
     This accesses kernel.
     So os_print_info MUST BE A SYSCALL or Internal!
     Since Serial is used, easier to keep it internal or wrapper that calls
     internal k_print_info.
  */
  // k_print_info required if MPU active.
  // For now, assume privileged execution or MPU disabled for debugging.
#endif
}
