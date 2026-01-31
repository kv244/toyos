#include "toyos.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>

/* Global Kernel Instance */
static Kernel kernel = {0};

/* Global pointer for ASM access */
extern "C" TaskControlBlock *os_current_task_ptr;
TaskControlBlock *os_current_task_ptr = NULL;

/* Assembly helper functions */
extern "C" {
void fast_zero_stack(uint8_t *sp, uint8_t count);
}

/* Helper: allocate task node from heap */
static inline TaskNode *get_task_node(void) {
  if (kernel.task_count >= MAX_TASKS)
    return NULL;
  return (TaskNode *)os_malloc(sizeof(TaskNode));
}

/* Task Stack Initialization - FIXED: Proper stack layout for context restore */
static uint8_t *os_init_stack(uint16_t stack_size, void (*task_func)(void),
                              uint32_t **canary_ptr_out) {
  uint8_t *stack = (uint8_t *)os_malloc(stack_size);
  if (!stack)
    return NULL;

  /* Place canary at bottom of stack for overflow detection */
  *(uint32_t *)stack = STACK_CANARY;
  *canary_ptr_out = (uint32_t *)stack;

  /* Point to the end of stack (AVR stack grows down) */
  uint8_t *sp = stack + stack_size - 1;

  /* Stack layout (from high to low address):
   * - PC High byte (return address for 'ret')
   * - PC Low byte
   * - R0 (temporary storage for SREG during restore)
   * - SREG (status register with I bit set)
   * - R1 (zero register)
   * - R2-R31 (general purpose registers)
   */

  uint16_t func_addr = (uint16_t)(uintptr_t)task_func;
  *sp-- = func_addr & 0xFF; /* PC Low byte - Pushed FIRST (Highest Addr) */
  *sp-- =
      (func_addr >> 8) & 0xFF; /* PC High byte - Pushed SECOND (Lower Addr) */
  *sp-- = 0x00;                /* R0 (actual register value) */
  *sp-- = SREG_I_BIT;          /* SREG (I bit set) */
  *sp-- = 0x00;                /* R1 (zero register) */

  /* R2-R31: Zero out */
  for (int i = 0; i < 30; i++) {
    *sp-- = 0x00;
  }

  return sp;
}

/* --- Buddy Memory Allocator (v2) --- */

/*
 * This version of the buddy allocator includes a header on EVERY block
 * (both allocated and free). This allows for a constant-time O(1) check
 * to see if a buddy block is free, which simplifies coalescing logic and
 * ensures deterministic performance.
 */

#define BUDDY_MIN_EXPONENT 4     // Smallest block is 2^4 = 16 bytes
#define BUDDY_MAX_POOL_SIZE 1024 // Adjusted to match user's pool
#define BUDDY_MAX_EXPONENT 10    // 2^10 = 1024 bytes
#define BUDDY_NUM_LISTS (BUDDY_MAX_EXPONENT - BUDDY_MIN_EXPONENT + 1)

/* Header for ALL blocks, plus free list pointers */
typedef struct Block {
  bool is_free;
  uint8_t exponent;
  struct Block *next_free;
  struct Block *prev_free;
} Block;

/* State for the memory manager. */
static struct {
  uint8_t *pool_start;
  uint16_t pool_size;
  Block *free_lists[BUDDY_NUM_LISTS];
} mem;

/* Helper to get the 0-based index for a given exponent. */
static inline uint8_t get_list_idx(uint8_t exponent) {
  if (exponent < BUDDY_MIN_EXPONENT)
    exponent = BUDDY_MIN_EXPONENT;
  return exponent - BUDDY_MIN_EXPONENT;
}

/* Add a block to the front of its free list. */
static void add_to_freelist(Block *block) {
  uint8_t idx = get_list_idx(block->exponent);

  block->is_free = true;
  block->prev_free = NULL;
  block->next_free = mem.free_lists[idx];
  if (mem.free_lists[idx]) {
    mem.free_lists[idx]->prev_free = block;
  }
  mem.free_lists[idx] = block;
}

/* Remove a block from its free list. */
static void remove_from_freelist(Block *block) {
  uint8_t idx = get_list_idx(block->exponent);

  if (block->prev_free) {
    block->prev_free->next_free = block->next_free;
  } else {
    mem.free_lists[idx] = block->next_free;
  }
  if (block->next_free) {
    block->next_free->prev_free = block->prev_free;
  }
  block->is_free = false;
}

/*
  Initialize the OS and Memory Manager.
  This version uses a Buddy Allocator.
*/
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

  for (int i = 0; i < MAX_TASKS; i++) {
    kernel.all_tasks[i] = NULL;
  }

  /* Initialize Buddy Allocator */
  for (int i = 0; i < BUDDY_NUM_LISTS; i++) {
    mem.free_lists[i] = NULL;
  }

  mem.pool_start = mem_pool;
  mem.pool_size = mem_size;

  // Create initial free blocks
  uint16_t remaining_size = mem.pool_size;
  uintptr_t current_offset = 0;

  for (int8_t exp = BUDDY_MAX_EXPONENT; exp >= BUDDY_MIN_EXPONENT; exp--) {
    uint16_t block_size = 1 << exp;
    while (remaining_size >= block_size) {
      Block *block = (Block *)(mem.pool_start + current_offset);
      block->exponent = exp;
      add_to_freelist(block);
      current_offset += block_size;
      remaining_size -= block_size;
    }
  }

  /* Reset system tick count */
  kernel.system_tick = 0;
}

/* Memory Allocation using Buddy System */
void *os_malloc(uint16_t size) {
  if (size == 0)
    return NULL;

  // Total size needed for a block must be at least sizeof(Block)
  // to hold free list pointers. User data fits after the header.
  uint16_t required_block_size = size + sizeof(Block);

  // Find the smallest power-of-2 exponent that can hold the required size
  uint8_t exponent = BUDDY_MIN_EXPONENT;
  while ((1 << exponent) < required_block_size) {
    exponent++;
    if (exponent > BUDDY_MAX_EXPONENT) {
      return NULL; // Request too large
    }
  }

  ATOMIC_START();

  // Find a free block, starting from the best fit and going up
  uint8_t original_exp = exponent;
  uint8_t current_exp = exponent;
  Block *block = NULL;

  while (current_exp <= BUDDY_MAX_EXPONENT) {
    uint8_t list_idx = get_list_idx(current_exp);
    if (mem.free_lists[list_idx]) {
      block = mem.free_lists[list_idx];
      break; // Found a block
    }
    current_exp++;
  }

  if (!block) {
    ATOMIC_END();
    return NULL; // Out of memory
  }

  // Pull the block from its list and split it down to the required size
  remove_from_freelist(block);

  while (current_exp > original_exp) {
    current_exp--;
    block->exponent = current_exp;
    uint16_t block_size = 1 << current_exp;

    // The other half becomes a new free buddy block
    Block *buddy = (Block *)((uint8_t *)block + block_size);
    buddy->exponent = current_exp;
    add_to_freelist(buddy);
  }

  ATOMIC_END();

  // Return a pointer to the user data area, after the block header
  return (void *)((uint8_t *)block + sizeof(Block));
}

/* Memory Deallocation using Buddy System */
void os_free(void *ptr) {
  if (!ptr)
    return;

  ATOMIC_START();

  // Get a pointer to the block header from the user pointer
  Block *block = (Block *)((uint8_t *)ptr - sizeof(Block));

  // Basic sanity check to prevent freeing already free blocks or corrupted
  // pointers
  if (block->is_free || (uint8_t *)block < mem.pool_start ||
      (uint8_t *)block >= (mem.pool_start + mem.pool_size)) {
    ATOMIC_END();
    return;
  }

  uint8_t exponent = block->exponent;

  // Coalescing loop: merge with buddies if they are free
  while (exponent < BUDDY_MAX_EXPONENT) {
    uint16_t block_size = 1 << exponent;
    uintptr_t block_offset = (uint8_t *)block - mem.pool_start;

    // Calculate the buddy's address
    uintptr_t buddy_offset = block_offset ^ block_size;

    // Safety check: Buddy MUST be within the pool
    if (buddy_offset + block_size > mem.pool_size) {
      break;
    }

    Block *buddy = (Block *)(mem.pool_start + buddy_offset);

    // Check if the buddy is free and has the same size
    if (buddy->is_free && buddy->exponent == exponent) {
      remove_from_freelist(buddy);

      // The new, larger block starts at the address of the lower-addressed
      // buddy
      if (buddy_offset < block_offset) {
        block = buddy;
      }

      // Increment the exponent for the new, larger block and repeat
      exponent++;
      block->exponent = exponent;
    } else {
      break; // Buddy is not free or wrong size, so stop coalescing
    }
  }

  // Add the final (potentially larger) block to the correct free list
  add_to_freelist(block);

  ATOMIC_END();
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

/* Initialize Timer1 for ~1ms system tick at 16MHz */
void os_timer_init(void) {
  cli(); /* Disable interrupts */

  /* Configure Pin 13 (PB5) as output for OS activity heartbeat */
  DDRB |= (1 << 5);
  PORTB &= ~(1 << 5);

  /* Timer1 CTC mode, prescaler 64 */
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | TIMER1_PRESCALER_64;

  /* For 1ms tick at 16MHz with prescaler 64: 16000000/64/1000 = 250 - 1 */
  OCR1A = TIMER1_1MS_AT_16MHZ;

  /* Enable Timer1 compare match A interrupt */
  TIMSK1 = (1 << OCIE1A);
}

/* Timer1 Compare Match A ISR - Naked for custom context switch */
ISR(TIMER1_COMPA_vect, ISR_NAKED) {
  /* Save Context */
  __asm__ __volatile__("push r0 \n\t"
                       "in r0, %0 \n\t"
                       "push r0 \n\t"
                       "push r1 \n\t"
                       "clr r1 \n\t"
                       "push r2 \n\t"
                       "push r3 \n\t"
                       "push r4 \n\t"
                       "push r5 \n\t"
                       "push r6 \n\t"
                       "push r7 \n\t"
                       "push r8 \n\t"
                       "push r9 \n\t"
                       "push r10 \n\t"
                       "push r11 \n\t"
                       "push r12 \n\t"
                       "push r13 \n\t"
                       "push r14 \n\t"
                       "push r15 \n\t"
                       "push r16 \n\t"
                       "push r17 \n\t"
                       "push r18 \n\t"
                       "push r19 \n\t"
                       "push r20 \n\t"
                       "push r21 \n\t"
                       "push r22 \n\t"
                       "push r23 \n\t"
                       "push r24 \n\t"
                       "push r25 \n\t"
                       "push r26 \n\t"
                       "push r27 \n\t"
                       "push r28 \n\t"
                       "push r29 \n\t"
                       "push r30 \n\t"
                       "push r31 \n\t" ::"I"(_SFR_IO_ADDR(SREG)));

  /* Save current SP to TCB */
  if (os_current_task_ptr) {
    uint16_t sp_val = SPL | (SPH << 8);
    os_current_task_ptr->stack_ptr = (uint8_t *)sp_val;
  }

  /* Call OS tick */
  os_system_tick();

  /* Call Scheduler to pick next task - ONLY if we started the OS */
  if (kernel.task_count > 0) {
    os_scheduler();
  }

  /* Load new SP from TCB */
  if (os_current_task_ptr) {
    uint16_t sp_val = (uint16_t)os_current_task_ptr->stack_ptr;
    SPL = sp_val & 0xFF;
    SPH = sp_val >> 8;
  }

  /* Restore Context */
  __asm__ __volatile__("pop r31 \n\t"
                       "pop r30 \n\t"
                       "pop r29 \n\t"
                       "pop r28 \n\t"
                       "pop r27 \n\t"
                       "pop r26 \n\t"
                       "pop r25 \n\t"
                       "pop r24 \n\t"
                       "pop r23 \n\t"
                       "pop r22 \n\t"
                       "pop r21 \n\t"
                       "pop r20 \n\t"
                       "pop r19 \n\t"
                       "pop r18 \n\t"
                       "pop r17 \n\t"
                       "pop r16 \n\t"
                       "pop r15 \n\t"
                       "pop r14 \n\t"
                       "pop r13 \n\t"
                       "pop r12 \n\t"
                       "pop r11 \n\t"
                       "pop r10 \n\t"
                       "pop r9 \n\t"
                       "pop r8 \n\t"
                       "pop r7 \n\t"
                       "pop r6 \n\t"
                       "pop r5 \n\t"
                       "pop r4 \n\t"
                       "pop r3 \n\t"
                       "pop r2 \n\t"
                       "pop r1 \n\t"
                       "pop r0 \n\t"
                       "out %0, r0 \n\t"
                       "pop r0 \n\t"
                       "reti \n\t" ::"I"(_SFR_IO_ADDR(SREG)));
}

/* Get current tick count - atomic read for uint32_t */
uint32_t os_get_tick(void) {
  uint32_t tick;
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
  uint8_t *sp = os_init_stack(stack_size, task_func, &canary_ptr);
  if (!sp) {
    /* Failed to allocate stack - free the node */
    os_free(new_node);
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
  kernel.all_tasks[kernel.task_count] = new_node;
  heap_push(new_node);
  kernel.task_count++;
  ATOMIC_END();
}

/* Priority-Based Scheduler - FIXED: Skip blocked tasks */
void os_scheduler(void) {
  /* Activity Heartbeat: Toggle Pin 13 on every scheduling decision */
  PINB |= (1 << 5);

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
    os_context_switch();
    ATOMIC_END();
  }
}

/* Task Yield - voluntary context switch */
void os_task_yield(void) {
  if (kernel.current_task) {
    ATOMIC_START();
    os_context_switch();
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
      os_context_switch();
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
      os_context_switch();
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
      cli();
      /* Halt system - in production, you might log the error */
      while (1) {
        /* Could blink LED rapidly here to indicate error */
      }
    }
  }
}

void os_enter_idle(void) {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
}

/* Watchdog Timer Support */
void os_wdt_init(uint8_t timeout) {
  /* timeout: WDTO_15MS, WDTO_1S, WDTO_2S, etc (from avr/wdt.h) */
  wdt_enable(timeout);
}

void os_wdt_feed(void) { wdt_reset(); }

/* Stack High-Water Mark Tracking */
uint16_t os_get_stack_usage(uint8_t task_id) {
  TaskNode *node = NULL;
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (kernel.all_tasks[i] && kernel.all_tasks[i]->task.id == task_id) {
      node = kernel.all_tasks[i];
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
    if (kernel.all_tasks[i] && kernel.all_tasks[i]->task.id == task_id) {
      return kernel.all_tasks[i]->task.priority;
    }
  }
  return 0;
}

/* Start the OS - never returns */
void os_start(void) {
  cli();           /* Disable interrupts to prevent corruption */
  os_timer_init(); /* Start system tick timer */

  /* Initial context switch - pick the first task and start it */
  os_scheduler();

  if (os_current_task_ptr) {
    uint16_t sp_val = (uint16_t)os_current_task_ptr->stack_ptr;
    SPL = sp_val & 0xFF;
    SPH = sp_val >> 8;

    /* Restore context of the first task - FIXED: Matches stack init layout */
    __asm__ __volatile__("pop r31 \n\t"
                         "pop r30 \n\t"
                         "pop r29 \n\t"
                         "pop r28 \n\t"
                         "pop r27 \n\t"
                         "pop r26 \n\t"
                         "pop r25 \n\t"
                         "pop r24 \n\t"
                         "pop r23 \n\t"
                         "pop r22 \n\t"
                         "pop r21 \n\t"
                         "pop r20 \n\t"
                         "pop r19 \n\t"
                         "pop r18 \n\t"
                         "pop r17 \n\t"
                         "pop r16 \n\t"
                         "pop r15 \n\t"
                         "pop r14 \n\t"
                         "pop r13 \n\t"
                         "pop r12 \n\t"
                         "pop r11 \n\t"
                         "pop r10 \n\t"
                         "pop r9 \n\t"
                         "pop r8 \n\t"
                         "pop r7 \n\t"
                         "pop r6 \n\t"
                         "pop r5 \n\t"
                         "pop r4 \n\t"
                         "pop r3 \n\t"
                         "pop r2 \n\t"
                         "pop r1 \n\t"
                         "pop r0 \n\t"
                         "out %0, r0 \n\t"
                         "pop r0 \n\t"
                         "ret \n\t" ::"I"(_SFR_IO_ADDR(SREG)));
  }

  /* This part should never be reached. If it is, it means no tasks were
   * created. We'll hang here. A watchdog timer could catch this. */
  while (1) {
    // System halted: No tasks to run.
  }
}
