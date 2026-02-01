/**
 * ToyOS - Tiny Operating System for Multi-Platform
 *
 * A preemptive, priority-based Real-Time Operating System (RTOS) with
 * portability layer for multiple architectures.
 *
 * Features:
 * - Priority-based preemptive multitasking
 * - Binary heap scheduler for O(log n) task selection
 * - Delta queue for efficient sleep/delay operations
 * - Semaphores and mutexes for synchronization
 * - Message queues for inter-task communication
 * - Stack overflow detection with canaries
 * - Optimized context switching in assembly
 * - Multiplatform support (AVR, ARM Cortex-M planned)
 *
 * Author: [Your Name]
 * Version: 2.5.0
 * Date: January 2026
 */

#ifndef TOYOS_H
#define TOYOS_H

/* ToyOS Version */
#define TOYOS_VERSION_MAJOR 2
#define TOYOS_VERSION_MINOR 5
#define TOYOS_VERSION_PATCH 2
#define TOYOS_VERSION_STRING "2.5.2"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * USER CONFIGURATION
 * ======================================================================== */

/**
 * Include user configuration if TOYOS_USER_CONFIG is defined.
 * Users can create their own toyos_user_config.h to override defaults.
 */
#ifdef TOYOS_USER_CONFIG
#include "toyos_user_config.h"
#endif

/**
 * Include default configuration.
 * This provides #ifndef guards so user overrides take precedence.
 */
#include "toyos_config.h"

/* ========================================================================
 * CONFIGURATION CONSTANTS (from toyos_config.h)
 * ======================================================================== */

/**
 * Maximum number of tasks that can be created in the system.
 * Configurable via TOYOS_MAX_TASKS in toyos_config.h
 */
#define MAX_TASKS TOYOS_MAX_TASKS

/**
 * Default stack size for tasks (in bytes).
 * Configurable via TOYOS_DEFAULT_STACK_SIZE in toyos_config.h
 */
#define DEFAULT_STACK_SIZE TOYOS_DEFAULT_STACK_SIZE

/**
 * Minimum stack size allowed (safety check).
 * Configurable via TOYOS_MIN_STACK_SIZE in toyos_config.h
 */
#define MIN_STACK_SIZE TOYOS_MIN_STACK_SIZE

/* ========================================================================
 * STACK OVERFLOW DETECTION
 * ======================================================================== */

/**
 * Stack canary value for overflow detection.
 * Configurable via TOYOS_STACK_CANARY in toyos_config.h
 */
#define STACK_CANARY TOYOS_STACK_CANARY

/* ========================================================================
 * HARDWARE-SPECIFIC CONSTANTS (ATmega328P @ 16MHz)
 * ======================================================================== */

/**
 * Timer1 prescaler configuration.
 * Calculated from TOYOS_TIMER_PRESCALER in toyos_config.h
 */
#define TIMER1_PRESCALER_64 TOYOS_TIMER_PRESCALER_BITS

/**
 * Timer1 compare value for system tick.
 * Calculated from TOYOS_TICK_RATE_HZ and TOYOS_TIMER_PRESCALER
 */
#define TIMER1_1MS_AT_16MHZ ((uint16_t)TOYOS_TIMER_COMPARE_VALUE)

/**
 * System tick frequency in Hz.
 * Configurable via TOYOS_TICK_RATE_HZ in toyos_config.h
 */
#define OS_TICK_RATE_HZ TOYOS_TICK_RATE_HZ

/**
 * Convert milliseconds to ticks.
 * Usage: os_delay(MS_TO_TICKS(500)); // 500ms delay
 */
#define MS_TO_TICKS(ms) TOYOS_MS_TO_TICKS(ms)

/**
 * Convert seconds to ticks.
 * Usage: os_delay(SEC_TO_TICKS(2)); // 2 second delay
 */
#define SEC_TO_TICKS(sec) TOYOS_SEC_TO_TICKS(sec)

/* ========================================================================
 * TASK CONTROL BLOCK STRUCTURE OFFSETS
 * ======================================================================== */

/**
 * Offset of stack_ptr field in TaskControlBlock structure.
 * Used by assembly code for efficient access.
 *
 * CRITICAL: Must be updated if TCB structure changes!
 *
 * Current TCB layout:
 *   Offset 0: uint8_t *stack_ptr       (2 bytes) <- TARGET
 *   Offset 2: uint8_t id               (1 byte)
 *   Offset 3: TaskState state          (1 byte)
 *   Offset 4: void (*task_func)(void)  (2 bytes)
 *   Offset 6: uint16_t stack_size      (2 bytes)
 *   Offset 8: uint8_t priority         (1 byte)
 *   Offset 9: uint16_t delta_ticks     (2 bytes)
 */
#define TCB_STACK_PTR_OFFSET 0

/* ========================================================================
 * TASK STATES
 * ======================================================================== */

/**
 * Task state enumeration.
 *
 * State transitions:
 *   READY -> RUNNING  (selected by scheduler)
 *   RUNNING -> READY  (preempted by higher priority task)
 *   RUNNING -> BLOCKED (waiting on semaphore/mutex/delay)
 *   BLOCKED -> READY  (resource available/delay expired)
 *
 * State diagram:
 *
 *     ┌─────────┐
 *     │  READY  │◄───────────────┐
 *     └────┬────┘                │
 *          │                     │
 *          │ scheduler           │ unblock/
 *          │ dispatch            │ delay expires
 *          ▼                     │
 *     ┌─────────┐                │
 *     │ RUNNING │                │
 *     └────┬────┘                │
 *          │                     │
 *          │ wait/delay/         │
 *          │ preempt             │
 *          ▼                     │
 *     ┌─────────┐                │
 *     │ BLOCKED │────────────────┘
 *     └─────────┘
 */
typedef enum {
  TASK_READY,   /**< Task is ready to run */
  TASK_RUNNING, /**< Task is currently executing */
  TASK_BLOCKED  /**< Task is waiting (semaphore/mutex/delay) */
} TaskState;

/* ========================================================================
 * TASK CONTROL BLOCK (TCB)
 * ======================================================================== */

/**
 * Task Control Block - Contains all state for a single task.
 *
 * Each task in the system has one TCB that stores:
 * - Identification and state
 * - Execution context (stack pointer)
 * - Scheduling information (priority)
 * - Blocking information (delta ticks for delays)
 * - Stack overflow detection (canary pointer)
 *
 * Memory layout must be stable for assembly code access!
 * See TCB_STACK_PTR_OFFSET for assembly dependencies.
 */
typedef struct {
  /**
   * Current stack pointer for this task.
   * Saved during context switch, restored when task resumes.
   * Points to top of saved context on stack.
   *
   * CRITICAL: Offset must match TCB_STACK_PTR_OFFSET!
   * MOVED TO TOP for Optimization.
   */
  uint8_t *stack_ptr;

  /** Unique task identifier (user-defined) */
  uint8_t id;

  /** Current task state (READY/RUNNING/BLOCKED) */
  TaskState state;

  /** Pointer to task's entry function */
  void (*task_func)(void);

  /** Total stack size allocated for this task (bytes) */
  uint16_t stack_size;

  /** Task priority (0-255, higher number = higher priority) */
  uint8_t priority;

  /** Original priority (used for restoring after Priority Inheritance) */
  uint8_t base_priority;

  /**
   * Delta ticks for blocked queue.
   * When task is blocked with delay, stores remaining time
   * in differential encoding for O(1) tick processing.
   */
  uint16_t delta_ticks;

  /**
   * Pointer to stack canary (at bottom of stack).
   * Used for overflow detection - if canary value changes,
   * stack has been corrupted.
   */
  uint32_t *canary_ptr;
} TaskControlBlock;

/* ========================================================================
 * ASSEMBLY FUNCTIONS (Implemented in os_switch.S)
 * ======================================================================== */

/**
 * Context switch routine.
 * Saves current task's CPU context, calls scheduler, restores new task's
 * context.
 *
 * Called from:
 * - Timer ISR (preemptive scheduling)
 * - os_delay() (voluntary blocking)
 * - os_task_yield() (voluntary yield)
 * - os_sem_wait() / os_mutex_lock() (blocking on resources)
 *
 * Context saved (35 bytes total):
 * - R0-R31 (32 general purpose registers)
 * - SREG (status register)
 * - PC (program counter, saved automatically by call/interrupt)
 *
 * @note This function never returns to the same task!
 *       It always returns to the task selected by scheduler.
 */
void os_context_switch(void);

/* ========================================================================
 * TASK QUEUE DATA STRUCTURES
 * ======================================================================== */

/**
 * Task Queue Node - Linked list node containing a task.
 *
 * Used for:
 * - Blocked queues (tasks waiting on semaphores/mutexes/delays)
 * - Message queue blocked task lists
 *
 * The ready queue uses a binary heap instead for O(log n) operations.
 */
typedef struct TaskNode {
  TaskControlBlock task; /**< Embedded task control block */
  struct TaskNode *next; /**< Pointer to next node in queue */
} TaskNode;

/**
 * Simple FIFO Queue for tasks.
 *
 * Used for blocking queues where insertion order matters
 * (semaphores/mutexes follow FIFO fairness).
 *
 * Operations:
 * - Enqueue: O(1) - add to tail
 * - Dequeue: O(1) - remove from head
 * - Count: O(1) - tracked explicitly
 */
typedef struct {
  TaskNode *head; /**< First task in queue (next to be removed) */
  TaskNode *tail; /**< Last task in queue (where new tasks are added) */
  uint8_t count;  /**< Number of tasks in queue */
} TaskQueue;

/* ========================================================================
 * MEMORY MANAGEMENT
 * ======================================================================== */

/**
 * First-Fit Free List allocator for memory management.
 *
 * Memory is allocated from a fixed pool using a linked list of free blocks.
 * Supports deallocation via os_free() with coalescing of adjacent free blocks.
 *
 * Suitable for:
 * - Dynamic task and queue creation/destruction
 * - Temporary buffers and variable-lifetime objects
 * - System data structures
 *
 * Design:
 * - First-fit search strategy
 * - Alignment-aware (2-byte boundary)
 * - Coalescing to mitigate external fragmentation
 */
typedef struct {
  void *free_list_head; /**< Pointer to the first block in the free list */
} MemoryManager;

/* ========================================================================
 * PRIORITY QUEUE (Binary Heap)
 * ======================================================================== */

/**
 * Binary Max-Heap for priority-based ready queue.
 *
 * Properties:
 * - Parent priority >= child priority (max-heap property)
 * - Complete binary tree (filled level-by-level, left-to-right)
 * - Array-based representation for cache efficiency
 *
 * Operations:
 * - Insert (push): O(log n) - bubble up
 * - Extract max (pop): O(log n) - bubble down
 * - Peek: O(1)
 *
 * Array indexing:
 * - Parent of node i: (i-1)/2
 * - Left child of node i: 2*i + 1
 * - Right child of node i: 2*i + 2
 *
 * Example (priorities shown):
 *           10
 *          /  \
 *         8    6
 *        / \   /
 *       3  5  4
 *
 * Array: [10, 8, 6, 3, 5, 4]
 */
typedef struct {
  TaskNode *nodes[MAX_TASKS]; /**< Array of task node pointers */
  uint8_t size;               /**< Current number of elements in heap */
} BinaryHeap;

/* ========================================================================
 * SYNCHRONIZATION PRIMITIVES
 * ======================================================================== */

/**
 * Counting Semaphore for resource management.
 *
 * Use cases:
 * - Resource pools (e.g., buffer slots, connections)
 * - Signaling between tasks
 * - Rate limiting
 *
 * Operations:
 * - wait(): Decrement count, block if zero
 * - post(): Increment count, wake one waiting task
 *
 * Example - Producer/Consumer:
 *   Semaphore empty_slots;  // Count of empty buffer slots
 *   Semaphore full_slots;   // Count of full buffer slots
 *
 *   Producer:              Consumer:
 *     wait(empty_slots);     wait(full_slots);
 *     produce_item();        consume_item();
 *     post(full_slots);      post(empty_slots);
 */
typedef struct {
  volatile uint8_t count;  /**< Current semaphore count */
  TaskQueue blocked_tasks; /**< FIFO queue of blocked tasks */
} Semaphore;

/**
 * Binary Mutex for mutual exclusion.
 *
 * Properties:
 * - Only one task can hold mutex at a time
 * - Ownership tracking (for future priority inheritance)
 * - FIFO fairness for blocked tasks
 *
 * Use cases:
 * - Protecting shared data structures
 * - Serializing access to hardware resources
 * - Critical sections
 *
 * Example:
 *   Mutex uart_mutex;
 *
 *   Task A:                Task B:
 *     lock(uart_mutex);      lock(uart_mutex);
 *     Serial.print("A");     Serial.print("B");
 *     unlock(uart_mutex);    unlock(uart_mutex);
 *
 * Note: Currently does NOT implement priority inheritance.
 *       High priority tasks can be blocked by low priority tasks
 *       holding the mutex (priority inversion).
 */
typedef struct {
  volatile uint8_t locked; /**< 0 = unlocked, 1 = locked */
  TaskControlBlock *owner; /**< Task currently holding mutex */
  TaskQueue blocked_tasks; /**< FIFO queue of waiting tasks */
} Mutex;

/* ========================================================================
 * INTER-TASK COMMUNICATION
 * ======================================================================== */

/**
 * Fixed-size circular buffer message queue.
 *
 * Thread-safe message passing between tasks using semaphores and mutex.
 *
 * Properties:
 * - FIFO ordering
 * - Fixed capacity (set at creation)
 * - Blocking send when full
 * - Blocking receive when empty
 *
 * Internals:
 * - sem_read: Count of messages available to read (starts at 0)
 * - sem_write: Count of slots available to write (starts at capacity)
 * - mutex: Protects buffer access from concurrent modification
 *
 * Operations:
 * - send(): wait(sem_write) -> lock(mutex) -> write -> unlock(mutex) ->
 * post(sem_read)
 * - receive(): wait(sem_read) -> lock(mutex) -> read -> unlock(mutex) ->
 * post(sem_write)
 *
 * Fast path optimization:
 * - send_fast()/receive_fast() check availability without blocking
 * - Reduces context switches by ~75% when queue not full/empty
 */
typedef struct {
  void **buffer;       /**< Circular buffer of message pointers */
  uint8_t capacity;    /**< Maximum number of messages */
  uint8_t head;        /**< Index of next message to read */
  uint8_t tail;        /**< Index where next message will be written */
  uint8_t count;       /**< Current number of messages in queue */
  Semaphore sem_read;  /**< Semaphore for messages available */
  Semaphore sem_write; /**< Semaphore for slots available */
  Mutex mutex;         /**< Mutex for buffer protection */
} MessageQueue;

/* ========================================================================
 * KERNEL STATE
 * ======================================================================== */

/**
 * Global kernel state structure.
 *
 * Single instance holds all OS state:
 * - Ready queue (priority heap)
 * - Blocked queue (delta queue for delays)
 * - Current running task
 * - Memory manager
 * - System tick counter
 *
 * This structure is accessed from:
 * - C code (direct access)
 * - ISR (timer interrupt)
 * - Assembly code (via os_current_task_ptr)
 */
typedef struct {
  BinaryHeap ready_heap;          /**< Priority queue of ready tasks */
  TaskQueue blocked_queue;        /**< Delta queue of delayed tasks */
  TaskControlBlock *current_task; /**< Currently running task */
  TaskNode *current_node;         /**< Node pointer for current task */
  MemoryManager mem_manager;      /**< Bump allocator state */
  volatile uint32_t system_tick;  /**< Milliseconds since os_start() */
  uint8_t task_count;             /**< Total number of tasks created */
} Kernel;

/* ========================================================================
 * DEBUGGING AND ASSERTIONS
 * ======================================================================== */

/**
 * Runtime assertion for debugging.
 *
 * In DEBUG builds, checks condition and halts system if false.
 * In RELEASE builds, compiles to nothing (zero overhead).
 *
 * Usage:
 *   ASSERT(ptr != NULL);
 *   ASSERT(count <= MAX_TASKS);
 *
 * To enable:
 *   Add -DDEBUG to compiler flags
 *
 * On assertion failure:
 *   - Disables interrupts
 *   - Halts in infinite loop
 *   - Could be extended to log error or blink LED
 */
#ifdef DEBUG
#define ASSERT(condition)                                                      \
  do {                                                                         \
    if (!(condition)) {                                                        \
      cli();                                                                   \
      while (1) { /* Halt on assertion failure */                              \
      }                                                                        \
    }                                                                          \
  } while (0)
#else
#define ASSERT(condition) ((void)0)
#endif

/**
 * Compile-time assertion.
 *
 * Checks condition at compile time using array size trick.
 * If condition is false, array size is negative = compiler error.
 *
 * Usage:
 *   STATIC_ASSERT(sizeof(TaskControlBlock) == 15);
 *   STATIC_ASSERT(MAX_TASKS <= 255);
 */
#define STATIC_ASSERT(condition)                                               \
  typedef char static_assertion_failed[(condition) ? 1 : -1]

/* Verify critical assumptions at compile time */
STATIC_ASSERT(TCB_STACK_PTR_OFFSET == 0);
STATIC_ASSERT(MAX_TASKS > 0 && MAX_TASKS <= 32);
STATIC_ASSERT(MIN_STACK_SIZE >= 48);
STATIC_ASSERT(DEFAULT_STACK_SIZE >= MIN_STACK_SIZE);
// STATIC_ASSERT(TIMER1_1MS_AT_16MHZ > 0 && TIMER1_1MS_AT_16MHZ < 65535);

/* ========================================================================
 * OPTIMIZED ATOMIC OPERATIONS
 * ======================================================================== */

/**
 * Begin atomic section (disable interrupts).
 *
 * Saves current SREG (including interrupt flag) and disables interrupts.
 * Must be paired with ATOMIC_END() to restore interrupt state.
 *
 * Optimized version using inline assembly:
 * - Saves ~4-6 CPU cycles vs function call
 * - More efficient than ATOMIC_BLOCK macro
 *
 * Example:
 *   ATOMIC_START();
 *   critical_variable++;
 *   ATOMIC_END();
 *
 * @warning Never return or jump out of atomic section without ATOMIC_END()!
 * @note Atomic sections should be as short as possible (<50 instructions)
 */
#define ATOMIC_START() port_enter_critical()

/**
 * End atomic section (restore interrupts).
 *
 * Restores SREG saved by ATOMIC_START(), including interrupt flag.
 * If interrupts were enabled before ATOMIC_START(), they will be re-enabled.
 * If interrupts were already disabled, they remain disabled.
 *
 * @note Must be called exactly once for each ATOMIC_START()
 */
#define ATOMIC_END() port_exit_critical()

/* ========================================================================
 * PUBLIC API - CORE FUNCTIONS
 * ======================================================================== */

/**
 * Initialize the operating system.
 *
 * Must be called once before creating tasks or starting scheduler.
 *
 * @param mem_pool Pointer to memory pool for dynamic allocation
 * @param mem_size Size of memory pool in bytes
 *
 * The memory pool is used for:
 * - Task stack allocation
 * - Message queue buffer allocation
 * - Other system data structures
 *
 * Recommended size:
 * - At least (MAX_TASKS * DEFAULT_STACK_SIZE) for stacks
 * - Plus extra for message queues and system structures
 * - 1024 bytes is typical for 8 tasks with 128-byte stacks
 *
 * Example:
 *   static uint8_t mem_pool[1024];
 *   os_init(mem_pool, sizeof(mem_pool));
 */
void os_init(uint8_t *mem_pool, uint16_t mem_size);

/**
 * Create a new task.
 *
 * Allocates stack, initializes TCB, and adds task to ready queue.
 * Task will begin executing when scheduler selects it.
 *
 * @param id User-defined task ID (for identification/debugging)
 * @param task_func Pointer to task entry function
 * @param priority Task priority (0-255, higher = more important)
 * @param stack_size Stack size in bytes (minimum MIN_STACK_SIZE)
 *
 * Task function signature:
 *   void my_task(void) {
 *     while(1) {
 *       // Task code here
 *       os_delay(100); // Usually includes delay/blocking
 *     }
 *   }
 *
 * Priority guidelines:
 * - 0-3: Background tasks (logging, LED blinking)
 * - 4-7: Normal tasks (most application tasks)
 * - 8-15: High priority (sensor reading, control loops)
 * - 16+: Critical (communication protocols, safety)
 *
 * @warning Do not create tasks after os_start() is called!
 *
 * Example:
 *   os_create_task(1, led_task, 5, 96);
 *   os_create_task(2, sensor_task, 10, 128);
 */
void os_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                    uint16_t stack_size);

/**
 * Start the operating system and scheduler.
 *
 * This function never returns!
 * - Initializes system timer (1ms tick)
 * - Performs initial context switch to highest priority task
 * - Enters infinite loop with idle mode
 *
 * Call after:
 * - os_init()
 * - All os_create_task() calls
 * - All semaphore/mutex initializations
 *
 * Example:
 *   void setup() {
 *     os_init(mem_pool, sizeof(mem_pool));
 *     os_create_task(1, task1, 5, 128);
 *     os_create_task(2, task2, 10, 128);
 *     os_start(); // Never returns!
 *   }
 */
/**
 * Start the OS - This function never returns.
 * The scheduler will pick the highest priority task and switch context to it.
 */
void os_start(void);

/**
 * Print detailed system information (Version, Platform, Memory).
 * This function uses Serial port and blocks until output is done.
 * Best called during setup() before os_start().
 */
void os_print_info(void);

/**
 * Task scheduler - selects next task to run.
 *
 * Called from:
 * - Timer ISR (preemptive scheduling every 1ms)
 * - os_context_switch() (after task blocks/yields)
 * - os_start() (initial task selection)
 *
 * Algorithm:
 * 1. Put current task back in ready heap if still ready
 * 2. Pop tasks from heap until ready task found (skip blocked)
 * 3. Set new task as current and mark RUNNING
 *
 * @note This function is called from interrupt context - keep it fast!
 * @note Do not call directly - use os_task_yield() instead
 */
void os_scheduler(void);

/**
 * Delay current task for specified ticks.
 *
 * Blocks current task and yields CPU to other tasks.
 * Task is unblocked after specified number of system ticks.
 *
 * @param ticks Number of milliseconds to delay (0 = no delay)
 *
 * Resolution: 1ms (system tick period)
 * Accuracy: ±1ms (depends on other task load)
 *
 * Uses delta queue for O(1) tick processing:
 * - Only first blocked task is decremented each tick
 * - Tasks stored in differential encoding
 *
 * Example:
 *   os_delay(100);           // Delay 100ms
 *   os_delay(MS_TO_TICKS(500)); // Delay 500ms
 *   os_delay(SEC_TO_TICKS(1));  // Delay 1 second
 */
void os_delay(uint16_t ticks);

/**
 * Allocate memory from system pool.
 *
 * Implements a First-Fit Free List Allocator.
 * - Searches for the first free block large enough to satisfy the request.
 * - Splits larger blocks to minimize internal fragmentation.
 *
 * @param size Number of bytes to allocate
 * @return Pointer to allocated memory, or NULL if out of memory
 *
 * Alignment: Automatically aligned to 2-byte boundary
 * Overhead: sizeof(BlockHeader) per allocation
 *
 * Use cases:
 * - Dynamic data structures
 * - Temporary buffers
 * - Task stacks and queues
 *
 * @note Replaces the legacy bump allocator.
 */
void *os_malloc(uint16_t size);

/**
 * Free previously allocated memory.
 *
 * Returns the memory block to the free list and coalesces adjacent
 * free blocks to reduce external fragmentation.
 *
 * @param ptr Pointer to memory returned by os_malloc()
 *
 * @note Safe to call with NULL.
 * @note Complexity: O(n) where n is number of free blocks
 */
void os_free(void *ptr);

/**
 * Voluntarily yield CPU to other tasks.
 *
 * Current task is put back in ready queue and scheduler runs.
 * Useful for cooperative multitasking within priority level.
 *
 * Difference from os_delay():
 * - yield: Task remains ready, may run again immediately
 * - delay: Task blocked for specified time
 *
 * Example - Polling loop:
 *   while (!ready) {
 *     os_task_yield(); // Let other tasks run while waiting
 *   }
 */
void os_task_yield(void);

/**
 * System tick handler - called from timer ISR.
 *
 * Increments system tick counter and processes delta queue.
 * Unblocks tasks whose delay has expired.
 *
 * Complexity: O(1) due to delta queue encoding
 * - Only first task's delta is decremented
 * - Multiple tasks unblocked in same tick are processed
 *
 * @note Called every 1ms from TIMER1_COMPA interrupt
 * @note Do not call directly!
 */
void os_system_tick(void);

/**
 * Initialize system timer for 1ms tick.
 *
 * Configures Timer1 in CTC mode with 1ms period:
 * - Prescaler: 64
 * - Compare value: 249
 * - Interrupt: TIMER1_COMPA
 *
 * @note Called automatically by os_start()
 * @note Do not call directly unless implementing custom startup
 */
void os_timer_init(void);

/**
 * Get current system tick count.
 *
 * Returns number of milliseconds since os_start() was called.
 *
 * @return System tick count (0 to 2^32-1)
 *
 * Resolution: 1ms
 * Rollover: After 49.7 days
 *
 * Thread-safe: Uses atomic read for uint32_t
 *
 * Use cases:
 * - Timestamp events
 * - Measure elapsed time
 * - Timeout detection
 *
 * Example - Timeout:
 *   uint32_t start = os_get_tick();
 *   while (!ready) {
 *     if (os_get_tick() - start > 1000) {
 *       // Timeout after 1 second
 *       break;
 *     }
 *   }
 */
uint32_t os_get_tick(void);

/* ========================================================================
 * PUBLIC API - SYNCHRONIZATION PRIMITIVES
 * ======================================================================== */

/**
 * Initialize a semaphore.
 *
 * @param sem Pointer to semaphore structure
 * @param initial_count Initial count value
 *
 * Example - Resource pool:
 *   Semaphore buffer_slots;
 *   os_sem_init(&buffer_slots, 5); // 5 slots available
 */
void os_sem_init(Semaphore *sem, uint8_t initial_count);

/**
 * Wait on semaphore (P operation, acquire).
 *
 * If count > 0: Decrement count and continue
 * If count = 0: Block until another task calls os_sem_post()
 *
 * @param sem Pointer to semaphore
 *
 * Blocking: Yes (if count = 0)
 * Fairness: FIFO (first waiting task is woken first)
 *
 * Example:
 *   os_sem_wait(&buffer_slots); // Wait for available slot
 *   // Use resource
 */
void os_sem_wait(Semaphore *sem);

/**
 * Signal semaphore (V operation, release).
 *
 * If tasks waiting: Wake highest priority waiting task
 * If no tasks waiting: Increment count
 *
 * @param sem Pointer to semaphore
 *
 * Example:
 *   // Done with resource
 *   os_sem_post(&buffer_slots); // Release slot
 */
void os_sem_post(Semaphore *sem);

/**
 * Initialize a mutex.
 *
 * @param mutex Pointer to mutex structure
 *
 * Example:
 *   Mutex uart_mutex;
 *   os_mutex_init(&uart_mutex);
 */
void os_mutex_init(Mutex *mutex);

/**
 * Lock mutex (acquire exclusive access).
 *
 * If unlocked: Lock mutex and continue
 * If locked: Block until owner unlocks
 *
 * @param mutex Pointer to mutex
 *
 * Blocking: Yes (if already locked)
 * Ownership: Tracked (for future priority inheritance)
 * Fairness: FIFO
 *
 * @warning Must call os_mutex_unlock() from same task!
 * @warning No recursive locking - will deadlock!
 *
 * Example:
 *   os_mutex_lock(&uart_mutex);
 *   Serial.println("Critical section");
 *   os_mutex_unlock(&uart_mutex);
 */
void os_mutex_lock(Mutex *mutex);

/**
 * Unlock mutex (release exclusive access).
 *
 * If tasks waiting: Wake highest priority waiting task
 * If no tasks waiting: Mark mutex as unlocked
 *
 * @param mutex Pointer to mutex
 *
 * @warning Only the owner should unlock the mutex!
 *          Currently not enforced, but may be in future versions.
 */
void os_mutex_unlock(Mutex *mutex);

/* ========================================================================
 * PUBLIC API - MESSAGE QUEUES
 * ======================================================================== */

/**
 * Create a message queue.
 *
 * Allocates queue structure and buffer from memory pool.
 *
 * @param capacity Maximum number of messages queue can hold
 * @return Pointer to message queue, or NULL if out of memory
 *
 * Memory used: sizeof(MessageQueue) + capacity * sizeof(void*)
 *
 * Example:
 *   MessageQueue *mq = os_mq_create(10); // Queue with 10 slots
 *   if (mq == NULL) {
 *     // Out of memory!
 *   }
 */
MessageQueue *os_mq_create(uint8_t capacity);

/**
 * Send message to queue (blocking).
 *
 * If queue full: Block until space available
 * Otherwise: Add message and wake waiting receiver
 *
 * @param mq Pointer to message queue
 * @param msg Message pointer (can be any pointer)
 *
 * Blocking: Yes (if queue full)
 * Overhead: Up to 4 context switches (sem, mutex, mutex, sem)
 *
 * @note Consider os_mq_send_fast() for better performance
 *
 * Example:
 *   int *data = (int*)malloc(sizeof(int));
 *   *data = 42;
 *   os_mq_send(mq, data);
 */
void os_mq_send(MessageQueue *mq, void *msg);

/**
 * Receive message from queue (blocking).
 *
 * If queue empty: Block until message available
 * Otherwise: Remove message and wake waiting sender
 *
 * @param mq Pointer to message queue
 * @return Message pointer
 *
 * Blocking: Yes (if queue empty)
 * Overhead: Up to 4 context switches
 *
 * @note Consider os_mq_receive_fast() for better performance
 *
 * Example:
 *   void *msg = os_mq_receive(mq);
 *   int *data = (int*)msg;
 *   process_data(*data);
 */
void *os_mq_receive(MessageQueue *mq);

/**
 * Send message to queue (fast path optimization).
 *
 * Checks for space availability before blocking.
 * If space available, performs entire operation atomically.
 *
 * @param mq Pointer to message queue
 * @param msg Message pointer
 *
 * Performance:
 * - Non-blocking case: 0 context switches (vs 4 for os_mq_send)
 * - Blocking case: Falls back to os_mq_send()
 *
 * Improvement: ~75% reduction in context switches for typical usage
 *
 * Example:
 *   os_mq_send_fast(mq, data); // Much faster when not full!
 */
void os_mq_send_fast(MessageQueue *mq, void *msg);

/**
 * Receive message from queue (fast path optimization).
 *
 * Checks for message availability before blocking.
 * If message available, performs entire operation atomically.
 *
 * @param mq Pointer to message queue
 * @return Message pointer
 *
 * Performance: Same benefits as os_mq_send_fast()
 *
 * Example:
 *   void *msg = os_mq_receive_fast(mq);
 */
void *os_mq_receive_fast(MessageQueue *mq);

/* ========================================================================
 * PUBLIC API - SYSTEM MONITORING
 * ======================================================================== */

/**
 * Check for stack overflow in current task.
 *
 * Verifies stack canary value at bottom of stack.
 * If corrupted, halts system to prevent further corruption.
 *
 * Call periodically from idle loop or low-priority task.
 *
 * On overflow detection:
 * - Disables interrupts
 * - Enters infinite loop
 * - Could be extended to log error, blink LED, etc.
 *
 * Example:
 *   void idle_task(void) {
 *     while(1) {
 *       os_check_stack_overflow();
 *       os_delay(1000);
 *     }
 *   }
 */
void os_check_stack_overflow(void);

/**
 * Enter idle mode (sleep until interrupt).
 *
 * Puts CPU in low-power IDLE mode.
 * CPU wakes on any interrupt (timer, UART, etc).
 *
 * Power savings: ~10-20% reduction vs busy waiting
 *
 * Called automatically from os_start() main loop.
 * Can also be called from idle task.
 *
 * @note Interrupts must be enabled (sei) for wake-up to work
 */
void os_enter_idle(void);

/**
 * Initialize Watchdog Timer.
 * @param timeout_ms Timeout period in milliseconds.
 */
void os_wdt_init(uint16_t timeout_ms);

/**
 * Reset Watchdog Timer ("Feed the dog").
 *
 * Must be called periodically before the timeout expires.
 * Usually called from the OS Idle Task.
 */
void os_wdt_feed(void);

/**
 * Get stack usage high-water mark for a task.
 *
 * Scans the stack to find the deepest point reached during execution.
 *
 * @param task_id ID of the task to check
 * @return Number of bytes used (including context and canary)
 */
uint16_t os_get_stack_usage(uint8_t task_id);

/**
 * Get current priority of a task (may be inherited).
 * @param task_id ID of the task
 * @return Current priority
 */
uint8_t os_get_priority(uint8_t task_id);

#ifdef __cplusplus
}
#endif

#endif
