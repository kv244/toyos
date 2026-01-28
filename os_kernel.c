#include "toyos.h"
#include <avr/interrupt.h>
#include <avr/io.h>


/* Global Kernel Instance */
static Kernel kernel = {0};

/* Initialize the OS */
void os_init(uint8_t *mem_pool, uint16_t mem_size) {
  kernel.ready_queue.head = NULL;
  kernel.ready_queue.tail = NULL;
  kernel.ready_queue.count = 0;
  kernel.blocked_queue.head = NULL;
  kernel.blocked_queue.tail = NULL;
  kernel.blocked_queue.count = 0;
  kernel.current_task = NULL;
  kernel.system_tick = 0;
  kernel.task_count = 0;

  /* Initialize memory manager */
  kernel.mem_manager.memory_pool = mem_pool;
  kernel.mem_manager.total_size = mem_size;
  kernel.mem_manager.allocated = 0;

  memset(mem_pool, 0, mem_size);
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

/* Get current tick count */
uint16_t os_get_tick(void) {
  uint16_t tick;
  cli();
  tick = kernel.system_tick;
  sei();
  return tick;
}

/* Add task node to a queue */
static void queue_add(TaskQueue *queue, TaskNode *node) {
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

/* Remove task node from front of queue */
static TaskNode *queue_remove(TaskQueue *queue) {
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

/* Create a new task */
void os_create_task(uint8_t id, void (*task_func)(void), uint8_t priority,
                    uint16_t stack_size) {
  if (kernel.task_count >= 8)
    return; /* Max 8 tasks on Arduino UNO */

  TaskNode *new_node = (TaskNode *)os_malloc(sizeof(TaskNode));
  if (!new_node)
    return;

  new_node->task.id = id;
  new_node->task.state = TASK_READY;
  new_node->task.task_func = task_func;
  new_node->task.priority = priority;
  new_node->task.stack_size = stack_size;
  new_node->task.tick_count = 0;
  new_node->next = NULL;

  queue_add(&kernel.ready_queue, new_node);
  kernel.task_count++;
}

/* Simple Round-Robin Scheduler */
void os_scheduler(void) {
  if (kernel.ready_queue.head == NULL)
    return;

  /* Get next ready task from queue */
  TaskNode *next_node = queue_remove(&kernel.ready_queue);
  if (next_node && next_node->task.state == TASK_READY) {
    kernel.current_task = &next_node->task;
    kernel.current_task->state = TASK_RUNNING;

    /* Re-queue the task at the back */
    queue_add(&kernel.ready_queue, next_node);
  }
}

/* Execute current task */
static void os_run_task(void) {
  if (kernel.current_task && kernel.current_task->task_func) {
    kernel.current_task->task_func();
  }
}

/* Task Delay (blocks task for N ticks) */
void os_delay(uint16_t ticks) {
  if (kernel.current_task && ticks > 0) {
    kernel.current_task->state = TASK_BLOCKED;
    kernel.current_task->tick_count = ticks;

    /* Move task to blocked queue - find and remove from ready queue */
    TaskNode *prev = NULL;
    TaskNode *curr = kernel.ready_queue.head;
    while (curr) {
      if (&curr->task == kernel.current_task) {
        /* Remove from ready queue */
        if (prev) {
          prev->next = curr->next;
        } else {
          kernel.ready_queue.head = curr->next;
        }
        if (curr == kernel.ready_queue.tail) {
          kernel.ready_queue.tail = prev;
        }
        kernel.ready_queue.count--;

        /* Add to blocked queue */
        queue_add(&kernel.blocked_queue, curr);
        break;
      }
      prev = curr;
      curr = curr->next;
    }

    kernel.current_task = NULL;
  }
}

/* Task Yield - voluntary context switch */
void os_task_yield(void) {
  if (kernel.current_task) {
    kernel.current_task->state = TASK_READY;
    kernel.current_task = NULL;
  }
}

/* Simple Memory Allocation (bump allocator) */
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

/* System tick (called from Timer1 ISR) */
void os_system_tick(void) {
  kernel.system_tick++;

  /* Check blocked tasks and unblock if ready */
  TaskNode *prev = NULL;
  TaskNode *curr = kernel.blocked_queue.head;

  while (curr) {
    TaskNode *next = curr->next;

    if (curr->task.tick_count > 0) {
      curr->task.tick_count--;
      if (curr->task.tick_count == 0) {
        /* Remove from blocked queue */
        if (prev) {
          prev->next = next;
        } else {
          kernel.blocked_queue.head = next;
        }
        if (curr == kernel.blocked_queue.tail) {
          kernel.blocked_queue.tail = prev;
        }
        kernel.blocked_queue.count--;

        /* Add to ready queue */
        curr->task.state = TASK_READY;
        queue_add(&kernel.ready_queue, curr);

        /* Don't update prev since we removed curr */
      } else {
        prev = curr;
      }
    } else {
      prev = curr;
    }
    curr = next;
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
