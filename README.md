# Arduino Toy RTOS

A basic real-time operating system (RTOS) implementation for Arduino UNO R3 with task scheduling and memory management.

## Features

- **Task Management**: Create and manage multiple tasks with priority levels
- **Round-Robin Scheduler**: Fair task switching with context preservation
- **Task States**: Ready, Running, and Blocked states
- **Delay/Blocking**: Tasks can voluntarily block for a specified number of ticks
- **Memory Management**: Simple memory allocator for task control blocks
- **Tick System**: Automatic unblocking of delayed tasks via system tick

## Files

- `toyos.h`: Header with core data structures and API declarations
- `toyos.c`: Implementation of scheduler, memory manager, and kernel
- `main.c`: Example usage with three sample tasks

## Architecture

### Task Control Block (TCB)
Each task has a control block containing:
- Task ID
- State (Ready/Running/Blocked)
- Task function pointer
- Stack size
- Priority level
- Tick count (for delays)

### Scheduler
- Uses a FIFO ready queue
- Round-robin scheduling (time-slice based)
- Automatic task rotation

### Memory Management
- Simple linear allocator
- Fixed memory pool (1KB default)
- Allocation map tracking

## API Usage

```c
// Initialize OS with memory pool
uint8_t mem_pool[1024];
os_init(mem_pool, sizeof(mem_pool));

// Create a task
void my_task(void) {
    // Task code here
}
os_create_task(1, my_task, 1, 64);

// Block current task for N ticks
os_delay(10);

// Yield to scheduler
os_task_yield();

// Start scheduler loop
os_start();
```

## Compilation

For Arduino UNO:
```bash
avr-gcc -mmcu=atmega328p -Os -c toyos.c -o toyos.o
avr-gcc -mmcu=atmega328p -Os -c main.c -o main.o
avr-gcc -mmcu=atmega328p toyos.o main.o -o toyos.elf
avr-objcopy -O ihex toyos.elf toyos.hex
avrdude -p atmega328p -c arduino -P /dev/ttyUSB0 -U flash:w:toyos.hex
```

For testing on host:
```bash
gcc -o toyos_test toyos.c main.c
./toyos_test
```

## System Tick Integration

For real-time blocking, integrate `os_system_tick()` into a timer interrupt:

```c
// Arduino Timer1 interrupt (1ms tick)
ISR(TIMER1_COMPA_vect) {
    os_system_tick();
}
```

## Limitations

- Maximum 8 concurrent tasks (can be increased with more memory)
- Simple linear memory allocator (no fragmentation management)
- No dynamic memory deallocation (pool-based)
- Fixed time slice duration
- No task priority enforcement (FIFO behavior)

## Future Enhancements

- Binary heap for priority queue
- Fragmented memory allocator
- Semaphores and mutexes
- Message queues
- Task stack overflow detection
- Power management modes
