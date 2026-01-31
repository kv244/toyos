# ToyOS - Tiny Operating System for Arduino

**Version 2.2 - FIXED & OPTIMIZED**

ToyOS is a lightweight, preemptive Real-Time Operating System (RTOS) designed specifically for the Arduino UNO (ATmega328P). It provides priority-based multitasking, inter-process communication, and efficient resource management while maintaining a minimal memory footprint.

## Features

- **Preemptive Multitasking**: Hardware timer-driven context switching (1ms tick).
- **Priority Scheduling**: Binary heap scheduler ensuring O(log n) task selection.
- **Inter-Task Communication**:
  - **Semaphores**: Counting semaphores for resource coordination.
  - **Mutexes**: Binary mutexes for data protection.
  - **Message Queues**: Thread-safe data passing with "Fast Path" optimization.
- **Memory Efficient**:
  - Stackless scheduler (uses task stacks).
  - O(1) "Delta Queue" for efficient `os_delay()`.
  - Flash-string support (`F()`) to save SRAM.
- **Robustness**:
  - Stack overflow detection (Canaries).
  - Atomic startup sequence.

## Architecture

The system consists of four main components:

1.  **`toyos.h`**: core configurations and API definitions.
2.  **`os_kernel_fixed.cpp`**: The kernel logic (scheduler, IPC primitives, memory manager).
3.  **`os_switch_fixed.S`**: Hand-optimized assembly for context switching.
4.  **`toyos.ino`**: The main application file (sketch).

## Setup & Compilation

### Requirements
- Arduino IDE or `arduino-cli`.
- Arduino UNO (ATmega328P) or compatible.

### Compiling with arduino-cli

```bash
# Compile
arduino-cli compile --fqbn arduino:avr:uno .

# Upload (Windows Example)
taskkill /F /IM arduino-cli.exe  # Ensure COM port is free
arduino-cli upload -p COM6 --fqbn arduino:avr:uno .

# Monitor Output
arduino-cli monitor -p COM6 --config baudrate=9600
```

## Demo Application

The included `toyos.ino` demonstrates a **Multi-Consumer** pattern:

- **Producer Task**: Generates an integer every 1 second and sends it to a queue.
- **Consumer Tasks (1 & 2)**: Two tasks compete to read from the queue. The OS load-balances messages between them.
- **Idle Task**: Runs when no other task is active, checking system health.

### Expected Output
```
ToyOS V2.2 - Multi-Consumer Demo (FAST MQ)
================================
Starting Pre-emptive Scheduler...
Prod Sent: 0
Cons2 Got: 0
Prod Sent: 1
Cons1 Got: 1
Prod Sent: 2
Cons2 Got: 2
...
```

## Known Limitations
- No dynamic memory deallocation (`os_free` is a no-op).
- No priority inheritance for mutexes (risk of priority inversion).
- Max 32 tasks (configurable).
