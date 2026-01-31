# ToyOS - Tiny Operating System for Arduino

**Version 2.5 - BUDDY ALLOCATOR & MEMORY OPTIMIZATION**
**Status:** ✅ Production Ready for Hobbyist/Edu
**Last Updated:** January 2026

ToyOS is a lightweight, preemptive Real-Time Operating System (RTOS) designed specifically for the Arduino UNO (ATmega328P). It provides priority-based multitasking, inter-process communication, and efficient resource management while maintaining a minimal memory footprint.

---

## 🌟 Features

### Core Features
- ✅ **Priority-Based Preemptive Scheduling**: Uses binary heap for O(log N) task selection.
- ✅ **Priority Inheritance Protocol**: Prevents priority inversion for Mutexes.
- ✅ **Buddy Allocator**: Deterministic $O(\log N)$ memory management with coalescing.
- ✅ **Watchdog Timer Integration**: Hardware recovery from system hangs.
- ✅ **Stack High-Water Mark tracking**: Accurate stack usage monitoring.
- ✅ **Stack Overflow Detection**: Canary-based protection.
- ✅ **Delta Queue Delays**: O(1) tick processing for sleeping tasks.
- ✅ **Optimized Context Switching**: Hand-coded assembly for minimal overhead (~35 cycles).

### Synchronization Primitives
- ✅ **Semaphores**: Counting semaphores for resource coordination.
- ✅ **Mutexes**: Binary mutexes for data protection.
- ✅ **Message Queues**: Thread-safe data passing with "Fast Path" optimization (75% fewer context switches).

### Memory Efficiency
- **Dynamic Memory**: `os_malloc` / `os_free` using **Buddy System** with atomic protection.
- **Dynamic Tasking**: Task control blocks are allocated from the heap to save global RAM.
- **Flash Strings**: `F()` macro support to save SRAM.
- **Low Footprint**: ~6KB Flash, ~1.1KB SRAM (highly optimized global usage).

### Robustness
- **Atomic Heap**: All memory operations are wrapped in atomic sections for thread safety.
- **Buddy Bounds Checking**: Prevents memory corruption via strict pool boundary verification.

---

## 🏗️ Architecture

The system consists of four main components:
1.  **`toyos.h`**: core configurations and API definitions.
2.  **`os_kernel_fixed.cpp`**: The kernel logic (scheduler, IPC, Buddy Allocator).
3.  **`os_switch_fixed.S`**: Hand-optimized assembly for context switching.
4.  **`toyos.ino`**: The main application file (sketch).

### Scheduler Flow
1. **Priority-Based**: Binary max-heap selects highest priority task.
2. **Preemptive**: Timer1 interrupt (1ms) triggers context switch.
3. **Delta Queue**: Sleeping tasks are stored in a differential list for O(1) wake-up.

---

## 🚀 Quick Start

### 1. Requirements
- Arduino IDE or `arduino-cli`.
- Arduino UNO (ATmega328P) or compatible.

### 2. Compiling with arduino-cli

```bash
# Compile
arduino-cli compile --fqbn arduino:avr:uno .

# Upload (Windows Example)
# Ensure to kill any Serial Monitor instances first to release the COM port lock
taskkill /F /IM arduino-cli.exe
arduino-cli upload -p COM6 --fqbn arduino:avr:uno .

# Monitor Output
arduino-cli monitor -p COM6 --config baudrate=115200
```

### 3. Basic Example (Blinky)

```cpp
#include "toyos.h"
#include <Arduino.h>

/* Use aligned memory pool for the buddy allocator */
static uint8_t mem_pool[896] __attribute__((aligned(2)));

void task_blink(void) {
  pinMode(13, OUTPUT);
  while (1) {
    digitalWrite(13, HIGH);
    os_delay(500);
    digitalWrite(13, LOW);
    os_delay(500);
  }
}

void setup() {
  os_init(mem_pool, sizeof(mem_pool));
  os_create_task(1, task_blink, 5, 96);
  os_start();
}

void loop() {}
```

---

## 🎮 Demo Application (Buddy Allocator Stress Test)

The included `toyos.ino` demonstrates a **Buddy Allocator Stress Test**:

- **Memory Test Task**: Repeatedly allocates and frees blocks of random sizes (8-128 bytes).
- **Corruption Detection**: Verifies block integrity using pattern matching.
- **Dynamic Monitoring**: Prints status updates to Serial.

---

## 🔧 Configuration (toyos.h)

| Constant | Default | Description |
|----------|---------|-------------|
| `MAX_TASKS` | 4 | Maximum number of concurrent tasks (Memory Optimized) |
| `DEFAULT_STACK_SIZE` | 128 | Default stack size in bytes |
| `MIN_STACK_SIZE` | 48 | Minimum safety limit |
| `STACK_CANARY` | 0xDEADBEEF | Overflow detection pattern |

---

## 📝 Version History

### v2.5 (January 2026) - BUDDY ALLOCATOR & OPTIMIZATION
- ✅ **Buddy Allocator**: Replaced Free List with deterministic $O(\log N)$ Buddy System.
- ✅ **Global Memory Optimization**: Reduced SRAM footprint by **300+ bytes**.
- ✅ **Dynamic Tasking**: Switched to heap-allocated `TaskNode` management.
- ✅ **Stability**: Added atomic protection and strict bounds checking to memory operations.

### v2.4 (January 2026) - ADVANCED FEATURES
- ✅ **Priority Inheritance Protocol**: Added support for Mutex priority bumping.
- ✅ **Starvation Safeguard**: Integrated hardware **Watchdog Timer**.
- ✅ **Performance Tuning**: Added **Stack High-Water Mark tracking**.

### v2.2 (January 2026) - FIXED & OPTIMIZED
- ✅ **Fixed Critical Bug**: Stack initialization order (PCH/PCL swap) preventing crashes.
- ✅ **Fixed Race Condition**: Premature interrupt enabling during startup.
- ✅ **Optimization**: Re-enabled "Fast Path" message queue operations.
- ✅ **Multi-Consumer Demo**: Clean load-balancing example.

### v2.1 (January 2026)
- Initial production features (semaphores, mutexes).

---

## 📄 License

MIT License. Use at your own risk.
