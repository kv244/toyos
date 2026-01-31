# ToyOS - Tiny Operating System for Arduino

**Version 2.2 - FIXED & OPTIMIZED**
**Status:** ✅ Production Ready for Hobbyist/Edu
**Last Updated:** January 2026

ToyOS is a lightweight, preemptive Real-Time Operating System (RTOS) designed specifically for the Arduino UNO (ATmega328P). It provides priority-based multitasking, inter-process communication, and efficient resource management while maintaining a minimal memory footprint.

---

## 🌟 Features

### Core Features
- ✅ **Preemptive Multitasking**: Hardware timer-driven context switching (1ms tick).
- ✅ **Priority Scheduling**: Binary heap scheduler ensuring O(log n) task selection.
- ✅ **Stack Overflow Detection**: Canary-based protection.
- ✅ **Delta Queue Delays**: O(1) tick processing for sleeping tasks.
- ✅ **Optimized Context Switching**: Hand-coded assembly for minimal overhead (~35 cycles).

### Synchronization Primitives
- ✅ **Semaphores**: Counting semaphores for resource coordination.
- ✅ **Mutexes**: Binary mutexes for data protection.
- ✅ **Message Queues**: Thread-safe data passing with "Fast Path" optimization (75% fewer context switches).

### Memory Efficiency
- **Dynamic Memory**: `os_malloc` / `os_free` with Coalescing (First-Fit Free List).
- **Stackless Scheduler**: Uses task stacks directly.
- **Flash Strings**: `F()` macro support to save SRAM.
- **Low Footprint**: ~4KB Flash, ~1KB SRAM (leaving ~1KB for user app).

### Robustness
- **Atomic Startup**: Prevents race conditions during initialization.
- **Critical Fixes**: v2.2 resolves stack corruption (PCH/PCL swap) and startup races.

---

## 🏗️ Architecture

The system consists of four main components:
1.  **`toyos.h`**: core configurations and API definitions.
2.  **`os_kernel_fixed.cpp`**: The kernel logic (scheduler, IPC, Free List Allocator).
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
arduino-cli monitor -p COM6 --config baudrate=9600
```

### 3. Basic Example (Blinky)

```cpp
#include "toyos.h"
#include <Arduino.h>

/* Use aligned memory pool for correct malloc behavior */
static uint8_t mem_pool[1024] __attribute__((aligned(2)));

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
  os_create_task(1, task_blink, 5, 128);
  os_start();
}

void loop() {}
```

---

## 🎮 Demo Application (Multi-Consumer & Allocator Test)

The included `toyos.ino` demonstrates a **Producer-Consumer** pattern and **Dynamic Memory Test**:

- **Producer Task**: Generates data AND tests `os_malloc`/`os_free` every second (stress test).
- **Consumer Tasks 1 & 2**: Two tasks compete to read from the load-balanced message queue.
- **Fast Path**: Uses `os_mq_send_fast` / `os_mq_receive_fast`.

### Expected Serial Output
```
ToyOS V2.2 - Comprehensive Test Suite
=====================================
OS Init: OK
...
Starting Pre-emptive Scheduler...
Prod Sent: 0 @ Tick: 1
Cons2 Got (Fast): 0
Prod Sent: 1 @ Tick: 1001
Cons1 Got (Fast): 1
```

---

## 🔧 Configuration (toyos.h)

| Constant | Default | Description |
|----------|---------|-------------|
| `MAX_TASKS` | 8 | Maximum number of concurrent tasks (1-32) |
| `DEFAULT_STACK_SIZE` | 128 | Default stack size in bytes |
| `MIN_STACK_SIZE` | 48 | Minimum safety limit |
| `STACK_CANARY` | 0xDEADBEEF | Overflow detection pattern |

---

## 📝 Version History

### v2.3 (January 2026) - DYNAMIC MEMORY
- ✅ **Dynamic Memory**: Replaced Bump Allocator with **Free List Allocator**.
- ✅ **`os_free` Support**: Added support for freeing and coalescing memory blocks.
- ✅ **Verification**: Added `malloc`/`free` stress tests improving Heap/Stack stability.

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
