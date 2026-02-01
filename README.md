# ToyOS - Tiny Operating System for Arduino

**Version 2.5.2 - MULTI-PLATFORM (AVR + ARM Cortex-M)**  
**Status:** ✅ Production Ready for Hobbyist/Education  
**Last Updated:** February 1, 2026

ToyOS is a lightweight, preemptive Real-Time Operating System (RTOS) designed for embedded systems. It supports multiple architectures including **Arduino UNO (AVR)** and **Arduino UNO R4 (ARM Cortex-M4)** with a unified CPU abstraction layer for maximum performance. Features include priority-based multitasking, inter-process communication, efficient resource management, and a persistent key-value database.

**Recent Improvements (v2.5.2):**
- ✅ **Hardened MPU Sandbox** - Restricted peripheral access and kernel protection (ARM)
- ✅ **Secure Serial Gateway** - Use `os_print()` for thread-safe, MPU-aware logging
- ✅ **KV Database Stabilization** - Complete rewrite for LFS support and wear leveling
- ✅ **Multi-Platform Build Verified** - 100% success on both AVR and ARM architectures
- ✅ **RAM-Based Indexing** - Optimized lookups with O(1) performance on ARM
- ✅ **Hierarchical Keys** - Supporting prefix-based iteration for structured paths

---

## 🌟 Features

### Core RTOS Features
- ✅ **Multi-Platform Architecture**: Seamlessly runs on AVR (ATmega328P) and ARM Cortex-M (Renesas RA4M1).
- ✅ **Hardware Abstraction Layer (HAL)**: Clean separation of kernel and hardware specifics.
- ✅ **Priority-Based Preemptive Scheduling**: Uses binary heap for O(log N) task selection.
- ✅ **Priority Inheritance Protocol**: Prevents priority inversion for Mutexes.
- ✅ **Watchdog Timer Integration**: Hardware recovery from system hangs.
- ✅ **Stack High-Water Mark tracking**: Accurate stack usage monitoring.
- ✅ **Stack Overflow Detection**: Canary-based protection.
- ✅ **Optimized Context Switching**: 
  - AVR: Hand-coded assembly (~35 cycles)
  - ARM: Efficient PendSV handler with FPU context support

### KV Database (Storage HAL)
- ✅ **Log-Structured File System (LFS)**: Sector-based circular log for **Wear Leveling**.
- ✅ **Hierarchical Namespaces**: Support for structured keys (e.g., `config/net/ssid`) with **Prefix Iteration**.
- ✅ **RAM-Based Indexing**: Fast O(log N) binary search for lookups and iterations.
- ✅ **Platform-Agnostic Storage**: Auto-selects driver (AVR EEPROM / ARM Data Flash).
- ✅ **Data Integrity**: Magic numbers and CRC32 checksums (Key + Value) for robustness.
- ✅ **Thread-Safe**: Fully protected by OS primitives.

### Security & Reliability
- ✅ **True MPU Isolation (ARM)**:
  - **Privilege Separation**: Kernel runs Privileged, Tasks run Unprivileged.
  - **System Call Interface (SVC)**: User tasks access kernel services via secure gates.
  - **Peripheral Lockdown**: Denies tasks direct access to hardware registers (UART/GPIO/Timers).
  - **Memory Protection**: Hardware-enforced barriers for Kernel Code/Data and Task Stacks.
  - **Secure Serial Gateway**: `os_print()` provides mediated access to debug output.
- ✅ **Stack Protection**: Guard regions and Canary detection.
- ✅ **Watchdog Timer**: Hardware recovery.

### Synchronization Primitives
- ✅ **Semaphores**: Counting semaphores for resource coordination.
- ✅ **Mutexes**: Binary mutexes with priority inheritance for data protection.
- ✅ **Message Queues**: Thread-safe data passing with "Fast Path" optimization (75% fewer context switches).

### Memory Efficiency
- **Dynamic Memory**: `os_malloc` / `os_free` with Coalescing (First-Fit Free List).
- **Optimized Memory Usage**: 512-byte pool + 6-task limit = 61% SRAM usage (down from 79%).
- **Flash Strings**: `F()` macro support to save SRAM.
- **Low Footprint**: ~9KB Flash, ~1.3KB SRAM (leaving ~700 bytes for user apps).

### Robustness
- **Atomic Startup**: Prevents race conditions during initialization.
- **Critical Fixes**: v2.2 resolves stack corruption (PCH/PCL swap) and startup races.

---

## 🛠️ Development Tools

### Memory Analysis
ToyOS includes tools to analyze memory usage and identify top RAM consumers.

**Local Analysis:**
Run the PowerShell script to compile and analyze RAM usage:
```powershell
.\analyze_memory.ps1
```
This script will:
1. Compile the project (generating a map file)
2. Report total Flash/SRAM usage (`avr-size`)
3. List the top 20 global variables consuming RAM (`avr-nm`)

**CI/CD Pipeline:**
The project includes a GitHub Actions workflow (`.github/workflows/memory_analysis.yml`) that automatically:
- Compiles the code on every push/PR
- Analyzes memory usage
- Uploads the map file and ELF as build artifacts
- Logs the top RAM consumers directly in the workflow run summary

---

## 🏗️ Project Structure

```
toyos/
├── .gitignore                      # Build artifacts exclusion
├── README.md                       # Project documentation
├── CODE_QUALITY_SUMMARY.md         # Code quality report
├── app/                            # Application examples
│   └── kv_db_demo/                 # KV database demo
│       └── kv_db_demo.ino
└── libraries/
    ├── ToyOS/                      # Core RTOS library
    │   ├── library.properties
    │   └── src/
    │       ├── toyos.h             # Main RTOS API
    │       ├── toyos_config.h      # Configuration
    │       ├── os_kernel_fixed.cpp # Kernel implementation
    │       ├── port.h              # Unified porting interface
    │       ├── port/               # Platform abstraction layer
    │       │   ├── README.md       # Porting guide
    │       │   ├── cpu_port.h      # CPU primitives (inline)
    │       │   ├── avr/            # AVR (ATmega328P) port
    │       │   │   ├── port_avr.h
    │       │   │   ├── port_avr.c
    │       │   │   └── port_avr_asm.S
    │       │   └── arm/            # ARM Cortex-M port
    │       │       ├── port_arm.h
    │       │       └── port_arm.c
    │       └── hal/                # Hardware abstraction layer
    │           ├── README.md       # HAL documentation
    │           ├── storage_driver.h/c      # Unified storage API
    │           ├── storage_avr_eeprom.h/c  # Native AVR EEPROM
    │           └── storage_arduino_eeprom.h/cpp  # Arduino library wrapper
    └── KV_DB/                      # Key-Value database library
        ├── library.properties
        └── src/
            ├── kv_db.h             # Database API
            └── kv_db.c             # Implementation
```

### Design Principles

- **Separation of Concerns**: Kernel, HAL, and platform code are cleanly separated
- **Inline Performance**: Critical path functions use inline assembly/intrinsics  
- **Zero Overhead Abstraction**: Platform detection at compile-time
- **Documentation**: Each major subsystem has its own README

---

## 📚 Documentation

ToyOS includes comprehensive documentation at multiple levels:

### Core Documentation
- **README.md** (this file) - Project overview and quick start guide
- **CODE_QUALITY_SUMMARY.md** - Detailed quality analysis and metrics
- **CODEBASE_ANALYSIS.md** - Comprehensive improvement recommendations

### Subsystem Documentation
- **port/README.md** - Porting layer architecture and platform guide
- **hal/README.md** - Hardware abstraction layer documentation

### Development Guides
- **CLEANUP_SUMMARY.md** - Source tree organization report

**Code Quality Grade: A- (Excellent with minor improvements)**

---

## 🚀 Quick Start

### 1. Requirements
- **Arduino CLI** (recommended) or Arduino IDE
- **Arduino UNO R3** (ATmega328P) OR
- **Arduino UNO R4** (Minima/WiFi)
- **Serial Monitor** for output

### 2. Installing Arduino CLI (if needed)

**Windows (PowerShell):**
```powershell
# Download and install
Invoke-WebRequest -Uri https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Windows_64bit.zip -OutFile arduino-cli.zip
Expand-Archive arduino-cli.zip -DestinationPath C:\arduino-cli
$env:PATH += ";C:\arduino-cli"

# Initialize and install cores
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install arduino:renesas_uno
```

**Linux/macOS:**
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install arduino:renesas_uno
```

### 3. Building the Project

**For Arduino UNO R3 (AVR):**
```bash
arduino-cli compile --fqbn arduino:avr:uno --libraries libraries app/kv_db_demo
```

**For Arduino UNO R4 WiFi (ARM):**
```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --libraries libraries app/kv_db_demo
```

**Expected Output:**
```
Sketch uses 8798 bytes (27%) of program storage space.
Global variables use 1268 bytes (61%) of dynamic memory, leaving 780 bytes for local variables.
```

### 4. Uploading to Arduino

**Step 1: Find your Arduino's COM port**
```bash
arduino-cli board list
```

**Step 2: Upload the sketch** (replace `COM6` with your port):
```bash
arduino-cli upload -p COM6 --fqbn arduino:avr:uno app/kv_db_demo
```

**On Linux/macOS**, the port will be something like `/dev/ttyACM0` or `/dev/ttyUSB0`:
```bash
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno app/kv_db_demo
```

### 5. Monitoring Serial Output

```bash
arduino-cli monitor -p COM6 --config baudrate=9600
```

**Expected Output:**
```
--- Starting DB Test Suite ---
[TEST] Basic CRUD: PASS
[TEST] Persistence: PASS
[TEST] Key too long limit: PASS
[TEST] Value too long limit: PASS
[TEST] Concurrency: PASS
--- All Tests Passed ---

[DB] Starting KV Database Demo...
[DB] Writing key 'username' with value 'julia'...
[DB] Write successful.
[DB] Reading key 'username'...
Read: julia
...
```

---

## 📚 KV Database API

### Initialization
```cpp
#include <kv_db.h>

kv_result_t kv_init(void);  // Must be called once at startup
```

### CRUD Operations
```cpp
// Write (creates or updates)
kv_result_t kv_write(const char* key, const char* value, uint16_t val_len);

// Read
kv_result_t kv_read(const char* key, char* buffer, uint16_t max_len, uint16_t* actual_len);

// Delete
kv_result_t kv_delete(const char* key);

// Clear entire database
kv_result_t kv_clear(void);
```

### Return Codes
- `KV_SUCCESS`: Operation completed successfully
- `KV_ERR_NOT_FOUND`: Key does not exist
- `KV_ERR_FULL`: EEPROM is full
- `KV_ERR_KEY_TOO_LONG`: Key exceeds 24 characters
- `KV_ERR_VAL_TOO_LONG`: Value exceeds 1024 bytes

### Example Usage
```cpp
#include <kv_db.h>
#include <toyos.h>

void task_demo(void) {
  kv_init();
  
  // Write
  const char* username = "alice";
  kv_write("user", username, strlen(username));
  
  // Read
  char buffer[32];
  uint16_t len;
  if (kv_read("user", buffer, sizeof(buffer), &len) == KV_SUCCESS) {
    buffer[len] = '\0';
    Serial.println(buffer);  // Prints: alice
  }
  
  // Delete
  kv_delete("user");
}
```

---

## 🎮 Demo Application

The `kv_db_demo` application demonstrates:
1. **Automated Test Suite**: Runs comprehensive tests on startup
2. **Manual CRUD Operations**: Demonstrates write, read, update, and delete
3. **Thread Safety**: Shows database access from RTOS tasks

### Demo Flow
1. Initializes ToyOS and KV Database
2. Runs automated tests (CRUD, persistence, edge cases, concurrency)
3. Performs manual operations on the `username` key
4. Uses a watchdog-protected idle task

---

## 🔧 Configuration

ToyOS v2.4+ features a comprehensive configuration system allowing customization without modifying library code.

### Quick Configuration

For most projects, default values work great. To customize:

```cpp
// Define custom parameters BEFORE including toyos.h
#define TOYOS_MAX_TASKS 8
#define TOYOS_DEFAULT_STACK_SIZE 128
#include <toyos.h>
```

### Full Configuration

Create `toyos_user_config.h` in your sketch folder:

```cpp
#ifndef TOYOS_USER_CONFIG_H
#define TOYOS_USER_CONFIG_H

#define TOYOS_MAX_TASKS 8
#define TOYOS_DEFAULT_STACK_SIZE 128
#define TOYOS_MEMORY_POOL_SIZE 1024
#define TOYOS_ENABLE_DEBUG 1

#endif
```

Then enable it in your sketch:

```cpp
#define TOYOS_USER_CONFIG
#include <toyos.h>
```

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_MAX_TASKS` | 6 | Maximum concurrent tasks (1-32) |
| `TOYOS_DEFAULT_STACK_SIZE` | 96 | Default stack size in bytes |
| `TOYOS_MIN_STACK_SIZE` | 48 | Minimum safety limit |
| `TOYOS_DEFAULT_PRIORITY` | 5 | Default task priority (0-255) |
| `TOYOS_TICK_RATE_HZ` | 1000 | System tick frequency (1ms tick) |
| `TOYOS_ENABLE_WATCHDOG` | 1 | Enable watchdog timer |
| `TOYOS_DEBUG` | 0 | Enable debug assertions |

**See [`CONFIGURATION.md`](libraries/ToyOS/CONFIGURATION.md) for complete documentation.**

### KV Database (kv_db.h)
| Constant | Default | Description |
|----------|---------|-------------|
| `KV_MAX_KEY_LEN` | 24 | Maximum key length |
| `KV_MAX_VAL_LEN` | 1024 | Maximum value length |
| `KV_EEPROM_SIZE` | 1024 | Total EEPROM size (Arduino UNO) |
| `KV_MAX_KEYS` | 18 | Maximum number of unique keys |

---

## 🛠️ Troubleshooting

### Compilation Fails
- **Error:** `avr-gcc: not found`
  - **Fix:** Install Arduino AVR core: `arduino-cli core install arduino:avr`

### Upload Fails
- **Error:** `Permission denied` or `Access is denied`
  - **Fix (Windows):** Close Serial Monitor or any program using the COM port
  - **Fix (Linux):** Add user to dialout group: `sudo usermod -a -G dialout $USER`

### SRAM Issues
- **Error:** `Low memory available, stability problems may occur`
  - **Fix:** Reduce `mem_pool` size or use fewer tasks
  - **Tip:** Use `F()` macro for constant strings to save SRAM

### EEPROM Full
- **Error:** `KV_ERR_FULL` when writing
  - **Fix:** Call `kv_clear()` to reset the database
  - **Note:** The log-structured design appends all writes; use `kv_compact()` to reclaim space.

---

## 📝 Version History

### v2.5.2 (February 2026) - SECURITY HARDENING
- ✅ **Hardened Sandbox**: Denied unprivileged access to all peripheral blocks on ARM.
- ✅ **Secure Serial**: Added `os_print()` syscall for mediated Serial access.
- ✅ **KV DB Stabilization**: Full rewrite of `kv_db.cpp` with LFS and wear leveling.
- ✅ **Security Demo**: Added intentional violation test to verify MPU enforcement.

### v2.5.1 (January 2026) - PORTABILITY REFINED
- ✅ **O(1) ARM Performance**: Added Shadow Index (Hash Table) for constant-time lookups on R4.
- ✅ **Hardware CRC32**: Integrated Renesas RA4M1 CRC hardware engine for KV Database.
- ✅ **Stability Fixes**: Resolved multi-platform lock/mutex bugs in compaction and concurrency routines.
- ✅ **Full Compaction**: Verified automated EEPROM space reclamation on all architectures.

### v2.5 (January 2026) - MULTI-PLATFORM
- ✅ **ARM Cortex-M Port**: Full support for Arduino UNO R4 (PendSV/SysTick).
- ✅ **Portability Layer**: Clean separation of Kernel and Hardware (`port.h`).
- ✅ **Storage HAL**: Driver-based storage abstraction (AVR EEPROM / R4 Flash).
- ✅ **Zero Regression**: 100% backward compatible with AVR.

### v2.4.1 (January 2026) - KV DATABASE
- ✅ **New Feature**: Persistent key-value database with EEPROM storage
- ✅ **Thread-Safe**: All operations protected with ToyOS Mutex
- ✅ **Test Suite**: Comprehensive unit, edge-case, and concurrency tests
- ✅ **Memory Optimization**: Reduced MAX_TASKS to 6, optimized mem_pool (61% SRAM usage)
- ✅ **Bug Fix**: Fixed 1026-byte stack overflow in test suite

### v2.4 (January 2026) - ADVANCED FEATURES
- ✅ **Priority Inheritance Protocol**: Added support for Mutex priority bumping
- ✅ **Watchdog Timer Integration**: Hardware recovery from system hangs
- ✅ **Stack High-Water Mark tracking**: Performance tuning via `os_get_stack_usage()`

### v2.3 (January 2026) - DYNAMIC MEMORY
- ✅ **Free List Allocator**: Replaced Bump Allocator with coalescing support
- ✅ **`os_free` Support**: Memory can now be freed and reused

### v2.2 (January 2026) - STABILITY
- ✅ **Fixed Critical Bug**: Stack initialization order (PCH/PCL swap)
- ✅ **Fixed Race Condition**: Premature interrupt enabling during startup

---

## 📄 License

MIT License. Use at your own risk.

---

## 🤝 Contributing

This is an educational project. Feel free to fork, modify, and learn from the code!

**Key Learning Areas:**
- RTOS internals (scheduling, context switching, IPC)
- AVR assembly programming
- Memory management on constrained systems
- EEPROM persistence and log-structured storage
- Thread-safe programming with mutexes
## 🎓 Lessons Learned (AVR Porting)

### Stack Initialization & Context Switching
*   **PC Byte Order:** The AVR stack stores the Program Counter (PC) in **High Byte then Low Byte** order (when pushed manually). This means the High Byte is at the lower memory address (Top of Stack). This aligns with the `RET` instruction which pops `$PC_H` then `$PC_L`.
    *   *Correct:* `push PCH; push PCL` (Logic: PCH at `SP`, PCL at `SP-1`? No. `*sp-- = PCL; *sp-- = PCH` results in `[PCH][PCL]`).
    *   *Verified Code:* `*sp-- = (func & 0xFF); *sp-- = ((func >> 8) & 0xFF);`
*   **GCC Function Pointers:** `avr-gcc` function pointers for ATmega328P (16-bit PC) are **Word Addresses** (e.g., `0x35D`). They do **not** need to be shifted (`>> 1`) when pushing to the stack for `RET`. The hardware `RET` uses the value directly (or GCC already pre-adjusted it).
*   **Register Order:** Context save/restore order must match exactly.
    *   Save: `Push R0, SREG, R1 ... R31`
    *   Restore: `Pop R31 ... R1, SREG, R0`

### Stack Overflow & ISRs
*   **ISR Overhead:** The Timer1 ISR (Scheduler tick) consumes ~35 bytes of stack (32 registers + SREG + return address).
*   **Task Stack Sizing:** Small stacks (e.g., 64-96 bytes) are insufficient for tasks that get preempted by the scheduler ISR.
    *   *Minimum Recommended:* 160 bytes for Idle, 256+ bytes for Application tasks.
*   **Manifestation:** Stack overflows often looked like "Reset loops" or "Freezes" because the ISR would overwrite the TCB or kernel variables.

### Debugging Strategy
*   **Isolate Interrupts:** Disabling `TIMSK1` (Timer Interrupt) was key to verifying if the "Task Jump" logic was correct. If the task started with Timer disabled, the Jump was good, and the crash was likely the ISR or Stack Overflow.
*   **Memory Protection:** Increased `mem_pool` allocation significantly (400 -> 800 bytes) to rule out heap exhaustion.
