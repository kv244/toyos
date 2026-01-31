# ToyOS - Tiny Operating System for Arduino

**Version 2.5 - MULTI-PLATFORM (AVR + ARM Cortex-M)**
**Status:** ✅ Production Ready for Hobbyist/Education
**Last Updated:** January 2026

ToyOS is a lightweight, preemptive Real-Time Operating System (RTOS) designed for embedded systems. It now supports multiple architectures including **Arduino UNO (AVR)** and **Arduino UNO R4 (ARM Cortex-M4)**. It provides priority-based multitasking, inter-process communication, efficient resource management, and a persistent key-value database.

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
- ✅ **Platform-Agnostic Storage**: Auto-selects driver based on board.
  - **AVR**: Internal EEPROM
  - **ARM (R4)**: Data Flash Emulation (via EEPROM.h)
- ✅ **EEPROM Persistence**: 1KB indexable storage.
- ✅ **Thread-Safe CRUD Operations**: Read, Write, Delete with Mutex protection.
- ✅ **Log-Structured Storage**: Append-only writes for simplicity.
- ✅ **Configurable Limits**: Keys up to 24 chars, values up to 1024 bytes.
- ✅ **Comprehensive Test Suite**: Unit, edge-case, and concurrency tests.

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

## 🏗️ Project Structure

```
toyos/
├── libraries/
│   ├── ToyOS/              # Core RTOS library
│   │   ├── src/
│   │   │   ├── toyos.h              # API & configuration
│   │   │   ├── port.h               # Portability Interface
│   │   │   ├── os_kernel_fixed.cpp  # Portable Kernel
│   │   │   └── port/                # Hardware Ports
│   │   │       ├── avr/             # AVR Implementation
│   │   │       └── arm/             # ARM Cortex-M Implementation
│   │   └── library.properties
│   └── KV_DB/              # Key-Value Database library
│       ├── src/
│       │   ├── kv_db.h              # Database API
│       │   ├── hal/                 # Storage Drivers
│       │       ├── storage_driver.h
│       │       ├── storage_avr_eeprom.c
│       │       └── storage_arduino_eeprom.cpp
│       │   ├── kv_db.cpp            # Implementation
│       │   └── test_kv_db.cpp       # Test suite
│       └── library.properties
└── app/
    └── kv_db_demo/         # Demo application
        └── kv_db_demo.ino
```

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

### v2.5 (January 2026) - MULTI-PLATFORM
- ✅ **ARM Cortex-M Port**: Full support for Arduino UNO R4 (PendSV/SysTick).
- ✅ **Portability Layer**: Clean separate of Kernel and Hardware (`port.h`).
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
