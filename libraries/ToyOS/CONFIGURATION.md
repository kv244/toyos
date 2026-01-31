# ToyOS Configuration Guide

## Overview

ToyOS now supports comprehensive configuration through header files, allowing you to customize task limits, stack sizes, priorities, and many other parameters without modifying the library code.

## Configuration Architecture

The configuration system uses a three-layer approach:

1. **Default Configuration** (`toyos_config.h`) - Provides sensible defaults for all parameters
2. **User Configuration** (`toyos_user_config.h`) - Optional user overrides (your custom settings)
3. **Main Header** (`toyos.h`) - Includes configs and uses them throughout the system

## Quick Start

### Method 1: Use Defaults (No Configuration Needed)

Just include ToyOS normally - it will use the default configuration:

```cpp
#include <toyos.h>

static uint8_t mem_pool[512];

void setup() {
  os_init(mem_pool, sizeof(mem_pool));
  os_create_task(1, task_blink, 5, 96);
  os_start();
}
```

### Method 2: Create Custom Configuration

1. **Copy the example configuration:**
   ```
   libraries/ToyOS/examples/toyos_user_config_example.h
   → your_sketch_folder/toyos_user_config.h
   ```

2. **Edit `toyos_user_config.h`** to customize parameters

3. **Enable it in your sketch** by adding `#define TOYOS_USER_CONFIG` **before** including toyos.h:
   ```cpp
   #define TOYOS_USER_CONFIG
   #include <toyos.h>
   ```

### Method 3: Inline Configuration

For quick testing, define parameters directly in your sketch:

```cpp
// Define custom parameters BEFORE including toyos.h
#define TOYOS_MAX_TASKS 8
#define TOYOS_DEFAULT_STACK_SIZE 128
#include <toyos.h>
```

## Configurable Parameters

### Task Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_MAX_TASKS` | 6 | Maximum number of tasks (1-32) |
| `TOYOS_DEFAULT_STACK_SIZE` | 96 | Default stack size in bytes |
| `TOYOS_MIN_STACK_SIZE` | 48 | Minimum allowed stack size |

**Example:**
```cpp
#define TOYOS_MAX_TASKS 4              // Small system
#define TOYOS_DEFAULT_STACK_SIZE 128   // Larger stacks
```

### Priority Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_DEFAULT_PRIORITY` | 5 | Default task priority (0-255) |
| `TOYOS_IDLE_TASK_PRIORITY` | 0 | Idle task priority |

**Priority Guidelines:**
- 0-3: Background tasks (logging, blinking)
- 4-7: Normal application tasks
- 8-15: High priority (sensors, control)
- 16+: Critical (safety, real-time)

**Example:**
```cpp
#define TOYOS_DEFAULT_PRIORITY 10      // Higher default priority
```

### Memory Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_MEMORY_POOL_SIZE` | 512 | Suggested memory pool size in bytes |
| `TOYOS_MEM_ALIGNMENT` | 2 | Memory alignment boundary |

**Sizing Guidelines:**
- Minimum: `(MAX_TASKS × DEFAULT_STACK_SIZE) + 128`
- Recommended: Add 20-50% overhead for queues and structures

**Example:**
```cpp
#define TOYOS_MEMORY_POOL_SIZE 1024    // Larger pool for 8 tasks
static uint8_t mem_pool[TOYOS_MEMORY_POOL_SIZE];
```

### Timing Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_TICK_RATE_HZ` | 1000 | System tick frequency (1ms tick) |
| `TOYOS_TIMER_PRESCALER` | 64 | Timer prescaler value |

**Tick Rate Options:**
- 1000 Hz (1ms): Best responsiveness, higher overhead
- 500 Hz (2ms): Good balance
- 100 Hz (10ms): Low overhead, coarse timing

**Example:**
```cpp
#define TOYOS_TICK_RATE_HZ 500         // 2ms tick for lower overhead
#define TOYOS_TIMER_PRESCALER 64
```

### Stack Overflow Detection

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_STACK_CANARY` | 0xDEADBEEF | Canary value for overflow detection |
| `TOYOS_AUTO_STACK_CHECK` | 0 | Auto-check on context switch |

**Example:**
```cpp
#define TOYOS_AUTO_STACK_CHECK 1       // Enable for debugging
#define TOYOS_STACK_CANARY 0xCAFEBABE  // Custom canary
```

### Watchdog Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_ENABLE_WATCHDOG` | 1 | Enable watchdog support |
| `TOYOS_WATCHDOG_TIMEOUT` | WDTO_4S | Watchdog timeout period |

**Timeout Options:**
`WDTO_15MS`, `WDTO_30MS`, `WDTO_60MS`, `WDTO_120MS`, `WDTO_250MS`, `WDTO_500MS`, `WDTO_1S`, `WDTO_2S`, `WDTO_4S`, `WDTO_8S`

**Example:**
```cpp
#define TOYOS_ENABLE_WATCHDOG 1
#define TOYOS_WATCHDOG_TIMEOUT WDTO_2S // 2 second timeout
```

### Debug and Diagnostics

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_DEBUG` | 0 | Enable debug assertions |
| `TOYOS_ENABLE_STATS` | 0 | Enable runtime statistics |

**Example:**
```cpp
#define TOYOS_DEBUG 1                  // Enable assertions
#define TOYOS_ENABLE_STATS 1           // Track statistics
```

### Feature Toggles

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TOYOS_ENABLE_PRIORITY_INHERITANCE` | 1 | Priority inheritance for mutexes |
| `TOYOS_ENABLE_MESSAGE_QUEUES` | 1 | Message queue support |
| `TOYOS_MAX_QUEUE_CAPACITY` | 16 | Max messages per queue |

**Example:**
```cpp
#define TOYOS_ENABLE_MESSAGE_QUEUES 0  // Disable to save flash
#define TOYOS_ENABLE_PRIORITY_INHERITANCE 1
```

## Configuration Examples

### Minimal Configuration (Tight Memory)

For Arduino UNO with limited SRAM:

```cpp
#define TOYOS_MAX_TASKS 4
#define TOYOS_DEFAULT_STACK_SIZE 80
#define TOYOS_MEMORY_POOL_SIZE 384
#define TOYOS_ENABLE_MESSAGE_QUEUES 0
#include <toyos.h>

static uint8_t mem_pool[384];

void setup() {
  os_init(mem_pool, sizeof(mem_pool));
  os_create_task(1, task1, 10, 80);
  os_create_task(2, task2, 5, 80);
  os_create_task(0, idle_task, 0, 64);
  os_start();
}
```

**Savings:**
- 2 fewer task slots: ~80 bytes
- Smaller stacks: ~96 bytes
- No message queues: ~200 bytes flash
- **Total: ~376 bytes SRAM saved**

### Performance Configuration

For demanding applications:

```cpp
#define TOYOS_MAX_TASKS 8
#define TOYOS_DEFAULT_STACK_SIZE 128
#define TOYOS_MEMORY_POOL_SIZE 1200
#define TOYOS_TICK_RATE_HZ 1000
#define TOYOS_ENABLE_PRIORITY_INHERITANCE 1
#include <toyos.h>

static uint8_t mem_pool[1200];

void setup() {
  os_init(mem_pool, sizeof(mem_pool));
  // Create up to 8 tasks with larger stacks
  os_create_task(1, high_priority_task, 20, 150);
  os_create_task(2, sensor_task, 15, 128);
  os_create_task(3, control_task, 10, 128);
  os_create_task(4, comm_task, 8, 128);
  os_create_task(5, logging_task, 3, 96);
  os_create_task(0, idle_task, 0, 64);
  os_start();
}
```

### Debug Configuration

For development and testing:

```cpp
#define TOYOS_DEBUG 1
#define TOYOS_AUTO_STACK_CHECK 1
#define TOYOS_ENABLE_STATS 1
#define TOYOS_MIN_STACK_SIZE 64
#include <toyos.h>

void task_monitor(void) {
  while (1) {
    os_check_stack_overflow();
    // Print statistics
    os_delay(1000);
  }
}

void setup() {
  Serial.begin(9600);
  os_init(mem_pool, sizeof(mem_pool));
  os_create_task(99, task_monitor, 1, 96);
  os_create_task(1, app_task, 5, 128);
  os_start();
}
```

### Low Power Configuration

For battery-powered applications:

```cpp
#define TOYOS_TICK_RATE_HZ 100         // 10ms tick
#define TOYOS_TIMER_PRESCALER 256      // Slower timer
#define TOYOS_ENABLE_WATCHDOG 1
#define TOYOS_WATCHDOG_TIMEOUT WDTO_4S
#include <toyos.h>

void task_idle(void) {
  while (1) {
    os_wdt_feed();
    os_enter_idle();  // Enter sleep mode
  }
}

void setup() {
  os_init(mem_pool, sizeof(mem_pool));
  os_wdt_init(TOYOS_WATCHDOG_TIMEOUT);
  os_create_task(0, task_idle, 0, 64);
  os_create_task(1, task_sensor, 10, 96);
  os_start();
}
```

## Using toyos_user_config.h

Create `toyos_user_config.h` in your sketch folder:

```cpp
#ifndef TOYOS_USER_CONFIG_H
#define TOYOS_USER_CONFIG_H

// My custom configuration for Robot Controller v2
#define TOYOS_MAX_TASKS 6
#define TOYOS_DEFAULT_STACK_SIZE 112
#define TOYOS_MEMORY_POOL_SIZE 768
#define TOYOS_TICK_RATE_HZ 1000
#define TOYOS_ENABLE_PRIORITY_INHERITANCE 1
#define TOYOS_ENABLE_WATCHDOG 1
#define TOYOS_WATCHDOG_TIMEOUT WDTO_2S

#endif
```

Then in your sketch:

```cpp
#define TOYOS_USER_CONFIG
#include <toyos.h>

static uint8_t mem_pool[768];

void setup() {
  os_init(mem_pool, sizeof(mem_pool));
  // ... create tasks
  os_start();
}
```

## Memory Sizing Calculator

Use this formula to estimate memory pool size:

```
Memory Pool Size = (Task Stacks) + (Message Queues) + (Overhead)

Task Stacks = MAX_TASKS × Average Stack Size
Message Queues = Number of Queues × (Capacity × 2) + 16
Overhead = ~128 bytes (heap metadata)

Example for 6 tasks, 1 queue:
  Stacks: 6 × 96 = 576
  Queues: 1 × (8 × 2) + 16 = 32
  Overhead: 128
  Total: 736 bytes → round up to 768 or 1024
```

## Validation and Errors

The configuration system validates parameters at compile time:

```cpp
#define TOYOS_MAX_TASKS 50  // Too many!
#include <toyos.h>
// Compile error: "TOYOS_MAX_TASKS must be between 1 and 32"
```

```cpp
#define TOYOS_MIN_STACK_SIZE 32  // Too small!
#include <toyos.h>
// Compile error: "TOYOS_MIN_STACK_SIZE must be at least 48 bytes"
```

## Tips and Best Practices

1. **Start with defaults** - Only customize what you need
2. **Measure first** - Profile your tasks before changing stack sizes
3. **Test thoroughly** - Enable debug mode when developing
4. **Document your config** - Add comments explaining why you changed values
5. **Version control** - Keep `toyos_user_config.h` in your project repo

## Troubleshooting

### Compile Errors

**Error: "TOYOS_MAX_TASKS must be between 1 and 32"**
- Solution: Reduce `TOYOS_MAX_TASKS` to valid range

**Error: "TOYOS_MIN_STACK_SIZE must be at least 48 bytes"**
- Solution: Increase `TOYOS_MIN_STACK_SIZE` to at least 48

### Runtime Issues

**Task crashes or random behavior:**
- Increase stack sizes
- Enable `TOYOS_AUTO_STACK_CHECK` to detect overflows

**Out of memory errors:**
- Increase `TOYOS_MEMORY_POOL_SIZE`
- Reduce number of tasks or stack sizes
- Disable unused features (message queues)

**System freezes:**
- Check watchdog is being fed
- Increase `TOYOS_WATCHDOG_TIMEOUT`

## Files Reference

- `toyos_config.h` - Default configuration (DO NOT EDIT)
- `toyos_user_config.h` - Your custom configuration (create this)
- `toyos.h` - Main header (includes configs)
- `examples/toyos_user_config_example.h` - Configuration template

## Migration Guide

### From ToyOS 2.3 or earlier:

Old code (hardcoded):
```cpp
// No configuration - used library defaults
#include <toyos.h>
```

New code (same behavior):
```cpp
// Still works! Uses defaults from toyos_config.h
#include <toyos.h>
```

New code (customized):
```cpp
// Now you can customize!
#define TOYOS_MAX_TASKS 8
#define TOYOS_DEFAULT_STACK_SIZE 128
#include <toyos.h>
```

## Summary

The ToyOS configuration system provides:
- ✅ Flexible customization without modifying library code
- ✅ Sensible defaults for quick start
- ✅ Compile-time validation of parameters
- ✅ Multiple configuration methods (inline, file-based)
- ✅ Backward compatibility with existing code

For most projects, the defaults work great. Customize only when you need to optimize for specific constraints or requirements.
