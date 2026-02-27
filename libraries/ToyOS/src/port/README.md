# ToyOS Porting Layer

This directory contains the platform abstraction layer for ToyOS RTOS.

## Directory Structure

```
port/
├── cpu_port.h          # Unified CPU abstraction (interrupts, atomics, critical sections)
└── arm/                # ARM Cortex-M implementation
    ├── port_arm.h      # ARM-specific declarations
    └── port_arm.c      # ARM platform implementation
```

## Architecture

### Abstraction Layers

1. **CPU Primitives (`cpu_port.h`)**
   - Low-level CPU operations (interrupts, critical sections, atomics)
   - Inline functions for maximum performance
   - Architecture-specific implementations in same file

2. **Platform Interface (`port.h`)**
   - Unified porting interface
   - Maps `port_*` calls to `cpu_*` primitives
   - Platform detection and configuration

3. **Platform Implementation (`port_arm.c`)**
   - Stack initialization
   - Context switching (cooperative)
   - Timer/tick management (polling-based)
   - Watchdog and power management
   - Platform information

## Supported Platforms

### ARM Cortex-M (RA4M1)
- **Target**: Arduino UNO R4 Minima / WiFi
- **Features**:
  - CMSIS-based implementation
  - Millis-based polling for system tick
  - Cooperative multitasking
  - Data Flash storage support (via Storage HAL)

## Adding a New Platform

1. Create `port/<platform>/` directory
2. Implement platform-specific headers and source files
3. Add architecture detection in `port.h`
4. Optionally extend `cpu_port.h` for new CPU primitives
5. Update build configuration

## Performance Notes

- Critical path functions are inlined for zero overhead
- Context switching uses optimized assembly where available
- Atomic operations use hardware-specific instructions
