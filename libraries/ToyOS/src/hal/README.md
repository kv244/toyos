# ToyOS Hardware Abstraction Layer (HAL)

This directory contains hardware abstraction drivers for ToyOS RTOS.

## Storage Drivers

The storage HAL provides a unified interface for persistent storage across different platforms.

### Files

- **`storage_driver.h`** - Unified storage interface
- **`storage_driver.c`** - Storage driver dispatcher
- **`storage_avr_eeprom.h/c`** - Native AVR EEPROM driver (direct register access)
- **`storage_arduino_eeprom.h/cpp`** - Arduino EEPROM library wrapper (cross-platform)

### Architecture

```
Application (KV Database)
         ↓
  storage_driver.h (Unified API)
         ↓
  ┌──────┴──────┐
  ↓             ↓
AVR EEPROM   Arduino EEPROM
(Native)     (Library)
```

### Platform Selection

The storage driver automatically selects the appropriate backend:
- **AVR**: Uses native EEPROM driver for optimal performance
- **ARM**: Uses Arduino EEPROM library (Data Flash emulation)

### API

```c
void storage_init(void);
uint8_t storage_read(uint16_t addr);
void storage_write(uint16_t addr, uint8_t data);
void storage_update(uint16_t addr, uint8_t data);
uint16_t storage_length(void);
```

## Adding New HAL Drivers

1. Create `<driver>_driver.h` with unified interface
2. Implement platform-specific backends
3. Add platform detection logic
4. Update documentation
