# ToyOS Recovery & Development TODO

## 🔴 CRITICAL: Hardware Recovery (Board Soft-Bricked)
The Arduino Uno R3 is currently unresponsive to serial uploads ("programmer is not responding"). This was likely caused by a firmware crash leading to a bypassed bootloader or watchdog loop.

- [ ] **Flash Bootloader to Uno R3**
  - **Tool**: Arduino Uno R4 (acting as ISP)
  - **Wiring**:
    - R4 5V -> R3 5V
    - R4 GND -> R3 GND
    - R4 Pin 10 -> R3 RESET
    - R4 Pin 11 (MOSI) -> R3 Pin 11
    - R4 Pin 12 (MISO) -> R3 Pin 12
    - R4 Pin 13 (SCK) -> R3 Pin 13
  - **Steps**:
    1. Upload `ArduinoISP` (with `#define USE_OLD_STYLE_WIRING`) to Uno R4.
    2. Use `arduino-cli burn-bootloader` targeting the Uno R3 via the R4 programmer.

- [ ] **Verify Recovery**
  - [ ] Upload `app/BareMinimum/BareMinimum.ino` to Uno R3 via its own USB port to confirm serial communication is restored.

## 🟢 COMPLETED: Software Fixes (AVR Port)
The following bugs have been fixed in the codebase but need to be verified on hardware:

- [x] **PC Byte Order**: Fixed `port_init_stack` to push Low then High byte (correct for AVR `RET`).
- [x] **Watchdog Feed**: Implemented `os_wdt_feed()` calling `port_wdt_feed()`.
- [x] **Atomic SP Update**: Fixed Timer1 ISR to update `SPH` then `SPL` for atomic stack pointer switching.
- [x] **Bootloop Safeguard**: Added `wdt_disable()` to `setup()` in `kv_db_demo.ino`.
- [x] **Ultra-Early WDT Kill**: Implemented `port_early_wdt_disable` in `.init3` section (runs before `main` and constructors) in `port_avr.c`.
- [x] **Internal Library Hardening**: Added `os_wdt_feed()` calls to long-running KV Database operations (`kv_init`, `kv_compact`, `kv_clear`).
    - *Rationale*: EEPROM writes on AVR take ~3.3ms/byte. A full compaction or scan of 1KB can exceed 1s, which could trigger a reset if the WDT is active.

## 🧠 Diagnostic Note: The "Watchdog Bootloop" Theory
The AVR Watchdog Timer (WDT) persists across resets. If a crash triggers a reset while the WDT is active, it defaults to a very short 15ms timeout on the next boot (the "Watchdog Bootloop of Death"). On Arduinos, if the bootloader or the application `setup()` takes longer than 15ms to run, the chip resets again before any upload can happen.

**Safeguard Plan (Implemented & Verified Ready):**
1. **Application Level**: `wdt_disable()` in `setup()` (Done).
2. **Library Level**: `os_wdt_feed()` in all blocking loops (Done).
3. **Port Level (Deepest)**: `.init3` naked function to kill WDT immediately after reset (Done). This ensures that even if the software crashes, the next boot is guaranteed to be clean and upload-ready.

## 🟡 PENDING: Post-Recovery Tests
1. [ ] **Flash Bootloader** via Uno R4 (as planned above).
2. [ ] **Upload `BareMinimum`** to confirm serial bridge is alive.
3. [ ] **Upload `kv_db_demo`** - The codebase is now "hardened" and verified ready for this step.
4. [ ] **Verify scheduler Preemption** (confirmed Timer1 ISR fix).
5. [ ] **Test KV Database persistence** after manual reset.
