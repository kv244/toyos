# ToyOS v2.5.2 Release Summary

**Release Date:** February 1, 2026  
**Branch:** feature/kv-database  
**Status:** ✅ Production Ready (Hobbyist/Educational Use)

---

## 🎉 What's New in v2.5.2

### Major Improvements

#### 1. **KV Database Implementation Rewrite** 💾
- **Log-Structured File System (LFS)**: Implemented a robust architecture for persistent storage.
- **Wear Leveling**: Sector-based circular writing to extend the lifespan of EEPROM/Flash.
- **RAM-Based Indexing**: 
  - **ARM (Uno R4)**: Optimized with O(1) key lookups via RAM-cached keys.
  - **AVR (Uno R3)**: Optimized for low SRAM usage while maintaining O(log N) lookup speed.
- **Stability Fixes**: Resolved all conflicting declarations and missing functions in the KV Database core.

#### 2. **Verified Multi-Platform Build** 🚀
- **Unified HAL**: Re-verified the storage abstraction layer on both AVR (Atmega328P) and ARM (RA4M1).
- **Zero-Error Compilation**: Successfully compiled the full test suite (`kv_db_demo`) for both platforms using `arduino-cli`.
- **Memory Optimization**: Balanced global variable usage to fit within the constrained SRAM of the Arduino Uno R3.

#### 3. **Hierarchical Key Support** 📂
- **Prefix Iteration**: Full support for hierarchical keys (e.g., `config/net/ssid`).
- **`kv_iterate` API**: Efficiently traverse structured data with minimal memory overhead.
- **Test Suite Expansion**: Added new test cases for hierarchy and prefix matching.

---

## 📈 Build Statistics (v2.5.2)

### AVR (Arduino UNO)
```
Flash:  9,314 bytes (28% of 32KB)
SRAM:   1,752 bytes (85% of 2KB)
Status: ✅ PASSED
```

### ARM (Arduino UNO R4 WiFi)
```
Flash:  60,760 bytes (23% of 256KB)
SRAM:   18,648 bytes (56% of 32KB)
Status: ✅ PASSED
```

---

## 🔧 Technical Changes

### KV Database Core
- Rewrote `kv_db.cpp` to match `kv_db.h` and implement the intended LFS architecture.
- Added `hal/storage_driver.h` to the core includes for unified storage access.
- Implemented `kv_compact` for manual space recovery.
- Added CRC32 verification for data integrity.

### RTOS Kernel
- Updated `TOYOS_VERSION_PATCH` to 2.
- Verified system stability during high-frequency DB operations.

---

## 🎯 What Was Fixed

1. ✅ **Conflicting Declarations** - Resolved header/implementation mismatches for `KVRecord` and `KVResult`.
2. ✅ **Missing API Functions** - Implemented `kv_compact`, `kv_iterate`, and `kv_clear`.
3. ✅ **Build Breakage** - Fixed failures that occurred when compiling with standard Arduino tools.
4. ✅ **Memory Footprint** - Optimized the DB core to leave enough room for user applications on AVR.

---

## 🚀 Upgrade Path

### From v2.5.1 to v2.5.2

**Breaking Changes:** None (backward compatible API).  
**API Changes:** Stabilized `kv_compact` and `kv_iterate`.  

### Steps to Upgrade
1. Pull latest code from `feature/kv-database` branch.
2. Rebuild your application.
3. If running on AVR, monitor SRAM usage carefully (now at ~85% with default demo).

---

## 🔍 Testing & Verification

- ✅ **Basic CRUD Test**: Verified write/read/delete/update operations.
- ✅ **Persistence Test**: Data remains valid after simulated reboot.
- ✅ **Concurrency Test**: Verified thread-safe access from multiple RTOS tasks.
- ✅ **Compaction Test**: Verified space reclamation after multiple deletions.
- ✅ **Hierarchy Test**: Verified prefix-based key iteration.

---

**ToyOS v2.5.2 is our most stable release yet, providing a powerful and reliable data persistence layer for both legacy and modern Arduino platforms.**
