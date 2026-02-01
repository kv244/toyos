# ToyOS MISRA C:2012 Compliance Report

**Date:** February 1, 2026  
**Status:** In Progress (Hardening Phase)  
**Target Architecture:** ARM Cortex-M4 (RA4M1)

## 📊 Summary of Findings

ToyOS has been analyzed against the **MISRA C:2012** guidelines. While the codebase is highly optimized, several patterns common in "efficient" C code violate safety-critical standards.

### 🔴 High Impact Violations

| Rule | Description | Status | Note |
| :--- | :--- | :--- | :--- |
| **Dir 4.12** | Dynamic memory shall not be used | ⚠️ Deviation | ToyOS uses a restricted heap (`os_malloc`). In safety-critical systems, this should be replaced by static allocation. |
| **Rule 15.5** | Single point of exit for functions | 🔴 Violation | Multiple `return` statements found in kernel and database core. |
| **Rule 11.4** | Conversion between pointer to object and integer | 🔴 Violation | Syscall dispatcher casts pointers to `uint32_t` for register passing. |
| **Rule 17.2** | No recursion | ✅ Compliant | No recursion detected in the kernel. |
| **Rule 4.10** | Include guards | ✅ Compliant | All headers use `#ifndef` guards. |

---

## 🛠️ Remediation Plan

### 1. Single Point of Exit (Rule 15.5)
We are refactoring core functions to use a single `return` statement at the end of the function. This improves traceability and ensures cleanup code (like mutex unlocking) is always executed.

**Example: `k_malloc` Refactor**
```cpp
// Before
if (curr->size >= total_size) {
    ...
    return ptr;
}
return NULL;

// After
void* result = NULL;
if (curr->size >= total_size) {
    ...
    result = ptr;
}
return result;
```

### 2. Explicit Type Conversions (Rule 10.4)
Adding explicit casts for all arithmetic operations involving mixed types and pointer arithmetic.

### 3. Pointer-to-Integer Casting (Rule 11.4)
The Syscall interface requires passing pointers through registers (`uint32_t`). This is a necessary "hardware-level" violation. We will document this as a **Formal Deviation** supported by the `os_kernel_syscall` design.

---

## 📝 Formal Deviations

1.  **Syscall Parameters**: Pointers are cast to `uint32_t` to cross the Unprivileged/Privileged boundary via ARM registers R0-R3. This is standard practice for Cortex-M RTOS but violates Rule 11.4.
2.  **Heap Management**: `os_malloc` is provided for flexibility. Users requiring strict MISRA compliance should use static task creation only.

---

## ✅ Progress Log

- [x] Initial Scan of Kernel Core
- [x] Initial Scan of KV Database
- [x] Created Compliance Report
- [x] Refactor: Single Exit Point in `os_kernel_fixed.cpp`
- [x] Refactor: Single Exit Point in `kv_db.cpp`
- [ ] Add explicit boolean checks in controlling expressions (Rule 14.4)
