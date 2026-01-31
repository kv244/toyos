# ToyOS MISRA C Compliance Guide

This document outlines the guidelines for maintaining MISRA compliance in ToyOS. While full certification is not currently claimed, we strive to adhere to key safety-critical rules to ensure robustness.

## 🎯 Target Standards
- **MISRA C:2012** / **MISRA C++:2008**
- **CERT C Secure Coding Standard**

## 🛡️ Key Rules Enforced

### 1. Control Flow (Rule 15.5)
**Rule:** Functions should have a single point of exit (one `return` statement).
**Why:** Improves readability and simplifies static analysis / code verification.

**Example:**
```c
// ❌ Non-Compliant
int check_val(int x) {
    if (x < 0) return -1;
    if (x > 10) return 1;
    return 0;
}

// ✅ Compliant
int check_val(int x) {
    int status;
    if (x < 0) {
        status = -1;
    } else if (x > 10) {
        status = 1;
    } else {
        status = 0;
    }
    return status;
}
```

### 2. Type Conversions (Rule 10.x)
**Rule:** Implicit conversions between types (especially different sizes) are forbidden. Use explicit casts.
**Why:** Prevents data loss and sign extension bugs.

**Example:**
```c
// ❌ Non-Compliant
uint8_t a = 10;
uint16_t b = a + 5; // Implicit promotion

// ✅ Compliant
uint8_t a = 10;
uint16_t b = (uint16_t)a + 5U;
```

### 3. Pointers and Casting (Rule 11.4)
**Rule:** Avoid casting between pointers and integers, or between pointers of different types (type punning).
**Exception:** implementation of memory allocators (os_malloc) requires this, but it must be isolated and documented.

### 4. Recursion (Rule 17.2)
**Rule:** Recursion is strictly forbidden.
**Why:** Stack usage cannot be statically analyzed if recursion exists.

## 🔍 How to Verify
If `cppcheck` is installed, run:
```bash
cppcheck --enable=all --inconclusive --std=c99 --std=c++11 src/
```
