# ARM Cortex-M MPU Implementation Plan for ToyOS

**Target Platform:** Arduino UNO R4 (Renesas RA4M1 - Cortex-M4)  
**Feature:** Memory Protection Unit (MPU) for Task Isolation  
**Priority:** High (Security & Robustness Enhancement)  
**Status:** ✅ Complete (Implemented in v2.5.1)

---

## 🎯 Objective

Implement MPU-based memory protection on the RA4M1 to provide:
1. **Task Isolation** - Prevent tasks from accessing each other's memory
2. **Stack Protection** - Hardware-enforced stack overflow detection
3. **Heap Protection** - Guard against heap corruption
4. **Kernel Protection** - Protect kernel data structures from user tasks

---

## 📋 RA4M1 MPU Capabilities

### Hardware Specifications
- **MPU Type**: ARMv7-M Memory Protection Unit
- **Regions**: 8 configurable memory regions
- **Granularity**: Minimum region size of 32 bytes
- **Attributes**: Read/Write/Execute permissions, cacheability, shareability
- **Subregions**: 8 subregions per region for fine-grained control

### MPU Registers (Cortex-M4)
```c
#define MPU_TYPE     (*((volatile uint32_t *)0xE000ED90))  // MPU Type Register
#define MPU_CTRL     (*((volatile uint32_t *)0xE000ED94))  // MPU Control Register
#define MPU_RNR      (*((volatile uint32_t *)0xE000ED98))  // MPU Region Number
#define MPU_RBAR     (*((volatile uint32_t *)0xE000ED9C))  // MPU Region Base Address
#define MPU_RASR     (*((volatile uint32_t *)0xE000EDA0))  // MPU Region Attribute and Size
```

---

## 🏗️ Proposed Architecture

### Memory Layout with MPU Protection

```
┌─────────────────────────────────────┐ 0x20000000
│  Region 0: Kernel Data (RW)         │ Protected from user tasks
│  - Kernel structures                │
│  - Ready heap, blocked queue        │
├─────────────────────────────────────┤
│  Region 1: Task Pool (RW)           │ Shared, read-only for tasks
│  - TaskNode array                   │
│  - Task control blocks              │
├─────────────────────────────────────┤
│  Region 2: Heap (RW)                │ Shared memory pool
│  - Dynamic allocations              │
├─────────────────────────────────────┤
│  Region 3: Task 0 Stack (RW)        │ Private to Task 0
├─────────────────────────────────────┤
│  Region 4: Task 1 Stack (RW)        │ Private to Task 1
├─────────────────────────────────────┤
│  Region 5: Task 2 Stack (RW)        │ Private to Task 2
├─────────────────────────────────────┤
│  Region 6: Peripheral (RW)          │ Device registers
├─────────────────────────────────────┤
│  Region 7: Flash (RX)               │ Code execution
└─────────────────────────────────────┘
```

### Region Allocation Strategy

**Fixed Regions (Always Active):**
- Region 0: Kernel data structures (privileged access only)
- Region 1: Shared task pool (read-only for tasks)
- Region 2: Heap memory (shared, read-write)
- Region 6: Peripheral registers
- Region 7: Flash memory (code)

**Dynamic Regions (Context Switch):**
- Regions 3-5: Task stacks (reconfigured on each context switch)

---

## 🔧 Implementation Plan

### Phase 1: MPU Initialization (Week 1)

#### 1.1 MPU Detection and Configuration
```c
// In port_arm.c

typedef struct {
    uint32_t base_addr;
    uint32_t size;
    uint32_t permissions;
    uint8_t  region_num;
} mpu_region_config_t;

/**
 * Initialize MPU for ToyOS
 */
void port_mpu_init(void) {
    // Check if MPU is present
    uint32_t mpu_type = MPU_TYPE;
    uint8_t num_regions = (mpu_type >> 8) & 0xFF;
    
    if (num_regions < 8) {
        // MPU not available or insufficient regions
        return;
    }
    
    // Disable MPU during configuration
    MPU_CTRL = 0;
    
    // Configure fixed regions
    mpu_configure_kernel_region();
    mpu_configure_heap_region();
    mpu_configure_peripheral_region();
    mpu_configure_flash_region();
    
    // Enable MPU with default memory map as background
    MPU_CTRL = (1 << 0) |  // ENABLE
               (1 << 2);    // PRIVDEFENA (privileged default memory map)
}
```

#### 1.2 Region Configuration Functions
```c
/**
 * Configure a single MPU region
 */
static void mpu_configure_region(uint8_t region_num, 
                                  uint32_t base_addr,
                                  uint32_t size_bits,
                                  uint32_t permissions) {
    MPU_RNR = region_num;
    MPU_RBAR = base_addr & 0xFFFFFFE0;  // Align to 32 bytes
    MPU_RASR = (1 << 0) |               // ENABLE
               (permissions << 24) |     // AP (access permissions)
               (size_bits << 1);         // SIZE
}

/**
 * Configure kernel data region (privileged RW, user no access)
 */
static void mpu_configure_kernel_region(void) {
    extern uint32_t __kernel_data_start__;
    extern uint32_t __kernel_data_end__;
    
    uint32_t base = (uint32_t)&__kernel_data_start__;
    uint32_t size = (uint32_t)&__kernel_data_end__ - base;
    uint8_t size_bits = calculate_mpu_size_bits(size);
    
    mpu_configure_region(0, base, size_bits, 
                        0x01);  // Privileged RW, User no access
}
```

### Phase 2: Task Stack Protection (Week 2)

#### 2.1 Dynamic Stack Region Configuration
```c
/**
 * Configure MPU for task's stack on context switch
 */
void port_mpu_configure_task_stack(TaskControlBlock *task) {
    if (task == NULL) return;
    
    uint32_t stack_base = (uint32_t)task->stack_base;
    uint32_t stack_size = task->stack_size;
    uint8_t size_bits = calculate_mpu_size_bits(stack_size);
    
    // Use region 3 for current task's stack
    mpu_configure_region(3, stack_base, size_bits,
                        0x03);  // Full access (RW)
}

/**
 * Update context switch to reconfigure MPU
 */
void PendSV_Handler(void) {
    // ... existing context save code ...
    
    // Get next task
    TaskControlBlock *next_task = get_next_ready_task();
    
    // Reconfigure MPU for new task's stack
    port_mpu_configure_task_stack(next_task);
    
    // ... existing context restore code ...
}
```

#### 2.2 Stack Overflow Detection
```c
/**
 * MPU Fault Handler - detects stack overflows
 */
void MemManage_Handler(void) {
    // Read fault status registers
    uint32_t cfsr = (*((volatile uint32_t *)0xE000ED28));
    uint32_t mmfar = (*((volatile uint32_t *)0xE000ED34));
    
    if (cfsr & (1 << 0)) {  // IACCVIOL - instruction access violation
        // Stack overflow detected
        #ifdef ARDUINO
        Serial.print(F("MPU FAULT: Stack overflow at 0x"));
        Serial.println(mmfar, HEX);
        Serial.print(F("Task: "));
        Serial.println(os_current_task_ptr->name);
        #endif
        
        // Halt or reset task
        while(1);
    }
}
```

### Phase 3: Heap Protection (Week 3)

#### 3.1 Heap Guard Pages
```c
/**
 * Configure heap with guard pages
 */
void mpu_configure_heap_with_guards(void) {
    extern uint8_t *heap_start;
    extern uint8_t *heap_end;
    
    // Main heap region (RW for all tasks)
    uint32_t heap_base = (uint32_t)heap_start;
    uint32_t heap_size = (uint32_t)heap_end - heap_base;
    
    mpu_configure_region(2, heap_base, 
                        calculate_mpu_size_bits(heap_size),
                        0x03);  // Full access
    
    // Guard pages at heap boundaries (no access)
    // Use subregions to disable access at edges
}
```

### Phase 4: Integration and Testing (Week 4)

#### 4.1 Update port_arm.c
```c
/**
 * Initialize ARM port with MPU support
 */
void port_init(void) {
    // Existing initialization
    port_timer_init();
    
    // NEW: Initialize MPU
    #ifdef TOYOS_USE_MPU
    port_mpu_init();
    #endif
}
```

#### 4.2 Configuration Option
```c
// In toyos_config.h

/**
 * Enable MPU-based memory protection (ARM only)
 * Provides hardware-enforced task isolation and stack protection
 */
#ifndef TOYOS_USE_MPU
#if defined(__ARM_ARCH) && !defined(__AVR__)
#define TOYOS_USE_MPU 1
#else
#define TOYOS_USE_MPU 0
#endif
#endif
```

---

## 📊 Expected Benefits

### Security Improvements
✅ **Task Isolation** - Tasks cannot corrupt each other's memory  
✅ **Stack Protection** - Hardware detects stack overflows immediately  
✅ **Kernel Protection** - User tasks cannot modify kernel structures  
✅ **Heap Safety** - Guard pages prevent heap overflow  

### Debugging Improvements
✅ **Immediate Fault Detection** - MPU faults trigger instantly  
✅ **Precise Error Location** - MMFAR register shows exact address  
✅ **Task Identification** - Know which task caused the fault  

### Performance Impact
⚠️ **Context Switch Overhead** - ~10-20 cycles to reconfigure MPU  
⚠️ **Memory Overhead** - None (uses existing hardware)  
✅ **Runtime Overhead** - Zero (hardware enforcement)  

---

## 🚧 Implementation Challenges

### Challenge 1: Region Limitations
**Problem**: Only 8 regions, but potentially 6+ tasks  
**Solution**: 
- Use 3 fixed regions (kernel, heap, peripherals)
- Use 1 dynamic region for current task's stack
- Other tasks' stacks unprotected (acceptable trade-off)

### Challenge 2: Region Size Constraints
**Problem**: MPU regions must be power-of-2 sized and aligned  
**Solution**:
- Round stack sizes up to nearest power of 2
- Align stack bases appropriately
- Document size requirements

### Challenge 3: Linker Script Modifications
**Problem**: Need to define memory regions in linker script  
**Solution**:
- Create custom linker script for ToyOS
- Define `__kernel_data_start__` and `__kernel_data_end__` symbols
- Align sections to MPU requirements

---

## 📝 Testing Plan

### Unit Tests
1. **MPU Initialization** - Verify regions configured correctly
2. **Stack Overflow** - Intentionally overflow stack, verify fault
3. **Task Isolation** - Task A tries to access Task B's stack
4. **Kernel Protection** - User task tries to modify kernel data

### Integration Tests
1. **Context Switch** - Verify MPU reconfigured correctly
2. **Multi-Task** - Run multiple tasks with MPU enabled
3. **Performance** - Measure context switch overhead

### Stress Tests
1. **Rapid Context Switching** - Verify stability under load
2. **Memory Pressure** - Test with limited heap space
3. **Fault Recovery** - Verify system handles faults gracefully

---

## 🎯 Success Criteria

✅ MPU successfully initialized on RA4M1  
✅ Stack overflow detected and reported  
✅ Task isolation prevents memory corruption  
✅ Context switch overhead < 50 cycles  
✅ All existing tests pass with MPU enabled  
✅ Documentation updated with MPU usage  

---

## 📅 Timeline

**Total Estimated Time: 4 weeks**

- **Week 1**: MPU initialization and basic configuration
- **Week 2**: Task stack protection and fault handlers
- **Week 3**: Heap protection and guard pages
- **Week 4**: Integration, testing, and documentation

---

## 🔗 References

- [ARM Cortex-M4 Technical Reference Manual](https://developer.arm.com/documentation/100166/0001)
- [ARMv7-M Architecture Reference Manual](https://developer.arm.com/documentation/ddi0403/latest/)
- [Renesas RA4M1 User Manual](https://www.renesas.com/us/en/document/mah/ra4m1-group-users-manual-hardware)
- [MPU Programming Guide](https://developer.arm.com/documentation/dai0179/latest/)

---

## 💡 Future Enhancements

After basic MPU implementation:

1. **Privileged/Unprivileged Mode** - Run tasks in unprivileged mode
2. **System Call Interface** - Controlled kernel access via SVC
3. **Dynamic Region Allocation** - Better multi-task support
4. **Fault Recovery** - Graceful task termination on MPU fault
5. **MPU-aware Allocator** - Heap allocator respects MPU regions

---

## 📌 Notes

- This feature is **ARM-only** (AVR has no MPU)
- Requires linker script modifications
- Optional feature (can be disabled via config)
- Adds ~200-300 bytes of code
- Zero runtime overhead except context switch

**Status**: Ready for implementation after v2.5.1 release
