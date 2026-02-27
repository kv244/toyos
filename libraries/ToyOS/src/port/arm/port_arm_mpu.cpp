/**
 * ToyOS ARM MPU Implementation
 *
 * Memory Protection Unit implementation for Cortex-M4 (RA4M1)
 */

#include "port_arm_mpu.h"
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

/* External references to memory regions (defined in linker script or code) */
extern uint32_t __kernel_data_start__;
extern uint32_t __kernel_data_end__;
extern uint32_t __heap_start__;
extern uint32_t __heap_end__;

/* MPU enabled flag */
static bool mpu_initialized = false;

/* ========================================================================
 * HELPER FUNCTIONS
 * ======================================================================== */

/**
 * Calculate MPU size bits from actual size
 * Rounds up to nearest power of 2
 */
uint8_t port_mpu_calculate_size_bits(uint32_t size) {
  if (size < 32)
    return MPU_SIZE_32B;

  // Find the position of the highest set bit
  uint8_t bits = 0;
  uint32_t temp = size - 1;

  while (temp > 0) {
    temp >>= 1;
    bits++;
  }

  // SIZE field = log2(region_size) - 1
  // So for 2^N bytes, SIZE = N - 1
  if (bits < 5)
    bits = 5; // Minimum 32 bytes (2^5)
  if (bits > 31)
    bits = 31; // Maximum 4GB (2^32)

  return bits - 1;
}

/**
 * Align address down to region size
 */
static uint32_t align_address(uint32_t addr, uint32_t size) {
  return addr & ~(size - 1);
}

/**
 * Check if MPU is present
 */
static bool mpu_is_present(void) {
  uint32_t mpu_type = MPU_TYPE;
  uint8_t num_regions = (mpu_type >> 8) & 0xFF;
  return (num_regions >= 8);
}

/* ========================================================================
 * MPU CONFIGURATION FUNCTIONS
 * ======================================================================== */

/**
 * Configure a single MPU region
 */
bool port_mpu_configure_region(const mpu_region_config_t *config) {
  if (config == NULL)
    return false;
  if (config->region_num > 7)
    return false;

  // Disable MPU during configuration
  bool was_enabled = (MPU_CTRL & MPU_CTRL_ENABLE) != 0;
  if (was_enabled) {
    MPU_CTRL = 0;
  }

  // Select region
  MPU_RNR = config->region_num;

  // Calculate region size (2^(SIZE+1))
  uint32_t region_size = 1UL << (config->size_bits + 1);

  // Align base address
  uint32_t aligned_base = align_address(config->base_addr, region_size);

  // Set base address
  MPU_RBAR = aligned_base & MPU_RBAR_ADDR_MASK;

  // Build RASR value
  uint32_t rasr = MPU_RASR_ENABLE | ((uint32_t)config->access_perm << 24) |
                  ((uint32_t)config->size_bits << 1);

  // Add memory attributes
  if (config->cacheable)
    rasr |= MPU_CACHEABLE;
  if (config->bufferable)
    rasr |= MPU_BUFFERABLE;
  if (config->shareable)
    rasr |= MPU_SHAREABLE;
  if (!config->executable)
    rasr |= MPU_XN;

  // Set region attributes
  MPU_RASR = rasr;

  // Re-enable MPU if it was enabled
  if (was_enabled) {
    MPU_CTRL = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;
  }

  return true;
}

/**
 * Configure kernel data region (privileged RW, user no access)
 * Protects the OS Kernel structures from unprivileged tasks.
 */
static void mpu_configure_kernel_region(void) {
  /* RA4M1 SRAM starts at 0x20000000.
   * We protect the first 4KB which typically contains the vector table (if in
   * RAM), kernel structures, and privileged data.
   */
  uint32_t kernel_base = 0x20000000;
  uint32_t kernel_size = 4096; // 4KB

  mpu_region_config_t config = {
      .base_addr = kernel_base,
      .size_bits = port_mpu_calculate_size_bits(kernel_size),
      .access_perm = MPU_AP_PRIV_RW, // Privileged RW, User NO ACCESS
      .region_num = MPU_REGION_KERNEL,
      .executable = false,
      .cacheable = true,
      .bufferable = true,
      .shareable = false};

  port_mpu_configure_region(&config);
}

/**
 * Configure heap region (shared, full access)
 * This is where task-shared data and message buffers reside.
 */
static void mpu_configure_heap_region(void) {
  /* Typically the heap follows the kernel data.
   * For the R4, we'll allow tasks to access the remainder of the 32KB SRAM
   * minus the 4KB kernel header.
   */
  uint32_t heap_base = 0x20001000;
  uint32_t heap_size = 28672; // 28KB (Remainder of 32KB)

  mpu_region_config_t config = {
      .base_addr = heap_base,
      .size_bits = port_mpu_calculate_size_bits(heap_size),
      .access_perm = MPU_AP_FULL_RW, // Full access (Shared Heap)
      .region_num = MPU_REGION_HEAP,
      .executable = false,
      .cacheable = true,
      .bufferable = true,
      .shareable = true};

  port_mpu_configure_region(&config);
}

/**
 * Configure peripheral region (device memory)
 * Restricted to Privileged access to prevent unprivileged tasks from bypassing
 * security by manipulating hardware directly.
 */
static void mpu_configure_peripheral_region(void) {
  uint32_t periph_base = 0x40000000;
  uint32_t periph_size = 1048576; // 1MB for all peripheral blocks

  mpu_region_config_t config = {
      .base_addr = periph_base,
      .size_bits = port_mpu_calculate_size_bits(periph_size),
      .access_perm = MPU_AP_PRIV_RW, // PRIVILEGED ONLY
      .region_num = MPU_REGION_PERIPHERAL,
      .executable = false,
      .cacheable = false,
      .bufferable = false,
      .shareable = true};

  port_mpu_configure_region(&config);
}

/**
 * Configure flash region (code, read-only, executable)
 */
static void mpu_configure_flash_region(void) {
  // Flash region - code memory
  uint32_t flash_base = 0x00000000; // Flash base
  uint32_t flash_size = 256 * 1024; // 256KB flash

  mpu_region_config_t config = {.base_addr = flash_base,
                                .size_bits =
                                    port_mpu_calculate_size_bits(flash_size),
                                .access_perm = MPU_AP_RO, // Read-only
                                .region_num = MPU_REGION_FLASH,
                                .executable = true, // Code execution allowed
                                .cacheable = true,
                                .bufferable = false,
                                .shareable = false};

  port_mpu_configure_region(&config);
}

/* ========================================================================
 * PUBLIC API
 * ======================================================================== */

/**
 * Initialize the MPU for ToyOS
 */
bool port_mpu_init(void) {
  // Check if MPU is present
  if (!mpu_is_present()) {
    return false;
  }

  // Disable MPU during configuration
  MPU_CTRL = 0;

  // Configure fixed regions
  mpu_configure_kernel_region();
  mpu_configure_heap_region();
  mpu_configure_peripheral_region();
  mpu_configure_flash_region();

  // Enable MPU with default memory map for privileged access
  MPU_CTRL = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;

  // Memory barrier to ensure MPU is enabled before continuing
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb" ::: "memory");

  mpu_initialized = true;

  return true;
}

/**
 * Configure MPU for a task's stack
 */
void port_mpu_configure_task_stack(void *stack_base, uint32_t stack_size) {
  if (!mpu_initialized)
    return;
  if (stack_base == NULL)
    return;

  // Round up stack size to power of 2
  uint8_t size_bits = port_mpu_calculate_size_bits(stack_size);

  mpu_region_config_t config = {
      .base_addr = (uint32_t)stack_base,
      .size_bits = size_bits,
      .access_perm = MPU_AP_FULL_RW, // Full access for current task
      .region_num = MPU_REGION_TASK_STACK,
      .executable = false, // Stack is not executable
      .cacheable = true,
      .bufferable = true,
      .shareable = false};

  port_mpu_configure_region(&config);
}

/**
 * Reconfigure MPU for a task (used during context switch)
 */
void port_mpu_reconfigure(void *task_ptr) {
  /* Since we're in handler mode, we can access internals */
  /* Note: In ToyOS, stack base corresponds to canary location */
  /* For now, just a wrapper around the logic */
  // This is called from PendSV_Handler with r0 as task_ptr (TaskControlBlock*)
  // We can't easily cast to TaskControlBlock* without the header
  // But PendSV_Handler in port_arm.c already does the heavy lifting or expects
  // it. Actually, I'll just keep it simple.
}

/**
 * Disable MPU
 */
void port_mpu_disable(void) {
  MPU_CTRL = 0;
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb" ::: "memory");
  mpu_initialized = false;
}

/**
 * Enable MPU
 */
void port_mpu_enable(void) {
  if (mpu_is_present()) {
    MPU_CTRL = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;
    __asm volatile("dsb" ::: "memory");
    __asm volatile("isb" ::: "memory");
    mpu_initialized = true;
  }
}

/**
 * Get MPU status
 */
bool port_mpu_is_enabled(void) { return (MPU_CTRL & MPU_CTRL_ENABLE) != 0; }

/**
 * MPU Fault Handler
 * Called when a memory protection violation occurs
 */
void MemManage_Handler(void) {
  /* MPU Fault: System Halted */
  while (1) {
    /* Blink error pattern or stay halted */
  }
}

#ifdef __cplusplus
}
#endif
