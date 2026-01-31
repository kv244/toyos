/**
 * ToyOS ARM MPU Support
 *
 * Memory Protection Unit configuration for Cortex-M4 (RA4M1)
 * Provides hardware-enforced task isolation and stack protection
 */

#ifndef TOYOS_PORT_ARM_MPU_H
#define TOYOS_PORT_ARM_MPU_H

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * MPU REGISTER DEFINITIONS (Cortex-M4)
 * ======================================================================== */

/* MPU Type Register - Read-only */
#define MPU_TYPE_ADDR 0xE000ED90UL
#define MPU_TYPE (*(volatile uint32_t *)MPU_TYPE_ADDR)

/* MPU Control Register */
#define MPU_CTRL_ADDR 0xE000ED94UL
#define MPU_CTRL (*(volatile uint32_t *)MPU_CTRL_ADDR)
#define MPU_CTRL_ENABLE (1UL << 0) // Enable MPU
#define MPU_CTRL_HFNMIENA                                                      \
  (1UL << 1) // Enable MPU during hard fault, NMI, FAULTMASK
#define MPU_CTRL_PRIVDEFENA                                                    \
  (1UL << 2) // Enable default memory map for privileged access

/* MPU Region Number Register */
#define MPU_RNR_ADDR 0xE000ED98UL
#define MPU_RNR (*(volatile uint32_t *)MPU_RNR_ADDR)

/* MPU Region Base Address Register */
#define MPU_RBAR_ADDR 0xE000ED9CUL
#define MPU_RBAR (*(volatile uint32_t *)MPU_RBAR_ADDR)
#define MPU_RBAR_VALID (1UL << 4) // Region number valid
#define MPU_RBAR_ADDR_MASK                                                     \
  0xFFFFFFE0UL // Address must be aligned to region size

/* MPU Region Attribute and Size Register */
#define MPU_RASR_ADDR 0xE000EDA0UL
#define MPU_RASR (*(volatile uint32_t *)MPU_RASR_ADDR)
#define MPU_RASR_ENABLE (1UL << 0) // Region enable

/* Access Permission (AP) field values */
#define MPU_AP_NONE 0x0            // No access
#define MPU_AP_PRIV_RW 0x1         // Privileged RW, User no access
#define MPU_AP_PRIV_RW_USER_RO 0x2 // Privileged RW, User RO
#define MPU_AP_FULL_RW 0x3         // Full access (RW)
#define MPU_AP_PRIV_RO 0x5         // Privileged RO, User no access
#define MPU_AP_RO 0x6              // Read-only (both)

/* Memory attributes */
#define MPU_TEX_NORMAL 0x1 // Normal memory
#define MPU_SHAREABLE (1UL << 18)
#define MPU_CACHEABLE (1UL << 17)
#define MPU_BUFFERABLE (1UL << 16)

/* Execute Never (XN) bit */
#define MPU_XN (1UL << 28)

/* ========================================================================
 * MPU REGION ALLOCATION
 * ======================================================================== */

/* Fixed regions (always configured) */
#define MPU_REGION_KERNEL 0 // Kernel data structures
#define MPU_REGION_TASK_POOL                                                   \
  1                       // Task control blocks (shared, read-only for tasks)
#define MPU_REGION_HEAP 2 // Shared heap
#define MPU_REGION_PERIPHERAL 6 // Device registers
#define MPU_REGION_FLASH 7      // Code (read-only, executable)

/* Dynamic region (reconfigured on context switch) */
#define MPU_REGION_TASK_STACK 3 // Current task's stack

/* Reserved for future use */
#define MPU_REGION_RESERVED_4 4
#define MPU_REGION_RESERVED_5 5

/* ========================================================================
 * MPU SIZE ENCODING
 * ======================================================================== */

/* Region size encoding (SIZE field in RASR)
 * Size = 2^(SIZE+1) bytes
 * Minimum: 32 bytes (SIZE=4), Maximum: 4GB (SIZE=31)
 */
#define MPU_SIZE_32B 4    // 2^5 = 32 bytes
#define MPU_SIZE_64B 5    // 2^6 = 64 bytes
#define MPU_SIZE_128B 6   // 2^7 = 128 bytes
#define MPU_SIZE_256B 7   // 2^8 = 256 bytes
#define MPU_SIZE_512B 8   // 2^9 = 512 bytes
#define MPU_SIZE_1KB 9    // 2^10 = 1KB
#define MPU_SIZE_2KB 10   // 2^11 = 2KB
#define MPU_SIZE_4KB 11   // 2^12 = 4KB
#define MPU_SIZE_8KB 12   // 2^13 = 8KB
#define MPU_SIZE_16KB 13  // 2^14 = 16KB
#define MPU_SIZE_32KB 14  // 2^15 = 32KB
#define MPU_SIZE_64KB 15  // 2^16 = 64KB
#define MPU_SIZE_128KB 16 // 2^17 = 128KB
#define MPU_SIZE_256KB 17 // 2^18 = 256KB
#define MPU_SIZE_512KB 18 // 2^19 = 512KB
#define MPU_SIZE_1MB 19   // 2^20 = 1MB

/* ========================================================================
 * MPU CONFIGURATION STRUCTURE
 * ======================================================================== */

typedef struct {
  uint32_t base_addr;  // Region base address (must be aligned)
  uint8_t size_bits;   // Size encoding (MPU_SIZE_xxx)
  uint8_t access_perm; // Access permissions (MPU_AP_xxx)
  uint8_t region_num;  // Region number (0-7)
  bool executable;     // false = XN bit set
  bool cacheable;      // Memory caching
  bool bufferable;     // Write buffering
  bool shareable;      // Shareable memory
} mpu_region_config_t;

/* ========================================================================
 * MPU FUNCTION PROTOTYPES
 * ======================================================================== */

/**
 * Initialize the MPU for ToyOS
 * Configures fixed regions and enables MPU
 * @return true if MPU is available and initialized, false otherwise
 */
bool port_mpu_init(void);

/**
 * Configure MPU for a task's stack
 * Called during context switch to protect current task's stack
 * @param stack_base Base address of the stack
 * @param stack_size Size of the stack in bytes
 */
void port_mpu_configure_task_stack(void *stack_base, uint32_t stack_size);

/**
 * Disable MPU (for debugging or special cases)
 */
void port_mpu_disable(void);

/**
 * Enable MPU
 */
void port_mpu_enable(void);

/**
 * Get MPU status
 * @return true if MPU is enabled, false otherwise
 */
bool port_mpu_is_enabled(void);

/**
 * Calculate MPU size bits from actual size
 * Rounds up to nearest power of 2
 * @param size Size in bytes
 * @return MPU size encoding (MPU_SIZE_xxx)
 */
uint8_t port_mpu_calculate_size_bits(uint32_t size);

/**
 * Configure a single MPU region
 * @param config Region configuration
 * @return true if successful, false if invalid configuration
 */
bool port_mpu_configure_region(const mpu_region_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* TOYOS_PORT_ARM_MPU_H */
