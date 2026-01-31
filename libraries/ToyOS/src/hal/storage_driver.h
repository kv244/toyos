/**
 * ToyOS Storage Hardware Abstraction Layer (HAL)
 *
 * Defines a generic interface for storage backends (EEPROM, Flash, SD Card).
 * Allows KV Database and other services to be storage-agnostic.
 *
 * Author: [Your Name]
 * Version: 1.0
 * Date: January 2026
 */

#ifndef TOYOS_STORAGE_DRIVER_H
#define TOYOS_STORAGE_DRIVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Storage Operation Result Codes */
typedef enum {
  STORAGE_OK = 0,
  STORAGE_ERR_INVALID_ADDR = -1,
  STORAGE_ERR_WRITE_FAILED = -2,
  STORAGE_ERR_READ_FAILED = -3,
  STORAGE_ERR_INIT_FAILED = -4
} storage_result_t;

/**
 * Storage Driver Interface.
 * Implement this structure for each storage backend.
 */
typedef struct {
  /**
   * Initialize the storage backend.
   * @return STORAGE_OK on success.
   */
  storage_result_t (*init)(void);

  /**
   * Read bytes from storage.
   * @param addr Source address (offset from 0).
   * @param buf Destination buffer.
   * @param len Number of bytes to read.
   * @return STORAGE_OK on success.
   */
  storage_result_t (*read)(uint32_t addr, void *buf, size_t len);

  /**
   * Write bytes to storage.
   * @param addr Destination address (offset from 0).
   * @param buf Source buffer.
   * @param len Number of bytes to write.
   * @return STORAGE_OK on success.
   */
  storage_result_t (*write)(uint32_t addr, const void *buf, size_t len);

  /**
   * Erase a region (optional, if backend requires it).
   * @param addr Start address.
   * @param len Length in bytes.
   * @return STORAGE_OK on success.
   */
  storage_result_t (*erase)(uint32_t addr, size_t len);

  /**
   * Get total capacity of storage in bytes.
   * @return Total size.
   */
  uint32_t (*get_capacity)(void);

  /**
   * Name of the driver (for debugging).
   */
  const char *name;

} storage_driver_t;

/* Global pointer to the active storage driver */
extern const storage_driver_t *current_storage_driver;

/**
 * Set the active storage driver.
 * @param driver Pointer to initialized driver structure.
 */
void storage_set_driver(const storage_driver_t *driver);

/**
 * Helper wrappers for ease of use
 */
storage_result_t storage_init(void);
storage_result_t storage_read(uint32_t addr, void *buf, size_t len);
storage_result_t storage_write(uint32_t addr, const void *buf, size_t len);
storage_result_t storage_erase(uint32_t addr, size_t len);
uint32_t storage_get_capacity(void);

#ifdef __cplusplus
}
#endif

#endif /* TOYOS_STORAGE_DRIVER_H */
