/**
 * ToyOS Storage HAL Implementation
 */

#include "storage_driver.h"

/* Active driver pointer - defaults to NULL */
const storage_driver_t *current_storage_driver = NULL;

void storage_set_driver(const storage_driver_t *driver) {
  current_storage_driver = driver;
}

storage_result_t storage_init(void) {
  if (current_storage_driver && current_storage_driver->init) {
    return current_storage_driver->init();
  }
  return STORAGE_ERR_INIT_FAILED;
}

storage_result_t storage_read(uint32_t addr, void *buf, size_t len) {
  if (current_storage_driver && current_storage_driver->read) {
    return current_storage_driver->read(addr, buf, len);
  }
  return STORAGE_ERR_READ_FAILED;
}

storage_result_t storage_write(uint32_t addr, const void *buf, size_t len) {
  if (current_storage_driver && current_storage_driver->write) {
    return current_storage_driver->write(addr, buf, len);
  }
  return STORAGE_ERR_WRITE_FAILED;
}

storage_result_t storage_erase(uint32_t addr, size_t len) {
  if (current_storage_driver && current_storage_driver->erase) {
    return current_storage_driver->erase(addr, len);
  }
  /* If erase is not supported, it might be EEPROM which doesn't need it.
   * Return OK to allow seamless compaction on EEPROM.
   */
  return STORAGE_OK;
}

uint32_t storage_get_capacity(void) {
  if (current_storage_driver && current_storage_driver->get_capacity) {
    return current_storage_driver->get_capacity();
  }
  return 0;
}
